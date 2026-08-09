# RacerBot's F1TENTH simulator

This directory is our simulator. It is **not** a fork of F1TENTH Gym.

`setup.sh` clones upstream [`f1tenth/f1tenth_gym`](https://github.com/f1tenth/f1tenth_gym)
at pinned commit `bdaec142` into the gitignored `.sim/` directory and leaves it
completely untouched — it refuses to run at all if that checkout has local
modifications. Everything we change lives here, in tracked files, and layers on
top of upstream from the outside. Nothing here is ever pushed upstream, and a
`rm -rf .sim && ./setup.sh` puts you back on clean upstream code.

```
tools/f1tenth_sim/
├── setup.sh              # clones upstream, pinned; refuses a dirty checkout
├── run_validation.py     # the scenarios and the controllers under test
├── README.md             # this file
└── sim_fidelity/         # our fidelity layer
    ├── calibration.py    # this car's parameters, each tagged with provenance
    ├── grip.py           # friction circle the stock tyre model does not have
    ├── actuation.py      # steering servo and command transport lag
    ├── sensing.py        # pose/odometry/scan error
    ├── plant.py          # wraps env.step(); also does collision detection
    └── bootstrap.py      # shared import setup
```

Why this shape: [`docs/sim-fidelity-audit.md`](../../docs/sim-fidelity-audit.md)
measured how far the stock simulator was from this car. This is the fix for
what that audit found, plus three things it missed.

## Quick start

```bash
./setup.sh                                    # once
python3 run_validation.py --scenario all      # the suite
python3 sim_fidelity/calibration.py           # what we know about the car
```

## The headline: collision detection never worked

**Every `"collision": false` this harness has ever reported was true by
construction, not because the car avoided anything.**

Measured: a car driven dead straight at 4 m/s from the Spielberg start line
travelled **198.9 m through the circuit and its barriers** — the entire 50 s
run — and was never once flagged as having crashed. With the fix, the same run
flags contact at 35.5 m, which is where the first barrier actually is.

Two independent faults, both of which had to be true:

1. `check_ttc_jit` compares each beam against `side_distances`, the distance
   from the LiDAR to the car's own outline. That array is built by
   intersecting each ray with the collision rectangle *from the LiDAR's
   position*, assumed to be inside it. This car's LiDAR is 0.33 m forward of
   `base_link`, and with `collision_body_center_x = wheelbase/2` against the
   ST model's CoG-referenced state the 0.58 m collision box ends up centred on
   `base_link`, spanning ±0.29 m. **0.33 > 0.29: the sensor sits outside its
   own collision body**, no intersection is found, and the helper returns
   `0.0` for all 819 beams.
2. With `side_distances` all zero the test degenerates to "is any beam range
   ≤ 0.005 m", and `ScanSimulator2D.scan()` ends with
   `np.clip(scan, min_range, max_range)` where `min_range` is 0.05 m. No range
   can ever be small enough.

Gym's defaults dodge this by a hair — its 0.275 m LiDAR offset is just inside
a 0.29 m half-length. Moving the sensor to where this car's actually is
silently disabled the check.

`plant.py` replaces it with real geometry: chassis-vs-map through the map's
distance transform, and chassis-vs-chassis with the same separating-axis test
gym ships. Both work all the way round the car, so the rear arc the LiDAR
cannot see is no longer blind (audit F10).

The good news from re-running with working detection: **`gap_solo` and
`pure_solo` record zero contact steps.** Those laps were genuinely clean. It is
`pure_traffic` that was hiding something.

## What else changed

| # | Fix | Audit | Effect |
|---|---|---|---|
| 1 | Working collision detection, real chassis, 360° | F10 | above |
| 2 | Friction circle on steering | F2–F5, R4 | car can no longer corner past `μg`; braking now costs cornering |
| 3 | Proportional steering servo | F6, R3 | removes a permanent 0.08 rad limit cycle |
| 4 | 1081 LiDAR beams | F9, R2 | matches the Hokuyo UST-10LX; measured free |
| 5 | Real steering limits, +0.313/−0.263 rad | F12 | the car's servo travel, from `vesc.yaml` |
| 6 | Real speed ceiling, 5.039 m/s | F13 | the VESC eRPM cap |
| 7 | `a_max = μg` = 6.87 m/s² | F1 | was 9.51 (0.97 g), an asphalt figure |
| 8 | Pose/odometry/scan error | F7–F9, R5 | controllers no longer see ground truth |
| 9 | Command transport lag | R7 | one tick, steering *and* speed |
| 10 | Parameter provenance | F11, R1 | every constant says where it came from |

### 2. A friction circle (`grip.py`)

The stock tyre is linear and unbounded — measured 17.74 m/s² (1.81 g) at full
lock with no plateau — and longitudinal and lateral forces are uncoupled, so
braking mid-corner is free. Both live inside upstream's numba-compiled
derivative, so the envelope is imposed from outside by clamping the steering
angle to what the floor supports at the current speed and braking effort:

```
budget = sqrt((μg)² − a_long²)          # friction circle, longitudinal priority
δ_max  = atan(L · budget / v²)          # steady-state bicycle, inverted
```

This reproduces *the envelope*, not *loss of control*: at the limit the car
understeers along the boundary instead of sliding or spinning. That is still
far better than infinite grip, and `grip_limited_steps` in the results makes
every intervention visible rather than silently flattering the controller.

**`longitudinal_grip_coupling` is the knob to know about.** At 1.0 — the honest
ellipse, and the default — braking at full authority leaves exactly zero
cornering. That is correct physics, but it meets upstream's `pid_accl`, which
answers *any* speed error with a demand that clips straight to `a_max`. So the
plant is at full braking far more often than a real car, and the lateral budget
collapses to 0.0 m/s² in the traffic scenario. Setting it to 0.7 keeps the
minimum budget at 4.9 m/s² and the clamp never binds. We left the default
honest and documented the trade rather than quietly weakening the physics.

### 3. A steering servo that settles (`actuation.py`)

`pid_steer` has no proportional term at all: `sv = sign(error) · sv_max`,
always full slew. At 25 ms and 3.2 rad/s that is a 0.08 rad quantum, so any
target off that grid is overshot forever — measured as a permanent 0.08 rad
(4.58°) limit cycle at 20 Hz, with 0.26 rad commands actually reaching 0.32 rad.

The audit suggested a 5 ms control step, which shrinks the limit cycle 5× and
costs 3.1× in run time. Gym has a better door: `SteerActionType.STEERING_SPEED`
takes a steering velocity directly, so the actuator model can live in our code.
A real hobby servo runs a proportional position loop against a slew ceiling,
which is both more faithful *and* free of the limit cycle, at no run-time cost.

This also settles the open question in `docs/simulator.md` about
`max_steering_rate: 1.0` appearing to help: it was suppressing this artifact.

### 7. Acceleration and braking from one friction coefficient

`a_max` was 9.51 m/s² — 0.97 g, an asphalt-and-warm-rubber number nobody
checked. It is now `μ·g`, so the same coefficient bounds acceleration, braking
and cornering, exactly as a friction circle says it should. At μ = 0.70 that is
6.87 m/s², and stopping from 4 m/s takes 1.16 m rather than 0.84 m.

This chassis is four-wheel drive (confirmed 2026-08-05), so the motor brakes
all four wheels through the transmission and `μg` is the right bound in both
directions. On a rear-drive car it would not be: rear axle load under forward
weight transfer would cap braking near 2.9 m/s², less than half this.

### 8. Sensor error (`sensing.py`)

Controllers were handed `std_state`: exact ground truth. They now get pose from
a particle-filter stand-in (2 ticks lag, 2 cm jitter), speed from a separate
odometry channel, and a scan that is a tick stale with beams occasionally
dropping to `inf`. Metrics still use ground truth, so noise cannot corrupt the
measurement of its own effect.

`plant.expected_scan()` also ray-casts from the *estimated* pose. Map
subtraction used to predict from the same pose and map that generated the
measurement, so `map_subtraction_margin: 0.4` faced nothing at all (F8); it now
faces the dominant real error source.

There is a `freeze_pose()` hook for the stalled-localisation failure that put
this car into a wall on 2026-07-27. It is not yet wired into a scenario,
because the harness re-implements the controller and so does not contain the
`pose_frozen_*` guards being tested — that needs audit R6 first.

## Profiles

`--fidelity` picks how much of this applies.

| Profile | What it is |
|---|---|
| `legacy` | the pre-audit harness, bit for bit |
| `plant` | vehicle fixes, perfect sensing — isolates physics from sensing |
| `car` | everything (default) |

`legacy` reproduces `docs/f1tenth-sim-results.json` exactly for `gap_solo` and
`pure_solo` — same step counts, cross-track and sim times — so the old numbers
stay meaningful and a fidelity change can be bisected against them.

`pure_traffic` no longer matches, and should not: a genuine controller bug in
the overtake logic was fixed (below), so the trajectory legitimately changed.
`legacy` pins the *simulator*, not the controller.

## Results

Spielberg, seed 12345:

| Scenario | `legacy` | `plant` | `car` |
|---|---|---|---|
| `gap_solo` | pass | pass | pass |
| `pure_solo` | pass | pass | pass |
| `pure_traffic` | pass | pass | **fail** |

Across eight seeds, `pure_traffic` is 8/8 on `legacy`, 6/8 on `plant` with
**zero collisions of any kind**, and 0/8 on `car`.

The rear-end this scenario used to hide is gone: `contact_steps` is now 0 for
`plant`, where before the ego drove into the back of the opponent at t = 3.55 s
with the two 0.535 m chassis overlapping by 44 mm and nothing noticing.

What remains is a **sensing-robustness gap, not a safety one**. Under the `car`
profile the ego does not crash into the opponent — it fails to finish, sitting
in the emergency-stop tier or eventually clipping a wall while running on a
pose that lags 50 ms and jitters 2 cm. `pure_solo` shrugs the same sensor error
off entirely (cross-track actually *improved*, 0.1459 → 0.1378), so this is
specific to racing in traffic, where the margins are already thin.

Two cautions about reading this scenario at all:

- **It is chaotic.** Changing only the LiDAR beam count — pure angular
  sampling, no physics — flips individual outcomes. Use `--repeat-seeds`; a
  single run is weak evidence either way, and none of the fixes below were
  justified on its pass rate.
- **The opponent is a fixture, not a driver.** It follows the racing line at
  2 m/s and now brakes for what is in front of it, but it does not steer,
  race, or defend.

## What the overtake investigation found

Two defects, both in `pure_pursuit_node.py` itself rather than in the harness's
copy of it.

### Fixed: the pass was declared complete while the cars were still alongside

`track_progress_gap` wraps everything into `[0, total_length)`, so it cannot
tell "3 m behind" from "all but 3 m ahead". The completion test read:

```python
if gap_ahead > self.total_track_length - self.overtake_clear_margin:
```

which *looks* like "at least `clear_margin` past the opponent" and in fact
means **"at most `clear_margin` past"** — satisfied the instant the ego's nose
edges in front. `pure_pursuit.yaml` and `docs/racing-autonomy.md` both document
the intent as *at least*.

Traced on Spielberg: the pass was declared complete with the ego just 0.20 m
ahead — the cars are 0.535 m long, so still fully overlapped. Steering swung
from +0.028 to +0.219 rad hauling the car back onto the racing line, and it
sideswiped the opponent 0.45 s later.

Fixed with `racing_math.track_lead_distance`, a signed lead that picks the
shorter way round the loop, plus unit tests that pin both the correct behaviour
and the naive-flip trap (comparing the other way makes every *approach* look
like a finished pass). After the fix the same seed completes the pass cleanly
and rejoins the line with the opponent 5.2 m behind.

### Fixed: nothing checked the passing line had room

This is now the dominant failure, and fixing the above exposes more of it,
because the car correctly holds the offset line for longer.

`pick_pass_side` compares the average scan range just outside the cluster on
each side and returns whichever is larger — but it **always returns a side**.
It never asks whether the better side has enough room for a 0.35 m lateral
offset plus half a car. Meanwhile a committed pass sets
`allow_avoidance=not self.overtake_active`, disabling the 1.5 m avoidance tier
for the duration.

So the car commits to a pass, steers 0.35 m toward a wall, has no avoidance
tier left, and only reacts at the 0.4 m emergency stop — then stands still.
Measured at contact across five seeds: forward clearance 0.19–0.34 m, safety
tier `stop`, and the opponent **behind** the ego by 0.5–1.5 m. The scripted
opponent is a constant-2 m/s path follower with no braking and no avoidance
whatsoever, so it simply drives into the stopped ego.

Two consequences worth separating:

- **The ego stopping dead against a wall mid-pass is a real bug.** It is also
  what the "stuck" outcomes are: ~9,190 of 9,600 ticks pinned in the emergency
  tier.
- **`opponent_collision` is partly an artifact of the test opponent.** Any time
  the ego legitimately stops, a brainless opponent rear-ends it. That criterion
  says as much about the scripted car as about the controller.

Fixed with two changes, both strictly conservative — each can only make the car
do less or go slower, never commit to something new:

1. **A commit-time room check.** `racing_math.overtake_side_has_room` measures
   the perpendicular wall distance on the chosen side and refuses the pass
   below `overtake_min_side_clearance` (0.70 m = the 0.35 m offset + half a
   0.31 m car + 0.15 margin). Declining to pass is always available.
2. **A speed cap during a committed pass.** Steering still stays with the pass
   — swerving away mid-overtake is what the suppression exists to prevent — but
   the speed no longer gets a free ride to the 0.4 m emergency stop.

The cap **must** be computed from the *mapped* track edge, never the raw scan.
Capping on the raw scan was tried and is a straightforward regression: the
nearest thing ahead during a pass is by definition the car being passed, so the
ego throttles below the opponent's speed and the pass becomes mathematically
impossible. Every seed deadlocked. `_static_closest_in_cone` uses the map ray
cast, and returns `None` when map subtraction is unavailable, which the caller
reads as "cannot tell walls from traffic, so do not cap".

### And the scripted opponent needed brakes

Not a controller bug, but it was corrupting the measurement. The test opponent
was a blind constant-2 m/s path follower, so it drove into the ego whenever the
ego legitimately stopped — making `opponent_collision` a statement about the
fixture. It now declines to rear-end a stationary object. That change alone
took `legacy` from 3/8 to 8/8 and removed every artifact collision.

**Floor-test the overtake before relying on it.** These are simulator results
against a fixture opponent, on a car whose mass and centre of gravity are still
unmeasured.

## Compute

Not a constraint. The audit benchmarked every fix as affordable and the suite
still runs in well under a minute per profile on the Jetson; raising the beam
count to 1081 measured marginally *faster* than 819. See
[the audit's compute section](../../docs/sim-fidelity-audit.md#does-this-cost-too-much-compute).

## Calibration and its provenance

`python3 sim_fidelity/calibration.py` prints every constant with where it came
from, least trustworthy first:

- `MEASURED` — measured on this car. *Currently none.*
- `DERIVED` — computed from this car's committed configs (`vesc.yaml` etc.)
- `SPEC` — datasheet for a part actually fitted
- `ESTIMATED` — physics-based estimate, not this car
- `STOCK` — F1TENTH Gym default, describing a different vehicle

**6 of 32 values are still `STOCK`**: mass, yaw inertia, CoG height, CoG
position, and both cornering stiffnesses. Each is one line to fix once
measured; see audit [R1](../../docs/sim-fidelity-audit.md#r1-measure-the-car-highest-value-single-action).

Two deliberate non-changes worth knowing about, both cases where the obvious
"fix" would have made things worse:

- **`cog_front_fraction` stays at 0.5.** The stock F1TENTH ratio is 0.4808, and
  the audit rightly called 0.5 an assumption — but 0.4808 belongs to a car with
  a 0.3302 m wheelbase, so it is no more a measurement of *this* vehicle. It
  also matters: sweeping it 0.46 → 0.54 flips the traffic scenario. Swapping one
  unmeasured number for another would move every result without improving
  fidelity. Weigh the axles.
- **`v_max` is not written into gym's parameters.** Gym derives `pid_accl`'s
  proportional gain from it as `10·a_max/v_max`, so setting the car's real
  5.039 m/s ceiling there quietly *quadruples* the plant's speed-loop gain —
  which alone deadlocks the traffic scenario. The ceiling is enforced by
  clamping the speed command instead.

## Careful with

- **`legacy` must keep reproducing the baseline.** It is the only guard against
  a fidelity change being mistaken for a controller regression. Check it after
  touching anything in `sim_fidelity/`.
- **Determinism.** Every sensor RNG is seeded from the scenario seed. Do not
  introduce unseeded randomness; reproducibility is what the harness is for.
- **`run_validation.py` re-implements the node's command stage.** Audit R6 is
  still open, and it remains the highest-probability way for the harness to
  pass while the real node does something different.
- **None of this makes the simulator a safety argument.** Passing here is
  evidence against algorithm and regression errors, nothing more. The
  wheels-off-the-ground → low speed → open space order in
  [`docs/writing-your-own-node.md`](../../docs/writing-your-own-node.md#testing-before-its-on-wheels)
  and the mandatory LB deadman still apply to every physical run.
