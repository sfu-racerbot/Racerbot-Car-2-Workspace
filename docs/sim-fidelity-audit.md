# Simulator fidelity audit

> **Who this is for:** anyone about to trust a simulator result, or wondering why the car behaves differently than the sim said it would.
> **Read first:** [simulator.md](simulator.md) and [ros-simulator.md](ros-simulator.md).
> **What's in it:** measured divergences between simulator and physical car — grip, braking, steering, localization — and what they mean for tuning.

How closely does `tools/f1tenth_sim/` match this physical car, where does it
diverge, and what would it cost to close the gap?

This audit was done by reading the harness and the pinned F1TENTH Gym dynamics,
comparing every vehicle parameter against this car's own configs
(`vesc.yaml`, `sensors.yaml`, `pure_pursuit.yaml`), and **running probes against
the installed `.sim/` checkout to measure what the simulated car actually does**.
Every number below marked *measured* came from an executed probe on this Jetson,
not from reading code.

Read [docs/simulator.md](simulator.md) first — it describes what the harness is
and how to run it. This document is about how much to trust it.

> **Status: acted on.** Most of the recommendations below are implemented in
> `tools/f1tenth_sim/sim_fidelity/`; see
> [tools/f1tenth_sim/README.md](../tools/f1tenth_sim/README.md) for what was
> done, what was deliberately left alone, and what the fixes then revealed.
>
> Two corrections to this document, both found while implementing it:
>
> - **F10 badly understates the problem.** The collision check is not a 5 mm
>   proximity test with a rear blind spot — it *cannot fire at all* in this
>   harness's geometry. `side_distances` is identically zero on every beam,
>   because the LiDAR at +0.33 m sits outside the ±0.29 m collision box it is
>   supposed to be measured from, and `range_min` clipping at 0.05 m then makes
>   the degenerate test unreachable. Measured: 198.9 m driven straight through
>   the circuit and its barriers, never flagged.
> - **R3's proposed fix is superseded.** A 5 ms control step shrinks the
>   steering limit cycle 5× and costs 3.1× in run time.
>   `SteerActionType.STEERING_SPEED` removes the limit cycle entirely, is more
>   faithful to a real servo, and is free.

## Verdict

**The harness is a good algorithm-and-regression check and a poor vehicle
model.** Its geometry and timing match the car closely. Its grip, braking,
steering actuator, and localization do not — and every one of those diverges in
the *optimistic* direction.

The most important single sentence: **no dynamics parameter in the harness has
ever been measured on this car.** Wheelbase and body size are correct because
they were taken from a spec sheet. Mass, yaw inertia, centre-of-gravity
position, cornering stiffness, and acceleration/braking authority are all
inherited unchanged from F1TENTH Gym's stock parameter set, which describes a
different vehicle.

**Compute is not the constraint.** Every fidelity fix worth making runs
comfortably on this Jetson — see [Does this cost too much compute?](#does-this-cost-too-much-compute)
Even the heaviest tested configuration runs 3.2× faster than real time in
391 MiB. What limits fidelity here is *knowledge of the car*, not CPU.

## What the harness gets right

Worth stating plainly, because the list is longer than you might expect:

- **Wheelbase 0.324 m** matches `vesc.yaml` exactly.
- **Body 0.31 × 0.58 m** is deliberately padded beyond the real chassis, with
  the collision centre at wheelbase/2 — conservative in the right direction.
- **LiDAR mounting** — ±135° FOV and the +0.33 m forward offset match the real
  `laser` frame.
- **Control timing** — 40 Hz control with a 5 ms RK4 integration substep matches
  `control_rate_hz: 40.0`.
- **Longitudinal load transfer is modelled correctly.** The ST equations use
  `glr = g·lr − a·h` for the front axle load term and `glf = g·lf + a·h` for the
  rear. The naming is confusing (`glr` feeds the *front* cornering term) but the
  physics is right: braking loads the front axle and increases front grip.
- **Steering actuation has one tick (25 ms) of delay**, which is at least some
  acknowledgement of actuation lag.
- **It is seeded and deterministic**, which is exactly correct for regression
  testing and is the property most of the fixes below must not break.

One modelling note that is a legitimate choice rather than a defect: the ST
model **switches to a kinematic bicycle below 0.5 m/s**, because the dynamic
equations divide by velocity. Be aware that `pure_pursuit.yaml`'s
`min_speed: 0.5` sits exactly on that boundary, so the car's slowest commanded
speed is right at a discontinuity in the model's derivative.

## Findings

Ordered by how much damage each could do, not by how interesting it is.

### Tier 1 — optimistic physics, safety-relevant

#### F1. Braking and acceleration authority is unmeasured and set to ~1 g

*Measured:* peak acceleration **9.51 m/s²**, peak deceleration **9.51 m/s²**,
**0.840 m** to stop from 4 m/s, 0 → 4 m/s in 1.00 s.

Both numbers are `a_max`, inherited from the gym's stock parameters. Nothing
about this car was consulted. 9.51 m/s² is 0.97 g — for a 3.74 kg car that
demands a friction coefficient near 1.0 at every tyre simultaneously, which is
an asphalt-and-warm-rubber figure, not a smooth-indoor-floor figure.

Stopping distance scales as `v²/2a`, so the error is not subtle:

| assumed decel | stop distance from 4 m/s |
|---:|---:|
| 9.51 m/s² *(what the sim assumes)* | 0.84 m |
| 6.00 m/s² | 1.33 m |
| 4.00 m/s² | 2.00 m |
| 3.00 m/s² | 2.67 m |

**Why it matters here:** `emergency_stop_distance: 0.4` and
`avoidance_trigger_distance: 1.5` were both validated against the 0.84 m figure.
The layered design saves it in the common case — avoidance drops the car to
`avoidance_speed: 1.0` at 1.5 m, and stopping from 1 m/s needs only 0.05 m even
at a pessimistic decel. But the margin chain assumes the avoidance layer fires.
When it doesn't — an object appearing suddenly, or map subtraction rejecting a
real obstacle — the car arrives at 0.4 m still doing 4 m/s, and the sim says it
needs 0.84 m while the real car may need two or three times that.

**Fix:** measure it. See [R1](#r1-measure-the-car-highest-value-single-action).
This is one parameter and it fixes acceleration and braking together.

#### F2. The tyre model cannot saturate — there is no grip limit at all

*Measured*, sustained lateral acceleration at 4 m/s:

| steering (rad) | 0.05 | 0.10 | 0.20 | 0.26 | 0.35 | 0.419 |
|---|---|---|---|---|---|---|
| a_lat (m/s²) | 1.82 | 5.32 | 8.82 | 12.31 | 15.80 | **17.74** |
| (g) | 0.19 | 0.54 | 0.90 | 1.25 | 1.61 | **1.81** |

It climbs monotonically straight past μ·g = 10.29 m/s² and never plateaus.

The ST model uses a **purely linear tyre**: lateral force is
`F_y = C_α · α`, proportional to slip angle with no upper bound. A real tyre
follows a Pacejka-style curve — force rises roughly linearly, peaks near
α ≈ 5–8°, then *falls off* — and can never exceed `μ·F_z`.

**Consequence: the simulated car physically cannot slide, understeer, oversteer,
or spin.** Those are the failure modes that put a real racing car into a wall,
and this harness is structurally incapable of producing any of them. A "no
collision" result says nothing about whether the real car would have held the
corner.

#### F3. `mu` is not a friction limit — you cannot model a slippery floor with it

*Measured*, sustained a_lat at full steer and 4 m/s while varying `mu`:

| mu | 0.30 | 0.60 | 1.0489 | 2.00 |
|---|---|---|---|---|
| μ·g (the "limit" it implies) | 2.94 | 5.89 | 10.29 | 19.62 |
| actual a_lat (m/s²) | **10.46** | 11.57 | 12.31 | 13.11 |

Reducing grip by a factor of 3.5 changed achievable cornering by **15%**, and at
mu = 0.30 the car still pulled 3.6× the lateral acceleration that μ·g permits.

The mechanism: in the yaw-moment equation `mu` appears as an overall multiplier,
so it cancels completely at steady state (`ψ̈ = 0`). It survives only in the
slip-angle equation, where the `− ψ̇` term is not multiplied by it. So **`mu`
sets how fast the car settles into a corner, not how hard it can corner.**

This matters because "just lower `mu` to be conservative" is the obvious
intuition and it is wrong. It buys almost nothing.

#### F4. No friction ellipse — braking and cornering are independent

A real tyre shares one contact patch between longitudinal and lateral force:

```
    F_x² + F_y²  ≤  (μ · F_z)²
```

Spend grip on braking and less remains for cornering. This is why trail-braking
into a corner is where a real car loses the rear, and it is the single most
likely way this car crashes at speed.

The ST model has no such coupling. `ACCL` enters only through load transfer;
lateral force is computed from slip angle alone. **In simulation, braking hard
mid-corner is free.** The harness cannot see the most probable real
loss-of-control mechanism.

#### F5. The linear tyre model is being used outside its valid range

*Measured:* peak body slip angle **5.40°** during the 0.26 rad cornering probe.

`F_y = C_α · α` is a small-angle approximation, valid to roughly α ≈ 3–5° before
real tyre force starts falling away from the line. The sim is already operating
at or beyond that boundary during ordinary cornering — so even setting aside the
missing saturation, the model is extrapolating past where its own linearisation
holds.

### Tier 2 — artifacts that actively mislead tuning

#### F6. The steering actuator is bang-bang and never settles

*Measured*, commanding a constant 0.15 rad and reading achieved angle each tick:

```
+0.000  +0.080  +0.160  +0.080  +0.160  +0.080  +0.160  +0.080  ...
```

A permanent **0.08 rad (4.58°) peak-to-peak limit cycle at 20 Hz**, forever.

`pid_steer` is not a PID. It outputs `sign(error) × sv_max` — always full slew
rate, 3.2 rad/s — with no proportional term. One 25 ms tick moves 0.08 rad,
which overshoots any target not on the 0.08 rad grid, so it oscillates
indefinitely.

Two consequences, both bad:

1. **Commanding 0.26 rad actually reaches 0.32 rad**, overshooting the
   controller's own `max_steering_angle: 0.26` clamp. The lateral-acceleration
   figures in F2 are inflated for this reason.
2. **It explains an open puzzle in `docs/simulator.md`.** That document records
   that removing `max_steering_rate` *cost* 11 s on Spielberg, and speculates it
   was "presumably by letting the steering chatter more." That is exactly right,
   and the chatter is a simulator artifact. `max_steering_rate: 1.0` is
   therefore partly tuned against a bug rather than against vehicle physics, and
   its value on the real car is unvalidated.

Any conclusion this harness has produced about steering smoothness should be
treated as suspect until this is fixed.

#### F7. Localization is perfect — the pose-failure guards are never exercised

The controllers are handed `ego["std_state"]`: exact simulator ground truth. No
particle-filter lag, no jitter, no divergence, no freeze.

On the real car `pure_pursuit_node` consumes `/pf/viz/inferred_pose`, which lags,
jitters, and can stop updating. The node has explicit defences for this —
`pose_timeout_sec`, `max_cross_track_error`, and the `pose_frozen_*` trio added
in commit `dbdbac3` specifically because frozen localization was observed on the
real car. **None of that code is reachable in simulation.** The harness cannot
regress the defences written for the failure mode that actually happened.

#### F8. Map subtraction is perfect by construction

`opponent_detection_mode: map` — the default — works by ray-casting the known map
and flagging beams that come back `map_subtraction_margin: 0.4` shorter than
predicted.

In the harness, `static_expected_scan()` calls the *same* `scan_sims` object
against the *same* map used to generate the measured scan. The prediction and
the measurement are drawn from one identical source, so subtraction is exact up
to 1 cm of Gaussian noise.

On the real car the map comes from SLAM and carries drift and discretisation
error, and the pose used to place the ray cast carries its own error. The
0.4 m margin is doing real work there and none at all here. The measured
robustness of opponent detection is therefore not evidence about the real car.

#### F9. LiDAR is zero-latency, artifact-free, and the wrong beam count

The model is a 2D ray cast plus σ = 1 cm Gaussian noise. Missing: latency
(scan and pose arrive instantly and perfectly synchronised), dropouts, `inf`/
`NaN` returns, mixed pixels at depth discontinuities, reflectivity and incidence
effects, and multi-echo. The real Hokuyo produces all of these, and the node has
`max_range` clipping and inf/NaN handling written specifically to survive them —
untested code paths.

The beam count is also simply wrong: the harness uses **819 beams**, while the
car's **Hokuyo UST-10LX** (`docs/hardware-reference.md`) gives **1081** at 0.25°
over 270°, and `sensors.yaml` requests full resolution (`cluster: 1, skip: 0`).

The 1081 figure is the UST-10LX datasheet value, not a reading taken from this
unit — confirm it with `ros2 topic echo /scan --once --field angle_increment`
(and the `ranges` length) before hard-coding it.

#### F10. Collision detection is a 5 mm proximity test, blind to the rear 90°

Despite the name, `check_ttc_jit` performs no time-to-collision calculation —
that code is commented out. What runs is:

```python
ttc = scan - side_distances
in_collision = np.any(ttc <= 0.005)
```

A geometric test: is any beam within 5 mm of the car's body outline. That is
reasonable, but it inherits the LiDAR's field of view. **The scan spans ±135°
from a sensor mounted 0.33 m forward of `base_link`, leaving a 90° wedge behind
the car with no beams — contacts there cannot be detected.** In the two-car
scenario, the opponent being struck from behind would not register as a
collision on the opponent's flag.

### Tier 3 — unmeasured or mismatched parameters

#### F11. Mass, yaw inertia, and CoG are stock values, and `lf = lr` is an assumption

| parameter | harness value | status |
|---|---|---|
| `m` | 3.74 kg | gym stock — never weighed |
| `I` (yaw inertia) | 0.04712 kg·m² | gym stock — never measured |
| `h` (CoG height) | 0.074 m | gym stock |
| `C_Sf` / `C_Sr` | 4.718 / 5.4562 | gym stock, original F1TENTH tyres |
| `lf` / `lr` | 0.162 / 0.162 | **assumed** — CoG at wheelbase midpoint |

`docs/hardware-reference.md` records no mass figure at all.

`lf = lr` is the most consequential assumption, because the lf/lr split *is* the
understeer/oversteer balance. This car carries a Jetson Orin Nano, a Hokuyo, a
VESC and a battery; the odds that they sum to a CoG exactly at the wheelbase
midpoint are poor.

Yaw inertia is worth a sanity check. A uniform slab of the car's footprint gives
`I = m(L² + W²)/12 = 0.135 kg·m²` — **2.9× the stock value**. The stock figure
implies a radius of gyration of 0.112 m and a dynamic index
`k²/(lf·lr) = 0.48`, against roughly 0.8–1.0 for typical vehicles. A low
dynamic index means the car rotates more eagerly than it should.

An RC car genuinely does concentrate mass centrally (battery and motor low and
inboard), so a below-typical index is defensible — but 0.48 is low enough to
deserve measurement rather than assumption, and it directly scales yaw response.

#### F12. Steering limits are symmetric in sim, asymmetric on the car

From `vesc.yaml` (`servo_min: 0.15`, `servo_max: 0.85`, gain −1.2135, offset
0.5304):

| | left | right |
|---|---:|---:|
| real car | **+0.3135 rad** (+17.96°) | **−0.2634 rad** (−15.09°) |
| sim `s_min`/`s_max` | +0.4189 rad | −0.4189 rad |

The real car steers noticeably harder one way than the other. The controller's
symmetric `max_steering_angle: 0.26` clamp keeps both within reach, so this is
not currently causing wrong behaviour — but the sim can never reproduce the
asymmetry, and F6's overshoot to 0.32 rad exceeds the real right-hand limit.

#### F13. No rolling resistance or drivetrain drag, and `v_max` is 4× the car's real ceiling

`V̇ = ACCL` and nothing else — there is no resistive term anywhere in the model.
Commanded to hold speed, the simulated car coasts frictionlessly forever.

Real magnitudes at 4 m/s, for calibration:

| source | decel |
|---|---:|
| aerodynamic drag | 0.07 m/s² — genuinely negligible, ignore it |
| rolling resistance (Crr 0.010–0.020) | 0.10–0.20 m/s² |
| drivetrain drag (geared RC, motor braking) | larger, and worth measuring |

Aero is not worth modelling at these speeds. Rolling and drivetrain drag are
small but systematic, and they bias every coast-down and every speed reduction.

Separately: `vesc.yaml`'s `speed_max: 23250` eRPM over gain 4614 caps the real
car at **5.04 m/s**, while the sim allows `v_max: 20.0`. It does not bind at the
current 4.0 m/s profile, but it does mean the sim will happily validate speeds
the hardware cannot reach.

One more real effect that is absent: **tyre relaxation length**. Real tyres need
0.1–0.3 m of travel to build lateral force, which at 4 m/s is 25–75 ms of lag —
comparable to or larger than the 25 ms control period. Steering response in
simulation is instantaneous by comparison.

### Tier 4 — divergence between the harness and the node

#### F14. The harness re-implements the node's command stage

`CommandShaper` in `run_validation.py` is documented as a "deterministic
equivalent of pure_pursuit_node's final command stage." It calls the shared
`racing_math` primitives, which is good, but the *sequencing* — ceiling order,
ramp basis selection, slew application — is a hand-maintained copy.
`docs/simulator.md` already warns "if you change one, change both" about two
constants.

This is not a physics gap but it may be the highest-probability failure: the
harness can pass while the node does something different, and nothing detects
the drift. Fidelity to the real vehicle is worthless if the code under test
isn't the code that ships.

## Does this cost too much compute?

**No.** Benchmarked on this Jetson Orin Nano Super (6 cores, 8 GB shared).
Metric is real-time factor — simulated seconds per wall-clock second, so 1.0×
is real time and higher is better.

| Configuration | RTF | vs baseline | Peak RSS |
|---|---:|---:|---:|
| **Baseline** — ST, 819 beams, 25 ms control, 5 ms integrator | **29.9×** | — | 363 MiB |
| + 1081 beams (F9 fix) | 33.6× | **free** | 361 MiB |
| + 1 ms integrator | 17.3× | 1.7× slower | — |
| + 5 ms control step (F6 fix) | 9.7× | 3.1× slower | — |
| **All cheap fixes** — 1081 beams + 5 ms control | **8.1×** | 3.7× slower | 361 MiB |
| 2 agents, baseline | 15.8× | 1.9× slower | — |
| **2 agents + all cheap fixes** *(worst case)* | **3.2×** | 9.5× slower | 391 MiB |
| Multi-body (MB) model | — | **crashes** | — |

Three conclusions:

1. **Raising the beam count to 1081 is free.** It measured marginally *faster*
   than 819 — run-to-run variance. Ray casting is not the bottleneck, so there
   is no reason to keep the wrong number.
2. **Even the worst case stays well ahead of real time.** Two agents with every
   fix applied still runs 3.2× faster than the car does. The full validation
   suite currently takes ~55 s wall; with every cheap fix applied it lands
   around 3–4 minutes. That is a perfectly reasonable price for a suite you run
   before putting a car on the floor.
3. **Memory is a non-issue.** Every configuration sat between 361 and 391 MiB.
   Beam count and control rate barely move it — the arrays involved are a few
   kilobytes. This holds even with only ~2 GB free.

Note the baseline discrepancy worth understanding: this benchmark measures 29.9×
with a constant action, while the checked-in results in
`docs/f1tenth-sim-results.json` work out to ~20× (44,057 steps, 55.3 s wall).
The difference is the controller math itself, which costs roughly a third of
each step. Budget accordingly.

### The multi-body model is not a viable route

An earlier reading of this suggested switching to `DynamicModel.MB` for a
saturating tyre. **That recommendation was wrong, and the benchmark caught it:**
MB crashes immediately with `cannot convert float NaN to integer`.

`VehicleParameters` has 89 fields. The ST model uses 18. `F1TENTH_VEHICLE_PARAMETERS`
leaves the other **69 as `NaN`** — sprung and unsprung masses, suspension
stiffness and damping, roll-axis heights, wheel inertias, and a complete Pacejka
Magic Formula coefficient set (`tire_p_cx1` … `tire_r_vy6`).

Most of those cannot be measured without a tyre test rig and a suspension
kinematics-and-compliance rig. Populating them by guesswork would produce a
model that is more elaborate and no more truthful. **Do not go down this path.**
[R4](#r4-add-a-grip-envelope-to-the-existing-model) is the cheaper and more
honest answer to the same problem.

## Recommended work, in priority order

### R1. Measure the car (highest-value single action)

Nothing else on this list changes as much per hour spent. All of it is bench or
floor work, no code.

- **Mass** — kitchen scale, fully assembled with battery. Sets `m`.
- **CoG position** — weigh front and rear axles separately;
  `lf = L · (W_rear / W_total)`. Replaces the `lf = lr` assumption and fixes the
  understeer/oversteer balance.
- **Acceleration and braking authority** — the big one. You already have
  `race_diagnostics` recording rosbags ([docs/run-diagnostics.md](run-diagnostics.md)).
  Do a straight-line full-throttle run and a hard-brake run in open space with
  LB held, then differentiate `/odom` velocity. Sets `a_max`, which corrects F1
  in both directions at once.
- **Coast-down** — from 4 m/s, cut command and log `/odom`. The decay rate gives
  combined rolling and drivetrain drag directly (F13).
- **Yaw inertia** *(optional, more effort)* — a bifilar pendulum gives `I` from
  the oscillation period. Worth it only if yaw response still mismatches after
  the rest is set.

Do this on the floor, at low speed, with the LB deadman held, per
[docs/writing-your-own-node.md](writing-your-own-node.md#testing-before-its-on-wheels).

### R2. Fix the beam count

819 → 1081. Measured free. One constant in `run_validation.py`.

### R3. Fix the steering chatter

Set the env `timestep` to 5 ms and step it 5× per 40 Hz control decision. The
bang-bang granularity drops from 0.08 rad to 0.016 rad — a 5× reduction in the
limit cycle — while the controller keeps deciding at 40 Hz exactly as the node
does. Costs 3.1×, landing at 9.7× real time.

Then **re-examine `max_steering_rate: 1.0`**, whose apparent benefit (F6) may
have been an artifact of the chatter it was suppressing.

### R4. Add a grip envelope to the existing model

Since the plant cannot saturate (F2) and MB is unavailable, impose the envelope
from outside. Two complementary pieces:

- **A guardrail.** Log demanded lateral acceleration every tick and **fail the
  scenario** if it exceeds a measured real grip limit. This converts missing
  physics into an explicit, visible assertion rather than a silent optimism.
- **A grip-limited plant.** Before passing steering to `env.step()`, clamp it to
  what μ permits at the current speed: `|δ| ≤ atan(L · μ · g / v²)`. Roughly
  fifteen lines.

Be honest about what this buys: it reproduces *the envelope* — the car can no
longer corner harder than physics allows — but not *loss-of-control behaviour*.
It will understeer at the limit rather than slide. That is still a large
improvement over an infinite-grip car, and it makes the limit visible in results.

### R5. Degrade the sensing to match reality

Cheap to add, since the harness already sits between the sim and the controller
and can simply perturb what it hands over.

- Delay pose by 1–3 ticks and add jitter representative of the particle filter.
- **Add a scenario that freezes the pose mid-lap**, so the `pose_frozen_*` guards
  from commit `dbdbac3` are actually exercised (F7).
- Delay the scan one tick; inject `inf`/`NaN` and dropouts to exercise the
  `max_range` handling (F9).
- Perturb the map used for `static_expected_scan` relative to the ray-cast map,
  so `map_subtraction_margin` faces a non-zero error (F8).

### R6. Test the node's real code path

Retire `CommandShaper` (F14) in favour of importing the node's own command stage,
or run the node in-process against a fake `rclpy`. Removes the silent-drift
class of failure entirely.

### R7. Model actuation lag on the longitudinal channel

`steer_delay_steps: 1` delays steering only; speed commands take effect
instantly. The real path is ROS → USB → VESC firmware → motor, plus drivetrain
lash. Measure it during R1's step-response runs and add matching delay.

## What simulation still cannot tell you

Unchanged from [docs/simulator.md](simulator.md), and worth repeating because
none of the above alters it. Even with every fix applied, a passing run is
evidence against algorithm and regression errors — not proof that tyre grip,
servo calibration, LiDAR mounting, SLAM quality, or a real opponent's behaviour
match the model.

Before new settings go on the floor: build and run unit tests, test with wheels
off the ground, run at low speed in open space, validate map subtraction and
stop distances against the real LiDAR, and raise speed one parameter at a time.

**The mandatory LB deadman policy applies to every physical run regardless of
what the simulator says.** See
[docs/architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car).

## Reproducing these measurements

Every measured figure above came from probing the installed `.sim/` checkout
directly, using the same import bootstrap as `run_validation.py`:

```python
ROOT = Path("~/racerbot-ws").expanduser(); SIM = ROOT / ".sim"
for p in (SIM/"python", SIM/"f1tenth_gym", ROOT/"src"/"gap_follow", ROOT/"src"/"pure_pursuit"):
    sys.path.insert(0, str(p))
os.environ.setdefault("NUMBA_CACHE_DIR", str(SIM/"numba-cache"))
```

Build the env exactly as `make_env()` does, then:

- **Braking / acceleration (F1)** — hold 4 m/s until settled, command `speed = 0`,
  integrate position until `v ≤ 0.05`.
- **Grip (F2, F3)** — hold 4 m/s, apply constant steering, record
  `max |v · ψ̇|` over ~1.5 s. Sweep steering, then sweep `mu`.
- **Steering actuator (F6)** — command a constant 0.15 rad and print
  `std_state[2]` each tick.
- **Performance** — time N steps of `env.step()` with a constant action after a
  warm-up pass to let numba JIT settle, then `RTF = N · timestep / wall`.

Note the LiDAR cannot be disabled for these probes: collision detection reads
the scan cache, and `enabled=False` raises `IndexError`.
