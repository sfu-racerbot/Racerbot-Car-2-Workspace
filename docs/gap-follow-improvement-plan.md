# gap_follow improvement — handoff notes

> **Status (2026-10-19): implemented.** The fix this plan led to — swept-path
> clearance, an arc guard and corner setup — landed on `main`. What it does,
> how to switch each part off, and the measured results are in
> **[gap-follow.md](gap-follow.md)**; read that first. This file is kept as the
> record of the diagnosis. Its numbers describe the controller *before* the
> fix, and "the algorithm is unchanged" below was true when it was written.

Branch (at the time): `feat/gap-follow-improvement`. The algorithm in
`src/gap_follow/gap_follow/` was then **unchanged** from `superpowers/tracking-laws`
except for one dead-end experiment that was measured, found worse, and fully
reverted (see "Dead end" below).

---

## 1. The two tools built for this work

Both live in `tools/f1tenth_sim/` and are standalone (no ROS needed, but
`source install/setup.bash` is still useful for `gap_follow` importability).

### `eval_gap.py` — the iteration loop

```bash
python3 tools/f1tenth_sim/eval_gap.py --track indoor          # all 4 indoor tracks
python3 tools/f1tenth_sim/eval_gap.py --track asb_10000
python3 tools/f1tenth_sim/eval_gap.py --track all              # indoor + gym circuits
python3 tools/f1tenth_sim/eval_gap.py --track indoor --set corner_speed=1.8
```

It drives the **real `gap_logic` pipeline** (same functions, same order, as
`gap_follow_node._scan_step`) with every knob in `config/gap_follow.yaml`
overridable via `--set name=value`, across:

- the four `racerbot_sim` indoor layouts (`indoor_oval`, `indoor_tight`,
  `indoor_wide`, `asb_10000`) — the size class this car is actually driven
  in, and where every reported failure lives; and
- the three F1TENTH Gym circuits (`Spielberg`, `Silverstone`, `BrandsHatch`).

**Why this exists:** `run_validation.py`'s `gap_command` is a *stale mirror*
of the node. It still hardcodes `max_speed=2.0` (node: 3.5),
`safety_margin=0.10` (node: 0.18), `corner_speed=0.5` (node: 1.5),
`STEERING_LIMIT=0.26` (node: 0.349), `FORWARD_STOP_CLEARANCE=0.25` (node:
0.20), and has **no** adaptive width, **no** gap hysteresis and **no** escape
creep. Its numbers do not describe the algorithm the car runs. `eval_gap.py`
fixes that by construction — but it is *also* a mirror, and must be kept in
step with `gap_follow_node._scan_step` (same caveat the docs already record
for `run_validation.py`).

### `trace_gap.py` — the diagnostic trace

```bash
python3 tools/f1tenth_sim/trace_gap.py --track indoor_tight --out /tmp/trace.csv
```

Emits one CSV row per tick: position, yaw, measured/commanded speed, steering,
**every speed cap's value**, which cap bound, the chosen gap's angular width,
side-wall distances, forward clearance, minimum clearance, minimum TTC, and
the TTC-effective speed. This is what the diagnosis below was read off.

---

## 2. Baseline (seed 12345, fidelity profile `car`)

| track | result | laps | sim_time | mean_speed | stops | contact | min_fwd |
|---|---|---|---|---|---|---|---|
| indoor_oval (1.8m) | PASS | 1.0 | 15.8s | 1.91 | 2 | 0 | 1.50 |
| indoor_tight (1.4m) | **FAIL** | 0.0 | 6.2s | 1.49 | 9 | **1** | 1.19 |
| indoor_wide (2.6m) | PASS | 1.0 | 19.9s | 2.21 | 2 | 0 | 1.94 |
| asb_10000 (1.5m) | PASS | 1.0 | 24.6s | 2.05 | 2 | 0 | 1.34 |

Binding-cap histogram (which speed ceiling won each tick):

| track | curve | clearance | accel | stop |
|---|---:|---:|---:|---:|
| indoor_oval | 300 | 97 | 234 | 2 |
| indoor_tight | 120 | 32 | 87 | 9 |
| indoor_wide | 404 | 26 | 365 | 2 |
| **asb_10000** | 327 | **457** | 199 | 2 |

---

## 3. Diagnosis — what is actually wrong

### 3.1 "Turns too tightly / hugs the inside wall" — the primary defect

From `trace_gap.py --track indoor_tight`, approaching the first left corner:

```
t=1.5  yaw= -2.3  spd=2.28  steer=-0.004  L=0.719  R=0.679   <- straight, centred
t=2.1  yaw= +8.6  spd=1.52  steer=+0.166  L=0.585  R=0.825   <- turning left
t=2.7  yaw=+11.2  spd=1.28  steer=+0.163  L=0.254  R=1.107   <- INSIDE wall 0.25m
t=3.0  yaw=+17.9  spd=1.06  steer=+0.319  L=0.203  R=1.135   <- at the apex
t=3.4  yaw=+10.7  spd=1.08  steer=+0.141  L=0.276  R=1.126
t=3.475                                                     <- CONTACT
```

`L` is the left wall distance. It collapses **0.72 → 0.20 m** through the
corner. The car turns in *from the inside*: it enters a left-hand corner
already hugging the inside wall, arrives at the apex 20 cm from it, and
touches. A racing line enters a left corner from the **right** (outside),
crosses to the apex late, and exits wide.

**Root cause:** `gap_logic.aim_within_gap` picks the gap midpoint between two
real obstacle edges. On a corner the two edges are *not* the two walls of a
straight — the far edge is the opening around the bend, and the near edge is
the inside wall. The midpoint of that opening sits next to the inside wall.
Nothing in the node has any concept of corner entry geometry, of an apex, or
of an exit line. The `corridor_centering_bias` term pulls toward the corridor
*middle*, which on a corner is still the inside-biased midpoint.

### 3.2 "Too slow"

`asb_10000`'s binding cap is **`clearance`** (457 of ~980 ticks), not
curvature. The clearance ceiling is
`v <= sqrt(2 * max_braking_decel * (forward_clearance - forward_stop_clearance))`
with `max_braking_decel=3.0` and a `forward_stop_clearance=0.20` reserve:

| speed | forward clearance needed |
|---|---|
| 1.5 m/s | 0.57 m |
| 2.0 m/s | 0.87 m |
| 2.5 m/s | 1.24 m |
| 3.0 m/s | 1.70 m |

On a 1.5 m hallway with blind right-angle corners, the forward cone rarely
sees past ~1.25 m, so the ceiling sits at ~2.4 m/s even where the corridor is
straight and the car is nowhere near a wall. Measured mean speed is 1.95 m/s
against a 3.5 m/s cap, and the trace shows the clearance cap binding on
*straight* sections too (t=0.9, 1.2, 1.5, 2.0 all bind on `clearance` with
`fwd_clear` ~1.1-1.25 m and no corner in sight).

Two contributing causes, both worth attacking:
- the reserve is a *fixed* 0.20 m regardless of speed;
- the ceiling assumes the car travels **straight ahead** at the commanded
  speed, which is not what it does in a corner (see 3.1) and not what it can
  do at all when it is already steering.

### 3.3 "Gets stuck near a wall and stops"

On indoor_tight the failure is a **cycle**: `ttc_brake` → creep →
`ttc_brake` → creep, with the car progressively tighter against the inside
wall, until contact. The TTC brake (0.30 s) fires on the *inside* wall of the
corner because the car is hugging it (3.1), the directional escape gate lets
it creep out, and it immediately re-hugs. `emergency_clearance` /
`ttc_brake` are not independently wrong; they are reacting correctly to a
car that is in the wrong place.

`max_stuck_steps=5` on every track in the harness (5 ticks = 0.125 s) is the
"commanded to move but not moving" detector; the real stall is the *repeated*
trip/creep cycle, not a single long freeze.

---

## 4. What to change (designed, not yet implemented)

Ordered by expected leverage.

### A. Corner setup — bias the aim toward the OUTSIDE of an upcoming turn (primary)

Detect the turn and pull the car toward the outside wall *before* turn-in, so
it arrives at the apex with room and can then cut in.

- Signal available today: the chosen gap's **near-only bearing vs whole-gap
  bearing** disagreement (`gap_logic.near_gap_bearing`, already written and
  already used by the shipped-off anticipation feature) says which way the
  corridor leans. A left lean means a left corner is coming.
- Shape: a bounded cross-track bias toward the outside wall, faded in by turn
  confidence and by how far the corner is, in the same bounded/ramped style as
  `corridor_centering_bias` (which is the closest existing precedent in this
  file — follow it).
- Direction: a left turn needs a bias to the **right** (positive lateral
  offset), a right turn to the **left**.
- It must fade out *through* the corner once the car is committed, or it will
  hold the car wide into the apex and clip it. Fading by the aim bearing
  (as `corridor_centering_bias` does with `centering_full_bearing_deg`) is
  the natural mechanism.
- **Authority bound**: like `centering_max_steering`, it must be validated
  against `max_steering_angle` at startup — a corner-setup bias that can
  outvote the gap choice is a second driving policy.

### B. Anticipation speed cap, properly normalised (secondary)

`enable_cornering_anticipation` exists, is unit-tested, and **ships off**
because it over-braked in open rooms: the docstring says why — *"a wide gap's
own angular span naturally spans tens of degrees toward whichever edge is
more open, which near_gap_bearing's 'whole gap vs near-only' comparison reads
as 'a bend is coming' even in a straight, open room"*.

The fix the repo's own docs name: **normalise the disagreement by the gap's
own angular width** (e.g. `disagreement / gap_width_deg`), so a wide gap
leaning 20° does not read as a bend, while a narrow gap leaning 20° does.
Same mechanism, same `curvature_speed_limit` chain, better trigger.

With (A) in place this becomes much more valuable: the car will be *setting
up wide* for the corner, and the anticipation cap is what lets it then slow
*into* the corner at a speed it can hold, and speed back out.

### C. The clearance ceiling on a narrow corridor (the "too slow" fix)

Options, in order of preference:
- **Measure the clearance along the arc the car will actually sweep**, the
  way `time_to_collision` already does with `path_curvature` and
  `swept_half_width` (see `gap_logic.time_to_collision`'s turning branch).
  The forward-cone clearance is a straight-line measure; in a corner the
  car does not go straight, so the straight-line number is the wrong number.
- Make the reserve **speed-dependent** rather than a flat 0.20 m.
- Only then, consider relaxing `forward_stop_clearance` — but note it is
  shared with the *creep* threshold, so changing it moves the creep trigger
  too. The repo's own history warns that a smaller margin previously left
  the car "clipping corner apexes and then sitting ~0.08 m off the wall".

### D. Re-tune the flat caps once A-C land

`corner_speed` / `corner_speed_wide` / `max_lateral_accel` were last set
against a controller that had no corner setup. Once the car actually sets up
wide, the same curvature cap produces a faster, safer corner. Re-measure with
`eval_gap.py --set ...` rather than guessing.

---

## 5. Dead end already explored (do not repeat)

**`aim_physical_center`** — replaced the angular-midpoint aim with the beam
maximising the *physical* clearance to both gap edges (chord geometry, since
beams are evenly spaced in angle and not in metres, so an angular midpoint
does land closer to the nearer wall). Rationale was sound and it is the
obvious "principled" fix for 3.1. **It measured strictly worse:**

| track | midpoint | physical_center |
|---|---|---|
| indoor_oval | PASS 15.8s v=1.91 stops=2 | PASS 23.1s v=1.29 stops=**37** |
| indoor_tight | FAIL contact | FAIL contact (earlier) |
| indoor_wide | PASS 19.9s v=2.21 | PASS 26.3s v=1.57 stops=10 |
| asb_10000 | PASS 24.6s v=2.05 | **FAIL** contact |

The function was fully removed from `gap_logic.py`; `git log` on this branch
has it in the history if the reasoning is worth revisiting. The likely
reason it failed: maximising worst-case clearance inside the gap points at
the *most open* direction, which in a corner is the opening around the bend
— so it points further into the turn, and it chases a moving target (the
opening changes every tick), which is what the 37 stops on indoor_oval are.

---

## 6. Rules that bind this work

From `CLAUDE.md` and `TEST_QUALITY_STANDARDS.md`:

- **LB deadman stays enabled.** Do not touch the deadman path.
- **Every new test must be seen to fail** against a stub before it is trusted
  (break the code it covers, watch it fail, revert, watch it pass).
- **Never weaken a test to make a suite pass.** If a test is genuinely wrong
  after a design change, flag it with reasoning instead of editing it.
- `gap_follow` tests import `rclpy`; they must be run **sourced** and in an
  isolated domain, or 70 of them silently fail to collect:
  ```bash
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  python3 -m pytest src/gap_follow/test/ -v
  ```
  (229 tests currently pass on this branch.)
- `run_validation.py`'s `gap_command` must be updated to match the node if
  the node's pipeline changes — it is a hand-maintained mirror and nothing
  warns when it drifts. **This is a known, unfixed debt**; `eval_gap.py` is
  the better mirror but it is not wired into `colcon test`.
- Keep `docs/` and `src/gap_follow/README.md` in step with any behaviour
  change. The README documents the algorithm step by step and is the
  reference the team reads.
- The sim is not physical sign-off: wheels off the ground first, then low
  speed in open space (see `docs/writing-your-own-node.md`).
