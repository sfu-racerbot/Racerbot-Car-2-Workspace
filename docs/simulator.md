# F1TENTH Gym simulation and validation

> **Who this is for:** anyone who wants to test controller *math* quickly, with no ROS and no car.
> **Read first:** nothing. See [ros-simulator.md](ros-simulator.md) for the other simulator, which tests the ROS wiring instead.
> **You'll be able to:** run headless solo and multi-car validation, and know what those results do and don't prove.

This workspace includes a reproducible, headless validation harness for
`gap_follow` and `pure_pursuit`. It uses the official F1TENTH Gym vehicle
dynamics, LiDAR ray casting, map collision checks, and multi-agent collision
model while calling the same framework-independent controller math used by the
ROS nodes.

The supported path is intentionally headless and deterministic. It is suitable
for regression tests and tuning on this Jetson/ARM64 machine; it does not need
Docker, a display, RViz, or ROS topics.

## One-time setup

From the workspace root:

```bash
tools/f1tenth_sim/setup.sh
```

The script:

- clones the official `f1tenth/f1tenth_gym` `dev-humble` branch at pinned
  commit `bdaec1420c3b0f103858d289866d0d4e2e597c30`;
- installs pinned Python dependencies under `.sim/python`;
- downloads the Spielberg track and performs a smoke test; and
- leaves everything under `.sim/`, which is ignored by Git.

No `sudo`, global `pip` installation, or virtual environment is required.
Network access is required the first time. Re-running setup is safe; it refuses
to overwrite local modifications inside the simulator checkout.

Why `dev-humble` rather than the old default branch? The default branch pins
legacy Gym and NumPy versions that are not compatible with this workspace's
Python 3.12 environment. The official development branch uses Gymnasium and
lists Python 3.12/Ubuntu 24.04 support.

## Run the validation suite

Run all three solo tracks plus the two-car scenario:

```bash
python3 tools/f1tenth_sim/run_validation.py \
    --scenario all \
    --output docs/f1tenth-sim-results.json
```

Useful shorter runs:

```bash
# One track, both solo controllers, then traffic
python3 tools/f1tenth_sim/run_validation.py --scenario all --quick

# Gap follow only, all configured tracks
python3 tools/f1tenth_sim/run_validation.py --scenario gap

# Pure pursuit only
python3 tools/f1tenth_sim/run_validation.py --scenario pure

# Two cars: ego pure pursuit versus a slower path-following opponent
python3 tools/f1tenth_sim/run_validation.py --scenario traffic --quick
```

Two flags exist for measuring a change rather than just gating it:

```bash
# What does gap_follow's corridor centering actually do? Run both ways.
python3 tools/f1tenth_sim/run_validation.py --scenario gap --no-centering

# Which line should pure_pursuit follow: the one shipped with the track, one
# computed by pure_pursuit.raceline_optimizer, or the bare centerline?
python3 tools/f1tenth_sim/run_validation.py --scenario pure --raceline optimized
python3 tools/f1tenth_sim/run_validation.py --scenario pure --raceline centerline
```

`--no-centering` is a measurement tool, not a supported car configuration.

The process exits `0` only when every selected scenario passes, so it can be
used in CI. Each scenario prints one JSON object; `--output` writes a combined
machine-readable report. Other supported options are visible with `--help`,
including `--tracks`, `--seed`, and simulated-time `--timeout`.

The default tracks are Spielberg, Silverstone, and Brands Hatch. Track names
are resolved by F1TENTH Gym and cached in `.sim/f1tenth_gym/maps`.

## What is actually tested

The simulator runs at 40 Hz with a 5 ms RK4 integration step. Its vehicle is
matched to the Traxxas 74276-4's published 0.324m wheelbase, the deliberately
padded 0.31m × 0.58m collision body, and estimated +0.33m LiDAR offset. The
LiDAR model has 819 beams over ±135° with
small seeded noise.

`gap_solo` validates:

- scan sanitization, all-direction body clearance, the odometry-independent
  forward-cone brake, and conservative command-backed TTC braking;
- disparity extension without double-padding and the width-aware safety bubble;
- preferred gap scoring plus the slow tight-corner fallback;
- `aim_within_gap`'s choice of target beam — midpoint between two real
  obstacle edges, deepest beam when an edge is only the field-of-view
  boundary;
- bearing-proportional steering plus the corridor-centering bias, their
  lateral-acceleration and stopping-distance speed ceilings, and the
  steering/acceleration rate limits; and
- one complete lap with no Gym collision.

`pure_solo` validates:

- curvature-aware velocity profiling;
- adaptive lookahead — sized from measured speed, as the node does from
  `/odom` — and pure-pursuit steering;
- the online curvature speed ceiling and command rate limits applied to the
  final command;
- the fallback reactive safety layer before map subtraction is available;
- one complete lap with no collision and less than 0.5 m maximum raceline
  error.

`pure_traffic` uses two simulated cars. It validates map-subtraction detection,
wrapped opponent progress at start/finish, pass commitment, emergency-stop
priority, completion of at least one overtake, a full ego lap, and zero
collisions for both cars. The opponent follows the raceline at 2.0 m/s while
the ego uses the 4.0 m/s profile. Only the ego's commands are shaped and only
the ego sizes its lookahead from measured speed — the scripted opponent is not
the controller under test, so it deliberately keeps its previous fixed
behavior and stays a stable reference between runs.

This is the scenario that constrains `overtake_lookahead_distance` and
`max_acceleration`; see "Tuning changes justified by simulation" below.

The runner imports `gap_follow.gap_logic` and `pure_pursuit.racing_math`
directly. ROS-specific wiring, the joystick, physical VESC behavior, SLAM
quality, and topic timing are covered by unit/integration/launch checks rather
than emulated by this harness.

## The gap scenario was broken, and is fixed

Worth knowing before reading any `gap_solo` number below: between 2026-07-27
and 2026-07-30 the harness could not run `--scenario gap` at all. When
`gap_follow_node` moved from a pure-pursuit curvature law to bearing
steering, `gap_logic.target_curvature` and `gap_logic.steering_from_curvature`
went away with it, and `gap_command` was left calling both — an immediate
`AttributeError`. The 2026-07-27 `gap_solo` figures in the table below are
therefore the last ones the old code produced, not a baseline the current
code was ever measured against.

`gap_command` now mirrors the node: bearing steering, `aim_within_gap` (moved
into `gap_logic` so both the node and this harness can use it), and the
corridor-centering bias. One drift is deliberately left in place and flagged
rather than silently fixed: the harness still hard-stops inside
`forward_stop_clearance` where the node creeps out at `escape_creep_speed`.

**The lesson for anyone editing a controller here: `run_validation.py` is a
hand-maintained mirror of the nodes, and nothing tells you when it stops
matching.** If you change the steering law, change it in both places, and run
the scenario afterwards.

## Corridor centering, measured

`--scenario gap` with and without `--no-centering`, seed `12345`.
`mean_corridor_offset_m` is the metric that answers the question directly:
the average distance from the middle of the corridor over the lap, sampled
only where both walls are actually visible.

| Track | Off-centre, no centering | Off-centre, with | Change | Lap time | Safety stops |
|---|---:|---:|---:|---:|---:|
| Spielberg | 0.076 m | **0.062 m** | −18.5% | +0.65 s | 1 → 1 |
| Silverstone | 0.095 m | **0.076 m** | −20.0% | +1.27 s | 2 → 2 |
| Brands Hatch | 0.092 m | **0.060 m** | −34.6% | −0.60 s | **18 → 1** |

No collisions in either configuration. Maximum cross-track error fell on all
three tracks (0.950→0.919, 0.980→0.932, 0.870→0.849 m). The Brands Hatch
result is the interesting one: better lateral positioning meant the car
stopped tripping its own safety layers, and the stop count fell from 18 to 1.

The cost is 0.4–0.6% of lap time on the two tracks where nothing else
changed, which is the centering bias asking for steering that the curvature
speed ceiling then answers by slowing down slightly. That is the trade being
made deliberately: a little lap time for consistently better track position
and more clearance in reserve for whatever the next corner holds.

## Current validated result

The checked-in [full JSON report](f1tenth-sim-results.json) was regenerated on
2026-07-30 with seed `12345`, after the corridor-centering work. All seven
scenarios passed. `pure_solo` and `pure_traffic` are unchanged to the
millisecond from the 2026-07-27 run, which is the intended result — the
raceline optimizer is an offline tool and changes nothing the node does.
The previous column is the 2026-07-21 run:

| Controller | Track | Simulated lap time | (was) | Max raceline error | (was) | Collision |
|---|---:|---:|---:|---:|---:|---:|
| Gap follow | Spielberg | 173.03 s | 178.20 s | 0.919 m | 0.954 m | No |
| Gap follow | Silverstone | 231.70 s | 238.33 s | 0.932 m | 0.964 m | No |
| Gap follow | Brands Hatch | 179.47 s | 185.53 s | 0.849 m | 0.866 m | No |
| Pure pursuit | Spielberg | 133.53 s | 94.45 s | 0.146 m | 0.197 m | No |
| Pure pursuit | Silverstone | 201.05 s | 127.25 s | 0.175 m | 0.219 m | No |
| Pure pursuit | Brands Hatch | 90.20 s | 88.95 s | 0.119 m | 0.157 m | No |
| Pure pursuit + opponent | Spielberg | 92.18 s | 86.53 s | 0.636 m | 0.413 m | No, either car |

**Gap follow tracks better on every track than the 2026-07-21 baseline.**
Read those two columns as separate experiments rather than a controlled
comparison, though: the "was" column predates both the switch to bearing
steering and the harness repair described above.

**The raceline optimizer does not appear in this table at all, by design.**
`optimize_raceline` is an offline tool that writes a `waypoints_file`; the
node is untouched, so the default `--raceline shipped` numbers must not move,
and they don't. Its own measurement is in
[racing-autonomy.md](racing-autonomy.md#what-it-actually-buys-measured) —
including why Brands Hatch is the only one of the three tracks on which that
comparison is readable.

**Pure pursuit solo is still slower than the pre-change baseline on two of
three tracks, and this is understood.** It is the `max_acceleration` command
ramp paying for avoidance recovery.

One real defect was found and fixed along the way: the ramp originally
integrated from the *last command* rather than the car's measured speed, so a
one-tick ceiling (avoidance, curvature) left it climbing back from a speed the
car had never actually dropped to — braking a car that never slowed. Seeding
the ramp from `/odom` recovered 148.33 s → 133.53 s on Spielberg and
213.55 s → 201.05 s on Silverstone. The remainder is genuine: after a
*sustained* avoidance the car really is at 1 m/s and must accelerate back.

Bisecting the residual on Spielberg, one component at a time (before that
fix, so the absolute numbers are the 148.33 s baseline):

| Relaxed to unlimited | Lap time | Avoidance steps |
|---|---:|---:|
| *(nothing — as committed)* | 148.33 s | 1361 |
| `max_acceleration` only | 103.53 s | 677 |
| `max_lateral_accel` (curvature ceiling) only | 146.80 s | 1350 |
| `max_braking_decel` only | 148.33 s | 1361 |
| `max_steering_rate` only | 159.15 s | 1422 |

Only the acceleration ramp matters. The braking limit is bit-for-bit
irrelevant here because avoidance already drops speed instantly via the hard
cap, never through the slew. The steering rate limit is mildly *helping* —
removing it costs 11 s, presumably by letting the steering chatter more.

The mechanism: every time the reactive net engages, speed is capped to
`avoidance_speed` (1.0 m/s) immediately; every time it clears, the ramp takes
~0.5 s to climb back to 4.0 m/s. Spielberg triggers that cycle often enough
(1133 steps, ~28 s, spent in avoidance) for those recoveries to dominate.

The controlled case confirms it: **Brands Hatch has zero avoidance steps and
cost only 1.4%** (88.95 → 90.18 s). With no avoidance events there is nothing
for the ramp to pay for. Spielberg and Silverstone spend 21% and 29% of their
steps in avoidance respectively — but only because `pure_solo` deliberately
exercises the *no-map fallback* trigger, where ordinary track walls set it
off. The car's actual default is `opponent_detection_mode: map`, which
subtracts those walls, so this cost should be far smaller on a mapped track.
Treat the Brands Hatch figure, not the Spielberg one, as the estimate for a
mapped race — and treat the avoidance flicker itself as the real lap-time
lever here, since it predates this change (Spielberg was already at 476
avoidance steps before it).

The traffic run completed one pass and one independently measured 338.128 m
lap. The pinned Gym revision leaves its native `lap_counts` at zero in this
two-agent case, so the validator uses wrapped nearest-raceline progress for the
multi-agent lap criterion while continuing to use Gym for dynamics, LiDAR, and
both collision flags. Solo scenarios use Gym's native lap counter.

## Tuning changes justified by simulation

The original 4 m/s pure-pursuit configuration used a 2.0 m lookahead and
collided in roughly 10 simulated seconds. The validated defaults are now:

- `min_lookahead: 0.6`, `lookahead_speed_gain: 0.15`,
  `max_lookahead: 1.5`;
- profile `v_max: 4.0 m/s`, `a_lat_max: 2.5 m/s²`;
- a 60° avoidance cone;
- map-aware dynamic-object trigger at 1.5 m and raw-scan fallback at 0.7 m;
- map-subtraction opponent detection by default; and
- a 0.35 m overtake target offset.

Map subtraction matters because an unfiltered 1.5 m raw-scan trigger sees
normal track walls almost continuously. During a committed pass, generic
1.0 m/s gap avoidance is suppressed because it would make passing a 2.0 m/s
opponent impossible; stale-scan and 0.4 m emergency stops still always win.

### The adaptive-speed work: two values the traffic scenario pinned down

Adding the online curvature speed ceiling and command rate limits passed
`gap_solo` and `pure_solo` immediately, and broke `pure_traffic` — which is
the useful outcome, because the interaction it exposed is real and would have
shown up on track rather than in a test.

**`overtake_lookahead_distance: 4.0`.** The overtake used to offset the
*normal* pure-pursuit target, at most `max_lookahead` (1.5 m) away. Nudging a
point that close 0.35 m sideways demands a curvature well past the 0.26 rad
steering clamp, and the new lateral-acceleration ceiling correctly answers a
demand like that by slowing down — mid-pass, which is precisely wrong. The ego
stalled behind the opponent and covered **0.1 laps in 240 s**. Applying the
same offset to a point 4 m ahead makes it a gentle arc instead. Probed at 4, 6,
and 8 m; all three completed the lap, and 4 m held the smallest raceline error
(0.78 m vs 1.05 m at 6 m), so the shortest preview that works is the default.

**`max_acceleration: 6.0`.** With the 4 m preview in place but a 3.0 m/s²
command ramp, the run still failed the same way (**9100 of 9600 steps
stopped**): the ego could not rebuild speed between safety stops behind the
slower car. At 6.0 m/s² it completes a clean lap with one pass and 78 stopped
steps; 10.0 also passes with no further gain. This is a limit on how fast a
*command* may rise, not a demand on the motor — the lesson worth carrying to
the physical car is that an over-tight ramp is not the conservative choice it
looks like.

Both values are mirrored in `run_validation.py` (`OVERTAKE_LOOKAHEAD`,
`PURE_MAX_ACCELERATION`) so the harness keeps testing what the node actually
does. If you change one, change both.

`max_acceleration` is bounded on **both** sides, and the usable band is narrow.
`6.0` sits near the top of it:

| `max_acceleration` | Traffic scenario | Solo Spielberg |
|---:|---|---:|
| 3.0 | **fails** — stalls, 0.1 laps in 240 s | — |
| 5.0 | passes, 92.48 s, 2 passes | — |
| 6.0 | passes, 92.18 s, 1 pass | 133.53 s |
| 7.0 | **fails** — 8991/9600 steps stopped | — |
| 8.0 | **fails** — 9007/9600 steps stopped | 123.08 s |
| 10.0 | **fails** — 9157/9600 steps stopped | 114.15 s |

Too low and the car cannot rebuild speed between safety stops, so it stalls
behind a slower car. **Too high and it arrives behind that slower car too fast
for the overtake to commit**, hard-stops inside `emergency_stop_distance`,
and enters a stop-go cycle it never leaves. The upper bound is a real dynamic,
not a simulator quirk — arriving hot behind traffic defeats the pass — so
solo lap time cannot simply be bought by raising this. `6.0` passes across
seeds 12345/777/2024 (92.2/92.7/92.0 s), but it is one step from the cliff:
**re-run `--scenario traffic` after any change to it.**

## The ROS-level simulator is a separate thing

This harness has no ROS in it at all, by design. That makes it fast and
deterministic and blind to an entire class of failure: SLAM, TF, launch
files, topic wiring, and handovers between nodes. Every defect found in
`auto_map_race_launch.py` on 2026-08-08 was in that class.

`racerbot_sim` ([docs/ros-simulator.md](ros-simulator.md)) puts the same
F1TENTH Gym physics behind the car's real topics and runs the real launch
files over it. Use this harness to tune a control law; use that one to
prove a launch file works.

### Wall collisions in this harness do not fire

Worth knowing before trusting the `collision` column above. In the pinned
Gym revision the wall check is `raw_scan - side_distances <= 0.005`, where
`side_distances` is the distance from the LiDAR to the body edge along
each beam. With the vehicle parameters this file sets
(`collision_body_center_x = WHEELBASE/2`), the ST model's own `-lr` offset
cancels it exactly, putting the collision rectangle on `base_link` -- and
the LiDAR at `+0.33m` then sits 0.04m *outside* that rectangle, so
`_ray_to_rect_distance_vec` returns 0 for every beam. The test becomes
"did a beam return less than 5mm", which it cannot: `range_min` is 0.05m.

Measured directly: a car commanded straight into a wall at 1 m/s reports
`collision=False` for the whole run and drives off the edge of the map.

The scenarios above still fail on the criteria that do work -- lap
completion, cross-track error, stop counts, TTC braking -- and the
controllers were tuned against those. But "No" in the collision column
means "not detected", not "did not happen". `racerbot_sim` samples the
padded body against the occupancy grid directly instead, and
`tools/racerbot_sim/run_auto_map_validation.py` gates on that.

## Optional official ROS bridge

The official [`f1tenth_gym_ros`](https://github.com/f1tenth/f1tenth_gym_ros)
bridge is useful for interactive ROS visualization and currently supports one
or two agents. It is not required by this repository's supported validator.
If you install it separately, use its ROS 2 development branch and follow the
upstream topic/config instructions; do not mix its `/drive` publisher with the
physical-car bringup.

The base Gym supports configurable agent counts; see the official
[`f1tenth_gym` repository](https://github.com/f1tenth/f1tenth_gym) and
[customized usage documentation](https://f1tenth-gym.readthedocs.io/en/stable/customized_usage.html).

## Simulation is not physical sign-off

A passing simulation is evidence against algorithm and regression errors, not
proof that tire grip, servo calibration, LiDAR mounting, SLAM, or real opponent
behavior match the model. Before using new settings on the floor:

1. build and run unit tests;
2. test with wheels off the ground;
3. run at low speed in open space;
4. validate map subtraction and stop distances on the real LiDAR; and
5. increase speed one parameter at a time.

The mandatory LB deadman policy remains enabled in every physical launch.

**How far off is the model, specifically?** See
[docs/sim-fidelity-audit.md](sim-fidelity-audit.md) for a measured audit of
where this harness diverges from the real car. Short version: the geometry and
timing match, but braking authority, tire grip, the steering actuator, and
localization all diverge in the *optimistic* direction, and no dynamics
parameter has ever been measured on this car. The audit also explains why the
"steering rate limit is mildly helping" result above is a simulator artifact.
