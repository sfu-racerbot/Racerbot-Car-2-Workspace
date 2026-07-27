# F1TENTH Gym simulation and validation

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
- the pure-pursuit target curvature from the gap midpoint's *range and*
  bearing, its lateral-acceleration and stopping-distance speed ceilings, and
  the steering/acceleration rate limits; and
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

## Current validated result

The checked-in [full JSON report](f1tenth-sim-results.json) was regenerated on
2026-07-27 with seed `12345`, after the adaptive-speed work below. All seven
scenarios passed. The previous column is the 2026-07-21 run, before that work:

| Controller | Track | Simulated lap time | (was) | Max raceline error | (was) | Collision |
|---|---:|---:|---:|---:|---:|---:|
| Gap follow | Spielberg | 173.88 s | 178.20 s | 0.863 m | 0.954 m | No |
| Gap follow | Silverstone | 231.53 s | 238.33 s | 0.859 m | 0.964 m | No |
| Gap follow | Brands Hatch | 179.08 s | 185.53 s | 0.914 m | 0.866 m | No |
| Pure pursuit | Spielberg | 133.53 s | 94.45 s | 0.146 m | 0.197 m | No |
| Pure pursuit | Silverstone | 201.05 s | 127.25 s | 0.175 m | 0.219 m | No |
| Pure pursuit | Brands Hatch | 90.20 s | 88.95 s | 0.119 m | 0.157 m | No |
| Pure pursuit + opponent | Spielberg | 92.18 s | 86.53 s | 0.636 m | 0.413 m | No, either car |

**Gap follow got faster and tracked better on every track** — the range-aware
Pure Pursuit steering and the curvature/clearance speed ceilings are a
straight improvement over the old bearing-as-steering, linear-speed-scale law.

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
