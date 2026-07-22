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
matched to this repository's configured 0.25 m wheelbase and 0.30 m width. The
LiDAR model has 819 beams over ±135° with small seeded noise.

`gap_solo` validates:

- scan sanitization and emergency stopping;
- disparity extension and the width-aware safety bubble;
- gap scoring, steering, and steering-dependent speed; and
- one complete lap with no Gym collision.

`pure_solo` validates:

- curvature-aware velocity profiling;
- adaptive lookahead and pure-pursuit steering;
- the fallback reactive safety layer before map subtraction is available;
- one complete lap with no collision and less than 0.5 m maximum raceline
  error.

`pure_traffic` uses two simulated cars. It validates map-subtraction detection,
wrapped opponent progress at start/finish, pass commitment, emergency-stop
priority, completion of at least one overtake, a full ego lap, and zero
collisions for both cars. The opponent follows the raceline at 2.0 m/s while
the ego uses the 4.0 m/s profile.

The runner imports `gap_follow.gap_logic` and `pure_pursuit.racing_math`
directly. ROS-specific wiring, the joystick, physical VESC behavior, SLAM
quality, and topic timing are covered by unit/integration/launch checks rather
than emulated by this harness.

## Current validated result

The checked-in [full JSON report](f1tenth-sim-results.json) was generated on
2026-07-21 with seed `12345`. All seven scenarios passed:

| Controller | Track | Simulated lap time | Max raceline error | Collision |
|---|---:|---:|---:|---:|
| Gap follow | Spielberg | 178.20 s | 0.954 m | No |
| Gap follow | Silverstone | 238.33 s | 0.964 m | No |
| Gap follow | Brands Hatch | 185.53 s | 0.866 m | No |
| Pure pursuit | Spielberg | 94.45 s | 0.197 m | No |
| Pure pursuit | Silverstone | 127.25 s | 0.219 m | No |
| Pure pursuit | Brands Hatch | 88.95 s | 0.157 m | No |
| Pure pursuit + opponent | Spielberg | 86.53 s | 0.413 m | No, either car |

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
