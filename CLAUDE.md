# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS2 Jazzy workspace for SFU Racerbot's **car 2** roboracer/F1TENTH car (Jetson Orin Nano Super, JetPack 7.2, Ubuntu 24.04).

**Car 1 vs car 2.** [Racerbot-Car-1-Workspace](https://github.com/sfu-racerbot/Racerbot-Car-1-Workspace) is an independent sister repo, not an upstream — port from it only when explicitly asked (git remote `car1`), and never overwrite car 2's hardware differences: **Ethernet Hokuyo UST-10LX** at `192.168.0.10` (car 1: USB-serial UST-30LX), the **RealSense D435i** (car 1: UVC webcam), car 2's own VESC calibration in `vesc.yaml` and steering limits, dashboard id **`rb2`** in `racerbot_launch/config/web_dashboard_rb2.yaml`, and `racerbot_doctor`'s Ethernet/RealSense checks. **Car 2 has no BNO085 IMU yet** (being mounted late October 2026) and no `~/imu-calibration` checkout, so the ported `imu_bridge` stack is **off by default here** (`map_course imu:=false`, `vehicle_state_launch.py bno:=false`). **Before porting anything from car 1, read `docs/car1-vs-car2.md`**, and run `src/racerbot_launch/test/test_car2_hardware_config.py` after every port (a failure there means the port clobbered car 2; fix the port, not the test). Add a row there and a check there for any new difference you find. This is a physical robot that can hurt itself, people, or property if driving code is wrong — treat any change to a node that can publish `/drive` with corresponding care (see "Safety model" below).

**`docs/` is the primary source of truth and is unusually thorough — read the relevant doc before making non-trivial changes rather than inferring behavior from code alone.** Start with `docs/architecture.md` (node/topic graph, safety model) before writing or touching any driving code. Full index in [README.md](README.md).

## Build, test, run

```bash
source /opt/ros/jazzy/setup.bash        # every new terminal, always first
cd ~/racerbot-ws
colcon build --symlink-install          # full build
colcon build --symlink-install --packages-select <pkg>   # fast, single-package build while iterating
source install/setup.bash               # every new terminal, always second
```
- On the Jetson's 8GB RAM, add `--parallel-workers 1` to a full rebuild to avoid OOM.
- `--symlink-install` means edited `.py`/launch/config files take effect on next launch with no rebuild — only rebuild after touching `package.xml`, `setup.py`, C++ sources, or adding/removing files.
- `rm -rf build install log && colcon build` is safe if the build state is ever broken (these three dirs are gitignored, pure build artifacts).

Run the tests — **source ROS first, and isolate the ROS domain.** `gap_follow`, `pure_pursuit`, and `usb_cam_stream` each contain test files that import `rclpy`; unsourced they fail to collect, hiding **177 of the 900 tests**, including every deadman, watchdog, and node-behavior test. Sourced, those tests start real driving nodes that join whatever ROS graph they can see. Until 2026-09-27 `gap_follow`'s node tests faked an LB press and published to the **real `/drive`**, which with the driver stack up reaches `ackermann_mux` (it happened that day; the battery was off). Every driving-node test now publishes on `/test_only/drive` — `gap_follow`'s `test_no_test_here_builds_a_node_on_the_real_drive` enforces it, and the remap must come **after** any `--params-file` or the YAML's `drive_topic: /drive` wins — but they still put latched topics (`/racing_line`, `/drive_intent`) on whatever graph they join. So always give ROS tests their own domain:
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # never the car's graph
python3 -m pytest src/<pkg>/test/ -v                              # one package
colcon test --packages-select <pkg> && colcon test-result --verbose
python3 -m pytest tools/f1tenth_sim/tests/ tools/racerbot_sim/tests/ tools/run_analysis/tests/ -q   # the tool suites
```
The three tool suites are not colcon packages, so `colcon test` never runs them; the last line does (580 tests from 480 `def test` on car 2, 2026-10-29, the rest parametrized; car 1 had 472 from 382).
Verified 2026-08-22: unsourced, `src/pure_pursuit/test/` collects 185 of 274 and aborts on 5 collection errors, `gap_follow` hides 70, and `usb_cam_stream` collects nothing at all. **Never reach for `--continue-on-collection-errors`** — it turns that loud abort into a green run over a fifth of the suite. Compare the collected count against `grep -c '^def test' src/<pkg>/test/*.py`.

`drive_intent`, `odom_calibration`, `race_diagnostics`, `racerbot_sim`, and (apart from one rclpy file) `web_dashboard` are pure Python and do run unsourced — that is the pattern to follow for any new package with non-trivial math or parsing: keep the logic importable without `rclpy` (`pure_pursuit/racing_math.py`, `web_dashboard/protocol.py`, all of `drive_intent`).

Drive the car (manual):
```bash
ros2 launch f1tenth_stack bringup_launch.py     # terminal 1: hardware + arbitration, never moves the car alone
ros2 launch f1tenth_stack teleop_launch.py       # terminal 2: control layer, hold LB, sticks = drive
```
Full command reference for every workflow (mapping, localization, autonomy, racing, shutdown) is in `docs/operations.md`.

## Architecture

Everything communicates over ROS2 topics only — no shared memory, no direct function calls between packages. The topic graph *is* the system; full diagram and topic table in `docs/architecture.md`.

**Two-tier launch pattern, always:**
1. `bringup_launch.py` (`f1tenth_stack`) — the shared foundation, started once: `joy_node`, VESC chain, LiDAR (`urg_node`), `ackermann_mux`, and `race_diagnostics`' `auto_record` (records every run of an algorithm; off with `record:=false`). Deliberately starts nothing that can move the car by itself.
2. Exactly one **control layer** on top, in a separate terminal: `teleop_launch.py` (manual), `gap_follow_launch.py` (reactive autonomy), `pure_pursuit_launch.py` (map-based race controller), or a new node. Each publishes `AckermannDriveStamped` to either `/teleop` or `/drive`; `ackermann_mux` arbitrates.

**Arbitration:** `/teleop` (priority 100) always beats `/drive` (priority 10) while `/teleop` hasn't timed out. Verified behavior: `joy_teleop`'s default profile publishes continuously even when LB isn't held, so **`/teleop` never times out while `teleop_launch.py` is running** — meaning autonomy's `/drive` commands never reach the VESC if `teleop_launch.py` is also up. Running autonomy means simply not launching `teleop_launch.py`, not starting-then-stopping it.

**Packages** (`src/`):
| Package | Kind | Role |
|---|---|---|
| `f1tenth_system` (+ `ackermann_mux`, `teleop_tools`, `vesc`) | vendored, plain tracked files (not a submodule — see `docs/git-setup.md`) | VESC driver, `urg_node`, joystick teleop, command muxing; owns all launch files + YAML config |
| `transport_drivers` | git submodule (`humble`) | serial transport dep for `vesc` |
| `particle_filter` (+ `range_libc`) | git submodules (`humble-devel`) | Monte Carlo localization against a saved map |
| `gap_follow` | local | reactive autonomy, follow-the-gap on `/scan` → `/drive`, no map — the reference template for new driving nodes. Opponent awareness for several cars (`opponent_tracking.py`, `enable_opponent_awareness`), **on by default since 2026-10-28**. Off with `gap_follow_launch.py opponent_awareness:=false`. The mapping launches (`map_course`, `auto_map_race`, `autonomous_mapping_launch.py`) turn it off, since mapping is always one car; `opponent_awareness:=true` turns it on there. The follow cap is held for 1 s after a followed car is lost (`opponent_follow_memory_sec`; 0 turns the memory off). Simulator-validated only; never run on the floor. Race defaults from car 1 (2026-10-27, ported 2026-10-29): `max_speed` **5.0**, tied speeds ×5/3.5, ramp 5.0. Gyro steering damping and adaptive grip exist but are **off on car 2** (`steering_damping:=false` default) until the BNO085 is set up (`docs/car1-vs-car2.md#imu`); `max_lateral_accel` 2.5 like car 1. IMU features ported from car 1 2026-10-29 (`fused_speed`, `imu_understeer`, `imu_traction`, `imu_impact_stop`, `imu_all` launch flags) all ship **off**; their thresholds are car 1's measurements until car 2 records its own (`docs/car1-vs-car2.md#imu-feature-thresholds`) |
| `pure_pursuit` | local | map-based race controller: recorded+paced racing line, pure pursuit control, reactive safety net, opponent overtaking — see `docs/racing-autonomy.md` |
| `drive_intent` | local | shared schema + trajectory prediction for `/drive_intent` (what a driving algorithm is trying to do, and why); pure Python, no `rclpy`, plus a single-header C++ port for `racerbot_a`/`racerbot_b` — see `docs/drive-intent.md` |
| `racerbot_sim` | local | F1TENTH Gym exposed as the car's own topics (`/scan`, `/odom`, TF), replacing only the hardware layer so the real stack — SLAM, `auto_map_race`, the dashboard — runs unchanged above it; `sim_race_launch.py` runs two cars, including `opponent:=team_a`. Has a hard interlock against running beside real drivers — see `docs/ros-simulator.md` |
| `web_dashboard` (+ `usb_cam_stream`) | git submodule `src/web_dashboards` → [sfu-racerbot/web-dashboards](https://github.com/sfu-racerbot/web-dashboards) (`main`), packages under `car/ros/` | the car-side server of the browser dashboard at dashboard.sfuracerbot.ca — the site, both packages and their docs live in that repo, which ships **generic** defaults; car 2's settings are `racerbot_launch/config/web_dashboard_rb2.yaml`, started with `ros2 launch racerbot_launch dashboard_launch.py` (a bare `web_dashboard_launch.py` allows no site). Also ships foxglove_bridge's config (client publishing **off**) and its boot unit. **Car 2 also runs the dashboard at boot**: `racerbot-dashboard.service` (template + `install.sh` in `racerbot_launch/systemd/`, `Restart=always`), so never start a second `dashboard_launch.py` there — `dashboard_test_launch.py` and `sim_auto_map_race_launch.py` default `dashboard:=false`. See `docs/web-dashboard.md`. Browser dashboard over WebSocket. Publishes to **no ROS topic**, so it is not subject to the safety policy below. Four bounded write paths: live tuning (`set_parameters`), stopping a driving process (OS signals), resetting live SLAM (`slam_toolbox/reset`, refused while a driving node runs), and deleting a saved run directory on disk. All on by default; see `docs/web-dashboard.md`'s security note |
| `race_diagnostics` | local | read-only run recorder + post-run analyzer. **`auto_record`, started by every bringup (`record:=false` turns it off), records each run of any algorithm** (any non-bringup publisher on `/drive` and the other command topics, ours or team A's) by running `record_run.py` with `name_prefix:=auto_`, and never beside another recorder (`race_diag_node`, `rosbag2_recorder*`); it only reads the graph, no pub/sub; logic in `auto_record.py`, no rclpy; an addition to vendored `f1tenth_system`, ported from car 1 2026-10-29 (`docs/git-setup.md` Addition 6); records into `data/racerbot_runs/` on car 2. Analyzer: pipeline health, pose lag, watchdog stops, rosbag — subscribes only; each run also gets `metadata/` (git state + diff of racerbot-ws and imu-calibration, IMU calibration copies, every node's parameters via `snapshot_params`, which only calls parameter services); see `docs/run-diagnostics.md`. `tools/run_analysis/imu_report.py <run>` compares gyro / steering-model / LiDAR turn rates and SLAM's heading corrections |
| `racerbot_doctor` | local | read-only hardware + bringup self-check, `ros2 run racerbot_doctor doctor`: per device, names the first broken layer (USB/link → kernel driver → /dev + perms → ROS driver process → rate → content), 3S battery from `/sensors/core`. Never publishes, never opens the VESC tty or camera, never TCP-probes the Hokuyo while `urg_node` runs (static test enforces the first two). Only `live.py` imports rclpy; the rest runs unsourced. Thresholds/IDs in `car.py` — keep in step with `vesc.yaml`, `sensors.yaml`, `bringup_launch.py`'s laser TF |
| `imu_bridge` | local (written on car 2, PR #2; extended on car 1 and ported back 2026-10-24) | `imu_bridge_node`: calibrated car-axis `/imu/*` from the VESC IMU and a BNO085 (car 1: Jetson I2C bus 7 via `~/imu-calibration/tools/bno_i2c_bridge.py` → `/tmp/bno085`; **car 2: no BNO085 yet**). Needs `~/imu-calibration` (mounts and calibration) or `imu_bridge_node` refuses to start — not cloned on car 2 yet. `vehicle_state_node`: `/odom/fused` (Kalman speed from wheels + BNO forward accel, slip-gated, **clamped to ±0.3 m/s of the wheel speed**, exactly the wheel speed without accel) and `/steering/measured` (`JointState` "steering", `atan(L·yaw/v)`, NaN below 0.3 m/s or when gyros disagree), and **gyro odometry** `/odom/imu` (frame `odom_imu`: vesc_to_odom's distance, gyro heading, exact `/odom` fallback). With `publish_odom_tf` it also publishes `odom_imu -> odom` so SLAM can map on `odom_imu` while vesc_to_odom keeps `odom -> base_link`; it stops that transform for good if anything else parents `odom`. Never publishes drive topics; `check_output_topics`/`check_frames` refuse drive-like names, `/odom`, and tree-breaking frames. Math in `vehicle_state.py`, no rclpy. `imu_bridge_node` also publishes `/imu/status` (latched JSON `ok`/`waiting`/`fault` for the primary IMU: startup bias, stationary gravity check, all-zero, non-finite, stale; `imu_health.py`), which `neural_racer` requires. It is the only owner of `/imu/*` — the separate `racerbot_imu` package was removed 2026-10-24. `vehicle_state_launch.py` starts all three; `map_course` includes it with `imu:=true` (**default false on car 2** until the BNO and imu-calibration are set up); `fused_speed:=true` points `gap_follow` at `/odom/fused`; `gyro_odom:=true` moves slam_toolbox to `odom_imu` (both off by default). See `src/imu_bridge/README.md` |
| `neural_racer` | local (ported from car 1, 2026-10-24) | learned racing policy (NumPy on the CPU, trained off-car with `tools/neural_racer/` on the FIR cluster, which needs `torch`; `tools/neural_racer/tests` fail to collect on the car without it). **Driving code**: LB deadman, shadow mode by default (`/neural_racer/shadow_drive`, never `/drive` until `shadow:=false`), only loads models approved by `approve_model`, deterministic safety wrapper (car 2: `max_steering_angle` 0.349). `require_imu: true` means it commands zero unless `/imu/status` is `ok`, so **on car 2 it cannot drive until the IMU stack runs**. Never driven on car 2. See `src/neural_racer/README.md` |
| `racerbot_wiimote` (`racerbot_wiimote_driver`, `racerbot_wiimote_msgs`) | vendored from sfu-racerbot/ros2_wiimote_driver, plain files, fixed here | Wiimote → `sensor_msgs/Joy` on **`/wiimote/joy`, never `/joy`** (a Wiimote must not act as the F710's LB for autonomy). `wiimote_teleop_launch.py` is a manual control layer (driver + `joy_teleop` → `/teleop`); run it instead of `teleop_launch.py`, never beside it. Every button and axis is zero unless "2" (its deadman) is held, and it publishes that zero frame whenever the Wiimote is missing, disconnected or silent for 0.5 s. Needs `apt install xwiimote libxwiimote-dev bluez`; see `docs/wiimote.md` |
| `racerbot_launch` | local | launch glue not owned by any single driver repo (SLAM, race-day localization+pure_pursuit combos). `ros2 run racerbot_launch map_course` (needs `bringup_launch.py` already up; never starts drivers itself) = 3 gap_follow laps → map + racing line saved → shuts down → prints `race_launch.py run_dir:=...` commands (seeded from the run's `start_pose.yaml`, no RViz). Daily commands: `docs/launchstart.md` |

`slam_toolbox` is apt-installed (`ros-jazzy-slam-toolbox`), not vendored.

## Safety model — read before writing or running any driving code

**Workspace policy, currently in force, do not relax unilaterally:** every node that can move the car requires the driver to be actively holding **LB** on the physical F710 controller (XInput mode), on top of `ackermann_mux` arbitration. This is enforced in code, independently, by every autonomy node — `gap_follow_node`, `pure_pursuit_node`, `auto_map_race_node` and `neural_racer_node` each subscribe to `/joy` directly and refuse to publish non-zero drive commands without a live LB hold (`enable_deadman: true` default in every shipped config). **Never set `enable_deadman: false`** — that's a unilateral policy change, not a tuning knob. Full reasoning: `docs/architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car`.

**Any new node that publishes to `/drive`, `/ackermann_cmd`, or `/commands/motor|servo/*` must implement the same LB deadman check** — copy-paste pattern from `gap_follow_node.py`'s `joy_callback`/`_deadman_engaged`, documented in full in `docs/writing-your-own-node.md`. Decide which category new code falls into (driving vs. support/tooling) using `docs/adding-your-own-code.md` — when in doubt, treat it as driving code.

**Diagnostics published from a driving node** (currently only `/drive_intent`) must never be able to cost the car anything: publish strictly *after* the drive command for the tick, wrap the whole thing in one try/except that disables the diagnostic rather than the node, and read only what the control path already computed. Full contract and tests: `docs/drive-intent.md#safety-contract-for-publishers-read-this-first`.

Test order for any new driving node, never skip ahead: static topic check (no driver stack running) → wheels off the ground (full stack + LB held) → floor, low speed, open space. See `docs/writing-your-own-node.md#testing-before-its-on-wheels`.

## Test Quality — binding on all test work in this repo

**Source of truth: [TEST_QUALITY_STANDARDS.md](TEST_QUALITY_STANDARDS.md)** — the reasoning, worked examples from this tree, and copy-paste sweeps live there. This section is the short version, meant to be re-read every session. **It governs all test-writing in this repo from now on** — new packages, edits to existing suites, and work done in future sessions by anyone, human or agent. It is not scoped to the audit that produced it.

The rule behind all of it: **a test that cannot fail is worse than no test.** It costs review time, occupies the name of coverage it isn't providing, and turns "the suite is green" from evidence into noise. On a car that can hurt someone, that is a safety problem, not a tidiness one.

### Hard rule 1 — prove every new test can fail, before calling it done

**Before marking any new or changed test complete, break the code it covers and watch the test fail.**

1. Stub the unit under test — `pass`, `return <the exact constant the test expects>`, or `raise NotImplementedError`.
2. Run only that test.
3. It **must** fail. If it still passes, the test is hollow — it is detecting nothing. Rewrite it.
4. Revert the break, re-run, confirm green. Never commit a stub.

`return <the expected constant>` is the case people skip and the one that catches a test that has memorised an answer instead of checking a computation. For math, geometry, and control code, also require at least one of these to be caught: flipped comparison (`<`→`<=`), negated sign, a constant changed by 10%, two adjacent same-typed args swapped, an input returned unchanged, a clamp deleted. Line coverage is not evidence — a test can execute every line of `racing_math.py` and assert nothing about any of it.

A test never seen to fail is an assumption, not a test.

### Hard rule 2 — never weaken a test to make a suite pass

**A failing test is a finding, not an obstacle. The default response is to fix the code.**

Do not widen a tolerance, weaken an equality to a bound or a bound to a type check, drop the edge case that failed, add `skip`/`xfail`, or delete the test — and never do any of these in the same change as an edit to the code it covers.

If a test genuinely looks wrong after a spec change, **stop and flag it to me with your reasoning instead of editing it silently.** Say what the old test asserted, what the new behavior is, **where that behavior is written down** (a `docs/` section, datasheet, or message definition), and why the old behavior was *wrong* rather than merely inconvenient. "Flaky", "no longer relevant", "failing after the refactor", and "the new value is what the code does now" are not reasons.

### Anti-pattern checklist

| # | Reject a test that… |
|---|---|
| A1 | asserts on a mock's own `return_value`, or whose only assertion is `assert_called_*`. Mock only a real process boundary (socket, serial, clock, filesystem) — **never** our own ROS-free modules (`racing_math`, `protocol`, `netbind`, `drive_intent`); they were split out precisely so they can be called for real. |
| A2 | would still pass against a stub — asserts only a type or shape (`isinstance`, `len(out) == 2`), only that nothing raised, or only on a value the test itself passed straight through. |
| A3 | hardcodes today's output with no independent oracle. Every expected value must trace to a **closed form** recomputed in the test, an **invariant**, a **cited spec** (give the path), or a **recorded measurement** (name the run) — and the test must say which. An unexplained golden number is a change detector: label it `# change-detector (not an oracle)` and give the same unit a real-oracle test too. |
| A4 | covers only the happy path. Required edges: zero, negative, exactly at the boundary and one step past it, `NaN`/`inf`, empty and single-element input; parsers also need truncated, wrong-type, unknown-field, oversized, malformed-UTF-8; scan consumers also need all-`inf`, all-zero, `NaN` beams, wrong beam count, stale stamp. For driving code the specified edge behavior is nearly always "command zero / publish nothing", never "raises whatever it happens to raise". |
| A5 | asserts something trivially true, swallows exceptions (`except: pass`), uses `pytest.raises(Exception)` rather than a specific error with `match=`, uses a bare `pytest.approx` with no explicit tolerance and unit (`abs=1e-3  # 1 mm`), or hides its only assertion behind an `if`/loop filter that may match zero times and pass vacuously. |
| A6 | was loosened, skipped, or deleted because it was failing — see Hard rule 2. Every `skip`/`xfail` needs a reason naming an issue or doc section and a condition for removal; prefer `xfail(strict=True)`. |
| A7 | takes its verdict from the thing under test. Above all: gym's own collision flag is not a verdict — under the old geometry it never fired and a car drove 35.5 m through a barrier unflagged; now it fires on the padded envelope, not the real chassis. The pass/fail signal must be computed independently of the system producing the behavior. |
| A8 | tests only the permissive side of a safety check. "LB held → the car drives" is the half that fails safely; the refusal is the half that matters. |
| A9 | isn't run by the command people actually run. State which runner executes a new test, and check it is actually collected — never paper over a collection error with `--continue-on-collection-errors`. |

### Standing requirement — the LB deadman

Every node that can publish `/drive`, `/ackermann_cmd`, or `/commands/motor|servo/*` implements the deadman gate independently, so **each node needs its own four deny-path tests — another node's coverage does not transfer.** Assert the published **command**, not an internal flag:

| State | Required assertion |
|---|---|
| LB held | non-zero command published |
| LB released | zero command / no command |
| `/joy` never received | zero command / no command |
| `/joy` stale past timeout | zero command / no command |

`enable_deadman:=false` is permitted in a node test **only** when the same node construction also passes `drive_topic:=/test_only/drive` — without the remap, a test publishes live commands straight into `ackermann_mux` if the driver stack is up. Never in shipped config, and never in a test whose subject *is* the deadman.

## Package anatomy (local `ament_python` packages)

Every local package (`gap_follow`, `pure_pursuit`, `racerbot_launch`) follows the same shape — see `docs/concepts.md#anatomy-of-a-package` for the full breakdown:
```
src/<pkg>/
├── package.xml            # deps: rclpy always; sensor_msgs/ackermann_msgs/nav_msgs as needed
├── setup.py / setup.cfg   # ament_python build, registers console_script entry points
├── resource/<pkg>         # empty marker file, required by ament_python
├── <pkg>/                 # importable module; ROS-dependent node + any framework-agnostic logic split out
├── launch/<pkg>_launch.py
├── config/<pkg>.yaml      # tune behavior here, not in code — every knob is a declared ROS parameter
└── test/                  # only for logic with no rclpy dependency (see racing_math.py, protocol.py)
```
New packages go under `src/`, one per feature — don't add files into an existing package unless genuinely extending its purpose.

## Known non-obvious facts worth knowing before you touch things

- `f1tenth_system` is vendored, not a submodule, specifically to carry four committed local fixes: `joy_teleop.yaml`'s `human_control` steering axis (`axis: 3`, not upstream's `axis: 2` — this F710's right stick, not its left trigger); splitting `joy_teleop` out of `bringup_launch.py` into its own `teleop_launch.py`; `vesc_to_odom_node`'s **negative** `speed_to_erpm_gain` (`-4447.983786` after a September 2026 recalibration; the sign is the fix, the magnitude is calibration); and this car's measured geometry (`wheelbase: 0.36` in `vesc.yaml`, and `0.26 0.0 0.11` on the `base_link`->`laser` static transform in all three `*_bringup_launch.py`). It also carries one local addition, bringup starting `race_diagnostics`' `auto_record` (Addition 6, ported from car 1 2026-10-29). All four fixes get silently clobbered by a naive upstream sync, and the last two fail silently — see `docs/git-setup.md` before touching this package.
- **The car's geometry was tape-measured on 2026-08-24 and is not the Traxxas spec.** Wheelbase **0.36 m** (not 0.324), **0.30 m** outer tire to outer tire (not the 0.281 body figure), LiDAR **0.10 m behind the front axle** = **0.26 m** ahead of the rear-axle `base_link` (not 0.33). `car_width: 0.31` is that 0.30 plus 5 mm a side of deliberate padding (cut from 14.5 mm/side on 2026-08-25, after false `emergency_clearance` latches in real run logs tracked the old padding almost exactly — see `docs/hardware-reference.md`); `car_length: 0.58` is unchanged and still pads an **unmeasured** overall length. Consequences: minimum turning radius was **1.35 m** at the old 0.26 rad limit (now **0.99 m** at the controllers' 0.349 rad, **1.16 m** at the planner's 0.30 rad, since the 2026-10-16 steering calibration), and racing lines recorded under the old LiDAR offset are ~0.07 m out. Every file each number has to stay in step with is listed in `docs/hardware-reference.md#physical-dimensions-used-in-config` — change a row together or two parts of the stack model different cars.
- Servo position `0.5904` is neutral/center, not a bug (`servo_position = -0.78 * steering_angle + 0.5904`; gain and `servo_max: 0.95` measured with circles 2026-10-16, see `docs/hardware-reference.md`).
- None of the upstream f1tenth/roboracer repos have a `jazzy` branch yet; everything here is `humble`/`humble-devel` source built against Jazzy.
- Two simulators, deliberately: `tools/f1tenth_sim/run_validation.py` calls the controller *math* with no ROS at all (`docs/simulator.md`), while `racerbot_sim` puts the same F1TENTH Gym physics behind the real ROS topics so whole launch files can be validated (`docs/ros-simulator.md`). Wiring bugs are invisible to the first and are most of what has actually broken. The official ROS bridge `f1tenth_gym_ros` is still not installed.
- **The web dashboard lives in a submodule and speaks a contract with this workspace.** `src/web_dashboards` (sfu-racerbot/web-dashboards) is pinned to a commit; take a newer one per `docs/web-dashboard.md#updating-the-submodule`. Two agreements cross the repo boundary and are tested on both sides: the `live_tunable_spec` format and YAML save (`gap_follow`/`pure_pursuit` `test_*_live_tuning.py` against the submodule's `tuning.py`), and car 2's `web_dashboard_rb2.yaml` (`racerbot_launch/test/`, against the node's declared parameters and the bringup transforms). `drive_intent` stays here and is **optional** for the dashboard. The `foxglove-bridge` systemd unit runs from `src/web_dashboards/car/systemd/` (reinstall: `docs/web-dashboard.md`).
- **`racerbot_sim` must never run beside the real drivers**: `sim_joy_node` forges the LB deadman and `gym_bridge_node` publishes a second `/scan` and `/odom`. Both start blocked for a 3 s ROS discovery hold, then refuse while `vesc_driver_node`/`urg_node`/`joy` are on the graph, re-checked continuously — but do not defeat that check.
- **Runs and maps are saved into the git repo, under `data/`** (since 2026-10-25; `data/README.md`). `data/racerbot_runs/<ts>/` (`record_run.py`, `map_course` logs), `data/racerbot_auto/<ts>/` (what `auto_map_race_node` writes), `data/racerbot_sim/auto/<ts>/` (the simulator's) — every default is pinned by `racerbot_launch/test/test_run_output_paths.py`. They are committed and pushed with `tools/save_runs.sh`, so **no file in `data/` may exceed GitHub's 100 MB limit**: `record_run.py` splits bags at 50 MB (`race_diagnostics/test/test_record_run_storage.py`). Runs before that date may still sit in `~/.ros/racerbot_{runs,auto,sim}/` (all copied into `data/` on 2026-10-25). The git-tracked upstream demo maps are in `src/particle_filter/maps/`, and the manual `map_saver_cli` workflow writes into whatever CWD you ran it from. The dashboard's saved-runs panel (`map_roots` in `web_dashboard_rb2.yaml`) lists and deletes runs in `data/racerbot_auto` and `data/racerbot_sim/auto`. Being inside a git repo no longer makes a folder unmanageable (changed 2026-10-25 at the user's request, web-dashboards `mapstore.py`); instead `particle_filter/maps`, `range_libc/maps` and `racerbot_sim/tracks` are refused by name, along with anything inside them. **Never make the particle_filter maps deletable.**
- In the pinned F1TENTH Gym revision, gym's own collision check **is not trusted by anything here**. Measured under the pre-2026-08-24 geometry it never fired at all (`side_distances` computed to all zeros with the LiDAR outside the ±0.29 m box, and `range_min` 0.05 exceeds the 0.005 `ttc_threshold`) — a car drove the whole circuit through walls unflagged. The corrected 0.26 m LiDAR offset puts the sensor back inside that box, and **the flag does fire now** (measured 2026-10-20) — when padded envelopes close on each other or a wall, and its response is `state[3:] = 0`, which zeroes the car's **yaw** and turned two simulated cars round to lap backwards. `FidelityPlant` disables that response in the `plant`/`car` profiles (`tools/f1tenth_sim/README.md` #12, `tests/test_gym_collision_response.py`); `SimBridge` also disables it with `ttc_threshold = -inf` (two-run test in `src/racerbot_sim/test/test_controlled_cars.py`). Neither simulator takes its verdict from the flag: `tools/f1tenth_sim/sim_fidelity/` checks the chassis against the map distance transform and does car-to-car overlap itself (`tools/f1tenth_sim/README.md`), and `racerbot_sim` samples the body against the occupancy grid. Do not reinstate a "no collision" claim that rests on gym's own flag.
