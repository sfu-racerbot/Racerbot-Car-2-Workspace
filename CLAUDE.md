# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS2 Jazzy workspace for a team's roboracer/F1TENTH car (Jetson Orin Nano Super, JetPack 7.2, Ubuntu 24.04). This is a physical robot that can hurt itself, people, or property if driving code is wrong — treat any change to a node that can publish `/drive` with corresponding care (see "Safety model" below).

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

Run a package's standalone (non-ROS) unit tests directly, no sourcing/build required:
```bash
python3 -m pytest src/pure_pursuit/test/ -v
python3 -m pytest src/web_dashboard/test/ -v
```
These test framework-agnostic logic pulled out of the ROS nodes (`pure_pursuit/racing_math.py`, `web_dashboard/protocol.py`) — the pattern to follow for any new package with non-trivial math/parsing: keep it importable without `rclpy`.

Drive the car (manual):
```bash
ros2 launch f1tenth_stack bringup_launch.py     # terminal 1: hardware + arbitration, never moves the car alone
ros2 launch f1tenth_stack teleop_launch.py       # terminal 2: control layer, hold LB, sticks = drive
```
Full command reference for every workflow (mapping, localization, autonomy, racing, shutdown) is in `docs/operations.md`.

## Architecture

Everything communicates over ROS2 topics only — no shared memory, no direct function calls between packages. The topic graph *is* the system; full diagram and topic table in `docs/architecture.md`.

**Two-tier launch pattern, always:**
1. `bringup_launch.py` (`f1tenth_stack`) — the shared foundation, started once: `joy_node`, VESC chain, LiDAR (`urg_node`), `ackermann_mux`. Deliberately starts nothing that can move the car by itself.
2. Exactly one **control layer** on top, in a separate terminal: `teleop_launch.py` (manual), `gap_follow_launch.py` (reactive autonomy), `pure_pursuit_launch.py` (map-based race controller), or a new node. Each publishes `AckermannDriveStamped` to either `/teleop` or `/drive`; `ackermann_mux` arbitrates.

**Arbitration:** `/teleop` (priority 100) always beats `/drive` (priority 10) while `/teleop` hasn't timed out. Verified behavior: `joy_teleop`'s default profile publishes continuously even when LB isn't held, so **`/teleop` never times out while `teleop_launch.py` is running** — meaning autonomy's `/drive` commands never reach the VESC if `teleop_launch.py` is also up. Running autonomy means simply not launching `teleop_launch.py`, not starting-then-stopping it.

**Packages** (`src/`):
| Package | Kind | Role |
|---|---|---|
| `f1tenth_system` (+ `ackermann_mux`, `teleop_tools`, `vesc`) | vendored, plain tracked files (not a submodule — see `docs/git-setup.md`) | VESC driver, `urg_node`, joystick teleop, command muxing; owns all launch files + YAML config |
| `transport_drivers` | git submodule (`humble`) | serial transport dep for `vesc` |
| `particle_filter` (+ `range_libc`) | git submodules (`humble-devel`) | Monte Carlo localization against a saved map |
| `gap_follow` | local | reactive autonomy, follow-the-gap on `/scan` → `/drive`, no map — the reference template for new driving nodes |
| `pure_pursuit` | local | map-based race controller: recorded+paced racing line, pure pursuit control, reactive safety net, opponent overtaking — see `docs/racing-autonomy.md` |
| `web_dashboard` | local | read-only browser dashboard over WebSocket — subscribes only, never publishes, not subject to the safety policy below |
| `usb_cam_stream` | local | MJPEG webcam stream over plain HTTP |
| `racerbot_launch` | local | launch glue not owned by any single driver repo (SLAM, race-day localization+pure_pursuit combos) |

`slam_toolbox` is apt-installed (`ros-jazzy-slam-toolbox`), not vendored.

## Safety model — read before writing or running any driving code

**Workspace policy, currently in force, do not relax unilaterally:** every node that can move the car requires the driver to be actively holding **LB** on the physical F710 controller (XInput mode), on top of `ackermann_mux` arbitration. This is enforced in code, independently, by every autonomy node — `gap_follow_node` and `pure_pursuit_node` each subscribe to `/joy` directly and refuse to publish non-zero drive commands without a live LB hold (`enable_deadman: true` default in both configs). **Never set `enable_deadman: false`** — that's a unilateral policy change, not a tuning knob. Full reasoning: `docs/architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car`.

**Any new node that publishes to `/drive`, `/ackermann_cmd`, or `/commands/motor|servo/*` must implement the same LB deadman check** — copy-paste pattern from `gap_follow_node.py`'s `joy_callback`/`_deadman_engaged`, documented in full in `docs/writing-your-own-node.md`. Decide which category new code falls into (driving vs. support/tooling) using `docs/adding-your-own-code.md` — when in doubt, treat it as driving code.

Test order for any new driving node, never skip ahead: static topic check (no driver stack running) → wheels off the ground (full stack + LB held) → floor, low speed, open space. See `docs/writing-your-own-node.md#testing-before-its-on-wheels`.

## Package anatomy (local `ament_python` packages)

Every local package (`gap_follow`, `pure_pursuit`, `web_dashboard`, `racerbot_launch`) follows the same shape — see `docs/concepts.md#anatomy-of-a-package` for the full breakdown:
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

- `f1tenth_system` is vendored, not a submodule, specifically to carry two committed local fixes: `joy_teleop.yaml`'s `human_control` steering axis (`axis: 3`, not upstream's `axis: 2` — this F710's right stick, not its left trigger), and splitting `joy_teleop` out of `bringup_launch.py` into its own `teleop_launch.py`. Both get silently clobbered by a naive upstream sync — see `docs/git-setup.md` before touching this package.
- Servo position `0.5304` is neutral/center, not a bug (`servo_position = -1.2135 * steering_angle + 0.5304`, see `docs/hardware-reference.md`).
- None of the upstream f1tenth/roboracer repos have a `jazzy` branch yet; everything here is `humble`/`humble-devel` source built against Jazzy.
- Simulator (`f1tenth_gym_ros`) is intentionally not installed here; sim testing happens on a separate machine.
