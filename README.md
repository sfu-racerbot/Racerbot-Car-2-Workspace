# racerbot-ws

> **Who this is for:** anyone landing on this repo — new team members included. No ROS2 or robotics experience assumed.
> **Read first:** nothing. For a guided path into the docs, go to [docs/README.md](docs/README.md).
> **What's in it:** what this repo is, what's in `src/`, and how to build and drive the car.

ROS2 Jazzy workspace for the team's roboracer/F1TENTH car (Jetson Orin Nano Super, JetPack 7.2, Ubuntu 24.04).

Recent changes to the team's own packages are logged in [CHANGELOG.md](CHANGELOG.md) — check it after pulling to see what changed and whether anything still needs on-car validation.

## Code provenance

The team's actively developed codebases live in the [`racerbot_a`](src/racerbot_a) and [`racerbot_b`](src/racerbot_b) git submodules. The workspace-specific integration code and documentation outside those two team repositories were produced through vibe coding (AI-assisted development). Third-party dependencies are separately identified as upstream submodules or vendored upstream code below.

## Documentation

**New here? Start with [docs/README.md](docs/README.md)** — it has a numbered reading order for newcomers and lists every doc grouped by what you're trying to do.

The three you'll want first:

| Doc | What's in it |
|---|---|
| [docs/glossary.md](docs/glossary.md) | Every term the docs use — node, topic, mux, deadman, SLAM — defined for someone new to robotics |
| [docs/concepts.md](docs/concepts.md) | What `ros2 launch`, `colcon build` and `source` actually do, and what each folder is for |
| [docs/architecture.md](docs/architecture.md) | The node/topic graph and the safety model — **read before writing any driving code** |

Full index, including the deep-dive and reference docs: **[docs/README.md](docs/README.md)**.

## Layout (`src/`)
| Package | Source | Purpose |
|---|---|---|
| `racerbot_a` | [team git submodule](https://github.com/sfu-racerbot/racerbot_a), `main` | Racerbot Team A's actively developed codebase; its ROS2 packages are under `racerbot_a/src/` |
| `racerbot_b` | [team git submodule](https://github.com/sfu-racerbot/racerbot_b), `main` | Racerbot Team B's actively developed codebase, currently containing `gap_follow_node` |
| `f1tenth_system` (+ `ackermann_mux`, `teleop_tools`, `vesc`) | vendored (plain tracked files, **not** a git submodule — see [docs/git-setup.md](docs/git-setup.md)) | VESC driver, `urg_node` (Hokuyo), joystick teleop, command muxing |
| `transport_drivers` | git submodule, `humble` | serial transport dependency for `vesc` |
| `particle_filter` (+ `range_libc`) | git submodules, `humble-devel` | Monte Carlo localization against a saved map |
| `realsense-ros` | git submodule, `ros2-master` (natively supports Jazzy, unlike the submodules above) | Intel RealSense D435i driver — color/depth over ROS2. Detail: [docs/realsense-camera.md](docs/realsense-camera.md) |
| `gap_follow` | local package | baseline reactive autonomy — follow-the-gap on `/scan` → `/drive`, no map needed. Code/algorithm detail: [src/gap_follow/README.md](src/gap_follow/README.md) |
| `pure_pursuit` | local package | map-based race controller — pure pursuit over a curvature-paced recorded racing line, plus the tools to record and pace one. Pipeline/workflow: [docs/racing-autonomy.md](docs/racing-autonomy.md); code/math detail: [src/pure_pursuit/README.md](src/pure_pursuit/README.md) |
| `race_diagnostics` | local package | read-only run recorder and post-run analyzer: pipeline health, localization lag, watchdog stops, rosbag. Not an autonomy node, safe alongside anything. Workflow: [docs/run-diagnostics.md](docs/run-diagnostics.md); code detail: [src/race_diagnostics/README.md](src/race_diagnostics/README.md) |
| `drive_intent` | local package | shared schema and trajectory prediction for `/drive_intent` — what a driving algorithm is trying to do, and why. Pure Python (no `rclpy`), plus a single-header C++ port for teammates' codebases. Detail: [docs/drive-intent.md](docs/drive-intent.md) |
| `web_dashboard` | local package | live browser dashboard of the map/LIDAR/pose over a WebSocket, plus a panel for tuning the driving nodes' parameters live — not an autonomy node, publishes to no topic, safe to run alongside anything else. Workflow: [docs/web-dashboard.md](docs/web-dashboard.md); code detail: [src/web_dashboard/README.md](src/web_dashboard/README.md) |
| `racerbot_launch` | local package | launch glue not owned by any single driver repo (SLAM, one-command autonomous map→race, saved-map race-day localization, and cameras) |
| `racerbot_sim` | local package | F1TENTH Gym behind the car's own topics: stands in for the LiDAR, VESC and joystick so the real driving stack runs unchanged above it. Refuses to publish while the real drivers are on the graph. Detail: [docs/ros-simulator.md](docs/ros-simulator.md) |
| `usb_cam_stream` | local package | live MJPEG video stream from a USB webcam, served over plain HTTP for viewing in any browser. Detail: [docs/usb-camera-livestream.md](docs/usb-camera-livestream.md), [src/usb_cam_stream/README.md](src/usb_cam_stream/README.md) |

`slam_toolbox` is installed system-wide via apt (`ros-jazzy-slam-toolbox`), not vendored in `src/`.

## Quick start

```bash
source /opt/ros/jazzy/setup.bash
cd ~/racerbot-ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
On the Orin Nano's 8GB RAM, prefer `colcon build --symlink-install --parallel-workers 1` if you hit OOM during a full rebuild. Every new shell needs both `source` lines above, in that order, before any `ros2`/`colcon` command.

Drive it:
```bash
ros2 launch f1tenth_stack bringup_launch.py
# in another terminal:
ros2 launch f1tenth_stack teleop_launch.py
```
Hold **LB** on the F710 (must be in **XInput mode**), left stick = speed, right stick = steering. The car will not move on its own from the first command alone — see [docs/architecture.md](docs/architecture.md#the-node-graph) for why driving needs a second control-layer launch on top of the shared bringup.

For mapping, localization, running `gap_follow` or your own autonomy code, and every other workflow: see [docs/operations.md](docs/operations.md).

## One-time setup (already done on this machine)
- ROS2 Jazzy + dev tools installed, `rosdep` initialized.
- `racerbotcar-2` and `racermember-2` added to the `dialout` group (VESC serial access) and `input` group (joystick device access). **Group membership only applies to sessions started after it was added** — open a fresh terminal if you hit a permission error on `/dev/sensors/vesc` or `/dev/input/js0`.
- `racermember-2` has ACL access scoped to this workspace only — see `getfacl racerbot-ws`.

## Notes
- **Current safety policy:** every autonomy node in this workspace (`gap_follow`, `pure_pursuit`, and any new one) requires the driver to hold **LB** on the physical controller for the car to move, on top of the usual `ackermann_mux` arbitration — see [docs/architecture.md](docs/architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car). This stays in force until the team explicitly confirms the car's behavior is trustworthy enough to relax it — don't set any node's `enable_deadman` parameter to `false` unilaterally.
- The official `f1tenth`/roboracer driver repos don't have a `jazzy` branch yet; everything here is the `humble-devel`/`humble` source built against ROS2 Jazzy. If a future `rosdep update`/dependency bump breaks the build, check each package's upstream (submodule or vendored — see [docs/git-setup.md](docs/git-setup.md)) for a newer ROS2-distro branch before patching locally.
- F1TENTH Gym is installed reproducibly under the ignored `.sim/` directory by `tools/f1tenth_sim/setup.sh`; see [docs/simulator.md](docs/simulator.md). It does not modify the system Python environment.
- `src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml`'s `human_control` profile was patched locally: upstream ships `drive-steering_angle` mapped to `axis: 2` (this F710's left trigger, not the right stick). Changed to `axis: 3`. `f1tenth_system` is vendored (not a submodule) specifically so this fix could be committed — see [docs/git-setup.md](docs/git-setup.md) before pulling in any upstream changes to it.
