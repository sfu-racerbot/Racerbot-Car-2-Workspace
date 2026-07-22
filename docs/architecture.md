# Architecture

How the car's software is put together: every node, every topic, and how they connect. Everything communicates over ROS2 Jazzy topics — there is no shared memory or direct function calls between packages, so this topic map *is* the system.

## The node graph

`ros2 launch f1tenth_stack bringup_launch.py` is the shared **foundation layer**: hardware drivers plus arbitration, and nothing that can drive the car on its own. It starts `joy_node`, the full VESC chain, the LiDAR, and `ackermann_mux` — but deliberately no control layer — so running it by itself brings the hardware up and then just sits there; nothing publishes to `/teleop` or `/drive` until you launch something on top of it. See "Control layers" below for what runs on top, and [operations.md](operations.md) for exact commands.

```
                                    ┌─────────────┐
                                    │  F710 pad   │
                                    └──────┬──────┘
                                           │ USB (XInput mode required)
                                           ▼
                                     ┌───────────┐
                                     │  joy_node │   (bringup_launch.py)
                                     └─────┬─────┘
                                           │ /joy  (sensor_msgs/Joy, ~15-20Hz) -- also
                                           │ read directly by every autonomy node's own
                                           │ LB deadman check, bypassing the mux entirely
                                           ▼
                                    ┌─────────────┐
                                    │  joy_teleop │   (teleop_launch.py -- a control layer)
                                    └──────┬──────┘
                                           │ /teleop  (AckermannDriveStamped)
                                           │ ALWAYS publishing while running — see "Safety model" below
                                           ▼
  /drive  ◄── control layers         ┌─────────────┐
  (AckermannDriveStamped)  ────────►│ ackermann_mux│   (bringup_launch.py)
                                    └──────┬──────┘
                                           │ /ackermann_cmd  (AckermannDriveStamped)
                                           ▼
                                 ┌─────────────────────┐
                                 │ ackermann_to_vesc_node│   (bringup_launch.py)
                                 └──────────┬───────────┘
                                            │ /commands/motor/speed (Float64)
                                            │ /commands/servo/position (Float64)
                                            ▼
                                    ┌─────────────────┐
                                    │ vesc_driver_node │◄──── serial (/dev/sensors/vesc)
                                    └────────┬─────────┘             │
                                             │                       ▼
                          /sensors/core, /sensors/imu,           ┌───────┐
                          /sensors/imu/raw,                      │ VESC  │──► drive motor
                          /sensors/servo_position_command        └───┬───┘
                                             │                       │
                                             ▼                   PPM/PWM
                                    ┌──────────────────┐             │
                                    │ vesc_to_odom_node │             ▼
                                    └────────┬──────────┘      steering servo
                                             │
                                    /odom (nav_msgs/Odometry)
                                    tf: odom → base_link


  ┌──────────────┐
  │ Hokuyo UST-10LX│──Ethernet (192.168.0.10:10940)──►┌──────────┐
  └──────────────┘                                    │ urg_node │──► /scan (sensor_msgs/LaserScan)   (bringup_launch.py)
                                                        └──────────┘

  static_transform_publisher ──► tf: base_link → laser (fixed offset: 0.27m fwd, 0.11m up)   (bringup_launch.py)
```

## Control layers: exactly one at a time, in a second terminal

`bringup_launch.py` never moves the car by itself — something has to publish into `ackermann_mux` from *outside* it, launched separately, in its own terminal, on top of an already-running bringup:

| Control layer | Command | Publishes |
|---|---|---|
| Manual driving | `ros2 launch f1tenth_stack teleop_launch.py` | `/teleop` |
| Reactive autonomy | `ros2 launch gap_follow gap_follow_launch.py` | `/drive` |
| Map-based race controller | `ros2 launch pure_pursuit pure_pursuit_launch.py` | `/drive` |
| Automatic map → race composition | `ros2 launch racerbot_launch auto_map_race_launch.py` | `/drive` (supervisor only) |
| Your own node | see [writing-your-own-node.md](writing-your-own-node.md) | `/drive` |

Run **exactly one** of these at a time — `Ctrl+C` whichever is currently running before starting a different one, rather than stacking them in additional terminals. Nothing stops you from running two at once, but that isn't "blending" them: per the priority table just below, `/teleop` always beats `/drive` while it's live, so a second control layer just gets silently masked, not mixed in.

The automatic composition is the deliberate exception *inside one launch*: gap follow publishes only to `/auto_map/drive`, pure pursuit only to `/auto_race/drive`, and `auto_map_race_node` forwards exactly one of those to the real `/drive`. Both child controllers can run without competing at the mux.

Everything below `/scan` and `/odom` is a separate kind of optional layer — not a control layer competing for the mux, but sensor processing (mapping, localization) that a control layer like `pure_pursuit` depends on:

```
/scan ──┬──► gap_follow_node ──────────────► /drive   (reactive, no map needed)
        │
        ├──► slam_toolbox (mapping mode) ──► /map, saved to a .yaml+.pgm file
        │
        └──► particle_filter (localization,  ──► /pf/viz/inferred_pose, /pf/pose/odom
              needs a saved map + /odom)          (your planner would consume these)
```

The reusable saved-map race layers one more node on top of `particle_filter`'s output — see [racing-autonomy.md](racing-autonomy.md) for the full pipeline:

```
/pf/viz/inferred_pose ──┬──► pure_pursuit_node ──► /drive   (saved-map mode)
/scan ───────────────────┘

/scan ──► gap_follow ──► /auto_map/drive ──┐
                                            ├──► auto_map_race_node ──► /drive
/map + tf ──► /slam_pose ──► pure_pursuit ─► /auto_race/drive ────────┘
                         (automatic map→race mode; one branch selected at a time)
```

`web_dashboard` is passive and layers on top of whatever's already running — it never publishes, so it isn't part of the driving path at all, just a viewer for it (see [web-dashboard.md](web-dashboard.md)):

```
/map ───────────────────┐
/scan ───────────────────┼──► web_dashboard_node ──► WebSocket ──► any browser on the network
/pf/viz/inferred_pose ───┘    (read-only; no /drive, no /joy, exempt from the deadman policy below)
```

## Topic reference

All topics as they actually appear on the bus with `bringup_launch.py` plus a control layer running (verified via `ros2 topic list` / `ros2 node info`, not just read from source):

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | `joy_node` | `joy_teleop` (if `teleop_launch.py` is running), every autonomy node's own deadman check |
| `/teleop` | `ackermann_msgs/AckermannDriveStamped` | `joy_teleop` (`teleop_launch.py`) | `ackermann_mux` |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | your autonomy node, or `auto_map_race_node` in automatic mode | `ackermann_mux` |
| `/auto_map/drive` / `/auto_race/drive` | `ackermann_msgs/AckermannDriveStamped` | gap follow / pure pursuit in automatic mode | `auto_map_race_node` only |
| `/slam_pose` | `geometry_msgs/PoseStamped` | `auto_map_race_node` from SLAM TF | pure pursuit in automatic mode |
| `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | `ackermann_mux` | `ackermann_to_vesc_node` |
| `/commands/motor/speed` | `std_msgs/Float64` | `ackermann_to_vesc_node` | `vesc_driver_node` |
| `/commands/servo/position` | `std_msgs/Float64` | `ackermann_to_vesc_node` | `vesc_driver_node` |
| `/commands/motor/{duty_cycle,current,brake,position}` | `std_msgs/Float64` | (unused by this stack — direct low-level VESC control, available if you need it) | `vesc_driver_node` |
| `/sensors/core` | `vesc_msgs/VescStateStamped` | `vesc_driver_node` | `vesc_to_odom_node` |
| `/sensors/imu`, `/sensors/imu/raw` | `sensor_msgs/Imu` | `vesc_driver_node` | (nothing by default — the VESC's onboard IMU, available if you want it) |
| `/sensors/servo_position_command` | `std_msgs/Float64` | `vesc_driver_node` | `vesc_to_odom_node` |
| `/odom` | `nav_msgs/Odometry` | `vesc_to_odom_node` | `particle_filter` (if running) |
| `/scan` | `sensor_msgs/LaserScan` | `urg_node` | `gap_follow`, `slam_toolbox`, `particle_filter` (whichever is running) |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `static_transform_publisher`, `vesc_to_odom_node` | RViz, `slam_toolbox`, `particle_filter` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `urg_node`, `ackermann_mux` | RViz / `ros2 topic echo` for debugging |

## Package reference

| Package | Where it comes from | Role |
|---|---|---|
| `f1tenth_stack` (in `f1tenth_system`) | git submodule, `humble-devel` | Owns the launch files and all the YAML configs (`vesc.yaml`, `sensors.yaml`, `mux.yaml`, `joy_teleop.yaml`) that wire everything below together |
| `vesc_driver`, `vesc_ackermann`, `vesc_msgs` (in `vesc`) | git submodule (nested inside `f1tenth_system`), `humble` | Talks to the VESC over serial; converts between Ackermann drive commands and raw VESC motor/servo commands |
| `serial_driver`, `io_context` (in `transport_drivers`) | git submodule, `humble` | Low-level serial port library the VESC driver is built on |
| `urg_node` | apt (`ros-jazzy-urg-node`) | Hokuyo LiDAR driver |
| `joy`, `joy_teleop`, `teleop_tools` | apt / submodule | Gamepad input and teleop mapping |
| `ackermann_mux` (in `f1tenth_system`) | git submodule, `humble-devel` | Arbitrates between teleop and autonomy commands — see safety model below |
| `particle_filter`, `range_libc` | git submodules, `humble-devel` | Monte Carlo localization against a saved map |
| `slam_toolbox` | apt (`ros-jazzy-slam-toolbox`) | Builds a map during manual or autonomous course discovery; remains online for the automatic race |
| `gap_follow` | local, `src/gap_follow` | Baseline reactive autonomy (follow-the-gap) — see [writing-your-own-node.md](writing-your-own-node.md), this package *is* the worked example |
| `pure_pursuit` | local, `src/pure_pursuit` | Race controller, record/profile tools, and automatic map-to-race supervisor — see [racing-autonomy.md](racing-autonomy.md) |
| `web_dashboard` | local, `src/web_dashboard` | Read-only live browser dashboard of the map/scan/pose over a WebSocket — never publishes anything, not subject to the deadman policy below — see [web-dashboard.md](web-dashboard.md) |
| `racerbot_launch` | local, `src/racerbot_launch` | Top-level SLAM, automatic map-to-race, and saved-map race launches |

## The safety model (read this before writing autonomy code)

`ackermann_mux` picks between two input channels and publishes the winner to `/ackermann_cmd`:

```yaml
joystick:   topic: teleop, priority: 100, timeout: 0.2s
navigation: topic: drive,  priority: 10,  timeout: 0.2s
```

Higher priority wins, *as long as that channel hasn't gone silent for more than its timeout*. The intent is straightforward: a human on the joystick should always be able to override autonomy.

**Important, verified behavior:** `joy_teleop`'s `default` profile (in `joy_teleop.yaml`) has no deadman-button restriction — whenever `teleop_launch.py` is running, it unconditionally and continuously publishes a neutral `(steering=0, speed=0)` command to `/teleop`, whether or not LB is held. This means `/teleop` **never times out** as long as `teleop_launch.py` is running, so it **always** wins arbitration over `/drive` — even when nobody is touching the controller.

This was directly confirmed by test: with `bringup_launch.py` and `teleop_launch.py` both running and LB *not* held, publishing a distinct, continuous command to `/drive` had zero effect on `/ackermann_cmd`, which stayed locked at `0.0 / 0.0` the whole time.

**Practical consequence: your autonomy node's `/drive` commands will never reach the VESC while `teleop_launch.py` is also running.** This is exactly why `bringup_launch.py` doesn't start `joy_teleop` itself — running autonomy means simply *not launching* `teleop_launch.py` in the first place, rather than starting it and then stopping it (see [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node)). There's an unused hook for a cleaner handoff in `joy_teleop.yaml` — an `autonomous_control` profile bound to the RB button — but it currently just publishes an `Int8` to `/dev/null` (a no-op placeholder), it isn't wired to anything. If your team wants a "flip RB to hand off to autonomy" workflow where both control layers run at once, that's the place to build it — it doesn't exist yet.

### Workspace policy: the LB deadman button is mandatory for every node that can move the car

**Current, standing policy — read this before running or writing any driving code.** Regardless of the `ackermann_mux` arbitration above, no code in this workspace — autonomous or not — is allowed to move the car unless the driver is actively holding **LB** on the physical controller. Manual teleop already works this way (`joy_teleop`'s `human_control` profile is deadman-gated). **This policy stays in force, unrelaxed, until the team has explicitly confirmed the car's behavior is trustworthy enough to change it.**

All three moving/command-selecting nodes enforce this in code: `gap_follow_node`, `pure_pursuit_node`, and `auto_map_race_node` each subscribe to `/joy` directly and refuse to publish a non-zero drive command unless button index `deadman_button` (default `4`, i.e. LB) is currently held on a live `/joy` stream (`joy_timeout_sec`, default `0.5s`) — checked *first*, ahead of every other watchdog that node has. This is a **second, independent safety layer on top of** the mux arbitration above: even when no control layer is publishing to `/teleop` at all (because `teleop_launch.py` was simply never started — see [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node)), the autonomy node itself still won't drive without LB held. Concretely, this means **`joy_node` must always be up** for any autonomy node to drive at all — with LB held — which is exactly why it lives in `bringup_launch.py` (the shared foundation) rather than `teleop_launch.py` (the manual-driving control layer).

Each node exposes this as an `enable_deadman` parameter (default `true` in all three configs). **Do not set it to `false` on any node** — that would be a unilateral decision to bypass the current policy, not just a tuning change. **Any new autonomy node added to this workspace must implement the same check before it's allowed to drive the car** — see [writing-your-own-node.md](writing-your-own-node.md#the-interface-contract) for the required pattern.

## Frame conventions

- `base_link`: origin of the car, at the rear axle (matches `wheelbase: 0.25m` in `vesc.yaml`, used by `vesc_to_odom_node` for odometry)
- `laser`: the Hokuyo's frame, offset `+0.27m` forward / `+0.11m` up from `base_link` (fixed, via `static_transform_publisher`)
- `odom`: continuous but drifting frame, published by `vesc_to_odom_node` from wheel-speed + servo-angle integration (no encoders/IMU fusion — this is dead-reckoning only)
- `map`: only exists once `slam_toolbox` or `particle_filter`'s `map_server` is running

Note `slam_toolbox`'s configured `base_frame` is `laser`, not `base_link` (see `f1tenth_stack/config/f1tenth_online_async.yaml`) — a deliberate upstream choice, not a typo, but worth knowing if you're debugging a `tf` tree.
