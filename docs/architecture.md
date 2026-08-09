# Architecture

> **Who this is for:** anyone about to write or run code that can move the car. **Required reading before driving code.**
> **Read first:** [concepts.md](concepts.md) for what nodes and topics are, and [glossary.md](glossary.md) for the vocabulary.
> **You'll be able to:** read the node/topic graph, explain why driving needs two launch files, and state the safety rules and their reasons.
> **Time:** about 30 minutes.

How the car's software is put together: every [node](glossary.md#node), every [topic](glossary.md#topic), and how they connect.

Two words to have straight before you start, both covered in [concepts.md](concepts.md#first-what-is-ros2). A **node** is one running program that does one job. A **topic** is a named channel that nodes send messages over — one node publishes, any number listen.

Everything here communicates over ROS2 Jazzy topics. There is no shared memory, and no direct function calls between one [package](glossary.md#package) and another.

That has a consequence worth sitting with: **this topic map *is* the system.** There is no other place the overall behavior is written down. No single file you can read to find out what the car does.

---

## The 60-second version

If you read nothing else on this page, read this.

**1. Commands flow one way, through a single gate.** Something decides where to steer, publishes that decision to a topic, and a component called `ackermann_mux` picks *one* of those decisions and passes it to the motors. Everything else is detail.

**2. Starting the car takes two commands, not one.** The first (`bringup_launch.py`) turns on the hardware and deliberately cannot move the car. The second decides *how* it's driven — by hand, or by an autonomy node. This split is a safety feature.

**3. When two things both want to steer, the human wins.** Manual teleop has priority 100; autonomy has priority 10. The human always overrides.

**4. Nothing moves unless a person is holding the LB button.** This is a workspace policy, enforced separately inside every node that can drive, on top of everything above. It is not optional and not a tuning knob. [Full rule below.](#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)

Everything on the rest of this page elaborates those four points.

---

## The node graph

`ros2 launch f1tenth_stack bringup_launch.py` is the shared **foundation layer**: hardware drivers plus arbitration, and nothing that can drive the car on its own.

It starts `joy_node` (the gamepad reader), the full VESC chain (the motor controller), the LiDAR, and `ackermann_mux` — but deliberately no control layer.

The consequence is worth stating plainly, because people expect otherwise: **run the bringup by itself and the car sits there.** The hardware powers up, the LiDAR spins, and nothing happens, because nothing publishes to `/teleop` or `/drive` until you launch something on top of it.

See [Control layers](#control-layers-exactly-one-at-a-time-in-a-second-terminal) below for what runs on top, and [operations.md](operations.md) for exact commands.

### How to read the diagram

Boxes are nodes (running programs). Arrows are topics (named channels carrying messages). The label on an arrow is the topic name and its message type.

Read it top to bottom: a human input at the top, a spinning wheel at the bottom. Every stage in between is a separate program that could be swapped out without the others noticing.

Two arrows are labelled `tf:` rather than with a topic name. [TF](glossary.md#tf--transform--frame) is ROS2's bookkeeping for "where things are relative to each other" — it's how the system knows the LiDAR sits 33 cm in front of the car's centre. There's more on frames [at the end of this page](#frame-conventions).

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

  static_transform_publisher ──► tf: base_link → laser (estimated offset: 0.33m fwd, 0.11m up)   (bringup_launch.py)
```

**The chain in words:**

1. The gamepad produces `/joy`.
2. If manual teleop is running, `joy_teleop` turns that into a steering-and-speed command on `/teleop`.
3. `ackermann_mux` chooses between `/teleop` and `/drive`, and emits the winner on `/ackermann_cmd`.
4. `ackermann_to_vesc_node` converts that into raw motor and servo numbers.
5. `vesc_driver_node` sends those down a serial cable to the VESC, which spins the motor and moves the steering servo.

Two things happen off to the side of that chain. `vesc_to_odom_node` reads what the VESC reports back and estimates how far the car has travelled, publishing `/odom`.

And the LiDAR publishes `/scan` over Ethernet, independently of everything above — it doesn't care whether anything is driving.

---

## Control layers: exactly one at a time, in a second terminal

`bringup_launch.py` never moves the car by itself. Something has to [publish](glossary.md#publish--subscribe) into `ackermann_mux` from *outside* it — that is, send messages to the mux — launched separately, in its own terminal, on top of an already-running bringup.

| Control layer | Command | Publishes |
|---|---|---|
| Manual driving | `ros2 launch f1tenth_stack teleop_launch.py` | `/teleop` |
| Reactive autonomy | `ros2 launch gap_follow gap_follow_launch.py` | `/drive` |
| Map-based race controller | `ros2 launch pure_pursuit pure_pursuit_launch.py` | `/drive` |
| Automatic map → race composition | `ros2 launch racerbot_launch auto_map_race_launch.py` | `/drive` (supervisor only) |
| Your own node | see [writing-your-own-node.md](writing-your-own-node.md) | `/drive` |

**Run exactly one of these at a time.** `Ctrl+C` whichever is currently running before starting a different one, rather than stacking them in additional terminals.

Nothing stops you from running two at once — but that isn't "blending" them. Per the priority table [below](#the-safety-model-read-this-before-writing-autonomy-code), `/teleop` always beats `/drive` while it's live, so a second control layer just gets silently masked. You get no error, no warning, and no clue: the car simply ignores the code you thought you were testing.

The whole automatic composition can be run without the car — the real launch file, the real SLAM, the real dashboard, over a simulated LiDAR and VESC. See [ros-simulator.md](ros-simulator.md).

### The one deliberate exception

The automatic composition runs two controllers at once, *inside one launch*, and it gets away with it by keeping both off `/drive`:

- gap follow publishes only to `/auto_map/drive`
- [pure pursuit](glossary.md#pure-pursuit) — the map-based racing controller — publishes only to `/auto_race/drive`
- `auto_map_race_node` forwards exactly one of those to the real `/drive`

So both child controllers run without ever competing at the [mux](glossary.md#mux--multiplexer), the component that picks which command wins. Only the supervisor talks to `/drive`.

If you ever need two driving algorithms alive simultaneously, this is the pattern: a supervisor that picks, not two publishers that race.

### `gap_follow` and corridor centering: two questions, not one

[Follow-the-gap](glossary.md#follow-the-gap) — the strategy of steering toward the largest open space the laser can see — answers exactly one question: *which way should the car point*. It answers it well.

It never answers the second question: *where across the corridor should the car be*.

Those two come apart on a straight. The aim there is the deepest beam in the [scan](glossary.md#scan), the array of distance readings the LiDAR produces. On a straight, that deepest beam runs parallel to both walls.

So the steering law reports zero error no matter how far off-centre the car actually is.

The practical failure: enter a straight 15 cm off the left wall and the car tracks 15 cm off the left wall for the whole length of it. It spends its entire clearance budget for nothing, and starts the next corner from the worst available place.

`gap_logic.corridor_centering_bias` supplies the missing half — a small steering bias proportional to how far off the middle of the corridor the car sits.

**Three properties make this safe to add underneath an obstacle-avoidance pipeline:**

- **It is bounded.** `centering_max_steering` (0.08 rad of the 0.26 rad the rack has) caps it, and `gap_follow_node` refuses to start if that is set above half the steering limit.
- **It fades, never switches.** Full weight below a 4° aim bearing, zero above 15°, linear between — silent through a corner, where the fast line is deliberately *not* the middle.
- **It needs two real walls.** Both sides must return a wall within `centering_zero_side_distance`. An opening on one side is not something to centre against.

Every existing safety layer is untouched and still runs first. Contact clearance, the forward-cone brake, and TTC all decide before steering is computed.

The bias is then applied before the node's existing steering clip and slew limiter, so it can never exceed the limits those enforce.

<details>
<summary><b>Why it's built this way, and what it measured</b> — the control-theory framing, the reasoning behind each bound, and the validation numbers. Skip unless you're tuning centering or building something similar.</summary>

**The control structure.** Together with the existing bearing term, the centering bias is the standard two-state lane-centring law: cross-track error plus heading error.

That's the same structure as Stanley control and the classic F1TENTH wall-follower — this is a well-trodden result, not something invented here.

The bearing term is what damps it: as the car turns toward the middle its heading tilts, the aim bearing swings the other way, and the two oppose. So there is no derivative term and nothing to wind up.

**Why bounded.** Centering *refines* the gap the avoidance pipeline chose. A bias big enough to cancel that choice would be a second, unreviewed driving policy — a different algorithm wearing the same node's name. Hence the hard cap, and the refusal to start above half the steering limit rather than a silent clamp.

**Why fading rather than switching.** A hard on/off on a steering term is exactly what produced the scan-rate steering chatter documented in `gap_logic.aim_within_gap`. Any term that engages and disengages abruptly can oscillate against the term it's correcting.

**Why two walls are required.** Without the both-sides check, the bias would steer *into* a doorway or side opening — the geometry says "the corridor got wider on the left, move left". It is also suppressed entirely while the car is creeping out of the forward reserve.

**Measured result.** In the F1TENTH Gym harness, mean off-centre distance fell **18–35% across the three validation tracks, with no collisions** — see [simulator.md](simulator.md). Note the caveat in that doc about what "no collisions" does and doesn't prove in the pinned Gym revision.

Tuning lives in `gap_follow.yaml`, which documents every knob.

</details>

### Sensor processing is a different kind of layer

Everything below `/scan` and `/odom` is optional in a different sense. These aren't control layers competing for the mux — they're sensor processing (mapping, localization) that a control layer like `pure_pursuit` depends on:

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
/scan ──────────────────┤
/odom ──────────────────┘   (measured speed, sizes the adaptive lookahead)

/scan ──► gap_follow ──► /auto_map/drive ──┐
                                            ├──► auto_map_race_node ──► /drive
/map + tf ──► /slam_pose ──► pure_pursuit ─► /auto_race/drive ────────┘
                         (automatic map→race mode; one branch selected at a time)
```

### The dashboard sits outside the driving path entirely

`web_dashboard` layers on top of whatever's already running and publishes to no topic, so it isn't part of the driving path at all (see [web-dashboard.md](web-dashboard.md)):

```
/map ───────────────────┐
/scan ───────────────────┼──► web_dashboard_node ──► WebSocket ──► any browser on the network
/pf/viz/inferred_pose ───┘    (no /drive publisher, so exempt from the deadman policy below)
                              │
                              └─ set_parameters service ──► pure_pursuit_node / gap_follow_node
                                 (live tuning: changes how a driving node behaves,
                                  never whether it drives — see below)
```

Live tuning is the one path from the dashboard back to the car, and it is deliberately not a driving path.

It cannot publish a command, cannot start the car, and cannot relax the deadman — `enable_deadman` is refused at runtime by every node that has it. What it changes is *tuning*, on a car that is already being driven by an autonomy node with LB held.

Each driving node enforces its own hard bounds on every such change, in its own process, so the browser is never the authority on what is safe. Full reasoning in [web-dashboard.md](web-dashboard.md#live-parameter-tuning).

---

## Topic reference

These are all topics as they actually appear on the bus with `bringup_launch.py` plus a control layer running — verified via `ros2 topic list` and `ros2 node info`, not just read from source.

**If you're new, five of these matter and the rest are detail:**

| Topic | What it tells you |
|---|---|
| `/joy` | what the gamepad is doing |
| `/scan` | what the LiDAR sees |
| `/drive` | what autonomy wants |
| `/teleop` | what the human wants |
| `/ackermann_cmd` | what actually won, and is being sent to the motors |

Comparing the last three is how you diagnose "why isn't the car doing what my code says".

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
| `/odom` | `nav_msgs/Odometry` | `vesc_to_odom_node` | `gap_follow` (TTC speed), `pure_pursuit` (lookahead sizing only, never a stop watchdog), `particle_filter` (if running) |
| `/scan` | `sensor_msgs/LaserScan` | `urg_node` | `gap_follow`, `slam_toolbox`, `particle_filter` (whichever is running) |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `static_transform_publisher`, `vesc_to_odom_node` | RViz, `slam_toolbox`, `particle_filter` |
| `/drive_intent` | `std_msgs/String` (JSON) | `gap_follow`, `pure_pursuit` (read-only diagnostics — never a control path) | `web_dashboard` — see [drive-intent.md](drive-intent.md) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `urg_node`, `ackermann_mux` | RViz / `ros2 topic echo` for debugging |

---

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
| `web_dashboard` | local, `src/web_dashboard` | Live browser dashboard of the map/scan/pose over a WebSocket, plus a live tuning panel for the driving nodes' parameters. Publishes to no topic and cannot move the car, so it isn't subject to the deadman policy below — see [web-dashboard.md](web-dashboard.md) |
| `racerbot_launch` | local, `src/racerbot_launch` | Top-level SLAM, automatic map-to-race, and saved-map race launches |

---

## The safety model (read this before writing autonomy code)

`ackermann_mux` picks between two input channels and publishes the winner to `/ackermann_cmd`:

```yaml
joystick:   topic: teleop, priority: 100, timeout: 0.2s
navigation: topic: drive,  priority: 10,  timeout: 0.2s
```

Higher priority wins, *as long as that channel hasn't gone silent for more than its timeout*.

The intent is straightforward: **a human on the joystick should always be able to override autonomy.** If autonomy is doing something alarming, grabbing the controller takes over. That's the design goal, and it works.

### The consequence people get wrong

**Verified behavior:** `joy_teleop`'s `default` profile (in `joy_teleop.yaml`) has no deadman-button restriction. Whenever `teleop_launch.py` is running, it unconditionally and continuously publishes a neutral `(steering=0, speed=0)` command to `/teleop` — whether or not LB is held.

So `/teleop` **never times out** while `teleop_launch.py` is running. And since it never times out, it **always** wins arbitration over `/drive` — even when nobody is touching the controller.

This was directly confirmed by test. With `bringup_launch.py` and `teleop_launch.py` both running and LB *not* held, a distinct, continuous command was published to `/drive`.

It had zero effect. `/ackermann_cmd` stayed locked at `0.0 / 0.0` the whole time.

> **Practical consequence: your autonomy node's `/drive` commands will never reach the VESC while `teleop_launch.py` is also running.**
>
> Your node runs. It publishes. It logs. Nothing moves, and nothing tells you why.

This is exactly why `bringup_launch.py` doesn't start `joy_teleop` itself.

**Running autonomy means simply *not launching* `teleop_launch.py` in the first place** — not starting it and then stopping it. See [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node).

<details>
<summary><b>The unused handoff hook</b> — there's a half-built "flip a button to hand off to autonomy" path in the config. Read this only if you want to build that workflow.</summary>

`joy_teleop.yaml` contains an `autonomous_control` profile bound to the RB button. It currently publishes an `Int8` to `/dev/null` — a no-op placeholder. It is not wired to anything.

If your team wants a workflow where both control layers run at once and RB hands control between them, that profile is the place to build it. **It does not exist yet**, so don't write code that assumes it does.

</details>

### Workspace policy: the LB deadman button is mandatory for every node that can move the car

**Current, standing policy. Read this before running or writing any driving code.**

> **No code in this workspace — autonomous or not — may move the car unless the driver is actively holding LB on the physical controller.**
>
> Let go of LB and the car stops. That is the guarantee, and it holds regardless of what the software is doing or believes.

This is *on top of* the `ackermann_mux` arbitration above, not instead of it. Manual teleop already worked this way (`joy_teleop`'s `human_control` profile is deadman-gated); the policy extends the same rule to everything else.

**This policy stays in force, unrelaxed, until the team has explicitly confirmed the car's behavior is trustworthy enough to change it.**

**Why a second layer at all?** Because mux arbitration only protects you when teleop is running. Run autonomy the correct way — without `teleop_launch.py` — and there is nothing on `/teleop` to override with. The deadman is what covers that gap. Without it, "the car is driving itself and I can't stop it" is a real state the system can reach.

**How it's enforced.** All three moving/command-selecting nodes implement the check in code: `gap_follow_node`, `pure_pursuit_node`, and `auto_map_race_node`.

Each one subscribes to `/joy` directly and refuses to publish a non-zero drive command unless:

- button index `deadman_button` (default `4`, i.e. LB) is currently held, and
- the `/joy` stream is live (`joy_timeout_sec`, default `0.5s`)

That check runs **first**, ahead of every other watchdog the node has.

Note the dependency this creates: **`joy_node` must always be up** for any autonomy node to drive at all. That's exactly why it lives in `bringup_launch.py` (the shared foundation) rather than `teleop_launch.py` (the manual-driving control layer).

**The two rules that follow from this:**

1. **Never set `enable_deadman` to `false`.** It's exposed as a parameter (default `true` in all three configs), but changing it is a unilateral decision to bypass a standing team policy — not a tuning change. If you think it should be relaxed, that's a conversation with the team, not an edit.

2. **Any new node that can move the car must implement the same check.** This applies to anything publishing to `/drive`, `/ackermann_cmd`, or `/commands/motor|servo/*`. The required pattern is in [writing-your-own-node.md](writing-your-own-node.md#the-interface-contract), and `gap_follow_node` is the reference implementation to copy from.

---

## Frame conventions

A **frame** is a named coordinate system — an agreed answer to "position relative to what?". The LiDAR reports distances from where the LiDAR is; odometry reports position from where the car started. Those are different origins, so ROS2 tracks the relationships between them (the "TF tree") and converts between them for you.

The four frames on this car:

| Frame | Where it is | Notes |
|---|---|---|
| `base_link` | Origin of the car, at the rear axle | Matches the Traxxas 74276-4 `wheelbase: 0.324m` in `vesc.yaml`, used by `vesc_to_odom_node` for odometry |
| `laser` | The Hokuyo's position | Estimated at `+0.33m` forward and `+0.11m` up from `base_link`, fixed via `static_transform_publisher`. **Measure the x offset to finalize it** — it's currently an estimate |
| `odom` | Where the car started | Continuous but **drifting**, published by `vesc_to_odom_node` from wheel-speed and servo-angle integration |
| `map` | A fixed point in the mapped world | Only exists once `slam_toolbox` or `particle_filter`'s `map_server` is running |

**Why `odom` drifts, and why it matters.** It's computed from how fast the wheels turned and where the steering was pointed — dead reckoning, with no encoders and no IMU fusion. Wheelspin, tyre slip and small angle errors accumulate and never get corrected. Over a lap it can be metres off.

That's the whole reason localization exists: `particle_filter` corrects the drifting `odom` estimate against a known map, which is why racing on a saved map needs it and reactive `gap_follow` doesn't.

> **Worth knowing when debugging a `tf` tree:** `slam_toolbox`'s configured `base_frame` is `laser`, not `base_link` (see `f1tenth_stack/config/f1tenth_online_async.yaml`). This is a deliberate upstream choice, not a typo.
