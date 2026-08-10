# Writing your own node

> **Who this is for:** anyone writing code that can move the car. **This is a safety contract, not a style guide.**
> **Read first:** [architecture.md](architecture.md) for the safety model, then [adding-your-own-code.md](adding-your-own-code.md) for where the code goes.
> **You'll be able to:** write a driving node that meets the workspace's mandatory deadman requirement, and test it in the right order.

How to add your own driving code to this car.

This page covers **driving code specifically** — code that can make the wheels turn. If you're not sure whether what you're building counts, read [adding-your-own-code.md](adding-your-own-code.md) first; it covers both categories and shows where the line falls.

Read [architecture.md](architecture.md) first if you haven't, specifically the safety model section. It changes how you actually get your code to drive, and skipping it will cost you an afternoon wondering why a node that runs correctly moves nothing.

---

## What you're signing up for

Three things are non-negotiable. Everything else on this page is detail.

1. **Publish to `/drive` and nothing further downstream.** `ackermann_mux` handles the rest.
2. **Implement the LB deadman check.** Your node refuses to publish a non-zero command unless a human is holding LB. This is workspace policy, enforced in your code, not something the framework does for you.
3. **Test in the prescribed order.** Static, then wheels up, then floor. [Skipping ahead is how equipment and people get hurt.](#testing-before-its-on-wheels)

---

## The interface contract

Your node needs to:

**Subscribe** to `/scan` (`sensor_msgs/LaserScan`) and, if you need it, `/odom` (`nav_msgs/Odometry`).

**Publish** `ackermann_msgs/AckermannDriveStamped` to `/drive`.

**Subscribe to `/joy` and implement an LB deadman check.** This is **mandatory current workspace policy** ([the rule and its reasoning](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)), not optional and not a matter of taste.

> Your node must refuse to publish a non-zero drive command unless LB is currently held on a live `/joy` stream.

Copy the pattern directly from `gap_follow_node.py` (`joy_callback` / `_deadman_engaged`):

```python
self.declare_parameter('joy_topic', '/joy')
self.declare_parameter('deadman_button', 4)      # LB
self.declare_parameter('joy_timeout_sec', 0.5)
self.declare_parameter('enable_deadman', True)   # leave True -- see architecture.md
...
self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)

def joy_callback(self, msg):
    self.last_joy_time = self.get_clock().now()
    self.deadman_held = len(msg.buttons) > self.deadman_button and bool(msg.buttons[self.deadman_button])

def _deadman_engaged(self):
    if not self.enable_deadman:
        return True
    if not self.deadman_held or self.last_joy_time is None:
        return False
    return (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9 < self.joy_timeout_sec
```

**Call `_deadman_engaged()` first**, before any other logic, in whatever callback or timer actually publishes to `/drive`. If it returns false, publish `0.0 / 0.0` and return early.

This is a *second, independent* safety layer, sitting on top of `ackermann_mux`'s [arbitration](glossary.md#mux--multiplexer). Arbitration is the mux's job of picking which one of several competing commands actually reaches the motors.

It is not a replacement for that. You need both.

<details>
<summary><b>Why the check is written this way</b> — what each of the three conditions defends against. Worth reading once, so you don't "simplify" one of them out later.</summary>

The check has three parts, and each one covers a different failure:

**`self.deadman_held`** — the obvious one. The button is physically down right now.

**`self.last_joy_time is None`** — covers startup. Before any `/joy` message has arrived, the node has no idea what the controller is doing. The safe assumption is "not held". Without this, a node could drive during the window between launching and the first joystick message.

**The `joy_timeout_sec` age check** — covers the dangerous case. If `joy_node` dies, the gamepad's battery goes flat, or the USB cable is knocked out, no new `/joy` messages arrive. `self.deadman_held` keeps its last value, which might be `True`, forever.

Without the timeout, "the controller died while I was holding LB" would mean "the car drives itself indefinitely with no way to stop it". With it, the car stops 0.5 seconds later.

That third condition is the one people delete when tidying up, because in normal operation it never fires. It fires exactly when everything else has already gone wrong.

</details>

That's the entire contract. `ackermann_mux` takes it from there.

> **You never touch the VESC, the motor topics, or `/ackermann_cmd` directly.** Publishing anywhere downstream of the mux — straight to `/commands/servo/position`, for example — bypasses the joystick's safety override completely.
>
> Don't do that, except for isolated, supervised hardware testing (see [troubleshooting.md](troubleshooting.md)).

### The message fields that matter

```
drive.steering_angle   # radians, positive = left. Clamped in vesc.yaml to what the physical rack can do (servo_min/max: 0.15-0.85 → roughly ±0.34 rad in the stock config)
drive.speed            # m/s, positive = forward
```

Two things that catch people out. **Steering is in radians, not degrees** — ±0.34 rad is about ±19°, which is all the physical steering rack has. And **speed is in metres per second**, so `1.0` is a brisk walking pace, not a crawl.

---

## Getting your code to actually drive the car

This section exists because of one non-obvious behavior that will otherwise waste your time.

Because of the always-on joystick override described in [architecture.md](architecture.md#the-safety-model-read-this-before-writing-autonomy-code), **`/drive` is masked and does nothing while `teleop_launch.py` is running.** Your node will run, publish, and log perfectly while the car ignores it entirely.

Your node is a **control layer**, the same as `teleop_launch.py`, `gap_follow_launch.py` and `pure_pursuit_launch.py` — see [architecture.md](architecture.md#control-layers-exactly-one-at-a-time-in-a-second-terminal). So getting it to drive comes down to one rule: **don't run a different control layer at the same time.**

**Step 1 — Terminal 1:** launch the driver stack as normal.

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** the [LiDAR](glossary.md#lidar) — the spinning laser scanner on the front — spins up, and the logs settle. It never starts `teleop_launch.py` itself, so `/drive` is never masked to begin with — there's nothing you need to stop.

**Step 2 — Terminal 2:** launch your node.

Your node's deadman check needs a live `/joy` stream to ever engage. `joy_node` lives in `bringup_launch.py`, not `teleop_launch.py`, so it's already running from Step 1.

**Step 3 — hold LB, wheels off the ground, every time on a first run.**

> With no `teleop_launch.py` running, the mux's human override **does not exist in this session**. Your node's own deadman check is now the only thing standing between a bug and an unsupervised, moving car.

Before you trust it near the ground, watch the output in a third terminal:

```bash
ros2 topic echo /drive
```

**Working when:** it reads `0.0 / 0.0` the *instant* you release LB. If there's any lag, or it keeps commanding speed, stop and fix that before going further. That behavior is the whole safety property.

**Step 4 — when you're done**, `Ctrl+C` your node's terminal. The bringup terminal can stay up. Launch `teleop_launch.py` in your node's place to switch back to manual driving, or kill everything and start fresh.

Exact commands are in [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node). It's the same procedure used for both `gap_follow` and `pure_pursuit`, since both implement the mandatory deadman check.

---

## Package structure — using `gap_follow` as the template

`gap_follow` (in `src/gap_follow`) is a minimal, working example of exactly this pattern. [src/gap_follow/README.md](../src/gap_follow/README.md) has a line-by-line walkthrough of its algorithm and every parameter.

Copy its structure for a new package:

```
src/your_package/
├── package.xml            # declares dependencies: rclpy, sensor_msgs, ackermann_msgs
├── setup.py                # ament_python build, registers your node as a console_script
├── setup.cfg
├── resource/your_package    # empty marker file, required by ament_python
├── your_package/
│   ├── __init__.py
│   └── your_node.py         # the actual node
├── launch/
│   └── your_node_launch.py
└── config/
    └── your_node.yaml        # parameters, loaded by the launch file
```

<details>
<summary><b>What each file does in the real <code>gap_follow</code></b> — file-by-file, with links to the source. Read it while you copy the package, or skip if you already know the <code>ament_python</code> layout.</summary>

**`package.xml`** ([source](../src/gap_follow/package.xml)) — its dependencies are `rclpy`, `sensor_msgs`, `nav_msgs`, and `ackermann_msgs`. Gap follow consumes `/odom` for TTC as well as `/scan` and `/joy`. Note that `sensor_msgs` covers both `LaserScan` and `Joy`.

**`gap_follow/gap_follow_node.py`** ([source](../src/gap_follow/gap_follow/gap_follow_node.py)) — the node itself:

```python
self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
```

Topic names are ROS parameters (`scan_topic`, `odom_topic`, `drive_topic`) rather than hardcoded, so tests and bag playback can retarget them without editing code. All the tuning knobs — speed limits, steering limits, safety margins — are parameters too, not constants. See the pattern in `__init__`.

**`config/gap_follow.yaml`** ([source](../src/gap_follow/config/gap_follow.yaml)) — the actual parameter values, loaded at launch. Change behavior by editing this file, not the code.

**`launch/gap_follow_launch.py`** ([source](../src/gap_follow/launch/gap_follow_launch.py)) — loads the YAML above and starts the node. This is the minimum viable launch file for a single-node package.

**`resource/gap_follow`** — an empty marker file, not code. `ament_python`'s package index (`ament_index`) uses its mere presence to know the package exists. Every `ament_python` package needs one named after itself, and a missing one makes the package silently invisible.

</details>

> **Copy the deadman, not just the plumbing.** `gap_follow_node.py`'s `joy_callback` / `_deadman_engaged` is the reference implementation of the mandatory check above. It only publishes non-zero drive commands while LB is held on a live `/joy` stream, and publishes `0.0/0.0` otherwise.
>
> It is easy to copy `gap_follow`'s [scan](glossary.md#scan)-and-drive plumbing — the code that reads the laser and publishes a command — and leave the deadman behind. The node works fine in testing without it. Don't.

---

## Build and run workflow

**Terminal 1, from `~/racerbot-ws`:**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/racerbot-ws
colcon build --symlink-install --packages-select your_package
source install/setup.bash
ros2 launch your_package your_node_launch.py
```

**Working when:** the build ends with `Summary: 1 package finished`, then your node's startup logs appear and it stays running.

`--symlink-install` means editing your `.py` files takes effect immediately on the next launch, with no rebuild. You only need to rebuild when you change `package.xml`, `setup.py`, or add and remove files.

`--packages-select your_package` builds just your package, which is fast. Drop it to rebuild everything — slow, and since this Jetson has 8GB of RAM, prefer `--parallel-workers 1` for a full rebuild to avoid an OOM kill.

---

## Testing before it's on wheels

**Do these in order. Every step catches a class of bug that the next step would let hurt something.**

### 1. Static topic check

Launch your node with the rest of the driver stack *not* running at all.

```bash
ros2 topic echo /drive
```

Sanity-check the values against what you'd expect from known LaserScan inputs. You can play a recorded bag, or just watch it react to you waving a hand in front of the LiDAR.

**What this catches:** inverted steering signs, wrong units, values wildly out of range — with zero possibility of movement, because nothing downstream is running.

### 2. Wheels off the ground

Bringup running, your node launched per the procedure above (**no `teleop_launch.py`**), car propped up so the wheels spin freely.

Confirm steering and speed behave sensibly before anything touches the floor. Release LB and confirm it stops.

**What this catches:** everything the static test can't — real sensor timing, the actual deadman path, whether the motor spins the direction you expected.

### 3. Floor, low speed, open space

Only after 1 and 2 both look right. Keep a hand near the power switch.

> **Don't skip straight to the floor.** The whole reason `gap_follow` exists as a template is that it was built and tested this way first. Every step you skip is one you're trusting your first-draft code to have got right.
