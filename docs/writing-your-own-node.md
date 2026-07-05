# Writing your own node

How to add your own driving code to this car. This doc covers driving code specifically — if you're not sure whether what you're building even counts as "driving code," see [adding-your-own-code.md](adding-your-own-code.md) first, which covers both categories and where the line is. Read [architecture.md](architecture.md) first if you haven't — specifically the safety model section, since it changes how you actually get your code to drive.

## The interface contract

Your node needs to:
- **Subscribe** to `/scan` (`sensor_msgs/LaserScan`) and, if you need it, `/odom` (`nav_msgs/Odometry`)
- **Publish** `ackermann_msgs/AckermannDriveStamped` to `/drive`
- **Subscribe to `/joy` and implement an LB deadman check** — **mandatory, current workspace policy** (see [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)), not optional. Your node must refuse to publish a non-zero drive command unless LB is currently held on a live `/joy` stream. Copy the pattern directly from `gap_follow_node.py` (`joy_callback`/`_deadman_engaged`):
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
  Check `_deadman_engaged()` first, before any other logic, in whatever callback/timer actually publishes to `/drive` — publish `0.0 / 0.0` and return early if it's not engaged. This is a *second*, independent safety layer on top of `ackermann_mux`'s arbitration below, not a replacement for it — you need both.

That's the entire contract. `ackermann_mux` takes it from there — you never touch the VESC, the motor topics, or `/ackermann_cmd` directly. Publishing anywhere downstream of the mux (e.g. straight to `/commands/servo/position`) bypasses the joystick's safety override entirely — don't do that except for isolated, supervised hardware testing (see [troubleshooting.md](troubleshooting.md)).

`AckermannDriveStamped` fields that matter here:
```
drive.steering_angle   # radians, positive = left. Clamped in vesc.yaml to what the physical rack can do (servo_min/max: 0.15-0.85 → roughly ±0.34 rad in the stock config)
drive.speed            # m/s, positive = forward
```

## Getting your code to actually drive the car

Because of the always-on joystick override described in [architecture.md](architecture.md#the-safety-model-read-this-before-writing-autonomy-code), `/drive` is masked and does nothing while `teleop_launch.py` is running. Your node is a **control layer**, same as `teleop_launch.py`, `gap_follow_launch.py`, and `pure_pursuit_launch.py` — see [architecture.md](architecture.md#control-layers-exactly-one-at-a-time-in-a-second-terminal) — so getting it to drive is just: don't run a different control layer at the same time.

1. Launch the driver stack as normal, in its own terminal: `ros2 launch f1tenth_stack bringup_launch.py`. It never starts `teleop_launch.py` itself, so `/drive` is never masked to begin with — there's nothing to stop.
2. In a second terminal, launch your node. Since your node's own deadman check (mandatory, see above) needs a live `/joy` stream to ever engage, and `joy_node` lives in `bringup_launch.py` (not `teleop_launch.py`), it's already up.
3. **Hold LB, and wheels off the ground for the first run, every time.** With no `teleop_launch.py` running, the mux's override doesn't exist in this session — your node's own deadman check (which you implemented per the contract above) is now the only thing standing between a bug and an unsupervised, moving car. Watch `/drive` with `ros2 topic echo /drive` before you trust it near the ground; it should read `0.0 / 0.0` the instant you let go of LB.
4. When you're done, `Ctrl+C` your node's terminal. The bringup terminal can stay up — launch `teleop_launch.py` in its place to switch back to manual driving, or kill everything and start fresh.

See [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node) for the exact commands — this is the same procedure used for both `gap_follow` and `pure_pursuit`, since both implement the mandatory deadman check.

## Package structure — using `gap_follow` as the template

`gap_follow` (in `src/gap_follow`) is a minimal, working example of exactly this pattern — see [src/gap_follow/README.md](../src/gap_follow/README.md) for a line-by-line walkthrough of its algorithm and every parameter. Copy its structure for a new package:

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

Concretely, from `gap_follow`:

- **`package.xml`** ([src/gap_follow/package.xml](../src/gap_follow/package.xml)) — note the three real dependencies: `rclpy`, `sensor_msgs`, `ackermann_msgs`. Match these (plus `nav_msgs` if you subscribe `/odom`). `sensor_msgs` already covers `Joy`, needed for the deadman check below — no extra dependency required.
- **`gap_follow/gap_follow_node.py`** ([src/gap_follow/gap_follow/gap_follow_node.py](../src/gap_follow/gap_follow/gap_follow_node.py)) — the node itself:
  - `self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)`
  - `self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)`
  - Topic names are declared as ROS parameters (`scan_topic`, `drive_topic`) with `/scan`/`/drive` as defaults, rather than hardcoded — this lets you retarget the node (e.g. for testing against a bag file) without editing code.
  - All the tuning knobs (speed limits, steering limits, safety margins) are also parameters, not constants — see the pattern in `__init__`.
- **`config/gap_follow.yaml`** ([src/gap_follow/config/gap_follow.yaml](../src/gap_follow/config/gap_follow.yaml)) — the actual parameter values, loaded at launch. Change behavior by editing this file, not the code.
- **`launch/gap_follow_launch.py`** ([src/gap_follow/launch/gap_follow_launch.py](../src/gap_follow/launch/gap_follow_launch.py)) — loads the YAML above and starts the node. This is the minimum viable launch file pattern for a single-node package.
- **`resource/gap_follow`** — an empty marker file, not code. `ament_python`'s package index (`ament_index`) uses its mere presence to know the package exists; every `ament_python` package needs one named after itself.

`gap_follow_node.py`'s `joy_callback`/`_deadman_engaged` is the reference implementation of the **mandatory** deadman check from the interface contract above — it only publishes non-zero drive commands while LB is held on a live `/joy` stream, publishing `0.0/0.0` otherwise. Copy this pattern into `your_node.py`, not just `gap_follow`'s scan/drive plumbing — see [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node) for the exact launch procedure once your node is driving hands-off (short version: `bringup_launch.py` first, your node's launch file on top, and simply don't launch `teleop_launch.py` in the same session).

## Build and run workflow

```bash
source /opt/ros/jazzy/setup.bash
cd ~/racerbot-ws
colcon build --symlink-install --packages-select your_package
source install/setup.bash
ros2 launch your_package your_node_launch.py
```

`--symlink-install` means editing your `.py` files takes effect immediately on the next launch — no rebuild needed. You only need to rebuild when you change `package.xml`, `setup.py`, or add/remove files.

`--packages-select your_package` builds just your package (fast). Drop it to rebuild everything (slow — this Jetson has 8GB RAM, so prefer `--parallel-workers 1` for a full rebuild to avoid OOM).

## Testing before it's on wheels

In order of increasing risk, all before you trust a new node near the ground:

1. **Static topic check** — launch your node with the rest of the driver stack *not* running at all. `ros2 topic echo /drive` and sanity-check the values against what you'd expect from known LaserScan inputs (you can play a recorded bag, or just watch it react to you waving a hand in front of the LiDAR).
2. **Wheels off the ground** — bringup up, your node launched per the procedure above (no `teleop_launch.py` running), car propped up so wheels spin freely. Confirm steering and speed behave sensibly before anything touches the floor.
3. **Floor, low speed, open space** — only after 1 and 2 look right. Keep a hand near the power switch.

Don't skip straight to the floor — the whole point of `gap_follow` existing as a template is that it was built and tested this way first.
