# imu_bridge

> **Who this is for:** anyone who wants the car's IMU readings in ROS, to record them, check them, or feed them to an EKF. Also anyone who wants the car's *real* speed and steering angle, not what odometry and the servo command claim.
> **Read first:** [docs/concepts.md](../../docs/concepts.md), so you know what a *topic*, *node* and *launch file* are.
> **You'll be able to:** get calibrated IMU topics, a fused speed (`/odom/fused`) and the measured steering angle (`/steering/measured`). Then check them by moving the car by hand, and use them from your own node.
> **Time:** about 15 minutes the first time, mostly the setup.

An **IMU** (inertial measurement unit) is a chip with two sensors in it:
- a **gyro**, which measures how fast the car is rotating
- an **accelerometer**, which measures acceleration, including gravity

This package takes the IMU readings the car already produces and fixes their units, axes and bias. It publishes them as standard ROS messages that other tools can use directly.

On top of that, it combines them with the wheel speed into three things the rest of the car can use:
- **`/odom/fused`**: the car's forward speed, from the wheels *and* the accelerometer.
- **`/steering/measured`**: the angle the car is really turning at, from the gyros and the speed.
- **`/odom/imu`**: the car's odometry with its heading from the gyro instead of from the steering command, which SLAM can map on (`map_course gyro_odom:=true`).

**This is support code. It cannot move the car.** It publishes no `/drive`, `/ackermann_cmd` or `/commands/*`, so you can start and stop it freely at any time, including mid-run. See [docs/adding-your-own-code.md](../../docs/adding-your-own-code.md) for what that category means.

## Highlights

- **Converts the VESC's IMU to SI units and car axes.** The [VESC](../../docs/glossary.md#vesc) (the motor controller) reports g and deg/s, in whatever direction its board happens to face. On car 2 that's rotated about 85° from the car. This node outputs m/s² and rad/s, with x forward, y left and z up.
- **Uses the club's existing calibration, unchanged.** It reads `car.yaml` and the calibration files from the [imu-calibration](https://github.com/sfu-racerbot/imu-calibration) repo, and applies the same formula as that repo's `racer_imu`. A test checks the two give the same answer to 1e-12 on the car's real calibration file.
- **Removes leftover gyro bias at startup.** For the first 3 s it averages the gyro while the car sits still. It only counts samples where the wheels report under 0.03 m/s *and* every gyro axis is under 0.05 rad/s. Any motion throws the window away and starts again.
- **Gives the wheel [odometry](../../docs/glossary.md#odometry--odom) an honest uncertainty.** `/odom` arrives claiming zero uncertainty, which an EKF would treat as perfect. This node republishes it with a speed uncertainty of 0.08 m/s, and marks everything else as untrusted.
- **Ships a "shadow" EKF config.** It runs the EKF alongside everything else for comparison, without changing what any driving node uses.
- **A better speed, with a hard bound.** `vehicle_state_node` fuses the wheel speed with the BNO085's forward accelerometer. In a synthetic test with noisy wheels (±0.08 m/s) and a vibrating IMU (1 m/s² rms), its error was 0.37–0.46× the wheels' own, over six noise seeds. It is **never more than 0.3 m/s from the wheel speed**, and is exactly the wheel speed whenever the accelerometer is missing.
- **Wheelspin and lockup are rejected, not followed.** A wheel speed that jumps further than the filter's own uncertainty allows is skipped as slip, for up to 0.5 s. In the test, with the truth at 1.0 m/s and the wheels spinning at 1.6, the output held at 1.3 (the 0.3 m/s bound) instead of following the spin.
- **The steering angle the car really turned at.** `/steering/measured` solves the bicycle model backwards from the measured yaw rate and speed. It includes understeer and slip, which the servo command can't show. Below 0.3 m/s it reads `NaN` rather than guessing.
- **Two gyros cross-checked; a wrongly mounted IMU is refused.** Both gyros are averaged. If they steadily disagree by more than 0.15 rad/s, the angle reads `NaN` instead of being wrong. An IMU whose gravity reading says it is upside down or on its side is left out entirely.
- **Gyro odometry for SLAM, without touching bringup.** `/odom/imu` is `vesc_to_odom`'s distance with the gyro's heading. An extra frame, `odom_imu -> odom`, makes the chain `odom_imu -> base_link` exactly that pose (checked with tf2 to 1e-15), so `slam_toolbox` can map on it while `vesc_to_odom` keeps its own transform. With no usable gyro it *is* today's `/odom`, to the bit.
- **Car 1's BNO085 is on the Jetson's own I2C bus.** There is no microcontroller in between: imu-calibration's `tools/bno_i2c_bridge.py` reads the sensor (bus 7, address 0x4A, ~250 Hz) and presents it as a serial port at `/tmp/bno085`. `vehicle_state_launch.py` starts it for you. (Car 2's BNO085 sits behind a microcontroller on a USB port; that path still works with `bno_port:=/dev/ttyACM1`.)
- **All the math runs without ROS.** `convert.py` and `vehicle_state.py` have no `rclpy` import, so their tests run with a bare `pytest`. Every new test was seen to fail against deliberately broken code (52 different breaks).

### Why it exists

Nothing on this car measures how much it actually turned. Heading comes from integrating the *commanded* steering angle (see [docs/localization.md](../../docs/localization.md)), so it drifts with every bit of servo error, steering slop and tyre slip.

A gyro measures rotation directly. This package is the first step: get that measurement into ROS, correctly calibrated, and recorded alongside everything else. Then you can tell from real runs whether it actually helps.

Two more numbers the car relies on are also guesses:
- **Speed** comes from counting motor revolutions. When the wheels spin or lock, the count keeps going, and the "speed" is wrong.
- **The steering angle** in `/drive` is what was *asked for*. Servo lag, steering slop and understeer all hide in the gap between that and what the car did.

SLAM suffers most from the steering one. `slam_toolbox` starts every scan match from the odometry's guess of where the car is. On car 2 (2026-10-19) that guess turned 0.15 s before the car did, and 5% too far: 0.44 rad/s RMS of turn-rate error.

`slam_toolbox` has no IMU input of its own. It only reads the odometry transform, so the gyro has to reach it through that transform. That is what `/odom/imu` is for.

`vehicle_state_node` measures both, so a driving node (starting with `gap_follow` in `map_course`) can use them, and every recorded run shows how far off the guesses were.

## What this actually is

Two nodes sit between the hardware drivers and anything that wants IMU data. `imu_bridge_node` cleans up the raw readings; `vehicle_state_node` turns them into speed and steering:

```
vesc_driver_node ── /sensors/imu/raw (g, deg/s, VESC board axes) ──┐
vesc_to_odom_node ─ /odom (wheel speed) ───────────────────────────┤
BNO085 (car 1: Jetson I2C → bno_i2c_bridge.py → /tmp/bno085) ──────┤
                                                                   ▼
                                                         imu_bridge_node
                                    calibrate · rotate to car axes · startup bias
                                                                   │
           /imu/data, /imu/vesc/data, /imu/bno/*, /imu_bridge/wheel_odom
                                                                   │
              ┌────────────────────────────┬───────────────────────┴───────┐
              ▼                            ▼                               ▼
   vehicle_state_node (+ /odom)    EKF in shadow mode (optional)    race_diagnostics
   fuse speed · measure steering   /odometry/filtered, no TF        records all of it
              │
   /odom/fused, /steering/measured
```

It never opens the VESC's serial port. `vesc_driver_node` owns that port during bringup, and the IMU readings reach this node over `/sensors/imu/raw`.

An **EKF** (extended Kalman filter) combines two imperfect sensors into one better estimate. Here it would combine wheel speed (good at *how fast*) with the gyro (good at *how much it turned*).

**"Shadow mode"** means the EKF publishes its estimate on `/odometry/filtered`, and nothing reads it. In particular, it does **not** publish the `odom -> base_link` [transform](../../docs/glossary.md#tf--transform--frame) that every driving node uses. That stays with `vesc_to_odom_node`, exactly as before. The point is to record the EKF next to SLAM and the particle filter, and compare them offline.

## Before you start

- [ ] **The VESC's IMU must be switched on.** It was measured on 2026-08-19 sending all zeros, meaning it was off or not fitted (see [docs/localization.md](../../docs/localization.md)). It was calibrated through the imu-calibration repo in September 2026, which means it works over that repo's own serial driver. This node prints a warning every 5 s if it still sees all zeros.
- [ ] **Car 1: the BNO085 must answer on I2C.** It is wired to the Jetson's 40-pin header (pin 1 3.3 V, pin 3 SDA, pin 5 SCL, pin 6 GND), which is I2C bus 7. Working when `i2cdetect -y -r 7` shows `4a` in its grid. Only the `-r` form works on this Jetson. A `Remote I/O error` on every address means SDA and SCL are swapped. Your user must be in group `i2c` (`id` lists it).
- [ ] **Clone the imu-calibration repo** where the config expects it:
  ```bash
  git clone git@github.com:sfu-racerbot/imu-calibration.git ~/imu-calibration
  ```
  Working when `ls ~/imu-calibration/config/car.yaml` prints the path. If it lives somewhere else, pass `racer_imu_root:=/that/path` when launching. The node **refuses to start** without its `config/car.yaml`. Without the mount rotation it would swap the x and y axes.
- [ ] **For the EKF only**, install robot_localization once:
  ```bash
  sudo apt install ros-jazzy-robot-localization
  ```
  Working when `ros2 pkg prefix robot_localization` prints a path instead of an error.
- [ ] **Build** (from `~/racerbot-ws`):
  ```bash
  colcon build --symlink-install --packages-select imu_bridge
  source install/setup.bash
  ```
  Working when the build ends with `Summary: 1 package finished`.

## Running it

**Terminal 1** is the hardware layer. Leave it running. On its own it never moves the car.

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** `ros2 topic hz /sensors/imu/raw` (in another terminal) shows about 50 Hz.

**Terminal 2** runs the IMU side. **Keep the car still for the first 3 seconds** after this starts.

On car 1, start everything with one command: the BNO085's I2C bridge, `imu_bridge_node` and `vehicle_state_node`.

```bash
ros2 launch imu_bridge vehicle_state_launch.py
# without the BNO085 (VESC IMU only; /odom/fused is then exactly the wheel speed):
ros2 launch imu_bridge vehicle_state_launch.py bno:=false
```

If a `bno_i2c_bridge.py` is already running (you started one by hand to calibrate, say), the launch uses it and says so, rather than starting a second one on the same sensor. That one must have been started with `--pty /tmp/bno085 --no-dyncal`.

To run `imu_bridge_node` on its own instead:

```bash
ros2 launch imu_bridge imu_bridge_launch.py
# with a BNO085 behind a microcontroller on USB (car 2's setup):
ros2 launch imu_bridge imu_bridge_launch.py bno_port:=/dev/ttyACM1
```

**Working when** you see, within about 5 s:
- a line saying `vesc: startup gyro bias removed [...] rad/s`, and one starting `bno:` the same. The numbers should be small, about 0.01 rad/s or less.
- every 5 s, from `imu_bridge_node`: `rates: vesc 50.0 Hz, odom ... Hz, bno 250 Hz`
- every 5 s, from `vehicle_state_node`: `rates: bno 250 Hz, vesc 50 Hz | speed: fused 250 | ...`. `fused` means the accelerometer is being used; `wheel_only` means it isn't (see [Troubleshooting](#troubleshooting)).

**If it doesn't:**
- `car.yaml not found`: the repo isn't cloned where `racer_imu_root` points.
- `no VESC IMU data`: bringup isn't running.
- `all zeros`: the IMU is off in VESC Tool.
- The bias line never appears: something is moving the car, or `/odom` is missing (see [Troubleshooting](#troubleshooting)).

**Terminal 3 (optional)** runs the shadow EKF. Start it after terminal 2.

```bash
ros2 launch imu_bridge imu_ekf_shadow_launch.py
```

**Working when:** `ros2 topic hz /odometry/filtered` shows about 50 Hz. `ros2 run tf2_ros tf2_echo odom base_link` should be unchanged, because the EKF publishes no transform.

To record everything for later analysis, run `ros2 launch race_diagnostics record_run.py` as usual. Its default list of [topics](../../docs/glossary.md#topic) already includes every one below.

### Check it's right: turn the car by hand

With terminals 1 and 2 running, and the car on the floor or the bench:

```bash
ros2 topic echo /imu/data --field angular_velocity
```

1. **Rotate the car counterclockwise, seen from above** (turning left). `z` must go **positive**. If it goes negative, the mount rotation in `car.yaml` is wrong.
2. **Leave it still.** `x`, `y` and `z` should all sit within about ±0.005 rad/s of zero.

```bash
ros2 topic echo /imu/data --field linear_acceleration
```

3. **At rest, `z` reads about +9.8**, and `x` and `y` read close to 0. An accelerometer at rest measures the floor pushing up against gravity, so +9.8 is correct here, not −9.8. If z reads −9.8, the IMU is mounted upside down relative to `car.yaml`.
4. **Push the car forward by hand.** `x` should jump positive briefly.

Then the two outputs of `vehicle_state_node`:

```bash
ros2 topic echo /steering/measured --field position
```

5. **With the car still, it reads `[nan]`.** That is correct: an IMU can't see the steering angle of a car that isn't moving.
6. **Drive a slow circle with the steering held fully left.** This is manual driving: `teleop_launch.py` in another terminal, LB held, on the floor with room around the car. The value should be **positive** and at most about 0.38 rad, which is full stick. Less than that is understeer: the car turning less than the wheels point. Fully right gives negative.

```bash
ros2 topic echo /odom/fused --field twist.twist.linear.x
```

7. **At rest it reads about 0**, and while driving it stays within 0.3 m/s of `ros2 topic echo /odom --field twist.twist.linear.x`.

## Topics

Each [topic](../../docs/glossary.md#topic) below is a named channel you can `ros2 topic echo`. The *frame* says which axes the numbers are in ([glossary](../../docs/glossary.md#tf--transform--frame)).

| Topic | Type | Frame | What it is |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | `base_link` | **The one to use.** The primary IMU, calibrated, in car axes. `primary: auto` picks the BNO085 if `bno_port` is set, otherwise the VESC. |
| `/imu/vesc/data` | `sensor_msgs/Imu` | `base_link` | The VESC IMU, calibrated, in car axes (~50 Hz) |
| `/imu/bno/raw` | `sensor_msgs/Imu` | `bno_sensor` | The BNO085 as received, in SI units and in the sensor's own axes, with its orientation quaternion. Only when `bno_port` is set. No transform exists for this frame. |
| `/imu/bno/data` | `sensor_msgs/Imu` | `base_link` | The BNO085, calibrated, in car axes. Only when `bno_port` is set. |
| `/imu_bridge/wheel_odom` | `nav_msgs/Odometry` | `odom` → `base_link` | `/odom`, unchanged except for the twist covariance: 0.08² on forward speed, 1000 on everything else |
| `/imu/status` | `std_msgs/String` (JSON, latched) | — | Whether `/imu/data` can be trusted right now: `{"state": "ok" \| "waiting" \| "fault", "reason": ..., "imu": "bno" \| "vesc"}`. See [The `/imu/status` verdict](#the-imustatus-verdict). |
| `/odometry/filtered` | `nav_msgs/Odometry` | `odom` | Published by the shadow EKF, not this node |

Every `base_link` message reports **no orientation**: `orientation_covariance[0] = -1`, the `sensor_msgs/Imu` convention for "not provided". Each one does carry an angular velocity and acceleration covariance from `gyro_sigma` and `accel_sigma`.

It subscribes to `/sensors/imu/raw` and `/odom`, using sensor-data QoS.

Every setting is in [`config/imu_bridge.yaml`](config/imu_bridge.yaml), with a comment for each.

`vehicle_state_node` adds:

| Topic | Type | What it is |
|---|---|---|
| `/odom/fused` | `nav_msgs/Odometry` | `/odom` with its forward speed (`twist.twist.linear.x`) replaced by the fused speed, and that speed's variance in `twist.covariance[0]`. Pose, yaw rate, frames and timestamp are `/odom`'s own. One message per `/odom` message, so it stops when `/odom` stops. |
| `/steering/measured` | `sensor_msgs/JointState` | One joint, named `steering`. `position[0]` is the effective steering angle in radians, positive = left. `NaN` when it can't be measured. 50 Hz. |
| `/odom/imu` | `nav_msgs/Odometry` | Gyro odometry, frame `odom_imu` → `base_link`: `vesc_to_odom`'s distance, the gyro's heading. Yaw rate is the gyro's. One message per `vesc_to_odom` transform (50 Hz). Always published. |
| `/tf`: `odom_imu` → `odom` | transform | Only with `publish_odom_tf:=true`: the correction that makes `odom_imu → base_link` equal `/odom/imu`. |

It reads `/odom`, `/imu/bno/data` and `/imu/vesc/data`. Its settings are in [`config/vehicle_state.yaml`](config/vehicle_state.yaml), with a comment for each.

## The `/imu/status` verdict

`/imu/data` is published from the first sample, for every consumer. `/imu/status` says whether those samples can be trusted *right now*. `neural_racer` requires `"state": "ok"` (`require_imu: true`) and commands zero otherwise. Other consumers can ignore it.

It describes the primary IMU only, the one on `/imu/data`:

| State | When |
|---|---|
| `waiting` | Startup: the gyro bias isn't removed yet, or the gravity check hasn't run. Keep the car still for about 3 s after launch. |
| `ok` | Bias removed, gravity check passed, and the latest sample is fresh and valid. |
| `fault` | One of: no sample for `status_stale_timeout_s` (0.1 s); all six raw fields exactly zero (the IMU is off in VESC Tool, or not fitted); a non-finite field; or the gravity check failed. |

**The gravity check:** over `gravity_window_s` (1 s) of stillness, the mean acceleration must be 9.80665 ± 0.5 m/s² and within 10° of straight up. Wrong units or a wrong mount rotation in `car.yaml` fail it. A gravity failure **latches until the node restarts**, because a wrong mount doesn't fix itself. The other faults clear as soon as good samples return.

```bash
ros2 topic echo /imu/status --once     # any terminal, with imu_bridge running
```

**Working when:** after the car has been still for about 3 s, it shows `"state": "ok"`. The `reason` field names the problem otherwise.

The logic is in `imu_bridge/imu_health.py` (no `rclpy`). It replaces the separate `racerbot_imu` package, which re-converted the same raw data with placeholder calibration and published a second `/imu/data`; that package was removed on 2026-10-24.

## Fused speed and measured steering

**What the speed is.** The wheels are good at average speed, but wheelspin fools them, and they only report every 20 ms. The accelerometer reacts instantly, but drifts if you add it up for long.

A small Kalman filter combines the two. (A Kalman filter is a standard way to blend imperfect sensors, weighting each by how much it can be trusted; the EKF above is one too.) The accelerometer predicts the speed between wheel readings, and the wheels correct it. The filter also learns the accelerometer's own offset, every time the car is stopped.

**What the steering angle is.** A car driving a circle of radius R turns at `yaw rate = speed / R`. Its front wheels point at `tan(angle) = wheelbase / R`. Put together: `angle = atan(wheelbase × yaw rate / speed)`, with car 1's 0.36 m wheelbase. The gyros give the yaw rate, and the fused speed gives the speed.

The two gyros are averaged. A rigid car turns at the same rate everywhere on it, so the second gyro reduces noise and catches a faulty one, but adds no new information.

**When it falls back, and to what:**

| Situation | `/odom/fused` | `/steering/measured` |
|---|---|---|
| everything working | fused speed, within 0.3 m/s of the wheels | the angle |
| wheels slipping (jump the filter can't explain) | the accelerometer's speed, held within 0.3 m/s of the wheels, for up to 0.5 s | the angle |
| no BNO085 accelerometer, or it is stale or mounted wrong | **exactly** the wheel speed | the angle, from whichever gyro works |
| slower than 0.3 m/s | fused speed | `NaN` |
| no usable gyro, or the two steadily disagree by > 0.15 rad/s | fused speed | `NaN` |
| `/odom` stops | stops too (consumers' stale-odometry checks fire as for `/odom`) | `NaN` after 0.2 s |
| `vehicle_state_node` dies | stops | stops |

### Using them in your own node

**Speed:** every driving node here already reads speed through an `odom_topic` parameter, so switching is one parameter, no code. For example, `gap_follow` uses it for TTC braking, its acceleration ramp and its stale-odometry stop.

This is the shape of the change, not a command to paste. Run it only where you would start that node anyway, with bringup and this IMU side already up, following the test order below:

```bash
ros2 run gap_follow gap_follow_node --ros-args --params-file <its yaml> -p odom_topic:=/odom/fused
```

**Working when:** the node starts as usual, and `ros2 topic info /odom/fused` lists it as a subscriber.

Keep SLAM, the particle filter and the `odom -> base_link` [transform](../../docs/glossary.md#tf--transform--frame) on `/odom`: `/odom/fused` only changes the speed.

**Steering:** subscribe to it (ask ROS to deliver every message on that topic to a function of yours, a *callback*) as a `JointState`. Treat `NaN` as "not known right now", never as zero:

```python
import math
from sensor_msgs.msg import JointState

self.create_subscription(JointState, '/steering/measured', self.on_steering, 10)

def on_steering(self, msg):
    angle = msg.position[0]        # radians, positive = left
    if math.isfinite(angle):
        self.measured_steering = angle
        self.measured_steering_time = self.get_clock().now()
```

**A node that changes how the car drives because of either topic is driving code.** Bring it up in the workspace's test order: no driver stack, then wheels off the ground, then the floor at low speed ([docs/writing-your-own-node.md](../../docs/writing-your-own-node.md#testing-before-its-on-wheels)). Using them only to display or record changes nothing about driving.

### In `map_course`

`ros2 run racerbot_launch map_course` starts this whole IMU side by default (`imu:=true`) and records it. Driving on the fused speed is opt-in:

| Option | What it does |
|---|---|
| *(nothing)* | IMU side runs and is recorded; `gap_follow` uses `/odom` as before |
| `fused_speed:=true` | `gap_follow` uses `/odom/fused` instead |
| `record_bag:=true` | also writes a bag with `/odom`, `/odom/fused`, `/steering/measured` and the steering commands side by side. Use it for both runs of an A/B comparison. |
| `imu:=false` | no IMU side. The `map_course` wrapper adds this by itself in the simulator. |

## Gyro odometry for SLAM

**What it is.** `vesc_to_odom` works out the car's pose by adding up small steps: how far it went (from the wheels) and how much it turned (from the *steering command*). `/odom/imu` adds up the same steps, but the turning comes from the gyro. Distance is unchanged, so the only difference between the two is where heading comes from.

**How SLAM gets it.** `slam_toolbox` reads its odometry from the transform tree, not from a topic: the transform from its "odom" frame (the [odometry](../../docs/glossary.md#odometry--odom) frame, fixed where the car powered up) to the car. So `vehicle_state_node` publishes one extra transform, `odom_imu → odom`. It is chosen so that `odom_imu → odom → base_link` comes out as exactly the gyro pose, and `slam_toolbox` is then told its odom frame is `odom_imu`:

```
today:        map → odom ─────────────→ base_link → laser
gyro_odom:    map → odom_imu → odom ──→ base_link → laser
                    (vehicle_state)  (vesc_to_odom, unchanged)
```

`vesc_to_odom` keeps publishing `odom → base_link` exactly as before. Nothing in bringup changes, and every node that looks up `map → base_link` (the lap recorder, the dashboard) gets the gyro-corrected pose through the chain without knowing.

**Where heading comes from, step by step:** the BNO085's gyro if it is fresh, mounted the right way up, and has no gap in its samples over the step. Otherwise the VESC's. Otherwise, or while the two gyros disagree, `vesc_to_odom`'s own heading change, so the worst case is today's odometry.

**The one way it could break the tree, and the guard.** A frame can have only one parent. If `slam_toolbox` were still on `odom`, it would publish `map → odom` while this node publishes `odom_imu → odom`, and the tree would flip between them. So the transform is off by default (`publish_odom_tf: false`), and `map_course gyro_odom:=true` turns it on only together with moving SLAM to `odom_imu`.

On top of that, the node watches `/tf`. The moment anything else publishes a parent for `odom`, it stops publishing its transform for good, and logs an error every 5 s. Checked on 2026-10-24: one message after a fake `map → odom` appeared, it stopped.

**Using it elsewhere.** The particle filter and `race_launch.py` are untouched. They could take the same chain later (point their odom frame at `odom_imu`), but that changes what `pure_pursuit` steers on: driving code, full test order.

<details>
<summary><b>Deep dive: the filter, the lever arm and the checks</b> — only needed if you're changing <code>vehicle_state.py</code> or tuning <code>vehicle_state.yaml</code>. Safe to skip.</summary>

**State and equations.** The filter's state is `[v, b]`: base_link's forward speed and the accelerometer's forward bias.

```
predict (each BNO sample, dt from the sensor's own timestamps):
    v += (a - b) dt
    P  = F P Fᵀ + Q,   F = [[1, -dt], [0, 1]],   Q = diag(sigma_accel² dt, sigma_bias_rw² dt)
update (each /odom message, z = wheel speed):
    y = z - v,   S = P₀₀ + sigma_wheel²
    if |y| > slip_gate · √S (and slipping < max_slip_time):  skip, report "slip"
    else:  K = P[:,0] / S,   [v, b] += K y,   P = (I - K H) P
output = clamp(v, z - max_divergence, z + max_divergence)
```

`sigma_accel` is a speed random walk (m/s per √s), not a per-sample noise, so the filter behaves the same at any IMU rate. Its value, 0.15, also sets how long a slip can be told apart from a real change of speed: the gate widens as the filter's uncertainty grows between accepted wheel readings. At 0.5 the gate opened within 50 ms, shorter than a spin.

**Lever arm.** The BNO085 sits on the front axle, 0.36 m ahead of base_link (the rear axle). On a turning car a point ahead of the axle also accelerates towards the turn centre, which shows up on its x axis as `−yaw_rate² × 0.36`: about −0.36 m/s² at 1 rad/s, read as braking in every corner. `accel_at_base_link` adds it back, using the BNO's own gyro (same chip, same instant). The test checks it against the circle's geometry differentiated numerically.

**Gravity check.** Each IMU's z acceleration is averaged (1 s time constant). Under 0.7 g, or before 10 samples, that IMU is not used at all. An upside-down IMU reads −9.8, and its gyro reads yaw rate with the wrong sign. Averaged in, it would cancel the good gyro and report the car driving straight.

**Gyro agreement.** The running mean (0.5 s) of the *signed* difference between the gyros must stay under 0.15 rad/s. The VESC's samples are late by its serial round-trip (about 20 ms), so in a slalom the two disagree back and forth. That averages out: about 0.08 rad/s in a ±2 rad/s, 1 Hz slalom. A sign or scale error in a steady turn doesn't average out.

**Gyro odometry details.** Each gyro's yaw rate is integrated into a heading against its own stamps (trapezoid rule; nothing across a gap over `max_gyro_gap`, 50 ms). Each `vesc_to_odom` transform is a step: distance = the wheel pose's change along its previous heading (exactly `v·dt` as `vesc_to_odom` integrated it). Heading change = the gyro's change between the two transform stamps. A gyro step moves along its mid-step heading; a fallback step along its start heading, as `vesc_to_odom` does, which is why the fallback is exact. A transform stamped up to 50 ms past the newest gyro sample is reached by extrapolating at the newest rate. The next step starts from that same value, so an extrapolation error is corrected rather than kept (tested: no accumulation over 6 s of a sinusoidal turn).

**Timestamps.** Staleness is judged on the node's own receive clock. Time between accelerometer samples uses the message stamps: the BNO's come from its own clock, so they stay evenly spaced even when samples arrive in bursts. A gap over `max_accel_dt` (50 ms) is not integrated; the filter restarts from the next wheel speed.

</details>

## Known limits

- **Acceleration is measured where the IMU sits, not at the car's centre.** In a turn, an IMU off-centre also feels centripetal acceleration. The VESC is about 9 cm left of centre. Yaw rate is the same everywhere on a rigid car, so it's unaffected. `/imu/*` carries the acceleration as measured. The full lever-arm correction lives in `racer_imu`'s `CenterFuser` and isn't ported here. `vehicle_state_node` corrects only what it uses: the BNO's forward axis.
- **VESC samples are timestamped when they arrive, at 50 Hz.** `vesc_driver_node` polls every 20 ms and stamps each reply with the time it was received, so every reading is late by the serial round-trip. The BNO085 path uses the sensor's own clock, mapped onto ROS time.
- **No orientation is published.** The VESC's own orientation estimate is in board axes and is dropped. The BNO085's quaternion is kept only on `/imu/bno/raw`.
- **The startup bias needs `/odom`.** Unknown wheel speed counts as moving. Without `vesc_to_odom_node` the bias never completes, and the node says so every 5 s.
- **Gyro odometry is untested on a moving car.** On 2026-10-24 it passed two checks, both on an isolated ROS domain:
  - a fake car on a circle: heading from the gyro, the tf2 chain exact, the two-parents guard fired;
  - parked, with the real LiDAR, real BNO and real `slam_toolbox` on `odom_imu`: map built, `map → base_link` resolved, 0.009° of gyro drift in 45 s.

  The first moving test is the A/B in [docs/launchstart.md](../../docs/launchstart.md#testing-the-imu-in-mapping-ab).
- **The lap counter still counts turning in `odom`**, the steering model's frame, even with `gyro_odom:=true`. That keeps the A/B to one difference. Switching it to `odom_imu` (`auto_map_race.yaml` `odom_frame`) would make its 300° gate exact; do it after the A/B.
- **Not yet driven on.** As of 2026-10-24, on car 1:
  - `vehicle_state_node` passed a static topic check on an isolated ROS domain, against a fake car on a steady circle: speed exact, angle matching the closed form.
  - `vehicle_state_launch.py` ran against the real BNO085 on the bench: 250 Hz, startup bias removed, gravity check passed.
  - Nothing has been driven on yet. Do the hand checks above before trusting any of it.
- **Its tuning is reasoned, not fitted.** `sigma_accel`, `slip_gate` and the rest come from the sensors' specs and synthetic tests, not car data. Record some runs with `record_bag:=true` and check them before trusting the numbers.
- **Car 1's VESC IMU mount and calibration are car 2's.** imu-calibration's `car.yaml` still holds car 2's `rpy_deg` and `vesc_calib.json` for the VESC. So `vehicle_state_node` uses only the VESC's yaw rate, which survives that because both boards lie flat, and the gravity check would catch one that doesn't. It does not use the VESC's accelerometer. Redo `calibrate_imu.py --step mount` on car 1.
- **The BNO085's mount yaw is not measured yet.** On 2026-10-24, parked on a level floor, the nominal mount read −0.41 m/s² forward and −0.89 m/s² sideways: about 2.4° of pitch and 5.2° of roll error. `rpy_deg` in imu-calibration's `car.yaml` is now `[92.44, 5.22, 90.11]`, which puts gravity at 0.00 / 0.00 / 9.76. Which way is *forward* can't be seen from gravity, so the yaw is still the nominal one: check it with the forward push in [the A/B test](../../docs/launchstart.md#testing-the-imu-in-mapping-ab), step 1.
- **The angle is the path's, not the wheels'.** It assumes the rear wheels don't slide sideways (the kinematic bicycle model). In a rear-wheel slide the car yaws faster than its wheels point, and the reported angle reads high.

<details>
<summary><b>Deep dive: the exact calibration formula</b> — only needed if you're changing <code>convert.py</code> or the imu-calibration repo. Safe to skip.</summary>

For each sample, in SI units (the VESC's g and deg/s are converted first, with g = 9.80665 m/s²):

```
acc_car  = R @ (A_inv @ (acc  - (acc_bias  - device_offsets.accel_g    * 9.80665)))
gyro_car = R @ (gyro - (gyro_bias - radians(device_offsets.gyro_deg_s))) - startup_residual
```

- `A_inv`, `acc_bias`, `gyro_bias` come from `calib/<imu>_calib.json`. That's an ellipsoid fit of the accelerometer over 12 static poses, and the gyro mean while still.
- `device_offsets` is the part of the bias the VESC already subtracts itself (VESC Tool → App Settings → IMU). `acc_bias`/`gyro_bias` are always the **total** bias, so only the remainder is removed and nothing is corrected twice. A file without `device_offsets` means the device removes nothing.
- `R = Rz(yaw) · Ry(pitch) · Rx(roll)` from `imus.<imu>.rpy_deg` in `car.yaml`. This is Z-Y-X order, like ROS/tf, and maps sensor axes to car axes.
- The calib path in `car.yaml` is relative to the repo root, the parent of `config/`.
- A missing calib file gives an identity calibration. The node logs `identity (no calibration file)`, and the mount rotation is still applied. A file that exists but is malformed (missing key, non-3×3 or non-finite `A_inv`) stops the node instead.

This must stay identical to `racer_imu.calibration.ImuCalibration.apply` followed by `racer_imu.frames` rotation. `test_matches_racer_imu_on_its_real_calibration_file` checks it against the real repo whenever it's present:

```bash
RACER_IMU_ROOT=~/imu-calibration python3 -m pytest src/imu_bridge/test/ -v
```

**Why the EKF fuses only forward speed and yaw rate:** SLAM and the particle filter already turn wheel odometry plus LiDAR into a pose. Fusing a pose, or anything else they already use, would count the same information twice, and the filter would become confident without good reason. The reasoning is in [`config/ekf_shadow.yaml`](config/ekf_shadow.yaml) and [docs/localization.md](../../docs/localization.md).

</details>

## Tests

```bash
python3 -m pytest src/imu_bridge/test/ -v
```

Source ROS first and give the tests their own domain (`export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`): `test_imu_status_node.py` starts a real node, and unsourced it fails to collect, which stops the whole run. Every other file needs no ROS and can be run on its own. Without the imu-calibration repo present, the two `racer_imu` cross-check cases skip and say why.

- `test_vehicle_state.py`: the filter against the Kalman equations written out by hand, steering against the circle geometry, the lever arm against numerically differentiated geometry, and every fallback in the table above, at and just past each limit.
- `test_gyro_odometry.py`: heading against a constant-rate arc and the true circle (forwards and reversing), distance identical to `vesc_to_odom`'s, the transform chain composing to the gyro pose, the exact fallback, gaps, extrapolation not accumulating, source preference, and the two-parents check.
- `test_vehicle_state_config.py`: `vehicle_state.yaml` against the code's defaults and `vesc.yaml`'s wheelbase, and a static check that the node publishes only `Odometry`, `JointState` and its one transform.
- `test_imu_health.py`: every `/imu/status` state against standard gravity and closed-form tilts, at and just past each limit, the latch, and `neural_racer`'s own parser.
- `test_imu_status_node.py` (**needs ROS sourced**): the real `imu_bridge_node` fed raw VESC samples, read by a subscriber with `neural_racer`'s QoS.

`map_course`'s wiring is tested in `src/racerbot_launch/test/test_map_course_imu.py`, which needs ROS sourced.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Node exits with `car.yaml not found` | imu-calibration not cloned at `racer_imu_root` | Clone it there, or pass `racer_imu_root:=...` |
| `startup gyro bias removed` never appears | car moving, or no `/odom` | Keep it still; check `ros2 topic hz /odom` |
| Warning: all zeros | VESC IMU disabled | VESC Tool → App Settings → IMU, sample rate > 0 |
| Turning left gives negative `z` | wrong `rpy_deg` for this IMU in `car.yaml` | Redo the mount step in imu-calibration (`tools/calibrate_imu.py --step mount`) |
| `ModuleNotFoundError: racer_imu` with `bno_port` set | repo missing at `racer_imu_root` | Same as the first row |
| `no BNO085 data` (car 2) | wrong port, or the MCU isn't streaming | Check `ls /dev/ttyACM*`. The VESC is usually `ttyACM0`. |
| `no BNO085 data ... retrying every 2 s` (car 1) | the I2C bridge isn't running or can't reach the sensor | Look for `bno_i2c_bridge` errors above it; check `i2cdetect -y -r 7` shows `4a`. It reconnects by itself once the bridge works. |
| vehicle_state: `no usable bno accelerometer: /odom/fused is exactly the wheel speed` | no `/imu/bno/data`, or it failed the gravity check | Check `ros2 topic hz /imu/bno/data`, then the `mounted wrong` warning |
| vehicle_state: `... mounted wrong ...` | that IMU's z doesn't read about +9.8 at rest | Fix `rpy_deg` in imu-calibration's `car.yaml` (mount step) |
| vehicle_state: `gyros disagree` | one gyro reads a different yaw rate in steady turns | Compare `/imu/bno/data` and `/imu/vesc/data` `angular_velocity.z` while turning by hand; the wrong one needs its mount redone |
| vehicle_state: `wheels rejected as slipping` on many messages | real wheelspin, or a bad accelerometer | A few during hard launches is the point. All the time while cruising means the accelerometer is off: check the mount and its calibration |
| vehicle_state: `STOPPED publishing odom_imu -> odom` | something else publishes a parent for `odom` (SLAM left on `odom`, a particle filter) | Run SLAM on `odom_imu` (`map_course gyro_odom:=true`), or drop `publish_odom_tf` |
| vehicle_state: `every /odom/imu step used the steering model` | no usable gyro | Check `ros2 topic hz /imu/bno/data` and the mount warning |
| `/steering/measured` always `NaN` while driving | under 0.3 m/s, no gyro, or the gyros disagree | Read `vehicle_state_node`'s 5 s status line: it says which |

## See also

- [docs/localization.md](../../docs/localization.md): why heading drifts today, and the plan an EKF fits into
- [docs/run-diagnostics.md](../../docs/run-diagnostics.md): recording runs, including these topics
- [imu-calibration](https://github.com/sfu-racerbot/imu-calibration): where the calibration comes from, and how to redo it
