# imu_bridge

> **Who this is for:** anyone who wants the car's IMU readings in ROS. That means recording them, checking them, or feeding them to an EKF.
> **Read first:** [docs/concepts.md](../../docs/concepts.md), so you know what a *topic*, *node* and *launch file* are.
> **You'll be able to:** get calibrated, correctly oriented IMU topics, and check they're right by turning the car by hand.
> **Time:** about 15 minutes the first time, mostly the setup.

An **IMU** (inertial measurement unit) is a chip with two sensors in it:
- a **gyro**, which measures how fast the car is rotating
- an **accelerometer**, which measures acceleration, including gravity

This package takes the IMU readings the car already produces and fixes their units, axes and bias. It publishes them as standard ROS messages that other tools can use directly.

**This is support code. It cannot move the car.** It publishes no `/drive`, `/ackermann_cmd` or `/commands/*`, so you can start and stop it freely at any time, including mid-run. See [docs/adding-your-own-code.md](../../docs/adding-your-own-code.md) for what that category means.

## Highlights

- **Converts the VESC's IMU to SI units and car axes.** The [VESC](../../docs/glossary.md#vesc) (the motor controller) reports g and deg/s, in whatever direction its board happens to face. On car 2 that's rotated about 85° from the car. This node outputs m/s² and rad/s, with x forward, y left and z up.
- **Uses the club's existing calibration, unchanged.** It reads `car.yaml` and the calibration files from the [imu-calibration](https://github.com/sfu-racerbot/imu-calibration) repo, and applies the same formula as that repo's `racer_imu`. A test checks the two give the same answer to 1e-12 on the car's real calibration file.
- **Removes leftover gyro bias at startup.** For the first 3 s it averages the gyro while the car sits still. It only counts samples where the wheels report under 0.03 m/s *and* every gyro axis is under 0.05 rad/s. Any motion throws the window away and starts again.
- **Gives the wheel [odometry](../../docs/glossary.md#odometry--odom) an honest uncertainty.** `/odom` arrives claiming zero uncertainty, which an EKF would treat as perfect. This node republishes it with a speed uncertainty of 0.08 m/s, and marks everything else as untrusted.
- **Ships a "shadow" EKF config.** It runs the EKF alongside everything else for comparison, without changing what any driving node uses.
- **Optional second IMU.** A BNO085 on its own USB port (~200 Hz) is read through `racer_imu`'s driver if you set `bno_port`.
- **All the math runs without ROS.** `convert.py` has no `rclpy` import, so its tests run with a bare `pytest`.

### Why it exists

Nothing on this car measures how much it actually turned. Heading comes from integrating the *commanded* steering angle (see [docs/localization.md](../../docs/localization.md)), so it drifts with every bit of servo error, steering slop and tyre slip.

A gyro measures rotation directly. This package is the first step: get that measurement into ROS, correctly calibrated, and recorded alongside everything else. Then you can tell from real runs whether it actually helps.

## What this actually is

One node, `imu_bridge_node`, sits between the hardware drivers and anything that wants IMU data:

```
vesc_driver_node ── /sensors/imu/raw (g, deg/s, VESC board axes) ──┐
vesc_to_odom_node ─ /odom (wheel speed) ───────────────────────────┤
BNO085 (optional, own USB port) ───────────────────────────────────┤
                                                                   ▼
                                                         imu_bridge_node
                                    calibrate · rotate to car axes · startup bias
                                                                   │
           /imu/data, /imu/vesc/data, /imu/bno/*, /imu_bridge/wheel_odom
                                                                   │
                                     ┌─────────────────────────────┴───────┐
                                     ▼                                     ▼
                     EKF in shadow mode (optional)               race_diagnostics
                     /odometry/filtered, no TF                   records all of it
```

It never opens the VESC's serial port. `vesc_driver_node` owns that port during bringup, and the IMU readings reach this node over `/sensors/imu/raw`.

An **EKF** (extended Kalman filter) combines two imperfect sensors into one better estimate. Here it would combine wheel speed (good at *how fast*) with the gyro (good at *how much it turned*).

**"Shadow mode"** means the EKF publishes its estimate on `/odometry/filtered`, and nothing reads it. In particular, it does **not** publish the `odom -> base_link` [transform](../../docs/glossary.md#tf--transform--frame) that every driving node uses. That stays with `vesc_to_odom_node`, exactly as before. The point is to record the EKF next to SLAM and the particle filter, and compare them offline.

## Before you start

- [ ] **The VESC's IMU must be switched on.** It was measured on 2026-08-19 sending all zeros, meaning it was off or not fitted (see [docs/localization.md](../../docs/localization.md)). It was calibrated through the imu-calibration repo in September 2026, which means it works over that repo's own serial driver. This node prints a warning every 5 s if it still sees all zeros.
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

**Terminal 2** runs the IMU bridge. **Keep the car still for the first 3 seconds** after this starts.

```bash
ros2 launch imu_bridge imu_bridge_launch.py
# with the BNO085 plugged in:
ros2 launch imu_bridge imu_bridge_launch.py bno_port:=/dev/ttyACM1
```

**Working when** you see, within about 5 s:
- a line saying `vesc: startup gyro bias removed [...] rad/s`. The numbers should be small, about 0.01 rad/s or less.
- every 5 s, `rates: vesc 50.0 Hz, odom ... Hz`

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

## Topics

Each [topic](../../docs/glossary.md#topic) below is a named channel you can `ros2 topic echo`. The *frame* says which axes the numbers are in ([glossary](../../docs/glossary.md#tf--transform--frame)).

| Topic | Type | Frame | What it is |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | `base_link` | **The one to use.** The primary IMU, calibrated, in car axes. `primary: auto` picks the BNO085 if `bno_port` is set, otherwise the VESC. |
| `/imu/vesc/data` | `sensor_msgs/Imu` | `base_link` | The VESC IMU, calibrated, in car axes (~50 Hz) |
| `/imu/bno/raw` | `sensor_msgs/Imu` | `bno_sensor` | The BNO085 as received, in SI units and in the sensor's own axes, with its orientation quaternion. Only when `bno_port` is set. No transform exists for this frame. |
| `/imu/bno/data` | `sensor_msgs/Imu` | `base_link` | The BNO085, calibrated, in car axes. Only when `bno_port` is set. |
| `/imu_bridge/wheel_odom` | `nav_msgs/Odometry` | `odom` → `base_link` | `/odom`, unchanged except for the twist covariance: 0.08² on forward speed, 1000 on everything else |
| `/odometry/filtered` | `nav_msgs/Odometry` | `odom` | Published by the shadow EKF, not this node |

Every `base_link` message reports **no orientation**: `orientation_covariance[0] = -1`, the `sensor_msgs/Imu` convention for "not provided". Each one does carry an angular velocity and acceleration covariance from `gyro_sigma` and `accel_sigma`.

It subscribes to `/sensors/imu/raw` and `/odom`, using sensor-data QoS.

Every setting is in [`config/imu_bridge.yaml`](config/imu_bridge.yaml), with a comment for each.

## Known limits

- **Acceleration is measured where the IMU sits, not at the car's centre.** In a turn, an IMU off-centre also feels centripetal acceleration. The VESC is about 9 cm left of centre. Yaw rate is the same everywhere on a rigid car, so it's unaffected. The lever-arm correction lives in `racer_imu`'s `CenterFuser` and isn't ported here.
- **VESC samples are timestamped when they arrive, at 50 Hz.** `vesc_driver_node` polls every 20 ms and stamps each reply with the time it was received, so every reading is late by the serial round-trip. The BNO085 path uses the sensor's own clock, mapped onto ROS time.
- **No orientation is published.** The VESC's own orientation estimate is in board axes and is dropped. The BNO085's quaternion is kept only on `/imu/bno/raw`.
- **The startup bias needs `/odom`.** Unknown wheel speed counts as moving. Without `vesc_to_odom_node` the bias never completes, and the node says so every 5 s.
- **Not yet run on the car.** As of this package's first commit, `convert.py` is tested and the node and launch files compile. The node itself hasn't been started against real hardware. Do the hand check above before trusting any of its output.

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

No ROS sourcing needed (no `source /opt/ros/jazzy/setup.bash` first). Without the imu-calibration repo present, the two `racer_imu` cross-check cases skip and say why. Every other test runs.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Node exits with `car.yaml not found` | imu-calibration not cloned at `racer_imu_root` | Clone it there, or pass `racer_imu_root:=...` |
| `startup gyro bias removed` never appears | car moving, or no `/odom` | Keep it still; check `ros2 topic hz /odom` |
| Warning: all zeros | VESC IMU disabled | VESC Tool → App Settings → IMU, sample rate > 0 |
| Turning left gives negative `z` | wrong `rpy_deg` for this IMU in `car.yaml` | Redo the mount step in imu-calibration (`tools/calibrate_imu.py --step mount`) |
| `ModuleNotFoundError: racer_imu` with `bno_port` set | repo missing at `racer_imu_root` | Same as the first row |
| `no BNO085 data` | wrong port, or the MCU isn't streaming | Check `ls /dev/ttyACM*`. The VESC is usually `ttyACM0`. |

## See also

- [docs/localization.md](../../docs/localization.md): why heading drifts today, and the plan an EKF fits into
- [docs/run-diagnostics.md](../../docs/run-diagnostics.md): recording runs, including these topics
- [imu-calibration](https://github.com/sfu-racerbot/imu-calibration): where the calibration comes from, and how to redo it
