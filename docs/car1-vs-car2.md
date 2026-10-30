# Car 1 vs car 2: what must stay different

> **Who this is for:** anyone (person or Claude) about to port a change from the [car 1 workspace](https://github.com/sfu-racerbot/Racerbot-Car-1-Workspace) into this one, or wondering why a car 1 instruction doesn't work here.
> **Read first:** [git-setup.md](git-setup.md#remote): how the two repos relate (sister repos, not upstream and fork).
> **You'll be able to:** take car 1's improvements without breaking car 2's LiDAR, camera, steering, dashboard or IMU setup, and prove you didn't.
> **Time:** 5 minutes to read; the check at the end takes seconds.

The two cars run almost the same code, but they are **not the same hardware**. Car 1's [workspace](glossary.md) (its whole repo of ROS packages) started as a copy of this one. Improvements now flow both ways and are ported by hand.

A port that copies car 1's files over car 2's breaks car 2 **silently**. It builds fine. Then the [LiDAR](glossary.md) (the spinning laser scanner) never publishes, or the doctor looks for a webcam that was never fitted, or the steering stops short of what car 2 can do.

This page is the list of things that must survive every port. It is also the instruction to Claude: **when asked to port anything from car 1, read this page first, keep every car 2 value below, and run the check at the end before calling the port done.** Car 1 keeps the same page from its side, in its own repo.

## The differences

### Hardware

| Thing | Car 2 (this repo) | Car 1 | Where car 2's version lives |
|---|---|---|---|
| LiDAR | Hokuyo **UST-10LX over Ethernet**, `192.168.0.10:10940`, reached from `enP8p1s0` through the `nmcli` profile `hokuyo` (`192.168.0.15/24`) | Hokuyo UST-30LX over **USB serial** at `/dev/sensors/hokuyo` | `f1tenth_stack/config/sensors.yaml` (`ip_address`/`ip_port`, `serial_port` commented out); the `hokuyo` NetworkManager profile on the Jetson |
| Camera | Intel **RealSense D435i**, `realsense-ros` submodule, started with `realsense_camera_launch.py`; the dashboard streams its ROS image topic | a plain UVC USB webcam at `/dev/video0`, `webcam_launch.py`, no RealSense | `racerbot_launch/launch/realsense_camera_launch.py`; `image_topic` under `usb_cam_stream_node` in `racerbot_launch/config/web_dashboard_rb2.yaml` |
| Steering and speed calibration | servo `= -0.78 × steering angle + 0.5904`, clamped to `0.15 … 0.95`; `speed_to_erpm_gain` `4614.0` (and `-4447.983786` for `vesc_to_odom_node`) | `-0.83 × angle + 0.366`, `0.05 … 0.75`; `3650.0` / `-4040.0` | `f1tenth_stack/config/vesc.yaml`, quoted in [hardware-reference.md](hardware-reference.md) |
| Steering limits | full stick **0.436 rad**; autonomy `max_steering_angle` **0.349**; race planner `profile_max_steering_angle` **0.30**. From car 2's 2026-10-16 circles | 0.38 / 0.33 / 0.30, from car 1's 2026-10-21 circles | `joy_teleop.yaml` scale; `gap_follow.yaml`, `pure_pursuit.yaml`, `auto_map_race.yaml` |
| IMU | **No BNO085 yet** (to be mounted in the last week of October 2026). Only the VESC's own IMU, `/sensors/imu/raw`. No `~/imu-calibration` checkout | a BNO085 on the Jetson's I2C bus, read through imu-calibration's `bno_i2c_bridge.py` | see [IMU](#imu) below |
| Geometry | wheelbase `0.36`, LiDAR `0.26 0.0 0.11`, **measured on car 2** 2026-08-24 | the same numbers, assumed, not measured on car 1 | [hardware-reference.md](hardware-reference.md#physical-dimensions-used-in-config) |
| VESC udev rule | `GROUP="dialout"`, `MODE="0660"`: the user must be in `dialout` | `MODE=0666`, no group | `/etc/udev/rules.d/99-vesc.rules` on the Jetson (not in git) |

### IMU

An **IMU** (inertial measurement unit) is the chip that measures rotation (gyro) and acceleration. `imu_bridge` was written on car 2 (PR #2, 2026-10-24) for the VESC's IMU and an optional BNO085. Car 1 extended it around its own BNO085 (fused speed, measured steering, gyro odometry), and that version was ported back here the same day. Car 2 can't run it the way car 1 does yet:

- `imu_bridge_node` reads the IMU mount rotations from `~/imu-calibration/config/car.yaml`. Without that checkout it **refuses to start**.
- Car 1's `vehicle_state_launch.py` starts a bridge for a BNO085 on the Jetson's I2C bus. Car 2 has no BNO085.

So on car 2 the IMU side is **off by default**:

| Setting | Car 2 default | Car 1 default | Where |
|---|---|---|---|
| `map_course` `imu:=` | `false` | `true` | `racerbot_launch/launch/map_course_launch.py` |
| `vehicle_state_launch.py` `bno:=` | `false` | `true` | `imu_bridge/launch/vehicle_state_launch.py` |
| `gap_follow_launch.py` `steering_damping:=` (starts the IMU stack) | `false` | `true` | `gap_follow/launch/gap_follow_launch.py` |
| `enable_steering_damping` (the gyro term) | `false` | `true` | `gap_follow/config/gap_follow.yaml` |

The IMU feature flags ported from car 1's Part B-2/3 (2026-10-29) default **off on both cars**.

- In `gap_follow_launch.py`: `imu_all`, `fused_speed`, `imu_understeer`, `imu_traction` and `imu_impact_stop`.
- In `map_course`: `imu_all`, `gyro_odom`, `fused_speed` and `imu_lap_turns`.

One difference follows from the table above. On car 2, `map_course` needs `imu:=true` written out before any of its IMU flags, because its `imu` default is `false`.

`gap_follow_launch.py` has an `imu:=` too, but it is only a permission: it starts nothing by itself. It defaults to `true` on both cars. So `src/racerbot_launch/test/test_imu_flags_both_launches.py` expects `map_course` `false` and `gap_follow_launch.py` `true` here, and both `true` on car 1.

Everything else in `imu_bridge` is the same code as car 1's, apart from car 2's own speed filter: `vehicle_state.py`'s four-state filter, `x = [v, a, b_vesc, b_bno]` (car 2 PR #4).

Car 1's `gap_follow` fused-speed gate was ported without car 1's `vehicle_state`. It reads car 2's `/odom/fused`. It trusts it only while `/imu/status` is `ok` and `/imu/data` is fresh, and falls back to `/odom` otherwise.

### IMU feature thresholds

Car 1's Part B-2/3 came with numbers **measured on car 1**. They are in car 2's files as placeholders, labelled as car 1's, and every feature that uses them ships **off**.

Don't turn one on for car 2 until car 2 has recorded its own value. Then change the number and its label together. `test_car_1_imu_values_stay_labelled_as_car_1s` checks the label is there while car 1's number is.

| Value | Car 1's number (used on car 2 for now) | Where car 1 measured it | Where it lives | How car 2 gets its own |
|---|---|---|---|---|
| Impact stop threshold `imu_impact_threshold` | 33.73 m/s² (1.5 × its largest recorded horizontal spike, 22.48 m/s²) | car 1 run `20261029-132902` | `gap_follow/config/gap_follow.yaml`, `IMPACT_DEFAULTS` in `gap_follow_node.py` | record a wheels-off bump on car 2 with the BNO085 running ([gap-follow.md](gap-follow.md#impact-stop-imu)) |
| Traction slip `imu_traction_slip_threshold` / `release_slip` | 0.246 / 0.123 m/s (1.5 × the combined p99 of `/odom` − `/odom/fused`) | car 1 runs `20261026-194514`, `-194913` | `gap_follow.yaml`, `TRACTION_DEFAULTS` | the same percentile sweep over car 2 runs recorded with `imu:=true` |
| Understeer bands `imu_understeer_engage_ratio` / `release_ratio`, `steering_lag_sec` | 0.75 / 0.85, 0.15 s | car 1 runs `20261026-194514`, `-194913` (ratios 0.84–0.97) | `gap_follow.yaml`, `UNDERSTEER_DEFAULTS` | `tools/run_analysis/grip_replay.py` on a car 2 run; car 2's own steering lag from `tools/run_analysis/yaw_tracking.py` |
| Simulated IMU noise (BNO and VESC, gyro and accelerometer std) | `BNO_NOISE`, `VESC_NOISE` | car 1 run `20261026-194514`, a stationary stretch | `racerbot_sim/sim_imu.py` | a stationary stretch of a car 2 bag with the BNO085 running |
| Simulated IMU positions `BNO_X_OFFSET` / `VESC_X_OFFSET` | 0.36 m / 0.18 m ahead of the rear axle | car 1's mounts | `racerbot_sim/sim_imu.py` (and `bno_x_offset` in `vehicle_state.yaml`) | tape-measure car 2's mounts (step 3 of the list above) |
| `map_course` `mapping_max_speed` default | 3.5 m/s | car 1's gyro SLAM profile, replay-validated to about 3.3 m/s | `racerbot_launch/launch/map_course_launch.py` | a faster recorded gyro mapping run on car 2 with a clean `slam_report` |

The lap-time floor in `auto_map_race.yaml` (`minimum_lap_duration_sec: 4.0`) holds on both cars; its comment works the bound out from car 2's 0.99 m minimum turning radius (car 1's is 1.05 m).

**When the BNO085 is mounted on car 2**, do these in order, and update this section as you go:

1. Decide how it is wired. On the Jetson's I2C bus, like car 1, `vehicle_state_launch.py bno:=true` starts the bridge as it does there. Behind a microcontroller on USB instead, pass `bno_port:=/dev/ttyACM…` and don't start the I2C bridge.
2. Clone [imu-calibration](https://github.com/sfu-racerbot/imu-calibration) to `~/imu-calibration`. Set car 2's numbers in its `config/car.yaml` (wheelbase, odometry gain, the BNO's position and mount), then run its calibration steps.
3. Set `bno_x_offset` in `imu_bridge/config/vehicle_state.yaml` to the BNO's distance ahead of the rear axle. The current `0.36` is car 1's front-axle mount.
4. Do the forward-push check and the A/B mapping test in [launchstart.md](launchstart.md#testing-the-imu-in-mapping-ab), with `imu:=true`.
5. Only then flip the four defaults above to `true`, and change `test_car2_hardware_config.py`'s three IMU tests (`map_course`, `vehicle_state_launch`, steering damping) in the same commit. Steering damping's own tests also pin car 2's off value: `test_race_defaults.py` and two tests in `test_gap_follow_launch_steering_damping.py`; flip them with it.

`robot_localization` (only needed for `imu_ekf_shadow_launch.py`) is also not installed on car 2 yet.

### Software and setup

| Thing | Car 2 | Car 1 | Where car 2's version lives |
|---|---|---|---|
| Web dashboard id | **`rb2`** | `rb1` | `racerbot_launch/config/web_dashboard_rb2.yaml`, loaded by `dashboard_launch.py` |
| Self-check | `racerbot_doctor` checks the Ethernet link, host IP, ping and TCP port of the LiDAR, and the RealSense (USB 3 speed, `/camera/camera/color/image_raw` rate) | checks the USB Hokuyo ladder and a UVC webcam. The joystick `input`-permission check and the *Drive path* section are the same on both cars | `racerbot_doctor/car.py` and the LiDAR/camera ladders in `checks.py`, plus their tests |
| `neural_racer` safety wrapper | `max_steering_angle` **0.349**, matching `gap_follow.yaml` (its own test checks they agree). The learned policy itself was trained on car 1 and never asks for more than 0.33 (`observation.STEER_LIMIT`) | 0.33 | `neural_racer/config/neural_racer.yaml`, `WrapperConfig` in `neural_racer/safety_wrapper.py` |
| Where runs are saved | inside the repo, `~/racerbot-ws/data/` ([data/README.md](../data/README.md)): `auto_record` and `record_run.py` into `data/racerbot_runs/`, the simulator's race referee into `data/racerbot_sim/races/`. `tools/run_analysis/grip_replay.py` looks up a bare run name there too | `~/.ros/racerbot_runs/`, `~/.ros/racerbot_sim/races/` | every writer's default is pinned by `racerbot_launch/test/test_run_output_paths.py`; `grip_replay.py`'s by `test_car2_hardware_config.py`. Run-folder paths in `test_map_course_imu.py` are car 2's |
| System log (`tegrastats`, per-node CPU/memory) | `tegrastats` runs without sudo on car 2 (checked 2026-10-29, JetPack 7.2) | the same (checked 2026-10-26) | `race_diagnostics/system_log.py`, started by `record_run.py` (`system_log:=false` turns it off) |
| [Odometry](glossary.md) calibration label | car 2's | `vehicle_model: "... (car 1)"` | `odom_calibration/config/odom_calibration.yaml` |
| Simulator vehicle model | **car 2's** servo map and limits | the same (car 2's) model, a known gap on car 1 | `racerbot_sim/sim_bridge.py`, `tools/f1tenth_sim/sim_fidelity/calibration.py` |

## Porting from car 1, step by step

1. **Port only what was asked for.** Car 1 is not upstream. The repos share no git history, so a port is a patch: `git fetch car1`, `git diff <last ported car 1 commit> car1/main`, then `git apply --3way`. The last port took `car1/main` up to `8a88dcc` (2026-10-29, car 1's PR #8, Part B-2/3; before it `4bea6f2`, PRs #3 to #7; [CHANGELOG.md](../CHANGELOG.md)). On top of that, the launch boost alone was taken from car 1's PR #10 (`git diff bf56121 ffbfac0`, 2026-10-30). Car 1's PR #9 (`db3e9e9`) has not been ported, so the next full port still starts from `8a88dcc` and should expect the launch boost hunks to be there already.
2. **For every file the patch touches, check it against the tables above.** Merge the change *into* car 2's file by hand. Never replace the file.
3. **Never bring in** car 1's USB LiDAR settings, `webcam_launch.py`, `web_dashboard_rb1.yaml`, car 1's calibration or steering limits, car 1's `racerbot_doctor` LiDAR/camera checks, or `test_car1_hardware_config.py`.
4. **`f1tenth_system` has its own extra checklist** of local fixes, two of which fail unsafely: see [git-setup.md](git-setup.md#updating-f1tenth_system--manual-no-submodule-machinery).
5. **Run the checks below.** If one fails, the port broke car 2. Fix the port; don't edit the test.

**Terminal 1** — no ROS needed:

```bash
cd ~/racerbot-ws
python3 -m pytest src/racerbot_launch/test/test_car2_hardware_config.py -v
```

**Working when:** all 16 tests pass. They fail if any of car 1's values come back: USB LiDAR settings, webcam launch, `rb1` dashboard config, calibration, steering limits, odometry gain. They also fail if the doctor loses its Ethernet LiDAR or RealSense checks.

And they fail if the IMU side is switched on by default before car 2 has a calibrated BNO085: gap_follow's steering damping, and its understeer, traction and impact features. Finally, they fail if car 1's IMU numbers lose their "car 1's" label.

**If it doesn't:** the failing test's name says which row of the tables above was clobbered.

Then the hardware check.

**Terminal 2** — with the car powered and `bringup_launch.py` running in another terminal:

```bash
ros2 run racerbot_doctor doctor
```

**Working when:** the LiDAR section shows the Ethernet link, `192.168.0.10` and `/scan` passing, and nothing mentions `/dev/sensors/hokuyo` or a webcam.

## Updating this page

When you find a new car 2 difference, add a row here **and**, if it lives in a file, a check to `src/racerbot_launch/test/test_car2_hardware_config.py`. A difference that's only written down will eventually be overwritten by a port.

## See also

- [hardware-reference.md](hardware-reference.md): car 2's devices, ports and measured numbers in detail.
- [git-setup.md](git-setup.md): how the repos relate, and the `f1tenth_system` sync checklist.
- [known-issues.md](known-issues.md): loose ends found on both cars.
