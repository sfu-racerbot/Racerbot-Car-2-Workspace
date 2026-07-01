# Operations

Step-by-step procedures for actually using the car. For *why* things are wired this way, see [architecture.md](architecture.md). For hardware specifics, see [hardware-reference.md](hardware-reference.md).

## Every session, before anything else

```bash
source /opt/ros/jazzy/setup.bash
source ~/racerbot-ws/install/setup.bash
```
Every new terminal needs both lines, in that order, before any `ros2`/`colcon` command works.

**Safety checklist, every time before powering the drive motor:**
- [ ] Wheels off the ground (car propped up) for the first run of any new code or after any config change
- [ ] F710 controller is in **XInput mode** (switch on the back) and powered on
- [ ] You know where the VESC's power switch / battery disconnect is and can reach it
- [ ] If running autonomy (not just teleop): confirmed a human is ready to physically cut power — the joystick override is *not* active in that mode (see below)

## Manual driving (teleop)

```bash
ros2 launch f1tenth_stack bringup_launch.py
```
Hold **LB**, left stick = speed, right stick = steering. The car does not move on its own from this command alone — `joy_teleop`'s default state is neutral.

Sanity-check before trusting it near the ground:
```bash
ros2 topic echo /joy          # buttons[4] should read 1 while LB is held
ros2 topic echo /commands/servo/position   # should vary as you move the right stick
```

## Building a map

1. Start the driver stack (above).
2. In a second terminal:
   ```bash
   ros2 launch racerbot_launch slam_launch.py
   ```
3. Drive the car manually around the track (LB + sticks), covering the whole area you want mapped, ideally closing the loop back to your starting point.
4. Save the map:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f <map_name>
   ```
   This produces `<map_name>.yaml` and `<map_name>.pgm` in your current directory.

## Localizing against a saved map

1. Copy your saved `<map_name>.yaml` + `<map_name>.pgm` into `src/particle_filter/maps/`.
2. Edit `src/particle_filter/config/localize.yaml`, set `map_server.ros__parameters.map` to `<map_name>` (no extension). Two example maps (`levine`, `basement_fixed.map`) already ship in that folder from upstream — those are generic demo maps, not this track, don't confuse them for real data.
3. Rebuild so the map gets installed into the package's share directory:
   ```bash
   colcon build --symlink-install --packages-select particle_filter
   ```
4. Start the driver stack, then:
   ```bash
   ros2 launch particle_filter localize_launch.py
   ```
5. Open RViz, use "2D Pose Estimate" to give the particle filter its starting guess — it won't localize correctly without this initial seed.
6. Localization output your own planning code can consume: `/pf/viz/inferred_pose` (`geometry_msgs/PoseStamped`), or `/pf/pose/odom` (`nav_msgs/Odometry`, only published if `publish_odom: 1` in the config, which it is by default).

## Running autonomy (`gap_follow`, `pure_pursuit`, or your own node)

This is the one place the standard bringup isn't enough by itself — see [architecture.md](architecture.md#the-safety-model-read-this-before-writing-autonomy-code) for why. **Current workspace policy (see [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)): every autonomy node — `gap_follow`, `pure_pursuit`, and any new one — requires LB held to move the car, on top of the mux arbitration below. That means you only ever stop `joy_teleop`, never `joy_node`.**

1. Start the driver stack as normal:
   ```bash
   ros2 launch f1tenth_stack bringup_launch.py
   ```
2. **Prop the wheels up.** From here on the joystick's *mux* override is being removed — the deadman button is your only remaining safety net, so this still matters.
3. Stop **only** `joy_teleop`, leaving `joy_node`, the VESC, LiDAR, and mux running:
   ```bash
   pkill -f joy_teleop
   ```
4. Confirm `/teleop` has actually gone quiet (should time out within 0.2s):
   ```bash
   ros2 topic hz /teleop   # should report nothing / "no new messages"
   ```
5. Launch your autonomy node, e.g.:
   ```bash
   ros2 launch gap_follow gap_follow_launch.py
   # or:
   ros2 launch pure_pursuit pure_pursuit_launch.py waypoints_file:=...
   ```
6. **Hold LB** on the controller — no autonomy node in this workspace will publish a non-zero drive command without it (`enable_deadman: true` is the default and required policy for every node's config). Watch it before trusting it: `ros2 topic echo /drive` should show sensible values reacting to `/scan` only while LB is held, and drop to `0.0 / 0.0` the instant you release it.
7. **When you're done, restore `joy_teleop` before doing anything else:**
   ```bash
   ros2 run joy_teleop joy_teleop --ros-args -r __node:=joy_teleop --params-file install/f1tenth_stack/share/f1tenth_stack/config/joy_teleop.yaml &
   ```
   or just kill everything and re-run `bringup_launch.py` fresh.

If you're writing your own node, this deadman behavior is **required, not optional** — see [writing-your-own-node.md](writing-your-own-node.md#the-interface-contract) for the pattern to copy from `gap_follow_node.py`. If you ever see an autonomy node stuck at `0.0/0.0` even with LB held, first check `enable_deadman`/`joy_topic`/`deadman_button` in its config YAML, then confirm `joy_node` is actually still running (`ros2 node list | grep joy`) and publishing (`ros2 topic echo /joy`).

`gap_follow`'s tuning parameters (speed limits, steering limits, safety bubble radius, emergency stop distance, `deadman_button`, `joy_timeout_sec`, `enable_deadman`) live in `src/gap_follow/config/gap_follow.yaml`. Defaults are conservative (`max_speed: 2.0` m/s) — increase gradually, not all at once, and re-test wheels-off-ground after any change.

## Racing with the pure-pursuit stack

The map-based race controller — see [racing-autonomy.md](racing-autonomy.md) for how the algorithm works and how to tune it in depth. Same joystick-override consideration as the section above; folded into the procedure here.

### 1. One-time per track: record a racing line

Requires a saved map and working localization (both sections above) already set up for this track.

1. Start the driver stack, then localization:
   ```bash
   ros2 launch f1tenth_stack bringup_launch.py
   # in another terminal:
   ros2 launch particle_filter localize_launch.py
   ```
2. In RViz, give it a "2D Pose Estimate" seed, same as normal localization.
3. Start recording (choose your own output path):
   ```bash
   ros2 launch pure_pursuit waypoint_recorder_launch.py \
       output_file:=/home/racerbotcar-2/racerbot-ws/src/pure_pursuit/waypoints/my_track_raw.csv
   ```
4. Hold LB and drive one clean lap by hand, back to roughly your starting point.
5. `Ctrl+C` the recorder — it prints how many waypoints it recorded.

### 2. One-time per track: generate the velocity profile

```bash
ros2 run pure_pursuit generate_velocity_profile \
    --input src/pure_pursuit/waypoints/my_track_raw.csv \
    --output src/pure_pursuit/waypoints/my_track_profiled.csv \
    --v-max 4.0 --a-lat-max 6.0 --a-accel-max 2.5 --a-brake-max 6.0
```
Start conservative on these limits (the values above are already more cautious than the tool's own defaults) — see [racing-autonomy.md](racing-autonomy.md#choosing-a_lat_max--a_accel_max--a_brake_max--v_max) for how to raise them safely. The tool prints the resulting speed range and an estimated lap time when it's done.

A small synthetic example track is also checked in at `src/pure_pursuit/waypoints/example_stadium_raw.csv`, if you want to try the tool (and `pure_pursuit_node`, wheels off the ground) before you have a real recorded lap.

### 3. Every race run: drive it

1. Start the driver stack as normal:
   ```bash
   ros2 launch f1tenth_stack bringup_launch.py
   ```
2. **Prop the wheels up** for the first run of any new racing line or after any parameter change — same rule as every other autonomy node.
3. Stop **only** `joy_teleop`, leaving `joy_node` running — `pure_pursuit_node` has its own LB deadman check too (mandatory workspace policy, see [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)), so `joy_node` must stay up:
   ```bash
   pkill -f joy_teleop
   ros2 topic hz /teleop   # should report nothing
   ```
4. Launch localization + the race controller together:
   ```bash
   ros2 launch racerbot_launch race_launch.py \
       waypoints_file:=/home/racerbotcar-2/racerbot-ws/src/pure_pursuit/waypoints/my_track_profiled.csv
   ```
5. Give it a "2D Pose Estimate" seed in RViz — same as any other time you start localization.
6. **Hold LB** — `pure_pursuit_node` won't drive without it. Watch it before trusting it: `ros2 topic echo /drive` should show sensible, smoothly varying values once the pose seed is in and LB is held, and drop to `0.0 / 0.0` the instant you release it.
7. **When you're done, restore `joy_teleop`** — same as step 7 above, or just kill everything and re-run `bringup_launch.py` fresh.

`pure_pursuit`'s tuning parameters (lookahead, speed limits, steering limits, safety watchdogs, `enable_deadman`) live in `src/pure_pursuit/config/pure_pursuit.yaml` — see [racing-autonomy.md](racing-autonomy.md#parameter-reference) for what each one does.

## Shutting down cleanly

`Ctrl+C` each `ros2 launch` terminal (one `Ctrl+C` triggers a clean shutdown of every node that launch file started). If something's stuck:
```bash
pkill -f "joy_node|joy_teleop|vesc_driver_node|urg_node_driver|ackermann_mux|ackermann_to_vesc_node|vesc_to_odom_node|static_transform_publisher|gap_follow_node|slam_toolbox|particle_filter|pure_pursuit_node|waypoint_recorder_node"
```
Power down the VESC/battery last, after ROS nodes have stopped cleanly (avoids the driver logging a serial disconnect error, which is harmless but noisy).

## Common gotchas that aren't bugs

- **New terminal, permission denied on `/dev/sensors/vesc` or `/dev/input/js0`**: group membership (`dialout`, `input`) only applies to sessions started *after* the group was added. Open a fresh terminal, or `newgrp dialout && newgrp input` in the current one.
- **Servo position shows `0.5304` and nothing seems to be happening**: that's neutral (center), not zero-as-in-broken. See the formula in [hardware-reference.md](hardware-reference.md#vesc-motor--steering-controller).
- **You published to `/drive` and nothing happened**: teleop is still running and masking it. This is the safety model working as designed — see above.
