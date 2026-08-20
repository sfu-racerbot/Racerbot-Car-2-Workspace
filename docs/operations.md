# Operations

> **Who this is for:** anyone actually using the car — driving it, mapping a track, or running autonomy. No ROS2 experience assumed.
> **Read first:** [concepts.md](concepts.md) for what a launch file and a topic are, and [glossary.md](glossary.md) for the vocabulary. Skim both; come back when a word bites.
> **You'll be able to:** drive the car by hand, build a map of a track, and run the autonomous race stack on it.
> **Time:** 20 minutes for your first manual drive. An afternoon to get through the racing workflow.

This is the "how do I do X" doc. For *why* things are wired this way, see [architecture.md](architecture.md). For hardware specifics — ports, addresses, exact config values — see [hardware-reference.md](hardware-reference.md).

## Contents

- [Every session, before anything else](#every-session-before-anything-else)
- [The two-layer pattern used in every procedure below](#the-two-layer-pattern-used-in-every-procedure-below)
- [Manual driving (teleop)](#manual-driving-teleop)
- [Building a map](#building-a-map)
- [Building a map autonomously (no steering required)](#building-a-map-autonomously-no-steering-required)
- [Localizing against a saved map](#localizing-against-a-saved-map)
- [Running autonomy](#running-autonomy-gap_follow-pure_pursuit-or-your-own-node)
- [Racing with the pure-pursuit stack](#racing-with-the-pure-pursuit-stack)
- [Shutting down cleanly](#shutting-down-cleanly)
- [Common gotchas that aren't bugs](#common-gotchas-that-arent-bugs)

## Every session, before anything else

Run these two lines in **every** new terminal, in this order, before any `ros2` or `colcon` command:

```bash
source /opt/ros/jazzy/setup.bash
source ~/racerbot-ws/install/setup.bash
```

**Working when:** nothing is printed. That's success — these commands are silent.

**If it doesn't:** if a later `ros2` command says "command not found", or can't find a package you know exists, you missed one of these two lines. There's no way around repeating them; environment settings don't carry between terminals. [Why](concepts.md#why-you-have-to-source-things-and-what-that-means).

### Safety checklist, every time before powering the drive motor

- [ ] **Wheels off the ground** (car propped up) for the first run of any new code, or after any config change.
- [ ] F710 controller is in **XInput mode** (switch on the back) and powered on.
- [ ] You know where the [VESC](glossary.md#vesc)'s power switch / battery disconnect is (the VESC is the motor controller board), and you can reach it.
- [ ] If running autonomy rather than manual driving: a human is standing by, ready to cut power.

That last point is worth stating precisely, because it is easy to misread.

**When you run autonomy, you still hold LB, and releasing it still stops the car.** Every autonomy node in this workspace enforces the [LB deadman](glossary.md#deadman) itself, in its own code.

What you *don't* have during autonomy is the second, separate protection that manual driving gives you: the [mux](glossary.md#mux--multiplexer) override.

During autonomy you deliberately don't run `teleop_launch.py`. That means there's no higher-priority manual channel sitting between the driving code and the motor.

So a human on the power switch matters here specifically: LB is your only layer instead of two. See [the safety model](architecture.md#the-safety-model-read-this-before-writing-autonomy-code).

## The two-layer pattern used in every procedure below

Every procedure in this doc is built the same way: one foundation launch, then exactly one control layer on top of it, each in its own terminal. Full explanation in [architecture.md](architecture.md#the-node-graph); the short version:

**Layer 1 — the foundation.** `ros2 launch f1tenth_stack bringup_launch.py` starts the gamepad reader (`joy_node`), the VESC chain that talks to the motor, the LiDAR, and the arbitration mux.

**It never moves the car by itself, and that's deliberate.** Nothing publishes to `/teleop` or `/drive` until you launch something on top of it. Run it alone and the hardware wakes up, then nothing happens.

**Layer 2 — exactly one control layer, in a second terminal.** This is the part that actually decides what the car does: `teleop_launch.py` for manual driving, or `gap_follow_launch.py`, `pure_pursuit_launch.py`, or your own node for autonomy.

The foundation terminal stays up the whole time you're using the car. To switch between manual driving and autonomy, `Ctrl+C` only the control-layer terminal and launch a different one. The foundation terminal — and the VESC and LiDAR connections it holds open — never needs touching.

## Manual driving (teleop)

Start here. Do this before anything autonomous.

**Terminal 1** — the foundation. Leave it running.

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** the output settles and stops scrolling, with no repeating red `ERROR` lines. The car sits still and does nothing — correct, not broken.

**Terminal 2** — the control layer.

```bash
ros2 launch f1tenth_stack teleop_launch.py
```

**Working when:** output settles. The car still doesn't move — `joy_teleop` starts in neutral, and nothing happens until you hold LB.

Now drive: **hold LB**, left stick = speed, right stick = steering.

### Check it before trusting it near the ground

**Terminal 3** — one-off checks.

```bash
ros2 topic echo /joy          # buttons[4] should read 1 while LB is held
ros2 topic echo /commands/servo/position   # should vary as you move the right stick
```

**Working when:** `buttons[4]` flips between `0` and `1` as you press and release LB, and the servo position changes as you move the right stick. If `buttons[4]` never reaches `1`, the controller is probably in DirectInput rather than XInput mode — check the switch on its back.

## Building a map

Driving a track while [SLAM](glossary.md#slam) builds a map of it.

1. Start the foundation and manual control as above — `bringup_launch.py` in terminal 1, `teleop_launch.py` in terminal 2.

2. **Terminal 3** — start mapping:

   ```bash
   ros2 launch racerbot_launch slam_launch.py
   ```

   **Working when:** it settles without repeating errors. Run [`web_dashboard`](web-dashboard.md) in another terminal if you want to watch the map appear live in a browser — recommended the first few times, since it's the only easy way to see whether the map is any good.

3. Drive the car manually (LB + sticks) around the whole area you want mapped. Finish back where you started — closing the loop is what lets SLAM correct its own drift.

4. **Terminal 4** — save the map:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f <map_name>
   ```

   **Working when:** it prints that it wrote the map, and you have `<map_name>.yaml` and `<map_name>.pgm` in the directory you ran it from.

## Building a map autonomously (no steering required)

Same result as above, except `gap_follow` drives the lap instead of a human — see [racing-autonomy.md](racing-autonomy.md#phase-1-map-the-track-slam) for why this needs no new code.

**You still cannot walk away.** The [mandatory LB-deadman policy](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car) applies here exactly as everywhere else: a human holds LB the entire time, ready to release it.

"Autonomous" here means nobody touches the sticks. It does not mean nobody is supervising.

1. **Terminal 1** — the foundation. Leave it running.

   ```bash
   ros2 launch f1tenth_stack bringup_launch.py
   ```

2. **Prop the wheels up for the first attempt on any new track.**

   `gap_follow` starts driving on its own the moment the next command launches.

   Do **not** also launch `teleop_launch.py` here. Its always-on neutral `/teleop` would mask `gap_follow`'s `/drive` entirely, regardless of LB, and the car would simply never move — see [the safety model](architecture.md#the-safety-model-read-this-before-writing-autonomy-code).

3. **Terminal 2** — SLAM and `gap_follow` together, at a deliberately cautious default speed:

   ```bash
   ros2 launch racerbot_launch autonomous_mapping_launch.py
   ```

   By default the car maps at `gap_follow`'s own tuned speeds from `config/gap_follow.yaml`. Its sensed limits — curvature, clearance, the safety bubble — do the slowing down.

   For a track nobody has driven yet, cap it explicitly: `mapping_max_speed:=1.5`. That is worth doing the first time and rarely after. The launch file (the script that starts a set of nodes together — see [glossary.md](glossary.md)) documents both arguments in its own docstring.

   A cap also scales the settings defined relative to `max_speed` (`corner_speed`, `corner_speed_wide`). `gap_follow_node` checks their ordering at startup and [exits rather than driving](troubleshooting.md#auto_map_race_launchpy-never-moves-no-racing-line-profile-is-active-waiting-for-waypoints_file-to-be-loaded) if only `max_speed` moves, so they are moved together for you.

4. **Hold LB.** The car will not move at all otherwise — this is `gap_follow`'s own deadman check, independent of the mux.

   **Working when:** the car starts driving itself, slowly, steering away from walls, and stops the instant you release LB.

   Watch it for a while before trusting it near anything you care about. `gap_follow`'s reactive gap-following is robust but not infallible, especially at higher speeds or in tight, cluttered spaces.

5. Optional but recommended: run [`web_dashboard`](web-dashboard.md) in another terminal and watch the map build live, so you can tell when the loop has closed and it's safe to stop.

6. Once the map looks good, release LB (or `Ctrl+C` the launch), then save it exactly as in the manual procedure above:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f <map_name>
   ```

## Localizing against a saved map

[Localization](glossary.md#localization) is working out where the car is on a map it already has, using a [particle filter](glossary.md#particle-filter).

1. Copy your saved `<map_name>.yaml` and `<map_name>.pgm` into `src/particle_filter/maps/`.

2. Edit `src/particle_filter/config/localize.yaml` and set `map_server.ros__parameters.map` to `<map_name>` (no file extension).

   Two example maps (`levine`, `basement_fixed.map`) already ship in that folder from upstream. They are generic demo maps, not this track — don't mistake them for real data.

3. **Terminal 3** — rebuild, so the map gets installed into the package's share directory:

   ```bash
   colcon build --symlink-install --packages-select particle_filter
   ```

   **Working when:** it finishes with `Summary: 1 package finished` and no failures.

4. Start the foundation in terminal 1 as usual, then **terminal 2**:

   ```bash
   ros2 launch particle_filter localize_launch.py
   ```

5. Open RViz and use **2D Pose Estimate** to give the particle filter its starting guess.

   **This step is not optional** — the filter will not localize correctly without an initial seed, because it has no other way to know which part of the map it's looking at.

   **Working when:** the cloud of pose guesses collapses onto a single tight cluster near where the car actually is, and tracks the car as you push it around.

6. Localization output your own planning code can [subscribe](glossary.md#publish--subscribe) to (that is, ask to receive):

   | Topic | Type | Note |
   |---|---|---|
   | `/pf/viz/inferred_pose` | `geometry_msgs/PoseStamped` | always published |
   | `/pf/pose/odom` | `nav_msgs/Odometry` | only if `publish_odom: 1` in the config, which is the default |

## Running autonomy (`gap_follow`, `pure_pursuit`, or your own node)

**Standing workspace policy: every autonomy node — `gap_follow`, `pure_pursuit`, and any new one — requires LB held to move the car**, on top of the mux arbitration. See [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car).

1. **Terminal 1** — the foundation. Leave it running.

   ```bash
   ros2 launch f1tenth_stack bringup_launch.py
   ```

2. **Prop the wheels up.**

   `bringup_launch.py` never starts `teleop_launch.py`, so there is no mux override sitting between your node and the VESC. The deadman button is your only safety net here, which is exactly why this step still matters.

3. **Terminal 2** — launch your autonomy node. Nothing needs stopping first.

   ```bash
   ros2 launch gap_follow gap_follow_launch.py
   # or:
   ros2 launch pure_pursuit pure_pursuit_launch.py waypoints_file:=...
   ```

4. **Hold LB.** No autonomy node in this workspace publishes a non-zero drive command without it (`enable_deadman: true` is the default, and required policy, in every node's config).

   **Working when:** `ros2 topic echo /drive` in a third terminal shows sensible values reacting to `/scan` while LB is held — and drops to `0.0 / 0.0` the instant you release it. Check this with the wheels up before you put the car on the floor.

5. **When you're done:** `Ctrl+C` your autonomy node's terminal. The foundation terminal can keep running — launch `teleop_launch.py` in its place to go back to manual driving, or kill everything and start fresh.

### If you're writing your own node

The deadman behavior is **required, not optional** — see [writing-your-own-node.md](writing-your-own-node.md#the-interface-contract) for the pattern to copy from `gap_follow_node.py`.

**If an autonomy node sits at `0.0/0.0` even with LB held**, check in this order:

1. `enable_deadman`, `joy_topic`, and `deadman_button` in the node's config YAML.
2. That `joy_node` is still running: `ros2 node list | grep joy`.
3. That it's actually publishing: `ros2 topic echo /joy`.
4. That you don't *also* have `teleop_launch.py` running in another terminal — it masks `/drive` at the mux no matter what your node publishes ([why](architecture.md#the-safety-model-read-this-before-writing-autonomy-code)).

### Tuning `gap_follow`

Its parameters — speed limits, steering limits, safety bubble radius, emergency stop distance, `deadman_button`, `joy_timeout_sec`, `enable_deadman` — live in `src/gap_follow/config/gap_follow.yaml`.

Defaults are deliberately conservative (`max_speed: 2.5` m/s). Raise them gradually rather than all at once, and re-test with the wheels off the ground after every change.

## Racing with the pure-pursuit stack

Two ways to do this. The automatic path is the one to use on a new course; the manual path gives you a reusable saved map and a racing line you can keep.

### Automatic map → raceline → race (recommended for a new course)

The no-manual-setup path. One launch starts hardware, online SLAM, `gap_follow`, `pure_pursuit`, and the supervisor that decides which controller may reach `/drive`.

**Terminal 1** — everything. Do **not** separately start bringup, teleop, localization, or either controller.

```bash
ros2 launch racerbot_launch auto_map_race_launch.py
```

**Hold LB continuously.** The car immediately begins a cautious `gap_follow` mapping lap.

Here's what it does, in order, without you touching anything:

1. Drives two autonomous laps by default. The first discovers the course and closes the SLAM loop. The second records a cleaner racing line in map coordinates (the map [frame](glossary.md#tf--transform--frame)), now that the loop is closed.
2. Stops.
3. Creates the speed profile, and saves the map and pose graph.
4. Loads the profile into the already-running pure-pursuit node.
5. Waits two seconds, then switches to racing.

SLAM stays online as the localization source, so there's no RViz pose seed to give and no process to restart.

**Working when:** you see the lap counter advancing in the terminal (see [Reading the terminal while it maps](#reading-the-terminal-while-it-maps) below), then a handover summary, then the car starts racing the line it just recorded.

Generated artifacts are written to `~/.ros/racerbot_auto/<YYYYMMDD-HHMMSS>/`:

- `raceline_raw.csv` and `raceline_profiled.csv`
- `map.yaml` and `map.pgm`
- `posegraph.posegraph` plus its data file

Useful overrides:

```bash
# Cap the mapping speed for a track nobody has driven yet. Uncapped by
# default -- gap_follow.yaml governs. A cap scales corner_speed and
# corner_speed_wide with it, since they are defined relative to max_speed and
# the node refuses to start if only max_speed moves.
ros2 launch racerbot_launch auto_map_race_launch.py mapping_max_speed:=1.5

# Skip the run recorder (on by default; it is subscribe-only and cheap).
ros2 launch racerbot_launch auto_map_race_launch.py diagnostics:=false

# Bag the run as well -- big, because /scan dominates the size.
ros2 launch racerbot_launch auto_map_race_launch.py record_bag:=true

# Use one mapping/recording lap instead of the cleaner two-lap default
ros2 launch racerbot_launch auto_map_race_launch.py mapping_laps:=1

# Hardware stack is already running in another terminal
ros2 launch racerbot_launch auto_map_race_launch.py include_bringup:=false

# Race a particular course to its own limits, without editing the packaged
# config: copy auto_map_race.yaml, change profile_max_speed /
# profile_max_lateral_accel, and point the launch at it
ros2 launch racerbot_launch auto_map_race_launch.py \
    supervisor_config:=$HOME/my_course.yaml
```

Releasing LB stops the selected controller immediately, but does not erase the map or the recorded progress.

**The first real run still follows the wheels-off-ground and low-speed ladder.** Simulator validation does not sign off physical grip or SLAM quality — see [simulator.md](simulator.md) for what the validation evidence actually covers.

#### Reading the terminal while it maps

The supervisor prints one line a second:

```
lap 1/2: samples=168, distance=27.2/5.0m, turn=341/300deg, elapsed=31.4/15.0s,
departed=yes, start distance=0.43/0.75m, heading error=6.2/30.0deg,
SLAM corrections absorbed=6
```

Two fields tell you whether it's going to work:

- **`turn`** is the gate that actually decides a lap is a lap. 360 degrees of accumulated yaw is one revolution of a closed circuit, whatever its size. If this climbs past 300 and the lap still doesn't close, the car isn't getting back within `closure_distance` of where it started.

- **`SLAM corrections absorbed`** counts how many times the map-frame pose jumped further than the car could physically have moved. A handful over a lap is normal, and they're applied to the recorded path rather than recorded *as* path. Dozens means SLAM is struggling — check the map in the dashboard before trusting anything downstream.

Then, once, at the handover:

```
Recorded lap cleaned up: 180 points over 27.1m (from 164 recorded, 164 after
trimming to one lap); kept 18 harmonics (nothing shorter than 1.50m), moving
the line at most 0.03m off the recorded one; peak curvature 0.696/m of the
0.821/m the rack can reach (needs 12.7deg steering, limit 14.9deg) on 0.0% of
waypoints; seam heading error 3.8deg; closest wall 0.62m, needs 0.30m
```

That is the whole verdict on the racing line. Two numbers matter:

- **`% of waypoints` past the rack limit** — should be `0`. Much above that and the car will understeer where it physically cannot steer harder.

- **`closest wall`** — must clear the number after it. This is the finished line measured against SLAM's own map.

  It's the check that catches the line being *rounded into a wall*. Filtering a recorded lap pulls its corners inward, and on a tight course, inward is where the wall is.

  `not checked (no map)` means `/map` never arrived — treat the run as unverified.

**If the run refuses** with *"Refusing to hand it to [pure pursuit](glossary.md#pure-pursuit)"*, the message says which of the two checks failed, and by how much.

`raceline_raw.csv` is still written either way, so you can plot it against the saved map.

Usual causes, in order:

1. A course with a corner tighter than this car's 1.22 m turning circle.
2. A smeared map.

See [racing-autonomy.md](racing-autonomy.md#what-a-recorded-lap-actually-looks-like).

#### Trying it without the car

The whole composition runs against the simulator, dashboard included:

```bash
ros2 launch racerbot_sim sim_auto_map_race_launch.py dashboard:=true
tools/racerbot_sim/run_auto_map_validation.py --scenario all
```

See [ros-simulator.md](ros-simulator.md). This is the fastest way to check a change to any part of the automatic path, and the only thing that exercises the launch *wiring* rather than the control math.

### Manual/reusable saved-map workflow

The map-based race controller, driven by hand through each stage. See [racing-autonomy.md](racing-autonomy.md) for how the algorithm works and how to tune it in depth.

The same joystick-override consideration as the automatic path applies here, and it's folded into the procedure below.

#### 1. One-time per track: record a racing line

Requires a saved map and working localization for this track — both sections above — already set up.

1. Start three terminals:

   **Terminal 1** — the foundation:

   ```bash
   ros2 launch f1tenth_stack bringup_launch.py
   ```

   **Terminal 2** — manual control:

   ```bash
   ros2 launch f1tenth_stack teleop_launch.py
   ```

   **Terminal 3** — localization:

   ```bash
   ros2 launch particle_filter localize_launch.py
   ```

2. In RViz, give it a **2D Pose Estimate** seed, same as normal localization.

3. **Terminal 4** — start recording (choose your own output path):

   ```bash
   ros2 launch pure_pursuit waypoint_recorder_launch.py \
       output_file:=/home/racerbotcar-2/racerbot-ws/src/pure_pursuit/waypoints/my_track_raw.csv
   ```

4. Hold LB and drive one clean lap by hand, finishing roughly back where you started.

5. `Ctrl+C` the recorder.

   **Working when:** it prints how many waypoints it recorded.

#### 2. One-time per track: generate the velocity profile

This turns the path you drove into a path *plus a target speed at every point* — see [velocity profile](glossary.md#velocity-profile).

```bash
ros2 run pure_pursuit generate_velocity_profile \
    --input src/pure_pursuit/waypoints/my_track_raw.csv \
    --output src/pure_pursuit/waypoints/my_track_profiled.csv \
    --v-max 4.0 --a-lat-max 2.5 --a-accel-max 3.0 --a-brake-max 8.0
```

**Working when:** it prints the resulting speed range and an estimated lap time.

Those numbers are the simulator-validated defaults. Start lower on untested physical surfaces — see [racing-autonomy.md](racing-autonomy.md#choosing-a_lat_max--a_accel_max--a_brake_max--v_max) for how to raise them safely.

A small synthetic example track is checked in at `src/pure_pursuit/waypoints/example_stadium_raw.csv`.

Use it to try the tool — and `pure_pursuit_node`, wheels off the ground — before you have a real recorded lap of your own.

#### 2b. Optional, better: optimize the line instead of just pacing it

`generate_velocity_profile` paces the lap you drove. `optimize_raceline` also *reshapes* it, using the saved map to find the minimum-curvature line within the track's real width — see [racing-autonomy.md](racing-autonomy.md#phase-4b-optional-optimize-the-line-itself-not-just-its-speed).

It writes the same `(x, y, speed)` file, so it's a drop-in replacement for step 2:

```bash
ros2 run pure_pursuit optimize_raceline \
    --map ~/maps/my_track.yaml \
    --recorded-lap src/pure_pursuit/waypoints/my_track_raw.csv \
    --output src/pure_pursuit/waypoints/my_track_profiled.csv \
    --safety-margin 0.15
```

**Working when:** it prints the before/after curvature and estimated lap time. Expect it to take a minute or two — it's an offline optimization, not something that runs on the car.

It **refuses to write the file** if the resulting line needs more steering than the rack has, or passes closer to a wall than the car is wide.

`--safety-margin` is the fast-versus-safe dial. The optimizer spends every centimetre it's given, so raise this — never lower it — if the first laps look tight.

**An optimized line apexes much closer to the walls than a recorded lap does, by design.** Drive the first laps at reduced `--v-max` and watch the decision log for reactive-avoidance engagements: if map subtraction isn't working, the car will keep braking for its own racing line.

#### 3. Every race run: drive it

1. **Terminal 1** — the foundation. Leave it running.

   ```bash
   ros2 launch f1tenth_stack bringup_launch.py
   ```

2. **Prop the wheels up** for the first run of any new racing line, or after any parameter change — the same rule as every other autonomy node.

   `pure_pursuit_node` has its own LB deadman check ([mandatory workspace policy](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)).

3. **Terminal 2** — localization and the race controller together. Nothing needs stopping first, since `bringup_launch.py` never started `teleop_launch.py`.

   ```bash
   ros2 launch racerbot_launch race_launch.py \
       waypoints_file:=/home/racerbotcar-2/racerbot-ws/src/pure_pursuit/waypoints/my_track_profiled.csv
   ```

4. Give it a **2D Pose Estimate** seed in RViz, same as any other time you start localization.

5. **Hold LB** — `pure_pursuit_node` won't drive without it.

   **Working when:** `ros2 topic echo /drive` shows sensible, smoothly varying values once the pose seed is in and LB is held, dropping to `0.0 / 0.0` the instant you release. Confirm this with the wheels up first.

6. **When you're done:** `Ctrl+C` the `race_launch.py` terminal. Launch `teleop_launch.py` on top of the still-running foundation to go back to manual driving, or kill everything and re-run `bringup_launch.py` fresh.

`pure_pursuit`'s tuning parameters — lookahead, speed limits, steering limits, safety watchdogs, `enable_deadman` — live in `src/pure_pursuit/config/pure_pursuit.yaml`. See [racing-autonomy.md](racing-autonomy.md#parameter-reference) for what each one does.

## Shutting down cleanly

`Ctrl+C` each `ros2 launch` terminal. One `Ctrl+C` cleanly shuts down every node that launch file started.

If something's stuck:

```bash
pkill -f "joy_node|joy_teleop|vesc_driver_node|urg_node_driver|ackermann_mux|ackermann_to_vesc_node|vesc_to_odom_node|static_transform_publisher|gap_follow_node|slam_toolbox|particle_filter|pure_pursuit_node|waypoint_recorder_node"
```

Power down the VESC and battery **last**, after the ROS nodes have stopped. Doing it in the other order makes the driver log a serial disconnect error — harmless, but noisy.

## Common gotchas that aren't bugs

**New terminal, permission denied on `/dev/sensors/vesc` or `/dev/input/js0`**

Group membership (`dialout`, `input`) only applies to sessions started *after* the group was added. Open a fresh terminal, or run `newgrp dialout && newgrp input` in the current one.

**Servo position shows `0.5304` and nothing seems to be happening**

That's neutral — dead center — not zero-as-in-broken. See the formula in [hardware-reference.md](hardware-reference.md#vesc-motor--steering-controller).

**You published to `/drive` and nothing happened**

Check whether `teleop_launch.py` is running in another terminal. If it is, its always-on neutral `/teleop` is masking your `/drive` command at the mux.

This is the safety model working as designed. If you follow [Running autonomy](#running-autonomy-gap_follow-pure_pursuit-or-your-own-node) as written you won't have `teleop_launch.py` running at all, so this only comes up if you started it deliberately alongside an autonomy node.

**The dashboard's camera panel says "camera offline" even though the camera and stream nodes are definitely running**

You're almost certainly viewing the dashboard through an editor's forwarded port (VS Code, SSH `-L`) at `localhost:8080`, rather than the car's real address.

The dashboard itself works fine over a tunnel, but the camera panel specifically needs a second, unforwarded connection straight to port `9090`. See [web-dashboard.md](web-dashboard.md#finding-the-cars-address-and-viewing-through-a-forwarded-port) for the fix.
