# Launch commands: the daily cheat sheet

**Who this is for:** anyone running the car today who already knows roughly what they want to do and just needs the command.
**Read first:** the [safety rules](#safety-every-time) below. If this is your first week, do [quickstart.md](quickstart.md) once.
**You'll be able to:** start any of the car's modes from a copy-paste command and tell whether it worked.
**Time:** a minute to find your command.

Every section follows the same pattern: what it does (one line), the terminals, the commands, and what you see when it works. The reasons and the details are in [operations.md](operations.md), linked from each section.

## Contents

- [Every terminal, first](#every-terminal-first)
- [Safety, every time](#safety-every-time)
- [Check the hardware](#check-the-hardware)
- [Drive by hand](#drive-by-hand)
- [gap_follow: drive itself, no map](#gap_follow-drive-itself-no-map)
- [Map a course: 3 laps, then stop](#map-a-course-3-laps-then-stop)
- [Race on a saved map](#race-on-a-saved-map)
- [Map and race in one go](#map-and-race-in-one-go)
- [Dashboard](#dashboard)
- [Record a run and analyse it](#record-a-run-and-analyse-it)
- [Simulator (no car)](#simulator-no-car)
- [Stop everything](#stop-everything)
- [Where things are saved](#where-things-are-saved)

## Every terminal, first

```bash
source /opt/ros/jazzy/setup.bash
source ~/racerbot-ws/install/setup.bash
```

**Working when:** nothing prints. If a `ros2` command later says "not found", you skipped this in that terminal.

## Safety, every time

- **Hold LB** on the F710 (XInput mode) whenever the car should move. Let go and it stops. This is true in every mode below, including the autonomous ones.
- **Wheels off the ground** the first time you run anything new or changed.
- **Never run `teleop_launch.py` together with an autonomous mode.** Its commands always win, so the car would just sit there (or do what the sticks say).
- Someone stands next to the battery switch whenever the car drives itself.

Why: [operations.md, safety checklist](operations.md#safety-checklist-every-time-before-powering-the-drive-motor).

## Check the hardware

**Any terminal:**

```bash
ros2 run racerbot_doctor doctor
```

**Working when:** the last line reads `0 FAIL`. Read-only; safe any time.

## Drive by hand

**Terminal 1** (hardware; leave it running for everything below unless a section says otherwise):

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** the VESC (motor controller) and the [LiDAR](glossary.md#lidar) come up without errors. It never moves the car by itself.

**Terminal 2:**

```bash
ros2 launch f1tenth_stack teleop_launch.py
```

**Working when:** holding LB and moving the sticks drives the car. Wiimote instead of the F710: [wiimote.md](wiimote.md). Details: [operations.md](operations.md#manual-driving-teleop).

## gap_follow: drive itself, no map

**Terminal 1:** `ros2 launch f1tenth_stack bringup_launch.py`

**Terminal 2:**

```bash
ros2 launch gap_follow gap_follow_launch.py
```

**Working when:** it prints a decision line every second, and the car drives itself while you hold LB.

**Other cars on the track too?** Turn on opponent awareness, on every car (simulator-validated only, see [gap-follow.md](gap-follow.md#several-cars-at-once)). **Terminal 2**, instead of the command above:

```bash
ros2 launch gap_follow gap_follow_launch.py opponent_awareness:=true
```

**Working when:** the decision lines show `opponent mode=follow` (or `clear`) while another car is moving ahead.

It follows a slower car instead of squeezing past it, and passes only on a straight with room. On a track as narrow as ours it will mostly follow. Do not run two cars at once without it.

**Slower for one run:** add `max_speed:=1.0` (any speed in m/s up to `gap_follow.yaml`'s `max_speed`, 3.5 on both cars). It also lowers `min_speed`, `corner_speed` and `corner_speed_wide` if they are above it. A value above the YAML's, or anything that isn't a positive number, is refused and nothing starts.

### Car 1 and car 2 together: first run

Opponent awareness has only been tested in the simulator. Do these steps in order, and don't skip ahead. Each step is the cheapest place to find a problem the next one would find with a crash.

**Two people, one per car.** Each holds LB on **their own car's** F710 controller. Letting go of LB stops that car.

1. **Check the cars can't hear each other.** Both cars on, `bringup_launch.py` running on both. On each car, **any terminal**:

   ```bash
   ros2 topic info /joy
   ```

   **Working when:** `Publisher count: 1`. **If it says 2:** the cars share one ROS graph (the system of nodes and topics that can talk to each other, see [glossary.md](glossary.md)). Car 2's LB would then drive car 1, and each car would get the other's drive commands. Stop, and give the cars different `ROS_DOMAIN_ID` values ([concepts.md](concepts.md)).

2. **Wheels off the ground, both cars.** Car on a stand, wheels free. **Terminal 2** on each car:

   ```bash
   ros2 launch gap_follow gap_follow_launch.py opponent_awareness:=true max_speed:=1.0
   ```

   Both cars have `max_speed:=` (car 2 since the 2026-10-24 port from car 1).

   **Working when:** each car's wheels turn only while its own LB is held, and stop when it's released.

3. **Floor, low speed, open space.** Same commands, one car about 2 m behind the other, nobody standing on the track. Start somewhere at least 2 m wide, not the 1.2 m track. In the simulator, at these speeds in a 1.2 m corridor, 1–3 of every 8 runs touched a wall, with or without opponent awareness and even with one car alone (2026-10-23, `narrow_1p2`). In wider layouts there was no contact.

   **Working when:** the decision lines (one a second) of the car behind show `opponent mode=follow` and a `follow cap=`. **If either car shows `opponent mode=` with no other car moving,** it's mistaking a wall for a car: stop, relaunch without `opponent_awareness:=true`, and keep that run's log.

## Map a course: 3 laps, then stop

`gap_follow` drives 3 laps while [SLAM](glossary.md#slam) builds the map. Then it saves the map and a [racing line](glossary.md#racing-line), stops the car, shuts everything down, and prints the commands to race on that map. Like every mode here it needs `bringup_launch.py` running first, and it refuses to start without it. Simulator-validated only; not yet run on the car.

**Before you start:** put the car on the track, pointing the way it should lap.

**Terminal 1** (leave it running for mapping *and* racing):

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Terminal 2:**

```bash
ros2 run racerbot_launch map_course
```

**Hold LB for the whole run.** (With `imu:=true`, keep the car still for the first 3 seconds first, while the IMU's gyro bias is measured.) The car stays still for a moment at the start, until SLAM has recorded the start pose (`Start pose recorded: ...`). Releasing LB pauses mapping; holding it again carries on.

**Working when:** you see `Closed mapping lap 1/3`, `2/3` and `3/3`. The car stops just past the line, `Saved occupancy map successfully.` appears, Terminal 2 shuts down (Terminal 1 keeps running), and the last thing on screen is a block starting `MAPPING FINISHED.` with the race commands in it. Copy them from there.

**Then leave the car where it stopped.** That spot is where racing on this map starts. If you'll race this map on another day, mark the spot and the direction (tape on the floor) before moving the car.

Options, added to the end of the command:

| Option | When |
|---|---|
| `mapping_max_speed:=1.5` | a course nobody has driven yet |
| `laps:=4` | a different number of laps (at least 2) |
| `fused_speed:=true` | `gap_follow` brakes and accelerates on the IMU-fused speed (`/odom/fused`) instead of the wheel speed. Never more than 0.3 m/s from the wheel speed. **Not yet driven on:** do one normal run first. See [the IMU README](../src/imu_bridge/README.md#in-map_course) |
| `gyro_odom:=true` | SLAM maps on **gyro odometry** (`/odom/imu`: heading from the BNO085's gyro instead of from the steering command). Only the map changes: `gap_follow` drives exactly as before. **Not yet driven on:** see [Testing the IMU in mapping](#testing-the-imu-in-mapping-ab) |
| `record_bag:=false` | don't save a bag. It is saved by default (a few hundred MB per run, in `~/racerbot-ws/data/racerbot_runs/`), because it's what lets a run be replayed and analysed afterwards |
| `imu:=true` | run the IMU side too (the BNO085 bridge, `imu_bridge`, `vehicle_state`). **Off by default on car 2**: it has no BNO085 yet and no `~/imu-calibration` checkout, which `imu_bridge` refuses to start without (see [car1-vs-car2.md](car1-vs-car2.md#imu)). `fused_speed:=true` and `gyro_odom:=true` need it. Car 1 runs it by default |

**If it says `the hardware is not running`:** start Terminal 1 first.

**If it doesn't finish:** `Ctrl+C` stops it, and it tells you no map was saved. If the last lap fails the quality check, it drives up to two more by itself. Lap details: [operations.md](operations.md#reading-the-terminal-while-it-maps).

## Testing the IMU in mapping (A/B)

> **Car 1 only for now.** Car 2 has no BNO085 yet. Once it is mounted and set up ([car1-vs-car2.md](car1-vs-car2.md#imu)), add `imu:=true` to both `map_course` commands below.

The question: does SLAM make a better map when its odometry gets its heading from the gyro? The test is two mapping runs of the same course from the same spot, one without and one with `gyro_odom:=true`, both recorded. Everything but the odometry SLAM uses stays the same, so the difference between the two maps is the IMU's.

**Before you start:**
- The car on **battery**, on the track. Mark the start spot and direction with tape.
- Terminal 1: `ros2 launch f1tenth_stack bringup_launch.py`, left running for every step below.

**1. Check the BNO085 points forward** (once; 2 minutes). Its roll and pitch were set from gravity on 2026-10-24, but which way is "forward" can only be seen while moving.

**Terminal 2:**

```bash
ros2 launch imu_bridge vehicle_state_launch.py
```

**Terminal 3**, then push the car forward by hand, firmly, a few times:

```bash
ros2 topic echo /imu/bno/data --field linear_acceleration
```

**Working when:** `x` jumps **positive** on each push, while `y` stays within about ±0.3. If `y` jumps instead of `x`, the mount's yaw is wrong: stop and redo `calibrate_imu.py --imu bno --step mount` in `~/imu-calibration`. Then `Ctrl+C` terminals 2 and 3.

**2. Run A, today's odometry.** Car on the tape mark. **Terminal 2:**

```bash
ros2 run racerbot_launch map_course
```

Keep the car still for the first 3 seconds, then hold LB as for any mapping run. **Working when:** it finishes as in [Map a course](#map-a-course-3-laps-then-stop), and prints `Recording run to: ~/racerbot-ws/data/racerbot_runs/<time>` near the start. Write that folder down as run A.

**3. Run B, gyro odometry.** Put the car back on the tape mark, facing the same way. **Terminal 2:**

```bash
ros2 run racerbot_launch map_course gyro_odom:=true
```

**Working when:** the same as run A, and the start of the output includes `publishing odom_imu -> odom` from `vehicle_state_node`. **If it logs `STOPPED publishing odom_imu -> odom`:** something else is giving `odom` a parent. `Ctrl+C`, and keep the log.

**4. Compare.** For each run's folder:

```bash
python3 tools/run_analysis/imu_report.py ~/racerbot-ws/data/racerbot_runs/<time>
```

**Working when:** it prints three sections: turn rate from each source, the steering lag and turning gain the gyro measured, and how much heading correction SLAM had to apply. Run B should need far less heading correction than run A.

Also compare the two saved maps on the dashboard (straight walls, no doubled corridors) and the `SLAM corrections absorbed` count each mapping log ends with. Both bags stay on the car, so the SLAM settings can be tuned afterwards without it.

## Race on a saved map

**The car must be where mapping left it:** still there, or put back on the marked spot facing the same way. The localizer is told the car starts there, because there is no RViz on the car to click a start pose, and it only searches about 0.5 m and 20° around that spot. Simulator-validated only.

**Terminal 1:** the `bringup_launch.py` from mapping, still running. Start it again if you closed it.

**Terminal 2**, pick one (the end-of-mapping printout has these with your run's folder already filled in):

```bash
# pure pursuit (the default racing controller)
ros2 launch racerbot_launch race_launch.py run_dir:=$HOME/racerbot-ws/data/racerbot_auto/<run>
# Stanley
ros2 launch racerbot_launch race_launch.py run_dir:=$HOME/racerbot-ws/data/racerbot_auto/<run> steering_law:=stanley
# LQR
ros2 launch racerbot_launch race_launch.py run_dir:=$HOME/racerbot-ws/data/racerbot_auto/<run> steering_law:=lqr
# MPC
ros2 launch racerbot_launch race_launch.py run_dir:=$HOME/racerbot-ws/data/racerbot_auto/<run> steering_law:=mpc
```

**Working when:** `Particle filter is at the start pose; racing can begin.` appears (the race controller only starts after it), then the car drives the racing line while you hold LB.

**If it says the filter never took the start pose:** the car isn't on the spot where mapping left it, or this is the wrong map. Move it there and run the command again. Nothing drives in the meantime.

**Forgot the run folder?**

```bash
ls -t ~/racerbot-ws/data/racerbot_auto | head -3                    # newest first
cat ~/racerbot-ws/data/racerbot_auto/<run>/commands.txt             # that run's commands again
```

**In the simulator** (map, then race without moving the car): pure pursuit, LQR and MPC each raced 128–142 m with no contact. In one run Stanley swerved around something it wrongly took for another car, and brushed a wall. Start with pure pursuit.

What the steering laws are: [steering-laws.md](steering-laws.md). A better racing line from the same map: [operations.md, optimize the line](operations.md#2b-optional-better-optimize-the-line-instead-of-just-pacing-it).

## Map and race in one go

The same mapping, but it switches straight to racing as soon as lap 2 is clean, with no restart. Starts the hardware too.

**Terminal 1:**

```bash
ros2 launch racerbot_launch auto_map_race_launch.py
```

**Working when:** lap messages, then a handover summary, then `Transition complete: pure pursuit now has drive control.` Details: [operations.md](operations.md#automatic-map--raceline--race-recommended-for-a-new-course).

## Dashboard

**Nothing to start** — it runs at boot (`racerbot-dashboard.service`), alongside any mode. **Any terminal**, to check:

```bash
systemctl status racerbot-dashboard
```

**Working when:** it says `active (running)` and the car shows up at https://dashboard.sfuracerbot.ca. Don't also launch `dashboard_launch.py` by hand: the second copy fails on the busy port. Details: [web-dashboard.md](web-dashboard.md#the-dashboards-boot-service).

## Record a run and analyse it

**Terminal 1, first:** `ros2 launch race_diagnostics record_run.py`. It prints the folder it records into, `~/racerbot-ws/data/racerbot_runs/<ts>/`.

Then start the mode you want as usual. Afterwards, in **any terminal**:

```bash
ros2 run race_diagnostics summarize_run ~/racerbot-ws/data/racerbot_runs/<ts>
python3 ~/racerbot-ws/tools/run_analysis/analyze_run.py ~/racerbot-ws/data/racerbot_runs/<ts>
```

**Working when:** the second writes `analysis/report.md` in the run folder, explaining every stop. The mapping commands above record a run by themselves. Details: [gap-follow.md, recording a floor run](gap-follow.md#recording-a-floor-run) and [run-analysis.md](run-analysis.md).

## Simulator (no car)

**Never on the car while its drivers are running.** The simulator fakes LB and the LiDAR. It refuses to start if it sees the real drivers, but don't rely on that.

```bash
# the whole map-then-race flow, in the simulator
ros2 launch racerbot_sim sim_auto_map_race_launch.py
# gap_follow on every indoor layout, no ROS (fast)
python3 ~/racerbot-ws/tools/f1tenth_sim/sweep_gap.py --track indoor --seeds 6
# several gap_follow cars at once, opponent awareness on
python3 ~/racerbot-ws/tools/f1tenth_sim/eval_gap_multi.py --track indoor_wide --cars 3 --speeds 1.8,3.5,3.5 --set enable_opponent_awareness=true
```

**Working when:** every line reads `pass N/N`, with `contact 0` (single car) or `wall 0  car-car 0` (several cars). Details: [ros-simulator.md](ros-simulator.md), [simulator.md](simulator.md).

## Stop everything

`Ctrl+C` in each terminal. If something is stuck:

```bash
pkill -f "joy_node|joy_teleop|vesc_driver_node|urg_node_driver|ackermann_mux|ackermann_to_vesc_node|vesc_to_odom_node|static_transform_publisher|gap_follow_node|slam_toolbox|particle_filter|pure_pursuit_node|waypoint_recorder_node"
```

Battery off **last**.

## Where things are saved

| What | Where |
|---|---|
| Maps, racing lines, `commands.txt` from mapping runs | `~/racerbot-ws/data/racerbot_auto/<YYYYMMDD-HHMMSS>/` |
| The same from the simulator | `~/racerbot-ws/data/racerbot_sim/auto/<YYYYMMDD-HHMMSS>/` |
| Run recordings (logs, events, optional bag) | `~/racerbot-ws/data/racerbot_runs/<ts>/` |
| Saving those three to GitHub after a session | `tools/save_runs.sh` (see [data/README.md](../data/README.md)) |
| Speeds and limits for `gap_follow` | `src/gap_follow/config/gap_follow.yaml` |
| Racing-line limits for mapping runs | `src/pure_pursuit/config/auto_map_race.yaml` |
