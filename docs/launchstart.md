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

**Hold LB for the whole run.** The car stays still for a moment at the start, until SLAM has recorded the start pose (`Start pose recorded: ...`). Releasing LB pauses mapping; holding it again carries on.

**Working when:** you see `Closed mapping lap 1/3`, `2/3` and `3/3`. The car stops just past the line, `Saved occupancy map successfully.` appears, Terminal 2 shuts down (Terminal 1 keeps running), and the last thing on screen is a block starting `MAPPING FINISHED.` with the race commands in it. Copy them from there.

**Then leave the car where it stopped.** That spot is where racing on this map starts. If you'll race this map on another day, mark the spot and the direction (tape on the floor) before moving the car.

Options, added to the end of the command:

| Option | When |
|---|---|
| `mapping_max_speed:=1.5` | a course nobody has driven yet |
| `laps:=4` | a different number of laps (at least 2) |

**If it says `the hardware is not running`:** start Terminal 1 first.

**If it doesn't finish:** `Ctrl+C` stops it, and it tells you no map was saved. If the last lap fails the quality check, it drives up to two more by itself. Lap details: [operations.md](operations.md#reading-the-terminal-while-it-maps).

## Race on a saved map

**The car must be where mapping left it:** still there, or put back on the marked spot facing the same way. The localizer is told the car starts there, because there is no RViz on the car to click a start pose, and it only searches about 0.5 m and 20° around that spot. Simulator-validated only.

**Terminal 1:** the `bringup_launch.py` from mapping, still running. Start it again if you closed it.

**Terminal 2**, pick one (the end-of-mapping printout has these with your run's folder already filled in):

```bash
# pure pursuit (the default racing controller)
ros2 launch racerbot_launch race_launch.py run_dir:=$HOME/.ros/racerbot_auto/<run>
# Stanley
ros2 launch racerbot_launch race_launch.py run_dir:=$HOME/.ros/racerbot_auto/<run> steering_law:=stanley
# LQR
ros2 launch racerbot_launch race_launch.py run_dir:=$HOME/.ros/racerbot_auto/<run> steering_law:=lqr
# MPC
ros2 launch racerbot_launch race_launch.py run_dir:=$HOME/.ros/racerbot_auto/<run> steering_law:=mpc
```

**Working when:** `Particle filter is at the start pose; racing can begin.` appears (the race controller only starts after it), then the car drives the racing line while you hold LB.

**If it says the filter never took the start pose:** the car isn't on the spot where mapping left it, or this is the wrong map. Move it there and run the command again. Nothing drives in the meantime.

**Forgot the run folder?**

```bash
ls -t ~/.ros/racerbot_auto | head -3                    # newest first
cat ~/.ros/racerbot_auto/<run>/commands.txt             # that run's commands again
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

**Any spare terminal**, alongside any mode:

```bash
ros2 launch racerbot_launch dashboard_launch.py
```

**Working when:** the car shows up at https://dashboard.sfuracerbot.ca. Details: [web-dashboard.md](web-dashboard.md).

## Record a run and analyse it

**Terminal 1, first:** `ros2 launch race_diagnostics record_run.py`. It prints the folder it records into, `~/.ros/racerbot_runs/<ts>/`.

Then start the mode you want as usual. Afterwards, in **any terminal**:

```bash
ros2 run race_diagnostics summarize_run ~/.ros/racerbot_runs/<ts>
python3 ~/racerbot-ws/tools/run_analysis/analyze_run.py ~/.ros/racerbot_runs/<ts>
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
| Maps, racing lines, `commands.txt` from mapping runs | `~/.ros/racerbot_auto/<YYYYMMDD-HHMMSS>/` |
| The same from the simulator | `~/.ros/racerbot_sim/auto/<YYYYMMDD-HHMMSS>/` |
| Run recordings (logs, events, optional bag) | `~/.ros/racerbot_runs/<ts>/` |
| Speeds and limits for `gap_follow` | `src/gap_follow/config/gap_follow.yaml` |
| Racing-line limits for mapping runs | `src/pure_pursuit/config/auto_map_race.yaml` |
