# ROS-level simulator (`racerbot_sim`)

> **Who this is for:** anyone who wants to run the real driving stack — launch files, SLAM, the dashboard — without the physical car.
> **Read first:** [concepts.md](concepts.md), and [simulator.md](simulator.md) for the other, simpler simulator and how the two differ.
> **You'll be able to:** run and validate whole launch files against simulated physics, and understand the interlock that stops this running beside real hardware.
> **Time:** about 15 minutes to read, plus setup and race time.
> **Looking for the steps?** [sim-validation.md](sim-validation.md) is the step-by-step workflow — set up, run your own node, connect the dashboard, get a verdict. This doc is the reference behind it.

## Highlights

- **Two ROS-controlled cars** can share a track, each with its own simulated sensors and drive commands.
- **Three laps by default** in the race launch; a wall or car contact ends the race early.
- **A 3 s discovery hold** blocks simulated sensor and gamepad messages at startup.
- **1081 LiDAR beams at 40 Hz** feed the same control stack used on the car.

`tools/f1tenth_sim/run_validation.py` ([docs/simulator.md](simulator.md))
calls the controllers' *math* directly and skips ROS entirely. That is the
right shape for tuning a control law. It is the wrong shape for the way
`auto_map_race_launch.py` actually broke, which was in the wiring around
the math: SLAM, TF, a racing line recorded from a live pose, a runtime
parameter handover between two nodes, and a safety layer with no way out
of its own stop. None of that exists in a harness with no ROS in it.

`racerbot_sim` is the other half. It replaces **only** the hardware:

| `bringup_launch.py` | `sim_bringup_launch.py` |
|---|---|
| `joy_node` (physical F710) | `sim_joy_node` (synthetic LB) |
| `urg_node` (Hokuyo) | `gym_bridge_node` |
| `vesc_driver` + `ackermann_to_vesc` + `vesc_to_odom` | `gym_bridge_node` |
| `ackermann_mux` | `ackermann_mux` — the same node, same config |
| static `base_link->laser` | the same node, same 0.26/0/0.11 offset |

Everything above that line is the real thing: the real `ackermann_mux`
arbitrating `/teleop` over `/drive`, the real `slam_toolbox`, the real
`gap_follow_node`, `pure_pursuit_node` and `auto_map_race_node`, the real
`web_dashboard`. `sim_auto_map_race_launch.py` deliberately *includes*
`racerbot_launch`'s own `auto_map_race_launch.py` with
`include_bringup:=false` rather than reproducing it, so what is validated
is the launch file people actually run.

## It refuses to run next to the car

Two nodes here are dangerous beside real hardware, and the Jetson on the
car is exactly where this is most convenient to run:

- **`sim_joy_node` forges the LB deadman.** The whole workspace safety
  policy ([architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car))
  is that nothing moves unless a human is holding LB. This node is that
  human's hand, forged.
- **`gym_bridge_node` publishes an imaginary `/scan` and `/odom`.**
  Alongside `urg_node` and `vesc_to_odom_node` that is two publishers per
  topic, and the driving nodes would be steering on a blend of a real room
  and an invented one.

So neither publishes anything while any of `vesc_driver_node`,
`ackermann_to_vesc_node`, `vesc_to_odom_node`, `urg_node`, `joy` or
`imu_bridge_node` is on the ROS graph.

The check repeats, so bringing the car up *after* the
simulator silences the simulator too (`racerbot_sim/hardware_guard.py`).
It is a live-graph check rather than a flag because a flag is something
you can forget.

**Never run this simulator beside the real car's drivers, and never defeat the interlock.** Forged LB could let a physical car move without a person holding the button, and simulated sensor readings could misdirect it.

The interlock starts blocked and waits `interlock_discovery_hold_sec` (3 s by default) before checking the graph. Before discovery, an empty graph can look like "no car here" even when the car is present.

After the hold, the graph must show no car on every check for `interlock_clear_confirm_sec` (3 s by default). Only then do simulated `/scan`, `/odom` or `/joy` messages start, so the simulator starts about 6 s after launch. The same window applies whenever the car's drivers leave the graph later.

One clean check isn't enough. On the dev box, discovery took about 2.45 s, and with the hold alone a forged `/joy` sometimes got out beside a running `urg_node` (measured 2026-10-29).

If the graph cannot be read, the interlock stays blocked. It continues checking after it opens. `sim_joy_node` starts its release, hold and silence scenario timers when the interlock first opens, not when the process starts.

**On the physical car, hold LB throughout motion; releasing it stops the car.** The deadman ([glossary](glossary.md#deadman)) provides a human stop if driving code makes a mistake.

**Test changed driving code with the wheels off the ground first, then on the floor at low speed in open space.** Simulation cannot reproduce every physical response; follow [the test order](writing-your-own-node.md#testing-before-its-on-wheels).

## Running it

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# The whole automatic composition, plus the dashboard (watch at dashboard.sfuracerbot.ca)
ros2 launch racerbot_sim sim_auto_map_race_launch.py dashboard:=true

# A tighter course, with two slower cars on it
ros2 launch racerbot_sim sim_auto_map_race_launch.py \
    track:=indoor_tight opponents:="6,0.7,0.0;14,0.6,0.0"

# Just the hardware layer, to put your own control layer on top
ros2 launch racerbot_sim sim_bringup_launch.py
ros2 launch gap_follow gap_follow_launch.py     # second terminal
```

Needs the F1TENTH Gym from `tools/f1tenth_sim/setup.sh` (one time).

## Simulated IMU

The simulator publishes an IMU (inertial measurement unit), which measures
turn rate and acceleration, for every controlled car. Car 1 uses
`/imu/bno/data` and `/imu/vesc/data`; `/imu/data` repeats the BNO reading.
Car 2 has the same topics under `/car2/imu/`, and later cars use `/carN/imu/`.

Each car also has an `/imu/status` topic under its own prefix. It carries
JSON describing the primary BNO's health. The status is latched, so a
subscriber that starts later receives the last verdict.
The latched `/imu/status` for each car is withdrawn whenever the hardware interlock blocks and offered again with the current verdict when it reopens.

**Terminal 2**, with the simulator running in Terminal 1 and this terminal
sourced as above, check the sensor topics and car 1's status:

```bash
ros2 topic list -t | grep imu
ros2 topic echo --once /imu/status
```

**Working when:** car 1's four topics appear and the status says
`{"state": "ok", "reason": null, "imu": "bno"}`. The default race launch
also lists `/car2/imu/data` and `/car2/imu/status`.

Both default launches passed this check on 2026-10-29.
If no topics appear, check Terminal 1 for
a startup error; if topics exist but no data arrives, check the interlock.

The gyro uses the gym's true yaw rate. Acceleration comes from the change
in true speed and yaw rate between simulation ticks, plus the acceleration
caused by each sensor's position ahead of the rear axle (the lever arm).
The BNO is 0.36 m ahead; the VESC (motor controller) IMU is 0.18 m ahead,
matching `vehicle_state.yaml`.

Forward acceleration includes `-offset × yaw_rate²`, and lateral
acceleration includes `offset × change_in_yaw_rate / dt`. At rest, the
vertical accelerometer reads gravity, +9.80665 m/s². Readings use car axes:
x forward, y left, z up. Orientation is marked unknown.

Independent Gaussian noise is added to every axis. These per-axis standard
deviations are the mean of the three measured axis standard deviations:

| Sensor | Gyro noise (rad/s) | Acceleration noise (m/s²) | Samples |
|---|---:|---:|---:|
| BNO | 0.000711561 | 0.021534955 | 4481 |
| VESC | 0.001347428 | 0.014463125 | 878 |

The source is run `20261026-194514`, `/imu/bno/data` and `/imu/vesc/data`,
3.5–21.3 seconds after the bag began, after the 3-second startup bias. All
890 `/odom` samples in that window showed zero speed. Each axis's mean was
removed before measuring its spread. The window is long because the BNO gyro
reports values in coarse steps, and a short window samples too few of them.

### Inject an IMU fault

`publish_imu:=false` on either simulator launch omits all IMU publishers.
Otherwise `imu_fault` affects car 1 and `car2_imu_fault` affects car 2.
Both default to `none` and can change while the simulator runs.

| Fault | Output |
|---|---|
| `none` | Normal readings; status `ok`. |
| `bias` | Add 0.1 rad/s to both gyros' z axes; status stays `ok`. |
| `scale` | Multiply both gyros' z axes by 1.1; status stays `ok`. |
| `zero` | All accelerometer and gyro axes become exactly zero, without noise; status `fault`. |
| `status_fault` | Normal readings with status `fault`. |
| `stale` | Stop IMU readings and retain the last status. Consumers must check data age. |

**Terminal 2**, with the simulator still running, stop car 1's IMU stream:

```bash
ros2 param set /gym_bridge_node imu_fault stale
```

**Working when:** the parameter change succeeds and `/imu/data` stops.
After a healthy stream, `/imu/status` stays `ok`; `gap_follow` then reports
`imu stale` and removes steering damping. Set `imu_fault none` with the
same command to resume readings. Car 2 continues throughout.

Start the IMU-derived estimator in another sourced terminal with
`ros2 launch imu_bridge vehicle_state_launch.py hardware:=false`.
This starts only `vehicle_state_node`, using the simulated BNO and VESC
topics. The default `hardware:=true` starts the real sensor stack.

When `steering_damping` or `adaptive_grip` is on, `gap_follow_launch.py` starts
this estimator. In the simulator it chooses the simulated IMU by itself:
`imu_hardware` defaults to `auto`, which sees the running simulator and starts
neither `imu_bridge_node` nor the BNO085 bridge. `imu_hardware:=false` forces
the simulated IMU. The `auto` check runs once, when gap_follow starts. If you
start gap_follow within a few seconds of `sim_bringup_launch.py`, the simulator
may not be up yet and `auto` picks the real IMU (the simulator then refuses to
run beside it). Pass `imu_hardware:=false` in that case; the validation
harness always does.

`map_course` also uses the simulated IMU. With `sim_bringup_launch.py`
running, the wrapper adds `imu_hardware:=false` for you, so the estimator
runs without starting the real sensor readers. You can use `gyro_odom:=true`
to give SLAM the gyro's heading. `imu:=false` explicitly disables the estimator.

The first comparison on 2026-10-29 used `car1_course_2026_10_26` at a 2.0 m/s cap. Its yaw-rate scale was deliberately wrong at 1.1. In the no-gyro mapping run, wheel odometry (estimated from speed and steering, [glossary](glossary.md#odometry--odom)) drifted 43°, 93° and 128° after successive revolutions.

The no-gyro run's maximum gyro heading error was 0.353°. The gyro-mapping run's maximum gyro error was 0.472° (I1b step 4, commit `b980e82`).

All runs saved maps, including a scale-1.0 control. Only the gyro-mapping run generated a racing profile. It still reported steering and wall-clearance problems, so the result demonstrates heading fidelity rather than a clean map. That comparison used `diagnostics:=false` to avoid the recorder directory leak, since fixed.

On 2026-10-29, on car 1, `map_course imu_all:=true` (on car 2: `imu:=true imu_all:=true`) on the default simulator track saved a map and profile to `/tmp/oc-part-b23-j1/auto/20261029-163557`. The recorded run was `20261029-163357`. SLAM used `odom_imu`, and its report measured a maximum 72.40° difference between gyro odometry and SLAM.

The final lap was accepted at the five-lap ceiling despite failing its quality check. The optimized line was rejected for wall clearance. This run verifies the wiring and report pipeline, not map quality.

Its five recorded laps counted 724°, 719°, 751°, 752° and 734° (2.01, 2.00, 2.09, 2.09 and 2.04 revolutions). Those were two real circuits each, about 58.5 m combined, rather than a doubling of one circuit's turning.

At the first return, the log showed 362° of turning, 1.12 m from the start and 5.7° heading error, but only 11.2 s elapsed. The 15 s duration floor blocked closure until the second return. The floor is now 4 s; the 300° turning gate remains.

The odometry lookup worked: a scratch-instrumented supervisor made 1,499 successful `odom_imu` lookups with no exceptions. No lookup change was needed. Integration of the first run's gyro and `/odom/imu` samples also confirmed roughly 720° per recorded lap.

A separate intermittent-odometry bug did count turns twice: it added map heading during a gap, then the same motion again when odometry returned. Bounded gap reconciliation fixes that bug, but it was not the cause of this run's two-circuit laps.

The repeat run `20261029-170502` used `indoor_oval` with `imu_all:=true` and the corrected 4 s minimum lap duration. Each recorded lap covered one circuit:

| Lap | Distance | Turning | Revolutions | Quality result |
|---|---|---|---|---|
| 1 | 29.2 m | 363° | 1.01 | Discovery lap discarded |
| 2 | 29.2 m | 360° | 1.00 | Intermediate lap, not judged |
| 3 | 29.0 m | 359° | 1.00 | Passed |

`map_course` exited 0 and saved an accepted, optimized 204-point profile at `/tmp/oc-part-b23-j1/auto2/20261029-170540/raceline_profiled.csv`. No lap exceeded the 1.5-revolution quality limit.

This run was on car 1's machine; its recorder and launch log stayed there, under `/tmp/oc-part-b23-j1/acceptance-home/.ros/racerbot_runs/` (it redirected `HOME` to keep writes out of car 1's real `~/.ros`). On car 2 the same run records into `data/racerbot_runs/`, and needs `imu:=true` written out, since car 2's `map_course` defaults to `imu:=false`.

Its SLAM report named `odom_imu` and measured 19.19° maximum gyro-to-SLAM heading difference. This was a mapping acceptance run, not a racing or physical-car trial.

Steering damping is active in the simulator: `/drive_intent` reports a
non-zero correction against the gyro's turn rate while cornering. With
`imu_fault:=stale`, its reason changes to `steering_damping_off: imu stale`
while driving continues; setting the fault back to `none` restores damping.
Reasons appear periodically, so some messages omit that field.

### IMU fall-backs, closed loop

On 2026-10-29, the validation harness ran `gap_follow` on `indoor_wide`
with a requested 120 seconds of driving and each fault requested at 40 seconds.
It uses the simulator's chassis-versus-map check for wall contacts. A stop
is a transition to an odometry speed below 0.05 m/s.

**Terminal 1**, from the workspace root, run the stale-fault case:

```bash
RB_WS="$(git rev-parse --show-toplevel)" rb env ROS_DOMAIN_ID=64 \
  python3 tools/racerbot_sim/run_gap_follow_validation.py --duration 120 --fault-at 40 \
  --imu-fault stale --gap-follow-arg imu_all:=true --output /tmp/stale-report.json
```

**Working when:** the command writes the JSON report and prints its ordinary
contact-and-distance verdict. Inspect `imu_validation` separately: that verdict
does not apply the IMU acceptance thresholds below. Steering damping is on by
default; adaptive grip is off and `imu_all` leaves it off. The harness passes
`imu_hardware:=false` itself. Its own tests run with the other tool suites:
`python3 -m pytest tools/f1tenth_sim/tests/ tools/racerbot_sim/tests/ tools/run_analysis/tests/ -q`
(sourced, on an isolated domain; see CLAUDE.md). `colcon test` does not run them.

The explicit `imu_hardware:=false` selects the simulator's IMU even if the
controller starts before automatic simulator detection completes.

For the feature-off baseline, omit the four feature arguments and fault, and
add `--gap-follow-arg steering_damping:=false`. For the no-fault comparison,
omit only `--imu-fault stale`. Replace `stale` with `status_fault`, `zero` or
`bias` for the other runs.

| Run | Distance (m) | Contact steps | Stops | Mean speed before / after (m/s) | Commanded speed before / after (m/s) | Five fall-back fractions | Result |
|---|---:|---:|---:|---:|---:|---|---|
| Features off | 444.41 | 0 | 0 | 3.553 / 3.539 | 3.732 / 3.720 | — | baseline |
| Features on | 412.68 | 0 | 0 | 3.416 / 3.408 | 3.596 / 3.579 | — | comparison |
| `stale` | 412.99 | 0 | 0 | 3.403 / 3.495 | 3.577 / 3.694 | 1 / 1 / 1 / 1 / 1 | pass |
| `status_fault` | 425.82 | 0 | 0 | 3.367 / 3.521 | 3.580 / 3.719 | 1 / 1 / 1 / 1 / 1 | pass |
| `zero` | 433.06 | 0 | 0 | 3.417 / 3.520 | 3.624 / 3.702 | 0.9744 / 0.9615 / 0.9744 / 0.9744 / 0.9615 | pass |
| `bias` | 420.48 | 0 | 0 | 3.320 / 3.411 | 3.517 / 3.585 | — | pass, zero contact |

The fractions are steering damping, fused speed, understeer, traction and
impact stop, in that order. They count only intent messages with a non-empty
reason at least 0.5 s after acknowledged injection. Messages without a reason
are ignored because the intent publisher sends reasons periodically.

All three fall-back runs pass: each fraction is at least 95%, contacts and
stops are zero, and mean speed after injection is within 10% of the baseline's
3.539 m/s after its 40 s split. Bias had zero contact and status stayed `ok`.

There is a timing limitation: the requested 40 s injections were acknowledged
at 43.624 s (`stale`), 43.844 s (`status_fault`), 45.017 s (`zero`) and
45.312 s (`bias`). The measured post-split distances were 303.514 m for the
baseline and 271.702, 278.231 and 282.243 m for the three fall-back runs,
respectively.

The intervals differ, so distance is reported rather than compared as a
same-time acceptance criterion.

The stale stream's last sample arrived at 43.465 s; it had no later samples.
For `status_fault`, status changed from `ok` to `fault` at 43.693 s. For
`zero`, status reported `all zero` at 44.879 s and later samples had zero
vertical acceleration and yaw rate.

The stale and status-fault runs had 78 and 77 reason-bearing messages, all
naming every expected fall-back. Zero had 78: two named `imu stale` for all
five features, and a third named `imu stale` for fused speed and impact stop.
Their complete texts and the first post-injection reason are in the JSON report.

The first reason-bearing messages arrived at 43.742 s (`stale`), 43.970 s
(`status_fault`) and 45.881 s (`zero`), measured from the start of driving.
Each named the expected fall-back for all five features. The `missing_fallback`
report counts mismatches for each feature and keeps the first three texts.

The gym's tyres do not slide. These runs check the fall-back wiring, not
whether grip or understeer behaviour is valid on the floor.

The ROS-free harness tests also caught two deliberate in-memory breaks:
counting pre-fault messages failed the fraction test, and allowing an unknown
fault failed the argument-refusal test. The unmodified tests passed.
Counting reason-less messages again also failed the new denominator test.

**All simulated IMU output sits behind the hardware interlock.**
The simulator refuses to send readings beside `imu_bridge_node`, including in
other namespaces, because mixing real and simulated gyros would mislead
controllers. The same startup and clean-graph waits apply as for `/scan`.

### Limitations

The gym's tyres cannot slide: tyre force is linear in slip angle. Use this
simulator to check IMU wiring and fault fall-backs, not to tune grip or
understeer thresholds; those need recorded runs from the car.

## Two cars and racing

Start two simulated hardware layers to check a second car's topics (named message channels, [glossary](glossary.md#topic)). This does not start driving controllers.

**Terminal 1 — simulator**, from the workspace root, with the setup above complete and no real drivers running:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch racerbot_sim sim_bringup_launch.py controlled_cars:=2
```

**Working when:** `gym_bridge_node` reports two cars; after the discovery hold, the scan (one sweep of LiDAR readings, [glossary](glossary.md#scan)) check below receives a message. Leave this terminal running. **If it doesn't:** check for `REAL HARDWARE DETECTED` or a missing Gym installation.

**Terminal 2 — check car 2**, sourced in a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic echo /car2/scan --once
```

**Working when:** one scan message appears. **If it doesn't:** wait for the 3 s hold and check Terminal 1 for an interlock or track error. Stop Terminal 1 with Ctrl+C before starting the race below.

`/car2` is a namespace: a prefix that keeps the second car's ROS names separate from car 1's ([glossary](glossary.md#namespace)). Car 2 receives `/car2/scan`, `/car2/odom` and `/car2/joy`. Its controller sends `/car2/drive` to `/car2/ackermann_mux`, which forwards commands on `/car2/ackermann_cmd`.

The second car's TF frames (coordinate systems, [glossary](glossary.md#tf--transform--frame)) are `car2/odom` → `car2/base_link` → `car2/laser`. Its simulator-only true pose is `/sim/car2/ground_truth_pose`; controllers must not use it.

`controlled_start_offsets_m` sets one comma-separated centreline offset in metres per controlled car. With two cars the default is `0.0,-3.0`. For an imported map, set `track_directory:=tools/f1tenth_sim/maps` and `track:=car1_course_2026_10_26`; `tracks.resolve` looks for its map YAML and centreline there.

### Run a race

This launch starts two controllers and a referee, the program that scores the race ([glossary](glossary.md#race-referee)). It stops on contact, when both cars finish, or at the timeout.

**Terminal 1 — race**, from the workspace root, after stopping the previous simulator:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch racerbot_sim sim_race_launch.py track:=indoor_wide opponent:=ours record:=false output_dir:=/tmp/racerbot-race
```

**Working when:** the terminal prints a standings table and `/tmp/racerbot-race/race.json` exists after the race ends. A contact can end it before three laps; that is a result, not a launch failure. **If it doesn't:** check the terminal for missing packages, Gym or an interlock message.

The referee reads `/sim/status` only. Each status message carries positions, speeds and contacts for one simulation step, so the score does not depend on separate pose messages arriving together.

`race.json` contains `finish_order`, `laps`, `overtakes`, `stalls`, `gap_changes`, `time_gained_s`, `contacts`, `missed_steps` and `race_laps`. An unfinished car can appear in the order without having finished; `winner: null` means no winner. See [reading head-to-head metrics](run-analysis.md#testing-several-cars-at-once).

| Race argument | Default | Meaning |
|---|---|---|
| `opponent` | `ours` | `ours` runs another `gap_follow`; `team_a` runs team A's controller and safety layer. |
| `track` | `indoor_oval` | Generated layout or imported map name; the launch finds imported maps in `tools/f1tenth_sim/maps/`. |
| `laps`, `timeout_s` | `3.0`, `300.0` | Required laps and simulation-time timeout in seconds. |
| `swap_starts`, `seed` | `false`, `12345` | Swap the two start positions; set sensor noise seed. |
| `car1_params`, `car2_params` | empty | Paths to `gap_follow` YAML files; empty uses the shipped file. `car2_params` applies only to `ours`. |
| `car1_hold_deadman`, `car2_hold_deadman` | `true`, `true` | Whether each simulated gamepad holds LB. |
| `record`, `record_dir` | `true`, empty | Record with `record_run.py`; optionally choose its run directory. |
| `output_dir` | empty | Directory for `race.json`; otherwise a timestamped directory under `~/racerbot-ws/data/racerbot_sim/races/`. |
| `reverse` | `false` | Reverse the referee's progress direction only; it does not reverse the simulator's starting poses. |

Recording adds car 2's topics and simulator status and ground-truth topics to the recorder's usual list. It is separate from the `race.json` score.

### Race team A

Set `opponent:=team_a` on the race command to run team A's real `disparity_extender` and `safety_node` from their launch files. Their code is pinned at `6bef23d`, their latest `main`, and is not edited for this race.

The launch puts both nodes in `/car2` and remaps their absolute names: `/joy`, `/scan`, `/ego_racecar/odom`, `/safety/brake` and `/drive` to the corresponding car 2 topics. Their safety layer remains on and receives car 2's simulated LB.

Team A and team B both have a package named `safety_node`. If `src/racerbot_b` is checked out, place an untracked `COLCON_IGNORE` in `src/racerbot_b/src/safety_node` before building, or leave `racerbot_b` uninitialised as on the car. Otherwise colcon cannot distinguish the packages. See [the simulated results](gap-follow.md#racing-team-a-in-the-simulator).

It runs in real time and does not use `/clock`: every node above the
hardware layer is the one that runs on the car, with its own wall-clock
timers and its own `use_sim_time: false`. That costs test duration (a full
validation is about four minutes per scenario) and buys the thing this
exists for -- the timing the nodes actually see. It also means runs are not
bit-identical: the ROS graph is real, so scheduling varies, and a result
that depends on a thousandth of a radian will vary with it.

### Arguments

| Argument | Default | What it does |
|---|---|---|
| `track` | `indoor_oval` | `indoor_oval` (30m lap, 1.8m corridor), `indoor_tight` (27m, 1.4m), `indoor_wide` (42m, 2.6m — the one this car can actually *race*, see below), `asb_10000` (50m, 1.5m — the real hallway loop this car is test-driven on, see below) |
| `opponents` | *(none)* | `;`-separated `offset_m,speed_mps,lateral_m`. Speed `0` parks that car on the line as a static obstacle |
| `hold_deadman` | `true` | `false` proves the car stays stopped with LB released |
| `release_after_sec` | `-1` | Release the synthetic LB this far into the run |
| `odom_speed_scale` | `1.0` | Odometry scale error, as a miscalibrated `speed_to_erpm_gain` would give |
| `odom_yaw_rate_scale` | `1.0` | Scale the turn rate used by dead-reckoned `/odom`, modelling a miscalibrated steering model. The simulated gyro still measures the true turn rate. |
| `seed` | `12345` | LiDAR noise and odometry noise |
| `dashboard` | `false` | Also start `web_dashboard` |

### Why the tracks are small

The official F1TENTH tracks are 300m+ laps. `auto_map_race_launch.py` maps
at 1.0 m/s, so two mapping laps plus racing on one of those is over twenty
minutes of wall clock per run — long enough that nobody runs it, which is
how the automatic mode reached the state it was in. The generated layouts
are room-sized closed loops in the same size class as the space this car
is actually driven in: a ~30m lap, mapped in about thirty seconds, whole
validation under four minutes.

Their corner radii are all comfortably outside the car's own 1.22m minimum
turning circle, which the real course this car is driven on is *not* —
see [racing-autonomy.md](racing-autonomy.md#what-a-recorded-lap-actually-looks-like).

### `asb_10000` — the real course

The other layouts are generic; `asb_10000` is a specific place: the
hallway loop around the office block on the Applied Sciences Building's
10000 level, where this car is actually test-driven (floor plan:
[Applied_Science_Building_10000_Level.pdf](Applied_Science_Building_10000_Level.pdf)).
The loop, as driven: start near 10863 → right at 10851 → right at 10911 →
down the 10957 hallway → right at 10869 → back to start — four right
turns closing a loop, which is exactly `rounded_rectangle_centerline`'s
shape, so this track needed no new geometry, only the right size and
corridor width for it. (Room 10911 doesn't appear on the scanned floor
plan itself — the nearest labels there are 10912.1–10912.4 and 10913 —
so that corner is taken on the team's word over the drawing.)

Two things this layout does *not* claim:

- **The size is a grid estimate, not a survey.** The floor plan has no
  coordinates to trace, only the printed structural column-grid
  dimensions (bays of 4–8.5m). The loop spans roughly two bays each way,
  which is what `half_x`/`half_y` in `tracks.py` are sized from. Treat the
  50m lap figure as approximate.
- **The corridor is intentionally narrow — second-tightest of any layout
  here, after `indoor_tight`.** `corridor_width=1.5` is what the team
  reports for the actual hallway, and the corner `radius` (1.2m) is set
  just past the car's 1.22m turning-circle floor to mimic the hallway's
  real right-angle corners as tightly as this car can actually take them.
  Both `gap_follow` and `pure_pursuit` are already known (see the
  validation table above) to touch the wall on corridors narrower than
  `indoor_wide`'s 2.6m; on `asb_10000`, expect that, and read it as a
  statement about the real hallway rather than a regression.

#### Measured (2026-08-10, seed 12345)

```bash
tools/racerbot_sim/run_gap_follow_validation.py --track asb_10000
tools/racerbot_sim/run_auto_map_validation.py --scenario solo --track asb_10000
```

Full numbers: [asb-10000-sim-results.json](asb-10000-sim-results.json).

| Test | Result |
|---|---|
| `gap_follow`, direct (no map, no SLAM) | Drove 121m (~2.4 laps of the ~50m course) in 66s, but 209 control ticks (~5.2s total) were in wall contact — the corners the docs above already flagged as too tight for the 0.25-0.35m clearance this controller drives at. |
| `pure_pursuit`, full auto-map-race pipeline | Reached every phase — SLAM up, mapping lap closed, line cleaned, profile written, map saved, handover — with the accepted line already at 0.817 of the 0.821/m rack limit (needing 14.8° of the car's 14.9° of steering). Once racing, wall contact recurred and the car covered ~0m net distance, cycling through `body_contact`/`emergency_clearance`/`racing_controller_stop` safety stops (plus one map-subtraction false positive that read a SLAM artifact as an opponent to overtake). |

Neither result is a regression to chase: it is the same corridor-width
statement the docs already made (`auto_map_race.yaml`: *"on a 1.8m indoor
corridor the car runs wide and touches the wall, at any speed where the
error exceeds the room"*), now measured on the real course instead of a
generic one, and this hallway is narrower still. Racing it for real would
start with a slower `supervisor_config` (see `auto_map_race_launch.py`'s
docstring) — a tuning question, not a course-validity one.

`run_gap_follow_validation.py` has no `pure_pursuit` equivalent for the
reason above: `run_auto_map_validation.py --track asb_10000` already is
the direct test for it, since pure_pursuit needs a localized pose and the
automatic pipeline is what supplies one in this simulator today.

**Update:** a real-hallway report on this course (too slow, corners taken
too slowly and too tightly, "driving kind of drunk") led to three
`gap_follow` algorithm changes -- gap-selection hysteresis, a
curvature-justified `corner_speed` raise, and live corridor-width sensing
-- documented in full in [`src/gap_follow/README.md`](../src/gap_follow/README.md#three-additions-from-a-real-hallway-report)
with the before/after numbers on both `asb_10000` and `indoor_wide` in
[asb-10000-sim-results.json](asb-10000-sim-results.json). Short version: a
clean win on `indoor_wide` (+10% average speed, still 0 wall contact); a
modest one on `asb_10000` itself (wall contact -6%, speed +2%) since this
course's corners are limited by curvature near the car's own turning
circle, which none of the three changes touch. **None of this is validated
on the real car yet** -- wheels off the ground, then floor, low speed,
before trusting it.

## Seeing what the dashboard sees

"The SLAM map looked really glitchy" is the way a mapping run is usually
judged, and it is hard to act on without the picture.

```bash
ros2 launch racerbot_sim sim_auto_map_race_launch.py dashboard:=true
tools/racerbot_sim/capture_dashboard.py --output /tmp/dashboard.png
```

`capture_dashboard.py` connects to the same WebSocket a browser does,
decodes the same messages `web_dashboard/protocol.py` sends, and draws them
the way the site's `apps/simple/web/dashboard.js` does -- occupancy grid, LiDAR points, car arrow
-- into a PNG. It only listens, so it is safe against the real car too.

With `--seconds` and `--interval` it is the dashboard's test instrument
rather than a screenshot tool: it watches a whole run, writes a frame per
phase, checks every binary frame against the length its header declared,
and exits non-zero if any failed. Measured over a full auto-map race:

| | mapping -> handover -> racing |
|---|---|
| binary frames | 2516, **0 failed the length check** |
| pose updates | 11196 |
| map republished | 57 times, resizing 17 times |
| view disturbances | 28 under the old auto-fit, **2** under the current one |

It also replays both view-fitting policies over the recorded map sequence,
so "the map looks glitchy" can be attributed to the view moving, to SLAM
genuinely smearing, or to two stacks running at once -- see
[web-dashboard.md](../src/web_dashboards/car/docs/web-dashboard.md#the-map-looks-glitchy).

## Automated validation

```bash
tools/racerbot_sim/run_auto_map_validation.py --scenario all
```

Runs three scenarios, each end to end, and exits non-zero if any fails:

| Scenario | What it puts on the track | What it proves |
|---|---|---|
| `solo` | nothing | mapping, cleanup, save, handover, and a clean racing stint |
| `obstacle` | one parked car on the racing line | the racing controller gets past a static obstacle rather than latching stopped in front of it |
| `traffic` | two slower cars | mapping with moving traffic, and racing among it without contact |

### The one-command mapping flow (`map_course` → `race_launch.py run_dir:=`)

**Any terminal**, sourced, with no car drivers running (about 10 minutes):

```bash
tools/racerbot_sim/run_map_course_validation.py --track indoor_oval
```

It runs the two commands [launchstart.md](launchstart.md#map-a-course-3-laps-then-stop) gives, for real, over the simulator. First `ros2 run racerbot_launch map_course`, with `sim_bringup_launch.py` standing in for the hardware. Then each of the four race commands printed in `commands.txt`, each on a freshly started simulator so the car is back on the start spot. It uses ROS domain 79 unless you have set one.

**Working when:** every JSON line it prints has `"passed": true`. The race lines must also show `raced_m` of 20 or more.


**All three scenarios run on `indoor_wide`**, overriding the launch files'
`indoor_oval` default, because what limits them is the *corridor* rather
than the controller:

- **Racing at all.** The line comes from `gap_follow`, which drives
  0.25-0.35m from a corner's wall, and pure pursuit's cross-track error
  through a corner near this car's turning circle measured 0.39-0.57m at
  2.5-3.0 m/s. On `indoor_oval`'s 1.8m corridor those do not both fit, and
  the car touches the wall -- a true statement about the course, not a
  defect to gate a regression suite on.
- **Getting past anything.** `gap_follow` inflates every obstacle by
  `car_width/2 + safety_margin`, which demands 0.67m of width, and a car in
  the middle of a 1.8m corridor leaves 0.6m either side and is simply a
  roadblock. Failing that proves nothing about the code.

Run `--track indoor_oval` or `--track indoor_tight` deliberately to ask the
narrower question: does the pipeline still map, clean up, and *refuse*
correctly. Note the consequence for a hand-run demo -- launching
`sim_auto_map_race_launch.py` on its `indoor_oval` default and watching the
car wedge itself into a corner during the racing phase is the expected
result, not a regression. Pass `track:=indoor_wide` to watch it race.

Each scenario is judged on:

- every phase reached — SLAM up → mapping lap → line cleaned → profile
  written → map saved → profile loaded → racing;
- no node crash, no unhandled exception, no refused profile;
- no wall contact for *any* car, and no car-to-car contact;
- **distance covered after the handover**, not just total distance. Total
  distance is dominated by the mapping laps and stays high even when the
  racing phase never moves. A run of the obstacle scenario passed on
  exactly that hole, having spent its entire racing phase stopped nose to
  nose with the parked car.

The generated racing line and the full launch log are kept for a failure.
The checked-in result is [docs/auto-map-sim-results.json](auto-map-sim-results.json)
(2026-08-08, seed 12345, all three scenarios passing):

| Scenario | Raced | Wall contact | Car contact | Line |
|---|---:|---:|---:|---|
| solo | 138.7 m / 81.6 s | 0 | 0 | 12.5° of the 14.9° rack, 0.83 m from the wall |
| obstacle | 29.7 m / 82.0 s | 0 | 0 | 0.0% of waypoints over the rack |
| traffic | 83.1 m / 80.7 s | 0 | 0 | map flagged as containing the other cars |

## What it does and does not model

**Real:** F1TENTH Gym single-track dynamics with RK4 at 5ms, multi-car
ray-cast LiDAR and collisions, the measured 0.26m LiDAR offset, the padded
0.58 x 0.31m body, 40Hz control, and the real ROS graph above the drivers.
The scan is 1081 beams over 270°, the 0.25° spacing that car 1's UST-30LX
and car 2's UST-10LX share (datasheet values).

**Dead-reckoned odometry, not ground truth.** `/odom` is integrated from
wheel speed and the *commanded* servo angle exactly as `vesc_to_odom` does
with `use_servo_cmd_to_calc_angular_velocity`, so it drifts in the same
shape the car's does. Publishing ground truth there would hide every
mapping problem this exists to find. Ground truth is published separately
on `/sim/ground_truth_pose` in its own unconnected `sim_world` frame, for
scoring only — nothing that drives the car may read it.

**Not modelled:** tyre grip against a real floor, VESC and servo dynamics,
LiDAR reflectivity and multi-echo, WiFi, and CPU contention on the Jetson.
[sim-fidelity-audit.md](sim-fidelity-audit.md) covers how far the shared
vehicle model is from this car; all of it applies here.

### One thing the older harness cannot see

**Gym's own collision flag is not a verdict here.** In the pinned Gym
revision, wall collision is `raw_scan - side_distances <= 0.005`.

Under the old 0.33m LiDAR offset the sensor sat 0.04m *outside* gym's
collision rectangle, `side_distances` came out all zeros, and the flag
**never fired**: a car driven straight into a wall at 1 m/s reported
`collision=False` and kept going, right off the edge of the map.

The measured 0.26m offset puts the sensor back inside the box, and the
flag **does fire now** (measured in the older harness on 2026-10-20). It
fires on the *padded* envelope rather than the real chassis, and its
response is `state[3:] = 0`, which zeroes the car's speed, steering and
**yaw**.

In the older harness that turned two cars round to lap backwards, so
`FidelityPlant` switches the response off
([tools/f1tenth_sim/README.md](../tools/f1tenth_sim/README.md) #12).
`SimBridge` switches it off too by setting `ttc_threshold = -inf`.

Either way, `racerbot_sim` samples the padded body against the occupancy
grid itself (`SimBridge.body_contact`) and treats off-map as contact. `SimBridge.car_contact` checks car-to-car overlap. These checks, not gym's flag, supply the race contact verdict.
