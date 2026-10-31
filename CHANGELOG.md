# Changelog

Notable changes to the workspace's own packages, newest first. Format:
one dated section per work session, grouped by package, with behavior
changes and new/removed parameters called out explicitly. Upstream
submodule bumps don't go here (see `docs/git-setup.md`) — this file is
for changes the team made.

## 2026-10-31 (latest) — Automatic recorder: catch more of a run's start, report failures properly

Follows a check of the 2026-10-30 session's 19 automatic recordings. All were intact, but two bags began with the car already moving. **Tested on an isolated ROS domain against fake publishers, not yet on a real run.**

- **`race_diagnostics` recording start** (behavior change): `auto_record` starts `ros2 bag record` itself, then `record_run.py` with `record_bag:=false` once the bag is live. **`start_delay_sec` 3.0 -> 1.5, `poll_period_sec` 0.5 -> 0.25.** New parameter `live_topics`, new terminal line `recording live: ...`. Measured with the same method, the bag now misses 2.95 to 6.9 s of a run (median about 4, 8 runs), against 5.45 to 7.0 s before (median about 6.6, 4 runs). ROS discovery causes the slow end, so the worst case barely moved and a fast press of LB can still be missed.
- **New `auto_record.yaml` fields**: `bag_live_sec`, `bag_exit_code`, `recorder_failures` (`recorder_exit_code` stays 0 when a child dies), `superseded`, and `activity` (`started_late`, `moved`, `max_speed_mps`, `first_odom_speed_mps`, `lb_held_fraction`), from the new `run_activity.py`. `summarize_run` prints them under `Recording problems`.
- **`data/racerbot_runs/auto_record.log`**: every start, stop, refusal and late start, kept with the runs.
- **Shutdown**: `system_log` runs without the `ros2 run` wrapper, which swallowed SIGINT and made every recording wait 5 s and log an error. `race_diag_node` no longer dies with a traceback on rclpy's shutdown race. `record_run.py` is signalled only once it has finished starting.
- **`snapshot_params`**: default `--delay` 15 -> 3 s, so short runs get their parameters.
- **`tools/save_runs.sh`**: lists runs that never moved and stubs; new `--skip-idle`; refuses while `run_activity` is running.
- **`recording.py`**: `DEFAULT_TOPICS` and `MAX_BAG_FILE_BYTES` moved here from `record_run.py`.
- **`ros2 launch` hanging at exit**: seen in `threading._shutdown` with every child gone. `record_run.py` now exits on its own 18 s after shutdown begins. The cause inside launch was not found, so a stop that hits this can still report `recorder_killed: true` after 15 s. The bag is not affected.
- **Test**: `test_auto_record_node.py` failed 4 of 4 runs on the old code, because fixed sleeps were racing the slow bag start. It now waits for the bag to be live.

## 2026-10-30 — Port from car 1: gap_follow's launch boost only

Car 1's PR #10 (`git diff bf56121 ffbfac0`). Car 1's PR #9 is not included. **Never driven on car 2.**

- **`gap_follow`**: new `launch_boost.py`. On the first driving tick after LB, once per node: straight to `max_speed` with no ramp, corner, corridor or opponent caps, no speed or traction hold, holding straight while its own path is clear. Deadman, TTC, emergency clearance, braking, curvature and understeer ceilings still apply. New parameters `enable_launch_boost` (shipped `true`), `launch_boost_max_sec`, `launch_boost_max_distance`, `launch_boost_end_steering`; launch argument `launch_boost:=false`. The mapping launches turn it off.
- **`tools/f1tenth_sim`**: `gap_mirror.py` models it.
- **Tests**: car 1's, plus five added on car 2 for what a mutation run showed nothing caught: the braking and curvature ceilings during a launch, the emergency escape, and two mirror parity scenes. Mid-race scenes turn the launch off in their setup; no assertion changed.
- **Simulator, car 2's numbers** ([gap-follow.md](docs/gap-follow.md#launch-boost)): on car 2's room map the solo lap is 0.86 s slower, and two-car runs finish 5 to 6 of 8 against 8 of 8 with it off, no contact.

## 2026-10-29 — Port from car 1 (to `car1/main` `8a88dcc`): Part B-2/3, SLAM observability and IMU

Car 1's PR #8 ([Racerbot-Car-1-Workspace](https://github.com/sfu-racerbot/Racerbot-Car-1-Workspace)),
ported on branch `port-car1-part-b23` (`git diff 4bea6f2 car1/main`, `git apply --3way`,
five conflicts merged by hand). Car 2's differences are in `docs/car1-vs-car2.md`
(new section "IMU feature thresholds"); `test_car2_hardware_config.py` gained three checks.
**Nothing here was driven on car 2.**

- **`race_diagnostics`**: `snapshot_params` keeps watching for the whole run, so nodes that start
  late (or restart, `<name>.<n>.yaml`, or share a name across namespaces) are snapshotted.
  New `system_log` (`<run>/system/tegrastats.log` and 1 Hz per-node `proc.jsonl`;
  `system_log:=false` turns it off; a missing `tegrastats` only warns). `metadata/packages.txt`
  (`ros-jazzy-*` versions, slam_toolbox first) and `metadata/submodules.txt`. Lap-quality parser
  in `run_events.py`.
- **`tools/run_analysis`**: `slam_report.py` (drift, corrections, laps, map consistency, scan drops
  against CPU, `--compare`), `slam_replay.py --exact` with `slam_exact_feed.py`, `grip_replay.py`.
- **`racerbot_sim`**: a simulated IMU per controlled car (`sim_imu.py`; `/imu/*` and `/car2/imu/*`),
  injectable faults (`imu_fault`, `car2_imu_fault`), covered by the interlock.
  `tools/racerbot_sim/run_gap_follow_validation.py` gained the IMU fall-back harness.
- **`imu_bridge`**: `vehicle_state_launch.py hardware:=false` starts only `vehicle_state_node`,
  on the simulator's IMU topics.
- **`gap_follow`**: `fused_speed`, `imu_understeer`, `imu_traction` and `imu_impact_stop`, each with
  its own launch flag, plus `imu`, `imu_all` and `imu_hardware`. All off by default. Every IMU
  failure gives the feature-off command. Conservative safety inputs: with fused speed on, TTC
  and the stale-odometry stop take the more cautious of wheel and fused speed.
- **`racerbot_launch`**: `map_course` takes `imu_all`, `imu_lap_turns` and `imu_hardware`, runs
  `summarize_run`, `imu_report` and `slam_report` after the launch (`analysis:=false` skips them),
  and maps at `mapping_max_speed` **3.5** by default (was empty, i.e. gap_follow.yaml's 5.0).
- **`pure_pursuit`** (`auto_map_race`): odometry-gap reconciliation for gyro lap turns; the minimum
  lap time is `max(4 s, 0.5 × previous lap)` (was a fixed 15 s).
- **Car 2 keeps:** `map_course imu:=false`, `vehicle_state_launch.py bno:=false`,
  `steering_damping:=false` (a literal, not car 1's `imu` default); runs in `data/racerbot_runs/`
  (`grip_replay.py` and the ported tests' run paths changed to match); car 2's four-state speed
  filter (car 1's fused-speed fallback tests pass against it unchanged).
- **Car 1's values, labelled as car 1's, features off:** impact threshold 33.73 m/s², traction slip
  0.246 / 0.123 m/s, understeer bands 0.75 / 0.85, simulated IMU noise and mount offsets.
- **Tests:** car 1's new map_course IMU tests now pass `imu:=true` explicitly (car 2's default is
  false), and `test_imu_flags_both_launches.py` expects car 2's per-launch `imu` defaults.
  `test_snapshot_params_node.py` finds the recorder's child with `pgrep -P`: car 2's Tegra kernel
  has no `/proc/<pid>/task/<pid>/children` (`CONFIG_PROC_CHILDREN` not set).

## 2026-10-29 — Port from car 1 (to `car1/main` `4bea6f2`): racing, Part A and B-1

Car 1's PRs #3 to #7 ([Racerbot-Car-1-Workspace](https://github.com/sfu-racerbot/Racerbot-Car-1-Workspace)),
ported feature by feature on branch `port-car1-part-b1` (`git diff 0265263 car1/main`,
applied in date order with `git apply --3way`, then merged by hand). Car 2's
differences are in `docs/car1-vs-car2.md`; `test_car2_hardware_config.py` gained two checks.

- **`gap_follow`**: the `max_speed:=` launch flag can raise the top speed up to 5.0 m/s.
  Gyro **steering damping** and **adaptive grip** (`gap_logic.py`, the `steering_damping:=` and
  `adaptive_grip:=` launch arguments). **Race defaults** (car 1, 2026-10-27): `max_speed`
  3.5 -> **5.0**, `min_speed` 1.429, `corner_speed` 2.143, `corner_speed_wide` 2.714,
  `narrow_corridor_speed` 3.571, `max_acceleration` 3.0 -> 5.0, `max_lateral_accel` 2.0 -> 2.5, `steering_damping_gain` 0.10,
  `enable_corner_setup` false. **Opponent awareness on by default** (`opponent_awareness:='true'`):
  follow hold, `FollowCapMemory` (`opponent_follow_memory_sec` 1.0), the 0.31 x 0.58 m outline,
  the narrow-section pass refusal and the alongside abort. **Never driven on car 2.**
- **Car 2 keeps:** `max_steering_angle` 0.349 and `arc_guard_max_correction` 0.1745; steering damping **off**
  (`enable_steering_damping: false`, `steering_damping:=false`) until the BNO085 is mounted
  and calibrated; `neural_racer`'s 0.349 wrapper (its `max_lateral_accel` stays 2.0, no longer paired).
- **`racerbot_launch`**: `opponent_awareness` argument, default `false`, in `map_course`,
  `auto_map_race_launch.py` and `autonomous_mapping_launch.py`. `slam_gyro.yaml` (3 m loop
  search with `gyro_odom:=true`). `map_course` keeps `imu:=false`.
- **`race_diagnostics`**: `auto_record`, started by every bringup (`record:=false` turns it off),
  records each algorithm run into `data/racerbot_runs/` (car 1: `~/.ros/racerbot_runs/`).
- **`pure_pursuit`**: `auto_map_race` closes a lap at its closest approach to the start
  (`close_at_closest_approach`); four LB deadman deny-path tests for `auto_map_race_node`.
- **`racerbot_sim`**: two controlled cars (`/car2`), `sim_race_launch.py` and the race referee
  (results in `data/racerbot_sim/races/`), team A as car 2, the interlock's 3 s discovery hold
  plus `interlock_clear_confirm_sec` 3 s starting blocked, `/test_only` drive chains and the
  conftest isolation guard, gym's yaw-zeroing response disabled.
- **`tools/`**: `eval_gap_multi.py` head-to-head score, `recognition.py`, gap_mirror parity,
  car 1's course as a track; `run_analysis` `slam_replay`, `slam_metrics`, `slam_bag`,
  `odom_tf_relay`, `weave.py`.
- **Docs**: gap-follow.md (race defaults, steering damping, adaptive grip, "Several cars at
  once"), ros-simulator.md, launchstart.md, run-diagnostics.md, glossary, git-setup.md
  Addition 6, car 1's plans under `docs/superpowers/`.

## 2026-10-24 — Port from car 1 (to `car1/main` `0265263`); branch cleanup

Everything car 1 ([Racerbot-Car-1-Workspace](https://github.com/sfu-racerbot/Racerbot-Car-1-Workspace))
built since it took car 2's `38c35a0`, as a patch (`git diff 0cd347f car1/main`,
`git apply --3way`; the repos share no history), with car 2's hardware kept.
Car 1's own entries in its `CHANGELOG.md` (2026-10-23 and 2026-10-24) have the
detail and its measurements. **Nothing below has been driven on car 2.**

### New on car 2

- **`imu_bridge`**: car 1's extension of PR #2's package (merged here the
  same day, `6dec009`): `/imu/status`, the BNO restart watchdog, and `vehicle_state_node` (`/odom/fused`,
  `/steering/measured`, gyro odometry `/odom/imu` + optional `odom_imu -> odom`).
  Support code, publishes no drive topic.
- **`neural_racer`** (new package, driving code, shadow mode by default) and
  `tools/neural_racer/` (FIR training stack; its tests need `torch`, which the
  car doesn't have). `docs/superpowers/specs/2026-10-23-neural-racer-plan.md`.
- **`gap_follow_launch.py max_speed:=`** (`gap_follow/launch_speed.py`), and
  `gap_follow` no longer claims the mux stops the car when `/drive` goes quiet.
- **`map_course`**: `imu:=`, `fused_speed:=`, `gyro_odom:=`; `record_bag:=true`
  by default. `slam_launch.py slam_odom_frame:=`.
- **`race_diagnostics`**: more bagged topics (IMU, SLAM pose graph, `/rosout`,
  `/drive_intent` ...), and `<run>/metadata/` (git state, IMU calibration
  copies, `snapshot_params`).
- `tools/run_analysis/imu_report.py`.
- **Bringup**: the dead `ackermann_cmd_out` remap is gone from all three
  `*_bringup_launch.py`, guarded by `racerbot_launch/test/test_drive_path_wiring.py`.
  `racerbot_sim` follows.
- `optimize_raceline.py --max-steering-angle` defaults to the planner's 0.30.
- **`racerbot_doctor`**: car 1's generic checks only. The joystick layer now
  checks `event*` as well as `js*` (`joy_node` reads `event*`; a user outside
  `input` got a silent `/joy` on car 1), a new *Drive path (teleop)* section
  follows `/joy` -> `/teleop` -> `/ackermann_cmd` -> `/commands/motor/speed`
  with LB-held and non-zero-motor counts, and `/odom` silent before the first
  servo command is `INFO`. Car 2's Ethernet LiDAR and RealSense checks are
  unchanged. New test `test_lb_button_is_the_teleop_deadman` (car 1's live LB
  test took the button index from `car.py` itself). 158 tests.
- Docs: `docs/known-issues.md`, launchstart's two-car first run and IMU A/B,
  and car 1's doc fixes (architecture, ros-simulator, racing-autonomy,
  troubleshooting's `input` group entry, ...).

### Kept as car 2's (`docs/car1-vs-car2.md`, new)

- Ethernet LiDAR, RealSense, `rb2` dashboard, calibration, steering limits,
  odometry gain, VESC udev rule, doctor LiDAR/camera ladders.
- **IMU off by default**: car 2 has no BNO085 yet and no `~/imu-calibration`
  (without it `imu_bridge_node` raises). `map_course imu:=false` and
  `vehicle_state_launch.py bno:=false` here; car 1 ships both `true`.
  `test_map_course_imu.py`'s default test now asserts `false`, and the tests
  that turn on `fused_speed`/`gyro_odom` pass `imu:=true` explicitly.
- **`neural_racer`'s wrapper `max_steering_angle` 0.349** (car 1: 0.33), as its
  own `test_wrapper_values_match_gap_follow` demands; `test_steering_clipped`
  now takes its expected cap from the config instead of a literal 0.33.
- New guard test `racerbot_launch/test/test_car2_hardware_config.py` (12
  tests, each seen to fail against car 1's version of the file it guards).

### Repo housekeeping

- Remote `car1` added. Every extra branch deleted, local and on GitHub, after
  checking each one's commits were already in `main`; the five worktrees
  under `.claude/worktrees/` and `.oc-worktrees/` removed. The one uncommitted
  change found (`wd-migration`: `docs/sim-validation.md`'s dashboard log line)
  is in this commit.

## 2026-10-22 — gap_follow opponent awareness; one-command mapping; launch cheat sheet

All of it is **simulator-validated only**. Bring it up wheels-off first, then at
low speed ([docs/gap-follow.md](docs/gap-follow.md#validation)).

### gap_follow

- **Opponent awareness** (`enable_opponent_awareness`, **off by default**;
  `gap_follow_launch.py opponent_awareness:=true`). New
  `gap_follow/opponent_tracking.py`, no ROS needed:
  - it finds free-standing, car-sized clusters and tracks them against this
    car's dead-reckoned motion (the steering modelled as the servo follows it);
  - behind a moving car it caps speed to `(gap − 0.5 m) / 1.0 s`;
  - it passes only on a straight, and only with 0.15 m of extra room around
    where the other car will be over the next 0.8 s.

  Safety checks still read the real scan. With it off, the commands are
  identical. New `opponent_*` parameters are all commented in
  `config/gap_follow.yaml`.
- Simulator, leader 1.8 m/s and the others 3.5 m/s, both directions:

  | | Layer on | Layer off |
  |---|---|---|
  | Two cars | 40/40 | 34/40 |
  | Three cars | 24/24 | 15/24 |
  | One car | 111/132 | 108/132 |

  There was no contact in any run with it on. It follows rather than
  overtakes on every track tried.

### racerbot_launch / pure_pursuit

- **`ros2 run racerbot_launch map_course`** (with `bringup_launch.py` already
  running; it never starts the drivers and refuses without them): `gap_follow` drives 3 laps, then
  the run saves the map, pose graph, racing line and `start_pose.yaml`, shuts
  down and prints the race commands. It is `auto_map_race_node` in the new
  `finish_after_mapping` mode, with `map_course_launch.py`.
- **`race_launch.py run_dir:=<dir> [steering_law:=...]`**: races a saved run.
  The new `seed_start_pose` node seeds the particle filter where mapping
  stopped the car, so no RViz is needed. `pure_pursuit` only starts once the
  filter has taken that pose.
- `localize_run_launch.py standalone:=true` publishes the filter's TF and
  `/map` when no SLAM is running.
- `tools/racerbot_sim/run_map_course_validation.py` runs the whole flow end to
  end over the simulator. On `indoor_wide`, mapping passed every run. Pure
  pursuit, LQR and MPC each raced 128–142 m with no contact. Stanley once
  swerved around a false "opponent" (a scan/map mismatch) and brushed a wall;
  that is the existing `pure_pursuit` overtake logic.
- `localize_run_launch.py standalone:=true` starts the particle filter 4 s
  after `map_server`. Started together, the filter got an empty map and
  crashed (1 race start in 8 in simulation).

### docs

- New [docs/launchstart.md](docs/launchstart.md): every mode as one
  copy-paste command.

## 2026-10-19 — gap_follow corridor rework; Stanley, LQR and MPC made usable

Both changes are **simulator-validated only**. Bring them up wheels-off first,
then at low speed ([docs/gap-follow.md](docs/gap-follow.md#validation),
[docs/steering-laws.md](docs/steering-laws.md#safety)).

### gap_follow

- **Swept-path clearance** (`enable_swept_clearance`, on): the braking speed
  ceiling uses how far the body can drive along its arc, not the nearest
  return in the ±30° cone (which read hallway side walls as obstacles ahead).
- **Arc guard** (`enable_arc_guard`, on): if the requested steering would
  sweep the body into something, use the nearest clear steering angle, at
  most `arc_guard_max_correction` (0.1745 rad) away.
- **Corner setup** (`enable_corner_setup`, on): move toward the outside wall
  before a corridor turn, so the car turns in late.
- **`max_lateral_accel` 1.0 → 2.0 m/s²** — simulator only, not yet
  validated on the floor.
- New parameters (all documented in `config/gap_follow.yaml`); startup
  refuses arc-guard/corner-setup authority above half of `max_steering_angle`.
- Simulator, 5 indoor layouts × 2 directions × 6 seeds: 36/60 passed with 24
  wall-contact runs → 60/60, none; `asb_10000` lap 24.6 → 18.9 s
  (`docs/gap-follow-sim-results.json`).
- Tools: `tools/f1tenth_sim/gap_mirror.py` (the simulator's copy of the
  per-scan pipeline, reads the YAML, held to the node to 1e-6 by
  `test_gap_mirror_parity.py`), `sweep_gap.py`, and `trace_gap.py` rebuilt on
  it. `run_validation.py`'s stale `gap_command` was replaced by it.

### pure_pursuit steering laws

- **Stanley formula corrected** (a change to spec 02): the separate
  `+atan(L·κ)` feedforward is gone (the front-axle heading error already
  carries it, so it was counted twice and held the car inside every bend),
  and the front axle is steered to the offset a rear-axle racing line implies.
  Spec tests 6/7 now check the kinematic answer, `atan(L/R)` on a circle.
- **New parameters:** `stanley.delay_compensation`, `lqr.delay_compensation`,
  `mpc.delay_compensation` (predict the pose ~0.1 s ahead before steering),
  `stanley.steering_time_constant`, `lqr.steering_time_constant` (output
  low-pass against localisation jitter).
- **Retuned defaults:** Stanley gain 5.0, softening 1.0; MPC `q_lateral` 4.0;
  plus the values above. Dataclasses, YAML and node defaults all agree
  (tested).
- MPC's returned plan is projected onto the exact angle and rate limits.
- The node logs `steering law: <name> <params>` at startup.
- Review fixes: MPC miss latch read a non-existent attribute; live lookahead
  and steering-rate tunes did not reach the law; `/drive_intent` could run the
  MPC solver; MPC's condensed dynamics were wrong at varying speed; the bound
  check was tighter than OSQP's accuracy.
- `tools/f1tenth_sim/run_validation.py` reads its limits from
  `pure_pursuit.yaml` (it had a stale 0.26 rad steering limit);
  `tools/f1tenth_sim/law_sweep.py` runs comparisons in parallel.
- **osqp 1.1.3 installed** on the car (`pip install --user
  --break-system-packages osqp==1.1.3`). MPC benchmark G0 passed: p99.9
  3.5 ms against an 8 ms deadline under load
  (`docs/mpc-benchmark-2026-10-19.json`).
- **Judging (seeds 12345–12349): Stanley, LQR and MPC all PROMOTE**, 15/15
  runs each, no contact, every margin under 5%
  (`docs/tracking-law-comparison.json`). This supersedes the REJECTs in the
  entry below. Pure pursuit stays the default.

## 2026-10-19 — Pluggable steering laws for the race controller

`pure_pursuit_node` can now steer with a choice of tracking law —
`pure_pursuit` (today's, the default), `stanley`, `lqr`, or `mpc` —
selected at launch with `steering_law`, without changing anything else the
node does. Every safety mechanism (LB deadman, pose-stale/frozen and
cross-track watchdogs, off-line recovery, opponent overtaking, the reactive
LiDAR net, steering slew and curvature speed cap) applies to every law,
because the law is swapped in *between* them rather than beside them. The
laws output steering only; speed stays with the profile and the node's caps.

### Behaviour changes

- **New parameter `steering_law`** (default `pure_pursuit`, launch-time only)
  plus per-law `stanley.*`, `lqr.*`, `mpc.*` gains and
  `tracking_curvature_smoothing`. All are refused at runtime (the law is built
  once at startup), and none are live-tunable from the dashboard. The launch
  file passes `steering_law:=...` through.
- **New failure states** in the decision log and `race_diagnostics`:
  `steering_law_failed` (a law that cannot produce a trustworthy command stops
  the car), `mpc_fallback` (a single-tick pure-pursuit fallback on an MPC miss),
  and `mpc_miss_latched` (MPC gave up after `max_consecutive_misses`; cleared
  by releasing LB).
- **`/drive_intent`** re-runs the *active* law (replays the MPC solver's
  predicted sequence rather than re-solving).

### Code

- `pure_pursuit/racing_math.py`: `estimate_signed_path_curvature`,
  `project_onto_path`, `interpolate_along` (all ROS-free, all unit-tested).
- `pure_pursuit/tracking_laws.py` (new, ROS-free): the `TrackingLaw` seam,
  `PurePursuitLaw` (today's steering moved unchanged, verified by a
  change-detector test against the pre-refactor computation), `StanleyLaw`,
  `LqrLaw` + `LqrGainTable`, `MpcLaw` + `MissPolicy`, and `make_law`.
- `tools/f1tenth_sim/run_validation.py`: `PathFollower` steers through the
  pluggable law; `--steering-law`, `--law-param`, and `--compare` run a
  candidate against pure pursuit over the same tracks × seeds. Refactor proof:
  identical `pure_solo` metrics before/after. The promotion rule is pure
  Python in `tools/f1tenth_sim/law_compare.py`, tested in
  `tools/f1tenth_sim/tests/` (`python3 -m pytest tools/f1tenth_sim/tests/ -v`).
- `tools/mpc_benchmark/benchmark_mpc.py` (new): the spec 04 G0 solve-time
  benchmark, ready for the osqp install decision.

### Judging results (spec 01)

Both `stanley` and `lqr` **REJECT** at the shipped starting gains on the
judging seeds (`docs/tracking-law-comparison.json`, command and analysis in
`docs/sim-validation.md`). Neither ships; pure pursuit remains the default and
the baseline. MPC is gated on spec 04 G0 (the osqp install decision and a
Jetson benchmark) before it can be judged. Specs:
`docs/superpowers/specs/2026-09-27-tracking-*.md`.

## 2026-09-27 (latest) — The dashboard moves to sfu-racerbot/web-dashboards

`web_dashboard` and `usb_cam_stream` moved, with their git history
(`git subtree split` here, `git subtree add` there), into
**sfu-racerbot/web-dashboards** under `car/ros/`. That repo now holds the
site and everything a team needs on its car; this workspace consumes it as
the git submodule **`src/web_dashboards`** and keeps only car 2's settings.
Package names, `ros2 run` and `ros2 launch` commands are unchanged.

### Behaviour changes

- **Start the dashboard with `ros2 launch racerbot_launch dashboard_launch.py`.**
  The package now ships generic defaults, so a bare
  `ros2 launch web_dashboard web_dashboard_launch.py` allows **no** site
  (the site shows CAR OFFLINE), tunes and stops nothing, and lists no maps.
  Car 2's values are in the new **`racerbot_launch/config/web_dashboard_rb2.yaml`**,
  passed as `car_config:=`: `allowed_origins`, `laser_offset_x: 0.26`,
  `tuning_nodes`/`tuning_config_files`, `killable_nodes`, `map_roots`, and
  the RealSense stream's `image_topic`/`passthrough`. Values unchanged from
  before. `realsense_camera_launch.py`, `dashboard_test_launch.py` and
  `racerbot_sim`'s `dashboard:=true` use it.
- **The car serves no pages.** The old frontend (`web/`) and its tests were
  deleted — the site's `apps/simple` replaced them. `serve_static` defaults
  to false and is ignored (with a warning) if set; `http://<car-ip>:8080/`
  answers 404. Open https://dashboard.sfuracerbot.ca.
- **`drive_intent` is now optional for the dashboard** (it stays in this
  workspace): without it, nothing subscribes to `/drive_intent` and the
  node logs `drive intent: OFF` once.
- **`remote_check` takes `--site https://dashboard.sfuracerbot.ca --car rb2`.**
- **foxglove_bridge's config, launch and safety test moved into
  `web_dashboard`** (`ros2 launch web_dashboard foxglove_bridge_launch.py`);
  client publishing stays **off**. `racerbot_launch/launch/foxglove_bridge_launch.py`
  is now a **temporary forwarder**, kept with `tools/systemd/` only so the
  installed `foxglove-bridge` unit survives a reboot; both go once the unit is
  reinstalled from `src/web_dashboards/car/systemd/` (`docs/web-dashboard.md`).
- `usb_cam_stream`'s `realsense_stream_launch.py`/`realsense_stream.yaml`
  are gone (car-2 config, now in the rb2 YAML).
- `tools/web_dashboard/` moved to the submodule's `car/tools/`.
- Later the same day, after `foxglove-bridge.service` was reinstalled from
  the submodule and the site's `/rb2/check` passed every hop: the forwarder
  `racerbot_launch/launch/foxglove_bridge_launch.py` and `tools/systemd/`
  were **removed**. `ros2 launch web_dashboard foxglove_bridge_launch.py` is
  the bridge's launch. `racerbot_launch`'s test now fails if a bridge launch
  or config reappears in it.

### Tests

- New `racerbot_launch/test/test_web_dashboard_rb2_config.py`: every rb2 key
  is a parameter the node declares; `laser_offset_x` equals every bringup's
  `base_link`→`laser` x; the site origin equals the site's own
  `PUBLIC_ORIGIN`; no stoppable process or map folder would be silently
  dropped. Replaces `test_foxglove_bridge_config.py`, which moved with the
  bridge config.
- `gap_follow`/`pure_pursuit` `test_*_live_tuning.py`: the dashboard's
  `tuning.py` is imported from the submodule, and each node's **live**
  config now round-trips through its YAML writer — this workspace's half of
  the `live_tunable_spec` contract (the dashboard repo round-trips a snapshot).
- **Run ROS tests on an isolated domain** (`ROS_DOMAIN_ID=79
  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, now in `CLAUDE.md`):
  `gap_follow`'s node tests published to the real `/drive`. Found the hard way
  while taking baseline counts for this move, with bringup up and the
  battery detached.
- **`gap_follow`'s node tests no longer publish to the real `/drive`.** Every
  `rclpy.init` in `src/gap_follow/test/` ends with `TEST_ONLY_DRIVE`
  (`-p drive_topic:=/test_only/drive`), as `pure_pursuit`'s already did.
  It must be **last**: measured on Jazzy, once any `-p` precedes
  `--params-file`, the YAML's node-keyed values beat every `-p`, so a
  remap placed first left `test_packaged_config_plus_mapping_overrides_actually_starts`
  on `/drive`. `test_no_test_here_builds_a_node_on_the_real_drive` checks
  every call (by AST), each fixture asserts its publisher's topic, and
  `test_test_only_drive_must_come_after_the_params_file` pins the ordering
  rule. The isolated domain stays: the tests still publish latched topics.

## 2026-09-27 (later) — Diagnosing the remote site from the car

The site could not connect although every service on the car was healthy.
Checked from the car: **rb2-bridge-origin.sfuracerbot.ca had no DNS record**
(NXDOMAIN on 1.1.1.1) although the tunnel routed it. Every hostname also
shows non-browser clients a Cloudflare bot challenge; this entry first
blamed that for the failure, which was **wrong** — the site's Worker got
HTTP 101 from the dashboard through the same zone. The site's check also
reports the bridge as 502 because it probes it with a plain request, which
foxglove_bridge hangs up on; a WebSocket upgrade with its subprotocol gets
101.

### web_dashboard

- **New `ros2 run web_dashboard remote_check`** (`remote_check.py`): local
  services, a relay handshake as the site makes it, tunnel routes from
  cloudflared's journal, public DNS, how the Cloudflare edge answers
  (Access vs tunnel down vs *unprotected*; a challenge is a warning), and the site
  connections that reached the dashboard. Read-only; prints a fix per
  failure. Verdicts tested in `test_remote_check.py` (mutation-checked).
- **The dashboard logs every connection open and close**, all roles, with
  user, address, duration and close code. Before, direct connections were
  not logged at all.
- Needs rebuilding: new entry point (`colcon build --packages-select
  web_dashboard`).

## 2026-09-27 — foxglove_bridge: client publishing switched off (safety fix)

**foxglove_bridge 3.5.0 does not enforce `client_topic_whitelist`.** The
2026-09-26 config relied on it to allow only `/initialpose`. Tested on an
isolated copy of the bridge (ROS domain 79, its own port, this YAML,
nothing connected to the car):

- with `clientPublish` on, a client advertised `/drive`, published one
  zero-speed message, and `ros2 topic echo /drive` **received it**;
- with `clientPublish` removed from `capabilities`, the bridge answered
  `Server does not support clientPublish capability` and nothing arrived.

A `/drive` published from a browser goes straight to `ackermann_mux` and
skips every driving node's LB deadman. So:

### racerbot_launch

- `config/foxglove_bridge.yaml`: **`clientPublish` removed from
  `capabilities`**. Nothing can be published through the bridge.
  `client_topic_whitelist` stays as a second layer for a future version that
  enforces it; it is not what protects the car.
- `test/test_foxglove_bridge_config.py`: the test that asserted
  `clientPublish` was **on** now asserts it is **off**, citing the
  measurement above. Seen to fail with `clientPublish` put back.
- Cost: Lichtblick can no longer send a "2D Pose Estimate"
  (`/initialpose`). Use RViz on the car's network for that.

### Checked live on the car (bringup + RealSense + dashboard + bridge, no control layer)

- Bridge listening on `127.0.0.1:8765` only; offers 34 topics, all
  compressed image topics, **no raw `Image`**; 83 services;
  `getParameters` returns values (391 across the graph); calling
  `/foxglove_bridge/list_parameters` returned 39 names.
- dashboard-rb2.sfuracerbot.ca is retired; mentions removed.

## 2026-09-27 — Particle-filter pose is the LiDAR's, now converted to the rear axle

**Bug fixed.** `particle_filter` ray-casts each scan from the particle's own
pose (`particle_filter.py`, `sensor_model`: `queries = proposal_dist`), so
`/pf/viz/inferred_pose` is the pose of the **LiDAR**, 0.26 m ahead of the
rear-axle `base_link` everything else assumes. Nothing converted it:

- `pure_pursuit_node` (saved-map racing) believed the car was 0.26 m ahead
  of itself, and its map ray caster added `laser_offset_x` on top, placing
  the expected scan 0.52 m forward of the rear axle.
- `auto_map_race_node` recorded the line from SLAM's real `base_link`, then
  at handover republished the filter's LiDAR pose on `/slam_pose` as if it
  were `base_link` (a 0.26 m jump), and seeded the filter with a
  `base_link` pose as if it were the LiDAR's.
- `waypoint_recorder_node` recorded the LiDAR's path.
- `web_dashboard` drew the car and scan 0.26 m forward in PF mode.

Changes:
- `racing_math`: new `laser_pose_to_base_link` / `base_link_pose_to_laser`.
- `pure_pursuit_node`, `waypoint_recorder_node`: **new parameter
  `pose_frame`** (`laser` default, matching the default
  `/pf/viz/inferred_pose`; or `base_link`); unknown values refuse to start.
  `waypoint_recorder.yaml` gains `laser_offset_x/y`.
- `auto_map_race_node`: converts the PF pose before republishing and the
  seed before sending; **new parameters `laser_offset_x/y`** in
  `auto_map_race.yaml`.
- `auto_map_race_launch.py`: sets `pose_frame: base_link` with
  `pose_topic: /slam_pose`.
- `web_dashboard`: **new parameter `laser_pose_topics`**
  (`[/pf/viz/inferred_pose]`).
- Docs: `localization.md#which-point-on-the-car-the-position-means`, param
  tables, the `hardware-reference.md` offset row.

Tests: 339 `pure_pursuit` (sourced), 607 `web_dashboard`, 53
`racerbot_launch`; every new test seen failing against 11 mutations. Two
existing tests changed, flagged in the commit: the auto_map_race handover
test asserted the unconverted pose (the bug), and the recorder status test
now declares its poses as `base_link`.

Old PF-recorded lines trace the LiDAR's path: on a closed loop the same line
shifted along itself, about 3 cm outside on a 1 m-radius corner
(0.26² / 2R), so still usable.

**Needs on-car validation:**
- [ ] `auto_map_race` handover: no forward jump in `/slam_pose` at the
  switch to `particle_filter` (plot `race_diagnostics`' pose around it).
- [ ] Saved-map `race_launch.py`: dashboard car and scan overlay line up
  with the map walls; map-mode opponent detection shows no phantom or
  missed opponents near walls.

Also: implementation specs for Stanley / LQR / linear MPC steering laws in
`docs/superpowers/specs/2026-09-27-tracking-*.md` (not implemented).

## 2026-09-26 — Car side of dashboard.sfuracerbot.ca, and foxglove_bridge

The dashboard frontend is moving to sfu-racerbot/web-dashboards, served at
https://dashboard.sfuracerbot.ca through the Cloudflare Tunnel. This is the
car's half of that contract. `web_dashboard` still creates **no ROS
publisher**. Verified: 603 `web_dashboard` tests, 53 bridge-config tests
and a mutation pass over every new test; a real `dashboard_node` driven
over real sockets by relay, control and direct clients on an isolated ROS
domain; `usb_cam_stream` fetched with the tunnel's headers.

**Needs on-car validation** (none of this has been through the real tunnel):
- [x] ~~dashboard-rb2.sfuracerbot.ca still connects~~ — retired on
  2026-09-27 (it was not working), so nothing to check.
- [ ] The remote site's relay and control sockets connect through
  `rb2-dash-origin`, get `hello` first, and the relay's writes are refused.
- [ ] foxglove_bridge: Lichtblick through `rb2-bridge-origin` lists topics,
  reads/sets a parameter and calls a service. (Publishing is now off
  entirely — see 2026-09-27.)
- [ ] Camera through `rb2-cam-origin`.
- [ ] A recorded run's bag contains `/parameter_events` entries for a
  change made from the dashboard and one made from Lichtblick.

### web_dashboard — who may connect, and as what

- **Behavior change: WebSocket origins are checked.** It used to accept
  every origin. Now: same-origin (every LAN/Tailscale/port-forward use),
  an exact match in the new **`allowed_origins`** parameter (ships
  `["https://dashboard.sfuracerbot.ca"]`), or no `Origin` header (scripts).
  Anything else is a 403. `origins.py`, tested in `test_origins.py`.
- **`hello` first on every connection**:
  `{"type":"hello","protocol_version":1}`. New `protocol.PROTOCOL_VERSION`;
  bump it on any incompatible wire change (rule in the package README).
- **Connection roles** from `X-Racerbot-Role`: `relay` (full telemetry,
  every write refused with a new `write_refused` message; unknown requests
  count as writes), `control` (write-panel state, stopwatch, and replies to
  its own actions only; no map/scan/batch), none (unchanged). Any other
  role value is a 400. Every write is logged with `X-Racerbot-User`, or
  `unknown (direct)`. `roles.py`, tested in `test_roles.py`.
- **New `serve_static`** (default `true`): `false` serves only `/ws` and
  404s every page.
- **`server.py`**: the Tornado handler and fan-out moved out of
  `dashboard_node.py`, so `test_server.py` runs them on a real socket
  without `rclpy`.
- **`web/` is deprecated**, a frozen fallback. New frontend work goes to
  sfu-racerbot/web-dashboards. Its tests stay until that repo ports them.

### racerbot_launch — foxglove_bridge

- New `config/foxglove_bridge.yaml` and `launch/foxglove_bridge_launch.py`:
  `127.0.0.1:8765`; topics, services and parameters (get and set) open;
  **client publishing limited to `^/initialpose$`** (**WRONG — superseded
  2026-09-27: 3.5.0 does not enforce that whitelist; publishing is now off
  entirely**), because a raw `/drive` publish would skip every driving
  node's LB deadman; raw camera/depth
  images hidden. Parameter names checked against foxglove_bridge 3.5.0.
- `exec_depend foxglove_bridge` (apt `ros-jazzy-foxglove-bridge`, **not
  installed yet**). First test in this package:
  `test/test_foxglove_bridge_config.py`.
- `tools/systemd/foxglove-bridge.service` + `foxglove-bridge.sh`, written
  but **not installed or enabled**. See `docs/foxglove-bridge.md`.

### race_diagnostics

- `/parameter_events` added to `record_run.py`'s default bag topics, so
  parameter changes from the dashboard, Lichtblick or a terminal land in
  each run's bag.

## 2026-08-24 — Measuring on the map, and three ways to clear one

Both features live entirely in `web_dashboard`. The package still creates
**no ROS publisher** — the SLAM reset is a service client, like the existing
live-tuning clients. Verified end to end against a running node: every
refusal path exercised over a real WebSocket, and one throwaway run
directory created and deleted for real. **Not yet exercised on the car with
`slam_toolbox` actually running** — the reset path was only observed
refusing when the service was absent.

### web_dashboard — a measuring tool on the map

- **Tap the map to measure.** Two taps give a point-to-point distance; more
  keep chaining, with each leg labelled and a running total. Dragging still
  pans and pinching still zooms — only a tap adds a point, judged on the
  *furthest* the pointer travelled rather than where it ended, so a drag
  that wanders and returns is not mistaken for a click.
- **The geometry is its own file.** `web/measure.js` touches no DOM, canvas
  or WebSocket, so `test/browser/measure_test.js` loads the real file under
  plain node and checks it against closed forms (79 checks).
- **`dashboard.js` gained `canvasToWorld` / `canvasToBody` /
  `canvasToActive`.** The inverse transform previously existed only inline
  inside `zoomAt`, in two copies; `zoomAt` now calls the extracted pair.
- **A measurement taken before localization is cleared when a pose
  arrives,** with a note saying why. Those points were relative to a car
  that has since moved, and redrawing them in map coordinates would put a
  confident-looking number in the wrong place.
- New sidebar section `measure`, a readout pinned over the map (the phone
  sheet starts closed), and <kbd>M</kbd> / <kbd>Esc</kbd> /
  <kbd>Backspace</kbd>. No new parameters.

### web_dashboard — clearing a map, three ways

- **clear view** — the browser forgets its copy and asks for a fresh
  keyframe. Touches nothing on the car. Both outcomes are diagnostic: the
  map returning at once means the car is still publishing it.
- **reset live SLAM** — calls `/slam_toolbox/reset`. **Refused while any
  driving node is running**, on the reasoning already recorded in
  `killable_nodes`: a controller with a frozen pose is more dangerous than
  one with no pose. `pause_new_measurements` is hard-coded `False` and
  deliberately not a parameter.
- **delete a saved run** — permanently removes a run directory under
  `map_roots`. Gated on typing the run's name exactly, plus a content
  digest that refuses a delete from a page listing that has gone stale.
- **The deletion unit is a whole run directory, never a file inside one.**
  `map.yaml` names its image relatively, the racing line was optimized
  against that grid, and the pose graph is the only route back to a map
  whose save raced. Deleting `map.pgm` while leaving `map.yaml` makes
  `map_server` fail to configure and `particle_filter` block in its
  constructor — so a partial delete is made impossible rather than
  discouraged.
- **`delete_run` does not `rmtree` the run.** It re-classifies every entry
  first, unlinks only what it recognised as run output, and finishes with
  `os.rmdir`, so an unexpected file aborts the delete instead of being
  swept up with it. `shutil` work runs on a worker thread, not the rclpy
  thread that serves telemetry.
- New `web_dashboard/mapstore.py` (ROS-free, 133 tests) and
  `proccontrol.DRIVING_CONTROLLERS`. `package.xml` gains
  `<exec_depend>slam_toolbox</exec_depend>`; the import is guarded so a
  machine without it still gets a working dashboard.
- **New parameters** in `config/web_dashboard.yaml`, all defaulting on:
  `enable_map_delete` (`true`), `map_roots`
  (`[~/.ros/racerbot_auto, ~/.ros/racerbot_sim/auto]`),
  `map_scan_interval_sec` (`10.0`), `enable_slam_reset` (`true`),
  `slam_reset_service` (`/slam_toolbox/reset`), `slam_reset_timeout_sec`
  (`10.0`).
- **Security posture is unchanged and now weaker in one direction:** this
  port has no authentication, and deleting a map is the first irreversible
  thing on it. `enable_map_delete: false` at a shared venue. The typed name
  guards against a mis-tap, not against someone hostile.

Tests: 232 → 400 in `web_dashboard`, still running with no ROS sourced.

## 2026-08-22 (later) — A racing line worth racing, and a filter to follow it with

Builds on the GPU work below: with the particle filter finally affordable,
the auto-map-race flow can hand localization over for the race, and the
optimizer that was always sitting in the repo can run inline. Validated in
`racerbot_sim`; **needs on-car validation**, and see the speed warning.

### pure_pursuit — raceline optimization is now part of the auto flow

- **`auto_map_race_node` optimizes the racing line before racing it.**
  `_write_profile` previously cleaned up the recorded lap and paced it, but
  never reshaped it — the car raced wherever `gap_follow` happened to drive.
  It now feeds that cleaned lap to `raceline_optimizer` as a *seed*,
  re-derives the minimum-curvature line inside the measured track width, and
  paces **that**. Curvature is re-measured from whichever line wins; pacing
  new geometry with old curvature would have thrown the gain away.
- **It only races the optimized line if that line is genuinely better.**
  Three gates, any failure falling back to the cleaned recording: wall
  clearance below `profile_wall_clearance` is a hard refusal; steering past
  the rack limit is refused only when *also* worse than the recording (which
  `prepare` itself merely warns about, so an absolute bar would have
  disabled the feature on exactly the tight courses it was tested on); and
  **estimated lap time must actually improve**.
- **That last gate matters more than it sounds.** Minimum curvature buys
  corner speed by using the full track width, which lengthens the path, so
  it only pays where curvature is the binding limit. Measured: a lumpy
  ~34m circuit went 13.70s → 12.69s, but this car's own 13.3m test course
  produced a 16.9m line a second per lap *slower* — correctly refused. A
  "not faster" log line is the feature working.
- New `optimize_*` parameters in `auto_map_race.yaml`, `optimize_raceline`
  defaulting **true**. Saves `raceline_optimized.csv` beside the raw and
  profiled files.

### pure_pursuit — the optimizer is 4x faster

- **`solve_lateral_offsets` forced `lsq_solver='lsmr'`.** Profiled on the
  Jetson: a 126m circuit spent 55s in `trf_linear`, of which 46s was 781
  `lsmr` calls and ~36 million Python-level matvec dispatches through
  scipy's `LinearOperator` — big-sparse machinery on a few hundred unknowns.
  Densifying and asking for the direct solve is **55.7s → 13.9s**, same line
  to within 0.9mm, identical mean curvature. New `DENSE_SOLVER_MAX_UNKNOWNS`
  (4000) keeps the old iterative path for anything genuinely large.
  This is what makes the step affordable inline, while the car is stopped.

### pure_pursuit + racerbot_launch — localization hands over for the race

- **The racing phase now uses the particle filter, not `slam_toolbox`.**
  Once the map is saved, `auto_map_race_node` spawns localization against
  it, seeds `/initialpose` from the pose SLAM already knows, waits for
  `pf_settle_poses` consecutive poses, then republishes *the filter's*
  estimate on `pose_topic`. `pure_pursuit_node` is not reconfigured — this
  node always owned that topic, so the handover changes the source, not the
  wiring. `slam_toolbox` measured 106 pose jumps >0.12m in a 136s lap.
- **Fails safe at every step**, each logging its reason and leaving the car
  on `slam_toolbox`: no saved map, a filter that will not start, one that
  never converges inside `pf_startup_timeout_sec`, or one that goes quiet
  later. Demotion is one-way for the rest of the run — flapping between two
  pose sources at speed is worse than either.
- **New `racerbot_launch/launch/localize_run_launch.py`.** `particle_filter`'s
  own `localize_launch.py` could not be used: it points `map_server` at the
  map packaged *inside the submodule*, and hardcodes `use_sim_time: True`,
  which on the real car waits on a `/clock` that never ticks. Being a
  submodule, neither is fixable in place.
- **The `/tf` remap in that file is load-bearing.** `particle_filter`
  publishes `map -> laser` unconditionally with no parameter to stop it,
  while `slam_toolbox` still publishes `map -> odom` and `laser` already
  descends from `odom`. Two parents for one frame is a broken TF tree. Its
  transforms go to a dead topic; only its PoseStamped output is consumed.
  The `/map_server/map` *service* is deliberately left unremapped — moving
  it silently breaks the pairing with `map_server`.
- **The spawned process now dies with its parent** (`PR_SET_PDEATHSIG`).
  Found by leaking: killing the sim validation run left `map_server` and
  the particle filter alive afterwards, holding the service name the next
  run's handover needs. `start_new_session=True` was the cause and is gone,
  so an ordinary Ctrl+C now reaches `ros2 launch` and shuts it down in order.

### pure_pursuit — profile speed caps raised

- **`profile_max_speed` 2.0 → 4.0, `profile_max_lateral_accel` 1.2 → 2.5,
  `profile_max_accel` 3.0 → 6.0**, matching what `pure_pursuit_node` already
  enforces online, so the profile is no longer the binding limit. Asked for
  deliberately, to race the optimized line as fast as the controller allows.
- **This contradicts a measured warning that is still in the file above it.**
  The 0.39-0.57m cross-track error recorded through a corner was measured at
  2.5-3.0 m/s — *inside* the new range — and on a 1.8m indoor corridor these
  values are expected to put the car into a wall. Use `supervisor_config:=`
  with a copy holding the old 2.0/1.2/3.0 for anywhere tight.

### Validation

- `racerbot_sim`, `indoor_wide`, full map-then-race: optimizer accepted the
  line (mean |curvature| 0.2454 → 0.1471, estimated lap **15.16s → 11.38s**,
  0.30m wall clearance) in 0.44s; handover completed after 20 poses; 48
  racing ticks at up to 4.00 m/s; **zero errors, zero collisions, the filter
  never demoted**.
- 274 `pure_pursuit` tests pass (10 new covering both features).

## 2026-08-22 — The GPU ray caster, which was never actually built

Came out of an audit of what the Jetson's GPU is worth to this workspace
(new `docs/gpu-acceleration.md`). Needs on-car validation — the library
under localization was replaced.

### range_libc (build change, not a submodule bump)

- **`range_libc` was installed without CUDA, while `particle_filter` asks
  for `range_method: 'rmgpu'`.** `PyRayMarchingGPU.__cinit__` prints a
  warning and then returns *without constructing anything*, leaving a null
  pointer; the config is accepted, the node starts, and the first scan
  kills the process with `Aborted (core dumped)`. `race_launch.py` includes
  `localize_launch.py`, so the documented race-day launch would have lost
  localization on its first scan. Latent only because the auto-map-then-race
  flow keeps `slam_toolbox` running through the racing phase instead.
- **Rebuilt with `WITH_CUDA=ON`** (`cd src/range_libc/pywrapper &&
  WITH_CUDA=ON python3 setup.py install --user`). `setup.py` already
  carried the correct `-arch=sm_87`. `range_libc.SHOULD_USE_CUDA` now
  reports `True`. No submodule source was modified; this is a build
  artifact, and `colcon build` does not touch it (no `package.xml`).
- **Measured: 4.8× on the particle filter's own sensor-update path** —
  28.86ms → 5.96ms at the shipped 4000 particles, against a 25ms scan
  period, so the CPU path did not fit. Raw ray casting is 10.7× (34.05ms →
  3.19ms for 240k rays), returning bit-identical distances to the CPU
  method (max difference exactly 0.0).
- **CPU range methods are unchanged by the rebuild** — verified identical
  output, so `pure_pursuit`'s map-subtraction opponent detection
  (`map_subtraction.py`, `PyRayMarching`) is unaffected.

### racerbot_launch

- **`race_launch.py` now refuses to launch on a GPU config that would
  fail**, via a new `_check_gpu_ray_caster()` run at launch-description
  generation time. It reads `particle_filter`'s `localize.yaml` the same
  way the file already reads `pure_pursuit.yaml`. Two checks, both only
  when `range_method` is `'rmgpu'`:
  - `range_libc` built without CUDA → raises with the rebuild command,
    instead of letting the node abort on its first scan.
  - `max_particles × rays > 262144` → raises with the computed limit.
    Above that threshold `range_libc`'s GPU ray caster returns **silently
    wrong ranges** rather than failing: measured, one particle over the
    limit made 100% of returned ranges wrong, with one line on stdout.
    Cause is two defects in `numpy_calc_range_angles` (RangeLib.h) — a
    `ceil` that oversizes the chunk past the `CHUNK_SIZE` device buffer,
    and an offset computed from `num_in_chunk` where it means
    `particles_per_iter`. Both upstream, both documented in
    `docs/gpu-acceleration.md`.
  - New module constants `LIDAR_BEAMS` (1081, matching
    `racerbot_sim/sim_bridge.py`) and `RANGE_LIBC_CHUNK_SIZE` (262144).
    No new ROS parameters.
- With the shipped `max_particles: 4000` and `angle_step: 18` (61 rays),
  the limit computes to 4297 particles — the current config passes with
  about 7% headroom.

### docs

- **New `docs/gpu-acceleration.md`** — the full audit: what is and isn't in
  this Jetson (no DLA, no PVA, no NVENC, no CUDA in OpenCV, all verified on
  the board), why the driving loops stay on the CPU (`gap_follow` 2.8% and
  `pure_pursuit` 4.3% of their 25ms budget, against a 36–61µs GPU round
  trip), and what the idle tensor cores would need to be worth using.
- **`docs/racing-autonomy.md` corrected.** It described `range_libc` as
  doing "fast GPU-accelerated ray casting", which was not true at the time
  it was written. It is true now, with the two conditions stated.

## 2026-08-19 — Making a mapping run finish, and finish quicker

Everything here came out of one run's logs (`~/.ros/log`, 2026-08-19
18:46). Needs on-car validation.

### gap_follow

- **`gap_follow_node` would not start under any mapping launch file.** The
  previous commit added `corner_speed_wide` and a startup check enforcing
  `corner_speed <= corner_speed_wide <= max_speed`. Every mapping launch
  file overrode `max_speed` to `1.0` and nothing else, leaving the
  packaged `corner_speed: 1.1` / `corner_speed_wide: 1.4` above it, so the
  node exited with `ValueError` 1.8s after launch. Nothing then published
  `/auto_map/drive`, the supervisor recorded no lap, and
  `pure_pursuit_node` sat in `waiting_for_profile` with the car
  motionless — three symptoms, one dead node.
- **New `gap_follow/speed_overrides.py`.** Pure Python, no rclpy. Given a
  requested speed cap it scales every parameter coupled to `max_speed` by
  the same factor and clamps the result into `[min_speed, max_speed]`,
  preserving the ordering the node validates.
- **The mapping speed cap is gone by default.** `mapping_max_speed` /
  `mapping_min_speed` now default to empty, meaning "no cap": the car maps
  at `gap_follow.yaml`'s tuned speeds and the sensed curvature/clearance
  caps do the limiting. On the 2026-08-19 run the forced 1.0m/s cap was
  the binding limit on **154 of 191** logged driving ticks — 81% of the
  run was the car obeying an override rather than anything it could see.
  Pass a number to cap it for an unfamiliar course; the coupled corner
  caps scale with it automatically.

### pure_pursuit

- **Lap turning is counted in the `odom` frame.** Absorbing a SLAM
  correction also discarded the car's real turning during that tick. The
  run absorbed **106 corrections in one 136s lap** and measured 335° for a
  genuine revolution, against a 300° gate. Odometry is never
  re-optimised, so the gate no longer depends on how busy the pose graph
  was. Falls back to map yaw when no `odom` transform exists. New
  parameter: `odom_frame` (default `odom`).
- **A lap that will not close now widens rather than running forever.**
  Past `closure_widen_after_revolutions` (new, 1.25) the proximity gate
  opens in proportion to the extra turning, up to `max_closure_distance`
  (new, 4.0m). A reactive controller does not repeat its line, and on the
  126m course this car maps each missed closure costs 2.3 minutes.
- **`LapRecorder.lap_points()` trims a recording to its final
  revolution**, so a closure that needed two laps still produces a
  one-lap racing line instead of two overlapping ones.
- **Lap progress is reported in plain terms.** The decision log now leads
  with `~34% round, ~83m to go (~90s at last lap's pace)`, and the
  supervisor prints an estimate of what lap 2 will cost when lap 1 closes.
  The operator on 2026-08-19 stopped a working run 47s into lap 2; the
  raw gate numbers gave no way to tell progress from a stall.
- **Stray-return speckle is cleaned out of the saved map and the copy the
  racing line is checked against.** New `occupancy_map.despeckle_grid()`
  removes small occupied blobs *only* where nothing unknown sits anywhere
  in their neighbourhood — a real object occludes and so keeps an unknown
  shadow behind it, while a stray beam is a direction the LiDAR swept
  clear on every other pass. Measured on this car's own saved map: 12
  phantom blobs in clear track removed, the 34 small ones against walls
  and the map edge deliberately kept. **Not** applied to the live `/map`
  used for opponent detection. New parameter:
  `map_despeckle_max_cells` (default 4).
- **New latched `/auto_map_race/controller` topic** naming the controller
  currently selected, for the dashboard. Read-only, published after the
  tick's drive command, disables itself rather than the node on error.
- **`pure_pursuit_node` broadcasts its active racing line** on a latched
  `/racing_line` topic, once per loaded profile and never from the
  control loop. Read-only; disables itself rather than the node on error.

### web_dashboard

- **The racing line is drawn on the map**, coloured by its own target
  speed (green fast, amber slow), from a new latched `/racing_line`
  topic that `pure_pursuit_node` publishes once per loaded profile.
  Its presence is the visual answer to "is pure pursuit racing yet" --
  it cannot appear until a profile is accepted. New parameters:
  `racing_line_topic` (both nodes) and `racing_line_max_points`.
- **The tuning panel says which node is driving.** During an
  `auto_map_race` run both controllers are online and tunable while only
  one drives. The driving one is sorted to the top and badged in green;
  the other is dimmed and badged "not driving". Every live-tune write of
  the 2026-08-19 run went to `pure_pursuit_node` while `gap_follow_node`
  drove the entire run. New parameter: `controller_topic`.

### racerbot_launch

- **New `config/slam_tracking.yaml`**, layered over the vendored
  `f1tenth_online_async.yaml` by `slam_launch.py` so the vendored file
  stays clean and an upstream sync cannot take our tuning with it.
  `minimum_time_interval` 0.5 → 0.2, `minimum_travel_distance` 0.5 →
  0.25, `minimum_travel_heading` 0.5 → 0.25, `throttle_scans` 1 → 2,
  `loop_search_maximum_distance` 3.0 → 8.0,
  `loop_match_minimum_chain_size` 10 → 20, `ceres_loss_function` None →
  HuberLoss. The loop-search radius is the likely reason the map was not
  closing: the course is 126m round and stock SLAM only looked 3m for a
  candidate. New launch argument: `slam_tracking_file`.
- **`auto_map_race_launch.py` starts `race_diagnostics` by default**
  (`diagnostics:=true`, `record_bag:=false`), so a run leaves pose-lag and
  watchdog numbers instead of only terminal scrollback.

### Hardware findings (no code change)

- **The VESC's onboard IMU reports all zeros.** Measured directly:
  `/sensors/imu/raw` publishes at a steady 50Hz with every gyro and
  accelerometer axis exactly `0.0`, and no gravity vector on any axis.
  Either the board has no IMU fitted or it is disabled in firmware
  (VESC Tool -> App Settings -> IMU). This matters because `/odom`'s
  heading is integrated from the *commanded* steering angle through a
  bicycle model, with nothing measuring actual rotation — a working gyro
  is the single biggest available improvement to it. Recorded in
  [hardware-reference.md](docs/hardware-reference.md) with a re-check
  procedure.

### docs

- **New [localization.md](docs/localization.md)** — where the position
  estimate comes from, why it arrived late, what was tuned, and a ranked
  plan for what would improve it next. Records that `vesc_to_odom`
  integrates *commanded* steering angle through a bicycle model, that the
  VESC IMU is dead, and why "fuse odom and LiDAR with an EKF" is not a
  change: SLAM already fuses them, and an EKF between them would
  double-count the odometry.

## 2026-08-08 — A dashboard the car can afford, and a camera worth watching

### web_dashboard

- **The map is sent as changes, not as a map.** `slam_toolbox` republishes
  its entire grid every `map_update_interval` for as long as it is mapping
  — which is exactly while somebody is driving — and at the levine map's
  2048×2048 that is 4MB a message, 819 kB/s, per open browser tab. New
  `mapstream.py` answers each grid with a *keyframe* (first sight, a
  resize, a new tab, or every `map_keyframe_sec`), a *patch* covering only
  the rectangle that changed, or nothing at all when the grid is identical
  to the last one. Measured: ~200 bytes where it used to be 4MB. Every
  frame carries a sequence number, and the browser applies a patch only if
  it is the exact successor of the last one — on a gap it waits for a
  keyframe rather than painting a map that is quietly wrong.
- **~155 WebSocket frames a second became ~40.** Pose, command, speed,
  intent, stopwatch and stats are collected into one `batch` frame at
  `telemetry_rate_hz`. A frame costs about the same however small it is, so
  the framing was the cost. Latest-wins per type — except `/drive_intent`,
  whose *state transitions* the browser builds its decision log from, so
  those are queued in order and every one survives (`batching.py`).
- Scans are uint16 millimetres rather than float32: half the bytes, for a
  difference below one screen pixel and below the LIDAR's own accuracy.
  Intent messages drop their commanded path while it is indistinguishable
  from the desired one.
- `protocol.map_cells`/`scan_ranges` now hand rclpy's `array.array`
  straight to the wire instead of unpacking 4.2 million ints into
  `struct.pack`: 178ms → 2.2ms per map message, byte-for-byte identical
  (the existing round-trip tests pass unedited, which is the proof).
- **Nothing happens at all when no browser is connected.** Every
  broadcasting callback checks first; a newly connected tab is caught up by
  `send_initial_state()` as before. Stats still sample on their timer and
  intent messages are still validated, since both are useful with no tab
  open.
- Measured over a live 60s mapping run: **914 kB/s → 61 kB/s** (7.1 →
  0.48 Mbit/s), zero dropped frames. Re-runnable:
  `tools/web_dashboard/bench_protocol.py`.
- **The sidebar can be used now.** The decision log had `overflow-y: auto`
  and was impossible to scroll, because `#overlay` was
  `pointer-events: none` so the wheel went past it to the canvas and
  zoomed the map. And the sidebar had no height bound while the page could
  not scroll, so on a laptop or phone its bottom — the decision log, the
  tuning button, "reset view" — was rendered off-screen and unreachable.
  It is now bounded and scrolls, with the mode banner and view controls
  pinned outside the scroll region.
- Sections are collapsible `<details>` that remember their state per
  browser, and a collapsed one still shows its headline value in its
  header. Feeds, vehicle and system moved to two compact columns with the
  long form in each row's tooltip. Scrollbars are styled, since the
  defaults are invisible on this theme.
- The browser coalesces repaints through `requestAnimationFrame` (pose
  alone used to force 40 full canvas repaints a second) and paints map
  cells through a 256-entry palette. A hidden tab now draws nothing.
- **New parameters:** `telemetry_rate_hz` (20.0), `map_compression`
  (true), `map_patching` (true), `map_keyframe_sec` (30.0),
  `scan_encoding` (`u16mm`), `scan_decimation` (1).
  **Changed default:** `stopwatch_update_rate_hz` 10.0 → 4.0, since the
  browser runs the clock between updates.

### usb_cam_stream

- **Two tiers.** `/stream` is a small preview (`preview_width`, default
  480) for the dashboard's camera inset; `/stream?tier=full` is the
  camera's own resolution for the recording view. The inset is at most 220
  CSS pixels wide and was being fed 1280×720 at 12–18 Mbit/s — about 34×
  more picture than it could show, competing with the dashboard's own
  telemetry for the same link. Each tier is encoded once and shared, and a
  tier nobody is watching is never encoded at all.
- **The blur was a second JPEG generation.** The camera is asked for
  MJPEG, OpenCV silently decodes it to BGR, and this node re-encoded it.
  With `passthrough` (default true) the camera's own JPEG is served
  untouched — sharper, and it removes a decode and an encode per frame.
  Probed at startup, falls back cleanly, logs which mode is active.
- Frames are pushed as soon as they exist instead of polled for on a
  `1/stream_fps` sleep, which used to add up to 66ms to every frame.
  Encoding moved to its own thread and always works on the newest frame,
  dropping any backlog rather than letting latency accumulate — in
  `image_topic` mode it used to run on the rclpy executor thread, stalling
  every other ROS callback behind a 720p encode.
- `image_topic` mode subscribes with sensor QoS at depth 1, not the default
  10: a ten-frame backlog is a third of a second of stale frames.
- Downscaling uses `INTER_AREA`, so the preview is legible rather than
  aliased.
- **New parameters:** `preview_width` (480), `preview_quality` (65),
  `full_width` (0), `full_quality` (90), `passthrough` (true).
  **Changed default:** `stream_fps` 15.0 → 30.0 (now a cap, not a poll
  rate). **Deprecated:** `jpeg_quality` — still honoured, applied to both
  tiers, warns.

### tools

- `tools/web_dashboard/bench_protocol.py` — packing speed, bytes on the
  wire, and WebSocket framing cost at real dimensions (2048² map, 1081
  beams), with thresholds so it fails on a regression.
- `tools/web_dashboard/check_wire_format.py` — drives `dashboard_node`'s
  real callbacks and checks what it puts on the wire.
- `tools/racerbot_sim/capture_dashboard.py` speaks the new protocol and
  reports kB/s per message type, keyframe/patch counts, and any patch it
  had to ignore.

## 2026-08-08 — The automatic map-to-race path, and a simulator that can see it

### racerbot_sim (new package)

- F1TENTH Gym behind the car's real topics. `gym_bridge_node` replaces
  `urg_node` and the whole VESC chain: it consumes `/ackermann_cmd` and
  publishes `/scan` (1081 beams over 270°, matching the Hokuyo UST-10LX),
  `/odom`, and the `odom->base_link` transform. `sim_joy_node` replaces the
  physical F710 with a synthetic LB hold. `sim_bringup_launch.py` mirrors
  `bringup_launch.py`, keeping the real `ackermann_mux` and the real static
  `base_link->laser` transform, and `sim_auto_map_race_launch.py` *includes*
  `racerbot_launch`'s own `auto_map_race_launch.py` rather than copying it,
  so what is validated is the launch file people run.
- Odometry is dead-reckoned from wheel speed and the *commanded* servo angle,
  exactly as `vesc_to_odom` does, so it drifts the way the car's does.
  Ground truth is published separately on `/sim/ground_truth_pose` in an
  unconnected frame, for scoring only.
- **Hard interlock against real hardware.** Neither node publishes anything
  while `vesc_driver_node`, `ackermann_to_vesc_node`, `vesc_to_odom_node`,
  `urg_node` or `joy` is on the ROS graph, re-checked continuously so
  bringing the car up after the simulator silences the simulator too. A
  synthetic `/joy` holding LB defeats the workspace safety policy by design,
  and a second `/scan` publisher is its own hazard.
- Room-sized generated tracks (`indoor_oval`, `indoor_tight`, `indoor_wide`),
  because the official 300m+ circuits make one automatic-mode run over twenty
  minutes long, and a validation nobody runs is how the automatic mode got
  into the state it was in.
- `tools/racerbot_sim/run_auto_map_validation.py` runs the whole composition
  solo, with a parked car, and in traffic, and exits non-zero if any scenario
  fails to reach racing or touches anything. All three pass
  (`docs/auto-map-sim-results.json`, seed 12345): 138.7m, 29.7m and 83.1m
  raced in ~80s each, zero wall contact and zero car-to-car contact.
  `tools/racerbot_sim/capture_dashboard.py` renders what the web dashboard is
  drawing as a PNG, from the terminal.
- Found along the way: in the pinned Gym revision, **wall collisions never
  fire** with this workspace's vehicle parameters, because `side_distances`
  computes to all zeros and the test reduces to "did a beam return under
  5mm" against a 0.05m `range_min`. `tools/f1tenth_sim/run_validation.py`'s
  "no collision" column has therefore always meant "not detected".
  `racerbot_sim` samples the padded body against the occupancy grid instead.

### web_dashboard

- **The map was not glitchy; the camera was.** The view auto-framed the map
  by re-deriving centre and zoom from *every* `/map` message, and
  slam_toolbox resizes and re-origins its grid constantly as the map grows,
  shrinking as often as growing. Measured over 130s of mapping: 27 map
  messages, **18 view disturbances, the picture sliding up to 3.6m and
  rescaling by up to 36%** while the map itself was fine. It now frames the
  map once and re-fits only when the map no longer fits on screen — the
  same run gives **2**, both of them the map genuinely growing.
- Map and scan headers now carry `bytes`, and the browser drops any binary
  frame whose length disagrees rather than painting it. The browser holds
  one "what does the next binary mean" slot, so a header that never got its
  binary made every later payload decode as the wrong type — a 1081-beam
  scan read as occupancy cells is 4324 bytes against an 80000-cell header,
  every read past the end undefined, every colour NaN, and the map paints
  as garbage rather than failing. `applyMap` also refuses a short payload
  outright, dropped frames are counted in the mode banner, and
  `_send_to_all` now drops a client whose pair it could not complete (it
  caught only `WebSocketClosedError` before) so that browser reconnects and
  resynchronises instead of decoding everything wrongly from then on.
  Across ~7,500 binary frames of validation **zero** failed the check, so
  this is a guard against an unobserved failure, not a fix for a seen one.
- `tools/racerbot_sim/capture_dashboard.py` renders what the dashboard is
  drawing as a PNG from the terminal, and doubles as its test instrument:
  `--seconds`/`--interval` watches a whole run, validates every frame, and
  exits non-zero if any failed. It also replays both view-fitting policies
  over the recorded map sequence, which is where the numbers above come
  from.

### pure_pursuit — the recorded racing line

- New `recorded_path.py`: trims a recording to one revolution, resamples it
  to uniform spacing, and low-passes the closed loop in space, discarding
  features shorter than the car's own 1.22m turning circle. Reports peak
  curvature against the steering rack, the seam heading error, and how far
  the cleanup moved the line off the one actually driven.
- `auto_map_race_node` now **refuses** to hand pure pursuit a line needing
  more than `profile_reject_ratio` (1.5) times the rack limit, or needing
  more than the rack has over more than `profile_reject_fraction` (0.25) of
  the lap, and says so with numbers. Every racing line this car had ever
  generated demanded more steering than the rack has on a third of its
  waypoints, peaking at 47° against a 14.9° limit.
- The supervisor now subscribes to `/map` and checks the finished line
  against SLAM's own occupancy grid, because filtering a recorded lap
  rounds its corners *inward*, toward the wall, and on a course whose
  corners are near the car's 1.22m turning circle that is the direction
  that hurts: a measured run finished with the line 0.05m from a wall --
  under the car's own 0.155m half-width -- while curvature, seam error and
  deviation all read healthy. Clearance outranks curvature when choosing
  the filter cutoff. The requirement is capped by the driven line's own
  clearance, because mapping with traffic paints the other cars into the
  grid -- the two-car scenario produced a recorded line measuring 0.00m of
  clearance on a lap the ego had just driven twice without touching
  anything, and refusing there would be refusing to believe the lap that
  happened. `OccupancyMap.from_grid_message` is new, and stays ROS-free by
  taking the message's fields rather than the message.
- **New parameters** in `auto_map_race.yaml`: `minimum_lap_turn_deg` (300),
  `max_pose_jump_m` (0.12), `profile_max_steering_angle`, `profile_wheelbase`,
  `map_save_retries`, `map_save_retry_delay_sec`,
  `profile_min_feature_wavelength`, `profile_curvature_margin`,
  `profile_max_deviation`, `profile_reject_ratio`, `profile_reject_fraction`,
  `map_topic`, `profile_wall_clearance` (0.30), and
  `profile_map_occupied_threshold`.
  **Removed**: `profile_smoothing_window` (replaced by the above).
- **Changed defaults**: `minimum_lap_distance` 20.0 → 5.0 (it was longer than
  the loop this car is driven on, so closure could not fire until the car had
  been round *twice*, and every recorded line was two overlapping laps);
  `profile_max_brake` 8.0 → 3.0 (it is a braking-authority assumption in the
  velocity profile, not a slew rate, and 8.0 was 2.7× what `gap_follow.yaml`
  will assume for the same car); `profile_max_speed` 4.0 → 2.0 and
  `profile_max_lateral_accel` 2.5 → 1.2 (that was the surveyed-track profile,
  applied to a course discovered thirty seconds earlier at 1 m/s). What sets
  that ceiling is width, not grip: the line comes from `gap_follow`, which
  drives 0.25-0.35m from a corner's wall, and pure pursuit's cross-track
  error through a corner near this car's turning circle measured 0.39-0.57m
  at 2.5-3.0 m/s. On a 1.8m indoor corridor those do not both fit, at any
  speed where the error exceeds the room.
- `closure_distance` 0.75 → 1.5. That gate existed to keep the start/finish
  seam short, because the seam used to be a naked chord from the last
  recorded point to the first; `recorded_path` now trims and resamples it
  instead, so the gate can be what it should always have been -- "did the
  car come back past its start". 0.75m was also simply unachievable for a
  reactive controller weaving around traffic: the two-car scenario drove
  413m and 3726 degrees of turning -- ten laps -- passing 6.5m wide of its
  start every time, without ever closing. The progress log now warns
  explicitly once two laps of turning have gone by with no closure, instead
  of leaving that to be inferred from two numbers.
- `LapRecorder` now absorbs SLAM pose corrections into the already-recorded
  path instead of recording them as geometry, and counts them in the progress
  log. Closure is gated on accumulated yaw, which does not need to know how
  big the course is.

### pure_pursuit — saving the map

- The occupancy-map save is retried (`map_save_retries`, 3, spaced
  `map_save_retry_delay_sec`). slam_toolbox runs nav2's `map_saver` inline
  and `map_saver` gives up after ~2s if no `/map` arrives, while `/map` is
  only republished every `map_update_interval` (5s) — so whether the save
  works is a race against when the request lands, and the same run fails or
  succeeds with nothing else different. A retry lands in a different part of
  the window; if all attempts fail the run still continues and says the pose
  graph can be turned back into a map with `deserialize_map`. The handover
  gate deliberately does not treat a save as settled while a retry is due.

### pure_pursuit — the reactive safety net

- The `emergency_obstacle` hard stop is no longer a latch. A stopped car
  cannot clear its own safety cone, so the stop, on its own, ended the run
  wherever it fired. It now crawls toward a genuine opening at
  `emergency_escape_speed` (0.25 m/s) when the whole body still has
  `emergency_escape_clearance` (0.10m) of room and a gap deeper than
  `emergency_escape_min_gap` (0.8m) exists. The contact and stale-scan stops
  still win outright; `emergency_escape_speed: 0.0` restores the old
  behaviour. Same reasoning as `gap_follow`'s `escape_creep_speed`.

### racerbot_launch

- `auto_map_race_launch.py` takes `supervisor_config:=`, so a course can have
  its own racing profile without editing the packaged config.

### Tests

- `test_overtakes_toward_the_*` asserted the *sign* of a steering command
  that back-to-back `control_loop()` calls left pinned within a thousandth
  of a radian of zero -- `max_steering_rate` permits about 0.0001 rad per
  microsecond-long tick, so the sign was scheduler noise, and a few extra
  microseconds of work per tick flipped it. They now give the slew limiter a
  realistic time step (the idiom already used by
  `test_shaped_speed_ramps_up_to_the_profiled_speed`) and assert the pass
  side as a comparison between passing left and passing right, which is what
  the claim actually is: the 0.35m offset applied 4m ahead is about five
  degrees of bearing, a nudge on top of the racing line's own curvature
  rather than a replacement for it, so on that fixture's track both passes
  come out positive.

## 2026-07-25 — Runtime autonomy decision logging

### gap_follow and pure_pursuit

- Both local autonomy nodes now print immediate `STOP [reason]` /
  `DRIVE [reason]` state transitions and one-second steady-state summaries in
  their launch terminals. The diagnostics include the measurements,
  thresholds, algorithm choice, and final steering/speed command that explain
  each decision.
- Pure pursuit reports deadman/profile/localization/cross-track/LIDAR stop
  gates; path target/lookahead/curvature/profile-speed choices; reactive
  overrides; and opponent/overtake gating. Gap follow reports deadman and scan
  failures, emergency/no-gap stops, chosen gap bearings, steering clipping,
  and steering-based speed scaling.
- New `decision_log_period_sec` parameter in both packages controls periodic
  summaries (`1.0` s default, `0.0` for transitions only). Gap follow also has
  a `scan_timeout_sec` diagnostic watchdog (`0.5` s) so a dead scan stream no
  longer causes an unexplained mux timeout. Driving and deadman behavior are
  unchanged.

### Automatic workflow, waypoint recorder, and camera stream

- `auto_map_race_node` now reports its final forwarded/stopped command and why:
  deadman state, missing/stale child commands, selected controller, lap number
  and closure measurements, profile loading, and transition hold time. Its new
  `decision_log_period_sec` defaults to `1.0` s.
- `waypoint_recorder_node` now reports missing/stale pose input and live
  recording progress (waypoint count, path length, sample spacing, and output
  file). New defaults are `pose_timeout_sec: 1.0` and
  `status_log_period_sec: 2.0`.
- `usb_cam_stream_node` now reports device/topic wait states, negotiated camera
  mode, frame health, loss/recovery, stale frames, and conversion/JPEG errors.
  New defaults are `frame_timeout_sec: 2.0` and
  `status_log_period_sec: 5.0`.

## 2026-07-21 — F1TENTH Gym validation and automatic map-to-race launch

**Simulator validation complete; physical validation still required.** The
workspace now has a pinned, local F1TENTH Gym setup and deterministic headless
runner. Gap follow and pure pursuit completed Spielberg, Silverstone, and
Brands Hatch without collisions; a two-car Spielberg run completed one pass
and a full independently measured lap with neither car colliding. See
`docs/simulator.md` and `docs/f1tenth-sim-results.json`.

### pure_pursuit

- Simulator-tuned lookahead is now `0.6 + 0.15*v`, capped at `1.5m`; the old
  2.0m lookahead at 4m/s cut corners and collided in the dynamics model.
- Velocity-profile defaults are now `v_max=4.0m/s`, `a_lat_max=2.5m/s²`, and
  five smoothing passes.
- Map subtraction is the default opponent detector and now gates general
  reactive avoidance too. Added `avoidance_fallback_trigger_distance: 0.7m`
  for operation before a map arrives; the map-aware trigger remains `1.5m`.
- Active overtakes no longer get overwritten by the generic 1m/s avoidance
  command. Emergency stopping and stale-scan stopping still always win.
- Opponent progress-rate tracking now wraps correctly at start/finish, and the
  simulator-validated lateral pass offset is `0.35m`.
- `pure_pursuit_node` can opt into a stopped waiting state and atomically load a
  generated `waypoints_file` at runtime. Normal launches retain fail-fast
  behavior for a missing profile.
- New `auto_map_race_node`, config, and top-level
  `auto_map_race_launch.py`: one command starts cautious autonomous mapping,
  detects/records closed laps, generates and loads the profile, saves the map
  and pose graph, then transitions to pure-pursuit racing while SLAM remains
  online. Existing mapping, recorder, profiler, and saved-map race modes remain.

### gap_follow and tooling

- Gap follow's disparity-extender pipeline is now documented explicitly and
  validated for collision-free laps on all three simulator tracks.
- Added `tools/f1tenth_sim/setup.sh`, the deterministic validation runner,
  pinned dependencies, simulator documentation, and a checked-in JSON report.
- The LB deadman requirement is unchanged and is enforced again by the new
  command-selector supervisor.

## 2026-07-09 — Algorithm review follow-up: safety fixes, profile quality, map-based opponent detection

Full review + implementation session over the driving algorithms
(`gap_follow`, `pure_pursuit`). 86 tests pass (73 plain-pytest unit
tests, 13 rclpy integration tests); both packages rebuilt with
`colcon build --symlink-install --packages-select pure_pursuit gap_follow`.

**⚠ Not yet validated on-car.** Everything below passed the test suites
only. Before racing it, run the standard ladder from
`docs/writing-your-own-node.md`: static topic check → wheels off the
ground with LB held → low speed on open floor. The deadman policy is
untouched: `enable_deadman` stays `true` everywhere.

### pure_pursuit — bug fixes

- **Recovers after a localization jump** (`pure_pursuit_node.py`).
  Previously, if the particle filter re-converged somewhere outside the
  ±40-waypoint nearest-search window (pose jump, re-seed, car
  repositioned), the cross-track watchdog stopped the car *permanently*
  until a node restart. Now the watchdog retries once with a full-line
  search and only stops if that also misses; it stays un-anchored while
  lost so a recovered pose re-locks cleanly.
- **Overtakes complete on ego progress, not detection freshness**
  (`pure_pursuit_node.py`, `OpponentTracker`). Previously an active pass
  was cancelled 1.0s (`opponent_lost_timeout_sec`) after the opponent
  left the forward detection cone — which it always does when you pull
  alongside — snapping the steering target back onto the racing line the
  opponent might still occupy. Now, mid-pass, the opponent's position is
  dead-reckoned forward at its tracked progress rate and the pass ends
  only once the ego car is `overtake_clear_margin` past that predicted
  position. New parameter: `overtake_max_blind_sec` (3.0) — the hard cap
  after which a pass with no re-detection is abandoned instead.
- **Sub-`range_min` LIDAR readings no longer count as obstacles** in
  `_closest_in_cone` (the hard-stop / avoidance-trigger check). They are
  the sensor's "invalid" encoding, not a real 4cm object.

### pure_pursuit — map-subtraction opponent detection (new, opt-in)

- New detection mode: ray-cast the scan the LIDAR *should* see from the
  current pose using the saved map (`range_libc`, the same library the
  particle filter uses — new module `pure_pursuit/map_subtraction.py`),
  and flag whatever is meaningfully shorter than the map predicts.
  Unlike the shape heuristic, this cannot be fooled by wall corners and
  works with an opponent right in front of a wall. Comparison/clustering
  logic is in `racing_math.py` (`dynamic_beam_mask`,
  `detect_dynamic_cluster`) so it stays unit-testable without range_libc.
- New parameters (`config/pure_pursuit.yaml`): `opponent_detection_mode`
  (**ships as `heuristic`** — flip to `map` only after on-car
  validation), `map_topic`, `map_beam_step`, `map_subtraction_margin`.
  With no map received yet, `map` mode falls back to the heuristic with
  a warning rather than racing blind.
- `package.xml` gains `<depend>nav_msgs</depend>` (OccupancyGrid
  subscription, transient-local QoS).

### pure_pursuit — velocity-profile quality

- **Waypoint smoothing before profiling** (`racing_math.smooth_path`,
  wired into `generate_velocity_profile` as `--smoothing-window`,
  default 3, `0` disables). Localization jitter on the recorded line
  reads as curvature and produced phantom braking zones — on realistic
  recording geometry the raw jittered line measures >2× the true
  curvature; smoothed recovers it within 10%. The smoothed line is what
  gets written to the output CSV.
- **Friction-ellipse coupling** in `compute_velocity_profile` (default
  on; `--no-friction-ellipse` restores the old behavior for
  comparison). Accel/brake budget is scaled by
  `sqrt(1 − (v²κ/a_lat_max)²)` so corner-entry/exit speeds stop
  assuming the tires can brake at full force mid-corner. Strictly more
  conservative than the old profile.
- **Regenerate any profiled racing lines** you care about — existing
  CSVs still load fine, but they were paced with the old math.

### gap_follow — invalid-beam handling + physical clearance model

- Scan-processing logic split out of the node into importable
  `gap_follow/gap_logic.py` with its own plain-pytest test dir
  (`src/gap_follow/test/`), mirroring pure_pursuit's `racing_math.py`
  pattern.
- **NaN/invalid beams are no longer 0.0m obstacles.** Previously a
  single NaN dropout became the "closest obstacle": spurious emergency
  stops on scan noise, and the safety bubble carved around a beam with
  nothing in it. Invalid beams are now excluded from the e-stop check
  but stay non-free for gap selection (never steer into a blind spot);
  `+inf` correctly counts as free space at `max_range`.
- **Disparity extender**: every sharp range jump (obstacle edge) is
  extended by half a car width at that edge's distance, so gap selection
  is clearance-aware at *every* edge, not just around the single closest
  point.
- **Width-based safety bubble**: the fixed 20° bubble
  (`bubble_angle_deg`, **parameter removed**) is replaced by
  `atan2(car_width/2 + safety_margin, closest_dist)` — a fixed angle was
  far too little clearance up close and wastefully much far away.
- **Gaps narrower than the car are discarded** outright instead of being
  eligible to win as the least-bad option.
- New parameters (`config/gap_follow.yaml`): `car_width` (0.30),
  `safety_margin` (0.10), `disparity_threshold` (0.4).

### Machine setup (not a repo change)

- The `range_libc` **Python** module was never installed on this Jetson
  — `colcon build` only builds the C++ lib, so even `particle_filter`
  could not import it. Installed user-level:
  `pip3 install --user --break-system-packages cython`, then
  `python3 setup.py install --user` in `src/range_libc/pywrapper`.
  A fresh OS/user setup must repeat this (it survives
  `rm -rf build install log`, but lives in `~/.local`, not the repo).
