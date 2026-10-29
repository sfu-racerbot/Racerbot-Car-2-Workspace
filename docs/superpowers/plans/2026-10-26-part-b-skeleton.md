# Part B (IMU, observability, recording, overtaking) skeleton plan

> **Who this is for:** whoever expands and builds a Part B item, in this session or another, and the bringup-datalogger session (B1's recording contract).
> **Read first:** [the spec](../specs/2026-10-26-slam-and-gap-follow-plan.md), and [glossary.md](../../glossary.md) for [odometry](../../glossary.md#odometry--odom), [topic](../../glossary.md#topic), [scan](../../glossary.md#scan) and [deadman](../../glossary.md#deadman).
> **You'll be able to:** pick up any Part B item, see what it must deliver and when it may start, and expand it into a full plan.
> **Time:** 10 minutes to read. Each item is days of work once expanded.

> **For agentic workers:** this is a **skeleton**. Each item below must be expanded into its own full plan (superpowers:writing-plans) before anyone builds it. Expand one item at a time, in the order given, in this session or another.

**Goal:** everything from the 2026-10-26 requests that is not built in Part A: SLAM observability, `map_course` IMU flags, a simulated IMU, `gap_follow` IMU features, overtaking, and car 1's course in the simulator.

**Spec:** [docs/superpowers/specs/2026-10-26-slam-and-gap-follow-plan.md](../specs/2026-10-26-slam-and-gap-follow-plan.md), Part B and "Decisions already made". The Part A plan ([2026-10-26-part-a-slam-and-gap-follow.md](2026-10-26-part-a-slam-and-gap-follow.md)) builds the pieces several items reuse:
- `slam_metrics`
- `slam_bag`
- `slam_replay`
- `weave`
- `gap_logic.steering_damping`

## Global Constraints

Everything in Part A's Global Constraints applies unchanged, plus these decisions from 2026-10-26:
- **IMU failure:** each IMU feature switches itself off and `gap_follow` drives as it would without it. A loud warning goes to the log, and `/drive_intent` carries the reason.
- **Flags:** one per feature, plus `imu_all`, with the same names in `map_course` and `gap_follow_launch.py`.
  - All default off.
  - An explicit `false` beats `imu_all`.
  - Any IMU flag with `imu:=false` refuses to start.
- **Analysis runs after the run,** never on the car's CPU while it drives or maps.

## Order and entry criteria

| Item | Starts when | Why this order |
|---|---|---|
| **B6 Overtaking** and **B9 Two controlled cars, racing team A** | **Done 2026-10-29**, merged to `main` as PR #6 ([the B-1 plan](2026-10-27-part-b1-racing.md), "Outcome and lessons") | |
| B1 SLAM observability, B2 `map_course` IMU flags, B3 Simulated IMU, B4 `gap_follow` IMU features | B-1 merged to `main`. **Now one plan, two parallel tracks:** [the combined B-2/3 plan](2026-10-29-part-b23-slam-imu.md) | Disjoint code, one shared join (the IMU flags); B3's simulated IMU also closes a simulator gap B-1 found |
| B10 B-1 follow-ups | When the race schedule needs them; the floor test (R9) first | See B10 below |
| B7 Car 1's course in the simulator | **Done 2026-10-27** (`car1_course_2026_10_26`, commit `a343d44`) | |

B5 (recording every run) is done: the **bringup datalogger** (`auto_record`) merged to `main` on 2026-10-26. The SLAM-specific additions it still lacks are listed under B1.

---

## B1. SLAM observability

**Outcome:** after any run with SLAM, one command (or `map_course` automatically) says:
- which odometry SLAM used;
- how far each odometry source drifted;
- every correction and loop closure, sized at the car;
- per-lap closure and quality numbers;
- how consistent the final map is with the scans;
- when and why SLAM dropped scans.

A replay can **recreate exactly** what SLAM did.

### The recording contract (for the bringup datalogger)

The datalogger owns recording. It merged to `main` as `race_diagnostics`' `auto_record` (PR #4, `ed2c34e`). It runs `record_run.py` for every algorithm run and stands down beside another recorder, so `map_course`'s own `record_run.py` and `auto_record` never record the same run twice.

**Status against `main` at `ed2c34e`** (checked 2026-10-26):

| Item | Status |
|---|---|
| 1. Topic list | ✅ Already met: `record_run.py`'s list has every topic below, plus `/parameter_events` for live setting changes |
| 2. Snapshot when `slam_toolbox` appears | ❌ **To build.** `snapshot_params` runs once, `--delay` seconds after the recorder starts, so a node started later (SLAM in a gap_follow-then-map workflow) is never snapshotted |
| 3. CPU and thermal logs | ❌ **To build** |
| 4. Folder layout | ✅ Unchanged |
| 5. One recorder per run | ✅ Met by `auto_record` standing down; no `map_course` change needed |

**Build for B1, in `race_diagnostics`:**
- `snapshot_params` keeps watching the node list for the whole run and snapshots each node the first time it appears. Today it snapshots once.
- `record_run.py` also starts the two system loggers in item 3.
- `metadata/` gains the version of each apt-installed ROS package the run used (`slam_toolbox` first), because git state can't cover them.

SLAM's needs are:

1. **Always in the topic list.** A topic nobody publishes records nothing, so these cost nothing on runs without SLAM:
   - `/scan`, at full rate and never throttled;
   - `/tf`, and `/tf_static` with transient-local durability;
   - `/odom`, `/odom/imu`, `/odom/fused`;
   - `/imu/data`, `/imu/bno/data`, `/imu/vesc/data`, `/imu/bno/raw`, `/sensors/imu/raw`, `/sensors/core`, `/sensors/servo_position_command`;
   - **`/pose`**, slam_toolbox's, published once per scan it processes; **this is the processed-scan record**;
   - `/slam_pose`, `/map`, `/map_metadata`, `/slam_toolbox/graph_visualization`;
   - `/drive`, `/teleop`, `/ackermann_cmd`, `/joy`, `/drive_intent`, `/initialpose`, `/rosout`, `/diagnostics`.
2. **When `slam_toolbox` appears on the graph** (poll the node list), snapshot into `<run>/metadata/`:
   - its full parameter set, as `params/slam_toolbox.yaml`, in the same format `race_diagnostics`' `snapshot_params` writes today;
   - the `odom_frame` it was given;
   - `slam_toolbox`'s package version.
3. **On every run, with or without SLAM:**
   - `tegrastats --interval 1000` to `<run>/system/tegrastats.log`. It works without sudo on car 1, checked 2026-10-26.
   - 1 Hz CPU %, RSS and thread count per ROS node process from `/proc`, to `<run>/system/proc.jsonl`.
4. **Folder layout** stays `record_run.py`'s: `<run>/bag/` (mcap), `<run>/metadata/{params,git,files}/`, `<run>/events.jsonl`, plus the new `<run>/system/`. The replay and report tools read that layout.
5. **One recorder per run.** `auto_record` already stands down beside `map_course`'s recorder, and both use the same `record_run.py`, so everything lives in one place without changing `map_course`.

**Why no end-of-run save is needed:** `/pose` stamps mark exactly which scans SLAM processed. Checked on 2026-10-26's runs: 315 `/pose` against 314 SLAM updates (`-194913`), 229/228 and 257/255. That survives Ctrl+C.

### Tasks to expand

1. **`slam_replay --exact`:** run `sync_slam_toolbox_node` and play only the `/scan` messages whose stamps match `/pose` stamps. The test is that the replayed `/pose` sequence equals the recorded one to 1 mm and 0.1°.
2. **`slam_report.py`** (`tools/run_analysis/`, cached `analysis/slam.npz`, unsourced after the first run). It reports:
   - setup: odometry frame, speeds, distance;
   - heading drift of `/odom`, `/odom/imu` and SLAM against each other;
   - corrections and loop closures, from Part A `slam_metrics`;
   - per-lap closures, from `race_diagnostics.run_events`;
   - map consistency: the share of scan endpoints within one cell of an occupied cell of the final map, per lap;
   - scan drops against CPU, from `system/`;
   - wheel distance against scan-matched distance.
   - Output: `slam_report.txt`, `summary.json` and PNGs. `--compare runA runB` gives a side-by-side table and overlays.
3. **Automatic analysis:** the `map_course` wrapper runs `summarize_run`, `imu_report` and `slam_report` after the launch exits; `analysis:=false` skips them. The wrapper strips the argument before the launch.

**Tests (shape):**
- Every metric against a synthetic run with a closed-form answer, including map consistency with a known scan offset.
- Edge cases: empty, single sample, NaN, missing topics.
- `--exact` against a recorded run.

## B2. IMU flags in `map_course`

**Outcome:**
- `imu_lap_turns:=true` makes the lap counter count turning from the gyro (`auto_map_race_node` `odom_frame: odom_imu`). It is a parameter only, and needs `gyro_odom:=true`.
- `imu_all:=true` turns on `gyro_odom`, `imu_lap_turns` and `fused_speed`, and later every B4 flag. An explicit `false` overrides it.

**Files:** `src/racerbot_launch/launch/map_course_launch.py`, `src/racerbot_launch/test/test_map_course_imu.py`, `docs/launchstart.md`.

**Tests (shape):**
- Each flag sets its parameters.
- `imu_all` expands correctly, and an explicit `false` wins.
- Every refusal is checked with `pytest.raises(RuntimeError, match=...)`.
- The lap counter's frame name equals `vehicle_state.yaml`'s `imu_odom_frame`.

**Known limitation to document:** if `vehicle_state_node` dies mid-run with `gyro_odom` on, SLAM loses odometry and the map is lost. The car stays safe, because `gap_follow` steers from `/scan`. `slam_report` flags it.

## B3. Simulated IMU in `racerbot_sim`

**Outcome:** `gym_bridge_node` publishes `/imu/bno/data`, `/imu/vesc/data`, `/imu/data` and a latched `/imu/status` in `imu_bridge`'s format, all from the gym's true state:
- **Gyro z:** the yaw rate.
- **Accelerometer:** forward from the change in speed, sideways from speed × yaw rate, and gravity on z.
- **Noise:** measured from a stationary stretch of a real bag.

The real `vehicle_state_node` runs on top (`vehicle_state_launch.py hardware:=false` skips the BNO bridge and `imu_bridge_node`).

**Fault injection:** parameter `imu_fault`, one of `none`, `stale`, `status_fault`, `bias`, `scale` or `zero`, settable mid-run.

**Safety:** `imu_bridge_node` and the BNO bridge join `hardware_guard.py`'s refuse list.

**Files:** `src/racerbot_sim/racerbot_sim/{sim_bridge.py,gym_bridge_node.py,hardware_guard.py}`, `src/imu_bridge/launch/vehicle_state_launch.py`, `src/racerbot_launch/scripts/map_course` (it stops forcing `imu:=false` in sim).

**Tests (shape):**
- On a constant circle: the gyro equals v/R and the sideways acceleration equals v²/R.
- On a ramp, the forward acceleration matches.
- The noise spread matches its setting.
- Each fault mode gives exactly its output.
- `/imu/status` parses in `imu_health` and `neural_racer`.
- Reproduction: with `odom_yaw_rate_scale` about 1.1, `map_course` fails without `gyro_odom` and succeeds with it.

**Limit:** the gym's tyre force is linear in slip angle (`.sim/f1tenth_gym/.../single_track.py`), so the car can't slide. Grip features are checked for wiring and fall-back only.

## B4. `gap_follow` IMU features

Each feature gets its own flag, falls back as decided, keeps the four deadman deny-path tests, and has a design approved before it is built.

| Flag | Feature | Main input | Note |
|---|---|---|---|
| `fused_speed` | TTC braking and the acceleration ramp on `/odom/fused` | `/odom/fused` | Exists in `map_course`; add it to `gap_follow_launch.py` |
| `imu_grip_cap` | Slow down as measured sideways acceleration nears a limit | `/imu/data` linear y, speed | Thresholds come from bag replay, not the simulator (B3 limit) |
| `imu_understeer` | Slow down when the car turns much less than commanded | `/steering/measured` against the command | NaN below 0.3 m/s means no action |
| `steering_damping` | Gyro yaw-rate damping | `/imu/data` angular z | **Built in Part A Task 9** if its gate opens |
| `imu_traction` | Hold acceleration while the wheels outrun the car | `/odom` against `/odom/fused` | |
| `imu_impact_stop` | An accelerometer spike stops the car until LB is released and pressed again | `/imu/data` linear | A safety addition; its threshold comes from recorded bumps |

`imu_all` turns them all on, in both `gap_follow_launch.py` and `map_course`.

**Tests (shape):** per feature:
- the logic against closed forms;
- the four IMU failure modes (status `fault`/`waiting`, stale, never received, NaN) give today's command;
- the deadman deny paths with the flag on;
- a simulator run with B3, including a fault injected mid-run.

## B5. Recording every run

Owned by the bringup-datalogger session. Nothing to build here beyond the B1 contract. Part A Task 7 and B1 read whatever folder it writes, as long as it keeps the layout in the contract.

## B6. Overtaking (priority 1 after Part A)

**Decided 2026-10-27:** the team says passing is fine, because the course has enough room. The cleaned map of car 1's course measures **1.52 m median width** (1.30 m at the 10th percentile, 1.06 m at the narrowest; `car1_course_2026_10_26`). Two 0.31 m cars leave about 0.9 m at the median.

**The "about 2.4 m" figure came from today's conservative settings in `gap_follow.yaml`:**
- `opponent_radius: 0.30`, a 0.60 m footprint for a 0.30 m-wide car;
- `opponent_passing_margin: 0.15` m on each side;
- `opponent_follow_lateral` and `opponent_relevant_lateral`;
- `opponent_pass_max_steering: 0.10`, which allows no pass while turning.

So B6 is now a tuning-and-behaviour task, not a policy question.

**Build, in this order:**
1. **Stop stalling behind another car.** In the 2026-10-27 head-to-head (`eval_gap_multi.py`, car 1's course, seed 12346), the trailing car stopped 2.7 cm from the other car. It never restarted: creep speed is 0 and `gap_follow` never reverses. The 2026-10-26 races showed the same pattern: the follow cap was usually 0.00 m/s.

   Following must hold the leader's speed minus a gap, never a dead stop while the leader moves.
2. **See the other car sooner.** `opponent_min_hits: 8` scans (0.2 s) delays recognition. On the real races the car was recognised on only about 5% of logged seconds.
3. **Size passes to the real car.**
   - Re-derive `opponent_radius` from the car's measured 0.30 × 0.58 m outline instead of a circle.
   - Choose `opponent_passing_margin` so a pass fits the 1.30 m (p10) sections.
   - Revisit `opponent_pass_max_steering` for this course's bends.
4. **Prove it in the head-to-head simulator.** On `car1_course_2026_10_26` and `indoor_wide`, with the **faster car starting behind** (`eval_gap_multi.py --cars 2 --speeds ...` or `--car-set 1:max_speed=...`, `--swap-starts`):
   - count overtakes, contacts and stalls;
   - accept only with 0 contacts and 0 stalls;
   - add the finish-gap-change metric to `head_to_head` first, since start order decides "wins" today (Task 14 minor).
5. **Then the car:** wheels off, then a slow two-car floor test, recorded by bringup's `auto_record`.

**Evidence base:**
- The 19:51, 19:59 and 20:07 races' decision logs. Future races get bags through bringup's `auto_record`.
- `gap_follow_launch.py opponent_awareness:=true` in the multi-car simulator (`docs/gap-follow.md#several-cars-at-once`).

**Evidence base:**
- The 19:51, 19:59 and 20:07 races' decision logs. They have no bags; future races get bags through B5.
- `gap_follow_launch.py opponent_awareness:=true` in the multi-car simulator (`docs/gap-follow.md#several-cars-at-once`).

## B9. Two controlled cars in `racerbot_sim`, and racing team A (priority 2 after Part A)

**What exists:** `tools/f1tenth_sim/eval_gap_multi.py` (Part A Task 14) races two or more of **our** `gap_follow` controllers. It has no ROS: each car is a `gap_mirror` copy of our node, with per-car `--car-set` settings and a head-to-head score. It is fast and good for tuning, but it cannot run the real nodes or launch files, and it cannot run team A's code.

**What `racerbot_sim` has today:** several cars, but only the first is driven by ROS. The others are scripted `CenterlineFollower` opponents (`sim_bridge.py`, `OpponentPlan`).

**Build:**

1. **A second ROS-controlled car in `racerbot_sim`.** `gym_bridge_node` gets a `controlled_cars` parameter (default 1, today's behaviour). Each car beyond the first gets its own namespace, `/car2/...`:
   - `scan`, `odom`, the TF frames `car2/odom` → `car2/base_link` → `car2/laser`, and a `drive` subscription;
   - a forged-deadman `/car2/joy` from `sim_joy_node`, so each car's own deadman gate is exercised.

   The existing hardware interlock covers every namespace, so the simulator refuses to start beside real drivers.

   Tests: pure-function tests on namespacing and topic maps in `sim_bridge.py`. The interlock refuses for each car. Car 2's commands move only car 2.
2. **A two-car launch,** `sim_race_launch.py`: one bringup for the simulator, then two control layers, each remapped into its namespace.
   - Car 1 is our `gap_follow_launch.py`.
   - Car 2 is chosen by an argument: `ours` (a second `gap_follow` with its own params file, for ours-vs-ours) or `team_a` (team A's node).
   - Recording uses the normal run recorder, with both namespaces.
3. **Team A's real node on car 2.**
   - Build `src/racerbot_a/src/disparity_extender` (and `ftg`) with `colcon build --packages-select disparity_extender ftg`. Remember car 1's local `COLCON_IGNORE` on `racerbot_a`'s `pure_pursuit`, whose package name clashes with ours.
   - Run it with its sim settings (`enable_deadman:=false`, safety node as its README says), remapped to `/car2/scan` and `/car2/drive`. Their deadman lives in their own `safety_node`.
   - Score with a shared race referee (laps, gap, overtakes, contacts) computed from the simulator's ground truth, never from either car's own claims.
4. **Optional, for fast tuning:** a Python copy of `disparity_extender` (about 250 lines of C++) as a controller type in `eval_gap_multi.py`.

   Before any result is trusted, a **parity test** must show it gives the same command as the compiled node on recorded scans, to 1e-6.

**Done when:**
- `ros2 launch racerbot_sim sim_race_launch.py opponent:=team_a track:=car1_course_2026_10_26` races our `gap_follow` against team A's `disparity_extender` on car 1's course;
- the referee prints the result;
- the full-ROS result for ours-vs-ours agrees in ranking with `eval_gap_multi.py` on the same settings.

**Keep in mind:** a simulator win is evidence, not a verdict. The real races, now recorded by bringup's `auto_record`, are the check.

## B10. B-1 follow-ups (added 2026-10-29)

Found while building B-1; none blocks B-2/3. Details in the B-1 plan's "Outcome and lessons".
- **Floor test (R9):** two-car wheels-off check, a slow two-car floor run with `max_speed:=1.0` recorded by `auto_record`, then the race. Opponent awareness is now **on by default for racing** and has never run on the floor.
- **Standing-start acquisition:** two cars side by side before either track is confirmed (8 scans) touched 0.875 s in.
- **Passing is rare with the follow memory on.** Revisit with floor data.
- **Track confirmation is lost when a track is deleted and re-created** (deferred: inheriting it by position could confirm a wall).
- **R1's minor findings** in `eval_gap_multi.py`.

## Coverage of every request (2026-10-26/27)

| Request | Where |
|---|---|
| Where yesterday's maps are; why SLAM failed; which IMU parts are wired | Answered in chat; the evidence is in the spec |
| Fix SLAM | Part A Tasks 1-5 (done) |
| SLAM recorded well enough to recreate and analyse in depth | B1, plus the recording contract for `auto_record` |
| IMU in `map_course` with on/off flags | `gyro_odom` exists; B2 adds `imu_lap_turns` and `imu_all` |
| IMU in `gap_follow` with on/off flags (one per feature plus all), falling back to today | Part A: steering damping and adaptive grip. B4: the rest. The `imu_all` shortcut is in B2/B4 |
| Allow some slip using IMU feedback | Part A Task 15, adaptive grip (built, ships off: no simulator win yet) |
| Simulated IMU, and bag replay for IMU features | B3 (sim IMU). `eval_gap` already feeds the gym's true yaw rate (Part A) |
| Faster, better, race-winning `gap_follow`; aggressive defaults | Part A Tasks 6-11 and 17 (done) |
| Two cars in the simulator to race | Fast simulator: Part A Task 14 (done). Full-ROS `racerbot_sim`: **B9** |
| Race team A's `gap_follow` in the simulator | **B9** (their real node; optional Python copy with parity test) |
| Improve the simulator | Grip re-checked on car 1 (Task 13); car 1's course added (B7). Two-car ROS simulator in B9 |
| Overtaking / passing (the course has room) | **B6** |
| Clean map of the course, used for simulator testing | Done (`~/.ros/racerbot_auto/20261026-194514-clean/`, `car1_course_2026_10_26`) |
| Bringup logger | Merged on `main` by another session; B1 adds the SLAM pieces |
| VESC hardware tuning | Team note (above), not a build item |

## Note for the team, after the coding: VESC speed-loop tuning

**Not a build item.** The team decided on 2026-10-27 to do this hardware tuning themselves, after the coding is done. It is kept here so the evidence isn't lost.

**What was measured:** while speeding up on 2026-10-26, the car achieved a median of only 0.3-0.7 m/s² of the roughly 2.8 m/s² it was commanded. The 90th percentile was 1.5-2.5 m/s², yet it reached 4-5 m/s² at times, so the motor can deliver it (runs `20261026-194514`, `-194913`).

The VESC's speed controller lags the command. It is likely the single biggest speed lever measured, and it is tuned in VESC Tool, a hardware calibration, not `gap_follow` code.

**When the team tunes it:**
- In VESC Tool, the speed PID, the current and ERPM limits, and any ramp limits are the fields that decide it.
- Change one at a time. Record a run before and after, starting wheels off with LB held; bringup's `auto_record` saves the runs.
- To compare commanded against achieved acceleration, the inline check used on 2026-10-27 was `accel_check.py`, in this session's job folder. It is not in the repo; it is about 30 lines over `tools/run_analysis/runlog.py`.
- Afterwards, re-fit the simulator's speed-loop model with `tools/run_analysis/fit_sim.py`.

## Lower priority

The B4 IMU features beyond Part A's steering damping and adaptive grip come after B6:
- fused speed for standalone `gap_follow`;
- the understeer check;
- traction control;
- the impact stop.

## B7. Car 1's course in the simulator

**Outcome:** the clean map from Part A Task 4's replay becomes a track for `eval_gap.py` and `racerbot_sim`, beside `indoor_oval`, `indoor_tight`, `indoor_wide` and `asb_10000`. `gap_follow` is then tuned on the course it races.

**Files:** `src/racerbot_sim/racerbot_sim/tracks.py`, `tools/f1tenth_sim/eval_gap.py`.

**Tests (shape):** the track loads, its centreline closes, and its width matches the map's measured corridor width (1.5-1.8 m) within one cell.
