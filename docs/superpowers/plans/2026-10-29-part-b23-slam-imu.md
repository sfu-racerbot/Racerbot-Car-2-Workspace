# Part B-2/3 (SLAM observability and IMU, combined) implementation plan

> **Who this is for:** the session that builds the rest of Part B after B-1 (racing), human or agent.
> **Read first:** [the Part B outline](2026-10-26-part-b-skeleton.md) items B1-B4; [the B-1 plan](2026-10-27-part-b1-racing.md), especially its "Outcome and lessons" section; [run-diagnostics.md](../../run-diagnostics.md); [src/imu_bridge/README.md](../../../src/imu_bridge/README.md); [gap-follow.md](../../gap-follow.md) ("Steering damping (gyro)", "Adaptive grip (IMU)", "Several cars at once"); [ros-simulator.md](../../ros-simulator.md); and the [glossary](../../glossary.md) for [SLAM](../../glossary.md#slam) and [odometry](../../glossary.md#odometry--odom).
> **You'll be able to:** get a one-command SLAM health report after every run, replay a run exactly, run the real IMU nodes in the ROS simulator with injectable faults, and add the remaining IMU features to `gap_follow` and `map_course`, each behind its own flag.
> **Time:** two tracks in parallel, a join, then a final review. Expect several sessions.

> **For agentic workers:** REQUIRED SUB-SKILL: Use opencode-delegate:opencode-driven-development (the controller writes briefs, opencode models implement and review), or superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax for tracking.

**This plan replaces** [the B-2 plan](2026-10-27-part-b2-slam.md) and [the B-3 plan](2026-10-27-part-b3-imu.md). Their task text is carried over here, task IDs kept (S1-S7, I1-I7), with changes marked **(changed 2026-10-29)**.

## Why one plan

The two parts were written to run one after the other. B-1 showed they are better run together:
- **They touch disjoint code.** The SLAM track lives in `race_diagnostics` and `tools/run_analysis`. The IMU track lives in `racerbot_sim`, `imu_bridge` and `gap_follow`. B-1 ran two such tracks at once in sibling worktrees with no merge conflicts.
- **They meet in the same files at the end.** S6/S7 (`map_course` flags) and I7 (`imu_all` in `gap_follow_launch.py`) must use the same flag names and refusal rules. One join task does both.
- **The simulated IMU (I1) helps the SLAM track.** With it, `map_course imu:=true gyro_odom:=true` runs in `racerbot_sim`, so S5's `slam_report` and S7's flags can be checked end to end without the car.
- **It also fixes a simulator gap B-1 found.** `racerbot_sim` publishes no `/imu/data`, so `gap_follow`'s steering damping (shipped **on**) is inert there, while the fast simulator feeds it gym's true yaw rate. The two simulators therefore ran different steering laws on "the same settings" (B-1 Task R6b).

**Goal:** never debug SLAM by hand again, and prove every IMU feature, including its faults, closed-loop in the ROS simulator before it reaches the car.

**Architecture:**
- **Track S (SLAM observability):** recording is `race_diagnostics`' `record_run.py`, used by bringup's `auto_record` and by `map_course`. Analysis builds on Part A's ROS-free `tools/run_analysis/slam_metrics.py`, `slam_bag.py` and `slam_replay.py`.
- **Track I (IMU):** a simulated IMU in `racerbot_sim`'s bridge, per controlled car. ROS-free feature logic in `gap_logic.py`, wired through `gap_follow_node._imu_gate()`, and mirrored in `tools/f1tenth_sim/gap_mirror.py`.
- **Join (J1):** the IMU flags in `map_course` and `gap_follow_launch.py`, plus `map_course` running the reports.

**Tech stack:** Python 3.12, ROS 2 Jazzy, `rosbag2_py`, slam_toolbox (apt), numpy and scipy, F1TENTH Gym through `racerbot_sim` and `tools/f1tenth_sim`, pytest.

**Starting point:** `main`, which has B-1 since PR #6 (`7842762`, 2026-10-29) and the interlock test fix after it. Before Task 1, move `src/racerbot_a` to team A's latest `main` (pin-only commit; memory `team-a-submodule-latest`).

## Global Constraints

- **The deadman is untouched.** The LB deadman, its four deny-path tests per node, TTC, emergency clearance and the contact stop do not change. Every new path that can set a speed gets the four deny-path tests with its feature on, **asserting the published command**.
- **Prove deny-path tests with an in-memory break, never a flipped expectation** (changed 2026-10-29). In a scratch script, monkeypatch the node's deadman gate (`GapFollowNode._deadman_status`, the method the scan step actually checks) to report "engaged", run each deny test with `pytest.main`, and require each one to fail. No deadman file is edited. Flipping an assertion only proves the assertion runs (B-1 final review).
- **Opponent awareness is on by default** (B-1, user decision 2026-10-28) for racing launches, off for the mapping launches. Every new `gap_follow` test that uses the shipped YAML therefore runs with awareness on. A test whose subject is "feature off" must say `enable_opponent_awareness:=false` explicitly if awareness matters to it (B-1 Task R2c).
- **IMU failure falls back to today's behaviour.** Any IMU feature with an IMU that isn't trusted (status `waiting`/`fault`, stale, never received, non-finite) behaves exactly as with the feature off, and names the reason on `/drive_intent`, reusing `gap_follow_node._imu_gate()`.
- **Flags:** one per feature, plus `imu_all`, with the same names in `map_course` and `gap_follow_launch.py`. All default off. An explicit `false` beats `imu_all`. Any IMU flag with `imu:=false` refuses to start.
- **Aggressive tuning** (memory `aggressive-race-tuning`): a feature ships **on** only if the simulator protocol shows it faster with zero contacts and no added stalls. Otherwise it ships off and is noted for a floor trial.
- **Simulator protocol for any tuning claim:** `eval_gap.py` / `eval_gap_multi.py` on the indoor tracks **and** `car1_course_2026_10_26`; both directions (`--reverse`); at least 2 seeds; report mean lap, contacts, stops and `steering_reversals_per_s`, and for races wins, finish-gap change (`time_gained_s`), overtakes, contacts and stalls. **Check that each scenario exercises the behaviour under test before reading its result** (changed 2026-10-29). B-1's first pass-margin sweep was meaningless because the "slower" leader (2.5 m/s cap) was barely slower than a 5.0 m/s-capped car's real mean on car 1's course (~2.4 m/s), so the follower never caught it. Report the closest approach, recognised fraction or similar, and size the speeds to the course.
- **Judge contacts with the simulators' own chassis checks,** never gym's collision flag (CLAUDE.md). **The gym's tyres can't slide:** their force is linear in slip angle, so grip and understeer thresholds come from recorded runs, and the simulator checks wiring and fall-back only.
- **`racerbot_sim` must never run beside the real drivers.** Its interlock starts blocked for a 3 s discovery hold, then needs the graph clean for another 3 s (`interlock_clear_confirm_sec`, 2026-10-29), so simulated output starts about 6 s after launch. It must cover every new node, including the simulated IMU and its namespaces. Any ROS test that checks for *no* output must watch for longer than that 6 s, or it passes without testing anything. Compute the watch from the nodes' defaults, as `test_controlled_cars.py` does with `INTERLOCK_OPENS_SEC` and `LEAK_WINDOW_SEC`; I1's new IMU publisher joins that list. **`racerbot_sim` ROS tests use `/test_only/...` drive chains and the fail-fast isolation guard in `src/racerbot_sim/test/conftest.py`** (B-1 final fix wave). New ROS-level tests follow that pattern.
- **Analysis never costs the car CPU while it drives or maps.** Analysis runs after the run. The only additions during a run are `tegrastats` and a 1 Hz `/proc` sampler.
- **Recorders only read.** They subscribe, read `/proc`, and call parameter services. They never publish a drive topic, and never write inside an existing run folder except `analysis/`.
- **Replays run isolated:** `slam_replay.py` keeps its isolation (domain 77, localhost) and its refusal beside `vesc_driver_node`, `urg_node` or `joy`.
- **Test quality:** Hard rule 1 (every new or changed test is seen to fail against a deliberate break), Hard rule 2 (never loosen a failing test without stating its reasoning, and flag any change to an existing test to the user), and an oracle for every expected value (`TEST_QUALITY_STANDARDS.md` A3).
- **Known flaky tests on the x86 dev box** (don't edit them; re-run alone and report): `test_gap_follow_corners_node.py::test_corner_setup_is_mirror_symmetric_for_a_right_turn` (fails about 3 runs in 5 on clean `main`); `test_raceline_optimizer.py::test_the_trust_region_bounds_a_single_pass` (1.6e-6 overshoot, x86 only); `test_auto_record_node.py::test_records_while_an_algorithm_runs_and_closes_the_bag_when_it_goes` (under load).
- **Docs** follow the `novice-docs` skill, and `check_docs.py` must add no new findings.
- **Commits** end with the attribution lines the session gives. Never push to `main`.

## How to run it (learned in B-1, 2026-10-28)

- **One controller per plan.** Before starting, check with `ListAgents` (or `ps -eo args | grep claude`) that no other session is already working this plan. In B-1, a resumed session and a forked copy of it both believed they owned the plan, and broke each other's tests for hours.
- **Two worktrees, two ROS domains.** Track S and Track I run in sibling `oc-task` worktrees (e.g. `.oc-worktrees/oc/part-b23-slam` and `.oc-worktrees/oc/part-b23-imu`), each with its own submodule checkout, a `.sim` symlink to the main checkout's gym, and its own `colcon build --packages-skip range_lib`. Use `RB_WS=<worktree> rb <command>` on the dev box. Give each track its own `ROS_DOMAIN_ID` (79 and 78), so their ROS tests never see each other's topics. Merge both into the integration branch before J1.
- **Duplicate package names on the dev box.** Team A's `safety_node` clashes with team B's. Keep the untracked `COLCON_IGNORE` in `src/racerbot_b/src/safety_node`. After moving team A's pin, `rm -rf build/safety_node install/safety_node` before rebuilding, or colcon silently reuses the other team's CMake cache (B-1 Task R7).
- **The machine is shared.** `--workers 2` on every `eval_gap*.py`, one suite or sweep at a time, and hold heavy sweeps while a real-time `ros2 launch racerbot_sim` race runs (`while pgrep -f 'ros2 launch racerbot_sim' >/dev/null; do sleep 60; done`).
- **`pytest --basetemp` needs an existing parent directory**, or every `tmp_path` test errors with `FileNotFoundError`.
- **Models.** `openai/gpt-6.1-sol` for driving code and final reviews, `openai/gpt-6.1-terra` for the rest. On a usage limit (exit 5), continue on tokenzap `sol-6`, or on a Claude Haiku subagent at max effort (memory `opencode-models`). Keep the ledger at `.oc-runs/<plan-slug>/progress.md`, so work survives limits and context resets.
- **Briefs.** Every brief must state the scenario's purpose and a check that it occurs (see the simulator protocol above). Expect good models to stop with NEEDS_CONTEXT on contradictions or on an existing test that must change. Answer with a ruling, and flag every existing-test change to the user (Hard rule 2).
- **At the end, give the user a paste-ready prompt for Claude Code on the car** (memory `car-handoff-prompt`): pull, rebuild with `--parallel-workers 1`, run the suites, fix Jetson-only failures, then the wheels-off → floor order for anything that drives.

## Review Focus

1. **A node that starts minutes after the recorder** (SLAM started after `gap_follow` in a mapping workflow) must still have its parameters snapshotted.
2. **A run without `/pose`** (no SLAM) must give a report that says so plainly, not a stack trace.
3. **`tegrastats` missing or failing:** the run still records, with a logged warning and no crash.
4. **An exact replay of a run whose `/scan` was dropped by the recorder itself:** detect the missing stamps and refuse, instead of silently replaying a different run.
5. **An IMU flag with `imu:=false`, or `imu_all` with the IMU stack absent:** refuse with a clear message, in both launches.
6. **The IMU drops out mid-corner** with a feature on: the command equals the feature-off command on that tick, with no jump.
7. **The simulated IMU beside a real IMU stack:** refuse. **Car 2's simulated IMU must reach only car 2** (`/car2/imu/...`).
8. **The impact stop** latches until LB is released and pressed again, never clearing by itself.
9. **Traction control at standstill or reversing** must not hold acceleration forever.

---

## Track S: SLAM observability

### Task S1: Snapshot nodes that start late

**Files:** `src/race_diagnostics/race_diagnostics/snapshot_params.py` and its tests.

**Behaviour:**
- Keep watching the node list for the run's whole lifetime, polling every 2 s.
- Snapshot each node the first time it appears. Re-snapshot a node that restarts, as `<name>.<n>.yaml`.
- Live changes are already in the bag through `/parameter_events`.
- **(changed 2026-10-29)** Namespaced nodes (`/car2/gap_follow_node` in `sim_race_launch.py` races) are snapshotted under their full name, e.g. `car2__gap_follow_node.yaml`. Two nodes with the same name in different namespaces must never overwrite each other.

**Tests:** the pure "which nodes are new" logic over successive node-list samples: a new node, a restart, a vanished node, and two same-named nodes in different namespaces.

### Task S2: System logs

**Files:** `src/race_diagnostics/launch/record_run.py`; a new ROS-free `src/race_diagnostics/race_diagnostics/system_log.py` with a small entry point.

**What it logs:**
- `<run>/system/tegrastats.log` from `tegrastats --interval 1000`. It runs without sudo on car 1 (checked 2026-10-26). On the dev box there is no `tegrastats`, which exercises Review Focus 3.
- `<run>/system/proc.jsonl`: 1 Hz CPU %, RSS and thread count per ROS node process, found by `__node:=` in `/proc/<pid>/cmdline`.

**Failure handling:** each logger failing to start is logged and the run continues (Review Focus 3).

**Tests:** the `/proc` parse on a fake `/proc` tree (CPU % from two `stat` samples, with a closed-form expectation); a missing `tegrastats` binary gives a warning, not an exception.

### Task S3: Package versions

**Files:** `src/race_diagnostics/race_diagnostics/run_metadata.py`.

**What it adds:** `metadata/packages.txt` listing `dpkg-query -W 'ros-jazzy-*'` versions, slam_toolbox first. Git state can't cover apt packages. **(changed 2026-10-29)** Also record each submodule's commit, including `src/racerbot_a` (team A's pin moves often now) and whether `src/racerbot_b` is initialised.

**Tests:** parsing of the `dpkg-query` output, its absence handled, and the submodule list on a fake `.gitmodules`.

### Task S4: Exact replay

**Files:** `tools/run_analysis/slam_replay.py`, plus a small rclpy feeder script beside it.

**`--exact` mode:**
- Runs `sync_slam_toolbox_node`.
- Feeds only the `/scan` messages whose stamps equal the recorded `/pose` stamps. That's the processed-scan record: 315 `/pose` against 314 SLAM updates on run `20261026-194913`.

**Tests:**
- The stamp-matching function, including a missing scan, which refuses (Review Focus 4).
- End to end: `--exact` on `20261026-194913` must reproduce the recorded `/pose` sequence to 1 mm and 0.1°. If it can't, report how close it gets and why. Don't loosen the bar silently.

**Also fold in Part A's deferred minor:** when slam_toolbox's map saver fails, write `map.pgm` and `map.yaml` from the replay bag's last `/map` (as was done for the cleaned 19:45 map) and set `map_saved: from_bag`.

**Do this last in Track S.** It is the riskiest task, and S5 doesn't need it.

### Task S5: `slam_report.py`

**Files:** `tools/run_analysis/slam_report.py`, tests.

**Report contents:**
- **Setup:** the odometry frame SLAM used (from the parameter snapshot), speeds and distance.
- **Heading drift** of `/odom`, `/odom/imu` and SLAM against each other: final value, maximum, and per-second RMS. This is the 2026-10-26 analysis made permanent.
- **Corrections and loop closures,** from `slam_metrics`.
- **Laps:** per-lap closure distance, heading error, corrections and quality verdict, from `race_diagnostics.run_events` log parsing.
- **Map consistency:** the share of scan endpoints within one cell of an occupied cell of the final map, per lap. Test it with a synthetic map and scans at a known offset (closed form).
- **Scan drops** ("queue is full") on a timeline against slam_toolbox's CPU % from S2.
- **Wheel distance against scan-matched distance.**

**Output:** `slam_report.txt`, `summary.json` and PNGs. `--compare runA runB` gives side-by-side numbers and overlays.

**Tests:** every metric against synthetic runs with closed-form answers, plus Review Focus 2. **(changed 2026-10-29)** Once I1 lands, add one end-to-end check on a real `racerbot_sim` mapping run (`map_course` in the simulator with `imu:=true gyro_odom:=true`): the report names `odom_imu` as SLAM's frame and gives finite numbers.

---

## Track I: IMU

### Task I1: A simulated IMU in `racerbot_sim`, per controlled car

**Files:** `src/racerbot_sim/racerbot_sim/{sim_bridge.py,gym_bridge_node.py,hardware_guard.py}`, `src/racerbot_sim/launch/{sim_bringup_launch.py,sim_race_launch.py}`, `src/imu_bridge/launch/vehicle_state_launch.py` (`hardware:=false` skips the BNO bridge and `imu_bridge_node`), `src/racerbot_launch/scripts/map_course` (stop forcing `imu:=false` in the simulator), tests.

**What it publishes,** in `imu_bridge`'s format, from the gym's true state of each controlled car: `/imu/bno/data`, `/imu/vesc/data`, `/imu/data` and latched `/imu/status` for car 1, and the same under `/car2/` for car 2 **(changed 2026-10-29: B-1 added a second controlled car)**. `sim_race_launch.py` already points car 2's `gap_follow` at `/car2/imu/data` and `/car2/imu/status`.
- **Gyro z:** `state[5]`.
- **Accelerometer:** forward from the change in speed, sideways from speed × yaw rate, and gravity on z.
- **Noise:** measured from a stationary stretch of a real bag (cite the run).

**Faults:** parameter `imu_fault` (per car: `imu_fault`, `car2_imu_fault`) is one of `none`, `stale`, `status_fault`, `bias`, `scale` or `zero`, and can be set mid-run.

**Tests:**
- On a constant circle, the gyro equals v/R and the sideways acceleration equals v²/R.
- On a ramp, the forward acceleration matches.
- The noise spread matches its setting.
- Each fault gives exactly its output.
- `/imu/status` parses in `imu_health` and `neural_racer`.
- The interlock (with its discovery hold) refuses beside `imu_bridge_node` (Review Focus 7), and car 2's IMU publishes only under `/car2/`.
- Reproduction: with `odom_yaw_rate_scale` about 1.1, `map_course` without `gyro_odom` fails to close laps, and with it succeeds.
- **(changed 2026-10-29) The fidelity check B-1 needed:** with the simulated IMU, steering damping is live in `racerbot_sim` (`/drive_intent` no longer says "no imu"). Rerun B-1's full-ROS race (`sim_race_launch.py opponent:=ours track:=car1_course_2026_10_26 laps:=3`, both start orders) and `eval_gap_multi.py` on the same settings, and report whether their rankings now agree.

### Task I2: Adaptive grip, made to earn its place

**Background (Part A Task 15):** even with lag compensation, the simulator's grip ratio sits around 0.91, because the gym's linear tyres understeer. With relaxed thresholds (hold 0.90, cut 0.85, ceiling 3.5), adaptive grip was slower on car 1's course than plain 2.5 (12.30 s against 12.04 s). A fixed 3.0 limit at a 5.0 m/s top speed cost a wall contact.

**Steps:** replay car 1's recorded runs (`20261026-194514`, `-194913`) through `gap_logic.adaptive_grip_step` offline, to see what limit it would have chosen on the real floor (the ROS-free functions take the recorded yaw rate, speed and commanded steering). Use I1's simulated IMU to check the fall-backs closed-loop. Decide ship-on or ship-off by the aggressive policy, and record why.

### Task I3: Fused speed in `gap_follow_launch.py`

`fused_speed:=true` points `gap_follow`'s `odom_topic` at `/odom/fused` and starts the IMU stack, mirroring `map_course`'s existing flag (`odom_overrides`). Tests follow `test_map_course_imu.py`'s pattern.

### Task I4: The understeer check (`imu_understeer`)

Slow down when `/steering/measured` shows the car turning much less than commanded. The input is NaN below 0.3 m/s, which means no action. Thresholds come from the recorded runs' grip bands (`analysis/report.md` in `20261026-194514`).

### Task I5: Traction control (`imu_traction`)

Hold the acceleration ramp while the wheel speed (`/odom`) outruns the fused speed (`/odom/fused`) by more than a threshold. Release after a timeout, so the ramp is never held forever (Review Focus 9).

### Task I6: The impact stop (`imu_impact_stop`)

An accelerometer spike above a threshold, taken from recorded bumps (find one in the bags, or record one wheels-off), commands zero speed and latches until LB is released and pressed again (Review Focus 8). This is a safety addition, so its deny paths are the main tests.

**Each feature task I3-I6:**
- ROS-free logic in `gap_logic.py` with closed-form tests.
- Node wiring through `_imu_gate`.
- The four IMU failure modes give exactly the feature-off command (Review Focus 6).
- The four deadman deny paths with the flag on, **and with the shipped YAML's awareness on**, proven by the in-memory break.
- Mirror parity with the flag off and on, each controller keeping its own state (no per-tick state copying; B-1 Task R2).
- The simulator protocol, and docs.

---

## Join

### Task J1: The IMU flags in both launches, and `map_course` runs the reports

Starts when Track S's S5 and Track I's I1-I3 are merged into the integration branch.

**Files:** `src/racerbot_launch/scripts/map_course`, `src/racerbot_launch/launch/map_course_launch.py`, `src/gap_follow/launch/gap_follow_launch.py`, `src/racerbot_launch/test/test_map_course_imu.py`, the gap_follow launch tests, `docs/launchstart.md`, `docs/gap-follow.md`.

**From S6:** after the launch exits, the `map_course` wrapper runs `summarize_run`, `imu_report` and `slam_report` on the run folder and prints the headline lines. `analysis:=false` skips them; the wrapper strips that argument before the launch.
- **Mapping speed.** `gap_follow.yaml` ships `max_speed: 5.0`, and `map_course` maps on the same YAML. The gyro SLAM profile (`slam_gyro.yaml`) was validated by replay only up to about 3.3 m/s. Make `map_course`'s `mapping_max_speed` default **3.5**, with a comment citing that. Lift it only after a recorded gyro run at a higher speed gives a clean `slam_report`. The test checks the default and that an explicit value overrides it.
- `map_course` already takes `opponent_awareness` (default `false`, B-1 Task R2d). Keep it.

**From S7 (B2):**
- `imu_lap_turns:=true` sets `auto_map_race_node` `odom_frame: odom_imu`, so the lap counter's turning comes from the gyro. It refuses without `gyro_odom:=true`, and the lap counter's frame must equal `vehicle_state.yaml`'s `imu_odom_frame`.
- `imu_all:=true` turns on `gyro_odom`, `imu_lap_turns` and `fused_speed`; an explicit `false` overrides it.
- Everything refuses with `imu:=false` (Review Focus 5).

**From I7:** `imu_all:=true` in `gap_follow_launch.py` turns on every IMU flag that exists there by then (`fused_speed`, and any of `imu_understeer`, `imu_traction`, `imu_impact_stop`, `steering_damping`, `adaptive_grip` that have shipped), and an explicit `false` overrides it. It refuses with the IMU stack absent (Review Focus 5).

**Tests:** in the existing `_wired_context` style, with `pytest.raises(RuntimeError, match=...)` for every refusal; one table-driven test that `map_course` and `gap_follow_launch.py` accept the same flag names and the same `imu_all` override rule.

**Then:** the remaining I4-I6 features join `imu_all` as each one lands.

---

## Not in this plan (B-1 follow-ups, outline item B10)

These came out of B-1 and are tracked in the outline's B10. Do them when the race schedule needs them, not as part of B-2/3:
- **The floor test (B-1 Task R9):** two-car wheels-off check, a slow two-car floor run with `max_speed:=1.0`, then the race. Awareness is now on by default, and it has never run on the floor.
- **Standing-start acquisition:** two cars starting side by side, before either track is confirmed (8 scans), touched 0.875 s in, in a B-1 final-fix check.
- **Passing is rare with the follow memory on** (1 overtake against 8 without it in R2b's matrix). The memory was chosen to stop stalls; revisit passing with floor data.
- **R1's deferred minors** in `eval_gap_multi.py` (silent defaults in `aggregate_head_to_head`, comments on `STALL_*`, NaN from mixed infinities with 3+ cars).
- **Confirmation lost when a track is deleted and re-created** (deferred in R2: inheriting confirmation by position could confirm a wall).
