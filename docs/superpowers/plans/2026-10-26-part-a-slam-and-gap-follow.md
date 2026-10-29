# Part A (SLAM and follow-the-gap) implementation plan

> **Who this is for:** whoever builds Part A, human or agent, task by task.
> **Read first:** [the spec](../specs/2026-10-26-slam-and-gap-follow-plan.md), and [glossary.md](../../glossary.md) for [odometry](../../glossary.md#odometry--odom), [topic](../../glossary.md#topic), [publish](../../glossary.md#publish--subscribe), [package](../../glossary.md#package) and [scan](../../glossary.md#scan).
> **You'll be able to:** build each Part A item with its exact files, interfaces and tests, and know when to stop and report instead.
> **Time:** one task at a time. Tasks 3-4 need the car parked with bringup stopped.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** make `map_course` able to save a map on car 1, and make `gap_follow` smoother and faster, with every change proven before the car drives on it.

**Architecture:**
- **SLAM tuning:** a ROS-free metrics module and an offline replay tool let SLAM settings be tuned on today's recorded runs. The winning settings ship as a profile that `map_course` loads only with `gyro_odom:=true`.
- **Lap closing:** `LapRecorder` gains an opt-in "close at the closest approach" mode, which the shipped config turns on.
- **`gap_follow`:** it gets a higher grip limit, a diagnosed and fixed steering weave (gain schedule, then optional gyro damping behind a flag), and two measured reports: corner setup and race pace.

**Tech Stack:**
- Python 3.12, ROS 2 Jazzy (`rclpy`, `launch`, `rosbag2_py`), slam_toolbox (apt), numpy, scipy 1.11.
- The F1TENTH Gym through `tools/f1tenth_sim`, and pytest.

**Spec:** [docs/superpowers/specs/2026-10-26-slam-and-gap-follow-plan.md](../specs/2026-10-26-slam-and-gap-follow-plan.md), Part A (items A1-A8). Read it first.

## Global Constraints

- **The LB deadman is never touched.** `enable_deadman` stays `true` in every shipped config. Every node that can move the car keeps its four deny-path tests (LB held, released, `/joy` never received, `/joy` stale), asserting on the published command.
- **Node tests publish on `/test_only/drive`,** with the remap placed **after** any `--params-file`.
- **ROS tests run sourced and isolated:** `source /opt/ros/jazzy/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Never use `--continue-on-collection-errors`; compare the collected count with `grep -c '^def test'`.
- **Hard rule 1:** every new or changed test is seen to fail against a broken version of its code, then the code is reverted and the test passes. Record the break used in the commit message.
- **Hard rule 2:** never loosen, skip or delete a failing test. If an existing test conflicts with a spec change, stop and report it to the user, with what it asserted, the new behaviour, where that is written down, and why the old behaviour was wrong.
- **Expected values** trace to a closed form, an invariant, a cited spec or a named recorded run, and the test says which (`TEST_QUALITY_STANDARDS.md` A3).
- **`gap_follow` pipeline changes** land in `tools/f1tenth_sim/gap_mirror.py` in the same commit. `src/gap_follow/test/test_gap_mirror_parity.py` must pass.
- **New behaviour defaults:** off in the code's declared default. Only the shipped YAML switches on what the spec makes the new default (A3's closest-approach closure).
- **Replays and car work:**
  - The replay tool refuses to run while `vesc_driver_node`, `urg_node` or `joy_node` processes exist. **Bringup must be stopped (car parked) for Tasks 3 and 4.**
  - Recorded runs live in `~/.ros/racerbot_runs/` and are read-only inputs. Never write into an existing run folder.
- **Docs:** any Markdown touched under `docs/` or a package README follows the `novice-docs` skill, and `python3 .claude/skills/novice-docs/scripts/check_docs.py <files>` reports no findings.
- **Commits:** end every commit message with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Never push to `main`.

## Review Focus

1. **Replay beside a live car:** a driver started by systemd or under a different process name, for example matched by its executable path rather than `__node:=`, must still trip the interlock. Test in Task 2.
2. **A run without `/odom/imu`** (gyro-off and older runs) replayed with `--odom odom_imu` must refuse with a clear message before starting anything, not hang waiting for a topic. Test in Task 2.
3. **A closest approach whose heading is still over 30°** must not close the lap. The lap continues, as the old gate did. Test in Task 5.
4. **The car stopping (LB released) inside the closure circle** must not close the lap while stationary, and must close it once the car moves away past the minimum. Test in Task 5.
5. **Steering damping with a bad IMU** (status `fault` or `waiting`, data stale, never received, or a NaN yaw rate) must add exactly zero damping, never NaN steering. Tests in Task 9.

---

## File map

| File | Responsibility | Task |
|---|---|---|
| `tools/run_analysis/slam_metrics.py` (new) | ROS-free: SLAM corrections measured at the car, loop-closure times from logs, acceptance summary | 1 |
| `tools/run_analysis/slam_bag.py` (new) | Sourced: read a run bag into the arrays `slam_metrics` takes, cached as `analysis/slam.npz` | 1 |
| `tools/run_analysis/slam_replay.py` (new) | Replay a run through slam_toolbox: interlock, isolated environment, parameter merge, process orchestration | 2 |
| `tools/run_analysis/odom_tf_relay.py` (new) | rclpy: turn an `Odometry` topic into its single transform | 2 |
| `tools/run_analysis/tests/test_slam_metrics.py`, `test_slam_replay.py` (new) | Unsourced tests | 1, 2 |
| `src/racerbot_launch/config/slam_gyro.yaml` (new) | The good-odometry SLAM profile | 4 |
| `src/racerbot_launch/launch/slam_launch.py` | New `slam_profile_file` argument, layered after `slam_tracking_file` | 4 |
| `src/racerbot_launch/launch/map_course_launch.py` | `gyro_odom:=true` also sets `slam_profile_file` | 4 |
| `src/racerbot_launch/test/test_map_course_imu.py` | Wiring and profile-invariant tests | 4 |
| `src/pure_pursuit/pure_pursuit/auto_map_race_node.py` | `LapRecorder(close_at_closest_approach=...)` and the node parameter | 5 |
| `src/pure_pursuit/config/auto_map_race.yaml` | `close_at_closest_approach: true` | 5 |
| `src/pure_pursuit/test/test_auto_map_race.py` | Closest-approach tests | 5 |
| `src/gap_follow/config/gap_follow.yaml`, `src/gap_follow/README.md`, `docs/gap-follow.md` | `max_lateral_accel` 2.5, gain schedule, damping parameters | 6, 8, 9 |
| `tools/run_analysis/weave.py` (new), `tools/run_analysis/tests/test_weave.py` (new) | Weave diagnosis and the loop model | 7 |
| `tools/f1tenth_sim/gap_mirror.py`, `tools/f1tenth_sim/eval_gap.py` | Telemetry `target_angle`; a `steering_reversals` metric; damping fed by the sim's true yaw rate | 7, 9 |
| `src/gap_follow/gap_follow/gap_logic.py`, `gap_follow_node.py`, `launch/gap_follow_launch.py` | Gyro steering damping behind `enable_steering_damping` | 9 |
| `docs/superpowers/specs/2026-10-26-slam-and-gap-follow-plan.md` | Results sections for A1, A5, A6, A7 | 4, 8, 10, 11 |

---

### Task 1: SLAM metrics, independent of SLAM's own claims

**Files:**
- Create: `tools/run_analysis/slam_metrics.py`, `tools/run_analysis/slam_bag.py`, `tools/run_analysis/tests/test_slam_metrics.py`

**Interfaces:**
- Produces:
  - `slam_metrics.correction_events(map_odom: np.ndarray, odom: np.ndarray) -> np.ndarray`
    - `map_odom` rows are `(t, x, y, yaw)` of `map → <odom frame>`; `odom` rows are `(t, x, y, yaw)` of `<odom frame> → base_link`.
    - It returns rows `(t, move_m, rotate_rad)`: one per change of `map_odom`, measured as the displacement of the **car's** pose (the old and new correction applied to the car's odometry pose at `t`).
  - `slam_metrics.loop_closure_times(log_text: str) -> np.ndarray`: epoch seconds of every Ceres solve line (`W<yyyymmdd> <hh:mm:ss.ffffff> ... num_threads`), read as local time.
  - `slam_metrics.summarize(events: np.ndarray, closures: np.ndarray, window_s: float = 1.5, false_jump_m: float = 1.5) -> dict`, with keys:
    - `updates`, `loop_closures`;
    - `max_move_m`, `max_rotate_deg`;
    - `jumps_over_0_3m`, `jumps_over_false_jump_m`;
    - `jumps_at_loop_closures`, the count of over-0.3 m jumps within `window_s` of a closure.
  - `slam_bag.load(run_dir: Path, refresh: bool = False) -> dict`, with keys `map_odom`, `odom`, `odom_imu`, `slam_pose`, `pose`, `map` (the last `/map`: `(resolution, origin_x, origin_y, grid int8 2-D)`) and `odom_frame` (the child frame found under `map`). It is cached at `<run>/analysis/slam.npz`; a replay folder writes its own cache.

- [ ] **Step 1: Write the failing tests** in `tools/run_analysis/tests/test_slam_metrics.py`. Use the `sys.path.insert(0, .../tools)` import pattern from `test_actuation.py`.
  - `test_pure_translation_jump_is_measured_at_the_car`
    - oracle: closed form. The correction jumps by `(2.4, 0, 0)` once and the car is anywhere.
    - `events[:, 1] == pytest.approx([2.4], abs=1e-9)  # m`
  - `test_rotation_about_the_map_origin_moves_the_car_by_the_chord`
    - oracle: closed form. The correction rotates by θ = 0.1 rad about the origin, with the car at radius r = 5 m.
    - The move must be `2*r*sin(θ/2)` to `abs=1e-9` m, and the rotation 0.1 rad to `abs=1e-12`.
  - `test_no_change_no_events`: a constant correction gives `events.shape == (0, 3)`.
  - `test_empty_and_single_sample_inputs`: zero or one row gives `(0, 3)` and doesn't raise.
  - `test_nan_rows_are_skipped_not_propagated`: a NaN row between two equal rows gives no event and no NaN in the output.
  - `test_yaw_wraps_at_pi`: a correction going from yaw 3.13 to -3.13 is a rotation of 0.0232 rad (`abs=1e-9`), not 6.26.
  - `test_loop_closure_times_parse_the_ceres_lines`:
    - Two real lines copied from `~/.ros/racerbot_runs/map_course-20261026-194912.log`, plus an unrelated line.
    - Two times result, matching `datetime(...).timestamp()` of the stamps to `abs=1e-6` s.
    - Empty text gives an empty array.
  - `test_summarize_counts_only_jumps_near_closures`
    - Synthetic: three jumps (2.4 m at t=10, 0.4 m at t=20, 0.05 m at t=30) and closures at t=10.5 and 30.2.
    - Expect `jumps_over_0_3m == 2`, `jumps_at_loop_closures == 1`, `jumps_over_false_jump_m == 1`, and `max_move_m` 2.4.

- [ ] **Step 2: Run them; they must fail.** `python3 -m pytest tools/run_analysis/tests/test_slam_metrics.py -v` gives FAIL with `ModuleNotFoundError: slam_metrics`.

- [ ] **Step 3: Implement `slam_metrics.py` (numpy only).**
  - Interpolate the car pose from `odom` at each change time, using unwrapped yaw.
  - Compose `old∘car` and `new∘car` to measure each change.
  - Generalise `$CLAUDE_JOB_DIR/tmp/jumps.py`'s approach. That file is a reference only; do not import it.

- [ ] **Step 4: Run the tests; they must pass.** Then break `correction_events` to return the correction's own translation instead of the car's displacement. `test_rotation_about_the_map_origin...` must fail. Revert.

- [ ] **Step 5: Implement `slam_bag.load`** with `rosbag2_py` (storage `mcap`).
  - Read `/tf`, keeping `map → {odom, odom_imu}` and `{odom, odom_imu} → base_link`, plus `/odom`, `/odom/imu`, `/slam_pose`, `/pose` and the last `/map`.
  - Generalise `$CLAUDE_JOB_DIR/tmp/slam_extract.py`, which keeps only the **last** map.
  - Verify, sourced: `python3 -c "import sys; sys.path.insert(0,'tools'); from run_analysis import slam_bag, slam_metrics; from pathlib import Path; d=slam_bag.load(Path('~/.ros/racerbot_runs/20261026-194913').expanduser()); e=slam_metrics.correction_events(d['map_odom'], d['odom_imu']); print(len(e), e[:,1].max())"`
  - Expected: about 300 events and a maximum of about 2.5 m (the recorded measurement: 2.56 m at t=+46.9 s).

- [ ] **Step 6: Commit** the module, the loader and the tests, with the Hard rule 1 break noted.

### Task 2: Offline SLAM replay tool

**Files:**
- Create: `tools/run_analysis/slam_replay.py`, `tools/run_analysis/odom_tf_relay.py`, `tools/run_analysis/tests/test_slam_replay.py`

**Interfaces:**
- Consumes: `slam_bag.load` (Task 1), used to verify outputs.
- Produces:
  - `slam_replay.DRIVER_MARKERS = ('vesc_driver_node', 'urg_node', 'joy_node', '__node:=joy')`
  - `slam_replay.running_drivers(proc_root: Path = Path('/proc')) -> list[str]`: the markers found in any `/proc/<pid>/cmdline`. Unreadable or vanished pids are skipped.
  - `slam_replay.REPLAY_DOMAIN_ID = 77`
  - `slam_replay.child_env(base: Mapping[str, str]) -> dict[str, str]`: a copy of `base` with `ROS_DOMAIN_ID='77'`, `ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'` and `ROS_STATIC_PEERS=''`, whatever `base` had.
  - `slam_replay.ODOM_TOPICS = {'odom': '/odom', 'odom_imu': '/odom/imu'}`
  - `slam_replay.slam_parameters(snapshot: dict, odom_frame: str, profile: dict | None) -> dict`
    - It returns `{'slam_toolbox': {'ros__parameters': {...}}}`, with the layers in this order: snapshot, then profile, then forced keys.
    - Forced: `use_sim_time=True`, `odom_frame`.
    - Dropped: keys starting with `qos_overrides` and `start_type_description_service`.
  - `slam_replay.bag_topics(bag_dir: Path) -> set[str]`, read from `metadata.yaml` (no rosbag2 needed).
  - `slam_replay.plan_replay(run_dir: Path, odom_frame: str, profile_path: Path | None, out_root: Path) -> ReplayPlan`
    - It raises `ReplayRefused` (a `RuntimeError` subclass) when the run has no `bag/`, no `metadata/params/slam_toolbox.yaml`, or no topic for `odom_frame`. The message names what's missing.
    - `ReplayPlan` is a dataclass with `out_dir: Path`, `params: dict`, `play_topics: list[str]` (always `/scan`, `/tf_static` and the odometry topic; never `/tf`), and `odom_topic: str`.
  - `slam_replay.main(argv=None) -> int`
    - CLI: `slam_replay.py <run_dir> [--odom odom|odom_imu] [--profile FILE] [--rate R] [--label NAME]`
    - Output folder: `~/.ros/racerbot_runs/<run name>-replay-<odom>-<label>/`
  - `odom_tf_relay.py` (executable, rclpy): it takes `--topic` and publishes each message's pose as `header.frame_id → child_frame_id` on `/tf`, with the message stamp, under `use_sim_time`.

- [ ] **Step 1: Write the failing tests** in `test_slam_replay.py` (unsourced).
  - `test_child_env_always_isolates`
    - `base={'ROS_DOMAIN_ID': '0', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET', 'ROS_STATIC_PEERS': '10.0.0.5', 'PATH': '/x'}`
    - The three ROS keys are overridden, `PATH` is kept, and `base` is unchanged.
  - `test_running_drivers_finds_each_marker`
    - A `tmp_path` fake `/proc` with pid dirs whose `cmdline` is NUL-joined, e.g. `b'/opt/ros/jazzy/lib/urg_node/urg_node_driver\x00--ros-args\x00-r\x00__node:=urg_node\x00'`.
    - Every marker is found; a non-ROS process gives `[]`.
  - `test_running_drivers_matches_executable_path_without_node_remap`: `b'/home/x/install/vesc_driver/lib/vesc_driver/vesc_driver_node\x00'` gives `['vesc_driver_node']`. This is Review Focus 1.
  - `test_running_drivers_skips_unreadable_and_vanished_pids`: one pid dir with no `cmdline` file, one with an empty file, and a non-numeric dir. None of them raises.
  - `test_slam_parameters_layer_order`
    - Snapshot `{'loop_search_maximum_distance': 8.0, 'odom_frame': 'odom', 'use_sim_time': False, 'qos_overrides./tf.publisher.depth': 10}`, profile `{'loop_search_maximum_distance': 3.0}`, frame `odom_imu`.
    - Result: 3.0, `odom_imu`, `True`, and no `qos_overrides` key.
  - `test_plan_refuses_a_run_without_the_odometry_topic`: a fake run whose `bag/metadata.yaml` lists `/scan`, `/odom` and `/tf_static`, but not `/odom/imu`. Asking for `odom_imu` raises `ReplayRefused` matching `'/odom/imu'`. This is Review Focus 2.
  - `test_plan_never_plays_recorded_tf`: for a fake run listing `/tf`, `play_topics` is `['/scan', '/tf_static', '/odom/imu']`, with `/tf` absent.
  - `test_main_refuses_while_drivers_run`
    - `monkeypatch` `running_drivers` to return `['urg_node']` and `subprocess.Popen` to raise if called. This mocks a real process boundary, which A1 allows.
    - `main([...])` returns 2 and prints `urg_node`.

- [ ] **Step 2: Run them; they must fail** with a missing module.

- [ ] **Step 3: Implement `slam_replay.py`'s pure parts** (everything above except the orchestration in `main`).

- [ ] **Step 4: Run the tests; they must pass.** Break `child_env` so it keeps the caller's `ROS_DOMAIN_ID`: `test_child_env_always_isolates` must fail. Revert.

- [ ] **Step 5: Implement the orchestration in `main`.** Every child gets `child_env(os.environ)`, and every step is logged to `<out>/replay.log`.
  1. **Refuse** if `running_drivers()` finds anything (exit 2). Then plan, and write `<out>/slam_params.yaml`.
  2. **Start, in order:**
     - `python3 tools/run_analysis/odom_tf_relay.py --topic <odom_topic> --ros-args -p use_sim_time:=true`
     - `ros2 launch racerbot_launch slam_launch.py slam_params_file:=<out>/slam_params.yaml slam_tracking_file:=<out>/slam_params.yaml use_sim_time:=true slam_odom_frame:=<frame>` (the real launch file, with autostart)
     - `ros2 bag record -o <out>/bag --use-sim-time /tf /pose /map /map_metadata /slam_toolbox/graph_visualization`
  3. **Wait** for `/map` to be listed by `ros2 topic list` in the child environment, or 30 s, then run `ros2 bag play <run>/bag --clock --rate <R> --topics <play_topics>` in the foreground.
  4. **When it ends:** sleep 5 s, then call `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '<out>/map'}}"` and `/slam_toolbox/serialize_map` with `<out>/posegraph`. Then SIGINT every child in reverse order (10 s grace, then SIGTERM).
  5. **Write `<out>/replay.json`:** source run, odom frame, profile path and its contents, rate, git commit (`git rev-parse HEAD`), slam_toolbox version (`ros2 pkg xml slam_toolbox --tag version`), start and end times.
  6. **Copy** the source run's `metadata/` into `<out>/metadata/` so `slam_bag.load` and later tools treat the folder like a run. `slam_replay.py` writes `<out>/slam_toolbox.log` from the launch's output, so `loop_closure_times` works on replays.

- [ ] **Step 6: Commit** the tool, the relay and the tests.

### Task 3: Prove the replay reproduces a live run

**Prerequisite:** the car is parked and bringup is stopped (`running_drivers()` is empty).

- [ ] **Step 1: Replay the 2.5 m/s gyro run** with the run's own settings and no profile:
  `python3 tools/run_analysis/slam_replay.py ~/.ros/racerbot_runs/20261026-194913 --odom odom_imu --label baseline`
  Expected: it exits 0, and `<out>/map.pgm`, `<out>/map.yaml` and `<out>/bag/` exist.
- [ ] **Step 2: Measure it.** Sourced, run `slam_bag.load` and `summarize` on the replay folder (closures from `<out>/slam_toolbox.log`).
  Expected, against the recorded measurement of the live run: `jumps_over_false_jump_m >= 1` and `max_move_m >= 1.5`. The live run had five jumps over 0.5 m, up to 2.56 m.
  - If the replay shows **no** false jumps, the replay does not reproduce the failure. **Stop**, and report the replay's numbers next to the live run's. Async SLAM depends on timing, so try `--rate 1.0` and `--rate 0.5` before concluding anything.
- [ ] **Step 3: Record** both runs' summaries in the spec's A1 section, under a new "Results" heading. Commit.

### Task 4: The good-odometry SLAM profile

**Files:**
- Create: `src/racerbot_launch/config/slam_gyro.yaml`
- Modify: `src/racerbot_launch/launch/slam_launch.py`, `src/racerbot_launch/launch/map_course_launch.py`, `src/racerbot_launch/test/test_map_course_imu.py`, `docs/launchstart.md` (the `gyro_odom` row)

**Interfaces:**
- Consumes: `slam_replay.py` and `slam_metrics.summarize` (Tasks 1-2).
- Produces:
  - `slam_launch.py` argument `slam_profile_file`, whose default is `LaunchConfiguration('slam_tracking_file')`. Re-applying the tracking file is a no-op. It is layered **after** `slam_tracking_file` and before the dict.
  - `map_course_launch.SLAM_GYRO_PROFILE = 'slam_gyro.yaml'`
  - `map_course_launch.slam_profile_for(gyro_odom: str) -> str | None`: the profile's installed path when `gyro_odom` is true, else `None`. It parses the value with `_flag`.

- [ ] **Step 1: Tune by replay.**
  - Write the candidate profiles to `$CLAUDE_JOB_DIR/tmp/profiles/`, each a full `slam_toolbox: ros__parameters:` file:
    - **P1:** `loop_search_maximum_distance: 3.0`, `loop_search_space_dimension: 3.0`
    - **P2:** P1 plus `minimum_travel_distance: 0.5`, `minimum_travel_heading: 0.5`
    - **P3:** P2 plus `loop_match_minimum_response_coarse: 0.45`, `loop_match_minimum_response_fine: 0.55`
  - Replay each against `20261026-194514`, `-194714` and `-194913` with `--odom odom_imu`.
  - **Pick the first candidate (P1, then P2, then P3) where all three replays meet the spec's done-when:**
    - `jumps_over_false_jump_m == 0`;
    - `loop_closures` at least the number of laps driven: 194514 about 3.7 laps, 194714 about 2.6, 194913 about 4.8, from 108 m, 76 m and 140 m on a ~29 m loop;
    - the final map's walls single by inspection of the saved `map.pgm`.
  - If none passes, **stop and report** the table. Do not invent a P4 without the user.
- [ ] **Step 2: Write the failing tests** in `test_map_course_imu.py`.
  - `test_gyro_odom_loads_the_gyro_slam_profile`: `_wired_context(gyro_odom='true')` gives `ctx.launch_configurations['slam_profile_file']` ending in `/slam_gyro.yaml`, and the file exists.
  - `test_without_gyro_odom_no_profile_is_set`: `gyro_odom='false'` leaves `'slam_profile_file'` absent from the launch configurations.
  - `test_slam_launch_layers_the_profile_last`: evaluate the real `slam_launch.py` the way `test_slam_toolbox_gets_the_wired_odom_frame` does, with `slam_profile_file` set. The node's parameter list order is `slam_params_file`, tracking, profile, then the dict.
  - `test_gyro_profile_window_cannot_reach_the_neighbouring_track`
    - oracle: recorded measurement. The false jumps on runs `20261026-194714/-194913` were at least 2.2 m.
    - Assert `profile['loop_search_space_dimension'] / 2 < 2.2`.
  - `test_gyro_profile_evidence_chain_is_at_least_10m`
    - oracle: cited spec (the A1 done-when).
    - Assert `chain_size * effective minimum_travel_distance >= 10.0`, where the effective value is the profile's if set, else `slam_tracking.yaml`'s.
  - Run them; the first four fail (missing file or argument). Then write `slam_gyro.yaml` with the chosen values and comments citing the replay numbers.
- [ ] **Step 3: Implement** the argument in `slam_launch.py` and `slam_profile_for` in `map_course`. In `_wiring`, append `SetLaunchConfiguration('slam_profile_file', path)` only when the profile path is not `None`.
- [ ] **Step 4: Run the tests and Hard rule 1.**
  - Run `python3 -m pytest src/racerbot_launch/test/ -v` (sourced, isolated); all pass.
  - Break it: make `slam_profile_for` return `None` always. The first test must fail. Revert.
- [ ] **Step 5: Docs.** Update `docs/launchstart.md`'s `gyro_odom:=true` row: "also loads `slam_gyro.yaml`: a tighter loop-closure search for good odometry". Add a one-line pointer to `slam_gyro.yaml` at the top of `slam_tracking.yaml`'s loop-closure comment. Run `check_docs.py docs/launchstart.md`.
- [ ] **Step 6: Commit**, with the replay table in the message body.

### Task 5: Laps close at the closest approach

**Files:**
- Modify: `src/pure_pursuit/pure_pursuit/auto_map_race_node.py` (`LapRecorder` around lines 65-305, the parameter at ~362, the construction at ~557), `src/pure_pursuit/config/auto_map_race.yaml`, `src/pure_pursuit/test/test_auto_map_race.py`

**Interfaces:**
- Produces:
  - `LapRecorder(..., close_at_closest_approach: bool = False, closest_approach_margin: float = 0.05)`. The class default keeps today's behaviour, so every existing test stays valid.
  - Node parameters `close_at_closest_approach` (declared default `False`) and `closest_approach_margin_m` (`0.05`). The YAML sets `close_at_closest_approach: true`.

**The behaviour** when `close_at_closest_approach` is true:
- Each tick, evaluate every existing closure condition **except** the heading gate.
- While those hold, track the minimum `distance_from_start` seen. Store the pose's heading error at that minimum, and the index of the last recorded point.
- Close (return `True`) on the first tick where `distance_from_start > minimum + closest_approach_margin`, or where the car leaves the circle. **And** the heading error **at the minimum** must be `<= closure_heading_rad`.
- On closing:
  - set `heading_error` and `distance_from_start` to the values at the minimum;
  - truncate `points` and `point_turn` to the minimum's index;
  - **reset** the closing state.
- If the heading at the minimum fails the gate, reset the closing state and continue the lap. That is Review Focus 3.
- While the car is stationary, distance doesn't rise, so the lap doesn't close. That is Review Focus 4.

- [ ] **Step 1: Write the failing tests.**
  - `test_closest_approach_closes_at_the_nearest_point_on_a_curve`
    - oracle: closed form. A circle of radius 3 m sampled every 0.05 m, with the start on the arc.
    - At the closest approach, distance is at most 0.05 m and heading error at most `0.05/3` rad plus sampling. Assert `heading_error < math.radians(2.0)`.
    - The **old** mode on the same input closes with a heading error above 10°; assert that too, to show the test can tell the modes apart.
  - `test_closest_approach_does_not_close_while_still_approaching`: `update` returns `False` on every tick until the distance first rises by more than the margin.
  - `test_closest_approach_rejects_a_bad_heading_at_the_minimum`: approach the start facing 90° off. No closure, and the lap goes on (`departed` still true, points still growing).
  - `test_stationary_inside_the_circle_does_not_close_then_moving_away_does`: 50 identical poses at the minimum give `False`; a pose 0.1 m further away gives `True`.
  - `test_closing_truncates_points_to_the_minimum`: the last stored point equals the minimum's sample, to `abs=1e-9` m.
  - `test_recorded_lap_passes_the_quality_check` (recorded measurement)
    - Load `/slam_pose` and `/odom` from `20261026-194913`'s cached `slam.npz`, using Task 1's loader. **Skip, with a reason naming this plan's Task 5, if the run directory is absent.**
    - Replay through `LapRecorder` with that run's parameters (`auto_map_race_node.yaml` snapshot: spacing 0.15, min_distance 5.0, departure 2.0, closure 1.5, heading 30°, duration 15 s, turn 300°, jump 0.12, widen 1.25, max closure 4.0) and `close_at_closest_approach=True`.
    - Every closed lap's `heading_error` must be at most `math.radians(15.0)`. The old mode's laps were 28.1-29.7°.
- [ ] **Step 2: Run them** (sourced, isolated); they must fail with an unexpected keyword `close_at_closest_approach`.
- [ ] **Step 3: Implement** the behaviour above, plus the node parameter and YAML. Log one INFO line at closure: `closed at closest approach: d=..m, heading error=..deg`.
- [ ] **Step 4: Run the whole of `src/pure_pursuit/test/`.**
  - Collected must equal the `grep -c '^def test'` total. Everything passes, including every deadman test.
  - Break it: close at the first tick inside the circle in the new mode. `test_closest_approach_closes_at_the_nearest_point_on_a_curve` must fail. Revert.
- [ ] **Step 5: Docs.** In `docs/operations.md#reading-the-terminal-while-it-maps`, explain the new closure line. Run `check_docs.py`.
- [ ] **Step 6: Commit.**

### Task 6: `max_lateral_accel` 2.0 → 2.5

**Files:**
- Modify: `src/gap_follow/config/gap_follow.yaml:110`, `src/gap_follow/README.md:455` (shipped value), `docs/gap-follow.md` lines 42, 80, 139, 165 and 424 (each says the shipped value is 2.0)
- **Decision, 2026-10-26:** `src/neural_racer/test/test_config.py::test_wrapper_values_match_gap_follow` requires `neural_racer.yaml`'s `max_lateral_accel` to equal `gap_follow.yaml`'s. The user chose to raise **both** to 2.5 and leave the test unchanged. That lifts `neural_racer.yaml`'s "Phase 10+" reservation for this one value. Nothing else in `src/neural_racer/` changes.
- The grip evidence was measured with the limit at 2.0, and `docs/gap-follow.md`'s 2026-10-19 floor run reported sliding above 2.0. The docs give both measurements, and the first floor runs at 2.5 follow the wheels-off, then low-speed order. Task 15 replaces the guess with a measured, adaptive limit.

- [ ] **Step 1: Find any test pinning the shipped value.** `grep -rn "max_lateral_accel" src/gap_follow/test tools/f1tenth_sim/tests src/racerbot_launch/test`. If a test asserts 2.0 as a spec, **stop and report** under Hard rule 2.
- [ ] **Step 2: Simulator check before the change.**
  - Baseline: `python3 tools/f1tenth_sim/eval_gap.py --track indoor --output $CLAUDE_JOB_DIR/tmp/eval_a6_base.json`
  - Candidate: the same with `--set max_lateral_accel=2.5 --output .../eval_a6_25.json`
  - Expected: every track that passed still passes, and `contact` stays 0. Lap times are reported.
- [ ] **Step 3: Change the YAML and docs.** Cite run `20261026-194514`'s grip measurement in the YAML comment: 99-107% of the commanded turn up to 2.5 m/s², 93% at 2.5-3.0. Run `check_docs.py docs/gap-follow.md src/gap_follow/README.md`.
- [ ] **Step 4: Run the gap_follow suite** (sourced, isolated). The collected count must match `grep -c`, and everything passes.
- [ ] **Step 5: Commit**, with both `eval_gap` summaries in the message.

### Task 7: Diagnose the weave

**Files:**
- Create: `tools/run_analysis/weave.py`, `tools/run_analysis/tests/test_weave.py`
- Modify: `tools/f1tenth_sim/gap_mirror.py` (telemetry only: `tel["target_angle"]` and `tel["requested_steering"]`), `tools/f1tenth_sim/eval_gap.py` (adds `steering_reversals_per_s` to each track's result)

**Interfaces:**
- Consumes: `runlog.load_run`, `replay.replay` / `replay.Tick.telemetry`, `imu_report.load_imu`, `imu_report.gyro_motion`, `actuation.analyze`.
- Produces:
  - `weave.reversals_per_s(t: np.ndarray, steering: np.ndarray, swing_rad: float = 0.2) -> float`: the number of direction changes whose peak-to-peak swing exceeds `swing_rad`, divided by the duration.
  - `weave.loop_model(tau_s: float, speed: float, gain: float, wheelbase: float = 0.36) -> tuple[float, float]`: `(zeta, f_hz)` of `τθ̈ + θ̇ + (vK/L)θ = 0`, i.e. `zeta = 1/(2*sqrt(tau*v*K/L))` and `f_hz = sqrt(v*K/(L*tau))/(2π) * sqrt(max(0, 1 - zeta**2))`.
  - `weave.target_jump_share(ticks, swing_t: np.ndarray, jump_rad: float = 0.15) -> float`: the share of swings that start within one scan of a `target_angle` jump larger than `jump_rad`.
  - `weave.diagnose(run_dir: Path) -> dict`, with keys `peak_hz`, `coherence_at_peak`, `phase_lag_deg`, `tau_s`, `model_zeta`, `model_hz`, `target_jump_share` and `verdict` (`'loop'`, `'aim_point'` or `'unclear'`).
  - `eval_gap` result key `steering_reversals_per_s`.

**The decision rule, stated so it can't be bent afterwards:**
- **`'loop'`:** the steering-to-gyro-yaw-rate coherence at the steering's spectral peak is at least 0.7, `model_hz` is within 30% of `peak_hz`, and `target_jump_share` is below 0.3.
- **`'aim_point'`:** `target_jump_share` is at least 0.5.
- **Otherwise `'unclear'`.**

- [ ] **Step 1: Write the failing tests.**
  - `test_reversals_counts_a_known_sine`: oracle closed form. A 0.6 Hz sine of amplitude 0.15 rad (peak-to-peak 0.3) over 10 s gives `== pytest.approx(1.2, abs=0.11)  # reversals/s`. Amplitude 0.05 gives 0.
  - `test_loop_model_matches_the_closed_form`: `tau=0.2, v=2.0, K=0.9, L=0.36` gives zeta `1/(2*sqrt(1.0))=0.5` (`abs=1e-12`) and `f_hz = sqrt(25)/(2π)*sqrt(0.75)` (`abs=1e-12`).
  - `test_loop_model_rejects_nonpositive_inputs`: zero or negative `tau`, speed or gain raises `ValueError` matching `'positive'`.
  - `test_target_jump_share_on_synthetic_ticks`: four swings, three starting at a 0.3 rad target jump, gives `0.75`. No swings gives `nan`, not a `ZeroDivisionError`.
- [ ] **Step 2: Run them; they fail. Implement; they pass.** Hard rule 1: drop `sqrt(1 - zeta**2)` from `f_hz`, and `test_loop_model_matches_the_closed_form` must fail. Revert.
- [ ] **Step 3: Mirror telemetry.** Add the two telemetry keys and the eval metric. Check that `test_gap_mirror_parity.py` passes unchanged: telemetry is not in parity.
- [ ] **Step 4: Run it on the evidence.** Sourced, run `weave.diagnose` on `~/.ros/racerbot_runs/20261026-194514` and `-194913`, and run `eval_gap.py --track indoor` to read baseline `steering_reversals_per_s`.
  - Record all numbers, and **the verdict for each run**, in the spec's A5 section under "Results".
  - If the simulator's baseline reversals are below a third of the car's, the simulator does not reproduce the weave. Say so in the results: Tasks 8-9 then lean on the loop model and the car, not the simulator.
- [ ] **Step 5: Commit.**

### Task 8: Weave fix, part 1, the gain schedule (no code)

**Gate:** Task 7's verdict is `'loop'` on both runs. If it's `'aim_point'`, skip Tasks 8-9 and **stop to report**: the fix is target hysteresis, which needs its own design. If `'unclear'`, stop and report.

**Files:**
- Modify: `src/gap_follow/config/gap_follow.yaml` (`steering_gain`, `steering_gain_speed_ref`, maybe `max_steering_rate`), `docs/gap-follow.md`, `src/gap_follow/README.md`

- [ ] **Step 1: Pick candidates from the model.**
  - With Task 7's measured `tau_s`, compute the largest `v*K` that gives `zeta >= 0.7`: `vK <= L/(4*tau*0.49)`.
  - Candidate schedules keep `K = steering_gain * min(1, speed_ref/v)`, with `steering_gain` in {1.0, 0.8} and `steering_gain_speed_ref` chosen so that `speed_ref*steering_gain <= vK_max`.
- [ ] **Step 2: Sweep in the simulator.**
  - Each candidate runs through `eval_gap.py --track indoor --set ...`, both directions (`--reverse`), seeds 12345, 1 and 2.
  - **Accept** a candidate only if, against Task 6's results:
    - contacts stay at 0;
    - `stop_steps` doesn't rise;
    - mean lap time is no worse than 3%;
    - `steering_reversals_per_s` falls by at least 30%.
  - If no candidate meets all four, keep the YAML as it is and say so in the results.
- [ ] **Step 3: Apply and document.** Apply the winner, update the docs, run the gap_follow suite (sourced, isolated, full collection count) and commit with the sweep table.

### Task 9: Weave fix, part 2, gyro steering damping behind a flag

**Gate:** Task 7's verdict is `'loop'`.

**Files:**
- Modify: `src/gap_follow/gap_follow/gap_logic.py`, `src/gap_follow/gap_follow/gap_follow_node.py`, `src/gap_follow/config/gap_follow.yaml`, `src/gap_follow/launch/gap_follow_launch.py`, `tools/f1tenth_sim/gap_mirror.py`, `tools/f1tenth_sim/eval_gap.py`, `docs/gap-follow.md`, `src/gap_follow/README.md`
- Test: `src/gap_follow/test/test_gap_logic.py` (the logic function); `src/gap_follow/test/test_gap_follow_steering_damping_node.py` (new, node behaviour; copy the node fixture, `/test_only/drive` remap and fake-`/joy` helpers from `test_gap_follow_node.py`); `src/gap_follow/test/test_gap_follow_live_tuning.py` (if the new parameters are live-tunable, its existing spec checks must pass)

**Interfaces:**
- Produces:
  - `gap_logic.steering_damping(yaw_rate: float, gain_s: float) -> float`: returns `-gain_s * yaw_rate` in radians, and `0.0` when `yaw_rate` is not finite or `gain_s <= 0`.
  - Node parameters:
    - `enable_steering_damping` (bool, default `False` in code **and** YAML)
    - `steering_damping_gain` (s, default `0.0`, YAML `0.07`, the model value from the spec reasoning; Task 8's sweep or the car refines it)
    - `imu_topic` (`/imu/data`), `imu_status_topic` (`/imu/status`), `imu_timeout_sec` (`0.1`)
  - `gap_follow_launch.py` argument `steering_damping:=true|false` (default false). When true, it includes `imu_bridge`'s `vehicle_state_launch.py` and sets `enable_steering_damping: true`. Same `_flag` parsing and refusal style as `map_course`.
  - `/drive_intent` reason detail `steering_damping_off: <why>`, where why is `imu waiting`, `imu fault`, `imu stale` or `no imu`, whenever the flag is on but damping is zero.

**The law:**
- Add the damping term to `requested_steering` (the line that sums `bearing_gain * target_angle + centering_bias + setup_bias`), **before** the clip, the arc guard and the slew limit.
- Arc guard and swept clearance therefore still check the damped command.
- TTC, emergency clearance and the deadman are downstream and unchanged.
- The yaw rate is used only when the latest `/imu/status` `state == 'ok'` **and** the latest `/imu/data` is younger than `imu_timeout_sec`. Otherwise the term is 0, which is the user's "fall back to today" decision.

- [ ] **Step 1: Write the failing tests.**
  - `steering_damping`:
    - `(0.5, 0.07) == pytest.approx(-0.035, abs=1e-12)  # rad`
    - `(nan, 0.07) == 0.0`, `(inf, 0.07) == 0.0`, `(0.5, 0.0) == 0.0`, `(0.5, -1) == 0.0`
    - Sign: a positive (left) yaw rate gives negative (right) damping.
  - Node tests (published command on `/test_only/drive`, with LB held through the existing fake `/joy` helper, and a fixed scan whose gap gives a known steering):
    - with the flag on and an `ok` IMU at yaw rate 1.0, the published steering equals the undamped command minus `gain*1.0`, within slew limits (set `max_steering_rate` high in the test);
    - with the flag on and `/imu/status` `fault`, the published command equals the undamped one, and `/drive_intent` contains `steering_damping_off: imu fault`;
    - the same for **stale** (last IMU 0.2 s old), **never received**, and a **NaN yaw rate**. These are Review Focus 5;
    - with the flag off and an `ok` IMU, the command equals the undamped one;
    - **the four deadman deny-path tests run with the flag on**: LB released, never received and stale publish zero speed. These are new instances; the existing ones are unchanged.
- [ ] **Step 2: Run them** (sourced, isolated); they must fail.
- [ ] **Step 3: Implement** the logic function, the node subscriptions (status latched, QoS like `neural_racer_node.py`'s), the gating, the intent reason, the YAML and the launch argument.
- [ ] **Step 4: Mirror it.** `gap_mirror` takes an optional `yaw_rate` per `command()` call, and `eval_gap` passes the gym's true `state[5]` when `--set enable_steering_damping=true`. Parity must pass with the flag off.
- [ ] **Step 5: Simulator check.** Sweep `steering_damping_gain` in {0.04, 0.07, 0.10} on top of Task 8's result, with the same acceptance as Task 8. If none passes, ship the flag **off** with gain 0.07 and record that it is unproven in the simulator.
- [ ] **Step 6: Run the whole suite and Hard rule 1.** Run the whole gap_follow suite (full count). Break it: flip the sign in `steering_damping`; the sign test and the node `ok`-IMU test must fail. Revert.
- [ ] **Step 7: Docs and commit.** Document it in `docs/gap-follow.md` (a new section "Steering damping (gyro)", with how to turn it on, what it falls back to, and the car test order) and in the README's parameter table. Run `check_docs.py`, then commit.

### Task 10: Corner setup on vs. off (report only)

- [ ] **Step 1: Simulator.** Run `eval_gap.py --track indoor --set enable_corner_setup=false`, both directions, seeds 12345, 1 and 2, against the same with `true`, on the YAML as left by Tasks 6-9.
- [ ] **Step 2: Real scans.** Run `python3 tools/run_analysis/analyze_run.py ~/.ros/racerbot_runs/20261026-194514 --set enable_corner_setup=false`, and the same for `-194913`. Note the binding-cap counts and the replayed mean speed against the plain replay.
- [ ] **Step 3: Record** a table in the spec's A6 section: lap time, contacts, stops, binding caps and mean speed. **Do not change the default.** Commit the docs.

### Task 11: Race pace, every lever costed (report only)

- [ ] **Step 1: Can the motor deliver more acceleration?**
  - From `20261026-194514`'s `run.npz`, compare commanded speed slope (`/drive`) with achieved slope (`/odom`) in windows where `binding == 'accel'` (from `analyze_run`'s replay ticks).
  - Report the achieved acceleration's 90th percentile against `max_acceleration` 3.0.
  - If achieved stays below 3.0 at the 90th percentile, raising the ramp buys nothing: say so.
- [ ] **Step 2: Cost each lever** with the minimum-time model on the driven path. Find it with `grep -rn "minimum.time\|min_time" tools/ docs/`; it was used by the 2026-10-26 gap_follow analysis.
  - If it isn't in the tree, implement `tools/run_analysis/lap_time_model.py`, with a test against a closed form (a circle of radius R at `a_lat` gives `T = 2πR/sqrt(a_lat*R)`).
  - Cost these levers: smooth speed (Tasks 7-9), `a_lat` 2.5 (Task 6), the acceleration ramp (Step 1) and corner setup (Task 10).
  - **The curve-cap basis (spec A5 step 3):** replay `20261026-194514` with `analyze_run.py --set` using the YAML as left by Tasks 6-9, and count speed dips over 0.7 m/s.
    - Today there are 22 in 48 s.
    - If 10 or more remain, list "base the curve cap on the commanded rack angle, behind a flag" as a lever, with its cost from the model.
    - Do not build it here.
- [ ] **Step 3: Record** the table in the spec's A7 section with the next three levers ranked. Commit.

### Task 13: Calibrate the simulator's grip to car 1

**Why:** `tools/f1tenth_sim/sim_fidelity/calibration.py`'s `surface_friction` 0.55 (a 5.4 m/s² friction circle) was fitted to run `20261019-200703`. That run is not on car 1: likely car 2's floor, since the workspace was ported from car 2 on 2026-10-23, but unverified. Car 1 achieved 93% of its commanded turn at 2.5-3.0 m/s² on 2026-10-26. Racing in the simulator decides nothing while its grip may be another car's.

**Result (2026-10-27):** the refit on car 1's runs confirmed 0.55. Run `20261026-194514` is at its minimum there, and `-194913` is nearly flat. No value changed; only the provenance now cites car 1 (commits `2594468`, `641c23b`).

**Files:** `tools/f1tenth_sim/sim_fidelity/calibration.py` (value and provenance entry), `docs/sim-fidelity-audit.md` (one row), plus any test in `tools/f1tenth_sim/tests/` that pins the provenance format.

- [ ] **Step 1: Fit, inline by the controller.** Run `tools/run_analysis/fit_sim.py <run> --grid surface_friction=0.25,0.30,0.35,0.40,0.45,0.55` on `20261026-194514` and `20261026-194913`, each LB hold. Pick the value that best fits both runs. Also report the steering delay and servo gain the fitter prefers, without changing them, as the input for Task 7.
- [ ] **Step 2: Change and cite.** Set `surface_friction` to the fit, with provenance `MEASURED` naming both runs and the grid result. Keep the Oct-19 value in the provenance text as car 2's.
- [ ] **Step 3: Re-run the baselines.** Run `eval_gap.py --track indoor` before and after the change and record both in the commit. This is a simulator calibration, not a test oracle change; a test pinning 0.55 means stop and report.

### Task 14: Head-to-head racing between two of our cars

**Files:** `tools/f1tenth_sim/eval_gap_multi.py`, and `tools/f1tenth_sim/tests/test_eval_gap_multi_head_to_head.py` (new).

**Interfaces:**
- Produces:
  - CLI `--car-set N:name=value` (repeatable; N is the 0-based car index), applied after `--set`, so each car runs its own `gap_follow` parameters.
  - `head_to_head(run_result: dict) -> dict`, a pure function, with keys:
    - `finish_order` (car indices, by time to complete `--laps`; unfinished cars last, ordered by distance);
    - `gap_s` (time between the first two finishers, or `inf`);
    - `overtakes` (per car: how many times it moved from behind to ahead along the centreline);
    - `contacts` (per car, from sim_fidelity's chassis check, never gym's flag);
    - `winner` (the first finisher with no contact, else `None`).
  - Per-seed and aggregate output: wins per car, mean gap and contacts. Exit 0 always for a head-to-head run; it is a comparison, not a pass/fail.
  - **Start fairness:** with `--swap-starts`, every seed is run twice with the start order swapped, so starting position cancels out.

- [ ] **Step 1: Write the failing tests:**
  - `head_to_head` on synthetic results: a clean win, a win with contact (the winner is the clean car), nobody finishing, and an overtake counted once, not on every tick of side-by-side.
  - `--car-set` parsing: a bad index or a malformed item raises `SystemExit` naming the bad item.
- [ ] **Step 2: Implement.** Hard rule 1: count every tick as an overtake, and the overtake test must fail.
- [ ] **Step 3: Smoke.** Run `--cars 2 --car-set 1:max_lateral_accel=2.0 --swap-starts --seeds 2 --track indoor_wide` and show the table. Commit.

### Task 15: Adaptive grip from the IMU (`adaptive_grip`)

**Gate:** Tasks 9 (IMU subscription and gating), 13 and 14 are done.

**Files:** `src/gap_follow/gap_follow/gap_logic.py`, `gap_follow_node.py`, `config/gap_follow.yaml`, `launch/gap_follow_launch.py`, `tools/f1tenth_sim/gap_mirror.py`, `eval_gap.py`, `eval_gap_multi.py`, `docs/gap-follow.md`, `src/gap_follow/README.md`. Tests in `src/gap_follow/test/test_gap_logic.py` and the Task 9 node-test file.

**Interfaces:**
- Produces:
  - `gap_logic.grip_ratio(yaw_rate: float, speed: float, steering: float, wheelbase: float) -> float`: `yaw_rate*speed / (speed**2*tan(steering)/wheelbase)`. Returns `nan` when `speed < 1.0` or `|steering| < 0.05`, or for non-finite inputs.
  - `gap_logic.AdaptiveGrip`, a dataclass with fields `base`, `ceiling`, `rise_per_s`, `cut_factor`, `hold_ratio`, `slip_ratio`, `judge_fraction` and `limit`. Its method `update(ratio: float, demand: float, dt: float) -> float` returns the lateral limit to use this tick:
    - if `ratio` is nan, or `demand < judge_fraction*limit`: unchanged;
    - if `ratio >= hold_ratio`: `limit = min(ceiling, limit + rise_per_s*dt)`;
    - if `ratio < slip_ratio`: `limit = max(base, limit*cut_factor)`;
    - otherwise unchanged.
  - Node parameters:
    - `enable_adaptive_grip` (default `False` in code and YAML);
    - `adaptive_grip_ceiling` (3.0), `adaptive_grip_rise_per_s` (0.2), `adaptive_grip_cut_factor` (0.85), `adaptive_grip_hold_ratio` (0.95), `adaptive_grip_slip_ratio` (0.90), `adaptive_grip_judge_fraction` (0.8);
    - the base is `max_lateral_accel`;
    - the ratio is low-passed with a 0.3 s time constant before judging.
  - The curve cap and cornering anticipation use the adaptive limit instead of `max_lateral_accel`.
  - **IMU failure:** with the IMU not `ok`, stale, never received or NaN, the limit resets to `max_lateral_accel`, today's behaviour, and `/drive_intent` gets `adaptive_grip_off: <why>`. This reuses Task 9's IMU gating.
  - Launch argument: `adaptive_grip:=true|false` in `gap_follow_launch.py`, which starts the IMU stack like `steering_damping:=true`.

- [ ] **Step 1: Write the failing tests.**
  - `grip_ratio`, oracle closed form: on a circle of radius 2 m at 2 m/s, true yaw rate 1.0 rad/s and steering `atan(0.36/2)` give 1.0 (`abs=1e-12`). Half the yaw rate gives 0.5. Each guard returns nan.
  - `AdaptiveGrip`, oracle closed form:
    - 10 s of ratio 1.0 at full demand from 2.0 gives `min(3.0, 2.0 + 0.2*10)`;
    - one slip tick from 2.8 gives `2.8*0.85`;
    - never below `base`, never above `ceiling`;
    - low demand leaves it unchanged;
    - nan leaves it unchanged.
  - Node tests: flag on with a healthy IMU raises the curve-capped published speed over time on a fixed scan and yaw-rate feed. Each IMU failure mode publishes exactly the fixed-2.0 command, with the `/drive_intent` reason.
  - The four deadman deny-path tests with the flag on.
- [ ] **Step 2: Run them; they fail. Implement; they pass.** Hard rule 1 breaks: flip the comparison in the hold branch (`>=` to `<`), and remove the `base` floor.
- [ ] **Step 3: Mirror it.** `gap_mirror` takes the yaw rate (already added in Task 9). `eval_gap` and `eval_gap_multi` pass the gym's true yaw rate. Parity passes with the flag off.
- [ ] **Step 4: Prove it in the simulator, inline.** With Task 13's grip, `eval_gap.py --track indoor` with the flag on must give no new contacts and stops. Then race `eval_gap_multi.py --cars 2 --car-set 1:enable_adaptive_grip=true --swap-starts --seeds 6` on each indoor track. Record wins, gap and contacts in the spec. The flag ships **off** either way; it is only proposed for floor testing if it wins without contacts.
- [ ] **Step 5: Docs, full suite, commit.** Document it in `docs/gap-follow.md` ("Adaptive grip (IMU)": what it does, how it falls back, the car test order) and in the README parameter table. Run `check_docs.py`.

### Task 12: Final verification and hand-over

**Run order (decided 2026-10-26):** Tasks 1-11 and 13-15 before Task 12. Task 13 runs before Tasks 8, 9 and 10's simulator checks, so they judge against car 1's grip.

- [ ] **Step 1: Run every affected suite,** sourced and isolated, comparing each collected count to `grep -c '^def test'`:
  - `src/gap_follow/test/`, `src/pure_pursuit/test/`, `src/racerbot_launch/test/`
  - `tools/run_analysis/tests/`, plus `tools/f1tenth_sim/tests/` if it exists.
- [ ] **Step 2: Build** with `colcon build --symlink-install --packages-select gap_follow pure_pursuit racerbot_launch`. A clean build is the success signal.
- [ ] **Step 3: Static launch check,** with nothing driving:
  - `ros2 launch racerbot_launch map_course_launch.py gyro_odom:=true --show-args`
  - `ros2 launch gap_follow gap_follow_launch.py steering_damping:=true --show-args`
  Both list the new arguments.
- [ ] **Step 4: Update the spec's status** with what was built, what was gated off and why. Run `check_docs.py` on every touched doc.
- [ ] **Step 5: Push** the branch `worktree-slam-gapfollow-2026-10-26` to `origin` (never `main`).
- [ ] **Step 6: Hand over the car test order** (spec A8) with the exact commands:
  - `ros2 run racerbot_launch map_course gyro_odom:=true`
  - `ros2 launch gap_follow gap_follow_launch.py steering_damping:=true`, first wheels off, LB held.
