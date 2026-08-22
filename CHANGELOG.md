# Changelog

Notable changes to the workspace's own packages, newest first. Format:
one dated section per work session, grouped by package, with behavior
changes and new/removed parameters called out explicitly. Upstream
submodule bumps don't go here (see `docs/git-setup.md`) — this file is
for changes the team made.

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
