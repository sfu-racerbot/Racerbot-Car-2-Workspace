# Changelog

Notable changes to the workspace's own packages, newest first. Format:
one dated section per work session, grouped by package, with behavior
changes and new/removed parameters called out explicitly. Upstream
submodule bumps don't go here (see `docs/git-setup.md`) — this file is
for changes the team made.

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
