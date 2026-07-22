# Changelog

Notable changes to the workspace's own packages, newest first. Format:
one dated section per work session, grouped by package, with behavior
changes and new/removed parameters called out explicitly. Upstream
submodule bumps don't go here (see `docs/git-setup.md`) — this file is
for changes the team made.

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
