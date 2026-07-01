# `pure_pursuit`

Map-based race controller: given a saved (x, y, speed) racing line and a live localization pose, computes steering + speed every control tick via the Pure Pursuit algorithm. This file documents the code, the math, and every parameter in detail. For the end-to-end workflow (mapping → localizing → recording a lap → generating the profile → racing) and the full derivations/diagrams, see [docs/racing-autonomy.md](../../docs/racing-autonomy.md); for running it, see [docs/operations.md](../../docs/operations.md#racing-with-the-pure-pursuit-stack).

## Files

| File | What it is |
|---|---|
| [`pure_pursuit/racing_math.py`](pure_pursuit/racing_math.py) | **All the actual math**, as plain functions with no `rclpy`/ROS imports — frame conversions, Pure Pursuit geometry, path indexing, offline velocity-profile generation, CSV I/O, gap-finding, and opponent detection/tracking geometry. Deliberately kept separate from ROS plumbing so it's readable end-to-end and unit-testable without a robot (see [`test/test_racing_math.py`](test/test_racing_math.py)). |
| [`pure_pursuit/pure_pursuit_node.py`](pure_pursuit/pure_pursuit_node.py) | The race controller ROS node — loads a profiled CSV, wires up subscriptions/timer, calls into `racing_math` every control tick, and owns every safety watchdog plus opponent tracking/overtaking (see [`test/test_opponent_integration.py`](test/test_opponent_integration.py) for the node-level tests). |
| [`pure_pursuit/waypoint_recorder_node.py`](pure_pursuit/waypoint_recorder_node.py) | Records localized `(x, y)` positions to a CSV while you drive a lap by hand. |
| [`pure_pursuit/generate_velocity_profile.py`](pure_pursuit/generate_velocity_profile.py) | Offline CLI tool — turns a raw `(x, y)` recording into a paced `(x, y, speed)` racing line. |
| [`config/pure_pursuit.yaml`](config/pure_pursuit.yaml), [`config/waypoint_recorder.yaml`](config/waypoint_recorder.yaml) | Parameters for the two nodes above. |
| [`launch/pure_pursuit_launch.py`](launch/pure_pursuit_launch.py), [`launch/waypoint_recorder_launch.py`](launch/waypoint_recorder_launch.py) | Launch files — both read their YAML at launch-generation time so a `waypoints_file:=`/`output_file:=` argument overrides just that one value. |
| [`waypoints/example_stadium_raw.csv`](waypoints/example_stadium_raw.csv) | Synthetic demo recording to try the pipeline without a real car. |

## Interface (`pure_pursuit_node`)

- **Subscribes:** `<pose_topic>` (`geometry_msgs/PoseStamped`, default `/pf/viz/inferred_pose`, from `particle_filter`), `<scan_topic>` (`sensor_msgs/LaserScan`, default `/scan`, reactive safety net), `<joy_topic>` (`sensor_msgs/Joy`, default `/joy`, deadman button)
- **Publishes:** `<drive_topic>` (`ackermann_msgs/AckermannDriveStamped`, default `/drive`)
- Runs a fixed-rate `create_timer` control loop (`control_rate_hz`, default `40Hz`) rather than driving control directly off the pose callback — see *"Why a timer, not the callback"* below.

## The math (`racing_math.py`), in detail

### 1. Frame conversions

**Yaw from quaternion** — the car only needs 2D heading, so this is the standard atan2 yaw-only extraction, not a full Euler decomposition:

$$\psi = \operatorname{atan2}\big(2(wz + xy),\ 1 - 2(y^2 + z^2)\big)$$

**World → body frame** — rotates a map-frame offset $(dx, dy)$ into the car's body frame (REP-103: x forward, y left) by $-\psi$:

$$x_{body} = \cos\psi \cdot dx + \sin\psi \cdot dy \qquad y_{body} = -\sin\psi \cdot dx + \cos\psi \cdot dy$$

### 2. Pure Pursuit steering geometry

Given a lookahead target at body-frame coordinates $(x_{body}, y_{body})$, the unique circle tangent to the car's current heading (the body-frame x-axis) at the origin and passing through the target has curvature:

$$\kappa = \frac{2\, y_{body}}{x_{body}^2 + y_{body}^2}$$

*(derivation: such a circle is centered at $(0, R)$; substituting the target point into $x_{body}^2 + (y_{body}-R)^2 = R^2$ and solving for $R = 1/\kappa$ gives the formula above.)* Target to the left ($y_{body} > 0$) → positive curvature → left turn, matching `AckermannDriveStamped`'s sign convention directly. Near-zero distance to target is guarded explicitly (returns `kappa = 0`, i.e. straight) to avoid dividing by ~0.

The bicycle-model steering angle that achieves curvature $\kappa$ with wheelbase $L$:

$$\delta = \arctan(L \cdot \kappa)$$

**Adaptive lookahead** — a *fixed* lookahead is a bad compromise (oscillates at low speed if long enough to be smooth at race speed, or cuts corners at high speed if short enough to corner tightly at low speed), so lookahead scales with the car's current speed:

$$L_d = \operatorname{clip}(k \cdot v + L_{min},\ L_{min},\ L_{max})$$

`gain` ($k$) is meters of extra lookahead per m/s; defaults `min_lookahead=0.6`, `max_lookahead=2.5`, `lookahead_speed_gain=0.35`.

### 3. Path indexing

**Nearest-point search** (`find_nearest_index`) — brute-force distance to every waypoint, *unless* a previous index + `nearest_search_window` are given, in which case only a window of `±search_window` waypoints around last tick's answer is searched. Two reasons: it's faster (O(window) vs O(N)), and — more importantly — on a track that comes close to itself (a hairpin, a figure-eight, a pit-lane split) the *globally* nearest waypoint can be on a completely different branch of the track than the car is actually on. Windowing keeps the tracker locked onto the correct branch instead of "teleporting" across the track. This distance also **is** the cross-track error used by the lost/kidnapped watchdog.

**Lookahead-point search** (`find_lookahead_index`) — walks forward along the path from the nearest index, accumulating `seg_len` (precomputed once at startup, not per-tick), until the accumulated distance reaches $L_d$. This snaps to the nearest *recorded* waypoint rather than solving the textbook circle-path intersection exactly — simpler, and the error introduced is bounded by the waypoint spacing (keep `waypoint_recorder_node`'s `min_spacing_m` small and it's negligible).

### 4. Offline raceline generation (`generate_velocity_profile`)

**Curvature** (`estimate_path_curvature`) — Menger curvature from each waypoint and its immediate neighbors $A, B, C$, using the triangle-area/circumradius identity $\text{area} = \frac{abc}{4R}$:

$$\kappa = \frac{4 \cdot \text{area}(A,B,C)}{|AB|\cdot|BC|\cdot|CA|}$$

No calculus or spline-fitting needed — works directly on raw, slightly-noisy hand-driven recordings. Tight corners → small triangle area relative to side lengths → high $\kappa$; straights → near-zero area → $\kappa \approx 0$.

**Cornering speed limit** (simplified friction circle, lateral-only): a car on a curvature-$\kappa$ arc at speed $v$ feels lateral acceleration $a_{lat} = v^2\kappa$. Capping at the tunable grip limit $a_{lat,max}$:

$$v_{corner} = \min\left(v_{max},\ \sqrt{\dfrac{a_{lat,max}}{\kappa}}\right)$$

**Forward/backward smoothing** (`compute_velocity_profile`) — a raw per-point cornering limit alone would demand teleporting from race speed to walking pace one waypoint before a corner, which is physically impossible. Two sweeps fix this, each capping how much speed may change between adjacent waypoints a distance $ds$ apart:

$$\text{forward (accel):}\quad v_i \leftarrow \min\big(v_i,\ \sqrt{v_{i-1}^2 + 2\,a_{accel,max}\,ds}\big)$$
$$\text{backward (brake):}\quad v_i \leftarrow \min\big(v_i,\ \sqrt{v_{i+1}^2 + 2\,a_{brake,max}\,ds}\big)$$

The **backward pass is what actually creates real braking zones** — it propagates a corner's low speed limit backward into the straight leading up to it, so the car starts slowing down early instead of "discovering" the corner's limit only at the apex. A closed loop has no single seed point for these sweeps (index 0's "previous" is the *last* index, not yet finalized on the first pass), so both are repeated `smoothing_passes` times (default `3`) to let the start/finish seam converge — harmless to over-run since both passes only ever lower a value, never raise one.

This is **not** a time-optimal racing line (that needs a full path+speed nonlinear optimizer, e.g. TUM's `global_racetrajectory_optimization`) — it's a fast, dependency-light approximation using the same cornering-speed-plus-smoothing technique, without also re-optimizing the path geometry itself. See [docs/racing-autonomy.md](../../docs/racing-autonomy.md#limitations-and-how-to-go-further) for what a further iteration could add.

### 5. CSV I/O

Two file "shapes": raw `x,y` (written by `waypoint_recorder_node`, read by `generate_velocity_profile`) and profiled `x,y,speed` (written by `generate_velocity_profile`, read by `pure_pursuit_node`). `load_profiled_csv` raises `ValueError` if the `speed` column is missing — the most common mistake being pointing `pure_pursuit_node` at a raw recording that hasn't been profiled yet.

## The control loop (`pure_pursuit_node.py`)

### Why a timer, not the pose callback

`pose_callback`/`scan_callback`/`joy_callback` only ever *cache* the latest message and its arrival time — the actual driving logic in `control_loop()` runs on a `create_timer()` at a fixed rate instead. If a sensor feed died outright (localization crashes, a LIDAR cable falls out) and control were driven directly by that sensor's own callback, the control loop would simply stop being invoked — and the last published command would stay "live" on `/drive` forever. A timer-driven loop keeps re-checking every watchdog on its own schedule regardless of whether new sensor data is still arriving, so a dead feed is something the watchdogs below can actually catch.

`control_loop()` also wraps the entire per-tick logic in `try/except`: any unhandled exception publishes a stop command *before* re-raising, so a bug can never leave the last (possibly full-speed) command sitting on `/drive`.

### Per-tick sequence, in order

1. **Deadman watchdog** (checked first, ahead of everything else) — LB must be held on a live `/joy` stream within `joy_timeout_sec`. **Mandatory workspace policy**, not specific to this node — see [docs/architecture.md](../../docs/architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car). `enable_deadman` should stay `true`.
2. **Localization watchdog** — no pose yet, or `pose_topic` stale beyond `pose_timeout_sec` → stop.
3. **Find nearest waypoint** → also yields cross-track error.
4. **Cross-track watchdog** — error beyond `max_cross_track_error` → stop (car is lost/kidnapped/localization has diverged; steering at a stale position estimate is worse than not steering at all).
5. **Steering** — adaptive lookahead sized off the speed *at the car's current position on the line* (not the target's — lookahead should reflect how fast the car is going *right now*), then the Pure Pursuit formulas above.
6. **Speed** — read straight from the profiled speed at the car's current nearest waypoint, clipped to `[min_speed, max_speed]` as a hard ceiling independent of whatever the CSV says.
7. **Opponent tracking + overtake** (if `enable_opponent_overtake`) — look for another car in the scan, track its progress along the racing line, and if closing on it within `overtake_trigger_gap`, replace the steering target with one nudged sideways toward whichever side has more room, via `lateral_offset_point`. See [docs/racing-autonomy.md](../../docs/racing-autonomy.md#racing-against-opponents-detection-tracking-and-overtaking) for the full strategy, and `OpponentTracker` (in `pure_pursuit_node.py`) for the progress-rate estimator behind it.
8. **Reactive LIDAR safety net** (if `enable_lidar_safety`) — two tiers, always the final word regardless of the racing line or any overtake in progress: steer at the best gap (`find_best_gap`) if something is inside `avoidance_trigger_distance` but there's still room (`enable_obstacle_avoidance`), or hard-stop unconditionally if something is inside `emergency_stop_distance`, or if the scan itself is stale/missing (treated identically to "obstacle detected" — a safety net that's gone blind isn't a safety net). This exists for anything **not** in the map — an opponent car, a spun-out car, debris.
9. **Publish.**

## Parameters (`config/pure_pursuit.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `waypoints_file` | *(required)* | Profiled `(x,y,speed)` CSV — node refuses to start without a valid one |
| `closed_loop` | `true` | Whether the racing line wraps around |
| `pose_topic` | `/pf/viz/inferred_pose` | Localization input |
| `scan_topic` / `drive_topic` | `/scan` / `/drive` | LIDAR / output |
| `control_rate_hz` | `40.0` | Control loop frequency |
| `wheelbase` | `0.25` m | Must match `vesc.yaml` |
| `min_lookahead` / `max_lookahead` / `lookahead_speed_gain` | `0.6` / `2.5` / `0.35` | Adaptive lookahead formula above |
| `nearest_search_window` | `40` | ±waypoints searched around last tick's nearest point (`0` = search all) |
| `max_speed` / `min_speed` | `4.0` / `0.5` m/s | Hard safety ceiling/floor, independent of the CSV |
| `max_steering_angle` | `0.26` rad | Derived from this car's real servo limits — see [docs/racing-autonomy.md](../../docs/racing-autonomy.md#where-026-rad-comes-from) |
| `pose_timeout_sec` | `0.5` s | Localization watchdog |
| `max_cross_track_error` | `1.0` m | Lost/kidnapped watchdog |
| `enable_lidar_safety` / `safety_fov_deg` / `emergency_stop_distance` / `scan_timeout_sec` | `true` / `60.0°` / `0.4` m / `0.5` s | Master switch + hard emergency-stop tier |
| `enable_obstacle_avoidance` / `avoidance_fov_deg` / `avoidance_trigger_distance` / `avoidance_min_gap_distance` / `avoidance_speed` | `true` / `100.0°` / `1.5` m / `1.0` m / `1.0` m/s | Steer-around tier — requires `enable_lidar_safety` too |
| `enable_opponent_overtake` | `true` | Master switch for opponent detection/tracking/overtaking — requires `enable_lidar_safety` too |
| `opponent_min_width` / `opponent_max_width` | `0.15` / `0.7` m | Car-shaped cluster width bounds |
| `opponent_cluster_gap` | `0.3` m | Range jump that splits one cluster into two |
| `opponent_engagement_range` | `5.0` m | Ignore detections farther than this |
| `opponent_open_side_margin` | `0.5` m | How much more open the surroundings must be to count as "isolated" |
| `opponent_velocity_smoothing` | `0.3` | Exponential smoothing (0-1) on the tracked progress-rate estimate |
| `opponent_lost_timeout_sec` | `1.0` s | Forget the tracked opponent if not re-detected within this long |
| `overtake_trigger_gap` / `overtake_closing_margin` | `3.0` m / `0.3` m/s | Track-distance + closing-speed thresholds to start a pass |
| `overtake_clear_margin` / `overtake_lateral_offset` | `1.0` m / `0.5` m | Track distance to consider a pass finished / sideways nudge while passing |
| `laser_offset_x` / `laser_offset_y` | `0.27` / `0.0` m | LIDAR mounting offset from `base_link`, used to place opponent detections in the map frame |
| `max_range` | `10.0` m | Range clip ceiling for every LIDAR-based check above (also fills in `inf` returns) |
| `enable_deadman` / `joy_topic` / `deadman_button` / `joy_timeout_sec` | `true` / `/joy` / `4` / `0.5` s | **Mandatory** LB deadman button — do not disable |

## `generate_velocity_profile` CLI flags

| Flag | Default | Meaning |
|---|---|---|
| `--input` / `--output` | *(required)* | Raw `x,y` CSV in, profiled `x,y,speed` CSV out |
| `--open-path` | off (closed loop) | Treat the recording as a single pass instead of a lap |
| `--v-max` / `--v-min` | `6.0` / `0.5` m/s | Absolute speed bounds |
| `--a-lat-max` | `8.0` m/s² | Max cornering acceleration — lower if the car slides mid-corner |
| `--a-accel-max` | `3.0` m/s² | Max forward acceleration the drivetrain can produce |
| `--a-brake-max` | `8.0` m/s² | Max braking deceleration — lower if the car runs wide entering a corner |
| `--smoothing-passes` | `3` | Forward+backward sweep repetitions (closed-loop seam convergence) |

## Tuning: symptom → cause → fix

| Symptom | Cause | Fix |
|---|---|---|
| Oscillates on straights | Lookahead too short | Raise `min_lookahead`/`lookahead_speed_gain` |
| Cuts corners | Lookahead too long | Lower `lookahead_speed_gain`/`max_lookahead` |
| Slides mid-corner | `a_lat_max` above actual grip | Lower `--a-lat-max`, regenerate profile |
| Runs wide exiting a corner | `a_brake_max` above actual capability | Lower `--a-brake-max`, regenerate profile |
| Stops unexpectedly mid-lap | Cross-track watchdog tripped (bad localization seed or genuine drift) | Re-seed "2D Pose Estimate"; only loosen `max_cross_track_error` after confirming localization is healthy |
| Refuses to launch | `waypoints_file` unset/missing, or still a raw (no `speed` column) file | Point at a *profiled* CSV — run `generate_velocity_profile` first |
| Won't move even with LB held | Check the other watchdogs above in order — localization first, then LIDAR staleness | `ros2 topic hz /pf/viz/inferred_pose` and `/scan` |
| Swerves at a wall/curve like it's an opponent | A curving wall segment briefly measured as car-width | Narrow `opponent_min_width`/`opponent_max_width`, or raise `opponent_open_side_margin` |
| Never attempts to overtake a slower car ahead | Not closing fast enough, or opponent not detected | Lower `overtake_closing_margin`; check the opponent is within `opponent_engagement_range` and roughly car-sized in the scan |
| Overtakes then swerves back too early/late | `overtake_clear_margin` mismatched to this car | Raise it if the pass looks unfinished when it ends, lower it if the car lingers off-line too long |

See [docs/racing-autonomy.md](../../docs/racing-autonomy.md) for the full pipeline write-up, `mermaid` diagrams, and the design rationale for choosing Pure Pursuit over a purely reactive or full-MPC approach.
