# `gap_follow`

Reactive "follow-the-gap" autonomy: no map, no localization, no memory of the track — every LIDAR scan is looked at fresh and the car steers into the biggest safe opening it currently sees. This file documents the algorithm and code in detail; for the broader workspace context (safety model, how to run it, how to write your own node) see [docs/architecture.md](../../docs/architecture.md), [docs/operations.md](../../docs/operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node), and [docs/writing-your-own-node.md](../../docs/writing-your-own-node.md).

## Files

| File | What it is |
|---|---|
| [`gap_follow/gap_follow_node.py`](gap_follow/gap_follow_node.py) | ROS orchestration: deadman, odometry watchdog, TTC brake, diagnostics, and drive publisher. |
| [`gap_follow/gap_logic.py`](gap_follow/gap_logic.py) | Unit-tested footprint, TTC, disparity extension, safety bubble, and gap-selection math. |
| [`config/gap_follow.yaml`](config/gap_follow.yaml) | Every tunable parameter, loaded at launch. Change behavior here, not in the code. |
| [`launch/gap_follow_launch.py`](launch/gap_follow_launch.py) | Starts the node with the YAML above as its parameters. |
| `resource/gap_follow` | Empty marker file required by `ament_python` — not code. |

## Interface

- **Subscribes:** `/scan` (`sensor_msgs/LaserScan`), `/odom` (`nav_msgs/Odometry`, for TTC), `/joy` (`sensor_msgs/Joy`, for the deadman button)
- **Publishes:** `/drive` (`ackermann_msgs/AckermannDriveStamped`)

The launch file sends ROS logs to the terminal. Each controller state change is
printed immediately as `STOP [reason]` or `DRIVE [reason]`, including the sensor
measurement and threshold behind the decision plus the final steering/speed
command. An unchanged decision is repeated every `decision_log_period_sec`
(1.0 s by default); set it to `0.0` for transitions only. A small status timer
also reports a missing/stale `/scan`, which otherwise stops this callback-driven
node by letting `/drive` time out silently at the mux.

## The algorithm, step by step

All of this happens in `scan_callback`, once per incoming `LaserScan` message (the LIDAR's native rate).

### 0. Deadman button (checked first)

Before touching the scan at all: if LB (button index `deadman_button`, default `4`) isn't currently held on a live `/joy` stream (received within `joy_timeout_sec`, default `0.5s`), publish `0.0 / 0.0` and return immediately. This is a **mandatory, workspace-wide safety policy** — see [docs/architecture.md](../../docs/architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car) — not specific to this algorithm; every other step below only ever runs while LB is held.

```python
def _deadman_engaged(self) -> bool:
    if not self.enable_deadman:
        return True
    if not self.deadman_held or self.last_joy_time is None:
        return False
    age_sec = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
    return age_sec < self.joy_timeout_sec
```

`enable_deadman` defaults `true` and should stay that way — see the linked policy doc before ever changing it.

### 1. Require live odometry for TTC

The [official F1TENTH automatic-emergency-braking lab](https://github.com/f1tenth/f1tenth_lab2_template) defines instantaneous TTC from LiDAR range and measured longitudinal odometry speed. With `enable_ttc: true`, no drive command is allowed until `/odom` has arrived, and a sample older than `odom_timeout_sec` produces `STOP [odometry_stale]`.

### 2. Sanitize the scan and restrict to a forward field of view

```python
clean, valid = gap_logic.sanitize_ranges(
    scan.ranges, max_range=self.max_range, range_min=scan.range_min)
```

LIDAR scans can contain `NaN`, `inf`, and sub-`range_min` values. Unknown
beams remain non-free (`0.0`) for gap selection and are excluded from
clearance/TTC with `valid`; positive infinity means no return within sensor
range and becomes `max_range` for gap selection. This prevents both steering
into a blind beam and treating a scan dropout as a zero-range collision.

Then the array is sliced down to a forward-facing window (`forward_fov_deg`, default `180°`, i.e. ±90° from straight ahead) via `_fov_indices()`, which converts the angular window into array index bounds using the scan's own `angle_min`/`angle_increment`:

```python
lo_idx = int((lo_angle - scan.angle_min) / scan.angle_increment)
hi_idx = int((hi_angle - scan.angle_min) / scan.angle_increment)
```

This exists so the car never steers toward a "gap" that's actually behind or beside it — LIDAR returns outside the driving direction are simply never considered.

### 3. Check footprint clearance and instantaneous TTC

The collision model is a rectangle around `base_link` (rear axle). The [Traxxas 74276-4 specifications](https://traxxas.com/74276-4-ford-fiesta-st-rally-vxl) are 0.281m wide, 0.535m long, and 0.324m wheelbase; the configured rectangle deliberately remains inflated to 0.31m × 0.58m. `vehicle_boundary_distances()` ray-casts from the estimated LiDAR origin (+0.33m forward, about 0.10m behind the physical nose) to that padded rectangle. Subtracting this per-beam distance from the scan produces clearance from the body rather than from the sensor.

Collision detection has three layers. The all-direction contact floor stops at `emergency_stop_clearance` (0.02m from the body). A separate odometry-independent fallback stops at `forward_stop_clearance` (0.25m) within the narrow `forward_stop_fov_deg` cone (60°, or ±30°); close side walls outside that cone do not trigger it. The speed-aware layer then evaluates every approaching beam:

```text
iTTC = max(0, range - body_boundary) / (v_x * cos(beam_angle))
```

Beams with no positive closing speed have infinite TTC, so a close wall exactly beside the car does not create the old closest-range corner false positive. When fresh odometry is effectively zero (at or below `ttc_command_fallback_max_odom_speed`, 0.10m/s), TTC uses the larger of odometry and the latest drive command when that command is positive and no older than `ttc_command_speed_timeout_sec` (0.5s). Once odometry reports meaningful motion, TTC uses measured speed. This catches a stuck-zero/lagging reading without treating full requested speed as instantaneous in a healthy tight corner. A zero brake command supersedes the prior positive command, preventing stale intent from latching the stop. If minimum TTC is at or below `ttc_threshold_sec` (0.5s), the node publishes zero speed and logs `STOP [ttc_brake]`. Invalid LiDAR beams are excluded from all three checks.

### 4. Extend obstacle edges by the car's physical clearance (disparity extender)

Yes — this implementation includes the standard follow-the-gap **disparity extender**. It finds every sharp range jump between adjacent beams whose size exceeds `disparity_threshold` (default `0.4m`). At each jump, it identifies the nearer side and copies that nearer distance onto the far side for as many beams as `car_width / 2 + safety_margin` subtends at the obstacle's distance:

```python
half_width = car_width / 2.0 + safety_margin
window = gap_logic.disparity_extend(
    window, scan.angle_increment, disparity_threshold, half_width)
```

This models the car's width rather than treating it as a point. The closer the obstacle edge, the more angular beams are extended. The operation only lowers range values (`np.minimum`), so it cannot invent free space.

### 5. Carve a distance-aware "safety bubble" around the closest obstacle

```python
window = gap_logic.safety_bubble(
    window, closest_idx, closest_dist, scan.angle_increment, half_width)
```

The same `car_width / 2 + safety_margin` clearance is converted to an angle using the closest obstacle's actual distance (`atan2(clearance, distance)`) and zeroed. Unlike a fixed-angle bubble, this demands more angular clearance close to the car and less at a distance. Together with disparity extension, it prevents the selected gap from grazing obstacle edges.

### 6. Pick a preferred gap, with a tight-corner fallback

```python
free = window > min_gap_distance
```

A preferred gap is a contiguous run deeper than `min_gap_distance` (2.0m in the physical config). `find_best_gap()` scores each run — **not by angular width alone**:

```python
def score(run):
    start, end = run
    segment = window[start:end + 1]
    width = end - start + 1
    avg_depth = float(np.mean(segment))
    return width * avg_depth
```

$$\text{score} = \text{width} \times \overline{\text{depth}}$$

**Why not just pick the widest gap?** A shallow dead end can subtend a wider angle than a genuinely open corridor. Scoring by `width × average_depth` rewards useful depth.

The former implementation stopped whenever no run remained continuously deeper than 2.0m. A blind 90-degree corner can be physically wide enough while hiding everything beyond the turn, so that rule created `no_safe_gap` immediately before turn-in. The new second pass accepts `fallback_min_gap_distance` (0.8m) and caps speed at `corner_speed` (0.5m/s). It still uses the inflated scan and `min_centerline_gap_width`, so a boxed-in scene still stops.

Obstacle inflation has already removed `car_width / 2 + safety_margin` from both sides of every edge. The old candidate filter required another `car_width + safety_margin` after inflation, effectively demanding roughly a 0.9m raw opening for a 0.31m car. `min_centerline_gap_width` now checks only the small corridor remaining for candidate center points, eliminating that double-padding.

### 7. Steer at the middle of the winning gap

```python
target_idx_in_window = (gap_start + gap_end) // 2
target_idx = lo_idx + target_idx_in_window
steering_angle = scan.angle_min + target_idx * scan.angle_increment
steering_angle = float(np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle))
```

The steering angle is simply the LIDAR bearing of the midpoint of the chosen gap, converted from an array index back into radians via the scan's own `angle_min`/`angle_increment` — the inverse of the conversion in step 1 — then clipped to what the servo can physically achieve (`max_steering_angle`, default `0.4189 rad` ≈ 24°; see [docs/racing-autonomy.md](../../docs/racing-autonomy.md#where-026-rad-comes-from) for how this car's actual servo limits were derived, though `gap_follow`'s default here is less conservative than `pure_pursuit`'s).

### 8. Speed: slow down proportionally to how hard you're turning

```python
speed_scale = 1.0 - (abs(steering_angle) / self.max_steering_angle)
speed = self.min_speed + speed_scale * (self.max_speed - self.min_speed)
```

$$v = v_{min} + \left(1 - \frac{|\delta|}{\delta_{max}}\right)(v_{max} - v_{min})$$

Straight ahead ($\delta = 0$) drives at `max_speed`; steering at the full clamp drives at `min_speed`; everything in between scales linearly. A gap found only by the tight-corner fallback is additionally capped at `corner_speed`.

## Parameters (`config/gap_follow.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `scan_topic` / `drive_topic` / `odom_topic` | `/scan` / `/drive` / `/odom` | Sensor, command, and measured-speed topics |
| `max_range` / `forward_fov_deg` | `10.0m` / `180°` | Scan clipping and planning window |
| `car_width` / `car_length` | `0.31` / `0.58` m | Deliberately padded from the Traxxas body (0.281 × 0.535m) |
| `wheelbase` | `0.324` m | Published Traxxas rear-to-front axle distance; also centers the padded body from rear-axle `base_link` |
| `laser_offset_x` / `laser_offset_y` | `0.33` / `0.0` m | Estimated LiDAR origin relative to `base_link`; measure x to finalize |
| `safety_margin` / `disparity_threshold` | `0.10` / `0.4` m | Edge inflation clearance and range-jump threshold |
| `min_centerline_gap_width` | `0.10` m | Minimum center corridor remaining after obstacle inflation |
| `min_gap_distance` / `fallback_min_gap_distance` | `2.0` / `0.8` m | Preferred depth and tight-corner fallback depth |
| `max_speed` / `min_speed` / `corner_speed` | `2.0` / `0.8` / `0.5` m/s | Normal speed range and fallback cap |
| `max_steering_angle` | `0.4189` rad (~24°) | Hard command clamp |
| `emergency_stop_clearance` | `0.02` m | All-direction final contact floor measured from the padded body |
| `forward_stop_clearance` / `forward_stop_fov_deg` | `0.25m` / `60°` | Odom-independent forward-cone braking fallback |
| `enable_ttc` / `ttc_threshold_sec` | `true` / `0.5s` | Enable iTTC braking and set its trigger |
| `ttc_min_closing_speed` / `odom_timeout_sec` | `0.05m/s` / `0.5s` | Ignore negligible closing rates; fail closed on stale odometry |
| `ttc_command_speed_timeout_sec` | `0.5s` | Freshness limit for a latest positive command used as TTC backup |
| `ttc_command_fallback_max_odom_speed` | `0.10m/s` | Use command backup only while fresh odometry is effectively zero |
| `joy_topic` / `deadman_button` / `joy_timeout_sec` | `/joy` / `4` / `0.5s` | Mandatory LB deadman input |
| `enable_deadman` | `true` | **Do not disable** — see the workspace policy link above |
| `decision_log_period_sec` / `scan_timeout_sec` | `1.0` / `0.5s` | Decision-log repeat and scan-staleness diagnostics |

## Simulator validation

The exact `gap_logic` pipeline above is exercised in the official F1TENTH Gym,
including the footprint clearance, TTC brake, corner fallback, disparity
extender, noisy LiDAR, vehicle dynamics, and collision model. On the current
deterministic matrix it completed
Spielberg, Silverstone, and Brands Hatch without a collision. The full setup,
commands, metrics, and checked-in JSON report are in
[docs/simulator.md](../../docs/simulator.md).

## Tuning notes

- **Fallback enters an alcove it should reject:** raise `fallback_min_gap_distance` or lower `corner_speed`; keep the preferred 2.0m threshold for normal selection.
- **Car still refuses a real tight corner:** inspect the logged `STOP` reason first. For `no_safe_gap`, cautiously lower `fallback_min_gap_distance` or `min_centerline_gap_width`. Never reduce `car_width` below the measured body width.
- **`forward_clearance` brakes at a corner:** verify the LiDAR transform first, then cautiously narrow `forward_stop_fov_deg`; do not reduce the padded vehicle dimensions.
- **TTC brakes too early/late:** compare logged odometry and recent-command speeds, then validate odometry scale and the footprint/LiDAR offsets before tuning `ttc_threshold_sec`. Do not compensate for wrong geometry by lowering the threshold.
- **Disparity extension is too aggressive or misses obstacle edges:** lower `disparity_threshold` to detect smaller range jumps, or raise it to ignore more scan noise.
- **Car oscillates rapidly between two nearby gaps:** this implementation has no gap "memory"/hysteresis between scans — each `LaserScan` is scored completely independently. If this becomes a real problem, the fix is adding a bias term favoring the previous tick's chosen gap, which doesn't exist here today.
- **Speed always feels too conservative/too aggressive:** the `speed_scale` formula is linear and only looks at the *chosen* steering angle, not any measure of how open the track actually is — retune `max_speed`/`min_speed` directly rather than expecting curvature-aware pacing like `pure_pursuit` has.
- Change one parameter at a time and re-test wheels-off-ground (see [docs/writing-your-own-node.md](../../docs/writing-your-own-node.md#testing-before-its-on-wheels)) — the interactions between the physical-clearance parameters, `min_gap_distance`, and `forward_fov_deg` are not always intuitive.
