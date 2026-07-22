# `gap_follow`

Reactive "follow-the-gap" autonomy: no map, no localization, no memory of the track — every LIDAR scan is looked at fresh and the car steers into the biggest safe opening it currently sees. This file documents the algorithm and code in detail; for the broader workspace context (safety model, how to run it, how to write your own node) see [docs/architecture.md](../../docs/architecture.md), [docs/operations.md](../../docs/operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node), and [docs/writing-your-own-node.md](../../docs/writing-your-own-node.md).

## Files

| File | What it is |
|---|---|
| [`gap_follow/gap_follow_node.py`](gap_follow/gap_follow_node.py) | The ROS node and pipeline orchestration, including the deadman gate. |
| [`gap_follow/gap_logic.py`](gap_follow/gap_logic.py) | Importable, unit-tested scan-processing math, including disparity extension, the safety bubble, and gap scoring. |
| [`config/gap_follow.yaml`](config/gap_follow.yaml) | Every tunable parameter, loaded at launch. Change behavior here, not in the code. |
| [`launch/gap_follow_launch.py`](launch/gap_follow_launch.py) | Starts the node with the YAML above as its parameters. |
| `resource/gap_follow` | Empty marker file required by `ament_python` — not code. |

## Interface

- **Subscribes:** `/scan` (`sensor_msgs/LaserScan`), `/joy` (`sensor_msgs/Joy`, for the deadman button — see below)
- **Publishes:** `/drive` (`ackermann_msgs/AckermannDriveStamped`)

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

### 1. Sanitize the scan and restrict to a forward field of view

```python
ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
ranges = np.clip(ranges, 0.0, self.max_range)
```

LIDAR scans can contain `NaN`/`inf` for out-of-range or invalid returns; both get mapped to sane finite values before any math touches them (a `NaN` propagating into `np.argmin`/comparisons would silently corrupt the result).

Then the array is sliced down to a forward-facing window (`forward_fov_deg`, default `180°`, i.e. ±90° from straight ahead) via `_fov_indices()`, which converts the angular window into array index bounds using the scan's own `angle_min`/`angle_increment`:

```python
lo_idx = int((lo_angle - scan.angle_min) / scan.angle_increment)
hi_idx = int((hi_angle - scan.angle_min) / scan.angle_increment)
```

This exists so the car never steers toward a "gap" that's actually behind or beside it — LIDAR returns outside the driving direction are simply never considered.

### 2. Find the closest obstacle; emergency-stop if it's too close

```python
closest_idx = int(np.argmin(window))
closest_dist = float(window[closest_idx])
if closest_dist < self.emergency_stop_distance:
    self._publish_drive(0.0, 0.0)
    return
```

A hard, unconditional floor (`emergency_stop_distance`, default `0.15m`) — no gap-finding logic runs at all if something is this close; the car simply stops.

### 3. Extend obstacle edges by the car's physical clearance (disparity extender)

Yes — this implementation includes the standard follow-the-gap **disparity extender**. It finds every sharp range jump between adjacent beams whose size exceeds `disparity_threshold` (default `0.4m`). At each jump, it identifies the nearer side and copies that nearer distance onto the far side for as many beams as `car_width / 2 + safety_margin` subtends at the obstacle's distance:

```python
half_width = car_width / 2.0 + safety_margin
window = gap_logic.disparity_extend(
    window, scan.angle_increment, disparity_threshold, half_width)
```

This models the car's width rather than treating it as a point. The closer the obstacle edge, the more angular beams are extended. The operation only lowers range values (`np.minimum`), so it cannot invent free space.

### 4. Carve a distance-aware "safety bubble" around the closest obstacle

```python
window = gap_logic.safety_bubble(
    window, closest_idx, closest_dist, scan.angle_increment, half_width)
```

The same `car_width / 2 + safety_margin` clearance is converted to an angle using the closest obstacle's actual distance (`atan2(clearance, distance)`) and zeroed. Unlike a fixed-angle bubble, this demands more angular clearance close to the car and less at a distance. Together with disparity extension, it prevents the selected gap from grazing obstacle edges.

### 5. Find every candidate gap, score them, pick the best one

```python
free = window > min_gap_distance
```

A "gap" is a contiguous run of scan points all farther than `min_gap_distance` (default `2.0m`). `_best_gap()` finds every such run, then scores each one — **not by angular width alone**:

```python
def score(run):
    start, end = run
    segment = window[start:end + 1]
    width = end - start + 1
    avg_depth = float(np.mean(segment))
    return width * avg_depth
```

$$\text{score} = \text{width} \times \overline{\text{depth}}$$

**Why not just pick the widest gap?** A shallow dead end — say a doorway-sized alcove only 1m deep — can subtend a *wider* angle than a genuinely open corridor or track section that's angularly narrower but far deeper. Scoring by `width × average_depth` means a gap has to actually stay open for a while, not just be wide at its mouth, to win. This is the single biggest behavioral difference between this implementation and a textbook "pick the widest gap" follow-the-gap.

### 6. Steer at the middle of the winning gap

```python
target_idx_in_window = (gap_start + gap_end) // 2
target_idx = lo_idx + target_idx_in_window
steering_angle = scan.angle_min + target_idx * scan.angle_increment
steering_angle = float(np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle))
```

The steering angle is simply the LIDAR bearing of the midpoint of the chosen gap, converted from an array index back into radians via the scan's own `angle_min`/`angle_increment` — the inverse of the conversion in step 1 — then clipped to what the servo can physically achieve (`max_steering_angle`, default `0.4189 rad` ≈ 24°; see [docs/racing-autonomy.md](../../docs/racing-autonomy.md#where-026-rad-comes-from) for how this car's actual servo limits were derived, though `gap_follow`'s default here is less conservative than `pure_pursuit`'s).

### 7. Speed: slow down proportionally to how hard you're turning

```python
speed_scale = 1.0 - (abs(steering_angle) / self.max_steering_angle)
speed = self.min_speed + speed_scale * (self.max_speed - self.min_speed)
```

$$v = v_{min} + \left(1 - \frac{|\delta|}{\delta_{max}}\right)(v_{max} - v_{min})$$

Straight ahead ($\delta = 0$) drives at `max_speed`; steering at the full clamp drives at `min_speed`; everything in between scales linearly. There's no explicit curvature/lookahead model here (unlike `pure_pursuit`) — this is a simple, cheap proxy for "the harder I'm turning, the more likely I'm near something," appropriate for a purely reactive controller with no map to plan braking zones against.

## Parameters (`config/gap_follow.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `scan_topic` / `drive_topic` | `/scan` / `/drive` | Topics |
| `max_range` | `10.0` m | Range clip ceiling (also fills in `inf` returns) |
| `forward_fov_deg` | `180.0°` | Total forward field of view considered |
| `car_width` | `0.30` m | Chassis width used by the physical-clearance model |
| `safety_margin` | `0.10` m | Extra clearance added to each side of the half-width |
| `disparity_threshold` | `0.4` m | Minimum adjacent-range jump treated as an obstacle edge |
| `min_gap_distance` | `2.0` m | Minimum depth for a run of scan points to count as a "gap" |
| `max_speed` / `min_speed` | `2.0` / `0.8` m/s | Speed range, scaled by steering angle |
| `max_steering_angle` | `0.4189` rad (~24°) | Hard clamp on commanded steering |
| `emergency_stop_distance` | `0.15` m | Unconditional hard-stop floor |
| `joy_topic` | `/joy` | Deadman button input |
| `deadman_button` | `4` | Button index (LB on the F710 in XInput mode) |
| `joy_timeout_sec` | `0.5` s | Deadman button staleness watchdog |
| `enable_deadman` | `true` | **Do not disable** — see the workspace policy link above |

## Simulator validation

The exact `gap_logic` pipeline above is exercised in the official F1TENTH Gym,
including the disparity extender, width-aware bubble, noisy LiDAR, vehicle
dynamics, and collision model. On the current deterministic matrix it completed
Spielberg, Silverstone, and Brands Hatch without a collision. The full setup,
commands, metrics, and checked-in JSON report are in
[docs/simulator.md](../../docs/simulator.md).

## Tuning notes

- **Car cuts into shallow pockets it shouldn't:** raise `min_gap_distance` so shallower alcoves stop qualifying as candidate gaps at all.
- **Car won't take an opening it should be able to fit through:** lower `safety_margin` cautiously (less clearance demanded around obstacle edges), reduce `car_width` only if the configured chassis width is wrong, or lower `min_gap_distance` to accept shallower gaps. `car_width + safety_margin` is also the minimum physical gap width used during candidate filtering.
- **Disparity extension is too aggressive or misses obstacle edges:** lower `disparity_threshold` to detect smaller range jumps, or raise it to ignore more scan noise.
- **Car oscillates rapidly between two nearby gaps:** this implementation has no gap "memory"/hysteresis between scans — each `LaserScan` is scored completely independently. If this becomes a real problem, the fix is adding a bias term favoring the previous tick's chosen gap, which doesn't exist here today.
- **Speed always feels too conservative/too aggressive:** the `speed_scale` formula is linear and only looks at the *chosen* steering angle, not any measure of how open the track actually is — retune `max_speed`/`min_speed` directly rather than expecting curvature-aware pacing like `pure_pursuit` has.
- Change one parameter at a time and re-test wheels-off-ground (see [docs/writing-your-own-node.md](../../docs/writing-your-own-node.md#testing-before-its-on-wheels)) — the interactions between the physical-clearance parameters, `min_gap_distance`, and `forward_fov_deg` are not always intuitive.
