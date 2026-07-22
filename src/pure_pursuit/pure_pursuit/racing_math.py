"""
racing_math.py

Pure, dependency-light math for the pure-pursuit race stack. Every
function in this file takes plain numbers/arrays in and returns plain
numbers/arrays out -- nothing here imports rclpy or talks to ROS topics.

That is deliberate. It means every formula actually used to steer and
pace the car can be unit-tested in complete isolation (see
test/test_racing_math.py) without a running robot, a simulator, or even
ROS installed -- and it means you can read this one file top to bottom
to understand the *entire* algorithm, with the ROS plumbing kept
completely separate in pure_pursuit_node.py / waypoint_recorder_node.py /
generate_velocity_profile.py.

See docs/racing-autonomy.md for the full write-up with derivations and
diagrams. Short version of what lives in each section below:

  1. Frame conversions   -- world (map) frame  <->  car body frame
  2. Pure pursuit         -- the steering-geometry formula itself
  3. Path indexing        -- "where am I on the line" / "where should I aim"
  4. Raceline generation  -- curvature -> cornering speed -> smoothed profile
  5. CSV I/O              -- reading/writing waypoint files
  6. Reactive avoidance   -- gap-finding, for anything not in the map
  7. Opponent racing      -- detect another car, track it, decide/steer an overtake
"""

import csv
import math

import numpy as np


# ============================================================================
# 1. Frame conversions
# ============================================================================

def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (rotation about +Z) from a geometry_msgs Quaternion.

    The car only ever needs its heading in the 2D ground plane -- roll and
    pitch are irrelevant to steering a car that can't leave the ground --
    so this is the standard atan2-based yaw-only extraction from a unit
    quaternion, not a full 3D Euler decomposition:

        yaw = atan2( 2*(w*z + x*y), 1 - 2*(y^2 + z^2) )
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_body(dx: float, dy: float, yaw: float):
    """Rotate a world/map-frame vector (dx, dy) into the car's body frame.

    `yaw` is the car's current heading in the map frame. The body frame
    follows REP-103 (x out the nose, y to the left). Rotating a world
    vector into body coordinates means rotating it by -yaw:

        x_body =  cos(yaw)*dx + sin(yaw)*dy
        y_body = -sin(yaw)*dx + cos(yaw)*dy

    Sanity check: car heading along world +X (yaw=0), point at world +Y
    (i.e. to the car's left): dx=0, dy>0 -> y_body = dy > 0. Left stays
    left. Good.
    """
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_body = cos_yaw * dx + sin_yaw * dy
    y_body = -sin_yaw * dx + cos_yaw * dy
    return x_body, y_body


# ============================================================================
# 2. Pure pursuit steering geometry
# ============================================================================

def steering_arc_curvature(x_body: float, y_body: float) -> float:
    """The Pure Pursuit formula. Curvature of the arc from the car's rear
    axle (the origin, in body-frame coordinates) to a target point
    (x_body, y_body) ahead of it, that is tangent to the car's current
    heading (the body-frame x-axis).

    Derivation sketch: a circle tangent to the x-axis at the origin has
    its center at (0, R) for some radius R (positive R = center to the
    left = a left turn). For such a circle to also pass through
    (x_body, y_body):

        x_body^2 + (y_body - R)^2 = R^2
        x_body^2 + y_body^2 - 2*R*y_body = 0
        R = (x_body^2 + y_body^2) / (2 * y_body)

    Curvature kappa = 1/R, so:

        kappa = 2 * y_body / (x_body^2 + y_body^2)

    Using the *actual* squared distance to the target rather than the
    nominal lookahead distance parameter keeps this exact even though the
    chosen waypoint is essentially never sitting at precisely the
    requested lookahead distance (see find_lookahead_index). Positive
    y_body (target to the left) gives positive curvature -> a left turn,
    matching AckermannDriveStamped's "positive steering_angle = left"
    convention.
    """
    dist_sq = x_body * x_body + y_body * y_body
    if dist_sq < 1e-9:
        # Target essentially on top of the car -- no meaningful direction
        # to curve toward. Drive straight rather than divide by ~0.
        return 0.0
    return 2.0 * y_body / dist_sq


def steering_from_curvature(kappa: float, wheelbase: float) -> float:
    """Bicycle-model steering angle that produces a given path curvature.

    Collapsing the car's front and rear wheel pairs to a single front and
    single rear "wheel" (the standard car-like-robot bicycle model), a
    vehicle with wheelbase L needs a front steer angle:

        delta = atan(L * kappa)

    to drive an arc of curvature kappa.
    """
    return math.atan(wheelbase * kappa)


def adaptive_lookahead(speed: float, gain: float, min_lookahead: float, max_lookahead: float) -> float:
    """Speed-scaled lookahead distance: L_d = clip(gain*speed + min, min, max).

    A *fixed* lookahead distance is a bad compromise: short enough to
    corner tightly at parking-lot speed and the car oscillates/overshoots
    at race speed; long enough to be smooth at race speed and it cuts
    corners at low speed. Scaling lookahead with the current speed (a
    standard, well-known extension to textbook Pure Pursuit) fixes both
    problems with one parameter (`gain`, meters of extra lookahead per
    m/s of speed).
    """
    ld = gain * abs(speed) + min_lookahead
    return float(np.clip(ld, min_lookahead, max_lookahead))


# ============================================================================
# 3. Path indexing
# ============================================================================

def compute_segment_lengths(xy: np.ndarray, closed: bool = True) -> np.ndarray:
    """Euclidean distance from each waypoint to the next one.

    seg_len[i] = |xy[i+1] - xy[i]|, wrapping xy[-1] -> xy[0] if the track
    is a closed loop. For an open path, seg_len[-1] is set to 0 since
    there is no "next" waypoint after the last one.
    """
    nxt = np.roll(xy, -1, axis=0)
    seg_len = np.hypot(nxt[:, 0] - xy[:, 0], nxt[:, 1] - xy[:, 1])
    if not closed:
        seg_len[-1] = 0.0
    return seg_len


def find_nearest_index(xy: np.ndarray, car_xy, closed: bool = True,
                        prev_index=None, search_window=None):
    """Index of the waypoint closest to the car, and the distance to it
    (the car's instantaneous cross-track error).

    If `prev_index` and a positive `search_window` are both given, the
    search is restricted to the +/- search_window waypoints around the
    previous tick's result (wrapping around the array if `closed`), for
    two reasons:

      1. Speed: O(window) instead of O(N) every control tick.
      2. Correctness: on a track that comes close to itself (a hairpin, a
         figure-eight, a pit-lane split), the *globally* closest waypoint
         can be on a completely different part of the track than the one
         the car is actually on. Searching only near where the car was
         last tick keeps it locked onto the correct branch instead of
         "teleporting" its target to the wrong side of the track.

    Falls back to a full-array search on the very first call (no
    `prev_index` yet) or when `search_window` disables windowing.
    """
    n = len(xy)
    if prev_index is None or not search_window or search_window <= 0 or search_window >= n:
        candidate_idx = np.arange(n)
    else:
        offsets = np.arange(-search_window, search_window + 1)
        candidate_idx = prev_index + offsets
        if closed:
            candidate_idx = np.mod(candidate_idx, n)
        else:
            candidate_idx = candidate_idx[(candidate_idx >= 0) & (candidate_idx < n)]

    deltas = xy[candidate_idx] - np.asarray(car_xy)
    dists = np.hypot(deltas[:, 0], deltas[:, 1])
    best_local = int(np.argmin(dists))
    return int(candidate_idx[best_local]), float(dists[best_local])


def find_lookahead_index(seg_len: np.ndarray, nearest_index: int, lookahead_dist: float,
                          closed: bool = True) -> int:
    """Walk forward along the path from `nearest_index`, accumulating
    segment lengths, and return the index reached once >= lookahead_dist
    has been covered.

    This is a simple, robust stand-in for the textbook description of
    Pure Pursuit target selection ("intersect the path with a circle of
    radius lookahead_dist centered on the car"). Snapping to the next
    recorded waypoint instead of solving that circle-line intersection
    exactly is less precise, but trivial to get right, and the error it
    introduces is bounded by the spacing between recorded waypoints --
    keep that spacing small (waypoint_recorder_node's default is 0.15m)
    and the difference is negligible.
    """
    n = len(seg_len)
    acc = 0.0
    idx = nearest_index
    for _ in range(n):
        acc += seg_len[idx]
        nxt = idx + 1
        if nxt >= n:
            if not closed:
                return n - 1
            nxt = 0
        if acc >= lookahead_dist:
            return nxt
        idx = nxt
    # lookahead_dist is longer than the entire path (a very short/degenerate
    # recording, or badly misconfigured lookahead) -- aim at the farthest
    # point reachable rather than raising mid-race.
    return idx


# ============================================================================
# 4. Offline raceline generation
# ============================================================================

def smooth_path(xy: np.ndarray, half_window: int, closed: bool = True) -> np.ndarray:
    """Moving-average smoothing of a waypoint path: each point becomes the
    mean of itself and `half_window` neighbors on each side.

    The recorded waypoints come from particle-filter poses sampled every
    ~0.15m, and localization jitter of even a couple of centimeters at
    that spacing reads as *curvature* to estimate_path_curvature -- three
    nearly-colinear points wiggled slightly produce a spuriously tight
    circumcircle. That fake curvature then flows straight into
    compute_velocity_profile as phantom braking zones in the middle of
    straights. Averaging out the jitter before estimating curvature is
    the single highest-leverage cleanup on profile quality.

    For a closed loop the window wraps around the start/finish seam; for
    an open path the ends are averaged over whatever neighbors actually
    exist (the window shrinks at the boundaries rather than wrapping).
    `half_window <= 0` returns the input unchanged. Smoothing pulls
    corners slightly inward (toward the chord), which shortens and
    *tightens* the line a little -- a conservative direction for the
    velocity profile to err in.
    """
    n = len(xy)
    if half_window <= 0 or n < 3:
        return np.asarray(xy, dtype=np.float64).copy()
    half_window = min(half_window, (n - 1) // 2)

    kernel_size = 2 * half_window + 1
    if closed:
        padded = np.vstack([xy[-half_window:], xy, xy[:half_window]])
        cumsum = np.cumsum(np.vstack([[[0.0, 0.0]], padded]), axis=0)
        return (cumsum[kernel_size:] - cumsum[:-kernel_size]) / kernel_size

    smoothed = np.empty_like(xy, dtype=np.float64)
    for i in range(n):
        lo = max(0, i - half_window)
        hi = min(n, i + half_window + 1)
        smoothed[i] = xy[lo:hi].mean(axis=0)
    return smoothed


def estimate_path_curvature(xy: np.ndarray, closed: bool = True) -> np.ndarray:
    """Per-waypoint path curvature via the Menger curvature of each point
    and its immediate neighbors -- no calculus or spline-fitting needed.

    For three points A, B, C, the (unique) circle passing through all
    three has radius:

        R = (|AB| * |BC| * |CA|) / (4 * area(A,B,C))

    (from the identity area = a*b*c / (4R) for a triangle with sides
    a,b,c and circumradius R). Curvature is 1/R:

        kappa = 4 * area(A,B,C) / (|AB| * |BC| * |CA|)

    Tight corners -> small R -> large kappa. Straights (A,B,C nearly
    colinear) -> area ~ 0 -> kappa ~ 0. Using each point's immediate
    neighbors (A=previous waypoint, B=this waypoint, C=next waypoint)
    means this works directly on raw recorded waypoints.

    For an open (non-closed) path the first and last points have no
    "previous"/"next" neighbor respectively; their curvature is left at 0
    (irrelevant in practice -- real tracks are closed loops, which is the
    default).
    """
    n = len(xy)
    prev_pts = np.roll(xy, 1, axis=0)
    next_pts = np.roll(xy, -1, axis=0)

    if not closed:
        prev_pts = prev_pts.copy()
        next_pts = next_pts.copy()
        prev_pts[0] = xy[0]
        next_pts[-1] = xy[-1]

    a = np.hypot(next_pts[:, 0] - xy[:, 0], next_pts[:, 1] - xy[:, 1])              # |B->C|
    b = np.hypot(xy[:, 0] - prev_pts[:, 0], xy[:, 1] - prev_pts[:, 1])              # |A->B|
    c = np.hypot(next_pts[:, 0] - prev_pts[:, 0], next_pts[:, 1] - prev_pts[:, 1])  # |A->C|

    # Twice the *signed* area of triangle ABC via the shoelace/cross-product
    # formula; abs() + 0.5 gives the true (unsigned) area.
    cross = ((xy[:, 0] - prev_pts[:, 0]) * (next_pts[:, 1] - prev_pts[:, 1]) -
             (xy[:, 1] - prev_pts[:, 1]) * (next_pts[:, 0] - prev_pts[:, 0]))
    area = 0.5 * np.abs(cross)

    denom = a * b * c
    kappa = np.zeros(n)
    valid = denom > 1e-9
    kappa[valid] = 4.0 * area[valid] / denom[valid]
    return kappa


def compute_velocity_profile(seg_len: np.ndarray, curvature: np.ndarray,
                              v_max: float, v_min: float,
                              a_lat_max: float, a_accel_max: float, a_brake_max: float,
                              closed: bool = True, smoothing_passes: int = 5,
                              friction_ellipse: bool = True) -> np.ndarray:
    """Turn per-waypoint path curvature into a per-waypoint target speed.
    This is the core of "how fast should the car go here" -- three passes,
    each a standard, well-established piece of racing-line theory:

    1. Cornering limit (simplified friction circle, lateral-only): taking
       a corner of curvature kappa at speed v produces lateral
       acceleration a_lat = v^2 * kappa (uniform circular motion). Solving
       for the fastest speed that keeps a_lat under the tire/chassis limit
       a_lat_max:

           v_corner = sqrt(a_lat_max / kappa)

    2. Forward (acceleration) pass: the car can only speed up as fast as
       a_accel_max allows between two waypoints ds apart:

           v_i <= sqrt(v_(i-1)^2 + 2 * a_accel_max * ds)

       Sweeping the array forward and capping each point at this limits
       how quickly the profile is allowed to ramp speed up, e.g. coming
       out of a hairpin onto a straight.

    3. Backward (braking) pass: symmetric to (2), using a_brake_max, swept
       from the end of the array backward. This is the pass that actually
       creates *braking zones*: it propagates a low corner-speed limit
       backward along the straight leading into that corner, so the
       profile tells the car to start slowing down early enough to
       actually make the corner, instead of arriving too fast and only
       "finding out" at the apex.

    A closed loop has no single first point to seed sweep (1)/(2) from --
    index 0's "previous" point is the *last* index, whose value hasn't
    been finalized yet on the first sweep. Repeating the forward+backward
    pair (`smoothing_passes` times, default 5) lets that start/finish seam
    converge. Both passes only ever lower values (never raise them), so
    repeating them on an already-converged open path is a harmless no-op
    -- there is no need to special-case open vs. closed here.

    With `friction_ellipse` (default on), passes (2)/(3) don't get the
    tire's *full* longitudinal budget while the car is also cornering:
    grip is one shared resource, and lateral acceleration spends it too.
    The available longitudinal fraction follows the standard friction
    ellipse -- (a_long/a_long_max)^2 + (a_lat/a_lat_max)^2 <= 1, so

        a_long_available = a_long_max * sqrt(1 - (v^2*kappa / a_lat_max)^2)

    evaluated at the neighbor whose speed is already known in each sweep.
    On a straight (kappa ~ 0) this is the full budget; at a corner already
    at the cornering limit it's zero (all grip is lateral). The effect is
    strictly-lower (more conservative) speeds around corner entry/exit --
    honest about physics the uncoupled version quietly ignores. Pass
    `friction_ellipse=False` for the old lateral/longitudinal-independent
    behavior (useful for comparing profiles).

    This is *not* a time-optimal racing line -- true time-optimality needs
    a nonlinear optimizer over both the path geometry and the speed
    simultaneously (e.g. minimum-curvature QP solvers like TUM's
    global_racetrajectory_optimization). This is a fast, dependency-light,
    easy-to-verify approximation that is nonetheless the same "cornering
    speed + forward/backward smoothing" technique used in mainstream
    racing-line tooling -- it just doesn't also re-optimize the path
    geometry itself. See docs/racing-autonomy.md for more on this
    trade-off and how to go further.
    """
    n = len(curvature)
    eps = 1e-6
    v_corner = np.sqrt(a_lat_max / np.maximum(curvature, eps))
    v = np.minimum(v_corner, v_max).astype(float)

    def long_fraction(speed: float, kappa: float) -> float:
        # Fraction of the longitudinal budget left after cornering at
        # `speed` through curvature `kappa` (friction ellipse). v_corner
        # already caps a_lat at a_lat_max, so the sqrt argument only goes
        # negative through float round-off -- clamp at 0.
        if not friction_ellipse:
            return 1.0
        lat_fraction = (speed * speed * kappa) / a_lat_max
        return math.sqrt(max(0.0, 1.0 - lat_fraction * lat_fraction))

    for _ in range(max(1, smoothing_passes)):
        # Forward / acceleration-limited sweep.
        fwd_order = range(n) if closed else range(1, n)
        for i in fwd_order:
            prev_i = (i - 1) % n
            a_avail = a_accel_max * long_fraction(v[prev_i], curvature[prev_i])
            v_cap = math.sqrt(v[prev_i] ** 2 + 2.0 * a_avail * seg_len[prev_i])
            if v_cap < v[i]:
                v[i] = v_cap

        # Backward / braking-limited sweep.
        bwd_order = range(n - 1, -1, -1) if closed else range(n - 2, -1, -1)
        for i in bwd_order:
            next_i = (i + 1) % n
            a_avail = a_brake_max * long_fraction(v[next_i], curvature[next_i])
            v_cap = math.sqrt(v[next_i] ** 2 + 2.0 * a_avail * seg_len[i])
            if v_cap < v[i]:
                v[i] = v_cap

    return np.clip(v, v_min, v_max)


def estimate_lap_time(seg_len: np.ndarray, speed: np.ndarray, closed: bool = True) -> float:
    """Rough lap/pass time estimate: sum, over every segment, of (segment
    length / average speed across that segment).

    This is a simple kinematic estimate from the speed profile alone -- it
    assumes the car can actually *hit* this speed profile exactly, which a
    real controller only approximates. Treat it as a way to compare two
    tunings against each other, not as a lap-time guarantee.
    """
    n = len(speed)
    count = n if closed else n - 1
    total = 0.0
    for i in range(count):
        j = (i + 1) % n
        avg_speed = 0.5 * (speed[i] + speed[j])
        if avg_speed > 1e-6:
            total += seg_len[i] / avg_speed
    return total


# ============================================================================
# 5. CSV file I/O
# ============================================================================
#
# Two file "shapes" are used across this package:
#   raw waypoints      -- header "x,y"       -- written by waypoint_recorder_node
#   profiled waypoints -- header "x,y,speed" -- written by generate_velocity_profile,
#                                                read by pure_pursuit_node

def load_xy_csv(path: str) -> np.ndarray:
    """Load a raw (x, y) waypoints CSV into an (N, 2) array."""
    rows = []
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        next(reader, None)  # header
        for row in reader:
            if not row:
                continue
            rows.append((float(row[0]), float(row[1])))
    return np.array(rows, dtype=np.float64)


def save_xy_csv(path: str, xy: np.ndarray) -> None:
    """Write an (N, 2) array of (x, y) points as a raw waypoints CSV."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y'])
        for x, y in xy:
            writer.writerow([f'{x:.4f}', f'{y:.4f}'])


def load_profiled_csv(path: str):
    """Load a profiled (x, y, speed) waypoints CSV.

    Returns (xy, speed): xy is an (N, 2) array, speed is an (N,) array.
    Raises ValueError if the file doesn't look like a profiled file (i.e.
    is missing the speed column) -- most likely a raw recording that
    hasn't been run through generate_velocity_profile yet.
    """
    rows = []
    speeds = []
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None or len(header) < 3:
            raise ValueError(
                f"'{path}' does not look like a profiled waypoints file "
                f"(expected a header with 3 columns, 'x,y,speed'). "
                f"Run generate_velocity_profile on it first."
            )
        for row in reader:
            if not row:
                continue
            rows.append((float(row[0]), float(row[1])))
            speeds.append(float(row[2]))
    return np.array(rows, dtype=np.float64), np.array(speeds, dtype=np.float64)


def save_profiled_csv(path: str, xy: np.ndarray, speed: np.ndarray) -> None:
    """Write (x, y, speed) waypoints -- the file pure_pursuit_node loads."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'speed'])
        for (x, y), v in zip(xy, speed):
            writer.writerow([f'{x:.4f}', f'{y:.4f}', f'{v:.3f}'])


# ============================================================================
# 6. Reactive obstacle avoidance (gap-finding, reused from gap_follow)
# ============================================================================
#
# pure_pursuit_node's racing line has no idea an opponent's car, a
# spun-out car, or anything else not in the map exists. The reactive
# LIDAR safety net catches this at runtime, in two tiers: emergency stop
# if something is very close, or -- what this function is for -- steer
# *around* it if there's room, instead of just stopping. This is the same
# "find the best gap" idea gap_follow uses to drive with no map at all,
# reused here as a temporary override on top of the racing line rather
# than as the whole control strategy.

def find_best_gap(ranges: np.ndarray, min_gap_distance: float):
    """Find the best drivable opening in a window of range readings.

    A "gap" is a contiguous run of readings all farther than
    `min_gap_distance`. Candidates are scored by width * average_depth,
    not width alone: a shallow dead end (e.g. a ~1m doorway alcove) can
    subtend a *wider* angle than a genuinely open corridor that's
    angularly narrower but far deeper. Scoring this way means a gap has
    to actually stay open for a while, not just be wide at its mouth, to
    win -- the same reasoning gap_follow's own gap search uses.

    Returns (start_index, end_index) of the winning run within `ranges`,
    or (None, None) if nothing in the window is farther than
    `min_gap_distance` at all (boxed in on every side).
    """
    free = ranges > min_gap_distance
    candidates = []
    run_start = None
    for i, is_free in enumerate(free):
        if is_free and run_start is None:
            run_start = i
        elif not is_free and run_start is not None:
            candidates.append((run_start, i - 1))
            run_start = None
    if run_start is not None:
        candidates.append((run_start, len(free) - 1))

    if not candidates:
        return None, None

    def score(run):
        start, end = run
        segment = ranges[start:end + 1]
        width = end - start + 1
        avg_depth = float(np.mean(segment))
        return width * avg_depth

    return max(candidates, key=score)


# ============================================================================
# 7. Opponent detection, tracking, and overtaking
# ============================================================================
#
# "Proper racing" against another car needs more than reactive avoidance
# -- it needs to notice the other car is *there*, work out whether it's
# gaining or falling behind relative to it, and -- if it's gaining --
# actually get past, not just follow at a safe distance forever. This
# section is the math behind that, in three steps that mirror how a
# human driver actually thinks about a car ahead:
#
#   1. "Is that a car?"           -- detect_opponent_cluster / cluster_scan_ranges
#   2. "Are they pulling away, or am I catching them?" -- track progress
#      along the racing line itself (compute_cumulative_arc_length /
#      track_progress_gap), not raw x/y -- see the note on why below.
#   3. "Which side has room, and how do I actually steer there?"  --
#      pick_pass_side / lateral_offset_point
#
# None of this needs a second sensor, a neural network, or radio contact
# with the other car -- just the same LIDAR scan and racing line already
# in use everywhere else in this node.

def cluster_scan_ranges(ranges: np.ndarray, max_range: float, cluster_gap_threshold: float = 0.3):
    """Group consecutive LIDAR readings into clusters: contiguous runs of
    "something is there" (a reading noticeably less than max_range, i.e.
    not just open track) where consecutive readings don't jump by more
    than `cluster_gap_threshold`. A big jump between neighbors means a
    *different* object even if both readings are "close" -- e.g. a car
    sitting in front of a wall shows up as one cluster for the car, a
    jump, then a separate cluster for the wall behind it.

    Returns a list of (start_index, end_index) tuples, one per cluster, in
    scan order. Deliberately simple -- no calibration, no learned model,
    just "is there a return here, and is it continuous with its
    neighbor" -- see detect_opponent_cluster for how that's enough to
    tell an isolated, car-sized object apart from a wall.
    """
    is_object = ranges < (max_range * 0.95)
    clusters = []
    start = None
    for i in range(len(ranges)):
        if is_object[i]:
            if start is None:
                start = i
            elif abs(float(ranges[i]) - float(ranges[i - 1])) > cluster_gap_threshold:
                clusters.append((start, i - 1))
                start = i
        else:
            if start is not None:
                clusters.append((start, i - 1))
                start = None
    if start is not None:
        clusters.append((start, len(ranges) - 1))
    return clusters


def cluster_geometry(ranges: np.ndarray, angle_min: float, angle_increment: float,
                      start_idx: int, end_idx: int):
    """Physical size and centroid of a cluster (see cluster_scan_ranges),
    relative to the LIDAR. `physical_width` is the straight-line (chord)
    distance between the cluster's first and last point in Cartesian
    coordinates -- a much better estimate of an object's actual size than
    its angular width alone, which exaggerates anything close and
    shrinks anything far away.

    Returns (physical_width, centroid_range, centroid_angle).
    """
    angles = angle_min + np.arange(start_idx, end_idx + 1) * angle_increment
    segment = ranges[start_idx:end_idx + 1]
    xs = segment * np.cos(angles)
    ys = segment * np.sin(angles)
    physical_width = float(np.hypot(xs[-1] - xs[0], ys[-1] - ys[0]))
    centroid_idx = (start_idx + end_idx) // 2
    centroid_range = float(ranges[centroid_idx])
    centroid_angle = float(angle_min + centroid_idx * angle_increment)
    return physical_width, centroid_range, centroid_angle


def detect_opponent_cluster(ranges: np.ndarray, angle_min: float, angle_increment: float,
                             max_range: float, min_width: float, max_width: float,
                             max_engagement_range: float, cluster_gap_threshold: float = 0.3,
                             open_side_margin: float = 0.5):
    """Look for a single object in the scan shaped and sized like another
    race car, sitting out in the open track rather than flush against a
    wall.

    A car-sized object looks like: a cluster of readings noticeably
    closer than its surroundings (by at least `open_side_margin`),
    spanning a physical width between `min_width` and `max_width` --
    narrow enough it isn't a wall segment (walls produce much longer or
    far more irregular runs), wide enough it isn't sensor noise or a
    thin post. Among every cluster that qualifies, the closest one wins
    (the one most immediately relevant to a racing decision right now).

    This is a geometric heuristic, not object recognition -- no map is
    consulted, no machine learning, and nothing is tracked between calls
    (see OpponentTracker in pure_pursuit_node.py for that part). It's
    exactly the kind of reasoning gap_follow already does for its own gap
    search, just aimed at *finding* an object instead of *avoiding* one.

    Returns (start_idx, end_idx, centroid_range, centroid_angle) for the
    winning cluster, or None if nothing currently in view qualifies.
    """
    best = None
    for start_idx, end_idx in cluster_scan_ranges(ranges, max_range, cluster_gap_threshold):
        width, centroid_range, centroid_angle = cluster_geometry(
            ranges, angle_min, angle_increment, start_idx, end_idx)
        if not (min_width <= width <= max_width):
            continue
        if centroid_range > max_engagement_range:
            continue

        # Confirm open space immediately on both sides -- otherwise this
        # is plausibly just one section of a curving wall, not a
        # discrete object sitting in the middle of the drivable track.
        before = ranges[max(0, start_idx - 5):start_idx]
        after = ranges[end_idx + 1:end_idx + 6]
        if before.size and float(np.mean(before)) < centroid_range + open_side_margin:
            continue
        if after.size and float(np.mean(after)) < centroid_range + open_side_margin:
            continue

        if best is None or centroid_range < best[2]:
            best = (start_idx, end_idx, centroid_range, centroid_angle)

    return best


def compute_cumulative_arc_length(seg_len: np.ndarray) -> np.ndarray:
    """Running total of track distance up to (not including) each
    waypoint: cumulative[i] is how far along the track waypoint i is from
    waypoint 0. Precomputed once at startup (the racing line doesn't
    change during a run) so "how far along the track is index i" is an
    O(1) array read instead of re-summing seg_len from scratch -- needed
    at least once per control tick, for both the ego car and any tracked
    opponent.
    """
    cumulative = np.zeros(len(seg_len))
    cumulative[1:] = np.cumsum(seg_len[:-1])
    return cumulative


def track_progress_gap(ego_arc_length: float, other_arc_length: float, total_length: float) -> float:
    """Signed track-distance from the ego car to another point on a
    closed-loop racing line, expressed as "how far *ahead*" the other
    point is: always in [0, total_length), wrapping the finish line back
    to the start -- on a loop, "ahead" always exists somewhere less than
    one full lap away, which is exactly what predicting an opponent's
    position needs (see OpponentTracker.predicted_arc_length).
    """
    if total_length <= 0.0:
        return 0.0
    return float((other_arc_length - ego_arc_length) % total_length)


def lateral_offset_point(xy: np.ndarray, index: int, next_index: int, offset: float):
    """A point `offset` meters to the *left* of waypoint `index` (negative
    offset = right), measured perpendicular to the track's local
    direction of travel (estimated from index -> next_index). Used to
    nudge the Pure Pursuit steering target sideways during an overtake,
    without needing a whole second recorded racing line -- see
    "Overtaking" in docs/racing-autonomy.md.
    """
    dx = xy[next_index][0] - xy[index][0]
    dy = xy[next_index][1] - xy[index][1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return float(xy[index][0]), float(xy[index][1])
    # Perpendicular-left unit vector of the direction of travel (dx, dy):
    # rotate it +90 degrees, matching pick_pass_side's left/right sense.
    perp_x, perp_y = -dy / length, dx / length
    return float(xy[index][0] + perp_x * offset), float(xy[index][1] + perp_y * offset)


def pick_pass_side(ranges: np.ndarray, start_idx: int, end_idx: int, window: int = 20) -> int:
    """Which side of a detected cluster has more open room to pass
    through: the average range in a small window of readings just
    outside the cluster on the left vs. the right (higher scan index is
    further left, per REP-103 -- angle increases with index, and +angle
    means +Y, which is left). Returns +1 (pass on the left) or -1 (pass
    on the right); a tie breaks toward the left arbitrarily -- both
    sides being this close to equal means neither choice is meaningfully
    better.
    """
    left = ranges[end_idx + 1:end_idx + 1 + window]
    right = ranges[max(0, start_idx - window):start_idx]
    left_room = float(np.mean(left)) if left.size else 0.0
    right_room = float(np.mean(right)) if right.size else 0.0
    return 1 if left_room >= right_room else -1


# ----------------------------------------------------------------------------
# Map-subtraction opponent detection
# ----------------------------------------------------------------------------
#
# detect_opponent_cluster above has to *guess* "car vs. wall" from cluster
# shape alone, which is exactly what fools it on curving walls and clipped
# clusters. When a map and a localized pose are available (they are,
# whenever pure_pursuit is racing), there's a categorically better signal:
# ray-cast the scan the LIDAR *should* see from the current pose given
# only the map (see map_subtraction.py for the range_libc wrapper), and
# compare it with the scan it *actually* sees. Any beam that comes back
# meaningfully shorter than the map predicts is, by definition, hitting
# something that is not in the map -- an opponent, wherever it is and
# whatever the wall behind it looks like. The comparison/clustering here
# is kept free of range_libc so it stays unit-testable like everything
# else in this file.

def dynamic_beam_mask(measured: np.ndarray, expected: np.ndarray,
                       margin: float, range_min: float = 0.0) -> np.ndarray:
    """Boolean mask of beams whose measured range is at least `margin`
    shorter than the map-predicted range -- i.e. beams hitting something
    that is not in the map. Non-finite or sub-`range_min` measurements are
    never flagged (an invalid beam is *unknown*, not evidence of an
    object), and neither is anything the map already explains.
    """
    measured = np.asarray(measured, dtype=np.float64)
    expected = np.asarray(expected, dtype=np.float64)
    valid = np.isfinite(measured) & (measured >= range_min)
    return valid & (measured < (expected - margin))


def detect_dynamic_cluster(measured: np.ndarray, expected: np.ndarray,
                            angle_min: float, angle_increment: float,
                            margin: float, min_width: float, max_width: float,
                            max_engagement_range: float, range_min: float = 0.0,
                            cluster_gap_threshold: float = 0.3):
    """Find the closest car-plausible cluster of *dynamic* beams -- beams
    the map fails to explain (see dynamic_beam_mask). Contiguous flagged
    beams form a cluster, split where consecutive measured ranges jump by
    more than `cluster_gap_threshold` (two unmapped objects at different
    depths must not merge into one). Width and engagement-range gating
    reuse the same limits as the heuristic detector, but note the width
    check here is only guarding against garbage (a flock of noise beams,
    two cars merged), not doing the car-vs-wall discrimination -- the map
    subtraction itself already did that.

    Returns (start_idx, end_idx, centroid_range, centroid_angle) in the
    same shape detect_opponent_cluster returns, or None. Indices are into
    `measured`/`expected` -- if the caller downsampled the scan, it owns
    mapping them back (and must pass the *downsampled* angle_increment).
    """
    mask = dynamic_beam_mask(measured, expected, margin, range_min)

    best = None
    start = None
    n = len(measured)
    for i in range(n + 1):
        in_cluster = i < n and mask[i]
        split = (in_cluster and start is not None
                 and abs(float(measured[i]) - float(measured[i - 1])) > cluster_gap_threshold)
        if in_cluster and start is None:
            start = i
            continue
        if in_cluster and not split:
            continue
        if start is not None:
            end = i - 1
            width, centroid_range, centroid_angle = cluster_geometry(
                measured, angle_min, angle_increment, start, end)
            if (min_width <= width <= max_width
                    and centroid_range <= max_engagement_range
                    and (best is None or centroid_range < best[2])):
                best = (start, end, centroid_range, centroid_angle)
            start = i if split else None
    return best
