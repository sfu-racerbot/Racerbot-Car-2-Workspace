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
                              closed: bool = True, smoothing_passes: int = 3) -> np.ndarray:
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
    pair (`smoothing_passes` times, default 3) lets that start/finish seam
    converge. Both passes only ever lower values (never raise them), so
    repeating them on an already-converged open path is a harmless no-op
    -- there is no need to special-case open vs. closed here.

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

    for _ in range(max(1, smoothing_passes)):
        # Forward / acceleration-limited sweep.
        fwd_order = range(n) if closed else range(1, n)
        for i in fwd_order:
            prev_i = (i - 1) % n
            v_cap = math.sqrt(v[prev_i] ** 2 + 2.0 * a_accel_max * seg_len[prev_i])
            if v_cap < v[i]:
                v[i] = v_cap

        # Backward / braking-limited sweep.
        bwd_order = range(n - 1, -1, -1) if closed else range(n - 2, -1, -1)
        for i in bwd_order:
            next_i = (i + 1) % n
            v_cap = math.sqrt(v[next_i] ** 2 + 2.0 * a_brake_max * seg_len[i])
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
