"""
predict.py

Forward-integrate the trajectory a driving algorithm *intends* to follow.

This is deliberately not "where the car is going" -- odometry already
answers that, and the whole point of the intent arrow on the dashboard is
to show the plan the controller is acting on so a wrong plan can be seen
*before* the car acts it out. Everything here therefore takes the
controller's own desired steering/speed as input and asks a kinematic
question: if the algorithm got what it is asking for, where would the car
be over the next `horizon_s` seconds?

No ROS, no numpy, no I/O -- plain `math` only, so that:

  * it is directly unit-testable without a robot (see test/test_predict.py),
  * every step has an exact closed form that is easy to re-derive, and
  * the C++ port in include/drive_intent/drive_intent.hpp, which the
    racerbot_a/racerbot_b codebases use, is a line-for-line translation
    rather than a reinterpretation.

Frames: everything is the standard ROS body convention -- +X forward,
+Y left, yaw counter-clockwise about +Z, distances in meters, angles in
radians.
"""

import math

# Below this curvature magnitude an arc and a straight line differ by less
# than a LIDAR pixel over any horizon we draw, and the 1/kappa in the arc
# form blows up -- so treat it as straight. 1e-9/m is a ~1000km radius.
STRAIGHT_CURVATURE_EPS = 1e-9


def curvature_from_steering(steering: float, wheelbase: float) -> float:
    """Path curvature (1/m) of a bicycle-model car at a given steer angle.

    The inverse of racing_math.steering_from_curvature; duplicated here
    rather than imported so this module stays dependency-free for the
    C++ port and for packages that don't depend on pure_pursuit.
    """
    if not (math.isfinite(wheelbase) and wheelbase > 0.0):
        raise ValueError(f'wheelbase must be finite and positive, got {wheelbase!r}')
    if not math.isfinite(steering):
        raise ValueError(f'steering must be finite, got {steering!r}')
    return math.tan(steering) / wheelbase


def arc_step(x: float, y: float, yaw: float, speed: float, steering: float,
             wheelbase: float, dt: float):
    """Advance one pose by `dt` at constant speed and constant steering.

    Uses the *exact* constant-curvature update, not an Euler step. Over a
    1.5s horizon at full lock the two disagree by several centimeters,
    which is precisely the scale at which someone would be squinting at
    the arrow trying to decide whether the car is going to clip a cone.
    An arrow that is wrong by the width of the thing it is about to hit
    is worse than no arrow.

    Returns the new (x, y, yaw).
    """
    kappa = curvature_from_steering(steering, wheelbase)
    ds = speed * dt
    if abs(kappa) < STRAIGHT_CURVATURE_EPS:
        return x + ds * math.cos(yaw), y + ds * math.sin(yaw), yaw
    dyaw = kappa * ds
    radius = 1.0 / kappa
    nx = x + radius * (math.sin(yaw + dyaw) - math.sin(yaw))
    ny = y - radius * (math.cos(yaw + dyaw) - math.cos(yaw))
    return nx, ny, yaw + dyaw


def integrate(steering_of, speed_of, wheelbase: float, horizon_s: float,
              samples: int, start=(0.0, 0.0, 0.0), max_length_m=None):
    """Roll the bicycle model forward and return the intended trajectory.

    `steering_of(i, x, y, yaw)` and `speed_of(i, x, y, yaw)` are called
    once per step, which is what lets one function serve both driving
    styles in this workspace: gap_follow passes constants (it produces a
    *direction to head*, so its intent really is one arc), while
    pure_pursuit re-evaluates the pure-pursuit law against the racing
    line at every step, so its arrow bends through the corner ahead
    instead of shooting off on a frozen tangent.

    Returns a list of `samples` (x, y, yaw, v) tuples in the same frame as
    `start`, first entry being `start` itself. `max_length_m`, if given,
    truncates the list once that much arc length has been covered -- an
    arrow longer than the visible map is not more informative, just
    harder to read.
    """
    if samples < 2:
        raise ValueError(f'samples must be at least 2, got {samples!r}')
    if not (math.isfinite(horizon_s) and horizon_s > 0.0):
        raise ValueError(f'horizon_s must be finite and positive, got {horizon_s!r}')

    dt = horizon_s / (samples - 1)
    x, y, yaw = start
    points = []
    travelled = 0.0
    for i in range(samples):
        speed = float(speed_of(i, x, y, yaw))
        if not math.isfinite(speed):
            raise ValueError(f'speed_of returned a non-finite value at step {i}')
        points.append((x, y, yaw, speed))
        if i == samples - 1:
            break
        if max_length_m is not None and travelled >= max_length_m:
            break
        steering = float(steering_of(i, x, y, yaw))
        x, y, yaw = arc_step(x, y, yaw, speed, steering, wheelbase, dt)
        travelled += abs(speed) * dt
    return points


def constant_arc(steering: float, speed: float, wheelbase: float,
                 horizon_s: float, samples: int, start=(0.0, 0.0, 0.0),
                 max_length_m=None):
    """The single-arc case: hold this steering and this speed for the
    whole horizon. This is the honest model for a reactive controller,
    which has chosen a heading rather than a path."""
    return integrate(
        lambda *_: steering,
        lambda *_: speed,
        wheelbase,
        horizon_s,
        samples,
        start=start,
        max_length_m=max_length_m,
    )


def to_body(points, origin_x: float, origin_y: float, origin_yaw: float):
    """Re-express world-frame integration output in a body frame.

    pure_pursuit has to simulate in the map frame (that is where the
    racing line lives) but the dashboard wants base_link, so that the
    arrow renders identically in robot-centric mode -- where there is no
    pose at all -- and in map-relative mode.
    """
    cos_yaw = math.cos(-origin_yaw)
    sin_yaw = math.sin(-origin_yaw)
    out = []
    for x, y, yaw, v in points:
        dx = x - origin_x
        dy = y - origin_y
        out.append((
            dx * cos_yaw - dy * sin_yaw,
            dx * sin_yaw + dy * cos_yaw,
            yaw - origin_yaw,
            v,
        ))
    return out


def path_length(points) -> float:
    """Total arc length of an integrated path, straight-line between
    samples. Used to decide whether a path is worth drawing at all."""
    total = 0.0
    for (x0, y0, _, _), (x1, y1, _, _) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def polar_to_body(bearing: float, distance: float,
                  sensor_offset_x: float = 0.0,
                  sensor_offset_y: float = 0.0):
    """A LIDAR-frame polar return as a base_link (x, y) point.

    The LIDAR on this car sits 0.33m ahead of base_link
    (docs/hardware-reference.md), so a gap target drawn straight from
    (bearing, distance) would be a third of a meter behind where the car
    is actually aiming -- roughly a car length of error on the one marker
    whose whole job is to say "there, that is the hole I picked."
    """
    return (
        sensor_offset_x + distance * math.cos(bearing),
        sensor_offset_y + distance * math.sin(bearing),
    )
