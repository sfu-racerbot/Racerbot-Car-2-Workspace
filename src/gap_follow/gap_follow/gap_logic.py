"""
gap_logic.py

All of gap_follow's scan-processing math, importable and unit-testable
without rclpy -- the same split pure_pursuit uses for racing_math.py
(see docs/writing-your-own-node.md). gap_follow_node.py owns the ROS
plumbing (parameters, topics, the deadman gate) and composes the
functions here; this file owns everything that turns a range array into
a steering target and can therefore be tested with plain numpy arrays:

    python3 -m pytest src/gap_follow/test/ -v

The processing pipeline, in the order the node applies it:

  sanitize_ranges     -> which beams are trustworthy, and a gap-safe copy
  vehicle_boundary... -> LiDAR-to-body clearance for each scan direction
  minimum_...         -> footprint clearance plus forward clearance floor
  conservative_...    -> safest recent odometry/command speed for TTC
  minimum_ttc         -> footprint-aware LiDAR collision timing
  curvature_speed...  -> lateral-acceleration speed ceiling
  braking_speed...    -> clearance-aware stopping speed ceiling
  slew_rate_limit     -> bounded normal command changes
  closest_valid       -> safety-bubble anchor (valid beams only)
  disparity_extend    -> widen every obstacle *edge* by half a car width
  safety_bubble       -> zero out a car-width bubble around the closest hit
  find_gap_with...    -> preferred deep gap, then a slow corner fallback
                         (with hysteresis against the previous tick's aim)
  aim_within_gap      -> which beam inside that gap to steer at
  near_gap_bearing    -> the same gap's near-only bearing, for anticipation
  side_wall_distance  -> perpendicular distance to the wall on one side
  corridor_centering  -> bounded cross-track bias, faded in on straights
  steering_damping    -> gyro yaw rate, opposed; added before the clip
  grip_ratio, AdaptiveGrip, adaptive_grip_step -> the gyro's lateral-limit judge
  SteeringLag         -> the rack steering from a lag ago, while it is steady
  corridor_width_fac. -> how open the sensed corridor is, as a 0..1 fade
  scale_between       -> blend a margin/speed between its narrow/wide values
"""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def sanitize_ranges(ranges, max_range: float, range_min: float = 0.0):
    """Split a raw scan into a gap-safe range array plus a validity mask.

    The two output semantics exist because an *invalid* beam (NaN, or a
    sub-range_min reading -- the sensor's own "no valid return" encoding)
    means two different things to the two consumers:

      - For the emergency stop / closest-obstacle check it is *unknown*,
        not an obstacle at 0.0m: counting it would slam the brakes on
        every scan dropout (the phantom-obstacle bug this replaces).
        The `valid` mask excludes those beams from that check.
      - For gap selection it must stay *non-free* (kept at 0.0 in
        `clean`): steering into a blind spot because it "looked open"
        is worse than stopping.

    +inf is different from NaN: it is a real measurement ("nothing within
    the sensor's reach"), so it becomes max_range -- genuinely free space.

    Returns (clean, valid): `clean` is float64, invalid beams 0.0, clipped
    to [0, max_range]; `valid` is a boolean mask of trustworthy beams.
    """
    raw = np.asarray(ranges, dtype=np.float64)
    valid = np.isfinite(raw) & (raw >= range_min) & (raw > 0.0)
    clean = np.where(valid, raw, 0.0)
    clean[np.isposinf(raw)] = max_range
    return np.clip(clean, 0.0, max_range), valid


def closest_valid(clean: np.ndarray, valid: np.ndarray):
    """Index and distance of the closest *trustworthy* reading, or
    (None, inf) if nothing in the window is valid. This is what the
    emergency stop and the safety bubble anchor on -- invalid beams
    never trigger a stop, and never get a bubble carved around them.
    """
    if clean.size == 0 or not np.any(valid):
        return None, math.inf
    masked = np.where(valid, clean, np.inf)
    idx = int(np.argmin(masked))
    return idx, float(masked[idx])


def vehicle_boundary_distances(angles: np.ndarray, car_width: float,
                               car_length: float, wheelbase: float,
                               laser_offset_x: float,
                               laser_offset_y: float = 0.0) -> np.ndarray:
    """Distance from the LiDAR to the rectangular vehicle edge per beam.

    ``base_link`` is the rear axle in this workspace. Following the F1TENTH
    collision model, the rectangular body is centered halfway along the
    wheelbase, with symmetric front/rear overhang. The LiDAR origin is offset
    from ``base_link`` by ``laser_offset_x/y``.

    The returned distance is the amount that must be subtracted from a raw
    LiDAR range before it represents clearance from the *car body*, rather
    than clearance from the sensor. The sensor must lie inside the footprint;
    a mismatched transform or footprint raises ``ValueError`` instead of
    silently creating unsafe collision distances.
    """
    beam_angles = np.asarray(angles, dtype=np.float64)
    dimensions = (car_width, car_length, wheelbase)
    if not all(math.isfinite(value) and value > 0.0 for value in dimensions):
        raise ValueError('car_width, car_length, and wheelbase must be finite and positive')
    if not math.isfinite(laser_offset_x) or not math.isfinite(laser_offset_y):
        raise ValueError('LiDAR offsets must be finite')

    half_width = car_width / 2.0
    half_length = car_length / 2.0
    body_center_x = wheelbase / 2.0
    x_min = body_center_x - half_length
    x_max = body_center_x + half_length
    y_min = -half_width
    y_max = half_width
    tolerance = 1e-9
    if not (x_min - tolerance <= laser_offset_x <= x_max + tolerance
            and y_min - tolerance <= laser_offset_y <= y_max + tolerance):
        raise ValueError('LiDAR origin must lie inside the configured vehicle footprint')

    direction_x = np.cos(beam_angles)
    direction_y = np.sin(beam_angles)
    epsilon = 1e-12
    distance_x = np.full(beam_angles.shape, np.inf, dtype=np.float64)
    distance_y = np.full(beam_angles.shape, np.inf, dtype=np.float64)

    positive_x = direction_x > epsilon
    negative_x = direction_x < -epsilon
    distance_x[positive_x] = (
        (x_max - laser_offset_x) / direction_x[positive_x])
    distance_x[negative_x] = (
        (x_min - laser_offset_x) / direction_x[negative_x])

    positive_y = direction_y > epsilon
    negative_y = direction_y < -epsilon
    distance_y[positive_y] = (
        (y_max - laser_offset_y) / direction_y[positive_y])
    distance_y[negative_y] = (
        (y_min - laser_offset_y) / direction_y[negative_y])

    return np.minimum(distance_x, distance_y)


def minimum_footprint_clearance(clean: np.ndarray, valid: np.ndarray,
                                boundary_distances: np.ndarray) -> float:
    """Smallest valid obstacle clearance measured from the vehicle body."""
    ranges = np.asarray(clean, dtype=np.float64)
    validity = np.asarray(valid, dtype=bool)
    boundaries = np.asarray(boundary_distances, dtype=np.float64)
    if ranges.shape != validity.shape or ranges.shape != boundaries.shape:
        raise ValueError('ranges, validity, and boundary distances must have matching shapes')
    usable = validity & np.isfinite(boundaries)
    if not np.any(usable):
        return math.inf
    return float(np.min(ranges[usable] - boundaries[usable]))


def minimum_footprint_clearance_in_cone(
        clean: np.ndarray, valid: np.ndarray, angles: np.ndarray,
        boundary_distances: np.ndarray, cone_width_rad: float) -> float:
    """Smallest body clearance inside a forward-centred angular cone."""
    ranges = np.asarray(clean, dtype=np.float64)
    validity = np.asarray(valid, dtype=bool)
    beam_angles = np.asarray(angles, dtype=np.float64)
    boundaries = np.asarray(boundary_distances, dtype=np.float64)
    if not (ranges.shape == validity.shape == beam_angles.shape == boundaries.shape):
        raise ValueError('all forward-clearance inputs must have matching shapes')
    if not math.isfinite(cone_width_rad) or not (
            0.0 < cone_width_rad <= 2.0 * math.pi):
        raise ValueError('cone_width_rad must be finite and in (0, 2*pi]')

    inside_cone = np.abs(beam_angles) <= cone_width_rad / 2.0
    return minimum_footprint_clearance(
        ranges[inside_cone], validity[inside_cone], boundaries[inside_cone])


def conservative_ttc_speed(measured_speed: float,
                           commanded_speed: float = 0.0,
                           command_age_sec: float = math.inf,
                           command_timeout_sec: float = 0.5,
                           fallback_max_measured_speed: float = 0.1) -> float:
    """Safest speed when fresh odometry is absent or effectively zero.

    Sign-agnostic: ``measured_speed`` is used by magnitude, so an odometry
    source with an inverted sign convention cannot suppress braking.
    """
    if not math.isfinite(measured_speed):
        raise ValueError('measured_speed must be finite')
    if not math.isfinite(commanded_speed):
        raise ValueError('commanded_speed must be finite')
    if math.isnan(command_age_sec) or command_age_sec < 0.0:
        raise ValueError('command_age_sec must be non-negative')
    if not math.isfinite(command_age_sec) and command_age_sec != math.inf:
        raise ValueError('command_age_sec must be finite or positive infinity')
    if not math.isfinite(command_timeout_sec) or command_timeout_sec < 0.0:
        raise ValueError('command_timeout_sec must be finite and non-negative')
    if (not math.isfinite(fallback_max_measured_speed)
            or fallback_max_measured_speed < 0.0):
        raise ValueError(
            'fallback_max_measured_speed must be finite and non-negative')

    # Use the *magnitude* of the measurement. A sign convention belongs to
    # the odometry source, not to how fast the car is actually moving, and
    # trusting it here does not fail loudly -- it fails silent and open:
    #
    #   A car driving forward at 1.8m/s whose odometry reports -1.8m/s reads
    #   as "not moving forward", passes the effectively-stationary test
    #   below, and falls through to the commanded speed. One tick after any
    #   brake the commanded speed is 0, so the effective TTC speed is 0, no
    #   beam counts as approaching, TTC is infinite -- and the brake releases
    #   itself on the very next scan. The result is a 20Hz square wave of
    #   full speed and zero rather than a stop. That crashed the car.
    #
    # This node never commands reverse, so the magnitude is always the
    # conservative estimate, and it is correct under either sign convention.
    speed = abs(measured_speed)
    if (speed <= fallback_max_measured_speed
            and command_age_sec <= command_timeout_sec):
        speed = max(speed, commanded_speed)
    return max(0.0, speed)


def time_to_collision(clean: np.ndarray, valid: np.ndarray,
                      angles: np.ndarray, speed: float,
                      boundary_distances: np.ndarray,
                      min_closing_speed: float = 0.05,
                      swept_half_width: float = None,
                      path_curvature: float = 0.0,
                      laser_offset_x: float = 0.0,
                      laser_offset_y: float = 0.0) -> np.ndarray:
    """Instantaneous TTC for every scan beam, with the car footprint removed.

    This is the F1TENTH iTTC construction: longitudinal odometry speed is
    projected onto each beam with ``speed * cos(angle)``. Beams the car is not
    approaching have infinite TTC. Raw ranges are converted to body clearance
    first, so the clock reaches zero when the rectangular car body reaches the
    obstacle, not when the LiDAR itself does.

    ``swept_half_width`` gates the whole thing on whether the car can actually
    *reach* a beam's obstacle. The radial projection above is only valid for
    something the car is driving at: for an obstacle off to the side it
    manufactures a collision that cannot happen, because the car passes beside
    it. On a wide track that is harmless, but it scales with how close the
    walls are, and on a 1m course it dominates -- measured on this car,
    perfectly centred, it capped the car at 1.42m/s off a wall 0.50m to the
    side, and at 0.27m/s once it drifted to 0.22m from that wall. That is not
    the brake being cautious, it is the brake being wrong.

    Gating on lateral offset (``|r*sin(angle)| <= swept_half_width``) keeps
    every obstacle the straight-ahead car would hit and drops the ones it
    would pass. Pass ``None`` to keep the ungated behaviour.

    Note this models straight-line travel, so it does not by itself cover a
    wall the car is *turning* into; the all-direction contact floor and the
    forward-cone clearance layer own that case.
    """
    ranges = np.asarray(clean, dtype=np.float64)
    validity = np.asarray(valid, dtype=bool)
    beam_angles = np.asarray(angles, dtype=np.float64)
    boundaries = np.asarray(boundary_distances, dtype=np.float64)
    if not (ranges.shape == validity.shape == beam_angles.shape == boundaries.shape):
        raise ValueError('all TTC inputs must have matching shapes')
    if not math.isfinite(speed):
        raise ValueError('speed must be finite')
    if not math.isfinite(min_closing_speed) or min_closing_speed < 0.0:
        raise ValueError('min_closing_speed must be finite and non-negative')

    closing_speed = speed * np.cos(beam_angles)
    approaching = (
        validity
        & np.isfinite(boundaries)
        & (closing_speed > min_closing_speed)
    )
    if swept_half_width is not None:
        if not math.isfinite(swept_half_width) or swept_half_width <= 0.0:
            raise ValueError('swept_half_width must be finite and positive')
        if not math.isfinite(path_curvature):
            raise ValueError('path_curvature must be finite')
        point_x = ranges * np.cos(beam_angles)
        point_y = ranges * np.sin(beam_angles)
        if abs(path_curvature) < 1e-6:
            offset_from_path = np.abs(point_y)
        else:
            # Turning: the swept region is an annulus about the turn centre,
            # not a straight band. Evaluating a hard-over car as if it were
            # going straight is what deadlocked it mid-corner -- the outer
            # wall sat in the straight-ahead band while the car was curving
            # around it, so TTC braked, the escape creep re-commanded motion,
            # and the two alternated at scan rate to a net standstill.
            radius = 1.0 / path_curvature
            centre_x = -laser_offset_x
            centre_y = -laser_offset_y + radius
            offset_from_path = np.abs(
                np.hypot(point_x - centre_x, point_y - centre_y)
                - abs(radius))
        approaching &= offset_from_path <= swept_half_width
    ttc = np.full(ranges.shape, np.inf, dtype=np.float64)
    clearances = np.maximum(0.0, ranges - boundaries)
    ttc[approaching] = clearances[approaching] / closing_speed[approaching]
    return ttc


def minimum_ttc(clean: np.ndarray, valid: np.ndarray,
                angles: np.ndarray, speed: float,
                boundary_distances: np.ndarray,
                min_closing_speed: float = 0.05,
                swept_half_width: float = None,
                path_curvature: float = 0.0,
                laser_offset_x: float = 0.0,
                laser_offset_y: float = 0.0) -> float:
    """Return the minimum finite footprint-aware iTTC, or infinity."""
    ttc = time_to_collision(
        clean, valid, angles, speed, boundary_distances, min_closing_speed,
        swept_half_width, path_curvature, laser_offset_x, laser_offset_y)
    return float(np.min(ttc)) if ttc.size else math.inf


def swept_path_clearance(clean: np.ndarray, valid: np.ndarray,
                         angles: np.ndarray, curvature: float,
                         half_width: float, car_length: float,
                         wheelbase: float, laser_offset_x: float,
                         laser_offset_y: float = 0.0,
                         max_length: float = 3.0) -> float:
    """How far the car can drive along a constant-curvature arc before its
    body reaches a scanned obstacle, or ``inf`` if nothing is in the way
    within ``max_length``.

    This is the question the forward-cone clearance was standing in for, and
    on a narrow corridor the cone answers it badly. The cone takes the
    *minimum* range over +/-30deg, and in a 1.5m hallway the beams at +/-30deg
    land on the side walls ~1.25m away -- so "forward clearance" there is
    really side-wall distance, which a car driving straight down the hallway
    will never hit. That number capped the car at ~2.4m/s on straights with
    nothing ahead of it (docs/gap-follow-improvement-plan.md section 3.2).

    Here only obstacles inside the band the body actually sweeps count, and
    the distance is measured *along* the arc. ``base_link`` is the rear axle
    (the bicycle model turns about it), so the arc is the rear axle's path,
    and the band is the region the rectangular body covers as it follows
    it: for a turn of radius R the body spans radii ``R - half_width`` (the
    inner side, alongside the rear axle) out to
    ``hypot(front, R + half_width)`` (the outer front corner). The inner
    edge is what models off-tracking -- the rear of the car cuts inside the
    path the LiDAR traces, which is how a car aimed past an inner corner
    still clips it with its flank.

    ``half_width`` is half the car's width plus whatever margin the caller
    wants (the node passes ``car_width/2 + safety_margin``, the same swept
    width the TTC brake uses). Returns the free distance from the body's
    front edge, so 0.0 means an obstacle is already inside the band
    alongside the body.

    Sign convention REP-103: positive curvature turns left. Invalid beams
    are unknown, not obstacles, as everywhere else in this file.
    """
    ranges = np.asarray(clean, dtype=np.float64)
    validity = np.asarray(valid, dtype=bool)
    beam_angles = np.asarray(angles, dtype=np.float64)
    if not (ranges.shape == validity.shape == beam_angles.shape):
        raise ValueError('ranges, validity, and angles must have matching shapes')
    for name, value in (('curvature', curvature), ('half_width', half_width),
                        ('car_length', car_length), ('wheelbase', wheelbase),
                        ('laser_offset_x', laser_offset_x),
                        ('laser_offset_y', laser_offset_y),
                        ('max_length', max_length)):
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
    if half_width <= 0.0 or car_length <= 0.0 or wheelbase <= 0.0 or max_length <= 0.0:
        raise ValueError('half_width, car_length, wheelbase and max_length must be positive')

    # Body extent along base_link's x axis (same model as
    # vehicle_boundary_distances: centred halfway along the wheelbase).
    front = wheelbase / 2.0 + car_length / 2.0
    rear = wheelbase / 2.0 - car_length / 2.0

    x = ranges * np.cos(beam_angles) + laser_offset_x
    y = ranges * np.sin(beam_angles) + laser_offset_y
    usable = validity & (ranges > 0.0)

    if abs(curvature) < 1e-6:
        in_band = usable & (np.abs(y) <= half_width) & (x >= rear)
        along = x - front
    else:
        radius = 1.0 / curvature
        abs_radius = abs(radius)
        if abs_radius <= half_width:
            raise ValueError('turn radius must exceed half_width')
        rho = np.hypot(x, y - radius)
        inner = abs_radius - half_width
        outer = math.hypot(front, abs_radius + half_width)
        # Angle swept about the turn centre, in the direction of travel,
        # starting from the rear axle (which sits at angle -pi/2 from a
        # left-turn centre, +pi/2 from a right-turn one).
        start = -math.pi / 2.0 if radius > 0.0 else math.pi / 2.0
        theta = np.arctan2(y - radius, x)
        swept = np.mod(math.copysign(1.0, radius) * (theta - start), 2.0 * math.pi)
        # Points just behind the rear axle wrap to ~2*pi; anything beyond
        # the horizon is ignored anyway, so cap the sweep below a full turn.
        max_sweep = min(max_length + front, 1.9 * math.pi * abs_radius) / abs_radius
        in_band = usable & (rho >= inner) & (rho <= outer) & (swept <= max_sweep)
        along = swept * abs_radius - front

    along = np.where(in_band, along, np.inf)
    nearest = float(np.min(along)) if along.size else math.inf
    if nearest > max_length:
        return math.inf
    return max(0.0, nearest)


def arc_guard_steering(clean: np.ndarray, valid: np.ndarray,
                       angles: np.ndarray, desired_steering: float,
                       required_length: float, half_width: float,
                       car_length: float, wheelbase: float,
                       laser_offset_x: float, laser_offset_y: float,
                       max_steering_angle: float, max_correction: float,
                       candidates: int = 29,
                       blocked_fallback: str = 'freest') -> float:
    """Nearest steering angle to ``desired_steering`` whose swept arc is clear.

    Follow-the-gap picks a *direction*, and a direction says nothing about
    the path the body sweeps on the way there. Approaching a corner the gap
    opens around the bend, its midpoint leans into the turn, and the car
    turns in early -- then the rear of the car, which tracks inside the
    LiDAR, clips the inner corner. That is the indoor_tight wall contact in
    docs/gap-follow-improvement-plan.md section 3.1, and the disparity
    extender cannot prevent it: it inflates the corner by half a car width
    *in angle as seen from the LiDAR*, not along the arc the body follows.

    This checks the arc the car would actually drive (see
    swept_path_clearance) for the requested steering. If the body can
    travel ``required_length`` along it, the request is returned unchanged
    -- on an open track this function never does anything. If not, it tries
    steering angles in order of distance from the request and returns the
    first whose arc is clear. Near an inner corner that is a slightly
    straighter arc: the car holds its line a little longer and turns in
    later, which is a late apex.

    Bounded so it stays a correction, not a second policy: only angles
    within ``max_correction`` of the request are considered (and within
    +/-``max_steering_angle``). If none of those is clear either,
    ``blocked_fallback`` decides: ``'freest'`` returns the one with the most
    free length, ``'request'`` returns the (clipped) request unchanged. Either
    way the swept-path speed ceiling and the reactive layers deal with what
    is left.

    ``'request'`` exists because "freest" can be the wrong way. Entering a
    tight right-hander in the recalibrated simulator (room_2026_07_27,
    reversed, 2026-10-20), full right lock was blocked within the check length
    by the inner corner and every arc within the bound by the outer wall; the
    freest arc was simply the straighter one, whose wall was a little further
    off. The guard unwound the steering from -0.35 to -0.17 rad and the car
    drove into the outer wall it had been turning away from. When nothing in
    reach is clear, the gap's own choice is the better bet.
    """
    for name, value in (('desired_steering', desired_steering),
                        ('required_length', required_length),
                        ('max_steering_angle', max_steering_angle),
                        ('max_correction', max_correction)):
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
    if required_length <= 0.0 or max_correction < 0.0:
        raise ValueError('required_length must be positive and max_correction non-negative')
    if not 0.0 < max_steering_angle < math.pi / 2.0:
        raise ValueError('max_steering_angle must be in (0, pi/2)')
    if candidates < 2:
        raise ValueError('candidates must be at least 2')
    if blocked_fallback not in ('freest', 'request'):
        raise ValueError("blocked_fallback must be 'freest' or 'request'")

    desired = min(max_steering_angle, max(-max_steering_angle, desired_steering))
    horizon = required_length * 1.5

    def free_length(steering):
        return swept_path_clearance(
            clean, valid, angles, math.tan(steering) / wheelbase, half_width,
            car_length, wheelbase, laser_offset_x, laser_offset_y, horizon)

    if free_length(desired) >= required_length:
        return desired

    grid = np.linspace(-max_steering_angle, max_steering_angle, candidates)
    grid = grid[np.abs(grid - desired) <= max_correction + 1e-12]
    best, best_free = desired, free_length(desired)
    for steering in sorted(grid, key=lambda s: (abs(s - desired), abs(s))):
        free = free_length(float(steering))
        if free >= required_length:
            return float(steering)
        if free > best_free:
            best, best_free = float(steering), free
    return desired if blocked_fallback == 'request' else best


def curvature_speed_limit(curvature: float, max_lateral_accel: float,
                          max_speed: float) -> float:
    """Maximum speed satisfying ``v^2 * abs(curvature) <= a_lat_max``."""
    if not all(math.isfinite(value) for value in (
            curvature, max_lateral_accel, max_speed)):
        raise ValueError('curvature speed-limit inputs must be finite')
    if max_lateral_accel <= 0.0 or max_speed < 0.0:
        raise ValueError('max_lateral_accel must be positive and max_speed non-negative')
    if abs(curvature) < 1e-9:
        return max_speed
    return min(max_speed, math.sqrt(max_lateral_accel / abs(curvature)))


def braking_speed_limit(clearance: float, reserve_distance: float,
                        max_braking_decel: float, max_speed: float,
                        latency: float = 0.0) -> float:
    """Clearance speed ceiling: the fastest ``v`` from which the car can stop
    within ``clearance - reserve``.

    With no latency that is ``v^2 <= 2*a*(c - r)``.
    ``latency`` is the time the car keeps going at ``v`` before braking
    starts -- command transport plus the speed loop's response, measured at
    ~0.3 s on this car (docs/run-analysis.md) -- so the distance it covers,
    ``v * latency``, comes off the room first:

        v * latency + v^2 / (2a) <= c - r

    whose positive root is ``-a*t + sqrt((a*t)^2 + 2a(c - r))``.

    Positive infinity means no obstacle was observed and therefore leaves the
    configured top speed unchanged.
    """
    if clearance == math.inf:
        return max_speed
    if not all(math.isfinite(value) for value in (
            clearance, reserve_distance, max_braking_decel, max_speed)):
        raise ValueError('braking speed-limit inputs must be finite')
    if reserve_distance < 0.0 or max_braking_decel <= 0.0 or max_speed < 0.0:
        raise ValueError(
            'reserve_distance/max_speed must be non-negative and braking decel positive')
    if not (math.isfinite(latency) and latency >= 0.0):
        raise ValueError('latency must be finite and non-negative')
    usable_distance = max(0.0, clearance - reserve_distance)
    a_t = max_braking_decel * latency
    v = -a_t + math.sqrt(a_t * a_t + 2.0 * max_braking_decel * usable_distance)
    return min(max_speed, v)


def side_clearance_speed_limit(side_clearance: float, narrow_speed: float,
                               narrow_clearance: float, open_clearance: float,
                               max_speed: float) -> float:
    """Speed ceiling from how close the body runs to the nearer side wall.

    ``side_clearance`` is the gap between the side of the car's body and the
    nearer side wall, in metres. At or below ``narrow_clearance`` the ceiling
    is ``narrow_speed``; at or above ``open_clearance`` it is ``max_speed``;
    in between it rises linearly.

    This is the narrow-corridor limit that pays for the narrower TTC
    corridor (ttc_path_margin, 2026-10-20). The old 0.18 m corridor stopped
    the car dead whenever a side wall came within 0.18 m of its path; the new
    one ignores walls beside the path, so the speed that used to be removed by
    a hard stop is now removed smoothly, and only as much as the squeeze
    calls for. +inf (no wall seen) is open; NaN is treated as narrow.
    """
    for name, value in (('narrow_speed', narrow_speed),
                        ('narrow_clearance', narrow_clearance),
                        ('open_clearance', open_clearance),
                        ('max_speed', max_speed)):
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
    if not 0.0 <= narrow_speed <= max_speed:
        raise ValueError('narrow_speed must be within [0, max_speed]')
    if not narrow_clearance < open_clearance:
        raise ValueError('narrow_clearance must be below open_clearance')
    if math.isnan(side_clearance):
        return narrow_speed
    if side_clearance >= open_clearance:
        return max_speed
    if side_clearance <= narrow_clearance:
        return narrow_speed
    fraction = (side_clearance - narrow_clearance) / (open_clearance - narrow_clearance)
    return narrow_speed + fraction * (max_speed - narrow_speed)


def speed_scheduled_gain(speed: float, reference_speed: float) -> float:
    """Factor on the bearing-steering gain that keeps the heading loop stable
    at speed: ``min(1, reference_speed / speed)``.

    Steering at the gap bearing (delta = K * alpha) closes a heading loop
    whose crossover is ``v * K / wheelbase``: the faster the car, the faster
    a given steering angle turns it. The loop also has a delay -- servo lag
    plus a scan period, ~0.175 s measured on this car (docs/run-analysis.md)
    -- which costs ``crossover * delay`` of phase. At 3 m/s with K = 1 the
    crossover is 8.3 rad/s and the delay eats 84 deg of the 90 available:
    the car weaves, about once a second, which the recalibrated simulator
    reproduced on every straight. Holding the crossover at
    ``reference_speed / wheelbase`` instead keeps a fixed phase margin at any
    speed; below ``reference_speed`` the full gain (and with it the full
    low-speed authority tight corners need) is untouched.

    ``reference_speed`` <= 0 disables the schedule (factor 1).
    """
    if not (math.isfinite(speed) and math.isfinite(reference_speed)):
        raise ValueError('speed and reference_speed must be finite')
    if reference_speed <= 0.0 or speed <= reference_speed:
        return 1.0
    return reference_speed / speed


class SpeedHold:
    """Sliding minimum of the desired speed over the last ``hold_sec``.

    The speed ceilings (curvature, clearance) are recomputed from every scan
    and wobble with the aim, and the car's speed loop takes ~0.3 s to answer
    (docs/run-analysis.md). Following every wobble means accelerating into a
    dip the car then has to brake for: in the simulator that shed two to three
    times the speed each corner actually needs. Holding the lowest ceiling of
    the last ``hold_sec`` removes that sawtooth. It can only ever lower the
    speed, never raise it above the current ceiling, so no stop or cap is
    weakened.
    """

    def __init__(self, hold_sec: float):
        if not math.isfinite(hold_sec) or hold_sec < 0.0:
            raise ValueError('hold_sec must be finite and non-negative')
        self.hold_sec = hold_sec
        self._samples = []

    def reset(self) -> None:
        self._samples = []

    def update(self, now: float, desired: float) -> float:
        if not math.isfinite(now):
            raise ValueError('now must be finite')
        self._samples = [(t, v) for t, v in self._samples
                         if now - t < self.hold_sec and t <= now]
        self._samples.append((now, desired))
        return min(v for _t, v in self._samples)


def slew_rate_limit(target: float, previous: float, dt: float,
                    increase_rate: float, decrease_rate: float = None) -> float:
    """Limit a normal command's rise and fall per second.

    Emergency stops deliberately do not call this helper. ``decrease_rate``
    defaults to ``increase_rate``, which is convenient for steering.
    """
    if decrease_rate is None:
        decrease_rate = increase_rate
    if not all(math.isfinite(value) for value in (
            target, previous, dt, increase_rate, decrease_rate)):
        raise ValueError('slew-rate inputs must be finite')
    if dt < 0.0 or increase_rate < 0.0 or decrease_rate < 0.0:
        raise ValueError('dt and slew rates must be non-negative')
    lower = previous - decrease_rate * dt
    upper = previous + increase_rate * dt
    return float(np.clip(target, lower, upper))


def disparity_extend(clean: np.ndarray, angle_increment: float,
                     disparity_threshold: float, extend_width_m: float) -> np.ndarray:
    """The standard follow-the-gap 'disparity extender' preprocessing:
    at every sharp jump between adjacent ranges (an obstacle *edge*),
    overwrite the far side with the near side's distance for as many
    beams as `extend_width_m` subtends at that distance.

    A raw scan reports where each beam lands, but the car is not a beam
    -- it is ~30cm wide, and steering right next to an obstacle's edge
    clips it with the side of the chassis. Extending every edge by half
    a car width (plus margin) makes the range array describe where the
    *car's center* can safely go, so anything downstream (bubble, gap
    picking) is automatically clearance-aware at every edge, not just
    around the single closest point.

    Values are only ever lowered (np.minimum), never raised, so this can
    never invent free space. Returns a new array; the input is untouched.
    """
    extended = clean.copy()
    n = len(clean)
    if n < 2 or angle_increment <= 0.0:
        return extended

    jumps = np.abs(np.diff(clean))
    for i in np.nonzero(jumps > disparity_threshold)[0]:
        near = min(clean[i], clean[i + 1])
        if near <= 0.0:
            # The near side is an invalid/contact beam -- there's no
            # meaningful distance to extend at (and atan2(w, 0) would
            # smear a half-circle). The bubble/e-stop path owns this case.
            continue
        num_beams = int(math.ceil(math.atan2(extend_width_m, near) / angle_increment))
        if clean[i] < clean[i + 1]:
            lo, hi = i + 1, min(n, i + 1 + num_beams)
        else:
            lo, hi = max(0, i + 1 - num_beams), i + 1
        extended[lo:hi] = np.minimum(extended[lo:hi], near)
    return extended


def safety_bubble(window: np.ndarray, closest_idx: int, closest_dist: float,
                  angle_increment: float, bubble_width_m: float) -> np.ndarray:
    """Zero out the beams around the closest obstacle so no chosen gap
    can graze it. The bubble's angular radius is whatever half a car
    width (plus margin) actually subtends *at the obstacle's distance* --
    atan2(width, dist) -- rather than a fixed angle: a fixed 20 degrees
    is far too little clearance at 0.3m and wastefully much at 5m.
    Returns a new array; the input is untouched.
    """
    out = window.copy()
    if out.size == 0 or angle_increment <= 0.0:
        return out
    if closest_dist <= 0.0:
        radius_idx = out.size  # contact distance: everything is too close
    else:
        radius_idx = int(math.ceil(math.atan2(bubble_width_m, closest_dist) / angle_increment))
    radius_idx = max(1, radius_idx)
    lo = max(0, closest_idx - radius_idx)
    hi = min(out.size, closest_idx + radius_idx + 1)
    out[lo:hi] = 0.0
    return out


def find_best_gap(window: np.ndarray, min_gap_distance: float,
                  angle_increment: float = 0.0, min_gap_width_m: float = 0.0,
                  previous_target_idx: int = None, switch_margin: float = 1.0):
    """Pick the best drivable opening, not just the widest one.

    A shallow dead end (e.g. a ~1m doorway alcove) can be angularly
    wider than a genuine, much deeper corridor or track opening. Scoring
    candidates by width * average_depth rather than width alone means a
    gap has to actually be open for a while, not just wide at the mouth,
    to win -- so the car stops driving into pockets it can't get back
    out of.

    If `min_gap_width_m` is set (and `angle_increment` supplied), candidates
    narrower than the conservative chord at their nearest depth are rejected.
    Returns (start, end) indices into `window`, or (None, None).

    `previous_target_idx`/`switch_margin` add hysteresis to which candidate
    wins: if the previous tick's aim still falls inside a still-open
    candidate, that candidate is kept unless another one scores at least
    `switch_margin` times better (1.0, the default, disables this -- plain
    best-score, unchanged behaviour). Two similarly-open candidates trading
    the top score by a beam of LiDAR noise is exactly what flips the aim,
    and therefore the steering command, at scan rate -- see
    aim_within_gap's docstring for the 2026-07-27 incident this generalizes
    the fix for, and corridor_centering_bias's own ramps-not-switches
    reasoning, which this mirrors for a discrete choice instead of a
    continuous term. Hysteresis only ever chooses *among* candidates that
    already passed every filter above (depth, post-inflation width) -- it
    cannot keep the car aimed somewhere that stopped being safe, only
    reduce how often it re-aims somewhere equally safe for no real reason.
    """
    free = window > min_gap_distance
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

    def physical_width(run):
        start, end = run
        # Use the chord at the narrowest depth, rather than arc length at
        # average depth. That prevents a wedge-shaped opening from looking
        # wide enough merely because its far edge is deep.
        min_depth = float(np.min(window[start:end + 1]))
        angular_width = min(math.pi, (end - start + 1) * angle_increment)
        return 2.0 * min_depth * math.sin(angular_width / 2.0)

    if min_gap_width_m > 0.0 and angle_increment > 0.0:
        candidates = [run for run in candidates if physical_width(run) >= min_gap_width_m]

    if not candidates:
        return None, None

    def score(run):
        start, end = run
        segment = window[start:end + 1]
        width = end - start + 1
        avg_depth = float(np.mean(segment))
        return width * avg_depth

    best = max(candidates, key=score)
    if previous_target_idx is not None and switch_margin > 1.0:
        sticky = next((run for run in candidates
                       if run[0] <= previous_target_idx <= run[1]), None)
        if (sticky is not None and sticky != best
                and score(best) < score(sticky) * switch_margin):
            best = sticky
    return best


def find_gap_with_fallback(window: np.ndarray, preferred_distance: float,
                           fallback_distance: float, angle_increment: float,
                           min_gap_width_m: float, previous_target_idx: int = None,
                           switch_margin: float = 1.0):
    """Find a deep gap first, then a nearer safe opening for tight corners.

    A fixed deep-range threshold can deadlock follow-the-gap immediately
    before a passable corner: there may be ample width, but the turn hides
    everything beyond the corner. The fallback relaxes only the visibility
    depth; disparity extension and the post-inflation width requirement still
    apply. Returns ``(start, end, used_fallback)``.

    `previous_target_idx`/`switch_margin` pass straight through to
    find_best_gap -- see its docstring. Applied independently within each
    tier (preferred vs fallback), not across the switch between them: which
    tier is even tried is already a depth-based structural decision, not a
    noisy tie between similar candidates.
    """
    start, end = find_best_gap(
        window,
        preferred_distance,
        angle_increment=angle_increment,
        min_gap_width_m=min_gap_width_m,
        previous_target_idx=previous_target_idx,
        switch_margin=switch_margin,
    )
    if start is not None:
        return start, end, False

    if fallback_distance >= preferred_distance:
        return None, None, False
    start, end = find_best_gap(
        window,
        fallback_distance,
        angle_increment=angle_increment,
        min_gap_width_m=min_gap_width_m,
        previous_target_idx=previous_target_idx,
        switch_margin=switch_margin,
    )
    return start, end, start is not None


def aim_within_gap(window, gap_start, gap_end, beam_angles):
    """Pick the beam to steer at inside a chosen gap.

    The midpoint is the classic follow-the-gap answer, and it is the right
    one when both edges are real obstacles -- it centres the car between
    them. It is the wrong one when an edge is just the limit of the
    sensor's field of view, because that edge carries no information about
    where the track goes. A gap running from the -90deg FOV boundary to
    +3deg has a midpoint of -44deg, which points into the wall beside the
    car rather than down the course.

    That is what tore the steering apart on 2026-07-27. Around a corner
    nothing clears the preferred depth, so the fallback gap took over --
    edge-clipped, aiming -44deg -- while on alternate scans a sliver of
    deep space reappeared and aimed +6deg. Desired steering alternated
    +0.26/-0.26rad at scan rate and the slew limiter averaged the pair to
    0.009rad, so the car drove straight into the corner it was supposedly
    turning away from.

    When an edge is FOV-clipped, aim at the deepest beam in the gap
    instead: that is the direction the course actually continues, and it
    agrees with the preferred gap rather than fighting it.

    Depth ties have to be broken deliberately. In open space every beam
    reads max_range, so a plain argmax returns the first index -- which is
    the edge of the field of view, i.e. hard over to one side in exactly
    the situation where the car should go straight. Among the beams that
    are effectively as deep as the deepest, take the one closest to
    straight ahead.
    """
    if gap_start is None:
        return None
    if gap_start != 0 and gap_end != len(window) - 1:
        return (gap_start + gap_end) // 2

    depths = window[gap_start:gap_end + 1]
    deepest = float(np.max(depths))
    near_deepest = np.nonzero(
        depths >= deepest - max(1e-6, 0.05 * deepest))[0]
    angles = beam_angles[gap_start:gap_end + 1][near_deepest]
    return gap_start + int(near_deepest[int(np.argmin(np.abs(angles)))])


def near_gap_bearing(window, gap_start, gap_end, beam_angles, near_depth: float):
    """Mean bearing of the chosen gap's own near-only sub-extent, or None.

    aim_within_gap answers "which way to steer" from the *whole* gap,
    weighted toward its deepest point -- correct for steering, but that
    same depth-weighting is what makes it a poor anticipation signal: a
    bend that is about to matter can sit entirely beyond the depth this
    restricts to, so the two disagree earliest exactly when a corner is
    approaching and hasn't been steered into yet. Comparing this to the
    real aim (aim_within_gap's own result) is the caller's job -- see
    gap_follow_node.py's cornering-anticipation speed cap -- because a
    small numeric difference is meaningless without knowing whether the
    gap it came from was actually the fallback or the preferred one, which
    only the caller still has.

    Returns ``None`` when there is nothing to compare against: no beam in
    the gap is within ``near_depth`` (no near view at all), or every beam
    is (the "near" and "whole gap" views are the same thing -- a small
    room, not a corridor with something beyond the near view to diverge
    from).
    """
    if gap_start is None:
        return None
    if not math.isfinite(near_depth) or near_depth <= 0.0:
        raise ValueError('near_depth must be finite and positive')
    segment = window[gap_start:gap_end + 1]
    seg_angles = beam_angles[gap_start:gap_end + 1]
    near = segment <= near_depth
    if not np.any(near) or np.all(near):
        return None
    return float(np.mean(seg_angles[near]))


def side_wall_distance(clean: np.ndarray, valid: np.ndarray,
                       angles: np.ndarray, centre_angle: float,
                       half_span: float) -> float:
    """Perpendicular distance to the wall on one side of the car.

    Takes the *minimum* valid range in an angular window centred on
    ``centre_angle`` (+pi/2 for the left wall, -pi/2 for the right). The
    minimum is not a conservative fudge, it is the correct estimator: a beam
    striking a locally straight wall at angle ``theta`` away from the
    perpendicular reads ``d / cos(theta)``, which is minimised exactly at the
    perpendicular. So the window recovers the true perpendicular distance even
    when the car is yawed relative to the wall, as long as the yaw stays inside
    ``half_span`` -- and it is simultaneously robust to a doorway or gap in the
    middle of the window, because the nearer surrounding wall still wins.

    Returns ``inf`` when the window contains no valid beam (no wall on that
    side, or the scan does not reach that far around), which the caller must
    read as "no usable wall here" rather than "very far away".
    """
    ranges = np.asarray(clean, dtype=np.float64)
    validity = np.asarray(valid, dtype=bool)
    beam_angles = np.asarray(angles, dtype=np.float64)
    if not (ranges.shape == validity.shape == beam_angles.shape):
        raise ValueError('ranges, validity, and angles must have matching shapes')
    if not math.isfinite(centre_angle):
        raise ValueError('centre_angle must be finite')
    if not math.isfinite(half_span) or half_span <= 0.0:
        raise ValueError('half_span must be finite and positive')

    inside = validity & (np.abs(beam_angles - centre_angle) <= half_span)
    if not np.any(inside):
        return math.inf
    return float(np.min(ranges[inside]))


def _ramp(value: float, full_at: float, zero_at: float) -> float:
    """Linear fade in [0, 1]: 1.0 at ``full_at``, 0.0 at ``zero_at``.

    Works in either direction -- ``full_at < zero_at`` fades out as ``value``
    grows, ``full_at > zero_at`` fades out as it shrinks. Equal endpoints
    degenerate to a hard step, which callers are expected to avoid.
    """
    if not all(math.isfinite(v) for v in (value, full_at, zero_at)):
        raise ValueError('ramp inputs must be finite')
    if full_at == zero_at:
        return 1.0 if value == full_at else 0.0
    fraction = (value - zero_at) / (full_at - zero_at)
    return float(min(1.0, max(0.0, fraction)))


def corridor_centering_bias(left_distance: float, right_distance: float,
                            aim_bearing: float, aim_depth: float,
                            gain: float, max_bias: float,
                            bearing_full: float, bearing_zero: float,
                            depth_full: float, depth_zero: float,
                            side_full: float, side_zero: float):
    """A small steering bias that pulls the car to the middle of a corridor.

    Follow-the-gap answers "which way should I point", never "where in the
    corridor should I be". Those are different questions, and the second one
    is unanswered anywhere else in this node: aiming at the deepest beam sends
    the car parallel to the walls, which holds whatever lateral offset it
    happened to enter the straight with. Enter 0.15m off the left wall and it
    tracks 0.15m off the left wall for the length of the straight, spending
    the whole clearance budget for no reason and starting the next corner from
    the worst possible place.

    This adds the missing cross-track term. Together with the existing
    ``steering_gain * aim_bearing`` heading term it forms the standard
    two-state lane-centring law (cross-track error + heading error), the same
    structure as Stanley control and the classic F1TENTH wall-follower. The
    heading term supplies the damping: as the car turns toward the middle its
    heading tilts, the aim bearing swings the other way, and the two oppose --
    so no derivative term or error history is needed here, and there is
    nothing to wind up.

    Sign convention is ROS REP-103 (positive angle = left). The car sits left
    of centre when ``left_distance < right_distance``, and the returned bias is
    then negative, steering it back right.

    Three independent conditions fade the bias in, all as smooth ramps rather
    than switches -- a hard on/off on a steering term is what produced the
    scan-rate steering chatter documented in ``aim_within_gap``:

      * **the car is going straight** (``aim_bearing`` near zero). This is the
        user-visible contract -- centre on the straights, never fight the gap
        logic mid-corner -- and it also targets the failure mode precisely.
        A large aim bearing already means either a corner, where the racing
        line is deliberately not the middle, or an off-centre car that the
        midpoint aim is *already* correcting.
      * **there is a straight to centre in** (``aim_depth`` far enough ahead).
        No point centring for a wall 1m away.
      * **both walls are real** (each within ``side_zero``). Without this, a
        doorway or an opening on one side reads as "acres of room over there"
        and the bias would steer into it. An unbounded side is not a wall.

    Returns ``(bias_rad, weight)``. The bias is already clamped to
    ``+/-max_bias`` and scaled by the weight; the weight is returned only so
    the caller can log why the bias is what it is. The clamp is the safety
    property that matters: this term is a bounded nudge that can never
    outvote the gap the obstacle-avoidance pipeline chose, and it is applied
    before the node's existing steering clip and slew limiter, so every
    downstream safety layer still sees a command it can shape.
    """
    if not all(math.isfinite(v) for v in (gain, max_bias)):
        raise ValueError('centering gain and max_bias must be finite')
    if gain < 0.0 or max_bias < 0.0:
        raise ValueError('centering gain and max_bias must be non-negative')
    if not math.isfinite(aim_bearing):
        raise ValueError('aim_bearing must be finite')

    # An unbounded side is not a wall to centre against.
    if not (math.isfinite(left_distance) and math.isfinite(right_distance)):
        return 0.0, 0.0

    weight = (
        _ramp(abs(aim_bearing), bearing_full, bearing_zero)
        * _ramp(aim_depth, depth_full, depth_zero)
        * _ramp(left_distance, side_full, side_zero)
        * _ramp(right_distance, side_full, side_zero)
    )
    if weight <= 0.0:
        return 0.0, 0.0

    # Lateral offset from the middle, positive when the car sits left of it.
    # The car body is symmetric, so its half-width cancels in the difference
    # and raw ranges are the right thing to subtract here.
    offset_from_middle = (right_distance - left_distance) / 2.0
    bias = -gain * offset_from_middle
    bias = float(min(max_bias, max(-max_bias, bias)))
    return weight * bias, weight


def side_opening_signal(clean: np.ndarray, valid: np.ndarray,
                        angles: np.ndarray, left_distance: float,
                        right_distance: float, sector_lo: float,
                        sector_hi: float, excess_threshold: float) -> float:
    """Which side of the corridor opens up ahead: +1 left .. -1 right.

    For every beam in a forward-diagonal sector on each side, compare the
    range with where that side's wall *would* be if it carried straight on
    (``side_distance / sin(bearing)``). A beam that reaches more than
    ``excess_threshold`` further has gone past the end of the wall -- around
    an inner corner, through the opening of a turn, or past a wall that is
    curving away. The signal is the fraction of left-sector beams that do so,
    minus the fraction of right-sector beams.

    Why not something simpler (both were tried, docs/gap-follow.md):

      * the steering aim's bearing wanders a few degrees either side of zero
        on a straight, and at a square inner corner leans *away* from the
        turn because the inflated corner clips the gap's inside edge;
      * the deepest beam's bearing points the wrong way ~2m before a square
        corner, where the far outer corner of the end wall is the deepest
        return in view.

    This one is measured against each side's own wall, so a car sitting off
    centre on a plain straight reads exactly zero, and it can only read a
    turn on the side whose wall actually ends. On a square 1.5m hallway it
    starts responding ~3m before the corner and grows steadily to the
    turn-in point. A doorway reads like a small turn while it is in the
    sector; corner setup's other gates (an end wall within reach, a wall on
    both sides) and its authority bound keep that a brief bounded nudge.

    Returns 0.0 if either side has no wall to project (``inf``).
    """
    ranges = np.asarray(clean, dtype=np.float64)
    validity = np.asarray(valid, dtype=bool)
    beam_angles = np.asarray(angles, dtype=np.float64)
    if not (ranges.shape == validity.shape == beam_angles.shape):
        raise ValueError('ranges, validity, and angles must have matching shapes')
    if not (math.isfinite(sector_lo) and math.isfinite(sector_hi)
            and 0.0 < sector_lo < sector_hi < math.pi):
        raise ValueError('sector must satisfy 0 < sector_lo < sector_hi < pi')
    if not (math.isfinite(excess_threshold) and excess_threshold > 0.0):
        raise ValueError('excess_threshold must be finite and positive')
    if not (math.isfinite(left_distance) and math.isfinite(right_distance)):
        return 0.0

    def opening_fraction(side, wall_distance):
        bearing = side * beam_angles
        in_sector = validity & (bearing >= sector_lo) & (bearing <= sector_hi)
        if not np.any(in_sector):
            return 0.0
        expected = wall_distance / np.sin(bearing[in_sector])
        return float(np.mean(ranges[in_sector] - expected > excess_threshold))

    return opening_fraction(1.0, left_distance) - opening_fraction(-1.0, right_distance)


def corner_setup_bias(left_distance: float, right_distance: float,
                      turn_signal: float, straight_clearance: float,
                      car_width: float, wall_margin: float,
                      setup_fraction: float, gain: float, max_bias: float,
                      signal_zero: float, signal_full: float,
                      far_zero: float, far_full: float,
                      near_full: float, near_zero: float,
                      side_full: float, side_zero: float):
    """Steering bias that moves the car to the OUTSIDE of an upcoming turn.

    Follow-the-gap starts turning the moment a bend's opening comes into view:
    the gap's midpoint leans into the turn, so the car drifts to the inside of
    the corridor long before the corner and arrives at it on the inside line,
    which is the tightest radius it could have picked. Measured on
    indoor_tight that line put the inner corner ~0.08m from the body and the
    TTC brake fired on the way into every corner (docs/gap-follow.md).

    A racing line does the opposite -- outside before the turn, late apex,
    outside on exit -- and this term supplies the first part. It is the same
    cross-track law as corridor_centering_bias, aimed at a point offset toward
    the outside wall instead of the middle:

        target   = setup_fraction * (corridor_half - car_width/2 - wall_margin)
        position = how far the car already sits toward the outside wall
        bias     = gain * (target - position), clamped to +/-max_bias,
                   signed to steer toward the outside

    ``turn_signal`` says which way the corridor turns and how surely: the
    node passes side_opening_signal() (positive = opening on the left).
    REP-103: a left turn has its outside on the right, so a car short of the
    target gets a negative (rightward) bias.

    Faded in by four smooth ramps, all multiplied, so it acts only on the
    approach to a corner in a corridor:

      * turn confidence -- ``|turn_signal|`` from ``signal_zero`` (0) up to
        ``signal_full`` (1). On a straight neither side opens and this is
        off.
      * the end of the straight is near -- ``straight_clearance`` (how far
        the body can drive straight ahead, swept_path_clearance at zero
        curvature) under ``far_zero`` .. ``far_full``.
      * the corner is not yet here -- the same clearance above
        ``near_zero`` .. ``near_full``. Below that the car is at the turn-in
        point and must be free to turn; holding it wide any longer would
        hold it into the outer wall.
      * both walls are real -- each side within ``side_zero`` .. ``side_full``
        (a doorway or an open room is not a corridor to set up in).

    Returns ``(bias_rad, weight)``; the weight lets the caller fade
    corridor_centering_bias out while this is active, so the two do not
    fight over the same lateral position.
    """
    values = (left_distance, right_distance, turn_signal, car_width,
              wall_margin, setup_fraction, gain, max_bias)
    if not all(math.isfinite(v) for v in values[2:]):
        raise ValueError('corner setup inputs must be finite')
    if gain < 0.0 or max_bias < 0.0 or wall_margin < 0.0:
        raise ValueError('gain, max_bias and wall_margin must be non-negative')
    if not 0.0 <= setup_fraction <= 1.0:
        raise ValueError('setup_fraction must be in [0, 1]')
    if not (math.isfinite(left_distance) and math.isfinite(right_distance)):
        return 0.0, 0.0
    if math.isnan(straight_clearance):
        raise ValueError('straight_clearance must not be NaN')
    # inf means nothing ahead within the horizon: no corner coming.
    if straight_clearance == math.inf:
        return 0.0, 0.0

    weight = (
        _ramp(abs(turn_signal), signal_full, signal_zero)
        * _ramp(straight_clearance, far_full, far_zero)
        * _ramp(straight_clearance, near_full, near_zero)
        * _ramp(left_distance, side_full, side_zero)
        * _ramp(right_distance, side_full, side_zero)
    )
    if weight <= 0.0:
        return 0.0, 0.0

    turn_sign = 1.0 if turn_signal > 0.0 else -1.0
    corridor_half = (left_distance + right_distance) / 2.0
    room = max(0.0, corridor_half - car_width / 2.0 - wall_margin)
    target = setup_fraction * room
    # Positive = toward the outside wall. For a left turn the outside is the
    # right, and a car right of the middle has left_distance > right_distance.
    position = turn_sign * (left_distance - right_distance) / 2.0
    magnitude = min(max_bias, max(-max_bias, gain * (target - position)))
    return -turn_sign * magnitude * weight, weight


def steering_damping(yaw_rate: float, gain_s: float) -> float:
    """Steering, in radians, that opposes the car's own rotation.

    ``yaw_rate`` is the gyro's z rate in rad/s from /imu/data, positive to the
    left (imu_bridge's car axes: x forward, y left, z up). ``gain_s`` is a time
    constant in seconds, and the term is ``-gain_s * yaw_rate``: a car turning
    left is steered right by gain_s radians per rad/s. gap_follow_node adds it
    to the requested steering before the clip, the arc guard and the slew limit
    (docs/gap-follow.md#steering-damping-gyro), so everything downstream checks
    the damped command.

    Exactly zero whenever the term cannot be trusted: a non-finite yaw rate or
    gain (never NaN steering), or a gain that is not positive. A negative gain
    would steer *with* the rotation, so it switches the term off rather than
    flipping it.
    """
    if not (math.isfinite(yaw_rate) and math.isfinite(gain_s)) or gain_s <= 0.0:
        return 0.0
    return -gain_s * yaw_rate


# ============================================================================
# Adaptive grip from the gyro: how much of the turn the tyres are making
# (docs/gap-follow.md#adaptive-grip-imu)
# ============================================================================

# Time constant, in seconds, of the low-pass on the grip ratio before it is
# judged. A constant rather than a parameter: the spec fixes it.
GRIP_RATIO_TAU_SEC = 0.3


def grip_ratio(yaw_rate: float, speed: float, steering: float,
               wheelbase: float) -> float:
    """How much of the requested turn the car is making: the yaw rate the gyro
    measures, over the yaw rate the steering geometry asks for.

    A car that does not slip yaws at speed * tan(steering) / wheelbase, so the
    ratio is 1.0 when the tyres hold the line the rack asks for. Below 1.0 the
    car yaws less than the geometry says, which is what sliding wide looks
    like. ``yaw_rate`` is positive to the left, the same sign as ``steering``.

    NaN, meaning no judgement, for any non-finite input, for speed below
    1 m/s (the ratio divides by speed squared), and for a rack within 0.05 rad
    of centre (the demand is too small to tell a slip from noise).
    """
    if not all(math.isfinite(value) for value in (yaw_rate, speed, steering, wheelbase)):
        return math.nan
    if wheelbase <= 0.0:
        raise ValueError('wheelbase must be positive')
    if speed < 1.0 or abs(steering) < 0.05:
        return math.nan
    demanded = speed * speed * math.tan(steering) / wheelbase
    return yaw_rate * speed / demanded


class LowPass:
    """First-order low-pass over a stream of samples, with time constant
    ``time_constant_s``.

    A NaN sample is not a value, so it breaks the chain: the filter forgets
    its state, and the next finite sample starts it afresh. reset() does the
    same. Nothing from before a gap is blended into what comes after it.
    """

    def __init__(self, time_constant_s: float):
        if not (math.isfinite(time_constant_s) and time_constant_s > 0.0):
            raise ValueError('time_constant_s must be finite and positive')
        self.time_constant_s = float(time_constant_s)
        self.value = None

    def reset(self) -> None:
        self.value = None

    def update(self, sample: float, dt: float) -> float:
        """Feed one sample taken ``dt`` seconds after the previous one. Returns
        the filtered value, or NaN when there is none."""
        if not (math.isfinite(dt) and dt >= 0.0):
            raise ValueError('dt must be finite and non-negative')
        if not math.isfinite(sample):
            self.value = None
            return math.nan
        if self.value is None:
            self.value = float(sample)
        else:
            alpha = 1.0 - math.exp(-dt / self.time_constant_s)
            self.value += alpha * (float(sample) - self.value)
        return self.value


@dataclass
class AdaptiveGrip:
    """The adaptive lateral-acceleration limit, in m/s^2.

    ``base`` is max_lateral_accel: the limit never drops below it. ``ceiling``
    is the highest it may rise to. ``limit`` starts at the base. ``update``
    judges one tick and returns the limit to use on that tick.
    """
    base: float
    ceiling: float
    rise_per_s: float
    cut_factor: float
    hold_ratio: float
    slip_ratio: float
    judge_fraction: float
    limit: float = None

    def __post_init__(self):
        values = (self.base, self.ceiling, self.rise_per_s, self.cut_factor,
                  self.hold_ratio, self.slip_ratio, self.judge_fraction)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('adaptive grip parameters must be finite')
        if self.limit is None:
            self.limit = self.base
        if not math.isfinite(self.limit):
            raise ValueError('adaptive grip limit must be finite')
        if self.base <= 0.0:
            raise ValueError('base must be positive')
        if self.ceiling < self.base:
            raise ValueError('ceiling must be at least base')
        if self.rise_per_s < 0.0:
            raise ValueError('rise_per_s must be non-negative')
        if not 0.0 < self.cut_factor <= 1.0:
            raise ValueError('cut_factor must be in (0, 1]')
        if not self.slip_ratio < self.hold_ratio:
            raise ValueError('slip_ratio must be below hold_ratio, or the two bands overlap')
        if self.judge_fraction < 0.0:
            raise ValueError('judge_fraction must be non-negative')
        if not self.base <= self.limit <= self.ceiling:
            raise ValueError('limit must lie between base and ceiling')

    def update(self, ratio: float, demand: float, dt: float) -> float:
        """Judge one tick and return the limit to use on it.

        The tick is not judged, and the limit stays where it is, when the
        ratio is NaN, when the demand is not finite, or when the demand is
        below judge_fraction of the limit (the car is not asking the tyres for
        much). Otherwise a ratio at or above hold_ratio raises the limit by
        rise_per_s * dt, up to the ceiling. A ratio below slip_ratio cuts it to
        cut_factor of itself, never below the base. Between the two it holds.
        """
        if not (math.isfinite(dt) and dt >= 0.0):
            raise ValueError('dt must be finite and non-negative')
        if not (math.isfinite(ratio) and math.isfinite(demand)):
            return self.limit
        if demand < self.judge_fraction * self.limit:
            return self.limit
        if ratio >= self.hold_ratio:
            self.limit = min(self.ceiling, self.limit + self.rise_per_s * dt)
        elif ratio < self.slip_ratio:
            self.limit = max(self.base, self.limit * self.cut_factor)
        return self.limit

    def follow_base(self, base: float, configured_ceiling: float) -> None:
        """Track a new base. max_lateral_accel is live-tunable and is the floor,
        so the band follows it. A change restarts the limit at the new base.
        The ceiling is never below the base, so the band stays valid."""
        if not (math.isfinite(base) and base > 0.0):
            raise ValueError('base must be finite and positive')
        if base == self.base:
            return
        self.base = float(base)
        self.ceiling = max(float(configured_ceiling), self.base)
        self.limit = self.base


# Reading the lag back off the control clock. The clock is a running sum of
# float intervals, so a sample exactly lag_sec old can land a few 1e-17 s the
# wrong side of the window's edge. This slack counts such a sample as covering
# the edge.
LAG_SLACK_SEC = 1e-9


class SteeringLag:
    """The rack steering on each control tick, time-stamped on the control
    clock, so the steering commanded ``lag_sec`` ago can be read back.

    The measured yaw trails the rack, so the grip compares the yaw with the
    steering that produced it (docs/gap-follow.md#adaptive-grip-imu). The clock
    is the sum of the command intervals, which the node and the simulator both
    have, not the wall clock, so the two judge identically tick for tick. A stop
    neither advances the clock nor is recorded: the rack is held during a stop
    (gap_follow_node._stop), so nothing is lost.
    """

    def __init__(self, lag_sec: float):
        if not (math.isfinite(lag_sec) and lag_sec >= 0.0):
            raise ValueError('lag_sec must be finite and non-negative')
        self.lag_sec = float(lag_sec)
        self._clock = 0.0
        self._samples = deque()   # (control time, steering), oldest first

    def step(self, dt: float, steering: float, max_change: float) -> float:
        """Record this tick's steering, and return the steering to judge against.

        That is the steering commanded ``lag_sec`` ago, interpolated between the
        two samples either side of that moment. It is returned only when the
        history reaches back that far and the steering moved no more than
        ``max_change`` across the window (quasi-steady). Otherwise NaN: this
        tick is not judged.
        """
        if not (math.isfinite(dt) and dt >= 0.0):
            raise ValueError('dt must be finite and non-negative')
        if not math.isfinite(steering):
            raise ValueError('steering must be finite')
        if not (math.isfinite(max_change) and max_change >= 0.0):
            raise ValueError('max_change must be finite and non-negative')
        self._clock += dt
        self._samples.append((self._clock, float(steering)))
        edge = self._clock - self.lag_sec
        lagged = self._value_at(edge)
        self._trim(edge)
        # NaN compares false, so a window the history does not cover is never judged.
        # NaN compares false, so a window the history does not cover is never judged.
        if abs(steering - lagged) <= max_change:
            return lagged
        return math.nan

    def _value_at(self, t: float) -> float:
        left = right = None
        for time, value in self._samples:
            if time <= t + LAG_SLACK_SEC:
                left = (time, value)
            else:
                right = (time, value)
                break
        if left is None:
            return math.nan                  # the history does not reach back to t
        if right is None:
            return left[1]
        return left[1] + (right[1] - left[1]) * (t - left[0]) / (right[0] - left[0])

    def _trim(self, t: float) -> None:
        # The clock only moves forward, so only the last sample at or before t
        # is ever read again. Older samples go.
        while len(self._samples) >= 2 and self._samples[1][0] <= t + LAG_SLACK_SEC:
            self._samples.popleft()


def adaptive_grip_step(grip: AdaptiveGrip, ratio_filter: LowPass, yaw_rate: float,
                       speed: float, steering: float, wheelbase: float,
                       dt: float) -> float:
    """One tick of the adaptive grip with a healthy gyro: the grip ratio,
    low-passed, judged against the demand the steering asks for. Returns the
    lateral limit to use on this tick.

    ``steering`` is the steering to judge against: the value SteeringLag.step
    returns, which is the rack's steering from a lag ago when it is steady and
    NaN when this tick is not judged. The demand is speed squared times
    |tan(steering)| over the wheelbase: the lateral acceleration that steering
    asks the tyres for. Call this only while the gyro can be trusted
    (gap_follow_node._imu_gate).
    """
    ratio = ratio_filter.update(grip_ratio(yaw_rate, speed, steering, wheelbase), dt)
    if math.isfinite(speed) and math.isfinite(steering):
        demand = speed * speed * abs(math.tan(steering)) / wheelbase
    else:
        demand = math.nan
    return grip.update(ratio, demand, dt)


def corridor_width_factor(left_distance: float, right_distance: float,
                          narrow_width: float, reference_width: float) -> float:
    """How open the sensed corridor is right now, as a 0..1 fade.

    1.0 at ``reference_width`` or wider -- the width today's static
    defaults (``safety_margin``, ``corner_speed``) are already tuned for,
    so nothing changes. 0.0 at ``narrow_width`` or tighter. Linear between.

    An unbounded or missing side is not a corridor to measure -- returns
    1.0, the same "assume the wide, already-validated defaults, do not
    loosen anything" answer ``corridor_centering_bias`` gives for an open
    side. In practice this means the adaptation mostly acts on narrow
    *straights*, where both walls are reliably in view, and quietly steps
    back to today's proven behaviour deep in a blind corner where one side
    can drop out of the scan window -- which is also where extra caution
    is most warranted, not less.
    """
    if not (math.isfinite(narrow_width) and math.isfinite(reference_width)):
        raise ValueError('narrow_width and reference_width must be finite')
    if not (0.0 < narrow_width < reference_width):
        raise ValueError('narrow_width must be positive and less than reference_width')
    if not (math.isfinite(left_distance) and math.isfinite(right_distance)):
        return 1.0
    measured_width = left_distance + right_distance
    return _ramp(measured_width, reference_width, narrow_width)


def scale_between(width_factor: float, narrow_value: float, wide_value: float) -> float:
    """Linear blend from ``narrow_value`` (width_factor=0) to ``wide_value`` (1).

    The one piece of arithmetic both adaptive knobs below share -- the
    margin shrinks toward ``narrow_value`` and the corner-speed cap grows
    toward ``wide_value`` using the same ``width_factor``, so this is
    written and tested once rather than twice.
    """
    if not all(math.isfinite(v) for v in (width_factor, narrow_value, wide_value)):
        raise ValueError('scale_between inputs must be finite')
    if not (0.0 <= width_factor <= 1.0):
        raise ValueError('width_factor must be in [0, 1]')
    return narrow_value + width_factor * (wide_value - narrow_value)
