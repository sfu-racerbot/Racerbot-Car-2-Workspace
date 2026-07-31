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
  aim_within_gap      -> which beam inside that gap to steer at
  side_wall_distance  -> perpendicular distance to the wall on one side
  corridor_centering  -> bounded cross-track bias, faded in on straights
"""

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
                        max_braking_decel: float, max_speed: float) -> float:
    """Clearance speed ceiling from ``v^2 <= 2*a*(clearance-reserve)``.

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
    usable_distance = max(0.0, clearance - reserve_distance)
    return min(max_speed, math.sqrt(2.0 * max_braking_decel * usable_distance))


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
                  angle_increment: float = 0.0, min_gap_width_m: float = 0.0):
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

    best_start, best_end = max(candidates, key=score)
    return best_start, best_end


def find_gap_with_fallback(window: np.ndarray, preferred_distance: float,
                           fallback_distance: float, angle_increment: float,
                           min_gap_width_m: float):
    """Find a deep gap first, then a nearer safe opening for tight corners.

    A fixed deep-range threshold can deadlock follow-the-gap immediately
    before a passable corner: there may be ample width, but the turn hides
    everything beyond the corner. The fallback relaxes only the visibility
    depth; disparity extension and the post-inflation width requirement still
    apply. Returns ``(start, end, used_fallback)``.
    """
    start, end = find_best_gap(
        window,
        preferred_distance,
        angle_increment=angle_increment,
        min_gap_width_m=min_gap_width_m,
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
