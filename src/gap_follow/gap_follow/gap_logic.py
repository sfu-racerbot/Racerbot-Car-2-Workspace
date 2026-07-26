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
  closest_valid       -> safety-bubble anchor (valid beams only)
  disparity_extend    -> widen every obstacle *edge* by half a car width
  safety_bubble       -> zero out a car-width bubble around the closest hit
  find_gap_with...    -> preferred deep gap, then a slow corner fallback
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
    """Safest speed when fresh odometry is absent or effectively zero."""
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

    speed = max(0.0, measured_speed)
    if (measured_speed <= fallback_max_measured_speed
            and command_age_sec <= command_timeout_sec):
        speed = max(speed, commanded_speed)
    return max(0.0, speed)


def time_to_collision(clean: np.ndarray, valid: np.ndarray,
                      angles: np.ndarray, speed: float,
                      boundary_distances: np.ndarray,
                      min_closing_speed: float = 0.05) -> np.ndarray:
    """Instantaneous TTC for every scan beam, with the car footprint removed.

    This is the F1TENTH iTTC construction: longitudinal odometry speed is
    projected onto each beam with ``speed * cos(angle)``. Beams the car is not
    approaching have infinite TTC. Raw ranges are converted to body clearance
    first, so the clock reaches zero when the rectangular car body reaches the
    obstacle, not when the LiDAR itself does.
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
    ttc = np.full(ranges.shape, np.inf, dtype=np.float64)
    clearances = np.maximum(0.0, ranges - boundaries)
    ttc[approaching] = clearances[approaching] / closing_speed[approaching]
    return ttc


def minimum_ttc(clean: np.ndarray, valid: np.ndarray,
                angles: np.ndarray, speed: float,
                boundary_distances: np.ndarray,
                min_closing_speed: float = 0.05) -> float:
    """Return the minimum finite footprint-aware iTTC, or infinity."""
    ttc = time_to_collision(
        clean, valid, angles, speed, boundary_distances, min_closing_speed)
    return float(np.min(ttc)) if ttc.size else math.inf


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
