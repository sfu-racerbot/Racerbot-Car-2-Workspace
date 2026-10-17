"""Pure calibration math for the odometry wizard.

This module deliberately has no ROS, Tornado, filesystem, or browser imports.
Every calculation used in the final report can therefore be tested with small
synthetic data sets before it is trusted with measurements from the car.
"""

from __future__ import annotations

import bisect
import math
import statistics
from typing import Iterable

from odom_calibration import steering_curve


EPSILON = 1e-9


def _finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) \
        and math.isfinite(float(value))


def median(values: Iterable[float]):
    clean = [float(value) for value in values if _finite(value)]
    return statistics.median(clean) if clean else None


def percentile(values: Iterable[float], fraction: float):
    """Linearly interpolated percentile for fraction in [0, 1]."""
    if not 0.0 <= fraction <= 1.0:
        raise ValueError('fraction must be in [0, 1]')
    clean = sorted(float(value) for value in values if _finite(value))
    if not clean:
        return None
    if len(clean) == 1:
        return clean[0]
    position = fraction * (len(clean) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return clean[lower]
    weight = position - lower
    return clean[lower] * (1.0 - weight) + clean[upper] * weight


def median_absolute_deviation(values: Iterable[float]):
    clean = [float(value) for value in values if _finite(value)]
    centre = median(clean)
    if centre is None:
        return None
    return median(abs(value - centre) for value in clean)


def integrate_samples(samples, field: str, max_gap_sec: float = 0.5):
    """Trapezoid-integrate a timestamped field, skipping unsafe time gaps.

    Samples are dictionaries with monotonic receive time ``t``. Invalid
    readings and non-increasing timestamps are rejected rather than allowed to
    poison a calibration. A gap larger than ``max_gap_sec`` is not integrated:
    assuming the signal stayed constant across missing telemetry would silently
    manufacture distance.
    """
    if not _finite(max_gap_sec) or max_gap_sec <= 0.0:
        raise ValueError('max_gap_sec must be finite and positive')

    valid = []
    rejected = 0
    for sample in samples:
        timestamp = sample.get('t')
        value = sample.get(field)
        if not _finite(timestamp) or not _finite(value):
            rejected += 1
            continue
        valid.append((float(timestamp), float(value)))
    valid.sort(key=lambda item: item[0])

    integral = 0.0
    integrated_duration = 0.0
    skipped_duration = 0.0
    non_increasing = 0
    max_observed_gap = 0.0
    for (time_a, value_a), (time_b, value_b) in zip(valid, valid[1:]):
        dt = time_b - time_a
        if dt <= 0.0:
            non_increasing += 1
            continue
        max_observed_gap = max(max_observed_gap, dt)
        if dt > max_gap_sec:
            skipped_duration += dt
            continue
        integral += 0.5 * (value_a + value_b) * dt
        integrated_duration += dt

    return {
        'integral': integral,
        'duration_sec': integrated_duration,
        'skipped_duration_sec': skipped_duration,
        'sample_count': len(valid),
        'rejected_samples': rejected,
        'non_increasing_timestamps': non_increasing,
        'max_gap_sec': max_observed_gap,
    }


class RunningIntegral:
    """Incremental twin of ``integrate_samples`` for live progress readouts.

    Accepts samples in arrival order and applies the same rules: non-finite
    values and non-increasing timestamps are ignored, and a gap longer than
    ``max_gap_sec`` is not integrated.
    """

    def __init__(self, max_gap_sec: float = 0.5):
        if not _finite(max_gap_sec) or max_gap_sec <= 0.0:
            raise ValueError('max_gap_sec must be finite and positive')
        self.max_gap_sec = float(max_gap_sec)
        self.total = 0.0
        self._last = None

    def add(self, t, value):
        if not _finite(t) or not _finite(value):
            return
        t, value = float(t), float(value)
        if self._last is not None:
            last_t, last_value = self._last
            dt = t - last_t
            if dt <= 0.0:
                return
            if dt <= self.max_gap_sec:
                self.total += 0.5 * (last_value + value) * dt
        self._last = (t, value)


def _series_stats(samples, field: str):
    values = [
        float(sample[field])
        for sample in samples
        if _finite(sample.get(field))
    ]
    return {
        'count': len(values),
        'median': median(values),
        'mean': statistics.fmean(values) if values else None,
        'p05': percentile(values, 0.05),
        'p95': percentile(values, 0.95),
        'minimum': min(values) if values else None,
        'maximum': max(values) if values else None,
    }


def _wrapped_delta(start: float, end: float):
    return math.atan2(math.sin(end - start), math.cos(end - start))


# A steering sample only describes the turn being measured while the driver
# holds LB and the car is actually rolling. Before LB is pressed and after it
# is released, teleop sends a centred command, so the servo sits on the
# configured offset; when that idle time outweighs the drive, a plain median
# over the whole capture reads the offset back for every circle and the
# fitted gain collapses to zero (2026-10-06 session).
MIN_MOVING_SPEED_MPS = 0.15
# A reading older than this cannot vouch for a later sample.
SAMPLE_MATCH_AGE_SEC = 0.5
# Fewer filtered servo samples than this cannot represent a circle or a drift.
MIN_ACTIVE_SERVO_SAMPLES = 10


class _Lookup:
    """Latest sample at or before a time, ignoring samples older than max_age."""

    def __init__(self, samples, field, max_age=SAMPLE_MATCH_AGE_SEC):
        pairs = sorted(
            (float(sample['t']), sample.get(field))
            for sample in samples
            if _finite(sample.get('t'))
        )
        self.times = [time for time, _ in pairs]
        self.values = [value for _, value in pairs]
        self.max_age = max_age

    def at(self, time):
        index = bisect.bisect_right(self.times, time) - 1
        if index < 0 or time - self.times[index] > self.max_age:
            return None
        return self.values[index]


def active_samples(capture: dict, topic: str, require_moving: bool,
                   min_speed: float = MIN_MOVING_SPEED_MPS):
    """Samples of ``topic`` taken while LB was held (and, optionally, moving).

    A sample with no recent ``/joy`` (or ``/odom`` when movement is required)
    is excluded: unknown is not the same as held.
    """
    lb = _Lookup(capture.get('joy', []), 'lb_held')
    speed = _Lookup(capture.get('odom', []), 'speed')
    kept = []
    for sample in capture.get(topic, []):
        time = sample.get('t')
        if not _finite(time):
            continue
        if lb.at(float(time)) is not True:
            continue
        if require_moving:
            moving = speed.at(float(time))
            if not _finite(moving) or abs(float(moving)) < min_speed:
                continue
        kept.append(sample)
    return kept


def summarize_capture(capture: dict, max_gap_sec: float = 0.5):
    """Convert raw asynchronous topic samples into one reviewable trial."""
    odom = list(capture.get('odom', []))
    vesc = list(capture.get('vesc', []))
    servo = list(capture.get('servo', []))
    drive = list(capture.get('drive', []))
    joy = list(capture.get('joy', []))
    started = capture.get('started_monotonic')
    ended = capture.get('ended_monotonic')
    wall_duration = (
        float(ended) - float(started)
        if _finite(started) and _finite(ended) and float(ended) >= float(started)
        else 0.0
    )

    odom_distance = integrate_samples(odom, 'speed', max_gap_sec)
    odom_yaw = integrate_samples(odom, 'angular_z', max_gap_sec)
    raw_erpm = integrate_samples(vesc, 'raw_forward_erpm', max_gap_sec)
    command_distance = integrate_samples(drive, 'speed', max_gap_sec)

    pose_displacement = None
    pose_yaw_delta = None
    finite_pose = [
        sample for sample in odom
        if all(_finite(sample.get(field)) for field in ('x', 'y', 'yaw'))
    ]
    if len(finite_pose) >= 2:
        first = finite_pose[0]
        last = finite_pose[-1]
        pose_displacement = math.hypot(
            float(last['x']) - float(first['x']),
            float(last['y']) - float(first['y']),
        )
        pose_yaw_delta = _wrapped_delta(float(first['yaw']), float(last['yaw']))

    lb_samples = [
        bool(sample.get('lb_held'))
        for sample in joy
        if sample.get('lb_held') is not None
    ]
    lb_held_fraction = (
        sum(lb_samples) / len(lb_samples) if lb_samples else None
    )

    warnings = []
    if wall_duration < 1.0:
        warnings.append('Capture is shorter than 1 second.')
    if odom_distance['sample_count'] < 5:
        warnings.append('Too few valid odometry samples were recorded.')
    if raw_erpm['sample_count'] < 5:
        warnings.append(
            'Too few raw VESC samples; movement scale may use odometry fallback.'
        )
    for label, result in (
            ('odometry', odom_distance),
            ('raw VESC', raw_erpm),
            ('drive command', command_distance)):
        if result['skipped_duration_sec'] > 0.0:
            warnings.append(
                f"{label} had telemetry gaps; "
                f"{result['skipped_duration_sec']:.2f}s was not integrated."
            )
        if result['rejected_samples']:
            warnings.append(
                f"{label} contained {result['rejected_samples']} non-finite samples."
            )
    if capture.get('truncated_topics'):
        warnings.append(
            'The capture reached its sample limit on: '
            + ', '.join(sorted(capture['truncated_topics']))
            + '.'
        )

    odom_signed = odom_distance['integral']
    raw_signed = raw_erpm['integral']
    if abs(odom_signed) > 0.02 and abs(raw_signed) > 50.0:
        if math.copysign(1.0, odom_signed) != math.copysign(1.0, raw_signed):
            warnings.append(
                'Odometry and forward-positive raw ERPM disagree on direction.'
            )

    kind = capture.get('kind')
    servo_active = _series_stats(
        active_samples(capture, 'servo', require_moving=True), 'value')
    if kind in STEERING_KINDS and kind != 'steering_center' and servo:
        if servo_active['count'] < MIN_ACTIVE_SERVO_SAMPLES:
            warnings.append(
                f"Only {servo_active['count']} servo samples were recorded "
                'while LB was held and the car was moving; this run cannot '
                'be used for steering. Hold LB the whole time you drive.'
            )
    return {
        'servo_active': servo_active,
        'kind': capture.get('kind'),
        'started_at': capture.get('started_at'),
        'duration_sec': wall_duration,
        'odom_distance_m': odom_signed,
        'odom_integration': odom_distance,
        'odom_yaw_rad': odom_yaw['integral'],
        'odom_yaw_integration': odom_yaw,
        'pose_displacement_m': pose_displacement,
        'pose_yaw_delta_rad': pose_yaw_delta,
        'raw_erpm_integral': raw_signed,
        'raw_erpm_integration': raw_erpm,
        'command_distance_m': command_distance['integral'],
        'command_integration': command_distance,
        'odom_speed': _series_stats(odom, 'speed'),
        'raw_forward_erpm': _series_stats(vesc, 'raw_forward_erpm'),
        'servo': _series_stats(servo, 'value'),
        'command_speed': _series_stats(drive, 'speed'),
        'command_steering': _series_stats(drive, 'steering'),
        'lb_held_fraction': lb_held_fraction,
        'dropped_samples': dict(capture.get('dropped_samples', {})),
        'warnings': warnings,
    }


def _robust_positive_candidates(candidates):
    """Return inliers, rejected values, median, and relative robust spread."""
    positive = [float(value) for value in candidates if _finite(value) and value > 0.0]
    if not positive:
        return [], [], None, None
    centre = median(positive)
    mad = median_absolute_deviation(positive)
    if len(positive) < 3 or mad is None:
        inliers = positive
    else:
        robust_sigma = 1.4826 * mad
        tolerance = max(3.5 * robust_sigma, abs(centre) * 0.02, 1.0)
        inliers = [value for value in positive if abs(value - centre) <= tolerance]
    rejected = [value for value in positive if value not in inliers]
    result = median(inliers)
    result_mad = median_absolute_deviation(inliers)
    relative_spread = (
        1.4826 * result_mad / abs(result)
        if result and result_mad is not None
        else 0.0
    )
    return inliers, rejected, result, relative_spread


def parameter_values_from_response(names, response):
    """Decode a ROS GetParameters response without depending on rclpy."""
    if len(response.values) != len(names):
        raise ValueError('live parameter response has the wrong number of values')
    result = {}
    for name, value in zip(names, response.values):
        if value.type == 2:  # rcl_interfaces/ParameterType.PARAMETER_INTEGER
            number = float(value.integer_value)
        elif value.type == 3:  # PARAMETER_DOUBLE
            number = float(value.double_value)
        else:
            raise ValueError(f'{name} must be a numeric parameter')
        if not math.isfinite(number):
            raise ValueError(f'{name} must be finite')
        if name == 'speed_to_erpm_gain' and number == 0.0:
            raise ValueError('speed_to_erpm_gain must be nonzero')
        if name == 'wheelbase' and number <= 0.0:
            raise ValueError('wheelbase must be positive')
        result[name] = number
    return result


def movement_calibration(session: dict):
    params = session.get('current_parameters', {})
    current_gain = float(params.get('speed_to_erpm_gain', 4614.0))
    if not math.isfinite(current_gain) or current_gain == 0.0:
        raise ValueError('speed_to_erpm_gain must be finite and nonzero')
    gain_sign = math.copysign(1.0, current_gain)
    current_offset = float(params.get('speed_to_erpm_offset', 0.0))
    trials = [
        trial for trial in session.get('trials', [])
        if trial.get('accepted')
    ]
    stationary = [
        trial for trial in trials if trial.get('kind') == 'stationary'
    ]
    movement = [
        trial for trial in trials if trial.get('kind') == 'movement'
    ]
    warnings = []

    stationary_medians = [
        trial.get('summary', {}).get('raw_forward_erpm', {}).get('median')
        for trial in stationary
    ]
    stationary_medians = [
        value for value in stationary_medians if _finite(value)
    ]
    offset_source = 'configured'
    suggested_offset = current_offset
    if stationary_medians:
        suggested_offset = median(stationary_medians)
        offset_source = 'stationary capture'
        spans = []
        for trial in stationary:
            stats = trial.get('summary', {}).get('raw_forward_erpm', {})
            if _finite(stats.get('p05')) and _finite(stats.get('p95')):
                spans.append(float(stats['p95']) - float(stats['p05']))
        if spans and max(spans) > 100.0:
            warnings.append(
                'Stationary raw ERPM varied by more than 100 ERPM; '
                'check that the wheels were fully stopped.'
            )
    else:
        warnings.append(
            'No usable stationary raw-ERPM sample; retaining the configured offset.'
        )

    candidate_records = []
    for index, trial in enumerate(movement, start=1):
        summary = trial.get('summary', {})
        direction = trial.get('direction')
        distance = trial.get('measured_distance_m')
        record = {
            'trial_id': trial.get('id'),
            'trial_number': index,
            'direction': direction,
            'measured_distance_m': distance,
            'usable': False,
            'warnings': [],
        }
        if not _finite(distance) or float(distance) <= 0.0:
            record['warnings'].append('Measured distance is missing or not positive.')
            candidate_records.append(record)
            continue
        if direction not in ('forward', 'reverse'):
            record['warnings'].append('Direction must be forward or reverse.')
            candidate_records.append(record)
            continue

        truth = float(distance) if direction == 'forward' else -float(distance)
        raw_integral = summary.get('raw_erpm_integral')
        raw_duration = summary.get('raw_erpm_integration', {}).get('duration_sec')
        odom_distance = summary.get('odom_distance_m')
        candidate = None
        source = None
        if _finite(raw_integral) and _finite(raw_duration) and float(raw_duration) > 0.0:
            corrected = float(raw_integral) - suggested_offset * float(raw_duration)
            candidate = corrected / truth
            source = 'raw VESC integral'
            record['corrected_raw_erpm_integral'] = corrected
        elif _finite(odom_distance):
            # /odom was scaled by whatever gain was live when this run was
            # recorded; a gain applied since must not rescale old runs.
            recorded_gain = (trial.get('parameters_at_capture') or {}).get(
                'speed_to_erpm_gain')
            scale = float(recorded_gain) if _finite(recorded_gain) \
                and recorded_gain != 0.0 else current_gain
            candidate = scale * float(odom_distance) / truth
            source = 'integrated odometry fallback'
            record['warnings'].append(
                'Raw VESC integration unavailable; candidate uses current odometry scale.'
            )
        else:
            record['warnings'].append('No usable raw VESC or odometry distance.')

        record['signed_truth_distance_m'] = truth
        record['odom_distance_m'] = odom_distance
        record['candidate_gain'] = candidate
        record['candidate_source'] = source
        if _finite(odom_distance) and abs(float(odom_distance)) > 0.02:
            if math.copysign(1.0, float(odom_distance)) != math.copysign(1.0, truth):
                record['warnings'].append(
                    'Odometry direction disagrees with the confirmed direction.'
                )
        if not _finite(candidate):
            pass
        elif candidate * gain_sign <= 0.0:
            record['warnings'].append(
                'Candidate gain is zero or has the wrong sign for the configured '
                'gain (a negative candidate requires a negative baseline); '
                'direction/sign wiring must be checked.'
            )
        else:
            record['usable'] = True
        candidate_records.append(record)

    candidate_values = [
        record['candidate_gain'] * gain_sign
        for record in candidate_records
        if record['usable']
    ]
    inliers, rejected, suggested_gain, relative_spread = \
        _robust_positive_candidates(candidate_values)
    inliers = [value * gain_sign for value in inliers]
    rejected = [value * gain_sign for value in rejected]
    if suggested_gain is not None:
        suggested_gain *= gain_sign
    for record in candidate_records:
        if record.get('candidate_gain') in rejected:
            record['usable'] = False
            record['warnings'].append('Rejected as a robust statistical outlier.')

    if suggested_gain is None:
        confidence = 'insufficient'
        warnings.append('No gain-sign-consistent movement trial is usable.')
    elif len(inliers) >= 3 and relative_spread <= 0.03:
        confidence = 'high'
    elif len(inliers) >= 2 and relative_spread <= 0.07:
        confidence = 'good'
    else:
        confidence = 'low'
        warnings.append(
            'Repeat at least three consistent trials before applying this gain.'
        )
    if relative_spread is not None and relative_spread > 0.07:
        warnings.append(
            f"Movement candidates vary by about {relative_spread * 100.0:.1f}%."
        )
    directions = {
        record['direction'] for record in candidate_records if record['usable']
    }
    if suggested_gain is not None and directions != {'forward', 'reverse'}:
        warnings.append(
            'Only one direction is represented; add a reverse/forward check '
            'to expose sign or drivetrain asymmetry.'
        )

    return {
        'status': confidence,
        'usable_trial_count': len(inliers),
        'accepted_trial_count': len(movement),
        'suggested_speed_to_erpm_gain': suggested_gain,
        'current_speed_to_erpm_gain': current_gain,
        'relative_spread': relative_spread,
        'suggested_speed_to_erpm_offset': suggested_offset,
        'current_speed_to_erpm_offset': current_offset,
        'offset_source': offset_source,
        'trial_results': candidate_records,
        'rejected_candidate_gains': rejected,
        'warnings': warnings,
    }


def _linear_fit(points):
    """Least-squares y = slope*x + intercept."""
    if len(points) < 2:
        return None
    mean_x = statistics.fmean(point[0] for point in points)
    mean_y = statistics.fmean(point[1] for point in points)
    denominator = sum((point[0] - mean_x) ** 2 for point in points)
    if denominator <= EPSILON:
        return None
    slope = sum(
        (point[0] - mean_x) * (point[1] - mean_y)
        for point in points
    ) / denominator
    intercept = mean_y - slope * mean_x
    residuals = [
        point[1] - (slope * point[0] + intercept) for point in points
    ]
    rmse = math.sqrt(statistics.fmean(value * value for value in residuals))
    return slope, intercept, rmse


STEERING_KINDS = (
    'steering_center',
    'steering_drift',
    'steering_left',
    'steering_right',
)
CIRCLE_METHODS = ('inner_tire', 'rear_tires', 'axle_centre')
# Rear track (m, rear tire centre to centre) assumed by the inner-tire-only
# method when the session does not say. Car 2's is 0.27 m, from the inner and
# outer circles of 2026-10-10 (docs/hardware-reference.md).
DEFAULT_REAR_TRACK_M = 0.27
# A servo reading this close to servo_min/servo_max is treated as clamped.
SERVO_LIMIT_TOLERANCE = 0.002
# A "centred wheels" capture whose servo value is this close to the configured
# offset carries no information: with the stick neutral the servo command is
# computed *from* that offset, so the capture just reads it back.
CENTRE_ECHO_TOLERANCE = 0.002
# Left/right gains that differ by more than this fraction mean one straight
# line does not describe the servo linkage well.
ASYMMETRY_WARNING_FRACTION = 0.10
# Rear-tire track widths from different circles that disagree by more than
# this suggest a marking or measuring error.
TRACK_SPREAD_WARNING_M = 0.03
# A rear-axle radius outside this range is a typing or unit mistake: the
# wheelbase-0.36 m car cannot steer past ~40 degrees (0.43 m), and a "full
# lock" circle wider than 20 m is a straight line.
MIN_CIRCLE_RADIUS_M = 0.43
MAX_CIRCLE_RADIUS_M = 20.0
# A half circle is pi radians; odometry yaw well short of that means the
# operator measured across an arc, not a diameter. Odometry is an estimate,
# so this warns rather than rejects.
MIN_CIRCLE_YAW_RAD = 0.75 * math.pi
# Repeat full-lock circles on one side whose radii differ by more than this
# fraction disagree too much to average.
CIRCLE_REPEAT_SPREAD_FRACTION = 0.10


def drift_steering_angle(wheelbase, forward_m, lateral_m):
    """Steering angle from a straight-line drift test.

    The rear-axle centre starts on a lane line, heading along it. After the
    run it sits ``forward_m`` along the line and ``lateral_m`` off it (left
    positive). A constant-curvature path tangent to the line through both
    points has radius R = (s^2 + d^2) / (2 d); the kinematic bicycle model
    then gives steering = atan(wheelbase / R).
    """
    for name, value in (
            ('wheelbase', wheelbase),
            ('forward distance', forward_m),
            ('lateral offset', lateral_m)):
        if not _finite(value):
            raise ValueError(f'{name} must be a finite number')
    if wheelbase <= 0.0:
        raise ValueError('wheelbase must be positive')
    if forward_m <= 0.0:
        raise ValueError('forward distance must be positive')
    if abs(lateral_m) >= forward_m:
        raise ValueError(
            'lateral offset must be smaller than the forward distance; '
            'the drift test is for a nearly straight run')
    curvature = 2.0 * lateral_m / (forward_m ** 2 + lateral_m ** 2)
    return math.atan(wheelbase * curvature)


def circle_radius(trial: dict, rear_track_m=None):
    """Rear-axle-centre radius and (optionally) rear track for a circle trial.

    ``inner_tire`` measures only across the inner rear tire's marks; the axle
    centre is half the rear track further out, so its radius is
    (inner + track) / 2 with the track from the car's settings.
    ``rear_tires`` uses the circles traced by the inner and outer rear tires.
    The axle centre sits midway between them, so its radius is the mean of
    the two radii exactly, with no track-width constant needed. The
    difference is the rear track, which becomes a free consistency check.
    """
    method = trial.get('measurement_method', 'axle_centre')
    if method == 'inner_tire':
        inner = trial.get('measured_inner_diameter_m')
        if not _finite(inner) or float(inner) <= 0.0:
            raise ValueError('Inner tire diameter must be positive.')
        if not _finite(rear_track_m) or float(rear_track_m) <= 0.0:
            raise ValueError('The rear track is not set, so an inner-tire '
                             'circle cannot be converted; set rear_track_m.')
        return (float(inner) + float(rear_track_m)) / 2.0, None
    if method == 'rear_tires':
        inner = trial.get('measured_inner_diameter_m')
        outer = trial.get('measured_outer_diameter_m')
        if not _finite(inner) or not _finite(outer) or inner <= 0.0:
            raise ValueError('Inner and outer tire diameters must be positive.')
        if outer <= inner:
            raise ValueError(
                'The outer tire circle must be larger than the inner one.')
        return (float(inner) + float(outer)) / 4.0, \
            (float(outer) - float(inner)) / 2.0
    if method != 'axle_centre':
        raise ValueError(f'Unknown circle measurement method: {method!r}.')
    diameter = trial.get('measured_diameter_m')
    if not _finite(diameter) or float(diameter) <= 0.0:
        raise ValueError('Measured diameter must be positive.')
    return float(diameter) / 2.0, None


def _positive_number(payload, name, label):
    value = payload.get(name)
    if not _finite(value) or float(value) <= 0.0:
        raise ValueError(f'{label} must be a positive number.')
    return float(value)


def validate_steering_measurement(kind: str, payload: dict):
    """Validate operator-entered measurements; return the fields to store."""
    if kind == 'steering_center':
        return {}
    if kind == 'steering_drift':
        forward = _positive_number(
            payload, 'measured_forward_m', 'Forward distance along the line')
        lateral = payload.get('measured_lateral_m')
        if not _finite(lateral):
            raise ValueError('Sideways offset must be a number (0 if none).')
        lateral = float(lateral)
        if abs(lateral) >= forward:
            raise ValueError(
                'Sideways offset must be smaller than the forward distance.')
        return {'measured_forward_m': forward, 'measured_lateral_m': lateral}
    if kind not in ('steering_left', 'steering_right'):
        raise ValueError(f'Unknown steering capture kind: {kind!r}.')
    full_lock = payload.get('full_lock', False)
    if not isinstance(full_lock, bool):
        raise ValueError('full_lock must be true or false.')
    method = payload.get('measurement_method', 'axle_centre')
    fields = {'measurement_method': method, 'full_lock': full_lock}
    if method == 'inner_tire':
        fields['measured_inner_diameter_m'] = _positive_number(
            payload, 'measured_inner_diameter_m', 'Inner tire circle diameter')
        return fields
    if method == 'axle_centre':
        fields['measured_diameter_m'] = _positive_number(
            payload, 'measured_diameter_m', 'Rear-axle circle diameter')
    elif method == 'rear_tires':
        fields['measured_inner_diameter_m'] = _positive_number(
            payload, 'measured_inner_diameter_m', 'Inner tire circle diameter')
        fields['measured_outer_diameter_m'] = _positive_number(
            payload, 'measured_outer_diameter_m', 'Outer tire circle diameter')
    else:
        raise ValueError(f'Unknown circle measurement method: {method!r}.')
    circle_radius(fields)  # cross-field checks (outer > inner)
    return fields


def _servo_limits(session: dict):
    vehicle = session.get('vehicle', {}) or {}
    low, high = vehicle.get('servo_min'), vehicle.get('servo_max')
    if _finite(low) and _finite(high) and float(low) < float(high):
        return float(low), float(high)
    return None


def _steering_servo_stats(kind, summary):
    """Servo statistics a steering trial should be fitted with, and why.

    Returns (stats, source). ``active``: LB held and moving, the only samples
    that describe the measured path. ``whole_capture``: a session recorded
    before the filter existed. ``too_few_active``: the filter left too little
    to trust. A stationary centre capture keeps the whole capture by design.
    """
    if kind == 'steering_center':
        return summary.get('servo', {}) or {}, 'whole_capture'
    active = summary.get('servo_active')
    if active is None:
        return summary.get('servo', {}) or {}, 'whole_capture'
    if (active.get('count') or 0) < MIN_ACTIVE_SERVO_SAMPLES:
        return active, 'too_few_active'
    return active, 'active'


def _side_gain(points):
    fit = _linear_fit(points)
    return fit[0] if fit else None


def _constrained_fit(points, anchor):
    """Least-squares line through ``anchor`` exactly: (gain, offset, rmse).

    ``anchor`` is the drive-straight runs' mean (angle, servo). A straight
    run is the one direct measurement of the centre, so the line is pinned
    to it rather than letting a few full-lock circles drag the centre off;
    the gain is the slope that best fits the circles through that point.
    """
    if not points:
        return None
    anchor_angle, anchor_servo = anchor
    denominator = sum((angle - anchor_angle) ** 2 for angle, _ in points)
    if denominator <= EPSILON:
        return None
    gain = sum((angle - anchor_angle) * (servo - anchor_servo)
               for angle, servo in points) / denominator
    offset = anchor_servo - gain * anchor_angle
    residuals = [servo - (gain * angle + offset) for angle, servo in points]
    rmse = math.sqrt(statistics.fmean(value * value for value in residuals))
    return gain, offset, rmse


def _trial_parameters(trial, params):
    at_capture = trial.get('parameters_at_capture') or {}
    gain = at_capture.get('steering_angle_to_servo_gain',
                          params.get('steering_angle_to_servo_gain'))
    offset = at_capture.get('steering_angle_to_servo_offset',
                            params.get('steering_angle_to_servo_offset'))
    return gain, offset


def steering_calibration(session: dict):
    params = session.get('current_parameters', {})
    vehicle = session.get('vehicle', {}) or {}
    wheelbase = float(params.get('wheelbase', 0.36))
    current_gain = float(params.get('steering_angle_to_servo_gain', -0.78))
    current_offset = float(params.get('steering_angle_to_servo_offset', 0.5904))
    rear_track = vehicle.get('rear_track_m', DEFAULT_REAR_TRACK_M)
    limits = _servo_limits(session)
    trials = [
        trial for trial in session.get('trials', [])
        if trial.get('accepted') and trial.get('kind') in STEERING_KINDS
    ]
    warnings = []
    trial_results = []
    track_widths = []

    for index, trial in enumerate(trials, start=1):
        kind = trial.get('kind')
        summary = trial.get('summary', {})
        stats, source = _steering_servo_stats(kind, summary)
        servo = stats.get('median')
        item = {
            'trial_id': trial.get('id'),
            'trial_number': index,
            'kind': kind,
            'median_servo': servo,
            'servo_source': source,
            'usable': False,
            'warnings': [],
            'warnings_overridden': bool(trial.get('warnings_overridden')),
        }
        if source == 'too_few_active':
            if not (item['warnings_overridden'] and (stats.get('count') or 0) > 0):
                item['warnings'].append(
                    f"Only {stats.get('count', 0)} servo samples were taken while "
                    'LB was held and the car was moving; re-record this run.')
                trial_results.append(item)
                continue
            item['warnings'].append(
                f"Only {stats.get('count', 0)} servo samples were taken while "
                'LB was held and the car was moving; used anyway, as chosen.')
        if source == 'whole_capture':
            item['warnings'].append(
                'Recorded before idle time was filtered out; its servo value '
                'may include time with LB released. Re-record if it sits off '
                'the fitted line.')
        if not _finite(servo):
            item['warnings'].append('No finite servo command was captured.')
            trial_results.append(item)
            continue
        servo = float(servo)
        servo_p05 = stats.get('p05')
        servo_p95 = stats.get('p95')
        if _finite(servo_p05) and _finite(servo_p95):
            servo_span = float(servo_p95) - float(servo_p05)
            item['servo_span'] = servo_span
            if servo_span > 0.05:
                item['warnings'].append(
                    'Servo command varied substantially; hold steering steadier.'
                )
        capture_limits = trial.get('servo_limits_at_capture') or limits
        if capture_limits:
            item['at_servo_limit'] = (
                abs(servo - capture_limits[0]) <= SERVO_LIMIT_TOLERANCE
                or abs(servo - capture_limits[1]) <= SERVO_LIMIT_TOLERANCE
            )

        if kind == 'steering_center':
            actual_angle = 0.0
            if abs(servo - current_offset) <= CENTRE_ECHO_TOLERANCE:
                item['warnings'].append(
                    'Servo value matches the configured offset, so this capture '
                    'only reads the current setting back. Use a straight-line '
                    'drift test to measure the true centre.'
                )
        elif kind == 'steering_drift':
            forward = trial.get('measured_forward_m')
            lateral = trial.get('measured_lateral_m')
            item['measured_forward_m'] = forward
            item['measured_lateral_m'] = lateral
            try:
                actual_angle = drift_steering_angle(wheelbase, forward, lateral)
            except ValueError as exc:
                item['warnings'].append(f'Drift measurement unusable: {exc}.')
                trial_results.append(item)
                continue
        else:
            try:
                radius, track = circle_radius(trial, rear_track)
            except ValueError as exc:
                item['warnings'].append(str(exc))
                trial_results.append(item)
                continue
            item['measurement_method'] = trial.get(
                'measurement_method', 'axle_centre')
            item['full_lock'] = bool(trial.get('full_lock', False))
            item['rear_axle_radius_m'] = radius
            if not MIN_CIRCLE_RADIUS_M <= radius <= MAX_CIRCLE_RADIUS_M:
                item['warnings'].append(
                    f'Rear-axle radius {radius:.2f} m is outside '
                    f'{MIN_CIRCLE_RADIUS_M:.1f}–{MAX_CIRCLE_RADIUS_M:.0f} m, '
                    'which no steered circle on this car can produce; check '
                    'that the numbers are diameters in metres.')
                trial_results.append(item)
                continue
            if track is not None:
                item['rear_track_m'] = track
                track_widths.append(track)
            magnitude = math.atan(wheelbase / radius)
            actual_angle = magnitude if kind == 'steering_left' else -magnitude
            yaw = summary.get('odom_yaw_rad')
            item['odom_yaw_rad'] = yaw
            if _finite(yaw) and abs(float(yaw)) > 0.2:
                expected_sign = 1.0 if kind == 'steering_left' else -1.0
                if math.copysign(1.0, float(yaw)) != expected_sign:
                    item['warnings'].append(
                        'Odometry yaw sign disagrees with the confirmed turn direction.'
                    )
                item['odom_turns'] = float(yaw) / (2.0 * math.pi)
            yaw_samples = (summary.get('odom_yaw_integration') or {}).get(
                'sample_count', 0)
            if (_finite(yaw) and yaw_samples >= 5
                    and abs(float(yaw)) < MIN_CIRCLE_YAW_RAD):
                item['warnings'].append(
                    f'Odometry says the car turned only '
                    f'{abs(float(yaw)) / (2.0 * math.pi):.2f} turns. Measure '
                    'across the circle only after a full half circle (0.50 '
                    'turns); a shorter arc is not a diameter.')
            gain_then, offset_then = _trial_parameters(trial, params)
            item['check'] = steering_curve.check_circle(
                actual_angle, servo, gain_then, offset_then)

        item['actual_steering_angle_rad'] = actual_angle
        item['usable'] = True
        trial_results.append(item)

    usable = [item for item in trial_results if item['usable']]
    overridden = {item['trial_id']: item['warnings_overridden'] for item in trial_results}
    drifts = [item for item in usable if item['kind'] == 'steering_drift']
    centres = [item for item in usable if item['kind'] == 'steering_center']
    circles = [item for item in usable
               if item['kind'] in ('steering_left', 'steering_right')]

    # The straight-ahead servo, used to order each side's circles by travel.
    # Old sessions carry a stationary centre capture instead of a drift run.
    centre_items = drifts or centres
    if centre_items:
        anchor = (statistics.fmean(i['actual_steering_angle_rad'] for i in centre_items),
                  statistics.fmean(i['median_servo'] for i in centre_items))
        centre_source = 'drive_straight' if drifts else 'centre_capture'
    else:
        anchor = None
        centre_source = 'circles_only'
    centre_servo = anchor[1] if anchor else current_offset

    curves = {}
    for side, kind in (('left', 'steering_left'), ('right', 'steering_right')):
        side_items = [{'servo': i['median_servo'],
                       'angle_rad': i['actual_steering_angle_rad'],
                       'radius_m': i['rear_axle_radius_m'],
                       'trial_id': i['trial_id']}
                      for i in circles if i['kind'] == kind]
        # Which servo limit this side runs towards: left is positive angle,
        # and servo = gain * angle + offset, so the sign of the gain decides.
        towards_max = (current_gain > 0.0) == (side == 'left')
        limit = None
        if limits:
            limit = limits[1] if towards_max else limits[0]
        curves[side] = steering_curve.side_curve(
            side_items, centre_servo, limit, limits)
        curves[side]['limit_name'] = 'servo_max' if towards_max else 'servo_min'
        curves[side]['limit'] = limit

    # Circles past a side's stop describe the linkage pushing against
    # something, not its working range; they are shown but not fitted.
    for item in circles:
        side = 'left' if item['kind'] == 'steering_left' else 'right'
        stop = curves[side]['stop']
        if stop and abs(item['median_servo'] - centre_servo) > \
                abs(stop['servo'] - centre_servo) + steering_curve.LIMIT_TOLERANCE:
            item['beyond_stop'] = True
    fit_circles = [i for i in circles if not i.get('beyond_stop')]
    points = [(i['actual_steering_angle_rad'], i['median_servo'])
              for i in fit_circles]

    directions = {i['kind'] for i in fit_circles}
    if anchor:
        fit = _constrained_fit(points, anchor)
    else:
        fit = _linear_fit(points)
    max_commandable = None
    if fit is None:
        suggested_gain = None
        suggested_offset = None
        rmse = None
        status = 'insufficient'
        warnings.append(
            'Record a drive-straight run and at least one circle (or two '
            'distinct circles) before the steering can be fitted.'
        )
    else:
        suggested_gain, suggested_offset, rmse = fit
        for item in usable:
            item['fit_residual_servo'] = item['median_servo'] - (
                suggested_gain * item['actual_steering_angle_rad']
                + suggested_offset)
        if abs(suggested_gain) < 0.1:
            status = 'invalid'
            warnings.append(
                'Fitted steering gain is implausibly close to zero.'
            )
        elif directions == {'steering_left', 'steering_right'} and anchor:
            status = 'good' if rmse <= 0.02 else 'low'
        else:
            status = 'low'
        if directions != {'steering_left', 'steering_right'}:
            warnings.append('Record both a left and a right circle.')
        if not anchor:
            warnings.append(
                'No straight-line drift test was recorded; the steering '
                'offset rests on the circles alone and is less certain.'
            )
        if suggested_gain * current_gain < 0.0:
            warnings.append(
                'Suggested steering gain reverses the configured sign; '
                'verify left/right labels before applying it.'
            )
        if rmse > 0.02:
            warnings.append(
                f"Steering fit residual is {rmse:.3f} servo units. One straight "
                'line does not describe this linkage well (it is usually not '
                'linear near full lock); the gain is a compromise across the '
                'circles you drove.')
        if limits is not None and status != 'invalid':
            ends = [(limit - suggested_offset) / suggested_gain for limit in limits]
            left, right = max(ends), min(ends)
            if left > 0.0 > right:
                max_commandable = {
                    'left_rad': left,
                    'right_rad': right,
                    'left_min_radius_m': wheelbase / math.tan(left),
                    'right_min_radius_m': wheelbase / math.tan(-right),
                }
        smallest = min((abs(i['actual_steering_angle_rad']) for i in fit_circles),
                       default=None)
        if smallest is not None and math.degrees(smallest) > 15.0:
            warnings.append(
                f'The tightest-steering circle is the only kind recorded: the '
                f'smallest angle measured is {math.degrees(smallest):.1f}°. '
                'Driving nodes mostly steer below that, where this gain is '
                'an extrapolation.')

    for side, curve in curves.items():
        if curve['stop']:
            stop = curve['stop']
            warnings.append(
                f"The {side} steering levels off: going from servo "
                f"{stop['servo']:.3f} to {stop['beyond_servo']:.3f} added only "
                f"{stop['added_deg']:.1f}°, so the linkage is at or near its "
                f"stop past {stop['servo']:.3f}. Circles past it are not "
                'used in the fit.')
        elif curve['still_gaining_at_limit']:
            warnings.append(
                f"The {side} steering still gains at {curve['limit_name']} "
                f"{curve['limit']:.3f}. If you want more {side} lock, check "
                f"wheels-up that nothing binds, raise {curve['limit_name']} "
                'by about 0.05, and drive another full-lock circle.')
        for point in curve['points']:
            if point['radius_spread'] > steering_curve.REPEAT_SPREAD_FRACTION:
                kept = all(overridden.get(trial_id) for trial_id in point['trial_ids'])
                warnings.append(
                    f"The {side} full-lock circles disagree by "
                    f"{point['radius_spread'] * 100.0:.0f}% in radius (servo "
                    f"{point['servo']:.3f}); "
                    + ('kept anyway, as chosen for every one of them.' if kept else
                       're-measure them or remove the odd one before trusting '
                       'the fit.'))
                if status == 'good' and not kept:
                    status = 'low'

    suggestions = {}
    statuses = {}
    new_limits = list(limits) if limits else None
    for side, curve in curves.items():
        value = steering_curve.servo_limit_suggestion(
            curve, centre_servo, curve['limit'])
        if value is not None:
            suggestions[curve['limit_name']] = value
            statuses[curve['limit_name']] = 'good'
            if new_limits:
                new_limits[0 if curve['limit_name'] == 'servo_min' else 1] = value
    scale = None
    if suggested_gain is not None and status != 'invalid':
        scale = steering_curve.stick_scale_suggestion(
            curves['left']['reach_deg'], curves['right']['reach_deg'],
            suggested_gain, suggested_offset, new_limits)
    if scale is not None:
        suggestions['steering_stick_scale'] = scale
        statuses['steering_stick_scale'] = status

    sides = {}
    for side, kind in (('left', 'steering_left'), ('right', 'steering_right')):
        curve = curves[side]
        side_circles = [i for i in circles if i['kind'] == kind]
        reach = curve['reach_deg']
        reach_items = [i for i in side_circles
                       if curve['reach_servo'] is not None
                       and abs(i['median_servo'] - curve['reach_servo'])
                       <= steering_curve.SERVO_GROUP_TOLERANCE]
        radius = (median(i['rear_axle_radius_m'] for i in reach_items)
                  if reach_items else None)
        side_points = [(i['actual_steering_angle_rad'], i['median_servo'])
                       for i in fit_circles if i['kind'] == kind]
        anchor_points = [anchor] if anchor else []
        latest_full = [i for i in side_circles if i.get('full_lock')]
        sides[side] = {
            'circle_count': len(side_circles),
            'full_lock_count': len(latest_full),
            'full_lock_radius_m': radius,
            'full_lock_angle_rad': (math.radians(reach) * (1.0 if side == 'left' else -1.0)
                                    if reach is not None else None),
            'full_lock_at_servo_limit': (
                all(i.get('at_servo_limit') for i in reach_items)
                if reach_items and limits is not None else None),
            'gain': _side_gain(anchor_points + side_points)
            if anchor_points and side_points else None,
            'check': latest_full[-1].get('check') if latest_full else None,
        }
    left_gain, right_gain = sides['left']['gain'], sides['right']['gain']
    asymmetry = None
    if _finite(left_gain) and _finite(right_gain):
        mean_magnitude = (abs(left_gain) + abs(right_gain)) / 2.0
        if mean_magnitude > EPSILON:
            asymmetry = abs(left_gain - right_gain) / mean_magnitude
            if asymmetry > ASYMMETRY_WARNING_FRACTION:
                warnings.append(
                    f'Left and right steering gains differ by '
                    f'{asymmetry * 100.0:.0f}%; one straight line does not '
                    'describe this linkage well. The suggestion is a compromise.'
                )

    track = None
    if track_widths:
        track = median(track_widths)
        if max(track_widths) - min(track_widths) > TRACK_SPREAD_WARNING_M:
            warnings.append(
                'Rear track widths from different circles disagree by more '
                f'than {TRACK_SPREAD_WARNING_M * 100:.0f} cm; re-measure the '
                'tire circles.'
            )

    for item in trial_results:
        if item.get('kind') == 'steering_center' and item['warnings']:
            warnings.extend(
                w for w in item['warnings'] if 'configured offset' in w)
            break

    checks = [sides[side]['check'] for side in ('left', 'right')]
    confirmed = (all(check and check['passed'] for check in checks)
                 if all(checks) else None)

    return {
        'status': status,
        'wheelbase_m': wheelbase,
        'usable_point_count': len(points) + len(centre_items),
        'centre_source': centre_source,
        'suggested_steering_angle_to_servo_gain': suggested_gain,
        'current_steering_angle_to_servo_gain': current_gain,
        'suggested_steering_angle_to_servo_offset': suggested_offset,
        'current_steering_angle_to_servo_offset': current_offset,
        'fit_rmse_servo': rmse,
        'servo_limits': list(limits) if limits else None,
        'max_commandable_steering': max_commandable,
        'curves': curves,
        'limit_suggestions': suggestions,
        'limit_statuses': statuses,
        'calibration_confirmed': confirmed,
        'sides': sides,
        'side_gain_asymmetry': asymmetry,
        'rear_track_m': track,
        'assumed_rear_track_m': rear_track,
        'trial_results': trial_results,
        'warnings': warnings,
    }


MODES_WITH_MOVEMENT = ('movement', 'movement_steering')
MODES_WITH_STEERING = ('movement_steering', 'steering')

# Bounds a suggestion must sit inside before it is shown as a value, exported
# or applied. They catch broken fits, not fine calibration: a real
# recalibration of this car moved the odometry gain by 3.6% (2026-09-23).
SPEED_GAIN_RATIO_RANGE = (0.5, 2.0)
MAX_SPEED_OFFSET_ERPM = 1000.0
STEERING_GAIN_MAGNITUDE_RANGE = (0.1, 5.0)
# Full stick past this (rad, ~40 deg) is past anything a car this size steers.
MAX_STICK_SCALE_RAD = 0.70
CONFIDENT_STATUSES = ('good', 'high')


def _same_sign(a, b):
    return math.copysign(1.0, a) == math.copysign(1.0, b)


def vet_suggestion(name, value, current, status, limits=None):
    """Return (blocked_reason, apply_reason) for one suggested value.

    ``blocked_reason`` is set when the value is not a usable number for this
    car at all; such a value is never displayed as a suggestion, exported or
    applied. ``apply_reason`` is set when the value is plausible but the
    evidence behind it is too weak to write into vesc.yaml.
    """
    if status in ('invalid', 'insufficient'):
        return f'the {name} fit is {status}', None
    if not _finite(value):
        return 'no usable value was computed', None
    value = float(value)
    if name == 'speed_to_erpm_gain':
        if value == 0.0:
            return 'a zero gain would stop odometry counting distance', None
        if _finite(current) and current != 0.0:
            if not _same_sign(value, current):
                return 'its sign is the reverse of the configured gain', None
            ratio = abs(value / current)
            low, high = SPEED_GAIN_RATIO_RANGE
            if not low <= ratio <= high:
                return (f'it is {ratio:.2f}x the configured gain; outside '
                        f'{low}-{high}x is a broken run, not a calibration'), None
    elif name == 'speed_to_erpm_offset':
        if abs(value) > MAX_SPEED_OFFSET_ERPM:
            return (f'{abs(value):.0f} ERPM at standstill means the wheels '
                    'were moving during the baseline'), None
    elif name == 'steering_angle_to_servo_gain':
        low, high = STEERING_GAIN_MAGNITUDE_RANGE
        if not low <= abs(value) <= high:
            return (f'|gain| {abs(value):.3f} is outside {low}-{high}; the '
                    'servo would barely move or would slam to its limits'), None
        if _finite(current) and current != 0.0 and not _same_sign(value, current):
            return ('its sign is the reverse of the configured gain; left '
                    'and right would swap'), None
    elif name == 'steering_angle_to_servo_offset':
        low, high = limits if limits else (0.0, 1.0)
        if not low < value < high:
            return (f'{value:.4f} is outside the servo range '
                    f'{low:.2f}-{high:.2f}'), None
    elif name in ('servo_min', 'servo_max'):
        if not 0.0 <= value <= 1.0:
            return f'{value:.4f} is outside the 0-1 the servo signal has', None
        other = None if not limits else (limits[1] if name == 'servo_min' else limits[0])
        if limits and ((name == 'servo_min' and value >= other)
                       or (name == 'servo_max' and value <= other)):
            return (f'{value:.4f} would leave no servo range against the '
                    f'other limit {other:.4f}'), None
    elif name == 'steering_stick_scale':
        if not 0.0 < value <= MAX_STICK_SCALE_RAD:
            return (f'{value:.3f} rad is outside 0-{MAX_STICK_SCALE_RAD:.2f} '
                    'rad'), None
    if status not in CONFIDENT_STATUSES:
        return None, f'confidence is {status}; add consistent runs first'
    return None, None


def _vet_all(raw, current, statuses, limits):
    suggestions, blocked, not_ready = {}, {}, {}
    for name, value in raw.items():
        blocked_reason, apply_reason = vet_suggestion(
            name, value, current.get(name), statuses[name], limits)
        if blocked_reason:
            blocked[name] = blocked_reason
            continue
        suggestions[name] = float(value)
        if apply_reason:
            not_ready[name] = apply_reason
    return suggestions, blocked, not_ready


def current_values(session: dict):
    """Every value a report may suggest, as the car runs it now.

    The vesc.yaml calibration values, plus the servo limits (vesc_driver) and
    the full-stick steering angle (joy_teleop), which the session keeps in
    its vehicle settings.
    """
    current = dict(session.get('current_parameters', {}))
    vehicle = session.get('vehicle', {}) or {}
    for name in ('servo_min', 'servo_max', 'steering_stick_scale'):
        if _finite(vehicle.get(name)):
            current[name] = float(vehicle[name])
    return current


def build_report(session: dict):
    mode = session.get('mode')
    movement = (
        movement_calibration(session)
        if mode in MODES_WITH_MOVEMENT or mode is None
        else None
    )
    steering = (
        steering_calibration(session)
        if mode in MODES_WITH_STEERING
        else None
    )
    raw, part_status = {}, {}
    if movement:
        raw.update({
            'speed_to_erpm_gain': movement['suggested_speed_to_erpm_gain'],
            'speed_to_erpm_offset': movement['suggested_speed_to_erpm_offset'],
        })
        part_status['speed_to_erpm_gain'] = movement['status']
        # The offset comes from the stationary baseline, not the gain fit.
        part_status['speed_to_erpm_offset'] = (
            'good' if movement['offset_source'] == 'stationary capture'
            else 'insufficient')
    if steering:
        raw.update({
            'steering_angle_to_servo_gain':
                steering['suggested_steering_angle_to_servo_gain'],
            'steering_angle_to_servo_offset':
                steering['suggested_steering_angle_to_servo_offset'],
        })
        part_status['steering_angle_to_servo_gain'] = steering['status']
        part_status['steering_angle_to_servo_offset'] = steering['status']
        raw.update(steering['limit_suggestions'])
        part_status.update(steering['limit_statuses'])
    current = current_values(session)
    suggestions, blocked, not_ready = _vet_all(
        raw, current, part_status, _servo_limits(session))
    clamp = None
    if mode in MODES_WITH_STEERING:
        # Imported here: steering_chain imports this module.
        from odom_calibration import steering_chain
        vehicle = session.get('vehicle', {}) or {}
        clamp = steering_chain.clamp_check(
            session.get('current_parameters', {}), _servo_limits(session),
            vehicle.get('steering_stick_scale'),
            vehicle.get('planner_max_steering_rad'))
    statuses = [part['status'] for part in (movement, steering) if part]
    if blocked:
        statuses.append('invalid')
    overall = (
        'ready'
        if statuses and all(status in ('good', 'high') for status in statuses)
        else 'review'
    )
    notes = []
    if movement:
        notes.append(
            'Speed gain and offset suggestions apply to vesc_to_odom_node only; '
            'do not copy a negative odometry gain into the shared motor-command '
            'configuration.')
    if steering:
        notes.append(
            'Steering gain and offset apply to the shared steering conversion '
            'in vesc.yaml.')
    notes.append(
        'Review suggestions before editing YAML. Re-test wheels off the '
        'ground, then at low speed while holding LB.')
    return {
        'schema_version': 1,
        'session_id': session.get('session_id'),
        'created_at': session.get('created_at'),
        'generated_at': session.get('updated_at'),
        'mode': mode,
        'overall_status': overall,
        'vehicle': session.get('vehicle', {}),
        'current_parameters': current,
        'accepted_trials': [
            trial for trial in session.get('trials', [])
            if trial.get('accepted')
        ],
        'movement': movement,
        'steering': steering,
        'clamp_check': clamp,
        'parameter_suggestions': suggestions,
        'blocked_suggestions': blocked,
        'not_ready_to_apply': not_ready,
        'safety_note': ' '.join(notes),
    }


def _format_number(value, digits=4):
    if not _finite(value):
        return 'not available'
    return f"{float(value):.{digits}f}"


def report_markdown(report: dict):
    """Human-readable report suitable for download or review in Git."""
    lines = [
        '# RacerBot odometry calibration report',
        '',
        f"- Session: `{report.get('session_id')}`",
        f"- Mode: `{report.get('mode')}`",
        f"- Overall status: **{report.get('overall_status')}**",
        '',
        '## Suggested parameters',
        '',
        '```yaml',
    ]
    not_ready = report.get('not_ready_to_apply', {})
    for name, value in report.get('parameter_suggestions', {}).items():
        if _finite(value):
            note = f'  # not ready: {not_ready[name]}' if name in not_ready else ''
            lines.append(f'{name}: {_format_number(value, 6)}{note}')
        else:
            lines.append(f'# {name}: insufficient data')
    for name, reason in report.get('blocked_suggestions', {}).items():
        lines.append(f'# {name}: blocked, {reason}')
    lines.append('```')
    clamp = report.get('clamp_check')
    if clamp:
        lines.extend(['', '## Servo clamp, from the current settings', ''])
        if _finite(clamp.get('left_reach_rad')):
            lines.append(
                f"- servo_min/servo_max let the software command up to "
                f"{math.degrees(clamp['left_reach_rad']):.1f} deg left and "
                f"{math.degrees(clamp['right_reach_rad']):.1f} deg right.")
        for finding in clamp.get('findings', []):
            lines.append(f'- Warning: {finding}')
        if not clamp.get('findings'):
            lines.append('- Nothing asks for more steering than the clamp allows.')
    movement = report.get('movement')
    if movement:
        lines.extend(['', '## Movement calibration', ''])
        lines.extend([
            f"- Confidence: **{movement.get('status')}**",
            f"- Usable trials: {movement.get('usable_trial_count', 0)}",
            f"- Gain: {_format_number(movement.get('current_speed_to_erpm_gain'))}"
            f" → {_format_number(movement.get('suggested_speed_to_erpm_gain'))}",
            f"- Offset: {_format_number(movement.get('current_speed_to_erpm_offset'))}"
            f" → {_format_number(movement.get('suggested_speed_to_erpm_offset'))}",
        ])
        for warning in movement.get('warnings', []):
            lines.append(f"- Warning: {warning}")
    steering = report.get('steering')
    if steering:
        lines.extend(['', '## Steering calibration', ''])
        lines.extend([
            f"- Confidence: **{steering.get('status')}**",
            f"- Usable points: {steering.get('usable_point_count', 0)}",
            f"- Gain: "
            f"{_format_number(steering.get('current_steering_angle_to_servo_gain'))}"
            f" → "
            f"{_format_number(steering.get('suggested_steering_angle_to_servo_gain'))}",
            f"- Offset: "
            f"{_format_number(steering.get('current_steering_angle_to_servo_offset'))}"
            f" → "
            f"{_format_number(steering.get('suggested_steering_angle_to_servo_offset'))}",
        ])
        for side, data in (steering.get('sides') or {}).items():
            if data.get('full_lock_radius_m'):
                lines.append(
                    f"- Full lock {side}: rear-axle radius "
                    f"{_format_number(data['full_lock_radius_m'], 3)} m, "
                    f"{_format_number(math.degrees(abs(data['full_lock_angle_rad'])), 1)} degrees"
                )
        reach = steering.get('max_commandable_steering')
        if reach:
            lines.append(
                f"- Servo limits allow at most "
                f"{_format_number(reach['left_rad'], 3)} rad left and "
                f"{_format_number(-reach['right_rad'], 3)} rad right "
                "with the suggested values."
            )
        for warning in steering.get('warnings', []):
            lines.append(f"- Warning: {warning}")
    lines.extend([
        '',
        '## Safety',
        '',
        report.get('safety_note', ''),
        '',
    ])
    return '\n'.join(lines)
