"""Pure calibration math for the odometry wizard.

This module deliberately has no ROS, Tornado, filesystem, or browser imports.
Every calculation used in the final report can therefore be tested with small
synthetic data sets before it is trusted with measurements from the car.
"""

from __future__ import annotations

import math
import statistics
from typing import Iterable


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

    return {
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


def movement_calibration(session: dict):
    params = session.get('current_parameters', {})
    current_gain = float(params.get('speed_to_erpm_gain', 4614.0))
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
            candidate = current_gain * float(odom_distance) / truth
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
        elif candidate <= 0.0:
            record['warnings'].append(
                'Candidate gain is negative; direction/sign wiring must be checked.'
            )
        else:
            record['usable'] = True
        candidate_records.append(record)

    candidate_values = [
        record['candidate_gain']
        for record in candidate_records
        if record['usable']
    ]
    inliers, rejected, suggested_gain, relative_spread = \
        _robust_positive_candidates(candidate_values)
    for record in candidate_records:
        if record.get('candidate_gain') in rejected:
            record['usable'] = False
            record['warnings'].append('Rejected as a robust statistical outlier.')

    if suggested_gain is None:
        confidence = 'insufficient'
        warnings.append('No positive, direction-consistent movement trial is usable.')
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


def steering_calibration(session: dict):
    params = session.get('current_parameters', {})
    wheelbase = float(params.get('wheelbase', 0.324))
    current_gain = float(params.get('steering_angle_to_servo_gain', -1.2135))
    current_offset = float(params.get('steering_angle_to_servo_offset', 0.5304))
    trials = [
        trial for trial in session.get('trials', [])
        if trial.get('accepted')
        and trial.get('kind') in (
            'steering_center', 'steering_left', 'steering_right')
    ]
    warnings = []
    points = []
    trial_results = []
    directions = set()

    for index, trial in enumerate(trials, start=1):
        kind = trial.get('kind')
        summary = trial.get('summary', {})
        servo = summary.get('servo', {}).get('median')
        item = {
            'trial_id': trial.get('id'),
            'trial_number': index,
            'kind': kind,
            'median_servo': servo,
            'usable': False,
            'warnings': [],
        }
        if not _finite(servo):
            item['warnings'].append('No finite servo command was captured.')
            trial_results.append(item)
            continue
        servo_p05 = summary.get('servo', {}).get('p05')
        servo_p95 = summary.get('servo', {}).get('p95')
        if _finite(servo_p05) and _finite(servo_p95):
            servo_span = float(servo_p95) - float(servo_p05)
            item['servo_span'] = servo_span
            if servo_span > 0.05:
                item['warnings'].append(
                    'Servo command varied substantially; hold steering steadier.'
                )

        if kind == 'steering_center':
            actual_angle = 0.0
        else:
            diameter = trial.get('measured_diameter_m')
            item['measured_diameter_m'] = diameter
            if not _finite(diameter) or float(diameter) <= 0.0:
                item['warnings'].append('Measured diameter must be positive.')
                trial_results.append(item)
                continue
            radius = float(diameter) / 2.0
            magnitude = math.atan(wheelbase / radius)
            actual_angle = magnitude if kind == 'steering_left' else -magnitude
            directions.add(kind)
            yaw = summary.get('odom_yaw_rad')
            item['odom_yaw_rad'] = yaw
            if _finite(yaw) and abs(float(yaw)) > 0.2:
                expected_sign = 1.0 if kind == 'steering_left' else -1.0
                if math.copysign(1.0, float(yaw)) != expected_sign:
                    item['warnings'].append(
                        'Odometry yaw sign disagrees with the confirmed turn direction.'
                    )
                item['odom_turns'] = float(yaw) / (2.0 * math.pi)

        item['actual_steering_angle_rad'] = actual_angle
        item['usable'] = True
        points.append((actual_angle, float(servo)))
        trial_results.append(item)

    fit = _linear_fit(points)
    if fit is None:
        suggested_gain = None
        suggested_offset = None
        rmse = None
        status = 'insufficient'
        warnings.append(
            'At least two distinct usable steering measurements are required.'
        )
    else:
        suggested_gain, suggested_offset, rmse = fit
        if abs(suggested_gain) < 0.1:
            status = 'invalid'
            warnings.append(
                'Fitted steering gain is implausibly close to zero.'
            )
        elif directions == {'steering_left', 'steering_right'} and len(points) >= 3:
            status = 'good' if rmse <= 0.02 else 'low'
        else:
            status = 'low'
        if directions != {'steering_left', 'steering_right'}:
            warnings.append('Record both a left and a right circle.')
        if not any(trial.get('kind') == 'steering_center' for trial in trials):
            warnings.append(
                'No visually centred sample was recorded; steering offset is less certain.'
            )
        if suggested_gain * current_gain < 0.0:
            warnings.append(
                'Suggested steering gain reverses the configured sign; '
                'verify left/right labels before applying it.'
            )
        if rmse > 0.02:
            warnings.append(
                f"Steering fit residual is {rmse:.3f} servo units; repeat unstable trials."
            )

    return {
        'status': status,
        'wheelbase_m': wheelbase,
        'usable_point_count': len(points),
        'suggested_steering_angle_to_servo_gain': suggested_gain,
        'current_steering_angle_to_servo_gain': current_gain,
        'suggested_steering_angle_to_servo_offset': suggested_offset,
        'current_steering_angle_to_servo_offset': current_offset,
        'fit_rmse_servo': rmse,
        'trial_results': trial_results,
        'warnings': warnings,
    }


def build_report(session: dict):
    movement = movement_calibration(session)
    steering = (
        steering_calibration(session)
        if session.get('mode') == 'movement_steering'
        else None
    )
    suggestions = {
        'speed_to_erpm_gain': movement['suggested_speed_to_erpm_gain'],
        'speed_to_erpm_offset': movement['suggested_speed_to_erpm_offset'],
    }
    if steering:
        suggestions.update({
            'steering_angle_to_servo_gain':
                steering['suggested_steering_angle_to_servo_gain'],
            'steering_angle_to_servo_offset':
                steering['suggested_steering_angle_to_servo_offset'],
        })
    statuses = [movement['status']]
    if steering:
        statuses.append(steering['status'])
    overall = (
        'ready'
        if statuses and all(status in ('good', 'high') for status in statuses)
        else 'review'
    )
    return {
        'schema_version': 1,
        'session_id': session.get('session_id'),
        'created_at': session.get('created_at'),
        'generated_at': session.get('updated_at'),
        'mode': session.get('mode'),
        'overall_status': overall,
        'vehicle': session.get('vehicle', {}),
        'current_parameters': session.get('current_parameters', {}),
        'accepted_trials': [
            trial for trial in session.get('trials', [])
            if trial.get('accepted')
        ],
        'movement': movement,
        'steering': steering,
        'parameter_suggestions': suggestions,
        'safety_note': (
            'Review suggestions before editing YAML. Re-test wheels off the '
            'ground, then at low speed while holding LB.'
        ),
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
    for name, value in report.get('parameter_suggestions', {}).items():
        if _finite(value):
            lines.append(f'{name}: {_format_number(value, 6)}')
        else:
            lines.append(f'# {name}: insufficient data')
    lines.extend(['```', '', '## Movement calibration', ''])
    movement = report.get('movement', {})
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
