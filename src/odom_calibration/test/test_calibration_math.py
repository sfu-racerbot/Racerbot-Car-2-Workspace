import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import calibration_math  # noqa: E402


def _parameters():
    return {
        'speed_to_erpm_gain': 4614.0,
        'speed_to_erpm_offset': 0.0,
        'steering_angle_to_servo_gain': -1.2135,
        'steering_angle_to_servo_offset': 0.5304,
        'wheelbase': 0.324,
    }


def _stationary_trial(offset=20.0):
    return {
        'id': 'stationary',
        'kind': 'stationary',
        'accepted': True,
        'summary': {
            'raw_forward_erpm': {
                'median': offset,
                'p05': offset - 1.0,
                'p95': offset + 1.0,
            },
        },
    }


def _movement_trial(identifier, direction, distance, raw_integral,
                    raw_duration=5.0, odom_distance=None):
    return {
        'id': identifier,
        'kind': 'movement',
        'accepted': True,
        'direction': direction,
        'measured_distance_m': distance,
        'summary': {
            'raw_erpm_integral': raw_integral,
            'raw_erpm_integration': {'duration_sec': raw_duration},
            'odom_distance_m': odom_distance,
        },
    }


def test_integrator_skips_large_telemetry_gap():
    samples = [
        {'t': 0.0, 'value': 1.0},
        {'t': 0.1, 'value': 1.0},
        {'t': 1.0, 'value': 1.0},
        {'t': 1.1, 'value': 1.0},
    ]
    result = calibration_math.integrate_samples(
        samples, 'value', max_gap_sec=0.2)
    assert result['integral'] == pytest.approx(0.2)
    assert result['duration_sec'] == pytest.approx(0.2)
    assert result['skipped_duration_sec'] == pytest.approx(0.9)


def test_capture_summary_preserves_reverse_signs():
    times = [index * 0.1 for index in range(11)]
    capture = {
        'kind': 'movement',
        'started_monotonic': 0.0,
        'ended_monotonic': 1.0,
        'odom': [
            {'t': t, 'speed': -1.0, 'angular_z': 0.0,
             'x': -t, 'y': 0.0, 'yaw': 0.0}
            for t in times
        ],
        'vesc': [
            {'t': t, 'raw_forward_erpm': -4614.0} for t in times
        ],
        'servo': [{'t': t, 'value': 0.53} for t in times],
        'drive': [
            {'t': t, 'speed': -1.0, 'steering': 0.0} for t in times
        ],
        'joy': [{'t': t, 'lb_held': True} for t in times],
    }
    summary = calibration_math.summarize_capture(capture)
    assert summary['odom_distance_m'] == pytest.approx(-1.0)
    assert summary['raw_erpm_integral'] == pytest.approx(-4614.0)
    assert summary['lb_held_fraction'] == pytest.approx(1.0)


def test_forward_and_reverse_trials_produce_positive_gain():
    offset = 20.0
    gain = 5000.0
    session = {
        'current_parameters': _parameters(),
        'trials': [
            _stationary_trial(offset),
            _movement_trial(
                'forward', 'forward', 5.0,
                raw_integral=gain * 5.0 + offset * 5.0,
                odom_distance=4.6,
            ),
            _movement_trial(
                'reverse', 'reverse', 4.0,
                raw_integral=gain * -4.0 + offset * 5.0,
                odom_distance=-3.7,
            ),
        ],
    }
    report = calibration_math.movement_calibration(session)
    assert report['suggested_speed_to_erpm_gain'] == pytest.approx(gain)
    assert report['suggested_speed_to_erpm_offset'] == pytest.approx(offset)
    assert report['usable_trial_count'] == 2
    assert report['status'] == 'good'


def test_negative_gain_is_reported_not_absolute_valued():
    session = {
        'current_parameters': _parameters(),
        'trials': [
            _stationary_trial(0.0),
            _movement_trial(
                'wrong-sign', 'forward', 5.0,
                raw_integral=-23070.0,
                odom_distance=-5.0,
            ),
        ],
    }
    report = calibration_math.movement_calibration(session)
    assert report['suggested_speed_to_erpm_gain'] is None
    trial = report['trial_results'][0]
    assert trial['candidate_gain'] < 0.0
    assert any('negative' in warning for warning in trial['warnings'])


def test_movement_falls_back_to_current_odom_scale_without_raw_erpm():
    session = {
        'current_parameters': _parameters(),
        'trials': [
            _movement_trial(
                'odom-only', 'forward', 5.0,
                raw_integral=None,
                raw_duration=0.0,
                odom_distance=5.5,
            ),
        ],
    }
    report = calibration_math.movement_calibration(session)
    assert report['suggested_speed_to_erpm_gain'] == pytest.approx(
        4614.0 * 5.5 / 5.0)
    assert report['trial_results'][0]['candidate_source'] == \
        'integrated odometry fallback'


def test_robust_gain_rejects_outlier_even_when_good_values_match_exactly():
    gain = 5000.0
    session = {
        'current_parameters': _parameters(),
        'trials': [
            _stationary_trial(0.0),
            _movement_trial('a', 'forward', 5.0, gain * 5.0),
            _movement_trial('b', 'reverse', 5.0, gain * -5.0),
            _movement_trial('outlier', 'forward', 5.0, 9000.0 * 5.0),
        ],
    }
    report = calibration_math.movement_calibration(session)
    assert report['suggested_speed_to_erpm_gain'] == pytest.approx(gain)
    assert report['rejected_candidate_gains'] == [pytest.approx(9000.0)]


def _steering_trial(identifier, kind, servo, diameter=None, yaw=0.0):
    trial = {
        'id': identifier,
        'kind': kind,
        'accepted': True,
        'summary': {
            'servo': {'median': servo, 'p05': servo, 'p95': servo},
            'odom_yaw_rad': yaw,
        },
    }
    if diameter is not None:
        trial['measured_diameter_m'] = diameter
    return trial


def test_left_right_and_center_fit_steering_parameters():
    wheelbase = 0.324
    gain = -1.2
    offset = 0.53
    angle = 0.25
    diameter = 2.0 * wheelbase / math.tan(angle)
    params = _parameters()
    params['wheelbase'] = wheelbase
    session = {
        'current_parameters': params,
        'trials': [
            _steering_trial('center', 'steering_center', offset),
            _steering_trial(
                'left', 'steering_left',
                gain * angle + offset, diameter, yaw=2.0 * math.pi),
            _steering_trial(
                'right', 'steering_right',
                gain * -angle + offset, diameter, yaw=-2.0 * math.pi),
        ],
    }
    report = calibration_math.steering_calibration(session)
    assert report['suggested_steering_angle_to_servo_gain'] == \
        pytest.approx(gain)
    assert report['suggested_steering_angle_to_servo_offset'] == \
        pytest.approx(offset)
    assert report['fit_rmse_servo'] == pytest.approx(0.0, abs=1e-12)
    assert report['status'] == 'good'


def test_report_omits_steering_for_movement_only_mode():
    session = {
        'session_id': 'test',
        'mode': 'movement',
        'current_parameters': _parameters(),
        'trials': [],
    }
    report = calibration_math.build_report(session)
    assert report['steering'] is None
    assert 'steering_angle_to_servo_gain' not in \
        report['parameter_suggestions']
    assert report['overall_status'] == 'review'


def test_markdown_never_writes_none_as_parameter_value():
    session = {
        'session_id': 'test',
        'mode': 'movement',
        'current_parameters': _parameters(),
        'trials': [],
    }
    markdown = calibration_math.report_markdown(
        calibration_math.build_report(session))
    assert '# speed_to_erpm_gain: insufficient data' in markdown
    assert 'speed_to_erpm_gain: None' not in markdown
