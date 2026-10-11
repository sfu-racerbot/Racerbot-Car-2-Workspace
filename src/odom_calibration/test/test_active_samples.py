"""The 2026-10-06 "gain 0.0" bug and the filter that fixes it.

That day every circle recording spent most of its time with LB released and
the stick centred, so the median servo of each circle was the configured
offset and the fitted steering gain came out as zero. Steering trials must be
fitted only from samples taken while LB was held and the car was moving, and
a fit that still collapses must never be offered as a value.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import calibration_math  # noqa: E402

WHEELBASE = 0.36
OFFSET = 0.5904          # vesc.yaml on 2026-10-10
TRUE_GAIN = -1.15        # synthetic linkage, not the configured -1.2135
TRUE_OFFSET = 0.55
RATE_HZ = 20.0


def _capture(kind, idle_sec, drive_sec, drive_servo, idle_servo=OFFSET,
             drive_speed=0.5, yaw_rate=0.0):
    """A recording: idle with LB released, then driving with LB held.

    The idle part is longer than the drive, as on 2026-10-06, so a median
    over the whole capture lands on the idle (offset) value.
    """
    capture = {'kind': kind, 'started_monotonic': 0.0,
               'ended_monotonic': idle_sec + drive_sec,
               'odom': [], 'vesc': [], 'servo': [], 'servo_cmd': [],
               'drive': [], 'joy': []}
    steps = int((idle_sec + drive_sec) * RATE_HZ)
    for step in range(steps):
        t = step / RATE_HZ
        driving = t >= idle_sec
        capture['joy'].append({'t': t, 'lb_held': driving, 'steer_axis': 0.0})
        capture['servo'].append(
            {'t': t + 0.01, 'value': drive_servo if driving else idle_servo})
        capture['odom'].append({
            't': t + 0.02, 'speed': drive_speed if driving else 0.0,
            'angular_z': yaw_rate if driving else 0.0,
            'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    return capture


def _servo(angle):
    return TRUE_GAIN * angle + TRUE_OFFSET


def _arc_endpoint(steering_angle, arc_length):
    """Oracle, as in test_steering_math: forward kinematic-bicycle arc."""
    radius = WHEELBASE / math.tan(steering_angle)
    theta = arc_length / radius
    return radius * math.sin(theta), radius * (1.0 - math.cos(theta))


def _session(trials):
    return {
        'session_id': 'filter', 'mode': 'steering',
        'current_parameters': {
            'speed_to_erpm_gain': -4447.983786, 'speed_to_erpm_offset': 0.0,
            'steering_angle_to_servo_gain': -1.2135,
            'steering_angle_to_servo_offset': OFFSET, 'wheelbase': WHEELBASE},
        'vehicle': {'servo_min': 0.15, 'servo_max': 0.85},
        'trials': trials,
    }


def _trials_from_captures(drift_angle=0.01, left=0.30, right=-0.28):
    forward, lateral = _arc_endpoint(drift_angle, 5.0)
    trials = []
    for kind, angle, fields in (
            ('steering_drift', drift_angle,
             {'measured_forward_m': forward, 'measured_lateral_m': lateral}),
            ('steering_left', left,
             {'measurement_method': 'axle_centre', 'full_lock': True,
              'measured_diameter_m': 2.0 * WHEELBASE / math.tan(left)}),
            ('steering_right', right,
             {'measurement_method': 'axle_centre', 'full_lock': True,
              'measured_diameter_m': 2.0 * WHEELBASE / math.tan(-right)})):
        summary = calibration_math.summarize_capture(
            _capture(kind, idle_sec=12.0, drive_sec=8.0, drive_servo=_servo(angle)))
        trials.append({'id': kind, 'kind': kind, 'accepted': True,
                       'summary': summary, **fields})
    return trials


def test_filtered_fit_recovers_the_linkage_despite_long_idle_time():
    report = calibration_math.steering_calibration(_session(_trials_from_captures()))
    # Oracle: the synthetic linkage the captures were generated from.
    assert report['suggested_steering_angle_to_servo_gain'] == pytest.approx(
        TRUE_GAIN, abs=1e-9)  # servo units per rad
    assert report['suggested_steering_angle_to_servo_offset'] == pytest.approx(
        TRUE_OFFSET, abs=1e-9)  # servo units
    assert all(item['servo_source'] == 'active' for item in report['trial_results'])


def test_whole_capture_median_is_the_idle_offset_which_caused_gain_zero():
    # Documents the failure mode the filter exists for: the unfiltered
    # median of every trial is the idle offset, not the turn.
    for trial in _trials_from_captures():
        assert trial['summary']['servo']['median'] == pytest.approx(OFFSET, abs=1e-12)


def test_a_collapsed_fit_is_blocked_not_suggested_or_exported():
    # Old-format trials (no filtered stats) whose servo values all sit on
    # the offset: the 2026-10-06 data. The fit is flat.
    trials = _trials_from_captures()
    for trial in trials:
        trial['summary'].pop('servo_active')
    report = calibration_math.build_report(_session(trials))
    assert report['steering']['status'] == 'invalid'
    assert 'steering_angle_to_servo_gain' not in report['parameter_suggestions']
    assert 'invalid' in report['blocked_suggestions']['steering_angle_to_servo_gain']
    markdown = calibration_math.report_markdown(report)
    assert '# steering_angle_to_servo_gain: blocked' in markdown
    for line in markdown.splitlines():
        assert not line.startswith('steering_angle_to_servo_gain:')
    assert report['overall_status'] == 'review'


def test_run_with_too_few_moving_samples_is_unusable():
    trials = _trials_from_captures()
    # 0.4 s of driving = 8 samples < MIN_ACTIVE_SERVO_SAMPLES (10).
    summary = calibration_math.summarize_capture(
        _capture('steering_left', 12.0, 0.4, _servo(0.30)))
    trials[1]['summary'] = summary
    assert any('LB was held and the car was moving' in w for w in summary['warnings'])
    report = calibration_math.steering_calibration(_session(trials))
    left = next(item for item in report['trial_results'] if item['kind'] == 'steering_left')
    assert left['usable'] is False
    assert left['servo_source'] == 'too_few_active'


@pytest.mark.parametrize('count,source', [
    (9, 'too_few_active'),    # one under MIN_ACTIVE_SERVO_SAMPLES
    (10, 'active'),           # exactly at it
    (0, 'too_few_active'),
])
def test_active_sample_count_boundary(count, source):
    summary = {'servo': {'median': 0.5}, 'servo_active': {'median': 0.3, 'count': count}}
    assert calibration_math._steering_servo_stats('steering_left', summary)[1] == source


def _lb_capture(joy, odom, servo):
    return {'joy': joy, 'odom': odom, 'servo': servo}


def test_sample_without_recent_joy_is_not_counted_as_held():
    capture = _lb_capture(
        joy=[{'t': 0.0, 'lb_held': True}],
        odom=[{'t': 0.9, 'speed': 1.0}],
        servo=[{'t': 0.4, 'value': 0.3}, {'t': 1.0, 'value': 0.9}])
    kept = calibration_math.active_samples(capture, 'servo', require_moving=False)
    # 0.4 s after the last /joy is within SAMPLE_MATCH_AGE_SEC; 1.0 s is not.
    assert [s['value'] for s in kept] == [0.3]


def test_released_lb_excludes_samples_even_when_moving():
    capture = _lb_capture(
        joy=[{'t': 0.0, 'lb_held': False}],
        odom=[{'t': 0.0, 'speed': 2.0}],
        servo=[{'t': 0.1, 'value': 0.3}])
    assert calibration_math.active_samples(capture, 'servo', require_moving=True) == []


@pytest.mark.parametrize('speed,kept', [
    (0.15, True),      # exactly MIN_MOVING_SPEED_MPS
    (0.1499, False),   # one step under
    (-0.15, True),     # reverse counts as moving
    (float('nan'), False),
    (0.0, False),
])
def test_moving_threshold_boundary(speed, kept):
    capture = _lb_capture(
        joy=[{'t': 0.0, 'lb_held': True}],
        odom=[{'t': 0.0, 'speed': speed}],
        servo=[{'t': 0.1, 'value': 0.3}])
    result = calibration_math.active_samples(capture, 'servo', require_moving=True)
    assert (len(result) == 1) is kept


def test_no_odom_means_not_moving():
    capture = _lb_capture(joy=[{'t': 0.0, 'lb_held': True}], odom=[],
                          servo=[{'t': 0.1, 'value': 0.3}])
    assert calibration_math.active_samples(capture, 'servo', require_moving=True) == []
    assert len(calibration_math.active_samples(
        capture, 'servo', require_moving=False)) == 1


# --- vet_suggestion -----------------------------------------------------------

@pytest.mark.parametrize('name,value,current,match', [
    ('steering_angle_to_servo_gain', 0.0, -1.2135, 'outside 0.1-5.0'),
    ('steering_angle_to_servo_gain', -0.0999, -1.2135, 'outside 0.1-5.0'),
    ('steering_angle_to_servo_gain', 1.2, -1.2135, 'left and right would swap'),
    ('steering_angle_to_servo_gain', float('nan'), -1.2135, 'no usable value'),
    ('steering_angle_to_servo_gain', None, -1.2135, 'no usable value'),
    ('steering_angle_to_servo_offset', 0.15, 0.59, 'outside the servo range'),
    ('steering_angle_to_servo_offset', 0.85, 0.59, 'outside the servo range'),
    ('steering_angle_to_servo_offset', float('inf'), 0.59, 'no usable value'),
    ('speed_to_erpm_gain', 0.0, -4447.98, 'zero gain'),
    ('speed_to_erpm_gain', 4447.98, -4447.98, 'sign is the reverse'),
    ('speed_to_erpm_gain', -2223.0, -4447.98, '0.50x'),
    ('speed_to_erpm_gain', -8896.5, -4447.98, '2.00x'),
    ('speed_to_erpm_offset', 1000.1, 0.0, 'wheels were moving'),
])
def test_unusable_suggestions_are_blocked(name, value, current, match):
    blocked, _ = calibration_math.vet_suggestion(
        name, value, current, 'good', (0.15, 0.85))
    assert blocked is not None and match in blocked


@pytest.mark.parametrize('name,value,current', [
    ('steering_angle_to_servo_gain', -0.1, -1.2135),   # boundary, allowed
    ('steering_angle_to_servo_gain', -1.15, -1.2135),
    ('steering_angle_to_servo_offset', 0.1501, 0.59),
    ('speed_to_erpm_gain', -4614.0, -4447.98),
    ('speed_to_erpm_gain', -2224.0, -4447.98),         # just over 0.5x
    ('speed_to_erpm_offset', -1000.0, 0.0),            # boundary, allowed
])
def test_plausible_suggestions_pass(name, value, current):
    assert calibration_math.vet_suggestion(
        name, value, current, 'good', (0.15, 0.85)) == (None, None)


@pytest.mark.parametrize('status', ['low'])
def test_low_confidence_is_shown_but_not_ready_to_apply(status):
    blocked, not_ready = calibration_math.vet_suggestion(
        'steering_angle_to_servo_gain', -1.15, -1.2135, status, (0.15, 0.85))
    assert blocked is None
    assert 'confidence is low' in not_ready


@pytest.mark.parametrize('status', ['invalid', 'insufficient'])
def test_failed_fit_blocks_even_a_plausible_number(status):
    blocked, _ = calibration_math.vet_suggestion(
        'steering_angle_to_servo_gain', -1.15, -1.2135, status, (0.15, 0.85))
    assert blocked == f'the steering_angle_to_servo_gain fit is {status}'
