"""Hardware or software: the steering range diagnosis.

Expected values come from the conversion the car itself uses,
servo = gain * angle + offset, clamped to [servo_min, servo_max]
(vesc_ackermann/src/ackermann_to_vesc.cpp and vesc_driver.cpp), recomputed
here in closed form.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import calibration_math  # noqa: E402
from odom_calibration import steering_chain  # noqa: E402

GAIN = -1.2135
LIMITS = (0.15, 0.85)
TELEOP_SCALE = 0.34          # joy_teleop.yaml human_control steering scale
CAR_OFFSET = 0.5904          # vesc.yaml after the 2026-10-10 drift fix
RATE_HZ = 20.0


def _clamp(value, limits=LIMITS):
    return min(max(value, limits[0]), limits[1])


def _sweep_capture(offset, gain=GAIN, limits=LIMITS, scale=TELEOP_SCALE,
                   axis_left=1.0, axis_right=-1.0, servo_cmd=True,
                   hold_sec=3.0, holds=('left', 'right')):
    """LB held: centre, full left, centre, full right, centre."""
    capture = {'kind': 'steering_sweep', 'joy': [], 'drive': [], 'servo': [],
               'servo_cmd': [], 'odom': [], 'vesc': []}
    phases = [('centre', 0.0)]
    for side in holds:
        phases += [(side, axis_left if side == 'left' else axis_right), ('centre', 0.0)]
    t = 0.0
    for _, axis in phases:
        for _ in range(int(hold_sec * RATE_HZ)):
            angle = scale * axis
            raw = gain * angle + offset
            capture['joy'].append({'t': t, 'lb_held': True, 'steer_axis': axis})
            capture['drive'].append({'t': t + 0.005, 'speed': 0.0, 'steering': angle})
            if servo_cmd:
                capture['servo_cmd'].append({'t': t + 0.01, 'value': raw})
            capture['servo'].append({'t': t + 0.012, 'value': _clamp(raw, limits)})
            t += 1.0 / RATE_HZ
    return capture


def _params(offset):
    return {'steering_angle_to_servo_gain': GAIN,
            'steering_angle_to_servo_offset': offset, 'wheelbase': 0.36}


def _lock(degrees_left_wheel, degrees_right_wheel=None):
    lock = {'left_wheel_deg': degrees_left_wheel}
    if degrees_right_wheel is not None:
        lock['right_wheel_deg'] = degrees_right_wheel
    return lock


def _diagnose(offset, measurements=None, planner=None, **capture_args):
    sweep = steering_chain.sweep_summary(_sweep_capture(offset, **capture_args))
    return steering_chain.diagnose(sweep, measurements, _params(offset), LIMITS, planner)


def _layer(result, layer_id):
    return next(layer for layer in result['layers'] if layer['id'] == layer_id)


# Closed forms for this car on 2026-10-10.
LEFT_REACH = (CAR_OFFSET - LIMITS[0]) / -GAIN       # servo_min side
RIGHT_REACH = (LIMITS[1] - CAR_OFFSET) / -GAIN      # servo_max side


def test_the_cars_19_5_and_12_3_degrees_are_a_software_clamp():
    # Discord, 2026-10-10: left ~19.5 deg, right ~12.3 deg. Full stick asks
    # for 0.34 rad both ways; right needs servo 0.5904 + 1.2135*0.34 = 1.003,
    # which servo_max 0.85 clips. Wheels measured as following the command.
    left_deg = math.degrees(TELEOP_SCALE)
    right_deg = math.degrees(RIGHT_REACH)
    result = _diagnose(CAR_OFFSET, {'left_lock': _lock(left_deg),
                                    'right_lock': _lock(right_deg)}, planner=0.26)
    assert left_deg == pytest.approx(19.48, abs=0.01)    # deg, the Discord reading
    assert right_deg == pytest.approx(12.26, abs=0.01)   # deg, the Discord reading
    assert result['verdict'] == 'software'
    assert result['commanded_rad']['left'] == pytest.approx(TELEOP_SCALE, abs=1e-9)
    assert result['commanded_rad']['right'] == pytest.approx(RIGHT_REACH, abs=1e-9)
    clamp = _layer(result, 'clamp')
    assert clamp['status'] == 'limited'
    assert 'right is cut from 1.003 to 0.850 by servo_max' in clamp['finding']
    assert 'left is cut' not in clamp['finding']
    assert _layer(result, 'command')['status'] == 'ok'
    assert _layer(result, 'stick')['status'] == 'ok'
    assert result['software_causes'] == ['clamp']
    assert result['symmetric_reach_rad'] == pytest.approx(
        min(LEFT_REACH, RIGHT_REACH), abs=1e-9)  # rad
    joined = ' '.join(result['recommendations'])
    assert 're-centre the servo horn' in joined
    assert 'configured for up to 14.9 deg' in joined   # 0.26 rad planners


def test_software_asymmetry_without_wheel_measurements_is_still_software():
    result = _diagnose(CAR_OFFSET)
    assert result['verdict'] == 'software'
    assert _layer(result, 'wheels')['status'] == 'missing'
    assert 'Measure the wheels' in result['summary']


def test_symmetric_software_and_unequal_wheels_is_hardware():
    # Offset at the servo midpoint and a 0.25 rad scale: no clamp anywhere.
    result = _diagnose(0.5, {'left_lock': _lock(15.0, 13.0),
                             'right_lock': _lock(10.0, 11.0)}, scale=0.25)
    assert _layer(result, 'clamp')['status'] == 'ok'
    assert _layer(result, 'commanded')['status'] == 'ok'
    assert result['verdict'] == 'hardware'
    # Mean of the two wheels per lock, 14 and 10.5 deg.
    assert math.degrees(result['measured_rad']['left']) == pytest.approx(14.0, abs=1e-9)
    assert math.degrees(result['measured_rad']['right']) == pytest.approx(10.5, abs=1e-9)
    assert any('Mechanical checklist' in r for r in result['recommendations'])


def test_symmetric_everything_is_symmetric():
    angle = math.degrees(0.25)
    result = _diagnose(0.5, {'left_lock': _lock(angle), 'right_lock': _lock(angle)},
                       scale=0.25)
    assert result['verdict'] == 'symmetric'


def test_symmetric_software_without_measurements_is_unknown_not_symmetric():
    result = _diagnose(0.5, scale=0.25)
    assert result['verdict'] == 'unknown'


@pytest.mark.parametrize('left,right,verdict', [
    (14.3, 13.0, 'symmetric'),   # 1.3 deg apart: within MEASURED_SYMMETRY_DEG
    (14.3, 12.7, 'hardware'),    # 1.6 deg apart: past it
])
def test_measured_asymmetry_threshold(left, right, verdict):
    result = _diagnose(0.5, {'left_lock': _lock(left), 'right_lock': _lock(right)},
                       scale=0.25)
    assert result['verdict'] == verdict


def test_short_stick_travel_is_found_before_the_servo():
    result = _diagnose(0.5, scale=0.25, axis_right=-0.8)
    stick = _layer(result, 'stick')
    assert stick['status'] == 'short'
    assert 'right' in stick['finding']
    assert result['software_causes'][0] == 'stick'
    assert result['verdict'] == 'software'


def test_stick_exactly_at_full_threshold_is_ok():
    result = _diagnose(0.5, scale=0.25, axis_left=0.95, axis_right=-0.95)
    assert _layer(result, 'stick')['status'] == 'ok'


def test_wheels_equal_while_software_unequal_is_mixed():
    equal = math.degrees(RIGHT_REACH)
    result = _diagnose(CAR_OFFSET, {'left_lock': _lock(equal), 'right_lock': _lock(equal)})
    assert result['verdict'] == 'mixed'


def test_wheels_off_by_a_different_ratio_than_software_is_both():
    # Software: 19.5 vs 12.3. Wheels: 19.5 vs 8.0, far below the command.
    result = _diagnose(CAR_OFFSET, {'left_lock': _lock(math.degrees(TELEOP_SCALE)),
                                    'right_lock': _lock(8.0)})
    assert result['verdict'] == 'both'
    assert 'right side' in _layer(result, 'wheels')['finding']


def test_servo_strain_is_reported_as_a_mechanical_stop():
    result = _diagnose(0.5, {'left_lock': _lock(14.3), 'right_lock': _lock(14.3),
                             'buzz_left': True}, scale=0.25)
    assert any('strained at full left' in r for r in result['recommendations'])


def test_pre_clamp_value_is_computed_when_the_topic_is_missing():
    sweep = steering_chain.sweep_summary(_sweep_capture(CAR_OFFSET, servo_cmd=False))
    assert any('commands/servo/position' in w for w in sweep['warnings'])
    result = steering_chain.diagnose(sweep, None, _params(CAR_OFFSET), LIMITS)
    assert _layer(result, 'clamp')['status'] == 'limited'
    assert _layer(result, 'servo_command')['right_text'] == f'{GAIN * -TELEOP_SCALE + CAR_OFFSET:.4f}'


def test_servo_command_from_different_parameters_is_a_mismatch():
    # The car runs offset 0.5904 but the wizard believes 0.5304: the
    # restart-after-editing case.
    sweep = steering_chain.sweep_summary(_sweep_capture(CAR_OFFSET))
    result = steering_chain.diagnose(sweep, None, _params(0.5304), LIMITS)
    assert _layer(result, 'servo_command')['status'] == 'mismatch'
    assert 'servo_model' in result['software_causes']


@pytest.mark.parametrize('holds', [('left',), ('right',), ()])
def test_missing_hold_is_insufficient(holds):
    sweep = steering_chain.sweep_summary(_sweep_capture(0.5, holds=holds))
    result = steering_chain.diagnose(sweep, None, _params(0.5), LIMITS)
    assert result['verdict'] == 'insufficient'
    assert result['layers'] == []


def test_released_lb_is_not_a_hold():
    capture = _sweep_capture(CAR_OFFSET)
    for sample in capture['joy']:
        sample['lb_held'] = False
    sweep = steering_chain.sweep_summary(capture)
    assert sweep['sides']['left']['samples'] == 0
    assert len(sweep['warnings']) >= 2


def test_plateau_ignores_the_stick_on_its_way_over():
    capture = _sweep_capture(0.5, scale=0.25)
    # Add a slow ramp to half lock first, longer than the hold itself; it
    # must not pull the median in.
    for step in range(100):
        t = -10.0 + step / RATE_HZ
        capture['joy'].append({'t': t, 'lb_held': True, 'steer_axis': 0.5})
        capture['drive'].append({'t': t + 0.005, 'steering': 0.125})
        capture['servo'].append({'t': t + 0.012, 'value': GAIN * 0.125 + 0.5})
    sweep = steering_chain.sweep_summary(capture)
    assert sweep['sides']['left']['command_rad'] == pytest.approx(0.25, abs=1e-12)


def test_no_gain_is_insufficient():
    sweep = steering_chain.sweep_summary(_sweep_capture(0.5))
    result = steering_chain.diagnose(sweep, None, {'steering_angle_to_servo_gain': 0.0,
                                                   'steering_angle_to_servo_offset': 0.5},
                                     LIMITS)
    assert result['verdict'] == 'insufficient'


# --- symmetric_reach ------------------------------------------------------------

def test_symmetric_reach_closed_form():
    assert steering_chain.symmetric_reach(GAIN, CAR_OFFSET, LIMITS) == pytest.approx(
        min(LEFT_REACH, RIGHT_REACH), abs=1e-12)  # rad
    assert steering_chain.symmetric_reach(GAIN, 0.5, LIMITS) == pytest.approx(
        0.35 / 1.2135, abs=1e-12)  # rad, centred offset


@pytest.mark.parametrize('offset', [0.15, 0.85, 0.9])
def test_offset_on_or_past_a_limit_reaches_nothing(offset):
    assert steering_chain.symmetric_reach(GAIN, offset, LIMITS) == 0.0


@pytest.mark.parametrize('gain', [0.0, float('nan'), None])
def test_symmetric_reach_needs_a_gain(gain):
    assert steering_chain.symmetric_reach(gain, 0.5, LIMITS) is None


# --- validate_sweep_measurement ----------------------------------------------

def test_sweep_measurements_are_optional_and_unsigned():
    fields = steering_chain.validate_sweep_measurement({
        'left_lock': {'left_wheel_deg': -18.0, 'right_wheel_deg': ''},
        'right_lock': {}, 'buzz_left': True})
    assert fields == {'left_lock': {'left_wheel_deg': 18.0}, 'right_lock': {},
                      'buzz_left': True, 'buzz_right': False}


@pytest.mark.parametrize('payload,match', [
    ({'left_lock': {'left_wheel_deg': 60.1}}, 'between 0 and 60'),
    ({'left_lock': {'left_wheel_deg': float('nan')}}, 'numbers in degrees'),
    ({'left_lock': {'left_wheel_deg': 'twelve'}}, 'numbers in degrees'),
    ({'left_lock': {'left_wheel_deg': True}}, 'numbers in degrees'),
    ({'left_lock': [12.0]}, 'left_lock must be an object'),
    ({'buzz_left': 'yes'}, 'buzz_left must be true or false'),
])
def test_sweep_measurement_rejects_operator_mistakes(payload, match):
    with pytest.raises(ValueError, match=match):
        steering_chain.validate_sweep_measurement(payload)


def test_sixty_degrees_exactly_is_accepted():
    fields = steering_chain.validate_sweep_measurement(
        {'right_lock': {'right_wheel_deg': 60.0}})
    assert fields['right_lock'] == {'right_wheel_deg': 60.0}


# --- centre_offset_suggestion --------------------------------------------------

WHEELBASE = 0.36


def _arc_endpoint(steering_angle, arc_length):
    """Oracle: forward kinematic-bicycle arc (independent of the inversion)."""
    if steering_angle == 0.0:
        return arc_length, 0.0
    radius = WHEELBASE / math.tan(steering_angle)
    theta = arc_length / radius
    return radius * math.sin(theta), radius * (1.0 - math.cos(theta))


def _drift_session(true_offset, configured_offset=CAR_OFFSET, gain=GAIN, count=20):
    # Stick centred: the servo sits on the configured offset; the linkage's
    # real straight-ahead servo is true_offset, so the car turns by
    # (configured - true) / gain.
    angle = (configured_offset - true_offset) / gain
    forward, lateral = _arc_endpoint(angle, 5.0)
    params = {**_params(configured_offset), 'steering_angle_to_servo_gain': gain}
    return {
        'current_parameters': dict(params),
        'trials': [{
            'id': 'd1', 'kind': 'steering_drift', 'accepted': True,
            'parameters_at_capture': params,
            'measured_forward_m': forward, 'measured_lateral_m': lateral,
            'summary': {'servo_active': {'median': configured_offset, 'count': count}},
        }],
    }, angle


@pytest.mark.parametrize('true_offset', [0.5704, 0.5904, 0.6104, 0.6304])
def test_centre_offset_recovers_the_true_straight_ahead_servo(true_offset):
    session, angle = _drift_session(true_offset)
    result = steering_chain.centre_offset_suggestion(
        session, calibration_math.drift_steering_angle)
    # Oracle: the offset the synthetic run was generated from.
    assert result['suggested_offset'] == pytest.approx(true_offset, abs=1e-9)
    assert result['drift_angle_rad'] == pytest.approx(angle, abs=1e-9)
    assert result['blocked'] is None


def test_centre_offset_uses_the_gain_that_was_live_for_the_run():
    session, _ = _drift_session(0.6104, gain=-1.0)
    session['current_parameters']['steering_angle_to_servo_gain'] = GAIN
    result = steering_chain.centre_offset_suggestion(
        session, calibration_math.drift_steering_angle)
    assert result['suggested_offset'] == pytest.approx(0.6104, abs=1e-9)


@pytest.mark.parametrize('true_offset,straight', [
    (CAR_OFFSET, True),
    (CAR_OFFSET - 0.0025 * GAIN * 0.99, True),    # just inside the tolerance
    (CAR_OFFSET - 0.0025 * GAIN * 1.01, False),   # just past it
])
def test_straight_flag_follows_the_tolerance(true_offset, straight):
    session, _ = _drift_session(true_offset)
    result = steering_chain.centre_offset_suggestion(
        session, calibration_math.drift_steering_angle)
    assert result['straight'] is straight


def test_a_huge_offset_jump_is_blocked():
    session, _ = _drift_session(CAR_OFFSET - 0.11)
    result = steering_chain.centre_offset_suggestion(
        session, calibration_math.drift_steering_angle)
    assert 'more than 0.10 in one step' in result['blocked']


def test_centre_offset_with_too_few_driving_samples_is_blocked():
    session, _ = _drift_session(0.6, count=9)
    result = steering_chain.centre_offset_suggestion(
        session, calibration_math.drift_steering_angle)
    assert result['suggested_offset'] is None
    assert 'too few servo samples' in result['blocked']


def test_no_drift_run_means_no_suggestion():
    assert steering_chain.centre_offset_suggestion(
        {'trials': []}, calibration_math.drift_steering_angle) is None


def test_bad_drift_measurement_is_blocked_not_raised():
    session, _ = _drift_session(0.6)
    session['trials'][0]['measured_lateral_m'] = 6.0   # larger than forward
    result = steering_chain.centre_offset_suggestion(
        session, calibration_math.drift_steering_angle)
    assert 'smaller than the forward distance' in result['blocked']


# --- report integration --------------------------------------------------------

def test_report_carries_the_diagnosis_and_markdown_prints_it():
    capture = _sweep_capture(CAR_OFFSET)
    summary = calibration_math.summarize_capture(capture)
    session = {
        'session_id': 'chain', 'mode': 'steering',
        'current_parameters': _params(CAR_OFFSET),
        'vehicle': {'servo_min': LIMITS[0], 'servo_max': LIMITS[1],
                    'planner_max_steering_rad': 0.26},
        'trials': [{'id': 's', 'kind': 'steering_sweep', 'accepted': True,
                    'summary': summary, 'parameters_at_capture': _params(CAR_OFFSET),
                    'measurements': {}}],
    }
    report = calibration_math.build_report(session)
    assert report['steering_chain']['verdict'] == 'software'
    markdown = calibration_math.report_markdown(report)
    assert 'Verdict: **software**' in markdown
    assert 'Servo clamp (servo_min / servo_max)' in markdown
    # The sweep never feeds the servo fit: the wheels were off the ground.
    assert report['steering']['usable_point_count'] == 0


def test_a_range_check_from_before_an_offset_change_says_so():
    summary = calibration_math.summarize_capture(_sweep_capture(CAR_OFFSET))
    session = {
        'mode': 'steering',
        'current_parameters': _params(0.5712),     # offset changed since
        'vehicle': {'servo_min': LIMITS[0], 'servo_max': LIMITS[1]},
        'trials': [{'id': 's', 'kind': 'steering_sweep', 'accepted': True,
                    'summary': summary, 'parameters_at_capture': _params(CAR_OFFSET),
                    'servo_limits_at_capture': list(LIMITS), 'measurements': {}}],
    }
    result = steering_chain.diagnose_session(session)
    assert result['stale'] is True
    assert result['summary'].startswith(
        'Recorded before steering_angle_to_servo_offset changed')
    # Diagnosed with the values it was recorded under, not today's.
    assert result['commanded_rad']['right'] == pytest.approx(RIGHT_REACH, abs=1e-9)
    session['current_parameters'] = _params(CAR_OFFSET)
    assert steering_chain.diagnose_session(session)['stale'] is False
    session['vehicle']['servo_max'] = 0.9
    stale = steering_chain.diagnose_session(session)
    assert stale['stale'] is True and 'servo limits' in stale['summary']


def test_a_drift_run_from_before_an_offset_change_suggests_nothing():
    session, _ = _drift_session(0.5734)
    session['current_parameters']['steering_angle_to_servo_offset'] = 0.5734
    result = steering_chain.centre_offset_suggestion(
        session, calibration_math.drift_steering_angle)
    assert result['stale'] is True
    assert result['suggested_offset'] is None
    assert 'recorded with offset 0.5904, but the car now uses 0.5734' in result['blocked']
