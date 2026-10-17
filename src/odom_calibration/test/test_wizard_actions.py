"""What the wizard may write to vesc.yaml, and how it follows the car."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import wizard_actions  # noqa: E402

LIMITS = (0.15, 0.85)


def _report(**overrides):
    report = {
        'parameter_suggestions': {'steering_angle_to_servo_gain': -1.18,
                                  'steering_angle_to_servo_offset': 0.6},
        'blocked_suggestions': {'speed_to_erpm_gain': 'the speed_to_erpm_gain fit is insufficient'},
        'not_ready_to_apply': {'steering_angle_to_servo_offset': 'confidence is low'},
    }
    report.update(overrides)
    return report


def test_ready_report_value_becomes_an_update():
    assert wizard_actions.report_updates(_report(), ['steering_angle_to_servo_gain']) == {
        'steering_angle_to_servo_gain': -1.18}


@pytest.mark.parametrize('report,names,match', [
    (None, ['steering_angle_to_servo_gain'], 'Generate the report first'),
    ({}, ['steering_angle_to_servo_gain'], 'Generate the report first'),
    (_report(), [], 'at least one value'),
    (_report(), 'steering_angle_to_servo_gain', 'at least one value'),
    (_report(), ['brake_max'], "'brake_max' cannot be applied"),
    (_report(), ['speed_to_erpm_gain'], 'blocked: the speed_to_erpm_gain fit is insufficient'),
    (_report(), ['steering_angle_to_servo_offset'], 'not ready to apply: confidence is low'),
    (_report(), ['speed_to_erpm_offset'], 'no suggestion for speed_to_erpm_offset'),
    (_report(parameter_suggestions={'steering_angle_to_servo_gain': float('nan')}),
     ['steering_angle_to_servo_gain'], 'no suggestion'),
])
def test_report_refuses_anything_not_ready(report, names, match):
    with pytest.raises(ValueError, match=match):
        wizard_actions.report_updates(report, names)


def test_one_bad_name_refuses_the_whole_request():
    with pytest.raises(ValueError, match='blocked'):
        wizard_actions.report_updates(
            _report(), ['steering_angle_to_servo_gain', 'speed_to_erpm_gain'])


def _drift_session(servo, lateral, configured=0.5904, count=40, forward=5.0):
    params = {'steering_angle_to_servo_gain': -1.2135,
              'steering_angle_to_servo_offset': configured, 'wheelbase': 0.36}
    return {'current_parameters': dict(params), 'trials': [{
        'id': 'd', 'kind': 'steering_drift', 'accepted': True,
        'parameters_at_capture': params,
        'measured_forward_m': forward, 'measured_lateral_m': lateral,
        'summary': {'servo_active': {'median': servo, 'count': count}}}]}


def test_centre_update_is_the_straight_ahead_servo():
    # Closed form: offset = servo - gain * atan(L * 2d / (s^2 + d^2)).
    import math
    angle = math.atan(0.36 * 2 * 0.2 / (25.0 + 0.04))
    update = wizard_actions.centre_updates(_drift_session(0.5904, 0.2), LIMITS)
    assert update['steering_angle_to_servo_offset'] == pytest.approx(
        0.5904 + 1.2135 * angle, abs=1e-12)


def test_centre_update_refuses_without_a_drift_run():
    with pytest.raises(ValueError, match='drive-straight run first'):
        wizard_actions.centre_updates({'trials': [], 'current_parameters': {}}, LIMITS)


def test_centre_update_refuses_an_offset_outside_the_servo_range():
    # The servo sat at 0.86 already (a session from wider limits): even a
    # tiny correction stays outside today's 0.15-0.85.
    with pytest.raises(ValueError, match='outside the servo range'):
        wizard_actions.centre_updates(_drift_session(0.86, 0.0, configured=0.86), LIMITS)


def test_centre_update_refuses_a_blocked_suggestion():
    # 0.5 m sideways over 1 m: atan(0.36 * 1.0 / 1.25) = 0.28 rad, an offset
    # change of 0.34, far past the 0.10 one-step limit.
    with pytest.raises(ValueError, match='more than 0.10 in one step'):
        wizard_actions.centre_updates(_drift_session(0.5904, 0.5, forward=1.0), LIMITS)


def _session():
    return {'current_parameters': {'steering_angle_to_servo_offset': 0.5904,
                                   'steering_angle_to_servo_gain': -1.2135,
                                   'wheelbase': 0.36},
            'vehicle': {'servo_min': 0.15, 'servo_max': 0.85},
            'report': {'stale': True}}


def test_sync_follows_a_value_changed_on_the_car():
    session = _session()
    changed = wizard_actions.sync_session_parameters(
        session, {'steering_angle_to_servo_offset': 0.5712}, LIMITS)
    assert changed == ['steering_angle_to_servo_offset 0.5904 -> 0.5712']
    assert session['current_parameters']['steering_angle_to_servo_offset'] == 0.5712
    assert session['report'] is None


def test_sync_follows_new_servo_limits():
    session = _session()
    changed = wizard_actions.sync_session_parameters(session, {}, (0.1, 0.9))
    assert changed == ['servo_min 0.15 -> 0.1', 'servo_max 0.85 -> 0.9']
    assert session['vehicle']['servo_min'] == 0.1
    assert session['report'] is None


def test_sync_with_nothing_new_keeps_the_report():
    session = _session()
    assert wizard_actions.sync_session_parameters(
        session, {'steering_angle_to_servo_offset': 0.5904}, LIMITS) == []
    assert session['report'] == {'stale': True}


def test_sync_ignores_untracked_and_non_finite_values():
    session = _session()
    changed = wizard_actions.sync_session_parameters(
        session, {'speed_to_erpm_gain': -4000.0,
                  'steering_angle_to_servo_gain': float('nan')}, (float('nan'), 0.85))
    assert changed == []
    assert 'speed_to_erpm_gain' not in session['current_parameters']


def test_pending_restart_until_the_car_runs_the_written_value():
    session = {'applied_changes': [{'after': {'steering_angle_to_servo_offset': 0.5712},
                                    'restored': False}]}
    assert wizard_actions.pending_restart(
        session, {'steering_angle_to_servo_offset': 0.5904}) == ['steering_angle_to_servo_offset']
    assert wizard_actions.pending_restart(
        session, {'steering_angle_to_servo_offset': 0.5712}) == []
    assert wizard_actions.pending_restart(session, {}) == ['steering_angle_to_servo_offset']


def test_undone_change_needs_no_restart_notice():
    session = {'applied_changes': [{'after': {'steering_angle_to_servo_offset': 0.5712},
                                    'restored': True}]}
    assert wizard_actions.pending_restart(
        session, {'steering_angle_to_servo_offset': 0.5904}) == []


def test_centre_update_refuses_a_run_recorded_under_an_older_offset():
    session = _drift_session(0.5904, 0.2)
    session['current_parameters']['steering_angle_to_servo_offset'] = 0.5734
    with pytest.raises(ValueError, match='record another drive-straight run'):
        wizard_actions.centre_updates(session, LIMITS)
