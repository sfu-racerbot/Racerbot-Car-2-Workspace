"""Overrides: values typed on the page, the servo-limit step, undo-all, and
runs kept despite warnings.

Oracles: the servo map servo = gain * angle + offset (vesc.yaml), car 2's
2026-10-16 settings (gain -0.78, offset 0.5904, clip 0.15-0.95, full stick
0.436 rad; docs/hardware-reference.md), and invariants stated per test.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import calibration_math, wizard_actions  # noqa: E402

CURRENT = {'steering_angle_to_servo_gain': -0.78, 'steering_angle_to_servo_offset': 0.5904,
           'speed_to_erpm_gain': -4447.983786, 'speed_to_erpm_offset': 0.0,
           'servo_min': 0.15, 'servo_max': 0.95, 'steering_stick_scale': 0.436}
LIMITS = (0.15, 0.95)


# --- values typed by hand ----------------------------------------------------

def test_typed_values_are_written_as_typed():
    updates = wizard_actions.manual_updates(
        {'servo_max': 0.97, 'steering_stick_scale': 0.45}, CURRENT, LIMITS)
    assert updates == {'servo_max': 0.97, 'steering_stick_scale': 0.45}


@pytest.mark.parametrize('values,match', [
    ({}, 'at least one value'),
    ({'brake_max': 1.0}, 'cannot be written'),
    ({'servo_max': float('nan')}, 'finite number'),
    ({'servo_max': True}, 'finite number'),
    ({'servo_max': '0.9'}, 'finite number'),
    ({'servo_max': 1.2}, 'outside the 0-1'),
    ({'servo_max': 0.15}, 'no servo range'),
    ({'steering_angle_to_servo_gain': 0.78}, 'left and right would swap'),
    ({'steering_stick_scale': 0.8}, 'outside 0-0.70'),
    ({'speed_to_erpm_gain': 4447.98}, 'reverse of the configured gain'),
])
def test_typed_values_still_get_the_hard_checks(values, match):
    with pytest.raises(ValueError, match=match):
        wizard_actions.manual_updates(values, CURRENT, LIMITS)


def test_limits_that_would_strand_the_offset_are_refused():
    # servo_max 0.58 is below the 0.5904 offset: full right would not exist.
    with pytest.raises(ValueError, match='outside them'):
        wizard_actions.manual_updates({'servo_max': 0.58}, CURRENT, LIMITS)


def test_offset_and_limits_moved_together_are_checked_together():
    updates = wizard_actions.manual_updates(
        {'steering_angle_to_servo_offset': 0.50, 'servo_max': 0.58}, CURRENT, LIMITS)
    assert updates['servo_max'] == 0.58


def test_low_confidence_does_not_block_a_typed_value():
    # Confidence is about the fit's evidence; a person's chosen number has none.
    report = {'parameter_suggestions': {'steering_angle_to_servo_gain': -0.79},
              'not_ready_to_apply': {'steering_angle_to_servo_gain': 'confidence is low'}}
    with pytest.raises(ValueError, match='not ready to apply'):
        wizard_actions.report_updates(report, ['steering_angle_to_servo_gain'])
    assert wizard_actions.manual_updates(
        {'steering_angle_to_servo_gain': -0.79}, CURRENT, LIMITS) == \
        {'steering_angle_to_servo_gain': -0.79}


# --- the servo-limit step ----------------------------------------------------

def test_right_step_raises_servo_max_with_a_negative_gain():
    updates = wizard_actions.limit_step_updates(CURRENT, LIMITS, 0.436, 'right', 0.05, False)
    assert updates == {'servo_max': pytest.approx(1.00, abs=1e-9)}


def test_left_step_lowers_servo_min_with_a_negative_gain():
    updates = wizard_actions.limit_step_updates(CURRENT, LIMITS, 0.436, 'left', 0.05, False)
    assert updates == {'servo_min': pytest.approx(0.10, abs=1e-9)}


def test_a_positive_gain_swaps_which_limit_each_side_moves():
    current = {**CURRENT, 'steering_angle_to_servo_gain': 0.78}
    assert set(wizard_actions.limit_step_updates(current, LIMITS, 0.436, 'left', 0.05, False)) \
        == {'servo_max'}
    assert set(wizard_actions.limit_step_updates(current, LIMITS, 0.436, 'right', 0.05, False)) \
        == {'servo_min'}


def test_full_stick_is_raised_only_when_it_would_not_reach_the_new_limit():
    # Full right at 0.436 rad sends 0.5904 + 0.78 * 0.436 = 0.9305: short of 1.00.
    updates = wizard_actions.limit_step_updates(CURRENT, LIMITS, 0.436, 'right', 0.05, True)
    needed = (1.00 - 0.5904) / 0.78
    assert updates['steering_stick_scale'] == pytest.approx(
        math.ceil((needed + 0.01) * 100) / 100, abs=1e-12)
    assert 0.5904 + 0.78 * updates['steering_stick_scale'] > 1.00   # the clamp decides
    # 0.60 rad already reaches 1.00 (0.5904 + 0.468 = 1.058): left alone.
    updates = wizard_actions.limit_step_updates(CURRENT, LIMITS, 0.60, 'right', 0.05, True)
    assert 'steering_stick_scale' not in updates


def test_full_stick_is_left_alone_when_not_asked():
    updates = wizard_actions.limit_step_updates(CURRENT, LIMITS, 0.436, 'right', 0.05, False)
    assert 'steering_stick_scale' not in updates


def test_the_step_stops_at_the_end_of_the_servo_signal():
    updates = wizard_actions.limit_step_updates(CURRENT, (0.15, 0.98), 0.6, 'right', 0.05, False)
    assert updates == {'servo_max': 1.0}
    with pytest.raises(ValueError, match='already 1, the end'):
        wizard_actions.limit_step_updates(CURRENT, (0.15, 1.0), 0.6, 'right', 0.05, False)
    with pytest.raises(ValueError, match='already 0, the end'):
        wizard_actions.limit_step_updates(CURRENT, (0.0, 0.95), 0.6, 'left', 0.05, False)


@pytest.mark.parametrize('step,ok', [(0.005, True), (0.0049, False), (0.10, True),
                                     (0.1001, False), (-0.05, False), (float('nan'), False),
                                     (True, False)])
def test_step_size_bounds(step, ok):
    if ok:
        wizard_actions.limit_step_updates(CURRENT, LIMITS, 0.436, 'right', step, False)
    else:
        with pytest.raises(ValueError, match='step must be between'):
            wizard_actions.limit_step_updates(CURRENT, LIMITS, 0.436, 'right', step, False)


@pytest.mark.parametrize('current,limits,side,match', [
    ({**CURRENT, 'steering_angle_to_servo_gain': 0.0}, LIMITS, 'right', 'not known yet'),
    (CURRENT, None, 'right', 'servo limits are not known'),
    (CURRENT, (None, 0.95), 'right', 'servo limits are not known'),
    (CURRENT, LIMITS, 'up', "'left' or 'right'"),
])
def test_step_refuses_without_what_it_needs(current, limits, side, match):
    with pytest.raises(ValueError, match=match):
        wizard_actions.limit_step_updates(current, limits, 0.436, side, 0.05, False)


def test_a_step_too_far_for_full_stick_is_refused_by_the_hard_checks():
    # From servo_max 0.95 at offset 0.05, reaching 1.00 needs 1.21 rad of stick.
    current = {**CURRENT, 'steering_angle_to_servo_offset': 0.05, 'servo_min': 0.0}
    step = wizard_actions.limit_step_updates(current, (0.0, 0.95), 0.4, 'right', 0.05, True)
    with pytest.raises(ValueError, match='outside 0-0.70'):
        wizard_actions.manual_updates(step, current, (0.0, 0.95))


# --- undo --------------------------------------------------------------------

def _changes():
    return [
        {'apply_id': 'a', 'path': '/v', 'backup': 'v1', 'restored': False},
        {'apply_id': 'b', 'path': '/v', 'backup': 'v2', 'restored': False},
        {'apply_id': 'b', 'path': '/t', 'backup': 't2', 'restored': False},
        {'apply_id': 'c', 'path': '/v', 'backup': 'v3', 'restored': False},
    ]


def test_undo_puts_back_the_latest_write_only():
    restore, close = wizard_actions.restore_plan(_changes())
    assert [c['backup'] for c in restore] == ['v3'] and close == restore


def test_undo_of_a_two_file_write_puts_back_both():
    changes = _changes()[:3]
    restore, _ = wizard_actions.restore_plan(changes)
    assert sorted(c['backup'] for c in restore) == ['t2', 'v2']


def test_undo_everything_returns_each_file_to_before_the_first_write():
    changes = _changes()
    restore, close = wizard_actions.restore_plan(changes, everything=True)
    assert {c['path']: c['backup'] for c in restore} == {'/v': 'v1', '/t': 't2'}
    assert close == changes


def test_undo_everything_skips_what_was_already_undone():
    changes = _changes()
    changes[0]['restored'] = True
    restore, close = wizard_actions.restore_plan(changes, everything=True)
    assert {c['path']: c['backup'] for c in restore} == {'/v': 'v2', '/t': 't2'}
    assert len(close) == 3


def test_nothing_to_undo_is_refused():
    with pytest.raises(ValueError, match='left to undo'):
        wizard_actions.restore_plan([{'path': '/v', 'backup': 'x', 'restored': True}])
    with pytest.raises(ValueError, match='left to undo'):
        wizard_actions.restore_plan([], everything=True)


# --- runs kept despite warnings ----------------------------------------------

def _session(*circles):
    trials = [{'id': 'drift', 'kind': 'steering_drift', 'accepted': True,
               'summary': {'servo_active': {'median': 0.5904, 'count': 100}},
               'measured_forward_m': 5.0, 'measured_lateral_m': 0.0}]
    trials += list(circles)
    return {'mode': 'steering', 'trials': trials,
            'current_parameters': {'steering_angle_to_servo_gain': -0.78,
                                   'steering_angle_to_servo_offset': 0.5904, 'wheelbase': 0.36},
            'vehicle': {'servo_min': 0.15, 'servo_max': 0.95, 'rear_track_m': 0.27}}


def _circle(identifier, kind, servo, inner, count=100, overridden=None):
    trial = {'id': identifier, 'kind': kind, 'accepted': True,
             'summary': {'servo_active': {'median': servo, 'count': count}},
             'measurement_method': 'inner_tire', 'full_lock': True,
             'measured_inner_diameter_m': inner}
    if overridden is not None:
        trial['warnings_overridden'] = overridden
    return trial


def test_a_short_run_is_used_only_when_kept_anyway():
    left = _circle('l', 'steering_left', 0.25, 1.33)
    for overridden, usable in ((None, False), (False, False), (True, True)):
        right = _circle('r', 'steering_right', 0.93, 1.33, count=4, overridden=overridden)
        report = calibration_math.steering_calibration(_session(left, right))
        item = next(i for i in report['trial_results'] if i['trial_id'] == 'r')
        assert item['usable'] is usable
        assert any('used anyway' in w for w in item['warnings']) is usable


def test_a_run_with_no_moving_samples_is_never_used():
    right = _circle('r', 'steering_right', 0.93, 1.33, count=0, overridden=True)
    report = calibration_math.steering_calibration(_session(right))
    assert report['trial_results'][-1]['usable'] is False


def test_disagreeing_repeats_lower_confidence_unless_every_one_is_kept():
    left = _circle('l', 'steering_left', 0.25, 1.33)
    for kept, status_low in (((False, False), True), ((True, False), True),
                             ((True, True), False)):
        # Axle radii 0.785 and 0.885 m: 12% apart, past the 10% repeat limit.
        r1 = _circle('r1', 'steering_right', 0.93, 1.30, overridden=kept[0])
        r2 = _circle('r2', 'steering_right', 0.93, 1.50, overridden=kept[1])
        report = calibration_math.steering_calibration(_session(left, r1, r2))
        assert report['fit_rmse_servo'] <= 0.02, 'only the repeat check may lower it'
        disagree = [w for w in report['warnings'] if 'disagree by' in w]
        assert disagree, 'the disagreement is always shown'
        assert ('kept anyway' in disagree[0]) is (not status_low)
        assert report['status'] == ('low' if status_low else 'good')
