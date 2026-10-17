"""The settings-only clamp check, and the write decisions for the new values.

Oracle for the clamp: the closed form servo = gain * angle + offset inverted
at servo_min / servo_max, recomputed here, and the 2026-10-10 state of car 2
recorded in docs/odom-calibration.md (gain -1.2135, offset 0.5904, clip
0.15-0.85: full right stopped at 12.3 deg while full stick asked 19.5).
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import calibration_math, steering_chain, wizard_actions  # noqa: E402

OLD = {'steering_angle_to_servo_gain': -1.2135, 'steering_angle_to_servo_offset': 0.5904}
NEW = {'steering_angle_to_servo_gain': -0.78, 'steering_angle_to_servo_offset': 0.5904}


def test_the_2026_10_10_clip_is_found_from_settings_alone():
    result = steering_chain.clamp_check(OLD, (0.15, 0.85), 0.34, 0.26)
    assert math.degrees(result['right_reach_rad']) == pytest.approx(
        math.degrees((0.85 - 0.5904) / 1.2135), abs=1e-9)
    assert math.degrees(result['left_reach_rad']) == pytest.approx(
        math.degrees((0.5904 - 0.15) / 1.2135), abs=1e-9)
    right = [f for f in result['findings'] if 'right side' in f]
    assert len(right) == 2          # full stick and the driving nodes
    assert all('servo_max' in f for f in right)
    assert not [f for f in result['findings'] if 'left side' in f]


def test_the_2026_10_16_settings_clip_nothing():
    result = steering_chain.clamp_check(NEW, (0.15, 0.95), math.radians(25.0), 0.349)
    assert result['findings'] == []


@pytest.mark.parametrize('extra,clipped', [(0.0, False), (1e-5, True)])
def test_clamp_boundary(extra, clipped):
    reach = (0.95 - 0.5904) / 0.78
    result = steering_chain.clamp_check(NEW, (0.15, 0.95), reach + extra, None)
    assert bool(result['findings']) is clipped


def test_a_positive_gain_clips_left_at_servo_max():
    params = {'steering_angle_to_servo_gain': 1.0, 'steering_angle_to_servo_offset': 0.7}
    result = steering_chain.clamp_check(params, (0.15, 0.85), 0.2, None)
    assert result['left_reach_rad'] == pytest.approx(0.15, abs=1e-12)
    assert any('servo_max cuts the left' in f for f in result['findings'])


def test_an_offset_outside_the_limits_is_reported():
    params = {'steering_angle_to_servo_gain': -0.78, 'steering_angle_to_servo_offset': 0.96}
    result = steering_chain.clamp_check(params, (0.15, 0.95), 0.4, None)
    assert 'outside servo_min' in result['findings'][0]
    assert result['left_reach_rad'] is None


@pytest.mark.parametrize('params,limits', [
    ({}, (0.15, 0.95)),
    ({'steering_angle_to_servo_gain': 0.0, 'steering_angle_to_servo_offset': 0.5}, (0.15, 0.95)),
    (NEW, None),
    (NEW, (None, 0.95)),
])
def test_missing_settings_claim_nothing(params, limits):
    result = steering_chain.clamp_check(params, limits, 0.4, 0.3)
    assert result['findings'] == [] and result['left_reach_rad'] is None


# --- vetting the new suggestions ---------------------------------------------

@pytest.mark.parametrize('name,value,match', [
    ('servo_max', 1.01, 'outside the 0-1'),
    ('servo_max', -0.01, 'outside the 0-1'),
    ('servo_max', 0.15, 'no servo range'),           # equal to servo_min
    ('servo_min', 0.95, 'no servo range'),
    ('steering_stick_scale', 0.0, 'outside 0-0.70'),
    ('steering_stick_scale', 0.7001, 'outside 0-0.70'),
    ('steering_stick_scale', float('nan'), 'no usable value'),
])
def test_impossible_limits_are_blocked(name, value, match):
    blocked, _ = calibration_math.vet_suggestion(name, value, None, 'good', (0.15, 0.95))
    assert blocked is not None and match in blocked


@pytest.mark.parametrize('name,value', [
    ('servo_max', 1.0), ('servo_max', 0.1501), ('servo_min', 0.0),
    ('steering_stick_scale', 0.70), ('steering_stick_scale', 0.436)])
def test_possible_limits_pass(name, value):
    assert calibration_math.vet_suggestion(name, value, None, 'good', (0.15, 0.95)) == (None, None)


# --- what a write touches ----------------------------------------------------

def test_a_report_write_splits_into_the_two_files():
    report = {'parameter_suggestions': {'servo_max': 0.95, 'steering_stick_scale': 0.436,
                                        'steering_angle_to_servo_gain': -0.79}}
    updates = wizard_actions.report_updates(
        report, ['servo_max', 'steering_stick_scale', 'steering_angle_to_servo_gain'])
    vesc, scale = wizard_actions.split_updates(updates)
    assert vesc == {'servo_max': 0.95, 'steering_angle_to_servo_gain': -0.79}
    assert scale == 0.436


def test_a_vesc_only_write_has_no_scale():
    assert wizard_actions.split_updates({'servo_max': 0.95}) == ({'servo_max': 0.95}, None)


def test_a_not_ready_scale_is_refused():
    report = {'parameter_suggestions': {'steering_stick_scale': 0.436},
              'not_ready_to_apply': {'steering_stick_scale': 'confidence is low'}}
    with pytest.raises(ValueError, match='not ready to apply'):
        wizard_actions.report_updates(report, ['steering_stick_scale'])


def test_restart_is_pending_until_both_files_of_one_write_are_live():
    session = {'applied_changes': [
        {'apply_id': 'a', 'after': {'servo_max': 0.95}, 'restored': False},
        {'apply_id': 'a', 'after': {'steering_stick_scale': 0.436}, 'restored': False},
    ]}
    live = {'servo_max': 0.95, 'steering_stick_scale': 0.34}
    assert wizard_actions.pending_restart(session, live) == ['steering_stick_scale']
    live['steering_stick_scale'] = 0.436
    assert wizard_actions.pending_restart(session, live) == []
    live['servo_max'] = 0.85
    assert wizard_actions.pending_restart(session, live) == ['servo_max']


def test_an_earlier_write_does_not_count_once_a_later_one_exists():
    session = {'applied_changes': [
        {'apply_id': 'a', 'after': {'servo_max': 0.90}, 'restored': False},
        {'apply_id': 'b', 'after': {'servo_max': 0.95}, 'restored': False},
    ]}
    assert wizard_actions.pending_restart(session, {'servo_max': 0.95}) == []


def test_session_follows_the_live_stick_scale():
    session = {'current_parameters': {}, 'vehicle': {'steering_stick_scale': 0.34},
               'report': {'old': True}}
    changed = wizard_actions.sync_session_parameters(session, {}, [None, None], 0.436)
    assert changed == ['steering_stick_scale 0.34 -> 0.436']
    assert session['vehicle']['steering_stick_scale'] == 0.436
    assert session['report'] is None


@pytest.mark.parametrize('scale', [None, float('nan')])
def test_an_unknown_stick_scale_changes_nothing(scale):
    session = {'current_parameters': {}, 'vehicle': {'steering_stick_scale': 0.34},
               'report': {'old': True}}
    assert wizard_actions.sync_session_parameters(session, {}, [None, None], scale) == []
    assert session['report'] == {'old': True}
