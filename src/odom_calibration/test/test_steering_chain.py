"""The two steering checks that need no extra driving: reach and centre.

The wheels-up "steering range" diagnosis this file used to test was removed
on 2026-10-16 (docs/odom-calibration.md); the settings-only clamp check that
replaced it is tested in test_clamp_and_writes.py.

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
CAR_OFFSET = 0.5904          # vesc.yaml after the 2026-10-10 drift fix


def _params(offset):
    return {'steering_angle_to_servo_gain': GAIN,
            'steering_angle_to_servo_offset': offset, 'wheelbase': 0.36}


# Closed forms for this car on 2026-10-10.
LEFT_REACH = (CAR_OFFSET - LIMITS[0]) / -GAIN       # servo_min side
RIGHT_REACH = (LIMITS[1] - CAR_OFFSET) / -GAIN      # servo_max side


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


def test_a_drift_run_from_before_an_offset_change_suggests_nothing():
    session, _ = _drift_session(0.5734)
    session['current_parameters']['steering_angle_to_servo_offset'] = 0.5734
    result = steering_chain.centre_offset_suggestion(
        session, calibration_math.drift_steering_angle)
    assert result['stale'] is True
    assert result['suggested_offset'] is None
    assert 'recorded with offset 0.5904, but the car now uses 0.5734' in result['blocked']
