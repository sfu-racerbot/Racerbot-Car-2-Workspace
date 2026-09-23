import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import calibration_math  # noqa: E402
from odom_calibration import session_store  # noqa: E402

PARAMETERS = {
    'speed_to_erpm_gain': -4614.0,
    'speed_to_erpm_offset': 0.0,
    'steering_angle_to_servo_gain': -1.2135,
    'steering_angle_to_servo_offset': 0.5304,
    'wheelbase': 0.36,
}


def test_steering_only_session_can_be_created():
    session = session_store.new_session('steering', PARAMETERS)
    assert session['mode'] == 'steering'
    assert session['stage'] == 'preflight'


def test_unknown_mode_is_rejected():
    with pytest.raises(ValueError, match='mode must be one of'):
        session_store.new_session('drift', PARAMETERS)


@pytest.mark.parametrize('mode,stage,allowed', [
    ('steering', 'preflight', True),
    ('steering', 'steering', True),
    ('steering', 'report', True),
    ('steering', 'stationary', False),
    ('steering', 'movement', False),
    ('movement', 'steering', False),
    ('movement', 'movement', True),
    ('movement_steering', 'steering', True),
    ('movement_steering', 'stationary', True),
    ('unknown', 'preflight', False),
    ('steering', 'nonsense', False),
])
def test_stage_allowed_follows_mode_table(mode, stage, allowed):
    # Spec: MODE_STAGES in the plan's Task 2 interfaces.
    assert session_store.stage_allowed(mode, stage) is allowed


@pytest.mark.parametrize('mode,kind,allowed', [
    ('steering', 'steering_drift', True),
    ('steering', 'steering_left', True),
    ('steering', 'steering_center', True),
    ('steering', 'stationary', False),
    ('steering', 'movement', False),
    ('movement', 'steering_drift', False),
    ('movement', 'movement', True),
    ('movement', 'stationary', True),
    ('movement_steering', 'steering_right', True),
    ('movement_steering', 'movement', True),
    ('movement_steering', 'teleport', False),
    ('unknown', 'movement', False),
])
def test_capture_allowed_follows_mode_table(mode, kind, allowed):
    assert session_store.capture_allowed(mode, kind) is allowed


def test_every_mode_starts_at_preflight_and_ends_at_report():
    # Invariant the UI's step list relies on.
    for mode in session_store.VALID_MODES:
        stages = session_store.MODE_STAGES[mode]
        assert stages[0] == 'preflight'
        assert stages[-1] == 'report'


def _samples():
    # 10 Hz, value 2.0, with a NaN, a repeated timestamp, and a 1 s gap.
    samples = [{'t': 0.1 * i, 'v': 2.0} for i in range(11)]
    samples.insert(3, {'t': 0.25, 'v': float('nan')})
    samples.insert(6, {'t': 0.4, 'v': 2.0})  # duplicate timestamp
    samples += [{'t': 2.0 + 0.1 * i, 'v': 2.0} for i in range(6)]
    return samples


def test_running_integral_matches_batch_integrator():
    running = calibration_math.RunningIntegral(max_gap_sec=0.5)
    for sample in _samples():
        running.add(sample['t'], sample['v'])
    batch = calibration_math.integrate_samples(_samples(), 'v', max_gap_sec=0.5)
    # Oracle: the existing batch integrator (independent code path).
    assert running.total == pytest.approx(batch['integral'], abs=1e-12)


def test_running_integral_closed_form():
    running = calibration_math.RunningIntegral(max_gap_sec=0.5)
    for i in range(11):
        running.add(0.1 * i, 2.0)
    # Closed form: constant 2.0 for 1.0 s integrates to 2.0.
    assert running.total == pytest.approx(2.0, abs=1e-12)


def test_running_integral_skips_large_gap():
    running = calibration_math.RunningIntegral(max_gap_sec=0.5)
    running.add(0.0, 1.0)
    running.add(0.1, 1.0)
    running.add(5.0, 1.0)  # 4.9 s gap: not integrated
    running.add(5.1, 1.0)
    # Closed form: two 0.1 s intervals of value 1.0.
    assert running.total == pytest.approx(0.2, abs=1e-12)


def test_running_integral_ignores_non_finite_and_backwards_time():
    running = calibration_math.RunningIntegral(max_gap_sec=0.5)
    running.add(0.0, 1.0)
    running.add(0.1, float('inf'))
    running.add(0.05, 1.0)  # earlier than the last accepted sample
    running.add(0.2, 1.0)
    # Only 0.0 -> 0.2 at value 1.0 counts.
    assert running.total == pytest.approx(0.2, abs=1e-12)


def test_running_integral_starts_at_zero():
    assert calibration_math.RunningIntegral().total == 0.0


def test_running_integral_rejects_bad_gap():
    with pytest.raises(ValueError, match='max_gap_sec'):
        calibration_math.RunningIntegral(max_gap_sec=0.0)
