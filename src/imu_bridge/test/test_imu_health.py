"""imu_bridge.imu_health: the /imu/status verdict neural_racer requires.

Oracles: standard gravity G = 9.80665 m/s^2 (CGPM 1901) on +z at rest in car
axes (the convention convert.py and vehicle_state.py share); tilts are closed
forms, G*(sin, 0, cos). The contract with neural_racer is its own parser,
json.loads(msg).get('state') == 'ok' (neural_racer_node.imu_status_callback).

No ROS needed:

    python3 -m pytest src/imu_bridge/test/test_imu_health.py -v
"""
import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from imu_bridge.convert import G  # noqa: E402
from imu_bridge.imu_health import ImuHealth  # noqa: E402

DT = 0.01                     # 100 Hz
REST = (0.0, 0.0, G)
STILL_GYRO = (0.0, 0.0, 0.001)


def _feed(h, t0, seconds, acc=REST, gyro=STILL_GYRO, speed=0.0, bias_done=True, raw=None):
    """Samples every DT from t0 for `seconds`; returns the last sample time."""
    t = t0
    for i in range(int(round(seconds / DT)) + 1):
        t = t0 + i * DT
        raw_acc, raw_gyro = raw if raw is not None else (acc, gyro)
        h.sample(t, raw_acc, raw_gyro, acc, gyro, speed, bias_done)
    return t


def _neural_racer_accepts(text):
    # neural_racer_node.imu_status_callback, verbatim.
    return json.loads(text).get('state') == 'ok'


def test_still_upright_imu_with_bias_done_is_ok_and_neural_racer_accepts_it():
    h = ImuHealth('bno', gravity_window_s=1.0)
    t = _feed(h, 0.0, 1.0)
    assert h.status(t) == ('ok', None)
    assert _neural_racer_accepts(h.status_json(t))
    assert json.loads(h.status_json(t))['imu'] == 'bno'


def test_waits_until_the_gravity_window_is_complete():
    h = ImuHealth('vesc', gravity_window_s=1.0)
    t = _feed(h, 0.0, 0.98)
    state, reason = h.status(t)
    assert state == 'waiting' and 'gravity' in reason
    assert not _neural_racer_accepts(h.status_json(t))


def test_waits_while_the_startup_bias_is_not_done_even_with_gravity_ok():
    h = ImuHealth('vesc', gravity_window_s=1.0)
    t = _feed(h, 0.0, 2.0, bias_done=False)
    state, reason = h.status(t)
    assert state == 'waiting' and 'bias' in reason
    t = _feed(h, t + DT, 0.0, bias_done=True)
    assert h.status(t) == ('ok', None)


def test_moving_restarts_the_gravity_window():
    h = ImuHealth('vesc', gravity_window_s=1.0)
    t = _feed(h, 0.0, 0.9)
    t = _feed(h, t + DT, 0.0, speed=0.5)            # one moving sample
    t = _feed(h, t + DT, 0.9)                        # 0.9 s still again: not a full window
    assert h.status(t)[0] == 'waiting'
    t = _feed(h, t + DT, 0.2)
    assert h.status(t)[0] == 'ok'


@pytest.mark.parametrize('speed', [None, math.nan, 0.03])
def test_unknown_or_too_fast_wheel_speed_never_counts_as_still(speed):
    # 0.03 m/s is exactly the still_speed limit, which is exclusive.
    h = ImuHealth('vesc', gravity_window_s=1.0)
    t = _feed(h, 0.0, 3.0, speed=speed)
    assert h.status(t)[0] == 'waiting'


def test_upside_down_mount_faults_and_stays_faulted():
    h = ImuHealth('bno', gravity_window_s=1.0)
    t = _feed(h, 0.0, 1.0, acc=(0.0, 0.0, -G))
    state, reason = h.status(t)
    assert state == 'fault' and 'from +z' in reason
    # Driving, then a clean still window that would pass on its own, cannot
    # clear it: a wrong mount does not fix itself.
    t = _feed(h, t + DT, 0.0, speed=1.0)
    t = _feed(h, t + DT, 2.0)
    assert h.status(t)[0] == 'fault'


@pytest.mark.parametrize('norm, ok', [(G + 0.5, True), (G - 0.5, True),
                                      (G + 0.51, False), (G - 0.51, False),
                                      (G * math.pi / 180.0, False)])   # deg/s-style unit slip
def test_gravity_magnitude_boundary(norm, ok):
    h = ImuHealth('vesc', gravity_window_s=1.0, gravity_tol=0.5)
    t = _feed(h, 0.0, 1.0, acc=(0.0, 0.0, norm))
    assert (h.status(t)[0] == 'ok') is ok, h.status(t)


@pytest.mark.parametrize('tilt_deg, ok', [(10.0, True), (10.5, False), (90.0, False)])
def test_gravity_tilt_boundary(tilt_deg, ok):
    a = math.radians(tilt_deg)
    h = ImuHealth('vesc', gravity_window_s=1.0, max_tilt_deg=10.0)
    t = _feed(h, 0.0, 1.0, acc=(G * math.sin(a), 0.0, G * math.cos(a)))
    assert (h.status(t)[0] == 'ok') is ok, h.status(t)


def test_all_zero_samples_fault_then_recover_when_the_imu_comes_back():
    h = ImuHealth('vesc', gravity_window_s=1.0)
    t = _feed(h, 0.0, 1.0)
    t = _feed(h, t + DT, 0.0, raw=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    state, reason = h.status(t)
    assert state == 'fault' and 'all zero' in reason
    t = _feed(h, t + DT, 0.0)
    assert h.status(t) == ('ok', None)


def test_all_zero_from_power_up_never_reaches_ok():
    # Calibration can turn raw zeros into a non-zero vector; the raw check is
    # what catches a disabled IMU, so feed calibrated "gravity" with raw zeros.
    h = ImuHealth('vesc', gravity_window_s=1.0)
    t = _feed(h, 0.0, 3.0, raw=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    assert h.status(t)[0] == 'fault'


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_non_finite_sample_faults(bad):
    h = ImuHealth('vesc', gravity_window_s=1.0)
    t = _feed(h, 0.0, 1.0)
    t = _feed(h, t + DT, 0.0, gyro=(0.0, 0.0, bad))
    state, reason = h.status(t)
    assert state == 'fault' and 'non-finite' in reason


def test_wrong_axis_count_faults():
    h = ImuHealth('vesc')
    h.sample(0.0, (0.0, G), (0.0, 0.0, 0.0), (0.0, G), (0.0, 0.0, 0.0), 0.0, True)
    assert h.status(0.0)[0] == 'fault'


def test_no_samples_is_a_fault_not_waiting():
    assert ImuHealth('vesc').status(0.0) == ('fault', 'no IMU samples yet')


def test_stale_input_faults_past_the_timeout_and_not_before():
    h = ImuHealth('vesc', stale_timeout_s=0.1, gravity_window_s=1.0)
    t = _feed(h, 0.0, 1.0)
    assert h.status(t + 0.1)[0] == 'ok'              # exactly at the limit
    state, reason = h.status(t + 0.1001)
    assert state == 'fault' and 'stale' in reason


@pytest.mark.parametrize('field', ['stale_timeout_s', 'gravity_window_s', 'gravity_tol', 'max_tilt_deg'])
@pytest.mark.parametrize('value', [0.0, -1.0, math.nan, math.inf])
def test_rejects_non_positive_or_non_finite_settings(field, value):
    with pytest.raises(ValueError, match=field):
        ImuHealth('vesc', **{field: value})
