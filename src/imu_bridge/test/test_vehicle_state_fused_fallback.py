"""Exact wheel-speed fallback; pytest src/imu_bridge/test/ and colcon test.

Existing test_vehicle_state.py already proves never-received BNO samples in
test_no_imu_means_exactly_the_wheel_speed and timeout boundaries in
test_stale_accel_falls_back_to_exactly_the_wheel_speed. Its
test_bno_dropout_mid_run_falls_back_then_resumes uses an approximate speed
comparison; below pins exact equality after an active filter loses samples.
test_non_finite_imu_samples_are_dropped checks yaw, not fused speed, so the
non-finite speed fallback is covered here too.
"""
import math

import pytest

from imu_bridge.vehicle_state import (
    EstimatorConfig, GRAVITY_MIN_SAMPLES, MODE_WHEEL_ONLY, VehicleStateEstimator)


def _active_estimator():
    est = VehicleStateEstimator(EstimatorConfig())
    est.on_wheel(0.0, 1.0)
    for k in range(1, GRAVITY_MIN_SAMPLES + 20):
        t = k * 0.004
        est.on_imu('bno', t, t, 2.0, 9.80665, 0.0)
    estimate = est.on_wheel(t, 1.0)
    # oracle: invariant -- nonzero acceleration has actually moved the fused
    # speed off the wheels, so the later fallback cannot pass by coincidence.
    assert estimate.speed != 1.0
    assert estimate.mode != MODE_WHEEL_ONLY
    return est, t


def test_stopped_bno_samples_return_exact_wheel_speed_after_active_fusion():
    est, last = _active_estimator()
    for k, wheel in enumerate((1.13, 0.0, -0.7, 2.23)):
        t = last + est.cfg.imu_timeout + 0.001 + k * 0.02
        result = est.on_wheel(t, wheel)
        # oracle: spec -- src/imu_bridge/README.md fallback table: exactly the
        # wheel speed when BNO samples stop, not the old fused estimate.
        assert result.speed == wheel
        assert result.mode == MODE_WHEEL_ONLY


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('field', ['accel_x', 'accel_z', 'yaw_rate'])
@pytest.mark.parametrize('active_first', [False, True], ids=['startup', 'mid-run'])
def test_non_finite_bno_samples_leave_exact_wheel_speed(bad, field, active_first):
    est, last = _active_estimator() if active_first else (
        VehicleStateEstimator(EstimatorConfig()), 0.0)
    # Invalid samples cannot supply acceleration or refresh its age. An already
    # accepted sample remains usable until imu_timeout, just as after dropout.
    for k, wheel in enumerate((0.0, 1.13, -0.7)):
        t = last + est.cfg.imu_timeout + 0.001 + k * 0.02
        values = dict(accel_x=2.0, accel_z=9.80665, yaw_rate=0.0)
        values[field] = bad
        est.on_imu('bno', t, t, **values)
        result = est.on_wheel(t, wheel)
        # oracle: invariant -- no usable accel means exactly the wheel speed
        # (vehicle_state.py module contract), including a formerly active filter.
        assert result.speed == wheel
        assert result.mode == MODE_WHEEL_ONLY
