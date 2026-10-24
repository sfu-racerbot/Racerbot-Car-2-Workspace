"""
imu_bridge.convert: units, mount rotation, calibration, startup bias, covariances.

No ROS needed -- convert.py imports nothing from rclpy:

    python3 -m pytest src/imu_bridge/test/ -v

The cross-check against racer_imu itself reads the imu-calibration repo from
$RACER_IMU_ROOT (default ~/imu-calibration) and skips only if it is absent.

Oracles: every expected value below is a closed form written in the test
(unit definitions, elementary rotations, the calibration formula from
racer_imu/calibration.py's docstring), an invariant (orthonormality), or
the sensor_msgs/Imu.msg definition. Tolerances are absolute, with units.
"""
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from imu_bridge import convert  # noqa: E402
from imu_bridge.convert import (  # noqa: E402
    G,
    StartupBias,
    covariance_diag,
    load_calibration,
    load_mount,
    monotonic_to_ros_ns,
    resolve_primary,
    rpy_to_R,
    vesc_raw_to_si,
    wheel_twist_covariance,
)

EX, EY, EZ = np.eye(3)
ACC_TOL = 1e-9    # m/s^2
GYRO_TOL = 1e-12  # rad/s
ROT_TOL = 1e-12   # dimensionless (rotation matrix entries)


def _write_calib(path, **overrides):
    data = {
        'A_inv': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        'acc_bias': [0.0, 0.0, 0.0],
        'gyro_bias': [0.0, 0.0, 0.0],
    }
    data.update(overrides)
    data = {k: v for k, v in data.items() if v is not None}
    path.write_text(json.dumps(data))
    return path


# ---------------------------------------------------------------- units

def test_one_g_is_standard_gravity_and_180_dps_is_pi():
    # Closed form: 1 g = 9.80665 m/s^2 (CGPM 1901); 180 deg = pi rad.
    acc, gyro = vesc_raw_to_si([1.0, -2.0, 0.5], [180.0, -90.0, 0.0])
    np.testing.assert_allclose(acc, [9.80665, -2 * 9.80665, 0.5 * 9.80665],
                               rtol=0, atol=ACC_TOL)
    np.testing.assert_allclose(gyro, [math.pi, -math.pi / 2, 0.0], rtol=0, atol=GYRO_TOL)


def test_gravity_constant_is_the_standard_value():
    # Must be exactly racer_imu's G, or calibrated biases (fitted in m/s^2)
    # stop lining up with converted VESC readings.
    assert G == 9.80665


# ---------------------------------------------------------------- rotations

def test_yaw_90_maps_car_x_to_y():
    # Rz(90 deg): x -> y, y -> -x, z unchanged (right-hand rule about +z).
    R = rpy_to_R(0.0, 0.0, math.pi / 2)
    np.testing.assert_allclose(R @ EX, EY, atol=ROT_TOL)
    np.testing.assert_allclose(R @ EY, -EX, atol=ROT_TOL)
    np.testing.assert_allclose(R @ EZ, EZ, atol=ROT_TOL)


def test_roll_90_maps_y_to_z_and_pitch_90_maps_z_to_x():
    # Rx(90): y -> z. Ry(90): z -> x. Signs pin handedness for each axis.
    np.testing.assert_allclose(rpy_to_R(math.pi / 2, 0.0, 0.0) @ EY, EZ, atol=ROT_TOL)
    np.testing.assert_allclose(rpy_to_R(0.0, math.pi / 2, 0.0) @ EZ, EX, atol=ROT_TOL)


def test_order_is_z_y_x():
    # R = Rz(yaw) Ry(pitch) Rx(roll): roll is applied to the vector first.
    # roll 90, yaw 90 on +y: Rx takes y -> z, then Rz leaves z alone -> +z.
    # The opposite order (Rx Rz) would give Rx(Rz y) = Rx(-x) = -x.
    R = rpy_to_R(math.pi / 2, 0.0, math.pi / 2)
    np.testing.assert_allclose(R @ EY, EZ, atol=ROT_TOL)
    # pitch 90, yaw 90 on +z: Ry takes z -> x, then Rz takes x -> y.
    # Ry Rz would give Ry(z) = x.
    np.testing.assert_allclose(rpy_to_R(0.0, math.pi / 2, math.pi / 2) @ EZ, EY, atol=ROT_TOL)


def test_general_angles_match_the_closed_form_zyx_matrix():
    # Standard expansion of Rz(y) Ry(p) Rx(r) (e.g. REP-103 / tf2 Matrix3x3::setRPY).
    r, p, y = 0.3, -0.7, 2.1
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    expected = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    np.testing.assert_allclose(rpy_to_R(r, p, y), expected, rtol=0, atol=ROT_TOL)


@pytest.mark.parametrize('rpy', [(0.0, 0.0, 0.0), (0.3, -0.7, 2.1), (-3.0, 1.5, -0.2),
                                 tuple(np.radians([-0.35, -0.07, -84.69]))])
def test_R_is_orthonormal_with_determinant_one(rpy):
    # Invariant of any proper rotation: R R^T = I, det R = +1.
    R = rpy_to_R(*rpy)
    np.testing.assert_allclose(R @ R.T, np.eye(3), rtol=0, atol=ROT_TOL)
    assert np.linalg.det(R) == pytest.approx(1.0, abs=ROT_TOL)


# ---------------------------------------------------------------- calibration

def test_calibration_closed_form(tmp_path):
    # calibrated = R @ (A_inv @ (acc - (acc_bias - dev_acc))), gyro likewise
    # without A_inv. A_inv deliberately non-symmetric so A_inv @ v and
    # v @ A_inv (= A_inv.T @ v) differ.
    A_inv = [[2.0, 0.5, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 4.0]]
    path = _write_calib(tmp_path / 'c.json', A_inv=A_inv,
                        acc_bias=[1.0, 2.0, 3.0], gyro_bias=[0.01, 0.02, 0.03],
                        device_offsets={'accel_g': [0.1, 0.0, 0.0],
                                        'gyro_deg_s': [0.0, 0.0, 1.0]})
    cal = load_calibration(path, [0.0, 0.0, 90.0])
    acc_car, gyro_car = cal.apply([10.0, 20.0, 30.0], [0.5, -0.5, 0.25])

    d = [10.0 - (1.0 - 0.1 * G), 20.0 - 2.0, 30.0 - 3.0]
    v = [2.0 * d[0] + 0.5 * d[1], 3.0 * d[1], 4.0 * d[2]]
    w = [0.5 - 0.01, -0.5 - 0.02, 0.25 - (0.03 - math.radians(1.0))]
    # yaw 90: (x, y, z) -> (-y, x, z)
    np.testing.assert_allclose(acc_car, [-v[1], v[0], v[2]], rtol=0, atol=ACC_TOL)
    np.testing.assert_allclose(gyro_car, [-w[1], w[0], w[2]], rtol=0, atol=GYRO_TOL)
    assert cal.source == str(path)


def test_device_offsets_are_not_subtracted_twice(tmp_path):
    # The VESC already removes exactly the total bias, so what it sends is
    # bias-free and the only correction left is A_inv (and nothing for gyro).
    acc_bias, gyro_bias = [0.5, -0.25, 0.125], [0.02, -0.01, 0.004]
    path = _write_calib(tmp_path / 'c.json', A_inv=[[1.1, 0, 0], [0, 0.9, 0], [0, 0, 1.0]],
                        acc_bias=acc_bias, gyro_bias=gyro_bias,
                        device_offsets={'accel_g': [b / G for b in acc_bias],
                                        'gyro_deg_s': [math.degrees(b) for b in gyro_bias]})
    cal = load_calibration(path, [0.0, 0.0, 0.0])
    acc, gyro = cal.apply([1.0, 2.0, 9.8], [0.1, 0.2, 0.3])
    np.testing.assert_allclose(acc, [1.1 * 1.0, 0.9 * 2.0, 9.8], rtol=0, atol=ACC_TOL)
    np.testing.assert_allclose(gyro, [0.1, 0.2, 0.3], rtol=0, atol=GYRO_TOL)


def test_missing_offsets_mean_the_device_removes_nothing(tmp_path):
    path = _write_calib(tmp_path / 'c.json', acc_bias=[0.5, 0.0, 0.0], gyro_bias=[0.0, 0.0, 0.01])
    acc, gyro = load_calibration(path, [0.0, 0.0, 0.0]).apply([1.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(acc, [0.5, 0.0, 0.0], rtol=0, atol=ACC_TOL)
    np.testing.assert_allclose(gyro, [0.0, 0.0, -0.01], rtol=0, atol=GYRO_TOL)


@pytest.mark.parametrize('path', [None, '', 'does/not/exist.json'])
def test_missing_file_is_identity_but_the_mount_still_applies(tmp_path, path):
    if path:
        path = tmp_path / path
    cal = load_calibration(path, [0.0, 0.0, 90.0])
    assert cal.source == 'identity (no calibration file)'
    acc, gyro = cal.apply([1.0, 2.0, 3.0], [0.1, 0.2, 0.3])
    # identity calibration, yaw 90 mount: (x, y, z) -> (-y, x, z)
    np.testing.assert_allclose(acc, [-2.0, 1.0, 3.0], rtol=0, atol=ACC_TOL)
    np.testing.assert_allclose(gyro, [-0.2, 0.1, 0.3], rtol=0, atol=GYRO_TOL)


@pytest.mark.parametrize('A_inv, match', [
    ([[1, 0], [0, 1]], '3x3'),
    ([[1, 0, 0], [0, 1, 0]], '3x3'),
    ([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]], '3x3'),
    ([1, 0, 0], '3x3'),
    ([[1, 0, 0], [0, float('nan'), 0], [0, 0, 1]], 'not finite'),
    ([[1, 0, 0], [0, 1, 0], [0, 0, float('inf')]], 'not finite'),
    ([[1, 0, 0], [0, 'a', 0], [0, 0, 1]], 'numeric'),
])
def test_bad_A_inv_is_refused(tmp_path, A_inv, match):
    # json.dumps writes NaN/Infinity, which json.loads reads back as floats.
    path = _write_calib(tmp_path / 'c.json', A_inv=A_inv)
    with pytest.raises(ValueError, match=match):
        load_calibration(path, [0.0, 0.0, 0.0])


@pytest.mark.parametrize('key', ['A_inv', 'acc_bias', 'gyro_bias'])
def test_missing_key_is_refused(tmp_path, key):
    path = _write_calib(tmp_path / 'c.json', **{key: None})
    with pytest.raises(ValueError, match=f"missing key '{key}'"):
        load_calibration(path, [0.0, 0.0, 0.0])


@pytest.mark.parametrize('field, value, match', [
    ('acc_bias', [1.0, 2.0], 'acc_bias must have 3'),
    ('gyro_bias', [0.0, float('nan'), 0.0], 'gyro_bias is not finite'),
    ('device_offsets', {'accel_g': [0.0, 0.0]}, 'accel_g must have 3'),
])
def test_bad_bias_vectors_are_refused(tmp_path, field, value, match):
    path = _write_calib(tmp_path / 'c.json', **{field: value})
    with pytest.raises(ValueError, match=match):
        load_calibration(path, [0.0, 0.0, 0.0])


def test_non_finite_mount_is_refused(tmp_path):
    with pytest.raises(ValueError, match='rpy_deg is not finite'):
        load_calibration(None, [0.0, float('nan'), 0.0])


# ---------------------------------------------------------------- car.yaml

def _car_yaml(root, body):
    (root / 'config').mkdir(parents=True)
    path = root / 'config' / 'car.yaml'
    path.write_text(body)
    return path


def test_calib_path_resolves_against_the_repo_root(tmp_path, monkeypatch):
    car = _car_yaml(tmp_path / 'repo', (
        'imus:\n'
        '  vesc:\n'
        '    rpy_deg: [-0.35, -0.07, -84.69]\n'
        '    calib: calib/vesc_calib.json\n'))
    rpy, calib = load_mount(car, 'vesc')
    assert rpy == [-0.35, -0.07, -84.69]
    assert calib == tmp_path / 'repo' / 'calib' / 'vesc_calib.json'

    # A relative car.yaml path still yields an absolute calib path.
    monkeypatch.chdir(tmp_path)
    _, calib = load_mount(Path('repo/config/car.yaml'), 'vesc')
    assert calib.is_absolute()
    assert calib == tmp_path / 'repo' / 'calib' / 'vesc_calib.json'


def test_absolute_calib_path_is_kept_and_missing_fields_default(tmp_path):
    car = _car_yaml(tmp_path / 'repo', (
        'imus:\n'
        '  bno:\n'
        '    calib: /elsewhere/bno.json\n'
        '  bare: {}\n'))
    assert load_mount(car, 'bno') == ([0.0, 0.0, 0.0], Path('/elsewhere/bno.json'))
    assert load_mount(car, 'bare') == ([0.0, 0.0, 0.0], None)


def test_unlisted_imu_is_a_key_error(tmp_path):
    car = _car_yaml(tmp_path / 'repo', 'imus:\n  vesc:\n    rpy_deg: [0, 0, 0]\n')
    with pytest.raises(KeyError, match="'bno' is not listed"):
        load_mount(car, 'bno')


# ---------------------------------------------------------------- startup bias

def _feed_still(bias, t_values, gyro):
    for t in t_values:
        bias.update(t, gyro, 0.0)


def test_startup_bias_is_the_mean_of_the_still_window():
    bias = StartupBias(seconds=1.0, still_speed=0.03, still_gyro=0.05)
    samples = [(0.0, [0.01, -0.02, 0.003]), (0.25, [0.02, -0.01, 0.001]),
               (0.5, [0.0, -0.03, 0.002]), (0.75, [0.01, -0.02, 0.0]),
               (1.0, [0.03, -0.02, 0.004])]
    for t, g in samples:
        bias.update(t, g, 0.0)
    assert bias.done
    expected = [sum(g[i] for _, g in samples) / len(samples) for i in range(3)]
    np.testing.assert_allclose(bias.residual(), expected, rtol=0, atol=GYRO_TOL)


def test_completes_exactly_at_the_boundary_and_not_before():
    # Times are exact binary fractions, so t - t0 hits 3.0 exactly.
    bias = StartupBias(seconds=3.0)
    for t in np.arange(0.0, 3.0, 0.5):   # 0.0 .. 2.5
        assert bias.update(t, [0.01, 0.0, 0.0], 0.0) is False
        np.testing.assert_array_equal(bias.residual(), [0.0, 0.0, 0.0])
    assert bias.update(3.0, [0.01, 0.0, 0.0], 0.0) is True
    np.testing.assert_allclose(bias.residual(), [0.01, 0.0, 0.0], rtol=0, atol=GYRO_TOL)


def test_residual_is_frozen_once_done():
    bias = StartupBias(seconds=0.5)
    _feed_still(bias, [0.0, 0.5], [0.01, 0.0, 0.0])
    bias.update(1.0, [0.04, 0.04, 0.04], 0.0)
    bias.update(1.5, [0.0, 0.0, 0.0], 5.0)
    np.testing.assert_allclose(bias.residual(), [0.01, 0.0, 0.0], rtol=0, atol=GYRO_TOL)


def test_just_inside_both_thresholds_counts_as_still():
    # The permissive side: below still_speed and still_gyro on every axis.
    bias = StartupBias(seconds=0.5, still_speed=0.03, still_gyro=0.05)
    g = [0.0499, -0.0499, 0.0499]
    bias.update(0.0, g, -0.0299)
    assert bias.update(0.5, g, 0.0299) is True
    np.testing.assert_allclose(bias.residual(), g, rtol=0, atol=GYRO_TOL)


MOVING = [
    pytest.param(0.03, [0.0, 0.0, 0.0], id='speed-exactly-at-threshold'),
    pytest.param(-0.03, [0.0, 0.0, 0.0], id='reverse-speed-at-threshold'),
    pytest.param(0.5, [0.0, 0.0, 0.0], id='driving'),
    pytest.param(-0.5, [0.0, 0.0, 0.0], id='reversing'),
    pytest.param(None, [0.0, 0.0, 0.0], id='speed-unknown'),
    pytest.param(float('nan'), [0.0, 0.0, 0.0], id='speed-nan'),
    pytest.param(float('inf'), [0.0, 0.0, 0.0], id='speed-inf'),
    pytest.param(0.0, [0.05, 0.0, 0.0], id='gyro-x-exactly-at-threshold'),
    pytest.param(0.0, [0.0, -0.05, 0.0], id='gyro-y-negative-at-threshold'),
    pytest.param(0.0, [0.0, 0.0, 0.05], id='gyro-z-exactly-at-threshold'),
    pytest.param(0.0, [0.0, 0.0, -1.0], id='turning'),
    pytest.param(0.0, [float('nan'), 0.0, 0.0], id='gyro-nan'),
    pytest.param(0.0, [0.0, float('inf'), 0.0], id='gyro-inf'),
]


@pytest.mark.parametrize('speed, gyro', MOVING)
def test_motion_restarts_the_window(speed, gyro):
    # 2 s of still at gyro 0.04, one moving sample, then still at 0.01.
    # If the moving sample did not restart the window it would complete at
    # t=3.0 with the 0.04 samples mixed in; restarted, it completes at
    # t=2.5+3.0=5.5 and averages only the 0.01 samples.
    bias = StartupBias(seconds=3.0, still_speed=0.03, still_gyro=0.05)
    _feed_still(bias, [0.0, 0.5, 1.0, 1.5, 2.0], [0.04, 0.0, 0.0])
    assert bias.update(2.25, gyro, speed) is False
    for t in [2.5, 3.0, 3.5, 4.0, 4.5, 5.0]:
        assert bias.update(t, [0.01, 0.0, 0.0], 0.0) is False
    np.testing.assert_array_equal(bias.residual(), [0.0, 0.0, 0.0])
    assert bias.update(5.5, [0.01, 0.0, 0.0], 0.0) is True
    np.testing.assert_allclose(bias.residual(), [0.01, 0.0, 0.0], rtol=0, atol=GYRO_TOL)


def test_seconds_zero_is_off():
    bias = StartupBias(seconds=0.0)
    assert bias.done
    assert bias.update(0.0, [0.01, 0.02, 0.03], 0.0) is True
    np.testing.assert_array_equal(bias.residual(), [0.0, 0.0, 0.0])


@pytest.mark.parametrize('seconds', [-0.001, -3.0, float('nan')])
def test_negative_seconds_is_refused(seconds):
    with pytest.raises(ValueError, match='seconds must be >= 0'):
        StartupBias(seconds=seconds)


# ---------------------------------------------------------------- covariances

def test_covariance_diag_is_row_major_sigma_squared_on_the_diagonal():
    sigma = 0.0017
    cov = covariance_diag(sigma)
    assert len(cov) == 9
    for row in range(3):
        for col in range(3):
            want = sigma ** 2 if row == col else 0.0
            assert cov[row * 3 + col] == pytest.approx(want, rel=0, abs=1e-15)  # (rad/s)^2


def test_wheel_twist_covariance_trusts_only_vx():
    # nav_msgs/Odometry twist covariance: 6x6 row-major over
    # (vx, vy, vz, wx, wy, wz); the diagonal is index row*6 + row.
    sigma = 0.08
    cov = wheel_twist_covariance(sigma)
    assert len(cov) == 36
    assert cov[0] == pytest.approx(sigma ** 2, rel=0, abs=1e-15)  # (m/s)^2
    for row in range(1, 6):
        assert cov[row * 6 + row] == 1e3
    off_diagonal = [cov[r * 6 + c] for r in range(6) for c in range(6) if r != c]
    assert off_diagonal == [0.0] * 30


@pytest.mark.parametrize('fn', [covariance_diag, wheel_twist_covariance])
@pytest.mark.parametrize('sigma', [0.0, -0.01, float('nan'), float('inf')])
def test_covariance_helpers_refuse_a_non_positive_sigma(fn, sigma):
    with pytest.raises(ValueError, match='must be a finite number > 0'):
        fn(sigma)


def test_orientation_unknown_follows_sensor_msgs_imu():
    # sensor_msgs/Imu.msg: no estimate -> element 0 of the covariance is -1.
    assert len(convert.ORIENTATION_UNKNOWN) == 9
    assert convert.ORIENTATION_UNKNOWN[0] == -1.0
    assert list(convert.ORIENTATION_UNKNOWN[1:]) == [0.0] * 8


# ---------------------------------------------------------------- time

def test_monotonic_sample_keeps_its_age_on_the_ros_clock():
    ros_now = 1_700_000_000_123_456_789
    # 0.25 s before now on the monotonic clock is 0.25 s before now on ROS.
    assert monotonic_to_ros_ns(100.0, 100.25, ros_now) == ros_now - 250_000_000
    assert monotonic_to_ros_ns(100.5, 100.25, ros_now) == ros_now + 250_000_000
    assert monotonic_to_ros_ns(42.0, 42.0, ros_now) == ros_now
    assert isinstance(monotonic_to_ros_ns(1.0, 2.0, ros_now), int)


# ---------------------------------------------------------------- primary

@pytest.mark.parametrize('primary, port, want', [
    ('auto', '', 'vesc'),
    ('auto', '/dev/ttyACM1', 'bno'),
    ('vesc', '/dev/ttyACM1', 'vesc'),
    ('bno', '/dev/ttyACM1', 'bno'),
])
def test_primary_resolution(primary, port, want):
    assert resolve_primary(primary, port) == want


def test_primary_bno_without_a_port_is_refused():
    with pytest.raises(ValueError, match="'bno' but bno_port is empty"):
        resolve_primary('bno', '')


def test_unknown_primary_is_refused():
    with pytest.raises(ValueError, match='primary must be one of'):
        resolve_primary('both', '')


# ---------------------------------------------------------------- racer_imu

RACER_IMU_ROOT = Path(os.environ.get('RACER_IMU_ROOT', '~/imu-calibration')).expanduser()


@pytest.mark.skipif(
    not (RACER_IMU_ROOT / 'racer_imu' / 'calibration.py').is_file(),
    reason=f'imu-calibration repo not found at {RACER_IMU_ROOT} '
           '(clone github.com/sfu-racerbot/imu-calibration there, or set RACER_IMU_ROOT); '
           'runs whenever it is present')
@pytest.mark.parametrize('with_device_offsets', [False, True])
def test_matches_racer_imu_on_its_real_calibration_file(tmp_path, with_device_offsets):
    """Our port vs. racer_imu's own ImuCalibration.apply + mount rotation.

    Oracle: racer_imu itself, on the car's real car.yaml and calib file. The
    real vesc_calib.json carries no device_offsets, so the second case adds
    some to a copy, exercising the subtraction neither path would otherwise hit.
    """
    sys.path.insert(0, str(RACER_IMU_ROOT))
    try:
        from racer_imu.calibration import ImuCalibration
        from racer_imu.frames import load_config
        from racer_imu.types import G as racer_G, ImuSample
    finally:
        sys.path.remove(str(RACER_IMU_ROOT))
    assert racer_G == G

    car_yaml = RACER_IMU_ROOT / 'config' / 'car.yaml'
    config = load_config(car_yaml)
    ext = config.imus['vesc']
    calib_path = config.resolve(ext.extra['calib'])
    rpy_deg, our_path = load_mount(car_yaml, 'vesc')
    assert our_path == calib_path.resolve()
    assert calib_path.is_file(), f'{calib_path} missing: nothing real to compare against'

    if with_device_offsets:
        data = json.loads(calib_path.read_text())
        data['device_offsets'] = {'accel_g': [0.01, -0.02, 0.03], 'gyro_deg_s': [0.4, -0.1, 0.2]}
        our_path = calib_path = _write_calib(tmp_path / 'vesc_calib.json', **data)

    theirs = ImuCalibration.load(calib_path)
    ours = load_calibration(our_path, rpy_deg)
    assert ours.source == str(our_path)

    raw = [([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]),
           ([0.12, -0.31, 0.97], [12.5, -3.0, 45.0]),
           ([-1.5, 0.8, 0.2], [-200.0, 90.0, -400.0])]
    for acc_g, gyro_dps in raw:
        acc, gyro = vesc_raw_to_si(acc_g, gyro_dps)
        s = theirs.apply(ImuSample(0.0, acc, gyro))
        acc_car, gyro_car = ours.apply(acc, gyro)
        np.testing.assert_allclose(acc_car, ext.R @ s.acc, rtol=0, atol=1e-12)    # m/s^2
        np.testing.assert_allclose(gyro_car, ext.R @ s.gyro, rtol=0, atol=1e-12)  # rad/s
