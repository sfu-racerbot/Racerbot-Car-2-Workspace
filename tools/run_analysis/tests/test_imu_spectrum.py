"""imu_spectrum.py on synthetic runs whose answer is built in.

Arrays in load_imu_axes' layout, no bag, no ROS:

    python3 -m pytest tools/run_analysis/tests -q

Oracles: closed forms (a known bias and noise, a tone folded by sampling,
a first-order lag plus delay, a rotation by the mount angle) and the
statistics of the very samples injected. Car 2's numbers where they matter:
the VESC IMU at 50 Hz (vesc_driver.cpp's 20 ms poll), the 0.36 m wheelbase
read from vesc.yaml.
"""
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from run_analysis import imu_spectrum as ims  # noqa: E402
from run_analysis.scan_motion import Motion  # noqa: E402

G = 9.80665
FS = 50.0
RACER_IMU_ROOT = Path(os.environ.get('RACER_IMU_ROOT', '~/imu-calibration')).expanduser()


def _imu_rows(t, acc, gyro):
    t = np.asarray(t, float)
    return np.column_stack([t, t, acc, gyro])


def _axes(**over):
    base = {k: np.zeros((0, 8)) for k in ('vesc', 'bno', 'bno_raw', 'vesc_raw', 'core')}
    base.update(odom=np.zeros((0, 7)), ackermann=np.zeros((0, 4)), drive=np.zeros((0, 4)))
    base.update(over)
    return base


def _odom(t, v):
    t = np.asarray(t, float)
    v = np.broadcast_to(np.asarray(v, float), t.shape)
    return np.column_stack([t, t, v, t * 0, t * 0, t * 0, t * 0])


# ---------------------------------------------------------------- config references

def test_config_ref_points_at_the_line_that_holds_the_value():
    # oracle: invariant -- the file:line given must hold the key, and the
    # value is that line's.
    value, where = ims.config_ref('gyro_sigma')
    path, line = where.rsplit(':', 1)
    text = (ims.ROOT / path).read_text().splitlines()[int(line) - 1]
    assert 'gyro_sigma:' in text and float(text.split(':')[1].split('#')[0]) == value
    assert ims.config_ref('wheelbase')[0] == 0.36    # car 2, tape-measured (CLAUDE.md)


def test_config_ref_reads_a_dataclass_field_and_a_yaml_key_alike():
    # oracle: invariant -- params.py writes `imu_accel_noise: float = 0.15`;
    # the value is the number after '=', read from the line given.
    value, where = ims.config_ref('imu_accel_noise')
    path, line = where.rsplit(':', 1)
    text = (ims.ROOT / path).read_text().splitlines()[int(line) - 1]
    assert isinstance(value, float)
    assert value == float(text.split('=')[1].split('#')[0])


def test_config_ref_reports_what_is_missing(tmp_path):
    value, why = ims.config_ref('gyro_sigma', root=tmp_path)
    assert value is None and 'not found' in why


# ---------------------------------------------------------------- 1. bias and noise

def test_parked_bias_and_noise_come_from_the_still_samples_only():
    # oracle: closed form on the injected samples -- 60 s parked then 10 s
    # driving with a different gyro offset. The bias is the mean of the
    # parked samples exactly, the noise their np.std exactly; the driving
    # samples must not enter.
    rng = np.random.default_rng(10)
    t = np.arange(0, 70, 1 / FS)
    still = t < 60
    gyro = np.array([0.002, -0.001, 0.003]) + rng.normal(0, 0.0017, (len(t), 3))
    acc = np.array([0.05, -0.02, G]) + rng.normal(0, 0.03, (len(t), 3))
    gyro[~still] += 0.02                     # moving, but under the stillness gyro limit
    axes = _axes(vesc=_imu_rows(t, acc, gyro), odom=_odom(t, np.where(still, 0.0, 1.0)))
    stream = ims.vesc_stream(axes)
    segs, stats = ims.bias_noise(stream, axes)
    assert len(segs) == 1
    for j, a in enumerate('xyz'):
        dc, sigma, _ = stats[('gyro', a)]
        assert dc == pytest.approx(np.mean(gyro[still, j]), abs=1e-12)     # rad/s
        assert sigma == pytest.approx(np.std(gyro[still, j]), abs=1e-12)   # rad/s
        dc, sigma, _ = stats[('accel', a)]
        assert dc == pytest.approx(np.mean(acc[still, j]), abs=1e-12)      # m/s^2
        assert sigma == pytest.approx(np.std(acc[still, j]), abs=1e-12)
    # and the injected truth, within the sampling error of 3000 samples
    assert stats[('gyro', 'z')][0] == pytest.approx(0.003, abs=1e-4)
    text = '\n'.join(ims.bias_noise_lines(stream, axes))
    assert 'gyro_sigma 0.0017 at src/imu_bridge/config/imu_bridge.yaml:' in text


def test_no_still_stretch_and_no_odom_say_so():
    t = np.arange(0, 20, 1 / FS)
    z3 = np.zeros((len(t), 3))
    moving = _axes(vesc=_imu_rows(t, z3, z3), odom=_odom(t, 1.0))
    assert 'no still stretch' in '\n'.join(ims.bias_noise_lines(ims.vesc_stream(moving), moving))
    no_odom = _axes(vesc=_imu_rows(t, z3, z3))
    assert 'no /odom' in '\n'.join(ims.bias_noise_lines(ims.vesc_stream(no_odom), no_odom))


# ---------------------------------------------------------------- the VESC stream

def _raw(t, acc_g, gyro_dps):
    return _imu_rows(t, np.broadcast_to(acc_g, (len(t), 3)), np.broadcast_to(gyro_dps, (len(t), 3)))


def _metadata(run_dir, calib=None):
    files = run_dir / 'metadata' / 'files'
    files.mkdir(parents=True)
    (files / 'car.yaml').write_text(
        "imus:\n  vesc:\n    rpy_deg: [0.0, 0.0, -90.0]\n    calib: calib/vesc_calib.json\n")
    if calib is not None:
        (files / 'vesc_calib.json').write_text(json.dumps(calib))


def test_raw_fallback_is_rotated_with_the_runs_car_yaml(tmp_path):
    # oracle: closed form -- yaw -90 deg maps sensor x onto car -y, so a raw
    # 0.1 g on x reads -0.1 * 9.80665 on the car's y; 10 deg/s about z stays
    # 10 deg/s about z.
    _metadata(tmp_path)
    t = np.arange(0, 2, 1 / FS)
    axes = _axes(vesc_raw=_raw(t, [0.1, 0.0, 1.0], [0.0, 0.0, 10.0]))
    s = ims.vesc_stream(axes, tmp_path)
    assert s.axes_known and "rotated with the run's car.yaml" in s.label and 'uncalibrated' in s.label
    assert s.acc[0] == pytest.approx([0.0, -0.1 * G, G], abs=1e-9)               # m/s^2
    assert s.gyro[0] == pytest.approx([0.0, 0.0, math.radians(10.0)], abs=1e-12)  # rad/s


def test_raw_fallback_uses_the_runs_calibration_copy(tmp_path):
    # oracle: closed form -- a calibration whose accel bias is exactly the
    # 0.1 g on x removes it: the car's y reads 0.
    _metadata(tmp_path, calib={'A_inv': np.eye(3).tolist(), 'acc_bias': [0.1 * G, 0.0, 0.0],
                               'gyro_bias': [0.0, 0.0, 0.0]})
    t = np.arange(0, 2, 1 / FS)
    s = ims.vesc_stream(_axes(vesc_raw=_raw(t, [0.1, 0.0, 1.0], [0.0, 0.0, 0.0])), tmp_path)
    assert "calibrated with the run's vesc_calib.json" in s.label
    assert s.acc[0] == pytest.approx([0.0, 0.0, G], abs=1e-9)


def test_raw_fallback_without_car_yaml_reports_gyro_z_only(tmp_path):
    t = np.arange(0, 70, 1 / FS)
    axes = _axes(vesc_raw=_raw(t, [0.0, 1.0, 0.0], [0.0, 0.0, 0.1]), odom=_odom(t, 0.0))
    s = ims.vesc_stream(axes, tmp_path)
    assert not s.axes_known and 'gyro z only' in s.label
    segs, stats = ims.bias_noise(s, axes)
    assert set(stats) == {('gyro', 'z')}
    assert 'axes unknown' in '\n'.join(ims.bias_noise_lines(s, axes))


def test_calibrated_topic_wins_over_raw(tmp_path):
    _metadata(tmp_path)
    t = np.arange(0, 2, 1 / FS)
    z3 = np.zeros((len(t), 3))
    s = ims.vesc_stream(_axes(vesc=_imu_rows(t, z3, z3), vesc_raw=_raw(t, [1, 0, 0], [0, 0, 0])),
                        tmp_path)
    assert s.label.startswith('/imu/vesc/data')


def test_no_imu_and_no_bno_are_reported_not_crashed_on():
    assert ims.build_report(None, _axes())[0][-1].startswith('no IMU in this run')
    t = np.arange(0, 5, 1 / FS)
    z3 = np.zeros((len(t), 3))
    lines, _, _ = ims.build_report(None, _axes(vesc=_imu_rows(t, z3, z3)))
    text = '\n'.join(lines)
    assert 'BNO085: not in this run' in text
    assert '/imu/bno/data      not in this run' in text


# ---------------------------------------------------------------- 2. vibration

def _lifted(refs_hz, seconds=8.0):
    """Wheels lifted: the motor held at each electrical frequency in turn
    for `seconds`; the VESC accel x carries a 0.5 m/s^2 tone at exactly that
    frequency, sampled at 50 Hz with no anti-alias filter."""
    t = np.arange(0, seconds * len(refs_hz), 1 / FS)
    ref = np.asarray(refs_hz)[np.minimum((t // seconds).astype(int), len(refs_hz) - 1)]
    phase = 2 * math.pi * np.cumsum(ref) / FS
    acc = np.column_stack([0.5 * np.sin(phase), t * 0, np.full_like(t, G)])
    core = np.column_stack([t, t, ref * 60.0, t * 0, t * 0, t * 0, t * 0, t * 0])
    return _axes(vesc=_imu_rows(t, acc, np.zeros((len(t), 3))), core=core, odom=_odom(t, 0.0))


def test_motor_tones_above_nyquist_are_flagged_as_aliases_moving_backwards():
    # oracle: closed form -- 37/41/44 Hz sampled at 50 Hz land at 13/9/6 Hz:
    # each is harmonic 1 folded, and the peak moves DOWN as the motor speeds up.
    axes = _lifted([37.0, 41.0, 44.0])
    lines, _ = ims.vibration_lines(ims.vesc_stream(axes), axes)
    text = '\n'.join(lines)
    assert len(ims.steady_bins(axes)) == 3
    assert text.count('aliased (x1)') == 3
    assert 'accel x  backwards' in text
    assert 'folding into the data' in text


def test_one_aliased_speed_alone_is_evidence_of_folding():
    # oracle: closed form -- 37 Hz at 50 Hz folds to 13 Hz; one speed has no
    # trend, so the alias itself must carry the verdict.
    axes = _lifted([37.0])
    text = '\n'.join(ims.vibration_lines(ims.vesc_stream(axes), axes)[0])
    assert 'aliased (x1)' in text and 'too few speeds' in text
    assert 'folding into the data' in text


def test_in_band_motor_tones_scale_and_raise_no_alias():
    # oracle: closed form -- 10/14/18 Hz are below the 25 Hz Nyquist: in band,
    # moving up with the motor.
    axes = _lifted([10.0, 14.0, 18.0])
    text = '\n'.join(ims.vibration_lines(ims.vesc_stream(axes), axes)[0])
    assert text.count('in_band (x1)') == 3
    assert 'motor 840 ERPM (14.0 Hz electrical)' in text    # the bin holds one speed only
    assert 'accel x  scales' in text
    assert 'no sign of aliasing' in text
    assert 'settings not given' in text


def test_no_steady_motor_speed_says_record_run_b():
    t = np.arange(0, 10, 1 / FS)
    z3 = np.zeros((len(t), 3))
    axes = _axes(vesc=_imu_rows(t, z3, z3))
    assert 'record run B' in '\n'.join(ims.vibration_lines(ims.vesc_stream(axes), axes)[0])


@pytest.mark.skipif(not (RACER_IMU_ROOT / 'tools' / 'imu_filter_plot.py').is_file(),
                    reason=f'imu-calibration not at {RACER_IMU_ROOT} (set RACER_IMU_ROOT); '
                           'the VESC filter formula is imported from it, not copied')
def test_vesc_filter_gain_is_the_firmware_butterworth():
    # oracle: closed form -- a bilinear 2nd-order Butterworth (Q 0.707, the
    # firmware's) is 1/sqrt(2) at its cutoff and passes DC unchanged.
    assert ims.vesc_filter_gain(30.0, 200.0, 30.0, RACER_IMU_ROOT) == pytest.approx(
        1 / math.sqrt(2), abs=2e-3)
    assert ims.vesc_filter_gain(30.0, 200.0, 0.01, RACER_IMU_ROOT) == pytest.approx(1.0, abs=1e-6)
    assert ims.vesc_filter_gain(0.0, 200.0, 25.0, RACER_IMU_ROOT) == 1.0       # off


def test_vesc_filter_gain_without_imu_calibration_is_none(tmp_path):
    assert ims.vesc_filter_gain(30.0, 200.0, 25.0, tmp_path) is None


# ---------------------------------------------------------------- 3. delay and scale

def _yaw(t):
    return 0.8 * np.sin(1.3 * t) + 0.4 * np.sin(3.1 * t + 0.4)


def test_delay_and_scale_recovered_and_sign_follows_the_argument_order():
    # oracle: closed form -- the gyro is the LiDAR's yaw rate 3 ticks
    # (75 ms) later and 1.07 times larger, plus noise (plan P0.5's test).
    rng = np.random.default_rng(11)
    t_ref = np.arange(0, 60, 0.05)                                   # LiDAR, 20 Hz
    t_imu = np.arange(0, 60, 1 / FS)
    w_imu = 1.07 * _yaw(t_imu - 0.075) + rng.normal(0, 0.01, len(t_imu))
    delay, scale, _ = ims.imu_delay_and_scale(t_imu, w_imu, t_ref, _yaw(t_ref))
    assert delay == pytest.approx(0.075, abs=0.005)                 # s
    assert scale == pytest.approx(1.07, abs=0.01)
    back, _, _ = ims.imu_delay_and_scale(t_ref, _yaw(t_ref), t_imu, w_imu)
    assert back == pytest.approx(-0.075, abs=0.005)


def test_delay_section_needs_lidar_and_driving():
    t = np.arange(0, 20, 1 / FS)
    z3 = np.zeros((len(t), 3))
    axes = _axes(vesc=_imu_rows(t, z3, z3), odom=_odom(t, 0.0))
    s = ims.vesc_stream(axes)
    assert 'no LiDAR motion' in '\n'.join(ims.delay_lines(s, axes, None))
    m = Motion(t, t * 0, t * 0, t * 0, t * 0)
    assert 'drive (run C)' in '\n'.join(ims.delay_lines(s, axes, m))


# ---------------------------------------------------------------- 4. steering response

def _weave(K=0.95, T=0.08, tau=0.15, seconds=120.0, v=1.2, commands='ackermann'):
    """Commanded steering: a sum of sines 0.1-3 Hz. The car's yaw rate is
    K e^{-s tau} / (1 + T s) applied to v tan(delta) / L, integrated exactly
    (zero-order hold) at 1 ms, then sampled at 50 Hz."""
    L = ims.config_ref('wheelbase')[0]
    rng = np.random.default_rng(12)
    freqs = np.linspace(0.1, 3.0, 15)
    phases = rng.uniform(0, 2 * math.pi, len(freqs))

    def delta(x):
        return 0.02 * np.sum(np.sin(2 * math.pi * np.outer(x, freqs) + phases), axis=1)

    dt = 0.001
    tf = np.arange(0, seconds, dt)
    u = v * np.tan(delta(tf - tau)) / L
    a = math.exp(-dt / T)
    y = np.zeros_like(tf)
    for n in range(len(tf) - 1):
        y[n + 1] = a * y[n] + (1 - a) * K * u[n]
    t = np.arange(0, seconds, 1 / FS)
    w = np.interp(t, tf, y)
    cmd = np.column_stack([t, t, delta(t), np.full_like(t, v)])
    z3 = np.zeros((len(t), 3))
    gyro = np.column_stack([z3[:, :2], w])
    return _axes(vesc=_imu_rows(t, z3, gyro), odom=_odom(t, v), **{commands: cmd})


def test_steering_response_recovers_a_first_order_lag_plus_delay():
    # oracle: closed form -- the plant built into _weave: K 0.95 (car 2's
    # measured curvature gain), T 0.08 s, delay 0.15 s (its measured lag).
    # Tolerances cover the 50 Hz sampling of the command and the 1 ms
    # zero-order hold (half a millisecond of extra delay).
    axes = _weave()
    s = ims.vesc_stream(axes)
    fr, topic, lag, n = ims.steering_response(s.t, s.gyro[:, 2], axes, 0.36, FS)
    assert topic == '/ackermann_cmd' and n == 1
    fit = ims.sp.fit_first_order_delay(fr, f_max=3.0)
    assert fit['gain'] == pytest.approx(0.95, abs=0.01)
    assert fit['time_constant'] == pytest.approx(0.08, abs=0.01)     # s
    assert fit['delay'] == pytest.approx(0.15, abs=0.01)              # s
    text = '\n'.join(ims.steering_lines(s, axes, None)[0])
    assert 'plant: yaw_rate = 0.9' in text and 'documented 0.15 s' in text


def test_crawling_below_moving_speed_is_left_out_of_the_steering_fit():
    # oracle: the plant built into _weave, again -- 120 s of weaves at
    # 1.2 m/s, then 120 s crawling at 0.3 m/s (below MOVING_SPEED 0.5) in
    # which the car does not follow the stick at all (gyro: noise only).
    # Only the moving stretch may enter: the gain stays 0.95.
    fast = _weave()
    slow = _weave(v=0.3)
    t_slow = slow['vesc'][:, 0] + 120.0
    rng = np.random.default_rng(13)
    slow['vesc'][:, 7] = rng.normal(0, 0.05, len(t_slow))
    axes = {}
    for k in fast:
        b = slow[k].copy()
        if len(b):
            b[:, 0] += 120.0
            b[:, 1] += 120.0
        axes[k] = np.vstack([fast[k], b])
    s = ims.vesc_stream(axes)
    fr, topic, lag, n = ims.steering_response(s.t, s.gyro[:, 2], axes, 0.36, FS)
    assert n == 1
    assert ims.sp.fit_first_order_delay(fr, f_max=3.0)['gain'] == pytest.approx(0.95, abs=0.01)


def test_steering_prefers_drive_and_says_no_estimate_when_parked():
    t = np.arange(0, 2, 1 / FS)
    cmd_a = np.column_stack([t, t, t * 0 + 0.1, t * 0 + 1])
    cmd_d = np.column_stack([t, t, t * 0 + 0.2, t * 0 + 1])
    axes = _axes(odom=_odom(t, 1.0), ackermann=cmd_a, drive=cmd_d)
    rate, topic = ims.commanded_yaw_rate(t, axes, 0.36)
    assert topic == '/drive'
    assert rate[0] == pytest.approx(1.0 * math.tan(0.2) / 0.36, abs=1e-12)    # rad/s
    assert ims.commanded_yaw_rate(t, _axes(odom=_odom(t, 1.0)), 0.36) == (None, None)
    parked = _weave(v=0.0)
    s = ims.vesc_stream(parked)
    assert 'no estimate' in '\n'.join(ims.steering_lines(s, parked, None)[0])
