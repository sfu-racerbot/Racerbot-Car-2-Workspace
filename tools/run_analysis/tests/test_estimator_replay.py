"""estimator_replay.py: vehicle_state replayed on a synthetic run, scored
against a LiDAR whose motion is the truth. No ROS:

    python3 -m pytest tools/run_analysis/tests -q

Oracle: invariants on a run with the answer built in. Speed 1.0 +
0.5 sin(0.5 t) m/s, yaw rate 0.6 sin(0.3 t) rad/s; both IMUs read the true
forward acceleration at their own mount (dv/dt - yaw^2 d, as
vehicle_state.accel_at_base_link undoes) with 0.3 m/s^2 noise, and the true
yaw rate; the wheels read the speed with 0.08 m/s noise (sigma_wheel); and
/odom's heading turns 5% too far, like the steering model car 2 measured
(curvature gain 0.95, docs/run-analysis.md). The LiDAR reports the truth.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from run_analysis import estimator_replay as er  # noqa: E402
from run_analysis.scan_motion import Motion  # noqa: E402

G = 9.80665


def _synthetic(seconds=60.0, bno=True, seed=0):
    rng = np.random.default_rng(seed)

    def v(t):
        return 1.0 + 0.5 * np.sin(0.5 * t)

    def dv(t):
        return 0.25 * np.cos(0.5 * t)

    def r(t):
        return 0.6 * np.sin(0.3 * t)

    def imu(t, d):
        ax = dv(t) - r(t) ** 2 * d + rng.normal(0, 0.3, len(t))
        z = np.zeros_like(t)
        return np.column_stack([t, t, ax, z, np.full_like(t, G), z, z, r(t)])

    t50 = np.arange(0.0, seconds, 0.02)
    yaw = 1.05 * 0.6 / 0.3 * (1 - np.cos(0.3 * t50))     # /odom's heading: 5% too much turn
    vx = v(t50)
    x = np.concatenate([[0.0], np.cumsum(vx[:-1] * np.cos(yaw[:-1]) * 0.02)])
    y = np.concatenate([[0.0], np.cumsum(vx[:-1] * np.sin(yaw[:-1]) * 0.02)])
    odom = np.column_stack([t50, t50, vx + rng.normal(0, 0.08, len(t50)), 1.05 * r(t50), x, y, yaw])
    axes = {'vesc': imu(t50, 0.18), 'odom': odom,
            'bno': imu(np.arange(0.0, seconds, 0.004), 0.36) if bno else np.zeros((0, 8))}
    tl = np.arange(0.0, seconds, 0.05)
    motion = Motion(tl, v(tl), tl * 0, r(tl), tl * 0)
    return axes, motion


def _score(axes, motion, overrides=None):
    streams, _ = er.streams_from_axes(axes)
    res = er.replay(streams, axes['odom'], er.build_config(overrides or {}))
    return res, er.score(res, motion)


def test_replay_reproduces_the_known_speed_and_heading():
    axes, motion = _synthetic()
    res, s = _score(axes, motion)
    assert s['n_speed'] > 1000 and s['n_windows'] >= 25
    # Fusion exists to beat the wheels: half their error, as the
    # estimator's own test asks of it (test_fused_speed_beats_noisy_wheels).
    assert s['speed_fused'] < 0.5 * s['speed_wheel']
    assert s['speed_wheel'] == pytest.approx(0.08, rel=0.1)          # m/s: the injected noise
    # The gyro is the truth, so its heading change matches the LiDAR's;
    # /odom's carries the 5% over-read.
    assert s['heading_gyro'] < 0.005                                   # rad per 2 s window
    assert s['heading_odom'] > 10 * s['heading_gyro']
    assert res.heading_sources['bno'] > 0.9 * sum(res.heading_sources.values())


def test_a_deliberately_bad_override_scores_worse():
    # heading_sources=none makes /odom/imu exactly /odom (vehicle_state's
    # documented fallback), so its heading error becomes /odom's.
    axes, motion = _synthetic()
    _, good = _score(axes, motion)
    res, bad = _score(axes, motion, {'heading_sources': 'none'})
    assert set(res.heading_sources) == {'model'}
    assert bad['heading_gyro'] == pytest.approx(bad['heading_odom'], abs=1e-9)
    assert bad['heading_gyro'] > 10 * good['heading_gyro']
    # A slip gate this tight throws good wheel readings away as slip.
    _, tight = _score(axes, motion, {'slip_gate': '0.2'})
    assert tight['speed_fused'] > good['speed_fused']


def test_both_imus_fused_on_this_branch():
    # accel_source=both (DualImuSpeedFilter): both accelerometers measured.
    axes, motion = _synthetic()
    res, s = _score(axes, motion, {'accel_source': 'both'})
    assert res.modes['fused'] + res.modes['clamped'] > 0.9 * sum(res.modes.values())
    assert s['speed_fused'] < 0.5 * s['speed_wheel']


def test_vesc_only_run_falls_back_as_the_node_does():
    # No BNO: with accel_source bno (the default) /odom/fused is the wheel
    # speed exactly, and heading comes from the VESC gyro.
    axes, motion = _synthetic(bno=False)
    streams, notes = er.streams_from_axes(axes)
    assert 'bno: not in this run' in notes and set(streams) == {'vesc'}
    res, s = _score(axes, motion)
    assert set(res.modes) == {'wheel_only'}
    assert np.array_equal(res.fused, res.wheel)
    assert s['speed_fused'] == s['speed_wheel']
    assert res.heading_sources['vesc'] > 0.9 * sum(res.heading_sources.values())


def test_streams_carry_forward_accel_vertical_accel_and_yaw_rate():
    # oracle: spec -- VehicleStateEstimator.on_imu(accel_x, accel_z,
    # yaw_rate): car-axis x (forward), z (gravity check) and gyro z. Every
    # column distinct, so a swapped one shows.
    t = np.arange(0.0, 1.0, 0.02)
    rows = np.column_stack([t, t + 0.001, t * 0 + 1.0, t * 0 + 2.0, t * 0 + 3.0,
                            t * 0 + 4.0, t * 0 + 5.0, t * 0 + 6.0])
    streams, _ = er.streams_from_axes({'vesc': rows, 'bno': np.zeros((0, 8))})
    stamp, ax, az, gz = streams['vesc']
    assert np.array_equal(stamp, t + 0.001)
    assert (ax[0], az[0], gz[0]) == (1.0, 3.0, 6.0)


def test_a_raw_vesc_without_car_yaml_is_left_out_not_fed_unknown_axes(tmp_path):
    t = np.arange(0.0, 1.0, 0.02)
    raw = np.column_stack([t, t, t * 0, t * 0, t * 0 + 1.0, t * 0, t * 0, t * 0])
    streams, notes = er.streams_from_axes({'vesc_raw': raw}, tmp_path)
    assert streams == {}
    assert any(n.startswith('vesc: left out') for n in notes)


def test_build_config_takes_the_yaml_names():
    cfg = er.build_config({'bno_x_offset': '0.30', 'heading_sources': 'vesc,bno',
                           'accel_source': 'vesc', 'sigma_wheel': '0.1'})
    assert dict(cfg.x_offset) == {'bno': 0.30, 'vesc': 0.18}
    assert cfg.heading_sources == ('vesc', 'bno')
    assert cfg.accel_source == 'vesc' and cfg.sigma_wheel == 0.1


def test_every_estimator_parameter_is_settable_by_its_yaml_name():
    # oracle: invariant -- each name build_config accepts round-trips its
    # current value; a field added to EstimatorConfig without a mapping
    # here would fail to round-trip.
    base = er.EstimatorConfig()
    for name in er.settable_names():
        table = next((t for t, flat in er.TABLE_PARAMS.items() if name in flat), None)
        if table:
            value = str(dict(getattr(base, table))[name.split('_', 1)[0]])
        elif name in er.NAME_LISTS:
            value = ','.join(getattr(base, name))
        else:
            value = str(getattr(base, name))
        assert er.build_config({name: value}) == base, name


@pytest.mark.parametrize('overrides,match', [
    ({'sigma_whee1': '0.1'}, 'unknown parameter'),
    ({'sigma_wheel': 'abc'}, 'sigma_wheel'),
    ({'sigma_wheel': '-1'}, 'sigma_wheel must be finite and > 0'),
    ({'accel_source': 'gps'}, 'accel_source'),
    ({'bno_x_offset': 'nan'}, 'x_offset values must be finite'),
])
def test_overrides_the_node_would_refuse_are_refused(overrides, match):
    with pytest.raises(ValueError, match=match):
        er.build_config(overrides)


def test_nothing_to_score_against_is_nan_not_zero():
    axes, motion = _synthetic(seconds=5.0)
    streams, _ = er.streams_from_axes(axes)
    res = er.replay(streams, axes['odom'], er.build_config({}))
    s = er.score(res, None)
    assert math.isnan(s['speed_fused']) and math.isnan(s['heading_gyro'])
    lines = er.report({'odom': np.zeros((0, 7))}, motion, {})
    assert lines[-1] == '  no /odom in this run: nothing to replay'
