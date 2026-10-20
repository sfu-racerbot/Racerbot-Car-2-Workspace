"""Tests for tools.run_analysis.actuation.

Oracle notes: best_lag does linear interp of cmd at cmd_t shifted by lag; 
analyze uses specific formulas from the implementation.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # insert tools/
from run_analysis import actuation
from run_analysis.scan_motion import Motion


def test_best_lag_finds_correct_lag():
    """oracle: closed form - measured(t)=cmd(t-0.2), cmd is sum of 3 sines sampled at 40 Hz for 30 s."""
    dt = 1.0 / 40.0
    t_cmd = np.arange(0, 30.0, dt)
    # smooth random-ish but deterministic signal (sum of 3 sines)
    rng = np.random.RandomState(123)
    freqs = rng.uniform(0.1, 2.0, size=3)
    phases = rng.uniform(0, 2*math.pi, size=3)
    amps = rng.uniform(0.5, 1.0, size=3)
    cmd = sum(a * np.sin(2*math.pi*f*t_cmd + p) for a,f,p in zip(amps,freqs,phases))
    lag_true = 0.2
    t_meas = np.arange(0, 30.0, dt)
    # measured(t) = cmd(t - lag_true)
    cmd_shifted = np.interp(t_meas - lag_true, t_cmd, cmd)
    measured = cmd_shifted
    lags = np.arange(0.0, 0.61, 0.025)
    lag_found, rms = actuation.best_lag(t_meas, measured, t_cmd, cmd, lags)
    assert lag_found == pytest.approx(0.2, abs=0.025)
    assert rms < 0.01

    # With lags not containing 0.2 (e.g. only [0, 0.05]) returns the closer one
    lags2 = np.array([0.0, 0.05])
    lag_found2, rms2 = actuation.best_lag(t_meas, measured, t_cmd, cmd, lags2)
    # distance to 0 is 0.2, to 0.05 is 0.15, so closer is 0.05
    assert lag_found2 == 0.05


def test_analyze_synthetic_known_answer():
    """oracle: closed form per analyze - v(t)=speed_cmd(t-0.25), w(t)=0.9*v(t)*tan(steer_cmd(t-0.10))/wheelbase; odom v depends on curvature per bins."""
    dt = 1.0 / 40.0
    t = np.arange(0, 30.0, dt)
    wheelbase = 0.36
    speed_cmd = 1.5 + 1.0 * np.sin(0.7 * t)
    steer_cmd = 0.2 * np.sin(1.3 * t)
    speed_lag_true = 0.25
    steer_lag_true = 0.10
    curvature_gain_true = 0.9
    # build motion at 40 Hz
    v_lidar = np.interp(t - speed_lag_true, t, speed_cmd)
    steer_delayed = np.interp(t - steer_lag_true, t, steer_cmd)
    w_lidar = curvature_gain_true * v_lidar * np.tan(steer_delayed) / wheelbase
    motion = Motion(t=t.copy(), v=v_lidar.copy(), v_lateral=np.zeros_like(v_lidar),
                    w=w_lidar.copy(), residual=np.zeros_like(v_lidar))
    # drive commands at 40 Hz
    drive = np.column_stack([t.copy(), t.copy(), steer_cmd.copy(), speed_cmd.copy()])
    # odom speed depends on measured curvature: v_odom = v / ratio(|k|) where k = w / max(v,0.3)
    k_meas = w_lidar / np.maximum(v_lidar, 0.3)
    abs_k = np.abs(k_meas)
    ratio = np.ones_like(abs_k)
    ratio[abs_k < 0.1] = 0.97
    ratio[(abs_k >= 0.1) & (abs_k < 0.3)] = 0.94
    ratio[(abs_k >= 0.3) & (abs_k < 0.6)] = 0.91
    ratio[abs_k >= 0.6] = 0.88
    odom_v = v_lidar / ratio
    odom = np.column_stack([t.copy(), odom_v.copy(), np.zeros_like(t), np.zeros_like(t),
                            np.zeros_like(t), np.zeros_like(t), np.ones_like(t)])  # (K,7)
    report = actuation.analyze(motion, drive, odom, t[0], t[-1], wheelbase=wheelbase, min_speed=1.0)
    assert report.speed_lag_s == pytest.approx(0.25, abs=0.025)
    assert report.steering_lag_s == pytest.approx(0.10, abs=0.025)
    assert report.curvature_gain == pytest.approx(0.9, abs=0.02)
    # assert each returned bin's ratio equals its band's value within 0.005 and at least 3 bins present
    assert len(report.odom_scale_bins) >= 3
    expected_map = {(0.0, 0.1): 0.97, (0.1, 0.3): 0.94, (0.3, 0.6): 0.91, (0.6, 2.0): 0.88}
    for lo, hi, n, r in report.odom_scale_bins:
        assert (lo, hi) in expected_map
        assert r == pytest.approx(expected_map[(lo, hi)], abs=0.005)
    # all six bins of analyze's grip loop are populated: a_cmd = v^2*|k_cmd|
    # sweeps 0.3..5.0 m/s^2 over this signal (v in 0.5..2.5 m/s, |k| up to
    # ~0.75 1/m), and the >= 5 minimum count holds for each (the synthetic
    # samples per bin are 180/138/120/95/86/57). Assert the count first so an
    # empty grip_bins can never make the loop below pass vacuously.
    assert len(report.grip_bins) == 6
    # every grip_bins ratio 0.9 within 0.03
    for lo, hi, n, ratio_g in report.grip_bins:
        assert ratio_g == pytest.approx(0.9, abs=0.03)


def test_analyze_raises_fewer_than_20():
    """oracle: code checks if t.size < 20: raise ValueError("fewer than 20 matched scans in the window")"""
    t = np.array([0.0, 0.025] * 5)  # 10 samples
    motion = Motion(t=t, v=np.ones(10), v_lateral=np.zeros(10), w=np.zeros(10), residual=np.zeros(10))
    drive = np.column_stack([t, t, np.zeros(10), np.ones(10)])
    odom = np.column_stack([t, np.ones(10), np.zeros(10), np.zeros(10), np.zeros(10), np.zeros(10), np.ones(10)])
    with pytest.raises(ValueError, match="fewer than 20"):
        actuation.analyze(motion, drive, odom, 0.0, 10.0)
