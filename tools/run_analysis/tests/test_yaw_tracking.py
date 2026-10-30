"""yaw_tracking.py on signals whose answer is built in.

No bag, no ROS:

    python3 -m pytest tools/run_analysis/tests/test_yaw_tracking.py -q

Oracles, named in each test: closed forms (a delayed copy, a linear or
sinusoidal input to a moving average, white noise of known sigma, rigid-body
kinematics) and invariants. The end-to-end test builds a whole run with car
2's numbers (0.36 m wheelbase, LiDAR 0.26 m ahead of base_link, VESC IMU at
50 Hz, LiDAR at 40 Hz) and a known steering lag, LiDAR lag, /odom lead and
grip limit, and checks analyse() finds each one.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from run_analysis import imu_spectrum as ims  # noqa: E402
from run_analysis import yaw_tracking as yt  # noqa: E402
from run_analysis.scan_motion import Motion  # noqa: E402

FS = 50.0
L = 0.36


def _band_limited(n, fs, seed=0, cutoff_hz=3.0):
    """Random signal with no content above cutoff_hz: a sub-sample delay of
    it is well defined by interpolation."""
    rng = np.random.default_rng(seed)
    X = np.fft.rfft(rng.standard_normal(n))
    X[np.fft.rfftfreq(n, 1 / fs) > cutoff_hz] = 0
    return np.fft.irfft(X, n)


# ---------------------------------------------------------------- xcorr_lag

def test_xcorr_lag_finds_a_sub_sample_delay_and_its_sign():
    # oracle: closed form -- b is a delayed by exactly 37 ms (1.85 samples),
    # built by shifting in the frequency domain (exact for a band-limited a).
    n = 2000
    a = _band_limited(n, FS)
    f = np.fft.rfftfreq(n, 1 / FS)
    b = np.fft.irfft(np.fft.rfft(a) * np.exp(-2j * np.pi * f * 0.037), n)
    lag, r = yt.xcorr_lag(a, b, FS, 0.3)
    assert lag == pytest.approx(0.037, abs=0.002)   # 2 ms, a tenth of a sample
    assert r > 0.99
    lag_rev, _ = yt.xcorr_lag(b, a, FS, 0.3)
    assert lag_rev == pytest.approx(-0.037, abs=0.002)


def test_xcorr_lag_zero_for_identical_series():
    a = _band_limited(1000, FS, seed=1)
    lag, r = yt.xcorr_lag(a, a, FS, 0.3)
    assert lag == pytest.approx(0.0, abs=1e-9)
    assert r == pytest.approx(1.0, abs=1e-9)


def test_xcorr_lag_refuses_flat_short_and_out_of_range():
    a = _band_limited(1000, FS, seed=2)
    assert math.isnan(yt.xcorr_lag(a, np.zeros(1000), FS, 0.3)[0])        # flat
    assert math.isnan(yt.xcorr_lag(a[:20], a[:20], FS, 0.3)[0])           # shorter than 2 x 15 lags
    # A true delay of 0.5 s against a 0.3 s search, on a signal slow enough
    # (0.2 Hz) that its correlation falls monotonically across the range:
    # the peak sits on the edge, so the answer is "outside", never the edge.
    slow = _band_limited(3000, FS, seed=4, cutoff_hz=0.2)
    assert math.isnan(yt.xcorr_lag(slow[25:], slow[:-25], FS, 0.3)[0])
    assert yt.xcorr_lag(slow[10:], slow[:-10], FS, 0.3)[0] == pytest.approx(0.2, abs=0.002)


# ---------------------------------------------------------------- gains and bins

def test_ls_gain_closed_form_and_zero_reference():
    ref = np.array([1.0, -2.0, 3.0])
    assert yt.ls_gain(0.8 * ref, ref) == pytest.approx(0.8, abs=1e-12)
    assert math.isnan(yt.ls_gain(ref, np.zeros(3)))


def test_binned_recovers_a_different_gain_in_each_bin_with_edges_exact():
    # oracle: construction -- meas = 2 ref where x < 1, 0.5 ref where x >= 1;
    # x = 1.0 exactly must land in the upper bin ([lo, hi)).
    x = np.r_[np.full(30, 0.5), np.full(30, 1.0)]
    ref = np.linspace(1, 2, 60)
    meas = np.where(x < 1, 2.0, 0.5) * ref
    rows = yt.binned(x, ref, meas, (0.0, 1.0, 2.0, 3.0), min_n=25)
    assert rows[0]['n'] == 30 and rows[0]['gain'] == pytest.approx(2.0, abs=1e-12)
    assert rows[1]['n'] == 30 and rows[1]['gain'] == pytest.approx(0.5, abs=1e-12)
    assert rows[1]['nrmse'] == pytest.approx(0.5, abs=1e-12)   # |0.5r - r| / |r|
    assert rows[2]['n'] == 0 and math.isnan(rows[2]['gain'])


def test_binned_below_min_n_reports_count_only_and_exactly_min_n_counts():
    rows = yt.binned(np.full(24, 0.5), np.ones(24), np.ones(24), (0.0, 1.0), min_n=25)
    assert rows[0]['n'] == 24 and math.isnan(rows[0]['gain']) and math.isnan(rows[0]['rmse'])
    rows = yt.binned(np.full(25, 0.5), np.ones(25), np.full(25, 0.9), (0.0, 1.0), min_n=25)
    assert rows[0]['gain'] == pytest.approx(0.9, abs=1e-12)


def test_ratio_bins_median_ignores_zero_denominators_and_outliers():
    # oracle: the median of the ratios 1.5 (x10) and 15 (x1) is 1.5; their
    # mean would be 2.86. The zero denominator is excluded, not inf.
    x = np.full(12, 0.5)
    num = np.r_[np.full(10, 3.0), 30.0, 1.0]
    den = np.r_[np.full(11, 2.0), 0.0]
    r = yt.ratio_bins(x, num, den, (0.0, 1.0), min_n=10)[0]
    assert r['n'] == 11 and r['ratio'] == pytest.approx(1.5, abs=1e-12)


# ---------------------------------------------------------------- grip limit

def _rows(ratios, step=0.5):
    return [dict(lo=i * step, hi=(i + 1) * step, n=100, x=i * step, ratio=r) for i, r in enumerate(ratios)]


def test_grip_limit_is_where_the_ratio_stays_below_the_floor():
    # oracle: the definition -- the first bin from which every later bin is
    # below 0.95. Here bins 2 and 3 (lower edges 1.0, 1.5).
    assert yt.grip_limit(_rows([1.02, 1.00, 0.92, 0.82])) == 1.0


def test_grip_limit_ignores_an_isolated_low_bin():
    # One low bin followed by a good one is scatter; only the tail counts.
    assert yt.grip_limit(_rows([0.70, 1.00, 0.90])) == 1.0


def test_grip_limit_nan_when_the_tyres_never_give_up_or_sit_exactly_at_the_floor():
    assert math.isnan(yt.grip_limit(_rows([1.0, 0.99, 0.97])))
    assert math.isnan(yt.grip_limit(_rows([1.0, 0.95])))         # at the floor is not below it
    assert yt.grip_limit(_rows([1.0, 0.9499])) == 0.5            # one step past it is
    assert math.isnan(yt.grip_limit(_rows([1.0, math.nan])))     # unmeasured bins do not count
    assert math.isnan(yt.grip_limit([]))


# ---------------------------------------------------------------- sideslip

def test_sideslip_zero_for_a_rigid_body_with_no_slip():
    # oracle: rigid-body kinematics -- a sensor x ahead of a non-slipping
    # rear axle sees v_lat = x w exactly.
    w = np.array([1.2, -0.8, 0.3])
    v = np.array([2.0, 1.5, 3.0])
    assert np.allclose(yt.sideslip(0.26 * w, w, v, 0.26), 0.0, atol=1e-12)


def test_sideslip_negative_when_the_rear_slides_out_in_either_direction():
    # oracle: closed form -- rear axle sliding 0.1 m/s toward the outside of
    # the turn at 2 m/s is atan(0.05) outward, for a left and a right turn.
    v = np.array([2.0, 2.0])
    w = np.array([1.0, -1.0])                  # left, right
    rear_out = np.array([-0.1, +0.1])          # outside of a left turn is -y
    beta = yt.sideslip(0.26 * w + rear_out, w, v, 0.26)
    assert beta == pytest.approx([-math.atan(0.05)] * 2, abs=1e-12)


# ---------------------------------------------------------------- maneuvers

def test_find_maneuvers_merges_same_sign_gaps_and_drops_short_ones():
    t = np.arange(0, 10, 0.02)
    x = np.zeros_like(t)
    x[(t >= 1.0) & (t < 1.6)] = 1.0        # 0.6 s left ...
    x[(t >= 1.7) & (t < 2.2)] = 1.0        # ... 0.1 s gap, same sign: joined
    x[(t >= 2.3) & (t < 2.9)] = -1.0       # right, right after: not joined
    x[(t >= 5.0) & (t < 5.3)] = 1.0        # 0.3 s: too short
    got = yt.find_maneuvers(t, x, 0.5, min_s=0.4, merge_s=0.2)
    assert [(round(t[a], 2), round(t[b - 1], 2), s) for a, b, s in got] == [(1.0, 2.18, 1), (2.3, 2.88, -1)]
    assert yt.find_maneuvers(t, np.zeros_like(t), 0.5, 0.4, 0.2) == []


def test_find_maneuvers_duration_boundary():
    # oracle: the definition -- duration t[last] - t[first] >= min_s is kept.
    t = np.arange(0, 3, 0.02)
    x = np.zeros_like(t)
    x[50:71] = 1.0
    exact = t[70] - t[50]                # 0.40 s, as the floats have it
    assert len(yt.find_maneuvers(t, x, 0.5, min_s=exact, merge_s=0.0)) == 1
    x[70] = 0.0                          # one sample shorter: 0.38 s
    assert yt.find_maneuvers(t, x, 0.5, min_s=exact, merge_s=0.0) == []


def _turn(t, t0, t1, rise=0.2):
    """A trapezoid: ramps up over rise, holds, ramps down over rise."""
    return np.clip(np.minimum((t - t0) / rise, (t1 - t) / rise), 0.0, 1.0)


def test_maneuver_stats_lag_ratio_heading_and_rebound():
    # oracle: construction -- meas is the command, 0.1 s later, scaled 0.8,
    # then a -0.2 rad/s swing back the command does not have.
    t = np.arange(0, 6, 0.02)
    cmd = 2.0 * _turn(t, 1.0, 3.0)
    meas = 1.6 * _turn(t, 1.1, 3.1) - 0.2 * _turn(t, 3.3, 3.5, rise=0.1)
    i0, i1 = int(np.searchsorted(t, 1.0)) + 1, int(np.searchsorted(t, 3.0))
    st = yt.maneuver_stats(t, cmd, meas, i0, i1, +1, tail_s=0.6)
    assert st['peak_ratio'] == pytest.approx(0.8, abs=1e-9)
    assert st['lag'] == pytest.approx(0.1, abs=0.021)            # one 20 ms sample
    assert st['rebound'] == pytest.approx(0.2, abs=1e-9)
    # Heading: commanded area 2 x (2.0 - 0.2) = 3.6 rad (trapezoid), measured
    # 0.8 x 3.6 less the rebound's triangle 0.2 x 0.1 = 0.02 rad.
    assert st['heading_ratio'] == pytest.approx((0.8 * 3.6 - 0.02) / 3.6, abs=0.01)
    # Settled at its plateau: no overshoot.
    assert st['overshoot_pct'] == pytest.approx(0.0, abs=1e-9)


def test_maneuver_stats_rebound_is_net_of_the_commands_own_swing_back():
    # oracle: construction -- an S-turn: the command itself swings back
    # -0.3 rad/s, the car -0.2. The car swung back LESS than told: -0.1.
    t = np.arange(0, 6, 0.02)
    cmd = 2.0 * _turn(t, 1.0, 3.0) - 0.3 * _turn(t, 3.1, 3.5, rise=0.1)
    meas = 2.0 * _turn(t, 1.0, 3.0) - 0.2 * _turn(t, 3.1, 3.5, rise=0.1)
    i0, i1 = int(np.searchsorted(t, 1.0)) + 1, int(np.searchsorted(t, 3.0))
    st = yt.maneuver_stats(t, cmd, meas, i0, i1, +1, tail_s=0.6)
    assert st['rebound'] == pytest.approx(-0.1, abs=1e-9)


def test_maneuver_stats_overshoot_against_own_settled_value_and_mirror():
    # oracle: closed form -- settles at 1.0 after a 1.3 peak: 30 % overshoot,
    # the same for a right turn (sign -1, everything negated).
    t = np.arange(0, 4, 0.02)
    cmd = _turn(t, 0.5, 2.5)
    meas = cmd + 0.3 * _turn(t, 0.8, 1.0, rise=0.1)
    i0, i1 = int(np.searchsorted(t, 0.5)) + 1, int(np.searchsorted(t, 2.5))
    for sign in (+1, -1):
        st = yt.maneuver_stats(t, sign * cmd, sign * meas, i0, i1, sign, tail_s=0.5)
        assert st['overshoot_pct'] == pytest.approx(30.0, abs=0.5)


def test_maneuver_stats_overshoot_ignores_the_next_turn_in_the_tail():
    # oracle: construction -- a turn that settles exactly on its command
    # (overshoot 0), then the measured yaw rate dips to 0.1 and a second
    # same-sign turn starts inside the 0.6 s tail. Run 20261026-174911 at
    # 18.5 s read +171 % this way when the plateau search ran into the tail.
    t = np.arange(0, 5, 0.02)
    cmd = _turn(t, 0.5, 2.0)
    meas = cmd.copy()
    meas[(t > 2.0) & (t < 2.3)] = 0.1
    meas[t >= 2.3] = 1.0
    i0, i1 = int(np.searchsorted(t, 0.5)) + 1, int(np.searchsorted(t, 2.0))
    st = yt.maneuver_stats(t, cmd, meas, i0, i1, +1, tail_s=0.6)
    assert st['overshoot_pct'] == pytest.approx(0.0, abs=1e-9)


def test_maneuver_stats_no_overshoot_for_a_turn_too_short_to_settle():
    t = np.arange(0, 3, 0.02)
    cmd = _turn(t, 1.0, 1.6, rise=0.1)
    i0, i1 = int(np.searchsorted(t, 1.0)) + 1, int(np.searchsorted(t, 1.6))
    st = yt.maneuver_stats(t, cmd, cmd, i0, i1, +1, tail_s=0.5, min_settle_s=1.0)
    assert math.isnan(st['overshoot_pct'])


# ---------------------------------------------------------------- held, window_average, noise, near

def test_held_false_for_hold_s_after_a_step_then_true():
    x = np.r_[np.zeros(100), np.full(100, 0.1)]
    h = yt.held(x, FS, 0.3, 0.02)            # 15 samples
    assert not h[:15].any()                  # no history yet
    assert h[15:100].all()
    assert not h[100:115].any()              # the step is inside the window
    assert h[115:].all()


def test_held_tolerance_boundary():
    x = np.r_[np.zeros(20), np.full(20, 0.02)]
    assert yt.held(x, FS, 0.3, 0.02)[25]          # exactly tol: held
    x[20:] = 0.0201
    assert not yt.held(x, FS, 0.3, 0.02)[25]      # one step past it


def test_window_average_of_a_ramp_is_its_centre_and_of_a_sine_is_sinc_scaled():
    # oracle: closed forms -- the mean of a t over [t-h, t+h] is a t; the mean
    # of sin(w t) is sin(w t) sin(w h) / (w h).
    # Sampled at 1 ms so the trapezoid integration error (dt^2 w^2 / 12,
    # 0.5 % at 20 ms for 2 Hz) is far below the tolerance.
    ts = np.arange(0, 10, 0.001)
    t = np.arange(1, 9, 0.1)
    assert np.allclose(yt.window_average(ts, 3.0 * ts, t, 0.025), 3.0 * t, atol=1e-9)
    w, h = 2 * np.pi * 2.0, 0.1
    got = yt.window_average(ts, np.sin(w * ts), t, h)
    assert np.allclose(got, np.sin(w * t) * np.sin(w * h) / (w * h), atol=1e-4)


def test_white_equivalent_sigma_recovers_white_noise():
    # oracle: white noise of sigma 0.002 rad/s at 50 Hz has density
    # sigma^2 / 25 Hz; the estimate from any band is sigma.
    x = np.random.default_rng(3).normal(0, 0.002, 20000)
    from run_analysis import spectrum as sp
    f, P = sp.psd(x, FS, segment_s=256 / FS)
    assert yt.white_equivalent_sigma(f, P, 10.0, 25.0, FS / 2) == pytest.approx(0.002, rel=0.05)
    assert math.isnan(yt.white_equivalent_sigma(f, P, 30.0, 40.0, FS / 2))   # empty band


def test_commanded_shifts_the_command_later_by_the_lag():
    # oracle: closed form -- delta is a ramp 0.1 t, so delta(t - 0.12) is
    # 0.1 (t - 0.12); kappa = tan of that / L, yaw rate = v kappa.
    t = np.arange(0, 5, 0.02)
    g = dict(t=t, cmd_t=t, cmd_delta=0.1 * t, wheelbase=L, v=np.full(t.size, 2.0))
    kappa, w = yt.commanded(g, 0.12)
    sel = t > 0.2
    assert np.allclose(kappa[sel], np.tan(0.1 * (t[sel] - 0.12)) / L, atol=1e-12)
    assert np.allclose(w[sel], 2.0 * kappa[sel], atol=1e-12)


def test_near_marks_gaps():
    src = np.r_[np.arange(0, 1, 0.02), np.arange(2, 3, 0.02)]
    t = np.array([0.5, 1.5, 2.5, 1.03, 1.98])
    assert yt.near(t, src, 0.1).tolist() == [True, False, True, True, True]


# ---------------------------------------------------------------- end to end

GRIP = 2.0          # m/s^2, the synthetic floor's limit
STEER_LAG = 0.12    # s, command -> car
LIDAR_LAG = 0.010   # s, the LiDAR reports the car this much late
BIAS = 0.01         # rad/s, gyro z
SEGMENTS = [(1.3, d) for d in (0.05, 0.10, 0.20, 0.30)] + [(1.6, d) for d in (0.25, 0.30, 0.349)] \
    + [(2.0, d) for d in (0.20, 0.30)] + [(3.2, d) for d in (0.03, 0.05)]     # (v m/s, |delta| rad)


def _synthetic_run():
    """30 s parked, then held turns at four speeds (left and right), with the
    yaw rate kinematic below GRIP and capped at GRIP / v above it. /odom is
    the undelayed kinematic command with speed 5 % high."""
    segs = SEGMENTS
    sched = [(30.0, 0.0, 0.0)]                        # (duration, v, delta)
    for v, d in segs:
        for sgn in (1, -1):
            sched += [(1.0, v, 0.0), (1.5, v, sgn * d)]
    sched.append((1.0, 1.3, 0.0))
    ends = np.cumsum([s[0] for s in sched])

    vals = {k: np.array([s[k] for s in sched]) for k in (1, 2)}

    def at(t, k):
        """Piecewise constant, each change a RAMP over 0.2 s (a servo's slew;
        gap_follow's max_steering_rate): a yaw rate is continuous, and a
        step would put its sub-sample timing beyond what samples record."""
        t = np.atleast_1d(np.asarray(t, dtype=float))
        i = np.minimum(np.searchsorted(ends, t, side='right'), len(sched) - 1)
        prev = vals[k][np.maximum(i - 1, 0)]
        start = np.where(i > 0, ends[np.maximum(i - 1, 0)], -1.0)
        frac = np.clip((t - start) / 0.2, 0.0, 1.0)
        return prev + (vals[k][i] - prev) * frac

    v_of = lambda t: at(t, 1)                          # noqa: E731
    d_of = lambda t: at(t, 2)                          # noqa: E731

    def w_true(t):
        v, d = v_of(t), d_of(t - STEER_LAG)
        w = v * np.tan(d) / L
        cap = GRIP / np.maximum(v, 1e-6)
        return np.sign(w) * np.minimum(np.abs(w), cap)

    T = ends[-1]
    rng = np.random.default_rng(7)
    ti = np.arange(0, T, 1 / 50)
    gyro = np.column_stack([rng.normal(0, 0.002, ti.size), rng.normal(0, 0.002, ti.size),
                            w_true(ti) + BIAS + rng.normal(0, 0.002, ti.size)])
    stream = ims.Stream('vesc', 'synthetic', ti, ti, np.zeros((ti.size, 3)), gyro)
    tl = np.arange(0.0125, T - 0.1, 1 / 40)
    wl = w_true(tl - LIDAR_LAG)
    motion = Motion(tl, v_of(tl), 0.26 * wl, wl, np.full(tl.size, 0.006))
    to = np.arange(0.005, T, 1 / 50)
    vo = v_of(to)
    # vesc_to_odom: its own (wheel) speed, 5 % high, times the commanded
    # tan(delta) / L, with no delay: it is the command, not the car.
    odom = np.column_stack([to, to, 1.05 * vo, 1.05 * vo * np.tan(d_of(to)) / L, 0 * to, 0 * to, 0 * to])
    td = np.arange(0.003, T, 1 / 40)
    drive = np.column_stack([td, td, d_of(td), v_of(td)])
    axes = dict(odom=odom, drive=drive, ackermann=np.zeros((0, 4)), core=np.zeros((0, 8)))
    return axes, stream, motion


@pytest.fixture(scope='module')
def synthetic():
    axes, stream, motion = _synthetic_run()
    return yt.analyse(axes, stream, motion, L)


def test_end_to_end_latencies(synthetic):
    # oracle: the injected delays. Gyro is LIDAR_LAG earlier than the LiDAR;
    # /odom (the undelayed command) leads the LiDAR by STEER_LAG + LIDAR_LAG;
    # the gyro follows the command by STEER_LAG.
    lw = synthetic['lag_whole']
    assert lw['gyro vs LiDAR'][0] == pytest.approx(-LIDAR_LAG, abs=0.003)         # 3 ms
    assert lw['odom vs LiDAR'][0] == pytest.approx(-(STEER_LAG + LIDAR_LAG), abs=0.010)
    assert lw['gyro vs command'][0] == pytest.approx(STEER_LAG, abs=0.010)
    assert synthetic['tracking']['steer_lag'] == pytest.approx(STEER_LAG, abs=0.010)


def test_end_to_end_grip_limit_and_linear_steering(synthetic):
    # oracle: GRIP, and a kinematic car (ratio 1) below it.
    tr = synthetic['tracking']
    assert tr['grip_limit'] == GRIP
    # The gyro's ratio, not /odom's (which reads 1.05 here).
    assert tr['held_linear']['ratio'] == pytest.approx(1.0, abs=0.02)
    # Rigid, no-slip LiDAR lateral speed: sideslip is zero in every band.
    assert all(abs(b['deg']) < 0.2 for b in tr['sideslip_by_a_lat'] if math.isfinite(b['deg']))


def test_end_to_end_bias_noise_and_odom_speed(synthetic):
    # oracle: injected BIAS and sigma 0.002 rad/s, /odom speed 1.05 x.
    assert synthetic['gyro_bias'] == pytest.approx(BIAS, abs=5e-4)
    # 30 s parked = ~1500 samples: the sigma estimate is good to ~2 %.
    assert synthetic['noise']['psd']['sigma_floor'] == pytest.approx(0.002, rel=0.05)
    assert synthetic['odom_speed_ratio'] == pytest.approx(1.05, abs=0.005)
    # The gyro after alignment matches the LiDAR up to its own noise and the
    # LiDAR's 50 ms window (which the comparison applies to the gyro too).
    assert synthetic['rmse']['gyro_vs_lidar_aligned']['gain'] == pytest.approx(1.0, abs=0.01)
    # ... and with the parked bias removed, no mean offset (0.01 rad/s if not).
    assert synthetic['rmse']['gyro_vs_lidar_aligned']['bias'] == pytest.approx(0.0, abs=0.002)


def test_end_to_end_maneuvers_found_with_their_direction(synthetic):
    # oracle: construction -- every held turn whose commanded yaw rate
    # v tan(delta) / L is above MANEUVER_RATE is one maneuver, left then
    # right. Closed form over the schedule: 1.3 m/s at 0.20 and 0.30 rad,
    # every 1.6 and 2.0 m/s level; not 1.3 x tan(0.10) / 0.36 = 0.36 rad/s
    # nor anything at 3.2 m/s (0.27, 0.45 rad/s).
    levels = [(v, d) for v, d in SEGMENTS if v * math.tan(d) / L > yt.MANEUVER_RATE]
    assert len(levels) == 7
    ms = synthetic['maneuvers']
    assert len(ms) == 2 * len(levels)
    assert [m['sign'] for m in ms] == [1, -1] * len(levels)
