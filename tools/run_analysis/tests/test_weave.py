"""Tests for tools.run_analysis.weave -- diagnosing the steering weave.

Unsourced, no ROS:

    python3 -m pytest tools/run_analysis/tests/test_weave.py -v -p no:cacheprovider

Oracles are named per test: a closed form recomputed here, the characteristic
roots of the loop's own ODE (np.roots, independent of loop_model's formulas),
a construction whose answer is counted by hand, a stated invariant, or the
decision rule as written in the Task 7 brief (A5 of
docs/superpowers/specs/2026-10-26-slam-and-gap-follow-plan.md).
"""
import math
import os
import sys

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..'))                 # tools/
sys.path.insert(0, os.path.join(HERE, '..', '..', 'f1tenth_sim'))  # gap_mirror
from run_analysis import replay, weave  # noqa: E402
import gap_mirror  # noqa: E402

# The car's LiDAR sweep: 1081 beams over 270 degrees, from -135 degrees.
ANGLE_MIN = -2.35619449
ANGLE_INC = 4.71238898 / 1080


def _tick(t, target=None):
    """A replay.Tick at time t whose telemetry carries target_angle (none, for a stop tick)."""
    telemetry = {} if target is None else {'target_angle': target}
    return replay.Tick(t=t, scan_index=0, odom_speed=0.0, steering=0.0, speed=0.0,
                       real_steering=0.0, real_speed=0.0, stopped=False, stop_reason='',
                       binding='', telemetry=telemetry)


# ---------------------------------------------------------------- reversals_per_s

def test_reversals_counts_a_known_sine():
    # oracle: closed form. A 0.6 Hz sine of amplitude 0.15 rad swings 0.3 rad
    # peak to peak, twice a cycle: 1.2 reversals/s. Over [0, 10) s there are 12
    # extrema, at (k + 0.5)/1.2 s. The last (9.583 s) is never confirmed inside the
    # window: after it the trace rises only 0.15 rad (< 0.2 rad). So 11 turns are
    # counted over the 9.975 s span.
    t = np.arange(0.0, 10.0, 0.025)
    steering = 0.15 * np.sin(2 * math.pi * 0.6 * t)
    rate = weave.reversals_per_s(t, steering, swing_rad=0.2)
    assert rate == pytest.approx(11 / (t[-1] - t[0]), abs=1e-9)   # reversals/s
    assert rate == pytest.approx(1.2, abs=0.11)                    # reversals/s, the unbounded sine


def test_reversals_is_zero_for_a_wiggle_below_the_threshold():
    # oracle: closed form. A 0.05 rad amplitude sine swings 0.1 rad peak to peak,
    # which never exceeds 0.2 rad, so nothing is counted.
    t = np.arange(0.0, 10.0, 0.025)
    steering = 0.05 * np.sin(2 * math.pi * 0.6 * t)
    assert weave.reversals_per_s(t, steering, swing_rad=0.2) == 0.0


def test_reversals_counts_a_swing_only_when_it_exceeds_the_threshold():
    # oracle: closed form. A square wave of N samples alternating 0 and 0.5 has
    # N - 2 interior turns, each a 0.5 rad swing. N = 11 gives 9 turns. At
    # swing_rad 0.25 they count; at 0.5 they do not, because a swing must
    # EXCEED the threshold (brief: "whose peak-to-peak swing exceeds swing_rad").
    x = np.array([0.0, 0.5] * 5 + [0.0])
    t = np.arange(x.size) * 0.25                                    # span 2.5 s
    assert weave.reversals_per_s(t, x, swing_rad=0.25) == pytest.approx(9 / 2.5, abs=1e-12)
    assert weave.reversals_per_s(t, x, swing_rad=0.5) == 0.0


def test_reversals_ignores_non_finite_samples_and_rejects_bad_inputs():
    # invariant: a NaN sample is skipped, so a trace with NaNs has the same turns
    # as the same trace with them removed.
    x = np.array([0.0, 0.5, np.nan, 0.0, 0.5, 0.0, np.nan, 0.5, 0.0])
    finite = x[np.isfinite(x)]
    assert len(weave.turning_points(x, 0.25)) == len(weave.turning_points(finite, 0.25))
    assert len(weave.turning_points(finite, 0.25)) > 0    # the invariant is not vacuous
    t = np.arange(x.size) * 0.25
    assert math.isnan(weave.reversals_per_s(t[:1], x[:1]))            # one sample: no duration
    assert math.isnan(weave.reversals_per_s(np.zeros(3), np.array([0.0, 0.5, 0.0])))
    with pytest.raises(ValueError, match='same length'):
        weave.reversals_per_s(t, x[:-1])
    with pytest.raises(ValueError, match='positive'):
        weave.reversals_per_s(t, x, swing_rad=0.0)


# ---------------------------------------------------------------- loop_model

def test_loop_model_matches_the_closed_form():
    # oracle: closed form, the brief's values. tau = 0.2, v = 2.0, K = 0.9, L = 0.36:
    # v*K/L = 5, so zeta = 1/(2*sqrt(0.2*5)) = 0.5, and the damped frequency is
    # sqrt(25)/(2*pi) * sqrt(1 - 0.5**2) Hz.
    zeta, f_hz = weave.loop_model(0.2, 2.0, 0.9, 0.36)
    assert zeta == pytest.approx(1 / (2 * math.sqrt(1.0)), abs=1e-12)           # 0.5
    assert f_hz == pytest.approx(math.sqrt(25.0) / (2 * math.pi) * math.sqrt(0.75),
                                 abs=1e-12)                                     # Hz


@pytest.mark.parametrize('tau, v, gain, wheelbase', [
    (0.2, 2.0, 0.9, 0.36),
    (0.15, 1.5, 1.0, 0.36),
    (0.1, 3.0, 0.7, 0.30),
])
def test_loop_model_frequency_is_the_imaginary_part_of_the_characteristic_roots(
        tau, v, gain, wheelbase):
    # oracle: independent of loop_model's formulas. tau*s^2 + s + v*K/L = 0 is the
    # loop's characteristic equation. Its roots are -zeta*wn +- i*wd, and np.roots
    # solves them numerically.
    roots = np.roots([tau, 1.0, v * gain / wheelbase])
    assert np.all(np.abs(roots.imag) > 0)                                       # underdamped
    zeta_ref = -roots[0].real / abs(roots[0])
    f_ref = abs(roots[0].imag) / (2 * math.pi)                                  # Hz
    zeta, f_hz = weave.loop_model(tau, v, gain, wheelbase)
    assert zeta == pytest.approx(zeta_ref, abs=1e-9)                            # unitless
    assert f_hz == pytest.approx(f_ref, abs=1e-9)                               # Hz


def test_loop_model_rejects_nonpositive_inputs():
    # oracle: the stated domain. tau, speed, gain and wheelbase must be positive
    # and finite; zero, negative and NaN are refused, not returned as a number.
    bad = [(0.0, 2.0, 0.9), (-0.2, 2.0, 0.9), (0.2, 0.0, 0.9), (0.2, -2.0, 0.9),
           (0.2, 2.0, 0.0), (0.2, 2.0, -0.9), (math.nan, 2.0, 0.9)]
    for tau, v, gain in bad:
        with pytest.raises(ValueError, match='positive'):
            weave.loop_model(tau, v, gain, 0.36)
    with pytest.raises(ValueError, match='positive'):
        weave.loop_model(0.2, 2.0, 0.9, 0.0)


def test_loop_model_has_no_oscillation_when_overdamped():
    # oracle: the characteristic roots are real when 4*tau*v*K/L <= 1, so there is
    # no oscillation and no frequency. tau = 0.05, v = 1, K = 1, L = 0.36 gives
    # 4*tau*vK/L = 0.56 < 1.
    zeta, f_hz = weave.loop_model(0.05, 1.0, 1.0, 0.36)
    assert zeta > 1.0                                  # closed form: 1/(2*sqrt(0.139)) = 1.34
    assert f_hz == 0.0
    assert np.all(np.roots([0.05, 1.0, 1.0 / 0.36]).imag == 0)


# ---------------------------------------------------------------- target_jump_share

def test_target_jump_share_on_synthetic_ticks():
    # oracle: counted by hand. Ticks every 0.25 s. The target jumps 0 -> 0.3 rad at
    # 2.0 s, 0.3 -> 0 at 5.0 s and 0 -> 0.3 at 7.0 s. Four swings start at 2.0,
    # 3.5, 5.0 and 7.0 s. Three are at a jump, so the share is 3/4.
    times = np.arange(0.0, 10.0, 0.25)
    targets = np.where(((times >= 2.0) & (times < 5.0)) | (times >= 7.0), 0.3, 0.0)
    ticks = [_tick(t, a) for t, a in zip(times, targets)]
    assert weave.target_jump_share(ticks, np.array([2.0, 3.5, 5.0, 7.0])) == pytest.approx(
        0.75, abs=1e-12)
    # no swings: undefined, not a ZeroDivisionError and not a share of zero
    assert math.isnan(weave.target_jump_share(ticks, np.array([])))


def test_target_jump_share_counts_a_swing_within_one_scan_and_no_further():
    # oracle: the one-scan window, as stated in the docstring: a swing counts when
    # it starts within one scan (the 0.25 s tick spacing) of a jump, and the
    # boundary itself counts. The jump is at 2.0 s, the first tick at 0.3 rad.
    ticks = [_tick(t, 0.0 if t < 2.0 else 0.3) for t in np.arange(0.0, 4.0, 0.25)]

    def share(start):
        return weave.target_jump_share(ticks, np.array([start]))

    assert share(2.0) == 1.0     # on the jump
    assert share(2.25) == 1.0    # one scan after: inside the window
    assert share(1.75) == 1.0    # one scan before: inside the window
    assert share(2.5) == 0.0     # two scans after: outside
    assert share(1.5) == 0.0     # two scans before: outside


def test_target_jump_share_ignores_changes_not_larger_than_the_threshold():
    # oracle: "larger than jump_rad" is strict. A 0.25 rad step is not a jump at
    # jump_rad 0.25. A 0.5 rad step is, and a swing at it counts.
    quarter = [_tick(t, 0.0 if t < 2.0 else 0.25) for t in np.arange(0.0, 4.0, 0.25)]
    assert weave.target_jump_share(quarter, np.array([2.0]), jump_rad=0.25) == 0.0
    half = [_tick(t, 0.0 if t < 2.0 else 0.5) for t in np.arange(0.0, 4.0, 0.25)]
    assert weave.target_jump_share(half, np.array([2.0]), jump_rad=0.25) == 1.0


def test_target_jump_share_does_not_bridge_gaps_or_stop_ticks():
    # oracle: a jump is only seen between two consecutive ticks that both carry a
    # target angle and are one scan apart. A 4 s gap between LB windows (0 rad
    # before, 0.5 rad after) is not a jump.
    gap = ([_tick(t, 0.0) for t in np.arange(0.0, 1.25, 0.25)]
           + [_tick(t, 0.5) for t in np.arange(5.0, 6.0, 0.25)])
    assert weave.target_jump_share(gap, np.array([5.0])) == 0.0
    # a stop tick has no target angle: the 0 -> 0.5 step across it is not a jump either
    stop = [_tick(0.0, 0.0), _tick(0.25, 0.0), _tick(0.5), _tick(0.75, 0.5), _tick(1.0, 0.5)]
    assert weave.target_jump_share(stop, np.array([0.75])) == 0.0


def test_target_jump_share_is_nan_without_usable_targets():
    # oracle: no target evidence is undefined (nan), not a share of zero. A zero
    # would let a 'loop' verdict stand with no aim-point data behind it.
    ticks = [_tick(t) for t in np.arange(0.0, 2.0, 0.25)]
    assert math.isnan(weave.target_jump_share(ticks, np.array([1.0])))
    assert math.isnan(weave.target_jump_share([], np.array([1.0])))


# ---------------------------------------------------------------- response_at_peak

def test_response_at_peak_recovers_a_known_delay_and_peak():
    # oracle: closed form. Steering is a tone at 1.5625 Hz (exactly 10 bins of the
    # 256-sample Welch grid at 40 Hz). Yaw is 0.9 x that tone delayed by 0.1 s (4
    # samples). So the peak is the tone, the coherence is 1, and the yaw lags by
    # 360 * 1.5625 * 0.1 = 56.25 degrees. Only one line here: a second line leaks
    # into this bin and moves the phase, which is tested on its own below.
    fs, n = 40.0, 4000
    t = np.arange(n) / fs
    f0, delay = 1.5625, 0.1
    steer = np.sin(2 * math.pi * f0 * t)
    yaw = 0.9 * np.sin(2 * math.pi * f0 * (t - delay))
    peak, coherence, lag_deg = weave.response_at_peak([(steer, yaw)])
    assert peak == pytest.approx(f0, abs=1e-9)                       # Hz, an exact bin
    assert coherence == pytest.approx(1.0, abs=1e-6)                 # unitless
    assert lag_deg == pytest.approx(360 * f0 * delay, abs=1e-6)      # degrees, positive: yaw lags


def test_response_at_peak_searches_only_inside_the_band():
    # oracle: the search band (0.2 to 5 Hz). A 0.1 Hz swell at three times the tone's
    # amplitude sits outside the band and is the largest line in the steering
    # spectrum (it wins the argmax with no band, at 0.156 Hz). The peak must still be
    # the 1.5625 Hz tone. Only the peak is asserted: the swell's leakage moves the
    # phase at the tone's bin by up to 0.003 degrees.
    fs, n = 40.0, 4000
    t = np.arange(n) / fs
    steer = np.sin(2 * math.pi * 1.5625 * t) + 3.0 * np.sin(2 * math.pi * 0.1 * t)
    yaw = 0.9 * np.sin(2 * math.pi * 1.5625 * (t - 0.1))
    peak, _coherence, _lag = weave.response_at_peak([(steer, yaw)])
    assert peak == pytest.approx(1.5625, abs=1e-9)                   # Hz


def test_response_at_peak_is_low_for_unrelated_noise():
    # invariant: independent noise has coherence near its null bias, about 1/segments
    # (30 segments here). So the coherence at the peak stays far below the 0.7 loop bar.
    # Seeded, so deterministic.
    rng = np.random.default_rng(7)
    x = rng.standard_normal(4000)
    y = rng.standard_normal(4000)
    _peak, coherence, _lag = weave.response_at_peak([(x, y)])
    assert 0.0 <= coherence < 0.3


def test_response_at_peak_with_no_segment_long_enough_is_nan():
    # oracle: a segment shorter than one Welch window (256 samples) has no spectrum.
    short = np.zeros(100)
    peak, coherence, lag = weave.response_at_peak([(short, short)])
    assert math.isnan(peak) and math.isnan(coherence) and math.isnan(lag)


# ---------------------------------------------------------------- classify

@pytest.mark.parametrize('coherence, peak, model, share, expected', [
    # The decision rule as written in the Task 7 brief: 'loop' needs coherence at
    # least 0.7, the model within 30% of the peak and a jump share below 0.3;
    # 'aim_point' needs a jump share of at least 0.5; anything else is 'unclear'.
    (0.8, 1.0, 1.0, 0.1, 'loop'),
    (0.7, 1.0, 1.0, 0.1, 'loop'),            # coherence exactly 0.7 counts
    (0.69, 1.0, 1.0, 0.1, 'unclear'),        # just under
    (0.8, 1.0, 1.25, 0.1, 'loop'),           # model 25% off: within 30%
    (0.8, 1.0, 1.35, 0.1, 'unclear'),        # 35% off: outside 30%
    (0.8, 1.0, 1.0, 0.3, 'unclear'),         # share exactly 0.3 is not below 0.3
    (0.8, 1.0, 1.0, 0.29, 'loop'),
    (0.9, 1.0, 1.0, 0.5, 'aim_point'),       # share exactly 0.5 counts
    (0.9, 1.0, 1.0, 0.49, 'unclear'),
    (0.9, 1.0, 1.0, 0.8, 'aim_point'),       # the loop tests do not matter here
    (0.9, math.nan, math.nan, 0.6, 'aim_point'),
    (math.nan, 1.0, 1.0, 0.1, 'unclear'),    # every nan is unclear, never a verdict
    (0.9, 1.0, math.nan, 0.1, 'unclear'),
    (0.9, 1.0, 1.0, math.nan, 'unclear'),
])
def test_classify_applies_the_brief_decision_rule(coherence, peak, model, share, expected):
    assert weave.classify(coherence, peak, model, share) == expected


# ---------------------------------------------------------------- recorded_params

def test_recorded_params_reads_the_run_snapshot_and_its_key(tmp_path):
    # oracle: the snapshot written here, under the key race_diagnostics uses:
    # '/gap_follow_node' (a ros2 param dump). The value changed here must come
    # back, not the tree's value, and the result must run through the mirror.
    snapshot = dict(gap_mirror.node_params())
    snapshot['steering_gain'] = 0.7
    snap_dir = tmp_path / 'metadata' / 'params'
    snap_dir.mkdir(parents=True)
    (snap_dir / 'gap_follow_node.yaml').write_text(
        yaml.safe_dump({'/gap_follow_node': {'ros__parameters': snapshot}}))
    got = weave.recorded_params(tmp_path)
    assert got['steering_gain'] == 0.7
    assert gap_mirror.REQUIRED_PARAMS <= set(got)


def test_recorded_params_falls_back_to_the_tree_with_a_warning(tmp_path):
    # oracle: a run with no snapshot replays with the tree's gap_follow.yaml, and
    # says so.
    with pytest.warns(UserWarning, match='no gap_follow snapshot'):
        got = weave.recorded_params(tmp_path)
    assert got == gap_mirror.node_params()


# ---------------------------------------------------------------- diagnose

def _save_caches(run_dir, *, scan_t, scan, odom, drive, joy, bno):
    """The cached arrays load_run and load_imu read, so diagnose runs without a bag."""
    (run_dir / 'analysis').mkdir(parents=True)
    np.savez(run_dir / 'analysis' / 'run.npz', scan_t=scan_t, scan_stamp=scan_t, scan=scan,
             scan_meta=np.array([ANGLE_MIN, ANGLE_INC, 0.023, 30.0]),
             odom=odom, drive=drive, joy=joy)
    empty = np.zeros((0, 2))
    np.savez(run_dir / 'analysis' / 'imu.npz', bno=bno, vesc=empty,
             odom_imu=np.zeros((0, 4)), slam=np.zeros((0, 4)), slam_pose=np.zeros((0, 4)))


def _corridor_scan(angles, half_width=1.0, far_end=30.0, max_range=10.0):
    """Ranges from a straight corridor: walls at y = +-half_width, a far end at x = far_end."""
    s, c = np.sin(angles), np.cos(angles)
    side = half_width / np.maximum(np.abs(s), 1e-9)
    ahead = np.where(c > 0, far_end / np.maximum(c, 1e-9), np.inf)
    return np.clip(np.minimum(side, ahead), 0.05, max_range).astype(np.float32)


def test_diagnose_says_unclear_with_every_key_when_the_car_never_drove(tmp_path):
    # oracle: the output contract. LB is never held, so there is no driving to
    # measure. Every number is nan and the verdict is 'unclear', never a verdict
    # built on nothing.
    t = np.arange(0.0, 5.0, 0.025)
    angles = ANGLE_MIN + np.arange(1081) * ANGLE_INC
    _save_caches(
        tmp_path,
        scan_t=t, scan=np.repeat(_corridor_scan(angles)[None], t.size, axis=0),
        odom=np.column_stack([t, np.full_like(t, 1.5), np.zeros_like(t), np.zeros_like(t),
                              np.zeros_like(t), np.zeros_like(t), np.ones_like(t)]),
        drive=np.column_stack([t, t, np.zeros_like(t), np.full_like(t, 1.5)]),
        joy=np.column_stack([t, np.zeros_like(t)]),
        bno=np.column_stack([t, np.zeros_like(t)]))
    out = weave.diagnose(tmp_path)
    assert set(out) == {'peak_hz', 'coherence_at_peak', 'phase_lag_deg', 'tau_s', 'model_zeta',
                        'model_hz', 'target_jump_share', 'verdict'}
    assert out['verdict'] == 'unclear'
    for key in ('peak_hz', 'coherence_at_peak', 'phase_lag_deg', 'tau_s', 'model_zeta',
                'model_hz', 'target_jump_share'):
        assert math.isnan(out[key]), key


def test_diagnose_finds_the_loop_in_a_run_built_from_the_loop_model(tmp_path):
    # oracle: a run built to be a loop, with every expected value in closed form.
    # The heading loop has lag tau = 0.1 s, the car's turning gain is 0.9, the
    # controller's steering_gain and speed schedule come from the shipped config,
    # and v = 1.5 m/s on the 0.36 m wheelbase. The loop's damped frequency comes
    # from its characteristic roots (np.roots). Steering is a 0.2 rad sine at that
    # frequency. The gyro is the car's yaw for it, 0.9 * v * tan(steer(t - 0.1)) / L.
    # A straight corridor has no aim-point jumps. So the diagnosis must say 'loop',
    # with the lag, frequency, damping and phase all as built.
    p = gap_mirror.node_params()
    tau, speed, car_gain, wheelbase, amp = 0.1, 1.5, 0.9, 0.36, 0.2
    loop_gain = car_gain * p['steering_gain'] * min(1.0, p['steering_gain_speed_ref'] / speed)
    roots = np.roots([tau, 1.0, speed * loop_gain / wheelbase])
    f_model = abs(roots[0].imag) / (2 * math.pi)                    # Hz
    zeta_model = -roots[0].real / abs(roots[0])                     # unitless

    dt = 0.025
    t = np.arange(0.0, 40.0, dt)
    steer = amp * np.sin(2 * math.pi * f_model * t)
    gyro_t = np.arange(0.1, 40.0, dt)
    steer_at_lag = amp * np.sin(2 * math.pi * f_model * (gyro_t - tau))
    gyro_w = car_gain * speed * np.tan(steer_at_lag) / wheelbase
    angles = ANGLE_MIN + np.arange(1081) * ANGLE_INC
    joy_t = np.arange(0.0, 40.0, 0.1)
    odom_t = np.arange(0.0, 40.0, 0.02)
    _save_caches(
        tmp_path,
        scan_t=t, scan=np.repeat(_corridor_scan(angles)[None], t.size, axis=0),
        odom=np.column_stack([odom_t, np.full_like(odom_t, speed), np.zeros_like(odom_t),
                              np.zeros_like(odom_t), np.zeros_like(odom_t),
                              np.zeros_like(odom_t), np.ones_like(odom_t)]),
        drive=np.column_stack([t, t, steer, np.full_like(t, speed)]),
        joy=np.column_stack([joy_t, ((joy_t >= 1.0) & (joy_t < 39.0)).astype(float)]),
        bno=np.column_stack([gyro_t, gyro_w]))
    snap_dir = tmp_path / 'metadata' / 'params'
    snap_dir.mkdir(parents=True)
    (snap_dir / 'gap_follow_node.yaml').write_text(
        yaml.safe_dump({'/gap_follow_node': {'ros__parameters': p}}))

    out = weave.diagnose(tmp_path)
    assert out['verdict'] == 'loop'
    assert out['tau_s'] == pytest.approx(tau, abs=0.025)            # one lag-grid step
    assert out['model_hz'] == pytest.approx(f_model, rel=1e-2)      # Hz
    assert out['model_zeta'] == pytest.approx(zeta_model, rel=1e-2)  # unitless
    assert out['peak_hz'] == pytest.approx(f_model, rel=0.3)        # one bin is 0.156 Hz
    assert out['coherence_at_peak'] > 0.9                           # unitless
    assert out['phase_lag_deg'] == pytest.approx(360 * f_model * tau, abs=1.0)  # degrees
    assert out['target_jump_share'] == 0.0                          # a constant aim point
