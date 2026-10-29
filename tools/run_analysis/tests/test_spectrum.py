"""spectrum.py: timing, noise, spectra, aliasing and frequency response.

Every expected value is a closed form or an invariant, stated per test.
Synthetic signals only, seeded. Runs unsourced:

    python3 -m pytest tools/run_analysis/tests -q
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from run_analysis import spectrum as sp  # noqa: E402

FS = 50.0      # Hz, the VESC IMU's poll rate (vesc_driver.cpp 20 ms timer)


# ---------------------------------------------------------------- sinusoid, noise, DC

def test_known_sinusoid_frequency_and_amplitude_recovered():
    # oracle: closed form -- 0.2 + 0.7 sin(2 pi 3 t) over a whole number of
    # periods: one bin at 3 Hz reading 0.7 (the one-sided factor of 2 is
    # what makes it 0.7 rather than 0.35), DC 0.2, nothing else.
    t = np.arange(1000) / FS                    # 20 s: 3 Hz falls on bin 60
    x = 0.2 + 0.7 * np.sin(2 * math.pi * 3.0 * t)
    f, amp = sp.amplitude_spectrum(x, FS)
    k = int(np.argmax(amp[1:])) + 1
    assert f[k] == pytest.approx(3.0, abs=1e-12)            # Hz
    assert amp[k] == pytest.approx(0.7, abs=1e-9)
    assert amp[0] == pytest.approx(0.2, abs=1e-9)
    others = np.delete(amp, [0, k])
    assert np.max(others) < 1e-9


def test_white_noise_sigma_by_parseval():
    # oracle: invariant -- Parseval: sum_{k != 0} |X_k|^2 / N^2 IS the
    # population variance, to rounding. And the realised sigma of 20000
    # samples of N(0, 0.3^2) is 0.3 to within 3% (its standard error is 0.5%).
    rng = np.random.default_rng(1)
    x = rng.normal(0.0, 0.3, 20000)
    dc, sigma, sigma_std, n = sp.dc_and_sigma(x)
    assert n == 20000
    assert sigma == pytest.approx(np.std(x), abs=1e-12)
    assert sigma == pytest.approx(0.3, rel=0.03)
    assert sigma_std == pytest.approx(np.std(x), abs=0.0)
    # The Welch PSD integrated over the band gives the same noise.
    f, P = sp.psd(x, FS)
    assert sp.band_sigma(f, P, 0.0, FS / 2) == pytest.approx(0.3, rel=0.05)


def test_constant_offset_is_dc_and_excluded_from_sigma():
    # oracle: closed form -- DC is the mean; adding a constant changes
    # nothing but the k = 0 term, so sigma(2.5 + n) = sigma(n) exactly.
    rng = np.random.default_rng(2)
    noise = rng.normal(0.0, 0.05, 5000)
    dc, sigma, _, _ = sp.dc_and_sigma(2.5 + noise)
    assert dc == pytest.approx(2.5 + np.mean(noise), abs=1e-12)
    assert sigma == pytest.approx(np.std(noise), abs=1e-12)
    dc_c, sigma_c, _, _ = sp.dc_and_sigma(np.full(100, 2.5))
    assert dc_c == pytest.approx(2.5, abs=1e-12) and sigma_c == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------- frequency response

def _through(H_of_f, n=2000, fs=FS, seed=3):
    """u: a periodic multisine (every bin, random phase); y = H u, applied
    bin by bin, so Welch over one boxcar segment sees exactly H."""
    rng = np.random.default_rng(seed)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    U = np.exp(1j * rng.uniform(0, 2 * math.pi, len(f)))
    U[0] = 0.0
    U[-1] = 0.0
    u = np.fft.irfft(U, n)
    y = np.fft.irfft(U * H_of_f(f), n)
    return u, y


def _first_order_delay(K, T, tau):
    return lambda f: K * np.exp(-2j * math.pi * f * tau) / (1 + 2j * math.pi * f * T)


@pytest.mark.parametrize('tau', [0.15, -0.15])
def test_first_order_lag_plus_delay_matches_the_closed_form(tau):
    # oracle: closed form -- H = K e^{-i w tau} / (1 + i w T) with K 0.95,
    # T 0.1 s (car 2's measured gain and lag, docs/run-analysis.md, plus a
    # servo-like time constant): gain K / sqrt(1 + w^2 T^2), phase
    # -w tau - atan(w T). A negated delay must come back negated.
    K, T = 0.95, 0.1
    u, y = _through(_first_order_delay(K, T, tau))
    fr = sp.frequency_response(u, y, FS, window='boxcar', nperseg=len(u))
    sel = fr.valid & (fr.f <= 5.0)
    w = 2 * math.pi * fr.f[sel]
    assert sel.sum() > 50
    assert fr.gain[sel] == pytest.approx(K / np.sqrt(1 + (w * T) ** 2), abs=1e-9)
    assert fr.phase[sel] == pytest.approx(-w * tau - np.arctan(w * T), abs=1e-9)  # rad
    fit = sp.fit_first_order_delay(fr, f_max=5.0)
    assert fit['gain'] == pytest.approx(K, abs=1e-6)
    assert fit['time_constant'] == pytest.approx(T, abs=1e-6)       # s
    assert fit['delay'] == pytest.approx(tau, abs=1e-6)              # s


@pytest.mark.parametrize('tau', [0.15, -0.15])
def test_pure_delay_from_phase_slope(tau):
    # oracle: closed form -- H = e^{-i w tau}: phase slope -2 pi tau exactly.
    u, y = _through(lambda f: np.exp(-2j * math.pi * f * tau))
    fr = sp.frequency_response(u, y, FS, window='boxcar', nperseg=len(u))
    assert sp.delay_from_phase(fr, f_max=5.0) == pytest.approx(tau, abs=1e-9)  # s


def test_bandwidth_of_a_first_order_lag():
    # oracle: closed form -- a first-order lag falls to 1/sqrt(2) at
    # f = 1 / (2 pi T) = 1.592 Hz for T = 0.1 s.
    u, y = _through(_first_order_delay(1.0, 0.1, 0.0))
    fr = sp.frequency_response(u, y, FS, window='boxcar', nperseg=len(u))
    assert sp.bandwidth_hz(fr) == pytest.approx(1 / (2 * math.pi * 0.1), abs=0.01)  # Hz


def test_zero_coherence_gives_no_estimate_not_a_number():
    # oracle: invariant -- an output independent of the input has nothing
    # to estimate: no valid bin, so delay, fit and bandwidth are None.
    rng = np.random.default_rng(4)
    u = rng.normal(size=int(120 * FS))
    y = rng.normal(size=int(120 * FS))
    fr = sp.frequency_response(u, y, FS)
    assert fr.valid.sum() == 0
    assert fr.rows() == []
    assert sp.delay_from_phase(fr) is None
    assert sp.fit_first_order_delay(fr) is None
    assert sp.bandwidth_hz(fr) is None


def test_bins_the_input_never_excited_are_not_read():
    # oracle: invariant -- u has power only up to 3 Hz; the system is the
    # identity, so every bin is perfectly coherent, but above 3 Hz the input
    # is 300 dB down and those bins carry no information about H.
    def H(f):
        return np.ones_like(f, dtype=complex)
    rng = np.random.default_rng(5)
    n = 2000
    f = np.fft.rfftfreq(n, 1 / FS)
    U = np.exp(1j * rng.uniform(0, 2 * math.pi, len(f))) * np.where(f <= 3.0, 1.0, 1e-15)
    U[0] = 0.0
    u = np.fft.irfft(U, n)
    fr = sp.frequency_response(u, np.fft.irfft(U * H(f), n), FS, window='boxcar', nperseg=n)
    assert fr.valid.any() and np.all(fr.f[fr.valid] <= 3.0)


def test_pieces_combine_stretches_without_joining_them():
    # oracle: closed form -- two separate stretches through the same system
    # give the system's H at every bin; a stretch shorter than one segment
    # is left out rather than padded.
    sys_ = _first_order_delay(0.9, 0.05, 0.1)
    a = _through(sys_, seed=6)
    b = _through(sys_, seed=7)
    fr = sp.frequency_response_pieces([a, b, (np.ones(20), np.ones(20))], FS,
                                      window='boxcar', nperseg=2000)
    sel = fr.valid & (fr.f <= 5.0)
    w = 2 * math.pi * fr.f[sel]
    assert fr.gain[sel] == pytest.approx(0.9 / np.sqrt(1 + (w * 0.05) ** 2), abs=1e-9)
    assert fr.coherence[sel] == pytest.approx(1.0, abs=1e-9)
    # A constant input (no power once its mean is removed) has no estimate.
    assert sp.frequency_response_pieces([(np.ones(20), np.ones(20))], FS, nperseg=2000) is None


# ---------------------------------------------------------------- aliasing

def test_tone_above_nyquist_after_decimation_is_flagged_as_aliased():
    # oracle: closed form -- 37 Hz sampled at 200 Hz, then every 4th sample
    # kept (no anti-alias filter, as a 50 Hz poll of a faster IMU does),
    # folds to |37 - 50| = 13 Hz. The peak found is at 13 Hz, and against a
    # 37 Hz rotation it is classified as an alias of harmonic 1.
    t = np.arange(int(60 * 200)) / 200.0
    x = np.sin(2 * math.pi * 37.0 * t)[::4]
    f, P = sp.psd(x, FS)
    peak = sp.find_peaks(f, P)[0][0]
    assert peak == pytest.approx(13.0, abs=0.1)             # Hz
    assert sp.classify_peak(peak, 37.0, FS, tol_hz=0.2) == ('aliased', 1)
    # The same peak against a 13 Hz rotation is genuinely in band.
    assert sp.classify_peak(peak, 13.0, FS, tol_hz=0.2) == ('in_band', 1)
    assert sp.classify_peak(peak, 7.0, FS, tol_hz=0.2) == ('unexplained', None)


def test_find_peaks_keeps_real_tones_and_drops_leakage():
    # oracle: closed form -- tones of amplitude 1 and 0.3 (10.5 dB apart)
    # are both peaks; one of 0.001 (60 dB down, under the 20 dB limit) and
    # the Hann window's leakage around the strong tone are not.
    t = np.arange(int(60 * FS)) / FS
    x = (np.sin(2 * math.pi * 5.0 * t) + 0.3 * np.sin(2 * math.pi * 12.0 * t)
         + 0.001 * np.sin(2 * math.pi * 20.0 * t))
    f, P = sp.psd(x, FS)
    got = sorted(round(pf, 1) for pf, _ in sp.find_peaks(f, P))
    assert got == [5.0, 12.0]


def test_only_harmonics_one_to_four_are_matched():
    # oracle: spec (classify_peak docstring) -- 2 Hz is harmonic 7 of 14 Hz
    # folded at 50 Hz (98 Hz -> 2 Hz); beyond harmonic 4 it is not claimed.
    assert sp.fold(7 * 14.0, FS) == pytest.approx(2.0)
    assert sp.classify_peak(2.0, 14.0, FS, tol_hz=0.2) == ('unexplained', None)
    assert sp.classify_peak(sp.fold(3 * 14.0, FS), 14.0, FS, tol_hz=0.2) == ('aliased', 3)


def test_peak_trend_scales_folds_back_or_stays_put():
    # oracle: closed form -- harmonic 1 of 30/35/40 Hz sampled at 50 Hz
    # lands at 20/15/10 Hz: it moves DOWN as speed goes up.
    ref = [30.0, 35.0, 40.0]
    folded = [sp.fold(r, FS) for r in ref]
    assert folded == [20.0, 15.0, 10.0]
    assert sp.peak_trend(ref, folded) == (pytest.approx(-1.0), 'backwards')
    assert sp.peak_trend([5.0, 8.0, 11.0], [10.0, 16.0, 22.0]) == (pytest.approx(2.0), 'scales')
    assert sp.peak_trend(ref, [12.0, 12.0, 12.0])[1] == 'fixed'
    assert sp.peak_trend([5.0], [10.0])[1] == 'too few speeds'


# ---------------------------------------------------------------- timing and edges

def test_stream_stats_of_jittery_stamps_with_a_gap_and_a_backwards_stamp():
    # oracle: closed form -- dt cycling 18/20/22 ms in equal numbers:
    # median 20 ms (50 Hz), std 2 ms * sqrt(2/3). Plus one 0.5 s hole (a
    # gap, kept out of the jitter) and one repeated stamp.
    dt = np.tile([0.018, 0.020, 0.022], 100)
    t = np.concatenate([[0.0], np.cumsum(dt)])
    t = np.concatenate([t, t[-1] + 0.5 + np.concatenate([[0.0], np.cumsum(dt[:30])])])
    t = np.insert(t, 50, t[50])                     # a duplicate stamp
    s = sp.stream_stats(t)
    assert s.rate_hz == pytest.approx(50.0, abs=1e-6)
    assert s.dt_std_s == pytest.approx(0.002 * math.sqrt(2 / 3), abs=1e-9)   # s
    assert s.nyquist_hz == pytest.approx(25.0, abs=1e-9)
    assert len(s.gaps) == 1 and s.gaps[0][1] == pytest.approx(0.5, abs=1e-9)
    assert s.backwards == 1


@pytest.mark.parametrize('t', [[], [1.0], [math.nan, math.nan]])
def test_stream_stats_with_nothing_to_measure(t):
    s = sp.stream_stats(t)
    assert s.n == len([x for x in t if math.isfinite(x)])
    assert math.isnan(s.rate_hz) and math.isnan(s.nyquist_hz)
    assert 'no rate' in s.line()


def test_resample_never_interpolates_across_a_gap_and_drops_bad_samples():
    t = np.r_[np.arange(0.0, 1.0, 0.02), np.arange(3.0, 4.0, 0.02)]
    x = 2.0 * t
    x[10] = math.nan                                     # dropped, not interpolated as NaN
    t = np.insert(t, 21, t[20])                          # a second sample at t[20]:
    x = np.insert(x, 21, 99.0)                           # stamp does not move forward, dropped
    pieces = sp.resample_uniform(t, x, FS, max_gap_s=0.1)
    assert len(pieces) == 2
    (t1, x1), (t2, x2) = pieces
    assert t1[-1] < 1.0 and t2[0] >= 3.0                 # nothing inside the hole
    # oracle: closed form -- x = 2 t is linear, so interpolation is exact.
    assert x1 == pytest.approx(2.0 * t1, abs=1e-9)
    assert x2 == pytest.approx(2.0 * t2, abs=1e-9)
    assert np.allclose(np.diff(t1), 1 / FS)


def test_resample_edges():
    assert sp.resample_uniform([], [], FS, 0.1) == []
    assert sp.resample_uniform([0.0], [1.0], FS, 0.1) == []
    with pytest.raises(ValueError, match='t has 2 samples, x has 1'):
        sp.resample_uniform([0.0, 0.1], [1.0], FS, 0.1)
    with pytest.raises(ValueError, match='rate_hz'):
        sp.resample_uniform([0.0, 0.1], [1.0, 2.0], 0.0, 0.1)


def test_noise_and_spectrum_edges():
    dc, sigma, std, n = sp.dc_and_sigma([])
    assert n == 0 and math.isnan(dc) and math.isnan(sigma)
    dc, sigma, std, n = sp.dc_and_sigma([4.0])
    assert (dc, sigma, n) == (4.0, 0.0, 1)
    dc, sigma, std, n = sp.dc_and_sigma([1.0, math.nan, 3.0])
    assert (dc, n) == (2.0, 2) and sigma == pytest.approx(1.0, abs=1e-12)
    assert sp.psd(np.ones(10), FS) is None
    assert sp.frequency_response(np.ones(10), np.ones(10), FS) is None
    bad = np.ones(100)
    bad[5] = math.nan
    assert sp.frequency_response(bad, np.ones(100), FS) is None
    assert sp.delay_from_phase(None) is None and sp.bandwidth_hz(None) is None
