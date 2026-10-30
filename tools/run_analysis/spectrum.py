"""Timing, noise, spectra and frequency response of logged sensor streams.

numpy and scipy only: no ROS, no bag reading. imu_spectrum.py feeds it a
run's arrays; the tests feed it signals whose answer is known.

    stats = stream_stats(t)                     # rate, dt jitter, gaps, Nyquist
    for tu, xu in resample_uniform(t, x, 50.0, max_gap_s=0.1): ...
    dc, sigma, sigma_std, n = dc_and_sigma(xu)  # bias, and noise by Parseval
    f, P = psd(xu, 50.0)                        # Welch, one-sided density
    fr = frequency_response(u, y, 50.0)         # H = S_uy / S_uu with coherence
    delay_from_phase(fr)                        # None when nothing passes coherence

Logged stamps are not evenly spaced (the VESC IMU is stamped on arrival
after a 50 Hz poll), so every spectrum here is taken on a uniform grid made
by resample_uniform, and never across a gap.

"No estimate" is None, never a number: a frequency response with no bin
above its coherence threshold has no delay, no bandwidth and no fit.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np
from scipy import signal

# A dt this many times the median dt is a gap, not jitter.
GAP_FACTOR = 3.0
# Coherence below this and the gain/phase at that frequency are not
# reported: under 0.6, more than 40% of the output's power at that
# frequency is not explained by the input.
COHERENCE_MIN = 0.6
# And a bin is only read where the input actually drove it: input power
# within this many dB of the strongest bin. Above a weave's stick rate the
# input is nearly empty, yet a nonlinearity (tan) keeps those bins coherent
# while their gain is numerical leftovers.
EXCITATION_DB = 30.0


# ---------------------------------------------------------------- timing

@dataclass
class StreamStats:
    n: int
    duration_s: float
    rate_hz: float          # 1 / median dt
    dt_median_s: float
    dt_std_s: float
    dt_p99_s: float
    gaps: list = field(default_factory=list)   # [(t_start, length_s)] for dt > GAP_FACTOR * median
    backwards: int = 0      # dt <= 0: duplicate or out-of-order stamps
    nyquist_hz: float = math.nan

    def line(self) -> str:
        if self.n < 2:
            return f"{self.n} sample(s): no rate"
        return (f"{self.n} samples over {self.duration_s:.1f} s: {self.rate_hz:.1f} Hz "
                f"(dt median {self.dt_median_s * 1e3:.1f} ms, std {self.dt_std_s * 1e3:.1f} ms, "
                f"p99 {self.dt_p99_s * 1e3:.1f} ms), {len(self.gaps)} gap(s), "
                f"{self.backwards} backwards stamp(s), Nyquist {self.nyquist_hz:.1f} Hz")


def stream_stats(t, gap_factor: float = GAP_FACTOR) -> StreamStats:
    """Rate, dt jitter, gaps and Nyquist of a stream's timestamps [s]."""
    t = np.asarray(t, dtype=float)
    t = t[np.isfinite(t)]
    n = len(t)
    nan = math.nan
    if n < 2:
        return StreamStats(n, 0.0, nan, nan, nan, nan)
    dt = np.diff(t)
    backwards = int(np.sum(dt <= 0.0))
    pos = dt[dt > 0.0]
    if pos.size == 0:
        return StreamStats(n, 0.0, nan, nan, nan, nan, [], backwards)
    med = float(np.median(pos))
    gaps = [(float(t[i]), float(dt[i])) for i in np.where(dt > gap_factor * med)[0]]
    # Jitter is the spread of the dts that are not gaps: one dropout would
    # otherwise swamp it, and gaps are reported on their own.
    jitter = pos[pos <= gap_factor * med]
    rate = 1.0 / med
    return StreamStats(n, float(t[-1] - t[0]), rate, med, float(np.std(jitter)),
                       float(np.percentile(jitter, 99)), gaps, backwards, rate / 2.0)


def resample_uniform(t, x, rate_hz: float, max_gap_s: float, min_samples: int = 2) -> list:
    """[(tu, xu)]: x linearly interpolated onto a uniform grid, one piece per
    stretch without a gap longer than max_gap_s. x is (N,) or (N, k).
    Non-finite samples and stamps that do not move forward are dropped
    first; nothing is ever interpolated across a gap."""
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    if len(t) != len(x):
        raise ValueError(f"t has {len(t)} samples, x has {len(x)}")
    if not (math.isfinite(rate_hz) and rate_hz > 0.0):
        raise ValueError(f"rate_hz must be finite and > 0, got {rate_hz}")
    finite = np.isfinite(t) & (np.all(np.isfinite(x.reshape(len(x), -1)), axis=1) if len(x) else True)
    t, x = t[finite], x[finite]
    if len(t) == 0:
        return []
    keep = np.ones(len(t), dtype=bool)
    last = t[0]
    for i in range(1, len(t)):
        if t[i] <= last:
            keep[i] = False
        else:
            last = t[i]
    t, x = t[keep], x[keep]
    breaks = np.where(np.diff(t) > max_gap_s)[0]
    pieces = []
    for lo, hi in zip(np.r_[0, breaks + 1], np.r_[breaks + 1, len(t)]):
        ts, xs = t[lo:hi], x[lo:hi]
        if len(ts) < 2:
            continue
        tu = ts[0] + np.arange(int(math.floor((ts[-1] - ts[0]) * rate_hz)) + 1) / rate_hz
        if len(tu) < min_samples:
            continue
        if xs.ndim == 1:
            xu = np.interp(tu, ts, xs)
        else:
            xu = np.column_stack([np.interp(tu, ts, xs[:, j]) for j in range(xs.shape[1])])
        pieces.append((tu, xu))
    return pieces


# ---------------------------------------------------------------- noise

def dc_and_sigma(x):
    """(dc, sigma, sigma_std, n) of the finite samples of x.

    dc is the k = 0 term of the DFT (the mean: a still sensor's bias).
    sigma is the RMS of every other term, by Parseval:
    sigma^2 = sum_{k != 0} |X_k|^2 / N^2. sigma_std is np.std, the same
    number computed directly, printed beside it as a cross-check.
    """
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    n = len(x)
    if n == 0:
        return math.nan, math.nan, math.nan, 0
    X = np.fft.fft(x)
    dc = float(X[0].real / n)
    sigma = float(math.sqrt(float(np.sum(np.abs(X[1:]) ** 2))) / n)
    return dc, sigma, float(np.std(x)), n


def amplitude_spectrum(x, fs: float):
    """(f, amplitude): one-sided amplitude of each DFT bin, so a sinusoid of
    amplitude A on a bin reads A (twice |X_k| / N everywhere except DC and,
    for even N, Nyquist, which have no mirror image)."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n == 0:
        return np.zeros(0), np.zeros(0)
    X = np.fft.rfft(x)
    amp = np.abs(X) / n
    amp[1:] *= 2.0
    if n % 2 == 0:
        amp[-1] /= 2.0
    return np.fft.rfftfreq(n, 1.0 / fs), amp


def psd(x, fs: float, segment_s: float = 10.0):
    """(f, P) Welch power spectral density [unit^2/Hz], or None when there are
    fewer than 16 samples. Segments of segment_s, Hann window, mean removed."""
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if len(x) < 16:
        return None
    nperseg = int(min(len(x), max(16, round(segment_s * fs))))
    return signal.welch(x, fs=fs, nperseg=nperseg, detrend="constant")


def band_sigma(f, P, f_lo: float, f_hi: float) -> float:
    """RMS of the part of a PSD between f_lo and f_hi (DC bin excluded)."""
    sel = (f > 0.0) & (f >= f_lo) & (f <= f_hi)
    if sel.sum() < 2:
        return math.nan
    return float(math.sqrt(float(np.trapz(P[sel], f[sel]))))


def find_peaks(f, P, max_peaks: int = 5, prominence_db: float = 6.0, f_min: float = 0.5,
               within_db: float = 20.0) -> list:
    """[(frequency, power)] of up to max_peaks spectral peaks standing at
    least prominence_db above their surroundings and within within_db of
    the strongest, strongest first. The second limit drops a window's
    leakage around a strong tone, which otherwise reads as extra peaks."""
    sel = f >= f_min
    if sel.sum() < 3:
        return []
    fs_, Ps = f[sel], P[sel]
    db = 10.0 * np.log10(np.maximum(Ps, 1e-30))
    idx, _ = signal.find_peaks(db, prominence=prominence_db)
    if idx.size:
        idx = idx[db[idx] >= db[idx].max() - within_db]
    order = idx[np.argsort(Ps[idx])[::-1]][:max_peaks]
    return [(float(fs_[i]), float(Ps[i])) for i in order]


# ---------------------------------------------------------------- aliasing

def fold(f_hz: float, fs: float) -> float:
    """Where a tone at f_hz lands after sampling at fs: in [0, fs / 2]."""
    r = f_hz % fs
    return min(r, fs - r)


def classify_peak(peak_hz: float, ref_hz: float, fs: float, tol_hz: float,
                  orders=range(1, 5)):
    """('in_band', k), ('aliased', k) or ('unexplained', None).

    ref_hz is a rotation rate the vibration should follow (the motor's
    electrical frequency, say). A peak is harmonic k of it if k * ref_hz is
    within tol_hz of the peak; if that harmonic sits above Nyquist it can
    only appear folded, so a match through fold() is an alias. Harmonics
    1-4 only: with many more, their folded images cover so much of the band
    that some peak matches one by chance.
    """
    for k in orders:
        true = k * ref_hz
        if true <= fs / 2.0 and abs(true - peak_hz) <= tol_hz:
            return ("in_band", k)
    for k in orders:
        true = k * ref_hz
        if true > fs / 2.0 and abs(fold(true, fs) - peak_hz) <= tol_hz:
            return ("aliased", k)
    return ("unexplained", None)


def peak_trend(ref_hz, peak_hz) -> tuple:
    """(slope, verdict) of one peak tracked across several speeds.

    'scales': moves up with speed, as real vibration does. 'backwards':
    moves down as speed rises -- a tone folding back from above Nyquist.
    'fixed': does not move (structural resonance or electrical pickup).
    """
    ref = np.asarray(ref_hz, dtype=float)
    pk = np.asarray(peak_hz, dtype=float)
    ok = np.isfinite(ref) & np.isfinite(pk)
    ref, pk = ref[ok], pk[ok]
    if len(ref) < 2 or np.ptp(ref) == 0.0:
        return math.nan, "too few speeds"
    slope = float(np.polyfit(ref, pk, 1)[0])
    if abs(slope) * np.ptp(ref) < 0.5:          # moved less than 0.5 Hz over the range
        return slope, "fixed"
    return slope, "scales" if slope > 0.0 else "backwards"


# ---------------------------------------------------------------- frequency response

@dataclass
class FrequencyResponse:
    f: np.ndarray           # Hz, DC excluded
    gain: np.ndarray        # |H|
    phase: np.ndarray       # rad, unwrapped across the valid bins (NaN elsewhere)
    coherence: np.ndarray
    valid: np.ndarray       # coherence >= coherence_min and the input excited the bin
    coherence_min: float

    def rows(self) -> list:
        """[(f Hz, gain, phase deg, coherence)] for the valid bins only."""
        return [(float(f), float(g), float(math.degrees(p)), float(c))
                for f, g, p, c in zip(self.f[self.valid], self.gain[self.valid],
                                      self.phase[self.valid], self.coherence[self.valid])]


def frequency_response(u, y, fs: float, segment_s: float = 10.0,
                       coherence_min: float = COHERENCE_MIN, window: str = "hann",
                       nperseg: int | None = None):
    """H(f) = S_uy / S_uu from input u to output y (uniform, same length).

    None when the signals are too short or not finite. Otherwise every bin
    is returned, with ``valid`` marking those whose coherence passes
    coherence_min and whose input power is within EXCITATION_DB of the
    strongest; gain and phase elsewhere are not to be read.
    """
    return frequency_response_pieces([(u, y)], fs, segment_s, coherence_min, window, nperseg)


def frequency_response_pieces(pieces, fs: float, segment_s: float = 10.0,
                              coherence_min: float = COHERENCE_MIN, window: str = "hann",
                              nperseg: int | None = None):
    """frequency_response over several separate stretches [(u, y), ...]: the
    auto- and cross-spectra are summed (each weighted by its length), never
    the signals joined, so a stop between two weaves adds no false step.
    Coherence is |sum S_uy|^2 / (sum S_uu * sum S_yy), which for one piece is
    scipy.signal.coherence. Pieces shorter than one segment are skipped."""
    pieces = [(np.asarray(u, dtype=float), np.asarray(y, dtype=float)) for u, y in pieces]
    if any(len(u) != len(y) for u, y in pieces):
        return None
    pieces = [(u, y) for u, y in pieces
              if len(u) >= 16 and np.all(np.isfinite(u)) and np.all(np.isfinite(y))]
    if not pieces:
        return None
    nper = nperseg or int(max(16, round(segment_s * fs)))
    nper = min(nper, max(len(u) for u, _ in pieces))
    pieces = [(u, y) for u, y in pieces if len(u) >= nper]
    if not pieces:
        return None
    Suu = Syy = Suy = 0.0
    for u, y in pieces:
        w = len(u)
        f, Puu = signal.welch(u, fs=fs, window=window, nperseg=nper)
        _, Pyy = signal.welch(y, fs=fs, window=window, nperseg=nper)
        _, Puy = signal.csd(u, y, fs=fs, window=window, nperseg=nper)
        Suu, Syy, Suy = Suu + w * Puu, Syy + w * Pyy, Suy + w * Puy
    keep = (f > 0.0) & (Suu > 0.0)
    if not keep.any():
        return None                     # a constant input: nothing drove the output
    f, Suu, Syy, Suy = f[keep], Suu[keep], Syy[keep], Suy[keep]
    H = Suy / Suu
    with np.errstate(divide="ignore", invalid="ignore"):   # 0/0 where y has no power -> not valid
        C = np.abs(Suy) ** 2 / (Suu * Syy)
    excited = Suu >= Suu.max() * 10.0 ** (-EXCITATION_DB / 10.0)
    valid = np.isfinite(C) & (C >= coherence_min) & excited
    phase = np.full(len(f), math.nan)
    if valid.any():
        phase[valid] = np.unwrap(np.angle(H[valid]))
    return FrequencyResponse(f, np.abs(H), phase, C, valid, coherence_min)


def delay_from_phase(fr, f_max: float | None = None):
    """Pure delay [s] from the phase slope, phase ~ phi0 - 2 pi f tau, over
    the valid bins up to f_max. None with fewer than 3 valid bins. A lag
    (first-order or otherwise) also slopes the phase, so this reads that
    lag's group delay too; fit_first_order_delay separates the two."""
    if fr is None:
        return None
    sel = fr.valid & ((fr.f <= f_max) if f_max is not None else True)
    if sel.sum() < 3:
        return None
    slope = np.polyfit(fr.f[sel], fr.phase[sel], 1)[0]
    return float(-slope / (2.0 * math.pi))


def fit_first_order_delay(fr, f_max: float | None = None):
    """{'gain', 'time_constant', 'delay'} of H = K e^{-s tau} / (1 + s T), or None.

    |H|^-2 = (1 + w^2 T^2) / K^2 is linear in w^2, which gives K and T; then
    the phase left after removing -atan(w T) is -w tau, fitted through the
    origin. The form an MPC steering model takes directly.
    """
    if fr is None:
        return None
    sel = fr.valid & ((fr.f <= f_max) if f_max is not None else True)
    if sel.sum() < 3:
        return None
    w = 2.0 * math.pi * fr.f[sel]
    c1, c0 = np.polyfit(w * w, 1.0 / fr.gain[sel] ** 2, 1)
    if c0 <= 0.0:
        return None
    K = 1.0 / math.sqrt(c0)
    T = math.sqrt(c1 / c0) if c1 > 0.0 else 0.0
    rest = fr.phase[sel] + np.arctan(w * T)
    tau = float(-np.sum(w * rest) / np.sum(w * w))
    return {"gain": K, "time_constant": T, "delay": tau}


def bandwidth_hz(fr):
    """First frequency where the gain falls below 1/sqrt(2) of the lowest
    valid bin's (-3 dB), interpolated; None if it never does in the valid
    band, or with no valid bins."""
    if fr is None or not fr.valid.any():
        return None
    f, g = fr.f[fr.valid], fr.gain[fr.valid]
    ref = g[0] / math.sqrt(2.0)
    below = np.where(g < ref)[0]
    if below.size == 0 or below[0] == 0:
        return None
    i = below[0]
    return float(f[i - 1] + (ref - g[i - 1]) * (f[i] - f[i - 1]) / (g[i] - g[i - 1]))
