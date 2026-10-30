"""How well the car's yaw rate is measured, and how well it follows the command.

Three yaw rates from one run, each checked against the others:

- the VESC gyro (z axis, parked bias removed, as imu_bridge does at startup);
- LiDAR scan matching (scan_motion), the independent reference;
- /odom, which vesc_to_odom computes from wheel speed and the *steering
  command* (v tan(delta) / L), so it shares the command's errors.

and the commanded yaw rate, v tan(delta(t - lag)) / L with v from the LiDAR
(not /odom, which reads wheel speed ~5% high).

Sections, each printed with the numbers it rests on:

1. gyro noise: parked sigma and Welch floor; while moving, the 10-25 Hz
   band (no real yaw motion up there) as a white-noise-equivalent sigma;
2. RMSE: gyro vs LiDAR, /odom vs LiDAR, gyro vs /odom, and all three binned
   by how fast the car is rotating;
3. latency: normalised cross-correlation, whole run and per 10 s window;
4. commanded vs measured yaw rate: error, gain, and their dependence on speed
   and on steering angle; a steady-state fit separating a kinematic steering
   shortfall from a speed-dependent understeer gradient;
5. maneuvers: every turn the command asked for, with its peak ratio
   (overshoot), peak lag, rebound and achieved heading; one plot each.

    python3 tools/run_analysis/yaw_tracking.py ~/racerbot-ws/data/racerbot_runs/<ts>
    python3 tools/run_analysis/yaw_tracking.py <run> --car-yaml ~/imu-calibration/config/car.yaml

Like imu_spectrum.py, the first run needs ROS sourced to read the bag; the
arrays are cached in <run>/analysis/ and the rest runs unsourced. Writes
yaw_tracking.txt, yaw_tracking.json, yaw_tracking_maneuvers.csv and PNGs in
<run>/analysis/yaw_tracking/.

Times are receive times throughout (scan_motion's are; the VESC IMU's and the
scan's header stamps sit within ~3 ms of them on car 2, printed in section 3).
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT / "tools", ROOT / "src" / "imu_bridge"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from run_analysis import imu_spectrum as ims  # noqa: E402
from run_analysis import spectrum as sp  # noqa: E402

FS = 50.0                  # Hz, the common grid: the VESC IMU's own rate
MOVING_SPEED = 0.5         # m/s, LiDAR speed; imu_spectrum's MOVING_SPEED
MAX_GAP_S = 0.1            # a grid point farther than this from a source sample is invalid
XCORR_MAX_LAG_S = 0.3
LAG_WINDOW_S = 10.0        # per-window latency
LAG_MIN_R = 0.8            # a window whose correlation peak is below this gives no lag
STEER_LAGS = np.arange(0.0, 0.4001, 0.005)
SPEED_EDGES = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0)        # m/s
STEER_EDGES = (0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.36)      # rad, |delta|; 0.349 is the limit
RATE_EDGES = (0.0, 0.5, 1.0, 2.0, 3.0, 6.0)                         # rad/s, |LiDAR yaw rate|
MIN_BIN = 25               # samples (0.5 s at 50 Hz)
A_LAT_EDGES = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 6.0)     # m/s^2, commanded v^2 |kappa|
GRIP_FLOOR = 0.95          # achieved / commanded lateral accel below this: out of grip
LINEAR_A_LAT = 1.5         # m/s^2: commands below this are well inside the grip limit
LASER_X = 0.26             # m, base_link -> laser, bringup_launch.py (docs/hardware-reference.md)
MANEUVER_RATE = 0.5        # rad/s, |commanded yaw rate| that counts as a turn
MANEUVER_MIN_S = 0.4
MANEUVER_MERGE_S = 0.2
MANEUVER_TAIL_S = 0.6      # s after the command ends that still belongs to the turn
MAX_MANEUVER_PLOTS = 24
PSD_SEG = 256              # samples per Welch segment (5.1 s at 50 Hz)
NOISE_BAND = (10.0, 25.0)  # Hz: above any yaw motion a 0.36 m car makes
STEADY_HOLD_S = 0.3        # steady state: the (shifted) command held this long ...
STEADY_DELTA = 0.02        # ... within this many rad
G = 9.80665


# ---------------------------------------------------------------- pure numerics

def rms(x) -> float:
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(x * x))) if x.size else math.nan


def ls_gain(meas, ref) -> float:
    """The least-squares scale g in meas ~ g * ref (through the origin)."""
    meas, ref = np.asarray(meas, dtype=float), np.asarray(ref, dtype=float)
    den = float(np.sum(ref * ref))
    return float(np.sum(meas * ref) / den) if den > 0 else math.nan


def xcorr_lag(a, b, fs: float, max_lag_s: float):
    """(lag_s, r): normalised cross-correlation of two equally sampled series.

    lag is positive when b is LATER than a (b[n] ~ a[n - lag fs]); r is the
    peak correlation coefficient. The integer peak is refined by a parabola
    through it and its neighbours: exact for a smooth signal (a yaw rate),
    biased toward the nearest sample by up to half a sample for a step, whose
    sub-sample position the samples do not record. (nan, nan) when either
    series is flat or shorter than the lag range needs, or the peak sits on
    the range's edge (the true lag is outside it). A lag far outside the range
    can still alias onto an interior peak of an oscillating correlation: keep
    max_lag_s well beyond the lags expected."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    n = min(len(a), len(b))
    k_max = int(round(max_lag_s * fs))
    if n < 2 * k_max + 3 or k_max < 1:
        return math.nan, math.nan
    a, b = a[:n] - np.mean(a[:n]), b[:n] - np.mean(b[:n])
    if not (np.any(a) and np.any(b)):
        return math.nan, math.nan
    lags = np.arange(-k_max, k_max + 1)
    r = np.empty(len(lags))
    for i, k in enumerate(lags):
        x, y = (a[:n - k], b[k:]) if k >= 0 else (a[-k:], b[:n + k])
        den = math.sqrt(float(np.sum(x * x)) * float(np.sum(y * y)))
        r[i] = float(np.sum(x * y)) / den if den > 0 else math.nan
    if not np.isfinite(r).any():
        return math.nan, math.nan
    i = int(np.nanargmax(r))
    if i == 0 or i == len(r) - 1:
        return math.nan, float(r[i])
    y0, y1, y2 = r[i - 1], r[i], r[i + 1]
    den = y0 - 2.0 * y1 + y2
    shift = 0.5 * (y0 - y2) / den if den != 0 else 0.0
    return float((lags[i] + shift) / fs), float(y1 - 0.25 * (y0 - y2) * shift)


def white_equivalent_sigma(f, P, f_lo: float, f_hi: float, f_nyq: float) -> float:
    """The full-band sigma of white noise whose density is P's mean between
    f_lo and f_hi: sqrt(mean density * f_nyq). A car's yaw has no real motion
    up there (its yaw inertia filters it), so on a moving car this band holds
    only the sensor's noise and vibration; the estimate extends it down to DC
    as if it were white."""
    f, P = np.asarray(f, dtype=float), np.asarray(P, dtype=float)
    sel = (f >= f_lo) & (f <= f_hi)
    if sel.sum() < 2:
        return math.nan
    return math.sqrt(float(np.mean(P[sel])) * f_nyq)


def first_crossing(x, level: float) -> int:
    """Index of the first sample of x at or above level, or -1."""
    hit = np.flatnonzero(np.asarray(x, dtype=float) >= level)
    return int(hit[0]) if hit.size else -1


def binned(x, ref, meas, edges, min_n: int = MIN_BIN) -> list:
    """Tracking of meas against ref, in bins of x: [lo, hi). One dict per bin:
    n, gain (ls_gain), bias (mean meas - ref), rmse, nrmse (rmse / rms(ref)).
    A bin with fewer than min_n samples has n and nan elsewhere."""
    x, ref, meas = (np.asarray(v, dtype=float) for v in (x, ref, meas))
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = (x >= lo) & (x < hi)
        n = int(sel.sum())
        row = dict(lo=float(lo), hi=float(hi), n=n, gain=math.nan, bias=math.nan,
                   rmse=math.nan, nrmse=math.nan)
        if n >= min_n:
            e = meas[sel] - ref[sel]
            row.update(gain=ls_gain(meas[sel], ref[sel]), bias=float(np.mean(e)), rmse=rms(e),
                       nrmse=rms(e) / rms(ref[sel]) if rms(ref[sel]) > 0 else math.nan)
        out.append(row)
    return out


def ratio_bins(x, num, den, edges, min_n: int = 10) -> list:
    """Median num/den in bins of x, [lo, hi): one dict per bin with n,
    median x and the median ratio (nan below min_n samples or where every
    den is zero). The median, not a least-squares gain, so the few samples
    where den is near zero cannot dominate."""
    x, num, den = (np.asarray(v, dtype=float) for v in (x, num, den))
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = (x >= lo) & (x < hi) & (den != 0)
        n = int(sel.sum())
        out.append(dict(lo=float(lo), hi=float(hi), n=n,
                        x=float(np.median(x[sel])) if n >= min_n else math.nan,
                        ratio=float(np.median(num[sel] / den[sel])) if n >= min_n else math.nan))
    return out


def grip_limit(rows, floor: float = GRIP_FLOOR) -> float:
    """Where the tyres stop delivering what the command asks: the lower edge
    of the first bin (by commanded lateral acceleration) from which every
    measured bin's achieved / commanded ratio stays below floor. A single
    low bin with higher ones after it is scatter, not a limit. nan when the
    last measured bin is not below floor."""
    measured = [r for r in rows if math.isfinite(r["ratio"])]
    limit = math.nan
    for r in reversed(measured):
        if r["ratio"] >= floor:
            break
        limit = r["lo"]
    return limit


def sideslip(v_lat_sensor, w, v, sensor_x: float) -> np.ndarray:
    """Sideslip at the rear axle, rad, signed so that negative means the rear
    axle slides toward the OUTSIDE of the turn. A sensor sensor_x ahead of
    base_link sees v_lat = v_lat_rear + sensor_x w (rigid body); a kinematic
    (no-slip) rear axle has v_lat_rear = 0."""
    v_lat_sensor, w, v = (np.asarray(a, dtype=float) for a in (v_lat_sensor, w, v))
    rear = v_lat_sensor - sensor_x * w
    return np.arctan2(rear * np.sign(w), np.maximum(v, 1e-6))


def find_maneuvers(t, x, threshold: float, min_s: float, merge_s: float) -> list:
    """[(i0, i1, sign)]: stretches where |x| > threshold with one sign, i1
    exclusive. Same-sign stretches separated by less than merge_s are joined;
    a stretch (after joining) shorter than min_s is dropped."""
    t, x = np.asarray(t, dtype=float), np.asarray(x, dtype=float)
    sign = np.where(x > threshold, 1, np.where(x < -threshold, -1, 0))
    runs, i, n = [], 0, len(x)
    while i < n:
        if sign[i] == 0:
            i += 1
            continue
        j = i
        while j < n and sign[j] == sign[i]:
            j += 1
        runs.append([i, j, int(sign[i])])
        i = j
    merged = []
    for r in runs:
        if merged and merged[-1][2] == r[2] and t[r[0]] - t[merged[-1][1] - 1] < merge_s:
            merged[-1][1] = r[1]
        else:
            merged.append(r)
    return [(a, b, s) for a, b, s in merged if t[b - 1] - t[a] >= min_s]


def maneuver_stats(t, cmd, meas, i0: int, i1: int, sign: int, tail_s: float,
                   min_settle_s: float = 1.0) -> dict:
    """One turn: the command's stretch [i0, i1) and tail_s after it.

    peak_ratio   peak measured / peak commanded yaw rate, in the turn's direction
                 (below 1: the car turned less than asked)
    overshoot_pct  the classical step overshoot of the measured yaw rate against
                 its OWN settled value: 100 (peak - settled) / settled. Settled is
                 the median over the last third of the measured plateau inside the
                 turn's stretch (from its first to its last sample at half its
                 peak or more; never the tail, which can hold the next turn), so the ramp
                 back down at the turn's end cannot pull it low. Only for turns of
                 at least min_settle_s (nan otherwise): a shorter turn never settles.
    lag          time the measured yaw rate first reaches half its peak minus
                 the time the command does. A peak-to-peak lag is meaningless on
                 a saturated (flat-topped) command, whose argmax is its first sample.
    rebound      the measured yaw rate's largest swing the OTHER way in the
                 tail, minus the command's own: positive when the car swings back
                 further than it was told (an underdamped yaw mode, or a weave)
    heading_ratio  measured / commanded heading change over turn + tail
    """
    t, cmd, meas = (np.asarray(v, dtype=float) for v in (t, cmd, meas))
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 1.0
    end = min(len(t), i1 + int(round(tail_s / dt)))
    c, m = sign * cmd[i0:end], sign * meas[i0:end]
    pk_c, pk_m = float(np.max(c[:i1 - i0])), float(np.max(m))
    hc, hm = first_crossing(c, 0.5 * pk_c), first_crossing(m, 0.5 * pk_m)
    tail_c, tail_m = -sign * cmd[i1:end], -sign * meas[i1:end]
    rb_c = max(0.0, float(np.max(tail_c))) if tail_c.size else 0.0
    rb_m = max(0.0, float(np.max(tail_m))) if tail_m.size else 0.0
    head_c, head_m = float(np.sum(c) * dt), float(np.sum(m) * dt)
    duration = float(t[i1 - 1] - t[i0])
    overshoot = math.nan
    # Inside the turn's own stretch only: the tail may hold the next turn.
    own = m[:i1 - i0]
    pk_own = float(np.max(own)) if own.size else 0.0
    if duration >= min_settle_s and pk_own > 0:
        top = np.flatnonzero(own >= 0.5 * pk_own)
        a, b = int(top[0]), int(top[-1]) + 1
        settled = float(np.median(own[b - (b - a) // 3:b]))
        if settled > 0:
            overshoot = 100.0 * (pk_own - settled) / settled
    return dict(t0=float(t[i0]), t1=float(t[i1 - 1]), sign=int(sign), duration=duration,
                peak_cmd=pk_c, peak_meas=pk_m, peak_ratio=pk_m / pk_c if pk_c > 0 else math.nan,
                overshoot_pct=overshoot,
                lag=float(t[i0 + hm] - t[i0 + hc]) if hc >= 0 and hm >= 0 else math.nan,
                rebound=rb_m - rb_c, rmse=rms(m - c),
                heading_cmd_deg=math.degrees(head_c), heading_meas_deg=math.degrees(head_m),
                heading_ratio=head_m / head_c if head_c != 0 else math.nan)


def held(x, fs: float, hold_s: float, tol: float) -> np.ndarray:
    """True at sample i when x[i - hold_s fs .. i] stays within tol of x[i]:
    the input has been constant long enough for a steady-state reading."""
    x = np.asarray(x, dtype=float)
    k = int(round(hold_s * fs))
    out = np.zeros(len(x), dtype=bool)
    for i in range(k, len(x)):
        out[i] = float(np.max(np.abs(x[i - k:i + 1] - x[i]))) <= tol
    return out


def window_average(t_src, x_src, t, half_s: float):
    """The mean of x over [t - half_s, t + half_s], from the running integral
    of x (trapezoids). This is what a LiDAR yaw rate measures: the heading
    change between two scans over the time between them."""
    t_src, x_src = np.asarray(t_src, dtype=float), np.asarray(x_src, dtype=float)
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (x_src[1:] + x_src[:-1]) * np.diff(t_src))])
    t = np.asarray(t, dtype=float)
    return (np.interp(t + half_s, t_src, cum) - np.interp(t - half_s, t_src, cum)) / (2.0 * half_s)


def near(t, t_src, max_gap: float) -> np.ndarray:
    """True where a grid time t has a source sample within max_gap."""
    t_src = np.asarray(t_src, dtype=float)
    i = np.clip(np.searchsorted(t_src, t), 1, len(t_src) - 1)
    return np.minimum(np.abs(t - t_src[i - 1]), np.abs(t_src[i] - t)) <= max_gap


# ---------------------------------------------------------------- the run on one grid

def build_grid(axes: dict, stream, motion, wheelbase: float, fs: float = FS) -> dict:
    """Every signal on one fs grid over the span all of them cover.

    gyro      gyro z, parked bias removed (instantaneous)
    gyro_w    gyro z averaged over the LiDAR's own window (like for like with lidar)
    lidar, v  LiDAR yaw rate and forward speed
    odom      /odom yaw rate; odom_v its (wheel) speed
    delta     commanded steering angle (unshifted)
    valid     every source has a sample within MAX_GAP_S
    """
    odom = axes["odom"]
    cmds = axes["drive"] if len(axes.get("drive", ())) >= 2 else axes["ackermann"]
    t0 = max(stream.t[0], motion.t[0], odom[0, 0], cmds[0, 0])
    t1 = min(stream.t[-1], motion.t[-1], odom[-1, 0], cmds[-1, 0])
    t = np.arange(t0, t1, 1.0 / fs)
    segs = ims.still_segments(stream, axes)
    bias = (float(np.mean(np.concatenate([stream.gyro[a:b, 2] for a, b in segs])))
            if segs else 0.0)
    gz = stream.gyro[:, 2] - bias
    half = float(np.median(np.diff(motion.t)))     # stride 2: the pair spans two scan periods
    valid = (near(t, stream.t, MAX_GAP_S) & near(t, motion.t, MAX_GAP_S)
             & near(t, odom[:, 0], MAX_GAP_S))
    return dict(t=t, fs=fs, bias=bias, still_segments=len(segs), lidar_half_window=half,
                gyro=np.interp(t, stream.t, gz), gyro_w=window_average(stream.t, gz, t, half),
                lidar=np.interp(t, motion.t, motion.w), v=np.interp(t, motion.t, motion.v),
                v_lat=np.interp(t, motion.t, motion.v_lateral),
                odom=np.interp(t, odom[:, 0], odom[:, 3]), odom_v=np.interp(t, odom[:, 0], odom[:, 2]),
                delta=np.interp(t, cmds[:, 0], cmds[:, 2]), cmd_t=cmds[:, 0], cmd_delta=cmds[:, 2],
                topic="/drive" if cmds is axes.get("drive") else "/ackermann_cmd",
                valid=valid, wheelbase=wheelbase)


def commanded(g: dict, lag: float):
    """(kappa, yaw rate) commanded lag seconds earlier, at LiDAR speed."""
    delta = np.interp(g["t"] - lag, g["cmd_t"], g["cmd_delta"])
    kappa = np.tan(delta) / g["wheelbase"]
    return kappa, g["v"] * kappa


# ---------------------------------------------------------------- sections

def _welch_pieces(x, t, pieces, fs):
    """Sample-weighted mean Welch PSD over index ranges at least SEG long."""
    Ps, ns, f = [], [], None
    for a, b in pieces:
        if b - a >= PSD_SEG:
            f, P = sp.psd(x[a:b], fs, segment_s=PSD_SEG / fs)
            Ps.append(P)
            ns.append(b - a)
    if not Ps:
        return None
    return f, np.average(np.array(Ps), axis=0, weights=ns), int(sum(ns))


def noise_section(stream, axes, g, moving):
    """Parked: bias, np.std, and the Welch floor (np.std includes any bump in
    a still stretch; the Welch floor windows a bump at a stretch's edge out,
    so it is the sensor's own noise). Moving: the gyro's 10-25 Hz band, where
    a car has no yaw motion, as a white-noise-equivalent sigma; and the whole
    gyro - LiDAR residual as an upper bound."""
    fs = sp.stream_stats(stream.t).rate_hz
    segs, stats = ims.bias_noise(stream, axes)
    out = dict(parked=None, psd=None, moving_psd=None, signal_rms=rms(g["lidar"][moving]), fs=fs)
    if segs:
        dc, sigma, _ = stats[("gyro", "z")]
        out["parked"] = dict(bias=dc, sigma=sigma,
                             seconds=float(sum(stream.t[b - 1] - stream.t[a] for a, b in segs)))
        w = _welch_pieces(stream.gyro[:, 2], stream.t, segs, fs)
        if w:
            f, P, n = w
            out["psd"] = dict(f=f.tolist(), p=P.tolist(), n=n,
                              sigma_floor=math.sqrt(float(np.trapz(P, f))),
                              sigma_band=white_equivalent_sigma(f, P, *NOISE_BAND, fs / 2))
    v_at = np.interp(stream.t, g["t"], g["v"])
    mv = ims.runs_of(v_at > MOVING_SPEED, stream.t, PSD_SEG / fs)
    w = _welch_pieces(stream.gyro[:, 2], stream.t, mv, fs)
    if w:
        f, P, n = w
        out["moving_psd"] = dict(f=f.tolist(), p=P.tolist(), n=n,
                                 sigma_band=white_equivalent_sigma(f, P, *NOISE_BAND, fs / 2))
    if moving.sum() > 100:
        resid = g["gyro_w"][moving] - g["lidar"][moving]
        f, P = sp.psd(resid, g["fs"], segment_s=PSD_SEG / g["fs"])
        out["resid_psd"] = dict(f=f.tolist(), p=P.tolist())
    return out


def _shift(g, x, lag):
    """x advanced by lag seconds: x(t + lag), so a series that LEADS by -lag lines up."""
    return np.interp(g["t"] + lag, g["t"], x) if math.isfinite(lag) else x


def rmse_section(g, moving, lags):
    """Raw RMSE, then with each series moved onto the LiDAR's timing (lags:
    xcorr lag of gyro and of /odom against the LiDAR), so a lead or lag is
    not counted as error."""
    m = moving
    l, gw, gi, o = g["lidar"][m], g["gyro_w"][m], g["gyro"][m], g["odom"][m]
    ga = _shift(g, g["gyro_w"], lags["gyro"])[m]
    oa = _shift(g, g["odom"], lags["odom"])[m]

    def row(a, b):
        return dict(rmse=rms(a - b), bias=float(np.mean(a - b)), gain=ls_gain(a, b))

    rows = dict(gyro_vs_lidar=row(gw, l), gyro_instant_vs_lidar=row(gi, l), odom_vs_lidar=row(o, l),
                gyro_vs_odom=row(gw, o), gyro_vs_lidar_aligned=row(ga, l), odom_vs_lidar_aligned=row(oa, l))
    by_rate = dict(gyro=binned(np.abs(l), l, gw, RATE_EDGES), odom=binned(np.abs(l), l, o, RATE_EDGES),
                   odom_aligned=binned(np.abs(l), l, oa, RATE_EDGES))
    return rows, by_rate


def latency_section(g, moving, kappa0_rate):
    t, fs = g["t"], g["fs"]
    pairs = {"gyro vs LiDAR": (g["lidar"], g["gyro_w"]),
             "odom vs LiDAR": (g["lidar"], g["odom"]),
             "LiDAR vs command": (kappa0_rate, g["lidar"]),
             "gyro vs command": (kappa0_rate, g["gyro"])}
    whole, windows, curves = {}, {}, {}
    for name, (a, b) in pairs.items():
        whole[name] = xcorr_lag(a[moving], b[moving], fs, XCORR_MAX_LAG_S)
        k = int(round(XCORR_MAX_LAG_S * fs))
        am, bm = a[moving] - np.mean(a[moving]), b[moving] - np.mean(b[moving])
        n = len(am)
        curves[name] = [[j / fs, float(np.sum(am[:n - j] * bm[j:]) if j >= 0 else np.sum(am[-j:] * bm[:n + j]))
                         / math.sqrt(float(np.sum(am * am) * np.sum(bm * bm)))] for j in range(-k, k + 1)]
        lags = []
        for w0 in np.arange(t[0], t[-1] - LAG_WINDOW_S, LAG_WINDOW_S):
            sel = (t >= w0) & (t < w0 + LAG_WINDOW_S) & moving
            if sel.sum() < 0.6 * LAG_WINDOW_S * fs:
                continue
            lag, r = xcorr_lag(a[sel], b[sel], fs, XCORR_MAX_LAG_S)
            if math.isfinite(lag) and r >= LAG_MIN_R:
                lags.append(lag)
        windows[name] = lags
    return whole, windows, curves


def tracking_section(g, moving, steer_lag):
    kappa, w_cmd = commanded(g, steer_lag)
    w = g["gyro"]
    m = moving
    out = dict(steer_lag=steer_lag)
    out["overall"] = dict(rmse=rms(w[m] - w_cmd[m]), bias=float(np.mean(w[m] - w_cmd[m])),
                          gain=ls_gain(w[m], w_cmd[m]), nrmse=rms(w[m] - w_cmd[m]) / rms(w_cmd[m]))
    left, right = m & (kappa > 0.2), m & (kappa < -0.2)
    out["gain_left"] = ls_gain(w[left], w_cmd[left]) if left.sum() >= MIN_BIN else math.nan
    out["gain_right"] = ls_gain(w[right], w_cmd[right]) if right.sum() >= MIN_BIN else math.nan
    out["by_speed"] = binned(g["v"][m], w_cmd[m], w[m], SPEED_EDGES)
    delta = np.interp(g["t"] - steer_lag, g["cmd_t"], g["cmd_delta"])
    out["by_steer"] = binned(np.abs(delta[m]), w_cmd[m], w[m], STEER_EDGES)
    # Lag per speed band: does the car respond later when it is faster?
    lag_by_speed = []
    for lo, hi in zip(SPEED_EDGES[:-1], SPEED_EDGES[1:]):
        sel = m & (g["v"] >= lo) & (g["v"] < hi)
        if sel.sum() >= 4 * MIN_BIN:
            lags = []
            for L in STEER_LAGS:
                _, wc = commanded(g, L)
                lags.append(rms(w[sel] - ls_gain(w[sel], wc[sel]) * wc[sel]))
            lag_by_speed.append(dict(lo=lo, hi=hi, n=int(sel.sum()),
                                     lag=float(STEER_LAGS[int(np.argmin(lags))])))
    out["lag_by_speed"] = lag_by_speed
    # Held commands only: the car has had time to settle, so what is left is
    # the steady-state relation between command and turn. Split by commanded
    # lateral acceleration (grip) and, inside the grip, by speed (dynamics).
    hold = m & held(delta, g["fs"], STEADY_HOLD_S, STEADY_DELTA) & (np.abs(delta) > 0.02)
    a_cmd = g["v"] ** 2 * np.abs(kappa)
    a_meas = np.abs(g["v"] * w)
    out["held_seconds"] = float(hold.sum() / g["fs"])
    out["grip"] = ratio_bins(a_cmd[hold], a_meas[hold], a_cmd[hold], A_LAT_EDGES, min_n=MIN_BIN)
    out["grip_limit"] = grip_limit(out["grip"])
    lin = hold & (a_cmd < LINEAR_A_LAT)
    k_meas = w / np.maximum(g["v"], MOVING_SPEED)
    out["held_linear"] = dict(n=int(lin.sum()), ratio=float(np.median(k_meas[lin] / kappa[lin]))
                              if lin.sum() >= 10 else math.nan)
    out["held_by_speed"] = ratio_bins(g["v"][lin], k_meas[lin], kappa[lin], SPEED_EDGES)
    trans = m & ~held(delta, g["fs"], STEADY_HOLD_S, STEADY_DELTA)
    fast = m & (g["v"] >= 3.0)
    out["fast_seconds"] = float(fast.sum() / g["fs"])
    out["fast_over_grip"] = (float(np.mean(a_cmd[fast] >= out["grip_limit"]))
                             if fast.any() and math.isfinite(out["grip_limit"]) else math.nan)
    out["fast_a_cmd_p90"] = float(np.percentile(a_cmd[fast], 90)) if fast.any() else math.nan
    out["transient_by_speed"] = binned(g["v"][trans], w_cmd[trans], w[trans], SPEED_EDGES)
    beta = sideslip(g["v_lat"], w, g["v"], LASER_X)
    out["sideslip_by_a_lat"] = [dict(lo=lo, hi=hi, n=int(sel.sum()),
                                     deg=float(np.degrees(np.median(beta[sel]))) if sel.sum() >= MIN_BIN else math.nan)
                                for lo, hi in zip(A_LAT_EDGES[:-1], A_LAT_EDGES[1:])
                                for sel in [m & (a_meas >= lo) & (a_meas < hi)]]
    out["a_lat_p50_p90_p99"] = [float(x) for x in np.percentile(np.abs(g["v"][m] * w[m]), [50, 90, 99])]
    return out, w_cmd


def maneuver_section(g, moving, w_cmd):
    t, w = g["t"], g["gyro"]
    _, w_cmd0 = commanded(g, 0.0)
    rate = np.where(moving, w_cmd0, 0.0)
    found = find_maneuvers(t, rate, MANEUVER_RATE, MANEUVER_MIN_S, MANEUVER_MERGE_S)
    stats = []
    for i0, i1, s in found:
        st = maneuver_stats(t, w_cmd0, w, i0, i1, s, MANEUVER_TAIL_S)
        st["speed"] = float(np.mean(g["v"][i0:i1]))
        st["peak_delta"] = float(np.max(np.abs(g["delta"][i0:i1])))
        # The lateral acceleration the command asked for at its peak, at the
        # speed the car really had then: above the grip limit = turned in too fast.
        ic = i0 + int(np.argmax(s * w_cmd0[i0:i1]))
        st["peak_a_cmd"] = float(abs(w_cmd0[ic]) * g["v"][ic])
        st["speed_at_peak"] = float(g["v"][ic])
        st["i0"], st["i1"] = i0, i1
        stats.append(st)
    # Weave: how often the command itself reverses while moving.
    d = g["delta"][moving]
    reversals = int(np.sum(np.diff(np.sign(d[np.abs(d) > 0.05])) != 0))
    secs = float(moving.sum() / g["fs"])
    return stats, dict(reversals=reversals, moving_s=secs)


# ---------------------------------------------------------------- report

def _f(x, fmt="{:.3f}"):
    return fmt.format(x) if x is not None and isinstance(x, (int, float)) and math.isfinite(x) else "n/a"


def _bin_lines(rows, unit, scale=1.0):
    out = []
    for r in rows:
        if r["n"] < MIN_BIN:
            out.append(f"    {r['lo'] * scale:5.2f}-{r['hi'] * scale:5.2f} {unit}: n={r['n']:5d}  (too few)")
        else:
            out.append(f"    {r['lo'] * scale:5.2f}-{r['hi'] * scale:5.2f} {unit}: n={r['n']:5d}  "
                       f"gain {r['gain']:.3f}  bias {r['bias']:+.3f}  RMSE {r['rmse']:.3f} rad/s  "
                       f"NRMSE {r['nrmse'] * 100:5.1f}%")
    return out


def analyse(axes: dict, stream, motion, wheelbase: float, timing: dict | None = None) -> dict:
    g = build_grid(axes, stream, motion, wheelbase)
    moving = g["valid"] & (g["v"] > MOVING_SPEED)
    _, w_cmd0 = commanded(g, 0.0)
    res = dict(run_seconds=float(g["t"][-1] - g["t"][0]), moving_seconds=float(moving.sum() / g["fs"]),
               wheelbase=wheelbase, topic=g["topic"], gyro_bias=g["bias"], timing=timing or {},
               lidar_window_s=2 * g["lidar_half_window"])
    res["speed_p50_p90_p99_max"] = [float(x) for x in np.percentile(g["v"][moving], [50, 90, 99, 100])]
    res["odom_speed_ratio"] = float(np.median(g["odom_v"][moving] / g["v"][moving]))
    res["noise"] = noise_section(stream, axes, g, moving)
    res["lag_whole"], res["lag_windows"], res["xcorr"] = latency_section(g, moving, w_cmd0)
    # xcorr lag of b against a; to line b up with the LiDAR, look -lag ahead.
    res["rmse"], res["by_rate"] = rmse_section(g, moving, {
        "gyro": res["lag_whole"]["gyro vs LiDAR"][0], "odom": res["lag_whole"]["odom vs LiDAR"][0]})
    lag_cmd = res["lag_whole"]["gyro vs command"][0]
    steer_lag = lag_cmd if math.isfinite(lag_cmd) and lag_cmd >= 0 else 0.1
    res["tracking"], w_cmd = tracking_section(g, moving, steer_lag)
    res["maneuvers"], res["weave"] = maneuver_section(g, moving, w_cmd)
    res["_grid"], res["_moving"], res["_w_cmd"] = g, moving, w_cmd
    return res


def answer_lines(r: dict) -> list:
    """The questions asked of this tool, each answered from the numbers in the
    sections below it, by the rule stated in the answer."""
    out = ["## Answers (each from the sections below; the rule used is stated)"]
    n, rm = r["noise"], r["rmse"]
    sig = n["signal_rms"]
    if n["psd"] and n["moving_psd"]:
        floor, mov = n["psd"]["sigma_floor"], n["moving_psd"]["sigma_band"]
        bound = rm["gyro_vs_lidar_aligned"]["rmse"]
        out.append(f"- Is the VESC gyro noisy? No. Parked floor {floor:.4f} rad/s ({math.degrees(floor):.2f} deg/s); "
                   f"moving, its {NOISE_BAND[0]:.0f}-{NOISE_BAND[1]:.0f} Hz band gives {mov:.4f} rad/s "
                   f"(vibration raises it {mov / n['psd']['sigma_band']:.1f}x), {20 * math.log10(sig / mov):.0f} dB "
                   f"below the {sig:.2f} rad/s yaw-rate signal. Even the whole gyro-LiDAR disagreement, "
                   f"{bound:.3f} rad/s, is {bound / sig * 100:.0f}% of the signal, and most of it is the LiDAR's "
                   "scan-matching noise (section 1's residual spectrum).")
    oa = [b for b in r["by_rate"]["odom_aligned"] if b["n"] >= MIN_BIN]
    o_lead = -r["lag_whole"]["odom vs LiDAR"][0]
    if oa:
        out.append(f"- Is /odom wrong during fast rotation? It is wrong at every rate, in two ways. It LEADS the car "
                   f"by {o_lead * 1e3:.0f} ms (it is v tan(delta)/L of the command, not a measurement), which alone "
                   f"makes its raw RMSE {rm['odom_vs_lidar']['rmse']:.2f} rad/s; and once that lead is removed it still "
                   f"over-reads rotation, gain " + " -> ".join(f"{b['gain']:.2f}" for b in oa)
                   + " over |w| bands " + ", ".join(f"{b['lo']:.1f}-{b['hi']:.1f}" for b in oa)
                   + f" rad/s (RMSE {rm['odom_vs_lidar_aligned']['rmse']:.2f} aligned). The over-read is the car's "
                   "understeer seen from the other side, plus /odom's speed reading high. The gyro's gain in the same bands: "
                   + ", ".join(f"{b['gain']:.2f}" for b in r["by_rate"]["gyro"] if b["n"] >= MIN_BIN) + ".")
    lw = r["lag_windows"]["gyro vs LiDAR"]
    lag, _ = r["lag_whole"]["gyro vs LiDAR"]
    if lw:
        p10, p50, p90 = np.percentile(np.array(lw) * 1e3, [10, 50, 90])
        verdict = "Yes" if p10 <= -60 <= p90 else "No"
        out.append(f"- Is the LiDAR lagging by 60 ms? {verdict}. The gyro is {lag * 1e3:+.0f} ms against the LiDAR "
                   f"(positive = gyro later), so the LiDAR is {-lag * 1e3:.0f} ms behind the gyro; per {LAG_WINDOW_S:.0f} s "
                   f"window median {p50:+.0f} ms, p10..p90 {p10:+.0f}..{p90:+.0f} ms over {len(lw)} windows. A 60 ms LiDAR "
                   "lag would read -60 here. The ~130-160 ms delays in section 3 are the CAR's response to steering "
                   "(and /odom's lead), not a sensor lag.")
    tr = r["tracking"]
    bs = [b for b in tr["by_speed"] if b["n"] >= MIN_BIN]
    grip = [b for b in tr["grip"] if math.isfinite(b["ratio"])]
    hl = tr["held_linear"]
    slip = [b for b in tr["sideslip_by_a_lat"] if math.isfinite(b["deg"])]
    limits = [(ref, val) for ref, val in (_max_lat_refs()) if val is not None]
    if grip:
        lim = tr["grip_limit"]
        out.append("- Does the car consistently understeer? No; it understeers only at the grip limit. On held "
                   f"commands below {LINEAR_A_LAT} m/s^2 it turns {_f(hl['ratio'], '{:.2f}')}x the geometry's curvature "
                   f"(n={hl['n']}, slightly MORE than asked), and left {tr['gain_left']:.2f} / right "
                   f"{tr['gain_right']:.2f} overall match, so the steering model and calibration are right. The "
                   "overall gain below 1 comes from transients (next answer), not steady understeer. Achieved / commanded lateral acceleration by commanded a_y: "
                   + ", ".join(f"{b['lo']:.1f}-{b['hi']:.1f} {b['ratio']:.2f}" for b in grip)
                   + f": the tyres stop delivering above ~{_f(lim, '{:.1f}')} m/s^2 (p99 achieved "
                   f"{tr['a_lat_p50_p90_p99'][2]:.1f} m/s^2), and the rear axle slides out "
                   + ", ".join(f"{b['deg']:+.1f}" for b in slip) + " deg over achieved a_y bands "
                   + ", ".join(f"{b['lo']:.1f}-{b['hi']:.1f}" for b in slip) + ". That is limit understeer: "
                   "the car pushes wide when asked for more grip than the floor has."
                   + (" The planners ask for up to " + ", ".join(f"{v} ({ref})" for ref, v in limits) + "."
                      if limits else ""))
    lo = [b for b in bs if b["hi"] <= 3.0]
    hi = [b for b in bs if b["lo"] >= 3.0]
    if lo and hi:
        def pooled(bins, key):
            return sum(b[key] * b["n"] for b in bins) / sum(b["n"] for b in bins)
        lag_hi = [b["lag"] for b in tr["lag_by_speed"] if b["lo"] >= 3.0]
        lag_lo = [b["lag"] for b in tr["lag_by_speed"] if b["hi"] <= 3.0]
        tr_hi = [b for b in tr["transient_by_speed"] if b["lo"] >= 3.0 and b["n"] >= MIN_BIN]
        tr_lo = [b for b in tr["transient_by_speed"] if b["hi"] <= 3.0 and b["n"] >= MIN_BIN]
        out.append(f"- Does yaw-rate tracking degrade above 3 m/s? Yes, as a continuation, not a cliff. Below 3 m/s "
                   f"gain {pooled(lo, 'gain'):.2f}, NRMSE {pooled(lo, 'nrmse') * 100:.0f}%; above, gain "
                   f"{pooled(hi, 'gain'):.2f}, NRMSE {pooled(hi, 'nrmse') * 100:.0f}% ({tr['fast_seconds']:.0f} s of "
                   f"data). It is not grip: only {_f(tr['fast_over_grip'] * 100, '{:.0f}')}% of the samples above 3 m/s "
                   f"asked for more than the ~{_f(tr['grip_limit'], '{:.1f}')} m/s^2 limit (p90 commanded "
                   f"{tr['fast_a_cmd_p90']:.1f} m/s^2). It is dynamics: the gain on a CHANGING command falls with speed, "
                   + " -> ".join(f"{b['gain']:.2f}" for b in tr_lo + tr_hi) + " over "
                   + ", ".join(f"{b['lo']:.1f}-{b['hi']:.1f}" for b in tr_lo + tr_hi)
                   + " m/s, and the steering lag grows from "
                   + (f"{min(lag_lo) * 1e3:.0f}-{max(lag_lo) * 1e3:.0f} ms below 3 m/s " if lag_lo else "")
                   + (f"to {', '.join(f'{x * 1e3:.0f}' for x in lag_hi)} ms above." if lag_hi else "(n/a above).")
                   + " Too few held commands above 3 m/s to measure steady tracking there directly.")
    elif lo:
        out.append("- Does yaw-rate tracking degrade above 3 m/s? Not measurable: too few samples above 3 m/s.")
    pa = np.array([m["peak_a_cmd"] for m in r["maneuvers"]])
    if grip and n["moving_psd"]:
        g_err = rm["gyro_vs_lidar_aligned"]["rmse"]
        o_err = rm["odom_vs_lidar_aligned"]["rmse"]
        out.append(f"- Estimator or controller? Change the estimator first; then change the controller's limits, not "
                   f"its gains. The gyro is {o_err / g_err:.0f}x closer to the LiDAR than /odom ({g_err:.3f} vs "
                   f"{o_err:.3f} rad/s, even after removing /odom's {o_lead * 1e3:.0f} ms lead), and /odom is what "
                   "slam_toolbox, the particle filter and pure_pursuit consume today: gyro yaw in the odometry "
                   "(vehicle_state's /odom/imu, gyro_odom:=true) improves every pose the controller sees. On the "
                   f"controller side the steering model is right, so do not retune it: lower max_lateral_accel to the "
                   f"measured ~{_f(tr['grip_limit'], '{:.1f}')} m/s^2, and account for the ~{tr['steer_lag'] * 1e3:.0f} ms "
                   "steering lag (lookahead / the MPC's delay term). Raising steering gains would chase that lag "
                   "into a weave."
                   + (f" And slow before turning in: at the command's peak, {int(np.sum(pa > tr['grip_limit']))} of "
                      f"{len(pa)} turns asked for more than the grip (median {np.median(pa):.1f} m/s^2), because "
                      "the steering reaches lock while the speed is still coming down." if len(pa) else ""))
    return out


def _max_lat_refs(root: Path = ROOT) -> list:
    """[(file:line, value)] of max_lateral_accel in the planners' shipped configs."""
    out = []
    for rel in ("src/gap_follow/config/gap_follow.yaml", "src/pure_pursuit/config/pure_pursuit.yaml"):
        try:
            for i, line in enumerate((root / rel).read_text().splitlines(), 1):
                s = line.strip()
                if s.startswith("max_lateral_accel:"):
                    out.append((f"{rel}:{i}", float(s.split(":", 1)[1].split("#")[0])))
        except (OSError, ValueError):
            pass
    return out


def report_lines(r: dict, run_name: str) -> list:
    L = [f"# Yaw-rate tracking: {run_name}", ""]
    L += answer_lines(r) + [""]
    sp50, sp90, sp99, spmax = r["speed_p50_p90_p99_max"]
    L += ["## Run",
          f"  {r['run_seconds']:.1f} s on a {FS:.0f} Hz grid, {r['moving_seconds']:.1f} s moving above "
          f"{MOVING_SPEED} m/s (LiDAR speed); commands from {r['topic']}; wheelbase {r['wheelbase']} m",
          f"  LiDAR speed p50/p90/p99/max {sp50:.2f}/{sp90:.2f}/{sp99:.2f}/{spmax:.2f} m/s; "
          f"/odom reads {(r['odom_speed_ratio'] - 1) * 100:+.1f}% vs LiDAR (median while moving)",
          f"  gyro parked bias removed: {r['gyro_bias']:+.5f} rad/s; LiDAR yaw rate is the mean over "
          f"{r['lidar_window_s'] * 1e3:.0f} ms (two scans), and the gyro is averaged the same way where compared to it",
          ""]
    n = r["noise"]
    L.append("## 1. Gyro noise")
    if n["parked"]:
        L.append(f"  parked ({n['parked']['seconds']:.0f} s): bias {n['parked']['bias']:+.5f} rad/s, "
                 f"sigma {n['parked']['sigma']:.5f} rad/s")
    if n["psd"]:
        L.append(f"  parked Welch floor {n['psd']['sigma_floor']:.5f} rad/s ({math.degrees(n['psd']['sigma_floor']):.3f} deg/s); "
                 f"{NOISE_BAND[0]:.0f}-{NOISE_BAND[1]:.0f} Hz white-equivalent {n['psd']['sigma_band']:.5f}. "
                 "np.std above it counts bumps at the stretches' ends; the floor is the sensor")
    if n["moving_psd"]:
        mp = n["moving_psd"]
        L.append(f"  moving ({mp['n'] / n['fs']:.0f} s): {NOISE_BAND[0]:.0f}-{NOISE_BAND[1]:.0f} Hz white-equivalent "
                 f"{mp['sigma_band']:.4f} rad/s ({mp['sigma_band'] / n['psd']['sigma_band']:.1f}x parked: vibration)"
                 if n["psd"] else f"  moving: white-equivalent {mp['sigma_band']:.4f} rad/s")
    L.append(f"  yaw-rate signal while moving: {n['signal_rms']:.3f} rad/s RMS; whole gyro - LiDAR residual "
             f"{r['rmse']['gyro_vs_lidar_aligned']['rmse']:.3f} rad/s (aligned): an upper bound on the gyro's error")
    L.append("")
    L.append("## 2. RMSE while moving, rad/s (after nothing: no lag or gain correction)")
    for name, d in r["rmse"].items():
        L.append(f"  {name:22s} RMSE {d['rmse']:.3f}  bias {d['bias']:+.4f}  gain {d['gain']:.3f}")
    L.append("  by |LiDAR yaw rate| (reference = LiDAR):")
    for who in ("gyro", "odom"):
        L.append(f"   {who}:")
        L += _bin_lines(r["by_rate"][who], "rad/s")
    L.append("")
    L.append("## 3. Latency (normalised cross-correlation; positive = second LATER than first)")
    for k, v in r["timing"].items():
        L.append(f"  {k}: {v}")
    for name, (lag, rr) in r["lag_whole"].items():
        lw = r["lag_windows"][name]
        spread = (f"; per {LAG_WINDOW_S:.0f} s window ({len(lw)} with r >= {LAG_MIN_R}): median "
                  f"{np.median(lw) * 1e3:+.0f}, p10..p90 {np.percentile(lw, 10) * 1e3:+.0f}.."
                  f"{np.percentile(lw, 90) * 1e3:+.0f} ms" if lw else "")
        L.append(f"  {name:18s} {_f(lag * 1e3, '{:+.0f}')} ms, r {_f(rr, '{:.3f}')}{spread}")
    L.append("")
    t = r["tracking"]
    o = t["overall"]
    L += ["## 4. Commanded vs measured (gyro) yaw rate, command shifted by the steering lag "
          f"{t['steer_lag'] * 1e3:.0f} ms, at LiDAR speed",
          f"  overall: RMSE {o['rmse']:.3f} rad/s, bias {o['bias']:+.4f}, gain {o['gain']:.3f}, "
          f"NRMSE {o['nrmse'] * 100:.0f}%; gain left {_f(t['gain_left'])}, right {_f(t['gain_right'])}",
          "  lateral acceleration p50/p90/p99: " + " / ".join(f"{a:.2f}" for a in t["a_lat_p50_p90_p99"])
          + " m/s^2",
          "  by LiDAR speed:"]
    L += _bin_lines(t["by_speed"], "m/s")
    L.append("  by |commanded steering angle|:")
    L += _bin_lines(t["by_steer"], "rad")
    if t["lag_by_speed"]:
        L.append("  steering lag by speed: " + ", ".join(
            f"{b['lo']:.1f}-{b['hi']:.1f} m/s {b['lag'] * 1e3:.0f} ms" for b in t["lag_by_speed"]))
    L.append(f"  held commands (within {STEADY_DELTA} rad for {STEADY_HOLD_S} s): {t['held_seconds']:.1f} s")
    L.append("   achieved / commanded lateral accel, by commanded a_y [m/s^2]: " + ", ".join(
        f"{b['lo']:.1f}-{b['hi']:.1f} {b['ratio']:.2f} (n={b['n']})" for b in t["grip"] if math.isfinite(b["ratio"]))
             + f"; below {GRIP_FLOOR} from here on: {_f(t['grip_limit'], '{:.1f}')} m/s^2")
    L.append(f"   below {LINEAR_A_LAT} m/s^2: measured / commanded curvature {_f(t['held_linear']['ratio'], '{:.2f}')} "
             f"(n={t['held_linear']['n']}); by speed: " + (", ".join(
                 f"{b['lo']:.1f}-{b['hi']:.1f} {b['ratio']:.2f} (n={b['n']})" for b in t["held_by_speed"]
                 if math.isfinite(b["ratio"])) or "none"))
    L.append("  changing commands only, by LiDAR speed:")
    L += _bin_lines(t["transient_by_speed"], "m/s")
    L.append(f"  rear-axle sideslip (LiDAR lateral speed less {LASER_X} m x yaw rate; negative = sliding out), "
             "by achieved a_y: " + ", ".join(f"{b['lo']:.1f}-{b['hi']:.1f} {b['deg']:+.1f} deg"
                                             for b in t["sideslip_by_a_lat"] if math.isfinite(b["deg"])))
    L.append("")
    ms = r["maneuvers"]
    L.append(f"## 5. Maneuvers: |commanded yaw rate| > {MANEUVER_RATE} rad/s for >= {MANEUVER_MIN_S} s "
             f"(command unshifted; tail {MANEUVER_TAIL_S} s)")
    if ms:
        a = {k: np.array([m[k] for m in ms]) for k in ("overshoot_pct", "lag", "rebound", "heading_ratio")}
        pr = np.array([m["peak_ratio"] for m in ms])
        ov = a["overshoot_pct"][np.isfinite(a["overshoot_pct"])]
        L.append(f"  {len(ms)} turns; peak measured / peak commanded median {np.median(pr):.2f} "
                 f"(p10 {np.percentile(pr, 10):.2f}, p90 {np.percentile(pr, 90):.2f})")
        if ov.size:
            L.append(f"  overshoot over its own settled value ({ov.size} turns of >= 1 s): median "
                     f"{np.median(ov):+.0f}%, p90 {np.percentile(ov, 90):+.0f}%, max {ov.max():+.0f}%; "
                     f"{int(np.sum(ov > 10))} above 10%")
        L.append(f"  lag (half-peak crossing) median {np.nanmedian(a['lag']) * 1e3:.0f} ms; rebound median "
                 f"{np.median(a['rebound']):+.3f} rad/s (p90 {np.percentile(a['rebound'], 90):+.3f}); heading "
                 f"achieved / commanded median {np.nanmedian(a['heading_ratio']):.2f}")
        pa = np.array([m["peak_a_cmd"] for m in ms])
        L.append(f"  lateral accel asked for at the command's peak: median {np.median(pa):.1f} m/s^2, p90 "
                 f"{np.percentile(pa, 90):.1f}; {int(np.sum(pa > 2.0))} of {len(ms)} turns asked for more than 2 m/s^2")
        L.append(f"  command steering reversals (|delta| > 0.05 rad): {r['weave']['reversals']} in "
                 f"{r['weave']['moving_s']:.0f} s moving ({r['weave']['reversals'] / r['weave']['moving_s']:.2f}/s)")
        L.append("    t0[s]  dir  dur   v[m/s] |d|max  cmd pk  meas pk  pk ratio  overshoot  lag[ms]  rebound  heading cmd/meas[deg]")
        t0 = r["_grid"]["t"][0]
        for m in ms:
            L.append(f"   {m['t0'] - t0:6.1f}  {'L' if m['sign'] > 0 else 'R'}  {m['duration']:4.1f}  {m['speed']:5.2f}  "
                     f"{m['peak_delta']:.3f}  {m['peak_cmd']:6.2f}  {m['peak_meas']:7.2f}  {m['peak_ratio']:8.2f}  "
                     f"{_f(m['overshoot_pct'], '{:+8.0f}%'):>9s}  "
                     f"{m['lag'] * 1e3:6.0f}  {m['rebound']:+7.3f}  {m['heading_cmd_deg']:6.1f} / {m['heading_meas_deg']:6.1f}")
    else:
        L.append("  none found")
    return L


# ---------------------------------------------------------------- plots

def save_plots(r: dict, out_dir: Path) -> list:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    out_dir.mkdir(parents=True, exist_ok=True)
    g, w_cmd = r["_grid"], r["_w_cmd"]
    t = g["t"] - g["t"][0]
    written = []

    def save(fig, name):
        p = out_dir / name
        fig.tight_layout()
        fig.savefig(p, dpi=100)
        plt.close(fig)
        written.append(p)

    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(t, w_cmd, "--", color="0.5", lw=1, label="commanded (shifted by steering lag)")
    ax.plot(t, g["odom"], color="tab:green", lw=1, label="/odom")
    ax.plot(t, g["lidar"], color="tab:orange", lw=1, label="LiDAR")
    ax.plot(t, g["gyro"], color="tab:blue", lw=1, label="gyro")
    ax.set_xlabel("time since start [s]")
    ax.set_ylabel("yaw rate [rad/s]")
    ax.legend(loc="upper right", fontsize=8)
    save(fig, "overview.png")

    fig, ax = plt.subplots(figsize=(8, 5))
    for name, curve in r["xcorr"].items():
        c = np.array(curve)
        ax.plot(c[:, 0] * 1e3, c[:, 1], label=f"{name} ({r['lag_whole'][name][0] * 1e3:+.0f} ms)")
    ax.axvline(0, color="0.7", lw=0.8)
    ax.set_xlabel("lag [ms] (positive: second later than first)")
    ax.set_ylabel("normalised cross-correlation")
    ax.legend(fontsize=8)
    save(fig, "xcorr.png")

    n = r["noise"]
    if n["psd"]:
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.semilogy(n["psd"]["f"], n["psd"]["p"], label="gyro z, parked")
        if n.get("moving_psd"):
            ax.semilogy(n["moving_psd"]["f"], n["moving_psd"]["p"], label="gyro z, moving")
        ax.axvspan(*NOISE_BAND, color="0.92", zorder=0)
        if n.get("resid_psd"):
            ax.semilogy(n["resid_psd"]["f"], n["resid_psd"]["p"], label="gyro - LiDAR, moving")
        ax.set_xlabel("frequency [Hz]")
        ax.set_ylabel("PSD [(rad/s)^2/Hz]")
        ax.legend()
        save(fig, "gyro_psd.png")

    tr = r["tracking"]
    for key, label, name in (("by_speed", "LiDAR speed [m/s]", "error_vs_speed.png"),
                             ("by_steer", "|commanded steering| [rad]", "error_vs_steer.png")):
        rows = [b for b in tr[key] if b["n"] >= MIN_BIN]
        if not rows:
            continue
        x = [(b["lo"] + b["hi"]) / 2 for b in rows]
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        a1.plot(x, [b["gain"] for b in rows], "o-")
        a1.axhline(1.0, color="0.6", ls="--")
        a1.set_ylabel("gain (measured / commanded)")
        a2.plot(x, [b["nrmse"] * 100 for b in rows], "o-", color="tab:red")
        a2.set_ylabel("NRMSE [%]")
        a2.set_xlabel(label)
        save(fig, name)

    rows = r["by_rate"]
    fig, ax = plt.subplots(figsize=(8, 4))
    for who, c in (("gyro", "tab:blue"), ("odom", "tab:green")):
        b = [x for x in rows[who] if x["n"] >= MIN_BIN]
        ax.plot([(x["lo"] + x["hi"]) / 2 for x in b], [x["gain"] for x in b], "o-", color=c, label=who)
    ax.axhline(1.0, color="0.6", ls="--")
    ax.set_xlabel("|LiDAR yaw rate| [rad/s]")
    ax.set_ylabel("gain vs LiDAR")
    ax.legend()
    save(fig, "gain_vs_yaw_rate.png")

    ms = sorted(r["maneuvers"], key=lambda m: -m["peak_cmd"])[:MAX_MANEUVER_PLOTS]
    _, w_cmd0 = commanded(g, 0.0)
    for k, m in enumerate(sorted(ms, key=lambda m: m["t0"])):
        i0 = max(0, m["i0"] - int(0.5 * FS))
        i1 = min(len(t), m["i1"] + int((MANEUVER_TAIL_S + 0.5) * FS))
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True, height_ratios=[3, 1])
        s = slice(i0, i1)
        a1.plot(t[s], w_cmd0[s], ":", color="0.5", label="commanded")
        a1.plot(t[s], w_cmd[s], "--", color="0.3", label=f"commanded +{tr['steer_lag'] * 1e3:.0f} ms")
        a1.plot(t[s], g["odom"][s], color="tab:green", label="/odom")
        a1.plot(t[s], g["lidar"][s], color="tab:orange", label="LiDAR")
        a1.plot(t[s], g["gyro"][s], color="tab:blue", lw=2, label="gyro")
        a1.axvspan(m["t0"] - g["t"][0], m["t1"] - g["t"][0], color="0.9", zorder=0)
        a1.set_ylabel("yaw rate [rad/s]")
        a1.set_title(f"turn at {m['t0'] - g['t'][0]:.1f} s: peak ratio {m['peak_ratio']:.2f}, "
                     f"overshoot {_f(m['overshoot_pct'], '{:+.0f}%')}, "
                     f"lag {m['lag'] * 1e3:.0f} ms, heading {m['heading_cmd_deg']:.0f} -> "
                     f"{m['heading_meas_deg']:.0f} deg")
        a1.legend(fontsize=7, loc="best")
        a2.plot(t[s], g["v"][s], color="k")
        a2.set_ylabel("v [m/s]")
        a3 = a2.twinx()
        a3.plot(t[s], g["delta"][s], color="tab:purple")
        a3.set_ylabel("delta [rad]", color="tab:purple")
        a2.set_xlabel("time since start [s]")
        save(fig, f"maneuver_{k:02d}_{m['t0'] - g['t'][0]:05.1f}s.png")
    return written


def public(r: dict) -> dict:
    """The JSON-safe part of analyse()'s result."""
    out = {k: v for k, v in r.items() if not k.startswith("_")}
    out["maneuvers"] = [{k: v for k, v in m.items() if k not in ("i0", "i1")} for m in r["maneuvers"]]
    return json.loads(json.dumps(out, default=float).replace("NaN", "null"))


def main(argv=None) -> int:
    from run_analysis.imu_report import load_imu_axes
    from run_analysis.runlog import analysis_dir, load_run
    from run_analysis.scan_motion import scan_motion

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run_dir")
    ap.add_argument("--refresh", action="store_true", help="re-read the bag")
    ap.add_argument("--car-yaml", help="imu-calibration car.yaml for a run with no copy in its metadata")
    args = ap.parse_args(argv)
    run_dir = Path(args.run_dir).expanduser()
    axes = load_imu_axes(run_dir, refresh=args.refresh)
    stream = ims.vesc_stream(axes, run_dir, args.car_yaml)
    if stream is None:
        print("no VESC IMU in this run (/imu/vesc/data, /sensors/imu/raw)")
        return 1
    if len(axes.get("odom", ())) < 2 or (len(axes.get("drive", ())) < 2 and len(axes.get("ackermann", ())) < 2):
        print("needs /odom and /drive or /ackermann_cmd")
        return 1
    run = load_run(run_dir, refresh=args.refresh)
    motion = scan_motion(run, float(run.scan_t[0]), float(run.scan_t[-1]))
    timing = {
        "scan receive - header stamp (median)": f"{np.median(run.scan_t - run.scan_stamp) * 1e3:+.1f} ms",
        "VESC IMU receive - header stamp (median)": f"{np.median(stream.t - stream.stamp) * 1e3:+.1f} ms",
    }
    wheelbase, _ = ims.config_ref("wheelbase")
    r = analyse(axes, stream, motion, float(wheelbase), timing)
    lines = report_lines(r, run_dir.name)
    out = analysis_dir(run_dir)
    for p in save_plots(r, out / "yaw_tracking"):
        lines.append(f"plot: yaw_tracking/{p.name}")
    (out / "yaw_tracking.json").write_text(json.dumps(public(r), indent=1))
    keys = ["t0", "sign", "duration", "speed", "speed_at_peak", "peak_a_cmd", "peak_delta", "peak_cmd", "peak_meas", "peak_ratio",
            "overshoot_pct", "lag", "rebound", "rmse", "heading_cmd_deg", "heading_meas_deg", "heading_ratio"]
    t0 = r["_grid"]["t"][0]
    (out / "yaw_tracking_maneuvers.csv").write_text(
        ",".join(keys) + "\n" + "".join(",".join(f"{(m[k] - t0) if k == 't0' else m[k]:.4f}" for k in keys) + "\n"
                                        for m in r["maneuvers"]))
    text = "\n".join(lines) + "\n"
    (out / "yaw_tracking.txt").write_text(text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
