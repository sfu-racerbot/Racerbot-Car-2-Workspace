"""IMU noise, vibration, delay and steering response from one recorded run.

Four questions, each answered next to the value the workspace assumes
today and the file:line it lives at:

1. Bias and noise: in every still stretch (imu_bridge.convert.is_still,
   with /odom's speed), each axis' mean (the bias) and its spread.
2. Vibration: at each steady motor speed (/sensors/core ERPM held for a
   few seconds -- a wheels-lifted run), the spectrum of every axis, its
   peaks, and whether each peak follows the motor or folds back from above
   Nyquist (an alias of vibration the 50 Hz poll cannot represent).
3. IMU delay and gyro scale: the gyro's yaw rate against LiDAR scan
   matching (plan P0.5): the lag that best aligns them, and the gain.
4. Steering response (weave runs): commanded yaw rate v tan(delta) / L
   (/ackermann_cmd, or /drive) to the gyro's, as gain and phase per
   frequency where coherence allows, with a delay, a first-order fit and a
   bandwidth -- a plant model an MPC can take directly.

    python3 tools/run_analysis/imu_spectrum.py ~/racerbot-ws/data/racerbot_runs/<ts>
    python3 tools/run_analysis/imu_spectrum.py <run> --vesc-rate 200 --accel-hz 30 --gyro-hz 50
    python3 tools/run_analysis/imu_spectrum.py <run> --no-icp      # skip LiDAR (sections 3, 4b)

The first run needs ROS sourced to read the bag; arrays are cached in
<run>/analysis/ and the rest runs unsourced. Writes imu_spectrum.txt,
steering_frf.csv and PNGs into <run>/analysis/.

The VESC IMU is used as /imu/vesc/data (calibrated, car axes) when the run
has it. Otherwise /sensors/imu/raw is converted (imu_bridge.convert) and
rotated with the copy of imu-calibration's car.yaml the recorder saved in
<run>/metadata/files/; without that copy, only gyro z is reported. The
BNO085's sections say "not in this run" when /imu/bno/data is absent.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT / "tools", ROOT / "src" / "imu_bridge"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from imu_bridge import convert  # noqa: E402
from run_analysis import actuation  # noqa: E402
from run_analysis import spectrum as sp  # noqa: E402

# Stillness, as imu_bridge's startup bias judges it (convert.StartupBias).
STILL_SPEED = 0.03       # m/s
STILL_GYRO = 0.05        # rad/s
MIN_STILL_S = 5.0        # a still stretch shorter than this is not used
# A steady motor speed: every ERPM sample within this fraction of the mean for MIN_STEADY_S.
STEADY_FRACTION = 0.03
MIN_STEADY_S = 5.0
MIN_STEADY_ERPM = 500.0
# Moving, for delay and steering fits (imu_report's FIT_MIN_SPEED).
MOVING_SPEED = 0.5       # m/s
DELAY_LAGS = np.arange(-0.2, 0.3001, 0.005)      # s, both signs: the IMU may lead the LiDAR
STEER_LAGS = np.arange(0.0, 0.6001, 0.005)       # s
DOCUMENTED_STEER_LAG = 0.15   # s, run 20261019-200703 (docs/run-analysis.md)
SITE_VESC_LATENCY = 0.020     # s, "the serial round trip, about 20 ms" (imu_bridge README)
NEURAL_CONTROL_DT = 0.025     # s, gpu_sim/params.py CONTROL_DT: what imu_delay_ticks counts

# Config values the findings are compared against: (file relative to the
# workspace, the text that starts the line).
CONFIG = {
    "gyro_sigma": ("src/imu_bridge/config/imu_bridge.yaml", "gyro_sigma:"),
    "accel_sigma": ("src/imu_bridge/config/imu_bridge.yaml", "accel_sigma:"),
    "sigma_accel": ("src/imu_bridge/imu_bridge/vehicle_state.py", "sigma_accel: float ="),
    "imu_gyro_noise": ("tools/neural_racer/neural_racer_train/gpu_sim/params.py", "imu_gyro_noise:"),
    "imu_accel_noise": ("tools/neural_racer/neural_racer_train/gpu_sim/params.py", "imu_accel_noise:"),
    "imu_gyro_bias": ("tools/neural_racer/neural_racer_train/gpu_sim/params.py", "imu_gyro_bias:"),
    "imu_delay_ticks": ("tools/neural_racer/neural_racer_train/gpu_sim/params.py", "imu_delay_ticks:"),
    "wheelbase": ("src/f1tenth_system/f1tenth_stack/config/vesc.yaml", "wheelbase:"),
}


def config_ref(key: str, root: Path = ROOT):
    """(value, 'file:line') of a config value, read from the file now, so the
    report never quotes a stale number. (None, reason) if it is not found."""
    rel, start = CONFIG[key]
    path = root / rel
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return None, f"{rel} not found"
    for i, line in enumerate(lines, 1):
        s = line.strip()
        if s.startswith(start):
            raw = s[len(start):].split("#")[0].strip()
            if "=" in raw:                  # a dataclass field: `name: float = 0.15`
                raw = raw.split("=", 1)[1].strip()
            try:
                return float(raw), f"{rel}:{i}"
            except ValueError:
                return raw, f"{rel}:{i}"
    return None, f"{start!r} not in {rel}"


# ---------------------------------------------------------------- streams

@dataclass
class Stream:
    name: str
    label: str
    t: np.ndarray            # receive time [s]
    stamp: np.ndarray        # header stamp [s]
    acc: np.ndarray | None   # (N, 3) car axes, m/s^2; None when the axes are unknown
    gyro: np.ndarray         # (N, 3) rad/s; only column 2 is meaningful when acc is None

    @property
    def axes_known(self) -> bool:
        return self.acc is not None


def apply_rows(cal, acc, gyro):
    """convert.Calibration.apply over (N, 3) rows."""
    acc_cal = (acc - (cal.acc_bias - cal.dev_acc_offset)) @ cal.A_inv.T
    gyro_cal = gyro - (cal.gyro_bias - cal.dev_gyro_offset)
    return acc_cal @ cal.R.T, gyro_cal @ cal.R.T


def vesc_stream(axes: dict, run_dir=None):
    """The VESC IMU, best available form, or None. See the module docstring."""
    cal_rows = axes.get("vesc", np.zeros((0, 8)))
    if len(cal_rows) >= 2:
        return Stream("vesc", "/imu/vesc/data (imu_bridge: calibrated, car axes)",
                      cal_rows[:, 0], cal_rows[:, 1], cal_rows[:, 2:5], cal_rows[:, 5:8])
    raw = axes.get("vesc_raw", np.zeros((0, 8)))
    if len(raw) < 2:
        return None
    acc, gyro = convert.vesc_raw_to_si(raw[:, 2:5], raw[:, 5:8])
    car_yaml = Path(run_dir).expanduser() / "metadata" / "files" / "car.yaml" if run_dir else None
    if car_yaml is None or not car_yaml.is_file():
        return Stream("vesc", "/sensors/imu/raw, board axes (no car.yaml in the run's "
                      "metadata): gyro z only, accelerometer axes unknown",
                      raw[:, 0], raw[:, 1], None, gyro)
    rpy, calib = convert.load_mount(car_yaml, "vesc")
    copy = car_yaml.parent / calib.name if calib is not None else None
    cal = convert.load_calibration(copy if copy is not None and copy.is_file() else None, rpy)
    acc, gyro = apply_rows(cal, acc, gyro)
    how = ("calibrated with the run's " + copy.name) if cal.source != convert.IDENTITY_SOURCE \
        else "uncalibrated"
    return Stream("vesc", f"/sensors/imu/raw rotated with the run's car.yaml (rpy_deg {rpy}), "
                  f"{how}", raw[:, 0], raw[:, 1], acc, gyro)


def bno_stream(axes: dict):
    rows = axes.get("bno", np.zeros((0, 8)))
    if len(rows) < 2:
        return None
    return Stream("bno", "/imu/bno/data (imu_bridge: calibrated, car axes)",
                  rows[:, 0], rows[:, 1], rows[:, 2:5], rows[:, 5:8])


def speed_at(axes: dict, t) -> np.ndarray:
    """/odom speed at receive times t; NaN where there is no /odom near."""
    odom = axes.get("odom", np.zeros((0, 7)))
    t = np.asarray(t, dtype=float)
    if len(odom) < 2:
        return np.full(len(t), math.nan)
    v = np.interp(t, odom[:, 0], odom[:, 2])
    v[(t < odom[0, 0]) | (t > odom[-1, 0])] = math.nan
    return v


def runs_of(mask, t, min_s: float) -> list:
    """[(i0, i1)] index ranges where mask holds continuously for >= min_s."""
    out, start = [], None
    for i, m in enumerate(np.r_[mask, False]):
        if m and start is None:
            start = i
        elif not m and start is not None:
            if t[i - 1] - t[start] >= min_s:
                out.append((start, i))
            start = None
    return out


# ---------------------------------------------------------------- 1. bias and noise

def still_segments(stream: Stream, axes: dict, min_s: float = MIN_STILL_S) -> list:
    v = speed_at(axes, stream.t)
    still = np.array([convert.is_still(g, s, STILL_SPEED, STILL_GYRO)
                      for g, s in zip(stream.gyro, v)], dtype=bool)
    return runs_of(still, stream.t, min_s)


def bias_noise(stream: Stream, axes: dict):
    """(still segments, {('gyro'|'accel', 'x'|'y'|'z'): (dc, sigma, sigma_std)})
    over every still stretch together. Taken on the samples as logged, not
    resampled: interpolating onto a grid averages neighbours and would
    under-read the noise, and neither number needs a frequency axis. Only
    gyro z when the stream's axes are unknown."""
    segs = still_segments(stream, axes)
    if not segs:
        return segs, {}
    idx = np.concatenate([np.arange(a, b) for a, b in segs])
    out = {}
    for j in ((0, 1, 2) if stream.axes_known else (2,)):
        out[("gyro", "xyz"[j])] = sp.dc_and_sigma(stream.gyro[idx, j])[:3]
    if stream.axes_known:
        for j in range(3):
            out[("accel", "xyz"[j])] = sp.dc_and_sigma(stream.acc[idx, j])[:3]
    return segs, out


def bias_noise_lines(stream: Stream, axes: dict, root: Path = ROOT) -> list:
    """Section 1's lines: bias_noise next to imu_bridge.yaml's sigmas."""
    lines = [f"## 1. Bias and noise: {stream.name} ({stream.label})"]
    if len(axes.get("odom", ())) < 2:
        return lines + ["  no /odom in this run: stillness cannot be judged, so no bias or noise"]
    segs, stats = bias_noise(stream, axes)
    if not segs:
        return lines + [f"  no still stretch of {MIN_STILL_S:.0f} s or more (|v| < {STILL_SPEED} m/s "
                        f"and every gyro axis < {STILL_GYRO} rad/s): record run A, parked"]
    total = sum(stream.t[b - 1] - stream.t[a] for a, b in segs)
    lines.append(f"  {len(segs)} still stretch(es), {total:.1f} s, "
                 f"{sum(b - a for a, b in segs)} samples")
    gyro_sigma, gyro_ref = config_ref("gyro_sigma", root)
    accel_sigma, accel_ref = config_ref("accel_sigma", root)
    for (sensor, axis), (dc, sigma, std) in stats.items():
        if sensor == "gyro":
            lines.append(f"  gyro {axis}: bias {dc:+.5f} rad/s, noise {sigma:.5f} rad/s "
                         f"(np.std {std:.5f})   vs gyro_sigma {gyro_sigma} at {gyro_ref}")
        else:
            what = (f"reads {dc:+.4f} (gravity {convert.G}: off by {dc - convert.G:+.4f})"
                    if axis == "z" else f"bias {dc:+.4f}")
            lines.append(f"  accel {axis}: {what} m/s^2, noise {sigma:.4f} m/s^2 "
                         f"(np.std {std:.4f})   vs accel_sigma {accel_sigma} at {accel_ref}")
    if not stream.axes_known:
        lines.append("  accelerometer: axes unknown (no car.yaml in the run's metadata)")
    if len(segs) >= 2:
        biases = [float(np.mean(stream.gyro[a:b, 2])) for a, b in segs]
        lines.append(f"  gyro z bias across stretches: {', '.join(f'{b:+.5f}' for b in biases)} "
                     f"rad/s (spread {np.ptp(biases):.5f}): its stability between stops")
    return lines


# ---------------------------------------------------------------- 2. vibration

def steady_bins(axes: dict) -> list:
    """[(t0, t1, erpm)] where the motor held a steady, non-trivial speed."""
    core = axes.get("core", np.zeros((0, 8)))
    if len(core) < 10:
        return []
    t, erpm = core[:, 0], core[:, 2]
    out = []
    i = 0
    n = len(t)
    while i < n:
        j = i
        while j + 1 < n and t[j + 1] - t[i] <= MIN_STEADY_S:
            j += 1
        win = erpm[i:j + 1]
        mean = float(np.mean(win))
        # Every sample within the band, not just the spread: one sample from
        # the previous speed would otherwise pass and blur the bin.
        if (t[j] - t[i] >= 0.9 * MIN_STEADY_S and abs(mean) >= MIN_STEADY_ERPM
                and float(np.max(np.abs(win - mean))) <= STEADY_FRACTION * abs(mean)):
            out.append((float(t[i]), float(t[j]), mean))
            i = j + 1
        else:
            i += 1
    return out


def vesc_filter_gain(cutoff_hz: float, rate_hz: float, at_hz: float, racer_imu_root):
    """|H| of the VESC firmware's IMU low-pass at at_hz, using the biquad in
    imu-calibration's tools/imu_filter_plot.py (imported, not copied), or
    None if that file is not there. A cutoff of 0 is the filter off: 1.0."""
    if cutoff_hz <= 0.0:
        return 1.0
    tools = Path(racer_imu_root).expanduser() / "tools"
    if not (tools / "imu_filter_plot.py").is_file():
        return None
    if str(tools) not in sys.path:
        sys.path.insert(0, str(tools))
    from imu_filter_plot import vesc_biquad
    from scipy import signal
    b, a = vesc_biquad(cutoff_hz, rate_hz)
    _, h = signal.freqz(b, a, worN=[at_hz], fs=rate_hz)
    return float(abs(h[0]))


def vibration_lines(stream: Stream, axes: dict, filt=None, racer_imu_root="~/imu-calibration",
                    root: Path = ROOT) -> tuple:
    """(lines, plot data). filt: None, or dict(rate=, accel=[x, y, z], gyro=)."""
    lines = [f"## 2. Vibration: {stream.name}"]
    stats = sp.stream_stats(stream.stamp)
    fs = stats.rate_hz
    bins = steady_bins(axes)
    if not math.isfinite(fs):
        return lines + ["  no rate: too few samples"], []
    if not bins:
        return lines + [f"  no steady motor speed of {MIN_STEADY_S:.0f} s (/sensors/core ERPM within "
                        f"{STEADY_FRACTION:.0%}, over {MIN_STEADY_ERPM:.0f}): record run B, wheels "
                        "lifted, at 3-4 steady throttles"], []
    nyq = fs / 2.0
    lines.append(f"  sampled at {fs:.1f} Hz (header stamps): Nyquist {nyq:.1f} Hz. Peaks are "
                 "compared with the motor's electrical frequency ERPM/60 and its harmonics")
    channels = ([("accel " + a, stream.acc[:, j]) for j, a in enumerate("xyz")]
                if stream.axes_known else []) + [("gyro z", stream.gyro[:, 2])]
    plot, track = [], {}
    flags = []
    for t0, t1, erpm in bins:
        ref = abs(erpm) / 60.0
        sel = (stream.t >= t0) & (stream.t <= t1)
        lines.append(f"  motor {erpm:.0f} ERPM ({ref:.1f} Hz electrical), {t1 - t0:.1f} s:")
        for name, x in channels:
            pieces = sp.resample_uniform(stream.stamp[sel], x[sel], fs, max_gap_s=3.0 / fs)
            if not pieces:
                continue
            tu, xu = max(pieces, key=lambda p: len(p[0]))
            got = sp.psd(xu, fs, segment_s=min(4.0, (tu[-1] - tu[0]) / 2))
            if got is None:
                continue
            f, P = got
            res = f[1] - f[0]
            sigma = sp.band_sigma(f, P, 0.5, nyq)
            peaks = sp.find_peaks(f, P, max_peaks=3)
            desc = []
            for pf, _ in peaks:
                kind, k = sp.classify_peak(pf, ref, fs, tol_hz=2 * res)
                desc.append(f"{pf:.1f} Hz {kind}{'' if k is None else f' (x{k})'}")
                if kind == "aliased":
                    flags.append("aliased")
            if peaks:
                track.setdefault(name, []).append((ref, peaks[0][0]))
            lines.append(f"    {name:8s} band 0.5-{nyq:.0f} Hz: {sigma:.4f} RMS; peaks: "
                         f"{', '.join(desc) or 'none above 6 dB'}")
            plot.append((name, erpm, f, P))
    if track:
        lines.append("  strongest peak per channel across speeds:")
        for name, pts in track.items():
            slope, verdict = sp.peak_trend([p[0] for p in pts], [p[1] for p in pts])
            slope_text = f" (slope {slope:+.2f})" if math.isfinite(slope) else ""
            lines.append(f"    {name:8s} {verdict}{slope_text}")
            if verdict == "backwards":
                flags.append("backwards")
    if stream.axes_known:
        sigma_accel, sa_ref = config_ref("sigma_accel", root)
        noise, n_ref = config_ref("imu_accel_noise", root)
        lines.append(f"  compare: vehicle_state's sigma_accel {sigma_accel} m/s per sqrt(s) at {sa_ref} "
                     "assumes about 1 m/s^2 rms vibration (its comment); per-sample accel noise "
                     f"sigma at {fs:.0f} Hz equals sigma * sqrt(dt) = sigma * {math.sqrt(1 / fs):.3f} "
                     f"m/s per sqrt(s). neural_racer's imu_accel_noise {noise} at {n_ref}")
    if filt is None:
        lines.append("  VESC low-pass: settings not given (--vesc-rate, --accel-hz, --gyro-hz from "
                     "VESC Tool -> App Settings -> IMU), so how much it passes at Nyquist is unknown")
    else:
        for label, hz in [(f"accel {a}", h) for a, h in zip("xyz", filt["accel"])] + [("gyro", filt["gyro"])]:
            g = vesc_filter_gain(hz, filt["rate"], nyq, racer_imu_root)
            if g is None:
                lines.append(f"  VESC low-pass {label}: imu-calibration's tools/imu_filter_plot.py not "
                             f"found under {racer_imu_root}")
            else:
                lines.append(f"  VESC low-pass {label} ({'off' if hz <= 0 else f'{hz:g} Hz'} at "
                             f"{filt['rate']:g} Hz): passes {20 * math.log10(max(g, 1e-12)):+.1f} dB "
                             f"at our Nyquist {nyq:.1f} Hz")
                if g > 0.1:
                    flags.append("filter passes")
    if "aliased" in flags or "backwards" in flags:
        lines.append(f"  evidence: vibration above {nyq:.0f} Hz is folding into the data. Lower the "
                     "VESC's IMU low-pass cutoff below Nyquist, or poll the IMU faster")
    elif "filter passes" in flags:
        lines.append("  evidence: no alias seen in these bins, but the VESC filter leaves room for one "
                     "at higher motor speeds")
    else:
        lines.append("  evidence: no sign of aliasing in these bins")
    return lines, plot


# ---------------------------------------------------------------- 3. IMU delay and gyro scale

def imu_delay_and_scale(t_imu, w_imu, t_ref, w_ref, lags=DELAY_LAGS, valid=None):
    """(delay, scale, rms): the lag d minimising RMS(imu(t) - ref(t - d))
    (actuation.best_lag), positive when the IMU is LATER than the
    reference; then the least-squares gain imu ~ scale * ref(t - d)."""
    delay, rms = actuation.best_lag(t_imu, w_imu, t_ref, w_ref, lags, valid)
    if not math.isfinite(delay):
        return math.nan, math.nan, math.nan
    ref = np.interp(np.asarray(t_imu) - delay, t_ref, w_ref)
    imu = np.asarray(w_imu, dtype=float)
    if valid is not None:
        ref, imu = ref[valid], imu[valid]
    den = float(np.sum(ref * ref))
    return delay, (float(np.sum(imu * ref) / den) if den > 0 else math.nan), rms


def delay_lines(stream: Stream, axes: dict, motion, root: Path = ROOT) -> list:
    lines = [f"## 3. IMU delay and gyro scale: {stream.name} gyro z vs LiDAR scan matching"]
    if motion is None or len(motion.t) < 20:
        return lines + ["  no LiDAR motion (--no-icp, or no /scan): not measured"]
    inside = (stream.t >= motion.t[0]) & (stream.t <= motion.t[-1])
    moving = inside & (np.abs(speed_at(axes, stream.t)) > MOVING_SPEED)
    if moving.sum() < 50:
        return lines + [f"  only {int(moving.sum())} gyro samples above {MOVING_SPEED} m/s inside the "
                        "LiDAR's span: drive (run C) to measure it"]
    delay, scale, rms = imu_delay_and_scale(stream.t, stream.gyro[:, 2], motion.t, motion.w,
                                            valid=moving)
    ticks, t_ref = config_ref("imu_delay_ticks", root)
    lines += [
        f"  delay {delay * 1e3:+.0f} ms (gyro later than LiDAR when positive), scale {scale:.3f}, "
        f"RMS {rms:.3f} rad/s at that lag, {int(moving.sum())} samples",
        "  relative to the LiDAR, whose own latency is not removed (scan_motion uses receive times)",
        f"  vs the documented VESC latency ~{SITE_VESC_LATENCY * 1e3:.0f} ms (stamped on arrival) and "
        f"neural_racer's imu_delay_ticks {ticks} x {NEURAL_CONTROL_DT * 1e3:.0f} ms at {t_ref}",
    ]
    return lines


# ---------------------------------------------------------------- 4. steering response

def commanded_yaw_rate(t, axes: dict, wheelbase: float):
    """(rate, topic): v tan(delta) / L at times t, from /drive if the run has
    it, else /ackermann_cmd, with v from /odom. (None, None) without both."""
    for name, topic in (("drive", "/drive"), ("ackermann", "/ackermann_cmd")):
        cmds = axes.get(name, np.zeros((0, 4)))
        if len(cmds) >= 2:
            break
    else:
        return None, None
    v = speed_at(axes, t)
    delta = np.interp(t, cmds[:, 0], cmds[:, 2])
    return v * np.tan(delta) / wheelbase, topic


def steering_response(t_meas, w_meas, axes: dict, wheelbase: float, fs: float):
    """(FrequencyResponse or None, topic, lag, pieces) for commanded -> measured
    yaw rate over the stretches where the car moved above MOVING_SPEED."""
    t_meas = np.asarray(t_meas, dtype=float)
    pieces = sp.resample_uniform(t_meas, np.asarray(w_meas, dtype=float), fs, max_gap_s=3.0 / fs)
    pairs, lag_t, lag_m, lag_c = [], [], [], []
    topic = None
    for tu, wu in pieces:
        cmd, topic = commanded_yaw_rate(tu, axes, wheelbase)
        if cmd is None:
            return None, None, math.nan, 0
        moving = np.abs(speed_at(axes, tu)) > MOVING_SPEED
        for a, b in runs_of(moving, tu, 20.0):
            pairs.append((cmd[a:b], wu[a:b]))
            lag_t.append(tu[a:b])
            lag_m.append(wu[a:b])
            lag_c.append(cmd[a:b])
    if not pairs:
        return None, topic, math.nan, 0
    fr = sp.frequency_response_pieces(pairs, fs, segment_s=10.0)
    t_all, m_all, c_all = np.concatenate(lag_t), np.concatenate(lag_m), np.concatenate(lag_c)
    lag, _ = actuation.best_lag(t_all, m_all, t_all, c_all, STEER_LAGS)
    return fr, topic, lag, len(pairs)


def steering_lines(stream: Stream, axes: dict, motion, root: Path = ROOT):
    """(lines, fr, csv rows)."""
    lines = ["## 4. Steering response: commanded yaw rate v tan(delta) / L -> measured yaw rate"]
    wheelbase, wb_ref = config_ref("wheelbase", root)
    if not isinstance(wheelbase, float):
        return lines + [f"  wheelbase not found ({wb_ref})"], None, []
    lines.append(f"  wheelbase {wheelbase} m from {wb_ref}; coherence threshold "
                 f"{sp.COHERENCE_MIN} (below it a bin is not reported)")
    fs = sp.stream_stats(stream.t).rate_hz
    sources = [(f"{stream.name} gyro", stream.t, stream.gyro[:, 2], fs)]
    if motion is not None and len(motion.t) >= 20:
        sources.append(("LiDAR", motion.t, motion.w, sp.stream_stats(motion.t).rate_hz))
    first_fr, csv = None, []
    for name, t, w, rate in sources:
        if not math.isfinite(rate):
            continue
        fr, topic, lag, n = steering_response(t, w, axes, wheelbase, rate)
        if topic is None:
            return lines + ["  no /drive or /ackermann_cmd in this run: no commanded yaw rate"], None, []
        if fr is None or not fr.valid.any():
            lines.append(f"  {name}: no estimate (needs 20 s or more of moving weaves above "
                         f"{MOVING_SPEED} m/s with coherence {sp.COHERENCE_MIN}; record run C)")
            continue
        fit = sp.fit_first_order_delay(fr)
        slope_delay = sp.delay_from_phase(fr)
        bw = sp.bandwidth_hz(fr)
        lines.append(f"  measured by {name} ({n} moving stretch(es), commands from {topic}):")
        for f, g, ph, c in fr.rows():
            lines.append(f"    {f:5.2f} Hz  gain {g:5.3f}  phase {ph:+7.1f} deg  coherence {c:.2f}")
            csv.append((name, f, g, ph, c))
        lines.append(f"    delay from phase slope {slope_delay:.3f} s; best_lag {lag:.3f} s; "
                     f"documented {DOCUMENTED_STEER_LAG} s (run 20261019-200703)"
                     if slope_delay is not None else f"    best_lag {lag:.3f} s")
        if fit is not None:
            lines.append(f"    plant: yaw_rate = {fit['gain']:.3f} e^(-{fit['delay']:.3f} s) / "
                         f"(1 + {fit['time_constant']:.3f} s) x v tan(delta) / L")
        lines.append(f"    bandwidth (-3 dB) {bw:.2f} Hz" if bw is not None
                     else "    bandwidth: gain never fell 3 dB inside the coherent band")
        first_fr = first_fr or fr
    return lines, first_fr, csv


# ---------------------------------------------------------------- report

def stream_lines(axes: dict) -> list:
    lines = ["## Streams (header stamps; receive times in brackets)"]
    for name, topic in (("vesc_raw", "/sensors/imu/raw"), ("vesc", "/imu/vesc/data"),
                        ("bno", "/imu/bno/data"), ("bno_raw", "/imu/bno/raw")):
        rows = axes.get(name, np.zeros((0, 8)))
        if len(rows) < 2:
            lines.append(f"  {topic:18s} not in this run")
            continue
        s, r = sp.stream_stats(rows[:, 1]), sp.stream_stats(rows[:, 0])
        lines.append(f"  {topic:18s} {s.line()} [{r.rate_hz:.1f} Hz, dt std {r.dt_std_s * 1e3:.1f} ms]")
    return lines


def build_report(run_dir, axes: dict, motion=None, filt=None,
                 racer_imu_root="~/imu-calibration", root: Path = ROOT):
    """(lines, plots, csv rows) for one run. motion: scan_motion.Motion or None."""
    lines = [f"# IMU spectrum: {Path(run_dir).name if run_dir else 'run'}", ""]
    lines += stream_lines(axes) + [""]
    plots, csv = [], []
    streams = [s for s in (vesc_stream(axes, run_dir), bno_stream(axes)) if s is not None]
    if not streams:
        return lines + ["no IMU in this run (/imu/vesc/data, /sensors/imu/raw, /imu/bno/data)"], [], []
    if bno_stream(axes) is None:
        lines += ["BNO085: not in this run (no /imu/bno/data); every section below is the VESC's", ""]
    for stream in streams:
        lines += bias_noise_lines(stream, axes, root) + [""]
        vib, data = vibration_lines(stream, axes, filt, racer_imu_root, root)
        lines += vib + [""]
        plots += [(stream.name, *d) for d in data]
        lines += delay_lines(stream, axes, motion, root) + [""]
    steer, fr, csv = steering_lines(streams[-1] if len(streams) > 1 else streams[0], axes, motion, root)
    lines += steer
    return lines, plots, csv


def _save_plots(out_dir: Path, plots, csv) -> list:
    written = []
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return written
    for stream in sorted({p[0] for p in plots}):
        fig, ax = plt.subplots(figsize=(10, 6))
        for name, channel, erpm, f, P in [p for p in plots if p[0] == stream]:
            ax.semilogy(f, P, lw=1, label=f"{channel} @ {erpm:.0f} ERPM")
        ax.set_xlabel("frequency [Hz]")
        ax.set_ylabel("PSD [unit^2/Hz]")
        ax.set_title(f"{stream} IMU vibration spectra")
        ax.legend(fontsize=7)
        path = out_dir / f"imu_psd_{stream}.png"
        fig.savefig(path, dpi=100)
        plt.close(fig)
        written.append(path)
    if csv:
        fig, (ag, ap) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        for src in sorted({r[0] for r in csv}):
            rows = [r for r in csv if r[0] == src]
            ag.plot([r[1] for r in rows], [r[2] for r in rows], "o-", ms=3, label=src)
            ap.plot([r[1] for r in rows], [r[3] for r in rows], "o-", ms=3, label=src)
        ag.set_ylabel("gain")
        ap.set_ylabel("phase [deg]")
        ap.set_xlabel("frequency [Hz]")
        ag.set_title("steering: commanded -> measured yaw rate")
        ag.legend()
        path = out_dir / "steering_bode.png"
        fig.savefig(path, dpi=100)
        plt.close(fig)
        written.append(path)
    return written


def main(argv=None) -> int:
    from run_analysis.imu_report import load_imu_axes
    from run_analysis.runlog import analysis_dir, load_run
    from run_analysis.scan_motion import scan_motion

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run_dir")
    ap.add_argument("--no-icp", action="store_true", help="skip LiDAR scan matching")
    ap.add_argument("--refresh", action="store_true", help="re-read the bag")
    ap.add_argument("--vesc-rate", type=float, help="VESC Tool IMU sample rate [Hz]")
    ap.add_argument("--accel-hz", type=float, nargs="+", help="VESC accel low-pass X [Y Z] [Hz], 0 = off")
    ap.add_argument("--gyro-hz", type=float, help="VESC gyro low-pass [Hz], 0 = off")
    ap.add_argument("--racer-imu-root", default="~/imu-calibration")
    args = ap.parse_args(argv)
    filt = None
    if args.vesc_rate is not None and args.accel_hz is not None and args.gyro_hz is not None:
        acc = args.accel_hz * 3 if len(args.accel_hz) == 1 else args.accel_hz
        if len(acc) != 3:
            ap.error("--accel-hz takes 1 or 3 values")
        filt = dict(rate=args.vesc_rate, accel=acc, gyro=args.gyro_hz)
    elif any(v is not None for v in (args.vesc_rate, args.accel_hz, args.gyro_hz)):
        ap.error("--vesc-rate, --accel-hz and --gyro-hz go together")
    axes = load_imu_axes(args.run_dir, refresh=args.refresh)
    motion = None
    if not args.no_icp:
        try:
            run = load_run(args.run_dir, refresh=args.refresh)
            motion = scan_motion(run, float(run.scan_t[0]), float(run.scan_t[-1]))
        except (ValueError, IndexError) as exc:
            print(f"(LiDAR skipped: {exc})")
    lines, plots, csv = build_report(Path(args.run_dir).expanduser(), axes, motion, filt,
                                     args.racer_imu_root)
    out = analysis_dir(args.run_dir)
    if csv:
        (out / "steering_frf.csv").write_text(
            "source,f_hz,gain,phase_deg,coherence\n"
            + "".join(f"{s},{f:.4f},{g:.5f},{p:.2f},{c:.3f}\n" for s, f, g, p, c in csv))
    for path in _save_plots(out, plots, csv):
        lines.append(f"plot: {path.name}")
    text = "\n".join(lines) + "\n"
    (out / "imu_spectrum.txt").write_text(text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
