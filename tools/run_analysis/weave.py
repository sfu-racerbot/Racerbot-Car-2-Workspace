"""The steering weave, diagnosed on one recorded run: loop or aim point?

gap_follow's steering swings more than 0.2 rad about once a second (spec A5,
docs/superpowers/specs/2026-10-26-slam-and-gap-follow-plan.md). Two causes fit
that, and they leave different traces in a run:

- ``loop``: the heading loop itself oscillates. The steering sets the car's yaw
  with a lag ``tau``, and the controller steers back on the bearing to the gap
  with gain ``K``. That loop is ``tau*th'' + th' + (v*K/L)*th = 0``, with ``th``
  the bearing error and ``L`` the wheelbase. The gyro's yaw rate then swings at
  the steering's frequency and trails it, and the model predicts that frequency.
- ``aim_point``: the chosen gap's bearing (``target_angle`` in the replay's
  telemetry) jumps, and the steering follows each jump, so the swings start at
  the jumps.

``diagnose(run_dir)`` measures both on one run, and ``classify`` applies the
decision rule to the numbers. The helpers are pure numpy/scipy and run unsourced.
``diagnose`` reads the run through runlog and imu_report: the first call on a run
needs ROS sourced (rosbag2 reads the bag), and it caches the IMU arrays in the
run's ``analysis/`` folder, as imu_report does.

    from run_analysis import weave
    weave.diagnose("~/.ros/racerbot_runs/20261026-194514")
"""

from __future__ import annotations

import dataclasses
import math
import sys
import tempfile
import warnings
from pathlib import Path

import numpy as np
import yaml
from scipy import signal

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from run_analysis import actuation, imu_report, replay, runlog  # noqa: E402
import gap_mirror  # noqa: E402  (replay.py put tools/f1tenth_sim on sys.path)
from gap_follow import gap_logic  # noqa: E402  (and src/gap_follow)

# A steering swing this big is a weave (spec A5: "more than 0.2 rad").
SWING_RAD = 0.2
# A target_angle change this big between two ticks is an aim-point jump.
JUMP_RAD = 0.15
# The controller runs one tick per scan, at 40 Hz.
FS = 1.0 / gap_mirror.CONTROL_DT
# Welch segment: 256 samples, 6.4 s, so a 0.156 Hz bin.
NPERSEG = 256
# Where the steering's spectral peak is looked for. The weave is about 1 Hz; the
# band leaves out a lap's slow turns below it and everything far above the weave.
SEARCH_HZ = (0.2, 5.0)
WHEELBASE = actuation.WHEELBASE
# The decision rule, as the Task 7 brief states it.
COHERENCE_LOOP = 0.7     # 'loop': steering-to-yaw coherence at the peak, at least this
MODEL_TOLERANCE = 0.3    # 'loop': the model frequency within this fraction of the peak
SHARE_LOOP_MAX = 0.3     # 'loop': aim-point jumps account for less than this share
SHARE_AIM_MIN = 0.5      # 'aim_point': at least this share of swings start at a jump


def _positive(value) -> bool:
    return math.isfinite(value) and value > 0.0


def turning_points(x, swing_rad: float = SWING_RAD) -> np.ndarray:
    """Indices of the turning points of ``x`` that bound a swing larger than ``swing_rad``.

    A zigzag: a turn at a running extreme is confirmed once the trace has come
    back more than ``swing_rad`` from that extreme, so a wiggle below the
    threshold never counts. Non-finite samples are skipped. The first and last
    samples are not turning points, and an extreme the trace has not yet come
    back from is not counted.
    """
    x = np.asarray(x, dtype=float)
    finite = np.flatnonzero(np.isfinite(x))
    if finite.size < 2:
        return np.array([], dtype=int)
    first = finite[0]
    turns = []
    lo = hi = first     # undecided: the running min and max since the start
    trend = 0           # +1 rising (ext is the running max); -1 falling (ext is the running min)
    ext = first
    for i in finite[1:]:
        v = x[i]
        if trend == 0:
            if v > x[hi]:
                hi = i
            if v < x[lo]:
                lo = i
            if x[hi] - v > swing_rad:       # a fall from hi: hi was a turn
                if hi != first:
                    turns.append(hi)
                trend, ext = -1, lo
            elif v - x[lo] > swing_rad:     # a rise from lo: lo was a turn
                if lo != first:
                    turns.append(lo)
                trend, ext = +1, hi
        elif trend > 0:
            if v > x[ext]:
                ext = i
            elif x[ext] - v > swing_rad:
                turns.append(ext)
                trend, ext = -1, i
        else:
            if v < x[ext]:
                ext = i
            elif v - x[ext] > swing_rad:
                turns.append(ext)
                trend, ext = +1, i
    return np.array(turns, dtype=int)


def reversals_per_s(t, steering, swing_rad: float = SWING_RAD) -> float:
    """Direction changes per second whose swing exceeds ``swing_rad``.

    ``t`` must increase. The count is the turning points of ``steering`` (see
    ``turning_points``) divided by the span ``t[-1] - t[0]``. Nan when there is
    no span to divide by.
    """
    t = np.asarray(t, dtype=float)
    steering = np.asarray(steering, dtype=float)
    if t.shape != steering.shape:
        raise ValueError(
            f"t and steering must be the same length, got {t.size} and {steering.size}")
    if not swing_rad > 0.0:
        raise ValueError(f"swing_rad must be positive, got {swing_rad!r}")
    if t.size < 2:
        return math.nan
    duration = float(t[-1] - t[0])
    if not duration > 0.0:
        return math.nan
    return len(turning_points(steering, swing_rad)) / duration


def loop_model(tau_s: float, speed: float, gain: float,
               wheelbase: float = WHEELBASE) -> tuple[float, float]:
    """(zeta, f_hz) of the heading loop ``tau*th'' + th' + (v*K/L)*th = 0``.

    ``tau_s`` is the steering lag, ``gain`` the loop's bearing gain K (the car's
    turning gain times the controller's steering gain at this speed). Then
    ``zeta = 1/(2*sqrt(tau*v*K/L))``, and the frequency is the damped one,
    ``sqrt(v*K/(L*tau))/(2*pi) * sqrt(1 - zeta**2)``, which is zero when the loop
    is critically or over damped.
    """
    for name, value in (("tau_s", tau_s), ("speed", speed), ("gain", gain),
                        ("wheelbase", wheelbase)):
        if not _positive(value):
            raise ValueError(f"{name} must be positive and finite, got {value!r}")
    zeta = 1.0 / (2.0 * math.sqrt(tau_s * speed * gain / wheelbase))
    natural = math.sqrt(speed * gain / (wheelbase * tau_s))
    return zeta, natural / (2.0 * math.pi) * math.sqrt(max(0.0, 1.0 - zeta * zeta))


def target_jump_share(ticks, swing_t, jump_rad: float = JUMP_RAD) -> float:
    """Share of swing starts within one scan of an aim-point jump.

    A jump is a change of more than ``jump_rad`` in ``telemetry["target_angle"]``
    between two consecutive ticks, dated at the later tick. A swing starts within
    one scan of it when the two times are at most one scan apart, the median tick
    spacing. Only consecutive ticks that both carry a target angle and are at most
    two scans apart are compared, so a stop tick or a gap between LB windows never
    makes a jump. Nan when there are no swings, or no usable pair of targets. The
    times must share one clock with ``tick.t``.
    """
    swing_t = np.asarray(swing_t, dtype=float).ravel()
    if swing_t.size == 0:
        return math.nan
    t = np.array([tick.t for tick in ticks], dtype=float)
    if t.size < 2:
        return math.nan
    target = np.array([tick.telemetry.get("target_angle", math.nan) for tick in ticks],
                      dtype=float)
    spacing = np.diff(t)
    scan = float(np.median(spacing))
    if not scan > 0.0:
        return math.nan
    usable = np.isfinite(target[1:]) & np.isfinite(target[:-1]) & (spacing <= 2.0 * scan)
    if not usable.any():
        return math.nan
    jumped = usable & (np.abs(np.diff(target)) > jump_rad)
    jumps = np.sort(t[1:][jumped])
    if jumps.size == 0:
        return 0.0
    after = np.searchsorted(jumps, swing_t)          # first jump at or after each swing
    below = jumps[np.clip(after - 1, 0, jumps.size - 1)]
    above = jumps[np.clip(after, 0, jumps.size - 1)]
    nearest = np.minimum(np.abs(swing_t - below), np.abs(swing_t - above))
    return float(np.mean(nearest <= scan))


def response_at_peak(segments, fs: float = FS, nperseg: int = NPERSEG,
                     search_hz=SEARCH_HZ) -> tuple[float, float, float]:
    """(peak_hz, coherence, phase_lag_deg) of the steering against the yaw rate.

    ``segments`` is a list of (steering, yaw_rate) pairs, each on a uniform grid at
    ``fs`` Hz (one per LB window). Their Welch spectra are pooled, each weighted by
    its number of segments, which is what Welch gives over all of them at once.
    ``peak_hz`` is the steering's largest spectral line inside ``search_hz``, and the
    coherence and phase are the steering-to-yaw values at that line. The phase is
    positive when the yaw rate lags the steering. Nan when no segment is ``nperseg``
    samples long.
    """
    hop = nperseg // 2
    freqs = None
    sxx = syy = sxy = None
    total = 0
    for steer, yaw in segments:
        n = len(steer)
        if n < nperseg:
            continue
        count = (n - nperseg) // hop + 1             # Welch segments in this record
        f, pxx = signal.welch(steer, fs=fs, nperseg=nperseg)
        _, pyy = signal.welch(yaw, fs=fs, nperseg=nperseg)
        _, pxy = signal.csd(steer, yaw, fs=fs, nperseg=nperseg)
        if freqs is None:
            freqs, sxx, syy, sxy = f, count * pxx, count * pyy, count * pxy
        else:
            sxx = sxx + count * pxx
            syy = syy + count * pyy
            sxy = sxy + count * pxy
        total += count
    if total == 0:
        return math.nan, math.nan, math.nan
    pxx, pyy, pxy = sxx / total, syy / total, sxy / total
    band = np.flatnonzero((freqs >= search_hz[0]) & (freqs <= search_hz[1]))
    if band.size == 0:
        return math.nan, math.nan, math.nan
    k = band[np.argmax(pxx[band])]
    coherence = float(abs(pxy[k]) ** 2 / (pxx[k] * pyy[k]))
    lag_deg = float(-np.degrees(np.angle(pxy[k])))   # csd's phase is minus the lag
    return float(freqs[k]), coherence, lag_deg


def classify(coherence_at_peak: float, peak_hz: float, model_hz: float,
             jump_share: float) -> str:
    """The Task 7 decision rule: 'loop', 'aim_point' or 'unclear'.

    'loop' needs the coherence at the peak to be at least 0.7, the model frequency
    within 30% of the peak, and an aim-point jump share below 0.3. 'aim_point'
    needs a jump share of at least 0.5. Anything else, nan included, is 'unclear'.
    """
    if (coherence_at_peak >= COHERENCE_LOOP
            and abs(model_hz - peak_hz) <= MODEL_TOLERANCE * peak_hz
            and jump_share < SHARE_LOOP_MAX):
        return "loop"
    if jump_share >= SHARE_AIM_MIN:
        return "aim_point"
    return "unclear"


def recorded_params(run_dir) -> dict:
    """The gap_follow parameters the run was driven with, for replaying it.

    race_diagnostics snapshots each node's parameters into metadata/params/. That
    file keys the node '/gap_follow_node' (a ros2 param dump), which
    gap_mirror.node_params() does not read. So the snapshot is written under the
    bare key to a temporary YAML and read by node_params itself, with the same
    refusals and legacy fill as analyze_run's --yaml. A run without a snapshot is
    replayed with the tree's gap_follow.yaml, with a warning.
    """
    snapshot = Path(run_dir) / "metadata" / "params" / "gap_follow_node.yaml"
    if not snapshot.is_file():
        warnings.warn(f"{run_dir}: no gap_follow snapshot in metadata/params, "
                      "replaying with the tree's gap_follow.yaml", stacklevel=2)
        return gap_mirror.node_params()
    data = yaml.safe_load(snapshot.read_text()) or {}
    nodes = [value for key, value in data.items() if str(key).lstrip("/") == "gap_follow_node"]
    if not nodes:
        raise ValueError(f"{snapshot}: no gap_follow_node parameters")
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "gap_follow.yaml"
        path.write_text(yaml.safe_dump(
            {"gap_follow_node": {"ros__parameters": nodes[0]["ros__parameters"]}}))
        return gap_mirror.node_params(path, fill_legacy=True)


def _gyro(imu: dict):
    """(time, yaw rate) of the primary gyro: the BNO085, else the VESC, as build_report picks."""
    for name in ("bno", "vesc"):
        rows = imu.get(name)
        if rows is not None and len(rows) >= 2:
            return rows[:, 0], rows[:, 1]
    return None


def _replayed_ticks(run, windows, params) -> list:
    """The controller's replayed ticks over every LB window, dated by each scan's receive time."""
    ticks = []
    for a, b in windows:
        for tick in replay.replay(run, a, b, params=params):
            ticks.append(dataclasses.replace(tick, t=float(run.scan_t[tick.scan_index])))
    return ticks


def _loop_parameters(run, windows, gyro, params):
    """(tau_s, loop gain K, speed v) for the loop model, from the car's own response.

    tau_s is the steering lag and the car's turning gain from actuation.analyze,
    fitted against the gyro as imu_report does. K is that turning gain times the
    controller's steering gain at the driving speed, which is the bearing gain the
    loop closes through. v is the median odometry speed while driving.
    """
    gyro_t, gyro_w = gyro
    tau_s = curvature = math.nan
    if len(run.odom) >= 2:
        motion = imu_report.gyro_motion(gyro_t, gyro_w, run.odom)
        try:
            rep = actuation.analyze(motion, run.drive, run.odom, gyro_t[0], gyro_t[-1],
                                    min_speed=imu_report.FIT_MIN_SPEED)
        except ValueError:
            rep = None      # too little driving to fit: tau and the gain stay nan
        if rep is not None:
            tau_s, curvature = rep.steering_lag_s, rep.curvature_gain
    odom_t, odom_v = run.odom[:, 0], run.odom[:, 1]
    driving = np.zeros(odom_t.shape, dtype=bool)
    for a, b in windows:
        driving |= (odom_t > a) & (odom_t < b)
    driving &= odom_v > imu_report.FIT_MIN_SPEED
    speed = float(np.median(odom_v[driving])) if driving.any() else math.nan
    if not (_positive(speed) and math.isfinite(curvature)):
        return tau_s, math.nan, speed
    scheduled = gap_logic.speed_scheduled_gain(speed, params["steering_gain_speed_ref"])
    return tau_s, curvature * params["steering_gain"] * scheduled, speed


def diagnose(run_dir) -> dict:
    """Measure both weave causes on one recorded run, and say which one it is.

    Steering is the /drive command the car received, inside each LB window. The
    gyro is the primary IMU (BNO085, else VESC). Aim-point jumps come from the
    replay of the run's own scans with the parameters it was driven with.
    Returns eight keys: peak_hz, coherence_at_peak, phase_lag_deg, tau_s,
    model_zeta, model_hz, target_jump_share and verdict. Each is nan where the run
    has nothing to measure, and the verdict is then 'unclear'.
    """
    run_dir = Path(run_dir).expanduser()
    run = runlog.load_run(run_dir)
    imu = imu_report.load_imu(run_dir)
    windows = run.lb_windows()
    gyro = _gyro(imu)
    peak_hz = coherence = phase_deg = tau_s = model_zeta = model_hz = share = math.nan
    if windows:
        params = recorded_params(run_dir)
        drive_t, steer = run.drive[:, 0], run.drive[:, 2]
        swing_t, segments = [], []
        for a, b in windows:
            inside = (drive_t > a) & (drive_t < b)
            ts, ss = drive_t[inside], steer[inside]
            if ts.size < 2:
                continue
            swing_t.extend(ts[turning_points(ss, SWING_RAD)])
            if gyro is not None:
                gyro_t, gyro_w = gyro
                lo, hi = max(a, ts[0], gyro_t[0]), min(b, ts[-1], gyro_t[-1])
                grid = np.arange(lo, hi, 1.0 / FS)
                if grid.size:
                    segments.append((np.interp(grid, ts, ss), np.interp(grid, gyro_t, gyro_w)))
        share = target_jump_share(_replayed_ticks(run, windows, params),
                                  np.asarray(swing_t, dtype=float), JUMP_RAD)
        peak_hz, coherence, phase_deg = response_at_peak(segments)
        if gyro is not None:
            tau_s, gain, speed = _loop_parameters(run, windows, gyro, params)
            if _positive(tau_s) and _positive(gain) and _positive(speed):
                model_zeta, model_hz = loop_model(tau_s, speed, gain)
    return {
        "peak_hz": float(peak_hz),
        "coherence_at_peak": float(coherence),
        "phase_lag_deg": float(phase_deg),
        "tau_s": float(tau_s),
        "model_zeta": float(model_zeta),
        "model_hz": float(model_hz),
        "target_jump_share": float(share),
        "verdict": classify(coherence, peak_hz, model_hz, share),
    }
