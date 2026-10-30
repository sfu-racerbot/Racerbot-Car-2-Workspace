"""Export run 20261026-174911's IMU analysis to JSON for the visualization page."""
import csv
import json
import sys
from pathlib import Path

import numpy as np
from scipy import signal

WS = Path.home() / "racerbot-ws"
sys.path.insert(0, str(WS / "tools"))
from run_analysis import estimator_replay as er  # noqa: E402
from run_analysis import imu_spectrum as ims  # noqa: E402
from run_analysis.imu_report import load_imu_axes  # noqa: E402

RUN = WS / "data/racerbot_runs/20261026-174911"
CAR = Path.home() / "imu-calibration/config/car.yaml"
L = 0.36

axes = load_imu_axes(RUN)
vesc = ims.vesc_stream(axes, RUN, CAR)
mz = np.load(next((RUN / "analysis").glob("motion_*.npz")))
motion_t, motion_w, motion_v = mz["t"], mz["w"], mz["v"]
t0 = float(vesc.stamp[0])

# --- time series on a 20 Hz grid
grid = np.arange(max(vesc.stamp[0], motion_t[0]), min(vesc.stamp[-1], motion_t[-1]), 0.05)
gz = np.interp(grid, vesc.stamp, vesc.gyro[:, 2])
cmd, _ = ims.commanded_yaw_rate(grid, axes, L)
odom = axes["odom"]
odom_w = np.interp(grid, odom[:, 0], odom[:, 3])
lid = np.interp(grid, motion_t, motion_w)
speed = np.interp(grid, motion_t, motion_v)
series = {"t": (grid - t0).round(2).tolist(), "gyro": gz.round(4).tolist(),
          "cmd": np.nan_to_num(cmd).round(4).tolist(), "odom": odom_w.round(4).tolist(),
          "lidar": lid.round(4).tolist(), "speed": speed.round(3).tolist()}

# --- frequency response
frf = {}
for r in csv.DictReader(open(RUN / "analysis/steering_frf.csv")):
    frf.setdefault(r["source"], []).append(
        [float(r["f_hz"]), float(r["gain"]), float(r["phase_deg"]), float(r["coherence"])])

# --- heading windows (same procedure as estimator_replay.score, per window)
streams, _ = er.streams_from_axes(axes, RUN, CAR)
res = er.replay(streams, odom, er.build_config({}))
lidar_yaw = np.concatenate([[0.0], np.cumsum(0.5 * (motion_w[1:] + motion_w[:-1]) * np.diff(motion_t))])
a0 = max(motion_t[0], res.pose_t[0]); a1 = min(motion_t[-1], res.pose_t[-1])
wins = []
for a in np.arange(a0, a1 - er.HEADING_WINDOW_S + 1e-9, er.HEADING_WINDOW_S):
    b = a + er.HEADING_WINDOW_S
    sel = (motion_t >= a) & (motion_t <= b)
    if sel.sum() < 2 or np.mean(np.abs(motion_v[sel])) <= er.MOVING_SPEED:
        continue
    dl = np.interp(b, motion_t, lidar_yaw) - np.interp(a, motion_t, lidar_yaw)
    dg = np.interp(b, res.pose_t, res.gyro_yaw) - np.interp(a, res.pose_t, res.gyro_yaw)
    do = np.interp(b, res.pose_t, res.odom_yaw) - np.interp(a, res.pose_t, res.odom_yaw)
    wins.append([round(a - t0, 2), round(np.degrees(dl), 2), round(np.degrees(dg), 2), round(np.degrees(do), 2)])

# --- parked gyro z PSD (still stretches, 50 Hz)
segs = ims.still_segments(vesc, axes)
psds = []
for i, j in segs:
    x = vesc.gyro[i:j, 2]
    if len(x) >= 256:
        f, p = signal.welch(x, fs=50.0, nperseg=256, detrend="constant")
        psds.append((len(x), p))
w = sum(n for n, _ in psds)
P = sum(n * p for n, p in psds) / w
psd = {"f": f.round(3).tolist(), "p": P.tolist(), "n_segments": len(psds)}

# --- conjugate symmetry: two-sided DFT of 64 samples of gyro z while turning
moving = np.flatnonzero(np.abs(np.interp(vesc.stamp, motion_t, motion_v)) > 0.5)
k0 = moving[len(moving) // 3]
x = vesc.gyro[k0:k0 + 64, 2]
X = np.fft.fftshift(np.fft.fft(x))
k = np.arange(-32, 32)
dft = {"x": x.round(4).tolist(), "k": k.tolist(), "re": X.real.round(4).tolist(),
       "im": X.imag.round(4).tolist(), "t_start": round(float(vesc.stamp[k0] - t0), 2)}

json.dump({"series": series, "frf": frf, "windows": wins, "psd": psd, "dft": dft,
           "bias_z": float(np.mean(np.concatenate([vesc.gyro[i:j, 2] for i, j in segs])))},
          open(sys.argv[1], "w"), separators=(",", ":"))
print("ok", len(series["t"]), len(wins), psd["n_segments"], len(x))
