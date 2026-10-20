"""How the real car responded to its commands: lags, gains, grip, odom scale.

Every number here compares what the node *commanded* (/drive) with what the
car *did*, measured by scan_motion (LiDAR scan matching) -- never against
/odom's yaw rate, which is derived from the steering command itself.

- speed lag: the delay that best aligns commanded speed with achieved speed;
- steering lag and curvature gain: the delay and scale that best map
  ``tan(steering)/wheelbase`` onto measured curvature ``w / v``;
- grip: achieved / commanded curvature, binned by the lateral acceleration
  that command asked for -- where it falls below ~0.9 the tyres are sliding;
- odom scale: LiDAR speed / odom speed, binned by curvature.

These are the numbers the simulator's car model (tools/f1tenth_sim/
sim_fidelity/calibration.py) should agree with.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

WHEELBASE = 0.36  # m, tape-measured 2026-08-24 (docs/hardware-reference.md)


@dataclass
class ActuationReport:
    speed_lag_s: float
    speed_rms_at_lag: float
    steering_lag_s: float
    curvature_gain: float
    curvature_rms_at_lag: float
    curvature_gain_left: float
    curvature_gain_right: float
    grip_bins: list = field(default_factory=list)       # (a_lo, a_hi, n, median ratio)
    odom_scale_bins: list = field(default_factory=list)  # (k_lo, k_hi, n, median lidar/odom)
    lateral_accel_p50_p90_p99: tuple = ()

    def lines(self) -> list:
        out = [
            f"speed lag {self.speed_lag_s:.3f} s (RMS speed error {self.speed_rms_at_lag:.2f} m/s at that lag)",
            f"steering lag {self.steering_lag_s:.3f} s, curvature gain {self.curvature_gain:.3f} "
            f"(left {self.curvature_gain_left:.3f}, right {self.curvature_gain_right:.3f}; "
            f"RMS {self.curvature_rms_at_lag:.3f} 1/m)",
            "achieved lateral accel p50/p90/p99: "
            + " / ".join(f"{a:.2f}" for a in self.lateral_accel_p50_p90_p99) + " m/s^2",
            "grip (achieved / commanded curvature, by commanded lateral accel):",
        ]
        out += [f"  {lo:.1f}-{hi:.1f} m/s^2: n={n:4d} ratio {r:.3f}" for lo, hi, n, r in self.grip_bins]
        out.append("odom speed scale (LiDAR / odom, by curvature):")
        out += [f"  |k| {lo:.1f}-{hi:.1f} 1/m: n={n:4d} ratio {r:.3f}" for lo, hi, n, r in self.odom_scale_bins]
        return out


def best_lag(t, measured, cmd_t, cmd, lags, valid=None):
    """(lag, rms) minimising RMS(measured(t) - cmd(t - lag))."""
    best = (float("nan"), float("inf"))
    for lag in lags:
        c = np.interp(t - lag, cmd_t, cmd)
        err = measured - c
        if valid is not None:
            err = err[valid]
        rms = float(np.sqrt(np.mean(err ** 2))) if err.size else float("inf")
        if rms < best[1]:
            best = (float(lag), rms)
    return best


def _gain(measured, commanded):
    den = float(np.sum(commanded ** 2))
    return float(np.sum(measured * commanded) / den) if den > 0 else float("nan")


def analyze(motion, drive: np.ndarray, odom: np.ndarray, t0: float, t1: float,
            wheelbase: float = WHEELBASE, min_speed: float = 1.0) -> ActuationReport:
    keep = (motion.t >= t0) & (motion.t <= t1) & np.isfinite(motion.v)
    t, v, w = motion.t[keep], motion.v[keep], motion.w[keep]
    if t.size < 20:
        raise ValueError("fewer than 20 matched scans in the window")
    lags = np.arange(0.0, 0.61, 0.025)
    speed_lag, speed_rms = best_lag(t, v, drive[:, 0], drive[:, 3], lags)

    k_meas = w / np.maximum(v, 0.3)
    moving = v > min_speed
    best = None
    for lag in lags:
        k_cmd = np.tan(np.interp(t - lag, drive[:, 0], drive[:, 2])) / wheelbase
        g = _gain(k_meas[moving], k_cmd[moving])
        rms = float(np.sqrt(np.mean((k_meas[moving] - g * k_cmd[moving]) ** 2)))
        if best is None or rms < best[2]:
            best = (float(lag), g, rms, k_cmd)
    steer_lag, gain, k_rms, k_cmd = best
    left = moving & (k_cmd > 0.2)
    right = moving & (k_cmd < -0.2)

    a_cmd = v ** 2 * np.abs(k_cmd)
    grip = []
    for lo, hi in ((0.3, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.0), (3.0, 5.0)):
        sel = moving & (np.abs(k_cmd) > 0.15) & (a_cmd >= lo) & (a_cmd < hi)
        if sel.sum() >= 5:
            grip.append((lo, hi, int(sel.sum()), float(np.median(k_meas[sel] / k_cmd[sel]))))

    v_odom = np.interp(t, odom[:, 0], odom[:, 1])
    scale = []
    for lo, hi in ((0.0, 0.1), (0.1, 0.3), (0.3, 0.6), (0.6, 2.0)):
        sel = (v_odom > 0.8) & (np.abs(k_meas) >= lo) & (np.abs(k_meas) < hi)
        if sel.sum() >= 5:
            scale.append((lo, hi, int(sel.sum()), float(np.median(v[sel] / v_odom[sel]))))

    a_lat = np.abs(v * w)
    return ActuationReport(
        speed_lag, speed_rms, steer_lag, gain, k_rms,
        _gain(k_meas[left], k_cmd[left]) if left.any() else float("nan"),
        _gain(k_meas[right], k_cmd[right]) if right.any() else float("nan"),
        grip, scale, tuple(float(x) for x in np.percentile(a_lat, [50, 90, 99])))
