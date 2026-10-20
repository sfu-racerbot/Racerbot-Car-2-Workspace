"""The car's real motion, measured from the LiDAR alone.

/odom cannot be the reference for how the car actually moved. Its speed is
wheel speed (it reads high when the tyres slip or scrub), and its yaw rate is
*computed from the steering command* by vesc_to_odom (speed * tan(steer) /
wheelbase), so it agrees with the command by construction and cannot show
understeer or servo lag. Matching each LiDAR scan against the one ``stride``
scans earlier (point-to-point ICP) measures both independently.

Accuracy: the median match residual on run 20261019-200703 was 6 mm. In a
long featureless straight corridor ICP is weakly constrained along the
corridor and can under-read speed; check ``residual`` and compare against
odom on curved sections before trusting a straight-line speed figure.

    motion = scan_motion(run, t0, t1)   # cached in <run>/analysis/
    motion.t, motion.v, motion.w       # time, forward speed (m/s), yaw rate (rad/s)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Motion:
    t: np.ndarray          # midpoint time of each matched pair
    v: np.ndarray          # LiDAR-origin forward speed, m/s
    v_lateral: np.ndarray  # LiDAR-origin sideways speed, m/s (= laser_offset_x * w + slip)
    w: np.ndarray          # yaw rate, rad/s
    residual: np.ndarray   # median inlier distance of the match, m

    def curvature(self, min_speed: float = 0.3) -> np.ndarray:
        return self.w / np.maximum(self.v, min_speed)


def scan_points(ranges: np.ndarray, angles: np.ndarray, max_range: float = 8.0,
                min_range: float = 0.05, step: int = 2) -> np.ndarray:
    r = np.asarray(ranges, dtype=np.float64)
    ok = np.isfinite(r) & (r > min_range) & (r < max_range)
    return np.column_stack((r[ok] * np.cos(angles[ok]), r[ok] * np.sin(angles[ok])))[::step]


def icp(reference: np.ndarray, moving: np.ndarray, initial=(0.0, 0.0, 0.0),
        iterations: int = 30, min_inliers: int = 50):
    """Rigid transform (dtheta, dx, dy) mapping ``moving`` into ``reference``'s
    frame: ``reference ~ R(dtheta) @ moving + (dx, dy)``. Returns
    ``(dtheta, dx, dy, residual)``, or None if too few points matched."""
    from scipy.spatial import cKDTree

    if len(reference) < min_inliers or len(moving) < min_inliers:
        return None
    theta, tx, ty = initial
    tree = cKDTree(reference)
    residual = math.inf
    for it in range(iterations):
        c, s = math.cos(theta), math.sin(theta)
        moved = moving @ np.array([[c, s], [-s, c]]) + (tx, ty)
        dist, idx = tree.query(moved)
        inlier = dist < max(0.15, 0.5 * 0.8 ** it)
        if inlier.sum() < min_inliers:
            return None
        p, q = moved[inlier], reference[idx[inlier]]
        pm, qm = p.mean(axis=0), q.mean(axis=0)
        h = (p - pm).T @ (q - qm)
        dtheta = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        c2, s2 = math.cos(dtheta), math.sin(dtheta)
        rot = np.array([[c2, -s2], [s2, c2]])
        shift = qm - rot @ pm
        theta += dtheta
        tx, ty = rot @ np.array([tx, ty]) + shift
        residual = float(np.median(dist[inlier]))
        if abs(dtheta) < 1e-5 and math.hypot(*shift) < 1e-5:
            break
    return theta, float(tx), float(ty), residual


def scan_motion(run, t0: float, t1: float, stride: int = 2, cache: bool = True) -> Motion:
    """Motion between every scan in [t0, t1] and the one ``stride`` later."""
    from .runlog import analysis_dir

    cache_path = analysis_dir(run.path) / f"motion_{t0:.2f}_{t1:.2f}_{stride}.npz"
    if cache and cache_path.exists():
        z = np.load(cache_path)
        return Motion(z["t"], z["v"], z["v_lateral"], z["w"], z["residual"])
    angles = run.beam_angles()
    idx = np.where((run.scan_t >= t0) & (run.scan_t <= t1))[0]
    rows = []
    guess = (0.0, 0.0, 0.0)
    for k in range(len(idx) - stride):
        i, j = idx[k], idx[k + stride]
        dt = run.scan_t[j] - run.scan_t[i]
        if dt <= 0.0:
            continue
        match = icp(scan_points(run.scan[i], angles), scan_points(run.scan[j], angles), guess)
        if match is None:
            guess = (0.0, 0.0, 0.0)
            continue
        dtheta, dx, dy, res = match
        guess = (dtheta, dx, dy)
        rows.append((0.5 * (run.scan_t[i] + run.scan_t[j]), dx / dt, dy / dt, dtheta / dt, res))
    a = np.array(rows, dtype=np.float64).reshape(-1, 5)
    motion = Motion(a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4])
    if cache:
        np.savez_compressed(cache_path, t=motion.t, v=motion.v, v_lateral=motion.v_lateral,
                            w=motion.w, residual=motion.residual)
    return motion
