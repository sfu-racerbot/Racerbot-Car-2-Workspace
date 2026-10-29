"""Metrics of SLAM's map -> odometry corrections, measured at the car.

Everything is computed from the car's odometry and the correction SLAM published. Nothing
here reads SLAM's own claims (its loop-closure log lines aside, which are only used for
timing), its pose estimate or its map, so a report built from these numbers can disagree
with SLAM.

Inputs are float arrays, one row per sample, all on one clock (the bag's receive time, or
the log's local time converted to epoch seconds). Rows with a non-finite value are dropped,
and the rest are sorted by time:
    map_odom  (t, x, y, yaw)    map -> <odom frame>        SLAM's correction
    odom      (t, x, y, yaw)    <odom frame> -> base_link  the car, in that odom frame
    events    (t, move_m, rotate_rad)  one row per change of map_odom

ROS-free and numpy-only. slam_bag.load() gets the arrays from a bag.
"""
import re
from datetime import datetime

import numpy as np

CHANGE_TOL = 1e-9  # m or rad: smaller differences are float noise, not a SLAM update
JUMP_M = 0.3  # m: the threshold behind summarize's "jumps_over_0_3m"
_CERES_SOLVE = re.compile(r"W(\d{8}) (\d\d:\d\d:\d\d\.\d+) .*num_threads")


def _wrap(a):
    """Wrap angles into [-pi, pi)."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _rows(a, cols, name):
    """Finite rows of an (N, cols) array, sorted by time. Empty input gives (0, cols)."""
    a = np.asarray(a, dtype=float)
    if a.size == 0:
        return np.zeros((0, cols))
    if a.ndim != 2 or a.shape[1] != cols:
        raise ValueError(f"{name} must have {cols} columns, got shape {a.shape}")
    a = a[np.all(np.isfinite(a), axis=1)]
    return a[np.argsort(a[:, 0], kind="stable")]


def _compose(a, b):
    """Planar poses (N, 3) = (x, y, yaw): pose a applied to pose b, row by row."""
    c, s = np.cos(a[:, 2]), np.sin(a[:, 2])
    return np.column_stack([
        a[:, 0] + c * b[:, 0] - s * b[:, 1],
        a[:, 1] + s * b[:, 0] + c * b[:, 1],
        _wrap(a[:, 2] + b[:, 2]),
    ])


def correction_events(map_odom, odom) -> np.ndarray:
    """One (t, move_m, rotate_rad) row for every change of the map -> odom correction.

    The move is how far the car's own pose moves when the correction changes: the old and the
    new correction are each applied to the car's odometry pose at that instant, and the
    distance between those two map-frame positions is the move. The rotation is the
    correction's own yaw change, wrapped to [-pi, pi). The car's pose is interpolated from
    odom (yaw unwrapped first) and held at odom's first and last samples outside its time range.
    """
    mo = _rows(map_odom, 4, "map_odom")
    if len(mo) < 2:
        return np.zeros((0, 3))
    dxy = np.abs(np.diff(mo[:, 1:3], axis=0))
    dyaw = np.abs(_wrap(np.diff(mo[:, 3])))
    moved = np.any(np.column_stack([dxy, dyaw]) > CHANGE_TOL, axis=1)
    changed = np.nonzero(moved)[0] + 1  # the first row of each new correction
    if changed.size == 0:
        return np.zeros((0, 3))
    od = _rows(odom, 4, "odom")
    if len(od) == 0:
        raise ValueError("no odometry rows: cannot place the car at the correction times")
    t = mo[changed, 0]
    car = np.column_stack([
        np.interp(t, od[:, 0], od[:, 1]),
        np.interp(t, od[:, 0], od[:, 2]),
        np.interp(t, od[:, 0], np.unwrap(od[:, 3])),
    ])
    before = _compose(mo[changed - 1, 1:4], car)
    after = _compose(mo[changed, 1:4], car)
    move = np.hypot(after[:, 0] - before[:, 0], after[:, 1] - before[:, 1])
    rotate = _wrap(after[:, 2] - before[:, 2])
    return np.column_stack([t, move, rotate])


def loop_closure_times(log_text: str) -> np.ndarray:
    """Epoch seconds of each Ceres solve line in a slam_toolbox log, read as local time.

    Ceres prints one ``W<yyyymmdd> <hh:mm:ss.ffffff> ... num_threads`` line per pose-graph
    solve. Each solve is treated as a loop-closure optimisation, the same proxy jumps.py used;
    the solve lines are not checked against slam_toolbox's own loop-closure messages.
    """
    times = []
    for line in log_text.splitlines():
        m = _CERES_SOLVE.search(line)
        if m:
            stamp = datetime.strptime(m.group(1) + " " + m.group(2), "%Y%m%d %H:%M:%S.%f")
            times.append(stamp.timestamp())  # naive, so local time
    return np.array(times, dtype=float)


def summarize(events, closures, window_s: float = 1.5, false_jump_m: float = 1.5) -> dict:
    """Counts and maxima for a run's events, against its loop-closure times.

    A "jump" is a move over JUMP_M (0.3 m). "At loop closures" means within window_s seconds
    of a closure time (|dt| <= window_s). The false-jump count is the moves over false_jump_m.
    Non-finite closure times are ignored.
    """
    ev = np.asarray(events, dtype=float)
    if ev.size == 0:
        ev = np.zeros((0, 3))
    elif ev.ndim != 2 or ev.shape[1] != 3:
        raise ValueError(f"events must have 3 columns, got shape {ev.shape}")
    if not np.all(np.isfinite(ev)):
        raise ValueError("events contain non-finite values")
    closures = np.asarray(closures, dtype=float).reshape(-1)
    closures = closures[np.isfinite(closures)]
    t, move, turn = ev[:, 0], ev[:, 1], np.abs(ev[:, 2])
    if t.size and closures.size:
        gap = np.min(np.abs(t[:, None] - closures[None, :]), axis=1)
    else:
        gap = np.full(t.size, np.inf)
    jump = move > JUMP_M
    return {
        "updates": int(t.size),
        "loop_closures": int(closures.size),
        "max_move_m": float(move.max()) if move.size else 0.0,
        "max_rotate_deg": float(np.degrees(turn.max())) if turn.size else 0.0,
        "jumps_over_0_3m": int(jump.sum()),
        "jumps_over_false_jump_m": int((move > false_jump_m).sum()),
        "jumps_at_loop_closures": int((jump & (gap <= window_s)).sum()),
    }
