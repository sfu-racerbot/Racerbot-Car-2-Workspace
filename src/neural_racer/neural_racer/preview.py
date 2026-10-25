"""Map-mode geometry: the reference path, its free widths, and the preview block.

The reference path is the cleaned lap-1 path that auto_map_race already
writes. It is a ruler along the track, not a line to follow. The left and
right free widths come from the frozen lap-1 map. No rclpy import.
"""
import math

import numpy as np

from neural_racer import observation as obs


class PreviewError(ValueError):
    """The path or map cannot give a trustworthy preview."""


class GridMap:
    """A frozen occupancy map as a boolean 'blocked' grid. Unknown cells count
    as blocked: a width measured into unexplored space is not free width."""

    def __init__(self, blocked: np.ndarray, resolution: float, origin_x: float, origin_y: float):
        self.blocked = np.asarray(blocked, dtype=bool)
        if self.blocked.ndim != 2 or not (resolution > 0.0):
            raise PreviewError('map must be a 2D grid with positive resolution')
        self.res = float(resolution)
        self.ox, self.oy = float(origin_x), float(origin_y)

    @classmethod
    def from_occupancy(cls, data, width: int, height: int, resolution: float,
                       origin_x: float, origin_y: float, occupied_threshold: int = 50):
        """nav_msgs/OccupancyGrid layout: row-major from the origin corner, -1 unknown."""
        cells = np.asarray(data, dtype=np.int16).reshape(height, width)
        return cls((cells >= occupied_threshold) | (cells < 0), resolution, origin_x, origin_y)

    def is_blocked(self, x, y) -> np.ndarray:
        col = np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64)
        row = np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int64)
        h, w = self.blocked.shape
        inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)
        out = np.ones(np.shape(col), dtype=bool)
        out[inside] = self.blocked[row[inside], col[inside]]
        return out

    def first_blocked_along(self, polyline: np.ndarray) -> float | None:
        """Distance along a world-frame polyline to its first blocked cell, or None."""
        pts = np.asarray(polyline, dtype=np.float64)
        travelled = 0.0
        step = self.res / 2.0
        for a, b in zip(pts[:-1], pts[1:]):
            seg = float(np.hypot(*(b - a)))
            n = max(int(math.ceil(seg / step)), 1)
            t = np.linspace(0.0, 1.0, n + 1)[1:]
            xs, ys = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            hit = np.flatnonzero(self.is_blocked(xs, ys))
            if hit.size:
                return travelled + float(t[hit[0]]) * seg
            travelled += seg
        return None


def free_widths(path_xy: np.ndarray, grid: GridMap, max_width: float = 3.0):
    """Free distance to the left and right of every path point, marched along
    the path's local normal in half-cell steps, capped at max_width."""
    xy = np.asarray(path_xy, dtype=np.float64)
    tangent = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    heading = np.arctan2(tangent[:, 1], tangent[:, 0])
    normal = np.stack([-np.sin(heading), np.cos(heading)], axis=1)
    ts = np.arange(grid.res / 2.0, max_width + 1e-9, grid.res / 2.0)
    widths = []
    for sign in (1.0, -1.0):
        px = xy[:, :1] + sign * normal[:, :1] * ts[None, :]
        py = xy[:, 1:] + sign * normal[:, 1:] * ts[None, :]
        blocked = grid.is_blocked(px, py)
        first = np.where(blocked.any(axis=1), blocked.argmax(axis=1), ts.size)
        widths.append(np.where(first < ts.size, ts[np.minimum(first, ts.size - 1)], max_width))
    return widths[0], widths[1]


class ReferencePath:
    """A closed path with free widths, sampled by arc length."""

    def __init__(self, xy, w_left, w_right):
        xy = np.asarray(xy, dtype=np.float64)
        wl, wr = np.asarray(w_left, dtype=np.float64), np.asarray(w_right, dtype=np.float64)
        if xy.ndim != 2 or xy.shape[1] != 2 or xy.shape[0] < 4:
            raise PreviewError('reference path needs at least 4 (x, y) points')
        if wl.shape != (xy.shape[0],) or wr.shape != (xy.shape[0],):
            raise PreviewError('one left and one right width per path point')
        if not (np.all(np.isfinite(xy)) and np.all(np.isfinite(wl)) and np.all(np.isfinite(wr))):
            raise PreviewError('reference path is not finite')
        seg = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)
        keep = seg > 1e-6                      # drop duplicate points
        self.xy, self.wl, self.wr = xy[keep], wl[keep], wr[keep]
        if self.xy.shape[0] < 4:
            raise PreviewError('reference path collapses to fewer than 4 distinct points')
        self.seg = np.hypot(*(np.roll(self.xy, -1, axis=0) - self.xy).T)
        self.s = np.concatenate([[0.0], np.cumsum(self.seg)[:-1]])
        self.length = float(self.seg.sum())

    @classmethod
    def from_grid(cls, xy, grid: GridMap, max_width: float = 3.0):
        wl, wr = free_widths(xy, grid, max_width)
        return cls(xy, wl, wr)

    def project(self, x: float, y: float, hint: int | None = None, window: int = 40):
        """(segment index, arc length) of the closest point on the path."""
        n = self.xy.shape[0]
        idx = np.arange(n) if hint is None else (hint + np.arange(-window, window + 1)) % n
        a = self.xy[idx]
        b = self.xy[(idx + 1) % n]
        ab = b - a
        t = np.clip(((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1])
                    / np.maximum((ab ** 2).sum(axis=1), 1e-12), 0.0, 1.0)
        d2 = (a[:, 0] + t * ab[:, 0] - x) ** 2 + (a[:, 1] + t * ab[:, 1] - y) ** 2
        k = int(np.argmin(d2))
        i = int(idx[k])
        return i, float(self.s[i] + t[k] * self.seg[i])

    def sample(self, s) -> np.ndarray:
        """(len(s), 4): x, y, w_left, w_right at arc lengths s, wrapping round the lap."""
        s = np.mod(np.asarray(s, dtype=np.float64), self.length)
        i = np.searchsorted(self.s, s, side='right') - 1
        j = (i + 1) % self.xy.shape[0]
        t = (s - self.s[i]) / self.seg[i]
        lerp = lambda v: v[i] + t * (v[j] - v[i])   # noqa: E731
        return np.stack([lerp(self.xy[:, 0]), lerp(self.xy[:, 1]),
                         lerp(self.wl), lerp(self.wr)], axis=1)

    def preview_block(self, x: float, y: float, yaw: float, hint: int | None = None):
        """(block (4, 32) float32, segment index, arc length) for a car at (x, y, yaw)."""
        i, s0 = self.project(x, y, hint)
        ahead = self.sample(s0 + obs.PREVIEW_SPACING * np.arange(1, obs.PREVIEW_POINTS + 1))
        dx, dy = ahead[:, 0] - x, ahead[:, 1] - y
        c, sn = math.cos(yaw), math.sin(yaw)
        block = np.stack([
            (c * dx + sn * dy) / obs.PREVIEW_XY_SCALE,
            (-sn * dx + c * dy) / obs.PREVIEW_XY_SCALE,
            ahead[:, 2] / obs.PREVIEW_WIDTH_SCALE,
            ahead[:, 3] / obs.PREVIEW_WIDTH_SCALE,
        ]).astype(np.float32)
        return block, i, s0


def car_to_world(points, x: float, y: float, yaw: float) -> np.ndarray:
    p = np.asarray(points, dtype=np.float64)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], axis=1)
