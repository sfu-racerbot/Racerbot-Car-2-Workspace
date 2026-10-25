"""Map-mode geometry (neural_racer/preview.py). ROS-free.

Runner: python3 -m pytest src/neural_racer/test/test_preview.py -v   (unsourced)
Oracles: closed forms on a circle and a straight corridor drawn into a grid.
"""
import math

import nr_testutil  # noqa: F401
import numpy as np
import pytest

from neural_racer import observation as obs, preview

RES = 0.05
# Width error bound: the grid places a wall up to one cell from the true
# circle, and the march samples every half cell, so |error| <= 1.5 cells.
WIDTH_TOL = 1.5 * RES


def _ring_grid(r_in=2.0, r_out=3.0, half=4.0):
    """An annulus of free space between radii r_in and r_out, centred at 0."""
    n = int(2 * half / RES)
    c = (np.arange(n) + 0.5) * RES - half
    xx, yy = np.meshgrid(c, c)
    rr = np.hypot(xx, yy)
    blocked = (rr < r_in) | (rr > r_out)
    return preview.GridMap(blocked, RES, -half, -half)


def _circle(r=2.5, n=400, ccw=True):
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    if not ccw:
        t = -t
    return np.stack([r * np.cos(t), r * np.sin(t)], axis=1)


def test_free_widths_on_a_ring():
    """Path on the 2.5 m circle, CCW: left is the inside (0.5 m to r=2.0),
    right is the outside (0.5 m to r=3.0), within WIDTH_TOL."""
    wl, wr = preview.free_widths(_circle(), _ring_grid())
    assert np.all(np.abs(wl - 0.5) <= WIDTH_TOL) and np.all(np.abs(wr - 0.5) <= WIDTH_TOL)


def test_free_widths_swap_when_reversed():
    wl, wr = preview.free_widths(_circle(r=2.3, ccw=False), _ring_grid())
    assert np.all(np.abs(wl - 0.7) <= WIDTH_TOL) and np.all(np.abs(wr - 0.3) <= WIDTH_TOL)


def test_free_width_capped():
    grid = preview.GridMap(np.zeros((400, 400), bool), RES, -10, -10)
    wl, wr = preview.free_widths(_circle(), grid, max_width=1.5)
    assert np.all(wl == 1.5) and np.all(wr == 1.5)


def test_unknown_cells_count_as_blocked():
    data = np.zeros((10, 10), np.int16)
    data[5, 5] = -1
    g = preview.GridMap.from_occupancy(data.reshape(-1), 10, 10, RES, 0.0, 0.0)
    assert g.is_blocked(5.5 * RES, 5.5 * RES) and not g.is_blocked(1.5 * RES, 1.5 * RES)
    assert g.is_blocked(-1.0, 0.0)                      # outside the map


def test_path_length_and_projection():
    ref = preview.ReferencePath(_circle(), np.full(400, 0.5), np.full(400, 0.5))
    # perimeter of a 400-gon of radius 2.5 (closed form)
    assert ref.length == pytest.approx(400 * 2 * 2.5 * math.sin(math.pi / 400), abs=1e-9)
    i, s = ref.project(0.0, 2.6)                        # just outside the top of the circle
    assert s == pytest.approx(ref.length / 4, abs=0.02)


def test_preview_block_in_car_frame():
    """A car at (2.5, 0) heading +y (CCW tangent) on the circle: the first
    preview point is 0.5 m along the arc, i.e. ahead (+x car) and slightly left."""
    ref = preview.ReferencePath(_circle(), np.full(400, 0.4), np.full(400, 0.6))
    block, _, _ = ref.preview_block(2.5, 0.0, math.pi / 2)
    x0, y0 = block[0, 0] * 16, block[1, 0] * 16
    angle = 0.5 / 2.5
    assert x0 == pytest.approx(2.5 * math.sin(angle), abs=2e-3)
    assert y0 == pytest.approx(2.5 * (1 - math.cos(angle)), abs=2e-3)
    assert np.allclose(block[2] * 2, 0.4) and np.allclose(block[3] * 2, 0.6)
    assert block.shape == (4, obs.PREVIEW_POINTS) and block.dtype == np.float32


def test_preview_wraps_round_the_lap():
    ref = preview.ReferencePath(_circle(), np.full(400, 0.5), np.full(400, 0.5))
    block, _, s0 = ref.preview_block(2.5, -0.01, math.pi / 2)   # just before the seam
    assert np.all(np.isfinite(block))
    pts = np.stack([block[0] * 16, block[1] * 16], axis=1)
    steps = np.hypot(*np.diff(pts, axis=0).T)
    assert np.allclose(steps, 0.5, atol=0.01)                    # evenly spaced through the seam


def test_duplicate_points_dropped():
    xy = _circle(n=40)
    xy = np.repeat(xy, 2, axis=0)
    ref = preview.ReferencePath(xy, np.ones(80), np.ones(80))
    assert ref.xy.shape[0] == 40


@pytest.mark.parametrize('n', [0, 3])
def test_too_short_path_refused(n):
    with pytest.raises(preview.PreviewError, match='at least 4'):
        preview.ReferencePath(np.zeros((n, 2)), np.zeros(n), np.zeros(n))


def test_non_finite_path_refused():
    xy = _circle(n=10)
    xy[3, 0] = math.nan
    with pytest.raises(preview.PreviewError, match='finite'):
        preview.ReferencePath(xy, np.ones(10), np.ones(10))


def test_first_blocked_along():
    g = _ring_grid()
    # from the ring's middle (r=2.5) straight outward: wall at r=3.0, 0.5 m away
    d = g.first_blocked_along(np.array([[2.5, 0.0], [3.5, 0.0]]))
    assert d == pytest.approx(0.5, abs=RES)
    assert g.first_blocked_along(np.array([[2.5, 0.0], [2.5, 0.2]])) is None


def test_car_to_world():
    pts = preview.car_to_world(np.array([[1.0, 0.0], [0.0, 1.0]]), 1.0, 2.0, math.pi / 2)
    assert np.allclose(pts, [[1.0, 3.0], [0.0, 2.0]])
