"""Tests for tools.run_analysis.scan_motion (pure numpy/scipy parts).

Oracle notes: ICP formula matches the implementation (reference ~ R(dtheta) @ moving + (dx,dy)); 
scan_points filtering is by explicit conditions in code.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # insert tools/
from run_analysis import scan_motion


def test_icp_recovers_known_transform():
    """oracle: closed form - build L-shape + circle, apply moving = R(-θ)@(reference - (dx,dy)), ICP should recover."""
    # L-shaped room corner plus a circle of radius 0.5 (>=400 points total) - make asymmetric for stability
    points = []
    # L shape (asymmetric)
    for t in np.linspace(0, 1.0, 150):
        points.append([t, 0.0])  # along x
    for t in np.linspace(0, 0.6, 90):
        points.append([0.0, t])  # along y (shorter)
    # circle of radius 0.5
    for ang in np.linspace(0, 2 * math.pi, 300, endpoint=False):
        points.append([0.5 * math.cos(ang), 0.5 * math.sin(ang)])
    reference = np.array(points)
    assert len(reference) >= 400

    theta = 0.05  # rad
    dx = 0.10
    dy = -0.03
    # moving = R(-theta) @ (reference - (dx,dy)) because reference ~ R(dtheta)@moving + (dx,dy)
    c = math.cos(-theta)
    s = math.sin(-theta)
    Rm = np.array([[c, -s], [s, c]])  # R(-theta)
    centered = reference - np.array([dx, dy])
    moving = (Rm @ centered.T).T
    result = scan_motion.icp(reference, moving)
    assert result is not None
    dtheta, dxr, dyr, residual = result
    # within 1e-3 (rad / m)
    assert abs(dtheta - theta) < 1e-3
    assert abs(dxr - dx) < 1e-3
    assert abs(dyr - dy) < 1e-3
    assert residual < 1e-3


def test_icp_returns_none_when_too_few_points():
    """oracle: code checks if len(reference) < min_inliers or len(moving) < min_inliers: return None."""
    ref = np.array([[0,0],[1,0],[0,1]])  # 3 points
    mov = np.array([[0,0],[1,0],[0,1],[0.1,0.1]])  # 4 points
    assert scan_motion.icp(ref, mov, min_inliers=50) is None
    assert scan_motion.icp(mov, ref, min_inliers=50) is None


def test_scan_points_filtering():
    """oracle: drops NaN, inf, ranges <= min_range and >= max_range; with step=1 returns valid beams as (r cosθ,r sinθ)."""
    angles = np.array([0.0, np.pi/4, np.pi/2, np.pi])
    ranges = np.array([1.0, np.nan, 0.01, 10.0])  # min_range default 0.05, max_range 8.0
    # ok are those with finite, r > 0.05, r < 8.0 -> first is ok (1.0), others bad
    pts = scan_motion.scan_points(ranges, angles, max_range=8.0, min_range=0.05, step=1)
    # only first beam valid
    assert len(pts) == 1
    assert pts[0, 0] == pytest.approx(1.0 * np.cos(0.0), abs=1e-12)
    assert pts[0, 1] == pytest.approx(1.0 * np.sin(0.0), abs=1e-12)

    # exact boundary: the comparisons in scan_points are strict (r > min_range
    # and r < max_range), so a range sitting exactly on either bound is dropped
    pts_edge = scan_motion.scan_points(np.array([0.05, 8.0]), np.array([0.0, 0.1]),
                                       max_range=8.0, min_range=0.05, step=1)
    assert len(pts_edge) == 0

    # with step=2, test multiple valid
    angles2 = np.array([0.0, 0.1, 0.2, 0.3])
    ranges2 = np.array([1.0, 2.0, np.inf, 3.0])
    pts2 = scan_motion.scan_points(ranges2, angles2, step=2)
    # beam 2 is inf, so the ok beams are 0, 1, 3; step=2 keeps ok-array rows 0
    # and 2, i.e. this synthetic motion's beams 0 and 3, hard-coded:
    #   beam 0: (1.0*cos 0.0, 1.0*sin 0.0) = (1.0, 0.0)
    #   beam 3: (3.0*cos 0.3, 3.0*sin 0.3) = (2.866009467376818, 0.8865606199840186)
    expected = np.array([[1.0, 0.0],
                         [2.866009467376818, 0.8865606199840186]])
    assert np.allclose(pts2, expected)
