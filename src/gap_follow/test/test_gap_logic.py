"""
Unit tests for gap_follow's framework-agnostic scan-processing logic
(gap_logic.py). Runs with plain pytest -- no ROS sourcing, no build, no
rclpy -- exactly like pure_pursuit's racing_math tests:

    python3 -m pytest src/gap_follow/test/ -v
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from gap_follow import gap_logic  # noqa: E402


# ============================================================================
# sanitize_ranges: invalid beams are unknown, not contact
# ============================================================================

def test_sanitize_keeps_normal_readings_valid():
    clean, valid = gap_logic.sanitize_ranges([1.0, 2.5, 9.0], max_range=10.0, range_min=0.05)
    assert np.allclose(clean, [1.0, 2.5, 9.0])
    assert valid.all()


def test_sanitize_nan_is_invalid_and_non_free():
    # The phantom-obstacle bug this replaces: a NaN dropout must neither
    # look like a 0.0m obstacle (emergency stop on noise) nor like free
    # space (steering into a blind spot).
    clean, valid = gap_logic.sanitize_ranges([5.0, float('nan'), 5.0], max_range=10.0, range_min=0.05)
    assert clean[1] == 0.0     # non-free for gap selection
    assert not valid[1]        # excluded from the closest/e-stop check
    assert valid[0] and valid[2]


def test_sanitize_sub_range_min_is_invalid():
    # Readings below the sensor's own range_min are its "no valid
    # return" encoding, not a real 2cm obstacle.
    clean, valid = gap_logic.sanitize_ranges([0.02, 0.0, 1.0], max_range=10.0, range_min=0.05)
    assert clean[0] == 0.0 and clean[1] == 0.0
    assert not valid[0] and not valid[1]
    assert valid[2]


def test_sanitize_posinf_is_free_space_not_invalid():
    # +inf is a *measurement* -- "nothing within reach" -- unlike NaN.
    # It must become max_range (free) for gap selection.
    clean, valid = gap_logic.sanitize_ranges([5.0, float('inf')], max_range=10.0, range_min=0.05)
    assert clean[1] == 10.0
    assert not valid[1]  # still never the "closest obstacle"


def test_sanitize_clips_to_max_range():
    clean, _ = gap_logic.sanitize_ranges([25.0], max_range=10.0)
    assert clean[0] == 10.0


# ============================================================================
# closest_valid: what the emergency stop anchors on
# ============================================================================

def test_closest_valid_ignores_invalid_beams():
    clean, valid = gap_logic.sanitize_ranges(
        [float('nan'), 0.01, 3.0, 2.0, 8.0], max_range=10.0, range_min=0.05)
    idx, dist = gap_logic.closest_valid(clean, valid)
    assert idx == 3
    assert dist == pytest.approx(2.0)


def test_closest_valid_with_no_valid_beams_reports_nothing():
    clean, valid = gap_logic.sanitize_ranges(
        [float('nan'), float('nan')], max_range=10.0, range_min=0.05)
    idx, dist = gap_logic.closest_valid(clean, valid)
    assert idx is None
    assert dist == math.inf


def test_noisy_scan_does_not_emergency_stop():
    # End-to-end shape of the P1 fix: a clear corridor with scattered
    # dropouts must not read as "obstacle at 0.0m".
    ranges = [5.0] * 50
    for i in (3, 17, 30):
        ranges[i] = float('nan')
    clean, valid = gap_logic.sanitize_ranges(ranges, max_range=10.0, range_min=0.05)
    _, dist = gap_logic.closest_valid(clean, valid)
    assert dist == pytest.approx(5.0)  # nowhere near an e-stop threshold


# ============================================================================
# disparity_extend: obstacle edges widened by half a car width
# ============================================================================

def _corner_scene(n=100, near=2.0, far=8.0, edge=50):
    """Wall at `near` up to (not including) index `edge`, open at `far` after."""
    scene = np.full(n, far)
    scene[:edge] = near
    return scene


def test_disparity_extend_widens_the_far_side_of_an_edge():
    inc = 0.01
    scene = _corner_scene()
    out = gap_logic.disparity_extend(scene, inc, disparity_threshold=0.4, extend_width_m=0.25)
    expected_beams = math.ceil(math.atan2(0.25, 2.0) / inc)
    # The far side directly past the edge now reads the near distance...
    assert np.allclose(out[50:50 + expected_beams], 2.0)
    # ...and beyond the extension it is untouched.
    assert np.allclose(out[50 + expected_beams:], 8.0)
    # The near side itself is unchanged.
    assert np.allclose(out[:50], 2.0)


def test_disparity_extend_never_raises_a_range():
    inc = 0.01
    scene = _corner_scene()
    out = gap_logic.disparity_extend(scene, inc, disparity_threshold=0.4, extend_width_m=0.25)
    assert np.all(out <= scene + 1e-12)


def test_disparity_extend_reaches_further_for_closer_edges():
    # Angular clearance must grow as the edge gets closer -- the whole
    # point of sizing by atan2(width, distance) instead of a fixed angle.
    inc = 0.01
    near_scene = _corner_scene(near=0.5)
    far_scene = _corner_scene(near=4.0)
    near_out = gap_logic.disparity_extend(near_scene, inc, 0.4, 0.25)
    far_out = gap_logic.disparity_extend(far_scene, inc, 0.4, 0.25)
    assert np.count_nonzero(near_out == 0.5) - 50 > np.count_nonzero(far_out == 4.0) - 50


def test_disparity_extend_handles_an_edge_in_the_other_direction():
    inc = 0.01
    scene = _corner_scene()[::-1].copy()  # open first, wall after
    out = gap_logic.disparity_extend(scene, inc, disparity_threshold=0.4, extend_width_m=0.25)
    expected_beams = math.ceil(math.atan2(0.25, 2.0) / inc)
    assert np.allclose(out[50 - expected_beams:50], 2.0)
    assert np.allclose(out[:50 - expected_beams], 8.0)


def test_disparity_extend_ignores_smooth_walls():
    # A gently receding wall (no jump above the threshold) is not an
    # edge -- nothing to extend.
    scene = np.linspace(2.0, 4.0, 100)
    out = gap_logic.disparity_extend(scene, 0.01, disparity_threshold=0.4, extend_width_m=0.25)
    assert np.allclose(out, scene)


def test_disparity_extend_skips_invalid_zero_edges():
    # A jump against an invalid (0.0) beam has no meaningful distance to
    # extend at -- and must not smear half the scan.
    scene = np.full(100, 5.0)
    scene[50] = 0.0
    out = gap_logic.disparity_extend(scene, 0.01, disparity_threshold=0.4, extend_width_m=0.25)
    assert np.allclose(np.delete(out, 50), 5.0)


# ============================================================================
# safety_bubble: width-based, not fixed-angle
# ============================================================================

def test_safety_bubble_zeroes_around_the_closest_point():
    window = np.full(100, 5.0)
    window[40] = 1.0
    out = gap_logic.safety_bubble(window, closest_idx=40, closest_dist=1.0,
                                  angle_increment=0.01, bubble_width_m=0.25)
    radius = math.ceil(math.atan2(0.25, 1.0) / 0.01)
    assert np.all(out[40 - radius:40 + radius + 1] == 0.0)
    assert out[40 - radius - 1] == 5.0 and out[40 + radius + 1] == 5.0
    assert window[40] == 1.0  # input untouched


def test_safety_bubble_is_wider_for_closer_obstacles():
    window = np.full(200, 5.0)
    near = gap_logic.safety_bubble(window, 100, 0.3, 0.01, 0.25)
    far = gap_logic.safety_bubble(window, 100, 3.0, 0.01, 0.25)
    assert np.count_nonzero(near == 0.0) > np.count_nonzero(far == 0.0)


# ============================================================================
# find_best_gap: best drivable opening, now car-width-aware
# ============================================================================

def test_best_gap_prefers_deep_corridor_over_shallow_alcove():
    window = np.full(100, 0.5)
    window[10:40] = 2.5    # wide but shallow pocket
    window[60:80] = 9.0    # narrower but genuinely deep corridor
    start, end = gap_logic.find_best_gap(window, min_gap_distance=2.0)
    assert (start, end) == (60, 79)


def test_best_gap_rejects_gaps_narrower_than_the_car():
    # One "gap" of a couple of beams at moderate depth: angularly real,
    # physically impassable -- must be discarded, not returned as the
    # least-bad option.
    inc = 0.01
    window = np.full(100, 0.5)
    window[50:52] = 3.0  # physical width ~ 2 * 0.01 * 3.0 = 0.06m
    start, end = gap_logic.find_best_gap(window, min_gap_distance=2.0,
                                         angle_increment=inc, min_gap_width_m=0.40)
    assert start is None and end is None


def test_best_gap_accepts_gaps_wider_than_the_car():
    inc = 0.01
    window = np.full(100, 0.5)
    window[40:70] = 3.0  # physical width ~ 30 * 0.01 * 3.0 = 0.90m
    start, end = gap_logic.find_best_gap(window, min_gap_distance=2.0,
                                         angle_increment=inc, min_gap_width_m=0.40)
    assert (start, end) == (40, 69)


def test_best_gap_returns_none_when_boxed_in():
    window = np.full(50, 0.8)
    start, end = gap_logic.find_best_gap(window, min_gap_distance=2.0)
    assert start is None and end is None
