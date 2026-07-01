"""
Unit tests for pure_pursuit.racing_math.

These test the actual formulas used to steer and pace the car, using
plain synthetic geometry (circles, straight lines, a stadium shape) with
known-by-construction answers -- no ROS, no hardware, no simulator
needed. Run with either:

    cd ~/racerbot-ws
    python3 -m pytest src/pure_pursuit/test/test_racing_math.py -v

or, once the package is built:

    colcon test --packages-select pure_pursuit --event-handlers console_direct+
"""
import math
import os
import sys

import numpy as np
import pytest

# Make `import pure_pursuit` resolve to the package source directory
# (the parent of this test/ folder) when running plain `pytest` directly
# against the source tree, without needing the package installed first.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pure_pursuit import racing_math  # noqa: E402


# ============================================================================
# Frame conversions
# ============================================================================

def test_quaternion_to_yaw_identity():
    assert racing_math.quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


def test_quaternion_to_yaw_90_degrees():
    half = math.pi / 4.0  # quaternion for +90deg about Z is (0,0,sin(45deg),cos(45deg))
    yaw = racing_math.quaternion_to_yaw(0.0, 0.0, math.sin(half), math.cos(half))
    assert yaw == pytest.approx(math.pi / 2.0)


def test_world_to_body_straight_ahead():
    # Car facing +X world (yaw=0); target 2m further along +X is straight
    # ahead -> y_body should be 0.
    x_b, y_b = racing_math.world_to_body(dx=2.0, dy=0.0, yaw=0.0)
    assert x_b == pytest.approx(2.0)
    assert y_b == pytest.approx(0.0)


def test_world_to_body_target_to_the_left():
    x_b, y_b = racing_math.world_to_body(dx=1.0, dy=1.0, yaw=0.0)
    assert y_b > 0.0


def test_world_to_body_accounts_for_heading():
    # Car facing +Y world (yaw=90deg); a target further along +Y world is
    # "straight ahead" in the body frame regardless of world direction.
    x_b, y_b = racing_math.world_to_body(dx=0.0, dy=3.0, yaw=math.pi / 2.0)
    assert x_b == pytest.approx(3.0)
    assert y_b == pytest.approx(0.0, abs=1e-9)


# ============================================================================
# Pure pursuit steering geometry
# ============================================================================

def test_steering_arc_curvature_target_dead_ahead_is_straight():
    kappa = racing_math.steering_arc_curvature(x_body=5.0, y_body=0.0)
    assert kappa == pytest.approx(0.0)


def test_steering_arc_curvature_sign_matches_left_right():
    left = racing_math.steering_arc_curvature(x_body=2.0, y_body=1.0)
    right = racing_math.steering_arc_curvature(x_body=2.0, y_body=-1.0)
    assert left > 0.0
    assert right < 0.0
    assert left == pytest.approx(-right)


def test_steering_from_curvature_matches_bicycle_model():
    wheelbase = 0.25
    kappa = 2.0
    delta = racing_math.steering_from_curvature(kappa, wheelbase)
    assert delta == pytest.approx(math.atan(wheelbase * kappa))


def test_pure_pursuit_end_to_end_steers_left_for_a_left_target():
    # Car at the origin facing +X (yaw=0); target ahead and to the left.
    # The full chain (world_to_body -> curvature -> steering) should
    # produce a positive (left) steering angle, matching
    # AckermannDriveStamped's convention.
    dx, dy = 2.0, 0.5
    x_b, y_b = racing_math.world_to_body(dx, dy, yaw=0.0)
    kappa = racing_math.steering_arc_curvature(x_b, y_b)
    delta = racing_math.steering_from_curvature(kappa, wheelbase=0.25)
    assert delta > 0.0


def test_adaptive_lookahead_clips_to_bounds():
    lo = racing_math.adaptive_lookahead(0.0, gain=0.35, min_lookahead=0.6, max_lookahead=2.5)
    hi = racing_math.adaptive_lookahead(100.0, gain=0.35, min_lookahead=0.6, max_lookahead=2.5)
    mid = racing_math.adaptive_lookahead(2.0, gain=0.35, min_lookahead=0.6, max_lookahead=2.5)
    assert lo == pytest.approx(0.6)
    assert hi == pytest.approx(2.5)
    assert mid == pytest.approx(0.35 * 2.0 + 0.6)


# ============================================================================
# Path indexing
# ============================================================================

def test_segment_lengths_closed_vs_open():
    xy = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    closed = racing_math.compute_segment_lengths(xy, closed=True)
    open_ = racing_math.compute_segment_lengths(xy, closed=False)
    assert closed[0] == pytest.approx(1.0)
    assert closed[1] == pytest.approx(1.0)
    assert closed[2] == pytest.approx(math.hypot(1.0, 1.0))  # wraps back to (0,0)
    assert open_[2] == pytest.approx(0.0)  # no wraparound for an open path


def test_find_nearest_index_and_cross_track_error():
    xy = np.column_stack([np.arange(0.0, 10.0, 1.0), np.zeros(10)])
    idx, err = racing_math.find_nearest_index(xy, (4.9, 0.3), closed=False, prev_index=None, search_window=None)
    assert idx == 5
    assert err == pytest.approx(math.hypot(0.1, 0.3))


def test_find_nearest_index_search_window_restricts_candidates():
    # A track that comes back close to itself (a thin hairpin): indices 1
    # and 4 are spatially close but far apart along the path. With a
    # window anchored on prev_index=1, index 4 must never even be
    # considered, regardless of which point is geometrically closer.
    xy = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 0.01], [1.0, 0.01], [0.0, 0.01]])
    car_xy = (1.0, 0.005)  # equidistant from index 1 and index 4
    idx_windowed, _ = racing_math.find_nearest_index(xy, car_xy, closed=True, prev_index=1, search_window=1)
    assert idx_windowed == 1


def test_find_lookahead_index_walks_forward_expected_distance():
    xy = np.column_stack([np.arange(0.0, 10.0, 1.0), np.zeros(10)])
    seg_len = racing_math.compute_segment_lengths(xy, closed=False)
    # From index 2, 1m segments: index 5 is 3.0m away (not yet enough),
    # index 6 is 4.0m away (first index at/past the 3.5m lookahead).
    idx = racing_math.find_lookahead_index(seg_len, nearest_index=2, lookahead_dist=3.5, closed=False)
    assert idx == 6


def test_find_lookahead_index_wraps_around_a_closed_loop():
    # A unit square, closed -- all four sides are exactly 1m, including
    # the "closing" side back from the last point to the first, so the
    # wraparound distance is as uniform and easy to hand-check as the
    # rest of the path.
    xy = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    # From index 2, needing 2.5m: index 3 is 1.0m away, wrapping to index 0
    # is 2.0m away, index 1 is 3.0m away -> first index at/past 2.5m is 1.
    idx = racing_math.find_lookahead_index(seg_len, nearest_index=2, lookahead_dist=2.5, closed=True)
    assert idx == 1


# ============================================================================
# Offline raceline generation
# ============================================================================

def _circle_points(radius, n=200):
    angles = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    return np.column_stack([radius * np.cos(angles), radius * np.sin(angles)])


def test_curvature_of_a_circle_matches_1_over_r():
    radius = 3.0
    xy = _circle_points(radius, n=200)
    kappa = racing_math.estimate_path_curvature(xy, closed=True)
    assert kappa == pytest.approx(1.0 / radius, rel=1e-2)


def test_curvature_of_a_straight_line_is_near_zero():
    xy = np.column_stack([np.linspace(0.0, 10.0, 50), np.zeros(50)])
    kappa = racing_math.estimate_path_curvature(xy, closed=False)
    assert np.max(np.abs(kappa)) < 1e-9


def test_velocity_profile_uniform_on_constant_curvature_loop():
    # On a pure circle the cornering-speed limit is the same everywhere,
    # so the forward/backward smoothing passes should have nothing to do
    # -- the whole loop should end up at one uniform speed.
    radius = 3.0
    xy = _circle_points(radius, n=200)
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    kappa = racing_math.estimate_path_curvature(xy, closed=True)
    a_lat_max = 8.0
    speed = racing_math.compute_velocity_profile(
        seg_len, kappa, v_max=100.0, v_min=0.1,
        a_lat_max=a_lat_max, a_accel_max=3.0, a_brake_max=8.0, closed=True)
    expected = math.sqrt(a_lat_max / (1.0 / radius))
    assert speed == pytest.approx(expected, rel=1e-2)


def _stadium_points(straight_len=20.0, radius=0.6, n_straight=80, n_arc=40):
    """A straight leading into a tight hairpin, then a long return leg --
    enough like a real track to exercise braking-before-a-corner."""
    straight = np.column_stack([np.linspace(0.0, straight_len, n_straight), np.zeros(n_straight)])
    hairpin = _circle_points(radius, n=n_arc) + np.array([straight_len, radius])
    return np.vstack([straight, hairpin]), n_straight, n_arc


def test_velocity_profile_brakes_before_a_tight_corner():
    xy, n_straight, n_arc = _stadium_points()
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    kappa = racing_math.estimate_path_curvature(xy, closed=True)
    speed = racing_math.compute_velocity_profile(
        seg_len, kappa, v_max=6.0, v_min=0.5,
        a_lat_max=8.0, a_accel_max=3.0, a_brake_max=8.0, closed=True)

    # Cruising near v_max somewhere on the straight...
    assert speed[10:60].max() == pytest.approx(6.0, rel=1e-2)
    # ...clearly slower through the tight hairpin...
    assert speed[n_straight:n_straight + 20].min() < 3.0
    # ...with a genuine braking *zone* on the approach (monotonically
    # non-increasing), not a single-step drop right at the corner.
    approach = speed[60:n_straight]
    assert np.all(np.diff(approach) <= 1e-6)


def test_velocity_profile_never_exceeds_braking_capability():
    xy, n_straight, n_arc = _stadium_points()
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    kappa = racing_math.estimate_path_curvature(xy, closed=True)
    a_brake_max = 8.0
    speed = racing_math.compute_velocity_profile(
        seg_len, kappa, v_max=6.0, v_min=0.5,
        a_lat_max=8.0, a_accel_max=3.0, a_brake_max=a_brake_max, closed=True)

    n = len(speed)
    for i in range(n):
        j = (i + 1) % n
        if speed[i] > speed[j] + 1e-9:  # a braking segment: i is faster than i+1
            allowed = math.sqrt(speed[j] ** 2 + 2.0 * a_brake_max * seg_len[i]) + 1e-6
            assert speed[i] <= allowed


def test_velocity_profile_respects_v_max_and_v_min():
    xy, _, _ = _stadium_points()
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    kappa = racing_math.estimate_path_curvature(xy, closed=True)
    speed = racing_math.compute_velocity_profile(
        seg_len, kappa, v_max=6.0, v_min=0.5,
        a_lat_max=8.0, a_accel_max=3.0, a_brake_max=8.0, closed=True)
    assert speed.max() <= 6.0 + 1e-9
    assert speed.min() >= 0.5 - 1e-9


def test_estimate_lap_time_positive_and_matches_constant_speed_case():
    xy = _circle_points(radius=3.0, n=100)
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    speed = np.full(len(xy), 2.0)
    lap_time = racing_math.estimate_lap_time(seg_len, speed, closed=True)
    circumference = 2.0 * math.pi * 3.0
    assert lap_time == pytest.approx(circumference / 2.0, rel=1e-2)


# ============================================================================
# CSV file I/O
# ============================================================================

def test_csv_round_trip(tmp_path):
    xy = np.array([[0.0, 0.0], [1.0, 2.0], [3.0, 4.0]])
    speed = np.array([1.0, 2.0, 3.0])
    path = str(tmp_path / 'profiled.csv')
    racing_math.save_profiled_csv(path, xy, speed)
    xy2, speed2 = racing_math.load_profiled_csv(path)
    assert np.allclose(xy, xy2, atol=1e-3)
    assert np.allclose(speed, speed2, atol=1e-3)


def test_load_profiled_csv_rejects_raw_file(tmp_path):
    xy = np.array([[0.0, 0.0], [1.0, 2.0], [3.0, 4.0]])
    path = str(tmp_path / 'raw.csv')
    racing_math.save_xy_csv(path, xy)
    with pytest.raises(ValueError):
        racing_math.load_profiled_csv(path)
