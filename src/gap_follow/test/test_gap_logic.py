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


def test_curvature_speed_limit_is_fast_straight_and_slow_in_turn():
    assert gap_logic.curvature_speed_limit(0.0, 1.0, 2.0) == pytest.approx(2.0)
    assert gap_logic.curvature_speed_limit(1.0, 1.0, 2.0) == pytest.approx(1.0)
    assert gap_logic.curvature_speed_limit(-4.0, 1.0, 2.0) == pytest.approx(0.5)


def test_braking_speed_limit_reserves_stopping_clearance():
    assert gap_logic.braking_speed_limit(
        math.inf, 0.25, 3.0, 2.0) == pytest.approx(2.0)
    assert gap_logic.braking_speed_limit(
        0.25, 0.25, 3.0, 2.0) == pytest.approx(0.0)
    assert gap_logic.braking_speed_limit(
        0.75, 0.25, 3.0, 2.0) == pytest.approx(math.sqrt(3.0))


def test_slew_rate_limit_has_asymmetric_rise_and_fall():
    assert gap_logic.slew_rate_limit(2.0, 1.0, 0.1, 2.0, 4.0) == pytest.approx(1.2)
    assert gap_logic.slew_rate_limit(0.0, 1.0, 0.1, 2.0, 4.0) == pytest.approx(0.6)
    assert gap_logic.slew_rate_limit(-1.0, 0.0, 0.1, 1.5) == pytest.approx(-0.15)


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

# ============================================================================
# Rectangular footprint and instantaneous TTC
# ============================================================================


def _body_boundaries(angles):
    return gap_logic.vehicle_boundary_distances(
        np.asarray(angles, dtype=float),
        car_width=0.31,
        car_length=0.58,
        wheelbase=0.324,
        laser_offset_x=0.33,
    )


def test_vehicle_boundary_matches_padded_traxxas_rectangle():
    boundaries = _body_boundaries([0.0, math.pi / 2.0, math.pi])
    # base_link is the rear axle. Body center is x=wheelbase/2=0.162m,
    # so the padded 0.58m rectangle spans x=[-0.128, 0.452].
    assert boundaries[0] == pytest.approx(0.122)
    assert boundaries[1] == pytest.approx(0.155)
    assert boundaries[2] == pytest.approx(0.458)


def test_vehicle_boundary_rejects_lidar_outside_footprint():
    with pytest.raises(ValueError, match='LiDAR origin'):
        gap_logic.vehicle_boundary_distances(
            np.array([0.0]), 0.31, 0.58, 0.324, laser_offset_x=0.50)


def test_minimum_clearance_is_measured_from_body_not_lidar():
    boundaries = _body_boundaries([0.0, math.pi / 2.0])
    ranges = boundaries + np.array([0.40, 0.05])
    clearance = gap_logic.minimum_footprint_clearance(
        ranges, np.array([True, True]), boundaries)
    assert clearance == pytest.approx(0.05)


def test_ttc_projects_odometry_speed_and_subtracts_footprint():
    angles = np.array([-math.pi / 2.0, 0.0, math.pi / 3.0])
    boundaries = _body_boundaries(angles)
    ranges = boundaries + np.array([0.01, 1.0, 0.5])
    ttc = gap_logic.time_to_collision(
        ranges,
        np.ones(3, dtype=bool),
        angles,
        speed=2.0,
        boundary_distances=boundaries,
    )
    assert math.isinf(ttc[0])  # side wall: no longitudinal closing rate
    assert ttc[1] == pytest.approx(0.5)
    assert ttc[2] == pytest.approx(0.5)


def test_ttc_ignores_invalid_beams_and_stationary_vehicle():
    angles = np.array([0.0, 0.1])
    boundaries = _body_boundaries(angles)
    ranges = boundaries + np.array([0.01, 2.0])
    validity = np.array([False, True])
    assert gap_logic.minimum_ttc(
        ranges, validity, angles, 0.0, boundaries) == math.inf
    assert gap_logic.minimum_ttc(
        ranges, validity, angles, 1.0, boundaries) > 1.9


def test_forward_clearance_cone_ignores_close_side_obstacles():
    angles = np.array([-math.radians(45.0), 0.0, math.radians(45.0)])
    boundaries = _body_boundaries(angles)
    ranges = boundaries + np.array([0.01, 0.30, 0.01])
    clearance = gap_logic.minimum_footprint_clearance_in_cone(
        ranges,
        np.ones(3, dtype=bool),
        angles,
        boundaries,
        cone_width_rad=math.radians(60.0),
    )
    assert clearance == pytest.approx(0.30)


def test_forward_clearance_cone_reports_frontal_obstacle():
    angles = np.array([-0.20, 0.0, 0.20])
    boundaries = _body_boundaries(angles)
    ranges = boundaries + np.array([0.40, 0.12, 0.35])
    clearance = gap_logic.minimum_footprint_clearance_in_cone(
        ranges,
        np.ones(3, dtype=bool),
        angles,
        boundaries,
        cone_width_rad=math.radians(60.0),
    )
    assert clearance == pytest.approx(0.12)


def test_conservative_ttc_speed_uses_recent_positive_command():
    assert gap_logic.conservative_ttc_speed(
        measured_speed=0.0,
        commanded_speed=1.4,
        command_age_sec=0.1,
        command_timeout_sec=0.5,
    ) == pytest.approx(1.4)


def test_conservative_ttc_speed_keeps_higher_measured_speed():
    assert gap_logic.conservative_ttc_speed(
        measured_speed=1.8,
        commanded_speed=1.0,
        command_age_sec=0.1,
        command_timeout_sec=0.5,
    ) == pytest.approx(1.8)


def test_conservative_ttc_speed_trusts_meaningful_fresh_odom():
    assert gap_logic.conservative_ttc_speed(
        measured_speed=0.5,
        commanded_speed=1.8,
        command_age_sec=0.1,
        command_timeout_sec=0.5,
        fallback_max_measured_speed=0.1,
    ) == pytest.approx(0.5)


def test_conservative_ttc_speed_ignores_stale_command():
    assert gap_logic.conservative_ttc_speed(
        measured_speed=0.0,
        commanded_speed=1.5,
        command_age_sec=0.51,
        command_timeout_sec=0.5,
    ) == 0.0


def test_conservative_ttc_speed_uses_reverse_motion_magnitude():
    """A stale command must not hide real motion just because it reads
    negative -- the magnitude is what the TTC clock runs on."""
    assert gap_logic.conservative_ttc_speed(
        measured_speed=-0.2,
        commanded_speed=1.5,
        command_age_sec=0.51,
        command_timeout_sec=0.5,
    ) == pytest.approx(0.2)


def test_conservative_ttc_speed_survives_inverted_odometry_sign():
    """Regression: the 2026-07-27 collision.

    With a sign-inverted /odom, a car really doing 1.8m/s reported -1.8m/s.
    The old signed comparison read that as stationary, fell through to the
    commanded speed -- which is 0 for exactly one tick after every brake --
    and reported 0m/s, so TTC went infinite and released the brake on the
    next scan. The magnitude must survive both the sign and the zero
    command that a brake leaves behind.
    """
    assert gap_logic.conservative_ttc_speed(
        measured_speed=-1.8,
        commanded_speed=0.0,
        command_age_sec=0.02,
        command_timeout_sec=0.5,
        fallback_max_measured_speed=0.1,
    ) == pytest.approx(1.8)


def test_conservative_ttc_speed_rejects_invalid_age():
    with pytest.raises(ValueError, match='command_age_sec'):
        gap_logic.conservative_ttc_speed(0.0, 1.0, float('nan'), 0.5)


# ============================================================================
# Tight-corner fallback after obstacle inflation
# ============================================================================


def test_gap_fallback_accepts_passable_corner_hidden_inside_two_metres():
    window = np.full(100, 0.5)
    window[30:70] = 1.2
    start, end, used_fallback = gap_logic.find_gap_with_fallback(
        window,
        preferred_distance=2.0,
        fallback_distance=0.8,
        angle_increment=0.01,
        min_gap_width_m=0.10,
    )
    assert (start, end) == (30, 69)
    assert used_fallback


def test_gap_fallback_keeps_preferred_deep_gap_when_available():
    window = np.full(100, 0.5)
    window[30:70] = 3.0
    start, end, used_fallback = gap_logic.find_gap_with_fallback(
        window, 2.0, 0.8, 0.01, 0.10)
    assert (start, end) == (30, 69)
    assert not used_fallback


def test_gap_fallback_still_rejects_boxed_in_scene():
    window = np.full(100, 0.7)
    start, end, used_fallback = gap_logic.find_gap_with_fallback(
        window, 2.0, 0.8, 0.01, 0.10)
    assert start is None and end is None
    assert not used_fallback


def test_post_inflation_gap_does_not_require_second_full_car_width():
    # Disparity extension has already removed half-width + margin from each
    # obstacle edge. A 0.12m remaining center corridor is valid; applying the
    # old ~0.41m full-car filter here would double-pad it and cause a stop.
    window = np.full(100, 0.5)
    window[44:56] = 1.0
    start, end = gap_logic.find_best_gap(
        window, 0.8, angle_increment=0.01, min_gap_width_m=0.10)
    assert (start, end) == (44, 55)


# ============================================================================
# TTC swept-corridor gate: obstacles the car drives PAST are not collisions
# ============================================================================

def _beams(angles_deg, ranges):
    ang = np.radians(np.array(angles_deg, dtype=float))
    r = np.array(ranges, dtype=float)
    return r, np.ones_like(r, dtype=bool), ang, np.zeros_like(r)


def test_ttc_ignores_a_wall_the_car_drives_past():
    """Regression: the 1m-course crawl.

    iTTC projects speed radially onto every beam, so a wall well off to the
    side reads as an approach that can never happen. On a wide track that is
    harmless; in a 1m corridor it capped this car at 1.42m/s while perfectly
    centred, and at 0.27m/s once it drifted toward one wall.
    """
    # Wall 0.42m to the side, seen at 45deg => range 0.60m.
    r, valid, ang, bnd = _beams([45.0], [0.60])
    ungated = gap_logic.minimum_ttc(r, valid, ang, 2.0, bnd)
    gated = gap_logic.minimum_ttc(r, valid, ang, 2.0, bnd,
                                  swept_half_width=0.255)
    assert ungated < 0.5, 'the ungated model must show the phantom approach'
    assert gated == math.inf, 'a wall 0.5m to the side is driven past, not hit'


def test_ttc_still_brakes_for_an_obstacle_dead_ahead():
    r, valid, ang, bnd = _beams([0.0], [0.4])
    assert gap_logic.minimum_ttc(
        r, valid, ang, 2.0, bnd, swept_half_width=0.255) == pytest.approx(0.2)


def test_ttc_still_brakes_for_an_obstacle_inside_the_swept_width():
    # 0.15m to the side at 30deg => range 0.30m, inside a 0.255m half-width.
    r, valid, ang, bnd = _beams([30.0], [0.30])
    assert gap_logic.minimum_ttc(
        r, valid, ang, 1.0, bnd, swept_half_width=0.255) < math.inf


def test_ttc_swept_corridor_follows_the_turn():
    """A car at full lock curves around the outside of its own corner. Judging
    it as if it were going straight is what deadlocked it mid-turn."""
    # Obstacle 0.6m ahead, 0.1m to the left -- straight ahead that is a hit.
    r, valid, ang, bnd = _beams([9.5], [0.608])
    straight = gap_logic.minimum_ttc(
        r, valid, ang, 1.0, bnd, swept_half_width=0.255, path_curvature=0.0)
    turning = gap_logic.minimum_ttc(
        r, valid, ang, 1.0, bnd, swept_half_width=0.255,
        path_curvature=-0.821, laser_offset_x=0.33)
    assert straight < math.inf, 'straight ahead, this is on the path'
    assert turning == math.inf, 'turning hard right, the car goes around it'


# ============================================================================
# side_wall_distance: the perpendicular distance to one wall
# ============================================================================

def _corridor_scan(left_dist, right_dist, yaw=0.0, n=541, span_deg=135.0):
    """Synthetic 270deg scan inside a straight corridor.

    The car sits `left_dist` from the left wall and `right_dist` from the
    right, yawed `yaw` radians (positive = nose swung left). Returns
    (clean, valid, angles) as the node would after sanitize_ranges.
    """
    angles = np.linspace(-math.radians(span_deg), math.radians(span_deg), n)
    # Wall normals in the car frame rotate with the car's yaw.
    to_left = math.pi / 2.0 - yaw
    to_right = -math.pi / 2.0 - yaw
    ranges = np.full(n, 10.0)
    for i, a in enumerate(angles):
        hits = []
        cos_left = math.cos(a - to_left)
        if cos_left > 1e-3:
            hits.append(left_dist / cos_left)
        cos_right = math.cos(a - to_right)
        if cos_right > 1e-3:
            hits.append(right_dist / cos_right)
        if hits:
            ranges[i] = min(10.0, min(hits))
    return ranges, np.ones(n, dtype=bool), angles


def test_side_wall_distance_recovers_the_perpendicular_distance():
    clean, valid, angles = _corridor_scan(0.80, 1.40)
    left = gap_logic.side_wall_distance(clean, valid, angles, math.pi / 2, math.radians(30))
    right = gap_logic.side_wall_distance(clean, valid, angles, -math.pi / 2, math.radians(30))
    assert left == pytest.approx(0.80, abs=0.01)
    assert right == pytest.approx(1.40, abs=0.01)


def test_side_wall_distance_is_yaw_tolerant_inside_the_window():
    """A beam hits a straight wall at d/cos(theta), minimised at the
    perpendicular -- so a yawed car still measures the true distance as long
    as the perpendicular is inside the window."""
    clean, valid, angles = _corridor_scan(0.80, 1.40, yaw=math.radians(20))
    left = gap_logic.side_wall_distance(clean, valid, angles, math.pi / 2, math.radians(30))
    right = gap_logic.side_wall_distance(clean, valid, angles, -math.pi / 2, math.radians(30))
    assert left == pytest.approx(0.80, abs=0.02)
    assert right == pytest.approx(1.40, abs=0.02)


def test_side_wall_distance_survives_a_doorway_mid_window():
    """A hole in the wall must not read as 'acres of room on this side' --
    the minimum keeps the surrounding wall."""
    clean, valid, angles = _corridor_scan(0.80, 1.40)
    doorway = np.abs(angles - math.pi / 2) < math.radians(6)
    clean = clean.copy()
    clean[doorway] = 9.5
    left = gap_logic.side_wall_distance(clean, valid, angles, math.pi / 2, math.radians(30))
    assert left == pytest.approx(0.80, abs=0.03)


def test_side_wall_distance_is_infinite_with_no_valid_beam():
    clean, valid, angles = _corridor_scan(0.80, 1.40)
    assert gap_logic.side_wall_distance(
        clean, np.zeros_like(valid), angles, math.pi / 2, math.radians(30)) == math.inf


def test_side_wall_distance_ignores_invalid_beams():
    clean, valid, angles = _corridor_scan(0.80, 1.40)
    clean, valid = clean.copy(), valid.copy()
    near_left = np.abs(angles - math.pi / 2) < math.radians(4)
    clean[near_left] = 0.0      # dropout encoded as non-free
    valid[near_left] = False    # but not trustworthy
    left = gap_logic.side_wall_distance(clean, valid, angles, math.pi / 2, math.radians(30))
    assert left == pytest.approx(0.80, abs=0.03), 'a dropout is unknown, not contact'


# ============================================================================
# corridor_centering_bias: the cross-track term
# ============================================================================

CENTERING = dict(
    gain=0.25, max_bias=0.08,
    bearing_full=math.radians(4), bearing_zero=math.radians(15),
    depth_full=2.5, depth_zero=1.5,
    side_full=4.0, side_zero=5.0,
)


def _bias(left, right, bearing=0.0, depth=5.0, **overrides):
    kwargs = dict(CENTERING, **overrides)
    return gap_logic.corridor_centering_bias(left, right, bearing, depth, **kwargs)


def test_centering_is_silent_in_the_middle_of_the_corridor():
    bias, weight = _bias(1.0, 1.0)
    assert bias == pytest.approx(0.0)
    assert weight == pytest.approx(1.0)


def test_centering_steers_right_when_hugging_the_left_wall():
    bias, weight = _bias(0.30, 1.50)
    assert bias < 0.0, 'positive steering is left (REP-103), so this must be negative'
    assert weight == pytest.approx(1.0)


def test_centering_steers_left_when_hugging_the_right_wall():
    bias, _ = _bias(1.50, 0.30)
    assert bias > 0.0


def test_centering_is_proportional_to_the_offset():
    small, _ = _bias(0.90, 1.10)   # 0.10m off centre
    large, _ = _bias(0.80, 1.20)   # 0.20m off centre
    assert small == pytest.approx(-0.25 * 0.10)
    assert large == pytest.approx(-0.25 * 0.20)


def test_centering_is_clamped_to_its_authority_bound():
    """The clamp is the safety property: this term can never outvote the gap
    the obstacle-avoidance pipeline chose."""
    # 1.10m off centre asks for 0.275rad; it may only ever have 0.08.
    bias, weight = _bias(0.20, 2.40)
    assert weight == pytest.approx(1.0), 'both sides are real walls here'
    assert abs(bias) == pytest.approx(0.08)


def test_centering_fades_out_while_turning():
    assert _bias(0.30, 1.50, bearing=math.radians(3))[1] == pytest.approx(1.0)
    assert _bias(0.30, 1.50, bearing=math.radians(20))[1] == 0.0
    assert _bias(0.30, 1.50, bearing=math.radians(20))[0] == 0.0


def test_centering_fade_is_a_ramp_not_a_switch():
    """A hard on/off on a steering term is what produced the scan-rate
    chatter documented in aim_within_gap."""
    weights = [_bias(0.30, 1.50, bearing=math.radians(d))[1]
               for d in (4, 6, 8, 10, 12, 15)]
    assert weights[0] == pytest.approx(1.0)
    assert weights[-1] == 0.0
    assert all(a > b for a, b in zip(weights, weights[1:])), 'must decrease monotonically'
    assert all(abs(a - b) < 0.35 for a, b in zip(weights, weights[1:])), 'no cliff'


def test_centering_ignores_a_side_that_is_not_a_wall():
    """An opening on one side is not something to centre against -- otherwise
    the bias steers straight into it."""
    assert _bias(0.30, math.inf)[1] == 0.0
    assert _bias(0.30, 6.0)[1] == 0.0, 'past centering_zero_side_distance'
    assert _bias(0.30, 3.5)[1] == pytest.approx(1.0), 'inside it, this is a wall'


def test_centering_stops_before_a_corner():
    assert _bias(0.30, 1.50, depth=3.0)[1] == pytest.approx(1.0)
    assert _bias(0.30, 1.50, depth=1.2)[1] == 0.0


def test_centering_can_be_disabled_by_zero_gain():
    assert _bias(0.30, 1.50, gain=0.0)[0] == pytest.approx(0.0)


def test_centering_rejects_a_negative_authority_bound():
    with pytest.raises(ValueError):
        _bias(1.0, 1.0, max_bias=-0.1)
    with pytest.raises(ValueError):
        _bias(1.0, 1.0, gain=-1.0)


# ============================================================================
# Closed loop: does the law actually centre the car, without weaving?
# ============================================================================

def _drive_corridor(width, start_offset, steps=600, speed=1.5, dt=0.025,
                    wheelbase=0.324, steering_gain=1.0, max_steering=0.26,
                    centering=True):
    """Kinematic bicycle down a straight corridor under the real steering law.

    Mirrors gap_follow_node: the aim bearing on a straight is the corridor
    direction in the body frame (i.e. -yaw), the centering bias is added to
    it, and the sum is clipped to the steering limit. Returns the lateral
    offset history (0.0 = middle of the corridor, positive = left of it).
    """
    y, yaw = start_offset, 0.0
    history = []
    for _ in range(steps):
        left = width / 2.0 - y
        right = width / 2.0 + y
        aim_bearing = -yaw
        bias = _bias(left, right, bearing=aim_bearing)[0] if centering else 0.0
        steering = min(max_steering, max(-max_steering,
                                         steering_gain * aim_bearing + bias))
        yaw += speed / wheelbase * math.tan(steering) * dt
        y += speed * math.sin(yaw) * dt
        history.append(y)
    return np.array(history)


def test_centering_converges_to_the_middle_of_a_straight():
    history = _drive_corridor(width=2.0, start_offset=0.7)
    assert abs(history[-1]) < 0.05, (
        f'should settle in the middle, ended {history[-1]:+.3f}m off')


def test_centering_converges_from_either_side():
    for start in (0.7, -0.7, 0.35, -0.35):
        history = _drive_corridor(width=2.0, start_offset=start)
        assert abs(history[-1]) < 0.05, f'start {start:+.2f}m ended {history[-1]:+.3f}m'


def test_centering_does_not_weave():
    """The bearing term is the damping: as the car turns toward the middle its
    heading tilts and the aim bearing swings back. Overshoot must stay small
    and must not grow."""
    history = _drive_corridor(width=2.0, start_offset=0.7)
    overshoot = history[history < 0.0]
    assert overshoot.size == 0 or abs(overshoot.min()) < 0.10, (
        f'overshoot {abs(overshoot.min()) if overshoot.size else 0:.3f}m is too much')
    # No growing oscillation: the second half must be quieter than the first.
    assert np.std(history[300:]) < np.std(history[:300])


def test_without_centering_the_car_holds_its_offset_forever():
    """The behaviour being fixed: bearing steering alone is parallel to the
    walls, so whatever offset the car entered the straight with, it keeps."""
    history = _drive_corridor(width=2.0, start_offset=0.7, centering=False)
    assert history[-1] == pytest.approx(0.7, abs=1e-6)


def test_centering_never_leaves_the_corridor():
    for start in (0.7, -0.7):
        history = _drive_corridor(width=2.0, start_offset=start)
        assert np.abs(history).max() <= abs(start) + 1e-9, 'must not go wider than it started'
