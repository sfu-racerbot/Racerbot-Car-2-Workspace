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


# The values gap_follow.yaml actually configures, so the numbers below are
# the distances the running car really computes. wheelbase and
# laser_offset_x were tape-measured 2026-08-24; car_width is the measured
# 0.30m over the tires plus 5mm a side of deliberate padding (cut from
# 14.5mm/side on 2026-08-25).
CAR_WIDTH = 0.31
CAR_LENGTH = 0.58
CAR_WHEELBASE = 0.36
CAR_LASER_OFFSET_X = 0.26


def _body_boundaries(angles):
    return gap_logic.vehicle_boundary_distances(
        np.asarray(angles, dtype=float),
        car_width=CAR_WIDTH,
        car_length=CAR_LENGTH,
        wheelbase=CAR_WHEELBASE,
        laser_offset_x=CAR_LASER_OFFSET_X,
    )


def test_vehicle_boundary_matches_the_padded_rectangle():
    boundaries = _body_boundaries([0.0, math.pi / 2.0, math.pi])
    # base_link is the rear axle, and the rectangle is centred halfway along
    # the wheelbase: centre x = 0.36/2 = 0.18, so the padded 0.58m box spans
    # x = [0.18 - 0.29, 0.18 + 0.29] = [-0.11, 0.47] and y = +/-0.155.
    # From the LiDAR at x = 0.26 that is 0.21m to the nose, 0.155m to either
    # flank, and 0.37m to the tail.
    assert boundaries[0] == pytest.approx(0.21)
    assert boundaries[1] == pytest.approx(0.155)
    assert boundaries[2] == pytest.approx(0.37)


def test_vehicle_boundary_rejects_lidar_outside_footprint():
    with pytest.raises(ValueError, match='LiDAR origin'):
        gap_logic.vehicle_boundary_distances(
            np.array([0.0]), CAR_WIDTH, CAR_LENGTH, CAR_WHEELBASE,
            laser_offset_x=0.50)


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


# ============================================================================
# find_best_gap hysteresis: sticking with the previous aim through a noisy
# near-tie, instead of re-picking the top score every tick regardless
# ============================================================================

def _two_candidates(a_width=20, b_width=21):
    """Two same-depth candidates at indices [10, 10+a_width) and
    [50, 50+b_width): B scores a_width/b_width - 1 fraction better than A."""
    window = np.full(100, 0.5)
    window[10:10 + a_width] = 2.0
    window[50:50 + b_width] = 2.0
    return window


def test_hysteresis_is_a_no_op_with_no_previous_target():
    window = _two_candidates()
    assert gap_logic.find_best_gap(window, 0.6, angle_increment=0.01) == (50, 70)


def test_hysteresis_is_a_no_op_at_the_default_switch_margin():
    window = _two_candidates()
    assert gap_logic.find_best_gap(
        window, 0.6, angle_increment=0.01,
        previous_target_idx=15, switch_margin=1.0) == (50, 70)


def test_hysteresis_keeps_the_previous_gap_through_a_small_score_edge():
    """B (score 42) beats A (score 40) by 5% -- under the 20% margin, so
    the previous tick's aim (inside A) is kept rather than flipping to B."""
    window = _two_candidates()  # A: 20*2.0=40, B: 21*2.0=42
    start, end = gap_logic.find_best_gap(
        window, 0.6, angle_increment=0.01,
        previous_target_idx=15, switch_margin=1.2)
    assert (start, end) == (10, 29), 'stayed with A despite B scoring slightly higher'


def test_hysteresis_still_switches_for_a_real_improvement():
    """B now scores 50% better than A -- past the 20% margin, so the car
    does re-aim even though the previous tick was in A."""
    window = _two_candidates(a_width=20, b_width=30)  # A: 40, B: 60
    start, end = gap_logic.find_best_gap(
        window, 0.6, angle_increment=0.01,
        previous_target_idx=15, switch_margin=1.2)
    assert (start, end) == (50, 79), 'a real improvement must still win'


def test_hysteresis_falls_through_when_the_previous_target_has_no_gap_left():
    window = _two_candidates()
    start, end = gap_logic.find_best_gap(
        window, 0.6, angle_increment=0.01,
        previous_target_idx=40, switch_margin=1.2)  # 40 is in neither candidate
    assert (start, end) == (50, 70), 'nothing to stick to, so plain best score wins'


def test_hysteresis_never_picks_a_candidate_that_failed_the_width_filter():
    """The sticky candidate still has to have passed min_gap_width_m --
    hysteresis chooses only among already-safe options, never resurrects
    one that was filtered out."""
    window = np.full(100, 0.5)
    window[10:12] = 2.0    # A: 2 beams -- 0.04m post-inflation, fails the filter
    window[50:70] = 2.0    # B: 20 beams -- 0.40m, the only surviving candidate
    start, end = gap_logic.find_best_gap(
        window, 0.6, angle_increment=0.01, min_gap_width_m=0.10,
        previous_target_idx=10, switch_margin=1.2)
    assert (start, end) == (50, 69)


def test_find_gap_with_fallback_applies_hysteresis_within_the_preferred_tier():
    window = _two_candidates()
    start, end, used_fallback = gap_logic.find_gap_with_fallback(
        window, preferred_distance=0.6, fallback_distance=0.3,
        angle_increment=0.01, min_gap_width_m=0.0,
        previous_target_idx=15, switch_margin=1.2)
    assert (start, end) == (10, 29)
    assert not used_fallback


# ============================================================================
# near_gap_bearing: an early look at a bend, for cornering anticipation
# ============================================================================

def test_near_gap_bearing_uses_only_the_close_beams():
    """10 near beams (1.0m) around bearing ~0.045rad, 10 far beams (5.0m)
    already swung to ~0.545rad -- simulating a bend the far/deep view (what
    actually steers, per aim_within_gap) has picked up on but the near
    view has not reached yet."""
    window = np.concatenate([np.full(10, 1.0), np.full(10, 5.0)])
    beam_angles = np.concatenate([
        np.linspace(0.0, 0.09, 10), np.linspace(0.5, 0.59, 10)])
    bearing = gap_logic.near_gap_bearing(window, 0, 19, beam_angles, near_depth=1.5)
    assert bearing == pytest.approx(float(np.mean(beam_angles[:10])))


def test_near_gap_bearing_is_none_with_no_near_beams():
    """Nothing to compare against -- there is no early look here at all,
    not the same thing as "no divergence"."""
    window = np.full(20, 5.0)
    beam_angles = np.linspace(0.0, 1.0, 20)
    assert gap_logic.near_gap_bearing(window, 0, 19, beam_angles, near_depth=1.0) is None


def test_near_gap_bearing_is_none_when_the_whole_gap_is_near():
    """A small room, not a corridor with something beyond the near view."""
    window = np.full(20, 1.0)
    beam_angles = np.linspace(0.0, 1.0, 20)
    assert gap_logic.near_gap_bearing(window, 0, 19, beam_angles, near_depth=5.0) is None


def test_near_gap_bearing_is_none_without_a_gap():
    assert gap_logic.near_gap_bearing(np.zeros(5), None, None, np.zeros(5), 1.0) is None


def test_near_gap_bearing_rejects_a_non_positive_near_depth():
    window, beam_angles = np.full(10, 1.0), np.linspace(0.0, 1.0, 10)
    with pytest.raises(ValueError):
        gap_logic.near_gap_bearing(window, 0, 9, beam_angles, near_depth=0.0)
    with pytest.raises(ValueError):
        gap_logic.near_gap_bearing(window, 0, 9, beam_angles, near_depth=-1.0)


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
        path_curvature=-math.tan(0.26) / CAR_WHEELBASE,
        laser_offset_x=CAR_LASER_OFFSET_X)
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
                    wheelbase=CAR_WHEELBASE, steering_gain=1.0, max_steering=0.26,
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


# ============================================================================
# corridor_width_factor / scale_between: adapting to a sensed corridor width
# ============================================================================

NARROW, REFERENCE = 1.4, 2.6


def test_width_factor_is_one_at_or_above_the_reference_width():
    assert gap_logic.corridor_width_factor(1.3, 1.3, NARROW, REFERENCE) == pytest.approx(1.0)
    assert gap_logic.corridor_width_factor(2.0, 2.0, NARROW, REFERENCE) == pytest.approx(1.0)


def test_width_factor_is_zero_at_or_below_the_narrow_width():
    assert gap_logic.corridor_width_factor(0.7, 0.7, NARROW, REFERENCE) == pytest.approx(0.0)
    assert gap_logic.corridor_width_factor(0.3, 0.3, NARROW, REFERENCE) == pytest.approx(0.0)


def test_width_factor_is_linear_between_them():
    # 2.0m is (2.0-1.4)/(2.6-1.4) = 0.5 of the way from narrow to reference.
    assert gap_logic.corridor_width_factor(1.0, 1.0, NARROW, REFERENCE) == pytest.approx(0.5)


def test_width_factor_ignores_a_side_that_is_not_a_wall():
    """The safe default when a side is unbounded: assume the wide,
    already-validated static tuning, do not loosen anything. This is what
    keeps the adaptation from acting on ambiguous mid-corner sensing."""
    assert gap_logic.corridor_width_factor(0.5, math.inf, NARROW, REFERENCE) == pytest.approx(1.0)
    assert gap_logic.corridor_width_factor(math.inf, math.inf, NARROW, REFERENCE) == pytest.approx(1.0)


def test_width_factor_rejects_a_narrow_width_that_does_not_fit():
    with pytest.raises(ValueError):
        gap_logic.corridor_width_factor(1.0, 1.0, 2.6, 1.4)
    with pytest.raises(ValueError):
        gap_logic.corridor_width_factor(1.0, 1.0, 0.0, 2.6)


def test_scale_between_endpoints():
    assert gap_logic.scale_between(0.0, 0.12, 0.18) == pytest.approx(0.12)
    assert gap_logic.scale_between(1.0, 0.12, 0.18) == pytest.approx(0.18)
    assert gap_logic.scale_between(0.5, 0.12, 0.18) == pytest.approx(0.15)


def test_scale_between_works_growing_or_shrinking():
    # safety_margin shrinks narrow->wide; corner_speed grows narrow->wide.
    # Same helper, opposite ordering of its two endpoints.
    assert gap_logic.scale_between(0.25, 0.8, 1.4) == pytest.approx(0.95)


def test_scale_between_rejects_a_width_factor_outside_zero_one():
    with pytest.raises(ValueError):
        gap_logic.scale_between(-0.01, 0.12, 0.18)
    with pytest.raises(ValueError):
        gap_logic.scale_between(1.01, 0.12, 0.18)


# --- Measured-response speed shaping (2026-10-20, docs/gap-follow.md) -------

def test_braking_speed_limit_with_latency_stops_within_the_room():
    # Oracle: the closed form it solves. Driving at v for `latency` seconds
    # and then braking at a must use exactly the usable room c - r.
    a, latency, clearance, reserve = 3.0, 0.25, 2.2, 0.2
    v = gap_logic.braking_speed_limit(clearance, reserve, a, 10.0, latency=latency)
    used = v * latency + v * v / (2.0 * a)
    assert used == pytest.approx(clearance - reserve, abs=1e-9)  # metres
    # Strictly slower than the no-latency ceiling for the same room.
    assert v < gap_logic.braking_speed_limit(clearance, reserve, a, 10.0)


def test_braking_speed_limit_latency_edges():
    # Zero latency reproduces the original formula exactly.
    assert gap_logic.braking_speed_limit(0.75, 0.25, 3.0, 9.0, latency=0.0) == pytest.approx(
        math.sqrt(3.0), abs=1e-12)  # m/s
    # No room left: zero, with or without latency.
    assert gap_logic.braking_speed_limit(0.25, 0.25, 3.0, 9.0, latency=0.3) == 0.0
    assert gap_logic.braking_speed_limit(0.1, 0.25, 3.0, 9.0, latency=0.3) == 0.0
    # Nothing in view still means max_speed.
    assert gap_logic.braking_speed_limit(math.inf, 0.25, 3.0, 2.0, latency=0.3) == 2.0
    # Capped at max_speed when the room would allow more.
    assert gap_logic.braking_speed_limit(50.0, 0.25, 3.0, 2.0, latency=0.3) == 2.0
    for bad in (-0.1, math.nan, math.inf):
        with pytest.raises(ValueError, match='latency'):
            gap_logic.braking_speed_limit(1.0, 0.25, 3.0, 2.0, latency=bad)


def test_speed_scheduled_gain_holds_crossover_constant_above_reference():
    # Invariant it exists for: crossover = speed * K * factor / wheelbase is
    # the same at every speed above the reference.
    ref = 1.8
    crossovers = [v * gap_logic.speed_scheduled_gain(v, ref) for v in (1.8, 2.5, 3.5, 4.9)]
    assert crossovers == pytest.approx([ref] * 4, abs=1e-12)  # m/s
    # At or below the reference, and when disabled, the full gain.
    assert gap_logic.speed_scheduled_gain(1.8, ref) == 1.0
    assert gap_logic.speed_scheduled_gain(0.0, ref) == 1.0
    assert gap_logic.speed_scheduled_gain(1.81, ref) < 1.0   # one step past the boundary
    assert gap_logic.speed_scheduled_gain(3.0, 0.0) == 1.0
    assert gap_logic.speed_scheduled_gain(3.0, -1.0) == 1.0
    with pytest.raises(ValueError, match='finite'):
        gap_logic.speed_scheduled_gain(math.nan, ref)
    with pytest.raises(ValueError, match='finite'):
        gap_logic.speed_scheduled_gain(2.0, math.inf)


def test_speed_hold_returns_the_lowest_desired_speed_of_the_window():
    hold = gap_logic.SpeedHold(0.25)
    # Constructed input: a dip to 1.0 at t=0.125 between 3.0s. Times are
    # multiples of 1/8 so every age is exact in binary floating point and the
    # boundary below is really tested, not rounded away.
    seq = [(0.0, 3.0), (0.125, 1.0), (0.25, 3.0), (0.3125, 3.0), (0.375, 3.0), (0.5, 3.0)]
    out = [hold.update(t, v) for t, v in seq]
    # The dip is held while it is less than 0.25 s old (t < 0.375), and
    # released exactly when it reaches 0.25 s old: the window is half-open.
    assert out == [3.0, 1.0, 1.0, 1.0, 3.0, 3.0]


def test_speed_hold_never_raises_and_resets():
    hold = gap_logic.SpeedHold(0.3)
    rng = np.random.default_rng(3)
    for k in range(200):
        desired = float(rng.uniform(0.0, 4.0))
        assert hold.update(0.025 * k, desired) <= desired   # invariant: only lowers
    # Reset must forget a low value that is still inside the window.
    hold.update(5.0, 0.5)
    hold.reset()
    assert hold.update(5.01, 2.5) == 2.5
    # A zero window passes every value straight through.
    passthrough = gap_logic.SpeedHold(0.0)
    assert [passthrough.update(t, v) for t, v in ((0, 2.0), (0.025, 3.0))] == [2.0, 3.0]
    for bad in (-0.1, math.nan, math.inf):
        with pytest.raises(ValueError, match='hold_sec'):
            gap_logic.SpeedHold(bad)
    with pytest.raises(ValueError, match='now'):
        gap_logic.SpeedHold(0.3).update(math.nan, 1.0)


def test_side_clearance_speed_limit_ramp_and_edges():
    # Closed form: narrow below the narrow end, max past the open end, a
    # straight line between (gap_logic.side_clearance_speed_limit docstring).
    f = lambda c: gap_logic.side_clearance_speed_limit(c, 2.0, 0.05, 0.15, 4.0)
    assert f(0.05) == 2.0
    assert f(-0.3) == 2.0                          # already touching: narrow
    assert f(0.15) == 4.0
    assert f(0.15 + 1e-9) == 4.0
    assert f(0.15 - 0.025) == pytest.approx(3.5, abs=1e-9)   # m/s
    assert f(0.075) == pytest.approx(2.5, abs=1e-9)          # m/s
    assert f(math.inf) == 4.0                       # no wall seen
    assert f(math.nan) == 2.0                       # unknown is narrow
    assert all(f(a) <= f(b) for a, b in zip(np.linspace(0, 0.2, 40), np.linspace(0, 0.2, 40)[1:]))


def test_side_clearance_speed_limit_rejects_bad_parameters():
    with pytest.raises(ValueError, match='narrow_speed must be within'):
        gap_logic.side_clearance_speed_limit(0.1, 5.0, 0.05, 0.15, 4.0)
    with pytest.raises(ValueError, match='narrow_clearance must be below'):
        gap_logic.side_clearance_speed_limit(0.1, 2.0, 0.15, 0.15, 4.0)
    with pytest.raises(ValueError, match='max_speed must be finite'):
        gap_logic.side_clearance_speed_limit(0.1, 2.0, 0.05, 0.15, math.inf)


# ============================================================================
# steering_damping: the gyro's yaw rate, opposed, in radians
# (docs/gap-follow.md#steering-damping-gyro)
# ============================================================================

def test_steering_damping_is_minus_gain_times_yaw_rate():
    # oracle: closed form -- the law is -gain_s * yaw_rate.
    assert gap_logic.steering_damping(0.5, 0.07) == pytest.approx(-0.035, abs=1e-12)  # rad


def test_a_left_yaw_rate_is_damped_to_the_right():
    # oracle: invariant -- yaw is positive to the left (imu_bridge: x forward,
    # y left, z up), so the damping steers right, and the mirror image steers
    # left.
    assert gap_logic.steering_damping(1.0, 0.07) < 0.0
    assert gap_logic.steering_damping(-1.0, 0.07) > 0.0
    assert gap_logic.steering_damping(1.0, 0.07) == pytest.approx(-0.07, abs=1e-12)  # rad
    assert gap_logic.steering_damping(-1.0, 0.07) == pytest.approx(0.07, abs=1e-12)  # rad


@pytest.mark.parametrize('yaw, gain', [(0.3, 0.05), (-2.0, 0.1), (4.0, 0.02)])
def test_damping_is_linear_in_yaw_rate_and_in_gain(yaw, gain):
    # oracle: invariant -- doubling either input doubles the term.
    base = gap_logic.steering_damping(yaw, gain)
    assert gap_logic.steering_damping(2.0 * yaw, gain) == pytest.approx(2.0 * base, abs=1e-12)  # rad
    assert gap_logic.steering_damping(yaw, 2.0 * gain) == pytest.approx(2.0 * base, abs=1e-12)  # rad


def test_zero_yaw_rate_adds_no_steering():
    assert gap_logic.steering_damping(0.0, 0.07) == 0.0


@pytest.mark.parametrize('yaw', [math.nan, math.inf, -math.inf])
def test_a_non_finite_yaw_rate_adds_exactly_zero(yaw):
    # A bad gyro sample must never become NaN steering
    # (docs/gap-follow.md#steering-damping-gyro).
    assert gap_logic.steering_damping(yaw, 0.07) == 0.0


@pytest.mark.parametrize('gain', [0.0, -0.07, -1.0, math.nan, math.inf])
def test_a_zero_negative_or_non_finite_gain_adds_exactly_zero(gain):
    # A negative gain would steer *with* the rotation (positive feedback), so
    # it must switch the term off, never flip it.
    assert gap_logic.steering_damping(0.5, gain) == 0.0


# ============================================================================
# Adaptive grip from the gyro (docs/gap-follow.md#adaptive-grip-imu)
# ============================================================================

GRIP_WHEELBASE = 0.36
# Demand that is judged against any limit up to the 3.0 ceiling: at least
# judge_fraction (0.8) of 3.0 is 2.4 m/s^2, below this.
FULL_DEMAND = 3.0


def _grip(limit=None, **overrides):
    """The shipped grip values (gap_follow.yaml), base 2.0, ceiling 3.0."""
    params = dict(base=2.0, ceiling=3.0, rise_per_s=0.2, cut_factor=0.85,
                  hold_ratio=0.95, slip_ratio=0.90, judge_fraction=0.8)
    params.update(overrides)
    return gap_logic.AdaptiveGrip(limit=limit, **params)


def _kinematic_yaw(speed, steering, wheelbase=GRIP_WHEELBASE):
    # oracle: the yaw rate of a car that does not slip, speed * tan(steer) / L.
    return speed * math.tan(steering) / wheelbase


# --- grip_ratio -------------------------------------------------------------

def test_grip_ratio_is_one_on_the_circle_the_steering_asks_for():
    # oracle: closed form. A 2 m radius circle at 2 m/s is 1.0 rad/s of yaw,
    # and the steering that draws it is atan(0.36 / 2).
    steering = math.atan(GRIP_WHEELBASE / 2.0)
    assert gap_logic.grip_ratio(1.0, 2.0, steering, GRIP_WHEELBASE) == pytest.approx(
        1.0, abs=1e-12)  # dimensionless


def test_grip_ratio_halves_when_the_car_yaws_half_as_fast():
    # oracle: closed form -- the ratio is linear in the measured yaw rate.
    steering = math.atan(GRIP_WHEELBASE / 2.0)
    assert gap_logic.grip_ratio(0.5, 2.0, steering, GRIP_WHEELBASE) == pytest.approx(
        0.5, abs=1e-12)  # dimensionless


def test_grip_ratio_reads_a_mirror_image_turn_the_same_way():
    # oracle: invariant -- a right turn (negative yaw and steering) has the same ratio.
    steering = math.atan(GRIP_WHEELBASE / 2.0)
    assert gap_logic.grip_ratio(-1.0, 2.0, -steering, GRIP_WHEELBASE) == pytest.approx(
        1.0, abs=1e-12)  # dimensionless


@pytest.mark.parametrize('speed', [0.0, 0.5, 0.999, -2.0])
def test_grip_ratio_makes_no_judgement_below_one_metre_per_second(speed):
    # oracle: spec -- speed < 1.0 returns nan. Reversing (-2.0) is below it too.
    assert math.isnan(gap_logic.grip_ratio(1.0, speed, 0.2, GRIP_WHEELBASE))


def test_grip_ratio_judges_at_exactly_one_metre_per_second():
    # One step past the guard: 1.0 m/s is judged, because the guard is 'below 1.0'.
    assert math.isfinite(gap_logic.grip_ratio(0.5, 1.0, 0.2, GRIP_WHEELBASE))


@pytest.mark.parametrize('steering', [0.0, 0.0499, -0.0499])
def test_grip_ratio_makes_no_judgement_with_the_rack_near_centre(steering):
    assert math.isnan(gap_logic.grip_ratio(0.1, 2.0, steering, GRIP_WHEELBASE))


def test_grip_ratio_judges_at_exactly_five_hundredths_of_a_radian():
    assert math.isfinite(gap_logic.grip_ratio(0.1, 2.0, 0.05, GRIP_WHEELBASE))


@pytest.mark.parametrize('yaw, speed, steering', [
    (math.nan, 2.0, 0.2), (math.inf, 2.0, 0.2), (1.0, math.nan, 0.2),
    (1.0, math.inf, 0.2), (1.0, 2.0, math.nan), (1.0, 2.0, -math.inf),
])
def test_grip_ratio_makes_no_judgement_on_non_finite_input(yaw, speed, steering):
    assert math.isnan(gap_logic.grip_ratio(yaw, speed, steering, GRIP_WHEELBASE))


def test_grip_ratio_makes_no_judgement_on_a_non_finite_wheelbase():
    assert math.isnan(gap_logic.grip_ratio(1.0, 2.0, 0.2, math.nan))


def test_grip_ratio_refuses_a_wheelbase_that_is_not_positive():
    with pytest.raises(ValueError, match='wheelbase'):
        gap_logic.grip_ratio(1.0, 2.0, 0.2, 0.0)


# --- LowPass ----------------------------------------------------------------

def test_low_pass_starts_at_its_first_sample():
    f = gap_logic.LowPass(0.3)
    assert f.update(1.5, 0.025) == 1.5


def test_low_pass_steps_toward_a_new_value_by_the_closed_form():
    # oracle: closed form. A first-order filter's step response is
    # x + (y0 - x) * exp(-t / tau). After 12 ticks of 0.025 s (t = 0.3 s, one
    # time constant) the remaining gap is exp(-1) of the starting gap.
    f = gap_logic.LowPass(0.3)
    f.update(0.0, 0.025)
    for _ in range(12):
        last = f.update(1.0, 0.025)
    assert last == pytest.approx(1.0 - math.exp(-1.0), abs=1e-9)  # dimensionless


def test_the_grip_ratio_is_low_passed_over_three_tenths_of_a_second():
    # oracle: spec -- the Task 15 plan and docs/gap-follow.md#adaptive-grip-imu:
    # the ratio is low-passed with a 0.3 s time constant. Twelve ticks of 0.025 s
    # is one time constant, so the step response is 1 - exp(-1) of the way there.
    f = gap_logic.LowPass(gap_logic.GRIP_RATIO_TAU_SEC)
    f.update(0.0, 0.025)
    for _ in range(12):
        last = f.update(1.0, 0.025)
    assert last == pytest.approx(1.0 - math.exp(-1.0), abs=1e-9)  # dimensionless


def test_low_pass_breaks_its_chain_on_nan_and_restarts_from_the_next_sample():
    # A NaN is no value. The next finite sample starts the filter afresh, so
    # nothing from before the gap is blended in.
    f = gap_logic.LowPass(0.3)
    f.update(0.0, 0.025)
    assert math.isnan(f.update(math.nan, 0.025))
    assert f.update(1.0, 0.025) == 1.0


def test_low_pass_reset_forgets_its_value():
    f = gap_logic.LowPass(0.3)
    f.update(5.0, 0.025)
    f.reset()
    assert f.update(1.0, 0.025) == 1.0


def test_low_pass_refuses_a_negative_dt_and_a_non_positive_time_constant():
    with pytest.raises(ValueError, match='dt'):
        gap_logic.LowPass(0.3).update(1.0, -0.01)
    with pytest.raises(ValueError, match='time_constant_s'):
        gap_logic.LowPass(0.0)


# --- AdaptiveGrip: the rise, the cut, and their bounds -----------------------

def test_ten_seconds_of_grip_raises_the_limit_to_the_ceiling_by_the_closed_form():
    # oracle: closed form -- min(ceiling, base + rise_per_s * t) at t = 10 s.
    grip = _grip()
    for _ in range(100):
        grip.update(1.0, FULL_DEMAND, 0.1)
    assert grip.limit == pytest.approx(min(3.0, 2.0 + 0.2 * 10.0), abs=1e-9)  # m/s^2


def test_two_seconds_of_grip_raises_the_limit_by_the_rate_alone():
    # oracle: closed form -- 2.0 + 0.2 * 2 = 2.4, below the ceiling, so only
    # the rate sets it. Keeps the 10 s test above from passing on the ceiling alone.
    grip = _grip()
    for _ in range(20):
        grip.update(1.0, FULL_DEMAND, 0.1)
    assert grip.limit == pytest.approx(2.4, abs=1e-9)  # m/s^2


def test_one_slip_tick_cuts_the_limit_to_cut_factor_of_itself():
    # oracle: closed form -- 2.8 * 0.85 = 2.38.
    grip = _grip(limit=2.8)
    assert grip.update(0.5, FULL_DEMAND, 0.025) == pytest.approx(2.8 * 0.85, abs=1e-12)


def test_a_cut_never_takes_the_limit_below_the_base():
    # oracle: invariant -- the floor. From 2.1 at 0.85 a tick, the limit goes
    # under 2.0 on the third tick; it must stop at the base instead.
    grip = _grip(limit=2.1)
    for _ in range(50):
        grip.update(0.5, FULL_DEMAND, 0.025)
    assert grip.limit == 2.0  # m/s^2


def test_a_rise_never_takes_the_limit_above_the_ceiling():
    # oracle: invariant -- the ceiling.
    grip = _grip(limit=2.9)
    for _ in range(50):
        grip.update(1.0, FULL_DEMAND, 0.025)
    assert grip.limit == 3.0  # m/s^2


def test_a_ratio_exactly_at_hold_counts_as_grip():
    # The hold boundary is inclusive: ratio >= hold_ratio rises.
    grip = _grip()
    grip.update(0.95, FULL_DEMAND, 0.1)
    assert grip.limit == pytest.approx(2.0 + 0.2 * 0.1, abs=1e-12)  # m/s^2


def test_a_ratio_just_under_hold_holds_the_limit():
    grip = _grip()
    grip.update(0.95 - 1e-9, FULL_DEMAND, 0.1)
    assert grip.limit == 2.0  # m/s^2


def test_a_ratio_exactly_at_slip_is_not_a_slip():
    # The slip boundary is exclusive: ratio < slip_ratio cuts, equal does not.
    grip = _grip(limit=2.8)
    grip.update(0.90, FULL_DEMAND, 0.025)
    assert grip.limit == 2.8  # m/s^2


def test_a_ratio_just_under_slip_is_a_slip():
    grip = _grip(limit=2.8)
    grip.update(0.90 - 1e-9, FULL_DEMAND, 0.025)
    assert grip.limit == pytest.approx(2.8 * 0.85, abs=1e-12)  # m/s^2


def test_a_ratio_between_slip_and_hold_holds_the_limit():
    grip = _grip(limit=2.4)
    grip.update(0.92, FULL_DEMAND, 0.1)
    assert grip.limit == 2.4  # m/s^2


def test_a_demand_exactly_at_the_judge_fraction_is_judged():
    # The judge boundary is inclusive: demand == judge_fraction * limit is judged.
    grip = _grip(limit=2.0)
    grip.update(1.0, 0.8 * 2.0, 0.1)
    assert grip.limit == pytest.approx(2.0 + 0.2 * 0.1, abs=1e-12)  # m/s^2


def test_a_demand_just_under_the_judge_fraction_is_not_judged():
    grip = _grip(limit=2.0)
    grip.update(1.0, 0.8 * 2.0 - 1e-9, 0.1)
    assert grip.limit == 2.0  # m/s^2


def test_low_demand_leaves_the_limit_alone_whatever_the_ratio():
    # 0.8 * 2.4 = 1.92 m/s^2: at 0.5 the car is not asking the tyres for enough.
    grip = _grip(limit=2.4)
    grip.update(1.0, 0.5, 0.1)
    grip.update(0.5, 0.5, 0.1)
    assert grip.limit == 2.4  # m/s^2


def test_a_nan_ratio_leaves_the_limit_alone():
    grip = _grip(limit=2.4)
    grip.update(math.nan, FULL_DEMAND, 0.1)
    assert grip.limit == 2.4  # m/s^2


@pytest.mark.parametrize('demand', [math.nan, math.inf])
def test_a_non_finite_demand_leaves_the_limit_alone(demand):
    grip = _grip(limit=2.4)
    grip.update(1.0, demand, 0.1)
    assert grip.limit == 2.4  # m/s^2


def test_zero_dt_judges_but_moves_nothing_on_a_rise():
    grip = _grip(limit=2.4)
    grip.update(1.0, FULL_DEMAND, 0.0)
    assert grip.limit == 2.4  # m/s^2


@pytest.mark.parametrize('dt', [-0.1, math.nan, math.inf])
def test_a_bad_dt_is_refused(dt):
    with pytest.raises(ValueError, match='dt'):
        _grip().update(1.0, FULL_DEMAND, dt)


def test_the_limit_starts_at_the_base_unless_told_otherwise():
    assert _grip().limit == 2.0  # m/s^2


@pytest.mark.parametrize('overrides, match', [
    (dict(ceiling=1.9), 'ceiling'),
    (dict(slip_ratio=0.95, hold_ratio=0.95), 'slip_ratio'),
    (dict(slip_ratio=0.96), 'slip_ratio'),
    (dict(cut_factor=0.0), 'cut_factor'),
    (dict(cut_factor=1.01), 'cut_factor'),
    (dict(rise_per_s=-0.1), 'rise_per_s'),
    (dict(judge_fraction=-0.1), 'judge_fraction'),
    (dict(limit=3.5), 'limit'),
    (dict(base=0.0, ceiling=3.0), 'base'),
    (dict(hold_ratio=math.nan), 'finite'),
])
def test_a_band_that_cannot_hold_is_refused(overrides, match):
    with pytest.raises(ValueError, match=match):
        _grip(**overrides)


# --- AdaptiveGrip: following max_lateral_accel ------------------------------

def test_following_a_new_base_restarts_the_limit_at_it():
    grip = _grip(limit=2.6)
    grip.follow_base(2.4, 3.0)
    assert (grip.base, grip.limit, grip.ceiling) == (2.4, 2.4, 3.0)


def test_a_base_above_the_configured_ceiling_lifts_the_ceiling_and_a_lower_base_brings_it_back():
    grip = _grip()
    grip.follow_base(3.5, 3.0)
    assert (grip.ceiling, grip.limit) == (3.5, 3.5)
    grip.follow_base(2.4, 3.0)
    assert grip.ceiling == 3.0  # recomputed from the configured ceiling, not kept


def test_following_the_same_base_changes_nothing():
    grip = _grip(limit=2.6)
    grip.follow_base(2.0, 3.0)
    assert grip.limit == 2.6  # m/s^2


def test_following_a_base_refuses_a_non_positive_value():
    with pytest.raises(ValueError, match='base'):
        _grip().follow_base(0.0, 3.0)


# --- adaptive_grip_step: the grip ratio judged on a whole tick --------------

def test_a_car_on_the_circle_the_steering_asks_for_rises_by_the_rate_each_tick():
    # oracle: closed form. On the circle the rack asks for, every tick has
    # ratio 1.0 and a demand of 2.0 m/s^2 (judged below 2.5). Ten ticks of
    # 0.1 s give 2.0 + 0.2 * 1.0.
    steering = math.atan(GRIP_WHEELBASE / 2.0)
    grip = _grip()
    ratio_filter = gap_logic.LowPass(gap_logic.GRIP_RATIO_TAU_SEC)
    limit = None
    for _ in range(10):
        limit = gap_logic.adaptive_grip_step(
            grip, ratio_filter, _kinematic_yaw(2.0, steering), 2.0, steering,
            GRIP_WHEELBASE, 0.1)
    assert limit == pytest.approx(2.0 + 0.2 * 1.0, abs=1e-9)  # m/s^2


def test_a_tick_with_no_judgement_makes_none_and_restarts_the_filter():
    # Below 1 m/s the ratio is NaN, so the limit holds and the filter forgets.
    steering = math.atan(GRIP_WHEELBASE / 2.0)
    grip = _grip(limit=2.2)
    ratio_filter = gap_logic.LowPass(gap_logic.GRIP_RATIO_TAU_SEC)
    ratio_filter.update(0.4, 0.1)
    limit = gap_logic.adaptive_grip_step(
        grip, ratio_filter, 0.4, 0.5, steering, GRIP_WHEELBASE, 0.1)
    assert limit == 2.2  # m/s^2
    assert ratio_filter.value is None


# ============================================================================
# SteeringLag: the yaw is judged against the steering from a lag ago, while the
# steering is steady (docs/gap-follow.md#adaptive-grip-imu)
# ============================================================================

LAG = 0.15      # s: the steering lag the yaw trails by (gap_follow.yaml)
TICK = 0.025    # s: one control tick


def _first_order_yaw(kinematic_yaws, dt, tau):
    """A first-order lag on the kinematic yaw, from rest:
    y_k = y_(k-1) + (1 - exp(-dt / tau)) * (kin_k - y_(k-1)).
    That is the lag's own recurrence, written out here."""
    a = 1.0 - math.exp(-dt / tau)
    y, out = 0.0, []
    for kin in kinematic_yaws:
        y += a * (kin - y)
        out.append(y)
    return out


def test_steering_lag_refuses_bad_inputs():
    with pytest.raises(ValueError, match='lag_sec'):
        gap_logic.SteeringLag(-0.1)
    lag = gap_logic.SteeringLag(LAG)
    with pytest.raises(ValueError, match='dt'):
        lag.step(-0.01, 0.2, 0.05)
    with pytest.raises(ValueError, match='steering'):
        lag.step(TICK, math.nan, 0.05)
    with pytest.raises(ValueError, match='max_change'):
        lag.step(TICK, 0.2, -0.05)


def test_the_lagged_steering_is_not_judged_until_the_history_reaches_back_a_lag():
    # oracle: spec -- until the history covers the lag there is nothing to judge
    # against. The first sample is one tick in, and the window's edge is a lag
    # before each tick, so with 0.025 s ticks the seventh call (index 6) is the
    # first whose edge is at that first sample.
    lag = gap_logic.SteeringLag(LAG)
    judged = [lag.step(TICK, 0.2, 0.05) for _ in range(12)]
    assert all(math.isnan(v) for v in judged[:6])
    assert judged[6:] == pytest.approx([0.2] * 6, abs=1e-12)  # rad


def test_the_lagged_steering_is_interpolated_between_the_samples_either_side():
    # oracle: closed form. A steering linear in time is reproduced exactly by
    # linear interpolation, so the judged value is 2 * (time - lag). With 0.04 s
    # ticks the window's edge falls between two samples, so this exercises the
    # interpolation itself.
    dt = 0.04
    lag = gap_logic.SteeringLag(LAG)
    clock, judged = 0.0, []
    for _ in range(25):
        clock += dt
        judged.append((clock, lag.step(dt, 2.0 * clock, 1.0)))
    finite = [(t, j) for t, j in judged if math.isfinite(j)]
    assert len(finite) >= 15, judged                           # not vacuous
    for t, j in finite:
        assert j == pytest.approx(2.0 * (t - LAG), abs=1e-12)  # rad


def test_a_lag_of_zero_judges_the_steering_now_as_the_code_did_before():
    # oracle: spec -- lag 0 is the behaviour before this change: the steering
    # now, on every tick. The steadiness window is empty, so even a zero
    # threshold passes.
    lag = gap_logic.SteeringLag(0.0)
    judged = [lag.step(TICK, 0.1 * k, 0.0) for k in range(6)]
    assert judged == pytest.approx([0.1 * k for k in range(6)], abs=1e-12)  # rad


def _lag_response(kin_of_t, t_end, tau, sample_dt, h=1e-4):
    """The response of the continuous first-order lag dy/dt = (kin(t) - y) / tau
    from rest, integrated with RK4 on a fine grid and sampled every sample_dt.
    out[j] is the yaw at time (j + 1) * sample_dt."""
    def f(t, y):
        return (kin_of_t(t) - y) / tau
    steps, every = round(t_end / h), round(sample_dt / h)
    y, t, out = 0.0, 0.0, []
    for i in range(steps):
        k1 = f(t, y)
        k2 = f(t + h / 2, y + h / 2 * k1)
        k3 = f(t + h / 2, y + h / 2 * k2)
        k4 = f(t + h, y + h * k3)
        y += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        t += h
        if (i + 1) % every == 0:
            out.append(y)
    return out


def test_no_judgement_while_the_steering_changes_by_more_than_the_threshold():
    # oracle: closed form. The rack steps from 0 to 0.3 rad on the tenth call
    # (index 9, at 0.25 s). For the next six calls the window's edge is still
    # before the step, so the steering a lag ago is 0 and the change is 0.3 rad,
    # over the 0.05 rad threshold: no judgement. On the sixth call after the step
    # the edge is at the step and the window is steady, so judging resumes.
    lag = gap_logic.SteeringLag(LAG)
    judged = [lag.step(TICK, 0.3 if k >= 9 else 0.0, 0.05) for k in range(20)]
    assert all(math.isnan(v) for v in judged[9:15])
    assert judged[15] == pytest.approx(0.3, abs=1e-12)  # rad


def test_a_steering_ramp_reads_one_when_the_lag_is_compensated_and_one_minus_tau_over_t_without():
    # oracle: the continuous first-order lag (tau = 0.15 s) on the kinematic yaw,
    # integrated on a fine grid (_lag_response), at 2.5 m/s. The rack ramps at
    # 0.3 rad/s: over the 0.15 s window that is 0.045 rad, inside the 0.05 rad
    # threshold, so it is quasi-steady. A first-order lag follows a steady ramp
    # one time constant behind. With the lag compensated the ratio is 1; without
    # it, 1 - tau / t, which stays below 0.9 until t = 1.5 s. The early ticks
    # carry the lag's own transient, a few hundredths, so the check starts at 0.5 s.
    speed, dt, tau = 2.5, TICK, LAG
    ticks = 45                                          # 1.125 s
    rack = [0.3 * (k + 1) * dt for k in range(ticks)]   # rad
    yaw = _lag_response(lambda t: speed * math.tan(0.3 * t) / GRIP_WHEELBASE,
                        ticks * dt, tau, dt)
    comp, plain = gap_logic.SteeringLag(LAG), gap_logic.SteeringLag(0.0)
    grip_c, grip_p = _grip(base=2.5), _grip(base=2.5)
    filt_c = gap_logic.LowPass(gap_logic.GRIP_RATIO_TAU_SEC)
    filt_p = gap_logic.LowPass(gap_logic.GRIP_RATIO_TAU_SEC)
    for k in range(ticks):
        t = (k + 1) * dt
        judged_c = comp.step(dt, rack[k], 0.05)
        judged_p = plain.step(dt, rack[k], 0.05)
        gap_logic.adaptive_grip_step(
            grip_c, filt_c, yaw[k], speed, judged_c, GRIP_WHEELBASE, dt)
        gap_logic.adaptive_grip_step(
            grip_p, filt_p, yaw[k], speed, judged_p, GRIP_WHEELBASE, dt)
        if t >= 0.5 and math.isfinite(judged_c) and math.isfinite(judged_p):
            r_c = gap_logic.grip_ratio(yaw[k], speed, judged_c, GRIP_WHEELBASE)
            r_p = gap_logic.grip_ratio(yaw[k], speed, judged_p, GRIP_WHEELBASE)
            assert r_c == pytest.approx(1.0, abs=0.03)              # dimensionless
            assert r_p == pytest.approx(1.0 - tau / t, abs=0.03)    # dimensionless
    assert grip_c.limit > 2.5 + 0.02                        # m/s^2: the compensated limit rose
    assert grip_p.limit == pytest.approx(2.5, abs=1e-12)    # m/s^2: never rose


def test_a_step_in_the_steering_is_only_partly_rescued_by_the_lag():
    # oracle: closed form. The rack steps to 0.2 rad at time 0 and holds, so the
    # yaw is the first-order lag's step response: after k+1 ticks of 0.025 s it is
    # 1 - exp(-(k+1)/6) of the steady value, since tau is six ticks. Judged against
    # the steering now, that is the ratio on every tick. Judged against the
    # steering a lag ago it is the same curve from the seventh tick on: the lag
    # withholds the first lag's judgements and leaves the settling as it was.
    speed, steer, ticks = 2.0, 0.2, 30
    yaw = _first_order_yaw(
        [speed * math.tan(steer) / GRIP_WHEELBASE] * ticks, TICK, LAG)
    plain, comp = gap_logic.SteeringLag(0.0), gap_logic.SteeringLag(LAG)
    r_p, r_c = [], []
    for k in range(ticks):
        r_p.append(gap_logic.grip_ratio(
            yaw[k], speed, plain.step(TICK, steer, 0.05), GRIP_WHEELBASE))
        r_c.append(gap_logic.grip_ratio(
            yaw[k], speed, comp.step(TICK, steer, 0.05), GRIP_WHEELBASE))
    for k in range(ticks):
        expected = 1.0 - math.exp(-(k + 1) / 6.0)                # dimensionless
        assert r_p[k] == pytest.approx(expected, abs=1e-9)       # dimensionless
        if k < 6:
            assert math.isnan(r_c[k]), k
        else:
            assert r_c[k] == pytest.approx(expected, abs=1e-9)   # dimensionless
    # The transient: below 0.9 for the first thirteen ticks, with or without the lag.
    assert all(r < 0.9 for r in r_p[:13]) and r_p[13] >= 0.9
    # Settled: 0.95 is first reached at the eighteenth tick (0.45 s, three time
    # constants), not at the lag.
    assert r_p[16] < 0.95 <= r_p[17]
    assert r_c[16] < 0.95 <= r_c[17]
