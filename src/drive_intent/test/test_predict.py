"""
Unit tests for drive_intent.predict.

Pure geometry with answers known by construction -- no ROS, no hardware,
no simulator. Run with:

    cd ~/racerbot-ws
    python3 -m pytest src/drive_intent/test/test_predict.py -v
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from drive_intent import predict  # noqa: E402


# ============================================================================
# arc_step: the exact constant-curvature update
# ============================================================================

def test_zero_steering_is_a_straight_line_along_the_heading():
    x, y, yaw = predict.arc_step(0.0, 0.0, 0.0, 2.0, 0.0, 0.33, 0.5)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_straight_line_respects_a_non_zero_starting_heading():
    x, y, yaw = predict.arc_step(0.0, 0.0, math.pi / 2, 2.0, 0.0, 0.33, 1.0)
    assert x == pytest.approx(0.0, abs=1e-12)
    assert y == pytest.approx(2.0)
    assert yaw == pytest.approx(math.pi / 2)


def test_quarter_circle_lands_exactly_where_geometry_says():
    """kappa = tan(steer)/L = 0.5 -> radius 2m. A quarter turn from the
    origin heading +X must end at (2, 2) facing +Y, and because the update
    is exact for constant curvature it must do so regardless of how many
    steps we split the arc into."""
    wheelbase = 1.0
    steering = math.atan(0.5)
    speed = 1.0
    arc = math.pi  # (pi/2) * radius 2

    for samples in (2, 3, 17):
        points = predict.constant_arc(
            steering, speed, wheelbase, horizon_s=arc / speed, samples=samples)
        x, y, yaw, _ = points[-1]
        assert x == pytest.approx(2.0, abs=1e-9)
        assert y == pytest.approx(2.0, abs=1e-9)
        assert yaw == pytest.approx(math.pi / 2, abs=1e-9)


def test_right_turn_is_the_mirror_image_of_a_left_turn():
    left = predict.constant_arc(0.2, 1.5, 0.33, 1.0, 9)
    right = predict.constant_arc(-0.2, 1.5, 0.33, 1.0, 9)
    for (lx, ly, lyaw, _), (rx, ry, ryaw, _) in zip(left, right):
        assert rx == pytest.approx(lx)
        assert ry == pytest.approx(-ly)
        assert ryaw == pytest.approx(-lyaw)


def test_curvature_from_steering_matches_the_bicycle_model():
    assert predict.curvature_from_steering(math.atan(0.5), 1.0) == pytest.approx(0.5)
    assert predict.curvature_from_steering(0.0, 0.33) == pytest.approx(0.0)


@pytest.mark.parametrize('wheelbase', [0.0, -0.33, float('nan'), float('inf')])
def test_a_nonsense_wheelbase_is_rejected_not_silently_used(wheelbase):
    with pytest.raises(ValueError):
        predict.curvature_from_steering(0.1, wheelbase)


def test_non_finite_steering_is_rejected():
    with pytest.raises(ValueError):
        predict.curvature_from_steering(float('nan'), 0.33)


# ============================================================================
# integrate: the shape of the published path
# ============================================================================

def test_first_sample_is_the_starting_pose():
    points = predict.constant_arc(0.1, 2.0, 0.33, 1.0, 8, start=(3.0, -1.0, 0.5))
    assert points[0][:3] == pytest.approx((3.0, -1.0, 0.5))


def test_sample_count_and_speed_are_carried_through():
    points = predict.constant_arc(0.0, 1.75, 0.33, 2.0, 12)
    assert len(points) == 12
    assert all(v == pytest.approx(1.75) for _, _, _, v in points)


def test_a_stopped_car_intends_to_stay_exactly_where_it_is():
    """The arrow length is the point of the feature: zero speed must
    produce zero displacement, not a stub pointing somewhere the
    algorithm never asked to go."""
    points = predict.constant_arc(0.25, 0.0, 0.33, 1.5, 10)
    assert predict.path_length(points) == pytest.approx(0.0)
    assert all(x == pytest.approx(0.0) and y == pytest.approx(0.0)
               for x, y, _, _ in points)


def test_length_scales_with_speed_so_a_faster_plan_draws_a_longer_arrow():
    slow = predict.path_length(predict.constant_arc(0.0, 1.0, 0.33, 1.5, 16))
    fast = predict.path_length(predict.constant_arc(0.0, 3.0, 0.33, 1.5, 16))
    assert slow == pytest.approx(1.5)
    assert fast == pytest.approx(4.5)


def test_max_length_truncates_a_very_fast_plan():
    points = predict.constant_arc(
        0.0, 10.0, 0.33, 2.0, 21, max_length_m=4.0)
    assert len(points) < 21
    # Truncation happens at the first sample *past* the limit, so the path
    # covers the limit rather than stopping short of it.
    assert predict.path_length(points) >= 4.0
    assert predict.path_length(points) < 4.0 + 10.0 * (2.0 / 20)


def test_max_length_leaves_a_short_plan_untouched():
    points = predict.constant_arc(0.0, 1.0, 0.33, 1.0, 11, max_length_m=8.0)
    assert len(points) == 11


def test_callbacks_receive_the_evolving_pose():
    """pure_pursuit's arrow bends because it re-asks its steering law at
    every step; this is the hook that makes that possible."""
    seen = []

    def steering_of(i, x, y, yaw):
        seen.append((i, x, y, yaw))
        return 0.0

    predict.integrate(steering_of, lambda *_: 1.0, 0.33, 1.0, 5)
    assert [i for i, *_ in seen] == [0, 1, 2, 3]
    assert seen[0][1] == pytest.approx(0.0)
    assert seen[-1][1] == pytest.approx(0.75)


def test_a_steering_callback_that_corrects_cross_track_error_converges():
    """A crude proportional controller aiming back at y=0 should bring the
    simulated path back toward the axis -- i.e. the integrator really is
    responding to the callback, not quietly holding the first value."""
    def steering_of(i, x, y, yaw):
        return max(-0.4, min(0.4, -2.0 * y - 1.0 * yaw))

    points = predict.integrate(steering_of, lambda *_: 2.0, 0.33, 3.0, 60,
                               start=(0.0, 0.5, 0.0))
    assert abs(points[-1][1]) < 0.5


@pytest.mark.parametrize('samples', [0, 1, -3])
def test_fewer_than_two_samples_is_not_a_path(samples):
    with pytest.raises(ValueError):
        predict.constant_arc(0.0, 1.0, 0.33, 1.0, samples)


@pytest.mark.parametrize('horizon', [0.0, -1.0, float('nan')])
def test_a_nonsense_horizon_is_rejected(horizon):
    with pytest.raises(ValueError):
        predict.constant_arc(0.0, 1.0, 0.33, horizon, 8)


def test_a_non_finite_speed_from_a_callback_is_rejected():
    with pytest.raises(ValueError):
        predict.integrate(lambda *_: 0.0, lambda *_: float('nan'), 0.33, 1.0, 4)


# ============================================================================
# Frame helpers
# ============================================================================

def test_to_body_rotates_and_translates_into_the_cars_own_frame():
    world = [(1.0, 5.0, 0.0, 2.0)]
    body = predict.to_body(world, 1.0, 2.0, math.pi / 2)
    x, y, yaw, v = body[0]
    assert x == pytest.approx(3.0)     # 3m directly ahead
    assert y == pytest.approx(0.0, abs=1e-12)
    assert yaw == pytest.approx(-math.pi / 2)
    assert v == pytest.approx(2.0)


def test_to_body_of_the_origin_pose_is_the_origin():
    body = predict.to_body([(4.0, -2.0, 1.2, 1.0)], 4.0, -2.0, 1.2)
    assert body[0][:3] == pytest.approx((0.0, 0.0, 0.0))


def test_polar_to_body_adds_the_lidar_mounting_offset():
    """0.33m is the real offset on this car; a gap target drawn without it
    sits most of a car length behind where the controller is aiming."""
    x, y = predict.polar_to_body(0.0, 2.0, sensor_offset_x=0.33)
    assert x == pytest.approx(2.33)
    assert y == pytest.approx(0.0)

    x, y = predict.polar_to_body(math.pi / 2, 1.0, sensor_offset_x=0.33)
    assert x == pytest.approx(0.33)
    assert y == pytest.approx(1.0)


def test_path_length_of_a_single_point_is_zero():
    assert predict.path_length([(0.0, 0.0, 0.0, 0.0)]) == pytest.approx(0.0)
    assert predict.path_length([]) == pytest.approx(0.0)
