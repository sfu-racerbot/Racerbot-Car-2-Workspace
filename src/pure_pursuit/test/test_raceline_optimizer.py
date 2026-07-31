"""
Unit tests for the minimum-curvature raceline optimizer and the offline
occupancy-grid reader. Plain pytest -- no ROS sourcing, no build, no rclpy,
same as test_racing_math.py:

    python3 -m pytest src/pure_pursuit/test/ -v

The optimizer is checked mostly against closed-form cases whose answer is
known before running anything: a circular track (the best line is the
largest circle that fits), a straight (nothing to gain, so nothing should
move), and a corner (the line must cut to the inside). Those are what catch
a sign error -- an earlier version of the curvature linearisation converged
confidently on the *inner* wall of the circular track, which no amount of
looking at a plot of a real track would have made obvious.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pure_pursuit import racing_math  # noqa: E402
from pure_pursuit import raceline_optimizer as opt  # noqa: E402
from pure_pursuit import occupancy_map as occ  # noqa: E402


def circle_path(radius, count=160, clockwise=False):
    theta = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
    if clockwise:
        theta = -theta
    return np.column_stack([radius * np.cos(theta), radius * np.sin(theta)])


def bending_energy(xy):
    """The objective itself: the arc-length integral of squared curvature."""
    kappa = racing_math.estimate_path_curvature(xy)
    loop = np.vstack([xy, xy[:1]])
    spacing = float(np.mean(np.hypot(*np.diff(loop, axis=0).T)))
    return float(np.sum(kappa ** 2) * spacing)


# ============================================================================
# resample_closed_path / path_frames / signed_curvature
# ============================================================================

def test_resample_gives_uniform_spacing():
    path = opt.resample_closed_path(circle_path(3.0, count=37), 0.25)
    loop = np.vstack([path, path[:1]])
    steps = np.hypot(*np.diff(loop, axis=0).T)
    assert steps.std() < 1e-3
    assert steps.mean() == pytest.approx(0.25, rel=0.02)


def test_resample_preserves_the_shape():
    path = opt.resample_closed_path(circle_path(3.0), 0.1)
    assert np.hypot(path[:, 0], path[:, 1]) == pytest.approx(3.0, abs=0.01)


def test_normals_point_left_of_travel():
    # Counter-clockwise circle: left of travel is toward the centre.
    _, normals = opt.path_frames(circle_path(2.0))
    assert normals[0] == pytest.approx([-1.0, 0.0], abs=1e-3)


def test_signed_curvature_matches_the_radius_and_keeps_the_sign():
    ccw = opt.signed_curvature(circle_path(4.0))
    cw = opt.signed_curvature(circle_path(4.0, clockwise=True))
    assert ccw == pytest.approx(0.25, abs=0.01), 'left turn is positive'
    assert cw == pytest.approx(-0.25, abs=0.01), 'right turn is negative'


def test_signed_curvature_is_zero_on_a_straight():
    straight = np.column_stack([np.arange(20.0), np.zeros(20)])
    assert abs(opt.signed_curvature(straight)[5:15]).max() < 1e-9


def test_curvature_limit_matches_the_bicycle_model():
    assert opt.curvature_limit(0.26, 0.324) == pytest.approx(
        math.tan(0.26) / 0.324)
    assert 1.0 / opt.curvature_limit(0.26, 0.324) == pytest.approx(1.22, abs=0.01)


# ============================================================================
# The closed-form cases
# ============================================================================

def test_circular_track_takes_the_outer_wall():
    """The largest circle that fits inside an annulus is the outer one, and
    the largest circle has the least curvature. Anything else is a sign
    error -- this is the test that caught one."""
    radius, width, count = 5.0, 1.0, 120
    result = opt.optimize_minimum_curvature(
        circle_path(radius, count), np.full(count, width), np.full(count, width),
        vehicle_half_width=1e-6, safety_margin=0.0, spacing=0.3, iterations=6)
    radii = np.hypot(result['line'][:, 0], result['line'][:, 1])
    assert radii.mean() == pytest.approx(radius + width, abs=0.05)


def test_circular_track_answer_does_not_depend_on_lap_direction():
    radius, width, count = 5.0, 1.0, 120
    result = opt.optimize_minimum_curvature(
        circle_path(radius, count, clockwise=True),
        np.full(count, width), np.full(count, width),
        vehicle_half_width=1e-6, safety_margin=0.0, spacing=0.3, iterations=6)
    radii = np.hypot(result['line'][:, 0], result['line'][:, 1])
    assert radii.mean() == pytest.approx(radius + width, abs=0.05)


def test_the_optimizer_reduces_the_objective_it_claims_to():
    radius, width, count = 5.0, 1.0, 120
    centerline = circle_path(radius, count)
    result = opt.optimize_minimum_curvature(
        centerline, np.full(count, width), np.full(count, width),
        vehicle_half_width=1e-6, safety_margin=0.0, spacing=0.3, iterations=6)
    before = bending_energy(opt.resample_closed_path(centerline, 0.3))
    assert bending_energy(result['line']) < before


def test_corners_are_opened_out():
    """On a closed oval the corners are the whole cost, so the line has to
    make them rounder than the centerline's."""
    track = _rounded_rectangle()
    count = len(track)
    result = opt.optimize_minimum_curvature(
        track, np.full(count, 1.0), np.full(count, 1.0),
        vehicle_half_width=1e-6, safety_margin=0.0, spacing=0.2, iterations=6)
    reference = opt.resample_closed_path(track, 0.2)
    before = np.abs(opt.signed_curvature(reference)).max()
    after = np.abs(opt.signed_curvature(result['line'])).max()
    assert after < 0.6 * before, (
        f'tightest corner must open up substantially: {after:.3f} vs {before:.3f}')
    assert bending_energy(result['line']) < 0.5 * bending_energy(reference)


def test_the_whole_corridor_gets_used():
    """A racing line is not a constant offset. It has to move across the
    track -- reaching a wall somewhere and coming off it somewhere else --
    or it is just a shifted centerline."""
    track = _rounded_rectangle()
    count = len(track)
    result = opt.optimize_minimum_curvature(
        track, np.full(count, 1.0), np.full(count, 1.0),
        vehicle_half_width=1e-6, safety_margin=0.0, spacing=0.2, iterations=8)
    alpha = result['alpha']
    assert alpha.min() < -0.9, 'must reach a wall somewhere'
    assert alpha.max() - alpha.min() > 0.5, 'and move meaningfully across the track'


def test_the_answer_beats_every_constant_offset():
    """The cheapest way to fake this optimizer would be to shift the whole
    centerline sideways. Beating that is the minimum bar."""
    track = _rounded_rectangle()
    count = len(track)
    result = opt.optimize_minimum_curvature(
        track, np.full(count, 1.0), np.full(count, 1.0),
        vehicle_half_width=1e-6, safety_margin=0.0, spacing=0.2, iterations=8)
    reference = opt.resample_closed_path(track, 0.2)
    _, normals = opt.path_frames(reference)
    for shift in (-1.0, -0.5, 0.0, 0.5, 1.0):
        assert bending_energy(result['line']) < bending_energy(
            reference + shift * normals), f'lost to a constant {shift:+.1f}m offset'


def test_the_answer_is_a_local_minimum():
    """Probe the result with smooth random moves inside the corridor. If any
    of them lowers the objective, the solver returned something that is not
    even locally optimal."""
    track = _rounded_rectangle()
    count = len(track)
    result = opt.optimize_minimum_curvature(
        track, np.full(count, 1.0), np.full(count, 1.0),
        vehicle_half_width=1e-6, safety_margin=0.0, spacing=0.2, iterations=8)
    reference = opt.resample_closed_path(track, 0.2)
    _, normals = opt.path_frames(reference)
    n = len(reference)
    alpha = np.interp(np.linspace(0.0, 1.0, n, endpoint=False),
                      np.linspace(0.0, 1.0, len(result['alpha']), endpoint=False),
                      result['alpha'])
    best = bending_energy(reference + alpha[:, None] * normals)

    rng = np.random.default_rng(7)
    for _ in range(60):
        perturbation = np.zeros(n)
        for mode in range(1, 9):
            perturbation += rng.normal(0.0, 1.0 / mode) * np.sin(
                2 * math.pi * mode * np.arange(n) / n + rng.uniform(0.0, 2 * math.pi))
        perturbation *= 0.25 / (np.abs(perturbation).max() + 1e-9)
        candidate = np.clip(alpha + perturbation, -1.0, 1.0)
        assert bending_energy(reference + candidate[:, None] * normals) >= best - 1e-6


# ============================================================================
# Constraints -- the part that has to hold for the line to be drivable
# ============================================================================

def test_the_line_stays_inside_the_corridor():
    radius, width, count = 5.0, 1.0, 120
    keep_out = 0.2
    result = opt.optimize_minimum_curvature(
        circle_path(radius, count), np.full(count, width), np.full(count, width),
        vehicle_half_width=0.1, safety_margin=keep_out - 0.1,
        spacing=0.3, iterations=8)
    radii = np.hypot(result['line'][:, 0], result['line'][:, 1])
    assert radii.max() <= radius + width - keep_out + 0.02
    assert radii.min() >= radius - width + keep_out - 0.02


def test_asymmetric_widths_are_respected():
    """Room on one side only: the line may use that side and not the other."""
    radius, count = 5.0, 120
    result = opt.optimize_minimum_curvature(
        circle_path(radius, count),
        width_left=np.full(count, 0.1), width_right=np.full(count, 1.5),
        vehicle_half_width=1e-6, safety_margin=0.0, spacing=0.3, iterations=6)
    radii = np.hypot(result['line'][:, 0], result['line'][:, 1])
    # Left of travel is inward on a CCW circle, so the left limit is a floor
    # on the radius and the right limit is the ceiling.
    assert radii.min() >= radius - 0.1 - 0.02
    assert radii.max() <= radius + 1.5 + 0.02


def test_a_corridor_narrower_than_the_car_is_reported_not_hidden():
    radius, count = 5.0, 80
    result = opt.optimize_minimum_curvature(
        circle_path(radius, count), np.full(count, 0.1), np.full(count, 0.1),
        vehicle_half_width=0.155, safety_margin=0.15, spacing=0.3, iterations=2)
    assert result['clamped_fraction'] == pytest.approx(1.0)


def test_rejects_a_non_positive_vehicle_width():
    with pytest.raises(ValueError):
        opt.optimize_minimum_curvature(
            circle_path(5.0, 40), np.full(40, 1.0), np.full(40, 1.0),
            vehicle_half_width=0.0, safety_margin=0.0, spacing=0.3)


def test_rejects_a_non_positive_trust_region():
    with pytest.raises(ValueError):
        opt.optimize_minimum_curvature(
            circle_path(5.0, 40), np.full(40, 1.0), np.full(40, 1.0),
            vehicle_half_width=0.1, safety_margin=0.0, spacing=0.3,
            trust_region=-1.0)


def test_the_trust_region_bounds_a_single_pass():
    radius, width, count = 5.0, 2.0, 120
    result = opt.optimize_minimum_curvature(
        circle_path(radius, count), np.full(count, width), np.full(count, width),
        vehicle_half_width=1e-6, safety_margin=0.0, spacing=0.3,
        iterations=1, trust_region=0.25)
    radii = np.hypot(result['line'][:, 0], result['line'][:, 1])
    assert radii.max() <= radius + 0.25 + 1e-6


# ============================================================================
# occupancy_map
# ============================================================================

def _ring_map(resolution=0.05, outer=3.0, inner=1.5, pad=0.5):
    """A circular corridor between `inner` and `outer` radius, as a grid."""
    span = outer + pad
    cells = int(2 * span / resolution)
    axis = (np.arange(cells) + 0.5) * resolution - span
    gx, gy = np.meshgrid(axis, axis)
    radius = np.hypot(gx, gy)
    grid = np.where((radius > inner) & (radius < outer), occ.FREE, occ.OCCUPIED)
    return occ.OccupancyMap(grid.astype(np.int8), resolution, -span, -span)


def test_world_to_cell_round_trips():
    grid = _ring_map()
    col, row = grid.world_to_cell(0.0, 0.0)
    assert grid.grid[row, col] == occ.OCCUPIED  # the hub is blocked


def test_free_and_blocked_agree_with_the_geometry():
    grid = _ring_map()
    assert not grid.is_blocked(2.25, 0.0)   # mid-corridor
    assert grid.is_blocked(0.0, 0.0)        # inner hub
    assert grid.is_blocked(9.0, 9.0)        # off the grid entirely


def test_cast_ray_finds_the_wall():
    grid = _ring_map(outer=3.0, inner=1.5)
    # From mid-corridor, straight out: 3.0 - 2.25 = 0.75m to the outer wall.
    assert grid.cast_ray(2.25, 0.0, 0.0, max_distance=5.0) == pytest.approx(0.75, abs=0.06)
    # Straight in: 2.25 - 1.5 = 0.75m to the inner wall.
    assert grid.cast_ray(2.25, 0.0, math.pi, max_distance=5.0) == pytest.approx(0.75, abs=0.06)


def test_cast_ray_reports_max_distance_when_nothing_is_hit():
    grid = _ring_map()
    assert grid.cast_ray(2.25, 0.0, math.pi / 2, max_distance=0.2) == pytest.approx(0.2)


def test_clearance_field_matches_the_corridor():
    grid = _ring_map(outer=3.0, inner=1.5)
    # Dead centre of the corridor is 0.75m from either wall.
    assert float(grid.clearance_at(2.25, 0.0)) == pytest.approx(0.75, abs=0.06)


def test_rejects_a_rotated_map_origin(tmp_path):
    yaml_path = tmp_path / 'rotated.yaml'
    yaml_path.write_text(
        'image: nothing.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.7]\n')
    with pytest.raises(ValueError, match='rotated origin'):
        occ.OccupancyMap.from_yaml(str(yaml_path))


# ============================================================================
# refine_centerline: a lopsided recorded lap becomes a centerline
# ============================================================================

def test_refine_centerline_recovers_the_middle_of_a_ring():
    grid = _ring_map(outer=3.0, inner=1.5)
    # A "recorded lap" hugging the inner wall, as a real one usually does.
    seed = circle_path(1.8, count=90)
    centerline, width_left, width_right = opt.refine_centerline(
        grid, seed, spacing=0.1, max_width=4.0, iterations=5)
    radii = np.hypot(centerline[:, 0], centerline[:, 1])
    assert radii.mean() == pytest.approx(2.25, abs=0.08), 'should land mid-corridor'
    assert width_left.mean() == pytest.approx(0.75, abs=0.1)
    assert width_right.mean() == pytest.approx(0.75, abs=0.1)


def test_refine_centerline_then_optimize_stays_off_the_walls():
    """End to end on a synthetic map: extract, optimize, and check the result
    against the map independently of anything the optimizer believed."""
    grid = _ring_map(outer=3.0, inner=1.5)
    seed = circle_path(1.8, count=90)
    centerline, width_left, width_right = opt.refine_centerline(
        grid, seed, spacing=0.1, max_width=4.0, iterations=5)
    result = opt.optimize_minimum_curvature(
        centerline, width_left, width_right,
        vehicle_half_width=0.155, safety_margin=0.1,
        spacing=0.15, iterations=6)
    clearance = grid.clearance_at(result['line'][:, 0], result['line'][:, 1])
    assert clearance.min() >= 0.155, 'the car body must fit everywhere on the line'


def _rounded_rectangle(half_length=6.0, half_width=3.0, radius=1.5, spacing=0.1):
    """A closed rounded-rectangle centerline, counter-clockwise: four
    straights joined by four quarter-circle corners of `radius`."""
    inner_x = half_length - radius
    inner_y = half_width - radius
    if inner_x <= 0.0 or inner_y <= 0.0:
        raise ValueError('corner radius must be smaller than both half-extents')

    def straight(start, end):
        steps = max(2, int(round(math.dist(start, end) / spacing)))
        return np.column_stack([np.linspace(start[0], end[0], steps, endpoint=False),
                                np.linspace(start[1], end[1], steps, endpoint=False)])

    def arc(cx, cy, start_deg):
        steps = max(2, int(round(radius * math.pi / 2 / spacing)))
        angles = np.radians(np.linspace(start_deg, start_deg + 90.0, steps, endpoint=False))
        return np.column_stack([cx + radius * np.cos(angles),
                                cy + radius * np.sin(angles)])

    return np.vstack([
        straight((-inner_x, -half_width), (inner_x, -half_width)),
        arc(inner_x, -inner_y, -90.0),
        straight((half_length, -inner_y), (half_length, inner_y)),
        arc(inner_x, inner_y, 0.0),
        straight((inner_x, half_width), (-inner_x, half_width)),
        arc(-inner_x, inner_y, 90.0),
        straight((-half_length, inner_y), (-half_length, -inner_y)),
        arc(-inner_x, -inner_y, 180.0),
    ])
