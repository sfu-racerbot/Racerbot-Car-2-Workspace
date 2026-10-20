"""
Unit tests for the swept-path / arc-guard / corner-setup additions to
gap_logic.py (docs/gap-follow.md). Pure numpy -- runs unsourced:

    python3 -m pytest src/gap_follow/test/test_gap_logic_corners.py -v

Every expected value below is a closed form recomputed in the test (named
where it is used) or an invariant; there are no recorded golden numbers.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from gap_follow import gap_logic  # noqa: E402

# The car's geometry (vesc.yaml / gap_follow.yaml): base_link is the rear axle.
CAR_WIDTH = 0.31
CAR_LENGTH = 0.58
WHEELBASE = 0.36
LASER_X = 0.26
FRONT = WHEELBASE / 2.0 + CAR_LENGTH / 2.0   # 0.47 m: body front ahead of base_link
HW = CAR_WIDTH / 2.0 + 0.05                    # swept half-width used below


def _beams_at(points_base):
    """Laser-frame (range, angle) for points given in base_link."""
    pts = np.asarray(points_base, dtype=float).reshape(-1, 2)
    x = pts[:, 0] - LASER_X
    y = pts[:, 1]
    return np.hypot(x, y), np.arctan2(y, x)


def _swept(ranges, angles, curvature, half_width=HW, max_length=3.0, valid=None):
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    if valid is None:
        valid = np.ones(ranges.shape, dtype=bool)
    return gap_logic.swept_path_clearance(
        ranges, valid, angles, curvature, half_width, CAR_LENGTH, WHEELBASE,
        LASER_X, 0.0, max_length)


def _corridor(left, right, n=721, max_range=10.0):
    """A straight corridor, car parallel to both walls, 180deg window."""
    angles = np.linspace(-math.pi / 2, math.pi / 2, n)
    ranges = np.full(n, max_range)
    for i, a in enumerate(angles):
        s = math.sin(a)
        if s > 1e-6:
            ranges[i] = min(max_range, left / s)
        elif s < -1e-6:
            ranges[i] = min(max_range, right / -s)
    return ranges, angles


# ============================================================================
# swept_path_clearance
# ============================================================================

def test_swept_straight_obstacle_dead_ahead_is_its_distance_from_the_body_front():
    # Oracle: a point at base_link x on the centreline is reached when the
    # body's front edge (FRONT ahead of base_link) gets there: x - FRONT.
    r, a = _beams_at([(2.26, 0.0)])
    assert _swept(r, a, 0.0) == pytest.approx(2.26 - FRONT, abs=1e-9)


def test_swept_straight_ignores_side_walls_the_car_is_driving_past():
    # The bug this replaces: a 1.4 m corridor with nothing ahead. The forward
    # cone's minimum lands on the side walls; the swept path sees nothing.
    ranges, angles = _corridor(0.7, 0.7)
    valid = np.ones_like(ranges, dtype=bool)
    boundaries = gap_logic.vehicle_boundary_distances(
        angles, CAR_WIDTH, CAR_LENGTH, WHEELBASE, LASER_X)
    cone = gap_logic.minimum_footprint_clearance_in_cone(
        ranges, valid, angles, boundaries, math.radians(60.0))
    # Oracle for the cone: the wall at 30deg is 0.7/sin(30) = 1.4 m from the
    # LiDAR, minus the body boundary along that beam.
    assert cone < 1.4
    assert _swept(ranges, angles, 0.0) == math.inf


def test_swept_straight_band_edge_is_the_half_width():
    # Points at lateral offset exactly half_width are in the band; one
    # millimetre beyond is not.
    # (1 micron inside, so the polar round trip cannot push it out.)
    inside_r, inside_a = _beams_at([(1.5, HW - 1e-6)])
    outside_r, outside_a = _beams_at([(1.5, HW + 1e-3)])
    assert _swept(inside_r, inside_a, 0.0) == pytest.approx(1.5 - FRONT, abs=1e-9)
    assert _swept(outside_r, outside_a, 0.0) == math.inf


def test_swept_obstacle_alongside_the_body_is_zero_clearance():
    # Inside the band but behind the front edge: already level with the body.
    r, a = _beams_at([(0.3, 0.1)])
    assert _swept(r, a, 0.0) == 0.0


@pytest.mark.parametrize('sign', [1.0, -1.0])
def test_swept_arc_distance_is_arc_length_minus_body_front(sign):
    # Oracle: on a turn of radius R about (0, sign*R), the point reached after
    # sweeping angle phi is (R sin phi, sign*R(1 - cos phi)); the rear axle
    # has driven R*phi to get level with it, the front got there FRONT sooner.
    radius, phi = 1.2, math.pi / 3
    point = (radius * math.sin(phi), sign * radius * (1.0 - math.cos(phi)))
    r, a = _beams_at([point])
    assert _swept(r, a, sign / radius) == pytest.approx(radius * phi - FRONT, abs=1e-9)


def test_swept_arc_band_includes_off_tracking_on_the_inside():
    # The inner edge of the band is R - half_width (the inner flank beside the
    # rear axle). A point just inside it is in the path, just outside is not.
    radius, phi = 1.0, math.pi / 2
    def at(rho):
        return (rho * math.sin(phi), radius - rho * math.cos(phi))
    r_in, a_in = _beams_at([at(radius - HW + 0.01)])
    r_out, a_out = _beams_at([at(radius - HW - 0.01)])
    assert math.isfinite(_swept(r_in, a_in, 1.0 / radius))
    assert _swept(r_out, a_out, 1.0 / radius) == math.inf


def test_swept_arc_band_outer_edge_is_the_outer_front_corner():
    # Outer edge: hypot(FRONT, R + half_width), where the outer front corner runs.
    radius, phi = 1.0, math.pi / 2
    outer = math.hypot(FRONT, radius + HW)
    def at(rho):
        return (rho * math.sin(phi), radius - rho * math.cos(phi))
    r_in, a_in = _beams_at([at(outer - 0.005)])
    r_out, a_out = _beams_at([at(outer + 0.005)])
    assert math.isfinite(_swept(r_in, a_in, 1.0 / radius))
    assert _swept(r_out, a_out, 1.0 / radius) == math.inf


def test_swept_turning_ignores_a_wall_straight_ahead_it_curves_away_from():
    # A wall dead ahead at 1.0 m blocks the straight path but not a hard
    # left turn whose band never reaches it within the first 1.0 m of arc.
    r, a = _beams_at([(1.0 + LASER_X, 0.0)])
    assert _swept(r, a, 0.0) == pytest.approx(1.0 + LASER_X - FRONT, abs=1e-9)
    assert _swept(r, a, 1.0 / 0.99) > 1.0 + LASER_X - FRONT


def test_swept_beyond_max_length_counts_as_clear():
    # Exactly at the horizon still counts as an obstacle; one step short of
    # it, the path is clear. The boundary is taken from the function's own
    # unbounded answer so float round-off cannot move it.
    r, a = _beams_at([(FRONT + 2.0, 0.0)])
    distance = _swept(r, a, 0.0, max_length=10.0)
    assert distance == pytest.approx(2.0, abs=1e-9)
    assert _swept(r, a, 0.0, max_length=distance) == distance
    assert _swept(r, a, 0.0, max_length=distance - 1e-9) == math.inf


def test_swept_invalid_beams_are_unknown_not_obstacles():
    r, a = _beams_at([(1.0, 0.0)])
    assert _swept(r, a, 0.0, valid=np.array([False])) == math.inf


def test_swept_empty_scan_is_clear():
    assert _swept([], [], 0.0) == math.inf


def test_swept_rejects_bad_inputs():
    r, a = _beams_at([(1.0, 0.0)])
    with pytest.raises(ValueError, match='curvature must be finite'):
        _swept(r, a, float('nan'))
    with pytest.raises(ValueError, match='matching shapes'):
        gap_logic.swept_path_clearance(
            np.ones(2), np.ones(3, bool), np.zeros(2), 0.0, HW, CAR_LENGTH,
            WHEELBASE, LASER_X)
    with pytest.raises(ValueError, match='turn radius must exceed half_width'):
        _swept(r, a, 1.0 / (HW * 0.9))
    with pytest.raises(ValueError, match='must be positive'):
        _swept(r, a, 0.0, half_width=0.0)


# ============================================================================
# arc_guard_steering
# ============================================================================

def _guard(ranges, angles, desired, required=1.0, max_correction=0.1745,
           max_steering=0.349, candidates=29):
    ranges = np.asarray(ranges, dtype=float)
    return gap_logic.arc_guard_steering(
        ranges, np.ones(ranges.shape, dtype=bool), np.asarray(angles, dtype=float),
        desired, required, HW, CAR_LENGTH, WHEELBASE, LASER_X, 0.0,
        max_steering, max_correction, candidates)


def _post_on_arc(steering, arc_length):
    """A single obstacle point sitting on the arc that `steering` drives."""
    radius = WHEELBASE / math.tan(steering)
    phi = (arc_length + FRONT) / abs(radius)
    sign = math.copysign(1.0, radius)
    return _beams_at([(abs(radius) * math.sin(phi),
                       sign * abs(radius) * (1.0 - math.cos(phi)))])


def test_guard_leaves_a_clear_request_alone():
    ranges, angles = _corridor(1.5, 1.5)
    assert _guard(ranges, angles, 0.12) == 0.12


def test_guard_turns_in_later_when_the_requested_arc_is_blocked():
    # An obstacle 0.5 m along the requested left arc. The guard must return a
    # steering whose arc is clear for the required length, and the clear one
    # closest to the request -- here a straighter, later turn-in.
    r, a = _post_on_arc(0.30, 0.5)
    out = _guard(r, a, 0.30, max_correction=0.349)
    assert out < 0.30
    assert _swept(r, a, math.tan(out) / WHEELBASE, max_length=1.5) >= 1.0
    # Invariant: nothing on the grid strictly closer to the request is clear.
    grid = np.linspace(-0.349, 0.349, 29)
    closer = [s for s in grid if abs(s - 0.30) < abs(out - 0.30) - 1e-12]
    assert closer, 'the request itself must be among the blocked candidates'
    for s in closer:
        assert _swept(r, a, math.tan(s) / WHEELBASE, max_length=1.5) < 1.0


def test_guard_falls_back_to_the_freest_arc_inside_its_bound():
    # Same obstacle, default bound 0.1745: every angle within reach of 0.30 is
    # blocked inside 1.0 m, so the guard returns the in-bound angle with the
    # most free length (invariant), not the clear one outside its authority.
    r, a = _post_on_arc(0.30, 0.5)
    out = _guard(r, a, 0.30)
    assert abs(out - 0.30) <= 0.1745 + 1e-12
    grid = [s for s in np.linspace(-0.349, 0.349, 29) if abs(s - 0.30) <= 0.1745]
    free = lambda s: _swept(r, a, math.tan(s) / WHEELBASE, max_length=1.5)
    assert all(free(s) < 1.0 for s in grid)
    assert free(out) == pytest.approx(max(free(s) for s in grid), abs=1e-12)
    assert free(out) > free(0.30)


def test_guard_request_fallback_keeps_the_request_when_nothing_in_reach_is_clear():
    # Same scene as the freest test above, where the 'freest' fallback moves
    # off the request; 'request' must return the request itself (2026-10-20,
    # docs/gap-follow.md -- the freest arc can be the one into the outer wall).
    r, a = _post_on_arc(0.30, 0.5)
    freest = _guard(r, a, 0.30)
    assert freest != 0.30, 'precondition: freest really moves here'
    out = gap_logic.arc_guard_steering(
        np.asarray(r, dtype=float), np.ones(len(r), dtype=bool), np.asarray(a, dtype=float),
        0.30, 1.0, HW, CAR_LENGTH, WHEELBASE, LASER_X, 0.0, 0.349, 0.1745, 29,
        blocked_fallback='request')
    assert out == 0.30


def test_guard_request_fallback_still_takes_a_clear_arc():
    # 'request' only changes the all-blocked case: with a clear arc within the
    # bound it still steers to the nearest clear one, exactly as 'freest' does.
    r, a = _post_on_arc(0.30, 0.5)
    for request in (0.30, 0.25):
        both = [gap_logic.arc_guard_steering(
            np.asarray(r, dtype=float), np.ones(len(r), dtype=bool), np.asarray(a, dtype=float),
            request, 1.0, HW, CAR_LENGTH, WHEELBASE, LASER_X, 0.0, 0.349, 0.30, 29,
            blocked_fallback=mode) for mode in ('freest', 'request')]
        free = _swept(r, a, math.tan(both[1]) / WHEELBASE, max_length=1.5)
        assert both[0] == both[1]
        assert free >= 1.0, 'with a 0.30 rad bound a clear arc is in reach'
    with pytest.raises(ValueError, match='blocked_fallback'):
        gap_logic.arc_guard_steering(
            np.asarray(r, dtype=float), np.ones(len(r), dtype=bool), np.asarray(a, dtype=float),
            0.30, 1.0, HW, CAR_LENGTH, WHEELBASE, LASER_X, 0.0, 0.349, 0.1745, 29,
            blocked_fallback='nearest')


def test_guard_respects_its_authority_bound_even_when_nothing_is_clear():
    # A wall all the way across, 0.3 m ahead of the body: no arc is clear.
    angles = np.linspace(-math.pi / 2, math.pi / 2, 361)
    ranges = np.minimum(10.0, (FRONT - LASER_X + 0.3) / np.maximum(np.cos(angles), 1e-6))
    out = _guard(ranges, angles, 0.0, max_correction=0.05)
    assert abs(out) <= 0.05 + 1e-12


def test_guard_with_zero_authority_returns_the_request():
    r, a = _post_on_arc(0.30, 0.5)
    assert _guard(r, a, 0.30, max_correction=0.0) == 0.30


def test_guard_clips_the_request_to_the_steering_limit():
    ranges, angles = _corridor(3.0, 3.0)
    assert _guard(ranges, angles, 0.9) == 0.349


def test_guard_rejects_bad_inputs():
    ranges, angles = _corridor(1.5, 1.5)
    with pytest.raises(ValueError, match='desired_steering must be finite'):
        _guard(ranges, angles, float('nan'))
    with pytest.raises(ValueError, match='required_length must be positive'):
        _guard(ranges, angles, 0.0, required=0.0)
    with pytest.raises(ValueError, match='candidates must be at least 2'):
        _guard(ranges, angles, 0.0, candidates=1)


# ============================================================================
# side_opening_signal -- the corner-setup turn signal
# ============================================================================

def _raycast(segments, angles, max_range=10.0):
    """Ranges from the LiDAR origin to the nearest of some line segments."""
    ranges = np.full(angles.shape, max_range)
    for (x1, y1), (x2, y2) in segments:
        ex, ey = x2 - x1, y2 - y1
        for i, a in enumerate(angles):
            dx, dy = math.cos(a), math.sin(a)
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-12:
                continue
            t = (x1 * ey - y1 * ex) / denom
            u = (x1 * dy - y1 * dx) / denom
            if t > 0.0 and 0.0 <= u <= 1.0:
                ranges[i] = min(ranges[i], t)
    return ranges


def _turn(corner_x, left, right):
    """Hallway along +x turning 90deg left at corner_x, square inner corner,
    car `left`/`right` from the two walls."""
    outer = corner_x + left + right
    return [((-5, left), (corner_x, left)), ((corner_x, left), (corner_x, 9)),
            ((-5, -right), (outer, -right)), ((outer, -right), (outer, 9))]


ANGLES = np.linspace(-math.pi / 2, math.pi / 2, 721)
# Sector edges deliberately between beams (spacing 0.25deg), so mirroring a
# scene cannot change which beams are inside it.
SECTOR = (math.radians(10.1), math.radians(79.9))


def _opening(segments, left, right, threshold=0.3):
    ranges = _raycast(segments, ANGLES)
    return gap_logic.side_opening_signal(
        ranges, np.ones_like(ranges, bool), ANGLES, left, right, *SECTOR, threshold)


def _expected_opening(corner_x, left, right, threshold=0.3):
    """Independent oracle: per left-sector beam, does the ray-cast range exceed
    the continuing-wall projection left/sin(bearing) by the threshold? (The
    right wall runs to the end wall, so no right-sector beam can.)"""
    ranges = _raycast(_turn(corner_x, left, right), ANGLES)
    in_sector = (ANGLES >= SECTOR[0]) & (ANGLES <= SECTOR[1])
    beyond = [r - left / math.sin(a) > threshold
              for r, a in zip(ranges[in_sector], ANGLES[in_sector])]
    return sum(beyond) / len(beyond)


@pytest.mark.parametrize('left, right', [(0.75, 0.75), (0.4, 1.1), (1.1, 0.4)])
@pytest.mark.parametrize('corner_x', [0.5, 1.0, 2.0, 3.0])
def test_opening_reads_a_left_turn_wherever_the_car_sits(corner_x, left, right):
    signal = _opening(_turn(corner_x, left, right), left, right)
    assert signal == pytest.approx(_expected_opening(corner_x, left, right), abs=1e-12)
    if corner_x <= 2.0 and left >= 0.7:
        assert signal > 0.05
    assert signal >= 0.0


def test_opening_grows_as_the_corner_approaches():
    signals = [_opening(_turn(cx, 0.75, 0.75), 0.75, 0.75) for cx in (3.0, 2.0, 1.0, 0.5)]
    assert all(b > a for a, b in zip(signals, signals[1:]))


def test_opening_is_mirror_symmetric():
    segments = _turn(1.5, 0.75, 0.75)
    mirrored = [((x1, -y1), (x2, -y2)) for (x1, y1), (x2, y2) in segments]
    assert _opening(mirrored, 0.75, 0.75) == pytest.approx(
        -_opening(segments, 0.75, 0.75), abs=1e-12)


@pytest.mark.parametrize('left, right', [(0.75, 0.75), (0.4, 1.1), (1.1, 0.4)])
def test_opening_is_zero_on_a_plain_straight_however_off_centre(left, right):
    # The property the aim and deepest-beam signals lacked: an off-centre car
    # on a straight has asymmetric ranges but no side whose wall ends.
    assert _opening([((-5, left), (20, left)), ((-5, -right), (20, -right))],
                    left, right) == 0.0


def test_opening_threshold_is_strict_and_measured_from_the_wall():
    # One left beam at 45deg: wall projection 1.0/sin(45) = 1.4142 m. A range
    # a millimetre short of threshold beyond it does not count; a millimetre
    # past it does.
    angles = np.array([math.radians(45.0)])
    wall = 1.0 / math.sin(math.radians(45.0))
    at = gap_logic.side_opening_signal(np.array([wall + 0.299]), np.ones(1, bool),
                                       angles, 1.0, 1.0, *SECTOR, 0.3)
    past = gap_logic.side_opening_signal(np.array([wall + 0.301]), np.ones(1, bool),
                                         angles, 1.0, 1.0, *SECTOR, 0.3)
    assert at == 0.0
    assert past == 1.0


def test_opening_ignores_beams_outside_the_sector_and_invalid_beams():
    angles = np.radians([5.0, 45.0, 85.0])
    far = np.array([9.0, 9.0, 9.0])
    assert gap_logic.side_opening_signal(
        far, np.array([True, False, True]), angles, 1.0, 1.0, *SECTOR, 0.3) == 0.0
    assert gap_logic.side_opening_signal(
        far, np.array([True, True, True]), angles, 1.0, 1.0, *SECTOR, 0.3) == 1.0


def test_opening_without_a_wall_on_one_side_is_zero():
    ranges = _raycast(_turn(1.0, 0.75, 0.75), ANGLES)
    valid = np.ones_like(ranges, bool)
    assert gap_logic.side_opening_signal(ranges, valid, ANGLES, math.inf, 0.75, *SECTOR, 0.3) == 0.0
    assert gap_logic.side_opening_signal(ranges, valid, ANGLES, 0.75, math.inf, *SECTOR, 0.3) == 0.0


def test_opening_rejects_bad_inputs():
    ones = np.ones(3)
    with pytest.raises(ValueError, match='matching shapes'):
        gap_logic.side_opening_signal(ones, np.ones(2, bool), ones, 1, 1, *SECTOR, 0.3)
    with pytest.raises(ValueError, match='sector must satisfy'):
        gap_logic.side_opening_signal(ones, np.ones(3, bool), ones, 1, 1, 0.5, 0.2, 0.3)
    with pytest.raises(ValueError, match='excess_threshold'):
        gap_logic.side_opening_signal(ones, np.ones(3, bool), ones, 1, 1, *SECTOR, 0.0)


# ============================================================================
# corner_setup_bias
# ============================================================================

SETUP = dict(car_width=CAR_WIDTH, wall_margin=0.10, setup_fraction=0.6,
             gain=0.3, max_bias=0.10,
             signal_zero=0.03, signal_full=0.12,
             far_zero=4.0, far_full=2.5, near_full=1.6, near_zero=1.1,
             side_full=1.5, side_zero=2.5)


def _setup(left, right, turn, straight, **overrides):
    kw = dict(SETUP, **overrides)
    return gap_logic.corner_setup_bias(left, right, turn, straight, **kw)


def _expected_magnitude(left, right, turn_sign, **overrides):
    """Closed form from the docstring: gain*(target - position), clamped."""
    kw = dict(SETUP, **overrides)
    room = (left + right) / 2.0 - kw['car_width'] / 2.0 - kw['wall_margin']
    target = kw['setup_fraction'] * room
    position = turn_sign * (left - right) / 2.0
    return max(-kw['max_bias'], min(kw['max_bias'], kw['gain'] * (target - position)))


def test_setup_steers_right_before_a_left_turn():
    bias, weight = _setup(0.7, 0.7, turn=0.2, straight=2.0)
    assert weight == pytest.approx(1.0, abs=1e-12)
    assert bias == pytest.approx(-_expected_magnitude(0.7, 0.7, +1), abs=1e-12)
    assert bias < 0.0


def test_setup_is_mirror_symmetric_for_a_right_turn():
    left_turn, _ = _setup(0.6, 0.8, turn=0.2, straight=2.0)
    right_turn, _ = _setup(0.8, 0.6, turn=-0.2, straight=2.0)
    assert right_turn == pytest.approx(-left_turn, abs=1e-12)


def test_setup_is_silent_once_the_car_reaches_its_target():
    # Place the car exactly at the target offset toward the outside (right).
    room = 0.7 - CAR_WIDTH / 2.0 - 0.10
    offset = 0.6 * room
    bias, weight = _setup(0.7 + offset, 0.7 - offset, turn=0.2, straight=2.0)
    assert weight == pytest.approx(1.0, abs=1e-12)
    assert bias == pytest.approx(0.0, abs=1e-12)


def test_setup_pulls_back_in_when_the_car_is_past_its_target():
    # Hard against the outside wall: the bias reverses toward the inside.
    bias, _ = _setup(1.2, 0.2, turn=0.2, straight=2.0)
    assert bias > 0.0
    assert bias == pytest.approx(-_expected_magnitude(1.2, 0.2, +1), abs=1e-12)


def test_setup_never_exceeds_its_authority_bound():
    bias, _ = _setup(0.2, 1.2, turn=0.2, straight=2.0, gain=50.0)
    assert abs(bias) == pytest.approx(SETUP['max_bias'], abs=1e-12)


@pytest.mark.parametrize('turn', [0.0, 0.03, -0.03])
def test_setup_is_off_on_a_straight(turn):
    assert _setup(0.7, 0.7, turn=turn, straight=2.0) == (0.0, 0.0)


@pytest.mark.parametrize('straight', [math.inf, 4.0, 5.0, 1.1, 0.5, 0.0])
def test_setup_is_off_far_from_and_at_the_turn_in_point(straight):
    # inf / >= far_zero: no corner coming. <= near_zero: turning in now.
    assert _setup(0.7, 0.7, turn=0.2, straight=straight) == (0.0, 0.0)


def test_setup_fades_linearly_between_its_ramps():
    # Oracle: halfway between far_full (2.5) and far_zero (4.0) the far ramp
    # is 0.5; halfway between near_zero (1.1) and near_full (1.6), and
    # between signal_zero (0.03) and signal_full (0.12), likewise.
    _, w_far = _setup(0.7, 0.7, turn=0.2, straight=3.25)
    _, w_near = _setup(0.7, 0.7, turn=0.2, straight=1.35)
    _, w_bearing = _setup(0.7, 0.7, turn=0.075, straight=2.0)
    assert w_far == pytest.approx(0.5, abs=1e-9)
    assert w_near == pytest.approx(0.5, abs=1e-9)
    assert w_bearing == pytest.approx(0.5, abs=1e-9)


@pytest.mark.parametrize('left, right', [(math.inf, 0.7), (0.7, math.inf), (2.6, 0.7), (0.7, 3.0)])
def test_setup_needs_a_real_wall_on_both_sides(left, right):
    assert _setup(left, right, turn=0.2, straight=2.0) == (0.0, 0.0)


def test_setup_rejects_bad_inputs():
    with pytest.raises(ValueError, match='straight_clearance must not be NaN'):
        _setup(0.7, 0.7, turn=0.2, straight=float('nan'))
    with pytest.raises(ValueError, match='setup_fraction must be in'):
        _setup(0.7, 0.7, turn=0.2, straight=2.0, setup_fraction=1.5)
    with pytest.raises(ValueError, match='must be non-negative'):
        _setup(0.7, 0.7, turn=0.2, straight=2.0, gain=-1.0)
    with pytest.raises(ValueError, match='corner setup inputs must be finite'):
        _setup(0.7, 0.7, turn=float('nan'), straight=2.0)
