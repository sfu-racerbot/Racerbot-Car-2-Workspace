"""
Unit tests for opponent_tracking.py: moving-car detection, tracking and the
two things gap_follow does with them (predicted-footprint inflation for the
gap search, and the follow speed cap). Pure numpy -- runs unsourced:

    python3 -m pytest src/gap_follow/test/test_opponent_tracking.py -v

Scans are ray-cast from line segments in the test itself, so every expected
value is a closed form recomputed here (named where used) or an invariant.
There are no recorded golden numbers.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from gap_follow import opponent_tracking as ot  # noqa: E402

LASER_X = 0.26
WHEELBASE = 0.36
CAR_LENGTH = 0.58
FRONT = WHEELBASE / 2.0 + CAR_LENGTH / 2.0   # 0.47 m, body front in base_link
N_BEAMS = 1081
ANGLES = np.linspace(-0.75 * math.pi, 0.75 * math.pi, N_BEAMS)  # the Hokuyo's 270 deg
MAX_RANGE = 10.0


@pytest.mark.parametrize('speed,margin', [(1.5, 0.5), (0.2, 0.5), (1.5, 0.0)])
def test_alongside_abort_closed_form_and_ahead_law(speed, margin):
    p = ot.OpponentParams(abort_speed_margin=margin)
    layer = ot.OpponentAwareness(p, WHEELBASE, LASER_X, 0.0, CAR_LENGTH)
    track = ot.Track(FRONT, 0.7, vx=speed, moving=True)
    layer.tracker.tracks = [track]
    # oracle: closed form at the inclusive bumper boundary, ground speed minus margin.
    assert layer.follow_cap(False) == pytest.approx(max(0, speed - margin), abs=1e-12)
    assert layer.follow_cap(True) == math.inf
    track.x = FRONT + 2.0
    # oracle: unchanged gap-only law, even when the car is outside follow_lateral.
    assert layer.follow_cap(False) == pytest.approx(
        (2.0 - p.follow_min_gap) / p.follow_time_gap, abs=1e-12)  # m/s roundoff
    track.y = p.relevant_lateral + 0.001
    # oracle: invariant -- an irrelevant car adds no cap.
    assert layer.follow_cap(False) == math.inf
    layer.tracker.tracks = []
    assert layer.follow_cap(False) == math.inf


CENTRE = WHEELBASE / 2.0   # 0.18 m, body centre in base_link


@pytest.mark.parametrize('ahead', [0.25, 0.01, 0.0, -0.01, -0.25])
def test_alongside_abort_yields_only_the_car_that_is_behind(ahead):
    """Two identical cars side by side, B's centre ``ahead`` metres in front
    of A's, both running the rule. Each sees the other alongside (rear face
    at or behind its bumper) with passing refused."""
    speed = 1.5
    p = ot.OpponentParams()
    caps = []
    for other_x in (CENTRE + ahead, CENTRE - ahead):   # A sees B, B sees A
        layer = ot.OpponentAwareness(p, WHEELBASE, LASER_X, 0.0, CAR_LENGTH)
        layer.tracker.tracks = [ot.Track(other_x, 0.7, vx=speed, moving=True)]
        # oracle: invariant -- the scenario really is alongside for both cars.
        assert layer.moving_body()[0][4] <= FRONT
        caps.append(layer.follow_cap(False))
    yield_cap = max(0.0, speed - p.abort_speed_margin)
    # oracle: invariant -- exactly the car whose centre is behind yields, by the
    # closed-form abort cap; level centres yield neither, so no mutual ratchet.
    expected = ([yield_cap, math.inf] if ahead > 0 else
                [math.inf, yield_cap] if ahead < 0 else [math.inf, math.inf])
    assert caps == pytest.approx(expected, abs=1e-12)  # m/s roundoff


@pytest.mark.parametrize('margin', [-0.01, math.nan, math.inf, -math.inf])
def test_abort_margin_rejects_invalid_values(margin):
    # oracle: spec -- abort margin must be finite and nonnegative.
    with pytest.raises(ValueError, match='opponent_abort_speed_margin must be >= 0 and finite'):
        ot.OpponentParams(abort_speed_margin=margin).validate()


def _cast(segments, angles=ANGLES, origin=(LASER_X, 0.0)):
    """Ranges from ``origin`` (base_link) to the nearest of ``segments``
    [((x0, y0), (x1, y1)), ...] along each beam angle; inf where none."""
    ox, oy = origin
    dx, dy = np.cos(angles), np.sin(angles)
    best = np.full(angles.shape, np.inf)
    for (x0, y0), (x1, y1) in segments:
        ex, ey = x1 - x0, y1 - y0
        denom = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((x0 - ox) * ey - (y0 - oy) * ex) / denom
            u = ((x0 - ox) * dy - (y0 - oy) * dx) / denom
        hit = (np.abs(denom) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(hit & (t < best), t, best)
    return best


def _box(cx, cy, length=0.58, width=0.30, yaw=0.0):
    """The four sides of a car-sized rectangle centred at (cx, cy)."""
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length / 2.0, width / 2.0
    corners = [(cx + c * x - s * y, cy + s * x + c * y)
               for x, y in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


def _scan(segments):
    r = _cast(segments)
    valid = np.isfinite(r) & (r <= MAX_RANGE)
    return np.where(valid, r, MAX_RANGE), valid


P = ot.OpponentParams()


# --- segment_objects ------------------------------------------------------------

def test_car_in_open_space_is_one_detection_at_its_visible_face():
    # Oracle: the box's rear face is at x = cx - 0.29, and the laser sees
    # only that face (and no side, since it is dead ahead), so the cluster
    # centroid's x is the face x and its chord is the 0.30 m face width.
    r, v = _scan(_box(3.0, 0.0))
    dets = ot.segment_objects(r, v, ANGLES, LASER_X, 0.0, P)
    assert len(dets) == 1
    d = dets[0]
    assert d.x == pytest.approx(3.0 - 0.29, abs=0.01)   # 1 cm
    assert d.y == pytest.approx(0.0, abs=0.01)
    assert d.near_x == pytest.approx(3.0 - 0.29, abs=0.01)
    assert d.width == pytest.approx(0.30, abs=0.02)     # beam spacing at 2.4 m ~1 cm


def test_car_in_front_of_a_wall_is_found_and_the_wall_is_not():
    wall = [((5.0, -4.0), (5.0, 4.0))]
    r, v = _scan(wall + _box(2.5, 0.5))
    dets = ot.segment_objects(r, v, ANGLES, LASER_X, 0.0, P)
    assert len(dets) == 1
    # Oracle: the centroid of points on the box's visible faces lies within
    # the box (its rear face and the side facing the laser).
    assert 2.5 - 0.29 - 1e-6 <= dets[0].x <= 2.5 + 0.29
    assert 0.5 - 0.15 - 1e-6 <= dets[0].y <= 0.5 + 0.15


def test_stub_continuous_with_a_wall_is_not_free_standing():
    # A car-width piece of wall joined to a longer wall at the same range:
    # the far side of the "cluster" is not open, so it is not a car.
    r, v = _scan([((3.0, -0.15), (3.0, 3.0))])
    assert ot.segment_objects(r, v, ANGLES, LASER_X, 0.0, P) == []


def test_too_wide_and_too_narrow_clusters_are_rejected():
    wide = [((3.0, -0.5), (3.0, 0.5))]          # 1.0 m > max_width 0.7
    narrow = [((3.0, -0.04), (3.0, 0.04))]      # 0.08 m < min_width 0.15
    for seg in (wide, narrow):
        r, v = _scan(seg)
        assert ot.segment_objects(r, v, ANGLES, LASER_X, 0.0, P) == []


def test_width_limits_are_inclusive_at_the_boundary():
    # A cluster whose chord is exactly max_width/min_width is kept; the same
    # scene with the limit moved 1 cm inside is not.
    r, v = _scan([((3.0, -0.2), (3.0, 0.2))])
    dets = ot.segment_objects(r, v, ANGLES, LASER_X, 0.0, P)
    chord = dets[0].width
    keep = ot.OpponentParams(max_width=chord)
    drop = ot.OpponentParams(max_width=chord - 0.01)
    assert len(ot.segment_objects(r, v, ANGLES, LASER_X, 0.0, keep)) == 1
    assert ot.segment_objects(r, v, ANGLES, LASER_X, 0.0, drop) == []
    keep = ot.OpponentParams(min_width=chord)
    drop = ot.OpponentParams(min_width=chord + 0.01)
    assert len(ot.segment_objects(r, v, ANGLES, LASER_X, 0.0, keep)) == 1
    assert ot.segment_objects(r, v, ANGLES, LASER_X, 0.0, drop) == []


def test_min_beams_boundary():
    # Three valid beams at 2 m in an otherwise empty scan, wide enough apart
    # to make a 0.2 m chord: kept at min_beams=3, dropped at 4.
    r = np.full(N_BEAMS, MAX_RANGE)
    v = np.zeros(N_BEAMS, dtype=bool)
    mid = N_BEAMS // 2
    r[mid - 1:mid + 2] = 2.0
    v[mid - 1:mid + 2] = True
    angles = ANGLES.copy()
    angles[mid - 1:mid + 2] = [-0.05, 0.0, 0.05]   # chord 2*2*sin(0.05) = 0.2 m
    p3 = ot.OpponentParams(min_beams=3)
    p4 = ot.OpponentParams(min_beams=4)
    assert len(ot.segment_objects(r, v, angles, LASER_X, 0.0, p3)) == 1
    assert ot.segment_objects(r, v, angles, LASER_X, 0.0, p4) == []


def test_max_range_boundary():
    r, v = _scan(_box(4.0, 0.0))
    near = float(np.min(r[v]))
    assert len(ot.segment_objects(
        r, v, ANGLES, LASER_X, 0.0, ot.OpponentParams(max_range=near))) == 1
    assert ot.segment_objects(
        r, v, ANGLES, LASER_X, 0.0, ot.OpponentParams(max_range=near - 0.01)) == []


def test_cluster_touching_either_end_of_the_scan_is_dropped():
    r = np.full(N_BEAMS, MAX_RANGE)
    v = np.zeros(N_BEAMS, dtype=bool)
    angles = np.linspace(-0.1, 0.1, N_BEAMS)
    r[:5] = 2.0
    v[:5] = True
    assert ot.segment_objects(r, v, angles, LASER_X, 0.0, P) == []
    r2, v2 = r[::-1].copy(), v[::-1].copy()
    assert ot.segment_objects(r2, v2, angles, LASER_X, 0.0, P) == []


def test_degenerate_scans_give_no_detections():
    assert ot.segment_objects([], [], [], LASER_X, 0.0, P) == []
    r, v = _scan(_box(3.0, 0.0))
    assert ot.segment_objects(r, v, ANGLES[:-1], LASER_X, 0.0, P) == []   # wrong count
    assert ot.segment_objects(np.full(N_BEAMS, np.nan), np.ones(N_BEAMS, bool),
                              ANGLES, LASER_X, 0.0, P) == []
    assert ot.segment_objects(np.full(N_BEAMS, np.inf), np.ones(N_BEAMS, bool),
                              ANGLES, LASER_X, 0.0, P) == []


def test_invalid_neighbour_counts_as_open():
    # NaN beams beside the object are unknown; they do not join it to anything.
    r, v = _scan(_box(3.0, 0.0))
    idx = np.flatnonzero(v)
    lo, hi = idx[0], idx[-1]
    r2 = r.copy()
    v2 = v.copy()
    r2[lo - 1] = np.nan
    r2[hi + 1] = np.nan
    v2[lo - 1] = v2[hi + 1] = False
    assert len(ot.segment_objects(r2, v2, ANGLES, LASER_X, 0.0, P)) == 1


# --- EgoPose --------------------------------------------------------------------

def test_ego_straight_line():
    e = ot.EgoPose()
    for _ in range(20):
        e.advance(2.0, 0.0, WHEELBASE, 0.025)
    assert e.x == pytest.approx(2.0 * 0.5, abs=1e-9)   # v * t
    assert e.y == pytest.approx(0.0, abs=1e-12)
    assert e.yaw == pytest.approx(0.0, abs=1e-12)


def test_ego_full_circle_returns_home():
    # Kinematic bicycle: radius L / tan(delta); a full lap takes 2*pi*R / v.
    v, delta = 1.5, 0.3
    radius = WHEELBASE / math.tan(delta)
    steps = 4000
    dt = 2.0 * math.pi * radius / v / steps
    e = ot.EgoPose()
    max_y = 0.0
    for _ in range(steps):
        e.advance(v, delta, WHEELBASE, dt)
        max_y = max(max_y, e.y)
    assert e.yaw == pytest.approx(2.0 * math.pi, abs=1e-9)
    assert math.hypot(e.x, e.y) == pytest.approx(0.0, abs=1e-6)   # 1 um
    assert max_y == pytest.approx(2.0 * radius, abs=1e-3)          # 1 mm: the far side


def test_ego_frame_round_trip_and_rotation():
    e = ot.EgoPose(1.0, -2.0, 0.7)
    bx, by = e.to_body(*e.to_world(0.4, -1.3))
    assert (bx, by) == pytest.approx((0.4, -1.3), abs=1e-12)
    # A point straight ahead of a car facing +90 deg lies along world +y.
    e = ot.EgoPose(0.0, 0.0, math.pi / 2.0)
    assert e.to_world(2.0, 0.0) == pytest.approx((0.0, 2.0), abs=1e-12)
    assert e.vector_to_body(0.0, 1.0) == pytest.approx((1.0, 0.0), abs=1e-12)


# --- OpponentTracker ------------------------------------------------------------

def _feed(tracker, positions, dt=0.025):
    for x, y in positions:
        tracker.update([(x, y, 0.0)], dt)


def test_constant_velocity_target_converges_to_its_velocity():
    # Oracle: an alpha-beta filter has zero steady-state error on a
    # constant-velocity target.
    tr = ot.OpponentTracker(ot.OpponentParams())
    dt = 0.025
    _feed(tr, [(1.5 * k * dt, 0.3) for k in range(200)], dt)
    assert len(tr.tracks) == 1
    t = tr.tracks[0]
    assert t.vx == pytest.approx(1.5, abs=0.01)   # 1 cm/s
    assert t.vy == pytest.approx(0.0, abs=0.01)
    assert t.moving


def test_moving_needs_min_hits_of_history():
    params = ot.OpponentParams(min_hits=8)
    tr = ot.OpponentTracker(params)
    dt = 0.025
    # Fast enough that speed passes moving_speed long before 8 hits... if
    # the filter is primed: start it with a large jump on hit 2.
    positions = [(3.0 * k * dt, 0.0) for k in range(7)]
    _feed(tr, positions, dt)
    assert tr.tracks[0].hits == 7
    assert not tr.tracks[0].moving
    _feed(tr, [(3.0 * 7 * dt, 0.0)], dt)
    assert tr.tracks[0].hits == 8
    assert tr.tracks[0].speed >= params.moving_speed
    assert tr.tracks[0].moving


def test_static_object_never_counts_as_moving():
    tr = ot.OpponentTracker(ot.OpponentParams())
    rng = np.random.default_rng(3)
    # 2 cm of measurement noise, the order of a Hokuyo cluster centroid.
    _feed(tr, [(2.0 + rng.normal(0, 0.02), 1.0 + rng.normal(0, 0.02)) for _ in range(400)])
    assert len(tr.tracks) == 1
    assert not tr.tracks[0].moving


def test_moving_holds_until_speed_drops_below_half_threshold():
    params = ot.OpponentParams(moving_speed=0.6)
    tr = ot.OpponentTracker(params)
    tr.tracks = [ot.Track(0.0, 0.0, vx=0.31, vy=0.0, hits=50, moving=True)]
    tr.update([(0.31 * 0.025, 0.0, 0.0)], 0.025)    # exactly on its prediction
    assert tr.tracks[0].speed == pytest.approx(0.31, abs=1e-9)
    assert tr.tracks[0].moving                        # 0.31 >= 0.3
    tr.tracks = [ot.Track(0.0, 0.0, vx=0.29, vy=0.0, hits=50, moving=True)]
    tr.update([(0.29 * 0.025, 0.0, 0.0)], 0.025)
    assert not tr.tracks[0].moving                    # 0.29 < 0.3
    # And a track not yet moving needs the full threshold.
    tr.tracks = [ot.Track(0.0, 0.0, vx=0.59, vy=0.0, hits=50, moving=False)]
    tr.update([(0.59 * 0.025, 0.0, 0.0)], 0.025)
    assert not tr.tracks[0].moving


def test_track_survives_max_misses_and_is_dropped_one_after():
    params = ot.OpponentParams(max_misses=3)
    tr = ot.OpponentTracker(params)
    _feed(tr, [(1.0, 0.0)])
    for _ in range(3):
        tr.update([], 0.025)
    assert len(tr.tracks) == 1 and tr.tracks[0].misses == 3
    tr.update([], 0.025)
    assert tr.tracks == []


def test_detection_outside_the_gate_starts_a_new_track():
    params = ot.OpponentParams(gate=0.5)
    tr = ot.OpponentTracker(params)
    _feed(tr, [(1.0, 0.0)])
    tr.update([(1.0 + 0.49, 0.0, 0.0)], 0.025)   # inside: same track
    assert len(tr.tracks) == 1 and tr.tracks[0].hits == 2
    tr2 = ot.OpponentTracker(params)
    _feed(tr2, [(1.0, 0.0)])
    tr2.update([(1.0 + 0.51, 0.0, 0.0)], 0.025)  # outside: a second track
    assert len(tr2.tracks) == 2


def test_two_detections_go_to_the_nearer_tracks():
    tr = ot.OpponentTracker(ot.OpponentParams())
    tr.update([(0.0, 0.0, 0.0), (0.0, 1.0, 0.0)], 0.025)
    ids = {round(t.y): t.track_id for t in tr.tracks}
    tr.update([(0.0, 1.02, 0.0), (0.0, 0.02, 0.0)], 0.025)   # listed in swapped order
    assert {round(t.y): t.track_id for t in tr.tracks} == ids
    assert all(t.hits == 2 for t in tr.tracks)


def test_velocity_clamp():
    params = ot.OpponentParams(max_track_speed=8.0, gate=10.0)
    tr = ot.OpponentTracker(params)
    _feed(tr, [(0.0, 0.0), (5.0, 0.0)])          # 200 m/s implied
    assert tr.tracks[0].speed == pytest.approx(8.0, abs=1e-9)


# --- inflate_predicted ----------------------------------------------------------

def test_stationary_rectangle_entry_range_is_closed_form():
    # oracle: R4 visible-face centre recovery replaces the old disc oracle.
    # At rest heading defaults to line of sight; entry is near face - margin.
    params = ot.OpponentParams(radius=0.3, passing_margin=0.15, prediction_horizon=0.0)
    window = np.full(ANGLES.size, MAX_RANGE)
    out = ot.inflate_predicted(window, ANGLES, [(LASER_X + 2.0, 0.0, 0.0, 0.0, 2.0)],
                               LASER_X, 0.0, params)
    mid = int(np.argmin(np.abs(ANGLES)))
    assert out[mid] == pytest.approx(2.0 - params.passing_margin, abs=1e-9)
    # oracle: closed form tangent at the inflated rear corner.
    graze = math.atan2(params.width / 2 + params.passing_margin,
                       2.0 - params.passing_margin)
    assert np.all(out[np.abs(ANGLES) > graze + 0.01] == MAX_RANGE)
    assert np.all(out[np.abs(ANGLES) < graze - 0.01] < MAX_RANGE)
    assert window[mid] == MAX_RANGE                        # input not modified


def test_inflation_never_lengthens_a_beam():
    params = ot.OpponentParams(prediction_horizon=0.0)
    window = np.full(ANGLES.size, 1.0)
    out = ot.inflate_predicted(window, ANGLES, [(LASER_X + 3.0, 0.0, 0.0, 0.0, 3.0)],
                               LASER_X, 0.0, params)
    assert np.array_equal(out, window)


def test_prediction_blocks_where_the_car_is_going_not_only_where_it_is():
    # A car 2 m ahead driving left at 2 m/s will be at y = +1.6 m after 0.8 s.
    window = np.full(ANGLES.size, MAX_RANGE)
    opp = [(LASER_X + 2.0, 0.0, 0.0, 2.0, 2.0)]
    ahead = ot.inflate_predicted(window, ANGLES, opp, LASER_X, 0.0,
                                 ot.OpponentParams(prediction_horizon=0.8))
    now_only = ot.inflate_predicted(window, ANGLES, opp, LASER_X, 0.0,
                                    ot.OpponentParams(prediction_horizon=0.0))
    toward_future = int(np.argmin(np.abs(ANGLES - math.atan2(1.6, 2.0))))
    assert ahead[toward_future] < MAX_RANGE
    assert now_only[toward_future] == MAX_RANGE
    # And nothing on the side it is driving away from.
    away = int(np.argmin(np.abs(ANGLES - math.atan2(-1.0, 2.0))))
    assert ahead[away] == MAX_RANGE


def test_laser_inside_a_rectangle_gets_the_floor():
    params = ot.OpponentParams(radius=0.3, passing_margin=0.15, prediction_horizon=0.0)
    window = np.full(ANGLES.size, MAX_RANGE)
    # oracle: near face at 0.1 m, inside the 0.15 m expanded rear face.
    out = ot.inflate_predicted(window, ANGLES, [(LASER_X + 0.1, 0.0, 0.0, 0.0, 0.1)],
                               LASER_X, 0.0, params, floor=0.05)
    mid = int(np.argmin(np.abs(ANGLES)))
    assert out[mid] == pytest.approx(0.05, abs=1e-12)


def test_no_opponents_returns_an_equal_copy():
    window = np.linspace(1.0, 5.0, ANGLES.size)
    out = ot.inflate_predicted(window, ANGLES, [], LASER_X, 0.0, P)
    assert np.array_equal(out, window) and out is not window


# --- follow_speed_cap -----------------------------------------------------------

def test_follow_cap_closed_form():
    params = ot.OpponentParams(follow_min_gap=0.5, follow_time_gap=0.6, follow_lateral=0.45)
    near_x = FRONT + 1.7
    assert ot.follow_speed_cap([(near_x, 0.1)], FRONT, params) == pytest.approx(
        (1.7 - 0.5) / 0.6, abs=1e-12)
    # The nearest qualifying car binds.
    assert ot.follow_speed_cap([(FRONT + 3.0, 0.0), (near_x, -0.2)], FRONT, params) == \
        pytest.approx((1.7 - 0.5) / 0.6, abs=1e-12)


def test_follow_cap_is_zero_inside_min_gap_never_negative():
    params = ot.OpponentParams(follow_min_gap=0.5)
    assert ot.follow_speed_cap([(FRONT + 0.5, 0.0)], FRONT, params) == 0.0
    assert ot.follow_speed_cap([(FRONT + 0.2, 0.0)], FRONT, params) == 0.0


def test_follow_cap_lateral_boundary():
    params = ot.OpponentParams(follow_lateral=0.45)
    assert math.isfinite(ot.follow_speed_cap([(FRONT + 2.0, 0.45)], FRONT, params))
    assert ot.follow_speed_cap([(FRONT + 2.0, 0.46)], FRONT, params) == math.inf
    assert ot.follow_speed_cap([(FRONT + 2.0, -0.46)], FRONT, params) == math.inf


def test_follow_cap_ignores_a_car_alongside():
    # Nearest point at or behind the bumper: already being passed.
    assert ot.follow_speed_cap([(FRONT, 0.0)], FRONT, P) == math.inf
    assert ot.follow_speed_cap([(FRONT + 1e-6, 0.0)], FRONT, P) < math.inf
    assert ot.follow_speed_cap([], FRONT, P) == math.inf


# --- OpponentParams -------------------------------------------------------------

@pytest.mark.parametrize('name,value,match', [
    ('min_width', 0.0, 'opponent_min_width must be positive'),
    ('gate', math.nan, 'opponent_gate must be positive'),
    ('follow_time_gap', -1.0, 'opponent_follow_time_gap must be positive'),
    ('max_range', math.inf, 'opponent_max_range must be positive'),
    ('passing_margin', -0.01, 'opponent_passing_margin must be >= 0'),
    ('prediction_horizon', math.inf, 'opponent_prediction_horizon must be >= 0'),
    ('alpha', 0.0, r'opponent_alpha must be in \(0, 1\]'),
    ('beta', 1.5, r'opponent_beta must be in \(0, 1\]'),
    ('min_hits', 0, 'opponent_min_hits must be a positive integer'),
    ('max_misses', 2.5, 'opponent_max_misses must be a positive integer'),
])
def test_validate_rejects(name, value, match):
    params = ot.OpponentParams(**{name: value})
    with pytest.raises(ValueError, match=match):
        params.validate()


def test_validate_rejects_inverted_width_limits():
    with pytest.raises(ValueError, match='opponent_min_width .* must be below'):
        ot.OpponentParams(min_width=0.7, max_width=0.7).validate()


def test_defaults_validate_and_from_mapping_reads_ros_names():
    ot.OpponentParams().validate()
    p = ot.OpponentParams.from_mapping({'opponent_gate': 0.9, 'max_speed': 3.0})
    assert p.gate == 0.9
    assert p.radius == ot.OpponentParams().radius


# --- OpponentAwareness end to end ----------------------------------------------

def _run_scene(ego_speed, opp_speed, steps=40, dt=0.025, post=None):
    """Ego drives +x at ego_speed; another car starts 3 m ahead driving +x at
    opp_speed (1 s of scans by default, so a 1.5 m/s closing speed leaves
    1.5 m between them). ``post``: a fixed box at that world position. Returns the
    layer after ``steps`` scans."""
    layer = ot.OpponentAwareness(ot.OpponentParams(), WHEELBASE, LASER_X, 0.0, CAR_LENGTH)
    for k in range(steps):
        t = k * dt
        ego_x = ego_speed * t
        segs = _box(3.0 + opp_speed * t - ego_x, 0.0)
        if post is not None:
            segs += _box(post[0] - ego_x, post[1], length=0.3, width=0.3)
        r, v = _scan(segs)
        layer.observe(r, v, ANGLES, ego_speed, 0.0, dt)
    return layer


def test_layer_reports_a_moving_car_with_its_ground_speed():
    layer = _run_scene(ego_speed=3.0, opp_speed=1.5)
    movers = layer.moving_body()
    assert len(movers) == 1
    x, y, vx, vy, near_x, _near_range = movers[0]
    # Ground speed, not closing speed: 1.5, not 1.5 - 3.0.
    assert vx == pytest.approx(1.5, abs=0.05)
    assert vy == pytest.approx(0.0, abs=0.05)
    # Its rear face: start 3 m, closed 1.5 m/s for 79 scans' worth of time.
    t_last = 39 * 0.025
    assert near_x == pytest.approx(3.0 - 0.29 + (1.5 - 3.0) * t_last, abs=0.05)
    # And the follow cap reads that gap.
    gap = near_x - FRONT
    assert layer.follow_cap() == pytest.approx(
        max(0.0, (gap - P.follow_min_gap) / P.follow_time_gap), abs=1e-9)


def test_layer_ignores_a_parked_obstacle_while_driving_past_it():
    layer = _run_scene(ego_speed=2.0, opp_speed=0.0, steps=60, post=(6.0, 1.2))
    assert layer.moving_body() == []
    assert layer.follow_cap() == math.inf


def test_layer_bad_dt_resets_history():
    layer = _run_scene(ego_speed=3.0, opp_speed=1.5)
    assert layer.moving_body()
    r, v = _scan(_box(1.0, 0.0))
    layer.observe(r, v, ANGLES, 3.0, 0.0, 0.0)          # dt not positive
    assert layer.moving_body() == []
    assert all(t.hits == 1 for t in layer.tracker.tracks)
    layer = _run_scene(ego_speed=3.0, opp_speed=1.5)
    layer.observe(r, v, ANGLES, 3.0, 0.0, 0.6)          # older than 0.5 s
    assert layer.moving_body() == []
    layer = _run_scene(ego_speed=3.0, opp_speed=1.5)
    layer.observe(r, v, ANGLES, math.nan, 0.0, 0.025)   # speed unknown
    assert layer.moving_body() == []


def test_flickering_detection_never_becomes_moving():
    # A wall fragment that is seen every other scan, each time displaced as
    # if moving at 2 m/s: the measured false-mover pattern. Consecutive
    # detections never reach min_hits, so it is never "moving" -- while the
    # same motion seen on every scan is.
    params = ot.OpponentParams(min_hits=8, gate=1.0, max_misses=6)
    dt = 0.025
    tr = ot.OpponentTracker(params)
    for k in range(200):
        tr.update([(2.0 * k * dt, 0.0, 0.0)] if k % 2 == 0 else [], dt)
        assert not any(t.moving for t in tr.tracks)
    steady = ot.OpponentTracker(params)
    _feed(steady, [(2.0 * k * dt, 0.0) for k in range(40)], dt)
    assert steady.tracks[0].moving


def test_a_single_miss_resets_the_streak():
    tr = ot.OpponentTracker(ot.OpponentParams(min_hits=8))
    _feed(tr, [(3.0 * k * 0.025, 0.0) for k in range(7)])
    tr.update([], 0.025)
    _feed(tr, [(3.0 * k * 0.025, 0.0) for k in range(8, 15)])
    assert tr.tracks[0].streak == 7
    assert not tr.tracks[0].moving
    _feed(tr, [(3.0 * 15 * 0.025, 0.0)])
    assert tr.tracks[0].moving


# --- avoidance_usable -----------------------------------------------------------

def test_avoidance_may_pick_a_side_of_the_same_opening():
    assert ot.avoidance_usable((10, 40, False), (5, 60, False))
    assert ot.avoidance_usable((55, 80, False), (5, 60, False))    # overlaps by 6
    assert ot.avoidance_usable((60, 80, False), (5, 60, False))    # touches at 60


def test_avoidance_may_not_switch_to_another_opening():
    assert not ot.avoidance_usable((61, 80, False), (5, 60, False))
    assert not ot.avoidance_usable((0, 4, False), (5, 60, False))


def test_avoidance_may_not_settle_for_a_shallower_gap():
    assert not ot.avoidance_usable((10, 40, True), (5, 60, False))
    assert ot.avoidance_usable((10, 40, True), (5, 60, True))
    assert ot.avoidance_usable((10, 40, False), (5, 60, True))


def test_avoidance_with_no_gap_is_unusable_but_is_all_there_is_when_plain_has_none():
    assert not ot.avoidance_usable((None, None, False), (5, 60, False))
    assert ot.avoidance_usable((10, 40, False), (None, None, False))


# --- relevance (OpponentAwareness.moving_body) ---------------------------------

def _layer_with_track(x, y, vx, vy, **params):
    layer = ot.OpponentAwareness(ot.OpponentParams(**params), WHEELBASE, LASER_X,
                                 0.0, CAR_LENGTH)
    layer.tracker.tracks = [ot.Track(x, y, vx=vx, vy=vy, hits=50, streak=50, moving=True)]
    return layer


def test_relevance_lateral_boundary():
    assert _layer_with_track(2.0, 1.0, 1.5, 0.0, relevant_lateral=1.0).moving_body()
    assert _layer_with_track(2.0, -1.0, 1.5, 0.0, relevant_lateral=1.0).moving_body()
    assert _layer_with_track(2.0, 1.01, 1.5, 0.0, relevant_lateral=1.0).moving_body() == []


def test_a_car_moving_the_other_way_is_not_relevant():
    # The far leg of a hairpin, seen through the opening.
    assert _layer_with_track(2.0, 0.5, -1.5, 0.0).moving_body() == []
    assert _layer_with_track(2.0, 0.5, 0.0, 1.5).moving_body()      # crossing: kept


def test_a_car_behind_is_not_relevant():
    assert _layer_with_track(-0.31, 0.0, 1.5, 0.0, radius=0.3).moving_body() == []
    assert _layer_with_track(-0.29, 0.0, 1.5, 0.0, radius=0.3).moving_body()


def test_a_car_beyond_max_range_is_not_relevant():
    assert _layer_with_track(5.9, 0.0, 1.5, 0.0, max_range=6.0).moving_body()
    assert _layer_with_track(6.1, 0.0, 1.5, 0.0, max_range=6.0).moving_body() == []


def test_a_stationary_track_is_not_relevant_even_in_the_path():
    layer = _layer_with_track(2.0, 0.0, 0.0, 0.0)
    layer.tracker.tracks[0].moving = False
    assert layer.moving_body() == []
    assert layer.follow_cap() == math.inf


def test_partly_hidden_object_is_not_free_standing():
    # A far wall at 3 m, partly hidden behind a nearer wall at 2 m on its
    # left. The visible piece of the far wall is car-width and open on its
    # right, but its left neighbour is NEARER: it may be a corner of
    # anything, so it is not a car.
    near = [((2.0, 0.10), (2.0, 2.0))]
    far = [((3.0, -0.20), (3.0, 0.5))]
    r, v = _scan(near + far)
    assert ot.segment_objects(r, v, ANGLES, LASER_X, 0.0, P) == []
    # Without the near wall, the same far piece (0.7 m) is a detection.
    r, v = _scan(far)
    assert len(ot.segment_objects(r, v, ANGLES, LASER_X, 0.0, P)) == 1


def test_ego_step_moves_along_the_mean_heading():
    # Oracle: the chord of a circular arc points along the heading halfway
    # through it, so one step that turns by d_theta moves at d_theta / 2.
    v, delta, dt = 2.0, 0.3, 0.2
    d_theta = v * math.tan(delta) / WHEELBASE * dt
    e = ot.EgoPose()
    e.advance(v, delta, WHEELBASE, dt)
    assert math.atan2(e.y, e.x) == pytest.approx(d_theta / 2.0, abs=1e-12)
    assert math.hypot(e.x, e.y) == pytest.approx(v * dt, abs=1e-12)
    assert e.yaw == pytest.approx(d_theta, abs=1e-12)


def test_one_update_moves_position_by_alpha_and_velocity_by_beta_over_dt():
    params = ot.OpponentParams(alpha=0.5, beta=0.15)
    tr = ot.OpponentTracker(params)
    tr.tracks = [ot.Track(0.0, 0.0)]
    tr.update([(0.2, -0.1, 0.0)], 0.025)
    t = tr.tracks[0]
    assert (t.x, t.y) == pytest.approx((0.5 * 0.2, 0.5 * -0.1), abs=1e-12)
    assert (t.vx, t.vy) == pytest.approx((0.15 / 0.025 * 0.2, 0.15 / 0.025 * -0.1), abs=1e-12)


def test_association_takes_the_closest_pair_first():
    # Track A at y=0 has both detections in its gate, the nearer at -0.3;
    # track B at y=0.4 has only the one at 0.35, 0.05 away. Closest-first
    # gives B that one and A the other; track-order-first would give A the
    # 0.35 one and leave B unmatched.
    tr = ot.OpponentTracker(ot.OpponentParams(gate=0.5))
    tr.tracks = [ot.Track(0.0, 0.0, track_id=1), ot.Track(0.0, 0.4, track_id=2)]
    tr.update([(0.0, 0.35, 0.0), (0.0, -0.3, 0.0)], 0.025)
    assert len(tr.tracks) == 2
    by_id = {t.track_id: t for t in tr.tracks}
    assert by_id[2].y == pytest.approx(0.4 + 0.5 * (0.35 - 0.4), abs=1e-12)
    assert by_id[1].y == pytest.approx(0.5 * -0.3, abs=1e-12)


def test_prediction_follows_motion_along_x_too():
    # oracle: closed form. Plan R4 step 1 replaces the disc, so its fixed
    # 10-degree bearing is replaced by a rectangle tangent construction.
    # For h=(1,0), support=(L/2*2+W/2*1)/hypot(2,1) and centre=u*(r+s).
    # Choose halfway between the lower tangent to the unshifted rectangle
    # and the lower tangent to the recovered rectangle at t=0. The correct
    # rectangle misses now and sweeps over this beam at the horizon;
    # omitting centre recovery instead hits now, which the first assert kills.
    window = np.full(ANGLES.size, MAX_RANGE)
    near = math.hypot(2.0, 1.0)
    params = ot.OpponentParams()
    support = (params.length / 2 * 2.0 + params.width / 2) / near
    cx, cy = (2.0 * (near + support) / near, (near + support) / near)
    half_l = params.length / 2 + params.passing_margin
    half_w = params.width / 2 + params.passing_margin
    tangent_unshifted = math.atan2(1.0 - half_w, 2.0 + half_l)
    tangent_recovered = math.atan2(cy - half_w, cx + half_l)
    bearing = (tangent_unshifted + tangent_recovered) / 2
    opp = [(LASER_X + 2.0, 1.0, 2.0, 0.0, near)]
    beam = int(np.argmin(np.abs(ANGLES - bearing)))
    now_only = ot.inflate_predicted(window, ANGLES, opp, LASER_X, 0.0,
                                    ot.OpponentParams(prediction_horizon=0.0))
    ahead = ot.inflate_predicted(window, ANGLES, opp, LASER_X, 0.0,
                                 ot.OpponentParams(prediction_horizon=0.8))
    assert now_only[beam] == MAX_RANGE
    assert ahead[beam] < MAX_RANGE


def test_validate_rejects_relevant_lateral():
    with pytest.raises(ValueError, match='opponent_relevant_lateral must be positive'):
        ot.OpponentParams(relevant_lateral=0.0).validate()


# --- pass_allowed / follow cap when passing is not allowed ----------------------

def test_no_pass_while_steering_into_a_bend():
    params = ot.OpponentParams(pass_max_steering=0.10)
    ahead = [(2.0, 0.5, 1.8, 0.0, 1.8)]
    assert ot.pass_allowed(ahead, 0.10, params)
    assert ot.pass_allowed(ahead, -0.10, params)
    assert not ot.pass_allowed(ahead, 0.101, params)
    assert not ot.pass_allowed(ahead, -0.101, params)


def test_no_pass_while_the_car_ahead_is_turning():
    params = ot.OpponentParams(pass_max_heading_deg=15.0)
    v = 1.8
    at = lambda deg: [(2.0, 0.5, v * math.cos(math.radians(deg)),
                       v * math.sin(math.radians(deg)), 1.8)]
    assert ot.pass_allowed(at(14.9), 0.0, params)
    assert ot.pass_allowed(at(-14.9), 0.0, params)
    assert not ot.pass_allowed(at(15.1), 0.0, params)
    assert not ot.pass_allowed(at(-15.1), 0.0, params)
    assert ot.pass_allowed([], 0.0, params)


def test_follow_cap_widens_to_relevant_lateral_when_passing_is_not_allowed():
    # A car 0.8 m to the side, 2 m ahead: outside follow_lateral, inside
    # relevant_lateral. Ignored while passing is allowed; followed when not.
    layer = _layer_with_track(FRONT + 2.0, 0.8, 1.5, 0.0,
                              follow_lateral=0.45, relevant_lateral=1.0)
    assert layer.follow_cap(passing=True) == math.inf
    expected = (2.0 - layer.params.follow_min_gap) / layer.params.follow_time_gap
    assert layer.follow_cap(passing=False) == pytest.approx(expected, abs=1e-9)


def test_validate_rejects_pass_limits():
    with pytest.raises(ValueError, match='opponent_pass_max_steering must be positive'):
        ot.OpponentParams(pass_max_steering=0.0).validate()
    with pytest.raises(ValueError, match='opponent_pass_max_heading_deg must be positive'):
        ot.OpponentParams(pass_max_heading_deg=-1.0).validate()


# --- SteeringEstimate (the servo model the dead reckoning turns with) ------------

def test_rack_does_not_move_until_the_delay_has_passed():
    est = ot.SteeringEstimate(delay=0.075, gain=15.0, rate_max=3.2)
    dt = 0.025
    for _ in range(2):                      # 0.025 s and 0.05 s: still waiting
        assert est.update(0.3, dt) == 0.0
    # At 0.075 s the first command is old enough. 15/s * 0.3 rad = 4.5 rad/s
    # asks for more than the 3.2 rad/s slew limit, so it moves 3.2 * dt.
    assert est.update(0.3, dt) == pytest.approx(3.2 * dt, abs=1e-12)


def test_small_error_moves_at_gain_times_error():
    est = ot.SteeringEstimate(delay=0.0, gain=15.0, rate_max=3.2)
    est.reset(0.10)
    # 15/s * 0.01 rad = 0.15 rad/s, under the slew limit: proportional step.
    assert est.update(0.11, 0.01) == pytest.approx(0.10 + 15.0 * 0.01 * 0.01, abs=1e-12)


def test_rack_settles_on_the_command_and_follows_it_both_ways():
    est = ot.SteeringEstimate(delay=0.075, gain=15.0, rate_max=3.2)
    for _ in range(200):
        est.update(-0.25, 0.025)
    assert est.angle == pytest.approx(-0.25, abs=1e-6)
    for _ in range(200):
        est.update(0.2, 0.025)
    assert est.angle == pytest.approx(0.2, abs=1e-6)


def test_layer_turns_with_the_lagged_rack_not_the_command():
    # One scan interval after the rack is commanded to full lock, the car has
    # not turned yet: the command is still inside the 75 ms delay.
    layer = ot.OpponentAwareness(ot.OpponentParams(), WHEELBASE, LASER_X, 0.0, CAR_LENGTH)
    r, v = _scan([])
    layer.observe(r, v, ANGLES, 2.0, 0.0, 0.025)
    layer.observe(r, v, ANGLES, 2.0, 0.35, 0.025)
    assert layer.ego.yaw == 0.0
    assert layer.ego.x == pytest.approx(2.0 * 0.05, abs=1e-12)


def test_validate_rejects_servo_model_values():
    with pytest.raises(ValueError, match='opponent_steer_gain must be positive'):
        ot.OpponentParams(steer_gain=0.0).validate()
    with pytest.raises(ValueError, match='opponent_steer_delay must be >= 0'):
        ot.OpponentParams(steer_delay=-0.01).validate()


# R2 step-4 mutation checks: dropping the hold kills the stopped-car test;
# ignoring its timeout kills the release test; holding unconfirmed tracks kills
# both wall cases; forcing zero kills the increasing-gap test; removing the
# zero clamp kills the -1 mm boundary. Constant caps kill the closed-form tests.
def _confirmed_follow_layer(**kwargs):
    layer = ot.OpponentAwareness(ot.OpponentParams(**kwargs), WHEELBASE,
                                 LASER_X, 0.0, CAR_LENGTH)
    # Real detections confirm a 1 m/s car, not an injected confirmation flag.
    _feed(layer.tracker, [(1.0 + k * 0.025, 0.0) for k in range(40)])
    assert layer.tracker.tracks[0].moving  # oracle: constant-velocity target
    return layer


def test_follow_leader_two_mps_at_its_time_gap():
    layer = _layer_with_track(FRONT + 2.5, 0.0, 2.0, 0.0,
                              follow_time_gap=1.0, follow_min_gap=0.5)
    # oracle: closed form, gap-only law; leader speed cancels at its time gap.
    assert layer.follow_cap() == pytest.approx((2.5 - 0.5) / 1.0, abs=1e-12)


def test_confirmed_car_stays_followed_when_it_stops():
    layer = _confirmed_follow_layer()
    t = layer.tracker.tracks[0]
    for _ in range(24):
        layer.tracker.update([(t.x, t.y, 0.0)], 0.025)
    # oracle: R2 hold invariant, despite the unchanged moving latch releasing.
    assert not t.moving
    assert layer.followed_body()
    assert math.isfinite(layer.follow_cap())
    # Noise at rest must not turn a confirmed car into an oncoming car.
    t.vx = -0.01
    assert layer.followed_body()
    assert layer.moving_body() == []
    # oracle: follow-only hold cannot create an inflated obstacle or a passer.
    window = np.full(ANGLES.size, MAX_RANGE)
    assert np.array_equal(layer.augment(window, ANGLES), window)
    # oracle: review round 1, held tracks always use the relevant lateral band.
    t.x, t.y = FRONT + 1.0, layer.params.follow_lateral + 0.001
    assert layer.follow_cap(passing=True) == pytest.approx(
        (1.0 - layer.params.follow_min_gap) / layer.params.follow_time_gap, abs=1e-12)
    assert math.isfinite(layer.follow_cap(passing=False))
    for x, y in ((-layer.params.radius, 0.0),
                 (2.0, layer.params.relevant_lateral + 0.001),
                 (layer.params.max_range + 0.001, 0.0)):
        t.x, t.y = x, y
        assert layer.followed_body() == []
        assert layer.follow_cap(passing=False) == math.inf


@pytest.mark.parametrize('offset', [0.0, 0.001, -0.001])
def test_follow_minimum_gap_exact_boundary(offset):
    p = ot.OpponentParams(follow_min_gap=0.5, follow_time_gap=1.0)
    # oracle: closed form; below the minimum is clamped, never negative.
    result = ot.follow_speed_cap([(FRONT + p.follow_min_gap + offset, 0.0)], FRONT, p)
    assert result == pytest.approx(max(0.0, offset / p.follow_time_gap), abs=1e-12)


def test_held_car_cap_rises_with_gap():
    layer = _confirmed_follow_layer()
    t = layer.tracker.tracks[0]
    for _ in range(24):
        layer.tracker.update([(t.x, t.y, 0.0)], 0.025)
    t.vx = t.vy = 0.0
    caps = []
    for gap in (0.6, 0.8):
        t.x = FRONT + gap
        caps.append(layer.follow_cap())
        # oracle: closed form independent of the stopped leader's velocity.
        assert caps[-1] == pytest.approx(
            (gap - layer.params.follow_min_gap) / layer.params.follow_time_gap, abs=1e-12)
    assert caps[1] > caps[0]  # oracle: increasing gap must release a zero cap


@pytest.mark.parametrize('dt,release', [(0.125, 2.0), (0.025, 1.0)])
def test_held_car_release_exact_boundary(dt, release):
    # Exercise both binary-exact timing and the real 40 Hz scan interval;
    # roundoff in accumulated dt must not release on the equality tick.
    layer = _confirmed_follow_layer(stopped_release_sec=release)
    t = layer.tracker.tracks[0]
    t.vx = t.vy = 0.0
    for _ in range(round(release / dt)):
        layer.tracker.update([(t.x, t.y, 0.0)], dt)
    # oracle: R2 inclusive hold through exactly release seconds.
    assert layer.followed_body()
    layer.tracker.update([(t.x, t.y, 0.0)], dt)
    assert layer.followed_body() == []
    assert layer.follow_cap() == math.inf


@pytest.mark.parametrize('flicker', [False, True])
def test_never_confirmed_wall_is_never_followed(flicker):
    layer = ot.OpponentAwareness(ot.OpponentParams(), WHEELBASE, LASER_X, 0.0, CAR_LENGTH)
    for k in range(80):
        # Flickering fragment moves in the filter, but never has min_hits scans.
        x = 1.0 + k * 0.025 if flicker else 2.0
        detections = [] if flicker and k % 4 == 3 else [(x, 0.0, 0.0)]
        layer.tracker.update(detections, 0.025)
        # oracle: Review Focus 2, only once-moving-confirmed tracks may be held.
        assert layer.followed_body() == []
        assert layer.follow_cap() == math.inf


@pytest.mark.parametrize('value', [0.0, -1.0, math.nan, math.inf])
def test_stopped_release_must_be_positive_finite(value):
    # oracle: parameter validation contract, seconds must be positive and finite.
    with pytest.raises(ValueError, match='opponent_stopped_release_sec must be positive'):
        ot.OpponentParams(stopped_release_sec=value).validate()


def test_held_leader_cap_is_independent_of_passing_a_moving_car():
    # Review mutation: using the pass-dependent held band fails the cap oracle.
    layer = _confirmed_follow_layer()
    held = layer.tracker.tracks[0]
    for _ in range(24):
        layer.tracker.update([(held.x, held.y, 0.0)], 0.025)
    held.x, held.y = FRONT + 1.5, 0.75
    # Alongside mover permits a pass but cannot bind a forward follow cap.
    layer.tracker.tracks.append(ot.Track(
        0.0, -0.75, vx=1.0, moving=True, hits=50, streak=50))
    assert layer.may_pass(0.0)  # oracle: straight-moving alongside car
    assert not held.moving
    # oracle: closed form; 0.45 < |y|=0.75 < 1.0 must not discard a held leader.
    expected = (1.5 - layer.params.follow_min_gap) / layer.params.follow_time_gap
    assert layer.follow_cap(passing=True) == pytest.approx(expected, abs=1e-12)


# R4 tests run with the package pytest runner, through rb (see task protocol).
# R4 step 3, 2026-10-28: runtime mutants were observed failing, then removed
# by process exit (no broken source was written): no rotation -> visible-face
# entry at 90/45 deg; no centre shift -> prediction_follows_motion_along_x_too;
# strict slab inequality -> parallel_beam_margin_boundary[0.0]; no width gate
# -> narrow_corridor and the published-command test; half margin -> entry.
# Also killed: no prediction (future-hit assertion), low-speed heading fallback
# removed (undefined_heading), no width prediction (narrowing_beyond_current_body),
# 10% wrong gap extent (synthetic_corridor), and a mirror-only width mismatch
# (crossing_and_overtaking_mirror_parity[False]). Stub swaps covered every new
# geometry/validation/plumbing test; restored focused suite: 146 passed.
@pytest.mark.parametrize('heading', [0.0, math.pi / 2, math.pi / 4])
@pytest.mark.parametrize('laser', [(0.26, 0.0), (0.13, -0.2)])
def test_rectangle_entry_from_visible_face(heading, laser):
    p = ot.OpponentParams(length=0.58, width=0.31, passing_margin=0.05,
                          prediction_horizon=0.0)
    near = 2.0
    h = (math.cos(heading), math.sin(heading))
    support = p.length / 2 * abs(h[0]) + p.width / 2 * abs(h[1])
    centre_x = near + support
    # oracle: closed form slab intersection on the x axis. At 45 degrees
    # the narrower transverse slab determines entry, not the support plane.
    half_l = p.length / 2 + p.passing_margin
    half_w = p.width / 2 + p.passing_margin
    reach = min(half_l / abs(h[0]) if abs(h[0]) > 1e-12 else math.inf,
                half_w / abs(h[1]) if abs(h[1]) > 1e-12 else math.inf)
    expected = centre_x - reach
    out = ot.inflate_predicted([10.0], [0.0],
                               [(laser[0] + near, laser[1], *h, near)],
                               *laser, p)
    assert out[0] == pytest.approx(expected, abs=1e-12)  # metre tolerance
    if heading in (0.0, math.pi / 2):
        assert out[0] == pytest.approx(near - p.passing_margin, abs=1e-12)


@pytest.mark.parametrize('offset', [-0.001, 0.0, 0.001])
def test_rectangle_parallel_beam_margin_boundary(offset):
    p = ot.OpponentParams(length=0.58, width=0.31, passing_margin=0.05,
                          prediction_horizon=0.0)
    y = p.width / 2 + p.passing_margin + offset
    near = 2.0
    # oracle: closed form: at the near corner tan(b) * front_x = half-width.
    # Higher bearings miss; lower bearings enter the rectangle's rear face.
    front_x = near - p.passing_margin
    angle = math.atan2(y, front_x)
    out = ot.inflate_predicted([10.0], [angle], [(LASER_X + near, 0.0, 1.0, 0.0, near)],
                               LASER_X, 0.0, p)
    # At the near corner, a tangent ray has entry == exit and must be retained.
    if offset <= 0:
        entry = (near - p.passing_margin) / math.cos(angle)
        assert out[0] == pytest.approx(entry, abs=1e-12)
    else:
        assert out[0] == 10.0


@pytest.mark.parametrize('near', [0.0, 0.05, 0.10])
def test_rectangle_floor_and_nan(near):
    p = ot.OpponentParams(length=0.58, width=0.31, passing_margin=0.05,
                          prediction_horizon=0.0)
    out = ot.inflate_predicted([10.0, math.nan, 0.01], [0.0, 0.0, 0.0],
                               [(LASER_X + near, 0.0, 1.0, 0.0, near)],
                               LASER_X, 0.0, p, floor=0.05)
    # oracle: floor clamp, NaN preservation and never-lengthen invariants.
    assert out[0] == pytest.approx(max(0.05, near - p.passing_margin), abs=1e-12)
    assert math.isnan(out[1])
    assert out[2] == 0.01


@pytest.mark.parametrize('name', ['length', 'width'])
@pytest.mark.parametrize('value', [0.0, -0.1, math.nan, math.inf, -math.inf])
def test_rectangle_dimensions_positive_finite(name, value):
    # oracle: R4 interface contract, physical extents must be positive finite.
    with pytest.raises(ValueError, match=f'opponent_{name} must be positive and finite'):
        ot.OpponentParams(**{name: value}).validate()


def test_visible_nearest_range_is_carried_without_changing_follow_offset():
    layer = _run_scene(ego_speed=1.0, opp_speed=1.0)
    d = layer.detections[0]
    ranges, valid = _scan(_box(3.0, 0.0))
    # oracle: nearest beam range is measured from the LiDAR, not base_link.
    assert d.near_range == pytest.approx(float(np.min(ranges[valid])), abs=1e-12)
    mover = layer.moving_body()[0]
    assert mover[5] == pytest.approx(d.near_range, abs=1e-12)
    assert mover[4] == pytest.approx(d.near_x, abs=0.001)  # 1 mm filter residual


def test_rectangle_horizon_blocks_an_additional_beam():
    near, horizon, velocity = 2.0, 0.8, 2.0
    p = ot.OpponentParams(length=0.58, width=0.31, passing_margin=0.05,
                          prediction_horizon=horizon, prediction_step=horizon)
    # oracle: side-on centre starts at x=near+W/2, and translates v*T in y.
    cx = near + p.width / 2
    angle = math.atan2(velocity * horizon, cx)
    opp = [(LASER_X + near, 0.0, 0.0, velocity, near)]
    future = ot.inflate_predicted([10.0], [angle], opp, LASER_X, 0.0, p)
    p.prediction_horizon = 0.0
    now = ot.inflate_predicted([10.0], [angle], opp, LASER_X, 0.0, p)
    # At the future rectangle, the x slab gives the entry distance.
    assert future[0] == pytest.approx((near - p.passing_margin) / math.cos(angle),
                                       abs=1e-12)
    assert now[0] == 10.0


@pytest.mark.parametrize('width', [1.06, 1.52])
@pytest.mark.parametrize('margin', [0.05, 0.10, 0.15])
def test_narrow_corridor_refuses_pass_at_required_physical_width(width, margin):
    p = ot.OpponentParams(length=0.58, width=0.31, passing_margin=margin)
    # oracle: closed form, centred rectangle leaves half the corridor minus
    # half the opponent and one margin. Ego needs its width plus TWO margins.
    available = (width - p.width) / 2 - margin
    required = 0.31 + 2 * margin
    expected = available >= required
    assert ot.avoidance_usable((10, 40, False), (5, 60, False),
                               gap_width=available, required_width=required) == expected


def test_passing_width_exact_boundary_and_next_tick_abort():
    required = 0.31 + 2 * 0.05
    # oracle: inclusive physical-width gate, reevaluated without a pass latch.
    for available, expected in ((required, True), (required - 0.001, False),
                                ((1.52 - 0.31) / 2 - 0.05, True),
                                ((1.06 - 0.31) / 2 - 0.05, False)):
        assert ot.avoidance_usable((10, 40, False), (5, 60, False),
                                   gap_width=available, required_width=required) == expected


def test_rectangle_synthetic_corridor_aborts_on_first_narrow_tick():
    layer = ot.OpponentAwareness(ot.OpponentParams(passing_margin=0.05),
                                 WHEELBASE, LASER_X, 0.0, CAR_LENGTH)
    layer.tracker.tracks = [ot.Track(LASER_X + 2.0, 0.0, vx=1.0, moving=True,
                                     near_range=2.0)]
    side = (int(np.searchsorted(ANGLES, 0.15)), int(np.searchsorted(ANGLES, 0.25)), False)
    plain = (int(np.searchsorted(ANGLES, -0.4)), int(np.searchsorted(ANGLES, 0.4)), False)
    modes = []
    for width in (1.52, 1.06):
        walls = [((-1.0, sign * width / 2), (8.0, sign * width / 2))
                 for sign in (-1, 1)]
        window, _valid = _scan(walls + _box(LASER_X + 2.0 + CAR_LENGTH / 2, 0.0))
        room = layer.passing_gap_width(window, ANGLES, side)
        # oracle: physical gap between a straight wall and inflated car edge.
        expected_room = width / 2 - layer.params.width / 2 - layer.params.passing_margin
        assert room == pytest.approx(expected_room, abs=1e-12)
        passing = layer.may_pass(0.0) and ot.avoidance_usable(
            side, plain, gap_width=room,
            required_width=0.31 + 2 * layer.params.passing_margin)
        modes.append('pass' if passing else 'follow')
        if not passing:
            # oracle: unchanged R2 follow cap, nearest face 2 m from laser.
            assert layer.follow_cap(False) == pytest.approx(
                (LASER_X + 2.0 - FRONT - layer.params.follow_min_gap) /
                layer.params.follow_time_gap, abs=1e-12)
    assert modes == ['pass', 'follow']  # oracle: no tick of latched passing


def test_rectangle_prediction_checks_narrowing_beyond_current_body():
    layer = ot.OpponentAwareness(ot.OpponentParams(passing_margin=0.05),
                                 WHEELBASE, LASER_X, 0.0, CAR_LENGTH)
    layer.tracker.tracks = [ot.Track(LASER_X + 2.0, 0.0, vx=1.0, moving=True,
                                     near_range=2.0)]
    side = (int(np.searchsorted(ANGLES, 0.15)), int(np.searchsorted(ANGLES, 0.25)), False)
    # The neck starts beyond the present inflated rectangle's front, inside
    # its prediction horizon. Both walls are visible beside the other car.
    neck_x = LASER_X + 2.0 + CAR_LENGTH + 0.1
    walls = [((-1.0, s * .76), (neck_x, s * .76)) for s in (-1, 1)]
    walls += [((neck_x, s * .76), (neck_x, s * .53)) for s in (-1, 1)]
    walls += [((neck_x, s * .53), (8.0, s * .53)) for s in (-1, 1)]
    window, _ = _scan(walls + _box(LASER_X + 2.0 + CAR_LENGTH / 2, 0.0))
    # oracle: once prediction reaches the neck, room is .53-.155-.05=.325,
    # too narrow for ego's .31+2*.05=.41 m, on this first observed tick.
    assert layer.passing_gap_width(window, ANGLES, side) == pytest.approx(
        1.06 / 2 - layer.params.width / 2 - layer.params.passing_margin, abs=1e-12)


@pytest.mark.parametrize('heading', [0.0, math.pi / 2])
def test_rectangle_crossing_side_beam_closed_form(heading):
    p = ot.OpponentParams(passing_margin=0.05, prediction_horizon=0.0)
    near = 2.0
    transverse = p.width / 2 if heading == 0.0 else p.length / 2
    y = transverse + p.passing_margin - .001
    angle = math.atan2(y, near - p.passing_margin)
    out = ot.inflate_predicted([10.0], [angle],
                               [(LASER_X + near, 0.0, math.cos(heading),
                                 math.sin(heading), near)], LASER_X, 0, p)
    # oracle: rear view spans W, crossing view spans L; both meet x=d-margin.
    assert out[0] == pytest.approx((near - p.passing_margin) / math.cos(angle), abs=1e-12)


def test_rectangle_undefined_heading_uses_line_of_sight():
    p = ot.OpponentParams(passing_margin=0.05, prediction_horizon=0.0)
    # Below half moving_speed, even a perpendicular velocity must not set h.
    angle = math.atan2((p.length + p.width) / 4 + p.passing_margin,
                       2.0 - p.passing_margin)
    out = ot.inflate_predicted([10.0, 10.0], [0.0, angle],
                               [(LASER_X + 2.0, 0.0, 0.0, .29, 2.0)], LASER_X, 0, p)
    # oracle: line-of-sight rear face d, regardless of the low-speed vector.
    assert out[0] == pytest.approx(2.0 - p.passing_margin, abs=1e-12)
    assert out[1] == 10.0  # oracle: beyond rear-view width, inside crossing length


# --- FollowCapMemory: the follow cap held for a moment after its car is lost ---
#
# Spec: the FollowCapMemory docstring in opponent_tracking.py, and the user's
# decision of 2026-10-28 (option 1): awareness on, with opponent_follow_memory_sec
# 1.0 s. Each oracle is restated in its test: an inclusive hold of memory_sec
# measured from the last stored finite cap; the clock rule of
# OpponentAwareness.observe, 0 < dt <= 0.5 s; and speed-only, so a held cap is
# never above the stored one. FollowCapMemory is reached through ot.* inside each
# test body, so the module still imports before the class exists.

@pytest.mark.parametrize('memory_sec, dt', [(1.0, 0.025), (1.0, 0.125), (0.5, 0.025)])
def test_follow_memory_holds_inclusive_then_releases_one_tick_past(memory_sec, dt):
    # oracle: spec -- an infinite cap is held while t - t_stored <= memory_sec.
    # Times are k * dt (a product, as the mirror computes them). dt 0.125 is
    # binary-exact; dt 0.025 is the real 40 Hz scan interval.
    memory = ot.FollowCapMemory(memory_sec)
    assert memory.update(0.4, 0.0) == (0.4, False)
    last_hold = round(memory_sec / dt)
    for k in range(1, last_hold + 1):
        assert memory.update(math.inf, k * dt) == (0.4, True), k
    assert memory.update(math.inf, (last_hold + 1) * dt) == (math.inf, False)


def test_follow_memory_of_zero_disables_the_hold():
    # oracle: spec -- memory_sec == 0 always returns (follow_cap, False).
    memory = ot.FollowCapMemory(0.0)
    assert memory.update(0.4, 0.0) == (0.4, False)
    assert memory.update(math.inf, 0.0) == (math.inf, False)
    assert memory.update(math.inf, 0.025) == (math.inf, False)
    assert memory.update(0.9, 0.05) == (0.9, False)
    assert memory.update(math.inf, 0.075) == (math.inf, False)
    # oracle: the same spec -- NaN comes back as it is, not turned into no cap.
    # Removing the zero guard turns it into inf; this line catches that.
    cap, held = memory.update(math.nan, 0.1)
    assert math.isnan(cap) and held is False


def test_a_new_finite_cap_replaces_the_stored_value_and_restarts_the_hold():
    # oracle: spec -- the newest finite cap is the one held, and its hold runs
    # from its own stored time (0.9 s), not from the first cap (0.0 s).
    memory = ot.FollowCapMemory(1.0)
    assert memory.update(0.4, 0.0) == (0.4, False)
    for k in range(1, 36):                       # 0.025 .. 0.875 s: the old cap
        assert memory.update(math.inf, k * 0.025) == (0.4, True), k
    assert memory.update(0.9, 36 * 0.025) == (0.9, False)    # t = 0.9 s
    for k in range(37, 77):                      # to 1.9 s: 1.0 s after the new cap
        assert memory.update(math.inf, k * 0.025) == (0.9, True), k
    # From t=0 the hold would have ended at 1.0 s, so this is the restart.
    assert memory.update(math.inf, 77 * 0.025) == (math.inf, False)


@pytest.mark.parametrize('second_time', [0.5, 0.45])
def test_a_repeated_or_backwards_clock_clears_the_hold(second_time):
    # oracle: spec -- a non-monotonic t_s clears the memory. The rule is the one
    # OpponentAwareness.observe uses, 0 < dt <= 0.5 s, so a repeated stamp (dt 0,
    # second_time 0.5) clears it too; 0.45 is a step backwards.
    memory = ot.FollowCapMemory(1.0)
    assert memory.update(0.4, 0.5) == (0.4, False)
    assert memory.update(math.inf, second_time) == (math.inf, False)
    # Nothing is stored any more, so the clock moving on does not bring it back.
    assert memory.update(math.inf, 0.525) == (math.inf, False)


@pytest.mark.parametrize('gap, held', [(0.5, True), (0.5 + 2.0 ** -20, False)])
def test_a_gap_over_half_a_second_clears_the_hold_and_exactly_half_does_not(gap, held):
    # Binary-exact times (0.25 and 0.75) make the boundary exact. The hold would
    # still be inside memory_sec here (0.75 + 2**-20 < 1.0 s), so only the clock
    # rule can release it.
    memory = ot.FollowCapMemory(1.0)
    assert memory.update(0.4, 0.0) == (0.4, False)
    assert memory.update(math.inf, 0.25) == (0.4, True)
    # oracle: observe's staleness rule, 0 < dt <= 0.5 s, inclusive at exactly 0.5.
    expected = (0.4, True) if held else (math.inf, False)
    assert memory.update(math.inf, 0.25 + gap) == expected


def test_a_nan_cap_reads_as_no_cap_and_clears_the_hold():
    # oracle: spec -- NaN is no cap: clear, and return (inf, False).
    memory = ot.FollowCapMemory(1.0)
    assert memory.update(0.4, 0.0) == (0.4, False)
    assert memory.update(math.nan, 0.025) == (math.inf, False)
    assert memory.update(math.inf, 0.05) == (math.inf, False)


def test_the_held_cap_never_exceeds_the_last_stored_finite_cap():
    # oracle: invariant over a fixed-seed random run. Whenever a cap is held it
    # is the last finite cap stored, and never higher. The expected answer for
    # every call comes from a reference model written here from the spec, not
    # from the implementation: a finite cap is stored with its time; NaN and a
    # stale or backwards clock clear the store; an infinite cap is held while
    # t - t_stored <= 1.0 s (inclusive).
    rng = np.random.default_rng(20261028)
    memory = ot.FollowCapMemory(1.0)
    stored = None                 # (cap, time) as the spec defines it
    k = 0                         # tick index, t = k * 0.025 (no accumulated roundoff)
    holds = stale = nans = 0
    for _ in range(4000):
        u = rng.random()
        step = 24 if u < 0.03 else (-1 if u < 0.05 else 1)   # 0.6 s gap, backwards, normal
        if step != 1:
            stored = None
            stale += 1
        k += step
        t = k * 0.025
        u = rng.random()
        if u < 0.02:
            cap = math.nan
        elif u < 0.45:
            cap = float(rng.uniform(0.0, 3.0))
        else:
            cap = math.inf
        if math.isnan(cap):
            expected = (math.inf, False)
            stored = None
            nans += 1
        elif math.isfinite(cap):
            expected = (cap, False)
            stored = (cap, t)
        elif stored is not None and t - stored[1] <= 1.0 + 1e-12:
            expected = (stored[0], True)
            holds += 1
        else:
            expected = (math.inf, False)
        got = memory.update(cap, t)
        assert got == expected, (k, cap, got, expected)
        if got[1]:
            # oracle: invariant -- a held cap is finite and never above the stored one.
            assert math.isfinite(got[0]) and got[0] <= stored[0], (k, got, stored)
    # The run must exercise every branch, or the invariant proves little.
    assert holds >= 100 and stale >= 50 and nans >= 20, (holds, stale, nans)


@pytest.mark.parametrize('value', [-0.1, -1e-9, math.nan, math.inf])
def test_follow_memory_must_be_finite_and_not_negative(value):
    # oracle: spec -- validate() requires finite and >= 0; -1e-9 is one step past 0.
    with pytest.raises(ValueError, match='opponent_follow_memory_sec must be >= 0'):
        ot.OpponentParams(follow_memory_sec=value).validate()


def test_follow_memory_defaults_to_one_second_and_zero_is_accepted():
    # oracle: spec (Interfaces) -- default 1.0 s; 0.0 is the off switch and validates.
    assert ot.OpponentParams().follow_memory_sec == 1.0
    ot.OpponentParams(follow_memory_sec=0.0).validate()


def _layer():
    return ot.OpponentAwareness(ot.OpponentParams(), WHEELBASE, LASER_X, 0.0, CAR_LENGTH)


@pytest.mark.parametrize('memory_sec, expected', [(0.0, (math.inf, False)),
                                                  (1.0, (0.4, True))])
def test_the_layer_passes_its_memory_parameter_to_the_memory(memory_sec, expected):
    # oracle: spec -- follow_memory_sec is the memory's hold; 0 holds nothing.
    layer = ot.OpponentAwareness(ot.OpponentParams(follow_memory_sec=memory_sec),
                                 WHEELBASE, LASER_X, 0.0, CAR_LENGTH)
    layer.follow_memory.update(0.4, 0.0)
    assert layer.follow_memory.update(math.inf, 0.025) == expected


def test_resetting_the_layer_forgets_the_held_cap():
    # oracle: spec -- the memory is reset wherever the node resets its opponent layer.
    layer = _layer()
    layer.follow_memory.update(0.4, 0.0)
    layer.reset()
    assert layer.follow_memory.update(math.inf, 0.025) == (math.inf, False)


def test_a_bad_scan_interval_forgets_the_held_cap():
    # oracle: the same reset path as test_layer_bad_dt_resets_history. A dt that is
    # not positive makes the layer start over, and the memory starts over with it.
    layer = _layer()
    layer.follow_memory.update(0.4, 0.0)
    r, v = _scan(_box(1.0, 0.0))
    layer.observe(r, v, ANGLES, 1.0, 0.0, math.nan)
    assert layer.follow_memory.update(math.inf, 0.025) == (math.inf, False)


@pytest.mark.parametrize('bad_time', [math.nan, math.inf, -math.inf])
def test_a_non_finite_time_clears_the_memory_and_passes_the_cap_through(bad_time):
    # oracle: spec -- a t_s that is not finite clears the memory and returns
    # (follow_cap, False): the cap given comes back as it is, never held. The
    # layer's own reset is a separate path, so this is the memory's own branch.
    memory = ot.FollowCapMemory(1.0)
    assert memory.update(0.4, 0.0) == (0.4, False)
    assert memory.update(0.9, bad_time) == (0.9, False)
    assert memory.update(math.inf, 0.025) == (math.inf, False)
