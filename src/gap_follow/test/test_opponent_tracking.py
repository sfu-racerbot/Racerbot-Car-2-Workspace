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

def test_stationary_disc_entry_range_is_closed_form():
    # Disc of radius R = radius + passing_margin centred d ahead of the
    # laser: the beam straight at it enters at d - R.
    params = ot.OpponentParams(radius=0.3, passing_margin=0.15, prediction_horizon=0.0)
    window = np.full(ANGLES.size, MAX_RANGE)
    out = ot.inflate_predicted(window, ANGLES, [(LASER_X + 2.0, 0.0, 0.0, 0.0)],
                               LASER_X, 0.0, params)
    mid = int(np.argmin(np.abs(ANGLES)))
    assert out[mid] == pytest.approx(2.0 - 0.45, abs=1e-9)
    # A beam at bearing b grazes it when 2*sin(b) = R.
    graze = math.asin(0.45 / 2.0)
    assert np.all(out[np.abs(ANGLES) > graze + 0.01] == MAX_RANGE)
    assert np.all(out[np.abs(ANGLES) < graze - 0.01] < MAX_RANGE)
    assert window[mid] == MAX_RANGE                        # input not modified


def test_inflation_never_lengthens_a_beam():
    params = ot.OpponentParams(prediction_horizon=0.0)
    window = np.full(ANGLES.size, 1.0)
    out = ot.inflate_predicted(window, ANGLES, [(LASER_X + 3.0, 0.0, 0.0, 0.0)],
                               LASER_X, 0.0, params)
    assert np.array_equal(out, window)


def test_prediction_blocks_where_the_car_is_going_not_only_where_it_is():
    # A car 2 m ahead driving left at 2 m/s will be at y = +1.6 m after 0.8 s.
    window = np.full(ANGLES.size, MAX_RANGE)
    opp = [(LASER_X + 2.0, 0.0, 0.0, 2.0)]
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


def test_laser_inside_a_disc_gets_the_floor():
    params = ot.OpponentParams(radius=0.3, passing_margin=0.15, prediction_horizon=0.0)
    window = np.full(ANGLES.size, MAX_RANGE)
    out = ot.inflate_predicted(window, ANGLES, [(LASER_X + 0.2, 0.0, 0.0, 0.0)],
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
    x, y, vx, vy, near_x = movers[0]
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
    # 2 m ahead and 1 m left of the laser, driving +x at 2 m/s: in 0.8 s it
    # is 3.6 m ahead. A beam at 10 deg misses where it is now (its disc
    # spans ~15-38 deg) but crosses where it is going.
    window = np.full(ANGLES.size, MAX_RANGE)
    opp = [(LASER_X + 2.0, 1.0, 2.0, 0.0)]
    beam = int(np.argmin(np.abs(ANGLES - math.radians(10.0))))
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
