"""Coverage for turning a recorded SLAM lap into a drivable racing line.

Framework-agnostic (numpy only), so:

    python3 -m pytest src/pure_pursuit/test/test_recorded_path.py -v

Each test names the specific failure it protects against; between them
they cover every defect found in the three laps this car actually
recorded on 2026-07-27 (`~/.ros/racerbot_auto`).
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pure_pursuit import racing_math, recorded_path  # noqa: E402


CAR_STEERING = 0.26
CAR_WHEELBASE = 0.324
RACK_LIMIT = math.tan(CAR_STEERING) / CAR_WHEELBASE


def _circle(radius=3.0, count=200, start_angle=0.0, turns=1.0):
    angles = start_angle + np.linspace(
        0.0, turns * 2.0 * math.pi, int(count * turns), endpoint=False)
    return np.column_stack([radius * np.cos(angles), radius * np.sin(angles)])


def _jittered(path, sigma, seed=7):
    rng = np.random.default_rng(seed)
    return path + rng.normal(0.0, sigma, path.shape)


# --- curvature_limit ------------------------------------------------------

def test_curvature_limit_matches_the_bicycle_model():
    assert recorded_path.curvature_limit(0.26, 0.324) == pytest.approx(RACK_LIMIT)
    assert 1.0 / RACK_LIMIT == pytest.approx(1.218, abs=0.01)


@pytest.mark.parametrize('steering,wheelbase', [
    (0.0, 0.324), (-0.26, 0.324), (math.pi, 0.324), (0.26, 0.0), (0.26, -1.0),
])
def test_curvature_limit_rejects_impossible_geometry(steering, wheelbase):
    with pytest.raises(ValueError):
        recorded_path.curvature_limit(steering, wheelbase)


# --- lowpass_closed -------------------------------------------------------

def test_lowpass_keeps_the_loop_the_same_size():
    """Regression: repeated moving-average smoothing is curve-shortening
    flow. The first attempt at this cleanup used it and collapsed a 30m
    recorded lap to a 0.0m dot in 19-25 passes, reporting zero curvature --
    a point being, technically, very feasible. A spatial low-pass must not
    shrink the loop however hard it filters.
    """
    circle = _circle(radius=3.0, count=200)
    expected = racing_math.compute_segment_lengths(circle, closed=True).sum()
    for harmonics in (40, 10, 3, 1):
        filtered = recorded_path.lowpass_closed(circle, harmonics)
        length = racing_math.compute_segment_lengths(filtered, closed=True).sum()
        assert length == pytest.approx(expected, rel=1e-6)


def test_lowpass_removes_jitter_but_keeps_the_shape():
    circle = _circle(radius=3.0, count=200)
    noisy = _jittered(circle, 0.05)
    filtered = recorded_path.lowpass_closed(noisy, 6)
    radii = np.hypot(filtered[:, 0], filtered[:, 1])
    assert radii.mean() == pytest.approx(3.0, abs=0.05)
    assert radii.std() < 0.02          # the wobble is gone
    assert np.hypot(*(noisy - circle).T).std() > radii.std()


def test_lowpass_leaves_short_paths_alone():
    stub = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    assert np.allclose(recorded_path.lowpass_closed(stub, 4), stub)


# --- drop_repeated_points -------------------------------------------------

def test_drop_repeated_points_removes_duplicates_including_the_wrap():
    points = np.array([[0.0, 0.0], [0.0, 0.0], [1.0, 0.0],
                       [1.0, 1.0], [0.0, 0.0]])
    kept = recorded_path.drop_repeated_points(points)
    assert len(kept) == 3
    assert np.allclose(kept, [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])


def test_duplicate_points_leave_a_zero_length_segment_behind():
    """A car recorded twice in the same spot is not a corner, but it is a
    zero-length segment -- an undefined tangent, and a division by ~zero in
    the three-point curvature estimate. estimate_path_curvature guards the
    division and returns 0 there, which hides the problem rather than
    fixing it; the duplicate still distorts arc length and resampling."""
    points = np.vstack([_circle(3.0, 60), _circle(3.0, 60)[:1]])
    segments = racing_math.compute_segment_lengths(points, closed=True)
    assert segments.min() == pytest.approx(0.0, abs=1e-9)

    cleaned = recorded_path.drop_repeated_points(points)
    assert len(cleaned) == 60
    assert racing_math.compute_segment_lengths(cleaned, closed=True).min() > 0.1


# --- resample_closed ------------------------------------------------------

def test_resample_gives_uniform_spacing():
    """The recorded lap is sampled by distance travelled but never evenly:
    real recordings mix 0.15m steps with 0.44-0.69m ones wherever the pose
    jumped. Uneven spacing reads as curvature to a three-point estimate."""
    uneven = np.array([[0.0, 0.0], [0.1, 0.0], [2.0, 0.0],
                       [2.0, 2.0], [0.0, 2.0]])
    resampled = recorded_path.resample_closed(uneven, 0.15)
    steps = np.hypot(*np.diff(resampled, axis=0).T)
    # Spacing is uniform in *arc* length, so a step that straddles one of
    # this shape's four right-angle corners has a shorter chord than the
    # arc. That is correct behaviour, not a gap -- what matters is that no
    # step is ever longer than asked for, and none is dramatically shorter.
    assert steps.max() <= 0.151
    assert steps.min() > 0.7 * 0.15
    assert np.median(steps) == pytest.approx(0.15, abs=0.01)


@pytest.mark.parametrize('bad', [
    np.zeros((3, 2)),                       # too few points
    np.zeros((10, 2)),                      # zero length
])
def test_resample_rejects_degenerate_input(bad):
    with pytest.raises(ValueError):
        recorded_path.resample_closed(bad, 0.15)


# --- last_revolution ------------------------------------------------------

def test_last_revolution_trims_a_two_lap_recording():
    """The defect this exists for: `minimum_lap_distance: 20.0` is longer
    than this car's ~15m room, so the closure gate could not open until the
    car had been round twice, and all three laps it recorded are 1.96-1.98
    revolutions. Two overlapping passes are not a closed racing line."""
    two_laps = _circle(radius=3.0, count=200, turns=2.0)
    trimmed = recorded_path.last_revolution(two_laps)
    assert len(trimmed) == pytest.approx(200, abs=3)
    length = np.hypot(*np.diff(trimmed, axis=0).T).sum()
    assert length == pytest.approx(2.0 * math.pi * 3.0, rel=0.02)


def test_last_revolution_leaves_a_single_lap_untouched():
    one_lap = _circle(radius=3.0, count=200)
    assert len(recorded_path.last_revolution(one_lap)) == len(one_lap)


def test_last_revolution_survives_a_jittering_pose():
    """An earlier implementation integrated heading to count revolutions.
    On a pose jittering 8-15 degrees per sample -- which is what this car's
    recordings do -- the cumulative turn is not monotonic, the search lands
    20-30 degrees either side of a lap, and the shortfall or overlap leaves
    a hairpin in the seam. Distance to a fixed point has no such problem.

    Counted in samples rather than metres: jitter inflates measured path
    length by ~30% (every step picks up noise at both ends), so the length
    of a noisy lap is not a number to assert against.
    """
    noisy = _jittered(_circle(radius=3.0, count=200, turns=2.0), 0.05)
    trimmed = recorded_path.last_revolution(noisy)
    assert len(trimmed) == pytest.approx(200, abs=15)


def test_last_revolution_ignores_paths_too_short_to_judge():
    stub = _circle(3.0, 6)
    assert len(recorded_path.last_revolution(stub)) == len(stub)


# --- seam_heading_error ---------------------------------------------------

def test_seam_heading_error_is_zero_on_a_clean_loop():
    assert recorded_path.seam_heading_error(_circle(3.0, 200)) < math.radians(3.0)


def test_seam_heading_error_finds_a_kink():
    """Measured on the real recordings: 34.8, 38.6 and 110.1 degrees across
    the closing segment -- and the handover happens exactly there, because
    closure is detected at the seam."""
    loop = _circle(3.0, 200)[:150]            # three quarters of a circle
    assert recorded_path.seam_heading_error(loop) > math.radians(30.0)


# --- prepare --------------------------------------------------------------

def test_prepare_makes_a_jittered_lap_drivable():
    noisy = _jittered(_circle(radius=3.0, count=200), 0.05)
    before = racing_math.estimate_path_curvature(noisy, closed=True).max()
    assert before > RACK_LIMIT, 'the fixture must start out undrivable'

    prepared = recorded_path.prepare(
        noisy, spacing=0.15, max_steering_angle=CAR_STEERING,
        wheelbase=CAR_WHEELBASE)
    assert prepared.feasible
    assert prepared.acceptable
    assert prepared.max_curvature < RACK_LIMIT
    assert prepared.max_deviation_m < 0.2
    assert prepared.length_m == pytest.approx(2.0 * math.pi * 3.0, rel=0.05)


def test_prepare_barely_touches_an_already_clean_lap():
    clean = _circle(radius=3.0, count=200)
    prepared = recorded_path.prepare(
        clean, spacing=0.15, max_steering_angle=CAR_STEERING,
        wheelbase=CAR_WHEELBASE)
    assert prepared.max_deviation_m < 0.02
    assert prepared.length_m == pytest.approx(2.0 * math.pi * 3.0, rel=0.01)


def test_prepare_trims_and_cleans_a_two_lap_jittered_recording():
    """The exact shape of every real recording: two revolutions of a
    jittering pose, closed back on an arbitrary point."""
    noisy = _jittered(_circle(radius=3.0, count=200, turns=2.0), 0.05, seed=3)
    prepared = recorded_path.prepare(
        noisy, spacing=0.15, max_steering_angle=CAR_STEERING,
        wheelbase=CAR_WHEELBASE)
    assert prepared.trimmed_points < prepared.raw_points
    assert prepared.length_m == pytest.approx(2.0 * math.pi * 3.0, rel=0.08)
    assert prepared.acceptable


def test_prepare_refuses_a_course_tighter_than_the_car():
    """A 0.4m-radius loop is inside the car's 1.22m turning circle. No
    amount of filtering makes that drivable, and saying so is the point:
    loading it anyway is what saturates the rack and ends the run."""
    tiny = _circle(radius=0.4, count=120)
    prepared = recorded_path.prepare(
        tiny, spacing=0.05, max_steering_angle=CAR_STEERING,
        wheelbase=CAR_WHEELBASE)
    assert not prepared.feasible
    assert not prepared.acceptable
    assert prepared.max_curvature > RACK_LIMIT


def test_prepare_bounds_how_far_it_moves_the_driven_line():
    """Filtering harder always looks better by curvature and eventually
    stops describing the same track. In a 1.4m corridor a 0.4m sideways
    move is a wall."""
    noisy = _jittered(_circle(radius=3.0, count=200), 0.05)
    prepared = recorded_path.prepare(
        noisy, spacing=0.15, max_steering_angle=CAR_STEERING,
        wheelbase=CAR_WHEELBASE, max_deviation=0.1)
    assert prepared.max_deviation_m <= 0.15


def test_prepare_reports_a_seam_it_could_not_fix():
    prepared = recorded_path.prepare(
        _jittered(_circle(radius=3.0, count=200), 0.03), spacing=0.15,
        max_steering_angle=CAR_STEERING, wheelbase=CAR_WHEELBASE)
    assert prepared.seam_heading_error_rad < math.radians(10.0)
    assert 'seam heading error' in prepared.describe()


def test_prepare_rejects_input_that_is_not_a_path():
    with pytest.raises(ValueError):
        recorded_path.prepare(np.zeros((5, 3)), spacing=0.15,
                              max_steering_angle=CAR_STEERING,
                              wheelbase=CAR_WHEELBASE)
    with pytest.raises(ValueError):
        recorded_path.prepare(np.zeros((3, 2)), spacing=0.15,
                              max_steering_angle=CAR_STEERING,
                              wheelbase=CAR_WHEELBASE)


def test_grading_separates_slightly_wide_from_undrivable():
    """`feasible` and `acceptable` are different questions. A line a little
    past the rack at one apex understeers and pure pursuit pulls it back; a
    line far past it saturates the steering and ends the run."""
    marginal = recorded_path.PreparedPath(
        xy=np.zeros((4, 2)), raw_points=4, trimmed_points=4, length_m=10.0,
        harmonics_kept=3, shortest_wavelength_m=3.3, max_deviation_m=0.1,
        min_wall_clearance_m=0.5, driven_wall_clearance_m=0.6,
        required_wall_clearance_m=0.3, configured_wall_clearance_m=0.3,
        max_curvature=RACK_LIMIT * 1.2, curvature_limit=RACK_LIMIT,
        rack_curvature_limit=RACK_LIMIT, fraction_over_limit=0.10,
        reject_ratio=1.5, reject_fraction=0.25,
        max_steering_rad=0.30, max_steering_limit_rad=0.26,
        seam_heading_error_rad=0.05, max_segment_m=0.15)
    assert not marginal.feasible
    assert marginal.acceptable

    hopeless = recorded_path.PreparedPath(
        **{**marginal.__dict__,
           'max_curvature': RACK_LIMIT * 3.5, 'fraction_over_limit': 0.34})
    assert not hopeless.feasible
    assert not hopeless.acceptable


# --- wall clearance -------------------------------------------------------

def _corridor_clearance(centre_radius=3.0, half_width=0.5):
    """Clearance field of an annular corridor centred on the unit loop."""
    def clearance(xs, ys):
        radius = np.hypot(np.asarray(xs), np.asarray(ys))
        return np.maximum(0.0, half_width - np.abs(radius - centre_radius))
    return clearance


def test_clearance_is_not_checked_without_a_map():
    """Silence is not a pass. With no map the result says so rather than
    reporting a clearance it never measured."""
    prepared = recorded_path.prepare(
        _circle(3.0, 200), spacing=0.15, max_steering_angle=CAR_STEERING,
        wheelbase=CAR_WHEELBASE)
    assert prepared.min_wall_clearance_m == -1.0
    assert prepared.fits_the_track
    assert 'not checked' in prepared.describe()


def test_a_prepared_line_closer_than_required_does_not_fit():
    """The failure this check exists for. Filtering rounds a corner
    *inward*, toward the chord and therefore toward the wall, and on a
    course whose corners are near the car's own 1.22m turning circle that is
    the direction that hurts. A measured run finished with the line 0.05m
    from a wall -- less than the car's 0.155m half-width -- while peak
    curvature, seam error and deviation all looked healthy.

    Asserted on the decision rather than on contrived geometry: a circle
    low-passes to a circle, so reproducing an inward-pulled corner in a
    fixture takes more machinery than it tests.
    """
    through_a_wall = recorded_path.PreparedPath(
        xy=np.zeros((4, 2)), raw_points=200, trimmed_points=200,
        length_m=27.0, harmonics_kept=18, shortest_wavelength_m=1.5,
        max_deviation_m=0.03, min_wall_clearance_m=0.05,
        driven_wall_clearance_m=0.35, required_wall_clearance_m=0.20,
        configured_wall_clearance_m=0.20,
        max_curvature=RACK_LIMIT * 0.85, curvature_limit=RACK_LIMIT,
        rack_curvature_limit=RACK_LIMIT, fraction_over_limit=0.0,
        reject_ratio=1.5, reject_fraction=0.25,
        max_steering_rad=0.22, max_steering_limit_rad=0.26,
        seam_heading_error_rad=0.02, max_segment_m=0.15)
    assert through_a_wall.feasible, 'the curvature checks see nothing wrong'
    assert not through_a_wall.fits_the_track
    assert not through_a_wall.acceptable
    assert not through_a_wall.map_has_ghosts, 'the driven line had room'
    assert 'closest wall 0.05m, needs 0.20m' in through_a_wall.describe()


def test_the_requirement_is_capped_by_what_driving_achieved():
    """The lap that just happened is proof that a line fits. Where the map
    disagrees, the map is the thing that is wrong."""
    prepared = recorded_path.prepare(
        _circle(3.0, 200), spacing=0.15, max_steering_angle=CAR_STEERING,
        wheelbase=CAR_WHEELBASE,
        clearance_fn=lambda xs, ys: np.full(len(np.asarray(xs)), 0.10),
        required_clearance=0.30)
    assert prepared.driven_wall_clearance_m == pytest.approx(0.10)
    assert prepared.required_wall_clearance_m == pytest.approx(0.10)
    assert prepared.configured_wall_clearance_m == pytest.approx(0.30)
    assert prepared.fits_the_track
    assert prepared.map_has_ghosts


def test_a_line_with_room_around_it_is_accepted():
    prepared = recorded_path.prepare(
        _jittered(_circle(3.0, 200), 0.03), spacing=0.15,
        max_steering_angle=CAR_STEERING, wheelbase=CAR_WHEELBASE,
        clearance_fn=_corridor_clearance(3.0, 0.6), required_clearance=0.30)
    assert prepared.min_wall_clearance_m >= 0.30
    assert prepared.fits_the_track
    assert prepared.acceptable


def test_staying_inside_the_track_outranks_curvature():
    """The two constraints pull opposite ways -- filtering harder always
    improves curvature and always moves the line inward -- so the search is
    a trade, and the tie-break has to be the one that cannot be traded
    away. A line through a wall is not a slower racing line."""
    noisy = _jittered(_circle(3.0, 200), 0.06, seed=11)
    # A corridor that only tolerates a line very close to the driven one.
    tight = recorded_path.prepare(
        noisy, spacing=0.15, max_steering_angle=CAR_STEERING,
        wheelbase=CAR_WHEELBASE,
        clearance_fn=_corridor_clearance(3.0, 0.35), required_clearance=0.28)
    loose = recorded_path.prepare(
        noisy, spacing=0.15, max_steering_angle=CAR_STEERING,
        wheelbase=CAR_WHEELBASE)
    assert tight.fits_the_track
    # Keeping clearance means keeping more of the original line.
    assert tight.max_deviation_m <= loose.max_deviation_m + 1e-9


def test_a_map_with_other_cars_in_it_does_not_veto_the_lap_that_happened():
    """Mapping with traffic paints the other cars into the grid, so the
    driven line itself measures as passing through a wall. Refusing on that
    would be refusing to believe a lap the ego demonstrably completed --
    measured, in the two-car scenario, as 0.00m of clearance on a line the
    car had just driven twice without touching anything. The requirement is
    capped by what driving actually achieved."""
    prepared = recorded_path.prepare(
        _circle(3.0, 200), spacing=0.15, max_steering_angle=CAR_STEERING,
        wheelbase=CAR_WHEELBASE,
        clearance_fn=lambda xs, ys: np.zeros(len(np.asarray(xs))),
        required_clearance=0.30)
    assert prepared.driven_wall_clearance_m == pytest.approx(0.0)
    assert prepared.map_has_ghosts
    assert prepared.fits_the_track
    assert prepared.acceptable
    assert 'phantom obstacles' in prepared.describe()
