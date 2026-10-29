"""Coverage for the generated closed-loop tracks.

No ROS. One test loads the imported map into F1TENTH Gym (it skips without
the gym checkout at .sim/), so:

    python3 -m pytest src/racerbot_sim/test/test_tracks.py -v
"""

import math
import sys
import os
import shutil

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_sim import tracks  # noqa: E402


def _lap_length(xy):
    return float(np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T).sum())


def test_centerline_is_a_closed_loop_of_the_right_size():
    line = tracks.rounded_rectangle_centerline(half_x=5.0, half_y=3.2, radius=1.8)
    # Perimeter of a rounded rectangle: the straights plus one full circle.
    expected = 4.0 * (5.0 - 1.8) + 4.0 * (3.2 - 1.8) + 2.0 * math.pi * 1.8
    assert _lap_length(line) == pytest.approx(expected, rel=0.01)
    assert np.hypot(*(line[0] - line[-1])) < 0.1, 'the loop must close'


def test_centerline_starts_on_a_straight_facing_positive_x():
    """A car spawned mid-corner starts its mapping lap already steering,
    which is not how anyone places a car on a course."""
    line = tracks.rounded_rectangle_centerline(half_x=5.0, half_y=3.2, radius=1.8)
    assert line[0][0] == pytest.approx(0.0, abs=1e-9)
    assert line[0][1] == pytest.approx(-3.2, abs=1e-9)
    heading = math.atan2(line[1][1] - line[0][1], line[1][0] - line[0][0])
    assert heading == pytest.approx(0.0, abs=1e-6)


def test_centerline_is_evenly_sampled():
    line = tracks.rounded_rectangle_centerline(5.0, 3.2, 1.8, spacing=0.05)
    steps = np.hypot(*np.diff(line, axis=0).T)
    assert steps.max() < 0.055
    assert steps.min() > 0.045


@pytest.mark.parametrize('radius', [0.0, -1.0, 4.0])
def test_centerline_rejects_a_radius_that_does_not_fit(radius):
    with pytest.raises(ValueError):
        tracks.rounded_rectangle_centerline(half_x=5.0, half_y=3.2, radius=radius)


def test_occupancy_is_free_on_the_line_and_walled_beside_it():
    line = tracks.rounded_rectangle_centerline(3.0, 2.0, 1.0)
    image, origin_x, origin_y = tracks.occupancy_from_centerline(
        line, corridor_width=1.0, resolution=0.05, margin=0.5)
    height = image.shape[0]

    def cell(x, y):
        column = int((x - origin_x) / 0.05)
        row = height - 1 - int((y - origin_y) / 0.05)
        return image[row, column]

    for point in line[::20]:
        assert cell(*point) == tracks.FREE
    # 0.9m off the line is past the 0.5m half-corridor.
    assert cell(0.0, -2.0 - 0.9) == tracks.OCCUPIED
    assert cell(0.0, -2.0 + 0.9) == tracks.OCCUPIED


def test_every_named_layout_builds_and_is_big_enough_to_close_a_lap():
    """`minimum_lap_distance` and `minimum_lap_duration_sec` are real gates.
    A layout the supervisor could never declare closed is not a test."""
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        for name in tracks.LAYOUTS:
            path = tracks.build(name, directory)
            assert path.parent.joinpath(f'{name}_map.yaml').exists()
            assert path.parent.joinpath(f'{name}.png').exists()
            line = tracks.load_centerline(directory, name)
            assert _lap_length(line) > 20.0, f'{name} is too short to map'


def _config_params(package, config, node):
    path = os.path.join(os.path.dirname(__file__), '..', '..',
                        package, 'config', config)
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)[node]['ros__parameters']


def _min_turning_radius():
    """The tightest circle any part of the stack will drive or plan.

    Read out of the configs rather than written here, because it moves: the
    wheelbase was believed to be the 0.324 m Traxxas publishes until the car
    was tape-measured at 0.36 m on 2026-08-24 (1.22 m became 1.35 m), and the
    2026-10-16 steering calibration raised the controllers to 0.349 rad and
    the race planner to 0.30 rad.

    The planner's circle is the wider of the two and the one that decides
    whether a course is raced: auto_map_race refuses a line tighter than its
    profile_max_steering_angle, so a layout that clears only gap_follow's
    circle would be a course the race stack cannot plan. Bicycle model:
    kappa = tan(delta) / L, and the radius is 1/kappa.
    """
    gap = _config_params('gap_follow', 'gap_follow.yaml', 'gap_follow_node')
    plan = _config_params('pure_pursuit', 'auto_map_race.yaml',
                          'auto_map_race_node')
    return max(
        gap['wheelbase'] / math.tan(gap['max_steering_angle']),
        plan['profile_wheelbase'] / math.tan(plan['profile_max_steering_angle']))


def test_every_layout_corner_can_actually_be_driven():
    """No layout may have a corner this car physically cannot get round.

    The condition is NOT "the centreline radius exceeds the turning radius":
    the car does not have to drive the centreline. It can enter a corner at
    the outside wall and clip the inside one, so the widest line available
    through a corner has radius about

        centreline radius + (corridor width - car width) / 2

    and that is what has to clear the turning circle. indoor_tight's 1.10 m
    centreline is INSIDE the planner's 1.16 m circle (and asb_10000's 1.20 m
    was inside the old 1.35 m one) -- driveable only because the corridor
    gives the car somewhere to swing out to, and calling them undriveable
    would be wrong.

    The version of this test before the car was re-measured compared the
    bare centreline radius against 0.9 x the turning radius. The 0.9 had no
    stated derivation and happened to sit 2 mm under indoor_tight; with the
    corrected wheelbase it would have failed two layouts that are in fact
    perfectly driveable.
    """
    turning_radius = _min_turning_radius()
    # Cited spec: docs/hardware-reference.md (2026-10-16 calibration) gives the
    # planner 0.30 rad on the 0.36 m wheelbase, 0.36 / tan(0.30) = 1.164 m.
    assert turning_radius == pytest.approx(0.36 / math.tan(0.30), abs=0.005), (  # 5 mm
        'the configured geometry no longer gives the race planner the '
        'documented 1.16 m circle; check auto_map_race.yaml / gap_follow.yaml '
        'against docs/hardware-reference.md')
    car_width = _car_width()
    assert tracks.LAYOUTS, 'no layouts to check'
    for name, spec in tracks.LAYOUTS.items():
        usable = spec['corridor_width'] - car_width
        assert usable > 0.0, f'{name} corridor is narrower than the car'
        widest_line = spec['radius'] + usable / 2.0
        assert widest_line >= turning_radius, (
            f'{name}: the widest line through its corners is '
            f'{widest_line:.2f} m, tighter than the car\'s '
            f'{turning_radius:.2f} m turning circle')


def _car_width():
    path = os.path.join(os.path.dirname(__file__), '..', '..',
                        'gap_follow', 'config', 'gap_follow.yaml')
    with open(path, encoding='utf-8') as handle:
        params = yaml.safe_load(handle)['gap_follow_node']['ros__parameters']
    return params['car_width']


CAR1_COURSE = 'car1_course_2026_10_26'


def _copy_car1_course(directory):
    """The imported car 1 course, copied into `directory` (never linked: a hard
    link fails across filesystems, and tmp_path is often a different one)."""
    source = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'tools', 'f1tenth_sim', 'maps')
    for suffix in ('_map.yaml', '_centerline.csv', '.png'):
        shutil.copy2(os.path.join(source, CAR1_COURSE + suffix),
                     directory / (CAR1_COURSE + suffix))
    return directory


def test_resolve_generated_layout_builds_the_track(tmp_path):
    path = tracks.resolve('indoor_oval', tmp_path)
    assert path == tmp_path / 'indoor_oval.yaml'
    assert (tmp_path / 'indoor_oval_map.yaml').exists()


def test_resolve_imported_map_returns_stem_without_writing(tmp_path):
    _copy_car1_course(tmp_path)
    copies = sorted(CAR1_COURSE + suffix for suffix in ('_map.yaml', '_centerline.csv', '.png'))
    for item in tmp_path.iterdir():
        # A write would leave a fresh mtime behind; the copies start at 1970.
        os.utime(item, ns=(0, 0))
    assert tracks.resolve(CAR1_COURSE, tmp_path) == tmp_path / CAR1_COURSE
    assert sorted(item.name for item in tmp_path.iterdir()) == copies
    for item in tmp_path.iterdir():
        assert item.stat().st_mtime_ns == 0, f'resolve wrote {item.name}'


def test_unknown_track_is_refused_and_names_the_directory(tmp_path):
    with pytest.raises(KeyError, match='no_such_track') as refused:
        tracks.resolve('no_such_track', tmp_path)
    assert str(tmp_path) in refused.value.args[0]
    assert 'indoor_oval' in refused.value.args[0], 'the message should list the layouts'


def test_centerline_for_generated_layout_does_not_write(tmp_path):
    line = tracks.centerline_for('indoor_oval', tmp_path)
    # oracle: closed form, layout begins at (0, -half_y), and its rounded perimeter.
    assert line[0] == pytest.approx([0.0, -3.2], abs=1e-9)  # metres
    expected = 4 * (5 - 1.8) + 4 * (3.2 - 1.8) + 2 * math.pi * 1.8
    assert _lap_length(line) == pytest.approx(expected, abs=0.002)  # metres, polygon discretization
    assert list(tmp_path.iterdir()) == []


def test_centerline_for_imported_map_reads_csv_without_writing(tmp_path):
    # oracle: invariant, reads exactly the supplied CSV, without requiring a map build.
    csv = tmp_path / 'imported_centerline.csv'
    csv.write_text('x_m,y_m\n1,2\n3,4\n5,6\n')
    os.utime(csv, ns=(0, 0))
    assert tracks.centerline_for('imported', tmp_path).tolist() == [[1, 2], [3, 4], [5, 6]]
    assert csv.stat().st_mtime_ns == 0
    assert list(tmp_path.iterdir()) == [csv]


def test_imported_car1_course_starts_car_one_on_its_centreline(tmp_path):
    """Gym-backed: SimBridge loads the imported map and puts car 1 on the
    first centreline point, outside every wall."""
    from racerbot_sim import sim_bridge
    sim_bridge.add_sim_python_path()
    try:
        import f1tenth_gym  # noqa: F401
    except ModuleNotFoundError as exc:
        if exc.name != 'f1tenth_gym':
            raise
        pytest.skip('needs the gym checkout at .sim/ (docs/ros-simulator.md); runs wherever racerbot_sim runs')
    _copy_car1_course(tmp_path)
    centerline = tracks.load_centerline(tmp_path, CAR1_COURSE)
    path = tracks.resolve(CAR1_COURSE, tmp_path)
    bridge = sim_bridge.SimBridge(sim_bridge.SimConfig(
        track_path=str(path), centerline=centerline, odom_speed_noise_std=0.0))
    try:
        state = bridge.agent_state(0)
        # oracle: car 1 starts at offset 0.0 of the centreline, the first point; 5 cm allowed.
        assert math.hypot(state.x - centerline[0][0], state.y - centerline[0][1]) <= 0.05
        assert not bridge.body_contact(0)
    finally:
        bridge.close()
