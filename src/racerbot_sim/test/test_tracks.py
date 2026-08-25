"""Coverage for the generated closed-loop tracks.

No ROS and no F1TENTH Gym, so:

    python3 -m pytest src/racerbot_sim/test/test_tracks.py -v
"""

import math
import sys
import os

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


def _min_turning_radius():
    """This car's tightest circle, from the two configured numbers.

    Read out of gap_follow.yaml rather than written here, because it moved:
    the wheelbase was believed to be the 0.324 m Traxxas publishes until the
    car was tape-measured at 0.36 m on 2026-08-24, and a LONGER wheelbase
    turns WIDER for the same rack angle -- 1.22 m became 1.35 m. A test
    carrying its own copy of that number would have kept asserting the old,
    more permissive limit.
    """
    path = os.path.join(os.path.dirname(__file__), '..', '..',
                        'gap_follow', 'config', 'gap_follow.yaml')
    with open(path, encoding='utf-8') as handle:
        params = yaml.safe_load(handle)['gap_follow_node']['ros__parameters']
    # Bicycle model: kappa = tan(delta) / L, and the radius is 1/kappa.
    return params['wheelbase'] / math.tan(params['max_steering_angle'])


def test_every_layout_corner_can_actually_be_driven():
    """No layout may have a corner this car physically cannot get round.

    The condition is NOT "the centreline radius exceeds the turning radius":
    the car does not have to drive the centreline. It can enter a corner at
    the outside wall and clip the inside one, so the widest line available
    through a corner has radius about

        centreline radius + (corridor width - car width) / 2

    and that is what has to clear the turning circle. asb_10000's 1.20 m and
    indoor_tight's 1.10 m centrelines are both INSIDE the 1.35 m circle --
    they are driveable only because the corridor gives the car somewhere to
    swing out to, and calling them undriveable would be wrong.

    The version of this test before the car was re-measured compared the
    bare centreline radius against 0.9 x the turning radius. The 0.9 had no
    stated derivation and happened to sit 2 mm under indoor_tight; with the
    corrected wheelbase it would have failed two layouts that are in fact
    perfectly driveable.
    """
    turning_radius = _min_turning_radius()
    assert turning_radius == pytest.approx(1.353, abs=0.01), (
        'the configured geometry no longer gives this car a 1.35 m circle; '
        'check gap_follow.yaml wheelbase/max_steering_angle')
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


def test_unknown_layout_is_refused():
    with pytest.raises(KeyError):
        tracks.build('no_such_track', '/tmp')
