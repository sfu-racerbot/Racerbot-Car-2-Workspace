"""Coverage for the generated closed-loop tracks.

No ROS and no F1TENTH Gym, so:

    python3 -m pytest src/racerbot_sim/test/test_tracks.py -v
"""

import math
import sys
import os

import numpy as np
import pytest

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


def test_layout_corners_are_inside_the_cars_turning_circle():
    """tan(0.26)/0.324 is a 1.22m minimum turning radius. A track with a
    tighter corner than that cannot be raced by this car however good the
    mapping is, so no layout here may have one."""
    for name, spec in tracks.LAYOUTS.items():
        assert spec['radius'] > 1.22 * 0.9, f'{name} corners are too tight'


def test_unknown_layout_is_refused():
    with pytest.raises(KeyError):
        tracks.build('no_such_track', '/tmp')
