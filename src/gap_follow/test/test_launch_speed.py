"""
gap_follow_launch.py's ``max_speed:=`` argument (gap_follow/launch_speed.py).

Spec: the argument's description in launch/gap_follow_launch.py and the
module docstring -- it may only lower the speed, refuses anything else, and
lowers min_speed / corner_speed / corner_speed_wide with it because the node
refuses to start otherwise (gap_follow_node.py's speed checks). Oracles: those
invariants, recomputed here, and the node's own published command.

The first half needs no ROS. The node tests need ROS sourced:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/gap_follow/test/test_launch_speed.py -v
"""
import math
import os

import pytest

from gap_follow.launch_speed import (
    CAPPED_WITH_MAX, speed_cap_overrides, yaml_parameters)

PACKAGED_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'gap_follow.yaml')
SHIPPED = yaml_parameters(PACKAGED_CONFIG)
# Synthetic, so the expected overrides below follow from these numbers alone.
PARAMS = {'max_speed': 3.5, 'min_speed': 1.0, 'corner_speed': 1.5,
          'corner_speed_wide': 1.9}


@pytest.mark.parametrize('text', ['', '   '])
def test_empty_argument_leaves_the_yaml_alone(text):
    assert speed_cap_overrides(PARAMS, text) == {}


def test_cap_lowers_max_and_only_the_speeds_above_it():
    # 1.2: corner_speed 1.5 and corner_speed_wide 1.9 are above it, min 1.0 is not.
    assert speed_cap_overrides(PARAMS, '1.2') == {
        'max_speed': 1.2, 'corner_speed': 1.2, 'corner_speed_wide': 1.2}


def test_a_speed_exactly_at_the_cap_is_not_touched():
    # Boundary: min_speed 1.0 == cap 1.0 stays out of the overrides.
    assert speed_cap_overrides(PARAMS, '1.0') == {
        'max_speed': 1.0, 'corner_speed': 1.0, 'corner_speed_wide': 1.0}


def test_cap_below_every_speed_lowers_them_all():
    assert speed_cap_overrides(PARAMS, '0.5') == {
        'max_speed': 0.5, 'min_speed': 0.5, 'corner_speed': 0.5,
        'corner_speed_wide': 0.5}


@pytest.mark.parametrize('cap', [0.3, 0.8, 1.0, 1.45, 2.0, 3.0])
def test_shipped_yaml_with_any_cap_satisfies_the_nodes_speed_checks(cap):
    # Invariant oracle: the node starts only if 0 <= min <= max and
    # corner <= corner_wide <= max (gap_follow_node.py).
    merged = {**SHIPPED, **speed_cap_overrides(SHIPPED, str(cap))}
    assert merged['max_speed'] == cap
    assert 0.0 <= merged['min_speed'] <= merged['max_speed']
    assert merged['corner_speed'] <= merged['corner_speed_wide'] <= merged['max_speed']
    for key in CAPPED_WITH_MAX:
        assert merged[key] == min(SHIPPED[key], cap), key


def test_cap_equal_to_the_yaml_is_allowed_and_one_step_above_is_refused():
    assert speed_cap_overrides(PARAMS, '3.5')['max_speed'] == 3.5
    with pytest.raises(ValueError, match='only lowers the speed'):
        speed_cap_overrides(PARAMS, '3.5001')


@pytest.mark.parametrize('text', ['0', '0.0', '-1.0', 'nan', 'inf', '-inf'])
def test_non_positive_or_non_finite_cap_is_refused(text):
    with pytest.raises(ValueError, match='positive, finite speed'):
        speed_cap_overrides(PARAMS, text)


@pytest.mark.parametrize('text', ['abc', '1,0', '1.0m/s'])
def test_non_numeric_cap_is_refused(text):
    with pytest.raises(ValueError, match='is not a number'):
        speed_cap_overrides(PARAMS, text)


# ---------------------------------------------------------------- the node

import rclpy  # noqa: E402  (ROS sourced: see docstring)
from sensor_msgs.msg import Joy  # noqa: E402

from gap_follow.gap_follow_node import GapFollowNode  # noqa: E402
from test_gap_follow_corners_node import _corridor, _drive, _raycast  # noqa: E402

# Same rule as test_gap_follow_node.py: last in every rclpy.init, never the
# real /drive (enforced by test_no_test_here_builds_a_node_on_the_real_drive).
TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']
CAP = 1.0


def _node(overrides):
    args = ['--ros-args', '--params-file', PACKAGED_CONFIG]
    for key, value in overrides.items():
        args += ['-p', f'{key}:={value}']
    rclpy.init(args=[*args, *TEST_ONLY_DRIVE])
    node = GapFollowNode()
    assert node.drive_pub.topic_name == '/test_only/drive'
    return node


@pytest.fixture
def capped():
    node = _node(speed_cap_overrides(SHIPPED, str(CAP)))
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def uncapped():
    node = _node({})
    yield node
    node.destroy_node()
    rclpy.shutdown()


OPEN = _raycast(_corridor(0.8, 0.8))


def test_same_scene_drives_faster_than_the_cap_without_it(uncapped):
    # Precondition for the next test: this scene is one where the cap binds.
    assert _drive(uncapped, OPEN, CAP).speed > CAP + 0.05


def test_capped_node_drives_but_never_commands_above_the_cap(capped):
    assert capped.max_speed == CAP
    for _ in range(10):
        speed = _drive(capped, OPEN, CAP).speed
        assert 0.0 < speed <= CAP + 1e-9  # m/s


def test_capped_node_still_stops_when_lb_is_released(capped):
    _drive(capped, OPEN, CAP)
    released = Joy()
    released.buttons = [0] * 5
    capped.joy_callback(released)
    published = []
    original = capped.drive_pub.publish
    capped.drive_pub.publish = lambda msg: (published.append(msg), original(msg))
    capped.scan_callback(OPEN)
    capped.drive_pub.publish = original
    assert published, 'node published nothing'
    assert all(msg.drive.speed == 0.0 for msg in published)
