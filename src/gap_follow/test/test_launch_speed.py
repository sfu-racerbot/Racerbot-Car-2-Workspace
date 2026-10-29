"""
gap_follow_launch.py's ``max_speed:=`` argument (gap_follow/launch_speed.py).

Spec: the argument's description in launch/gap_follow_launch.py, the
module docstring and docs/launchstart.md -- it sets the top speed anywhere up
to MAX_SPEED_CEILING (5.0 m/s, requested 2026-10-26), refuses anything else,
and lowers min_speed / corner_speed / corner_speed_wide with it because the
node refuses to start otherwise (gap_follow_node.py's speed checks); above
the YAML it raises max_speed alone. Oracles: those invariants, recomputed
here, and the node's own published command.

The first half needs no ROS. The node tests need ROS sourced:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/gap_follow/test/test_launch_speed.py -v
"""
import math
import os

import pytest

from gap_follow.launch_speed import (
    CAPPED_WITH_MAX, MAX_SPEED_CEILING, speed_cap_overrides, yaml_parameters)

PACKAGED_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'gap_follow.yaml')
SHIPPED = yaml_parameters(PACKAGED_CONFIG)
# Synthetic, so the expected overrides below follow from these numbers alone.
PARAMS = {'max_speed': 3.5, 'min_speed': 1.0, 'corner_speed': 1.5,
          'corner_speed_wide': 1.9}
# The four speeds the shipped YAML had before the 2026-10-27 race defaults. The
# shipped max_speed is now the 5.0 ceiling itself, so a raise has no headroom on
# the shipped file: the raise path is tested from these instead.
PRE_RACE = {'max_speed': 3.5, 'min_speed': 1.0, 'corner_speed': 1.5,
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


@pytest.mark.parametrize('cap', [0.3, 0.8, 1.0, 1.45, 2.0, 3.0, 3.5, 4.2, 5.0])
def test_shipped_yaml_with_any_cap_satisfies_the_nodes_speed_checks(cap):
    # Invariant oracle: the node starts only if 0 <= min <= max and
    # corner <= corner_wide <= max (gap_follow_node.py).
    merged = {**SHIPPED, **speed_cap_overrides(SHIPPED, str(cap))}
    assert merged['max_speed'] == cap
    assert 0.0 <= merged['min_speed'] <= merged['max_speed']
    assert merged['corner_speed'] <= merged['corner_speed_wide'] <= merged['max_speed']
    for key in CAPPED_WITH_MAX:
        assert merged[key] == min(SHIPPED[key], cap), key


def test_raising_above_the_yaml_changes_max_speed_alone():
    # 4.2 > every speed in PARAMS, so nothing else is above it to lower.
    assert speed_cap_overrides(PARAMS, '4.2') == {'max_speed': 4.2}


def test_ceiling_is_the_requested_five_metres_per_second():
    # Cited spec: the 2026-10-26 request, docs/launchstart.md.
    assert MAX_SPEED_CEILING == 5.0  # m/s


def test_cap_at_the_ceiling_is_allowed_and_one_step_above_is_refused():
    assert speed_cap_overrides(PARAMS, '5.0') == {'max_speed': 5.0}
    for text in ('5.0001', '5.5', '10'):
        with pytest.raises(ValueError, match='above the 5.0 m/s ceiling'):
            speed_cap_overrides(PARAMS, text)


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
def raised():
    node = _node({**PRE_RACE, **speed_cap_overrides(PRE_RACE, str(MAX_SPEED_CEILING))})
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


# The speed the car is moving at in the stop test below.
FAST = 4.5  # m/s


def test_yaml_top_speed_holds_the_command_at_the_yaml_value(uncapped):
    # Already faster than the shipped top speed, so the acceleration ramp is
    # not what holds the command down -- max_speed is. The ramp from 5.5 m/s
    # allows 6.0 m/s on this tick (5.5 + 5.0 x 0.1 s), above the 5.0 cap.
    above_top = SHIPPED['max_speed'] + 0.5  # m/s
    assert _drive(uncapped, OPEN, above_top).speed == pytest.approx(
        SHIPPED['max_speed'], abs=1e-6)  # m/s


# Above the old 3.5 m/s top speed, and low enough that the ramp (4.9 + 5.0 x
# 0.1 s = 5.4 m/s) is not what holds a 5.0 m/s command down: the raised cap is.
RAISE_FROM = 4.9  # m/s


def test_raised_node_commands_above_the_yaml_top_speed(raised):
    assert raised.max_speed == MAX_SPEED_CEILING
    speed = _drive(raised, OPEN, RAISE_FROM).speed
    assert PRE_RACE['max_speed'] + 0.5 < speed  # m/s: above the old top speed
    assert speed == pytest.approx(MAX_SPEED_CEILING, abs=1e-6)  # m/s: the raised cap binds


def test_raised_node_still_stops_when_lb_is_released(raised):
    _drive(raised, OPEN, FAST)
    released = Joy()
    released.buttons = [0] * 5
    raised.joy_callback(released)
    published = []
    original = raised.drive_pub.publish
    raised.drive_pub.publish = lambda msg: (published.append(msg), original(msg))
    raised.scan_callback(OPEN)
    raised.drive_pub.publish = original
    assert published, 'node published nothing'
    assert all(msg.drive.speed == 0.0 for msg in published)
