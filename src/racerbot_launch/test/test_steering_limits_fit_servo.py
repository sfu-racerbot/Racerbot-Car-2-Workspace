"""
Every steering limit the car ships with must be one the servo can deliver.

A steering command becomes a servo value in ackermann_to_vesc.cpp,
``servo = steering_angle_to_servo_gain * angle + steering_angle_to_servo_offset``,
and vesc_driver then clips that to [servo_min, servo_max] without a warning.
A limit past the clip is steering the car silently never gets, on one side
only. That happened: from 2026-10-10 to 2026-10-16 full stick (0.34 rad) and
the autonomy limit (0.26 rad) both asked for more right steering than
servo_max 0.85 allowed (docs/odom-calibration.md, docs/hardware-reference.md).

Oracle: the closed form above, recomputed from vesc.yaml, applied to the
limits in each config that sets one. Also an invariant: the race planner must
plan below the controller's limit, so the controller has steering left to
correct with on the tightest planned corner (auto_map_race.yaml).

The servo mapping is linear but this car's linkage is not
(docs/hardware-reference.md), so this proves only that the servo is not
clipped, not that the wheels reach the angle.

No ROS needed:

    python3 -m pytest src/racerbot_launch/test/ -v
"""
import os

import pytest
import yaml

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_STACK = os.path.join(_SRC, 'f1tenth_system', 'f1tenth_stack', 'config')


def _params(path, node):
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)[node]['ros__parameters']


def _servo_mapping():
    vesc = _params(os.path.join(_STACK, 'vesc.yaml'), '/**')
    return (vesc['steering_angle_to_servo_gain'],
            vesc['steering_angle_to_servo_offset'],
            vesc['servo_min'], vesc['servo_max'])


def clipped_sides(angle, gain, offset, servo_min, servo_max):
    """The sides ('left', 'right') on which +/-angle rad is clipped."""
    clipped = []
    for side, signed in (('left', angle), ('right', -angle)):
        servo = gain * signed + offset
        if not servo_min <= servo <= servo_max:
            clipped.append(side)
    return clipped


def _limits():
    joy = _params(os.path.join(_STACK, 'joy_teleop.yaml'), 'joy_teleop')
    steering = joy['human_control']['axis_mappings']['drive-steering_angle']
    assert steering['offset'] == 0.0
    return {
        # Full stick: the F710 axis reads +/-1.0.
        'joy_teleop.yaml full stick': abs(steering['scale']),
        'gap_follow.yaml max_steering_angle': _params(
            os.path.join(_SRC, 'gap_follow', 'config', 'gap_follow.yaml'),
            'gap_follow_node')['max_steering_angle'],
        'pure_pursuit.yaml max_steering_angle': _params(
            os.path.join(_SRC, 'pure_pursuit', 'config', 'pure_pursuit.yaml'),
            'pure_pursuit_node')['max_steering_angle'],
        'auto_map_race.yaml profile_max_steering_angle': _params(
            os.path.join(_SRC, 'pure_pursuit', 'config', 'auto_map_race.yaml'),
            'auto_map_race_node')['profile_max_steering_angle'],
    }


@pytest.mark.parametrize('name', sorted(_limits()))
def test_shipped_limit_is_not_clipped_by_the_servo(name):
    angle = _limits()[name]
    assert angle > 0.0
    assert clipped_sides(angle, *_servo_mapping()) == [], (
        f'{name} = {angle} rad is clipped by servo_min/servo_max in vesc.yaml')


def test_planner_plans_below_the_controller_limit():
    limits = _limits()
    assert (limits['auto_map_race.yaml profile_max_steering_angle']
            < limits['pure_pursuit.yaml max_steering_angle'])


# The check itself, on known cases, so it is not only ever seen passing.
# Recorded state 2026-10-10 (docs/odom-calibration.md): gain -1.2135, offset
# 0.5904, clip [0.15, 0.85]; full right at 0.34 rad needs 0.5904 + 0.4126 =
# 1.003, and 0.26 rad needs 0.9059. Both past 0.85, on the right only.
@pytest.mark.parametrize('angle', [0.34, 0.26])
def test_check_catches_the_2026_10_10_right_side_clip(angle):
    assert clipped_sides(angle, -1.2135, 0.5904, 0.15, 0.85) == ['right']


def test_check_boundary_is_inclusive_and_one_step_past_fails():
    # gain -1, offset 0.5: angle 0.35 lands exactly on servo_max 0.85 (and
    # 0.15 on the left). 0.3501 is past both.
    assert clipped_sides(0.35, -1.0, 0.5, 0.15, 0.85) == []
    assert clipped_sides(0.3501, -1.0, 0.5, 0.15, 0.85) == ['left', 'right']


def test_check_catches_a_left_side_clip():
    # Offset low enough that the left end runs out first.
    assert clipped_sides(0.3, -1.0, 0.4, 0.15, 0.85) == ['left']
