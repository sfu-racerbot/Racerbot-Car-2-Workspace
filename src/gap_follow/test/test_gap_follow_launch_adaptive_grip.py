"""gap_follow_launch.py's adaptive_grip:= argument (docs/gap-follow.md#adaptive-grip-imu).

Spec: the argument's description in launch/gap_follow_launch.py. adaptive_grip
reads true and false the way steering_damping does (the same _flag), refuses
anything else before anything starts, and ships false. true sets
enable_adaptive_grip on the node and starts imu_bridge's vehicle_state_launch.py
once: the grip needs the gyro, as steering damping does. With both gyro features
on the IMU stack still starts once, not twice.

Nothing is started: the launch description is built and its node-building step
evaluated, as in test_gap_follow_launch_steering_damping.py, whose helpers this
file reuses. Needs ROS sourced and the workspace built:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/gap_follow/test/test_gap_follow_launch_adaptive_grip.py -v
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_gap_follow_launch_steering_damping import (  # noqa: E402
    _gap_node, _launch, _node_params, gl)


def test_the_argument_defaults_to_false():
    # oracle: spec -- docs/gap-follow.md#adaptive-grip-imu: the feature ships off.
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    ld = gl.generate_launch_description()
    defaults = {e.name: ''.join(s.perform(LaunchContext()) for s in e.default_value)
                for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert defaults['adaptive_grip'] == 'false'


def test_true_turns_the_grip_on_and_starts_the_imu_stack():
    nodes, includes, ctx = _launch(adaptive_grip='true')
    assert _node_params(_gap_node(nodes), ctx)['enable_adaptive_grip'] is True
    assert len(includes) == 1
    assert includes[0].launch_description_source.location.endswith('/vehicle_state_launch.py')


def test_false_keeps_the_grip_off_and_starts_no_imu_stack():
    # Damping ships on (2026-10-27), and it starts the IMU stack too, so both
    # switches are pinned off here: only then does "no IMU stack" hold.
    nodes, includes, ctx = _launch(steering_damping='false')
    assert _node_params(_gap_node(nodes), ctx)['enable_adaptive_grip'] is False
    assert includes == []


def test_both_gyro_features_start_the_imu_stack_once():
    nodes, includes, ctx = _launch(steering_damping='true', adaptive_grip='true')
    params = _node_params(_gap_node(nodes), ctx)
    assert params['enable_steering_damping'] is True
    assert params['enable_adaptive_grip'] is True
    assert len(includes) == 1


def test_steering_damping_alone_still_starts_the_imu_stack_once():
    nodes, includes, ctx = _launch(steering_damping='true')
    assert _node_params(_gap_node(nodes), ctx)['enable_adaptive_grip'] is False
    assert len(includes) == 1


@pytest.mark.parametrize('text', ['yes', 'on', '', 'maybe', '2'])
def test_anything_else_is_refused_with_the_argument_named(text):
    with pytest.raises(RuntimeError, match='adaptive_grip'):
        gl._flag('adaptive_grip', text)


def test_the_node_step_refuses_a_bad_value_before_it_builds_anything():
    with pytest.raises(RuntimeError, match='adaptive_grip'):
        _launch(adaptive_grip='maybe')
