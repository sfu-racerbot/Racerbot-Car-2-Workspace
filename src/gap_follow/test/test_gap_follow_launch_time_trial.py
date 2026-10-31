"""gap_follow_launch.py's time_trial:= argument (docs/gap-follow.md#speed-on-the-real-course-2026-10-30).

Spec: the argument's description in launch/gap_follow_launch.py. It ships false,
reads true and false through the same _flag as the others, refuses anything else
before anything starts, and when true forces opponent awareness off, because the
aim it sets does worse with another car on the track.

Nothing is started: the launch description is built and its node-building step
evaluated, as in test_gap_follow_launch_steering_damping.py, whose helpers this
file reuses. Needs ROS sourced:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/gap_follow/test/test_gap_follow_launch_time_trial.py -v
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_gap_follow_launch_steering_damping import (  # noqa: E402
    _gap_node, _launch, _node_params, gl)


def test_the_argument_defaults_to_false():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    ld = gl.generate_launch_description()
    defaults = {e.name: ''.join(s.perform(LaunchContext()) for s in e.default_value)
                for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert defaults['time_trial'] == 'false'


def test_off_it_sets_nothing_and_leaves_opponent_awareness_on():
    nodes, _includes, ctx = _launch()
    params = _node_params(_gap_node(nodes), ctx)
    assert 'aim_depth_bias' not in params
    assert 'max_lateral_accel' not in params
    assert params['enable_opponent_awareness'] is True


def test_on_it_sets_the_time_trial_values():
    nodes, _includes, ctx = _launch(time_trial='true')
    params = _node_params(_gap_node(nodes), ctx)
    # oracle: spec -- docs/gap-follow.md#speed-on-the-real-course-2026-10-30.
    assert params['aim_depth_bias'] == 1.0
    assert params['max_lateral_accel'] == 3.0


def test_on_it_forces_opponent_awareness_off_even_if_asked_for():
    # The refusal that matters: this aim is worse with another car on the track.
    nodes, _includes, ctx = _launch(time_trial='true', opponent_awareness='true')
    assert _node_params(_gap_node(nodes), ctx)['enable_opponent_awareness'] is False


@pytest.mark.parametrize('text', ['yes', '', 'ture'])
def test_anything_else_is_refused(text):
    with pytest.raises(RuntimeError, match='time_trial:='):
        _launch(time_trial=text)
