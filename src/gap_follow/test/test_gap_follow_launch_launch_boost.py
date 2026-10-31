"""gap_follow_launch.py's launch_boost:= argument (docs/gap-follow.md#launch-boost).

Spec: the argument's description in launch/gap_follow_launch.py. launch_boost
reads true and false through the same _flag as steering_damping, refuses
anything else before anything starts, and ships true (requested 2026-10-30).
The three mapping launches always turn it off: mapping is one car on an empty
course, with no race start.

Nothing is started: the launch description is built and its node-building step
evaluated, as in test_gap_follow_launch_steering_damping.py, whose helpers this
file reuses. Needs ROS sourced and the workspace built:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/gap_follow/test/test_gap_follow_launch_launch_boost.py -v
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_gap_follow_launch_steering_damping import (  # noqa: E402
    _gap_node, _launch, _node_params, gl)

MAPPING_LAUNCHES = ('map_course_launch.py', 'auto_map_race_launch.py',
                    'autonomous_mapping_launch.py')


def test_the_argument_defaults_to_true():
    # oracle: spec -- the user asked for the launch on every race start (2026-10-30).
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    ld = gl.generate_launch_description()
    defaults = {e.name: ''.join(s.perform(LaunchContext()) for s in e.default_value)
                for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert defaults['launch_boost'] == 'true'


@pytest.mark.parametrize('text, expected', [('true', True), ('false', False)])
def test_the_argument_reaches_the_node(text, expected):
    nodes, _includes, ctx = _launch(launch_boost=text)
    assert _node_params(_gap_node(nodes), ctx)['enable_launch_boost'] is expected


@pytest.mark.parametrize('text', ['yes', '', 'ture'])
def test_anything_else_is_refused(text):
    with pytest.raises(RuntimeError, match='launch_boost:='):
        _launch(launch_boost=text)


@pytest.mark.parametrize('name', MAPPING_LAUNCHES)
def test_mapping_launches_turn_it_off(name):
    # oracle: spec -- mapping is one car, no race start. Read from the source,
    # since these launches need a running bringup to evaluate.
    path = os.path.join(HERE, '..', '..', 'racerbot_launch', 'launch', name)
    with open(path, encoding='utf-8') as handle:
        text = handle.read()
    assert "'enable_launch_boost': False" in text
