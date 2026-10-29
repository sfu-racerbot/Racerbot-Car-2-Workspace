"""gap_follow_launch.py's steering_damping:= argument (docs/gap-follow.md#steering-damping-gyro).

Spec: the argument's description in launch/gap_follow_launch.py, and the
_flag() that file copies from racerbot_launch's map_course_launch.py: 'true'
or '1' for on, 'false' or '0' for off, and anything else refused before
anything starts. true also starts imu_bridge's vehicle_state_launch.py (the
IMU stack that publishes /imu/data and /imu/status) and sets
enable_steering_damping on the node. false does neither. Car 1 defaults to
true since 2026-10-27 (the race defaults); car 2 defaults to false until its
BNO085 is mounted and calibrated (docs/car1-vs-car2.md#imu), so a bare launch
here starts no IMU stack.

Nothing is started: the launch description is built, and its node-building
step is evaluated with launch's own LaunchContext. Needs ROS sourced and the
workspace built:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/gap_follow/test/test_gap_follow_launch_steering_damping.py -v
"""
import importlib.util
import os

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node

LAUNCH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'launch', 'gap_follow_launch.py')


def _load():
    spec = importlib.util.spec_from_file_location('gap_follow_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gl = _load()


def _node_params(node, ctx):
    """A launch_ros Node's dict parameters, evaluated in ctx. launch_ros keeps
    them private (name-mangled), as test_map_course_imu.py also reads them."""
    params = {}
    for entry in node._Node__parameters:
        if isinstance(entry, dict):
            for key, value in entry.items():
                name = ''.join(x.perform(ctx) for x in key)
                if isinstance(value, tuple) and all(hasattr(x, 'perform') for x in value):
                    value = yaml.safe_load(''.join(x.perform(ctx) for x in value))
                params[name] = value
    return params


def _launch(**overrides):
    """The launch's arguments at their declared defaults (with overrides), and
    the node-building step evaluated as launch evaluates it. Returns the nodes,
    the included launch descriptions, and the context they were built in."""
    ld = gl.generate_launch_description()
    ctx = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = ''.join(s.perform(ctx) for s in entity.default_value or [])
            ctx.launch_configurations[entity.name] = overrides.get(entity.name, default)
    (step,) = [e for e in ld.entities if isinstance(e, OpaqueFunction)]
    actions = step.execute(ctx)
    nodes = [a for a in actions if isinstance(a, Node)]
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    for include in includes:
        include.launch_description_source.get_launch_description(ctx)  # resolves location
    return nodes, includes, ctx


def _gap_node(nodes):
    gap = [n for n in nodes if n.node_package == 'gap_follow']
    assert len(gap) == 1
    return gap[0]


def test_the_argument_defaults_to_false_on_car_2():
    # oracle: spec -- docs/car1-vs-car2.md#imu: off on car 2 until the BNO085 is
    # mounted (car 1: on, docs/gap-follow.md#race-defaults-2026-10-27).
    ld = gl.generate_launch_description()
    defaults = {e.name: ''.join(s.perform(LaunchContext()) for s in e.default_value)
                for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert defaults['steering_damping'] == 'false'


def test_true_turns_damping_on_and_starts_the_imu_stack():
    nodes, includes, ctx = _launch(steering_damping='true')
    assert _node_params(_gap_node(nodes), ctx)['enable_steering_damping'] is True
    assert len(includes) == 1
    assert includes[0].launch_description_source.location.endswith('/vehicle_state_launch.py')


def test_the_shipped_default_keeps_damping_off_and_starts_no_imu_stack():
    # Bare launch, no argument: what car 2 runs by default. Without the BNO085
    # and ~/imu-calibration, the IMU stack refuses to start (docs/car1-vs-car2.md#imu).
    nodes, includes, ctx = _launch()
    assert _node_params(_gap_node(nodes), ctx)['enable_steering_damping'] is False
    assert includes == []


def test_false_keeps_damping_off_and_starts_no_imu_stack():
    nodes, includes, ctx = _launch(steering_damping='false')
    assert _node_params(_gap_node(nodes), ctx)['enable_steering_damping'] is False
    assert includes == []


@pytest.mark.parametrize('text, expected', [
    ('true', True), ('1', True), (' TRUE ', True), ('false', False), ('0', False),
])
def test_the_flag_reads_true_and_false_the_way_map_course_does(text, expected):
    # oracle: spec -- racerbot_launch/launch/map_course_launch.py's _flag().
    assert gl._flag('steering_damping', text) is expected


@pytest.mark.parametrize('text', ['yes', 'on', '', 'maybe', '2'])
def test_anything_else_is_refused_with_the_argument_named(text):
    with pytest.raises(RuntimeError, match='steering_damping'):
        gl._flag('steering_damping', text)


def test_the_node_step_refuses_a_bad_value_before_it_builds_anything():
    with pytest.raises(RuntimeError, match='steering_damping'):
        _launch(steering_damping='maybe')
