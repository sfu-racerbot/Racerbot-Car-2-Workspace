"""Fused-speed launch wiring; run with sourced pytest or colcon test gap_follow.

Evaluate the real launch's node-building step without starting processes.
"""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
import pytest
import yaml

SRC = Path(__file__).resolve().parents[2]
LAUNCH = SRC / 'gap_follow' / 'launch' / 'gap_follow_launch.py'
spec = importlib.util.spec_from_file_location('gap_follow_fused_launch', LAUNCH)
gl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gl)


def _context(ld, **overrides):
    ctx = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = ''.join(s.perform(ctx) for s in entity.default_value or [])
            ctx.launch_configurations[entity.name] = overrides.get(entity.name, default)
    return ctx


def _opaque(ld):
    (step,) = [e for e in ld.entities if isinstance(e, OpaqueFunction)]
    return step


def _node_params(node, ctx):
    """Resolve files and dicts in order: ROS parameter overrides are last-wins."""
    params = {}
    for entry in node._Node__parameters:
        if isinstance(entry, ParameterFile):
            path = ''.join(s.perform(ctx) for s in entry.param_file)
            params.update(yaml.safe_load(Path(path).read_text())
                          ['gap_follow_node']['ros__parameters'])
        else:
            for key, value in entry.items():
                name = ''.join(s.perform(ctx) for s in key)
                if isinstance(value, tuple) and all(hasattr(s, 'perform') for s in value):
                    value = yaml.safe_load(''.join(s.perform(ctx) for s in value))
                params[name] = value
    return params


def _launch(**overrides):
    ld = gl.generate_launch_description()
    ctx = _context(ld, **overrides)
    actions = _opaque(ld).execute(ctx)
    (node,) = [a for a in actions if isinstance(a, Node)]
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    for include in includes:
        include.launch_description_source.get_launch_description(ctx)
    return _node_params(node, ctx), includes, ctx


def test_fused_speed_defaults_off_and_keeps_the_yaml_wheel_topic():
    ctx = _context(gl.generate_launch_description())
    # oracle: spec -- Task I3 ships fused_speed off, like map_course_launch.py.
    assert ctx.launch_configurations['fused_speed'] == 'false'
    params, _, _ = _launch()
    wheel = yaml.safe_load((SRC / 'gap_follow/config/gap_follow.yaml').read_text())
    # oracle: spec -- src/gap_follow/config/gap_follow.yaml's odom_topic.
    assert params['odom_topic'] == wheel['gap_follow_node']['ros__parameters']['odom_topic']
    assert params['odom_topic'] == '/odom'


@pytest.mark.parametrize('flags', [
    {},
    {'steering_damping': 'false', 'adaptive_grip': 'false'},
    {'steering_damping': 'true', 'adaptive_grip': 'true'},
    {'steering_damping': 'false', 'adaptive_grip': 'true'},
])
def test_fused_speed_uses_the_estimators_topic_and_includes_it_once(flags):
    params, includes, _ = _launch(fused_speed='true', **flags)
    state = yaml.safe_load((SRC / 'imu_bridge/config/vehicle_state.yaml').read_text())
    # oracle: spec -- src/imu_bridge/config/vehicle_state.yaml's actual output.
    assert params['odom_topic'] == state['vehicle_state_node']['ros__parameters']['fused_odom_topic']
    # oracle: invariant -- shared IMU features must start exactly one estimator.
    assert len(includes) == 1
    assert includes[0].launch_description_source.location.endswith('/vehicle_state_launch.py')


def test_all_three_features_off_includes_no_imu_stack():
    _, includes, _ = _launch(steering_damping='false', adaptive_grip='false',
                             fused_speed='false')
    # oracle: invariant -- no consumer enabled, no IMU stack needed.
    assert includes == []


def test_bad_fused_speed_is_refused_before_starting():
    # oracle: spec -- map_course_launch.py's _flag semantics, mirrored here.
    with pytest.raises(RuntimeError, match='fused_speed'):
        _launch(fused_speed='maybe')


@pytest.mark.parametrize('hardware', ['true', 'false'])
def test_fused_speed_alone_passes_imu_hardware_to_the_shared_include(hardware):
    _, includes, _ = _launch(steering_damping='false', adaptive_grip='false',
                             fused_speed='true', imu_hardware=hardware)
    # oracle: spec -- I1: explicit imu_hardware reaches vehicle_state as hardware.
    assert len(includes) == 1
    assert dict(includes[0].launch_arguments)['hardware'] == hardware
