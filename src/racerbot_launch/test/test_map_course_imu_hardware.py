"""Evaluate real launch wiring without starting processes or joining a ROS graph.

Runner: python3 -m pytest src/racerbot_launch/test/ (ROS sourced), also colcon
test --packages-select racerbot_launch. Spec: I1 part b, approved 2026-10-29,
and map_course_launch.py's IMU docstring: auto selects simulated inputs when
the simulator process exists; explicit values win and imu:=false omits the stack.
"""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest


SRC = Path(__file__).resolve().parents[2]


def _load(package, filename):
    spec = importlib.util.spec_from_file_location(filename, SRC / package / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(ld, **overrides):
    ctx = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            ctx.launch_configurations[entity.name] = overrides.get(
                entity.name, perform_substitutions(ctx, entity.default_value))
    return ctx


def _imu_includes(probe, **overrides):
    # Car 2's map_course defaults to imu:=false (docs/car1-vs-car2.md#imu), so
    # these source-selection tests ask for the stack; a test may still pass
    # imu explicitly, and test_car2_default_starts_no_imu_stack covers the default.
    overrides.setdefault('imu', 'true')
    return _launch_imu_includes(probe, **overrides)


def _launch_imu_includes(probe, **overrides):
    module = _load('racerbot_launch', 'map_course_launch.py')
    ld = module.generate_launch_description(imu_probe=probe)
    ctx = _context(ld, **overrides)
    includes = []
    for entity in ld.entities:
        if isinstance(entity, OpaqueFunction):
            for action in entity.execute(ctx):
                if isinstance(action, IncludeLaunchDescription):
                    includes.append(action)
                elif action.__class__.__name__ == 'SetLaunchConfiguration':
                    action.execute(ctx)
        elif isinstance(entity, IncludeLaunchDescription):
            includes.append(entity)
    enabled = []
    for include in includes:
        if include.condition is not None and not include.condition.evaluate(ctx):
            continue
        include.launch_description_source.get_launch_description(ctx)
        if include.launch_description_source.location.endswith('/vehicle_state_launch.py'):
            enabled.append({key: perform_substitutions(
                ctx, normalize_to_list_of_substitutions(value))
                            for key, value in include.launch_arguments})
    return ctx, enabled


def test_imu_hardware_defaults_to_auto():
    ld = _load('racerbot_launch', 'map_course_launch.py').generate_launch_description()
    # oracle: spec -- I1 part b, map_course_launch.py IMU source default.
    assert _context(ld).launch_configurations['imu_hardware'] == 'auto'


@pytest.mark.parametrize('simulated,expected', [(True, 'false'), (False, 'true')])
def test_auto_passes_detected_hardware_to_vehicle_state(simulated, expected):
    _, includes = _imu_includes(lambda: simulated)
    # oracle: spec -- I1 part b: real readers only without a simulator.
    assert len(includes) == 1
    assert includes[0]['hardware'] == expected


@pytest.mark.parametrize('requested', ['true', 'false'])
def test_explicit_imu_hardware_wins_without_probing(requested):
    def forbidden_probe():
        raise AssertionError('explicit IMU source must not query processes')

    _, includes = _imu_includes(forbidden_probe, imu_hardware=requested)
    # oracle: spec -- I1 part b: explicit selections override detection.
    assert len(includes) == 1
    assert includes[0]['hardware'] == requested


def test_invalid_imu_hardware_refuses_to_launch():
    # oracle: spec -- I1 part b: only auto/true/false are valid selections.
    with pytest.raises(RuntimeError, match='imu_hardware'):
        _imu_includes(lambda: True, imu_hardware='maybe')


def test_imu_false_omits_vehicle_state_include():
    _, includes = _imu_includes(lambda: True, imu='false')
    # oracle: spec -- I1 part b: imu:=false disables the whole stack.
    assert includes == []


def test_car2_default_starts_no_imu_stack():
    # oracle: spec -- docs/car1-vs-car2.md#imu: car 2's map_course imu
    # default is false, so with nothing passed no IMU source is chosen and
    # vehicle_state_launch.py is not included, simulator or not.
    for simulated in (True, False):
        ctx, includes = _launch_imu_includes(lambda: simulated)
        assert ctx.launch_configurations['imu'] == 'false'
        assert 'imu_selected_hardware' not in ctx.launch_configurations
        assert includes == []


def test_gyro_odometry_accepts_simulated_imu():
    ctx, includes = _imu_includes(lambda: False, gyro_odom='true', imu_hardware='false')
    # oracle: spec -- map_course_launch.py gyro_odom contract: frame and TF
    # switch together; hardware:=false still provides the estimator.
    assert ctx.launch_configurations['slam_odom_frame'] == 'odom_imu'
    assert ctx.launch_configurations['publish_odom_tf'] == 'true'
    assert len(includes) == 1
    assert includes[0]['hardware'] == 'false'


@pytest.mark.parametrize('requested,expected', [(None, 1.0), ('1.1', 1.1)])
def test_sim_bringup_passes_odom_yaw_rate_scale_to_gym_bridge(requested, expected):
    ld = _load('racerbot_sim', 'sim_bringup_launch.py').generate_launch_description()
    overrides = {} if requested is None else {'odom_yaw_rate_scale': requested}
    ctx = _context(ld, **overrides)
    nodes = []

    def visit(entities):
        for entity in entities:
            if isinstance(entity, Node) and entity.node_executable == 'gym_bridge_node':
                nodes.append(entity)
            elif isinstance(entity, GroupAction):
                visit(entity.get_sub_entities())

    visit(ld.entities)
    # oracle: spec -- I1 part b: default unity, explicit 1.1 models 10% error.
    assert ctx.launch_configurations['odom_yaw_rate_scale'] == str(expected)
    assert len(nodes) == 1
    params = evaluate_parameters(ctx, nodes[0]._Node__parameters)
    assert params[0]['odom_yaw_rate_scale'] == expected
