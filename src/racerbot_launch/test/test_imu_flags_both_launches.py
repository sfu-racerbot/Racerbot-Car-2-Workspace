"""Shared IMU launch contract; ROS-sourced pytest and racerbot_launch colcon test."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import pytest

SRC = Path(__file__).resolve().parents[2]
PATHS = (SRC / 'racerbot_launch/launch/map_course_launch.py',
         SRC / 'gap_follow/launch/gap_follow_launch.py')
SHARED = {'imu', 'imu_all', 'fused_speed', 'imu_hardware'}
MAP_ONLY = {'gyro_odom', 'imu_lap_turns'}
GAP_ONLY = {'steering_damping', 'adaptive_grip', 'imu_understeer',
            'imu_traction', 'imu_impact_stop'}


def _load(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('overrides,expected', [
    ({}, False),
    ({'imu_all': 'true'}, True),
    ({'imu_all': 'true', 'fused_speed': 'false'}, False),
    ({'fused_speed': 'true'}, True),
    ({'imu': 'false'}, False),
    ({'imu': 'false', 'fused_speed': 'true'}, None),
    ({'imu': 'false', 'imu_all': 'true'}, None),
])
def test_shared_imu_flags_and_override_rule(overrides, expected):
    # oracle: plan Task J1: "one table-driven test that `map_course` and
    # `gap_follow_launch.py` accept the same flag names and the same `imu_all` override rule".
    descriptions = [_load(path).generate_launch_description(imu_probe=lambda: True)
                    for path in PATHS]
    arguments = [{e.name: e for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
                 for ld in descriptions]
    relevant = SHARED | MAP_ONLY | GAP_ONLY
    assert set(arguments[0]) & relevant == SHARED | MAP_ONLY
    assert set(arguments[1]) & relevant == SHARED | GAP_ONLY
    defaults = [{name: ''.join(s.perform(LaunchContext()) for s in args[name].default_value)
                 for name in SHARED} for args in arguments]
    # Car 2: map_course's imu defaults to false (docs/car1-vs-car2.md#imu,
    # guarded by test_car2_hardware_config.py); gap_follow_launch.py's imu
    # only permits the stack and stays true, as on car 1 (both true there).
    shared = {'imu_all': 'false', 'fused_speed': 'false', 'imu_hardware': 'auto'}
    assert defaults[0] == dict(shared, imu='false')
    assert defaults[1] == dict(shared, imu='true')
    for ld in descriptions:
        ctx = LaunchContext()
        # The same override rule in both launches, so imu is given the same
        # value in both: written out as true unless the row sets it.
        ctx.launch_configurations.update(dict({'imu': 'true'}, **overrides))
        for entity in ld.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(ctx)
        (step,) = [e for e in ld.entities if isinstance(e, OpaqueFunction)][:1]
        if expected is None:
            with pytest.raises(RuntimeError, match=r'imu:=true'):
                step.execute(ctx)
        else:
            step.execute(ctx)
            assert ctx.launch_configurations['fused_speed'] == str(expected).lower()
