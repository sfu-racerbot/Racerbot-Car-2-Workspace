"""Evaluate launch actions without starting hardware; pytest / colcon test.

Seen failing when hardware=false still starts imu_bridge_node, and against
an empty action-builder stub (including true-stack and invalid-value cases).
"""

import importlib.util
from pathlib import Path
import subprocess

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch.utilities import perform_substitutions
import pytest

LAUNCH = Path(__file__).resolve().parents[1] / 'launch/vehicle_state_launch.py'


def load_launch():
    spec = importlib.util.spec_from_file_location('vehicle_state_launch_hardware', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def actions(module, **overrides):
    description = module.generate_launch_description()
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value or [])
    context.launch_configurations.update(overrides)
    opaque = [e for e in description.entities if isinstance(e, OpaqueFunction)]
    # oracle: launch's node-building step is evaluated exactly once.
    assert len(opaque) == 1
    return opaque[0].execute(context), context


def executable(node, context):
    value = node.node_executable
    return value if isinstance(value, str) else perform_substitutions(context, value)


@pytest.mark.parametrize('hardware', ['false', 'FALSE', 'False'])
@pytest.mark.parametrize('bno', ['true', 'false'])
def test_hardware_false_starts_only_vehicle_state(hardware, bno):
    result, context = actions(load_launch(), hardware=hardware, bno=bno,
        racer_imu_root='/does/not/exist', bno_pty='/tmp/test-only-bno', publish_odom_tf='true')
    nodes = [a for a in result if isinstance(a, Node)]
    # oracle: I1 launch contract; Node inherits ExecuteProcess, so distinguish
    # the standalone bridge process from ROS Node actions.
    assert [executable(n, context) for n in nodes] == ['vehicle_state_node']
    assert not [a for a in result if isinstance(a, ExecuteProcess) and not isinstance(a, Node)]
    assert len([a for a in result if isinstance(a, LogInfo)]) == 1
    parameters = nodes[0]._Node__parameters
    # oracle: simulator supplies the BNO; hardware=false must not override
    # accel_source from vehicle_state.yaml even if bno=false is passed.
    keys = [perform_substitutions(context, key)
            for entry in parameters if isinstance(entry, dict) for key in entry]
    assert 'accel_source' not in keys
    assert 'publish_odom_tf' in keys


@pytest.mark.parametrize('hardware', ['true', 'TRUE'])
def test_hardware_true_keeps_real_stack(tmp_path, monkeypatch, hardware):
    root = tmp_path / 'imu-calibration'
    script = root / 'tools/bno_i2c_bridge.py'
    script.parent.mkdir(parents=True)
    script.write_text('# test-only bridge path; never executed\n')
    module = load_launch()
    # Process boundary: deterministic pgrep result, without starting hardware.
    monkeypatch.setattr(module.subprocess, 'run', lambda *args, **kwargs:
                        subprocess.CompletedProcess(args[0], 1))
    result, context = actions(module, hardware=hardware, bno='true',
        racer_imu_root=str(root), bno_pty='/tmp/test-only-bno', publish_odom_tf='false')
    nodes = [a for a in result if isinstance(a, Node)]
    processes = [a for a in result if isinstance(a, ExecuteProcess) and not isinstance(a, Node)]
    # oracle: the pre-existing vehicle_state_launch.py hardware stack consists
    # of the I2C bridge, calibrated IMU bridge, and estimator.
    assert [executable(n, context) for n in nodes] == ['imu_bridge_node', 'vehicle_state_node']
    assert len(processes) == 1
    command = [perform_substitutions(context, part) for part in processes[0].cmd]
    assert command == ['python3', '-u', str(script), '--pty', '/tmp/test-only-bno', '--no-dyncal']


@pytest.mark.parametrize('hardware', ['maybe', '', '1', '0'])
def test_hardware_rejects_unknown_values(hardware):
    # oracle: hardware accepts only case-insensitive true/false, not launch's
    # usual permissive boolean aliases, so typos cannot start real hardware.
    with pytest.raises(RuntimeError, match='hardware'):
        actions(load_launch(), hardware=hardware)
