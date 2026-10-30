"""IMU launch-source selection; evaluate actions without starting processes.

Run with rb env ROS_DOMAIN_ID=78 python3 -m pytest src/gap_follow/test/.
Oracles: I1 round-1 controller ruling and map_course's pgrep -f probe.
Seen failing with auto always choosing hardware, the simulator overriding
explicit values, unconditional inclusion, and accepted invalid values.
"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.utilities import perform_substitutions
import pytest

LAUNCH = Path(__file__).resolve().parents[1] / 'launch/gap_follow_launch.py'


def load_launch():
    spec = importlib.util.spec_from_file_location('gap_follow_launch_imu_hardware', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_actions(running, **overrides):
    calls = []

    def probe():
        calls.append(True)
        return running

    description = load_launch().generate_launch_description(imu_probe=probe)
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value or [])
    context.launch_configurations.update(overrides)
    (opaque,) = [a for a in description.entities if isinstance(a, OpaqueFunction)]
    result = opaque.execute(context)
    includes = [a for a in result if isinstance(a, IncludeLaunchDescription)]
    logs = [perform_substitutions(context, a.msg) for a in result if isinstance(a, LogInfo)]
    return includes, logs, calls, context


def include_arguments(include, context):
    include.launch_description_source.get_launch_description(context)
    # oracle: the selected hardware flag is passed to the actual estimator
    # launch, not to a different include with an unused argument.
    assert include.launch_description_source.location.endswith('/vehicle_state_launch.py')
    return dict(include.launch_arguments)


def test_imu_hardware_defaults_to_auto():
    description = load_launch().generate_launch_description()
    context = LaunchContext()
    defaults = {a.name: perform_substitutions(context, a.default_value or [])
                for a in description.entities if isinstance(a, DeclareLaunchArgument)}
    # oracle: I1 round-1 requires automatic source choice for the bare launch.
    assert defaults['imu_hardware'] == 'auto'


@pytest.mark.parametrize('running,expected,reason', [
    (True, 'false', 'simulator detected'),
    (False, 'true', 'simulator not detected'),
])
@pytest.mark.parametrize('feature', ['steering_damping', 'adaptive_grip'])
def test_auto_selects_source_from_process_probe(running, expected, reason, feature):
    overrides = dict(steering_damping='false', adaptive_grip='false')
    overrides[feature] = 'true'
    includes, logs, calls, context = launch_actions(running, **overrides)
    # oracle: auto selects simulated topics iff gym_bridge_node is running;
    # either gyro feature needs exactly one estimator include.
    assert len(includes) == 1
    assert include_arguments(includes[0], context) == {'hardware': expected}
    assert calls == [True]
    assert len(logs) == 1
    assert reason in logs[0]
    assert ('simulated IMU' if running else 'real IMU') in logs[0]


@pytest.mark.parametrize('running', [True, False])
@pytest.mark.parametrize('requested,expected', [('true', 'true'), ('false', 'false'),
                                              ('TRUE', 'true'), ('FALSE', 'false')])
def test_explicit_source_overrides_probe(running, requested, expected):
    # Car 2: steering_damping:=true written out (car 1's default, car 2's is
    # false: docs/car1-vs-car2.md#imu), so an IMU feature needs a source.
    includes, logs, calls, context = launch_actions(
        running, imu_hardware=requested, steering_damping='true')
    # oracle: an explicit value wins in either environment and needs no probe.
    assert len(includes) == 1
    assert include_arguments(includes[0], context) == {'hardware': expected}
    assert calls == []
    assert len(logs) == 1
    assert 'explicit' in logs[0]


@pytest.mark.parametrize('requested', ['maybe', '', '1', '0'])
def test_invalid_imu_hardware_is_refused(requested):
    # oracle: only auto/true/false are specified; argument name diagnoses typos.
    # Car 2: steering_damping:=true written out, as above; with every IMU
    # feature off the source is unused and not checked
    # (test_no_imu_include_when_both_features_are_off).
    with pytest.raises(RuntimeError, match='imu_hardware'):
        launch_actions(True, imu_hardware=requested, steering_damping='true')


@pytest.mark.parametrize('requested', ['auto', 'true', 'false', 'maybe'])
@pytest.mark.parametrize('running', [True, False])
def test_no_imu_include_when_both_features_are_off(requested, running):
    includes, logs, calls, _ = launch_actions(running, imu_hardware=requested,
        steering_damping='false', adaptive_grip='false')
    # oracle: no IMU feature means no stack, no source choice and no probe,
    # regardless of the unused source argument.
    assert includes == []
    assert logs == []
    assert calls == []


@pytest.mark.parametrize('command,expected', [
    ('/ws/install/racerbot_sim/lib/racerbot_sim/gym_bridge_node --ros-args', True),
    ('/usr/bin/python3 /ws/install/racerbot_sim/lib/racerbot_sim/gym_bridge_node', True),
    ('/bin/bash -c wait-for gym_bridge_node', False),
    ('/bin/bash -c wait-for /ws/install/racerbot_sim/lib/racerbot_sim/gym_bridge_node', False),
])
def test_auto_probe_uses_bridge_executable_not_incidental_shell_text(command, expected, monkeypatch):
    import re
    from types import SimpleNamespace
    from gap_follow import launch_imu

    def probe(argv, **kwargs):
        assert argv[:2] == ['pgrep', '-f']
        return SimpleNamespace(returncode=0 if re.search(argv[2], command) else 1)

    monkeypatch.setattr(launch_imu.subprocess, 'run', probe)
    hardware, reason = launch_imu.select_imu_hardware('auto')
    # oracle: spec R5b: a shell mentioning the bridge is still hardware mode.
    assert hardware == ('false' if expected else 'true')
    assert ('simulator detected' if expected else 'simulator not detected') in reason
