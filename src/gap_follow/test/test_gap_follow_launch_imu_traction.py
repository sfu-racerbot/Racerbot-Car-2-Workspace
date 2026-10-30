"""Traction flag and shared include; ROS-sourced package pytest runner."""
import itertools

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from test_gap_follow_launch_steering_damping import gl, _gap_node, _launch, _node_params


def test_traction_launch_default_is_off():
    defaults = {e.name: ''.join(s.perform(LaunchContext()) for s in e.default_value)
                for e in gl.generate_launch_description().entities
                if isinstance(e, DeclareLaunchArgument)}
    nodes, _, ctx = _launch()
    # oracle: spec I5: traction ships off.
    assert defaults['imu_traction'] == 'false'
    assert _node_params(_gap_node(nodes), ctx)['enable_imu_traction'] is False


@pytest.mark.parametrize('flags', list(itertools.product(['false', 'true'], repeat=5)))
def test_traction_shares_one_stack_for_every_flag_combination(flags):
    nodes, includes, ctx = _launch(**dict(zip(
        ('steering_damping', 'adaptive_grip', 'fused_speed', 'imu_understeer', 'imu_traction'),
        flags)), imu_hardware='false')
    # oracle: invariant: the OR of the five flags owns exactly one include.
    assert _node_params(_gap_node(nodes), ctx)['enable_imu_traction'] == (flags[-1] == 'true')
    assert len(includes) == int('true' in flags)
    for include in includes:
        assert include.launch_description_source.location.endswith('/vehicle_state_launch.py')
        assert dict(include.launch_arguments)['hardware'] == 'false'


def test_invalid_traction_launch_flag_is_refused():
    # oracle: spec: malformed flag is refused before a driving node starts.
    with pytest.raises(RuntimeError, match='imu_traction'):
        _launch(imu_traction='maybe')
