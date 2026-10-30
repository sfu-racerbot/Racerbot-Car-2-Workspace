"""Impact flag wiring; collected by ROS-sourced package pytest.

All 66 cases failed before launch wiring existed, then passed (2026-10-29).
"""
import itertools

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from test_gap_follow_launch_steering_damping import gl, _gap_node, _launch, _node_params


def test_impact_launch_default_off():
    defaults = {e.name: ''.join(s.perform(LaunchContext()) for s in e.default_value)
                for e in gl.generate_launch_description().entities
                if isinstance(e, DeclareLaunchArgument)}
    nodes, _, ctx = _launch()
    # oracle: spec I6: impact stop ships off, including the launch override.
    assert defaults['imu_impact_stop'] == 'false'
    assert _node_params(_gap_node(nodes), ctx)['enable_imu_impact_stop'] is False


@pytest.mark.parametrize('flags', list(itertools.product(['false', 'true'], repeat=6)))
def test_impact_shares_exactly_one_stack(flags):
    names = ('steering_damping', 'adaptive_grip', 'fused_speed',
             'imu_understeer', 'imu_traction', 'imu_impact_stop')
    nodes, includes, ctx = _launch(**dict(zip(names, flags)), imu_hardware='false')
    # oracle: invariant: one shared include iff any consumer is enabled.
    assert _node_params(_gap_node(nodes), ctx)['enable_imu_impact_stop'] == (flags[-1] == 'true')
    assert len(includes) == int('true' in flags)
    for include in includes:
        assert include.launch_description_source.location.endswith('/vehicle_state_launch.py')
        assert dict(include.launch_arguments)['hardware'] == 'false'


def test_invalid_impact_flag_refused():
    # oracle: spec: malformed flags cannot start a driving launch.
    with pytest.raises(RuntimeError, match='imu_impact_stop'):
        _launch(imu_impact_stop='maybe')
