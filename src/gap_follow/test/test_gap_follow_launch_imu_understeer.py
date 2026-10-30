"""Understeer launch wiring; pytest src/gap_follow/test/ with ROS sourced."""
import itertools

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from test_gap_follow_launch_steering_damping import (
    gl, _gap_node, _launch, _node_params)


def test_understeer_launch_default_is_off():
    defaults = {e.name: ''.join(s.perform(LaunchContext()) for s in e.default_value)
                for e in gl.generate_launch_description().entities
                if isinstance(e, DeclareLaunchArgument)}
    # oracle: spec I4: floor-trial feature ships off.
    assert defaults['imu_understeer'] == 'false'
    nodes, _, ctx = _launch()
    assert _node_params(_gap_node(nodes), ctx)['enable_imu_understeer'] is False


@pytest.mark.parametrize('damping,grip,fused', list(itertools.product(['false', 'true'], repeat=3)))
@pytest.mark.parametrize('understeer', ['false', 'true'])
def test_shared_imu_stack_is_included_exactly_once(damping, grip, fused, understeer):
    nodes, includes, ctx = _launch(steering_damping=damping, adaptive_grip=grip,
                                  fused_speed=fused, imu_understeer=understeer,
                                  imu_hardware='false')
    # oracle: integration invariant: OR of all feature flags owns one stack.
    assert _node_params(_gap_node(nodes), ctx)['enable_imu_understeer'] == (understeer == 'true')
    assert len(includes) == int('true' in (damping, grip, fused, understeer))
    if includes:
        assert includes[0].launch_description_source.location.endswith('/vehicle_state_launch.py')
        assert dict(includes[0].launch_arguments)['hardware'] == 'false'


def test_understeer_bad_launch_flag_names_the_argument():
    # oracle: spec: bad booleans are refused before starting a node.
    with pytest.raises(RuntimeError, match='imu_understeer'):
        _launch(imu_understeer='maybe')
