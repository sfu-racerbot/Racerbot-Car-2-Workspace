"""Task J1b launch wiring; run with ROS-sourced pytest or colcon test gap_follow."""
import pytest
import yaml

from test_gap_follow_launch_fused_speed import SRC, _launch


FEATURES = ('fused_speed', 'imu_understeer', 'imu_traction', 'imu_impact_stop')


@pytest.mark.parametrize('all_value', ['false', 'true'])
@pytest.mark.parametrize('explicit_off', [None, *FEATURES])
def test_imu_all_feature_overrides_and_adaptive_grip(all_value, explicit_off):
    # oracle: Task J1b R1/R2: explicit false wins; adaptive grip is excluded.
    overrides = {'imu_all': all_value, 'imu_hardware': 'false'}
    if explicit_off:
        overrides[explicit_off] = 'false'
    params, includes, _ = _launch(**overrides)
    for flag in FEATURES[1:]:
        assert params['enable_' + flag] is (all_value == 'true' and flag != explicit_off)
    wheel = yaml.safe_load((SRC / 'gap_follow/config/gap_follow.yaml').read_text())
    state = yaml.safe_load((SRC / 'imu_bridge/config/vehicle_state.yaml').read_text())
    expected_odom = (state['vehicle_state_node']['ros__parameters']['fused_odom_topic']
                     if all_value == 'true' and explicit_off != 'fused_speed'
                     else wheel['gap_follow_node']['ros__parameters']['odom_topic'])
    assert params['odom_topic'] == expected_odom
    assert params['enable_adaptive_grip'] is False
    # Car 2: steering damping defaults off (docs/car1-vs-car2.md#imu), so the
    # stack starts once exactly when imu_all leaves at least one feature on;
    # car 1's damping default made it start in every case.
    assert len(includes) == (1 if all_value == 'true' else 0)


def test_imu_false_disables_default_damping_and_excludes_stack():
    # oracle: Task J1b R1/R3: imu off defaults damping off and starts no stack.
    params, includes, _ = _launch(imu='false')
    assert params['enable_steering_damping'] is False
    assert includes == []


@pytest.mark.parametrize('flag', ['steering_damping', 'adaptive_grip', *FEATURES])
def test_enabled_feature_without_imu_refuses_with_fix(flag):
    # oracle: Task J1b R3: each enabled consumer needs imu:=true.
    with pytest.raises(RuntimeError, match=flag + r'.*imu:=true'):
        _launch(imu='false', **{flag: 'true'})


def test_imu_all_without_imu_refuses_with_fix():
    # oracle: Task J1b R3: identify imu_all when it enables consumers.
    with pytest.raises(RuntimeError, match=r'imu_all.*imu:=true'):
        _launch(imu='false', imu_all='true')


@pytest.mark.parametrize('flag', ['imu', 'imu_all'])
@pytest.mark.parametrize('value', ['maybe', '', 'yes', '2'])
def test_imu_switches_reject_invalid_booleans(flag, value):
    # oracle: Task J1b R3: _flag accepts only true/false/1/0.
    with pytest.raises(RuntimeError, match=flag + r'.*expected'):
        _launch(**{flag: value})
