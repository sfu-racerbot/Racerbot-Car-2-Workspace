"""R2 published-command invariant; ROS-sourced package pytest runner."""
import pytest

from test_gap_follow_traction_node import _node, _odom, _tick, DT
from test_gap_follow_steering_damping_node import (
    _capture, _feed_imu, _feed_odom, _hold_deadman)


def _bounded_command(enabled, fused, fused_source=False, imu_all=False, holding=False):
    # Feature set from gap_follow_launch.py imu_all: adaptive grip stays off.
    extra = ['-p', 'odom_topic:=/odom/fused'] if fused_source or imu_all else []
    if imu_all:
        extra += ['-p', 'enable_imu_understeer:=true',
                  '-p', 'enable_imu_impact_stop:=true',
                  '-p', 'enable_steering_damping:=true']
    with _node(enabled, extra) as n:
        commands = _capture(n)
        _feed_imu(n, yaw_rate=0.0)
        _feed_odom(n, fused if fused_source or imu_all else 2.0)
        _odom(n.wheel_odom_callback, 2.0)
        _odom(n.fused_odom_callback, fused)
        _hold_deadman(n)
        n.last_commanded_speed = 1.5
        if holding:
            n.traction_hold.update(2.0, 1.7, DT)
        _tick(n)
        return commands[-1].drive.speed, n.traction_hold.holding


@pytest.mark.parametrize('fused', [1.7, 1.8, 2.2])
@pytest.mark.parametrize('holding', [False, True])
@pytest.mark.parametrize('mode', ['wheel', 'fused', 'imu_all'])
def test_traction_never_raises_the_same_tick_feature_off_command(fused, holding, mode):
    args = dict(fused=fused, fused_source=mode == 'fused',
                imu_all=mode == 'imu_all', holding=holding)
    on, active = _bounded_command(True, **args)
    off, _ = _bounded_command(False, **args)
    # oracle: invariant R2: same inputs and fused setting, traction only lowers.
    assert on <= off
    if active:
        assert on == min(off, 1.5)


def test_review_negative_slip_case_equals_wheel_based_feature_off_ramp():
    on, active = _bounded_command(True, 2.2)
    off, _ = _bounded_command(False, 2.2)
    # oracle: closed form: wheel basis max(last=1.5,wheel=2), a=5 m/s².
    assert on == off == max(1.5, 2.0) + 5.0 * DT
    assert not active
