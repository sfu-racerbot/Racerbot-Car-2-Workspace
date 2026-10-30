"""Published-command checks on shipped YAML; run with ROS-sourced pytest."""
import math
from pathlib import Path

import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from gap_follow.gap_follow_node import GapFollowNode, UNDERSTEER_DEFAULTS
from test_gap_follow_corners_node import _corridor, _raycast
from test_gap_follow_steering_damping_node import (
    _capture, _capture_intent, _feed_imu, _feed_odom, _hold_deadman)

TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']
CONFIG = str(Path(__file__).parents[1] / 'config/gap_follow.yaml')
DT = 0.025


def _make(enabled=True, extra=()):
    rclpy.init(args=['--ros-args', '--params-file', CONFIG,
                     '-p', f'enable_imu_understeer:={str(enabled).lower()}',
                     '-p', 'command_slew_max_dt:=0.025',
                     '-p', 'intent_rate_hz:=0.0', *extra, *TEST_ONLY_DRIVE])
    node = GapFollowNode()
    assert node.enable_opponent_awareness
    assert node.drive_pub.topic_name == '/test_only/drive'
    return node


def _measured(node, angle=0.02):
    msg = JointState()
    msg.name, msg.position = ['steering'], [angle]
    node.measured_steering_callback(msg)


def _inputs(node, failure=None):
    _feed_odom(node, 2.0)
    if failure != 'never-imu':
        _feed_imu(node, yaw_rate=math.nan if failure == 'nan-imu' else 0.0,
                  state=(None if failure == 'no-status' else
                         failure if failure in ('waiting', 'fault') else 'ok'),
                  age_sec=0.2 if failure == 'stale-imu' else 0.0)
    if failure != 'never-measured':
        _measured(node, math.nan if failure == 'nan-measured' else 0.02)
        if failure == 'stale-measured':
            node.last_measured_steering_time = node.get_clock().now() - Duration(seconds=0.2)


def _run(enabled=True, failure=None, dropout=False, extra=()):
    node = _make(enabled, extra)
    try:
        commands, intents = _capture(node), _capture_intent(node)
        engaged = []
        scan = _raycast(_corridor(1.4, 0.6))
        for tick in range(60):
            _hold_deadman(node)
            _inputs(node, 'stale-imu' if dropout and tick == 59 else failure)
            node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
            # Ask for a fresh diagnostic each tick, independent of wall time.
            node._intent_throttle.reset()
            node.scan_callback(scan)
            engaged.append(node.understeer_check.engaged)
        return [(p.drive.speed, p.drive.steering_angle) for p in commands], intents, engaged
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_engaged_cap_is_min_of_feature_off_command_and_measured_speed_factor():
    on, intents, engaged = _run()
    off, _, _ = _run(False)
    # oracle: feature-off integration command and closed form 2*.85 m/s.
    assert engaged[-1]
    assert off[-1][0] > 2.0 * 0.85
    assert on[-1] == (min(off[-1][0], 2.0 * 0.85), off[-1][1])
    assert 'imu understeer cap 1.70m/s' in intents[-1]['reason']


@pytest.mark.parametrize('failure,reason', [
    ('waiting', 'imu waiting'), ('fault', 'imu fault'),
    ('stale-imu', 'imu stale'), ('never-imu', 'no imu'),
    ('nan-imu', 'imu yaw not finite'), ('no-status', 'no imu'),
    ('nan-measured', 'measured steering not finite'),
    ('stale-measured', 'measured steering stale'),
    ('never-measured', 'no measured steering'),
])
def test_untrusted_input_is_exactly_feature_off(failure, reason):
    on, intents, _ = _run(failure=failure)
    off, _, _ = _run(False, failure=failure)
    # oracle: invariant: identical inputs, no understeer contribution at all.
    assert on == off
    assert f'imu_understeer_off: {reason}' in intents[-1]['reason']


def test_imu_dropout_midcorner_restores_feature_off_on_that_tick():
    on, intents, engaged = _run(dropout=True)
    off, _, _ = _run(False, dropout=True)
    # oracle: invariant: engaged before dropout, no residual cap on dropout.
    assert engaged[-2]
    assert on[-2][0] < off[-2][0]
    assert on[-1] == off[-1]
    assert 'imu_understeer_off: imu stale' in intents[-1]['reason']


def _deny_case(state):
    node = _make()
    try:
        commands = _capture(node)
        # Warm the real check with judgeable, low-ratio inputs without sending
        # any Joy. This also exercises the never-received path honestly.
        for _ in range(40):
            _inputs(node)
            node._understeer_cap(0.1, DT)
        assert node.understeer_check.engaged
        if state != 'never':
            _hold_deadman(node, held=state != 'released')
        if state == 'stale':
            node.last_joy_time = node.get_clock().now() - Duration(
                seconds=node.joy_timeout_sec + 0.1)
        _inputs(node)
        node.scan_callback(_raycast(_corridor(1.4, 0.6)))
        return commands[-1].drive.speed
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_understeer_on_lb_held_drives():
    # oracle: spec: LB held is the permissive deadman path.
    assert _deny_case('held') > 0.0


def test_understeer_on_lb_released_refuses():
    # oracle: spec CLAUDE.md standing LB requirement, published command.
    assert _deny_case('released') == 0.0


def test_understeer_on_joy_never_received_refuses():
    # oracle: spec CLAUDE.md standing LB requirement, published command.
    assert _deny_case('never') == 0.0


def test_understeer_on_joy_stale_refuses():
    # oracle: spec CLAUDE.md standing LB requirement, published command.
    assert _deny_case('stale') == 0.0


def test_feature_off_has_no_measured_steering_subscription():
    node = _make(False)
    try:
        # oracle: spec: feature-off graph does not read measured steering.
        assert '/steering/measured' not in [sub.topic_name for sub in node.subscriptions]
    finally:
        node.destroy_node()
        rclpy.shutdown()


BAD_VALUES = [(name, 0.0) for name in UNDERSTEER_DEFAULTS] + [
    ('imu_understeer_engage_ratio', 0.85),
    ('imu_understeer_release_ratio', 1.51),
    ('imu_understeer_speed_factor', 1.0),
] + [(name, math.inf) for name in UNDERSTEER_DEFAULTS]


@pytest.mark.parametrize('name,value', BAD_VALUES)
def test_bad_live_parameter_is_refused_without_changing_the_value(name, value):
    node = _make()
    try:
        old = node.get_parameter(name).value
        result = node.set_parameters_atomically([Parameter(name, value=value)])
        # oracle: spec: invalid live parameters are refused, never applied.
        assert not result.successful
        assert name in result.reason
        assert node.get_parameter(name).value == old
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('name', list(UNDERSTEER_DEFAULTS))
def test_bad_startup_parameter_is_refused(name):
    rclpy.init(args=['--ros-args', '--params-file', CONFIG,
                     '-p', f'{name}:=0.0', *TEST_ONLY_DRIVE])
    try:
        # oracle: spec: reject invalid configuration before driving.
        with pytest.raises(ValueError, match=name):
            GapFollowNode()
    finally:
        rclpy.shutdown()


def test_valid_live_change_restarts_check_and_its_independent_lag():
    node = _make()
    try:
        result = node.set_parameters_atomically([
            Parameter('imu_understeer_steering_lag_sec', value=0.2)])
        # oracle: invariant: accepted parameter is what the control state uses.
        assert result.successful, result.reason
        assert node.understeer_steering_lag.lag_sec == 0.2
        assert node.adaptive_steering_lag.lag_sec == 0.15
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_speed_hold_does_not_remember_an_untrusted_understeer_cap():
    extra = ['-p', 'speed_hold_sec:=2.0']
    on, _, engaged = _run(dropout=True, extra=extra)
    off, _, _ = _run(False, dropout=True, extra=extra)
    # oracle: invariant: fallback is feature-off even with speed hold enabled.
    assert engaged[-2]
    assert on[-2][0] < off[-2][0]
    assert on[-1] == off[-1]
