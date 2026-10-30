"""I5 published-command contracts; run with ROS-sourced package pytest."""
from contextlib import contextmanager
import math
from pathlib import Path

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time

from gap_follow.gap_follow_node import GapFollowNode, TRACTION_DEFAULTS
from test_gap_follow_steering_damping_node import (
    _capture, _capture_intent, _feed_imu, _feed_odom, _hold_deadman, _scan)

CONFIG = str(Path(__file__).parents[1] / 'config/gap_follow.yaml')
TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']
DT = 0.025


@contextmanager
def _node(enabled=True, extra=()):
    rclpy.init(args=['--ros-args', '--params-file', CONFIG,
                     # Mid-race scene, not a start: the launch boost (on since
                     # 2026-10-30) skips the ramp and traction hold under test.
                     '-p', 'enable_launch_boost:=false',
                     '-p', f'enable_imu_traction:={str(enabled).lower()}',
                     '-p', 'intent_rate_hz:=0.0', *extra, *TEST_ONLY_DRIVE])
    n = None
    try:
        n = GapFollowNode()
        assert n.enable_opponent_awareness
        assert n.drive_pub.topic_name == '/test_only/drive'
        n.get_clock()._set_ros_time_is_active(True)
        n.get_clock().set_ros_time_override(Time(seconds=10))
        yield n
    finally:
        if n is not None:
            n.destroy_node()
        rclpy.shutdown()


def _odom(callback, speed):
    msg = Odometry()
    msg.twist.twist.linear.x = float(speed)
    callback(msg)


def _inputs(n, wheel=2.0, fused=1.7, failure=None):
    _feed_odom(n, wheel)
    if failure != 'never-imu':
        _feed_imu(n, yaw_rate=math.nan if failure == 'nan-imu' else 0.0,
                  state=failure if failure in ('waiting', 'fault') else 'ok',
                  age_sec=0.2 if failure == 'stale-imu' else 0.0)
    if failure != 'never-fused':
        _odom(n.fused_odom_callback, math.nan if failure == 'nan-fused' else
              math.inf if failure == 'inf-fused' else fused)
        if failure == 'stale-fused':
            n.last_fused_time -= Duration(seconds=0.3)


def _tick(n, tick=0):
    n.last_command_time = n.get_clock().now() - Duration(seconds=DT)
    n._intent_throttle.reset()
    n.scan_callback(_scan())
    n.get_clock().set_ros_time_override(Time(nanoseconds=10_000_000_000 +
                                            (tick + 1) * 25_000_000))


def _one(enabled=True, wheel=2.0, fused=1.7, last=1.5, failure=None):
    with _node(enabled) as n:
        commands, intents = _capture(n), _capture_intent(n)
        _inputs(n, wheel, fused, failure)
        _hold_deadman(n)
        n.last_commanded_speed = last
        _tick(n)
        return commands[-1].drive.speed, intents[-1]['reason']


def test_hold_publishes_min_of_desired_and_last_command():
    on, reason = _one()
    off, _ = _one(False)
    # oracle: spec I5 ruling: hold bypasses ramp; other ceilings still apply.
    assert off > 1.5
    assert on == min(off, 1.5)
    assert 'imu traction hold (slip 0.30m/s)' in reason


def test_not_holding_ramp_uses_fused_speed_not_wheels():
    on, _ = _one(fused=1.8)
    off, _ = _one(False, fused=1.8)
    # oracle: closed form, shipped max_acceleration=5 m/s², 25 ms tick.
    assert on == 1.8 + 5.0 * DT
    assert off == 2.0 + 5.0 * DT


def test_hold_applies_a_lower_speed_ceiling_immediately():
    with _node(extra=['-p', 'max_speed:=1.5', '-p', 'corner_speed:=1.5',
                      '-p', 'corner_speed_wide:=1.5']) as n:
        commands = _capture(n)
        _inputs(n)
        _hold_deadman(n)
        n.last_commanded_speed = 2.0
        _tick(n)
        # oracle: closed form: min(desired=1.5,last=2), not a brake ramp.
        assert commands[-1].drive.speed == 1.5


@pytest.mark.parametrize('wheel,fused', [(0.0, 0.0), (0.2, 0.0), (0.29, 0.0),
                                         (-1.0, -1.3)])
def test_standstill_and_reverse_accelerate_exactly_as_feature_off(wheel, fused):
    on, reason = _one(wheel=wheel, fused=fused, last=0.0)
    off, _ = _one(False, wheel=wheel, fused=fused, last=0.0)
    # oracle: spec I5: no judgement at either speed below the floor.
    assert on == off
    assert on > 0
    assert 'imu traction hold' not in reason


def test_continuous_slip_releases_at_max_hold_and_stays_released_for_two_seconds():
    with _node() as n:
        commands = _capture(n)
        n.last_commanded_speed = 1.5
        for tick in range(80):
            _inputs(n)
            _hold_deadman(n)
            _tick(n, tick)
        speeds = [c.drive.speed for c in commands]
        # oracle: closed form: 0.5 / .025 = 20 ticks; expiry tick is not held.
        assert speeds[:19] == [1.5] * 19
        assert speeds[19] == 1.7 + n.max_acceleration * DT
        assert all(b >= a for a, b in zip(speeds[19:], speeds[20:]))
        assert speeds[-1] > 1.5


@pytest.mark.parametrize('failure,reason', [
    ('waiting', 'imu waiting'), ('fault', 'imu fault'),
    ('stale-imu', 'imu stale'), ('never-imu', 'no imu'),
    ('nan-imu', 'imu yaw not finite'), ('never-fused', 'no fused odometry'),
    ('stale-fused', 'fused odometry stale'), ('nan-fused', 'fused speed not finite'),
    ('inf-fused', 'fused speed not finite')])
def test_untrusted_input_publishes_exact_feature_off_command(failure, reason):
    on, intent = _one(failure=failure)
    off, _ = _one(False, failure=failure)
    # oracle: invariant: no IMU contribution on any untrusted tick.
    assert on == off
    assert f'imu_traction_off: {reason}' in intent


def test_dropout_while_holding_restores_feature_off_on_next_tick():
    feature_off, _ = _one(False, failure='fault')
    with _node() as n:
        commands, intents = _capture(n), _capture_intent(n)
        n.last_commanded_speed = 1.5
        _inputs(n)
        _hold_deadman(n)
        _tick(n)
        assert commands[-1].drive.speed == 1.5
        _inputs(n, failure='fault')
        _tick(n, 1)
        expected = 2.0 + 5.0 * DT
        # oracle: closed form: dropout restores wheel basis on that same tick.
        assert commands[-1].drive.speed == feature_off == expected
        assert 'imu_traction_off: imu fault' in intents[-1]['reason']


def _deny(state):
    with _node() as n:
        commands = _capture(n)
        _inputs(n)
        n.last_commanded_speed = 1.5
        if state != 'never':
            _hold_deadman(n, held=state not in ('released', 'missing-button'))
        if state == 'stale':
            n.last_joy_time -= Duration(seconds=n.joy_timeout_sec + 0.1)
        if state == 'missing-button':
            from sensor_msgs.msg import Joy
            n.joy_callback(Joy())
        _tick(n)
        return commands[-1].drive.speed


@pytest.mark.parametrize('state', ['released', 'never', 'stale', 'missing-button'])
def test_traction_deadman_deny_publishes_zero(state):
    # oracle: spec CLAUDE.md: all four deny paths refuse published motion.
    assert _deny(state) == 0.0


def test_traction_deadman_held_drives():
    # oracle: invariant: the deny scene actually permits motion with live LB.
    assert _deny('held') == 1.5


@pytest.mark.parametrize('name', list(TRACTION_DEFAULTS))
@pytest.mark.parametrize('value', [0.0, -1.0, math.nan, math.inf])
def test_invalid_live_traction_parameter_is_refused(name, value):
    with _node() as n:
        old = n.get_parameter(name).value
        result = n.set_parameters_atomically([Parameter(name, value=value)])
        # oracle: spec: finite positive parameters; rejection is atomic.
        assert not result.successful
        assert name in result.reason
        assert n.get_parameter(name).value == old


@pytest.mark.parametrize('name', list(TRACTION_DEFAULTS))
def test_invalid_startup_traction_parameter_is_refused(name):
    # oracle: spec: reject invalid values before driving, even feature off.
    with pytest.raises(ValueError, match=name):
        with _node(extra=['-p', f'{name}:=0.0']):
            pass


def test_live_threshold_pair_is_validated_and_valid_change_restarts_hold():
    with _node() as n:
        bad = n.set_parameters_atomically([
            Parameter('imu_traction_release_slip', value=0.246)])
        # oracle: spec: release must be strictly below engagement threshold.
        assert not bad.successful
        n.traction_hold.update(2.0, 1.7, DT)
        good = n.set_parameters_atomically([
            Parameter('imu_traction_max_hold_sec', value=0.6)])
        assert good.successful
        assert n.traction_hold.max_hold_sec == 0.6
        assert not n.traction_hold.holding


def test_feature_off_adds_no_fused_subscription_or_traction_intent():
    with _node(False) as n:
        # oracle: spec: feature-off graph leaves fused odometry unused.
        assert '/odom/fused' not in [s.topic_name for s in n.subscriptions]
    _, reason = _one(False)
    assert 'imu traction' not in reason
    assert 'imu_traction_off' not in reason
