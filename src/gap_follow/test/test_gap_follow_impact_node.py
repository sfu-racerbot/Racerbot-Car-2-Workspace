"""I6 published commands on shipped YAML; ROS-sourced package pytest runner.

Mutation evidence (2026-10-29): forced-engaged _deadman_status fails all six
refusal cases (latched/unlatched). Ignored stops, validation, retuning, trust
gating and feature-off selection each fail the corresponding command tests.
Scripts and full failure output: /tmp/oc-part-b23-i6/ on the measurement host.
"""
from contextlib import contextmanager
import json
import math
from pathlib import Path

import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import String

from gap_follow.gap_follow_node import GapFollowNode
from test_gap_follow_steering_damping_node import _capture, _capture_intent, _feed_odom, _scan

CONFIG = str(Path(__file__).parents[1] / 'config/gap_follow.yaml')
TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']
DT = .025


@contextmanager
def _node(enabled=True, extra=()):
    rclpy.init(args=['--ros-args', '--params-file', CONFIG,
                     # Mid-race scene, not a start: the launch boost (on since
                     # 2026-10-30) skips the ramp and traction hold under test.
                     '-p', 'enable_launch_boost:=false',
                     '-p', f'enable_imu_impact_stop:={str(enabled).lower()}',
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


def _joy(n, pressed=True):
    m = Joy()
    m.buttons = [] if pressed is None else [0] * n.deadman_button + [int(pressed)]
    n.joy_callback(m)
    # Deliver to the separate subscription just as the ROS executor would.
    if n.enable_imu_impact_stop:
        n.impact_joy_callback(m)


def _imu(n, h=0.0, state='ok', yaw=0.0):
    n.imu_status_callback(String(data=json.dumps({'state': state})))
    msg = Imu()
    msg.linear_acceleration.x = float(h)
    msg.linear_acceleration.z = 9.80665
    msg.angular_velocity.z = yaw
    n.imu_callback(msg)


def _tick(n, tick=0, speed=1.0, scan=None):
    n.get_clock().set_ros_time_override(Time(nanoseconds=10_000_000_000 + tick*25_000_000))
    _feed_odom(n, speed)
    n.last_command_time = n.get_clock().now() - Duration(seconds=DT)
    n._intent_throttle.reset()
    n.scan_callback(_scan() if scan is None else scan)


def _command(msg):
    return msg.drive.steering_angle, msg.drive.speed


def test_feature_on_wires_imu_and_separate_lb_subscription():
    with _node(extra=['-p', 'enable_steering_damping:=false']) as n:
        # oracle: spec I6: impact alone owns one shared IMU input and its own
        # LB callback, alongside the unchanged deadman subscription.
        imu = [s for s in n.subscriptions if s.topic_name == '/imu/data']
        joy = [s for s in n.subscriptions if s.topic_name == '/joy']
        assert len(imu) == 1
        assert imu[0].callback == n.imu_callback
        assert len(joy) == 2
        assert {s.callback for s in joy} == {n.joy_callback, n.impact_joy_callback}


def test_spike_stops_for_five_seconds_and_only_release_press_resumes():
    with _node() as n:
        commands, intents = _capture(n), _capture_intent(n)
        _joy(n)
        _imu(n)
        _tick(n)
        assert commands[-1].drive.speed > 0
        rack = n.steering_basis = .12
        spike_time = n.get_clock().now().nanoseconds / 1e9
        h = n.imu_impact_threshold + 1
        _imu(n, h)
        for tick in range(202):
            _joy(n)
            _imu(n)
            _tick(n, tick)
            # oracle: spec I6: zero speed, rack held by _stop, no timed recovery.
            assert _command(commands[-1]) == (rack, 0.0)
            assert intents[-1]['state'] == 'imu_impact_stop'
            assert f'{h:.6f}' in intents[-1]['reason']
            assert f'{spike_time:.9f}s' in intents[-1]['reason']
            assert 'release LB and press it again to resume' in intents[-1]['reason']
        _joy(n, False)
        _tick(n, 202)
        assert _command(commands[-1]) == (rack, 0.0)
        assert intents[-1]['state'] == 'deadman_released'
        _joy(n, True)
        _tick(n, 203)
        assert commands[-1].drive.speed > 0


@pytest.mark.parametrize('failure', ['stale', 'fault', 'waiting', 'nonfinite'])
def test_imu_failure_never_lifts_latched_stop(failure):
    with _node() as n:
        commands, intents = _capture(n), _capture_intent(n)
        _joy(n)
        _imu(n, n.imu_impact_threshold + 1)
        if failure == 'stale':
            n.last_imu_time -= Duration(seconds=1)
        else:
            _imu(n, math.nan if failure == 'nonfinite' else 0,
                 state=failure if failure != 'nonfinite' else 'ok')
        _tick(n)
        # oracle: spec I6: IMU failure is never permission to resume a latched stop.
        assert _command(commands[-1]) == (0.0, 0.0)
        assert intents[-1]['state'] == 'imu_impact_stop'


def _untrusted(enabled, failure):
    with _node(enabled) as n:
        commands, intents = _capture(n), _capture_intent(n)
        _joy(n)
        if failure != 'never':
            # A newly received sample cannot itself be stale. Age a spike received
            # while waiting, then mark status ok without receiving another sample.
            _imu(n, math.nan if failure == 'nan' else n.imu_impact_threshold + 1,
                 state='waiting' if failure == 'stale' else
                 failure if failure in ('waiting', 'fault') else 'ok',
                 yaw=math.nan if failure == 'yaw' else 0.0)
            if failure == 'stale':
                n.imu_status_callback(String(data='{"state":"ok"}'))
                n.last_imu_time -= Duration(seconds=1)
        _tick(n)
        return _command(commands[-1]), intents[-1]['reason']


@pytest.mark.parametrize('failure,reason', [('waiting', 'imu waiting'), ('fault', 'imu fault'),
                                          ('stale', 'imu stale'), ('never', 'no imu'),
                                          ('nan', 'horizontal force not finite'),
                                          ('yaw', 'imu yaw not finite')])
def test_untrusted_spike_is_exact_feature_off_command(failure, reason):
    on, intent = _untrusted(True, failure)
    off, _ = _untrusted(False, failure)
    # oracle: invariant: untrusted detection contributes nothing to the command.
    assert on == off
    assert on[1] > 0
    assert f'imu_impact_stop_off: {reason}' in intent


@pytest.mark.parametrize('latched', [False, True])
@pytest.mark.parametrize('state', ['held', 'released', 'never', 'stale'])
def test_impact_deadman_paths(state, latched):
    with _node() as n:
        commands, intents = _capture(n), _capture_intent(n)
        _imu(n, n.imu_impact_threshold + 1 if latched else 0)
        if state != 'never':
            _joy(n, state != 'released')
        if state == 'stale':
            n.last_joy_time -= Duration(seconds=1)
        _tick(n)
        # oracle: spec CLAUDE.md: deadman first, including when impact-latched.
        if state == 'held' and not latched:
            assert commands[-1].drive.speed > 0
        else:
            assert _command(commands[-1]) == (0.0, 0.0)
        if state != 'held':
            assert intents[-1]['state'] == {
                'released': 'deadman_released', 'never': 'waiting_for_joy',
                'stale': 'joy_stale'}[state]
        elif latched:
            assert intents[-1]['state'] == 'imu_impact_stop'


def test_feature_off_adds_no_subscriptions_and_ignores_spike():
    with _node(False, ['-p', 'enable_steering_damping:=false']) as n:
        commands = _capture(n)
        # oracle: spec: no IMU/extra Joy subscription when no consumers are enabled.
        assert '/imu/data' not in [s.topic_name for s in n.subscriptions]
        assert [s.topic_name for s in n.subscriptions].count('/joy') == 1
        _joy(n)
        _tick(n)
        before = _command(commands[-1])
        _imu(n, n.imu_impact_threshold + 1)
        n.last_commanded_speed = 0.0
        _tick(n)
        assert _command(commands[-1]) == before
        assert before[1] > 0


@pytest.mark.parametrize('name,value', [('imu_impact_threshold', 0.0),
                                      ('imu_impact_threshold', -1.0),
                                      ('imu_impact_threshold', math.nan),
                                      ('imu_impact_threshold', math.inf),
                                      ('imu_impact_min_samples', 0),
                                      ('imu_impact_min_samples', -1)])
def test_bad_parameters_refused_startup_and_live(name, value):
    with _node() as n:
        previous = n.get_parameter(name).value
        result = n.set_parameters_atomically([Parameter(name, value=value)])
        # oracle: spec I6: invalid configurations cannot be applied, even live.
        assert not result.successful
        assert name in result.reason
        assert n.get_parameter(name).value == previous
    with pytest.raises(ValueError, match=name):
        with _node(extra=['-p', f'{name}:={value}']):
            pass


def test_live_retune_preserves_latch_and_release_edge():
    with _node() as n:
        commands = _capture(n)
        _imu(n, n.imu_impact_threshold + 1)
        _joy(n, False)
        result = n.set_parameters_atomically([
            Parameter('imu_impact_threshold', value=40.0),
            Parameter('imu_impact_min_samples', value=3)])
        # oracle: invariant: retuning changes detection, never clears an existing stop.
        assert result.successful
        _tick(n)
        assert commands[-1].drive.speed == 0
        _joy(n)
        _tick(n)
        assert commands[-1].drive.speed > 0
        for _ in range(2):
            _imu(n, 41)
            _tick(n)
            assert commands[-1].drive.speed > 0
        _imu(n, 41)
        _tick(n)
        assert commands[-1].drive.speed == 0


def test_missing_button_does_not_authorize_latch_reset():
    with _node() as n:
        commands = _capture(n)
        _imu(n, n.imu_impact_threshold + 1)
        _joy(n, None)
        _joy(n)
        _tick(n)
        # oracle: spec I6: a missing button is neither release nor press.
        assert commands[-1].drive.speed == 0
        _joy(n, False)
        _joy(n, None)
        _joy(n)
        _tick(n)
        assert commands[-1].drive.speed > 0


@pytest.mark.parametrize('failure', ['stale', 'fault', 'nan'])
def test_detection_requires_consecutive_trusted_samples(failure):
    with _node(extra=['-p', 'imu_impact_min_samples:=3']) as n:
        commands = _capture(n)
        _joy(n)
        h = n.imu_impact_threshold + 1
        for _ in range(2):
            _imu(n, h)
        if failure == 'stale':
            n.last_imu_time -= Duration(seconds=1)
        else:
            _imu(n, math.nan if failure == 'nan' else h,
                 state='fault' if failure == 'fault' else 'ok')
        for _ in range(2):
            _imu(n, h)
            _tick(n)
            # oracle: spec I6: an interrupted pair cannot complete the count of 3.
            assert commands[-1].drive.speed > 0
        _imu(n, h)
        _tick(n)
        assert commands[-1].drive.speed == 0


@pytest.mark.parametrize('latched', [False, True])
@pytest.mark.parametrize('state', ['released', 'never', 'stale'])
def test_deadman_refusal_command_without_intent_oracle(state, latched):
    """Command-only evidence: the two-gate mutation cannot hide behind intent."""
    with _node() as n:
        commands = _capture(n)
        _imu(n, n.imu_impact_threshold + 1 if latched else 0)
        if state != 'never':
            _joy(n, state != 'released')
        if state == 'stale':
            n.last_joy_time -= Duration(seconds=n.joy_timeout_sec + 0.1)
        _tick(n)
        # oracle: spec CLAUDE.md: every deadman refusal publishes zero speed,
        # irrespective of latch state. Assert the actual drive message only.
        assert commands[-1].drive.speed == 0.0


def _diagnostic_failure_commands(held, broken=None):
    with _node() as n:
        commands, intents = _capture(n), _capture_intent(n)
        gate = n._imu_gate
        publish = n._publish_drive
        after_drive = False
        failures = []

        def publish_then_mark(*args, **kwargs):
            nonlocal after_drive
            publish(*args, **kwargs)
            after_drive = True

        def raising_gate():
            if after_drive:
                failures.append(len(commands))
                raise RuntimeError('injected post-drive IMU gate failure')
            return gate()

        def raising_reason():
            failures.append(len(commands))
            raise RuntimeError('injected impact reason failure')

        n._publish_drive = publish_then_mark
        if broken == 'gate':
            n._imu_gate = raising_gate
        elif broken == 'reason':
            n._impact_detection_reason = raising_reason
        limit = n._intent_failures.max_failures
        for tick in range(limit + 2):
            after_drive = False
            _joy(n, held)
            _imu(n)
            _tick(n, tick)
        return ([_command(c) for c in commands], n._intent_failures.disabled,
                failures, len(intents), limit)


@pytest.mark.parametrize('held', [False, True])
@pytest.mark.parametrize('broken', ['gate', 'reason'])
def test_impact_diagnostic_exception_never_changes_commands_or_next_tick(held, broken):
    baseline, baseline_disabled, _, baseline_intents, limit = (
        _diagnostic_failure_commands(held))
    actual, disabled, failures, intents, _ = _diagnostic_failure_commands(held, broken)
    # oracle: spec docs/drive-intent.md: the entire diagnostic is post-publish,
    # failure-contained and disabled after repeated failures, without exiting.
    assert actual == baseline
    assert len(actual) == limit + 2
    assert baseline_intents == limit + 2
    assert not baseline_disabled
    assert disabled
    assert intents == 0
    assert failures == list(range(1, limit + 1))
    if held:
        assert all(speed > 0 for _, speed in actual)
    else:
        assert actual == [(0.0, 0.0)] * (limit + 2)
