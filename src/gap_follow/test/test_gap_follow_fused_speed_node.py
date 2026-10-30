"""Real-node fused-speed commands; sourced pytest and colcon test gap_follow.

All nodes load the shipped YAML, including opponent awareness. Only the speed
topic changes. Drive messages go to /test_only/drive, never the real /drive.
"""
from contextlib import contextmanager
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

from gap_follow.gap_follow_node import GapFollowNode
from test_gap_follow_steering_damping_node import (
    _blocked_right, _capture, _feed_imu, _feed_odom as _feed_primary_odom,
    _hold_deadman, _scan)

CONFIG = str(Path(get_package_share_directory('gap_follow')) / 'config/gap_follow.yaml')
TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']
FUSED = ['-p', 'odom_topic:=/odom/fused']
SPEED = 1.0  # m/s: synthetic wheel speed; well below the open scene's caps.


def _feed_odom(node, speed):
    # R1 requires fresh raw wheels beside fused odometry, as on the car.
    # These scenes compare identical wheel/fused speeds, not estimator error.
    _feed_primary_odom(node, speed)
    if node.using_fused_speed:
        wheel = Odometry()
        wheel.twist.twist.linear.x = float(speed)
        node.wheel_odom_callback(wheel)


@contextmanager
def _node(extra):
    rclpy.init(args=['--ros-args', '--params-file', CONFIG, *extra, *TEST_ONLY_DRIVE])
    n = GapFollowNode()
    _feed_imu(n, yaw_rate=0.0, state='ok')
    try:
        # oracle: spec -- shipped gap_follow.yaml enables opponent awareness.
        assert n.get_parameter('enable_opponent_awareness').value is True
        # oracle: invariant -- every test publisher is isolated from the car.
        assert n.drive_pub.topic_name == '/test_only/drive'
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def node():
    with _node(FUSED) as n:
        yield n


def test_fused_speed_subscribes_to_both_sources_and_default_to_wheels_once():
    with _node(FUSED) as node:
        topics = {s.topic_name for s in node.subscriptions}
        # oracle: spec -- I5 controller ruling: raw wheel fallback needs both.
        assert '/odom/fused' in topics
        assert '/odom' in topics
    with _node([]) as default:
        default_topics = [s.topic_name for s in default.subscriptions]
        # oracle: invariant: the ordinary wheel path adds no subscriptions.
        assert default_topics.count('/odom') == 1
        assert '/odom/fused' not in default_topics


def test_fused_speed_drives_with_lb_held(node):
    published = _capture(node)
    _feed_odom(node, SPEED)
    _hold_deadman(node)
    node.scan_callback(_scan())
    # oracle: invariant -- fresh joy + odometry and an open scan permit driving.
    assert published[-1].drive.speed > 0.0


def test_fused_speed_refuses_lb_released(node):
    published = _capture(node)
    _feed_odom(node, SPEED)
    _hold_deadman(node)
    _hold_deadman(node, held=False)
    node.scan_callback(_scan())
    # oracle: spec -- CLAUDE.md's mandatory LB deadman, even with fused speed.
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'deadman_released'


def test_fused_speed_refuses_no_joy(node):
    published = _capture(node)
    _feed_odom(node, SPEED)
    node.scan_callback(_scan())
    # oracle: spec -- CLAUDE.md: no joy means LB cannot be verified.
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'waiting_for_joy'


def test_fused_speed_refuses_stale_joy(node):
    published = _capture(node)
    _feed_odom(node, SPEED)
    _hold_deadman(node)
    node.last_joy_time = node.get_clock().now() - Duration(seconds=node.joy_timeout_sec + 0.1)
    node.scan_callback(_scan())
    # oracle: spec -- CLAUDE.md: LB must have a live joy stream.
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'joy_stale'


def _commands(extra, speed):
    with _node(extra) as n:
        # Inject the clock boundary so both runs get identical 40 Hz intervals,
        # including the acceleration and steering slew limits (no wall-time noise).
        clock = n.get_clock()
        clock._set_ros_time_is_active(True)
        published = _capture(n)
        for k in range(20):
            clock.set_ros_time_override(Time(nanoseconds=10_000_000_000 + k * 25_000_000))
            _hold_deadman(n)
            _feed_imu(n, yaw_rate=0.0, state='ok')
            _feed_odom(n, speed)
            ranges = _blocked_right() if k < 10 else list(reversed(_blocked_right()))
            n.scan_callback(_scan(ranges))
        return [(p.drive.speed, p.drive.steering_angle) for p in published]


def test_fused_wheel_speed_publishes_the_feature_off_commands():
    off = _commands([], SPEED)
    on = _commands(FUSED, SPEED)
    # oracle: invariant -- same speed, joy and 20 scans must give the feature-off
    # node's commands. Awareness remains ON in both: only fused_speed is off.
    assert len(on) == len(off) == 20
    assert any(speed > 0.0 for speed, _ in off)
    for actual, expected in zip(on, off):
        assert actual == pytest.approx(expected, rel=0.0, abs=1e-9)  # m/s, rad


def test_silent_fused_odometry_stops_after_the_stale_limit(node):
    published = _capture(node)
    _hold_deadman(node)
    _feed_odom(node, SPEED)
    node.scan_callback(_scan())
    # oracle: invariant -- demonstrate driving before the stream stops.
    assert published[-1].drive.speed > 0.0
    node.last_odom_time = node.get_clock().now() - Duration(seconds=node.odom_timeout_sec + 0.1)
    _hold_deadman(node)
    node.scan_callback(_scan())
    # oracle: spec -- imu_bridge/README.md: vehicle_state death triggers the
    # consumer's stale-odometry stop (same as test_gap_follow_node.py).
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'odometry_stale'
