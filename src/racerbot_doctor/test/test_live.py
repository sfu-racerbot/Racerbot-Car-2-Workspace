"""
Tests for live.py -- the only rclpy code in racerbot_doctor.

Needs ROS sourced and an isolated domain (CLAUDE.md):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/racerbot_doctor/test/ -v

Belt and braces: the publishers here only ever use /test_only/* names, so
even on the wrong domain they cannot reach /teleop, /scan or /tf_static.
Unsourced, this file fails to collect -- loudly, on purpose; do not reach
for --continue-on-collection-errors.

Oracle: the helper publishes at a known rate (timer period) for a known
window; the sampler must report that rate within one timer period's worth
of jitter either side, plus exact values for the TF and publisher count.
"""
import os
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_msgs.msg import TFMessage

from racerbot_doctor.live import sample

TOPICS = {'core': '/test_only/sensors/core', 'scan': '/test_only/scan',
          'odom': '/test_only/odom', 'joy': '/test_only/joy',
          'camera': '/test_only/camera', 'tf_static': '/test_only/tf_static',
          'teleop': '/test_only/teleop'}


def test_runs_on_the_isolated_test_domain():
    assert os.environ.get('ROS_DOMAIN_ID') == '79', \
        'export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST'


@pytest.fixture
def fake_car():
    """A helper node, on its own context and thread, publishing /test_only/scan
    at 20 Hz plus a latched base_link->laser transform."""
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node('doctor_test_fake_car', context=ctx)
    scan_pub = node.create_publisher(LaserScan, TOPICS['scan'], 10)
    tf_pub = node.create_publisher(TFMessage, TOPICS['tf_static'], QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE))
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = 'base_link', 'laser'
    t.transform.translation.x, t.transform.translation.z = 0.26, 0.11
    t.transform.rotation.w = 1.0
    # A second static transform from the same parent, published after the
    # laser one: the sampler must pick by child frame, not take the last.
    decoy = TransformStamped()
    decoy.header.frame_id, decoy.child_frame_id = 'base_link', 'camera_link'
    decoy.transform.translation.x = 9.0
    decoy.transform.rotation.w = 1.0
    tf_pub.publish(TFMessage(transforms=[t, decoy]))

    def tick():
        msg = LaserScan()
        msg.ranges = [1.0, 2.0, float('inf')]
        scan_pub.publish(msg)
    node.create_timer(0.05, tick)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.02)
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    yield node, ctx
    stop.set()
    thread.join(timeout=2)
    node.destroy_node()
    rclpy.shutdown(context=ctx)


def test_counts_rate_keeps_last_scan_and_reads_static_tf(fake_car):
    live = sample(2.0, discovery_s=1.5, topics=TOPICS)
    scan = live.topics['/scan']
    assert 2.0 <= scan.window_s < 2.5
    assert 15.0 <= scan.hz <= 25.0          # 20 Hz timer, +/- 5 Hz jitter
    assert list(scan.last.ranges)[:2] == [1.0, 2.0]
    assert live.topics['/sensors/core'].count == 0
    assert live.topics['/sensors/core'].last is None
    assert live.laser_tf == pytest.approx((0.26, 0.0, 0.11), abs=1e-9)  # exact copy
    assert live.teleop_publishers == 0


def test_results_keyed_by_real_topic_names_whatever_was_sampled(fake_car):
    live = sample(0.3, discovery_s=0.5, topics=TOPICS)
    assert sorted(live.topics) == sorted(
        ['/sensors/core', '/scan', '/odom', '/joy',
         '/camera/camera/color/image_raw'])


def test_counts_teleop_publishers(fake_car):
    node, _ = fake_car
    pub = node.create_publisher(AckermannDriveStamped, TOPICS['teleop'], 10)
    try:
        time.sleep(0.2)
        assert sample(0.2, discovery_s=1.0, topics=TOPICS).teleop_publishers == 1
    finally:
        node.destroy_publisher(pub)


def test_nothing_on_the_graph_is_empty_not_an_error():
    live = sample(0.3, discovery_s=0.3, topics=TOPICS)
    assert all(st.count == 0 for st in live.topics.values())
    assert live.laser_tf is None
