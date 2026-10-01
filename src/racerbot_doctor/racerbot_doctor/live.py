"""Sample the live ROS graph into a LiveSample. The only module importing rclpy.

Subscribe-only: this creates no publisher, no service client, and sets no
parameter (test_cli.py's static check enforces that for the whole package).
It runs on whatever ROS_DOMAIN_ID the shell has, because inspecting the
car's graph is the point.

Topic names are parameters so the test can sample /test_only/* names and
never put a /teleop, /scan or latched /tf_static on any graph.
"""
import os
import time

from . import car
from .snapshot import LiveSample, TopicStats

DEFAULT_TOPICS = {
    'core': '/sensors/core',
    'scan': '/scan',
    'odom': '/odom',
    'joy': '/joy',
    'camera': car.CAMERA_TOPIC,
    'tf_static': '/tf_static',
    'teleop': '/teleop',
}


def sample(window_s, *, discovery_s=1.0, topics=None):
    """Count messages per topic over window_s seconds (after discovery_s of
    discovery, which is not counted), keep the last message of the topics
    whose content is checked, read base_link->laser off /tf_static, and
    count /teleop publishers. Results are keyed by the *default* topic
    name, so checks.py never sees the test's remapped names.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, Joy, LaserScan
    from tf2_msgs.msg import TFMessage
    from vesc_msgs.msg import VescStateStamped

    names = dict(DEFAULT_TOPICS, **(topics or {}))
    # (key, msg type, keep the deserialized last message?)
    wanted = [('core', VescStateStamped, True), ('scan', LaserScan, True),
              ('odom', Odometry, False), ('joy', Joy, False),
              ('camera', Image, False)]

    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = None
    try:
        node = rclpy.create_node(f'racerbot_doctor_{os.getpid()}', context=ctx)
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        stats = {key: TopicStats() for key, _, _ in wanted}
        laser_tf = []

        def counter(key, keep):
            def cb(msg):
                stats[key].count += 1
                if keep:
                    stats[key].last = msg
            return cb

        for key, msg_type, keep in wanted:
            # Best-effort subscriber: matches reliable and best-effort
            # publishers alike. Count-only topics stay serialized (raw=True)
            # so sampling a camera costs no deserialization.
            node.create_subscription(msg_type, names[key], counter(key, keep),
                                     qos_profile_sensor_data, raw=not keep)

        def on_tf(msg):
            for t in msg.transforms:
                if (t.header.frame_id.lstrip('/') == 'base_link'
                        and t.child_frame_id.lstrip('/') == 'laser'):
                    v = t.transform.translation
                    laser_tf[:] = [(v.x, v.y, v.z)]
        node.create_subscription(TFMessage, names['tf_static'], on_tf, QoSProfile(
            depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE))

        def spin_until(deadline):
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=max(0.0, min(
                    0.05, deadline - time.monotonic())))

        spin_until(time.monotonic() + discovery_s)
        for st in stats.values():
            st.count = 0
        start = time.monotonic()
        spin_until(start + window_s)
        elapsed = time.monotonic() - start
        for st in stats.values():
            st.window_s = elapsed

        return LiveSample(
            topics={DEFAULT_TOPICS[key]: st for key, st in stats.items()},
            teleop_publishers=node.count_publishers(names['teleop']),
            laser_tf=laser_tf[0] if laser_tf else None)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown(context=ctx)
