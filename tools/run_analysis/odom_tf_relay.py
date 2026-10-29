#!/usr/bin/env python3
"""Publish the odometry transform of a replayed bag, so a replayed slam_toolbox has its TF.

slam_replay.py plays /scan, /tf_static and one odometry topic, and never the recorded /tf, which
holds the run's own map -> odom corrections that the replay has to make afresh. This node turns
each odometry message into header.frame_id -> child_frame_id on /tf, at the pose in the message
and with the message's own stamp. Under use_sim_time the stamps follow the replay's /clock.

    odom_tf_relay.py --topic /odom/imu --ros-args -p use_sim_time:=true
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster


class OdomTfRelay(Node):
    def __init__(self, topic: str):
        super().__init__('odom_tf_relay')
        self._tf = TransformBroadcaster(self)
        self._skipped = 0
        # Best effort receives from a publisher whatever its reliability. The depth is a burst
        # allowance: a dropped odometry message is a gap in the TF that the replay's SLAM reads.
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=100)
        self.create_subscription(Odometry, topic, self._on_odom, qos)

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(v) for v in values) or not msg.header.frame_id or not msg.child_frame_id:
            self._skipped += 1
            if self._skipped == 1:
                self.get_logger().warning('skipping an odometry message with a non-finite pose or '
                                          'no frame names; further skips are not logged')
            return
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = values[:3]
        t.transform.rotation = pose.orientation
        self._tf.sendTransform(t)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--topic', required=True, help='the odometry topic to turn into TF, e.g. /odom/imu')
    args, ros_args = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=[sys.argv[0], *ros_args])
    node = OdomTfRelay(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A SIGINT has already shut the context down by the time spin returns, and shutting it
        # down twice raises.
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
