"""seed_start_pose: tell the particle filter where the car is, with no RViz.

A mapping run (``map_course``) saves where the car stopped at the end, in
the map frame, as start_pose.yaml -- on the racing line it just recorded,
measured by SLAM while it was still running. A car left there (or put back
on that spot, facing the same way) is at that pose; race_launch.py passes it
here as x, y, yaw, and this node hands the filter exactly that, as the "2D
Pose Estimate" RViz would have sent. Default (0, 0, 0): the map origin.

The filter spreads its particles 0.5 m and 0.4 rad (about 23 deg) around the
seed (particle_filter.initialize_particles_pose), so "the same spot" means
within a few tens of centimetres and degrees, not millimetres.

Publishes only ``/initialpose``. It cannot move the car and is not driving
code (docs/adding-your-own-code.md). It waits until the filter is listening,
sends the seed ``repeats`` times ``period_sec`` apart (one message can be
lost while the filter is still loading its map), then waits for the filter
to report a pose within ``confirm_radius`` of the seed, and exits 0. Exit 1
means it never did -- no filter, or a filter that ignored the seed.

race_launch.py starts pure_pursuit only after this exits 0. Before that the
filter publishes its own unseeded guess, and in simulation that guess
jumped 6 m; a controller already driving on it steered into a wall.

    ros2 run pure_pursuit seed_start_pose
    ros2 run pure_pursuit seed_start_pose --ros-args -p x:=1.0 -p yaw:=3.14
"""

import math

import rclpy
import sys

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node

from pure_pursuit import racing_math


def seed_message_fields(x, y, yaw, laser_offset_x, laser_offset_y):
    """(x, y, qz, qw) of the filter's seed for base_link pose (x, y, yaw).

    The filter's particles are LiDAR poses, not base_link poses -- the same
    conversion auto_map_race_node._seed_particle_filter makes.
    """
    lx, ly, lyaw = racing_math.base_link_pose_to_laser(
        x, y, yaw, laser_offset_x, laser_offset_y)
    return lx, ly, math.sin(lyaw / 2.0), math.cos(lyaw / 2.0)


class SeedStartPose(Node):
    def __init__(self):
        super().__init__('seed_start_pose')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        # Keep in step with gap_follow.yaml / the bringup laser transform.
        self.declare_parameter('laser_offset_x', 0.26)
        self.declare_parameter('laser_offset_y', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('repeats', 3)
        self.declare_parameter('period_sec', 0.5)
        self.declare_parameter('give_up_sec', 60.0)
        self.declare_parameter('pose_topic', '/pf/viz/inferred_pose')
        self.declare_parameter('confirm_radius', 1.0)

        def value(name):
            return self.get_parameter(name).value

        self.fields = seed_message_fields(
            float(value('x')), float(value('y')), float(value('yaw')),
            float(value('laser_offset_x')), float(value('laser_offset_y')))
        self.map_frame = value('map_frame')
        self.remaining = int(value('repeats'))
        self.give_up_sec = float(value('give_up_sec'))
        self.started = self.get_clock().now()
        self.done = False
        self.succeeded = False
        self.seeded = False
        self.confirm_radius = float(value('confirm_radius'))
        self.create_subscription(PoseStamped, value('pose_topic'), self._pose, 10)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, value('initialpose_topic'), 1)
        self.timer = self.create_timer(float(value('period_sec')), self._tick)
        self.get_logger().info(
            f"Waiting for the particle filter to listen on "
            f"'{value('initialpose_topic')}', then seeding it at the mapping "
            f"start pose. The car must be where mapping left it, facing the "
            f"same way.")

    def _pose(self, msg):
        """Done once the filter's pose is near the seed: it has taken it."""
        if not self.seeded or self.done:
            return
        x, y, _qz, _qw = self.fields
        if math.hypot(msg.pose.position.x - x, msg.pose.position.y - y) <= self.confirm_radius:
            self.get_logger().info('Particle filter is at the start pose; racing can begin.')
            self.succeeded = True
            self.done = True

    def _tick(self):
        if self.done:
            return
        waited = (self.get_clock().now() - self.started).nanoseconds / 1e9
        if waited > self.give_up_sec:
            self.get_logger().error(
                'The particle filter never ' +
                ('reported a pose near the seed' if self.seeded else
                 'subscribed to the seed topic') +
                f' in {self.give_up_sec:.0f}s; giving up, and pure_pursuit will '
                'not start. Is localization running, and is this the right map?')
            self.done = True
            return
        if self.seeded:
            return
        if self.pub.get_subscription_count() == 0:
            return
        x, y, qz, qw = self.fields
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self.pub.publish(msg)
        self.remaining -= 1
        if self.remaining <= 0:
            self.get_logger().info(
                f'Seeded the particle filter at laser pose ({x:.2f}, {y:.2f}).')
            self.seeded = True


def main(args=None):
    rclpy.init(args=args)
    node = SeedStartPose()
    succeeded = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        succeeded = node.succeeded
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if succeeded else 1)


if __name__ == '__main__':
    main()
