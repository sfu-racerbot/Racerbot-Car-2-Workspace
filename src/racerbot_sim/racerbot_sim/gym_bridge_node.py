"""F1TENTH Gym behind the same topics the car's drivers publish.

Stands in for everything in `bringup_launch.py` that touches physical
hardware -- `urg_node`, the VESC chain -- so the real driving stack runs
unchanged above it:

    /ackermann_drive  (ackermann_mux output)  -->  vehicle dynamics
    vehicle dynamics  -->  /scan, /odom, TF odom->base_link

Everything else in the graph is the real thing: the real `ackermann_mux`
arbitrating `/teleop` over `/drive`, the real `slam_toolbox`, the real
`gap_follow_node`/`pure_pursuit_node`/`auto_map_race_node`, the real
`web_dashboard`. That is the point -- `tools/f1tenth_sim/run_validation.py`
already covers the control math, and the failures this exists to catch
live in the wiring around it.

Also publishes ground truth on `/sim/...` for scoring. Nothing on the car
subscribes to those, and nothing in this node ever publishes `/drive`.
"""

import json
import math

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, TransformStamped
import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from racerbot_sim import tracks
from racerbot_sim.hardware_guard import HardwareInterlock
from racerbot_sim.sim_bridge import (
    LIDAR_BEAMS, LIDAR_HALF_FOV, LIDAR_OFFSET_X, LIDAR_OFFSET_Z,
    LIDAR_RANGE_MAX, LIDAR_RANGE_MIN, OpponentPlan, SimBridge, SimConfig,
)


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


class GymBridgeNode(Node):

    def __init__(self):
        super().__init__('gym_bridge_node')

        self.declare_parameter('track', 'indoor_oval')
        self.declare_parameter('track_directory', '~/.ros/racerbot_sim/tracks')
        # /ackermann_cmd, not /ackermann_drive: ackermann_mux.cpp advertises
        # "ackermann_cmd" and ackermann_to_vesc.cpp subscribes to it, so
        # bringup_launch.py's `ackermann_cmd_out -> ackermann_drive` remap
        # renames a topic that does not exist and quietly does nothing. This
        # node stands where the VESC does, so it listens where the VESC does.
        self.declare_parameter('drive_topic', '/ackermann_cmd')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('rate_hz', 40.0)
        self.declare_parameter('seed', 12345)
        self.declare_parameter('ego_start_offset_m', 0.0)
        # Semicolon-separated, each entry "offset_m,speed_mps,lateral_m".
        # A speed of 0 parks that car on the line as a static obstacle.
        # One string rather than a string array because a launch argument
        # cannot substitute into an array-typed parameter.
        self.declare_parameter('opponents', '')
        # Mirrors ackermann_mux's own 0.2s input timeout: no fresh command
        # means no drive, which is what the car does when its control layer
        # goes quiet.
        self.declare_parameter('command_timeout_sec', 0.2)
        self.declare_parameter('odom_speed_scale', 1.0)
        self.declare_parameter('odom_speed_noise_std', 0.01)
        self.declare_parameter('odom_yaw_rate_scale', 1.0)
        self.declare_parameter('publish_ground_truth', True)

        def value(name):
            return self.get_parameter(name).value

        self.track_name = str(value('track'))
        self.odom_frame = str(value('odom_frame'))
        self.base_frame = str(value('base_frame'))
        self.laser_frame = str(value('laser_frame'))
        self.rate_hz = float(value('rate_hz'))
        self.command_timeout_sec = float(value('command_timeout_sec'))
        self.publish_ground_truth = bool(value('publish_ground_truth'))

        import os
        track_directory = os.path.expanduser(str(value('track_directory')))
        track_path = tracks.build(self.track_name, track_directory)
        centerline = tracks.load_centerline(track_directory, self.track_name)

        opponents = []
        for spec in str(value('opponents') or '').split(';'):
            spec = spec.strip()
            if not spec:
                continue
            fields = [float(part) for part in spec.split(',')]
            opponents.append(OpponentPlan(
                start_offset_m=fields[0],
                speed=fields[1] if len(fields) > 1 else 1.0,
                lateral_offset_m=fields[2] if len(fields) > 2 else 0.0,
            ))

        self.bridge = SimBridge(SimConfig(
            track_path=str(track_path),
            centerline=centerline,
            control_dt=1.0 / self.rate_hz,
            seed=int(value('seed')),
            ego_start_offset_m=float(value('ego_start_offset_m')),
            opponents=opponents,
            odom_speed_scale=float(value('odom_speed_scale')),
            odom_speed_noise_std=float(value('odom_speed_noise_std')),
            odom_yaw_rate_scale=float(value('odom_yaw_rate_scale')),
        ))

        self.interlock = HardwareInterlock(
            self, 'simulated /scan and /odom')
        self.commanded_speed = 0.0
        self.commanded_steering = 0.0
        self.last_command_time = None
        self.contact_steps = 0
        self.car_contact_steps = 0
        self.ever_contacted = False
        self.distance_travelled = 0.0
        self._previous_xy = None

        self.scan_pub = self.create_publisher(LaserScan, str(value('scan_topic')), 10)
        self.odom_pub = self.create_publisher(Odometry, str(value('odom_topic')), 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            AckermannDriveStamped, str(value('drive_topic')),
            self._drive_callback, 10)

        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.truth_pub = self.create_publisher(PoseStamped, '/sim/ground_truth_pose', 10)
        self.opponents_pub = self.create_publisher(PoseArray, '/sim/opponent_poses', 10)
        self.status_pub = self.create_publisher(String, '/sim/status', latched)

        self._beam_angles = self.bridge.beam_angles
        self.create_timer(1.0 / self.rate_hz, self._step)
        self.get_logger().info(
            f"gym_bridge_node ready: track='{self.track_name}' "
            f'({len(centerline)} centerline points), {self.bridge.num_agents} car(s), '
            f'{LIDAR_BEAMS} beams, {self.rate_hz:.0f}Hz. Listening on '
            f"'{value('drive_topic')}'.")

    # -- ROS plumbing ------------------------------------------------------

    def _drive_callback(self, msg: AckermannDriveStamped):
        self.commanded_speed = float(msg.drive.speed)
        self.commanded_steering = float(msg.drive.steering_angle)
        self.last_command_time = self.get_clock().now()

    def _command_is_fresh(self) -> bool:
        if self.last_command_time is None:
            return False
        age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        return age <= self.command_timeout_sec

    def _step(self):
        if not self.interlock.safe():
            return

        speed = self.commanded_speed if self._command_is_fresh() else 0.0
        steering = self.commanded_steering if self._command_is_fresh() else 0.0
        self.bridge.step(steering, speed)

        ego = self.bridge.agent_state(0)
        if self._previous_xy is not None:
            self.distance_travelled += math.hypot(
                ego.x - self._previous_xy[0], ego.y - self._previous_xy[1])
        self._previous_xy = (ego.x, ego.y)
        if self.bridge.body_contact(0):
            self.contact_steps += 1
            self.ever_contacted = True
        if self.bridge.car_contact():
            self.car_contact_steps += 1

        stamp = self.get_clock().now().to_msg()
        self._publish_scan(stamp)
        self._publish_odom(stamp)
        if self.publish_ground_truth:
            self._publish_truth(stamp, ego)
        self._publish_status(ego)

    def _publish_scan(self, stamp):
        ranges = self.bridge.ego_scan()
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.laser_frame
        msg.angle_min = float(-LIDAR_HALF_FOV)
        msg.angle_max = float(LIDAR_HALF_FOV)
        msg.angle_increment = float(2.0 * LIDAR_HALF_FOV / (LIDAR_BEAMS - 1))
        msg.time_increment = 0.0
        msg.scan_time = float(1.0 / self.rate_hz)
        msg.range_min = float(LIDAR_RANGE_MIN)
        msg.range_max = float(LIDAR_RANGE_MAX)
        msg.ranges = [float(value) for value in ranges]
        self.scan_pub.publish(msg)

    def _publish_odom(self, stamp):
        odom = self.bridge.odom
        qx, qy, qz, qw = yaw_to_quaternion(odom.yaw)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = odom.x
        msg.pose.pose.position.y = odom.y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = odom.speed
        msg.twist.twist.angular.z = odom.yaw_rate
        self.odom_pub.publish(msg)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom.x
        transform.transform.translation.y = odom.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _publish_truth(self, stamp, ego):
        # Deliberately in its own frame, unconnected to the TF tree: this is
        # the answer sheet, and nothing that drives the car may read it.
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'sim_world'
        pose.pose.position.x = ego.x
        pose.pose.position.y = ego.y
        qx, qy, qz, qw = yaw_to_quaternion(ego.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.truth_pub.publish(pose)

        if self.bridge.num_agents > 1:
            array = PoseArray()
            array.header = pose.header
            for index in range(1, self.bridge.num_agents):
                other = self.bridge.agent_state(index)
                item = Pose()
                item.position.x = other.x
                item.position.y = other.y
                ox, oy, oz, ow = yaw_to_quaternion(other.yaw)
                item.orientation.x = ox
                item.orientation.y = oy
                item.orientation.z = oz
                item.orientation.w = ow
                array.poses.append(item)
            self.opponents_pub.publish(array)

    def _publish_status(self, ego):
        message = String()
        message.data = json.dumps({
            'track': self.track_name,
            'sim_time_s': round(self.bridge.sim_time, 3),
            'steps': self.bridge.steps,
            'num_agents': self.bridge.num_agents,
            'ego': {
                'x': round(ego.x, 4), 'y': round(ego.y, 4),
                'yaw': round(ego.yaw, 4), 'speed': round(ego.speed, 4),
            },
            'commanded': {
                'speed': round(self.commanded_speed, 4),
                'steering': round(self.commanded_steering, 4),
                'fresh': self._command_is_fresh(),
            },
            'distance_travelled_m': round(self.distance_travelled, 3),
            'wall_contact_now': bool(self.bridge.body_contact(0)),
            'wall_contact_steps': self.contact_steps,
            'ever_contacted': self.ever_contacted,
            'opponent_wall_contact': any(
                self.bridge.body_contact(i)
                for i in range(1, self.bridge.num_agents)),
            'car_contact_now': bool(self.bridge.car_contact()),
            'car_contact_steps': self.car_contact_steps,
        })
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = GymBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.bridge.close()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
