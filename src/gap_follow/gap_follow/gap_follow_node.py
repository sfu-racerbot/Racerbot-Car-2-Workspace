import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped

from gap_follow import gap_logic


class GapFollowNode(Node):
    """Reactive 'follow the gap' driver: steers at the middle of the best
    obstacle-free gap in the LIDAR scan, with no map or localization
    needed. All the scan-processing math lives in gap_logic.py (importable
    and unit-tested without rclpy); this class owns the ROS plumbing and
    the LB deadman gate, and composes the gap_logic pipeline:
    sanitize -> emergency stop -> disparity extend -> safety bubble ->
    best gap -> steer at its middle.
    """

    def __init__(self):
        super().__init__('gap_follow_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('forward_fov_deg', 180.0)
        self.declare_parameter('min_gap_distance', 1.0)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('emergency_stop_distance', 0.15)
        # Physical clearance model, replacing the old fixed-angle
        # bubble_angle_deg: every obstacle edge and the safety bubble are
        # sized by what (car_width/2 + safety_margin) actually subtends
        # at the obstacle's distance -- see gap_logic.disparity_extend /
        # safety_bubble.
        self.declare_parameter('car_width', 0.30)
        self.declare_parameter('safety_margin', 0.10)
        self.declare_parameter('disparity_threshold', 0.4)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('joy_timeout_sec', 0.5)
        # Workspace policy (see docs/architecture.md): the deadman button
        # stays enabled until the team has enough confidence in the car's
        # behavior to deliberately relax it -- don't set this false otherwise.
        self.declare_parameter('enable_deadman', True)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.max_range = self.get_parameter('max_range').value
        self.forward_fov = math.radians(self.get_parameter('forward_fov_deg').value)
        self.min_gap_distance = self.get_parameter('min_gap_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.emergency_stop_distance = self.get_parameter('emergency_stop_distance').value
        self.car_width = float(self.get_parameter('car_width').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.disparity_threshold = float(self.get_parameter('disparity_threshold').value)
        self.joy_topic = self.get_parameter('joy_topic').value
        self.deadman_button = self.get_parameter('deadman_button').value
        self.joy_timeout_sec = self.get_parameter('joy_timeout_sec').value
        self.enable_deadman = bool(self.get_parameter('enable_deadman').value)

        # Deadman state: gap_follow only drives while this button is held on
        # a live /joy stream. Defaults to "not engaged" so the car never
        # drives before a held-button signal has actually been seen.
        self.deadman_held = False
        self.last_joy_time = None

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        if len(msg.buttons) > self.deadman_button:
            self.deadman_held = bool(msg.buttons[self.deadman_button])
        else:
            self.deadman_held = False

    def _deadman_engaged(self) -> bool:
        if not self.enable_deadman:
            return True
        if not self.deadman_held or self.last_joy_time is None:
            return False
        age_sec = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        return age_sec < self.joy_timeout_sec

    def scan_callback(self, scan: LaserScan):
        if not self._deadman_engaged():
            self._publish_drive(0.0, 0.0)
            return

        # Invalid beams (NaN, sub-range_min) are *unknown*, not contact:
        # they must never trigger the emergency stop, but must stay
        # non-free for gap selection -- see gap_logic.sanitize_ranges.
        clean, valid = gap_logic.sanitize_ranges(scan.ranges, self.max_range, scan.range_min)

        # Restrict processing to a forward-facing window so the car never
        # steers toward a "gap" that is behind or to the side of it.
        lo_idx, hi_idx = self._fov_indices(scan)
        window = clean[lo_idx:hi_idx + 1]
        window_valid = valid[lo_idx:hi_idx + 1]

        if window.size == 0:
            self._publish_drive(0.0, 0.0)
            return

        closest_idx, closest_dist = gap_logic.closest_valid(window, window_valid)

        if closest_dist < self.emergency_stop_distance:
            self._publish_drive(0.0, 0.0)
            return

        # Make the ranges clearance-aware: every obstacle edge is widened
        # by half a car width at its own distance, then a same-sized
        # bubble is carved around the closest obstacle so no chosen gap
        # can graze it.
        half_width = self.car_width / 2.0 + self.safety_margin
        window = gap_logic.disparity_extend(
            window, scan.angle_increment, self.disparity_threshold, half_width)
        if closest_idx is not None:
            window = gap_logic.safety_bubble(
                window, closest_idx, closest_dist, scan.angle_increment, half_width)

        gap_start, gap_end = gap_logic.find_best_gap(
            window, self.min_gap_distance,
            angle_increment=scan.angle_increment,
            min_gap_width_m=self.car_width + self.safety_margin)
        if gap_start is None:
            self._publish_drive(0.0, 0.0)
            return

        target_idx_in_window = (gap_start + gap_end) // 2
        target_idx = lo_idx + target_idx_in_window
        steering_angle = scan.angle_min + target_idx * scan.angle_increment
        steering_angle = float(np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle))

        speed_scale = 1.0 - (abs(steering_angle) / self.max_steering_angle)
        speed = self.min_speed + speed_scale * (self.max_speed - self.min_speed)

        self._publish_drive(steering_angle, speed)

    def _fov_indices(self, scan: LaserScan):
        half_fov = self.forward_fov / 2.0
        lo_angle = max(scan.angle_min, -half_fov)
        hi_angle = min(scan.angle_max, half_fov)
        num_points = len(scan.ranges)
        lo_idx = int((lo_angle - scan.angle_min) / scan.angle_increment)
        hi_idx = int((hi_angle - scan.angle_min) / scan.angle_increment)
        lo_idx = max(0, min(lo_idx, num_points - 1))
        hi_idx = max(0, min(hi_idx, num_points - 1))
        return lo_idx, hi_idx

    def _publish_drive(self, steering_angle: float, speed: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # The default SIGINT handler may already have shut the context down.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
