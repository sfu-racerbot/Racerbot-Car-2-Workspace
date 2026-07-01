import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped


class GapFollowNode(Node):
    """Reactive 'follow the gap' driver: steers at the middle of the widest
    obstacle-free gap in the LIDAR scan, with no map or localization needed.
    """

    def __init__(self):
        super().__init__('gap_follow_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('forward_fov_deg', 180.0)
        self.declare_parameter('bubble_angle_deg', 20.0)
        self.declare_parameter('min_gap_distance', 1.0)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('emergency_stop_distance', 0.15)
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
        self.bubble_angle = math.radians(self.get_parameter('bubble_angle_deg').value)
        self.min_gap_distance = self.get_parameter('min_gap_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.emergency_stop_distance = self.get_parameter('emergency_stop_distance').value
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

        ranges = np.array(scan.ranges, dtype=np.float64)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)

        # Restrict processing to a forward-facing window so the car never
        # steers toward a "gap" that is behind or to the side of it.
        lo_idx, hi_idx = self._fov_indices(scan)
        window = ranges[lo_idx:hi_idx + 1]

        if window.size == 0:
            self._publish_drive(0.0, 0.0)
            return

        closest_idx = int(np.argmin(window))
        closest_dist = float(window[closest_idx])

        if closest_dist < self.emergency_stop_distance:
            self._publish_drive(0.0, 0.0)
            return

        # Carve out a safety bubble around the closest obstacle so the chosen
        # gap never grazes it.
        bubble_radius_idx = max(1, int(self.bubble_angle / scan.angle_increment))
        bubble_lo = max(0, closest_idx - bubble_radius_idx)
        bubble_hi = min(window.size, closest_idx + bubble_radius_idx + 1)
        window[bubble_lo:bubble_hi] = 0.0

        gap_start, gap_end = self._best_gap(window, self.min_gap_distance)
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

    @staticmethod
    def _best_gap(window: np.ndarray, min_gap_distance: float):
        """Pick the best drivable opening, not just the widest one.

        A shallow dead end (e.g. a ~1m doorway alcove) can be angularly
        wider than a genuine, much deeper corridor or track opening. Scoring
        candidates by width * average_depth rather than width alone means a
        gap has to actually be open for a while, not just wide at the mouth,
        to win — so the car stops driving into pockets it can't get back out of.
        """
        free = window > min_gap_distance
        candidates = []
        run_start = None
        for i, is_free in enumerate(free):
            if is_free and run_start is None:
                run_start = i
            elif not is_free and run_start is not None:
                candidates.append((run_start, i - 1))
                run_start = None
        if run_start is not None:
            candidates.append((run_start, len(free) - 1))

        if not candidates:
            return None, None

        def score(run):
            start, end = run
            segment = window[start:end + 1]
            width = end - start + 1
            avg_depth = float(np.mean(segment))
            return width * avg_depth

        best_start, best_end = max(candidates, key=score)
        return best_start, best_end

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
