import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Joy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

from gap_follow import gap_logic


class GapFollowNode(Node):
    """Reactive 'follow the gap' driver: steers at the middle of the best
    obstacle-free gap in the LIDAR scan, with no map or localization
    needed. All the scan-processing math lives in gap_logic.py (importable
    and unit-tested without rclpy); this class owns the ROS plumbing and
    the LB deadman gate, and composes the gap_logic pipeline:
    sanitize -> footprint clearance/TTC -> disparity extend -> safety
    bubble -> deep gap or tight-corner fallback -> steer at its middle.
    """

    def __init__(self):
        super().__init__('gap_follow_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('forward_fov_deg', 180.0)
        self.declare_parameter('min_gap_distance', 1.0)
        self.declare_parameter('fallback_min_gap_distance', 0.8)
        self.declare_parameter('corner_speed', 0.5)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_steering_angle', 0.4189)
        # Padded Traxxas 74276-4 footprint (physical: 0.281 x 0.535 m),
        # combined with its rear-axle base_link and estimated LiDAR transform.
        self.declare_parameter('car_width', 0.31)
        self.declare_parameter('car_length', 0.58)
        self.declare_parameter('wheelbase', 0.324)
        self.declare_parameter('laser_offset_x', 0.33)
        self.declare_parameter('laser_offset_y', 0.0)
        self.declare_parameter('safety_margin', 0.10)
        self.declare_parameter('disparity_threshold', 0.4)
        # Obstacle inflation already accounts for the full car width. This
        # threshold is only the remaining centerline corridor after inflation.
        self.declare_parameter('min_centerline_gap_width', 0.10)
        self.declare_parameter('emergency_stop_clearance', 0.02)

        # F1TENTH instantaneous TTC, based on measured odometry speed.
        self.declare_parameter('enable_ttc', True)
        self.declare_parameter('ttc_threshold_sec', 0.5)
        self.declare_parameter('ttc_min_closing_speed', 0.05)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('joy_timeout_sec', 0.5)
        # Diagnostics are deliberately slower than the LIDAR callback:
        # state changes are logged immediately, then the current decision is
        # repeated at this interval so launch output stays readable.
        self.declare_parameter('decision_log_period_sec', 1.0)
        # Gap follow is callback-driven. If scans stop, /drive naturally
        # times out at the mux; this timeout lets the status timer explain
        # that otherwise-silent stop in the launch terminal.
        self.declare_parameter('scan_timeout_sec', 0.5)
        # Workspace policy (see docs/architecture.md): the deadman button
        # stays enabled until the team has enough confidence in the car's
        # behavior to deliberately relax it -- don't set this false otherwise.
        self.declare_parameter('enable_deadman', True)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.max_range = self.get_parameter('max_range').value
        self.forward_fov = math.radians(self.get_parameter('forward_fov_deg').value)
        self.min_gap_distance = float(
            self.get_parameter('min_gap_distance').value)
        self.fallback_min_gap_distance = float(
            self.get_parameter('fallback_min_gap_distance').value)
        self.corner_speed = float(
            self.get_parameter('corner_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value)
        self.car_width = float(self.get_parameter('car_width').value)
        self.car_length = float(self.get_parameter('car_length').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.laser_offset_x = float(
            self.get_parameter('laser_offset_x').value)
        self.laser_offset_y = float(
            self.get_parameter('laser_offset_y').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.disparity_threshold = float(
            self.get_parameter('disparity_threshold').value)
        self.min_centerline_gap_width = float(
            self.get_parameter('min_centerline_gap_width').value)
        self.emergency_stop_clearance = float(
            self.get_parameter('emergency_stop_clearance').value)
        self.enable_ttc = bool(self.get_parameter('enable_ttc').value)
        self.ttc_threshold_sec = float(
            self.get_parameter('ttc_threshold_sec').value)
        self.ttc_min_closing_speed = float(
            self.get_parameter('ttc_min_closing_speed').value)
        self.odom_timeout_sec = float(
            self.get_parameter('odom_timeout_sec').value)
        self.joy_topic = self.get_parameter('joy_topic').value
        self.deadman_button = self.get_parameter('deadman_button').value
        self.joy_timeout_sec = self.get_parameter('joy_timeout_sec').value
        self.decision_log_period_sec = max(
            0.0, float(self.get_parameter('decision_log_period_sec').value))
        self.scan_timeout_sec = max(
            0.05, float(self.get_parameter('scan_timeout_sec').value))
        self.enable_deadman = bool(self.get_parameter('enable_deadman').value)

        # Validate the footprint and sensor origin once at startup.
        gap_logic.vehicle_boundary_distances(
            np.array([0.0]),
            self.car_width,
            self.car_length,
            self.wheelbase,
            self.laser_offset_x,
            self.laser_offset_y,
        )

        # Deadman state: gap_follow only drives while this button is held on
        # a live /joy stream. Defaults to "not engaged" so the car never
        # drives before a held-button signal has actually been seen.
        self.deadman_held = False
        self.last_joy_time = None
        self.joy_button_available = False
        self.current_speed = 0.0
        self.last_odom_time = None

        # Runtime diagnostics. The scan watchdog is informational: when a
        # callback-driven controller stops receiving scans it publishes no
        # new command, and ackermann_mux stops the car when /drive times out.
        self.last_scan_time = None
        self.last_decision_state = None
        self.last_decision_log_time = None

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, 10)
        self.status_timer = self.create_timer(
            min(0.25, self.scan_timeout_sec / 2.0), self._sensor_status_callback)

        self.get_logger().info(
            f"gap_follow_node ready: scan='{self.scan_topic}' -> drive='{self.drive_topic}', "
            f"speed={self.min_speed:.2f}-{self.max_speed:.2f}m/s, "
            f"deadman {'ENABLED (LB must be held)' if self.enable_deadman else 'DISABLED'}, "
            f"decision logs every {self.decision_log_period_sec:.1f}s "
            "(plus immediate state changes).")

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        self.joy_button_available = len(msg.buttons) > self.deadman_button
        if self.joy_button_available:
            self.deadman_held = bool(msg.buttons[self.deadman_button])
        else:
            self.deadman_held = False

    def odom_callback(self, msg: Odometry):
        self.current_speed = float(msg.twist.twist.linear.x)
        self.last_odom_time = self.get_clock().now()

    def _odom_fresh(self) -> bool:
        if self.last_odom_time is None or not math.isfinite(self.current_speed):
            return False
        age_sec = (
            self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        return age_sec < self.odom_timeout_sec

    def _deadman_engaged(self) -> bool:
        return self._deadman_status()[0]

    def _deadman_status(self):
        """Return engagement plus a precise diagnostic when it is false."""
        if not self.enable_deadman:
            return True, None, None
        if self.last_joy_time is None:
            return (
                False,
                'waiting_for_joy',
                f"no Joy messages received on '{self.joy_topic}'; LB cannot be verified",
            )
        age_sec = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if age_sec >= self.joy_timeout_sec:
            return (
                False,
                'joy_stale',
                f"last Joy message is {age_sec:.2f}s old (limit {self.joy_timeout_sec:.2f}s)",
            )
        if not self.joy_button_available:
            return (
                False,
                'deadman_button_missing',
                f"Joy message has no button index {self.deadman_button} (LB)",
            )
        if not self.deadman_held:
            return False, 'deadman_released', 'LB deadman button is not held'
        return True, None, None

    def scan_callback(self, scan: LaserScan):
        self.last_scan_time = self.get_clock().now()

        deadman_ok, stop_state, stop_detail = self._deadman_status()
        if not deadman_ok:
            self._stop(stop_state, stop_detail)
            return
        if self.enable_ttc and not self._odom_fresh():
            self._stop(
                'odometry_stale',
                f"TTC is enabled but no odometry newer than "
                f"{self.odom_timeout_sec:.2f}s is available on "
                f"'{self.odom_topic}'",
            )
            return

        if not scan.ranges:
            self._stop('scan_empty', 'LaserScan contains no range beams')
            return
        if not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0.0:
            self._stop(
                'scan_invalid',
                f"LaserScan angle_increment={scan.angle_increment!r} is not positive and finite",
            )
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
            self._stop(
                'scan_window_empty',
                f"no beams fall inside the configured {math.degrees(self.forward_fov):.1f}deg FOV",
            )
            return

        beam_indices = np.arange(lo_idx, hi_idx + 1, dtype=np.float64)
        beam_angles = scan.angle_min + beam_indices * scan.angle_increment
        body_boundaries = gap_logic.vehicle_boundary_distances(
            beam_angles,
            self.car_width,
            self.car_length,
            self.wheelbase,
            self.laser_offset_x,
            self.laser_offset_y,
        )

        # Hard contact layer using actual clearance from the rectangular body,
        # rather than raw distance from the offset LiDAR origin.
        min_clearance = gap_logic.minimum_footprint_clearance(
            window, window_valid, body_boundaries)
        if min_clearance <= self.emergency_stop_clearance:
            self._stop(
                'emergency_clearance',
                f"minimum body clearance {min_clearance:.3f}m is at or below "
                f"the {self.emergency_stop_clearance:.3f}m threshold",
            )
            return

        # Independent speed-aware layer from F1TENTH Lab 2:
        # iTTC = body clearance / (longitudinal speed * cos(beam angle)).
        if self.enable_ttc:
            min_ttc = gap_logic.minimum_ttc(
                window,
                window_valid,
                beam_angles,
                self.current_speed,
                body_boundaries,
                self.ttc_min_closing_speed,
            )
            if min_ttc <= self.ttc_threshold_sec:
                self._stop(
                    'ttc_brake',
                    f"minimum footprint-aware TTC {min_ttc:.3f}s is at or "
                    f"below the {self.ttc_threshold_sec:.3f}s threshold at "
                    f"{self.current_speed:.2f}m/s",
                )
                return

        closest_idx, closest_dist = gap_logic.closest_valid(
            window, window_valid)

        # Inflate each edge by half the car width plus one side's margin.
        # Remaining ranges represent valid car-center positions.
        half_width = self.car_width / 2.0 + self.safety_margin
        window = gap_logic.disparity_extend(
            window,
            scan.angle_increment,
            self.disparity_threshold,
            half_width,
        )
        if closest_idx is not None:
            window = gap_logic.safety_bubble(
                window,
                closest_idx,
                closest_dist,
                scan.angle_increment,
                half_width,
            )

        gap_start, gap_end, used_fallback = gap_logic.find_gap_with_fallback(
            window,
            self.min_gap_distance,
            self.fallback_min_gap_distance,
            scan.angle_increment,
            self.min_centerline_gap_width,
        )
        if gap_start is None:
            closest_text = (
                f"{closest_dist:.2f}m"
                if math.isfinite(closest_dist)
                else "no valid return")
            self._stop(
                'no_safe_gap',
                f"no gap exceeds either the preferred "
                f"{self.min_gap_distance:.2f}m depth or tight-corner "
                f"{self.fallback_min_gap_distance:.2f}m depth with "
                f"{self.min_centerline_gap_width:.2f}m of center corridor; "
                f"closest={closest_text}",
            )
            return

        target_idx_in_window = (gap_start + gap_end) // 2
        target_idx = lo_idx + target_idx_in_window
        target_angle = scan.angle_min + target_idx * scan.angle_increment
        steering_angle = target_angle
        steering_angle = float(np.clip(
            steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle,
        ))

        speed_scale = 1.0 - (
            abs(steering_angle) / self.max_steering_angle)
        speed = self.min_speed + speed_scale * (
            self.max_speed - self.min_speed)
        if used_fallback:
            speed = min(speed, self.corner_speed)

        self._publish_drive(steering_angle, speed)
        gap_lo_angle = scan.angle_min + (lo_idx + gap_start) * scan.angle_increment
        gap_hi_angle = scan.angle_min + (lo_idx + gap_end) * scan.angle_increment
        closest_text = (
            f"{closest_dist:.2f}m" if math.isfinite(closest_dist) else "no valid return")
        clipped_text = (
            f", clipped from {target_angle:+.3f}rad"
            if not math.isclose(steering_angle, target_angle) else "")
        gap_mode = 'corner_fallback' if used_fallback else 'gap_follow'
        depth_text = (
            f"fallback depth {self.fallback_min_gap_distance:.2f}m"
            if used_fallback
            else f"preferred depth {self.min_gap_distance:.2f}m")
        speed_text = (
            f"corner cap {self.corner_speed:.2f}m/s"
            if used_fallback
            else f"{speed_scale * 100.0:.0f}% steering speed scale")
        self._log_decision(
            gap_mode,
            f"selected {depth_text} gap "
            f"{math.degrees(gap_lo_angle):+.1f}deg to "
            f"{math.degrees(gap_hi_angle):+.1f}deg; aiming at its midpoint, "
            f"closest={closest_text}; speed uses {speed_text}{clipped_text}",
            steering_angle,
            speed,
        )

    def _sensor_status_callback(self):
        """Explain a missing scan stream even though scan_callback is idle."""
        if self.last_scan_time is None:
            self._log_decision(
                'waiting_for_scan',
                f"no LaserScan received on '{self.scan_topic}'; "
                "no drive command is being generated",
                0.0,
                0.0,
                command_published=False,
            )
            return
        age_sec = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if age_sec >= self.scan_timeout_sec:
            self._log_decision(
                'scan_stale',
                f"last LaserScan is {age_sec:.2f}s old (limit {self.scan_timeout_sec:.2f}s); "
                "/drive has gone quiet and the mux will stop the car",
                0.0,
                0.0,
                command_published=False,
            )

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

    def _stop(self, state: str, detail: str):
        self._publish_drive(0.0, 0.0)
        self._log_decision(state, detail, 0.0, 0.0)

    def _log_decision(self, state: str, detail: str, steering_angle: float,
                      speed: float, command_published: bool = True):
        """Log state transitions immediately and steady decisions periodically."""
        now = self.get_clock().now()
        state_changed = state != self.last_decision_state
        period_elapsed = (
            self.decision_log_period_sec > 0.0
            and (
                self.last_decision_log_time is None
                or (now - self.last_decision_log_time).nanoseconds / 1e9
                >= self.decision_log_period_sec
            )
        )
        if not state_changed and not period_elapsed:
            return

        stopped = speed <= 0.0
        if command_published:
            output = f"command: steering={steering_angle:+.3f}rad, speed={speed:.2f}m/s"
        else:
            output = "command: none (the mux stops when /drive times out)"
        message = f"{'STOP' if stopped else 'DRIVE'} [{state}] {detail}; {output}"
        if stopped:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)
        self.last_decision_state = state
        self.last_decision_log_time = now


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
