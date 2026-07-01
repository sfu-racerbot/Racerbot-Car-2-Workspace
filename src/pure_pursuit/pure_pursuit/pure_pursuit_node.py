"""
pure_pursuit_node.py

The race controller. Turns a saved (x, y, speed) racing line plus a live
localization pose into steering + speed commands, at a fixed control rate.

This node does not do any path *planning* -- the racing line is
precomputed offline (drive a lap with waypoint_recorder_node, then run
generate_velocity_profile) and simply loaded from a .csv file here. At
runtime this node's only two jobs, every control tick, are:

  1. Steering: "which way do I need to turn to get back onto the racing
     line and stay on it?" -- answered with the Pure Pursuit algorithm
     (see racing_math.py for the full geometry/derivation).
  2. Speed: "how fast should I be going *right here*?" -- answered by
     reading the precomputed speed at the nearest point on the racing
     line (the curvature-aware velocity profile already baked into the
     .csv by generate_velocity_profile).

Layered on top of both of those is a set of independent safety checks
(a required LB deadman button, stale localization, off-track/lost, and
a reactive LIDAR check for anything not in the map -- an opponent car, a
spun-out car, a dropped glove) that can each unilaterally force the car
to stop. See docs/racing-autonomy.md for the full write-up of the
algorithm and how to tune every parameter below.

Workspace policy (see docs/architecture.md's safety model): every
autonomy node in this repo, this one included, requires the driver to
hold LB on the physical controller for the car to move at all, on top
of whatever ackermann_mux/joy_teleop are doing. This is enforced here
the same way gap_follow_node does it -- subscribing to /joy directly and
refusing to publish a non-zero command unless LB is currently held. This
stays on (`enable_deadman: true`) until the team has enough confidence in
the car's behavior to deliberately relax it -- see docs/architecture.md.

Interface (see docs/writing-your-own-node.md for the general contract
every autonomy node in this repo follows):
  subscribes:  <pose_topic>  geometry_msgs/PoseStamped     (localization)
               <scan_topic>  sensor_msgs/LaserScan          (safety net)
               <joy_topic>   sensor_msgs/Joy                (deadman button)
  publishes:   <drive_topic> ackermann_msgs/AckermannDriveStamped
"""

import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

from pure_pursuit import racing_math


class PurePursuitNode(Node):
    """Map-based race controller: pure pursuit over a precomputed racing line."""

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ------------------------------------------------------------------
        # Parameters. Real values live in config/pure_pursuit.yaml -- see
        # that file for what's actually used and why each one is set the
        # way it is. Declaring them here (instead of hardcoding numbers in
        # the code) is what lets you retune the car from YAML without
        # touching Python, exactly like gap_follow does (see
        # docs/writing-your-own-node.md).
        # ------------------------------------------------------------------
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('closed_loop', True)
        self.declare_parameter('pose_topic', '/pf/viz/inferred_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('control_rate_hz', 40.0)
        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('min_lookahead', 0.6)
        self.declare_parameter('max_lookahead', 2.5)
        self.declare_parameter('lookahead_speed_gain', 0.35)
        self.declare_parameter('nearest_search_window', 40)
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_steering_angle', 0.26)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('max_cross_track_error', 1.0)
        self.declare_parameter('enable_lidar_safety', True)
        self.declare_parameter('safety_fov_deg', 60.0)
        self.declare_parameter('emergency_stop_distance', 0.4)
        self.declare_parameter('scan_timeout_sec', 0.5)
        # --- Deadman button (workspace policy, see docs/architecture.md) ---
        self.declare_parameter('enable_deadman', True)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('joy_timeout_sec', 0.5)

        waypoints_file = self.get_parameter('waypoints_file').value
        self.closed_loop = bool(self.get_parameter('closed_loop').value)
        self.pose_topic = self.get_parameter('pose_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.min_lookahead = float(self.get_parameter('min_lookahead').value)
        self.max_lookahead = float(self.get_parameter('max_lookahead').value)
        self.lookahead_speed_gain = float(self.get_parameter('lookahead_speed_gain').value)
        self.nearest_search_window = int(self.get_parameter('nearest_search_window').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.max_cross_track_error = float(self.get_parameter('max_cross_track_error').value)
        self.enable_lidar_safety = bool(self.get_parameter('enable_lidar_safety').value)
        self.safety_fov_deg = float(self.get_parameter('safety_fov_deg').value)
        self.emergency_stop_distance = float(self.get_parameter('emergency_stop_distance').value)
        self.scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        self.enable_deadman = bool(self.get_parameter('enable_deadman').value)
        self.joy_topic = self.get_parameter('joy_topic').value
        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.joy_timeout_sec = float(self.get_parameter('joy_timeout_sec').value)

        # ------------------------------------------------------------------
        # Load the racing line. Fail loudly and refuse to start rather than
        # silently spinning with no line (or a garbage one) -- a race car
        # with a bad racing line is more dangerous than a race car that
        # refuses to launch at all.
        # ------------------------------------------------------------------
        if not waypoints_file:
            raise RuntimeError(
                "pure_pursuit_node: the 'waypoints_file' parameter is not set. "
                "Point it at a profiled (x,y,speed) .csv produced by "
                "generate_velocity_profile -- see docs/racing-autonomy.md."
            )
        try:
            self.xy, self.speed_profile = racing_math.load_profiled_csv(waypoints_file)
        except (OSError, ValueError) as exc:
            raise RuntimeError(
                f"pure_pursuit_node: could not load waypoints_file '{waypoints_file}': {exc}"
            ) from exc
        if len(self.xy) < 3:
            raise RuntimeError(
                f"pure_pursuit_node: '{waypoints_file}' only has {len(self.xy)} waypoint(s), need at least 3."
            )

        self.num_waypoints = len(self.xy)
        # Distance from each waypoint to the next, precomputed once here at
        # startup -- not every control tick -- since the racing line is
        # fixed for the whole run.
        self.seg_len = racing_math.compute_segment_lengths(self.xy, closed=self.closed_loop)

        # ------------------------------------------------------------------
        # Runtime state. The subscription callbacks below only ever *cache*
        # the latest message + arrival time; all the actual driving logic
        # runs in control_loop() on a fixed-rate timer instead of directly
        # inside a callback. This is deliberate: if a sensor stream dies
        # outright (localization crashes, a LIDAR cable falls out), a
        # callback-driven control loop would simply stop being invoked --
        # and the last command published would stay "live" on the topic
        # forever. Driving the control loop from a timer means the
        # watchdog checks below always keep running, and will notice and
        # stop the car even if a whole sensor feed goes silent.
        # ------------------------------------------------------------------
        self.car_x = None
        self.car_y = None
        self.car_yaw = None
        self.last_pose_time = None
        self.prev_nearest_index = None

        self.last_scan = None
        self.last_scan_time = None

        # Deadman state: same pattern as gap_follow_node -- only ever
        # engages after a live /joy stream has actually shown the button
        # held, so the car never drives before that's been observed.
        self.deadman_held = False
        self.last_joy_time = None

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)
        if self.enable_lidar_safety:
            self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        if self.enable_deadman:
            self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)

        control_period_sec = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(control_period_sec, self.control_loop)

        self.get_logger().info(
            f"pure_pursuit_node ready: {self.num_waypoints} waypoints from '{waypoints_file}' "
            f"({'closed loop' if self.closed_loop else 'open path'}), "
            f"speed profile {float(self.speed_profile.min()):.2f}-{float(self.speed_profile.max()):.2f} m/s, "
            f"control @ {self.control_rate_hz:.0f}Hz, "
            f"deadman button {'ENABLED (LB must be held)' if self.enable_deadman else 'DISABLED'}."
        )

    # ------------------------------------------------------------------------
    # Sensor callbacks -- cache-only, see the comment in __init__ above.
    # ------------------------------------------------------------------------

    def pose_callback(self, msg: PoseStamped):
        self.car_x = msg.pose.position.x
        self.car_y = msg.pose.position.y
        q = msg.pose.orientation
        self.car_yaw = racing_math.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.last_pose_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

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

    # ------------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------------

    def control_loop(self):
        """Runs at control_rate_hz. Wrapped in try/except so that *any*
        unexpected exception still results in a stop command being
        published before the error propagates -- see the module docstring
        on why a moving car should never be left driving on a stale
        command.
        """
        try:
            self._control_step()
        except Exception:
            self.get_logger().error(
                "Unhandled exception in pure_pursuit control loop -- stopping the car.",
                throttle_duration_sec=1.0,
            )
            self._publish_drive(0.0, 0.0)
            raise

    def _control_step(self):
        # --- Watchdog 0: LB deadman button (workspace policy -- see
        # docs/architecture.md). Checked first, ahead of every other
        # watchdog: no held button means no drive command, full stop,
        # regardless of how healthy localization/LIDAR/the racing line are. ---
        if not self._deadman_engaged():
            self._publish_drive(0.0, 0.0)
            return

        # --- Watchdog 1: localization must be alive and recent. ---
        if self.car_x is None or self._seconds_since(self.last_pose_time) > self.pose_timeout_sec:
            self._publish_drive(0.0, 0.0)
            return

        car_xy = (self.car_x, self.car_y)

        # --- Find where we are on the racing line. ---
        nearest_idx, cross_track_error = racing_math.find_nearest_index(
            self.xy, car_xy, closed=self.closed_loop,
            prev_index=self.prev_nearest_index, search_window=self.nearest_search_window,
        )
        self.prev_nearest_index = nearest_idx

        # --- Watchdog 2: are we still actually near the racing line? ---
        # A large cross-track error means the car is lost, kidnapped, or
        # localization has diverged -- driving the pure pursuit geometry
        # anyway would aim the car at a point that may bear no relation to
        # where it actually is.
        if cross_track_error > self.max_cross_track_error:
            self.get_logger().warn(
                f"cross-track error {cross_track_error:.2f}m > max_cross_track_error "
                f"({self.max_cross_track_error:.2f}m) -- stopping.",
                throttle_duration_sec=1.0,
            )
            self._publish_drive(0.0, 0.0)
            return

        # --- Steering: adaptive lookahead + pure pursuit geometry. ---
        # Use the speed *at the car's current position on the line* (not
        # the target's) to size the lookahead -- lookahead should reflect
        # how fast we're going right now, not how fast we will be going
        # once we arrive at the target point.
        speed_here = float(self.speed_profile[nearest_idx])
        lookahead = racing_math.adaptive_lookahead(
            speed_here, self.lookahead_speed_gain, self.min_lookahead, self.max_lookahead)
        target_idx = racing_math.find_lookahead_index(
            self.seg_len, nearest_idx, lookahead, closed=self.closed_loop)
        target_x, target_y = self.xy[target_idx]

        dx = target_x - self.car_x
        dy = target_y - self.car_y
        x_body, y_body = racing_math.world_to_body(dx, dy, self.car_yaw)
        kappa = racing_math.steering_arc_curvature(x_body, y_body)
        steering_angle = racing_math.steering_from_curvature(kappa, self.wheelbase)
        steering_angle = float(np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle))

        # --- Speed: the profiled speed for where the car is right now. ---
        speed_cmd = float(np.clip(speed_here, self.min_speed, self.max_speed))

        # --- Reactive safety net: independent of the plan above. ---
        if self.enable_lidar_safety and self._obstacle_too_close():
            speed_cmd = 0.0

        self._publish_drive(steering_angle, speed_cmd)

    def _obstacle_too_close(self) -> bool:
        """True if something is inside emergency_stop_distance in a narrow
        forward cone of the latest scan -- or if the scan feed itself has
        gone stale/missing, which is treated the same as "too close" on
        purpose: a safety net that has silently gone blind is not a safety
        net.
        """
        if self.last_scan is None or self._seconds_since(self.last_scan_time) > self.scan_timeout_sec:
            return True

        scan = self.last_scan
        half_fov = math.radians(self.safety_fov_deg) / 2.0
        lo_angle = max(scan.angle_min, -half_fov)
        hi_angle = min(scan.angle_max, half_fov)
        n = len(scan.ranges)
        lo_idx = int((lo_angle - scan.angle_min) / scan.angle_increment)
        hi_idx = int((hi_angle - scan.angle_min) / scan.angle_increment)
        lo_idx = max(0, min(lo_idx, n - 1))
        hi_idx = max(0, min(hi_idx, n - 1))
        if hi_idx <= lo_idx:
            return False

        window = np.array(scan.ranges[lo_idx:hi_idx + 1], dtype=np.float64)
        window = window[np.isfinite(window) & (window > 0.0)]
        if window.size == 0:
            return False
        return bool(np.min(window) < self.emergency_stop_distance)

    def _seconds_since(self, stamp) -> float:
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _publish_drive(self, steering_angle: float, speed: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PurePursuitNode()
    except RuntimeError as exc:
        print(f"[pure_pursuit_node] fatal: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
