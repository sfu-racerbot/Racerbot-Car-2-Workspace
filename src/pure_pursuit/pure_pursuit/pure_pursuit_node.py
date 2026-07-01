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
to stop or steer around something. On top of *that* is one more layer,
squarely about racing rather than just safety: if that LIDAR check
recognizes "something not in the map" specifically as another car --
not a wall, not debris -- and this car is closing in on it, it plans and
steers an overtake instead of just following at a safe distance
forever. See docs/racing-autonomy.md for the full write-up of the
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


class OpponentTracker:
    """Tracks a single detected opponent's progress *along the racing
    line* across control ticks -- not raw x/y position -- so
    pure_pursuit_node can answer "am I catching them?" directly, the same
    question a human racer actually asks. Kept as a small, separate
    object (rather than another half-dozen loose attributes on the node)
    since it has its own bit of state and update logic that reads more
    clearly grouped together, and is easier to unit-test in isolation.

    `arc_length` is how far along the recorded racing line the opponent
    currently is (see racing_math.compute_cumulative_arc_length).
    `progress_rate` is an exponentially-smoothed estimate of how fast
    that's increasing, in m/s -- i.e. the opponent's speed *along the
    track*, which is a far more useful prediction of "where will they be
    in a second" than raw x/y velocity would be, because it automatically
    follows the track's own curvature instead of assuming they drive in
    a straight line off of it.
    """

    def __init__(self, smoothing_alpha: float, lost_timeout_sec: float):
        self.smoothing_alpha = smoothing_alpha
        self.lost_timeout_sec = lost_timeout_sec
        self.arc_length = None
        self.progress_rate = 0.0
        self.last_update_time = None
        self.prev_nearest_index = None

    def update(self, arc_length: float, now_sec: float):
        if self.arc_length is not None and self.last_update_time is not None:
            dt = now_sec - self.last_update_time
            if dt > 1e-3:
                raw_rate = (arc_length - self.arc_length) / dt
                # Guard against a single bogus jump (a bad cluster match,
                # or the arc-length wrapping across the finish line)
                # corrupting the smoothed estimate for several seconds
                # afterward.
                if abs(raw_rate) < 20.0:
                    alpha = self.smoothing_alpha
                    self.progress_rate = alpha * raw_rate + (1.0 - alpha) * self.progress_rate
        self.arc_length = arc_length
        self.last_update_time = now_sec

    def is_fresh(self, now_sec: float) -> bool:
        return (self.last_update_time is not None
                and (now_sec - self.last_update_time) < self.lost_timeout_sec)


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

        # --- Reactive avoidance (steer around something close, not just
        # stop, when there's room) ---
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.declare_parameter('avoidance_fov_deg', 100.0)
        self.declare_parameter('avoidance_trigger_distance', 1.5)
        self.declare_parameter('avoidance_min_gap_distance', 1.0)
        self.declare_parameter('avoidance_speed', 1.0)

        # --- Opponent detection, tracking, and overtaking -- see
        # docs/racing-autonomy.md's "Racing against opponents" ---
        self.declare_parameter('enable_opponent_overtake', True)
        self.declare_parameter('opponent_min_width', 0.15)
        self.declare_parameter('opponent_max_width', 0.7)
        self.declare_parameter('opponent_cluster_gap', 0.3)
        self.declare_parameter('opponent_engagement_range', 5.0)
        self.declare_parameter('opponent_open_side_margin', 0.5)
        self.declare_parameter('opponent_velocity_smoothing', 0.3)
        self.declare_parameter('opponent_lost_timeout_sec', 1.0)
        self.declare_parameter('overtake_trigger_gap', 3.0)
        self.declare_parameter('overtake_closing_margin', 0.3)
        self.declare_parameter('overtake_clear_margin', 1.0)
        self.declare_parameter('overtake_lateral_offset', 0.5)
        self.declare_parameter('laser_offset_x', 0.27)
        self.declare_parameter('laser_offset_y', 0.0)

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

        self.max_range = float(self.get_parameter('max_range').value)
        self.enable_obstacle_avoidance = bool(self.get_parameter('enable_obstacle_avoidance').value)
        self.avoidance_fov_deg = float(self.get_parameter('avoidance_fov_deg').value)
        self.avoidance_trigger_distance = float(self.get_parameter('avoidance_trigger_distance').value)
        self.avoidance_min_gap_distance = float(self.get_parameter('avoidance_min_gap_distance').value)
        self.avoidance_speed = float(self.get_parameter('avoidance_speed').value)

        self.enable_opponent_overtake = bool(self.get_parameter('enable_opponent_overtake').value)
        self.opponent_min_width = float(self.get_parameter('opponent_min_width').value)
        self.opponent_max_width = float(self.get_parameter('opponent_max_width').value)
        self.opponent_cluster_gap = float(self.get_parameter('opponent_cluster_gap').value)
        self.opponent_engagement_range = float(self.get_parameter('opponent_engagement_range').value)
        self.opponent_open_side_margin = float(self.get_parameter('opponent_open_side_margin').value)
        self.opponent_velocity_smoothing = float(self.get_parameter('opponent_velocity_smoothing').value)
        self.opponent_lost_timeout_sec = float(self.get_parameter('opponent_lost_timeout_sec').value)
        self.overtake_trigger_gap = float(self.get_parameter('overtake_trigger_gap').value)
        self.overtake_closing_margin = float(self.get_parameter('overtake_closing_margin').value)
        self.overtake_clear_margin = float(self.get_parameter('overtake_clear_margin').value)
        self.overtake_lateral_offset = float(self.get_parameter('overtake_lateral_offset').value)
        self.laser_offset_x = float(self.get_parameter('laser_offset_x').value)
        self.laser_offset_y = float(self.get_parameter('laser_offset_y').value)

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
        # Arc-length (track distance from waypoint 0) at every waypoint,
        # and the total lap distance -- also precomputed once. Used to
        # compare *where along the track* the ego car and any tracked
        # opponent are, a far more meaningful notion of "ahead"/"behind"
        # on a lap than raw straight-line distance -- see
        # docs/racing-autonomy.md's "Racing against opponents".
        self.cumulative_arc_length = racing_math.compute_cumulative_arc_length(self.seg_len)
        self.total_track_length = float(np.sum(self.seg_len))

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

        # Opponent tracking + overtake state -- see OpponentTracker above
        # and _update_opponent_and_overtake below.
        self.opponent = OpponentTracker(self.opponent_velocity_smoothing, self.opponent_lost_timeout_sec)
        self.overtake_active = False
        self.overtake_side = 1  # +1 = pass on the left, -1 = pass on the right

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
            f"deadman button {'ENABLED (LB must be held)' if self.enable_deadman else 'DISABLED'}, "
            f"obstacle avoidance {'ON' if self.enable_obstacle_avoidance else 'OFF'}, "
            f"opponent overtaking {'ON' if self.enable_opponent_overtake else 'OFF'}."
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

        # --- Opponent tracking + overtaking: reconsiders the steering
        # *target* (not yet the final command) if another car has been
        # spotted and this car is closing in on it. Requires the reactive
        # safety net to be enabled too -- overtaking is a more assertive
        # behavior layered on top of it, not a substitute for it. ---
        if self.enable_lidar_safety and self.enable_opponent_overtake:
            overtake_target = self._update_opponent_and_overtake(nearest_idx, target_idx)
            if overtake_target is not None:
                dx = overtake_target[0] - self.car_x
                dy = overtake_target[1] - self.car_y
                x_body, y_body = racing_math.world_to_body(dx, dy, self.car_yaw)
                kappa = racing_math.steering_arc_curvature(x_body, y_body)
                steering_angle = racing_math.steering_from_curvature(kappa, self.wheelbase)
                steering_angle = float(np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle))

        # --- Reactive safety net: independent of everything above, and
        # always gets the final say regardless of the racing line or any
        # overtake in progress. ---
        if self.enable_lidar_safety:
            steering_override, speed_override = self._reactive_override()
            if steering_override is not None:
                steering_angle = steering_override
            if speed_override is not None:
                speed_cmd = speed_override

        self._publish_drive(steering_angle, speed_cmd)

    def _fov_indices(self, scan: LaserScan, fov_deg: float):
        """Index bounds in `scan.ranges` for a forward cone `fov_deg`
        wide, centered dead ahead. Shared by every reactive/opponent
        check below so the "angle -> array index" conversion is only
        written once.
        """
        half_fov = math.radians(fov_deg) / 2.0
        lo_angle = max(scan.angle_min, -half_fov)
        hi_angle = min(scan.angle_max, half_fov)
        n = len(scan.ranges)
        lo_idx = int((lo_angle - scan.angle_min) / scan.angle_increment)
        hi_idx = int((hi_angle - scan.angle_min) / scan.angle_increment)
        return max(0, min(lo_idx, n - 1)), max(0, min(hi_idx, n - 1))

    def _closest_in_cone(self, scan: LaserScan, fov_deg: float) -> float:
        """Closest *valid* (finite, positive) reading within a forward
        cone, or +inf if the cone is empty or nothing valid was seen.
        Used for yes/no distance checks (hard-stop, avoidance trigger)
        that only need to know *how close*, not *which beam*.
        """
        lo_idx, hi_idx = self._fov_indices(scan, fov_deg)
        if hi_idx <= lo_idx:
            return math.inf
        window = np.array(scan.ranges[lo_idx:hi_idx + 1], dtype=np.float64)
        window = window[np.isfinite(window) & (window > 0.0)]
        if window.size == 0:
            return math.inf
        return float(np.min(window))

    def _sanitized_window(self, scan: LaserScan, lo_idx: int, hi_idx: int) -> np.ndarray:
        """Ranges in [lo_idx, hi_idx], NaN/inf *replaced* (not removed) so
        the array's length and index positions still line up with the
        original scan -- required whenever the *position* of a reading
        matters (gap-finding), unlike _closest_in_cone above where only
        the closest value matters and indices don't need to survive.
        """
        if hi_idx <= lo_idx:
            return np.array([])
        window = np.array(scan.ranges[lo_idx:hi_idx + 1], dtype=np.float64)
        window = np.nan_to_num(window, nan=0.0, posinf=self.max_range, neginf=0.0)
        return np.clip(window, 0.0, self.max_range)

    def _reactive_override(self):
        """The reactive LIDAR safety net -- independent of the racing
        line and the overtake logic above, and always has the final say.
        Returns (steering_override, speed_override); either is None where
        that part of the already-computed plan should be left alone.

        Two tiers, most urgent first:
          1. Something is inside emergency_stop_distance in a narrow
             forward cone, or the scan feed itself is stale/missing
             (treated the same as "too close" -- a safety net that's
             gone blind isn't a safety net) -- hard stop, steering left
             alone so the wheels stay pointed to resume the line once
             clear.
          2. Something is inside avoidance_trigger_distance in a wider
             cone (but outside the hard-stop distance) -- steer at the
             best gap instead of stopping, at a capped cautious speed,
             *if* a wide-enough gap actually exists; otherwise also stop
             rather than commit to a guessed steering angle.
        """
        if self.last_scan is None or self._seconds_since(self.last_scan_time) > self.scan_timeout_sec:
            return None, 0.0

        scan = self.last_scan

        if self._closest_in_cone(scan, self.safety_fov_deg) < self.emergency_stop_distance:
            return None, 0.0

        if not self.enable_obstacle_avoidance:
            return None, None

        if self._closest_in_cone(scan, self.avoidance_fov_deg) >= self.avoidance_trigger_distance:
            return None, None

        lo_idx, hi_idx = self._fov_indices(scan, self.avoidance_fov_deg)
        window = self._sanitized_window(scan, lo_idx, hi_idx)
        if window.size == 0:
            return None, 0.0

        gap_start, gap_end = racing_math.find_best_gap(window, self.avoidance_min_gap_distance)
        if gap_start is None:
            return None, 0.0  # boxed in -- stop rather than guess

        target_idx = lo_idx + (gap_start + gap_end) // 2
        angle = scan.angle_min + target_idx * scan.angle_increment
        angle = float(np.clip(angle, -self.max_steering_angle, self.max_steering_angle))
        return angle, self.avoidance_speed

    def _update_opponent_and_overtake(self, nearest_idx: int, target_idx: int):
        """Look for another car in the live scan, track its progress along
        the racing line, and decide whether to start, continue, or end an
        overtake. Returns a (x, y) world-frame point to steer at instead
        of the normal Pure Pursuit target if an overtake is in progress,
        or None to leave the plan alone. See "Racing against opponents" in
        docs/racing-autonomy.md for the full strategy this implements.
        """
        now_sec = self.get_clock().now().nanoseconds / 1e9

        detection = None
        ranges = None
        if self.last_scan is not None and self._seconds_since(self.last_scan_time) <= self.scan_timeout_sec:
            scan = self.last_scan
            ranges = np.nan_to_num(np.array(scan.ranges, dtype=np.float64),
                                    nan=0.0, posinf=self.max_range, neginf=0.0)
            ranges = np.clip(ranges, 0.0, self.max_range)
            candidate = racing_math.detect_opponent_cluster(
                ranges, scan.angle_min, scan.angle_increment, self.max_range,
                self.opponent_min_width, self.opponent_max_width,
                self.opponent_engagement_range, self.opponent_cluster_gap,
                self.opponent_open_side_margin)
            if candidate is not None:
                half_fov = math.radians(self.avoidance_fov_deg) / 2.0
                if -half_fov <= candidate[3] <= half_fov:
                    detection = candidate

        start_idx = end_idx = None
        if detection is not None:
            start_idx, end_idx, centroid_range, centroid_angle = detection

            # Where that cluster actually is in the map frame, so its
            # progress along the racing line can be measured the same way
            # the ego car's own position is.
            cos_yaw, sin_yaw = math.cos(self.car_yaw), math.sin(self.car_yaw)
            laser_world_x = self.car_x + self.laser_offset_x * cos_yaw - self.laser_offset_y * sin_yaw
            laser_world_y = self.car_y + self.laser_offset_x * sin_yaw + self.laser_offset_y * cos_yaw
            world_angle = self.car_yaw + centroid_angle
            opponent_x = laser_world_x + centroid_range * math.cos(world_angle)
            opponent_y = laser_world_y + centroid_range * math.sin(world_angle)

            opp_idx, _ = racing_math.find_nearest_index(
                self.xy, (opponent_x, opponent_y), closed=self.closed_loop,
                prev_index=self.opponent.prev_nearest_index, search_window=self.nearest_search_window)
            self.opponent.prev_nearest_index = opp_idx
            self.opponent.update(float(self.cumulative_arc_length[opp_idx]), now_sec)

        if self.opponent.arc_length is None or not self.opponent.is_fresh(now_sec):
            # Never seen one yet, or haven't seen one recently enough to
            # trust its last known position -- nothing to react to, or
            # nothing left to finish reacting to.
            self.overtake_active = False
            return None

        ego_arc_length = float(self.cumulative_arc_length[nearest_idx])
        gap_ahead = racing_math.track_progress_gap(
            ego_arc_length, self.opponent.arc_length, self.total_track_length)
        ego_speed = float(self.speed_profile[nearest_idx])
        closing_fast_enough = (ego_speed - self.opponent.progress_rate) > self.overtake_closing_margin

        if self.overtake_active:
            # Already committed -- keep going until safely past. This is
            # *not* re-gated on a fresh detection this exact tick: once
            # alongside or just past an opponent, it commonly drops clean
            # out of the forward LIDAR cone, and that must not look like
            # "lost it, cancel" -- the last tracked position (still
            # "fresh" per the check above) is enough to tell whether the
            # pass is actually complete yet.
            if gap_ahead > self.total_track_length - self.overtake_clear_margin:
                self.overtake_active = False
        elif detection is not None and gap_ahead <= self.overtake_trigger_gap and closing_fast_enough:
            # Starting a *new* overtake does need this tick's actual scan
            # -- picking which side to pass on reads directly from it.
            self.overtake_active = True
            self.overtake_side = racing_math.pick_pass_side(ranges, start_idx, end_idx)
            self.get_logger().info(
                f"overtake: opponent {gap_ahead:.1f}m ahead on track, closing at "
                f"{ego_speed - self.opponent.progress_rate:.1f}m/s -- passing "
                f"{'left' if self.overtake_side > 0 else 'right'}.",
                throttle_duration_sec=1.0,
            )

        if not self.overtake_active:
            return None

        next_idx = (target_idx + 1) % self.num_waypoints if self.closed_loop \
            else min(target_idx + 1, self.num_waypoints - 1)
        return racing_math.lateral_offset_point(
            self.xy, target_idx, next_idx, self.overtake_side * self.overtake_lateral_offset)

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
