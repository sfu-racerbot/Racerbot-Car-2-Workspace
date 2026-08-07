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
               <odom_topic>  nav_msgs/Odometry                (measured speed)
               <joy_topic>   sensor_msgs/Joy                (deadman button)
  publishes:   <drive_topic> ackermann_msgs/AckermannDriveStamped
"""

import math
import sys

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import Joy, LaserScan

from pure_pursuit import live_tuning, racing_math


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

    def update(self, arc_length: float, now_sec: float, total_length: float = 0.0):
        if self.arc_length is not None and self.last_update_time is not None:
            dt = now_sec - self.last_update_time
            if dt > 1e-3:
                delta = arc_length - self.arc_length
                if total_length > 0.0:
                    delta = (delta + total_length / 2.0) % total_length - total_length / 2.0
                raw_rate = delta / dt
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

    def seconds_since_seen(self, now_sec: float) -> float:
        if self.last_update_time is None:
            return math.inf
        return now_sec - self.last_update_time

    def predicted_arc_length(self, now_sec: float, total_length: float):
        """Dead-reckoned arc length: last seen position advanced along the
        track at the smoothed progress rate. This is what makes finishing
        an overtake robust: alongside or just past the ego car, the
        opponent is *guaranteed* to leave the forward LIDAR cone, so "was
        it detected this tick" is precisely the wrong question -- "where
        must it be by now, given how it was moving" is the right one.
        """
        if self.arc_length is None or self.last_update_time is None:
            return None
        predicted = self.arc_length + self.progress_rate * (now_sec - self.last_update_time)
        if total_length > 0.0:
            predicted %= total_length
        return predicted


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
        self.declare_parameter('wait_for_waypoints', False)
        self.declare_parameter('closed_loop', True)
        self.declare_parameter('pose_topic', '/pf/viz/inferred_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('control_rate_hz', 40.0)
        self.declare_parameter('wheelbase', 0.324)
        self.declare_parameter('min_lookahead', 0.6)
        self.declare_parameter('max_lookahead', 1.5)
        self.declare_parameter('lookahead_speed_gain', 0.15)
        self.declare_parameter('nearest_search_window', 40)
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_steering_angle', 0.26)
        # Online dynamics protect corrections/overtakes that are sharper than
        # the offline racing line. Emergency stops bypass all command shaping.
        self.declare_parameter('max_lateral_accel', 2.5)
        self.declare_parameter('max_acceleration', 6.0)
        self.declare_parameter('max_braking_decel', 8.0)
        self.declare_parameter('max_steering_rate', 1.0)
        self.declare_parameter('command_slew_max_dt', 0.10)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('max_cross_track_error', 1.0)
        # Frozen-localization watchdog: only armed once odometry is sure the
        # car is really moving, so a legitimately parked car never trips it.
        self.declare_parameter('pose_frozen_timeout_sec', 0.5)
        self.declare_parameter('pose_frozen_min_speed', 0.3)
        self.declare_parameter('pose_frozen_min_travel', 0.05)
        # Rectangular collision envelope, mirroring gap_follow's.
        self.declare_parameter('car_width', 0.31)
        self.declare_parameter('car_length', 0.58)
        self.declare_parameter('emergency_stop_clearance', 0.05)
        self.declare_parameter('body_clearance_fov_deg', 180.0)
        self.declare_parameter('enable_lidar_safety', True)
        self.declare_parameter('safety_fov_deg', 60.0)
        self.declare_parameter('emergency_stop_distance', 0.4)
        self.declare_parameter('scan_timeout_sec', 0.5)
        # --- Deadman button (workspace policy, see docs/architecture.md) ---
        self.declare_parameter('enable_deadman', True)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('joy_timeout_sec', 0.5)
        # State changes are logged immediately. A slower periodic summary
        # explains steady-state path, speed, opponent, and LIDAR decisions
        # without printing at the full control_rate_hz.
        self.declare_parameter('decision_log_period_sec', 1.0)

        # --- Reactive avoidance (steer around something close, not just
        # stop, when there's room) ---
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('avoidance_fallback_trigger_distance', 0.7)
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.declare_parameter('avoidance_fov_deg', 60.0)
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
        self.declare_parameter('overtake_lateral_offset', 0.35)
        self.declare_parameter('overtake_lookahead_distance', 4.0)
        self.declare_parameter('overtake_max_blind_sec', 3.0)
        self.declare_parameter('laser_offset_x', 0.33)
        self.declare_parameter('laser_offset_y', 0.0)

        # --- Opponent detection mode: 'heuristic' (shape-based, no map
        # needed) or 'map' (ray-cast map subtraction via range_libc --
        # see map_subtraction.py). ---
        self.declare_parameter('opponent_detection_mode', 'map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_beam_step', 4)
        self.declare_parameter('map_subtraction_margin', 0.4)

        waypoints_file = str(self.get_parameter('waypoints_file').value)
        self.wait_for_waypoints = bool(self.get_parameter('wait_for_waypoints').value)
        self.closed_loop = bool(self.get_parameter('closed_loop').value)
        self.pose_topic = self.get_parameter('pose_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
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
        self.max_lateral_accel = float(
            self.get_parameter('max_lateral_accel').value)
        self.max_acceleration = float(
            self.get_parameter('max_acceleration').value)
        self.max_braking_decel = float(
            self.get_parameter('max_braking_decel').value)
        self.max_steering_rate = float(
            self.get_parameter('max_steering_rate').value)
        self.command_slew_max_dt = float(
            self.get_parameter('command_slew_max_dt').value)
        self.odom_timeout_sec = float(
            self.get_parameter('odom_timeout_sec').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.max_cross_track_error = float(self.get_parameter('max_cross_track_error').value)
        self.pose_frozen_timeout_sec = float(
            self.get_parameter('pose_frozen_timeout_sec').value)
        self.pose_frozen_min_speed = float(
            self.get_parameter('pose_frozen_min_speed').value)
        self.pose_frozen_min_travel = float(
            self.get_parameter('pose_frozen_min_travel').value)
        self.car_width = float(self.get_parameter('car_width').value)
        self.car_length = float(self.get_parameter('car_length').value)
        self.emergency_stop_clearance = float(
            self.get_parameter('emergency_stop_clearance').value)
        self.body_clearance_fov_deg = float(
            self.get_parameter('body_clearance_fov_deg').value)
        self.enable_lidar_safety = bool(self.get_parameter('enable_lidar_safety').value)
        self.safety_fov_deg = float(self.get_parameter('safety_fov_deg').value)
        self.emergency_stop_distance = float(self.get_parameter('emergency_stop_distance').value)
        self.scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        self.enable_deadman = bool(self.get_parameter('enable_deadman').value)
        self.joy_topic = self.get_parameter('joy_topic').value
        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.joy_timeout_sec = float(self.get_parameter('joy_timeout_sec').value)
        self.decision_log_period_sec = max(
            0.0, float(self.get_parameter('decision_log_period_sec').value))

        self.max_range = float(self.get_parameter('max_range').value)
        self.avoidance_fallback_trigger_distance = float(
            self.get_parameter('avoidance_fallback_trigger_distance').value)
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
        self.overtake_lookahead_distance = float(
            self.get_parameter('overtake_lookahead_distance').value)
        self.overtake_max_blind_sec = float(self.get_parameter('overtake_max_blind_sec').value)
        self.laser_offset_x = float(self.get_parameter('laser_offset_x').value)
        self.laser_offset_y = float(self.get_parameter('laser_offset_y').value)

        self.opponent_detection_mode = str(self.get_parameter('opponent_detection_mode').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.map_beam_step = max(1, int(self.get_parameter('map_beam_step').value))
        self.map_subtraction_margin = float(self.get_parameter('map_subtraction_margin').value)
        if self.opponent_detection_mode not in ('heuristic', 'map'):
            raise RuntimeError(
                f"pure_pursuit_node: opponent_detection_mode must be 'heuristic' or 'map', "
                f"got '{self.opponent_detection_mode}'."
            )
        dynamic_limits = (
            self.max_lateral_accel,
            self.max_acceleration,
            self.max_braking_decel,
            self.max_steering_rate,
            self.command_slew_max_dt,
            self.odom_timeout_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in dynamic_limits):
            raise RuntimeError(
                'pure_pursuit_node: dynamic steering/speed limits must be finite and positive.')
        if not (math.isfinite(self.min_speed) and math.isfinite(self.max_speed)
                and 0.0 <= self.min_speed <= self.max_speed):
            raise RuntimeError(
                'pure_pursuit_node: speed limits must satisfy 0 <= min_speed <= max_speed.')
        # Prove the footprint/LiDAR geometry is self-consistent now, at
        # startup, rather than letting vehicle_boundary_distances raise from
        # inside the control loop on the first scan -- an exception there
        # takes the node down mid-drive.
        try:
            racing_math.vehicle_boundary_distances(
                np.array([0.0]), self.car_width, self.car_length, self.wheelbase,
                self.laser_offset_x, self.laser_offset_y)
        except ValueError as exc:
            raise RuntimeError(
                f'pure_pursuit_node: vehicle footprint is unusable: {exc}') from exc
        if not (math.isfinite(self.emergency_stop_clearance)
                and self.emergency_stop_clearance >= 0.0):
            raise RuntimeError(
                'pure_pursuit_node: emergency_stop_clearance must be finite and non-negative.')
        if not all(math.isfinite(value) and value > 0.0 for value in (
                self.pose_frozen_timeout_sec, self.pose_frozen_min_speed,
                self.pose_frozen_min_travel)):
            raise RuntimeError(
                'pure_pursuit_node: pose_frozen_* limits must be finite and positive.')
        # The upper bound is not cosmetic. The online speed cap recovers
        # curvature as tan(delta)/wheelbase; at exactly pi/2 that is ~1e16 (the
        # car would be capped to a standstill forever) and past pi/2 tan goes
        # negative, silently inverting the curvature. Both fail quietly.
        if not (math.isfinite(self.max_steering_angle)
                and 0.0 < self.max_steering_angle < math.pi / 2.0):
            raise RuntimeError(
                'pure_pursuit_node: max_steering_angle must be finite and in '
                '(0, pi/2) radians.')
        if not (math.isfinite(self.overtake_lookahead_distance)
                and self.overtake_lookahead_distance >= self.max_lookahead):
            # A pass is a lateral *offset* from the line. Spread over a short
            # target it demands a curvature the online lateral-acceleration
            # cap then answers by braking, which stalls the pass instead of
            # completing it. The preview must be the longer horizon.
            raise RuntimeError(
                'pure_pursuit_node: overtake_lookahead_distance must be finite and '
                'at least max_lookahead.')

        # ------------------------------------------------------------------
        # Racing-line state. Normal launches still fail loudly if no file is
        # configured. The auto-map launch opts into wait_for_waypoints so it
        # can start this node stopped, generate a profile, and load it through
        # the runtime parameter callback without restarting the controller.
        # ------------------------------------------------------------------
        self.xy = np.empty((0, 2), dtype=np.float64)
        self.speed_profile = np.empty(0, dtype=np.float64)
        self.num_waypoints = 0
        self.seg_len = np.empty(0, dtype=np.float64)
        self.cumulative_arc_length = np.empty(0, dtype=np.float64)
        self.total_track_length = 0.0
        self.profile_ready = False
        if waypoints_file:
            self._activate_profile(waypoints_file)
        elif not self.wait_for_waypoints:
            raise RuntimeError(
                "pure_pursuit_node: the 'waypoints_file' parameter is not set. "
                "Point it at a profiled (x,y,speed) .csv produced by "
                "generate_velocity_profile -- see docs/racing-autonomy.md."
            )

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
        self.last_pose_stamp = None       # when localization computed the pose
        self._pose_reference_xy = None    # last pose that actually travelled
        self._pose_frozen_since = None    # moving-but-not-tracking window start
        self.prev_nearest_index = None

        self.last_scan = None
        self.last_scan_time = None
        self._boundary_distances = None   # per-beam body-edge distance cache
        self._boundary_geometry = None    # (beam count, angle_min, increment)

        self.current_speed = 0.0
        self.last_odom_time = None
        self.last_commanded_speed = 0.0
        # Basis the steering slew limiter rate-limits away from -- deliberately
        # NOT "the last steering angle published". See _stop().
        self.steering_basis = 0.0
        self.last_command_time = None

        # Deadman state: same pattern as gap_follow_node -- only ever
        # engages after a live /joy stream has actually shown the button
        # held, so the car never drives before that's been observed.
        self.deadman_held = False
        self.last_joy_time = None
        self.joy_button_available = False

        # Decision diagnostics: transitions are immediate, while an
        # unchanged state is rate-limited by decision_log_period_sec.
        self.last_decision_state = None
        self.last_decision_log_time = None
        self.last_opponent_status = 'opponent detection has not run yet'

        # Opponent tracking + overtake state -- see OpponentTracker above
        # and _update_opponent_and_overtake below.
        self.opponent = OpponentTracker(self.opponent_velocity_smoothing, self.opponent_lost_timeout_sec)
        self.overtake_active = False
        self.overtake_side = 1  # +1 = pass on the left, -1 = pass on the right

        # Map-subtraction detection state: stays None until a map arrives
        # (map_callback). Until then 'map' mode falls back to the
        # heuristic detector rather than racing blind.
        self.map_ray_caster = None

        # Live tuning: publish the catalogue of parameters this node will
        # accept changes to *while driving*, so the web dashboard can build
        # its panel from the node itself rather than from a hardcoded copy
        # of this list that would quietly rot. Read-only: the catalogue
        # describes what may change, and is not itself one of them.
        self._tunables = live_tuning.by_name(live_tuning.PURE_PURSUIT_TUNABLES)
        # Parameter-unit values (never the transformed attribute), so the
        # cross-parameter invariants always compare like with like.
        # Includes the read-only context values those invariants need.
        self._tunable_values = {
            name: self.get_parameter(name).value
            for name in tuple(self._tunables) + live_tuning.PURE_PURSUIT_INVARIANT_CONTEXT
        }
        self.declare_parameter(
            'live_tunable_spec',
            live_tuning.spec_json('pure_pursuit_node', live_tuning.PURE_PURSUIT_TUNABLES),
            ParameterDescriptor(
                read_only=True,
                description='JSON catalogue of the parameters this node can '
                            'apply live. Read by web_dashboard; see '
                            'pure_pursuit/live_tuning.py.'))

        # Accept a newly generated profile at runtime. This is registered
        # only after every state object it resets has been initialized.
        self.add_on_set_parameters_callback(self._parameter_callback)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        if self.enable_lidar_safety:
            self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        if self.enable_deadman:
            self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
        if self.opponent_detection_mode == 'map':
            # The map server publishes /map latched (transient local): a
            # matching durability here delivers the map even though this
            # node starts long after it was published once.
            map_qos = QoSProfile(depth=1,
                                 reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.map_sub = self.create_subscription(
                OccupancyGrid, self.map_topic, self.map_callback, map_qos)

        control_period_sec = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(control_period_sec, self.control_loop)

        if self.profile_ready:
            self._log_profile_ready(waypoints_file)
        else:
            self.get_logger().info(
                "pure_pursuit_node ready and stopped: waiting for an auto-generated "
                "waypoints_file profile.")

    def _activate_profile(self, path: str):
        """Load and atomically activate a profiled racing-line CSV."""
        try:
            xy, speed_profile = racing_math.load_profiled_csv(path)
        except (OSError, ValueError) as exc:
            raise RuntimeError(
                f"pure_pursuit_node: could not load waypoints_file '{path}': {exc}") from exc
        if len(xy) < 3:
            raise RuntimeError(
                f"pure_pursuit_node: '{path}' only has {len(xy)} waypoint(s), need at least 3.")

        seg_len = racing_math.compute_segment_lengths(xy, closed=self.closed_loop)
        cumulative = racing_math.compute_cumulative_arc_length(seg_len)
        total_length = float(np.sum(seg_len))
        if not np.isfinite(total_length) or total_length <= 0.0:
            raise RuntimeError(
                f"pure_pursuit_node: '{path}' has zero or invalid total path length.")

        # Assign only after all validation and derived values succeed, so a
        # bad runtime update cannot damage a previously active profile.
        self.xy = xy
        self.speed_profile = speed_profile
        self.num_waypoints = len(xy)
        self.seg_len = seg_len
        self.cumulative_arc_length = cumulative
        self.total_track_length = total_length
        self.prev_nearest_index = None
        if hasattr(self, 'opponent'):
            self.opponent = OpponentTracker(
                self.opponent_velocity_smoothing, self.opponent_lost_timeout_sec)
            self.overtake_active = False
        self.profile_ready = True

    def _log_profile_ready(self, path: str):
        self.get_logger().info(
            f"pure_pursuit_node ready: {self.num_waypoints} waypoints from '{path}' "
            f"({'closed loop' if self.closed_loop else 'open path'}), "
            f"speed profile {float(self.speed_profile.min()):.2f}-"
            f"{float(self.speed_profile.max()):.2f} m/s, control @ "
            f"{self.control_rate_hz:.0f}Hz, deadman button "
            f"{'ENABLED (LB must be held)' if self.enable_deadman else 'DISABLED'}, "
            f"obstacle avoidance {'ON' if self.enable_obstacle_avoidance else 'OFF'}, "
            f"opponent overtaking {'ON' if self.enable_opponent_overtake else 'OFF'}, "
            f"decision logs every {self.decision_log_period_sec:.1f}s "
            "(plus immediate state changes).")

    def _parameter_callback(self, parameters):
        """Runtime parameter changes: a new racing line, or a live tune.

        Anything else is *refused*, not ignored. This node caches every
        parameter on an attribute at startup (see __init__), so accepting a
        change it does not know how to apply would update the value the
        parameter server reports while the control loop kept driving on the
        old one -- a dashboard reading back "max_speed: 2.0" from a car
        still doing 4.0. Rejecting says so out loud instead. See
        live_tuning.py.
        """
        requested = {}
        for parameter in parameters:
            if parameter.name == 'waypoints_file':
                if parameter.type_ != Parameter.Type.STRING or not parameter.value:
                    return SetParametersResult(
                        successful=False,
                        reason="waypoints_file must be a non-empty profiled CSV path")
                try:
                    self._activate_profile(str(parameter.value))
                except RuntimeError as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
                self._log_profile_ready(str(parameter.value))
                continue
            requested[parameter.name] = parameter.value

        accepted, reason = live_tuning.review(
            self._tunables, requested, self._tunable_values,
            passthrough=('use_sim_time',),
            invariants=live_tuning.PURE_PURSUIT_INVARIANTS)
        if reason is not None:
            self.get_logger().warn(f'live tune rejected: {reason}')
            return SetParametersResult(successful=False, reason=reason)

        for name, value in accepted.items():
            tunable = self._tunables[name]
            previous = self._tunable_values[name]
            setattr(self, tunable.target_attr, tunable.store(value))
            self._tunable_values[name] = value
            self.get_logger().info(f'live tune: {name} {previous} -> {value}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------------
    # Sensor callbacks -- cache-only, see the comment in __init__ above.
    # ------------------------------------------------------------------------

    def pose_callback(self, msg: PoseStamped):
        self.car_x = msg.pose.position.x
        self.car_y = msg.pose.position.y
        q = msg.pose.orientation
        self.car_yaw = racing_math.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.last_pose_time = self.get_clock().now()

        # Freshness must come from when localization *computed* this pose,
        # not when the message showed up. auto_map_race_node republishes
        # SLAM's map->base_link TF at a fixed rate whatever its age, so a
        # frozen transform arrives just as punctually as a live one and
        # arrival time alone cannot tell them apart. The stamp is copied
        # from the TF and does carry the truth. Publishers that leave the
        # stamp at zero fall back to arrival time rather than tripping the
        # watchdog permanently.
        stamp = RclpyTime.from_msg(msg.header.stamp)
        self.last_pose_stamp = stamp if stamp.nanoseconds > 0 else None

        # Second, independent check on the same failure: a pose that does
        # not move while odometry says the car does. Reset the frozen-pose
        # window whenever the pose actually travels a meaningful distance.
        if (self._pose_reference_xy is None
                or math.hypot(self.car_x - self._pose_reference_xy[0],
                              self.car_y - self._pose_reference_xy[1])
                >= self.pose_frozen_min_travel):
            self._pose_reference_xy = (self.car_x, self.car_y)
            self._pose_frozen_since = None

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.current_speed = float(msg.twist.twist.linear.x)
        self.last_odom_time = self.get_clock().now()

    def _odom_fresh(self) -> bool:
        return (
            self.last_odom_time is not None
            and math.isfinite(self.current_speed)
            and self._seconds_since(self.last_odom_time) < self.odom_timeout_sec
        )

    def map_callback(self, msg: OccupancyGrid):
        # Import here, not at module top: range_libc is only needed in
        # 'map' mode, and the heuristic mode must keep working on a
        # machine where it isn't built.
        try:
            from pure_pursuit.map_subtraction import MapRayCaster
            self.map_ray_caster = MapRayCaster(msg, self.max_range)
        except Exception as exc:
            self.get_logger().error(
                f"Could not build the map ray caster ({exc}) -- opponent detection "
                f"stays on the heuristic fallback.")
            return
        self.get_logger().info(
            f"Map received ({msg.info.width}x{msg.info.height} @ "
            f"{msg.info.resolution:.3f}m/px) -- map-subtraction opponent detection active.")

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        self.joy_button_available = len(msg.buttons) > self.deadman_button
        if self.joy_button_available:
            self.deadman_held = bool(msg.buttons[self.deadman_button])
        else:
            self.deadman_held = False

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
        except Exception as exc:
            self._publish_drive(0.0, 0.0)
            self._log_decision(
                'control_exception',
                f"unhandled {type(exc).__name__}: {exc}; node will exit after publishing stop",
                0.0,
                0.0,
                level='error',
            )
            raise

    def _control_step(self):
        # --- Watchdog 0: LB deadman button (workspace policy -- see
        # docs/architecture.md). Checked first, ahead of every other
        # watchdog: no held button means no drive command, full stop,
        # regardless of how healthy localization/LIDAR/the racing line are. ---
        deadman_ok, stop_state, stop_detail = self._deadman_status()
        if not deadman_ok:
            self._stop(stop_state, stop_detail)
            return

        # Auto-map mode intentionally starts without a line. It remains at
        # a hard stop until the supervisor loads the generated profile.
        if not self.profile_ready:
            self._stop(
                'waiting_for_profile',
                "no racing-line profile is active; waiting for waypoints_file to be loaded",
            )
            return

        # --- Watchdog 1: localization must be alive and recent. ---
        if self.car_x is None:
            self._stop(
                'waiting_for_pose',
                f"no localization pose received on '{self.pose_topic}'",
            )
            return
        # Age from the pose's own stamp where there is one, so a stale
        # transform being faithfully republished at full rate is caught.
        # Arrival age still applies as well: it is the only thing that
        # notices the publisher itself going silent.
        arrival_age = self._seconds_since(self.last_pose_time)
        pose_age = arrival_age
        age_source = 'arrival'
        if self.last_pose_stamp is not None:
            stamp_age = self._seconds_since(self.last_pose_stamp)
            if stamp_age > pose_age:
                pose_age = stamp_age
                age_source = 'localization stamp'
        if pose_age > self.pose_timeout_sec:
            self._stop(
                'pose_stale',
                f"last localization pose is {pose_age:.2f}s old by {age_source} "
                f"(limit {self.pose_timeout_sec:.2f}s)",
            )
            return

        # --- Watchdog 1b: localization must actually be *tracking*. ---
        # A pose can be fresh by both measures above and still be wrong: if
        # SLAM stalls (a blocking map/pose-graph save, a lost scan match)
        # the transform stops advancing while the car keeps rolling, and
        # pure pursuit then steers from a position the car has already
        # left. Odometry is an independent witness to real motion, so
        # "odometry says we are moving, localization says we are not" is a
        # detectable contradiction -- and the one that put the car into a
        # wall on 2026-07-27 (see docs/troubleshooting.md).
        if self._odom_fresh() and abs(self.current_speed) >= self.pose_frozen_min_speed:
            if self._pose_frozen_since is None:
                self._pose_frozen_since = self.get_clock().now()
            else:
                frozen_for = self._seconds_since(self._pose_frozen_since)
                if frozen_for > self.pose_frozen_timeout_sec:
                    self._stop(
                        'pose_frozen',
                        f"odometry reports {abs(self.current_speed):.2f}m/s but the "
                        f"localization pose has not moved {self.pose_frozen_min_travel:.2f}m "
                        f"in {frozen_for:.2f}s (limit {self.pose_frozen_timeout_sec:.2f}s) -- "
                        "localization is not tracking the car",
                    )
                    return
        else:
            self._pose_frozen_since = None

        car_xy = (self.car_x, self.car_y)

        # --- Find where we are on the racing line. ---
        nearest_idx, cross_track_error = racing_math.find_nearest_index(
            self.xy, car_xy, closed=self.closed_loop,
            prev_index=self.prev_nearest_index, search_window=self.nearest_search_window,
        )
        if cross_track_error > self.max_cross_track_error:
            # The windowed search only looks near last tick's position, so
            # after a legitimate localization jump (the particle filter
            # re-converging, the car re-placed after a stop) it can keep
            # returning a far-away waypoint forever. Before declaring the
            # car lost, re-search the whole line once -- if the new pose
            # is actually near the track somewhere else, lock onto that
            # and keep driving instead of stopping until a node restart.
            nearest_idx, cross_track_error = racing_math.find_nearest_index(
                self.xy, car_xy, closed=self.closed_loop, prev_index=None)

        # --- Watchdog 2: are we still actually near the racing line? ---
        # A large cross-track error means the car is lost, kidnapped, or
        # localization has diverged -- driving the pure pursuit geometry
        # anyway would aim the car at a point that may bear no relation to
        # where it actually is.
        if cross_track_error > self.max_cross_track_error:
            # Stay un-anchored while lost so recovery next tick starts
            # from a clean global search, not this bad index.
            self.prev_nearest_index = None
            self._stop(
                'off_racing_line',
                f"cross-track error {cross_track_error:.2f}m exceeds "
                f"{self.max_cross_track_error:.2f}m even after a full-line search",
            )
            return
        self.prev_nearest_index = nearest_idx

        # --- Steering: adaptive lookahead + pure pursuit geometry. ---
        # Use the speed *at the car's current position on the line* (not
        # the target's) to size the lookahead -- lookahead should reflect
        # how fast we're going right now, not how fast we will be going
        # once we arrive at the target point.
        speed_here = float(self.speed_profile[nearest_idx])
        if self._odom_fresh():
            lookahead_speed = abs(self.current_speed)
            lookahead_speed_source = 'fresh odometry'
        else:
            lookahead_speed = speed_here
            lookahead_speed_source = 'profile fallback'
        lookahead = racing_math.adaptive_lookahead(
            lookahead_speed,
            self.lookahead_speed_gain,
            self.min_lookahead,
            self.max_lookahead,
        )
        target_idx = racing_math.find_lookahead_index(
            self.seg_len, nearest_idx, lookahead, closed=self.closed_loop)
        target_x, target_y = self.xy[target_idx]

        dx = target_x - self.car_x
        dy = target_y - self.car_y
        x_body, y_body = racing_math.world_to_body(dx, dy, self.car_yaw)
        kappa = racing_math.steering_arc_curvature(x_body, y_body)
        steering_unclipped = racing_math.steering_from_curvature(kappa, self.wheelbase)
        steering_angle = steering_unclipped
        steering_angle = float(np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle))

        # --- Speed: the profiled speed for where the car is right now. ---
        speed_cmd = float(np.clip(speed_here, self.min_speed, self.max_speed))
        hard_speed_cap = self.max_speed
        decision_state = 'pure_pursuit'

        # --- Opponent tracking + overtaking: reconsiders the steering
        # *target* (not yet the final command) if another car has been
        # spotted and this car is closing in on it. Requires the reactive
        # safety net to be enabled too -- overtaking is a more assertive
        # behavior layered on top of it, not a substitute for it. ---
        if self.enable_lidar_safety and self.enable_opponent_overtake:
            overtake_target = self._update_opponent_and_overtake(nearest_idx)
            if overtake_target is not None:
                target_x, target_y = overtake_target
                dx = target_x - self.car_x
                dy = target_y - self.car_y
                x_body, y_body = racing_math.world_to_body(dx, dy, self.car_yaw)
                kappa = racing_math.steering_arc_curvature(x_body, y_body)
                steering_unclipped = racing_math.steering_from_curvature(kappa, self.wheelbase)
                steering_angle = steering_unclipped
                steering_angle = float(np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle))
                decision_state = (
                    'overtake_left' if self.overtake_side > 0 else 'overtake_right')

        clipped_text = (
            f", steering clipped from {steering_unclipped:+.3f}rad"
            if not math.isclose(steering_angle, steering_unclipped) else "")
        decision_detail = (
            f"pose=({self.car_x:.2f},{self.car_y:.2f},{math.degrees(self.car_yaw):+.1f}deg), "
            f"nearest waypoint={nearest_idx}, steering target={target_idx}, "
            f"cross-track={cross_track_error:.2f}m, lookahead={lookahead:.2f}m "
            f"from {lookahead_speed_source}={lookahead_speed:.2f}m/s, "
            f"profile speed={speed_here:.2f}m/s, path curvature={kappa:+.3f}/m"
            f"{clipped_text}")
        if self.enable_lidar_safety and self.enable_opponent_overtake:
            decision_detail += f"; {self.last_opponent_status}"

        # --- Reactive safety net: independent of everything above, and
        # always gets the final say regardless of the racing line or any
        # overtake in progress. ---
        if self.enable_lidar_safety:
            # Emergency stopping always remains active. During a committed
            # pass, however, generic gap avoidance must not cap us below the
            # opponent's speed and make the pass mathematically impossible.
            steering_override, speed_override, reactive_state, reactive_detail = self._reactive_override(
                allow_avoidance=not self.overtake_active)
            if steering_override is not None:
                steering_angle = steering_override
            if speed_override is not None:
                speed_cmd = speed_override
                hard_speed_cap = min(hard_speed_cap, speed_override)
            if reactive_state is not None:
                decision_state = reactive_state
                decision_detail = f"{reactive_detail}; base plan: {decision_detail}"
            else:
                decision_detail += f"; {reactive_detail}"

        desired_steering = steering_angle
        desired_speed = speed_cmd
        if desired_speed <= 0.0 or hard_speed_cap <= 0.0:
            # Blind/stale/too-close safety overrides remain immediate. In
            # particular, never rate-limit a stop into a nonzero command.
            self._publish_drive(desired_steering, 0.0)
            speed_cmd = 0.0
            decision_detail += '; immediate safety stop bypassed command shaping'
        else:
            (now, steering_angle, speed_cmd, online_curvature,
             online_curve_speed, command_dt) = self._shape_normal_command(
                desired_steering, desired_speed, hard_speed_cap)
            self._publish_drive(steering_angle, speed_cmd, now=now)
            shaping_text = (
                f", steering shaped from {desired_steering:+.3f}rad"
                if not math.isclose(steering_angle, desired_steering) else "")
            speed_shape_text = (
                f", speed shaped/capped from {desired_speed:.2f}m/s"
                if not math.isclose(speed_cmd, desired_speed) else "")
            decision_detail += (
                f"; online command curvature={online_curvature:.3f}/m, "
                f"curve speed cap={online_curve_speed:.2f}m/s, "
                f"command dt={command_dt:.3f}s{shaping_text}{speed_shape_text}")

        self._log_decision(
            decision_state, decision_detail, steering_angle, speed_cmd)

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
        """Closest *valid* (finite, positive, >= the sensor's own
        range_min) reading within a forward cone, or +inf if the cone is
        empty or nothing valid was seen. Readings below range_min are the
        sensor's "invalid" encoding, not a real 4cm obstacle -- counting
        them would emergency-stop the car on scan noise. Used for yes/no
        distance checks (hard-stop, avoidance trigger) that only need to
        know *how close*, not *which beam*.
        """
        lo_idx, hi_idx = self._fov_indices(scan, fov_deg)
        if hi_idx <= lo_idx:
            return math.inf
        window = np.array(scan.ranges[lo_idx:hi_idx + 1], dtype=np.float64)
        window = window[np.isfinite(window) & (window > 0.0) & (window >= scan.range_min)]
        if window.size == 0:
            return math.inf
        return float(np.min(window))

    def _footprint_clearance(self, scan: LaserScan) -> float:
        """Smallest distance from the car's rectangular body to any valid
        return, in any direction. Unlike a forward-cone minimum this sees a
        wall the car is alongside. Readings below the sensor's own
        range_min are its "invalid" encoding, not real contact.
        """
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        geometry = (len(ranges), scan.angle_min, scan.angle_increment)
        if self._boundary_geometry != geometry:
            # Scan geometry is fixed for a given LiDAR, so the per-beam
            # body-edge distances are computed once, not every tick.
            angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
            self._boundary_distances = racing_math.vehicle_boundary_distances(
                angles, self.car_width, self.car_length, self.wheelbase,
                self.laser_offset_x, self.laser_offset_y)
            self._boundary_geometry = geometry

        # Restricted to a forward window, exactly as gap_follow does before
        # its own footprint check, and for a concrete reason: this Hokuyo
        # sweeps 270deg, so the rearmost beams look back along the car and
        # hit its own chassis. Those returns sit *inside* the footprint by
        # construction, giving a permanently negative clearance that pins
        # the car at a standstill -- observed as a steady -0.110m on
        # 2026-07-27. 180deg still spans both flanks (+/-90deg), which is
        # the whole point of measuring clearance from the body rather than
        # from a forward cone.
        lo_idx, hi_idx = self._fov_indices(scan, self.body_clearance_fov_deg)
        if hi_idx <= lo_idx:
            return math.inf
        window = ranges[lo_idx:hi_idx + 1]
        boundaries = self._boundary_distances[lo_idx:hi_idx + 1]
        valid = np.isfinite(window) & (window > 0.0) & (window >= scan.range_min)
        return racing_math.minimum_footprint_clearance(window, valid, boundaries)

    def _dynamic_closest_in_cone(self, scan: LaserScan, fov_deg: float):
        """Closest scan return not explained by the static map.

        Returns None while map subtraction is unavailable, allowing the
        caller to use the conservative raw-scan fallback threshold.
        """
        if (self.opponent_detection_mode != 'map' or self.map_ray_caster is None
                or self.car_x is None or self.car_y is None or self.car_yaw is None):
            return None

        step = self.map_beam_step
        all_ranges = np.asarray(scan.ranges, dtype=np.float64)
        sample_indices = np.arange(len(all_ranges))[::step]
        measured = all_ranges[::step]
        beam_angles = scan.angle_min + sample_indices * scan.angle_increment

        cos_yaw, sin_yaw = math.cos(self.car_yaw), math.sin(self.car_yaw)
        laser_x = self.car_x + self.laser_offset_x * cos_yaw - self.laser_offset_y * sin_yaw
        laser_y = self.car_y + self.laser_offset_x * sin_yaw + self.laser_offset_y * cos_yaw
        expected = self.map_ray_caster.expected_ranges(
            laser_x, laser_y, self.car_yaw, beam_angles)
        dynamic = racing_math.dynamic_beam_mask(
            measured, expected, self.map_subtraction_margin, scan.range_min)
        in_cone = np.abs(beam_angles) <= math.radians(fov_deg) / 2.0
        candidates = measured[dynamic & in_cone]
        if candidates.size == 0:
            return math.inf
        return float(np.min(candidates))

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

    def _reactive_override(self, allow_avoidance: bool = True):
        """The reactive LIDAR safety net -- independent of the racing
        line and the overtake logic above, and always has the final say.
        Returns (steering_override, speed_override, decision_state,
        diagnostic_detail). An override is None where that part of the
        already-computed plan should be left alone; decision_state is None
        when the racing-line/overtake plan remains in control.

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
        if self.last_scan is None:
            return (
                None,
                0.0,
                'lidar_scan_missing',
                f"no LaserScan received on '{self.scan_topic}', so the LIDAR safety net is blind",
            )

        scan_age = self._seconds_since(self.last_scan_time)
        if scan_age > self.scan_timeout_sec:
            return (
                None,
                0.0,
                'lidar_scan_stale',
                f"last LaserScan is {scan_age:.2f}s old (limit {self.scan_timeout_sec:.2f}s)",
            )

        scan = self.last_scan

        # --- Tier 0: is any part of the *body* about to touch something? ---
        # Ahead of the forward-cone check below, because that check is a
        # minimum range over a 60deg cone pointed straight ahead and is
        # structurally blind to a wall alongside the car: a beam pointing
        # sideways at a wall the bodywork is 1.5cm from still reports a
        # perfectly comfortable range, and no cone minimum will ever see
        # it. On 2026-07-27 that is exactly how the car accelerated into a
        # wall it was already touching while its safety net logged "LIDAR
        # clear". gap_follow has had this footprint check all along; the
        # race controller needs it just as much.
        body_clearance = self._footprint_clearance(scan)
        if body_clearance <= self.emergency_stop_clearance:
            return (
                None,
                0.0,
                'body_contact',
                f"minimum clearance from the car body is {body_clearance:.3f}m, at or "
                f"below the {self.emergency_stop_clearance:.3f}m contact threshold "
                "(measured over every beam, not just the forward cone)",
            )

        emergency_closest = self._closest_in_cone(scan, self.safety_fov_deg)
        if emergency_closest < self.emergency_stop_distance:
            return (
                None,
                0.0,
                'emergency_obstacle',
                f"closest valid return in the {self.safety_fov_deg:.1f}deg safety cone is "
                f"{emergency_closest:.2f}m, inside the "
                f"{self.emergency_stop_distance:.2f}m emergency threshold",
            )

        if not self.enable_obstacle_avoidance:
            return (
                None,
                None,
                None,
                f"LIDAR hard-stop cone clear (closest={emergency_closest:.2f}m); "
                "generic obstacle avoidance is disabled",
            )
        if not allow_avoidance:
            return (
                None,
                None,
                None,
                f"LIDAR hard-stop cone clear (closest={emergency_closest:.2f}m); "
                "generic avoidance suppressed during the committed overtake",
            )

        dynamic_closest = self._dynamic_closest_in_cone(scan, self.avoidance_fov_deg)
        if dynamic_closest is None:
            # Before a map arrives (or in heuristic mode), track walls are
            # indistinguishable from obstacles. The validated shorter
            # fallback threshold avoids constantly reacting to normal walls.
            closest = self._closest_in_cone(scan, self.avoidance_fov_deg)
            trigger_distance = self.avoidance_fallback_trigger_distance
            distance_source = 'raw scan (map subtraction unavailable)'
        else:
            closest = dynamic_closest
            trigger_distance = self.avoidance_trigger_distance
            distance_source = 'map-unexplained object'
        if closest >= trigger_distance:
            return (
                None,
                None,
                None,
                f"LIDAR clear: closest {distance_source}={closest:.2f}m, "
                f"avoidance trigger={trigger_distance:.2f}m",
            )

        lo_idx, hi_idx = self._fov_indices(scan, self.avoidance_fov_deg)
        window = self._sanitized_window(scan, lo_idx, hi_idx)
        if window.size == 0:
            return (
                None,
                0.0,
                'avoidance_scan_empty',
                f"{distance_source} at {closest:.2f}m triggered avoidance, but the "
                f"{self.avoidance_fov_deg:.1f}deg scan window is empty",
            )

        gap_start, gap_end = racing_math.find_best_gap(window, self.avoidance_min_gap_distance)
        if gap_start is None:
            return (
                None,
                0.0,
                'avoidance_boxed_in',
                f"{distance_source} at {closest:.2f}m triggered avoidance, but no gap "
                f"is deeper than {self.avoidance_min_gap_distance:.2f}m",
            )

        target_idx = lo_idx + (gap_start + gap_end) // 2
        angle = scan.angle_min + target_idx * scan.angle_increment
        angle = float(np.clip(angle, -self.max_steering_angle, self.max_steering_angle))
        gap_lo_angle = scan.angle_min + (lo_idx + gap_start) * scan.angle_increment
        gap_hi_angle = scan.angle_min + (lo_idx + gap_end) * scan.angle_increment
        return (
            angle,
            self.avoidance_speed,
            'lidar_avoidance',
            f"{distance_source} at {closest:.2f}m is inside the "
            f"{trigger_distance:.2f}m avoidance trigger; selected gap "
            f"{math.degrees(gap_lo_angle):+.1f}deg to "
            f"{math.degrees(gap_hi_angle):+.1f}deg and capped speed at "
            f"{self.avoidance_speed:.2f}m/s",
        )

    def _detect_opponent(self, scan: LaserScan, ranges: np.ndarray,
                         laser_world_x: float, laser_world_y: float):
        """One opponent detection, by whichever detector is configured
        and available: map subtraction (ray-cast what the LIDAR *should*
        see from the current pose, anything meaningfully shorter is not
        in the map -- see map_subtraction.py / racing_math's
        detect_dynamic_cluster) when mode is 'map' and a map has arrived,
        else the shape-based heuristic. Returns
        (start_idx, end_idx, centroid_range, centroid_angle) with indices
        into the *full* scan either way, or None. Both paths share the
        same forward-FOV gate: a car far off to the side is not one we
        are racing against right now.
        """
        if self.opponent_detection_mode == 'map' and self.map_ray_caster is not None:
            # Every map_beam_step-th beam is plenty of resolution for a
            # car-sized object, and keeps the per-tick ray-cast cost tiny.
            # The *raw* ranges go in (not the sanitized copy) -- the
            # dynamic-beam mask treats NaN/inf/sub-range_min as unknown,
            # and a sanitized NaN->0.0 would look like a phantom object
            # closer than the map predicts.
            step = self.map_beam_step
            measured = np.array(scan.ranges, dtype=np.float64)[::step]
            beam_angles = scan.angle_min + np.arange(len(scan.ranges))[::step] * scan.angle_increment
            expected = self.map_ray_caster.expected_ranges(
                laser_world_x, laser_world_y, self.car_yaw, beam_angles)
            candidate = racing_math.detect_dynamic_cluster(
                measured, expected, scan.angle_min, scan.angle_increment * step,
                self.map_subtraction_margin,
                self.opponent_min_width, self.opponent_max_width,
                self.opponent_engagement_range, range_min=scan.range_min,
                cluster_gap_threshold=self.opponent_cluster_gap)
            if candidate is not None:
                # Indices come back in downsampled space -- map them onto
                # the full scan for pick_pass_side and friends.
                candidate = (candidate[0] * step, candidate[1] * step,
                             candidate[2], candidate[3])
        else:
            candidate = racing_math.detect_opponent_cluster(
                ranges, scan.angle_min, scan.angle_increment, self.max_range,
                self.opponent_min_width, self.opponent_max_width,
                self.opponent_engagement_range, self.opponent_cluster_gap,
                self.opponent_open_side_margin)

        if candidate is None:
            return None
        half_fov = math.radians(self.avoidance_fov_deg) / 2.0
        if not (-half_fov <= candidate[3] <= half_fov):
            return None
        return candidate

    def _update_opponent_and_overtake(self, nearest_idx: int):
        """Look for another car in the live scan, track its progress along
        the racing line, and decide whether to start, continue, or end an
        overtake. Returns a (x, y) world-frame point to steer at instead
        of the normal Pure Pursuit target if an overtake is in progress,
        or None to leave the plan alone. See "Racing against opponents" in
        docs/racing-autonomy.md for the full strategy this implements.
        """
        now_sec = self.get_clock().now().nanoseconds / 1e9
        detector_name = (
            'map subtraction'
            if self.opponent_detection_mode == 'map' and self.map_ray_caster is not None
            else 'shape heuristic fallback')
        self.last_opponent_status = f"opponent: none detected by {detector_name}"

        # The LIDAR's own map-frame position -- needed both as the origin
        # for map-subtraction ray casting and to place a detected cluster
        # in the map frame below, so computed once up front.
        cos_yaw, sin_yaw = math.cos(self.car_yaw), math.sin(self.car_yaw)
        laser_world_x = self.car_x + self.laser_offset_x * cos_yaw - self.laser_offset_y * sin_yaw
        laser_world_y = self.car_y + self.laser_offset_x * sin_yaw + self.laser_offset_y * cos_yaw

        detection = None
        ranges = None
        if self.last_scan is not None and self._seconds_since(self.last_scan_time) <= self.scan_timeout_sec:
            scan = self.last_scan
            ranges = np.nan_to_num(np.array(scan.ranges, dtype=np.float64),
                                    nan=0.0, posinf=self.max_range, neginf=0.0)
            ranges = np.clip(ranges, 0.0, self.max_range)
            detection = self._detect_opponent(scan, ranges, laser_world_x, laser_world_y)
        else:
            self.last_opponent_status = 'opponent: not checked because the LaserScan is missing/stale'

        start_idx = end_idx = None
        if detection is not None:
            start_idx, end_idx, centroid_range, centroid_angle = detection
            detection_summary = (
                f"opponent detected at {centroid_range:.2f}m, "
                f"bearing={math.degrees(centroid_angle):+.1f}deg by {detector_name}")
            self.last_opponent_status = detection_summary

            # Where that cluster actually is in the map frame, so its
            # progress along the racing line can be measured the same way
            # the ego car's own position is.
            world_angle = self.car_yaw + centroid_angle
            opponent_x = laser_world_x + centroid_range * math.cos(world_angle)
            opponent_y = laser_world_y + centroid_range * math.sin(world_angle)

            opp_idx, _ = racing_math.find_nearest_index(
                self.xy, (opponent_x, opponent_y), closed=self.closed_loop,
                prev_index=self.opponent.prev_nearest_index, search_window=self.nearest_search_window)
            self.opponent.prev_nearest_index = opp_idx
            self.opponent.update(
                float(self.cumulative_arc_length[opp_idx]), now_sec, self.total_track_length)

        ego_arc_length = float(self.cumulative_arc_length[nearest_idx])

        if not self.overtake_active:
            # Not committed to anything yet: only a *recent* sighting is
            # worth reacting to, and starting a new overtake additionally
            # needs this tick's actual scan -- picking which side to pass
            # on reads directly from it.
            if self.opponent.arc_length is None or not self.opponent.is_fresh(now_sec):
                return None
            gap_ahead = racing_math.track_progress_gap(
                ego_arc_length, self.opponent.arc_length, self.total_track_length)
            ego_speed = float(self.speed_profile[nearest_idx])
            closing_rate = ego_speed - self.opponent.progress_rate
            closing_fast_enough = closing_rate > self.overtake_closing_margin
            if detection is None or gap_ahead > self.overtake_trigger_gap or not closing_fast_enough:
                reasons = []
                if detection is None:
                    reasons.append('not present in the current scan')
                if gap_ahead > self.overtake_trigger_gap:
                    reasons.append(
                        f"track gap {gap_ahead:.2f}m > {self.overtake_trigger_gap:.2f}m trigger")
                if not closing_fast_enough:
                    reasons.append(
                        f"closing rate {closing_rate:.2f}m/s <= "
                        f"{self.overtake_closing_margin:.2f}m/s trigger")
                self.last_opponent_status = (
                    f"opponent tracked: gap={gap_ahead:.2f}m, "
                    f"closing={closing_rate:.2f}m/s; no overtake because "
                    + ', '.join(reasons))
                return None
            self.overtake_active = True
            self.overtake_side = racing_math.pick_pass_side(ranges, start_idx, end_idx)
            self.last_opponent_status = (
                f"overtake active: opponent gap={gap_ahead:.2f}m, "
                f"closing={closing_rate:.2f}m/s, passing "
                f"{'left' if self.overtake_side > 0 else 'right'}")
            self.get_logger().info(
                f"overtake: opponent {gap_ahead:.1f}m ahead on track, closing at "
                f"{closing_rate:.1f}m/s -- passing "
                f"{'left' if self.overtake_side > 0 else 'right'}.",
                throttle_duration_sec=1.0,
            )
        else:
            # Mid-pass. Alongside or just past the opponent it is
            # *guaranteed* to leave the forward LIDAR cone, so "no fresh
            # detection" must not cancel the pass and snap the steering
            # back onto a racing line the opponent may still be occupying.
            # Instead, dead-reckon where the opponent must be by now
            # (last seen position advanced at its tracked progress rate)
            # and finish the pass on *ego progress*: it's over only once
            # the ego car is far enough past that predicted position.
            blind_sec = self.opponent.seconds_since_seen(now_sec)
            if blind_sec > self.overtake_max_blind_sec:
                # Blind for so long the dead-reckoned position is no
                # longer trustworthy -- give up the offset line rather
                # than keep driving it on a stale guess. (The reactive
                # safety net still covers whatever is actually out there.)
                self.get_logger().warn(
                    f"overtake: opponent unseen for {blind_sec:.1f}s -- abandoning the pass.",
                    throttle_duration_sec=1.0,
                )
                self.overtake_active = False
                self.last_opponent_status = (
                    f"overtake abandoned: opponent unseen for {blind_sec:.2f}s "
                    f"> {self.overtake_max_blind_sec:.2f}s limit")
                return None
            predicted_arc = self.opponent.predicted_arc_length(now_sec, self.total_track_length)
            # Signed lead, not the wrapped gap. track_progress_gap can only
            # say "somewhere ahead within one lap", so the natural-looking
            # test `gap > total - clear_margin` actually means "at most
            # clear_margin past" -- satisfied the moment the ego's nose edges
            # in front, with the cars still overlapping side by side. The car
            # then cut straight back onto the racing line and sideswiped the
            # opponent (reproduced in simulation, contact 0.45 s after the
            # pass was declared complete). See racing_math.track_lead_distance.
            lead = racing_math.track_lead_distance(
                ego_arc_length, predicted_arc, self.total_track_length)
            if lead >= self.overtake_clear_margin:
                self.get_logger().info("overtake: clear of the opponent -- back to the racing line.",
                                       throttle_duration_sec=1.0)
                self.overtake_active = False
                self.last_opponent_status = (
                    f"overtake complete: ego is {lead:.2f}m past the predicted "
                    f"opponent position (>= {self.overtake_clear_margin:.2f}m)")
                return None
            self.last_opponent_status = (
                f"overtake active: passing {'left' if self.overtake_side > 0 else 'right'}, "
                f"opponent unseen for {blind_sec:.2f}s, ego lead={lead:.2f}m")

        # Deliberately *not* the normal steering target: offsetting a target
        # only max_lookahead away demands a sharp turn to reach the passing
        # line, which the online curvature speed cap answers by braking --
        # slowing exactly when the pass needs speed. Previewing further ahead
        # spreads the same lateral offset over a gentler arc. See
        # docs/racing-autonomy.md's "Racing against opponents".
        overtake_idx = racing_math.find_lookahead_index(
            self.seg_len, nearest_idx, self.overtake_lookahead_distance,
            closed=self.closed_loop)
        next_idx = (overtake_idx + 1) % self.num_waypoints if self.closed_loop \
            else min(overtake_idx + 1, self.num_waypoints - 1)
        return racing_math.lateral_offset_point(
            self.xy, overtake_idx, next_idx, self.overtake_side * self.overtake_lateral_offset)

    def _seconds_since(self, stamp) -> float:
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _command_timing(self):
        """Return one bounded command interval and the clock sample behind it."""
        now = self.get_clock().now()
        if self.last_command_time is None:
            return now, self.command_slew_max_dt
        elapsed = (now - self.last_command_time).nanoseconds / 1e9
        return now, min(self.command_slew_max_dt, max(0.0, elapsed))

    def _shape_normal_command(self, desired_steering: float,
                              desired_speed: float, hard_speed_cap: float):
        """Shape an ordinary drive command; emergency stops never enter here."""
        now, command_dt = self._command_timing()
        steering = racing_math.slew_rate_limit(
            desired_steering,
            self.steering_basis,
            command_dt,
            self.max_steering_rate,
        )
        self.steering_basis = steering
        desired_curvature = math.tan(desired_steering) / self.wheelbase
        commanded_curvature = math.tan(steering) / self.wheelbase
        online_curvature = max(abs(desired_curvature), abs(commanded_curvature))
        curve_speed = racing_math.curvature_speed_limit(
            online_curvature, self.max_lateral_accel, self.max_speed)
        speed_target = min(desired_speed, curve_speed, hard_speed_cap)
        # Ramp from where the car actually *is*, not from the last command.
        # A ceiling (avoidance, curvature) drops the command instantly while
        # the car is still travelling at nearly its old speed; ramping from
        # that command would then hold the throttle far below the car's real
        # speed for the whole climb back -- braking a car that never slowed.
        # Fresh odometry closes that gap. It only ever *raises* the basis, is
        # clamped to max_speed so a wild reading cannot inflate it, and every
        # ceiling below still applies, so this cannot outrun a safety limit.
        ramp_basis = self.last_commanded_speed
        if self._odom_fresh():
            ramp_basis = max(
                ramp_basis, min(abs(self.current_speed), self.max_speed))
        speed = racing_math.slew_rate_limit(
            speed_target,
            ramp_basis,
            command_dt,
            self.max_acceleration,
            self.max_braking_decel,
        )
        # A runtime turn or reactive override is a safety ceiling, not merely a
        # comfort target. It may lower speed immediately; only acceleration is
        # always gradual.
        speed = min(speed, curve_speed, hard_speed_cap)
        return now, steering, max(0.0, speed), online_curvature, curve_speed, command_dt

    def _publish_drive(self, steering_angle: float, speed: float, now=None):
        if now is None:
            now = self.get_clock().now()
        if math.isfinite(speed) and math.isfinite(steering_angle):
            # The steering slew basis is deliberately not updated here --
            # _shape_normal_command() and _stop() each own how it evolves.
            self.last_commanded_speed = float(speed)
            self.last_command_time = now
        msg = AckermannDriveStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

    def _stop(self, state: str, detail: str):
        now, command_dt = self._command_timing()
        # Decay the steering slew basis at the rate the rack can actually
        # travel rather than snapping it to 0. Snapping means one transient
        # stop caps the *next* steering command at max_steering_rate*dt, and
        # a stop landing between every pair of control ticks pins steering
        # near zero indefinitely -- while the speed ramp, which reads measured
        # speed rather than the last command, recovers in full. That
        # asymmetry drove gap_follow into a wall on 2026-07-27; the same
        # shaping pattern lives here, so it gets the same treatment.
        self.steering_basis = racing_math.slew_rate_limit(
            0.0, self.steering_basis, command_dt, self.max_steering_rate)
        self._publish_drive(0.0, 0.0, now=now)
        self._log_decision(state, detail, 0.0, 0.0)

    def _log_decision(self, state: str, detail: str, steering_angle: float,
                      speed: float, level: str = None):
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
        message = (
            f"{'STOP' if stopped else 'DRIVE'} [{state}] {detail}; "
            f"command: steering={steering_angle:+.3f}rad, speed={speed:.2f}m/s")
        if level == 'error':
            self.get_logger().error(message)
        elif stopped:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)
        self.last_decision_state = state
        self.last_decision_log_time = now


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
        # The default SIGINT handler may already have shut the context down.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
