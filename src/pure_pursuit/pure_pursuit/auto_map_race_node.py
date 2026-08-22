"""Autonomously map a closed course, generate a raceline, then race it.

This node is deliberately a supervisor rather than another controller. During
mapping it forwards gap_follow commands; after a closed lap (two by default,
so SLAM loop closure has settled before the recorded lap) it profiles the path,
loads it into pure_pursuit_node through a runtime parameter, and forwards that
controller instead. It also republishes SLAM's map->base_link transform as the
PoseStamped input pure pursuit expects.
"""

import math
import os
from pathlib import Path
import shutil
import subprocess
from time import strftime

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
from pure_pursuit import occupancy_map, racing_math, recorded_path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from tf2_ros import Buffer, TransformException, TransformListener


def _die_with_parent():
    """Ask the kernel to kill this child when its parent dies.

    Runs in the child between fork and exec. The localization stack spawned
    for the racing handover is a whole `ros2 launch` tree, and if this
    supervisor is SIGKILLed -- or the terminal running it disappears -- no
    Python cleanup gets the chance to run. Without this the map_server and
    particle filter survive, keep the `/map_server/map` service name, and
    silently break the *next* run's handover, which is a miserable thing to
    diagnose a week later.

    Linux-only (this car is a Jetson). Failure to set it is not worth
    aborting a run over: the explicit terminate on shutdown still covers
    every ordinary exit.
    """
    try:
        import ctypes
        import signal as _signal
        PR_SET_PDEATHSIG = 1
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(
            PR_SET_PDEATHSIG, _signal.SIGTERM, 0, 0, 0)
    except Exception:  # noqa: BLE001 - best effort, never block the spawn
        pass


def angle_difference(a: float, b: float) -> float:
    """Smallest signed angular difference a-b, in radians."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class LapRecorder:
    """Distance-sampled map-frame path with conservative loop detection.

    Two things here are not obvious and both were bugs:

    **A SLAM correction is not motion.** slam_toolbox re-optimises its pose
    graph continuously, and each correction moves `map->odom` -- so the car's
    map-frame pose moves without the car moving. Recorded verbatim those
    corrections become geometry, and on this car's own recorded laps they
    were most of it: a median 8.8-15.5 degrees of heading change between
    consecutive 0.15m samples. `_reanchor` instead applies the correction to
    the *already recorded* points, so the stored shape stays rigid relative
    to the car and stays valid in the new frame. That is what a correction
    actually means: the whole map moved, including the part already driven.

    **A lap is one revolution, not a fixed number of metres.**
    `minimum_lap_distance` cannot know how big the course is, and when it is
    set longer than the course -- 20m against this car's ~15m room -- the
    closure gate cannot open until the car has been round *twice*. All three
    laps this car has recorded are two revolutions. Accumulated yaw closes
    that hole: by the turning-tangent theorem one lap of a closed circuit is
    2*pi of turning whatever its size.

    **Turning is counted in the odom frame where one is available.** The
    reanchor above deliberately skips the yaw change on a correction tick,
    because in the map frame that change is the map's and not the car's --
    but the car's own real turning during that tick is thrown away with it.
    The 2026-08-19 run absorbed 106 corrections in a single 136s lap and
    measured 335deg of turning for one genuine revolution, a 35deg margin
    against a 300deg gate. Odometry is not re-optimised, so `odom_yaw`
    accumulates the real turning and the gate stops depending on how busy
    the pose graph was. The map frame stays in charge of all *geometry*;
    only the turn counter moves.

    **A lap that will not close is trimmed rather than left to run.** The
    proximity gate can go unsatisfied indefinitely -- a reactive controller
    does not repeat its line, and one pass 1.8m wide of a 1.5m gate costs a
    whole extra revolution. On the 126m course this car actually maps that
    is 2.3 minutes per miss. So the gate widens once the car is clearly
    past one revolution without closing, and `lap_points` then trims the
    recording back to its final revolution, because a raceline recorded
    over two overlapping laps is not a raceline.
    """

    def __init__(self, spacing: float, min_distance: float,
                 departure_distance: float, closure_distance: float,
                 closure_heading_rad: float, min_duration_sec: float,
                 min_turn_rad: float = 0.0, max_pose_jump: float = 0.0,
                 closure_widen_after_revolutions: float = 0.0,
                 max_closure_distance: float = 0.0):
        self.spacing = spacing
        self.min_distance = min_distance
        self.departure_distance = departure_distance
        self.closure_distance = closure_distance
        self.closure_heading_rad = closure_heading_rad
        self.min_duration_sec = min_duration_sec
        self.min_turn_rad = min_turn_rad
        self.max_pose_jump = max_pose_jump
        # Revolutions past the first after which the proximity gate starts
        # opening up, and the ceiling it opens to. 0 for either disables
        # widening entirely and restores the fixed gate.
        self.closure_widen_after_revolutions = closure_widen_after_revolutions
        self.max_closure_distance = max(max_closure_distance, closure_distance)
        self.reset()

    def reset(self):
        self.points = []
        # Signed accumulated turn as of each entry in `points`, so the
        # recording can be trimmed back to its final revolution.
        self.point_turn = []
        self.start = None
        self.start_yaw = None
        self.start_time = None
        self.last_sample = None
        self.last_pose = None
        self.last_odom_yaw = None
        self.distance = 0.0
        self.turn = 0.0
        self.departed = False
        self.reanchor_count = 0
        # Latest closure-gate measurements, exposed for the supervisor's
        # rate-limited progress diagnostics.
        self.elapsed = 0.0
        self.distance_from_start = 0.0
        self.heading_error = 0.0
        # Closest the car has come back to its start since departing, so a
        # near miss is reportable rather than invisible.
        self.closest_approach = float('inf')

    @property
    def revolutions(self) -> float:
        """Turning done so far, in laps of a closed circuit (2*pi each)."""
        return abs(self.turn) / (2.0 * math.pi)

    @property
    def effective_closure_distance(self) -> float:
        """The proximity gate actually in force this tick.

        Fixed at `closure_distance` for the first revolution and a bit
        beyond, then opened in proportion to the extra turning done, up to
        `max_closure_distance`. A reactive controller does not repeat its
        line, so the car can lap a course all day passing consistently
        just outside a fixed gate -- and each of those misses costs a
        whole revolution, 2.3 minutes on the 126m course this car maps.
        Widening trades a looser idea of "back where we started" for
        actually finishing; `lap_points` then trims the extra away.
        """
        after = self.closure_widen_after_revolutions
        if after <= 0.0 or self.max_closure_distance <= self.closure_distance:
            return self.closure_distance
        excess = self.revolutions - after
        if excess <= 0.0:
            return self.closure_distance
        return min(self.max_closure_distance,
                   self.closure_distance * (1.0 + excess))

    def lap_points(self):
        """The recording, trimmed to its final revolution.

        A closure that took two revolutions produces two overlapping laps
        of points, and a raceline fitted through both is not a raceline --
        it self-intersects, and the speed profile then brakes for corners
        the car will not be in. Keeping the last 2*pi of turning yields
        exactly one lap ending where the closure fired, whatever happened
        before it. A lap that closed in one revolution or less is returned
        whole, which is the normal case and unchanged.
        """
        if len(self.points) < 3 or len(self.point_turn) != len(self.points):
            return list(self.points)
        total = self.point_turn[-1]
        direction = 1.0 if total >= 0.0 else -1.0
        target = total - direction * 2.0 * math.pi
        # Turning is constant along a straight, so many samples share the
        # exact target value and the boundary lands on a whole plateau of
        # them. Without the tolerance the accumulated float error decides
        # which end of that plateau wins, and a 16-sample straight gets
        # kept or dropped on the last bit of a mantissa.
        limit = direction * target + 1e-9
        start_index = 0
        for index, turn in enumerate(self.point_turn):
            if direction * turn <= limit:
                start_index = index
        if start_index == 0:
            return list(self.points)
        trimmed = list(self.points[start_index:])
        return trimmed if len(trimmed) >= 3 else list(self.points)

    def _reanchor(self, x: float, y: float, yaw: float):
        """Move everything already recorded into the corrected map frame.

        The correction is the rigid transform from the previous pose to the
        new one; applying it to the stored points keeps the recorded shape
        exactly as driven and keeps the start pose -- which the closure test
        measures against -- attached to the same piece of track.
        """
        previous_x, previous_y, previous_yaw = self.last_pose
        delta_yaw = angle_difference(yaw, previous_yaw)
        cos_delta, sin_delta = math.cos(delta_yaw), math.sin(delta_yaw)

        def moved(point):
            dx = point[0] - previous_x
            dy = point[1] - previous_y
            return (x + cos_delta * dx - sin_delta * dy,
                    y + sin_delta * dx + cos_delta * dy)

        self.points = [moved(point) for point in self.points]
        if self.start is not None:
            self.start = np.array(moved(self.start), dtype=np.float64)
            self.start_yaw = angle_difference(self.start_yaw + delta_yaw, 0.0)
        if self.last_sample is not None:
            self.last_sample = np.array(moved(self.last_sample), dtype=np.float64)
        self.reanchor_count += 1

    def update(self, x: float, y: float, yaw: float, now_sec: float,
               odom_yaw: float = None) -> bool:
        """Feed one localisation pose in. True means the lap just closed.

        `odom_yaw` is the same instant's heading in the odom frame. It is
        optional -- without it the turn counter falls back to map yaw and
        behaves exactly as it did before -- but supplying it is what makes
        the turn gate independent of how often SLAM re-optimises, since
        odometry is never re-optimised. See the class docstring.
        """
        point = np.array([x, y], dtype=np.float64)
        if self.start is None:
            self.start = point
            self.start_yaw = yaw
            self.start_time = now_sec
            self.last_sample = point
            self.last_pose = (x, y, yaw)
            self.last_odom_yaw = odom_yaw
            self.points.append((x, y))
            self.point_turn.append(self.turn)
            return False

        jumped = (
            self.max_pose_jump > 0.0
            and math.hypot(x - self.last_pose[0], y - self.last_pose[1]) > self.max_pose_jump
        )
        if jumped:
            self._reanchor(x, y, yaw)
        # Accumulated yaw, not path heading: yaw comes straight from
        # localisation and does not have the sampled path's sensitivity
        # to noise at 0.15m spacing.
        if odom_yaw is not None and self.last_odom_yaw is not None:
            # Odometry is not re-optimised, so this is the car's own
            # turning whether or not the pose graph moved this tick.
            self.turn += angle_difference(odom_yaw, self.last_odom_yaw)
        elif not jumped:
            # Map yaw, and only when the map did not move under us -- across
            # a re-anchor the yaw change is the map's, not the car's.
            self.turn += angle_difference(yaw, self.last_pose[2])
        self.last_pose = (x, y, yaw)
        if odom_yaw is not None:
            self.last_odom_yaw = odom_yaw

        distance_from_last = float(np.linalg.norm(point - self.last_sample))
        if distance_from_last >= self.spacing:
            self.distance += distance_from_last
            self.last_sample = point
            self.points.append((x, y))
            self.point_turn.append(self.turn)

        distance_from_start = float(np.linalg.norm(point - self.start))
        if distance_from_start >= self.departure_distance:
            self.departed = True
        if self.departed:
            self.closest_approach = min(self.closest_approach, distance_from_start)

        self.elapsed = now_sec - self.start_time
        self.distance_from_start = distance_from_start
        self.heading_error = abs(angle_difference(yaw, self.start_yaw))
        return bool(
            self.departed
            and self.distance >= self.min_distance
            and abs(self.turn) >= self.min_turn_rad
            and self.elapsed >= self.min_duration_sec
            and self.distance_from_start <= self.effective_closure_distance
            and self.heading_error <= self.closure_heading_rad
            and len(self.points) >= 3
        )


class AutoMapRaceNode(Node):
    """Select mapping/racing commands and automate the transition."""

    def __init__(self):
        super().__init__('auto_map_race_node')

        self.declare_parameter('mapping_drive_topic', '/auto_map/drive')
        self.declare_parameter('racing_drive_topic', '/auto_race/drive')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('pose_topic', '/slam_pose')
        self.declare_parameter('controller_topic', '/auto_map_race/controller')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('control_rate_hz', 40.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('waypoint_spacing', 0.15)
        self.declare_parameter('mapping_laps', 2)
        # A sanity floor only. It used to be the main closure gate at 20.0m,
        # which is longer than this car's ~15m room -- so the gate could not
        # open until the car had been round twice, and every lap it has ever
        # recorded is two overlapping revolutions. `minimum_lap_turn_deg` is
        # the gate that actually knows what a lap is.
        self.declare_parameter('minimum_lap_distance', 5.0)
        self.declare_parameter('minimum_lap_turn_deg', 300.0)
        # A map-frame pose that moves further than this between control ticks
        # is slam_toolbox correcting its graph, not the car moving. Recorded
        # as motion it becomes a corner nothing can steer; see
        # LapRecorder._reanchor. 0 disables the check.
        self.declare_parameter('max_pose_jump_m', 0.12)
        self.declare_parameter('minimum_lap_duration_sec', 15.0)
        self.declare_parameter('departure_distance', 2.0)
        self.declare_parameter('closure_distance', 0.75)
        self.declare_parameter('closure_heading_deg', 30.0)
        self.declare_parameter('closure_widen_after_revolutions', 1.25)
        self.declare_parameter('max_closure_distance', 4.0)
        self.declare_parameter('transition_stop_sec', 2.0)
        self.declare_parameter('map_save_timeout_sec', 20.0)
        # slam_toolbox's SaveMap runs nav2's map_saver inline, and map_saver
        # gives up after about two seconds of "Failed to spin map
        # subscription" if no /map arrives in that window. /map is republished
        # every map_update_interval (5s), so whether the save works is a race
        # against when the request happens to land. Retrying moves it into a
        # different part of that window.
        self.declare_parameter('map_save_retries', 3)
        self.declare_parameter('map_save_retry_delay_sec', 2.5)
        self.declare_parameter('output_directory', '~/.ros/racerbot_auto')
        self.declare_parameter('profile_max_speed', 4.0)
        self.declare_parameter('profile_min_speed', 0.5)
        self.declare_parameter('profile_max_lateral_accel', 2.5)
        self.declare_parameter('profile_max_accel', 3.0)
        self.declare_parameter('profile_max_brake', 8.0)
        self.declare_parameter('profile_smoothing_passes', 5)
        self.declare_parameter('pure_pursuit_node_name', 'pure_pursuit_node')
        # --- Racing-line cleanup (pure_pursuit/recorded_path.py) ---
        self.declare_parameter('profile_max_steering_angle', 0.26)
        self.declare_parameter('profile_wheelbase', 0.324)
        self.declare_parameter('profile_min_feature_wavelength', 1.5)
        self.declare_parameter('profile_curvature_margin', 1.0)
        self.declare_parameter('profile_max_deviation', 0.35)
        self.declare_parameter('profile_reject_ratio', 1.5)
        self.declare_parameter('profile_reject_fraction', 0.25)
        # The map the line has to fit inside. Checked because filtering a
        # recorded lap rounds its corners *inward*, toward the wall, and on
        # a course whose corners are near the car's turning circle that is
        # the direction that hurts -- one measured run finished 0.05m from a
        # wall with every other number looking healthy.
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('profile_wall_clearance', 0.30)
        self.declare_parameter('profile_map_occupied_threshold', 50)
        self.declare_parameter('map_despeckle_max_cells', 4)
        # --- Minimum-curvature raceline optimization (docs/racing-autonomy.md
        # Phase 4b, run inline here instead of as a separate manual step) ---
        self.declare_parameter('optimize_raceline', True)
        self.declare_parameter('optimize_spacing', 0.30)
        self.declare_parameter('optimize_output_spacing', 0.15)
        self.declare_parameter('optimize_iterations', 8)
        self.declare_parameter('optimize_trust_region', 0.30)
        self.declare_parameter('optimize_smoothing_weight', 0.0)
        self.declare_parameter('optimize_max_track_width', 6.0)
        self.declare_parameter('optimize_centerline_passes', 4)
        self.declare_parameter('optimize_centerline_smoothing', 3)
        self.declare_parameter('optimize_car_width', 0.31)
        self.declare_parameter('optimize_safety_margin', 0.15)
        # --- Handing localization to the particle filter for the race ---
        self.declare_parameter('localize_after_mapping', True)
        self.declare_parameter('pf_pose_topic', '/pf/viz/inferred_pose')
        self.declare_parameter('pf_initialpose_topic', '/initialpose')
        self.declare_parameter('pf_startup_timeout_sec', 45.0)
        self.declare_parameter('pf_pose_timeout_sec', 0.5)
        self.declare_parameter('pf_settle_poses', 20)
        self.declare_parameter('enable_deadman', True)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('joy_timeout_sec', 0.5)
        self.declare_parameter('decision_log_period_sec', 1.0)

        def value(name):
            return self.get_parameter(name).value

        self.mapping_drive_topic = str(value('mapping_drive_topic'))
        self.racing_drive_topic = str(value('racing_drive_topic'))
        self.drive_topic = str(value('drive_topic'))
        self.pose_topic = str(value('pose_topic'))
        self.map_frame = str(value('map_frame'))
        self.base_frame = str(value('base_frame'))
        self.odom_frame = str(value('odom_frame'))
        self.control_rate_hz = float(value('control_rate_hz'))
        self.command_timeout_sec = float(value('command_timeout_sec'))
        self.mapping_laps = max(1, int(value('mapping_laps')))
        self.transition_stop_sec = float(value('transition_stop_sec'))
        self.map_save_timeout_sec = float(value('map_save_timeout_sec'))
        self.map_save_retries = max(0, int(value('map_save_retries')))
        self.map_save_retry_delay_sec = float(value('map_save_retry_delay_sec'))
        self.output_directory = Path(os.path.expanduser(str(value('output_directory'))))
        self.profile_max_speed = float(value('profile_max_speed'))
        self.profile_min_speed = float(value('profile_min_speed'))
        self.profile_max_lateral_accel = float(value('profile_max_lateral_accel'))
        self.profile_max_accel = float(value('profile_max_accel'))
        self.profile_max_brake = float(value('profile_max_brake'))
        self.profile_smoothing_passes = int(value('profile_smoothing_passes'))
        self.enable_deadman = bool(value('enable_deadman'))
        self.joy_topic = str(value('joy_topic'))
        self.deadman_button = int(value('deadman_button'))
        self.joy_timeout_sec = float(value('joy_timeout_sec'))
        self.decision_log_period_sec = max(
            0.0, float(value('decision_log_period_sec')))

        self.profile_max_steering_angle = float(value('profile_max_steering_angle'))
        self.profile_wheelbase = float(value('profile_wheelbase'))
        self.profile_min_feature_wavelength = float(
            value('profile_min_feature_wavelength'))
        self.profile_curvature_margin = float(value('profile_curvature_margin'))
        self.profile_max_deviation = float(value('profile_max_deviation'))
        self.profile_reject_ratio = float(value('profile_reject_ratio'))
        self.profile_reject_fraction = float(value('profile_reject_fraction'))
        self.profile_wall_clearance = float(value('profile_wall_clearance'))
        self.profile_map_occupied_threshold = int(
            value('profile_map_occupied_threshold'))
        self.map_despeckle_max_cells = int(value('map_despeckle_max_cells'))
        self.optimize_raceline = bool(value('optimize_raceline'))
        self.optimize_spacing = float(value('optimize_spacing'))
        self.optimize_output_spacing = float(value('optimize_output_spacing'))
        self.optimize_iterations = int(value('optimize_iterations'))
        self.optimize_trust_region = float(value('optimize_trust_region'))
        self.optimize_smoothing_weight = float(value('optimize_smoothing_weight'))
        self.optimize_max_track_width = float(value('optimize_max_track_width'))
        self.optimize_centerline_passes = int(value('optimize_centerline_passes'))
        self.optimize_centerline_smoothing = int(value('optimize_centerline_smoothing'))
        self.optimize_car_width = float(value('optimize_car_width'))
        self.optimize_safety_margin = float(value('optimize_safety_margin'))
        self.localize_after_mapping = bool(value('localize_after_mapping'))
        self.pf_pose_topic = str(value('pf_pose_topic'))
        self.pf_initialpose_topic = str(value('pf_initialpose_topic'))
        self.pf_startup_timeout_sec = float(value('pf_startup_timeout_sec'))
        self.pf_pose_timeout_sec = float(value('pf_pose_timeout_sec'))
        self.pf_settle_poses = int(value('pf_settle_poses'))

        # Particle-filter handover state. `pf_active` is the one that decides
        # which estimate the car actually steers on -- see _lookup_and_publish_pose.
        self.pf_process = None
        self.pf_pose = None
        self.pf_pose_time = None
        self.pf_pose_count = 0
        self.pf_active = False
        self.pf_started_at = None
        self.pf_gave_up = False
        self.latest_map = None

        self.recorder = LapRecorder(
            spacing=float(value('waypoint_spacing')),
            min_distance=float(value('minimum_lap_distance')),
            departure_distance=float(value('departure_distance')),
            closure_distance=float(value('closure_distance')),
            closure_heading_rad=math.radians(float(value('closure_heading_deg'))),
            min_duration_sec=float(value('minimum_lap_duration_sec')),
            min_turn_rad=math.radians(float(value('minimum_lap_turn_deg'))),
            max_pose_jump=float(value('max_pose_jump_m')),
            closure_widen_after_revolutions=float(
                value('closure_widen_after_revolutions')),
            max_closure_distance=float(value('max_closure_distance')),
        )

        self.state = 'mapping'
        self.completed_mapping_laps = 0
        # Learned from the first closed lap, so progress on later laps is
        # reported against this course rather than against nothing.
        self.measured_lap_distance = 0.0
        self.measured_lap_duration_sec = 0.0
        self.mapping_cmd = None
        self.mapping_cmd_time = None
        self.racing_cmd = None
        self.racing_cmd_time = None
        self.deadman_held = False
        self.last_joy_time = None
        self.joy_button_available = False
        self.profile_request_started = False
        self.profile_path = None
        self.race_enable_time = None
        self.map_save_started = False
        self.map_saves_expected = 2      # occupancy map + pose graph
        self.map_saves_completed = 0
        self.map_save_deadline = None
        self.map_save_timed_out = False
        self.map_save_attempts = 0
        self.map_save_retry_at = None
        self.map_saved_ok = False
        self.last_decision_state = None
        self.last_decision_log_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.parameter_client = AsyncParameterClient(
            self, str(value('pure_pursuit_node_name')))
        self.save_map_client = self.create_client(
            SaveMap, '/slam_toolbox/save_map')
        self.serialize_map_client = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        # Latched: a browser that connects halfway through a run still
        # learns who is driving, without waiting for the next handover.
        self.controller_pub = self.create_publisher(
            String, str(value('controller_topic')),
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self._published_controller = None
        self.create_subscription(
            AckermannDriveStamped, self.mapping_drive_topic,
            self._mapping_drive_callback, 10)
        self.create_subscription(
            AckermannDriveStamped, self.racing_drive_topic,
            self._racing_drive_callback, 10)
        # slam_toolbox publishes /map transient-local, so matching durability
        # here delivers the current map even between its update intervals.
        self.create_subscription(
            OccupancyGrid, str(value('map_topic')), self._map_callback,
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        if self.enable_deadman:
            self.create_subscription(
                Joy, self.joy_topic, self._joy_callback, 10)
        if self.localize_after_mapping:
            self.create_subscription(
                PoseStamped, self.pf_pose_topic, self._pf_pose_callback, 10)
            self.initialpose_pub = self.create_publisher(
                PoseWithCovarianceStamped, self.pf_initialpose_topic, 10)

        self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.get_logger().info(
            f'Automatic mapping started: gap follow will map {self.mapping_laps} lap(s), '
            'then the generated profile will be loaded and pure pursuit will race. '
            f"Deadman {'ENABLED (hold LB)' if self.enable_deadman else 'DISABLED'}; "
            f"mapping source='{self.mapping_drive_topic}', racing source="
            f"'{self.racing_drive_topic}', output='{self.drive_topic}'; decision logs every "
            f"{self.decision_log_period_sec:.1f}s (plus immediate state changes).")

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _mapping_drive_callback(self, msg):
        self.mapping_cmd = msg
        self.mapping_cmd_time = self._now_sec()

    def _racing_drive_callback(self, msg):
        self.racing_cmd = msg
        self.racing_cmd_time = self._now_sec()

    def _map_callback(self, msg: OccupancyGrid):
        # Cache only. Building the clearance field is a whole-grid distance
        # transform and belongs in _write_profile, which runs once with the
        # car deliberately stopped -- not in a callback at map rate.
        self.latest_map = msg

    def _wall_clearance_function(self):
        """(callable, required_metres) for checking a line against the map.

        Returns (None, 0.0) when there is no map to check against, which
        `recorded_path.prepare` reports as "not checked" rather than
        silently passing.
        """
        # Also the grid the raceline optimizer measures track width from, so
        # the expensive despeckle+load happens once per profile run.
        self.profile_grid = None
        if self.latest_map is None or self.profile_wall_clearance <= 0.0:
            self.get_logger().warn(
                'No /map received, so the generated racing line cannot be '
                'checked for wall clearance. It may cut a corner into a wall.')
            return None, 0.0
        try:
            grid = occupancy_map.OccupancyMap.from_grid_message(
                self.latest_map.data,
                self.latest_map.info.width, self.latest_map.info.height,
                self.latest_map.info.resolution,
                self.latest_map.info.origin.position.x,
                self.latest_map.info.origin.position.y,
                occupied_threshold=self.profile_map_occupied_threshold,
                # Phantom obstacles in otherwise clear track are a
                # reason to reject a perfectly good racing line. Only
                # blobs the mapper saw straight through are dropped;
                # see occupancy_map.despeckle_grid.
                despeckle_max_cells=self.map_despeckle_max_cells)
            field = grid.clearance_field()
        except Exception as exc:  # noqa: BLE001 - never lose the run to this
            self.get_logger().error(
                f'Could not build a clearance field from /map ({exc}); the '
                'racing line will not be checked against the walls.')
            return None, 0.0
        self.profile_grid = grid
        return (lambda xs, ys: grid.clearance_at(xs, ys, field),
                self.profile_wall_clearance)

    def _optimize_line(self, prepared, clearance_fn, required_clearance):
        """The minimum-curvature line inside the mapped track, or None.

        None means "race the cleaned recording instead" and is a normal
        outcome, not an error: no map, the optimizer turned off, a track too
        narrow to measure, or a result that failed the same two checks
        `recorded_path.prepare` applies to the recording. Racing is never
        blocked by this step -- the worst case is the line that would have
        been raced anyway.

        The recorded lap is the seed, not the reference. `refine_centerline`
        walks it to the middle of the corridor it measures either side, which
        is what turns "wherever the car drove" into something the optimizer
        can hang a symmetric corridor off.
        """
        if not self.optimize_raceline:
            return None
        grid = getattr(self, 'profile_grid', None)
        if grid is None:
            self.get_logger().warn(
                'Raceline optimization is on but there is no usable /map to '
                'measure track width from. Racing the cleaned recording.')
            return None

        try:
            from pure_pursuit import raceline_optimizer

            centerline, width_left, width_right = raceline_optimizer.refine_centerline(
                grid, prepared.xy, self.optimize_spacing,
                self.optimize_max_track_width,
                iterations=self.optimize_centerline_passes,
                smoothing_window=self.optimize_centerline_smoothing)
            widths = width_left + width_right
            self.get_logger().info(
                f'Optimizing raceline: {len(centerline)} centerline points, '
                f'track width {widths.min():.2f}-{widths.max():.2f}m.')

            result = raceline_optimizer.optimize_minimum_curvature(
                centerline, width_left, width_right,
                vehicle_half_width=self.optimize_car_width / 2.0,
                safety_margin=self.optimize_safety_margin,
                spacing=self.optimize_spacing,
                iterations=self.optimize_iterations,
                smoothing_weight=self.optimize_smoothing_weight,
                trust_region=self.optimize_trust_region)
            line = raceline_optimizer.resample_closed_path(
                result['line'], self.optimize_output_spacing)
        except Exception as exc:  # noqa: BLE001 - never lose the run to this
            self.get_logger().error(
                f'Raceline optimization failed ({exc}). Racing the cleaned '
                'recording instead.')
            return None

        # Same two questions prepare() asks of the recording, asked of the
        # optimized line. The optimizer already holds itself off the walls
        # using the widths it measured -- this re-checks against the map
        # directly, because a mismeasured width is exactly the failure that
        # would put the line through a wall while every internal number
        # looked healthy.
        curvature = np.abs(racing_math.estimate_path_curvature(line, closed=True))
        old_curvature = np.abs(racing_math.estimate_path_curvature(
            prepared.xy, closed=True))
        curvature_limit = recorded_path.curvature_limit(
            self.profile_max_steering_angle, self.profile_wheelbase)
        max_curvature = float(curvature.max())
        old_max_curvature = float(old_curvature.max())

        clearance = None
        if clearance_fn is not None:
            clearance = float(np.min(clearance_fn(line[:, 0], line[:, 1])))

        # The whole justification for this step is lap time, so measure it
        # rather than assuming a less-curved line is always quicker. It is
        # not: minimum curvature buys corner speed by using the full width
        # of the track, which lengthens the path. On a wide circuit that
        # trade is strongly positive; on a tight little loop the extra
        # distance can cost more than the extra speed earns. Measured on
        # this car's own 13.3m test course, the optimized line came out
        # 16.9m long and a second per lap SLOWER.
        new_time = self._estimated_lap_time(line)
        old_time = self._estimated_lap_time(prepared.xy)

        detail = (
            f'mean |curvature| {float(old_curvature.mean()):.4f} -> '
            f'{float(curvature.mean()):.4f}, max {old_max_curvature:.3f} -> '
            f'{max_curvature:.3f} (rack limit {curvature_limit:.3f}), '
            f'estimated lap {old_time:.2f}s -> {new_time:.2f}s, '
            f'{len(line)} points, '
            + (f'min wall clearance {clearance:.2f}m'
               if clearance is not None else 'wall clearance not checked')
            + f", {result['clamped_fraction'] * 100:.0f}% of the track too "
              'narrow for the car plus its margin')

        # Wall clearance is the one hard refusal. This is the same question
        # `prepare` asks of the recording, and the same answer: a line the
        # car does not fit through is not raced, whatever else it offers.
        if clearance is not None and clearance < required_clearance:
            self.get_logger().error(
                f'The optimized racing line passes {clearance:.2f}m from a '
                f'wall, inside the {required_clearance:.2f}m required '
                f'({detail}). Racing the cleaned recording instead.')
            return None

        # Steering is a *comparison*, not an absolute bar. `prepare` already
        # accepts a recording that exceeds the rack and merely warns that the
        # car will understeer there, so refusing the optimized line outright
        # for the same fault would discard it on exactly the tight courses
        # where the recording is no better -- silently turning this whole
        # step off. It has to be no worse than the line it replaces.
        if max_curvature > curvature_limit and max_curvature > old_max_curvature:
            self.get_logger().error(
                'The optimized racing line asks for more steering than the '
                f'car has, and more than the recorded line does ({detail}). '
                'Racing the cleaned recording instead.')
            return None

        if new_time >= old_time:
            self.get_logger().warn(
                f'The optimized racing line is not faster ({detail}). Racing '
                'the cleaned recording instead -- this is normal on a course '
                'too tight or too narrow for a racing line to pay for the '
                'extra distance it travels.')
            return None

        if max_curvature > curvature_limit:
            self.get_logger().warn(
                f'The optimized racing line needs '
                f'{math.degrees(math.atan(max_curvature * self.profile_wheelbase)):.1f}deg '
                'of steering at its tightest point, past the rack limit -- but '
                'less than the recorded line needs, and it is faster. Expect a '
                'wide apex there.')

        self.get_logger().info(f'Optimized racing line accepted: {detail}.')
        return line

    def _estimated_lap_time(self, xy) -> float:
        """Lap time for a line under this run's own profile settings.

        Same call chain _write_profile uses, so the comparison that picks a
        line is made on the numbers the car will actually be given.
        """
        seg_len = racing_math.compute_segment_lengths(xy, closed=True)
        curvature = racing_math.estimate_path_curvature(xy, closed=True)
        speeds = racing_math.compute_velocity_profile(
            seg_len, curvature,
            v_max=self.profile_max_speed,
            v_min=self.profile_min_speed,
            a_lat_max=self.profile_max_lateral_accel,
            a_accel_max=self.profile_max_accel,
            a_brake_max=self.profile_max_brake,
            closed=True,
            smoothing_passes=self.profile_smoothing_passes,
        )
        return float(racing_math.estimate_lap_time(seg_len, speeds))

    def _joy_callback(self, msg):
        self.last_joy_time = self._now_sec()
        self.joy_button_available = len(msg.buttons) > self.deadman_button
        self.deadman_held = bool(
            self.joy_button_available and msg.buttons[self.deadman_button])

    def _deadman_engaged(self, now_sec: float) -> bool:
        return self._deadman_status(now_sec)[0]

    def _deadman_status(self, now_sec: float):
        if not self.enable_deadman:
            return True, None, None
        if self.last_joy_time is None:
            return (
                False,
                'waiting_for_joy',
                f"no Joy messages received on '{self.joy_topic}'; LB cannot be verified",
            )
        age_sec = now_sec - self.last_joy_time
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

    def _pf_pose_callback(self, msg: PoseStamped):
        self.pf_pose = msg
        self.pf_pose_time = self._now_sec()
        self.pf_pose_count += 1

    def _pf_pose_fresh(self, now_sec: float) -> bool:
        return (self.pf_pose is not None
                and self.pf_pose_time is not None
                and (now_sec - self.pf_pose_time) <= self.pf_pose_timeout_sec)

    def _start_particle_filter(self, now_sec: float):
        """Spawn particle-filter localization against the map just saved.

        Started as a child process rather than declared in the launch file
        because the map it localizes against does not exist until this point
        in the run -- `map_server` needs a file on disk at configure time,
        and `particle_filter` blocks in its constructor waiting for that
        map's service. Neither can be brought up before the map is written.

        Failure here is not fatal. Everything about the handover is written
        so that a particle filter which never starts, never converges, or
        dies later leaves the car racing on slam_toolbox exactly as it did
        before this existed.
        """
        if self.pf_process is not None or self.pf_gave_up:
            return
        if not hasattr(self, 'run_directory'):
            return
        # Path(), not `self.run_directory / ...`: the save path can time out
        # and reach here before _write_profile has run, and tests set this
        # attribute to a plain string.
        map_yaml = Path(self.run_directory) / 'map.yaml'
        if not map_yaml.is_file():
            self.get_logger().warn(
                f'No saved map at {map_yaml}, so localization stays on '
                'slam_toolbox for the race.')
            self.pf_gave_up = True
            return
        launcher = shutil.which('ros2')
        if launcher is None:
            self.get_logger().error(
                "'ros2' is not on PATH, so the particle filter cannot be "
                'started. Racing on slam_toolbox.')
            self.pf_gave_up = True
            return
        try:
            self.pf_process = subprocess.Popen(
                [launcher, 'launch', 'racerbot_launch', 'localize_run_launch.py',
                 f'map_yaml:={map_yaml}'],
                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                # Deliberately NOT start_new_session: staying in this
                # process group means the Ctrl+C that stops the run reaches
                # `ros2 launch` too, and it shuts its own nodes down in
                # order. PDEATHSIG below is the backstop for every other way
                # this node can die. Measured: with a new session and no
                # PDEATHSIG, killing a sim run left map_server and the
                # particle filter running afterwards, holding the service
                # name that the next run's handover needs.
                preexec_fn=_die_with_parent)
        except OSError as exc:
            self.get_logger().error(
                f'Could not start particle-filter localization ({exc}). '
                'Racing on slam_toolbox.')
            self.pf_gave_up = True
            return
        self.pf_started_at = now_sec
        self.get_logger().info(
            f'Starting particle-filter localization against {map_yaml} '
            f'(pid {self.pf_process.pid}); racing continues on slam_toolbox '
            'until it has converged.')

    def _seed_particle_filter(self, pose):
        """Hand the filter the pose SLAM already knows, instead of an RViz click.

        Without a seed the filter spreads its particles uniformly over every
        free cell in the map and has to converge from scratch, which on a
        symmetric course can settle confidently into the wrong place. The car
        is standing still at a pose slam_toolbox has just finished refining,
        so there is a better answer available for free.
        """
        if pose is None:
            return
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.initialpose_pub.publish(msg)

    def _update_pf_handover(self, now_sec: float, pose):
        """Drive the slam_toolbox -> particle filter switch, and back again.

        Called every tick once the map is on disk. Three jobs, in order:
        seed the filter while it is warming up, promote it once it has
        produced enough consecutive poses to be believable, and demote it
        immediately if it ever goes quiet.
        """
        if not self.localize_after_mapping or self.pf_gave_up:
            return
        if self.pf_process is None:
            return

        if self.pf_process.poll() is not None and not self.pf_active:
            self.get_logger().error(
                f'Particle-filter localization exited (code '
                f'{self.pf_process.returncode}) before it was ready. Racing '
                'on slam_toolbox.')
            self.pf_gave_up = True
            self.pf_process = None
            return

        if not self.pf_active:
            # Re-seed while waiting: the filter subscribes to /initialpose
            # after its own constructor finishes loading the map, which is
            # seconds after this process started, so a single seed sent too
            # early is simply dropped on the floor.
            self._seed_particle_filter(pose)
            if self.pf_pose_count >= self.pf_settle_poses and self._pf_pose_fresh(now_sec):
                self.pf_active = True
                self.get_logger().info(
                    f'Localization handed to the particle filter after '
                    f'{self.pf_pose_count} poses. Pure pursuit now steers on '
                    'it; slam_toolbox remains the fallback.')
            elif (self.pf_started_at is not None
                    and now_sec - self.pf_started_at > self.pf_startup_timeout_sec):
                self.get_logger().error(
                    f'The particle filter produced {self.pf_pose_count} pose(s) '
                    f'in {self.pf_startup_timeout_sec:.0f}s, short of the '
                    f'{self.pf_settle_poses} needed. Racing on slam_toolbox.')
                self.pf_gave_up = True
            return

        if not self._pf_pose_fresh(now_sec):
            # Demotion is deliberately one-way for the rest of the run. A
            # filter that has already gone quiet once at racing speed has
            # not earned a second chance mid-lap, and flapping between two
            # pose sources is worse than either of them.
            self.get_logger().error(
                'The particle filter stopped publishing; falling back to '
                'slam_toolbox for the rest of the run.')
            self.pf_active = False
            self.pf_gave_up = True

    def _stop_particle_filter(self):
        if self.pf_process is None:
            return
        try:
            self.pf_process.terminate()
            self.pf_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.pf_process.kill()
        except OSError:
            pass
        self.pf_process = None

    def _lookup_and_publish_pose(self):
        # Once the particle filter is trusted it becomes the pose the car
        # steers on. pure_pursuit is not reconfigured and does not know:
        # this node has always been the thing publishing `pose_topic`, so
        # the handover is a change of source, not a change of wiring.
        if self.pf_active and self.pf_pose is not None:
            pose = PoseStamped()
            pose.header = self.pf_pose.header
            pose.header.frame_id = self.map_frame
            pose.pose = self.pf_pose.pose
            self.pose_pub.publish(pose)
            q = pose.pose.orientation
            return (pose.pose.position.x, pose.pose.position.y,
                    racing_math.quaternion_to_yaw(q.x, q.y, q.z, q.w))
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for {self.map_frame}->{self.base_frame} from SLAM: {exc}',
                throttle_duration_sec=2.0)
            return None

        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pose_pub.publish(pose)
        q = transform.transform.rotation
        yaw = racing_math.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return pose.pose.position.x, pose.pose.position.y, yaw

    def _lookup_odom_yaw(self):
        """Heading in the odom frame, or None if odometry is not up yet.

        Only the lap recorder's turn counter uses this, and only to stay
        independent of pose-graph re-optimisation -- every piece of
        geometry still comes from the map frame. A missing odom transform
        is therefore not worth a warning: the recorder falls back to
        counting map yaw exactly as it did before.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        q = transform.transform.rotation
        return racing_math.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def _publish_stop(self):
        self._publish_command(None)

    def _publish_command(self, source):
        output = AckermannDriveStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_link'
        if source is not None:
            output.drive = source.drive
        self.drive_pub.publish(output)

    #: Supervisor state -> the node whose knobs actually reach the car.
    CONTROLLER_FOR_STATE = {
        'mapping': 'gap_follow_node',
        'racing': 'pure_pursuit_node',
    }

    def _publish_controller(self):
        """Name the controller currently selected, for the dashboard.

        Read-only diagnostic. Published strictly after this tick's drive
        command, only when the answer changes, and wrapped in its own
        try/except that disables the diagnostic rather than the node --
        the contract every diagnostic published from driving code follows
        here (docs/drive-intent.md#safety-contract-for-publishers-read-this-first).

        It reports which controller is *selected*, not whether the car is
        moving, so it does not flicker every time a stop is commanded or
        LB is released: the question it answers is "whose parameters
        affect this car right now", and during a mapping lap that is
        gap_follow even while the car sits still.

        This exists because of the 2026-08-19 run. The dashboard offers
        gap_follow_node and pure_pursuit_node side by side with nothing
        saying which one is driving, the car was under gap_follow for the
        entire mapping phase, and every live-tune write that run went to
        pure_pursuit -- where it changed nothing, because pure_pursuit was
        parked in waiting_for_profile the whole time.
        """
        if self.controller_pub is None:
            return
        controller = self.CONTROLLER_FOR_STATE.get(self.state, '')
        if controller == self._published_controller:
            return
        try:
            message = String()
            message.data = controller
            self.controller_pub.publish(message)
            self._published_controller = controller
        except Exception as exc:
            self.controller_pub = None
            self.get_logger().warn(
                f'controller diagnostic disabled after {type(exc).__name__}: {exc}; '
                'the supervisor itself is unaffected')

    def _command_status(self, command, stamp, now_sec, source_name: str):
        if command is None or stamp is None:
            return (
                None,
                f'{source_name}_command_missing',
                f"no command has arrived on the selected '{source_name}' input",
            )
        age_sec = now_sec - stamp
        if age_sec > self.command_timeout_sec:
            return (
                None,
                f'{source_name}_command_stale',
                f"selected '{source_name}' command is {age_sec:.2f}s old "
                f"(limit {self.command_timeout_sec:.2f}s)",
            )
        return (
            command,
            None,
            f"selected '{source_name}' command is fresh (age={age_sec:.2f}s)",
        )

    def _lap_progress_detail(self, pose) -> str:
        current_lap = min(self.completed_mapping_laps + 1, self.mapping_laps)
        prefix = f'lap {current_lap}/{self.mapping_laps}'
        if pose is None:
            return f'{prefix}: recorder waiting for a valid {self.map_frame}->{self.base_frame} pose'
        if self.recorder.start is None:
            return f'{prefix}: recorder waiting to initialize its start pose'
        return (
            f'{prefix}: {self._lap_progress_summary()}, '
            f'samples={len(self.recorder.points)}, '
            f'distance={self.recorder.distance:.1f}/{self.recorder.min_distance:.1f}m, '
            f'turn={math.degrees(self.recorder.turn):.0f}/'
            f'{math.degrees(self.recorder.min_turn_rad):.0f}deg, '
            f'elapsed={self.recorder.elapsed:.1f}/{self.recorder.min_duration_sec:.1f}s, '
            f"departed={'yes' if self.recorder.departed else 'no'}, "
            f'start distance={self.recorder.distance_from_start:.2f}/'
            f'{self.recorder.effective_closure_distance:.2f}m, heading error='
            f'{math.degrees(self.recorder.heading_error):.1f}/'
            f'{math.degrees(self.recorder.closure_heading_rad):.1f}deg, '
            f'SLAM corrections absorbed={self.recorder.reanchor_count}'
            + self._closure_warning())

    def _lap_progress_summary(self) -> str:
        """How far round the car is, in plain terms, first in the line.

        The 2026-08-19 run printed only the raw gates, and a lap on that
        course legitimately took 126m and 136s. With nothing saying "about
        a third of the way round" the operator had no way to tell a lap
        still in progress from a lap that would never close, and stopped a
        run that was working. Turning is the measure that knows what a lap
        is, so the fraction is turning done over one revolution; once a
        lap has actually closed, its length is the better estimate and the
        remaining time comes from that.
        """
        revolutions = self.recorder.revolutions
        percent = min(100.0, 100.0 * revolutions)
        summary = f'~{percent:.0f}% round'
        if self.measured_lap_distance and self.recorder.distance > 0.0:
            remaining = max(0.0, self.measured_lap_distance - self.recorder.distance)
            summary += f', ~{remaining:.0f}m to go'
            if self.measured_lap_duration_sec and self.measured_lap_distance > 0.0:
                pace = self.measured_lap_duration_sec / self.measured_lap_distance
                summary += f' (~{remaining * pace:.0f}s at last lap\'s pace)'
        return summary

    def _closure_warning(self) -> str:
        """Say so when the car is clearly lapping but nothing is closing.

        Turning is the gate that knows what a lap is; proximity to the start
        is the one that can quietly never be satisfied, because a reactive
        controller does not repeat its line. Twice round with no closure is
        not "still working on it", and the operator should not have to
        infer that from two numbers.

        The gate now opens by itself past
        `closure_widen_after_revolutions`, so this reports the widening in
        progress and the closest the car has actually come -- which says
        whether the proximity gate or the heading gate is the one holding
        out, the thing the old wording left the operator to guess.
        """
        turn_gate = self.recorder.min_turn_rad
        if turn_gate <= 0.0 or abs(self.recorder.turn) < 2.0 * turn_gate:
            return ''
        closest = self.recorder.closest_approach
        closest_text = ('never' if not math.isfinite(closest)
                        else f'{closest:.2f}m')
        gate = self.recorder.effective_closure_distance
        widened = gate > self.recorder.closure_distance
        return (
            f' -- WARNING: {self.recorder.revolutions:.1f} laps of turning with no '
            f'closure. Closest the car has come back to where the recorder started '
            f'is {closest_text}, against a gate now at {gate:.2f}m'
            + (f' (widened from {self.recorder.closure_distance:.2f}m and still '
               f'opening, up to {self.recorder.max_closure_distance:.2f}m)'
               if widened else
               ' (fixed -- closure_widen_after_revolutions/max_closure_distance '
               'are off)')
            + f'. Heading error there must also be within '
            f'{math.degrees(self.recorder.closure_heading_rad):.0f}deg. The '
            'recording will be trimmed back to its final revolution when it does '
            'close.')

    def _control_loop(self):
        try:
            self._control_step()
        except Exception as exc:
            self._publish_stop()
            self._log_decision(
                'control_exception',
                f'unhandled {type(exc).__name__}: {exc}; supervisor published stop and will exit',
                None,
                level='error',
            )
            raise

    def _control_step(self):
        now_sec = self._now_sec()
        pose = self._lookup_and_publish_pose()
        # After the pose is published, so a handover that happens this tick
        # takes effect on the next one and the car never steers on a pose
        # source that changed halfway through a single decision.
        self._update_pf_handover(now_sec, pose)
        deadman_ok, deadman_state, deadman_detail = self._deadman_status(now_sec)

        if self.state == 'mapping' and pose is not None and deadman_ok:
            if self.recorder.update(*pose, now_sec, self._lookup_odom_yaw()):
                self._mapping_lap_completed()

        if self.state == 'loading_profile':
            # Order matters, and this is a safety ordering, not a tidiness
            # one. slam_toolbox's save_map/serialize_map are long BLOCKING
            # calls on its own executor: while they run it stops advancing
            # map->odom, so the map->base_link TF this node republishes as
            # /slam_pose freezes -- at full rate, with no gap a
            # message-arrival watchdog could see. On 2026-07-27 the save
            # was fired concurrently with the handover and pure pursuit
            # spent its first second steering from a pose that was already
            # a second out of date, into a wall.
            #
            # The car is deliberately stopped for this whole state, so it
            # is the one moment a frozen transform is harmless. Finish the
            # save FIRST, then hand the profile over; racing cannot begin
            # until both have completed (or timed out).
            self._try_save_map(now_sec)
            if self._map_save_settled(now_sec):
                # After the save, so the filter reads the finished, despeckled
                # map rather than a half-written one -- _map_save_callback
                # despeckles before it counts the save as complete.
                self._start_particle_filter(now_sec)
                self._try_load_profile()

        if self.state == 'transition' and now_sec >= self.race_enable_time:
            self.state = 'racing'
            self.get_logger().info('Transition complete: pure pursuit now has drive control.')

        if not deadman_ok:
            self._publish_stop()
            self._log_decision(deadman_state, deadman_detail, None)
        elif self.state == 'mapping':
            command, stop_state, source_detail = self._command_status(
                self.mapping_cmd, self.mapping_cmd_time, now_sec, 'gap_follow')
            progress_detail = self._lap_progress_detail(pose)
            self._publish_command(command)
            if command is None:
                self._log_decision(
                    stop_state, f'{source_detail}; {progress_detail}', None)
            elif command.drive.speed <= 0.0:
                self._log_decision(
                    'mapping_controller_stop',
                    f'{source_detail}; gap follow requested a neutral command; {progress_detail}',
                    command,
                )
            else:
                self._log_decision(
                    'forwarding_mapping',
                    f'{source_detail}; forwarding gap follow; {progress_detail}',
                    command,
                )
        elif self.state == 'racing':
            command, stop_state, source_detail = self._command_status(
                self.racing_cmd, self.racing_cmd_time, now_sec, 'pure_pursuit')
            self._publish_command(command)
            if command is None:
                self._log_decision(stop_state, source_detail, None)
            elif command.drive.speed <= 0.0:
                self._log_decision(
                    'racing_controller_stop',
                    f'{source_detail}; pure pursuit requested a neutral command',
                    command,
                )
            else:
                self._log_decision(
                    'forwarding_racing',
                    f'{source_detail}; forwarding pure pursuit',
                    command,
                )
        elif self.state == 'loading_profile':
            self._publish_stop()
            self._log_decision(
                'loading_profile',
                f"generated profile='{self.profile_path}'; waiting for pure pursuit to accept it",
                None,
                level='info',
            )
        elif self.state == 'transition':
            self._publish_stop()
            remaining_sec = max(0.0, self.race_enable_time - now_sec)
            self._log_decision(
                'transition_hold',
                f'profile loaded; deliberate stop before racing has {remaining_sec:.2f}s remaining',
                None,
                level='info',
            )
        elif self.state == 'error':
            self._publish_stop()
            self._log_decision(
                'supervisor_error',
                'automatic map-to-race transition failed; remaining stopped',
                None,
                level='error',
            )
        else:
            self._publish_stop()
            self._log_decision(
                'unknown_supervisor_state',
                f"unrecognized supervisor state '{self.state}'; remaining stopped",
                None,
                level='error',
            )

        # Last, after the drive command for this tick has gone out.
        self._publish_controller()

    def _log_decision(self, state: str, detail: str, source, level: str = None):
        """Log selector transitions immediately and steady state periodically."""
        now_sec = self._now_sec()
        state_changed = state != self.last_decision_state
        period_elapsed = (
            self.decision_log_period_sec > 0.0
            and (
                self.last_decision_log_time is None
                or now_sec - self.last_decision_log_time >= self.decision_log_period_sec
            )
        )
        if not state_changed and not period_elapsed:
            return

        steering = 0.0 if source is None else source.drive.steering_angle
        speed = 0.0 if source is None else source.drive.speed
        stopped = speed <= 0.0
        message = (
            f"{'STOP' if stopped else 'FORWARD'} [{state}] {detail}; output command: "
            f'steering={steering:+.3f}rad, speed={speed:.2f}m/s')
        if level == 'error':
            self.get_logger().error(message)
        elif level == 'info' or not stopped:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        self.last_decision_state = state
        self.last_decision_log_time = now_sec

    def _mapping_lap_completed(self):
        self.completed_mapping_laps += 1
        self.measured_lap_distance = self.recorder.distance
        self.measured_lap_duration_sec = self.recorder.elapsed
        trimmed = len(self.recorder.points) - len(self.recorder.lap_points())
        self.get_logger().info(
            f'Closed mapping lap {self.completed_mapping_laps}/{self.mapping_laps} detected '
            f'({self.recorder.distance:.1f}m, {math.degrees(self.recorder.turn):.0f}deg of '
            f'turning, {self.recorder.revolutions:.2f} revolutions, '
            f'{len(self.recorder.points)} samples, closest approach to the start '
            f'{self.recorder.closest_approach:.2f}m against a '
            f'{self.recorder.effective_closure_distance:.2f}m gate, '
            f'{self.recorder.reanchor_count} SLAM corrections absorbed'
            + (f', trimming {trimmed} samples back to the final revolution'
               if trimmed else '')
            + ').')
        if self.completed_mapping_laps < self.mapping_laps:
            # Say the cost out loud. A lap on the course this car actually
            # maps is 126m and over two minutes, and an operator who does
            # not know that reads a second lap as a stuck run.
            self.get_logger().info(
                f'Discarding that discovery lap and recording lap '
                f'{self.completed_mapping_laps + 1}/{self.mapping_laps}, which should '
                f'take about {self.measured_lap_distance:.0f}m and '
                f'{self.measured_lap_duration_sec:.0f}s at the same pace. Keep holding '
                'LB; racing starts after it closes.')
            # Discard the discovery lap. The next lap is recorded after SLAM
            # has seen the start/finish again and had a chance to close its
            # loop, yielding a cleaner map-frame raceline.
            self.recorder.reset()
            return
        self.state = 'loading_profile'
        try:
            self.profile_path = self._write_profile()
        except (OSError, ValueError) as exc:
            self.state = 'error'
            self.get_logger().error(f'Could not generate the racing profile: {exc}')

    def _write_profile(self) -> str:
        run_directory = self.output_directory / strftime('%Y%m%d-%H%M%S')
        run_directory.mkdir(parents=True, exist_ok=False)
        # Trimmed to the final revolution: a closure that needed more
        # than one lap would otherwise fit a raceline through two
        # overlapping ones. See LapRecorder.lap_points.
        xy = np.asarray(self.recorder.lap_points(), dtype=np.float64)
        raw_path = run_directory / 'raceline_raw.csv'
        profile_path = run_directory / 'raceline_profiled.csv'
        # The unmodified recording is written first and always, whatever
        # happens next: if the cleanup below refuses the line, this file is
        # the evidence needed to work out why.
        racing_math.save_xy_csv(str(raw_path), xy)

        # A live SLAM pose is not a trajectory -- see recorded_path.py for
        # what this car's own recorded laps actually looked like and why
        # smooth_path on its own was nowhere near enough.
        clearance_fn, required_clearance = self._wall_clearance_function()
        prepared = recorded_path.prepare(
            xy,
            spacing=self.recorder.spacing,
            max_steering_angle=self.profile_max_steering_angle,
            wheelbase=self.profile_wheelbase,
            min_feature_wavelength=self.profile_min_feature_wavelength,
            curvature_margin=self.profile_curvature_margin,
            max_deviation=self.profile_max_deviation,
            reject_ratio=self.profile_reject_ratio,
            reject_fraction=self.profile_reject_fraction,
            clearance_fn=clearance_fn,
            required_clearance=required_clearance,
        )
        self.get_logger().info(f'Recorded lap cleaned up: {prepared.describe()}')
        if not prepared.acceptable:
            problem = (
                'passes closer to a wall than the car is wide'
                if not prepared.fits_the_track
                else 'asks for more steering than the car has')
            raise ValueError(
                f'the cleaned racing line {problem} ({prepared.describe()}). Refusing '
                'to hand it to pure pursuit: neither failure degrades gracefully -- the '
                'car either drives into the wall the line goes through, or saturates '
                'the steering, runs wide, and latches on the emergency stop. The raw '
                f'recording is at {raw_path} -- plot it over the saved map and check '
                'for smearing, and whether the course has a corner tighter than this '
                'car can turn.')
        if not prepared.feasible:
            self.get_logger().warn(
                'The racing line needs '
                f'{math.degrees(prepared.max_steering_rad):.1f}deg of steering at its '
                f'tightest point, past the {math.degrees(prepared.max_steering_limit_rad):.1f}deg '
                'budget. The car will understeer there and pure pursuit will pull it '
                'back; expect a wide apex rather than a clean one.')

        # The cleaned recording is a *safe* line, not a fast one -- it is
        # wherever gap_follow happened to drive, with the wobble taken out.
        # Phase 4b re-derives the geometrically fastest line inside the track
        # the map actually shows. It is allowed to fail: on any problem at
        # all this falls back to `prepared`, which is the line that would
        # have been raced before this step existed.
        smoothed = prepared.xy
        optimized = self._optimize_line(prepared, clearance_fn, required_clearance)
        if optimized is not None:
            smoothed = optimized
            racing_math.save_xy_csv(str(run_directory / 'raceline_optimized.csv'),
                                    optimized)

        # Curvature is re-measured from whichever line won. Pacing the
        # optimized geometry with the recorded line's curvature would ask for
        # the old corner speeds on the new corners -- too slow where it was
        # straightened, and too fast nowhere, so the whole point is lost.
        seg_len = racing_math.compute_segment_lengths(smoothed, closed=True)
        curvature = racing_math.estimate_path_curvature(smoothed, closed=True)
        speeds = racing_math.compute_velocity_profile(
            seg_len, curvature,
            v_max=self.profile_max_speed,
            v_min=self.profile_min_speed,
            a_lat_max=self.profile_max_lateral_accel,
            a_accel_max=self.profile_max_accel,
            a_brake_max=self.profile_max_brake,
            closed=True,
            smoothing_passes=self.profile_smoothing_passes,
        )
        racing_math.save_profiled_csv(str(profile_path), smoothed, speeds)
        self.run_directory = run_directory
        self.get_logger().info(
            f'Generated {len(smoothed)}-point racing profile at {profile_path} '
            f'({float(speeds.min()):.2f}-{float(speeds.max()):.2f}m/s).')
        return str(profile_path)

    def _try_load_profile(self):
        if self.profile_request_started or self.profile_path is None:
            return
        # services_are_ready(), NOT service_is_ready(). AsyncParameterClient
        # fronts a whole set of parameter services (get/set/list/describe),
        # so rclpy names its readiness check in the plural -- unlike the
        # single-service rclpy.client.Client used for save_map/serialize_map
        # below, which really is service_is_ready(). Getting this wrong
        # raises AttributeError inside the control loop, which kills the
        # supervisor at the exact moment the mapping laps finish and the
        # profile is handed to pure pursuit.
        if not self.parameter_client.services_are_ready():
            self.get_logger().warn(
                'Waiting for pure_pursuit_node parameter service before loading profile.',
                throttle_duration_sec=2.0)
            return
        self.profile_request_started = True
        future = self.parameter_client.set_parameters([
            Parameter('waypoints_file', Parameter.Type.STRING, self.profile_path)])
        future.add_done_callback(self._profile_loaded_callback)

    def _profile_loaded_callback(self, future):
        try:
            response = future.result()
            result = response.results[0]
        except Exception as exc:
            self.state = 'error'
            self.get_logger().error(f'Failed to call pure pursuit parameter service: {exc}')
            return
        if not result.successful:
            self.state = 'error'
            self.get_logger().error(
                f'Pure pursuit rejected the generated profile: {result.reason}')
            return
        self.race_enable_time = self._now_sec() + self.transition_stop_sec
        self.state = 'transition'
        self.get_logger().info(
            f'Profile loaded successfully; holding a {self.transition_stop_sec:.1f}s stop '
            'before switching to racing.')

    def _map_save_settled(self, now_sec: float) -> bool:
        """True once both SLAM saves have finished (or been given up on).

        Gates the handover: while a save is in flight slam_toolbox is not
        updating map->odom, so nothing downstream should be driving on the
        pose derived from it. On timeout this reports settled anyway and
        says so -- the racing line is already safely on disk, and
        pure_pursuit's own frozen-pose watchdog is the backstop if SLAM is
        genuinely wedged -- but it never reports settled while a save is
        known to still be running.
        """
        if self.map_saves_completed >= self.map_saves_expected:
            return True
        if self.map_save_deadline is None:
            return False
        if now_sec < self.map_save_deadline:
            self.get_logger().info(
                f'Waiting for slam_toolbox to finish saving before racing '
                f'({self.map_saves_completed}/{self.map_saves_expected} done, '
                f'{self.map_save_deadline - now_sec:.1f}s left).',
                throttle_duration_sec=2.0)
            return False
        if not self.map_save_timed_out:
            self.map_save_timed_out = True
            self.get_logger().error(
                f'slam_toolbox map save did not finish within '
                f'{self.map_save_timeout_sec:.0f}s '
                f'({self.map_saves_completed}/{self.map_saves_expected} completed). '
                'Continuing to the racing handover; the racing line is already saved.')
        return True

    def _request_occupancy_map_save(self):
        request = SaveMap.Request()
        request.name.data = str(self.run_directory / 'map')
        self.map_save_attempts += 1
        future = self.save_map_client.call_async(request)
        future.add_done_callback(
            lambda done: self._map_save_callback(done, 'occupancy map'))

    def _try_save_map(self, now_sec: float = None):
        now_sec = self._now_sec() if now_sec is None else now_sec
        if self.map_save_retry_at is not None and now_sec >= self.map_save_retry_at:
            self.map_save_retry_at = None
            self.get_logger().warn(
                f'Retrying the occupancy map save (attempt '
                f'{self.map_save_attempts + 1} of {self.map_save_retries + 1}).')
            self._request_occupancy_map_save()
            return
        if self.map_save_started or not hasattr(self, 'run_directory'):
            return
        if not (self.save_map_client.service_is_ready()
                and self.serialize_map_client.service_is_ready()):
            self.get_logger().warn(
                'Racing profile is ready; waiting for slam_toolbox map-save services.',
                throttle_duration_sec=2.0)
            return
        self.map_save_started = True
        self.map_save_deadline = now_sec + self.map_save_timeout_sec
        graph_request = SerializePoseGraph.Request()
        graph_request.filename = str(self.run_directory / 'posegraph')
        graph_future = self.serialize_map_client.call_async(graph_request)
        graph_future.add_done_callback(
            lambda future: self._map_save_callback(future, 'pose graph'))
        self._request_occupancy_map_save()
        self.get_logger().info(
            f'Requested map and pose-graph save in {self.run_directory}.')

    def _despeckle_saved_map(self):
        """Clear stray-beam blobs out of the map just written to disk.

        Runs only in the `loading_profile` state, where the car is
        deliberately stopped, and only after slam_toolbox has reported the
        save finished -- so this touches no file anything is still writing
        and costs the driving path nothing. Failure is logged and dropped:
        the map is already saved and correct-if-speckled, and a cleanup
        that did not work is not a reason to fail the run.
        """
        if self.map_despeckle_max_cells <= 0:
            return
        if not hasattr(self, 'run_directory'):
            # Same guard _try_save_map uses: run_directory only exists once
            # a profile has been written, and a save reported without one is
            # not this node's map to clean.
            return
        map_yaml = self.run_directory / 'map.yaml'
        try:
            blobs, cells = occupancy_map.despeckle_map_file(
                str(map_yaml), self.map_despeckle_max_cells)
        except Exception as exc:  # noqa: BLE001 - never lose a saved map to cleanup
            self.get_logger().warn(
                f'Could not clean stray returns out of {map_yaml} '
                f'({type(exc).__name__}: {exc}). The map is saved and usable '
                'as it is.')
            return
        if blobs:
            self.get_logger().info(
                f'Cleaned {blobs} stray-return blob(s) ({cells} cells) out of '
                f'{map_yaml} -- small, and with clear observed space all '
                'around, so the LiDAR saw straight through them.')

    def _map_save_callback(self, future, artifact: str):
        # Count the save as settled however it ended: a failed save is still
        # a save that is no longer blocking slam_toolbox's executor, which
        # is the only thing the handover gate actually cares about.
        self.map_saves_completed += 1
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f'Failed to save {artifact}: {exc}')
            result = -1
        if result == 0:
            self.get_logger().info(f'Saved {artifact} successfully.')
            if artifact == 'occupancy map':
                self.map_saved_ok = True
                self._despeckle_saved_map()
            return

        self.get_logger().error(
            f'slam_toolbox failed to save {artifact} (result code {result}).')
        if artifact != 'occupancy map':
            return
        if self.map_save_attempts <= self.map_save_retries:
            # Schedule rather than call: this runs in a service-response
            # callback, and firing the next request from here would stack it
            # on top of the one still unwinding.
            self.map_saves_completed -= 1
            self.map_save_retry_at = self._now_sec() + self.map_save_retry_delay_sec
        else:
            self.get_logger().error(
                f'Giving up on the occupancy map after {self.map_save_attempts} '
                'attempt(s). The pose graph and the racing line are still on '
                'disk; the map can be rebuilt from the pose graph with '
                'slam_toolbox deserialize_map.')


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapRaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort final stop. rclpy.ok() can still flip between the
        # check and the publish -- SIGINT tears the context down on another
        # thread -- and letting that race raise here would make an ordinary
        # Ctrl+C exit non-zero, which ros2 launch reports as
        # "process has died [exit code 1]", indistinguishable in the logs
        # from a real mid-run crash. The mux already brings the car to a
        # stop on /drive timeout, so this publish is a courtesy, not the
        # safety mechanism, and must never be the reason shutdown looks
        # like a failure.
        try:
            if rclpy.ok():
                node._publish_stop()
        except Exception:
            pass
        # The particle filter is a child process in its own session, so it
        # does not get the Ctrl+C that stopped this node. Left running it
        # would hold `map_server`'s service name and quietly break the next
        # run's handover, which is a confusing thing to debug a week later.
        try:
            node._stop_particle_filter()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
