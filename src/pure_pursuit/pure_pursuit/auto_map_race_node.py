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
from time import strftime

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
from pure_pursuit import racing_math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import Joy
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from tf2_ros import Buffer, TransformException, TransformListener


def angle_difference(a: float, b: float) -> float:
    """Smallest signed angular difference a-b, in radians."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class LapRecorder:
    """Distance-sampled map-frame path with conservative loop detection."""

    def __init__(self, spacing: float, min_distance: float,
                 departure_distance: float, closure_distance: float,
                 closure_heading_rad: float, min_duration_sec: float):
        self.spacing = spacing
        self.min_distance = min_distance
        self.departure_distance = departure_distance
        self.closure_distance = closure_distance
        self.closure_heading_rad = closure_heading_rad
        self.min_duration_sec = min_duration_sec
        self.reset()

    def reset(self):
        self.points = []
        self.start = None
        self.start_yaw = None
        self.start_time = None
        self.last_sample = None
        self.distance = 0.0
        self.departed = False
        # Latest closure-gate measurements, exposed for the supervisor's
        # rate-limited progress diagnostics.
        self.elapsed = 0.0
        self.distance_from_start = 0.0
        self.heading_error = 0.0

    def update(self, x: float, y: float, yaw: float, now_sec: float) -> bool:
        point = np.array([x, y], dtype=np.float64)
        if self.start is None:
            self.start = point
            self.start_yaw = yaw
            self.start_time = now_sec
            self.last_sample = point
            self.points.append((x, y))
            return False

        distance_from_last = float(np.linalg.norm(point - self.last_sample))
        if distance_from_last >= self.spacing:
            self.distance += distance_from_last
            self.last_sample = point
            self.points.append((x, y))

        distance_from_start = float(np.linalg.norm(point - self.start))
        if distance_from_start >= self.departure_distance:
            self.departed = True

        self.elapsed = now_sec - self.start_time
        self.distance_from_start = distance_from_start
        self.heading_error = abs(angle_difference(yaw, self.start_yaw))
        return bool(
            self.departed
            and self.distance >= self.min_distance
            and self.elapsed >= self.min_duration_sec
            and self.distance_from_start <= self.closure_distance
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
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('control_rate_hz', 40.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('waypoint_spacing', 0.15)
        self.declare_parameter('mapping_laps', 2)
        self.declare_parameter('minimum_lap_distance', 20.0)
        self.declare_parameter('minimum_lap_duration_sec', 15.0)
        self.declare_parameter('departure_distance', 2.0)
        self.declare_parameter('closure_distance', 0.75)
        self.declare_parameter('closure_heading_deg', 30.0)
        self.declare_parameter('transition_stop_sec', 2.0)
        self.declare_parameter('map_save_timeout_sec', 20.0)
        self.declare_parameter('output_directory', '~/.ros/racerbot_auto')
        self.declare_parameter('profile_max_speed', 4.0)
        self.declare_parameter('profile_min_speed', 0.5)
        self.declare_parameter('profile_max_lateral_accel', 2.5)
        self.declare_parameter('profile_max_accel', 3.0)
        self.declare_parameter('profile_max_brake', 8.0)
        self.declare_parameter('profile_smoothing_window', 3)
        self.declare_parameter('profile_smoothing_passes', 5)
        self.declare_parameter('pure_pursuit_node_name', 'pure_pursuit_node')
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
        self.control_rate_hz = float(value('control_rate_hz'))
        self.command_timeout_sec = float(value('command_timeout_sec'))
        self.mapping_laps = max(1, int(value('mapping_laps')))
        self.transition_stop_sec = float(value('transition_stop_sec'))
        self.map_save_timeout_sec = float(value('map_save_timeout_sec'))
        self.output_directory = Path(os.path.expanduser(str(value('output_directory'))))
        self.profile_max_speed = float(value('profile_max_speed'))
        self.profile_min_speed = float(value('profile_min_speed'))
        self.profile_max_lateral_accel = float(value('profile_max_lateral_accel'))
        self.profile_max_accel = float(value('profile_max_accel'))
        self.profile_max_brake = float(value('profile_max_brake'))
        self.profile_smoothing_window = int(value('profile_smoothing_window'))
        self.profile_smoothing_passes = int(value('profile_smoothing_passes'))
        self.enable_deadman = bool(value('enable_deadman'))
        self.joy_topic = str(value('joy_topic'))
        self.deadman_button = int(value('deadman_button'))
        self.joy_timeout_sec = float(value('joy_timeout_sec'))
        self.decision_log_period_sec = max(
            0.0, float(value('decision_log_period_sec')))

        self.recorder = LapRecorder(
            spacing=float(value('waypoint_spacing')),
            min_distance=float(value('minimum_lap_distance')),
            departure_distance=float(value('departure_distance')),
            closure_distance=float(value('closure_distance')),
            closure_heading_rad=math.radians(float(value('closure_heading_deg'))),
            min_duration_sec=float(value('minimum_lap_duration_sec')),
        )

        self.state = 'mapping'
        self.completed_mapping_laps = 0
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
        self.create_subscription(
            AckermannDriveStamped, self.mapping_drive_topic,
            self._mapping_drive_callback, 10)
        self.create_subscription(
            AckermannDriveStamped, self.racing_drive_topic,
            self._racing_drive_callback, 10)
        if self.enable_deadman:
            self.create_subscription(
                Joy, self.joy_topic, self._joy_callback, 10)

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

    def _lookup_and_publish_pose(self):
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

    def _publish_stop(self):
        self._publish_command(None)

    def _publish_command(self, source):
        output = AckermannDriveStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_link'
        if source is not None:
            output.drive = source.drive
        self.drive_pub.publish(output)

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
            f'{prefix}: samples={len(self.recorder.points)}, '
            f'distance={self.recorder.distance:.1f}/{self.recorder.min_distance:.1f}m, '
            f'elapsed={self.recorder.elapsed:.1f}/{self.recorder.min_duration_sec:.1f}s, '
            f"departed={'yes' if self.recorder.departed else 'no'}, "
            f'start distance={self.recorder.distance_from_start:.2f}/'
            f'{self.recorder.closure_distance:.2f}m, heading error='
            f'{math.degrees(self.recorder.heading_error):.1f}/'
            f'{math.degrees(self.recorder.closure_heading_rad):.1f}deg')

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
        deadman_ok, deadman_state, deadman_detail = self._deadman_status(now_sec)

        if self.state == 'mapping' and pose is not None and deadman_ok:
            if self.recorder.update(*pose, now_sec):
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
            self._try_save_map()
            if self._map_save_settled(now_sec):
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
        self.get_logger().info(
            f'Closed mapping lap {self.completed_mapping_laps}/{self.mapping_laps} detected '
            f'({self.recorder.distance:.1f}m, {len(self.recorder.points)} samples).')
        if self.completed_mapping_laps < self.mapping_laps:
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
        xy = np.asarray(self.recorder.points, dtype=np.float64)
        # Do not retain two nearly identical start/finish points in a closed
        # path; the closing segment is represented implicitly.
        if len(xy) > 3 and np.linalg.norm(xy[-1] - xy[0]) < 1.5 * self.recorder.spacing:
            xy = xy[:-1]
        raw_path = run_directory / 'raceline_raw.csv'
        profile_path = run_directory / 'raceline_profiled.csv'
        racing_math.save_xy_csv(str(raw_path), xy)

        smoothed = racing_math.smooth_path(
            xy, self.profile_smoothing_window, closed=True)
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

    def _try_save_map(self):
        if self.map_save_started or not hasattr(self, 'run_directory'):
            return
        if not (self.save_map_client.service_is_ready()
                and self.serialize_map_client.service_is_ready()):
            self.get_logger().warn(
                'Racing profile is ready; waiting for slam_toolbox map-save services.',
                throttle_duration_sec=2.0)
            return
        self.map_save_started = True
        self.map_save_deadline = self._now_sec() + self.map_save_timeout_sec
        map_base = str(self.run_directory / 'map')
        save_request = SaveMap.Request()
        save_request.name.data = map_base
        graph_request = SerializePoseGraph.Request()
        graph_request.filename = str(self.run_directory / 'posegraph')
        map_future = self.save_map_client.call_async(save_request)
        graph_future = self.serialize_map_client.call_async(graph_request)
        map_future.add_done_callback(
            lambda future: self._map_save_callback(future, 'occupancy map'))
        graph_future.add_done_callback(
            lambda future: self._map_save_callback(future, 'pose graph'))
        self.get_logger().info(
            f'Requested map and pose-graph save in {self.run_directory}.')

    def _map_save_callback(self, future, artifact: str):
        # Count the save as settled however it ended: a failed save is still
        # a save that is no longer blocking slam_toolbox's executor, which
        # is the only thing the handover gate actually cares about.
        self.map_saves_completed += 1
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f'Failed to save {artifact}: {exc}')
            return
        if result == 0:
            self.get_logger().info(f'Saved {artifact} successfully.')
        else:
            self.get_logger().error(
                f'slam_toolbox failed to save {artifact} (result code {result}).')


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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
