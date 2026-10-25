"""neural_racer_node: the learned racing policy, inside its safety wrapper.

THIS NODE CAN MOVE THE CAR. Every rule in CLAUDE.md's safety model applies:

  * it drives only while the LB deadman is held on the F710, checked here,
    independently of every other node (enable_deadman: true in shipped config);
  * shadow mode is the DEFAULT: it publishes only on shadow_topic, never on
    drive_topic, until shadow:=false is passed explicitly;
  * it loads only models approve_model put in the manifest, and drives /drive
    only with a fully approved one.

Each tick runs on scan arrival, as gap_follow does:

    deadman -> input freshness -> observation -> network -> safety wrapper
    -> publish the command -> then, separately, /drive_intent

Any stop publishes a zero command and resets the network's memory.
Design: docs/superpowers/specs/2026-10-23-neural-racer-plan.md (A4, A6, Phase 6).
"""
import json
import math
from pathlib import Path

from ackermann_msgs.msg import AckermannDriveStamped
from drive_intent import schema
from drive_intent.throttle import FailureLatch, IntentThrottle
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, Joy, LaserScan
from std_msgs.msg import String

from neural_racer import model_store, observation as obs, preview
from neural_racer.safety_wrapper import SafetyWrapper, WrapperConfig, scan_angles

# Topics a shadow-mode node must never publish on: each one reaches the motor.
COMMAND_TOPICS = ('/drive', '/teleop', '/ackermann_cmd', '/auto_map/drive', '/auto_race/drive')
COMMAND_PREFIXES = ('/commands/',)


def is_command_topic(topic: str) -> bool:
    return topic in COMMAND_TOPICS or topic.startswith(COMMAND_PREFIXES)


def _quat_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NeuralRacerNode(Node):

    def __init__(self):
        super().__init__('neural_racer_node')
        p = self.declare_parameter
        p('scan_topic', '/scan')
        p('odom_topic', '/odom')
        p('imu_topic', '/imu/data')
        p('imu_status_topic', '/imu/status')
        p('joy_topic', '/joy')
        p('drive_topic', '/drive')
        p('shadow', True)
        p('shadow_topic', '/neural_racer/shadow_drive')
        p('intent_topic', '/drive_intent')
        p('enable_deadman', True)
        p('deadman_button', 4)
        p('joy_timeout_sec', 0.5)
        p('models_dir', str(model_store.DEFAULT_MODELS_DIR))
        p('model_hash', '')
        p('max_speed', 1.0)
        p('mode', 'reactive')                 # reactive | map | auto
        p('require_imu', True)
        p('camera_enabled', False)
        p('pose_topic', '/slam_pose')
        p('path_topic', '/auto_map_race/race_path')
        p('frozen_map_topic', '/auto_map_race/frozen_map')
        p('scan_timeout_sec', 0.075)
        p('odom_timeout_sec', 0.1)
        p('imu_timeout_sec', 0.1)
        p('pose_timeout_sec', 0.2)
        p('check_scan_stamp', True)
        p('intent_rate_hz', 10.0)
        defaults = WrapperConfig()
        for name in ('max_steering_angle', 'car_width', 'car_length', 'wheelbase',
                     'laser_offset_x', 'reserve_distance', 'max_braking_decel',
                     'braking_latency_sec', 'max_lateral_accel', 'max_steering_rate',
                     'command_slew_max_dt', 'max_invalid_fraction'):
            p(name, getattr(defaults, name))

        v = lambda name: self.get_parameter(name).value   # noqa: E731
        self.joy_topic = v('joy_topic')
        self.drive_topic = v('drive_topic')
        self.shadow = bool(v('shadow'))
        self.shadow_topic = v('shadow_topic')
        self.enable_deadman = bool(v('enable_deadman'))
        self.deadman_button = int(v('deadman_button'))
        self.joy_timeout_sec = float(v('joy_timeout_sec'))
        self.max_speed = float(v('max_speed'))
        self.mode_param = v('mode')
        self.require_imu = bool(v('require_imu'))
        self.camera_enabled = bool(v('camera_enabled'))
        self.scan_timeout_sec = float(v('scan_timeout_sec'))
        self.odom_timeout_sec = float(v('odom_timeout_sec'))
        self.imu_timeout_sec = float(v('imu_timeout_sec'))
        self.pose_timeout_sec = float(v('pose_timeout_sec'))
        self.check_scan_stamp = bool(v('check_scan_stamp'))
        self.wrapper = SafetyWrapper(WrapperConfig(
            max_speed=self.max_speed,
            **{name: float(v(name)) for name in (
                'max_steering_angle', 'car_width', 'car_length', 'wheelbase',
                'laser_offset_x', 'reserve_distance', 'max_braking_decel',
                'braking_latency_sec', 'max_lateral_accel', 'max_steering_rate',
                'command_slew_max_dt', 'max_invalid_fraction')}))

        # --- refusals: the node then publishes only zero commands ------------
        self.refusal = None
        out_topic = self.shadow_topic if self.shadow else self.drive_topic
        if self.shadow and is_command_topic(self.shadow_topic):
            self.refusal = f"shadow_topic '{self.shadow_topic}' reaches the motor"
            out_topic = None                   # publish nothing anywhere
        elif self.mode_param not in ('reactive', 'map', 'auto'):
            self.refusal = f'unknown mode {self.mode_param!r}'
        elif self.camera_enabled:
            self.refusal = 'camera_enabled: the camera input is not implemented yet (Phase C)'
        elif not (math.isfinite(self.max_speed) and 0.0 <= self.max_speed <= 6.37):
            self.refusal = f'max_speed {self.max_speed} outside [0, 6.37] m/s (firmware cap)'
        self.policy, self.card = None, {}
        if self.refusal is None:
            try:
                self.policy, self.card, _ = model_store.load_approved(
                    v('model_hash'), models_dir=Path(v('models_dir')),
                    camera_enabled=self.camera_enabled, driving=not self.shadow)
            except (model_store.ModelStoreError, OSError, ValueError, KeyError) as exc:
                self.refusal = f'model refused: {exc}'
        if self.refusal is not None:
            self.get_logger().error(f'REFUSING TO DRIVE: {self.refusal}')

        self.cmd_pub = (None if out_topic is None
                        else self.create_publisher(AckermannDriveStamped, out_topic, 10))
        self.intent_pub = self.create_publisher(String, v('intent_topic'), 10)
        self._intent_throttle = IntentThrottle(float(v('intent_rate_hz')), 1.0)
        self._intent_failures = FailureLatch()

        # --- state -------------------------------------------------------------
        self.history = obs.History()
        self.last_joy_time = None
        self.joy_button_available = False
        self.deadman_held = False
        self.current_speed = math.nan
        self.last_odom_time = None
        self.imu_sample = None
        self.last_imu_time = None
        self.imu_status_ok = False
        self.pose = None
        self.last_pose_time = None
        self.ref_path = None
        self.grid = None
        self._path_msg = None
        self._path_hint = None
        self.prev_steering = 0.0
        self.last_command_time = None
        self.last_decision_state = 'starting'

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(LaserScan, v('scan_topic'), self.scan_callback, 10)
        self.create_subscription(Odometry, v('odom_topic'), self.odom_callback, 10)
        self.create_subscription(Imu, v('imu_topic'), self.imu_callback, 10)
        self.create_subscription(String, v('imu_status_topic'), self.imu_status_callback, latched)
        self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
        self.create_subscription(PoseStamped, v('pose_topic'), self.pose_callback, 10)
        self.create_subscription(PathMsg, v('path_topic'), self.path_callback, latched)
        self.create_subscription(OccupancyGrid, v('frozen_map_topic'), self.map_callback, latched)
        self.get_logger().info(
            f"neural_racer_node: {'SHADOW -> ' + self.shadow_topic if self.shadow else 'DRIVING -> ' + self.drive_topic}"
            f", mode={self.mode_param}, max_speed={self.max_speed:.2f} m/s")

    # --- inputs ------------------------------------------------------------------

    def _age(self, stamp) -> float:
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        self.joy_button_available = len(msg.buttons) > self.deadman_button
        self.deadman_held = bool(self.joy_button_available and msg.buttons[self.deadman_button])

    def odom_callback(self, msg: Odometry):
        self.current_speed = float(msg.twist.twist.linear.x)
        self.last_odom_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        self.imu_sample = (float(msg.angular_velocity.z), float(msg.linear_acceleration.x),
                           float(msg.linear_acceleration.y))
        self.last_imu_time = self.get_clock().now()

    def imu_status_callback(self, msg: String):
        try:
            self.imu_status_ok = json.loads(msg.data).get('state') == 'ok'
        except (ValueError, AttributeError):
            self.imu_status_ok = False

    def pose_callback(self, msg: PoseStamped):
        p = msg.pose
        self.pose = (float(p.position.x), float(p.position.y), _quat_yaw(p.orientation))
        self.last_pose_time = self.get_clock().now()

    def path_callback(self, msg: PathMsg):
        self._path_msg = msg
        self._build_reference()

    def map_callback(self, msg: OccupancyGrid):
        info = msg.info
        q = info.origin.orientation
        if abs(_quat_yaw(q)) > 1e-6:
            self.get_logger().error('frozen map has a rotated origin; map mode unavailable')
            return
        self.grid = preview.GridMap.from_occupancy(
            msg.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y)
        self._build_reference()

    def _build_reference(self):
        if self._path_msg is None or self.grid is None:
            return
        xy = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in self._path_msg.poses])
        try:
            self.ref_path = preview.ReferencePath.from_grid(xy, self.grid)
            self._path_hint = None
        except preview.PreviewError as exc:
            self.ref_path = None
            self.get_logger().error(f'reference path unusable: {exc}')

    def _deadman_status(self):
        """(engaged, reason) -- the same rules as gap_follow_node."""
        if not self.enable_deadman:
            return True, None
        if self.last_joy_time is None:
            return False, 'waiting_for_joy'
        if self._age(self.last_joy_time) >= self.joy_timeout_sec:
            return False, 'joy_stale'
        if not self.joy_button_available:
            return False, 'deadman_button_missing'
        if not self.deadman_held:
            return False, 'deadman_released'
        return True, None

    # --- the tick ------------------------------------------------------------------

    def _publish(self, steering: float, speed: float):
        if self.cmd_pub is None:
            return
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.cmd_pub.publish(msg)

    def _stop(self, reason: str):
        self._publish(0.0, 0.0)
        self.history.reset()
        if self.policy is not None:
            self.policy.reset()
        self.prev_steering = 0.0
        self.last_command_time = self.get_clock().now()
        if reason != self.last_decision_state:
            self.get_logger().info(f'stopped: {reason}')
        self.last_decision_state = reason

    def _map_mode(self) -> bool:
        ready = (self.ref_path is not None and self.grid is not None and self.pose is not None
                 and self._age(self.last_pose_time) < self.pose_timeout_sec)
        return self.mode_param == 'map' or (self.mode_param == 'auto' and ready)

    def scan_callback(self, scan: LaserScan):
        engaged, reason = self._deadman_status()
        if not engaged:
            return self._stop(reason)
        if self.refusal is not None:
            return self._stop('refused')
        stamp = Time.from_msg(scan.header.stamp)
        if self.check_scan_stamp and self._age(stamp) > self.scan_timeout_sec:
            return self._stop('stale_scan')
        if self._age(self.last_odom_time) >= self.odom_timeout_sec or not math.isfinite(self.current_speed):
            return self._stop('stale_odom')
        imu_fresh = (self.imu_status_ok and self.imu_sample is not None
                     and self._age(self.last_imu_time) < self.imu_timeout_sec)
        if self.require_imu and not imu_fresh:
            return self._stop('imu_unavailable')
        map_mode = self._map_mode()
        if map_mode and (self.ref_path is None or self.grid is None or self.pose is None
                         or self._age(self.last_pose_time) >= self.pose_timeout_sec):
            return self._stop('map_inputs_unavailable')

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        try:
            encoded, bins = obs.pool_scan(ranges, scan.range_min, scan.range_max)
            self.history.push_tick(encoded, bins, self.current_speed,
                                   self.imu_sample if imu_fresh else None)
            prev_block = race = None
            if map_mode:
                x, y, yaw = self.pose
                prev_block, self._path_hint, s0 = self.ref_path.preview_block(x, y, yaw, self._path_hint)
                race = obs.race_context(s0 / self.ref_path.length, 1, None, None)
            o = obs.build_observation(
                self.history, imu_present=imu_fresh and self.history.imu_complete(),
                mode=obs.MODE_MAP if map_mode else obs.MODE_REACTIVE, v_cap=self.max_speed,
                preview=prev_block, race=race)
        except (obs.ObservationError, preview.PreviewError):
            return self._stop('bad_observation')

        out = self.policy.step(o)
        map_hit = None
        if map_mode and np.all(np.isfinite(out.ego_path)):
            x, y, yaw = self.pose
            world = preview.car_to_world(np.vstack([[0.0, 0.0], out.ego_path]), x, y, yaw)
            map_hit = self.grid.first_blocked_along(world)
        dt = (0.025 if self.last_command_time is None
              else self._age(self.last_command_time))
        angles = scan_angles(scan.angle_min, scan.angle_increment, ranges.size)
        res = self.wrapper.apply(out.steering, out.speed, out.ego_path, ranges, angles,
                                 scan.range_min, scan.range_max, self.prev_steering, dt, map_hit)
        if res.stopped:
            return self._stop(res.reason)

        self._publish(res.steering, res.speed)                # the command, first
        self.history.push_command(res.steering, res.speed)
        self.prev_steering = res.steering
        self.last_command_time = self.get_clock().now()
        self.last_decision_state = 'map' if map_mode else 'reactive'
        self._publish_intent(out, res, map_mode)              # diagnostics, after

    def _publish_intent(self, out, res, map_mode: bool):
        """After the command, inside one try/except that disables the
        diagnostic rather than the node (docs/drive-intent.md)."""
        if self._intent_failures.disabled:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if not self._intent_throttle.should_publish(now):
            return
        try:
            payload = schema.build(
                'neural_racer_node', self.last_decision_state,
                # drive_intent points are (x, y, v); the forecast carries no
                # speed of its own, so each point gets the commanded speed.
                path=[(0.0, 0.0, res.speed)] + [(float(px), float(py), res.speed)
                                                for px, py in out.ego_path],
                desired_steering=out.steering, commanded_steering=res.steering,
                desired_speed=out.speed, commanded_speed=res.speed,
                horizon_s=obs.AUX_DT * obs.AUX_PATH_POINTS,
                factors=[schema.factor(res.binding, res.speed, binding=True)])
            self.intent_pub.publish(String(data=schema.encode(payload)))
            self._intent_failures.record_success()
        except Exception as exc:   # noqa: BLE001 -- a diagnostic must never stop the car
            if self._intent_failures.record_failure():
                self.get_logger().error(f'/drive_intent disabled after repeated failures: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = NeuralRacerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
