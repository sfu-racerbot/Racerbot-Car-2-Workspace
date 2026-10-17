"""Read-only ROS telemetry backend and Web server for the calibration wizard."""

from __future__ import annotations

from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

import tornado.ioloop
import tornado.web
import tornado.websocket
import yaml

from odom_calibration import calibration_math
from odom_calibration import steering_chain
from odom_calibration import teleop_yaml
from odom_calibration import vesc_yaml
from odom_calibration import wizard_actions
from odom_calibration.session_store import (
    CAPTURE_KINDS,
    SessionStore,
    VALID_STAGES,
    capture_allowed,
    new_session,
    stage_allowed,
    touch,
)


PARAMETER_NAMES = (
    'speed_to_erpm_gain',
    'speed_to_erpm_offset',
    'steering_angle_to_servo_gain',
    'steering_angle_to_servo_offset',
    'wheelbase',
)
# Where each live value is read from: the node that actually uses it. The
# servo command comes from ackermann_to_vesc_node and the clamp from
# vesc_driver_node, so a value edited for one node but not restarted in the
# other shows up as a mismatch instead of hiding behind the odom node's copy.
# A rear track outside this (m) is a typing or unit mistake for a car this size.
REAR_TRACK_RANGE_M = (0.10, 0.60)
# A live-parameter request older than this is given up and sent again.
PARAMETER_REQUEST_TIMEOUT_SEC = 5.0
STICK_SCALE_PARAMETER = 'human_control.axis_mappings.drive-steering_angle.scale'
LIVE_SOURCES = (
    ('vesc_to_odom_node',
     ('speed_to_erpm_gain', 'speed_to_erpm_offset', 'wheelbase')),
    ('ackermann_to_vesc_node',
     ('steering_angle_to_servo_gain', 'steering_angle_to_servo_offset')),
    ('vesc_driver_node', ('servo_min', 'servo_max')),
    # joy_teleop declares its YAML as parameters; this one is full stick.
    ('joy_teleop', (STICK_SCALE_PARAMETER,)),
)


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) \
        and math.isfinite(float(value))


def _number_or_none(value):
    return float(value) if _finite(value) else None


def _header_seconds(msg):
    header = getattr(msg, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    seconds = getattr(stamp, 'sec', None)
    nanoseconds = getattr(stamp, 'nanosec', None)
    if seconds is None or nanoseconds is None:
        return None
    value = float(seconds) + float(nanoseconds) * 1e-9
    return value if math.isfinite(value) else None


def _yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WizardError(Exception):
    def __init__(self, message, status=400, details=None):
        super().__init__(message)
        self.status = status
        self.details = details or {}


class TopicMonitor:
    def __init__(self, label):
        self.label = label
        self.receipts = deque(maxlen=256)
        self.count = 0
        self.invalid_count = 0
        self.last_header_stamp = None
        self.header_regressions = 0

    def update(self, receipt_time, header_stamp=None, valid=True):
        self.receipts.append(float(receipt_time))
        self.count += 1
        if not valid:
            self.invalid_count += 1
        if header_stamp is not None:
            if (
                self.last_header_stamp is not None
                and header_stamp + 1e-9 < self.last_header_stamp
            ):
                self.header_regressions += 1
            self.last_header_stamp = header_stamp

    def snapshot(self, now, stale_sec):
        if not self.receipts:
            return {
                'label': self.label,
                'status': 'missing',
                'age_sec': None,
                'rate_hz': 0.0,
                'message_count': self.count,
                'invalid_count': self.invalid_count,
                'header_regressions': self.header_regressions,
            }
        age = max(0.0, now - self.receipts[-1])
        if len(self.receipts) >= 2:
            span = self.receipts[-1] - self.receipts[0]
            rate = (len(self.receipts) - 1) / span if span > 0.0 else 0.0
        else:
            rate = 0.0
        recent_invalid = self.invalid_count > 0 and self.count <= 10
        if age > stale_sec:
            status = 'stale'
        elif recent_invalid or self.header_regressions:
            status = 'warning'
        else:
            status = 'good'
        return {
            'label': self.label,
            'status': status,
            'age_sec': age,
            'rate_hz': rate,
            'message_count': self.count,
            'invalid_count': self.invalid_count,
            'header_regressions': self.header_regressions,
        }


class CaptureRecorder:
    def __init__(self, kind, max_samples):
        self.kind = kind
        self.max_samples = int(max_samples)
        self.progress = {
            'odom_distance_m': calibration_math.RunningIntegral(),
            'odom_yaw_rad': calibration_math.RunningIntegral(),
        }
        self.started_monotonic = time.monotonic()
        self.started_at = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        self.samples = {
            'odom': [],
            'vesc': [],
            'servo': [],
            'servo_cmd': [],
            'drive': [],
            'joy': [],
        }
        self.dropped_samples = {name: 0 for name in self.samples}
        self.truncated_topics = set()

    def add(self, topic, sample):
        values = self.samples[topic]
        if len(values) >= self.max_samples:
            self.dropped_samples[topic] += 1
            self.truncated_topics.add(topic)
            return
        values.append(sample)

    def finish(self):
        return {
            'kind': self.kind,
            'started_monotonic': self.started_monotonic,
            'ended_monotonic': time.monotonic(),
            'started_at': self.started_at,
            **self.samples,
            'dropped_samples': self.dropped_samples,
            'truncated_topics': sorted(self.truncated_topics),
        }


class WizardWebSocket(tornado.websocket.WebSocketHandler):
    def initialize(self, node):
        self.node = node

    def check_origin(self, origin):
        # LAN-local, telemetry-only ROS access. The API can alter calibration
        # notes/sessions but this node owns no publishers and cannot move a car.
        return True

    def open(self):
        self.node.ws_clients.add(self)
        self.write_message(json.dumps(
            {'type': 'snapshot', **self.node.snapshot()},
            allow_nan=False,
        ))

    def on_close(self):
        self.node.ws_clients.discard(self)

    def on_message(self, message):
        # State-changing input uses the validated JSON API. The socket only
        # carries live telemetry and accepts a harmless ping.
        if message == 'ping':
            self.write_message('{"type":"pong"}')


class JsonHandler(tornado.web.RequestHandler):
    def initialize(self, node):
        self.node = node

    def set_default_headers(self):
        self.set_header('Cache-Control', 'no-store')

    def write_json(self, payload, status=200):
        self.set_status(status)
        self.set_header('Content-Type', 'application/json')
        self.finish(json.dumps(payload, allow_nan=False))

    def parse_json(self):
        # A browser sends application/json cross-origin only after a CORS
        # preflight, which this server never approves. Refusing every other
        # content type stops another web page from posting a "simple"
        # text/plain request at the car, now that an action can edit vesc.yaml.
        content_type = self.request.headers.get('Content-Type', '')
        if content_type.split(';')[0].strip().lower() != 'application/json':
            raise WizardError('Requests must be sent as application/json.', status=415)
        try:
            payload = json.loads(self.request.body.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise WizardError('Request body must be valid JSON.') from exc
        if not isinstance(payload, dict):
            raise WizardError('JSON request must be an object.')
        return payload


class StateHandler(JsonHandler):
    def get(self):
        self.write_json(self.node.snapshot())


class ActionHandler(JsonHandler):
    def post(self):
        try:
            result = self.node.handle_action(self.parse_json())
            self.write_json({'ok': True, **result})
        except WizardError as exc:
            self.write_json({
                'ok': False,
                'error': str(exc),
                'details': exc.details,
            }, status=exc.status)
        except Exception as exc:  # Defensive API boundary; log full exception.
            self.node.get_logger().error(
                f'Unexpected wizard action failure: {exc}')
            self.write_json({
                'ok': False,
                'error': f'Unexpected server error: {exc}',
            }, status=500)


class DownloadHandler(tornado.web.RequestHandler):
    def initialize(self, node):
        self.node = node

    def get(self, report_format):
        try:
            content, content_type, filename = self.node.report_download(
                report_format)
        except WizardError as exc:
            self.set_status(exc.status)
            self.finish(str(exc))
            return
        self.set_header('Content-Type', content_type)
        self.set_header(
            'Content-Disposition', f'attachment; filename="{filename}"')
        self.set_header('Cache-Control', 'no-store')
        self.finish(content)


class NoCacheStaticFileHandler(tornado.web.StaticFileHandler):
    """Static wizard files, revalidated on every load so an updated UI is
    never mixed with a cached older script or stylesheet."""

    def set_extra_headers(self, path):
        self.set_header('Cache-Control', 'no-cache')


class OdomCalibrationNode(Node):
    """Collect telemetry, own wizard state, and never publish to ROS."""

    def __init__(self):
        super().__init__('odom_calibration_node')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('vesc_state_topic', '/sensors/core')
        self.declare_parameter(
            'servo_topic', '/sensors/servo_position_command')
        self.declare_parameter(
            'servo_command_topic', '/commands/servo/position')
        self.declare_parameter('drive_topic', '/ackermann_cmd')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('steering_axis', 3)
        self.declare_parameter('vesc_yaml_path', '')
        self.declare_parameter('joy_teleop_yaml_path', '')
        self.declare_parameter('rear_track_m', 0.27)
        self.declare_parameter('allow_apply', True)
        self.declare_parameter('planner_max_steering_rad', 0.349)
        self.declare_parameter('vehicle_model', 'unnamed car')
        self.declare_parameter('physical_width_m', 0.30)
        self.declare_parameter('physical_length_m', 0.535)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8090)
        self.declare_parameter('telemetry_rate_hz', 5.0)
        self.declare_parameter('topic_stale_sec', 0.75)
        self.declare_parameter('max_capture_sec', 300.0)
        self.declare_parameter('max_samples_per_topic', 30000)
        self.declare_parameter(
            'report_directory', '~/.ros/odom_calibration')
        self.declare_parameter('speed_to_erpm_gain', -4614.0)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter(
            'steering_angle_to_servo_gain', -0.78)
        self.declare_parameter(
            'steering_angle_to_servo_offset', 0.5904)
        self.declare_parameter('wheelbase', 0.36)
        self.declare_parameter('servo_min', 0.15)
        self.declare_parameter('servo_max', 0.95)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.vesc_state_topic = self.get_parameter('vesc_state_topic').value
        self.servo_topic = self.get_parameter('servo_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.joy_topic = self.get_parameter('joy_topic').value
        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.steering_axis = int(self.get_parameter('steering_axis').value)
        self.servo_command_topic = self.get_parameter(
            'servo_command_topic').value
        self.allow_apply = bool(self.get_parameter('allow_apply').value)
        self.vesc_yaml_path = self._resolve_config(
            self.get_parameter('vesc_yaml_path').value, 'vesc.yaml')
        self.joy_teleop_yaml_path = self._resolve_config(
            self.get_parameter('joy_teleop_yaml_path').value, 'joy_teleop.yaml')
        self.vehicle_settings = {
            'model': str(self.get_parameter('vehicle_model').value),
            'physical_width_m': float(
                self.get_parameter('physical_width_m').value),
            'physical_length_m': float(
                self.get_parameter('physical_length_m').value),
            'planner_max_steering_rad': float(
                self.get_parameter('planner_max_steering_rad').value),
            'rear_track_m': float(self.get_parameter('rear_track_m').value),
        }
        if not (math.isfinite(self.vehicle_settings['rear_track_m'])
                and self.vehicle_settings['rear_track_m'] > 0.0):
            raise ValueError('rear_track_m must be positive')
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.telemetry_rate_hz = max(
            1.0, float(self.get_parameter('telemetry_rate_hz').value))
        self.topic_stale_sec = max(
            0.1, float(self.get_parameter('topic_stale_sec').value))
        self.max_capture_sec = max(
            5.0, float(self.get_parameter('max_capture_sec').value))
        self.max_samples_per_topic = max(
            100, int(self.get_parameter('max_samples_per_topic').value))

        self.default_parameters = {
            name: float(self.get_parameter(name).value)
            for name in PARAMETER_NAMES
        }
        default_gain = self.default_parameters['speed_to_erpm_gain']
        if not math.isfinite(default_gain) or default_gain == 0.0:
            raise ValueError('speed_to_erpm_gain must be finite and nonzero')
        if self.default_parameters['wheelbase'] <= 0.0:
            raise ValueError('wheelbase must be positive')
        self.servo_min = float(self.get_parameter('servo_min').value)
        self.servo_max = float(self.get_parameter('servo_max').value)
        if (
            not math.isfinite(self.servo_min)
            or not math.isfinite(self.servo_max)
            or self.servo_min >= self.servo_max
        ):
            raise ValueError('servo_min must be below servo_max')

        report_directory = self.get_parameter('report_directory').value
        try:
            self.store = SessionStore(report_directory)
        except OSError as exc:
            fallback = Path('/tmp/odom_calibration')
            self.get_logger().warning(
                f"Cannot use report_directory '{report_directory}' ({exc}); "
                f"falling back to '{fallback}'.")
            self.store = SessionStore(fallback)

        self._lock = threading.RLock()
        self.session = self.store.load()
        self.active_recorder = None
        self.latest = {
            'odom_speed': None,
            'odom_angular_z': None,
            'odom_pose': None,
            'raw_forward_erpm': None,
            'servo': None,
            'servo_command': None,
            'steer_axis': None,
            'command_speed': None,
            'command_steering': None,
            'lb_held': False,
            'joy_button_available': False,
        }
        self.live_parameters = dict(self.default_parameters)
        self.live_servo_limits = [self.servo_min, self.servo_max]
        # Full stick (rad): joy_teleop's live value once read, else the file.
        self.live_stick_scale = self._file_stick_scale()
        self.live_parameter_sources = {
            node: 'not read yet (using configured fallback)'
            for node, _ in LIVE_SOURCES
        }
        self.live_parameter_status = 'configured fallback'
        self.monitors = {
            'odom': TopicMonitor('Odometry'),
            'vesc': TopicMonitor('Raw VESC'),
            'servo': TopicMonitor('Servo command (after clamp)'),
            'servo_cmd': TopicMonitor('Servo command (before clamp)'),
            'drive': TopicMonitor('Selected drive command'),
            'joy': TopicMonitor('Remote / LB'),
        }
        self.ws_clients = set()
        self._loop = None
        # node -> when its parameter request was sent (monotonic seconds).
        self._parameter_requests_inflight = {}

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.vesc_sub = self.create_subscription(
            VescStateStamped,
            self.vesc_state_topic,
            self.vesc_callback,
            qos_profile_sensor_data,
        )
        self.servo_sub = self.create_subscription(
            Float64, self.servo_topic, self.servo_callback, 10)
        self.servo_cmd_sub = self.create_subscription(
            Float64, self.servo_command_topic, self.servo_command_callback, 10)
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, self.drive_topic, self.drive_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, 10)

        # Polled for the node's whole life, not just until the first answer:
        # bringup is restarted after every vesc.yaml change, and the wizard
        # must notice the new values rather than keep reporting the old ones.
        self.parameter_clients = {
            node: AsyncParameterClient(self, node) for node, _ in LIVE_SOURCES
        }
        self.parameter_timer = self.create_timer(
            2.0, self._request_live_parameters)
        self.telemetry_timer = self.create_timer(
            1.0 / self.telemetry_rate_hz, self._telemetry_tick)

        self.get_logger().info(
            'Odometry calibration wizard creates no ROS publishers and cannot '
            'command the car. '
            + (f'After a confirmation in the page it can write calibration '
               f'values into {self.vesc_yaml_path} (backup first). '
               if self.allow_apply and self.vesc_yaml_path else
               'Writing vesc.yaml is off. ')
            + f'Open http://<this-car-IP>:{self.port}/ after the web server starts.'
        )

    # ROS callbacks ---------------------------------------------------------

    def odom_callback(self, msg):
        now = time.monotonic()
        speed = float(msg.twist.twist.linear.x)
        angular_z = float(msg.twist.twist.angular.z)
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = _yaw_from_quaternion(orientation)
        valid = all(math.isfinite(value) for value in (
            speed, angular_z, position.x, position.y, yaw))
        with self._lock:
            self.monitors['odom'].update(
                now, _header_seconds(msg), valid=valid)
            self.latest['odom_speed'] = _number_or_none(speed)
            self.latest['odom_angular_z'] = _number_or_none(angular_z)
            self.latest['odom_pose'] = (
                {
                    'x': float(position.x),
                    'y': float(position.y),
                    'yaw': yaw,
                }
                if valid else None
            )
            if self.active_recorder:
                self.active_recorder.add('odom', {
                    't': now,
                    'speed': speed,
                    'angular_z': angular_z,
                    'x': float(position.x),
                    'y': float(position.y),
                    'yaw': yaw,
                })
                self.active_recorder.progress['odom_distance_m'].add(now, speed)
                self.active_recorder.progress['odom_yaw_rad'].add(now, angular_z)

    def vesc_callback(self, msg):
        now = time.monotonic()
        # vesc_to_odom.cpp uses -state.speed as forward-positive ERPM.
        raw_forward_erpm = -float(msg.state.speed)
        valid = math.isfinite(raw_forward_erpm)
        with self._lock:
            self.monitors['vesc'].update(
                now, _header_seconds(msg), valid=valid)
            self.latest['raw_forward_erpm'] = _number_or_none(
                raw_forward_erpm)
            if self.active_recorder:
                self.active_recorder.add('vesc', {
                    't': now,
                    'raw_forward_erpm': raw_forward_erpm,
                })

    def servo_callback(self, msg):
        now = time.monotonic()
        value = float(msg.data)
        valid = math.isfinite(value)
        with self._lock:
            self.monitors['servo'].update(now, valid=valid)
            self.latest['servo'] = _number_or_none(value)
            if self.active_recorder:
                self.active_recorder.add(
                    'servo', {'t': now, 'value': value})

    def servo_command_callback(self, msg):
        now = time.monotonic()
        value = float(msg.data)
        valid = math.isfinite(value)
        with self._lock:
            self.monitors['servo_cmd'].update(now, valid=valid)
            self.latest['servo_command'] = _number_or_none(value)
            if self.active_recorder:
                self.active_recorder.add(
                    'servo_cmd', {'t': now, 'value': value})

    def drive_callback(self, msg):
        now = time.monotonic()
        speed = float(msg.drive.speed)
        steering = float(msg.drive.steering_angle)
        valid = math.isfinite(speed) and math.isfinite(steering)
        with self._lock:
            self.monitors['drive'].update(
                now, _header_seconds(msg), valid=valid)
            self.latest['command_speed'] = _number_or_none(speed)
            self.latest['command_steering'] = _number_or_none(steering)
            if self.active_recorder:
                self.active_recorder.add('drive', {
                    't': now,
                    'speed': speed,
                    'steering': steering,
                })

    def joy_callback(self, msg):
        now = time.monotonic()
        available = len(msg.buttons) > self.deadman_button
        held = bool(msg.buttons[self.deadman_button]) if available else False
        axis = (
            _number_or_none(float(msg.axes[self.steering_axis]))
            if len(msg.axes) > self.steering_axis else None)
        with self._lock:
            self.monitors['joy'].update(
                now, _header_seconds(msg), valid=available)
            self.latest['lb_held'] = held
            self.latest['joy_button_available'] = available
            self.latest['steer_axis'] = axis
            if self.active_recorder:
                self.active_recorder.add('joy', {
                    't': now,
                    'lb_held': held,
                    'steer_axis': axis,
                })

    # Live parameter discovery ---------------------------------------------

    def _request_live_parameters(self):
        for node, names in LIVE_SOURCES:
            sent = self._parameter_requests_inflight.get(node)
            # A request to a node that was killed (a bringup restart) never
            # completes; without the age limit that node is never asked again
            # and the wizard keeps reporting its old values.
            if sent is not None and time.monotonic() - sent < PARAMETER_REQUEST_TIMEOUT_SEC:
                continue
            client = self.parameter_clients[node]
            if not client.services_are_ready():
                with self._lock:
                    if self.live_parameter_sources[node].startswith('read'):
                        self.live_parameter_sources[node] = (
                            'not running (last values kept)')
                continue
            self._parameter_requests_inflight[node] = time.monotonic()
            future = client.get_parameters(list(names))
            future.add_done_callback(
                lambda done, node=node, names=names:
                    self._live_parameters_done(node, names, done))

    def _live_parameters_done(self, node, names, future):
        self._parameter_requests_inflight.pop(node, None)
        try:
            values = calibration_math.parameter_values_from_response(
                names, future.result())
        except Exception as exc:
            with self._lock:
                self.live_parameter_sources[node] = f'query failed: {exc}'
            return
        with self._lock:
            for name, value in values.items():
                if name == STICK_SCALE_PARAMETER:
                    self.live_stick_scale = value
                elif name == 'servo_min':
                    self.live_servo_limits[0] = value
                elif name == 'servo_max':
                    self.live_servo_limits[1] = value
                else:
                    self.live_parameters[name] = value
            self.live_parameter_sources[node] = 'read'
            read = [n for n, s in self.live_parameter_sources.items() if s == 'read']
            self.live_parameter_status = (
                f"read from {', '.join(read)}" if read else 'configured fallback')
            self._sync_session_to_car()

    def _sync_session_to_car(self):
        """Follow the car's live values; called with the lock held.

        After a vesc.yaml change and a bringup restart the car runs new
        values. The session's "current" values must follow, or the report
        compares suggestions against settings the car no longer has.
        """
        if self.session is None or self.active_recorder:
            return
        changed = wizard_actions.sync_session_parameters(
            self.session, self.live_parameters, self.live_servo_limits,
            self.live_stick_scale)
        if changed:
            touch(self.session, 'parameters_changed_on_car', '; '.join(changed))
            self.store.save(self.session)
            self._schedule_broadcast()

    @staticmethod
    def _resolve_config(configured, name):
        """The f1tenth_stack config file the car really loads, in src/.

        With ``colcon build --symlink-install`` the installed config is a
        symlink into src/, so resolving it lands on the git-tracked file.
        """
        if configured:
            return Path(os.path.expanduser(configured)).resolve()
        try:
            share = get_package_share_directory('f1tenth_stack')
        except Exception:  # Package not built: apply stays unavailable.
            return None
        return Path(share, 'config', name).resolve()

    def _file_stick_scale(self):
        if self.joy_teleop_yaml_path is None:
            return None
        try:
            return teleop_yaml.read_scale(
                self.joy_teleop_yaml_path.read_text(encoding='utf-8'))
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            self.get_logger().warning(
                f'Cannot read the steering scale from {self.joy_teleop_yaml_path}: {exc}')
            return None

    # Session and capture actions ------------------------------------------

    def _validated_parameters(self, supplied):
        merged = dict(self.live_parameters)
        if supplied is not None:
            if not isinstance(supplied, dict):
                raise WizardError('current_parameters must be an object.')
            for name in PARAMETER_NAMES:
                if name in supplied:
                    value = supplied[name]
                    if not _finite(value):
                        raise WizardError(f'{name} must be a finite number.')
                    merged[name] = float(value)
        if merged['speed_to_erpm_gain'] == 0.0:
            raise WizardError('speed_to_erpm_gain must be finite and nonzero.')
        if merged['wheelbase'] <= 0.0:
            raise WizardError('wheelbase must be positive.')
        return merged

    def _require_session(self):
        if self.session is None:
            raise WizardError('Create a calibration session first.', status=409)

    def _save_and_snapshot(self):
        self.store.save(self.session)
        self._schedule_broadcast()
        return {'state': self.snapshot()}

    def handle_action(self, payload):
        action = payload.get('action')
        with self._lock:
            if action == 'new_session':
                if (
                    self.session is not None
                    and not payload.get('replace_existing', False)
                ):
                    raise WizardError(
                        'An active session already exists. Confirm replacement.',
                        status=409,
                        details={'requires_confirmation': True},
                    )
                if self.session is not None:
                    self.store.archive_session(self.session)
                mode = payload.get('mode')
                parameters = self._validated_parameters(
                    payload.get('current_parameters'))
                vehicle = {
                    **self.vehicle_settings,
                    'wheelbase_m': parameters['wheelbase'],
                    'servo_min': self.live_servo_limits[0],
                    'servo_max': self.live_servo_limits[1],
                    'steering_stick_scale': self.live_stick_scale,
                }
                try:
                    self.session = new_session(mode, parameters, vehicle)
                except ValueError as exc:
                    raise WizardError(str(exc)) from exc
                return self._save_and_snapshot()

            self._require_session()
            if action == 'set_stage':
                stage = payload.get('stage')
                if stage not in VALID_STAGES:
                    raise WizardError(f'Unknown wizard stage: {stage!r}.')
                if not stage_allowed(self.session.get('mode'), stage):
                    raise WizardError(f'Stage {stage!r} is not part of this session.')
                self.session['stage'] = stage
                touch(self.session, 'stage_changed', stage)
                return self._save_and_snapshot()
            if action == 'update_parameters':
                self.session['current_parameters'] = \
                    self._validated_parameters(payload.get('current_parameters'))
                self.session['vehicle']['wheelbase_m'] = \
                    self.session['current_parameters']['wheelbase']
                self.session['report'] = None
                touch(self.session, 'parameters_updated')
                return self._save_and_snapshot()
            if action == 'start_capture':
                return self._start_capture(payload)
            if action == 'stop_capture':
                return self._stop_capture(automatic=False)
            if action == 'accept_capture':
                return self._accept_capture(payload)
            if action == 'discard_capture':
                if not self.session.get('pending_capture'):
                    raise WizardError('There is no pending capture to discard.')
                self.session['pending_capture'] = None
                touch(self.session, 'capture_discarded')
                return self._save_and_snapshot()
            if action == 'delete_trial':
                trial_id = payload.get('trial_id')
                before = len(self.session.get('trials', []))
                self.session['trials'] = [
                    trial for trial in self.session.get('trials', [])
                    if trial.get('id') != trial_id
                ]
                if len(self.session['trials']) == before:
                    raise WizardError('Accepted trial was not found.', status=404)
                self.session['report'] = None
                touch(self.session, 'trial_deleted', str(trial_id))
                return self._save_and_snapshot()
            if action == 'set_trial_override':
                value = payload.get('value')
                if not isinstance(value, bool):
                    raise WizardError('value must be true or false.')
                trial = next((t for t in self.session.get('trials', [])
                              if t.get('id') == payload.get('trial_id')), None)
                if trial is None:
                    raise WizardError('Accepted trial was not found.', status=404)
                trial['warnings_overridden'] = value
                self.session['report'] = None
                touch(self.session, 'trial_override', f"{trial['id']} {value}")
                return self._save_and_snapshot()
            if action == 'update_vehicle':
                track = payload.get('rear_track_m')
                if isinstance(track, bool) or not _finite(track) \
                        or not REAR_TRACK_RANGE_M[0] <= float(track) <= REAR_TRACK_RANGE_M[1]:
                    raise WizardError(
                        f'Rear track must be {REAR_TRACK_RANGE_M[0]:g}-'
                        f'{REAR_TRACK_RANGE_M[1]:g} m (rear tire centre to centre).')
                self.session.setdefault('vehicle', {})['rear_track_m'] = float(track)
                self.session['report'] = None
                touch(self.session, 'vehicle_updated', f'rear_track_m {float(track)}')
                return self._save_and_snapshot()
            if action == 'generate_report':
                touch(self.session, 'report_generated')
                self.session['report'] = calibration_math.build_report(
                    self.session)
                markdown = calibration_math.report_markdown(
                    self.session['report'])
                json_path, markdown_path = self.store.archive_report(
                    self.session, markdown)
                self.session['report_files'] = {
                    'json': str(json_path),
                    'markdown': str(markdown_path),
                }
                return self._save_and_snapshot()
            if action == 'apply_parameters':
                return self._apply_parameters(payload)
            if action == 'restore_backup':
                return self._restore_backup(payload)
        raise WizardError(f'Unknown action: {action!r}.')

    # Writing vesc.yaml ----------------------------------------------------

    def _apply_guard(self, payload):
        if not self.allow_apply:
            raise WizardError(
                'Applying is switched off on this car (allow_apply: false). '
                'Copy the values into vesc.yaml by hand.', status=403)
        if self.vesc_yaml_path is None:
            raise WizardError(
                'vesc.yaml could not be located; set vesc_yaml_path in '
                'odom_calibration.yaml.', status=409)
        if self.active_recorder or self.session.get('active_capture'):
            raise WizardError('Stop recording first.', status=409)
        if payload.get('confirmed') is not True:
            raise WizardError('Explicit confirmation is required to edit vesc.yaml.')

    def _updates(self, payload):
        try:
            if payload.get('source') == 'report':
                return wizard_actions.report_updates(
                    self.session.get('report'), payload.get('names'))
            if payload.get('source') == 'centre':
                return wizard_actions.centre_updates(
                    self.session, self.live_servo_limits)
            current = calibration_math.current_values(self.session)
            if payload.get('source') == 'manual':
                return wizard_actions.manual_updates(
                    payload.get('values'), current, self.live_servo_limits)
            if payload.get('source') == 'limit_step':
                step = wizard_actions.limit_step_updates(
                    current, self.live_servo_limits, self.live_stick_scale,
                    payload.get('side'), payload.get('step'),
                    payload.get('raise_stick') is True)
                # The same hard checks as a value typed by hand.
                return wizard_actions.manual_updates(
                    step, current, self.live_servo_limits)
        except ValueError as exc:
            raise WizardError(str(exc), status=409) from exc
        raise WizardError("source must be 'report', 'centre', 'manual' or 'limit_step'.")

    def _apply_parameters(self, payload):
        self._apply_guard(payload)
        source = payload.get('source')
        vesc_updates, scale = wizard_actions.split_updates(self._updates(payload))
        if scale is not None and self.joy_teleop_yaml_path is None:
            raise WizardError(
                'joy_teleop.yaml could not be located; set joy_teleop_yaml_path '
                'in odom_calibration.yaml.', status=409)
        label = f"{source}-{str(self.session.get('session_id'))[:8]}"
        backups = self.store.directory / 'backups'
        results = []
        try:
            if vesc_updates:
                results.append(vesc_yaml.apply(
                    self.vesc_yaml_path, vesc_updates, backups, label=label))
            if scale is not None:
                results.append(teleop_yaml.apply(
                    self.joy_teleop_yaml_path, scale, backups, label=label))
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            # One write is all or nothing: put back what already landed.
            for result in results:
                vesc_yaml.restore(result['path'], result['backup'])
            raise WizardError(f'Nothing was changed: {exc}', status=409) from exc
        apply_id = str(uuid.uuid4())
        at = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        records = [{
            **result,
            'source': source,
            'apply_id': apply_id,
            'at': at,
            'restored': False,
            # A path inside install/ is a copy that the next colcon build
            # overwrites; only a path in src/ survives and shows in git.
            'in_source_tree': '/install/' not in result['path'],
        } for result in results]
        self.session.setdefault('applied_changes', []).extend(records)
        for record in records:
            touch(self.session, 'config_applied',
                  json.dumps({'path': record['path'], **record['after']}))
            self.get_logger().warning(
                f"{record['path']} updated: {record['after']} (backup "
                f"{record['backup']}). Restart "
                + ('teleop' if record['path'].endswith('joy_teleop.yaml') else 'bringup')
                + ' for the car to use the new values.')
        return {**self._save_and_snapshot(), 'applied': records}

    def _restore_backup(self, payload):
        self._apply_guard(payload)
        try:
            to_restore, to_close = wizard_actions.restore_plan(
                self.session.get('applied_changes', []),
                everything=payload.get('everything') is True)
        except ValueError as exc:
            raise WizardError(str(exc)) from exc
        results = []
        for change in to_restore:
            try:
                results.append(vesc_yaml.restore(change['path'], change['backup']))
            except (OSError, ValueError, yaml.YAMLError) as exc:
                raise WizardError(f"{change['path']} was not restored: {exc}",
                                  status=409) from exc
            touch(self.session, 'config_restored', change['backup'])
        for change in to_close:
            change['restored'] = True
        return {**self._save_and_snapshot(), 'restored': results}

    def _start_capture(self, payload):
        if self.active_recorder or self.session.get('active_capture'):
            raise WizardError('A capture is already running.', status=409)
        if self.session.get('pending_capture'):
            raise WizardError(
                'Accept or discard the pending capture first.', status=409)
        kind = payload.get('kind')
        if kind not in CAPTURE_KINDS:
            raise WizardError(f'Unknown capture kind: {kind!r}.')
        if not capture_allowed(self.session.get('mode'), kind):
            raise WizardError('That test is not part of this session.')
        health = self._health_snapshot(time.monotonic())
        if kind != 'stationary' and health['odom']['status'] not in (
                'good', 'warning'):
            raise WizardError(
                'Fresh odometry is required before movement capture.',
                status=409,
                details={'health': health},
            )
        self.active_recorder = CaptureRecorder(
            kind, self.max_samples_per_topic)
        # What the car was running for this run. Later vesc.yaml changes must
        # not reinterpret it (the odometry fallback and the centre-offset
        # step both depend on the values that produced the recording).
        self.active_recorder.parameters = copy.deepcopy(
            self.session.get('current_parameters', {}))
        self.active_recorder.servo_limits = list(self.live_servo_limits)
        self.session['active_capture'] = {
            'kind': kind,
            'started_at': self.active_recorder.started_at,
            'started_monotonic': self.active_recorder.started_monotonic,
        }
        touch(self.session, 'capture_started', kind)
        return self._save_and_snapshot()

    def _stop_capture(self, automatic=False):
        if not self.active_recorder:
            raise WizardError('There is no capture running.', status=409)
        recorder = self.active_recorder
        capture = recorder.finish()
        capture['parameters'] = getattr(recorder, 'parameters', None)
        capture['servo_limits'] = getattr(recorder, 'servo_limits', None)
        self.active_recorder = None
        summary = calibration_math.summarize_capture(capture)
        if automatic:
            summary['warnings'].append(
                f'Capture automatically stopped at {self.max_capture_sec:.0f}s.'
            )
        self.session['active_capture'] = None
        self.session['pending_capture'] = {
            'id': str(uuid.uuid4()),
            'kind': capture['kind'],
            'summary': summary,
            'parameters_at_capture': capture.get('parameters'),
            'servo_limits_at_capture': capture.get('servo_limits'),
        }
        touch(self.session, 'capture_stopped', capture['kind'])
        return self._save_and_snapshot()

    def _accept_capture(self, payload):
        pending = self.session.get('pending_capture')
        if pending is None:
            raise WizardError('There is no pending capture to accept.')
        if payload.get('confirmed') is not True:
            raise WizardError(
                'Explicit confirmation is required before accepting a trial.')
        trial = copy.deepcopy(pending)
        kind = trial['kind']
        if kind == 'movement':
            distance = payload.get('measured_distance_m')
            direction = payload.get('direction')
            if not _finite(distance) or float(distance) <= 0.0:
                raise WizardError(
                    'Tape-measured distance must be a positive number.')
            if direction not in ('forward', 'reverse'):
                raise WizardError('Confirm forward or reverse direction.')
            trial['measured_distance_m'] = float(distance)
            trial['direction'] = direction
        elif kind in calibration_math.STEERING_KINDS:
            try:
                fields = calibration_math.validate_steering_measurement(kind, payload)
            except ValueError as exc:
                raise WizardError(str(exc)) from exc
            trial.update(fields)
        override = payload.get('use_despite_warnings', False)
        if not isinstance(override, bool):
            raise WizardError('use_despite_warnings must be true or false.')
        trial['warnings_overridden'] = override
        trial['notes'] = str(payload.get('notes', ''))[:2000]
        trial['accepted'] = True
        trial['confirmed_at'] = time.strftime(
            '%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        self.session.setdefault('trials', []).append(trial)
        self.session['pending_capture'] = None
        self.session['report'] = None
        touch(self.session, 'capture_accepted', kind)
        return self._save_and_snapshot()

    # Snapshots and broadcasting -------------------------------------------

    def _health_snapshot(self, now):
        return {
            name: monitor.snapshot(now, self.topic_stale_sec)
            for name, monitor in self.monitors.items()
        }

    def snapshot(self):
        with self._lock:
            now = time.monotonic()
            active_duration = None
            if self.active_recorder:
                active_duration = now - self.active_recorder.started_monotonic
                progress = self.active_recorder.progress
                capture_progress = {
                    'kind': self.active_recorder.kind,
                    'odom_distance_m': progress['odom_distance_m'].total,
                    'odom_yaw_rad': progress['odom_yaw_rad'].total,
                }
            else:
                capture_progress = None
            steering_view = {}
            if self.session is not None and self.session.get('mode') in \
                    calibration_math.MODES_WITH_STEERING:
                vehicle = self.session.get('vehicle', {}) or {}
                try:
                    full = calibration_math.steering_calibration(self.session)
                    preview = {key: full[key] for key in (
                        'curves', 'limit_suggestions', 'sides', 'status',
                        'suggested_steering_angle_to_servo_gain',
                        'suggested_steering_angle_to_servo_offset')}
                    preview['trials'] = {
                        item['trial_id']: {
                            'usable': item['usable'], 'warnings': item['warnings'],
                            'warnings_overridden': item['warnings_overridden']}
                        for item in full['trial_results']}
                except Exception as exc:  # a preview must never break the page
                    preview = {'error': str(exc)}
                steering_view = {
                    'steering_preview': preview,
                    'clamp_check': steering_chain.clamp_check(
                        self.session.get('current_parameters', {}),
                        [vehicle.get('servo_min'), vehicle.get('servo_max')],
                        vehicle.get('steering_stick_scale'),
                        vehicle.get('planner_max_steering_rad')),
                    'centre_offset': steering_chain.centre_offset_suggestion(
                        self.session, calibration_math.drift_steering_angle),
                }
            return {
                'session': copy.deepcopy(self.session),
                'telemetry': copy.deepcopy(self.latest),
                'health': self._health_snapshot(now),
                'live_parameters': copy.deepcopy(self.live_parameters),
                'live_servo_limits': list(self.live_servo_limits),
                'live_stick_scale': self.live_stick_scale,
                'live_parameter_status': self.live_parameter_status,
                'live_parameter_sources': dict(self.live_parameter_sources),
                'pending_restart': self._pending_restart(),
                'apply': {
                    'allowed': self.allow_apply and self.vesc_yaml_path is not None,
                    'path': str(self.vesc_yaml_path) if self.vesc_yaml_path else None,
                    'teleop_path': (str(self.joy_teleop_yaml_path)
                                    if self.joy_teleop_yaml_path else None),
                },
                **steering_view,
                'capture_duration_sec': active_duration,
                'capture_progress': capture_progress,
                'read_only': True,
                'report_directory': str(self.store.directory),
                'server_time': time.time(),
            }

    def _pending_restart(self):
        """Applied values the running car has not picked up yet."""
        if self.session is None:
            return []
        live = {**self.live_parameters,
                'servo_min': self.live_servo_limits[0],
                'servo_max': self.live_servo_limits[1],
                teleop_yaml.NAME: self.live_stick_scale}
        return wizard_actions.pending_restart(self.session, live)

    def _telemetry_tick(self):
        should_stop = False
        with self._lock:
            if self.active_recorder:
                duration = time.monotonic() \
                    - self.active_recorder.started_monotonic
                should_stop = duration >= self.max_capture_sec
        if should_stop:
            with self._lock:
                try:
                    self._stop_capture(automatic=True)
                except WizardError:
                    pass
        self._schedule_broadcast()

    def _schedule_broadcast(self):
        if self._loop is None:
            return
        self._loop.add_callback(self._broadcast_snapshot)

    def _broadcast_snapshot(self):
        if not self.ws_clients:
            return
        message = json.dumps(
            {'type': 'snapshot', **self.snapshot()},
            allow_nan=False,
        )
        stale = []
        for client in list(self.ws_clients):
            try:
                client.write_message(message)
            except tornado.websocket.WebSocketClosedError:
                stale.append(client)
        for client in stale:
            self.ws_clients.discard(client)

    def report_download(self, report_format):
        with self._lock:
            self._require_session()
            report = self.session.get('report')
            if not report:
                raise WizardError('Generate the report first.', status=404)
            session_id = self.session.get('session_id', 'unknown')
            if report_format == 'json':
                return (
                    json.dumps(report, indent=2, sort_keys=True) + '\n',
                    'application/json',
                    f'calibration-{session_id}.json',
                )
            if report_format in ('md', 'markdown'):
                return (
                    calibration_math.report_markdown(report),
                    'text/markdown; charset=utf-8',
                    f'calibration-{session_id}.md',
                )
        raise WizardError('Report format must be json or md.', status=404)


def _spin_node(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        # rclpy turns Ctrl+C and SIGTERM into a ROS shutdown, which ends this
        # spin but not the web loop in the main thread. Without this the page
        # stayed up, its port taken, with no ROS behind it (found 2026-10-11).
        loop = node._loop
        if loop is not None:
            loop.add_callback(loop.stop)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCalibrationNode()
    spin_thread = threading.Thread(
        target=_spin_node,
        args=(node,),
        daemon=True,
        name='odom-calibration-rclpy',
    )
    spin_thread.start()

    web_directory = os.path.join(
        get_package_share_directory('odom_calibration'), 'web')
    # With `colcon build --symlink-install` every installed web file is a
    # symlink into build/ (and on into src/). Tornado 6.5 refuses to serve a
    # symlink that leaves the served folder (403), so serve the folder the
    # files really live in. A plain install resolves to itself.
    web_directory = os.path.dirname(
        os.path.realpath(os.path.join(web_directory, 'index.html')))
    application = tornado.web.Application([
        (r'/', tornado.web.RedirectHandler, {'url': '/index.html'}),
        (r'/api/state', StateHandler, {'node': node}),
        (r'/api/action', ActionHandler, {'node': node}),
        (r'/api/report/(json|md|markdown)', DownloadHandler, {'node': node}),
        (r'/ws', WizardWebSocket, {'node': node}),
        (
            r'/(.*)',
            NoCacheStaticFileHandler,
            {'path': web_directory, 'default_filename': 'index.html'},
        ),
    ])
    try:
        application.listen(node.port, address=node.host)
    except OSError as exc:
        node.get_logger().fatal(
            f'Cannot open the wizard web page on port {node.port}: {exc}. '
            'Another calibration wizard is probably still running; stop it '
            '(Ctrl+C in its terminal, or `pkill -f calibration_node`) and '
            'try again, or set a different port in odom_calibration.yaml.')
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        raise SystemExit(1) from exc
    loop = tornado.ioloop.IOLoop.current()
    node._loop = loop
    node.get_logger().info(
        f'Calibration Web UI listening on http://{node.host}:{node.port}/')
    try:
        loop.start()
    except KeyboardInterrupt:
        pass
    finally:
        node._loop = None
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
