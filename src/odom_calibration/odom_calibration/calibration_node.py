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

from odom_calibration import calibration_math
from odom_calibration.session_store import (
    SessionStore,
    VALID_STAGES,
    new_session,
    touch,
)


PARAMETER_NAMES = (
    'speed_to_erpm_gain',
    'speed_to_erpm_offset',
    'steering_angle_to_servo_gain',
    'steering_angle_to_servo_offset',
    'wheelbase',
)
CAPTURE_KINDS = (
    'stationary',
    'movement',
    'steering_center',
    'steering_left',
    'steering_right',
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
        self.started_monotonic = time.monotonic()
        self.started_at = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        self.samples = {
            'odom': [],
            'vesc': [],
            'servo': [],
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


class OdomCalibrationNode(Node):
    """Collect telemetry, own wizard state, and never publish to ROS."""

    def __init__(self):
        super().__init__('odom_calibration_node')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('vesc_state_topic', '/sensors/core')
        self.declare_parameter(
            'servo_topic', '/sensors/servo_position_command')
        self.declare_parameter('drive_topic', '/ackermann_cmd')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8090)
        self.declare_parameter('telemetry_rate_hz', 5.0)
        self.declare_parameter('topic_stale_sec', 0.75)
        self.declare_parameter('max_capture_sec', 300.0)
        self.declare_parameter('max_samples_per_topic', 30000)
        self.declare_parameter(
            'report_directory', '~/.ros/odom_calibration')
        self.declare_parameter('speed_to_erpm_gain', 4614.0)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter(
            'steering_angle_to_servo_gain', -1.2135)
        self.declare_parameter(
            'steering_angle_to_servo_offset', 0.5304)
        self.declare_parameter('wheelbase', 0.324)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.vesc_state_topic = self.get_parameter('vesc_state_topic').value
        self.servo_topic = self.get_parameter('servo_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.joy_topic = self.get_parameter('joy_topic').value
        self.deadman_button = int(self.get_parameter('deadman_button').value)
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
        if self.default_parameters['speed_to_erpm_gain'] <= 0.0:
            raise ValueError('speed_to_erpm_gain must be positive')
        if self.default_parameters['wheelbase'] <= 0.0:
            raise ValueError('wheelbase must be positive')

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
            'command_speed': None,
            'command_steering': None,
            'lb_held': False,
            'joy_button_available': False,
        }
        self.live_parameters = dict(self.default_parameters)
        self.live_parameter_status = 'configured fallback'
        self.monitors = {
            'odom': TopicMonitor('Odometry'),
            'vesc': TopicMonitor('Raw VESC'),
            'servo': TopicMonitor('Servo command'),
            'drive': TopicMonitor('Selected drive command'),
            'joy': TopicMonitor('Remote / LB'),
        }
        self.ws_clients = set()
        self._loop = None
        self._parameter_request_inflight = False

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
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, self.drive_topic, self.drive_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, 10)

        self.parameter_client = AsyncParameterClient(
            self, 'vesc_to_odom_node')
        self.parameter_timer = self.create_timer(
            2.0, self._request_live_parameters)
        self.telemetry_timer = self.create_timer(
            1.0 / self.telemetry_rate_hz, self._telemetry_tick)

        self.get_logger().info(
            'Odometry calibration wizard is read-only: it creates no ROS '
            'publishers and cannot command the car. '
            f'Open http://<this-car-IP>:{self.port}/ after the web server starts.'
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
        with self._lock:
            self.monitors['joy'].update(
                now, _header_seconds(msg), valid=available)
            self.latest['lb_held'] = held
            self.latest['joy_button_available'] = available
            if self.active_recorder:
                self.active_recorder.add('joy', {
                    't': now,
                    'lb_held': held,
                })

    # Live parameter discovery ---------------------------------------------

    def _request_live_parameters(self):
        if self._parameter_request_inflight:
            return
        if not self.parameter_client.services_are_ready():
            return
        self._parameter_request_inflight = True
        future = self.parameter_client.get_parameters(list(PARAMETER_NAMES))
        future.add_done_callback(self._live_parameters_done)

    def _live_parameters_done(self, future):
        self._parameter_request_inflight = False
        try:
            parameters = future.result()
            values = {
                name: float(parameter.value)
                for name, parameter in zip(PARAMETER_NAMES, parameters)
            }
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError('live parameter response contains non-finite values')
        except Exception as exc:
            self.live_parameter_status = f'query failed: {exc}'
            return
        with self._lock:
            self.live_parameters = values
            self.live_parameter_status = 'read from vesc_to_odom_node'
        self.parameter_timer.cancel()

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
        if merged['speed_to_erpm_gain'] <= 0.0:
            raise WizardError('speed_to_erpm_gain must be positive.')
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
                    'model': 'Traxxas Ford Fiesta ST Rally VXL 74276-4',
                    'wheelbase_m': parameters['wheelbase'],
                    'physical_width_m': 0.281,
                    'physical_length_m': 0.535,
                }
                self.session = new_session(mode, parameters, vehicle)
                return self._save_and_snapshot()

            self._require_session()
            if action == 'set_stage':
                stage = payload.get('stage')
                if stage not in VALID_STAGES:
                    raise WizardError(f'Unknown wizard stage: {stage!r}.')
                if (
                    stage == 'steering'
                    and self.session.get('mode') != 'movement_steering'
                ):
                    raise WizardError(
                        'This session was created as movement-only.')
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
        raise WizardError(f'Unknown action: {action!r}.')

    def _start_capture(self, payload):
        if self.active_recorder or self.session.get('active_capture'):
            raise WizardError('A capture is already running.', status=409)
        if self.session.get('pending_capture'):
            raise WizardError(
                'Accept or discard the pending capture first.', status=409)
        kind = payload.get('kind')
        if kind not in CAPTURE_KINDS:
            raise WizardError(f'Unknown capture kind: {kind!r}.')
        if (
            kind.startswith('steering_')
            and self.session.get('mode') != 'movement_steering'
        ):
            raise WizardError('Steering capture is disabled for this session.')
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
        capture = self.active_recorder.finish()
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
        elif kind in ('steering_left', 'steering_right'):
            diameter = payload.get('measured_diameter_m')
            if not _finite(diameter) or float(diameter) <= 0.0:
                raise WizardError(
                    'Tape-measured rear-axle circle diameter must be positive.')
            trial['measured_diameter_m'] = float(diameter)
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
            return {
                'session': copy.deepcopy(self.session),
                'telemetry': copy.deepcopy(self.latest),
                'health': self._health_snapshot(now),
                'live_parameters': copy.deepcopy(self.live_parameters),
                'live_parameter_status': self.live_parameter_status,
                'capture_duration_sec': active_duration,
                'read_only': True,
                'report_directory': str(self.store.directory),
                'server_time': time.time(),
            }

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
    application = tornado.web.Application([
        (r'/', tornado.web.RedirectHandler, {'url': '/index.html'}),
        (r'/api/state', StateHandler, {'node': node}),
        (r'/api/action', ActionHandler, {'node': node}),
        (r'/api/report/(json|md|markdown)', DownloadHandler, {'node': node}),
        (r'/ws', WizardWebSocket, {'node': node}),
        (
            r'/(.*)',
            tornado.web.StaticFileHandler,
            {'path': web_directory, 'default_filename': 'index.html'},
        ),
    ])
    application.listen(node.port, address=node.host)
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
