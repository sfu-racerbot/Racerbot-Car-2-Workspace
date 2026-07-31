"""
dashboard_node.py

Serves a live browser dashboard of what the car can see and where it is:
the SLAM/localization map, the raw LIDAR scan, and the car's own pose, all
drawn on an HTML5 canvas and pushed to any connected browser in real time
over a WebSocket. See docs/web-dashboard.md for the full picture,
including how the browser side lines everything up.

Telemetry-wise this node is entirely passive: it only ever subscribes, and
it publishes to no ROS *topic* at all. It cannot steer, accelerate, or
brake the car, and it can be left running at all times alongside anything
else in this workspace, including during an actual race.

It is not, however, purely read-only any more. With `enable_tuning` (the
default), it also acts as a ROS *service client* for the
`/<node>/set_parameters` service of the driving nodes listed in
`tuning_nodes`, so a phone at trackside can adjust max speed, lookahead,
and the reactive-safety margins between runs instead of editing YAML and
relaunching. That is a genuine control path, and it is bounded on four
sides, deliberately:

  1. It changes *tuning*, never motion. There is still no publisher to
     /drive here. The car does not move because of anything this node
     does; it moves because an autonomy node decided to, and only while
     the driver holds LB (docs/architecture.md's mandatory deadman
     policy, which is not tunable from here or anywhere else at runtime).
  2. Only parameters a node itself advertises as live-tunable can change,
     and only within bounds that node enforces in its own process. See
     pure_pursuit/live_tuning.py.
  3. Only the nodes named in `tuning_nodes` are ever probed -- which is
     also how this stays scoped to this workspace's own driving code and
     never reaches a teammate's package.
  4. A browser must explicitly arm tuning, per connection, before any
     write is accepted; that arm state is held server-side here, not just
     greyed-out in CSS, and it is dropped when the socket closes.

Set `enable_tuning: false` to remove the service clients entirely and get
the strictly-read-only node back. See docs/web-dashboard.md's security
note before exposing this to a network you don't control.

Two concurrency models have to coexist in one process here:
  - rclpy's own executor, which calls this node's subscription callbacks.
  - Tornado's IOLoop (asyncio-based), which runs the web server and every
    WebSocket connection.
They don't share a thread by default, so rclpy is spun on a background
thread while Tornado's IOLoop owns the main thread (see main() below).
Tornado handlers are documented as *not* safe to touch directly from any
thread but the IOLoop's own -- so every subscription callback below hands
its update to the IOLoop via `add_callback()` (documented by Tornado as
thread-safe, specifically for this purpose) instead of ever calling
`write_message()` itself.
"""

import functools
import glob
import json
import os
import queue
import tempfile
import threading
import time

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import Joy, LaserScan

import tornado.ioloop
import tornado.web
import tornado.websocket
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from web_dashboard import protocol, tuning
from web_dashboard.stopwatch import DeadmanStopwatch


def _read_cpu_temp_c():
    """Best-effort CPU temperature from the kernel's thermal framework.
    Returns None rather than raising if this machine has no readable
    'cpu-thermal' zone (e.g. developing on a laptop, not the Jetson) --
    system stats should degrade gracefully, not crash the node."""
    for type_path in glob.glob('/sys/class/thermal/thermal_zone*/type'):
        try:
            with open(type_path) as f:
                if f.read().strip() != 'cpu-thermal':
                    continue
            with open(type_path.replace('/type', '/temp')) as f:
                return int(f.read().strip()) / 1000.0
        except OSError:
            continue
    return None


def _read_wifi_signal_dbm():
    """Best-effort WiFi signal level in dBm, from the kernel's own
    /proc/net/wireless (the same source `iwconfig`/`nmcli` read) --
    returns None on anything wired-only (no wireless interface at all, e.g.
    developing off a laptop docked to Ethernet) rather than raising, same
    as _read_cpu_temp_c. Picks the first interface listed rather than
    hardcoding one by name, since that's stable across which physical NIC
    a given machine happens to use for WiFi."""
    try:
        with open('/proc/net/wireless') as f:
            lines = f.readlines()
    except OSError:
        return None
    for line in lines[2:]:  # first two lines are a fixed two-line header
        if ':' not in line:
            continue
        fields = line.split(':', 1)[1].split()
        if len(fields) < 3:
            continue
        try:
            return float(fields[2])  # signal level, dBm (fields[1] is link quality)
        except ValueError:
            continue
    return None


def _encode_parameter_value(value):
    """Python value -> rcl_interfaces ParameterValue.

    Only the two types the tuning catalogue can describe. Integers are
    sent as doubles on purpose: every numeric tunable in this workspace is
    declared a double, and an integer-typed set against one is refused by
    the parameter server as a type mismatch -- which would make a slider
    that happens to land exactly on 2 fail while 2.1 worked.
    """
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    return ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))


def _decode_parameter_value(value):
    """rcl_interfaces ParameterValue -> Python, or None if it's a type
    this dashboard has no control for."""
    if value.type == ParameterType.PARAMETER_BOOL:
        return bool(value.bool_value)
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if value.type == ParameterType.PARAMETER_INTEGER:
        return float(value.integer_value)
    return None


def _atomic_write(path, text):
    """Replace a file's contents without ever leaving a partial one behind.

    A half-written parameter YAML is a node that refuses to launch, and
    the moment this is most likely to be interrupted (Ctrl+C at the end of
    a session) is exactly when nobody would notice until the next run.
    Write a sibling temp file, fsync, then rename -- rename is atomic
    within a filesystem, so the config is either the old one or the new
    one at every instant.
    """
    directory = os.path.dirname(path) or '.'
    handle = tempfile.NamedTemporaryFile(
        'w', dir=directory, prefix='.tuning-', suffix='.yaml', delete=False)
    try:
        with handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(handle.name, path)
    except BaseException:
        try:
            os.unlink(handle.name)
        except OSError:
            pass
        raise


class _TuningTarget:
    """One driving node this dashboard is willing to tune.

    Owned entirely by the rclpy spin thread -- see the threading contract
    above DashboardNode._setup_tuning().
    """

    def __init__(self, node_name, config_path, config_error):
        self.node_name = node_name
        self.config_path = config_path
        self.config_error = config_error
        self.get_client = None
        self.set_client = None
        self.online = False
        self.spec = None        # parsed tunable catalogue, once read
        self.spec_error = None
        self.values = {}        # name -> what the node currently holds
        self.baseline = {}      # name -> value at first sight, for "revert"
        self.inflight = None    # deadline of an outstanding service call

    def reset(self):
        """Forget everything learned about a node that has gone away."""
        self.online = False
        self.spec = None
        self.spec_error = None
        self.values = {}
        self.baseline = {}
        self.inflight = None


class DashboardWebSocket(tornado.websocket.WebSocketHandler):
    """One instance per connected browser tab. Pure bookkeeping -- all the
    actual data comes from DashboardNode via _broadcast()/send_initial_state()."""

    def initialize(self, node: 'DashboardNode'):
        self.node = node
        # Live tuning is armed per connection and starts disarmed on every
        # single page load -- see arm_tuning() below.
        self.tuning_armed = False

    def check_origin(self, origin):
        # This dashboard has no login, and accepts any origin. For the
        # telemetry half that is a reasonable trade-off for a LAN-only
        # debugging tool: a malicious page could see map/scan/pose but
        # could never command the car.
        #
        # Live tuning does reach the car, so it is not covered by that
        # reasoning and does not rely on it: it is gated on an explicit
        # per-connection arm (below), bounded by each driving node's own
        # in-process clamps, and cannot move a car whose driver is not
        # holding LB. What it is *not* is protected against someone who
        # can already reach this port and means harm -- so this still
        # belongs on a trusted network or Tailscale, never a public one.
        # Set enable_tuning:false to close the write path entirely. See
        # docs/web-dashboard.md's security note.
        return True

    def open(self):
        self.node.ws_clients.add(self)
        self.node.send_initial_state(self)

    def on_close(self):
        self.node.ws_clients.discard(self)

    def arm_tuning(self, armed: bool):
        """Arm/disarm writes for *this* browser connection.

        Held here, on the connection, rather than on the node: arming is a
        statement about the person holding this particular device, and it
        should not outlive their tab. A reload, a dropped WiFi link, or a
        phone going to sleep all close the socket and take the arm with
        it, which is the behaviour you want from something that lets a
        pocket-tap change a moving car's speed limit.
        """
        self.tuning_armed = bool(armed) and self.node.enable_tuning
        self.write_message(json.dumps(
            protocol.tuning_armed_message(self.tuning_armed)))

    def on_message(self, message):
        """Browser -> server. Two kinds of input are accepted: the
        dashboard's own stopwatch (affects nothing outside this process),
        and live tuning (reaches the driving nodes -- see the class
        docstring and check_origin above)."""
        if not isinstance(message, str):
            return
        try:
            payload = json.loads(message)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return

        kind = payload.get('type')
        if kind == 'stopwatch_control':
            self.node.handle_stopwatch_control(
                payload.get('action'), payload.get('enabled'))
            return
        if kind != 'tuning_control':
            return

        action = payload.get('action')
        if action == 'arm':
            self.arm_tuning(payload.get('armed'))
            return
        if not self.node.enable_tuning:
            self.write_message(json.dumps(protocol.tuning_saved_message(
                False, 'live tuning is disabled on this dashboard '
                       '(enable_tuning is false)')))
            return
        if not self.tuning_armed:
            # Refused server-side, not merely disabled in the UI: a stale
            # tab, a replayed message, or a hand-rolled WebSocket client
            # all land here too.
            self.write_message(json.dumps(protocol.tuning_result_message(
                str(payload.get('node', '')), str(payload.get('name', '')),
                False, reason='tuning is not armed on this connection')))
            return

        if action == 'set':
            self.node.request_tuning_set(
                payload.get('node'), payload.get('name'), payload.get('value'))
        elif action == 'save':
            self.node.request_tuning_save()
        elif action == 'refresh':
            self.node.broadcast_tuning_state()


class DashboardNode(Node):
    """Fan read-only ROS telemetry out to every connected browser tab."""

    def __init__(self):
        super().__init__('web_dashboard_node')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        # A list, not a single topic, because which node localizes the car
        # depends on which stack is up and the dashboard is deliberately
        # started once and left running across all of them:
        #   /pf/viz/inferred_pose  particle_filter (race_launch, localization)
        #   /slam_pose             auto_map_race_node, while slam_toolbox maps
        # Subscribing to every candidate means the car shows up on the map
        # without having to relaunch the dashboard per mode. Only one of
        # them publishes at a time in practice; if two ever did, last
        # message wins, which is the right answer for a display anyway.
        self.declare_parameter(
            'pose_topics', ['/pf/viz/inferred_pose', '/slam_pose'])
        self.declare_parameter('drive_topic', '/ackermann_cmd')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('joy_timeout_sec', 0.5)
        self.declare_parameter('stopwatch_update_rate_hz', 10.0)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('scan_broadcast_rate_hz', 10.0)
        self.declare_parameter('stats_interval_sec', 1.0)
        self.declare_parameter('laser_offset_x', 0.33)
        self.declare_parameter('laser_offset_y', 0.0)

        # --- Live parameter tuning (see the module docstring) ---
        self.declare_parameter('enable_tuning', True)
        # The *only* nodes this dashboard will ever probe or write to.
        # Deliberately an explicit list rather than "every node on the
        # graph that looks tunable": scanning the bus would mean calling
        # services on a teammate's nodes and on vendored drivers, none of
        # which asked to be tuned from a browser. A node still has to
        # advertise a live_tunable_spec to appear in the panel, so this
        # list is a bound on the blast radius, not a wish.
        self.declare_parameter('tuning_nodes', ['pure_pursuit_node', 'gap_follow_node'])
        # Parallel to tuning_nodes: '<package>/<path under its share dir>'
        # for the config each node's "save" button writes back to. An
        # empty entry means that node's tune can be changed live but never
        # saved.
        self.declare_parameter('tuning_config_files', [
            'pure_pursuit/config/pure_pursuit.yaml',
            'gap_follow/config/gap_follow.yaml',
        ])
        self.declare_parameter('tuning_allow_save', True)
        self.declare_parameter('tuning_refresh_sec', 2.0)
        self.declare_parameter('tuning_request_rate_hz', 20.0)
        self.declare_parameter('tuning_service_timeout_sec', 3.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        # dict.fromkeys() rather than set(): de-duplicate without shuffling
        # the configured order, so the log below reads as written.
        self.pose_topics = list(dict.fromkeys(
            str(topic) for topic in self.get_parameter('pose_topics').value if str(topic)))
        self.drive_topic = self.get_parameter('drive_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.joy_topic = self.get_parameter('joy_topic').value
        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.joy_timeout_sec = float(self.get_parameter('joy_timeout_sec').value)
        self.stopwatch_update_rate_hz = float(
            self.get_parameter('stopwatch_update_rate_hz').value)
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.scan_broadcast_rate_hz = float(self.get_parameter('scan_broadcast_rate_hz').value)
        self.stats_interval_sec = float(self.get_parameter('stats_interval_sec').value)
        self.laser_offset_x = float(self.get_parameter('laser_offset_x').value)
        self.laser_offset_y = float(self.get_parameter('laser_offset_y').value)
        self.enable_tuning = bool(self.get_parameter('enable_tuning').value)
        self.tuning_allow_save = bool(self.get_parameter('tuning_allow_save').value)
        self.tuning_refresh_sec = max(
            0.2, float(self.get_parameter('tuning_refresh_sec').value))
        self.tuning_request_rate_hz = max(
            1.0, float(self.get_parameter('tuning_request_rate_hz').value))
        self.tuning_service_timeout_sec = max(
            0.5, float(self.get_parameter('tuning_service_timeout_sec').value))

        # Latest-known state, re-sent in full to any newly connected
        # browser tab (send_initial_state) so it isn't stuck waiting for
        # the next update to see anything. Written on the rclpy spin
        # thread, read on the IOLoop thread -- a benign, self-correcting
        # race (worst case a just-connected tab's first frame is a few ms
        # stale, fixed by the very next broadcast); this is a read-only
        # display, not a control path, so it isn't worth a lock.
        self._last_map_msg = None
        self._last_scan_msg = None
        self._last_pose = None  # (x, y, yaw)
        self._last_pose_topic = None  # which pose_topics entry last delivered
        self._last_drive = None  # (speed, steering_angle)
        self._last_speed = None
        self._last_stats = None  # (cpu_percent, mem_percent, cpu_temp_c, uptime_s)
        self._last_scan_broadcast_time = 0.0
        self._stopwatch_lock = threading.Lock()
        self._stopwatch = DeadmanStopwatch(self.joy_timeout_sec)

        # NOTE: named ws_clients, not clients -- rclpy.node.Node already
        # defines a read-only `clients` property (service clients created
        # via create_client()), and shadowing it raises an AttributeError
        # on assignment.
        self.ws_clients = set()
        self._loop = None  # set once Tornado's IOLoop actually starts, see main()

        # /map durability: nav2's map_server and slam_toolbox both publish
        # /map "transient local" (latched), so a subscriber that starts
        # after the map was published still receives it. Subscribing with
        # default (volatile) durability would silently miss any map
        # published before this node started.
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        # Sensor-data QoS (best-effort) is the broadly-compatible choice for
        # a LIDAR feed: a best-effort subscriber can match either a
        # best-effort *or* reliable publisher, whereas a reliable
        # subscriber can only match a reliable publisher.
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.pose_subs = [
            self.create_subscription(
                PoseStamped, topic,
                functools.partial(self.pose_callback, topic=topic), 10)
            for topic in self.pose_topics
        ]
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, self.drive_topic, self.drive_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, 10)
        self.stopwatch_timer = self.create_timer(
            1.0 / max(self.stopwatch_update_rate_hz, 0.1), self.stopwatch_callback)

        # Priming call: psutil.cpu_percent(interval=None) reports usage
        # *since the previous call*, so the very first call has nothing to
        # compare against and would otherwise report a meaningless 0.0 (or
        # a misleadingly huge number averaged since boot) the moment the
        # stats timer's first tick actually gets broadcast.
        psutil.cpu_percent(interval=None)
        self.stats_timer = self.create_timer(self.stats_interval_sec, self.stats_callback)

        self._setup_tuning()

        self.get_logger().info(
            f"web_dashboard_node ready: map={self.map_topic} scan={self.scan_topic} "
            f"pose={'|'.join(self.pose_topics) or '(none)'} "
            f"drive={self.drive_topic} odom={self.odom_topic} "
            f"joy={self.joy_topic} (LB index {self.deadman_button}). "
            f"live tuning {self._tuning_summary()}. "
            f"Once the web server starts, open http://<this car's IP>:{self.port}/ in a browser."
        )

    # ------------------------------------------------------------------------
    # ROS subscription callbacks -- run on the rclpy spin thread. Each one
    # hands off to the Tornado IOLoop rather than touching sockets directly.
    # ------------------------------------------------------------------------

    def map_callback(self, msg: OccupancyGrid):
        self._last_map_msg = msg
        self._broadcast(protocol.map_header(msg), protocol.map_cells(msg))

    def scan_callback(self, msg: LaserScan):
        self._last_scan_msg = msg
        # LIDAR arrives at ~40Hz; no browser needs to redraw that often,
        # and it's needless load on the WiFi link and the Jetson both --
        # throttle broadcasts to scan_broadcast_rate_hz regardless of how
        # fast /scan itself is actually publishing.
        now = time.monotonic()
        min_period = 1.0 / max(self.scan_broadcast_rate_hz, 0.1)
        if now - self._last_scan_broadcast_time < min_period:
            return
        self._last_scan_broadcast_time = now
        self._broadcast(
            protocol.scan_header(msg, self.laser_offset_x, self.laser_offset_y),
            protocol.scan_ranges(msg))

    def pose_callback(self, msg: PoseStamped, topic: str = ''):
        q = msg.pose.orientation
        yaw = protocol.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self._last_pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        if topic != self._last_pose_topic:
            self._last_pose_topic = topic
            self.get_logger().info(f"Pose display is now following '{topic}'.")
        self._broadcast(protocol.pose_message(*self._last_pose))

    def drive_callback(self, msg: AckermannDriveStamped):
        self._last_drive = (msg.drive.speed, msg.drive.steering_angle)
        self._broadcast(protocol.drive_message(*self._last_drive))

    def odom_callback(self, msg: Odometry):
        self._last_speed = msg.twist.twist.linear.x
        self._broadcast(protocol.speed_message(self._last_speed))

    def joy_callback(self, msg: Joy):
        with self._stopwatch_lock:
            self._stopwatch.update_joy(
                msg.buttons, self.deadman_button, time.monotonic())

    def _stopwatch_message(self):
        with self._stopwatch_lock:
            snapshot = self._stopwatch.snapshot(time.monotonic())
        return protocol.stopwatch_message(**snapshot)

    def _broadcast_stopwatch(self):
        self._broadcast(self._stopwatch_message())

    def stopwatch_callback(self):
        self._broadcast_stopwatch()

    def handle_stopwatch_control(self, action, enabled=None):
        """Handle dashboard-local controls on Tornado's IOLoop thread."""
        now = time.monotonic()
        with self._stopwatch_lock:
            if action == 'set_enabled':
                self._stopwatch.set_enabled(bool(enabled), now)
            elif action == 'reset':
                self._stopwatch.reset(now)
            else:
                return
        self._broadcast_stopwatch()

    def stats_callback(self):
        """Runs on a timer (rclpy spin thread, not per-message) -- cheap
        enough to sample every tick but adds no value redrawing faster
        than a human reads it. Uptime is the machine's, via psutil's own
        boot_time(), not this node's -- restarting the dashboard shouldn't
        reset what's meant to be a "how long has the car been on" readout."""
        self._last_stats = (
            psutil.cpu_percent(interval=None),
            psutil.virtual_memory().percent,
            _read_cpu_temp_c(),
            time.time() - psutil.boot_time(),
            _read_wifi_signal_dbm(),
        )
        self._broadcast(protocol.stats_message(*self._last_stats))

    # ------------------------------------------------------------------------
    # Live parameter tuning.
    #
    # Threading contract, which matters more here than anywhere else in
    # this file: *every* piece of tuning state below is owned by the rclpy
    # spin thread and touched only from it. The IOLoop thread never reads
    # a target's values or writes a service request itself -- it drops an
    # item on `_tuning_requests` (a stdlib Queue, thread-safe by design)
    # and a timer on the rclpy thread picks it up. The one thing the
    # IOLoop reads directly is `_last_tuning_json`, and that is a plain
    # immutable string replaced wholesale on each broadcast, so a
    # newly-connected tab either gets the previous snapshot or the next
    # one, never a half-built dict.
    #
    # rclpy service calls are issued with call_async() and answered on the
    # executor's thread, which is this same spin thread -- so the done
    # callbacks below are already where they need to be.
    # ------------------------------------------------------------------------

    def _setup_tuning(self):
        """Build one _TuningTarget per configured node, plus its timers."""
        self._tuning_targets = []
        self._tuning_requests = queue.Queue()
        if not self.enable_tuning:
            # Still publish one "disabled" snapshot rather than nothing, so
            # a browser hides the panel outright instead of sitting on
            # "looking for driving nodes..." forever.
            self._last_tuning_json = json.dumps(
                protocol.tuning_state_message([], False, False))
            return
        self._last_tuning_json = None

        names = [str(n) for n in self.get_parameter('tuning_nodes').value if str(n)]
        configs = [str(c) for c in self.get_parameter('tuning_config_files').value]
        for index, node_name in enumerate(names):
            config_ref = configs[index] if index < len(configs) else ''
            path, path_error = self._resolve_config_path(config_ref)
            if path_error:
                self.get_logger().warn(
                    f"live tuning: '{node_name}' can be tuned but not saved -- {path_error}")
            self._tuning_targets.append(_TuningTarget(node_name, path, path_error))

        self.tuning_refresh_timer = self.create_timer(
            self.tuning_refresh_sec, self._tuning_refresh_callback)
        # Deliberately much faster than the refresh: this one only drains
        # queued browser requests, and a slider that takes half a second
        # to reach the car is a slider nobody trusts.
        self.tuning_request_timer = self.create_timer(
            1.0 / self.tuning_request_rate_hz, self._tuning_drain_callback)

    def _tuning_summary(self):
        if not self.enable_tuning:
            return 'DISABLED (enable_tuning is false)'
        return (f"enabled for {', '.join(t.node_name for t in self._tuning_targets) or '(no nodes)'}"
                f"{'' if self.tuning_allow_save else ', save to YAML disabled'}")

    def _resolve_config_path(self, config_ref):
        """'<package>/<path>' -> the real file on disk, or (None, why not).

        realpath() matters: with `colcon build --symlink-install` the
        installed config is a symlink chain back to the file in `src/`, so
        resolving it means "save" edits the git-tracked source the team
        actually reviews and commits, rather than a build artifact that
        the next rebuild would overwrite.
        """
        if not config_ref:
            return None, 'no config file is configured for it'
        package, _, relative = config_ref.partition('/')
        if not relative:
            return None, f"'{config_ref}' is not '<package>/<path>'"
        try:
            share = get_package_share_directory(package)
        except (PackageNotFoundError, KeyError, ValueError) as exc:
            return None, f"package '{package}' not found ({exc})"
        path = os.path.realpath(os.path.join(share, relative))
        if not os.path.isfile(path):
            return None, f"'{path}' does not exist"
        return path, None

    def _ensure_tuning_clients(self, target):
        if target.get_client is None:
            target.get_client = self.create_client(
                GetParameters, f'/{target.node_name}/get_parameters')
            target.set_client = self.create_client(
                SetParameters, f'/{target.node_name}/set_parameters')

    def _tuning_refresh_callback(self):
        """Track which driving nodes are up, and keep their values fresh."""
        now = time.monotonic()
        present = {name for name, _ in self.get_node_names_and_namespaces()}
        dirty = False

        for target in self._tuning_targets:
            if target.inflight is not None and now > target.inflight:
                # A service call that never came back. Drop it and let the
                # next tick retry rather than wedging this target forever.
                target.inflight = None

            if target.node_name not in present:
                if target.online:
                    # Forget the spec *and* the baseline: a node that
                    # comes back has re-read its YAML, so the values it
                    # had before this restart are no longer what "revert"
                    # should mean.
                    target.reset()
                    dirty = True
                continue

            if not target.online:
                target.online = True
                dirty = True
            self._ensure_tuning_clients(target)
            if target.inflight is not None:
                continue
            if not target.get_client.service_is_ready():
                continue
            if target.spec is None:
                self._request_tuning_spec(target)
            else:
                self._request_tuning_values(target)

        if dirty:
            self._broadcast_tuning_state()

    def _request_tuning_spec(self, target):
        request = GetParameters.Request()
        request.names = ['live_tunable_spec']
        target.inflight = time.monotonic() + self.tuning_service_timeout_sec
        future = target.get_client.call_async(request)
        future.add_done_callback(functools.partial(self._on_tuning_spec, target))

    def _on_tuning_spec(self, target, future):
        target.inflight = None
        try:
            values = future.result().values
        except Exception as exc:  # noqa: BLE001 -- a dead service must not kill the timer
            target.spec_error = f'could not read live_tunable_spec: {exc}'
            self._broadcast_tuning_state()
            return
        text = values[0].string_value if values else ''
        params, error = tuning.parse_spec(text)
        target.spec, target.spec_error = (params or None), error
        if error:
            self.get_logger().warn(f"live tuning: '{target.node_name}' {error}")
        else:
            self.get_logger().info(
                f"live tuning: '{target.node_name}' offers "
                f"{len(params)} tunable parameter(s)")
        self._broadcast_tuning_state()

    def _request_tuning_values(self, target):
        request = GetParameters.Request()
        request.names = [param['name'] for param in target.spec]
        target.inflight = time.monotonic() + self.tuning_service_timeout_sec
        future = target.get_client.call_async(request)
        future.add_done_callback(
            functools.partial(self._on_tuning_values, target, list(request.names)))

    def _on_tuning_values(self, target, names, future):
        target.inflight = None
        try:
            values = future.result().values
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"live tuning: value read failed: {exc}")
            return
        fresh = {}
        for name, value in zip(names, values):
            decoded = _decode_parameter_value(value)
            if decoded is not None:
                fresh[name] = decoded
        if fresh == target.values:
            return
        target.values = fresh
        if not target.baseline:
            # First good read after this node appeared: that is what its
            # config file says, and so what "revert" restores.
            target.baseline = dict(fresh)
        self._broadcast_tuning_state()

    def _tuning_drain_callback(self):
        """Apply whatever the browsers asked for since the last tick.

        Requests are coalesced per node and per parameter -- a slider
        dragged across its range enqueues a trail of values and only the
        last one is worth sending, so the bus sees one write instead of
        forty.
        """
        batches = {}
        save_requested = False
        refresh_requested = False
        while True:
            try:
                action, node_name, name, value = self._tuning_requests.get_nowait()
            except queue.Empty:
                break
            if action == 'save':
                save_requested = True
            elif action == 'refresh':
                refresh_requested = True
            elif action == 'set':
                batches.setdefault(node_name, {})[name] = value

        for node_name, updates in batches.items():
            self._apply_tuning_batch(node_name, updates)
        if save_requested:
            self._save_tuning_to_yaml()
        if refresh_requested and not batches:
            self._broadcast_tuning_state()

    def _apply_tuning_batch(self, node_name, updates):
        target = next(
            (t for t in self._tuning_targets if t.node_name == node_name), None)
        if target is None or not target.online or not target.spec:
            for name in updates:
                self._broadcast(protocol.tuning_result_message(
                    node_name, name, False,
                    reason=f"'{node_name}' is not running"))
            return

        by_name = {param['name']: param for param in target.spec}
        request = SetParameters.Request()
        ordered = []
        for name, raw in updates.items():
            param = by_name.get(name)
            if param is None:
                self._broadcast(protocol.tuning_result_message(
                    node_name, name, False,
                    reason=f"'{name}' is not live-tunable on this node"))
                continue
            value, error = tuning.coerce_request(param, raw)
            if error:
                self._broadcast(protocol.tuning_result_message(
                    node_name, name, False, reason=error))
                continue
            request.parameters.append(
                ParameterMsg(name=name, value=_encode_parameter_value(value)))
            ordered.append((name, value))
        if not ordered:
            return

        future = target.set_client.call_async(request)
        future.add_done_callback(
            functools.partial(self._on_tuning_set, target, ordered))

    def _on_tuning_set(self, target, ordered, future):
        try:
            results = future.result().results
        except Exception as exc:  # noqa: BLE001
            for name, _ in ordered:
                self._broadcast(protocol.tuning_result_message(
                    target.node_name, name, False, reason=f'set_parameters failed: {exc}'))
            return

        changed = False
        for (name, value), result in zip(ordered, results):
            if result.successful:
                target.values[name] = value
                changed = True
                self._broadcast(protocol.tuning_result_message(
                    target.node_name, name, True, value=value))
            else:
                # The node refused it. Echo its own reason and its
                # unchanged value, so the browser snaps the control back
                # to what the car actually holds.
                self._broadcast(protocol.tuning_result_message(
                    target.node_name, name, False,
                    value=target.values.get(name),
                    reason=result.reason or 'rejected by the node'))
                self.get_logger().warn(
                    f"live tuning: '{target.node_name}' refused {name}={value}: "
                    f"{result.reason}")
        if changed:
            self._broadcast_tuning_state()

    def _save_tuning_to_yaml(self):
        """Write the values the car is running right now into the package
        configs, so the next launch starts from this tune."""
        if not self.tuning_allow_save:
            self._broadcast(protocol.tuning_saved_message(
                False, 'saving is disabled on this dashboard (tuning_allow_save is false)'))
            return

        written, skipped = [], []
        for target in self._tuning_targets:
            if not target.online or not target.spec or not target.values:
                continue
            if target.config_path is None:
                skipped.append(f'{target.node_name} ({target.config_error})')
                continue
            tunable_now = {
                param['name']: target.values[param['name']]
                for param in target.spec if param['name'] in target.values
            }
            try:
                with open(target.config_path) as handle:
                    original = handle.read()
                pending = tuning.values_needing_save(
                    original, target.node_name, tunable_now)
                if not pending:
                    continue
                updated, changed, added = tuning.update_yaml_values(
                    original, target.node_name, pending)
                _atomic_write(target.config_path, updated)
            except (OSError, ValueError) as exc:
                skipped.append(f'{target.node_name} ({exc})')
                self.get_logger().error(
                    f"live tuning: could not save '{target.config_path}': {exc}")
                continue
            written.append(f'{target.config_path} ({len(changed) + len(added)} value(s))')
            self.get_logger().info(
                f"live tuning: saved {len(changed) + len(added)} value(s) to "
                f"'{target.config_path}'")

        if written:
            detail = 'Saved. Review with `git diff` before committing.'
        elif skipped:
            detail = 'Nothing saved.'
        else:
            detail = 'Nothing to save -- every value already matches its config file.'
        if skipped:
            detail += ' Skipped: ' + '; '.join(skipped)
        self._broadcast(protocol.tuning_saved_message(
            bool(written) or not skipped, detail, written))

    def _tuning_state_nodes(self):
        nodes = []
        for target in self._tuning_targets:
            nodes.append({
                'node': target.node_name,
                'online': target.online,
                'error': target.spec_error,
                'savable': target.config_path is not None and self.tuning_allow_save,
                'config_path': target.config_path or '',
                'params': [
                    dict(param,
                         value=target.values.get(param['name']),
                         baseline=target.baseline.get(param['name']))
                    for param in (target.spec or [])
                ],
            })
        return nodes

    def _broadcast_tuning_state(self):
        """Publish the whole tuning picture. Runs on the rclpy thread."""
        message = protocol.tuning_state_message(
            self._tuning_state_nodes(), self.enable_tuning, self.tuning_allow_save)
        self._last_tuning_json = json.dumps(message)
        self._broadcast(message)

    # --- Entry points called from the IOLoop thread (see the contract above) ---

    def request_tuning_set(self, node_name, name, value):
        self._tuning_requests.put(('set', str(node_name or ''), str(name or ''), value))

    def request_tuning_save(self):
        self._tuning_requests.put(('save', None, None, None))

    def broadcast_tuning_state(self):
        self._tuning_requests.put(('refresh', None, None, None))

    # ------------------------------------------------------------------------
    # Bridging ROS callbacks (rclpy thread) -> the Tornado IOLoop thread.
    # ------------------------------------------------------------------------

    def _broadcast(self, header: dict, binary_payload: bytes = None):
        if self._loop is None:
            return  # web server hasn't started listening yet -- nothing to send to
        self._loop.add_callback(functools.partial(self._send_to_all, header, binary_payload))

    def _send_to_all(self, header: dict, binary_payload):
        """Runs on the IOLoop thread (via add_callback) -- only safe place
        to touch WebSocket connections."""
        dead = []
        for client in list(self.ws_clients):
            try:
                client.write_message(json.dumps(header))
                if binary_payload is not None:
                    client.write_message(binary_payload, binary=True)
            except tornado.websocket.WebSocketClosedError:
                dead.append(client)
        for client in dead:
            self.ws_clients.discard(client)

    def send_initial_state(self, client: DashboardWebSocket):
        """Runs on the IOLoop thread (called from WebSocketHandler.open) --
        catches a freshly connected browser tab up on whatever this node
        already knows, instead of leaving it blank until the next update."""
        if self._last_map_msg is not None:
            client.write_message(json.dumps(protocol.map_header(self._last_map_msg)))
            client.write_message(protocol.map_cells(self._last_map_msg), binary=True)
        if self._last_scan_msg is not None:
            client.write_message(json.dumps(
                protocol.scan_header(self._last_scan_msg, self.laser_offset_x, self.laser_offset_y)))
            client.write_message(protocol.scan_ranges(self._last_scan_msg), binary=True)
        if self._last_pose is not None:
            client.write_message(json.dumps(protocol.pose_message(*self._last_pose)))
        if self._last_drive is not None:
            client.write_message(json.dumps(protocol.drive_message(*self._last_drive)))
        if self._last_speed is not None:
            client.write_message(json.dumps(protocol.speed_message(self._last_speed)))
        if self._last_stats is not None:
            client.write_message(json.dumps(protocol.stats_message(*self._last_stats)))
        client.write_message(json.dumps(self._stopwatch_message()))
        # Tuning state is sent as the last snapshot the rclpy thread
        # published (a plain string -- see the threading contract), and
        # always alongside an explicit "disarmed", so a reconnecting tab
        # can never come back still armed from before.
        if self._last_tuning_json is not None:
            client.write_message(self._last_tuning_json)
        client.write_message(json.dumps(protocol.tuning_armed_message(False)))

    # ------------------------------------------------------------------------
    # Web server
    # ------------------------------------------------------------------------

    def make_app(self) -> tornado.web.Application:
        static_dir = os.path.join(get_package_share_directory('web_dashboard'), 'web')
        return tornado.web.Application([
            (r'/ws', DashboardWebSocket, {'node': self}),
            # Catch-all *after* /ws -- Tornado matches routes in order, so
            # /ws must be registered first or StaticFileHandler's '.*'
            # would swallow the WebSocket upgrade request too.
            (r'/(.*)', tornado.web.StaticFileHandler, {'path': static_dir, 'default_filename': 'index.html'}),
        ])


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app = node.make_app()
    app.listen(node.port, address=node.host)
    node._loop = tornado.ioloop.IOLoop.current()

    node.get_logger().info(f"Serving on http://{node.host}:{node.port}/ (Ctrl+C to stop)")
    try:
        node._loop.start()
    except KeyboardInterrupt:
        pass
    finally:
        node._loop.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
