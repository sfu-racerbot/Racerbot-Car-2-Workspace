"""
dashboard_node.py

Serves a live browser dashboard of what the car can see and where it is:
the SLAM/localization map, the raw LIDAR scan, and the car's own pose, all
drawn on an HTML5 canvas and pushed to any connected browser in real time
over a WebSocket. See docs/web-dashboard.md for the full picture,
including how the browser side lines everything up.

This node is entirely passive -- it only ever subscribes, it never
publishes anything -- so it carries zero risk to how the car drives and
can be left running at all times alongside anything else in this
workspace, including during an actual race.

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
import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

import tornado.ioloop
import tornado.web
import tornado.websocket
from ament_index_python.packages import get_package_share_directory

from web_dashboard import protocol


class DashboardWebSocket(tornado.websocket.WebSocketHandler):
    """One instance per connected browser tab. Pure bookkeeping -- all the
    actual data comes from DashboardNode via _broadcast()/send_initial_state()."""

    def initialize(self, node: 'DashboardNode'):
        self.node = node

    def check_origin(self, origin):
        # This dashboard is read-only (it never publishes to ROS, so a
        # malicious page could see the map/scan/pose but could never
        # command the car) and has no login -- see docs/web-dashboard.md's
        # security note for why that makes accepting any origin a
        # reasonable trade-off for a LAN-only debugging tool, and why it
        # should still never be exposed past a trusted network.
        return True

    def open(self):
        self.node.ws_clients.add(self)
        self.node.send_initial_state(self)

    def on_close(self):
        self.node.ws_clients.discard(self)

    def on_message(self, message):
        pass  # one-directional dashboard -- nothing expected from the browser


class DashboardNode(Node):
    """Subscribes to the map/scan/pose topics and fans out every update to
    every connected browser tab."""

    def __init__(self):
        super().__init__('web_dashboard_node')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_topic', '/pf/viz/inferred_pose')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('scan_broadcast_rate_hz', 10.0)
        self.declare_parameter('laser_offset_x', 0.27)
        self.declare_parameter('laser_offset_y', 0.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.scan_broadcast_rate_hz = float(self.get_parameter('scan_broadcast_rate_hz').value)
        self.laser_offset_x = float(self.get_parameter('laser_offset_x').value)
        self.laser_offset_y = float(self.get_parameter('laser_offset_y').value)

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
        self._last_scan_broadcast_time = 0.0

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
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)

        self.get_logger().info(
            f"web_dashboard_node ready: map={self.map_topic} scan={self.scan_topic} pose={self.pose_topic}. "
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

    def pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = protocol.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self._last_pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        self._broadcast(protocol.pose_message(*self._last_pose))

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
