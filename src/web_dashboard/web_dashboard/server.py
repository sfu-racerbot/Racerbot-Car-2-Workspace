"""
The dashboard's web server: the WebSocket handler, the page routes, and
the one loop that fans a message out to every connection.

Split out of dashboard_node.py so it has no rclpy import. Tornado is a
plain Python library, so test/test_server.py can run the real handler on
a real socket -- hello first, the Origin check, role refusal and routing,
serve_static -- against a small fake node, with no ROS and no car.

The node object passed in only needs what the handler actually calls:
`ws_clients`, `allowed_origins`, `get_logger()`, `send_initial_state()`,
the `enable_*` flags, and the `request_*`/`handle_*` entry points. All
of those hand work to the rclpy thread through queues; nothing here ever
touches a ROS handle. See dashboard_node.py's threading contract.
"""

import itertools
import json
import time

import tornado.web
import tornado.websocket

from web_dashboard import origins, protocol, roles


class _NotFoundHandler(tornado.web.RequestHandler):
    """Every page request when serve_static is false: a plain 404. The
    remote site serves the frontend; this node only answers /ws."""

    def get(self, *_args):
        raise tornado.web.HTTPError(404)


class DashboardWebSocket(tornado.websocket.WebSocketHandler):
    """One instance per connected browser tab. Pure bookkeeping -- all the
    actual data comes from DashboardNode via _broadcast()/send_initial_state().

    Each connection has a role (see roles.py): None for a direct LAN or
    Tailscale browser, which behaves exactly as it always has, or 'relay' /
    'control' for the remote site's connections through the tunnel. Every
    frame sent to a connection goes through send(), which applies that
    role's filter -- so no call site can forget it.
    """

    _ids = itertools.count(1)

    def initialize(self, node):
        self.node = node
        # Live tuning is armed per connection and starts disarmed on every
        # single page load -- see arm_tuning() below.
        self.tuning_armed = False
        # A plain integer rather than the handler object, so it can ride
        # through the rclpy thread's queues and come back as a tag on the
        # reply without that thread ever holding a Tornado object.
        self.conn_id = next(self._ids)
        self.role = roles.ROLE_DIRECT
        self.user = roles.UNKNOWN_USER

    async def get(self, *args, **kwargs):
        # Decided before the upgrade, so an unrecognised role is an HTTP
        # 400 the far side can see in its logs, not a socket that opens
        # and then behaves in some way nobody agreed on.
        role, error = roles.parse_role(
            self.request.headers.get('X-Racerbot-Role'))
        if error:
            self.node.get_logger().warn(
                f'refused a WebSocket from {self.request.remote_ip}: {error}')
            self.set_status(400)
            self.finish(error)
            return
        self.role = role
        self.user = roles.format_user(
            self.request.headers.get('X-Racerbot-User'), role)
        await super().get(*args, **kwargs)

    def check_origin(self, origin):
        # Same-origin (every LAN/Tailscale page this node serves itself),
        # or an exact match in allowed_origins
        # (the remote site). Anything else is a 403 before the upgrade.
        #
        # This used to accept every origin. That let any web page open in a
        # browser on the car's network reach the write paths below -- the
        # arm, the process stop, the map delete. They are still bounded the
        # way docs/web-dashboard.md's security note describes, and still
        # belong on a trusted network; this only closes the drive-by half.
        # See origins.py.
        allowed = origins.is_origin_allowed(
            origin, self.request.headers.get('Host'), self.node.allowed_origins)
        if not allowed:
            self.node.get_logger().warn(
                f'refused a WebSocket from origin {origin[:120]!r} '
                f'(not same-origin and not in allowed_origins)')
        return allowed

    def open(self):
        # hello goes first, before this connection is visible to any
        # broadcast: the remote site reads the protocol version off it
        # before it interprets anything else.
        self.write_message(json.dumps(protocol.hello_message()))
        self.node.ws_clients.add(self)
        # Every connection, every role, opened and closed: "nothing from the
        # site ever arrived" has to be something the log can show.
        # remote_check.py reads these lines back.
        self._opened_at = time.monotonic()
        self.node.get_logger().info(
            f'{self.role or "direct"} connection #{self.conn_id} opened '
            f'(user {self.user}, from {self.request.remote_ip}, '
            f'{len(self.node.ws_clients)} open)')
        self.node.send_initial_state(self)

    def on_close(self):
        self.node.ws_clients.discard(self)
        held = time.monotonic() - getattr(self, '_opened_at', time.monotonic())
        self.node.get_logger().info(
            f'{self.role or "direct"} connection #{self.conn_id} closed after '
            f'{held:.0f}s (code {self.close_code}, {len(self.node.ws_clients)} open)')

    def send(self, header, binary_payload=None, is_origin=True):
        """Write one message to this connection, through its role filter.

        IOLoop thread only. `is_origin` defaults to True because a call
        made directly on this handler is, by construction, an answer to
        this connection.
        """
        for frame, with_binary in roles.frames_for(self.role, header, is_origin):
            self.write_message(json.dumps(frame))
            if with_binary and binary_payload is not None:
                self.write_message(binary_payload, binary=True)

    def _log_write(self, payload, outcome):
        self.node.get_logger().info(
            f'dashboard write: {roles.describe_request(payload)} -- {outcome} '
            f'(user {self.user}, {self.role or "direct"} connection '
            f'#{self.conn_id}, from {self.request.remote_ip})')

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
        self.send(protocol.tuning_armed_message(self.tuning_armed))

    def on_message(self, message):
        """Browser -> server. Four kinds of input are accepted:

          stopwatch_control  affects nothing outside this process
          process_control    signals a driving process (never the mux)
          tuning_control     reaches the driving nodes, behind an arm
          map_control        clears the browser's map view, resets live
                             SLAM, or deletes a saved run from disk

        Every one of them ends in a queue hand-off or a local write. None
        of them touches a ROS handle here -- see the threading contract at
        the top of this file, and check_origin above for what does and does
        not protect these."""
        if not isinstance(message, str):
            return
        try:
            payload = json.loads(message)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return

        kind = payload.get('type')
        # Every write is logged with who sent it, and the relay may send
        # none at all -- see roles.py. Reads (refresh, clear_view) are
        # neither logged nor refused.
        if roles.is_write(payload):
            refusal = roles.inbound_refusal(self.role, payload)
            self._log_write(payload, f'REFUSED: {refusal}' if refusal else 'accepted')
            if refusal:
                self.send(protocol.write_refused_message(
                    kind, payload.get('action'), refusal))
                return
        if kind == 'stopwatch_control':
            self.node.handle_stopwatch_control(
                payload.get('action'), payload.get('enabled'))
            return
        if kind == 'process_control':
            # Deliberately *not* behind the tuning arm. Arming exists so a
            # pocket-tap cannot change how a moving car drives; the worst
            # a mistaken press does here is stop a driving node, which is
            # the direction of travel you want a mistake to go in. The UI
            # still asks for a confirm, and the server still re-vets every
            # pid against a fresh scan before signalling anything.
            if not self.node.enable_process_control:
                self.send(protocol.process_result_message(
                    0, '', False, 'stopping processes is disabled on this '
                                  'dashboard (enable_process_control is false)'))
                return
            action = payload.get('action')
            if action == 'stop':
                self.node.request_process_stop(payload.get('pid'), self.conn_id)
            elif action == 'refresh':
                self.node.request_process_refresh()
            return
        if kind == 'map_control':
            # Three actions, three gates. Nothing here touches a ROS handle
            # or the filesystem -- every one of these ends in a queue.put()
            # that the rclpy thread drains, and every decision that matters
            # is made there against a scan taken at that moment.
            action = payload.get('action')
            if action == 'clear_view':
                # Costs the car nothing and changes nothing on it, so it is
                # not gated at all -- it is the browser forgetting its own
                # copy of the map.
                self.node.send_map_keyframe(self)
                return
            if action == 'refresh':
                self.node.request_map_refresh()
                return
            if action == 'delete':
                if not self.node.enable_map_delete:
                    self.send(protocol.map_delete_result_message(
                        payload.get('id'), False,
                        'deleting saved maps is disabled on this '
                        'dashboard (enable_map_delete is false)'))
                    return
                self.node.request_map_delete(
                    payload.get('id'), payload.get('confirm'),
                    payload.get('digest'), self.conn_id)
                return
            if action == 'reset_slam':
                if not self.node.enable_slam_reset:
                    self.send(protocol.slam_reset_result_message(
                        False, 'resetting SLAM is disabled on this '
                               'dashboard (enable_slam_reset is false)'))
                    return
                self.node.request_slam_reset(self.conn_id)
                return
            # An action nobody recognises gets an answer, not silence: a
            # button that does nothing and says nothing is one people press
            # again, and again.
            self.send(protocol.map_delete_result_message(
                payload.get('id'), False,
                f'unknown map action {str(action)[:40]!r}'))
            return

        if kind != 'tuning_control':
            return

        action = payload.get('action')
        if action == 'arm':
            self.arm_tuning(payload.get('armed'))
            return
        if not self.node.enable_tuning:
            self.send(protocol.tuning_saved_message(
                False, 'live tuning is disabled on this dashboard '
                       '(enable_tuning is false)'))
            return
        if not self.tuning_armed:
            # Refused server-side, not merely disabled in the UI: a stale
            # tab, a replayed message, or a hand-rolled WebSocket client
            # all land here too.
            self.send(protocol.tuning_result_message(
                str(payload.get('node', '')), str(payload.get('name', '')),
                False, reason='tuning is not armed on this connection'))
            return

        if action == 'set':
            self.node.request_tuning_set(
                payload.get('node'), payload.get('name'), payload.get('value'),
                self.conn_id)
        elif action == 'save':
            self.node.request_tuning_save(self.conn_id)
        elif action == 'refresh':
            self.node.broadcast_tuning_state()


def send_to_all(clients, header, binary_payload=None, origin_ids=frozenset()):
    """Send one message to every connection. IOLoop thread only.

    `origin_ids` are the conn_ids the message answers (empty for plain
    telemetry). Direct and relay connections get everything, exactly as
    before roles existed; a control connection gets what
    roles.frames_for() allows.
    """
    # Serialised once for every connection that takes the message whole
    # (all of them, except control connections).
    text = json.dumps(header)
    dead = []
    for client in list(clients):
        try:
            if client.role == roles.ROLE_CONTROL:
                client.send(header, binary_payload,
                            is_origin=client.conn_id in origin_ids)
                continue
            client.write_message(text)
            if binary_payload is not None:
                client.write_message(binary_payload, binary=True)
        except Exception:  # noqa: BLE001
            # Any failure, not just WebSocketClosedError. A header that
            # went out without the binary that explains it leaves that
            # browser's "what does the next binary mean" slot pointing
            # at the wrong thing, and it decodes the *next* payload as
            # the wrong type from then on -- a scan read as occupancy
            # cells paints the map as garbage. Dropping the client makes
            # it reconnect and resynchronise, which is the only honest
            # recovery from a half-sent pair.
            dead.append(client)
    for client in dead:
        clients.discard(client)
        try:
            client.close()
        except Exception:  # noqa: BLE001
            pass


def make_app(node, static_dir):
    """The Tornado application. `static_dir` None means serve_static is
    false: /ws only, and a 404 for every page."""
    if static_dir is not None:
        pages = (r'/(.*)', tornado.web.StaticFileHandler,
                 {'path': static_dir, 'default_filename': 'index.html'})
    else:
        # WebSocket only: the frontend is served by the remote site.
        pages = (r'/(.*)', _NotFoundHandler)
    return tornado.web.Application([
        (r'/ws', DashboardWebSocket, {'node': node}),
        # Catch-all *after* /ws -- Tornado matches routes in order, so
        # /ws must be registered first or StaticFileHandler's '.*'
        # would swallow the WebSocket upgrade request too.
        pages,
    ])
