"""
The real WebSocket handler on a real socket (server.py), against a fake
node -- no ROS, no car.

What only an end-to-end test can show, and so what this file is for:
  * hello is the FIRST frame on every connection, before initial state;
  * the Origin check actually answers 403 before the upgrade;
  * an unknown X-Racerbot-Role is an HTTP 400, not an open socket;
  * a relay's write never reaches the node, and it is told why;
  * a control connection's write reaches the node tagged with its id,
    and every write is logged with the user;
  * send_to_all routes by role -- binary frames included;
  * serve_static: false serves /ws and a 404 for every page.

Oracle: the contract in docs/web-dashboard.md, "Remote access through
dashboard.sfuracerbot.ca".

    python3 -m pytest src/web_dashboard/test/test_server.py -v
"""
import json
import os
import tempfile

import tornado.gen
import tornado.httpclient
import tornado.testing
import tornado.websocket

from web_dashboard import protocol
from web_dashboard.origins import parse_allowed_origins
from web_dashboard.server import make_app, send_to_all

SITE = 'https://dashboard.sfuracerbot.ca'


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, message, **_):
        self.lines.append(('info', message))

    def warn(self, message, **_):
        self.lines.append(('warn', message))


class FakeNode:
    """Just the surface server.py calls. Records every request it is
    handed, so a test can prove a refused write never arrived."""

    def __init__(self):
        self.ws_clients = set()
        self.allowed_origins, _ = parse_allowed_origins([SITE])
        self.logger = _Logger()
        self.enable_tuning = True
        self.enable_process_control = True
        self.enable_map_delete = True
        self.enable_slam_reset = True
        self.calls = []

    def get_logger(self):
        return self.logger

    def send_initial_state(self, client):
        # Mirrors the node's shape: a binary-carrying map keyframe first,
        # then a stopwatch and a tuning snapshot, all through client.send.
        client.send({'type': 'map', 'seq': 1, 'bytes': 3}, b'\x00\x01\x02')
        client.send({'type': 'stopwatch', 'elapsed_s': 0.0})
        client.send({'type': 'tuning', 'nodes': []})

    def handle_stopwatch_control(self, action, enabled=None):
        self.calls.append(('stopwatch', action))

    def request_process_stop(self, pid, origin=None):
        self.calls.append(('stop', pid, origin))

    def request_process_refresh(self):
        self.calls.append(('process_refresh',))

    def request_map_refresh(self):
        self.calls.append(('map_refresh',))

    def request_map_delete(self, run_id, typed, digest, origin=None):
        self.calls.append(('delete', run_id, origin))

    def request_slam_reset(self, origin=None):
        self.calls.append(('reset_slam', origin))

    def send_map_keyframe(self, client):
        client.send({'type': 'map_cleared', 'has_map': True})

    def request_tuning_set(self, node_name, name, value, origin=None):
        self.calls.append(('set', node_name, name, value, origin))

    def request_tuning_save(self, origin=None):
        self.calls.append(('save', origin))

    def broadcast_tuning_state(self):
        self.calls.append(('tuning_refresh',))


class _ServerCase(tornado.testing.AsyncHTTPTestCase):
    static_dir = None

    def get_app(self):
        self.node = FakeNode()
        return make_app(self.node, self.static_dir)

    async def connect(self, role=None, user=None, origin=None, host=None):
        headers = {}
        if role is not None:
            headers['X-Racerbot-Role'] = role
        if user is not None:
            headers['X-Racerbot-User'] = user
        if origin is not None:
            headers['Origin'] = origin
        if host is not None:
            headers['Host'] = host
        request = tornado.httpclient.HTTPRequest(
            f'ws://127.0.0.1:{self.get_http_port()}/ws', headers=headers)
        return await tornado.websocket.websocket_connect(request)

    async def read_json(self, ws):
        frame = await ws.read_message()
        assert isinstance(frame, str), f'expected a JSON frame, got {frame!r}'
        return json.loads(frame)

    async def read_initial(self, ws, count):
        return [await ws.read_message() for _ in range(count)]


class HelloTest(_ServerCase):

    @tornado.testing.gen_test
    async def test_hello_is_the_first_frame_for_every_role(self):
        for role in (None, 'relay', 'control'):
            ws = await self.connect(role=role, user='alice@sfu.ca')
            first = await self.read_json(ws)
            assert first == {'type': 'hello',
                             'protocol_version': protocol.PROTOCOL_VERSION}
            ws.close()

    @tornado.testing.gen_test
    async def test_hello_carries_exactly_the_contract_keys(self):
        ws = await self.connect()
        assert set((await self.read_json(ws)).keys()) == {'type', 'protocol_version'}


class OriginTest(_ServerCase):

    @tornado.testing.gen_test
    async def test_the_remote_site_is_accepted(self):
        ws = await self.connect(role='relay', origin=SITE,
                                host='rb2-dash-origin.sfuracerbot.ca')
        assert (await self.read_json(ws))['type'] == 'hello'

    @tornado.testing.gen_test
    async def test_a_lookalike_site_gets_403_before_the_upgrade(self):
        for origin in ('https://dashboard.sfuracerbot.ca.evil.example',
                       'http://dashboard.sfuracerbot.ca',
                       'https://example.com'):
            try:
                await self.connect(origin=origin, host='rb2-dash-origin.sfuracerbot.ca')
            except tornado.httpclient.HTTPClientError as exc:
                assert exc.code == 403, origin
            else:
                raise AssertionError(f'{origin} was allowed to connect')

    @tornado.testing.gen_test
    async def test_a_same_origin_page_is_accepted(self):
        port = self.get_http_port()
        ws = await self.connect(origin=f'http://127.0.0.1:{port}',
                                host=f'127.0.0.1:{port}')
        assert (await self.read_json(ws))['type'] == 'hello'


class RoleTest(_ServerCase):

    @tornado.testing.gen_test
    async def test_an_unknown_role_gets_400_and_no_socket(self):
        try:
            await self.connect(role='admin')
        except tornado.httpclient.HTTPClientError as exc:
            assert exc.code == 400
        else:
            raise AssertionError('an unknown role was allowed to connect')
        assert self.node.ws_clients == set()

    @tornado.testing.gen_test
    async def test_a_relay_write_never_reaches_the_node_and_is_explained(self):
        ws = await self.connect(role='relay')
        await self.read_initial(ws, 1 + 4)       # hello, map + binary, stopwatch, tuning
        writes = [
            {'type': 'tuning_control', 'action': 'arm', 'armed': True},
            {'type': 'tuning_control', 'action': 'set', 'node': 'n', 'name': 'x', 'value': 1},
            {'type': 'tuning_control', 'action': 'save'},
            {'type': 'process_control', 'action': 'stop', 'pid': 4242},
            {'type': 'map_control', 'action': 'delete', 'id': 'r', 'confirm': 'r'},
            {'type': 'map_control', 'action': 'reset_slam'},
            {'type': 'stopwatch_control', 'action': 'set_enabled', 'enabled': True},
            {'type': 'stopwatch_control', 'action': 'reset'},
        ]
        for message in writes:
            ws.write_message(json.dumps(message))
            reply = await self.read_json(ws)
            assert reply['type'] == 'write_refused'
            assert reply['request'] == message['type']
            assert reply['action'] == message['action']
            assert 'use /control' in reply['detail']
        assert self.node.calls == []

    @tornado.testing.gen_test
    async def test_a_relay_read_still_works(self):
        ws = await self.connect(role='relay')
        await self.read_initial(ws, 5)
        ws.write_message(json.dumps({'type': 'process_control', 'action': 'refresh'}))
        ws.write_message(json.dumps({'type': 'map_control', 'action': 'clear_view'}))
        assert (await self.read_json(ws))['type'] == 'map_cleared'
        assert self.node.calls == [('process_refresh',)]

    @tornado.testing.gen_test
    async def test_a_control_write_reaches_the_node_tagged_and_logged(self):
        ws = await self.connect(role='control', user='alice@sfu.ca')
        await self.read_initial(ws, 3)            # hello, stopwatch, tuning -- no map
        ws.write_message(json.dumps(
            {'type': 'process_control', 'action': 'stop', 'pid': 4242}))
        ws.write_message(json.dumps({'type': 'map_control', 'action': 'reset_slam'}))
        # A read, to know both writes have been handled.
        ws.write_message(json.dumps({'type': 'process_control', 'action': 'refresh'}))
        while ('process_refresh',) not in self.node.calls:
            await tornado.gen.sleep(0.01)
        (client,) = self.node.ws_clients
        assert self.node.calls == [
            ('stop', 4242, client.conn_id), ('reset_slam', client.conn_id),
            ('process_refresh',)]
        logged = [m for level, m in self.node.logger.lines if 'dashboard write' in m]
        assert len(logged) == 2
        assert all('user alice@sfu.ca' in m and 'accepted' in m for m in logged)

    @tornado.testing.gen_test
    async def test_a_direct_write_is_logged_as_unknown_direct(self):
        ws = await self.connect()
        await self.read_initial(ws, 5)
        ws.write_message(json.dumps({'type': 'stopwatch_control', 'action': 'reset'}))
        while not self.node.calls:
            await tornado.gen.sleep(0.01)
        assert self.node.calls == [('stopwatch', 'reset')]
        (line,) = [m for _, m in self.node.logger.lines if 'dashboard write' in m]
        assert 'user unknown (direct)' in line

    @tornado.testing.gen_test
    async def test_the_tuning_arm_is_still_per_connection_on_control(self):
        """Arming on one control socket does not arm another, and a set on
        the unarmed one is refused and never reaches the node."""
        armed = await self.connect(role='control', user='a@x.ca')
        other = await self.connect(role='control', user='b@x.ca')
        await self.read_initial(armed, 3)
        await self.read_initial(other, 3)
        armed.write_message(json.dumps(
            {'type': 'tuning_control', 'action': 'arm', 'armed': True}))
        reply = await self.read_json(armed)
        assert (reply['type'], reply['armed']) == ('tuning_armed', True)
        other.write_message(json.dumps({'type': 'tuning_control', 'action': 'set',
                                        'node': 'n', 'name': 'x', 'value': 1}))
        refused = await self.read_json(other)
        assert refused['type'] == 'tuning_result' and refused['ok'] is False
        assert 'not armed' in refused['reason']
        assert not any(call[0] == 'set' for call in self.node.calls)


class RoutingTest(_ServerCase):
    """send_to_all with one connection of each role."""

    async def _three(self):
        direct = await self.connect()
        relay = await self.connect(role='relay')
        control = await self.connect(role='control', user='c@x.ca')
        await self.read_initial(direct, 5)
        await self.read_initial(relay, 5)
        await self.read_initial(control, 3)
        by_role = {c.role: c for c in self.node.ws_clients}
        return direct, relay, control, by_role['control'].conn_id

    @tornado.testing.gen_test
    async def test_relay_and_direct_get_map_and_binary_control_gets_neither(self):
        direct, relay, control, _ = await self._three()
        send_to_all(self.node.ws_clients, {'type': 'map_patch', 'bytes': 2}, b'\x07\x08')
        send_to_all(self.node.ws_clients, {'type': 'processes', 'targets': []})
        for ws in (direct, relay):
            assert (await self.read_json(ws))['type'] == 'map_patch'
            assert await ws.read_message() == b'\x07\x08'
            assert (await self.read_json(ws))['type'] == 'processes'
        # The very next frame on control is the process list: the patch
        # header and its binary were both skipped.
        assert (await self.read_json(control))['type'] == 'processes'

    @tornado.testing.gen_test
    async def test_a_reply_reaches_its_own_control_socket_and_not_another(self):
        direct, relay, control, control_id = await self._three()
        stranger = await self.connect(role='control', user='s@x.ca')
        await self.read_initial(stranger, 3)
        reply = {'type': 'process_result', 'pid': 4242, 'ok': True}
        send_to_all(self.node.ws_clients, reply, None, frozenset({control_id}))
        marker = {'type': 'saved_maps', 'runs': []}
        send_to_all(self.node.ws_clients, marker)
        assert await self.read_json(control) == reply
        assert await self.read_json(stranger) == marker      # reply skipped
        for ws in (direct, relay):                            # everyone else, as before
            assert await self.read_json(ws) == reply

    @tornado.testing.gen_test
    async def test_control_gets_the_stopwatch_out_of_a_batch(self):
        direct, relay, control, _ = await self._three()
        stopwatch = {'type': 'stopwatch', 'elapsed_s': 3.25}
        batch = {'type': 'batch', 'items': [{'type': 'pose', 'x': 1.0}, stopwatch]}
        send_to_all(self.node.ws_clients, batch)
        assert await self.read_json(control) == stopwatch
        assert await self.read_json(relay) == batch


class StaticOnTest(_ServerCase):

    def get_app(self):
        self._tmp = tempfile.TemporaryDirectory()
        with open(os.path.join(self._tmp.name, 'index.html'), 'w') as handle:
            handle.write('<title>fallback</title>')
        self.static_dir = self._tmp.name
        return super().get_app()

    def tearDown(self):
        super().tearDown()
        self._tmp.cleanup()

    def test_the_fallback_page_is_served(self):
        response = self.fetch('/')
        assert response.code == 200
        assert b'fallback' in response.body


class StaticOffTest(_ServerCase):
    static_dir = None

    def setUp(self):
        # Run from a directory that DOES hold a page, so a server that
        # quietly falls back to serving something (the cwd, a default
        # path) returns 200 here instead of a coincidental 404.
        self._tmp = tempfile.TemporaryDirectory()
        with open(os.path.join(self._tmp.name, 'index.html'), 'w') as handle:
            handle.write('<title>must not be served</title>')
        self._cwd = os.getcwd()
        os.chdir(self._tmp.name)
        super().setUp()

    def tearDown(self):
        super().tearDown()
        os.chdir(self._cwd)
        self._tmp.cleanup()

    def test_every_page_is_a_404(self):
        for path in ('/', '/index.html', '/dashboard.js', '/anything/else'):
            response = self.fetch(path)
            assert response.code == 404, path
            assert b'must not be served' not in response.body

    @tornado.testing.gen_test
    async def test_the_websocket_still_works(self):
        ws = await self.connect(role='relay')
        assert (await self.read_json(ws))['type'] == 'hello'
