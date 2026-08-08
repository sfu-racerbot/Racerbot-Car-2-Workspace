#!/usr/bin/env python3
"""Drive dashboard_node's publishing path directly and check what it puts
on the wire.

`src/web_dashboard/test/` covers the pieces that have no ROS dependency --
the map streamer, the batcher, the encodings -- and those tests are where
the detailed edge cases live. This fills the gap above them: that the
*node* is actually wired to those pieces, with its real parameters, real
ROS message types, and its real callbacks. That is the layer where this
workspace's bugs have historically actually been (see CLAUDE.md on why
there are two simulators), and it needs rclpy, so it cannot live in
`test/`.

No hardware, no network, no browser: the node is constructed, a fake
browser is attached so its "is anyone watching" guards let work through,
and its broadcasts are captured instead of being sent.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 tools/web_dashboard/check_wire_format.py

Exits 0 only if every check passes, so it can gate a change.
"""

from __future__ import annotations

import json
import struct
import sys
import zlib

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from web_dashboard.dashboard_node import DashboardNode


WIDTH, HEIGHT = 60, 40


class _FakeClient:
    """Stands in for a connected browser tab."""

    def __init__(self):
        self.messages = []

    def write_message(self, message, binary=False):
        self.messages.append(message)

    def close(self):
        pass


class Harness:
    """A node with its fan-out captured rather than sent."""

    def __init__(self):
        rclpy.init(args=['--ros-args',
                         '-p', 'port:=8099',
                         '-p', 'enable_tuning:=false'])
        self.node = DashboardNode()
        self.sent = []
        self.client = _FakeClient()
        self.node.ws_clients.add(self.client)
        # Run IOLoop callbacks inline, and capture instead of writing.
        self.node._loop = type(
            'Inline', (), {'add_callback': staticmethod(lambda fn: fn())})()
        self.node._send_to_all = lambda header, payload=None: self.sent.append(
            (header, payload))

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()

    @property
    def last(self):
        return self.sent[-1]


def _grid(cells: bytearray) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.info.width = WIDTH
    msg.info.height = HEIGHT
    msg.info.resolution = 0.05
    msg.info.origin.position.x = -1.0
    msg.info.origin.position.y = -2.0
    msg.info.origin.orientation.w = 1.0
    msg.data = list(memoryview(bytes(cells)).cast('b'))
    return msg


def _decode(header, payload):
    assert len(payload) == header['bytes'], (
        f"header promised {header['bytes']} bytes, frame is {len(payload)}")
    raw = zlib.decompress(payload) if header['encoding'] == 'deflate' else payload
    assert len(raw) == header['raw_bytes']
    return raw


class Checks:
    def __init__(self):
        self.failures = []
        self.passed = 0

    def check(self, label, condition, detail=''):
        if condition:
            self.passed += 1
            print(f'  ok    {label}')
        else:
            self.failures.append(f'{label}: {detail}')
            print(f'  FAIL  {label}  {detail}')


def main() -> int:
    harness = Harness()
    checks = Checks()
    node = harness.node

    try:
        print('map: keyframe, patch, and silence')
        cells = bytearray([255]) * (WIDTH * HEIGHT)   # 0xFF == -1 == unknown
        node.map_callback(_grid(cells))
        header, payload = harness.last
        checks.check('first grid is a keyframe', header['type'] == 'map', header['type'])
        checks.check('keyframe carries the whole grid',
                     _decode(header, payload) == bytes(cells))
        checks.check('keyframe geometry is the message geometry',
                     header['width'] == WIDTH and header['height'] == HEIGHT)

        for y in range(10, 14):
            for x in range(20, 25):
                cells[y * WIDTH + x] = 100
        node.map_callback(_grid(cells))
        header, payload = harness.last
        rect = (header.get('x'), header.get('y'), header.get('w'), header.get('h'))
        checks.check('a small change becomes a patch', header['type'] == 'map_patch',
                     header['type'])
        checks.check('the patch covers exactly the changed cells', rect == (20, 10, 5, 4),
                     str(rect))
        checks.check('the patch is far smaller than the grid',
                     header['bytes'] < WIDTH * HEIGHT / 4,
                     f"{header['bytes']} bytes")

        before = len(harness.sent)
        node.map_callback(_grid(cells))
        checks.check('an identical grid sends nothing at all',
                     len(harness.sent) == before,
                     f'{len(harness.sent) - before} frame(s)')

        print('map: a late joiner gets the current map, in sync')
        late = _FakeClient()
        node.send_initial_state(late)
        keyframe = [m for m in late.messages if isinstance(m, str) and '"map"' in m]
        checks.check('a connecting tab is sent a keyframe', bool(keyframe))
        kf = json.loads(keyframe[0])
        binary = [m for m in late.messages if isinstance(m, (bytes, bytearray))]
        checks.check('that keyframe is the map as it stands now',
                     _decode(kf, binary[0]) == bytes(cells))
        checks.check('and carries the sequence the next patch follows',
                     kf['seq'] == node._map_streamer.seq,
                     f"{kf['seq']} vs {node._map_streamer.seq}")

        print('scan: uint16 millimetres')
        scan = LaserScan()
        scan.angle_min, scan.angle_increment = -2.35, 0.0058
        scan.range_min, scan.range_max = 0.1, 10.0
        scan.ranges = [1.5] * 1081
        node._last_scan_broadcast_time = 0.0
        node.scan_callback(scan)
        header, payload = harness.last
        checks.check('scan is sent as u16mm', header['encoding'] == 'u16mm',
                     header['encoding'])
        checks.check('two bytes per beam', len(payload) == 2 * 1081, str(len(payload)))
        checks.check('header bytes matches the frame', header['bytes'] == len(payload))
        checks.check('1.5m round-trips as 1500mm',
                     struct.unpack('<H', payload[:2])[0] == 1500)

        print('telemetry: 155 messages a second become one frame a tick')
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0
        for i in range(30):
            pose.pose.position.x = float(i)
            node.pose_callback(pose, topic='/slam_pose')
        drive = AckermannDriveStamped()
        drive.drive.speed = 2.0
        for _ in range(20):
            node.drive_callback(drive)
        odom = Odometry()
        odom.twist.twist.linear.x = 1.9
        for _ in range(15):
            node.odom_callback(odom)

        before = len(harness.sent)
        node._flush_telemetry()
        frames = len(harness.sent) - before
        batch, _ = harness.last
        checks.check('65 messages produced exactly one frame', frames == 1, str(frames))
        checks.check('it is a batch', batch.get('type') == 'batch')
        poses = [i for i in batch['items'] if i['type'] == 'pose']
        checks.check('the newest pose wins', len(poses) == 1 and poses[0]['x'] == 29.0)

        print('telemetry: intent state transitions survive the batch')
        for state in ['racing', 'racing', 'emergency_stop', 'racing', 'racing']:
            node.intent_callback(String(data=_intent_json(state)))
        node._flush_telemetry()
        batch, _ = harness.last
        states = [i['intent']['state'] for i in batch['items'] if i['type'] == 'intent']
        checks.check('every transition is delivered',
                     states == ['racing', 'emergency_stop', 'racing'], str(states))

        print('nobody watching: the node stops doing the work')
        node.ws_clients.discard(harness.client)
        before = len(harness.sent)
        for i in range(30):
            pose.pose.position.x = float(100 + i)
            node.pose_callback(pose, topic='/slam_pose')
        node.scan_callback(scan)
        node._flush_telemetry()
        checks.check('no frames are produced with no clients',
                     len(harness.sent) == before,
                     f'{len(harness.sent) - before} frame(s)')
        checks.check('but the latest pose is still remembered for the next tab',
                     node._last_pose is not None and node._last_pose[0] == 129.0)
    finally:
        harness.close()

    print()
    if checks.failures:
        print(f'FAILED: {len(checks.failures)} check(s)')
        for failure in checks.failures:
            print(f'  - {failure}')
        return 1
    print(f'All {checks.passed} checks passed.')
    return 0


def _intent_json(state):
    """A real intent message, built through the schema the driving nodes
    actually publish -- not a hand-rolled lookalike, which would only prove
    that this file agrees with itself."""
    from drive_intent import schema  # noqa: PLC0415 - keeps the tool importable without it
    payload = schema.build(
        node='gap_follow_node',
        state=state,
        severity='drive' if state == 'racing' else 'stop',
        reason=f'now {state}',
        desired_speed=2.0, commanded_speed=2.0,
        desired_steering=0.0, commanded_steering=0.0,
        horizon_s=1.5,
        path=[(0.0, 0.0, 0.0, 2.0), (1.0, 0.0, 0.0, 2.0)],
        commanded_path=[(0.0, 0.0, 0.0, 2.0), (1.0, 0.0, 0.0, 2.0)],
    )
    return schema.encode(payload)


if __name__ == '__main__':
    sys.exit(main())
