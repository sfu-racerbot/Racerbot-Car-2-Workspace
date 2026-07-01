"""
Unit tests for web_dashboard.protocol -- pure data-shape conversions, no
ROS, no Tornado, no network, no browser. Fake ROS messages are built with
plain SimpleNamespace objects (matching just the fields protocol.py
actually reads) rather than real message classes, so these tests don't
even need rclpy importable. Run with:

    python3 -m pytest src/web_dashboard/test/test_protocol.py -v
"""
import math
import os
import struct
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from web_dashboard import protocol  # noqa: E402


def _quat(yaw):
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def test_quaternion_to_yaw_matches_known_angle():
    yaw = math.radians(37.0)
    q = _quat(yaw)
    assert protocol.quaternion_to_yaw(q.x, q.y, q.z, q.w) == pytest.approx(yaw)


def _fake_occupancy_grid(width=4, height=3, resolution=0.05, data=None):
    info = SimpleNamespace(
        width=width, height=height, resolution=resolution,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=-1.0, y=-2.0, z=0.0),
            orientation=_quat(0.0),
        ),
    )
    if data is None:
        data = [-1] * (width * height)
    return SimpleNamespace(info=info, data=data)


def test_map_header_carries_correct_metadata():
    msg = _fake_occupancy_grid(width=4, height=3, resolution=0.05)
    header = protocol.map_header(msg)
    assert header['type'] == 'map'
    assert header['width'] == 4
    assert header['height'] == 3
    assert header['resolution'] == pytest.approx(0.05)
    assert header['origin_x'] == pytest.approx(-1.0)
    assert header['origin_y'] == pytest.approx(-2.0)


def test_map_cells_round_trips_unknown_free_and_occupied():
    data = [-1, 0, 100, 50, -1, 0]
    msg = _fake_occupancy_grid(width=3, height=2, data=data)
    packed = protocol.map_cells(msg)
    assert len(packed) == len(data)
    unpacked = list(struct.unpack(f'<{len(data)}b', packed))
    assert unpacked == data


def _fake_laser_scan(ranges, angle_min=-1.0, angle_increment=0.01):
    return SimpleNamespace(
        angle_min=angle_min, angle_increment=angle_increment,
        range_min=0.1, range_max=10.0, ranges=ranges,
    )


def test_scan_header_carries_laser_offset_and_geometry():
    msg = _fake_laser_scan([1.0, 2.0, 3.0])
    header = protocol.scan_header(msg, laser_offset_x=0.27, laser_offset_y=0.0)
    assert header['type'] == 'scan'
    assert header['count'] == 3
    assert header['laser_offset_x'] == pytest.approx(0.27)
    assert header['angle_min'] == pytest.approx(-1.0)


def test_scan_ranges_round_trips_as_float32():
    ranges = [0.5, 1.2345, float('inf'), 9.999]
    msg = _fake_laser_scan(ranges)
    packed = protocol.scan_ranges(msg)
    unpacked = list(struct.unpack(f'<{len(ranges)}f', packed))
    for original, recovered in zip(ranges, unpacked):
        if math.isinf(original):
            assert math.isinf(recovered)
        else:
            assert recovered == pytest.approx(original, rel=1e-6)


def test_pose_message_shape():
    msg = protocol.pose_message(1.5, -2.5, math.pi / 4)
    assert msg['type'] == 'pose'
    assert msg['x'] == pytest.approx(1.5)
    assert msg['y'] == pytest.approx(-2.5)
    assert msg['yaw'] == pytest.approx(math.pi / 4)
    assert 'stamp' in msg
