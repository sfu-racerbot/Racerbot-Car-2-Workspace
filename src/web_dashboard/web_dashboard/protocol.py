"""
protocol.py

Pure-Python helpers that turn ROS2 messages into the wire format the
dashboard's browser-side JavaScript expects. No ROS, no Tornado, no
network code here -- just data-shape conversion -- so it's directly
unit-testable (see test/test_protocol.py) without a running robot,
browser, or web server.

Wire format: everything travels over one WebSocket connection as pairs of
messages -- one JSON *text* message describing "what is this and how do I
read the bytes that follow", immediately followed by one *binary* message
with the raw payload (skipped entirely for pose updates, which are small
enough to just be JSON):

  MAP:   {"type": "map",  ...metadata...} -> binary: int8 occupancy values,
         row-major, matching nav_msgs/OccupancyGrid.data exactly (-1
         unknown, 0 free, 100 occupied), one byte per cell.
  SCAN:  {"type": "scan", ...metadata...} -> binary: float32 ranges,
         little-endian, one 4-byte value per LaserScan.ranges entry.
  POSE:  {"type": "pose", "x":.., "y":.., "yaw":.., "stamp":..}  (no binary)

Both binary payloads are laid out to match a JavaScript TypedArray
byte-for-byte (Int8Array for the map, Float32Array for the scan), so the
browser needs no parsing beyond `new Int8Array(buf)` / `new
Float32Array(buf)` -- see web/dashboard.js.
"""

import math
import struct
import time


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about +Z) from a geometry_msgs Quaternion.

    Same standard atan2-based formula as pure_pursuit's racing_math.py --
    duplicated here rather than importing across packages for four lines
    of very standard, self-contained math.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def map_header(msg) -> dict:
    """JSON-serializable metadata for a nav_msgs/OccupancyGrid: everything
    the browser needs to place the map in world coordinates and size its
    canvas, except the actual cell data, which travels separately as a
    binary frame (see map_cells)."""
    info = msg.info
    o = info.origin.orientation
    return {
        'type': 'map',
        'width': int(info.width),
        'height': int(info.height),
        'resolution': float(info.resolution),
        'origin_x': float(info.origin.position.x),
        'origin_y': float(info.origin.position.y),
        'origin_yaw': quaternion_to_yaw(o.x, o.y, o.z, o.w),
        'stamp': time.time(),
    }


def map_cells(msg) -> bytes:
    """Raw occupancy bytes, one signed byte per cell, row-major -- an
    exact copy of OccupancyGrid.data, just packed for the wire.

    struct's signed-char format ('b') is what makes this a byte-for-byte
    match for JavaScript's Int8Array: a cell value of -1 (unknown) has to
    round-trip as the single byte 0xFF, which plain `bytes(data)` can't
    do (it only accepts 0-255), but `struct.pack('b', ...)` handles
    correctly by design.
    """
    data = list(msg.data)
    return struct.pack(f'<{len(data)}b', *data)


def scan_header(msg, laser_offset_x: float = 0.0, laser_offset_y: float = 0.0) -> dict:
    """JSON-serializable metadata for a sensor_msgs/LaserScan.

    Includes the LIDAR's mounting offset from base_link so the browser can
    place scan points correctly relative to the car's own pose without a
    second topic or a TF lookup -- see docs/web-dashboard.md for why this
    node reads pose from a plain topic instead of TF.
    """
    return {
        'type': 'scan',
        'angle_min': float(msg.angle_min),
        'angle_increment': float(msg.angle_increment),
        'range_min': float(msg.range_min),
        'range_max': float(msg.range_max),
        'count': len(msg.ranges),
        'laser_offset_x': float(laser_offset_x),
        'laser_offset_y': float(laser_offset_y),
        'stamp': time.time(),
    }


def scan_ranges(msg) -> bytes:
    """Raw range floats, one little-endian float32 per beam -- matches
    JavaScript's Float32Array byte-for-byte."""
    ranges = list(msg.ranges)
    return struct.pack(f'<{len(ranges)}f', *ranges)


def pose_message(x: float, y: float, yaw: float) -> dict:
    """The whole pose fits comfortably in JSON -- no binary payload needed."""
    return {
        'type': 'pose',
        'x': float(x),
        'y': float(y),
        'yaw': float(yaw),
        'stamp': time.time(),
    }


def drive_message(speed: float, steering_angle: float) -> dict:
    """The currently-arbitrated drive command (whatever /drive carries),
    small enough to just be JSON like pose_message."""
    return {
        'type': 'drive',
        'speed': float(speed),
        'steering_angle': float(steering_angle),
        'stamp': time.time(),
    }


def stats_message(cpu_percent: float, mem_percent: float, cpu_temp_c, uptime_s: float,
                   wifi_dbm=None) -> dict:
    """Coarse system health, sampled on a timer rather than per-message --
    cpu_temp_c/wifi_dbm are None if no readable thermal zone / wireless
    interface was found (not every machine this could run on has one)."""
    return {
        'type': 'stats',
        'cpu_percent': float(cpu_percent),
        'mem_percent': float(mem_percent),
        'cpu_temp_c': None if cpu_temp_c is None else float(cpu_temp_c),
        'uptime_s': float(uptime_s),
        'wifi_dbm': None if wifi_dbm is None else float(wifi_dbm),
        'stamp': time.time(),
    }
