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
with the raw payload (skipped for all compact telemetry updates, which fit
comfortably in JSON):

  MAP:   {"type": "map",  ...metadata...} -> binary: int8 occupancy values,
         row-major, matching nav_msgs/OccupancyGrid.data exactly (-1
         unknown, 0 free, 100 occupied), one byte per cell.
  SCAN:  {"type": "scan", ...metadata...} -> binary: float32 ranges,
         little-endian, one 4-byte value per LaserScan.ranges entry.

  POSE/DRIVE/SPEED/STOPWATCH/STATS: compact JSON only (no binary payload).
  INTENT: compact JSON only -- a driving node's own description of what it
         is trying to do, forwarded almost unchanged from /drive_intent.
         See drive_intent/schema.py and docs/drive-intent.md.

Both binary payloads are laid out to match a JavaScript TypedArray
byte-for-byte (Int8Array for the map, Float32Array for the scan), so the
browser needs no parsing beyond `new Int8Array(buf)` / `new
Float32Array(buf)` -- see web/dashboard.js.

Both binary-carrying headers also declare `bytes`, the exact length of the
frame that must follow. The browser holds a single "what does the next
binary mean" slot, so a header that never gets its binary -- a write that
fails between the two, a proxy that drops a frame -- leaves that slot
pointing at the wrong thing, and the *next* binary is then decoded as the
previous type. A 1081-beam scan payload read as occupancy cells is 4324
bytes against an 80000-cell header: every read past the end is undefined,
every colour computes to NaN, and the map paints as garbage rather than
failing. `bytes` makes that detectable instead of silent -- see
web/dashboard.js handleBinary.
"""

import math
import struct
import sys
import time


# The wire format is little-endian by definition (it has to match a
# JavaScript TypedArray, which is little-endian on every platform a browser
# runs on). On a little-endian host -- every machine this workspace targets:
# the Jetson's ARM64, and x86 laptops -- `array.array` already holds exactly
# those bytes, so the fast paths below can hand the buffer straight over.
# On a big-endian host they would be byte-swapped, so the explicit
# struct.pack('<...') fallback is what runs instead.
_LITTLE_ENDIAN = sys.byteorder == 'little'

try:
    import numpy as _np
except ImportError:  # pragma: no cover - present in any ROS2 env; degrade anyway
    _np = None

#: Scan payload encodings. 'f32' is the original one float per beam.
#: 'u16mm' is half the size for no visible difference: the browser paints
#: each return as a 2x2 pixel dot, so millimetre quantisation is far below
#: anything that can be seen, and it is already below the LIDAR's own
#: ~30mm accuracy. Invalid returns (inf/NaN/out of range) encode as 0,
#: which the browser already discards -- it filters everything below
#: range_min, and range_min is never 0 on a real scanner.
SCAN_F32 = 'f32'
SCAN_U16MM = 'u16mm'

#: A range this big or bigger cannot be expressed in millimetres in 16
#: bits. The Hokuyo used here tops out at 10m, so this is headroom, not a
#: limit anyone will meet.
_U16MM_MAX_M = 65.535


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
        # One signed byte per cell; see the module docstring on why the
        # browser is told the length rather than trusting the pairing.
        'bytes': int(info.width) * int(info.height),
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
    data = msg.data
    # rclpy hands OccupancyGrid.data over as array('b') -- one signed byte
    # per cell, already exactly this wire layout -- so on a little-endian
    # host .tobytes() is a straight buffer copy. The struct.pack path below
    # builds the same bytes out of individual Python ints, which for a
    # 2048x2048 map means unpacking 4.2 MILLION arguments into one call:
    # measured at 192ms on this car's Jetson against 4.8ms for .tobytes(),
    # a 40x difference, on a callback that fires every /map message.
    # Byte-for-byte identical either way -- test_map_cells_round_trips_*
    # passes unchanged, which is the proof.
    if _LITTLE_ENDIAN and getattr(data, 'typecode', None) == 'b':
        return data.tobytes()
    return struct.pack(f'<{len(data)}b', *data)


def scan_header(msg, laser_offset_x: float = 0.0, laser_offset_y: float = 0.0,
                encoding: str = SCAN_F32, decimation: int = 1) -> dict:
    """JSON-serializable metadata for a sensor_msgs/LaserScan.

    Includes the LIDAR's mounting offset from base_link so the browser can
    place scan points correctly relative to the car's own pose without a
    second topic or a TF lookup -- see docs/web-dashboard.md for why this
    node reads pose from a plain topic instead of TF.
    """
    decimation = max(1, int(decimation))
    count = len(range(0, len(msg.ranges), decimation))
    return {
        'type': 'scan',
        # Two or four bytes per beam, for the same reason map_header
        # carries it -- see the module docstring on why the browser is told
        # the length rather than trusting the pairing.
        'bytes': (2 if encoding == SCAN_U16MM else 4) * count,
        'encoding': encoding,
        'angle_min': float(msg.angle_min),
        # Scaled by the decimation: dropping every other beam doubles the
        # angle between the beams that remain, and the browser derives each
        # point's bearing from this. Getting it wrong would fan the scan out
        # across the wrong arc, which looks plausible and is completely
        # wrong -- the worst kind of display bug.
        'angle_increment': float(msg.angle_increment) * decimation,
        'range_min': float(msg.range_min),
        'range_max': float(msg.range_max),
        'count': count,
        'laser_offset_x': float(laser_offset_x),
        'laser_offset_y': float(laser_offset_y),
        'stamp': time.time(),
    }


def scan_ranges(msg) -> bytes:
    """Raw range floats, one little-endian float32 per beam -- matches
    JavaScript's Float32Array byte-for-byte."""
    ranges = msg.ranges
    # Same story as map_cells: rclpy delivers LaserScan.ranges as
    # array('f'), which is already the wire layout. 236x faster per scan.
    if _LITTLE_ENDIAN and getattr(ranges, 'typecode', None) == 'f':
        return ranges.tobytes()
    return struct.pack(f'<{len(ranges)}f', *ranges)


def scan_ranges_u16mm(msg, decimation: int = 1) -> bytes:
    """Ranges as little-endian uint16 millimetres -- half the bytes of
    scan_ranges(), and the largest steady saving on the link.

    Anything the browser would throw away anyway (inf, NaN, negative, or
    beyond what 16 bits of millimetres can hold) becomes 0, which is below
    every real scanner's range_min and so is already discarded by the
    drawing code. That keeps "no return" and "a return at 0mm"
    indistinguishable, which is correct: neither is a point to plot.
    """
    decimation = max(1, int(decimation))
    if _np is not None:
        values = _np.asarray(msg.ranges, dtype=_np.float32)
        if decimation > 1:
            values = values[::decimation]
        millimetres = values * 1000.0
        usable = _np.isfinite(values) & (values > 0.0) & (values <= _U16MM_MAX_M)
        quantised = _np.where(usable, _np.rint(millimetres), 0.0)
        return quantised.astype('<u2').tobytes()

    # No numpy: correctness matters more than speed on a path that should
    # never run in this workspace.
    out = []
    for index in range(0, len(msg.ranges), decimation):
        value = msg.ranges[index]
        if not math.isfinite(value) or value <= 0.0 or value > _U16MM_MAX_M:
            out.append(0)
        else:
            out.append(int(round(value * 1000.0)))
    return struct.pack(f'<{len(out)}H', *out)


def scan_payload(msg, encoding: str = SCAN_F32, decimation: int = 1) -> bytes:
    """The binary frame for a scan, in whichever encoding was asked for."""
    if encoding == SCAN_U16MM:
        return scan_ranges_u16mm(msg, decimation)
    if decimation > 1:
        return scan_ranges(_Decimated(msg.ranges, decimation))
    return scan_ranges(msg)


class _Decimated:
    """Just enough of a LaserScan for scan_ranges() to read a thinned copy."""

    __slots__ = ('ranges',)

    def __init__(self, ranges, decimation):
        self.ranges = list(ranges)[::decimation]


#: Below this, the commanded path and the desired path are the same line to
#: the eye -- well under one pixel at any zoom the dashboard uses.
INTENT_PATH_MERGE_TOLERANCE_M = 0.02


def thin_intent_payload(payload: dict,
                        tolerance_m: float = INTENT_PATH_MERGE_TOLERANCE_M) -> dict:
    """Drop `commanded_path` when it is indistinguishable from `path`.

    The two paths are most of an intent message's ~1.4kB, and at 18Hz that
    was 25 kB/s -- the second largest feed after the map. They only
    separate where slew-rate and acceleration shaping bend the command away
    from what the algorithm asked for, which is exactly when the dashed
    ghost line is worth drawing and exactly when this keeps it.

    Returns the payload unchanged (not a copy) when there is nothing to
    drop, so the common path allocates nothing. Never mutates its argument
    -- the caller's copy is the validated one.
    """
    wanted = payload.get('path')
    commanded = payload.get('commanded_path')
    if not wanted or not commanded or len(wanted) != len(commanded):
        return payload
    for a, b in zip(wanted, commanded):
        if (abs(a.get('x', 0.0) - b.get('x', 0.0)) > tolerance_m
                or abs(a.get('y', 0.0) - b.get('y', 0.0)) > tolerance_m):
            return payload
    thinned = dict(payload)
    # Empty rather than absent: the browser reads `intent.commanded_path ||
    # []`, so both work, but an explicit empty list says "there is no
    # separate commanded path" instead of "this field may be missing".
    thinned['commanded_path'] = []
    return thinned


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
    """The selected drive command from the mux output, as compact JSON."""
    return {
        'type': 'drive',
        'speed': float(speed),
        'steering_angle': float(steering_angle),
        'stamp': time.time(),
    }


def speed_message(speed: float) -> dict:
    """Measured longitudinal speed from odometry."""
    return {
        'type': 'speed',
        'speed': float(speed),
        'stamp': time.time(),
    }


def intent_message(payload: dict) -> dict:
    """Wrap one validated /drive_intent payload for the browser.

    Deliberately a pass-through rather than a re-encoding: the schema is
    owned by drive_intent/schema.py, which the driving nodes publish and
    the browser draws, and having this file paraphrase it would create a
    third definition to keep in sync with the other two. The nesting under
    'intent' keeps the dashboard's own envelope fields ('type', and the
    server-side receive 'stamp') from colliding with schema fields.

    The payload is validated by the caller *before* it gets here -- see
    DashboardNode.intent_callback -- so anything reaching this function is
    already known to be well-formed.
    """
    return {
        'type': 'intent',
        'intent': payload,
        # The server's own receive time, kept separate from payload['stamp']
        # (the car's). Two clocks, two fields: a browser on a laptop whose
        # clock disagrees with the Jetson's can still tell "this arrow is
        # stale" from the one it can trust.
        'stamp': time.time(),
    }


def stopwatch_message(elapsed_s: float, enabled: bool, running: bool,
                      lb_held: bool, joy_fresh: bool,
                      button_available: bool) -> dict:
    """Server-owned stopwatch state shared by every connected browser tab."""
    return {
        'type': 'stopwatch',
        'elapsed_s': float(elapsed_s),
        'enabled': bool(enabled),
        'running': bool(running),
        'lb_held': bool(lb_held),
        'joy_fresh': bool(joy_fresh),
        'button_available': bool(button_available),
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


def tuning_state_message(nodes, enabled: bool, allow_save: bool) -> dict:
    """The whole live-tuning picture: which driving nodes are up, what
    each will let you change, and what every knob currently reads.

    Sent whole rather than as per-parameter deltas. It is a few kilobytes
    at most, it only moves when a node appears/disappears or a value
    actually changes, and a browser that reconnects mid-session gets a
    complete, self-consistent panel from one message instead of
    reassembling one from a stream it partly missed.
    """
    return {
        'type': 'tuning',
        'enabled': bool(enabled),
        'allow_save': bool(allow_save),
        'nodes': nodes,
        'stamp': time.time(),
    }


def tuning_result_message(node: str, name: str, ok: bool, value=None,
                          reason: str = '') -> dict:
    """Outcome of one attempted change, echoed back to every tab.

    `value` is what the node actually holds afterwards -- not what was
    requested -- so a browser whose slider was refused snaps back to the
    truth rather than displaying a number the car never accepted.
    """
    return {
        'type': 'tuning_result',
        'node': str(node),
        'name': str(name),
        'ok': bool(ok),
        'value': value,
        'reason': str(reason),
        'stamp': time.time(),
    }


def tuning_saved_message(ok: bool, detail: str, files=()) -> dict:
    """Result of writing the current tune back into the package configs."""
    return {
        'type': 'tuning_saved',
        'ok': bool(ok),
        'detail': str(detail),
        'files': list(files),
        'stamp': time.time(),
    }


def tuning_armed_message(armed: bool) -> dict:
    """Per-connection arm state. Deliberately not shared between tabs:
    arming is a statement about the person holding *this* device, and a
    second tab inheriting it would be an arming nobody performed."""
    return {
        'type': 'tuning_armed',
        'armed': bool(armed),
        'stamp': time.time(),
    }


def process_state_message(targets, enabled: bool) -> dict:
    """Which driving processes are running, and which may be stopped.

    Carries the refused ones too, each with its reason. "pure_pursuit is
    up and you may not kill it from here, because it is the mux" is a
    far more useful thing to put in front of a person at trackside than
    a silently shorter list.
    """
    return {
        'type': 'processes',
        'enabled': bool(enabled),
        'targets': [t.as_dict() if hasattr(t, 'as_dict') else t
                    for t in targets],
        'stamp': time.time(),
    }


def process_result_message(pid: int, name: str, ok: bool, detail: str,
                           sent=(), done: bool = True) -> dict:
    """Progress of one stop request, echoed to every tab.

    `sent` is the escalation so far (['SIGINT', 'SIGTERM', ...]) because
    "this needed a SIGKILL" is exactly the diagnostic the person watching
    wants: a node that never answers Ctrl+C is a bug in that node, and
    hiding the escalation behind a green tick would hide the bug too.
    """
    return {
        'type': 'process_result',
        'pid': int(pid),
        'name': str(name),
        'ok': bool(ok),
        'done': bool(done),
        'detail': str(detail),
        'sent': list(sent),
        'stamp': time.time(),
    }
