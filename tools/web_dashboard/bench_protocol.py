#!/usr/bin/env python3
"""What the dashboard costs the car, as numbers you can re-run.

The dashboard was made much lighter by changing how it encodes and frames
what it sends. "Much lighter" is the kind of claim that rots quietly, so
this measures it instead: packing speed, bytes on the wire, and the cost
of WebSocket framing, all against the real dimensions this car works with
(the levine map's 2048x2048 grid, the Hokuyo's 1081 beams).

    python3 tools/web_dashboard/bench_protocol.py
    python3 tools/web_dashboard/bench_protocol.py --quick   # skip the socket test

No ROS, no browser, no car: everything is built from array.array and numpy
at the sizes the real thing uses. Exits non-zero if a result has regressed
past the thresholds at the bottom, so it can gate a change.
"""

from __future__ import annotations

import argparse
import array
import json
import os
import struct
import subprocess
import sys
import time
import zlib

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src', 'web_dashboard'))

from web_dashboard import protocol                      # noqa: E402
from web_dashboard.batching import TelemetryBatcher     # noqa: E402
from web_dashboard.mapstream import MapGeometry, MapStreamer  # noqa: E402

# The real thing, not a toy: levine.pgm is 2048x2048 at 0.05m/cell, and the
# Hokuyo UST-10LX returns 1081 beams.
MAP_W = MAP_H = 2048
SCAN_BEAMS = 1081

# Measured publish rates on this car, from `ros2 topic hz` during a run.
RATE_POSE = 40.0
RATE_DRIVE = 44.0
RATE_ODOM = 32.0
RATE_INTENT = 18.0
RATE_SCAN_BROADCAST = 10.0
RATE_STOPWATCH_BEFORE = 10.0
RATE_STOPWATCH_AFTER = 4.0
MAP_PERIOD_S = 5.0        # slam_toolbox's map_update_interval


class _Fake:
    """Just the fields protocol.py reads off a ROS message."""

    def __init__(self, **fields):
        self.__dict__.update(fields)


def _occupancy_grid():
    """A plausible mapped track: mostly unknown, a driven corridor, walls."""
    cells = np.full((MAP_H, MAP_W), -1, dtype=np.int8)
    cells[700:1400, 600:1500] = 0
    cells[700:1400, 600:610] = 100
    cells[700:1400, 1490:1500] = 100
    cells[700:710, 600:1500] = 100
    cells[1390:1400, 600:1500] = 100
    return cells


def _timed(fn, repeats=1):
    start = time.perf_counter()
    for _ in range(repeats):
        result = fn()
    return result, (time.perf_counter() - start) / repeats


def bench_packing(report):
    print('== packing the payloads ==')
    cells = _occupancy_grid()
    data = array.array('b')
    data.frombytes(cells.tobytes())
    grid = _Fake(data=data)

    def old_map():
        values = list(grid.data)
        return struct.pack(f'<{len(values)}b', *values)

    new_bytes, new_time = _timed(lambda: protocol.map_cells(grid))
    old_bytes, old_time = _timed(old_map)
    assert new_bytes == old_bytes, 'map_cells is no longer byte-identical!'
    print(f'  map {MAP_W}x{MAP_H}: struct.pack {old_time * 1e3:7.1f} ms  ->  '
          f'tobytes {new_time * 1e3:6.2f} ms   ({old_time / new_time:.0f}x, identical bytes)')
    report['map_pack_speedup'] = old_time / new_time

    ranges = array.array('f', [3.5] * SCAN_BEAMS)
    scan = _Fake(ranges=ranges, angle_min=-2.35, angle_increment=0.0058,
                 range_min=0.1, range_max=10.0)

    def old_scan():
        values = list(scan.ranges)
        return struct.pack(f'<{len(values)}f', *values)

    new_scan_bytes, new_scan_time = _timed(lambda: protocol.scan_ranges(scan), 200)
    old_scan_bytes, old_scan_time = _timed(old_scan, 200)
    assert new_scan_bytes == old_scan_bytes, 'scan_ranges is no longer byte-identical!'
    print(f'  scan {SCAN_BEAMS} beams: struct.pack {old_scan_time * 1e6:6.0f} us  ->  '
          f'tobytes {new_scan_time * 1e6:5.1f} us   ({old_scan_time / new_scan_time:.0f}x, identical bytes)')
    report['scan_pack_speedup'] = old_scan_time / new_scan_time
    return cells, scan


def bench_map_stream(cells, report):
    print()
    print('== the map on the wire ==')
    geometry = MapGeometry(MAP_W, MAP_H, 0.05, -51.2, -51.2, 0.0)
    streamer = MapStreamer()

    raw = cells.tobytes()
    print(f'  before: the whole grid, every {MAP_PERIOD_S:.0f}s, uncompressed'
          f'   {len(raw) / 1e6:6.2f} MB  = {len(raw) / MAP_PERIOD_S / 1024:8.1f} kB/s')
    report['map_before_kbs'] = len(raw) / MAP_PERIOD_S / 1024

    (header, payload), keyframe_time = _timed(
        lambda: streamer.update(raw, geometry, now=0.0))
    print(f'  keyframe (deflate, sent on connect and every 30s)'
          f'   {len(payload) / 1024:8.1f} kB  in {keyframe_time * 1e3:.0f} ms')
    report['keyframe_bytes'] = len(payload)

    # What actually happens while driving: a patch of ground that was
    # unknown a moment ago and has now been mapped. Deliberately outside
    # the corridor already carved out above, so it is a real change.
    moved = cells.copy()
    moved[1400:1600, 600:800] = 0
    moved[1590:1600, 600:800] = 100
    patched = moved.tobytes()
    frame, patch_time = _timed(lambda: streamer.update(patched, geometry, now=5.0))
    patch_header, patch_payload = frame
    assert patch_header['type'] == 'map_patch', patch_header['type']
    print(f'  patch ({patch_header["w"]}x{patch_header["h"]} cells changed)'
          f'                    {len(patch_payload):8d} B   in {patch_time * 1e3:.0f} ms')
    print(f'  after:  a patch every {MAP_PERIOD_S:.0f}s'
          f'                        {len(patch_payload) / MAP_PERIOD_S / 1024:14.2f} kB/s')
    report['patch_bytes'] = len(patch_payload)
    report['map_after_kbs'] = len(patch_payload) / MAP_PERIOD_S / 1024

    unchanged = streamer.update(patched, geometry, now=10.0)
    print(f'  a grid that did not change:                            '
          f'{"nothing sent" if unchanged is None else "RESENT!"}')
    report['silent_when_unchanged'] = unchanged is None


def _intent_message(points=16):
    path = [{'x': round(0.1 * i, 3), 'y': round(0.01 * i * i, 3), 'v': 2.5}
            for i in range(points)]
    return {
        'type': 'intent',
        'intent': {
            'v': 1, 'stamp': 0.0, 'node': 'pure_pursuit_node', 'frame': 'base_link',
            'state': 'racing', 'severity': 'drive', 'horizon_s': 1.5,
            'desired_steering': 0.12, 'commanded_steering': 0.1,
            'desired_speed': 3.0, 'commanded_speed': 2.8,
            'path': path, 'commanded_path': list(path),
            'factors': [
                {'name': 'corner speed', 'value': 3.0, 'unit': 'm/s', 'binding': True},
                {'name': 'max speed', 'value': 6.0, 'unit': 'm/s', 'binding': False},
                {'name': 'reactive floor', 'value': 4.2, 'unit': 'm/s', 'binding': False},
            ],
            'targets': [{'kind': 'steering_target', 'x': 1.2, 'y': 0.3}],
            'reason': 'corner speed limit binding: 3.0 m/s through the upcoming apex',
        },
        'stamp': 0.0,
    }


def _wire(message):
    return len(json.dumps(message).encode())


def bench_telemetry_mix(scan, report):
    print()
    print('== everything else, per second, per connected tab ==')
    # ~60 bytes of TCP/IP + WebSocket framing rides on every frame, and at
    # 155 frames a second that is not a rounding error.
    FRAMING = 60

    pose = protocol.pose_message(1.0, 2.0, 0.3)
    drive = protocol.drive_message(2.5, 0.1)
    speed = protocol.speed_message(2.4)
    stopwatch = protocol.stopwatch_message(12.3, True, True, True, True, True)
    stats = protocol.stats_message(30.0, 50.0, 51.0, 3600.0, -55.0)
    intent = _intent_message()

    scan_header_bytes = _wire(protocol.scan_header(scan, 0.33, 0.0))
    scan_f32 = len(protocol.scan_ranges(scan))
    scan_u16 = len(protocol.scan_payload(scan, protocol.SCAN_U16MM))

    before = (
        RATE_POSE * (_wire(pose) + FRAMING)
        + RATE_DRIVE * (_wire(drive) + FRAMING)
        + RATE_ODOM * (_wire(speed) + FRAMING)
        + RATE_INTENT * (_wire(intent) + FRAMING)
        + RATE_STOPWATCH_BEFORE * (_wire(stopwatch) + FRAMING)
        + RATE_SCAN_BROADCAST * (scan_header_bytes + scan_f32 + 2 * FRAMING)
        + 1.0 * (_wire(stats) + FRAMING)
    )

    # After: one batch frame at 20Hz carrying the newest of each, the scan
    # halved by u16mm, and the intent's redundant commanded_path dropped.
    thinned = dict(intent)
    thinned['intent'] = protocol.thin_intent_payload(intent['intent'])
    batch = {'type': 'batch', 'items': [pose, drive, speed, thinned, stopwatch]}
    after = (
        20.0 * (_wire(batch) + FRAMING)
        + RATE_SCAN_BROADCAST * (scan_header_bytes + scan_u16 + 2 * FRAMING)
    )

    print(f'  scan payload:  float32 {scan_f32} B  ->  u16mm {scan_u16} B')
    print(f'  intent:        {_wire(intent)} B  ->  {_wire(thinned)} B '
          f'(commanded_path dropped while it matches path)')
    print(f'  frames/s:      ~155  ->  ~{20 + RATE_SCAN_BROADCAST * 2:.0f}')
    print(f'  telemetry:     {before / 1024:7.1f} kB/s  ->  {after / 1024:6.1f} kB/s')
    report['telemetry_before_kbs'] = before / 1024
    report['telemetry_after_kbs'] = after / 1024
    return before / 1024, after / 1024


def bench_fanout(report):
    print()
    print('== WebSocket framing cost (server CPU) ==')
    # The client runs in a separate process on purpose: an in-process one
    # charges its own decode to this process's CPU time and roughly doubles
    # the apparent per-frame cost.
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '_bench_fanout_server.py')
    result = subprocess.run([sys.executable, script], capture_output=True, text=True,
                            timeout=180)
    if result.returncode != 0:
        print('  (skipped -- could not run the socket benchmark)')
        print(f'  {result.stderr.strip().splitlines()[-1] if result.stderr.strip() else ""}')
        return
    measured = json.loads(result.stdout.strip().splitlines()[-1])
    single = measured['single_us']
    batched = measured['batch_us']
    print(f'  one frame costs {single:.0f} us of server CPU, near enough '
          f'regardless of size')
    print(f'  155 separate frames/s   = {single * 155 / 1e4:5.2f}% of a core')
    print(f'  ~40 frames/s coalesced  = {batched * 40 / 1e4:5.2f}% of a core')
    report['frame_us'] = single
    report['fanout_before_pct'] = single * 155 / 1e4
    report['fanout_after_pct'] = batched * 40 / 1e4


def bench_batching(report):
    print()
    print('== batching keeps every intent transition ==')
    batcher = TelemetryBatcher()
    states = (['racing'] * 8) + ['emergency_stop'] + (['racing'] * 8)
    for state in states:
        batcher.add({'type': 'intent', 'intent': {'state': state}})
    batch = batcher.flush()
    seen = [item['intent']['state'] for item in batch['items']]
    print(f'  {len(states)} intent messages in one tick  ->  {len(seen)} sent: {seen}')
    report['transitions_preserved'] = seen == ['racing', 'emergency_stop', 'racing']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--quick', action='store_true',
                        help='skip the WebSocket framing benchmark')
    parser.add_argument('--json', metavar='PATH', help='also write the numbers here')
    args = parser.parse_args(argv)

    report = {}
    cells, scan = bench_packing(report)
    bench_map_stream(cells, report)
    telemetry_before, telemetry_after = bench_telemetry_mix(scan, report)
    bench_batching(report)
    if not args.quick:
        bench_fanout(report)

    total_before = report['map_before_kbs'] + telemetry_before
    total_after = report['map_after_kbs'] + telemetry_after
    print()
    print('== total, driving with SLAM mapping, per connected tab ==')
    print(f'  before  {total_before:8.1f} kB/s   ({total_before * 8 / 1024:5.2f} Mbit/s)')
    print(f'  after   {total_after:8.1f} kB/s   ({total_after * 8 / 1024:5.2f} Mbit/s)')
    print(f'  {total_before / total_after:.0f}x less')
    report['total_before_kbs'] = total_before
    report['total_after_kbs'] = total_after

    if args.json:
        with open(args.json, 'w') as handle:
            json.dump(report, handle, indent=2, sort_keys=True)

    # Thresholds, deliberately loose: they exist to catch a regression that
    # undoes the point of the exercise, not to pin exact numbers on a
    # machine whose load varies.
    problems = []
    if report['map_pack_speedup'] < 5:
        problems.append(f"map packing is only {report['map_pack_speedup']:.1f}x faster")
    if report['map_after_kbs'] > report['map_before_kbs'] / 20:
        problems.append('the map is no longer dramatically cheaper')
    if not report['silent_when_unchanged']:
        problems.append('an unchanged map is being re-sent')
    if not report['transitions_preserved']:
        problems.append('batching is dropping intent state transitions')
    if total_after > total_before / 5:
        problems.append('the overall saving has regressed')

    print()
    if problems:
        print('REGRESSED:')
        for problem in problems:
            print(f'  - {problem}')
        return 1
    print('All thresholds met.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
