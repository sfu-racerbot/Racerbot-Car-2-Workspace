#!/usr/bin/env python3
"""Render what the web dashboard is drawing, as a PNG, from the terminal.

The dashboard is the tool people actually judge a mapping run by ("the SLAM
map looked really glitchy"), and that judgement is hard to act on without
the picture. This connects to the same WebSocket a browser does, decodes
the same messages `web_dashboard/protocol.py` sends, and draws them the way
`web/dashboard.js` does: the occupancy grid as the background, the LiDAR
scan as points, the car as an arrow.

    tools/racerbot_sim/capture_dashboard.py --output /tmp/dashboard.png
    tools/racerbot_sim/capture_dashboard.py --seconds 240 --interval 60 \
        --output /tmp/run.png --report /tmp/run.json

It is also the dashboard's test instrument. `--seconds` with `--interval`
watches a whole run and writes a numbered frame per interval, and every
binary frame is checked against the length its header declared -- the same
check `web/dashboard.js` makes, and the one that turns "the map went to
garbage" into a named, counted failure. A run with `desyncs: 0` is a
positive statement about the visualisation, not just an absence of
complaints.

Read-only: it opens a WebSocket and listens. It never sends a control
message, so it is safe against the real car as well as the simulator.
"""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path
import sys
import time
import zlib

import numpy as np

try:
    from tornado.websocket import websocket_connect
except ImportError:  # pragma: no cover
    raise SystemExit('tornado is required (it already is, for web_dashboard)')

# Matching web/dashboard.js: unknown recedes into the background, free is a
# dark "track surface", occupied is bright. Same reading as the browser's.
UNKNOWN = (32, 34, 38)
FREE = (58, 64, 74)
OCCUPIED = (232, 236, 244)
SCAN = (90, 200, 250)
CAR = (255, 96, 96)


class Capture:
    def __init__(self):
        self.map_header = None
        self.map_cells = None
        self.scan_header = None
        self.scan_ranges = None
        self.pose = None
        self.drive = None
        self.speed = None
        self.intent_state = None
        self._pending = None
        # Map delta stream (see web_dashboard/mapstream.py): the grid is
        # sent once as a keyframe and thereafter as patches, so a healthy
        # mapping run is a handful of keyframes and many small patches.
        self.map_seq = None
        self.map_keyframes = 0
        self.map_patches = 0
        self.map_patches_ignored = 0
        # Everything below is the report: what arrived, and whether any of
        # it was wrong.
        self.counts = {}
        self.bytes = {}
        self.total_bytes = 0
        self.started_at = None
        self.desyncs = []
        self.map_shapes = []
        self.map_fits = []
        self.pose_track = []

    def _count(self, kind, size=0):
        self.counts[kind] = self.counts.get(kind, 0) + 1
        # Bytes actually on the wire, which is the number the whole
        # lightweight-dashboard exercise is about. Counted per message type
        # so it is obvious which feed is expensive rather than just that
        # something is.
        self.bytes[kind] = self.bytes.get(kind, 0) + size
        self.total_bytes += size

    def handle(self, message):
        if self.started_at is None:
            self.started_at = time.monotonic()
        if isinstance(message, (bytes, bytearray)):
            self._count('binary', len(message))
            pending = self._pending
            self._pending = None
            if pending is None:
                self.desyncs.append('binary frame with no header before it')
                return
            kind, header = pending
            # The same check web/dashboard.js makes. A header whose binary
            # never arrived leaves the browser's single pending slot
            # pointing at the wrong thing, and the next payload is decoded
            # as the previous type -- a 1081-beam scan read as occupancy
            # cells paints the map as garbage rather than failing.
            expected = header.get('bytes')
            if expected is None:
                expected = (header['width'] * header['height'] if kind == 'map'
                            else 4 * header['count'])
            if len(message) != expected:
                self.desyncs.append(
                    f'{kind} payload is {len(message)} bytes, header says {expected}')
                return
            try:
                raw = self._decode(header, bytes(message))
            except ValueError as exc:
                self.desyncs.append(f'{kind} payload: {exc}')
                return
            if kind == 'map':
                self._apply_map(header, raw)
            elif kind == 'map_patch':
                self._apply_map_patch(header, raw)
            elif kind == 'scan':
                self.scan_ranges = self._decode_ranges(header, raw)
                self.scan_header = header
            return

        payload = json.loads(message)
        kind = payload.get('type')
        self._count(kind, len(message))
        if kind == 'batch':
            # One frame carrying a tick's worth of compact telemetry -- see
            # web_dashboard/batching.py. Each item is exactly the message it
            # would have been on its own.
            for item in payload.get('items') or []:
                # Counted under its own type, but with no size of its own:
                # those bytes were already counted once as part of the batch
                # frame that carried it.
                self._count(item.get('type'), 0)
                self._dispatch(item)
            return
        self._dispatch(payload)

    def _dispatch(self, payload):
        """Apply one decoded JSON message, batched or standalone."""
        kind = payload.get('type')
        if kind in ('map', 'map_patch', 'scan'):
            if self._pending is not None:
                # Two headers in a row: the first one's binary never came.
                self.desyncs.append(
                    f"{self._pending[0]} header was not followed by its binary")
            self._pending = (kind, payload)
        elif kind == 'pose':
            self.pose = (payload['x'], payload['y'], payload['yaw'])
            self.pose_track.append((payload['x'], payload['y']))
        elif kind == 'drive':
            self.drive = (payload['speed'], payload['steering_angle'])
        elif kind == 'speed':
            self.speed = payload['speed']
        elif kind == 'intent':
            self.intent_state = (payload.get('intent') or {}).get('state')

    # -- decoding the wire format ------------------------------------------
    #
    # This is deliberately a second, independent implementation of what
    # web/dashboard.js does. Two implementations that agree on a recorded
    # run is a much stronger statement about the protocol than one
    # implementation checked against itself.

    @staticmethod
    def _decode(header, payload: bytes) -> bytes:
        """Undo the transport encoding and check the declared raw length."""
        encoding = header.get('encoding')
        raw = zlib.decompress(payload) if encoding == 'deflate' else payload
        expected = header.get('raw_bytes')
        if expected is not None and len(raw) != expected:
            raise ValueError(f'decoded to {len(raw)} bytes, header says {expected}')
        return raw

    @staticmethod
    def _decode_ranges(header, raw: bytes):
        """Scan ranges in metres, from whichever encoding arrived."""
        if header.get('encoding') == 'u16mm':
            return np.frombuffer(raw, dtype='<u2').astype(np.float64) / 1000.0
        return np.frombuffer(raw, dtype='<f4').astype(np.float64)

    def _apply_map(self, header, raw: bytes):
        """A keyframe: the whole grid."""
        self.map_keyframes += 1
        self.map_cells = np.frombuffer(raw, dtype=np.int8).copy()
        self.map_header = header
        self.map_seq = header.get('seq')
        shape = (header['width'], header['height'])
        if not self.map_shapes or self.map_shapes[-1] != shape:
            self.map_shapes.append(shape)
        # Everything web/dashboard.js's maybeAutoFit() reads, so the view it
        # would have computed can be replayed offline.
        self.map_fits.append((
            header['width'], header['height'], header['resolution'],
            header['origin_x'], header['origin_y']))

    def _apply_map_patch(self, header, raw: bytes):
        """A patch: just the rectangle of cells that changed.

        Grid coordinates throughout -- `map_cells` is held in the grid's own
        bottom-up row order and only flipped when a PNG is written, so no
        flip is needed here.
        """
        self.map_patches += 1
        if self.map_cells is None:
            return  # no keyframe yet
        seq = header.get('seq')
        if self.map_seq is None or seq != self.map_seq + 1:
            # Same rule as the browser: a patch out of sequence would leave
            # the map quietly wrong, so wait for the next keyframe instead.
            self.map_patches_ignored += 1
            self.desyncs.append(
                f'map patch {seq} does not follow frame {self.map_seq}')
            return
        width = self.map_header['width']
        x, y, w, h = header['x'], header['y'], header['w'], header['h']
        block = np.frombuffer(raw, dtype=np.int8).reshape(h, w)
        grid = self.map_cells.reshape(self.map_header['height'], width)
        grid[y:y + h, x:x + w] = block
        self.map_seq = seq

    @property
    def complete(self) -> bool:
        return self.map_cells is not None and self.pose is not None

    def render(self, path: str, scale: int = 4):
        from PIL import Image, ImageDraw

        if self.map_cells is None:
            raise SystemExit('no map was received -- is slam_toolbox publishing /map?')
        header = self.map_header
        width, height = header['width'], header['height']
        grid = self.map_cells.reshape(height, width)

        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[grid < 0] = UNKNOWN
        image[(grid >= 0) & (grid < 50)] = FREE
        image[grid >= 50] = OCCUPIED
        # Row 0 of an OccupancyGrid is the *smallest* world Y; a PNG's row 0
        # is the top. Flip, exactly as dashboard.js does when it builds its
        # offscreen canvas.
        picture = Image.fromarray(image[::-1], mode='RGB').resize(
            (width * scale, height * scale), Image.NEAREST)
        draw = ImageDraw.Draw(picture)

        resolution = header['resolution']
        origin_x, origin_y = header['origin_x'], header['origin_y']
        pixel_height = picture.height

        def to_pixel(x, y):
            return ((x - origin_x) / resolution * scale,
                    pixel_height - (y - origin_y) / resolution * scale)

        if self.scan_ranges is not None and self.pose is not None:
            scan = self.scan_header
            car_x, car_y, car_yaw = self.pose
            laser_x = car_x + scan['laser_offset_x'] * np.cos(car_yaw)
            laser_y = car_y + scan['laser_offset_x'] * np.sin(car_yaw)
            angles = (scan['angle_min']
                      + np.arange(len(self.scan_ranges)) * scan['angle_increment'])
            valid = np.isfinite(self.scan_ranges) & (self.scan_ranges > scan['range_min'])
            xs = laser_x + self.scan_ranges[valid] * np.cos(car_yaw + angles[valid])
            ys = laser_y + self.scan_ranges[valid] * np.sin(car_yaw + angles[valid])
            for x, y in zip(xs, ys):
                px, py = to_pixel(x, y)
                draw.ellipse([px - 1, py - 1, px + 1, py + 1], fill=SCAN)

        if self.pose is not None:
            car_x, car_y, car_yaw = self.pose
            nose = (car_x + 0.5 * np.cos(car_yaw), car_y + 0.5 * np.sin(car_yaw))
            draw.line([to_pixel(car_x, car_y), to_pixel(*nose)], fill=CAR, width=3)
            px, py = to_pixel(car_x, car_y)
            draw.ellipse([px - 4, py - 4, px + 4, py + 4], outline=CAR, width=2)

        picture.save(path)
        return {
            'map': f"{width}x{height} @ {resolution:.3f}m/px",
            'unknown_fraction': round(float((grid < 0).mean()), 3),
            'occupied_fraction': round(float((grid >= 50).mean()), 3),
            'scan_beams': None if self.scan_ranges is None else len(self.scan_ranges),
            'pose': None if self.pose is None else [round(v, 3) for v in self.pose],
            'drive': self.drive,
            'speed': self.speed,
            'intent_state': self.intent_state,
        }

    def _wire_report(self) -> dict:
        """What this actually cost the link.

        The point of the whole delta/batch protocol is a number, so measure
        it rather than assert it. Per message type, because "the dashboard
        uses N kB/s" is much less useful than knowing which feed is the N.
        """
        seconds = max(1e-6, time.monotonic() - (self.started_at or time.monotonic()))
        by_type = {
            kind: round(size / seconds / 1024, 2)
            for kind, size in sorted(self.bytes.items()) if size
        }
        return {
            'seconds': round(seconds, 1),
            'total_kb': round(self.total_bytes / 1024, 1),
            'total_kb_per_s': round(self.total_bytes / seconds / 1024, 2),
            'kb_per_s_by_type': by_type,
        }

    def report(self) -> dict:
        """What arrived over the whole session, and whether any of it was wrong."""
        travelled = 0.0
        for (x0, y0), (x1, y1) in zip(self.pose_track, self.pose_track[1:]):
            travelled += float(np.hypot(x1 - x0, y1 - y0))
        return {
            'messages': dict(sorted(self.counts.items())),
            'desyncs': len(self.desyncs),
            'desync_detail': self.desyncs[:10],
            'wire': self._wire_report(),
            'map_keyframes': self.map_keyframes,
            'map_patches': self.map_patches,
            # Non-zero means patches arrived out of order and the map went
            # stale until the next keyframe -- worth knowing about.
            'map_patches_ignored': self.map_patches_ignored,
            'map_sizes_seen': [f'{w}x{h}' for w, h in self.map_shapes],
            'pose_updates': len(self.pose_track),
            'pose_path_m': round(travelled, 2),
            'auto_fit': self.auto_fit_jitter(),
        }

    def auto_fit_jitter(self, canvas=(1200, 800)) -> dict:
        """Replay web/dashboard.js's view fitting over the maps that arrived.

        Both policies, from the same recorded sequence, so the comparison is
        on identical input:

        * `refit_every_map` -- what the dashboard used to do: re-derive
          centre and zoom from every map message. slam_toolbox resizes and
          re-origins its grid constantly as the map grows, shrinking as
          often as growing, so the whole picture moved under the viewer
          every few seconds while the map itself was fine.
        * `grow_to_fit` -- what it does now: frame the map once, then only
          re-fit when it no longer fits on screen.

        The canvas is assumed rather than known (this is not a browser).
        The conclusion is not sensitive to it: what changes is how many
        times the view moves at all.
        """
        if len(self.map_fits) < 2:
            return {'maps': len(self.map_fits)}
        canvas_width, canvas_height = canvas
        margin = 1.15

        def fit(width, height, resolution, origin_x, origin_y):
            min_x, min_y = origin_x, origin_y
            max_x = origin_x + width * resolution
            max_y = origin_y + height * resolution
            span = max(max_x - min_x, max_y - min_y)
            return ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0,
                    min(canvas_width, canvas_height) / (span * margin),
                    (min_x, max_x, min_y, max_y))

        # A "move" is anything the viewer would see: the picture sliding,
        # or rescaling, or both. Counting only the slide missed a step that
        # re-zoomed by a third without shifting the centre at all.
        def summarise(shifts, zooms):
            moves = sum(1 for shift, zoom in zip(shifts, zooms)
                        if shift > 1e-3 or zoom > 0.005)
            return {
                'view_moves': int(moves),
                'max_centre_shift_m': round(max(shifts), 3) if shifts else 0.0,
                'max_zoom_change_pct': round(100.0 * max(zooms), 2) if zooms else 0.0,
            }

        # Policy A: re-fit on every map.
        centres, scales = [], []
        for entry in self.map_fits:
            cx, cy, scale, _ = fit(*entry)
            centres.append((cx, cy))
            scales.append(scale)
        shifts = [float(np.hypot(b[0] - a[0], b[1] - a[1]))
                  for a, b in zip(centres, centres[1:])]
        zooms = [abs(b - a) / a for a, b in zip(scales, scales[1:])]
        every = summarise(shifts, zooms)

        # Policy B: frame once, then only when the map leaves the view.
        view_cx = view_cy = None
        view_scale = None
        grow_shifts, grow_zooms, grow_moves = [], [], 0
        for entry in self.map_fits:
            cx, cy, scale, (min_x, max_x, min_y, max_y) = fit(*entry)
            if view_scale is not None:
                half_w = canvas_width / (2.0 * view_scale)
                half_h = canvas_height / (2.0 * view_scale)
                visible = (min_x >= view_cx - half_w and max_x <= view_cx + half_w
                           and min_y >= view_cy - half_h and max_y <= view_cy + half_h)
                if visible:
                    continue
                grow_shifts.append(float(np.hypot(cx - view_cx, cy - view_cy)))
                grow_zooms.append(abs(scale - view_scale) / view_scale)
                grow_moves += 1
            view_cx, view_cy, view_scale = cx, cy, scale
        grow = summarise(grow_shifts, grow_zooms)
        grow['refits'] = grow_moves

        return {'maps': len(self.map_fits),
                'refit_every_map': every,
                'grow_to_fit': grow}


async def collect(url: str, seconds: float, interval: float = 0.0,
                  output: str = None, scale: int = 4) -> Capture:
    capture = Capture()
    connection = await websocket_connect(url)
    loop = asyncio.get_event_loop()
    deadline = loop.time() + seconds
    next_frame = loop.time() + interval if interval > 0.0 else None
    frame_index = 0
    while loop.time() < deadline:
        remaining = max(0.1, deadline - loop.time())
        if next_frame is not None:
            remaining = min(remaining, max(0.1, next_frame - loop.time()))
        try:
            message = await asyncio.wait_for(connection.read_message(), remaining)
        except asyncio.TimeoutError:
            message = None
            if loop.time() >= deadline:
                break
        if message is None and next_frame is None:
            break
        if message is not None:
            capture.handle(message)
        if next_frame is not None and loop.time() >= next_frame:
            next_frame += interval
            if capture.map_cells is not None and output:
                stem, _, extension = str(output).rpartition('.')
                path = f'{stem}-{frame_index:02d}.{extension}'
                try:
                    summary = capture.render(path, scale)
                    print(json.dumps({'frame': frame_index, **summary}), flush=True)
                except SystemExit:
                    pass
                frame_index += 1
    connection.close()
    return capture


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='ws://localhost:8080/ws')
    parser.add_argument('--seconds', type=float, default=8.0,
                        help='listen this long; /map is only republished every '
                             'map_update_interval (5s by default)')
    parser.add_argument('--output', default='dashboard.png')
    parser.add_argument('--scale', type=int, default=4)
    parser.add_argument('--interval', type=float, default=0.0,
                        help='also write a numbered frame this often, for '
                             'watching a whole run rather than one moment')
    parser.add_argument('--report', help='write the session report JSON here')
    args = parser.parse_args(argv)

    capture = asyncio.run(collect(args.url, args.seconds, args.interval,
                                  args.output, args.scale))
    summary = capture.render(args.output, args.scale)
    summary['output'] = args.output
    summary['session'] = capture.report()
    print(json.dumps(summary, indent=2))
    if args.report:
        Path(args.report).write_text(json.dumps(summary, indent=2) + '\n')
    # Non-zero when the visualisation was demonstrably wrong, so this can
    # gate a change the same way the sim validation does.
    return 1 if capture.report()['desyncs'] else 0


if __name__ == '__main__':
    sys.exit(main())
