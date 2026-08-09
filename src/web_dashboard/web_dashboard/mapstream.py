"""
mapstream.py

Turns a stream of whole occupancy grids into a stream of *changes*.

The problem this solves is the single largest cost the dashboard imposed
on the car. `slam_toolbox` republishes its entire grid every
`map_update_interval` (5s by default) for as long as it is mapping --
which is exactly while somebody is driving the car around. At the levine
map's 2048x2048 that is 4MB per message, 819 kB/s sustained, per
connected browser, over the same WiFi link the car needs for everything
else. Measured on this car: the region that actually changed between two
such messages compresses to about 200 bytes. The car was sending four
megabytes to convey two hundred bytes of news.

So the browser is made the owner of the map image, and this sends it
deltas:

  keyframe (`map`)      the whole grid -- on first sight, whenever the
                        grid's geometry changes (slam_toolbox grows it as
                        the car explores), for a newly connected browser,
                        and periodically so a client can never be wrong
                        forever.
  patch (`map_patch`)   just the bounding rectangle of the cells that
                        actually changed since the last frame.
  nothing               the grid is byte-identical to the last one. A
                        parked car publishes this forever.

Both are zlib-compressed (level 1: measured 11ms for 74x on a full grid,
and an occupancy grid is mostly long runs of the same value, so it
compresses extremely well).

Every frame carries a sequence number. A browser applies a patch only if
it is exactly the successor of the frame it last applied; on any gap it
ignores patches until the next keyframe rather than painting a map that
is subtly, silently wrong. That is the same stance `protocol.py` takes
with its `bytes` field: a display that knows it is broken is worth far
more than one that quietly lies.

Deliberately free of ROS, Tornado and network code -- see
test/test_mapstream.py, which runs without a robot.
"""

import time
import zlib

try:
    import numpy as np
except ImportError:  # pragma: no cover - numpy ships with ROS2, but degrade
    np = None        # gracefully rather than taking the dashboard down


#: Above this fraction of the grid changing, a patch is not worth it -- the
#: rectangle covers most of the map anyway, and a keyframe resynchronises
#: every client at the same time.
MAX_PATCH_FRACTION = 0.5


class MapGeometry:
    """Where a grid sits in the world, and how big it is.

    Compared by value: any change here means the browser's cached image is
    the wrong shape or in the wrong place, so patches into it would be
    meaningless and a keyframe has to be sent.
    """

    __slots__ = ('width', 'height', 'resolution', 'origin_x', 'origin_y', 'origin_yaw')

    def __init__(self, width, height, resolution, origin_x, origin_y, origin_yaw=0.0):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.origin_yaw = float(origin_yaw)

    def _key(self):
        return (self.width, self.height, self.resolution,
                self.origin_x, self.origin_y, self.origin_yaw)

    def __eq__(self, other):
        return isinstance(other, MapGeometry) and self._key() == other._key()

    def __repr__(self):  # pragma: no cover - debugging aid
        return (f'MapGeometry({self.width}x{self.height} @ {self.resolution} '
                f'origin=({self.origin_x}, {self.origin_y}, {self.origin_yaw}))')


def _encode(payload: bytes, compress: bool, level: int):
    """(bytes, encoding) -- compressed only when that actually helps.

    A tiny patch can deflate to *more* than it started as (zlib's header is
    ~11 bytes), and there is no sense paying decompression cost in the
    browser for a payload that got bigger.
    """
    if not compress:
        return payload, 'raw'
    packed = zlib.compress(payload, level)
    if len(packed) >= len(payload):
        return payload, 'raw'
    return packed, 'deflate'


class MapStreamer:
    """Stateful: holds the last grid broadcast, so it can describe the next
    one as a difference from it.

    Owned by one thread (the rclpy executor thread in this dashboard).
    `current_keyframe()` is the one method a second thread calls, and it
    reads a tuple that is only ever replaced wholesale, never mutated.
    """

    def __init__(self, compression=True, patching=True, keyframe_sec=30.0,
                 compress_level=1, max_patch_fraction=MAX_PATCH_FRACTION):
        self.compression = bool(compression)
        # numpy is what makes diffing 4.2M cells cheap (measured 4.2ms);
        # without it, patching is simply off and every frame is a keyframe.
        self.patching = bool(patching) and np is not None
        self.keyframe_sec = float(keyframe_sec)
        self.compress_level = int(compress_level)
        self.max_patch_fraction = float(max_patch_fraction)

        self._seq = 0
        self._cells = None          # bytes of the last grid broadcast
        self._geometry = None       # its MapGeometry
        self._last_keyframe_time = None
        # (header, payload) for the current grid as a keyframe, built lazily
        # and replaced wholesale so another thread can read it safely.
        self._keyframe_frame = None

    # -- introspection, for logs and tests -------------------------------

    @property
    def seq(self):
        return self._seq

    @property
    def geometry(self):
        return self._geometry

    def has_map(self):
        return self._cells is not None

    # -- the main entry point --------------------------------------------

    def update(self, cells: bytes, geometry: MapGeometry, now=None):
        """Fold a newly received grid in.

        Returns `(header, payload)` for the frame to broadcast, or None if
        this grid says nothing new. `cells` must be row-major with one
        signed byte per cell, exactly as `nav_msgs/OccupancyGrid.data`
        arrives (row 0 = smallest world Y); the flip to image orientation
        is the browser's job, and patches are expressed in these same grid
        coordinates so that stays true of them too.
        """
        now = time.monotonic() if now is None else now
        expected = geometry.width * geometry.height
        if len(cells) != expected:
            raise ValueError(
                f'grid is {len(cells)} bytes, geometry says '
                f'{geometry.width}x{geometry.height}={expected}')

        keyframe_due = (
            self._cells is None
            or self._geometry != geometry
            or not self.patching
            or self._last_keyframe_time is None
            or (self.keyframe_sec > 0.0
                and now - self._last_keyframe_time >= self.keyframe_sec)
        )
        if keyframe_due:
            return self._make_keyframe(cells, geometry, now)

        if cells == self._cells:
            return None  # nothing changed -- say nothing

        box = self._dirty_box(self._cells, cells, geometry)
        if box is None:
            return None
        x0, y0, x1, y1 = box
        area = (x1 - x0) * (y1 - y0)
        if area >= self.max_patch_fraction * expected:
            # Most of the map moved (a loop closure re-rasterises
            # everything). A patch that large is worse than starting over.
            return self._make_keyframe(cells, geometry, now)
        return self._make_patch(cells, geometry, box, now)

    # -- frame builders ---------------------------------------------------

    def _make_keyframe(self, cells, geometry, now):
        self._seq += 1
        self._cells = cells
        self._geometry = geometry
        self._last_keyframe_time = now
        frame = self._build_keyframe(cells, geometry, self._seq)
        self._keyframe_frame = frame
        return frame

    def _build_keyframe(self, cells, geometry, seq):
        payload, encoding = _encode(cells, self.compression, self.compress_level)
        header = {
            'type': 'map',
            'seq': seq,
            'width': geometry.width,
            'height': geometry.height,
            'resolution': geometry.resolution,
            'origin_x': geometry.origin_x,
            'origin_y': geometry.origin_y,
            'origin_yaw': geometry.origin_yaw,
            'encoding': encoding,
            # `bytes` keeps the meaning it has always had: the exact length
            # of the binary frame that follows, so the browser's existing
            # "did I get the payload this header promised" check still
            # works. `raw_bytes` is what it should hold once inflated.
            'bytes': len(payload),
            'raw_bytes': len(cells),
            'stamp': time.time(),
        }
        return header, payload

    def _make_patch(self, cells, geometry, box, now):
        x0, y0, x1, y1 = box
        width = geometry.width
        rows = [cells[y * width + x0:y * width + x1] for y in range(y0, y1)]
        raw = b''.join(rows)
        payload, encoding = _encode(raw, self.compression, self.compress_level)

        self._seq += 1
        self._cells = cells
        self._geometry = geometry
        # A patch invalidates the cached keyframe: a browser connecting now
        # must be given the *current* grid, not the one from the last
        # keyframe, or it would be permanently one delta behind.
        self._keyframe_frame = None
        header = {
            'type': 'map_patch',
            'seq': self._seq,
            # Grid coordinates, same convention as OccupancyGrid.data:
            # x from the left, y from the row with the smallest world Y.
            'x': x0,
            'y': y0,
            'w': x1 - x0,
            'h': y1 - y0,
            'encoding': encoding,
            'bytes': len(payload),
            'raw_bytes': len(raw),
            'stamp': time.time(),
        }
        return header, payload

    def current_keyframe(self):
        """A full-grid frame for a browser that just connected.

        Carries the *current* sequence number rather than a new one, so the
        very next patch is this client's successor frame and it is in sync
        with every other tab immediately.
        """
        frame = self._keyframe_frame
        if frame is not None:
            return frame
        if self._cells is None:
            return None
        frame = self._build_keyframe(self._cells, self._geometry, self._seq)
        self._keyframe_frame = frame
        return frame

    # -- the diff ---------------------------------------------------------

    def _dirty_box(self, old: bytes, new: bytes, geometry: MapGeometry):
        """(x0, y0, x1, y1) half-open bounding box of changed cells, or None.

        numpy rather than a Python loop: this runs over millions of cells,
        and measured at 2048x2048 it costs about 4ms.
        """
        height, width = geometry.height, geometry.width
        before = np.frombuffer(old, dtype=np.int8).reshape(height, width)
        after = np.frombuffer(new, dtype=np.int8).reshape(height, width)
        changed = before != after
        rows = np.flatnonzero(changed.any(axis=1))
        if rows.size == 0:
            return None
        cols = np.flatnonzero(changed.any(axis=0))
        return int(cols[0]), int(rows[0]), int(cols[-1]) + 1, int(rows[-1]) + 1
