"""
Unit tests for web_dashboard.mapstream -- the map keyframe/patch stream.

No ROS, no Tornado, no network, no browser: grids are plain `bytes`. Run
with:

    python3 -m pytest src/web_dashboard/test/test_mapstream.py -v

The test that matters most is
test_keyframe_then_patches_reconstruct_the_grid_exactly, which replays a
whole session through a model of what the browser does with these frames
and asserts the result is the source grid byte for byte. A patch applied
at the wrong offset still *looks* like a map, so an assertion on the
reconstructed bytes is the only honest check.
"""
import os
import sys
import zlib

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from web_dashboard import mapstream  # noqa: E402
from web_dashboard.mapstream import MapGeometry, MapStreamer  # noqa: E402


WIDTH, HEIGHT = 40, 30


def _geometry(width=WIDTH, height=HEIGHT, resolution=0.05,
              origin_x=-1.0, origin_y=-2.0, origin_yaw=0.0):
    return MapGeometry(width, height, resolution, origin_x, origin_y, origin_yaw)


def _blank(width=WIDTH, height=HEIGHT, value=0xFF):
    """A grid of `value` -- 0xFF is -1 as a signed byte, i.e. 'unknown'."""
    return bytes([value]) * (width * height)


def _with_rect(cells, x0, y0, w, h, value, width=WIDTH):
    """Paint a rectangle into a copy of `cells`, in grid coordinates."""
    grid = bytearray(cells)
    for y in range(y0, y0 + h):
        for x in range(x0, x0 + w):
            grid[y * width + x] = value
    return bytes(grid)


def _decode(header, payload):
    """What the browser does: undo the transport encoding."""
    assert len(payload) == header['bytes'], 'bytes must be the frame that follows'
    raw = zlib.decompress(payload) if header['encoding'] == 'deflate' else payload
    assert len(raw) == header['raw_bytes']
    return raw


class FakeBrowser:
    """A model of the browser's side of this protocol, used to prove that
    the frames actually reconstruct the map."""

    def __init__(self):
        self.cells = None
        self.geometry = None
        self.seq = None
        self.ignored_patches = 0

    def apply(self, frame):
        header, payload = frame
        raw = _decode(header, payload)
        if header['type'] == 'map':
            self.cells = bytearray(raw)
            self.geometry = (header['width'], header['height'])
            self.seq = header['seq']
            return
        # A patch is only safe to apply to the frame it was computed
        # against; anything else and we wait for the next keyframe.
        if self.seq is None or header['seq'] != self.seq + 1:
            self.ignored_patches += 1
            return
        width = self.geometry[0]
        x, y, w, h = header['x'], header['y'], header['w'], header['h']
        for row in range(h):
            start = (y + row) * width + x
            self.cells[start:start + w] = raw[row * w:(row + 1) * w]
        self.seq = header['seq']


# --------------------------------------------------------------------------
# Keyframes
# --------------------------------------------------------------------------

def test_first_grid_is_a_keyframe_carrying_the_whole_map():
    streamer = MapStreamer()
    cells = _blank()
    header, payload = streamer.update(cells, _geometry(), now=0.0)
    assert header['type'] == 'map'
    assert header['seq'] == 1
    assert header['width'] == WIDTH and header['height'] == HEIGHT
    assert _decode(header, payload) == cells


def test_keyframe_carries_the_geometry_the_browser_needs_to_place_it():
    streamer = MapStreamer()
    geometry = _geometry(resolution=0.05, origin_x=-8.25, origin_y=-6.1)
    header, _ = streamer.update(_blank(), geometry, now=0.0)
    assert header['resolution'] == pytest.approx(0.05)
    assert header['origin_x'] == pytest.approx(-8.25)
    assert header['origin_y'] == pytest.approx(-6.1)


def test_an_unchanged_grid_sends_nothing_at_all():
    streamer = MapStreamer()
    cells = _blank()
    streamer.update(cells, _geometry(), now=0.0)
    # slam_toolbox republishes on its own timer whether or not anything
    # moved; a parked car must not re-send the map to say nothing.
    assert streamer.update(cells, _geometry(), now=1.0) is None
    assert streamer.update(cells, _geometry(), now=2.0) is None


def test_growing_the_grid_forces_a_keyframe_not_a_patch():
    streamer = MapStreamer()
    streamer.update(_blank(), _geometry(), now=0.0)
    # slam_toolbox resizes its grid as the car explores. A patch into an
    # image of the wrong shape would be nonsense.
    bigger = _geometry(width=WIDTH + 10, height=HEIGHT + 6)
    header, payload = streamer.update(
        _blank(WIDTH + 10, HEIGHT + 6), bigger, now=1.0)
    assert header['type'] == 'map'
    assert header['width'] == WIDTH + 10


def test_moving_the_origin_forces_a_keyframe():
    streamer = MapStreamer()
    cells = _blank()
    streamer.update(cells, _geometry(), now=0.0)
    header, _ = streamer.update(cells, _geometry(origin_x=-3.0), now=1.0)
    assert header['type'] == 'map'


def test_a_keyframe_is_resent_periodically_so_a_client_cannot_be_wrong_forever():
    streamer = MapStreamer(keyframe_sec=30.0)
    cells = _blank()
    streamer.update(cells, _geometry(), now=0.0)
    changed = _with_rect(cells, 2, 2, 3, 3, 100)
    header, _ = streamer.update(changed, _geometry(), now=5.0)
    assert header['type'] == 'map_patch'
    changed2 = _with_rect(changed, 8, 8, 3, 3, 100)
    header, _ = streamer.update(changed2, _geometry(), now=40.0)
    assert header['type'] == 'map'


def test_keyframe_sec_zero_disables_the_periodic_refresh():
    streamer = MapStreamer(keyframe_sec=0.0)
    cells = _blank()
    streamer.update(cells, _geometry(), now=0.0)
    header, _ = streamer.update(_with_rect(cells, 1, 1, 2, 2, 100),
                                _geometry(), now=10_000.0)
    assert header['type'] == 'map_patch'


# --------------------------------------------------------------------------
# Patches
# --------------------------------------------------------------------------

def test_a_small_change_becomes_a_patch_of_just_that_rectangle():
    streamer = MapStreamer()
    cells = _blank()
    streamer.update(cells, _geometry(), now=0.0)
    header, payload = streamer.update(
        _with_rect(cells, 5, 7, 4, 3, 100), _geometry(), now=1.0)
    assert header['type'] == 'map_patch'
    assert (header['x'], header['y'], header['w'], header['h']) == (5, 7, 4, 3)
    assert header['raw_bytes'] == 4 * 3


def test_a_patch_is_dramatically_smaller_than_the_grid_it_updates():
    # The entire point of the exercise, asserted rather than assumed.
    streamer = MapStreamer()
    cells = _blank(200, 200)
    geometry = _geometry(200, 200)
    keyframe_header, _ = streamer.update(cells, geometry, now=0.0)
    patch_header, _ = streamer.update(
        _with_rect(cells, 20, 20, 10, 10, 100, width=200), geometry, now=1.0)
    assert patch_header['bytes'] < keyframe_header['bytes'] / 10


def test_a_change_covering_most_of_the_map_falls_back_to_a_keyframe():
    streamer = MapStreamer()
    cells = _blank()
    streamer.update(cells, _geometry(), now=0.0)
    # A loop closure re-rasterises nearly everything; a patch that large
    # costs more than starting over.
    header, _ = streamer.update(
        _with_rect(cells, 0, 0, WIDTH, HEIGHT - 1, 100), _geometry(), now=1.0)
    assert header['type'] == 'map'


def test_sequence_numbers_increase_by_one_per_frame():
    streamer = MapStreamer()
    cells = _blank()
    seqs = [streamer.update(cells, _geometry(), now=0.0)[0]['seq']]
    for i in range(1, 5):
        cells = _with_rect(cells, i, i, 2, 2, 100)
        seqs.append(streamer.update(cells, _geometry(), now=float(i))[0]['seq'])
    assert seqs == [1, 2, 3, 4, 5]


def test_keyframe_then_patches_reconstruct_the_grid_exactly():
    """The end-to-end proof: replay a mapping session and compare bytes.

    A patch applied at the wrong offset, or without accounting for row
    ordering, still produces something that looks like a map -- so the only
    check worth trusting is that the reconstruction is byte-identical.
    """
    streamer = MapStreamer()
    browser = FakeBrowser()
    geometry = _geometry()
    cells = _blank()

    frame = streamer.update(cells, geometry, now=0.0)
    browser.apply(frame)

    # Drive around: a sequence of small, scattered discoveries.
    for step, (x, y, value) in enumerate(
            [(3, 4, 0), (10, 2, 100), (20, 20, 0), (0, 0, 100),
             (WIDTH - 3, HEIGHT - 3, 100), (15, 9, 50)], start=1):
        cells = _with_rect(cells, x, y, 3, 2, value)
        frame = streamer.update(cells, geometry, now=float(step))
        assert frame is not None, 'a real change must produce a frame'
        browser.apply(frame)
        assert bytes(browser.cells) == cells, f'diverged at step {step}'

    assert browser.ignored_patches == 0


def test_a_patch_touching_the_last_row_and_column_is_placed_correctly():
    # Off-by-one in the half-open bounding box would show up here first.
    streamer = MapStreamer()
    browser = FakeBrowser()
    cells = _blank()
    browser.apply(streamer.update(cells, _geometry(), now=0.0))
    cells = _with_rect(cells, WIDTH - 1, HEIGHT - 1, 1, 1, 100)
    header, payload = streamer.update(cells, _geometry(), now=1.0)
    assert (header['x'], header['y'], header['w'], header['h']) == (
        WIDTH - 1, HEIGHT - 1, 1, 1)
    browser.apply((header, payload))
    assert bytes(browser.cells) == cells


def test_two_separate_changes_are_covered_by_one_bounding_box():
    streamer = MapStreamer()
    browser = FakeBrowser()
    cells = _blank()
    browser.apply(streamer.update(cells, _geometry(), now=0.0))
    cells = _with_rect(cells, 2, 2, 1, 1, 100)
    cells = _with_rect(cells, 9, 6, 1, 1, 100)
    header, payload = streamer.update(cells, _geometry(), now=1.0)
    assert (header['x'], header['y'], header['w'], header['h']) == (2, 2, 8, 5)
    browser.apply((header, payload))
    assert bytes(browser.cells) == cells


# --------------------------------------------------------------------------
# Late joiners
# --------------------------------------------------------------------------

def test_current_keyframe_reflects_patches_already_sent():
    """A tab that connects mid-session must get the map as it is *now*.

    Handing it the last keyframe would leave it permanently behind by
    every patch sent since.
    """
    streamer = MapStreamer()
    geometry = _geometry()
    cells = _blank()
    streamer.update(cells, geometry, now=0.0)
    cells = _with_rect(cells, 4, 4, 3, 3, 100)
    streamer.update(cells, geometry, now=1.0)

    header, payload = streamer.current_keyframe()
    assert header['type'] == 'map'
    assert _decode(header, payload) == cells


def test_a_late_joiner_is_in_sync_for_the_very_next_patch():
    streamer = MapStreamer()
    geometry = _geometry()
    cells = _blank()
    streamer.update(cells, geometry, now=0.0)
    cells = _with_rect(cells, 4, 4, 3, 3, 100)
    streamer.update(cells, geometry, now=1.0)

    latecomer = FakeBrowser()
    latecomer.apply(streamer.current_keyframe())

    cells = _with_rect(cells, 12, 12, 2, 2, 100)
    latecomer.apply(streamer.update(cells, geometry, now=2.0))
    assert bytes(latecomer.cells) == cells
    assert latecomer.ignored_patches == 0


def test_current_keyframe_is_none_before_any_map_arrives():
    assert MapStreamer().current_keyframe() is None


def test_a_browser_that_misses_a_patch_refuses_to_paint_a_wrong_map():
    streamer = MapStreamer()
    browser = FakeBrowser()
    geometry = _geometry()
    cells = _blank()
    browser.apply(streamer.update(cells, geometry, now=0.0))

    cells = _with_rect(cells, 3, 3, 2, 2, 100)
    streamer.update(cells, geometry, now=1.0)          # dropped in transit
    cells = _with_rect(cells, 9, 9, 2, 2, 100)
    browser.apply(streamer.update(cells, geometry, now=2.0))

    assert browser.ignored_patches == 1
    # It is behind, but it is not *wrong* -- and the next keyframe fixes it.
    browser.apply(streamer.current_keyframe())
    assert bytes(browser.cells) == cells


# --------------------------------------------------------------------------
# Encoding
# --------------------------------------------------------------------------

def test_compression_is_used_when_it_helps_and_declared_in_the_header():
    streamer = MapStreamer(compression=True)
    header, payload = streamer.update(_blank(200, 200), _geometry(200, 200), now=0.0)
    assert header['encoding'] == 'deflate'
    assert header['bytes'] == len(payload) < header['raw_bytes']
    assert zlib.decompress(payload) == _blank(200, 200)


def test_compression_can_be_turned_off_entirely():
    streamer = MapStreamer(compression=False)
    header, payload = streamer.update(_blank(), _geometry(), now=0.0)
    assert header['encoding'] == 'raw'
    assert payload == _blank()


def test_a_payload_that_would_grow_is_sent_raw_instead():
    # zlib's header is ~11 bytes, so a tiny incompressible patch can come
    # out bigger than it went in; paying browser decompression for that
    # would be silly.
    payload, encoding = mapstream._encode(b'\x01', compress=True, level=1)
    assert encoding == 'raw' and payload == b'\x01'


def test_bytes_always_describes_the_frame_that_actually_follows():
    # This is what makes the browser's desync check work; it must hold for
    # every frame type and both encodings.
    for compression in (True, False):
        streamer = MapStreamer(compression=compression)
        geometry = _geometry()
        cells = _blank()
        header, payload = streamer.update(cells, geometry, now=0.0)
        assert header['bytes'] == len(payload)
        cells = _with_rect(cells, 5, 5, 3, 3, 100)
        header, payload = streamer.update(cells, geometry, now=1.0)
        assert header['bytes'] == len(payload)


def test_a_grid_that_does_not_match_its_geometry_is_rejected_loudly():
    # Better a clear exception here than a reshape that silently succeeds
    # and paints garbage.
    streamer = MapStreamer()
    with pytest.raises(ValueError, match='geometry says'):
        streamer.update(b'\x00' * 10, _geometry(), now=0.0)


def test_patching_can_be_disabled_leaving_only_keyframes():
    streamer = MapStreamer(patching=False)
    geometry = _geometry()
    cells = _blank()
    streamer.update(cells, geometry, now=0.0)
    header, _ = streamer.update(_with_rect(cells, 1, 1, 2, 2, 100), geometry, now=1.0)
    assert header['type'] == 'map'


def test_unknown_cells_survive_the_round_trip_as_minus_one():
    # -1 (0xFF) is the value that plain bytes() cannot carry and that a
    # sloppy conversion turns into 255. It is also most of a fresh map.
    streamer = MapStreamer()
    cells = _blank(value=0xFF)
    header, payload = streamer.update(cells, _geometry(), now=0.0)
    raw = _decode(header, payload)
    assert set(raw) == {0xFF}
    assert all(int.from_bytes(bytes([b]), 'little', signed=True) == -1 for b in raw[:8])
