"""
Unit tests for the bandwidth-saving encodings added to
web_dashboard.protocol: uint16-millimetre scans, beam decimation, and
intent path thinning.

Kept separate from test_protocol.py deliberately. That file's round-trip
tests are the proof that the `.tobytes()` fast paths are byte-identical to
the `struct.pack` implementation they replaced, and that proof is only
worth anything if the file itself is untouched.

    python3 -m pytest src/web_dashboard/test/test_protocol_encoding.py -v
"""
import math
import os
import struct
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from web_dashboard import protocol  # noqa: E402


def _fake_laser_scan(ranges, angle_min=-1.0, angle_increment=0.01):
    return SimpleNamespace(
        angle_min=angle_min, angle_increment=angle_increment,
        range_min=0.1, range_max=10.0, ranges=ranges,
    )


# --------------------------------------------------------------------------
# Scan encodings
# --------------------------------------------------------------------------

def test_scan_header_still_defaults_to_float32():
    # The default must not shift under anyone who has not opted in.
    header = protocol.scan_header(_fake_laser_scan([1.0, 2.0, 3.0]))
    assert header['encoding'] == protocol.SCAN_F32
    assert header['bytes'] == 4 * 3


def test_u16mm_header_declares_two_bytes_per_beam():
    msg = _fake_laser_scan([1.0, 2.0, 3.0])
    header = protocol.scan_header(msg, encoding=protocol.SCAN_U16MM)
    assert header['bytes'] == 2 * 3
    assert len(protocol.scan_payload(msg, protocol.SCAN_U16MM)) == header['bytes']


def test_u16mm_round_trips_to_millimetre_accuracy():
    ranges = [0.5, 1.2345, 9.999, 0.101]
    packed = protocol.scan_ranges_u16mm(_fake_laser_scan(ranges))
    recovered = [v / 1000.0 for v in struct.unpack(f'<{len(ranges)}H', packed)]
    for original, got in zip(ranges, recovered):
        assert got == pytest.approx(original, abs=0.001)


def test_u16mm_maps_every_unusable_reading_to_zero():
    # inf/NaN mean "nothing came back". 0 is below every real scanner's
    # range_min, so the browser discards it exactly as it discarded inf --
    # no new case to handle on the drawing side.
    ranges = [float('inf'), float('nan'), -1.0, 0.0, 1e9]
    values = struct.unpack(f'<{len(ranges)}H',
                           protocol.scan_ranges_u16mm(_fake_laser_scan(ranges)))
    assert values == (0, 0, 0, 0, 0)


def test_u16mm_is_exactly_half_the_size_of_float32():
    msg = _fake_laser_scan([1.0] * 1081)   # a real Hokuyo UST-10LX scan
    assert len(protocol.scan_ranges_u16mm(msg)) * 2 == len(protocol.scan_ranges(msg))


def test_u16mm_holds_the_full_range_of_this_car_s_lidar():
    # The Hokuyo tops out at 10m; 65.535m is the encoding's ceiling.
    ranges = [0.02, 10.0, 65.0]
    values = struct.unpack('<3H', protocol.scan_ranges_u16mm(_fake_laser_scan(ranges)))
    assert values == (20, 10000, 65000)


def test_decimation_thins_the_beams_and_widens_the_angle_step():
    # If angle_increment were left alone, the surviving beams would be
    # drawn across half the arc they actually cover: a plausible-looking
    # and completely wrong picture, which is the worst kind of display bug.
    msg = _fake_laser_scan([float(i) for i in range(10)], angle_increment=0.01)
    header = protocol.scan_header(msg, encoding=protocol.SCAN_U16MM, decimation=2)
    assert header['count'] == 5
    assert header['angle_increment'] == pytest.approx(0.02)
    assert header['angle_min'] == pytest.approx(msg.angle_min)
    assert len(protocol.scan_payload(msg, protocol.SCAN_U16MM, 2)) == header['bytes']


def test_decimation_keeps_the_first_beam_so_angle_min_stays_true():
    values = struct.unpack(
        '<3H', protocol.scan_ranges_u16mm(
            _fake_laser_scan([1.0, 2.0, 3.0, 4.0, 5.0]), decimation=2))
    assert values == (1000, 3000, 5000)


def test_decimation_also_applies_to_the_float32_encoding():
    msg = _fake_laser_scan([1.0, 2.0, 3.0, 4.0])
    header = protocol.scan_header(msg, decimation=2)
    payload = protocol.scan_payload(msg, protocol.SCAN_F32, 2)
    assert header['bytes'] == len(payload) == 8
    assert struct.unpack('<2f', payload) == (1.0, 3.0)


def test_decimation_of_one_changes_nothing():
    msg = _fake_laser_scan([1.0, 2.0, 3.0])
    assert protocol.scan_payload(msg, protocol.SCAN_F32, 1) == protocol.scan_ranges(msg)
    assert protocol.scan_header(msg, decimation=1)['angle_increment'] == pytest.approx(
        msg.angle_increment)


def test_odd_counts_decimate_without_losing_the_tail():
    msg = _fake_laser_scan([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
    header = protocol.scan_header(msg, encoding=protocol.SCAN_U16MM, decimation=3)
    payload = protocol.scan_payload(msg, protocol.SCAN_U16MM, 3)
    assert header['count'] == 3
    assert len(payload) == header['bytes']
    assert struct.unpack('<3H', payload) == (1000, 4000, 7000)


def test_the_numpy_and_pure_python_quantisers_agree():
    # protocol.py falls back to a struct implementation if numpy is
    # missing; the two must not disagree about edge cases.
    ranges = [0.5, 1.2345, float('inf'), float('nan'), 0.0, -2.0, 9.999, 65.0]
    msg = _fake_laser_scan(ranges)
    fast = protocol.scan_ranges_u16mm(msg)

    out = []
    for value in ranges:
        if not math.isfinite(value) or value <= 0.0 or value > 65.535:
            out.append(0)
        else:
            out.append(int(round(value * 1000.0)))
    assert fast == struct.pack(f'<{len(out)}H', *out)


# --------------------------------------------------------------------------
# Intent thinning
# --------------------------------------------------------------------------

def _path(points):
    return [{'x': x, 'y': y, 'v': 1.0} for x, y in points]


def test_an_identical_commanded_path_is_dropped():
    # The two paths are most of a 1.4kB intent message, sent at 18Hz.
    points = _path([(0.0, 0.0), (1.0, 0.1), (2.0, 0.3)])
    thinned = protocol.thin_intent_payload(
        {'path': points, 'commanded_path': list(points)})
    assert thinned['commanded_path'] == []
    assert thinned['path'] == points


def test_a_meaningfully_different_commanded_path_is_kept():
    # The gap between the two *is* the slew-rate and acceleration shaping,
    # which is the entire reason the dashed ghost line exists.
    payload = {
        'path': _path([(0.0, 0.0), (1.0, 0.0)]),
        'commanded_path': _path([(0.0, 0.0), (1.0, 0.5)]),
    }
    assert protocol.thin_intent_payload(payload)['commanded_path'] != []


def test_a_difference_far_below_a_pixel_counts_as_identical():
    payload = {
        'path': _path([(0.0, 0.0), (1.0, 0.0)]),
        'commanded_path': _path([(0.0, 0.0), (1.0, 0.001)]),
    }
    assert protocol.thin_intent_payload(payload)['commanded_path'] == []


def test_a_difference_in_only_the_x_axis_is_still_a_difference():
    payload = {
        'path': _path([(0.0, 0.0), (1.0, 0.0)]),
        'commanded_path': _path([(0.0, 0.0), (1.4, 0.0)]),
    }
    assert protocol.thin_intent_payload(payload)['commanded_path'] != []


def test_paths_of_different_lengths_are_left_alone():
    payload = {
        'path': _path([(0.0, 0.0), (1.0, 0.0)]),
        'commanded_path': _path([(0.0, 0.0)]),
    }
    assert len(protocol.thin_intent_payload(payload)['commanded_path']) == 1


def test_thinning_never_mutates_the_callers_payload():
    # The caller's copy is the validated one; corrupting it would be a
    # nasty action-at-a-distance bug.
    points = _path([(0.0, 0.0), (1.0, 0.0)])
    payload = {'path': points, 'commanded_path': list(points)}
    protocol.thin_intent_payload(payload)
    assert payload['commanded_path'] == points


def test_thinning_tolerates_a_missing_or_empty_path():
    for payload in ({}, {'path': []}, {'commanded_path': []},
                    {'path': _path([(0.0, 0.0)]), 'commanded_path': []}):
        assert protocol.thin_intent_payload(payload) is not None


def test_thinning_leaves_every_other_field_untouched():
    points = _path([(0.0, 0.0), (1.0, 0.0)])
    payload = {
        'path': points, 'commanded_path': list(points),
        'state': 'racing', 'severity': 'drive', 'commanded_speed': 2.5,
        'factors': [{'name': 'corner speed', 'binding': True}],
    }
    thinned = protocol.thin_intent_payload(payload)
    assert thinned['state'] == 'racing'
    assert thinned['commanded_speed'] == 2.5
    assert thinned['factors'] == payload['factors']
