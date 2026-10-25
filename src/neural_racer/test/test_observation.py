"""Observation spec v1 (neural_racer/observation.py). ROS-free.

Runner: python3 -m pytest src/neural_racer/test/test_observation.py -v   (unsourced)
Oracle for every expected value: the spec table in
docs/superpowers/specs/2026-10-23-neural-racer-plan.md, section A4, or a
closed form recomputed here.
"""
import math

import nr_testutil  # noqa: F401  (sys.path)
import numpy as np
import pytest

from neural_racer import observation as obs


def _ranges(fill=8.0):
    return np.full(obs.N_BEAMS, fill, dtype=np.float64)


def _history(n_ticks=3, imu=True):
    h = obs.History()
    for k in range(n_ticks):
        enc, valid = obs.pool_scan(_ranges(1.0 + k), 0.02, 30.0)
        h.push_tick(enc, valid, speed=1.0 + k, imu=(0.1 * k, 0.2 * k, 0.3 * k) if imu else None)
        h.push_command(0.01 * (k + 1), 0.5 * (k + 1))
    return h


# --- pooling ------------------------------------------------------------------

def test_pool_keeps_minimum():
    r = _ranges()
    r[0:4] = [3.0, 0.4, 5.0, 5.0]
    enc, valid = obs.pool_scan(r, 0.02, 30.0)
    assert enc[0] == pytest.approx(math.log(0.4 / 0.05) / math.log(200.0), abs=1e-6)  # closed form
    assert valid[0] == 1.0


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf, 0.02, 0.01, 30.0, 31.0])
def test_invalid_beam_values(bad):
    """NaN, inf, at/below range_min and at/above range_max are not returns."""
    r = _ranges()
    r[4:8] = [bad, bad, bad, 2.0]
    enc, valid = obs.pool_scan(r, 0.02, 30.0)
    assert valid[1] == 1.0
    assert enc[1] == pytest.approx(math.log(2.0 / 0.05) / math.log(200.0), abs=1e-6)


def test_all_invalid_bin_reads_max_with_validity_zero():
    r = _ranges()
    r[8:12] = math.nan
    enc, valid = obs.pool_scan(r, 0.02, 30.0)
    assert valid[2] == 0.0
    assert enc[2] == pytest.approx(1.0, abs=1e-7)   # R_MAX encodes to exactly 1


def test_last_beam_is_dropped():
    r = _ranges()
    r[1080] = 0.1                       # beam 1080 is outside the 270 x 4 bins
    enc, _ = obs.pool_scan(r, 0.02, 30.0)
    assert enc.min() == pytest.approx(math.log(8.0 / 0.05) / math.log(200.0), abs=1e-6)


@pytest.mark.parametrize('n', [0, 1, 1080, 1082, 2162])
def test_beam_count(n):
    with pytest.raises(obs.ObservationError, match='beam count'):
        obs.pool_scan(np.full(n, 5.0), 0.02, 30.0)


def test_all_inf_scan_is_all_invalid():
    enc, valid = obs.pool_scan(np.full(obs.N_BEAMS, math.inf), 0.02, 30.0)
    assert valid.sum() == 0.0 and np.all(enc == 1.0)


@pytest.mark.parametrize('r, expected', [
    (0.05, 0.0), (10.0, 1.0), (20.0, 1.0), (0.0, 0.0), (0.01, 0.0),
    (1.0, math.log(20.0) / math.log(200.0)),
])
def test_encode_endpoints(r, expected):
    assert obs.encode_range(r) == pytest.approx(expected, abs=1e-12)


# --- history and the ego vector ----------------------------------------------------

def test_ego_layout_matches_spec():
    h = _history()
    o = obs.build_observation(h, imu_present=True, mode=obs.MODE_REACTIVE, v_cap=2.0)
    e = o['ego']
    # speeds pushed 1, 2, 3: index 0 is the newest (t)
    assert list(e[0:3] * 5.0) == pytest.approx([3.0, 2.0, 1.0], abs=1e-6)
    # commands pushed 0.01, 0.02, 0.03 (steer) and 0.5, 1.0, 1.5 (speed): t-1 first
    assert list(e[3:6] * 0.33) == pytest.approx([0.03, 0.02, 0.01], abs=1e-6)
    assert list(e[6:9] * 5.0) == pytest.approx([1.5, 1.0, 0.5], abs=1e-6)
    assert list(e[9:12] * 3.0) == pytest.approx([0.2, 0.1, 0.0], abs=1e-6)
    assert list(e[12:15] * 9.81) == pytest.approx([0.4, 0.2, 0.0], abs=1e-5)
    assert list(e[15:18] * 9.81) == pytest.approx([0.6, 0.3, 0.0], abs=1e-5)
    assert e[18] == 1.0 and e[19] == 0.0
    assert e[20] * 5.0 == pytest.approx(2.0, abs=1e-6)
    assert e[21] == 0.0 and e[22] == 0.0


def test_scan_block_order():
    h = _history()
    o = obs.build_observation(h, imu_present=True, mode=obs.MODE_REACTIVE, v_cap=1.0)
    # scans were 1 m, 2 m, 3 m everywhere: channel 0 is the newest (3 m)
    assert o['scan'][0, 0] == pytest.approx(obs.encode_range(3.0), abs=1e-6)
    assert o['scan'][2, 0] == pytest.approx(obs.encode_range(1.0), abs=1e-6)
    assert np.all(o['scan'][3] == 1.0)


def test_short_history_repeats_the_oldest():
    h = _history(n_ticks=1)
    o = obs.build_observation(h, imu_present=True, mode=obs.MODE_REACTIVE, v_cap=1.0)
    assert np.all(o['scan'][0] == o['scan'][1]) and np.all(o['scan'][1] == o['scan'][2])
    assert o['ego'][0] == o['ego'][1] == o['ego'][2]
    # commands start from zeros, not from the first pushed one
    assert o['ego'][4] == 0.0 and o['ego'][5] == 0.0


def test_imu_missing_zeroes_imu_slots():
    h = _history()
    o = obs.build_observation(h, imu_present=False, mode=obs.MODE_REACTIVE, v_cap=1.0)
    assert np.all(o['ego'][9:19] == 0.0)


def test_imu_present_with_gap_raises():
    h = _history()
    enc, valid = obs.pool_scan(_ranges(), 0.02, 30.0)
    h.push_tick(enc, valid, speed=1.0, imu=None)
    with pytest.raises(obs.ObservationError, match='gaps'):
        obs.build_observation(h, imu_present=True, mode=obs.MODE_REACTIVE, v_cap=1.0)


def test_reactive_mode_zeroes_map_blocks_even_if_given():
    h = _history()
    o = obs.build_observation(h, imu_present=True, mode=obs.MODE_REACTIVE, v_cap=1.0,
                              preview=np.ones((4, 32)), race=np.ones(6))
    assert np.all(o['preview'] == 0.0) and np.all(o['race'] == 0.0)


def test_map_mode_needs_its_blocks():
    with pytest.raises(obs.ObservationError, match='preview'):
        obs.build_observation(_history(), imu_present=True, mode=obs.MODE_MAP, v_cap=1.0)


def test_map_mode_copies_blocks():
    pv = np.arange(128, dtype=np.float32).reshape(4, 32) / 100
    o = obs.build_observation(_history(), imu_present=True, mode=obs.MODE_MAP, v_cap=1.0,
                              preview=pv, race=np.full(6, 0.5))
    assert np.array_equal(o['preview'], pv) and o['ego'][19] == 1.0


def test_camera_flags():
    o = obs.build_observation(_history(), imu_present=True, mode=obs.MODE_REACTIVE, v_cap=1.0,
                              camera=np.ones(64), camera_age=0.1)
    assert o['ego'][21] == 1.0 and o['ego'][22] == pytest.approx(0.5, abs=1e-6)
    o = obs.build_observation(_history(), imu_present=True, mode=obs.MODE_REACTIVE, v_cap=1.0,
                              camera=np.ones(64), camera_age=5.0)
    assert o['ego'][22] == 1.0                          # clipped


@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_non_finite_inputs_raise(bad):
    h = obs.History()
    enc, valid = obs.pool_scan(_ranges(), 0.02, 30.0)
    with pytest.raises(obs.ObservationError, match='speed'):
        h.push_tick(enc, valid, speed=bad)
    with pytest.raises(obs.ObservationError, match='IMU'):
        h.push_tick(enc, valid, speed=0.0, imu=(bad, 0.0, 0.0))
    with pytest.raises(obs.ObservationError, match='v_cap'):
        obs.build_observation(_history(), imu_present=True, mode=0, v_cap=bad)
    with pytest.raises(obs.ObservationError, match='not finite'):
        obs.build_observation(_history(), imu_present=True, mode=1, v_cap=1.0,
                              preview=np.full((4, 32), bad), race=np.zeros(6))


def test_empty_history_raises():
    with pytest.raises(obs.ObservationError, match='no scan'):
        obs.build_observation(obs.History(), imu_present=False, mode=0, v_cap=1.0)


def test_reset_clears_everything():
    h = _history()
    h.reset()
    assert not h.scans and h.valid is None and list(h.steer_cmds) == [0.0, 0.0, 0.0]


def test_race_context():
    r = obs.race_context(0.25, 2, 3.0, None)
    assert list(r) == pytest.approx([0.25, 0.5, 0.3, 0.0, 1.0, 0.0], abs=1e-6)
    assert obs.race_context(1.5, 1, 50.0, 0.0)[2] == 1.0   # gap clipped to 1
