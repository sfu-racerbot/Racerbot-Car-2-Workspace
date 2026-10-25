"""The safety wrapper (neural_racer/safety_wrapper.py). ROS-free.

Runner: python3 -m pytest src/neural_racer/test/test_safety_wrapper.py -v   (unsourced)

Oracles: gap_logic's own closed forms (braking_speed_limit), the curvature
cap's closed form v = sqrt(a_lat * L / tan(delta)), the steering-rate step
max_rate * dt, and the invariant that the wrapper never raises speed.
"""
import math

import nr_testutil
import numpy as np
import pytest

from gap_follow import gap_logic
from neural_racer.safety_wrapper import SafetyWrapper, WrapperConfig, scan_angles

ANGLES = scan_angles(nr_testutil.UST_ANGLE_MIN, nr_testutil.UST_INCREMENT, 1081)
PATH = np.zeros((6, 2))
RMIN, RMAX = 0.02, 30.0


def _apply(w, steer=0.0, speed=1.0, ranges=None, prev=0.0, dt=0.025, path=PATH, map_hit=None):
    r = nr_testutil.clear_scan() if ranges is None else ranges
    return w.apply(steer, speed, path, r, ANGLES, RMIN, RMAX, prev, dt, map_hit)


@pytest.fixture
def w():
    return SafetyWrapper(WrapperConfig(max_speed=4.0))


def test_open_space_passes_the_network_command(w):
    res = _apply(w, steer=0.0, speed=1.5)
    assert not res.stopped and res.speed == 1.5 and res.binding == 'network' and not res.intervened


@pytest.mark.parametrize('field', ['steer', 'speed'])
@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_non_finite_output_stops(w, field, bad):
    res = _apply(w, **{field: bad})
    assert res.stopped and res.speed == 0.0 and res.reason == 'nan_output'


def test_non_finite_forecast_stops(w):
    path = PATH.copy()
    path[3, 1] = math.nan
    res = _apply(w, path=path)
    assert res.stopped and res.reason == 'nan_output'


@pytest.mark.parametrize('n', [1080, 1082, 0])
def test_wrong_beam_count_stops(w, n):
    res = w.apply(0.0, 1.0, PATH, np.full(n, 5.0), np.zeros(n), RMIN, RMAX, 0.0, 0.025)
    assert res.stopped and res.reason == 'beam_count'


@pytest.mark.parametrize('fraction, stops', [(0.30, False), (0.31, True), (1.0, True)])
def test_invalid_fraction_boundary(w, fraction, stops):
    r = nr_testutil.clear_scan()
    r[:int(round(fraction * 1081))] = math.nan
    res = _apply(w, ranges=r)
    assert res.stopped == stops
    if stops:
        assert res.reason == 'invalid_beams' and res.speed == 0.0


def test_all_zero_scan_stops(w):
    res = _apply(w, ranges=np.zeros(1081))     # 0.0 is below range_min: every beam invalid
    assert res.stopped and res.reason == 'invalid_beams'


def test_braking_cap_matches_gap_logic(w):
    r = nr_testutil.clear_scan()
    r[530:551] = 1.2                             # a wall straight ahead
    res = _apply(w, speed=4.0, ranges=r)
    c = w.cfg
    valid = np.isfinite(r) & (r > RMIN) & (r < RMAX)
    clearance = gap_logic.minimum_footprint_clearance(r, valid, w.boundary(ANGLES))
    expected = gap_logic.braking_speed_limit(clearance, c.reserve_distance, c.max_braking_decel,
                                             c.max_speed, c.braking_latency_sec)
    assert res.binding == 'braking' and res.speed == pytest.approx(expected, abs=1e-12)
    assert res.speed < 4.0 and res.intervened


def test_obstacle_inside_reserve_gives_zero_speed(w):
    r = nr_testutil.clear_scan()
    r[530:551] = 0.35                            # 0.35 - 0.06 to the bumper: inside the 0.20 reserve
    res = _apply(w, speed=2.0, ranges=r)
    assert res.speed == 0.0 and not res.stopped and res.binding == 'braking'


def test_curvature_cap_closed_form(w):
    res = _apply(w, steer=0.33, speed=4.0, prev=0.33)
    expected = math.sqrt(2.0 * 0.36 / math.tan(0.33))
    assert res.binding == 'curvature' and res.speed == pytest.approx(expected, abs=1e-12)


def test_steering_rate_limited(w):
    res = _apply(w, steer=0.33, prev=0.0, dt=0.025)
    assert res.steering == pytest.approx(2.0 * 0.025, abs=1e-12)    # max_rate * dt
    assert res.intervened


def test_steering_rate_uses_capped_dt(w):
    res = _apply(w, steer=0.33, prev=0.0, dt=5.0)
    assert res.steering == pytest.approx(2.0 * 0.10, abs=1e-12)     # command_slew_max_dt


@pytest.mark.parametrize('steer', [0.5, -0.5, 10.0])
def test_steering_clipped(w, steer):
    # oracle: the configured cap (car 2: gap_follow.yaml's 0.349; car 1: 0.33)
    cap = w.cfg.max_steering_angle
    res = _apply(w, steer=steer, prev=math.copysign(cap, steer), dt=1.0)
    assert abs(res.steering) == pytest.approx(cap, abs=1e-12)


def test_speed_clipped_to_max_speed():
    w = SafetyWrapper(WrapperConfig(max_speed=1.0))
    assert _apply(w, speed=3.0).speed == 1.0


def test_negative_speed_clipped_to_zero(w):
    assert _apply(w, speed=-1.0).speed == 0.0


def test_map_hit_caps_speed(w):
    res = _apply(w, speed=4.0, map_hit=0.5)
    c = w.cfg
    expected = gap_logic.braking_speed_limit(0.5, c.reserve_distance, c.max_braking_decel,
                                             c.max_speed, c.braking_latency_sec)
    assert res.binding == 'map_path' and res.speed == pytest.approx(expected, abs=1e-12)


def test_never_raises_speed(w):
    """Invariant over random scans and commands."""
    rng = np.random.default_rng(0)
    for _ in range(300):
        r = rng.uniform(0.1, 12.0, 1081)
        speed = float(rng.uniform(0.0, 4.0))
        res = _apply(w, steer=float(rng.uniform(-0.33, 0.33)), speed=speed, ranges=r,
                     prev=float(rng.uniform(-0.33, 0.33)))
        assert res.speed <= speed + 1e-12


def test_bad_timing_stops(w):
    assert _apply(w, dt=math.nan).reason == 'bad_timing'
    assert _apply(w, prev=math.inf).reason == 'bad_timing'
