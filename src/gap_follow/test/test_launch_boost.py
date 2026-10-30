"""
gap_follow's launch boost state machine (gap_follow/launch_boost.py).

Spec: the module docstring and docs/gap-follow.md#launch-boost -- the launch
starts on the first driving tick only (the node calls begin() there), never
again after it ends, and
ends at the first of: braking ceiling at or below the car's speed, steering
past end_steering (when it is not 0, which is off), max_sec, max_distance, or
any end() from the node.
Oracles: those rules, and elapsed/distance recomputed here as sum(dt) and
sum(dt * speed).

Pure Python, runs unsourced:

    python3 -m pytest src/gap_follow/test/test_launch_boost.py -v
"""
import math

import pytest

from gap_follow.launch_boost import ACTIVE, ARMED, DONE, LaunchBoost

DT = 0.025      # s, one 40 Hz scan
OPEN = 5.0      # m/s, a braking ceiling that is not binding


def boost(**kw):
    args = dict(enabled=True, max_sec=3.0, max_distance=5.0, end_steering=0.15)
    args.update(kw)
    return LaunchBoost(**args)


def test_armed_until_the_first_engaged_tick():
    b = boost()
    assert b.state == ARMED and not b.active
    assert b.begin() is True
    assert b.state == ACTIVE


def test_disabled_never_launches():
    b = boost(enabled=False)
    assert b.begin() is False
    assert b.state == DONE and b.end_reason == 'disabled'


def test_a_stop_before_the_first_press_does_not_spend_the_launch():
    # The node calls end() from every _stop, including the waiting-for-LB
    # stops on the grid before the race; those must not use up the launch.
    b = boost()
    b.end('deadman_released')
    b.end('waiting_for_joy')
    assert b.state == ARMED
    assert b.begin() is True


def test_release_and_press_again_never_launches_twice():
    b = boost()
    b.begin()
    b.end('deadman_released')
    assert b.state == DONE and b.end_reason == 'deadman_released'
    assert b.begin() is False
    assert b.step(DT, 0.0, 0.0, OPEN) is False


def test_step_before_begin_does_nothing():
    b = boost()
    assert b.step(DT, 1.0, 0.0, OPEN) is False
    assert b.state == ARMED and b.elapsed == 0.0 and b.distance == 0.0


def test_stays_active_on_an_open_straight():
    b = boost()
    b.begin()
    for _ in range(10):
        assert b.step(DT, 1.0, 0.0, OPEN) is True


def test_elapsed_and_distance_integrate_the_measured_speed():
    b = boost(max_sec=100.0, max_distance=100.0)
    b.begin()
    speeds = [0.0, 0.5, 1.0, 2.0, -0.3]   # a reverse reading adds nothing
    for v in speeds:
        b.step(DT, v, 0.0, OPEN)
    assert b.elapsed == pytest.approx(DT * len(speeds), abs=1e-12)  # s
    assert b.distance == pytest.approx(DT * (0.5 + 1.0 + 2.0), abs=1e-12)  # m


@pytest.mark.parametrize('cap, speed, ends', [
    (2.00, 1.99, False),   # ceiling just above the car: keep accelerating
    (2.00, 2.00, True),    # exactly at the boundary: the car must now slow
    (2.00, 2.01, True),    # one step past
    (0.00, 0.00, True),    # creeping on the grid (blocked ahead)
])
def test_ends_when_the_braking_ceiling_reaches_the_car(cap, speed, ends):
    b = boost()
    b.begin()
    assert b.step(DT, speed, 0.0, cap) is (not ends)
    if ends:
        assert b.end_reason == 'braking'


@pytest.mark.parametrize('steering, ends', [
    (0.15, False), (-0.15, False),        # exactly at the threshold: still straight
    (0.1501, True), (-0.1501, True),      # just past, either direction
])
def test_ends_when_the_car_turns(steering, ends):
    b = boost()
    b.begin()
    assert b.step(DT, 1.0, steering, OPEN) is (not ends)
    if ends:
        assert b.end_reason == 'turn'


@pytest.mark.parametrize('steering', [0.0, 0.33, -0.33, 1.0])
def test_end_steering_zero_never_ends_on_steering(steering):
    b = boost(end_steering=0.0)
    b.begin()
    assert b.step(DT, 1.0, steering, OPEN) is True


def test_ends_after_max_sec():
    b = boost(max_sec=0.1, max_distance=100.0)
    b.begin()
    results = [b.step(DT, 0.0, 0.0, OPEN) for _ in range(4)]
    # 0.025, 0.05, 0.075 active; 0.100 reaches max_sec.
    assert results == [True, True, True, False]
    assert b.end_reason == 'time'


def test_ends_after_max_distance():
    b = boost(max_sec=100.0, max_distance=0.1)
    b.begin()
    # 2 m/s x 0.025 s = 0.05 m a tick: the second tick reaches 0.1 m.
    assert b.step(DT, 2.0, 0.0, OPEN) is True
    assert b.step(DT, 2.0, 0.0, OPEN) is False
    assert b.end_reason == 'distance'


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('slot', range(4))
def test_non_finite_input_ends_the_launch(bad, slot):
    b = boost()
    b.begin()
    args = [DT, 1.0, 0.0, OPEN]
    args[slot] = bad
    assert b.step(*args) is False
    assert b.end_reason == 'invalid input'


def test_end_is_final():
    b = boost()
    b.begin()
    b.end('ttc_brake')
    b.end('deadman_released')
    assert b.end_reason == 'ttc_brake'


@pytest.mark.parametrize('name', ['max_sec', 'max_distance'])
@pytest.mark.parametrize('bad', [0.0, -1.0, math.nan, math.inf])
def test_refuses_bad_limits(name, bad):
    with pytest.raises(ValueError, match=f'launch_boost_{name}'):
        boost(**{name: bad})


@pytest.mark.parametrize('bad', [-0.01, math.nan, math.inf])
def test_refuses_a_bad_end_steering(bad):
    with pytest.raises(ValueError, match='launch_boost_end_steering'):
        boost(end_steering=bad)
