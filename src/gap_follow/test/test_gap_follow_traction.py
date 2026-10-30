"""ROS-free traction contract; collected by the package pytest runner."""
import math

import pytest

from gap_follow import gap_logic


def _hold():
    return gap_logic.TractionHold(0.25, 0.125, 0.3, 0.5)


def test_engagement_is_strictly_above_threshold():
    hold = _hold()
    # oracle: spec I5: slip must exceed, not equal, the threshold.
    assert not hold.update(1.25, 1.0, 0.025)
    assert hold.update(math.nextafter(1.25, math.inf), 1.0, 0.025)


def test_max_hold_releases_on_exact_tick_and_requires_low_slip_to_rearm():
    hold = _hold()
    # oracle: closed form: four 0.125 s ticks reach the 0.5 s maximum.
    assert [hold.update(1.5, 1.0, 0.125) for _ in range(4)] == [True, True, True, False]
    assert not hold.update(1.5, 1.0, 0.125)
    assert not hold.update(1.125, 1.0, 0.125)
    assert not hold.update(1.5, 1.0, 0.125)
    assert not hold.update(1.0, 1.0, 0.125)
    assert hold.update(1.5, 1.0, 0.125)


def test_low_slip_releases_a_hold_immediately():
    hold = _hold()
    # oracle: spec I5 release hysteresis, strictly below release_slip.
    assert hold.update(1.5, 1.0, 0.025)
    assert hold.update(1.125, 1.0, 0.025)
    assert not hold.update(1.0, 1.0, 0.025)


@pytest.mark.parametrize('wheel,fused', [(0, 0), (0.2, 0), (2, 0.2),
                                         (0.2, 2), (-1, -2), (1, -2), (-1, 1)])
def test_no_judgement_at_standstill_or_reversing(wheel, fused):
    hold = _hold()
    # oracle: spec I5: either speed below min_speed precludes a hold.
    assert hold.update(1.5, 1.0, 0.025)
    assert not hold.update(wheel, fused, 0.025)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('index', range(3))
def test_nonfinite_input_releases_and_resets(bad, index):
    hold = _hold()
    hold.update(1.5, 1.0, 0.025)
    args = [1.5, 1.0, 0.025]
    args[index] = bad
    # oracle: spec I5: nonfinite inputs cannot hold or retain old state.
    assert not hold.update(*args)
    assert hold.update(1.5, 1.0, 0.025)


def test_reset_clears_expired_hold_lockout():
    hold = _hold()
    hold.update(1.5, 1.0, 0.5)
    hold.reset()
    # oracle: invariant: reset returns to the initially armed state.
    assert hold.update(1.5, 1.0, 0.025)


@pytest.mark.parametrize('desired,last,holding,expected', [
    (3.0, 2.0, True, 2.0), (1.0, 2.0, True, 1.0),
    (0.0, 2.0, True, 0.0), (3.0, 2.0, False, 3.0),
    (1.0, 2.0, False, 1.0)])
def test_traction_limited_speed_closed_forms(desired, last, holding, expected):
    # oracle: closed form: min(desired,last) on hold, otherwise desired.
    assert gap_logic.traction_limited_speed(desired, last, holding) == expected


@pytest.mark.parametrize('standstill_wheel', [0.0, 0.2])
def test_expired_hold_rearms_at_standstill_only_after_below_release_tick(standstill_wheel):
    hold = _hold()
    # oracle: spec task-I5-brief.md controller ruling: expiry locks out until
    # one below-release tick, including at standstill; low speed never holds.
    assert hold.update(1.5, 1.0, 0.25)
    assert not hold.update(1.5, 1.0, 0.25)
    assert not hold.update(standstill_wheel, 0.0, 0.025)
    if standstill_wheel > hold.release_slip:
        assert not hold.update(1.5, 1.0, 0.025)
        assert not hold.update(0.0, 0.0, 0.025)
    assert hold.update(1.5, 1.0, 0.025)
