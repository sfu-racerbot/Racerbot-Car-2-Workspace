"""Understeer math; collected by pytest src/gap_follow/test/ (no ROS needed)."""
import math

import pytest

from gap_follow import gap_logic as logic


@pytest.mark.parametrize('command', [-0.3, -0.05, 0.05, 0.3])
def test_ratio_is_the_tangent_ratio(command):
    # oracle: closed form, bicycle-model turn ratio, not angle ratio.
    assert logic.understeer_ratio(command / 2, command, 1.0) == pytest.approx(
        math.tan(command / 2) / math.tan(command), abs=1e-12)  # dimensionless


@pytest.mark.parametrize('index', range(3))
@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_nonfinite_ratio_inputs_cannot_be_judged(index, bad):
    args = [0.1, 0.2, 2.0]
    args[index] = bad
    # oracle: spec, I4 interface: every non-finite input means no judgement.
    assert math.isnan(logic.understeer_ratio(*args))


@pytest.mark.parametrize('command', [0.0, math.nextafter(0.05, 0.0),
                                     -math.nextafter(0.05, 0.0)])
def test_below_steering_floor_cannot_be_judged(command):
    # oracle: spec: |command| < 0.05, strictly; exact boundary tested above.
    assert math.isnan(logic.understeer_ratio(0.01, command, 2.0))


@pytest.mark.parametrize('speed', [-1.0, 0.0, math.nextafter(1.0, 0.0)])
def test_below_speed_floor_cannot_be_judged(speed):
    # oracle: spec: speed < 1, strictly; speed=1 tested in the closed form.
    assert math.isnan(logic.understeer_ratio(0.1, 0.2, speed))


@pytest.mark.parametrize('sign', [-1, 1])
def test_countersteering_boundary(sign):
    command = sign * 0.2
    # oracle: spec: opposite sign at |measured| >= 0.05 is not understeer.
    for magnitude in (0.05, math.nextafter(0.05, math.inf)):
        assert math.isnan(logic.understeer_ratio(-sign * magnitude, command, 2.0))
    small = -sign * math.nextafter(0.05, 0.0)
    assert logic.understeer_ratio(small, command, 2.0) == pytest.approx(
        math.tan(small) / math.tan(command), abs=1e-12)
    assert logic.understeer_ratio(0.0, command, 2.0) == 0.0


def test_engages_after_filtered_ratio_is_low_for_exactly_hold():
    check = logic.UndersteerCheck(0.75, 0.85, 0.2)
    dt, tau = 0.025, logic.GRIP_RATIO_TAU_SEC
    assert not check.update(1.0, dt)
    # oracle: closed form y_n=.6+.4 exp(-n dt/tau). Count low ticks
    # only once y_n < .75; eight such intervals cover the .2 s hold.
    first_low = math.floor(-tau * math.log((0.75 - 0.6) / 0.4) / dt) + 1
    engage_tick = first_low + math.ceil(0.2 / dt) - 1
    for tick in range(1, engage_tick + 2):
        assert check.update(0.6, dt) == (tick >= engage_tick)


def test_hysteresis_holds_both_states_then_releases():
    check = logic.UndersteerCheck(0.75, 0.85, 0.2)
    # oracle: spec: between the bands preserves either current state.
    for _ in range(30):
        assert not check.update(0.8, 0.025)
    for _ in range(60):
        check.update(0.6, 0.025)
    assert check.engaged
    for _ in range(60):
        assert check.update(0.8, 0.025)
    for _ in range(60):
        check.update(0.9, 0.025)
    assert not check.engaged


def test_missing_ratio_holds_until_longer_than_hold_then_resets():
    check = logic.UndersteerCheck(0.75, 0.85, 0.2)
    assert check.update(0.6, 0.2)
    # oracle: spec: short gap and exact hold preserve state; >hold resets.
    assert check.update(math.nan, 0.1)
    assert check.update(math.nan, 0.1)
    assert not check.update(math.nan, 0.001)
    assert not check.update(0.8, 0.025)
    assert check.update(0.6, 0.2)
    check.reset()
    assert not check.engaged
    assert not check.update(0.8, 0.025)


@pytest.mark.parametrize('speed', [-2.0, 0.0, 1.0, 3.0])
def test_cap_closed_form_and_off(speed):
    # oracle: closed form I4: max(0, measured speed * .85), or no ceiling.
    assert logic.understeer_speed_cap(True, speed, 0.85) == max(0.0, speed * 0.85)
    assert logic.understeer_speed_cap(False, speed, 0.85) == math.inf


@pytest.mark.parametrize('speed', [math.nan, math.inf, -math.inf])
def test_nonfinite_speed_has_no_cap(speed):
    # oracle: spec: an unknown speed must never set a driving ceiling.
    assert logic.understeer_speed_cap(True, speed, 0.85) == math.inf


def test_ratio_band_boundaries_are_strict():
    check = logic.UndersteerCheck(0.75, 0.85, 0.2)
    # oracle: spec: at engage is not below it; at release is not above it.
    assert not check.update(0.75, 0.2)
    check.reset()
    assert check.update(0.6, 0.2)
    check.ratio_filter.reset()
    assert check.update(0.85, 0.2)
    check.ratio_filter.reset()
    assert not check.update(math.nextafter(0.85, math.inf), 0.025)
