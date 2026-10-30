"""Closed-form turn oracles; collected by the package pytest runner."""
import math

import pytest

from pure_pursuit.auto_map_race_node import LapRecorder


def _circle(stride=1, gap=None, frozen=False, jump=False):
    recorder = LapRecorder(0.15, 1e9, 1e9, 0.0, 0.0, 1e9,
                           max_pose_jump=0.5)
    steps = 2000
    dt = 0.01
    for i in range(steps + 1):
        yaw = 2 * math.pi * i / steps
        missing = gap is not None and gap[0] < i <= gap[1]
        map_yaw = yaw
        if frozen and gap is not None:
            map_yaw -= 2 * math.pi * max(0, min(i, gap[1]) - gap[0]) / steps
        shifted = jump and i >= 810
        recorder.update(math.cos(yaw) + 10 * shifted, math.sin(yaw),
                        map_yaw + 0.7 * shifted, i * dt,
                        odom_yaw=yaw if stride and i % stride == 0 and not missing else None)
    return recorder


@pytest.mark.parametrize('stride', [1, 2, 10, 0])
def test_exact_circle_with_intermittent_or_missing_odom(stride):
    # oracle: closed form, turning-tangent theorem: one circle = 2*pi.
    assert math.degrees(_circle(stride).turn) == pytest.approx(360, abs=0.5)  # degrees


def test_short_frozen_map_gap_is_reconciled():
    # oracle: closed form, odometry describes exactly one circle.
    assert math.degrees(_circle(gap=(800, 820), frozen=True).turn) == pytest.approx(
        360, abs=0.5)  # degrees


def test_one_second_frozen_map_gap_exceeds_safe_bound():
    # oracle: closed form, long-gap fallback retains map turning only.
    expected = 360 * (1 - 100 / 2000)
    assert math.degrees(_circle(gap=(800, 900), frozen=True).turn) == pytest.approx(
        expected, abs=0.5)  # degrees


def test_long_gap_does_not_count_map_turning_twice():
    # oracle: closed form, map yaw itself traverses exactly one circle.
    assert math.degrees(_circle(gap=(800, 1000)).turn) == pytest.approx(
        360, abs=0.5)  # degrees


def test_gap_with_map_reanchor_is_reconciled():
    recorder = _circle(gap=(800, 820), jump=True)
    # oracle: invariant, a map-frame jump is not physical turning.
    assert recorder.reanchor_count == 1
    assert math.degrees(recorder.turn) == pytest.approx(360, abs=0.5)  # degrees


def test_reanchor_with_continuous_odom_preserves_physical_turn():
    # oracle: closed form, map correction cannot change the odometry circle.
    assert math.degrees(_circle(jump=True).turn) == pytest.approx(
        360, abs=0.5)  # degrees


def _anchored_recorder():
    recorder = LapRecorder(0.15, 1e9, 1e9, 0.0, 0.0, 1e9)
    recorder.update(0.0, 0.0, 0.0, 0.0, odom_yaw=0.0)
    return recorder


@pytest.mark.parametrize('return_time, expected', [
    pytest.param(0.25, 0.875, id='exact'),
    pytest.param(0.25 + 1 / 64, 0.5, id='one-tick-past'),
])
def test_gap_time_bound_at_and_one_tick_past(return_time, expected):
    recorder = _anchored_recorder()
    recorder.update(0.0, 0.0, 0.5, 0.125, odom_yaw=None)
    recorder.update(0.0, 0.0, 0.5, return_time, odom_yaw=0.875)
    # oracle: closed form, provisional map turn is 0.5 rad and odom turn
    # is 0.875 rad. At 0.25 s replace the former; one 1/64 s tick past
    # it retain only 0.5 rad. Binary-exact times avoid boundary roundoff.
    assert recorder.turn == pytest.approx(expected, abs=1e-12)  # radians


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('excess', [
    pytest.param(0.0, id='exact'),
    pytest.param(1 / 1024, id='just-past'),
])
def test_gap_provisional_turn_bound_at_and_just_past(direction, excess):
    recorder = _anchored_recorder()
    provisional = direction * (math.pi / 2 + excess)
    recorder.update(0.0, 0.0, provisional, 0.0625, odom_yaw=None)
    recorder.update(0.0, 0.0, provisional, 0.125, odom_yaw=direction * 0.75)
    # oracle: closed form, pi/2 is inclusive in either direction. Exactly
    # at it odom replaces the map estimate with +/-0.75 rad; any excess
    # leaves only the provisional map angle, with no extra odom credit.
    expected = direction * 0.75 if excess == 0.0 else provisional
    assert recorder.turn == pytest.approx(expected, abs=1e-12)  # radians


def test_short_gap_replaces_substantial_provisional_turning():
    recorder = _anchored_recorder()
    for step in range(1, 4):
        recorder.update(0.0, 0.0, step * 0.2, step / 32, odom_yaw=None)
    # oracle: closed form, three 0.2 rad map steps total 0.6 rad.
    assert recorder.turn == pytest.approx(3 * 0.2, abs=1e-12)  # radians
    recorder.update(0.0, 0.0, 0.8, 4 / 32, odom_yaw=1.0)
    # oracle: closed form, replace 0.6 with the full 1.0 rad odom turn:
    # 0.6 + (1.0 - 0.6) = 1.0, rather than adding another 1.0 rad.
    assert recorder.turn == pytest.approx(1.0, abs=1e-12)  # radians


@pytest.mark.parametrize('direction', [-1, 1])
def test_out_and_back_gap_retains_peak_bound_violation(direction):
    recorder = _anchored_recorder()
    recorder.update(0.0, 0.0, direction * 2.0, 0.0625, odom_yaw=None)
    recorder.update(0.0, 0.0, 0.0, 0.125, odom_yaw=None)
    recorder.update(0.0, 0.0, 0.0, 0.1875, odom_yaw=direction * 0.5)
    # oracle: closed form, map motion is +/-2 then -/+2 rad: zero net,
    # 4 rad absolute travel. The earlier peak exceeded pi/2, so returning
    # to zero cannot make reconciliation safe again: keep zero map turn.
    assert recorder.turn == pytest.approx(0.0, abs=1e-12)  # radians
