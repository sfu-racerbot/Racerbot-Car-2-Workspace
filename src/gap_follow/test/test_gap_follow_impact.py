"""I6 state-machine contracts; collected by package pytest (no ROS needed).

Mutation evidence (2026-10-29): all 24 cases fail a logic stub; the strict
boundary and held-LB tests also reject >= and press-only reset, respectively.
"""
import math

import pytest

from gap_follow.gap_logic import ImpactLatch, horizontal_force


@pytest.mark.parametrize('ax,ay,expected', [(0, 0, 0), (3, 4, 5), (-5, -12, 13)])
def test_horizontal_force(ax, ay, expected):
    # oracle: closed form: Pythagorean triples, independent of signs.
    assert horizontal_force(ax, ay) == expected


@pytest.mark.parametrize('ax,ay', [(math.nan, 0), (0, math.inf), (-math.inf, 1)])
def test_nonfinite_force_is_nan(ax, ay):
    # oracle: spec I6: non-finite accelerations cannot be trusted.
    assert math.isnan(horizontal_force(ax, ay))


def test_threshold_is_strict():
    latch = ImpactLatch(5.0, 1)
    # oracle: closed form: hypot(3,4)=5; equality must not latch.
    assert not latch.on_sample(3, 4, True)
    assert latch.on_sample(math.nextafter(5.0, math.inf), 0, True)


def test_three_consecutive_samples_required():
    latch = ImpactLatch(5, 3)
    # oracle: spec I6: only consecutive strict exceedances count.
    assert not latch.on_sample(6, 0, True)
    assert not latch.on_sample(6, 0, True)
    assert not latch.on_sample(5, 0, True)
    assert not latch.on_sample(6, 0, True)
    assert not latch.on_sample(6, 0, True)
    assert latch.on_sample(6, 0, True)


@pytest.mark.parametrize('ax,trusted', [(6, False), (math.nan, True), (math.inf, True)])
def test_invalid_sample_resets_count_but_never_latch(ax, trusted):
    latch = ImpactLatch(5, 3)
    for _ in range(2):
        assert not latch.on_sample(6, 0, True)
    assert not latch.on_sample(ax, 0, trusted)
    for _ in range(2):
        assert not latch.on_sample(6, 0, True)
    assert latch.on_sample(6, 0, True)
    # oracle: invariant: invalid data can interrupt detection, never lift a stop.
    assert latch.on_sample(ax, 0, trusted)


def test_release_then_press_clears_and_rearms():
    latch = ImpactLatch(5, 1)
    # oracle: spec I6: releases before the spike do not authorize clearing.
    assert not latch.on_lb(False)
    assert latch.on_sample(6, 0, True)
    assert latch.on_lb(True)
    assert latch.on_lb(False)
    assert not latch.on_lb(True)
    assert latch.on_sample(6, 0, True)
    assert latch.on_lb(True)


def test_press_without_release_never_clears():
    latch = ImpactLatch(5, 1)
    assert latch.on_sample(6, 0, True)
    # oracle: invariant: a held LB is not a reset, regardless of elapsed samples.
    for _ in range(250):
        assert latch.on_lb(True)
        assert latch.on_sample(0, 0, True)


def test_repeated_release_clears_once():
    latch = ImpactLatch(5, 1)
    assert latch.on_sample(6, 0, True)
    # oracle: spec I6: release-release-press clears, without banking another reset.
    assert latch.on_lb(False)
    assert latch.on_lb(False)
    assert not latch.on_lb(True)
    assert latch.on_sample(6, 0, True)
    assert latch.on_lb(True)


def test_spike_while_latched_does_not_disarm_release():
    latch = ImpactLatch(5, 1)
    assert latch.on_sample(6, 0, True)
    assert latch.on_lb(False)
    # oracle: invariant: further spikes retain the same latch, not a new reset cycle.
    assert latch.on_sample(12, 0, True)
    assert not latch.on_lb(True)


def test_missing_button_ignored():
    latch = ImpactLatch(5, 1)
    assert latch.on_sample(6, 0, True)
    # oracle: spec I6: None counts as neither release nor press.
    assert latch.on_lb(None)
    assert latch.on_lb(True)
    assert latch.on_lb(False)
    assert latch.on_lb(None)
    assert not latch.on_lb(True)


@pytest.mark.parametrize('threshold,samples', [(0, 1), (-1, 1), (math.nan, 1),
                                             (math.inf, 1), (5, 0), (5, -1),
                                             (5, 1.5), (5, True)])
def test_invalid_configuration_refused(threshold, samples):
    # oracle: spec I6: finite positive threshold; integer sample count >= 1.
    with pytest.raises(ValueError, match='threshold|min_samples'):
        ImpactLatch(threshold, samples)
