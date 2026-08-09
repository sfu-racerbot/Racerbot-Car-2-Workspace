"""
Unit tests for drive_intent.throttle.

These cover the two rules that keep a diagnostics feature from becoming a
driving problem: intent must not publish faster than it is configured to,
and broken intent code must disable itself rather than the node holding
the car's steering.

    cd ~/racerbot-ws
    python3 -m pytest src/drive_intent/test/test_throttle.py -v
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from drive_intent.throttle import FailureLatch, IntentThrottle  # noqa: E402


# ============================================================================
# IntentThrottle: publish rate
# ============================================================================

def test_the_first_call_always_publishes():
    assert IntentThrottle(20.0, 1.0).should_publish(100.0) is True


def test_publishing_is_capped_at_the_configured_rate():
    """gap_follow decides at scan rate (~40Hz) and pure_pursuit at
    control_rate_hz; no browser needs either, and the Jetson has better
    things to do."""
    throttle = IntentThrottle(20.0, 1.0)
    assert throttle.should_publish(0.0) is True
    assert throttle.should_publish(0.02) is False   # 50Hz tick, too soon
    assert throttle.should_publish(0.04) is False   # still inside 1/20s
    assert throttle.should_publish(0.051) is True


def test_a_refused_tick_does_not_reset_the_clock():
    throttle = IntentThrottle(10.0, 1.0)
    throttle.should_publish(0.0)
    for t in (0.02, 0.04, 0.06, 0.08):
        throttle.should_publish(t)
    assert throttle.should_publish(0.101) is True


def test_a_non_positive_rate_means_every_tick_not_silence():
    """A rate of 0 that silently published nothing would be a trap; the
    documented way to switch intent off is the publish_intent parameter."""
    throttle = IntentThrottle(0.0, 1.0)
    assert throttle.should_publish(0.0) is True
    assert throttle.should_publish(0.0) is True
    assert IntentThrottle(-5.0, 1.0).min_period == 0.0


# ============================================================================
# IntentThrottle: when to attach the (possibly expensive) reason string
# ============================================================================

def test_every_state_transition_carries_its_reason():
    """The transition is the diagnostic event -- it is exactly the moment
    someone is asking 'why did it just do that?'."""
    throttle = IntentThrottle(20.0, 1.0)
    assert throttle.wants_reason(0.0, 'gap_follow') is True
    assert throttle.wants_reason(0.01, 'ttc_brake') is True
    assert throttle.wants_reason(0.02, 'gap_follow') is True


def test_a_steady_state_repeats_its_reason_only_on_the_slow_period():
    throttle = IntentThrottle(20.0, 1.0)
    assert throttle.wants_reason(0.0, 'gap_follow') is True
    assert throttle.wants_reason(0.5, 'gap_follow') is False
    assert throttle.wants_reason(0.99, 'gap_follow') is False
    assert throttle.wants_reason(1.0, 'gap_follow') is True
    assert throttle.wants_reason(1.5, 'gap_follow') is False


def test_a_zero_reason_period_means_transitions_only():
    throttle = IntentThrottle(20.0, 0.0)
    assert throttle.wants_reason(0.0, 'gap_follow') is True
    assert throttle.wants_reason(100.0, 'gap_follow') is False
    assert throttle.wants_reason(100.1, 'no_safe_gap') is True


def test_reset_makes_the_next_tick_publish_and_explain():
    """A browser that just connected has no context at all; making it wait
    a second for the first reason is the wrong default."""
    throttle = IntentThrottle(20.0, 1.0)
    throttle.should_publish(0.0)
    throttle.wants_reason(0.0, 'gap_follow')
    throttle.reset()
    assert throttle.should_publish(0.001) is True
    assert throttle.wants_reason(0.001, 'gap_follow') is True


def test_publish_and_reason_clocks_are_independent():
    throttle = IntentThrottle(20.0, 1.0)
    throttle.wants_reason(0.0, 'gap_follow')
    for i in range(1, 10):
        throttle.should_publish(i * 0.06)
    assert throttle.wants_reason(0.9, 'gap_follow') is False


# ============================================================================
# FailureLatch: a broken drawing must not take down a driving node
# ============================================================================

def test_a_single_failure_is_tolerated():
    latch = FailureLatch(max_failures=5)
    assert latch.record_failure() is False
    assert latch.disabled is False


def test_sustained_failure_trips_the_latch_exactly_once():
    latch = FailureLatch(max_failures=3)
    assert latch.record_failure() is False
    assert latch.record_failure() is False
    assert latch.record_failure() is True      # the tripping call
    assert latch.disabled is True
    assert latch.record_failure() is False     # already off; do not re-log
    assert latch.disabled is True


def test_success_clears_the_count_so_one_bad_scan_never_accumulates():
    latch = FailureLatch(max_failures=3)
    latch.record_failure()
    latch.record_failure()
    latch.record_success()
    assert latch.record_failure() is False
    assert latch.record_failure() is False
    assert latch.disabled is False


def test_success_after_the_latch_trips_does_not_re_enable_it():
    """Once intent generation has proven broken for this configuration it
    stays off until the node restarts -- flapping diagnostics that
    re-break under load are worse than silent ones."""
    latch = FailureLatch(max_failures=2)
    latch.record_failure()
    latch.record_failure()
    latch.record_success()
    assert latch.disabled is True


@pytest.mark.parametrize('bad', [0, -1])
def test_a_latch_that_could_never_tolerate_anything_is_rejected(bad):
    with pytest.raises(ValueError):
        FailureLatch(max_failures=bad)
