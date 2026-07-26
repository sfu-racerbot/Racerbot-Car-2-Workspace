"""Unit tests for the ROS-free deadman-gated stopwatch."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from web_dashboard.stopwatch import DeadmanStopwatch  # noqa: E402


def test_disabled_stopwatch_does_not_run_with_lb_held():
    stopwatch = DeadmanStopwatch(joy_timeout_sec=0.5)
    stopwatch.update_joy([0, 0, 0, 0, 1], deadman_button=4, now=0.0)
    assert stopwatch.snapshot(0.4)['elapsed_s'] == pytest.approx(0.0)


def test_enabled_stopwatch_runs_only_while_lb_is_held():
    stopwatch = DeadmanStopwatch(joy_timeout_sec=0.5)
    stopwatch.set_enabled(True, now=0.0)
    stopwatch.update_joy([0, 0, 0, 0, 1], deadman_button=4, now=0.1)
    stopwatch.update_joy([0, 0, 0, 0, 1], deadman_button=4, now=0.3)
    stopwatch.update_joy([0, 0, 0, 0, 0], deadman_button=4, now=0.6)
    assert stopwatch.snapshot(1.0)['elapsed_s'] == pytest.approx(0.5)
    assert stopwatch.snapshot(1.0)['running'] is False


def test_stale_joy_stops_at_watchdog_deadline():
    stopwatch = DeadmanStopwatch(joy_timeout_sec=0.5)
    stopwatch.set_enabled(True, now=10.0)
    stopwatch.update_joy([0, 0, 0, 0, 1], deadman_button=4, now=10.0)
    snapshot = stopwatch.snapshot(12.0)
    assert snapshot['elapsed_s'] == pytest.approx(0.5)
    assert snapshot['joy_fresh'] is False
    assert snapshot['lb_held'] is False
    assert snapshot['running'] is False


def test_missing_lb_button_never_runs():
    stopwatch = DeadmanStopwatch(joy_timeout_sec=0.5)
    stopwatch.set_enabled(True, now=0.0)
    stopwatch.update_joy([1, 0], deadman_button=4, now=0.1)
    snapshot = stopwatch.snapshot(0.3)
    assert snapshot['button_available'] is False
    assert snapshot['elapsed_s'] == pytest.approx(0.0)


def test_reset_clears_elapsed_but_keeps_enabled_state():
    stopwatch = DeadmanStopwatch(joy_timeout_sec=0.5)
    stopwatch.set_enabled(True, now=0.0)
    stopwatch.update_joy([0, 0, 0, 0, 1], deadman_button=4, now=0.0)
    stopwatch.update_joy([0, 0, 0, 0, 1], deadman_button=4, now=0.3)
    stopwatch.reset(now=0.3)
    snapshot = stopwatch.snapshot(0.4)
    assert snapshot['elapsed_s'] == pytest.approx(0.1)
    assert snapshot['enabled'] is True
    assert snapshot['running'] is True
