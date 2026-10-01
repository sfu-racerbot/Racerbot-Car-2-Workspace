"""
Tests for snapshot.py -- the result types and the layer ladder that makes
"not plugged in" print differently from "driver did not load".

Pure Python, no rclpy:  python3 -m pytest src/racerbot_doctor/test/ -v

Oracle for every test here: the ladder rule in
docs/superpowers/plans/2026-10-01-racerbot-doctor.md ("The layer ladder"):
evaluate bottom-up, a FAIL or INFO blocks every layer above it (SKIP,
naming the blocker), a WARN does not.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_doctor.snapshot import (  # noqa: E402
    DeviceReport, Layer, Status, TopicStats, ladder)


def _step(name, status, calls):
    def fn():
        calls.append(name)
        return Layer(name, status, f'{name} detail')
    return (name, fn)


def test_ladder_all_pass_evaluates_every_layer():
    calls = []
    rep = ladder('dev', [_step('a', Status.PASS, calls),
                         _step('b', Status.PASS, calls)])
    assert calls == ['a', 'b']
    assert [l.status for l in rep.layers] == [Status.PASS, Status.PASS]


def test_ladder_stops_at_first_fail_and_names_the_blocker():
    calls = []
    rep = ladder('dev', [_step('usb', Status.PASS, calls),
                         _step('kernel', Status.FAIL, calls),
                         _step('ros', Status.PASS, calls),
                         _step('data', Status.PASS, calls)])
    # Layers above the failure must not even be evaluated: their probes
    # could be meaningless or misleading once a lower layer is broken.
    assert calls == ['usb', 'kernel']
    assert [l.status for l in rep.layers] == [
        Status.PASS, Status.FAIL, Status.SKIP, Status.SKIP]
    assert [l.name for l in rep.layers] == ['usb', 'kernel', 'ros', 'data']
    assert 'kernel' in rep.layers[2].detail
    assert 'kernel' in rep.layers[3].detail


def test_ladder_warn_does_not_block():
    calls = []
    rep = ladder('dev', [_step('usb', Status.WARN, calls),
                         _step('ros', Status.PASS, calls)])
    assert calls == ['usb', 'ros']
    assert rep.layers[1].status is Status.PASS


def test_ladder_info_blocks_like_a_fail():
    calls = []
    rep = ladder('dev', [_step('proc', Status.INFO, calls),
                         _step('rate', Status.PASS, calls)])
    assert calls == ['proc']
    assert rep.layers[1].status is Status.SKIP


def test_worst_orders_fail_over_warn_over_info_over_pass():
    def rep(*statuses):
        return DeviceReport('d', [Layer('x', s, '') for s in statuses])
    assert rep(Status.PASS, Status.FAIL, Status.WARN).worst() is Status.FAIL
    assert rep(Status.PASS, Status.WARN, Status.INFO).worst() is Status.WARN
    assert rep(Status.PASS, Status.INFO, Status.SKIP).worst() is Status.INFO
    assert rep(Status.PASS, Status.PASS).worst() is Status.PASS
    assert rep().worst() is Status.PASS


def test_topic_stats_hz_is_count_over_window():
    assert TopicStats(count=120, window_s=3.0).hz == 40.0


def test_topic_stats_zero_window_is_zero_hz_not_a_crash():
    assert TopicStats(count=5, window_s=0.0).hz == 0.0
