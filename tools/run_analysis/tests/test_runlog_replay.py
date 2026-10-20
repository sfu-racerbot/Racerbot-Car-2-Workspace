"""Tests for tools.run_analysis.runlog and tools.run_analysis.replay.

Oracle notes: values computed directly from the definitions in the code.
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # insert tools/
from run_analysis import replay, runlog


def test_run_lb_windows():
    """oracle: lb_windows filters by min_duration=0.5; press held to end closes at last sample; 0.2s press dropped."""
    # joy rows (t, button) [(0,0),(1,1),(2,1),(3,0),(4,1),(4.2,0),(5,1),(6,1)]
    joy = np.array([[0, 0], [1, 1], [2, 1], [3, 0], [4, 1], [4.2, 0], [5, 1], [6, 1]], dtype=float)
    run = runlog.Run(
        path=None,
        scan_t=np.array([]),
        scan_stamp=np.array([]),
        scan=np.array([]).reshape(0, 0),
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.0,
        range_max=0.0,
        odom=np.zeros((0, 7)),
        drive=np.zeros((0, 4)),
        joy=joy,
    )
    windows = run.lb_windows(min_duration=0.5)
    # First press: 1 to 3, duration 2.0 OK -> (1,3)
    # Second: 4 to 4.2, duration 0.2 -> dropped by min_duration 0.5
    # Third: 5 to end (no release shown, so closes at last sample 6.1? joy last is (6,1), so closes at joy[-1,0]=6.0)
    # Third press starts at 5, no release after -> closes at last sample t=6.0
    assert windows == [(1.0, 3.0), (5.0, 6.0)]

    # Empty joy -> []
    run_empty = runlog.Run(
        path=None,
        scan_t=np.array([]),
        scan_stamp=np.array([]),
        scan=np.array([]).reshape(0, 0),
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.0,
        range_max=0.0,
        odom=np.zeros((0, 7)),
        drive=np.zeros((0, 4)),
        joy=np.zeros((0, 2)),
    )
    assert run_empty.lb_windows() == []


def test_run_odom_speed_at():
    """oracle: returns speed at exact stamp (counts as received); t=1.5 -> 1.0;
    after the last stamp -> the last speed; clamps to first."""
    odom = np.array([[0, 0.0, 0, 0, 0, 0, 1], [1, 1.0, 0, 0, 0, 0, 1], [2, 2.0, 0, 0, 0, 0, 1]], dtype=float)
    run = runlog.Run(
        path=None,
        scan_t=np.array([]),
        scan_stamp=np.array([]),
        scan=np.array([]).reshape(0, 0),
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.0,
        range_max=0.0,
        odom=odom,
        drive=np.zeros((0, 4)),
        joy=np.zeros((0, 2)),
    )
    assert run.odom_speed_at(1.0) == 1.0  # exact stamp counts as received
    # exactly at a stamp: searchsorted(stamps, t, side="right") - 1 selects
    # that stamp's own row, so the returned speed is that stamp's speed
    assert run.odom_speed_at(2.0) == 2.0
    assert run.odom_speed_at(1.5) == 1.0
    # after the last stamp: side="right" lands past the end, i = n-1, so the
    # last stamp's speed is returned (not an error, not a wrap to the first)
    assert run.odom_speed_at(2.5) == 2.0
    assert run.odom_speed_at(100.0) == 2.0
    assert run.odom_speed_at(-1.0) == 0.0  # clamps to first


def test_stop_clusters_boundary():
    """oracle: gap_s=0.4; 0.45-0.05 = 0.40 is NOT < 0.4 so it starts a new cluster."""
    # stopped at t = [0.0, 0.025, 0.05, 0.45, 0.851]
    # one not-stopped tick between -> clusters grouped with gap_s=0.4 strictly: [0,0.025,0.05] | [0.45] | [0.851]
    # gap between 0.05 and 0.45 is exactly 0.40; condition is k.t - clusters[-1][-1].t < gap_s, so 0.40 < 0.40 is false
    ticks = []
    for i, t in enumerate([0.0, 0.025, 0.05, 0.2, 0.45, 0.851]):  # t=0.2 is not stopped
        stopped = t in [0.0, 0.025, 0.05, 0.45, 0.851]
        ticks.append(
            replay.Tick(
                t=t,
                scan_index=0,
                odom_speed=0.0,
                steering=0.0,
                speed=0.0,
                real_steering=0.0,
                real_speed=0.0,
                stopped=stopped,
                stop_reason="",
                binding="",
                telemetry={},
            )
        )
    clusters = replay.stop_clusters(ticks, gap_s=0.4)
    assert len(clusters) == 3
    assert [c.t for c in clusters[0]] == [0.0, 0.025, 0.05]
    assert [c.t for c in clusters[1]] == [0.45]
    assert [c.t for c in clusters[2]] == [0.851]
    # boundary: 0.45 - 0.05 is exactly 0.40, and the condition
    # k.t - clusters[-1][-1].t < gap_s makes 0.40 < 0.40 false, so the t=0.45
    # tick starts a new cluster (asserted by the cluster lists above)

    # Empty input -> []
    assert replay.stop_clusters([], gap_s=0.4) == []


def test_parity_computation():
    """oracle: parity computes steering_rms, speed_median_abs from paired ticks; NaN real_speed excluded from paired."""
    ticks = []
    # tick 0: paired, steering diff, speed diff
    ticks.append(
        replay.Tick(
            t=0.0,
            scan_index=0,
            odom_speed=0.5,
            steering=0.1,
            speed=1.0,
            real_steering=0.05,
            real_speed=1.1,
            stopped=False,
            stop_reason="",
            binding="",
            telemetry={},
        )
    )
    # tick 1: paired
    ticks.append(
        replay.Tick(
            t=0.5,
            scan_index=1,
            odom_speed=0.5,
            steering=-0.1,
            speed=0.9,
            real_steering=-0.05,
            real_speed=1.0,
            stopped=False,
            stop_reason="",
            binding="",
            telemetry={},
        )
    )
    # tick 2: has NaN real_speed - excluded from paired
    ticks.append(
        replay.Tick(
            t=1.0,
            scan_index=2,
            odom_speed=0.5,
            steering=0.0,
            speed=1.0,
            real_steering=0.0,
            real_speed=float("nan"),
            stopped=False,
            stop_reason="",
            binding="",
            telemetry={},
        )
    )
    # tick 3: stopped in replay and real
    ticks.append(
        replay.Tick(
            t=1.5,
            scan_index=3,
            odom_speed=0.0,
            steering=0.0,
            speed=0.0,
            real_steering=0.0,
            real_speed=0.0,
            stopped=True,
            stop_reason="",
            binding="",
            telemetry={},
        )
    )
    # tick 4: stopped only in replay
    ticks.append(
        replay.Tick(
            t=2.0,
            scan_index=4,
            odom_speed=0.0,
            steering=0.0,
            speed=0.0,
            real_steering=0.1,
            real_speed=0.5,
            stopped=True,
            stop_reason="",
            binding="",
            telemetry={},
        )
    )
    # tick 5: stopped only in real
    ticks.append(
        replay.Tick(
            t=2.5,
            scan_index=5,
            odom_speed=0.3,
            steering=0.0,
            speed=0.3,
            real_steering=0.0,
            real_speed=0.0,
            stopped=False,  # replay says not stopped
            stop_reason="",
            binding="",
            telemetry={},
        )
    )

    p = replay.parity(ticks)
    # paired are ticks 0,1,3,4,5 (tick 2 has NaN real_speed) - 5 paired
    assert p.ticks == 6
    assert p.paired == 5
    # steering diffs: tick0 0.05, tick1 -0.05, tick3 0, tick4 -0.1, tick5 0
    s = np.array([0.05, -0.05, 0.0, -0.1, 0.0])
    steering_rms = float(np.sqrt(np.mean(s**2)))
    assert p.steering_rms == pytest.approx(steering_rms, abs=1e-12)  # rad, exact float identity
    assert p.steering_max == pytest.approx(0.1, abs=1e-12)  # rad, exact float identity
    # speed diffs: tick0 -0.1, tick1 -0.1, tick3 0, tick4 -0.5, tick5 0.3
    v = np.array([-0.1, -0.1, 0.0, -0.5, 0.3])
    assert p.speed_median_abs == pytest.approx(float(np.median(np.abs(v))), abs=1e-12)  # m/s, exact float identity
    # stops: replay has ticks 3,4 stopped (speed==0.0 in replay and they are stopped) - rep {1.5, 2.0}
    # real stopped where real_speed==0.0 -> tick3, tick5 give real_speed 0.0 -> real {1.5, 2.5}
    assert p.stops_replay == 2
    assert p.stops_real == 2
    assert p.stop_ticks_only_replay == [2.0]
    assert p.stop_ticks_only_real == [2.5]
