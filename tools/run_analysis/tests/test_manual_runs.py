"""Manually driven runs: no /drive, commands on /ackermann_cmd.

joy_teleop publishes /teleop and ackermann_mux forwards the winner to
/ackermann_cmd, so a teleop run's bag has no /drive (CLAUDE.md,
Architecture). These tests hold the tools to their contract for such a run:
the ones that compare a driving node's /drive with the car refuse it with
runlog.NO_DRIVE; the ones that only need what the car was told use
/ackermann_cmd. Runs unsourced:

    python3 -m pytest tools/run_analysis/tests -q
"""
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from run_analysis import analyze_run, imu_report, replay, runlog  # noqa: E402

L = 0.36   # m, vesc.yaml wheelbase; actuation.WHEELBASE


def _run(drive=None, ackermann=None, joy=None, path=None):
    return runlog.Run(
        path=path, scan_t=np.array([]), scan_stamp=np.array([]),
        scan=np.zeros((0, 0)), angle_min=0.0, angle_increment=0.0, range_min=0.0,
        range_max=0.0, odom=np.zeros((0, 7)),
        drive=np.zeros((0, 4)) if drive is None else drive,
        joy=np.zeros((0, 2)) if joy is None else joy,
        ackermann=np.zeros((0, 4)) if ackermann is None else ackermann)


def _cmds(t, steer=0.1):
    return np.column_stack([t, t, np.full_like(t, steer), np.full_like(t, 1.0)])


# ---------------------------------------------------------------- Run.commands

def test_commands_prefer_drive_then_ackermann():
    # oracle: spec -- runlog.Run.commands docstring.
    t = np.arange(5.0)
    drive, ack = _cmds(t, 0.1), _cmds(t, -0.2)
    rows, topic = _run(drive=drive, ackermann=ack).commands()
    assert topic == '/drive' and rows is drive
    rows, topic = _run(ackermann=ack).commands()
    assert topic == '/ackermann_cmd' and rows is ack


def test_commands_with_neither_topic_raise():
    with pytest.raises(ValueError, match='no /drive or /ackermann_cmd'):
        _run().commands()


def _write_cache(run_dir, **arrays):
    base = dict(scan_t=np.array([0.0, 0.1]), scan_stamp=np.array([0.0, 0.1]),
                scan=np.ones((2, 3), np.float32), scan_meta=np.array([-1.0, 1.0, 0.05, 10.0]),
                odom=np.zeros((0, 7)), drive=np.zeros((0, 4)),
                joy=np.array([[0.0, 1.0], [5.0, 0.0]]))
    base.update(arrays)
    (run_dir / 'analysis').mkdir(parents=True)
    np.savez_compressed(run_dir / 'analysis' / runlog.CACHE_NAME, **base)


def test_load_run_reads_ackermann_and_tolerates_old_caches(tmp_path):
    # An old cache (no 'ackermann' array) loads with an empty one; a new one
    # round-trips its rows exactly.
    old, new = tmp_path / 'old', tmp_path / 'new'
    _write_cache(old)
    assert runlog.load_run(old).ackermann.shape == (0, 4)
    ack = _cmds(np.arange(3.0), 0.25)
    _write_cache(new, ackermann=ack)
    assert np.array_equal(runlog.load_run(new).ackermann, ack)


# ---------------------------------------------------------------- tools that need /drive

def test_analyze_run_refuses_a_manual_run_instead_of_reporting_its_lb_holds(tmp_path):
    # /joy shows LB held for 5 s (a person driving). Without /drive there is
    # nothing to replay: the report says so and has no "LB hold" section.
    _write_cache(tmp_path, ackermann=_cmds(np.arange(0.0, 5.0, 0.1)))
    text = analyze_run.analyze(tmp_path)
    assert runlog.NO_DRIVE in text
    assert '## LB hold' not in text
    assert (tmp_path / 'analysis' / 'report.md').read_text() == text


def test_replay_refuses_a_manual_run():
    run = _run(ackermann=_cmds(np.arange(0.0, 5.0, 0.1)))
    with pytest.raises(ValueError, match='driven manually'):
        replay.replay(run, 0.0, 5.0)


# ---------------------------------------------------------------- tools that use /ackermann_cmd

def _steering_run(t, lag, gain, commands_on):
    """odom at 1.2 m/s; the car turns at gain * v tan(delta(t - lag)) / L.
    Commands at every sample of a grid the lag falls on, so interpolating
    them adds no error of its own."""
    v = 1.2

    def delta(x):
        return 0.2 * np.sin(x)

    w = gain * v * np.tan(delta(t - lag)) / L
    yaw = np.concatenate([[0.0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(t))])
    odom = np.column_stack([t, np.full_like(t, v), w, t * 0, t * 0,
                            np.sin(yaw / 2), np.cos(yaw / 2)])
    cmds = np.column_stack([t, t, delta(t), np.full_like(t, v)])
    empty = np.zeros((0, 4))
    run = SimpleNamespace(odom=odom, drive=cmds if commands_on == '/drive' else empty,
                          ackermann=cmds if commands_on == '/ackermann_cmd' else empty)
    return run, np.column_stack([t, w])


def test_imu_report_fits_steering_from_ackermann_cmd_on_a_manual_run():
    # oracle: closed form -- the gyro is the command delayed 0.15 s and
    # scaled 0.95 (car 2's measured lag and gain, docs/run-analysis.md), so
    # the fit must recover exactly those, from /ackermann_cmd alone.
    t = np.arange(0.0, 30.0, 0.01)   # 0.15 s = 15 samples
    run, gyro = _steering_run(t, lag=0.15, gain=0.95, commands_on='/ackermann_cmd')
    text = '\n'.join(imu_report.build_report(run, {'bno': gyro, 'vesc': np.zeros((0, 2))}))
    assert 'commands from /ackermann_cmd' in text
    assert 'steering lag 0.150 s, curvature gain 0.950' in text


def test_imu_report_still_prefers_drive_when_both_exist():
    t = np.arange(0.0, 30.0, 0.01)   # 0.15 s = 15 samples
    run, gyro = _steering_run(t, lag=0.15, gain=0.95, commands_on='/drive')
    # A second, wrong command stream on /ackermann_cmd must be ignored.
    run.ackermann = run.drive.copy()
    run.ackermann[:, 2] = 0.0
    text = '\n'.join(imu_report.build_report(run, {'bno': gyro, 'vesc': np.zeros((0, 2))}))
    assert 'commands from /drive' in text
    assert 'steering lag 0.150 s, curvature gain 0.950' in text


def test_imu_report_with_no_commands_says_so():
    t = np.arange(0.0, 30.0, 0.01)   # 0.15 s = 15 samples
    run, gyro = _steering_run(t, lag=0.15, gain=0.95, commands_on='none')
    text = '\n'.join(imu_report.build_report(run, {'bno': gyro, 'vesc': np.zeros((0, 2))}))
    assert 'no /drive or /ackermann_cmd commands' in text
