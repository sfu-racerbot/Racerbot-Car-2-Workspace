"""
Unit tests for summarize_run.py's 2026-08-25 additions:

  * "Unclassified stops" in render() -- the THROTTLED_PATTERNS 'stopped'
    catch-all already tallied every STOP line whose state CRITICAL_PATTERNS
    doesn't name, but render() never printed the count, so a stale/missing
    watchdog pattern (like the lap-progress regex found in the same
    session) failed silently instead of showing up as a nonzero number.
  * Falling back to node_logs.find_node_logs/merged_lines when launch.log
    is missing, instead of reporting nothing at all.

Pure filesystem + string handling, no rclpy:

    python3 -m pytest src/race_diagnostics/test/ -v
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import race_diagnostics.summarize_run as summarize_run_module  # noqa: E402
from race_diagnostics.summarize_run import analyze, render  # noqa: E402


def _run_dir(tmp_path, name='20260825-120000'):
    d = tmp_path / name
    d.mkdir()
    return d


def test_render_shows_zero_unclassified_stops_when_every_stop_is_named(tmp_path):
    run_dir = _run_dir(tmp_path)
    (run_dir / 'launch.log').write_text(
        '[WARN] [1.0] [gap_follow_node]: STOP [emergency_clearance] '
        'minimum body clearance -0.05m; command: steering=+0.000rad, '
        'speed=0.00m/s\n')
    report = render(analyze(run_dir))
    assert '## Unclassified stops' in report
    assert '0 STOP line(s)' in report


def test_render_surfaces_a_nonzero_unclassified_stop_count(tmp_path):
    """The exact safety net section 6.3 exists for: a STOP state
    CRITICAL_PATTERNS's watchdog alternation does not (yet, or no longer)
    recognize must show up as a nonzero count here instead of silently
    vanishing into the generic throttled 'stopped' bucket with nothing to
    say so -- the same kind of silent drift the lap-progress regex had."""
    run_dir = _run_dir(tmp_path)
    (run_dir / 'launch.log').write_text(
        '[WARN] [1.0] [gap_follow_node]: STOP [some_future_state_nobody_added_yet] '
        'detail; command: steering=+0.000rad, speed=0.00m/s\n')
    report = render(analyze(run_dir))
    assert '## Unclassified stops' in report
    assert '1 STOP line(s)' in report


def test_auto_map_race_node_bookkeeping_stops_do_not_count_as_unclassified(tmp_path):
    """A real, ordinary auto_map_race_launch.py run logs loading_profile
    and transition_hold on EVERY successful handover, plus routine
    gap_follow/pure_pursuit_command_missing/_stale lines while the
    selected controller's command hasn't arrived yet. None of these are a
    driving node's safety watchdog; if they fell into the generic
    unclassified bucket, every real run would show a permanent nonzero
    'Unclassified stops' count for reasons that have nothing to do with
    regex drift -- exactly the false-positive-noise failure mode
    discovered by testing this feature against a real historical run."""
    run_dir = _run_dir(tmp_path)
    (run_dir / 'launch.log').write_text(
        "[WARN] [1.0] [auto_map_race_node]: STOP [gap_follow_command_missing] "
        "no command has arrived on the selected 'gap_follow' input; output "
        "command: steering=+0.000rad, speed=0.00m/s\n"
        "[INFO] [2.0] [auto_map_race_node]: STOP [loading_profile] generated "
        "profile='/x/raceline_profiled.csv'; waiting for pure pursuit to "
        "accept it; output command: steering=+0.000rad, speed=0.00m/s\n"
        "[INFO] [3.0] [auto_map_race_node]: STOP [transition_hold] profile "
        "loaded; deliberate stop before racing has 1.99s remaining; output "
        "command: steering=+0.000rad, speed=0.00m/s\n")
    report = render(analyze(run_dir))
    assert '0 STOP line(s)' in report


def test_render_omits_the_section_when_there_is_nothing_to_classify(tmp_path, monkeypatch):
    run_dir = _run_dir(tmp_path)   # no launch.log written
    monkeypatch.setattr(summarize_run_module.node_logs, 'find_node_logs',
                        lambda start, end, **kw: {})
    monkeypatch.setattr(summarize_run_module.node_logs, 'merged_lines',
                        lambda found: [])
    report = render(analyze(run_dir))
    assert '## Unclassified stops' not in report


def test_analyze_falls_back_to_node_logs_when_launch_log_is_missing(tmp_path, monkeypatch):
    """No `| tee` this run -- analyze() must still find and classify the
    per-node ~/.ros/log/ files node_logs.find_node_logs would locate for
    this run's own time window, exactly as if launch.log had existed.
    find_node_logs/merged_lines are monkeypatched rather than given real
    files: this test is about analyze() correctly wiring in whatever
    node_logs returns, not about node_logs' own time-window matching,
    which test_node_logs.py covers directly."""
    run_dir = _run_dir(tmp_path)
    fake_found = {'gap_follow_node': ['<fake path marker>']}
    fake_lines = [
        '[WARN] [1.0] [gap_follow_node]: STOP [emergency_clearance] '
        'clearance -0.05m; command: steering=+0.000rad, speed=0.00m/s',
    ]
    monkeypatch.setattr(summarize_run_module.node_logs, 'find_node_logs',
                        lambda start, end, **kw: fake_found)
    monkeypatch.setattr(
        summarize_run_module.node_logs, 'merged_lines',
        lambda found: fake_lines if found is fake_found else [])

    result = analyze(run_dir)
    assert result['watchdogs'].get('emergency_clearance') == 1
    assert any('reconstructed' in note for note in result['notes'])


def test_analyze_prefers_launch_log_over_node_logs_when_both_could_apply(tmp_path, monkeypatch):
    """launch.log, when present, is authoritative -- node_logs must not
    even be consulted."""
    run_dir = _run_dir(tmp_path)
    (run_dir / 'launch.log').write_text(
        '[WARN] [1.0] [gap_follow_node]: STOP [scan_empty] '
        'LaserScan contains no range beams; command: steering=+0.000rad, '
        'speed=0.00m/s\n')

    def _must_not_be_called(*args, **kwargs):
        raise AssertionError('node_logs.find_node_logs should not run when launch.log exists')

    monkeypatch.setattr(summarize_run_module.node_logs, 'find_node_logs', _must_not_be_called)

    result = analyze(run_dir)
    assert result['watchdogs'].get('scan_empty') == 1
