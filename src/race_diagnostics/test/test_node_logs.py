"""
Unit tests for node_logs.py -- locating and merging ROS's own per-node
~/.ros/log/python3_<PID>_<epoch_ms>.log files, so a run can be diagnosed
even when nobody remembered `| tee`.

Pure filesystem + string handling, no rclpy, so this runs with a bare
pytest and no ROS sourced at all:

    python3 -m pytest src/race_diagnostics/test/ -v

All fixtures here are synthetic (built with tmp_path), not real captures --
unlike test_run_events.py, the point under test is file discovery and
ordering mechanics, not log-line content, and the real ~/.ros/log/
directory on any given machine is both huge (~19k files) and outside this
test's control, so tests never touch it -- everything passes an explicit
`log_dir=tmp_path`.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from race_diagnostics.node_logs import find_node_logs, merged_lines  # noqa: E402


def _write(log_dir, pid, epoch_ms, lines):
    path = log_dir / f'python3_{pid}_{epoch_ms}.log'
    path.write_text('\n'.join(lines) + ('\n' if lines else ''))
    return path


def test_finds_only_files_inside_the_run_window(tmp_path):
    _write(tmp_path, 111, 1000000,
           ['[INFO] [1000.000000000] [gap_follow_node]: in window'])
    _write(tmp_path, 222, 5000000,
           ['[INFO] [5000.000000000] [gap_follow_node]: way too late'])
    found = find_node_logs(995.0, 1010.0, slack_sec=0.0, log_dir=tmp_path)
    all_paths = [p for paths in found.values() for p in paths]
    assert len(all_paths) == 1
    assert all_paths[0].name == 'python3_111_1000000.log'


def test_slack_widens_the_window_on_both_ends(tmp_path):
    _write(tmp_path, 111, 990000,
           ['[INFO] [990.000000000] [gap_follow_node]: just before'])
    without_slack = find_node_logs(1000.0, 1010.0, slack_sec=0.0, log_dir=tmp_path)
    assert without_slack == {}
    with_slack = find_node_logs(1000.0, 1010.0, slack_sec=15.0, log_dir=tmp_path)
    all_paths = [p for paths in with_slack.values() for p in paths]
    assert len(all_paths) == 1


def test_groups_by_node_name_read_from_the_files_own_first_line(tmp_path):
    _write(tmp_path, 111, 1000000,
           ['[INFO] [1000.100000000] [gap_follow_node]: DRIVE [gap_follow] hi'])
    _write(tmp_path, 222, 1000100,
           ['[INFO] [1000.200000000] [pure_pursuit_node]: DRIVE [pursuing] hi'])
    found = find_node_logs(999.0, 1001.0, slack_sec=0.0, log_dir=tmp_path)
    assert set(found) == {'gap_follow_node', 'pure_pursuit_node'}
    assert len(found['gap_follow_node']) == 1
    assert len(found['pure_pursuit_node']) == 1


def test_files_with_no_recognizable_line_group_as_unknown(tmp_path):
    _write(tmp_path, 111, 1000000, [])   # empty -- a Node built and torn
                                          # down without ever logging
    _write(tmp_path, 111, 1000010, ['not a ROS log line at all'])
    found = find_node_logs(999.0, 1001.0, slack_sec=0.0, log_dir=tmp_path)
    assert set(found) == {'unknown'}
    assert len(found['unknown']) == 2


def test_ignores_files_that_do_not_match_the_naming_pattern(tmp_path):
    (tmp_path / 'not_a_node_log.txt').write_text('irrelevant')
    (tmp_path / 'python3_notanumber_1000000.log').write_text('irrelevant')
    found = find_node_logs(0.0, 2000000.0, slack_sec=0.0, log_dir=tmp_path)
    assert found == {}


def test_paths_within_one_node_are_sorted_oldest_first_by_filename(tmp_path):
    _write(tmp_path, 111, 1000200, ['[INFO] [1000.2] [gap_follow_node]: third'])
    _write(tmp_path, 111, 1000000, ['[INFO] [1000.0] [gap_follow_node]: first'])
    _write(tmp_path, 111, 1000100, ['[INFO] [1000.1] [gap_follow_node]: second'])
    found = find_node_logs(999.0, 1001.0, slack_sec=0.0, log_dir=tmp_path)
    names = [p.name for p in found['gap_follow_node']]
    assert names == [
        'python3_111_1000000.log',
        'python3_111_1000100.log',
        'python3_111_1000200.log',
    ]


def test_raises_when_the_window_is_backwards():
    with pytest.raises(ValueError):
        find_node_logs(10.0, 5.0)


def test_merged_lines_orders_by_each_lines_own_timestamp_not_file_order(tmp_path):
    """Two nodes' files interleaved in real time, deliberately NOT in
    filename-epoch order relative to each other's *content*: the file
    that opened first can still contain later lines than a file that
    opened after it, once a node has been running a while. merged_lines
    must sort by each line's own embedded timestamp, not by which file
    (or which dict key) it came from -- confirmed by constructing exactly
    this interleaving and checking the output order is chronological."""
    _write(tmp_path, 111, 1000000, [
        '[INFO] [1000.000000000] [gap_follow_node]: first',
        '[INFO] [1000.300000000] [gap_follow_node]: third',
    ])
    _write(tmp_path, 222, 1000050, [
        '[INFO] [1000.100000000] [pure_pursuit_node]: second',
        '[INFO] [1000.400000000] [pure_pursuit_node]: fourth',
    ])
    found = find_node_logs(999.0, 1001.0, slack_sec=0.0, log_dir=tmp_path)
    lines = merged_lines(found)
    labels = [line.rsplit(': ', 1)[1] for line in lines]
    assert labels == ['first', 'second', 'third', 'fourth']


def test_merged_lines_keeps_a_non_timestamped_line_after_its_predecessor(tmp_path):
    """A line with no parseable timestamp of its own (a traceback
    continuation, a blank line) must not be dropped, and must not sort to
    some arbitrary position -- it stays immediately after the last
    timestamped line from the same file."""
    _write(tmp_path, 111, 1000000, [
        '[INFO] [1000.000000000] [gap_follow_node]: first',
        '    continuation line with no timestamp',
        '[INFO] [1000.100000000] [gap_follow_node]: third',
    ])
    found = find_node_logs(999.0, 1001.0, slack_sec=0.0, log_dir=tmp_path)
    lines = merged_lines(found)
    assert lines == [
        '[INFO] [1000.000000000] [gap_follow_node]: first',
        '    continuation line with no timestamp',
        '[INFO] [1000.100000000] [gap_follow_node]: third',
    ]


def test_merged_lines_of_no_files_is_empty():
    assert merged_lines({}) == []
