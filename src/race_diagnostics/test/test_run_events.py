"""
Unit tests for run_events.py -- the classification and gate-parsing logic
that decides what a run's logs actually mean.

Pure numbers and strings, no rclpy, so this runs with a bare pytest and no
ROS sourced at all:

    python3 -m pytest src/race_diagnostics/test/ -v

The sample lines below are real, copied verbatim from the 2026-07-27
session that produced this package.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from race_diagnostics.run_events import (  # noqa: E402
    LogClassifier, RunTimeline, blocking_gate, parse_lap_progress)


REAL_LINES = {
    'slam_lifecycle': '[async_slam_toolbox_node-8] [INFO] [1785207335.256745947] [slam_toolbox]: Configuring',
    'lap_closed': '[auto_map_race_node-11] [INFO] [1785209074.7] [auto_map_race_node]: Closed mapping lap 1/2 detected (123.3m, 665 samples).',
    'profile_generated': '[auto_map_race_node-11] [INFO] [1785209098.3] [auto_map_race_node]: Generated 163-point racing profile at /home/x/raceline_profiled.csv (0.87-2.60m/s).',
    'handover': '[auto_map_race_node-11] [INFO] [1785209100.0] [auto_map_race_node]: Transition complete: pure pursuit now has drive control.',
    'node_death': "[ERROR] [auto_map_race_node-11]: process has died [pid 129425, exit code 1, cmd '...'].",
}


@pytest.mark.parametrize('expected,line', REAL_LINES.items())
def test_real_lines_classify_as_expected(expected, line):
    category, emit = LogClassifier().classify(line, 0.0)
    assert category == expected
    assert emit is True, 'critical categories are never throttled'


def test_ordinary_drive_chatter_is_dropped():
    line = ('[gap_follow_node-9] [INFO] [1785206043.7] [gap_follow_node]: DRIVE '
            '[gap_follow] selected preferred depth 2.00m gap -8.3deg to +45.2deg')
    assert LogClassifier().classify(line, 0.0) == (None, False)


def test_noisy_categories_are_throttled_but_still_counted():
    classifier = LogClassifier(throttle_sec=20.0)
    line = ('[async_slam_toolbox_node-8] [INFO] [1.0] [slam_toolbox]: Message Filter '
            "dropping message: frame 'laser' for reason 'discarding message because the queue is full'")
    assert classifier.classify(line, 0.0) == ('scan_dropped', True)
    assert classifier.classify(line, 5.0) == ('scan_dropped', False)
    assert classifier.classify(line, 19.9) == ('scan_dropped', False)
    assert classifier.classify(line, 20.0) == ('scan_dropped', True)


def test_a_watchdog_stop_outranks_the_generic_stop_category():
    """STOP [pose_frozen] must report as the watchdog that fired, not as
    generic 'stopped' noise -- the whole point is that it stands out."""
    line = ('[pure_pursuit_node-10] [WARN] [1.0] [pure_pursuit_node]: STOP [pose_frozen] '
            'odometry reports 1.25m/s but the localization pose has not moved')
    category, emit = LogClassifier().classify(line, 0.0)
    assert category == 'watchdog'
    assert emit is True


REAL_LAP_LINE = (
    '[auto_map_race_node-11] [INFO] [1.0] [auto_map_race_node]: FORWARD '
    '[forwarding_mapping] lap 1/2: samples=605, distance=112.3/20.0m, '
    'elapsed=91.2/15.0s, departed=yes, start distance=3.76/0.75m, '
    'heading error=42.3/30.0deg; output command: steering=+0.0rad, speed=1.00m/s')


def test_lap_progress_parses_every_gate():
    progress = parse_lap_progress(REAL_LAP_LINE)
    assert progress['lap'] == 1 and progress['of'] == 2
    assert progress['samples'] == 605
    assert progress['departed'] is True
    assert progress['distance']['ok'] is True       # 112.3 >= 20.0
    assert progress['elapsed']['ok'] is True        # 91.2 >= 15.0
    assert progress['start_distance']['ok'] is False  # 3.76 > 0.75
    assert progress['heading_error']['ok'] is False   # 42.3 > 30.0


def test_blocking_gate_names_the_first_failing_gate_in_order():
    assert blocking_gate(parse_lap_progress(REAL_LAP_LINE)) == 'start_distance'


def test_the_heading_gate_is_identified_when_it_alone_fails():
    """The real 2026-07-27 case: everything passed except heading, by
    0.2 degrees, and the car drove 114m without ever closing a lap."""
    line = ('lap 1/2: samples=598, distance=111.1/20.0m, elapsed=90.2/15.0s, '
            'departed=yes, start distance=0.39/0.75m, heading error=30.2/30.0deg')
    progress = parse_lap_progress(line)
    assert blocking_gate(progress) == 'heading_error'
    assert progress['heading_error']['value'] == pytest.approx(30.2)
    assert progress['heading_error']['limit'] == pytest.approx(30.0)


def test_a_fully_satisfied_sample_has_no_blocking_gate():
    line = ('lap 2/2: samples=163, distance=30.0/20.0m, elapsed=24.0/15.0s, '
            'departed=yes, start distance=0.20/0.75m, heading error=5.0/30.0deg')
    assert blocking_gate(parse_lap_progress(line)) == ''


def test_not_having_departed_outranks_the_other_gates():
    """At the very start every distance is trivially small; reporting
    'start_distance OK' there would be actively misleading."""
    line = ('lap 1/2: samples=1, distance=0.0/20.0m, elapsed=0.0/15.0s, '
            'departed=no, start distance=0.00/0.75m, heading error=0.0/30.0deg')
    assert blocking_gate(parse_lap_progress(line)) == 'departed'


def test_non_lap_lines_parse_to_none():
    assert parse_lap_progress('some unrelated line') is None
    assert blocking_gate(None) == ''


def test_timeline_records_phases_and_worst_pose_lag():
    timeline = RunTimeline()
    timeline.add(1.0, 'slam_lifecycle', 'Configuring')
    timeline.add(2.0, 'lap_closed', 'lap 1/2')
    timeline.add(3.0, 'lap_closed', 'lap 2/2')
    timeline.note_pose_lag(2.5, 0.31)
    timeline.note_pose_lag(2.9, 3.38)
    timeline.note_pose_lag(3.1, 0.02)

    summary = timeline.summary()
    assert summary['event_counts']['lap_closed'] == 2
    assert summary['pose_lag_max_sec'] == pytest.approx(3.38)
    assert summary['pose_lag_max_at'] == pytest.approx(2.9)
    assert 'slam_lifecycle' in summary['phases_reached']
    assert 'handover' not in summary['phases_reached']


def test_classifier_rejects_a_negative_throttle():
    with pytest.raises(ValueError):
        LogClassifier(throttle_sec=-1.0)
