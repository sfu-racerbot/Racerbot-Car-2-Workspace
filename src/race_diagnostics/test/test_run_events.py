"""
Unit tests for run_events.py -- the classification and gate-parsing logic
that decides what a run's logs actually mean.

Pure numbers and strings, no rclpy, so this runs with a bare pytest and no
ROS sourced at all:

    python3 -m pytest src/race_diagnostics/test/ -v

The sample lines below are real, copied verbatim from the 2026-07-27
session that produced this package -- except the lap-progress lines
(REAL_LAP_LINE and its siblings below test_lap_progress_parses_every_gate),
which predate the 'turn=T/300deg' field auto_map_race_node's real lap-
progress lines have always included since (see run_events.py's
LAP_PROGRESS_RE docstring, and 2026-08-25's regex fix). Those lines have
had 'turn=330/300deg, ' spliced in by hand to keep them shaped like a
lap-progress line the current code can actually produce; every other
field is untouched from the original capture. NEW_WATCHDOG_LINES and
ESCAPE_MANEUVER_LINES below carry their own provenance notes -- most are
real captures, a few are constructed directly from source where this
machine's own log history happens not to contain a real example.
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


# Every hard-stop state gap_follow_node/pure_pursuit_node can log that was
# missing from CRITICAL_PATTERNS's watchdog alternation before 2026-08-25 --
# each was silently falling into the generic THROTTLED 'stopped' catch-all
# instead of being named, which is exactly the failure summarize_run's new
# "Unclassified stops" count exists to catch (see summarize_run.render).
# Lines are real, taken verbatim from this machine's own ~/.ros/log/; a few
# (noted below) are constructed directly from the source f-string because
# this machine's history does not happen to contain a real example of that
# specific state.
NEW_WATCHDOG_LINES = {
    'emergency_clearance': (
        '[WARN] [1787193350.514093650] [gap_follow_node]: STOP '
        '[emergency_clearance] minimum body clearance -0.147m is at or '
        'below the 0.020m threshold; command: steering=+0.000rad, speed=0.00m/s'),
    'ttc_brake': (
        '[WARN] [1787452767.875881949] [gap_follow_node]: STOP [ttc_brake] '
        'minimum footprint-aware TTC 0.273s is at or below the 0.350s '
        'threshold at effective speed 1.20m/s (odom 1.20m/s, recent '
        'command 0.00m/s); command: steering=+0.000rad, speed=0.00m/s'),
    'no_safe_gap': (
        '[WARN] [1787452880.837838271] [gap_follow_node]: STOP [no_safe_gap] '
        'no gap exceeds either the preferred 2.00m depth or tight-corner '
        '0.80m depth with 0.10m of center corridor; closest=0.35m; '
        'command: steering=+0.000rad, speed=0.00m/s'),
    'scan_window_empty': (  # constructed -- see module docstring above
        '[WARN] [1787452900.100000000] [gap_follow_node]: STOP '
        '[scan_window_empty] no beams fall inside the configured 180.0deg '
        'FOV; command: steering=+0.000rad, speed=0.00m/s'),
    'odometry_stale': (
        '[WARN] [1787197234.188158719] [gap_follow_node]: STOP '
        '[odometry_stale] TTC is enabled but no odometry newer than 0.50s '
        "is available on '/odom'; command: steering=+0.000rad, speed=0.00m/s"),
    'scan_empty': (
        '[WARN] [1785173325.938477436] [gap_follow_node]: STOP [scan_empty] '
        'LaserScan contains no range beams; command: steering=+0.000rad, '
        'speed=0.00m/s'),
    'scan_invalid': (  # constructed -- see module docstring above
        '[WARN] [1787452900.200000000] [gap_follow_node]: STOP '
        '[scan_invalid] LaserScan angle_increment=0.0 is not positive and '
        'finite; command: steering=+0.000rad, speed=0.00m/s'),
    'waiting_for_scan': (
        '[WARN] [1785207609.468717334] [gap_follow_node]: STOP '
        "[waiting_for_scan] no LaserScan received on '/scan'; no drive "
        'command is being generated; command: none (the mux stops when '
        '/drive times out)'),
    'scan_stale': (
        '[WARN] [1786423508.151770823] [gap_follow_node]: STOP [scan_stale] '
        'last LaserScan is 0.62s old (limit 0.50s); /drive has gone quiet '
        'and the mux will stop the car; command: none (the mux stops when '
        '/drive times out)'),
    'lidar_scan_missing': (
        '[WARN] [1785205076.111186664] [pure_pursuit_node]: STOP '
        "[lidar_scan_missing] no LaserScan received on '/scan', so the "
        'LIDAR safety net is blind; command: steering=+0.000rad, speed=0.00m/s'),
    'lidar_scan_stale': (  # constructed -- see module docstring above
        '[WARN] [1787452900.300000000] [pure_pursuit_node]: STOP '
        '[lidar_scan_stale] last LaserScan is 0.65s old (limit 0.50s); '
        'command: steering=+0.000rad, speed=0.00m/s'),
    'avoidance_scan_empty': (  # constructed -- see module docstring above
        '[WARN] [1787452900.400000000] [pure_pursuit_node]: STOP '
        '[avoidance_scan_empty] map-unexplained object at 1.20m triggered '
        'avoidance, but the 60.0deg scan window is empty; command: '
        'steering=+0.000rad, speed=0.00m/s'),
    'avoidance_boxed_in': (  # constructed -- see module docstring above
        '[WARN] [1787452900.500000000] [pure_pursuit_node]: STOP '
        '[avoidance_boxed_in] map-unexplained object at 1.20m triggered '
        'avoidance, but no gap is deeper than 1.00m; command: '
        'steering=+0.000rad, speed=0.00m/s'),
    'waiting_for_profile': (
        '[WARN] [1786314995.670998710] [pure_pursuit_node]: STOP '
        '[waiting_for_profile] no racing-line profile is active; waiting '
        'for waypoints_file to be loaded; command: steering=+0.000rad, '
        'speed=0.00m/s'),
    'control_exception': (  # constructed -- see module docstring above
        '[ERROR] [1787452993.693309396] [pure_pursuit_node]: STOP '
        '[control_exception] unhandled ZeroDivisionError: division by '
        'zero; node will exit after publishing stop; command: '
        'steering=+0.000rad, speed=0.00m/s'),
    'waiting_for_joy': (
        '[WARN] [1787452684.361634721] [gap_follow_node]: STOP '
        "[waiting_for_joy] no Joy messages received on '/joy'; LB cannot "
        'be verified; command: steering=+0.000rad, speed=0.00m/s'),
    'joy_stale': (
        '[WARN] [1785205073.992631553] [gap_follow_node]: STOP [joy_stale] '
        'last Joy message is 0.60s old (limit 0.50s); command: '
        'steering=+0.000rad, speed=0.00m/s'),
    'deadman_button_missing': (  # constructed -- see module docstring above
        '[WARN] [1787452900.600000000] [gap_follow_node]: STOP '
        '[deadman_button_missing] Joy message has no button index 4 (LB); '
        'command: steering=+0.000rad, speed=0.00m/s'),
    'deadman_released': (
        '[WARN] [1786315117.783993559] [gap_follow_node]: STOP '
        '[deadman_released] LB deadman button is not held; command: '
        'steering=+0.000rad, speed=0.00m/s'),
}


@pytest.mark.parametrize('state,line', NEW_WATCHDOG_LINES.items())
def test_new_watchdog_states_classify_as_watchdog(state, line):
    category, emit = LogClassifier().classify(line, 0.0)
    assert category == 'watchdog', (
        f'{state!r} did not classify as a watchdog: got {category!r} '
        f'from line {line!r}')
    assert emit is True


# emergency_escape is pure_pursuit's pre-existing tier-1 escape and is real,
# taken verbatim from this machine's ~/.ros/log/. The other three do not
# exist in gap_follow_node.py/pure_pursuit_node.py as of this commit -- this
# plan adds them next, in sections 2 and 3 -- so these are constructed in
# the exact _log_decision shape both nodes already use for every other
# state, proving the pattern against the shape those states will actually
# log in rather than just plausible-looking text.
ESCAPE_MANEUVER_LINES = {
    'emergency_escape': (
        '[INFO] [1787453010.120020347] [pure_pursuit_node]: DRIVE '
        '[emergency_escape] closest valid return in the 60.0deg safety '
        'cone is 0.30m, inside the 0.40m emergency threshold, but the '
        'body still has 0.178m all round and a 0.80m opening exists at '
        '-14.9deg -- crawling out at 0.25m/s rather than latching '
        'stopped; command: steering=-0.100rad, speed=0.25m/s'),
    'ttc_escape': (
        '[INFO] [1787453100.000000000] [gap_follow_node]: DRIVE '
        '[ttc_escape] minimum footprint-aware TTC 0.273s is at or below '
        'the 0.350s threshold, but a gap is visible dead ahead -- '
        'crawling out at 0.40m/s rather than braking to zero; command: '
        'steering=+0.166rad, speed=0.40m/s'),
    'body_contact_escape': (
        '[INFO] [1787453200.000000000] [pure_pursuit_node]: DRIVE '
        '[body_contact_escape] minimum clearance from the car body is '
        '0.02m, but a gap is visible dead ahead -- crawling out at '
        '0.25m/s rather than latching stopped; command: '
        'steering=+0.080rad, speed=0.25m/s'),
    'off_racing_line_recovery': (
        '[INFO] [1787453300.000000000] [pure_pursuit_node]: DRIVE '
        '[off_racing_line_recovery] cross-track error 1.40m exceeds '
        'max_cross_track_error, steering toward the nearest waypoint; '
        'command: steering=+0.120rad, speed=0.30m/s'),
}


@pytest.mark.parametrize('state,line', ESCAPE_MANEUVER_LINES.items())
def test_escape_maneuvers_classify_as_escape_maneuver(state, line):
    """These publish a nonzero crawl speed and log with the DRIVE prefix,
    never STOP -- so they must classify as the new escape_maneuver
    category. Before this category existed, a line exactly like
    emergency_escape's above matched neither the STOP-anchored watchdog
    pattern nor the STOP-anchored generic 'stopped' catch-all, so it was
    invisible to this whole classification scheme, not merely
    miscategorized."""
    category, emit = LogClassifier().classify(line, 0.0)
    assert category == 'escape_maneuver', (
        f'{state!r} did not classify as escape_maneuver: got {category!r}')
    assert emit is True


# auto_map_race_node's own stop states -- discovered missing by testing
# summarize_run's "Unclassified stops" count (section 6.3) against a real
# historical run: without naming these, every real auto_map_race_launch.py
# run shows a permanent nonzero baseline from these alone (40 of them in
# one ~2-minute window), burying the actual signal that count exists to
# surface. Lines are real, taken verbatim from this machine's own
# ~/.ros/log/, except pure_pursuit_command_missing and
# unknown_supervisor_state (constructed -- this machine's history has no
# real example of either).
SUPERVISOR_STOP_LINES = {
    'gap_follow_command_missing': (
        "[WARN] [1786214540.197635766] [auto_map_race_node]: STOP "
        "[gap_follow_command_missing] no command has arrived on the "
        "selected 'gap_follow' input; output command: steering=+0.000rad, "
        "speed=0.00m/s"),
    'gap_follow_command_stale': (
        "[WARN] [1785432005.420496828] [auto_map_race_node]: STOP "
        "[gap_follow_command_stale] selected 'gap_follow' command is "
        "0.60s old (limit 0.50s); output command: steering=+0.000rad, "
        "speed=0.00m/s"),
    'pure_pursuit_command_missing': (  # constructed -- see docstring above
        "[WARN] [1787453400.000000000] [auto_map_race_node]: STOP "
        "[pure_pursuit_command_missing] no command has arrived on the "
        "selected 'pure_pursuit' input; output command: steering=+0.000rad, "
        "speed=0.00m/s"),
    'pure_pursuit_command_stale': (
        "[WARN] [1786229879.430911542] [auto_map_race_node]: STOP "
        "[pure_pursuit_command_stale] selected 'pure_pursuit' command is "
        "0.50s old (limit 0.50s); output command: steering=+0.000rad, "
        "speed=0.00m/s"),
    'mapping_controller_stop': (
        "[WARN] [1786214542.948701076] [auto_map_race_node]: STOP "
        "[mapping_controller_stop] selected 'gap_follow' command is fresh "
        "(age=0.01s); gap follow requested a neutral command; output "
        "command: steering=+0.000rad, speed=0.00m/s"),
    'racing_controller_stop': (
        "[WARN] [1786214631.223896424] [auto_map_race_node]: STOP "
        "[racing_controller_stop] selected 'pure_pursuit' command is fresh "
        "(age=0.02s); pure pursuit requested a neutral command; output "
        "command: steering=-0.260rad, speed=0.00m/s"),
    'loading_profile': (
        "[INFO] [1786230244.260812657] [auto_map_race_node]: STOP "
        "[loading_profile] generated profile="
        "'/home/x/raceline_profiled.csv'; waiting for pure pursuit to "
        "accept it; output command: steering=+0.000rad, speed=0.00m/s"),
    'transition_hold': (
        "[INFO] [1786230246.617621539] [auto_map_race_node]: STOP "
        "[transition_hold] profile loaded; deliberate stop before racing "
        "has 1.99s remaining; output command: steering=+0.000rad, "
        "speed=0.00m/s"),
    'supervisor_error': (
        "[ERROR] [1786217608.882190476] [auto_map_race_node]: STOP "
        "[supervisor_error] automatic map-to-race transition failed; "
        "remaining stopped; output command: steering=+0.000rad, "
        "speed=0.00m/s"),
    'unknown_supervisor_state': (  # constructed -- see docstring above
        "[ERROR] [1787453500.000000000] [auto_map_race_node]: STOP "
        "[unknown_supervisor_state] unrecognized supervisor state "
        "'bogus'; remaining stopped; output command: steering=+0.000rad, "
        "speed=0.00m/s"),
}


@pytest.mark.parametrize('state,line', SUPERVISOR_STOP_LINES.items())
def test_supervisor_stop_states_classify_as_supervisor_stop_not_watchdog(state, line):
    """These are auto_map_race_node narrating why it forwarded nothing
    this tick -- routine bookkeeping (loading_profile/transition_hold fire
    on every successful run), not a driving node's safety watchdog. Must
    land in their own category, not 'watchdog' (which would make that
    safety tally misleading) and not the generic unclassified 'stopped'
    bucket (which would make every real run show a permanent false-
    positive baseline in summarize_run's "Unclassified stops" count)."""
    category, emit = LogClassifier().classify(line, 0.0)
    assert category == 'supervisor_stop', (
        f'{state!r} did not classify as supervisor_stop: got {category!r}')
    assert emit is True


REAL_LAP_LINE = (
    '[auto_map_race_node-11] [INFO] [1.0] [auto_map_race_node]: FORWARD '
    '[forwarding_mapping] lap 1/2: samples=605, distance=112.3/20.0m, '
    'turn=330/300deg, elapsed=91.2/15.0s, departed=yes, start distance=3.76/0.75m, '
    'heading error=42.3/30.0deg; output command: steering=+0.0rad, speed=1.00m/s')


def test_lap_progress_parses_every_gate():
    progress = parse_lap_progress(REAL_LAP_LINE)
    assert progress['lap'] == 1 and progress['of'] == 2
    assert progress['samples'] == 605
    assert progress['departed'] is True
    assert progress['distance']['ok'] is True       # 112.3 >= 20.0
    assert progress['turn']['ok'] is True           # abs(330) >= 300
    assert progress['turn']['value'] == pytest.approx(330.0)
    assert progress['elapsed']['ok'] is True        # 91.2 >= 15.0
    assert progress['start_distance']['ok'] is False  # 3.76 > 0.75
    assert progress['heading_error']['ok'] is False   # 42.3 > 30.0


def test_blocking_gate_names_the_first_failing_gate_in_order():
    assert blocking_gate(parse_lap_progress(REAL_LAP_LINE)) == 'start_distance'


def test_the_heading_gate_is_identified_when_it_alone_fails():
    """The real 2026-07-27 case: everything passed except heading, by
    0.2 degrees, and the car drove 114m without ever closing a lap."""
    line = ('lap 1/2: samples=598, distance=111.1/20.0m, turn=330/300deg, '
            'elapsed=90.2/15.0s, '
            'departed=yes, start distance=0.39/0.75m, heading error=30.2/30.0deg')
    progress = parse_lap_progress(line)
    assert blocking_gate(progress) == 'heading_error'
    assert progress['heading_error']['value'] == pytest.approx(30.2)
    assert progress['heading_error']['limit'] == pytest.approx(30.0)


def test_a_fully_satisfied_sample_has_no_blocking_gate():
    line = ('lap 2/2: samples=163, distance=30.0/20.0m, turn=330/300deg, '
            'elapsed=24.0/15.0s, '
            'departed=yes, start distance=0.20/0.75m, heading error=5.0/30.0deg')
    assert blocking_gate(parse_lap_progress(line)) == ''


def test_not_having_departed_outranks_the_other_gates():
    """At the very start every distance is trivially small; reporting
    'start_distance OK' there would be actively misleading."""
    line = ('lap 1/2: samples=1, distance=0.0/20.0m, turn=0.0/300deg, '
            'elapsed=0.0/15.0s, '
            'departed=no, start distance=0.00/0.75m, heading error=0.0/30.0deg')
    assert blocking_gate(parse_lap_progress(line)) == 'departed'


def test_non_lap_lines_parse_to_none():
    assert parse_lap_progress('some unrelated line') is None
    assert blocking_gate(None) == ''


def test_turn_gate_uses_absolute_value_but_reports_the_signed_reading():
    """LapRecorder.update gates on abs(self.turn) >= min_turn_rad -- a lap
    can be driven with net counter-steer in it and still satisfy the
    gate. The reported value must keep its sign (real information: which
    way the net turn went) while 'ok' compares magnitude against the
    limit."""
    line = ('lap 1/2: samples=605, distance=112.3/20.0m, turn=-159/300deg, '
            'elapsed=91.2/15.0s, departed=yes, start distance=3.76/0.75m, '
            'heading error=42.3/30.0deg')
    progress = parse_lap_progress(line)
    assert progress['turn']['value'] == pytest.approx(-159.0)
    assert progress['turn']['limit'] == pytest.approx(300.0)
    assert progress['turn']['ok'] is False   # abs(-159) = 159 < 300


def test_blocking_gate_names_turn_when_turn_alone_fails():
    """turn was silently missing from blocking_gate's gate list before
    2026-08-25, even though LAP_PROGRESS_RE (once fixed) always had a
    value for it -- and turn was the gate actually blocking every one of
    that day's real runs."""
    line = ('lap 1/2: samples=200, distance=30.0/20.0m, turn=170/300deg, '
            'elapsed=24.0/15.0s, departed=yes, start distance=0.20/0.75m, '
            'heading error=5.0/30.0deg')
    progress = parse_lap_progress(line)
    assert blocking_gate(progress) == 'turn'


def test_lap_progress_line_classifies_and_parses_end_to_end():
    """The real, current auto_map_race_node._lap_progress_detail format,
    verbatim in shape as of 2026-08-25: the '~X% round[, ~Ym to go...]'
    summary and 'turn=T/300deg' fields were both added to the node after
    LAP_PROGRESS_RE (and, independently, THROTTLED_PATTERNS's own
    lap_progress classify() pattern) were written, so neither matched a
    real line any more -- summarize_run's pipeline only ever calls
    parse_lap_progress after LogClassifier has *already* tagged a line
    'lap_progress' (see summarize_run.analyze), so both regexes had to
    agree, and both had gone stale the same way on the same day. This is
    the truest reproduction of the reported symptom -- on 2026-08-25,
    every real run's lap-progress lines were silently invisible to
    summarize_run end-to-end, not just to parse_lap_progress in
    isolation. Turn is deliberately negative (a counter-steered lap) to
    prove the sign is captured, not just the magnitude."""
    line = (
        "lap 1/2: ~53% round, ~59m to go (~72s at last lap's pace), "
        "samples=605, distance=112.3/20.0m, turn=-159/300deg, "
        "elapsed=91.2/15.0s, departed=yes, start distance=3.76/0.75m, "
        "heading error=42.3/30.0deg, SLAM corrections absorbed=12")
    category, emit = LogClassifier(throttle_sec=0.0).classify(line, 0.0)
    assert category == 'lap_progress'
    assert emit is True
    progress = parse_lap_progress(line)
    assert progress is not None
    assert progress['lap'] == 1 and progress['of'] == 2
    assert progress['samples'] == 605
    assert progress['turn']['value'] == pytest.approx(-159.0)
    assert blocking_gate(progress) == 'turn'


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
