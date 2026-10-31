"""
race_diagnostics.run_activity: what actually happened during a recorded run.
No ROS needed:

    python3 -m pytest src/race_diagnostics/test/test_run_activity.py -v

Oracles: closed forms over the speeds and button states passed in, and the
spec in docs/run-diagnostics.md#automatic-recording (a run whose first odometry
is already moving lost its start; a run that never moves is flagged, never
deleted). The real bag round trip is in test_auto_record_node.py.
"""
import math
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from race_diagnostics import run_activity as ra  # noqa: E402


def test_a_run_that_starts_at_rest_and_drives_is_not_late():
    out = ra.summarize([0.0, 0.0, 1.0, 3.5, 2.0], [0, 1, 1, 0])
    assert out['moved'] is True
    assert out['started_late'] is False
    assert out['first_odom_speed_mps'] == 0.0
    assert out['max_speed_mps'] == 3.5
    # oracle: 2 of the 4 /joy samples hold LB.
    assert out['lb_held_fraction'] == 0.5


def test_a_car_already_moving_at_the_first_odom_lost_its_start():
    # recorded measurement: run 20261030-184149's first /odom was 3.5 m/s.
    out = ra.summarize([3.5, 3.6, 3.7], [1, 1])
    assert out['started_late'] is True
    assert out['first_odom_speed_mps'] == 3.5


def test_reversing_counts_as_moving_and_as_a_late_start():
    out = ra.summarize([-2.0, -1.0], [])
    assert out['moved'] is True
    assert out['started_late'] is True
    # oracle: the reported maximum is a speed, the magnitude of -2.0.
    assert out['max_speed_mps'] == 2.0


def test_a_run_that_never_moves_is_flagged():
    out = ra.summarize([0.0] * 100, [0] * 10)
    assert out['moved'] is False
    assert out['started_late'] is False
    assert out['max_speed_mps'] == 0.0
    assert out['lb_held_fraction'] == 0.0


def test_the_moving_threshold_is_inclusive_and_one_step_below_is_not_moving():
    at = ra.MOVING_MPS
    below = math.nextafter(ra.MOVING_MPS, 0.0)
    assert ra.summarize([at], [])['moved'] is True
    assert ra.summarize([at], [])['started_late'] is True
    assert ra.summarize([below], [])['moved'] is False
    assert ra.summarize([below], [])['started_late'] is False


def test_no_odometry_says_unknown_rather_than_did_not_move():
    # A bag recorded with a custom topic list has no /odom. "Did not move"
    # would invite someone to discard it.
    out = ra.summarize([], [1, 0])
    assert out['moved'] is None
    assert out['started_late'] is None
    assert out['first_odom_speed_mps'] is None
    assert out['max_speed_mps'] is None
    assert out['lb_held_fraction'] == 0.5


def test_no_joy_says_unknown_rather_than_lb_never_held():
    assert ra.summarize([0.0], [])['lb_held_fraction'] is None


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_non_finite_speeds_are_ignored(bad):
    out = ra.summarize([bad, 0.0, 1.0], [])
    assert out['first_odom_speed_mps'] == 0.0
    assert out['started_late'] is False
    assert out['max_speed_mps'] == 1.0


@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_only_non_finite_speeds_is_the_same_as_no_odometry(bad):
    assert ra.summarize([bad, bad], [])['moved'] is None


def test_lb_is_read_from_the_deadman_button_gap_follow_ships():
    # oracle: spec -- src/gap_follow/config/gap_follow.yaml deadman_button.
    root = os.path.join(os.path.dirname(__file__), '..', '..', 'gap_follow', 'config')
    with open(os.path.join(root, 'gap_follow.yaml')) as handle:
        shipped = yaml.safe_load(handle)['gap_follow_node']['ros__parameters']
    assert shipped['deadman_button'] == ra.LB_BUTTON
    pressed = [0] * 11
    pressed[ra.LB_BUTTON] = 1
    assert ra.lb_state(pressed) == 1
    assert ra.lb_state([1] * ra.LB_BUTTON + [0] * 7) == 0


def test_a_joy_message_too_short_to_hold_the_button_is_not_held():
    assert ra.lb_state([]) == 0
    assert ra.lb_state([1] * ra.LB_BUTTON) == 0


def test_add_to_info_keeps_what_the_recorder_wrote(tmp_path):
    path = tmp_path / 'auto_record.yaml'
    path.write_text(yaml.safe_dump({'started': 'then', 'recorder_exit_code': 0}))
    ra.add_to_info(tmp_path, {'moved': False})
    assert yaml.safe_load(path.read_text()) == {
        'started': 'then', 'recorder_exit_code': 0, 'activity': {'moved': False}}
    assert sorted(p.name for p in tmp_path.iterdir()) == ['auto_record.yaml'], \
        'a temporary file was left beside it'


def test_a_late_start_is_written_to_the_decision_log(tmp_path):
    run = tmp_path / '20261030-184149'
    run.mkdir()
    ra.report(run, {'started_late': True, 'first_odom_speed_mps': 3.5})
    text = (tmp_path / 'auto_record.log').read_text()
    assert '20261030-184149' in text and '3.5 m/s' in text


def test_a_clean_start_adds_nothing_to_the_decision_log(tmp_path):
    run = tmp_path / '20261030-175124'
    run.mkdir()
    ra.report(run, {'started_late': False, 'first_odom_speed_mps': 0.0})
    assert not (tmp_path / 'auto_record.log').exists()


def test_an_unreadable_bag_is_recorded_as_an_error_and_never_raises(tmp_path):
    # A killed recorder leaves a bag with no metadata.yaml. main() runs
    # detached after the run; it has nowhere to raise to.
    (tmp_path / 'auto_record.yaml').write_text(yaml.safe_dump({'started': 'then'}))

    def broken_reader(_run):
        raise OSError('no bag here')

    assert ra.main([str(tmp_path)], reader=broken_reader) == 0
    activity = yaml.safe_load((tmp_path / 'auto_record.yaml').read_text())['activity']
    assert activity == {'error': "OSError('no bag here')"}
