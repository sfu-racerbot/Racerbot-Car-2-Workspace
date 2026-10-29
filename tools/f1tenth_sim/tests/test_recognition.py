"""Counting rules for the opponent-recognition measures (tools/f1tenth_sim/recognition.py).

Pure Python over synthetic per-tick records: no gym run happens here. Runs
unsourced:

    python3 -m pytest tools/f1tenth_sim/tests/ -v
"""
import math
import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import recognition as rec  # noqa: E402

DT = 0.025  # oracle: spec, the control tick every evaluator runs at (40 Hz, eval_gap.CONTROL_DT)


# --- false_movers: one car alone on track ------------------------------------

def test_no_ticks_and_no_moving_tracks_count_nothing():
    # oracle: closed form, nothing is moving in an empty run or in ticks with no moving track
    assert rec.false_movers([]) == {'false_mover_ticks': 0, 'false_mover_tracks': 0}
    assert rec.false_movers([[], [], []]) == {'false_mover_ticks': 0, 'false_mover_tracks': 0}


def test_one_track_moving_for_five_ticks_is_five_ticks_and_one_track():
    # oracle: closed form, track 3 moves on five ticks and nothing moves on the other three
    per_tick = [[3]] * 5 + [[]] * 3
    assert rec.false_movers(per_tick) == {'false_mover_ticks': 5, 'false_mover_tracks': 1}


def test_two_ids_on_separate_ticks_are_two_ticks_and_two_tracks():
    # oracle: closed form, track 1 moves on one tick and track 2 on another
    assert rec.false_movers([[1], [2]]) == {'false_mover_ticks': 2, 'false_mover_tracks': 2}


def test_two_ids_on_one_tick_are_one_tick_and_two_tracks():
    # oracle: closed form, the tick is counted once however many tracks move on it,
    # and each moving track is counted once
    assert rec.false_movers([[1, 2]]) == {'false_mover_ticks': 1, 'false_mover_tracks': 2}


def test_the_same_id_on_many_ticks_is_one_track():
    # oracle: closed form, four ticks have a moving track and the distinct ids are {1, 2}
    assert rec.false_movers([[1], [1], [2], [1]]) == {'false_mover_ticks': 4, 'false_mover_tracks': 2}


def test_track_id_zero_counts_like_any_other_id():
    # oracle: spec, a track id is a label; a truthiness test on the id would drop zero
    assert rec.false_movers([[0], []]) == {'false_mover_ticks': 1, 'false_mover_tracks': 1}


def _track(track_id, moving):
    """A record with the two attributes eval_gap reads off opponent_tracking.Track."""
    return SimpleNamespace(track_id=track_id, moving=moving)


def test_moving_track_ids_keeps_only_the_moving_tracks_in_order():
    # oracle: closed form, tracks 1 and 3 are moving; track 2 is not
    tracks = [_track(1, True), _track(2, False), _track(3, True)]
    assert rec.moving_track_ids(tracks) == [1, 3]


def test_no_tracks_or_none_moving_gives_no_ids():
    # oracle: closed form, empty and all-stationary inputs
    assert rec.moving_track_ids([]) == []
    assert rec.moving_track_ids([_track(4, False)]) == []


def test_moving_track_id_zero_is_kept():
    # oracle: spec, a track id is a label; zero is a valid id
    assert rec.moving_track_ids([_track(0, True)]) == [0]


# --- recognition_delays: visibility episodes ---------------------------------

def test_empty_input_has_no_episodes():
    # oracle: closed form, no ticks means no episodes
    assert rec.recognition_delays([], [], DT) == []


def test_recognised_after_exactly_k_visible_ticks_is_k_times_dt():
    k = 4
    visible = [True] * 10
    recognised = [False] * k + [True] * (10 - k)
    # oracle: closed form, the episode starts at tick 0 and is first recognised at tick k
    assert rec.recognition_delays(visible, recognised, DT) == [
        pytest.approx(k * DT, abs=1e-12)]  # s


def test_recognised_on_the_first_visible_tick_is_zero():
    # oracle: closed form, zero ticks elapsed from the episode's first tick
    assert rec.recognition_delays([True, True, True], [True, False, False], DT) == [0.0]


def test_a_hidden_lead_in_does_not_move_the_episode_start():
    # oracle: closed form, the episode starts at tick 2 (the first visible tick) and is recognised there
    visible = [False, False, True, True]
    recognised = [False, False, True, True]
    assert rec.recognition_delays(visible, recognised, DT) == [0.0]


def test_never_recognised_is_infinite():
    # oracle: spec, an episode with no recognised tick gives inf
    assert rec.recognition_delays([True] * 5, [False] * 5, DT) == [math.inf]


def test_two_episodes_give_two_delays():
    visible = [False, True, True, True, True, False, False, True, True, True]
    recognised = [False, False, False, True, True, False, False, False, True, True]
    # oracle: closed form, episode 1 (ticks 1-4) is first recognised at tick 3, two ticks in;
    # episode 2 (ticks 7-9) is first recognised at tick 8, one tick in
    assert rec.recognition_delays(visible, recognised, DT) == [
        pytest.approx(2 * DT, abs=1e-12), pytest.approx(1 * DT, abs=1e-12)]  # s


def test_an_episode_that_ends_before_recognition_is_infinite():
    visible = [True, True, True, False, False]
    recognised = [False, False, False, True, True]
    # oracle: spec, recognition after the car has left view does not belong to the episode
    assert rec.recognition_delays(visible, recognised, DT) == [math.inf]


def test_recognition_on_the_tick_after_an_episode_ends_is_not_its_delay():
    # one step past the end: tick 2 is recognised but the leader is not visible there
    visible = [True, True, False, True, True]
    recognised = [False, False, True, True, False]
    # oracle: closed form, episode (0, 1) is never recognised within itself; episode (3, 4) is
    # recognised on its first tick
    assert rec.recognition_delays(visible, recognised, DT) == [math.inf, 0.0]


def test_a_true_first_tick_starts_an_episode():
    # oracle: closed form, the episode starts at tick 0 and is recognised one tick later
    assert rec.recognition_delays([True, True], [False, True], DT) == [
        pytest.approx(DT, abs=1e-12)]  # s


def test_recognition_with_the_leader_hidden_starts_no_episode():
    # oracle: spec, an episode needs the leader visible
    assert rec.recognition_delays([False, False, False], [True, True, True], DT) == []


def test_single_tick_episodes():
    # oracle: closed form, one visible tick is recognised at delay zero or never
    assert rec.recognition_delays([True], [True], DT) == [0.0]
    assert rec.recognition_delays([True], [False], DT) == [math.inf]


def test_unequal_lengths_are_refused():
    # oracle: spec, the two per-tick records must describe the same ticks
    with pytest.raises(ValueError, match='differ in length'):
        rec.recognition_delays([True, False], [True], DT)


# --- recognition_counts and recognised_fraction ------------------------------

def test_counts_only_recognised_ticks_that_are_visible():
    # oracle: closed form, visible on ticks 0, 1 and 3; recognised on ticks 0 and 3 of those
    assert rec.recognition_counts([True, True, False, True], [True, False, True, True]) == {
        'visible_ticks': 3, 'recognised_ticks': 2}


def test_recognition_while_hidden_is_not_counted():
    # oracle: spec, a recognised tick with the leader hidden does not count toward the fraction
    assert rec.recognition_counts([False, True], [True, False]) == {
        'visible_ticks': 1, 'recognised_ticks': 0}


def test_empty_counts_are_zero():
    # oracle: closed form, no ticks
    assert rec.recognition_counts([], []) == {'visible_ticks': 0, 'recognised_ticks': 0}


def test_counts_refuse_unequal_lengths():
    # oracle: spec, the two per-tick records must describe the same ticks
    with pytest.raises(ValueError, match='differ in length'):
        rec.recognition_counts([True], [True, False])


def test_fraction_is_recognised_over_visible_and_none_without_visibility():
    # oracle: closed form, the fraction of visible ticks recognised; undefined with none visible
    assert rec.recognised_fraction(0, 0) is None
    assert rec.recognised_fraction(3, 2) == pytest.approx(2 / 3, abs=1e-12)
    assert rec.recognised_fraction(4, 4) == 1.0
    assert rec.recognised_fraction(5, 0) == 0.0


# --- median_delay -------------------------------------------------------------

def test_median_of_no_episodes_is_none():
    # oracle: spec, no episode has no median
    assert rec.median_delay([]) is None


def test_median_of_an_odd_count_is_the_middle_value_in_any_order():
    # oracle: closed form, sorted [0.1, 0.2, 0.3] has middle 0.2
    assert rec.median_delay([0.3, 0.1, 0.2]) == 0.2


def test_median_of_an_even_count_is_the_mean_of_the_middle_two():
    # oracle: closed form, (0.1 + 0.3) / 2
    assert rec.median_delay([0.1, 0.3]) == pytest.approx(0.2, abs=1e-12)  # s


def test_a_never_recognised_episode_is_infinite_in_the_median():
    # oracle: spec, inf sorts last; with one of three inf the middle is the larger finite value
    assert rec.median_delay([math.inf, 0.1, 0.3]) == 0.3
    # oracle: spec, with most episodes never recognised the median is inf
    assert rec.median_delay([math.inf, math.inf, 0.1]) == math.inf
