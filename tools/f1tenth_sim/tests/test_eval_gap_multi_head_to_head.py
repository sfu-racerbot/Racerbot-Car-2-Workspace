"""Head-to-head bookkeeping in tools/f1tenth_sim/eval_gap_multi.py (Task 14):
who finished first and by how much, what each car touched, who moved from
behind to ahead of whom, where each car starts, and the --car-set / --speeds
parsing that gives each car its own gap_follow parameters.

Pure Python over synthetic results and numpy geometry: no gym run happens in
this file. Runs unsourced:

    python3 -m pytest tools/f1tenth_sim/tests/ -v
"""
import math
import os
import re
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import eval_gap_multi as egm  # noqa: E402  (bootstraps gym, as test_gym_collision_response.py does)

TICK = egm.CONTROL_DT                       # the control tick every run uses
NEED = math.ceil(0.5 / TICK)                # oracle: closed form, ticks in 0.5 s (20 at 0.025 s)
RADIUS = 10.0                               # the circle the progress and start tests drive on, m
SAMPLE = 2.0 * math.pi * RADIUS / 3600      # oracle: closed form, 1.7 cm between samples of a 3600-point circle
# The shipped gap_follow.yaml values, as plain inputs (src/gap_follow/config/gap_follow.yaml).
BASE = {'max_speed': 3.5, 'min_speed': 1.0, 'corner_speed': 1.5, 'corner_speed_wide': 1.9,
        'max_lateral_accel': 2.5}


def _car(finish=None, laps=0.0, car_contact=False, wall_contact=False, progress=(), speed=None):
    return {'finish_time_s': finish, 'laps': laps, 'car_contact': car_contact,
            'wall_contact': wall_contact, 'progress_m': list(progress),
            'speed_mps': list(speed) if speed is not None else [1.0] * len(progress)}


def _run(*cars):
    return {'cars': list(cars), 'dt_s': TICK}


def _seq(*runs):
    """(value, ticks) pairs -> the values, each repeated ``ticks`` times."""
    return [value for value, ticks in runs for _ in range(ticks)]


def _traces(gap, speed=10.0):
    """Two cars' progress traces whose gap (car 1 minus car 0) is ``gap`` at
    every tick. Car 0 drives at ``speed`` m/s, so the traces look like a run."""
    car0 = [speed * TICK * (i + 1) for i in range(len(gap))]
    return [car0, [c + g for c, g in zip(car0, gap)]]


def _circle(radius=RADIUS, n=3600):
    """A closed circle sampled like a centreline: stands in for line.xs/ys/yaws."""
    theta = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    return SimpleNamespace(xs=radius * np.cos(theta), ys=radius * np.sin(theta),
                           yaws=theta + np.pi / 2.0)


def _reference():
    line = _circle()
    return np.column_stack((line.xs, line.ys))


def _on_circle(angle):
    return (RADIUS * math.cos(angle), RADIUS * math.sin(angle))


def _drive(progress, angles):
    for theta in angles:
        progress.update(_on_circle(theta))


# --- head_to_head: finishing, contact and winner -----------------------------

def test_clean_win_goes_to_the_earlier_finisher():
    out = egm.head_to_head(_run(_car(finish=41.55, laps=2.0), _car(finish=40.00, laps=2.0)))
    # oracle: closed form, worked out by hand from the synthetic input
    assert out['finish_order'] == [1, 0]
    assert out['winner'] == 1
    assert out['gap_s'] == pytest.approx(1.55, abs=1e-9)   # oracle: closed form, 41.55 - 40.00
    assert out['contacts'] == [False, False]
    assert out['overtakes'] == [0, 0]


def test_winner_is_the_clean_car_when_the_first_finisher_touched_another():
    # Car 1 crosses first but touched car 0 on the way; car 0 finishes clean a second later.
    out = egm.head_to_head(_run(_car(finish=41.00), _car(finish=40.00, car_contact=True)))
    # oracle: closed form, worked out by hand from the synthetic input
    assert out['finish_order'] == [1, 0]
    assert out['contacts'] == [False, True]
    assert out['winner'] == 0


@pytest.mark.parametrize('kind', ['car_contact', 'wall_contact'])
def test_a_wall_touch_disqualifies_a_finisher_as_much_as_a_car_touch(kind):
    out = egm.head_to_head(_run(_car(finish=39.0, **{kind: True}), _car(finish=40.0)))
    # oracle: closed form, worked out by hand from the synthetic input
    assert out['contacts'] == [True, False]
    assert out['winner'] == 1


def test_nobody_finishing_orders_the_cars_by_distance_and_has_no_winner():
    out = egm.head_to_head(_run(_car(laps=1.2), _car(laps=1.5)))
    # oracle: closed form, worked out by hand from the synthetic input
    assert out['finish_order'] == [1, 0]     # the car further round goes first
    assert out['winner'] is None
    assert math.isinf(out['gap_s']) and out['gap_s'] > 0


def test_a_contact_with_no_clean_finisher_has_no_winner():
    out = egm.head_to_head(_run(_car(finish=40.0, car_contact=True), _car(laps=1.9)))
    # oracle: closed form, worked out by hand from the synthetic input
    assert out['finish_order'] == [0, 1]
    assert out['winner'] is None


def test_one_finisher_wins_and_leaves_the_gap_undefined():
    out = egm.head_to_head(_run(_car(laps=1.9), _car(finish=42.0)))
    # oracle: closed form, worked out by hand from the synthetic input
    assert out['finish_order'] == [1, 0]
    assert out['winner'] == 1
    assert math.isinf(out['gap_s'])


def test_equal_finish_times_give_zero_gap_and_the_lower_index_first():
    out = egm.head_to_head(_run(_car(finish=40.0), _car(finish=40.0)))
    # oracle: closed form, worked out by hand from the synthetic input
    assert out['gap_s'] == 0.0               # oracle: closed form, 40.0 - 40.0
    assert out['finish_order'] == [0, 1]     # documented tie-break: lower index first
    assert out['winner'] == 0


def test_head_to_head_reads_the_overtakes_from_the_progress_traces():
    car0, car1 = _traces(_seq((-3.0, 60), (1.0, 60)))
    out = egm.head_to_head(_run(_car(progress=car0), _car(progress=car1)))
    # oracle: closed form, worked out by hand from the synthetic input
    assert out['overtakes'] == [0, 1]        # car 1 moved from behind to ahead once


# --- count_overtakes: behind to ahead, persistence, pairs --------------------

def test_one_clean_pass_counts_once_for_the_passing_car_only():
    gap = _seq((-3.0, 100), (1.0, 100))      # car 1 starts 3 m behind, then passes
    # oracle: closed form, worked out by hand from the synthetic input
    assert egm.count_overtakes(_traces(gap), TICK) == [0, 1]


def test_side_by_side_flicker_is_not_counted_tick_by_tick():
    # 2 s behind, 3 s flickering across zero every tick (5 cm either way, side by
    # side), then 2 s clearly ahead. One real pass, however many flickers.
    flicker = [0.05 if i % 2 == 0 else -0.05 for i in range(120)]
    gap = _seq((-3.0, 80)) + flicker + _seq((1.0, 80))
    # oracle: closed form, worked out by hand from the synthetic input
    assert egm.count_overtakes(_traces(gap), TICK) == [0, 1]


def test_a_pass_must_hold_for_the_persistence_time_at_the_boundary():
    # oracle: closed form. A pass holding for NEED ticks (0.5 s) counts; one tick less does not.
    assert egm.count_overtakes(_traces(_seq((-3.0, 60), (1.0, NEED))), TICK) == [0, 1]
    assert egm.count_overtakes(_traces(_seq((-3.0, 60), (1.0, NEED - 1))), TICK) == [0, 0]


def test_a_pass_and_a_repass_count_once_for_each_car():
    gap = _seq((-3.0, 60), (1.0, 60), (-1.0, 60))   # car 1 passes, then car 0 passes back
    # oracle: closed form, worked out by hand from the synthetic input
    assert egm.count_overtakes(_traces(gap), TICK) == [1, 1]


def test_every_pair_is_counted_for_the_car_that_passes_the_other():
    # Car 1 passes car 0 at tick 60. Car 2 starts last and passes car 0 and then
    # car 1 together at tick 120. oracle: closed form, the pairwise sign flips.
    car0 = [0.0] * 180
    car1 = _seq((-3.0, 60), (1.0, 120))
    car2 = _seq((-6.0, 60), (-2.0, 60), (2.0, 60))
    assert egm.count_overtakes([car0, car1, car2], TICK) == [0, 1, 2]


def test_a_car_that_stays_ahead_is_never_overtaken():
    # oracle: closed form, worked out by hand from the synthetic input
    assert egm.count_overtakes(_traces(_seq((1.0, 200))), TICK) == [0, 0]


def test_a_gap_that_opens_from_level_is_not_an_overtake():
    # Level at the start, then car 1 pulls clear: it was never behind.
    # oracle: closed form, worked out by hand from the synthetic input
    assert egm.count_overtakes(_traces(_seq((0.0, 50), (1.0, 60))), TICK) == [0, 0]


def test_a_gap_that_stays_level_throughout_is_not_an_overtake():
    # oracle: closed form, worked out by hand from the synthetic input
    assert egm.count_overtakes(_traces(_seq((0.0, 200))), TICK) == [0, 0]


def test_a_single_tick_and_an_empty_run_have_no_overtakes():
    # oracle: closed form, worked out by hand from the synthetic input
    assert egm.count_overtakes([[0.0], [1.0]], TICK) == [0, 0]
    assert egm.count_overtakes([[], []], TICK) == [0, 0]


@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_a_non_finite_progress_sample_is_refused(bad):
    car0, car1 = _traces(_seq((-3.0, 30), (1.0, 30)))
    car1[10] = bad
    # oracle: closed form, worked out by hand from the synthetic input
    with pytest.raises(ValueError, match='non-finite'):
        egm.count_overtakes([car0, car1], TICK)


def test_traces_of_different_lengths_are_refused():
    # oracle: closed form, worked out by hand from the synthetic input
    with pytest.raises(ValueError, match='differ in length'):
        egm.count_overtakes([[0.0, 1.0], [0.0]], TICK)


def test_numpy_float_traces_are_read_like_python_floats():
    # Regression, found by the first real head-to-head run (car1_course_2026_10_26,
    # seed 12345): run_multi's positions are numpy float64, and the sign was taken
    # as (g > 0) - (g < 0), which numpy refuses for its booleans (TypeError: numpy
    # boolean subtract). The crash came after the run, in the report. Against the
    # code of that run this test raises the same TypeError.
    car0, car1 = _traces(_seq((-3.0, 60), (1.0, 60)))
    traces = [np.array(car0, dtype=np.float64), np.array(car1, dtype=np.float64)]
    # oracle: closed form, worked out by hand from the synthetic input
    assert egm.count_overtakes(traces, TICK) == [0, 1]   # oracle: closed form, the same pass as the float test


# --- geometry: loop arcs, start line-up, progress ----------------------------

def test_wrap_arc_takes_the_shorter_way_round_a_loop():
    total = 40.0   # a 40 m loop
    # oracle: closed form, the shorter signed way round a 40 m loop
    assert egm.wrap_arc(3.0, total) == pytest.approx(3.0, abs=1e-12)     # oracle: closed form
    assert egm.wrap_arc(-3.0, total) == pytest.approx(-3.0, abs=1e-12)
    assert egm.wrap_arc(37.0, total) == pytest.approx(-3.0, abs=1e-12)   # 37 m ahead is 3 m behind
    assert egm.wrap_arc(-37.0, total) == pytest.approx(3.0, abs=1e-12)


def test_start_slots_line_the_cars_up_three_metres_apart_on_the_loop():
    line = _circle()
    poses = egm.start_poses(line, 2, 3.0, reverse=False)
    # oracle: closed form, circle geometry: 3 m of arc is angle -3/R
    assert poses[0, :2].tolist() == pytest.approx([RADIUS, 0.0], abs=1e-9)
    # oracle: closed form, 3 m behind along a circle of radius R is angle -3/R
    assert poses[1, :2].tolist() == pytest.approx(list(_on_circle(-3.0 / RADIUS)), abs=SAMPLE)
    swapped = egm.start_poses(line, 2, 3.0, reverse=False, slots=(1, 0))
    # Swapped: car 0 takes the place 3 m behind and car 1 leads.
    assert swapped[0, :2].tolist() == pytest.approx(poses[1, :2].tolist(), abs=1e-9)
    assert swapped[1, :2].tolist() == pytest.approx(poses[0, :2].tolist(), abs=1e-9)


def test_reversed_starts_put_the_follower_three_metres_behind_the_other_way_round():
    poses = egm.start_poses(_circle(), 2, 3.0, reverse=True)
    # oracle: closed form, 'behind' when reversed is +3 m of arc
    assert poses[1, :2].tolist() == pytest.approx(list(_on_circle(3.0 / RADIUS)), abs=SAMPLE)


def test_progress_unwraps_across_the_start_line_and_counts_metres():
    prog = egm._Progress(_reference(), (RADIUS, 0.0), reverse=False)
    angles = [0.05 * i for i in range(1, 315)]     # 2.5 turns, 0.5 m a tick
    _drive(prog, angles)
    travelled = RADIUS * angles[-1]                 # oracle: closed form, arc = radius x angle
    assert prog.distance == pytest.approx(travelled, abs=SAMPLE)
    assert prog.position == pytest.approx(travelled, abs=SAMPLE)   # the reference car's offset is 0


def test_reverse_progress_counts_travel_in_the_direction_the_car_faces():
    prog = egm._Progress(_reference(), (RADIUS, 0.0), reverse=True)
    angles = [-0.05 * i for i in range(1, 41)]     # clockwise: 2.0 rad, 20 m
    _drive(prog, angles)
    travelled = RADIUS * abs(angles[-1])            # oracle: closed form
    assert prog.distance == pytest.approx(travelled, abs=SAMPLE)
    assert prog.position == pytest.approx(travelled, abs=SAMPLE)


@pytest.mark.parametrize('reverse, arc', [(False, -3.0), (True, 3.0)])
def test_a_car_three_metres_behind_has_a_position_of_minus_three(reverse, arc):
    # Behind in its own direction of travel: 3 m back along the loop, which is
    # arc -3 going forward and +3 going backward.
    leader = egm._Progress(_reference(), (RADIUS, 0.0), reverse)
    follower = egm._Progress(_reference(), _on_circle(arc / RADIUS), reverse, origin_s=leader.start_s)
    # oracle: closed form, 3 m behind the leader in its own direction of travel
    assert follower.distance == 0.0
    assert follower.position == pytest.approx(-3.0, abs=SAMPLE)   # oracle: closed form, 3 m behind


def test_laps_count_from_the_line_so_a_car_behind_it_must_cover_its_offset_too():
    # The line is the leader's start; the follower starts 3 m behind it. Regression
    # for the first real head-to-head: the finish counted each car's own travel, so
    # this car would be "finished" at 2 loops of travel, 3 m short of the line, and
    # the car behind won a run it was still behind on the track.
    ref = _reference()
    leader = egm._Progress(ref, (RADIUS, 0.0), reverse=False)
    start = -3.0 / RADIUS
    follower = egm._Progress(ref, _on_circle(start), reverse=False, origin_s=leader.start_s)
    loop = leader.total
    _drive(follower, [start + d / RADIUS for d in [0.5 * i for i in range(1, 257)]])   # 0.5 .. 128.0 m
    # oracle: closed form. 128.0 m of travel from 3 m behind the line is 125.0 m along it,
    # and 2 loops are 2 x 62.83 = 125.66 m, so the car is still short.
    assert follower.position == pytest.approx(125.0, abs=SAMPLE)
    assert follower.laps < 2.0
    _drive(follower, [start + d / RADIUS for d in (128.5, 129.0, 129.5, 130.0)])
    # oracle: closed form. 130.0 m of travel puts it 127.0 m along, past 125.66 m.
    assert follower.position == pytest.approx(127.0, abs=SAMPLE)
    assert follower.laps > 2.0
    assert loop == pytest.approx(2.0 * math.pi * RADIUS, abs=SAMPLE)   # oracle: closed form, circumference


def test_a_swapped_line_up_is_still_measured_from_the_line():
    # Car 1 leads from the line and car 0 starts 3 m behind it. Regression, found by
    # the first real run after the finish fix: the origin was car 0's own start, so
    # with car 0 behind, the leader was measured 3 m too far ahead and finished 3 m
    # early (its time moved by 1.8 s with nothing else changed).
    poses = egm.start_poses(_circle(), 2, 3.0, reverse=False, slots=(1, 0))
    car0, car1 = egm.line_up_progress(_reference(), poses, reverse=False)
    assert car0.position == pytest.approx(-3.0, abs=SAMPLE)   # oracle: closed form, 3 m behind the line
    assert car1.position == pytest.approx(0.0, abs=SAMPLE)    # oracle: closed form, on the line
    assert car1.laps == pytest.approx(0.0, abs=SAMPLE / car1.total)   # oracle: closed form, zero laps on the line


# --- --car-set and --speeds parsing ------------------------------------------

@pytest.mark.parametrize('item', [
    '2:max_lateral_accel=2.0',           # index past the last car
    '-1:max_lateral_accel=2.0',          # negative index
    'x:max_lateral_accel=2.0',           # index is not a number
    '1.0:max_lateral_accel=2.0',         # index is not a whole number
    '1max_lateral_accel=2.0',            # no ':' after the index
    '1:max_lateral_accel',               # no '=' and no value
    '1:max_lateral_acel=2.0',            # not a gap_follow parameter
    '1:enable_deadman=false',            # the deadman is not mirrored, so not a tuning knob
    '1:max_lateral_accel=fast',          # not a number
    '1:max_lateral_accel=nan',           # not finite
    '1:opponent_min_beams=2.5',          # a whole-number parameter
    '1:enable_opponent_awareness=ture',  # a typo in a boolean is refused, not read as false
])
def test_a_bad_car_set_item_is_refused_naming_the_item(item):
    # oracle: spec, the refusal must name the bad item (the message carries it)
    with pytest.raises(SystemExit, match=re.escape(item)):
        egm.parse_car_set([item], cars=2)


def test_a_good_car_set_applies_to_its_car_only_and_is_typed_like_the_yaml():
    out = egm.parse_car_set(['1:max_lateral_accel=2.0', '1:enable_opponent_awareness=true',
                             '1:opponent_min_beams=4', '0:max_speed=2'], cars=2)
    # oracle: spec, typed like gap_follow.yaml: floats, a bool, an int
    assert out == {1: {'max_lateral_accel': 2.0, 'enable_opponent_awareness': True,
                       'opponent_min_beams': 4}, 0: {'max_speed': 2.0}}
    assert out[1]['enable_opponent_awareness'] is True
    assert isinstance(out[1]['opponent_min_beams'], int)


def test_a_later_car_set_item_for_the_same_car_and_name_wins():
    # oracle: closed form, the later item is the one that stays
    assert egm.parse_car_set(['1:max_lateral_accel=2.0', '1:max_lateral_accel=1.5'],
                             cars=2)[1]['max_lateral_accel'] == 1.5


@pytest.mark.parametrize('argv, message', [
    (['--cars', '2', '--car-set', '2:max_lateral_accel=2.0'], '2:max_lateral_accel=2.0'),
    (['--cars', '3', '--swap-starts'], '--swap-starts needs --cars 2'),
])
def test_a_refused_option_stops_the_run_before_any_simulation(argv, message):
    # oracle: spec, the refusal must name the option and come before any run
    with pytest.raises(SystemExit, match=re.escape(message)):
        egm.main(argv)


def test_a_head_to_head_runs_both_cars_at_the_same_top_speed_and_applies_the_car_set():
    params = egm.per_car_params(BASE, {1: {'max_lateral_accel': 2.0}}, cars=2)
    # oracle: closed form, equal top speeds by construction; BASE is the shipped YAML
    assert [p['max_speed'] for p in params] == [3.5, 3.5]          # the same speed, so the comparison is the parameters
    assert [p['max_lateral_accel'] for p in params] == [2.5, 2.0]  # --car-set applied after --set


def test_more_than_two_cars_keep_the_leader_at_seventy_percent():
    params = egm.per_car_params(BASE, {}, cars=3)
    # oracle: closed form, 0.7 x 3.5 (BASE max_speed, the shipped YAML)
    assert [p['max_speed'] for p in params] == [2.45, 3.5, 3.5]    # oracle: closed form, 0.7 x 3.5


def test_speeds_set_each_top_speed_and_clamp_the_limits_under_it():
    params = egm.per_car_params(BASE, {}, cars=2, speeds='1.2,3.5')
    # oracle: closed form, worked out by hand from the synthetic input
    assert [p['max_speed'] for p in params] == [1.2, 3.5]
    assert params[0]['corner_speed'] == pytest.approx(1.2)   # oracle: closed form, min(1.5, 1.2)
    assert params[1]['corner_speed'] == pytest.approx(1.5)   # 1.5 is under 3.5: untouched


def test_a_car_set_top_speed_is_kept_when_speeds_are_not_given():
    params = egm.per_car_params(BASE, {1: {'max_speed': 2.0}}, cars=2)
    # oracle: closed form, worked out by hand from the synthetic input
    assert [p['max_speed'] for p in params] == [3.5, 2.0]
    assert params[1]['corner_speed'] == pytest.approx(1.5)   # oracle: closed form, min(1.5, 2.0)


def test_speeds_and_car_set_cannot_both_set_one_cars_top_speed():
    # oracle: spec, the refusal must name the car and both sources
    with pytest.raises(SystemExit, match='car 1: max_speed is set by both'):
        egm.per_car_params(BASE, {1: {'max_speed': 2.0}}, cars=2, speeds='3.5,3.5')


def test_speeds_need_one_value_per_car():
    # oracle: spec, the refusal must say one value per car is needed
    with pytest.raises(SystemExit, match='one value per car'):
        egm.per_car_params(BASE, {}, cars=2, speeds='3.5')


# --- aggregate over runs -------------------------------------------------------

def test_the_aggregate_counts_wins_contacts_and_the_mean_of_the_finite_gaps():
    outcomes = [
        dict(winner=0, gap_s=1.0, overtakes=[1, 0], contacts=[False, True]),
        dict(winner=1, gap_s=3.0, overtakes=[0, 2], contacts=[False, False]),
        dict(winner=None, gap_s=math.inf, overtakes=[0, 0], contacts=[True, False]),
    ]
    agg = egm.aggregate_head_to_head(outcomes)
    # oracle: closed form, worked out by hand from the synthetic input
    assert agg['runs'] == 3
    assert agg['wins'] == [1, 1]
    assert agg['no_winner'] == 1
    assert agg['gap_runs'] == 2
    assert agg['mean_gap_s'] == pytest.approx((1.0 + 3.0) / 2)   # oracle: closed form, the inf gap is left out
    assert agg['overtakes'] == [1, 2]
    assert agg['contact_runs'] == [1, 1]


def test_the_mean_gap_is_none_when_no_run_had_two_finishers():
    agg = egm.aggregate_head_to_head([dict(winner=0, gap_s=math.inf, overtakes=[0, 0],
                                           contacts=[False, False])])
    # oracle: closed form, no finite gap means no mean
    assert agg['mean_gap_s'] is None
    assert agg['wins'] == [1, 0]


def test_stall_while_other_car_moves():
    # oracle: closed form, 80 ticks * 0.025 s = 2 s, one qualifying run.
    assert egm.count_stalls([[0.0] * 80, [1.0] * 80], TICK) == [1, 0]


def test_both_stopped_is_not_a_stall():
    # oracle: invariant, neither car has a moving opponent.
    assert egm.count_stalls([[0.0] * 80, [0.0] * 80], TICK) == [0, 0]


def test_stall_duration_is_strictly_longer_than_one_second():
    # oracle: closed form, 40 * 0.025 = 1 s; 41 * 0.025 > 1 s.
    assert egm.count_stalls([[0.0] * 40, [1.0] * 40], TICK) == [0, 0]
    assert egm.count_stalls([[0.0] * 41, [1.0] * 41], TICK) == [1, 0]


def test_separate_stalls_and_open_final_stall():
    speeds = _seq((0.0, 80), (1.0, 1), (0.0, 80))
    # oracle: closed form, two 2 s runs separated by a moving tick, last open.
    assert egm.count_stalls([speeds, [1.0] * len(speeds)], TICK) == [2, 0]


def test_other_car_stopping_breaks_a_stall():
    other = _seq((1.0, 20), (0.0, 40), (1.0, 60))
    # oracle: closed form, runs of 0.5 s and 1.5 s; only the latter exceeds 1 s.
    assert egm.count_stalls([[0.0] * 120, other], TICK) == [1, 0]


def test_single_car_and_empty_speed_traces():
    # oracle: invariant, a moving other car is absent in both cases.
    assert egm.count_stalls([[0.0] * 200], TICK) == [0]
    assert egm.count_stalls([[], []], TICK) == [0, 0]


def test_stall_speed_boundary_and_absolute_speed():
    # oracle: spec, |0.05| is moving and |0.0499| is stopped for 2 s.
    assert egm.count_stalls([[-0.0499] * 80, [-0.05] * 80], TICK) == [1, 0]


def test_invalid_speed_traces_are_refused():
    # oracle: spec, unequal lengths and non-finite samples are invalid.
    with pytest.raises(ValueError, match='differ in length'):
        egm.count_stalls([[0.0], []], TICK)
    with pytest.raises(ValueError, match='non-finite'):
        egm.count_stalls([[math.nan], [1.0]], TICK)


def _linear(start, speed, ticks=1201):
    return [start + speed * i * TICK for i in range(ticks)]


@pytest.mark.parametrize('finished', [False, True])
def test_equal_pace_has_no_gap_change(finished):
    times = [60.0 / 2.0, 63.0 / 2.0] if finished else [None, None]
    change = egm.pair_gap_changes([_linear(0.0, 2.0), _linear(-3.0, 2.0)], times, TICK)[0]
    # oracle: closed form, both start and end deficits are 3 / 2 seconds.
    assert change['start_gap_s'] == pytest.approx(3.0 / 2.0, abs=1e-9)  # s
    assert change['end_gap_s'] == pytest.approx(3.0 / 2.0, abs=1e-9)  # s
    assert change['gap_change_s'] == pytest.approx(0.0, abs=1e-9)  # s


def test_faster_trailer_gap_change_and_time_gained():
    changes = egm.pair_gap_changes([_linear(0.0, 2.0), _linear(-3.0, 2.2)],
                                   [60.0 / 2.0, 63.0 / 2.2], TICK)
    start = 3.0 / 2.2  # oracle: closed form, initial deficit / trailer speed, s
    end = 63.0 / 2.2 - 60.0 / 2.0  # oracle: closed form, finish time difference, s
    change = end - start  # oracle: closed form, change in deficit, s
    assert changes[0]['pair'] == (0, 1)  # oracle: spec, pair indices
    assert changes[0]['leader'] == 0  # oracle: input starts at 0 m
    assert changes[0]['trailer'] == 1  # oracle: input starts at -3 m
    assert changes[0]['start_gap_s'] == pytest.approx(start, abs=1e-9)  # s
    assert changes[0]['end_gap_s'] == pytest.approx(end, abs=1e-9)  # s
    assert changes[0]['gap_change_s'] == pytest.approx(change, abs=1e-9)  # s
    gained = egm.time_gained(changes, 2)
    assert gained == pytest.approx([change, -change], abs=1e-9)  # s; oracle: pair allocation
    assert sum(gained) == pytest.approx(0.0, abs=1e-9)  # s; oracle: zero-sum invariant


def test_swapped_start_selects_car_zero_as_trailer():
    change = egm.pair_gap_changes([_linear(-3.0, 2.2), _linear(0.0, 2.0)],
                                  [63.0 / 2.2, 60.0 / 2.0], TICK)[0]
    # oracle: spec, lower starting position trails regardless of index.
    assert change['leader'] == 1
    assert change['trailer'] == 0
    assert change['gap_change_s'] == pytest.approx(63.0 / 2.2 - 30.0 - 3.0 / 2.2, abs=1e-9)  # s


def test_gap_edge_cases_and_none_gain():
    stopped = egm.pair_gap_changes([[0.0, 1.0], [-3.0, -3.0]], [None, None], TICK)[0]
    # oracle: spec, zero trailer progress yields positive infinity for all gaps.
    assert stopped['start_gap_s'] == math.inf
    assert stopped['end_gap_s'] == math.inf
    assert stopped['gap_change_s'] == math.inf
    single = egm.pair_gap_changes([[0.0], [0.0]], [None, None], TICK)[0]
    # oracle: spec, ties choose higher index and fewer than two samples are undefined.
    assert single['trailer'] == 1
    assert single['start_gap_s'] is None
    assert single['end_gap_s'] is None
    assert single['gap_change_s'] is None
    assert egm.time_gained([single], 2) == [0, 0]  # oracle: undefined pair contributes zero


def test_invalid_progress_traces_for_gap_are_refused():
    # oracle: spec, unequal lengths and non-finite samples are invalid.
    with pytest.raises(ValueError, match='differ in length'):
        egm.pair_gap_changes([[0.0], []], [None, None], TICK)
    with pytest.raises(ValueError, match='non-finite'):
        egm.pair_gap_changes([[0.0], [math.nan]], [None, None], TICK)


def test_head_to_head_reports_gap_gain_and_stalls():
    out = egm.head_to_head(_run(_car(progress=_linear(0.0, 2.0), speed=[1.0] * 1201),
                                _car(progress=_linear(-3.0, 2.2), speed=[0.0] * 1201)))
    # oracle: closed form, 30 s elapsed and deficit closes by 0.2 * 30 m.
    change = ((3.0 - 0.2 * 1200 * TICK) / 2.2) - 3.0 / 2.2
    assert out['gap_changes'][0]['gap_change_s'] == pytest.approx(change, abs=1e-9)  # s
    assert out['time_gained_s'] == pytest.approx([change, -change], abs=1e-9)  # s; pair allocation
    assert out['stalls'] == [0, 1]  # oracle: one >1 s stopped run while car 0 moves


def test_aggregate_stalls_and_finite_gains():
    outcomes = [dict(winner=None, gap_s=math.inf, overtakes=[0, 0], contacts=[False, False],
                     stalls=[1, 2], time_gained_s=[2.0, -2.0]),
                dict(winner=None, gap_s=math.inf, overtakes=[0, 0], contacts=[False, False],
                     stalls=[3, 0], time_gained_s=[4.0, -4.0]),
                dict(winner=None, gap_s=math.inf, overtakes=[0, 0], contacts=[False, False],
                     stalls=[0, 1], time_gained_s=[math.inf, -math.inf])]
    agg = egm.aggregate_head_to_head(outcomes)
    # oracle: closed form, per-car sums and means of the two finite runs only.
    assert agg['stalls'] == [1 + 3 + 0, 2 + 0 + 1]
    assert agg['time_gained_runs'] == 2
    assert agg['mean_time_gained_s'] == pytest.approx([(2.0 + 4.0) / 2, (-2.0 - 4.0) / 2], abs=1e-9)  # s


# --- recognition: the other car, seen from this one --------------------------
# Five beams at -0.5, -0.25, 0, 0.25 and 0.5 rad. The sensor sits 0.5 m ahead of
# base_link and the other car's half length is 0.25 m here, so its near side is
# its centre's range from the sensor minus 0.25 m. Tolerance is the brief's 0.5 m.
SCAN_MIN, SCAN_INC, SCAN_N = -0.5, 0.25, 5
LIDAR_X = 0.5    # m, sensor ahead of base_link
HALF = 0.25      # m, the other car's half length
MAX_RANGE = 6.0  # m, opponent_max_range


def _scan(centre, centre_valid=True):
    """A scan whose centre beam (bearing 0) reads ``centre`` and whose other beams read 9 m."""
    ranges = np.full(SCAN_N, 9.0)
    ranges[2] = centre
    valid = np.ones(SCAN_N, dtype=bool)
    valid[2] = centre_valid
    return ranges, valid


def _visible(leader_base, ranges, valid):
    return egm.leader_visible(leader_base, LIDAR_X, ranges, valid, SCAN_MIN, SCAN_INC,
                              MAX_RANGE, HALF)


def test_a_car_ahead_is_visible_when_the_scan_reads_its_near_side():
    # the centre is 3.0 - 0.5 = 2.5 m from the sensor, so the near side is 2.5 - 0.25 = 2.25 m
    # oracle: closed form, the near side worked out by hand from the geometry above
    ranges, valid = _scan(2.25)
    assert _visible((3.0, 0.0), ranges, valid)


def test_side_on_leader_uses_its_width_for_near_side_distance():
    # oracle: closed form, centre 2.5 m from sensor; side-on half-width is
    # 0.1 m, so near side is 2.4 m. A 1.85 m wall is 0.55 m away.
    ranges, valid = _scan(1.85)
    assert not egm.leader_visible((3.0, 0.0), LIDAR_X, ranges, valid,
                                  SCAN_MIN, SCAN_INC, MAX_RANGE, HALF,
                                  leader_half_width=0.1, leader_yaw=math.pi / 2)


@pytest.mark.parametrize('offset, visible', [
    (-0.5, True),           # on the tolerance, below the near side
    (0.5, True),            # on the tolerance, beyond the near side
    (-0.5 - 1e-6, False),   # one step past the tolerance
    (0.5 + 1e-6, False),
])
def test_the_scan_must_agree_with_the_near_side_to_half_a_metre(offset, visible):
    # oracle: spec, the tolerance is 0.5 m (brief: "within 0.5 m of the near-side distance")
    ranges, valid = _scan(2.25 + offset)
    assert _visible((3.0, 0.0), ranges, valid) == visible


def test_a_wall_behind_the_car_is_not_the_car():
    # the scan reads a wall 5.0 m out, 2.75 m beyond the car's near side
    # oracle: closed form, |5.0 - 2.25| > 0.5
    ranges, valid = _scan(5.0)
    assert not _visible((3.0, 0.0), ranges, valid)


def test_something_nearer_than_the_car_hides_it():
    # the scan reads 1.0 m, 1.25 m short of the near side
    # oracle: closed form, |1.0 - 2.25| > 0.5
    ranges, valid = _scan(1.0)
    assert not _visible((3.0, 0.0), ranges, valid)


def test_the_near_side_is_measured_from_the_sensor_not_from_base_link():
    # from the sensor the near side is 2.25 m; from base_link it would be 2.75 m. A 2.0 m
    # reading is 0.25 m from the first and 0.75 m from the second.
    # oracle: closed form, the two distances above
    ranges, valid = _scan(2.0)
    assert _visible((3.0, 0.0), ranges, valid)


def test_a_car_outside_the_scan_field_of_view_is_not_visible():
    # centre at (0.5 + 1.0, 1.0) from the base: bearing atan2(1, 1) = 0.785 rad, past the
    # 0.5 rad edge. Every beam reads that car's near side, so only the field of view can hide it.
    # oracle: closed form, near side = hypot(1, 1) - 0.25 m
    near = math.hypot(1.0, 1.0) - HALF
    ranges = np.full(SCAN_N, near)
    valid = np.ones(SCAN_N, dtype=bool)
    assert not _visible((LIDAR_X + 1.0, 1.0), ranges, valid)


def test_a_car_inside_the_field_of_view_off_the_centre_beam_is_visible():
    # centre 2.5 m from the sensor at bearing 0.4 rad (inside the 0.5 rad edge), near side
    # 2.25 m, and every beam reads that near side
    # oracle: closed form, the centre placed in polar coordinates
    near = 2.5 - HALF
    ranges = np.full(SCAN_N, near)
    valid = np.ones(SCAN_N, dtype=bool)
    base = (LIDAR_X + 2.5 * math.cos(0.4), 2.5 * math.sin(0.4))
    assert _visible(base, ranges, valid)


@pytest.mark.parametrize('x, visible', [
    (6.0, True),           # exactly opponent_max_range from base_link
    (6.0 + 1e-6, False),   # one step past it
])
def test_a_car_beyond_opponent_max_range_is_not_visible(x, visible):
    # centre x metres from base_link, x - 0.5 from the sensor, so its near side is x - 0.75.
    # Every beam reads that near side, so only the range gate can hide the car.
    # oracle: closed form
    near = (x - LIDAR_X) - HALF
    ranges = np.full(SCAN_N, near)
    valid = np.ones(SCAN_N, dtype=bool)
    assert _visible((x, 0.0), ranges, valid) == visible


def test_a_centre_within_its_own_half_length_of_the_sensor_is_not_visible():
    # centre 0.2 m from the sensor, inside its own 0.25 m half length, so the near side would
    # lie behind the sensor (-0.05 m). The scan reads the car's own body at the sensor's minimum.
    # oracle: closed form, 0.2 - 0.25 < 0
    ranges, valid = _scan(0.05)
    assert not _visible((LIDAR_X + 0.2, 0.0), ranges, valid)


def test_an_invalid_centre_beam_means_not_visible():
    # oracle: spec, a dropped beam carries no reading to agree with
    ranges, valid = _scan(2.25, centre_valid=False)
    assert not _visible((3.0, 0.0), ranges, valid)


def test_an_empty_scan_sees_nothing():
    # oracle: spec, no beams, no visibility
    empty = np.array([], dtype=float)
    assert not egm.leader_visible((3.0, 0.0), LIDAR_X, empty, np.array([], dtype=bool),
                                  SCAN_MIN, SCAN_INC, MAX_RANGE, HALF)


def _moving(x, y):
    """A moving_body() entry: base_link x and y, then the velocity and nearest-point x."""
    return (x, y, 0.0, 0.0, x)


def test_a_track_exactly_half_a_metre_from_the_other_car_is_recognised():
    # oracle: closed form, 0.5 m exactly (the inclusive edge of "within 0.5 m")
    assert egm.is_recognised([_moving(3.5, 0.0)], (3.0, 0.0))


def test_a_track_one_step_past_half_a_metre_is_not_recognised():
    # oracle: closed form, 0.5 + 1/64 m
    assert not egm.is_recognised([_moving(3.0 + 0.5 + 1 / 64, 0.0)], (3.0, 0.0))


def test_the_recognition_distance_counts_both_axes():
    # oracle: closed form, 0.6 m to the side is beyond 0.5 m; a distance along x alone would say 0
    assert not egm.is_recognised([_moving(3.0, 0.6)], (3.0, 0.0))


def test_one_track_in_range_is_enough():
    # oracle: closed form, one track is 3.0 m away and another is 0 m away
    assert egm.is_recognised([_moving(6.0, 0.0), _moving(3.0, 0.0)], (3.0, 0.0))


def test_no_moving_track_recognises_nothing():
    # oracle: spec, empty and out-of-range inputs recognise nothing
    assert not egm.is_recognised([], (3.0, 0.0))
    assert not egm.is_recognised([_moving(6.0, 0.0)], (3.0, 0.0))


def test_base_link_is_the_rear_axle_behind_the_gym_state():
    # car state (1.0, 2.0) facing +x; base_link is 0.18 m behind it, at (0.82, 2.0)
    # oracle: closed form, (2.0, 2.0) is 1.18 m ahead of base_link on the same line
    bx, by = egm.in_base_link((1.0, 2.0, 0.0), (2.0, 2.0), 0.18)
    assert bx == pytest.approx(1.18, abs=1e-12)  # m
    assert by == pytest.approx(0.0, abs=1e-12)  # m


def test_base_link_axes_turn_with_the_car():
    # car state at the origin facing +y (yaw pi/2), so base_link is at (0, -0.18).
    # oracle: closed form. (0, 1) is 1.18 m straight ahead. (-1, 0) is 0.18 m ahead and 1 m to
    # the car's left, which is +y in base_link.
    bx, by = egm.in_base_link((0.0, 0.0, math.pi / 2), (0.0, 1.0), 0.18)
    assert bx == pytest.approx(1.18, abs=1e-12)  # m, ahead
    assert by == pytest.approx(0.0, abs=1e-12)  # m, no lateral offset
    lx, ly = egm.in_base_link((0.0, 0.0, math.pi / 2), (-1.0, 0.0), 0.18)
    assert lx == pytest.approx(0.18, abs=1e-12)  # m, ahead
    assert ly == pytest.approx(1.0, abs=1e-12)  # m, to the left


def test_head_to_head_reads_recognition_delays_and_counts_from_the_traces():
    # car 0 is visible on ticks 1-4 and 6-7, and recognised on ticks 3 and 7
    visible = [False, True, True, True, True, False, True, True]
    recognised = [False, False, False, True, False, False, False, True]
    car0 = dict(_car(finish=10.0, progress=[0.0] * 8), visible=visible, recognised=recognised)
    out = egm.head_to_head(_run(car0, _car(finish=11.0, progress=[0.0] * 8)))
    rec0 = out['recognition'][0]
    # oracle: closed form. Episode 1 (ticks 1-4) is first recognised at tick 3, two ticks in.
    # Episode 2 (ticks 6-7) is first recognised at tick 7, one tick in.
    assert rec0['delays'] == [pytest.approx(2 * TICK, abs=1e-12),  # s
                              pytest.approx(TICK, abs=1e-12)]  # s
    assert rec0['visible_ticks'] == 6  # oracle: closed form, ticks 1-4 and 6-7
    assert rec0['recognised_ticks'] == 2  # oracle: closed form, ticks 3 and 7 are both visible and recognised
    assert rec0['fraction'] == pytest.approx(2 / 6, abs=1e-12)
    assert rec0['median_s'] == pytest.approx(1.5 * TICK, abs=1e-12)  # s, the mean of 2 and 1 ticks
    # the other car has no recognition traces, so it has no visible ticks
    assert out['recognition'][1]['visible_ticks'] == 0
    assert out['recognition'][1]['fraction'] is None


def test_the_aggregate_pools_recognition_episodes_and_ticks_across_runs():
    outcomes = [
        dict(winner=0, gap_s=1.0, overtakes=[0, 0], contacts=[False, False],
             recognition=[dict(delays=[0.1, math.inf], visible_ticks=10, recognised_ticks=4),
                          dict(delays=[], visible_ticks=0, recognised_ticks=0)]),
        dict(winner=1, gap_s=2.0, overtakes=[0, 0], contacts=[False, False],
             recognition=[dict(delays=[0.3], visible_ticks=5, recognised_ticks=5),
                          dict(delays=[0.2], visible_ticks=4, recognised_ticks=2)]),
    ]
    agg = egm.aggregate_head_to_head(outcomes)
    car0 = agg['recognition'][0]
    # oracle: closed form. Pooled delays [0.1, inf, 0.3] have median 0.3. Visible ticks 10 + 5
    # = 15 and recognised 4 + 5 = 9.
    assert car0['delays'] == [0.1, math.inf, 0.3]
    assert car0['median_s'] == 0.3  # s
    assert car0['visible_ticks'] == 15
    assert car0['recognised_ticks'] == 9
    assert car0['fraction'] == pytest.approx(9 / 15, abs=1e-12)
    car1 = agg['recognition'][1]
    # oracle: closed form, one 0.2 s episode, and 2 of 4 visible ticks recognised
    assert car1['median_s'] == 0.2  # s
    assert car1['fraction'] == pytest.approx(0.5, abs=1e-12)


def test_aggregate_report_includes_each_pooled_episode_delay(capsys):
    car = dict(_car(progress=[0.0, 0.0]), visible=[True, True],
               recognised=[False, True])
    run = dict(_run(car, _car(progress=[0.0, 0.0])), seed=1, slots=[0, 1])
    egm._report_head_to_head('test', [run], [BASE, BASE])
    aggregate = capsys.readouterr().out.split('recognition, pooled over 1 runs:')[1]
    # oracle: closed form, recognition arrives one 25 ms tick after visibility.
    assert 'delays 0.03s' in aggregate


def test_outcomes_without_recognition_data_contribute_no_episodes():
    # oracle: spec, an outcome from before recognition was measured adds no visible ticks
    agg = egm.aggregate_head_to_head([dict(winner=0, gap_s=1.0, overtakes=[0, 0],
                                           contacts=[False, False])])
    assert agg['recognition'][0]['delays'] == []
    assert agg['recognition'][0]['median_s'] is None
    assert agg['recognition'][0]['fraction'] is None
