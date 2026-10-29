"""The ROS-free race referee (racerbot_sim/referee.py).

    python3 -m pytest src/racerbot_sim/test/test_referee.py -v

The scenarios run on a square loop 8 m round, sampled every 0.02 m, so a car
at speed v moves v * 0.1 s of loop per tick (0.1 s is the referee tick used
throughout). The expected numbers are closed forms of distance and speed,
worked out in the comments, not copied from a run.

R6 step 3 mutation audit (2026-10-28): reverse ignored, contact termination
removed, both-stopped stalls counted, partial updates before validation,
contact-tick finishes allowed, winner contact filter removed, and JSON
sanitization removed all failed their named regression tests. LapCounter,
Referee, overtakes and stalls stubs failed all 36 cases in this file;
the Referee stub also failed all three position-fed parity scenarios.
Numeric mutations caught: finish comparison flipped, reverse sign ignored,
perimeter +10%, XY swapped, update returning its input, and arc wrap removed.

R6 review round 1 audit (2026-10-28): loss accounting disabled failed all four
step-gap tests and the node clock test; joining persistence windows across
missing steps failed the stall and overtake tests; counting received samples
as elapsed time failed the 1.5 s gap oracle (reported 0.5 s); accepting duplicate
steps failed the unchanged-result invariant. All mutations were restored.
"""

import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_sim import referee as referee_module  # noqa: E402
from racerbot_sim.referee import LapCounter, Referee, count_overtakes, count_stalls  # noqa: E402

DT = 0.1
SPACING = 0.02  # metres between loop vertices: vertex i sits at arc length 0.02 * i
PERIMETER = 8.0


def _square_loop():
    corners = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    points = []
    for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
        for step in range(100):
            points.append((x0 + (x1 - x0) * step / 100.0, y0 + (y1 - y0) * step / 100.0))
    return np.array(points)


LOOP = _square_loop()


def _at(arc_m):
    """The loop vertex at arc length arc_m from the line, wrapped round the loop."""
    return LOOP[int(round(arc_m / SPACING)) % len(LOOP)]


def _tick(referee, index, speeds=(2.0, 2.0), wall=(False, False), car=False):
    """Car 1 at 2.0 m/s from the line; car 2 at 2.0 m/s, 3 m behind it."""
    t = index * DT
    referee.tick(t, [_at(2.0 * t), _at(-3.0 + 2.0 * t)], list(speeds), list(wall), car)


def _race(starts, speeds, laps_required, max_index):
    """Tick the referee with cars at constant speeds until it finishes or runs out of ticks."""
    referee = Referee(LOOP, [_at(start) for start in starts], laps_required, DT)
    for index in range(max_index + 1):
        t = index * DT
        positions = [_at(start + speed * t) for start, speed in zip(starts, speeds)]
        referee.tick(t, positions, list(speeds), [False] * len(starts), False)
        if referee.finished:
            break
    return referee


# --- lap counting -----------------------------------------------------------

def test_one_forward_revolution_is_one_lap():
    counter = LapCounter(LOOP, _at(0.0))
    laps = [counter.update(LOOP[index % len(LOOP)]) for index in range(1, len(LOOP) + 1)]
    assert laps[99] == pytest.approx(0.25, abs=1e-9)  # oracle: closed form, 2 m of an 8 m loop
    assert laps[-1] == pytest.approx(1.0, abs=1e-9)  # oracle: closed form, one full perimeter


@pytest.mark.parametrize('reverse', [False, True])
def test_circle_revolution(reverse):
    angles = np.linspace(0, 2 * math.pi, 2000, endpoint=False)
    circle = np.column_stack((5 * np.cos(angles), 5 * np.sin(angles)))
    counter = LapCounter(circle, circle[0], reverse=reverse)
    direction = -1 if reverse else 1
    for index in range(1, len(circle) + 1):
        laps = counter.update(circle[(direction * index) % len(circle)])
    # oracle: closed form, one complete revolution / one circumference.
    assert laps == pytest.approx(1.0, abs=1e-3)  # laps


def test_driving_the_loop_backwards_is_minus_one_lap_unless_reversed():
    # From the forward counter's view a car driving the loop the wrong way round has
    # gone one lap backwards (-1). The reversed counter scores the same motion as +1.
    backwards = [LOOP[(-index) % len(LOOP)] for index in range(1, len(LOOP) + 1)]
    forward = LapCounter(LOOP, _at(0.0))
    reverse = LapCounter(LOOP, _at(0.0), reverse=True)
    assert [forward.update(point) for point in backwards][-1] == pytest.approx(-1.0, abs=1e-9)  # oracle: closed form
    assert [reverse.update(point) for point in backwards][-1] == pytest.approx(1.0, abs=1e-9)  # oracle: closed form


def test_a_car_three_metres_behind_the_line_starts_at_minus_three():
    # oracle: closed form. Arc 5.0 m on an 8 m loop is 3 m short of the line: position -3 m.
    assert LapCounter(LOOP, _at(5.0)).position_m == pytest.approx(-3.0, abs=1e-9)
    # Reversed, the car is 3 m short of the line on its own side: arc 3.0 m, counted the other way.
    assert LapCounter(LOOP, _at(3.0), reverse=True).position_m == pytest.approx(-3.0, abs=1e-9)


@pytest.mark.parametrize('position, match', [
    (np.array([math.nan, 0.0]), 'two finite values'),
    (np.array([0.0]), 'two finite values'),
])
def test_lap_counter_refuses_a_malformed_position(position, match):
    counter = LapCounter(LOOP, _at(0.0))
    with pytest.raises(ValueError, match=match):
        counter.update(position)


def test_lap_counter_refuses_a_centreline_too_short_to_drive():
    with pytest.raises(ValueError, match='at least two'):
        LapCounter(LOOP[:1], _at(0.0))


# --- finishing, contact and winners -----------------------------------------

def test_two_constant_speed_cars_finish_at_distance_over_speed():
    # oracle: closed form. 0.99 laps is 7.92 m, which car 1 covers at 2 m/s in 3.96 s; the
    # referee reports the first tick at or past the line, 4.0 s. Car 2 covers 10.92 m in
    # 5.46 s, reported at 5.5 s. Car 2 is 3 m behind, so its gap is 3 m / 2 m/s = 1.5 s.
    referee = _race([0.0, -3.0], [2.0, 2.0], 0.99, max_index=60)
    result = referee.result()
    assert result['finish_time_s'] == [pytest.approx(4.0, abs=1e-9), pytest.approx(5.5, abs=1e-9)]
    assert result['finish_order'] == [1, 2]
    assert result['gap_s'] == pytest.approx(1.5, abs=1e-9)  # oracle: closed form, 3 m / 2 m/s
    assert referee.finished


def test_finish_order_puts_the_faster_car_first_when_it_starts_ahead():
    # oracle: closed form. Car 2 starts on the line at 2.4 m/s: 7.76 m (0.97 laps) takes
    # 3.23 s, reported at 3.3 s. Car 1 starts 3 m behind at 2.0 m/s: 10.76 m takes 5.38 s, 5.4 s.
    referee = _race([-3.0, 0.0], [2.0, 2.4], 0.97, max_index=80)
    result = referee.result()
    assert result['finish_order'] == [2, 1]
    assert result['finish_time_s'] == [pytest.approx(5.4, abs=1e-9), pytest.approx(3.3, abs=1e-9)]


@pytest.mark.parametrize('kind', ['wall', 'car'])
def test_a_contact_ends_the_race_and_the_car_that_touched_cannot_win(kind):
    referee = Referee(LOOP, [_at(0.0), _at(-3.0)], 0.99, DT)
    for index in range(1, 41):  # car 1 reaches the line at tick 40
        _tick(referee, index)
    assert not referee.finished  # car 2 is still out on the course
    touched = (True, False) if kind == 'wall' else (False, False)
    _tick(referee, 41, wall=touched, car=(kind == 'car'))
    assert referee.finished  # a contact ends the race
    result = referee.result()
    assert result['finish_time_s'][0] == pytest.approx(4.0, abs=1e-9)  # car 1 did finish first
    assert result['winner'] is None  # it had touched something, and car 2 never finished
    if kind == 'wall':
        assert result['contacts'] == [{'wall': True, 'car': False}, {'wall': False, 'car': False}]
    else:
        assert result['contacts'] == [{'wall': False, 'car': True}, {'wall': False, 'car': True}]


def test_a_contact_on_the_line_crossing_tick_records_no_finish():
    # oracle: spec, eval_gap_multi.run_multi breaks on a contact before it checks for the
    # finish, so a car that crosses the line on a contact tick has no finish time.
    referee = Referee(LOOP, [_at(0.0), _at(-3.0)], 0.99, DT)
    for index in range(1, 40):
        _tick(referee, index)
    _tick(referee, 40, wall=(True, False))  # car 1 crosses the line and touches the wall
    result = referee.result()
    assert referee.finished
    assert result['finish_time_s'] == [None, None]
    assert result['laps'][0] == pytest.approx(1.0, abs=1e-9)  # oracle: closed form, 8 m of 8 m
    assert result['winner'] is None


def test_format_table_puts_the_places_in_finish_order_and_names_the_winner():
    referee = _race([-3.0, 0.0], [2.0, 2.4], 0.97, max_index=80)
    lines = referee_module.format_table(referee.result()).splitlines()
    assert lines[1].split()[:2] == ['1', '2']  # place 1 is car 2
    assert lines[2].split()[:2] == ['2', '1']  # place 2 is car 1
    assert any('winner: car 2' in line for line in lines)


# --- stalls ------------------------------------------------------------------

def test_two_seconds_stopped_while_the_other_moves_is_one_stall():
    referee = Referee(LOOP, [_at(0.0), _at(5.0)], 10.0, DT)
    for index in range(21):  # samples at 0.0 .. 2.0 s
        referee.tick(index * DT, [_at(0.0), _at(5.0 + 0.1 * index)], [0.0, 1.0], [False, False], False)
    # oracle: spec, a stall is a run longer than 1.0 s (11 samples of 0.1 s); here 21 samples stopped.
    assert referee.result()['stalls'] == [1, 0]


def test_stopped_while_the_other_is_also_stopped_is_no_stall():
    referee = Referee(LOOP, [_at(0.0), _at(5.0)], 10.0, DT)
    for index in range(21):
        referee.tick(index * DT, [_at(0.0), _at(5.0)], [0.0, 0.0], [False, False], False)
    assert referee.result()['stalls'] == [0, 0]  # oracle: spec, a stall needs another car moving


@pytest.mark.parametrize('stopped_samples, stopped_speed, expected', [
    (10, 0.0, 0),     # exactly 1.0 s is not longer than 1.0 s
    (11, 0.0, 1),     # 1.1 s is
    (30, 0.05, 0),    # 0.05 m/s counts as moving (eval_gap_multi: abs(v) >= 0.05)
    (30, 0.0499, 1),  # just under it counts as stopped
])
def test_stall_boundaries(stopped_samples, stopped_speed, expected):
    stopped = [stopped_speed] * stopped_samples
    moving = [1.0] * stopped_samples
    assert count_stalls([stopped, moving], DT) == [expected, 0]


# --- gap changes -------------------------------------------------------------

def test_a_trailer_ten_percent_faster_closes_the_gap_by_the_closed_form():
    # The same closed form as tools/f1tenth_sim/tests/test_eval_gap_multi_head_to_head.py.
    referee = Referee(LOOP, [_at(0.0), _at(-3.0)], 0.99, DT)
    for index in range(61):  # the first sample is the start pose, at t = 0
        t = index * DT
        referee.tick(t, [_at(2.0 * t), _at(-3.0 + 2.2 * t)], [2.0, 2.2], [False, False], False)
        if referee.finished:
            break
    result = referee.result()
    pair = result['gap_changes'][0]
    start = 3.0 / 2.2  # oracle: closed form, initial 3 m deficit / trailer speed 2.2 m/s
    end = 5.0 - 4.0  # oracle: closed form, the trailer crosses at 5.0 s, the leader at 4.0 s
    assert (pair['leader'], pair['trailer']) == (0, 1)  # eval_gap_multi pair indices
    assert pair['start_gap_s'] == pytest.approx(start, abs=1e-9)
    assert pair['end_gap_s'] == pytest.approx(end, abs=1e-9)
    assert pair['gap_change_s'] == pytest.approx(end - start, abs=1e-9)  # negative: the trailer closed in
    assert result['time_gained_s'] == pytest.approx([end - start, start - end], abs=1e-9)  # zero-sum


def test_an_empty_run_has_no_overtakes():
    assert count_overtakes([[], []], DT) == [0, 0]  # oracle: spec, nothing happened


@pytest.mark.parametrize('pass_samples, expected', [(4, [0, 0]), (5, [0, 1])])
def test_a_pass_counts_only_once_its_new_order_has_held_for_half_a_second(pass_samples, expected):
    # oracle: spec, a change of order counts once the new order has held 0.5 s = 5 samples.
    # Car 2 starts 0.1 m behind car 1, is 0.1 m ahead from sample 10, and stays ahead.
    ahead_from = 10
    car1 = [0.2 * k for k in range(ahead_from + pass_samples)]
    car2 = [value - 0.1 if k < ahead_from else value + 0.1 for k, value in enumerate(car1)]
    assert count_overtakes([car1, car2], DT) == expected


# --- validation: refused ticks change nothing --------------------------------

def test_a_tick_with_the_wrong_number_of_cars_is_refused_and_changes_nothing():
    referee = Referee(LOOP, [_at(0.0), _at(-3.0)], 0.99, DT)
    with pytest.raises(ValueError, match='number of cars'):
        referee.tick(DT, [_at(0.2)], [2.0, 2.0], [False, False], False)
    with pytest.raises(ValueError, match='number of cars'):
        referee.tick(DT, [_at(0.2), _at(-2.8)], [2.0], [False, False], False)
    with pytest.raises(ValueError, match='wall contact'):
        referee.tick(DT, [_at(0.2), _at(-2.8)], [2.0, 2.0], [False], False)
    assert referee.progress_m == [[], []]
    assert referee.sim_time_s == 0.0


@pytest.mark.parametrize('bad_position,bad_speed', [
    (_at(-2.8), math.nan), ([math.inf, 0.0], 2.0),
])
def test_a_bad_speed_for_the_second_car_leaves_the_first_car_unmoved(bad_position, bad_speed):
    # Car 1's position is valid and car 2's speed is not: the whole tick is refused, so
    # car 1's progress is not written either.
    referee = Referee(LOOP, [_at(0.0), _at(-3.0)], 0.99, DT)
    before = referee.result()
    with pytest.raises(ValueError, match='finite'):
        referee.tick(DT, [_at(0.2), bad_position], [2.0, bad_speed], [False, False], False)
    assert referee.result()['laps'][0] == pytest.approx(0.0, abs=1e-9)  # oracle: closed form, still on the line
    assert referee.result() == before  # oracle: invariant, rejection is atomic
    assert referee.progress_m == [[], []]


@pytest.mark.parametrize('build, match', [
    (lambda: Referee(LOOP, [], 0.99, DT), 'at least one car'),
    (lambda: Referee(LOOP, [_at(0.0)], 0.0, DT), 'laps_required'),
    (lambda: Referee(LOOP, [_at(0.0)], 0.99, 0.0), 'dt_s'),
    (lambda: Referee(LOOP, [_at(0.0)], 0.99, DT).tick(DT, [[math.nan, 0.0]], [2.0], [False], False), 'finite'),
    (lambda: Referee(LOOP, [_at(0.0)], 0.99, DT).tick(DT, [_at(0.2)], [math.inf], [False], False), 'non-finite speed'),
    (lambda: Referee(LOOP, [_at(0.0)], 0.99, DT).tick(math.nan, [_at(0.2)], [2.0], [False], False), 'time must be finite'),
    (lambda: Referee(LOOP, [_at(0.0)], 0.99, DT).tick(DT, [_at(0.2)], [2.0], [False], 'yes'), 'boolean'),
])
def test_referee_refuses_bad_construction_and_inputs(build, match):
    with pytest.raises(ValueError, match=match):
        build()


def test_ticks_must_advance_in_time():
    referee = Referee(LOOP, [_at(0.0)], 0.99, DT)
    referee.tick(2 * DT, [_at(0.4)], [2.0], [False], False)
    with pytest.raises(ValueError, match='increase'):
        referee.tick(DT, [_at(0.2)], [2.0], [False], False)
    with pytest.raises(ValueError, match='increase'):
        referee.tick(2 * DT, [_at(0.4)], [2.0], [False], False)  # a repeat is not a new step


def test_missing_steps_advance_elapsed_time_and_duplicates_change_nothing():
    referee = Referee(LOOP, [_at(0), _at(-3)], 10.0, DT)
    referee.tick(DT, [_at(0.2), _at(-2.8)], [2, 2], [False, False], False, steps=1)
    first_time = referee.sim_time_s
    referee.tick(4 * DT, [_at(0.8), _at(-2.2)], [2, 2], [False, False], False, steps=4)
    result = referee.result()
    # oracle: closed form, steps 2 and 3 were lost; elapsed interval is 3 * dt.
    assert result['missed_steps'] == 2
    assert result['steps'] == 4
    assert referee.sim_time_s - first_time == pytest.approx(3 * DT, abs=1e-12)  # seconds
    # Mean trailer speed uses all 0.3 s, not just one received-sample interval.
    assert result['gap_changes'][0]['start_gap_s'] == pytest.approx(3 / 2, abs=1e-9)  # seconds
    referee.tick(5 * DT, [_at(1), _at(-2)], [2, 2], [True, True], True, steps=4)
    referee.tick(6 * DT, [_at(1), _at(-2)], [2, 2], [True, True], True, steps=3)
    assert referee.result() == result  # oracle: invariant, duplicate/old step is ignored
    assert [len(trace) for trace in referee.progress_m] == [2, 2]


def test_stall_persistence_does_not_bridge_missing_steps():
    referee = Referee(LOOP, [_at(0), _at(-3)], 10.0, DT)
    for step in [*range(1, 7), *range(9, 15)]:
        referee.tick(step * DT, [_at(0), _at(-3 + step * DT)],
                     [0, 1], [False, False], False, steps=step)
    # oracle: spec, two 0.6 s observed runs cannot prove a >1 s continuous stall.
    assert referee.result()['stalls'] == [0, 0]
    assert referee.result()['missed_steps'] == 2


def test_overtake_persistence_does_not_bridge_missing_steps():
    referee = Referee(LOOP, [_at(0), _at(-0.2)], 10.0, DT)
    for step in [*range(1, 6), *range(8, 13)]:
        trailer = -0.2 if step < 8 else 0.2
        referee.tick(step * DT, [_at(0), _at(trailer)],
                     [1, 1], [False, False], False, steps=step)
    # oracle: invariant, an order change across an unobserved interval is not a scored pass.
    assert referee.result()['overtakes'] == [0, 0]
    assert referee.result()['missed_steps'] == 2


def test_initial_missing_steps_are_counted_from_first_permitted_step():
    referee = Referee(LOOP, [_at(0)], 10.0, DT)
    referee.tick(5 * DT, [_at(0)], [0], [False], False, steps=5)
    assert referee.result()['missed_steps'] == 4  # oracle: closed form, steps 1..4 absent
    assert referee.sim_time_s == pytest.approx(5 * DT, abs=1e-12)  # seconds


# --- output -----------------------------------------------------------------

def test_result_is_strict_json_even_when_the_trailer_never_moves():
    referee = Referee(LOOP, [_at(0.0), _at(-3.0)], 0.99, DT)
    for index in range(21):
        referee.tick(index * DT, [_at(index * 0.2), _at(-3.0)], [2.0, 0.0], [False, False], False)
    payload = referee.result()
    # oracle: spec ruling 1: undefined/infinite time metrics serialize as null.
    assert payload['gap_s'] is None
    assert payload['gap_changes'][0]['gap_change_s'] is None
    assert payload['time_gained_s'] == [None, None]
    assert json.loads(json.dumps(payload, allow_nan=False)) == payload
