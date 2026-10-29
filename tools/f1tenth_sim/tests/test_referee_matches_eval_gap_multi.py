"""The ROS-free race referee scores a race exactly as eval_gap_multi does.

    python3 -m pytest tools/f1tenth_sim/tests/test_referee_matches_eval_gap_multi.py -v

The referee takes positions, not progress, so both scorers are fed the same
positions: a square loop 8 m round, sampled every 0.02 m, with each car on a
vertex every tick. eval_gap_multi turns those positions into progress with
its own _Progress; the referee with its LapCounter. The two traces must agree,
and so must every verdict built from them. Each scenario also carries its
closed-form expectation (worked out by hand from the arc lengths), so a
change that breaks both scorers the same way still fails.
"""

import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'src', 'racerbot_sim'))
sys.path.insert(0, os.path.join(ROOT, 'tools', 'f1tenth_sim'))
from racerbot_sim.referee import Referee  # noqa: E402
import eval_gap_multi as egm  # noqa: E402

DT = 0.1
SPACING = 0.02
CORNERS = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
LOOP = np.array([(x0 + (x1 - x0) * k / 100.0, y0 + (y1 - y0) * k / 100.0)
                 for (x0, y0), (x1, y1) in zip(CORNERS, CORNERS[1:] + CORNERS[:1])
                 for k in range(100)])
REFERENCE = np.column_stack((LOOP[:, 0], LOOP[:, 1]))


def _at(arc_m):
    """The loop vertex at arc length arc_m from the line (metres, wrapped round the loop)."""
    return LOOP[int(round(arc_m / SPACING)) % len(LOOP)]


def _eval_run(starts, arcs, speeds, laps_required):
    """eval_gap_multi.run_multi's per-tick bookkeeping, fed the same positions."""
    poses = [(float(_at(start)[0]), float(_at(start)[1]), 0.0) for start in starts]
    progress = egm.line_up_progress(REFERENCE, poses, False)
    cars = [dict(progress_m=[], speed_mps=[], finish_time_s=None, laps=0.0,
                 wall_contact=False, car_contact=False) for _ in starts]
    for n in range(len(arcs[0])):
        for car in range(len(starts)):
            laps = progress[car].update(_at(arcs[car][n]))
            cars[car]['progress_m'].append(float(progress[car].position))
            cars[car]['speed_mps'].append(float(speeds[car][n]))
            if cars[car]['finish_time_s'] is None and laps >= laps_required:
                cars[car]['finish_time_s'] = round((n + 1) * DT, 2)
    for car in range(len(starts)):
        cars[car]['laps'] = progress[car].laps
    return dict(cars=cars, dt_s=DT)


def _referee_run(starts, arcs, speeds, laps_required):
    referee = Referee(LOOP, [_at(start) for start in starts], laps_required, DT)
    for n in range(len(arcs[0])):
        referee.tick((n + 1) * DT, [_at(arcs[car][n]) for car in range(len(starts))],
                     [speeds[car][n] for car in range(len(starts))],
                     [False] * len(starts), False)
    return referee


def _scenarios():
    ticks = range(1, 101)
    yield dict(
        name='equal pace: car 2 follows 3 m behind all the way',
        starts=[0.0, -3.0], laps=1.99,
        arcs=[[0.2 * n for n in ticks], [-3.0 + 0.2 * n for n in ticks]],
        speeds=[[2.0] * 100, [2.0] * 100],
        # oracle: closed form. Gap stays 3 m, so nobody overtakes or stalls; car 1 finishes first.
        overtakes=[0, 0], stalls=[0, 0], finish_order=[0, 1], winner=0)
    yield dict(
        name='faster trailer passes: car 2 at 2.2 m/s from 0.6 m behind',
        starts=[0.0, -0.6], laps=1.99,
        arcs=[[0.2 * n for n in ticks], [-0.6 + 0.22 * n for n in ticks]],
        speeds=[[2.0] * 100, [2.2] * 100],
        # oracle: closed form. 0.22n - 0.6 > 0.2n from n = 31 on: car 2 passes once and stays ahead,
        # so it crosses 1.99 laps first (n = 76, against car 1's n = 80).
        overtakes=[0, 1], stalls=[0, 0], finish_order=[1, 0], winner=1)
    stall_ticks = range(1, 131)

    def car2_progress(n):
        if n <= 20:
            return -3.0 + 0.2 * n
        if n <= 50:
            return 1.0  # stopped from tick 21 to tick 50: 30 ticks, 3 s
        return 1.0 + 0.2 * (n - 50)

    yield dict(
        name='stall: car 2 stops for 3 s while car 1 drives on',
        starts=[0.0, -3.0], laps=1.99,
        arcs=[[0.2 * n for n in stall_ticks], [car2_progress(n) for n in stall_ticks]],
        speeds=[[2.0] * 130, [0.0 if 21 <= n <= 50 else 2.0 for n in stall_ticks]],
        # oracle: closed form. A 30-tick stop is longer than 1.0 s, so car 2 stalls once; the gap
        # never closes, so there is no pass; car 1 finishes at tick 80, car 2 at tick 125.
        overtakes=[0, 0], stalls=[0, 1], finish_order=[0, 1], winner=0)


@pytest.mark.parametrize('scenario', list(_scenarios()), ids=lambda s: s['name'][:24])
def test_referee_and_eval_gap_multi_score_the_same_positions_the_same_way(scenario):
    arcs, speeds = scenario['arcs'], scenario['speeds']
    referee = _referee_run(scenario['starts'], arcs, speeds, scenario['laps'])
    result = referee.result()
    run = _eval_run(scenario['starts'], arcs, speeds, scenario['laps'])
    outcome = egm.head_to_head(run)

    # The progress each scorer built from the positions is the same trace, tick by tick.
    for car in range(2):
        assert referee.progress_m[car] == pytest.approx(run['cars'][car]['progress_m'], abs=1e-9)  # metres

    # The verdicts: the closed forms first, then parity with eval_gap_multi.
    assert outcome['overtakes'] == scenario['overtakes']
    assert outcome['stalls'] == scenario['stalls']
    assert outcome['finish_order'] == scenario['finish_order']
    assert outcome['winner'] == scenario['winner']
    assert result['overtakes'] == outcome['overtakes']
    assert result['stalls'] == outcome['stalls']
    assert [number - 1 for number in result['finish_order']] == outcome['finish_order']
    assert (None if result['winner'] is None else result['winner'] - 1) == outcome['winner']
    assert [any(flags.values()) for flags in result['contacts']] == outcome['contacts']

    if math.isinf(outcome['gap_s']):
        assert result['gap_s'] is None  # strict JSON representation of undefined gap
    else:
        assert result['gap_s'] == pytest.approx(outcome['gap_s'], abs=1e-9)  # s
    assert len(result['gap_changes']) == len(outcome['gap_changes'])
    for ours, theirs in zip(result['gap_changes'], outcome['gap_changes']):
        assert tuple(ours['pair']) == theirs['pair']
        assert ours['leader'] == theirs['leader']
        assert ours['trailer'] == theirs['trailer']
        for field in ('start_gap_s', 'end_gap_s', 'gap_change_s'):
            assert ours[field] == pytest.approx(theirs[field], abs=1e-9)  # s
    assert result['time_gained_s'] == pytest.approx(outcome['time_gained_s'], abs=1e-9)  # s
