"""Tests for tools/f1tenth_sim/law_compare.py -- the promotion rule. Pure
Python, runs unsourced:

    python3 -m pytest tools/f1tenth_sim/tests/ -v

Rows are built by hand; every expected verdict follows from the three
conditions in spec 01 (safety, strictly-better-on-one, within-5%-on-the-other).
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import law_compare  # noqa: E402


def _row(track='Spielberg', seed=1, laps=1.0, collision=False,
         max_ct=0.2, mean_ct=0.05, lap_time=50.0, law_fail=0, latched=False):
    return {
        'track': track, 'seed': seed, 'raceline': 'shipped',
        'laps': laps, 'collision': collision,
        'max_cross_track_m': max_ct, 'mean_cross_track_m': mean_ct,
        'lap_time_s': lap_time, 'law_fail_steps': law_fail,
        'mpc_latched': latched,
    }


def _baseline(seeds=(1, 2, 3)):
    return [_row(seed=s) for s in seeds]


def test_better_cross_track_within_tolerance_promotes():
    base = _baseline()
    # 20% better cross-track, lap time +4% (within the 5% tolerance).
    cand = [_row(seed=s, mean_ct=0.04, lap_time=52.0) for s in (1, 2, 3)]
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'PROMOTE', v.reasons


def test_lap_time_six_percent_worse_rejects_naming_condition_3():
    base = _baseline()
    cand = [_row(seed=s, mean_ct=0.04, lap_time=53.0) for s in (1, 2, 3)]
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'REJECT'
    assert any('condition 3' in r and 'lap time' in r for r in v.reasons)


def test_exactly_five_percent_worse_promotes():
    base = _baseline()
    # +5.0% lap time exactly: 50.0 * 1.05 = 52.5. Boundary is inclusive.
    cand = [_row(seed=s, mean_ct=0.04, lap_time=52.5) for s in (1, 2, 3)]
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'PROMOTE', v.reasons


def test_just_over_five_percent_worse_rejects():
    base = _baseline()
    # 52.5000...1% : strictly above the inclusive boundary.
    cand = [_row(seed=s, mean_ct=0.04, lap_time=52.5 * 1.000001)
            for s in (1, 2, 3)]
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'REJECT'
    assert any('condition 3' in r for r in v.reasons)


def test_equal_on_both_rejects_condition_2_is_strict():
    base = _baseline()
    cand = [_row(seed=s) for s in (1, 2, 3)]   # identical
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'REJECT'
    assert any('condition 2' in r for r in v.reasons)


def test_better_lap_time_promotes():
    base = _baseline()
    # Faster lap, cross-track equal (within tolerance, not better).
    cand = [_row(seed=s, lap_time=48.0) for s in (1, 2, 3)]
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'PROMOTE', v.reasons


def test_one_collision_in_fifteen_rejects_naming_the_run():
    base = [_row(seed=s) for s in range(15)]
    cand = [_row(seed=s, mean_ct=0.03) for s in range(15)]
    cand[7]['collision'] = True   # one bad run out of 15
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'REJECT'
    assert any('seed=7' in r and 'collision' in r for r in v.reasons)


def test_candidate_without_a_lap_rejects():
    base = _baseline()
    cand = [_row(seed=s, mean_ct=0.03) for s in (1, 2, 3)]
    cand[1]['laps'] = 0.0
    cand[1]['lap_time_s'] = None
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'REJECT'
    assert any('seed=2' in r and 'laps' in r for r in v.reasons)


def test_baseline_failing_safety_invalidates_the_comparison():
    base = _baseline()
    base[0]['collision'] = True
    cand = [_row(seed=s, mean_ct=0.03) for s in (1, 2, 3)]
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'INVALID'
    assert any('baseline' in r for r in v.reasons)


def test_mismatched_run_sets_raise():
    base = _baseline(seeds=(1, 2, 3))
    cand = [_row(seed=s) for s in (1, 2, 4)]   # different seed set
    with pytest.raises(ValueError, match='unmatched'):
        law_compare.verdict(base, cand)


def test_nan_metric_raises():
    base = _baseline()
    cand = [_row(seed=s) for s in (1, 2, 3)]
    cand[0]['mean_cross_track_m'] = math.nan
    with pytest.raises(ValueError, match='non-finite'):
        law_compare.verdict(base, cand)


def test_law_fail_steps_reject():
    base = _baseline()
    cand = [_row(seed=s, mean_ct=0.03) for s in (1, 2, 3)]
    cand[0]['law_fail_steps'] = 2
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'REJECT'
    assert any('law_fail_steps' in r for r in v.reasons)


def test_mpc_latch_rejects():
    base = _baseline()
    cand = [_row(seed=s, mean_ct=0.03) for s in (1, 2, 3)]
    cand[2]['mpc_latched'] = True
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'REJECT'
    assert any('mpc_latched' in r for r in v.reasons)


def test_every_failed_condition_is_listed_not_just_the_first():
    base = _baseline()
    # Worse on cross-track AND a collision AND a latch: three failures.
    cand = [_row(seed=s, mean_ct=0.5) for s in (1, 2, 3)]
    cand[0]['collision'] = True
    cand[1]['mpc_latched'] = True
    v = law_compare.verdict(base, cand)
    assert v.verdict == 'REJECT'
    joined = ' '.join(v.reasons)
    assert 'collision' in joined and 'mpc_latched' in joined
    assert any('condition 2' in r for r in v.reasons)
