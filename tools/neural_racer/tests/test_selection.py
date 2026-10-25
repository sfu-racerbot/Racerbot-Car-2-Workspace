"""Release selection: sequential tests, Nash averaging, veto, retirement.

Runner: python3 -m pytest tools/neural_racer/tests/test_selection.py -v   (no PyTorch needed)
Oracles: closed forms (Wilson interval, rock-paper-scissors' uniform Nash),
and the plan's rules (docs/superpowers/specs/2026-10-23-neural-racer-plan.md, A8 / Phase 9).
"""
import math

import conftest  # noqa: F401
import numpy as np
import pytest

from neural_racer_train import selection as sel


Z = 1.959963984540054        # the 97.5% normal quantile (two-sided 95%), cited, not imported


def test_wilson_closed_form():
    """20/20: lower bound = 1/(1 + z^2/n) (the p=1 case of Wilson's formula)."""
    lo, hi = sel.wilson(20, 20)
    assert lo == pytest.approx(1 / (1 + Z ** 2 / 20), abs=1e-12)
    assert hi == pytest.approx(1.0, abs=1e-12)


def test_sequential_stops_when_clear():
    assert sel.sequential_verdict(20, 20) == 'win'            # settled in 20 races
    assert sel.sequential_verdict(11, 20) == 'continue'
    assert sel.sequential_verdict(0, 20) == 'loss'
    # the 95% boundary itself: 14/20 has Wilson lower bound 0.481 (still open),
    # 15/20 has 0.531 (settled) -- a narrower interval would call 14/20 a win
    assert sel.sequential_verdict(14, 20) == 'continue'
    assert sel.sequential_verdict(15, 20) == 'win'


def test_sequential_caps_at_200():
    assert sel.sequential_verdict(100, 200) == 'inconclusive'
    assert sel.sequential_verdict(99, 199) == 'continue'


def test_nash_rock_paper_scissors():
    rps = np.array([[0.5, 1.0, 0.0], [0.0, 0.5, 1.0], [1.0, 0.0, 0.5]]) - 0.5
    p = sel.maxent_nash(rps)
    assert np.allclose(p, 1 / 3, atol=1e-4)


def test_nash_redundant_copies_dont_change_ranking():
    """Adding 10 copies of a weak agent must not change the order of the others
    (a mean-win-rate ranking would be swayed by them)."""
    base = np.array([[0.5, 0.6, 0.9], [0.4, 0.5, 0.95], [0.1, 0.05, 0.5]])   # A, B, weak C
    order = np.argsort(-sel.nash_average(base))[:2]
    n = 13
    W = np.full((n, n), 0.5)
    W[:3, :3] = base
    for k in range(3, n):               # copies of C
        W[k, :3], W[:3, k] = base[2], base[:, 2]
    order2 = np.argsort(-sel.nash_average(W))[:2]
    assert list(order) == list(order2)


def _results(c_vs_panel):
    return {('cand', p): v for p, v in c_vs_panel.items()}


def test_veto_any_panel_loss():
    rep = sel.select(['cand'], ['gap_follow', 'pure_pursuit'],
                     _results({'gap_follow': (180, 200), 'pure_pursuit': (98, 200)}), {})
    assert rep['winner'] is None and 'pure_pursuit' in ' '.join(rep['vetoed']['cand'])


def test_veto_inconclusive_counts_as_loss():
    rep = sel.select(['cand'], ['gap_follow'], _results({'gap_follow': (100, 200)}), {})
    assert not rep['pass'] and 'inconclusive' in rep['vetoed']['cand'][0]


def test_veto_any_contact():
    rep = sel.select(['cand'], ['gap_follow'], _results({'gap_follow': (190, 200)}), {'cand': 1})
    assert not rep['pass'] and 'contact' in rep['vetoed']['cand'][0]


def test_veto_too_few_races():
    rep = sel.select(['cand'], ['gap_follow'], _results({'gap_follow': (20, 20)}), {})
    assert not rep['pass'] and 'only 20' in rep['vetoed']['cand'][0]


def test_clean_candidate_wins():
    rep = sel.select(['cand'], ['gap_follow', 'pure_pursuit'],
                     _results({'gap_follow': (150, 200), 'pure_pursuit': (140, 200)}), {})
    assert rep['pass'] and rep['winner'] == 'cand'


def test_best_ranked_clean_candidate_is_chosen():
    res = {('a', 'gap_follow'): (180, 200), ('b', 'gap_follow'): (150, 200), ('a', 'b'): (150, 200)}
    rep = sel.select(['a', 'b'], ['gap_follow'], res, {'a': 1})
    assert rep['winner'] == 'b'                   # a ranks higher but has a contact


def test_panel_retirement_and_return():
    r = sel.Retirement()
    for k in range(10):
        r.record(k, 0.96)
    assert r.retired and not r.should_check(11) and r.should_check(19)
    r.record(19, 0.90)                            # one failed re-check
    assert not r.retired and r.should_check(20)


def test_retirement_needs_strictly_above_95():
    r = sel.Retirement()
    for k in range(10):
        r.record(k, 0.95)
    assert not r.retired
