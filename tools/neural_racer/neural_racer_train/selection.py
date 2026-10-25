"""Choosing the release (plan A8, Phase 9): never "the latest checkpoint".

    python -m neural_racer_train.selection --crossplay crossplay.jsonl --out selection_report.json

Pieces, each tested against a closed form (tests/test_select.py):

  sequential test   a matchup stops as soon as the 95% Wilson interval on its
                    win rate sits wholly above or below the threshold; 200 races max,
                    and a matchup still straddling it then is "inconclusive",
                    which the veto treats as a loss
  Nash averaging    rank candidates against the maximum-entropy Nash mixture of
                    the antisymmetric payoff A = winrate - 0.5 (Balduzzi et al.
                    2018): redundant copies cannot inflate anyone's score
  veto              under 50% against any panel member, an inconclusive panel
                    matchup, or any at-fault contact -> out, whatever the rank
  retirement        a panel member beaten above 95% for 10 snapshots in a row is
                    checked every 10th snapshot; one failed check restores it
"""
import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import linprog, minimize

Z95 = 1.959963984540054
MAX_RACES = 200
BATCH = 10


def wilson(wins: int, n: int, z: float = Z95):
    if n == 0:
        return 0.0, 1.0
    p = wins / n
    den = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return centre - half, centre + half


def sequential_verdict(wins: int, n: int, threshold: float = 0.5, max_races: int = MAX_RACES) -> str:
    """'win' / 'loss' once the interval clears the threshold; 'continue' before
    the cap; 'inconclusive' at the cap."""
    lo, hi = wilson(wins, n)
    if n > 0 and lo > threshold:
        return 'win'
    if n > 0 and hi < threshold:
        return 'loss'
    return 'inconclusive' if n >= max_races else 'continue'


def nash_value(A: np.ndarray):
    """(game value, one maximin mixture) of the zero-sum game with payoff A (row player)."""
    n = A.shape[0]
    c = np.zeros(n + 1)
    c[-1] = -1.0                                        # maximise v
    A_ub = np.hstack([-A.T, np.ones((A.shape[1], 1))])  # v - p.A[:, j] <= 0 for every column j
    b_ub = np.zeros(A.shape[1])
    A_eq = np.hstack([np.ones((1, n)), np.zeros((1, 1))])
    res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=[1.0],
                  bounds=[(0, 1)] * n + [(None, None)], method='highs')
    if not res.success:
        raise RuntimeError(f'Nash LP failed: {res.message}')
    return float(res.x[-1]), res.x[:n]


def maxent_nash(A: np.ndarray, tol: float = 1e-6) -> np.ndarray:
    """The maximum-entropy mixture among those achieving the game value."""
    v, p0 = nash_value(A)
    n = A.shape[0]

    def neg_entropy(p):
        q = np.clip(p, 1e-12, 1)
        return float((q * np.log(q)).sum())
    cons = [{'type': 'eq', 'fun': lambda p: p.sum() - 1.0},
            {'type': 'ineq', 'fun': lambda p: A.T @ p - (v - tol)}]
    res = minimize(neg_entropy, np.full(n, 1.0 / n) if np.all(A.T @ np.full(n, 1 / n) >= v - tol) else p0,
                   bounds=[(0, 1)] * n, constraints=cons, method='SLSQP', options={'maxiter': 500, 'ftol': 1e-12})
    p = res.x if res.success else p0
    p = np.clip(p, 0, None)
    return p / p.sum()


def nash_average(winrate: np.ndarray) -> np.ndarray:
    """Each agent's score against the max-entropy Nash mixture of everyone."""
    A = winrate - 0.5
    A = 0.5 * (A - A.T)                                # enforce antisymmetry
    p = maxent_nash(A)
    return A @ p


@dataclass
class Retirement:
    """Per (role, panel member): consecutive snapshots beaten above 95%."""
    streak: int = 0
    retired: bool = False
    since: int = 0

    def should_check(self, snapshot_index: int) -> bool:
        return not self.retired or (snapshot_index - self.since) % 10 == 0

    def record(self, snapshot_index: int, win_rate: float) -> None:
        if win_rate > 0.95:
            self.streak += 1
            if self.streak >= 10 and not self.retired:
                self.retired, self.since = True, snapshot_index
        else:
            self.streak, self.retired = 0, False


def select(candidates: list, panel: list, results: dict, contacts: dict, min_races: int = 200) -> dict:
    """results[(a, b)] = (wins_of_a, races); contacts[a] = at-fault contacts over all held-out races.
    Returns the report: ranking, vetoes, and the winner (or None)."""
    agents = list(candidates) + [p for p in panel if p not in candidates]
    n = len(agents)
    W = np.full((n, n), 0.5)
    for i, a in enumerate(agents):
        for j, b in enumerate(agents):
            if i == j:
                continue
            if (a, b) in results:
                w, r = results[(a, b)]
            elif (b, a) in results:
                w0, r = results[(b, a)]
                w = r - w0
            else:
                continue
            W[i, j] = w / r if r else 0.5
    scores = dict(zip(agents, nash_average(W)))
    report = {'ranking': [], 'vetoed': {}, 'winner': None}
    for c in sorted(candidates, key=lambda a: -scores[a]):
        why = []
        races = sum(r for (a, b), (_, r) in results.items() if c in (a, b))
        if races < min_races:
            why.append(f'only {races} held-out races (need {min_races})')
        if contacts.get(c, 0) > 0:
            why.append(f'{contacts[c]} at-fault contact(s)')
        for p in panel:
            key = (c, p) if (c, p) in results else None
            if key is None and (p, c) in results:
                w0, r = results[(p, c)]
                w = r - w0
            elif key is not None:
                w, r = results[key]
            else:
                why.append(f'never raced panel member {p}')
                continue
            verdict = sequential_verdict(w, r)
            if verdict != 'win':
                why.append(f'{verdict} vs panel member {p} ({w}/{r})')
        report['ranking'].append({'agent': c, 'nash_score': float(scores[c])})
        if why:
            report['vetoed'][c] = why
        elif report['winner'] is None:
            report['winner'] = c
    report['pass'] = report['winner'] is not None
    return report


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--crossplay', type=Path, required=True,
                    help='jsonl lines {"a","b","wins_a","races","fault_a","fault_b"}')
    ap.add_argument('--panel', nargs='+', required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args(argv)
    results, contacts, cands = {}, {}, set()
    for line in args.crossplay.read_text().splitlines():
        d = json.loads(line)
        key = (d['a'], d['b'])
        w, r = results.get(key, (0, 0))
        results[key] = (w + d['wins_a'], r + d['races'])
        contacts[d['a']] = contacts.get(d['a'], 0) + d.get('fault_a', 0)
        contacts[d['b']] = contacts.get(d['b'], 0) + d.get('fault_b', 0)
        for x in key:
            if x not in args.panel:
                cands.add(x)
    report = select(sorted(cands), args.panel, results, contacts)
    args.out.write_text(json.dumps(report, indent=2))
    print(f"winner: {report['winner']}" if report['pass'] else 'no candidate passed')
    return 0 if report['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
