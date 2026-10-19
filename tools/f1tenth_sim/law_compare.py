"""The promotion rule for steering laws (spec 01), kept pure Python so it is
unit-testable without the simulator (see tests/test_law_compare.py).

A candidate law is PROMOTE only if it is safe on every run AND beats pure
pursuit on cross-track or lap time AND is no more than 5% worse on the other.
Selecting a law is never evidence it is better; only this rule, run on the
judging seeds, is. See docs/superpowers/specs/2026-09-27-tracking-01-harness.md.
"""

import math
from dataclasses import dataclass, field

# How much worse than pure pursuit a candidate may be on the metric it does
# NOT win on, before it is rejected. Spec 01 condition 3.
WORSE_TOLERANCE = 1.05


@dataclass(frozen=True)
class Verdict:
    """The comparison outcome. `verdict` is 'PROMOTE', 'REJECT', or 'INVALID'
    (the baseline itself failed safety, so the comparison says nothing).
    `reasons` lists every failed condition (not just the first)."""
    verdict: str
    reasons: tuple = ()
    candidate_mean_cross_track: float = math.nan
    baseline_mean_cross_track: float = math.nan
    candidate_mean_lap_time: float = math.nan
    baseline_mean_lap_time: float = math.nan


def _check_row_finite(row, key):
    value = row.get(key)
    if value is not None and not math.isfinite(value):
        raise ValueError(f'non-finite metric {key} in row for {row.get("track")}')


def _match_rows(baseline_rows, candidate_rows):
    """Match candidate rows to baseline rows on (track, seed, raceline).

    A missing or extra match is a ValueError -- never silently compare
    different sets."""
    def key(r):
        return (r.get('track'), r.get('seed'), r.get('raceline', 'shipped'))
    base = {key(r): r for r in baseline_rows}
    cand = {key(r): r for r in candidate_rows}
    if set(base) != set(cand):
        missing = set(base) - set(cand)
        extra = set(cand) - set(base)
        raise ValueError(
            f'unmatched run sets: baseline-only {sorted(missing)}, '
            f'candidate-only {sorted(extra)}')
    return [base[k] for k in sorted(base)], [cand[k] for k in sorted(cand)]


def _safety_failures(row):
    """Every safety condition (spec 01 condition 1) a run fails, as strings."""
    failures = []
    if not (row.get('laps', 0.0) >= 1.0):
        failures.append('laps < 1')
    if row.get('collision', True):
        failures.append('collision')
    if not (row.get('max_cross_track_m', math.inf) < 0.5):
        failures.append('max_cross_track_m >= 0.5')
    if row.get('law_fail_steps', 0) != 0:
        failures.append('law_fail_steps != 0')
    if row.get('mpc_latched', False):
        failures.append('mpc_latched')
    return failures


def verdict(baseline_rows, candidate_rows):
    """Compare a candidate law against the pure-pursuit baseline.

    Rows are dicts with at least track, seed, laps, collision,
    max_cross_track_m, law_fail_steps, mean_cross_track_m, lap_time_s (and
    mpc_latched for MPC). Returns a Verdict.
    """
    base, cand = _match_rows(baseline_rows, candidate_rows)
    for row in base + cand:
        for key in ('mean_cross_track_m', 'lap_time_s', 'max_cross_track_m'):
            _check_row_finite(row, key)

    # Safety first, on every run, for both sides. A baseline safety failure
    # makes the whole comparison INVALID (it says nothing about the candidate).
    reasons = []
    baseline_unsafe = False
    for row in base:
        fails = _safety_failures(row)
        if fails:
            baseline_unsafe = True
            reasons.append(
                f'baseline run {row.get("track")} seed={row.get("seed")} '
                f'fails safety: {", ".join(fails)}')
    if baseline_unsafe:
        return Verdict(verdict='INVALID', reasons=tuple(reasons))

    for row in cand:
        fails = _safety_failures(row)
        if fails:
            reasons.append(
                f'candidate run {row.get("track")} seed={row.get("seed")} '
                f'fails safety: {", ".join(fails)}')

    # Aggregate metrics over all matched runs.
    cand_xt = [r['mean_cross_track_m'] for r in cand]
    base_xt = [r['mean_cross_track_m'] for r in base]
    cand_lt = [r.get('lap_time_s') for r in cand]
    base_lt = [r.get('lap_time_s') for r in base]
    # A run that never lapped has lap_time_s None, which fails condition 1
    # (laps >= 1) above. Here we only aggregate runs that have a lap time.
    if any(t is None for t in cand_lt + base_lt):
        # laps < 1 already recorded as a safety failure; nothing more to add.
        cand_mean_lt = math.inf
        base_mean_lt = math.inf
    else:
        cand_mean_lt = sum(cand_lt) / len(cand_lt)
        base_mean_lt = sum(base_lt) / len(base_lt)
    cand_mean_xt = sum(cand_xt) / len(cand_xt)
    base_mean_xt = sum(base_xt) / len(base_xt)

    # Condition 2: strictly better on at least one metric.
    better_xt = cand_mean_xt < base_mean_xt
    better_lt = cand_mean_lt < base_mean_lt
    if not (better_xt or better_lt):
        reasons.append(
            'condition 2: not strictly better on either metric '
            f'(cross-track {cand_mean_xt:.4f} vs {base_mean_xt:.4f}, '
            f'lap time {cand_mean_lt:.3f} vs {base_mean_lt:.3f})')

    # Condition 3: not more than 5% worse on the *other* metric (both checked).
    if not (cand_mean_xt <= WORSE_TOLERANCE * base_mean_xt):
        reasons.append(
            f'condition 3: cross-track {cand_mean_xt:.4f} worse than '
            f'{WORSE_TOLERANCE}x baseline {base_mean_xt:.4f}')
    if not (cand_mean_lt <= WORSE_TOLERANCE * base_mean_lt):
        reasons.append(
            f'condition 3: lap time {cand_mean_lt:.3f} worse than '
            f'{WORSE_TOLERANCE}x baseline {base_mean_lt:.3f}')

    outcome = 'PROMOTE' if not reasons else 'REJECT'
    return Verdict(
        verdict=outcome, reasons=tuple(reasons),
        candidate_mean_cross_track=cand_mean_xt,
        baseline_mean_cross_track=base_mean_xt,
        candidate_mean_lap_time=cand_mean_lt,
        baseline_mean_lap_time=base_mean_lt)
