"""Who each league role trains against (plan A8).

    main agents        35% current self, 40% PFSP over the archive,
                       15% scripted pool (fading to a 3% floor), 10% uniform floor
    main exploiter     the current main agents only
    league exploiter   PFSP over the archive with p(1 - p)

PFSP weight (1 - p)^2 favours the opponents we lose to; the uniform floor
guarantees every archived agent keeps at least floor / N of the matches, so
nothing is ever discarded. The scripted share fades as we beat the pool:
share = max(0.03, min(0.15, 0.15 (1 - p_scripted) / 0.5)), and what it frees
goes to PFSP over the archive.
"""
from dataclasses import dataclass

import numpy as np

SCRIPTED = ('pure_pursuit_offset', 'gap_follow', 'blocker', 'erratic', 'parked')


@dataclass(frozen=True)
class OpponentRef:
    kind: str              # 'self' | 'archive' | 'scripted'
    name: str              # archive entry id, scripted driver name, or 'self'


class WinRates:
    """(wins + 1) / (games + 2) per opponent, from training matches."""

    def __init__(self):
        self.wins, self.games = {}, {}

    def record(self, opponent: str, won: bool) -> None:
        self.wins[opponent] = self.wins.get(opponent, 0) + int(won)
        self.games[opponent] = self.games.get(opponent, 0) + 1

    def p(self, opponent: str) -> float:
        return (self.wins.get(opponent, 0) + 1) / (self.games.get(opponent, 0) + 2)

    def state(self):
        return {'wins': self.wins, 'games': self.games}

    def load(self, st):
        self.wins, self.games = dict(st['wins']), dict(st['games'])


def pfsp_weights(p: np.ndarray, kind: str = 'hard') -> np.ndarray:
    p = np.asarray(p, dtype=np.float64)
    w = (1.0 - p) ** 2 if kind == 'hard' else p * (1.0 - p)
    if w.sum() <= 0:
        return np.full(len(p), 1.0 / len(p))
    return w / w.sum()


def scripted_share(p_scripted: float, base: float = 0.15, floor: float = 0.03) -> float:
    return max(floor, min(base, base * (1.0 - p_scripted) / 0.5))


def role_mix(role: str, p_scripted: float) -> dict:
    """Shares of matches by source for a role."""
    if role.startswith('main_exploiter'):
        return {'mains': 1.0}
    if role.startswith('league_exploiter'):
        return {'pfsp_var': 0.9, 'uniform': 0.1}
    sc = scripted_share(p_scripted)
    return {'self': 0.35, 'pfsp': 0.40 + (0.15 - sc), 'scripted': sc, 'uniform': 0.10}


def archive_probabilities(role: str, archive_ids: list, rates: WinRates) -> dict:
    """Each archived opponent's exact share of matches for a role -- the
    quantity the uniform floor guarantees (>= 0.10 / N for main agents)."""
    if not archive_ids:
        return {}
    scripted_ids = [f'scripted:{s}' for s in SCRIPTED]
    mix = role_mix(role, float(np.mean([rates.p(s) for s in scripted_ids])))
    n = len(archive_ids)
    prob = dict.fromkeys(archive_ids, 0.0)
    for src, share in mix.items():
        if src == 'uniform':
            for a in archive_ids:
                prob[a] += share / n
        elif src in ('pfsp', 'pfsp_var'):
            w = pfsp_weights([rates.p(a) for a in archive_ids], 'var' if src == 'pfsp_var' else 'hard')
            for a, wi in zip(archive_ids, w):
                prob[a] += share * wi
    return prob


def sample_opponents(role: str, n: int, archive_ids: list, rates: WinRates, rng,
                     main_ids: list | None = None) -> list:
    """n opponent references for one rollout's opponent groups."""
    scripted_ids = [f'scripted:{s}' for s in SCRIPTED]
    p_scr = float(np.mean([rates.p(s) for s in scripted_ids]))
    mix = role_mix(role, p_scr)
    sources = list(mix)
    probs = np.array([mix[s] for s in sources])
    probs = probs / probs.sum()
    out = []
    for _ in range(n):
        src = sources[int(rng.choice(len(sources), p=probs))]
        if src == 'self' or (src in ('pfsp', 'pfsp_var', 'uniform') and not archive_ids):
            out.append(OpponentRef('self', 'self'))
        elif src == 'scripted':
            w = pfsp_weights([rates.p(s) for s in scripted_ids])
            out.append(OpponentRef('scripted', SCRIPTED[int(rng.choice(len(SCRIPTED), p=w))]))
        elif src == 'mains':
            pool = main_ids or archive_ids
            out.append(OpponentRef('archive', pool[-1]) if pool else OpponentRef('self', 'self'))
        elif src == 'uniform':
            out.append(OpponentRef('archive', archive_ids[int(rng.integers(len(archive_ids)))]))
        else:
            kind = 'var' if src == 'pfsp_var' else 'hard'
            w = pfsp_weights([rates.p(a) for a in archive_ids], kind)
            out.append(OpponentRef('archive', archive_ids[int(rng.choice(len(archive_ids), p=w))]))
    return out


def exploiter_should_reset(win_rate: float, steps: int, threshold: float = 0.70,
                           budget: int = 2_000_000_000) -> bool:
    return win_rate > threshold or steps >= budget
