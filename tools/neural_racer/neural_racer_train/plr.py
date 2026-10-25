"""Prioritized level replay (robust PLR) with ACCEL-style mutation (plan A8).

A level is a track plus how an episode on it starts. 70% of episodes replay
high-scoring levels and are trained on; 30% are fresh levels, scored but not
trained on (robust PLR's rule). Score = mean positive value loss, the
estimate of how much there is left to learn on the level.
"""
from dataclasses import dataclass, field

import numpy as np


@dataclass
class LevelRecord:
    level_id: int
    track_digest: str
    seed: int
    score: float = 0.0
    last_seen: int = 0
    plays: int = 0
    meta: dict = field(default_factory=dict)


class LevelBuffer:

    def __init__(self, capacity: int = 4000, replay_prob: float = 0.7,
                 staleness: float = 0.3, temperature: float = 0.3, seed: int = 0):
        self.capacity, self.replay_prob = capacity, replay_prob
        self.staleness, self.temperature = staleness, temperature
        self.levels: dict[int, LevelRecord] = {}
        self.clock = 0
        self.next_id = 0
        self.rng = np.random.default_rng(seed)

    def new_id(self) -> int:
        self.next_id += 1
        return self.next_id - 1

    def add(self, rec: LevelRecord) -> None:
        if len(self.levels) >= self.capacity and rec.level_id not in self.levels:
            worst = min(self.levels.values(), key=lambda r: r.score)
            del self.levels[worst.level_id]
        self.levels[rec.level_id] = rec

    def probabilities(self, ids) -> np.ndarray:
        """Rank-based score prioritisation mixed with staleness, as robust PLR."""
        recs = [self.levels[i] for i in ids]
        scores = np.array([r.score for r in recs])
        ranks = np.empty(len(recs))
        ranks[np.argsort(-scores, kind='stable')] = np.arange(1, len(recs) + 1)
        h = (1.0 / ranks) ** (1.0 / self.temperature)
        p_score = h / h.sum()
        stale = np.array([self.clock - r.last_seen for r in recs], dtype=np.float64)
        p_stale = stale / stale.sum() if stale.sum() > 0 else np.full(len(recs), 1.0 / len(recs))
        return (1 - self.staleness) * p_score + self.staleness * p_stale

    def sample(self, n: int):
        """n draws: (level_id or None for 'make a fresh level', is_replay)."""
        self.clock += 1
        ids = list(self.levels)
        out = []
        for _ in range(n):
            if ids and self.rng.random() < self.replay_prob:
                i = int(self.rng.choice(ids, p=self.probabilities(ids)))
                self.levels[i].last_seen = self.clock
                self.levels[i].plays += 1
                out.append((i, True))
            else:
                out.append((None, False))
        return out

    def update_score(self, level_id: int, score: float) -> None:
        if level_id in self.levels:
            self.levels[level_id].score = float(score)

    def state(self) -> dict:
        return {'levels': {k: vars(v) for k, v in self.levels.items()}, 'clock': self.clock,
                'next_id': self.next_id, 'rng': self.rng.bit_generator.state}

    def load(self, st: dict) -> None:
        self.levels = {int(k): LevelRecord(**v) for k, v in st['levels'].items()}
        self.clock, self.next_id = st['clock'], st['next_id']
        self.rng.bit_generator.state = st['rng']


def positive_value_loss(adv) -> float:
    """Robust PLR's level score: mean of max(advantage, 0) over the episode."""
    a = np.asarray(adv, dtype=np.float64)
    return float(np.maximum(a, 0.0).mean()) if a.size else 0.0
