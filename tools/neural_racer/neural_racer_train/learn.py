"""The machinery every training stage shares: tracks and levels, the track
feeder, episode resets through PLR, and the auxiliary-head targets.

Seat 0 of every environment is the learner's car. Seats 1..C-1 are opponents
(scripted drivers, archived policies, or copies of the learner).
"""
import math
import multiprocessing as mp
from pathlib import Path

import numpy as np
import torch
import yaml

from neural_racer import observation as obs
from neural_racer_train import plr as plr_mod, tracks
from neural_racer_train.gpu_sim.sim import (EPISODE_HANDOVER, EPISODE_MAP, EPISODE_REACTIVE,
                                            BatchedRaceSim, Level)
from neural_racer_train.track_pool import TrackPool

HELDOUT_FILE = Path(__file__).resolve().parents[1] / 'heldout.yaml'
HELDOUT_SEED_BASE = 1_000_000_000          # seeds at or above this are never trained on
EPISODE_MIX = ((EPISODE_REACTIVE, 0.3), (EPISODE_MAP, 0.3), (EPISODE_HANDOVER, 0.4))
V_CAP_RANGE = (1.0, 6.37)                  # Phase 10's ladder, up to the firmware cap
IMU_DROPOUT_EPISODES = 0.05


def heldout_digests() -> set:
    if not HELDOUT_FILE.exists():
        return set()
    data = yaml.safe_load(HELDOUT_FILE.read_text()) or {}
    return {t['digest'] for t in data.get('procedural', []) if t.get('digest')}


def _generate(seed: int):
    """Worker entry: one training track, rasterized (seeds below the held-out base only)."""
    if seed >= HELDOUT_SEED_BASE:
        raise ValueError('held-out seed requested for training')
    return tracks.rasterize(tracks.generate(seed))


class TrackFeeder:
    """Background CPU processes producing rasterized training tracks."""

    def __init__(self, workers: int, seed: int):
        self.rng = np.random.default_rng(seed)
        self.workers = workers
        self.pool = mp.get_context('spawn').Pool(workers) if workers > 0 else None
        self.pending = []

    def request(self, n: int):
        for _ in range(n):
            s = int(self.rng.integers(HELDOUT_SEED_BASE))
            self.pending.append(self.pool.apply_async(_generate, (s,)) if self.pool else _generate(s))

    def ready(self) -> list:
        done, keep = [], []
        for p in self.pending:
            if not self.pool:
                done.append(p)
            elif p.ready():
                try:
                    done.append(p.get())
                except Exception:     # noqa: BLE001 -- a failed track is skipped, never fatal
                    pass
            else:
                keep.append(p)
        self.pending = keep
        return done

    def close(self):
        if self.pool:
            self.pool.terminate()


class LevelMaker:
    """A Level, deterministically from (slot, seed): replaying a level replays its start."""

    def __init__(self, max_cars: int, solo: bool = False):
        self.max_cars, self.solo = max_cars, solo
        self.specs = {}

    def make(self, slot: int, seed: int, level_id: int) -> Level:
        spec = self.specs[slot]
        rng = np.random.default_rng(seed)
        n_cars = 1 if self.solo else int(rng.integers(1, self.max_cars + 1))   # 1-4, team decision
        poses, idx = tracks.start_grid(spec, n_cars, rng)
        types, probs = zip(*EPISODE_MIX)
        ep = int(rng.choice(types, p=probs))
        amp = rng.uniform(0.0, [0.3, 0.15, 0.08])
        return Level(slot=slot, poses=poses, start_idx=idx, episode_type=ep,
                     v_cap=float(rng.uniform(*V_CAP_RANGE)),
                     imu_present=bool(rng.random() >= IMU_DROPOUT_EPISODES),
                     ref_offset=np.stack([amp, rng.uniform(0, 2 * math.pi, 3)], axis=1),
                     slam_drift=float(rng.uniform(-0.05, 0.05)),
                     slam_heading=float(rng.uniform(-math.radians(1), math.radians(1))),
                     level_id=level_id)


class Runner:
    """Owns the sim, the track pool, the level buffer and episode resets."""

    def __init__(self, num_envs: int, max_cars: int, device, seed: int, *, pool_slots: int = 64,
                 canvas=(1024, 1024), cpu_workers: int = 0, solo: bool = False, initial_tracks: int = 8,
                 warmup_levels: int = 64):
        self.E, self.C, self.device = num_envs, max_cars, device
        self.warmup_levels = warmup_levels
        self.rng = np.random.default_rng(seed)
        self.pool = TrackPool(pool_slots, canvas=canvas, device=device)
        self.levels = plr_mod.LevelBuffer(capacity=pool_slots * 8, seed=seed)
        self.maker = LevelMaker(max_cars, solo)
        self.held = heldout_digests()
        self.feeder = TrackFeeder(cpu_workers, seed + 1)
        self.slot_digest = {}
        self.free_slots = list(range(pool_slots))
        self.env_level = [None] * num_envs
        for k in range(min(initial_tracks, pool_slots)):
            self._insert(_generate(int(self.rng.integers(HELDOUT_SEED_BASE))))
        self.sim = BatchedRaceSim(num_envs, max_cars, self.pool, device=device, seed=seed)
        self.env_replay = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.score_sum = np.zeros(num_envs)
        self.score_n = np.zeros(num_envs)
        self.reset(list(range(num_envs)))

    def _insert(self, raster) -> bool:
        if raster.digest in self.held:
            raise RuntimeError(f'held-out track {raster.digest} reached the training pool')
        if raster.sdf.shape[0] > self.pool.H or raster.sdf.shape[1] > self.pool.W:
            return False
        if self.free_slots:
            slot = self.free_slots.pop()
        else:
            busy = {self.levels.levels[i].meta['slot'] for i in self.env_level if i in self.levels.levels}
            cand = [s for s in self.slot_digest if s not in busy]
            if not cand:
                return False
            score = {s: max((r.score for r in self.levels.levels.values() if r.meta.get('slot') == s), default=0.0)
                     for s in cand}
            slot = min(cand, key=score.get)
            for i in [i for i, r in self.levels.levels.items() if r.meta.get('slot') == slot]:
                del self.levels.levels[i]
        self.pool.insert(raster, slot)
        self.maker.specs[slot] = raster.spec
        self.slot_digest[slot] = raster.digest
        return True

    def refresh_tracks(self, want: int = 4):
        for r in self.feeder.ready():
            self._insert(r)
        if len(self.feeder.pending) < want:
            self.feeder.request(want - len(self.feeder.pending))

    def reset(self, env_ids):
        draws = self.levels.sample(len(env_ids))
        lv = []
        for e, (level_id, replay) in zip(env_ids, draws):
            if level_id is None:
                slot = int(self.rng.choice(list(self.slot_digest)))
                level_id = self.levels.new_id()
                self.levels.add(plr_mod.LevelRecord(level_id, self.slot_digest[slot],
                                                    int(self.rng.integers(2 ** 31)), meta={'slot': slot}))
            rec = self.levels.levels[level_id]
            lv.append(self.maker.make(rec.meta['slot'], rec.seed, level_id))
            self.env_level[e] = level_id
            # robust PLR trains only on replayed levels -- once there are
            # enough levels to replay; until then every episode trains
            self.env_replay[e] = replay or len(self.levels.levels) < self.warmup_levels
            self.score_sum[e] = self.score_n[e] = 0.0
        self.sim.reset_envs(env_ids, lv)

    def add_scores(self, adv):
        """adv (T, E) for seat 0: robust PLR's positive value loss, per env."""
        a = adv.detach().clamp(min=0).cpu().numpy()
        self.score_sum += a.sum(0)
        self.score_n += a.shape[0]

    def end_episodes(self, done_mask):
        ids = done_mask.nonzero().flatten().tolist()
        for e in ids:
            if self.score_n[e] > 0:
                self.levels.update_score(self.env_level[e], self.score_sum[e] / self.score_n[e])
        if ids:
            self.reset(ids)
        return ids


def aux_targets(sim: BatchedRaceSim):
    """Targets for the auxiliary heads, from the simulator's true state:

      ego path: the car's next 1.5 s under its current speed and yaw rate
      (constant-turn extrapolation), in its own rear-axle frame;
      opponents: up to 4 others by distance, present flag, and their next
      1.5 s at constant velocity in the same frame.
    v1 simplification: extrapolated from the true state rather than read from
    later frames of the rollout (plan, Phase 6 note).
    """
    E, C = sim.E, sim.C
    t = obs.AUX_DT * torch.arange(1, obs.AUX_PATH_POINTS + 1, device=sim.device).view(1, 1, -1)
    v, w = sim.x[..., 3].unsqueeze(-1), sim.x[..., 5].unsqueeze(-1)
    small = w.abs() < 1e-3
    ws = torch.where(small, torch.ones_like(w), w)
    px = torch.where(small, v * t, v * torch.sin(w * t) / ws)
    py = torch.where(small, 0.5 * v * w * t * t, v * (1 - torch.cos(w * t)) / ws)
    ego = torch.stack([px, py], -1)                                       # (E, C, 6, 2)
    bx, by, yaw = sim._rear_axle()
    vx = sim.x[..., 3] * torch.cos(yaw + sim.x[..., 6])
    vy = sim.x[..., 3] * torch.sin(yaw + sim.x[..., 6])
    d = torch.cdist(torch.stack([bx, by], -1), torch.stack([bx, by], -1))
    d = d + torch.eye(C, device=sim.device) * 1e6
    d = torch.where(sim.active.unsqueeze(1), d, torch.full_like(d, 1e6))
    order = d.argsort(-1)
    opp = torch.zeros(E, C, obs.OPP_SLOTS, 1 + 2 * obs.AUX_PATH_POINTS, device=sim.device)
    c, s = torch.cos(yaw).unsqueeze(-1), torch.sin(yaw).unsqueeze(-1)
    for slot in range(min(obs.OPP_SLOTS, C - 1)):
        k = order[..., slot]
        present = torch.gather(d, 2, k.unsqueeze(-1)).squeeze(-1) < 1e5
        ox = torch.gather(bx, 1, k).unsqueeze(-1) + torch.gather(vx, 1, k).unsqueeze(-1) * t - bx.unsqueeze(-1)
        oy = torch.gather(by, 1, k).unsqueeze(-1) + torch.gather(vy, 1, k).unsqueeze(-1) * t - by.unsqueeze(-1)
        lx, ly = c * ox + s * oy, -s * ox + c * oy
        opp[..., slot, 0] = present.float()
        opp[..., slot, 1::2] = lx * present.unsqueeze(-1)
        opp[..., slot, 2::2] = ly * present.unsqueeze(-1)
    return ego, opp


def aux_loss(path, opp, ego_t, opp_t):
    """MSE on the ego path; BCE on presence and masked MSE on opponent positions."""
    l_path = ((path - ego_t) ** 2).mean()
    present = opp_t[..., 0]
    bce = torch.nn.functional.binary_cross_entropy_with_logits(opp[..., 0], present)
    pos_err = ((opp[..., 1:] * obs.AUX_POS_SCALE - opp_t[..., 1:]) ** 2).mean(-1)
    l_pos = (pos_err * present).sum() / present.sum().clamp(min=1.0)
    return l_path, bce + l_pos


# --- opponent seats --------------------------------------------------------------------------

class SeatAssignment:
    """What drives each opponent seat (E, C): an index into `sources`, plus
    per-seat random draws for scripted drivers. Seat 0 is always the learner."""

    def __init__(self, E: int, C: int, device):
        self.source = torch.zeros(E, C, dtype=torch.long, device=device)
        self.params = torch.zeros(E, C, 3, device=device)
        self.h = torch.zeros(E, C, 128, device=device)

    def assign(self, env_ids, n_sources: int, gen):
        if len(env_ids) == 0:
            return
        ids = torch.as_tensor(env_ids, device=self.source.device)
        C = self.source.shape[1]
        self.source[ids] = torch.randint(0, n_sources, (len(ids), C), generator=gen,
                                         device=gen.device).to(self.source.device)
        self.params[ids] = torch.rand(len(ids), C, 3, generator=gen, device=gen.device).to(self.params.device)
        self.h[ids] = 0.0


def opponent_actions(sim, o: dict, priv, seats: SeatAssignment, sources: list, stacked=None):
    """Squashed actions (E, C, 2) for seats 1..C-1.

    sources[i] is ('scripted', name) or ('policy', g): group g of `stacked`.
    Every policy group's seats are gathered into one padded (M, B, ...) batch
    and run in a single vmap call -- M opponents for the price of one pass.
    """
    from neural_racer_train import scripted
    from neural_racer_train.model import squash
    E, C = sim.E, sim.C
    out = torch.zeros(E, C, 2, device=sim.device)
    t = float(sim.t.float().mean()) * 0.025
    v_cap = sim.v_cap.view(-1, 1).expand(E, C)
    racing = sim.active.clone()
    racing[:, 0] = False
    groups = {}
    for i, (kind, name) in enumerate(sources):
        m = (seats.source == i) & racing
        if not m.any():
            continue
        if kind == 'scripted':
            out[m] = scripted.act(name, priv[m], o['scan'][m], v_cap[m], seats.params[m], t)
        else:
            groups[int(name)] = m
    if groups:
        M = stacked.M
        idx = {g: m.nonzero(as_tuple=True) for g, m in groups.items()}
        B = max(len(ix[0]) for ix in idx.values())
        ob = {k: torch.zeros(M, B, *v.shape[2:], device=sim.device) for k, v in o.items()}
        hh = torch.zeros(M, B, 128, device=sim.device)
        for g, (ei, ci) in idx.items():
            n = len(ei)
            for k, v in o.items():
                ob[k][g, :n] = v[ei, ci]
            hh[g, :n] = seats.h[ei, ci]
        a, h2 = stacked.act(ob, hh)
        for g, (ei, ci) in idx.items():
            n = len(ei)
            out[ei, ci] = squash(a[g, :n], v_cap[ei, ci])
            seats.h[ei, ci] = h2[g, :n]
    return out
