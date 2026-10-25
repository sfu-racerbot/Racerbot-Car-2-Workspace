"""Race bookkeeping on the GPU: progress along the centreline, laps, places
with hysteresis, and the reward of plan A8.

    r =   0.1 * ds / (v_ref * dt)       progress (v_ref = 4 m/s)
        + 2.0 * places gained           and -2.0 per place lost (0.3 m hysteresis)
        - 20 at-fault contact           (ends that car's episode)
        -  2 not-at-fault contact
        - 20 wall contact               (ends that car's episode)
        - 0.5 safety wrapper intervened
        - 0.01 (d_delta / 0.33)^2       steering smoothness
        + 10 (N - place) / (N - 1)      at the finish, N cars
"""
from dataclasses import dataclass

import torch


@dataclass(frozen=True)
class RewardConfig:
    v_ref: float = 4.0
    progress: float = 0.1
    place: float = 2.0
    place_hysteresis: float = 0.3
    fault_contact: float = 20.0
    other_contact: float = 2.0
    wall: float = 20.0
    intervened: float = 0.5
    smooth: float = 0.01
    finish: float = 10.0


def project(pool, slot, x, y, last_idx, window: int = 30):
    """Nearest centreline segment around last_idx: (idx, s, lateral), each (E, C)."""
    E, C = x.shape
    n = pool.n_points[slot].view(E, 1, 1)
    offs = torch.arange(-window, window + 1, device=x.device).view(1, 1, -1)
    cand = torch.remainder(last_idx.unsqueeze(-1) + offs, n)                 # (E, C, K)
    nxt = torch.remainder(cand + 1, n)
    tab = pool.centre[slot]                                                   # (E, N, 5)
    flat = lambda i: torch.gather(tab, 1, i.reshape(E, -1, 1).expand(-1, -1, 5)).reshape(E, C, -1, 5)  # noqa: E731
    a, b = flat(cand), flat(nxt)
    ab = b[..., :2] - a[..., :2]
    ap = torch.stack([x, y], -1).unsqueeze(2) - a[..., :2]
    t = ((ap * ab).sum(-1) / (ab * ab).sum(-1).clamp(min=1e-12)).clamp(0, 1)
    q = a[..., :2] + t.unsqueeze(-1) * ab
    d2 = ((torch.stack([x, y], -1).unsqueeze(2) - q) ** 2).sum(-1)
    k = d2.argmin(-1, keepdim=True)
    pick = lambda v: torch.gather(v, 2, k).squeeze(2)                         # noqa: E731
    seg = torch.sqrt((ab * ab).sum(-1))
    s = pick(a[..., 2] + t * seg)
    abn = ab / seg.clamp(min=1e-9).unsqueeze(-1)
    lateral = pick(abn[..., 0] * ap[..., 1] - abn[..., 1] * ap[..., 0])
    return pick(cand), s, lateral


class RaceState:
    def __init__(self, E: int, C: int, device='cpu'):
        z = lambda dt=torch.float32: torch.zeros(E, C, device=device, dtype=dt)  # noqa: E731
        self.idx = z(torch.long)
        self.s = z()
        self.total = z()
        self.ahead = torch.zeros(E, C, C, dtype=torch.bool, device=device)
        self.finished = z(torch.bool)
        self.finish_order = z(torch.long)
        self.n_finished = torch.zeros(E, dtype=torch.long, device=device)

    def reset(self, env_mask, idx, s):
        m = env_mask
        self.idx[m], self.s[m] = idx[m], s[m]
        self.total[m] = 0.0
        self.ahead[m] = False
        self.finished[m] = False
        self.finish_order[m] = 0
        self.n_finished[m] = 0

    def update(self, idx, s, length, moving, racing):
        """New projections -> (ds (E, C), places gained (E, C)).

        moving: cars still racing (only they make progress); racing: every car
        in the race -- a crashed car keeps its place until someone passes it."""
        L = length.view(-1, 1)
        ds = torch.remainder(s - self.s + L / 2, L) - L / 2
        ds = torch.where(moving, ds, torch.zeros_like(ds))
        self.idx, self.s = idx, s
        self.total = self.total + ds
        before = self.ahead.sum(-1)
        d = self.total.unsqueeze(2) - self.total.unsqueeze(1)                 # i ahead of j by d
        both = racing.unsqueeze(2) & racing.unsqueeze(1)
        self.ahead = torch.where(both & (d > 0.3), torch.ones_like(self.ahead), self.ahead)
        self.ahead = torch.where(both & (d < -0.3), torch.zeros_like(self.ahead), self.ahead)
        gained = self.ahead.sum(-1) - before
        return ds, gained

    def place(self, active):
        """1 = leading. Places among the cars still racing or finished."""
        n_active = active.sum(-1, keepdim=True)
        return (n_active - self.ahead.sum(-1)).clamp(min=1)


def reward(cfg: RewardConfig, ds, gained, fault, other, wall, intervened, d_steer, dt: float):
    r = cfg.progress * ds / (cfg.v_ref * dt)
    r = r + cfg.place * gained.float()
    r = r - cfg.fault_contact * fault.float() - cfg.other_contact * (other & ~fault).float()
    r = r - cfg.wall * wall.float() - cfg.intervened * intervened.float()
    r = r - cfg.smooth * (d_steer / 0.33) ** 2
    return r
