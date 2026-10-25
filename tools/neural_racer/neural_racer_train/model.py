"""The networks (plan A4): the deployable student policy, the privileged
teacher, and the privileged critic.

PolicyNet's parameter names are exactly the keys src/neural_racer's
policy_numpy.py reads, so export.py copies the state_dict straight into
weights.npz. tests/test_export_golden.py holds the two forward passes to 1e-5.

Actions: the networks output pre-squash values a; the car applies
steering = 0.33 tanh(a0), speed = v_cap sigmoid(a1). PPO samples and stores
the pre-squash value, so the squash's Jacobian cancels in every ratio.
"""
import math

import torch
from torch import nn

from neural_racer import observation as obs
from neural_racer_train.gpu_sim.sim import PRIV_DIM

FULL = ((32, 64, 64, 64), 256)
HALF = ((16, 32, 32, 32), 128)
GRU = 128


class PolicyNet(nn.Module):
    def __init__(self, width=FULL, recurrent: bool = True):
        super().__init__()
        (c0, c1, c2, c3), fc = width
        self.width, self.recurrent = width, recurrent
        self.scan = nn.ModuleDict({
            'c0': nn.Conv1d(4, c0, 7, 2, 3), 'c1': nn.Conv1d(c0, c1, 5, 2, 2),
            'c2': nn.Conv1d(c1, c2, 5, 2, 2), 'c3': nn.Conv1d(c2, c3, 3, 2, 1),
            'fc': nn.Linear(c3 * 17, fc)})
        self.ego = nn.ModuleDict({'fc0': nn.Linear(obs.EGO_DIM, 64), 'fc1': nn.Linear(64, 64)})
        self.prev = nn.ModuleDict({'c0': nn.Conv1d(4, 32, 5, 2, 2), 'c1': nn.Conv1d(32, 32, 3, 2, 1),
                                   'fc': nn.Linear(256, 64)})
        self.race = nn.ModuleDict({'fc': nn.Linear(obs.RACE_DIM, 32)})
        ln = fc + 64 + 64 + 32
        self.fuse = nn.ModuleDict({'ln': nn.LayerNorm(ln), 'fc': nn.Linear(ln + obs.CAMERA_DIM, 256)})
        # the MLP ablation (main1_mlp) keeps the same names: a GRUCell whose
        # hidden input is always zero is a feed-forward layer with gates
        self.gru = nn.GRUCell(256, GRU)
        self.head = nn.ModuleDict({'fc0': nn.Linear(GRU, 128), 'act': nn.Linear(128, 2)})
        self.aux = nn.ModuleDict({'path': nn.Linear(GRU, 2 * obs.AUX_PATH_POINTS),
                                  'opp': nn.Linear(GRU, obs.OPP_SLOTS * (1 + 2 * obs.AUX_PATH_POINTS))})
        self.log_std = nn.Parameter(torch.full((2,), -1.0))     # training only; never exported
        with torch.no_grad():
            # v1 contract (A4/A5): everything the camera reaches starts at
            # exactly zero, so enabling a camera later changes nothing until
            # fine-tuning moves it.
            self.fuse['fc'].weight[:, ln:] = 0.0
            self.ego['fc0'].weight[:, obs.EGO['camera_present']:obs.EGO['camera_age'] + 1] = 0.0

    def forward(self, o: dict, h):
        """o: blocks with a leading batch dim. Returns (a (N, 2), h', path (N, 6, 2) m, opp (N, 4, 13))."""
        act = nn.functional.silu
        ego = o['ego']
        mode = ego[:, obs.EGO['mode']:obs.EGO['mode'] + 1]
        cam = ego[:, obs.EGO['camera_present']:obs.EGO['camera_present'] + 1]
        x = o['scan']
        for k in ('c0', 'c1', 'c2', 'c3'):
            x = act(self.scan[k](x))
        z_scan = act(self.scan['fc'](x.flatten(1)))
        z_ego = act(self.ego['fc1'](act(self.ego['fc0'](ego))))
        y = act(self.prev['c1'](act(self.prev['c0'](o['preview']))))
        z_prev = act(self.prev['fc'](y.flatten(1))) * mode
        z_race = act(self.race['fc'](o['race'])) * mode
        z = self.fuse['ln'](torch.cat([z_scan, z_ego, z_prev, z_race], dim=1))
        u = act(self.fuse['fc'](torch.cat([z, o['camera'] * cam], dim=1)))
        if not self.recurrent:
            h = torch.zeros_like(h)
        h = gru_cell(u, h, self.gru)
        a = self.head['act'](act(self.head['fc0'](h)))
        path = self.aux['path'](h).view(-1, obs.AUX_PATH_POINTS, 2) * obs.AUX_POS_SCALE
        opp = self.aux['opp'](h).view(-1, obs.OPP_SLOTS, 1 + 2 * obs.AUX_PATH_POINTS)
        return a, h, path, opp


class TeacherNet(nn.Module):
    """Stage 1: privileged state in, the same pre-squash actions out (no memory:
    its input is already close to complete)."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(PRIV_DIM, 512), nn.SiLU(), nn.Linear(512, 512), nn.SiLU(),
                                 nn.Linear(512, 256), nn.SiLU(), nn.Linear(256, 2))
        self.log_std = nn.Parameter(torch.full((2,), -1.0))

    def forward(self, priv):
        return self.net(priv)


class Critic(nn.Module):
    """The privileged value function (asymmetric actor-critic). Never ships."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(PRIV_DIM, 512), nn.SiLU(), nn.Linear(512, 512), nn.SiLU(),
                                 nn.Linear(512, 1))

    def forward(self, priv):
        return self.net(priv).squeeze(-1)


def gru_cell(x, h, cell: nn.GRUCell):
    """nn.GRUCell's equations from its own parameters, written as plain matrix
    products: identical results, but vmap can batch it (it has no batching
    rule for aten::gru_cell and falls back to a per-model loop)."""
    gi = nn.functional.linear(x, cell.weight_ih, cell.bias_ih)
    gh = nn.functional.linear(h, cell.weight_hh, cell.bias_hh)
    i_r, i_z, i_n = gi.chunk(3, dim=-1)
    h_r, h_z, h_n = gh.chunk(3, dim=-1)
    r = torch.sigmoid(i_r + h_r)
    z = torch.sigmoid(i_z + h_z)
    n = torch.tanh(i_n + r * h_n)
    return (1 - z) * n + z * h


def squash(a, v_cap):
    """Pre-squash actions -> (steering rad, speed m/s), as the car applies them."""
    return torch.stack([obs.STEER_LIMIT * torch.tanh(a[..., 0]), v_cap * torch.sigmoid(a[..., 1])], dim=-1)


def gaussian_logp(a, mean, log_std):
    var = torch.exp(2 * log_std)
    return (-(a - mean) ** 2 / (2 * var) - log_std - 0.5 * math.log(2 * math.pi)).sum(-1)


def flat_obs(o: dict, mask=None):
    """(E, C, ...) blocks -> (N, ...) for the cars in mask (default: all)."""
    if mask is None:
        return {k: v.reshape(-1, *v.shape[2:]) for k, v in o.items()}
    return {k: v[mask] for k, v in o.items()}
