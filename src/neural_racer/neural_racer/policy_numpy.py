"""The neural racer's forward pass in plain NumPy -- what runs on the car.

Weights are float32 arrays named exactly as the PyTorch model's state_dict
(tools/neural_racer/neural_racer_train/model.py), so export is a straight
copy. Architecture: docs/superpowers/specs/2026-10-23-neural-racer-plan.md, A4.

Convolutions are im2col over a strided window view, the fastest of the
variants measured on car 1's Jetson (3.40 ms median for the full width).
"""
from dataclasses import dataclass

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from neural_racer import observation as obs

# (name, stride, padding) of each scan / preview convolution, in order.
SCAN_CONVS = (('scan.c0', 2, 3), ('scan.c1', 2, 2), ('scan.c2', 2, 2), ('scan.c3', 2, 1))
PREVIEW_CONVS = (('prev.c0', 2, 2), ('prev.c1', 2, 1))
GRU_HIDDEN = 128


@dataclass(frozen=True)
class PolicyOutput:
    steering: float               # rad, within +/- STEER_LIMIT by construction
    speed: float                  # m/s, within [0, v_cap] by construction
    ego_path: np.ndarray          # (6, 2) car frame, m, every AUX_DT s
    opponents: np.ndarray         # (4, 13): presence probability, then 6 x (x, y) in m


class PolicyError(ValueError):
    """The weights do not describe a v1 neural-racer policy."""


def _sigmoid(x):
    # tanh form: no overflow for large |x|, and identical to 1/(1+e^-x)
    return 0.5 * (1.0 + np.tanh(0.5 * x))


def _silu(x):
    return x * _sigmoid(x)


class _Conv:
    """Conv1d as one matmul: weights pre-reshaped to (cin*k, cout)."""

    def __init__(self, w, b, stride: int, padding: int):
        self.cout, self.cin, self.k = w.shape
        self.w2 = np.ascontiguousarray(w.reshape(self.cout, -1).T)
        self.b = b
        self.stride, self.padding = stride, padding

    def __call__(self, x):
        if x.shape[0] != self.cin:
            raise PolicyError(f'conv expects {self.cin} channels, got {x.shape[0]}')
        xp = np.pad(x, ((0, 0), (self.padding, self.padding)))
        win = sliding_window_view(xp, self.k, axis=1)[:, ::self.stride, :]
        cols = np.ascontiguousarray(win.transpose(1, 0, 2)).reshape(win.shape[1], -1)
        return (cols @ self.w2 + self.b).T


class PolicyNumpy:
    """One car, one tick at a time. Keeps the GRU state between calls."""

    def __init__(self, weights: dict, card: dict | None = None):
        self.w = {k: np.asarray(v, dtype=np.float32) for k, v in weights.items()}
        self._check()
        self.scan_convs = [self._conv(n, s, p) for n, s, p in SCAN_CONVS]
        self.prev_convs = [self._conv(n, s, p) for n, s, p in PREVIEW_CONVS]
        if card is not None:
            want = card.get('scan_channels')
            have = [self.w[f'{n}.weight'].shape[0] for n, _, _ in SCAN_CONVS]
            if want is not None and list(want) != have:
                raise PolicyError(f'model card says scan channels {want}, weights have {have}')
        self.reset()

    def _conv(self, name, stride, padding):
        return _Conv(self.w[f'{name}.weight'], self.w[f'{name}.bias'], stride, padding)

    def _check(self):
        need = ['scan.fc.weight', 'ego.fc0.weight', 'ego.fc1.weight', 'prev.fc.weight',
                'race.fc.weight', 'fuse.ln.weight', 'fuse.ln.bias', 'fuse.fc.weight',
                'gru.weight_ih', 'gru.weight_hh', 'gru.bias_ih', 'gru.bias_hh',
                'head.fc0.weight', 'head.act.weight', 'aux.path.weight', 'aux.opp.weight']
        need += [f'{n}.weight' for n, _, _ in SCAN_CONVS + PREVIEW_CONVS]
        missing = [k for k in need if k not in self.w]
        if missing:
            raise PolicyError(f'weights missing {missing}')
        bad = [k for k, v in self.w.items() if not np.all(np.isfinite(v))]
        if bad:
            raise PolicyError(f'non-finite weights in {bad}')
        if self.w['gru.weight_hh'].shape != (3 * GRU_HIDDEN, GRU_HIDDEN):
            raise PolicyError('GRU hidden size must be 128')
        if self.w['fuse.fc.weight'].shape[1] != self.w['fuse.ln.weight'].shape[0] + obs.CAMERA_DIM:
            raise PolicyError('fusion layer must be LayerNorm width + the 64-wide camera slot')

    def reset(self) -> None:
        self.h = np.zeros(GRU_HIDDEN, dtype=np.float32)

    def _lin(self, name, x):
        return self.w[f'{name}.weight'] @ x + self.w[f'{name}.bias']

    def step(self, o: dict) -> PolicyOutput:
        ego = o['ego']
        mode = ego[obs.EGO['mode']]
        cam_present = ego[obs.EGO['camera_present']]
        v_cap = float(ego[obs.EGO['v_cap']]) * obs.V_SCALE

        x = o['scan']
        for conv in self.scan_convs:
            x = _silu(conv(x))
        z_scan = _silu(self._lin('scan.fc', x.reshape(-1)))
        z_ego = _silu(self._lin('ego.fc1', _silu(self._lin('ego.fc0', ego))))
        y = o['preview']
        for conv in self.prev_convs:
            y = _silu(conv(y))
        z_prev = _silu(self._lin('prev.fc', y.reshape(-1))) * mode
        z_race = _silu(self._lin('race.fc', o['race'])) * mode
        z_cam = o['camera'] * cam_present

        # LayerNorm over everything except the camera, so a camera added later
        # with zero-initialised columns changes nothing until it is trained.
        z = np.concatenate([z_scan, z_ego, z_prev, z_race])
        mu, var = z.mean(), z.var()
        z = (z - mu) / np.sqrt(var + 1e-5) * self.w['fuse.ln.weight'] + self.w['fuse.ln.bias']
        u = _silu(self._lin('fuse.fc', np.concatenate([z, z_cam])))

        H = GRU_HIDDEN
        gi = self.w['gru.weight_ih'] @ u + self.w['gru.bias_ih']
        gh = self.w['gru.weight_hh'] @ self.h + self.w['gru.bias_hh']
        r = _sigmoid(gi[:H] + gh[:H])
        zg = _sigmoid(gi[H:2 * H] + gh[H:2 * H])
        n = np.tanh(gi[2 * H:] + r * gh[2 * H:])
        self.h = ((1.0 - zg) * n + zg * self.h).astype(np.float32)

        t = _silu(self._lin('head.fc0', self.h))
        a = self._lin('head.act', t)
        path = self._lin('aux.path', self.h).reshape(obs.AUX_PATH_POINTS, 2) * obs.AUX_POS_SCALE
        opp = self._lin('aux.opp', self.h).reshape(obs.OPP_SLOTS, 1 + 2 * obs.AUX_PATH_POINTS)
        opp = np.concatenate([_sigmoid(opp[:, :1]), opp[:, 1:] * obs.AUX_POS_SCALE], axis=1)
        return PolicyOutput(
            steering=float(obs.STEER_LIMIT * np.tanh(a[0])),
            speed=float(v_cap * _sigmoid(a[1])),
            ego_path=path, opponents=opp)


def parameter_count(weights: dict) -> int:
    return int(sum(np.asarray(v).size for v in weights.values()))
