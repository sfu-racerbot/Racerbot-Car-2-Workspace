"""Shared helpers for neural_racer's tests: path setup and synthetic models.

Random weights here have the exact v1 shapes, so every test in this folder
runs on the Jetson with no PyTorch. The comparison against PyTorch's own
forward pass lives in tools/neural_racer/tests/test_export_golden.py.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for rel in ('..', '../../gap_follow', '../../drive_intent'):
    path = os.path.normpath(os.path.join(HERE, rel))
    if path not in sys.path:
        sys.path.insert(0, path)

import numpy as np  # noqa: E402
import yaml  # noqa: E402

FULL = ((32, 64, 64, 64), 256)
HALF = ((16, 32, 32, 32), 128)


def random_weights(seed: int = 0, width=FULL, scale: float = 0.3) -> dict:
    (c0, c1, c2, c3), fc = width
    rng = np.random.default_rng(seed)
    shapes = {
        'scan.c0': (c0, 4, 7), 'scan.c1': (c1, c0, 5), 'scan.c2': (c2, c1, 5),
        'scan.c3': (c3, c2, 3), 'scan.fc': (fc, c3 * 17),
        'ego.fc0': (64, 23), 'ego.fc1': (64, 64),
        'prev.c0': (32, 4, 5), 'prev.c1': (32, 32, 3), 'prev.fc': (64, 256),
        'race.fc': (32, 6),
        'fuse.fc': (256, fc + 64 + 64 + 32 + 64),
        'head.fc0': (128, 128), 'head.act': (2, 128),
        'aux.path': (12, 128), 'aux.opp': (52, 128),
    }
    w = {}
    for name, shape in shapes.items():
        fan_in = int(np.prod(shape[1:]))
        w[f'{name}.weight'] = (rng.standard_normal(shape) * scale / math.sqrt(fan_in) * 3).astype(np.float32)
        w[f'{name}.bias'] = (rng.standard_normal(shape[0]) * 0.1).astype(np.float32)
    d = fc + 64 + 64 + 32          # LayerNorm width; the camera slot sits after it
    w['fuse.ln.weight'] = (1.0 + 0.1 * rng.standard_normal(d)).astype(np.float32)
    w['fuse.ln.bias'] = (0.1 * rng.standard_normal(d)).astype(np.float32)
    w['gru.weight_ih'] = (rng.standard_normal((384, 256)) / 16).astype(np.float32)
    w['gru.weight_hh'] = (rng.standard_normal((384, 128)) / 11).astype(np.float32)
    w['gru.bias_ih'] = (0.1 * rng.standard_normal(384)).astype(np.float32)
    w['gru.bias_hh'] = (0.1 * rng.standard_normal(384)).astype(np.float32)
    # v1 contract: everything the camera reaches starts at exactly zero -- its
    # 64 fusion columns and the ego layer's camera_present / camera_age inputs.
    w['fuse.fc.weight'][:, d:] = 0.0
    w['ego.fc0.weight'][:, 21:23] = 0.0
    return w


def card(width=FULL, **extra) -> dict:
    c = {'obs_spec_version': 1, 'scan_channels': list(width[0]), 'scan_fc': width[1],
         'modalities': ['scan', 'ego', 'imu', 'preview', 'race'],
         'run_name': 'test', 'snapshot_id': 'test-0'}
    c.update(extra)
    return c


def write_model(root, weights, card_dict, golden=None, reports=None, name=None):
    """Write a model directory named by its hash (or `name`); returns (dir, hash)."""
    from neural_racer import model_store
    root = os.fspath(root)
    tmp = os.path.join(root, '_tmp_model')
    os.makedirs(tmp, exist_ok=True)
    np.savez(os.path.join(tmp, 'weights.npz'), **weights)
    with open(os.path.join(tmp, 'model_card.yaml'), 'w') as f:
        yaml.safe_dump(card_dict, f)
    if golden is not None:
        np.savez(os.path.join(tmp, 'golden.npz'), **golden)
    for fname, body in (reports or {}).items():
        with open(os.path.join(tmp, fname), 'w') as f:
            f.write(body)
    h = model_store.weights_hash(os.path.join(tmp, 'weights.npz'))
    final = os.path.join(root, name or h)
    os.replace(tmp, final)
    return final, h


def make_golden(policy, seqs: int = 2, steps: int = 5, seed: int = 0) -> dict:
    """Record a policy's own outputs as a golden file (a self-consistency
    fixture for the plumbing; the real golden comes from PyTorch)."""
    rng = np.random.default_rng(seed)
    g = {'scan': rng.random((seqs, steps, 4, 270)).astype(np.float32),
         'ego': rng.random((seqs, steps, 23)).astype(np.float32),
         'preview': rng.standard_normal((seqs, steps, 4, 32)).astype(np.float32),
         'race': rng.random((seqs, steps, 6)).astype(np.float32),
         'camera': np.zeros((seqs, steps, 64), np.float32)}
    g['ego'][..., 19] = 1.0
    g['ego'][..., 21] = 0.0
    g['ego'][..., 20] = 0.4
    action = np.zeros((seqs, steps, 2), np.float32)
    path = np.zeros((seqs, steps, 6, 2), np.float32)
    for s in range(seqs):
        policy.reset()
        for t in range(steps):
            out = policy.step({k: g[k][s, t] for k in ('scan', 'ego', 'preview', 'race', 'camera')})
            action[s, t] = (out.steering, out.speed)
            path[s, t] = out.ego_path
    policy.reset()
    g['action'], g['ego_path'] = action, path
    return g


def clear_scan(r: float = 8.0, n: int = 1081) -> np.ndarray:
    return np.full(n, r, dtype=np.float64)


UST_ANGLE_MIN = -math.radians(135.0)
UST_INCREMENT = math.radians(0.25)
