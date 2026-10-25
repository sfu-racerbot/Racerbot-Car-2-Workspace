"""export: a trained checkpoint -> the model directory the car loads (plan B0).

    python -m neural_racer_train.export <checkpoint.pt> <out_dir> [--golden-from <rollout.pt>]

Writes <out_dir>/<hash>/ with weights.npz (float32, policy_numpy's names),
model_card.yaml and golden.npz: 4 sequences x 16 ticks of inputs, with
PyTorch's own outputs for them, which approve_model on the car must
reproduce to 1e-5 before the model can load.
"""
import argparse
import datetime
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch
import yaml

from neural_racer_train import common  # noqa: F401  (paths)
from neural_racer import model_store, observation as obs
from neural_racer_train.model import PolicyNet, squash

EXCLUDE = ('log_std',)


def weights_of(policy: PolicyNet) -> dict:
    return {k: v.detach().cpu().float().numpy() for k, v in policy.state_dict().items() if k not in EXCLUDE}


def synthetic_inputs(seqs: int = 4, steps: int = 16, seed: int = 0) -> dict:
    """Plausible observation sequences when no recorded rollout is given:
    valid ranges, both modes, IMU on and off, speed caps across the ladder."""
    g = np.random.default_rng(seed)
    out = {'scan': g.random((seqs, steps, 4, 270)).astype(np.float32),
           'ego': (g.random((seqs, steps, obs.EGO_DIM)) * 2 - 1).astype(np.float32),
           'preview': g.normal(0, 0.5, (seqs, steps, 4, obs.PREVIEW_POINTS)).astype(np.float32),
           'race': g.random((seqs, steps, obs.RACE_DIM)).astype(np.float32),
           'camera': np.zeros((seqs, steps, obs.CAMERA_DIM), np.float32)}
    out['scan'][:, :, 3] = (out['scan'][:, :, 3] > 0.1)
    e = out['ego']
    e[..., obs.EGO['mode']] = np.arange(seqs)[:, None] % 2
    e[..., obs.EGO['imu_present']] = (np.arange(seqs)[:, None] // 2) % 2 == 0
    e[..., obs.EGO['v_cap']] = np.linspace(0.2, 1.27, seqs)[:, None]
    e[..., obs.EGO['camera_present']] = 0.0
    e[..., obs.EGO['camera_age']] = 0.0
    return out


@torch.no_grad()
def golden(policy: PolicyNet, inputs: dict) -> dict:
    """PyTorch's own outputs over the sequences, GRU state carried, float32."""
    policy = policy.cpu().float().eval()
    seqs, steps = inputs['ego'].shape[:2]
    action = np.zeros((seqs, steps, 2), np.float32)
    path = np.zeros((seqs, steps, obs.AUX_PATH_POINTS, 2), np.float32)
    for s in range(seqs):
        h = torch.zeros(1, 128)
        for t in range(steps):
            o = {k: torch.as_tensor(v[s, t:t + 1]) for k, v in inputs.items()}
            a, h, p, _ = policy(o, h)
            v_cap = o['ego'][:, obs.EGO['v_cap']] * obs.V_SCALE
            action[s, t] = squash(a, v_cap)[0].numpy()
            path[s, t] = p[0].numpy()
    return dict(inputs, action=action, ego_path=path)


def _commit() -> str:
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True,
                              cwd=common.REPO, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


def export(policy: PolicyNet, out_root: Path, *, run_name: str, snapshot_id: str,
           modalities=('scan', 'ego', 'imu', 'preview', 'race'), inputs=None) -> Path:
    out_root = Path(out_root)
    tmp = out_root / '_export_tmp'
    tmp.mkdir(parents=True, exist_ok=True)
    np.savez(tmp / 'weights.npz', **weights_of(policy))
    (c0, c1, c2, c3), fc = policy.width
    card = {'obs_spec_version': obs.OBS_SPEC_VERSION, 'scan_channels': [c0, c1, c2, c3], 'scan_fc': fc,
            'recurrent': policy.recurrent, 'modalities': list(modalities),
            'parameters': int(sum(v.size for v in weights_of(policy).values())),
            'run_name': run_name, 'snapshot_id': snapshot_id, 'training_commit': _commit(),
            'exported': datetime.datetime.now().isoformat(timespec='seconds')}
    (tmp / 'model_card.yaml').write_text(yaml.safe_dump(card, sort_keys=False))
    np.savez(tmp / 'golden.npz', **golden(policy, inputs or synthetic_inputs()))
    h = model_store.weights_hash(tmp / 'weights.npz')
    final = out_root / h
    if final.exists():
        import shutil
        shutil.rmtree(final)
    os.replace(tmp, final)
    return final


def load_policy(checkpoint: Path) -> tuple:
    state = torch.load(checkpoint, map_location='cpu', weights_only=False)
    cfg = state.get('policy_config', {})
    width = cfg.get('width', ((32, 64, 64, 64), 256))
    # stage checkpoints carry policy_config; league archive entries carry meta
    recurrent = cfg.get('recurrent', state.get('meta', {}).get('recurrent', True))
    policy = PolicyNet(width=(tuple(int(c) for c in width[0]), int(width[1])), recurrent=recurrent)
    policy.load_state_dict(state['policy'])
    return policy, state


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('checkpoint', type=Path)
    ap.add_argument('out_dir', type=Path)
    args = ap.parse_args(argv)
    policy, state = load_policy(args.checkpoint)
    path = export(policy, args.out_dir, run_name=state.get('run_name', '?'),
                  snapshot_id=str(state.get('snapshot_id', args.checkpoint.stem)))
    print(f'exported {path.name} -> {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
