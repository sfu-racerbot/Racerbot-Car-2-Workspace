"""Shared plumbing for every training stage: paths, devices, checkpoints, and
the trainer contract the FIR scripts rely on (tools/neural_racer/slurm/README.md):

    --run-dir DIR --archive DIR --role NAME --cpu-workers N --resume
    SIGUSR1            -> checkpoint, then exit 85
    exit 0             -> the run's whole budget is done
"""
import argparse
import json
import os
import random
import signal
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
for rel in ('src/neural_racer', 'src/gap_follow', 'src/drive_intent'):
    p = str(REPO / rel)
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402
import torch  # noqa: E402

EXIT_DONE = 0
EXIT_CHECKPOINTED = 85


def base_parser(description: str) -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument('--run-dir', type=Path, required=True)
    ap.add_argument('--archive', type=Path, default=None)
    ap.add_argument('--role', default='solo')
    ap.add_argument('--cpu-workers', type=int, default=0)
    ap.add_argument('--resume', action='store_true')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    ap.add_argument('--num-envs', type=int, default=16384)
    ap.add_argument('--log-every', type=int, default=10, help='updates between metric copies to the host')
    ap.add_argument('--checkpoint-every', type=int, default=50)
    return ap


def seed_everything(seed: int) -> torch.Generator:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    g = torch.Generator()
    g.manual_seed(seed)
    return g


def configure_torch(device: str) -> None:
    if device.startswith('cuda'):
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True


class StopFlag:
    """Set by SIGUSR1 (the time limit is near); checked between updates."""

    def __init__(self):
        self.requested = False
        signal.signal(signal.SIGUSR1, self._handle)

    def _handle(self, *_):
        self.requested = True


def save_checkpoint(run_dir: Path, state: dict, name: str = 'latest') -> Path:
    """Atomic: written to a temp file, then renamed over the old one."""
    ckpt_dir = Path(run_dir) / 'checkpoints'
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    path = ckpt_dir / f'{name}.pt'
    tmp = path.with_suffix('.pt.tmp')
    torch.save(state, tmp)
    os.replace(tmp, path)
    return path


def load_checkpoint(run_dir: Path, name: str = 'latest', map_location='cpu'):
    path = Path(run_dir) / 'checkpoints' / f'{name}.pt'
    if not path.exists():
        return None
    return torch.load(path, map_location=map_location, weights_only=False)


class MetricLog:
    """Append-only JSON lines; one per logged update."""

    def __init__(self, run_dir: Path):
        Path(run_dir).mkdir(parents=True, exist_ok=True)
        self.path = Path(run_dir) / 'metrics.jsonl'

    def write(self, **fields) -> None:
        fields.setdefault('time', time.time())
        with open(self.path, 'a') as f:
            f.write(json.dumps(fields, default=float) + '\n')


def run_stage(train_fn, args) -> int:
    """Wrap a stage: resume, train until done or SIGUSR1, checkpoint, exit code."""
    configure_torch(args.device)
    stop = StopFlag()
    finished = train_fn(args, stop)
    return EXIT_DONE if finished else EXIT_CHECKPOINTED
