"""approve_model: put an exported model on the car's approved list.

    ros2 run neural_racer approve_model ~/.ros/neural_racer/incoming/<hash> [--shadow-only]

Refuses, and says why, unless every check passes:
  * weights.npz hashes to the directory's name
  * the model card's observation spec matches this code
  * PolicyNumpy reproduces golden.npz (PyTorch's own outputs) to 1e-5
  * for a full approval: selection_report.json and referee_report.json both
    say "pass": true (docs/superpowers/specs/2026-10-23-neural-racer-plan.md, Phase 9)
A shadow-only approval needs no reports, and the node will not drive /drive with it.
"""
import argparse
import datetime
import json
import shutil
import sys
from pathlib import Path

from neural_racer import model_store
from neural_racer.policy_numpy import PolicyNumpy

REPORTS = ('selection_report.json', 'referee_report.json')


def _report_passes(path: Path) -> bool:
    try:
        return json.loads(path.read_text()).get('pass') is True
    except (OSError, ValueError, AttributeError):
        return False


def approve(src: Path, models_dir: Path, shadow_only: bool) -> str:
    src = Path(src)
    model_hash = model_store.weights_hash(src / 'weights.npz')
    if src.name != model_hash:
        raise model_store.ModelStoreError(
            f'directory {src.name!r} does not match its weights hash {model_hash}')
    weights, card = model_store.load_dir(src)
    model_store.check_modalities(card, camera_enabled='camera' in card.get('modalities', []))
    if not (src / 'golden.npz').exists():
        raise model_store.ModelStoreError('golden.npz is missing: re-run export.py')
    err = model_store.golden_error(PolicyNumpy(weights, card), src / 'golden.npz')
    if not err <= model_store.GOLDEN_ATOL:
        raise model_store.ModelStoreError(
            f'PolicyNumpy differs from the exported PyTorch outputs by {err:.2e} '
            f'(limit {model_store.GOLDEN_ATOL:.0e})')
    if not shadow_only:
        failing = [r for r in REPORTS if not _report_passes(src / r)]
        if failing:
            raise model_store.ModelStoreError(
                f'{failing} missing or not passing; use --shadow-only for shadow mode')
    models_dir = Path(models_dir)
    models_dir.mkdir(parents=True, exist_ok=True)
    dest = models_dir / model_hash
    if dest.resolve() != src.resolve():
        if dest.exists():
            shutil.rmtree(dest)
        shutil.copytree(src, dest)
    models = model_store.read_manifest(models_dir)
    models[model_hash] = {
        'approval': 'shadow_only' if shadow_only else 'full',
        'approved': datetime.date.today().isoformat(),
        'golden_error': float(err),
        'run': card.get('run_name'),
        'snapshot': card.get('snapshot_id'),
    }
    model_store.write_manifest(models_dir, models)
    return model_hash


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('model_dir', type=Path)
    parser.add_argument('--models-dir', type=Path, default=model_store.DEFAULT_MODELS_DIR)
    parser.add_argument('--shadow-only', action='store_true')
    args = parser.parse_args(argv)
    try:
        model_hash = approve(args.model_dir, args.models_dir, args.shadow_only)
    except model_store.ModelStoreError as exc:
        print(f'REFUSED: {exc}', file=sys.stderr)
        return 1
    kind = 'shadow mode only' if args.shadow_only else 'driving'
    print(f'approved {model_hash} for {kind}; golden check passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
