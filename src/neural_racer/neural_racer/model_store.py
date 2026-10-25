"""Loading a model the node is allowed to drive with.

A model is a directory ~/.ros/neural_racer/models/<hash>/ holding weights.npz,
model_card.yaml and golden.npz (plus Phase 9's reports). <hash> is the first
16 hex digits of the SHA-256 of weights.npz. Only models listed in that
folder's manifest.yaml load, and only approve_model writes the manifest.
No rclpy import.
"""
import hashlib
import os
from pathlib import Path

import numpy as np
import yaml

from neural_racer import observation as obs
from neural_racer.policy_numpy import PolicyNumpy

DEFAULT_MODELS_DIR = Path(os.path.expanduser('~/.ros/neural_racer/models'))
GOLDEN_ATOL = 1e-5
APPROVALS = ('full', 'shadow_only')


class ModelStoreError(ValueError):
    """This model must not be loaded."""


def weights_hash(weights_path: Path) -> str:
    digest = hashlib.sha256()
    with open(weights_path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()[:16]


def read_manifest(models_dir: Path) -> dict:
    path = Path(models_dir) / 'manifest.yaml'
    if not path.exists():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    models = data.get('models', {})
    if not isinstance(models, dict):
        raise ModelStoreError('manifest.yaml: "models" must be a mapping')
    return models


def write_manifest(models_dir: Path, models: dict) -> None:
    path = Path(models_dir) / 'manifest.yaml'
    tmp = path.with_suffix('.yaml.tmp')
    tmp.write_text(yaml.safe_dump({'models': models}, sort_keys=True))
    os.replace(tmp, path)


def load_dir(model_dir: Path):
    """(weights dict, card dict) from a model directory, with the hash checked
    against the directory name when the name looks like a hash."""
    model_dir = Path(model_dir)
    weights_path = model_dir / 'weights.npz'
    card_path = model_dir / 'model_card.yaml'
    if not weights_path.exists() or not card_path.exists():
        raise ModelStoreError(f'{model_dir} needs weights.npz and model_card.yaml')
    card = yaml.safe_load(card_path.read_text()) or {}
    if card.get('obs_spec_version') != obs.OBS_SPEC_VERSION:
        raise ModelStoreError(
            f"obs_spec_version {card.get('obs_spec_version')} != this code's "
            f'{obs.OBS_SPEC_VERSION}')
    with np.load(weights_path) as z:
        weights = {k: z[k] for k in z.files}
    return weights, card


def check_modalities(card: dict, *, camera_enabled: bool) -> None:
    modalities = set(card.get('modalities', []))
    unknown = modalities - {'scan', 'ego', 'imu', 'preview', 'race', 'camera'}
    if unknown:
        raise ModelStoreError(f'unknown modality {sorted(unknown)}')
    if 'camera' in modalities and not camera_enabled:
        raise ModelStoreError('modality camera is required by this model but camera_enabled is false')


def golden_error(policy: PolicyNumpy, golden_path: Path) -> float:
    """Largest absolute difference between this policy and the recorded
    PyTorch outputs, replaying the recorded sequences from a reset GRU."""
    with np.load(golden_path) as g:
        names = ('scan', 'ego', 'preview', 'race', 'camera')
        inputs = {k: g[k] for k in names}
        expected = g['action']                 # (S, T, 2) steering, speed
        expected_path = g['ego_path']          # (S, T, 6, 2)
    worst = 0.0
    for seq in range(expected.shape[0]):
        policy.reset()
        for t in range(expected.shape[1]):
            out = policy.step({k: inputs[k][seq, t] for k in names})
            worst = max(worst, abs(out.steering - expected[seq, t, 0]),
                        abs(out.speed - expected[seq, t, 1]),
                        float(np.max(np.abs(out.ego_path - expected_path[seq, t]))))
    policy.reset()
    return worst


def load_approved(model_hash: str, *, models_dir: Path = DEFAULT_MODELS_DIR,
                  camera_enabled: bool = False, driving: bool = True):
    """(PolicyNumpy, card, approval) for an approved model, or ModelStoreError.

    `driving` is False in shadow mode: a shadow-only approval is then enough.
    """
    models_dir = Path(models_dir)
    entry = read_manifest(models_dir).get(model_hash)
    if entry is None:
        raise ModelStoreError(f'model {model_hash!r} is not approved (not in manifest.yaml)')
    approval = entry.get('approval')
    if approval not in APPROVALS:
        raise ModelStoreError(f'model {model_hash!r} has unknown approval {approval!r}')
    if driving and approval != 'full':
        raise ModelStoreError(f'model {model_hash!r} is approved for shadow mode only')
    model_dir = models_dir / model_hash
    actual = weights_hash(model_dir / 'weights.npz')
    if actual != model_hash:
        raise ModelStoreError(f'weights hash {actual} != {model_hash}: the file changed')
    weights, card = load_dir(model_dir)
    check_modalities(card, camera_enabled=camera_enabled)
    return PolicyNumpy(weights, card), card, approval
