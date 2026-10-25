"""Model loading and approval (model_store.py, approve_model.py). ROS-free.

Runner: python3 -m pytest src/neural_racer/test/test_model_store.py -v   (unsourced)
Oracle: the refusal rules in docs/superpowers/specs/2026-10-23-neural-racer-plan.md (B0).
"""
import json

import nr_testutil
import numpy as np
import pytest

from neural_racer import approve_model, model_store
from neural_racer.policy_numpy import PolicyNumpy

PASS = json.dumps({'pass': True})
FAIL = json.dumps({'pass': False})


def _exported(tmp_path, reports=None, golden_tamper=False, card_extra=None):
    w = nr_testutil.random_weights(0)
    g = nr_testutil.make_golden(PolicyNumpy(w))
    if golden_tamper:
        g['action'][0, 2, 0] += 1e-3
    inc = tmp_path / 'incoming'
    inc.mkdir(exist_ok=True)
    return nr_testutil.write_model(inc, w, nr_testutil.card(**(card_extra or {})), golden=g,
                                   reports=reports)


def test_full_approval_then_load(tmp_path):
    src, h = _exported(tmp_path, reports={'selection_report.json': PASS, 'referee_report.json': PASS})
    models = tmp_path / 'models'
    assert approve_model.main([str(src), '--models-dir', str(models)]) == 0
    policy, card, approval = model_store.load_approved(h, models_dir=models)
    assert approval == 'full' and isinstance(policy, PolicyNumpy)


def test_unlisted_model_refused(tmp_path):
    with pytest.raises(model_store.ModelStoreError, match='not approved'):
        model_store.load_approved('0123456789abcdef', models_dir=tmp_path)


def test_shadow_only_cannot_drive(tmp_path):
    src, h = _exported(tmp_path)
    models = tmp_path / 'models'
    assert approve_model.main([str(src), '--models-dir', str(models), '--shadow-only']) == 0
    with pytest.raises(model_store.ModelStoreError, match='shadow mode only'):
        model_store.load_approved(h, models_dir=models, driving=True)
    _, _, approval = model_store.load_approved(h, models_dir=models, driving=False)
    assert approval == 'shadow_only'


@pytest.mark.parametrize('reports', [
    {}, {'selection_report.json': PASS}, {'selection_report.json': PASS, 'referee_report.json': FAIL},
    {'selection_report.json': PASS, 'referee_report.json': 'not json'},
])
def test_full_approval_needs_passing_reports(tmp_path, reports):
    src, _ = _exported(tmp_path, reports=reports)
    with pytest.raises(model_store.ModelStoreError, match='not passing'):
        approve_model.approve(src, tmp_path / 'models', shadow_only=False)


def test_golden_mismatch_refused(tmp_path):
    src, _ = _exported(tmp_path, golden_tamper=True)
    with pytest.raises(model_store.ModelStoreError, match='differs'):
        approve_model.approve(src, tmp_path / 'models', shadow_only=True)


def test_golden_at_tolerance_boundary(tmp_path):
    """1e-5 is the limit: exactly at it passes, a step past it does not."""
    w = nr_testutil.random_weights(0)
    p = PolicyNumpy(w)
    g = nr_testutil.make_golden(p)
    path = tmp_path / 'g.npz'
    g['action'][0, 0, 1] += 9e-6
    np.savez(path, **g)
    assert model_store.golden_error(p, path) <= model_store.GOLDEN_ATOL
    g['action'][0, 0, 1] += 2e-6
    np.savez(path, **g)
    assert model_store.golden_error(p, path) > model_store.GOLDEN_ATOL


def test_tampered_weights_refused(tmp_path):
    src, h = _exported(tmp_path)
    models = tmp_path / 'models'
    approve_model.approve(src, models, shadow_only=True)
    with np.load(models / h / 'weights.npz') as z:
        w = {k: z[k] for k in z.files}
    w['head.act.bias'] = w['head.act.bias'] + 1.0
    np.savez(models / h / 'weights.npz', **w)
    with pytest.raises(model_store.ModelStoreError, match='hash'):
        model_store.load_approved(h, models_dir=models, driving=False)


def test_directory_name_must_match_hash(tmp_path):
    w = nr_testutil.random_weights(0)
    src, _ = nr_testutil.write_model(tmp_path, w, nr_testutil.card(),
                                     golden=nr_testutil.make_golden(PolicyNumpy(w)), name='wrong')
    with pytest.raises(model_store.ModelStoreError, match='does not match'):
        approve_model.approve(src, tmp_path / 'models', shadow_only=True)


def test_obs_spec_version_mismatch_refused(tmp_path):
    src, _ = _exported(tmp_path, card_extra={'obs_spec_version': 2})
    with pytest.raises(model_store.ModelStoreError, match='obs_spec_version'):
        approve_model.approve(src, tmp_path / 'models', shadow_only=True)


def test_camera_model_needs_camera(tmp_path):
    src, h = _exported(tmp_path, card_extra={'modalities': ['scan', 'ego', 'camera']})
    models = tmp_path / 'models'
    approve_model.approve(src, models, shadow_only=True)
    with pytest.raises(model_store.ModelStoreError, match='modality camera'):
        model_store.load_approved(h, models_dir=models, driving=False, camera_enabled=False)


def test_unknown_modality_refused():
    with pytest.raises(model_store.ModelStoreError, match='unknown modality'):
        model_store.check_modalities({'modalities': ['lidar3d']}, camera_enabled=True)


def test_cli_reports_refusal(tmp_path, capsys):
    src, _ = _exported(tmp_path)
    assert approve_model.main([str(src), '--models-dir', str(tmp_path / 'm')]) == 1
    assert 'REFUSED' in capsys.readouterr().err
