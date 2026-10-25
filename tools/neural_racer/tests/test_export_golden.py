"""PyTorch training network vs the car's NumPy forward pass, through export.

Runner: PYTHONPATH=<torch> python3 -m pytest tools/neural_racer/tests/test_export_golden.py -v

Oracle: PyTorch's own forward pass -- an independent implementation from
policy_numpy.py's im2col version. Then the real approve_model, unmodified,
must accept the exported directory (it re-runs the golden check itself).
"""
import json

import conftest  # noqa: F401
import numpy as np
import pytest
import torch

from neural_racer import approve_model, model_store, observation as obs
from neural_racer.policy_numpy import PolicyNumpy
from neural_racer_train import export
from neural_racer_train.model import FULL, HALF, PolicyNet


def _randomized(width=FULL, seed=0, recurrent=True):
    torch.manual_seed(seed)
    p = PolicyNet(width, recurrent)
    with torch.no_grad():
        for name, v in p.named_parameters():       # move off PyTorch's init so biases matter
            if 'camera' not in name:
                v.add_(torch.randn_like(v) * 0.05)
        ln = p.fuse['ln'].normalized_shape[0]
        p.fuse['fc'].weight[:, ln:] = 0.0
        p.ego['fc0'].weight[:, 21:23] = 0.0
    return p


@pytest.mark.parametrize('width', [FULL, HALF])
def test_numpy_reproduces_pytorch(width):
    policy = _randomized(width)
    g = export.golden(policy, export.synthetic_inputs())
    w = export.weights_of(policy)
    err = 0.0
    numpy_policy = PolicyNumpy(w)
    for s in range(g['action'].shape[0]):
        numpy_policy.reset()
        for t in range(g['action'].shape[1]):
            out = numpy_policy.step({k: g[k][s, t] for k in ('scan', 'ego', 'preview', 'race', 'camera')})
            err = max(err, abs(out.steering - g['action'][s, t, 0]), abs(out.speed - g['action'][s, t, 1]),
                      float(np.abs(out.ego_path - g['ego_path'][s, t]).max()))
    assert err <= 1e-5, err


def test_state_dict_names_are_policy_numpy_names():
    w = export.weights_of(PolicyNet())
    assert 'log_std' not in w
    PolicyNumpy(w)                                    # raises PolicyError on any missing name


def test_camera_weights_start_at_zero():
    p = PolicyNet()
    ln = p.fuse['ln'].normalized_shape[0]
    assert torch.all(p.fuse['fc'].weight[:, ln:] == 0)
    assert torch.all(p.ego['fc0'].weight[:, obs.EGO['camera_present']:obs.EGO['camera_age'] + 1] == 0)


def test_mlp_ablation_ignores_memory():
    p = _randomized(recurrent=False)
    o = {k: torch.as_tensor(v[0, :1]) for k, v in export.synthetic_inputs().items()}
    a1, h, _, _ = p(o, torch.zeros(1, 128))
    a2, _, _, _ = p(o, h)                            # a non-zero hidden state, ignored
    assert torch.equal(a1, a2)


def test_exported_dir_passes_approve_model(tmp_path):
    policy = _randomized()
    d = export.export(policy, tmp_path / 'exports', run_name='test', snapshot_id='0')
    for name in ('selection_report.json', 'referee_report.json'):
        (d / name).write_text(json.dumps({'pass': True}))
    h = approve_model.approve(d, tmp_path / 'models', shadow_only=False)
    loaded, card, approval = model_store.load_approved(h, models_dir=tmp_path / 'models')
    assert approval == 'full' and card['parameters'] == 646_274


def test_manual_gru_equals_grucell():
    """Oracle: PyTorch's own nn.GRUCell (the manual form exists only so vmap batches it)."""
    from neural_racer_train.model import gru_cell
    torch.manual_seed(0)
    cell = torch.nn.GRUCell(256, 128)
    x, h = torch.randn(7, 256), torch.randn(7, 128)
    assert torch.allclose(gru_cell(x, h, cell), cell(x, h), atol=1e-6)


def test_stacked_opponents_match_individual_policies():
    """16 stacked opponents in one vmap call give each policy's own actions."""
    from neural_racer_train.opponents import StackedOpponents
    pols = [_randomized(seed=s) for s in range(4)]
    stacked = StackedOpponents([p.state_dict() for p in pols])
    inputs = export.synthetic_inputs(seqs=3, steps=1)
    o = {k: torch.as_tensor(v[:, 0]) for k, v in inputs.items()}           # B = 3
    ob = {k: v.unsqueeze(0).expand(4, *v.shape) for k, v in o.items()}
    a, _ = stacked.act(ob, torch.zeros(4, 3, 128))
    for m, p in enumerate(pols):
        ref, _, _, _ = p(o, torch.zeros(3, 128))
        assert torch.allclose(a[m], ref, atol=1e-5)
