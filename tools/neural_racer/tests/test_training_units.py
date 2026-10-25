"""Training units: GAE and PPO, the level buffer, the archive, matchmaking,
scripted drivers, the track generators, and the trainer contract.

Runner: PYTHONPATH=<torch> python3 -m pytest tools/neural_racer/tests/test_training_units.py -v
Oracles: hand computations and closed forms named in each test, and the
plan's rules (docs/superpowers/specs/2026-10-23-neural-racer-plan.md, A8 and Phase 4).
"""
import ast
import math
import os
import signal
import subprocess
import sys
from pathlib import Path

import conftest  # noqa: F401
import numpy as np
import pytest
import torch

from neural_racer_train import archive as archive_mod, matchmaking as mm, plr, ppo, scripted, tracks
from neural_racer_train.gpu_sim import race

HERE = Path(__file__).resolve().parent


# --- PPO --------------------------------------------------------------------------------

def test_gae_hand_computed():
    """3 steps, gamma 0.9, lambda 0.5, no terminals: delta_t = r_t + g V_{t+1} - V_t."""
    r = torch.tensor([[1.0], [0.0], [2.0]])
    v = torch.tensor([[0.5], [0.4], [0.3]])
    nv = torch.tensor([0.2])
    f = torch.zeros(3, 1, dtype=torch.bool)
    adv, ret = ppo.gae(r, v, nv, f, f, 0.9, 0.5)
    d2 = 2.0 + 0.9 * 0.2 - 0.3
    d1 = 0.0 + 0.9 * 0.3 - 0.4
    d0 = 1.0 + 0.9 * 0.4 - 0.5
    a2, a1 = d2, d1 + 0.45 * d2
    a0 = d0 + 0.45 * a1
    assert adv[:, 0].tolist() == pytest.approx([a0, a1, a2], abs=1e-6)
    assert torch.allclose(ret, adv + v)


def test_gae_terminal_cuts_bootstrap():
    r = torch.tensor([[1.0], [5.0]])
    v = torch.tensor([[0.0], [0.0]])
    term = torch.tensor([[True], [False]])
    adv, _ = ppo.gae(r, v, torch.tensor([9.0]), term, torch.zeros(2, 1, dtype=torch.bool), 0.9, 1.0)
    assert float(adv[0, 0]) == pytest.approx(1.0, abs=1e-6)       # nothing leaks back across the end


def test_ppo_learns_a_bandit():
    """A 1-step continuous bandit with its optimum at 0.7: the Gaussian mean converges there."""
    torch.manual_seed(0)
    mu = torch.zeros(1, requires_grad=True)
    log_std = torch.full((1,), -0.5, requires_grad=True)
    opt = torch.optim.Adam([mu, log_std], lr=0.05)
    from neural_racer_train.model import gaussian_logp
    for _ in range(300):
        with torch.no_grad():
            a = mu + torch.exp(log_std) * torch.randn(256, 1)
            old = gaussian_logp(a, mu, log_std)
            adv = ppo.normalize(-(a[:, 0] - 0.7) ** 2)
        for _ in range(4):
            loss = ppo.policy_loss(gaussian_logp(a, mu, log_std), old, adv, 0.2)
            opt.zero_grad()
            loss.backward()
            opt.step()
    assert float(mu) == pytest.approx(0.7, abs=0.05)


# --- the level buffer ----------------------------------------------------------------------

def _buffer(n=10, **kw):
    b = plr.LevelBuffer(**kw)
    for i in range(n):
        b.add(plr.LevelRecord(i, f'd{i}', i, score=0.0))
    return b


def test_plr_replay_ratio():
    b = _buffer(replay_prob=0.7)
    draws = b.sample(10_000)
    assert sum(r for _, r in draws) / 10_000 == pytest.approx(0.7, abs=0.02)


def test_plr_prefers_high_score():
    b = _buffer(staleness=0.0)
    b.update_score(3, 5.0)
    picks = [i for i, r in b.sample(5000) if r]
    assert np.bincount(picks, minlength=10).argmax() == 3


def test_plr_capacity_evicts_lowest_score():
    b = _buffer(n=3, capacity=3)
    b.update_score(0, 1.0), b.update_score(1, 0.1), b.update_score(2, 2.0)
    b.add(plr.LevelRecord(9, 'd9', 9))
    assert 1 not in b.levels and len(b.levels) == 3


def test_positive_value_loss():
    assert plr.positive_value_loss([-1.0, 2.0, 0.0, 4.0]) == pytest.approx(1.5)


# --- archive ----------------------------------------------------------------------------------

def test_archive_roundtrip_and_partial_line(tmp_path):
    a = archive_mod.Archive(tmp_path)
    a.add('main0', 200, {'w': torch.ones(2)}, {'steps': 1})
    with open(a.index, 'a') as f:
        f.write('{"id": "main0/00000400", "trunc')                # a writer killed mid-line
    entries = a.entries()
    assert [e.entry_id for e in entries] == ['main0/00000200']
    assert torch.equal(a.load('main0/00000200')['policy']['w'], torch.ones(2))


def test_archive_never_deletes():
    src = (HERE.parent / 'neural_racer_train' / 'archive.py').read_text()
    names = {n.attr if isinstance(n, ast.Attribute) else getattr(n, 'id', '')
             for n in ast.walk(ast.parse(src)) if isinstance(n, (ast.Attribute, ast.Name))}
    assert not names & {'unlink', 'rmtree', 'remove', 'rmdir'}


# --- matchmaking --------------------------------------------------------------------------------

def test_pfsp_hard_closed_form():
    w = mm.pfsp_weights([0.9, 0.5, 0.1], 'hard')
    expect = np.array([0.01, 0.25, 0.81]) / 1.07
    assert np.allclose(w, expect, atol=1e-12)


def test_pfsp_var_closed_form():
    w = mm.pfsp_weights([0.9, 0.5, 0.1], 'var')
    assert np.allclose(w, np.array([0.09, 0.25, 0.09]) / 0.43, atol=1e-12)


@pytest.mark.parametrize('p, share', [(0.5, 0.15), (0.9, 0.03), (0.99, 0.03), (0.2, 0.15), (0.7, 0.09)])
def test_scripted_share_fades(p, share):
    assert mm.scripted_share(p) == pytest.approx(share, abs=1e-12)


def test_main_mix_sums_to_one_and_freed_share_goes_to_pfsp():
    m = mm.role_mix('main0', 0.9)
    assert sum(m.values()) == pytest.approx(1.0) and m['pfsp'] == pytest.approx(0.52)


def _skewed_rates(n=500, strong=10):
    """n archived opponents: the first `strong` beat us, the rest we beat at 0.99."""
    rates = mm.WinRates()
    ids = [f'a{i}' for i in range(n)]
    for k, i in enumerate(ids):
        for _ in range(98):
            rates.record(i, k >= strong)
    return ids, rates


def test_floor_keeps_everyone():
    """PFSP pours its weight onto the 10 opponents we lose to, so only the
    uniform floor keeps the other 490 in play: each must keep at least
    floor / N = 0.10 / 500 of the matches (plan A8). Exact probabilities, not a
    sample, so the guarantee itself is what is checked."""
    ids, rates = _skewed_rates()
    prob = mm.archive_probabilities('main0', ids, rates)
    assert min(prob[i] for i in ids[10:]) >= 0.10 / 500 - 1e-12
    assert min(prob[i] for i in ids[:10]) > 10 * max(prob[i] for i in ids[10:])   # PFSP favours the hard ones


def test_sampling_follows_the_probabilities():
    """Over 200k draws the archive opponents are drawn at their exact
    probabilities: the 10 hard ones' total share within 1% absolute."""
    ids, rates = _skewed_rates()
    prob = mm.archive_probabilities('main0', ids, rates)
    refs = mm.sample_opponents('main0', 200_000, ids, rates, np.random.default_rng(0))
    hard = sum(1 for r in refs if r.kind == 'archive' and r.name in set(ids[:10])) / len(refs)
    assert hard == pytest.approx(sum(prob[i] for i in ids[:10]), abs=0.01)


def test_role_mix_shares():
    rates = mm.WinRates()
    rng = np.random.default_rng(1)
    ids = [f'a{i}' for i in range(20)]
    refs = mm.sample_opponents('main0', 20_000, ids, rates, rng)
    frac = lambda k: sum(r.kind == k for r in refs) / len(refs)   # noqa: E731
    assert frac('self') == pytest.approx(0.35, abs=0.02)
    assert frac('scripted') == pytest.approx(0.15, abs=0.02)      # p_scripted starts at 0.5
    assert frac('archive') == pytest.approx(0.50, abs=0.02)


def test_win_rate_prior():
    r = mm.WinRates()
    for won in (True, True, True, False):
        r.record('x', won)
    assert r.p('x') == pytest.approx(4 / 6)


def test_exploiter_reset_rule():
    assert mm.exploiter_should_reset(0.71, 0) and not mm.exploiter_should_reset(0.70, 0)
    assert mm.exploiter_should_reset(0.1, 2_000_000_000)


# --- scripted drivers ---------------------------------------------------------------------------------

def _priv_straight(n=1, v=2.0):
    p = torch.zeros(n, 286)
    s = 0.25 * torch.arange(1, 65)
    p[:, 0:64] = s / 16                       # centreline straight ahead
    p[:, 128:192] = 0.5                       # 1 m each side
    p[:, 192:256] = 0.5
    p[:, 256] = v / 5
    return p


def test_pursuit_on_a_straight_goes_straight():
    a = scripted.pursue(_priv_straight(), torch.tensor([3.0]), torch.tensor([0.0]), torch.tensor([1.0]))
    assert float(a[0, 0]) == pytest.approx(0.0, abs=1e-6) and float(a[0, 1]) == pytest.approx(3.0, abs=1e-5)


def test_pursuit_offset_left_steers_left():
    a = scripted.pursue(_priv_straight(), torch.tensor([3.0]), torch.tensor([0.5]), torch.tensor([1.0]))
    assert float(a[0, 0]) > 0.01


def test_parked_is_zero_and_speeds_bounded():
    priv = _priv_straight(5)
    params = torch.rand(5, 3)
    for name in scripted.SCRIPTED:
        a = scripted.act(name, priv, torch.rand(5, 4, 270), torch.full((5,), 2.0), params, 1.0)
        assert torch.all(a[:, 0].abs() <= 0.33 + 1e-6) and torch.all((a[:, 1] >= 0) & (a[:, 1] <= 2.0 + 1e-6))
    assert torch.all(scripted.act('parked', priv, None, torch.full((5,), 2.0), params, 1.0) == 0)


def test_unknown_driver_refused():
    with pytest.raises(ValueError, match='unknown scripted driver'):
        scripted.act('rocket', _priv_straight(), None, torch.ones(1), torch.zeros(1, 3), 0.0)


# --- tracks ---------------------------------------------------------------------------------------------

def _longest_straight(c):
    step = 5
    k = np.abs(tracks.signed_curvature(c[::step]))
    best = run = 0
    for v in np.concatenate([k, k]):
        run = run + 1 if v < 0.02 else 0
        best = max(best, run)
    return min(best, len(k)) * step * tracks.SPACING


@pytest.mark.parametrize('kind', ['smooth_loop', 'filleted_polygon'])
def test_generated_tracks_valid(kind):
    """Every track passes check(), re-verified here with an independent radius
    computation (circumradius from three points 0.5 m apart)."""
    for seed in range(40):
        spec = tracks.generate(seed, kind=kind)
        assert tracks.check(spec) == []
        c = spec.centre[::5]
        a, b, d = np.roll(c, 1, 0), c, np.roll(c, -1, 0)
        la, lb, lc = (np.linalg.norm(b - a, axis=1), np.linalg.norm(d - b, axis=1), np.linalg.norm(a - d, axis=1))
        area2 = np.abs((b - a)[:, 0] * (d - a)[:, 1] - (b - a)[:, 1] * (d - a)[:, 0])
        R = la * lb * lc / np.maximum(2 * area2, 1e-12)
        assert R.min() >= 1.3 - 1e-6
        assert np.all(spec.w_left + spec.w_right >= 0.8)


def test_check_catches_tight_corner():
    t = np.linspace(0, 2 * math.pi, 120, endpoint=False)
    c = np.stack([1.0 * np.cos(t), 1.0 * np.sin(t)], 1) * 3       # 3 m circle: fine
    c = np.vstack([c[:60], c[60:] * 0.3])                          # ... with a 0.9 m radius part
    spec = tracks.TrackSpec('t', 0, tracks.resample_closed(c), np.full(1, 0.5), np.full(1, 0.5))
    spec.w_left = np.full(len(spec.centre), 0.5)
    spec.w_right = np.full(len(spec.centre), 0.5)
    assert any('radius' in r for r in tracks.check(spec, tracks.TrackGenConfig(lap_min=1.0)))


def test_check_catches_self_intersection():
    t = np.linspace(0, 2 * math.pi, 600, endpoint=False)
    eight = np.stack([8 * np.sin(t), 8 * np.sin(t) * np.cos(t)], 1)  # a figure-eight crosses itself
    c = tracks.resample_closed(eight)
    spec = tracks.TrackSpec('t', 0, c, np.full(len(c), 0.5), np.full(len(c), 0.5))
    assert any('touches itself' in r for r in tracks.check(spec, tracks.TrackGenConfig(min_radius=0.1)))


def test_fillet_radius_closed_form():
    """A 10 m square with 2 m fillets: curvature 1/2 on the arcs, 0 on the straights."""
    sq = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=float)
    path = tracks._fillet_path(sq, np.full(4, 2.0))
    c = tracks.resample_closed(path)
    k = np.abs(tracks.signed_curvature(c[::3]))
    arcs = k[k > 0.1]
    assert np.median(arcs) == pytest.approx(0.5, rel=0.01)
    assert np.mean(k < 0.01) > 0.5                                 # most of the lap is straight


def test_polygons_have_straights():
    lengths = [_longest_straight(tracks.generate(s, kind='filleted_polygon').centre) for s in range(60)]
    assert np.median(lengths) >= 3.0, np.median(lengths)


def test_source_shares():
    rng_kinds = []
    for s in range(400):
        rng = np.random.default_rng(s)
        cfg = tracks.TrackGenConfig()
        rng_kinds.append(rng.choice(['smooth_loop', 'filleted_polygon'],
                                    p=np.array(cfg.shares[:2]) / sum(cfg.shares[:2])))
    frac = np.mean(np.array(rng_kinds) == 'smooth_loop')
    assert frac == pytest.approx(0.5, abs=0.07)       # 40/40 of the generated share


def test_mutation_keeps_tracks_valid():
    rng = np.random.default_rng(0)
    spec = tracks.generate(3)
    for _ in range(10):
        spec = tracks.mutate(spec, rng)
        assert tracks.check(spec) == []


def test_heldout_seed_refused_for_training():
    from neural_racer_train import learn
    with pytest.raises(ValueError, match='held-out'):
        learn._generate(learn.HELDOUT_SEED_BASE + 5)


def test_start_grid_cars_apart():
    spec = tracks.generate(1)
    poses, _ = tracks.start_grid(spec, 4, np.random.default_rng(0))
    d = [np.hypot(*(poses[i, :2] - poses[j, :2])) for i in range(4) for j in range(i + 1, 4)]
    assert min(d) > 0.535 + 0.2                    # a body length plus 20 cm of air, bends included


# --- the trainer contract ---------------------------------------------------------------------------------

def _stage(tmp_path, *extra):
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(HERE.parent), os.environ.get('PYTHONPATH', '')]),
               OMP_NUM_THREADS='2')
    cmd = [sys.executable, '-m', 'neural_racer_train.teacher', '--run-dir', str(tmp_path / 'run'),
           '--device', 'cpu', '--num-envs', '2', '--rollout', '4', '--pool-slots', '2', *extra]
    return subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def test_sigusr1_checkpoints_and_exits_85(tmp_path):
    p = _stage(tmp_path, '--steps', '1e12', '--checkpoint-every', '1000')
    import time
    deadline = time.time() + 400
    while time.time() < deadline and not (tmp_path / 'run' / 'metrics.jsonl').exists():
        if p.poll() is not None:
            break
        time.sleep(1)
    p.send_signal(signal.SIGUSR1)
    assert p.wait(timeout=600) == 85
    assert (tmp_path / 'run' / 'checkpoints' / 'latest.pt').exists()


def test_budget_done_exits_0_and_resume_continues(tmp_path):
    p = _stage(tmp_path, '--steps', '8', '--log-every', '1')
    assert p.wait(timeout=600) == 0
    first = torch.load(tmp_path / 'run' / 'checkpoints' / 'latest.pt', weights_only=False)['steps']
    p = _stage(tmp_path, '--steps', '16', '--log-every', '1', '--resume')
    assert p.wait(timeout=600) == 0
    second = torch.load(tmp_path / 'run' / 'checkpoints' / 'latest.pt', weights_only=False)['steps']
    assert first == 8 and second == 16                # resumed, not restarted


def test_place_reward_closed_form():
    cfg = race.RewardConfig()
    z = torch.zeros(1, 1)
    f = torch.zeros(1, 1, dtype=torch.bool)
    r = race.reward(cfg, torch.tensor([[0.1]]), torch.zeros(1, 1, dtype=torch.long), f, f, f, f, z, 0.025)
    assert float(r) == pytest.approx(0.1, abs=1e-6)   # 0.1 m at v_ref 4 m/s over 25 ms = one unit of 0.1
    t = torch.ones(1, 1, dtype=torch.bool)
    r = race.reward(cfg, z, torch.ones(1, 1, dtype=torch.long), t, t, f, f, z, 0.025)
    assert float(r) == pytest.approx(2.0 - 20.0, abs=1e-6)
