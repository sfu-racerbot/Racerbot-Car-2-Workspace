"""The NumPy forward pass (neural_racer/policy_numpy.py). ROS-free.

Runner: python3 -m pytest src/neural_racer/test/test_policy_numpy.py -v   (unsourced)

Oracles: an independent, deliberately naive float64 implementation written
out loop by loop below (it shares no code with the im2col version), plus
invariants of the architecture (bounds, mode gating, camera gating). The
comparison against PyTorch itself is tools/neural_racer/tests/test_export_golden.py.
"""
import math

import nr_testutil
import numpy as np
import pytest

from neural_racer import observation as obs
from neural_racer.policy_numpy import PolicyError, PolicyNumpy, parameter_count


def _obs(rng, mode=1.0, v_cap=0.4, cam_present=0.0):
    o = {'scan': rng.random((4, 270)).astype(np.float32),
         'ego': rng.random(23).astype(np.float32),
         'preview': rng.standard_normal((4, 32)).astype(np.float32),
         'race': rng.random(6).astype(np.float32),
         'camera': rng.standard_normal(64).astype(np.float32)}
    o['ego'][19], o['ego'][20], o['ego'][21] = mode, v_cap, cam_present
    return o


# --- an independent reference --------------------------------------------------

def _conv_naive(x, w, b, stride, pad):
    cout, cin, k = w.shape
    n = x.shape[1]
    nout = (n + 2 * pad - k) // stride + 1
    y = np.zeros((cout, nout))
    for o in range(cout):
        for j in range(nout):
            acc = b[o]
            for c in range(cin):
                for t in range(k):
                    src = j * stride + t - pad
                    if 0 <= src < n:
                        acc += w[o, c, t] * x[c, src]
            y[o, j] = acc
    return y


def _naive_step(w, o, h):
    w = {k: v.astype(np.float64) for k, v in w.items()}
    silu = lambda v: v / (1 + np.exp(-v))           # noqa: E731
    sig = lambda v: 1 / (1 + np.exp(-v))            # noqa: E731
    lin = lambda n, v: w[n + '.weight'] @ v + w[n + '.bias']   # noqa: E731
    x = o['scan'].astype(np.float64)
    for name, s, p in (('scan.c0', 2, 3), ('scan.c1', 2, 2), ('scan.c2', 2, 2), ('scan.c3', 2, 1)):
        x = silu(_conv_naive(x, w[name + '.weight'], w[name + '.bias'], s, p))
    zs = silu(lin('scan.fc', x.reshape(-1)))
    ze = silu(lin('ego.fc1', silu(lin('ego.fc0', o['ego']))))
    y = o['preview'].astype(np.float64)
    for name, s, p in (('prev.c0', 2, 2), ('prev.c1', 2, 1)):
        y = silu(_conv_naive(y, w[name + '.weight'], w[name + '.bias'], s, p))
    mode = o['ego'][19]
    zp = silu(lin('prev.fc', y.reshape(-1))) * mode
    zr = silu(lin('race.fc', o['race'])) * mode
    zc = o['camera'] * o['ego'][21]
    z = np.concatenate([zs, ze, zp, zr])
    z = (z - z.mean()) / math.sqrt(z.var() + 1e-5) * w['fuse.ln.weight'] + w['fuse.ln.bias']
    u = silu(lin('fuse.fc', np.concatenate([z, zc])))
    gi = w['gru.weight_ih'] @ u + w['gru.bias_ih']
    gh = w['gru.weight_hh'] @ h + w['gru.bias_hh']
    r = sig(gi[:128] + gh[:128])
    zz = sig(gi[128:256] + gh[128:256])
    n = np.tanh(gi[256:] + r * gh[256:])
    h2 = (1 - zz) * n + zz * h
    a = lin('head.act', silu(lin('head.fc0', h2)))
    return 0.33 * math.tanh(a[0]), o['ego'][20] * 5.0 * sig(a[1]), lin('aux.path', h2) * 4.0, h2


def test_matches_naive_reference_over_a_sequence():
    w = nr_testutil.random_weights(1)
    p = PolicyNumpy(w)
    rng = np.random.default_rng(2)
    h = np.zeros(128)
    for _ in range(3):                               # three ticks: the GRU state carries
        o = _obs(rng)
        out = p.step(o)
        steer, speed, path, h = _naive_step(w, o, h)
        assert out.steering == pytest.approx(steer, abs=2e-5)   # rad
        assert out.speed == pytest.approx(speed, abs=2e-5)      # m/s
        assert np.allclose(out.ego_path.reshape(-1), path, atol=1e-4)   # m


def test_half_width_runs_and_matches_reference():
    w = nr_testutil.random_weights(3, width=nr_testutil.HALF)
    p = PolicyNumpy(w, nr_testutil.card(width=nr_testutil.HALF))
    o = _obs(np.random.default_rng(4))
    steer, speed, _, _ = _naive_step(w, o, np.zeros(128))
    out = p.step(o)
    assert out.steering == pytest.approx(steer, abs=2e-5)
    assert out.speed == pytest.approx(speed, abs=2e-5)


# --- invariants --------------------------------------------------------------------

def test_outputs_bounded_for_any_input():
    p = PolicyNumpy(nr_testutil.random_weights(5, scale=3.0))   # large weights saturate the heads
    rng = np.random.default_rng(6)
    for i in range(300):
        o = _obs(rng, v_cap=rng.choice([0.0, 0.2, 1.27]))
        o['scan'] *= rng.choice([1.0, 1e3, -1e3])
        o['ego'][:19] *= rng.choice([1.0, 1e3])
        out = p.step(o)
        v_cap = float(o['ego'][20]) * 5.0
        assert abs(out.steering) <= 0.33 + 1e-9
        assert 0.0 <= out.speed <= v_cap + 1e-6
        assert np.all(np.isfinite(out.ego_path)) and np.all(np.isfinite(out.opponents))


def test_zero_speed_cap_gives_zero_speed():
    p = PolicyNumpy(nr_testutil.random_weights(7))
    assert p.step(_obs(np.random.default_rng(8), v_cap=0.0)).speed == 0.0


def test_reactive_mode_ignores_preview_and_race():
    w = nr_testutil.random_weights(9)
    rng = np.random.default_rng(10)
    o1 = _obs(rng, mode=0.0)
    o2 = dict(o1, preview=o1['preview'] * 7 + 3, race=o1['race'] * -5)
    a, b = PolicyNumpy(w).step(o1), PolicyNumpy(w).step(o2)
    assert a.steering == b.steering and a.speed == b.speed


def test_map_mode_uses_preview():
    w = nr_testutil.random_weights(9)
    o1 = _obs(np.random.default_rng(10), mode=1.0)
    o2 = dict(o1, preview=o1['preview'] * 7 + 3)
    assert PolicyNumpy(w).step(o1).steering != PolicyNumpy(w).step(o2).steering


def test_camera_ignored_while_absent():
    w = nr_testutil.random_weights(11)
    w['fuse.fc.weight'][:, -64:] = np.random.default_rng(0).standard_normal((256, 64))
    o1 = _obs(np.random.default_rng(12), cam_present=0.0)
    o2 = dict(o1, camera=o1['camera'] * 9)
    assert PolicyNumpy(w).step(o1).steering == PolicyNumpy(w).step(o2).steering


def test_zero_initialised_camera_starts_identical():
    """The v1 contract (plan A4/A5): with everything the camera reaches at
    zero -- its fusion columns and the two camera flags' ego weights --
    switching a camera on, with any embedding and any age, changes no output
    at all until fine-tuning moves those weights. Oracle: exact equality."""
    w = nr_testutil.random_weights(13)
    o_off = _obs(np.random.default_rng(14), cam_present=0.0)
    o_on = dict(o_off, camera=np.random.default_rng(1).standard_normal(64).astype(np.float32) * 5)
    o_on['ego'] = o_off['ego'].copy()
    o_on['ego'][21], o_on['ego'][22] = 1.0, 0.7
    a, b = PolicyNumpy(w).step(o_off), PolicyNumpy(w).step(o_on)
    assert a.steering == b.steering and a.speed == b.speed
    assert np.array_equal(a.ego_path, b.ego_path)


def test_trained_camera_columns_do_change_output():
    """The other half: once the camera weights are non-zero, the camera matters."""
    w = nr_testutil.random_weights(13)
    w['fuse.fc.weight'][:, -64:] = np.random.default_rng(2).standard_normal((256, 64)) * 0.2
    o_off = _obs(np.random.default_rng(14), cam_present=0.0)
    o_on = dict(o_off, camera=np.ones(64, np.float32))
    o_on['ego'] = o_off['ego'].copy()
    o_on['ego'][21] = 1.0
    assert PolicyNumpy(w).step(o_off).steering != PolicyNumpy(w).step(o_on).steering


def test_gru_state_carries_and_reset_restores():
    p = PolicyNumpy(nr_testutil.random_weights(15))
    o = _obs(np.random.default_rng(16))
    first = p.step(o).steering
    second = p.step(o).steering
    assert first != second                      # memory changed the output
    p.reset()
    assert p.step(o).steering == first


def test_parameter_count_full_width():
    """Closed form for the v1 layout, layer by layer (weights + biases)."""
    scan = (4*32*7 + 32) + (32*64*5 + 64) + (64*64*5 + 64) + (64*64*3 + 64) + (1088*256 + 256)
    ego = (23*64 + 64) + (64*64 + 64)
    prev = (4*32*5 + 32) + (32*32*3 + 32) + (256*64 + 64)
    race = 6*32 + 32
    fuse = 2*416 + (480*256 + 256)            # LayerNorm(416) + Linear(416 + 64 camera -> 256)
    gru = 384*256 + 384*128 + 2*384
    heads = (128*128 + 128) + (2*128 + 2) + (12*128 + 12) + (52*128 + 52)
    n = parameter_count(nr_testutil.random_weights(0))
    assert n == scan + ego + prev + race + fuse + gru + heads == 646_274, n


# --- refusals ----------------------------------------------------------------------

def test_missing_weight_refused():
    w = nr_testutil.random_weights(0)
    del w['gru.weight_hh']
    with pytest.raises(PolicyError, match='missing'):
        PolicyNumpy(w)


def test_non_finite_weight_refused():
    w = nr_testutil.random_weights(0)
    w['head.act.bias'][0] = np.nan
    with pytest.raises(PolicyError, match='non-finite'):
        PolicyNumpy(w)


def test_card_width_mismatch_refused():
    with pytest.raises(PolicyError, match='scan channels'):
        PolicyNumpy(nr_testutil.random_weights(0), nr_testutil.card(width=nr_testutil.HALF))


def test_spec_constants_match_the_layout():
    assert obs.SCAN_CHANNELS == 4 and obs.N_BINS * obs.POOL <= obs.N_BEAMS
    assert obs.EGO_DIM == 23 and obs.EGO['camera_age'] == obs.EGO_DIM - 1
