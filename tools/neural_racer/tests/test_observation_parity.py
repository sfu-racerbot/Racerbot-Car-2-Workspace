"""The training side's observation and safety wrapper vs. the car's own code.

Runner: PYTHONPATH=<torch> python3 -m pytest tools/neural_racer/tests/test_observation_parity.py -v

Oracle: src/neural_racer's NumPy build_observation and SafetyWrapper.apply --
an independent, per-tick implementation of the same spec. Duplicated
preprocessing that drifts is a classic silent sim-to-real bug (plan B0).
"""
import math

import conftest  # noqa: F401
import numpy as np
import pytest
import torch

from neural_racer import observation as obs
from neural_racer.safety_wrapper import SafetyWrapper, WrapperConfig
from neural_racer_train.gpu_sim import observation_torch as ot
from neural_racer_train.gpu_sim.safety_torch import SafetyTorch, nominal_angles


def _ranges(rng, n):
    r = rng.uniform(0.0, 14.0, (n, obs.N_BEAMS))
    r[rng.random(r.shape) < 0.05] = math.nan
    r[rng.random(r.shape) < 0.03] = math.inf
    r[rng.random(r.shape) < 0.03] = 0.01
    return r


def test_pool_scan_matches():
    r = _ranges(np.random.default_rng(0), 64)
    enc_t, val_t = ot.pool_scan(torch.tensor(r, dtype=torch.float64), 0.02, 30.0)
    for i in range(64):
        enc, val = obs.pool_scan(r[i], 0.02, 30.0)
        assert np.allclose(enc_t[i].numpy(), enc, atol=1e-6)
        assert np.array_equal(val_t[i].numpy(), val)


@pytest.mark.parametrize('ticks', [1, 2, 3, 5])
def test_history_and_build_match_over_a_sequence(ticks):
    """E x C = 3 x 2 cars, `ticks` pushes after a reset: covers the short-history
    padding (1 and 2 ticks) and a full rolling history."""
    rng = np.random.default_rng(ticks)
    E, C = 3, 2
    hist_t = ot.ObsHistory(E, C)
    hist_t.reset(torch.ones(E, C, dtype=torch.bool))
    hist_n = [[obs.History() for _ in range(C)] for _ in range(E)]
    for _ in range(ticks):
        r = _ranges(rng, E * C).reshape(E, C, -1)
        speed = rng.uniform(-1, 6, (E, C))
        imu = rng.normal(0, 1, (E, C, 3))
        enc, val = ot.pool_scan(torch.tensor(r, dtype=torch.float32), 0.02, 30.0)
        hist_t.push_tick(enc, val, torch.tensor(speed, dtype=torch.float32), torch.tensor(imu, dtype=torch.float32))
        steer_c, speed_c = rng.uniform(-0.33, 0.33, (E, C)), rng.uniform(0, 4, (E, C))
        for e in range(E):
            for c in range(C):
                en, vn = obs.pool_scan(r[e, c], 0.02, 30.0)
                hist_n[e][c].push_tick(en, vn, speed[e, c], tuple(imu[e, c]))
        # the command recorded *after* this tick's observation, as the node does
        mode = torch.tensor(rng.integers(0, 2, (E, C)))
        v_cap = torch.full((E, C), 1.7)
        imu_present = torch.tensor(rng.random((E, C)) < 0.7)
        preview = torch.tensor(rng.normal(0, 1, (E, C, 4, 32)), dtype=torch.float32)
        race = torch.tensor(rng.random((E, C, 6)), dtype=torch.float32)
        out = ot.build(hist_t, imu_present, mode, v_cap, preview, race)
        for e in range(E):
            for c in range(C):
                ref = obs.build_observation(
                    hist_n[e][c], imu_present=bool(imu_present[e, c]), mode=int(mode[e, c]), v_cap=1.7,
                    preview=preview[e, c].numpy(), race=race[e, c].numpy())
                for k in ('scan', 'ego', 'preview', 'race', 'camera'):
                    assert np.allclose(out[k][e, c].numpy(), ref[k], atol=1e-6), (k, e, c)
        hist_t.push_command(torch.tensor(steer_c, dtype=torch.float32), torch.tensor(speed_c, dtype=torch.float32))
        for e in range(E):
            for c in range(C):
                hist_n[e][c].push_command(steer_c[e, c], speed_c[e, c])


def test_reset_only_resets_masked_cars():
    hist = ot.ObsHistory(2, 1)
    enc = torch.ones(2, 1, 270)
    hist.push_tick(enc, enc, torch.ones(2, 1), torch.ones(2, 1, 3))
    hist.reset(torch.tensor([[True], [False]]))
    assert hist.count.tolist() == [[0], [1]] and hist.scans[0].abs().sum() == 0
    assert hist.scans[1].abs().sum() > 0


def test_safety_matches_numpy_wrapper():
    cfg = WrapperConfig(max_speed=4.0)
    angles = nominal_angles()
    st = SafetyTorch(cfg, angles)
    sw = SafetyWrapper(cfg)
    rng = np.random.default_rng(7)
    n = 400
    # Mostly open scans, so the cornering cap binds and is really compared
    # (fully random scans brake every sample to zero, which hid a missing tan()).
    r = rng.uniform(4.0, 12.0, (n, obs.N_BEAMS))
    near = rng.random(n) < 0.3
    r[near] = rng.uniform(0.2, 12.0, (int(near.sum()), obs.N_BEAMS))
    r[: n // 8, :400] = math.nan                     # some scans over the invalid limit
    steer = rng.uniform(-0.5, 0.5, n)
    speed = rng.uniform(-0.5, 4.5, n)
    prev = rng.uniform(-0.33, 0.33, n)
    s_t, v_t, stop_t, _ = st.apply(
        torch.tensor(steer, dtype=torch.float32).view(n, 1), torch.tensor(speed, dtype=torch.float32).view(n, 1),
        torch.tensor(r, dtype=torch.float32).view(n, 1, -1), 0.02, 30.0,
        torch.tensor(prev, dtype=torch.float32).view(n, 1), 0.025, torch.full((n, 1), 4.0))
    binding = []
    for i in range(n):
        ref = sw.apply(steer[i], speed[i], np.zeros((6, 2)), r[i], angles, 0.02, 30.0, prev[i], 0.025)
        binding.append(ref.binding)
        assert bool(stop_t[i, 0]) == ref.stopped
        assert float(v_t[i, 0]) == pytest.approx(ref.speed, abs=1e-4)        # m/s, float32 vs float64
        assert float(s_t[i, 0]) == pytest.approx(ref.steering, abs=1e-6)     # rad
    # every cap was actually exercised, so none of them can be wrong unnoticed
    assert {'curvature', 'braking', 'network'} <= set(binding), set(binding)
