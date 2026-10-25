"""Performance gate: the on-car tick fits its budget. ROS-free.

Runner: python3 -m pytest src/neural_racer/test/test_latency.py -v   (unsourced, on the Jetson)

Budget: p99 of observation + forward pass + safety wrapper <= 5 ms, 20% of
the 25 ms tick (docs/superpowers/specs/2026-10-23-neural-racer-plan.md,
Phase 2). Measured reference: the forward pass alone at 3.50 ms p99 on car
1's Jetson, one core, 2026-10-23. This is a timing gate, not a behaviour
test; Phase 6 repeats it with the whole race stack running, and that number
decides between scan_width full and half.
"""
import time

import nr_testutil
import numpy as np

from neural_racer import observation as obs
from neural_racer.policy_numpy import PolicyNumpy
from neural_racer.safety_wrapper import SafetyWrapper, WrapperConfig, scan_angles

BUDGET_P99_MS = 5.0


def _tick_times(width, n=1500):
    policy = PolicyNumpy(nr_testutil.random_weights(0, width=width))
    wrapper = SafetyWrapper(WrapperConfig())
    angles = scan_angles(nr_testutil.UST_ANGLE_MIN, nr_testutil.UST_INCREMENT, 1081)
    history = obs.History()
    rng = np.random.default_rng(0)
    scans = rng.uniform(0.3, 12.0, (64, 1081))
    times = []
    for i in range(n + 100):
        r = scans[i % 64]
        t0 = time.perf_counter()
        enc, valid = obs.pool_scan(r, 0.02, 30.0)
        history.push_tick(enc, valid, 1.0, (0.1, 0.0, 0.2))
        o = obs.build_observation(history, imu_present=True, mode=0, v_cap=1.0)
        out = policy.step(o)
        res = wrapper.apply(out.steering, out.speed, out.ego_path, r, angles, 0.02, 30.0, 0.0, 0.025)
        history.push_command(res.steering, res.speed)
        if i >= 100:
            times.append(time.perf_counter() - t0)
    return np.array(times) * 1e3


def test_full_width_tick_within_budget():
    t = _tick_times(nr_testutil.FULL)
    p99 = float(np.percentile(t, 99))
    print(f'full width: median {np.median(t):.2f} ms, p99 {p99:.2f} ms')
    assert p99 <= BUDGET_P99_MS, f'p99 {p99:.2f} ms over the {BUDGET_P99_MS} ms budget'


def test_half_width_is_faster():
    full = np.median(_tick_times(nr_testutil.FULL, 400))
    half = np.median(_tick_times(nr_testutil.HALF, 400))
    assert half < full
