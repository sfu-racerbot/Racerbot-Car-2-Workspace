"""GPU simulator dynamics and actuation vs. the CPU referee's own code.

Runner: PYTHONPATH=<torch> python3 -m pytest tools/neural_racer/tests/test_dynamics_parity.py -v
(needs PyTorch, and the pinned gym in .sim/ from tools/f1tenth_sim/setup.sh; runs on CPU).

Oracles, all independent implementations: gym's numba vehicle_dynamics_st,
steering_constraint, accl_constraints, pid_accl; sim_fidelity's
SteeringServo, GripEnvelope and FidelityPlant._measured_speed_loop. Plan,
Phase 3: no training result counts until these pass.
"""
import math

import conftest  # noqa: F401
import numpy as np
import pytest
import torch

from neural_racer_train import fidelity
from neural_racer_train.gpu_sim import actuation, dynamics
from neural_racer_train.gpu_sim.params import BatchParams, CarParams

pytestmark = pytest.mark.skipif(not fidelity.gym_available(),
                                reason='needs the pinned gym: run tools/f1tenth_sim/setup.sh')


@pytest.fixture(scope='module')
def gym():
    fidelity.ensure_paths()
    from f1tenth_gym.envs.dynamic_models import single_track, utils
    from f1tenth_gym.envs.dynamic_models import F1TENTH_VEHICLE_PARAMETERS
    return single_track, utils, F1TENTH_VEHICLE_PARAMETERS


def _gym_param_vector(p: CarParams):
    """The 16-entry vector gym's vehicle_dynamics_st reads, in its order."""
    return np.array([p.mu, p.C_Sf, p.C_Sr, p.lf, p.lr, p.h, p.m, p.I,
                     p.steering_max_right, p.steering_max_left,
                     -p.steering_rate_max, p.steering_rate_max,
                     p.v_switch, p.a_max, p.gym_v_min, p.gym_v_max], dtype=np.float64)


def _states(n, rng):
    x = np.zeros((n, 7))
    x[:, 0:2] = rng.uniform(-5, 5, (n, 2))
    x[:, 2] = rng.uniform(-0.45, 0.45, n)
    x[:, 3] = np.concatenate([rng.uniform(-0.3, 0.49, n // 4), rng.uniform(0.51, 7.0, n - n // 4)])
    x[:, 4] = rng.uniform(-math.pi, math.pi, n)
    x[:, 5] = rng.uniform(-3, 3, n)
    x[:, 6] = rng.uniform(-0.3, 0.3, n)
    u = np.stack([rng.uniform(-5, 5, n), rng.uniform(-12, 12, n)], axis=1)
    return x, u


def test_derivative_matches_gym_both_branches(gym):
    single_track, _, _ = gym
    p = CarParams()
    vec = _gym_param_vector(p)
    rng = np.random.default_rng(0)
    x, u = _states(400, rng)
    bp = BatchParams.constant(p, 400, 'cpu', dtype=torch.float64)
    ours = dynamics.st_derivative(torch.tensor(x), torch.tensor(u), bp).numpy()
    theirs = np.array([single_track.vehicle_dynamics_st(x[i], u[i], vec) for i in range(400)])
    assert np.allclose(ours, theirs, rtol=1e-9, atol=1e-9)


def test_constraints_match_gym(gym):
    _, utils, _ = gym
    rng = np.random.default_rng(1)
    T = lambda v: torch.tensor(v, dtype=torch.float64)   # noqa: E731
    for _ in range(300):
        d, sv = rng.uniform(-0.6, 0.6), rng.uniform(-6, 6)
        ours = dynamics.steering_constraint(T(d), T(sv), -0.388, 0.401, -3.2, 3.2)
        assert float(ours) == pytest.approx(utils.steering_constraint(d, sv, -0.388, 0.401, -3.2, 3.2), abs=1e-12)
        v, a = rng.uniform(-6, 22), rng.uniform(-15, 15)
        ours = dynamics.accl_constraints(T(v), T(a), 7.319, 5.4, -5.0, 20.0)
        assert float(ours) == pytest.approx(utils.accl_constraints(v, a, 7.319, 5.4, -5.0, 20.0), abs=1e-12)
        cmd = rng.uniform(-6, 7)
        ours = dynamics.pid_accl(T(cmd), T(v), 5.4, 20.0, -5.0)
        assert float(ours) == pytest.approx(utils.pid_accl(cmd, v, 5.4, 20.0, -5.0), abs=1e-12)


def test_rk4_matches_gym_integrator(gym):
    single_track, _, _ = gym
    fidelity.ensure_paths()
    from f1tenth_gym.envs.integrators import rk4_integration
    p = CarParams()
    vec = _gym_param_vector(p)
    x, u = _states(50, np.random.default_rng(2))
    bp = BatchParams.constant(p, 50, 'cpu', dtype=torch.float64)
    ours = dynamics.rk4(torch.tensor(x), torch.tensor(u), bp, 0.005).numpy()
    theirs = np.array([rk4_integration(single_track.vehicle_dynamics_st, x[i], u[i], 0.005, vec)
                       for i in range(50)])
    assert np.allclose(ours, theirs, rtol=1e-9, atol=1e-9)


def test_servo_matches_sim_fidelity():
    fidelity.ensure_paths()
    from sim_fidelity.actuation import SteeringServo
    p = CarParams()
    servo = SteeringServo(gain=p.servo_gain, rate_max=p.steering_rate_max,
                          angle_min=p.steering_max_right, angle_max=p.steering_max_left)
    bp = BatchParams.constant(p, 1, 'cpu')
    rng = np.random.default_rng(3)
    for _ in range(200):
        t, c = rng.uniform(-0.6, 0.6), rng.uniform(-0.4, 0.4)
        ours = float(actuation.servo_velocity(torch.tensor([t]), torch.tensor([c]), bp))
        assert ours == pytest.approx(servo.velocity(t, c), abs=1e-6)


def test_speed_loop_matches_plant():
    """Oracle: FidelityPlant._measured_speed_loop's body, called with a stub
    `self` carrying the same calibration and gym parameters."""
    fidelity.ensure_paths()
    from types import SimpleNamespace
    from sim_fidelity.plant import FidelityPlant
    p = CarParams()
    stub = SimpleNamespace(
        calibration=SimpleNamespace(
            speed_loop_accel_gain=p.speed_loop_accel_gain, speed_loop_accel_max=p.speed_loop_accel_max,
            speed_loop_brake_gain=p.speed_loop_brake_gain, speed_loop_brake_max=p.speed_loop_brake_max),
        params=SimpleNamespace(a_max=p.a_max, v_max=p.gym_v_max, v_min=p.gym_v_min))
    bp = BatchParams.constant(p, 1, 'cpu')
    rng = np.random.default_rng(4)
    for _ in range(300):
        cmd, meas = rng.uniform(-2, 6.5), rng.uniform(-1, 6.5)
        ours = float(actuation.measured_speed_loop(torch.tensor([cmd]), torch.tensor([meas]), bp))
        assert ours == pytest.approx(FidelityPlant._measured_speed_loop(stub, cmd, meas), abs=1e-5)


def test_grip_matches_sim_fidelity():
    fidelity.ensure_paths()
    from sim_fidelity.grip import GripEnvelope
    p = CarParams()
    env = GripEnvelope(friction_accel_limit=p.a_max, wheelbase=p.wheelbase,
                       longitudinal_coupling=p.longitudinal_grip_coupling)
    bp = BatchParams.constant(p, 1, 'cpu')
    rng = np.random.default_rng(5)
    for _ in range(300):
        s, v, a = rng.uniform(-0.45, 0.45), rng.uniform(0.0, 6.5), rng.uniform(-6, 6)
        ours = float(actuation.grip_clamp(torch.tensor([s]), torch.tensor([v]), torch.tensor([a]), bp))
        assert ours == pytest.approx(env.apply(s, v, a), abs=1e-6)


def test_delay_line_matches_transport_delay():
    fidelity.ensure_paths()
    from sim_fidelity.actuation import TransportDelay
    for ticks in (0, 1, 3):
        ref = TransportDelay(ticks)
        line = actuation.DelayLine((1,), 4, torch.tensor([ticks]))
        for k in range(12):
            x = float(k + 1)
            assert float(line.push(torch.tensor([x]))) == pytest.approx(ref.push(x))


def test_car1_overrides_applied():
    """Oracle: vesc.yaml speed_max 23250 eRPM at car 1's 3650 gain, and car 1's
    2026-10-21 circles (docs/hardware-reference.md)."""
    p = CarParams()
    assert p.speed_max == pytest.approx(23250 / 3650, abs=1e-12)
    assert (p.steering_max_left, p.steering_max_right) == (0.401, -0.388)
