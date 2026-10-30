"""True-state IMU physics; run with python3 -m pytest src/racerbot_sim/test/.

Mutation checks: removing the forward lever arm fails the circle's forward
mean; subtracting bias fails test_fault_outputs[bias]; using previous speed
in v*r fails test_accelerating_turn_uses_current_speed. Constant ideal readings,
a step stub, a 10% gravity change and an unchecked fault setter also fail.
"""

import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu_bridge.convert import G
from imu_bridge.imu_health import ImuHealth, STATE_OK, STATE_WAITING, STATE_FAULT
from racerbot_sim import sim_bridge as sim, tracks
from racerbot_sim.sim_imu import (
    GRAVITY, BNO_X_OFFSET, VESC_X_OFFSET, BNO_NOISE, VESC_NOISE, FAULTS,
    ImuNoise, SimImu, ideal_reading,
)

ZERO_NOISE = ImuNoise(0.0, 0.0)


def noiseless(**kwargs):
    return SimImu(np.random.default_rng(42), ZERO_NOISE, ZERO_NOISE, **kwargs)


@pytest.mark.parametrize('speed,previous,r,previous_r,dt,x', [
    (2.0, 2.0, 0.5, 0.5, 0.025, 0.36),
    (2.0, 1.5, 0.0, 0.0, 0.025, 0.36),
    (2.0, 2.0, 0.5, 0.25, 0.025, 0.36),
    (-2.0, -2.0, -0.5, -0.5, 0.025, 0.0),
])
def test_ideal_reading_closed_forms(speed, previous, r, previous_r, dt, x):
    acc, gyro = ideal_reading(speed, previous, r, previous_r, dt, x)
    # oracle: rigid-body acceleration at (x,0,0), including centripetal and
    # tangential lever arms; specific force includes upward standard gravity.
    assert acc == ((speed - previous) / dt - x * r*r,
                   speed * r + x * (r - previous_r) / dt, G)
    assert gyro == (0.0, 0.0, r)


def test_accelerating_turn_uses_current_speed():
    acc, gyro = ideal_reading(2.0, 1.5, 0.5, 0.5, 0.025, 0.0)
    # oracle: controller ruling: dv/dt=20 m/s²; current v*r=1 m/s²,
    # whereas the previous speed would incorrectly give 0.75 m/s².
    assert acc == (20.0, 1.0, G)
    assert gyro == (0.0, 0.0, 0.5)


@pytest.mark.parametrize('index,value', [(4, 0.0), (4, -0.025)] + [
    (i, bad) for i in range(6) for bad in (float('nan'), float('inf'), -float('inf'))])
def test_ideal_reading_invalid_inputs(index, value):
    args = [2.0, 1.5, 0.5, 0.25, 0.025, 0.36]
    args[index] = value
    # oracle: interface rejects nonpositive dt and every non-finite input.
    with pytest.raises(ValueError, match='finite|dt'):
        ideal_reading(*args)


def test_constants_match_physical_sources():
    config = Path(__file__).resolve().parents[2] / 'imu_bridge/config/vehicle_state.yaml'
    params = yaml.safe_load(config.read_text())['vehicle_state_node']['ros__parameters']
    # oracle: independent consumer configuration and conversion constant.
    assert GRAVITY == G
    assert BNO_X_OFFSET == params['bno_x_offset']
    assert VESC_X_OFFSET == params['vesc_x_offset']


def test_first_tick_has_zero_derivatives():
    sample = noiseless().step(2.0, 0.5, 0.025)
    # oracle: specified first tick repeats speed and rate, leaving only the
    # steady centripetal acceleration at each sensor's configured position.
    assert sample.bno_acc == (-BNO_X_OFFSET * 0.5**2, 1.0, G)
    assert sample.vesc_acc == (-VESC_X_OFFSET * 0.5**2, 1.0, G)
    assert sample.bno_gyro == sample.vesc_gyro == (0.0, 0.0, 0.5)


@pytest.fixture
def gym_bridge(tmp_path):
    sim.add_sim_python_path()
    try:
        import f1tenth_gym  # noqa: F401
    except ModuleNotFoundError as exc:
        if exc.name != 'f1tenth_gym':
            raise
        pytest.skip('needs .sim/ gym checkout (docs/ros-simulator.md); remove skip when available')
    # A generated circular corridor with enough room for the acceleration
    # transient. Contact is checked against the rasterized body, never gym's flag.
    angle = np.linspace(-np.pi/2, 3*np.pi/2, 320, endpoint=False)
    centerline = 2.5 * np.column_stack((np.cos(angle), np.sin(angle)))
    path = tracks.write_track(tmp_path, 'imu_circle', centerline, corridor_width=3.0)
    bridge = sim.SimBridge(sim.SimConfig(str(path), centerline, odom_speed_noise_std=0.0))
    bridge.reset(poses=[[0.0, -2.5, 0.0]])
    yield bridge
    bridge.close()


def test_gym_circle_geometry(gym_bridge):
    bridge, imu = gym_bridge, noiseless()
    states, samples, previous_rates = [], [], []
    previous_rate = bridge.agent_state(0).yaw_rate
    for tick in range(200):  # 5 s: settle 3 s, fit the final 2 s
        bridge.step_cars([(0.15, 2.0)])
        state = bridge.agent_state(0)
        sample = imu.step(state.speed, state.yaw_rate, bridge.config.control_dt)
        # oracle: chassis-vs-occupancy geometry, not the gym collision flag.
        assert not bridge.body_contact(0), (tick, state)
        if tick >= 120:
            states.append(state)
            samples.append(sample)
            previous_rates.append(previous_rate)
        previous_rate = state.yaw_rate
    xy = np.array([(s.x, s.y) for s in states])
    cx, cy, c = np.linalg.lstsq(np.column_stack((2*xy, np.ones(len(xy)))),
                               np.sum(xy**2, axis=1), rcond=None)[0]
    radius = np.sqrt(c + cx*cx + cy*cy)
    for state, sample, prev_rate in zip(states, samples, previous_rates):
        # oracle: controller ruling -- instantaneous wiring uses the true
        # state even though the gym rate oscillates around geometric v/R.
        assert sample.bno_gyro[2] == pytest.approx(state.yaw_rate, abs=1e-12)  # rad/s
        assert sample.bno_acc[1] == pytest.approx(
            state.speed * state.yaw_rate
            + BNO_X_OFFSET * (state.yaw_rate-prev_rate)/bridge.config.control_dt,
            abs=1e-9)  # m/s²
    speed = np.mean([s.speed for s in states])
    rates = np.array([s.yaw_rate for s in states])
    # oracle: fitted position geometry gives mean r=v/R and a_y=v²/R.
    # Controller ruling permits window means to average the plant oscillation.
    assert np.mean([s.bno_gyro[2] for s in samples]) == pytest.approx(speed/radius, rel=0.02)
    assert np.mean([s.bno_acc[1] for s in samples]) == pytest.approx(speed**2/radius, rel=0.02)
    assert np.mean([s.bno_acc[0] for s in samples]) == pytest.approx(
        -BNO_X_OFFSET * np.mean(rates**2), abs=0.05)  # m/s²
    print(f'circle R={radius:.9f} m; 200 ticks without body_contact; '
          f'true rate mean={rates.mean():.9f}, peak-to-peak={np.ptp(rates):.9f}, '
          f'std={rates.std():.9f}, v/R={speed/radius:.9f} rad/s')


def test_gym_speed_ramp(gym_bridge):
    bridge, imu = gym_bridge, noiseless()
    previous = bridge.agent_state(0).speed
    imu.step(previous, 0.0, bridge.config.control_dt)
    accels = []
    for _ in range(40):
        bridge.step_cars([(0.0, 2.0)])
        state = bridge.agent_state(0)
        sample = imu.step(state.speed, state.yaw_rate, bridge.config.control_dt)
        # oracle: straight steering has no rotation; finite difference of
        # independently read true speeds is forward acceleration.
        assert abs(state.yaw_rate) < 1e-12  # rad/s
        assert sample.bno_acc[0] == pytest.approx(
            (state.speed-previous)/bridge.config.control_dt, abs=1e-9)  # m/s²
        assert not bridge.body_contact(0)
        previous = state.speed
        accels.append(sample.bno_acc[0])
    # oracle: the commanded step from rest must actually exercise acceleration.
    assert max(accels) > 0.0
    print(f'ramp peak={max(accels):.9f} m/s²')


def test_stationary_noise_spread():
    imu = SimImu(np.random.default_rng(20261026))
    samples = [imu.step(0.0, 0.0, 0.025) for _ in range(20000)]
    for prefix, noise in [('bno', BNO_NOISE), ('vesc', VESC_NOISE)]:
        for suffix, std in [('gyro', noise.gyro_std), ('acc', noise.accel_std)]:
            values = np.array([getattr(s, f'{prefix}_{suffix}') for s in samples])
            # oracle: injected independent Gaussian noise has configured
            # per-axis standard deviation; fixed seed and 20k samples give
            # a 5% statistical allowance. Zero noise is exactly constant.
            np.testing.assert_allclose(np.std(values, axis=0), std, rtol=0.05, atol=0.0)


def test_zero_noise_is_deterministic():
    a = SimImu(np.random.default_rng(1), ZERO_NOISE, ZERO_NOISE)
    b = SimImu(np.random.default_rng(2), ZERO_NOISE, ZERO_NOISE)
    # oracle: with zero noise the RNG cannot affect any measurement.
    for speed, rate in [(0.0, 0.0), (1.0, -0.4), (2.0, 0.5)]:
        assert a.step(speed, rate, 0.025) == b.step(speed, rate, 0.025)


@pytest.mark.parametrize('fault', FAULTS)
def test_fault_outputs(fault):
    imu = noiseless(fault=fault, bias=0.17, scale=1.3)
    expected = noiseless().step(2.0, 0.5, 0.025)
    actual = imu.step(2.0, 0.5, 0.025)
    # oracle: I1 fault table specifies exact changes, not loose tolerances.
    if fault == 'stale':
        assert actual is None
        health = ImuHealth('bno')
        state, reason = health.status(0.0)
        assert imu.status() == dict(state=state, reason=reason, imu='bno')
    elif fault == 'zero':
        assert actual.bno_acc == actual.bno_gyro == actual.vesc_acc == actual.vesc_gyro == (0.0,)*3
        # oracle: zero bypasses configured noise, not just noiseless physics.
        noisy_zero = SimImu(np.random.default_rng(7), fault='zero').step(2.0, 0.5, 0.025)
        assert noisy_zero == actual
        health = ImuHealth('bno')
        health.sample(0.0, (0,)*3, (0,)*3, (0,)*3, (0,)*3, 0.0, True)
        assert imu.status()['reason'] == health.status(0.0)[1]
    else:
        assert actual.bno_acc == expected.bno_acc
        assert actual.vesc_acc == expected.vesc_acc
        for sensor in ('bno', 'vesc'):
            gyro = getattr(actual, sensor + '_gyro')
            reference = getattr(expected, sensor + '_gyro')
            assert gyro[:2] == reference[:2]
            z = reference[2] + 0.17 if fault == 'bias' else reference[2] * 1.3 if fault == 'scale' else reference[2]
            assert gyro[2] == z
        if fault == 'status_fault':
            assert imu.status() == dict(state='fault', imu='bno',
                reason='simulated fault (imu_fault=status_fault)')


def test_stale_retains_last_published_status_and_recovers():
    imu = noiseless()
    for _ in range(10):
        imu.step(0.0, 0.0, 0.025)
    before = imu.status()
    imu.fault = 'stale'
    # oracle: controller ruling -- a dead node leaves its last ok status;
    # consumers must use age, and clearing the fault resumes samples.
    assert before == dict(state='ok', reason=None, imu='bno')
    for _ in range(10):
        assert imu.step(1.0, 0.5, 0.025) is None
        assert imu.status() == before
    imu.fault = 'none'
    assert imu.step(1.0, 0.5, 0.025).bno_gyro == (0.0, 0.0, 0.5)
    assert imu.status() == before


@pytest.mark.parametrize('fault', FAULTS)
def test_status_consumer_contract(fault):
    imu = noiseless(fault=fault)
    imu.step(0.0, 0.0, 0.025)
    status = json.loads(json.dumps(imu.status()))
    # oracle: imu_health schema and neural_racer's exact status-only parser.
    assert set(status) == {'state', 'reason', 'imu'}
    assert status['state'] in (STATE_OK, STATE_WAITING, STATE_FAULT)
    assert status['imu'] == 'bno'
    assert (status.get('state') == 'ok') == (fault in ('none', 'bias', 'scale'))


def test_unknown_fault_is_rejected():
    # oracle: FAULTS is a closed enumeration, both on construction and update.
    with pytest.raises(ValueError, match='fault'):
        noiseless(fault='banana')
    imu = noiseless()
    with pytest.raises(ValueError, match='fault'):
        imu.fault = 'banana'
    assert imu.fault == 'none'
