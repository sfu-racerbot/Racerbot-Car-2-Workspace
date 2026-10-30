"""sim_fidelity.sensing.ImuSensor and body_acceleration.

Pure numpy: no gym checkout needed, unlike the other tests here. Run
unsourced:

    python3 -m pytest tools/f1tenth_sim/tests/test_sensing_imu.py -v

Oracles: rigid-body closed forms (a steady turn read at a mount offset),
exact tick counting for delay and sample rate, and tools/run_analysis/
spectrum.py run on the sensor's output to recover the bias, noise and
vibration frequency the sensor was given.
"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..'))
from sim_fidelity.sensing import ImuSensor, body_acceleration  # noqa: E402
from run_analysis import spectrum  # noqa: E402

TICK = 0.005      # s, the simulator's integrator step (run_validation.INTEGRATOR_DT)


def _sensor(**kw):
    args = dict(offset_x=0.0, gyro_noise=0.0, accel_noise=0.0, gyro_bias=0.0, delay_ticks=0,
                rng=np.random.default_rng(0))
    args.update(kw)
    return ImuSensor(**args)


def test_zero_noise_and_bias_reproduce_the_plants_yaw_rate_exactly():
    # oracle: invariant -- with nothing added, the gyro IS the plant.
    s = _sensor()
    rates = np.random.default_rng(1).normal(0, 1, 200)
    out = [s.update(w, 0.0, 0.0, 0.0)[0] for w in rates]
    assert out == list(rates)


def test_constant_turn_at_an_offset_gives_the_centripetal_closed_form():
    # oracle: closed form -- v 2 m/s on a 4 m radius: w = 0.5 rad/s, and
    # the reference point accelerates v w = 1.0 m/s^2 towards the centre
    # (+y). A mount 0.3 m ahead and 0.1 m left adds w x (w x r):
    # a_x = -w^2 0.3 = -0.075, a_y = 1.0 - w^2 0.1 = 0.975.
    s = _sensor(offset_x=0.3, offset_y=0.1)
    gz, ax, ay = s.update(0.5, 0.0, 0.0, 1.0)
    assert gz == 0.5
    assert ax == pytest.approx(-0.075, abs=1e-12)      # m/s^2
    assert ay == pytest.approx(0.975, abs=1e-12)       # m/s^2


def test_yaw_acceleration_acts_through_the_offset():
    # oracle: closed form -- alpha x r for alpha 2 rad/s^2 at r = (0.3, 0.1):
    # (-alpha 0.1, +alpha 0.3) = (-0.2, +0.6).
    s = _sensor(offset_x=0.3, offset_y=0.1)
    _, ax, ay = s.update(0.0, 2.0, 0.0, 0.0)
    assert (ax, ay) == (pytest.approx(-0.2, abs=1e-12), pytest.approx(0.6, abs=1e-12))


@pytest.mark.parametrize('ticks', [0, 1, 3])
def test_output_is_delayed_by_exactly_delay_ticks(ticks):
    # oracle: invariant -- an impulse at tick 5 comes out at tick 5 + ticks.
    s = _sensor(delay_ticks=ticks)
    out = [s.update(1.0 if k == 5 else 0.0, 0.0, 0.0, 0.0)[0] for k in range(12)]
    assert out.index(1.0) == 5 + ticks and sum(out) == 1.0


def test_output_rate_samples_and_holds_without_filtering():
    # oracle: invariant -- at 50 Hz on 5 ms ticks the output changes every
    # 4th tick, and holds the value sampled then (no averaging).
    s = _sensor(tick_s=TICK, rate_hz=50.0)
    out = [s.update(float(k), 0.0, 0.0, 0.0)[0] for k in range(12)]
    assert out == [0.0] * 4 + [4.0] * 4 + [8.0] * 4


def test_spectrum_recovers_the_configured_bias_noise_and_vibration():
    # oracle: the sensor's own settings -- 60 s parked-but-running at 50 Hz:
    # gyro bias 0.01 rad/s with 0.005 rad/s noise; an accel x tone of order 1
    # at wheel speed 1 m/s on a 0.05 m wheel = 1 / (2 pi 0.05) = 3.183 Hz.
    rng = np.random.default_rng(2)
    s = ImuSensor(0.0, 0.005, 0.0, 0.01, 0, rng, tick_s=TICK, rate_hz=50.0,
                  vibration=[(1, 0.5, 'x')], wheel_radius=0.05)
    out = np.array([s.update(0.0, 0.0, 0.0, 0.0, wheel_speed=1.0) for _ in range(int(60 / TICK))])
    samples = out[::4]                                    # one per 50 Hz sample
    dc, sigma, _, n = spectrum.dc_and_sigma(samples[:, 0])
    assert dc == pytest.approx(0.01, abs=3 * 0.005 / math.sqrt(n))     # rad/s
    assert sigma == pytest.approx(0.005, rel=0.05)                      # rad/s
    f, P = spectrum.psd(samples[:, 1], 50.0)
    assert spectrum.find_peaks(f, P)[0][0] == pytest.approx(1 / (2 * math.pi * 0.05), abs=f[1])


def test_vibration_frequency_follows_wheel_speed():
    # oracle: closed form -- doubling the wheel speed doubles the tone.
    peaks = []
    for speed in (1.0, 2.0):
        s = ImuSensor(0.0, 0.0, 0.0, 0.0, 0, np.random.default_rng(3), tick_s=TICK,
                      vibration=[(1, 0.5, 'y')], wheel_radius=0.05)
        y = np.array([s.update(0.0, 0.0, 0.0, 0.0, wheel_speed=speed)[2] for _ in range(4000)])
        f, P = spectrum.psd(y, 1 / TICK)
        peaks.append(spectrum.find_peaks(f, P)[0][0])
    assert peaks[1] == pytest.approx(2 * peaks[0], abs=0.2)


def test_bad_settings_and_inputs_are_refused():
    rng = np.random.default_rng(0)
    with pytest.raises(ValueError, match='tick_s'):
        ImuSensor(0.0, 0.0, 0.0, 0.0, 0, rng, rate_hz=50.0)
    with pytest.raises(ValueError, match='wheel_radius'):
        ImuSensor(0.0, 0.0, 0.0, 0.0, 0, rng, tick_s=TICK, vibration=[(1, 0.5, 'x')])
    with pytest.raises(ValueError, match="'x' or 'y'"):
        ImuSensor(0.0, 0.0, 0.0, 0.0, 0, rng, tick_s=TICK, vibration=[(1, 0.5, 'z')],
                  wheel_radius=0.05)
    with pytest.raises(ValueError, match='non-finite'):
        _sensor().update(math.nan, 0.0, 0.0, 0.0)


def test_body_acceleration_closed_forms():
    # oracle: closed form -- steady circle (v, beta constant): only omega x v,
    # (-w v sin b, w v cos b); straight line speeding up 1.0 -> 1.1 in 0.1 s: 1 m/s^2.
    ax, ay = body_acceleration(2.0, 0.1, 2.0, 0.1, 0.5, 0.005)
    assert ax == pytest.approx(-0.5 * 2.0 * math.sin(0.1), abs=1e-12)
    assert ay == pytest.approx(0.5 * 2.0 * math.cos(0.1), abs=1e-12)
    assert body_acceleration(1.0, 0.0, 1.1, 0.0, 0.0, 0.1) == pytest.approx((1.0, 0.0), abs=1e-12)
    with pytest.raises(ValueError, match='dt'):
        body_acceleration(1.0, 0.0, 1.0, 0.0, 0.0, 0.0)
