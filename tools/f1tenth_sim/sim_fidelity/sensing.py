"""Sensor error, because the controllers were being handed ground truth.

The harness used to pass ``ego["std_state"]`` -- the simulator's exact state --
straight into the controllers. On the real car ``pure_pursuit_node`` consumes
``/pf/viz/inferred_pose`` from a particle filter that lags, jitters, and can
stop updating altogether; speed comes from VESC eRPM; the scan is a tick stale
and occasionally drops beams. None of the node's defences against any of that
were reachable in simulation (F7, F9), and map subtraction faced zero error
because prediction and measurement came from the same ray caster (F8).

Everything here is seeded from the scenario seed, so runs stay reproducible --
the property the harness exists for.
"""

from __future__ import annotations

from collections import deque
import math

import numpy as np


class _Delay:
    """Whole-tick FIFO lag on an array-valued signal."""

    def __init__(self, ticks: int):
        self.ticks = max(0, int(ticks))
        self._queue: deque = deque()

    def reset(self) -> None:
        self._queue.clear()

    def push(self, value: np.ndarray) -> np.ndarray:
        if self.ticks == 0:
            return value
        self._queue.append(np.array(value, copy=True))
        while len(self._queue) > self.ticks + 1:
            self._queue.popleft()
        return self._queue[0]


class PoseSensor:
    """Particle-filter stand-in: lag, jitter, and the ability to freeze.

    The freeze is the point of the exercise. ``pose_timeout_sec`` cannot catch
    a pose that keeps *arriving* while no longer *tracking* the car, which is
    what drove this car into a wall on 2026-07-27 and what the
    ``pose_frozen_*`` guards added in commit ``dbdbac3`` exist to catch.
    """

    def __init__(
        self,
        delay_ticks: int,
        noise_xy: float,
        noise_yaw: float,
        rng: np.random.Generator,
    ):
        self.delay = _Delay(delay_ticks)
        self.noise_xy = float(noise_xy)
        self.noise_yaw = float(noise_yaw)
        self.rng = rng
        self.frozen = False
        self._last: np.ndarray | None = None
        self.max_error = 0.0

    def reset(self) -> None:
        self.delay.reset()
        self.frozen = False
        self._last = None
        self.max_error = 0.0

    def update(self, x: float, y: float, yaw: float) -> np.ndarray:
        """Return the estimated (x, y, yaw) for this tick."""
        if self.frozen and self._last is not None:
            return self._last
        noisy = np.array(
            [
                x + self.rng.normal(0.0, self.noise_xy),
                y + self.rng.normal(0.0, self.noise_xy),
                yaw + self.rng.normal(0.0, self.noise_yaw),
            ],
            dtype=float,
        )
        estimate = self.delay.push(noisy)
        self.max_error = max(
            self.max_error, float(math.hypot(estimate[0] - x, estimate[1] - y))
        )
        self._last = estimate
        return estimate


class OdomSensor:
    """VESC eRPM speed estimate: a separate sensor from the pose, so a
    separate error. The 2026-07-27 log had it tracking commanded speed to
    within about 5%."""

    def __init__(self, noise: float, rng: np.random.Generator):
        self.noise = float(noise)
        self.rng = rng

    def update(self, speed: float) -> float:
        if self.noise <= 0.0:
            return float(speed)
        return float(speed + self.rng.normal(0.0, self.noise * max(abs(speed), 0.1)))


def body_acceleration(v0: float, beta0: float, v1: float, beta1: float,
                      yaw_rate: float, dt: float) -> tuple:
    """(a_x, a_y) of the plant's reference point in its own body frame, from
    two consecutive states' speed and slip angle (gym's single-track state:
    v = state[3], beta = state[6]) over dt: the finite difference of the
    body-frame velocity (v cos beta, v sin beta) plus omega x v
    (neural-racer plan P0.7)."""
    if not dt > 0.0:
        raise ValueError(f"dt must be > 0, got {dt}")
    u0, w0 = v0 * math.cos(beta0), v0 * math.sin(beta0)
    u1, w1 = v1 * math.cos(beta1), v1 * math.sin(beta1)
    u, w = 0.5 * (u0 + u1), 0.5 * (w0 + w1)
    return (u1 - u0) / dt - yaw_rate * w, (w1 - w0) / dt + yaw_rate * u


class ImuSensor:
    """A body-mounted IMU: yaw rate and planar specific force at its mount,
    with bias, white noise, a whole-tick delay, its own output rate, and
    optional vibration tones that follow the wheels.

    Per tick the caller passes the plant's yaw rate w, yaw acceleration
    alpha, and the reference point's body-frame acceleration (a_x, a_y)
    (body_acceleration; a point circling at speed v has a_y = v w). At a
    mount (dx, dy) from that point a rigid body adds alpha x r + w x (w x r):

        a_x' = a_x - w^2 dx - alpha dy        a_y' = a_y - w^2 dy + alpha dx

    which is the term imu_bridge's vehicle_state.accel_at_base_link removes
    again on the car. Output: (gyro_z, accel_x, accel_y).

    rate_hz below the tick rate samples every round(1 / (rate_hz tick_s))
    ticks and holds the value in between, with no anti-alias filter -- as
    the VESC driver's 50 Hz poll does. Vibration: (order, amplitude
    [m/s^2], axis 'x' or 'y'), a tone at order * wheel_speed / (2 pi
    wheel_radius) Hz. Every value is the caller's: the measured ones, with
    their provenance, belong in calibration.py.
    """

    def __init__(self, offset_x: float, gyro_noise: float, accel_noise: float,
                 gyro_bias: float, delay_ticks: int, rng: np.random.Generator,
                 offset_y: float = 0.0, accel_bias=(0.0, 0.0), tick_s: float | None = None,
                 rate_hz: float | None = None, vibration=(), wheel_radius: float | None = None):
        if (rate_hz is not None or vibration) and not (tick_s and tick_s > 0.0):
            raise ValueError("rate_hz and vibration need tick_s > 0")
        if vibration and not (wheel_radius and wheel_radius > 0.0):
            raise ValueError("vibration needs wheel_radius > 0")
        for order, amplitude, axis in vibration:
            if axis not in ("x", "y"):
                raise ValueError(f"vibration axis must be 'x' or 'y', got {axis!r}")
        self.offset_x = float(offset_x)
        self.offset_y = float(offset_y)
        self.gyro_noise = float(gyro_noise)
        self.accel_noise = float(accel_noise)
        self.bias = np.array([gyro_bias, *accel_bias], dtype=float)
        self.rng = rng
        self.tick_s = tick_s
        self.every = 1 if rate_hz is None else max(1, int(round(1.0 / (rate_hz * tick_s))))
        self.vibration = tuple(vibration)
        self.wheel_radius = wheel_radius
        self.delay = _Delay(delay_ticks)
        self.reset()

    def reset(self) -> None:
        self.delay.reset()
        self._tick = 0
        self._phase = np.zeros(len(self.vibration))
        self._held = None

    def update(self, yaw_rate: float, yaw_accel: float, accel_x: float, accel_y: float,
               wheel_speed: float = 0.0) -> np.ndarray:
        """This tick's (gyro_z, accel_x, accel_y) as the IMU reports it."""
        values = (yaw_rate, yaw_accel, accel_x, accel_y, wheel_speed)
        if not all(math.isfinite(v) for v in values):
            raise ValueError(f"non-finite plant input {values}")
        w, alpha, dx, dy = yaw_rate, yaw_accel, self.offset_x, self.offset_y
        truth = np.array([w,
                          accel_x - w * w * dx - alpha * dy,
                          accel_y - w * w * dy + alpha * dx])
        for i, (order, amplitude, axis) in enumerate(self.vibration):
            f = order * abs(wheel_speed) / (2.0 * math.pi * self.wheel_radius)
            self._phase[i] += 2.0 * math.pi * f * self.tick_s
            truth[1 if axis == "x" else 2] += amplitude * math.sin(self._phase[i])
        if self._held is None or self._tick % self.every == 0:
            noise = np.array([
                self.rng.normal(0.0, self.gyro_noise) if self.gyro_noise > 0.0 else 0.0,
                self.rng.normal(0.0, self.accel_noise) if self.accel_noise > 0.0 else 0.0,
                self.rng.normal(0.0, self.accel_noise) if self.accel_noise > 0.0 else 0.0])
            self._held = truth + self.bias + noise
        self._tick += 1
        return self.delay.push(self._held)


class ScanSensor:
    """One tick of staleness plus dropped beams.

    Beams that get no echo -- dark, glancing, or beyond range -- come back as
    ``inf`` in a ``sensor_msgs/LaserScan``. The controllers have explicit
    handling for that (``max_range`` clipping, ``sanitize_ranges``) which had
    no way of being exercised while every beam always returned a clean number.
    """

    def __init__(
        self,
        delay_ticks: int,
        dropout_rate: float,
        rng: np.random.Generator,
    ):
        self.delay = _Delay(delay_ticks)
        self.dropout_rate = float(dropout_rate)
        self.rng = rng
        self.dropped_beams = 0

    def reset(self) -> None:
        self.delay.reset()
        self.dropped_beams = 0

    def update(self, scan: np.ndarray) -> np.ndarray:
        values = np.asarray(scan, dtype=float).copy()
        if self.dropout_rate > 0.0:
            dropped = self.rng.random(values.shape) < self.dropout_rate
            count = int(np.count_nonzero(dropped))
            if count:
                values[dropped] = math.inf
                self.dropped_beams += count
        return self.delay.push(values)
