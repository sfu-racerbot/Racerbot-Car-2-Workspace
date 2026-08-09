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
