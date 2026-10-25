"""Health of the IMU on /imu/data, for consumers that must not act on a bad one.

imu_bridge_node publishes this as JSON on /imu/status (latched):

    {"state": "ok" | "waiting" | "fault", "reason": str | null, "imu": "bno" | "vesc"}

"ok" means every sample on /imu/data can be trusted right now. neural_racer
requires it (require_imu: true) and commands zero otherwise. Other consumers
keep reading /imu/data as before; nothing here changes what that topic carries.

  waiting  startup: the gyro bias is not removed yet, or the car has not been
           still long enough for the gravity check (keep it still ~3 s).
  fault    one of:
           - all six fields exactly zero: the IMU is off in VESC Tool, or not
             fitted. A working IMU at rest reads gravity, so real data never
             trips this (measured on car 2, 2026-08-19).
           - a non-finite field.
           - the gravity check failed: over a still window the mean specific
             force must be 9.80665 +/- gravity_tol m/s^2 and within
             max_tilt_deg of +z (car axes, z up). A wrong mount rotation or
             wrong units fails it. Latched until the node restarts: nothing
             later can fix a wrong mount.
           - no sample for longer than stale_timeout_s.

Ported from racerbot_imu's imu_conditioner_node (removed 2026-10-24), which
wrapped the same checks around a second conversion of the same raw data.
No rclpy here; tested in test/test_imu_health.py.
"""

from __future__ import annotations

import json
import math

import numpy as np

from imu_bridge.convert import G, is_still

STATE_OK = 'ok'
STATE_WAITING = 'waiting'
STATE_FAULT = 'fault'


class ImuHealth:
    def __init__(self, imu, stale_timeout_s=0.1, gravity_window_s=1.0,
                 gravity_tol=0.5, max_tilt_deg=10.0, still_speed=0.03, still_gyro=0.05):
        for name, value in (('stale_timeout_s', stale_timeout_s),
                            ('gravity_window_s', gravity_window_s),
                            ('gravity_tol', gravity_tol), ('max_tilt_deg', max_tilt_deg)):
            if not (math.isfinite(float(value)) and float(value) > 0.0):
                raise ValueError(f'{name} must be a finite number > 0, got {value}')
        self.imu = imu
        self.stale_timeout_s = float(stale_timeout_s)
        self.gravity_window_s = float(gravity_window_s)
        self.gravity_tol = float(gravity_tol)
        self.max_tilt_deg = float(max_tilt_deg)
        self.still_speed = float(still_speed)
        self.still_gyro = float(still_gyro)
        self.last_t = None
        self.gravity_ok = False
        self.gravity_fault = None          # latched reason
        self.sample_fault = None           # reason for the latest sample, if bad
        self.bias_done = False
        self._restart_window()

    def _restart_window(self):
        self._t0 = None
        self._acc_sum = np.zeros(3)
        self._n = 0

    def sample(self, t, raw_acc, raw_gyro, acc, gyro, speed, bias_done):
        """One sample of the primary IMU at monotonic time t [s].

        raw_*: as the device sent them (only checked for all-zero/non-finite).
        acc, gyro: calibrated, car axes, SI. speed: wheel speed or None.
        bias_done: the node's StartupBias has finished.
        """
        self.last_t = float(t)
        self.bias_done = bool(bias_done)
        raw = np.concatenate([np.asarray(raw_acc, float).ravel(), np.asarray(raw_gyro, float).ravel()])
        cal = np.concatenate([np.asarray(acc, float).ravel(), np.asarray(gyro, float).ravel()])
        if raw.shape != (6,) or cal.shape != (6,):
            self.sample_fault = 'IMU sample does not have 3 accel and 3 gyro axes'
            return
        if not (np.all(np.isfinite(raw)) and np.all(np.isfinite(cal))):
            self.sample_fault = 'non-finite IMU field'
            return
        if not np.any(raw != 0.0):
            self.sample_fault = 'all zero: IMU disabled in firmware (VESC Tool -> App Settings -> IMU), or not fitted'
            return
        self.sample_fault = None
        if self.gravity_ok or self.gravity_fault:
            return
        if not is_still(cal[3:], speed, self.still_speed, self.still_gyro):
            self._restart_window()
            return
        if self._t0 is None:
            self._t0 = self.last_t
        self._acc_sum += cal[:3]
        self._n += 1
        if self.last_t - self._t0 >= self.gravity_window_s:
            self.gravity_fault = self._gravity_error(self._acc_sum / self._n)
            self.gravity_ok = self.gravity_fault is None

    def _gravity_error(self, mean):
        norm = float(np.linalg.norm(mean))
        if not abs(norm - G) <= self.gravity_tol + 1e-9:     # exactly at tol passes
            return (f'gravity check: |a| = {norm:.3f} m/s^2 at rest, expected '
                    f'{G:.3f} +/- {self.gravity_tol} (wrong units or calibration)')
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, mean[2] / norm))))
        if tilt > self.max_tilt_deg + 1e-9:
            return (f'gravity check: up is {tilt:.1f} deg from +z, limit '
                    f'{self.max_tilt_deg} (wrong mount rotation in car.yaml)')
        return None

    def status(self, now):
        """(state, reason) at monotonic time now [s]."""
        if self.last_t is None:
            return STATE_FAULT, 'no IMU samples yet'
        age = float(now) - self.last_t
        if not age <= self.stale_timeout_s + 1e-9:          # exactly at the limit is fresh
            return STATE_FAULT, f'input stale (no sample for {age:.2f} s, limit {self.stale_timeout_s:.2f} s)'
        if self.gravity_fault:
            return STATE_FAULT, self.gravity_fault
        if self.sample_fault:
            return STATE_FAULT, self.sample_fault
        if not self.bias_done:
            return STATE_WAITING, 'startup gyro bias not removed yet (keep the car still)'
        if not self.gravity_ok:
            return STATE_WAITING, 'gravity check not run yet (keep the car still)'
        return STATE_OK, None

    def status_json(self, now):
        state, reason = self.status(now)
        return json.dumps({'state': state, 'reason': reason, 'imu': self.imu})
