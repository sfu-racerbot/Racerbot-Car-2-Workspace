"""IMU unit conversion, calibration and mounting, with no ROS imports.

Everything imu_bridge_node does to a sample lives here, so it can be tested
with a bare `python3 -m pytest src/imu_bridge/test/` and no ROS install.

The calibration model is a port of racer_imu.calibration.ImuCalibration.apply
from the imu-calibration repo (github.com/sfu-racerbot/imu-calibration),
followed by that repo's sensor -> car mount rotation:

    acc_car  = R @ (A_inv @ (acc  - (acc_bias  - dev_acc_offset)))
    gyro_car = R @ (gyro - (gyro_bias - dev_gyro_offset))

acc_bias/gyro_bias are the sensor's TOTAL bias; dev_*_offset is the part the
VESC already subtracts itself (VESC Tool -> App Settings -> IMU), so only the
remainder is removed here and nothing is corrected twice.
test_convert.py::test_matches_racer_imu_on_its_real_calibration_file checks
this port against racer_imu itself.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np
import yaml

# Standard gravity [m/s^2]. The VESC reports acceleration in g; racer_imu
# uses this same constant (racer_imu/types.py).
G = 9.80665

# sensor_msgs/Imu: "If you have no estimate for one of the data elements
# (e.g. your IMU doesn't produce an orientation estimate), please set element
# 0 of the associated covariance matrix to -1".
ORIENTATION_UNKNOWN = (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

IDENTITY_SOURCE = 'identity (no calibration file)'

# Diagonal for the twist components the shadow EKF must never fuse from the
# wheels: large enough that a filter weighting them gives them ~no influence.
UNTRUSTED_VARIANCE = 1e3

PRIMARY_CHOICES = ('auto', 'vesc', 'bno')


def vesc_raw_to_si(acc_g, gyro_dps):
    """VESC units (g, deg/s) -> SI (m/s^2, rad/s)."""
    acc = np.asarray(acc_g, dtype=float) * G
    gyro = np.radians(np.asarray(gyro_dps, dtype=float))
    return acc, gyro


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_R(roll, pitch, yaw):
    """Z-Y-X (yaw, then pitch, then roll) as in ROS/tf: Rz(yaw) Ry(pitch) Rx(roll).

    Radians. Maps a sensor-frame vector into the car frame: v_car = R @ v_sensor.
    """
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def _vector3(value, name, source):
    try:
        v = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{source}: {name} is not numeric: {value!r}') from exc
    if v.shape != (3,):
        raise ValueError(f'{source}: {name} must have 3 elements, got shape {v.shape}')
    if not np.all(np.isfinite(v)):
        raise ValueError(f'{source}: {name} is not finite: {v.tolist()}')
    return v


def _mount_R(rpy_deg, source):
    return rpy_to_R(*np.radians(_vector3(rpy_deg, 'rpy_deg', source)))


@dataclass
class Calibration:
    A_inv: np.ndarray            # accelerometer scale + cross-axis, 3x3
    acc_bias: np.ndarray         # total accelerometer bias [m/s^2]
    gyro_bias: np.ndarray        # total gyro bias [rad/s]
    dev_acc_offset: np.ndarray   # part of acc_bias the device already removes [m/s^2]
    dev_gyro_offset: np.ndarray  # part of gyro_bias the device already removes [rad/s]
    R: np.ndarray                # sensor -> car mount rotation
    source: str                  # where this came from, for the startup log

    def apply(self, acc, gyro):
        """Sensor-frame SI sample -> calibrated car-axis SI (acc m/s^2, gyro rad/s)."""
        acc = np.asarray(acc, dtype=float)
        gyro = np.asarray(gyro, dtype=float)
        acc_cal = self.A_inv @ (acc - (self.acc_bias - self.dev_acc_offset))
        gyro_cal = gyro - (self.gyro_bias - self.dev_gyro_offset)
        return self.R @ acc_cal, self.R @ gyro_cal


def load_calibration(json_path, rpy_deg):
    """Read a racer_imu <name>_calib.json and combine it with the mount rotation.

    A missing file (or no path at all) gives an identity calibration -- the
    mount rotation is still applied, so the axes are right even uncalibrated.
    A file that exists but is malformed raises ValueError: silently falling
    back to identity there would hide a broken calibration.
    """
    R = _mount_R(rpy_deg, 'mount')
    zeros = np.zeros(3)
    if not json_path or not Path(json_path).is_file():
        return Calibration(np.eye(3), zeros, zeros.copy(), zeros.copy(), zeros.copy(),
                           R, IDENTITY_SOURCE)

    source = str(json_path)
    data = json.loads(Path(json_path).read_text())
    if not isinstance(data, dict):
        raise ValueError(f'{source}: expected a JSON object, got {type(data).__name__}')
    for key in ('A_inv', 'acc_bias', 'gyro_bias'):
        if key not in data:
            raise ValueError(f'{source}: missing key {key!r}')

    try:
        A_inv = np.asarray(data['A_inv'], dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{source}: A_inv is not a numeric 3x3 matrix') from exc
    if A_inv.shape != (3, 3):
        raise ValueError(f'{source}: A_inv must be 3x3, got shape {A_inv.shape}')
    if not np.all(np.isfinite(A_inv)):
        raise ValueError(f'{source}: A_inv is not finite')

    dev = data.get('device_offsets') or {}
    return Calibration(
        A_inv=A_inv,
        acc_bias=_vector3(data['acc_bias'], 'acc_bias', source),
        gyro_bias=_vector3(data['gyro_bias'], 'gyro_bias', source),
        dev_acc_offset=_vector3(dev.get('accel_g', [0.0, 0.0, 0.0]),
                                'device_offsets.accel_g', source) * G,
        dev_gyro_offset=np.radians(_vector3(dev.get('gyro_deg_s', [0.0, 0.0, 0.0]),
                                            'device_offsets.gyro_deg_s', source)),
        R=R,
        source=source,
    )


def load_mount(car_yaml, imu):
    """(rpy_deg, absolute calib path or None) for one IMU in racer_imu's car.yaml.

    The calib path in car.yaml is relative to the repo root, the parent of
    config/ -- the same rule as racer_imu.frames.Config.resolve.
    Raises KeyError if the IMU is not listed.
    """
    car_yaml = Path(car_yaml).expanduser().resolve()
    raw = yaml.safe_load(car_yaml.read_text()) or {}
    imus = raw.get('imus') or {}
    if imu not in imus:
        raise KeyError(f'IMU {imu!r} is not listed under imus: in {car_yaml} '
                       f'(listed: {sorted(imus)})')
    entry = imus[imu] or {}
    rpy_deg = [float(a) for a in entry.get('rpy_deg', [0.0, 0.0, 0.0])]
    calib = entry.get('calib')
    if not calib:
        return rpy_deg, None
    calib = Path(calib).expanduser()
    if not calib.is_absolute():
        calib = car_yaml.parent.parent / calib
    return rpy_deg, calib


def _finite_float(value):
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


class StartupBias:
    """Measure the gyro's residual bias while the car sits still at startup.

    Averages the calibrated gyro over `seconds` of continuous stillness:
    |wheel speed| < still_speed and every |gyro axis| < still_gyro. Unknown
    speed (None/NaN) or a non-finite gyro counts as moving. Any moving sample
    throws the window away and restarts it, so a nudge mid-window cannot leak
    into the bias. residual() is zeros until the window completes, then fixed.
    seconds=0 turns it off.
    """

    def __init__(self, seconds=3.0, still_speed=0.03, still_gyro=0.05):
        seconds = float(seconds)
        if not seconds >= 0.0:
            raise ValueError(f'StartupBias seconds must be >= 0, got {seconds}')
        self.seconds = seconds
        self.still_speed = float(still_speed)
        self.still_gyro = float(still_gyro)
        self.done = seconds == 0.0
        self._residual = np.zeros(3)
        self._restart()

    def _restart(self):
        self._t0 = None
        self._sum = np.zeros(3)
        self._n = 0

    def _still(self, gyro, speed):
        speed = _finite_float(speed)
        if speed is None or not abs(speed) < self.still_speed:
            return False
        return bool(np.all(np.isfinite(gyro)) and np.all(np.abs(gyro) < self.still_gyro))

    def update(self, t, gyro, speed):
        """Feed one calibrated gyro sample [rad/s] at time t [s]. Returns done."""
        if self.done:
            return True
        gyro = np.asarray(gyro, dtype=float)
        if not self._still(gyro, speed):
            self._restart()
            return False
        if self._t0 is None:
            self._t0 = t
        self._sum += gyro
        self._n += 1
        if t - self._t0 >= self.seconds:
            self._residual = self._sum / self._n
            self.done = True
        return self.done

    def residual(self):
        return self._residual.copy()


def _positive_sigma(sigma, name):
    sigma = float(sigma)
    if not (sigma > 0.0 and math.isfinite(sigma)):
        raise ValueError(f'{name} must be a finite number > 0, got {sigma}')
    return sigma


def covariance_diag(sigma):
    """3x3 row-major covariance with sigma^2 on the diagonal (sensor_msgs/Imu)."""
    var = _positive_sigma(sigma, 'sigma') ** 2
    return [var if i % 4 == 0 else 0.0 for i in range(9)]


def wheel_twist_covariance(sigma_vx):
    """6x6 row-major twist covariance: sigma_vx^2 for vx, untrusted elsewhere.

    vesc_to_odom_node publishes an all-zero twist covariance, which an EKF
    reads as a perfect measurement. Only vx is a real measurement; its yaw
    rate comes from the commanded steering angle, not a sensor.
    """
    var = _positive_sigma(sigma_vx, 'sigma_vx') ** 2
    cov = [0.0] * 36
    for i in range(6):
        cov[i * 7] = UNTRUSTED_VARIANCE
    cov[0] = var
    return cov


def monotonic_to_ros_ns(t_mono, mono_now, ros_now_ns):
    """Map a host time.monotonic() sample time onto the ROS clock [ns].

    The BNO driver timestamps on the monotonic clock; the shift is taken at
    the moment the sample is published, so the sample keeps its age.
    """
    return int(ros_now_ns) + int(round((t_mono - mono_now) * 1e9))


def resolve_primary(primary, bno_port):
    """Which IMU feeds /imu/data: auto -> bno if a BNO port is set, else vesc."""
    if primary not in PRIMARY_CHOICES:
        raise ValueError(f'primary must be one of {PRIMARY_CHOICES}, got {primary!r}')
    if primary == 'auto':
        return 'bno' if bno_port else 'vesc'
    if primary == 'bno' and not bno_port:
        raise ValueError("primary is 'bno' but bno_port is empty")
    return primary
