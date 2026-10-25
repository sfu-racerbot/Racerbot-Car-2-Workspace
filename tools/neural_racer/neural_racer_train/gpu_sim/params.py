"""Vehicle and sensor parameters, batched per environment, with randomization
keyed to how well each number is known (plan A7).

Values come from tools/f1tenth_sim/sim_fidelity/calibration.py -- the same
numbers the CPU referee uses -- with car 1's own measurements applied on top
(CAR1_OVERRIDES), because that calibration still describes car 2 in places
(docs/car1-vs-car2.md: "Simulator vehicle model: not car 1's"). Overriding
here, rather than editing calibration.py, keeps this branch from touching
existing code; Phase 1 of the plan moves the overrides into calibration.py.
"""
import math
from dataclasses import dataclass, fields, replace

import torch

G = 9.81                   # gym's and calibration.py's gravity
CONTROL_DT = 0.025         # 40 Hz, run_validation.CONTROL_DT
INTEGRATOR_DT = 0.005      # run_validation.INTEGRATOR_DT, RK4

# Car 1 measurements that differ from calibration.py (car 2's numbers).
CAR1_OVERRIDES = {
    'speed_max': 23250.0 / 3650.0,     # vesc.yaml speed_max / car 1's command gain = 6.37 m/s
    'speed_min': -23250.0 / 3650.0,
    'steering_max_left': 0.401,        # car 1 circles at full stick, 2026-10-21 (hardware-reference.md)
    'steering_max_right': -0.388,
}


@dataclass(frozen=True)
class CarParams:
    # gym single-track parameter vector (calibration.vehicle_parameters)
    mu: float = 0.55
    C_Sf: float = 4.718
    C_Sr: float = 5.4562
    wheelbase: float = 0.36
    cog_front_fraction: float = 0.5
    h: float = 0.074
    m: float = 3.74
    I: float = 0.04712
    steering_max_left: float = CAR1_OVERRIDES['steering_max_left']
    steering_max_right: float = CAR1_OVERRIDES['steering_max_right']
    steering_rate_max: float = 3.2
    v_switch: float = 7.319            # gym stock, not overridden by calibration
    gym_v_min: float = -5.0            # gym stock; sets pid_accl's gain, see calibration.py
    gym_v_max: float = 20.0
    # FidelityPlant's actuation layer
    speed_max: float = CAR1_OVERRIDES['speed_max']
    speed_min: float = CAR1_OVERRIDES['speed_min']
    servo_gain: float = 15.0
    steer_delay_ticks: int = 3
    speed_delay_ticks: int = 2
    speed_loop_accel_gain: float = 5.0
    speed_loop_accel_max: float = 4.5
    speed_loop_brake_gain: float = 4.0
    speed_loop_brake_max: float = 4.0
    longitudinal_grip_coupling: float = 1.0
    # bodies
    body_width: float = 0.30
    body_length: float = 0.535
    envelope_width: float = 0.31
    envelope_length: float = 0.58
    lidar_offset_x: float = 0.26
    # sensors
    lidar_noise_std: float = 0.01
    lidar_dropout_rate: float = 0.002
    lidar_delay_ticks: int = 1
    odom_speed_noise: float = 0.05     # fractional
    imu_offset_x: float = 0.45         # primary IMU, front bumper (team, 2026-10-23); P0.4 measures it
    imu_gyro_noise: float = 0.005      # ESTIMATED until P0.5
    imu_accel_noise: float = 0.15      # ESTIMATED until P0.5 (driving vibration)
    imu_delay_ticks: int = 1           # ESTIMATED until P0.5
    imu_gyro_bias: float = 0.005

    @property
    def lf(self) -> float:
        return self.wheelbase * self.cog_front_fraction

    @property
    def lr(self) -> float:
        return self.wheelbase * (1.0 - self.cog_front_fraction)

    @property
    def a_max(self) -> float:
        return self.mu * G                       # calibration: friction_accel_limit


def from_calibration() -> CarParams:
    """CarParams built from sim_fidelity's CALIBRATION plus car 1's overrides.
    Falls back to the defaults above when the simulator checkout is absent."""
    try:
        from neural_racer_train.fidelity import calibration_module
        cal = calibration_module().CALIBRATION
    except Exception:  # noqa: BLE001 -- no checkout: the defaults mirror it
        return CarParams()
    return CarParams(
        mu=cal.surface_friction, C_Sf=cal.cornering_stiffness_front,
        C_Sr=cal.cornering_stiffness_rear, wheelbase=cal.wheelbase,
        cog_front_fraction=cal.cog_front_fraction, h=cal.cog_height, m=cal.mass,
        I=cal.yaw_inertia, steering_rate_max=cal.steering_rate_max,
        servo_gain=cal.steering_servo_gain, steer_delay_ticks=cal.steering_delay_ticks,
        speed_delay_ticks=cal.speed_delay_ticks,
        speed_loop_accel_gain=cal.speed_loop_accel_gain, speed_loop_accel_max=cal.speed_loop_accel_max,
        speed_loop_brake_gain=cal.speed_loop_brake_gain, speed_loop_brake_max=cal.speed_loop_brake_max,
        longitudinal_grip_coupling=cal.longitudinal_grip_coupling,
        body_width=cal.body_width, body_length=cal.body_length,
        envelope_width=cal.width, envelope_length=cal.length, lidar_offset_x=cal.lidar_offset_x,
        lidar_noise_std=cal.lidar_noise_std, lidar_dropout_rate=cal.lidar_dropout_rate,
        lidar_delay_ticks=cal.lidar_delay_ticks, odom_speed_noise=cal.odom_speed_noise,
    )


# Randomization by provenance (plan A7): (kind, spread). 'rel' = +/- fraction,
# 'abs' = +/- absolute, 'ticks' = integer range [lo, hi], 'range' = [lo, hi].
RANDOMIZATION = {
    'speed_loop_accel_gain': ('rel', 0.05), 'speed_loop_accel_max': ('rel', 0.05),
    'speed_loop_brake_gain': ('rel', 0.05), 'speed_loop_brake_max': ('rel', 0.05),
    'lidar_noise_std': ('rel', 0.10), 'lidar_dropout_rate': ('rel', 0.30),
    'mu': ('range', (0.40, 1.10)),             # STOCK-grade knowledge of the floor
    'C_Sf': ('rel', 0.30), 'C_Sr': ('rel', 0.30), 'h': ('rel', 0.30), 'I': ('rel', 0.30),
    'servo_gain': ('rel', 0.30),
    'steer_delay_ticks': ('ticks', (2, 4)), 'speed_delay_ticks': ('ticks', (1, 3)),
    'lidar_offset_x': ('abs', 0.02), 'imu_offset_x': ('abs', 0.02),
    'imu_gyro_noise': ('rel', 0.30), 'imu_accel_noise': ('rel', 0.30),
    'imu_delay_ticks': ('ticks', (0, 2)),
}
# Steering-gain error of +/-10% (car 1's gain was 30% off until 2026-10-21):
STEER_GAIN_ERROR = 0.10


class BatchParams:
    """Every CarParams field as an (E,) tensor, plus derived fields."""

    def __init__(self, values: dict, device):
        self.device = device
        for k, v in values.items():
            setattr(self, k, v)

    @classmethod
    def constant(cls, p: CarParams, n: int, device='cpu', dtype=torch.float32):
        vals = {f.name: torch.full((n,), float(getattr(p, f.name)), device=device, dtype=dtype)
                for f in fields(CarParams)}
        vals['steer_gain'] = torch.ones(n, device=device, dtype=dtype)
        out = cls(vals, device)
        out._derive()
        return out

    @classmethod
    def randomized(cls, p: CarParams, n: int, gen: torch.Generator, device='cpu'):
        out = cls.constant(p, n, device)
        for name, (kind, spread) in RANDOMIZATION.items():
            base = getattr(out, name)
            u = torch.rand(n, generator=gen, device=gen.device).to(device)
            if kind == 'rel':
                val = base * (1.0 + spread * (2 * u - 1))
            elif kind == 'abs':
                val = base + spread * (2 * u - 1)
            elif kind == 'range':
                val = spread[0] + (spread[1] - spread[0]) * u
            else:
                lo, hi = spread
                val = torch.floor(lo + (hi - lo + 1) * u).clamp(lo, hi)
            setattr(out, name, val)
        out.steer_gain = 1.0 + STEER_GAIN_ERROR * (2 * torch.rand(n, generator=gen, device=gen.device).to(device) - 1)
        out._derive()
        return out

    def _derive(self):
        self.lf = self.wheelbase * self.cog_front_fraction
        self.lr = self.wheelbase * (1.0 - self.cog_front_fraction)
        self.a_max = self.mu * G

    def index(self, idx):
        vals = {k: v[idx] for k, v in vars(self).items() if isinstance(v, torch.Tensor)}
        out = BatchParams(vals, self.device)
        return out

    def assign(self, idx, other: 'BatchParams'):
        for k, v in vars(other).items():
            if isinstance(v, torch.Tensor):
                getattr(self, k)[idx] = v


def with_overrides(p: CarParams, **kw) -> CarParams:
    return replace(p, **kw)
