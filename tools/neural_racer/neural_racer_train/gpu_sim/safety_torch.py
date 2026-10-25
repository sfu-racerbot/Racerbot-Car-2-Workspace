"""Batched safety wrapper -- the training side of neural_racer/safety_wrapper.py.

Training runs the same wrapper as the car, and every tick it overrides the
network costs reward, so the policy learns not to need it (plan A6). The
footprint geometry comes from gap_logic itself; tests/test_safety_parity.py
holds this to the NumPy wrapper.
"""
import math

import numpy as np
import torch

from gap_follow import gap_logic
from neural_racer import observation as obs
from neural_racer.safety_wrapper import WrapperConfig


class SafetyTorch:
    def __init__(self, cfg: WrapperConfig, angles: np.ndarray, device='cpu'):
        self.cfg = cfg
        b = gap_logic.vehicle_boundary_distances(np.asarray(angles, dtype=np.float64), cfg.car_width,
                                                 cfg.car_length, cfg.wheelbase, cfg.laser_offset_x)
        self.boundary = torch.as_tensor(b, dtype=torch.float32, device=device)
        self.boundary_ok = torch.isfinite(self.boundary)

    def braking_limit(self, clearance, max_speed):
        c = self.cfg
        usable = (clearance - c.reserve_distance).clamp(min=0.0)
        at = c.max_braking_decel * c.braking_latency_sec
        v = -at + torch.sqrt(at * at + 2.0 * c.max_braking_decel * usable)
        v = torch.where(torch.isinf(clearance), max_speed, v)
        return torch.minimum(v, max_speed)

    def apply(self, net_steer, net_speed, ranges, range_min, range_max, prev_steer, dt, max_speed):
        """All (E, C) except ranges (E, C, B). Returns (steer, speed, stopped, intervened)."""
        c = self.cfg
        valid = torch.isfinite(ranges) & (ranges > range_min) & (ranges < range_max)
        invalid_frac = 1.0 - valid.float().mean(dim=-1)
        bad = (invalid_frac > c.max_invalid_fraction) | ~torch.isfinite(net_steer) | ~torch.isfinite(net_speed)
        steer = net_steer.clamp(-c.max_steering_angle, c.max_steering_angle)
        speed = torch.minimum(net_speed.clamp(min=0.0), max_speed)
        step = c.max_steering_rate * min(dt, c.command_slew_max_dt)
        steer = torch.minimum(torch.maximum(steer, prev_steer - step), prev_steer + step)
        usable = valid & self.boundary_ok
        gap = torch.where(usable, ranges - self.boundary, torch.full_like(ranges, math.inf))
        clearance = gap.min(dim=-1).values
        v = torch.minimum(speed, self.braking_limit(clearance, max_speed))
        kappa = torch.tan(steer).abs() / c.wheelbase
        curve = torch.sqrt(c.max_lateral_accel / kappa.clamp(min=1e-9))
        v = torch.where(kappa > 1e-9, torch.minimum(v, curve), v)
        intervened = (v < speed - 1e-6) | ((steer - net_steer).abs() > 1e-6) | bad
        steer = torch.where(bad, torch.zeros_like(steer), steer)
        v = torch.where(bad, torch.zeros_like(v), v)
        return steer, v, bad, intervened


def nominal_angles():
    """The car's beam angles: UST-30LX, 1081 beams over 270 degrees."""
    return np.linspace(-math.radians(135.0), math.radians(135.0), obs.N_BEAMS)
