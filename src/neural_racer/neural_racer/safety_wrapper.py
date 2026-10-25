"""The deterministic wrapper around the network: every step can only make the
command safer (docs/superpowers/specs/2026-10-23-neural-racer-plan.md, A6).

The deadman and input-freshness checks need clocks, so they live in the node.
This module covers the rest, in order:

    scan sanity -> network output sanity -> clip -> steering rate
    -> LiDAR braking cap -> curvature cap -> map-path cap

The braking geometry is gap_follow's own (gap_logic), imported, never copied,
with gap_follow.yaml's values as defaults. No rclpy import.
"""
import math
from dataclasses import dataclass

import numpy as np

from gap_follow import gap_logic
from neural_racer import observation as obs


@dataclass(frozen=True)
class WrapperConfig:
    max_steering_angle: float = 0.349     # rad, gap_follow.yaml / pure_pursuit.yaml (car 2; car 1: 0.33)
    max_speed: float = 1.0                # m/s, raised only by Phase 10's ladder
    car_width: float = 0.31               # m, gap_follow.yaml
    car_length: float = 0.58
    wheelbase: float = 0.36
    laser_offset_x: float = 0.26
    reserve_distance: float = 0.20        # gap_follow.yaml forward_stop_clearance
    max_braking_decel: float = 3.0        # m/s^2, gap_follow.yaml
    braking_latency_sec: float = 0.25
    max_lateral_accel: float = 2.0        # m/s^2, gap_follow.yaml
    max_steering_rate: float = 2.0        # rad/s, gap_follow.yaml
    command_slew_max_dt: float = 0.10     # s, gap_follow.yaml
    max_invalid_fraction: float = 0.30


@dataclass(frozen=True)
class WrapperResult:
    steering: float
    speed: float
    stopped: bool                 # a zero command for a reason, not a cap
    reason: str | None            # why it stopped
    binding: str                  # what set the speed: 'network', 'braking', ...
    intervened: bool              # anything other than the network shaped the command
    clearance: float              # m, footprint clearance seen this tick (inf if none)


def _stop(reason: str, clearance: float = math.inf) -> WrapperResult:
    return WrapperResult(0.0, 0.0, True, reason, 'stop', True, clearance)


class SafetyWrapper:
    """Caches the per-beam footprint geometry, which only changes with the scan's angles."""

    def __init__(self, cfg: WrapperConfig):
        self.cfg = cfg
        self._angles_key = None
        self._boundary = None

    def boundary(self, angles: np.ndarray) -> np.ndarray:
        key = (angles.size, float(angles[0]), float(angles[-1]))
        if key != self._angles_key:
            c = self.cfg
            self._boundary = gap_logic.vehicle_boundary_distances(
                angles, c.car_width, c.car_length, c.wheelbase, c.laser_offset_x)
            self._angles_key = key
        return self._boundary

    def apply(self, net_steering: float, net_speed: float, ego_path, ranges, angles,
              range_min: float, range_max: float, prev_steering: float, dt: float,
              map_hit_distance: float | None = None) -> WrapperResult:
        c = self.cfg
        ranges = np.asarray(ranges, dtype=np.float64)
        angles = np.asarray(angles, dtype=np.float64)

        # 1. The scan itself.
        if ranges.ndim != 1 or ranges.size != obs.N_BEAMS or angles.shape != ranges.shape:
            return _stop('beam_count')
        valid = obs.beam_validity(ranges, range_min, range_max)
        if 1.0 - valid.mean() > c.max_invalid_fraction:
            return _stop('invalid_beams')

        # 2. The network's output. A broken network is a stop, never a guess.
        path = np.asarray(ego_path, dtype=np.float64)
        if not (math.isfinite(net_steering) and math.isfinite(net_speed)
                and np.all(np.isfinite(path))):
            return _stop('nan_output')
        if not (math.isfinite(prev_steering) and math.isfinite(dt) and dt >= 0.0):
            return _stop('bad_timing')

        # 3. Clip (redundant with the network's own bounds, on purpose).
        steering = min(max(net_steering, -c.max_steering_angle), c.max_steering_angle)
        speed = min(max(net_speed, 0.0), c.max_speed)
        caps = {'network': speed}

        # 4. Steering rate.
        max_step = c.max_steering_rate * min(dt, c.command_slew_max_dt)
        steering = min(max(steering, prev_steering - max_step), prev_steering + max_step)

        # 5. LiDAR braking cap, gap_follow's formula.
        boundary = self.boundary(angles)
        clearance = gap_logic.minimum_footprint_clearance(ranges, valid, boundary)
        caps['braking'] = gap_logic.braking_speed_limit(
            clearance, c.reserve_distance, c.max_braking_decel, c.max_speed,
            c.braking_latency_sec)

        # 6. Curvature cap.
        kappa = abs(math.tan(steering)) / c.wheelbase
        if kappa > 1e-9:
            caps['curvature'] = math.sqrt(c.max_lateral_accel / kappa)

        # 7. Map mode: the forecast path crosses a wall of the frozen map.
        if map_hit_distance is not None:
            caps['map_path'] = gap_logic.braking_speed_limit(
                max(map_hit_distance, 0.0), c.reserve_distance, c.max_braking_decel,
                c.max_speed, c.braking_latency_sec)

        binding = min(caps, key=caps.get)
        final_speed = caps[binding]
        intervened = (binding != 'network' and final_speed < speed - 1e-9) or \
            abs(steering - net_steering) > 1e-9
        return WrapperResult(steering, final_speed, False, None, binding, intervened, clearance)


def scan_angles(angle_min: float, angle_increment: float, n: int) -> np.ndarray:
    return angle_min + angle_increment * np.arange(n, dtype=np.float64)
