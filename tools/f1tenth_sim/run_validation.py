#!/usr/bin/env python3
"""Headless, deterministic F1TENTH Gym validation for RacerBot controllers.

This runner uses the official F1TENTH Gym dynamics, LiDAR, map collision
checking, and multi-agent ray casting.  It directly calls the repository's
framework-independent controller math so failures are reproducible without
ROS scheduling, joystick hardware, RViz, or wall-clock timing.

The vehicle the controllers meet is not stock gym: racerbot_sim layers this
car's measured and derived parameters, a friction circle, a real steering
servo and realistic sensing on top of the pinned upstream checkout, which is
left pristine.  See tools/f1tenth_sim/README.md; --fidelity legacy reproduces
the original harness.

Run tools/f1tenth_sim/setup.sh once before invoking this file.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from dataclasses import dataclass
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))
from racerbot_sim.bootstrap import bootstrap  # noqa: E402

ROOT = bootstrap()

try:
    import gymnasium as gym
    import numpy as np
    import f1tenth_gym  # noqa: F401 - registers the Gymnasium environment
    from f1tenth_gym.envs.dynamic_models import DynamicModel
    from f1tenth_gym.envs.env_config import (
        EnvConfig,
        ObservationConfig,
        SimulationConfig,
    )
    from f1tenth_gym.envs.integrators import IntegratorType
    from f1tenth_gym.envs.observation import ObservationType
except ImportError as exc:
    raise SystemExit(
        f"F1TENTH Gym is not set up ({exc}). Run tools/f1tenth_sim/setup.sh first."
    ) from exc

from gap_follow import gap_logic
from pure_pursuit import racing_math

from racerbot_sim import CALIBRATION, FidelityPlant, PROFILES
from racerbot_sim.calibration import unmeasured as unmeasured_parameters
from racerbot_sim.plant import DEFAULT_PROFILE, env_components


CONTROL_DT = 0.025  # 40 Hz, matching pure_pursuit.yaml
INTEGRATOR_DT = 0.005
WHEELBASE = CALIBRATION.wheelbase
CAR_WIDTH = CALIBRATION.width
CAR_LENGTH = CALIBRATION.length
STEERING_LIMIT = 0.26
LIDAR_OFFSET_X = CALIBRATION.lidar_offset_x
FORWARD_STOP_CLEARANCE = 0.25
FORWARD_STOP_FOV = math.radians(60.0)
TTC_COMMAND_SPEED_TIMEOUT = 0.5
TTC_COMMAND_FALLBACK_MAX_ODOM_SPEED = 0.10
GAP_MAX_LATERAL_ACCEL = 1.0
GAP_MAX_ACCELERATION = 3.0
GAP_MAX_BRAKING_DECEL = 3.0
MAX_STEERING_RATE = 1.0
PURE_MAX_LATERAL_ACCEL = 2.5
PURE_MAX_ACCELERATION = 6.0
PURE_MAX_BRAKING_DECEL = 8.0
# Mirrors pure_pursuit.yaml's overtake_lookahead_distance. Values at or below
# max_lookahead (1.5) make the passing line a sharp turn that the curvature
# speed cap brakes for, and the ego then stalls behind the opponent.
OVERTAKE_LOOKAHEAD = 4.0
# Mirrors pure_pursuit.yaml's overtake_clear_margin. Must exceed the car's
# 0.535 m length or the ego resumes the racing line while its tail is still
# beside the opponent.
OVERTAKE_CLEAR_MARGIN = 1.0

# Which fidelity fixes are in force. Set from --fidelity in main(); the
# module-level default keeps probe scripts that import this file honest.
PROFILE = PROFILES[DEFAULT_PROFILE]
COMPONENTS = env_components(PROFILE, CALIBRATION)
LIDAR = COMPONENTS["lidar_config"]


def select_profile(name: str) -> None:
    """Switch the active fidelity profile, rebuilding what depends on it."""
    global PROFILE, COMPONENTS, LIDAR
    PROFILE = PROFILES[name]
    COMPONENTS = env_components(PROFILE, CALIBRATION)
    LIDAR = COMPONENTS["lidar_config"]


def make_plant(track: str, num_agents: int, seed: int) -> FidelityPlant:
    """Build the environment and wrap it in this car's fidelity layer."""
    config = EnvConfig(
        seed=seed,
        map_name=track,
        params=COMPONENTS["params"],
        num_agents=num_agents,
        control_config=COMPONENTS["control_config"],
        simulation_config=SimulationConfig(
            timestep=CONTROL_DT,
            integrator_timestep=INTEGRATOR_DT,
            integrator=IntegratorType.RK4,
            dynamics_model=DynamicModel.ST,
            max_laps=1,
        ),
        observation_config=ObservationConfig(type=ObservationType.DIRECT),
        lidar_config=COMPONENTS["lidar_config"],
        collision_check=COMPONENTS["collision_check"],
        render_enabled=False,
    )
    env = gym.make("f1tenth_gym:f1tenth-v0", config=config, render_mode=None)
    return FidelityPlant(
        env, PROFILE, num_agents, seed, CONTROL_DT, CALIBRATION
    )


def initial_pose(line, index: int) -> np.ndarray:
    return np.array([line.xs[index], line.ys[index], line.yaws[index]], dtype=float)


def plan_initial_pose(plan, shipped_line) -> np.ndarray:
    """Spawn pose on the path the car will follow, heading along it."""
    if RACELINE_SOURCE == "shipped":
        return initial_pose(shipped_line, 0).reshape(1, 3)
    start, nxt = plan.xy[0], plan.xy[1]
    yaw = math.atan2(nxt[1] - start[1], nxt[0] - start[0])
    return np.array([[start[0], start[1], yaw]], dtype=float)


def cone_indices(half_angle_rad: float) -> tuple[int, int]:
    lo = int((-half_angle_rad - LIDAR.angle_min) / LIDAR.angle_increment)
    hi = int((half_angle_rad - LIDAR.angle_min) / LIDAR.angle_increment)
    return max(0, lo), min(LIDAR.num_beams - 1, hi)


def closest_valid(scan: np.ndarray, half_angle_rad: float) -> float:
    lo, hi = cone_indices(half_angle_rad)
    values = np.asarray(scan[lo : hi + 1], dtype=float)
    values = values[
        np.isfinite(values) & (values > 0.0) & (values >= LIDAR.range_min)
    ]
    return float(values.min()) if values.size else math.inf


# Which line pure_pursuit follows: "shipped" is the raceline that comes with
# the track, "optimized" is one computed here from the track's own centerline.
RACELINE_SOURCE = "shipped"
_OPTIMIZED_CACHE: dict = {}


def track_centerline(track):
    """(xy, width_left, width_right) from the centerline CSV the track ships.

    Gym's Raceline object drops the track widths, so read them back from the
    file: x_m, y_m, w_tr_right_m, w_tr_left_m.
    """
    csv_path = Path(track.filepath).parent / f"{track.spec.name}_centerline.csv"
    table = np.loadtxt(csv_path, delimiter=",", comments="#")
    return (table[:, :2].astype(float),
            table[:, 3].astype(float), table[:, 2].astype(float))


def optimized_raceline(track) -> np.ndarray:
    """Minimum-curvature line from the track's centerline and track widths.

    Uses the same optimizer and the same car dimensions and margin that
    optimize_raceline defaults to, so what is measured here is what the tool
    would actually produce for this track.
    """
    key = getattr(track, "spec", None)
    key = getattr(key, "name", None) or id(track)
    if key in _OPTIMIZED_CACHE:
        return _OPTIMIZED_CACHE[key]

    from pure_pursuit import raceline_optimizer

    xy, width_left, width_right = track_centerline(track)

    result = raceline_optimizer.optimize_minimum_curvature(
        xy, width_left, width_right,
        vehicle_half_width=CAR_WIDTH / 2.0,
        safety_margin=0.15,
        spacing=0.5,
        iterations=6,
    )
    line = raceline_optimizer.resample_closed_path(result["line"], 0.15)
    _OPTIMIZED_CACHE[key] = line
    return line


@dataclass
class PathPlan:
    xy: np.ndarray
    seg_len: np.ndarray
    speed: np.ndarray

    @classmethod
    def from_track(cls, track, v_max: float = 4.0, a_lat_max: float = 2.5):
        if RACELINE_SOURCE == "optimized":
            xy = optimized_raceline(track)
        elif RACELINE_SOURCE == "centerline":
            xy = track_centerline(track)[0]
        else:
            line = track.raceline
            xy = np.column_stack((line.xs, line.ys)).astype(float)
        seg_len = racing_math.compute_segment_lengths(xy, closed=True)
        curvature = racing_math.estimate_path_curvature(xy, closed=True)
        speed = racing_math.compute_velocity_profile(
            seg_len,
            curvature,
            v_max=v_max,
            v_min=0.5,
            a_lat_max=a_lat_max,
            a_accel_max=3.0,
            a_brake_max=8.0,
            closed=True,
            smoothing_passes=8,
            friction_ellipse=True,
        )
        return cls(xy=xy, seg_len=seg_len, speed=speed)


class CommandShaper:
    """Deterministic equivalent of pure_pursuit_node's final command stage."""

    def __init__(self):
        self.previous_steering = 0.0
        self.previous_speed = 0.0

    def command(self, desired_steering: float, desired_speed: float,
                hard_speed_cap: float = 4.0,
                measured_speed: Optional[float] = None) -> tuple[float, float]:
        if desired_speed <= 0.0 or hard_speed_cap <= 0.0:
            self.previous_steering = desired_steering
            self.previous_speed = 0.0
            return desired_steering, 0.0

        steering = racing_math.slew_rate_limit(
            desired_steering,
            self.previous_steering,
            CONTROL_DT,
            MAX_STEERING_RATE,
        )
        desired_curvature = math.tan(desired_steering) / WHEELBASE
        commanded_curvature = math.tan(steering) / WHEELBASE
        online_curvature = max(abs(desired_curvature), abs(commanded_curvature))
        curve_speed = racing_math.curvature_speed_limit(
            online_curvature, PURE_MAX_LATERAL_ACCEL, 4.0)
        speed_target = min(desired_speed, curve_speed, hard_speed_cap)
        # Mirrors the node: ramp from the car's measured speed when it is
        # higher than the last command, so a one-tick ceiling does not force a
        # slow climb back from a speed the car was never actually at.
        ramp_basis = self.previous_speed
        if measured_speed is not None:
            ramp_basis = max(ramp_basis, min(abs(float(measured_speed)), 4.0))
        speed = racing_math.slew_rate_limit(
            speed_target,
            ramp_basis,
            CONTROL_DT,
            PURE_MAX_ACCELERATION,
            PURE_MAX_BRAKING_DECEL,
        )
        speed = min(speed, curve_speed, hard_speed_cap)
        self.previous_steering = steering
        self.previous_speed = max(0.0, speed)
        return steering, self.previous_speed


class PathFollower:
    def __init__(
        self,
        plan: PathPlan,
        *,
        min_lookahead: float = 0.6,
        max_lookahead: float = 1.5,
        lookahead_gain: float = 0.15,
        use_measured_lookahead: bool = True,
    ):
        self.plan = plan
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lookahead_gain = lookahead_gain
        self.use_measured_lookahead = use_measured_lookahead
        self.previous_index: Optional[int] = None

    def command(
        self,
        state: np.ndarray,
        speed: Optional[float] = None,
        target_override: Optional[tuple[float, float]] = None,
    ) -> tuple[float, float, int, int, float]:
        car_x, car_y, _delta, velocity, yaw = np.asarray(state, dtype=float)[:5]
        nearest, error = racing_math.find_nearest_index(
            self.plan.xy,
            (car_x, car_y),
            closed=True,
            prev_index=self.previous_index,
            search_window=40,
        )
        if error > 1.0:
            nearest, error = racing_math.find_nearest_index(
                self.plan.xy, (car_x, car_y), closed=True
            )
        self.previous_index = nearest

        speed_command = (
            float(self.plan.speed[nearest]) if speed is None else float(speed)
        )
        lookahead_basis = (
            abs(float(velocity)) if self.use_measured_lookahead else speed_command)
        lookahead = racing_math.adaptive_lookahead(
            lookahead_basis,
            self.lookahead_gain,
            self.min_lookahead,
            self.max_lookahead,
        )
        target_index = racing_math.find_lookahead_index(
            self.plan.seg_len, nearest, lookahead, closed=True
        )
        target_x, target_y = (
            self.plan.xy[target_index]
            if target_override is None
            else target_override
        )
        x_body, y_body = racing_math.world_to_body(
            float(target_x - car_x), float(target_y - car_y), float(yaw)
        )
        curvature = racing_math.steering_arc_curvature(x_body, y_body)
        steering = racing_math.steering_from_curvature(curvature, WHEELBASE)
        steering = float(np.clip(steering, -STEERING_LIMIT, STEERING_LIMIT))
        return steering, speed_command, nearest, target_index, float(error)


class OpponentProgress:
    def __init__(self, smoothing_alpha: float = 0.3):
        self.alpha = smoothing_alpha
        self.arc_length: Optional[float] = None
        self.progress_rate = 0.0
        self.last_time: Optional[float] = None
        self.previous_index: Optional[int] = None

    def update(self, arc_length: float, now: float, total_length: float) -> None:
        if self.arc_length is not None and self.last_time is not None:
            dt = now - self.last_time
            if dt > 1e-3:
                delta = (
                    (arc_length - self.arc_length + total_length / 2.0)
                    % total_length
                    - total_length / 2.0
                )
                raw_rate = delta / dt
                if abs(raw_rate) < 20.0:
                    self.progress_rate = (
                        self.alpha * raw_rate
                        + (1.0 - self.alpha) * self.progress_rate
                    )
        self.arc_length = arc_length
        self.last_time = now

    def seconds_since_seen(self, now: float) -> float:
        if self.last_time is None:
            return math.inf
        return now - self.last_time

    def predicted_arc_length(self, now: float, total_length: float) -> Optional[float]:
        if self.arc_length is None or self.last_time is None:
            return None
        return (
            self.arc_length + self.progress_rate * (now - self.last_time)
        ) % total_length


# gap_follow.yaml's corridor-centering block, mirrored. ENABLE_CENTERING is
# flipped by --no-centering so the same harness can measure the term's effect.
ENABLE_CENTERING = True
GAP_STEERING_GAIN = 1.0
CENTERING_GAIN = 0.25
CENTERING_MAX_STEERING = 0.08
CENTERING_SIDE_HALF_SPAN = math.radians(60.0) / 2.0
CENTERING_FULL_BEARING = math.radians(4.0)
CENTERING_ZERO_BEARING = math.radians(15.0)
CENTERING_FULL_FORWARD_DEPTH = 2.5
CENTERING_ZERO_FORWARD_DEPTH = 1.5
CENTERING_FULL_SIDE_DISTANCE = 4.0
CENTERING_ZERO_SIDE_DISTANCE = 5.0


def centering_bias_for(clean: np.ndarray, valid: np.ndarray,
                       aim_bearing: float, aim_depth: float) -> float:
    """gap_follow_node._centering_bias against the full scan, not the cone."""
    angles = LIDAR.angle_min + np.arange(
        clean.size, dtype=float) * LIDAR.angle_increment
    left = gap_logic.side_wall_distance(
        clean, valid, angles, math.pi / 2.0, CENTERING_SIDE_HALF_SPAN)
    right = gap_logic.side_wall_distance(
        clean, valid, angles, -math.pi / 2.0, CENTERING_SIDE_HALF_SPAN)
    bias, _ = gap_logic.corridor_centering_bias(
        left, right, aim_bearing, aim_depth,
        CENTERING_GAIN, CENTERING_MAX_STEERING,
        CENTERING_FULL_BEARING, CENTERING_ZERO_BEARING,
        CENTERING_FULL_FORWARD_DEPTH, CENTERING_ZERO_FORWARD_DEPTH,
        CENTERING_FULL_SIDE_DISTANCE, CENTERING_ZERO_SIDE_DISTANCE,
    )
    return bias


def corridor_offset(scan: np.ndarray) -> Optional[float]:
    """How far off the middle of the corridor the car is, in meters.

    Positive means it sits left of centre. Returns None where the car is not
    in a corridor at all -- one side open, or no wall within reach -- because
    "the middle" is not defined there and averaging a made-up number in would
    hide the cases that matter.
    """
    clean, valid = gap_logic.sanitize_ranges(scan, max_range=10.0, range_min=0.05)
    angles = LIDAR.angle_min + np.arange(
        clean.size, dtype=float) * LIDAR.angle_increment
    left = gap_logic.side_wall_distance(
        clean, valid, angles, math.pi / 2.0, CENTERING_SIDE_HALF_SPAN)
    right = gap_logic.side_wall_distance(
        clean, valid, angles, -math.pi / 2.0, CENTERING_SIDE_HALF_SPAN)
    if not (math.isfinite(left) and math.isfinite(right)):
        return None
    if max(left, right) > CENTERING_ZERO_SIDE_DISTANCE:
        return None
    return (right - left) / 2.0


def gap_command(
    scan: np.ndarray,
    current_speed: float,
    last_commanded_speed: float = 0.0,
    last_commanded_steering: float = 0.0,
    command_age_sec: float = math.inf,
) -> tuple[float, float, float, bool]:
    clean, valid = gap_logic.sanitize_ranges(
        scan, max_range=10.0, range_min=0.05)
    lo, hi = cone_indices(math.pi / 2.0)
    window = clean[lo : hi + 1]
    window_valid = valid[lo : hi + 1]
    beam_indices = np.arange(lo, hi + 1, dtype=float)
    beam_angles = LIDAR.angle_min + beam_indices * LIDAR.angle_increment
    boundaries = gap_logic.vehicle_boundary_distances(
        beam_angles,
        CAR_WIDTH,
        CAR_LENGTH,
        WHEELBASE,
        LIDAR_OFFSET_X,
    )
    closest_index, closest_distance = gap_logic.closest_valid(
        window, window_valid)

    clearance = gap_logic.minimum_footprint_clearance(
        window, window_valid, boundaries)
    forward_clearance = gap_logic.minimum_footprint_clearance_in_cone(
        window,
        window_valid,
        beam_angles,
        boundaries,
        FORWARD_STOP_FOV,
    )
    effective_speed = gap_logic.conservative_ttc_speed(
        current_speed,
        last_commanded_speed,
        command_age_sec,
        TTC_COMMAND_SPEED_TIMEOUT,
        TTC_COMMAND_FALLBACK_MAX_ODOM_SPEED,
    )
    min_ttc = gap_logic.minimum_ttc(
        window,
        window_valid,
        beam_angles,
        effective_speed,
        boundaries,
        min_closing_speed=0.05,
    )
    if (
        clearance <= 0.02
        or forward_clearance <= FORWARD_STOP_CLEARANCE
        or min_ttc <= 0.5
    ):
        return 0.0, 0.0, float(closest_distance), True

    half_width = CAR_WIDTH / 2.0 + 0.10
    window = gap_logic.disparity_extend(
        window, LIDAR.angle_increment, 0.4, half_width)
    if closest_index is not None:
        window = gap_logic.safety_bubble(
            window,
            closest_index,
            closest_distance,
            LIDAR.angle_increment,
            half_width,
        )
    gap_start, gap_end, used_fallback = gap_logic.find_gap_with_fallback(
        window,
        preferred_distance=2.0,
        fallback_distance=0.8,
        angle_increment=LIDAR.angle_increment,
        min_gap_width_m=0.10,
    )
    if gap_start is None:
        return 0.0, 0.0, float(closest_distance), True

    target_in_window = gap_logic.aim_within_gap(
        window, gap_start, gap_end, beam_angles)
    target_index = lo + target_in_window
    target_angle = LIDAR.angle_min + target_index * LIDAR.angle_increment
    target_distance = float(window[target_in_window])
    # Bearing steering, mirroring gap_follow_node. This replaced a pure
    # pursuit curvature law whose helpers (target_curvature,
    # steering_from_curvature) no longer exist in gap_logic -- the harness
    # had been left calling them and could not run the gap scenario at all.
    centering_bias = 0.0
    if ENABLE_CENTERING:
        centering_bias = centering_bias_for(
            clean, valid, target_angle, target_distance)
    desired_steering = float(
        np.clip(GAP_STEERING_GAIN * target_angle + centering_bias,
                -STEERING_LIMIT, STEERING_LIMIT))
    steering = gap_logic.slew_rate_limit(
        desired_steering,
        last_commanded_steering,
        CONTROL_DT,
        MAX_STEERING_RATE,
    )
    limited_curvature = math.tan(desired_steering) / WHEELBASE
    curve_speed = gap_logic.curvature_speed_limit(
        limited_curvature, GAP_MAX_LATERAL_ACCEL, 2.0)
    clearance_speed = gap_logic.braking_speed_limit(
        forward_clearance,
        FORWARD_STOP_CLEARANCE,
        GAP_MAX_BRAKING_DECEL,
        2.0,
    )
    desired_speed = min(max(0.8, curve_speed), clearance_speed)
    if used_fallback:
        desired_speed = min(desired_speed, 0.5)
    # Mirrors the node: ramp from the car's measured speed when it exceeds the
    # last command, so a transient ceiling does not brake a car that is still
    # rolling at its old speed.
    ramp_basis = max(last_commanded_speed, min(abs(float(current_speed)), 2.0))
    speed = min(desired_speed, ramp_basis + GAP_MAX_ACCELERATION * CONTROL_DT)
    return steering, float(speed), float(closest_distance), False


def result_base(
    scenario: str,
    track: str,
    obs: dict,
    info: dict,
    steps: int,
    wall_seconds: float,
) -> dict:
    ego = obs["agent_0"]
    return {
        "scenario": scenario,
        "track": track,
        "passed": False,
        "collision": bool(ego["collision"]),
        "laps": float(info["lap_counts"][0]),
        "sim_time_s": round(float(info["sim_time"]), 3),
        "wall_time_s": round(float(wall_seconds), 3),
        "steps": int(steps),
    }


def run_gap_solo(track: str, seed: int, timeout_s: float) -> dict:
    plant = make_plant(track, 1, seed)
    line = plant.env.unwrapped.track.raceline
    obs, _ = plant.reset(initial_pose(line, 0).reshape(1, 3))
    reference = np.column_stack((line.xs, line.ys))
    previous_index = None
    max_cross_track = 0.0
    min_scan = math.inf
    stop_steps = 0
    # Directly measures what corridor centering is for: how far off the middle
    # of the corridor the car sits, averaged over the lap. max_cross_track_m
    # only says how far it is from the reference raceline, which is a
    # different question when the reference is not the middle either.
    corridor_offsets = []
    last_commanded_speed = 0.0
    last_commanded_steering = 0.0
    last_command_step = None
    started = time.monotonic()
    max_steps = math.ceil(timeout_s / CONTROL_DT)

    try:
        for step in range(max_steps):
            ego = obs["agent_0"]
            state = np.asarray(ego["std_state"], dtype=float)
            command_age_sec = (
                math.inf
                if last_command_step is None
                else (step - last_command_step) * CONTROL_DT
            )
            steering, speed, nearest_scan, stopped = gap_command(
                ego["scan"],
                float(state[3]),
                last_commanded_speed,
                last_commanded_steering,
                command_age_sec,
            )
            last_commanded_speed = speed
            last_commanded_steering = steering
            last_command_step = step
            min_scan = min(min_scan, nearest_scan)
            stop_steps += int(stopped)
            offset = corridor_offset(ego["scan"])
            if offset is not None:
                corridor_offsets.append(offset)

            # Measured against ground truth, not the degraded pose: sensor
            # error must not be allowed to corrupt the metric that reports
            # its effect.
            nearest, error = racing_math.find_nearest_index(
                reference,
                plant.truth(0)[:2],
                prev_index=previous_index,
                search_window=80,
            )
            previous_index = nearest
            max_cross_track = max(max_cross_track, float(error))

            obs, _reward, done, _truncated, info = plant.step(
                [(steering, speed)]
            )
            # Upstream ends the episode on its own collision flag; ours has
            # to do the same or the car keeps driving through the barrier.
            if done or obs["agent_0"]["collision"]:
                break
        else:
            step = max_steps - 1
    finally:
        plant.close()

    result = result_base(
        "gap_solo", track, obs, info, step + 1, time.monotonic() - started
    )
    result.update(plant.report())
    result.update(
        {
            "max_cross_track_m": round(max_cross_track, 4),
            "min_forward_scan_m": round(min_scan, 4),
            "mean_corridor_offset_m": (
                round(float(np.mean(np.abs(corridor_offsets))), 4)
                if corridor_offsets else None),
            "corridor_samples": len(corridor_offsets),
            "stop_steps": stop_steps,
        }
    )
    result["passed"] = result["laps"] >= 1.0 and not result["collision"]
    return result


def apply_fallback_safety(
    scan: np.ndarray,
    steering: float,
    speed: float,
    *,
    dynamic_ranges: Optional[np.ndarray] = None,
    dynamic_angles: Optional[np.ndarray] = None,
    overtake_active: bool = False,
) -> tuple[float, float, str]:
    # The emergency tier is unconditional, including during an overtake.
    if closest_valid(scan, math.radians(30.0)) < 0.4:
        return steering, 0.0, "stop"

    # A committed pass has already selected a route around the tracked car.
    # Replacing it with the generic 1 m/s avoidance command makes passing a
    # 2 m/s opponent impossible. Other close hazards still hit the raw-scan
    # emergency tier above.
    if overtake_active:
        return steering, speed, "none"

    if dynamic_ranges is None or dynamic_angles is None:
        trigger_distance = closest_valid(scan, math.radians(30.0))
        trigger = trigger_distance < 0.7
    else:
        in_cone = np.abs(dynamic_angles) <= math.radians(30.0)
        values = dynamic_ranges[in_cone]
        trigger = bool(values.size and float(np.min(values)) < 1.5)

    if not trigger:
        return steering, speed, "none"

    lo, hi = cone_indices(math.radians(30.0))
    window = np.nan_to_num(
        np.asarray(scan[lo : hi + 1], dtype=float),
        nan=0.0,
        posinf=10.0,
        neginf=0.0,
    )
    window = np.clip(window, 0.0, 10.0)
    gap_start, gap_end = racing_math.find_best_gap(window, 1.0)
    if gap_start is None:
        return steering, 0.0, "stop"

    target_index = lo + (gap_start + gap_end) // 2
    steering = LIDAR.angle_min + target_index * LIDAR.angle_increment
    steering = float(np.clip(steering, -STEERING_LIMIT, STEERING_LIMIT))
    return steering, 1.0, "avoid"


def run_pure_solo(track: str, seed: int, timeout_s: float) -> dict:
    plant = make_plant(track, 1, seed)
    line = plant.env.unwrapped.track.raceline
    plan = PathPlan.from_track(plant.env.unwrapped.track)
    # Start on the line the car is actually going to follow. Spawning it on
    # the shipped raceline while it tracks a different one would begin the lap
    # with most of a corridor's worth of cross-track error already on the
    # clock, and the scenario's 0.5m limit would fail the comparison before
    # the car had turned a wheel.
    obs, _ = plant.reset(plan_initial_pose(plan, line))
    follower = PathFollower(plan)
    command_shaper = CommandShaper()
    max_cross_track = 0.0
    truth_index = None
    min_scan = math.inf
    avoid_steps = 0
    stop_steps = 0
    started = time.monotonic()
    max_steps = math.ceil(timeout_s / CONTROL_DT)

    try:
        for step in range(max_steps):
            ego = obs["agent_0"]
            # `error` is the controller's own view, from the estimated pose,
            # so the max_cross_track_error watchdog below trips on the same
            # information the node has. The reported metric uses truth.
            steering, speed, _nearest, _target, error = follower.command(
                ego["std_state"]
            )
            _truth_nearest, truth_error = racing_math.find_nearest_index(
                plan.xy,
                plant.truth(0)[:2],
                closed=True,
                prev_index=truth_index,
                search_window=80,
            )
            truth_index = _truth_nearest
            max_cross_track = max(max_cross_track, float(truth_error))
            min_scan = min(
                min_scan, closest_valid(ego["scan"], math.radians(30.0))
            )
            steering, speed, safety = apply_fallback_safety(
                ego["scan"], steering, speed
            )
            avoid_steps += int(safety == "avoid")
            stop_steps += int(safety == "stop")

            if error > 1.0:
                speed = 0.0
                stop_steps += 1

            hard_speed_cap = 1.0 if safety == "avoid" else 4.0
            steering, speed = command_shaper.command(
                steering, speed, hard_speed_cap,
                measured_speed=ego["std_state"][3])
            obs, _reward, done, _truncated, info = plant.step(
                [(steering, speed)]
            )
            if done or obs["agent_0"]["collision"]:
                break
        else:
            step = max_steps - 1
    finally:
        plant.close()

    result = result_base(
        "pure_solo", track, obs, info, step + 1, time.monotonic() - started
    )
    result.update(plant.report())
    result.update(
        {
            "max_cross_track_m": round(max_cross_track, 4),
            "min_forward_scan_m": round(min_scan, 4),
            "avoid_steps": avoid_steps,
            "stop_steps": stop_steps,
            "profile_min_mps": round(float(plan.speed.min()), 3),
            "profile_max_mps": round(float(plan.speed.max()), 3),
            "estimated_lap_time_s": round(
                racing_math.estimate_lap_time(plan.seg_len, plan.speed), 3
            ),
        }
    )
    result["passed"] = (
        result["laps"] >= 1.0
        and not result["collision"]
        and max_cross_track < 0.5
    )
    return result


def run_pure_traffic(track: str, seed: int, timeout_s: float) -> dict:
    plant = make_plant(track, 2, seed)
    line = plant.env.unwrapped.track.raceline
    opponent_start_index = max(1, int(6.0 / 0.2))
    poses = np.vstack(
        (initial_pose(line, 0), initial_pose(line, opponent_start_index))
    )
    obs, _ = plant.reset(poses)

    plan = PathPlan.from_track(plant.env.unwrapped.track)
    ego_follower = PathFollower(plan)
    ego_command_shaper = CommandShaper()
    # The scripted opponent is not the ROS pure-pursuit node under test;
    # retain its established fixed 2 m/s lookahead so only ego changes here.
    opponent_follower = PathFollower(plan, use_measured_lookahead=False)
    cumulative = racing_math.compute_cumulative_arc_length(plan.seg_len)
    total_length = float(plan.seg_len.sum())
    tracker = OpponentProgress()

    overtake_active = False
    overtake_side = 1
    contact: dict = {}
    overtake_starts = 0
    completed_passes = 0
    detection_steps = 0
    avoid_steps = 0
    stop_steps = 0
    max_cross_track = 0.0
    truth_index = None
    accumulated_progress = 0.0
    previous_arc = None
    min_commanded_speed = math.inf
    max_commanded_speed = 0.0
    started = time.monotonic()
    max_steps = math.ceil(timeout_s / CONTROL_DT)
    downsample = 4

    try:
        for step in range(max_steps):
            now = step * CONTROL_DT
            ego = obs["agent_0"]
            opponent = obs["agent_1"]

            steering, speed, nearest, target, error = ego_follower.command(
                ego["std_state"]
            )
            # Progress and cross-track come from ground truth, so completing
            # a lap means the car really went round rather than its estimate
            # having drifted round.
            truth_nearest, truth_error = racing_math.find_nearest_index(
                plan.xy,
                plant.truth(0)[:2],
                closed=True,
                prev_index=truth_index,
                search_window=80,
            )
            truth_index = truth_nearest
            max_cross_track = max(max_cross_track, float(truth_error))
            current_arc = float(cumulative[truth_nearest])
            if previous_arc is not None:
                delta_arc = (
                    (current_arc - previous_arc + total_length / 2.0)
                    % total_length
                    - total_length / 2.0
                )
                accumulated_progress += delta_arc
            previous_arc = current_arc
            opponent_steering, _opp_speed, _oni, _oti, _oe = (
                opponent_follower.command(opponent["std_state"], speed=2.0)
            )

            scan = np.asarray(ego["scan"], dtype=float)
            expected = plant.expected_scan(0)
            measured_ds = scan[::downsample]
            expected_ds = expected[::downsample]
            angle_increment_ds = LIDAR.angle_increment * downsample
            detection = racing_math.detect_dynamic_cluster(
                measured_ds,
                expected_ds,
                LIDAR.angle_min,
                angle_increment_ds,
                margin=0.4,
                min_width=0.15,
                max_width=0.70,
                max_engagement_range=5.0,
                range_min=LIDAR.range_min,
                cluster_gap_threshold=0.3,
            )
            if detection is not None and abs(detection[3]) > math.radians(30.0):
                detection = None

            full_detection = None
            if detection is not None:
                detection_steps += 1
                full_detection = (
                    detection[0] * downsample,
                    detection[1] * downsample,
                    detection[2],
                    detection[3],
                )
                state = np.asarray(ego["std_state"], dtype=float)
                laser_x = state[0] + LIDAR_OFFSET_X * math.cos(state[4])
                laser_y = state[1] + LIDAR_OFFSET_X * math.sin(state[4])
                opponent_x = laser_x + detection[2] * math.cos(
                    state[4] + detection[3]
                )
                opponent_y = laser_y + detection[2] * math.sin(
                    state[4] + detection[3]
                )
                opponent_index, _ = racing_math.find_nearest_index(
                    plan.xy,
                    (opponent_x, opponent_y),
                    prev_index=tracker.previous_index,
                    search_window=40,
                )
                tracker.previous_index = opponent_index
                tracker.update(
                    float(cumulative[opponent_index]), now, total_length
                )

            if not overtake_active and full_detection is not None:
                gap_ahead = racing_math.track_progress_gap(
                    float(cumulative[nearest]),
                    float(tracker.arc_length),
                    total_length,
                )
                if (
                    gap_ahead <= 3.0
                    and speed - tracker.progress_rate > 0.3
                ):
                    overtake_active = True
                    overtake_side = racing_math.pick_pass_side(
                        scan, full_detection[0], full_detection[1]
                    )
                    overtake_starts += 1
            elif overtake_active:
                if tracker.seconds_since_seen(now) > 3.0:
                    overtake_active = False
                else:
                    predicted = tracker.predicted_arc_length(now, total_length)
                    # Mirrors the node: a pass is finished only once the ego
                    # is genuinely clear_margin past the opponent. Using the
                    # wrapped gap here ended the pass the moment the ego's
                    # nose edged ahead, while the cars were still alongside.
                    lead = racing_math.track_lead_distance(
                        float(cumulative[nearest]),
                        float(predicted),
                        total_length,
                    )
                    if lead >= OVERTAKE_CLEAR_MARGIN:
                        overtake_active = False
                        completed_passes += 1

            if overtake_active:
                overtake_target = racing_math.find_lookahead_index(
                    plan.seg_len, nearest, OVERTAKE_LOOKAHEAD, closed=True)
                target_xy = racing_math.lateral_offset_point(
                    plan.xy,
                    overtake_target,
                    (overtake_target + 1) % len(plan.xy),
                    overtake_side * 0.35,
                )
                steering, speed, nearest, target, error = ego_follower.command(
                    ego["std_state"],
                    speed=speed,
                    target_override=target_xy,
                )

            dynamic_mask = racing_math.dynamic_beam_mask(
                measured_ds, expected_ds, margin=0.4, range_min=LIDAR.range_min
            )
            dynamic_ranges = measured_ds[dynamic_mask]
            dynamic_angles = (
                LIDAR.angle_min
                + np.nonzero(dynamic_mask)[0] * angle_increment_ds
            )
            steering, speed, safety = apply_fallback_safety(
                scan,
                steering,
                speed,
                dynamic_ranges=dynamic_ranges,
                dynamic_angles=dynamic_angles,
                overtake_active=overtake_active,
            )
            avoid_steps += int(safety == "avoid")
            stop_steps += int(safety == "stop")
            hard_speed_cap = 1.0 if safety == "avoid" else 4.0
            steering, speed = ego_command_shaper.command(
                steering, speed, hard_speed_cap,
                measured_speed=ego["std_state"][3])
            min_commanded_speed = min(min_commanded_speed, float(speed))
            max_commanded_speed = max(max_commanded_speed, float(speed))

            obs, _reward, done, _truncated, info = plant.step(
                [(steering, speed), (opponent_steering, 2.0)]
            )
            # The pinned Gym branch does not advance lap_counts reliably in
            # multi-agent mode. Wrapped nearest-raceline progress remains
            # deterministic and is independently checked against collisions.
            if accumulated_progress >= total_length and completed_passes >= 1:
                break
            if (
                done
                or obs["agent_0"]["collision"]
                or obs["agent_1"]["collision"]
            ):
                # Capture how the cars were arranged when they touched.
                # Whether the pass was still committed, and whether contact
                # was nose-to-tail or side-by-side, is the whole difference
                # between "closed too fast" and "cut back too early".
                ego_truth = plant.truth(0)
                opp_truth = plant.truth(1)
                dx = float(opp_truth[0] - ego_truth[0])
                dy = float(opp_truth[1] - ego_truth[1])
                yaw = float(ego_truth[4])
                contact = {
                    "contact_at_s": round(now, 3),
                    "contact_longitudinal_m": round(
                        dx * math.cos(yaw) + dy * math.sin(yaw), 3),
                    "contact_lateral_m": round(
                        -dx * math.sin(yaw) + dy * math.cos(yaw), 3),
                    "contact_speed_mps": round(float(ego_truth[3]), 3),
                    "contact_during_overtake": bool(overtake_active),
                    "contact_safety_tier": safety,
                    # What the ego could see ahead. The emergency tier fires
                    # below 0.4 m, so this says whether it stopped for the
                    # opponent or for a wall it had steered towards.
                    "contact_forward_scan_m": round(
                        closest_valid(scan, math.radians(30.0)), 3),
                }
                break
        else:
            step = max_steps - 1
    finally:
        plant.close()

    result = result_base(
        "pure_traffic", track, obs, info, step + 1, time.monotonic() - started
    )
    result.update(plant.report())
    result.update(contact)
    result.update(
        {
            "opponent_collision": bool(obs["agent_1"]["collision"]),
            "opponent_laps": float(info["lap_counts"][1]),
            "max_cross_track_m": round(max_cross_track, 4),
            "detection_steps": detection_steps,
            "overtake_starts": overtake_starts,
            "completed_passes": completed_passes,
            "overtake_active_at_end": overtake_active,
            "last_pass_side": "left" if overtake_side > 0 else "right",
            "avoid_steps": avoid_steps,
            "stop_steps": stop_steps,
            "accumulated_progress_m": round(accumulated_progress, 3),
            "track_length_m": round(total_length, 3),
            "progress_laps": round(accumulated_progress / total_length, 3),
            "min_commanded_speed_mps": round(min_commanded_speed, 3),
            "max_commanded_speed_mps": round(max_commanded_speed, 3),
            "final_speed_mps": round(float(plant.truth(0)[3]), 3),
            "final_pose": [
                round(float(plant.truth(0)[i]), 3) for i in (0, 1, 4)
            ],
        }
    )
    result["passed"] = (
        accumulated_progress >= total_length
        and not result["collision"]
        and not result["opponent_collision"]
        and completed_passes >= 1
    )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        choices=("all", "gap", "pure", "traffic"),
        default="all",
        help="Scenario family to run (default: %(default)s).",
    )
    parser.add_argument(
        "--tracks",
        nargs="+",
        default=["Spielberg", "Silverstone", "BrandsHatch"],
        help="Official track names for solo tests. Traffic uses the first track.",
    )
    parser.add_argument("--seed", type=int, default=12345)
    parser.add_argument(
        "--repeat-seeds",
        type=int,
        default=1,
        metavar="N",
        help="Re-run every scenario with N consecutive seeds starting at "
             "--seed. The traffic scenario is genuinely seed-fragile -- it "
             "deadlocks nose-to-wall behind the opponent on a meaningful "
             "fraction of seeds even with stock parameters -- so a single "
             "run of it is weak evidence either way (default: %(default)s).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=240.0,
        help="Maximum simulated seconds per scenario.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional JSON report path. Parent directories are created.",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Run only the first track (useful during development).",
    )
    parser.add_argument(
        "--no-centering",
        action="store_true",
        help="Disable gap_follow's corridor-centering term, for measuring what "
             "it changes. Not a supported car configuration.",
    )
    parser.add_argument(
        "--fidelity",
        choices=tuple(PROFILES),
        default=DEFAULT_PROFILE,
        help="How closely the simulated vehicle models this car. 'car' is "
             "everything (calibrated parameters, friction circle, real servo, "
             "degraded sensing); 'plant' is the vehicle fixes with perfect "
             "sensing, for telling a physics change apart from a sensing one; "
             "'legacy' reproduces the pre-audit harness and the results "
             "checked in at docs/f1tenth-sim-results.json "
             "(default: %(default)s).",
    )
    parser.add_argument(
        "--raceline",
        choices=("shipped", "optimized", "centerline"),
        default="shipped",
        help="Path pure_pursuit follows: the raceline shipped with the track, "
             "one computed here by pure_pursuit.raceline_optimizer from the "
             "track's own centerline and widths, or the bare centerline -- the "
             "closest stand-in for a hand-recorded lap, and the thing the "
             "optimizer is actually replacing (default: %(default)s).",
    )
    return parser.parse_args()


def main() -> int:
    global ENABLE_CENTERING, RACELINE_SOURCE
    args = parse_args()
    ENABLE_CENTERING = not args.no_centering
    RACELINE_SOURCE = args.raceline
    select_profile(args.fidelity)
    tracks = args.tracks[:1] if args.quick else args.tracks
    results: list[dict] = []

    for seed in range(args.seed, args.seed + max(1, args.repeat_seeds)):
        batch: list[dict] = []
        if args.scenario in ("all", "gap"):
            batch += [run_gap_solo(t, seed, args.timeout) for t in tracks]
        if args.scenario in ("all", "pure"):
            batch += [run_pure_solo(t, seed, args.timeout) for t in tracks]
        if args.scenario in ("all", "traffic"):
            batch.append(run_pure_traffic(tracks[0], seed, args.timeout))
        for result in batch:
            result["seed"] = seed
            print(json.dumps(result, sort_keys=True), flush=True)
        results += batch

    report = {
        "schema_version": 2,
        "gym_commit": "bdaec1420c3b0f103858d289866d0d4e2e597c30",
        "seed": args.seed,
        "control_rate_hz": 1.0 / CONTROL_DT,
        "fidelity_profile": PROFILE.name,
        "lidar_beams": LIDAR.num_beams,
        "surface_friction": CALIBRATION.surface_friction,
        "grip_limit_mps2": round(CALIBRATION.friction_accel_limit, 3),
        # Parameters still inherited from gym rather than justified for this
        # car. Recorded in the report so a stale result cannot quietly claim
        # more authority than it has.
        "unmeasured_parameters": unmeasured_parameters(),
        "passed": all(item["passed"] for item in results),
        "results": results,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")

    print(
        json.dumps(
            {
                "summary": "PASS" if report["passed"] else "FAIL",
                "scenarios": len(results),
                "passed": sum(int(item["passed"]) for item in results),
            },
            sort_keys=True,
        )
    )
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
