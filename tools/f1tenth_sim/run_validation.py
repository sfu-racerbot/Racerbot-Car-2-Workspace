#!/usr/bin/env python3
"""Headless, deterministic F1TENTH Gym validation for RacerBot controllers.

This runner uses the official F1TENTH Gym dynamics, LiDAR, map collision
checking, and multi-agent ray casting.  It directly calls the repository's
framework-independent controller math so failures are reproducible without
ROS scheduling, joystick hardware, RViz, or wall-clock timing.

The vehicle the controllers meet is not stock gym: sim_fidelity layers this
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
from sim_fidelity.bootstrap import bootstrap  # noqa: E402

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
from gap_mirror import GapController, node_params
from pure_pursuit import racing_math, tracking_laws

from sim_fidelity import CALIBRATION, FidelityPlant, PROFILES
from sim_fidelity.calibration import unmeasured as unmeasured_parameters
from sim_fidelity.plant import DEFAULT_PROFILE, env_components


# The race controller's limits come from the file the car loads, so a change
# there (the 2026-10-16 steering recalibration raised max_steering_angle
# 0.26 -> 0.349 and this file kept 0.26 for three days) is what the harness
# judges, with no second copy to forget.
PURE_PURSUIT_CONFIG = (Path(__file__).resolve().parents[2] / "src" / "pure_pursuit"
                       / "config" / "pure_pursuit.yaml")


def _pure_pursuit_params() -> dict:
    import yaml
    data = yaml.safe_load(PURE_PURSUIT_CONFIG.read_text())
    return data["pure_pursuit_node"]["ros__parameters"]


PURE_PURSUIT_PARAMS = _pure_pursuit_params()
CONTROL_DT = 1.0 / float(PURE_PURSUIT_PARAMS["control_rate_hz"])  # 40 Hz
INTEGRATOR_DT = 0.005
WHEELBASE = CALIBRATION.wheelbase
CAR_WIDTH = CALIBRATION.width
CAR_LENGTH = CALIBRATION.length
STEERING_LIMIT = float(PURE_PURSUIT_PARAMS["max_steering_angle"])
LIDAR_OFFSET_X = CALIBRATION.lidar_offset_x
FORWARD_STOP_CLEARANCE = 0.25
FORWARD_STOP_FOV = math.radians(60.0)
TTC_COMMAND_SPEED_TIMEOUT = 0.5
TTC_COMMAND_FALLBACK_MAX_ODOM_SPEED = 0.10
GAP_MAX_LATERAL_ACCEL = 1.0
GAP_MAX_ACCELERATION = 3.0
GAP_MAX_BRAKING_DECEL = 3.0
MAX_STEERING_RATE = float(PURE_PURSUIT_PARAMS["max_steering_rate"])
PURE_MAX_LATERAL_ACCEL = float(PURE_PURSUIT_PARAMS["max_lateral_accel"])
PURE_MAX_ACCELERATION = float(PURE_PURSUIT_PARAMS["max_acceleration"])
PURE_MAX_BRAKING_DECEL = float(PURE_PURSUIT_PARAMS["max_braking_decel"])
PURE_MAX_SPEED = float(PURE_PURSUIT_PARAMS["max_speed"])
# Mirrors pure_pursuit.yaml's overtake_lookahead_distance. Values at or below
# max_lookahead (1.5) make the passing line a sharp turn that the curvature
# speed cap brakes for, and the ego then stalls behind the opponent.
OVERTAKE_LOOKAHEAD = 4.0
# Mirrors pure_pursuit.yaml's overtake_clear_margin. Must exceed the car's
# 0.535 m length or the ego resumes the racing line while its tail is still
# beside the opponent.
OVERTAKE_CLEAR_MARGIN = 1.0
# Mirrors pure_pursuit.yaml's overtake_min_side_clearance. Room needed on the
# passing side before committing: the 0.35 m offset, half a car, and margin.
OVERTAKE_MIN_SIDE_CLEARANCE = 0.70
# Stand-off the scripted opponent keeps from whatever is in front of it.
OPPONENT_RESERVE_DISTANCE = 0.5

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

    def path_context(self, tracking_curvature_smoothing: int = 2):
        """The PathContext the node would build for this plan (spec 00):
        signed curvature from the smoothed path, unsmoothed xy tracked."""
        smoothed = racing_math.smooth_path(xy=self.xy, half_window=tracking_curvature_smoothing, closed=True)
        curvature = racing_math.estimate_signed_path_curvature(smoothed, closed=True)
        return tracking_laws.PathContext(
            xy=self.xy, seg_len=self.seg_len,
            cumulative=racing_math.compute_cumulative_arc_length(self.seg_len),
            speed_profile=self.speed, curvature=curvature, closed=True)


class CommandShaper:
    """Deterministic equivalent of pure_pursuit_node's final command stage."""

    def __init__(self):
        self.previous_steering = 0.0
        self.previous_speed = 0.0

    def command(self, desired_steering: float, desired_speed: float,
                hard_speed_cap: float = PURE_MAX_SPEED,
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
            online_curvature, PURE_MAX_LATERAL_ACCEL, PURE_MAX_SPEED)
        speed_target = min(desired_speed, curve_speed, hard_speed_cap)
        # Mirrors the node: ramp from the car's measured speed when it is
        # higher than the last command, so a one-tick ceiling does not force a
        # slow climb back from a speed the car was never actually at.
        ramp_basis = self.previous_speed
        if measured_speed is not None:
            ramp_basis = max(ramp_basis, min(abs(float(measured_speed)), PURE_MAX_SPEED))
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
    """Follows a PathPlan with a pluggable steering law (spec 01).

    The default is a pure_pursuit TrackingLaw built from the same lookahead
    parameters the node ships, so `pure_pursuit` here reproduces the previous
    hard-coded PathFollower exactly (the refactor proof in spec 01). A
    TrackingLawError (or any steer exception) commands speed 0 for the tick
    and increments law_fail_steps, mirroring the node's stop.
    """

    VEHICLE_LIMITS = tracking_laws.VehicleLimits(
        wheelbase=WHEELBASE, max_steering_angle=STEERING_LIMIT,
        max_steering_rate=MAX_STEERING_RATE, control_dt=CONTROL_DT)

    def __init__(
        self,
        plan: PathPlan,
        *,
        law=None,
        law_name: str = "pure_pursuit",
        law_params: Optional[dict] = None,
        min_lookahead: float = float(PURE_PURSUIT_PARAMS["min_lookahead"]),
        max_lookahead: float = float(PURE_PURSUIT_PARAMS["max_lookahead"]),
        lookahead_gain: float = float(PURE_PURSUIT_PARAMS["lookahead_speed_gain"]),
        use_measured_lookahead: bool = True,
    ):
        self.plan = plan
        self.path_context = plan.path_context()
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lookahead_gain = lookahead_gain
        self.use_measured_lookahead = use_measured_lookahead
        self.previous_index: Optional[int] = None
        self.law_fail_steps = 0
        self.mpc_miss_steps = 0
        self.mpc_fallback_steps = 0
        self._compute_ms: list[float] = []
        if law is None:
            params = {'min_lookahead': min_lookahead,
                      'max_lookahead': max_lookahead,
                      'lookahead_speed_gain': lookahead_gain}
            params.update(law_params or {})
            law = tracking_laws.make_law(law_name, self.VEHICLE_LIMITS, params)
        self.law = law
        # MPC's single-tick fallback (spec 04), mirrored from the node.
        self._mpc_fallback = None
        if isinstance(self.law, tracking_laws.MpcLaw):
            self._mpc_fallback = tracking_laws.make_law(
                'pure_pursuit', self.VEHICLE_LIMITS,
                {'min_lookahead': min_lookahead, 'max_lookahead': max_lookahead,
                 'lookahead_speed_gain': lookahead_gain})

    @property
    def law_compute_ms(self):
        return list(self._compute_ms)

    def command(
        self,
        state: np.ndarray,
        speed: Optional[float] = None,
        target_override: Optional[tuple[float, float]] = None,
        previous_steering: float = 0.0,
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
        if target_override is not None:
            # Overtaking stays pure-pursuit geometry, as in the node.
            x_body, y_body = racing_math.world_to_body(
                float(target_override[0] - car_x), float(target_override[1] - car_y),
                float(yaw))
            curvature = racing_math.steering_arc_curvature(x_body, y_body)
            steering = racing_math.steering_from_curvature(curvature, WHEELBASE)
            steering = float(np.clip(steering, -STEERING_LIMIT, STEERING_LIMIT))
            return steering, speed_command, nearest, target_index, float(error)

        started = time.perf_counter()
        try:
            result = self.law.steer(tracking_laws.TrackingInput(
                x=float(car_x), y=float(car_y), yaw=float(yaw),
                speed=abs(float(velocity)), nearest_idx=nearest,
                previous_steering=float(previous_steering)),
                self.path_context)
            steering = result.steering
            if isinstance(self.law, tracking_laws.MpcLaw):
                self.law.miss_policy.on_success()
        except tracking_laws.MpcMiss:
            # Mirror spec 04's node policy exactly: fall back to pure pursuit
            # for this tick until the consecutive-miss limit, then latch a
            # stop. The latch clears only at the end of a run (no LB here).
            policy = self.law.miss_policy
            outcome = policy.on_miss()
            self.mpc_miss_steps += 1
            if outcome == 'latched':
                steering, speed_command = 0.0, 0.0
            else:
                self.mpc_fallback_steps += 1
                fb = self._mpc_fallback.steer(tracking_laws.TrackingInput(
                    x=float(car_x), y=float(car_y), yaw=float(yaw),
                    speed=abs(float(velocity)), nearest_idx=nearest,
                    previous_steering=float(previous_steering)),
                    self.path_context)
                steering = fb.steering
        except Exception:
            # Mirror the node: a non-miss law failure commands a stop.
            self.law_fail_steps += 1
            steering, speed_command = 0.0, 0.0
        self._compute_ms.append((time.perf_counter() - started) * 1e3)
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


# --no-centering flips this so the same harness can measure the term's effect.
ENABLE_CENTERING = True
# Only for the corridor-offset metric below (same window the node uses).
CENTERING_SIDE_HALF_SPAN = math.radians(60.0) / 2.0
CENTERING_ZERO_SIDE_DISTANCE = 5.0


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


def gap_controller() -> GapController:
    """gap_follow's pipeline (gap_mirror.GapController) with the shipped
    gap_follow.yaml -- the controller the car runs, not a hand-kept copy.

    This replaced a gap_command() that re-implemented the pipeline with its
    own hardcoded numbers and had fallen far behind the node (max_speed 2.0
    vs 3.5, steering 0.26 vs 0.349, no adaptive width, hysteresis, escape
    creep, swept-path clearance, arc guard or corner setup).
    """
    params = node_params()
    params["enable_centering"] = ENABLE_CENTERING
    return GapController(params, LIDAR)


def nearest_valid_range(scan: np.ndarray) -> float:
    """Closest valid return in the forward 180 degrees (the gap window)."""
    clean, valid = gap_logic.sanitize_ranges(scan, max_range=10.0, range_min=LIDAR.range_min)
    lo, hi = cone_indices(math.pi / 2.0)
    return gap_logic.closest_valid(clean[lo:hi + 1], valid[lo:hi + 1])[1]


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
    controller = gap_controller()
    started = time.monotonic()
    max_steps = math.ceil(timeout_s / CONTROL_DT)

    try:
        for step in range(max_steps):
            ego = obs["agent_0"]
            state = np.asarray(ego["std_state"], dtype=float)
            steering, speed = controller.command(ego["scan"], float(state[3]), step,
                                                 horizontal_accel=0.0, lb_pressed=True,
                                                 yaw_rate=float(state[5]),
                                                 fused_speed=float(plant.truth(0)[3]))
            min_scan = min(min_scan, nearest_valid_range(ego["scan"]))
            stop_steps += int(controller.stopped)
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
    expected_scan: Optional[np.ndarray] = None,
) -> tuple[float, float, str]:
    # The emergency tier is unconditional, including during an overtake.
    if closest_valid(scan, math.radians(30.0)) < 0.4:
        return steering, 0.0, "stop"

    # A committed pass has already selected a route around the tracked car.
    # Replacing it with the generic 1 m/s avoidance command makes passing a
    # 2 m/s opponent impossible. Other close hazards still hit the raw-scan
    # emergency tier above. Steering therefore stays with the pass -- but the
    # speed does not get a free ride: cap it at what still allows stopping
    # short, or the car runs the racing-line speed straight into a wall and
    # only reacts at 0.4 m. Mirrors the node's _reactive_override.
    if overtake_active:
        if expected_scan is None:
            # No map prediction: walls and traffic are indistinguishable, so
            # capping would throttle the ego below the car it is passing.
            return steering, speed, "none"
        # The mapped track edge only -- never the opponent, which is by
        # definition the nearest thing ahead during a pass.
        pass_clearance = closest_valid(expected_scan, math.radians(30.0))
        pass_speed = racing_math.braking_speed_limit(
            pass_clearance, 0.4, PURE_MAX_BRAKING_DECEL, 4.0)
        return steering, min(speed, pass_speed), "none"

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


def run_pure_solo(track: str, seed: int, timeout_s: float,
                  law_name: str = "pure_pursuit",
                  law_params: Optional[dict] = None) -> dict:
    plant = make_plant(track, 1, seed)
    line = plant.env.unwrapped.track.raceline
    plan = PathPlan.from_track(plant.env.unwrapped.track)
    # Start on the line the car is actually going to follow. Spawning it on
    # the shipped raceline while it tracks a different one would begin the lap
    # with most of a corridor's worth of cross-track error already on the
    # clock, and the scenario's 0.5m limit would fail the comparison before
    # the car had turned a wheel.
    obs, _ = plant.reset(plan_initial_pose(plan, line))
    follower = PathFollower(plan, law_name=law_name, law_params=law_params)
    command_shaper = CommandShaper()
    max_cross_track = 0.0
    cross_track_sum = 0.0
    cross_track_sq: list[float] = []
    truth_index = None
    min_scan = math.inf
    avoid_steps = 0
    stop_steps = 0
    lap_time_s = None
    started = time.monotonic()
    max_steps = math.ceil(timeout_s / CONTROL_DT)

    try:
        for step in range(max_steps):
            ego = obs["agent_0"]
            # `error` is the controller's own view, from the estimated pose,
            # so the max_cross_track_error watchdog below trips on the same
            # information the node has. The reported metric uses truth.
            steering, speed, _nearest, _target, error = follower.command(
                ego["std_state"],
                previous_steering=command_shaper.previous_steering,
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
            cross_track_sum += float(truth_error)
            cross_track_sq.append(float(truth_error))
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

            hard_speed_cap = 1.0 if safety == "avoid" else PURE_MAX_SPEED
            steering, speed = command_shaper.command(
                steering, speed, hard_speed_cap,
                measured_speed=ego["std_state"][3])
            obs, _reward, done, _truncated, info = plant.step(
                [(steering, speed)]
            )
            if lap_time_s is None and float(info["lap_counts"][0]) >= 1.0:
                lap_time_s = float(info["sim_time"])
            if done or obs["agent_0"]["collision"]:
                break
        else:
            step = max_steps - 1
    finally:
        plant.close()

    compute = sorted(follower.law_compute_ms)
    n_compute = len(compute)

    def _pct(p):
        return round(compute[min(n_compute - 1, int(p * n_compute))], 4) \
            if n_compute else None

    result = result_base(
        "pure_solo", track, obs, info, step + 1, time.monotonic() - started
    )
    result.update(plant.report())
    result.update(
        {
            "steering_law": law_name,
            "max_cross_track_m": round(max_cross_track, 4),
            "mean_cross_track_m": round(
                cross_track_sum / max(1, len(cross_track_sq)), 4),
            "p95_cross_track_m": round(
                float(np.percentile(cross_track_sq, 95)), 4)
            if cross_track_sq else None,
            "lap_time_s": lap_time_s,
            "law_fail_steps": follower.law_fail_steps,
            "law_compute_ms_p50": _pct(0.50),
            "law_compute_ms_p99": _pct(0.99),
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
    if law_name == "mpc":
        policy = follower.law.miss_policy
        result.update({
            "mpc_miss_steps": follower.mpc_miss_steps,
            "mpc_fallback_steps": follower.mpc_fallback_steps,
            "mpc_latched": policy.latched(),
        })
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
    declined_overtakes = 0
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
                ego["std_state"],
                previous_steering=ego_command_shaper.previous_steering,
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
            # Give the scripted opponent brakes. Without them it is a blind
            # constant-speed car that drives into anything in front of it,
            # including an ego that has legitimately stopped -- which made
            # opponent_collision a statement about the test fixture rather
            # than about the controller under test. It still does not steer
            # or race; it only declines to rear-end a stationary object.
            opponent_speed = min(2.0, gap_logic.braking_speed_limit(
                closest_valid(opponent["scan"], math.radians(30.0)),
                OPPONENT_RESERVE_DISTANCE,
                GAP_MAX_BRAKING_DECEL,
                2.0,
            ))

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
                    side = racing_math.pick_pass_side(
                        scan, full_detection[0], full_detection[1]
                    )
                    # pick_pass_side names the more open side but never asks
                    # whether it is open enough. Mirrors the node.
                    if racing_math.overtake_side_has_room(
                            scan, LIDAR.angle_min, LIDAR.angle_increment,
                            side, OVERTAKE_MIN_SIDE_CLEARANCE,
                            math.radians(30.0), LIDAR.range_min,
                            LIDAR.range_max):
                        overtake_active = True
                        overtake_side = side
                        overtake_starts += 1
                    else:
                        declined_overtakes += 1
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
                    previous_steering=ego_command_shaper.previous_steering,
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
                expected_scan=expected,
            )
            avoid_steps += int(safety == "avoid")
            stop_steps += int(safety == "stop")
            hard_speed_cap = 1.0 if safety == "avoid" else PURE_MAX_SPEED
            steering, speed = ego_command_shaper.command(
                steering, speed, hard_speed_cap,
                measured_speed=ego["std_state"][3])
            min_commanded_speed = min(min_commanded_speed, float(speed))
            max_commanded_speed = max(max_commanded_speed, float(speed))

            obs, _reward, done, _truncated, info = plant.step(
                [(steering, speed), (opponent_steering, opponent_speed)]
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
            "declined_overtakes": declined_overtakes,
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
    parser.add_argument(
        "--steering-law",
        choices=tracking_laws.LAW_NAMES,
        default="pure_pursuit",
        help="Steering law for the pure and traffic scenarios (default: "
             "%(default)s).",
    )
    parser.add_argument(
        "--law-param",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Override a law parameter, repeatable, e.g. "
             "--law-param stanley.gain=1.2. Unknown keys are an error.",
    )
    parser.add_argument(
        "--compare",
        nargs="+",
        metavar="LAW",
        help="Run pure_pursuit plus each named law over the same "
             "tracks x seeds x raceline, and print the comparison table and "
             "promotion verdicts. Implies --scenario pure. Requires "
             "--fidelity car.",
    )
    return parser.parse_args()


def _parse_law_params(overrides):
    """Turn repeated KEY=VALUE strings into a params dict, erroring on keys no
    law uses."""
    known = set()
    for prefix in ('stanley', 'lqr', 'mpc'):
        for name in getattr(tracking_laws, f'{prefix.capitalize()}Params').__dataclass_fields__:
            known.add(f'{prefix}.{name}')
    known |= {'min_lookahead', 'max_lookahead', 'lookahead_speed_gain'}
    params = {}
    for item in overrides:
        if '=' not in item:
            raise SystemExit(f'--law-param expects KEY=VALUE, got {item!r}')
        key, _, value = item.partition('=')
        if key not in known:
            raise SystemExit(f'--law-param: unknown key {key!r}')
        params[key] = float(value)
    return params


def main() -> int:
    global ENABLE_CENTERING, RACELINE_SOURCE
    args = parse_args()
    ENABLE_CENTERING = not args.no_centering
    RACELINE_SOURCE = args.raceline
    select_profile(args.fidelity)
    tracks = args.tracks[:1] if args.quick else args.tracks
    law_params = _parse_law_params(args.law_param)

    if args.compare:
        # The comparison is meaningless under the legacy profile: gym's own
        # collision flag is untrustworthy there (CLAUDE.md), and spec 01
        # forbids it outright.
        if args.fidelity != DEFAULT_PROFILE:
            raise SystemExit(
                "--compare requires --fidelity car: the verdict is only "
                "trustworthy when collision comes from FidelityPlant's own "
                "_wall_contact/_chassis_contact, not gym's flag (see "
                "CLAUDE.md).")
        return _run_comparison(args, tracks, law_params)

    results: list[dict] = []

    for seed in range(args.seed, args.seed + max(1, args.repeat_seeds)):
        batch: list[dict] = []
        if args.scenario in ("all", "gap"):
            batch += [run_gap_solo(t, seed, args.timeout) for t in tracks]
        if args.scenario in ("all", "pure"):
            batch += [run_pure_solo(t, seed, args.timeout,
                                    law_name=args.steering_law,
                                    law_params=law_params) for t in tracks]
        if args.scenario in ("all", "traffic"):
            batch.append(run_pure_traffic(tracks[0], seed, args.timeout))
        for result in batch:
            result["seed"] = seed
            result["raceline"] = RACELINE_SOURCE
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


def _run_comparison(args, tracks, law_params) -> int:
    """Run pure_pursuit plus each --compare law over the same tracks x seeds
    and print the promotion verdicts (spec 01)."""
    laws = ["pure_pursuit"] + list(args.compare)
    seeds = list(range(args.seed, args.seed + max(1, args.repeat_seeds)))
    rows_by_law: dict[str, list[dict]] = {law: [] for law in laws}
    for law in laws:
        for seed in seeds:
            for track in tracks:
                result = run_pure_solo(
                    track, seed, args.timeout,
                    law_name=law, law_params=law_params)
                result["seed"] = seed
                result["raceline"] = RACELINE_SOURCE
                # The verdict is only meaningful on the car fidelity profile.
                assert result["fidelity_profile"] == "car", (
                    f"--compare row has fidelity {result['fidelity_profile']!r}")
                rows_by_law[law].append(result)
                print(json.dumps(result, sort_keys=True), flush=True)

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import law_compare

    baseline = rows_by_law["pure_pursuit"]
    verdicts = {}
    print("\n=== tracking-law comparison "
          f"(tracks={tracks}, seeds={seeds}, raceline={RACELINE_SOURCE}, "
          "fidelity=car) ===")
    print("note: law_compute_ms is HOST wall time, not the Jetson\n")
    header = (f"{'law':<13} {'track':<12} {'mean_ct':>8} {'p95_ct':>8} "
              f"{'max_ct':>8} {'lap_s':>8} {'fails':>6} {'p99_ms':>7}")
    print(header)
    print("-" * len(header))
    for law in laws:
        for r in rows_by_law[law]:
            print(f"{law:<13} {r['track']:<12} "
                  f"{r['mean_cross_track_m']:>8.4f} "
                  f"{(r['p95_cross_track_m'] or 0):>8.4f} "
                  f"{r['max_cross_track_m']:>8.4f} "
                  f"{(r['lap_time_s'] or 0):>8.2f} "
                  f"{r['law_fail_steps']:>6d} "
                  f"{(r['law_compute_ms_p99'] or 0):>7.2f}")
        # Verdict for every compared law, including a pure_pursuit self-test
        # (which must REJECT on condition 2 with identical metrics -- a
        # built-in determinism/strictness check, spec 01).
        if law in args.compare:
            v = law_compare.verdict(baseline, rows_by_law[law])
            verdicts[law] = v.verdict
            print(f"\n{law}: {v.verdict}")
            for reason in v.reasons:
                print(f"    - {reason}")
    print()

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({
            "schema_version": 2,
            "compare": True,
            "seed": args.seed,
            "tracks": tracks,
            "raceline": RACELINE_SOURCE,
            "fidelity_profile": PROFILE.name,
            "laws": laws,
            "verdicts": verdicts,
            "results": [r for law in laws for r in rows_by_law[law]],
        }, indent=2, sort_keys=True) + "\n")
    # The comparison itself is the product; exit non-zero if any candidate was
    # rejected, so a CI/launch gate can read it.
    return 0 if all(v == "PROMOTE" for v in verdicts.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
