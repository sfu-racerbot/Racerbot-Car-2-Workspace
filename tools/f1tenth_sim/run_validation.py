#!/usr/bin/env python3
"""Headless, deterministic F1TENTH Gym validation for RacerBot controllers.

This runner uses the official F1TENTH Gym dynamics, LiDAR, map collision
checking, and multi-agent ray casting.  It directly calls the repository's
framework-independent controller math so failures are reproducible without
ROS scheduling, joystick hardware, RViz, or wall-clock timing.

Run tools/f1tenth_sim/setup.sh once before invoking this file.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
from dataclasses import dataclass
from typing import Optional

ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = ROOT / ".sim"
for import_path in (
    SIM_ROOT / "python",
    SIM_ROOT / "f1tenth_gym",
    ROOT / "src" / "gap_follow",
    ROOT / "src" / "pure_pursuit",
):
    sys.path.insert(0, str(import_path))
os.environ.setdefault("NUMBA_CACHE_DIR", str(SIM_ROOT / "numba-cache"))

try:
    import gymnasium as gym
    import numpy as np
    import f1tenth_gym  # noqa: F401 - registers the Gymnasium environment
    from f1tenth_gym.envs.dynamic_models import (
        DynamicModel,
        F1TENTH_VEHICLE_PARAMETERS,
    )
    from f1tenth_gym.envs.env_config import (
        ControlConfig,
        EnvConfig,
        ObservationConfig,
        SimulationConfig,
    )
    from f1tenth_gym.envs.integrators import IntegratorType
    from f1tenth_gym.envs.lidar import LiDARConfig
    from f1tenth_gym.envs.observation import ObservationType
except ImportError as exc:
    raise SystemExit(
        f"F1TENTH Gym is not set up ({exc}). Run tools/f1tenth_sim/setup.sh first."
    ) from exc

from gap_follow import gap_logic
from pure_pursuit import racing_math


CONTROL_DT = 0.025  # 40 Hz, matching pure_pursuit.yaml
INTEGRATOR_DT = 0.005
WHEELBASE = 0.25
CAR_WIDTH = 0.30
STEERING_LIMIT = 0.26
LIDAR_OFFSET_X = 0.275

LIDAR = LiDARConfig(
    enabled=True,
    num_beams=819,
    angle_min=math.radians(-135.0),
    angle_max=math.radians(135.0),
    range_min=0.05,
    range_max=25.0,
    noise_std=0.01,
    base_link_to_lidar_tf=(LIDAR_OFFSET_X, 0.0, 0.0),
)

VEHICLE_PARAMS = F1TENTH_VEHICLE_PARAMETERS.with_updates(
    # Match this workspace's configured 0.25 m wheelbase and 0.30 m chassis.
    lf=0.12,
    lr=0.13,
    width=CAR_WIDTH,
    collision_body_center_x=WHEELBASE / 2.0,
)


def make_env(track: str, num_agents: int, seed: int):
    config = EnvConfig(
        seed=seed,
        map_name=track,
        params=VEHICLE_PARAMS,
        num_agents=num_agents,
        control_config=ControlConfig(steer_delay_steps=1),
        simulation_config=SimulationConfig(
            timestep=CONTROL_DT,
            integrator_timestep=INTEGRATOR_DT,
            integrator=IntegratorType.RK4,
            dynamics_model=DynamicModel.ST,
            max_laps=1,
        ),
        observation_config=ObservationConfig(type=ObservationType.DIRECT),
        lidar_config=LIDAR,
        render_enabled=False,
    )
    return gym.make("f1tenth_gym:f1tenth-v0", config=config, render_mode=None)


def initial_pose(line, index: int) -> np.ndarray:
    return np.array([line.xs[index], line.ys[index], line.yaws[index]], dtype=float)


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


@dataclass
class PathPlan:
    xy: np.ndarray
    seg_len: np.ndarray
    speed: np.ndarray

    @classmethod
    def from_track(cls, track, v_max: float = 4.0, a_lat_max: float = 2.5):
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


class PathFollower:
    def __init__(
        self,
        plan: PathPlan,
        *,
        min_lookahead: float = 0.6,
        max_lookahead: float = 1.5,
        lookahead_gain: float = 0.15,
    ):
        self.plan = plan
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lookahead_gain = lookahead_gain
        self.previous_index: Optional[int] = None

    def command(
        self,
        state: np.ndarray,
        speed: Optional[float] = None,
        target_override: Optional[tuple[float, float]] = None,
    ) -> tuple[float, float, int, int, float]:
        car_x, car_y, _delta, _velocity, yaw = np.asarray(state, dtype=float)[:5]
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
        lookahead = racing_math.adaptive_lookahead(
            speed_command,
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


def gap_command(scan: np.ndarray) -> tuple[float, float, float, bool]:
    clean, valid = gap_logic.sanitize_ranges(scan, max_range=10.0, range_min=0.05)
    lo, hi = cone_indices(math.pi / 2.0)
    window = clean[lo : hi + 1]
    window_valid = valid[lo : hi + 1]
    closest_index, closest_distance = gap_logic.closest_valid(window, window_valid)

    if closest_distance < 0.15:
        return 0.0, 0.0, float(closest_distance), True

    half_width = CAR_WIDTH / 2.0 + 0.10
    window = gap_logic.disparity_extend(
        window, LIDAR.angle_increment, 0.4, half_width
    )
    if closest_index is not None:
        window = gap_logic.safety_bubble(
            window,
            closest_index,
            closest_distance,
            LIDAR.angle_increment,
            half_width,
        )
    gap_start, gap_end = gap_logic.find_best_gap(
        window,
        2.0,
        angle_increment=LIDAR.angle_increment,
        min_gap_width_m=CAR_WIDTH + 0.10,
    )
    if gap_start is None:
        return 0.0, 0.0, float(closest_distance), True

    target_index = lo + (gap_start + gap_end) // 2
    steering = LIDAR.angle_min + target_index * LIDAR.angle_increment
    steering = float(np.clip(steering, -0.4189, 0.4189))
    speed_scale = 1.0 - abs(steering) / 0.4189
    speed = 0.8 + speed_scale * (2.0 - 0.8)
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
    env = make_env(track, 1, seed)
    line = env.unwrapped.track.raceline
    obs, _ = env.reset(options={"poses": initial_pose(line, 0).reshape(1, 3)})
    reference = np.column_stack((line.xs, line.ys))
    previous_index = None
    max_cross_track = 0.0
    min_scan = math.inf
    stop_steps = 0
    started = time.monotonic()
    max_steps = math.ceil(timeout_s / CONTROL_DT)

    try:
        for step in range(max_steps):
            ego = obs["agent_0"]
            steering, speed, nearest_scan, stopped = gap_command(ego["scan"])
            min_scan = min(min_scan, nearest_scan)
            stop_steps += int(stopped)

            state = np.asarray(ego["std_state"], dtype=float)
            nearest, error = racing_math.find_nearest_index(
                reference,
                state[:2],
                prev_index=previous_index,
                search_window=80,
            )
            previous_index = nearest
            max_cross_track = max(max_cross_track, float(error))

            obs, _reward, done, _truncated, info = env.step(
                np.array([[steering, speed]], dtype=np.float32)
            )
            if done:
                break
        else:
            step = max_steps - 1
    finally:
        env.close()

    result = result_base(
        "gap_solo", track, obs, info, step + 1, time.monotonic() - started
    )
    result.update(
        {
            "max_cross_track_m": round(max_cross_track, 4),
            "min_forward_scan_m": round(min_scan, 4),
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
    env = make_env(track, 1, seed)
    line = env.unwrapped.track.raceline
    obs, _ = env.reset(options={"poses": initial_pose(line, 0).reshape(1, 3)})
    plan = PathPlan.from_track(env.unwrapped.track)
    follower = PathFollower(plan)
    max_cross_track = 0.0
    min_scan = math.inf
    avoid_steps = 0
    stop_steps = 0
    started = time.monotonic()
    max_steps = math.ceil(timeout_s / CONTROL_DT)

    try:
        for step in range(max_steps):
            ego = obs["agent_0"]
            steering, speed, _nearest, _target, error = follower.command(
                ego["std_state"]
            )
            max_cross_track = max(max_cross_track, error)
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

            obs, _reward, done, _truncated, info = env.step(
                np.array([[steering, speed]], dtype=np.float32)
            )
            if done:
                break
        else:
            step = max_steps - 1
    finally:
        env.close()

    result = result_base(
        "pure_solo", track, obs, info, step + 1, time.monotonic() - started
    )
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


def static_expected_scan(env, agent_index: int) -> np.ndarray:
    simulator = env.unwrapped.sim
    lidar_pose = simulator._lidar_pose_from_base(  # official Gym internal API
        simulator.state.poses[agent_index]
    )
    return simulator.scan_sims[agent_index].scan(lidar_pose, rng=None)


def run_pure_traffic(track: str, seed: int, timeout_s: float) -> dict:
    env = make_env(track, 2, seed)
    line = env.unwrapped.track.raceline
    opponent_start_index = max(1, int(6.0 / 0.2))
    poses = np.vstack(
        (initial_pose(line, 0), initial_pose(line, opponent_start_index))
    )
    obs, _ = env.reset(options={"poses": poses})

    plan = PathPlan.from_track(env.unwrapped.track)
    ego_follower = PathFollower(plan)
    opponent_follower = PathFollower(plan)
    cumulative = racing_math.compute_cumulative_arc_length(plan.seg_len)
    total_length = float(plan.seg_len.sum())
    tracker = OpponentProgress()

    overtake_active = False
    overtake_side = 1
    overtake_starts = 0
    completed_passes = 0
    detection_steps = 0
    avoid_steps = 0
    stop_steps = 0
    max_cross_track = 0.0
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
            max_cross_track = max(max_cross_track, error)
            current_arc = float(cumulative[nearest])
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
            expected = static_expected_scan(env, 0)
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
                    gap_ahead = racing_math.track_progress_gap(
                        float(cumulative[nearest]),
                        float(predicted),
                        total_length,
                    )
                    if gap_ahead > total_length - 1.0:
                        overtake_active = False
                        completed_passes += 1

            if overtake_active:
                target_xy = racing_math.lateral_offset_point(
                    plan.xy,
                    target,
                    (target + 1) % len(plan.xy),
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
            min_commanded_speed = min(min_commanded_speed, float(speed))
            max_commanded_speed = max(max_commanded_speed, float(speed))

            actions = np.array(
                [[steering, speed], [opponent_steering, 2.0]], dtype=np.float32
            )
            obs, _reward, done, _truncated, info = env.step(actions)
            # The pinned Gym branch does not advance lap_counts reliably in
            # multi-agent mode. Wrapped nearest-raceline progress remains
            # deterministic and is independently checked against collisions.
            if accumulated_progress >= total_length and completed_passes >= 1:
                break
            if done:
                break
        else:
            step = max_steps - 1
    finally:
        env.close()

    result = result_base(
        "pure_traffic", track, obs, info, step + 1, time.monotonic() - started
    )
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
            "final_speed_mps": round(
                float(obs["agent_0"]["std_state"][3]), 3
            ),
            "final_pose": [
                round(float(obs["agent_0"]["std_state"][i]), 3)
                for i in (0, 1, 4)
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
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    tracks = args.tracks[:1] if args.quick else args.tracks
    results: list[dict] = []

    if args.scenario in ("all", "gap"):
        for track in tracks:
            result = run_gap_solo(track, args.seed, args.timeout)
            results.append(result)
            print(json.dumps(result, sort_keys=True), flush=True)

    if args.scenario in ("all", "pure"):
        for track in tracks:
            result = run_pure_solo(track, args.seed, args.timeout)
            results.append(result)
            print(json.dumps(result, sort_keys=True), flush=True)

    if args.scenario in ("all", "traffic"):
        result = run_pure_traffic(tracks[0], args.seed, args.timeout)
        results.append(result)
        print(json.dumps(result, sort_keys=True), flush=True)

    report = {
        "schema_version": 1,
        "gym_commit": "bdaec1420c3b0f103858d289866d0d4e2e597c30",
        "seed": args.seed,
        "control_rate_hz": 1.0 / CONTROL_DT,
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
