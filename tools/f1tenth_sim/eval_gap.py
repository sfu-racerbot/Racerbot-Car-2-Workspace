#!/usr/bin/env python3
"""Fast, parameterized headless evaluation of the gap_follow pipeline.

Unlike tools/f1tenth_sim/run_validation.py's gap_solo -- whose gap_command is a
hand-maintained *mirror* of the node that has drifted out of date (it still
hardcodes max_speed=2.0, safety_margin=0.10, corner_speed=0.5, steering 0.26,
and has no adaptive width, no gap hysteresis and no escape creep) -- this
harness drives the *same* gap_logic functions the node calls, with every knob
the YAML exposes overridable from the command line. It exists so an iteration
loop can ask "did this change help, and on which tracks" in seconds, per track,
without ROS.

It is still a mirror: it composes gap_logic the way gap_follow_node._scan_step
does, in the same order, and it must be kept in step with that method. The
payoff over run_validation.py is that the parameters are not baked in, so a
tuning sweep does not require editing the harness.

Tracks: the three F1TENTH Gym circuits plus the racerbot_sim indoor layouts
(indoor_oval, indoor_tight, indoor_wide, asb_10000), which are the size class
this car is actually driven in and where the reported failures (too slow, too
tight, gets stuck near walls) live.

    python3 tools/f1tenth_sim/eval_gap.py --track asb_10000
    python3 tools/f1tenth_sim/eval_gap.py --track all --set corner_speed=1.5
    python3 tools/f1tenth_sim/eval_gap.py --track all --baseline

Exit code is 0 when every selected track passes (one lap, no wall contact).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_fidelity.bootstrap import bootstrap  # noqa: E402

ROOT = bootstrap()
sys.path.insert(0, str(ROOT / "src" / "racerbot_sim"))

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import f1tenth_gym  # noqa: F401,E402 - registers the environment
from f1tenth_gym.envs.dynamic_models import DynamicModel  # noqa: E402
from f1tenth_gym.envs.env_config import (  # noqa: E402
    EnvConfig,
    ObservationConfig,
    SimulationConfig,
)
from f1tenth_gym.envs.integrators import IntegratorType  # noqa: E402
from f1tenth_gym.envs.observation import ObservationType  # noqa: E402

from gap_follow import gap_logic  # noqa: E402
from racerbot_sim import tracks as sim_tracks  # noqa: E402
from sim_fidelity import CALIBRATION, PROFILES  # noqa: E402
from sim_fidelity.plant import DEFAULT_PROFILE, FidelityPlant, env_components  # noqa: E402

CONTROL_DT = 0.025
INTEGRATOR_DT = 0.005

GYM_TRACKS = ("Spielberg", "Silverstone", "BrandsHatch")
INDOOR_TRACKS = ("indoor_oval", "indoor_tight", "indoor_wide", "asb_10000")

# Default parameter set: mirrors config/gap_follow.yaml's shipped values so
# the harness measures the algorithm the car actually runs. Every entry is
# overridable with --set name=value.
DEFAULTS = dict(
    max_range=10.0,
    forward_fov_deg=180.0,
    min_gap_distance=2.0,
    fallback_min_gap_distance=0.8,
    gap_switch_margin=1.2,
    corner_speed=1.5,
    corner_speed_wide=1.9,
    max_speed=3.5,
    min_speed=1.0,
    max_steering_angle=0.349,
    steering_gain=1.0,
    max_lateral_accel=1.0,
    max_acceleration=3.0,
    max_braking_decel=3.0,
    max_steering_rate=1.0,
    command_slew_max_dt=0.10,
    car_width=CALIBRATION.width,
    car_length=CALIBRATION.length,
    wheelbase=CALIBRATION.wheelbase,
    laser_offset_x=CALIBRATION.lidar_offset_x,
    laser_offset_y=0.0,
    safety_margin=0.18,
    disparity_threshold=0.4,
    min_centerline_gap_width=0.10,
    emergency_stop_clearance=0.0,
    forward_stop_clearance=0.20,
    forward_stop_fov_deg=60.0,
    escape_creep_speed=0.40,
    # aim_mode: 'midpoint' (aim_within_gap, the node's current law) or
    # 'physical_center' (aim_physical_center, clearance-maximising).
    aim_mode='midpoint',
    enable_adaptive_width=True,
    adaptive_width_narrow=1.4,
    adaptive_width_reference=2.6,
    min_safety_margin=0.18,
    enable_centering=True,
    centering_gain=0.25,
    centering_max_steering=0.08,
    centering_side_fov_deg=60.0,
    centering_full_bearing_deg=4.0,
    centering_zero_bearing_deg=15.0,
    centering_full_forward_depth=2.5,
    centering_zero_forward_depth=1.5,
    centering_full_side_distance=4.0,
    centering_zero_side_distance=5.0,
    enable_ttc=True,
    ttc_threshold_sec=0.30,
    ttc_min_closing_speed=0.05,
    ttc_min_brake_speed=0.6,
    ttc_command_speed_timeout_sec=0.5,
    ttc_command_fallback_max_odom_speed=0.10,
)


def make_plant(track_spec: str, seed: int) -> FidelityPlant:
    """Build a fidelity plant for a gym circuit name or an indoor layout."""
    components = env_components(PROFILES[DEFAULT_PROFILE], CALIBRATION)
    if track_spec in INDOOR_TRACKS:
        directory = Path(ROOT) / ".sim" / "eval_gap_tracks"
        map_name = str(sim_tracks.build(track_spec, directory).with_suffix(""))
    else:
        map_name = track_spec
    config = EnvConfig(
        seed=seed,
        map_name=map_name,
        params=components["params"],
        num_agents=1,
        control_config=components["control_config"],
        simulation_config=SimulationConfig(
            timestep=CONTROL_DT,
            integrator_timestep=INTEGRATOR_DT,
            integrator=IntegratorType.RK4,
            dynamics_model=DynamicModel.ST,
            max_laps=1,
        ),
        observation_config=ObservationConfig(type=ObservationType.DIRECT),
        lidar_config=components["lidar_config"],
        collision_check=components["collision_check"],
        render_enabled=False,
    )
    env = gym.make("f1tenth_gym:f1tenth-v0", config=config, render_mode=None)
    return FidelityPlant(env, PROFILES[DEFAULT_PROFILE], 1, seed, CONTROL_DT, CALIBRATION)


class GapController:
    """Composable replica of gap_follow_node._scan_step's decision pipeline.

    Holds parameters as attributes and reproduces the node's per-scan chain:
    sanitize -> side walls/adaptive width -> footprint/TTC -> disparity extend
    -> safety bubble -> gap pick -> bearing steering -> speed ceilings -> slew.
    State that the node keeps between ticks (previous aim for hysteresis,
    steering basis, last command) lives here too.
    """

    def __init__(self, params: dict, lidar):
        self.p = dict(params)
        self.lidar = lidar
        self.previous_target_idx = None
        self.steering_basis = 0.0
        self.last_commanded_speed = 0.0
        self.last_command_time = None  # in sim steps
        self.stopped = False
        self.stop_reason = None
        # Which speed ceiling won this tick, for the harness's binding-cap
        # histogram: 'curve', 'clearance', 'corner', 'creep', 'accel', 'stop'.
        self.binding_cap = None
        self.cap_counts = None

    def command(self, scan, current_speed, step):
        p = self.p
        clean, valid = gap_logic.sanitize_ranges(
            scan, max_range=p["max_range"], range_min=self.lidar.range_min)

        angles_all = self.lidar.angle_min + np.arange(
            clean.size, dtype=float) * self.lidar.angle_increment

        if p["enable_centering"] or p["enable_adaptive_width"]:
            half_span = math.radians(p["centering_side_fov_deg"]) / 2.0
            side_left = gap_logic.side_wall_distance(
                clean, valid, angles_all, math.pi / 2.0, half_span)
            side_right = gap_logic.side_wall_distance(
                clean, valid, angles_all, -math.pi / 2.0, half_span)
        else:
            side_left = side_right = math.inf

        width_factor, effective_margin, effective_corner = self._adaptive_width(
            side_left, side_right)

        half_fov = math.radians(p["forward_fov_deg"]) / 2.0
        lo = int((-half_fov - self.lidar.angle_min) / self.lidar.angle_increment)
        hi = int((half_fov - self.lidar.angle_min) / self.lidar.angle_increment)
        lo = max(0, lo)
        hi = min(clean.size - 1, hi)
        window = clean[lo:hi + 1]
        window_valid = valid[lo:hi + 1]
        beam_angles = angles_all[lo:hi + 1]

        boundaries = gap_logic.vehicle_boundary_distances(
            beam_angles, p["car_width"], p["car_length"], p["wheelbase"],
            p["laser_offset_x"], p["laser_offset_y"])

        min_clearance = gap_logic.minimum_footprint_clearance(
            window, window_valid, boundaries)
        forward_clearance = gap_logic.minimum_footprint_clearance_in_cone(
            window, window_valid, beam_angles, boundaries,
            math.radians(p["forward_stop_fov_deg"]))

        directional = math.isclose(
            forward_clearance, min_clearance, rel_tol=0.0, abs_tol=1e-6)

        emergency_tripped = False
        if min_clearance <= p["emergency_stop_clearance"]:
            if not directional:
                return self._stop(step, "emergency_clearance")
            emergency_tripped = True

        creeping = forward_clearance <= p["forward_stop_clearance"]

        ttc_tripped = False
        if p["enable_ttc"]:
            command_age = (
                math.inf if self.last_command_time is None
                else (step - self.last_command_time) * CONTROL_DT)
            effective_speed = gap_logic.conservative_ttc_speed(
                current_speed, self.last_commanded_speed, command_age,
                p["ttc_command_speed_timeout_sec"],
                p["ttc_command_fallback_max_odom_speed"])
            if effective_speed >= p["ttc_min_brake_speed"]:
                min_ttc = gap_logic.minimum_ttc(
                    window, window_valid, beam_angles, effective_speed,
                    boundaries, p["ttc_min_closing_speed"],
                    p["car_width"] / 2.0 + p["safety_margin"],
                    math.tan(self.steering_basis) / p["wheelbase"],
                    p["laser_offset_x"], p["laser_offset_y"])
                if min_ttc <= p["ttc_threshold_sec"]:
                    if not directional:
                        return self._stop(step, "ttc_brake")
                    ttc_tripped = True

        # Gap selection
        closest_idx, closest_dist = gap_logic.closest_valid(window, window_valid)
        half_width = p["car_width"] / 2.0 + effective_margin
        processed = gap_logic.disparity_extend(
            window, self.lidar.angle_increment, p["disparity_threshold"], half_width)
        if closest_idx is not None:
            processed = gap_logic.safety_bubble(
                processed, closest_idx, closest_dist,
                self.lidar.angle_increment, half_width)
        gap_start, gap_end, used_fallback = gap_logic.find_gap_with_fallback(
            processed, p["min_gap_distance"], p["fallback_min_gap_distance"],
            self.lidar.angle_increment, p["min_centerline_gap_width"],
            previous_target_idx=self.previous_target_idx,
            switch_margin=p["gap_switch_margin"])

        if gap_start is None:
            return self._stop(step, "no_safe_gap")

        if emergency_tripped or ttc_tripped:
            creeping = True

        if p["aim_mode"] == "physical_center":
            target_in_window = gap_logic.aim_physical_center(
                processed, gap_start, gap_end, self.lidar.angle_increment,
                free_depths=window)
        else:
            target_in_window = gap_logic.aim_within_gap(
                processed, gap_start, gap_end, beam_angles)
        self.previous_target_idx = target_in_window
        target_angle = float(beam_angles[target_in_window])
        target_distance = float(processed[target_in_window])

        centering_bias = 0.0
        if p["enable_centering"] and not creeping:
            centering_bias, _w = gap_logic.corridor_centering_bias(
                side_left, side_right, target_angle, target_distance,
                p["centering_gain"], p["centering_max_steering"],
                math.radians(p["centering_full_bearing_deg"]),
                math.radians(p["centering_zero_bearing_deg"]),
                p["centering_full_forward_depth"],
                p["centering_zero_forward_depth"],
                p["centering_full_side_distance"],
                p["centering_zero_side_distance"])

        desired_steering = float(np.clip(
            p["steering_gain"] * target_angle + centering_bias,
            -p["max_steering_angle"], p["max_steering_angle"]))

        dt = CONTROL_DT
        if self.last_command_time is not None:
            dt = min(p["command_slew_max_dt"],
                     max(0.0, (step - self.last_command_time) * CONTROL_DT))
        else:
            dt = p["command_slew_max_dt"]

        steering = gap_logic.slew_rate_limit(
            desired_steering, self.steering_basis, dt, p["max_steering_rate"])
        self.steering_basis = steering

        limited_curvature = math.tan(desired_steering) / p["wheelbase"]
        curve_speed = gap_logic.curvature_speed_limit(
            limited_curvature, p["max_lateral_accel"], p["max_speed"])
        normal_speed = max(p["min_speed"], curve_speed)

        clearance_speed = gap_logic.braking_speed_limit(
            forward_clearance, p["forward_stop_clearance"],
            p["max_braking_decel"], p["max_speed"])
        if creeping:
            creep_reserve = max(p["emergency_stop_clearance"],
                                p["forward_stop_clearance"] / 2.0)
            clearance_speed = min(
                p["escape_creep_speed"],
                gap_logic.braking_speed_limit(
                    forward_clearance, creep_reserve,
                    p["max_braking_decel"], p["max_speed"]))

        desired_speed = min(normal_speed, clearance_speed)
        binding = 'curve' if normal_speed <= clearance_speed else 'clearance'
        if creeping:
            binding = 'creep'
        if used_fallback and effective_corner < desired_speed:
            desired_speed = min(desired_speed, effective_corner)
            binding = 'corner'
        else:
            desired_speed = min(desired_speed, effective_corner) if used_fallback else desired_speed

        ramp_basis = max(self.last_commanded_speed,
                         min(abs(float(current_speed)), p["max_speed"]))
        accel_ceiling = ramp_basis + p["max_acceleration"] * dt
        speed = min(desired_speed, accel_ceiling)
        if accel_ceiling < desired_speed:
            binding = 'accel'
        self.binding_cap = binding

        self.last_commanded_speed = speed
        self.last_command_time = step
        self.stopped = False
        return steering, float(speed)

    def _adaptive_width(self, left, right):
        p = self.p
        if not p["enable_adaptive_width"]:
            return 1.0, p["safety_margin"], p["corner_speed"]
        width_factor = gap_logic.corridor_width_factor(
            left, right, p["adaptive_width_narrow"], p["adaptive_width_reference"])
        margin = gap_logic.scale_between(
            width_factor, p["min_safety_margin"], p["safety_margin"])
        corner = gap_logic.scale_between(
            width_factor, p["corner_speed"], p["corner_speed_wide"])
        return width_factor, margin, corner

    def _stop(self, step, reason):
        self.last_commanded_speed = 0.0
        self.last_command_time = step
        self.stopped = True
        self.stop_reason = reason
        self.binding_cap = 'stop'
        return self.steering_basis, 0.0


def run_track(track: str, params: dict, seed: int, timeout_s: float) -> dict:
    plant = make_plant(track, seed)
    line = plant.env.unwrapped.track.raceline
    start = np.array([[line.xs[0], line.ys[0], line.yaws[0]]], dtype=float)
    obs, _ = plant.reset(start)
    lidar = plant.env.unwrapped.sim.scan_sims[0]
    # Use the config's lidar for geometry: angle_min/increment come from the env.
    class _Lidar:
        pass
    lid = _Lidar()
    lid.angle_min = float(-components_half_fov())
    lid.angle_increment = float(components_increment())
    lid.range_min = float(components_range_min())
    ctrl = GapController(params, lid)

    max_steps = int(math.ceil(timeout_s / CONTROL_DT))
    started = time.monotonic()
    stop_steps = 0
    min_forward = math.inf
    corridor_offsets = []
    speeds = []
    prev_idx = None
    max_cross = 0.0
    reference = np.column_stack((line.xs, line.ys))
    stuck_steps = 0
    last_pos = None
    stationary_window = 0
    cap_histogram = {}

    for step in range(max_steps):
        ego = obs["agent_0"]
        state = np.asarray(ego["std_state"], dtype=float)
        speed_meas = float(state[3])
        steering, cmd_speed = ctrl.command(ego["scan"], speed_meas, step)
        stop_steps += int(ctrl.stopped)
        speeds.append(cmd_speed)
        if ctrl.binding_cap is not None:
            cap_histogram[ctrl.binding_cap] = cap_histogram.get(ctrl.binding_cap, 0) + 1

        # stuck detection: commanded to move but not moving
        pos = plant.truth(0)[:2]
        if last_pos is not None:
            moved = float(np.hypot(*(pos - last_pos)))
            if cmd_speed > 0.05 and speed_meas < 0.05 and moved < 0.005:
                stationary_window += 1
            else:
                stationary_window = 0
            stuck_steps = max(stuck_steps, stationary_window)
        last_pos = pos

        # corridor offset (same definition as run_validation)
        co = corridor_offset(ego["scan"], lid)
        if co is not None:
            corridor_offsets.append(co)
        idx = nearest_index(reference, pos, prev_idx)
        prev_idx = idx
        max_cross = max(max_cross, cross_track(reference, idx, pos))

        # min forward scan (+-30 deg)
        fs = forward_scan_min(ego["scan"], lid)
        if math.isfinite(fs):
            min_forward = min(min_forward, fs)

        obs, _r, done, _t, info = plant.step([(steering, cmd_speed)])
        if done or obs["agent_0"]["collision"]:
            break

    plant_report = plant.report()
    plant.close()
    wall_time = time.monotonic() - started
    laps = float(info["lap_counts"][0])
    collision = bool(obs["agent_0"]["collision"]) or plant_report.get("contact_steps", 0) > 0
    result = {
        "track": track,
        "passed": bool(laps >= 1.0 and not collision),
        "laps": laps,
        "collision": collision,
        "sim_time_s": round(float(info["sim_time"]), 2),
        "wall_time_s": round(wall_time, 2),
        "steps": step + 1,
        "stop_steps": stop_steps,
        "mean_speed": round(float(np.mean(speeds)), 3) if speeds else 0.0,
        "min_forward_scan_m": round(min_forward, 3) if math.isfinite(min_forward) else None,
        "max_cross_track_m": round(max_cross, 3),
        "mean_corridor_offset_m": (
            round(float(np.mean(np.abs(corridor_offsets))), 4)
            if corridor_offsets else None),
        "max_stuck_steps": stuck_steps,
        "cap_histogram": cap_histogram,
    }
    result.update(plant_report)
    return result


_COMPONENTS_CACHE = None


def _components():
    global _COMPONENTS_CACHE
    if _COMPONENTS_CACHE is None:
        _COMPONENTS_CACHE = env_components(PROFILES[DEFAULT_PROFILE], CALIBRATION)
    return _COMPONENTS_CACHE


def components_half_fov():
    return -_components()["lidar_config"].angle_min


def components_increment():
    cfg = _components()["lidar_config"]
    return (cfg.angle_max - cfg.angle_min) / (cfg.num_beams - 1) if hasattr(cfg, "num_beams") else (cfg.angle_max - cfg.angle_min) / 1080.0


def components_range_min():
    return _components()["lidar_config"].range_min


def corridor_offset(scan, lid):
    clean, valid = gap_logic.sanitize_ranges(scan, max_range=10.0, range_min=lid.range_min)
    angles = lid.angle_min + np.arange(clean.size, dtype=float) * lid.angle_increment
    half_span = math.radians(60.0) / 2.0
    left = gap_logic.side_wall_distance(clean, valid, angles, math.pi / 2.0, half_span)
    right = gap_logic.side_wall_distance(clean, valid, angles, -math.pi / 2.0, half_span)
    if not (math.isfinite(left) and math.isfinite(right)):
        return None
    if max(left, right) > 5.0:
        return None
    return (right - left) / 2.0


def forward_scan_min(scan, lid):
    half = math.radians(30.0)
    lo = int((-half - lid.angle_min) / lid.angle_increment)
    hi = int((half - lid.angle_min) / lid.angle_increment)
    vals = np.asarray(scan[max(0, lo):hi + 1], dtype=float)
    vals = vals[np.isfinite(vals) & (vals > 0.0) & (vals >= lid.range_min)]
    return float(vals.min()) if vals.size else math.inf


def nearest_index(reference, pos, prev_idx):
    if prev_idx is None:
        d = np.hypot(reference[:, 0] - pos[0], reference[:, 1] - pos[1])
        return int(np.argmin(d))
    n = len(reference)
    window = 80
    best = prev_idx
    best_d = math.inf
    for k in range(-window, window + 1):
        i = (prev_idx + k) % n
        d = (reference[i, 0] - pos[0]) ** 2 + (reference[i, 1] - pos[1]) ** 2
        if d < best_d:
            best_d = d
            best = i
    return best


def cross_track(reference, idx, pos):
    return float(math.hypot(reference[idx, 0] - pos[0], reference[idx, 1] - pos[1]))


def parse_set(values):
    params = dict(DEFAULTS)
    for item in values or []:
        if "=" not in item:
            raise SystemExit(f"--set expects name=value, got {item!r}")
        name, _, raw = item.partition("=")
        name = name.strip()
        if name not in DEFAULTS:
            raise SystemExit(f"unknown parameter {name!r}")
        default = DEFAULTS[name]
        if isinstance(default, bool):
            params[name] = raw.strip().lower() in ("1", "true", "yes", "on")
        elif isinstance(default, str):
            params[name] = raw.strip()
        else:
            params[name] = type(default)(float(raw))
    return params


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--track", default="asb_10000",
                        help="track name, 'indoor', 'gym', or 'all'")
    parser.add_argument("--seed", type=int, default=12345)
    parser.add_argument("--timeout", type=float, default=240.0,
                        help="simulated seconds before giving up on a lap")
    parser.add_argument("--set", action="append", default=[],
                        help="override a parameter, name=value (repeatable)")
    parser.add_argument("--output", help="write JSON results here")
    args = parser.parse_args(argv)

    if args.track == "all":
        selected = list(INDOOR_TRACKS) + list(GYM_TRACKS)
    elif args.track == "indoor":
        selected = list(INDOOR_TRACKS)
    elif args.track == "gym":
        selected = list(GYM_TRACKS)
    else:
        selected = [args.track]

    params = parse_set(args.set)
    results = []
    for track in selected:
        res = run_track(track, params, args.seed, args.timeout)
        results.append(res)
        status = "PASS" if res["passed"] else "FAIL"
        print(f"{status} {track:14s} laps={res['laps']:.2f} "
              f"time={res['sim_time_s']:7.1f}s vmean={res['mean_speed']:.2f} "
              f"stops={res['stop_steps']} stuck={res['max_stuck_steps']} "
              f"contact={res.get('contact_steps', 0)} "
              f"minfwd={res['min_forward_scan_m']} caps={res['cap_histogram']}", flush=True)

    passed = sum(1 for r in results if r["passed"])
    print(json.dumps({"passed": passed, "total": len(results),
                      "results": results}, indent=2))
    if args.output:
        Path(args.output).write_text(json.dumps(results, indent=2) + "\n")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
