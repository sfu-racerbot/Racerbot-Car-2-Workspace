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
    python3 tools/f1tenth_sim/eval_gap.py --track indoor_oval --seeds 2 --workers 2 --set enable_opponent_awareness=true
    python3 tools/f1tenth_sim/eval_gap.py --track all --baseline

With opponent awareness on and one car on track, every moving track is a
false mover: a wall taken for a car. ``false_mover_ticks`` counts the ticks
with any moving track and ``false_mover_tracks`` the distinct tracks that
moved (recognition.false_movers). The aggregate sums both over the runs.

Exit code is 0 when every selected track passes (one lap, no wall contact).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# tools/, for run_analysis. Appended, so it shadows nothing.
sys.path.append(str(Path(__file__).resolve().parents[1]))
from sim_fidelity.bootstrap import bootstrap  # noqa: E402

ROOT = bootstrap()
sys.path.insert(0, str(ROOT / 'src' / 'imu_bridge'))
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
from gap_mirror import GapController, node_params  # noqa: E402
from imu_bridge.vehicle_state import steering_from_yaw_rate  # noqa: E402
from recognition import false_movers, moving_track_ids  # noqa: E402
from racerbot_sim import tracks as sim_tracks  # noqa: E402
from run_analysis import weave  # noqa: E402
from sim_fidelity import CALIBRATION, PROFILES  # noqa: E402
from sim_fidelity.plant import DEFAULT_PROFILE, FidelityPlant, env_components  # noqa: E402

CONTROL_DT = 0.025
INTEGRATOR_DT = 0.005

GYM_TRACKS = ("Spielberg", "Silverstone", "BrandsHatch")
INDOOR_TRACKS = ("indoor_oval", "indoor_tight", "indoor_wide", "asb_10000", "asb_sharp")
# Harness-only layouts (not racerbot_sim LAYOUTS). asb_sharp is asb_10000
# with a centreline radius equal to the corridor's half-width, so every INNER
# corner is a true right angle -- the real hallway's blind corners, which the
# rounded racerbot_sim layouts only approximate. (The outer corner stays
# rounded, which leaves less room than a square one: conservative.)
EXTRA_LAYOUTS = {
    "asb_sharp": dict(half_x=7.5, half_y=5.5, radius=0.75, corridor_width=1.5),
    # As narrow as the loop driven on 2026-10-19 (run 20261019-200703): the
    # car's side-wall distances summed to 1.24 m at the median and 0.83 m at
    # the 5th percentile. Every layout above is 1.4 m or wider, which is why
    # the simulator never showed that run's TTC brakes
    # (docs/gap-follow-real-run-2026-10-19.md).
    "narrow_1p2": dict(half_x=4.5, half_y=2.6, radius=1.0, corridor_width=1.2),
    "narrow_1p0": dict(half_x=4.5, half_y=2.6, radius=1.0, corridor_width=1.0),
}
NARROW_TRACKS = ("narrow_1p2", "narrow_1p0")

# Tracks imported from real occupancy maps by track_import.py (SLAM maps,
# other teams' maps). Each is a gym triplet in tools/f1tenth_sim/maps/.
MAPS_DIR = Path(__file__).resolve().parent / "maps"


def map_tracks() -> tuple:
    import yaml
    catalog = MAPS_DIR / "catalog.yaml"
    if not catalog.exists():
        return ()
    return tuple(sorted(yaml.safe_load(catalog.read_text()) or {}))

# Parameters: exactly what the car loads (gap_follow.yaml, via
# gap_mirror.node_params), every one overridable with --set name=value.
DEFAULTS = node_params()


# Indoor layouts already written by this process (or by a parent process
# before it forked workers -- see sweep_gap.py), so parallel runs never race
# on rewriting the same map files.
_BUILT = {}


def build_indoor_track(name: str) -> Path:
    if name not in _BUILT:
        directory = Path(ROOT) / ".sim" / "eval_gap_tracks"
        if name in EXTRA_LAYOUTS:
            spec = dict(EXTRA_LAYOUTS[name])
            width = spec.pop("corridor_width")
            _BUILT[name] = sim_tracks.write_track(
                directory, name, sim_tracks.rounded_rectangle_centerline(**spec), width)
        else:
            _BUILT[name] = sim_tracks.build(name, directory)
    return _BUILT[name]


def track_path(track_spec: str) -> str:
    """The path gym loads for ``track_spec``: an indoor/narrow layout (built
    on first use), an imported map, or a gym circuit name as-is."""
    if track_spec in INDOOR_TRACKS or track_spec in NARROW_TRACKS:
        return str(build_indoor_track(track_spec).with_suffix(""))
    if track_spec in map_tracks():
        return str(MAPS_DIR / track_spec)
    return track_spec


def track_map_yaml(track_spec: str):
    """The map_server yaml behind a track, for plotting; None for gym circuits."""
    path = track_path(track_spec)
    if path == track_spec:
        return None
    return Path(path).parent / f"{Path(path).name}_map.yaml"


def make_plant(track_spec: str, seed: int) -> FidelityPlant:
    """Build a fidelity plant for a gym circuit name or an indoor layout."""
    components = env_components(PROFILES[DEFAULT_PROFILE], CALIBRATION)
    map_name = track_path(track_spec)
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


def _reverse_laps(plant) -> float:
    """Laps driven *against* the track's direction. The gym only counts
    forward laps; its signed Frenet progress goes negative in reverse."""
    env = plant.env.unwrapped
    s_max = env.track.centerline.spline.s_frame_max
    return max(0.0, -float(env.cumulative_s[0]) / s_max)


def run_track(track: str, params: dict, seed: int, timeout_s: float,
              reverse: bool = False) -> dict:
    """One lap of ``track``. ``reverse`` starts facing backwards, so the
    indoor layouts (all counter-clockwise) get exercised with right turns."""
    plant = make_plant(track, seed)
    line = plant.env.unwrapped.track.raceline
    start = np.array([[line.xs[0], line.ys[0],
                       line.yaws[0] + (math.pi if reverse else 0.0)]], dtype=float)
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
    steerings = []
    prev_idx = None
    max_cross = 0.0
    reference = np.column_stack((line.xs, line.ys))
    stuck_steps = 0
    last_pos = None
    stationary_window = 0
    cap_histogram = {}
    stop_reasons = {}
    min_body_clear = math.inf
    # Braking, from ground truth: speed shed (sum of every drop in true
    # speed, m/s) and ticks spent decelerating harder than 2 m/s^2. A lap
    # that holds its speed scores low on both.
    speed_shed = 0.0
    hard_brake_ticks = 0
    last_true_speed = None
    # Per tick, the ids of the tracks marked moving (awareness on only).
    moving_per_tick = []

    for step in range(max_steps):
        ego = obs["agent_0"]
        state = np.asarray(ego["std_state"], dtype=float)
        speed_meas = float(state[3])
        # The gym's true yaw rate (std_state is x, y, delta, v, yaw, yaw_rate,
        # beta). The mirror reads it only with enable_steering_damping or
        # enable_adaptive_grip on, so with both off this changes nothing.
        steering, cmd_speed = ctrl.command(ego["scan"], speed_meas, step,
                                           horizontal_accel=0.0, lb_pressed=True,
                                           fused_speed=float(plant.truth(0)[3]),
                                           yaw_rate=float(state[5]),
                                           measured_steering=steering_from_yaw_rate(
                                               float(state[5]), float(state[3]),
                                               params['wheelbase'], 0.3))
        steerings.append(steering)
        if ctrl.opponents is not None:
            moving_per_tick.append(moving_track_ids(ctrl.opponents.tracker.tracks))
        stop_steps += int(ctrl.stopped)
        min_body_clear = min(min_body_clear, ctrl.telemetry.get("min_clear", math.inf))
        if ctrl.stopped:
            reason = ctrl.stop_reason
            stop_reasons[reason] = stop_reasons.get(reason, 0) + 1
        speeds.append(cmd_speed)
        if ctrl.binding_cap is not None:
            cap_histogram[ctrl.binding_cap] = cap_histogram.get(ctrl.binding_cap, 0) + 1

        true_speed = float(plant.truth(0)[3])
        if last_true_speed is not None:
            drop = last_true_speed - true_speed
            if drop > 0.0:
                speed_shed += drop
            if drop > 2.0 * CONTROL_DT:
                hard_brake_ticks += 1
        last_true_speed = true_speed

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
        if reverse and _reverse_laps(plant) >= 1.0:
            break
        if done or obs["agent_0"]["collision"]:
            break

    laps = _reverse_laps(plant) if reverse else float(info["lap_counts"][0])
    plant_report = plant.report()
    plant.close()
    wall_time = time.monotonic() - started
    collision = bool(obs["agent_0"]["collision"]) or plant_report.get("contact_steps", 0) > 0
    # Weave: the commanded steering's direction changes per second, each a swing over 0.2 rad.
    reversals = weave.reversals_per_s(np.arange(len(steerings)) * CONTROL_DT,
                                      np.asarray(steerings), swing_rad=weave.SWING_RAD)
    result = {
        "track": track,
        "seed": seed,
        "reverse": reverse,
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
        "stop_reasons": stop_reasons,
        "min_body_clearance_m": (round(min_body_clear, 3)
                                 if math.isfinite(min_body_clear) else None),
        "speed_shed_mps": round(speed_shed, 2),
        "hard_brake_ticks": hard_brake_ticks,
        "steering_reversals_per_s": round(reversals, 3),
    }
    # One car alone on track, so every moving track is a false mover. Without
    # awareness nothing is measured, so None rather than a count of zero.
    if ctrl.opponents is not None:
        result.update(false_movers(moving_per_tick))
    else:
        result.update(false_mover_ticks=None, false_mover_tracks=None)
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


def select_tracks(spec: str) -> list:
    """'all', 'indoor', 'narrow', 'maps', 'gym', or comma-separated names."""
    if spec == "all":
        return list(INDOOR_TRACKS) + list(NARROW_TRACKS) + list(map_tracks()) + list(GYM_TRACKS)
    if spec == "indoor":
        return list(INDOOR_TRACKS)
    if spec == "narrow":
        return list(NARROW_TRACKS)
    if spec == "maps":
        return list(map_tracks())
    if "," in spec:
        return [name for part in spec.split(",") for name in select_tracks(part)]
    if spec == "gym":
        return list(GYM_TRACKS)
    return [spec]


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


def _job(job):
    track, seed, params, timeout, reverse = job
    return run_track(track, params, seed, timeout, reverse)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--track", default="asb_10000",
                        help="track name, 'indoor', 'narrow', 'maps', 'gym', 'all', "
                             "or a comma-separated list of any of these")
    parser.add_argument("--seed", type=int, default=12345, help="the first seed")
    parser.add_argument("--seeds", type=int, default=1,
                        help="seeds per track: --seed and the ones after it")
    parser.add_argument("--workers", type=int, default=1,
                        help="processes to run the (track, seed) jobs in")
    parser.add_argument("--timeout", type=float, default=240.0,
                        help="simulated seconds before giving up on a lap")
    parser.add_argument("--set", action="append", default=[],
                        help="override a parameter, name=value (repeatable)")
    parser.add_argument("--output", help="write JSON results here")
    parser.add_argument("--reverse", action="store_true",
                        help="drive each track the other way round")
    args = parser.parse_args(argv)

    selected = select_tracks(args.track)
    params = parse_set(args.set)
    for track in selected:
        track_path(track)  # build the layouts before any worker process forks
    jobs = [(track, args.seed + k, params, args.timeout, args.reverse)
            for track in selected for k in range(args.seeds)]
    if args.workers > 1:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            results = list(pool.map(_job, jobs))
    else:
        results = [_job(job) for job in jobs]
    for res in results:
        status = "PASS" if res["passed"] else "FAIL"
        print(f"{status} {res['track']:14s} seed={res['seed']} laps={res['laps']:.2f} "
              f"time={res['sim_time_s']:7.1f}s vmean={res['mean_speed']:.2f} "
              f"stops={res['stop_steps']} stuck={res['max_stuck_steps']} "
              f"contact={res.get('contact_steps', 0)} "
              f"minfwd={res['min_forward_scan_m']} rev={res['steering_reversals_per_s']:.2f}/s "
              f"false_mover_ticks={res['false_mover_ticks']} "
              f"false_mover_tracks={res['false_mover_tracks']} "
              f"caps={res['cap_histogram']}", flush=True)

    passed = sum(1 for r in results if r["passed"])
    measured = [r for r in results if r["false_mover_ticks"] is not None]
    fm_ticks = sum(r["false_mover_ticks"] for r in measured) if measured else None
    fm_tracks = sum(r["false_mover_tracks"] for r in measured) if measured else None
    if measured:
        print(f"false movers over {len(measured)} runs with awareness on: "
              f"false_mover_ticks={fm_ticks} false_mover_tracks={fm_tracks} "
              "(tracks are counted within each run, then summed)", flush=True)
    print(json.dumps({"passed": passed, "total": len(results),
                      "false_mover_ticks": fm_ticks, "false_mover_tracks": fm_tracks,
                      "results": results}, indent=2))
    if args.output:
        Path(args.output).write_text(json.dumps(results, indent=2) + "\n")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
