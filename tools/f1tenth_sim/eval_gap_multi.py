#!/usr/bin/env python3
"""Several gap_follow cars on one track at once.

Every car runs its own GapController (tools/f1tenth_sim/gap_mirror.py, the
node's pipeline) on its own simulated LiDAR, which sees the other cars as
obstacles -- exactly what happens when two of these cars share a hallway.
Cars start one behind another along the centreline, ``--spacing`` metres
apart, the leader first. Giving the cars different top speeds (``--speeds``)
makes the faster ones catch the slower ones, so following, and passing where
the corridor allows it, actually happen.

A run passes when no car touches a wall or another car (checked against the
real chassis by sim_fidelity, never gym's own flag) and every car completes
``--laps`` laps within the time limit. A car that catches a slower one and
follows it is fine; a car that stops behind it for good is not, and shows up
as an unfinished lap.

    python3 tools/f1tenth_sim/eval_gap_multi.py --track indoor_wide --cars 3 --seeds 4
    python3 tools/f1tenth_sim/eval_gap_multi.py --track asb_10000 --cars 2 --speeds 3.5,2.0

Exit code 0 when every run passes.
"""

from __future__ import annotations

import argparse
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_gap  # noqa: E402  (bootstraps gym and the car's packages)

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
from f1tenth_gym.envs.dynamic_models import DynamicModel  # noqa: E402
from f1tenth_gym.envs.env_config import EnvConfig, ObservationConfig, SimulationConfig  # noqa: E402
from f1tenth_gym.envs.integrators import IntegratorType  # noqa: E402
from f1tenth_gym.envs.observation import ObservationType  # noqa: E402

from gap_mirror import GapController  # noqa: E402
from sim_fidelity import CALIBRATION, PROFILES  # noqa: E402
from sim_fidelity.plant import DEFAULT_PROFILE, FidelityPlant, env_components  # noqa: E402

CONTROL_DT = eval_gap.CONTROL_DT


def make_plant(track: str, seed: int, cars: int) -> FidelityPlant:
    components = env_components(PROFILES[DEFAULT_PROFILE], CALIBRATION)
    config = EnvConfig(
        seed=seed, map_name=eval_gap.track_path(track), params=components["params"],
        num_agents=cars, control_config=components["control_config"],
        simulation_config=SimulationConfig(
            timestep=CONTROL_DT, integrator_timestep=eval_gap.INTEGRATOR_DT,
            integrator=IntegratorType.RK4, dynamics_model=DynamicModel.ST, max_laps=1000),
        observation_config=ObservationConfig(type=ObservationType.DIRECT),
        lidar_config=components["lidar_config"], collision_check=components["collision_check"],
        render_enabled=False)
    env = gym.make("f1tenth_gym:f1tenth-v0", config=config, render_mode=None)
    # Every car is a car under test, so every car gets the real sensor error.
    return FidelityPlant(env, PROFILES[DEFAULT_PROFILE], cars, seed, CONTROL_DT, CALIBRATION,
                         degrade_agents=frozenset(range(cars)))


def start_poses(line, cars: int, spacing: float, reverse: bool) -> np.ndarray:
    """Leader at the centreline's first point, the rest ``spacing`` metres
    behind each other along the direction of travel."""
    xs, ys, yaws = np.asarray(line.xs), np.asarray(line.ys), np.asarray(line.yaws)
    seg = np.hypot(np.diff(xs, append=xs[0]), np.diff(ys, append=ys[0]))
    s = np.concatenate(([0.0], np.cumsum(seg)[:-1]))
    total = float(np.sum(seg))
    poses = []
    for k in range(cars):
        # Behind = smaller s when driving forward, larger s when reversed.
        target = (-k * spacing if not reverse else k * spacing) % total
        i = int(np.argmin(np.abs(s - target)))
        poses.append((xs[i], ys[i], yaws[i] + (math.pi if reverse else 0.0)))
    return np.array(poses, dtype=float)


class _Progress:
    """Laps driven along the centreline, from ground truth, in either
    direction. gym's own lap counter only counts the forward direction and
    only the first agent's start line, so it is not used for verdicts."""

    def __init__(self, reference: np.ndarray, start_xy, reverse: bool):
        self.ref = reference
        seg = np.hypot(*np.diff(np.vstack((reference, reference[:1])), axis=0).T)
        self.s = np.concatenate(([0.0], np.cumsum(seg)[:-1]))
        self.total = float(np.sum(seg))
        self.sign = -1.0 if reverse else 1.0
        self.idx = eval_gap.nearest_index(reference, np.asarray(start_xy), None)
        self.distance = 0.0

    def update(self, xy) -> float:
        new = eval_gap.nearest_index(self.ref, np.asarray(xy), self.idx)
        ds = self.s[new] - self.s[self.idx]
        if ds > self.total / 2.0:
            ds -= self.total
        elif ds < -self.total / 2.0:
            ds += self.total
        self.distance += self.sign * ds
        self.idx = new
        return self.laps

    @property
    def laps(self) -> float:
        return self.distance / self.total


def run_multi(track: str, params_list: list, seed: int, duration_s: float,
              spacing: float = 3.0, laps_required: float = 2.0, reverse: bool = False) -> dict:
    cars = len(params_list)
    plant = make_plant(track, seed, cars)
    line = plant.env.unwrapped.track.raceline
    poses = start_poses(line, cars, spacing, reverse)
    obs, _ = plant.reset(poses)
    lid = type("Lidar", (), {})()
    lid.angle_min = float(-eval_gap.components_half_fov())
    lid.angle_increment = float(eval_gap.components_increment())
    lid.range_min = float(eval_gap.components_range_min())
    ctrls = [GapController(p, lid) for p in params_list]
    reference = np.column_stack((line.xs, line.ys))
    progress = [_Progress(reference, poses[k, :2], reverse) for k in range(cars)]
    stats = [dict(stop_ticks=0, ttc_ticks=0, speed_sum=0.0, ticks=0, braking_ticks=0,
                  wall_contact=False, car_contact=False, laps=0.0, finish_time_s=None,
                  opponent_ticks=0, follow_ticks=0)
             for _ in range(cars)]
    min_gap = math.inf
    last_cmd = [0.0] * cars
    steps = int(math.ceil(duration_s / CONTROL_DT))
    for step in range(steps):
        commands = []
        for k, ctrl in enumerate(ctrls):
            ego = obs[f"agent_{k}"]
            state = np.asarray(ego["std_state"], dtype=float)
            steer, speed = ctrl.command(ego["scan"], float(state[3]), step)
            st = stats[k]
            st["ticks"] += 1
            st["speed_sum"] += speed
            st["stop_ticks"] += int(ctrl.stopped)
            st["ttc_ticks"] += int(ctrl.stopped and ctrl.stop_reason == "ttc_brake")
            st["braking_ticks"] += int(speed < last_cmd[k] - 0.05)
            # Ticks with a moving car in view (opponent awareness on), and
            # ticks where the follow cap bound -- with one car on track both
            # should be ~0, or walls are being taken for cars.
            st["opponent_ticks"] += int(ctrl.opponent_mode is not None)
            st["follow_ticks"] += int(ctrl.binding_cap == "follow")
            last_cmd[k] = speed
            commands.append((steer, speed))
        obs, _r, _done, _t, _info = plant.step(commands)
        truth = [plant.truth(k) for k in range(cars)]
        for a in range(cars):
            for b in range(a + 1, cars):
                min_gap = min(min_gap, math.hypot(truth[a][0] - truth[b][0], truth[a][1] - truth[b][1]))
        if any(obs[f"agent_{k}"]["collision"] for k in range(cars)):
            pair = plant._chassis_contact()
            walls = plant._wall_contact()
            for k in range(cars):
                stats[k]["car_contact"] |= k in pair
                stats[k]["wall_contact"] |= k in walls
            if pair or walls:
                break
        for k in range(cars):
            laps = progress[k].update(truth[k][:2])
            stats[k]["laps"] = round(laps, 3)
            if stats[k]["finish_time_s"] is None and laps >= laps_required:
                stats[k]["finish_time_s"] = round((step + 1) * CONTROL_DT, 2)
        if all(s["finish_time_s"] is not None for s in stats):
            break
    plant.close()
    for k, s in enumerate(stats):
        s["mean_speed"] = round(s.pop("speed_sum") / max(1, s["ticks"]), 3)
        s["max_speed_param"] = params_list[k]["max_speed"]
    contact = any(s["wall_contact"] or s["car_contact"] for s in stats)
    finished = all(s["finish_time_s"] is not None for s in stats)
    return dict(track=track, seed=seed, reverse=reverse, cars=stats,
                passed=bool(finished and not contact), contact=contact,
                min_center_distance_m=round(min_gap, 3), sim_time_s=round((step + 1) * CONTROL_DT, 2))


def _job(job):
    return run_multi(*job)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", default="indoor_wide", help="as eval_gap: a name, group, or comma list")
    ap.add_argument("--cars", type=int, default=2)
    ap.add_argument("--speeds", help="max_speed per car, comma-separated (leader first); "
                                     "default: the YAML's max_speed for the leader, 0.7x for the others")
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--first-seed", type=int, default=12345)
    ap.add_argument("--laps", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=90.0, help="simulated seconds per run")
    ap.add_argument("--spacing", type=float, default=3.0, help="metres between start positions")
    ap.add_argument("--reverse", action="store_true")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--set", action="append", default=[], help="override a parameter for every car")
    args = ap.parse_args(argv)
    base = eval_gap.parse_set(args.set)
    if args.speeds:
        speeds = [float(v) for v in args.speeds.split(",")]
        if len(speeds) != args.cars:
            raise SystemExit("--speeds needs one value per car")
    else:
        # The leader is the SLOWER car so the others catch it.
        speeds = [round(0.7 * base["max_speed"], 2)] + [base["max_speed"]] * (args.cars - 1)
    params_list = []
    for v in speeds:
        p = dict(base)
        p["max_speed"] = v
        p["min_speed"] = min(p["min_speed"], v)
        p["corner_speed"] = min(p["corner_speed"], v)
        p["corner_speed_wide"] = min(p["corner_speed_wide"], v)
        params_list.append(p)
    tracks = eval_gap.select_tracks(args.track)
    for t in tracks:
        eval_gap.track_path(t)
    jobs = [(t, params_list, args.first_seed + k, args.duration, args.spacing, args.laps, args.reverse)
            for t in tracks for k in range(args.seeds)]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(_job, jobs))
    ok = True
    for t in tracks:
        runs = [r for r in results if r["track"] == t]
        passed = sum(r["passed"] for r in runs)
        ok &= passed == len(runs)
        wall = sum(any(c["wall_contact"] for c in r["cars"]) for r in runs)
        car = sum(any(c["car_contact"] for c in r["cars"]) for r in runs)
        unfinished = sum(any(c["finish_time_s"] is None for c in r["cars"]) for r in runs)
        gap = min(r["min_center_distance_m"] for r in runs)
        speeds_txt = " ".join(f"{np.mean([r['cars'][k]['mean_speed'] for r in runs]):.2f}" for k in range(args.cars))
        print(f"{t:16s} cars {args.cars} pass {passed}/{len(runs)}  wall {wall}  car-car {car}  "
              f"unfinished {unfinished}  closest {gap:.2f}m  vmean/car {speeds_txt}", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
