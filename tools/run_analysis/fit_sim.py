"""Fit the simulator's car model to a recorded run.

Plays the run's recorded /drive commands, tick for tick, into the simulator's
car (tools/f1tenth_sim/sim_fidelity FidelityPlant) on an empty floor, and
compares how the simulated car moved with how the real one did (scan_motion).
Repeats for a grid of calibration values and reports which fits best.

Open-loop on purpose: the real car's commands depended on what it saw, which
the simulated car does not reproduce, so a closed-loop comparison would mix
controller and plant. Replaying the commands isolates the plant -- speed loop,
transport lag, servo, grip -- which is exactly what calibration.py models.

The empty floor is a ~100 m disc, far bigger than any run, so no wall is
ever in reach.
"""

from __future__ import annotations

import dataclasses
import itertools
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT / "tools" / "f1tenth_sim"
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))

from sim_fidelity.bootstrap import bootstrap  # noqa: E402

bootstrap()
sys.path.insert(0, str(ROOT / "src" / "racerbot_sim"))

CONTROL_DT = 0.025
_FLOOR = None


def empty_floor() -> str:
    global _FLOOR
    if _FLOOR is None:
        from racerbot_sim import tracks
        directory = ROOT / ".sim" / "fit_sim_floor"
        line = tracks.rounded_rectangle_centerline(27.0, 27.0, 25.0, spacing=0.5)
        _FLOOR = str(tracks.write_track(directory, "floor", line, 48.0, resolution=0.1).with_suffix(""))
    return _FLOOR


def simulate_commands(commands: np.ndarray, calibration, profile_name: str = "car", seed: int = 1):
    """Drive the plant with ``commands`` ((N, 2) steering, speed at 40 Hz);
    return (speed, yaw_rate) arrays of the true simulated state per tick."""
    import gymnasium as gym
    import f1tenth_gym  # noqa: F401
    from f1tenth_gym.envs.dynamic_models import DynamicModel
    from f1tenth_gym.envs.env_config import EnvConfig, ObservationConfig, SimulationConfig
    from f1tenth_gym.envs.integrators import IntegratorType
    from f1tenth_gym.envs.observation import ObservationType
    from sim_fidelity import PROFILES
    from sim_fidelity.plant import FidelityPlant, env_components

    profile = PROFILES[profile_name]
    comp = env_components(profile, calibration)
    config = EnvConfig(
        seed=seed, map_name=empty_floor(), params=comp["params"], num_agents=1,
        control_config=comp["control_config"],
        simulation_config=SimulationConfig(timestep=CONTROL_DT, integrator_timestep=0.005,
                                           integrator=IntegratorType.RK4,
                                           dynamics_model=DynamicModel.ST, max_laps=1000),
        observation_config=ObservationConfig(type=ObservationType.DIRECT),
        lidar_config=comp["lidar_config"], collision_check=comp["collision_check"],
        render_enabled=False)
    env = gym.make("f1tenth_gym:f1tenth-v0", config=config, render_mode=None)
    plant = FidelityPlant(env, profile, 1, seed, CONTROL_DT, calibration)
    plant.reset(np.array([[0.0, -25.0, 0.0]]))
    speed, yaw_rate = [], []
    for steering, v in commands:
        plant.step([(float(steering), float(v))])
        s = plant.truth(0)
        speed.append(float(s[3]))
        yaw_rate.append(float(s[5]))
    plant.close()
    return np.array(speed), np.array(yaw_rate)


def resample_commands(drive: np.ndarray, t0: float, t1: float) -> tuple:
    """The /drive command in force at every 40 Hz tick in [t0, t1)."""
    ticks = np.arange(t0, t1, CONTROL_DT)
    j = np.clip(np.searchsorted(drive[:, 0], ticks, side="right") - 1, 0, len(drive) - 1)
    return ticks, drive[j][:, 2:4]


def score(ticks, sim_speed, sim_yaw_rate, motion, skip_s: float = 1.0) -> dict:
    """RMS speed and yaw-rate error of the simulated car against measured motion."""
    keep = (motion.t > ticks[0] + skip_s) & (motion.t < ticks[-1])
    t = motion.t[keep]
    v = np.interp(t, ticks, sim_speed)
    w = np.interp(t, ticks, sim_yaw_rate)
    return dict(speed_rms=float(np.sqrt(np.mean((v - motion.v[keep]) ** 2))),
                yaw_rate_rms=float(np.sqrt(np.mean((w - motion.w[keep]) ** 2))))


def fit(run, motion, t0: float, t1: float, grid: dict, base=None) -> list:
    """Score every combination in ``grid`` ({calibration field: [values]}).

    Returns [(score dict, {field: value}), ...] sorted by speed + yaw-rate RMS.
    """
    from sim_fidelity import CALIBRATION

    base = base or CALIBRATION
    ticks, commands = resample_commands(run.drive, t0, t1)
    results = []
    names = sorted(grid)
    for values in itertools.product(*(grid[n] for n in names)):
        setting = dict(zip(names, values))
        cal = dataclasses.replace(base, **setting)
        sp, wr = simulate_commands(commands, cal)
        s = score(ticks, sp, wr, motion)
        results.append((s, setting))
    results.sort(key=lambda r: r[0]["speed_rms"] + r[0]["yaw_rate_rms"])
    return results


def main(argv=None) -> int:
    import argparse
    sys.path.insert(0, str(ROOT / "tools"))
    from run_analysis.runlog import load_run
    from run_analysis.scan_motion import scan_motion

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--window", type=int, default=0, help="which LB hold to use (0 = first)")
    ap.add_argument("--grid", action="append", default=[],
                    help="field=v1,v2,... (repeatable), e.g. speed_delay_ticks=1,4,8")
    args = ap.parse_args(argv)
    run = load_run(args.run_dir)
    t0, t1 = run.lb_windows()[args.window]
    motion = scan_motion(run, t0, t1)
    grid = {}
    for item in args.grid or ["speed_delay_ticks=1,4,8", "steering_delay_ticks=1,3,5"]:
        name, _, vals = item.partition("=")
        grid[name] = [type_of(name)(float(v)) for v in vals.split(",")]
    for s, setting in fit(run, motion, t0, t1, grid)[:10]:
        print(f"speed RMS {s['speed_rms']:.3f} m/s  yaw-rate RMS {s['yaw_rate_rms']:.3f} rad/s  {setting}")
    return 0


def type_of(name: str):
    from sim_fidelity import CALIBRATION
    return type(getattr(CALIBRATION, name))


if __name__ == "__main__":
    sys.exit(main())
