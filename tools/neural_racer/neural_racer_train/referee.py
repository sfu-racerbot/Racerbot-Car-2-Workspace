"""The referee (plan Phase 9): races on the calibrated CPU simulator.

Every candidate is judged here, never in the GPU training simulator:
tools/f1tenth_sim's FidelityPlant with the 'car' profile, its own chassis
contact geometry (gym's collision flag is never a verdict), and the *exported
NumPy policy* behind the car's own observation code and safety wrapper --
exactly what neural_racer_node runs.

Drivers:
  NeuralDriver    policy_numpy + observation + safety_wrapper (the car's code)
  GapDriver       gap_mirror's GapController: gap_follow's per-scan pipeline,
                  held to the node by tools/f1tenth_sim's own parity test
  PursuitDriver   run_validation's PathFollower + CommandShaper + fallback
                  safety, as run_pure_solo runs pure_pursuit
"""
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from neural_racer_train import fidelity
from neural_racer import observation as obs
from neural_racer.policy_numpy import PolicyNumpy
from neural_racer.safety_wrapper import SafetyWrapper, WrapperConfig, scan_angles

SCAN_RANGE_MIN, SCAN_RANGE_MAX = 0.05, 25.0          # sim_fidelity's LiDAR config
GRID_STAGGER = 1.0           # m along the line; 0.6 overlapped 0.535 m bodies on bends


def _rv():
    fidelity.ensure_paths()
    import run_validation as rv
    return rv


def track_map_name(track: str) -> str:
    """gym circuit names pass through; tools/f1tenth_sim/maps catalog entries become paths."""
    import yaml
    if track.startswith('heldout_'):
        from neural_racer_train import heldout
        heldout.ensure_maps()
        return str(heldout.MAP_DIR / track)
    maps = fidelity.TOOLS_SIM / 'maps'
    catalog = maps / 'catalog.yaml'
    names = set(yaml.safe_load(catalog.read_text()) or {}) if catalog.exists() else set()
    return str(maps / track) if track in names else track


class NeuralDriver:
    """The car's code path: History -> build_observation -> PolicyNumpy -> SafetyWrapper."""

    def __init__(self, weights: dict, card: dict | None = None, v_cap: float = 2.5,
                 imu_offset_from_cog: float = 0.27, imu: bool = True):
        self.policy = PolicyNumpy(weights, card)
        self.wrapper = SafetyWrapper(WrapperConfig(max_speed=v_cap))
        self.v_cap, self.imu, self.d = v_cap, imu, imu_offset_from_cog
        self.history = obs.History()
        self.prev_steer = 0.0
        self.prev_state = None
        self.angles = None

    def _imu(self, s):
        if self.prev_state is None:
            return (float(s[5]), 0.0, 0.0)
        p = self.prev_state
        dt = 0.025
        vx, vy = s[3] * math.cos(s[6]), s[3] * math.sin(s[6])
        pvx, pvy = p[3] * math.cos(p[6]), p[3] * math.sin(p[6])
        w, w_dot = float(s[5]), (float(s[5]) - float(p[5])) / dt
        ax = (vx - pvx) / dt - w * vy - w * w * self.d
        ay = (vy - pvy) / dt + w * vx + w_dot * self.d
        return (w, float(ax), float(ay))

    def command(self, agent_obs, truth, step):
        scan = np.asarray(agent_obs['scan'], dtype=np.float64)
        if self.angles is None:
            self.angles = scan_angles(-math.radians(135), math.radians(270) / (len(scan) - 1), len(scan))
        state = np.asarray(agent_obs['std_state'], dtype=np.float64)
        try:
            enc, valid = obs.pool_scan(scan, SCAN_RANGE_MIN, SCAN_RANGE_MAX)
            self.history.push_tick(enc, valid, float(state[3]), self._imu(truth) if self.imu else None)
            o = obs.build_observation(self.history, imu_present=self.imu, mode=obs.MODE_REACTIVE, v_cap=self.v_cap)
        except obs.ObservationError:
            return 0.0, 0.0
        self.prev_state = np.array(truth, dtype=np.float64)
        out = self.policy.step(o)
        res = self.wrapper.apply(out.steering, out.speed, out.ego_path, scan, self.angles,
                                 SCAN_RANGE_MIN, SCAN_RANGE_MAX, self.prev_steer, 0.025)
        if res.stopped:
            self.history.reset()
            self.policy.reset()
            self.prev_steer = 0.0
            return 0.0, 0.0
        self.history.push_command(res.steering, res.speed)
        self.prev_steer = res.steering
        return res.steering, res.speed


class GapDriver:
    def __init__(self):
        self.controller = _rv().gap_controller()

    def command(self, agent_obs, truth, step):
        return self.controller.command(agent_obs['scan'], float(agent_obs['std_state'][3]), step)


class PursuitDriver:
    def __init__(self, plant):
        rv = _rv()
        self.rv = rv
        self.plan = rv.PathPlan.from_track(plant.env.unwrapped.track)
        self.follower = rv.PathFollower(self.plan)
        self.shaper = rv.CommandShaper()

    def command(self, agent_obs, truth, step):
        steer, speed, _n, _t, error = self.follower.command(agent_obs['std_state'],
                                                            previous_steering=self.shaper.previous_steering)
        steer, speed, safety = self.rv.apply_fallback_safety(agent_obs['scan'], steer, speed)
        if error > 1.0:
            speed = 0.0
        cap = 1.0 if safety == 'avoid' else self.rv.PURE_MAX_SPEED
        return self.shaper.command(steer, speed, cap, measured_speed=agent_obs['std_state'][3])


@dataclass
class RaceResult:
    track: str
    seed: int
    names: list
    laps: list
    finish_time: list                 # s, None if not finished
    contact: list
    at_fault: list
    order: list = field(default_factory=list)       # names, first to last
    progress: list = field(default_factory=list)    # m along the line, on ground truth

    def as_dict(self):
        return vars(self)


def race(track: str, drivers: dict, seed: int, laps: int = 1, timeout_s: float = 120.0) -> RaceResult:
    """drivers: {name: factory(plant) -> driver}. Grid start in the given order."""
    rv = _rv()
    names = list(drivers)
    n = len(names)
    plant = rv.make_plant(track_map_name(track), n, seed)
    line = plant.env.unwrapped.track.raceline
    xy = np.column_stack((line.xs, line.ys))
    # closed lines may repeat their first point at the end; a duplicate makes
    # a parked car's projection jump 0 -> N-1 and read as moving backwards
    while len(xy) > 3 and np.hypot(*(xy[-1] - xy[0])) < 1e-6:
        xy = xy[:-1]
    spacing = float(np.median(np.hypot(*np.diff(xy, axis=0).T)))
    poses = np.array([rv.initial_pose(line, -int(round(k * GRID_STAGGER / spacing)) % len(xy)) for k in range(n)])
    obs_, _ = plant.reset(poses)
    if any(obs_[f'agent_{i}']['collision'] for i in range(n)):
        plant.close()
        raise RuntimeError(f'{track}: cars overlap at the grid start; the race would be void')
    cars = [drivers[nm](plant) for nm in names]
    racing_math = rv.racing_math
    progress = [0.0] * n
    last_idx = [None] * n
    finish = [None] * n
    contact = [False] * n
    fault = [False] * n
    stopped = [False] * n
    steps = int(math.ceil(timeout_s / 0.025))
    info = {'lap_counts': np.zeros(n)}
    try:
        for step in range(steps):
            cmds = []
            truths = [np.asarray(plant.truth(i), dtype=np.float64) for i in range(n)]
            for i, car in enumerate(cars):
                if stopped[i] or finish[i] is not None:
                    cmds.append((0.0, 0.0))
                    continue
                cmds.append(car.command(obs_[f'agent_{i}'], truths[i], step))
            obs_, _r, done, _t, info = plant.step(cmds)
            for i in range(n):
                idx, _ = racing_math.find_nearest_index(xy, plant.truth(i)[:2], closed=True,
                                                        prev_index=last_idx[i], search_window=80)
                if last_idx[i] is not None:
                    d = (idx - last_idx[i] + len(xy) // 2) % len(xy) - len(xy) // 2
                    progress[i] += d * spacing
                last_idx[i] = idx
                if not stopped[i] and obs_[f'agent_{i}']['collision']:
                    stopped[i] = contact[i] = True
                    fault[i] = _at_fault(i, truths, n)
                if finish[i] is None and float(info['lap_counts'][i]) >= laps:
                    finish[i] = float(info['sim_time'])
            if all(stopped[i] or finish[i] is not None for i in range(n)):
                break
    finally:
        plant.close()
    lap_counts = [float(v) for v in info['lap_counts']]
    # finishers by time, then everyone else by distance covered on ground truth
    key = lambda i: (0, finish[i]) if finish[i] is not None else (1, -progress[i])   # noqa: E731
    order = [names[i] for i in sorted(range(n), key=key)]
    return RaceResult(track, seed, names, lap_counts, finish, contact, fault, order,
                      [round(p, 3) for p in progress])


def _at_fault(i, truths, n):
    """Closing-speed fault (plan A8), from the referee's own ground truth. A
    car alone in contact (a wall) is at fault."""
    s = truths[i]
    best = None
    for j in range(n):
        if j == i:
            continue
        o = truths[j]
        d = np.hypot(o[0] - s[0], o[1] - s[1])
        if best is None or d < best[0]:
            best = (d, j)
    if best is None or best[0] > 0.8:
        return True
    o = truths[best[1]]
    nx, ny = (o[0] - s[0]) / max(best[0], 1e-9), (o[1] - s[1]) / max(best[0], 1e-9)
    vi = (s[3] * math.cos(s[4] + s[6]), s[3] * math.sin(s[4] + s[6]))
    vj = (o[3] * math.cos(o[4] + o[6]), o[3] * math.sin(o[4] + o[6]))
    return max(0.0, vi[0] * nx + vi[1] * ny) > max(0.0, -(vj[0] * nx + vj[1] * ny))


def neural_factory(weights, card=None, **kw):
    return lambda plant: NeuralDriver(weights, card, **kw)


def gap_factory():
    return lambda plant: GapDriver()


def pursuit_factory():
    return lambda plant: PursuitDriver(plant)
