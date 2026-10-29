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

Two cars (``--cars 2``) is a head-to-head. ``--car-set N:name=value`` gives
car N (0-based) its own gap_follow parameters, applied after ``--set``. Both
cars run at their own top speed unless ``--speeds`` says otherwise, so the
comparison is about the parameters. The output is then a comparison, not a
pass/fail, and the run exits 0: for each run the finish order, the gap between
the first two finishers, overtakes and contacts (see head_to_head), and in
aggregate the wins, the mean gap and the contacts per car. ``--swap-starts``
runs every seed twice with the start order swapped, so the starting position
cancels out.

For a two-car head-to-head, each car's recognition of the other is measured
too: whether the other car is visible to its scan, and whether a moving track
sits on it (leader_visible and is_recognised give the exact rules). The report
shows, per car, the delay of each visibility episode from the other car coming
into view to its first recognition, the median of those delays, and the share
of visible ticks recognised.

    python3 tools/f1tenth_sim/eval_gap_multi.py --track indoor_wide --cars 3 --seeds 4
    python3 tools/f1tenth_sim/eval_gap_multi.py --track asb_10000 --cars 2 --speeds 3.5,2.0
    python3 tools/f1tenth_sim/eval_gap_multi.py --track indoor_wide --cars 2 --car-set 1:max_lateral_accel=2.0 --swap-starts --seeds 2

Exit code 0 when every run passes; always 0 for a two-car head-to-head.
"""

from __future__ import annotations

import argparse
import json
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

from gap_follow import gap_logic  # noqa: E402
from gap_mirror import GapController  # noqa: E402
from recognition import (  # noqa: E402
    median_delay,
    recognised_fraction,
    recognition_counts,
    recognition_delays,
)
from sim_fidelity import CALIBRATION, PROFILES  # noqa: E402
from sim_fidelity.plant import DEFAULT_PROFILE, FidelityPlant, env_components  # noqa: E402

CONTROL_DT = eval_gap.CONTROL_DT
# An overtake is a change of the gap's sign that holds at least this long.
PERSIST_S = 0.5
STALL_SPEED_MPS = 0.05
STALL_MIN_S = 1.0
# Recognition: a moving track within this distance of the other car's centre
# recognises it, and a scan reading within it of the car's near side sees it.
RECOGNITION_TOLERANCE_M = 0.5
_TRUE_WORDS = ("1", "true", "yes", "on")
_FALSE_WORDS = ("0", "false", "no", "off")


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


def wrap_arc(delta: float, total: float) -> float:
    """``delta`` folded into [-total/2, total/2): the shorter way round a loop."""
    return (delta + total / 2.0) % total - total / 2.0


def start_poses(line, cars: int, spacing: float, reverse: bool, slots=None) -> np.ndarray:
    """Leader at the centreline's first point, the rest ``spacing`` metres
    behind each other along the direction of travel. ``slots[k]`` is the place
    in the line-up that car k starts in (0 is the leader's place, on the line
    at arc length 0); by default car k starts in place k."""
    xs, ys, yaws = np.asarray(line.xs), np.asarray(line.ys), np.asarray(line.yaws)
    seg = np.hypot(np.diff(xs, append=xs[0]), np.diff(ys, append=ys[0]))
    s = np.concatenate(([0.0], np.cumsum(seg)[:-1]))
    total = float(np.sum(seg))
    poses = []
    for slot in (range(cars) if slots is None else slots):
        # Behind = smaller s when driving forward, larger s when reversed.
        target = (-slot * spacing if not reverse else slot * spacing) % total
        i = int(np.argmin(np.abs(s - target)))
        poses.append((xs[i], ys[i], yaws[i] + (math.pi if reverse else 0.0)))
    return np.array(poses, dtype=float)


class _Progress:
    """Laps driven along the centreline, from ground truth, in either
    direction. gym's own lap counter only counts the forward direction and
    only the first agent's start line, so it is not used for verdicts.

    ``distance`` is the travel since this car's own start. ``position`` is
    where the car is along the centreline in its direction of travel, unwrapped
    across laps and measured from ``origin_s`` (a centreline arc length; this
    car's own start when omitted), so two cars' positions compare directly.
    ``laps`` is the position in laps of the line the cars start on: a car that
    starts behind the line is below zero until it crosses it, so all cars
    finish at the same line, and a car behind it covers its offset as well.
    """

    def __init__(self, reference: np.ndarray, start_xy, reverse: bool, origin_s=None):
        self.ref = reference
        seg = np.hypot(*np.diff(np.vstack((reference, reference[:1])), axis=0).T)
        self.s = np.concatenate(([0.0], np.cumsum(seg)[:-1]))
        self.total = float(np.sum(seg))
        self.sign = -1.0 if reverse else 1.0
        self.idx = eval_gap.nearest_index(reference, np.asarray(start_xy), None)
        self.start_s = float(self.s[self.idx])
        start_from = self.start_s if origin_s is None else float(origin_s)
        self.offset = self.sign * wrap_arc(self.start_s - start_from, self.total)
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
        return self.position / self.total

    @property
    def position(self) -> float:
        return self.offset + self.distance


def line_up_progress(reference: np.ndarray, poses, reverse: bool) -> list:
    """One _Progress per car, all measured from the line: arc length 0 of the
    centreline, where start_poses puts the place at the front of the line-up.
    A car placed behind it starts at a negative position."""
    return [_Progress(reference, pose[:2], reverse, origin_s=0.0) for pose in poses]


def in_base_link(car, point, rear_offset: float) -> tuple:
    """``point`` (x, y) in the base_link frame of ``car`` = (x, y, yaw).

    ``car`` is a gym state, which is the car's centre of gravity. base_link is
    the rear axle, ``rear_offset`` metres behind it along the heading. It is the
    frame OpponentAwareness.moving_body() is expressed in, since its dead
    reckoning is the rear axle's. The axes follow the car: +x ahead, +y left.
    """
    x, y, yaw = car
    ox = x - rear_offset * math.cos(yaw)
    oy = y - rear_offset * math.sin(yaw)
    dx, dy = point[0] - ox, point[1] - oy
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def leader_visible(leader_base, lidar_x: float, ranges, valid, angle_min: float,
                   angle_increment: float, max_range: float, leader_half_length: float,
                   tolerance: float = RECOGNITION_TOLERANCE_M,
                   leader_half_width=None, leader_yaw: float = 0.0) -> bool:
    """Whether a follower's scan sees the other car on this tick.

    ``leader_base`` is the other car's centre in the follower's base_link.
    ``lidar_x`` is where the scan starts along base_link x, the simulator's
    real sensor position. ``ranges`` and ``valid`` are the scan as the
    controller gets it, after gap_logic.sanitize_ranges.

    Visible when all four hold:

    1. the centre is within ``max_range`` of base_link (opponent_max_range, the
       same gate moving_body() applies);
    2. the centre's bearing from the sensor is inside the scan's field of view;
    3. the beam nearest that bearing is valid and reads within ``tolerance``
       of the near side of a rectangle centred on the truth position, oriented
       at ``leader_yaw`` relative to base_link. The centre ray intersects the
       rectangle at distance min(half_length/|cos|, half_width/|sin|) before
       its centre. If no width is supplied, use a circle of half_length radius;
    4. the near side lies in front of the sensor.

    Returns a bool. An empty scan sees nothing.
    """
    if len(ranges) == 0:
        return False
    bx, by = leader_base
    if math.hypot(bx, by) > max_range:
        return False
    qx, qy = bx - lidar_x, by
    bearing = math.atan2(qy, qx)
    last = angle_min + (len(ranges) - 1) * angle_increment
    if not angle_min <= bearing <= last:
        return False
    index = round((bearing - angle_min) / angle_increment)
    if not valid[index]:
        return False
    if leader_half_width is None:
        radius = leader_half_length
    else:
        relative = bearing - leader_yaw
        along, across = abs(math.cos(relative)), abs(math.sin(relative))
        radius = min(leader_half_length / along if along else math.inf,
                     leader_half_width / across if across else math.inf)
    near_side = math.hypot(qx, qy) - radius
    if near_side <= 0.0:
        return False
    return bool(abs(ranges[index] - near_side) <= tolerance)


def is_recognised(moving_body, leader_base, tolerance: float = RECOGNITION_TOLERANCE_M) -> bool:
    """Whether some moving track lies within ``tolerance`` of the other car's centre.

    ``moving_body`` is OpponentAwareness.moving_body(), whose entries begin with
    base_link x and y, and ``leader_base`` is the other car's centre in that frame.
    """
    lx, ly = leader_base
    return any(math.hypot(entry[0] - lx, entry[1] - ly) <= tolerance for entry in moving_body)


def run_multi(track: str, params_list: list, seed: int, duration_s: float,
              spacing: float = 3.0, laps_required: float = 2.0, reverse: bool = False,
              slots=None) -> dict:
    """One run of every car. The result's ``cars`` entries carry each car's
    stats, ``progress_m`` (its position along the centreline every tick, see
    _Progress.position), ``speed_mps`` (ground-truth speed magnitude in m/s)
    and the contact flags; ``slots`` is the line-up the
    cars started in and ``dt_s`` the tick, for head_to_head. With two cars each
    car also gets ``visible`` and ``recognised``, one bool per tick: whether it
    can see the other car, and whether it recognises it (see leader_visible
    and is_recognised)."""
    cars = len(params_list)
    slots = tuple(range(cars)) if slots is None else tuple(slots)
    if sorted(slots) != list(range(cars)):
        raise ValueError(f"slots must be a line-up of 0..{cars - 1}, got {slots}")
    plant = make_plant(track, seed, cars)
    line = plant.env.unwrapped.track.raceline
    poses = start_poses(line, cars, spacing, reverse, slots)
    obs, _ = plant.reset(poses)
    lid = type("Lidar", (), {})()
    lid.angle_min = float(-eval_gap.components_half_fov())
    lid.angle_increment = float(eval_gap.components_increment())
    lid.range_min = float(eval_gap.components_range_min())
    ctrls = [GapController(p, lid) for p in params_list]
    reference = np.column_stack((line.xs, line.ys))
    progress = line_up_progress(reference, poses, reverse)
    stats = [dict(stop_ticks=0, ttc_ticks=0, speed_sum=0.0, ticks=0, braking_ticks=0,
                  wall_contact=False, car_contact=False, laps=0.0, finish_time_s=None,
                  opponent_ticks=0, follow_ticks=0, progress_m=[], speed_mps=[],
                  visible=[], recognised=[])
             for _ in range(cars)]
    # Recognition is a head-to-head measure: each car against the other one.
    recognition_on = cars == 2
    # Where the scans start, in the follower's base_link. The gym state is the centre
    # of gravity, lr ahead of base_link (the rear axle that the controller's dead
    # reckoning uses). The simulator then puts the LiDAR lidar_offset_x ahead of the
    # state itself (gym's _lidar_pose_from_base), so the sensor sits lr + offset ahead
    # of base_link, not at the controller's laser_offset_x. See leader_visible.
    rear = float(plant.params.lr)
    lidar_x_base = rear + float(plant.calibration.lidar_offset_x)
    half_length = float(plant.calibration.body_length) / 2.0
    half_width = float(plant.calibration.body_width) / 2.0
    min_gap = math.inf
    last_cmd = [0.0] * cars
    steps = int(math.ceil(duration_s / CONTROL_DT))
    for step in range(steps):
        # Current ground truth, before the step moves the cars. The car-profile
        # scan for the degraded agent can be delayed by the sensor model.
        truth_now = ([np.array(plant.truth(k), dtype=float) for k in range(cars)]
                     if recognition_on else None)
        commands = []
        for k, ctrl in enumerate(ctrls):
            ego = obs[f"agent_{k}"]
            state = np.asarray(ego["std_state"], dtype=float)
            # The gym's true yaw rate (std_state[5]). The mirror reads it only
            # with steering damping or the adaptive grip on, so with both off
            # this changes nothing.
            steer, speed = ctrl.command(ego["scan"], float(state[3]), step,
                                        yaw_rate=float(state[5]))
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
            if recognition_on:
                other = truth_now[1 - k]
                mine = truth_now[k]
                leader = in_base_link((mine[0], mine[1], mine[4]), (other[0], other[1]), rear)
                clean, valid = gap_logic.sanitize_ranges(
                    ego["scan"], max_range=params_list[k]["max_range"], range_min=lid.range_min)
                st["visible"].append(leader_visible(
                    leader, lidar_x_base, clean, valid, lid.angle_min, lid.angle_increment,
                    params_list[k]["opponent_max_range"], half_length,
                    leader_half_width=half_width, leader_yaw=other[4] - mine[4]))
                st["recognised"].append(
                    ctrl.opponents is not None
                    and is_recognised(ctrl.opponents.moving_body(), leader))
            commands.append((steer, speed))
        obs, _r, _done, _t, _info = plant.step(commands)
        truth = [plant.truth(k) for k in range(cars)]
        for a in range(cars):
            for b in range(a + 1, cars):
                min_gap = min(min_gap, math.hypot(truth[a][0] - truth[b][0], truth[a][1] - truth[b][1]))
        laps_now = []
        for k in range(cars):
            laps_now.append(progress[k].update(truth[k][:2]))
            stats[k]["laps"] = round(laps_now[k], 3)
            stats[k]["progress_m"].append(float(progress[k].position))
            stats[k]["speed_mps"].append(abs(float(truth[k][3])))
        # Contact is sim_fidelity's own real-chassis check, run on every tick.
        # gym's collision flag is not read.
        pair = plant._chassis_contact()
        walls = plant._wall_contact()
        for k in range(cars):
            stats[k]["car_contact"] |= k in pair
            stats[k]["wall_contact"] |= k in walls
        if pair or walls:
            break
        for k in range(cars):
            if stats[k]["finish_time_s"] is None and laps_now[k] >= laps_required:
                stats[k]["finish_time_s"] = round((step + 1) * CONTROL_DT, 2)
        if all(s["finish_time_s"] is not None for s in stats):
            break
    plant.close()
    for k, s in enumerate(stats):
        s["mean_speed"] = round(s.pop("speed_sum") / max(1, s["ticks"]), 3)
        s["max_speed_param"] = params_list[k]["max_speed"]
    contact = any(s["wall_contact"] or s["car_contact"] for s in stats)
    finished = all(s["finish_time_s"] is not None for s in stats)
    return dict(track=track, seed=seed, reverse=reverse, slots=list(slots), dt_s=CONTROL_DT,
                cars=stats, passed=bool(finished and not contact), contact=contact,
                min_center_distance_m=round(min_gap, 3), sim_time_s=round((step + 1) * CONTROL_DT, 2))


def _solid_signs(gap, need: int) -> list:
    """The signs (+1 or -1) that ``gap`` holds for at least ``need`` ticks in a
    row, in the order they happen. A tick at exactly zero continues the run it
    is in. Consecutive solid runs of one sign appear once, so a flicker too
    short to be a run of its own leaves no trace."""
    runs = []  # [sign, ticks] for each maximal run of one sign
    for g in gap:
        # Not (g > 0) - (g < 0): numpy's comparisons are numpy booleans, which cannot be subtracted.
        sign = 1 if g > 0 else (-1 if g < 0 else 0)
        if sign == 0:
            if runs:
                runs[-1][1] += 1
            continue
        if runs and runs[-1][0] == sign:
            runs[-1][1] += 1
        else:
            runs.append([sign, 1])
    solid = []
    for sign, ticks in runs:
        if ticks >= need and (not solid or solid[-1] != sign):
            solid.append(sign)
    return solid


def count_overtakes(traces, dt_s: float, persist_s: float = PERSIST_S) -> list:
    """Per car, how many times it moved from behind to ahead of another car.

    ``traces[k][i]`` is car k's position along the centreline at tick i (see
    _Progress.position: metres, in the direction of travel, unwrapped across
    laps), each tick ``dt_s`` seconds long. For every pair of cars the sign of
    their gap is followed, and a change of sign counts only once the new sign
    has held for ``persist_s``. Side-by-side jitter, where the gap flickers
    across zero tick to tick, counts nothing; a real pass counts once.
    """
    lengths = {len(trace) for trace in traces}
    if len(lengths) > 1:
        raise ValueError(f"progress traces differ in length: {sorted(lengths)}")
    for car, trace in enumerate(traces):
        if not all(math.isfinite(x) for x in trace):
            raise ValueError(f"car {car}: non-finite progress along the centreline")
    need = max(1, math.ceil(persist_s / dt_s - 1e-9))
    counts = [0] * len(traces)
    for a in range(len(traces)):
        for b in range(a + 1, len(traces)):
            ahead = [y - x for x, y in zip(traces[a], traces[b])]  # > 0: b is ahead of a
            for after in _solid_signs(ahead, need)[1:]:
                # The car that has just moved from behind to ahead of the other.
                counts[b if after > 0 else a] += 1
    return counts


def count_stalls(speeds, dt_s: float, stop_speed: float = STALL_SPEED_MPS,
                 min_s: float = STALL_MIN_S) -> list:
    """Count per-car stalls in speed traces (m/s), sampled every ``dt_s`` seconds.

    A stall is a maximal run longer than ``min_s`` seconds with absolute speed
    below ``stop_speed`` while another car is at or above that speed. Either
    condition breaking ends the run; an open final run counts. Empty traces
    and a single car have no stalls. Unequal or non-finite traces raise ValueError.
    """
    lengths = {len(trace) for trace in speeds}
    if len(lengths) > 1:
        raise ValueError(f"speed traces differ in length: {sorted(lengths)}")
    for car, trace in enumerate(speeds):
        if not all(math.isfinite(x) for x in trace):
            raise ValueError(f"car {car}: non-finite speed")
    need = math.floor(min_s / dt_s + 1e-9) + 1
    counts = [0] * len(speeds)
    runs = [0] * len(speeds)
    for tick in zip(*speeds):
        moving = [abs(v) >= stop_speed for v in tick]
        for k in range(len(speeds)):
            if not moving[k] and any(moving):
                runs[k] += 1
                if runs[k] == need:
                    counts[k] += 1
            else:
                runs[k] = 0
    return counts


def pair_gap_changes(traces, finish_times, dt_s: float) -> list:
    """Return each pair's change in the initial trailer's time deficit, in seconds.

    ``traces[k]`` is unwrapped centreline progress in metres, one value per
    ``dt_s`` seconds; ``finish_times[k]`` is seconds or None if unfinished.
    The smaller initial position is the trailer (higher index on a tie). Its
    mean speed v is (last - first) / ((n - 1) * dt_s). The start gap is the
    initial distance deficit / v. The end gap is trailer finish time minus
    leader finish time if both finished, otherwise final distance deficit / v.
    Gap change is end minus start; negative means the trailer closed in.
    Pairs are ordered by indices a < b. Fewer than two ticks gives None gaps;
    nonpositive trailer speed gives infinite gaps. Unequal or non-finite traces
    raise ValueError.
    """
    lengths = {len(trace) for trace in traces}
    if len(lengths) > 1:
        raise ValueError(f"progress traces differ in length: {sorted(lengths)}")
    for car, trace in enumerate(traces):
        if not all(math.isfinite(x) for x in trace):
            raise ValueError(f"car {car}: non-finite progress along the centreline")
    changes = []
    for a in range(len(traces)):
        for b in range(a + 1, len(traces)):
            n = len(traces[a])
            trailer = a if n and traces[a][0] < traces[b][0] else b
            leader = b if trailer == a else a
            start = end = change = None
            if n >= 2:
                speed = (traces[trailer][-1] - traces[trailer][0]) / ((n - 1) * dt_s)
                if speed <= 0:
                    start = end = change = math.inf
                else:
                    start = (traces[leader][0] - traces[trailer][0]) / speed
                    if finish_times[trailer] is not None and finish_times[leader] is not None:
                        end = finish_times[trailer] - finish_times[leader]
                    else:
                        end = (traces[leader][-1] - traces[trailer][-1]) / speed
                    change = end - start
            changes.append(dict(pair=(a, b), leader=leader, trailer=trailer,
                                start_gap_s=start, end_gap_s=end, gap_change_s=change))
    return changes


def time_gained(pair_changes, cars: int) -> list:
    """Per-car seconds gained: trailer gets minus the gap change, leader plus it.

    Undefined (None) pair changes contribute nothing; infinite changes retain
    their sign. A car's score sums its contributions against every other car.
    """
    gained = [0.0] * cars
    for pair in pair_changes:
        change = pair["gap_change_s"]
        if change is not None:
            gained[pair["leader"]] += change
            gained[pair["trailer"]] -= change
    return gained


def _recognition_of(car: dict, dt_s: float) -> dict:
    """One car's recognition of the other car, from its per-tick traces."""
    visible = car.get("visible", [])
    recognised = car.get("recognised", [])
    counts = recognition_counts(visible, recognised)
    delays = recognition_delays(visible, recognised, dt_s)
    return dict(delays=delays, median_s=median_delay(delays),
                visible_ticks=counts["visible_ticks"],
                recognised_ticks=counts["recognised_ticks"],
                fraction=recognised_fraction(counts["visible_ticks"], counts["recognised_ticks"]))


def head_to_head(run_result: dict) -> dict:
    """The verdict of one run, from run_multi's result, for any number of cars.

    ``finish_order``: car indices -- the finishers by the time they cross the
    line after the laps (a tie goes to the lower index), then the unfinished
    cars, furthest round first. ``gap_s``: time between the first two finishers, ``inf`` with fewer
    than two. ``overtakes``: per car, how many times it moved from behind to
    ahead of another car (count_overtakes). ``contacts``: per car, whether
    sim_fidelity's chassis check found it touching a wall or another car.
    ``winner``: the first finisher with no contact, or None. ``gap_changes``:
    pairwise deficit changes in seconds; ``time_gained_s``: per-car seconds
    gained; ``stalls``: per-car counts from the ground-truth speed traces.
    ``recognition``: per car, the delay of each visibility episode of the other
    car (``delays``, inf if never recognised), its ``median_s``, the visible and
    recognised tick counts, and the ``fraction`` of visible ticks recognised.
    """
    cars = run_result["cars"]
    times = [c["finish_time_s"] for c in cars]
    finished = sorted((k for k in range(len(cars)) if times[k] is not None),
                      key=lambda k: (times[k], k))
    unfinished = sorted((k for k in range(len(cars)) if times[k] is None),
                        key=lambda k: (-cars[k]["laps"], k))
    contacts = [bool(c["car_contact"] or c["wall_contact"]) for c in cars]
    winner = next((k for k in finished if not contacts[k]), None)
    gap_s = times[finished[1]] - times[finished[0]] if len(finished) >= 2 else math.inf
    gap_changes = pair_gap_changes([c["progress_m"] for c in cars], times, run_result["dt_s"])
    return dict(
        finish_order=finished + unfinished,
        gap_s=gap_s,
        overtakes=count_overtakes([c["progress_m"] for c in cars], run_result["dt_s"]),
        gap_changes=gap_changes,
        time_gained_s=time_gained(gap_changes, len(cars)),
        stalls=count_stalls([c["speed_mps"] for c in cars], run_result["dt_s"]),
        contacts=contacts,
        winner=winner,
        recognition=[_recognition_of(c, run_result["dt_s"]) for c in cars],
    )


def aggregate_head_to_head(outcomes: list) -> dict:
    """Wins, the mean gap, overtakes and contacts over head_to_head() results.
    The mean gap leaves out runs with fewer than two finishers. Mean time gained
    uses only runs where every car's gain is finite, or None if there are none.
    Recognition is pooled over the runs: every car's episode delays and its
    visible and recognised ticks. Older outcomes without the new fields contribute
    no stalls, gain or recognition runs."""
    n = len(outcomes[0]["contacts"]) if outcomes else 0
    agg = dict(runs=len(outcomes), wins=[0] * n, no_winner=0,
               overtakes=[0] * n, contact_runs=[0] * n, stalls=[0] * n)
    gaps = []
    gains = []
    pooled_delays = [[] for _ in range(n)]
    visible_total = [0] * n
    recognised_total = [0] * n
    for out in outcomes:
        if out["winner"] is None:
            agg["no_winner"] += 1
        else:
            agg["wins"][out["winner"]] += 1
        if math.isfinite(out["gap_s"]):
            gaps.append(out["gap_s"])
        if "time_gained_s" in out and all(math.isfinite(x) for x in out["time_gained_s"]):
            gains.append(out["time_gained_s"])
        for k in range(n):
            agg["overtakes"][k] += out["overtakes"][k]
            agg["contact_runs"][k] += int(out["contacts"][k])
            agg["stalls"][k] += out.get("stalls", [0] * n)[k]
        for k, car in enumerate(out.get("recognition", [])):
            pooled_delays[k].extend(car["delays"])
            visible_total[k] += car["visible_ticks"]
            recognised_total[k] += car["recognised_ticks"]
    agg["gap_runs"] = len(gaps)
    agg["mean_gap_s"] = sum(gaps) / len(gaps) if gaps else None
    agg["time_gained_runs"] = len(gains)
    agg["mean_time_gained_s"] = [sum(g[k] for g in gains) / len(gains) if gains else None
                                 for k in range(n)]
    agg["recognition"] = [
        dict(delays=pooled_delays[k], median_s=median_delay(pooled_delays[k]),
             visible_ticks=visible_total[k], recognised_ticks=recognised_total[k],
             fraction=recognised_fraction(visible_total[k], recognised_total[k]))
        for k in range(n)]
    return agg


def _car_set_value(item: str, name: str, raw: str):
    """One --car-set value, typed the way gap_follow's YAML types it. A boolean
    that is not spelled out is refused, not read as false."""
    default = eval_gap.DEFAULTS[name]
    text = raw.strip()
    if isinstance(default, bool):
        if text.lower() in _TRUE_WORDS:
            return True
        if text.lower() in _FALSE_WORDS:
            return False
        raise SystemExit(f"--car-set {item!r}: {name} is true or false, not {raw!r}")
    if isinstance(default, str):
        return text
    try:
        number = float(text)
    except ValueError:
        raise SystemExit(f"--car-set {item!r}: {name} needs a number, not {raw!r}") from None
    if not math.isfinite(number):
        raise SystemExit(f"--car-set {item!r}: {name} must be finite, not {raw!r}")
    if isinstance(default, int) and not number.is_integer():
        raise SystemExit(f"--car-set {item!r}: {name} is a whole number, not {raw!r}")
    return type(default)(number)


def parse_car_set(items, cars: int) -> dict:
    """``--car-set N:name=value`` items as ``{car index: {name: value}}``.

    A bad item raises SystemExit naming it: an index that is not a whole number
    or is outside ``0..cars-1``, a missing ``:`` or ``=``, a parameter that is
    not gap_follow's (``enable_deadman`` is not mirrored, so it is not one), or
    a value that does not parse for that parameter. A later item for the same
    car and parameter wins, as it does for ``--set``.
    """
    overrides = {}
    for item in items or []:
        index_text, colon, assignment = item.partition(":")
        name, equals, raw = assignment.partition("=")
        if not colon or not equals:
            raise SystemExit(f"--car-set expects N:name=value, got {item!r}")
        try:
            index = int(index_text)
        except ValueError:
            raise SystemExit(f"--car-set {item!r}: car index {index_text!r} is not a whole number") from None
        if not 0 <= index < cars:
            raise SystemExit(f"--car-set {item!r}: car index {index} is outside 0..{cars - 1}")
        name = name.strip()
        if name not in eval_gap.DEFAULTS:
            raise SystemExit(f"--car-set {item!r}: unknown parameter {name!r}")
        overrides.setdefault(index, {})[name] = _car_set_value(item, name, raw)
    return overrides


def per_car_params(base: dict, car_sets: dict, cars: int, speeds=None) -> list:
    """Each car's gap_follow parameters: ``base`` (``--set``), then its own
    ``--car-set`` values, then its top speed.

    Two cars (a head-to-head) both run at their own max_speed, which is the same
    for both unless ``--car-set`` changes it. Otherwise the leader (car 0) runs
    at 0.7x so the others catch it. ``--speeds`` replaces both rules, with one
    value per car, and cannot be combined with a ``--car-set`` max_speed.
    """
    if speeds:
        values = [float(v) for v in speeds.split(",")]
        if len(values) != cars:
            raise SystemExit("--speeds needs one value per car")
        for k in sorted(car_sets):
            if "max_speed" in car_sets[k]:
                raise SystemExit(f"car {k}: max_speed is set by both --speeds and --car-set")
    else:
        top = base["max_speed"]
        rule = [top, top] if cars == 2 else [round(0.7 * top, 2)] + [top] * (cars - 1)
        values = [car_sets.get(k, {}).get("max_speed", rule[k]) for k in range(cars)]
    params = []
    for k, v in enumerate(values):
        p = dict(base)
        p.update(car_sets.get(k, {}))
        p["max_speed"] = v
        p["min_speed"] = min(p["min_speed"], v)
        p["corner_speed"] = min(p["corner_speed"], v)
        p["corner_speed_wide"] = min(p["corner_speed_wide"], v)
        params.append(p)
    return params


def _job(job):
    return run_multi(*job)


def _seconds(value) -> str:
    if value is None:
        return "n/a"
    return "inf" if math.isinf(value) else f"{value:.2f}s"


def _recognition_text(entry: dict, show_delays: bool) -> str:
    """One car's recognition of the other car, as a fragment of a report line."""
    if entry["fraction"] is None:
        return "no visible ticks"
    never = sum(math.isinf(d) for d in entry["delays"])
    text = (f"{len(entry['delays'])} episodes ({never} never recognised), "
            f"median {_seconds(entry['median_s'])}")
    if show_delays and entry["delays"]:
        text += ", delays " + " ".join(_seconds(d) for d in entry["delays"])
    return text + (f", recognised {entry['fraction']:.0%} of "
                   f"{entry['visible_ticks']} visible ticks")


def _report_head_to_head(track: str, runs: list, params_list: list) -> None:
    """Per-run table and aggregate for a two-car head-to-head."""
    differ = [key for key in params_list[0]
              if any(p[key] != params_list[0][key] for p in params_list)]
    print(f"  head-to-head on {track}: " + "; ".join(
        f"car {k}: " + " ".join(f"{key}={p[key]}" for key in differ)
        for k, p in enumerate(params_list)), flush=True)
    print(f"  {'seed':>6}  {'starts':<6}  {'finish order':<44}  {'gap':>6}  {'overtakes':>9}  "
          f"{'contact':<18}  {'winner':<6}  {'stalls':>9}  {'gained':>8}", flush=True)
    outcomes = []
    for run in runs:
        out = head_to_head(run)
        outcomes.append(out)
        cars = run["cars"]
        finish = " | ".join(
            f"car {k} {cars[k]['finish_time_s']:.2f}s" if cars[k]["finish_time_s"] is not None
            else f"car {k} unfinished at {cars[k]['laps']:.2f} laps"
            for k in out["finish_order"])
        gap = f"{out['gap_s']:.2f}s" if math.isfinite(out["gap_s"]) else "n/a"
        touched = []
        for k in range(len(cars)):
            kinds = [kind for kind, hit in (("car", cars[k]["car_contact"]), ("wall", cars[k]["wall_contact"]))
                     if hit]
            if kinds:
                touched.append(f"car {k} ({'+'.join(kinds)})")
        contact = ", ".join(touched) or "none"
        winner = f"car {out['winner']}" if out["winner"] is not None else "none"
        gain = out["time_gained_s"][0]
        gained = f"{gain:+.2f}s" if math.isfinite(gain) else ("inf" if math.isinf(gain) else "n/a")
        print(f"  {run['seed']:>6}  car {run['slots'].index(0):<2}  {finish:<44}  {gap:>6}  "
              f"{' / '.join(str(x) for x in out['overtakes']):>9}  {contact:<18}  {winner:<6}  "
              f"{' / '.join(str(x) for x in out['stalls']):>9}  {gained:>8}", flush=True)
        print("          recognition: " + "; ".join(
            f"car {k} {_recognition_text(entry, show_delays=True)}"
            for k, entry in enumerate(out["recognition"])), flush=True)
    agg = aggregate_head_to_head(outcomes)
    n = len(params_list)
    wins = ", ".join(f"car {k} {agg['wins'][k]}" for k in range(n))
    mean = (f"{agg['mean_gap_s']:.2f}s over {agg['gap_runs']} of {agg['runs']} runs with two finishers"
            if agg["mean_gap_s"] is not None else "n/a (no run had two finishers)")
    contacts = ", ".join(f"car {k} {agg['contact_runs'][k]}" for k in range(n))
    overtakes = ", ".join(f"car {k} {agg['overtakes'][k]}" for k in range(n))
    print(f"  aggregate, {agg['runs']} runs: wins {wins}; no winner {agg['no_winner']}", flush=True)
    print(f"  mean gap {mean}", flush=True)
    print(f"  runs with contact: {contacts}; overtakes: {overtakes}", flush=True)
    stalls = ", ".join(f"car {k} {agg['stalls'][k]}" for k in range(n))
    gained = ", ".join(f"car {k} {v:+.2f}s" if v is not None else f"car {k} n/a"
                       for k, v in enumerate(agg["mean_time_gained_s"]))
    print(f"  stalls: {stalls}; mean time gained: {gained} over {agg['time_gained_runs']} of {agg['runs']} runs",
          flush=True)
    print(f"  recognition, pooled over {agg['runs']} runs: " + "; ".join(
        f"car {k} {_recognition_text(entry, show_delays=True)}"
        for k, entry in enumerate(agg["recognition"])), flush=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", default="indoor_wide", help="as eval_gap: a name, group, or comma list")
    ap.add_argument("--cars", type=int, default=2)
    ap.add_argument("--speeds", help="top speed per car, comma-separated, car 0 first; default: each car's "
                                     "own max_speed for --cars 2, else 0.7x for car 0 (the leader) and "
                                     "max_speed for the rest")
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--first-seed", type=int, default=12345)
    ap.add_argument("--laps", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=90.0, help="simulated seconds per run")
    ap.add_argument("--spacing", type=float, default=3.0, help="metres between start positions")
    ap.add_argument("--reverse", action="store_true")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--output", help="write the per-run results as JSON")
    ap.add_argument("--set", action="append", default=[], help="override a parameter for every car")
    ap.add_argument("--car-set", action="append", default=[], metavar="N:name=value",
                    help="override a parameter for car N only (0-based), applied after --set; repeatable")
    ap.add_argument("--swap-starts", action="store_true",
                    help="two cars only: run every seed twice, once with each car starting in front")
    args = ap.parse_args(argv)
    if args.swap_starts and args.cars != 2:
        raise SystemExit("--swap-starts needs --cars 2")
    base = eval_gap.parse_set(args.set)
    params_list = per_car_params(base, parse_car_set(args.car_set, args.cars), args.cars, args.speeds)
    tracks = eval_gap.select_tracks(args.track)
    for t in tracks:
        eval_gap.track_path(t)
    orders = [None, (1, 0)] if args.swap_starts else [None]
    jobs = [(t, params_list, args.first_seed + k, args.duration, args.spacing, args.laps, args.reverse, order)
            for t in tracks for k in range(args.seeds) for order in orders]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(_job, jobs))
    if args.output:
        Path(args.output).write_text(json.dumps(results) + "\n")
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
        if args.cars == 2:
            _report_head_to_head(t, runs, params_list)
    if args.cars == 2:
        return 0  # a comparison, not a pass/fail
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
