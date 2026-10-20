"""Replay a run's real scans through the simulator's copy of gap_follow.

tools/f1tenth_sim/gap_mirror.GapController is the node's per-scan pipeline
without ROS, held to the node by src/gap_follow/test/test_gap_mirror_parity.py.
Feeding it the bag's scans, with the odom speed the node had at each scan,
should reproduce the /drive the node published -- ``parity`` says how
closely. Once it does, the replay can say *why* each command was what it was
(every speed cap, the TTC beam, the side walls), and can be re-run with
parameter overrides to ask what a change would have done on these exact
scans.

That counterfactual is open-loop: the scans are the ones the real car saw,
so the replay cannot show where a changed car would have gone. Use it to
count which brakes a change removes, and the closed-loop simulator to judge
whether removing them is safe.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for p in (ROOT / "tools" / "f1tenth_sim", ROOT / "src" / "gap_follow"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import gap_mirror  # noqa: E402
from gap_follow import gap_logic  # noqa: E402

DT = gap_mirror.CONTROL_DT


class _Lidar:
    def __init__(self, run):
        self.angle_min = run.angle_min
        self.angle_increment = run.angle_increment
        self.range_min = run.range_min


@dataclass
class Tick:
    t: float              # seconds since t0
    scan_index: int
    odom_speed: float
    steering: float       # replayed command
    speed: float
    real_steering: float  # what the node published after this scan (NaN if none)
    real_speed: float
    stopped: bool
    stop_reason: str
    binding: str
    telemetry: dict


def replay(run, t0: float, t1: float, overrides: dict | None = None,
           params: dict | None = None) -> list:
    """One Tick per scan received in (t0, t1); t0 should be an LB press."""
    p = dict(params if params is not None else gap_mirror.node_params())
    p.update(overrides or {})
    ctrl = gap_mirror.GapController(p, _Lidar(run))
    idx = np.where((run.scan_t > t0) & (run.scan_t < t1))[0]
    if idx.size == 0:
        return []
    ctrl.last_command_time = (run.scan_t[idx[0]] - t0) / DT - 1.0
    drive_t = run.drive[:, 0]
    ticks = []
    for n, i in enumerate(idx):
        t = run.scan_t[i]
        v = run.odom_speed_at(t)
        steer, speed = ctrl.command(run.scan[i].astype(np.float64), v, (t - t0) / DT)
        nxt = run.scan_t[idx[n + 1]] if n + 1 < len(idx) else t + 0.05
        j = int(np.searchsorted(drive_t, t, side="right"))
        real = run.drive[j, 2:4] if j < len(drive_t) and drive_t[j] < nxt else (math.nan, math.nan)
        stopped = ctrl.binding_cap == "stop"
        ticks.append(Tick(t - t0, int(i), v, steer, speed, float(real[0]), float(real[1]),
                          stopped, ctrl.stop_reason if stopped else "", ctrl.binding_cap or "",
                          dict(ctrl.telemetry)))
    return ticks


@dataclass
class Parity:
    ticks: int
    paired: int
    steering_rms: float
    steering_max: float
    speed_median_abs: float
    stops_replay: int
    stops_real: int
    stop_ticks_only_replay: list
    stop_ticks_only_real: list

    def lines(self) -> list:
        return [
            f"replayed {self.ticks} ticks, {self.paired} paired with a published /drive",
            f"steering error RMS {self.steering_rms:.4f} rad, max {self.steering_max:.4f} rad",
            f"speed error median {self.speed_median_abs:.4f} m/s",
            f"zero-speed ticks: replay {self.stops_replay}, real {self.stops_real}; "
            f"only in replay at {self.stop_ticks_only_replay[:10]}, "
            f"only in real at {self.stop_ticks_only_real[:10]}",
        ]


def parity(ticks: list) -> Parity:
    paired = [k for k in ticks if math.isfinite(k.real_speed)]
    s = np.array([k.steering - k.real_steering for k in paired])
    v = np.array([k.speed - k.real_speed for k in paired])
    rep = {round(k.t, 3) for k in paired if k.speed == 0.0}
    real = {round(k.t, 3) for k in paired if k.real_speed == 0.0}
    return Parity(len(ticks), len(paired),
                  float(np.sqrt(np.mean(s ** 2))) if s.size else math.nan,
                  float(np.max(np.abs(s))) if s.size else math.nan,
                  float(np.median(np.abs(v))) if v.size else math.nan,
                  len(rep), len(real), sorted(rep - real), sorted(real - rep))


def stop_clusters(ticks: list, gap_s: float = 0.4) -> list:
    """Consecutive stop ticks grouped into events: [[tick, ...], ...]."""
    clusters = []
    for k in ticks:
        if not k.stopped:
            continue
        if clusters and k.t - clusters[-1][-1].t < gap_s:
            clusters[-1].append(k)
        else:
            clusters.append([k])
    return clusters


def explain_ttc(run, tick: Tick, steering_basis: float, params: dict | None = None) -> dict:
    """Geometry of the beam that set the minimum TTC on ``tick``, and whether
    the body's own swept path along the same arc was clear.

    ``steering_basis`` is the rack position the node used (the previous
    tick's commanded steering).
    """
    p = params or gap_mirror.node_params()
    clean, valid = gap_logic.sanitize_ranges(run.scan[tick.scan_index].astype(np.float64),
                                             p["max_range"], run.range_min)
    angles = run.beam_angles()
    half = math.radians(p["forward_fov_deg"]) / 2.0
    lo = max(0, int((-half - run.angle_min) / run.angle_increment))
    hi = min(clean.size - 1, int((half - run.angle_min) / run.angle_increment))
    w, wv, ba = clean[lo:hi + 1], valid[lo:hi + 1], angles[lo:hi + 1]
    bd = gap_logic.vehicle_boundary_distances(ba, p["car_width"], p["car_length"], p["wheelbase"],
                                              p["laser_offset_x"], p["laser_offset_y"])
    kappa = math.tan(steering_basis) / p["wheelbase"]
    gate = p["car_width"] / 2.0 + p.get("ttc_path_margin", p["safety_margin"])
    ttc = gap_logic.time_to_collision(w, wv, ba, tick.odom_speed, bd, p["ttc_min_closing_speed"],
                                      gate, kappa, p["laser_offset_x"], p["laser_offset_y"])
    j = int(np.argmin(ttc))
    x = w[j] * math.cos(ba[j]) + p["laser_offset_x"]
    y = w[j] * math.sin(ba[j]) + p["laser_offset_y"]
    if abs(kappa) < 1e-6:
        offset = abs(y)
    else:
        r = 1.0 / kappa
        offset = abs(math.hypot(x, y - r) - abs(r))
    body_path = gap_logic.swept_path_clearance(
        w, wv, ba, kappa, p["car_width"] / 2.0, p["car_length"], p["wheelbase"],
        p["laser_offset_x"], p["laser_offset_y"], 5.0)
    return dict(ttc=float(ttc[j]), beam_deg=math.degrees(ba[j]), range=float(w[j]),
                x=x, y=y, offset_from_path=offset, gate_half_width=gate,
                beyond_body_edge=offset - p["car_width"] / 2.0,
                body_path_clear_m=body_path,
                stopping_distance_m=tick.odom_speed ** 2 / (2.0 * p["max_braking_decel"]))
