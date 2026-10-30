"""Replay a run's IMUs and /odom through vehicle_state, scored against LiDAR.

The run's recorded /imu/{bno,vesc}/data and /odom go, in header-stamp
order, through the same imu_bridge.vehicle_state.VehicleStateEstimator and
GyroOdometry that vehicle_state_node runs -- with today's settings and with
any you override -- and two things are scored against LiDAR scan matching
(scan_motion, independent of every sensor the estimator uses):

- fused speed (/odom/fused) against LiDAR speed, next to the raw wheel speed;
- gyro-odometry heading (/odom/imu) against the LiDAR's heading change over
  short windows, next to /odom's own (steering-model) heading.

    python3 tools/run_analysis/estimator_replay.py ~/racerbot-ws/data/racerbot_runs/<ts>
    python3 tools/run_analysis/estimator_replay.py <run> --set accel_source=vesc --set sigma_wheel=0.1

``--set name=value`` takes vehicle_state.yaml's names (bno_x_offset,
heading_sources=bno,vesc, ...) and is validated by EstimatorConfig itself:
a value the node would refuse is refused here. "current" is EstimatorConfig's
defaults, which test_vehicle_state_config.py holds equal to the YAML.

Open-loop, like replay.py: the estimator's output changed nothing about how
the car was driven, so this scores the estimate, not what a controller using
it would have done. The replay clock is the header stamps (staleness
included), not the node's receive clock.

The first run needs ROS sourced to read the bag; arrays are cached in
<run>/analysis/ and the rest runs unsourced.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass, field, fields
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT / "tools", ROOT / "src" / "imu_bridge"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from imu_bridge.vehicle_state import (  # noqa: E402
    EstimatorConfig, GyroOdometry, VehicleStateEstimator)

# vehicle_state_node's flattening of EstimatorConfig's per-IMU tables
# (vehicle_state_node.py TABLE_PARAMS, NAME_LISTS).
TABLE_PARAMS = {"x_offset": ("bno_x_offset", "vesc_x_offset"),
                "gyro_weight": ("bno_gyro_weight", "vesc_gyro_weight")}
NAME_LISTS = ("heading_sources",)
STRING_FIELDS = ("accel_source",)
# Scoring: only where the LiDAR says the car moves, and heading over windows
# short enough that the LiDAR's own heading drift is negligible.
MOVING_SPEED = 0.3          # m/s
HEADING_WINDOW_S = 2.0


def settable_names() -> list:
    out = []
    for f in fields(EstimatorConfig):
        out += list(TABLE_PARAMS[f.name]) if f.name in TABLE_PARAMS else [f.name]
    return out


def build_config(overrides: dict) -> EstimatorConfig:
    """EstimatorConfig with overrides {yaml name: text}. Raises ValueError
    for an unknown name, a non-number, or anything EstimatorConfig refuses."""
    base = EstimatorConfig()
    kwargs = {f.name: getattr(base, f.name) for f in fields(EstimatorConfig)}
    tables = {name: dict(kwargs[name]) for name in TABLE_PARAMS}
    for name, text in overrides.items():
        table = next((t for t, flat in TABLE_PARAMS.items() if name in flat), None)
        try:
            if table is not None:
                tables[table][name.split("_", 1)[0]] = float(text)
            elif name in NAME_LISTS:
                kwargs[name] = tuple(s.strip() for s in str(text).split(",") if s.strip())
            elif name in STRING_FIELDS:
                kwargs[name] = str(text)
            elif name in kwargs:
                kwargs[name] = float(text)
            else:
                raise ValueError(f"unknown parameter {name!r}; settable: {', '.join(settable_names())}")
        except (TypeError, ValueError) as exc:
            if "unknown parameter" in str(exc):
                raise
            raise ValueError(f"{name}={text!r}: {exc}") from exc
    for name, table in tables.items():
        kwargs[name] = tuple(table.items())
    return EstimatorConfig(**kwargs)


@dataclass
class ReplayResult:
    t: np.ndarray                 # stamp of each /odom update
    wheel: np.ndarray             # /odom speed
    fused: np.ndarray             # /odom/fused speed
    modes: Counter = field(default_factory=Counter)
    pose_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    gyro_yaw: np.ndarray = field(default_factory=lambda: np.zeros(0))    # /odom/imu heading
    odom_yaw: np.ndarray = field(default_factory=lambda: np.zeros(0))    # /odom heading
    heading_sources: Counter = field(default_factory=Counter)


def replay(streams: dict, odom: np.ndarray, cfg: EstimatorConfig) -> ReplayResult:
    """streams: {'bno'|'vesc': (stamp, accel_x, accel_z, yaw_rate)} in car
    axes, SI; odom: rows (t_recv, stamp, vx, wz, x, y, yaw). Everything is
    fed in stamp order, IMU samples before an /odom with the same stamp."""
    events = []
    for name, (stamp, ax, az, gz) in streams.items():
        events += [(float(s), 0, name, float(a), float(z), float(g))
                   for s, a, z, g in zip(stamp, ax, az, gz)]
    events += [(float(r[1]), 1, "odom", float(r[2]), float(r[4]), float(r[5]), float(r[6]))
               for r in odom]
    events.sort(key=lambda e: (e[0], e[1]))
    est = VehicleStateEstimator(cfg)
    odo = GyroOdometry(est)
    t, wheel, fused, modes = [], [], [], Counter()
    pose_t, gyro_yaw, odom_yaw, sources = [], [], [], Counter()
    for ev in events:
        stamp = ev[0]
        if ev[1] == 0:
            _, _, name, ax, az, gz = ev
            est.on_imu(name, stamp, stamp, ax, az, gz)
            continue
        _, _, _, vx, x, y, yaw = ev
        e = est.on_wheel(stamp, vx)
        t.append(stamp)
        wheel.append(vx)
        fused.append(e.speed)
        modes[e.mode] += 1
        step = odo.on_wheel_pose(stamp, stamp, x, y, yaw)
        if step is not None:
            pose_t.append(stamp)
            gyro_yaw.append(step.pose[2])
            odom_yaw.append(yaw)
            sources[step.source] += 1
    return ReplayResult(np.array(t), np.array(wheel), np.array(fused), modes,
                        np.array(pose_t), np.array(gyro_yaw), np.unwrap(np.array(odom_yaw)),
                        sources)


def score(res: ReplayResult, motion, window_s: float = HEADING_WINDOW_S) -> dict:
    """RMS errors against LiDAR. Speed: the latest estimate at each LiDAR
    sample while moving.
    Heading: the change over consecutive windows of window_s while moving,
    LiDAR's being the integral of its yaw rate. NaN where nothing overlaps."""
    out = dict(speed_fused=math.nan, speed_wheel=math.nan, n_speed=0,
               heading_gyro=math.nan, heading_odom=math.nan, n_windows=0)
    if motion is None or len(motion.t) < 2 or len(res.t) < 2:
        return out
    inside = (motion.t >= res.t[0]) & (motion.t <= res.t[-1]) & (np.abs(motion.v) > MOVING_SPEED)
    if inside.any():
        # The value a consumer had at each LiDAR time: the latest message,
        # not an interpolation, which would average two readings' noise and
        # flatter both estimates.
        j = np.searchsorted(res.t, motion.t[inside], side="right") - 1
        out["speed_fused"] = float(np.sqrt(np.mean((res.fused[j] - motion.v[inside]) ** 2)))
        out["speed_wheel"] = float(np.sqrt(np.mean((res.wheel[j] - motion.v[inside]) ** 2)))
        out["n_speed"] = int(inside.sum())
    if len(res.pose_t) < 2:
        return out
    lidar_yaw = np.concatenate([[0.0], np.cumsum(0.5 * (motion.w[1:] + motion.w[:-1])
                                                 * np.diff(motion.t))])
    t0 = max(motion.t[0], res.pose_t[0])
    t1 = min(motion.t[-1], res.pose_t[-1])
    edges = np.arange(t0, t1 - window_s + 1e-9, window_s)
    errs_g, errs_o = [], []
    for a in edges:
        b = a + window_s
        sel = (motion.t >= a) & (motion.t <= b)
        if sel.sum() < 2 or np.mean(np.abs(motion.v[sel])) <= MOVING_SPEED:
            continue
        d_lidar = np.interp(b, motion.t, lidar_yaw) - np.interp(a, motion.t, lidar_yaw)
        d_gyro = np.interp(b, res.pose_t, res.gyro_yaw) - np.interp(a, res.pose_t, res.gyro_yaw)
        d_odom = np.interp(b, res.pose_t, res.odom_yaw) - np.interp(a, res.pose_t, res.odom_yaw)
        errs_g.append(d_gyro - d_lidar)
        errs_o.append(d_odom - d_lidar)
    if errs_g:
        out["heading_gyro"] = float(np.sqrt(np.mean(np.square(errs_g))))
        out["heading_odom"] = float(np.sqrt(np.mean(np.square(errs_o))))
        out["n_windows"] = len(errs_g)
    return out


def streams_from_axes(axes: dict, run_dir=None, car_yaml=None):
    """({'bno'|'vesc': (stamp, ax, az, gz)}, notes). The VESC comes through
    imu_spectrum.vesc_stream, so a run without /imu/vesc/data falls back to
    the raw topic rotated with the run's car.yaml (or car_yaml, when the run
    has no copy) -- and is left out, with a note, if its axes cannot be known."""
    from run_analysis.imu_spectrum import bno_stream, vesc_stream
    streams, notes = {}, []
    for s in (bno_stream(axes), vesc_stream(axes, run_dir, car_yaml)):
        if s is None:
            continue
        if not s.axes_known:
            notes.append(f"{s.name}: left out ({s.label})")
            continue
        streams[s.name] = (s.stamp, s.acc[:, 0], s.acc[:, 2], s.gyro[:, 2])
        notes.append(f"{s.name}: {s.label}")
    if "bno" not in streams:
        notes.append("bno: not in this run")
    return streams, notes


def report(axes: dict, motion, overrides: dict, run_dir=None, car_yaml=None) -> list:
    streams, notes = streams_from_axes(axes, run_dir, car_yaml)
    odom = axes.get("odom", np.zeros((0, 7)))
    lines = ["# Estimator replay (vehicle_state, open loop) scored against LiDAR", ""]
    lines += [f"  {n}" for n in notes]
    if len(odom) < 2:
        return lines + ["  no /odom in this run: nothing to replay"]
    configs = [("current", EstimatorConfig())]
    if overrides:
        configs.append(("proposed " + " ".join(f"{k}={v}" for k, v in overrides.items()),
                        build_config(overrides)))
    lines += ["", f"  {'':40s} {'speed RMS':>12s} {'heading RMS':>12s}"]
    for name, cfg in configs:
        res = replay(streams, odom, cfg)
        s = score(res, motion)
        heading = math.degrees(s['heading_gyro'])
        lines.append(f"  {name[:40]:40s} {s['speed_fused']:9.3f} m/s {heading:9.2f} deg"
                     f"   modes {dict(res.modes)}, heading from {dict(res.heading_sources)}")
    if configs:
        s = score(replay(streams, odom, configs[0][1]), motion)
        lines.append(f"  {'reference: raw /odom':40s} {s['speed_wheel']:9.3f} m/s "
                     f"{math.degrees(s['heading_odom']):9.2f} deg")
        lines.append(f"  ({s['n_speed']} LiDAR speed samples while moving; heading over "
                     f"{s['n_windows']} windows of {HEADING_WINDOW_S:.0f} s)")
    if motion is None:
        lines.append("  no LiDAR motion (--no-icp, or no /scan): nothing to score against")
    return lines


def main(argv=None) -> int:
    from run_analysis.imu_report import load_imu_axes
    from run_analysis.runlog import analysis_dir, load_run
    from run_analysis.scan_motion import scan_motion

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run_dir")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="vehicle_state.yaml parameter to change (repeatable)")
    ap.add_argument("--no-icp", action="store_true", help="skip LiDAR scan matching (no scores)")
    ap.add_argument("--refresh", action="store_true", help="re-read the bag")
    ap.add_argument("--car-yaml", help="imu-calibration car.yaml for a run that has no copy in "
                                       "its metadata (e.g. ~/imu-calibration/config/car.yaml)")
    args = ap.parse_args(argv)
    overrides = {}
    for item in args.set:
        name, sep, value = item.partition("=")
        if not sep:
            ap.error(f"--set {item!r}: expected NAME=VALUE")
        overrides[name.strip()] = value.strip()
    try:
        build_config(overrides)
    except ValueError as exc:
        ap.error(str(exc))
    axes = load_imu_axes(args.run_dir, refresh=args.refresh)
    motion = None
    if not args.no_icp:
        run = load_run(args.run_dir, refresh=args.refresh)
        motion = scan_motion(run, float(run.scan_t[0]), float(run.scan_t[-1]))
    text = "\n".join(report(axes, motion, overrides, Path(args.run_dir).expanduser(),
                            args.car_yaml)) + "\n"
    (analysis_dir(args.run_dir) / "estimator_replay.txt").write_text(text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
