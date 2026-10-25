"""How the car really turned on one recorded run: gyros vs odometry vs LiDAR.

Car 1 has three independent measures of yaw rate, and /odom's is not one of
them: vesc_to_odom computes it from the *steering command*
(speed * tan(steer) / wheelbase), so it cannot show servo lag or understeer.

- the BNO085's gyro (/imu/bno/data) and the VESC's (/imu/vesc/data), via
  imu_bridge;
- LiDAR scan matching (scan_motion), slower to compute but independent of
  every IMU.

This report puts all of them side by side, measures the steering lag and
turning gain against the gyro (actuation.analyze, the same fit the LiDAR
gets), and shows how much heading correction SLAM had to apply on top of
/odom and on top of the gyro odometry /odom/imu over the run.

    python3 tools/run_analysis/imu_report.py ~/.ros/racerbot_runs/<ts>
    python3 tools/run_analysis/imu_report.py <run> --no-icp     # skip scan matching (faster)

The first run needs ROS sourced to read the bag; the arrays are cached in
<run>/analysis/imu.npz and the rest runs unsourced. Prints the report and
writes it to <run>/analysis/imu_report.txt.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from run_analysis import actuation  # noqa: E402
from run_analysis.scan_motion import Motion  # noqa: E402

IMU_CACHE = "imu.npz"
# topic -> array name. Pose topics: slam_toolbox's /pose
# (PoseWithCovarianceStamped) or auto_map_race's /slam_pose (PoseStamped).
GYRO_TOPICS = {"/imu/bno/data": "bno", "/imu/vesc/data": "vesc"}
POSE_TOPICS = {"/odom/imu": "odom_imu", "/pose": "slam", "/slam_pose": "slam_pose"}
MIN_SPEED = 0.3   # m/s: below this nothing turns enough to compare
# Steering fit: only samples above this speed. actuation.analyze's own
# default (1.0) would leave nothing of a 0.8 m/s mapping run.
FIT_MIN_SPEED = 0.5
# Fewer moving samples than this and a fit says nothing: report that instead.
FIT_MIN_SAMPLES = 20


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_imu_bag(bag_dir: Path) -> dict:
    """Gyro yaw rates and poses from the bag, as (receive time, ...) arrays."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = Path(bag_dir)
    storage = "mcap" if any(bag_dir.glob("*.mcap")) else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in (*GYRO_TOPICS, *POSE_TOPICS) if t in types]
    rows = {name: [] for name in (*GYRO_TOPICS.values(), *POSE_TOPICS.values())}
    if wanted:
        reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    while wanted and reader.has_next():
        topic, raw, t_ns = reader.read_next()
        msg = deserialize_message(raw, get_message(types[topic]))
        t = t_ns / 1e9
        if topic in GYRO_TOPICS:
            rows[GYRO_TOPICS[topic]].append((t, msg.angular_velocity.z))
        else:
            pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
            rows[POSE_TOPICS[topic]].append(
                (t, pose.position.x, pose.position.y, yaw_of(pose.orientation)))
    out = {}
    for name in GYRO_TOPICS.values():
        out[name] = np.array(rows[name], dtype=np.float64).reshape(-1, 2)
    for name in POSE_TOPICS.values():
        out[name] = np.array(rows[name], dtype=np.float64).reshape(-1, 4)
    return out


def load_imu(run_dir, refresh: bool = False) -> dict:
    from run_analysis.runlog import analysis_dir

    run_dir = Path(run_dir).expanduser()
    cache = run_dir / "analysis" / IMU_CACHE
    if refresh or not cache.exists():
        np.savez_compressed(analysis_dir(run_dir) / IMU_CACHE, **read_imu_bag(run_dir / "bag"))
    z = np.load(cache)
    return {k: z[k] for k in z.files}


# ---------------------------------------------------------------- analyses (pure)

def odom_yaw(odom: np.ndarray) -> np.ndarray:
    """runlog's odom rows (t, vx, wz, x, y, qz, qw) -> planar yaw."""
    return 2.0 * np.arctan2(odom[:, 5], odom[:, 6])


def turn_rate_table(ref_t, ref_w, others: dict, odom: np.ndarray, min_speed=MIN_SPEED):
    """Each source against the reference gyro, while the car is moving.

    others: {name: (t, w)}. Each is interpolated onto the reference's times
    inside its own time span. Returns [(name, n, rms of other - ref,
    mean of other - ref, least-squares gain other ~ g * ref)]."""
    speed = np.interp(ref_t, odom[:, 0], odom[:, 1]) if len(odom) else np.zeros_like(ref_t)
    rows = []
    for name, (t, w) in others.items():
        if len(t) < 2:
            rows.append((name, 0, math.nan, math.nan, math.nan))
            continue
        inside = (ref_t >= t[0]) & (ref_t <= t[-1]) & (np.abs(speed) >= min_speed)
        if not inside.any():
            rows.append((name, 0, math.nan, math.nan, math.nan))
            continue
        other = np.interp(ref_t[inside], t, w)
        ref = ref_w[inside]
        diff = other - ref
        den = float(np.sum(ref * ref))
        rows.append((name, int(inside.sum()), float(np.sqrt(np.mean(diff ** 2))),
                     float(np.mean(diff)), float(np.sum(other * ref) / den) if den > 0 else math.nan))
    return rows


def gyro_motion(t, w, odom: np.ndarray) -> Motion:
    """A gyro as a scan_motion.Motion: its yaw rate, /odom's speed. So
    actuation.analyze can fit lag and gain against it like against the LiDAR."""
    v = np.interp(t, odom[:, 0], odom[:, 1])
    zeros = np.zeros_like(t)
    return Motion(np.asarray(t, float), v, zeros, np.asarray(w, float), zeros)


def heading_corrections(ref: np.ndarray, series: dict, odom: np.ndarray):
    """How much heading the reference pose had to add on top of each series.

    ref: (t, x, y, yaw) rows, normally SLAM's map-frame pose. series:
    {name: (t, yaw)}. Both unwrapped; correction(t) = ref yaw - series yaw,
    zeroed at the first common time. Returns [(name, final, max |.|,
    per 100 m of /odom travel)] in radians. A good odometry needs little.
    """
    out = []
    if len(ref) < 2:
        return out
    path = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(odom[:, 3]), np.diff(odom[:, 4])))]) \
        if len(odom) > 1 else None
    for name, (t, yaw) in series.items():
        if len(t) < 2:
            continue
        inside = (ref[:, 0] >= t[0]) & (ref[:, 0] <= t[-1])
        if inside.sum() < 2:
            continue
        rt = ref[inside, 0]
        c = np.unwrap(ref[inside, 3]) - np.interp(rt, t, np.unwrap(yaw))
        c -= c[0]
        dist = float(np.interp(rt[-1], odom[:, 0], path) - np.interp(rt[0], odom[:, 0], path)) \
            if path is not None else math.nan
        out.append((name, float(c[-1]), float(np.max(np.abs(c))),
                    float(abs(c[-1]) / dist * 100.0) if dist > 1.0 else math.nan))
    return out


# ---------------------------------------------------------------- report

def _fmt_rate_rows(rows) -> list:
    out = []
    for name, n, rms, mean, gain in rows:
        if n == 0:
            out.append(f"  {name:22s} no overlap while moving")
        else:
            out.append(f"  {name:22s} n={n:6d}  RMS {rms:.3f} rad/s  mean {mean:+.3f}  "
                       f"gain {gain:.3f}")
    return out


def build_report(run, imu: dict, motion=None) -> list:
    """The report's lines. run: runlog.Run; imu: load_imu(); motion: an
    optional scan_motion.Motion (LiDAR) for the whole run."""
    lines = []
    ref_name = "bno" if len(imu.get("bno", ())) >= 2 else (
        "vesc" if len(imu.get("vesc", ())) >= 2 else None)
    if ref_name is None:
        return ["no gyro in this bag (/imu/bno/data, /imu/vesc/data): was the IMU stack up?"]
    ref = imu[ref_name]
    lines.append(f"Turn rate vs the {ref_name} gyro, while moving (>= {MIN_SPEED} m/s):")
    others = {"odom (steering model)": (run.odom[:, 0], run.odom[:, 2])}
    if ref_name == "bno" and len(imu.get("vesc", ())) >= 2:
        others["vesc gyro"] = (imu["vesc"][:, 0], imu["vesc"][:, 1])
    if len(imu.get("odom_imu", ())) >= 2:
        oi = imu["odom_imu"]
        yaw = np.unwrap(oi[:, 3])
        mid = 0.5 * (oi[1:, 0] + oi[:-1, 0])
        dt = np.diff(oi[:, 0])
        ok = dt > 0
        others["/odom/imu"] = (mid[ok], np.diff(yaw)[ok] / dt[ok])
    if motion is not None and len(motion.t) >= 2:
        others["LiDAR scan matching"] = (motion.t, motion.w)
    lines += _fmt_rate_rows(turn_rate_table(ref[:, 0], ref[:, 1], others, run.odom))

    gm = gyro_motion(ref[:, 0], ref[:, 1], run.odom) if len(run.odom) >= 2 else None
    moving = int(np.sum(gm.v > FIT_MIN_SPEED)) if gm is not None else 0
    if len(run.drive) < 20:
        lines.append("Steering response: no /drive commands in this bag")
    elif moving < FIT_MIN_SAMPLES:
        lines.append(f"Steering response: only {moving} gyro samples above {FIT_MIN_SPEED} m/s "
                     "-- not enough driving to fit lag and gain")
    else:
        try:
            rep = actuation.analyze(gm, run.drive, run.odom, ref[0, 0], ref[-1, 0],
                                    min_speed=FIT_MIN_SPEED)
            lines.append(f"Steering response measured by the {ref_name} gyro "
                         f"(above {FIT_MIN_SPEED} m/s):")
            # [1:3]: steering and lateral acceleration. Line 0 is speed lag,
            # which here would be /odom's own wheel speed, not the gyro.
            lines += ["  " + line for line in rep.lines()[1:3]]
        except ValueError as e:
            lines.append(f"Steering response: not enough driving ({e})")
        if motion is not None and int(np.sum(motion.v > FIT_MIN_SPEED)) >= FIT_MIN_SAMPLES:
            try:
                rep = actuation.analyze(motion, run.drive, run.odom, motion.t[0], motion.t[-1],
                                        min_speed=FIT_MIN_SPEED)
                lines.append("Steering response measured by LiDAR scan matching:")
                lines += ["  " + line for line in rep.lines()[:3]]
            except ValueError as e:
                lines.append(f"LiDAR steering response: not enough driving ({e})")

    slam_name = "slam" if len(imu.get("slam", ())) >= 2 else (
        "slam_pose" if len(imu.get("slam_pose", ())) >= 2 else None)
    if slam_name is None:
        lines.append("Heading corrections: no SLAM pose (/pose, /slam_pose) in this bag")
    else:
        series = {"/odom": (run.odom[:, 0], odom_yaw(run.odom))} if len(run.odom) else {}
        if len(imu.get("odom_imu", ())) >= 2:
            series["/odom/imu"] = (imu["odom_imu"][:, 0], imu["odom_imu"][:, 3])
        rows = heading_corrections(imu[slam_name], series, run.odom)
        lines.append(f"Heading correction SLAM ({slam_name}) applied on top of each odometry:")
        for name, final, peak, per100 in rows:
            lines.append(f"  {name:10s} end {math.degrees(final):+7.2f} deg  max "
                         f"{math.degrees(peak):6.2f} deg  per 100 m {math.degrees(per100):6.2f} deg")
        lines.append("  (SLAM's pose is built on the odometry it was given: compare runs, not "
                     "just these numbers)")
    return lines


def main(argv=None) -> int:
    from run_analysis.runlog import analysis_dir, load_run
    from run_analysis.scan_motion import scan_motion

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run_dir")
    ap.add_argument("--no-icp", action="store_true", help="skip LiDAR scan matching")
    ap.add_argument("--refresh", action="store_true", help="re-read the bag")
    args = ap.parse_args(argv)
    run = load_run(args.run_dir, refresh=args.refresh)
    imu = load_imu(args.run_dir, refresh=args.refresh)
    motion = None
    if not args.no_icp:
        motion = scan_motion(run, float(run.scan_t[0]), float(run.scan_t[-1]))
    text = "\n".join(build_report(run, imu, motion)) + "\n"
    (analysis_dir(args.run_dir) / "imu_report.txt").write_text(text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
