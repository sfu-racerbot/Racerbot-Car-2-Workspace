"""SLAM-side arrays from a recorded run's bag, cached in <run>/analysis/slam.npz.

load(run_dir) reads <run>/bag on the first call and serves the cache after that;
refresh=True rebuilds it. A replay folder writes its own cache, in its own analysis/.
Reading the bag needs rosbag2_py and rclpy (ROS sourced, or via tools/rosenv); importing
this module needs neither.

Times are the bag's receive times in epoch seconds, the same clock slam_metrics compares
against the Ceres log.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

CACHE = "slam.npz"
FORMAT = 1  # bump when the cached arrays change, so an older cache is rebuilt, not misread
# Topics kept as rows (t, x, y, yaw), by the key they are stored under.
POSE_TOPICS = {"/odom": "odom", "/odom/imu": "odom_imu", "/pose": "pose", "/slam_pose": "slam_pose"}
MAP_TF_CHILDREN = ("odom", "odom_imu")  # the odometry frames SLAM can map on
ROW_KEYS = ("map_odom", "odom_base", "odom", "odom_imu", "pose", "slam_pose")


class NoSlamMapping(ValueError):
    """Raised when the bag has no map → odom/odom_imu transform."""


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(bag_dir) -> dict:
    """Read one bag directory into the arrays that load() caches. Touches no cache."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = Path(bag_dir)
    storage = "mcap" if any(bag_dir.glob("*.mcap")) else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in ("/tf", "/map", *POSE_TOPICS) if t in types]
    if wanted:
        reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    rows = {k: [] for k in ROW_KEYS}
    children = set()
    last_map = None  # only the newest /map is kept, and it is decoded once, after the loop
    while wanted and reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns * 1e-9
        if topic == "/map":
            last_map = raw
            continue
        msg = deserialize_message(raw, get_message(types[topic]))
        if topic == "/tf":
            for tr in msg.transforms:
                q = tr.transform
                row = (t, q.translation.x, q.translation.y, _yaw(q.rotation))
                if tr.header.frame_id == "map" and tr.child_frame_id in MAP_TF_CHILDREN:
                    rows["map_odom"].append(row)
                    children.add(tr.child_frame_id)
                elif tr.header.frame_id in MAP_TF_CHILDREN and tr.child_frame_id == "base_link":
                    rows["odom_base"].append(row)
        else:
            # Odometry and PoseWithCovariance carry pose.pose; PoseStamped carries the pose directly.
            pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
            rows[POSE_TOPICS[topic]].append(
                (t, pose.position.x, pose.position.y, _yaw(pose.orientation)))
    if not children:
        raise NoSlamMapping(f"{bag_dir}: no map -> odom or map -> odom_imu transform on /tf; "
                            "SLAM did not map this run")
    if len(children) > 1:
        raise ValueError(f"{bag_dir}: map has more than one odometry child, {sorted(children)}")
    out = {k: np.array(v, dtype=float).reshape(-1, 4) for k, v in rows.items()}
    out["map_grid"] = np.zeros((0, 0), dtype=np.int8)
    out["map_meta"] = np.zeros(0)
    if last_map is not None:
        msg = deserialize_message(last_map, get_message(types["/map"]))
        info = msg.info
        out["map_grid"] = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)
        out["map_meta"] = np.array([info.resolution, info.origin.position.x, info.origin.position.y])
    out["odom_frame"] = np.array(children.pop())
    return out


def _unpack(arrays: dict) -> dict:
    grid, meta = arrays["map_grid"], arrays["map_meta"]
    out = {k: arrays[k] for k in ROW_KEYS}
    out["map"] = (float(meta[0]), float(meta[1]), float(meta[2]), grid) if grid.size else None
    out["odom_frame"] = str(arrays["odom_frame"])
    return out


def load(run_dir, refresh: bool = False) -> dict:
    """A run's SLAM arrays. Keys:

    map_odom, odom_base, odom, odom_imu, pose, slam_pose: rows (t, x, y, yaw); empty when
        the bag has none. map_odom is map -> the odometry frame SLAM mapped on; odom_base is
        odom(_imu) -> base_link from /tf; odom and odom_imu are the /odom and /odom/imu topics.
    map: the newest /map as (resolution, origin_x, origin_y, grid int8 2-D), or None.
    odom_frame: that odometry frame, 'odom' or 'odom_imu'.
    """
    from run_analysis.runlog import analysis_dir

    run_dir = Path(run_dir).expanduser()
    cache = run_dir / "analysis" / CACHE
    arrays = None
    if cache.exists() and not refresh:
        with np.load(cache) as z:
            if "format" in z.files and int(z["format"]) == FORMAT:
                arrays = {k: z[k] for k in z.files}
    if arrays is None:
        arrays = read_bag(run_dir / "bag")
        arrays["format"] = np.array(FORMAT)
        partial = analysis_dir(run_dir) / "slam.partial.npz"
        np.savez_compressed(partial, **arrays)
        partial.replace(cache)
    return _unpack(arrays)
