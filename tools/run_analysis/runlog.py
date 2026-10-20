"""Load a recorded run (~/.ros/racerbot_runs/<ts>/) into plain numpy arrays.

``race_diagnostics``' ``record_run.py`` leaves a directory holding
``launch.log``, ``events.jsonl`` and ``bag/`` (rosbag2, mcap). The bag is the
only thing with every tick in it, and reading it needs ``rosbag2_py`` -- so
this module needs ROS sourced, but only the first time: the arrays are cached
in ``<run>/analysis/run.npz``, and every other tool in this directory reads
the cache and runs unsourced.

    from run_analysis.runlog import load_run
    run = load_run("~/.ros/racerbot_runs/20261019-200703")
    run.scan            # (N, beams) float32, NaN/inf as recorded
    run.drive           # (M, 4): receive time, header stamp, steering, speed
    run.lb_windows()    # [(t_press, t_release), ...] from /joy button 4

Times are receive times in seconds since the epoch, from the bag.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

CACHE_NAME = "run.npz"
DEADMAN_BUTTON = 4  # LB on the F710 in XInput mode -- gap_follow.yaml deadman_button


@dataclass
class Run:
    path: Path
    scan_t: np.ndarray        # (N,) receive time
    scan_stamp: np.ndarray    # (N,) header stamp
    scan: np.ndarray          # (N, beams) ranges
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    odom: np.ndarray          # (K, 7): t, v_x, w_z, x, y, qz, qw
    drive: np.ndarray         # (M, 4): t, stamp, steering, speed
    joy: np.ndarray           # (J, 2): t, deadman button state

    def beam_angles(self) -> np.ndarray:
        return self.angle_min + np.arange(self.scan.shape[1]) * self.angle_increment

    def lb_windows(self, min_duration: float = 0.5):
        """Spans during which the deadman button was held, from /joy."""
        if self.joy.size == 0:
            return []
        held = self.joy[:, 1] > 0.5
        windows, start = [], None
        for (t, _), h in zip(self.joy, held):
            if h and start is None:
                start = t
            elif not h and start is not None:
                windows.append((start, t))
                start = None
        if start is not None:
            windows.append((start, self.joy[-1, 0]))
        return [w for w in windows if w[1] - w[0] >= min_duration]

    def odom_speed_at(self, t: float) -> float:
        """The latest /odom speed received at or before ``t`` -- what the node
        had in hand when it processed a scan received at ``t``."""
        i = int(np.searchsorted(self.odom[:, 0], t, side="right")) - 1
        return float(self.odom[max(i, 0), 1])


def read_bag(bag_dir: Path) -> dict:
    """Read the topics the analysis needs straight from the rosbag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = Path(bag_dir)
    storage = "mcap" if any(bag_dir.glob("*.mcap")) else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in ("/scan", "/odom", "/drive", "/joy") if t in types]
    if "/scan" not in wanted or "/drive" not in wanted:
        raise ValueError(f"{bag_dir}: bag needs /scan and /drive, has {sorted(types)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    scans, odom, drive, joy = [], [], [], []
    meta = None
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        msg = deserialize_message(raw, get_message(types[topic]))
        t = t_ns / 1e9
        if topic == "/scan":
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            scans.append((t, stamp, np.asarray(msg.ranges, dtype=np.float32)))
            meta = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        elif topic == "/odom":
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            odom.append((t, msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                         p.x, p.y, q.z, q.w))
        elif topic == "/drive":
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            drive.append((t, stamp, msg.drive.steering_angle, msg.drive.speed))
        elif topic == "/joy":
            button = msg.buttons[DEADMAN_BUTTON] if len(msg.buttons) > DEADMAN_BUTTON else 0
            joy.append((t, button))
    if not scans:
        raise ValueError(f"{bag_dir}: no /scan messages")
    widths = {len(s[2]) for s in scans}
    if len(widths) != 1:
        raise ValueError(f"{bag_dir}: scans have varying beam counts {sorted(widths)}")
    return dict(
        scan_t=np.array([s[0] for s in scans]),
        scan_stamp=np.array([s[1] for s in scans]),
        scan=np.stack([s[2] for s in scans]),
        scan_meta=np.array(meta, dtype=np.float64),
        odom=np.array(odom, dtype=np.float64).reshape(-1, 7),
        drive=np.array(drive, dtype=np.float64).reshape(-1, 4),
        joy=np.array(joy, dtype=np.float64).reshape(-1, 2),
    )


def analysis_dir(run_dir) -> Path:
    out = Path(run_dir).expanduser() / "analysis"
    out.mkdir(exist_ok=True)
    return out


def load_run(run_dir, refresh: bool = False) -> Run:
    """The run's arrays, from the cache if present, else from the bag."""
    run_dir = Path(run_dir).expanduser()
    cache = run_dir / "analysis" / CACHE_NAME
    if refresh or not cache.exists():
        data = read_bag(run_dir / "bag")
        np.savez_compressed(analysis_dir(run_dir) / CACHE_NAME, **data)
    z = np.load(cache)
    meta = z["scan_meta"]
    return Run(run_dir, z["scan_t"], z["scan_stamp"], z["scan"], float(meta[0]),
               float(meta[1]), float(meta[2]), float(meta[3]), z["odom"], z["drive"], z["joy"])


def read_events(run_dir) -> list:
    """events.jsonl from race_diagnostics' probe, one dict per line."""
    path = Path(run_dir).expanduser() / "events.jsonl"
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
