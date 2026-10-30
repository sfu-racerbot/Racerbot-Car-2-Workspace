"""Replay a recorded run through slam_toolbox with the car parked, to tune SLAM without driving.

    python3 tools/run_analysis/slam_replay.py ~/.ros/racerbot_runs/<ts> \
        [--odom odom|odom_imu] [--profile FILE] [--rate R | --exact] [--label NAME]

The run's bag is played into a slam_toolbox started from the run's own recorded settings, with
the profile's overrides laid over them. Rate mode and --exact-mode all-scans publish /scan,
/tf_static and one odometry topic, with odom_tf_relay.py rebuilding odometry TF. Forced exact
mode instead publishes original /tf payloads minus map-parented transforms, bracketed per
needed dynamic pair, plus /tf_static and only the forced scan plan. SLAM recomputes map TF.

Refuses while a driver process is alive. DRIVER_MARKERS are matched against every process's whole
command line, so a driver started under another name still counts. Every child runs on the
replay's own ROS domain (REPLAY_DOMAIN_ID), so a replay never joins the car's graph.

plan_replay and the pure planning helpers import without rclpy. Running a replay starts ROS
children, so it needs the ROS environment sourced (ros2 on PATH).
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from bisect import bisect_left
from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]  # for the commit recorded in replay.json
RELAY = Path(__file__).resolve().with_name('odom_tf_relay.py')
RUNS_ROOT = Path.home() / '.ros' / 'racerbot_runs'  # where replay folders are written
PROC_ROOT = Path('/proc')  # the process table the interlock reads

REPLAY_DOMAIN_ID = 77
DRIVER_MARKERS = ('vesc_driver_node', 'urg_node', 'joy_node', '__node:=joy')
ODOM_TOPICS = {'odom': '/odom', 'odom_imu': '/odom/imu'}
PLAY_TOPICS = ('/scan', '/tf_static')  # the odometry topic is played as well
RECORD_TOPICS = ('/tf', '/pose', '/map', '/map_metadata', '/slam_toolbox/graph_visualization')
DROPPED_PARAMS = ('qos_overrides', 'start_type_description_service')  # describe the live graph
LABEL_PATTERN = re.compile(r'[A-Za-z0-9][A-Za-z0-9._-]{0,63}')
MAP_WAIT_S = 30.0  # how long slam_toolbox has to list /map; the replay is refused after that
MAP_SAVE_ATTEMPTS = 3  # slam_toolbox's map saver can answer success and write nothing, so save is retried
MAP_SAVE_GAP_S = 2.0  # the pause between save attempts
SETTLE_S = 5.0  # time for slam_toolbox to finish the scans it has queued once the bag ends
STOP_GRACE_S = 10.0  # time a child gets after each stop signal
SERVICE_TIMEOUT_S = 60.0
SAVE_MAP = ('/slam_toolbox/save_map', 'slam_toolbox/srv/SaveMap')
SERIALIZE_MAP = ('/slam_toolbox/serialize_map', 'slam_toolbox/srv/SerializePoseGraph')


class ReplayRefused(RuntimeError):
    """The run or the request cannot be replayed. The message says why, and nothing was started."""


class ReplayFailed(RuntimeError):
    """A replay that had started could not finish. The output folder keeps its logs."""


class StampMismatch(ReplayRefused):
    """The recorded processed scans cannot be identified unambiguously."""


def exact_scan_plan(scan_stamps_ns, pose_stamps_ns) -> list[int]:
    """Return one scan stamp per recorded pose, in pose order; no exceptions.

    Run 20261026-194913 has 315 poses, all matching distinct recorded scans.
    Its 314 'updates' are changes between map->odom_imu TF corrections (initial
    correction identity), not 314 processed scans. No start pose is exempt.
    Missing scans mean the recorder dropped input SLAM processed; duplicates
    are ambiguous. Either case refuses before starting replay children.
    """
    seen = set()
    for stamp in scan_stamps_ns:
        if stamp in seen:
            raise StampMismatch(f'duplicate scan stamp {stamp}; exact replay is ambiguous')
        seen.add(stamp)
    poses = list(pose_stamps_ns)
    missing = [stamp for stamp in poses if stamp not in seen]
    if missing:
        raise StampMismatch(f'{len(missing)} processed scan stamps missing; first {missing[0]}; '
                            'the recorder dropped a scan SLAM used; no start-pose exception')
    return poses


def exact_feed_order(scan_stamps_ns, pose_stamps_ns):
    """Feed every recorded scan, using recorded pose stamps only as wait checkpoints."""
    scans = list(scan_stamps_ns)
    planned = set(exact_scan_plan(scans, pose_stamps_ns))
    return [(stamp, stamp in planned) for stamp in scans]


def forced_scan_plan(scan_stamps_ns, pose_stamps_ns):
    """First planned scan, three startup sacrifices, then remaining planned scans.

    2.8.5 src/slam_toolbox_common.cpp:795 shouldProcessScan rejects callback counts
    below 5 after accepting the first measurement. The three recorded scans
    immediately before the second pose consume those callbacks without adding
    nodes to the map. They must lie strictly after the first planned scan;
    the entire feed must advance in stamp. A false flag means sacrificial,
    not a pose checkpoint.
    """
    scans = list(scan_stamps_ns)
    planned = exact_scan_plan(scans, pose_stamps_ns)
    if len(planned) < 2:
        return [(stamp, True) for stamp in planned]
    first, second = scans.index(planned[0]), scans.index(planned[1])
    if second - first - 1 < 3:
        raise ReplayRefused(f'need three distinct intervening scans after first planned stamp {planned[0]} '
                            f'before second planned stamp {planned[1]}')
    feed = ([(planned[0], True)]
            + [(stamp, False) for stamp in scans[second - 3:second]]
            + [(stamp, True) for stamp in planned[1:]])
    for (previous, _), (current, _) in zip(feed, feed[1:]):
        if current <= previous:
            raise ReplayRefused(f'forced scan stamps must be strictly increasing: {previous} -> {current}; '
                                f'second planned stamp {planned[1]}')
    return feed


def exact_admission_parameters(params, exact_mode, recorded=None):
    """Apply admission-only overrides in forced mode, with an audit of recorded values.

    Every planned scan already passed the car's travel/time filters relative
    to its previous processed pose. With identical inputs it passes again;
    forcing these gates open removes timing/callback-dependent rejections,
    while leaving the scan matching, map and solver configuration unchanged.
    """
    body = dict(params['slam_toolbox']['ros__parameters'])
    audit = {}
    if exact_mode == 'forced':
        source = body if recorded is None else recorded
        overrides = dict(throttle_scans=1, minimum_time_interval=0.0,
                         minimum_travel_distance=0.0, minimum_travel_heading=0.0)
        audit = {key: {'recorded': source.get(key), 'override': value}
                 for key, value in overrides.items()}
        body.update(overrides)
    return {'slam_toolbox': {'ros__parameters': body}}, audit


def exact_feed_steps(odom_stamps_ns, scan_stamps_ns, pose_stamps_ns):
    """(scan stamp, wait for pose, odometry end index, bracket available), in bag order.

    Only unplanned scans may lack brackets. Send available odometry through
    the first sample at/after the scan (or the end of the bag for a tail scan).
    """
    odoms = list(odom_stamps_ns)
    steps = []
    for stamp, planned in exact_feed_order(scan_stamps_ns, pose_stamps_ns):
        bracketed = bool(odoms) and odoms[0] <= stamp <= odoms[-1]
        if planned and not bracketed:
            raise StampMismatch(f'no odometry interpolation bracket for planned scan {stamp}')
        end = min(bisect_left(odoms, stamp) + 1, len(odoms))
        steps.append((stamp, planned, end, bracketed))
    return steps


class ExactPoseAccounting:
    """Track all observed poses and planned timeouts without making extras fatal."""

    def __init__(self, planned_stamps):
        self.planned = set(planned_stamps)
        self.observed = set()
        self.timed_out = []
        self.consecutive_missing = []

    def observe(self, stamp):
        self.observed.add(stamp)

    def checkpoint(self, stamp):
        if stamp in self.observed:
            self.consecutive_missing.clear()
            return True
        self.timed_out.append(stamp)
        self.consecutive_missing.append(stamp)
        if len(self.consecutive_missing) > 10:
            raise ReplayFailed(f'{len(self.consecutive_missing)} consecutive planned poses missing: '
                               f'{self.consecutive_missing}')
        return False

    def summary(self):
        return {
            'matched_planned': len(self.planned & self.observed),
            'missing_planned_stamps': sorted(self.planned - self.observed),
            'extra_pose_stamps': sorted(self.observed - self.planned),
            'timed_out_stamps': list(self.timed_out),
        }


def pose_sequence_error(recorded, replayed) -> dict:
    """Match (stamp_ns, x, y, yaw) rows; measure against 1 mm / 0.1 degree.

    Missing and extra stamps are counted separately from numeric errors.
    first_over_bar follows recorded order, and yaw wraps to [-pi, pi).
    """
    recorded, replayed = list(recorded), list(replayed)
    source = {row[0]: row[1:] for row in recorded}
    target = {row[0]: row[1:] for row in replayed}
    if len(source) != len(recorded) or len(target) != len(replayed):
        raise ValueError('duplicate pose stamps cannot be compared unambiguously')
    result = dict(matched=0, missing_in_replay=len(source.keys() - target.keys()),
                  extra_in_replay=len(target.keys() - source.keys()),
                  max_position_m=0.0, max_yaw_deg=0.0, first_over_bar=None)
    for stamp, (x, y, yaw) in source.items():
        if stamp not in target:
            continue
        rx, ry, ryaw = target[stamp]
        if not all(math.isfinite(v) for v in (x, y, yaw, rx, ry, ryaw)):
            raise ValueError(f'non-finite pose at {stamp}')
        position = math.hypot(rx - x, ry - y)
        rotation = abs(math.degrees((ryaw - yaw + math.pi) % (2 * math.pi) - math.pi))
        result['matched'] += 1
        result['max_position_m'] = max(result['max_position_m'], position)
        result['max_yaw_deg'] = max(result['max_yaw_deg'], rotation)
        if result['first_over_bar'] is None and (position > .001 or rotation > .1):
            result['first_over_bar'] = stamp
    return result


def map_from_bag_files(grid, resolution, origin_xy, out_dir) -> tuple[Path, Path]:
    """Save trinary pixels with nav2's y flip, and S4's 0.65/0.25 YAML thresholds.

    Convention checked in navigation2 jazzy nav2_map_server/src/map_io.cpp,
    tryWriteMapToFile: <=25 free, >=65 occupied, others unknown; rows inverted.
    https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_map_server/src/map_io.cpp
    YAML uses the requested free_thresh 0.25 (newer nav2 uses 0.196 there).
    """
    import numpy as np
    cells = np.asarray(grid)
    if cells.ndim != 2 or not cells.size:
        raise ValueError('grid must be a nonempty 2D occupancy grid')
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('resolution must be finite and positive')
    pixels = np.full(cells.shape, 205, dtype=np.uint8)
    pixels[(cells >= 0) & (cells <= 25)] = 254
    pixels[(cells >= 65) & (cells <= 100)] = 0
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    pgm, meta = out / 'map.pgm', out / 'map.yaml'
    height, width = cells.shape
    pgm.write_bytes(f'P5\n{width} {height}\n255\n'.encode('ascii') + pixels[::-1].tobytes())
    meta.write_text(yaml.safe_dump(dict(image='map.pgm', mode='trinary',
                    resolution=float(resolution), origin=[float(origin_xy[0]), float(origin_xy[1]), 0],
                    negate=0, occupied_thresh=.65, free_thresh=.25), sort_keys=False), encoding='utf-8')
    return pgm, meta


def exact_children(params_file, bag, odom_topic, base_env):
    """Plan exact-only children with their isolated environments (also used by _run)."""
    return [
        ('slam_toolbox', ['ros2', 'run', 'slam_toolbox', 'sync_slam_toolbox_node',
                         '--ros-args', '--params-file', str(params_file), '-p', 'use_sim_time:=true'],
         child_env(base_env)),
        ('exact_feed', [sys.executable, str(Path(__file__).with_name('slam_exact_feed.py')),
                        str(bag), '--odom-topic', odom_topic], child_env(base_env)),
    ]


def bag_messages(bag, topics):
    """Read selected messages lazily; ROS imports stay outside the pure planning API."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, types[topic])


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def bag_pose_rows(bag):
    rows = []
    for _, msg in bag_messages(bag, ['/pose']):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        rows.append((stamp_ns(msg), p.position.x, p.position.y, yaw))
    return rows


def save_map_from_bag(bag, out_dir):
    """Save the last recorded /map once the recorder has closed and flushed its bag."""
    import numpy as np
    last = None
    for _, msg in bag_messages(bag, ['/map']):
        last = msg
    if last is None:
        return False
    grid = np.asarray(last.data).reshape(last.info.height, last.info.width)
    origin = last.info.origin.position
    map_from_bag_files(grid, last.info.resolution, (origin.x, origin.y), out_dir)
    return 'from_bag'


def bag_exact_plan(bag):
    stamps = {'/scan': [], '/pose': []}
    for topic, msg in bag_messages(bag, stamps):
        stamps[topic].append(stamp_ns(msg))
    if not stamps['/pose']:
        raise ReplayRefused('exact replay needs recorded /pose messages')
    return exact_scan_plan(stamps['/scan'], stamps['/pose'])


def odometry_brackets(odom_stamps_ns, scan_stamps_ns):
    """Exclusive odometry end indices including each scan's interpolation bracket."""
    odoms = list(odom_stamps_ns)
    ends = []
    for stamp in scan_stamps_ns:
        index = bisect_left(odoms, stamp)
        if index == len(odoms) or stamp < odoms[0]:
            raise StampMismatch(f'no odometry interpolation bracket for scan {stamp}')
        ends.append(index + 1)
    return ends


def filter_tf_transforms(transforms):
    """Keep original non-map transforms, in message order; never mutate the input."""
    return [tf for tf in transforms if tf.header.frame_id != 'map']


def needed_tf_pairs(dynamic_pairs, static_pairs, odom_frame, base_frame):
    """Derive the dynamic interpolation pairs along the recorded odom/base TF chain."""
    dynamic, static = set(dynamic_pairs), set(static_pairs)
    graph, parents = {}, {}
    for parent, child in sorted(dynamic | static):
        if child in parents and parents[child] != parent:
            raise ReplayRefused(f'ambiguous recorded TF parents for {child}: {parents[child]}, {parent}')
        parents[child] = parent
        graph.setdefault(parent, []).append((child, (parent, child)))
        graph.setdefault(child, []).append((parent, (parent, child)))
    pending, seen = deque([(odom_frame, [])]), {odom_frame}
    while pending:
        frame, path = pending.popleft()
        if frame == base_frame:
            return [pair for pair in path if pair not in static]
        for neighbor, pair in graph.get(frame, []):
            if neighbor not in seen:
                seen.add(neighbor)
                pending.append((neighbor, [*path, pair]))
    raise ReplayRefused(f'no recorded TF chain from {odom_frame} to {base_frame}')


def tf_bracket_plan(messages, needed_pairs, scan_stamps_ns):
    """Exclusive recorded-message prefixes covering every required pair at each scan.

    messages contains rows (parent, child, header_stamp_ns), grouped exactly
    as recorded. Cover a sample at/before and the first at/after the scan for
    each pair, without sorting or regrouping the original message stream.
    """
    samples = {pair: [] for pair in needed_pairs}
    for index, message in enumerate(messages):
        for parent, child, stamp in message:
            if (parent, child) in samples:
                samples[parent, child].append((stamp, index + 1))
    ends = []
    for stamp in scan_stamps_ns:
        end = 0
        for pair, rows in samples.items():
            before = next((index for time_ns, index in rows if time_ns <= stamp), None)
            after = next((index for time_ns, index in rows if time_ns >= stamp), None)
            if before is None or after is None:
                raise StampMismatch(f'no recorded TF bracket for {pair[0]} -> {pair[1]} at scan {stamp}')
            end = max(end, before, after)
        ends.append(end)
    return ends


def prepare_recorded_tf(messages, statics, odom_frame, base_frame, scan_stamps_ns):
    """Filter correction payloads, derive the chain and plan prefixes; ROS-free data access."""
    filtered, rows = [], []
    dropped = mixed = 0
    for message in messages:
        kept = filter_tf_transforms(message.transforms)
        removed = len(message.transforms) - len(kept)
        dropped += removed
        mixed += bool(removed and kept)
        if kept:
            filtered.append(kept)
            rows.append([(tf.header.frame_id, tf.child_frame_id, stamp_ns(tf)) for tf in kept])
    pairs = needed_tf_pairs(
        [(parent, child) for message in rows for parent, child, _ in message],
        [(tf.header.frame_id, tf.child_frame_id) for tf in statics], odom_frame, base_frame)
    ends = tf_bracket_plan(rows, pairs, scan_stamps_ns)
    return filtered, ends, {
        'needed_tf_pairs': [list(pair) for pair in pairs],
        'map_transforms_dropped': dropped, 'mixed_tf_messages': mixed,
        'tf_messages_available': len(filtered),
    }


class ServiceError(RuntimeError):
    """One call to a slam_toolbox service did not answer, or answered with an error.

    By itself this does not stop a replay: save_with_retries decides what it means.
    """


# --- the interlock ------------------------------------------------------------------------------

def running_drivers(proc_root: Path = Path('/proc')) -> list[str]:
    """The DRIVER_MARKERS that appear in any process's command line, in marker order.

    Only numeric entries are processes. A process that exits or cannot be read between listing
    and reading is skipped, and so is an empty command line (a kernel thread).
    """
    seen: set[str] = set()
    for entry in Path(proc_root).iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / 'cmdline').read_bytes()
        except OSError:
            continue
        cmdline = raw.replace(b'\0', b' ').decode('utf-8', errors='replace')
        seen.update(marker for marker in DRIVER_MARKERS if marker in cmdline)
    return [marker for marker in DRIVER_MARKERS if marker in seen]


def child_env(base: Mapping[str, str]) -> dict[str, str]:
    """A copy of base that puts a child on the replay's own ROS graph, whatever base had."""
    env = dict(base)
    env['ROS_DOMAIN_ID'] = str(REPLAY_DOMAIN_ID)
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    env['ROS_STATIC_PEERS'] = ''
    return env


# --- reading the run ---------------------------------------------------------------------------

def _load_yaml(path: Path, what: str):
    try:
        return yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ReplayRefused(f'{what} {path} is unreadable: {exc}') from exc


def _parameters(doc, what: str, path: Path) -> dict:
    """The parameters of a YAML document, in either the recorded shape or a flat mapping.

    The recorded shape is {'/slam_toolbox': {'ros__parameters': {...}}}, which is how a run's
    metadata/params files are written. A profile may be flat or in that shape.
    """
    if not isinstance(doc, dict):
        raise ReplayRefused(f'{what} {path} is not a parameter mapping')
    if len(doc) == 1:
        (only,) = doc.values()
        if isinstance(only, dict) and 'ros__parameters' in only:
            inner = only['ros__parameters']
            if not isinstance(inner, dict):
                raise ReplayRefused(f'{what} {path}: ros__parameters is not a mapping')
            return dict(inner)
    return dict(doc)


def bag_topics(bag_dir: Path) -> set[str]:
    """The topic names listed in a rosbag2 bag's metadata.yaml. Needs no rosbag2 install."""
    bag_dir = Path(bag_dir)
    doc = _load_yaml(bag_dir / 'metadata.yaml', 'bag metadata')
    try:
        entries = doc['rosbag2_bagfile_information']['topics_with_message_count']
        return {str(entry['topic_metadata']['name']) for entry in entries}
    except (KeyError, TypeError) as exc:
        raise ReplayRefused(f'{bag_dir}/metadata.yaml has no topic list ({exc!r})') from exc


# --- the parameters a replay runs with ---------------------------------------------------------

def slam_parameters(snapshot: Mapping, odom_frame: str, profile: Mapping | None) -> dict:
    """The slam_toolbox params file: the recorded settings, then the profile, then forced keys.

    The forced keys are use_sim_time (the replay's clock is the bag's) and odom_frame. The keys
    in DROPPED_PARAMS are removed from the result: they describe the live graph, not this one.
    """
    merged = dict(snapshot)
    if profile:
        merged.update(profile)
    merged['use_sim_time'] = True
    merged['odom_frame'] = odom_frame
    kept = {key: value for key, value in merged.items() if not str(key).startswith(DROPPED_PARAMS)}
    return {'slam_toolbox': {'ros__parameters': kept}}


def loadable(params: dict) -> dict:
    """The params without the values a ROS 2 parameter file cannot hold: null and the empty list.

    slam_toolbox aborts on an empty list in a parameter file ("parameter_value_from failed ...
    No parameter value set"). A key left out gets the node's own default. The recorded
    map_start_pose is [], and no file in the stack sets it (f1tenth_online_async.yaml has it
    commented out, slam_tracking.yaml does not mention it), so the default is what the live node
    ran with.
    """
    body = params['slam_toolbox']['ros__parameters']
    kept = {key: value for key, value in body.items() if value is not None and value != []}
    return {'slam_toolbox': {'ros__parameters': kept}}


@dataclass
class ReplayPlan:
    run_dir: Path
    odom_frame: str
    out_dir: Path
    params: dict
    play_topics: list[str]
    odom_topic: str
    profile: dict | None = None


def plan_replay(run_dir, odom_frame: str, profile_path, out_root, label: str = 'default') -> ReplayPlan:
    """Check the run and the request, and say what a replay would do. Nothing is started or written."""
    run = Path(run_dir).expanduser().resolve()
    if odom_frame not in ODOM_TOPICS:
        raise ReplayRefused(f'unknown odometry frame {odom_frame!r}; choose one of {sorted(ODOM_TOPICS)}')
    if not LABEL_PATTERN.fullmatch(label):
        raise ReplayRefused(f'label {label!r} must be letters, digits, "." "_" or "-", '
                            'starting with a letter or digit, at most 64 characters')
    if not (run / 'bag' / 'metadata.yaml').is_file():
        raise ReplayRefused(f'{run} has no bag/metadata.yaml, so there is nothing to replay')
    settings = run / 'metadata' / 'params' / 'slam_toolbox.yaml'
    if not settings.is_file():
        raise ReplayRefused(f'{run} has no metadata/params/slam_toolbox.yaml; its SLAM settings were not recorded')

    topics = bag_topics(run / 'bag')
    odom_topic = ODOM_TOPICS[odom_frame]
    missing = [topic for topic in (*PLAY_TOPICS, odom_topic) if topic not in topics]
    if missing:
        raise ReplayRefused(f'{run} bag has no {", ".join(missing)}; cannot replay on odometry {odom_frame}')

    out_dir = (Path(out_root).expanduser() / f'{run.name}-replay-{odom_frame}-{label}').resolve()
    # record_run.py:111-116 always creates metadata/, including probe-only
    # recordings; :180-183 optionally creates bag/. Either directory marks
    # an existing run. Check every resolved ancestor, including nested runs.
    for existing_run in out_dir.parents:
        if (existing_run / 'bag').is_dir() or (existing_run / 'metadata').is_dir():
            if not out_dir.is_relative_to(existing_run / 'analysis'):
                raise ReplayRefused(f'{out_dir} is inside existing run {existing_run}; '
                                    'only its analysis/ directory may receive replay output')
    if out_dir.exists():
        raise ReplayRefused(f'{out_dir} already exists; use another --label (replays are never overwritten)')

    profile = None
    if profile_path is not None:
        profile = _parameters(_load_yaml(Path(profile_path), 'profile'), 'profile', Path(profile_path))
    recorded = _parameters(_load_yaml(settings, 'recorded settings'), 'recorded settings', settings)
    return ReplayPlan(
        run_dir=run,
        odom_frame=odom_frame,
        out_dir=out_dir,
        params=slam_parameters(recorded, odom_frame, profile),
        play_topics=[*PLAY_TOPICS, odom_topic],
        odom_topic=odom_topic,
        profile=profile,
    )


_RESULT_CODE = re.compile(r'Response\(result=(\d+)\)')


def service_result(output: str) -> int | None:
    """The result code in a `ros2 service call` answer, or None if the answer has no response line.

    For SaveMap, 0 is success, 1 is no map received and 255 an undefined failure. SerializePoseGraph
    uses 0 for success and 255 for a file it could not write (both in slam_toolbox's srv directory).
    """
    found = _RESULT_CODE.findall(output)
    return int(found[-1]) if found else None


def save_with_retries(save: Callable[[], None], saved: Callable[[], bool],
                      attempts: int = MAP_SAVE_ATTEMPTS, pause_s: float = MAP_SAVE_GAP_S,
                      sleep: Callable[[float], None] = time.sleep,
                      note: Callable[[str], None] = lambda message: None) -> int | None:
    """Call save() until saved() is true: at most `attempts` times, `pause_s` apart.

    saved() decides after every attempt, including one whose call raised, because a service can
    answer success and write nothing, or report an error after it has written. A ServiceError is
    a failed attempt. Any other exception, such as ReplayFailed for a child that has died, is not
    retried and propagates. Returns the attempt that worked, or None if none did.
    """
    if attempts < 1:
        raise ValueError('attempts must be at least 1')
    for attempt in range(1, attempts + 1):
        try:
            save()
            problem = 'it answered, but map.yaml is missing'
        except ServiceError as exc:
            problem = str(exc)
        if saved():
            return attempt
        note(f'save attempt {attempt} of {attempts} failed: {problem}')
        if attempt < attempts:
            sleep(pause_s)
    return None


# --- the command line --------------------------------------------------------------------------

def _positive_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f'not a number: {text!r}') from None
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError(f'must be a positive number: {text!r}')
    return value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Replay a recorded run through slam_toolbox with the car parked. '
                    'Refuses while any driver process runs.')
    parser.add_argument('run_dir', type=Path, help='a recorded run folder, e.g. ~/.ros/racerbot_runs/<ts>')
    parser.add_argument('--odom', choices=sorted(ODOM_TOPICS), default='odom',
                        help='the odometry to localise on: odom (wheels) or odom_imu (wheels and gyro)')
    parser.add_argument('--profile', type=Path, default=None,
                        help='YAML of slam_toolbox parameters laid over the recorded ones')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--rate', type=_positive_float, default=None, help='playback speed (default 1.0)')
    mode.add_argument('--exact', action='store_true',
                      help='replay with synchronous SLAM and recorded pose checkpoints')
    parser.add_argument('--exact-mode', choices=('forced', 'all-scans'), default='forced',
                        help='exact admission: forced (default) or the all-scans timing experiment')
    parser.add_argument('--out-root', type=Path, default=None, help='output parent (default ~/.ros/racerbot_runs)')
    parser.add_argument('--label', default='default', help='names the output folder (default "default")')
    return parser


# --- the replay: starts the ROS children, so it needs the ROS environment ----------------------

def _now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec='seconds')


class _Log:
    """A step goes to replay.log in the output folder, and to stderr so the operator can follow it."""

    def __init__(self, path: Path):
        self.path = path

    def __call__(self, message: str) -> None:
        with self.path.open('a', encoding='utf-8') as handle:
            handle.write(f'{_now()} {message}\n')
        print(message, file=sys.stderr, flush=True)


@dataclass
class _Child:
    name: str
    proc: subprocess.Popen
    handle: BinaryIO
    log_name: str


def _start(children: list[_Child], name: str, argv: list[str], env: dict[str, str],
           log_path: Path, log: _Log) -> _Child:
    """Start one child in a session of its own, so that the stop signals are ours to send."""
    log(f'start {name}: {shlex.join(argv)}')
    handle = log_path.open('wb')
    try:
        proc = subprocess.Popen(argv, env=env, stdin=subprocess.DEVNULL, stdout=handle,
                                stderr=subprocess.STDOUT, start_new_session=True)
    except OSError as exc:
        handle.close()
        raise ReplayFailed(f'could not start {name} ({argv[0]}): {exc}') from exc
    child = _Child(name, proc, handle, log_path.name)
    children.append(child)
    return child


def _check_alive(*children: _Child) -> None:
    for child in children:
        if child.proc.poll() is not None:
            raise ReplayFailed(f'{child.name} exited with {child.proc.returncode}; see {child.log_name}')


def _stop(child: _Child) -> int:
    """SIGINT, then SIGTERM, then SIGKILL, each given STOP_GRACE_S to act. Returns the exit code.

    The signal goes to the child's whole process group, so the nodes that ros2 launch started
    get it too, as they would from a terminal Ctrl-C.
    """
    proc = child.proc
    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            if proc.poll() is not None:
                break
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=STOP_GRACE_S)
            except subprocess.TimeoutExpired:
                pass
        return proc.wait()
    finally:
        child.handle.close()


def _topic_listed(topic: str, env: dict[str, str]) -> bool:
    try:
        done = subprocess.run(['ros2', 'topic', 'list'], env=env, capture_output=True, text=True,
                              timeout=SERVICE_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return topic in done.stdout.split()


def _wait_for_topic(topic: str, env: dict[str, str], child: _Child, timeout_s: float) -> bool:
    """True once `topic` is listed on the replay's graph. False on timeout, or once child has exited."""
    deadline = time.monotonic() + timeout_s
    while child.proc.poll() is None:
        if _topic_listed(topic, env):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(1.0)
    return False


def _nodes_listed(env: dict[str, str]) -> set[str] | None:
    """The names of the nodes on the replay's graph, or None if the ros2 CLI could not list them."""
    try:
        done = subprocess.run(['ros2', 'node', 'list'], env=env, capture_output=True, text=True,
                              timeout=SERVICE_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if done.returncode != 0:
        return None
    return set(done.stdout.split())


def _require_slam(slam: _Child, env: dict[str, str]) -> None:
    """Stop the replay unless slam_toolbox is still running.

    The launch process can outlive a node that has died, so the node itself is looked for too.
    """
    _check_alive(slam)
    nodes = _nodes_listed(env)
    if nodes is None:
        raise ReplayFailed('could not list the nodes on the replay graph; see replay.log')
    if '/slam_toolbox' not in nodes:
        raise ReplayFailed('/slam_toolbox is no longer on the replay graph; slam_toolbox has stopped, '
                           'see slam_toolbox.log')


def _service_call(service: tuple[str, str], request: dict, env: dict[str, str], log: _Log) -> None:
    name, srv_type = service
    try:
        done = subprocess.run(['ros2', 'service', 'call', name, srv_type, json.dumps(request)],
                              env=env, capture_output=True, text=True, timeout=SERVICE_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ServiceError(f'{name} did not answer: {exc}') from exc
    code = service_result(done.stdout)
    if code is not None:
        log(f'{name}: exit {done.returncode}; result {code}')
    else:
        log(f'{name}: exit {done.returncode}; {done.stdout.strip()[-200:]}')
    if done.returncode != 0:
        raise ServiceError(f'{name} failed with exit {done.returncode}')
    if code not in (None, 0):
        raise ServiceError(f'{name} answered with result {code}')


def _git_commit() -> str | None:
    try:
        done = subprocess.run(['git', '-C', str(REPO_ROOT), 'rev-parse', 'HEAD'],
                              capture_output=True, text=True, timeout=SERVICE_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if done.returncode != 0:
        return None
    return done.stdout.strip() or None


def _slam_version(env: dict[str, str]) -> str | None:
    try:
        done = subprocess.run(['ros2', 'pkg', 'xml', 'slam_toolbox', '--tag', 'version'],
                              env=env, capture_output=True, text=True, timeout=SERVICE_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if done.returncode != 0:
        return None
    return done.stdout.strip() or None


def _run(plan: ReplayPlan, rate: float | None, profile_path: Path | None, label: str,
         exact_mode: str = 'forced') -> int:
    out = plan.out_dir
    out.mkdir(parents=True)
    log = _Log(out / 'replay.log')
    started = _now()
    env = {**child_env(os.environ), 'PYTHONUNBUFFERED': '1'}  # so the logs are complete if a child dies
    git_commit = _git_commit()
    slam_version = _slam_version(env)
    exact = rate is None
    run_params, admission_overrides = plan.params, {}
    if exact:
        settings = plan.run_dir / 'metadata' / 'params' / 'slam_toolbox.yaml'
        recorded = _parameters(_load_yaml(settings, 'recorded settings'), 'recorded settings', settings)
        run_params, admission_overrides = exact_admission_parameters(plan.params, exact_mode, recorded)
    params_file = out / 'slam_params.yaml'
    file_params = loadable(run_params)
    params_file.write_text(yaml.safe_dump(file_params, sort_keys=False), encoding='utf-8')
    pace = f'exact mode {exact_mode}' if exact else f'rate {rate}'
    log(f'replaying {plan.run_dir} on {plan.odom_topic} (frame {plan.odom_frame}) at {pace}; '
        f'slam_toolbox {slam_version or "version unknown"}; slam_replay at {git_commit or "unknown commit"}')
    left_out = sorted(set(plan.params['slam_toolbox']['ros__parameters'])
                      - set(file_params['slam_toolbox']['ros__parameters']))
    if left_out:
        log(f'left out of slam_params.yaml, which cannot hold them (slam_toolbox keeps its default): '
            f'{", ".join(left_out)}')

    children: list[_Child] = []
    status, error = 'ok', None
    map_saved = posegraph_saved = False
    map_save_exhausted = False
    relay = None
    try:
        if not exact or exact_mode == 'all-scans':
            relay = _start(children, 'odom_tf_relay',
                           [sys.executable, str(RELAY), '--topic', plan.odom_topic,
                            '--ros-args', '-p', 'use_sim_time:=true'],
                           env, out / 'relay.log', log)
        exact_specs = exact_children(params_file, plan.run_dir / 'bag', plan.odom_topic, env) if exact else []
        slam_argv = exact_specs[0][1] if exact else ['ros2', 'launch', 'racerbot_launch', 'slam_launch.py',
                        f'slam_params_file:={params_file}', f'slam_tracking_file:={params_file}',
                        'use_sim_time:=true', f'slam_odom_frame:={plan.odom_frame}']
        slam = _start(children, 'slam_toolbox', slam_argv,
                      exact_specs[0][2] if exact else env, out / 'slam_toolbox.log', log)
        if exact:
            for transition in ('configure', 'activate'):
                deadline = time.monotonic() + MAP_WAIT_S
                while True:
                    _check_alive(slam)
                    done = subprocess.run(['ros2', 'lifecycle', 'set', '/slam_toolbox', transition],
                                          env=env, capture_output=True, text=True, timeout=SERVICE_TIMEOUT_S)
                    log(f'lifecycle {transition}: {done.stdout.strip()} {done.stderr.strip()}')
                    if done.returncode == 0 and 'Transitioning successful' in done.stdout:
                        break
                    if time.monotonic() >= deadline:
                        raise ReplayFailed(f'could not {transition} synchronous SLAM')
                    time.sleep(.2)
        recorder = _start(children, 'bag_record',
                          ['ros2', 'bag', 'record', '-o', str(out / 'bag'), '--storage', 'mcap',
                           '--use-sim-time', *RECORD_TOPICS],
                          env, out / 'bag_record.log', log)
        if not _wait_for_topic('/map', env, slam, MAP_WAIT_S):
            # Playing into a SLAM that never came up would spend the whole bag producing nothing.
            raise ReplayFailed(f'/map was not listed within {MAP_WAIT_S:g} s, so slam_toolbox did not come '
                               'up; see slam_toolbox.log')
        if relay is not None:
            _check_alive(relay)
        _check_alive(recorder)
        if exact:
            name, argv, feed_env = exact_specs[1]
            play = _start(children, name, [*argv, '--result', str(out / 'exact_feed.json'),
                                          '--slam-params', str(params_file), '--exact-mode', exact_mode],
                          feed_env, out / 'exact_feed.log', log)
        else:
            play = _start(children, 'bag_play',
                      ['ros2', 'bag', 'play', str(plan.run_dir / 'bag'), '--clock', '--rate', str(rate),
                       '--topics', *plan.play_topics],
                      env, out / 'bag_play.log', log)
        code = play.proc.wait()
        if code != 0:
            details = (out / play.log_name).read_text(encoding='utf-8').splitlines()
            last_line = details[-1] if details else 'no output'
            raise ReplayFailed(f'{play.name} exited with {code}: {last_line}; see {play.log_name}')
        log(f'bag played; giving slam_toolbox {SETTLE_S:g} s to finish its queue')
        time.sleep(SETTLE_S)

        def save_once() -> None:
            _require_slam(slam, env)
            _service_call(SAVE_MAP, {'name': {'data': str(out / 'map')}}, env, log)

        attempt = save_with_retries(save_once, saved=lambda: (out / 'map.yaml').is_file(), note=log)
        map_saved = attempt is not None
        if map_saved:
            log(f'map saved on attempt {attempt}')
        else:
            map_save_exhausted = True
            log(f'map saver exhausted {MAP_SAVE_ATTEMPTS} attempts; will save the last /map '
                'after the recorder has flushed its bag')
        _require_slam(slam, env)
        try:
            _service_call(SERIALIZE_MAP, {'filename': str(out / 'posegraph')}, env, log)
        except ServiceError as exc:
            log(f'WARNING: the pose graph call did not succeed: {exc}')
        posegraph_saved = (out / 'posegraph.posegraph').is_file()
        if not posegraph_saved:
            log('WARNING: posegraph.posegraph was not written; the replay goes on without it')
    except ReplayFailed as exc:
        status, error = 'failed', str(exc)
        log(f'failed: {error}')
    except KeyboardInterrupt:
        status, error = 'interrupted', 'interrupted by the operator'
        log('interrupted; stopping the children')
    except Exception as exc:
        status, error = 'error', repr(exc)
        raise
    finally:
        for child in reversed(children):
            log(f'stopped {child.name} with exit {_stop(child)}')
        if map_save_exhausted and (out / 'bag' / 'metadata.yaml').is_file():
            map_saved = save_map_from_bag(out / 'bag', out)
            log('map saved from the replay bag' if map_saved else 'WARNING: replay bag has no /map to save')
        log(f'replay {status}: {out}')
        summary = {
            'mode': 'exact' if exact else 'rate',
            'status': status, 'error': error, 'label': label,
            'source_run': str(plan.run_dir), 'out_dir': str(out),
            'odom_frame': plan.odom_frame, 'odom_topic': plan.odom_topic,
            'play_topics': ['/scan', '/tf_static', '/tf'] if exact and exact_mode == 'forced'
                           else plan.play_topics, 'rate': rate,
            'profile_path': str(profile_path) if profile_path is not None else None,
            'profile': plan.profile,
            'git_commit': git_commit, 'slam_toolbox_version': slam_version,
            'map_saved': map_saved, 'posegraph_saved': posegraph_saved,
            'started': started, 'ended': _now(),
        }
        if exact and (out / 'bag' / 'metadata.yaml').exists():
            summary['pose_sequence_error'] = pose_sequence_error(
                bag_pose_rows(plan.run_dir / 'bag'), bag_pose_rows(out / 'bag'))
        if exact:
            feed_result = out / 'exact_feed.json'
            summary['exact'] = json.loads(feed_result.read_text()) if feed_result.exists() else {
                'lookahead_samples': 0}
            summary['exact']['feed_mode'] = exact_mode
            summary['exact']['admission_overrides'] = admission_overrides
            summary['exact']['tf_source'] = 'recorded' if exact_mode == 'forced' else 'odom_relay'
        (out / 'replay.json').write_text(json.dumps(summary, indent=2, default=str) + '\n', encoding='utf-8')
        if (plan.run_dir / 'metadata').is_dir():
            shutil.copytree(plan.run_dir / 'metadata', out / 'metadata')
    if status == 'ok':
        print(out)
        return 0
    return 1


def main(argv=None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.exact_mode == 'all-scans' and not args.exact:
        parser.error('--exact-mode all-scans requires --exact')
    drivers = running_drivers(PROC_ROOT)
    if drivers:
        print(f'refusing: driver processes are running ({", ".join(drivers)}). '
              'Stop bringup first, so the car is parked.', file=sys.stderr)
        return 2
    try:
        plan = plan_replay(args.run_dir, args.odom, args.profile,
                           args.out_root if args.out_root is not None else RUNS_ROOT, args.label)
        if args.exact and plan.profile and any(
                key in plan.profile for key in ('mode', 'sync', 'async', 'synchronous', 'slam_mode')):
            raise ReplayRefused('--exact does not allow profile overrides of sync/async mode')
        if args.exact:
            scan_plan = bag_exact_plan(plan.run_dir / 'bag')
            if args.exact_mode == 'forced':
                source_scans = [stamp_ns(msg) for _, msg in bag_messages(plan.run_dir / 'bag', ['/scan'])]
                feed_plan = forced_scan_plan(source_scans, scan_plan)
                scan_plan = [stamp for stamp, _ in feed_plan]
                tf_messages = [msg for _, msg in bag_messages(plan.run_dir / 'bag', ['/tf'])]
                statics = [tf for _, msg in bag_messages(plan.run_dir / 'bag', ['/tf_static'])
                           for tf in msg.transforms]
                prepare_recorded_tf(tf_messages, statics, plan.odom_frame,
                                    plan.params['slam_toolbox']['ros__parameters'].get(
                                        'base_frame', 'base_footprint'), scan_plan)
            else:
                odometry_brackets(sorted(stamp_ns(msg) for _, msg in
                                  bag_messages(plan.run_dir / 'bag', [plan.odom_topic])), scan_plan)
    except ReplayRefused as exc:
        print(f'refusing: {exc}', file=sys.stderr)
        return 2
    if args.exact:
        return _run(plan, None, args.profile, args.label, args.exact_mode)
    return _run(plan, args.rate if args.rate is not None else 1.0, args.profile, args.label)


if __name__ == '__main__':
    sys.exit(main())
