"""Replay a recorded run through slam_toolbox with the car parked, to tune SLAM without driving.

    python3 tools/run_analysis/slam_replay.py ~/.ros/racerbot_runs/<ts> \
        [--odom odom|odom_imu] [--profile FILE] [--rate R] [--label NAME]

The run's bag is played into a slam_toolbox started from the run's own recorded settings, with
the profile's overrides laid over them. Only /scan, /tf_static and one odometry topic are played,
never the recorded /tf, and odom_tf_relay.py rebuilds the odometry transform from that topic.

Refuses while a driver process is alive. DRIVER_MARKERS are matched against every process's whole
command line, so a driver started under another name still counts. Every child runs on the
replay's own ROS domain (REPLAY_DOMAIN_ID), so a replay never joins the car's graph.

plan_replay and the functions it calls do no ROS and import without rclpy. A replay starts ros2 and
odom_tf_relay.py, so it needs the ROS environment sourced (ros2 on PATH).
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

    out_dir = Path(out_root) / f'{run.name}-replay-{odom_frame}-{label}'
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
    parser.add_argument('--rate', type=_positive_float, default=1.0, help='playback speed (default 1.0)')
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


def _run(plan: ReplayPlan, rate: float, profile_path: Path | None, label: str) -> int:
    out = plan.out_dir
    out.mkdir(parents=True)
    log = _Log(out / 'replay.log')
    started = _now()
    env = {**child_env(os.environ), 'PYTHONUNBUFFERED': '1'}  # so the logs are complete if a child dies
    git_commit = _git_commit()
    slam_version = _slam_version(env)
    params_file = out / 'slam_params.yaml'
    file_params = loadable(plan.params)
    params_file.write_text(yaml.safe_dump(file_params, sort_keys=False), encoding='utf-8')
    log(f'replaying {plan.run_dir} on {plan.odom_topic} (frame {plan.odom_frame}) at rate {rate}; '
        f'slam_toolbox {slam_version or "version unknown"}; slam_replay at {git_commit or "unknown commit"}')
    left_out = sorted(set(plan.params['slam_toolbox']['ros__parameters'])
                      - set(file_params['slam_toolbox']['ros__parameters']))
    if left_out:
        log(f'left out of slam_params.yaml, which cannot hold them (slam_toolbox keeps its default): '
            f'{", ".join(left_out)}')

    children: list[_Child] = []
    status, error = 'ok', None
    map_saved = posegraph_saved = False
    try:
        relay = _start(children, 'odom_tf_relay',
                       [sys.executable, str(RELAY), '--topic', plan.odom_topic,
                        '--ros-args', '-p', 'use_sim_time:=true'],
                       env, out / 'relay.log', log)
        slam = _start(children, 'slam_toolbox',
                      ['ros2', 'launch', 'racerbot_launch', 'slam_launch.py',
                       f'slam_params_file:={params_file}', f'slam_tracking_file:={params_file}',
                       'use_sim_time:=true', f'slam_odom_frame:={plan.odom_frame}'],
                      env, out / 'slam_toolbox.log', log)
        recorder = _start(children, 'bag_record',
                          ['ros2', 'bag', 'record', '-o', str(out / 'bag'), '--storage', 'mcap',
                           '--use-sim-time', *RECORD_TOPICS],
                          env, out / 'bag_record.log', log)
        if not _wait_for_topic('/map', env, slam, MAP_WAIT_S):
            # Playing into a SLAM that never came up would spend the whole bag producing nothing.
            raise ReplayFailed(f'/map was not listed within {MAP_WAIT_S:g} s, so slam_toolbox did not come '
                               'up; see slam_toolbox.log')
        _check_alive(relay, recorder)
        play = _start(children, 'bag_play',
                      ['ros2', 'bag', 'play', str(plan.run_dir / 'bag'), '--clock', '--rate', str(rate),
                       '--topics', *plan.play_topics],
                      env, out / 'bag_play.log', log)
        code = play.proc.wait()
        if code != 0:
            raise ReplayFailed(f'ros2 bag play exited with {code}; see bag_play.log')
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
            log(f'WARNING: map.yaml was not written in {MAP_SAVE_ATTEMPTS} attempts; the replay goes on '
                'without it, and the map is still on /map in the bag')
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
        log(f'replay {status}: {out}')
        summary = {
            'status': status, 'error': error, 'label': label,
            'source_run': str(plan.run_dir), 'out_dir': str(out),
            'odom_frame': plan.odom_frame, 'odom_topic': plan.odom_topic,
            'play_topics': plan.play_topics, 'rate': rate,
            'profile_path': str(profile_path) if profile_path is not None else None,
            'profile': plan.profile,
            'git_commit': git_commit, 'slam_toolbox_version': slam_version,
            'map_saved': map_saved, 'posegraph_saved': posegraph_saved,
            'started': started, 'ended': _now(),
        }
        (out / 'replay.json').write_text(json.dumps(summary, indent=2, default=str) + '\n', encoding='utf-8')
        if (plan.run_dir / 'metadata').is_dir():
            shutil.copytree(plan.run_dir / 'metadata', out / 'metadata')
    if status == 'ok':
        print(out)
        return 0
    return 1


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    drivers = running_drivers(PROC_ROOT)
    if drivers:
        print(f'refusing: driver processes are running ({", ".join(drivers)}). '
              'Stop bringup first, so the car is parked.', file=sys.stderr)
        return 2
    try:
        plan = plan_replay(args.run_dir, args.odom, args.profile, RUNS_ROOT, args.label)
    except ReplayRefused as exc:
        print(f'refusing: {exc}', file=sys.stderr)
        return 2
    return _run(plan, args.rate, args.profile, args.label)


if __name__ == '__main__':
    sys.exit(main())
