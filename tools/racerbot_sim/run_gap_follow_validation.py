#!/usr/bin/env python3
"""Direct validation of gap_follow against a racerbot_sim track -- no SLAM,
no recorded line, no supervisor. gap_follow is purely reactive (LiDAR gap
selection, no map), so it's the one controller that can be driven straight
at a known track and judged the same way run_auto_map_validation.py judges
the racing phase of the automatic pipeline: no wall contact, real distance
covered.

`run_auto_map_validation.py` never exercises gap_follow at all -- it isn't
part of the auto_map_race supervisor flow, only the mapping half of it (as
`/auto_map/drive`, superseded the instant the supervisor hands off). This
is the only way to answer "how does gap_follow itself do on this track" in
the ROS-level simulator.

pure_pursuit has no equivalent script here: it needs a localized pose in
the map frame, not a track. The automatic pipeline supplies that live from
slam_toolbox; `run_auto_map_validation.py --track <name>` is the direct way
to test pure_pursuit against a specific track. Routing it through
particle_filter instead (`racerbot_launch/race_launch.py`, the manual/
saved-map workflow) does not currently work against this simulator:
`particle_filter/launch/localize_launch.py` hardcodes `use_sim_time: true`
on its `nav2_map_server`/`nav2_lifecycle_manager` nodes, and racerbot_sim
deliberately publishes no `/clock` (see docs/ros-simulator.md -- every node
here runs on its own wall-clock timers, same as the car). Fixing that is a
localization-stack change, not a track-validation script; out of scope
here.

    tools/racerbot_sim/run_gap_follow_validation.py --track asb_10000

Exits 0 only if the run passes. Needs a built, sourced workspace and the
F1TENTH Gym from tools/f1tenth_sim/setup.sh, same as
run_auto_map_validation.py.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import harness_procs

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'racerbot_sim'))
sys.path.insert(0, str(ROOT / 'src' / 'imu_bridge'))
from racerbot_sim.sim_imu import FAULTS
from racerbot_sim.hardware_guard import conflicting_hardware_nodes

FEATURES = {
    'steering_damping': 'steering_damping_off',
    'fused_speed': 'fused_speed_imu',
    'understeer': 'imu_understeer_off',
    'traction': 'imu_traction_off',
    'impact_stop': 'imu_impact_stop_off',
}


def process_snapshot(proc_root=Path('/proc')):
    """Read local process parents and bridge executables; fail closed on errors."""
    parents, bridges = {}, []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / 'stat').read_text()
            # comm is parenthesised and can contain whitespace or ')'.
            fields = stat[stat.rindex(')') + 1:].split()
            ppid = int(fields[1])
            argv = (entry / 'cmdline').read_bytes().split(b'\0')
        except FileNotFoundError:
            continue  # process exited while inspecting it
        pid = int(entry.name)
        parents[pid] = ppid
        if any(Path(os.fsdecode(arg)).name == 'gym_bridge_node' for arg in argv[:2] if arg):
            bridges.append(pid)
    return sorted(bridges), parents


def injection_guard(hardware, bridges, launch_pid, parents):
    """Return a refusal reason unless the single bridge belongs to the launch."""
    if hardware is None:
        return 'cannot read hardware graph'
    if hardware:
        return f'real hardware detected: {", ".join(hardware)}'
    if len(bridges) != 1:
        return f'expected exactly one gym_bridge_node process, found {len(bridges)}'
    pid, seen = bridges[0], set()
    while pid != launch_pid and pid not in seen and pid in parents:
        seen.add(pid)
        pid = parents[pid]
    if pid != launch_pid or bridges[0] == launch_pid:
        return 'gym_bridge_node is not a descendant of this launch'
    return None


def check_injection_target(node, launch_pid, proc_root=Path('/proc')):
    hardware = conflicting_hardware_nodes(node)
    if hardware is None or hardware:
        return injection_guard(hardware, [], launch_pid, {})
    try:
        nodes = node.get_node_names_and_namespaces()
        bridges, parents = process_snapshot(proc_root)
    except Exception:  # noqa: BLE001 -- unknown graph/process state must refuse
        return 'cannot inspect bridge processes or ROS graph'
    if sum(name == 'gym_bridge_node' for name, _ in nodes) != 1 or (
            'gym_bridge_node', '/') not in nodes:
        return 'expected exactly one root /gym_bridge_node on the ROS graph'
    return injection_guard(hardware, bridges, launch_pid, parents)


def write_fault(fault, env):
    """Return an error for any parameter-client failure, or None on success."""
    try:
        completed = subprocess.run(
            ['ros2', 'param', 'set', '/gym_bridge_node', 'imu_fault', fault],
            capture_output=True, text=True, timeout=15, cwd=str(ROOT), env=env)
    except subprocess.TimeoutExpired:
        return 'parameter client timed out'
    except Exception as exc:  # noqa: BLE001 -- still clean up and report failures
        return f'parameter client error: {exc}'
    if completed.returncode or 'successful' not in completed.stdout.lower():
        return f'parameter client failed: {completed.stdout} {completed.stderr}'
    return None


def inject_fault(node, launch_pid, fault, env, proc_root=Path('/proc')):
    refusal = check_injection_target(node, launch_pid, proc_root)
    if refusal:
        return refusal
    return write_fault(fault, env)


def gap_follow_launch_args(requested):
    """The gap_follow launch arguments, with the simulated IMU selected.

    imu_hardware:=auto probes for gym_bridge_node when the launch starts and
    can lose that race against the simulator's own start-up (it then picks
    the real IMU, and the interlock refuses). This harness only ever drives
    the simulator, so it says so explicitly; an explicit imu_hardware:= wins.
    """
    args = list(requested or [])
    if not any(arg.split(':=', 1)[0] == 'imu_hardware' for arg in args):
        args.append('imu_hardware:=false')
    return args


def injection_environment(harness_env):
    env = dict(harness_env)
    env.pop('ROS_STATIC_PEERS', None)
    env['ROS_DOMAIN_ID'] = harness_env.get('ROS_DOMAIN_ID', '0')
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    return env


def injection_environment_refusal(harness_env, client_env):
    """Bind the service target to the owned local bridge before a write.

    The launches and graph-inspecting node must use LOCALHOST discovery on
    the same ROS domain as the LOCALHOST parameter client. Static peers can
    reach remote hosts even with LOCALHOST, so the harness must have no
    non-empty ROS_STATIC_PEERS and the client environment removes it. On
    that local graph, the single root bridge name and single local bridge
    process, checked by inject_fault, refer to the harness-owned descendant.
    """
    if (harness_env.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
            or client_env.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        return 'fault injection requires LOCALHOST discovery for harness and client'
    if harness_env.get('ROS_DOMAIN_ID', '0') != client_env.get('ROS_DOMAIN_ID', '0'):
        return 'fault injection refused: ROS domain mismatch'
    if harness_env.get('ROS_STATIC_PEERS'):
        return 'fault injection refused: ROS_STATIC_PEERS is non-empty'
    return None


def inject_with_environment(node, launch_pid, fault, harness_env, client_env):
    refusal = injection_environment_refusal(harness_env, client_env)
    if refusal:
        return refusal
    return inject_fault(node, launch_pid, fault, client_env)


def injection_failure(fault, injected_at, fatal):
    if fatal:
        return fatal
    if fault and injected_at is None:
        return 'fault injection failed: injection was not acknowledged'
    return None


def validation_split(fault, requested, injected_at):
    return injected_at if fault else requested


def reason_report(log, fault_at, fault):
    expected = 'imu stale' if fault == 'stale' else 'imu fault'
    eligible = [reason for t, reason in log
                if t >= fault_at + 0.5 and isinstance(reason, str) and reason.strip()]
    first = next(((t, reason) for t, reason in log
                  if t >= fault_at and isinstance(reason, str) and reason.strip()), None)
    fractions, missing = {}, {}
    for feature, label in FEATURES.items():
        value = expected.removeprefix('imu ') if feature == 'fused_speed' else expected
        absent = [r for r in eligible if f'{label}: {value}' not in r]
        fractions[feature] = (1 - len(absent) / len(eligible)) if eligible else None
        missing[feature] = {'count': len(absent), 'first_three': absent[:3]}
    return {'fallback_fractions': fractions, 'reason_messages_after_settle': len(eligible),
            'first_reason_after_fault': ({'time_s': round(first[0], 3), 'text': first[1]}
                                         if first else None),
            'missing_fallback': missing}


def fallback_fractions(log, fault_at, fault):
    return reason_report(log, fault_at, fault)['fallback_fractions']


def speed_summary(samples, split):
    def means(rows):
        def mean(values):
            values = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
            return sum(values) / len(values) if values else None
        return {'mean_speed': mean([s for _, s, _ in rows]),
                'mean_commanded_speed': mean([c for _, _, c in rows])}
    return {'before': means([x for x in samples if x[0] < split]),
            'after': means([x for x in samples if x[0] >= split])}


class Recorder:
    """Read-only topic capture; calculation happens after the drive ends."""
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import String
        from sensor_msgs.msg import Imu
        from nav_msgs.msg import Odometry
        rclpy.init()
        self.rclpy = rclpy
        self.node = Node('gap_validation_recorder')
        self.intent, self.status, self.imu, self.odom, self.sim = [], [], [], [], []
        self.node.create_subscription(String, '/drive_intent', self._intent, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, '/imu/status', self._status, qos)
        self.node.create_subscription(Imu, '/imu/data', self._imu, 10)
        self.node.create_subscription(Odometry, '/odom', self._odom, 10)
        self.node.create_subscription(String, '/sim/status', self._sim, 10)
        self.running = True
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        while self.running:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def _intent(self, msg):
        try:
            data = json.loads(msg.data)
            self.intent.append((time.monotonic(), data.get('reason'), data.get('commanded_speed')))
        except (ValueError, TypeError):
            return

    def _status(self, msg):
        self.status.append((time.monotonic(), msg.data))

    def _imu(self, msg):
        self.imu.append((time.monotonic(), msg.linear_acceleration.z, msg.angular_velocity.z))

    def _odom(self, msg):
        self.odom.append((time.monotonic(), msg.twist.twist.linear.x))

    def _sim(self, msg):
        try:
            self.sim.append((time.monotonic(), json.loads(msg.data)))
        except (ValueError, TypeError):
            return

    def stop(self):
        self.running = False
        self.thread.join()
        self.node.destroy_node()
        self.rclpy.shutdown()


def recorder_report(rec, start, split, fault):
    intents = [(t - start, reason) for t, reason, _ in rec.intent if t >= start]
    # Pair latest commanded speed with each odometry sample; ignore pre-drive data.
    commands = [(t, speed) for t, _, speed in rec.intent if t >= start]
    speeds, index, command = [], 0, None
    for t, speed in rec.odom:
        if t < start:
            continue
        while index < len(commands) and commands[index][0] <= t:
            command = commands[index][1]
            index += 1
        speeds.append((t - start, speed, command))
    before = [s for t, s in rec.sim if t <= start + split]
    after = [s for t, s in rec.sim if t >= start + split]
    distance = (after[-1]['distance_travelled_m'] - before[-1]['distance_travelled_m']
                if before and after else None)
    # Include later samples: the parameter request and the simulator tick are
    # asynchronous, and a stale fault stops the stream altogether.
    imu_before = [x for x in rec.imu if x[0] < start + split]
    imu_after = [x for x in rec.imu if x[0] >= start + split]
    later = [x for x in imu_after if x[0] >= start + split + 1.0]
    evidence = imu_before[-1:] + imu_after[:3] + later[:1]
    if imu_after and imu_after[-1] not in evidence:
        evidence.append(imu_after[-1])
    stopped = 0
    was_stopped = False
    for _, speed, _ in speeds:
        is_stopped = math.isfinite(speed) and abs(speed) < 0.05
        if is_stopped and not was_stopped:
            stopped += 1
        was_stopped = is_stopped
    return {'fault': fault, 'fault_at_s': split,
            **reason_report(intents, split, fault),
            'intent_messages_after_settle': sum(t >= split + 0.5 for t, _ in intents),
            'speeds': speed_summary(speeds, split), 'distance_after_m': distance,
            'stops': stopped,
            'imu_evidence': [[round(t - start, 3), z, yaw] for t, z, yaw in evidence],
            'status_evidence': [[round(t - start, 3), s] for t, s in rec.status]}

CLEANUP_PATTERNS = (
    'racerbot_sim/lib', 'ackermann_mux/lib', 'tf2_ros/static_transform_publisher',
    'gap_follow/lib',
)


def stop_everything():
    """Refuse to start beside anything matching CLEANUP_PATTERNS.

    Never kills: those patterns match the real car's installed stack as well
    as a stale simulator. Each launch below cleans up its own process group;
    see harness_procs.
    """
    harness_procs.refuse_if_strays(CLEANUP_PATTERNS)


class Launch:
    """One `ros2 launch` process, with its own log."""

    def __init__(self, log_path: Path, package: str, launch_file: str, arguments: list):
        self.log_path = log_path
        self.args = ['ros2', 'launch', package, launch_file] + arguments
        self.process = None
        self.handle = None

    def start(self, env=None):
        self.handle = open(self.log_path, 'w')
        self.process = subprocess.Popen(
            self.args, stdout=self.handle, stderr=subprocess.STDOUT,
            cwd=str(ROOT), start_new_session=True, env=env)

    def text(self) -> str:
        try:
            return self.log_path.read_text(errors='replace')
        except OSError:
            return ''

    def stop(self):
        harness_procs.stop_group(self.process)
        if self.handle is not None:
            self.handle.close()


def sim_status() -> dict:
    try:
        completed = subprocess.run(
            ['ros2', 'topic', 'echo', '/sim/status', '--once', '--field', 'data'],
            capture_output=True, text=True, timeout=15, cwd=str(ROOT))
    except subprocess.TimeoutExpired:
        return {}
    for line in completed.stdout.splitlines():
        line = line.strip()
        if line.startswith('{'):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
    return {}


FATAL_PATTERNS = (
    'process has died',
    'Traceback (most recent call last)',
)


def run(args) -> dict:
    stop_everything()
    harness_env = os.environ.copy()
    injection_env = injection_environment(harness_env)
    log_dir = Path(args.log_directory)
    log_dir.mkdir(parents=True, exist_ok=True)

    bringup = Launch(log_dir / 'bringup.log', 'racerbot_sim', 'sim_bringup_launch.py', [
        f'track:={args.track}',
        f'seed:={args.seed}',
        f'hold_deadman:={"true" if args.hold_deadman else "false"}',
        'imu_fault:=none',
    ])
    controller = Launch(log_dir / 'gap_follow.log', 'gap_follow', 'gap_follow_launch.py',
                        gap_follow_launch_args(args.gap_follow_arg))

    result = {
        'controller': 'gap_follow',
        'track': args.track,
        'seed': args.seed,
        'passed': False,
        'failure': None,
    }

    started = time.monotonic()
    fatal = None
    driving_since = None
    distance_at_start = None
    recorder = Recorder() if args.fault_at is not None else None
    injected_at = None
    try:
        bringup.start(env=harness_env)
        controller.start(env=harness_env)
        while True:
            elapsed = time.monotonic() - started
            for launch in (bringup, controller):
                text = launch.text()
                if any(pattern in text for pattern in FATAL_PATTERNS):
                    fatal = f'{launch.args[2]} crashed or raised'
            if fatal:
                break
            for launch in (bringup, controller):
                if launch.process.poll() is not None:
                    fatal = f'{launch.args[2]} exited early'
            if fatal:
                break

            status = sim_status()
            if driving_since is None:
                # "Driving" once a fresh, non-zero command has actually reached
                # the bridge -- not just once the graph is up, which can be
                # true well before the deadman/mux chain lets a command through.
                commanded = status.get('commanded') or {}
                if commanded.get('fresh') and abs(commanded.get('speed') or 0.0) > 0.01:
                    driving_since = time.monotonic()
                    distance_at_start = float(status.get('distance_travelled_m') or 0.0)
                elif elapsed >= args.startup_timeout:
                    fatal = f'never started driving within {args.startup_timeout:.0f}s'
                    break
            else:
                if time.monotonic() - driving_since >= args.duration:
                    break
                if (args.imu_fault and injected_at is None
                        and time.monotonic() - driving_since >= args.fault_at):
                    error = (injection_environment_refusal(os.environ, injection_env)
                             or inject_with_environment(recorder.node, bringup.process.pid,
                                                        args.imu_fault, harness_env, injection_env))
                    if error:
                        fatal = f'fault injection failed: {error}'
                        break
                    injected_at = time.monotonic() - driving_since
            time.sleep(1.0)

        status = sim_status()
    finally:
        if recorder:
            recorder.stop()
        controller.stop()
        bringup.stop()

    result['sim'] = {
        key: status.get(key) for key in
        ('sim_time_s', 'distance_travelled_m', 'wall_contact_steps', 'ever_contacted')
    }
    driven_metres = (
        None if distance_at_start is None
        else round(float(status.get('distance_travelled_m') or 0.0) - distance_at_start, 2))
    result['driven_metres'] = driven_metres
    result['driving_seconds'] = (
        round(time.monotonic() - driving_since, 1) if driving_since else 0.0)
    if recorder and driving_since is not None:
        split = validation_split(args.imu_fault, args.fault_at, injected_at)
        result['imu_validation'] = (recorder_report(recorder, driving_since, split, args.imu_fault)
                                    if split is not None else
                                    {'fault': args.imu_fault, 'fault_at_s': None})
        result['imu_validation']['injected_at_s'] = injected_at

    fatal = injection_failure(args.imu_fault, injected_at, fatal)
    if fatal:
        result['failure'] = fatal
    elif status.get('ever_contacted'):
        result['failure'] = (
            f"hit a wall ({status.get('wall_contact_steps')} steps in contact)")
    elif driven_metres is not None and driven_metres < args.min_distance:
        result['failure'] = (
            f'only covered {driven_metres:.1f}m in {result["driving_seconds"]:.0f}s, '
            f'under the {args.min_distance:.1f}m this run needs')
    else:
        result['passed'] = True

    return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--track', default='indoor_wide',
                        help='indoor_oval, indoor_tight, indoor_wide or asb_10000.')
    parser.add_argument('--seed', type=int, default=12345)
    parser.add_argument('--hold-deadman', action='store_true', default=True)
    parser.add_argument('--startup-timeout', type=float, default=30.0,
                        help='seconds to wait for the first real drive command')
    parser.add_argument('--duration', type=float, default=90.0,
                        help='seconds to watch once driving starts')
    parser.add_argument('--min-distance', type=float, default=20.0,
                        help='metres the car must cover in --duration to pass')
    parser.add_argument('--log-directory',
                        default=str(ROOT / '.sim' / 'gap_follow_logs'))
    parser.add_argument('--output', help='write the JSON report here')
    parser.add_argument('--gap-follow-arg', action='append', default=[], metavar='NAME:=VALUE')
    parser.add_argument('--imu-fault', choices=FAULTS)
    parser.add_argument('--fault-at', type=float)
    args = parser.parse_args(argv)
    if args.imu_fault and args.fault_at is None:
        parser.error('--imu-fault requires --fault-at')
    if args.fault_at is not None and (not math.isfinite(args.fault_at)
                                      or not math.isfinite(args.duration)
                                      or not 0 < args.fault_at < args.duration):
        parser.error('--fault-at must be finite and strictly between 0 and --duration')
    if any(':=' not in arg or not arg.split(':=', 1)[0] for arg in args.gap_follow_arg):
        parser.error('--gap-follow-arg must be NAME:=VALUE')
    return args


def main(argv=None) -> int:
    args = parse_args(argv)

    print(f'--- gap_follow on {args.track} ---', flush=True)
    result = run(args)
    print(json.dumps(result, indent=2), flush=True)
    print(f"gap_follow on {args.track}: {'PASS' if result['passed'] else 'FAIL'}"
          f"{'' if result['passed'] else ' -- ' + str(result['failure'])}", flush=True)

    if args.output:
        Path(args.output).write_text(json.dumps(result, indent=2) + '\n')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
