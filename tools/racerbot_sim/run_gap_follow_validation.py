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
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

CLEANUP_PATTERNS = (
    'racerbot_sim/lib', 'ackermann_mux/lib', 'tf2_ros/static_transform_publisher',
    'gap_follow/lib',
)


def stop_everything():
    for pattern in CLEANUP_PATTERNS:
        subprocess.run(['pkill', '-f', pattern], capture_output=True)
    time.sleep(1.0)
    for pattern in CLEANUP_PATTERNS:
        subprocess.run(['pkill', '-9', '-f', pattern], capture_output=True)
    time.sleep(1.0)


class Launch:
    """One `ros2 launch` process, with its own log."""

    def __init__(self, log_path: Path, package: str, launch_file: str, arguments: list):
        self.log_path = log_path
        self.args = ['ros2', 'launch', package, launch_file] + arguments
        self.process = None
        self.handle = None

    def start(self):
        self.handle = open(self.log_path, 'w')
        self.process = subprocess.Popen(
            self.args, stdout=self.handle, stderr=subprocess.STDOUT,
            cwd=str(ROOT), start_new_session=True)

    def text(self) -> str:
        try:
            return self.log_path.read_text(errors='replace')
        except OSError:
            return ''

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                self.process.wait(timeout=15)
            except (subprocess.TimeoutExpired, ProcessLookupError, PermissionError):
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
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
    log_dir = Path(args.log_directory)
    log_dir.mkdir(parents=True, exist_ok=True)

    bringup = Launch(log_dir / 'bringup.log', 'racerbot_sim', 'sim_bringup_launch.py', [
        f'track:={args.track}',
        f'seed:={args.seed}',
        f'hold_deadman:={"true" if args.hold_deadman else "false"}',
    ])
    controller = Launch(log_dir / 'gap_follow.log', 'gap_follow', 'gap_follow_launch.py', [])

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
    try:
        bringup.start()
        controller.start()
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
            time.sleep(1.0)

        status = sim_status()
    finally:
        controller.stop()
        bringup.stop()
        stop_everything()

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


def main(argv=None) -> int:
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
    args = parser.parse_args(argv)

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
