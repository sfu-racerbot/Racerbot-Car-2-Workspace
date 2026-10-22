#!/usr/bin/env python3
"""End-to-end check of the one-command mapping flow against the simulator.

The two commands docs/launchstart.md tells people to run, for real, over
racerbot_sim in place of the hardware:

1. ``ros2 run racerbot_launch map_course`` (sim_bringup_launch.py as the
   hardware) -- must
   drive its laps, save map.yaml and raceline_profiled.csv, write
   commands.txt, exit by itself, and print the commands last.
2. The race command it printed, on the same simulator with the car left
   where mapping stopped it (as the doc says) -- must confirm the start
   pose, then drive at least ``--min-race-distance`` metres with no wall or
   car contact.

Each steering law gets its own fresh simulator and mapping run, because a
race leaves the car somewhere else.

    tools/racerbot_sim/run_map_course_validation.py
    tools/racerbot_sim/run_map_course_validation.py --track indoor_wide --laws pure_pursuit,mpc

Run it on its own ROS domain, never beside the car's real drivers
(CLAUDE.md): this script exports ROS_DOMAIN_ID=79 and LOCALHOST discovery for
everything it starts unless they are already set. Exits 0 only if every step
passes. Needs a built, sourced workspace and tools/f1tenth_sim/setup.sh.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import harness_procs

ROOT = Path(__file__).resolve().parents[2]
CLEANUP_PATTERNS = (
    'racerbot_sim/lib', 'ackermann_mux/lib', 'tf2_ros/static_transform_publisher',
    'gap_follow/lib', 'pure_pursuit/lib', 'async_slam_toolbox_node',
    'particle_filter', 'map_server',
)


def _env():
    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', '79')
    env.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    return env


def _start(cmd, log_path):
    handle = open(log_path, 'w')
    process = subprocess.Popen(cmd, stdout=handle, stderr=subprocess.STDOUT,
                               cwd=str(ROOT), env=_env(), start_new_session=True)
    return process, handle


def _stop(process, handle):
    harness_procs.stop_group(process)
    handle.close()


def sim_status() -> dict:
    try:
        done = subprocess.run(
            ['ros2', 'topic', 'echo', '/sim/status', '--once', '--field', 'data'],
            capture_output=True, text=True, timeout=15, cwd=str(ROOT), env=_env())
    except subprocess.TimeoutExpired:
        return {}
    for line in done.stdout.splitlines():
        line = line.strip()
        if line.startswith('{'):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
    return {}


def _contact(status) -> str | None:
    if status.get('ever_contacted'):
        return f"hit a wall ({status.get('wall_contact_steps')} steps)"
    if status.get('car_contact_steps'):
        return f"cars touched ({status.get('car_contact_steps')} steps)"
    return None


def _map(args, logs: Path, out: Path, tag: str) -> dict:
    """map_course on the running simulator. Returns the step result."""
    result = {'step': f'map_course ({tag})', 'passed': False}
    try:
        done = subprocess.run(
            ['ros2', 'run', 'racerbot_launch', 'map_course', 'diagnostics:=false', f'laps:={args.laps}', f'output_directory:={out}'],
            capture_output=True, text=True, timeout=args.map_timeout, cwd=str(ROOT),
            env=_env(), start_new_session=True)
    except subprocess.TimeoutExpired as exc:
        (logs / f'map_course_{tag}.log').write_text(str(exc.stdout or ''))
        result['failure'] = f'map_course did not finish within {args.map_timeout:.0f}s'
        return result
    (logs / f'map_course_{tag}.log').write_text(done.stdout + done.stderr)
    runs = sorted(out.glob('*/commands.txt'))
    run_dir = runs[-1].parent if runs else None
    result.update(exit_code=done.returncode, run_dir=str(run_dir) if run_dir else None,
                  laps_closed=done.stdout.count('Closed mapping lap'))
    contact = _contact(sim_status())
    if done.returncode != 0:
        result['failure'] = f'map_course exited {done.returncode}'
    elif run_dir is None:
        result['failure'] = 'no commands.txt written'
    elif not all((run_dir / f).is_file() for f in
                 ('map.yaml', 'raceline_profiled.csv', 'start_pose.yaml')):
        result['failure'] = 'map.yaml, raceline_profiled.csv or start_pose.yaml missing'
    elif '!!' in (run_dir / 'commands.txt').read_text():
        result['failure'] = 'commands.txt reports something missing'
    elif not done.stdout.rstrip().endswith('=' * 72):
        result['failure'] = 'the commands were not the last thing printed'
    elif contact:
        result['failure'] = contact
    else:
        result['passed'] = True
    return result


def _race(args, logs: Path, command: str, law: str) -> dict:
    """One printed race command, on the simulator mapping just left running,
    with the car exactly where mapping stopped it."""
    result = {'step': f'race {law}', 'command': command, 'passed': False}
    log = logs / f'race_{law}.log'
    race, race_log = _start(command.split(), log)
    started = time.monotonic()
    go_at = None
    start_distance = None
    try:
        while time.monotonic() - started < args.race_timeout:
            text = log.read_text(errors='replace')
            if re.search(r'Traceback \(most recent call last\)', text):
                result['failure'] = 'exception in the race launch'
                break
            if 'NOT starting pure_pursuit' in text:
                result['failure'] = 'localization never took the start pose'
                break
            if go_at is None and 'racing can begin' in text:
                go_at = time.monotonic()
                start_distance = float(sim_status().get('distance_travelled_m') or 0.0)
            if go_at is not None and time.monotonic() - go_at >= args.race_seconds:
                break
            if race.poll() is not None:
                result['failure'] = f'race launch exited {race.returncode}'
                break
            time.sleep(1.0)
        status = sim_status()
    finally:
        _stop(race, race_log)
    if result.get('failure'):
        return result
    if go_at is None:
        result['failure'] = 'never confirmed the start pose'
        return result
    raced = float(status.get('distance_travelled_m') or 0.0) - (start_distance or 0.0)
    result['raced_m'] = round(raced, 1)
    contact = _contact(status)
    if contact:
        result['failure'] = contact
    elif raced < args.min_race_distance:
        result['failure'] = f'only raced {raced:.1f} m (need {args.min_race_distance:.1f})'
    else:
        result['passed'] = True
    return result


def map_then_race(args, logs: Path, law: str) -> list:
    """Fresh simulator; map_course; then the printed command for ``law``,
    without moving the car -- exactly what launchstart.md tells people."""
    harness_procs.refuse_if_strays(CLEANUP_PATTERNS)
    out = Path(tempfile.mkdtemp(prefix='map_course_runs_'))
    sim, sim_log = _start(['ros2', 'launch', 'racerbot_sim', 'sim_bringup_launch.py',
                           f'track:={args.track}', f'seed:={args.seed}'],
                          logs / f'sim_{law}.log')
    try:
        time.sleep(5.0)
        mapped = _map(args, logs, out, law)
        if not mapped['passed']:
            return [mapped]
        commands = {}
        for line in (Path(mapped['run_dir']) / 'commands.txt').read_text().splitlines():
            line = line.strip()
            if line.startswith('ros2 launch racerbot_launch race_launch.py'):
                found = re.search(r'steering_law:=(\w+)', line)
                commands[found.group(1) if found else 'pure_pursuit'] = line
        if law not in commands:
            return [mapped, {'step': f'race {law}', 'passed': False,
                             'failure': 'no such command in commands.txt'}]
        return [mapped, _race(args, logs, commands[law], law)]
    finally:
        _stop(sim, sim_log)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--track', default='indoor_oval')
    ap.add_argument('--seed', type=int, default=12345)
    ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--laws', default='pure_pursuit,stanley,lqr,mpc',
                    help='which of the printed race commands to run')
    ap.add_argument('--map-timeout', type=float, default=600.0)
    ap.add_argument('--race-seconds', type=float, default=40.0)
    ap.add_argument('--race-timeout', type=float, default=120.0)
    ap.add_argument('--min-race-distance', type=float, default=20.0)
    ap.add_argument('--log-directory', default=None)
    args = ap.parse_args(argv)
    logs = Path(args.log_directory or tempfile.mkdtemp(prefix='map_course_validation_'))
    logs.mkdir(parents=True, exist_ok=True)
    results = []
    for law in args.laws.split(','):
        for result in map_then_race(args, logs, law):
            results.append(result)
            print(json.dumps(result), flush=True)
    print(f'logs: {logs}')
    passed = all(r['passed'] for r in results)
    print('PASS' if passed else 'FAIL')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
