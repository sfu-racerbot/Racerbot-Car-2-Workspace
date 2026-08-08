#!/usr/bin/env python3
"""End-to-end validation of `auto_map_race_launch.py` against the simulator.

`tools/f1tenth_sim/run_validation.py` calls the controllers' *math* with no
ROS at all. That is the right shape for tuning a control law and the wrong
shape for the way the automatic mode actually broke, which was in the
wiring: SLAM, TF, the recorded racing line, a runtime parameter handover
between two nodes, and a safety layer with no way out of its own stop.

This runs the real launch file over `racerbot_sim`, watches the log and
`/sim/status`, and answers one question per scenario: did the car map a
course, generate a line it can steer, hand over to pure pursuit, and then
race it without hitting anything?

    tools/racerbot_sim/run_auto_map_validation.py --scenario all
    tools/racerbot_sim/run_auto_map_validation.py --scenario solo --track indoor_tight
    tools/racerbot_sim/run_auto_map_validation.py --scenario traffic --keep-logs

Exits 0 only if every selected scenario passes, so it can gate a change.
Needs a built, sourced workspace (`source install/setup.bash`) and the
F1TENTH Gym from `tools/f1tenth_sim/setup.sh`.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]

# Everything the run has to reach, in order. Each is a substring searched for
# in the launch log; the phase names are what a failure is reported as.
PHASES = [
    ('slam_up', 'Registering sensor'),
    ('mapping_lap_1', 'Closed mapping lap 1/'),
    ('line_cleaned', 'Recorded lap cleaned up:'),
    ('profile_written', 'Generated '),
    ('map_saved', 'Saved occupancy map successfully.'),
    ('profile_loaded', 'Profile loaded successfully'),
    ('racing', 'Transition complete: pure pursuit now has drive control.'),
]

FATAL_PATTERNS = [
    ('node_crash', re.compile(r'process has died.*exit code [1-9]')),
    ('exception', re.compile(r'Traceback \(most recent call last\)')),
    ('supervisor_error', re.compile(r'supervisor_error|Could not generate the racing profile')),
    ('profile_refused', re.compile(r'Refusing to hand it to pure pursuit')),
]

CLEANUP_PATTERNS = (
    'racerbot_sim/lib', 'ackermann_mux/lib', 'tf2_ros/static_transform_publisher',
    'gap_follow/lib', 'pure_pursuit/lib', 'web_dashboard/lib',
    'async_slam_toolbox_node',
)


def stop_everything():
    """Leave no node behind: the next scenario shares the ROS graph."""
    for pattern in CLEANUP_PATTERNS:
        subprocess.run(['pkill', '-f', pattern], capture_output=True)
    time.sleep(1.0)
    for pattern in CLEANUP_PATTERNS:
        subprocess.run(['pkill', '-9', '-f', pattern], capture_output=True)
    time.sleep(1.0)


class Run:
    """One launch of the stack, with its log and its final /sim/status."""

    def __init__(self, log_path: Path, arguments: list, output_directory: Path):
        self.log_path = log_path
        self.arguments = arguments
        self.output_directory = output_directory
        self.process = None
        self.handle = None

    def start(self):
        self.handle = open(self.log_path, 'w')
        self.process = subprocess.Popen(
            ['ros2', 'launch', 'racerbot_sim', 'sim_auto_map_race_launch.py']
            + self.arguments,
            stdout=self.handle, stderr=subprocess.STDOUT, cwd=str(ROOT),
            start_new_session=True)

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
        stop_everything()


def sim_status() -> dict:
    """Latest /sim/status. It is latched, so one --once is enough."""
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


def parse_cleanup(text: str) -> dict:
    """Pull the racing line's own report out of the log."""
    match = re.search(r'Recorded lap cleaned up: (.+)', text)
    if not match:
        return {}
    line = match.group(1)
    numbers = {}
    for key, pattern in (
        ('points', r'^(\d+) points'),
        ('length_m', r'over ([\d.]+)m'),
        ('harmonics', r'kept (\d+) harmonics'),
        ('deviation_m', r'at most ([\d.]+)m off'),
        ('peak_curvature', r'peak curvature ([\d.]+)/m'),
        ('rack_limit', r'of the ([\d.]+)/m the rack'),
        ('steering_deg', r'needs ([\d.]+)deg steering'),
        ('over_limit_pct', r'on ([\d.]+)% of waypoints'),
        ('seam_deg', r'seam heading error ([\d.]+)deg'),
    ):
        found = re.search(pattern, line)
        if found:
            numbers[key] = float(found.group(1))
    numbers['text'] = line
    return numbers


# All three default to indoor_wide's 2.6m corridor, because width is what
# these scenarios are actually limited by:
#
# * Racing at all. The line comes from gap_follow, which drives 0.25-0.35m
#   from a corner's wall, and pure pursuit's cross-track error through a
#   corner near this car's turning circle measured 0.39-0.57m at 2.5-3.0
#   m/s. On indoor_oval's 1.8m corridor those do not both fit and the car
#   touches the wall -- which is a true statement about the course, not a
#   defect to gate a regression suite on.
# * Getting past anything. gap_follow inflates every obstacle by
#   car_width/2 + safety_margin, demanding 0.67m of width; a car parked in
#   the middle of a 1.8m corridor leaves 0.6m either side and is simply a
#   roadblock.
#
# Run --track indoor_oval or indoor_tight deliberately to ask the narrower
# question: does the pipeline still map, clean up, and *refuse* correctly.
SCENARIO_TRACKS = {
    'solo': 'indoor_wide',
    'obstacle': 'indoor_wide',
    'traffic': 'indoor_wide',
}

SCENARIO_OPPONENTS = {
    # A car parked to one side of the racing line -- a spun-out car, not a
    # roadblock. This is the case that used to end a run outright, because
    # a hard stop cannot clear its own safety cone.
    'obstacle': '9.0,0.0,0.45',
    # Two slower cars, far enough apart that the ego meets them separately.
    'traffic': '8.0,0.7,0.0;22.0,0.6,0.0',
}


def scenario_arguments(scenario: str, args) -> list:
    arguments = [
        f'track:={args.track or SCENARIO_TRACKS[scenario]}',
        f'seed:={args.seed}',
        f'mapping_laps:={args.mapping_laps}',
        f'mapping_max_speed:={args.mapping_max_speed}',
    ]
    if scenario in SCENARIO_OPPONENTS:
        arguments.append(f'opponents:={SCENARIO_OPPONENTS[scenario]}')
    for extra in args.launch_argument or []:
        arguments.append(extra)
    return arguments


def run_scenario(scenario: str, args) -> dict:
    stop_everything()
    log_path = Path(args.log_directory) / f'{scenario}.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    output_directory = Path(tempfile.mkdtemp(prefix=f'auto_map_{scenario}_'))

    run = Run(log_path, scenario_arguments(scenario, args)
              + [f'output_directory:={output_directory}'], output_directory)
    track = args.track or SCENARIO_TRACKS[scenario]
    result = {
        'scenario': scenario,
        'track': track,
        'seed': args.seed,
        'passed': False,
        'phases': {},
        'failure': None,
        'log': str(log_path),
    }

    started = time.monotonic()
    reached = {}
    fatal = None
    racing_since = None
    racing_distance = None
    try:
        run.start()
        while True:
            elapsed = time.monotonic() - started
            text = run.text()

            for name, pattern in FATAL_PATTERNS:
                if pattern.search(text):
                    fatal = name
                    break
            if fatal:
                break

            for name, marker in PHASES:
                if name not in reached and marker in text:
                    reached[name] = round(elapsed, 1)
                    if name == 'racing':
                        racing_since = time.monotonic()

            if racing_since is not None:
                if racing_distance is None:
                    racing_distance = float(
                        sim_status().get('distance_travelled_m') or 0.0)
                if time.monotonic() - racing_since >= args.race_seconds:
                    break
            elif elapsed >= args.timeout:
                fatal = 'timeout'
                break
            if run.process.poll() is not None:
                fatal = 'launch_exited'
                break
            time.sleep(1.0)

        status = sim_status()
    finally:
        run.stop()

    text = run.text()
    result['phases'] = reached
    result['cleanup'] = parse_cleanup(text)
    result['sim'] = {
        key: status.get(key) for key in
        ('sim_time_s', 'distance_travelled_m', 'wall_contact_steps',
         'ever_contacted', 'opponent_wall_contact', 'car_contact_steps',
         'num_agents')
    }
    result['racing_seconds'] = (
        round(time.monotonic() - racing_since, 1) if racing_since else 0.0)
    raced_metres = (
        None if racing_distance is None
        else round(float(status.get('distance_travelled_m') or 0.0) - racing_distance, 1))
    result['raced_metres'] = raced_metres

    missing = [name for name, _ in PHASES if name not in reached]
    if fatal:
        result['failure'] = fatal
    elif missing:
        result['failure'] = f'never reached: {", ".join(missing)}'
    elif status.get('ever_contacted'):
        result['failure'] = (
            f"hit a wall ({status.get('wall_contact_steps')} steps in contact)")
    elif status.get('opponent_wall_contact'):
        result['failure'] = 'an opponent hit a wall'
    elif status.get('car_contact_steps'):
        # Checked separately from wall contact, and it has to be: an ego
        # wedged against a parked opponent touches no wall at all. A run of
        # the obstacle scenario passed on exactly that hole while the car
        # spent its whole racing phase nose-to-nose with the other car.
        result['failure'] = (
            f"cars touched ({status.get('car_contact_steps')} steps in contact)")
    elif raced_metres is not None and raced_metres < args.min_raced_distance:
        # The distance that matters is the one covered *after* the handover.
        # Total distance is dominated by the mapping laps and stays high even
        # when the racing phase never moves at all.
        result['failure'] = (
            f'only covered {raced_metres:.1f}m in {result["racing_seconds"]:.0f}s '
            f'of racing, under the {args.min_raced_distance:.1f}m a real racing '
            'stint needs -- it reached pure pursuit and then stopped')
    else:
        distance = float(status.get('distance_travelled_m') or 0.0)
        if distance < args.min_distance:
            result['failure'] = (
                f'only travelled {distance:.1f}m, under the '
                f'{args.min_distance:.1f}m a completed run needs')
        else:
            result['passed'] = True

    # Keep the generated line: it is the artifact a failure is diagnosed from.
    lines = sorted(output_directory.glob('*/raceline_profiled.csv'))
    result['raceline'] = str(lines[-1]) if lines else None
    if not lines and output_directory.exists():
        shutil.rmtree(output_directory, ignore_errors=True)
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scenario', default='all',
                        choices=['all', 'solo', 'obstacle', 'traffic'])
    parser.add_argument('--track', default=None,
                        help=('indoor_oval, indoor_tight or indoor_wide. '
                              'Default: whichever the scenario needs.'))
    parser.add_argument('--seed', type=int, default=12345)
    parser.add_argument('--mapping-laps', type=int, default=2)
    parser.add_argument('--mapping-max-speed', type=float, default=1.0)
    parser.add_argument('--timeout', type=float, default=420.0,
                        help='seconds to reach the racing handover')
    parser.add_argument('--race-seconds', type=float, default=60.0,
                        help='seconds to watch the racing phase before judging')
    parser.add_argument('--min-distance', type=float, default=40.0,
                        help='metres the ego must cover over the whole run')
    parser.add_argument('--min-raced-distance', type=float, default=25.0,
                        help='metres the ego must cover after the racing handover')
    parser.add_argument('--log-directory',
                        default=str(ROOT / '.sim' / 'auto_map_logs'))
    parser.add_argument('--launch-argument', action='append',
                        help='extra name:=value passed straight to ros2 launch')
    parser.add_argument('--output', help='write the combined JSON report here')
    args = parser.parse_args(argv)

    scenarios = (['solo', 'obstacle', 'traffic'] if args.scenario == 'all'
                 else [args.scenario])
    results = []
    for scenario in scenarios:
        track = args.track or SCENARIO_TRACKS[scenario]
        print(f'--- {scenario} on {track} ---', flush=True)
        result = run_scenario(scenario, args)
        results.append(result)
        print(json.dumps(result, indent=2), flush=True)
        print(f"{scenario}: {'PASS' if result['passed'] else 'FAIL'}"
              f"{'' if result['passed'] else ' -- ' + str(result['failure'])}\n",
              flush=True)

    if args.output:
        Path(args.output).write_text(json.dumps(results, indent=2) + '\n')
    passed = sum(1 for result in results if result['passed'])
    print(f'{passed}/{len(results)} scenario(s) passed')
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
