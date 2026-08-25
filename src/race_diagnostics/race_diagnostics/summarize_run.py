"""
summarize_run.py

Turn a recorded run directory into one page of structured findings --
the thing you paste to a colleague, or hand to an agent, instead of
50,000 characters of terminal scrollback.

    ros2 run race_diagnostics summarize_run ~/.ros/racerbot_runs/20260727-202458
    ros2 run race_diagnostics summarize_run <dir> --json     # machine-readable

It reads whatever is present and says so when something is missing --
a missing artifact is itself a finding (no map.pgm means the occupancy
save failed; no events.jsonl means the probe was not running, so pose lag
was never measured and cannot be recovered afterwards).

Deliberately answers the questions that actually came up while debugging
this car, in the order they matter:
  1. How far did the run get? (mapping -> lap closed -> profile -> racing)
  2. What stopped it, if anything?
  3. Was localization healthy -- worst pose lag, any frozen-pose events?
  4. Which lap-closure gate was holding a lap open?
  5. Did the artifacts save?
"""

import argparse
import json
import sys
import time
from pathlib import Path

from race_diagnostics import node_logs
from race_diagnostics.run_events import (LogClassifier, RunTimeline,
                                         blocking_gate, parse_lap_progress)

PHASE_ORDER = [
    ('slam_lifecycle', 'SLAM configured and activated'),
    ('lap_closed', 'at least one mapping lap closed'),
    ('profile_generated', 'racing profile generated'),
    ('profile_loaded', 'profile accepted by pure_pursuit'),
    ('handover', 'pure pursuit took drive control'),
]


def _run_time_window(run_dir: Path):
    """(start_epoch_sec, end_epoch_sec) this run covers, best-effort.

    The run directory is named from its own start time -- record_run.py
    and auto_map_race_node.py's _write_profile both use
    strftime('%Y%m%d-%H%M%S') -- parsed back out here rather than tracked
    as a second, separately-written stamp. Nothing records an explicit
    run-end time, so the end is approximated as the newest mtime among
    whatever the run has written so far; a run that hasn't written
    anything yet (or a directory name that doesn't parse) falls back to
    the start time alone, and find_node_logs's own slack_sec absorbs the
    rest.
    """
    try:
        start = time.mktime(time.strptime(run_dir.name, '%Y%m%d-%H%M%S'))
    except ValueError:
        start = run_dir.stat().st_mtime
    end = start
    for path in run_dir.rglob('*'):
        try:
            end = max(end, path.stat().st_mtime)
        except OSError:
            continue
    return start, end


def read_events(run_dir: Path):
    path = run_dir / 'events.jsonl'
    if not path.exists():
        return None
    events = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def analyze(run_dir: Path) -> dict:
    result = {
        'run_directory': str(run_dir),
        'artifacts': {},
        'phases_reached': [],
        'phases_missing': [],
        'pose': {},
        'watchdogs': {},
        'lap_gate_blocking': None,
        'errors': [],
        'notes': [],
    }

    for name, description in [
            ('launch.log', 'launch terminal output (reconstructed from '
                           '~/.ros/log/ if missing; `| tee` gives the '
                           'authoritative copy)'),
            ('events.jsonl', 'probe event stream (pose lag lives here)'),
            ('probe.log', 'human-readable probe output'),
            ('bag', 'rosbag for offline replay'),
            ('map.pgm', 'saved occupancy map'),
            ('raceline_profiled.csv', 'generated racing line')]:
        present = (run_dir / name).exists()
        result['artifacts'][name] = present
        if not present:
            result['notes'].append(f'MISSING {name} -- {description}')

    events = read_events(run_dir)
    if events is not None:
        lags = [e['pose_lag_max_sec'] for e in events
                if e.get('pose_lag_max_sec') is not None]
        result['pose']['lag_max_sec'] = max(lags) if lags else None
        result['pose']['lag_high_alerts'] = sum(
            1 for e in events if e.get('category') == 'pose_lag_high')
        result['pose']['frozen_samples'] = sum(
            1 for e in events if e.get('pose_frozen'))
        result['pose']['tf_lost_events'] = sum(
            1 for e in events if e.get('category') == 'tf_lost')
        topics = {e.get('topic') for e in events
                  if e.get('category') == 'first_message'}
        result['pose']['topics_ever_seen'] = sorted(t for t in topics if t)

    log_path = run_dir / 'launch.log'
    if log_path.exists():
        lines = log_path.read_text(errors='replace').splitlines()
    else:
        # No `| tee` this run -- ROS still wrote every node's own stdout
        # to ~/.ros/log/ on its own; find and merge it instead of giving
        # up. See node_logs.py.
        start_sec, end_sec = _run_time_window(run_dir)
        found = node_logs.find_node_logs(start_sec, end_sec)
        lines = node_logs.merged_lines(found)
        if lines:
            result['notes'].append(
                f'no launch.log (no `| tee`) -- reconstructed {len(lines)} '
                f'line(s) from {sum(len(v) for v in found.values())} '
                f"per-node log file(s) under ~/.ros/log/ instead: "
                f"{', '.join(sorted(found))}")
        else:
            result['notes'].append(
                'no launch.log, and no matching ~/.ros/log/ node logs found '
                "for this run's time window -- nothing to classify")

    if lines:
        classifier = LogClassifier(throttle_sec=0.0)   # count everything
        timeline = RunTimeline()
        last_progress = None
        for index, line in enumerate(lines):
            category, _ = classifier.classify(line, float(index))
            if category is None:
                continue
            timeline.add(float(index), category, line[:400])
            if category == 'lap_progress':
                progress = parse_lap_progress(line)
                if progress is not None:
                    last_progress = progress
                    timeline.note_lap_progress(progress)
            if category in ('error', 'node_death'):
                result['errors'].append(line.strip()[:400])
            if category == 'watchdog':
                state = line.split('STOP [', 1)[1].split(']', 1)[0]
                result['watchdogs'][state] = result['watchdogs'].get(state, 0) + 1

        summary = timeline.summary()
        reached = set(summary['phases_reached'])
        for key, description in PHASE_ORDER:
            (result['phases_reached'] if key in reached
             else result['phases_missing']).append(description)
        if last_progress is not None and blocking_gate(last_progress):
            gate = blocking_gate(last_progress)
            result['lap_gate_blocking'] = {
                'gate': gate,
                'detail': last_progress.get(gate) if gate != 'departed' else None,
            }
        result['event_counts'] = summary['event_counts']

    # De-duplicate errors while keeping first-seen order: the same failure
    # repeated 200 times is one finding, not two hundred.
    seen = set()
    unique = []
    for error in result['errors']:
        if error not in seen:
            seen.add(error)
            unique.append(error)
    result['errors'] = unique[:25]
    return result


def render(result: dict) -> str:
    lines = [f"# Run summary: {result['run_directory']}", '']

    lines.append('## How far it got')
    for description in result['phases_reached']:
        lines.append(f'  [reached] {description}')
    for description in result['phases_missing']:
        lines.append(f'  [ NOT   ] {description}')
    if 'event_counts' not in result:
        lines.append('  (no log lines to classify -- see notes below)')
    lines.append('')

    pose = result.get('pose') or {}
    if pose:
        lines.append('## Localization health')
        lag = pose.get('lag_max_sec')
        if lag is not None:
            verdict = ('OK' if lag < 0.15 else
                       'MARGINAL' if lag < 0.5 else
                       'BAD -- pure_pursuit will stop through stalls this long')
            lines.append(f'  worst pose lag: {lag:.2f}s  [{verdict}]')
        lines.append(f"  pose-lag alerts: {pose.get('lag_high_alerts', 0)}")
        lines.append(f"  frozen-pose samples: {pose.get('frozen_samples', 0)}")
        lines.append(f"  map->base_link losses: {pose.get('tf_lost_events', 0)}")
        lines.append(f"  topics ever seen: {', '.join(pose.get('topics_ever_seen') or []) or 'none'}")
        lines.append('')

    if result['watchdogs']:
        lines.append('## Watchdog stops (count by reason)')
        for state, count in sorted(result['watchdogs'].items(), key=lambda kv: -kv[1]):
            lines.append(f'  {state:22s} {count}')
        lines.append('')

    event_counts = result.get('event_counts')
    if event_counts is not None:
        # THROTTLED_PATTERNS's 'stopped' catch-all counts every 'STOP [...]'
        # line whose state isn't one of CRITICAL_PATTERNS's named watchdog
        # states -- i.e. every stop this report's watchdog breakdown above
        # did NOT account for. 0 is the healthy value; a regex that has
        # drifted out of sync with the driving nodes (exactly what section
        # 5 of the 2026-08-25 plan found for the lap-progress regex) shows
        # up here as a silent nonzero count instead of a wrong answer that
        # looks like a right one.
        unclassified = event_counts.get('stopped', 0)
        lines.append('## Unclassified stops')
        lines.append(
            f'  {unclassified} STOP line(s) matched no known watchdog state '
            "in run_events.py's CRITICAL_PATTERNS. 0 is expected; nonzero "
            'means that pattern is missing a state and needs updating.')
        lines.append('')

    if result['lap_gate_blocking']:
        gate = result['lap_gate_blocking']
        lines.append('## Lap closure')
        detail = gate['detail']
        if detail:
            lines.append(f"  last sample was held open by '{gate['gate']}': "
                         f"{detail['value']} vs limit {detail['limit']}")
        else:
            lines.append(f"  last sample was held open by '{gate['gate']}'")
        lines.append('')

    if result['errors']:
        lines.append('## Errors and node deaths (de-duplicated)')
        for error in result['errors']:
            lines.append(f'  {error}')
        lines.append('')

    if result['notes']:
        lines.append('## Missing artifacts')
        for note in result['notes']:
            lines.append(f'  {note}')
        lines.append('')

    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory')
    parser.add_argument('--json', action='store_true',
                        help='emit the raw structured result instead of a report')
    args = parser.parse_args(argv)

    run_dir = Path(args.run_directory).expanduser()
    if not run_dir.is_dir():
        print(f'not a directory: {run_dir}', file=sys.stderr)
        return 1
    result = analyze(run_dir)
    print(json.dumps(result, indent=2) if args.json else render(result))
    return 0


if __name__ == '__main__':
    sys.exit(main())
