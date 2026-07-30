"""
filter_log.py

Follow the launch terminal's tee'd log and print only the lines worth a
human's attention, using run_events.LogClassifier.

The launch terminal produces several hundred lines a minute -- three nodes
logging their decision every second at a 40Hz control rate. Perhaps a
dozen lines in a whole run change what you would do next. This keeps
those, throttles the recognized-but-repetitive ones, and drops the rest.

    ros2 run race_diagnostics filter_log ~/.ros/racerbot_runs/latest/launch.log

Add --from-start to re-classify a finished log instead of following a
live one, which is the usual way to review a run afterwards.
"""

import argparse
import sys
import time

from race_diagnostics.run_events import (LogClassifier, blocking_gate,
                                         parse_lap_progress)


def follow(path: str, from_start: bool, poll_sec: float = 0.5):
    """Yield lines from a growing file, tolerating it not existing yet and
    being truncated underneath us (which is exactly what `tee` does when
    the next run starts)."""
    handle = None
    position = 0
    while True:
        if handle is None:
            try:
                handle = open(path, 'r')
                if not from_start:
                    handle.seek(0, 2)
                position = handle.tell()
            except FileNotFoundError:
                time.sleep(poll_sec)
                continue

        line = handle.readline()
        if line:
            position = handle.tell()
            yield line.rstrip('\n')
            continue

        # No new data. Detect truncation (a fresh run reusing the path).
        try:
            if handle.tell() > 0 and _size(path) < position:
                handle.close()
                handle = None
                from_start = True
                continue
        except OSError:
            pass
        if from_start and _at_eof_of_static_file(handle, path, position):
            return
        time.sleep(poll_sec)


def _size(path: str) -> int:
    import os
    return os.path.getsize(path)


def _at_eof_of_static_file(handle, path, position) -> bool:
    """When re-reading a finished log, stop at the end instead of hanging."""
    try:
        return _size(path) == position
    except OSError:
        return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log_path', help="the launch terminal's tee'd log")
    parser.add_argument('--throttle-sec', type=float, default=20.0,
                        help='seconds between repeats of a noisy category')
    parser.add_argument('--from-start', action='store_true',
                        help='classify an existing log from the top and exit at its end')
    args = parser.parse_args(argv)

    classifier = LogClassifier(throttle_sec=args.throttle_sec)
    counts = {}
    try:
        for line in follow(args.log_path, args.from_start):
            category, emit = classifier.classify(line, time.time())
            if category is None:
                continue
            counts[category] = counts.get(category, 0) + 1
            if not emit:
                continue
            if category == 'lap_progress':
                progress = parse_lap_progress(line)
                gate = blocking_gate(progress)
                if gate:
                    print(f'[lap_progress] lap {progress["lap"]}/{progress["of"]} '
                          f'held open by: {gate} '
                          f'({progress[gate]["value"]} vs limit {progress[gate]["limit"]})'
                          if gate != 'departed' else
                          f'[lap_progress] lap {progress["lap"]}/{progress["of"]} '
                          'held open by: has not departed the start yet', flush=True)
                    continue
            print(f'[{category}] {line}', flush=True)
    except KeyboardInterrupt:
        pass
    if counts:
        print('\n--- category totals ---', file=sys.stderr)
        for category, total in sorted(counts.items(), key=lambda kv: -kv[1]):
            print(f'  {category:20s} {total}', file=sys.stderr)


if __name__ == '__main__':
    main()
