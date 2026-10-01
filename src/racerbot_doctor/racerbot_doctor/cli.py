"""`ros2 run racerbot_doctor doctor` -- read-only self-check of the car.

Exit code: 0 = no FAIL, 1 = at least one FAIL, 2 = the doctor itself crashed.
See src/racerbot_doctor/README.md.
"""
import argparse
import sys
import traceback

from . import car
from .checks import run_all
from .report import render


def _default_sample(window_s):
    from .live import sample      # imports rclpy; may raise ImportError
    return sample(window_s)


def main(argv=None, *, gather_fn=None, sample_fn=None, out=None):
    out = out or sys.stdout
    parser = argparse.ArgumentParser(
        prog='doctor', description='Read-only self-check: is each device '
        'plugged in, is its kernel driver bound, is its ROS driver running, '
        'and is it producing sane data. Never publishes or moves the car.')
    parser.add_argument('--window', type=float, default=3.0,
                        help='seconds to sample each topic (default 3)')
    parser.add_argument('--no-live', action='store_true',
                        help='hardware/OS checks only; do not join the ROS graph')
    parser.add_argument('--no-color', action='store_true')
    args = parser.parse_args(argv)
    color = not args.no_color and out.isatty()

    try:
        if gather_fn is None:
            from .system import gather as gather_fn
        if sample_fn is None:
            sample_fn = _default_sample
        snap = gather_fn(probe_lidar_tcp='auto')

        notes = []
        live = None
        drivers_up = any(p.runs(exe) for p in snap.processes
                         for exe in car.BRINGUP_EXES + (car.EXE_REALSENSE,))
        if args.no_live:
            notes.append('live stage off (--no-live): topic layers skipped')
        elif drivers_up:
            try:
                live = sample_fn(args.window)
            except ImportError as e:
                notes.append(f'live stage skipped, ROS is not importable ({e}): '
                             'source /opt/ros/jazzy/setup.bash && source '
                             '~/racerbot-ws/install/setup.bash')
            except Exception as e:      # noqa: BLE001 -- report, don't crash
                notes.append(f'live stage failed: {type(e).__name__}: {e}')

        reports = run_all(snap, live)
        out.write(render(reports, color=color, notes=notes))
        failed = any(l.status.value == 'FAIL' for r in reports for l in r.layers)
        return 1 if failed else 0
    except Exception:                   # noqa: BLE001
        out.write('racerbot_doctor crashed -- this is a bug in the doctor, '
                  'not a finding about the car:\n' + traceback.format_exc())
        return 2


if __name__ == '__main__':
    sys.exit(main())
