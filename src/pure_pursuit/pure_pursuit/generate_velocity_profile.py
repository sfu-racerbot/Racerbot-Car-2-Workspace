"""
generate_velocity_profile.py

Command-line tool (not a ROS node): turns a raw (x, y) waypoint recording
from waypoint_recorder_node into a race-ready (x, y, speed) file that
pure_pursuit_node can actually drive. This is Phase 4 of the pipeline
described in docs/racing-autonomy.md.

The speed at each point is computed from the *path's own geometry* --
tighter curvature means a lower cornering-speed limit, and a physically
plausible forward/backward smoothing pass turns that into real braking
zones before corners and acceleration zones out of them. See
racing_math.compute_velocity_profile()'s docstring for the exact
algorithm.

Usage:
    ros2 run pure_pursuit generate_velocity_profile \\
        --input  src/pure_pursuit/waypoints/my_track_raw.csv \\
        --output src/pure_pursuit/waypoints/my_track_profiled.csv \\
        --v-max 6.0 --a-lat-max 8.0 --a-accel-max 3.0 --a-brake-max 8.0

Run with --help for the full list of tunable physical limits, and see
docs/racing-autonomy.md for how to choose them safely (start
conservative, raise gradually -- exactly like every other speed
parameter on this car).
"""

import argparse
import sys

from pure_pursuit import racing_math


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--input', required=True,
                         help='Raw waypoints .csv (x,y columns) from waypoint_recorder_node.')
    parser.add_argument('--output', required=True,
                         help='Where to write the profiled (x,y,speed) .csv.')
    parser.add_argument('--open-path', action='store_true',
                         help='Treat the recording as a single start-to-finish pass instead of '
                              'a closed loop (default: closed loop -- correct for a normal lap track).')
    parser.add_argument('--v-max', type=float, default=6.0,
                         help='Absolute top speed cap, m/s (default: %(default)s).')
    parser.add_argument('--v-min', type=float, default=0.5,
                         help='Absolute minimum speed -- the car never fully stops mid-track, m/s '
                              '(default: %(default)s).')
    parser.add_argument('--a-lat-max', type=float, default=8.0,
                         help='Max lateral (cornering) acceleration the tires/chassis can hold, m/s^2. '
                              'Start conservative; raise it only after confirming the car does not '
                              'slide at the speeds this already produces (default: %(default)s).')
    parser.add_argument('--a-accel-max', type=float, default=3.0,
                         help='Max forward acceleration the drivetrain can produce, m/s^2 '
                              '(default: %(default)s).')
    parser.add_argument('--a-brake-max', type=float, default=8.0,
                         help='Max braking deceleration, m/s^2 (default: %(default)s).')
    parser.add_argument('--smoothing-passes', type=int, default=3,
                         help='Forward+backward smoothing iterations. A closed loop has no single '
                              'starting speed to seed the sweep from, so it takes more than one pass '
                              'for the start/finish seam to converge; see docs/racing-autonomy.md '
                              '(default: %(default)s).')
    parser.add_argument('--smoothing-window', type=int, default=3,
                         help='Half-window (in waypoints, each side) of the moving-average smoothing '
                              'applied to the recorded line *before* estimating curvature. Recorded '
                              'waypoints carry localization jitter, and at ~0.15m spacing even a '
                              'couple of centimeters of wiggle reads as curvature -- i.e. phantom '
                              'braking zones mid-straight. The smoothed line is also what gets '
                              'written to the output (the profile must describe the line the car '
                              'will actually drive). 0 disables (default: %(default)s).')
    parser.add_argument('--no-friction-ellipse', action='store_true',
                         help='Give the accel/brake sweeps the full longitudinal budget even '
                              'mid-corner, instead of the friction-ellipse-coupled (shared-grip) '
                              'budget. Less physically honest -- only useful for comparing against '
                              'previously generated profiles.')
    return parser


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)
    closed = not args.open_path

    xy = racing_math.load_xy_csv(args.input)
    if len(xy) < 3:
        print(f"error: '{args.input}' has only {len(xy)} point(s), need at least 3", file=sys.stderr)
        return 1

    xy = racing_math.smooth_path(xy, args.smoothing_window, closed=closed)

    seg_len = racing_math.compute_segment_lengths(xy, closed=closed)
    curvature = racing_math.estimate_path_curvature(xy, closed=closed)
    speed = racing_math.compute_velocity_profile(
        seg_len, curvature,
        v_max=args.v_max, v_min=args.v_min,
        a_lat_max=args.a_lat_max, a_accel_max=args.a_accel_max, a_brake_max=args.a_brake_max,
        closed=closed, smoothing_passes=args.smoothing_passes,
        friction_ellipse=not args.no_friction_ellipse,
    )

    racing_math.save_profiled_csv(args.output, xy, speed)

    total_distance = float(seg_len.sum())
    lap_time = racing_math.estimate_lap_time(seg_len, speed, closed=closed)
    print(f"Wrote {len(xy)} waypoints to {args.output}")
    print(f"  path length:    {total_distance:.1f} m ({'closed loop' if closed else 'open path'})")
    print(f"  speed range:    {speed.min():.2f} - {speed.max():.2f} m/s")
    print(f"  estimated time: {lap_time:.1f} s "
          f"(kinematic estimate from the speed profile alone -- not a substitute for a real test lap)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
