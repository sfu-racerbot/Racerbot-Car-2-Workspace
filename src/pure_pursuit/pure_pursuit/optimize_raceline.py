"""
optimize_raceline.py

Command-line tool (not a ROS node): compute the fastest safe line around a
mapped track and pace it, producing the same (x, y, speed) file
pure_pursuit_node already drives. This is the optimizing alternative to
generate_velocity_profile, which only paces the line it is given.

The difference in one sentence: generate_velocity_profile answers "how fast
can the car drive *this* line", and this answers "which line should it drive
in the first place". A recorded lap is wherever the car happened to be
driven; the racing line is a property of the track.

Two ways to describe the track:

    # From a SLAM map plus a recorded lap (the normal path on this car).
    ros2 run pure_pursuit optimize_raceline \\
        --map maps/my_track.yaml \\
        --recorded-lap src/pure_pursuit/waypoints/my_track_raw.csv \\
        --output src/pure_pursuit/waypoints/my_track_optimized.csv

    # From a ready-made centerline in the standard TUM/F1TENTH format
    # (x_m, y_m, w_tr_right_m, w_tr_left_m) -- what the F1TENTH Gym tracks ship.
    ros2 run pure_pursuit optimize_raceline \\
        --centerline Spielberg_centerline.csv \\
        --output spielberg_optimized.csv

The output is a drop-in replacement for the `waypoints_file` parameter, so
nothing about pure_pursuit_node changes -- the node, its safety layers, and
its deadman gate are untouched by this tool.

Safety. The optimizer is told the car's padded width and a clearance margin
and is not allowed to plan outside them; then the finished line is checked
independently against the map and against the steering rack's own curvature
limit. If either check fails the tool refuses to write the file, because a
raceline that the car cannot steer or that clips a wall is worse than no
raceline at all. `--allow-infeasible` downgrades that to a warning for
inspection, and prints what it found.

Everything this writes still has to be driven the same careful way as any
other new line: see docs/racing-autonomy.md and the test order in
docs/writing-your-own-node.md. A raceline runs deliberately close to the
inside of corners -- that is what makes it fast -- so raise
`--safety-margin` rather than lowering it if the first laps look tight.
"""

import argparse
import math
import sys

import numpy as np

from pure_pursuit import racing_math, raceline_optimizer


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)

    source = parser.add_argument_group('track definition (pick one)')
    source.add_argument('--map',
                        help='ROS map_server .yaml of the saved occupancy grid.')
    source.add_argument('--recorded-lap',
                        help='Raw (x,y) lap from waypoint_recorder_node. Used as the seed '
                             'for centerline extraction; requires --map.')
    source.add_argument('--centerline',
                        help='Ready-made centerline .csv in x,y,w_tr_right,w_tr_left format, '
                             'instead of --map/--recorded-lap.')

    parser.add_argument('--output', required=True,
                        help='Where to write the profiled (x,y,speed) .csv.')

    geometry = parser.add_argument_group('optimizer')
    geometry.add_argument('--optimize-spacing', type=float, default=0.30,
                          help='Arc-length resolution the optimization runs at, m. Finer is '
                               'slower and not obviously better -- the line is smooth by '
                               'construction (default: %(default)s).')
    geometry.add_argument('--output-spacing', type=float, default=0.15,
                          help='Waypoint spacing of the written file, m. Matches '
                               'waypoint_recorder_node so pure_pursuit\'s lookahead walk is '
                               'as accurate as it is for a recorded line (default: %(default)s).')
    geometry.add_argument('--iterations', type=int, default=8,
                          help='Re-linearisation passes (default: %(default)s).')
    geometry.add_argument('--trust-region', type=float, default=0.30,
                          help='Largest lateral step a single pass may take, m. The curvature '
                               'linearisation is only good locally; without this the first '
                               'pass overshoots and later passes spend themselves undoing it '
                               '(default: %(default)s).')
    geometry.add_argument('--smoothing-weight', type=float, default=0.0,
                          help='Extra penalty on lateral wander. Off by default; raise only '
                               'if a noisy hand-recorded centerline gives a restless line, '
                               'and expect to pay curvature for it (default: %(default)s).')
    geometry.add_argument('--max-track-width', type=float, default=6.0,
                          help='Longest ray cast when measuring the track, m. A side that '
                               'finds no wall within this is treated as this wide '
                               '(default: %(default)s).')
    geometry.add_argument('--centerline-passes', type=int, default=4,
                          help='Re-centering passes turning the recorded lap into a '
                               'centerline (default: %(default)s).')
    geometry.add_argument('--centerline-smoothing', type=int, default=3,
                          help='Half-window of the moving average applied to the extracted '
                               'centerline. Every ray cast is quantised to the map cell size, '
                               'and that cell-scale wiggle reads as curvature '
                               '(default: %(default)s).')

    vehicle = parser.add_argument_group('vehicle and safety')
    vehicle.add_argument('--car-width', type=float, default=0.31,
                         help='Padded car width, m. Matches pure_pursuit.yaml and '
                              'gap_follow.yaml (default: %(default)s).')
    vehicle.add_argument('--safety-margin', type=float, default=0.15,
                         help='Clearance held off both walls on top of half the car width, m. '
                              'This is the fast-versus-safe dial: the optimizer will use every '
                              'centimeter it is given, so this is what stops it apexing on the '
                              'paint (default: %(default)s).')
    vehicle.add_argument('--wheelbase', type=float, default=0.324,
                         help='Wheelbase, m (default: %(default)s).')
    vehicle.add_argument('--max-steering-angle', type=float, default=0.26,
                         help='Steering limit, rad. With the wheelbase this sets the tightest '
                              'curvature the car can physically drive, which the finished line '
                              'is checked against (default: %(default)s).')
    vehicle.add_argument('--allow-infeasible', action='store_true',
                         help='Write the file even if the curvature or clearance check fails. '
                              'For inspection only.')

    speed = parser.add_argument_group('speed profile (see generate_velocity_profile)')
    speed.add_argument('--v-max', type=float, default=4.0)
    speed.add_argument('--v-min', type=float, default=0.5)
    speed.add_argument('--a-lat-max', type=float, default=2.5)
    speed.add_argument('--a-accel-max', type=float, default=3.0)
    speed.add_argument('--a-brake-max', type=float, default=8.0)
    speed.add_argument('--smoothing-passes', type=int, default=5)
    speed.add_argument('--no-friction-ellipse', action='store_true')
    return parser


def load_track(args):
    """Return (centerline, width_left, width_right, occupancy_map_or_None)."""
    if args.centerline:
        table = np.loadtxt(args.centerline, delimiter=',', comments='#')
        if table.ndim != 2 or table.shape[1] < 4:
            raise ValueError(
                f"'{args.centerline}' must have 4 columns "
                '(x_m, y_m, w_tr_right_m, w_tr_left_m)')
        return table[:, :2], table[:, 3], table[:, 2], None

    if not (args.map and args.recorded_lap):
        raise ValueError(
            'give either --centerline, or both --map and --recorded-lap')

    from pure_pursuit import occupancy_map as occ
    grid = occ.OccupancyMap.from_yaml(args.map)
    seed = racing_math.load_xy_csv(args.recorded_lap)
    if len(seed) < 4:
        raise ValueError(f"'{args.recorded_lap}' has only {len(seed)} point(s)")

    centerline, width_left, width_right = raceline_optimizer.refine_centerline(
        grid, seed, args.optimize_spacing, args.max_track_width,
        iterations=args.centerline_passes,
        smoothing_window=args.centerline_smoothing)
    return centerline, width_left, width_right, grid


def report_line(label: str, xy: np.ndarray, args) -> dict:
    seg = racing_math.compute_segment_lengths(xy)
    kappa = racing_math.estimate_path_curvature(xy)
    speed = racing_math.compute_velocity_profile(
        seg, kappa, v_max=args.v_max, v_min=args.v_min,
        a_lat_max=args.a_lat_max, a_accel_max=args.a_accel_max,
        a_brake_max=args.a_brake_max, closed=True,
        smoothing_passes=args.smoothing_passes,
        friction_ellipse=not args.no_friction_ellipse)
    return {
        'label': label,
        'length': float(seg.sum()),
        'bending_energy': float(np.sum(kappa ** 2) * np.mean(seg)),
        'max_curvature': float(kappa.max()),
        'lap_time': racing_math.estimate_lap_time(seg, speed),
        'speed': speed,
        'seg': seg,
        'curvature': kappa,
    }


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)

    try:
        centerline, width_left, width_right, grid = load_track(args)
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1

    print(f'Track: {len(centerline)} centerline points, '
          f'width {(width_left + width_right).min():.2f}-'
          f'{(width_left + width_right).max():.2f} m')

    result = raceline_optimizer.optimize_minimum_curvature(
        centerline, width_left, width_right,
        vehicle_half_width=args.car_width / 2.0,
        safety_margin=args.safety_margin,
        spacing=args.optimize_spacing,
        iterations=args.iterations,
        smoothing_weight=args.smoothing_weight,
        trust_region=args.trust_region,
    )

    line = raceline_optimizer.resample_closed_path(
        result['line'], args.output_spacing)
    baseline = report_line(
        'centerline',
        raceline_optimizer.resample_closed_path(centerline, args.output_spacing),
        args)
    optimized = report_line('raceline', line, args)

    print(f"\nOptimizer: {len(result['line'])} points at "
          f"{result['spacing']:.2f}m, {args.iterations} passes")
    print('  integral of squared curvature per pass: '
          + ' -> '.join(f"{v:.2f}" for v in result['curvature_history']))
    if result['clamped_fraction'] > 0.0:
        print(f"  WARNING: {result['clamped_fraction']:.1%} of the track is narrower "
              f'than the car plus its {args.safety_margin:.2f}m margin; the line is '
              'pinned to the middle there')
    print(f"  lateral offset used: {result['alpha'].min():+.2f} .. "
          f"{result['alpha'].max():+.2f} m from the centerline")

    print(f"\n{'':<12}{'length':>9}{'int k^2 ds':>12}{'max |k|':>10}{'est. lap':>10}")
    for row in (baseline, optimized):
        print(f"  {row['label']:<10}{row['length']:8.1f}m{row['bending_energy']:12.2f}"
              f"{row['max_curvature']:10.3f}{row['lap_time']:9.1f}s")
    gain = baseline['lap_time'] - optimized['lap_time']
    print(f"  {'gain':<10}{'':>8} {'':>11} {'':>9} {gain:+9.1f}s "
          f'({gain / baseline["lap_time"]:+.1%}, kinematic estimate only)')

    problems = []
    limit = raceline_optimizer.curvature_limit(
        args.max_steering_angle, args.wheelbase)
    if optimized['max_curvature'] > limit:
        problems.append(
            f"the line needs curvature {optimized['max_curvature']:.3f}/m "
            f'(radius {1.0 / optimized["max_curvature"]:.2f}m) but the steering rack '
            f'can only reach {limit:.3f}/m (radius {1.0 / limit:.2f}m)')
    else:
        print(f"\n  steering feasibility: max curvature {optimized['max_curvature']:.3f}/m "
              f'of the {limit:.3f}/m the rack can reach -- OK')

    if grid is not None:
        clearance = grid.clearance_at(line[:, 0], line[:, 1])
        needed = args.car_width / 2.0
        worst = float(clearance.min())
        if worst < needed:
            problems.append(
                f'the line passes within {worst:.3f}m of a wall, closer than the '
                f'{needed:.3f}m half-width of the car itself')
        else:
            print(f'  wall clearance:       worst {worst:.3f}m from the map, '
                  f'car half-width {needed:.3f}m -- OK')

    if problems:
        print('\nINFEASIBLE:', file=sys.stderr)
        for problem in problems:
            print(f'  - {problem}', file=sys.stderr)
        if not args.allow_infeasible:
            print('\nRefusing to write. Raise --safety-margin, re-check the map, or '
                  'pass --allow-infeasible to inspect the result anyway.',
                  file=sys.stderr)
            return 2
        print('\n--allow-infeasible given: writing anyway. Do not drive this.',
              file=sys.stderr)

    racing_math.save_profiled_csv(args.output, line, optimized['speed'])
    print(f"\nWrote {len(line)} waypoints to {args.output}")
    print(f"  speed range: {optimized['speed'].min():.2f} - "
          f"{optimized['speed'].max():.2f} m/s")
    print('  Simulation and arithmetic are not physical sign-off: drive this at low '
          'speed first, LB held, per docs/writing-your-own-node.md.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
