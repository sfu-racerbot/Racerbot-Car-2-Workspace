#!/usr/bin/env python3
"""Benchmark the MPC steering law's solve time (spec 04, gate G0).

Measures the p50/p90/p99/p99.9/max wall-clock solve time of `MpcLaw.steer()`
over a long, varied workload that exercises warm starting the way driving
does: a closed loop profiled with the same velocity profile the node uses,
walked at 40 Hz with the pose perturbed by a slowly varying random walk.

This is pure Python + osqp -- no ROS. It needs osqp installed, which is the
G0 install decision (see docs/superpowers/specs/2026-09-27-tracking-04-mpc.md).
Run it on the Jetson, idle and then under load (the ROS simulator stack plus
pure_pursuit_node), and commit both reports as docs/mpc-benchmark-<date>.json.
The G0 gate is the *loaded* run's p99 < 8 ms.

    python3 tools/mpc_benchmark/benchmark_mpc.py [--csv PATH] [--steps 20000]
"""

from __future__ import annotations

import argparse
import json
import math
import platform
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "pure_pursuit"))

import numpy as np  # noqa: E402

try:
    import osqp  # noqa: F401
except ImportError:
    raise SystemExit(
        "osqp is not installed. This benchmark is spec 04 gate G0; install it "
        "first (see the spec for the install decision) and re-run.")

from pure_pursuit import racing_math, tracking_laws  # noqa: E402


def synthetic_loop(v_max=4.0, a_lat_max=2.5):
    """A closed loop: two 1.5 m-radius hairpins joined by straights with a
    chicane, profiled like a real racing line."""
    pts = []
    # Long straight
    for i in range(80):
        pts.append((i * 0.1, 0.0))
    # Hairpin 1 (left, R=1.5)
    R = 1.5
    n = int(round(math.pi * R / 0.1))
    for i in range(n):
        a = (i / n) * math.pi
        pts.append((8.0 + R * math.sin(a), R - R * math.cos(a)))
    # Chicane (a quick left-right wiggle) on the back straight
    for i in range(60):
        x = 8.0 - i * 0.1
        y = 3.0 + 0.5 * math.sin(i * 0.2)
        pts.append((x, y))
    # Hairpin 2 (left, R=1.5)
    for i in range(n):
        a = (i / n) * math.pi
        pts.append((2.0 - R * math.sin(a), 3.0 + R + R * math.cos(a) - R))
    xy = np.array(pts)
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    curvature = racing_math.estimate_path_curvature(xy, closed=True)
    speed = racing_math.compute_velocity_profile(
        seg_len, curvature, v_max=v_max, v_min=0.5, a_lat_max=a_lat_max,
        a_accel_max=3.0, a_brake_max=8.0, closed=True, smoothing_passes=8)
    return xy, seg_len, speed


def environment():
    """Record the host environment for the benchmark report."""
    env = {
        'git_commit': subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            capture_output=True, text=True).stdout.strip(),
        'python': platform.python_version(),
        'osqp': osqp.__version__,
        'numpy': np.__version__,
        'machine': platform.machine(),
    }
    for cmd, key in ((['nvpmodel', '-q'], 'nvpmodel'),
                     (['jetson_clocks', '--show'], 'jetson_clocks')):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=5).stdout.strip()
            env[key] = out.splitlines()[0] if out else 'unknown'
        except Exception:
            env[key] = 'unavailable'
    try:
        env['cpu_governor'] = Path(
            '/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor'
        ).read_text().strip()
    except Exception:
        env['cpu_governor'] = 'unavailable'
    return env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path, default=None,
                        help='A profiled (x,y,speed) racing line to drive; '
                             'default is a built-in synthetic loop.')
    parser.add_argument('--steps', type=int, default=20000)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--output', type=Path, default=None)
    args = parser.parse_args()

    if args.csv:
        xy, speed = racing_math.load_profiled_csv(str(args.csv))
        seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    else:
        xy, seg_len, speed = synthetic_loop()

    limits = tracking_laws.VehicleLimits(
        wheelbase=0.36, max_steering_angle=0.26, max_steering_rate=1.0,
        control_dt=0.025)
    law = tracking_laws.make_law('mpc', limits, {})
    curvature = racing_math.estimate_signed_path_curvature(
        racing_math.smooth_path(xy, 2, True), closed=True)
    path = tracking_laws.PathContext(
        xy=xy, seg_len=seg_len,
        cumulative=racing_math.compute_cumulative_arc_length(seg_len),
        speed_profile=speed, curvature=curvature, closed=True)

    rng = np.random.default_rng(args.seed)
    lat_walk, head_walk, speed_walk = 0.0, 0.0, 1.0
    solve_ms = []
    misses = {}
    iterations = []
    prev_steering = 0.0

    started = time.perf_counter()
    nearest = 0
    for step in range(args.steps):
        # Slowly varying perturbation: lateral within +/-0.5 m, heading within
        # +/-0.3 rad, speed +/-20%.
        lat_walk = float(np.clip(lat_walk + rng.normal(0, 0.01), -0.5, 0.5))
        head_walk = float(np.clip(head_walk + rng.normal(0, 0.01), -0.3, 0.3))
        speed_walk = float(np.clip(speed_walk + rng.normal(0, 0.02), 0.8, 1.2))
        nearest, _ = racing_math.find_nearest_index(
            xy, xy[nearest] + [lat_walk, 0.0], closed=True,
            prev_index=nearest, search_window=40)
        bx, by = xy[nearest]
        # Walk nearest forward at the profile speed for a 25 ms step.
        nearest = racing_math.find_lookahead_index(
            seg_len, nearest, float(speed[nearest]) * 0.025, closed=True)
        inp = tracking_laws.TrackingInput(
            x=float(bx + lat_walk), y=float(by),
            yaw=float(head_walk), speed=float(speed[nearest]) * speed_walk,
            nearest_idx=nearest, previous_steering=prev_steering)
        t0 = time.perf_counter()
        try:
            result = law.steer(inp, path)
            solve_ms.append((time.perf_counter() - t0) * 1e3)
            prev_steering = result.steering
        except tracking_laws.MpcMiss as exc:
            reason = str(exc).split(':')[0]
            misses[reason] = misses.get(reason, 0) + 1
    total = time.perf_counter() - started

    solve_ms = np.array(solve_ms)
    report = {
        'environment': environment(),
        'steps': args.steps,
        'solves': int(solve_ms.size),
        'wall_time_s': round(total, 2),
        'solve_ms': {
            'p50': round(float(np.percentile(solve_ms, 50)), 3),
            'p90': round(float(np.percentile(solve_ms, 90)), 3),
            'p99': round(float(np.percentile(solve_ms, 99)), 3),
            'p99.9': round(float(np.percentile(solve_ms, 99.9)), 3),
            'max': round(float(solve_ms.max()), 3),
        },
        'misses': misses,
        'g0_gate_ms': 8.0,
        'g0_pass': bool(np.percentile(solve_ms, 99) < 8.0),
    }
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + '\n')
    return 0 if report['g0_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
