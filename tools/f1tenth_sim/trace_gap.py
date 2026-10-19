#!/usr/bin/env python3
"""Per-tick diagnostic trace of the gap_follow pipeline on one track.

Dumps a CSV of everything the controller decided each tick -- position, yaw,
measured/commanded speed, steering, every speed cap's value, the chosen gap's
geometry, side-wall distances, and which cap bound -- so the slow/stuck/tight
behaviour can be attributed to a specific layer instead of guessed at.

    python3 tools/f1tenth_sim/trace_gap.py --track indoor_tight --out /tmp/trace.csv
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_fidelity.bootstrap import bootstrap  # noqa: E402

ROOT = bootstrap()
sys.path.insert(0, str(ROOT / "src" / "racerbot_sim"))

import numpy as np  # noqa: E402

from gap_follow import gap_logic  # noqa: E402
from sim_fidelity.plant import DEFAULT_PROFILE, env_components  # noqa: E402
from sim_fidelity import CALIBRATION, PROFILES  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_gap import DEFAULTS, make_plant  # noqa: E402

CONTROL_DT = 0.025


def trace(track, params, seed, timeout_s, out_path):
    plant = make_plant(track, seed)
    line = plant.env.unwrapped.track.raceline
    obs, _ = plant.reset(np.array([[line.xs[0], line.ys[0], line.yaws[0]]]))
    comp = env_components(PROFILES[DEFAULT_PROFILE], CALIBRATION)["lidar_config"]

    class L:
        pass
    lid = L()
    lid.angle_min = float(comp.angle_min)
    lid.range_min = float(comp.range_min)
    lid.angle_increment = float((comp.angle_max - comp.angle_min) / (comp.num_beams - 1))

    p = dict(params)
    previous_target_idx = None
    steering_basis = 0.0
    last_commanded_speed = 0.0
    last_command_time = None

    rows = []
    max_steps = int(math.ceil(timeout_s / CONTROL_DT))
    for step in range(max_steps):
        ego = obs["agent_0"]
        st = np.asarray(ego["std_state"], dtype=float)
        speed_meas = float(st[3])
        scan = ego["scan"]
        clean, valid = gap_logic.sanitize_ranges(scan, p["max_range"], lid.range_min)
        angles = lid.angle_min + np.arange(clean.size) * lid.angle_increment
        half_span = math.radians(p["centering_side_fov_deg"]) / 2.0
        side_left = gap_logic.side_wall_distance(clean, valid, angles, math.pi / 2, half_span)
        side_right = gap_logic.side_wall_distance(clean, valid, angles, -math.pi / 2, half_span)

        half_fov = math.radians(p["forward_fov_deg"]) / 2.0
        lo = max(0, int((-half_fov - lid.angle_min) / lid.angle_increment))
        hi = min(clean.size - 1, int((half_fov - lid.angle_min) / lid.angle_increment))
        window = clean[lo:hi + 1]
        wv = valid[lo:hi + 1]
        ba = angles[lo:hi + 1]
        bnd = gap_logic.vehicle_boundary_distances(
            ba, p["car_width"], p["car_length"], p["wheelbase"],
            p["laser_offset_x"], p["laser_offset_y"])
        min_clear = gap_logic.minimum_footprint_clearance(window, wv, bnd)
        fwd_clear = gap_logic.minimum_footprint_clearance_in_cone(
            window, wv, ba, bnd, math.radians(p["forward_stop_fov_deg"]))

        # adaptive width
        width_factor = gap_logic.corridor_width_factor(
            side_left, side_right, p["adaptive_width_narrow"], p["adaptive_width_reference"])
        eff_margin = gap_logic.scale_between(width_factor, p["min_safety_margin"], p["safety_margin"])
        eff_corner = gap_logic.scale_between(width_factor, p["corner_speed"], p["corner_speed_wide"])

        # TTC
        cmd_age = math.inf if last_command_time is None else (step - last_command_time) * CONTROL_DT
        eff_speed = gap_logic.conservative_ttc_speed(
            speed_meas, last_commanded_speed, cmd_age,
            p["ttc_command_speed_timeout_sec"], p["ttc_command_fallback_max_odom_speed"])
        min_ttc = math.inf
        if eff_speed >= p["ttc_min_brake_speed"]:
            min_ttc = gap_logic.minimum_ttc(
                window, wv, ba, eff_speed, bnd, p["ttc_min_closing_speed"],
                p["car_width"] / 2.0 + p["safety_margin"],
                math.tan(steering_basis) / p["wheelbase"],
                p["laser_offset_x"], p["laser_offset_y"])

        # gap selection
        closest_idx, closest_dist = gap_logic.closest_valid(window, wv)
        half_width = p["car_width"] / 2.0 + eff_margin
        processed = gap_logic.disparity_extend(window, lid.angle_increment,
                                               p["disparity_threshold"], half_width)
        if closest_idx is not None:
            processed = gap_logic.safety_bubble(processed, closest_idx, closest_dist,
                                                lid.angle_increment, half_width)
        gap_start, gap_end, used_fallback = gap_logic.find_gap_with_fallback(
            processed, p["min_gap_distance"], p["fallback_min_gap_distance"],
            lid.angle_increment, p["min_centerline_gap_width"],
            previous_target_idx=previous_target_idx, switch_margin=p["gap_switch_margin"])

        target_angle = 0.0
        curve_speed = p["max_speed"]
        clearance_speed = p["max_speed"]
        steering = steering_basis
        cmd_speed = 0.0
        binding = "stop"
        gap_w = 0.0
        if gap_start is not None:
            target_in_window = gap_logic.aim_within_gap(processed, gap_start, gap_end, ba)
            previous_target_idx = target_in_window
            target_angle = float(ba[target_in_window])
            gap_w = (gap_end - gap_start + 1) * lid.angle_increment
            desired_steering = float(np.clip(p["steering_gain"] * target_angle,
                                             -p["max_steering_angle"], p["max_steering_angle"]))
            dt = p["command_slew_max_dt"] if last_command_time is None else min(
                p["command_slew_max_dt"], max(0.0, (step - last_command_time) * CONTROL_DT))
            steering = gap_logic.slew_rate_limit(desired_steering, steering_basis, dt,
                                                 p["max_steering_rate"])
            steering_basis = steering
            curv = math.tan(desired_steering) / p["wheelbase"]
            curve_speed = gap_logic.curvature_speed_limit(curv, p["max_lateral_accel"], p["max_speed"])
            normal_speed = max(p["min_speed"], curve_speed)
            clearance_speed = gap_logic.braking_speed_limit(
                fwd_clear, p["forward_stop_clearance"], p["max_braking_decel"], p["max_speed"])
            creeping = fwd_clear <= p["forward_stop_clearance"]
            if creeping:
                clearance_speed = min(p["escape_creep_speed"], clearance_speed)
            desired_speed = min(normal_speed, clearance_speed)
            binding = "curve" if normal_speed <= clearance_speed else "clearance"
            if used_fallback and eff_corner < desired_speed:
                desired_speed = min(desired_speed, eff_corner)
                binding = "corner"
            ramp = max(last_commanded_speed, min(abs(speed_meas), p["max_speed"]))
            accel_ceil = ramp + p["max_acceleration"] * dt
            cmd_speed = min(desired_speed, accel_ceil)
            if accel_ceil < desired_speed:
                binding = "accel"
            last_commanded_speed = cmd_speed
            last_command_time = step
        else:
            last_commanded_speed = 0.0
            last_command_time = step

        truth = plant.truth(0)
        rows.append(dict(
            step=step, t=round(step * CONTROL_DT, 3),
            x=round(float(truth[0]), 3), y=round(float(truth[1]), 3),
            yaw=round(math.degrees(float(truth[2])), 2),
            speed_meas=round(speed_meas, 3), cmd_speed=round(cmd_speed, 3),
            steering=round(steering, 4), target_angle=round(math.degrees(target_angle), 2),
            binding=binding, fallback=int(used_fallback),
            curve_cap=round(curve_speed, 3), clearance_cap=round(clearance_speed, 3),
            corner_cap=round(eff_corner, 3),
            fwd_clear=round(fwd_clear, 3) if math.isfinite(fwd_clear) else "inf",
            min_clear=round(min_clear, 3) if math.isfinite(min_clear) else "inf",
            min_ttc=round(min_ttc, 3) if math.isfinite(min_ttc) else "inf",
            eff_speed=round(eff_speed, 3),
            side_left=round(side_left, 3) if math.isfinite(side_left) else "inf",
            side_right=round(side_right, 3) if math.isfinite(side_right) else "inf",
            width_factor=round(width_factor, 3),
            gap_width_deg=round(math.degrees(gap_w), 1),
        ))
        obs, _r, done, _t, info = plant.step([(steering, cmd_speed)])
        if done or obs["agent_0"]["collision"] or plant.chassis_contact_steps > 0:
            rows.append(dict(step=step + 1, t=round((step + 1) * CONTROL_DT, 3),
                             x=0, y=0, yaw=0, speed_meas=0, cmd_speed=0, steering=0,
                             target_angle=0, binding="CONTACT", fallback=0,
                             curve_cap=0, clearance_cap=0, corner_cap=0,
                             fwd_clear=0, min_clear=0, min_ttc=0, eff_speed=0,
                             side_left=0, side_right=0, width_factor=0, gap_width_deg=0))
            break
        if info["lap_counts"][0] >= 1:
            break
    plant.close()

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {len(rows)} rows to {out_path}; laps={info['lap_counts'][0]}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", default="indoor_tight")
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--out", default="/tmp/opencode/trace.csv")
    args = ap.parse_args(argv)
    trace(args.track, DEFAULTS, args.seed, args.timeout, args.out)


if __name__ == "__main__":
    main()
