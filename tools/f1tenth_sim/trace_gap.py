#!/usr/bin/env python3
"""Per-tick diagnostic trace of the gap_follow pipeline on one track.

Dumps a CSV of everything the controller decided each tick -- position, yaw,
measured/commanded speed, steering, every speed cap's value, the chosen gap's
geometry, side-wall distances, and which cap bound -- so the slow/stuck/tight
behaviour can be attributed to a specific layer instead of guessed at.

It drives eval_gap.GapController, the same controller eval_gap.py scores, so
the trace and the pass/fail numbers always describe one pipeline. (An earlier
version re-implemented the pipeline a third time and had silently dropped
corridor centering, the creep reserve and the TTC/emergency escape.)

    python3 tools/f1tenth_sim/trace_gap.py --track indoor_tight --out /tmp/trace.csv
    python3 tools/f1tenth_sim/trace_gap.py --track indoor_tight --plot /tmp/trace.png
    python3 tools/f1tenth_sim/trace_gap.py --track asb_10000 --set enable_corner_setup=false
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_gap import (  # noqa: E402
    CONTROL_DT, GapController, components_half_fov, components_increment,
    components_range_min, make_plant, parse_set)

import numpy as np  # noqa: E402

COLUMNS = (
    "step", "t", "x", "y", "yaw_deg", "wheel_angle", "speed_meas", "cmd_speed", "steering",
    "desired_steering", "target_deg", "target_distance", "gap_width_deg",
    "fallback", "creeping", "centering_bias", "corner_setup_bias",
    "binding", "stop_reason", "curve_cap", "clearance_cap", "corner_cap",
    "anticipation_cap", "fwd_clear", "arc_clear", "min_clear", "min_ttc",
    "side_left", "side_right", "width_factor", "contact",
)


def _fmt(value):
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            return "inf" if value > 0 else "-inf"
        return round(value, 4)
    return value


def trace(track, params, seed, timeout_s):
    plant = make_plant(track, seed)
    line = plant.env.unwrapped.track.raceline
    obs, _ = plant.reset(np.array([[line.xs[0], line.ys[0], line.yaws[0]]]))

    class _Lidar:
        pass
    lid = _Lidar()
    lid.angle_min = float(-components_half_fov())
    lid.angle_increment = float(components_increment())
    lid.range_min = float(components_range_min())
    ctrl = GapController(params, lid)

    rows = []
    laps = 0.0
    for step in range(int(math.ceil(timeout_s / CONTROL_DT))):
        ego = obs["agent_0"]
        speed_meas = float(np.asarray(ego["std_state"], dtype=float)[3])
        steering, cmd_speed = ctrl.command(ego["scan"], speed_meas, step,
                                          horizontal_accel=0.0, lb_pressed=True,
                                          yaw_rate=float(ego["std_state"][5]),
                                          fused_speed=float(plant.truth(0)[3]))
        truth = plant.truth(0)
        tel = ctrl.telemetry
        row = dict(
            step=step, t=step * CONTROL_DT, x=float(truth[0]), y=float(truth[1]),
            # std_state is [x, y, delta, v, yaw, yaw_rate, beta]: index 2 is the
            # road-wheel angle, not yaw.
            yaw_deg=math.degrees(float(truth[4])), wheel_angle=float(truth[2]),
            speed_meas=speed_meas,
            cmd_speed=cmd_speed, steering=steering,
            desired_steering=tel.get("desired_steering", ""),
            target_deg=(math.degrees(tel["target_angle"])
                        if "target_angle" in tel else ""),
            target_distance=tel.get("target_distance", ""),
            gap_width_deg=(math.degrees(tel["gap_width"])
                           if "gap_width" in tel else ""),
            fallback=tel.get("fallback", ""), creeping=tel.get("creeping", ""),
            centering_bias=tel.get("centering_bias", ""),
            corner_setup_bias=tel.get("corner_setup_bias", ""),
            binding=tel.get("binding", ""), stop_reason=tel.get("stop_reason", ""),
            curve_cap=tel.get("curve_cap", ""),
            clearance_cap=tel.get("clearance_cap", ""),
            corner_cap=tel.get("corner_cap", ""),
            anticipation_cap=tel.get("anticipation_cap", ""),
            fwd_clear=tel.get("fwd_clear", ""), arc_clear=tel.get("arc_clear", ""),
            min_clear=tel.get("min_clear", ""), min_ttc=tel.get("min_ttc", ""),
            side_left=tel.get("side_left", ""), side_right=tel.get("side_right", ""),
            width_factor=tel.get("width_factor", ""), contact=0,
        )
        rows.append({k: _fmt(v) for k, v in row.items()})
        obs, _r, done, _t, info = plant.step([(steering, cmd_speed)])
        laps = float(info["lap_counts"][0])
        if plant.chassis_contact_steps > 0 or obs["agent_0"]["collision"]:
            rows[-1]["contact"] = 1
            break
        if done or laps >= 1.0:
            break
    plant.close()
    return rows, laps


def plot(rows, track, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image
    import yaml

    from eval_gap import track_map_yaml
    fig, ax = plt.subplots(figsize=(10, 8))
    meta_path = track_map_yaml(track)
    if meta_path is not None:
        meta = yaml.safe_load(meta_path.read_text())
        image = np.asarray(Image.open(meta_path.parent / meta["image"]))
        res = float(meta["resolution"])
        ox, oy = meta["origin"][:2]
        h, w = image.shape[:2]
        ax.imshow(image, cmap="gray", origin="upper",
                  extent=(ox, ox + w * res, oy, oy + h * res))
    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    sc = ax.scatter(xs, ys, c=[r["cmd_speed"] for r in rows], s=6, cmap="viridis")
    contact = [r for r in rows if r["contact"]]
    if contact:
        ax.plot(contact[-1]["x"], contact[-1]["y"], "rx", ms=14, mew=3)
    fig.colorbar(sc, label="commanded speed (m/s)")
    ax.set_aspect("equal")
    ax.set_title(f"gap_follow trace: {track}")
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", default="indoor_tight")
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--set", action="append", default=[],
                    help="override a parameter, name=value (repeatable)")
    ap.add_argument("--out", help="write the per-tick CSV here")
    ap.add_argument("--plot", help="write a PNG of the driven path here")
    args = ap.parse_args(argv)
    rows, laps = trace(args.track, parse_set(args.set), args.seed, args.timeout)
    if args.out:
        with open(args.out, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(COLUMNS))
            writer.writeheader()
            writer.writerows(rows)
        print(f"wrote {len(rows)} rows to {args.out}")
    if args.plot:
        plot(rows, args.track, args.plot)
        print(f"wrote {args.plot}")
    contact = any(r["contact"] for r in rows)
    print(f"track={args.track} laps={laps:.2f} ticks={len(rows)} contact={contact}")


if __name__ == "__main__":
    main()
