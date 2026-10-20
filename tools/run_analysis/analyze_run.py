#!/usr/bin/env python3
"""Analyse a recorded gap_follow run, end to end, in one command.

    source /opt/ros/jazzy/setup.bash          # first time only: reads the bag
    python3 tools/run_analysis/analyze_run.py ~/.ros/racerbot_runs/<ts>

Writes ``<run>/analysis/report.md`` and plots beside it, and prints the
report. For every LB hold in the run:

1. what the car did -- distance, speeds, how much of the time it was
   commanded to zero;
2. how it actually moved (LiDAR scan matching), and from that the speed and
   steering lags, steering gain, grip and odometry scale (actuation.py);
3. a replay of every scan through the simulator's copy of gap_follow, with
   how closely it reproduces the published commands (replay.py) -- and, when
   it does, every stop explained: which layer stopped the car, and for the
   TTC brake, where the tripping obstacle was relative to the car's path;
4. optionally the same replay with parameter overrides (``--set name=value``),
   to count which stops a change would have removed on these exact scans.

Steps 2-4 run unsourced once ``analysis/run.npz`` exists.
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from run_analysis import actuation, replay  # noqa: E402
from run_analysis.runlog import analysis_dir, load_run, read_events  # noqa: E402
from run_analysis.scan_motion import scan_motion  # noqa: E402


def parse_overrides(items) -> dict:
    out = {}
    for item in items or []:
        name, _, raw = item.partition("=")
        low = raw.strip().lower()
        if low in ("true", "false"):
            out[name.strip()] = low == "true"
        else:
            out[name.strip()] = float(raw)
    return out


def summarize_window(run, t0, t1) -> list:
    drive = run.drive[(run.drive[:, 0] >= t0) & (run.drive[:, 0] <= t1)]
    odom = run.odom[(run.odom[:, 0] >= t0) & (run.odom[:, 0] <= t1)]
    dist = float(np.trapz(odom[:, 1], odom[:, 0])) if len(odom) > 1 else 0.0
    zero = int(np.sum(drive[:, 3] == 0.0))
    return [
        f"- held {t1 - t0:.1f} s, {len(drive)} drive commands "
        f"({len(drive) / max(t1 - t0, 1e-9):.1f} Hz)",
        f"- odometry distance {dist:.1f} m, mean {np.mean(odom[:, 1]):.2f} m/s, "
        f"peak {np.max(odom[:, 1]):.2f} m/s",
        f"- zero-speed commands: {zero} ticks ({zero * 0.025:.2f} s)",
    ]


def stop_lines(run, ticks, params=None) -> list:
    out = []
    for cluster in replay.stop_clusters(ticks):
        first = cluster[0]
        idx = ticks.index(first)
        basis = ticks[idx - 1].steering if idx > 0 else 0.0
        line = (f"- t={first.t:6.2f} s, {len(cluster)} ticks, {first.stop_reason}, "
                f"odom {first.odom_speed:.2f} m/s")
        if first.stop_reason == "ttc_brake":
            e = replay.explain_ttc(run, first, basis, params)
            line += (f": TTC {e['ttc']:.3f} s on a beam at {e['beam_deg']:+.0f} deg, "
                     f"{e['range']:.2f} m; obstacle {e['offset_from_path']:.2f} m from the path "
                     f"({e['beyond_body_edge']:+.2f} m beyond the body edge, gate "
                     f"{e['gate_half_width']:.3f} m); body's own path clear for "
                     f"{e['body_path_clear_m']:.2f} m vs stopping distance "
                     f"{e['stopping_distance_m']:.2f} m")
        out.append(line)
    return out or ["- none"]


def plot(run, t0, t1, motion, ticks, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    d = run.drive
    ax[0].plot(d[:, 0] - t0, d[:, 3], label="commanded speed")
    ax[0].plot(motion.t - t0, motion.v, label="actual speed (LiDAR)")
    ax[0].plot(run.odom[:, 0] - t0, run.odom[:, 1], label="odom speed", alpha=0.5)
    for k in ticks:
        if k.stopped:
            ax[0].axvline(k.t, color="r", alpha=0.08)
    ax[0].set_ylabel("m/s")
    ax[0].legend()
    ax[1].plot(d[:, 0] - t0, np.tan(d[:, 2]) / actuation.WHEELBASE, label="commanded curvature")
    ax[1].plot(motion.t - t0, motion.curvature(), label="actual curvature (LiDAR)")
    ax[1].set_ylim(-1.5, 1.5)
    ax[1].set_ylabel("1/m")
    ax[1].legend()
    caps = Counter(k.binding for k in ticks)
    ax[2].plot([k.t for k in ticks], [k.telemetry.get("side_left", np.nan) for k in ticks], label="left wall")
    ax[2].plot([k.t for k in ticks], [-k.telemetry.get("side_right", np.nan) for k in ticks], label="right wall (neg)")
    ax[2].set_ylabel("m")
    ax[2].set_xlabel(f"s since LB press; binding caps {dict(caps)}")
    ax[2].legend()
    for a in ax:
        a.grid(True)
        a.set_xlim(0, t1 - t0)
    fig.tight_layout()
    fig.savefig(path, dpi=70)
    plt.close(fig)


def analyze(run_dir, overrides=None, refresh=False, yaml_path=None) -> str:
    run = load_run(run_dir, refresh=refresh)
    params = None
    config_name = "shipped YAML"
    if yaml_path:
        params = replay.gap_mirror.node_params(Path(yaml_path), fill_legacy=True)
        config_name = str(yaml_path)
    out_dir = analysis_dir(run_dir)
    lines = [f"# Run analysis: {Path(run_dir).expanduser().name}", ""]
    events = read_events(run_dir)
    if events:
        cats = Counter(e.get("category") for e in events)
        lines += [f"events.jsonl: {dict(cats)}", ""]
    windows = run.lb_windows()
    if not windows:
        lines.append("No LB hold in /joy: the car was never driving autonomously.")
    for n, (t0, t1) in enumerate(windows):
        lines += [f"## LB hold {n + 1}: {t1 - t0:.1f} s", ""]
        lines += summarize_window(run, t0, t1)
        motion = scan_motion(run, t0, t1)
        try:
            act = actuation.analyze(motion, run.drive, run.odom, t0, t1)
            lines += ["", "### How the car responded", "", "```", *act.lines(), "```"]
        except ValueError as exc:
            lines += ["", f"(actuation analysis skipped: {exc})"]
        ticks = replay.replay(run, t0, t1, params=params)
        par = replay.parity(ticks)
        lines += ["", f"### Replay through gap_mirror ({config_name})", "", *[f"- {x}" for x in par.lines()]]
        lines += ["", "binding speed caps: " + str(dict(Counter(k.binding for k in ticks)))]
        lines += ["", "### Stops", "", *stop_lines(run, ticks, params)]
        if overrides:
            alt = replay.replay(run, t0, t1, overrides=overrides, params=params)
            stops = sum(k.stopped for k in alt)
            lines += ["", f"### Counterfactual replay with {overrides}", "",
                      f"- zero-speed ticks: {stops} (without the change: {sum(k.stopped for k in ticks)})",
                      "- open-loop: same scans, so where the changed car would have gone is not modelled",
                      "", *stop_lines(run, alt, params)]
        plot_path = out_dir / f"hold{n + 1}.png"
        plot(run, t0, t1, motion, ticks, plot_path)
        lines += ["", f"![hold {n + 1}]({plot_path.name})", ""]
    text = "\n".join(lines) + "\n"
    (out_dir / "report.md").write_text(text)
    return text


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--set", action="append", default=[],
                    help="counterfactual replay with this gap_follow parameter changed (repeatable)")
    ap.add_argument("--refresh", action="store_true", help="re-read the bag even if run.npz exists")
    ap.add_argument("--yaml", help="gap_follow.yaml the run actually used (default: the tree's). "
                                   "An older file is accepted: parameters it predates take the "
                                   "value that reproduces the old behaviour")
    args = ap.parse_args(argv)
    print(analyze(args.run_dir, parse_overrides(args.set), args.refresh, args.yaml))
    return 0


if __name__ == "__main__":
    sys.exit(main())
