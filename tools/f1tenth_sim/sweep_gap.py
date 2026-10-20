#!/usr/bin/env python3
"""Run eval_gap over several tracks x seeds in parallel and summarise.

    python3 tools/f1tenth_sim/sweep_gap.py --track all --seeds 5
    python3 tools/f1tenth_sim/sweep_gap.py --track indoor --seeds 8 --set enable_arc_guard=false

One process per (track, seed). Prints one line per track: passes, mean lap
time over passing runs, worst minimum body clearance, total stops. Exit code 0
when every run passes.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_gap  # noqa: E402


def _one(job):
    track, seed, params, timeout, reverse = job
    return eval_gap.run_track(track, params, seed, timeout, reverse)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", default="all")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--first-seed", type=int, default=12345)
    ap.add_argument("--timeout", type=float, default=240.0)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--output")
    ap.add_argument("--reverse", action="store_true",
                    help="drive each track the other way round (right turns)")
    args = ap.parse_args(argv)
    tracks = eval_gap.select_tracks(args.track)
    params = eval_gap.parse_set(args.set)
    jobs = [(t, args.first_seed + k, params, args.timeout, args.reverse)
            for t in tracks for k in range(args.seeds)]
    for track in tracks:
        eval_gap.track_path(track)  # build layouts once, before forking
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(_one, jobs))
    all_pass = True
    for track in tracks:
        runs = [r for r in results if r["track"] == track]
        passed = [r for r in runs if r["passed"]]
        all_pass &= len(passed) == len(runs)
        lap = (sum(r["sim_time_s"] for r in passed) / len(passed)) if passed else math.nan
        clear = [r["min_body_clearance_m"] for r in runs if r["min_body_clearance_m"] is not None]
        print(f"{track:14s} pass {len(passed)}/{len(runs)}  lap {lap:7.2f}s  "
              f"vmean {sum(r['mean_speed'] for r in runs)/len(runs):.2f}  "
              f"min_clear {min(clear) if clear else float('nan'):.3f}m  "
              f"stops {sum(r['stop_steps'] for r in runs)}  "
              f"shed {sum(r['speed_shed_mps'] for r in passed) / max(1, len(passed)):5.1f}  "
              f"hardbrk {sum(r['hard_brake_ticks'] for r in passed) / max(1, len(passed)):5.1f}  "
              f"contact {sum(1 for r in runs if r['collision'])}", flush=True)
    if args.output:
        Path(args.output).write_text(json.dumps(results, indent=2) + "\n")
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
