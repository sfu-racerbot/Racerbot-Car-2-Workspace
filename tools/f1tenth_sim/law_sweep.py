#!/usr/bin/env python3
"""Parallel steering-law comparison: the same runs and the same verdict as
``run_validation.py --compare``, spread over worker processes.

    # tune on the tuning seeds (never the judging ones)
    python3 tools/f1tenth_sim/law_sweep.py --laws stanley --seed 1000 \\
        --law-param stanley.gain=2.0
    # judge
    python3 tools/f1tenth_sim/law_sweep.py --laws stanley lqr mpc --seed 12345 \\
        --output docs/tracking-law-comparison.json

Every run is run_validation.run_pure_solo on the ``car`` fidelity profile;
the verdict is law_compare.verdict against pure pursuit on the same tracks x
seeds. Pure pursuit always runs at its shipped parameters.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import law_compare  # noqa: E402
import run_validation as rv  # noqa: E402

TRACKS = ("Spielberg", "Silverstone", "BrandsHatch")


def _one(job):
    law, track, seed, params, timeout = job
    result = rv.run_pure_solo(track, seed, timeout, law_name=law,
                              law_params=params if law != "pure_pursuit" else {})
    result["seed"] = seed
    result["raceline"] = rv.RACELINE_SOURCE
    return result


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--laws", nargs="+", required=True)
    ap.add_argument("--tracks", nargs="+", default=list(TRACKS))
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--repeat-seeds", type=int, default=5)
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--law-param", action="append", default=[])
    ap.add_argument("--output", type=Path)
    ap.add_argument("--baseline-cache", type=Path,
                    help="reuse pure pursuit's rows from this JSON if it matches "
                         "the tracks x seeds, else run them and write it")
    args = ap.parse_args(argv)

    params = rv._parse_law_params(args.law_param)
    seeds = list(range(args.seed, args.seed + args.repeat_seeds))
    laws = ["pure_pursuit"] + [law for law in args.laws if law != "pure_pursuit"]
    cached = []
    if args.baseline_cache and args.baseline_cache.exists():
        cached = json.loads(args.baseline_cache.read_text())
        wanted = {(t, s) for t in args.tracks for s in seeds}
        if {(r["track"], r["seed"]) for r in cached} != wanted:
            cached = []
    run_laws = laws[1:] if cached else laws
    jobs = [(law, t, s, params, args.timeout) for law in run_laws for t in args.tracks for s in seeds]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = cached + list(pool.map(_one, jobs))
    if args.baseline_cache and not cached:
        args.baseline_cache.write_text(json.dumps(
            [r for r in results if r["steering_law"] == "pure_pursuit"]) + "\n")

    rows = {law: [r for r in results if r["steering_law"] == law] for law in laws}
    print(f"seeds {seeds[0]}..{seeds[-1]}  params {params or '{}'}")
    for law in laws:
        for track in args.tracks:
            rs = [r for r in rows[law] if r["track"] == track]
            ok = [r for r in rs if r["passed"]]
            laps = [r["lap_time_s"] for r in ok if r["lap_time_s"]]
            print(f"  {law:<12} {track:<12} pass {len(ok)}/{len(rs)}  "
                  f"mean_ct {statistics.mean(r['mean_cross_track_m'] for r in rs):.4f}  "
                  f"max_ct {max(r['max_cross_track_m'] for r in rs):.3f}  "
                  f"lap {statistics.mean(laps) if laps else float('nan'):7.2f}  "
                  f"stops {sum(r['stop_steps'] for r in rs):5d}  "
                  f"avoid {sum(r['avoid_steps'] for r in rs):5d}  "
                  f"contact {sum(1 for r in rs if r['collision'])}  "
                  f"p99ms {max((r['law_compute_ms_p99'] or 0) for r in rs):.2f}")
    verdicts = {}
    for law in laws[1:]:
        v = law_compare.verdict(rows["pure_pursuit"], rows[law])
        verdicts[law] = v.verdict
        print(f"{law}: {v.verdict}  (cross-track {v.candidate_mean_cross_track:.4f} vs "
              f"{v.baseline_mean_cross_track:.4f}, lap {v.candidate_mean_lap_time:.2f} vs "
              f"{v.baseline_mean_lap_time:.2f})")
        for reason in v.reasons[:6]:
            print(f"    - {reason}")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({
            "schema_version": 2, "compare": True, "seed": args.seed,
            "seeds": seeds, "tracks": args.tracks, "raceline": rv.RACELINE_SOURCE,
            "fidelity_profile": rv.PROFILE.name, "laws": laws,
            "law_params": params, "verdicts": verdicts, "results": results,
        }, indent=2, sort_keys=True) + "\n")
    return 0 if all(v == "PROMOTE" for v in verdicts.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
