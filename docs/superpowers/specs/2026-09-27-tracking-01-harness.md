# Path-tracking laws — 01: comparison harness

**Status:** approved design, 2026-09-27. Not implemented. **Read [00 overview](2026-09-27-tracking-00-overview.md) first.**
**Depends on:** 00 Units 1–2 (`racing_math` additions, `tracking_laws.py` with at least `PurePursuitLaw`). Build this spec **before** specs 02–04 are tuned: it is how they are tuned and judged.

## Why

The research compared four new controllers only against each other, at 10 Hz, in a homemade plant with no walls. What we need to know is simpler and harder: **does law X beat our pure pursuit, in our simulator, at our 40 Hz rate, without hitting anything?** `tools/f1tenth_sim/run_validation.py` already runs pure pursuit through F1TENTH Gym with the calibrated fidelity plant and its own wall/chassis contact check (`sim_fidelity/plant.py`, `FidelityPlant`). This spec makes it run any law and produce a verdict.

## Changes to `tools/f1tenth_sim/run_validation.py`

1. **One copy of the law.** `PathFollower.command` currently re-implements pure pursuit. Give `PathFollower.__init__` a `law: TrackingLaw` argument (built with `tracking_laws.make_law`) and a `PathContext` built from the `PathPlan` exactly as the node builds it (signed curvature via `estimate_signed_path_curvature(smooth_path(xy, 2, closed=True))`). `command()` keeps its nearest-index search and its `target_override` path (overtaking stays pure pursuit, as in the node) and calls `law.steer(...)` otherwise. `TrackingInput.speed` = `abs(state velocity)` (the harness always has "fresh odometry"); `previous_steering` = the `CommandShaper`'s `previous_steering`.
   - **Refactor proof, before anything else:** run `python3 tools/f1tenth_sim/run_validation.py --scenario pure --repeat-seeds 1 --output /tmp/before.json` on the unmodified tree, refactor, run again. Every metric of every `pure_solo` row must be identical. Record both commands and the result in the commit message. If any number moves, the refactor changed pure pursuit — fix it, don't explain it.
2. **Law failure mirrors the node.** A `TrackingLawError` (or any exception from `steer`) → that tick commands speed 0 and increments `law_fail_steps`. MPC's miss/fallback policy (spec 04) is mirrored exactly in the harness too, using the same counter-and-latch rules; the latch clears only at the end of a run (there is no LB in the sim).
3. **New arguments.**
   - `--steering-law {pure_pursuit,stanley,lqr,mpc}` (default `pure_pursuit`) — applies to the `pure` and `traffic` scenarios.
   - `--law-param KEY=VALUE` (repeatable) — overrides a law parameter, e.g. `--law-param stanley.gain=1.2`. Unknown keys are an error.
   - `--compare LAW [LAW ...]` — runs `pure_pursuit` plus each named law, same tracks × seeds × raceline × fidelity, and prints the comparison table and verdicts below. Implies `--scenario pure`.
4. **New per-run metrics** (added to the `pure_solo` result dict; measured against *truth*, as `max_cross_track_m` already is):
   `mean_cross_track_m`, `p95_cross_track_m`, `lap_time_s` (sim time at which `lap_counts` first reaches 1; `null` if never), `law_fail_steps`, `law_compute_ms_p50`, `law_compute_ms_p99` (wall clock around `steer()` on the sim host — label it **host timing, not Jetson** in the output), and for MPC `mpc_miss_steps`, `mpc_fallback_steps`, `mpc_latched` (bool).
5. **Forbid the untrustworthy verdict.** `--compare` refuses `--fidelity legacy` (exit non-zero with a message citing `CLAUDE.md`'s note on gym's collision flag). Under the `car` profile the `collision` field is set by `FidelityPlant`'s own `_wall_contact`/`_chassis_contact` (`plant.py:424–439`), which is the independent check `TEST_QUALITY_STANDARDS.md` A7 requires. Add an assertion in `--compare` that `result["fidelity_profile"] == "car"` for every row.

## The promotion rule — `tools/f1tenth_sim/law_compare.py` (new, pure Python)

Keep the verdict out of `run_validation.py` so it can be tested without gym:

```python
def verdict(baseline_rows: list[dict], candidate_rows: list[dict]) -> Verdict: ...
```

Rows are matched on `(track, seed, raceline)`; a missing or extra match is a `ValueError` (never silently compare different sets). A candidate is **PROMOTE** only if all hold:

1. **Safety, every run:** `laps >= 1`, `collision == False`, `max_cross_track_m < 0.5`, `law_fail_steps == 0`, and (MPC) `mpc_latched == False`.
2. **Better on at least one**, aggregated over all matched runs: mean of `mean_cross_track_m` strictly lower than pure pursuit's, **or** mean of `lap_time_s` strictly lower.
3. **Not more than 5 % worse on the other:** `candidate ≤ 1.05 × baseline` for both aggregated metrics.

Otherwise **REJECT**, with every failed condition listed (not just the first). If the *baseline* itself fails condition 1 on some run, the verdict is **INVALID** (the comparison says nothing) — report it, do not drop the run.

Output: a Markdown table (law × track: mean/p95/max cross-track, lap time, fails, host compute p99) plus the verdict lines, and the full rows in `--output` JSON.

**Tests** — `tools/f1tenth_sim/tests/test_law_compare.py` (new dir; pure Python, runs unsourced with `python3 -m pytest tools/f1tenth_sim/tests/`). Add that command to `docs/sim-validation.md` and to the A9 section of `TEST_QUALITY_STANDARDS.md` — it is not collected by any existing command (A9). Build rows by hand; every expected verdict follows from the three conditions:
- better cross-track, lap time +4 % → PROMOTE; lap time +6 % → REJECT naming condition 3; exactly +5 % → PROMOTE (boundary) and +5.0001 % → REJECT.
- equal on both → REJECT (condition 2 is strict).
- one collision in 15 runs, otherwise far better → REJECT naming the run.
- `lap_time_s = None` on a candidate run → REJECT (fails `laps >= 1`); on a baseline run → INVALID.
- mismatched `(track, seed)` sets → `ValueError(match="unmatched")`.
- `NaN` metric → `ValueError(match="non-finite")`.
Stub-swap `verdict` to always return PROMOTE, and to use `<` where `≤` belongs, and watch the boundary tests fail.

## Protocol for tuning and judging (write it into `docs/sim-validation.md`)

- **Tuning seeds and judging seeds are different.** Tune a law's parameters with `--seed 1000 --repeat-seeds 5`; judge with `--seed 12345 --repeat-seeds 5`. A law tuned and judged on the same seeds has been fitted to them.
- **Judging configuration** (the one that gates promotion): `--compare <law> --fidelity car --raceline shipped --tracks Spielberg Silverstone BrandsHatch --seed 12345 --repeat-seeds 5`. Also run and *report* (not gating) `--raceline centerline`, the closest stand-in for a hand-recorded line; a law that promotes on shipped lines but falls apart on the centerline must be called out in the write-up.
- Pure pursuit is judged with its **shipped** parameters from `pure_pursuit.yaml` — do not re-tune the baseline to make a candidate look good or bad.
- Parameters tuned here are written into the law's params dataclass and `pure_pursuit.yaml` together (the 00 mirror test enforces it), in the same commit as the judging results.
- Results: commit the judging JSON as `docs/tracking-law-comparison.json` and a short results section in `docs/sim-validation.md` stating the exact command, commit, host, and verdicts.

## Acceptance for this spec

- Refactor proof: identical `pure_solo` metrics before/after.
- `law_compare` tests green and seen failing against stubs.
- `--compare pure_pursuit` (pure pursuit against itself) → REJECT on condition 2 with identical metrics — a built-in sanity check that the harness is deterministic and the rule is strict. Put it in the docs as the first command a user runs.
