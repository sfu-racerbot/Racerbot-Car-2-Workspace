# Path-tracking laws — 04: linear time-varying MPC (OSQP), staged

**Status:** approved design, 2026-09-27. Not implemented. **Read [00 overview](2026-09-27-tracking-00-overview.md) and [03 LQR](2026-09-27-tracking-03-lqr.md) first** — this law is LQR's model plus preview and constraints, and reuses `LqrGainTable`. Judged by [01 harness](2026-09-27-tracking-01-harness.md).

## Stages — each is a hard gate; do not start the next until the previous passes and the user has seen the result

| Gate | What | Pass criterion | If it fails |
|---|---|---|---|
| **G0** | Install OSQP on the Jetson; implement `MpcLaw`; benchmark solve time on the Jetson | 99th-percentile solve wall time **< 8 ms** with the simulator stack running (below) | Stop. Report numbers. Options (shorter horizon, fewer iterations, condensed vs sparse) go to the user; do not wire anything into the node. |
| **G1** | Harness judging run (spec 01) | **PROMOTE** | Commit results, record REJECT, stop. |
| **G2** | Node wiring (00 Unit 3 + this spec's fallback), node tests, then 00's rollout order | Node tests green; on wheels-off, in-node p99 < 8 ms and `mpc_miss_latched` never fires | Stop and report. |

Note: the `MpcLaw` unit tests below are written in G0, not later — the benchmark is meaningless on an untested solver.

## What it is (for the docs, in plain words)

LQR looks only at the error *now*. MPC ("model predictive control") also looks at the next second of the line: it plans 20 steering moves 0.05 s apart that keep the car on the line through the bends ahead, **while never exceeding the servo's angle or speed limits**, applies the first, and re-plans next tick. The two things pure pursuit cannot do — see a bend coming precisely, and respect the steering rate as a hard limit instead of discovering it afterwards — are the whole reason to try it.

What this MPC deliberately is *not*: it does not plan speed (speed stays with the profile and the node's caps, 00), it does not know about walls or opponents (the reactive net still has the final say), and it models no tyre slip.

## Formulation

Prediction step `dt_p` (`mpc.dt`, default 0.05 s), horizon `N` (`mpc.horizon_steps`, default 20 → 1.0 s preview). Solved every control tick.

**Reference along the line.** From the rear-axle projection (`project_onto_path`, as LQR): `s_0 = proj.arc_length`, `v_0 = max(inp.speed, v_floor)`. For `k = 0 … N−1`: `s_{k+1} = s_k + v_k·dt_p` (modulo track length when closed; clamped at the end when open), `v_{k+1} = max(profile speed at s_{k+1}, v_floor)`. At each `s_k`: signed curvature `κ_k` and `δ̄_k = atan(L·κ_k)` (the feedforward, and the linearisation point). Sampling "at arc length s" is a helper `sample_at_arc(values, cumulative, s, closed)` in `tracking_laws.py`, linear between waypoints. `v_floor = 0.1`.

**Dynamics** (state `x_k = [e_k, θ_k]`, 00 conventions; decision variable `δ_k`, absolute steering). Kinematic bicycle, `tan δ` linearised at `δ̄_k`:

```
e_{k+1} = e_k + v_k·dt_p·θ_k
θ_{k+1} = θ_k + v_k·dt_p·sec²(δ̄_k)·(δ_k − δ̄_k) / L
```

(Exact at `δ_k = δ̄_k`, because `tan δ̄_k / L = κ_k` cancels the path's own turning. This is the "time-varying" part: `v_k` and `δ̄_k` change along the horizon.)

**Cost**

```
J = Σ_{k=1}^{N−1} ( q_lat·e_k² + q_head·θ_k² )  +  x_Nᵀ P_N x_N
  + Σ_{k=0}^{N−1} r_steer·(δ_k − δ̄_k)²
  + Σ_{k=0}^{N−1} r_rate·((δ_k − δ_{k−1}) / τ_k)²
```

`δ_{−1} = inp.previous_steering`; `τ_0 = limits.control_dt`, `τ_k = dt_p` for `k ≥ 1`. `P_N = LqrGainTable(q_lat, q_head, r_steer, dt=dt_p, …).riccati_at(v_N)` — the terminal cost that makes a short horizon behave like an infinite one on straights.

**Constraints**

```
|δ_k| ≤ max_steering_angle                          k = 0 … N−1
|δ_k − δ_{k−1}| ≤ max_steering_rate · τ_k           k = 0 … N−1
```

Clamp `inp.previous_steering` into `±max_steering_angle` before use, so the problem is always feasible (there are no state constraints).

**QP form: condensed.** Substitute the dynamics so the only variables are `δ_0 … δ_{N−1}` (N = 20 variables, 2N inequality rows, dense N×N Hessian). The Hessian changes every tick (it depends on `v_k`, `δ̄_k`) but its sparsity pattern doesn't: set OSQP up once with the full upper-triangular pattern, then `update(Px=…, q=…, l=…, u=…)` each tick. **Gotcha:** `scipy.sparse` drops explicit zeros — build the pattern from `np.triu(np.ones((N, N)))` and write the values into `.data` in the pattern's order, or OSQP will reject the update when an entry happens to be 0.

**OSQP settings** (check the names against the pinned version's docs — they changed between 0.6.x and 1.x, e.g. `warm_start`/`warm_starting`, `polish`/`polishing`): `eps_abs = eps_rel = 1e-5`, `max_iter = mpc.max_iter` (default 4000), polishing off, warm starting on, `time_limit = mpc.deadline_sec`, verbose off.

**Accepting a solution.** Measure wall time with `time.perf_counter()` around the whole `steer()` body. Raise `MpcMiss(TrackingLawError)` with a reason string when any of these hold:
- status is anything other than *solved* (`match="status"`; "solved inaccurate", max-iterations and time-limit all count as misses),
- elapsed > `mpc.deadline_sec` (default 0.008) even if solved (`match="deadline"`),
- the solution is non-finite or violates a bound by more than 1e-6 (`match="solution"`).
On a miss, drop the warm start (the next solve is cold). Non-finite **input** is not a miss — raise plain `TrackingLawError`, which the node turns into a stop (00).

**Output.** `δ = δ_0` (already within bounds; clip anyway), `predicted = (δ_0, …, δ_{N−1})` for `/drive_intent` (00 Unit 3 step 6), `detail = f"mpc e={e:+.3f}m th={θ:+.3f}rad solve={ms:.1f}ms it={iters}"`. Keep a `solves_since_reset` counter (00's reset test). `reset()` drops the warm start and zeroes the counter.

**Parameters** (`mpc.*`, launch-time only; validated in the constructor, `ValueError` naming the parameter): `dt` (> 0, ≤ 0.2), `horizon_steps` (int, 2–60), `q_lateral`, `q_heading`, `r_steer` (> 0; start at LQR's starting values), `r_rate` (≥ 0; start 0.01), `deadline_sec` (> 0, default 0.008), `max_iter` (≥ 1, default 4000), `max_consecutive_misses` (int ≥ 1, default 3).

**Dependency.** `import osqp` lazily inside `MpcLaw.__init__`. If it fails and `steering_law == 'mpc'`, the node refuses to start (`RuntimeError` naming the missing package and pointing to this spec's G0). Never fall back silently to another law at startup. Every other law, and all of `tracking_laws.py`'s tests except MPC's, must work without osqp installed.

## Miss policy — shared by node and harness

Put it in `tracking_laws.py` as a small pure class so both use exactly the same rules and it is unit-testable unsourced:

```python
class MissPolicy:
    def __init__(self, max_consecutive: int): ...
    def on_success(self) -> None          # consecutive = 0
    def on_miss(self) -> str              # returns 'fallback' or 'latched'
    def latched(self) -> bool
    def clear(self) -> None               # node: on LB release; harness: never within a run
```

Node behaviour when `steering_law == 'mpc'` (the node also constructs a `PurePursuitLaw`, used only here):
- `MpcMiss` and not latched → `on_miss()`. If it returns `'fallback'`: steer with `PurePursuitLaw` for **this tick only**; `decision_state = 'mpc_fallback'`; the reason is in the detail. Everything downstream (reactive net, shaping) runs as normal.
- If it returns `'latched'`, or the policy is already latched → `_stop('mpc_miss_latched', ...)`, every tick, until LB is released; clear the policy in the deadman-not-engaged branch at the top of `_control_step`, next to `_off_line_recovery_exhausted = False`.
- Successful solve → `on_success()`.
- Any other exception → 00's generic `steering_law_failed` stop.
- Every 5 s while running, log one line: `mpc timing: n=…, p50=…ms, p99=…ms, max=…ms, misses=…, fallbacks=…, latches=…` (a rolling window; this is what G2 reads). Add `mpc_fallback` and `mpc_miss_latched` to `race_diagnostics` states (00 Unit 3 step 5).

## G0 — install and benchmark

1. **Install.** On 2026-09-27, `apt-cache policy python3-osqp` returned nothing on this Jetson (no apt package), and osqp was not importable. Find out whether PyPI has an aarch64 wheel for the current osqp release (don't build from source without asking), and whether rosdep has a key for it (`rosdep resolve python3-osqp-pip`). **Stop and ask the user before installing** — present: the exact version, `pip install --user` vs a venv with `--system-site-packages` (ROS runs on the system Python, Ubuntu 24.04's PEP 668 blocks plain pip), and the package.xml line you'll add (`<exec_depend>` with the rosdep key if it exists; otherwise document the install step in `docs/operations.md` and the package README). Pin the major.minor version.
2. **Benchmark script** `tools/mpc_benchmark/benchmark_mpc.py` (pure Python + osqp, no ROS). Workload that exercises warm starting as driving does: take a closed profiled line (`--csv PATH`, a real one from `~/.ros/racerbot_auto/<ts>/` if present; otherwise a built-in synthetic loop — two 1.5 m-radius hairpins joined by straights with a chicane — profiled with `racing_math.compute_velocity_profile` at `v_max = 4.0`, `a_lat_max = 2.5`). Walk along it in 25 ms steps at the profile speed for 20,000 steps, with the pose perturbed by a slowly varying random walk (lateral within ±0.5 m, heading within ±0.3 rad, speed ±20 %, seeded), calling `steer()` each step. Report p50/p90/p99/p99.9/max solve time, misses by reason, and iterations p50/p99.
3. **Record the environment** in the output: `nvpmodel -q`, whether `jetson_clocks` is active, the CPU governor, the osqp, numpy and Python versions, and the git commit.
4. **Run it twice**: idle, and **under load** — with the ROS simulator stack (`docs/ros-simulator.md`) plus `pure_pursuit_node` (with `steering_law: pure_pursuit`) running on the same Jetson. The gate is the loaded run's p99 < 8 ms. Commit both reports as `docs/mpc-benchmark-<date>.json`, plus a short section in `docs/sim-validation.md`.

## Tests

**`MpcLaw`** (`src/pure_pursuit/test/test_tracking_laws.py`; the MPC tests use `pytest.importorskip("osqp", reason="spec 04 G0: osqp not installed")` — the only allowed skip, and it becomes a hard failure once osqp is in `package.xml`: add a check that `tracking_laws` can import osqp whenever `package.xml` lists it). Tests set `eps_abs = eps_rel = 1e-8` and a generous deadline unless the test is about the deadline.

| # | Test | Oracle | Catches |
|---|---|---|---|
| 1 | **LQR equivalence.** Straight line, constant profile speed 1.5 m/s, `dt_p = control_dt = 0.025`, `r_rate = 0`, limits widened (angle 10 rad, rate 1e6 rad/s), `x_0 = [0.05, 0.02]`, `N ∈ {5, 20}` | `δ_0 = −K·x_0`, `K = LqrGainTable(dt=0.025).gain_at(1.5)` (itself tested against the Riccati recursion in spec 03), `abs=1e-5  # rad` | wrong condensing, wrong terminal cost, sign errors, dt mix-ups — any N must agree, because an unconstrained finite-horizon problem with the DARE terminal cost *is* LQR |
| 2 | Angle bound active: `e = 2.0 m`, rate limit relaxed | `δ_0 = −0.26`, `abs=1e-6` | missing box constraint |
| 3 | Rate bound active: `previous_steering = 0`, `e = 1.0 m`, rate 1.0 rad/s, `control_dt = 0.025` | `δ_0 = −0.025`, `abs=1e-6` | missing/mis-scaled rate constraint, `τ_0 = dt_p` bug (would allow 0.05) |
| 4 | Preview sign: car on a straight, aligned; a left arc (R = 2 m) begins 0.2 m ahead vs a right arc vs no arc, v = 2 m/s, default weights | `δ_0(left) > 0 = δ_0(straight) > δ_0(right)` | preview ignored, curvature sign |
| 5 | Steady circle R = 2 m CCW, on it and tangent, `previous_steering = atan(L/R)` | `δ_0 = atan(0.36/2.0) = 0.178093`, `abs=2e-3  # rad, 0.02 m waypoints` | feedforward / linearisation point |
| 6 | Linearisation: finite difference of the nonlinear update `θ + v·dt_p·(tan δ/L − κ)` in `δ` at `δ̄ = 0.2` vs the model's `v·dt_p·sec²(δ̄)/L` | FD with step 1e-7, `rel=1e-5` | missing `sec²`, wrong `L` |
| 7 | Mirror invariant (as spec 02 test 10) | `δ(mirror) = −δ` | asymmetric bugs |
| 8 | `deadline_sec = 1e-9` → `MpcMiss(match="deadline")`; `max_iter = 1` on a non-trivial state → `MpcMiss(match="status")`; after a miss the next solve is cold (warm-start flag observable on the law) | real solver, no mocks | misses reported as successes |
| 9 | Non-finite input → `TrackingLawError`, **not** `MpcMiss` | exception type | a bug turned into a silent fallback |
| 10 | Parameter validation (each bound in the list above, and one step past it) | `ValueError` naming the parameter | |
| 11 | `predicted` has length N, all finite, all within bounds; consecutive differences within the rate bound | invariant | |
| 12 | Open path, horizon running past the end | finite; `s_k` clamped | end-of-path crash |
| 13 | `reset()` → `solves_since_reset == 0` and the next solve is cold | | |

**`MissPolicy`** (unsourced): 1 miss → `'fallback'`; `max_consecutive` misses in a row → `'latched'`; a success in between resets the count; latched stays latched through successes until `clear()`; `max_consecutive = 1` latches on the first miss.

**Node** (`test_tracking_law_integration.py`, ROS-sourced, `drive_topic:=/test_only/drive` always, deadman **enabled** the `test_opponent_integration.py:842` way, `mpc.deadline_sec:=1e-9` to force misses):
- tick 1 with LB held: published speed > 0 and published steering equals `PurePursuitLaw` on the same input (before shaping — compare `desired_steering` in the published `/drive_intent`, or compare after shaping with a shaper replica; say which in the test); state `mpc_fallback`.
- after `max_consecutive_misses` ticks: published speed 0, state `mpc_miss_latched`; still 0 on further ticks with LB held.
- release LB (speed stays 0), hold again: the next tick is `mpc_fallback` again (the latch cleared), not `mpc_miss_latched`.
- the four deadman deny cases from 00, with `steering_law = mpc` and a normal deadline.

Mutations to show caught (Hard rule 1): drop the rate constraint; `τ_0 = dt_p`; terminal cost `P_N = 0` (test 1 with N = 5 must catch it); drop `sec²`; treat "solved inaccurate" as solved; `MissPolicy.on_success` not resetting; not clearing the latch on LB release (node test).

## After G2 — when to revisit NMPC

NMPC (acados, nonlinear bicycle) is out of scope. Reopen it only when all hold: this MPC is promoted and has run on the car; `race_diagnostics` runs show tracking error growing with lateral acceleration (i.e. the model, not the tuning, is the limit); tyre and servo parameters in `tools/f1tenth_sim/sim_fidelity/calibration.py` are marked *measured*, not stock/estimated; and a G0-style Jetson benchmark exists for an acados-generated solver.
