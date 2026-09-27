# Path-tracking laws — 03: LQR

**Status:** approved design, 2026-09-27. Not implemented. **Read [00 overview](2026-09-27-tracking-00-overview.md) first.** Judged by [01 harness](2026-09-27-tracking-01-harness.md). Spec [04 MPC](2026-09-27-tracking-04-mpc.md) reuses this spec's model and gain code — build this one first.

## What it is (for the docs, in plain words)

Describe how far the car is from the line (`e`) and how crooked it is (`θ`) as two numbers. For small errors, one control tick later they change in a simple, predictable way that depends on speed and steering. LQR ("linear-quadratic regulator") picks the fixed feedback rule `steering = feedforward − K·[e, θ]` that minimises, over the whole future, "error squared" weighted by `Q` plus "steering effort squared" weighted by `R`. Big `Q` = hug the line; big `R` = gentle wheel. The research's results for LQR were poor, but they used a 10 Hz model on a 40 Hz car and arbitrary weights — this spec fixes the first and the harness tunes the second.

## Model (discrete, at the node's control period)

State `s = [e, θ]` of the **rear axle** (00 conventions: `e` + = left, `θ = wrap(yaw − ψ_p)` + = pointing left). Input `u = δ − δ_ff`, with `δ_ff = atan(L·κ)` at the rear-axle projection. Small-angle kinematic bicycle, forward-Euler at `dt = limits.control_dt` (0.025 s at 40 Hz — **never** the research's 0.1 s):

```
A(v) = [[1, v·dt],
        [0, 1   ]]
B(v) = [[0        ],
        [v·dt / L ]]
```

(Derivation for the docs: `ė = v·sin θ ≈ v·θ`; `θ̇ = v·tan δ / L − v·κ ≈ v·(δ − δ_ff)/L` for small `δ − δ_ff`. The `sec²(δ_ff)` factor from linearising `tan` is ≤ 1.07 at 0.26 rad and is dropped here; spec 04 keeps it.)

Cost `J = Σ sᵀQs + uᵀRu`, `Q = diag(q_lateral, q_heading)`, `R = [r_steer]`.

## The law

```
proj = project_onto_path(xy, seg_len, (x, y), inp.nearest_idx, closed)   # rear axle
e = proj.lateral_error;  θ = wrap(yaw − proj.heading)
κ = interpolate_along(curvature, proj, closed)
K = gain_at(max(inp.speed, v_floor))                                     # 1x2
δ_unclipped = atan(L·κ) − K[0]·e − K[1]·θ
δ = clip(δ_unclipped, ±max_steering_angle)
```

**Gain schedule.** At construction, for each speed on the grid `v_floor, v_floor + 0.05, …, v_grid_max`, solve `P = scipy.linalg.solve_discrete_are(A(v), B(v), Q, R)` and `K = (R + BᵀPB)⁻¹ BᵀPA`. `gain_at(v)` interpolates `K` linearly between grid points and clamps to the end values outside the grid. Implement this as its own class, `LqrGainTable(q_lateral, q_heading, r_steer, dt, wheelbase, v_floor, v_grid_max)`, with public `gain_at(v)` and `riccati_at(v)` (the `P` table, same interpolation). `LqrLaw` builds one with `dt = limits.control_dt`; spec 04 builds its own with its prediction step `dt_p` — a `P` computed at the wrong `dt` is the wrong terminal cost. `v_floor = 0.1 m/s` (the model is uncontrollable at `v = 0`; the floor is for gain design only — speed comes from the node). `scipy` is already a declared dependency (`python3-scipy` in `package.xml`).

Parameters (`lqr.*`, launch-time only): `q_lateral`, `q_heading`, `r_steer` (all must be > 0; constructor raises `ValueError` naming the parameter), `v_grid_max` (default 6.0, must be > `v_floor`). **Starting values** `q_lateral = 1.0`, `q_heading = 1.0`, `r_steer = 1.0`: at 40 Hz these give roughly 0.17–0.20 rad of steering per 0.2 m of lateral error from 0.5 to 4.0 m/s, with closed-loop spectral radius 0.90–0.99 (computed 2026-09-27 for this spec — a starting point, not a tuned value). Tune in the harness on the tuning seeds.

`detail`: `f"lqr e={e:+.3f}m th={θ:+.3f}rad K=[{K0:.2f},{K1:.2f}] ff={δ_ff:+.3f}rad"`. `reset()` is a no-op (the gain table depends only on parameters).

## Tests (`src/pure_pursuit/test/test_tracking_laws.py`, runs unsourced)

Oracles are either independent computations or invariants — never "the value scipy gave us today".

| # | Test | Oracle | Catches |
|---|---|---|---|
| 1 | For v ∈ {0.5, 1.5, 4.0}: `gain_at(v)` equals the fixed point of the Riccati *recursion* `P ← Q + AᵀPA − AᵀPB(R+BᵀPB)⁻¹BᵀPA`, iterated from `P = Q` in the test until the change is < 1e-12, then `K` from it. `rel=1e-6` | independent computation | wrong A/B, wrong K formula, `dt` = 0.1 |
| 2 | Closed-loop stability: spectral radius of `A(v) − B(v)K(v)` < 1 at every grid speed | invariant | sign error in K (unstable), A/B transposed |
| 3 | Local optimality at v = 1.5: simulate the linear system from `s0 = [0.2, 0.05]` for 2000 steps under `u = −Ks`, and under `u = −(K + ε·dK)s` for ε = ±0.05 along both gain components; the cost is lowest at ε = 0 | invariant (definition of LQR) | a stable but non-optimal K |
| 4 | Straight path, rear axle at `(0, 0.2)`, yaw 0, v = 1.5 | `δ = −K[0]·0.2` with `K` from test 1's recursion, and `δ < 0` | sign |
| 5 | Straight, `e = 0`, yaw = +0.1 | `δ = −K[1]·0.1 < 0` | heading sign / missing wrap |
| 6 | Heading wrap: path along −x, yaw `−π + 0.05` | `θ = wrap((−π + 0.05) − π) = +0.05`, so `δ = −K[1]·0.05` (not ≈ 2π·K[1]) | wrap |
| 7 | Circle R = 2.0 m CCW (0.02 m waypoints), rear axle on it, tangent | `δ = atan(0.36/2.0) = 0.178093`, `abs=1e-3  # rad, discretization` ; CW → negated | feedforward missing / sign |
| 8 | `gain_at` at exact grid speeds equals the direct DARE result; at a midpoint equals the mean of its two neighbours; at 0 → `gain_at(0.1)`; at 100 → `gain_at(v_grid_max)` | closed form (linear interpolation) | interpolation / clamping bugs |
| 9 | `dt` really is the control period: a law built with `control_dt = 0.025` and one with `0.1` give different `K` at v = 1.5, and each matches test 1's recursion for its own `dt` | independent computation | hard-coded dt |
| 10 | Mirror invariant (as spec 02 test 10) | `δ(mirror) = −δ` | asymmetric sign bugs |
| 11 | Non-finite pose / speed | `TrackingLawError(match="non-finite")` | |
| 12 | `q_lateral ≤ 0`, `q_heading ≤ 0`, `r_steer ≤ 0`, `v_grid_max ≤ 0.1` | `ValueError` naming the parameter | |
| 13 | Clip: `e = 5 m` | unclipped reported, `δ = −0.26` exactly | |

Mutations to show caught (Hard rule 1): swap `K[0]`/`K[1]`; `+K` instead of `−K`; `dt = 0.1`; `A[0][1] = dt` (missing `v`); drop feedforward; `gain_at` returns the first grid entry always; `return inp.previous_steering`.

## Acceptance

Same as spec 02's acceptance list, with `steering_law = lqr`. In particular: if the judging run is REJECT after honest tuning on the tuning seeds, commit the results and the tuned values, record the verdict in `docs/sim-validation.md`, and stop. LQR's gain code is still needed by spec 04 either way.
