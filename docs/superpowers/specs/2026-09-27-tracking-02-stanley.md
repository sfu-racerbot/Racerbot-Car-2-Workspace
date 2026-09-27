# Path-tracking laws — 02: Stanley

**Status:** approved design, 2026-09-27. Not implemented. **Read [00 overview](2026-09-27-tracking-00-overview.md) first** — conventions, interface, node integration and node tests live there and are not repeated. Judged by [01 harness](2026-09-27-tracking-01-harness.md).

## What it is (for the docs, in plain words)

Pure pursuit steers toward a point some distance ahead on the line. Stanley instead looks at the **front axle**: how far it is to the side of the line, and which way the car is pointing compared with the line. It turns the wheels to (a) point along the line and (b) pull the front axle back onto it — harder when slow, gentler when fast — plus (c) the steering a bend needs anyway. Reference: Hoffmann et al., ACC 2007, https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf.

Why try it here: it corrects sideways error directly and has no lookahead distance to tune; pure pursuit cuts corners by roughly its lookahead. Why it might lose: it reacts to the line where the front axle *is*, with no preview, and its lateral term amplifies pose jitter at low speed.

## The law

Inputs from `TrackingInput`/`PathContext` (00). `L = limits.wheelbase`.

```
xf = x + L·cos(yaw);  yf = y + L·sin(yaw)                     # front axle
proj = project_onto_path(xy, seg_len, (xf, yf), nearest_f, closed)
e_f  = proj.lateral_error                                      # + = front axle left of line
θ_e  = wrap(proj.heading − yaw)                                # NOTE: path minus car (= −θ of 00)
κ_f  = interpolate_along(curvature, proj, closed)              # signed
v    = max(inp.speed, 0.0)

δ_unclipped = θ_e − atan( k · e_f / (v + v_soft) ) + atan( L · κ_f )
δ = clip(δ_unclipped, −max_steering_angle, +max_steering_angle)
```

- `nearest_f`: the front axle is ahead of the rear axle, so the node's `nearest_idx` (a rear-axle search) is not the right seed. Walk forward from `inp.nearest_idx` with `find_lookahead_index(seg_len, nearest_idx, L, closed)` and use that index as the seed. Test that this lands within one segment of a global nearest search for the front axle on a gently curving line.
- `k` (`stanley.gain`, units 1/s) and `v_soft` (`stanley.softening`, m/s) are the only parameters. Constructor rejects `gain <= 0` and `softening <= 0` (`ValueError` naming the parameter; `softening = 0` would divide by zero at standstill).
- `detail`: `f"stanley e_f={e_f:+.3f}m th_e={θ_e:+.3f}rad ff={atan(L·κ_f):+.3f}rad"`.
- No internal state; `reset()` is a no-op.
- Non-finite anywhere → `TrackingLawError`.

**Starting parameters** (from the research; demonstration values, **not** measured on this car): `gain = 1.6`, `softening = 0.4`. Tune in the harness on the tuning seeds (01) and commit the tuned values with their evidence.

**Node parameters** (00 Unit 3, launch-time only): `stanley.gain`, `stanley.softening`.

## Known risks to watch in sim and on the car

- **Pose frame** — 00 hard stop point 1 (resolved 2026-09-27: poses reach the law already converted to the rear axle; do not convert again). A 0.26 m error in where "rear axle" is becomes a heading-dependent error in `e_f`, so a wrong `pose_frame` in a new launch file shows up here first.
- **Pose jitter at low speed:** 2 cm of jitter gives `k·0.02/(v + v_soft)` rad of steering noise — about 0.035 rad at 0.5 m/s with the starting gains. The node's 1.0 rad/s slew limit caps it at 0.025 rad per tick. Watch the harness's `p95` versus `mean` cross-track for oscillation; the `car` fidelity profile includes pose noise.
- **Curvature noise:** the feedforward uses per-waypoint Menger curvature, which is noisy on raw recorded lines. That is why 00 smooths curvature (`tracking_curvature_smoothing`). If sim shows feedforward chatter, raise the smoothing — do not delete the term.
- **Open path end:** the projection clamps to the last segment; the law keeps steering along it. Stopping at the end of an open path is the node's job (profile speed), not the law's.

## Tests (`src/pure_pursuit/test/test_tracking_laws.py`, runs unsourced)

Every expected value is a closed form, recomputed in the test from the formula above (A3). Use `gain = 1.6`, `softening = 0.4`, `L = 0.36`, `max_steering_angle = 0.26` unless a test says otherwise. Straight path = waypoints along +x every 0.05 m, `curvature` all zero.

| # | Setup | Expected (tolerance `abs=1e-9  # rad` unless noted) | Catches |
|---|---|---|---|
| 1 | Straight, rear axle at `(0, 0)`, yaw 0, v = 1.0 | `δ = 0` exactly | stray offsets |
| 2 | Straight, rear axle at `(0, 0.2)`, yaw 0, v = 1.0 (front axle also at y = 0.2) | `−atan(1.6·0.2/1.4) = −0.224711…` | sign of lateral term, gain ±10 %, softening dropped |
| 3 | As 2 but y = −0.2 | `+0.224711…` (mirror) | sign |
| 4 | Straight, rear axle `(0, 0)`, yaw = +0.1, v = 1.0 | `−0.1 − atan(1.6·0.36·sin 0.1/1.4) = −0.141051…` — **not** −0.1 | rear axle used instead of front |
| 5 | Straight, heading-wrap: path along −x (`ψ_p = π`), car on it with yaw `−π + 0.05` | `θ_e = −0.05` → `δ = −0.05` | missing wrap (would give ≈ 2π) |
| 6 | Circle radius 2.0 m, CCW, waypoints every 0.02 m. Place the **front** axle on the circle with `yaw` = the circle's tangent there; rear axle = front − L·(cos yaw, sin yaw). v = 1.0 | `e_f ≈ 0`, `θ_e ≈ 0`, so `δ = atan(0.36/2.0) = 0.178093`, `abs=1e-3  # rad, 0.02 m waypoint discretization` | missing / wrong-sign feedforward |
| 7 | Same, circle traversed CW | `−0.178093`, same tolerance | curvature sign |
| 8 | Test 2 at v = 0 | `−atan(1.6·0.2/0.4) = −0.674741…` unclipped; `δ = −0.26` exactly | division at standstill, clip missing |
| 9 | Test 2 at v = 3.0 | `−atan(0.32/3.4) = −0.093841…` | speed term inverted |
| 10 | Mirror invariant: 50 random poses within 0.5 m / 0.4 rad of a random smooth closed path, and the same poses/path reflected in the x-axis | `δ(mirror) = −δ(original)`, `abs=1e-9` | any asymmetric sign bug |
| 11 | `x = NaN`; `yaw = inf`; `speed = NaN` | `TrackingLawError` (`match="non-finite"`) | returning garbage |
| 12 | `gain = 0`, `gain = −1`, `softening = 0` | `ValueError`, `match="stanley.gain"` / `"stanley.softening"` | bad config accepted |
| 13 | Open 3-waypoint path, front axle 1 m beyond its end, on the extension line | finite, `δ = 0` (aligned with last segment) | end-of-path crash |

Test 6 places the front axle, not the rear, on the circle so that the lateral and heading terms are zero and the feedforward term is tested alone; axle placement is test 4's job.

Mutation checks required before calling this done (Hard rule 1): negate the lateral term; use `(x, y)` instead of `(xf, yf)`; drop `atan(L·κ)`; change `gain` by 10 % inside the law only; remove `wrap`; return `inp.previous_steering` unchanged. Each must fail at least one test above. Record which test caught each in the PR description.

## Acceptance

1. 00's `racing_math`/`tracking_laws` units and node integration are in place (or land in the same PR).
2. The table above passes and every listed mutation is caught.
3. 00's node tests pass with `steering_law = stanley`.
4. Spec 01's judging run gives **PROMOTE**; results committed. If REJECT: commit the results anyway, write down why, and stop — no on-car steps.
5. Then 00's rollout order, confirming `pose_frame` matches `pose_topic` for the launch in use (00 hard stop point 1).
