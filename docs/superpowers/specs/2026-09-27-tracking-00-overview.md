# Path-tracking laws — 00: shared architecture (read first)

**Status:** approved design, 2026-09-27. Not implemented.
**Specs in this set:** 00 overview (this file) · [01 comparison harness](2026-09-27-tracking-01-harness.md) · [02 Stanley](2026-09-27-tracking-02-stanley.md) · [03 LQR](2026-09-27-tracking-03-lqr.md) · [04 linear MPC](2026-09-27-tracking-04-mpc.md)
**Origin:** the GLM 5.3 research note "Racerbot path-tracking research — consolidated notes" (25 Sep 2026, against commit `022e6fa`). Its methods are used; its integration advice, its 10 Hz homemade simulator and its gains are not. Its runnable ZIP is not in this repo and nothing here depends on it.

## Who this is for

An AI (or human) implementer working in this repo. Read `CLAUDE.md`, `TEST_QUALITY_STANDARDS.md`, `docs/architecture.md` and `docs/racing-autonomy.md` before starting. This file defines the contract every other spec builds on; the later specs do not repeat it.

## Goal

Let `pure_pursuit_node` steer with a choice of tracking law — `pure_pursuit` (today's, the default), `stanley`, `lqr`, `mpc` — **without changing anything else the node does.** Every existing safety mechanism (LB deadman, pose-stale, pose-frozen and cross-track watchdogs, off-line recovery, opponent overtaking, the reactive LiDAR net, steering slew and curvature speed cap) must apply to every law automatically, because the law is swapped in *between* them rather than beside them.

Success = a law may be selected on the car only after it has beaten pure pursuit in the comparison harness (spec 01's promotion rule) and passed the node tests below. Selecting a law is never evidence it is better.

## Non-goals (do not do these)

- Do not touch speed. Speed stays exactly as today: profile speed → reactive caps → `_shape_normal_command` curve cap and ramps. Laws output **steering only**.
- Do not write a second driving node, and do not copy the node's safety logic anywhere.
- No NMPC, MPCC, MPPI, dynamic tyre models, or learned components.
- Do not change the behaviour of off-line recovery or overtaking — both keep using pure-pursuit geometry whatever law is selected.
- Do not make `steering_law` live-tunable from the dashboard.
- Never set `enable_deadman: false` in shipped config.

## Hard stop points — stop and report to the user, do not work around

1. **Pose frame (blocks every on-car step, not the sim work).** `pure_pursuit_node.pose_callback` uses the pose as the rear-axle `base_link` pose with no offset. With `pose_topic: /slam_pose` (`auto_map_race`) that is what it is. With `/pf/viz/inferred_pose` it is suspect: `particle_filter.py:247` publishes the transform as `map -> /laser`, which suggests the particle filter's pose is the **LiDAR's**, 0.26 m ahead of the rear axle. Unverified as of 2026-09-27. Stanley measures error at the front axle and is directly sensitive to a 0.26 m frame error; so, less visibly, is today's pure pursuit. Before any on-car test of any new law in particle-filter mode: determine the frame (read `particle_filter.py`'s motion/sensor model, or drive the car with both `/slam_pose` and `/pf/viz/inferred_pose` live and compare), and **report the finding to the user**. Do not "fix" it silently — it changes today's pure-pursuit behaviour too.
2. Any test that fails and seems to need loosening (Hard rule 2 in `CLAUDE.md`).
3. Spec 04's G0 benchmark failing its budget, or needing a system-wide `pip install`.
4. A law that fails spec 01's promotion rule. The answer is "it does not ship", not "relax the rule".

## Conventions (every spec uses these; get them wrong and the car steers the wrong way)

| Quantity | Definition |
|---|---|
| Pose `(x, y, yaw)` | map frame, of the **rear axle** (`base_link`, see `docs/hardware-reference.md`). `yaw` in rad, CCW from +x. |
| Front axle | `(x + L·cos yaw, y + L·sin yaw)`, `L = wheelbase = 0.36 m` (node parameter, never a literal). |
| Steering `δ` | rad, **positive = left** (REP-103; same sign as today's `steering_from_curvature`). Clipped to `±max_steering_angle` (0.26). |
| Path heading `ψ_p` | direction of the path segment the point projects onto, `atan2(Δy, Δx)`. |
| Signed lateral error `e` | `e = −sin ψ_p·(px − qx) + cos ψ_p·(py − qy)`, `q` the projection of point `p` on the path. **Positive = the point is left of the path.** |
| Heading error `θ` | `θ = wrap(yaw − ψ_p)` into `(−π, π]`. Positive = the car points left of the path. (Stanley's formula uses `−θ`; spec 02 says so explicitly.) |
| Signed path curvature `κ` | 1/m, **positive = path turns left (CCW)**. |
| Curvature feedforward | `δ_ff = atan(L·κ)` — the steady-state steering for a circle of radius `1/κ` in the kinematic bicycle. |

Sanity check every law against these: a car left of the path (`e > 0`) must command `δ < 0`; a car pointing left (`θ > 0`) must command `δ < 0`; a left-hand bend (`κ > 0`) must command `δ > 0`.

## Unit 1 — `racing_math` additions (ROS-free)

Add to `src/pure_pursuit/pure_pursuit/racing_math.py`. Do **not** modify `estimate_path_curvature` — it returns *unsigned* curvature and the velocity profile depends on that.

```python
def estimate_signed_path_curvature(xy: np.ndarray, closed: bool = True) -> np.ndarray:
    """Menger curvature with sign: + for a left (CCW) turn, - for right, 0 on straights.
    Same magnitude as estimate_path_curvature(xy, closed) everywhere."""

def project_onto_path(xy: np.ndarray, seg_len: np.ndarray, point, nearest_idx: int,
                      closed: bool = True) -> PathProjection:
    """Orthogonal projection of `point` onto the two segments adjacent to
    nearest_idx ((i-1 -> i) and (i -> i+1), wrapping when closed; only the
    existing one at an open path's ends). Chooses the closer projection."""
```

`PathProjection` is a frozen dataclass: `segment_index: int` (start waypoint of the chosen segment), `t: float` in `[0, 1]` (clamped), `qx, qy: float`, `heading: float` (`ψ_p`), `lateral_error: float` (signed `e`), `arc_length: float` (distance from waypoint 0 along the path to `q`, using `compute_cumulative_arc_length`). Zero-length segments (duplicate waypoints) are skipped; if every candidate segment has zero length, raise `ValueError` with `match="zero-length"`.

Curvature at a projection is interpolated linearly between the two waypoints' `κ` by `t` — write it as a small helper `interpolate_along(values, projection, closed)` so every law uses the same one.

**Tests** (`src/pure_pursuit/test/test_racing_math.py`, runs unsourced):
- Signed curvature: waypoints on a circle of radius R = 2.0 m traversed CCW → `+1/R` at every point, `abs=1e-3  # 1/m`; the same points reversed → `−1/R`; a straight → 0; `abs(signed) == estimate_path_curvature` on a random closed polyline (invariant). Oracle: closed form 1/R.
- Projection: straight path along +x, point `(1.3, 0.2)` → `q = (1.3, 0)`, `e = +0.2`, heading 0; point `(1.3, −0.2)` → `e = −0.2`. A path along +y (heading π/2): point to its left is at **−x**, so `(−0.2, 1.0)` → `e = +0.2`. Heading-wrap seam: segment heading near ±π. Open path, point beyond the last waypoint → `t = 1` on the last segment, finite. Duplicate waypoints. `NaN` point → `ValueError`.
- Stub-swap each (Hard rule 1): return `+abs` curvature, drop the sign in `e`, swap `x`/`y`.

## Unit 2 — `tracking_laws.py` (new, ROS-free)

New module `src/pure_pursuit/pure_pursuit/tracking_laws.py`. No `rclpy` import anywhere in it (keep it runnable unsourced, like `racing_math`). Spec 04's `osqp` import is lazy, inside the MPC class.

```python
class TrackingLawError(ValueError):
    """A law could not produce a trustworthy steering angle this tick."""

@dataclass(frozen=True)
class VehicleLimits:
    wheelbase: float           # m
    max_steering_angle: float  # rad, symmetric (as today's node)
    max_steering_rate: float   # rad/s (node's max_steering_rate)
    control_dt: float          # s, 1 / control_rate_hz

@dataclass(frozen=True)
class PathContext:
    xy: np.ndarray             # (N, 2)
    seg_len: np.ndarray        # (N,)
    cumulative: np.ndarray     # (N,), compute_cumulative_arc_length(seg_len)
    speed_profile: np.ndarray  # (N,) m/s
    curvature: np.ndarray      # (N,) SIGNED, 1/m
    closed: bool

@dataclass(frozen=True)
class TrackingInput:
    x: float; y: float; yaw: float   # rear axle, map frame
    speed: float                     # m/s, >= 0 (see "speed input" below)
    nearest_idx: int                 # from the node's existing windowed search
    previous_steering: float         # node's steering_basis (last shaped command)

@dataclass(frozen=True)
class SteeringResult:
    steering: float     # rad, clipped to +/- max_steering_angle, finite
    unclipped: float    # rad, before clipping (for the decision log)
    detail: str         # one short clause for the decision log, e.g. "stanley e_f=+0.12m th=-0.03rad"
    predicted: tuple = ()  # optional: planned steering sequence (MPC only), for /drive_intent

class TrackingLaw(Protocol):
    name: str
    def steer(self, inp: TrackingInput, path: PathContext) -> SteeringResult: ...
    def reset(self) -> None: ...     # called on every profile activation

LAW_NAMES = ('pure_pursuit', 'stanley', 'lqr', 'mpc')
def make_law(name: str, limits: VehicleLimits, params: Mapping[str, float]) -> TrackingLaw:
    """Unknown name -> ValueError(match='unknown steering_law'). Invalid params -> ValueError naming the param."""
```

Rules every law obeys:
- **Never return a non-finite value.** If anything computed is non-finite, or the input contains `NaN`/`inf`, raise `TrackingLawError`. A law never returns a "safe default"; deciding what is safe is the node's job.
- Clip to `±max_steering_angle` inside the law; report `unclipped` too.
- Parameters are validated in the constructor (e.g. gains > 0) and raise `ValueError` naming the parameter.
- Default parameter values live in one frozen dataclass per law in this module (`StanleyParams`, `LqrParams`, `MpcParams`). `config/pure_pursuit.yaml` mirrors them, and a test asserts the YAML defaults equal the dataclass defaults (parse with `yaml`; runs unsourced). This is what stops the harness and the car running different gains.

**`PurePursuitLaw`** — move today's steering computation here unchanged: `adaptive_lookahead` on `inp.speed` → `find_lookahead_index` → `world_to_body` → `steering_arc_curvature` → `steering_from_curvature` → clip. It uses `min_lookahead`, `max_lookahead`, `lookahead_speed_gain` (the node's existing parameters, not new ones). It must reproduce the node's current output exactly:
- Oracle test (closed form): straight path along +x, car at origin, yaw 0, lookahead target at `(Ld, y_t)` → `κ = 2·y_t/Ld²`, `δ = atan(L·κ)`.
- Refactor regression, labelled `# change-detector (not an oracle)`: **before** touching the node, record the node's steering for ~20 fixed poses on a fixed profiled CSV (a test that drives the current node code); after the refactor the same test must still pass unmodified.

**Speed input.** `inp.speed` is what today's lookahead already uses: `abs(current_speed)` when odometry is fresh, otherwise `speed_profile[nearest_idx]`. The node computes it once and passes the same value to every law. The source is already in the decision detail (`lookahead_speed_source`); keep it there.

**Tests** (`src/pure_pursuit/test/test_tracking_laws.py`, runs unsourced): the per-law specs list their oracles; this file also holds `make_law` (unknown name, bad params) and the YAML-mirror test.

## Unit 3 — node integration (`pure_pursuit_node.py`)

1. **Parameters.** Declare `steering_law` (string, default `'pure_pursuit'`), plus each law's parameters with a dotted prefix (`stanley.gain`, `lqr.q_lateral`, `mpc.horizon_steps`, …; full lists in specs 02–04), and `tracking_curvature_smoothing` (int half-window, default 2). Build the law once in `__init__` with `make_law(...)`; an unknown name or invalid value raises `RuntimeError` and the node **refuses to start**. `_parameter_callback` must reject any runtime change to `steering_law` or any `stanley.* / lqr.* / mpc.*` parameter with a clear reason (they are launch-time only). Do not add them to `live_tuning.PURE_PURSUIT_TUNABLES`.
2. **Path context.** In `_activate_profile`, compute `curvature = estimate_signed_path_curvature(smooth_path(xy, tracking_curvature_smoothing, closed), closed)` and build the `PathContext` alongside the existing `self.xy`/`self.seg_len` assignment, **inside the same all-validated-first block** so a bad profile cannot half-activate. Then call `self.tracking_law.reset()`. Smoothing is used for curvature only; the law still tracks the unsmoothed `xy`, as pure pursuit does today.
3. **The seam.** In `_control_step`, the block that computes `steering_angle` in the normal (not `recovering`) branch (`pure_pursuit_node.py:1191–1228` at commit `0b4e104`: from the "Steering: adaptive lookahead" comment through the `np.clip`) becomes a call to `self.tracking_law.steer(...)`. Keep computing the pure-pursuit lookahead `target_idx`/`target_x/y` in every mode: overtaking, recovery and the `/drive_intent` target marker still use it. Everything before the seam and everything after it (overtake reconsideration, `_reactive_override`, `_shape_normal_command`, `_publish_drive`, intent) is untouched.
4. **Failure.** Wrap only the `steer` call: on *any* exception, call `self._stop('steering_law_failed', f"{law}: {type(exc).__name__}: {exc}")` and return. Never reuse the previous tick's command. (Catching broadly here is deliberate and the only such catch: a crashing law must become a stop, not a dead node. Log it through the existing decision logging so it is rate-limited.) MPC adds a fallback layer on top of this — see spec 04; its specific miss exception is handled *before* this generic catch.
5. **Decision log.** In the normal branch, `decision_state` becomes the law's name (`'pure_pursuit'` stays `'pure_pursuit'`, so existing log parsing and `race_diagnostics` keep working); append `SteeringResult.detail` to `decision_detail`. Check `src/race_diagnostics/race_diagnostics/run_events.py` for any hard-coded list of states and add the new ones (`stanley`, `lqr`, `mpc`, `steering_law_failed`, and spec 04's two).
6. **`/drive_intent`.** In `_intent_path`, `steering_of` currently re-runs the pure-pursuit law at each integration step. Make it re-run the **active** law for `stanley` and `lqr` (build a `TrackingInput` from the simulated `(x, y, yaw)`, speed from the profile, `nearest_idx` from the existing cursor). For `mpc`, integrate the solver's `predicted` steering sequence instead of re-solving: at integration time `t`, use `predicted[min(floor(t / mpc.dt), N − 1)]` (so the last planned value is held past the horizon). The intent contract in `docs/drive-intent.md` still applies: intent is published after the drive command and a failure there disables intent, not the node.

**Node tests** (`src/pure_pursuit/test/test_tracking_law_integration.py`; needs ROS sourced — collected only by `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 -m pytest src/pure_pursuit/test/`; check the collected count). Build nodes the way `test_opponent_integration.py` does, **always** with `drive_topic:=/test_only/drive`. Parametrize over `LAW_NAMES` (for `mpc`, skip with a reason naming spec 04 G0 until osqp is installed — `pytest.importorskip("osqp", reason=...)`, which is removed as soon as G0 lands):

| Test | Assertion (on the published command, never on an internal flag) |
|---|---|
| LB held, on the line, slightly left of it | non-zero speed, and steering `< 0` (towards the line) |
| LB released | speed 0 |
| `/joy` never received | speed 0 |
| `/joy` stale past timeout | speed 0 |
| Law raises (see note) | speed 0, `last_decision_state == 'steering_law_failed'` |
| Real failure path: pose with `yaw = NaN` | speed 0 (from whichever check catches it first — assert the command, then record which state caught it in the test's docstring) |
| Runtime `set_parameters(steering_law=...)` | rejected, law unchanged |
| Unknown `steering_law` at construction | node constructor raises `RuntimeError` |
| Profile reload | `reset()` observed by behaviour: for `mpc`, the first solve after reload is cold (no warm start) — assert via the law's own `solves_since_reset` counter |

Deadman tests enable the deadman the way `test_opponent_integration.py:842` does (`node.enable_deadman = True`, publish `Joy`). The "law raises" test replaces `node.tracking_law` with a tiny test-only class whose `steer` raises `TrackingLawError`. This is not the A1 anti-pattern: the double stands in for a dependency to exercise the node's failure path, and the assertion is on the node's published command.

## Rollout order (for any law, after it passes spec 01)

Exactly `CLAUDE.md`'s order, with the law selected via `steering_law:=<law>` on `pure_pursuit_launch.py` (the launch file must pass the argument through — add that):
1. Static topic check (no driver stack): node starts, `/drive` silent without LB.
2. Wheels off the ground, full stack, LB held: steering direction sanity — push the car left of the line by hand, the wheels must turn right.
3. Floor, low speed (`max_speed` ≤ 1.0 via the launch argument), open space, `race_diagnostics` recording. Compare cross-track against a pure-pursuit run on the same line in the same session.

Hard stop point 1 (pose frame) must be resolved before step 2 in particle-filter mode.

## Docs to update when implementing

`docs/racing-autonomy.md` (a "Choosing a steering law" section, novice-readable — use the `novice-docs` skill), `docs/sim-validation.md` (spec 01's comparison), `README.md` index if a new doc is added, `CHANGELOG.md`.
