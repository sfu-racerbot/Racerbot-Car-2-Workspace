"""Steering laws for the race controller, kept ROS-free.

This module is the seam where `pure_pursuit_node` picks *how* it steers
without changing anything else it does (the LB deadman, the pose and
cross-track watchdogs, off-line recovery, the reactive LiDAR net, the
steering slew and curvature speed caps all stay in the node, upstream and
downstream of this one call). See
docs/superpowers/specs/2026-09-27-tracking-00-overview.md for the contract
every law obeys and the conventions they share -- get the sign conventions
wrong and the car steers the wrong way.

Deliberately nothing in here imports rclpy: every law is plain numbers in,
a plain `SteeringResult` out, so the whole module is unit-testable with no
robot, simulator, or ROS install (see test/test_tracking_laws.py). The
only optional dependency is osqp, imported lazily inside `MpcLaw` so the
other laws -- and this module's tests -- work without it.
"""

import math
import time
from dataclasses import dataclass
from typing import Mapping, Protocol

import numpy as np

from pure_pursuit import racing_math


class TrackingLawError(ValueError):
    """A law could not produce a trustworthy steering angle this tick.

    Raised instead of returning a "safe default": deciding what is safe is
    the node's job (it turns this into `_stop`), never the law's.
    """


class MpcMiss(TrackingLawError):
    """MPC-specific transient failure (bad status, deadline, bad solution).

    Distinct from a plain TrackingLawError because the node answers a miss
    with a one-tick pure-pursuit fallback (up to a consecutive-miss latch)
    rather than an immediate stop. See spec 04's miss policy.
    """


@dataclass(frozen=True)
class VehicleLimits:
    """The fixed vehicle numbers a law is allowed to assume."""
    wheelbase: float            # m
    max_steering_angle: float   # rad, symmetric
    max_steering_rate: float    # rad/s
    control_dt: float           # s, 1 / control_rate_hz


@dataclass(frozen=True)
class PathContext:
    """Everything about the racing line a law needs, precomputed once per
    profile activation (never re-derived per tick)."""
    xy: np.ndarray              # (N, 2) the line being tracked (unsmoothed)
    seg_len: np.ndarray         # (N,)
    cumulative: np.ndarray      # (N,) compute_cumulative_arc_length(seg_len)
    speed_profile: np.ndarray   # (N,) m/s
    curvature: np.ndarray       # (N,) SIGNED, 1/m
    closed: bool


@dataclass(frozen=True)
class TrackingInput:
    """One tick's snapshot of the car, handed to a law."""
    x: float
    y: float
    yaw: float                  # rear axle, map frame
    speed: float                # m/s, >= 0 (see "speed input" in spec 00)
    nearest_idx: int            # from the node's existing windowed search
    previous_steering: float    # node's steering_basis (last shaped command)


@dataclass(frozen=True)
class SteeringResult:
    """What a law decided, and enough detail to log *why*."""
    steering: float             # rad, clipped to +/- max_steering_angle, finite
    unclipped: float            # rad, before clipping (for the decision log)
    detail: str                 # one short clause for the decision log
    predicted: tuple = ()       # optional planned steering sequence (MPC only)


class TrackingLaw(Protocol):
    """The seam. `name` is stable (it becomes the decision log's state);
    `steer` produces one command; `reset` is called on every profile
    activation so any warm-start or state starts clean."""
    name: str

    def steer(self, inp: TrackingInput, path: PathContext) -> SteeringResult:
        ...

    def reset(self) -> None:
        ...


LAW_NAMES = ('pure_pursuit', 'stanley', 'lqr', 'mpc')


def _require_finite(*values):
    for value in values:
        if not math.isfinite(value):
            raise TrackingLawError(
                f'non-finite input to a steering law: {values}')


def _check_delay(name: str, value: float) -> None:
    if not (math.isfinite(value) and 0.0 <= value <= 0.5):
        raise ValueError(f'{name} must be in [0, 0.5] s, got {value}')


def smooth_steering(target: float, previous: float, time_constant: float,
                    dt: float) -> float:
    """First-order low-pass from the last command toward the law's output.

    Stanley and LQR answer the measured error *now*, so localisation noise
    reaches the rack at full gain: with the simulator's 2 cm of pose noise,
    Stanley at gain 3 and 1 m/s jitters ~0.04 rad a tick, three times pure
    pursuit (whose lookahead averages it away). The jitter is not free --
    the speed cap reads it as curvature and slows the car
    (docs/steering-laws.md#tuning-stanley-and-lqr). A time constant of a
    tick or two removes it; the lag it adds is what delay_compensation is
    for. Stateless: `previous` is the last command, TrackingInput's
    previous_steering.
    """
    if time_constant <= 0.0:
        return target
    alpha = dt / (time_constant + dt)
    return previous + alpha * (target - previous)


def _wrap(angle: float) -> float:
    """Wrap to (-pi, pi] -- see the conventions table in spec 00."""
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def sample_at_arc(values: np.ndarray, cumulative: np.ndarray, s: float,
                  closed: bool = True) -> float:
    """Linear interpolation of a per-waypoint field at arc length `s`.

    `cumulative[i]` is the arc length of waypoint i (from waypoint 0). On a
    closed loop `s` wraps modulo the total length; on an open path it clamps
    to the ends. Used by MPC to sample curvature and speed along the horizon.
    """
    n = len(values)
    if n == 0:
        raise TrackingLawError('sample_at_arc: empty values')
    total = float(cumulative[-1])
    if closed:
        s = s % total
        idx = int(np.searchsorted(cumulative, s, side='right') - 1)
        idx = max(0, min(idx, n - 1))
        nxt = (idx + 1) % n
        seg_start = cumulative[idx]
        seg_end = (cumulative[nxt] if nxt > idx else total)
    else:
        s = min(max(s, 0.0), total)
        idx = int(np.searchsorted(cumulative, s, side='right') - 1)
        idx = max(0, min(idx, n - 1))
        nxt = min(idx + 1, n - 1)
        seg_start = cumulative[idx]
        seg_end = cumulative[nxt]
    span = seg_end - seg_start
    t = 0.0 if span < 1e-9 else (s - seg_start) / span
    return float(values[idx]) + t * (float(values[nxt]) - float(values[idx]))


def predict_input(inp: 'TrackingInput', path: 'PathContext', delay: float,
                  wheelbase: float) -> 'TrackingInput':
    """The car `delay` seconds from now, if it holds its speed and steering.

    A command is not acted on the instant it is computed: the pose it was
    computed from is already a scan or two old, the command waits a tick in
    transport, and the servo takes ~40 ms to reach it (the simulator's car
    profile models all three, ~0.1 s together -- tools/f1tenth_sim/
    sim_fidelity/calibration.py). Pure pursuit hides that by aiming a metre
    or more ahead; a law that corrects the error *now* -- Stanley, LQR --
    is always correcting an error the car has already moved past, and at
    race speed that is an oscillation. Computing the law at the predicted
    pose instead is standard delay compensation.

    The prediction is the kinematic bicycle on a constant-curvature arc
    (curvature tan(previous_steering)/wheelbase, the rack's current angle),
    and the nearest waypoint is walked forward by the same distance then
    refined locally, so the result can be handed to any law unchanged.
    """
    if delay <= 0.0:
        return inp
    distance = max(inp.speed, 0.0) * delay
    if distance <= 0.0:
        return inp
    curvature = math.tan(inp.previous_steering) / wheelbase
    yaw0 = inp.yaw
    if abs(curvature) < 1e-9:
        x = inp.x + distance * math.cos(yaw0)
        y = inp.y + distance * math.sin(yaw0)
        yaw = yaw0
    else:
        yaw = yaw0 + curvature * distance
        x = inp.x + (math.sin(yaw) - math.sin(yaw0)) / curvature
        y = inp.y - (math.cos(yaw) - math.cos(yaw0)) / curvature
    walked = racing_math.find_lookahead_index(
        path.seg_len, inp.nearest_idx, distance, closed=path.closed)
    nearest, _ = racing_math.find_nearest_index(
        path.xy, (x, y), closed=path.closed, prev_index=walked, search_window=6)
    return TrackingInput(x=x, y=y, yaw=_wrap(yaw), speed=inp.speed,
                         nearest_idx=int(nearest),
                         previous_steering=inp.previous_steering)


# ---------------------------------------------------------------------------
# Pure pursuit -- today's law, moved here unchanged (spec 00 Unit 2).
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class PurePursuitParams:
    min_lookahead: float = 0.6
    max_lookahead: float = 1.5
    lookahead_speed_gain: float = 0.15


class PurePursuitLaw:
    """The steering pure_pursuit_node has always done, as a TrackingLaw.

    Reproduces the node's current output exactly: adaptive_lookahead on the
    input speed -> find_lookahead_index -> world_to_body ->
    steering_arc_curvature -> steering_from_curvature -> clip. This is the
    baseline every other law has to beat in the harness; do not "improve"
    it here.
    """

    name = 'pure_pursuit'

    def __init__(self, limits: VehicleLimits, params: PurePursuitParams):
        self.limits = limits
        self.params = params

    def reset(self) -> None:
        # Pure pursuit is stateless; nothing to reset.
        return None

    def steer(self, inp: TrackingInput, path: PathContext) -> SteeringResult:
        _require_finite(inp.x, inp.y, inp.yaw, inp.speed)
        lookahead = racing_math.adaptive_lookahead(
            inp.speed,
            self.params.lookahead_speed_gain,
            self.params.min_lookahead,
            self.params.max_lookahead,
        )
        target_idx = racing_math.find_lookahead_index(
            path.seg_len, inp.nearest_idx, lookahead, closed=path.closed)
        target_x, target_y = path.xy[target_idx]
        x_body, y_body = racing_math.world_to_body(
            float(target_x - inp.x), float(target_y - inp.y), inp.yaw)
        kappa = racing_math.steering_arc_curvature(x_body, y_body)
        unclipped = racing_math.steering_from_curvature(kappa, self.limits.wheelbase)
        steering = float(np.clip(
            unclipped, -self.limits.max_steering_angle, self.limits.max_steering_angle))
        detail = (
            f"pure_pursuit lookahead={lookahead:.2f}m target={target_idx} "
            f"kappa={kappa:+.3f}/m")
        return SteeringResult(steering=steering, unclipped=float(unclipped),
                              detail=detail)


# ---------------------------------------------------------------------------
# Stanley (spec 02): correct at the front axle.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class StanleyParams:
    # Tuned on the harness's tuning seeds, judged on the judging seeds
    # (docs/steering-laws.md#tuning-stanley-and-lqr); keep in step with
    # pure_pursuit.yaml.
    gain: float = 5.0           # stanley.gain, 1/s
    softening: float = 1.0      # stanley.softening, m/s
    delay_compensation: float = 0.10  # stanley.delay_compensation, s (predict_input)
    steering_time_constant: float = 0.05  # stanley.steering_time_constant, s (smooth_steering)


class StanleyLaw:
    """Stanley: steer by the front axle's heading error plus a correction for
    its lateral error. Reference: Hoffmann et al., ACC 2007.

        delta = theta_e - atan(k * (e_f - e_f*) / (v + v_soft))

    theta_e = wrap(path_heading - yaw) is measured at the *front* axle's
    projection, path minus car (the negative of spec 00's `theta`); e_f is
    the front axle's signed lateral error.

    Two corrections to spec 02's formula, both from the kinematic bicycle
    (docs/steering-laws.md#stanley):

    * **No separate curvature feedforward.** On a bend the path heading at the
      front axle's projection is already ahead of the car's heading by the
      steering the bend needs (atan(L/R) on a circle), so theta_e carries the
      feedforward. Spec 02 also added atan(L * kappa), which counted it twice:
      at steady state that forces a front-axle offset of v*L*kappa/k to the
      inside of the bend -- ~0.6 m at 3 m/s in a 1 m-radius corner, clipped
      only by the steering limit -- and the car ran inside and hit the wall on
      every Silverstone run in the harness.
    * **The line is a rear-axle line.** Every racing line in this workspace is
      a base_link (rear axle) path. A car whose rear axle follows a circle of
      radius R has its front axle sqrt(R^2 + L^2) - R *outside* it, so that is
      the front-axle offset e_f* Stanley steers to, not zero. Without it the
      rear axle cuts L^2/(2R) inside every bend.
    """

    name = 'stanley'

    def __init__(self, limits: VehicleLimits, params: StanleyParams):
        if not (math.isfinite(params.gain) and params.gain > 0.0):
            raise ValueError(f'stanley.gain must be > 0, got {params.gain}')
        if not (math.isfinite(params.softening) and params.softening > 0.0):
            raise ValueError(f'stanley.softening must be > 0, got {params.softening}')
        _check_delay('stanley.delay_compensation', params.delay_compensation)
        _check_delay('stanley.steering_time_constant', params.steering_time_constant)
        self.limits = limits
        self.params = params

    def reset(self) -> None:
        return None

    @staticmethod
    def front_axle_offset(kappa: float, wheelbase: float) -> float:
        """Signed lateral offset of the front axle when the rear axle is on a
        path of curvature `kappa` (+ = left): outside the bend, so negative
        for a left turn."""
        if abs(kappa) < 1e-9:
            return 0.0
        radius = 1.0 / abs(kappa)
        return -math.copysign(math.hypot(radius, wheelbase) - radius, kappa)

    def steer(self, inp: TrackingInput, path: PathContext) -> SteeringResult:
        _require_finite(inp.x, inp.y, inp.yaw, inp.speed, inp.previous_steering)
        L = self.limits.wheelbase
        previous = inp.previous_steering
        inp = predict_input(inp, path, self.params.delay_compensation, L)
        # The error is measured at the front axle, not the rear.
        xf = inp.x + L * math.cos(inp.yaw)
        yf = inp.y + L * math.sin(inp.yaw)
        # Seed the front-axle projection: walk forward from the rear-axle
        # nearest by one wheelbase (cheap, and on a straight lands exactly),
        # then refine with a small windowed nearest search around that seed.
        # The refinement matters on curves, where the walked index can sit a
        # segment or two off and project the front axle onto the wrong side.
        walked = racing_math.find_lookahead_index(
            path.seg_len, inp.nearest_idx, L, closed=path.closed)
        nearest_f, _ = racing_math.find_nearest_index(
            path.xy, (xf, yf), closed=path.closed,
            prev_index=walked, search_window=4)
        proj = racing_math.project_onto_path(
            path.xy, path.seg_len, (xf, yf), nearest_f, path.closed)
        e_f = proj.lateral_error
        theta_e = _wrap(proj.heading - inp.yaw)   # path minus car = -theta
        kappa_f = racing_math.interpolate_along(path.curvature, proj, path.closed)
        e_target = self.front_axle_offset(kappa_f, L)
        v = max(inp.speed, 0.0)

        unclipped = (theta_e
                     - math.atan(self.params.gain * (e_f - e_target)
                                 / (v + self.params.softening)))
        limit = self.limits.max_steering_angle
        steering = float(np.clip(smooth_steering(
            float(np.clip(unclipped, -limit, limit)), previous,
            self.params.steering_time_constant, self.limits.control_dt), -limit, limit))
        detail = (f"stanley e_f={e_f:+.3f}m (target {e_target:+.3f}m) "
                  f"th_e={theta_e:+.3f}rad")
        return SteeringResult(steering=steering, unclipped=float(unclipped),
                              detail=detail)


# ---------------------------------------------------------------------------
# LQR (spec 03): fixed feedback on [e, theta], gain scheduled on speed.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class LqrParams:
    q_lateral: float = 1.0
    q_heading: float = 1.0
    r_steer: float = 1.0
    v_grid_max: float = 6.0
    delay_compensation: float = 0.08  # lqr.delay_compensation, s (predict_input)
    steering_time_constant: float = 0.05  # lqr.steering_time_constant, s (smooth_steering)


class LqrGainTable:
    """Speed-scheduled LQR gain, solved once over a speed grid and
    interpolated. `dt` is the discretization step the gain is *designed*
    for -- the control period for LQR itself, the prediction step for
    MPC's terminal cost; a P computed at the wrong dt is the wrong answer.

    The kinematic-bicycle error model (spec 03) is:

        A(v) = [[1, v*dt], [0, 1]]        B(v) = [[0], [v*dt / L]]

    and the gain minimizes sum(s' Q s + u' R u) subject to that model,
    solved with the discrete algebraic Riccati equation.
    """

    V_FLOOR = 0.1   # m/s; the model is uncontrollable at v = 0
    V_STEP = 0.05   # m/s grid spacing

    def __init__(self, q_lateral: float, q_heading: float, r_steer: float,
                 dt: float, wheelbase: float, v_grid_max: float):
        import scipy.linalg  # local import: only needed at construction

        for name, value in (('lqr.q_lateral', q_lateral),
                            ('lqr.q_heading', q_heading),
                            ('lqr.r_steer', r_steer)):
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be > 0, got {value}')
        if not (math.isfinite(v_grid_max) and v_grid_max > self.V_FLOOR):
            raise ValueError(
                f'lqr.v_grid_max must be > {self.V_FLOOR}, got {v_grid_max}')
        if not (math.isfinite(dt) and dt > 0.0):
            raise ValueError(f'lqr dt must be > 0, got {dt}')
        if not (math.isfinite(wheelbase) and wheelbase > 0.0):
            raise ValueError(f'wheelbase must be > 0, got {wheelbase}')

        self.q_lateral = q_lateral
        self.q_heading = q_heading
        self.r_steer = r_steer
        self.dt = dt
        self.wheelbase = wheelbase
        self.v_grid_max = v_grid_max

        self.Q = np.array([[q_lateral, 0.0], [0.0, q_heading]])
        self.R = np.array([[r_steer]])

        speeds = np.arange(self.V_FLOOR, v_grid_max + 1e-9, self.V_STEP)
        self._grid_speeds = speeds
        gains = np.zeros((len(speeds), 2))
        riccati = np.zeros((len(speeds), 2, 2))
        for i, v in enumerate(speeds):
            A, B = self._ab(v)
            P = scipy.linalg.solve_discrete_are(A, B, self.Q, self.R)
            K = np.linalg.solve(self.R + B.T @ P @ B, B.T @ P @ A)
            gains[i] = K[0]
            riccati[i] = P
        self._gains = gains
        self._riccati = riccati

    def _ab(self, v: float):
        v = max(v, self.V_FLOOR)
        A = np.array([[1.0, v * self.dt], [0.0, 1.0]])
        B = np.array([[0.0], [v * self.dt / self.wheelbase]])
        return A, B

    def _interpolate(self, table, v: float):
        v = float(np.clip(v, self._grid_speeds[0], self._grid_speeds[-1]))
        n = len(self._grid_speeds)
        pos = (v - self._grid_speeds[0]) / self.V_STEP
        lo = int(np.clip(math.floor(pos), 0, n - 1))
        hi = min(lo + 1, n - 1)
        t = 0.0 if hi == lo else (pos - lo) / (hi - lo)
        return table[lo] + t * (table[hi] - table[lo])

    def gain_at(self, v: float) -> np.ndarray:
        """The 1x2 feedback gain K at speed v, linearly interpolated."""
        return np.asarray(self._interpolate(self._gains, v))

    def riccati_at(self, v: float) -> np.ndarray:
        """The 2x2 DARE solution P at speed v, linearly interpolated (the
        terminal-cost matrix MPC uses)."""
        return np.asarray(self._interpolate(self._riccati, v))


class LqrLaw:
    """LQR over the rear-axle [e, theta] error: delta = ff - K . [e, theta].

    The feedforward ff = atan(L * kappa) is the steering the bend needs
    anyway; the feedback term -K.[e, theta] corrects being off it. The gain
    K is speed-scheduled (see LqrGainTable) because the same physical
    steering corrects less per tick the faster the car goes.
    """

    name = 'lqr'

    def __init__(self, limits: VehicleLimits, params: LqrParams):
        _check_delay('lqr.delay_compensation', params.delay_compensation)
        _check_delay('lqr.steering_time_constant', params.steering_time_constant)
        self.limits = limits
        self.params = params
        self._table = LqrGainTable(
            params.q_lateral, params.q_heading, params.r_steer,
            limits.control_dt, limits.wheelbase, params.v_grid_max)

    def reset(self) -> None:
        return None

    def steer(self, inp: TrackingInput, path: PathContext) -> SteeringResult:
        _require_finite(inp.x, inp.y, inp.yaw, inp.speed, inp.previous_steering)
        L = self.limits.wheelbase
        previous = inp.previous_steering
        inp = predict_input(inp, path, self.params.delay_compensation, L)
        proj = racing_math.project_onto_path(
            path.xy, path.seg_len, (inp.x, inp.y), inp.nearest_idx, path.closed)
        e = proj.lateral_error
        theta = _wrap(inp.yaw - proj.heading)
        kappa = racing_math.interpolate_along(path.curvature, proj, path.closed)
        K = self._table.gain_at(max(inp.speed, LqrGainTable.V_FLOOR))
        feedforward = math.atan(L * kappa)
        unclipped = feedforward - float(K[0]) * e - float(K[1]) * theta
        limit = self.limits.max_steering_angle
        steering = float(np.clip(smooth_steering(
            float(np.clip(unclipped, -limit, limit)), previous,
            self.params.steering_time_constant, self.limits.control_dt), -limit, limit))
        detail = (f"lqr e={e:+.3f}m th={theta:+.3f}rad "
                  f"K=[{K[0]:.2f},{K[1]:.2f}] ff={feedforward:+.3f}rad")
        return SteeringResult(steering=steering, unclipped=float(unclipped),
                              detail=detail)


# ---------------------------------------------------------------------------
# MPC (spec 04): linear time-varying model predictive control, OSQP.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class MpcParams:
    dt: float = 0.05                    # prediction step, s
    horizon_steps: int = 20             # N
    q_lateral: float = 4.0
    q_heading: float = 1.0
    r_steer: float = 1.0
    r_rate: float = 0.01
    deadline_sec: float = 0.008
    max_iter: int = 4000
    max_consecutive_misses: int = 3
    delay_compensation: float = 0.05    # mpc.delay_compensation, s (predict_input)


class MissPolicy:
    """The consecutive-miss fallback/latch shared by node and harness.

    A single MPC miss falls back to pure pursuit for that tick; too many in
    a row latches a stop until `clear()`. Kept as one small class so both
    call sites apply exactly the same rule and it is unit-testable unsourced.
    """

    def __init__(self, max_consecutive: int):
        if not (isinstance(max_consecutive, int) and max_consecutive >= 1):
            raise ValueError(
                f'mpc.max_consecutive_misses must be an int >= 1, got {max_consecutive}')
        self.max_consecutive = max_consecutive
        self._consecutive = 0
        self._latched = False

    def on_success(self) -> None:
        self._consecutive = 0

    def on_miss(self) -> str:
        """Record a miss; 'fallback' while under the limit, 'latched' past it."""
        self._consecutive += 1
        if self._consecutive >= self.max_consecutive:
            self._latched = True
            return 'latched'
        return 'fallback'

    def latched(self) -> bool:
        return self._latched

    def clear(self) -> None:
        self._consecutive = 0
        self._latched = False


# How far past the steering-angle bound an OSQP solution may land before it
# counts as a miss (see MpcLaw.steer). Above the solver's 1e-5 tolerance.
BOUND_TOLERANCE = 1e-4


class MpcLaw:
    """Linear time-varying MPC over the [e, theta] error, solved with OSQP.

    Plans `horizon_steps` steering moves `dt` apart that track the line
    while respecting the steering angle and rate limits as hard constraints,
    applies the first, and re-plans next tick. The model is LQR's kinematic
    bicycle with the tan(delta) linearization point carried along the
    horizon (the "time-varying" part), plus an LQR terminal cost so the
    short horizon behaves like an infinite one on straights.

    osqp is imported lazily in __init__ so the rest of this module (and all
    the other laws) work without it installed.
    """

    name = 'mpc'

    def __init__(self, limits: VehicleLimits, params: MpcParams):
        # Validate every parameter before touching osqp, so a bad config
        # fails identically with or without the solver installed.
        if not (math.isfinite(params.dt) and 0.0 < params.dt <= 0.2):
            raise ValueError(f'mpc.dt must be in (0, 0.2], got {params.dt}')
        if not (isinstance(params.horizon_steps, int) and 2 <= params.horizon_steps <= 60):
            raise ValueError(
                f'mpc.horizon_steps must be an int in [2, 60], got {params.horizon_steps}')
        for name, value in (('mpc.q_lateral', params.q_lateral),
                            ('mpc.q_heading', params.q_heading),
                            ('mpc.r_steer', params.r_steer)):
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be > 0, got {value}')
        if not (math.isfinite(params.r_rate) and params.r_rate >= 0.0):
            raise ValueError(f'mpc.r_rate must be >= 0, got {params.r_rate}')
        if not (math.isfinite(params.deadline_sec) and params.deadline_sec > 0.0):
            raise ValueError(f'mpc.deadline_sec must be > 0, got {params.deadline_sec}')
        if not (isinstance(params.max_iter, int) and params.max_iter >= 1):
            raise ValueError(f'mpc.max_iter must be an int >= 1, got {params.max_iter}')
        _check_delay('mpc.delay_compensation', params.delay_compensation)

        self.limits = limits
        self.params = params
        self.miss_policy = MissPolicy(params.max_consecutive_misses)
        self.solves_since_reset = 0
        self._warm_start = None

        try:
            import osqp  # noqa: F401
        except ImportError as exc:
            raise RuntimeError(
                "steering_law 'mpc' needs the 'osqp' package, which is not "
                "installed. See spec 04 G0 "
                "(docs/superpowers/specs/2026-09-27-tracking-04-mpc.md) for the "
                "install decision and benchmark gate before wiring MPC into "
                "the node.") from exc
        import scipy.sparse  # noqa: F401
        self._osqp = osqp
        self._sparse = scipy.sparse

        # Terminal cost reuses the LQR DARE at the prediction step dt_p -- a
        # P computed at the wrong dt is the wrong terminal cost.
        self._terminal = LqrGainTable(
            params.q_lateral, params.q_heading, params.r_steer,
            params.dt, limits.wheelbase, LqrGainTable.V_FLOOR + 6.0)
        # Set OSQP up once with the constant sparsity pattern (see the
        # "condensed QP" note in spec 04): a full upper-triangular Hessian
        # pattern and a fixed (2N x N) constraint pattern, so a zero *value*
        # never changes the pattern and per-tick updates are cheap. scipy.sparse
        # drops explicit zeros from .data, which makes a later update look like
        # a partial one, so the per-tick matrices are rebuilt onto the *same*
        # (row, col) coordinates the pattern was built from -- explicit zeros
        # included.
        n = params.horizon_steps
        self._P_pattern = self._sparse.csc_matrix(np.triu(np.ones((n, n))))
        self._P_coo = self._P_pattern.tocoo()
        A_pattern = np.vstack([np.eye(n), np.tril(np.ones((n, n)))])
        self._A_pattern = self._sparse.csc_matrix(A_pattern)
        self._A_coo = self._A_pattern.tocoo()
        self._solver = self._osqp.OSQP()
        self._solver.setup(
            P=self._P_pattern, q=np.zeros(n),
            A=self._A_pattern, l=np.zeros(2 * n), u=np.zeros(2 * n),
            eps_abs=1e-5, eps_rel=1e-5,
            max_iter=params.max_iter,
            polishing=False, warm_starting=True,
            time_limit=params.deadline_sec,
            verbose=False)

    def reset(self) -> None:
        self._warm_start = None
        self.solves_since_reset = 0

    # -- QP assembly -----------------------------------------------------

    def _reference(self, inp: TrackingInput, path: PathContext):
        """Sample curvature, feedforward, and speed along the horizon."""
        proj = racing_math.project_onto_path(
            path.xy, path.seg_len, (inp.x, inp.y), inp.nearest_idx, path.closed)
        s0 = proj.arc_length
        n = self.params.horizon_steps
        dt_p = self.params.dt
        v = max(inp.speed, LqrGainTable.V_FLOOR)
        speeds = np.zeros(n)
        kappas = np.zeros(n)
        ff = np.zeros(n)
        s = s0
        for k in range(n):
            speeds[k] = max(
                sample_at_arc(path.speed_profile, path.cumulative, s, path.closed),
                LqrGainTable.V_FLOOR)
            kappas[k] = sample_at_arc(path.curvature, path.cumulative, s, path.closed)
            ff[k] = math.atan(self.limits.wheelbase * kappas[k])
            if k + 1 < n:
                s = s + speeds[k] * dt_p
                if not path.closed:
                    s = min(s, float(path.cumulative[-1]))
        return proj, speeds, kappas, ff

    @staticmethod
    def _condensed_dynamics(speeds, ff, dt_p, wheelbase):
        """(Phi, Gamma) such that the stacked states x_1..x_N of
        x_{k+1} = A_k x_k + B_k (delta_k - ff_k) are Phi x_0 + Gamma (delta - ff).

        A_k = [[1, v_k dt_p], [0, 1]],  B_k = [[0], [v_k dt_p sec^2(ff_k) / L]].
        Every A_k is a shear and a product of shears is the shear of the
        summed offsets, so with c_k = a_0 + ... + a_k (a_k = v_k dt_p) and
        b_j = v_j dt_p sec^2(ff_j) / L:
          (A_k ... A_0)          = [[1, c_k], [0, 1]]               (Phi)
          (A_k ... A_{j+1}) B_j  = [(c_k - c_j) b_j, b_j],  j <= k  (Gamma)
        Closed form rather than an O(N^3) loop of 2x2 products: this runs
        inside the solve deadline every tick.
        """
        speeds = np.asarray(speeds, dtype=float)
        ff = np.asarray(ff, dtype=float)
        n = len(speeds)
        a = speeds * dt_p
        b = speeds * dt_p / (np.cos(ff) ** 2) / wheelbase
        c = np.cumsum(a)
        lower = np.tril(np.ones((n, n)))
        Phi = np.zeros((2 * n, 2))
        Phi[0::2, 0] = 1.0
        Phi[0::2, 1] = c
        Phi[1::2, 1] = 1.0
        Gamma = np.zeros((2 * n, n))
        Gamma[0::2] = (c[:, None] - c[None, :]) * b[None, :] * lower
        Gamma[1::2] = b[None, :] * lower
        return Phi, Gamma

    def _build_qp(self, inp: TrackingInput, path: PathContext):
        proj, speeds, _kappas, ff = self._reference(inp, path)
        n = self.params.horizon_steps
        dt_p = self.params.dt
        L = self.limits.wheelbase
        e0 = proj.lateral_error
        theta0 = _wrap(inp.yaw - proj.heading)
        prev = float(np.clip(
            inp.previous_steering,
            -self.limits.max_steering_angle, self.limits.max_steering_angle))

        Phi, Gamma = self._condensed_dynamics(speeds, ff, dt_p, L)

        x0 = np.array([e0, theta0])

        # Cost in x: sum_{k=1}^{N-1} x_k' Q x_k + x_N' P_N x_N.
        # Build the (2N x 2N) block-diagonal weight W over x_1..x_N.
        Q = np.diag([self.params.q_lateral, self.params.q_heading])
        P_N = self._terminal.riccati_at(speeds[-1])
        W = np.zeros((2 * n, 2 * n))
        for k in range(n):
            W[2 * k:2 * k + 2, 2 * k:2 * k + 2] = P_N if k == n - 1 else Q

        # Tracking weight on (delta - ff): r_steer * I.
        R = self.params.r_steer * np.eye(n)
        # Rate weight: r_rate * sum ((delta_k - delta_{k-1}) / tau_k)^2
        # = r_rate * (D delta - d0)' T (D delta - d0), D the difference matrix.
        D = np.diff(np.eye(n), axis=0)  # (n-1) x n: delta_k - delta_{k-1}, k>=1
        tau = np.full(n, dt_p)
        tau[0] = self.limits.control_dt
        # rows of the full difference including the k=0 term (delta_0 - prev):
        Dfull = np.zeros((n, n))
        Dfull[0, 0] = 1.0
        Dfull[1:] = D
        Tinv = np.diag(1.0 / (tau ** 2))
        Rrate = self.params.r_rate * (Dfull.T @ Tinv @ Dfull)
        rate_offset = np.zeros(n)   # the "- prev" constant enters via Dfull@delta - b0
        rate_offset[0] = prev
        # expand r_rate * ||Dfull delta - rate_offset||^2_T
        #   = delta' (Dfull' T Dfull) delta - 2 (Dfull' T rate_offset)' delta + const
        Rrate_lin = -2.0 * self.params.r_rate * (Dfull.T @ Tinv @ rate_offset)

        # Total Hessian/gradient in delta:
        #   state cost: (Gamma (delta-ff) + Phi x0)' W (Gamma (delta-ff) + Phi x0)
        #   tracking:   r_steer (delta-ff)' (delta-ff)
        #   rate:       above
        f = ff
        G = Gamma
        w0 = Phi @ x0
        H = (G.T @ W @ G) + R + Rrate
        q = (2.0 * (G.T @ W @ (w0 - G @ f))
             - 2.0 * self.params.r_steer * f
             + Rrate_lin)

        # Bounds: angle and rate. Angle: |delta_k| <= max_steering_angle.
        # Rate: |delta_k - delta_{k-1}| <= max_steering_rate * tau_k, where
        # delta_{-1} = previous_steering (clamped), folded into the k=0 row.
        max_ang = self.limits.max_steering_angle
        max_rate = self.limits.max_steering_rate
        lower_delta = -max_ang * np.ones(n)
        upper_delta = max_ang * np.ones(n)
        # Inequality rows on the difference Dfull: (Dfull delta)_k = delta_k -
        # delta_{k-1}. For k=0 the "previous" is the constant prev, so
        #   -max_rate*tau_0 <= delta_0 - prev <= max_rate*tau_0 becomes
        #   prev - max_rate*tau_0 <= delta_0 <= prev + max_rate*tau_0.
        l_ineq = -max_rate * tau
        u_ineq = max_rate * tau
        l_ineq[0] += prev
        u_ineq[0] += prev

        # Stack box (as identity rows) and rate inequalities.
        A = np.vstack([np.eye(n), Dfull])
        l = np.concatenate([lower_delta, l_ineq])
        u = np.concatenate([upper_delta, u_ineq])
        return H, q, A, l, u, ff, proj

    def steer(self, inp: TrackingInput, path: PathContext) -> SteeringResult:
        _require_finite(inp.x, inp.y, inp.yaw, inp.speed, inp.previous_steering)
        started = time.perf_counter()
        inp = predict_input(inp, path, self.params.delay_compensation,
                            self.limits.wheelbase)
        H, q, A, l, u, ff, proj = self._build_qp(inp, path)
        n = self.params.horizon_steps

        # Write this tick's values into the fixed sparsity patterns and update
        # in place (no re-setup): rebuild each matrix onto the *same* (row,
        # col) coordinates the setup pattern uses -- explicit zeros included --
        # so a value happening to be 0 can never shrink nnz into a partial
        # update. OSQP's P is the upper triangle of 2*H (it minimizes
        # 0.5 x'Px + q'x over the upper triangle, and H is the gradient-2x
        # Hessian of the condensed cost).
        P_new = self._sparse.csc_matrix(
            (2.0 * H[self._P_coo.row, self._P_coo.col],
             (self._P_coo.row, self._P_coo.col)), shape=(n, n))
        A_new = self._sparse.csc_matrix(
            (A[self._A_coo.row, self._A_coo.col],
             (self._A_coo.row, self._A_coo.col)), shape=(2 * n, n))
        self._solver.update(Px=P_new.data, q=q, Ax=A_new.data, l=l, u=u)
        if self._warm_start is not None:
            self._solver.warm_start(x=self._warm_start)
        result = self._solver.solve()
        elapsed = time.perf_counter() - started

        status_ok = result.info.status == 'solved'
        if not status_ok:
            # 'solved inaccurate', max-iterations, and OSQP's own time-limit
            # all count as misses -- checked before the wall-clock deadline so
            # a solver that blew its own limit reports 'status', not both.
            self._warm_start = None
            raise MpcMiss(f'status: osqp returned "{result.info.status}"')
        if elapsed > self.params.deadline_sec:
            self._warm_start = None
            raise MpcMiss(
                f'deadline: solve took {elapsed * 1e3:.1f}ms '
                f'(limit {self.params.deadline_sec * 1e3:.1f}ms)')
        delta = np.asarray(result.x, dtype=float).reshape(-1)
        if delta.shape[0] != n or not np.all(np.isfinite(delta)):
            self._warm_start = None
            raise MpcMiss('solution: non-finite or wrong-size decision vector')
        max_ang = self.limits.max_steering_angle
        # OSQP (eps_abs = eps_rel = 1e-5, polishing off) only meets constraints
        # to about 1e-5, so a tighter check than that turns every tick where
        # the angle bound is active -- i.e. every tight corner -- into a miss.
        # 1e-4 rad (0.006 deg) is above the solver's own tolerance and far
        # below anything the servo resolves; the output is clipped below.
        if np.any(delta > max_ang + BOUND_TOLERANCE) or np.any(delta < -max_ang - BOUND_TOLERANCE):
            self._warm_start = None
            raise MpcMiss('solution: violates the steering angle bound')

        self._warm_start = delta
        self.solves_since_reset += 1

        planned = self._project_onto_limits(
            delta, inp.previous_steering, max_ang, self.limits.max_steering_rate,
            [self.limits.control_dt] + [self.params.dt] * (n - 1))
        steering = planned[0]
        e = proj.lateral_error
        theta = _wrap(inp.yaw - proj.heading)
        detail = (f"mpc e={e:+.3f}m th={theta:+.3f}rad "
                  f"solve={elapsed * 1e3:.1f}ms it={result.info.iter}")
        return SteeringResult(steering=steering, unclipped=float(delta[0]),
                              detail=detail, predicted=tuple(planned))

    @staticmethod
    def _project_onto_limits(delta, previous, max_angle, max_rate, taus):
        """The plan moved, move by move, onto the exact angle and rate bounds.

        OSQP meets its constraints only to ~1e-5, so a move at an active
        bound can sit just past it; the plan that leaves this law (and the
        intent the dashboard draws from it) must not. Each move is clamped to
        within max_rate * tau of the one before, then to the angle bound; the
        first is measured from the last command, as in _build_qp.
        """
        out = []
        last = float(np.clip(previous, -max_angle, max_angle))
        for value, tau in zip(delta, taus):
            step = max_rate * tau
            value = min(max(float(value), last - step), last + step)
            last = min(max(value, -max_angle), max_angle)
            out.append(last)
        return out


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def make_law(name: str, limits: VehicleLimits,
             params: Mapping[str, float]) -> TrackingLaw:
    """Build one law by name. `params` carries the law-specific knobs,
    keyed by their *dotted* node parameter names ('stanley.gain',
    'lqr.q_lateral', 'mpc.horizon_steps', ...).

    Unknown name -> ValueError(match='unknown steering_law'). An invalid
    parameter value -> ValueError naming the parameter.
    """
    if name == 'pure_pursuit':
        return PurePursuitLaw(limits, PurePursuitParams(
            min_lookahead=float(params.get('min_lookahead', 0.6)),
            max_lookahead=float(params.get('max_lookahead', 1.5)),
            lookahead_speed_gain=float(params.get('lookahead_speed_gain', 0.15)),
        ))
    if name == 'stanley':
        d = StanleyParams()
        return StanleyLaw(limits, StanleyParams(
            gain=float(params.get('stanley.gain', d.gain)),
            softening=float(params.get('stanley.softening', d.softening)),
            delay_compensation=float(params.get('stanley.delay_compensation', d.delay_compensation)),
            steering_time_constant=float(params.get('stanley.steering_time_constant', d.steering_time_constant)),
        ))
    if name == 'lqr':
        d = LqrParams()
        return LqrLaw(limits, LqrParams(
            q_lateral=float(params.get('lqr.q_lateral', d.q_lateral)),
            q_heading=float(params.get('lqr.q_heading', d.q_heading)),
            r_steer=float(params.get('lqr.r_steer', d.r_steer)),
            v_grid_max=float(params.get('lqr.v_grid_max', d.v_grid_max)),
            delay_compensation=float(params.get('lqr.delay_compensation', d.delay_compensation)),
            steering_time_constant=float(params.get('lqr.steering_time_constant', d.steering_time_constant)),
        ))
    if name == 'mpc':
        d = MpcParams()
        return MpcLaw(limits, MpcParams(
            dt=float(params.get('mpc.dt', d.dt)),
            horizon_steps=int(params.get('mpc.horizon_steps', d.horizon_steps)),
            q_lateral=float(params.get('mpc.q_lateral', d.q_lateral)),
            q_heading=float(params.get('mpc.q_heading', d.q_heading)),
            r_steer=float(params.get('mpc.r_steer', d.r_steer)),
            r_rate=float(params.get('mpc.r_rate', d.r_rate)),
            deadline_sec=float(params.get('mpc.deadline_sec', d.deadline_sec)),
            max_iter=int(params.get('mpc.max_iter', d.max_iter)),
            max_consecutive_misses=int(params.get('mpc.max_consecutive_misses', d.max_consecutive_misses)),
            delay_compensation=float(params.get('mpc.delay_compensation', d.delay_compensation)),
        ))
    raise ValueError(
        f"unknown steering_law '{name}' (expected one of {LAW_NAMES})")
