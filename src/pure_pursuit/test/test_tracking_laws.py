"""Unit tests for pure_pursuit.tracking_laws -- the steering-law seam.

ROS-free (no rclpy), runnable with a bare:

    python3 -m pytest src/pure_pursuit/test/test_tracking_laws.py -v

Every expected value is a closed form recomputed in the test, an invariant,
or (for LQR/MPC) an independent computation of the same equation -- never
"the value the code happens to return today" (TEST_QUALITY_STANDARDS A3).
The one exception is the pure-pursuit refactor regression, labelled as a
change-detector where it lives.

osqp-backed MPC tests use pytest.importorskip -- the only allowed skip,
which becomes a hard failure once osqp is in package.xml (spec 04).
"""
import math
import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pure_pursuit import racing_math, tracking_laws  # noqa: E402
from pure_pursuit.tracking_laws import (  # noqa: E402
    LqrGainTable, MissPolicy, PathContext, SteeringResult, TrackingInput,
    VehicleLimits,
)

PACKAGE_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
CONFIG = os.path.join(PACKAGE_DIR, 'config', 'pure_pursuit.yaml')

# The numbers the specs' worked examples use. L = 0.36 is the measured
# wheelbase (docs/hardware-reference.md); 0.26 rad is the steering clip the
# specs' closed forms assume.
LIMITS = VehicleLimits(
    wheelbase=0.36, max_steering_angle=0.26, max_steering_rate=1.0,
    control_dt=0.025)
L = LIMITS.wheelbase
GAIN = 1.6
SOFTENING = 0.4


def straight_context(n=50, length=10.0, speed=1.0, closed=False):
    """A straight racing line along +x at `speed`, zero curvature."""
    xy = np.column_stack([np.linspace(0.0, length, n), np.zeros(n)])
    seg_len = racing_math.compute_segment_lengths(xy, closed=closed)
    return PathContext(
        xy=xy, seg_len=seg_len,
        cumulative=racing_math.compute_cumulative_arc_length(seg_len),
        speed_profile=np.full(n, speed), curvature=np.zeros(n), closed=closed)


def circle_context(radius=2.0, spacing=0.02, ccw=True, speed=1.0):
    """A closed circle, CCW (+ curvature) or CW, waypoints `spacing` apart."""
    circumference = 2.0 * math.pi * radius
    n = max(8, int(round(circumference / spacing)))
    angles = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    if not ccw:
        angles = angles[::-1]
    xy = np.column_stack([radius * np.cos(angles), radius * np.sin(angles)])
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    curvature = racing_math.estimate_signed_path_curvature(xy, closed=True)
    return PathContext(
        xy=xy, seg_len=seg_len,
        cumulative=racing_math.compute_cumulative_arc_length(seg_len),
        speed_profile=np.full(n, speed), curvature=curvature, closed=True)


def make(name, **params):
    merged = dict(params)
    return tracking_laws.make_law(name, LIMITS, merged)


# ============================================================================
# Factory and shared contract
# ============================================================================

def test_make_law_unknown_name_raises():
    with pytest.raises(ValueError, match='unknown steering_law'):
        tracking_laws.make_law('not_a_law', LIMITS, {})


def test_make_law_builds_every_known_name_except_mpc_without_osqp():
    for name in ('pure_pursuit', 'stanley', 'lqr'):
        law = tracking_laws.make_law(name, LIMITS, {})
        assert law.name == name


def test_pure_pursuit_on_a_straight_line_at_zero_error_is_zero():
    ctx = straight_context()
    law = make('pure_pursuit')
    result = law.steer(TrackingInput(
        x=1.0, y=0.0, yaw=0.0, speed=1.0, nearest_idx=5, previous_steering=0.0), ctx)
    assert result.steering == pytest.approx(0.0, abs=1e-12)


def test_pure_pursuit_oracle_closed_form():
    # Oracle (closed form, spec 00): a lookahead target at body (Ld, y_t)
    # gives kappa = 2 y_t / Ld^2 and delta = atan(L * kappa).
    ctx = straight_context(speed=1.0)
    law = make('pure_pursuit', min_lookahead=0.6, max_lookahead=1.5,
               lookahead_speed_gain=0.15)
    inp = TrackingInput(x=0.0, y=0.0, yaw=0.0, speed=1.0, nearest_idx=0,
                        previous_steering=0.0)
    result = law.steer(inp, ctx)
    # Recompute what the law should have done, step by step.
    lookahead = racing_math.adaptive_lookahead(1.0, 0.15, 0.6, 1.5)
    target_idx = racing_math.find_lookahead_index(ctx.seg_len, 0, lookahead, closed=False)
    tx, ty = ctx.xy[target_idx]
    x_body, y_body = racing_math.world_to_body(tx - 0.0, ty - 0.0, 0.0)
    kappa = racing_math.steering_arc_curvature(x_body, y_body)
    expected = math.atan(L * kappa)
    assert result.steering == pytest.approx(expected, abs=1e-12)


def test_pure_pursuit_steers_left_for_a_target_to_the_left():
    # A car below the line (y<0, i.e. the line is to its left) must steer
    # positive (left) to reach it.
    ctx = straight_context()
    law = make('pure_pursuit')
    result = law.steer(TrackingInput(
        x=1.0, y=-0.3, yaw=0.0, speed=1.0, nearest_idx=5, previous_steering=0.0), ctx)
    assert result.steering > 0.0


def test_pure_pursuit_clips_to_max_steering():
    # A near-90deg target demands far more than max_steering_angle.
    ctx = straight_context()
    law = make('pure_pursuit')
    result = law.steer(TrackingInput(
        x=1.0, y=-2.0, yaw=0.0, speed=1.0, nearest_idx=5, previous_steering=0.0), ctx)
    assert result.steering == pytest.approx(LIMITS.max_steering_angle, abs=1e-12)
    assert result.unclipped > LIMITS.max_steering_angle


def test_every_law_rejects_nan_input():
    ctx = straight_context()
    for name in ('pure_pursuit', 'stanley', 'lqr'):
        law = make(name)
        with pytest.raises(tracking_laws.TrackingLawError, match='non-finite'):
            law.steer(TrackingInput(x=math.nan, y=0.0, yaw=0.0, speed=1.0,
                                    nearest_idx=5, previous_steering=0.0), ctx)


def test_yaml_mirrors_the_param_dataclasses():
    """config/pure_pursuit.yaml's tracking-law defaults must equal the
    dataclass defaults, so the harness and the car run the same gains."""
    with open(CONFIG) as handle:
        cfg = yaml.safe_load(handle)['pure_pursuit_node']['ros__parameters']
    stanley = tracking_laws.StanleyParams()
    assert cfg['stanley.gain'] == pytest.approx(stanley.gain)
    assert cfg['stanley.softening'] == pytest.approx(stanley.softening)
    assert cfg['stanley.delay_compensation'] == pytest.approx(stanley.delay_compensation)
    assert cfg['stanley.steering_time_constant'] == pytest.approx(
        stanley.steering_time_constant)
    lqr = tracking_laws.LqrParams()
    assert cfg['lqr.q_lateral'] == pytest.approx(lqr.q_lateral)
    assert cfg['lqr.q_heading'] == pytest.approx(lqr.q_heading)
    assert cfg['lqr.r_steer'] == pytest.approx(lqr.r_steer)
    assert cfg['lqr.v_grid_max'] == pytest.approx(lqr.v_grid_max)
    assert cfg['lqr.delay_compensation'] == pytest.approx(lqr.delay_compensation)
    assert cfg['lqr.steering_time_constant'] == pytest.approx(lqr.steering_time_constant)
    mpc = tracking_laws.MpcParams()
    assert cfg['mpc.dt'] == pytest.approx(mpc.dt)
    assert cfg['mpc.horizon_steps'] == mpc.horizon_steps
    assert cfg['mpc.q_lateral'] == pytest.approx(mpc.q_lateral)
    assert cfg['mpc.q_heading'] == pytest.approx(mpc.q_heading)
    assert cfg['mpc.r_steer'] == pytest.approx(mpc.r_steer)
    assert cfg['mpc.r_rate'] == pytest.approx(mpc.r_rate)
    assert cfg['mpc.deadline_sec'] == pytest.approx(mpc.deadline_sec)
    assert cfg['mpc.max_iter'] == mpc.max_iter
    assert cfg['mpc.max_consecutive_misses'] == mpc.max_consecutive_misses
    assert cfg['mpc.delay_compensation'] == pytest.approx(mpc.delay_compensation)


# ============================================================================
# Stanley (spec 02). Every expected value recomputed from
#   delta = theta_e - atan(k e_f / (v + v_soft)) + atan(L kappa_f).
# ============================================================================

def _stanley():
    # The bare law: delay compensation and output smoothing off (they have
    # their own tests below), so each expected value is the formula alone.
    return make('stanley', **{'stanley.gain': GAIN, 'stanley.softening': SOFTENING,
                              'stanley.delay_compensation': 0.0,
                              'stanley.steering_time_constant': 0.0})


def test_stanley_test1_on_line_zero():
    ctx = straight_context(speed=1.0)
    r = _stanley().steer(TrackingInput(x=0.0, y=0.0, yaw=0.0, speed=1.0,
                                       nearest_idx=0, previous_steering=0.0), ctx)
    assert r.steering == pytest.approx(0.0, abs=1e-9)


def test_stanley_test2_left_of_line():
    # Front axle at y=+0.2 on a +x line: delta = -atan(k*0.2/(v+v_soft)).
    ctx = straight_context(speed=1.0)
    r = _stanley().steer(TrackingInput(x=0.0, y=0.2, yaw=0.0, speed=1.0,
                                       nearest_idx=0, previous_steering=0.0), ctx)
    expected = -math.atan(GAIN * 0.2 / (1.0 + SOFTENING))
    assert r.steering == pytest.approx(expected, abs=1e-9)  # rad
    assert r.steering < 0.0


def test_stanley_test3_right_of_line_mirrors():
    ctx = straight_context(speed=1.0)
    r = _stanley().steer(TrackingInput(x=0.0, y=-0.2, yaw=0.0, speed=1.0,
                                       nearest_idx=0, previous_steering=0.0), ctx)
    expected = math.atan(GAIN * 0.2 / (1.0 + SOFTENING))
    assert r.steering == pytest.approx(expected, abs=1e-9)


def test_stanley_test4_uses_front_axle_not_rear():
    # Rear axle on the line, yaw +0.1: the front axle is off the line, so the
    # lateral term is nonzero -- a rear-axle implementation would return -0.1.
    ctx = straight_context(speed=1.0)
    r = _stanley().steer(TrackingInput(x=0.0, y=0.0, yaw=0.1, speed=1.0,
                                       nearest_idx=0, previous_steering=0.0), ctx)
    e_f = L * math.sin(0.1)   # front-axle lateral offset
    expected = -0.1 - math.atan(GAIN * e_f / (1.0 + SOFTENING))
    assert r.steering == pytest.approx(expected, abs=1e-9)
    assert r.steering != pytest.approx(-0.1, abs=1e-3)


def test_stanley_test5_heading_wrap_seam():
    # Path along -x (psi_p = pi), car on it with yaw = -pi + 0.05.
    xy = np.column_stack([np.linspace(10.0, 0.0, 50), np.zeros(50)])
    seg_len = racing_math.compute_segment_lengths(xy, closed=False)
    ctx = PathContext(xy=xy, seg_len=seg_len,
                      cumulative=racing_math.compute_cumulative_arc_length(seg_len),
                      speed_profile=np.ones(50), curvature=np.zeros(50), closed=False)
    yaw = -math.pi + 0.05
    rear_x, rear_y = 5.0, 0.0
    # Seed nearest from the front axle (the law searches there, then walks).
    nearest, _ = racing_math.find_nearest_index(
        xy, (rear_x + L * math.cos(yaw), rear_y + L * math.sin(yaw)), closed=False)
    r = _stanley().steer(TrackingInput(x=rear_x, y=rear_y, yaw=yaw, speed=1.0,
                                       nearest_idx=nearest, previous_steering=0.0), ctx)
    # theta_e = wrap(psi_p - yaw) = wrap(pi - (-pi+0.05)) = wrap(2pi-0.05) = -0.05.
    # The heading term is the whole point of the wrap test; the front-axle
    # lateral offset from yaw adds a small correction, recomputed in closed
    # form so a missing wrap (which would make theta_e ~ 2pi) still fails loudly.
    e_f = -L * math.sin(yaw)        # front axle drifts off the y=0 line by yaw
    expected = -0.05 - math.atan(GAIN * e_f / (1.0 + SOFTENING))
    assert r.steering == pytest.approx(expected, abs=1e-3)  # rad
    assert abs(r.steering) < 0.2   # a missing wrap would give ~ +/- 2pi


def _stanley_on_circle(radius, ccw):
    """Stanley's command with the rear axle on a circle, tangent to it.

    Returns (command, closed-form value of the formula at that pose, the
    front axle's lateral error). The closed form is recomputed here from
    the projection, independently of the law's code path."""
    ctx = circle_context(radius=radius, spacing=0.01, ccw=ccw, speed=1.0)
    alpha = math.pi / 4.0
    # On the polygon, not the true circle: the chord midpoint is inset.
    r_poly = radius - (ctx.seg_len[0] / 2.0) ** 2 / (2.0 * radius)
    rear = np.array([r_poly * math.cos(alpha), r_poly * math.sin(alpha)])
    yaw = alpha + (math.pi / 2.0 if ccw else -math.pi / 2.0)
    nearest, _ = racing_math.find_nearest_index(ctx.xy, (rear[0], rear[1]), closed=True)
    r = _stanley().steer(TrackingInput(x=rear[0], y=rear[1], yaw=yaw, speed=1.0,
                                       nearest_idx=nearest, previous_steering=0.0), ctx)
    front = rear + L * np.array([math.cos(yaw), math.sin(yaw)])
    nf, _ = racing_math.find_nearest_index(ctx.xy, front, closed=True)
    proj_f = racing_math.project_onto_path(ctx.xy, ctx.seg_len, front, nf, True)
    theta_e = (proj_f.heading - yaw + math.pi) % (2 * math.pi) - math.pi
    kappa_f = racing_math.interpolate_along(ctx.curvature, proj_f, True)
    big_r = 1.0 / abs(kappa_f)
    e_target = -math.copysign(math.hypot(big_r, L) - big_r, kappa_f)
    expected = theta_e - math.atan(
        GAIN * (proj_f.lateral_error - e_target) / (1.0 + SOFTENING))
    return r, expected, proj_f.lateral_error


def test_stanley_test6_circle_holds_the_kinematic_steering_ccw():
    # Oracle: the kinematic bicycle. A car whose rear axle runs on a circle
    # of radius R, tangent to it, keeps doing so only at delta = atan(L/R)
    # -- so that is what Stanley must command there, with no lateral
    # correction left over. (Spec 02's formula added +atan(L kappa) on top
    # of theta_e, which already carries it, and commanded ~2x this; see
    # docs/steering-laws.md#stanley.) R = 2 m keeps atan(L/R) = 0.178 rad
    # well inside the 0.26 rad test clip, so a doubled term cannot hide.
    r, expected, e_f = _stanley_on_circle(2.0, ccw=True)
    assert r.steering == pytest.approx(expected, abs=1e-9)  # rad, the formula
    # The front axle sits sqrt(R^2+L^2)-R outside (right of) a left bend.
    assert e_f == pytest.approx(-(math.hypot(2.0, L) - 2.0), abs=1e-3)  # m
    # 2 mrad: the path is a 1 cm polygon, so its heading and curvature are
    # piecewise, not the circle's exact ones.
    assert r.steering == pytest.approx(math.atan(L / 2.0), abs=2e-3)  # rad


def test_stanley_test7_circle_holds_the_kinematic_steering_cw():
    # Mirror of test 6: a right bend needs -atan(L/R), front axle outside
    # (left of) the bend.
    r, expected, e_f = _stanley_on_circle(2.0, ccw=False)
    assert r.steering == pytest.approx(expected, abs=1e-9)
    assert e_f == pytest.approx(math.hypot(2.0, L) - 2.0, abs=1e-3)
    assert r.steering == pytest.approx(-math.atan(L / 2.0), abs=2e-3)


def test_stanley_front_axle_offset_closed_form():
    # Rear axle on a circle of radius R: the front axle is L ahead along the
    # tangent, so at distance hypot(R, L) from the centre.
    assert tracking_laws.StanleyLaw.front_axle_offset(0.0, L) == 0.0
    for kappa in (0.5, -0.5, 1.0 / 1.35):
        big_r = 1.0 / abs(kappa)
        expected = -math.copysign(math.hypot(big_r, L) - big_r, kappa)
        assert tracking_laws.StanleyLaw.front_axle_offset(kappa, L) == pytest.approx(
            expected, abs=1e-12)  # m
    assert tracking_laws.StanleyLaw.front_axle_offset(1.0, L) < 0.0   # left bend
    assert tracking_laws.StanleyLaw.front_axle_offset(-1.0, L) > 0.0  # right bend


def test_stanley_test8_standstill_uses_softening_and_clips():
    ctx = straight_context(speed=1.0)
    r = _stanley().steer(TrackingInput(x=0.0, y=0.2, yaw=0.0, speed=0.0,
                                       nearest_idx=0, previous_steering=0.0), ctx)
    expected_unclipped = -math.atan(GAIN * 0.2 / (0.0 + SOFTENING))
    assert r.unclipped == pytest.approx(expected_unclipped, abs=1e-9)
    assert r.steering == pytest.approx(-LIMITS.max_steering_angle, abs=1e-12)


def test_stanley_test9_speed_reduces_lateral_term():
    ctx = straight_context(speed=3.0)
    r = _stanley().steer(TrackingInput(x=0.0, y=0.2, yaw=0.0, speed=3.0,
                                       nearest_idx=0, previous_steering=0.0), ctx)
    expected = -math.atan(GAIN * 0.2 / (3.0 + SOFTENING))
    assert r.steering == pytest.approx(expected, abs=1e-9)


def test_stanley_test10_mirror_invariant():
    # Reflecting the world in the x-axis must negate the steering.
    rng = np.random.default_rng(11)
    base = circle_context(radius=3.0, spacing=0.1, ccw=True)
    law = _stanley()
    for _ in range(50):
        idx = int(rng.integers(0, len(base.xy)))
        px, py = base.xy[idx] + rng.uniform(-0.5, 0.5, size=2)
        yaw = float(rng.uniform(-0.4, 0.4)) + math.atan2(
            base.xy[(idx + 1) % len(base.xy)][1] - base.xy[idx][1],
            base.xy[(idx + 1) % len(base.xy)][0] - base.xy[idx][0])
        inp = TrackingInput(x=px, y=py, yaw=yaw, speed=1.5,
                            nearest_idx=idx, previous_steering=0.0)
        original = law.steer(inp, base)
        # Mirror path and pose.
        mirrored = PathContext(
            xy=base.xy * np.array([1.0, -1.0]), seg_len=base.seg_len,
            cumulative=base.cumulative, speed_profile=base.speed_profile,
            curvature=-base.curvature, closed=base.closed)
        mirrored_inp = TrackingInput(x=px, y=-py, yaw=-yaw, speed=1.5,
                                     nearest_idx=idx, previous_steering=0.0)
        reflected = law.steer(mirrored_inp, mirrored)
        assert reflected.steering == pytest.approx(-original.steering, abs=1e-9)


def test_stanley_test12_param_validation():
    with pytest.raises(ValueError, match='stanley.gain'):
        make('stanley', **{'stanley.gain': 0.0})
    with pytest.raises(ValueError, match='stanley.gain'):
        make('stanley', **{'stanley.gain': -1.0})
    with pytest.raises(ValueError, match='stanley.softening'):
        make('stanley', **{'stanley.softening': 0.0})


def test_stanley_test13_open_path_beyond_end_is_finite():
    xy = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    seg_len = racing_math.compute_segment_lengths(xy, closed=False)
    ctx = PathContext(xy=xy, seg_len=seg_len,
                      cumulative=racing_math.compute_cumulative_arc_length(seg_len),
                      speed_profile=np.ones(3), curvature=np.zeros(3), closed=False)
    # Front axle 1 m past the end, still on the extension line (y=0).
    rear_x, rear_y, yaw = 3.0 - L, 0.0, 0.0
    nearest, _ = racing_math.find_nearest_index(xy, (rear_x, rear_y), closed=False)
    r = _stanley().steer(TrackingInput(x=rear_x, y=rear_y, yaw=yaw, speed=1.0,
                                       nearest_idx=nearest, previous_steering=0.0), ctx)
    assert math.isfinite(r.steering)
    assert r.steering == pytest.approx(0.0, abs=1e-9)


# ============================================================================
# LQR (spec 03). The oracle is an independent Riccati *recursion* in the test,
# never scipy's own answer (A3/A7).
# ============================================================================

def _riccati_recursion(A, B, Q, R, iterations=20000, tol=1e-12):
    """Independent oracle: iterate P <- Q + A'PA - A'PB(R+B'PB)^-1 B'PA from
    P=Q until convergence, then K = (R+B'PB)^-1 B'PA. Solves the same DARE
    scipy does, by a different route."""
    P = Q.copy()
    for _ in range(iterations):
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        P_next = Q + A.T @ P @ A - A.T @ P @ B @ K
        if np.max(np.abs(P_next - P)) < tol:
            P = P_next
            break
        P = P_next
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    return K, P


def _lqr():
    # The bare law, as _stanley(): prediction and smoothing off.
    return make('lqr', **{'lqr.q_lateral': 1.0, 'lqr.q_heading': 1.0,
                          'lqr.r_steer': 1.0, 'lqr.v_grid_max': 6.0,
                          'lqr.delay_compensation': 0.0,
                          'lqr.steering_time_constant': 0.0})


def test_lqr_test1_gain_matches_riccati_recursion():
    dt = LIMITS.control_dt
    table = LqrGainTable(1.0, 1.0, 1.0, dt, L, 6.0)
    Q = np.diag([1.0, 1.0])
    R = np.array([[1.0]])
    for v in (0.5, 1.5, 4.0):
        A = np.array([[1.0, v * dt], [0.0, 1.0]])
        B = np.array([[0.0], [v * dt / L]])
        K_expected, _ = _riccati_recursion(A, B, Q, R)
        K = table.gain_at(v)
        assert K[0] == pytest.approx(K_expected[0, 0], rel=1e-6)
        assert K[1] == pytest.approx(K_expected[0, 1], rel=1e-6)


def test_lqr_test2_closed_loop_is_stable():
    dt = LIMITS.control_dt
    table = LqrGainTable(1.0, 1.0, 1.0, dt, L, 6.0)
    v = LqrGainTable.V_FLOOR
    while v <= 6.0 + 1e-9:
        A = np.array([[1.0, v * dt], [0.0, 1.0]])
        B = np.array([[0.0], [v * dt / L]])
        K = table.gain_at(v)
        radius = max(abs(np.linalg.eigvals(A - B @ K.reshape(1, 2))))
        assert radius < 1.0
        v += LqrGainTable.V_STEP


def test_lqr_test3_gain_is_a_local_cost_minimum():
    dt = LIMITS.control_dt
    v = 1.5
    A = np.array([[1.0, v * dt], [0.0, 1.0]])
    B = np.array([[0.0], [v * dt / L]])
    Q = np.diag([1.0, 1.0])
    R = np.array([[1.0]])
    table = LqrGainTable(1.0, 1.0, 1.0, dt, L, 6.0)
    K = table.gain_at(v).reshape(1, 2)
    s0 = np.array([0.2, 0.05])

    def cost(Kmat):
        s = s0.copy()
        total = 0.0
        for _ in range(2000):
            u = -(Kmat @ s)[0]
            total += float(s @ Q @ s + u * R[0, 0] * u)
            s = A @ s + (B[:, 0] * u)
        return total

    base = cost(K)
    for component in (0, 1):
        for eps in (0.05, -0.05):
            Kd = K.copy()
            Kd[0, component] += eps
            assert cost(Kd) > base


def test_lqr_test4_lateral_sign():
    ctx = straight_context(speed=1.5)
    table = LqrGainTable(1.0, 1.0, 1.0, LIMITS.control_dt, L, 6.0)
    K = table.gain_at(1.5)
    r = _lqr().steer(TrackingInput(x=0.0, y=0.2, yaw=0.0, speed=1.5,
                                   nearest_idx=0, previous_steering=0.0), ctx)
    assert r.steering == pytest.approx(-K[0] * 0.2, abs=1e-9)
    assert r.steering < 0.0


def test_lqr_test5_heading_sign():
    ctx = straight_context(speed=1.5)
    table = LqrGainTable(1.0, 1.0, 1.0, LIMITS.control_dt, L, 6.0)
    K = table.gain_at(1.5)
    r = _lqr().steer(TrackingInput(x=0.0, y=0.0, yaw=0.1, speed=1.5,
                                   nearest_idx=0, previous_steering=0.0), ctx)
    assert r.steering == pytest.approx(-K[1] * 0.1, abs=1e-9)
    assert r.steering < 0.0


def test_lqr_test6_heading_wrap():
    xy = np.column_stack([np.linspace(10.0, 0.0, 50), np.zeros(50)])
    seg_len = racing_math.compute_segment_lengths(xy, closed=False)
    ctx = PathContext(xy=xy, seg_len=seg_len,
                      cumulative=racing_math.compute_cumulative_arc_length(seg_len),
                      speed_profile=np.full(50, 1.5), curvature=np.zeros(50), closed=False)
    table = LqrGainTable(1.0, 1.0, 1.0, LIMITS.control_dt, L, 6.0)
    K = table.gain_at(1.5)
    yaw = -math.pi + 0.05
    nearest, _ = racing_math.find_nearest_index(xy, (5.0, 0.0), closed=False)
    r = _lqr().steer(TrackingInput(x=5.0, y=0.0, yaw=yaw, speed=1.5,
                                   nearest_idx=nearest, previous_steering=0.0), ctx)
    # theta = wrap((-pi+0.05) - pi) = +0.05
    assert r.steering == pytest.approx(-K[1] * 0.05, abs=1e-6)


def test_lqr_test7_circle_feedforward():
    # Car on the waypoint polygon at a mid-segment point, tangent to the
    # segment: e = 0 and theta = 0 exactly, so delta = atan(L * kappa) (the
    # feedforward). The oracle kappa is the signed curvature, separately
    # tested to equal +1/R to 1e-3 here.
    for ccw, yaw_off in ((True, math.pi / 2.0), (False, -math.pi / 2.0)):
        ctx = circle_context(radius=2.0, spacing=0.02, ccw=ccw, speed=1.5)
        alpha = math.pi / 4.0
        half = ctx.seg_len[0] / 2.0
        r_in = 2.0 - half ** 2 / (2.0 * 2.0)
        px, py = r_in * math.cos(alpha), r_in * math.sin(alpha)
        yaw = alpha + yaw_off
        nearest, _ = racing_math.find_nearest_index(ctx.xy, (px, py), closed=True)
        proj = racing_math.project_onto_path(ctx.xy, ctx.seg_len, (px, py), nearest, True)
        kappa = racing_math.interpolate_along(ctx.curvature, proj, True)
        assert proj.lateral_error == pytest.approx(0.0, abs=1e-6)
        r = _lqr().steer(TrackingInput(x=px, y=py, yaw=yaw, speed=1.5,
                                       nearest_idx=nearest, previous_steering=0.0), ctx)
        assert r.steering == pytest.approx(math.atan(L * kappa), abs=1e-3)
    # Sanity: the magnitude really is the R=2 feedforward both ways.
    assert math.atan(L * kappa) == pytest.approx(-math.atan(L / 2.0), abs=1e-3)


def test_lqr_test8_grid_interpolation_and_clamping():
    dt = LIMITS.control_dt
    table = LqrGainTable(1.0, 1.0, 1.0, dt, L, 6.0)
    # Exact grid speed equals the direct DARE result.
    Q = np.diag([1.0, 1.0])
    R = np.array([[1.0]])
    v = 1.0
    A = np.array([[1.0, v * dt], [0.0, 1.0]])
    B = np.array([[0.0], [v * dt / L]])
    K_expected, _ = _riccati_recursion(A, B, Q, R)
    assert table.gain_at(v)[0] == pytest.approx(K_expected[0, 0], rel=1e-6)
    # A midpoint is the mean of its neighbours.
    lo = table.gain_at(1.0)
    hi = table.gain_at(1.05)
    mid = table.gain_at(1.025)
    assert mid[0] == pytest.approx(0.5 * (lo[0] + hi[0]), rel=1e-9)
    # Clamping: below the floor and above the grid return the end values.
    np.testing.assert_allclose(table.gain_at(0.0), table.gain_at(LqrGainTable.V_FLOOR))
    np.testing.assert_allclose(table.gain_at(100.0), table.gain_at(6.0))


def test_lqr_test9_dt_is_the_control_period():
    fast = LqrGainTable(1.0, 1.0, 1.0, 0.025, L, 6.0)
    slow = LqrGainTable(1.0, 1.0, 1.0, 0.1, L, 6.0)
    assert not np.allclose(fast.gain_at(1.5), slow.gain_at(1.5))
    # Each matches the recursion for its own dt.
    Q = np.diag([1.0, 1.0])
    R = np.array([[1.0]])
    for table, dt in ((fast, 0.025), (slow, 0.1)):
        v = 1.5
        A = np.array([[1.0, v * dt], [0.0, 1.0]])
        B = np.array([[0.0], [v * dt / L]])
        K_expected, _ = _riccati_recursion(A, B, Q, R)
        assert table.gain_at(v)[0] == pytest.approx(K_expected[0, 0], rel=1e-6)


def test_lqr_test10_mirror_invariant():
    rng = np.random.default_rng(5)
    base = circle_context(radius=3.0, spacing=0.1, ccw=True, speed=1.5)
    law = _lqr()
    for _ in range(50):
        idx = int(rng.integers(0, len(base.xy)))
        px, py = base.xy[idx] + rng.uniform(-0.5, 0.5, size=2)
        yaw = float(rng.uniform(-0.4, 0.4))
        inp = TrackingInput(x=px, y=py, yaw=yaw, speed=1.5,
                            nearest_idx=idx, previous_steering=0.0)
        original = law.steer(inp, base)
        mirrored = PathContext(
            xy=base.xy * np.array([1.0, -1.0]), seg_len=base.seg_len,
            cumulative=base.cumulative, speed_profile=base.speed_profile,
            curvature=-base.curvature, closed=base.closed)
        mirrored_inp = TrackingInput(x=px, y=-py, yaw=-yaw, speed=1.5,
                                     nearest_idx=idx, previous_steering=0.0)
        reflected = law.steer(mirrored_inp, mirrored)
        assert reflected.steering == pytest.approx(-original.steering, abs=1e-9)


def test_lqr_test12_param_validation():
    for key in ('lqr.q_lateral', 'lqr.q_heading', 'lqr.r_steer'):
        with pytest.raises(ValueError, match=key):
            make('lqr', **{key: 0.0})
    with pytest.raises(ValueError, match='lqr.v_grid_max'):
        make('lqr', **{'lqr.v_grid_max': 0.1})


def test_lqr_test13_clip_reports_unclipped():
    ctx = straight_context(speed=1.5)
    r = _lqr().steer(TrackingInput(x=0.0, y=5.0, yaw=0.0, speed=1.5,
                                   nearest_idx=0, previous_steering=0.0), ctx)
    assert r.steering == pytest.approx(-LIMITS.max_steering_angle, abs=1e-12)
    assert r.unclipped < -LIMITS.max_steering_angle


# ============================================================================
# MissPolicy (spec 04) -- pure, unsourced.
# ============================================================================

def test_miss_policy_single_miss_falls_back():
    policy = MissPolicy(3)
    assert policy.on_miss() == 'fallback'
    assert not policy.latched()


def test_miss_policy_latches_at_the_limit():
    policy = MissPolicy(3)
    assert policy.on_miss() == 'fallback'
    assert policy.on_miss() == 'fallback'
    assert policy.on_miss() == 'latched'
    assert policy.latched()


def test_miss_policy_success_resets_the_count():
    policy = MissPolicy(3)
    policy.on_miss()
    policy.on_miss()
    policy.on_success()
    assert policy.on_miss() == 'fallback'


def test_miss_policy_latch_survives_successes_until_cleared():
    policy = MissPolicy(1)
    assert policy.on_miss() == 'latched'
    policy.on_success()
    assert policy.latched()           # still latched
    policy.clear()
    assert not policy.latched()
    assert policy.on_miss() == 'latched'   # max_consecutive=1 latches immediately


def test_miss_policy_rejects_bad_limit():
    with pytest.raises(ValueError, match='max_consecutive_misses'):
        MissPolicy(0)


# ============================================================================
# MPC (spec 04) -- osqp-backed, skipped only until G0 lands osqp.
# ============================================================================


@pytest.fixture
def osqp_available():
    """Import osqp, skipping only the MPC tests until spec 04 G0 installs it.
    Scoped as a fixture (not module-level importorskip) so the rest of the
    module -- and the directory's collection -- is never skipped with it."""
    return pytest.importorskip('osqp', reason='spec 04 G0: osqp not installed')


def _mpc(**overrides):
    params = {'mpc.deadline_sec': 10.0, 'mpc.max_iter': 8000,
              'mpc.delay_compensation': 0.0}
    params.update(overrides)
    return make('mpc', **params)


def test_mpc_condensed_dynamics_match_step_by_step_simulation():
    # Oracle: roll the time-varying model forward one step at a time, with
    # speed and feedforward changing along the horizon (test 1's LQR
    # equivalence only covers constant speed, where every shear is the same).
    # No osqp needed: this is the model, not the solver.
    rng = np.random.default_rng(7)
    n, dt_p = 12, 0.05
    speeds = rng.uniform(0.5, 4.0, n)
    ff = rng.uniform(-0.3, 0.3, n)
    delta = rng.uniform(-0.35, 0.35, n)
    x0 = np.array([0.12, -0.05])
    Phi, Gamma = tracking_laws.MpcLaw._condensed_dynamics(speeds, ff, dt_p, L)
    stacked = Phi @ x0 + Gamma @ (delta - ff)
    x = x0.copy()
    for k in range(n):
        A = np.array([[1.0, speeds[k] * dt_p], [0.0, 1.0]])
        B = np.array([0.0, speeds[k] * dt_p / math.cos(ff[k]) ** 2 / L])
        x = A @ x + B * (delta[k] - ff[k])
        assert stacked[2 * k:2 * k + 2] == pytest.approx(x, abs=1e-12)


def test_mpc_test1_lqr_equivalence(osqp_available):
    # An unconstrained finite-horizon problem with the DARE terminal cost IS
    # LQR, for any N. Wide limits, no rate weight, dt_p = control_dt.
    ctx = straight_context(speed=1.5)
    wide = VehicleLimits(wheelbase=L, max_steering_angle=10.0,
                         max_steering_rate=1e6, control_dt=0.05)
    e0, th0 = 0.05, 0.02
    table = LqrGainTable(1.0, 1.0, 1.0, 0.05, L, 6.0)
    K = table.gain_at(1.5)
    expected = -(K[0] * e0 + K[1] * th0)
    for n in (5, 20):
        # Weights pinned to the table's (1, 1, 1): the shipped defaults are
        # tuned, and this is an identity, not a tuning check.
        law = tracking_laws.make_law('mpc', wide, {
            'mpc.dt': 0.05, 'mpc.horizon_steps': n, 'mpc.r_rate': 0.0,
            'mpc.q_lateral': 1.0, 'mpc.q_heading': 1.0, 'mpc.r_steer': 1.0,
            'mpc.delay_compensation': 0.0,
            'mpc.deadline_sec': 10.0, 'mpc.max_iter': 8000})
        r = law.steer(TrackingInput(x=1.0, y=e0, yaw=th0, speed=1.5,
                                    nearest_idx=5, previous_steering=0.0), ctx)
        assert r.steering == pytest.approx(expected, abs=1e-5)  # rad


def test_mpc_test2_angle_bound_active(osqp_available):
    ctx = straight_context(speed=1.5)
    law = _mpc(**{'mpc.r_rate': 0.0})
    # Large lateral error demands far more than max_steering_angle. With
    # previous_steering already at the bound, the rate limit is satisfied and
    # the angle bound is the active constraint (decoupling it from test 3).
    r = law.steer(TrackingInput(x=0.0, y=2.0, yaw=0.0, speed=1.5,
                                nearest_idx=0, previous_steering=-LIMITS.max_steering_angle), ctx)
    assert r.steering == pytest.approx(-LIMITS.max_steering_angle, abs=1e-5)


def test_mpc_test3_rate_bound_uses_control_dt_for_first_step(osqp_available):
    ctx = straight_context(speed=1.5)
    law = _mpc()
    r = law.steer(TrackingInput(x=0.0, y=1.0, yaw=0.0, speed=1.5,
                                nearest_idx=0, previous_steering=0.0), ctx)
    # |delta_0 - prev| <= max_steering_rate * control_dt = 1.0 * 0.025.
    assert r.steering == pytest.approx(-0.025, abs=1e-5)


def _loop_context(turn_sign, R=1.0, straight=3.0, spacing=0.05, speed=1.0):
    """A closed stadium loop (two straights + two same-sign caps). `turn_sign`
    = +1 turns left (CCW), -1 turns right (the loop is mirrored in y, which
    flips the signed curvature without changing the +x bottom-straight driving
    direction). Closed, so the MPC horizon wraps and sees the cap coming."""
    pts = []
    n = int(round(straight / spacing))
    for i in range(n):                                  # bottom straight, +x
        pts.append((i * spacing, 0.0))
    ncap = int(round(math.pi * R / spacing))
    for i in range(ncap):                               # end cap, `turn_sign`
        a = (i / ncap) * math.pi
        pts.append((straight + turn_sign * R * math.sin(a),
                    turn_sign * (R - R * math.cos(a))))
    for i in range(1, n):                               # top straight, -x
        pts.append((straight - i * spacing, turn_sign * 2 * R))
    for i in range(ncap):                               # start cap, `turn_sign`
        a = (i / ncap) * math.pi
        pts.append((0.0 - turn_sign * R * math.sin(a),
                    turn_sign * (R + R * math.cos(a))))
    xy = np.array(pts)
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    curvature = racing_math.estimate_signed_path_curvature(
        racing_math.smooth_path(xy, 2, True), closed=True)
    return PathContext(xy=xy, seg_len=seg_len,
                       cumulative=racing_math.compute_cumulative_arc_length(seg_len),
                       speed_profile=np.full(len(xy), speed),
                       curvature=curvature, closed=True)


def test_mpc_test4_preview_turns_the_right_way(osqp_available):
    # A car on the bottom straight of a stadium loop, close enough to the cap
    # that the bend is inside the MPC horizon, must already steer toward it:
    # positive for a left cap, negative for a right one. That preview is the
    # whole point of MPC; a no-preview or wrong-sign law cannot produce it.
    results = {}
    for sign in (+1.0, -1.0):
        ctx = _loop_context(sign)
        # Bottom straight runs (0,0)->(straight,0); cap starts at x=straight.
        px, py = 2.6, 0.0
        nearest, _ = racing_math.find_nearest_index(ctx.xy, (px, py), closed=True)
        law = _mpc(**{'mpc.q_lateral': 4.0, 'mpc.q_heading': 4.0})
        results[sign] = law.steer(TrackingInput(
            x=px, y=py, yaw=0.0, speed=1.0, nearest_idx=nearest,
            previous_steering=0.0), ctx).steering
    assert results[+1.0] > 0.0      # left bend ahead -> steer left
    assert results[-1.0] < 0.0      # right bend ahead -> steer right


def test_mpc_test5_steady_circle_is_feedforward(osqp_available):
    ctx = circle_context(radius=2.0, spacing=0.02, ccw=True, speed=1.5)
    law = _mpc()
    nearest, _ = racing_math.find_nearest_index(ctx.xy, (2.0, 0.0), closed=True)
    proj = racing_math.project_onto_path(ctx.xy, ctx.seg_len, (2.0, 0.0), nearest, True)
    kappa = racing_math.interpolate_along(ctx.curvature, proj, True)
    prev = math.atan(L * kappa)
    r = law.steer(TrackingInput(x=2.0, y=0.0, yaw=math.pi / 2.0, speed=1.5,
                                nearest_idx=nearest, previous_steering=prev), ctx)
    assert r.steering == pytest.approx(math.atan(L * kappa), abs=2e-3)
    assert math.atan(L * kappa) == pytest.approx(math.atan(L / 2.0), abs=1e-3)


def test_mpc_test6_linearisation_matches_finite_difference(osqp_available):
    # The model's d(theta)/d(delta) = v*dt_p*sec^2(ff)/L must match a finite
    # difference of the true nonlinear update theta + v dt_p (tan d / L - k).
    v, dt_p, ff = 1.5, 0.05, 0.2
    Lw = L
    kappa = math.tan(ff) / Lw

    def nonlinear(delta):
        return v * dt_p * (math.tan(delta) / Lw - kappa)

    step = 1e-7
    fd = (nonlinear(ff + step) - nonlinear(ff - step)) / (2.0 * step)
    model = v * dt_p / (math.cos(ff) ** 2) / Lw
    assert model == pytest.approx(fd, rel=1e-5)


def test_mpc_test7_mirror_invariant(osqp_available):
    rng = np.random.default_rng(2)
    base = circle_context(radius=3.0, spacing=0.1, ccw=True, speed=1.5)
    law = _mpc()
    for _ in range(6):
        idx = int(rng.integers(0, len(base.xy)))
        px, py = base.xy[idx] + rng.uniform(-0.3, 0.3, size=2)
        yaw = float(rng.uniform(-0.3, 0.3))
        inp = TrackingInput(x=px, y=py, yaw=yaw, speed=1.5,
                            nearest_idx=idx, previous_steering=0.0)
        original = law.steer(inp, base)
        mirrored = PathContext(
            xy=base.xy * np.array([1.0, -1.0]), seg_len=base.seg_len,
            cumulative=base.cumulative, speed_profile=base.speed_profile,
            curvature=-base.curvature, closed=base.closed)
        law.reset()
        reflected = law.steer(TrackingInput(x=px, y=-py, yaw=-yaw, speed=1.5,
                                            nearest_idx=idx, previous_steering=0.0),
                              mirrored)
        assert reflected.steering == pytest.approx(-original.steering, abs=1e-4)


def test_mpc_test8_deadline_and_status_misses(osqp_available):
    ctx = straight_context(speed=1.5)
    # An impossible deadline: the wall-clock check fires (elapsed > 1e-9 s is
    # always true). Rebuild the solver without OSQP's own time_limit so the
    # *wall-clock deadline*, not the solver's internal time-limit status, is
    # what is under test.
    tight = _mpc(**{'mpc.deadline_sec': 1e-9, 'mpc.max_iter': 8000})
    n = tight.params.horizon_steps
    tight._solver = tight._osqp.OSQP()
    tight._solver.setup(P=tight._P_pattern, q=np.zeros(n), A=tight._A_pattern,
                        l=np.zeros(2 * n), u=np.zeros(2 * n), eps_abs=1e-5,
                        eps_rel=1e-5, max_iter=8000, polishing=False,
                        warm_starting=True, verbose=False)
    with pytest.raises(tracking_laws.MpcMiss, match='deadline'):
        tight.steer(TrackingInput(x=0.0, y=0.3, yaw=0.1, speed=1.5,
                                  nearest_idx=0, previous_steering=0.0), ctx)
    # max_iter=1 on a non-trivial state cannot converge to 'solved'.
    one_iter = _mpc(**{'mpc.max_iter': 1})
    with pytest.raises(tracking_laws.MpcMiss, match='status'):
        one_iter.steer(TrackingInput(x=0.0, y=0.3, yaw=0.1, speed=1.5,
                                     nearest_idx=0, previous_steering=0.0), ctx)


def test_mpc_test8b_a_miss_drops_the_warm_start(osqp_available):
    ctx = straight_context(speed=1.5)
    law = _mpc()
    inp = TrackingInput(x=0.0, y=0.05, yaw=0.0, speed=1.5,
                        nearest_idx=0, previous_steering=0.0)
    law.steer(inp, ctx)
    assert law._warm_start is not None      # a success keeps the warm start
    tight = _mpc(**{'mpc.deadline_sec': 1e-9})
    try:
        tight.steer(inp, ctx)
    except tracking_laws.MpcMiss:
        pass
    assert tight._warm_start is None        # a miss drops it


class _FixedSolution:
    """Stands in for the OSQP solver object (a third-party boundary) and
    returns one fixed, 'solved' decision vector."""

    def __init__(self, x):
        self.x = np.asarray(x, dtype=float)

    def update(self, **_kwargs):
        pass

    def warm_start(self, **_kwargs):
        pass

    def solve(self):
        from types import SimpleNamespace
        return SimpleNamespace(x=self.x, info=SimpleNamespace(status='solved', iter=1))


@pytest.mark.parametrize('overshoot, is_miss', [
    (0.0, False),
    (0.5 * tracking_laws.BOUND_TOLERANCE, False),   # within OSQP's own accuracy
    (2.0 * tracking_laws.BOUND_TOLERANCE, True),
    (0.05, True),
])
def test_mpc_bound_check_tolerates_solver_accuracy_but_not_real_violations(
        osqp_available, overshoot, is_miss):
    # OSQP meets constraints to ~1e-5 (eps_abs/eps_rel, no polishing). A
    # solution that far past the angle bound is the solver's accuracy, not a
    # bad solve, and must be clipped and used; one clearly past it is a miss.
    ctx = straight_context(speed=1.5)
    law = _mpc()
    n = law.params.horizon_steps
    law._solver = _FixedSolution([LIMITS.max_steering_angle + overshoot] * n)
    # Already at the bound, so the plan is rate-feasible and the angle bound
    # is the only one in play.
    inp = TrackingInput(x=0.0, y=0.0, yaw=0.0, speed=1.5, nearest_idx=0,
                        previous_steering=LIMITS.max_steering_angle)
    if is_miss:
        with pytest.raises(tracking_laws.MpcMiss, match='solution: violates'):
            law.steer(inp, ctx)
    else:
        assert law.steer(inp, ctx).steering == LIMITS.max_steering_angle


def test_mpc_test9_non_finite_input_is_not_a_miss(osqp_available):
    ctx = straight_context(speed=1.5)
    law = _mpc()
    with pytest.raises(tracking_laws.TrackingLawError, match='non-finite') as exc:
        law.steer(TrackingInput(x=math.nan, y=0.0, yaw=0.0, speed=1.5,
                                nearest_idx=0, previous_steering=0.0), ctx)
    assert not isinstance(exc.value, tracking_laws.MpcMiss)


def test_mpc_test10_param_validation(osqp_available):
    bad = {'mpc.dt': 0.0, 'mpc.horizon_steps': 1, 'mpc.horizon_steps': 61,
           'mpc.q_lateral': 0.0, 'mpc.r_steer': -1.0, 'mpc.r_rate': -0.1,
           'mpc.deadline_sec': 0.0, 'mpc.max_iter': 0}
    for key, value in bad.items():
        with pytest.raises(ValueError, match=key.split('.')[-1] if 'horizon' not in key else 'horizon_steps'):
            _mpc(**{key: value})


def test_mpc_test11_predicted_sequence_is_well_formed(osqp_available):
    ctx = circle_context(radius=2.0, spacing=0.02, ccw=True, speed=1.5)
    law = _mpc()
    nearest, _ = racing_math.find_nearest_index(ctx.xy, (2.0, 0.0), closed=True)
    r = law.steer(TrackingInput(x=2.0, y=0.0, yaw=math.pi / 2.0, speed=1.5,
                                nearest_idx=nearest, previous_steering=0.0), ctx)
    assert len(r.predicted) == law.params.horizon_steps
    assert all(math.isfinite(d) for d in r.predicted)
    assert all(abs(d) <= LIMITS.max_steering_angle + 1e-6 for d in r.predicted)
    # Consecutive differences respect the rate bound (tau_0 = control_dt for
    # the first step measured against previous_steering, dt_p after).
    prev = 0.0
    taus = [LIMITS.control_dt] + [law.params.dt] * (len(r.predicted) - 1)
    for delta, tau in zip(r.predicted, taus):
        assert abs(delta - prev) <= LIMITS.max_steering_rate * tau + 1e-6
        prev = delta


def test_mpc_test12_open_path_past_the_end_clamps(osqp_available):
    xy = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    seg_len = racing_math.compute_segment_lengths(xy, closed=False)
    ctx = PathContext(xy=xy, seg_len=seg_len,
                      cumulative=racing_math.compute_cumulative_arc_length(seg_len),
                      speed_profile=np.ones(4), curvature=np.zeros(4), closed=False)
    law = _mpc()
    # Near the end, the horizon runs off the path; the arc-length sampling
    # must clamp rather than crash, and still return a finite command.
    r = law.steer(TrackingInput(x=2.8, y=0.05, yaw=0.0, speed=1.5,
                                nearest_idx=3, previous_steering=0.0), ctx)
    assert math.isfinite(r.steering)


def test_mpc_test13_reset_zeroes_the_counter_and_drops_warm_start(osqp_available):
    ctx = straight_context(speed=1.5)
    law = _mpc()
    law.steer(TrackingInput(x=0.0, y=0.05, yaw=0.0, speed=1.5,
                            nearest_idx=0, previous_steering=0.0), ctx)
    assert law.solves_since_reset >= 1
    assert law._warm_start is not None
    law.reset()
    assert law.solves_since_reset == 0
    assert law._warm_start is None


def test_osqp_importable_when_declared():
    """Once package.xml lists osqp (G0 lands), tracking_laws must import it.
    This guards the importorskip above becoming a real dependency."""
    import xml.etree.ElementTree as ET
    tree = ET.parse(os.path.join(PACKAGE_DIR, 'package.xml'))
    deps = {e.text for e in tree.getroot().iter('depend')}
    deps |= {e.text for e in tree.getroot().iter('exec_depend')}
    declared = any('osqp' in d for d in deps)
    if declared:
        import importlib
        importlib.import_module('osqp')


# ============================================================================
# Pure-pursuit refactor regression (spec 00 Unit 2).
# Change-detector (not an oracle): this pins PurePursuitLaw's outputs on a
# real profiled line to fixed values recorded from the pre-refactor
# computation (the node's own formula chain, unmodified). If any of these
# move, the refactor changed pure pursuit -- fix the refactor, not this test.
# ============================================================================

def test_pure_pursuit_matches_the_pre_refactor_node_computation():
    src = os.path.join(PACKAGE_DIR, 'waypoints', 'example_stadium_raw.csv')
    xy = racing_math.load_xy_csv(src)
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    speed = racing_math.compute_velocity_profile(
        seg_len, racing_math.estimate_path_curvature(xy, closed=True),
        v_max=6.0, v_min=0.5, a_lat_max=8.0, a_accel_max=3.0, a_brake_max=8.0,
        closed=True)
    ctx = PathContext(xy=xy, seg_len=seg_len,
                      cumulative=racing_math.compute_cumulative_arc_length(seg_len),
                      speed_profile=speed, curvature=np.zeros(len(xy)), closed=True)
    law = make('pure_pursuit')
    poses = [
        (-1.5, -1.2, 0.0, 1.0), (-1.5, -1.3, 0.0, 2.0), (-1.5, -1.4, 0.1, 3.0),
        (0.0, -1.2, 0.0, 1.0), (1.0, -1.25, -0.05, 1.5), (2.0, -1.2, 0.2, 2.5),
        (3.0, -1.3, 0.0, 0.5), (3.1, -1.2, 1.0, 4.0), (0.5, -1.15, 0.3, 1.0),
        (2.5, -1.1, -0.3, 2.0), (-2.0, -1.5, -0.2, 1.2), (1.5, -1.35, 0.6, 3.3),
        (3.05, -1.25, 1.55, 2.0), (0.0, -1.2, -1.0, 1.0), (-1.0, -1.2, 1.0, 0.8),
        (-2.5, -1.3, 0.45, 2.2), (-3.0, -1.15, -0.9, 1.1), (1.2, -1.28, 0.12, 3.6),
        (2.9, -1.32, -0.75, 1.9), (-1.8, -1.22, 0.85, 2.7),
    ]
    expected_steering = [
        0.000000, 0.073837, 0.047142, 0.000000, 0.091215, 0.165836,
        0.260000, 0.243821, -0.260000, 0.260000, 0.260000, -0.164283,
        -0.063168, 0.260000, -0.260000, -0.226443, 0.260000, 0.006642,
        0.260000, -0.260000,
    ]
    for (px, py, yaw, v), expected in zip(poses, expected_steering):
        nearest, _ = racing_math.find_nearest_index(xy, (px, py), closed=True)
        result = law.steer(TrackingInput(x=px, y=py, yaw=yaw, speed=v,
                                         nearest_idx=nearest,
                                         previous_steering=0.0), ctx)
        assert result.steering == pytest.approx(expected, abs=2e-5), \
            (px, py, yaw, v, result.steering, expected)


# ============================================================================
# Delay compensation, output smoothing, and MPC's exact-limit projection
# (docs/steering-laws.md#delay-compensation-and-smoothing). Expected values
# are the kinematic bicycle and the first-order filter in closed form.
# ============================================================================

def test_predict_input_without_delay_or_motion_is_the_input():
    ctx = straight_context()
    inp = TrackingInput(x=1.0, y=0.1, yaw=0.2, speed=2.0, nearest_idx=5,
                        previous_steering=0.1)
    assert tracking_laws.predict_input(inp, ctx, 0.0, L) == inp
    still = TrackingInput(x=1.0, y=0.1, yaw=0.2, speed=0.0, nearest_idx=5,
                          previous_steering=0.1)
    assert tracking_laws.predict_input(still, ctx, 0.1, L) == still


def test_predict_input_straight_ahead_and_nearest_index_follows():
    # Wheels straight: the car moves v * delay along its heading. On a 1 cm
    # path the nearest waypoint must be the global nearest to the new pose.
    ctx = straight_context(n=1001, length=10.0)
    inp = TrackingInput(x=1.0, y=0.1, yaw=0.0, speed=2.0, nearest_idx=100,
                        previous_steering=0.0)
    out = tracking_laws.predict_input(inp, ctx, 0.1, L)
    assert out.x == pytest.approx(1.2, abs=1e-12)   # m
    assert out.y == pytest.approx(0.1, abs=1e-12)
    assert out.yaw == pytest.approx(0.0, abs=1e-12)
    assert (out.speed, out.previous_steering) == (2.0, 0.0)
    expected_idx, _ = racing_math.find_nearest_index(ctx.xy, (1.2, 0.1), closed=False)
    assert out.nearest_idx == expected_idx == 120


@pytest.mark.parametrize('delta', [0.2, -0.2])
def test_predict_input_follows_the_bicycle_arc(delta):
    # Oracle: rear axle on a circle about the instantaneous centre
    # c = p + (1/kappa) * (-sin yaw, cos yaw), kappa = tan(delta)/L, swept
    # through angle kappa * s for arc length s = v * delay.
    ctx = straight_context(n=1001, length=10.0)
    yaw0, v, delay = 0.3, 3.0, 0.1
    inp = TrackingInput(x=2.0, y=0.0, yaw=yaw0, speed=v, nearest_idx=200,
                        previous_steering=delta)
    out = tracking_laws.predict_input(inp, ctx, delay, L)
    kappa = math.tan(delta) / L
    cx, cy = 2.0 - math.sin(yaw0) / kappa, 0.0 + math.cos(yaw0) / kappa
    yaw1 = yaw0 + kappa * v * delay
    assert out.x == pytest.approx(cx + math.sin(yaw1) / kappa, abs=1e-9)  # m
    assert out.y == pytest.approx(cy - math.cos(yaw1) / kappa, abs=1e-9)
    assert out.yaw == pytest.approx(yaw1, abs=1e-12)  # rad
    expected_idx, _ = racing_math.find_nearest_index(ctx.xy, (out.x, out.y), closed=False)
    assert out.nearest_idx == expected_idx


def test_smooth_steering_is_a_first_order_filter():
    assert tracking_laws.smooth_steering(0.2, -0.1, 0.0, 0.025) == 0.2
    # tau = dt: halfway.
    assert tracking_laws.smooth_steering(0.2, -0.1, 0.025, 0.025) == pytest.approx(
        0.05, abs=1e-12)
    # tau = 2 dt: a third of the way.
    assert tracking_laws.smooth_steering(0.2, -0.1, 0.05, 0.025) == pytest.approx(
        -0.1 + 0.3 / 3.0, abs=1e-12)


def _smoothed_stanley(tau):
    return make('stanley', **{'stanley.gain': GAIN, 'stanley.softening': SOFTENING,
                              'stanley.delay_compensation': 0.0,
                              'stanley.steering_time_constant': tau})


def test_stanley_smoothing_moves_from_the_last_command():
    ctx = straight_context(speed=1.0)
    alpha = LIMITS.control_dt / (0.05 + LIMITS.control_dt)
    r = _smoothed_stanley(0.05).steer(TrackingInput(
        x=0.0, y=0.2, yaw=0.0, speed=1.0, nearest_idx=0, previous_steering=0.1), ctx)
    bare = -math.atan(GAIN * 0.2 / (1.0 + SOFTENING))
    assert r.steering == pytest.approx(0.1 + alpha * (bare - 0.1), abs=1e-12)  # rad
    assert r.unclipped == pytest.approx(bare, abs=1e-12)


def test_stanley_smoothing_filters_the_clipped_command():
    # At standstill the bare law asks for -0.675 rad; the filter must move
    # toward the clip (-0.26), not toward the request.
    ctx = straight_context(speed=1.0)
    alpha = LIMITS.control_dt / (0.05 + LIMITS.control_dt)
    r = _smoothed_stanley(0.05).steer(TrackingInput(
        x=0.0, y=0.2, yaw=0.0, speed=0.0, nearest_idx=0, previous_steering=0.1), ctx)
    limit = LIMITS.max_steering_angle
    assert r.steering == pytest.approx(0.1 + alpha * (-limit - 0.1), abs=1e-12)


def test_lqr_smoothing_moves_from_the_last_command():
    ctx = straight_context(speed=2.0)
    inp = TrackingInput(x=1.0, y=0.1, yaw=0.05, speed=2.0, nearest_idx=5,
                        previous_steering=0.05)
    bare = _lqr().steer(inp, ctx).steering
    smoothed = make('lqr', **{'lqr.delay_compensation': 0.0,
                              'lqr.steering_time_constant': 0.05}).steer(inp, ctx)
    alpha = LIMITS.control_dt / (0.05 + LIMITS.control_dt)
    assert smoothed.steering == pytest.approx(0.05 + alpha * (bare - 0.05), abs=1e-12)


def test_stanley_delay_compensation_steers_from_the_predicted_pose():
    # Straight line, wheels straight, heading 0.1 rad off it at 2 m/s: in
    # 0.1 s the rear axle moves 0.2 m along its heading, and the bare law is
    # evaluated there (front axle L further on).
    ctx = straight_context(n=1001, length=10.0)
    yaw, v, delay = 0.1, 2.0, 0.1
    law = make('stanley', **{'stanley.gain': GAIN, 'stanley.softening': SOFTENING,
                             'stanley.delay_compensation': delay,
                             'stanley.steering_time_constant': 0.0})
    r = law.steer(TrackingInput(x=1.0, y=0.0, yaw=yaw, speed=v, nearest_idx=100,
                                previous_steering=0.0), ctx)
    e_f = (v * delay + L) * math.sin(yaw)
    expected = -yaw - math.atan(GAIN * e_f / (v + SOFTENING))
    assert r.steering == pytest.approx(expected, abs=1e-9)  # rad


def _predicted_by_hand(x, y, yaw, v, delta, delay, ctx):
    """The bicycle arc in centre form (independent of predict_input's)."""
    s = v * delay
    kappa = math.tan(delta) / L
    if abs(kappa) < 1e-12:
        x1, y1, yaw1 = x + s * math.cos(yaw), y + s * math.sin(yaw), yaw
    else:
        cx, cy = x - math.sin(yaw) / kappa, y + math.cos(yaw) / kappa
        yaw1 = yaw + kappa * s
        x1, y1 = cx + math.sin(yaw1) / kappa, cy - math.cos(yaw1) / kappa
    idx, _ = racing_math.find_nearest_index(ctx.xy, (x1, y1), closed=ctx.closed)
    return TrackingInput(x=x1, y=y1, yaw=yaw1, speed=v, nearest_idx=int(idx),
                         previous_steering=delta)


def test_lqr_delay_compensation_steers_from_the_predicted_pose():
    ctx = circle_context(radius=3.0, spacing=0.01, ccw=True, speed=2.0)
    # Heading 0.2 rad outward and the rack turned the wrong way: the car
    # is about to move well off where it is now.
    x, y, yaw, v, delta, delay = 3.05, 0.0, math.pi / 2.0 + 0.2, 2.0, -0.1, 0.08
    nearest, _ = racing_math.find_nearest_index(ctx.xy, (x, y), closed=True)
    law = make('lqr', **{'lqr.delay_compensation': delay,
                         'lqr.steering_time_constant': 0.0})
    r = law.steer(TrackingInput(x=x, y=y, yaw=yaw, speed=v, nearest_idx=nearest,
                                previous_steering=delta), ctx)
    expected = _lqr().steer(_predicted_by_hand(x, y, yaw, v, delta, delay, ctx), ctx)
    assert r.steering == pytest.approx(expected.steering, abs=1e-9)  # rad
    # And it is not the undelayed answer, or this proves nothing.
    undelayed = _lqr().steer(TrackingInput(x=x, y=y, yaw=yaw, speed=v,
                                           nearest_idx=nearest, previous_steering=delta), ctx)
    assert abs(r.steering - undelayed.steering) > 0.01


def test_mpc_delay_compensation_steers_from_the_predicted_pose(osqp_available):
    ctx = circle_context(radius=3.0, spacing=0.01, ccw=True, speed=2.0)
    x, y, yaw, v, delta, delay = 3.05, 0.0, math.pi / 2.0 + 0.2, 2.0, -0.1, 0.05
    nearest, _ = racing_math.find_nearest_index(ctx.xy, (x, y), closed=True)
    r = _mpc(**{'mpc.delay_compensation': delay}).steer(TrackingInput(
        x=x, y=y, yaw=yaw, speed=v, nearest_idx=nearest, previous_steering=delta), ctx)
    expected = _mpc().steer(_predicted_by_hand(x, y, yaw, v, delta, delay, ctx), ctx)
    # 1e-6 rad: two separate OSQP solves of the same QP.
    assert r.steering == pytest.approx(expected.steering, abs=1e-6)
    undelayed = _mpc().steer(TrackingInput(x=x, y=y, yaw=yaw, speed=v,
                                           nearest_idx=nearest, previous_steering=delta), ctx)
    assert abs(r.steering - undelayed.steering) > 1e-3


@pytest.mark.parametrize('dotted', [
    'stanley.delay_compensation', 'stanley.steering_time_constant',
    'lqr.delay_compensation', 'lqr.steering_time_constant'])
def test_delay_and_smoothing_params_are_bounded(dotted):
    law = dotted.split('.')[0]
    for value in (0.0, 0.5):   # both ends are allowed
        make(law, **{dotted: value})
    for value in (-0.01, 0.51, math.nan, math.inf):
        with pytest.raises(ValueError, match=dotted):
            make(law, **{dotted: value})


def test_mpc_delay_param_is_bounded(osqp_available):
    _mpc(**{'mpc.delay_compensation': 0.5})
    for value in (-0.01, 0.51, math.nan):
        with pytest.raises(ValueError, match='mpc.delay_compensation'):
            _mpc(**{'mpc.delay_compensation': value})


def test_mpc_projection_onto_exact_limits():
    project = tracking_laws.MpcLaw._project_onto_limits
    # Rate: 1 rad/s * 0.025 s first, then 0.05 s a move.
    out = project([0.3, -0.3, 0.0], 0.0, 0.26, 1.0, [0.025, 0.05, 0.05])
    assert out == pytest.approx([0.025, -0.025, 0.0], abs=1e-12)
    # Angle: within the rate step of 0.25 but past 0.26.
    assert project([0.4], 0.25, 0.26, 1.0, [0.025]) == pytest.approx([0.26], abs=1e-12)
    # A feasible plan passes through untouched.
    plan = [0.01, 0.03, 0.05]
    assert project(plan, 0.0, 0.26, 1.0, [0.025, 0.05, 0.05]) == plan
    # A previous command past the bound is clamped before measuring from it:
    # from 0.26 (not 0.4) the first move may drop to 0.235.
    assert project([-0.3], 0.4, 0.26, 1.0, [0.025]) == pytest.approx([0.235], abs=1e-12)
