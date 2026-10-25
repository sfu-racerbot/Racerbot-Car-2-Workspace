"""
imu_bridge.vehicle_state: fused speed and measured steering angle.

No ROS needed -- vehicle_state.py imports nothing from rclpy:

    python3 -m pytest src/imu_bridge/test/ -v

Oracles: the kinematic bicycle on a circle (tan(delta) = L / R, yaw rate =
v / R), rigid-body acceleration recomputed here by finite differences of the
geometry, the standard Kalman filter equations written out by hand, and
invariants (the published speed never leaves max_divergence of the wheel
speed; with no accelerometer it IS the wheel speed). Scenarios are synthetic:
truth is known by construction and the sensors are generated from it.
Tolerances are absolute, with units.
"""
import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from imu_bridge import vehicle_state as vs  # noqa: E402
from imu_bridge.vehicle_state import (  # noqa: E402
    MIN_GRAVITY_Z,
    MODE_CLAMPED,
    MODE_FUSED,
    MODE_INVALID,
    MODE_SLIP,
    MODE_WHEEL_ONLY,
    EstimatorConfig,
    SpeedFilter,
    VehicleStateEstimator,
    accel_at_base_link,
    bounded,
    check_frames,
    check_output_topics,
    steering_from_yaw_rate,
)

G = 9.80665
L = 0.36          # car 1 wheelbase [m], vesc.yaml
D_BNO = 0.36      # BNO on the front axle = one wheelbase ahead of base_link [m]
SPEED_TOL = 1e-9  # m/s
ANGLE_TOL = 1e-9  # rad
ACC_TOL = 1e-9    # m/s^2

BNO_DT = 0.004    # 250 Hz
SLOW_EVERY = 5    # VESC IMU and wheel speed every 5th BNO tick = 50 Hz


def drive(est, t_end, speed, yaw, *, t0=0.0, wheel=None, vesc_yaw=None,
          bno_ax_extra=None, bno_z=G, vesc_z=G, bno_on=None, vesc_on=None):
    """Run a synthetic scenario through `est`; return [(t, truth, estimate)].

    speed(t), yaw(t): base_link's true forward speed [m/s] and yaw rate
    [rad/s]. The BNO (250 Hz, D_BNO ahead) reads the true acceleration at its
    own position: dv/dt - yaw^2 D_BNO along x (see accel_at_base_link). The
    VESC IMU and the wheels come every 5th tick (50 Hz). Receipt time and
    stamp are the same clock here.
    """
    out = []
    h = 1e-6
    n = int(round((t_end - t0) / BNO_DT))
    for k in range(n + 1):
        t = t0 + k * BNO_DT
        r = yaw(t)
        if bno_on is None or bno_on(t):
            dv = (speed(t + h) - speed(t - h)) / (2 * h)
            ax = dv - r * r * D_BNO + (bno_ax_extra(t) if bno_ax_extra else 0.0)
            est.on_imu('bno', t, t, ax, bno_z, r)
        if k % SLOW_EVERY == 0:
            if vesc_on is None or vesc_on(t):
                est.on_imu('vesc', t, t, 0.0, vesc_z, vesc_yaw(t) if vesc_yaw else r)
            z = wheel(t) if wheel else speed(t)
            out.append((t, speed(t), est.on_wheel(t, z)))
    return out


def const(value):
    return lambda t: value


# ---------------------------------------------------------------- config

POSITIVE_FIELDS = ('wheelbase', 'sigma_accel', 'sigma_bias_rw', 'sigma_wheel',
                   'sigma_bias0', 'max_divergence', 'reset_divergence',
                   'slip_gate', 'max_slip_time', 'imu_timeout', 'wheel_timeout', 'max_accel_dt',
                   'min_steering_speed', 'gyro_disagree_threshold',
                   'gyro_disagree_tau')


@pytest.mark.parametrize('field', POSITIVE_FIELDS)
@pytest.mark.parametrize('bad', [0.0, -1.0, math.nan, math.inf])
def test_config_rejects_non_positive_or_non_finite(field, bad):
    with pytest.raises(ValueError, match=field):
        EstimatorConfig(**{field: bad})


def test_config_needs_reset_beyond_the_clamp():
    # Equal would mean a reset fires before the clamp can ever act.
    with pytest.raises(ValueError, match='reset_divergence'):
        EstimatorConfig(max_divergence=0.5, reset_divergence=0.5)
    EstimatorConfig(max_divergence=0.5, reset_divergence=0.5 + 1e-9)


def test_config_rejects_unknown_accel_source():
    with pytest.raises(ValueError, match='accel_source'):
        EstimatorConfig(accel_source='realsense')


@pytest.mark.parametrize('weights,match', [
    ((('bno', 0.0), ('vesc', 0.0)), 'at least one'),
    ((('bno', -1.0), ('vesc', 1.0)), 'at least one'),
    ((('bno', 1.0),), 'exactly bno and vesc'),
    ((('bno', 1.0), ('vesc', math.nan)), 'finite'),
])
def test_config_rejects_bad_gyro_weights(weights, match):
    with pytest.raises(ValueError, match=match):
        EstimatorConfig(gyro_weight=weights)


# ---------------------------------------------------------------- output topics

def test_shipped_output_topics_pass():
    check_output_topics('/odom', '/odom/fused', '/steering/measured', '/odom/imu')


@pytest.mark.parametrize('fused,steering,imu,match', [
    ('/odom', '/steering/measured', '/odom/imu', 'fused_odom_topic must differ from odom_topic'),
    ('/drive', '/steering/measured', '/odom/imu', 'drive command'),
    ('/odom/fused', '/auto_map/drive', '/odom/imu', 'drive command'),
    ('/odom/fused', '/teleop', '/odom/imu', 'drive command'),
    ('/ackermann_cmd', '/steering/measured', '/odom/imu', 'drive command'),
    ('/commands/motor/speed', '/steering/measured', '/odom/imu', 'drive command'),
    ('/odom/fused', '/steering/measured', '/drive', 'drive command'),
    ('/odom/fused', '/odom/fused', '/odom/imu', 'must all differ'),
    ('/odom/fused', '/steering/measured', '/odom/fused', 'must all differ'),
    ('/odom/fused', '/odom', '/odom/imu', 'steering_topic must differ from odom_topic'),
    ('/odom/fused', '/steering/measured', '/odom', 'imu_odom_topic must differ from odom_topic'),
    ('odom/fused', '/steering/measured', '/odom/imu', 'absolute'),
    ('/', '/steering/measured', '/odom/imu', 'absolute'),
    ('', '/steering/measured', '/odom/imu', 'absolute'),
    ('/odom/fused', '/steering/measured', 'odom_imu', 'absolute'),
])
def test_output_topics_that_could_reach_the_drive_path_are_refused(fused, steering, imu, match):
    # oracle: spec -- CLAUDE.md safety model: only nodes holding the LB
    # deadman may publish /drive, /ackermann_cmd or /commands/*.
    with pytest.raises(ValueError, match=match):
        check_output_topics('/odom', fused, steering, imu)


def test_shipped_frames_pass():
    check_frames('odom_imu', 'odom', 'base_link')


@pytest.mark.parametrize('imu,odom,base,match', [
    ('odom', 'odom', 'base_link', 'must be a new frame'),
    ('base_link', 'odom', 'base_link', 'must be a new frame'),
    ('map', 'odom', 'base_link', 'must be a new frame'),
    ('', 'odom', 'base_link', 'plain frame name'),
    ('/odom_imu', 'odom', 'base_link', 'plain frame name'),
    ('odom imu', 'odom', 'base_link', 'plain frame name'),
    ('odom_imu', 'odom', 'odom', 'must differ'),
])
def test_frames_that_would_break_the_tree_are_refused(imu, odom, base, match):
    # oracle: spec -- tf2: one parent per frame, no loops (check_frames docstring).
    with pytest.raises(ValueError, match=match):
        check_frames(imu, odom, base)


# ---------------------------------------------------------------- lever arm

def _sensor_accel_x(speed, t, d, R):
    """x acceleration, in body axes, of a point d ahead of base_link while
    base_link drives a circle of radius R at speed(t). Computed from the
    point's world position by central differences -- no rigid-body formula."""
    def heading(t):
        # theta(t) = s(t) / R; s by Simpson's rule on speed.
        n = 200
        hs = t / n
        s = speed(0.0) + speed(t) + sum(
            (4 if i % 2 else 2) * speed(i * hs) for i in range(1, n))
        return s * hs / 3.0 / R

    def point(t):
        th = heading(t)
        # base_link on the circle centred at (0, R), heading th.
        bx, by = R * math.sin(th), R - R * math.cos(th)
        return bx + d * math.cos(th), by + d * math.sin(th)

    h = 1e-3
    (x0, y0), (x1, y1), (x2, y2) = point(t - h), point(t), point(t + h)
    ax, ay = (x0 - 2 * x1 + x2) / h ** 2, (y0 - 2 * y1 + y2) / h ** 2
    th = heading(t)
    return ax * math.cos(th) + ay * math.sin(th)


@pytest.mark.parametrize('R', [1.5, -2.0, 4.0])
def test_lever_arm_recovers_base_link_accel_on_a_circle(R):
    # oracle: invariant -- the geometry, differentiated numerically, against
    # base_link's own tangential acceleration dv/dt.
    for accel, v0 in ((0.0, 1.5), (0.8, 0.5)):
        def speed(t):
            return v0 + accel * t
        t = 1.2
        a_sensor = _sensor_accel_x(speed, t, D_BNO, R)
        yaw = speed(t) / R
        # 1e-4 m/s^2: finite-difference error at h = 1 ms on these curves.
        assert accel_at_base_link(a_sensor, yaw, D_BNO) == pytest.approx(
            accel, abs=1e-4)  # m/s^2
        # Uncorrected it is off by the centripetal term, by over 100x the
        # tolerance above -- not a case a no-op would pass.
        assert abs(a_sensor - accel) > 100 * 1e-4


def test_lever_arm_is_zero_at_base_link_and_even_in_yaw():
    # oracle: closed form -- yaw^2 d vanishes at d = 0 and ignores yaw's sign.
    assert accel_at_base_link(0.7, 2.0, 0.0) == 0.7
    assert accel_at_base_link(0.0, -1.0, D_BNO) == pytest.approx(
        accel_at_base_link(0.0, 1.0, D_BNO), abs=ACC_TOL)  # m/s^2
    assert accel_at_base_link(0.0, -1.0, D_BNO) == pytest.approx(D_BNO, abs=ACC_TOL)


# ---------------------------------------------------------------- steering

@pytest.mark.parametrize('R', [1.05, 1.5, 3.0, -1.2, -5.0])
@pytest.mark.parametrize('v', [0.5, 2.0, -0.8])
def test_steering_matches_the_bicycle_on_a_circle(R, v):
    # oracle: closed form -- rear-axle circle of radius R: tan(delta) = L / R,
    # yaw rate = v / R. Reverse flips v and yaw rate together, so the angle
    # (which way the wheels point) is the same forwards and backwards.
    expected = math.atan(L / R)
    assert steering_from_yaw_rate(v / R, v, L, 0.3) == pytest.approx(
        expected, abs=ANGLE_TOL)  # rad


def test_steering_is_zero_driving_straight():
    # oracle: closed form -- no yaw, no curvature.
    assert steering_from_yaw_rate(0.0, 1.0, L, 0.3) == 0.0


@pytest.mark.parametrize('speed,valid', [
    (0.3, True), (-0.3, True),            # exactly at min_speed
    (0.3 - 1e-9, False), (-0.3 + 1e-9, False),   # one step inside
    (0.0, False),
])
def test_steering_min_speed_boundary(speed, valid):
    # oracle: spec -- NaN strictly below min_steering_speed (module docstring).
    angle = steering_from_yaw_rate(0.1, speed, L, 0.3)
    assert math.isnan(angle) is (not valid)


@pytest.mark.parametrize('yaw,speed', [
    (math.nan, 1.0), (math.inf, 1.0), (0.5, math.nan), (0.5, math.inf),
    (0.5, -math.inf), (None, 1.0),
])
def test_steering_is_nan_for_unusable_input(yaw, speed):
    assert math.isnan(steering_from_yaw_rate(yaw, speed, L, 0.3))


def test_bounded_clamps_both_sides_and_keeps_the_inside():
    # oracle: closed form
    assert bounded(1.1, 1.0, 0.3) == 1.1
    assert bounded(1.3, 1.0, 0.3) == 1.3          # exactly on the edge
    assert bounded(1.5, 1.0, 0.3) == 1.3
    assert bounded(0.2, 1.0, 0.3) == 0.7
    assert bounded(-5.0, -1.0, 0.3) == pytest.approx(-1.3, abs=SPEED_TOL)  # m/s


# ---------------------------------------------------------------- SpeedFilter

def test_filter_one_cycle_matches_the_kalman_equations():
    # oracle: closed form -- predict/update for x = [v, b], F = [[1, -dt],
    # [0, 1]], Q = diag(qa dt, qb dt), H = [1, 0], written out here.
    cfg = EstimatorConfig(sigma_accel=0.5, sigma_bias_rw=0.02,
                          sigma_wheel=0.08, sigma_bias0=0.3)
    f = SpeedFilter(cfg)
    z0, a, dt, z1 = 1.0, 2.0, 0.02, 1.1
    f.update(z0)
    assert f.predict(a, dt)
    rw, pb = 0.08 ** 2, 0.3 ** 2
    p00 = rw + dt * dt * pb + 0.5 ** 2 * dt
    p01 = -dt * pb
    p11 = pb + 0.02 ** 2 * dt
    v_pred = z0 + a * dt
    assert f.v == pytest.approx(v_pred, abs=SPEED_TOL)  # m/s
    assert f.P[0][0] == pytest.approx(p00, abs=1e-15)
    assert f.P[0][1] == pytest.approx(p01, abs=1e-15)
    assert f.P[1][1] == pytest.approx(p11, abs=1e-15)

    f.update(z1)
    s = p00 + rw
    k0, k1 = p00 / s, p01 / s
    y = z1 - v_pred
    assert f.v == pytest.approx(v_pred + k0 * y, abs=SPEED_TOL)  # m/s
    assert f.b == pytest.approx(k1 * y, abs=ACC_TOL)  # m/s^2
    assert f.P[0][0] == pytest.approx((1 - k0) * p00, abs=1e-15)
    assert f.P[1][1] == pytest.approx(p11 - k1 * p01, abs=1e-15)


def test_filter_is_exact_when_accel_and_wheels_agree():
    # oracle: invariant -- true acceleration integrated exactly and a wheel
    # speed that matches it leave a zero innovation: v stays the truth, b 0.
    f = SpeedFilter(EstimatorConfig())
    a, dt = 1.5, 0.004
    f.update(0.0)
    for k in range(1, 501):
        assert f.predict(a, dt)
        if k % SLOW_EVERY == 0:
            f.update(a * k * dt)
    assert f.v == pytest.approx(a * 500 * dt, abs=1e-9)  # m/s
    assert f.b == pytest.approx(0.0, abs=1e-9)  # m/s^2


def test_filter_learns_a_constant_accel_bias_while_stopped():
    # oracle: invariant -- wheels at 0 for good and the accelerometer reading
    # +0.2 m/s^2 leave one consistent explanation: b = 0.2, v = 0.
    f = SpeedFilter(EstimatorConfig())
    f.update(0.0)
    for k in range(1, 5001):          # 20 s
        f.predict(0.2, 0.004)
        if k % SLOW_EVERY == 0:
            f.update(0.0)
    assert f.b == pytest.approx(0.2, abs=0.01)  # m/s^2
    assert f.v == pytest.approx(0.0, abs=0.01)  # m/s


def test_reset_forgets_speed_but_keeps_the_learned_bias():
    f = SpeedFilter(EstimatorConfig())
    f.update(0.0)
    for k in range(1, 5001):
        f.predict(0.2, 0.004)
        if k % SLOW_EVERY == 0:
            f.update(0.0)
    b, pbb = f.b, f.P[1][1]
    f.reset()
    assert not f.predict(1.0, 0.004)      # nothing to predict until re-seeded
    f.update(2.5)
    # oracle: spec -- reset() docstring: v re-seeded from the wheels, bias kept.
    assert f.v == 2.5
    assert f.b == b and f.P[1][1] == pbb
    assert f.P[0][1] == 0.0


def test_filter_restarts_only_strictly_past_reset_divergence():
    cfg = EstimatorConfig(reset_divergence=1.0)
    at = SpeedFilter(cfg)
    at.update(0.0)
    at.predict(0.0, 0.004)
    at.update(1.0)                    # exactly at the threshold: a KF step
    assert 0.0 < at.v < 1.0
    past = SpeedFilter(cfg)
    past.update(0.0)
    past.predict(0.0, 0.004)
    past.update(1.0 + 1e-9)           # one step past: snapped to the wheels
    assert past.v == 1.0 + 1e-9


@pytest.mark.parametrize('accel,dt', [
    (math.nan, 0.004), (math.inf, 0.004), (1.0, math.nan),
    (1.0, 0.0), (1.0, -0.004),
])
def test_filter_ignores_unusable_predictions(accel, dt):
    f = SpeedFilter(EstimatorConfig())
    f.update(1.0)
    before = (f.v, f.b, [row[:] for row in f.P])
    assert not f.predict(accel, dt)
    assert (f.v, f.b, f.P) == before


@pytest.mark.parametrize('z', [math.nan, math.inf, -math.inf])
def test_filter_ignores_non_finite_wheel_speed(z):
    f = SpeedFilter(EstimatorConfig())
    f.update(1.0)
    f.predict(0.5, 0.004)
    before = (f.v, f.b, [row[:] for row in f.P])
    f.update(z)
    assert (f.v, f.b, f.P) == before


def test_filter_does_nothing_before_the_first_wheel_speed():
    f = SpeedFilter(EstimatorConfig())
    assert not f.predict(1.0, 0.004)
    assert not f.initialized


# ---------------------------------------------------------------- estimator: speed

def test_no_imu_means_exactly_the_wheel_speed():
    # oracle: invariant -- with nothing to fuse, /odom/fused is /odom.
    est = VehicleStateEstimator(EstimatorConfig())
    for k, z in enumerate((0.0, 0.4, 1.7, -0.6)):
        e = est.on_wheel(k * 0.02, z)
        assert e.speed == z and e.mode == MODE_WHEEL_ONLY


def test_accel_source_none_ignores_a_live_imu():
    est = VehicleStateEstimator(EstimatorConfig(accel_source='none'))
    out = drive(est, 2.0, speed=lambda t: 0.5 + 0.8 * t, yaw=const(0.0),
                wheel=lambda t: 0.5 + 0.8 * t + 0.05)
    for t, truth, e in out:
        assert e.mode == MODE_WHEEL_ONLY
        assert e.speed == pytest.approx(truth + 0.05, abs=SPEED_TOL)  # m/s


def test_fused_speed_beats_noisy_wheels():
    # oracle: invariant -- truth is known by construction. The filter exists
    # to do better than the wheels; it must, with both sensors noisy:
    # wheels +-0.08 m/s (sigma_wheel), BNO with 1 m/s^2 rms vibration.
    rng = random.Random(7)

    def speed(t):     # 0 -> 2 m/s in 2 s, cruise, brake to 0.5 at 2 m/s^2
        if t < 2.0:
            return t
        if t < 4.0:
            return 2.0
        return max(2.0 - 2.0 * (t - 4.0), 0.5)

    noise = {}

    def wheel(t):
        noise[t] = rng.gauss(0.0, 0.08)
        return speed(t) + noise[t]

    est = VehicleStateEstimator(EstimatorConfig())
    out = drive(est, 6.0, speed, yaw=const(0.0), wheel=wheel,
                bno_ax_extra=lambda t: rng.gauss(0.0, 1.0))
    later = [(t, truth, e) for t, truth, e in out if t >= 0.5]
    assert all(e.mode != MODE_WHEEL_ONLY for _, _, e in later)
    fused = math.sqrt(sum((e.speed - truth) ** 2 for _, truth, e in later) / len(later))
    raw = math.sqrt(sum(noise[t] ** 2 for t, _, _ in later) / len(later))
    # Half the wheels' error: a margin, not a measured golden number.
    assert fused < 0.5 * raw


def test_wheelspin_is_rejected_and_held_to_the_clamp():
    # oracle: invariant -- the truth holds 1.0 m/s while the wheels spin at
    # 1.6 for 0.3 s; the accelerometer sees no acceleration. The spin must be
    # skipped as slip, so the speed sits on the clamp's lower edge,
    # wheel - max_divergence = 1.3, instead of following the wheels to 1.6.
    cfg = EstimatorConfig(max_divergence=0.3)
    est = VehicleStateEstimator(cfg)
    drive(est, 2.0, speed=const(1.0), yaw=const(0.0))
    spin = drive(est, 2.3, t0=2.0 + BNO_DT, speed=const(1.0), yaw=const(0.0),
                 wheel=const(1.6))
    assert len(spin) >= 14
    for _, _, e in spin:
        assert e.mode == MODE_SLIP
        assert e.speed == pytest.approx(1.3, abs=SPEED_TOL)  # m/s
    assert est.filter.v == pytest.approx(1.0, abs=0.02)  # m/s: the truth, not the spin
    grip = drive(est, 2.6, t0=2.3 + BNO_DT, speed=const(1.0), yaw=const(0.0))
    assert all(e.mode == MODE_FUSED for _, _, e in grip)
    assert grip[-1][2].speed == pytest.approx(1.0, abs=0.02)  # m/s


def test_wheels_slipping_past_max_slip_time_are_believed_again():
    # oracle: spec -- max_slip_time (EstimatorConfig): skipped for strictly
    # less than 0.5 s, then the filter restarts from the wheels.
    # A 0.9 m/s spin: wide enough that the gate (which widens as the
    # filter's uncertainty grows without wheel updates) still rejects it at
    # 0.5 s, so it is the time limit that ends the slip, not the gate.
    cfg = EstimatorConfig(max_slip_time=0.5)
    est = VehicleStateEstimator(cfg)
    drive(est, 1.0, speed=const(1.0), yaw=const(0.0))
    t_slip = 1.0 + BNO_DT
    est.on_imu('bno', t_slip, t_slip, 0.0, G, 0.0)
    assert est.on_wheel(t_slip, 1.9).mode == MODE_SLIP
    t = t_slip
    while t < t_slip + 0.5 - BNO_DT:
        t += BNO_DT
        est.on_imu('bno', t, t, 0.0, G, 0.0)
    est.on_imu('bno', t_slip + 0.5 - 1e-6, t_slip + 0.5 - 1e-6, 0.0, G, 0.0)
    e = est.on_wheel(t_slip + 0.5 - 1e-6, 1.9)
    assert e.mode == MODE_SLIP and e.speed == pytest.approx(1.6, abs=SPEED_TOL)  # m/s
    est.on_imu('bno', t_slip + 0.5, t_slip + 0.5, 0.0, G, 0.0)
    e = est.on_wheel(t_slip + 0.5, 1.9)
    assert e.mode == MODE_FUSED and e.speed == 1.9


def test_published_speed_never_leaves_max_divergence_of_the_wheels():
    # oracle: invariant -- the clamp, over a hostile run: a 3 m/s^2 accel
    # bias appearing mid-drive, vibration, and wheelspin bursts.
    rng = random.Random(11)
    cfg = EstimatorConfig(max_divergence=0.3)
    est = VehicleStateEstimator(cfg)
    wheels = {}

    def wheel(t):
        spin = 0.7 if 3.0 < t % 4.0 < 3.4 else 0.0
        wheels[t] = 1.0 + 0.5 * math.sin(t) + spin + rng.gauss(0.0, 0.08)
        return wheels[t]

    out = drive(est, 12.0, speed=lambda t: 1.0 + 0.5 * math.sin(t), yaw=const(0.4),
                wheel=wheel,
                bno_ax_extra=lambda t: (3.0 if t > 6.0 else 0.0) + rng.gauss(0.0, 1.0))
    assert len(out) > 500
    for t, _, e in out:
        assert abs(e.speed - wheels[t]) <= 0.3 + 1e-12
    # The clamp did act (not a vacuous pass over a well-behaved run).
    assert sum(e.mode in (MODE_CLAMPED, MODE_SLIP) for _, _, e in out) > 10


def test_stale_accel_falls_back_to_exactly_the_wheel_speed():
    # Timeouts and times are binary fractions so "exactly at the limit" is
    # exact in floating point (1.1 - 1.0 > 0.1 in binary).
    cfg = EstimatorConfig(imu_timeout=0.125)
    est = VehicleStateEstimator(cfg)
    drive(est, 1.0, speed=lambda t: 1.0 + t, yaw=const(0.0))
    last_bno = 1.0
    # oracle: spec -- imu_timeout: fresh up to and including the limit.
    assert est.on_wheel(last_bno + 0.125, 2.12).mode != MODE_WHEEL_ONLY
    e = est.on_wheel(last_bno + 0.125 + 1e-6, 2.17)
    assert e.mode == MODE_WHEEL_ONLY and e.speed == 2.17


def test_bno_dropout_mid_run_falls_back_then_resumes():
    est = VehicleStateEstimator(EstimatorConfig())
    out = drive(est, 3.0, speed=lambda t: 0.5 + 0.5 * t, yaw=const(0.0),
                wheel=lambda t: 0.5 + 0.5 * t + 0.02,
                bno_on=lambda t: not 1.0 < t < 2.0)
    modes = {round(t, 3): e for t, _, e in out}
    gap = [e for t, e in modes.items() if 1.2 <= t < 2.0]
    assert gap and all(e.mode == MODE_WHEEL_ONLY for e in gap)
    for t, e in modes.items():
        if 1.2 <= t < 2.0:
            assert e.speed == pytest.approx(0.5 + 0.5 * t + 0.02, abs=SPEED_TOL)  # m/s
    assert all(e.mode != MODE_WHEEL_ONLY for t, e in modes.items() if t >= 2.2)


def test_an_accel_gap_restarts_the_filter_from_the_wheels():
    # Accel says +2 m/s^2 while the wheels hold 1.0, so the filter sits off
    # the wheels. Then two BNO samples 60 ms apart (> max_accel_dt 50 ms, but
    # < imu_timeout, so the accel still counts as fresh): the gap is not
    # integrated, the filter restarts, and the next output IS the wheel speed.
    cfg = EstimatorConfig(max_accel_dt=0.05, imu_timeout=0.1)
    est = VehicleStateEstimator(cfg)
    drive(est, 1.0, speed=const(1.0), yaw=const(0.0))
    t = 1.0
    for k in range(1, 26):
        t = 1.0 + k * BNO_DT
        est.on_imu('bno', t, t, 2.0, G, 0.0)
        if k % SLOW_EVERY == 0:
            e = est.on_wheel(t, 1.0)
    assert e.speed != 1.0                     # the filter really was off the wheels
    t_gap = t + 0.06
    est.on_imu('bno', t_gap, t_gap, 2.0, G, 0.0)
    e = est.on_wheel(t_gap, 1.0)
    # oracle: spec -- max_accel_dt (EstimatorConfig): restart from the wheels.
    assert e.speed == 1.0 and e.mode == MODE_FUSED


def test_a_stamp_going_backwards_restarts_the_filter():
    est = VehicleStateEstimator(EstimatorConfig())
    drive(est, 1.0, speed=const(1.0), yaw=const(0.0))
    for k in range(1, 26):
        t = 1.0 + k * BNO_DT
        est.on_imu('bno', t, t, 2.0, G, 0.0)
        if k % SLOW_EVERY == 0:
            est.on_wheel(t, 1.0)
    est.on_imu('bno', 1.101, 1.0, 2.0, G, 0.0)     # stamp behind the last one
    assert est.on_wheel(1.101, 1.0).speed == 1.0


@pytest.mark.parametrize('z', [math.nan, math.inf, -math.inf])
def test_non_finite_wheel_speed_passes_through_and_kills_steering(z):
    est = VehicleStateEstimator(EstimatorConfig())
    drive(est, 1.0, speed=const(1.5), yaw=const(1.0))
    e = est.on_wheel(1.001, z)
    assert e.mode == MODE_INVALID
    assert (math.isnan(e.speed) and math.isnan(z)) or e.speed == z
    assert math.isnan(est.speed(1.002))
    assert math.isnan(est.steering(1.002))


def test_corner_does_not_read_as_braking():
    # oracle: invariant -- a steady circle at constant speed has no forward
    # acceleration at base_link. The BNO on the front axle reads
    # -yaw^2 * 0.36 = -0.36 m/s^2 there; if that reached the filter it would
    # be learned as a bias of the same size.
    est = VehicleStateEstimator(EstimatorConfig())
    out = drive(est, 10.0, speed=const(1.5), yaw=const(1.5 / 1.5))
    assert est.filter.b == pytest.approx(0.0, abs=0.02)  # m/s^2
    assert out[-1][2].speed == pytest.approx(1.5, abs=0.01)  # m/s


def test_upside_down_accel_source_is_not_used():
    # oracle: spec -- MIN_GRAVITY_Z: an IMU reading -g on z is mounted wrong.
    est = VehicleStateEstimator(EstimatorConfig())
    out = drive(est, 1.0, speed=lambda t: t, yaw=const(0.0), bno_z=-G,
                wheel=lambda t: t + 0.05)
    assert all(e.mode == MODE_WHEEL_ONLY for _, _, e in out)


# ---------------------------------------------------------------- estimator: gyros

FEED_DT = 2.0 ** -10   # ~1 ms, a binary fraction so timeout boundaries are exact


def _feed(est, source, t, yaw, z=G, n=vs.GRAVITY_MIN_SAMPLES):
    for k in range(n):
        est.on_imu(source, t + k * FEED_DT, t + k * FEED_DT, 0.0, z, yaw)
    return t + (n - 1) * FEED_DT


def test_yaw_rate_is_the_weighted_mean_of_usable_gyros():
    # oracle: closed form -- (1 * 0.5 + 3 * 0.7) / (1 + 3)
    cfg = EstimatorConfig(gyro_weight=(('bno', 1.0), ('vesc', 3.0)))
    est = VehicleStateEstimator(cfg)
    _feed(est, 'bno', 0.0, 0.5)
    t = _feed(est, 'vesc', 0.0, 0.7)
    assert est.yaw_rate(t) == pytest.approx((0.5 + 2.1) / 4.0, abs=1e-12)  # rad/s


def test_a_zero_weight_gyro_is_left_out():
    est = VehicleStateEstimator(EstimatorConfig(gyro_weight=(('bno', 1.0), ('vesc', 0.0))))
    _feed(est, 'bno', 0.0, 0.5)
    t = _feed(est, 'vesc', 0.0, 9.0)
    assert est.yaw_rate(t) == 0.5


def test_a_stale_gyro_drops_out_at_imu_timeout():
    est = VehicleStateEstimator(EstimatorConfig(imu_timeout=0.125))
    t_v = _feed(est, 'vesc', 0.0, 0.7)
    t_b = _feed(est, 'bno', 0.0625, 0.5)
    both = (0.5 + 0.7) / 2
    # oracle: spec -- imu_timeout, fresh up to and including the limit
    # (binary-fraction times so the boundary is exact).
    assert est.yaw_rate(t_v + 0.125) == pytest.approx(both, abs=1e-12)  # rad/s
    assert est.yaw_rate(t_v + 0.125 + 1e-6) == 0.5   # VESC just went stale
    assert est.yaw_rate(t_b + 0.125 + 1e-6) is None  # both gone


def test_gravity_check_needs_enough_samples_and_enough_g():
    est = VehicleStateEstimator(EstimatorConfig())
    t = _feed(est, 'bno', 0.0, 0.5, n=vs.GRAVITY_MIN_SAMPLES - 1)
    assert est.yaw_rate(t) is None
    t = _feed(est, 'bno', t + FEED_DT, 0.5, n=1)
    assert est.yaw_rate(t) == 0.5
    at = VehicleStateEstimator(EstimatorConfig())
    t = _feed(at, 'vesc', 0.0, 0.3, z=MIN_GRAVITY_Z)
    assert at.yaw_rate(t) == 0.3                     # exactly at the limit: ok
    below = VehicleStateEstimator(EstimatorConfig())
    t = _feed(below, 'vesc', 0.0, 0.3, z=MIN_GRAVITY_Z - 1e-6)
    assert below.yaw_rate(t) is None


def test_upside_down_gyro_is_excluded_not_averaged_in():
    # An upside-down IMU reads yaw rate with the wrong sign; averaged in, it
    # would cancel the good one and report the car driving straight.
    est = VehicleStateEstimator(EstimatorConfig())
    _feed(est, 'bno', 0.0, 0.8)
    t = _feed(est, 'vesc', 0.0, -0.8, z=-G)
    assert est.yaw_rate(t) == 0.8


@pytest.mark.parametrize('source,value', [
    ('bno', math.nan), ('bno', math.inf), ('vesc', math.nan)])
def test_non_finite_imu_samples_are_dropped(source, value):
    est = VehicleStateEstimator(EstimatorConfig())
    _feed(est, 'bno', 0.0, 0.5)
    t = _feed(est, 'vesc', 0.0, 0.5)
    est.on_imu(source, t + FEED_DT, t + FEED_DT, 0.0, G, value)
    est.on_imu(source, t + 2 * FEED_DT, t + 2 * FEED_DT, value, G, 0.9)
    est.on_imu(source, t + 3 * FEED_DT, t + 3 * FEED_DT, 0.0, value, 0.9)
    assert est.yaw_rate(t + 3 * FEED_DT) == 0.5


# ---------------------------------------------------------------- estimator: steering

def test_steering_on_a_steady_circle_end_to_end():
    # oracle: closed form -- R = 1.5 m: delta = atan(L / R).
    est = VehicleStateEstimator(EstimatorConfig())
    out = drive(est, 3.0, speed=const(1.5), yaw=const(1.0))
    t = out[-1][0]
    assert est.steering(t) == pytest.approx(math.atan(L / 1.5), abs=ANGLE_TOL)  # rad


def test_steering_shows_understeer_the_command_cannot():
    # The command asks for R = 1.05 m (0.33 rad); the car only yaws for 1.6 m.
    # /steering/measured must report the 1.6 m circle's angle.
    est = VehicleStateEstimator(EstimatorConfig())
    drive(est, 2.0, speed=const(1.6), yaw=const(1.0))
    # oracle: closed form -- atan(L / 1.6)
    assert est.steering(2.0) == pytest.approx(math.atan(L / 1.6), abs=ANGLE_TOL)  # rad
    assert est.steering(2.0) < 0.33 - 0.1


def test_steering_is_nan_when_slow_or_stale():
    # Binary-fraction times and timeouts: see the stale-accel test.
    cfg = EstimatorConfig(min_steering_speed=0.3, wheel_timeout=0.25, imu_timeout=0.125)
    est = VehicleStateEstimator(cfg)
    drive(est, 1.0, speed=const(0.2), yaw=const(0.5))
    assert math.isnan(est.steering(1.0))             # below min speed
    est2 = VehicleStateEstimator(cfg)
    drive(est2, 1.0, speed=const(1.0), yaw=const(0.5))
    assert math.isfinite(est2.steering(1.0))
    # oracle: spec -- wheel_timeout: valid up to and including the limit.
    est2.on_imu('bno', 1.25, 1.25, 0.0, G, 0.5)
    est2.on_imu('vesc', 1.25, 1.25, 0.0, G, 0.5)
    assert math.isfinite(est2.steering(1.25))
    est2.on_imu('bno', 1.25 + 1e-6, 1.25 + 1e-6, 0.0, G, 0.5)
    assert math.isnan(est2.steering(1.25 + 1e-6))    # wheels just went stale


def test_steering_is_nan_with_no_gyro():
    est = VehicleStateEstimator(EstimatorConfig())
    drive(est, 1.0, speed=const(1.0), yaw=const(0.5), bno_on=const(False),
          vesc_on=const(False))
    assert math.isfinite(est.speed(1.0))
    assert math.isnan(est.steering(1.0))


def test_gyros_steadily_apart_give_nan_steering():
    # VESC yaw at half scale in a steady 1 rad/s turn: 0.5 rad/s apart, past
    # the 0.15 threshold. Neither can be picked as the right one.
    est = VehicleStateEstimator(EstimatorConfig())
    drive(est, 3.0, speed=const(1.5), yaw=const(1.0), vesc_yaw=const(0.5))
    assert est.gyros_disagree(3.0)
    assert math.isnan(est.steering(3.0))


def test_lagging_vesc_in_a_slalom_is_not_a_disagreement():
    # The VESC IMU is stamped on arrival, ~20 ms late. In a 1 Hz slalom of
    # +-2 rad/s that is a +-0.25 rad/s instantaneous difference, but it
    # averages out in the signed running mean (attenuated ~3x by the 0.5 s
    # time constant to ~0.08 < 0.15).
    est = VehicleStateEstimator(EstimatorConfig())
    w = 2 * math.pi

    def yaw(t):
        return 2.0 * math.sin(w * t)

    def vesc_yaw(t):
        return 2.0 * math.sin(w * (t - 0.02))

    flagged = []
    for t, _, _ in drive(est, 6.0, speed=const(2.0), yaw=yaw, vesc_yaw=vesc_yaw):
        flagged.append(est.gyros_disagree(t))
    assert len(flagged) > 200 and not any(flagged)


def test_after_a_disagreement_the_survivor_is_used_alone():
    est = VehicleStateEstimator(EstimatorConfig(imu_timeout=0.1))
    drive(est, 3.0, speed=const(1.5), yaw=const(1.0), vesc_yaw=const(0.5))
    assert math.isnan(est.steering(3.0))
    drive(est, 3.5, t0=3.0 + BNO_DT, speed=const(1.5), yaw=const(1.0),
          vesc_on=const(False))
    assert est.steering(3.5) == pytest.approx(math.atan(L / 1.5), abs=ANGLE_TOL)  # rad
