"""
imu_bridge.vehicle_state: gyro odometry (GyroOdometry, HeadingTrack).

No ROS needed:

    python3 -m pytest src/imu_bridge/test/ -v

Oracles: closed forms (a constant-rate arc: heading r*T, position on the
circle of radius v/r; the trapezoidal rule, exact for a linear rate; the
integral of a sinusoid), and invariants (correction * wheel pose = gyro pose;
with no gyro the pose IS vesc_to_odom's; distance is vesc_to_odom's).
Wheel poses are generated exactly the way vesc_to_odom.cpp integrates them:
x += v cos(yaw) dt, y += v sin(yaw) dt, yaw += model_rate dt.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from imu_bridge import vehicle_state as vs  # noqa: E402
from imu_bridge.vehicle_state import (  # noqa: E402
    MODEL,
    EstimatorConfig,
    GyroOdometry,
    HeadingTrack,
    VehicleStateEstimator,
    compose,
    foreign_parent,
    inverse,
    wrap_angle,
)

G = 9.80665
POS_TOL = 1e-9    # m
ANG_TOL = 1e-9    # rad
GYRO_DT = 0.004   # BNO, 250 Hz
WHEEL_EVERY = 5   # vesc_to_odom pose every 5th gyro tick = 50 Hz


WARMUP_TICKS = 60   # 0.24 s of gyro before the first wheel pose: >= 10 samples of each


def run(odo, t_end, v, true_rate, model_rate=None, *, t0=0.0, x0=(0.0, 0.0, 0.0),
        bno=True, vesc=False, vesc_rate=None, bno_z=G, wheel_lead=0.0, bno_on=None,
        warmup_ticks=WARMUP_TICKS):
    """Drive a GyroOdometry: gyros at true_rate(t), wheel poses integrated
    like vesc_to_odom with model_rate(t) (default: the truth). Returns the
    list of (wheel pose, OdomStep). wheel_lead stamps each wheel pose that
    much after the newest gyro sample (needs extrapolation). The gyros run
    warmup_ticks before the first wheel pose, as on the car, where imu_bridge
    streams long before the car moves (and the gravity check needs 10)."""
    model_rate = model_rate or true_rate
    est = odo.est
    x, y, yaw = x0
    steps = []
    n = int(round((t_end - t0) / GYRO_DT))
    last_wheel_t = None
    for k in range(-warmup_ticks, 0):
        t = t0 + k * GYRO_DT
        if bno:
            est.on_imu('bno', t, t, 0.0, bno_z, true_rate(t0))
        if vesc and k % WHEEL_EVERY == 0:
            est.on_imu('vesc', t, t, 0.0, G, (vesc_rate or true_rate)(t0))
    for k in range(n + 1):
        t = t0 + k * GYRO_DT
        if bno and (bno_on is None or bno_on(t)):
            est.on_imu('bno', t, t, 0.0, bno_z, true_rate(t))
        if k % WHEEL_EVERY == 0:
            if vesc:
                est.on_imu('vesc', t, t, 0.0, G, (vesc_rate or true_rate)(t))
            tw = t + wheel_lead
            if last_wheel_t is not None:
                dt = tw - last_wheel_t
                x += v * math.cos(yaw) * dt
                y += v * math.sin(yaw) * dt
                yaw += model_rate(tw) * dt
            last_wheel_t = tw
            steps.append(((x, y, yaw), odo.on_wheel_pose(tw, tw, x, y, yaw)))
    return steps


def const(c):
    return lambda t: c


def new(**cfg):
    return GyroOdometry(VehicleStateEstimator(EstimatorConfig(**cfg)))


# ---------------------------------------------------------------- pose algebra

@pytest.mark.parametrize('p', [(1.0, -2.0, 0.3), (0.0, 0.0, 0.0), (-3.5, 0.2, -2.9)])
def test_compose_with_inverse_is_identity(p):
    # oracle: invariant
    for q in (compose(p, inverse(p)), compose(inverse(p), p)):
        assert q[0] == pytest.approx(0.0, abs=POS_TOL)  # m
        assert q[1] == pytest.approx(0.0, abs=POS_TOL)  # m
        assert wrap_angle(q[2]) == pytest.approx(0.0, abs=ANG_TOL)  # rad


def test_compose_rotates_then_translates():
    # oracle: closed form -- a at (1, 0) facing +y; b one metre ahead of a.
    assert compose((1.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0)) == pytest.approx(
        (1.0, 1.0, math.pi / 2), abs=POS_TOL)


@pytest.mark.parametrize('a,expected', [
    (0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi), (3 * math.pi, math.pi),
    (2 * math.pi + 0.1, 0.1), (-0.1 - 4 * math.pi, -0.1)])
def test_wrap_angle(a, expected):
    # oracle: spec -- docstring: wrapped into (-pi, pi].
    assert wrap_angle(a) == pytest.approx(expected, abs=1e-12)


# ---------------------------------------------------------------- HeadingTrack

def test_track_integrates_a_linear_rate_exactly():
    # oracle: closed form -- trapezoid is exact for rate = a + b t:
    # integral over [0, T] = a T + b T^2 / 2.
    track = HeadingTrack()
    a, b = 0.3, 0.8
    for k in range(101):
        track.add(k * 0.004, a + b * k * 0.004, max_gap=0.05)
    T = 0.4
    assert track.value(T, 0.0) == pytest.approx(a * T + b * T * T / 2, abs=1e-12)  # rad


def test_track_interpolates_between_samples_and_extrapolates_at_the_last_rate():
    track = HeadingTrack()
    track.add(0.0, 1.0, 0.05)
    track.add(0.01, 1.0, 0.05)                      # heading 0.01
    # oracle: closed form -- constant rate 1 rad/s.
    assert track.value(0.004, 0.0) == pytest.approx(0.004, abs=1e-12)
    assert track.value(0.01 + 0.03, 0.03) == pytest.approx(0.04, abs=1e-12)
    # oracle: spec -- max_extrapolation: up to and including the limit.
    assert track.value(0.01 + 0.0625, 0.0625) is not None
    assert track.value(0.01 + 0.0625 + 1e-9, 0.0625) is None
    assert track.value(-1e-9, 1.0) is None          # before the first sample


def test_track_does_not_integrate_across_a_gap_and_remembers_it():
    track = HeadingTrack()
    track.add(0.0, 2.0, 0.05)
    track.add(0.02, 2.0, 0.05)                      # heading 0.04
    track.add(0.12, 2.0, 0.05)                      # 0.10 s gap: not integrated
    assert track.value(0.12, 0.0) == pytest.approx(0.04, abs=1e-12)
    assert track.has_gap(0.0, 0.05) and track.has_gap(0.10, 0.2)
    assert not track.has_gap(0.0, 0.02)             # wholly before the gap
    assert not track.has_gap(0.12, 0.2)             # wholly after it


def test_track_ignores_duplicate_and_backward_stamps():
    track = HeadingTrack()
    track.add(0.0, 1.0, 0.05)
    track.add(0.01, 1.0, 0.05)
    track.add(0.01, 50.0, 0.05)
    track.add(0.005, 50.0, 0.05)
    assert track.value(0.01, 0.0) == pytest.approx(0.01, abs=1e-12)
    assert track.rate == 1.0


# ---------------------------------------------------------------- GyroOdometry

def test_first_pose_starts_both_frames_together():
    odo = new()
    step = odo.on_wheel_pose(0.0, 0.0, 2.0, -1.0, 0.7)
    assert step.pose == (2.0, -1.0, 0.7)
    assert step.correction == (0.0, 0.0, 0.0)


def test_until_a_gyro_has_proven_its_mount_the_model_is_used():
    # oracle: spec -- GRAVITY_MIN_SAMPLES: a gyro is not trusted before its
    # gravity check has 10 samples. Gyro and wheels starting together, the
    # first step (0.02 s = 6 BNO samples) is the model's; later ones the BNO's.
    odo = new()
    steps = run(odo, 0.2, v=1.0, true_rate=const(1.0), model_rate=const(0.5),
                warmup_ticks=0)
    sources = [s.source for _, s in steps[1:]]
    assert sources[0] == MODEL
    assert set(sources[2:]) == {'bno'}


def test_heading_follows_the_gyro_not_the_steering_model():
    # Car 2's measured turning gain: the car turns 0.95x what the command
    # (and so vesc_to_odom) says. oracle: closed form -- heading = r * T.
    r = 1.0
    odo = new()
    steps = run(odo, 3.0, v=1.5, true_rate=const(r), model_rate=const(r / 0.95))
    wheel, step = steps[-1]
    T = 3.0
    assert step.pose[2] == pytest.approx(r * T, abs=1e-9)  # rad
    assert wheel[2] == pytest.approx(r / 0.95 * T, abs=1e-9)  # rad: odometry's wrong answer
    assert {s.source for _, s in steps[1:]} == {'bno'}


def test_position_lies_on_the_true_circle():
    # oracle: closed form -- base_link on a circle of radius R = v / r from
    # (0, 0) facing +x: (R sin rT, R (1 - cos rT)). The midpoint rule's
    # error is v dt (r dt)^2 / 24 per step: ~1e-4 m over this run.
    v, r, T = 1.5, 1.0, 3.0
    odo = new()
    steps = run(odo, T, v=v, true_rate=const(r), model_rate=const(r * 1.3))
    R = v / r
    x, y, _ = steps[-1][1].pose
    assert x == pytest.approx(R * math.sin(r * T), abs=1e-3)  # 1 mm
    assert y == pytest.approx(R * (1 - math.cos(r * T)), abs=1e-3)  # 1 mm


def test_distance_is_exactly_vesc_to_odoms():
    # oracle: invariant -- each step's distance is the wheel pose change
    # along its previous heading, i.e. v * dt as vesc_to_odom integrated it.
    odo = new()
    steps = run(odo, 2.0, v=1.2, true_rate=lambda t: math.sin(3 * t),
                model_rate=lambda t: 0.8 * math.sin(3 * t))
    dt = GYRO_DT * WHEEL_EVERY
    for _, step in steps[1:]:
        assert step.speed == pytest.approx(1.2, abs=1e-9)  # m/s
    path = sum(s.speed * dt for _, s in steps[1:])
    assert path == pytest.approx(1.2 * 2.0, abs=1e-9)  # m


def test_correction_composes_to_the_gyro_pose():
    # oracle: invariant -- the transform chain odom_imu -> odom -> base_link
    # must be exactly the gyro pose, at every step.
    odo = new()
    steps = run(odo, 4.0, v=1.0, true_rate=lambda t: 0.9 * math.cos(t),
                model_rate=lambda t: 1.1 * math.cos(t))
    for wheel, step in steps:
        chained = compose(step.correction, wheel)
        assert chained[0] == pytest.approx(step.pose[0], abs=POS_TOL)  # m
        assert chained[1] == pytest.approx(step.pose[1], abs=POS_TOL)  # m
        assert wrap_angle(chained[2] - step.pose[2]) == pytest.approx(0.0, abs=ANG_TOL)


def test_without_a_gyro_it_is_exactly_vesc_to_odom():
    # oracle: invariant -- the documented fallback: today's odometry, to the bit.
    odo = new()
    steps = run(odo, 3.0, v=1.4, true_rate=const(0.7), model_rate=const(0.9), bno=False)
    for wheel, step in steps:
        assert step.source == MODEL
        assert step.pose == pytest.approx(wheel, abs=1e-12)
        assert step.correction == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_heading_sources_none_ignores_a_live_gyro():
    odo = new(heading_sources=('none',))
    steps = run(odo, 1.0, v=1.0, true_rate=const(1.0), model_rate=const(0.5))
    assert all(s.source == MODEL for _, s in steps)
    assert steps[-1][1].pose == pytest.approx(steps[-1][0], abs=1e-12)


def test_a_step_across_a_gyro_gap_falls_back_for_that_step_only():
    # BNO silent from 1.00 to 1.10 s (> max_gyro_gap 0.05). Steps spanning
    # that use the model; heading = gyro outside, model inside.
    r_true, r_model = 1.0, 0.5
    odo = new(max_gyro_gap=0.05, imu_timeout=0.5)
    steps = run(odo, 2.0, v=1.0, true_rate=const(r_true), model_rate=const(r_model),
                bno_on=lambda t: not 1.0 < t < 1.1)
    sources = [(round(s.stamp, 3), s.source) for _, s in steps[1:]]
    model_steps = [t for t, src in sources if src == MODEL]
    assert model_steps and all(1.0 <= t <= 1.1 + 1e-9 for t in model_steps)
    assert all(src == 'bno' for t, src in sources if t <= 1.0 - 1e-9 or t > 1.12)
    # oracle: closed form -- heading = r_true over gyro steps + r_model over model steps
    dt = GYRO_DT * WHEEL_EVERY
    expected = dt * (r_true * (len(sources) - len(model_steps)) + r_model * len(model_steps))
    assert steps[-1][1].pose[2] == pytest.approx(expected, abs=1e-9)  # rad


def test_extrapolation_errors_do_not_accumulate():
    # Each wheel pose is stamped 3 ms after the newest gyro sample, so every
    # step's end heading is an extrapolation. With a changing rate each one
    # is a little wrong; the next step must correct it, not keep it.
    # oracle: closed form -- heading = integral of 2 sin(2 t) = 1 - cos(2 T),
    # off only by the final extrapolation (< rate change over 3 ms).
    odo = new(max_heading_extrapolation=0.005)
    rate = lambda t: 2.0 * math.sin(2.0 * t)  # noqa: E731
    steps = run(odo, 6.0, v=1.0, true_rate=rate, wheel_lead=0.003)
    T = steps[-1][1].stamp
    assert {s.source for _, s in steps[1:]} == {'bno'}
    # trapezoid error over 6 s at 4 ms plus one 3 ms extrapolation: < 1e-4 rad
    assert steps[-1][1].pose[2] == pytest.approx(1.0 - math.cos(2.0 * T), abs=1e-4)  # rad


def test_too_far_to_extrapolate_falls_back():
    odo = new(max_heading_extrapolation=0.002)
    steps = run(odo, 0.5, v=1.0, true_rate=const(1.0), model_rate=const(0.5),
                wheel_lead=0.003)
    assert all(s.source == MODEL for _, s in steps[1:])


def test_bno_preferred_then_vesc_then_model():
    odo = new(imu_timeout=0.1)
    steps = run(odo, 1.0, v=1.0, true_rate=const(1.0), vesc=True)
    assert steps[-1][1].source == 'bno'
    # BNO stops: within imu_timeout of its last sample it is still used, after it the VESC.
    steps = run(odo, 1.5, t0=1.0 + GYRO_DT, v=1.0, true_rate=const(1.0), vesc=True, bno=False)
    late = [s for _, s in steps if s.stamp > 1.0 + 0.1]
    assert late and all(s.source == 'vesc' for s in late)


def test_upside_down_bno_is_skipped_for_the_vesc():
    odo = new()
    steps = run(odo, 1.0, v=1.0, true_rate=const(1.0), vesc=True, bno_z=-G)
    assert {s.source for _, s in steps[1:]} == {'vesc'}


def test_disagreeing_gyros_fall_back_to_the_model():
    # VESC at half the BNO's rate in a steady turn: one is wrong, which is unknown.
    odo = new()
    steps = run(odo, 3.0, v=1.5, true_rate=const(1.0), vesc=True, vesc_rate=const(0.5))
    assert odo.est.gyros_disagree(3.0)
    assert steps[-1][1].source == MODEL


def test_heading_change_crossing_pi_is_small():
    # vesc_to_odom's yaw is wrapped by its quaternion: +3.1 -> -3.1 is a
    # 0.083 rad turn, not -6.2.
    odo = new(heading_sources=('none',))
    odo.on_wheel_pose(0.0, 0.0, 0.0, 0.0, 3.1)
    step = odo.on_wheel_pose(0.02, 0.02, 0.0, 0.0, -3.1)
    # oracle: closed form -- 2 pi - 6.2
    assert step.pose[2] - 3.1 == pytest.approx(2 * math.pi - 6.2, abs=1e-12)


@pytest.mark.parametrize('stamp', [0.02, 0.01])
def test_duplicate_or_backward_wheel_stamp_is_skipped(stamp):
    odo = new()
    odo.on_wheel_pose(0.0, 0.0, 0.0, 0.0, 0.0)
    odo.on_wheel_pose(0.02, 0.02, 0.03, 0.0, 0.0)
    before = odo.pose
    assert odo.on_wheel_pose(0.03, stamp, 9.0, 9.0, 1.0) is None
    assert odo.pose == before


@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_non_finite_wheel_pose_is_skipped(bad):
    odo = new()
    odo.on_wheel_pose(0.0, 0.0, 0.0, 0.0, 0.0)
    before = odo.pose
    assert odo.on_wheel_pose(0.02, 0.02, bad, 0.0, 0.0) is None
    assert odo.on_wheel_pose(0.02, 0.02, 0.0, 0.0, bad) is None
    assert odo.pose == before


@pytest.mark.parametrize('sources,ok', [
    (('bno', 'vesc'), True), (('vesc',), True), (('none',), True),
    ((), False), (('bno', 'bno'), False), (('none', 'bno'), False), (('realsense',), False)])
def test_heading_sources_validation(sources, ok):
    if ok:
        EstimatorConfig(heading_sources=sources)
    else:
        with pytest.raises(ValueError, match='heading_sources'):
            EstimatorConfig(heading_sources=sources)


# ---------------------------------------------------------------- TF conflict

def test_foreign_parent_of_odom_is_found():
    # oracle: spec -- tf2: one parent per frame. slam_toolbox left on `odom`
    # publishes map -> odom; that and odom_imu -> odom cannot both stand.
    msg = [('odom', 'base_link'), ('map', 'odom')]
    assert foreign_parent(msg, 'odom_imu', 'odom') == 'map'


def test_our_own_transform_and_other_children_are_not_conflicts():
    msg = [('odom_imu', 'odom'), ('odom', 'base_link'), ('map', 'odom_imu'),
           ('base_link', 'laser')]
    assert foreign_parent(msg, 'odom_imu', 'odom') is None
    assert foreign_parent([], 'odom_imu', 'odom') is None


def test_model_constant_is_not_a_gyro_name():
    assert MODEL not in vs.VehicleStateEstimator.SOURCES


def test_reversing_moves_backwards_along_the_gyro_heading():
    # oracle: closed form -- reversing at -0.8 m/s for 1 s on a straight,
    # facing +x: x = -0.8, y = 0, and every step's speed is -0.8.
    odo = new()
    steps = run(odo, 1.0, v=-0.8, true_rate=const(0.0))
    x, y, yaw = steps[-1][1].pose
    assert x == pytest.approx(-0.8, abs=1e-9)  # m
    assert y == pytest.approx(0.0, abs=1e-9)  # m
    assert all(s.speed == pytest.approx(-0.8, abs=1e-9) for _, s in steps[1:])  # m/s


def test_reversing_on_an_arc_matches_the_circle():
    # oracle: closed form -- reversing (v < 0) with yaw rate r < 0 traces the
    # same left-hand circle backwards: from (0,0) facing +x, after T the car
    # is at (R sin(rT), R (1 - cos(rT))) with R = v / r > 0.
    v, r, T = -1.0, -0.5, 2.0
    odo = new()
    steps = run(odo, T, v=v, true_rate=const(r), model_rate=const(0.3 * r))
    R = v / r
    x, y, yaw = steps[-1][1].pose
    assert yaw == pytest.approx(r * T, abs=1e-9)  # rad
    assert x == pytest.approx(R * math.sin(r * T), abs=1e-3)  # 1 mm
    assert y == pytest.approx(R * (1 - math.cos(r * T)), abs=1e-3)  # 1 mm


def test_step_yaw_rate_is_the_gyro_heading_change_over_dt():
    # oracle: closed form -- constant gyro rate r: each step's rate is r,
    # not the steering model's.
    odo = new()
    steps = run(odo, 1.0, v=1.0, true_rate=const(0.7), model_rate=const(0.2))
    for _, s in steps[1:]:
        assert s.yaw_rate == pytest.approx(0.7, abs=1e-9)  # rad/s
