"""Integration tests for the tracking-law seam in pure_pursuit_node (spec 00
Unit 3): selecting a law, its failure behavior, and that every law stays
behind the same LB deadman and watchdogs.

Needs ROS2 sourced and the pure_pursuit package built:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/pure_pursuit/test/test_tracking_law_integration.py -v

Always built with drive_topic:=/test_only/drive so a test can never reach
ackermann_mux even if the driver stack is up. The deadman tests enable the
gate the way test_opponent_integration.py does.
"""
import math
import os

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy, LaserScan

from pure_pursuit import racing_math, tracking_laws
from pure_pursuit.pure_pursuit_node import PurePursuitNode
from pure_pursuit.tracking_laws import TrackingLawError


@pytest.fixture(scope='module')
def profiled_csv(tmp_path_factory):
    src = os.path.join(os.path.dirname(__file__), '..', 'waypoints',
                       'example_stadium_raw.csv')
    xy = racing_math.load_xy_csv(src)
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    curvature = racing_math.estimate_path_curvature(xy, closed=True)
    speed = racing_math.compute_velocity_profile(
        seg_len, curvature, v_max=6.0, v_min=0.5, a_lat_max=8.0,
        a_accel_max=3.0, a_brake_max=8.0, closed=True)
    out_path = str(tmp_path_factory.mktemp('waypoints') / 'profiled.csv')
    racing_math.save_profiled_csv(out_path, xy, speed)
    return out_path


def _make_node(profiled_csv, law_name, extra_params=()):
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'enable_deadman:=false',
                     '-p', f'steering_law:={law_name}',
                     '-p', 'drive_topic:=/test_only/drive',
                     *extra_params])
    node = PurePursuitNode()
    return node


@pytest.fixture
def node_factory(profiled_csv):
    built = []

    def make(law_name, extra_params=()):
        node = _make_node(profiled_csv, law_name, extra_params)
        built.append(node)
        return node

    yield make
    for node in built:
        node.destroy_node()
    rclpy.shutdown()


def _set_pose(node, x, y, yaw=0.0):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    node.pose_callback(msg)


def _clear_scan(n=361, angle_span=math.pi):
    scan = LaserScan()
    scan.angle_min = -angle_span / 2.0
    scan.angle_increment = angle_span / n
    scan.angle_max = scan.angle_min + (n - 1) * scan.angle_increment
    scan.range_min, scan.range_max = 0.05, 12.0
    scan.ranges = [10.0] * n
    return scan


def _hold_lb(node):
    joy = Joy()
    joy.buttons = [0] * (node.deadman_button + 1)
    joy.buttons[node.deadman_button] = 1
    node.joy_callback(joy)


def _release_lb(node):
    joy = Joy()
    joy.buttons = [0] * (node.deadman_button + 1)
    node.joy_callback(joy)


def _capture_published(node):
    published = []
    original = node.drive_pub.publish
    node.drive_pub.publish = lambda msg: (published.append(msg), original(msg))
    return published


def _one_command_interval(node):
    node.last_command_time = node.get_clock().now() - Duration(
        seconds=node.command_slew_max_dt)


# A pose on the stadium line, slightly left of it, so a correct law steers
# right (negative). The bottom straight runs along y=-1.2.
def _pose_slightly_left(node):
    _set_pose(node, -1.5, -1.1, 0.0)
    node.scan_callback(_clear_scan())


@pytest.mark.parametrize('law_name', ['pure_pursuit', 'stanley', 'lqr'])
def test_lb_held_on_line_steers_toward_it(node_factory, law_name):
    node = node_factory(law_name)
    node.enable_deadman = True
    _hold_lb(node)
    _pose_slightly_left(node)
    _one_command_interval(node)
    published = _capture_published(node)
    node.control_loop()
    last = published[-1]
    assert last.drive.speed > 0.0
    assert last.drive.steering_angle < 0.0   # the line is below-right; steer right
    assert node.last_decision_state == law_name


@pytest.mark.parametrize('law_name', ['pure_pursuit', 'stanley', 'lqr'])
def test_lb_released_stops(node_factory, law_name):
    node = node_factory(law_name)
    node.enable_deadman = True
    _hold_lb(node)
    _pose_slightly_left(node)
    node.control_loop()                  # establish a driving command
    published = _capture_published(node)
    _release_lb(node)
    node.control_loop()
    assert published[-1].drive.speed == 0.0


@pytest.mark.parametrize('law_name', ['pure_pursuit', 'stanley', 'lqr'])
def test_joy_never_received_stops(node_factory, law_name):
    node = node_factory(law_name)
    node.enable_deadman = True
    _pose_slightly_left(node)
    published = _capture_published(node)
    node.control_loop()
    assert published[-1].drive.speed == 0.0


@pytest.mark.parametrize('law_name', ['pure_pursuit', 'stanley', 'lqr'])
def test_joy_stale_stops(node_factory, law_name):
    node = node_factory(law_name)
    node.enable_deadman = True
    _hold_lb(node)
    _pose_slightly_left(node)
    node.control_loop()
    # Age the last /joy past the timeout.
    node.last_joy_time = node.get_clock().now() - Duration(
        seconds=node.joy_timeout_sec + 1.0)
    published = _capture_published(node)
    node.control_loop()
    assert published[-1].drive.speed == 0.0


@pytest.mark.parametrize('law_name', ['pure_pursuit', 'stanley', 'lqr'])
def test_law_failure_stops_the_car(node_factory, law_name):
    node = node_factory(law_name)
    node.enable_deadman = True
    _hold_lb(node)
    _pose_slightly_left(node)

    class ExplodingLaw:
        name = law_name

        def steer(self, inp, path):
            raise TrackingLawError('boom')

        def reset(self):
            pass

    node.tracking_law = ExplodingLaw()
    published = _capture_published(node)
    node.control_loop()
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'steering_law_failed'


def test_nan_pose_never_reaches_the_law(node_factory):
    """A NaN yaw must stop the car. The pose callback's finite guard or the
    law's own non-finite check catches it; either way the published command is
    zero. (Which check fires is recorded here: pose_callback drops non-finite
    poses before they are cached, so the node stops on 'waiting_for_pose'.)"""
    node = node_factory('stanley')
    node.enable_deadman = True
    _hold_lb(node)
    node.scan_callback(_clear_scan())
    _set_pose(node, -1.5, -1.1, math.nan)
    published = _capture_published(node)
    node.control_loop()
    assert published[-1].drive.speed == 0.0


def test_unknown_steering_law_refuses_to_start(profiled_csv):
    # Self-contained: this test owns its own rclpy context (it does not use
    # node_factory, which would hold and re-shutdown the same context).
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'steering_law:=bogus',
                     '-p', 'drive_topic:=/test_only/drive'])
    try:
        with pytest.raises(RuntimeError, match='steering_law'):
            PurePursuitNode()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('law_name', ['pure_pursuit', 'stanley', 'lqr'])
def test_steering_law_is_not_live_tunable(node_factory, law_name):
    node = node_factory(law_name)
    result = node._parameter_callback([
        Parameter('steering_law', Parameter.Type.STRING, 'lqr')])
    assert not result.successful
    assert node.get_parameter('steering_law').value == law_name


# Pure pursuit's lookahead knobs are deliberately live-tunable (a
# pre-existing dashboard feature); test_live_lookahead_tune_reaches_the_law
# below covers them. Only the new laws' dotted params must be refused.
@pytest.mark.parametrize('law_name, dotted', [
    ('stanley', 'stanley.gain'), ('lqr', 'lqr.q_lateral')])
def test_law_params_are_not_live_tunable(node_factory, law_name, dotted):
    node = node_factory(law_name)
    result = node._parameter_callback([
        Parameter(dotted, Parameter.Type.DOUBLE, 9.9)])
    assert not result.successful


@pytest.mark.parametrize('law_name, params_class', [
    ('stanley', tracking_laws.StanleyParams), ('lqr', tracking_laws.LqrParams)])
def test_node_defaults_are_the_law_defaults(node_factory, law_name, params_class):
    # No params file: the node's declare_parameter defaults alone. They must
    # build the same law the harness tunes and judges (tracking_laws' dataclass
    # defaults), or a launch without the YAML runs untested gains.
    node = node_factory(law_name)
    assert node.tracking_law.params == params_class()


@pytest.mark.parametrize('law_name, dotted, field', [
    ('stanley', 'stanley.delay_compensation', 'delay_compensation'),
    ('stanley', 'stanley.steering_time_constant', 'steering_time_constant'),
    ('lqr', 'lqr.delay_compensation', 'delay_compensation'),
    ('lqr', 'lqr.steering_time_constant', 'steering_time_constant')])
def test_delay_and_smoothing_params_reach_the_law(node_factory, law_name, dotted, field):
    node = node_factory(law_name, ['-p', f'{dotted}:=0.237'])
    assert getattr(node.tracking_law.params, field) == 0.237


def _tune(node, **values):
    result = node._parameter_callback([
        Parameter(name, Parameter.Type.DOUBLE, float(value))
        for name, value in values.items()])
    assert result.successful, result.reason


def test_live_lookahead_tune_reaches_the_law(node_factory):
    # The dashboard can live-tune min/max_lookahead and lookahead_speed_gain,
    # and the node accepts them. PurePursuitLaw must then steer with the new
    # values -- not keep the ones it was built with while the node reports
    # the tuned ones back (the "dashboard reads a value the car is not using"
    # failure live_tuning.py exists to prevent).
    node = node_factory('pure_pursuit')
    node.enable_deadman = True
    _hold_lb(node)
    _pose_slightly_left(node)
    inp = tracking_laws.TrackingInput(
        x=node.car_x, y=node.car_y, yaw=node.car_yaw, speed=0.0,
        nearest_idx=racing_math.find_nearest_index(
            node.xy, (node.car_x, node.car_y), closed=True)[0],
        previous_steering=0.0)
    for lookahead in (0.6, 2.0):
        _tune(node, min_lookahead=lookahead, max_lookahead=lookahead,
              lookahead_speed_gain=0.0)
        # Oracle: plain pure pursuit geometry from racing_math at that
        # lookahead (speed 0, gain 0 -> lookahead is exactly min_lookahead).
        target = racing_math.find_lookahead_index(
            node.seg_len, inp.nearest_idx, lookahead, closed=True)
        bx, by = racing_math.world_to_body(
            float(node.xy[target][0] - inp.x), float(node.xy[target][1] - inp.y), inp.yaw)
        expected = float(np.clip(racing_math.steering_from_curvature(
            racing_math.steering_arc_curvature(bx, by), node.wheelbase),
            -node.max_steering_angle, node.max_steering_angle))
        result = node.tracking_law.steer(inp, node.path_context)
        assert result.steering == pytest.approx(expected, abs=1e-12)
        assert f'lookahead={lookahead:.2f}m' in result.detail


def test_live_steering_rate_tune_reaches_the_law_limits(node_factory):
    # max_steering_rate is live-tunable too, and MPC's rate constraint reads
    # it from the law's VehicleLimits.
    node = node_factory('pure_pursuit')
    _tune(node, max_steering_rate=2.5)
    assert node.tracking_law.limits.max_steering_rate == 2.5


@pytest.mark.parametrize('law_name', ['pure_pursuit', 'stanley', 'lqr'])
def test_profile_reload_resets_the_law(node_factory, profiled_csv, law_name):
    node = node_factory(law_name)
    node.enable_deadman = True
    _hold_lb(node)
    _pose_slightly_left(node)
    node.control_loop()
    resets = []
    original_reset = node.tracking_law.reset
    node.tracking_law.reset = lambda: (resets.append(1), original_reset())
    node._parameter_callback([
        Parameter('waypoints_file', Parameter.Type.STRING, profiled_csv)])
    assert resets, 'reset() must be called on every profile activation'


# ============================================================================
# MPC node behaviour (spec 04). osqp-backed; skipped until G0 lands osqp into
# the ROS Python (the law itself is unit-tested in test_tracking_laws.py).
# ============================================================================


@pytest.fixture
def osqp_available():
    return pytest.importorskip('osqp', reason='spec 04 G0: osqp not installed')


def _mpc_node(profiled_csv, **params):
    extra = []
    for key, value in params.items():
        extra += ['-p', f'{key}:={value}']
    return _make_node(profiled_csv, 'mpc', extra)


def test_mpc_lb_held_drives(osqp_available, profiled_csv):
    node = _mpc_node(profiled_csv)
    try:
        node.enable_deadman = True
        _hold_lb(node)
        _pose_slightly_left(node)
        _one_command_interval(node)
        published = _capture_published(node)
        node.control_loop()
        assert published[-1].drive.speed > 0.0
        assert node.last_decision_state == 'mpc'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mpc_node_defaults_are_the_law_defaults_and_delay_reaches_it(
        osqp_available, profiled_csv):
    node = _mpc_node(profiled_csv)
    try:
        assert node.tracking_law.params == tracking_laws.MpcParams()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    node = _mpc_node(profiled_csv, **{'mpc.delay_compensation': 0.237})
    try:
        assert node.tracking_law.params.delay_compensation == 0.237
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mpc_miss_falls_back_to_pure_pursuit(osqp_available, profiled_csv):
    # deadline_sec=1e-9 forces every solve to miss; with the default
    # max_consecutive_misses=3 the first two ticks fall back.
    node = _mpc_node(profiled_csv, **{'mpc.deadline_sec': 1e-9})
    try:
        node.enable_deadman = True
        _hold_lb(node)
        _pose_slightly_left(node)
        _one_command_interval(node)
        published = _capture_published(node)
        node.control_loop()
        assert node.last_decision_state == 'mpc_fallback'
        assert published[-1].drive.speed > 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mpc_latches_after_max_consecutive_misses(osqp_available, profiled_csv):
    node = _mpc_node(profiled_csv, **{'mpc.deadline_sec': 1e-9,
                                      'mpc.max_consecutive_misses': 3})
    try:
        node.enable_deadman = True
        _hold_lb(node)
        _pose_slightly_left(node)
        published = _capture_published(node)
        for _ in range(3):
            _one_command_interval(node)
            node.control_loop()
        assert published[-1].drive.speed == 0.0
        assert node.last_decision_state == 'mpc_miss_latched'
        # Stays latched on further ticks with LB held.
        _one_command_interval(node)
        node.control_loop()
        assert published[-1].drive.speed == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mpc_latch_clears_when_lb_is_released(osqp_available, profiled_csv):
    node = _mpc_node(profiled_csv, **{'mpc.deadline_sec': 1e-9,
                                      'mpc.max_consecutive_misses': 2})
    try:
        node.enable_deadman = True
        _hold_lb(node)
        _pose_slightly_left(node)
        published = _capture_published(node)
        for _ in range(2):
            _one_command_interval(node)
            node.control_loop()
        assert node.last_decision_state == 'mpc_miss_latched'
        # Release and re-hold LB: the latch clears, the next tick falls back.
        _release_lb(node)
        node.control_loop()
        _hold_lb(node)
        _pose_slightly_left(node)
        _one_command_interval(node)
        node.control_loop()
        assert node.last_decision_state == 'mpc_fallback'
        assert published[-1].drive.speed > 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_live_lookahead_tune_reaches_the_mpc_fallback_law(osqp_available, profiled_csv):
    node = _mpc_node(profiled_csv)
    try:
        _tune(node, min_lookahead=0.9, max_lookahead=0.9, lookahead_speed_gain=0.0)
        assert node._mpc_fallback_law.params == tracking_laws.PurePursuitParams(
            min_lookahead=0.9, max_lookahead=0.9, lookahead_speed_gain=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mpc_intent_never_runs_the_solver(osqp_available, profiled_csv):
    # Drive-intent safety contract (docs/drive-intent.md): the diagnostic
    # reads what the control path computed and must not run the controller
    # again. With no MPC plan to replay, the arrow comes from the
    # pure-pursuit fallback law, and the MPC law is never called.
    node = _mpc_node(profiled_csv)
    try:
        _pose_slightly_left(node)
        node._last_law_predicted = ()
        calls = []
        original = node.tracking_law.steer
        node.tracking_law.steer = lambda *a, **k: (calls.append(1), original(*a, **k))[1]
        nearest = racing_math.find_nearest_index(
            node.xy, (node.car_x, node.car_y), closed=True)[0]
        path = node._intent_path(0.0, 2.0, nearest_idx=nearest)
        assert calls == []
        assert len(path) >= 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
