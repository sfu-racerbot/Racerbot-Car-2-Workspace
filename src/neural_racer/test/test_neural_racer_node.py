"""The real NeuralRacerNode: deadman, shadow mode, refusals, stops, intent.

Needs ROS 2 sourced, and its own domain -- never the car's graph:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/neural_racer/test/test_neural_racer_node.py -v

Each test drives the node through its callbacks directly and asserts on the
*published command*, never only on an internal flag (CLAUDE.md, deadman table).
"""
import ast
import json
import math
import os

import nr_testutil
import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import Imu, Joy, LaserScan
from std_msgs.msg import String

from neural_racer import approve_model, neural_racer_node as nrn
from neural_racer.policy_numpy import PolicyNumpy

# Every node built here publishes drive commands on this topic, never on the
# real /drive (which reaches ackermann_mux when the driver stack is up). It
# goes LAST in every rclpy.init; test_no_test_here_builds_a_node_on_the_real_drive
# checks every call in this file.
TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']
DEADMAN = 4


@pytest.fixture(scope='module')
def models(tmp_path_factory):
    """A models dir holding one fully approved and one shadow-only model."""
    root = tmp_path_factory.mktemp('nr')
    out = {}
    for seed, kind in ((0, 'full'), (1, 'shadow_only')):
        w = nr_testutil.random_weights(seed)
        reports = {'selection_report.json': json.dumps({'pass': True}),
                   'referee_report.json': json.dumps({'pass': True})}
        src, h = nr_testutil.write_model(
            root / f'in{seed}', w, nr_testutil.card(),
            golden=nr_testutil.make_golden(PolicyNumpy(w)), reports=reports) \
            if (root / f'in{seed}').mkdir() is None else (None, None)
        approve_model.approve(src, root / 'models', shadow_only=(kind == 'shadow_only'))
        out[kind] = h
    out['dir'] = str(root / 'models')
    return out


def _make(models, *params):
    rclpy.init(args=['--ros-args', '-p', f"models_dir:={models['dir']}", *params, *TEST_ONLY_DRIVE])
    return nrn.NeuralRacerNode()


@pytest.fixture
def driving(models):
    n = _make(models, '-p', f"model_hash:={models['full']}", '-p', 'shadow:=false')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _scan(node, ranges=None):
    msg = LaserScan()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.angle_min = nr_testutil.UST_ANGLE_MIN
    msg.angle_increment = nr_testutil.UST_INCREMENT
    msg.angle_max = msg.angle_min + 1080 * msg.angle_increment
    msg.range_min, msg.range_max = 0.02, 30.0
    msg.ranges = list(nr_testutil.clear_scan() if ranges is None else ranges)
    return msg


def _joy(node, held=True):
    msg = Joy()
    msg.buttons = [0] * (DEADMAN + 1)
    msg.buttons[DEADMAN] = 1 if held else 0
    node.joy_callback(msg)


def _odom(node, speed=0.5):
    msg = Odometry()
    msg.twist.twist.linear.x = speed
    node.odom_callback(msg)


def _imu(node, ok=True):
    node.imu_status_callback(String(data=json.dumps({'state': 'ok' if ok else 'fault'})))
    msg = Imu()
    msg.angular_velocity.z = 0.1
    msg.linear_acceleration.x = 0.2
    node.imu_callback(msg)


def _ready(node, held=True):
    _joy(node, held)
    _odom(node)
    _imu(node)


def _capture(node):
    published = []
    original = node.cmd_pub.publish
    node.cmd_pub.publish = lambda msg: (published.append(msg), original(msg))
    return published


def _ago(node, seconds):
    return node.get_clock().now() - Duration(nanoseconds=int(seconds * 1e9))


# --- the four deadman paths, on the published command -----------------------------

def test_lb_held_drives(driving):
    """The permissive half: everything fresh, LB held -> a non-zero command."""
    out = _capture(driving)
    _ready(driving)
    driving.scan_callback(_scan(driving))
    assert out[-1].drive.speed > 0.0
    assert driving.last_decision_state == 'reactive'


def test_lb_released_zero(driving):
    out = _capture(driving)
    _ready(driving, held=False)
    driving.scan_callback(_scan(driving))
    assert out[-1].drive.speed == 0.0 and driving.last_decision_state == 'deadman_released'


def test_joy_never_received_zero(driving):
    out = _capture(driving)
    _odom(driving)
    _imu(driving)
    driving.scan_callback(_scan(driving))
    assert out[-1].drive.speed == 0.0 and driving.last_decision_state == 'waiting_for_joy'


def test_joy_stale_zero(driving):
    out = _capture(driving)
    _ready(driving)
    driving.last_joy_time = _ago(driving, 0.6)            # limit 0.5 s
    driving.scan_callback(_scan(driving))
    assert out[-1].drive.speed == 0.0 and driving.last_decision_state == 'joy_stale'


def test_joy_message_without_lb_button_zero(driving):
    out = _capture(driving)
    _odom(driving)
    _imu(driving)
    msg = Joy()
    msg.buttons = [1, 1, 1]                                # too short to have index 4
    driving.joy_callback(msg)
    driving.scan_callback(_scan(driving))
    assert out[-1].drive.speed == 0.0 and driving.last_decision_state == 'deadman_button_missing'


# --- other inputs ------------------------------------------------------------------------

def test_imu_stale_zero(driving):
    out = _capture(driving)
    _ready(driving)
    driving.last_imu_time = _ago(driving, 0.15)
    driving.scan_callback(_scan(driving))
    assert out[-1].drive.speed == 0.0 and driving.last_decision_state == 'imu_unavailable'


def test_imu_status_fault_zero(driving):
    out = _capture(driving)
    _ready(driving)
    driving.imu_status_callback(String(data=json.dumps({'state': 'fault'})))
    driving.scan_callback(_scan(driving))
    assert out[-1].drive.speed == 0.0 and driving.last_decision_state == 'imu_unavailable'


def test_odom_stale_zero(driving):
    out = _capture(driving)
    _ready(driving)
    driving.last_odom_time = _ago(driving, 0.2)
    driving.scan_callback(_scan(driving))
    assert out[-1].drive.speed == 0.0 and driving.last_decision_state == 'stale_odom'


def test_stale_scan_zero(driving):
    out = _capture(driving)
    _ready(driving)
    scan = _scan(driving)
    scan.header.stamp = _ago(driving, 0.2).to_msg()
    driving.scan_callback(scan)
    assert out[-1].drive.speed == 0.0 and driving.last_decision_state == 'stale_scan'


def test_wrong_beam_count_zero(driving):
    out = _capture(driving)
    _ready(driving)
    driving.scan_callback(_scan(driving, ranges=np.full(1080, 5.0)))
    assert out[-1].drive.speed == 0.0 and driving.last_decision_state == 'bad_observation'


def test_wall_ahead_brakes(driving):
    out = _capture(driving)
    _ready(driving)
    r = nr_testutil.clear_scan()
    r[530:551] = 0.3
    driving.scan_callback(_scan(driving, ranges=r))
    assert out[-1].drive.speed == 0.0


def test_command_within_limits_over_many_ticks(driving):
    out = _capture(driving)
    rng = np.random.default_rng(0)
    for _ in range(40):
        _ready(driving)
        driving.scan_callback(_scan(driving, ranges=rng.uniform(0.5, 12.0, 1081)))
    assert all(abs(m.drive.steering_angle) <= 0.33 + 1e-9 and 0.0 <= m.drive.speed <= 1.0 + 1e-9
               for m in out)


def test_gru_and_history_reset_on_stop(driving):
    _capture(driving)
    for _ in range(3):
        _ready(driving)
        driving.scan_callback(_scan(driving))
    assert np.any(driving.policy.h != 0.0)
    _joy(driving, held=False)
    driving.scan_callback(_scan(driving))
    assert np.all(driving.policy.h == 0.0) and not driving.history.scans


def test_map_mode_without_map_inputs_zero(models):
    n = _make(models, '-p', f"model_hash:={models['full']}", '-p', 'shadow:=false', '-p', 'mode:=map')
    try:
        out = _capture(n)
        _ready(n)
        n.scan_callback(_scan(n))
        assert out[-1].drive.speed == 0.0 and n.last_decision_state == 'map_inputs_unavailable'
    finally:
        n.destroy_node()
        rclpy.shutdown()


# --- shadow mode and refusals -----------------------------------------------------------

def test_shadow_never_publishes_drive(models):
    n = _make(models, '-p', f"model_hash:={models['full']}", '-p', 'shadow_topic:=/test_only/shadow')
    try:
        assert n.shadow and n.cmd_pub.topic_name == '/test_only/shadow'
        assert n.get_publishers_info_by_topic('/test_only/drive') == []
        out = _capture(n)
        _ready(n)
        n.scan_callback(_scan(n))
        assert out[-1].drive.speed > 0.0                       # it computes, on the shadow topic
    finally:
        n.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('topic', ['/drive', '/teleop', '/ackermann_cmd', '/commands/motor/speed',
                                   '/auto_race/drive'])
def test_shadow_topic_guard(models, topic):
    n = _make(models, '-p', f"model_hash:={models['full']}", '-p', f'shadow_topic:={topic}')
    try:
        assert n.refusal is not None and n.cmd_pub is None
        assert n.get_publishers_info_by_topic(topic) == []
        _ready(n)
        n.scan_callback(_scan(n))                              # must not raise, must not publish
        assert n.last_decision_state == 'refused'
    finally:
        n.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('params, why', [
    (['-p', 'model_hash:=0123456789abcdef'], 'not approved'),
    (['-p', 'max_speed:=7.0'], 'max_speed'),
    (['-p', 'camera_enabled:=true'], 'camera'),
    (['-p', 'mode:=fast'], 'unknown mode'),
])
def test_refusals_publish_only_zero(models, params, why):
    base = ['-p', f"model_hash:={models['full']}", '-p', 'shadow:=false']
    n = _make(models, *base, *params)
    try:
        assert why in n.refusal
        out = _capture(n)
        _ready(n)
        n.scan_callback(_scan(n))
        assert out and all(m.drive.speed == 0.0 for m in out)
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_shadow_only_model_cannot_drive(models):
    n = _make(models, '-p', f"model_hash:={models['shadow_only']}", '-p', 'shadow:=false')
    try:
        assert 'shadow mode only' in n.refusal
        out = _capture(n)
        _ready(n)
        n.scan_callback(_scan(n))
        assert all(m.drive.speed == 0.0 for m in out)
    finally:
        n.destroy_node()
        rclpy.shutdown()


# --- diagnostics never cost the car anything ----------------------------------------------------

def test_intent_failure_keeps_driving(driving, monkeypatch):
    out = _capture(driving)

    def boom(*a, **k):
        raise RuntimeError('diagnostic broke')
    monkeypatch.setattr(nrn.schema, 'build', boom)
    for _ in range(8):
        _ready(driving)
        driving._intent_throttle.reset()
        driving.scan_callback(_scan(driving))
    assert len(out) == 8 and all(m.drive.speed > 0.0 for m in out)
    assert driving._intent_failures.disabled


def test_intent_published_after_command(driving):
    order = []
    driving.cmd_pub.publish = lambda m: order.append('cmd')
    driving.intent_pub.publish = lambda m: order.append('intent')
    _ready(driving)
    driving.scan_callback(_scan(driving))
    assert order == ['cmd', 'intent']


# --- the guard on this file itself ------------------------------------------------------------------

def _rclpy_init_arg_lists():
    tree = ast.parse(open(__file__).read())
    calls = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'init' and getattr(node.func.value, 'id', '') == 'rclpy'):
            args = [kw.value for kw in node.keywords if kw.arg == 'args']
            elts = args[0].elts if args and isinstance(args[0], ast.List) else []
            calls.append((node.lineno, elts))
    return calls


def test_no_test_here_builds_a_node_on_the_real_drive():
    calls = _rclpy_init_arg_lists()
    assert len(calls) >= 1
    bad = [line for line, elts in calls if not elts or ast.unparse(elts[-1]) != '*TEST_ONLY_DRIVE']
    assert bad == []


def test_test_only_drive_really_moves_the_publisher(driving):
    assert driving.drive_topic == '/test_only/drive'
    assert driving.cmd_pub.topic_name == '/test_only/drive'
    assert driving.get_publishers_info_by_topic('/drive') == []
