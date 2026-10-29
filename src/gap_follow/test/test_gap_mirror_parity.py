"""
The simulators' copy of the gap_follow pipeline (tools/f1tenth_sim/
gap_mirror.py) must command what the real node commands. Every simulator
number in docs/gap-follow.md comes from that copy, so if it drifts, the docs
describe a controller the car does not run -- which is exactly what happened
to run_validation.py's old gap_command (docs/gap-follow-improvement-plan.md).

This drives the real GapFollowNode (shipped YAML) and GapController (same
YAML, via gap_mirror.node_params) through one scan sequence and compares the
published steering and speed tick by tick. Needs ROS sourced:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/gap_follow/test/test_gap_mirror_parity.py -v
"""
import json
import math
import os
import sys

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import String

from gap_follow.gap_follow_node import GapFollowNode

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'tools', 'f1tenth_sim'))
import gap_mirror  # noqa: E402

sys.path.insert(0, HERE)
from test_gap_follow_corners_node import (  # noqa: E402
    _corridor, _left_turn, _mirror, _raycast)

TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']
PACKAGED_CONFIG = os.path.join(HERE, '..', 'config', 'gap_follow.yaml')
DT = 0.025
# Commands must agree to float precision. The node takes its slew/ramp
# interval from its own clock and the mirror from the tick count; capping
# command_slew_max_dt at exactly one tick on both sides makes both intervals
# exactly DT (the node's clock is always at least DT past the last command
# here), because near the forward reserve the braking ceiling is steep enough
# that a millisecond of clock jitter moves it by a tenth of a m/s.
TOLERANCE = 1e-6


class _Lidar:
    def __init__(self, scan):
        self.angle_min = scan.angle_min
        self.angle_increment = scan.angle_increment
        self.range_min = scan.range_min


def _scene_sequence():
    """A drive through the cases the pipeline branches on: straights,
    approaching corners both ways, hugging a wall, a blocked arc, a creep,
    open road. Each item is (scan, speed to force the car to, or None)."""
    scenes = [_corridor(0.75, 0.75)] * 3
    # Off centre before a turn: centering and corner setup both active.
    scenes += [_left_turn(0.55, 0.95, cx) for cx in (2.6, 2.2, 1.8)]
    scenes += [_left_turn(0.7, 0.7, cx) for cx in (3.0, 2.0, 1.5, 1.0, 0.6)]
    scenes += [_mirror(_left_turn(0.7, 0.7, cx)) for cx in (2.0, 1.2, 0.8)]
    scenes += [_corridor(0.4, 1.1), _corridor(1.1, 0.4)]
    scenes += [[]] * 2
    # A short wall filling the forward cone 0.4 m ahead, open either side:
    # inside the forward reserve, so the car creeps toward an escape.
    half = 0.4 * math.tan(math.radians(30.0))
    scenes += [[((0.4, -half), (0.4, half))]] * 3
    scans = [(_raycast(segments), None) for segments in scenes]
    # At walking pace (forced), hugging a wall before a square corner: the
    # turn the gap asks for sweeps the body into it, so the arc guard steps in.
    hug = _raycast(_left_turn(0.25, 1.15, 0.6))
    scans += [(hug, 0.5), (hug, None), (hug, None)]
    scans += [(_raycast([]), None)] * 3
    return scans


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '--params-file', PACKAGED_CONFIG, *TEST_ONLY_DRIVE])
    n = GapFollowNode()
    assert n.drive_pub.topic_name == '/test_only/drive'
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_mirror_reads_every_parameter_it_needs_from_the_shipped_yaml():
    params = gap_mirror.node_params()
    assert gap_mirror.REQUIRED_PARAMS <= set(params)
    # Nothing in the YAML goes silently unread: a new node parameter must be
    # either mirrored or listed as deliberately not mirrored.
    assert set(params) <= gap_mirror.REQUIRED_PARAMS


def test_mirror_refuses_a_config_it_does_not_model(tmp_path):
    text = open(PACKAGED_CONFIG).read().replace(
        'enable_cornering_anticipation: false', 'enable_cornering_anticipation: true')
    assert 'enable_cornering_anticipation: true' in text
    path = tmp_path / 'gap_follow.yaml'
    path.write_text(text)
    with pytest.raises(ValueError, match='cornering anticipation'):
        gap_mirror.node_params(path)


def test_mirror_commands_what_the_node_commands(node):
    joy = Joy()
    joy.buttons = [0] * 5
    joy.buttons[4] = 1
    published = []
    node.drive_pub.publish = lambda msg: published.append(msg)

    scans = _scene_sequence()
    # This sequence exists to exercise corner setup, which ships OFF since
    # 2026-10-27, so it is switched on for the node and the mirror alike.
    params = gap_mirror.node_params()
    params['command_slew_max_dt'] = DT
    params['enable_corner_setup'] = True
    node.command_slew_max_dt = DT
    node.enable_corner_setup = True
    mirror = gap_mirror.GapController(params, _Lidar(scans[0][0]))
    # Already rolling fast with the rack hard left while the corridor runs
    # straight: the first ticks are braked by the arc the rack is still on.
    speed = 3.0
    node.last_commanded_speed = speed
    mirror.last_commanded_speed = speed
    node.steering_basis = mirror.steering_basis = 0.3
    node.last_command_time = node.get_clock().now() - Duration(seconds=DT)

    compared = 0
    guard_ticks = setup_ticks = 0
    for step, (scan, forced_speed) in enumerate(scans):
        if forced_speed is not None:
            speed = node.last_commanded_speed = forced_speed
        node.joy_callback(joy)
        odom = Odometry()
        odom.twist.twist.linear.x = speed
        node.odom_callback(odom)
        if node.last_command_time is not None:
            node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
        # Start each tick from the node's own carried state, so the comparison
        # is one decision from identical inputs and clock jitter cannot
        # accumulate across ticks of slewing.
        mirror.steering_basis = node.steering_basis
        mirror.last_commanded_speed = node.last_commanded_speed
        mirror.previous_target_idx = node.previous_target_idx
        mirror.last_command_time = step - 1
        node.scan_callback(scan)
        expected_steering, expected_speed = mirror.command(
            np.asarray(scan.ranges), speed, step)
        drive = published[-1].drive
        assert drive.steering_angle == pytest.approx(expected_steering, abs=TOLERANCE), (
            f'tick {step}: steering')
        assert drive.speed == pytest.approx(expected_speed, abs=TOLERANCE), (
            f'tick {step}: speed')
        compared += 1
        guard_ticks += mirror.telemetry.get('arc_guard', 0.0) != 0.0
        setup_ticks += mirror.telemetry.get('corner_setup_bias', 0.0) != 0.0
        # Odometry lags a braking command, as a real car's does, so the
        # acceleration ramp's "start from the measured speed" branch is used.
        speed = max(drive.speed, 0.8 * speed)
    assert compared == len(scans)
    # The sequence must actually exercise the branches, or this proves
    # nothing about them.
    assert any(abs(p.drive.steering_angle) > 0.05 for p in published)
    assert guard_ticks >= 1 and setup_ticks >= 3, (guard_ticks, setup_ticks)
    assert any(0.0 < p.drive.speed <= node.escape_creep_speed for p in published)


def _box_ahead(rear_x, y=0.0, length=0.58, width=0.30):
    x0, x1, hw = rear_x, rear_x + length, width / 2.0
    return [((x0, y - hw), (x0, y + hw)), ((x0, y + hw), (x1, y + hw)),
            ((x1, y + hw), (x1, y - hw)), ((x1, y - hw), (x0, y - hw))]


def test_mirror_matches_the_node_with_opponent_awareness_on():
    """The same comparison with enable_opponent_awareness on and another car
    in the scene: ahead in a 2 m corridor, closing, drifting sideways, so the
    tracker confirms it, the follow cap binds and the pass decision is made
    -- in both, identically, every tick."""
    rclpy.init(args=['--ros-args', '--params-file', PACKAGED_CONFIG,
                     '-p', 'enable_opponent_awareness:=true', *TEST_ONLY_DRIVE])
    node = GapFollowNode()
    try:
        assert node.drive_pub.topic_name == '/test_only/drive'
        published = []
        node.drive_pub.publish = published.append
        node.command_slew_max_dt = DT
        params = gap_mirror.node_params()
        params['enable_opponent_awareness'] = True
        params['command_slew_max_dt'] = DT
        joy = Joy()
        joy.buttons = [0] * 5
        joy.buttons[4] = 1
        speed = 2.5
        node.last_commanded_speed = speed
        mirror = None
        modes = []
        for step in range(60):
            # Ego at `speed` along a straight corridor; the other car at
            # 1.4 m/s, drifting left at 0.6 m/s (heading ~23 deg, so no pass)
            # from 0.8 m right: it crosses out of the 0.45 m follow band,
            # where only the widened band of a refused pass still caps.
            rear_x = 2.4 + (1.4 - speed) * step * DT
            scan = _raycast(_corridor(1.0, 1.0) + _box_ahead(rear_x, -0.8 + 0.6 * step * DT))
            t = 50.0 + step * DT
            scan.header.stamp.sec = int(t)
            scan.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
            if mirror is None:
                mirror = gap_mirror.GapController(params, _Lidar(scan))
            node.joy_callback(joy)
            odom = Odometry()
            odom.twist.twist.linear.x = speed
            node.odom_callback(odom)
            # The scene is drawn for a car that does not turn: keep the rack
            # (and so both dead-reckoned poses) straight, on both sides.
            node.steering_basis = mirror.steering_basis = 0.0
            node.last_commanded_speed = mirror.last_commanded_speed = speed
            mirror.previous_target_idx = node.previous_target_idx
            mirror.last_command_time = step - 1
            node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
            node.scan_callback(scan)
            expected_steering, expected_speed = mirror.command(
                np.asarray(scan.ranges), speed, step)
            drive = published[-1].drive
            assert node.opponent_mode == mirror.opponent_mode, f'tick {step}: mode'
            assert drive.steering_angle == pytest.approx(expected_steering, abs=TOLERANCE), (
                f'tick {step}: steering')
            assert drive.speed == pytest.approx(expected_speed, abs=TOLERANCE), (
                f'tick {step}: speed')
            modes.append((mirror.opponent_mode, mirror.binding_cap))
        # The layer must actually have been exercised, or this proves nothing:
        # a confirmed mover for most of the run, and the follow cap binding.
        assert sum(m is not None for m, _b in modes) >= 30, modes
        assert sum(b == 'follow' for _m, b in modes) >= 5, modes
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _feed_imu(node, state, yaw_rate):
    """What imu_bridge publishes for one tick: its verdict, then one sample."""
    node.imu_status_callback(String(data=json.dumps(
        {'state': state, 'reason': None, 'imu': 'bno'})))
    msg = Imu()
    msg.angular_velocity.z = float(yaw_rate)
    node.imu_callback(msg)


def test_mirror_matches_the_node_with_steering_damping_on():
    """The same comparison with steering damping on at the shipped gain, a yaw
    rate that swings both ways, and a faulty IMU for the last third of the
    sequence. In that third both must add exactly nothing, and they must agree
    on every tick. The healthy part must really exercise the term, or this
    proves nothing about it (docs/gap-follow.md#steering-damping-gyro)."""
    rclpy.init(args=['--ros-args', '--params-file', PACKAGED_CONFIG,
                     '-p', 'enable_steering_damping:=true', *TEST_ONLY_DRIVE])
    node = GapFollowNode()
    try:
        assert node.drive_pub.topic_name == '/test_only/drive'
        published = []
        node.drive_pub.publish = lambda msg: published.append(msg)
        node.command_slew_max_dt = DT
        params = gap_mirror.node_params()
        params['enable_steering_damping'] = True
        params['command_slew_max_dt'] = DT
        joy = Joy()
        joy.buttons = [0] * 5
        joy.buttons[4] = 1
        scans = _scene_sequence()
        mirror = gap_mirror.GapController(params, _Lidar(scans[0][0]))
        speed = 2.0
        node.last_commanded_speed = speed
        node.steering_basis = mirror.steering_basis = 0.0
        node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
        healthy_ticks = 2 * len(scans) // 3
        damping = []
        for step, (scan, forced_speed) in enumerate(scans):
            if forced_speed is not None:
                speed = node.last_commanded_speed = forced_speed
            yaw = 2.0 * math.sin(0.35 * step)   # rad/s: left, then right
            healthy = step < healthy_ticks
            _feed_imu(node, 'ok' if healthy else 'fault', yaw)
            node.joy_callback(joy)
            odom = Odometry()
            odom.twist.twist.linear.x = speed
            node.odom_callback(odom)
            if node.last_command_time is not None:
                node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
            mirror.steering_basis = node.steering_basis
            mirror.last_commanded_speed = node.last_commanded_speed
            mirror.previous_target_idx = node.previous_target_idx
            mirror.last_command_time = step - 1
            node.scan_callback(scan)
            expected_steering, expected_speed = mirror.command(
                np.asarray(scan.ranges), speed, step,
                yaw_rate=yaw if healthy else None)
            drive = published[-1].drive
            assert drive.steering_angle == pytest.approx(expected_steering, abs=TOLERANCE), (
                f'tick {step}: steering')
            assert drive.speed == pytest.approx(expected_speed, abs=TOLERANCE), (
                f'tick {step}: speed')
            damping.append(mirror.telemetry.get('steering_damping', 0.0))
            speed = max(drive.speed, 0.8 * speed)
        # The healthy part must carry a real term on enough driving ticks, or
        # the agreement above is agreement about nothing.
        assert sum(abs(d) > 1e-3 for d in damping[:healthy_ticks]) >= 10, damping
        assert all(d == 0.0 for d in damping[healthy_ticks:]), damping
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _grip_scene_sequence():
    """A corridor the car steers through at a steady 2 m/s. Every tick is forced
    to 2.0 m/s, so the grip's demand is high enough to be judged on each tick
    (the scenes in _scene_sequence are mostly too slow to judge)."""
    return [(_raycast(_corridor(1.4, 0.6)), 2.0)] * 74


def test_mirror_matches_the_node_with_adaptive_grip_on():
    """The same comparison with the adaptive grip on at the shipped values, on a
    steady corridor at 2 m/s. Steering damping is pinned off: it ships on since
    2026-10-27, and with it on no healthy tick moved the limit, so the test
    would prove nothing about the grip. Eight ticks warm up with the IMU at fault, long
    enough for the rack to settle on the corridor's steering, so the first
    judged tick is not lagged by the rack's slew. Then the healthy part: the car
    yaws as the rack was last commanded, scaled to a grip ratio of 1.0 for
    fourteen ticks and 0.5 for six, then 1.0 again, so the limit rises and is
    cut. The last ten ticks are a faulty IMU, where both must sit at the base.
    The healthy part must really move the limit both ways, or the agreement
    proves nothing (docs/gap-follow.md#adaptive-grip-imu)."""
    rclpy.init(args=['--ros-args', '--params-file', PACKAGED_CONFIG,
                     '-p', 'enable_adaptive_grip:=true',
                     '-p', 'enable_steering_damping:=false', *TEST_ONLY_DRIVE])
    node = GapFollowNode()
    try:
        assert node.drive_pub.topic_name == '/test_only/drive'
        published = []
        node.drive_pub.publish = lambda msg: published.append(msg)
        node.command_slew_max_dt = DT
        params = gap_mirror.node_params()
        params['enable_adaptive_grip'] = True
        params['enable_steering_damping'] = False
        params['command_slew_max_dt'] = DT
        joy = Joy()
        joy.buttons = [0] * 5
        joy.buttons[4] = 1
        scans = _grip_scene_sequence()
        mirror = gap_mirror.GapController(params, _Lidar(scans[0][0]))
        speed = 2.0
        node.last_commanded_speed = speed
        node.steering_basis = mirror.steering_basis = 0.0
        node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
        warm_ticks, healthy_ticks = 8, 8 + 56
        limits = []
        for step, (scan, forced_speed) in enumerate(scans):
            if forced_speed is not None:
                speed = node.last_commanded_speed = forced_speed
            healthy = warm_ticks <= step < healthy_ticks
            # The yaw a car makes at the rack's last command, scaled to the grip
            # ratio this block calls for. gap_logic reads the ratio back as
            # yaw * speed / (speed^2 * tan(steer) / wheelbase), so this is exact.
            last_steer = published[-1].drive.steering_angle if published else 0.0
            j = step - warm_ticks   # index within the healthy part
            ratio = 0.5 if 14 <= j < 20 else 1.0
            yaw = ratio * speed * math.tan(last_steer) / params['wheelbase']
            _feed_imu(node, 'ok' if healthy else 'fault', yaw)
            node.joy_callback(joy)
            odom = Odometry()
            odom.twist.twist.linear.x = speed
            node.odom_callback(odom)
            if node.last_command_time is not None:
                node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
            mirror.steering_basis = node.steering_basis
            mirror.last_commanded_speed = node.last_commanded_speed
            mirror.previous_target_idx = node.previous_target_idx
            mirror.last_command_time = step - 1
            node.scan_callback(scan)
            expected_steering, expected_speed = mirror.command(
                np.asarray(scan.ranges), speed, step,
                yaw_rate=yaw if healthy else None)
            drive = published[-1].drive
            assert drive.steering_angle == pytest.approx(expected_steering, abs=TOLERANCE), (
                f'tick {step}: steering')
            assert drive.speed == pytest.approx(expected_speed, abs=TOLERANCE), (
                f'tick {step}: speed')
            # The limit itself, not only the speed it produces, must agree.
            assert node.adaptive_grip.limit == pytest.approx(
                mirror.grip.limit, abs=TOLERANCE), f'tick {step}: lateral limit'
            limits.append(mirror.telemetry.get('lateral_limit'))
            speed = max(drive.speed, 0.8 * speed)
        base = params['max_lateral_accel']
        # The healthy part must move the limit both ways, or this proves nothing.
        healthy_limits = [v for v in limits[warm_ticks:healthy_ticks] if v is not None]
        rises = sum(b > a for a, b in zip(healthy_limits, healthy_limits[1:]))
        cuts = sum(b < a for a, b in zip(healthy_limits, healthy_limits[1:]))
        assert rises >= 3 and cuts >= 1, (rises, cuts, healthy_limits)
        assert max(healthy_limits) > base + 0.05, healthy_limits
        # The faulty part: the limit sits at the base on every driving tick. It
        # must first have risen above the base, or the reset proves nothing.
        assert limits[healthy_ticks - 1] > base + 1e-3, limits[healthy_ticks - 1]
        faulty_limits = [v for v in limits[healthy_ticks:] if v is not None]
        assert faulty_limits, 'the faulty part must drive, or it proves nothing'
        assert all(v == pytest.approx(base, abs=1e-12) for v in faulty_limits), faulty_limits
    finally:
        node.destroy_node()
        rclpy.shutdown()
