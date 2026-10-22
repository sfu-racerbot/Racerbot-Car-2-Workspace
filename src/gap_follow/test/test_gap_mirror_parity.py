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
import math
import os
import sys

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import Joy

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
    params = gap_mirror.node_params()
    params['command_slew_max_dt'] = DT
    node.command_slew_max_dt = DT
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
