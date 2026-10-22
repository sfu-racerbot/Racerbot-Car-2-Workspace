"""
gap_follow_node with opponent awareness (opponent_tracking.py) -- the node
wiring, the toggle, and the refusal paths. Needs ROS sourced and an isolated
domain (CLAUDE.md):

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/gap_follow/test/test_opponent_awareness_node.py -v

Every node here publishes on /test_only/drive (the remap comes after
--params-file, so the YAML's /drive cannot win). Scenes are ray-cast from
segments in the LiDAR frame; the follow-cap oracle is the closed form
(gap - follow_min_gap) / follow_time_gap from the box's true position.
"""
import math
import os
import sys

import numpy as np
import pytest
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import Joy

from gap_follow import opponent_tracking
from gap_follow.gap_follow_node import GapFollowNode

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_gap_follow_corners_node import _corridor, _raycast  # noqa: E402

PACKAGED_CONFIG = os.path.join(HERE, '..', 'config', 'gap_follow.yaml')
TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']
DT = 0.025
LASER_X = 0.26
FRONT = 0.36 / 2.0 + 0.58 / 2.0          # body front in base_link (0.47 m)
EGO_SPEED = 2.5
OPP_SPEED = 1.5


def _make(*extra):
    # command_slew_max_dt at exactly one tick makes the ramp interval exactly
    # DT whatever the wall clock did, as in test_gap_mirror_parity, so two
    # runs of the same scenes can be compared to float precision.
    rclpy.init(args=['--ros-args', '--params-file', PACKAGED_CONFIG,
                     '-p', f'command_slew_max_dt:={DT}', *extra, *TEST_ONLY_DRIVE])
    node = GapFollowNode()
    assert node.drive_pub.topic_name == '/test_only/drive'
    published = []
    node.drive_pub.publish = published.append
    return node, published


def _box(rear_x, y=0.0, length=0.58, width=0.30):
    """A car-sized rectangle whose rear face is rear_x ahead of the LiDAR."""
    x0, x1, hw = rear_x, rear_x + length, width / 2.0
    return [((x0, y - hw), (x0, y + hw)), ((x0, y + hw), (x1, y + hw)),
            ((x1, y + hw), (x1, y - hw)), ((x1, y - hw), (x0, y - hw))]


def _drive(node, published, ticks=40, rear_x0=2.2, lb=True, stamped=True,
           y0=0.0, vy=0.0, dt=DT):
    """Feed a 2 m corridor with a car ahead closing at EGO - OPP m/s.

    Odometry is held at EGO_SPEED and the rack at straight, so the ego motion
    the node dead-reckons is exactly the motion the scenes were drawn with.
    Returns [(rear_x, published speed)] per tick.
    """
    joy = Joy()
    joy.buttons = [0] * 5
    joy.buttons[4] = 1 if lb else 0
    out = []
    for k in range(ticks):
        rear_x = rear_x0 + (OPP_SPEED - EGO_SPEED) * k * dt
        y = y0 + vy * k * dt
        scan = _raycast(_corridor(1.0, 1.0) + _box(rear_x, y))
        if stamped:
            t = 100.0 + k * dt
            scan.header.stamp.sec = int(t)
            scan.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        node.joy_callback(joy)
        odom = Odometry()
        odom.twist.twist.linear.x = EGO_SPEED
        node.odom_callback(odom)
        node.steering_basis = 0.0
        node.last_commanded_speed = EGO_SPEED
        node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
        node.scan_callback(scan)
        out.append((rear_x, y, published[-1].drive.speed))
    return out


def _closed_form_cap(rear_x, params=opponent_tracking.OpponentParams()):
    gap = rear_x + LASER_X - FRONT
    return max(0.0, (gap - params.follow_min_gap) / params.follow_time_gap)


def test_shipped_config_has_it_off_and_lists_every_field_at_its_default():
    params = yaml.safe_load(open(PACKAGED_CONFIG))['gap_follow_node']['ros__parameters']
    assert params['enable_opponent_awareness'] is False
    defaults = opponent_tracking.OpponentParams()
    for name in opponent_tracking.OpponentParams.__dataclass_fields__:
        assert params[f'opponent_{name}'] == getattr(defaults, name), name


def test_off_builds_no_layer_and_reads_none_of_its_values():
    # Values the layer would refuse: with it off they must not even be read.
    node, published = _make('-p', 'opponent_alpha:=5.0', '-p', 'opponent_gate:=-1.0')
    try:
        assert node.opponents is None
        _drive(node, published, ticks=5)
        assert node.opponent_mode is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_on_with_a_bad_value_refuses_to_start():
    rclpy.init(args=['--ros-args', '--params-file', PACKAGED_CONFIG,
                     '-p', 'enable_opponent_awareness:=true',
                     '-p', 'opponent_alpha:=5.0', *TEST_ONLY_DRIVE])
    try:
        with pytest.raises(ValueError, match=r'opponent_alpha must be in \(0, 1\]'):
            GapFollowNode()
    finally:
        rclpy.shutdown()


def _speeds(*extra, **kw):
    node, published = _make(*extra)
    try:
        return _drive(node, published, **kw), node.opponent_mode
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_on_follows_a_slower_car_ahead_at_the_follow_cap():
    on, mode = _speeds('-p', 'enable_opponent_awareness:=true')
    off, _ = _speeds()
    assert mode == 'follow'       # 2 m corridor, car dead ahead: no room to pass
    # Once the track is confirmed (8 consecutive scans) and its velocity has
    # settled, the published speed is held to the closed-form follow cap.
    # 0.1 m/s allows for the alpha-beta filter's residual position lag.
    late = range(25, 40)
    for k in late:
        rear_x, _y, speed = on[k]
        assert speed <= _closed_form_cap(rear_x) + 0.1, (k, speed, _closed_form_cap(rear_x))
    # And the cap is what made the difference: the same scenes without the
    # layer command more on every one of those ticks.
    assert all(off[k][2] > on[k][2] + 0.2 for k in late)


def test_on_without_scan_stamps_does_nothing():
    # Unstamped scans give no valid interval, so the tracker resets every
    # scan and never confirms a mover: the commands match the layer off.
    on, mode = _speeds('-p', 'enable_opponent_awareness:=true', stamped=False)
    off, _ = _speeds()
    assert mode is None
    assert [s for _x, _y, s in on] == pytest.approx([s for _x, _y, s in off], abs=1e-9)


def test_on_in_a_static_world_commands_exactly_what_off_commands():
    # Nothing moves: a parked car ahead, seen from a car that is moving.
    # Static means its ground position is fixed, so in the scan it closes at
    # the ego speed -- OPP_SPEED = 0 in _drive's terms.
    global OPP_SPEED
    saved, OPP_SPEED = OPP_SPEED, 0.0
    try:
        on, mode = _speeds('-p', 'enable_opponent_awareness:=true', ticks=30, rear_x0=3.5)
        off, _ = _speeds(ticks=30, rear_x0=3.5)
    finally:
        OPP_SPEED = saved
    assert mode is None
    assert [s for _x, _y, s in on] == pytest.approx([s for _x, _y, s in off], abs=1e-9)


def test_lb_released_still_commands_zero_with_a_car_ahead():
    on, _mode = _speeds('-p', 'enable_opponent_awareness:=true', lb=False, ticks=20)
    assert all(speed == 0.0 for _x, _y, speed in on)


def test_a_turning_car_off_to_the_side_is_followed_not_passed():
    # Drifting left at 0.55 m/s while doing 1.5 m/s: heading 20 deg, over
    # opponent_pass_max_heading_deg (15), so no pass -- and once it is more
    # than follow_lateral (0.45 m) to the side, only the widened follow band
    # (relevant_lateral, 1.0 m) still caps the speed.
    on, mode = _speeds('-p', 'enable_opponent_awareness:=true', y0=0.2, vy=0.55)
    assert mode == 'follow'
    wide = [(rear_x, speed) for rear_x, y, speed in on[20:] if 0.45 < y + 0.0 <= 1.0]
    assert len(wide) >= 5
    for rear_x, speed in wide:
        assert speed <= _closed_form_cap(rear_x) + 0.1, (rear_x, speed)


def test_tracking_uses_the_scan_stamps_not_a_fixed_rate():
    # An 80 Hz scanner (stamps 12.5 ms apart) passing a parked car. Taking
    # the interval as anything but the stamps' difference mis-integrates this
    # car's own motion, and the parked car would appear to drive off.
    global OPP_SPEED
    saved, OPP_SPEED = OPP_SPEED, 0.0
    try:
        on, mode = _speeds('-p', 'enable_opponent_awareness:=true', ticks=60,
                           rear_x0=3.0, dt=0.0125)
    finally:
        OPP_SPEED = saved
    assert mode is None
