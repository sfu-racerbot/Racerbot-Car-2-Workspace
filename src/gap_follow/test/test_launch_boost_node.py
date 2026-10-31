"""
Node-level tests for gap_follow's launch boost (docs/gap-follow.md#launch-boost).

Spec: gap_follow/launch_boost.py's docstring and the YAML comment above
enable_launch_boost. The state machine is tested unsourced in
test_launch_boost.py; these check that GapFollowNode wires it in: the first LB
press publishes the braking/curvature ceiling with no ramp, opponents are
ignored, the deadman refuses all four ways, a release never relaunches, and TTC
still ends the launch. Oracles: closed forms recomputed here (the ramp
max_acceleration * dt, the shipped max_speed on an open straight) and the same
scene with a layer off. Needs ROS sourced:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/gap_follow/test/test_launch_boost_node.py -v
"""
import math
import os
import sys

import pytest
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import Joy

from gap_follow import launch_boost

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_gap_follow_corners_node import _corridor, _raycast  # noqa: E402
from test_opponent_awareness_node import (  # noqa: E402
    DT, PACKAGED_CONFIG, _box, _closed_form_cap)
from test_opponent_awareness_node import _make as _make_mid_race  # noqa: E402

def _make(*extra):
    """The shipped config with the launch boost on, as the car runs it (the
    opponent tests' helper turns it off for their mid-race scenes)."""
    return _make_mid_race('-p', 'enable_launch_boost:=true', *extra)


OPEN_ROAD = _raycast(_corridor(1.0, 1.0))   # 2 m wide, 20 m long: nothing binds


def _joy(node, held=True):
    joy = Joy()
    joy.buttons = [0] * 5
    joy.buttons[4] = 1 if held else 0
    node.joy_callback(joy)


def _odom(node, speed):
    odom = Odometry()
    odom.twist.twist.linear.x = float(speed)
    node.odom_callback(odom)


def _tick(node, published, scan, speed=0.0, held=True, joy=True):
    """One scan with fresh odometry at ``speed`` and, unless ``joy`` is False,
    a fresh /joy. The command interval is exactly DT (command_slew_max_dt)."""
    if joy:
        _joy(node, held)
    _odom(node, speed)
    node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
    node.scan_callback(scan)
    return published[-1].drive


@pytest.fixture
def made():
    node, published = _make()
    yield node, published
    node.destroy_node()
    rclpy.shutdown()


def test_shipped_config_has_it_on():
    # oracle: spec -- requested on every race start (2026-10-30).
    params = yaml.safe_load(open(PACKAGED_CONFIG))['gap_follow_node']['ros__parameters']
    assert params['enable_launch_boost'] is True


# --- The four deadman states (CLAUDE.md, standing requirement) -------------

def test_lb_held_first_press_publishes_full_speed_with_no_ramp(made):
    node, published = made
    drive = _tick(node, published, OPEN_ROAD, speed=0.0)
    # oracle: closed form -- on an open straight both ceilings are max_speed,
    # and without the launch the ramp would allow only max_acceleration * DT.
    assert drive.speed == pytest.approx(node.max_speed, abs=1e-9)  # m/s
    assert node.max_acceleration * DT < 0.2 * node.max_speed
    assert node.launch_boost.active


def test_without_the_launch_the_first_press_is_ramp_limited():
    # The half above means nothing unless the same tick is ramped with it off.
    node, published = _make('-p', 'enable_launch_boost:=false')
    try:
        drive = _tick(node, published, OPEN_ROAD, speed=0.0)
        assert drive.speed == pytest.approx(node.max_acceleration * DT, abs=1e-9)  # m/s
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_lb_released_publishes_zero_and_keeps_the_launch_armed(made):
    node, published = made
    assert _tick(node, published, OPEN_ROAD, held=False).speed == 0.0
    assert node.launch_boost.state == launch_boost.ARMED
    # The first real press still launches.
    assert _tick(node, published, OPEN_ROAD).speed == pytest.approx(node.max_speed, abs=1e-9)


def test_joy_never_received_publishes_zero_and_keeps_the_launch_armed(made):
    node, published = made
    assert _tick(node, published, OPEN_ROAD, joy=False).speed == 0.0
    assert node.launch_boost.state == launch_boost.ARMED


def test_joy_stale_publishes_zero_and_keeps_the_launch_armed(made):
    node, published = made
    _joy(node, held=True)
    node.last_joy_time = node.get_clock().now() - Duration(
        seconds=node.joy_timeout_sec + 0.05)
    assert _tick(node, published, OPEN_ROAD, joy=False).speed == 0.0
    assert node.launch_boost.state == launch_boost.ARMED


def test_joy_stale_mid_launch_publishes_zero_and_ends_it(made):
    node, published = made
    _tick(node, published, OPEN_ROAD)
    node.last_joy_time = node.get_clock().now() - Duration(
        seconds=node.joy_timeout_sec + 0.05)
    assert _tick(node, published, OPEN_ROAD, joy=False).speed == 0.0
    assert node.launch_boost.state == launch_boost.DONE
    assert node.launch_boost.end_reason == 'joy_stale'


# --- First press only ---------------------------------------------------------

def test_a_stop_on_the_first_lb_tick_does_not_spend_the_launch(made):
    # LB held, but no odometry yet: TTC refuses to drive ('odometry_stale').
    # The launch is spent by the first tick that drives, not by the press.
    node, published = made
    _joy(node, held=True)
    node.scan_callback(OPEN_ROAD)
    assert published[-1].drive.speed == 0.0
    assert node.launch_boost.state == launch_boost.ARMED
    assert _tick(node, published, OPEN_ROAD).speed == pytest.approx(node.max_speed, abs=1e-9)


@pytest.mark.parametrize('rear_x', [0.5, 0.35])
def test_a_blocked_grid_does_not_start_the_launch(made, rear_x):
    # A car-sized box dead ahead: at 0.5 m the braking ceiling is 0 (no room
    # to speed up), at 0.35 m the tick creeps. Neither is a launch, and the
    # launch stays armed for when the way is clear.
    node, published = made
    blocked = _raycast(_corridor(1.0, 1.0) + _box(rear_x))
    drive = _tick(node, published, blocked)
    assert drive.speed <= node.escape_creep_speed
    assert node.launch_boost.state == launch_boost.ARMED
    assert _tick(node, published, OPEN_ROAD).speed == pytest.approx(node.max_speed, abs=1e-9)

def test_release_and_press_again_is_ramped_not_launched(made):
    node, published = made
    assert _tick(node, published, OPEN_ROAD).speed == pytest.approx(node.max_speed, abs=1e-9)
    assert _tick(node, published, OPEN_ROAD, held=False).speed == 0.0
    assert node.launch_boost.end_reason == 'deadman_released'
    again = _tick(node, published, OPEN_ROAD, speed=0.0)
    # oracle: closed form -- the stop left the last command at 0, so the ramp.
    assert again.speed == pytest.approx(node.max_acceleration * DT, abs=1e-9)  # m/s


# --- What the launch ignores -------------------------------------------------

def test_narrow_corridor_cap_is_not_applied(made):
    node, published = made
    # Body 0.10 m from both walls: the narrow-corridor ceiling is the closed
    # form narrow + 0.5 * (max - narrow), below max_speed.
    half = node.car_width / 2.0 + 0.10
    narrow_cap = node.narrow_corridor_speed + 0.5 * (node.max_speed - node.narrow_corridor_speed)
    assert narrow_cap < node.max_speed - 0.2
    drive = _tick(node, published, _raycast(_corridor(half, half)), speed=0.0)
    assert drive.speed == pytest.approx(node.max_speed, abs=1e-9)  # m/s


def _start_beside_a_departing_car(*extra, ticks=30):
    """Ego on the grid (odometry 0), another car 3 m ahead pulling away at
    1.5 m/s in a 2 m straight. Returns [(rear_x, published speed, mode)]."""
    node, published = _make(*extra)
    try:
        out = []
        for k in range(ticks):
            rear_x = 3.0 + 1.5 * k * DT
            scan = _raycast(_corridor(1.0, 1.0) + _box(rear_x))
            t = 100.0 + k * DT
            scan.header.stamp.sec = int(t)
            scan.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
            node.steering_basis = 0.0
            drive = _tick(node, published, scan, speed=0.0)
            out.append((rear_x, drive.speed, node.opponent_mode))
        return out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_opponents_are_ignored_during_the_launch():
    aware = _start_beside_a_departing_car('-p', 'enable_opponent_awareness:=true')
    blind = _start_beside_a_departing_car('-p', 'enable_opponent_awareness:=false')
    following = _start_beside_a_departing_car('-p', 'enable_opponent_awareness:=true',
                                              '-p', 'enable_launch_boost:=false')
    late = range(20, 30)
    # oracle: "ignored" means exactly what the node commands with the layer off.
    assert [s for _x, s, _m in aware] == pytest.approx(
        [s for _x, s, _m in blind], abs=1e-9)
    assert all(aware[k][2] == 'launch' for k in late)
    # And the scene does exercise the follow cap: without the launch it binds
    # (closed form, 0.1 m/s for the tracker's lag, as in
    # test_on_follows_a_slower_car_ahead_at_the_follow_cap), and the launch
    # commands well above it on the same ticks.
    for k in late:
        cap = _closed_form_cap(following[k][0])
        assert following[k][1] <= cap + 0.1, (k, following[k])
        assert aware[k][1] > cap + 0.2, (k, aware[k], cap)


# --- What the launch never ignores --------------------------------------------

def test_ttc_still_trips_and_ends_the_launch(made):
    node, published = made
    assert _tick(node, published, OPEN_ROAD).speed == pytest.approx(node.max_speed, abs=1e-9)
    # A car-sized box 0.6 m dead ahead while the car does 2 m/s: TTC
    # (0.6 + 0.26 - 0.47) / 2 = 0.2 s, under the 0.3 s threshold.
    drive = _tick(node, published, _raycast(_corridor(1.0, 1.0) + _box(0.6)), speed=2.0)
    assert drive.speed <= node.escape_creep_speed
    assert node.launch_boost.state == launch_boost.DONE
    assert node.launch_boost.end_reason in ('ttc_brake', 'ttc_escape',
                                            'emergency_clearance', 'emergency_escape')


def test_launch_ends_when_the_braking_ceiling_reaches_the_car(made):
    node, published = made
    _tick(node, published, OPEN_ROAD)
    # A wall 2.0 m ahead of the LiDAR with the car already at 4 m/s: the
    # braking ceiling is below 4 m/s, so the car must now slow.
    dead_end = _raycast(_corridor(1.0, 1.0, length=2.0) + [((2.0, -1.0), (2.0, 1.0))])
    drive = _tick(node, published, dead_end, speed=4.0)
    assert node.launch_boost.end_reason == 'braking'
    assert drive.speed < 4.0


# --- Holding straight past a car alongside --------------------------------------

# A car level with us 0.55 m to the left, its nose 0.88 m ahead of the LiDAR:
# it fills the left of the forward window, so the gap logic aims right of it.
NOSE_AHEAD = _raycast(_corridor(1.0, 1.0) + _box(0.3, y=0.55))


def test_a_car_alongside_does_not_steer_the_launch(made):
    node, published = made
    drive = _tick(node, published, NOSE_AHEAD)
    # oracle: spec (user, 2026-10-30) -- keep going straight and ignore it.
    # No IMU here, so the damping term is exactly zero and straight is 0 rad.
    assert drive.steering_angle == 0.0
    assert drive.speed == pytest.approx(node.max_speed, abs=1e-9)  # m/s
    assert node.launch_boost.active


def test_without_the_launch_the_same_car_steers_us_away():
    # The half above means nothing unless this scene really does pull the car
    # off line: gap steering aims away, held to the slew bound on one tick.
    node, published = _make('-p', 'enable_launch_boost:=false')
    try:
        drive = _tick(node, published, NOSE_AHEAD)
        assert drive.steering_angle == pytest.approx(
            -node.max_steering_rate * DT, abs=1e-9)   # rad, slew-limited, away (right)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_a_car_cutting_into_our_path_hands_back_to_gap_steering(made):
    node, published = made
    # Rear face 1.0 m ahead, centred 0.15 m left: inside the straight band
    # (half width 0.155 + 0.05 m margin), within the 1.0 m arc_guard_length.
    drive = _tick(node, published, _raycast(_corridor(1.0, 1.0) + _box(1.0, y=0.15)))
    assert drive.steering_angle == pytest.approx(
        -node.max_steering_rate * DT, abs=1e-9)   # rad: steering round it, to the right
    assert drive.speed < node.max_speed


# --- The ceilings the launch keeps (added on car 2, 2026-10-30) -----------------
# "Braking and curvature always apply" is the launch's safety claim, and a
# mutation run showed nothing above would notice either ceiling being dropped:
# the tests above end the launch at the ceiling, or never reach one.

def test_the_braking_ceiling_caps_the_launch_command(made):
    node, published = made
    # A wall 2.5 m ahead of the LiDAR, inside swept_clearance_max_length, with
    # the car at rest: the launch starts, and may only ask for the braking root.
    wall = 2.5
    dead_end = _raycast(_corridor(1.0, 1.0, length=wall) + [((wall, -1.0), (wall, 1.0))])
    drive = _tick(node, published, dead_end, speed=0.0)
    # oracle: closed form, gap_logic.braking_speed_limit's docstring, recomputed
    # here. The body's front is (car_length + wheelbase) / 2 from base_link.
    front = (node.car_length + node.wheelbase) / 2.0
    room = wall + node.laser_offset_x - front - node.forward_stop_clearance
    a_t = node.max_braking_decel * node.braking_latency_sec
    root = -a_t + math.sqrt(a_t ** 2 + 2.0 * node.max_braking_decel * room)
    assert node.launch_boost.active
    assert root < node.max_speed - 1.0
    assert drive.speed == pytest.approx(root, abs=1e-3)  # m/s


def test_the_curvature_ceiling_caps_the_launch_command():
    # A wall across the road 1.2 m ahead, open on the left: the straight path is
    # blocked, so gap steering turns hard. Anticipation is off and the slew is
    # out of the way, so the curvature cap alone is what binds.
    node, published = _make('-p', 'enable_cornering_anticipation:=false')
    try:
        node.max_steering_rate = 100.0
        drive = _tick(node, published,
                      _raycast(_corridor(3.0, 1.0) + [((1.2, -1.0), (1.2, 0.6))]))
        assert node.launch_boost.active
        # oracle: closed form, v = sqrt(a_lat * L / tan(steering)): about 1.57
        # m/s at car 2's 0.349 rad and 2.5 m/s^2 (gap_follow.yaml).
        assert drive.steering_angle == pytest.approx(node.max_steering_angle, abs=1e-9)  # rad
        assert drive.speed == pytest.approx(math.sqrt(
            node.max_lateral_accel * node.wheelbase
            / math.tan(node.max_steering_angle)), abs=1e-3)  # m/s
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_an_emergency_escape_ends_the_launch(made):
    node, published = made
    _tick(node, published, OPEN_ROAD)
    # A thin obstacle touching the padded body dead ahead, room to either side:
    # the directional emergency tier escapes rather than stopping outright.
    drive = _tick(node, published,
                  _raycast(_corridor(1.0, 1.0) + [((0.205, -0.01), (0.205, 0.01))]))
    assert node.launch_boost.end_reason == 'emergency_escape'
    assert drive.speed <= node.escape_creep_speed
