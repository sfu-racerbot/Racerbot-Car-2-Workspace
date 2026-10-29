"""
Node-level tests for the 2026-10-20 changes measured on the real car
(docs/gap-follow.md#the-2026-10-19-run): the TTC path margin, braking
latency, speed-scheduled steering gain and the speed hold. The math is
tested unsourced in test_gap_logic.py; these check GapFollowNode applies it
and publishes what it returns. Needs ROS sourced:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/gap_follow/test/test_gap_follow_speed_shaping_node.py -v
"""
import math
import os
import sys

import pytest
import rclpy

from gap_follow import gap_logic
from gap_follow.gap_follow_node import GapFollowNode

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_gap_follow_corners_node import (  # noqa: E402
    CAR, TEST_ONLY_DRIVE, _corridor, _drive, _raycast)


def _make(extra=()):
    # TEST_ONLY_DRIVE last, after the params file and any override, so the
    # YAML's drive_topic: /drive can never win (CLAUDE.md).
    rclpy.init(args=['--ros-args', *CAR, *extra, *TEST_ONLY_DRIVE])
    n = GapFollowNode()
    assert n.drive_pub.topic_name == '/test_only/drive'
    n.max_steering_rate = 100.0
    return n


@pytest.fixture
def node():
    n = _make()
    yield n
    n.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def old_margin_node():
    # The TTC corridor as it was before 2026-10-20: the full safety_margin.
    n = _make(['-p', 'ttc_path_margin:=0.18'])
    yield n
    n.destroy_node()
    rclpy.shutdown()


# A straight corridor whose left wall steps in to 0.32 m from the path for a
# metre ahead, like the inside of the bends on the 2026-10-19 run: 0.165 m
# outside the 0.155 m half-body, inside the old 0.335 m TTC corridor but
# outside the new 0.205 m one. The right wall is far enough to keep that
# step the closest thing in view.
SIDE_STEP = _corridor(0.75, 0.75) + [((0.35, 0.32), (1.4, 0.32)), ((0.35, 0.32), (0.35, 0.75))]
SPEED = 2.4  # m/s, as on the run


def test_scene_is_the_old_flank_trip(old_margin_node):
    """Oracle check for the next test: with the old corridor this exact scene
    must hard-stop on TTC, or the next test proves nothing."""
    drive = _drive(old_margin_node, _raycast(SIDE_STEP), SPEED)
    assert old_margin_node.last_decision_state == 'ttc_brake'
    assert drive.speed == 0.0


def test_ttc_does_not_brake_for_a_wall_the_car_is_passing(node):
    drive = _drive(node, _raycast(SIDE_STEP), SPEED)
    assert node.last_decision_state != 'ttc_brake'
    assert drive.speed > 1.0, 'a clear path beside a wall must not be braked to a crawl'


def test_ttc_still_brakes_for_an_obstacle_in_the_path(node):
    """The deny half (A8): a post dead ahead, inside the narrowed corridor,
    still trips TTC. 0.45 m from the LiDAR at 2.4 m/s is well inside the
    0.30 s threshold; with open space either side it is a directional trip,
    so the car either stops or crawls toward the gap -- never drives on."""
    post = [((0.45, -0.05), (0.45, 0.05))]
    drive = _drive(node, _raycast(_corridor(1.5, 1.5) + post), SPEED)
    assert node.last_decision_state in ('ttc_brake', 'ttc_escape')
    assert drive.speed <= node.escape_creep_speed + 1e-9


def test_ttc_margin_covers_just_outside_the_body(node):
    """The margin itself, not just the body: a post 0.025 m outside the body
    edge (inside the 0.05 m ttc_path_margin) is still braked for."""
    edge = node.car_width / 2.0 + 0.025
    post = [((0.45, edge), (0.45, edge + 0.02))]
    drive = _drive(node, _raycast(_corridor(1.5, 1.5) + post), SPEED)
    assert node.last_decision_state in ('ttc_brake', 'ttc_escape')
    assert drive.speed <= node.escape_creep_speed + 1e-9


@pytest.mark.parametrize('params, match', [
    (['-p', 'ttc_path_margin:=0.19'], 'ttc_path_margin'),   # one step past safety_margin
    (['-p', 'ttc_path_margin:=-0.01'], 'ttc_path_margin'),
    (['-p', 'braking_latency_sec:=-0.1'], 'braking_latency_sec'),
    (['-p', 'braking_latency_sec:=1.5'], 'braking_latency_sec'),
    (['-p', 'steering_gain_speed_ref:=-1.0'], 'steering_gain_speed_ref'),
    (['-p', 'speed_hold_sec:=2.5'], 'speed_hold_sec'),
])
def test_refuses_out_of_range_speed_shaping(params, match):
    rclpy.init(args=['--ros-args', *CAR, *params, *TEST_ONLY_DRIVE])
    try:
        with pytest.raises(ValueError, match=match):
            GapFollowNode()
    finally:
        rclpy.shutdown()


END_X = 2.5  # m from the LiDAR: a closed dead end, straight ahead
DEAD_END = [((-5.0, 0.45), (END_X, 0.45)), ((-5.0, -0.45), (END_X, -0.45)),
            ((END_X, -0.45), (END_X, 0.45))]


def test_braking_latency_lowers_the_clearance_ceiling(node):
    """A dead end 2.5 m ahead down a straight 0.9 m corridor: the gap is
    straight ahead, so the published speed is the latency-aware
    stopping-distance ceiling, checked against its closed form."""
    node.max_lateral_accel = 100.0          # keep the curvature cap out of it
    node.max_acceleration = 100.0           # and the ramp
    node.speed_hold_sec = 0.0
    drive = _drive(node, _raycast(DEAD_END), 1.0)
    assert abs(drive.steering_angle) < 0.02, 'scene must keep the car straight'
    # Room to the body's front along a straight arc: wall x from base_link
    # minus the body's front (wheelbase/2 + car_length/2).
    front = node.wheelbase / 2.0 + node.car_length / 2.0
    room = END_X + node.laser_offset_x - front
    a, t, r = node.max_braking_decel, node.braking_latency_sec, node.forward_stop_clearance
    expected = -a * t + math.sqrt((a * t) ** 2 + 2.0 * a * (room - r))
    assert t > 0.0
    assert drive.speed == pytest.approx(min(expected, node.max_speed), abs=0.02)  # m/s
    assert drive.speed < gap_logic.braking_speed_limit(room, r, a, node.max_speed)


def test_speed_hold_keeps_a_dip_for_the_hold_time(node):
    """After one tick whose ceiling is low, the next ticks on an open
    straight stay at that speed until speed_hold_sec of command time has
    passed, then rise again. The dip is made by lowering max_speed for one
    tick, so nothing but the hold can explain the held ticks."""
    from rclpy.duration import Duration
    # 1/32 s ticks and a 0.25 s hold: both exact in binary, so the release
    # tick is exactly 0.25 s after the dip rather than a rounding error away.
    dt = 1.0 / 32.0
    node.speed_hold_sec = 0.25
    node.speed_hold = gap_logic.SpeedHold(0.25)
    node.command_slew_max_dt = dt
    # The release tick must not be ramp-limited. At 100 m/s^2 the ramp reaches
    # only 1.2 + 100/32 = 4.3 m/s on that tick, which was under the old 3.5
    # top speed but is under the 5.0 shipped since 2026-10-27. At 1000 it
    # reaches 32 m/s, so the top speed is what the release has to reach.
    node.max_acceleration = 1000.0
    open_road = _raycast(_corridor(1.5, 1.5))
    top = node.max_speed

    def tick():
        node.last_command_time = node.get_clock().now() - Duration(seconds=dt)
        return _drive(node, open_road, node.last_commanded_speed).speed

    node.last_commanded_speed = top
    assert tick() == pytest.approx(top, abs=1e-9)   # open road runs at max_speed
    node.max_speed = 1.2
    low = tick()
    node.max_speed = top
    held = [tick() for _ in range(7)]   # up to 7/32 s after the dip: inside 0.25 s
    assert low == pytest.approx(1.2, abs=1e-9)
    assert held == pytest.approx([1.2] * 7, abs=1e-9)
    assert tick() == pytest.approx(top, abs=1e-9)   # exactly 8/32 = 0.25 s on: released


def test_speed_hold_is_forgotten_by_a_stop(node):
    """A stop (here LB released) must clear the hold: the car resuming must
    not be held down by a dip from before it stopped."""
    from rclpy.duration import Duration
    from sensor_msgs.msg import Joy
    dt = 1.0 / 32.0
    node.speed_hold_sec = 0.25
    node.speed_hold = gap_logic.SpeedHold(0.25)
    node.command_slew_max_dt = dt
    node.max_acceleration = 100.0
    open_road = _raycast(_corridor(1.5, 1.5))
    top = node.max_speed
    node.last_commanded_speed = top
    node.max_speed = 1.2
    node.last_command_time = node.get_clock().now() - Duration(seconds=dt)
    assert _drive(node, open_road, top).speed == pytest.approx(1.2, abs=1e-9)
    node.max_speed = top
    released = Joy()
    released.buttons = [0] * 5
    node.joy_callback(released)
    node.scan_callback(open_road)                 # deadman stop
    node.last_command_time = node.get_clock().now() - Duration(seconds=dt)
    assert _drive(node, open_road, top).speed == pytest.approx(top, abs=1e-9)


def test_arc_guard_blocked_fallback_is_validated():
    rclpy.init(args=['--ros-args', *CAR, '-p', 'arc_guard_blocked_fallback:=nearest',
                     *TEST_ONLY_DRIVE])
    try:
        with pytest.raises(ValueError, match='arc_guard_blocked_fallback'):
            GapFollowNode()
    finally:
        rclpy.shutdown()


def _sector_scan(center_deg, half_deg, near=1.6, far=9.0):
    """Wall at `near` everywhere except one deep sector: the only gap, its
    midpoint at `center_deg` (not touching the FOV edge, so the node aims at
    the midpoint)."""
    msg = _raycast([])
    angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
    msg.ranges = [far if abs(math.degrees(a) - center_deg) <= half_deg else near
                  for a in angles]
    return msg


def test_steering_gain_falls_with_speed_above_the_reference(node):
    """Same gap at 1.0 and 3.0 m/s: below the reference the wheels point at
    the gap; at 3.0 m/s they turn ref/3.0 of that (closed form of
    gap_logic.speed_scheduled_gain)."""
    node.enable_centering = False
    node.enable_corner_setup = False
    node.enable_arc_guard = False
    node.steering_gain_speed_ref = 1.8
    # +-26 deg: the disparity extender trims ~12 deg off each edge at 1.6 m.
    scan = _sector_scan(8.0, 26.0)
    slow = _drive(node, scan, 1.0).steering_angle
    node.previous_target_idx = None
    fast = _drive(node, scan, 3.0).steering_angle
    assert slow == pytest.approx(node.steering_gain * math.radians(8.0), abs=0.01)  # rad
    assert fast == pytest.approx(slow * 1.8 / 3.0, abs=0.005)  # rad


def test_arc_guard_blocked_fallback_reaches_the_guard(node):
    """Hugging the wall before a square left corner at walking pace, a
    0.2 rad request with a 0.1 rad bound has no clear arc in reach.
    'freest' moves off the request (the precondition that makes this a test),
    'request' keeps it. The request is set to 0.2 rad by scaling
    steering_gain, with every other steering term off."""
    from test_gap_follow_corners_node import _left_turn
    scan = _raycast(_left_turn(0.25, 1.15, corner_x=0.6))
    node.enable_centering = False
    node.enable_corner_setup = False
    node.steering_gain_speed_ref = 0.0
    node.enable_arc_guard = False
    unscaled = _drive(node, scan, speed=0.5).steering_angle
    assert unscaled != 0.0
    # Whatever the gap asks for, scale it to exactly +0.2 rad: toward the
    # wall's corner, where no arc within 0.1 rad is clear (rightward arcs are,
    # so the sign matters). The gain may come out negative; this test is
    # about the guard's wiring, not about the gap's choice.
    node.steering_gain = 0.2 / unscaled
    node.enable_arc_guard = True
    node.arc_guard_max_correction = 0.1
    out = {}
    for mode in ('freest', 'request'):
        node.arc_guard_blocked_fallback = mode
        node.previous_target_idx = None
        out[mode] = _drive(node, scan, speed=0.5).steering_angle
    assert out['request'] == pytest.approx(0.2, abs=1e-3)  # rad; freest lands ~0.075 away
    assert abs(out['freest'] - 0.2) > 0.03, 'precondition: freest must move here'


def _straight_on(node, scan, speed):
    """One tick at `speed` with acceleration unlimited and the steering
    terms that would bend a straight corridor switched off, so only the
    speed ceilings decide the command."""
    node.enable_centering = False
    node.enable_corner_setup = False
    node.enable_arc_guard = False
    node.max_acceleration = 100.0
    node.previous_target_idx = None
    return _drive(node, scan, speed)


def test_narrow_corridor_cap_slows_the_car_beside_a_close_wall(node):
    """Body 0.10 m from both walls (symmetric, so the gap is dead ahead and
    no curvature cap competes), halfway between the 0.05 and 0.15 m ends of
    the ramp: the ceiling is the closed form
    narrow + 0.5 * (max - narrow). With the cap off the same scene runs
    faster, so it is this cap, not another, that binds."""
    node.narrow_corridor_speed = 2.5
    node.narrow_corridor_clearance = 0.05
    node.open_corridor_clearance = 0.15
    half = node.car_width / 2.0 + 0.10
    scan = _raycast(_corridor(half, half))
    capped = _straight_on(node, scan, node.max_speed).speed
    expected = 2.5 + 0.5 * (node.max_speed - 2.5)
    assert capped == pytest.approx(expected, abs=0.02)  # m/s; the wall estimate is a beam minimum
    node.narrow_corridor_speed = node.max_speed
    assert _straight_on(node, scan, node.max_speed).speed > expected + 0.2


def test_narrow_corridor_cap_leaves_an_open_corridor_alone(node):
    """0.40 m each side, well past open_corridor_clearance: no slowdown."""
    scan = _raycast(_corridor(node.car_width / 2.0 + 0.40, node.car_width / 2.0 + 0.40))
    with_cap = _straight_on(node, scan, node.max_speed).speed
    node.narrow_corridor_speed = node.max_speed
    assert _straight_on(node, scan, node.max_speed).speed == pytest.approx(with_cap, abs=1e-9)


@pytest.mark.parametrize('params, match', [
    (['-p', 'narrow_corridor_speed:=-0.1'], 'narrow_corridor_speed'),
    (['-p', 'narrow_corridor_clearance:=0.2'], 'narrow_corridor_clearance'),
    (['-p', 'narrow_corridor_clearance:=-0.01'], 'narrow_corridor_clearance'),
])
def test_narrow_corridor_parameters_are_validated(params, match):
    rclpy.init(args=['--ros-args', *CAR, *params, *TEST_ONLY_DRIVE])
    try:
        with pytest.raises(ValueError, match=match):
            GapFollowNode()
    finally:
        rclpy.shutdown()
