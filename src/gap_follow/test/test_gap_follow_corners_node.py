"""
Node-level wiring tests for swept-path clearance, the arc guard and corner
setup (docs/gap-follow.md). The math is tested unsourced in
test_gap_logic_corners.py; these check that GapFollowNode feeds it the right
arrays and parameters and publishes what it returns. Needs ROS sourced:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/gap_follow/test/test_gap_follow_corners_node.py -v

Scenes are ray-cast from line segments in the LiDAR frame, so every expected
number traces to the scene geometry.
"""
import math
import os

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, LaserScan

from gap_follow import gap_logic
from gap_follow.gap_follow_node import GapFollowNode

# Same rule as test_gap_follow_node.py: last in every rclpy.init, never the
# real /drive (enforced by test_no_test_here_builds_a_node_on_the_real_drive).
TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']
# The shipped configuration: these features are tuned for it, and the
# constructor's own declared defaults are not what the car runs.
PACKAGED_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'gap_follow.yaml')
CAR = ['--params-file', PACKAGED_CONFIG]
N_BEAMS = 1081
SPAN = math.radians(270.0)


# Every scene here is a car mid-course meeting a corner, not a race start. The
# launch boost (shipped on since 2026-10-30) holds the car straight on its first
# ticks while the straight path is clear, so it is off here
# (test_launch_boost_node.py covers it).
MID_RACE = ['-p', 'enable_launch_boost:=false']


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', *CAR, *MID_RACE, *TEST_ONLY_DRIVE])
    n = GapFollowNode()
    assert n.drive_pub.topic_name == '/test_only/drive'
    # Slew is tested elsewhere; here it would only hide the steering decision.
    n.max_steering_rate = 100.0
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _raycast(segments, max_range=10.0):
    msg = LaserScan()
    msg.angle_min = -SPAN / 2.0
    msg.angle_increment = SPAN / (N_BEAMS - 1)
    msg.angle_max = msg.angle_min + (N_BEAMS - 1) * msg.angle_increment
    msg.range_min, msg.range_max = 0.05, 12.0
    angles = msg.angle_min + np.arange(N_BEAMS) * msg.angle_increment
    ranges = np.full(N_BEAMS, max_range)
    for (x1, y1), (x2, y2) in segments:
        ex, ey = x2 - x1, y2 - y1
        for i, a in enumerate(angles):
            dx, dy = math.cos(a), math.sin(a)
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-12:
                continue
            t = (x1 * ey - y1 * ex) / denom
            u = (x1 * dy - y1 * dx) / denom
            if t > 0.0 and 0.0 <= u <= 1.0:
                ranges[i] = min(ranges[i], t)
    msg.ranges = [float(r) for r in ranges]
    return msg


def _corridor(left, right, length=20.0):
    return [((-5.0, left), (length, left)), ((-5.0, -right), (length, -right))]


def _left_turn(left, right, corner_x, width=None):
    """Corridor along +x that turns 90deg left at corner_x (inner corner
    at (corner_x, left)); the new corridor runs up +y."""
    width = left + right if width is None else width
    outer_x = corner_x + width
    return [((-5.0, left), (corner_x, left)), ((corner_x, left), (corner_x, 8.0)),
            ((-5.0, -right), (outer_x, -right)), ((outer_x, -right), (outer_x, 8.0))]


def _mirror(segments):
    return [((x1, -y1), (x2, -y2)) for (x1, y1), (x2, y2) in segments]


def _ready(node, speed):
    joy = Joy()
    joy.buttons = [0] * 5
    joy.buttons[4] = 1
    node.joy_callback(joy)
    odom = Odometry()
    odom.twist.twist.linear.x = float(speed)
    node.odom_callback(odom)
    node.last_commanded_speed = float(speed)


def _drive(node, scan, speed):
    """One tick; returns the published drive command. The decision text the
    node built for it lands in node.recorded_details (newest last)."""
    published = []
    original = node.drive_pub.publish
    node.drive_pub.publish = lambda msg: (published.append(msg), original(msg))
    _ready(node, speed)
    node.scan_callback(scan)
    node.drive_pub.publish = original
    assert published, 'node published nothing'
    return published[-1].drive


def _record_details(node):
    """Keep every decision detail the node formats, then log as normal. The
    log itself is throttled to state changes, so this is the only way to see
    a steady state's text from a test."""
    node.recorded_details = []
    original = node._log_decision

    def recording(state, detail, *args, **kwargs):
        node.recorded_details.append(detail() if callable(detail) else detail)
        return original(state, detail, *args, **kwargs)
    node._log_decision = recording


def _forward_cone_ceiling(node, scan):
    """Oracle for the old braking ceiling: the forward-cone minimum fed to
    the same stopping-distance formula (both independently unit-tested)."""
    clean, valid = gap_logic.sanitize_ranges(scan.ranges, node.max_range, scan.range_min)
    lo, hi = node._fov_indices(scan)
    angles = scan.angle_min + np.arange(lo, hi + 1) * scan.angle_increment
    boundaries = gap_logic.vehicle_boundary_distances(
        angles, node.car_width, node.car_length, node.wheelbase, node.laser_offset_x)
    cone = gap_logic.minimum_footprint_clearance_in_cone(
        clean[lo:hi + 1], valid[lo:hi + 1], angles, boundaries, node.forward_stop_fov)
    return gap_logic.braking_speed_limit(
        cone, node.forward_stop_clearance, node.max_braking_decel, node.max_speed)


# ============================================================================
# Swept-path clearance: the braking ceiling on a narrow straight
# ============================================================================

def test_swept_clearance_lets_the_car_past_the_old_cone_ceiling_on_a_narrow_straight(node):
    scan = _raycast(_corridor(0.75, 0.75))
    old_ceiling = _forward_cone_ceiling(node, scan)
    assert old_ceiling < 2.6          # the side walls were the "obstacle"
    cmd = _drive(node, scan, speed=3.0)
    assert cmd.speed > old_ceiling + 0.3


def test_without_swept_clearance_the_cone_ceiling_still_binds(node):
    node.enable_swept_clearance = False
    scan = _raycast(_corridor(0.75, 0.75))
    cmd = _drive(node, scan, speed=3.0)
    assert cmd.speed <= _forward_cone_ceiling(node, scan) + 1e-9


def test_swept_clearance_still_brakes_for_a_wall_in_the_path(node):
    # The refusal half (A8). A wall across the corridor 1.5 m ahead of the
    # LiDAR: swept clearance = 1.5 + laser_offset_x - (wheelbase/2 +
    # car_length/2), and the ceiling is the stopping-distance formula on it.
    wall = 1.5
    scan = _raycast(_corridor(0.75, 0.75, length=wall) + [((wall, -0.75), (wall, 0.75))])
    clearance = wall + node.laser_offset_x - (node.wheelbase / 2.0 + node.car_length / 2.0)
    ceiling = math.sqrt(2.0 * node.max_braking_decel
                        * (clearance - node.forward_stop_clearance))
    cmd = _drive(node, scan, speed=3.0)
    assert cmd.speed <= ceiling + 1e-6


# ============================================================================
# Corner setup: move to the outside before a turn
# ============================================================================

def _setup_pair(node, segments, speed=2.0):
    # Corner setup is the feature under test. It ships OFF since 2026-10-27
    # (docs/gap-follow.md#race-defaults-2026-10-27), so turn it on here rather
    # than rely on the shipped default.
    node.enable_corner_setup = True
    node.enable_arc_guard = False      # isolate the setup term
    scan = _raycast(segments)
    with_setup = _drive(node, scan, speed).steering_angle
    node.enable_corner_setup = False
    node.previous_target_idx = None
    without = _drive(node, scan, speed).steering_angle
    return with_setup, without


def test_corner_setup_holds_the_car_wide_before_a_left_turn(node):
    # Centred in a 1.4 m hallway, a square left corner 1.5 m ahead. The gap
    # aim alone leans slightly *right* here (the inflated inner corner clips
    # the gap's left edge), so this is also the case the side-opening turn
    # signal exists for: setup must still read "left turn" and push right.
    with_setup, without = _setup_pair(node, _left_turn(0.7, 0.7, corner_x=1.5))
    assert with_setup < without
    assert without - with_setup <= node.corner_setup_max_steering + 1e-9


def test_corner_setup_is_mirror_symmetric_for_a_right_turn(node):
    left_with, left_without = _setup_pair(node, _left_turn(0.7, 0.7, corner_x=1.5))
    node.enable_corner_setup = True
    node.previous_target_idx = None
    right_with, right_without = _setup_pair(
        node, _mirror(_left_turn(0.7, 0.7, corner_x=1.5)))
    assert right_with > right_without
    assert right_with - right_without == pytest.approx(
        left_without - left_with, abs=1e-6)


def test_corner_setup_is_off_far_from_the_corner(node):
    # 6 m out the end wall is beyond corner_setup_far_zero: no setup yet.
    with_setup, without = _setup_pair(node, _left_turn(0.7, 0.7, corner_x=6.0))
    assert with_setup == pytest.approx(without, abs=1e-12)


def test_corner_setup_does_nothing_on_a_straight(node):
    with_setup, without = _setup_pair(node, _corridor(0.7, 0.7))
    assert with_setup == pytest.approx(without, abs=1e-12)


def test_corner_setup_is_reported_in_the_decision_log(node):
    node.enable_corner_setup = True    # off by default since 2026-10-27
    node.enable_arc_guard = False
    _record_details(node)
    _drive(node, _raycast(_left_turn(0.7, 0.7, corner_x=1.5)), speed=2.0)
    assert 'corner setup' in node.recorded_details[-1]


# ============================================================================
# Arc guard: never steer the body into an inner corner
# ============================================================================

def test_arc_guard_backs_off_a_turn_whose_arc_is_blocked(node):
    # Hugging one wall (0.25 m) with a square corner 0.6 m ahead: the hard
    # turn the gap aim asks for sweeps the body into a wall. Without the
    # guard the swept-path ceiling reads 0 m and the car stops dead -- a
    # latch, since nothing then changes the steering. With it, the car takes
    # the nearest arc that is clear and keeps going.
    node.enable_corner_setup = False
    scan = _raycast(_left_turn(0.25, 1.15, corner_x=0.6))
    node.enable_arc_guard = False
    unguarded = _drive(node, scan, speed=0.5)
    requested = unguarded.steering_angle
    node.enable_arc_guard = True
    node.previous_target_idx = None
    _record_details(node)
    guarded_cmd = _drive(node, scan, speed=0.5)
    guarded = guarded_cmd.steering_angle

    clean, valid = gap_logic.sanitize_ranges(scan.ranges, node.max_range, scan.range_min)
    lo, hi = node._fov_indices(scan)
    angles = scan.angle_min + np.arange(lo, hi + 1) * scan.angle_increment

    def free(steering):
        return gap_logic.swept_path_clearance(
            clean[lo:hi + 1], valid[lo:hi + 1], angles,
            math.tan(steering) / node.wheelbase,
            node.car_width / 2.0 + node.arc_guard_margin, node.car_length,
            node.wheelbase, node.laser_offset_x, 0.0, 10.0)

    assert free(requested) < node.arc_guard_length, 'precondition: requested arc is blocked'
    assert unguarded.speed == 0.0
    assert guarded_cmd.speed > 0.0
    assert abs(guarded) < abs(requested)
    assert math.copysign(1.0, guarded) == math.copysign(1.0, requested)
    assert abs(requested - guarded) <= node.arc_guard_max_correction + 1e-9
    assert free(guarded) > free(requested)
    assert 'arc guard' in node.recorded_details[-1]


def test_arc_guard_leaves_an_open_corridor_alone(node):
    scan = _raycast(_corridor(1.0, 1.0))
    node.enable_arc_guard = False
    requested = _drive(node, scan, speed=0.5).steering_angle
    node.enable_arc_guard = True
    node.previous_target_idx = None
    assert _drive(node, scan, speed=1.5).steering_angle == pytest.approx(requested, abs=1e-12)


# ============================================================================
# Startup validation: the authority bounds
# ============================================================================

@pytest.mark.parametrize('params, match', [
    (['-p', 'arc_guard_max_correction:=0.2'], 'arc_guard_max_correction'),
    (['-p', 'corner_setup_max_steering:=0.2'], 'corner_setup_max_steering'),
    (['-p', 'corner_setup_near_zero:=1.7'], 'corner setup distances'),
    (['-p', 'swept_clearance_margin:=0.5'], 'swept_clearance_margin'),
    (['-p', 'arc_guard_margin:=-0.1'], 'arc_guard_margin'),
    (['-p', 'corner_setup_fraction:=1.5'], 'corner_setup_fraction'),
])
def test_refuses_to_start_with_an_unsafe_configuration(params, match):
    rclpy.init(args=['--ros-args', '-p', 'max_steering_angle:=0.349', *params,
                     *TEST_ONLY_DRIVE])
    try:
        with pytest.raises(ValueError, match=match):
            GapFollowNode()
    finally:
        rclpy.shutdown()


def test_disabled_features_are_not_validated():
    # Off means off: a feature that is switched off cannot stop the node
    # starting because of its own (unused) knobs.
    rclpy.init(args=['--ros-args', '-p', 'max_steering_angle:=0.349',
                     '-p', 'enable_arc_guard:=false',
                     '-p', 'arc_guard_max_correction:=0.3', *TEST_ONLY_DRIVE])
    try:
        node = GapFollowNode()
        node.destroy_node()
    finally:
        rclpy.shutdown()


# ============================================================================
# Cases the mutation pass showed were missing
# ============================================================================

def test_arc_guard_never_corrects_past_its_bound(node):
    # Same blocked scene. Every arc within 0.10 rad of the request sweeps the
    # body into the wall; the nearest clear one is 0.125 rad away. With the
    # bound at 0.10 the guard must not reach for it -- it keeps its best
    # effort inside the bound (here the request itself, all being equal).
    node.enable_corner_setup = False
    scan = _raycast(_left_turn(0.25, 1.15, corner_x=0.6))
    node.enable_arc_guard = False
    requested = _drive(node, scan, speed=0.5).steering_angle
    node.enable_arc_guard = True
    node.arc_guard_max_correction = 0.10
    node.previous_target_idx = None
    guarded = _drive(node, scan, speed=0.5).steering_angle

    clean, valid = gap_logic.sanitize_ranges(scan.ranges, node.max_range, scan.range_min)
    lo, hi = node._fov_indices(scan)
    angles = scan.angle_min + np.arange(lo, hi + 1) * scan.angle_increment
    clear_just_outside = gap_logic.swept_path_clearance(
        clean[lo:hi + 1], valid[lo:hi + 1], angles,
        math.tan(requested + math.copysign(0.125, -requested)) / node.wheelbase,
        node.car_width / 2.0 + node.arc_guard_margin, node.car_length,
        node.wheelbase, node.laser_offset_x, 0.0, 10.0)
    assert clear_just_outside >= node.arc_guard_length, 'precondition'
    assert abs(guarded - requested) <= 0.10 + 1e-9


def test_corner_setup_reads_the_turn_from_an_off_centre_car(node):
    # Sitting on the inside (0.5 m from the left wall, 0.9 m from the right)
    # before a square left corner: each side must be judged against its own
    # wall distance, so this is still a left turn and setup pushes right.
    with_setup, without = _setup_pair(node, _left_turn(0.5, 0.9, corner_x=1.2))
    assert with_setup < without - 0.01


def test_swept_clearance_brakes_for_the_arc_the_rack_is_still_on(node):
    # The rack is hard left (steering_basis) and the gap now asks for straight.
    # The slew limiter only lets the rack come back max_steering_rate*dt this
    # tick, so the car is still sweeping a left arc toward the left wall --
    # and the braking ceiling must be the shorter of the two arcs' clearances.
    node.max_steering_rate = 1.0
    node.enable_arc_guard = False
    node.enable_corner_setup = False
    scan = _raycast(_corridor(0.75, 0.75))
    node.steering_basis = node.max_steering_angle
    cmd = _drive(node, scan, speed=3.0)
    assert cmd.steering_angle > 0.1, 'precondition: the rack is still turning left'

    clean, valid = gap_logic.sanitize_ranges(scan.ranges, node.max_range, scan.range_min)
    lo, hi = node._fov_indices(scan)
    angles = scan.angle_min + np.arange(lo, hi + 1) * scan.angle_increment
    arc_clearance = gap_logic.swept_path_clearance(
        clean[lo:hi + 1], valid[lo:hi + 1], angles,
        math.tan(cmd.steering_angle) / node.wheelbase,
        node.car_width / 2.0 + node.swept_clearance_margin, node.car_length,
        node.wheelbase, node.laser_offset_x, 0.0, node.swept_clearance_max_length)
    ceiling = gap_logic.braking_speed_limit(
        arc_clearance, node.forward_stop_clearance, node.max_braking_decel, node.max_speed)
    assert ceiling < node.max_speed, 'precondition: the left wall is on that arc'
    assert cmd.speed <= ceiling + 1e-9
