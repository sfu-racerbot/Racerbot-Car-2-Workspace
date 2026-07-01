"""
Integration tests for pure_pursuit_node's reactive safety net and
opponent-overtake logic, exercising the *real* rclpy node rather than
just racing_math's pure functions (see test_racing_math.py for those).

Unlike test_racing_math.py, this file needs ROS2 sourced and the
pure_pursuit package built/importable -- it is not meant to run via a
bare `pytest` with nothing else set up:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/pure_pursuit/test/test_opponent_integration.py -v

Each test drives the node through pose_callback/scan_callback/
control_loop() directly (no actual topics/executor involved) with the
deadman check disabled via a parameter override, since these tests are
specifically about the racing-line/avoidance/overtake logic, not the
already-covered deadman gate itself.
"""
import math
import os

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from pure_pursuit.pure_pursuit_node import PurePursuitNode
from pure_pursuit import racing_math

@pytest.fixture(scope='module')
def profiled_csv(tmp_path_factory):
    """Profile the checked-in example track once for the whole test module."""
    src = os.path.join(os.path.dirname(__file__), '..', 'waypoints', 'example_stadium_raw.csv')
    xy = racing_math.load_xy_csv(src)
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    curvature = racing_math.estimate_path_curvature(xy, closed=True)
    speed = racing_math.compute_velocity_profile(
        seg_len, curvature, v_max=6.0, v_min=0.5, a_lat_max=8.0, a_accel_max=3.0, a_brake_max=8.0, closed=True)
    out_path = str(tmp_path_factory.mktemp('waypoints') / 'profiled.csv')
    racing_math.save_profiled_csv(out_path, xy, speed)
    return out_path


@pytest.fixture
def node(profiled_csv):
    """A real PurePursuitNode against the profiled example track, in its
    own rclpy context per test (parameter overrides -- like the profiled
    CSV path, and disabling the deadman gate -- have to be supplied as
    `rclpy.init(args=...)` at construction time; they can't be poked in
    afterward as if they were plain attributes). The deadman gate is
    disabled here because these tests are specifically about the
    racing-line/avoidance/overtake logic, which is independent of (and
    already covered separately from) that gate.
    """
    rclpy.init(args=['--ros-args', '-p', f'waypoints_file:={profiled_csv}', '-p', 'enable_deadman:=false'])
    n = PurePursuitNode()
    yield n
    n.destroy_node()
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


def _car_ahead_scan(car_range=2.0, left_room=9.9, right_room=9.6):
    """A scan with a single, correctly car-sized cluster dead ahead, and
    (deliberately non-object, i.e. still 'open track') differing amounts
    of room on either side of it, for testing pass-side selection."""
    scan = _clear_scan()
    n = len(scan.ranges)
    center = n // 2
    for i in range(center - 9, center + 9):
        scan.ranges[i] = car_range
    for i in range(center + 15, center + 65):
        scan.ranges[i] = left_room
    for i in range(center - 65, center - 15):
        scan.ranges[i] = right_room
    return scan, center


def _capture_published(node):
    published = []
    original = node.drive_pub.publish
    node.drive_pub.publish = lambda msg: (published.append(msg), original(msg))
    return published


def test_drives_normally_on_a_clear_track(node):
    published = _capture_published(node)
    _set_pose(node, -1.5, -1.2, 0.0)
    node.scan_callback(_clear_scan())
    node.control_loop()
    last = published[-1]
    assert abs(last.drive.steering_angle) < 0.05
    assert last.drive.speed > 0.5
    assert node.overtake_active is False


def test_overtakes_toward_the_more_open_side(node):
    published = _capture_published(node)
    scan, _center = _car_ahead_scan(left_room=9.9, right_room=9.6)
    node.scan_callback(scan)
    for step in range(4):
        _set_pose(node, -1.5 + step * 0.2, -1.2, 0.0)
        node.control_loop()
    last = published[-1]
    assert node.overtake_active is True
    assert node.overtake_side == 1  # more open on the left -> pass left
    assert last.drive.steering_angle > 0.0


def test_overtakes_toward_the_right_when_thats_more_open(node):
    published = _capture_published(node)
    scan, _center = _car_ahead_scan(left_room=9.6, right_room=9.9)
    node.scan_callback(scan)
    for step in range(4):
        _set_pose(node, -1.5 + step * 0.2, -1.2, 0.0)
        node.control_loop()
    last = published[-1]
    assert node.overtake_active is True
    assert node.overtake_side == -1
    assert last.drive.steering_angle < 0.0


def test_hard_stop_overrides_an_active_overtake(node):
    published = _capture_published(node)
    scan, center = _car_ahead_scan()
    node.scan_callback(scan)
    for step in range(4):
        _set_pose(node, -1.5 + step * 0.2, -1.2, 0.0)
        node.control_loop()
    assert node.overtake_active is True  # sanity check before the real assertion

    close_scan = _clear_scan()
    close_scan.ranges[center] = 0.15
    node.scan_callback(close_scan)
    node.control_loop()
    last = published[-1]
    assert last.drive.speed == 0.0, "the hard-stop safety net must win regardless of overtake state"


def test_overtake_resolves_once_ego_is_past_the_opponent(node):
    scan, _center = _car_ahead_scan()
    node.scan_callback(scan)
    for step in range(4):
        _set_pose(node, -1.5 + step * 0.2, -1.2, 0.0)
        node.control_loop()
    assert node.overtake_active is True

    # Opponent now out of view (behind the car, as it would be after a
    # real pass) -- resolution must use the last tracked position, not
    # wait for opponent_lost_timeout_sec to just give up.
    node.scan_callback(_clear_scan())
    _set_pose(node, 1.5, -1.2, 0.0)
    for _ in range(3):
        node.control_loop()
    assert node.overtake_active is False
