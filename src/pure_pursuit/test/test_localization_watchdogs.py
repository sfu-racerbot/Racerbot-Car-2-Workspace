"""
Regression tests for pure_pursuit_node's localization watchdogs.

These exist because of a real crash on 2026-07-27: during the automatic
map-to-race handover, slam_toolbox's blocking map/pose-graph save stalled
the map->odom transform, auto_map_race_node kept faithfully republishing
that frozen transform to /slam_pose at its usual 40Hz, and pure pursuit --
whose only staleness test was "when did a message last arrive" -- drove for
a full second from a position the car had already left, into a wall.

Two independent defences are tested here, either of which catches it:
  1. Age judged from the pose's own header stamp, not its arrival time.
  2. A pose that does not move while odometry insists the car does.

Needs ROS2 sourced and pure_pursuit importable:
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/pure_pursuit/test/test_localization_watchdogs.py -v
"""
import math
import os
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan

from pure_pursuit import racing_math
from pure_pursuit.pure_pursuit_node import PurePursuitNode


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


@pytest.fixture
def node(profiled_csv):
    # drive_topic is remapped away from the real /drive: these nodes run with
    # the deadman disabled, so an un-remapped test would publish live
    # commands straight into ackermann_mux if the driver stack were up.
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'enable_deadman:=false',
                     '-p', 'drive_topic:=/test_only/drive'])
    n = PurePursuitNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def quick_frozen_node(profiled_csv):
    """Same node, but with a 50ms frozen-pose window instead of the real
    500ms one. The watchdog measures elapsed wall-clock time, and a test
    loop runs its ticks in microseconds rather than at the real 40Hz, so
    shortening the window (and sleeping for real) exercises the actual
    timing comparison without a slow test."""
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'enable_deadman:=false',
                     '-p', 'drive_topic:=/test_only/drive',
                     '-p', 'pose_frozen_timeout_sec:=0.05'])
    n = PurePursuitNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _clear_scan(n=361, angle_span=math.pi):
    scan = LaserScan()
    scan.angle_min = -angle_span / 2.0
    scan.angle_increment = angle_span / n
    scan.angle_max = scan.angle_min + (n - 1) * scan.angle_increment
    scan.range_min, scan.range_max = 0.05, 12.0
    scan.ranges = [10.0] * n
    return scan


def _publish_pose(node, x, y, yaw=0.0, stamp=None):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    if stamp is not None:
        msg.header.stamp = stamp.to_msg()
    node.pose_callback(msg)


def _set_odom(node, speed):
    msg = Odometry()
    msg.twist.twist.linear.x = float(speed)
    node.odom_callback(msg)


def _capture(node):
    published = []
    original = node.drive_pub.publish
    node.drive_pub.publish = lambda msg: (published.append(msg), original(msg))
    return published


def test_frozen_pose_stops_the_car_although_messages_keep_arriving(quick_frozen_node):
    """The 2026-07-27 signature: pose republished punctually but never
    advancing, while odometry reports the car accelerating."""
    node = quick_frozen_node
    published = _capture(node)
    node.scan_callback(_clear_scan())

    # Drive normally first, so we know the car is otherwise happy.
    _set_odom(node, 1.25)
    _publish_pose(node, -1.5, -1.2, 0.0)
    node.control_loop()
    assert published[-1].drive.speed > 0.0
    assert node.last_decision_state == 'pure_pursuit'

    # Now localization stalls. The publisher does not: the identical pose
    # keeps arriving, freshly stamped, exactly as a republished stale TF
    # would. Odometry continues to report real motion.
    for _ in range(30):
        _set_odom(node, 1.25)
        _publish_pose(node, -1.5, -1.2, 0.0, stamp=node.get_clock().now())
        node.scan_callback(_clear_scan())
        node.control_loop()
        if node.last_decision_state == 'pose_frozen':
            break
        time.sleep(0.01)

    assert node.last_decision_state == 'pose_frozen', (
        'a pose that never advances while odometry reports motion must stop '
        'the car, however punctually it arrives')
    assert published[-1].drive.speed == 0.0


def test_frozen_watchdog_recovers_when_localization_resumes(quick_frozen_node):
    """A stall must stop the car, not latch it off forever -- once the
    pose starts advancing again the car is free to drive."""
    node = quick_frozen_node
    published = _capture(node)
    for _ in range(30):
        _set_odom(node, 1.25)
        _publish_pose(node, -1.5, -1.2, 0.0, stamp=node.get_clock().now())
        node.scan_callback(_clear_scan())
        node.control_loop()
        if node.last_decision_state == 'pose_frozen':
            break
        time.sleep(0.01)
    assert node.last_decision_state == 'pose_frozen'

    # Localization catches up and starts tracking again.
    for step in range(5):
        _set_odom(node, 1.25)
        _publish_pose(node, -1.4 + step * 0.1, -1.2, 0.0,
                      stamp=node.get_clock().now())
        node.scan_callback(_clear_scan())
        node.control_loop()

    assert node.last_decision_state == 'pure_pursuit'
    assert published[-1].drive.speed > 0.0


def test_a_parked_car_does_not_trip_the_frozen_pose_watchdog(node):
    """The pose is equally motionless when the car is simply stopped --
    that must not be mistaken for a localization failure."""
    published = _capture(node)
    for _ in range(40):
        _set_odom(node, 0.0)          # genuinely stationary
        _publish_pose(node, -1.5, -1.2, 0.0, stamp=node.get_clock().now())
        node.scan_callback(_clear_scan())
        node.control_loop()

    assert node.last_decision_state != 'pose_frozen'
    assert published[-1].drive.speed > 0.0, 'the car should still be free to pull away'


def test_a_moving_car_with_tracking_localization_is_left_alone(node):
    """The watchdog must not fire while localization is doing its job."""
    published = _capture(node)
    for step in range(40):
        _set_odom(node, 1.25)
        _publish_pose(node, -1.5 + step * 0.05, -1.2, 0.0,
                      stamp=node.get_clock().now())
        node.scan_callback(_clear_scan())
        node.control_loop()

    assert node.last_decision_state != 'pose_frozen'
    assert published[-1].drive.speed > 0.0


def test_stale_header_stamp_is_caught_even_when_the_message_just_arrived(node):
    """Second, independent defence: the message is brand new, but the pose
    inside it was computed long ago -- which is precisely what
    republishing an old transform produces."""
    published = _capture(node)
    node.scan_callback(_clear_scan())
    old = node.get_clock().now() - Duration(seconds=5.0)
    _publish_pose(node, -1.5, -1.2, 0.0, stamp=old)
    node.control_loop()

    assert node.last_decision_state == 'pose_stale'
    assert published[-1].drive.speed == 0.0


def test_unstamped_poses_still_drive(node):
    """Publishers that leave header.stamp at zero (the default-constructed
    message) must fall back to arrival time rather than being treated as
    infinitely old, which would park the car permanently."""
    published = _capture(node)
    node.scan_callback(_clear_scan())
    _publish_pose(node, -1.5, -1.2, 0.0)   # no stamp set at all
    node.control_loop()

    assert node.last_decision_state == 'pure_pursuit'
    assert published[-1].drive.speed > 0.0


def _full_sweep_scan(n=1081, angle_span=math.radians(270.0)):
    """A scan shaped like this car's actual Hokuyo: a 270deg sweep, so the
    rearmost beams look back along the car."""
    scan = LaserScan()
    scan.angle_min = -angle_span / 2.0
    scan.angle_increment = angle_span / (n - 1)
    scan.angle_max = scan.angle_min + (n - 1) * scan.angle_increment
    scan.range_min, scan.range_max = 0.023, 30.0
    scan.ranges = [10.0] * n
    return scan


def test_rear_beams_hitting_the_cars_own_chassis_do_not_pin_it(node):
    """Regression test for the 2026-07-27 standstill.

    The body-clearance check originally ran over all 270deg. The rearmost
    beams see the car's own structure, which is inside the footprint by
    construction, so clearance read a permanent -0.110m and pure pursuit
    refused to move at all. Restricting to a 180deg forward window fixes
    it without giving up the lateral coverage the check exists for.
    """
    published = _capture(node)
    scan = _full_sweep_scan()
    n = len(scan.ranges)
    # Returns from the car's own bodywork, directly behind the LiDAR.
    for i in list(range(0, 40)) + list(range(n - 40, n)):
        scan.ranges[i] = 0.30
    node.scan_callback(scan)
    _publish_pose(node, -1.5, -1.2, 0.0)
    node.control_loop()

    assert node.last_decision_state != 'body_contact', (
        "the car's own chassis behind the LiDAR must not read as contact")
    assert published[-1].drive.speed > 0.0


def test_a_wall_on_the_flank_is_still_caught_inside_the_window(node):
    """The narrowed window must not cost the lateral coverage that is the
    entire reason the body check exists."""
    published = _capture(node)
    scan = _full_sweep_scan()
    n = len(scan.ranges)
    center = n // 2
    # +/-90deg lies exactly at the edge of the 180deg window; put the wall
    # just inside it, hard against the car's left flank.
    per_rad = 1.0 / scan.angle_increment
    left = center + int(math.radians(80.0) * per_rad)
    for i in range(left - 10, left + 10):
        scan.ranges[i] = 0.16
    node.scan_callback(scan)
    _publish_pose(node, -1.5, -1.2, 0.0)
    node.control_loop()

    assert node.last_decision_state == 'body_contact'
    assert published[-1].drive.speed == 0.0
