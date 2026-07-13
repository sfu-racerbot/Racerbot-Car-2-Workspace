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

import numpy as np
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


def test_recovers_after_a_localization_jump(node):
    # The nearest-waypoint search is windowed (+/-40) around last tick's
    # index. A particle-filter re-convergence can legally teleport the
    # pose to a completely different part of the track -- outside that
    # window -- and the node must re-lock with a global search and keep
    # driving, not stop until someone restarts it.
    published = _capture_published(node)
    node.scan_callback(_clear_scan())
    _set_pose(node, -1.5, -1.2, 0.0)   # bottom straight, near waypoint ~3
    node.control_loop()
    assert published[-1].drive.speed > 0.0  # sanity: driving normally

    # Localization jumps to the *top* straight (~50 waypoints away,
    # outside the +/-40 window), heading along the track there.
    _set_pose(node, 2.0, 1.2, math.pi)
    node.control_loop()
    assert published[-1].drive.speed > 0.0, \
        "a clean pose elsewhere on the track must re-lock via global search, not stop forever"


def test_stays_stopped_when_genuinely_lost(node):
    # The recovery above must not weaken the actual watchdog: a pose far
    # from the track everywhere is still "lost", still a stop.
    published = _capture_published(node)
    node.scan_callback(_clear_scan())
    _set_pose(node, 20.0, 20.0, 0.0)
    node.control_loop()
    assert published[-1].drive.speed == 0.0


def test_overtake_survives_losing_sight_of_the_opponent_mid_pass(node):
    # Mid-pass, alongside the opponent, it is *guaranteed* to be outside
    # the forward detection cone -- typically for longer than
    # opponent_lost_timeout_sec (1.0s). The old logic canceled the
    # overtake right then and snapped the steering target back onto the
    # racing line the opponent was still occupying.
    scan, _center = _car_ahead_scan()
    node.scan_callback(scan)
    for step in range(4):
        _set_pose(node, -1.5 + step * 0.2, -1.2, 0.0)
        node.control_loop()
    assert node.overtake_active is True

    node.scan_callback(_clear_scan())          # opponent out of the cone
    node.opponent.last_update_time -= 2.0      # ...and for 2s already (> lost timeout, < blind cap)
    _set_pose(node, -0.7, -1.2, 0.0)           # ego still clearly *behind* the opponent
    node.control_loop()
    assert node.overtake_active is True, \
        "losing sight mid-pass must not cancel the pass while still behind the opponent"


def test_overtake_aborts_after_too_long_blind(node):
    # The counterpart cap: blind for longer than overtake_max_blind_sec
    # and the dead-reckoned opponent position can't be trusted anymore --
    # holding the offset line on a stale guess is worse than giving up.
    scan, _center = _car_ahead_scan()
    node.scan_callback(scan)
    for step in range(4):
        _set_pose(node, -1.5 + step * 0.2, -1.2, 0.0)
        node.control_loop()
    assert node.overtake_active is True

    node.scan_callback(_clear_scan())
    node.opponent.last_update_time -= 5.0      # well past overtake_max_blind_sec (3.0)
    _set_pose(node, -0.7, -1.2, 0.0)
    node.control_loop()
    assert node.overtake_active is False


# ============================================================================
# Map-subtraction detection mode
# ============================================================================

@pytest.fixture
def map_mode_node(profiled_csv):
    """Same as `node`, but with opponent_detection_mode:=map."""
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'enable_deadman:=false',
                     '-p', 'opponent_detection_mode:=map'])
    n = PurePursuitNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _synthetic_map(size_px=400, resolution=0.05, origin=-10.0):
    """A 20x20m free arena with 2-cell walls on the border -- big enough
    that the whole example stadium track sits in mapped-free space."""
    from nav_msgs.msg import OccupancyGrid
    grid = np.zeros((size_px, size_px), dtype=np.int8)
    grid[:2, :] = 100
    grid[-2:, :] = 100
    grid[:, :2] = 100
    grid[:, -2:] = 100
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = resolution
    msg.info.width = size_px
    msg.info.height = size_px
    msg.info.origin.position.x = origin
    msg.info.origin.position.y = origin
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.flatten().tolist()
    return msg


def test_map_mode_falls_back_to_heuristic_until_a_map_arrives(map_mode_node):
    # 'map' mode with no map yet must not race blind: the heuristic
    # detector keeps working, so this behaves exactly like the plain
    # overtake test above.
    node = map_mode_node
    assert node.map_ray_caster is None
    scan, _center = _car_ahead_scan()
    node.scan_callback(scan)
    for step in range(4):
        _set_pose(node, -1.5 + step * 0.2, -1.2, 0.0)
        node.control_loop()
    assert node.overtake_active is True


def test_map_mode_overtakes_using_map_subtraction(map_mode_node):
    # Full pipeline with a real ray caster over a synthetic map: the
    # walls are in the map (never "an opponent"), the car-sized cluster
    # is not (always one) -- and the overtake triggers off it end to end.
    pytest.importorskip('range_libc')
    node = map_mode_node
    node.map_callback(_synthetic_map())
    assert node.map_ray_caster is not None, "map_callback must build the ray caster"

    scan, _center = _car_ahead_scan(left_room=9.9, right_room=9.6)
    node.scan_callback(scan)
    for step in range(4):
        _set_pose(node, -1.5 + step * 0.2, -1.2, 0.0)
        node.control_loop()
    assert node.overtake_active is True
    assert node.overtake_side == 1


def test_map_subtraction_ignores_what_the_map_explains(map_mode_node):
    # A clear scan of the mapped arena leaves no residual -- nothing to
    # detect, no matter how the walls curve. (This is the wall
    # false-positive immunity the heuristic can't offer.)
    pytest.importorskip('range_libc')
    node = map_mode_node
    node.map_callback(_synthetic_map())
    _set_pose(node, -1.5, -1.2, 0.0)

    scan = _clear_scan()
    ranges = np.clip(np.nan_to_num(np.array(scan.ranges), nan=0.0, posinf=node.max_range),
                     0.0, node.max_range)
    detection = node._detect_opponent(scan, ranges, node.car_x + 0.27, node.car_y)
    assert detection is None


def test_map_subtraction_detects_an_unmapped_car_directly(map_mode_node):
    pytest.importorskip('range_libc')
    node = map_mode_node
    node.map_callback(_synthetic_map())
    _set_pose(node, -1.5, -1.2, 0.0)

    scan, center = _car_ahead_scan(car_range=2.0)
    ranges = np.clip(np.nan_to_num(np.array(scan.ranges), nan=0.0, posinf=node.max_range),
                     0.0, node.max_range)
    detection = node._detect_opponent(scan, ranges, node.car_x + 0.27, node.car_y)
    assert detection is not None
    start_idx, end_idx, centroid_range, centroid_angle = detection
    assert centroid_range == pytest.approx(2.0, abs=0.05)
    assert abs(centroid_angle) < 0.15
    # Indices must come back in *full-scan* space (downsampling mapped back).
    assert center - 12 <= start_idx <= center
    assert center <= end_idx <= center + 12
