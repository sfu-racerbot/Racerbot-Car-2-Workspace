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
from rclpy.duration import Duration
from rclpy.parameter import Parameter

from pure_pursuit.pure_pursuit_node import OpponentTracker, PurePursuitNode
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
def node_factory(profiled_csv):
    """Build more than one identically configured node in one context.

    Needed by the pass-side tests, which are comparisons: "steers further
    right *than it would have* without the opponent" is the claim, and a
    comparison needs a baseline run from the same starting state.
    """
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'enable_deadman:=false',
                     '-p', 'drive_topic:=/test_only/drive'])
    built = []

    def make():
        node = PurePursuitNode()
        built.append(node)
        return node

    yield make
    for node in built:
        node.destroy_node()
    rclpy.shutdown()


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
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'enable_deadman:=false',
                     '-p', 'drive_topic:=/test_only/drive'])
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


def _one_command_interval(node):
    """Make the next control_loop() integrate a realistic time step.

    Back-to-back control_loop() calls are microseconds apart, and
    max_steering_rate (1.0 rad/s) then permits about 0.0001 rad of steering
    change per call. Four of those leaves the command within a thousandth
    of a radian of zero, and a test asserting its *sign* is really
    asserting how long each call took -- which flips with machine load.
    Rewinding last_command_time by one command_slew_max_dt gives the slew
    limiter the same budget it gets on the car, so what is measured is the
    pass-side decision rather than the scheduler. (Same idiom as
    test_shaped_speed_ramps_up_to_the_profiled_speed.)
    """
    node.last_command_time = node.get_clock().now() - Duration(
        seconds=node.command_slew_max_dt)


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
    # This is the very first command, so it starts from a standstill and the
    # acceleration limit -- not the racing line -- sets it: exactly one
    # command_slew_max_dt step's worth of ramp.
    assert last.drive.speed == pytest.approx(
        node.max_acceleration * node.command_slew_max_dt)
    assert last.drive.speed > 0.5, \
        "the first command must still be a real drive command, not a crawl"
    assert node.overtake_active is False
    assert node.last_decision_state == 'pure_pursuit'


def test_shaped_speed_ramps_up_to_the_profiled_speed(node):
    # The acceleration limit shapes how fast a command may *rise*. It must
    # converge on the profiled speed rather than hold the car below it: an
    # over-tight ramp is what stalled the car behind traffic in simulation.
    # Each loop is spaced one full command interval apart, as a real one is.
    published = _capture_published(node)
    node.scan_callback(_clear_scan())
    _set_pose(node, -1.5, -1.2, 0.0)
    for _ in range(12):
        node.last_command_time = node.get_clock().now() - Duration(
            seconds=node.command_slew_max_dt)
        node.control_loop()

    speeds = [msg.drive.speed for msg in published]
    assert all(later >= earlier - 1e-9
               for earlier, later in zip(speeds, speeds[1:])), speeds
    assert speeds[-1] > speeds[0], "the ramp must actually let the car speed up"
    assert speeds[-1] == pytest.approx(speeds[-2]), \
        "the ramp must converge, leaving the racing line in charge of speed"
    assert speeds[-1] > 1.0


def test_missing_lidar_stop_has_a_diagnostic_reason(node):
    published = _capture_published(node)
    _set_pose(node, -1.5, -1.2, 0.0)
    node.control_loop()
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'lidar_scan_missing'


def _drive_four_ticks(node, scan):
    """Four control ticks from the same start, returning the last command."""
    published = _capture_published(node)
    node.scan_callback(scan)
    for step in range(4):
        _set_pose(node, -1.5 + step * 0.2, -1.2, 0.0)
        _one_command_interval(node)
        node.control_loop()
    return published[-1]


def test_overtakes_toward_the_more_open_side(node_factory):
    node = node_factory()
    scan, _center = _car_ahead_scan(left_room=9.9, right_room=9.6)
    _drive_four_ticks(node, scan)
    assert node.overtake_active is True
    assert node.overtake_side == 1  # more open on the left -> pass left
    assert node.last_decision_state == 'overtake_left'


def test_overtakes_toward_the_right_when_thats_more_open(node_factory):
    node = node_factory()
    scan, _center = _car_ahead_scan(left_room=9.6, right_room=9.9)
    _drive_four_ticks(node, scan)
    assert node.overtake_active is True
    assert node.overtake_side == -1
    assert node.last_decision_state == 'overtake_right'


def test_the_chosen_pass_side_actually_moves_the_steering_that_way(node_factory):
    """Passing right must steer further right than passing left, from the
    same pose against the same opponent.

    Stated as a comparison between the two sides rather than as the sign of
    either one. The overtake offsets a point `overtake_lookahead_distance`
    (4m) ahead by `overtake_lateral_offset` (0.35m), which is about five
    degrees of bearing -- a nudge on top of whatever the racing line is
    already doing over those four metres, not a replacement for it. On this
    fixture's track the line's own curvature is the larger term, so *both*
    passes come out with a positive steering angle and an assertion on the
    sign of one of them is an assertion about the test track.

    It used to pass anyway: back-to-back control_loop() calls are
    microseconds apart, max_steering_rate then allowed about 0.0001 rad of
    change per call, and the command stayed pinned near zero where its sign
    was noise. Adding a few microseconds of work per tick flipped it.
    """
    left_node = node_factory()
    left_scan, _ = _car_ahead_scan(left_room=9.9, right_room=9.6)
    left = _drive_four_ticks(left_node, left_scan)

    right_node = node_factory()
    right_scan, _ = _car_ahead_scan(left_room=9.6, right_room=9.9)
    right = _drive_four_ticks(right_node, right_scan)

    assert left_node.overtake_side == 1
    assert right_node.overtake_side == -1
    assert right.drive.steering_angle < left.drive.steering_angle


def _set_odom(node, speed):
    from nav_msgs.msg import Odometry
    msg = Odometry()
    msg.twist.twist.linear.x = float(speed)
    node.odom_callback(msg)


def test_acceleration_ramp_starts_from_measured_speed_not_the_last_command(node):
    # A ceiling (avoidance, curvature, an overtake cap) drops the *command*
    # instantly, but the car keeps rolling at nearly its old speed -- it
    # cannot shed 3 m/s in one 25ms tick. Ramping back up from that command
    # would hold the throttle far below the car's real speed for the whole
    # climb, braking a car that never actually slowed. The ramp must start
    # from measured speed instead.
    published = _capture_published(node)
    node.scan_callback(_clear_scan())
    _set_pose(node, -1.5, -1.2, 0.0)
    _set_odom(node, 3.0)
    node.last_commanded_speed = 1.0   # as a one-tick ceiling would leave it

    node.control_loop()
    speed = published[-1].drive.speed
    assert speed > 2.5, \
        "must not crawl back up from the capped command while the car is at 3 m/s"

    # ...and stale odometry must fall back to the old, conservative behavior
    # rather than trusting a reading that is no longer current.
    node.last_commanded_speed = 1.0
    node.last_odom_time = node.get_clock().now() - Duration(
        seconds=node.odom_timeout_sec + 0.1)
    node.last_command_time = None
    node.control_loop()
    assert published[-1].drive.speed == pytest.approx(
        1.0 + node.max_acceleration * node.command_slew_max_dt)


def test_ramp_basis_never_exceeds_the_configured_speed_ceiling(node):
    # A wildly wrong odometry reading must not be able to inflate the ramp
    # basis past max_speed and let the command jump beyond the safety ceiling.
    published = _capture_published(node)
    node.scan_callback(_clear_scan())
    _set_pose(node, -1.5, -1.2, 0.0)
    _set_odom(node, 500.0)
    node.control_loop()
    assert published[-1].drive.speed <= node.max_speed


def test_overtake_aims_further_ahead_than_the_normal_steering_target(node):
    # The passing line is a lateral offset applied to a waypoint
    # overtake_lookahead_distance ahead, deliberately *not* to the normal
    # max_lookahead target. Offsetting a target only max_lookahead away asks
    # for a turn sharp enough that the online curvature speed cap answers it
    # by braking -- which stalls the pass instead of completing it.
    scan, _center = _car_ahead_scan()
    node.scan_callback(scan)
    for step in range(4):
        _set_pose(node, -1.5 + step * 0.2, -1.2, 0.0)
        node.control_loop()
    assert node.overtake_active is True

    nearest_idx, _ = racing_math.find_nearest_index(
        node.xy, (node.car_x, node.car_y), closed=node.closed_loop)
    normal_target = node.xy[racing_math.find_lookahead_index(
        node.seg_len, nearest_idx, node.max_lookahead, closed=node.closed_loop)]
    overtake_x, overtake_y = node._update_opponent_and_overtake(nearest_idx)

    overtake_range = math.hypot(overtake_x - node.car_x, overtake_y - node.car_y)
    normal_range = math.hypot(
        normal_target[0] - node.car_x, normal_target[1] - node.car_y)
    assert overtake_range > normal_range
    assert overtake_range > node.max_lookahead


def test_overtake_preview_shorter_than_the_lookahead_is_rejected(profiled_csv):
    # Guards the invariant the preview exists for: a preview at or inside
    # max_lookahead reintroduces exactly the sharp passing turn it prevents.
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'enable_deadman:=false',
                     '-p', 'drive_topic:=/test_only/drive',
                     '-p', 'overtake_lookahead_distance:=0.5'])
    try:
        with pytest.raises(RuntimeError, match='overtake_lookahead_distance'):
            PurePursuitNode()
    finally:
        rclpy.shutdown()


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
    # 0.15m from the LiDAR straight ahead is only ~0.03m from the bodywork,
    # so the footprint tier is the correct (and more urgent) classification.
    assert node.last_decision_state == 'body_contact'


def test_forward_cone_obstacle_crawls_out_rather_than_latching(node):
    """An obstacle inside the forward cone with an open track beside it is
    escaped at a crawl, not sat next to forever.

    A hard stop cannot clear its own safety cone: at zero speed nothing
    about the scene changes, so whatever stopped the car keeps stopping it.
    That ended a simulated auto-map race on 2026-08-08 -- pure pursuit ran
    wide on its first corner, stopped 0.38m from a wall, and stayed there.
    The classification must still be its own tier, distinct from
    body_contact, and the speed must still be a crawl rather than a drive.
    """
    published = _capture_published(node)
    _set_pose(node, -1.5, -1.2, 0.0)
    scan = _clear_scan()
    center = len(scan.ranges) // 2
    # 0.30m ahead: inside emergency_stop_distance (0.40m), but ~0.18m of
    # body clearance -- comfortably outside emergency_stop_clearance.
    scan.ranges[center] = 0.30
    node.scan_callback(scan)
    node.control_loop()
    assert node.last_decision_state == 'emergency_escape'
    assert published[-1].drive.speed == pytest.approx(node.emergency_escape_speed)
    assert published[-1].drive.speed <= 0.3, 'the escape is a crawl, not a drive'


def test_forward_cone_hard_stop_still_fires_with_nowhere_to_go(node):
    """With no opening wide enough to crawl toward, the hard stop stands.

    The escape above is allowed to move the car only because there is
    somewhere to move it to. Boxed in, the node must fall back to the
    original behaviour and report emergency_obstacle.
    """
    published = _capture_published(node)
    _set_pose(node, -1.5, -1.2, 0.0)
    scan = _clear_scan()
    # A wall right across the avoidance cone: inside the emergency distance
    # ahead, and nothing anywhere near emergency_escape_min_gap to aim at.
    scan.ranges = [0.30] * len(scan.ranges)
    node.scan_callback(scan)
    node.control_loop()
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state in ('emergency_obstacle', 'body_contact')


def test_emergency_escape_yields_to_the_body_contact_tier(node):
    """Nearly touching means stop, not nudge -- whatever the cone shows.

    emergency_escape_clearance sits above emergency_stop_clearance for
    exactly this: a car with room to crawl may crawl; a car already against
    something may not, even when the forward cone shows an open track to
    aim at.
    """
    published = _capture_published(node)
    _set_pose(node, -1.5, -1.2, 0.0)
    scan = _clear_scan()
    center = len(scan.ranges) // 2
    # ~0.03m from the bodywork straight ahead, with the rest of the cone open.
    scan.ranges[center] = 0.15
    node.scan_callback(scan)
    node.control_loop()
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'body_contact'


def test_wall_alongside_the_car_stops_it_although_the_forward_cone_is_clear(node):
    """Regression test for the 2026-07-27 collision.

    The car sat against a wall that was *beside* it. Every forward-cone
    minimum read clear (the track ahead genuinely was open), so the old
    safety net logged "LIDAR clear" and pure pursuit accelerated into the
    wall. Only a clearance measured from the car's own footprint can see
    this, which is why the body tier exists.
    """
    published = _capture_published(node)
    _set_pose(node, -1.5, -1.2, 0.0)
    scan = _clear_scan()
    n = len(scan.ranges)
    center = n // 2
    # A wall hard against the car's left flank, and nothing at all ahead.
    quarter = n // 4          # +90deg with this helper's symmetric span
    for i in range(center + quarter - 12, center + quarter + 12):
        scan.ranges[i] = 0.17
    node.scan_callback(scan)
    node.control_loop()

    forward_cone_min = min(scan.ranges[center - 30:center + 31])
    assert forward_cone_min > node.emergency_stop_distance, (
        "test is only meaningful if the forward cone really is clear")
    assert published[-1].drive.speed == 0.0, "a wall against the flank must stop the car"
    assert node.last_decision_state == 'body_contact'


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
    assert node.last_decision_state == 'off_racing_line'


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
                     '-p', 'drive_topic:=/test_only/drive',
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
    detection = node._detect_opponent(scan, ranges, node.car_x + node.laser_offset_x, node.car_y)
    assert detection is None


def test_map_subtraction_detects_an_unmapped_car_directly(map_mode_node):
    pytest.importorskip('range_libc')
    node = map_mode_node
    node.map_callback(_synthetic_map())
    _set_pose(node, -1.5, -1.2, 0.0)

    scan, center = _car_ahead_scan(car_range=2.0)
    ranges = np.clip(np.nan_to_num(np.array(scan.ranges), nan=0.0, posinf=node.max_range),
                     0.0, node.max_range)
    detection = node._detect_opponent(scan, ranges, node.car_x + node.laser_offset_x, node.car_y)
    assert detection is not None
    start_idx, end_idx, centroid_range, centroid_angle = detection
    assert centroid_range == pytest.approx(2.0, abs=0.05)
    assert abs(centroid_angle) < 0.15
    # Indices must come back in *full-scan* space (downsampling mapped back).
    assert center - 12 <= start_idx <= center
    assert center <= end_idx <= center + 12


def test_opponent_progress_rate_wraps_cleanly_at_start_finish():
    tracker = OpponentTracker(smoothing_alpha=1.0, lost_timeout_sec=1.0)
    tracker.update(99.5, now_sec=10.0, total_length=100.0)
    tracker.update(0.5, now_sec=10.5, total_length=100.0)
    assert tracker.progress_rate == pytest.approx(2.0)


def test_waiting_node_loads_generated_profile_at_runtime(profiled_csv):
    rclpy.init(args=['--ros-args',
                     '-p', 'wait_for_waypoints:=true',
                     '-p', 'enable_deadman:=false',
                     '-p', 'drive_topic:=/test_only/drive'])
    waiting = PurePursuitNode()
    try:
        published = _capture_published(waiting)
        waiting.control_loop()
        assert published[-1].drive.speed == 0.0
        assert waiting.profile_ready is False
        assert waiting.last_decision_state == 'waiting_for_profile'

        result = waiting._parameter_callback([
            Parameter('waypoints_file', Parameter.Type.STRING, profiled_csv)])
        assert result.successful, result.reason
        assert waiting.profile_ready is True
        assert waiting.num_waypoints >= 3
    finally:
        waiting.destroy_node()
        rclpy.shutdown()
