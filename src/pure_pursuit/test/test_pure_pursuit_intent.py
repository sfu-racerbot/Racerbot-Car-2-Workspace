"""
Integration tests for pure_pursuit's /drive_intent publishing.

The interesting difference from gap_follow's equivalent tests is the
shape of the predicted path. gap_follow chooses a *heading*, so one
constant-curvature arc is the honest picture. pure_pursuit is following a
stored line, so its arrow re-runs the pure pursuit law at every
integration step and must bend through the corner ahead -- that is the
whole diagnostic value of drawing it, and the tests below are what stop
it quietly regressing into a straight tangent.

The three safety rules from docs/drive-intent.md are covered here too.

Needs ROS2 sourced and pure_pursuit importable:
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/pure_pursuit/test/test_pure_pursuit_intent.py -v
"""
import math
import os

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from drive_intent import schema
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
    # drive_topic remapped away from the real /drive for the same reason as
    # the other node tests: the deadman is disabled here, so an
    # un-remapped test would publish live commands into ackermann_mux if
    # the driver stack happened to be up.
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'enable_deadman:=false',
                     '-p', 'drive_topic:=/test_only/drive',
                     '-p', 'intent_topic:=/test_only/drive_intent',
                     '-p', 'intent_rate_hz:=0.0'])
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


def _publish_pose(node, x, y, yaw=0.0):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    node.pose_callback(msg)


def _set_odom(node, speed):
    msg = Odometry()
    msg.twist.twist.linear.x = float(speed)
    node.odom_callback(msg)


def _capture_intent(node):
    seen = []
    original = node.intent_pub.publish

    def spy(msg):
        seen.append(schema.decode(msg.data))
        return original(msg)

    node.intent_pub.publish = spy
    return seen


def _capture_drive(node):
    seen = []
    original = node.drive_pub.publish
    node.drive_pub.publish = lambda m: (seen.append(m), original(m))
    return seen


def _drive_once(node, x=-1.5, y=-1.2, yaw=0.0, speed=1.25):
    node.scan_callback(_clear_scan())
    _set_odom(node, speed)
    _publish_pose(node, x, y, yaw)
    node.control_loop()


def _straightness(pts):
    """0 for a perfectly straight path, growing with how much it bends:
    the ratio of end-to-end distance to arc length, inverted."""
    if len(pts) < 2:
        return 0.0
    arc = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y'])
              for a, b in zip(pts, pts[1:]))
    chord = math.hypot(pts[-1]['x'] - pts[0]['x'], pts[-1]['y'] - pts[0]['y'])
    return 0.0 if arc <= 0 else 1.0 - (chord / arc)


# ============================================================================
# What gets published while racing
# ============================================================================

def test_a_racing_tick_publishes_a_valid_intent(node):
    intents = _capture_intent(node)
    _drive_once(node)
    assert intents, 'a normal control tick should publish intent'
    assert schema.validate(intents[-1]) is None
    assert intents[-1]['node'] == 'pure_pursuit_node'
    assert intents[-1]['state'] == 'pure_pursuit'
    assert intents[-1]['severity'] == 'drive'


def test_the_intent_reports_the_command_that_was_actually_published(node):
    intents = _capture_intent(node)
    drives = _capture_drive(node)
    _drive_once(node)
    assert intents[-1]['commanded_speed'] == pytest.approx(
        drives[-1].drive.speed, abs=1e-3)
    assert intents[-1]['commanded_steering'] == pytest.approx(
        drives[-1].drive.steering_angle, abs=1e-4)


def test_the_predicted_path_starts_at_the_car(node):
    """Published in base_link, so the first sample is the origin by
    construction -- that is what lets the arrow draw with no pose at all
    in robot-centric mode."""
    intents = _capture_intent(node)
    _drive_once(node)
    first = intents[-1]['path'][0]
    assert first['x'] == pytest.approx(0.0, abs=1e-6)
    assert first['y'] == pytest.approx(0.0, abs=1e-6)


def test_the_speed_profile_varies_along_the_predicted_path(node):
    """pure_pursuit's plan is not one speed held for a horizon: it is the
    profile of the line ahead, which is what makes the arrow taper into a
    corner and flare out of it."""
    intents = _capture_intent(node)
    _drive_once(node, speed=3.0)
    speeds = [p['v'] for p in intents[-1]['path']]
    assert len(set(round(v, 3) for v in speeds)) > 1


def test_the_steering_target_is_reported_ahead_of_the_car(node):
    intents = _capture_intent(node)
    _drive_once(node)
    targets = intents[-1]['targets']
    assert [t['kind'] for t in targets] == ['steering_target']
    assert math.hypot(targets[0]['x'], targets[0]['y']) > 0.3


def test_the_speed_ceilings_are_named_and_the_lowest_binds(node):
    intents = _capture_intent(node)
    _drive_once(node)
    factors = intents[-1]['factors']
    assert {'profile speed', 'max speed'} <= {f['name'] for f in factors}
    binding = [f for f in factors if f['binding']]
    lowest = min(f['value'] for f in factors)
    assert binding and all(f['value'] == pytest.approx(lowest) for f in binding)


# ============================================================================
# The line-following prediction -- the reason this arrow is worth drawing
# ============================================================================

def test_the_arrow_follows_the_racing_line_round_a_corner(node):
    """Placed at a corner of the stadium, the predicted path must bend.
    A frozen-arc prediction would leave on the tangent and say nothing
    about the plan."""
    intents = _capture_intent(node)
    # Walk the line to find a genuinely curved stretch, then sit on it.
    curvature = racing_math.estimate_path_curvature(node.xy, closed=True)
    corner_idx = int(max(range(len(curvature)), key=lambda i: abs(curvature[i])))
    cx, cy = node.xy[corner_idx]
    nx, ny = node.xy[(corner_idx + 1) % len(node.xy)]
    _drive_once(node, x=float(cx), y=float(cy),
                yaw=math.atan2(ny - cy, nx - cx), speed=2.0)

    assert intents[-1]['state'] == 'pure_pursuit'
    assert _straightness(intents[-1]['path']) > 0.01, \
        'the predicted path through a corner should not be straight'


def test_the_predicted_path_tracks_the_line_rather_than_the_current_heading(node):
    """Deliberately point the car off the line and check the prediction
    curves back onto it: the arrow shows what the *controller* will do,
    not where the nose currently points."""
    intents = _capture_intent(node)
    curvature = racing_math.estimate_path_curvature(node.xy, closed=True)
    corner_idx = int(max(range(len(curvature)), key=lambda i: abs(curvature[i])))
    cx, cy = node.xy[corner_idx]
    nx, ny = node.xy[(corner_idx + 1) % len(node.xy)]
    along = math.atan2(ny - cy, nx - cx)
    _drive_once(node, x=float(cx), y=float(cy), yaw=along + 0.35, speed=2.0)

    pts = intents[-1]['path']
    # A car nosed 0.35rad off the line that stayed on that heading would
    # end the horizon well off to one side; tracking the line brings it
    # back, so the lateral excursion stays bounded.
    lateral = [abs(p['y']) for p in pts]
    assert max(lateral) < 3.0
    assert _straightness(pts) > 0.005


def test_a_reactive_override_falls_back_to_a_single_arc(node):
    """When the reactive net takes over, following the line is no longer
    the plan -- drawing the line the controller is ignoring would show
    intent it does not have."""
    intents = _capture_intent(node)
    blocked = _clear_scan()
    blocked.ranges = [0.35] * len(blocked.ranges)
    _set_odom(node, 1.25)
    _publish_pose(node, -1.5, -1.2, 0.0)
    node.scan_callback(blocked)
    node.control_loop()

    assert intents[-1]['state'] != 'pure_pursuit'
    assert intents[-1]['severity'] in ('caution', 'stop')


# ============================================================================
# Stops and reasons
# ============================================================================

def test_waiting_for_a_pose_publishes_a_stop_intent(node):
    intents = _capture_intent(node)
    node.scan_callback(_clear_scan())
    node.control_loop()
    assert intents[-1]['state'] == 'waiting_for_pose'
    assert intents[-1]['severity'] == 'stop'
    assert intents[-1]['commanded_speed'] == pytest.approx(0.0)


def test_a_transition_carries_its_reason(node):
    intents = _capture_intent(node)
    node.scan_callback(_clear_scan())
    node.control_loop()                     # waiting_for_pose
    _drive_once(node)                       # -> pure_pursuit
    assert intents[-1]['state'] == 'pure_pursuit'
    assert intents[-1].get('reason')
    assert 'nearest waypoint' in intents[-1]['reason']


def test_a_steady_state_does_not_repeat_its_reason_every_tick(node):
    intents = _capture_intent(node)
    for _ in range(6):
        _drive_once(node)
    assert any('reason' not in payload for payload in intents[1:])


# ============================================================================
# The safety rules
# ============================================================================

def test_the_drive_command_is_published_before_the_intent(node):
    order = []
    original_drive = node.drive_pub.publish
    original_intent = node.intent_pub.publish
    node.drive_pub.publish = lambda m: (order.append('drive'), original_drive(m))
    node.intent_pub.publish = lambda m: (order.append('intent'), original_intent(m))

    _drive_once(node)
    assert order and order[0] == 'drive'
    assert order.index('drive') < order.index('intent')


def test_a_broken_intent_builder_does_not_stop_the_car_driving(node):
    drives = _capture_drive(node)

    def explode(msg):
        raise RuntimeError('simulated intent bug')

    node.intent_pub.publish = explode
    _drive_once(node)                        # must not raise

    assert drives[-1].drive.speed > 0.0
    assert node.last_decision_state == 'pure_pursuit'


def test_sustained_intent_failure_switches_intent_off_not_the_node(node):
    drives = _capture_drive(node)

    def explode(msg):
        raise RuntimeError('simulated intent bug')

    node.intent_pub.publish = explode
    for _ in range(20):
        _drive_once(node)

    assert node._intent_failures.disabled
    assert drives[-1].drive.speed > 0.0


def test_a_broken_path_predictor_does_not_stop_the_car_driving(node):
    """The prediction touches more of the node's state than the encoder
    does -- the waypoint array, the profile, the lookahead law -- so it
    gets its own version of the same test."""
    drives = _capture_drive(node)
    intents = _capture_intent(node)

    def explode(*args, **kwargs):
        raise IndexError('simulated prediction bug')

    node._intent_path = explode
    _drive_once(node)

    assert drives[-1].drive.speed > 0.0
    assert intents == []


def test_publish_intent_false_creates_no_publisher_at_all(profiled_csv):
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'enable_deadman:=false',
                     '-p', 'drive_topic:=/test_only/drive',
                     '-p', 'publish_intent:=false'])
    n = PurePursuitNode()
    try:
        assert n.intent_pub is None
        _drive_once(n)
        assert n.last_decision_state == 'pure_pursuit'
    finally:
        n.destroy_node()
        rclpy.shutdown()
