"""
Integration tests for the *real* GapFollowNode, exercising the ROS node
rather than only gap_logic's pure functions (see test_gap_logic.py for
those). These cover the things that decide whether the car behaves safely
on the floor: the deadman gate, every emergency-stop path, the steering
sign convention, and the physical envelope of the published command.

Like pure_pursuit's node tests, this file needs ROS2 sourced and the
package importable -- it is not meant to run via a bare `pytest`:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/gap_follow/test/test_gap_follow_node.py -v

Each test drives the node through scan_callback/odom_callback/
joy_callback directly -- no topics or executor involved.
"""
import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import Joy, LaserScan

from gap_follow.gap_follow_node import GapFollowNode


DEADMAN_BUTTON = 4
MAX_STEERING = 0.26
MAX_SPEED = 2.0


@pytest.fixture
def node():
    """A real GapFollowNode with the shipped defaults, in its own context."""
    rclpy.init(args=['--ros-args',
                     '-p', 'min_gap_distance:=2.0',
                     '-p', 'fallback_min_gap_distance:=0.8',
                     '-p', 'min_speed:=0.8',
                     '-p', f'max_speed:={MAX_SPEED}',
                     '-p', f'max_steering_angle:={MAX_STEERING}'])
    n = GapFollowNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _scan(ranges=None, n=541, span=math.pi):
    msg = LaserScan()
    msg.angle_min = -span / 2.0
    msg.angle_increment = span / (n - 1)
    msg.angle_max = msg.angle_min + (n - 1) * msg.angle_increment
    msg.range_min, msg.range_max = 0.05, 12.0
    msg.ranges = [8.0] * n if ranges is None else list(ranges)
    return msg


def _hold_deadman(node, held=True):
    msg = Joy()
    msg.buttons = [0] * (DEADMAN_BUTTON + 1)
    msg.buttons[DEADMAN_BUTTON] = 1 if held else 0
    node.joy_callback(msg)


def _feed_odom(node, speed=0.0):
    msg = Odometry()
    msg.twist.twist.linear.x = float(speed)
    node.odom_callback(msg)


def _ready(node, speed=0.0):
    _hold_deadman(node)
    _feed_odom(node, speed)


def _capture(node):
    published = []
    original = node.drive_pub.publish
    node.drive_pub.publish = lambda msg: (published.append(msg), original(msg))
    return published


# ============================================================================
# The deadman gate -- mandatory workspace policy
# ============================================================================

def test_refuses_to_drive_before_any_joy_message(node):
    published = _capture(node)
    _feed_odom(node)
    node.scan_callback(_scan())
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'waiting_for_joy'


def test_refuses_to_drive_when_the_deadman_is_released(node):
    published = _capture(node)
    _ready(node)
    _hold_deadman(node, held=False)
    node.scan_callback(_scan())
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'deadman_released'


def test_stops_when_the_joy_stream_goes_stale(node):
    published = _capture(node)
    _ready(node)
    node.last_joy_time = node.get_clock().now() - Duration(
        seconds=node.joy_timeout_sec + 0.1)
    node.scan_callback(_scan())
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'joy_stale'


# ============================================================================
# Normal driving, and the physical envelope of what it publishes
# ============================================================================

def test_drives_forward_down_an_open_corridor(node):
    published = _capture(node)
    _ready(node)
    node.scan_callback(_scan())
    cmd = published[-1].drive
    assert cmd.speed > 0.0
    assert abs(cmd.steering_angle) < 0.05, "an open corridor must not steer"
    assert node.last_decision_state == 'gap_follow'


def test_every_command_stays_inside_the_physical_envelope(node):
    # Whatever the scan looks like, the published command must never exceed
    # the configured steering or speed limits -- these map directly onto
    # servo travel and motor ERPM on the real car.
    published = _capture(node)
    _ready(node)
    for offset in range(0, 541, 37):
        ranges = [8.0] * 541
        for i in range(offset, min(offset + 120, 541)):
            ranges[i] = 0.6
        node.scan_callback(_scan(ranges))
    assert published, "expected commands"
    for msg in published:
        assert -MAX_STEERING - 1e-9 <= msg.drive.steering_angle <= MAX_STEERING + 1e-9
        assert 0.0 <= msg.drive.speed <= MAX_SPEED + 1e-9
        assert math.isfinite(msg.drive.steering_angle)
        assert math.isfinite(msg.drive.speed)


def _blocked_right(n=541):
    ranges = [8.0] * n
    for i in range(0, 250):
        ranges[i] = 1.0
    return ranges


def _tick(node, scan_msg, dt=0.025):
    """Feed one scan as a 40Hz control interval would deliver it."""
    if node.last_command_time is not None:
        node.last_command_time = node.get_clock().now() - Duration(seconds=dt)
    node.scan_callback(scan_msg)


def test_steers_away_from_an_obstacle_on_one_side(node):
    # REP-103 / AckermannDriveStamped: +steering is left. An obstacle filling
    # the right half must produce a positive (left) steering command, and the
    # mirror image must produce the opposite sign. Getting this backwards
    # steers the car into the thing it is trying to avoid.
    _ready(node)
    published = _capture(node)

    _tick(node, _scan(_blocked_right()))
    assert published[-1].drive.steering_angle > 0.0, \
        "obstacle on the right must steer left"

    # Mirror the scene. The rack is rate-limited, so give it the same handful
    # of 40Hz intervals the real car would have to swing across.
    mirrored = _scan(list(reversed(_blocked_right())))
    for _ in range(30):
        _tick(node, mirrored)
    assert published[-1].drive.steering_angle < 0.0, \
        "obstacle on the left must steer right"


def test_steering_slews_at_the_configured_rate_and_no_faster(node):
    # The rate limit is what stops one noisy scan becoming a step input at
    # the servo. It also means a target that flips sides takes real time to
    # follow -- worth knowing before trusting this in tight spaces.
    _ready(node)
    published = _capture(node)
    _tick(node, _scan(_blocked_right()))
    first = published[-1].drive.steering_angle

    mirrored = _scan(list(reversed(_blocked_right())))
    _tick(node, mirrored, dt=0.025)
    step = abs(published[-1].drive.steering_angle - first)

    # The node's hard guarantee: one command can never move further than the
    # rate limit integrated over command_slew_max_dt, however long the control
    # loop was stalled. (The exact per-tick dt here is the test's own wall
    # clock, so it lands a hair above the nominal 0.025s interval.)
    assert step <= node.max_steering_rate * node.command_slew_max_dt + 1e-9
    assert step == pytest.approx(node.max_steering_rate * 0.025, rel=0.15), \
        "a full sign reversal should saturate the rate limit, not undershoot it"


# ============================================================================
# Emergency paths -- these must bypass all command shaping
# ============================================================================

def test_close_obstacle_stops_immediately_without_ramping_down(node):
    published = _capture(node)
    _ready(node, speed=1.5)
    node.scan_callback(_scan())
    assert published[-1].drive.speed > 0.0   # sanity: was driving

    blocked = [0.12] * 541
    node.scan_callback(_scan(blocked))
    assert published[-1].drive.speed == 0.0, \
        "a stop must be published outright, never rate-limited down"
    assert node.last_decision_state in (
        'emergency_clearance', 'forward_clearance', 'ttc_brake', 'no_safe_gap')


def test_stops_when_odometry_goes_stale_while_ttc_is_enabled(node):
    published = _capture(node)
    _ready(node, speed=1.0)
    node.last_odom_time = node.get_clock().now() - Duration(
        seconds=node.odom_timeout_sec + 0.1)
    node.scan_callback(_scan())
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'odometry_stale'


def test_stops_on_a_malformed_scan_rather_than_guessing(node):
    published = _capture(node)
    _ready(node)
    empty = _scan()
    empty.ranges = []
    node.scan_callback(empty)
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'scan_empty'


def test_invalid_returns_do_not_fake_an_emergency_stop(node):
    # NaN/zero beams are *unknown*, not contact. They must not trigger the
    # emergency stop, or a dirty lens would permanently immobilise the car.
    published = _capture(node)
    _ready(node)
    ranges = [8.0] * 541
    for i in range(200, 340):
        ranges[i] = float('nan')
    node.scan_callback(_scan(ranges))
    assert node.last_decision_state != 'emergency_clearance'
    assert math.isfinite(published[-1].drive.speed)


# ============================================================================
# Command shaping
# ============================================================================

def test_speed_ramp_starts_from_measured_speed_not_the_last_command(node):
    # After a ceiling or a stop drops the command, the car is still rolling.
    # Ramping up from the command would brake a car that never slowed.
    published = _capture(node)
    _ready(node, speed=1.6)
    node.last_commanded_speed = 0.0
    node.last_command_time = None
    node.scan_callback(_scan())
    assert published[-1].drive.speed > 1.0, \
        "must not crawl up from 0 while the car is measurably doing 1.6 m/s"


def test_a_wild_odometry_reading_cannot_exceed_the_speed_ceiling(node):
    published = _capture(node)
    _ready(node, speed=500.0)
    node.scan_callback(_scan())
    assert published[-1].drive.speed <= MAX_SPEED
