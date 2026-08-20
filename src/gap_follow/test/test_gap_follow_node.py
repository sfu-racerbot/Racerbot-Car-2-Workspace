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
import os

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import Joy, LaserScan

from gap_follow import gap_logic
from gap_follow.gap_follow_node import GapFollowNode
from gap_follow.speed_overrides import mapping_speed_overrides


PACKAGED_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'gap_follow.yaml')
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


def test_a_transient_stop_does_not_cost_the_car_its_steering(node):
    """Regression: the 2026-07-27 wall collision.

    A TTC brake was landing between every pair of scans. Each one published
    steering=0, and that zero was fed straight back to the slew limiter as
    its basis, so every drive command in between was re-limited starting
    from centre and pinned at max_steering_rate*dt (~0.03rad, about 1.7deg).
    The speed ramp reads *measured* speed rather than the last command, so
    it recovered the full 1.8m/s every time. Full speed with almost no
    steering authority is how the car drifted into a wall while its logs
    showed it steering toward the gap the whole way.
    """
    _ready(node, speed=1.5)
    published = _capture(node)
    scene = _scan(_blocked_right())

    for _ in range(30):
        _tick(node, scene)
        node._stop('injected_brake', 'transient safety stop between scans')

    drive_steering = [
        msg.drive.steering_angle for msg in published if msg.drive.speed > 0.0]
    assert drive_steering, 'expected the node to keep issuing drive commands'

    one_tick = node.max_steering_rate * node.command_slew_max_dt
    assert abs(drive_steering[-1]) > one_tick, (
        'steering must accumulate across ticks; a command still pinned at the '
        'one-tick slew bound means a transient stop reset the limiter basis')
    assert drive_steering[-1] > 0.0, 'obstacle on the right must steer left'


def test_braking_mid_run_holds_the_rack_where_it_is(node):
    """Braking is not a reason to throw away the turn the car is mid-way
    through -- a stationary car's steering angle cannot cause motion, and the
    car needs that angle the moment it is allowed to move again."""
    _ready(node, speed=1.5)
    published = _capture(node)
    for _ in range(30):
        _tick(node, _scan(_blocked_right()))
    turning = node.steering_basis
    assert abs(turning) > 0.05, 'expected a real turn to build up'

    for _ in range(40):
        node.last_command_time = node.get_clock().now() - Duration(seconds=0.025)
        node._stop('sustained', 'held stop')
    assert node.steering_basis == pytest.approx(turning)
    assert published[-1].drive.speed == 0.0, 'a stop must still command zero speed'
    assert published[-1].drive.steering_angle == pytest.approx(turning)


def test_recovery_never_depends_on_cycling_the_deadman(node):
    """Nothing in the stop path may need an operator to notice the car and
    release LB. Whatever a stop leaves behind, the node must be able to drive
    straight back out of it on its own the moment the scan says it can."""
    _ready(node, speed=1.5)
    published = _capture(node)
    for _ in range(30):
        _tick(node, _scan(_blocked_right()))
    turning = node.steering_basis
    assert abs(turning) > 0.05

    # A long blocking stop, LB held the whole time -- exactly the deadlock the
    # car sat in on 2026-07-27.
    blocked = _scan([0.30] * 541)
    for _ in range(40):
        _tick(node, blocked)
    assert published[-1].drive.speed == 0.0, 'expected the node to be stopped'

    # The obstacle clears. No LB cycle, no operator, no reset: the very next
    # scans must produce motion again.
    for _ in range(10):
        _tick(node, _scan(_blocked_right()))
    assert published[-1].drive.speed > 0.0, \
        'the node must resume on its own once the scan clears'


def _wall_ahead(distance, n=541, elsewhere=8.0):
    """A wall `distance` away across the forward +/-30deg cone only."""
    ranges = [elsewhere] * n
    for i in range(180, 361):          # -30deg .. +30deg on a 541/pi scan
        ranges[i] = distance
    return ranges


def test_a_blocked_forward_cone_crawls_out_instead_of_latching(node):
    """Regression: the deadlock that stranded the car on 2026-07-27.

    The forward clearance cone points where the car is *aimed*, not where it
    is *going*. A car turning out of a corner therefore trips it on the wall
    it is already steering around -- and a hard stop there is unrecoverable,
    because stopping removes the very motion that would clear the cone. With
    an escape visible the car must keep crawling toward it.
    """
    _ready(node, speed=0.1)
    published = _capture(node)
    for _ in range(5):
        _tick(node, _scan(_wall_ahead(0.35)))

    last = published[-1]
    assert last.drive.speed > 0.0, \
        'a blocked cone with a visible escape must crawl, not latch at zero'
    assert last.drive.speed <= node.escape_creep_speed + 1e-9, \
        'the crawl must stay within escape_creep_speed'
    assert abs(last.drive.steering_angle) > 0.0, 'it must steer while crawling'


def test_the_crawl_never_exceeds_its_stopping_distance(node):
    """The creep spends at most half the forward reserve, so it still has
    real margin in hand -- it must taper toward zero as the wall closes."""
    _ready(node, speed=0.1)
    published = _capture(node)

    speeds = []
    for distance in (0.35, 0.28, 0.24, 0.20):
        for _ in range(3):
            _tick(node, _scan(_wall_ahead(distance)))
        speeds.append(published[-1].drive.speed)

    assert speeds == sorted(speeds, reverse=True), \
        f'crawl speed must fall as the wall closes, got {speeds}'
    assert speeds[-1] < speeds[0]


def test_a_dead_end_still_stops_the_car_dead(node):
    """The creep must not become a licence to drive into a wall. With no gap
    anywhere, the car stops at zero and stays there."""
    _ready(node, speed=0.1)
    published = _capture(node)
    for _ in range(10):
        _tick(node, _scan([0.35] * 541))

    assert published[-1].drive.speed == 0.0, \
        'boxed in with no escape, the car must command a full stop'
    assert node.last_decision_state in ('no_safe_gap', 'emergency_clearance')


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


def _wall_ahead(distance, half_angle_deg=10.0, n=541, span=math.pi):
    """A slab at `distance` straight ahead, open space either side of it."""
    increment = span / (n - 1)
    ranges = [8.0] * n
    for i in range(n):
        angle = -span / 2.0 + i * increment
        if abs(angle) <= math.radians(half_angle_deg):
            ranges[i] = distance
    return ranges


def test_ttc_brakes_at_speed_but_not_at_a_crawl(node):
    """The TTC brake is armed only above ttc_min_brake_speed.

    TTC is clearance/closing-speed, so a crawling car reaches the threshold
    on a *tiny* clearance -- exactly the state a car is in once it has eased
    up to the inside of a corner. Braking there removes the only motion that
    would clear the corner, and the measured 2026-08-06 run shows the result:
    stops at 0.15-0.39m/s, several reading `odom 0.00m/s` while braking on
    the car's own commanded speed. Above the gate the brake must still fire.
    """
    _ready(node, speed=1.2)
    published = _capture(node)
    _tick(node, _scan(_wall_ahead(0.45)))
    assert node.last_decision_state == 'ttc_brake', \
        'at 1.2m/s a wall 0.45m ahead must still trip the TTC brake'
    assert published[-1].drive.speed == 0.0

    # Same closing geometry, scaled to a crawl: TTC is still under threshold,
    # but the car must be left free to creep out rather than latched at zero.
    _ready(node, speed=0.55)
    published = _capture(node)
    _tick(node, _scan(_wall_ahead(0.28)))
    assert node.last_decision_state != 'ttc_brake', \
        'below ttc_min_brake_speed the TTC brake must not be what stops the car'
    assert published[-1].drive.speed > 0.0, \
        'a crawling car with a visible way out must still be able to move'


def test_the_crawl_case_is_the_gate_and_not_the_geometry(node):
    """Guard against the test above passing for an unrelated reason.

    With the gate opened back up, the identical crawl scan must brake -- which
    is what pins ttc_min_brake_speed as the thing that changed.
    """
    node.ttc_min_brake_speed = 0.0
    _ready(node, speed=0.55)
    published = _capture(node)
    _tick(node, _scan(_wall_ahead(0.28)))
    assert node.last_decision_state == 'ttc_brake'
    assert published[-1].drive.speed == 0.0


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


# ============================================================================
# Corridor centering -- the cross-track term, wired end to end
# ============================================================================

def _corridor_scan(left_dist, right_dist, n=811, span=math.radians(270)):
    """A straight corridor seen by a 270deg scan, car parallel to the walls.

    Uses the real sensor span rather than the 180deg default, because the node
    deliberately measures the side walls against the *full* scan -- the
    forward window puts both side directions exactly on its boundary.
    """
    msg = _scan(n=n, span=span)
    angles = msg.angle_min + np.arange(n) * msg.angle_increment
    ranges = np.full(n, 10.0)
    for i, a in enumerate(angles):
        hits = []
        if math.cos(a - math.pi / 2) > 1e-3:
            hits.append(left_dist / math.cos(a - math.pi / 2))
        if math.cos(a + math.pi / 2) > 1e-3:
            hits.append(right_dist / math.cos(a + math.pi / 2))
        if hits:
            ranges[i] = min(10.0, min(hits))
    msg.ranges = [float(r) for r in ranges]
    return msg


def test_centering_steers_off_a_wall_it_is_running_parallel_to(node):
    """The behaviour being fixed. In a corridor wide enough that the free
    space reaches the FOV edge, the aim is the deepest beam -- straight down
    the corridor, parallel to both walls -- so bearing steering alone reports
    zero error while the car sits 0.75m off centre."""
    published = _capture(node)
    _ready(node, speed=1.0)
    node.scan_callback(_corridor_scan(left_dist=1.50, right_dist=3.00))
    assert published[-1].drive.speed > 0.0, 'should be driving, not stopped'
    assert published[-1].drive.steering_angle < -0.01, (
        'closer to the left wall, so it must steer right')


def test_centering_is_symmetric(node):
    published = _capture(node)
    _ready(node, speed=1.0)
    node.scan_callback(_corridor_scan(left_dist=3.00, right_dist=1.50))
    assert published[-1].drive.steering_angle > 0.01


def test_centering_is_silent_when_already_centred(node):
    published = _capture(node)
    _ready(node, speed=1.0)
    node.scan_callback(_corridor_scan(left_dist=2.25, right_dist=2.25))
    assert abs(published[-1].drive.steering_angle) < 0.01


def test_centering_never_exceeds_its_authority_bound(node):
    """Hard against one wall, the bias is still only a bias."""
    published = _capture(node)
    _ready(node, speed=1.0)
    node.scan_callback(_corridor_scan(left_dist=0.60, right_dist=3.50))
    assert abs(published[-1].drive.steering_angle) <= node.centering_max_steering + 1e-9


def test_without_centering_the_car_holds_its_offset(node):
    """Same scan with the term switched off: bearing steering sees a target
    dead ahead and commands nothing, which is exactly the wall-hugging."""
    published = _capture(node)
    node.enable_centering = False
    _ready(node, speed=1.0)
    node.scan_callback(_corridor_scan(left_dist=1.50, right_dist=3.00))
    assert abs(published[-1].drive.steering_angle) < 1e-9


def test_narrow_corridor_still_uses_the_midpoint_aim(node):
    """Where both gap edges are real obstacles the midpoint already steers off
    the near wall. Centering must not fight it -- same sign, no cancellation."""
    published = _capture(node)
    _ready(node, speed=1.0)
    node.scan_callback(_corridor_scan(left_dist=0.55, right_dist=1.25))
    assert published[-1].drive.steering_angle < -0.01


def test_centering_does_not_disturb_the_open_road(node):
    """No walls in range is not a corridor; the car must go straight."""
    published = _capture(node)
    _ready(node, speed=1.0)
    node.scan_callback(_scan(n=811, span=math.radians(270)))
    assert abs(published[-1].drive.steering_angle) < 1e-9


def test_centering_refuses_to_start_with_too_much_authority():
    """A bias able to cancel the chosen gap is a second driving policy."""
    rclpy.init(args=['--ros-args',
                     '-p', 'max_steering_angle:=0.26',
                     '-p', 'centering_max_steering:=0.25'])
    try:
        with pytest.raises(ValueError, match='centering_max_steering'):
            GapFollowNode()
    finally:
        rclpy.shutdown()


# ============================================================================
# Adaptive corridor width -- shrink the gap-selection margin and raise the
# fallback corner-speed cap on a sensed-narrow straight (see gap_logic's
# corridor_width_factor/scale_between for the pure math; this is the wiring).
# ============================================================================

def test_adaptive_width_margin_is_a_no_op_by_default_even_on_a_narrow_straight(node):
    """Shipped behaviour (config/gap_follow.yaml): min_safety_margin
    defaults to safety_margin, so the margin never actually shrinks --
    measured worse, not better, on a uniformly narrow real course (see
    docs/asb-10000-sim-results.json). corner_speed_wide is the part that
    ships active; test_adaptive_width_raises_the_corner_speed_ceiling_on_a_
    wide_bend below covers it."""
    assert node.min_safety_margin == pytest.approx(node.safety_margin)
    _ready(node, speed=1.0)
    node.scan_callback(_corridor_scan(left_dist=0.70, right_dist=0.70))  # 1.4m total
    assert node.width_factor == pytest.approx(0.0)
    assert node.effective_safety_margin == pytest.approx(node.safety_margin)


def test_adaptive_width_leaves_the_margin_alone_on_a_sensed_wide_corridor(node):
    _ready(node, speed=1.0)
    node.scan_callback(_corridor_scan(left_dist=1.50, right_dist=1.50))  # 3.0m total
    assert node.width_factor == pytest.approx(1.0)
    assert node.effective_safety_margin == pytest.approx(node.safety_margin)


def test_adaptive_width_margin_still_scales_linearly_when_configured_to_shrink(node):
    """The interpolation itself is still correct if a future per-course
    config lowers min_safety_margin below safety_margin -- exercised
    directly here since it no longer happens with the shipped defaults."""
    node.min_safety_margin = 0.12
    _ready(node, speed=1.0)
    node.scan_callback(_corridor_scan(left_dist=1.00, right_dist=1.00))  # 2.0m: halfway
    assert node.width_factor == pytest.approx(0.5, abs=0.02)
    expected = (node.min_safety_margin + node.safety_margin) / 2.0
    assert node.effective_safety_margin == pytest.approx(expected, abs=0.005)


def test_disabling_adaptive_width_holds_the_static_corner_speed_on_a_wide_bend(node):
    """Margin is unaffected by enable_adaptive_width either way under the
    shipped defaults (min_safety_margin == safety_margin already), so the
    differentiating case is corner_speed on a *wide* corridor -- disabled
    holds the floor, enabled reaches the ceiling."""
    node.enable_adaptive_width = False
    _ready(node, speed=1.0)
    node.scan_callback(_corridor_scan(left_dist=1.50, right_dist=1.50))  # wide
    assert node.width_factor == pytest.approx(1.0)
    assert node.effective_safety_margin == pytest.approx(node.safety_margin)
    assert node.effective_corner_speed == pytest.approx(node.corner_speed)


def test_adaptive_width_raises_the_corner_speed_ceiling_on_a_wide_bend(node):
    """corner_speed itself is the floor -- today's tuned narrow-corner
    behaviour is unchanged. corner_speed_wide is the ceiling a fallback
    gap taken for lack of forward visibility (not lack of room) may reach."""
    _, narrow_margin, narrow_corner_speed = node._adaptive_width(0.70, 0.70)
    _, wide_margin, wide_corner_speed = node._adaptive_width(1.50, 1.50)
    assert narrow_corner_speed == pytest.approx(node.corner_speed)
    assert wide_corner_speed == pytest.approx(node.corner_speed_wide)
    assert wide_corner_speed > narrow_corner_speed


def test_adaptive_width_ignores_an_open_side_and_holds_the_static_defaults(node):
    """One side unbounded is not a corridor to measure -- treated as "wide"
    (width_factor=1.0), the same "assume the already-validated defaults"
    answer corridor centering gives for an open side. For the margin, wide
    *is* the static default, so nothing moves; corner_speed's wide end is
    its ceiling, so it is the one that rises here, same as a genuinely
    measured-wide corridor would."""
    width_factor, margin, corner_speed = node._adaptive_width(0.70, math.inf)
    assert width_factor == pytest.approx(1.0)
    assert margin == pytest.approx(node.safety_margin)
    assert corner_speed == pytest.approx(node.corner_speed_wide)


def test_smaller_effective_margin_recovers_a_gap_a_static_one_would_miss(node):
    """The mechanism the ASB 10000-level course exercised (see
    docs/asb-10000-sim-results.json): at the static 0.18m margin,
    disparity_extend's inflation converging from both sides of a narrow
    opening can fully bridge it, reporting no gap at all. The same opening
    at the adaptive floor (0.12m) leaves enough of it standing to find a
    preferred gap instead of stopping. These exact widths (a 94-beam
    opening at 0.01rad spacing, walls at 0.6m) were chosen so the static
    margin's disparity reach (51 beams each side, fully bridging it) and
    the adaptive floor's reach (43 beams each side, leaving an 8-beam
    sliver) land on opposite sides of that bridge -- not a coincidence of
    this car's real car_width/margin numbers, so this is pinned rather than
    computed from them."""
    n = 600
    angle_increment = 0.01
    beam_angles = np.zeros(n)
    window = np.full(n, 0.6)
    window[150:244] = 2.2
    window_valid = np.ones(n, dtype=bool)

    node.effective_safety_margin = node.safety_margin
    _, _, gap_start, _, _, _, _ = node._select_gap(
        window.copy(), window_valid, angle_increment, beam_angles)
    assert gap_start is None, 'the static margin should fully bridge this opening'

    node.effective_safety_margin = 0.12  # a margin a per-course config could set
    _, _, gap_start, _, used_fallback, _, _ = node._select_gap(
        window.copy(), window_valid, angle_increment, beam_angles)
    assert gap_start is not None, 'a smaller margin should leave the opening standing'
    assert not used_fallback, 'the surviving sliver still clears the preferred depth'


def test_adaptive_width_refuses_a_floor_above_the_static_margin():
    """min_safety_margin is a floor the margin may shrink to, not a value
    it may exceed -- this feature only ever narrows toward it, never widens
    the margin beyond safety_margin on a sensed-wide corridor."""
    rclpy.init(args=['--ros-args',
                     '-p', 'safety_margin:=0.18',
                     '-p', 'min_safety_margin:=0.20'])
    try:
        with pytest.raises(ValueError, match='min_safety_margin'):
            GapFollowNode()
    finally:
        rclpy.shutdown()


def test_adaptive_width_refuses_a_narrow_reference_that_does_not_fit():
    rclpy.init(args=['--ros-args',
                     '-p', 'adaptive_width_narrow:=2.6',
                     '-p', 'adaptive_width_reference:=1.4'])
    try:
        with pytest.raises(ValueError, match='adaptive_width_narrow'):
            GapFollowNode()
    finally:
        rclpy.shutdown()


def test_packaged_config_plus_mapping_overrides_actually_starts():
    """The real regression guard for 2026-08-19.

    Every mapping launch file lowers max_speed for a cautious first lap.
    Doing that alone left the packaged corner caps above the new top
    speed, this constructor raised, gap_follow_node exited, nothing
    published /auto_map/drive, and the whole one-command auto_map_race run
    sat in pure_pursuit's 'waiting_for_profile' with the car motionless.

    Asserted against the shipped YAML rather than the constructor's own
    declared defaults, because the YAML is what the launch files load and
    the two disagree -- the config's corner_speed is not the code's.
    """
    for mapping_max_speed in ('', 0.4, 1.0, 1.5, 2.5, 3.5):
        speeds = mapping_speed_overrides(PACKAGED_CONFIG, mapping_max_speed, '')
        overrides = []
        for name, value in speeds.items():
            overrides += ['-p', f'{name}:={value}']
        rclpy.init(args=['--ros-args', '--params-file', PACKAGED_CONFIG] + overrides)
        try:
            node = GapFollowNode()
            assert node.corner_speed <= node.corner_speed_wide <= node.max_speed
            assert node.min_speed <= node.max_speed
            node.destroy_node()
        finally:
            rclpy.shutdown()


def test_lowering_only_max_speed_is_still_rejected():
    """The launch-side fix must not have quietly relaxed the check itself:
    an inconsistent set is a configuration error and the node is still
    expected to refuse it rather than pick an interpretation."""
    rclpy.init(args=['--ros-args', '--params-file', PACKAGED_CONFIG,
                     '-p', 'max_speed:=1.0'])
    try:
        with pytest.raises(ValueError, match='corner_speed_wide'):
            GapFollowNode()
    finally:
        rclpy.shutdown()


def test_adaptive_width_refuses_a_corner_speed_ceiling_below_its_floor():
    rclpy.init(args=['--ros-args',
                     '-p', 'corner_speed:=0.8',
                     '-p', 'corner_speed_wide:=0.5'])
    try:
        with pytest.raises(ValueError, match='corner_speed_wide'):
            GapFollowNode()
    finally:
        rclpy.shutdown()


# ============================================================================
# Gap-selection hysteresis and cornering anticipation -- both added after a
# real-hallway report of the car "driving kind of drunk, side to side" and
# taking corners "too slowly and too tightly" (see
# docs/asb-10000-sim-results.json for the course this was measured against).
# ============================================================================

def test_select_gap_hysteresis_keeps_the_previous_gap_through_a_small_edge(node):
    """Same shape as gap_logic's own hysteresis tests, run through the real
    node method (self.previous_target_idx/self.gap_switch_margin) rather
    than the pure function directly, to prove the node actually reads its
    own state instead of just having the mechanism available unused. Sized
    well past disparity_extend's inflation reach at the default margin so
    that reach isn't what the assertion is really about."""
    n = 2000
    angle_increment = 0.01
    window = np.full(n, 1.5)   # background, below min_gap_distance (2.0)
    window[500:900] = 3.0     # A: 400 wide, score 1200
    window[1200:1621] = 3.0   # B: 421 wide, score 1263 -- ~5% better than A
    window_valid = np.ones(n, dtype=bool)
    beam_angles = np.zeros(n)

    node.effective_safety_margin = node.safety_margin
    node.gap_switch_margin = 1.2
    node.previous_target_idx = 700  # inside A
    _, _, gap_start, gap_end, _, _, _ = node._select_gap(
        window.copy(), window_valid, angle_increment, beam_angles)
    assert gap_start < 900, 'kept A despite B scoring slightly higher'


def test_select_gap_hysteresis_is_a_no_op_with_no_previous_target(node):
    n = 2000
    angle_increment = 0.01
    window = np.full(n, 1.5)
    window[500:900] = 3.0     # A
    window[1200:1621] = 3.0   # B -- the plain best score
    window_valid = np.ones(n, dtype=bool)
    beam_angles = np.zeros(n)

    node.effective_safety_margin = node.safety_margin
    node.gap_switch_margin = 1.2
    node.previous_target_idx = None  # nothing to stick to yet (a fresh node)
    _, _, gap_start, gap_end, _, _, _ = node._select_gap(
        window.copy(), window_valid, angle_increment, beam_angles)
    assert gap_start > 1000, 'no previous target means plain best score wins'


def test_cornering_anticipation_caps_speed_beyond_ordinary_curvature(node):
    """A gap whose near portion (<=anticipation_near_depth) points one way
    and whose overall angular span points another -- the shape a bend just
    coming into view produces, well before the car's actual steering (the
    span's midpoint) reflects it. Deliberately oversized and exaggerated
    (not a realistic LiDAR geometry) so the construction survives
    disparity_extend's inflation intact; this is a wiring test; realistic
    magnitudes are covered by gap_logic's own near_gap_bearing tests.
    Confirms the wiring end to end: _select_gap -> near_gap_bearing,
    combined with the same steering_gain/clip/tan/wheelbase/
    curvature_speed_limit chain scan_callback uses, produces a cap below
    what the ordinary curvature-only reading (from target_angle alone)
    would give."""
    # A realistic 1081-beam, 270deg scan (Hokuyo UST-10LX geometry), so the
    # angles involved stay physically plausible even though the scene
    # itself (a symmetric near-left/far-right split) is a stress-test
    # shape rather than a real corridor.
    n = 1081
    angle_increment = math.radians(270) / (n - 1)
    beam_angles = -math.radians(135) + np.arange(n, dtype=np.float64) * angle_increment
    mid = n // 2
    window = np.full(n, 1.5)   # background, below min_gap_distance (2.0)
    window[mid - 150:mid] = 2.2   # near (<= anticipation_near_depth default 2.5): left
    window[mid:mid + 150] = 4.0   # far: right
    window_valid = np.ones(n, dtype=bool)

    node.effective_safety_margin = node.safety_margin
    node.previous_target_idx = None
    (_, _, gap_start, gap_end, _, target,
     near_bearing) = node._select_gap(window, window_valid, angle_increment, beam_angles)
    assert gap_start is not None and near_bearing is not None
    target_angle = beam_angles[target]
    assert near_bearing < target_angle, 'near view should lag the wider far span'

    def capped_speed(delta):
        steering = float(np.clip(node.steering_gain * delta,
                                 -node.max_steering_angle, node.max_steering_angle))
        return gap_logic.curvature_speed_limit(
            math.tan(steering) / node.wheelbase, node.max_lateral_accel, node.max_speed)

    curve_only = capped_speed(target_angle)
    anticipated = capped_speed(target_angle - near_bearing)
    assert anticipated < curve_only, (
        'a genuine near/far disagreement must cap speed further than the '
        'ordinary curvature reading alone')


def test_cornering_anticipation_near_depth_must_exceed_zero():
    rclpy.init(args=['--ros-args', '-p', 'anticipation_near_depth:=0.0'])
    try:
        with pytest.raises(ValueError, match='anticipation_near_depth'):
            GapFollowNode()
    finally:
        rclpy.shutdown()
