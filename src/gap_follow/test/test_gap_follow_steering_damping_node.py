"""
Integration tests for steering damping from the IMU yaw rate, in the real
GapFollowNode (docs/gap-follow.md#steering-damping-gyro).

The term is gap_logic.steering_damping: minus the gain times the gyro's yaw
rate, added to the requested steering before the clip, the arc guard and the
slew limit. The gyro is trusted only while /imu/status says 'ok' and
/imu/data is fresh. Otherwise the term is exactly zero, and the node commands
exactly what it commands with the feature off.

GAIN here is 0.07 s, not the shipped 0.03 s (gap_follow.yaml), so the term is
large enough to measure against the undamped command. Where a test needs the
undamped command it gets it from a feature-off node run on the same scan, so
the oracle is the node's own feature-off behaviour, and the damping itself is
checked against the closed form -GAIN x yaw.

The adaptive grip (docs/gap-follow.md#adaptive-grip-imu) shares this file's
IMU gate and helpers, and its node tests are at the bottom.

Needs ROS2 sourced and the package built, not a bare pytest:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/gap_follow/test/test_gap_follow_steering_damping_node.py -v
"""
import json
import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, Joy, LaserScan
from std_msgs.msg import String

from drive_intent import schema
from gap_follow.gap_follow_node import GapFollowNode

# Every node these tests build publishes its drive commands here, never on
# the real /drive (see test_gap_follow_node.py for why). It goes LAST in every
# rclpy.init: test_gap_follow_node.py's test_no_test_here_builds_a_node_on_the_
# real_drive checks every rclpy.init in this directory for it.
TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']

DEADMAN_BUTTON = 4
GAIN = 0.07           # the test gain; the shipped value is 0.03 (gap_follow.yaml)
MAX_STEERING = 0.26
# The slew limit and the clip are kept out of the way here, so the steering a
# test reads is the requested steering, except where a test says otherwise.
BASE = [
    '-p', 'min_gap_distance:=2.0',
    '-p', 'fallback_min_gap_distance:=0.8',
    '-p', 'min_speed:=0.8',
    '-p', 'max_speed:=2.0',
    '-p', f'max_steering_angle:={MAX_STEERING}',
    '-p', 'max_steering_rate:=100.0',
    # Every scan publishes an intent, so a test can read the reason it gives.
    '-p', 'intent_rate_hz:=0.0',
]
DAMPED = ['-p', 'enable_steering_damping:=true',
          '-p', f'steering_damping_gain:={GAIN}']


@pytest.fixture
def node():
    """A real GapFollowNode with steering damping ON at GAIN, in its own context.

    Nothing is fed yet: each test says what the deadman, odometry and IMU are."""
    rclpy.init(args=['--ros-args', *BASE, *DAMPED, *TEST_ONLY_DRIVE])
    n = GapFollowNode()
    assert n.drive_pub.topic_name == '/test_only/drive'
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


def _blocked_right(n=541):
    """An obstacle filling the right half: a correct node steers left (positive)."""
    ranges = [8.0] * n
    for i in range(0, 250):
        ranges[i] = 1.0
    return ranges


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


def _feed_imu(node, yaw_rate=None, state='ok', age_sec=0.0):
    """What imu_bridge would send: one /imu/status verdict and one /imu/data
    sample. yaw_rate=None sends no sample; state=None sends no verdict.
    age_sec makes the sample that much older than the node's clock."""
    if state is not None:
        node.imu_status_callback(String(data=json.dumps(
            {'state': state, 'reason': None, 'imu': 'bno'})))
    if yaw_rate is not None:
        msg = Imu()
        msg.angular_velocity.z = float(yaw_rate)
        node.imu_callback(msg)
        if age_sec:
            node.last_imu_time = node.get_clock().now() - Duration(seconds=age_sec)


def _capture(node):
    published = []
    original = node.drive_pub.publish
    node.drive_pub.publish = lambda msg: (published.append(msg), original(msg))
    return published


def _capture_intent(node):
    """Record every payload the node puts on /drive_intent, decoded."""
    seen = []
    original = node.intent_pub.publish

    def spy(msg):
        seen.append(schema.decode(msg.data))
        return original(msg)

    node.intent_pub.publish = spy
    return seen


def _drive_once(extra, *, imu=None, ranges=None, speed=0.0):
    """One scan into a fresh node built with `extra` parameters, after LB is
    held and odometry says `speed`. `imu` is a dict of _feed_imu's keyword
    arguments, or None to give the node no IMU at all. Returns the drive
    commands it published and the intent payloads it put out."""
    rclpy.init(args=['--ros-args', *BASE, *extra, *TEST_ONLY_DRIVE])
    node = GapFollowNode()
    try:
        assert node.drive_pub.topic_name == '/test_only/drive'
        published = _capture(node)
        intents = _capture_intent(node)
        _ready(node, speed)
        if imu is not None:
            _feed_imu(node, **imu)
        node.scan_callback(_scan(_blocked_right() if ranges is None else ranges))
        return published, intents
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _steering(published):
    return published[-1].drive.steering_angle


# ============================================================================
# A healthy gyro: the command moves by exactly -GAIN x yaw
# ============================================================================

def test_a_healthy_imu_moves_the_command_by_minus_gain_times_yaw():
    # oracle: closed form on the difference. The feature-off run on the same
    # scan is the undamped command; the term at yaw 1.0 rad/s is -GAIN rad.
    undamped = _steering(_drive_once([])[0])
    published, _intents = _drive_once(DAMPED, imu=dict(yaw_rate=1.0))
    damped = _steering(published)
    assert damped == pytest.approx(undamped - GAIN * 1.0, abs=1e-9)  # rad
    assert damped < undamped, 'a left yaw rate must steer the command right'


def test_the_damped_request_is_clipped_with_the_rest_of_the_request():
    # The term joins the request before the clip (gap_logic's requested_steering
    # line), so a large term cannot push the command past max_steering_angle.
    # oracle: the configured limit. Added after the clip, a -5 rad/s yaw (term
    # +0.35 rad on top of the request) would publish about 0.6 rad.
    published, _intents = _drive_once(DAMPED, imu=dict(yaw_rate=-5.0))
    assert _steering(published) == pytest.approx(MAX_STEERING, abs=1e-9)  # rad


def test_a_healthy_imu_is_named_in_the_intent_reason():
    _published, intents = _drive_once(DAMPED, imu=dict(yaw_rate=1.0))
    reason = intents[-1].get('reason', '')
    assert 'steering damping' in reason
    assert 'steering_damping_off' not in reason


# ============================================================================
# A bad gyro: exactly zero damping, the feature-off command, and the reason
# ============================================================================

BAD_IMU = [
    pytest.param(dict(yaw_rate=1.0, state='fault'), 'imu fault', id='fault'),
    pytest.param(dict(yaw_rate=1.0, state='waiting'), 'imu waiting', id='waiting'),
    pytest.param(dict(yaw_rate=1.0, state='ok', age_sec=0.2), 'imu stale', id='stale'),
    pytest.param(dict(yaw_rate=None, state='ok'), 'no imu', id='no-data'),
    pytest.param(dict(yaw_rate=1.0, state=None), 'no imu', id='no-status'),
    pytest.param(dict(yaw_rate=None, state=None), 'no imu', id='never-received'),
    pytest.param(dict(yaw_rate=math.nan, state='ok'), 'imu yaw not finite', id='nan-yaw'),
]


@pytest.mark.parametrize('imu, why', BAD_IMU)
def test_a_bad_imu_adds_exactly_zero_damping_and_the_car_still_drives(imu, why):
    # Review Focus 5 (docs/gap-follow.md#steering-damping-gyro): with the feature
    # on and a bad gyro the command is the feature-off command, finite, and the
    # car keeps driving. The reason names what is wrong.
    undamped = _steering(_drive_once([])[0])
    published, intents = _drive_once(DAMPED, imu=imu)
    steering = _steering(published)
    assert math.isfinite(steering)
    assert steering == pytest.approx(undamped, abs=1e-12)  # rad: zero damping
    assert published[-1].drive.speed > 0.0, 'a bad gyro must not stop the car'
    assert f'steering_damping_off: {why}' in intents[-1].get('reason', '')


def test_with_damping_off_the_imu_is_not_read_at_all():
    # The feature is off, so a healthy gyro changes nothing and is not named.
    baseline, _ = _drive_once([])
    published, intents = _drive_once([], imu=dict(yaw_rate=1.0))
    assert _steering(published) == _steering(baseline)
    assert 'damping' not in intents[-1].get('reason', '')


# ============================================================================
# The deadman, with steering damping on: the four deny-path tests
# ============================================================================

def test_damping_on_still_drives_with_lb_held(node):
    published = _capture(node)
    _ready(node)
    _feed_imu(node, yaw_rate=1.0)
    node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed > 0.0
    assert node.last_decision_state == 'gap_follow'


def test_damping_on_refuses_to_drive_when_the_deadman_is_released(node):
    published = _capture(node)
    _ready(node)
    _feed_imu(node, yaw_rate=1.0)
    _hold_deadman(node, held=False)
    node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'deadman_released'


def test_damping_on_refuses_to_drive_before_any_joy_message(node):
    published = _capture(node)
    _feed_odom(node)
    _feed_imu(node, yaw_rate=1.0)
    node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'waiting_for_joy'


def test_damping_on_stops_when_the_joy_stream_goes_stale(node):
    published = _capture(node)
    _ready(node)
    _feed_imu(node, yaw_rate=1.0)
    node.last_joy_time = node.get_clock().now() - Duration(
        seconds=node.joy_timeout_sec + 0.1)
    node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'joy_stale'


# ============================================================================
# Refusing a configuration that would steer with the rotation
# ============================================================================

def test_a_negative_gain_refuses_to_start():
    # oracle: spec -- docs/gap-follow.md#steering-damping-gyro: the gain is a
    # non-negative time constant. A negative one would make the term steer
    # with the rotation.
    rclpy.init(args=['--ros-args', *BASE, '-p', 'enable_steering_damping:=true',
                     '-p', 'steering_damping_gain:=-0.07', *TEST_ONLY_DRIVE])
    try:
        with pytest.raises(ValueError, match='steering_damping_gain'):
            GapFollowNode()
    finally:
        rclpy.shutdown()


def test_a_non_positive_imu_timeout_refuses_to_start():
    # oracle: spec -- a timeout must be a positive time; zero would call every
    # sample stale and silently disable the term.
    rclpy.init(args=['--ros-args', *BASE, *DAMPED, '-p', 'imu_timeout_sec:=0.0',
                     *TEST_ONLY_DRIVE])
    try:
        with pytest.raises(ValueError, match='imu_timeout_sec'):
            GapFollowNode()
    finally:
        rclpy.shutdown()


# ============================================================================
# Adaptive grip (docs/gap-follow.md#adaptive-grip-imu)
#
# The lateral limit behind the curvature caps starts at max_lateral_accel and is
# judged from the gyro. The node's own default for max_lateral_accel is 1.0, so
# these tests set the spec's base, 2.0 (GRIP_BASE). The scene is the
# blocked-right one above, at 2 m/s: the rack asks for 12 degrees, the curve cap
# binds below max_speed, and the clearance cap does not.
# ============================================================================

GRIP_BASE = BASE + ['-p', 'max_lateral_accel:=2.0']
GRIP = ['-p', 'enable_adaptive_grip:=true']
GRIP_DT = 0.025                 # one control tick, 40 Hz
WHEELBASE = 0.36                # the node's default, and gap_follow.yaml's
SPEED = 2.0                     # m/s, from odometry
RACK = math.radians(12.0)       # rad: the steering the scene asks for (pinned below)


@pytest.fixture
def grip_node():
    """A real GapFollowNode with the adaptive grip ON, in its own context."""
    rclpy.init(args=['--ros-args', *GRIP_BASE, *GRIP, *TEST_ONLY_DRIVE])
    n = GapFollowNode()
    assert n.drive_pub.topic_name == '/test_only/drive'
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _rack():
    """The steering the rack asks for on the blocked-right scene at SPEED, from
    a feature-off node. max_lateral_accel does not change it."""
    return _steering(_drive_once([], speed=SPEED)[0])


def _turn_yaw(steering):
    """The yaw rate of a car making the turn `steering` asks for: no slip, so
    speed * tan(steering) / wheelbase."""
    return SPEED * math.tan(steering) / WHEELBASE


def _grip_tick(node, scan, k, *, imu=None):
    """One control tick on a live node: LB held, odometry at SPEED, the IMU
    verdict `imu` (_feed_imu's keyword arguments, or None for no IMU at all),
    then the scan.

    Returns the command interval this tick used, in seconds. The test sets the
    previous command's time back by GRIP_DT, but the node reads its clock after
    its own scan processing, which takes real time, so the interval is measured
    from the node's own clock: the time the node stamped on its command, less the
    time the test set. The closed forms in the tests sum these intervals."""
    set_at = None
    if k:
        set_at = node.get_clock().now() - Duration(seconds=GRIP_DT)
        node.last_command_time = set_at
    _ready(node, SPEED)
    if imu is not None:
        _feed_imu(node, **imu)
    node.scan_callback(scan)
    if set_at is None:
        return node.command_slew_max_dt   # no earlier command: the node's cap
    since = (node.last_command_time - set_at).nanoseconds / 1e9
    return min(node.command_slew_max_dt, since)


def _run_grip(extra, *, imu=None, ticks=1, ranges_of=None):
    """A fresh node with GRIP_BASE and `extra`, ticked `ticks` times on the
    blocked-right scan, or on ranges_of(k) for tick k. `imu` is _feed_imu's
    keyword arguments for every tick, or a function of the tick index that
    returns them. Returns the commands published, the intent payloads, the
    command interval of each tick, and the adaptive limit after each tick."""
    rclpy.init(args=['--ros-args', *GRIP_BASE, *extra, *TEST_ONLY_DRIVE])
    node = GapFollowNode()
    try:
        assert node.drive_pub.topic_name == '/test_only/drive'
        published = _capture(node)
        intents = _capture_intent(node)
        intervals, limits = [], []
        for k in range(ticks):
            scan = _scan(_blocked_right() if ranges_of is None else ranges_of(k))
            intervals.append(_grip_tick(node, scan, k, imu=imu(k) if callable(imu) else imu))
            limits.append(node.adaptive_grip.limit)
        return dict(published=published, intents=intents, intervals=intervals,
                    limits=limits)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_the_rack_on_the_scene_is_twelve_degrees():
    # oracle: measured on this scene -- the node aims at the gap's +12 degree
    # bearing, the rack is not clipped (max_steering_angle is 0.26 rad), and
    # the gap is preferred. Every other adaptive test is built on this value.
    assert _rack() == pytest.approx(RACK, abs=1e-9)  # rad


# --- The rise and the cut, on a live node -----------------------------------

def test_a_healthy_gyro_raises_the_curve_capped_speed_over_time():
    ticks = 40
    imu = dict(yaw_rate=_turn_yaw(RACK))
    on = _run_grip(GRIP, imu=imu, ticks=ticks)
    off = _run_grip([], imu=imu, ticks=ticks)
    # oracle: closed form. On the turn the rack asks for, every judged tick has
    # grip ratio 1.0 and a demand (2.36 m/s^2) above 0.8 of the limit, so the
    # limit is min(3.0, 2.0 + 0.2 * t). t is the sum of the command intervals
    # from the first judged tick on: the yaw is judged against the steering a
    # lag ago, so nothing is judged until that lag is recorded (_judged_start).
    t = sum(on['intervals'][_judged_start(on['intervals']):])
    limit = min(3.0, 2.0 + 0.2 * t)
    assert on['limits'][-1] == pytest.approx(limit, abs=1e-3)  # m/s^2 (1 mm/s^2)
    # oracle: closed form. The speed is the curve cap at that limit,
    # sqrt(limit / curvature), which binds below max_speed.
    curvature = math.tan(RACK) / WHEELBASE
    speeds = [p.drive.speed for p in on['published']]
    assert speeds[-1] == pytest.approx(min(2.0, math.sqrt(limit / curvature)), abs=1e-3)  # m/s
    assert speeds == sorted(speeds) and speeds[-1] > speeds[0] + 0.05, speeds
    # With the feature off the base cap holds for the whole run.
    assert all(p.drive.speed == pytest.approx(math.sqrt(2.0 / curvature), abs=1e-9)
               for p in off['published'])


def test_a_sustained_slip_cuts_the_raised_limit_and_stops_at_the_base():
    # A hundred ticks at full grip (about ninety-four of them judged) raise the
    # limit above 2.4. Then the car reads half the turn (ratio 0.5). The ratio is
    # low-passed over 0.3 s, so the first slip tick alone is not enough to cut:
    # the cuts start a few ticks later. oracle: spec -- each cut is cut_factor
    # (0.85) of the limit and never goes below the base.
    yaw = _turn_yaw(RACK)
    run = _run_grip(GRIP, imu=lambda k: dict(
        yaw_rate=yaw if k < 100 else 0.5 * yaw, state='ok'), ticks=120)
    limits = run['limits']
    assert limits[99] > 2.4  # m/s^2: risen, so a cut is not the floor
    cuts = [k for k in range(1, len(limits)) if limits[k] < limits[k - 1]]
    assert cuts, 'a sustained slip must cut the limit'
    assert cuts[0] > 100, 'the filter must see the slip before the first cut'
    first = cuts[0]
    assert limits[first] == pytest.approx(max(2.0, limits[first - 1] * 0.85), abs=1e-9)  # m/s^2
    assert min(limits) == pytest.approx(2.0, abs=1e-12)  # m/s^2: the base, reached and held


# --- The fallback: an untrusted gyro is the feature-off command ---------------

BAD_GYRO = [
    pytest.param('fault', 'imu fault', id='fault'),
    pytest.param('waiting', 'imu waiting', id='waiting'),
    pytest.param('stale', 'imu stale', id='stale'),
    pytest.param('no-data', 'no imu', id='no-data'),
    pytest.param('no-status', 'no imu', id='no-status'),
    pytest.param('never-received', 'no imu', id='never-received'),
    pytest.param('nan-yaw', 'imu yaw not finite', id='nan-yaw'),
]


def _bad_gyro(kind, yaw):
    """_feed_imu's arguments for one way the gyro can fail. `yaw` is the turn a
    healthy gyro would show, so a gate that let the failure through would raise
    the limit and the test would see it."""
    return {
        'fault': dict(yaw_rate=yaw, state='fault'),
        'waiting': dict(yaw_rate=yaw, state='waiting'),
        'stale': dict(yaw_rate=yaw, state='ok', age_sec=0.2),
        'no-data': dict(yaw_rate=None, state='ok'),
        'no-status': dict(yaw_rate=yaw, state=None),
        'never-received': dict(yaw_rate=None, state=None),
        'nan-yaw': dict(yaw_rate=math.nan, state='ok'),
    }[kind]


@pytest.mark.parametrize('kind, why', BAD_GYRO)
def test_a_bad_gyro_publishes_the_fixed_two_point_zero_command(kind, why):
    # Review Focus 5 applies here too: the command is finite and the car keeps
    # driving. oracle: closed form -- the curve cap at the base, sqrt(2.0 /
    # curvature), which is also what the feature-off node publishes.
    on = _run_grip(GRIP, imu=_bad_gyro(kind, _turn_yaw(RACK)))
    off = _run_grip([], imu=_bad_gyro(kind, _turn_yaw(RACK)))
    drive = on['published'][-1].drive
    curvature = math.tan(RACK) / WHEELBASE
    assert drive.speed == pytest.approx(math.sqrt(2.0 / curvature), abs=1e-9)  # m/s
    assert drive.speed == pytest.approx(off['published'][-1].drive.speed, abs=1e-12)  # m/s
    assert drive.steering_angle == pytest.approx(
        off['published'][-1].drive.steering_angle, abs=1e-12)  # rad
    assert math.isfinite(drive.steering_angle) and drive.speed > 0.0
    assert on['limits'] == [pytest.approx(2.0, abs=1e-12)]  # m/s^2: the base
    assert f'adaptive_grip_off: {why}' in on['intents'][-1].get('reason', '')


def test_a_bad_gyro_restarts_the_low_pass_as_well_as_the_limit():
    # The ratio filter forgets across an untrusted gyro. Ten slip ticks leave it
    # near 0.5, then a fault, then one full-grip tick. oracle: closed form -- the
    # healthy tick seeds the filter at its own ratio, 1.0, and rises from the
    # base by 0.2 times its interval. Had the filter kept its slip value, the
    # blend would still read as a slip, and the limit would be cut to the base.
    yaw = _turn_yaw(RACK)

    def imu(k):
        if k < 10:
            return dict(yaw_rate=0.5 * yaw, state='ok')
        return dict(yaw_rate=yaw, state='fault' if k == 10 else 'ok')

    run = _run_grip(GRIP, imu=imu, ticks=12)
    assert run['limits'][10] == pytest.approx(2.0, abs=1e-12)  # m/s^2: the fault
    assert run['limits'][11] == pytest.approx(
        2.0 + 0.2 * run['intervals'][11], abs=1e-9)  # m/s^2: judged on its own


def test_a_bad_gyro_after_a_rise_restarts_the_limit_at_the_base():
    # oracle: closed form. Twenty judged ticks raise the limit to
    # 2.0 + 0.2 * t, t the sum of their command intervals. Tick 21 (index 20)
    # is a fault: the limit is back at the base, and so is the speed. Tick 22
    # (index 21) is healthy again and rises from the base by 0.2 * its interval,
    # so the filter restarted too.
    yaw = _turn_yaw(RACK)
    run = _run_grip(GRIP, imu=lambda k: dict(
        yaw_rate=yaw, state='fault' if k == 20 else 'ok'), ticks=22)
    risen = 2.0 + 0.2 * sum(run['intervals'][_judged_start(run['intervals']):20])
    assert run['limits'][19] == pytest.approx(risen, abs=1e-9)  # m/s^2
    assert run['limits'][20] == pytest.approx(2.0, abs=1e-12)  # m/s^2: reset
    curvature = math.tan(RACK) / WHEELBASE
    assert run['published'][20].drive.speed == pytest.approx(
        math.sqrt(2.0 / curvature), abs=1e-9)  # m/s
    assert run['limits'][21] == pytest.approx(
        2.0 + 0.2 * run['intervals'][21], abs=1e-9)  # m/s^2


def test_with_the_grip_off_a_healthy_gyro_changes_nothing_and_is_not_named():
    # The feature is off, so the gyro is not read at all.
    plain = _run_grip([], imu=None, ticks=5)
    run = _run_grip([], imu=dict(yaw_rate=_turn_yaw(RACK)), ticks=5)
    assert [p.drive.speed for p in run['published']] == [
        p.drive.speed for p in plain['published']]
    assert [p.drive.steering_angle for p in run['published']] == [
        p.drive.steering_angle for p in plain['published']]
    assert run['limits'] == [2.0] * 5
    # The reason is resolved on the first tick (a state change), so that is the
    # one to read: a missing reason would make the check vacuous.
    reason = run['intents'][0].get('reason', '')
    assert 'curve cap' in reason, reason   # every decision reason carries it
    assert 'adaptive' not in reason


def test_a_trusted_gyro_is_named_in_the_intent_reason():
    run = _run_grip(GRIP, imu=dict(yaw_rate=_turn_yaw(RACK)), ticks=2)
    reason = run['intents'][0].get('reason', '')
    assert 'adaptive grip limit' in reason
    assert 'adaptive_grip_off' not in reason


# --- The deadman, with the adaptive grip on: the four deny-path tests ---------

def test_grip_on_still_drives_with_lb_held(grip_node):
    published = _capture(grip_node)
    _ready(grip_node)
    _feed_imu(grip_node, yaw_rate=1.0)
    grip_node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed > 0.0
    assert grip_node.last_decision_state == 'gap_follow'


def test_grip_on_refuses_to_drive_when_the_deadman_is_released(grip_node):
    published = _capture(grip_node)
    _ready(grip_node)
    _feed_imu(grip_node, yaw_rate=1.0)
    _hold_deadman(grip_node, held=False)
    grip_node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed == 0.0
    assert grip_node.last_decision_state == 'deadman_released'


def test_grip_on_refuses_to_drive_before_any_joy_message(grip_node):
    published = _capture(grip_node)
    _feed_odom(grip_node)
    _feed_imu(grip_node, yaw_rate=1.0)
    grip_node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed == 0.0
    assert grip_node.last_decision_state == 'waiting_for_joy'


def test_grip_on_stops_when_the_joy_stream_goes_stale(grip_node):
    published = _capture(grip_node)
    _ready(grip_node)
    _feed_imu(grip_node, yaw_rate=1.0)
    grip_node.last_joy_time = grip_node.get_clock().now() - Duration(
        seconds=grip_node.joy_timeout_sec + 0.1)
    grip_node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed == 0.0
    assert grip_node.last_decision_state == 'joy_stale'


# --- Wiring and startup ------------------------------------------------------

def test_the_adaptive_grip_subscribes_to_the_gyro_on_its_own(grip_node):
    # Steering damping is off here, and the grip still needs the gyro topics.
    assert grip_node.imu_sub.topic_name == '/imu/data'
    assert grip_node.imu_status_sub.topic_name == '/imu/status'


def test_with_both_gyro_features_off_the_node_has_no_imu_subscriptions():
    rclpy.init(args=['--ros-args', *BASE, *TEST_ONLY_DRIVE])
    try:
        node = GapFollowNode()
        try:
            assert not hasattr(node, 'imu_sub')
            assert not hasattr(node, 'imu_status_sub')
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('extra, match', [
    (['-p', 'adaptive_grip_ceiling:=0.0'], 'adaptive_grip_ceiling'),
    (['-p', 'adaptive_grip_rise_per_s:=-0.2'], 'adaptive_grip_rise_per_s'),
    (['-p', 'adaptive_grip_cut_factor:=1.5'], 'adaptive_grip_cut_factor'),
    (['-p', 'adaptive_grip_cut_factor:=0.0'], 'adaptive_grip_cut_factor'),
    # Equal to the default hold ratio of 0.95: the two bands would overlap.
    (['-p', 'adaptive_grip_slip_ratio:=0.95'], 'adaptive_grip_slip_ratio'),
    (['-p', 'adaptive_grip_judge_fraction:=-0.1'], 'adaptive_grip_judge_fraction'),
    (['-p', 'adaptive_grip_steering_lag_sec:=-0.1'], 'adaptive_grip_steering_lag_sec'),
    (['-p', 'adaptive_grip_max_steering_change:=-0.05'], 'adaptive_grip_max_steering_change'),
])
def test_a_bad_grip_parameter_refuses_to_start(extra, match):
    # oracle: spec -- a band that cannot hold is refused at startup, not
    # discovered on the floor. Checked with the feature off as well.
    rclpy.init(args=['--ros-args', *GRIP_BASE, *extra, *TEST_ONLY_DRIVE])
    try:
        with pytest.raises(ValueError, match=match):
            GapFollowNode()
    finally:
        rclpy.shutdown()


def test_a_live_change_of_max_lateral_accel_restarts_the_limit_at_the_new_base(grip_node):
    # max_lateral_accel is live-tunable and is the floor, so the grip follows it
    # on the next tick. oracle: closed form -- the limit restarts at the new base
    # and that tick rises by 0.2 times its command interval, as a fresh grip would.
    scan = _scan(_blocked_right())
    imu = dict(yaw_rate=_turn_yaw(RACK))
    for k in range(20):
        _grip_tick(grip_node, scan, k, imu=imu)
    # Fourteen judged ticks (the lag is recorded first) add about 0.07 m/s^2.
    assert grip_node.adaptive_grip.limit > 2.05  # m/s^2: risen
    results = grip_node.set_parameters(
        [Parameter('max_lateral_accel', Parameter.Type.DOUBLE, 2.4)])
    assert results[0].successful, results[0].reason
    dt = _grip_tick(grip_node, scan, 20, imu=imu)
    assert grip_node.adaptive_grip.base == pytest.approx(2.4, abs=1e-12)  # m/s^2
    assert grip_node.adaptive_grip.limit == pytest.approx(
        2.4 + 0.2 * dt, abs=1e-9)  # m/s^2


# --- The lag, and the steady-steering gate (docs/gap-follow.md#adaptive-grip-imu)

STEERING_LAG = 0.15            # s: adaptive_grip_steering_lag_sec's shipped value


def _judged_start(intervals):
    """The first tick the node can judge: the first whose window's edge, a lag
    before it on the control clock, reaches back to the first sample. The clock is
    the running sum of the intervals the node used, as gap_logic.SteeringLag keeps it."""
    clock, times = 0.0, []
    for dt in intervals:
        clock += dt
        times.append(clock)
    return next(k for k, t in enumerate(times) if t - STEERING_LAG >= times[0] - 1e-9)


def _blocked_left(n=541):
    """An obstacle filling the left half: a correct node steers right (negative)."""
    ranges = [8.0] * n
    for i in range(n - 250, n):
        ranges[i] = 1.0
    return ranges


def _rack_on(ranges):
    """The steering a feature-off node asks for on a scene, at SPEED."""
    return _steering(_drive_once([], ranges=ranges, speed=SPEED)[0])


def test_no_judgement_until_the_history_reaches_back_a_lag():
    # oracle: spec -- the yaw is judged against the steering a lag ago, which does
    # not exist until the history has been recorded for that long. The first
    # judged tick is the first whose window is covered.
    run = _run_grip(GRIP, imu=dict(yaw_rate=_turn_yaw(RACK)), ticks=12)
    first = _judged_start(run['intervals'])
    span = sum(run['intervals'][1:first + 1])
    assert sum(run['intervals'][1:first]) < STEERING_LAG <= span + 1e-9  # covered only now
    assert run['limits'][:first] == [pytest.approx(2.0, abs=1e-12)] * first  # m/s^2: held
    assert run['limits'][first] > 2.0 + 1e-6   # m/s^2: the first judged tick rises


def test_a_steering_jump_holds_the_limit_for_a_lag_and_then_judging_resumes():
    # The rack's steering changes sign on tick 20. Until the window's edge is a
    # lag past the change, the steering a lag ago is the old one, far more than
    # 0.05 rad away: no judgement, so the limit holds exactly. Judging resumes on
    # the first tick whose edge is at the change. oracle: closed form on the
    # control clock, the running sum of the intervals.
    right, left = _rack_on(_blocked_right()), _rack_on(_blocked_left())
    assert abs(left - right) > 0.05, (left, right)     # rad: well over the threshold
    jump, ticks = 20, 32

    def imu(k):
        return dict(yaw_rate=_turn_yaw(right if k < jump else left), state='ok')

    run = _run_grip(GRIP, imu=imu, ticks=ticks,
                    ranges_of=lambda k: _blocked_right() if k < jump else _blocked_left())
    clock, total = [], 0.0
    for dt in run['intervals']:
        total += dt
        clock.append(total)
    reopen = next(k for k in range(jump + 1, ticks)
                  if clock[k] - clock[jump] >= STEERING_LAG - 1e-9)
    assert run['limits'][jump - 1] > 2.05          # m/s^2: risen before the change
    for k in range(jump, reopen):
        assert run['limits'][k] == pytest.approx(
            run['limits'][jump - 1], abs=1e-12)    # m/s^2: held while the window is mixed
    assert run['limits'][reopen] > run['limits'][reopen - 1]   # m/s^2: judging resumed


def test_an_untrusted_gyro_does_not_restart_the_steering_history():
    # oracle: spec -- the history is the rack's, so it keeps recording through an
    # untrusted gyro. The gyro faults on ticks 20 and 21, so the limit is back at
    # the base; on tick 22 it is healthy and the window was already covered, so the
    # limit rises at once, by 0.2 times that tick's interval.
    yaw = _turn_yaw(RACK)
    run = _run_grip(GRIP, imu=lambda k: dict(
        yaw_rate=yaw, state='fault' if k in (20, 21) else 'ok'), ticks=23)
    assert run['limits'][21] == pytest.approx(2.0, abs=1e-12)  # m/s^2: held at the base
    assert run['limits'][22] == pytest.approx(
        2.0 + 0.2 * run['intervals'][22], abs=1e-9)           # m/s^2: judged at once
