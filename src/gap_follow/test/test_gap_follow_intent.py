"""
Integration tests for gap_follow's /drive_intent publishing.

The point of this file is not that the arrow looks nice -- it is that a
diagnostics feature bolted onto a node that steers a physical car obeys
the three rules in docs/drive-intent.md:

  1. intent goes out only *after* the drive command for the tick,
  2. a bug in intent generation disables intent, never the node, and
  3. intent reads the control path and never writes to it.

Rules 1 and 2 are the ones with teeth, and both are tested here by
breaking intent generation on purpose and checking the car still drives.

Needs ROS2 sourced and the package built -- not a bare `pytest`:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/gap_follow/test/test_gap_follow_intent.py -v
"""
import json
import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, LaserScan

from drive_intent import schema
from gap_follow.gap_follow_node import GapFollowNode


DEADMAN_BUTTON = 4


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args',
                     '-p', 'min_gap_distance:=2.0',
                     '-p', 'fallback_min_gap_distance:=0.8',
                     '-p', 'min_speed:=0.8',
                     '-p', 'max_speed:=2.0',
                     # Every scan should produce an intent message, so the
                     # tests below never have to reason about the throttle.
                     '-p', 'intent_rate_hz:=0.0'])
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


def _ready(node, speed=0.0):
    joy = Joy()
    joy.buttons = [0] * (DEADMAN_BUTTON + 1)
    joy.buttons[DEADMAN_BUTTON] = 1
    node.joy_callback(joy)
    odom = Odometry()
    odom.twist.twist.linear.x = float(speed)
    node.odom_callback(odom)


def _capture_intent(node):
    """Record every payload the node puts on /drive_intent."""
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

    def spy(msg):
        seen.append(msg)
        return original(msg)

    node.drive_pub.publish = spy
    return seen


# ============================================================================
# What gets published while driving
# ============================================================================

def test_a_driving_tick_publishes_a_valid_intent(node):
    intents = _capture_intent(node)
    _ready(node)
    node.scan_callback(_scan())
    assert len(intents) == 1
    assert schema.validate(intents[0]) is None
    assert intents[0]['state'] == 'gap_follow'
    assert intents[0]['severity'] == 'drive'
    assert intents[0]['node'] == 'gap_follow_node'
    assert intents[0]['frame'] == 'base_link'


def test_the_intent_reports_the_command_that_was_actually_published(node):
    """If these two could drift apart the arrow would be fiction, and the
    dashboard would be quietly lying about a moving car."""
    intents = _capture_intent(node)
    drives = _capture_drive(node)
    _ready(node)
    node.scan_callback(_scan())
    assert intents[-1]['commanded_speed'] == pytest.approx(
        drives[-1].drive.speed, abs=1e-3)
    assert intents[-1]['commanded_steering'] == pytest.approx(
        drives[-1].drive.steering_angle, abs=1e-4)


def _length(payload, key='path'):
    pts = payload[key]
    return sum(math.hypot(b['x'] - a['x'], b['y'] - a['y'])
               for a, b in zip(pts, pts[1:]))


def test_the_arrow_length_is_distance_covered_over_the_horizon(node):
    """The arrow's length is the feature: it has to mean 'this far, this
    fast', not be a fixed decoration."""
    intents = _capture_intent(node)
    _ready(node)
    node.scan_callback(_scan())
    payload = intents[-1]
    assert _length(payload) == pytest.approx(
        payload['desired_speed'] * payload['horizon_s'], rel=0.02)


def test_the_intent_arrow_shows_the_plan_not_the_acceleration_ramp(node):
    """This is the distinction the whole feature rests on. On a clear
    straight gap_follow *wants* max speed from the very first scan, and
    only the acceleration ramp keeps the command below it. The intent
    arrow must therefore be full length immediately -- if it grew in step
    with the car it would just be re-drawing measured speed, which is
    already on screen and is not what anyone needs to see."""
    intents = _capture_intent(node)
    _ready(node)
    for _ in range(30):        # let the acceleration ramp climb
        node.scan_callback(_scan())

    assert intents[0]['desired_speed'] == pytest.approx(intents[-1]['desired_speed'])
    assert _length(intents[0]) == pytest.approx(_length(intents[-1]), rel=1e-3)


def test_the_ghost_path_is_what_grows_with_the_acceleration_ramp(node):
    """...and the gap between the two is exactly the command shaping,
    which is what the dashed ghost line exists to make visible."""
    intents = _capture_intent(node)
    _ready(node)
    for _ in range(30):
        node.scan_callback(_scan())

    assert intents[-1]['commanded_speed'] > intents[0]['commanded_speed']
    assert _length(intents[-1], 'commanded_path') > _length(intents[0], 'commanded_path')
    # Still catching up: the plan is longer than what the car will do next.
    assert _length(intents[0], 'commanded_path') < _length(intents[0])


def test_the_path_speed_profile_is_what_drives_the_arrow_width(node):
    intents = _capture_intent(node)
    _ready(node)
    node.scan_callback(_scan())
    speeds = [p['v'] for p in intents[-1]['path']]
    assert speeds
    assert all(v == pytest.approx(intents[-1]['desired_speed'], abs=1e-2)
               for v in speeds)


def test_a_clear_corridor_names_a_gap_target_ahead_of_the_lidar(node):
    intents = _capture_intent(node)
    _ready(node)
    node.scan_callback(_scan())
    targets = intents[-1]['targets']
    assert [t['kind'] for t in targets] == ['gap_target']
    # Placed in base_link, so it must include the LIDAR's 0.33m offset
    # rather than being measured from the rear axle.
    assert targets[0]['x'] > node.laser_offset_x


def test_the_gap_wedge_spans_the_selected_gap(node):
    intents = _capture_intent(node)
    _ready(node)
    node.scan_callback(_scan())
    wedge = intents[-1]['wedge']
    assert wedge['a1'] > wedge['a0']
    assert wedge['r'] > 0.0
    assert wedge['x'] == pytest.approx(node.laser_offset_x)


def test_every_speed_ceiling_is_reported_and_exactly_one_group_binds(node):
    intents = _capture_intent(node)
    _ready(node)
    node.scan_callback(_scan())
    factors = intents[-1]['factors']
    names = {f['name'] for f in factors}
    assert {'curve cap', 'clearance cap', 'accel ceiling'} <= names
    binding = [f for f in factors if f['binding']]
    assert binding, 'something must be limiting the command'
    lowest = min(f['value'] for f in factors)
    assert all(f['value'] == pytest.approx(lowest) for f in binding)


def test_the_binding_factor_matches_the_speed_actually_commanded(node):
    """The whole diagnostic claim of the panel is 'this is the limit in
    charge'. It has to actually be the limit in charge."""
    intents = _capture_intent(node)
    _ready(node)
    node.scan_callback(_scan())
    payload = intents[-1]
    binding_value = min(f['value'] for f in payload['factors'])
    assert payload['commanded_speed'] == pytest.approx(binding_value, abs=1e-3)


# ============================================================================
# Stops
# ============================================================================

def test_a_deadman_release_publishes_a_stop_intent(node):
    intents = _capture_intent(node)
    _ready(node)
    joy = Joy()
    joy.buttons = [0] * (DEADMAN_BUTTON + 1)
    node.joy_callback(joy)
    node.scan_callback(_scan())
    assert intents[-1]['state'] == 'deadman_released'
    assert intents[-1]['severity'] == 'stop'
    assert intents[-1]['commanded_speed'] == pytest.approx(0.0)


def test_a_stop_predicts_no_movement_at_all(node):
    """A stopped car intends to stay where it is; drawing a stub arrow
    pointing somewhere would be inventing intent the car does not have."""
    intents = _capture_intent(node)
    _ready(node)
    node.scan_callback(_scan(ranges=[0.05] * 541))
    payload = intents[-1]
    assert payload['severity'] == 'stop'
    assert all(p['x'] == pytest.approx(0.0) and p['y'] == pytest.approx(0.0)
               for p in payload['path'])


def test_a_stop_still_reports_where_the_rack_is_held(node):
    """gap_follow deliberately holds the steering rack through a stop
    rather than centring it (see _stop). The dashboard draws that held
    angle, so it has to be in the message."""
    intents = _capture_intent(node)
    _ready(node)
    node.scan_callback(_scan())            # build up a non-zero basis
    node.steering_basis = 0.2
    node.scan_callback(_scan(ranges=[0.05] * 541))
    assert intents[-1]['desired_steering'] == pytest.approx(0.2, abs=1e-3)


def test_an_emergency_stop_names_its_state_not_just_a_stop(node):
    intents = _capture_intent(node)
    _ready(node)
    node.scan_callback(_scan(ranges=[0.05] * 541))
    assert intents[-1]['state'] == 'emergency_clearance'
    assert 'clearance' in intents[-1]['reason']


# ============================================================================
# Reasons
# ============================================================================

def test_a_state_transition_always_carries_its_reason(node):
    """The transition is the diagnostic event -- it is the moment someone
    is asking 'why did it just do that?'."""
    intents = _capture_intent(node)
    _ready(node)
    node.scan_callback(_scan())
    node.scan_callback(_scan(ranges=[0.05] * 541))
    assert intents[-1]['state'] != intents[0]['state']
    assert intents[-1].get('reason')


def test_a_steady_state_does_not_repeat_its_reason_every_tick(node):
    """The reason string can be expensive to build; it is attached on
    transitions and on the slow period, not at the publish rate."""
    intents = _capture_intent(node)
    _ready(node)
    for _ in range(6):
        node.scan_callback(_scan())
    assert intents[0].get('reason')
    assert any('reason' not in payload for payload in intents[1:])


def test_the_intent_reason_is_the_same_text_the_terminal_logs(node):
    intents = _capture_intent(node)
    logged = []
    node.get_logger().info = lambda msg, *a, **k: logged.append(msg)
    _ready(node)
    node.scan_callback(_scan())
    assert logged, 'the decision should have been logged'
    assert intents[-1]['reason'] in logged[-1]


# ============================================================================
# Rule 1 and rule 2: intent must never cost the car anything
# ============================================================================

def test_a_broken_intent_builder_does_not_stop_the_car_driving(node):
    """Rule 2. If this ever regresses, a bad drawing takes the steering
    with it."""
    intents = _capture_intent(node)
    drives = _capture_drive(node)
    _ready(node)

    def explode(msg):
        raise RuntimeError('simulated intent bug')

    node.intent_pub.publish = explode

    node.scan_callback(_scan())              # must not raise
    assert drives[-1].drive.speed > 0.0
    assert node.last_decision_state == 'gap_follow'
    assert intents == []


def test_sustained_intent_failure_switches_intent_off_not_the_node(node):
    drives = _capture_drive(node)
    _ready(node)

    def explode(msg):
        raise RuntimeError('simulated intent bug')

    node.intent_pub.publish = explode
    for _ in range(20):
        node.scan_callback(_scan())

    assert node._intent_failures.disabled
    assert drives[-1].drive.speed > 0.0      # still driving, unaffected


def test_the_drive_command_is_published_before_the_intent(node):
    """Rule 1. Nothing in the intent path may sit in front of a command,
    least of all a stop."""
    order = []
    original_drive = node.drive_pub.publish
    original_intent = node.intent_pub.publish
    node.drive_pub.publish = lambda m: (order.append('drive'), original_drive(m))
    node.intent_pub.publish = lambda m: (order.append('intent'), original_intent(m))
    _ready(node)

    node.scan_callback(_scan())
    assert order == ['drive', 'intent']

    order.clear()
    node.scan_callback(_scan(ranges=[0.05] * 541))   # the stop path
    assert order == ['drive', 'intent']


def test_an_expensive_stop_reason_is_computed_at_most_once_per_tick(node):
    """gap_follow's TTC stop reason re-runs the whole gap pipeline. The
    logger and the intent publisher both want that string; memoizing is
    what stops it being paid for twice."""
    calls = []
    original = node._escape_report
    node._escape_report = lambda *a, **k: (calls.append(1), original(*a, **k))[1]
    _ready(node, speed=3.0)
    node.scan_callback(_scan(ranges=[0.6] * 541))
    assert len(calls) <= 1


# ============================================================================
# Configuration
# ============================================================================

def test_publish_intent_false_creates_no_publisher_at_all():
    rclpy.init(args=['--ros-args', '-p', 'publish_intent:=false'])
    n = GapFollowNode()
    try:
        assert n.intent_pub is None
        _ready(n)
        n.scan_callback(_scan())             # must still drive normally
        assert n.last_decision_state == 'gap_follow'
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_the_publish_rate_is_honoured():
    rclpy.init(args=['--ros-args', '-p', 'intent_rate_hz:=1.0'])
    n = GapFollowNode()
    try:
        intents = _capture_intent(n)
        _ready(n)
        for _ in range(10):                  # ten scans inside one second
            n.scan_callback(_scan())
        assert len(intents) == 1
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_the_payload_stays_small_enough_to_stream(node):
    """~1KB at 20Hz per client is fine over the LAN; an unbounded reason
    string or an unbounded path would not be."""
    _ready(node)
    sizes = []
    original = node.intent_pub.publish
    node.intent_pub.publish = lambda m: (sizes.append(len(m.data)), original(m))
    node.scan_callback(_scan())
    assert sizes[-1] < 4096


def test_the_wire_format_is_strict_json(node):
    """Not just 'json.loads accepts it' -- Python tolerates bare NaN and
    raw control characters, and browsers do not."""
    raw = []
    original = node.intent_pub.publish
    node.intent_pub.publish = lambda m: (raw.append(m.data), original(m))
    _ready(node)
    node.scan_callback(_scan())
    for text in raw:
        assert 'NaN' not in text and 'Infinity' not in text
        json.loads(text, parse_constant=lambda c: pytest.fail(f'non-JSON {c}'))
