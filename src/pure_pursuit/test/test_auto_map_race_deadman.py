"""LB deadman deny-path tests for auto_map_race_node.

auto_map_race_node is a driving node: it forwards gap_follow's mapping commands
to /drive, and pure_pursuit's racing commands in race mode. It must enforce the
LB deadman itself, independently of ackermann_mux and of every other node
(CLAUDE.md, "Standing requirement -- the LB deadman"), so it gets its own tests
for the four states that rule lists. Each test asserts on what reaches
/test_only/drive, not on an internal flag.

Runner: this file is pure rclpy and needs ROS sourced and the test domain
isolated (CLAUDE.md), e.g.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/pure_pursuit/test/test_auto_map_race_deadman.py -v

Time is injected. Every freshness decision reads the clock through _now_sec(),
which these tests replace with a fake clock. Outgoing header stamps still come
from ROS time, and no decision reads them. The node's own control timer is
pushed out to 100 s, so each control tick is one explicit call to _control_loop.
enable_deadman stays at its shipped value (true) in every test here.
"""

import os
import sys
import time

from ackermann_msgs.msg import AckermannDriveStamped
import pytest
import rclpy
from sensor_msgs.msg import Joy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pure_pursuit.auto_map_race_node import AutoMapRaceNode  # noqa: E402

# oracle: spec docs/hardware-reference.md (LB = button index 4); auto_map_race.yaml agrees
LB_BUTTON = 4
# The length of a full pad's button list; LB sits inside it, not at the end.
PAD_BUTTON_COUNT = 11

# The mapping command gap_follow would send. Non-zero, so a forwarded command
# and a stop are told apart by speed alone.
MAPPING_SPEED = 0.8       # m/s
MAPPING_STEERING = 0.12   # rad

# ackermann_msgs stores speed and steering as float32, so 0.8 comes back as
# 0.800000011920929. 1e-6 is still far finer than any command the car can act on.
FLOAT32_TOL = 1e-6

# Every fed input is stamped a little after the one before it. Fake time has to
# move for a feed to be distinguishable from the previous one, and 1 ms is far
# below every freshness window (0.5 s) the node applies.
FEED_STEP_SEC = 1e-3

# How far either side of joy_timeout_sec the stale test samples the joy's age.
STALE_MARGIN_SEC = 0.05


def _joy(lb_held):
    msg = Joy()
    msg.buttons = [0] * PAD_BUTTON_COUNT
    msg.buttons[LB_BUTTON] = 1 if lb_held else 0
    return msg


def _mapping_command():
    msg = AckermannDriveStamped()
    msg.drive.speed = MAPPING_SPEED
    msg.drive.steering_angle = MAPPING_STEERING
    return msg


def _spin_until(node, predicate, timeout_sec=5.0):
    """Spin ROS until predicate() holds; False if it never does within timeout_sec.

    The bound is only a failure limit. Callers assert the result, so a message
    that never arrives fails the test instead of passing it.
    """
    deadline = time.monotonic() + timeout_sec
    while not predicate():
        if time.monotonic() >= deadline:
            return False
        rclpy.spin_once(node, timeout_sec=0.01)
    return True


def _assert_all_stops(cmds):
    """At least one command went out, and every one of them is a full stop."""
    assert len(cmds) >= 1, 'expected stop commands on /test_only/drive, got none'
    for cmd in cmds:
        # oracle: spec docs/architecture.md (no non-zero drive command without LB and a live /joy);
        # the zero itself is what the node's own stop publishes (_publish_stop -> default message).
        assert cmd.drive.speed == 0.0
        assert cmd.drive.steering_angle == 0.0


class _Rig:
    """Drive one AutoMapRaceNode from the outside.

    Inputs are real /joy and mapping-command publishes, the clock is injected,
    and the output is a real subscriber on /test_only/drive. The node's
    `last_joy_time` and `mapping_cmd_time` are read only to learn that a
    callback has run, so the next step is not taken before its input is in.
    Nothing is asserted on them.
    """

    def __init__(self, node):
        self.node = node
        self.now = 0.0
        node._now_sec = lambda: self.now
        self.joy_pub = node.create_publisher(Joy, node.joy_topic, 10)
        self.cmd_pub = node.create_publisher(
            AckermannDriveStamped, node.mapping_drive_topic, 10)
        self.received = []
        node.create_subscription(
            AckermannDriveStamped, '/test_only/drive', self.received.append, 100)
        matched = _spin_until(node, lambda: (
            node.drive_pub.get_subscription_count() >= 1
            and self.joy_pub.get_subscription_count() >= 1
            and self.cmd_pub.get_subscription_count() >= 1))
        assert matched, 'the node, its inputs and the /test_only/drive listener never matched'

    def feed_joy(self, lb_held):
        """Publish one /joy, wait until the node has taken it, return its stamp."""
        self.now += FEED_STEP_SEC
        self.joy_pub.publish(_joy(lb_held))
        assert _spin_until(self.node, lambda: self.node.last_joy_time == self.now), \
            'the node never took the /joy message'
        return self.now

    def feed_mapping_command(self):
        """Publish one fresh gap_follow command, wait until the node has taken it."""
        self.now += FEED_STEP_SEC
        self.cmd_pub.publish(_mapping_command())
        assert _spin_until(self.node, lambda: self.node.mapping_cmd_time == self.now), \
            'the node never took the mapping command'

    def advance_to(self, t):
        assert t > self.now
        self.now = t

    def tick(self):
        """One control tick: the node's own _control_loop, which its timer would call.

        Returns the commands that reached /test_only/drive during this tick.
        """
        before = len(self.received)
        self.node._control_loop()
        assert _spin_until(self.node, lambda: len(self.received) > before), \
            'the node published no drive command on this control tick'
        return self.received[before:]


@pytest.fixture
def rig():
    """Build an AutoMapRaceNode with the deadman on, wrapped in a _Rig."""
    rclpy.init(args=['--ros-args',
                     '-p', 'enable_deadman:=true',
                     '-p', 'control_rate_hz:=0.01',
                     '-p', 'decision_log_period_sec:=0.0',
                     '-p', 'drive_topic:=/test_only/drive'])
    node = AutoMapRaceNode()
    try:
        # Preconditions, so a wrong setup fails here rather than as a pass.
        assert node.enable_deadman is True
        assert node.deadman_button == LB_BUTTON
        assert node.drive_pub.topic_name == '/test_only/drive'
        yield _Rig(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_deadman_lb_held_forwards_a_nonzero_command(rig):
    """LB held on a fresh /joy: the fed mapping command goes out unchanged.

    This is the permissive side. It also shows the rig forwards when it should,
    which the three deny tests below rely on.
    """
    rig.feed_joy(lb_held=True)
    rig.feed_mapping_command()
    cmd = rig.tick()[-1].drive
    # oracle: closed form (the values this test fed in); m/s and rad.
    assert cmd.speed == pytest.approx(MAPPING_SPEED, abs=FLOAT32_TOL)
    assert cmd.steering_angle == pytest.approx(MAPPING_STEERING, abs=FLOAT32_TOL)


def test_deadman_lb_released_publishes_zero(rig):
    """LB released on a fresh /joy: the node publishes stops, never the fed command.

    Control first: the same rig forwards the command while LB is held, so the
    zeros that follow are the deadman closing, not a rig that never forwards.
    The deny path is auto_map_race_node.py `_control_step`: `if not deadman_ok:`
    calls `_publish_stop()`, which publishes an all-zero AckermannDriveStamped.
    """
    rig.feed_joy(lb_held=True)
    rig.feed_mapping_command()
    assert rig.tick()[-1].drive.speed == pytest.approx(MAPPING_SPEED, abs=FLOAT32_TOL)

    rig.feed_joy(lb_held=False)
    start = len(rig.received)
    for _ in range(3):
        rig.feed_mapping_command()   # fresh and still non-zero on every tick
        rig.tick()
    _assert_all_stops(rig.received[start:])


def test_deadman_joy_never_received_publishes_zero(rig):
    """No /joy since start-up: the fed command never goes out; every tick is a stop.

    The rig has already shown the node listens on /joy (its subscription is
    matched), so "nothing arrived" is the real condition, not "nobody listened".
    The command is fresh and non-zero throughout, so only the deadman can stop it.
    """
    start = len(rig.received)
    for _ in range(3):
        rig.feed_mapping_command()
        rig.tick()
    _assert_all_stops(rig.received[start:])


def test_deadman_joy_stale_publishes_zero(rig):
    """LB held on one /joy, then no more /joy: the node stops once that joy is too old.

    Three samples: fresh (control), just inside joy_timeout_sec, and past it with
    no new /joy. The mapping command is fed fresh at every tick, so the joy's age
    is the only thing that changes between the second sample and the third.
    The check is auto_map_race_node.py `_deadman_status`: `age_sec >= joy_timeout_sec`
    returns 'joy_stale'.
    """
    timeout = rig.node.joy_timeout_sec   # the node's own limit: 0.5 s in auto_map_race.yaml
    joy_at = rig.feed_joy(lb_held=True)
    rig.feed_mapping_command()
    assert rig.tick()[-1].drive.speed == pytest.approx(MAPPING_SPEED, abs=FLOAT32_TOL)

    rig.advance_to(joy_at + timeout - STALE_MARGIN_SEC)
    rig.feed_mapping_command()
    assert rig.tick()[-1].drive.speed == pytest.approx(MAPPING_SPEED, abs=FLOAT32_TOL)

    rig.advance_to(joy_at + timeout + STALE_MARGIN_SEC)
    start = len(rig.received)
    for _ in range(3):
        rig.feed_mapping_command()
        rig.tick()
    _assert_all_stops(rig.received[start:])
