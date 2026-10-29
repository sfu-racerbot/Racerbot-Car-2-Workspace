"""
gap_follow_node with opponent awareness (opponent_tracking.py) -- the node
wiring, the toggle, and the refusal paths. Needs ROS sourced and an isolated
domain (CLAUDE.md):

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/gap_follow/test/test_opponent_awareness_node.py -v

Every node here publishes on /test_only/drive (the remap comes after
--params-file, so the YAML's /drive cannot win). Scenes are ray-cast from
segments in the LiDAR frame; the follow-cap oracle is the closed form
(gap - follow_min_gap) / follow_time_gap from the box's true position.
"""
import math
import os
import sys

import numpy as np
import pytest
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import Joy

from drive_intent import schema
from gap_follow import opponent_tracking
from gap_follow.gap_follow_node import GapFollowNode

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_gap_follow_corners_node import _corridor, _raycast  # noqa: E402

PACKAGED_CONFIG = os.path.join(HERE, '..', 'config', 'gap_follow.yaml')
TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']
DT = 0.025
LASER_X = 0.26
FRONT = 0.36 / 2.0 + 0.58 / 2.0          # body front in base_link (0.47 m)
EGO_SPEED = 2.5
OPP_SPEED = 1.5


def _make(*extra):
    # command_slew_max_dt at exactly one tick makes the ramp interval exactly
    # DT whatever the wall clock did, as in test_gap_mirror_parity, so two
    # runs of the same scenes can be compared to float precision.
    rclpy.init(args=['--ros-args', '--params-file', PACKAGED_CONFIG,
                     '-p', f'command_slew_max_dt:={DT}', *extra, *TEST_ONLY_DRIVE])
    node = GapFollowNode()
    assert node.drive_pub.topic_name == '/test_only/drive'
    published = []
    node.drive_pub.publish = published.append
    return node, published


def _box(rear_x, y=0.0, length=0.58, width=0.30):
    """A car-sized rectangle whose rear face is rear_x ahead of the LiDAR."""
    x0, x1, hw = rear_x, rear_x + length, width / 2.0
    return [((x0, y - hw), (x0, y + hw)), ((x0, y + hw), (x1, y + hw)),
            ((x1, y + hw), (x1, y - hw)), ((x1, y - hw), (x0, y - hw))]


def _drive(node, published, ticks=40, rear_x0=2.2, lb=True, stamped=True,
           y0=0.0, vy=0.0, dt=DT, corridor_width=2.0):
    """Feed a 2 m corridor with a car ahead closing at EGO - OPP m/s.

    Odometry is held at EGO_SPEED and the rack at straight, so the ego motion
    the node dead-reckons is exactly the motion the scenes were drawn with.
    Returns [(rear_x, published speed)] per tick.
    """
    joy = Joy()
    joy.buttons = [0] * 5
    joy.buttons[4] = 1 if lb else 0
    out = []
    for k in range(ticks):
        rear_x = rear_x0 + (OPP_SPEED - EGO_SPEED) * k * dt
        y = y0 + vy * k * dt
        scan = _raycast(_corridor(corridor_width / 2, corridor_width / 2) + _box(rear_x, y))
        if stamped:
            t = 100.0 + k * dt
            scan.header.stamp.sec = int(t)
            scan.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        node.joy_callback(joy)
        odom = Odometry()
        odom.twist.twist.linear.x = EGO_SPEED
        node.odom_callback(odom)
        node.steering_basis = 0.0
        node.last_commanded_speed = EGO_SPEED
        node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
        node.scan_callback(scan)
        out.append((rear_x, y, published[-1].drive.speed))
    return out


def _closed_form_cap(rear_x, params=opponent_tracking.OpponentParams()):
    gap = rear_x + LASER_X - FRONT
    return max(0.0, (gap - params.follow_min_gap) / params.follow_time_gap)


def test_shipped_config_has_it_on_and_lists_every_field_at_its_default():
    # Approved spec change (user decision 2026-10-28, option 1): awareness ships ON,
    # with the 1 s follow-cap memory. The "every field at its default" half is unchanged.
    params = yaml.safe_load(open(PACKAGED_CONFIG))['gap_follow_node']['ros__parameters']
    assert params['enable_opponent_awareness'] is True
    defaults = opponent_tracking.OpponentParams()
    for name in opponent_tracking.OpponentParams.__dataclass_fields__:
        assert params[f'opponent_{name}'] == getattr(defaults, name), name


def test_off_builds_no_layer_and_reads_none_of_its_values():
    # Values the layer would refuse: with it off they must not even be read.
    node, published = _make('-p', 'enable_opponent_awareness:=false',
                            '-p', 'opponent_alpha:=5.0', '-p', 'opponent_gate:=-1.0')
    try:
        assert node.opponents is None
        _drive(node, published, ticks=5)
        assert node.opponent_mode is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_on_with_a_bad_value_refuses_to_start():
    rclpy.init(args=['--ros-args', '--params-file', PACKAGED_CONFIG,
                     '-p', 'enable_opponent_awareness:=true',
                     '-p', 'opponent_alpha:=5.0', *TEST_ONLY_DRIVE])
    try:
        with pytest.raises(ValueError, match=r'opponent_alpha must be in \(0, 1\]'):
            GapFollowNode()
    finally:
        rclpy.shutdown()


def _speeds(*extra, **kw):
    node, published = _make(*extra)
    try:
        return _drive(node, published, **kw), node.opponent_mode
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_on_follows_a_slower_car_ahead_at_the_follow_cap():
    on, mode = _speeds('-p', 'enable_opponent_awareness:=true')
    off, _ = _speeds('-p', 'enable_opponent_awareness:=false')
    assert mode == 'follow'       # 2 m corridor, car dead ahead: no room to pass
    # Once the track is confirmed (8 consecutive scans) and its velocity has
    # settled, the published speed is held to the closed-form follow cap.
    # 0.1 m/s allows for the alpha-beta filter's residual position lag.
    late = range(25, 40)
    for k in late:
        rear_x, _y, speed = on[k]
        assert speed <= _closed_form_cap(rear_x) + 0.1, (k, speed, _closed_form_cap(rear_x))
    # And the cap is what made the difference: the same scenes without the
    # layer command more on every one of those ticks.
    assert all(off[k][2] > on[k][2] + 0.2 for k in late)


def test_on_without_scan_stamps_does_nothing():
    # Unstamped scans give no valid interval, so the tracker resets every
    # scan and never confirms a mover: the commands match the layer off.
    on, mode = _speeds('-p', 'enable_opponent_awareness:=true', stamped=False)
    off, _ = _speeds('-p', 'enable_opponent_awareness:=false')
    assert mode is None
    assert [s for _x, _y, s in on] == pytest.approx([s for _x, _y, s in off], abs=1e-9)


def test_on_in_a_static_world_commands_exactly_what_off_commands():
    # Nothing moves: a parked car ahead, seen from a car that is moving.
    # Static means its ground position is fixed, so in the scan it closes at
    # the ego speed -- OPP_SPEED = 0 in _drive's terms.
    global OPP_SPEED
    saved, OPP_SPEED = OPP_SPEED, 0.0
    try:
        on, mode = _speeds('-p', 'enable_opponent_awareness:=true', ticks=30, rear_x0=3.5)
        off, _ = _speeds('-p', 'enable_opponent_awareness:=false', ticks=30, rear_x0=3.5)
    finally:
        OPP_SPEED = saved
    assert mode is None
    assert [s for _x, _y, s in on] == pytest.approx([s for _x, _y, s in off], abs=1e-9)


def test_lb_released_still_commands_zero_with_a_car_ahead():
    on, _mode = _speeds('-p', 'enable_opponent_awareness:=true', lb=False, ticks=20)
    assert all(speed == 0.0 for _x, _y, speed in on)


def test_a_turning_car_off_to_the_side_is_followed_not_passed():
    # Drifting left at 0.55 m/s while doing 1.5 m/s: heading 20 deg, over
    # opponent_pass_max_heading_deg (15), so no pass -- and once it is more
    # than follow_lateral (0.45 m) to the side, only the widened follow band
    # (relevant_lateral, 1.0 m) still caps the speed.
    on, mode = _speeds('-p', 'enable_opponent_awareness:=true', y0=0.2, vy=0.55)
    assert mode == 'follow'
    wide = [(rear_x, speed) for rear_x, y, speed in on[20:] if 0.45 < y + 0.0 <= 1.0]
    assert len(wide) >= 5
    for rear_x, speed in wide:
        assert speed <= _closed_form_cap(rear_x) + 0.1, (rear_x, speed)


def test_tracking_uses_the_scan_stamps_not_a_fixed_rate():
    # An 80 Hz scanner (stamps 12.5 ms apart) passing a parked car. Taking
    # the interval as anything but the stamps' difference mis-integrates this
    # car's own motion, and the parked car would appear to drive off.
    global OPP_SPEED
    saved, OPP_SPEED = OPP_SPEED, 0.0
    try:
        on, mode = _speeds('-p', 'enable_opponent_awareness:=true', ticks=60,
                           rear_x0=3.0, dt=0.0125)
    finally:
        OPP_SPEED = saved
    assert mode is None


@pytest.fixture
def aware_node():
    node, published = _make('-p', 'enable_opponent_awareness:=true',
                            '-p', 'intent_rate_hz:=0.0')
    yield node, published
    node.destroy_node()
    rclpy.shutdown()


def _follow_tick(node, published, step, gap, ego_speed, joy_state='held'):
    scan = _raycast(_corridor(2.0, 2.0) + _box(gap + FRONT - LASER_X))
    ns = round((100.0 + step * DT) * 1e9)
    scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(ns, 10**9)
    if joy_state in ('held', 'released', 'stale'):
        joy = Joy()
        joy.buttons = [0, 0, 0, 0, int(joy_state != 'released')]
        node.joy_callback(joy)
        if joy_state == 'stale':
            node.last_joy_time = node.get_clock().now() - Duration(
                seconds=node.joy_timeout_sec + DT)
    odom = Odometry()
    odom.twist.twist.linear.x = float(ego_speed)
    node.odom_callback(odom)
    node.steering_basis = 0.0
    node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
    node.scan_callback(scan)
    return published[-1].drive.speed


def test_known_stopped_car_publishes_zero_then_resumes(aware_node):
    node, published = aware_node
    intents = []
    node.intent_pub.publish = lambda msg: intents.append(schema.decode(msg.data))
    # Leader and ego roll together at 1 m/s: constant gap, real confirmation.
    for step in range(40):
        _follow_tick(node, published, step, 0.4, 1.0)
    for step in range(40, 64):
        stopped_speed = _follow_tick(node, published, step, 0.4, 0.0)
    # oracle: R2 gap-only law at 0.4 < 0.5 m, on the published command.
    assert stopped_speed == 0.0
    assert node.opponents.followed_body()
    assert node.opponent_mode == 'follow'
    # oracle: only the follow ceiling binds this zero; clearance, curvature
    # and acceleration still permit motion in the same published intent.
    factors = intents[-1]['factors']
    assert {f['name'] for f in factors if f['binding']} == {'follow cap'}
    follow = next(f for f in factors if f['name'] == 'follow cap')
    assert follow['value'] == 0.0
    assert all(f['value'] > 0.0 for f in factors if f['name'] != 'follow cap')
    assert intents[-1]['commanded_speed'] == stopped_speed
    speeds = [_follow_tick(node, published, 64 + k, 0.4 + k * DT, 0.0)
              for k in range(1, 13)]
    # oracle: leader moves 0.3 m at 1 m/s; gap now exceeds minimum by 0.2 m.
    assert speeds[-1] > 0.0


def _deadman_with_moving_car(aware_node, state, start=0, ticks=40):
    node, published = aware_node
    # A leader at 1 m/s recedes from the stationary ego.
    return [_follow_tick(node, published, step, 1.5 + step * DT, 0.0, state)
            for step in range(start, start + ticks)]


def _confirm_moving_car(aware_node):
    speeds = _deadman_with_moving_car(aware_node, 'held')
    # oracle: the awareness path really ran before testing a later denial.
    assert aware_node[0].opponents.moving_body()
    assert speeds[-1] > 0.0
    return speeds


def test_awareness_deadman_held_publishes_motion(aware_node):
    # R2 review step 5: all four revised command expectations were flipped
    # and failed, after adding moving-car confirmation and the fresh-node
    # no-Joy control. The deadman implementation itself was never modified.
    # oracle: LB policy permissive path, open road and a moving car ahead.
    assert _confirm_moving_car(aware_node)[-1] > 0.0


def test_awareness_deadman_released_publishes_zero(aware_node):
    _confirm_moving_car(aware_node)
    # oracle: CLAUDE.md safety model; released LB must deny motion.
    speeds = _deadman_with_moving_car(aware_node, 'released', start=40, ticks=4)
    assert all(speed == 0.0 for speed in speeds)


def test_awareness_deadman_never_received_publishes_zero():
    node, published = _make('-p', 'enable_opponent_awareness:=true')
    try:
        speeds = _deadman_with_moving_car((node, published), 'never')
        # oracle: CLAUDE.md safety model; every command without /joy is zero.
        assert all(speed == 0.0 for speed in speeds)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # Fresh node, identical scans, only LB differs: the scenario can drive.
    node, published = _make('-p', 'enable_opponent_awareness:=true')
    try:
        assert _confirm_moving_car((node, published))[-1] > 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_awareness_deadman_stale_publishes_zero(aware_node):
    _confirm_moving_car(aware_node)
    node, _published = aware_node
    node.last_joy_time = node.get_clock().now() - Duration(
        seconds=node.joy_timeout_sec + DT)
    # oracle: CLAUDE.md safety model; expired /joy must deny motion.
    speeds = _deadman_with_moving_car(aware_node, 'never', start=40, ticks=4)
    assert all(speed == 0.0 for speed in speeds)


def test_rectangle_narrow_corridor_publishes_follow_cap():
    node, published = _make('-p', 'enable_opponent_awareness:=true',
                            '-p', 'opponent_passing_margin:=0.05',
                            '-p', 'opponent_pass_max_steering:=0.33')
    try:
        sequence = _drive(node, published, corridor_width=1.06, rear_x0=2.0)
        # oracle: R4 1.06 m corridor cannot fit the inflated opponent and ego.
        assert node.opponent_mode == 'follow'
        assert node.opponents.params.width == 0.31
        assert node.opponents.moving_body()
        for rear_x, _y, speed in sequence[25:]:
            # oracle: closed form follow law; 1 cm/s covers filter residual.
            assert speed <= _closed_form_cap(rear_x) + 0.01
    finally:
        node.destroy_node()
        rclpy.shutdown()


# --- the follow-cap memory (opponent_follow_memory_sec) ----------------------
#
# Decision of 2026-10-28 (option 1): awareness on, and the follow cap held for
# opponent_follow_memory_sec (1.0 s) after its car is lost. A car is confirmed and
# followed at CAR_GAP with LB held, then it leaves the scan. The tracker still
# reports a car it cannot see for opponent_max_misses scans (6), so the cap stays
# live on those scans; the memory takes over on the scan after them.

CAR_GAP = 0.9        # m, bumper to the car ahead: follow cap (0.9 - 0.5) / 1.0 = 0.4 m/s
EGO_CRUISE = 1.0     # m/s, odometry while the car is followed


def _follow_law(gap):
    # oracle: closed form of the gap-only follow law, with the shipped defaults.
    p = opponent_tracking.OpponentParams()
    return (gap - p.follow_min_gap) / p.follow_time_gap


def _scene_tick(node, published, step, gap, ego_speed, joy_state='held',
                walls=None):
    """One 40 Hz scan with a car ahead at ``gap`` metres from the bumper (None:
    the scan holds no car), odometry at ego_speed, and the deadman in joy_state.
    ``walls``, when given, replaces the open corridor (a closed one makes the
    node stop with no_safe_gap).
    'held' and 'released' are Joy messages with LB as named. 'stale' is a held
    Joy that arrived more than joy_timeout_sec ago. 'never' is a node that has
    heard no Joy at all: last_joy_time is cleared, the state the deadman reports
    as waiting_for_joy. Returns the published speed."""
    segments = _corridor(2.0, 2.0) if walls is None else walls
    if gap is not None:
        segments = segments + _box(gap + FRONT - LASER_X)
    scan = _raycast(segments)
    ns = round((100.0 + step * DT) * 1e9)
    scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(ns, 10**9)
    if joy_state in ('held', 'released', 'stale'):
        joy = Joy()
        joy.buttons = [0, 0, 0, 0, int(joy_state != 'released')]
        node.joy_callback(joy)
        if joy_state == 'stale':
            node.last_joy_time = node.get_clock().now() - Duration(
                seconds=node.joy_timeout_sec + DT)
    elif joy_state == 'never':
        node.last_joy_time = None
    odom = Odometry()
    odom.twist.twist.linear.x = float(ego_speed)
    node.odom_callback(odom)
    node.steering_basis = 0.0
    node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
    node.scan_callback(scan)
    return published[-1].drive.speed


def _confirm_then_drop(node, published, drop_ticks):
    """Confirm the car at CAR_GAP with LB held, then let it leave the scan for
    drop_ticks scans. Steps 0-39 confirm; steps 40 on are the drop. Returns the
    published speed and the opponent mode, keyed by step, for the drop ticks."""
    for step in range(40):
        confirm_speed = _scene_tick(node, published, step, CAR_GAP, EGO_CRUISE)
    # oracle: the car is tracked and its follow cap binds before it is lost, so
    # there is a cap to hold. 'clear' is a tracked car too: the layer may choose
    # to pass a car it still follows, and the follow cap applies either way.
    assert node.opponent_mode in ('clear', 'follow')
    assert _follow_law(CAR_GAP) - 0.05 <= confirm_speed <= _follow_law(CAR_GAP) + 0.01
    speeds, modes = {}, {}
    for step in range(40, 40 + drop_ticks):
        speeds[step] = _scene_tick(node, published, step, None, EGO_CRUISE)
        modes[step] = node.opponent_mode
    return speeds, modes


def _live_held_cap(node, published):
    """Confirm and drop the car with LB held, until the memory alone is capping
    the speed. Returns the next step number."""
    speeds, modes = _confirm_then_drop(node, published, drop_ticks=8)
    # oracle: the memory is live -- its held cap binds, and motion is permitted.
    assert modes[47] == 'follow_memory'
    assert 0.0 < speeds[47] <= _follow_law(CAR_GAP) + 0.01
    return 48


def test_a_dropped_car_keeps_its_follow_cap_for_exactly_one_second(aware_node):
    node, published = aware_node
    speeds, modes = _confirm_then_drop(node, published, drop_ticks=100)
    # The last tick the tracker still reports the car (mode 'clear' or 'follow')
    # holds the last finite cap. The hold then covers the next 40 scans: 1.0 s.
    last = max(k for k in modes if modes[k] in ('clear', 'follow'))
    held = [k for k in modes if last < k <= last + 40]
    released = [k for k in modes if k > last + 40]
    # oracle: inclusive 1.0 s of scan stamps = 40 scans of 0.025 s.
    assert len(held) == 40 and released
    cap = _follow_law(CAR_GAP)
    for k in held:
        assert modes[k] == 'follow_memory', k
        # oracle: closed form, the published speed never above the held cap.
        # 1 cm/s covers the alpha-beta filter's residual.
        assert speeds[k] <= cap + 0.01, (k, speeds[k], cap)
    # The held cap is what binds: the speed sits at it, not well below.
    assert max(speeds[k] for k in held) >= cap - 0.05
    for k in released:
        assert modes[k] != 'follow_memory', k
    # oracle: once released, nothing caps the speed at the held value.
    assert max(speeds[k] for k in released) > cap + 0.2


def test_with_the_memory_at_zero_the_cap_goes_with_its_car():
    node, published = _make('-p', 'enable_opponent_awareness:=true',
                            '-p', 'opponent_follow_memory_sec:=0.0',
                            '-p', 'intent_rate_hz:=0.0')
    try:
        # oracle: the override reached the layer, so the zero is really in force.
        assert node.opponents.params.follow_memory_sec == 0.0
        speeds, modes = _confirm_then_drop(node, published, drop_ticks=20)
        last = max(k for k in modes if modes[k] in ('clear', 'follow'))
        assert all(mode != 'follow_memory' for mode in modes.values())
        # oracle: the first scan without the tracked car is uncapped, so the speed
        # climbs past the cap it had (the accel ramp from 1.0 m/s odometry).
        assert speeds[last + 1] > _follow_law(CAR_GAP) + 0.2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_the_held_cap_is_the_binding_follow_cap_in_the_intent(aware_node):
    node, published = aware_node
    intents = []
    node.intent_pub.publish = lambda msg: intents.append(schema.decode(msg.data))
    _live_held_cap(node, published)
    factors = intents[-1]['factors']
    # oracle: the intent names the held cap as the binding follow cap, as it
    # does for a live cap (test_known_stopped_car_publishes_zero_then_resumes).
    assert {f['name'] for f in factors if f['binding']} == {'follow cap'}
    follow = next(f for f in factors if f['name'] == 'follow cap')
    assert follow['value'] == pytest.approx(_follow_law(CAR_GAP), abs=0.01)


def test_held_cap_publishes_motion_with_lb_held(aware_node):
    node, published = aware_node
    step = _live_held_cap(node, published)
    speeds = [_scene_tick(node, published, s, None, EGO_CRUISE)
              for s in range(step, step + 4)]
    # oracle: the permissive side -- LB held and the held cap binding, so the
    # command is non-zero and no higher than the cap.
    assert all(0.0 < v <= _follow_law(CAR_GAP) + 0.01 for v in speeds), speeds
    assert node.opponent_mode == 'follow_memory'


def test_held_cap_publishes_zero_with_lb_released(aware_node):
    node, published = aware_node
    step = _live_held_cap(node, published)
    speeds = [_scene_tick(node, published, s, None, EGO_CRUISE, 'released')
              for s in range(step, step + 4)]
    # oracle: CLAUDE.md safety model -- released LB denies motion, whatever the
    # memory is holding.
    assert speeds == [0.0] * 4, speeds


def test_held_cap_publishes_zero_when_joy_is_never_received(aware_node):
    node, published = aware_node
    step = _live_held_cap(node, published)
    speeds = [_scene_tick(node, published, s, None, EGO_CRUISE, 'never')
              for s in range(step, step + 4)]
    # oracle: CLAUDE.md safety model -- without Joy the car does not move, whatever
    # the memory is holding.
    assert speeds == [0.0] * 4, speeds


def test_held_cap_publishes_zero_when_joy_is_stale(aware_node):
    node, published = aware_node
    step = _live_held_cap(node, published)
    speeds = [_scene_tick(node, published, s, None, EGO_CRUISE, 'stale')
              for s in range(step, step + 4)]
    # oracle: CLAUDE.md safety model -- an expired Joy denies motion, whatever the
    # memory is holding.
    assert speeds == [0.0] * 4, speeds


# A corridor closed 0.55 m ahead: no gap the node will take, so it stops with
# no_safe_gap. The clock test below stops the car on it.
CLOSED_CORRIDOR = _corridor(0.5, 0.5) + [((0.55, -0.5), (0.55, 0.5))]


def _alongside_scan(step, narrow=False):
    # Wide passing lane, narrowing ahead to the course's 1.06 m minimum.
    walls = _corridor(2.0, 2.0)
    if narrow:
        walls = [((-10.0, s * 2.0), (0.3, s * 2.0)) for s in (-1, 1)]
        walls += [((0.3, s * 2.0), (0.6, 0.30 + s * 0.53)) for s in (-1, 1)]
        walls += [((0.6, 0.30 + s * 0.53), (20.0, 0.30 + s * 0.53)) for s in (-1, 1)]
    scan = _raycast(walls + _box(1.2 - step * DT, 0.65))
    ns = round((100.0 + step * DT) * 1e9)
    scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(ns, 10**9)
    return scan


def _alongside_tick(node, published, step, narrow=False, state='held'):
    if state != 'never':
        joy = Joy()
        joy.buttons = [0, 0, 0, 0, int(state != 'released')]
        node.joy_callback(joy)
        if state == 'stale':
            node.last_joy_time = node.get_clock().now() - Duration(
                seconds=node.joy_timeout_sec + DT)
    else:
        node.last_joy_time = None
    odom = Odometry()
    odom.twist.twist.linear.x = EGO_SPEED
    node.odom_callback(odom)
    node.steering_basis = 0.0
    node.last_commanded_speed = EGO_SPEED
    node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
    node.scan_callback(_alongside_scan(step, narrow))
    return published[-1].drive.speed


def test_alongside_pass_aborts_on_first_narrow_tick(aware_node):
    node, published = aware_node
    for step in range(44):
        _alongside_tick(node, published, step)
    # oracle: invariant -- a pass really is under way with an alongside mover.
    assert node.opponent_mode == 'clear'
    assert any(m[4] <= FRONT for m in node.opponents.moving_body())
    speed = _alongside_tick(node, published, 44, narrow=True)
    assert node.opponent_mode == 'follow'
    # oracle: invariant -- drop behind the true 1.5 m/s car on the first refusal.
    # Exact margin arithmetic is tested with known velocities in the ROS-free test;
    # scan centroid changes introduce velocity-estimation error here.
    assert 0.0 < speed < OPP_SPEED


@pytest.mark.parametrize('state', ['held', 'released', 'never', 'stale'])
def test_alongside_abort_deadman_published_command(aware_node, state):
    node, published = aware_node
    for step in range(44):
        _alongside_tick(node, published, step)
    speed = _alongside_tick(node, published, 44, narrow=True, state=state)
    # oracle: CLAUDE.md LB policy on the published command with abort cap active.
    if state == 'held':
        assert node.opponent_mode == 'follow'
        assert 0.0 < speed < OPP_SPEED
    else:
        assert speed == 0.0


def test_the_memory_clock_runs_through_a_no_safe_gap_stop(aware_node):
    node, published = aware_node
    cap = _follow_law(CAR_GAP)
    for step in range(40):
        confirm_speed = _scene_tick(node, published, step, CAR_GAP, EGO_CRUISE)
    # oracle: the car is tracked and its follow cap binds before it is lost.
    assert node.opponent_mode in ('clear', 'follow'), node.opponent_mode
    assert cap - 0.05 <= confirm_speed <= cap + 0.01, (confirm_speed, cap)
    # The car is lost on an open corridor at cruise speed, the odometry the track
    # was confirmed with, so the held cap stays at its confirm-phase value. The
    # track ends about 7 scans after the loss and the memory takes over.
    drop_speeds = []
    for step in range(40, 50):
        drop_speeds.append(_scene_tick(node, published, step, None, EGO_CRUISE))
    # oracle: the held cap binds on every loss scan (closed form, 1 cm/s allowance),
    # and once the track has ended the mode names the memory.
    assert all(v <= cap + 0.01 for v in drop_speeds), (drop_speeds, cap)
    assert node.opponent_mode == 'follow_memory', node.opponent_mode
    # Steps 50-69: the corridor closes and the car is still gone, 20 scans (0.5 s)
    # of no_safe_gap. The memory's clock is the scan stamp, and its last update
    # before the stop is step 49. Step 70 is 21 scans (0.525 s) later, past the
    # 0.5 s staleness rule, so step 70 is still held only if the stop scans
    # updated the memory.
    stop_states, stop_speeds = [], []
    for step in range(50, 70):
        stop_speeds.append(_scene_tick(node, published, step, None, 0.5,
                                       walls=CLOSED_CORRIDOR))
        stop_states.append(node.last_decision_state)
    # oracle: every stop scan is no_safe_gap, and no_safe_gap publishes zero.
    assert stop_states == ['no_safe_gap'] * 20, stop_states
    assert stop_speeds == [0.0] * 20, stop_speeds
    # The corridor opens again, still with no car, at 0.5 m/s. The cap was stored
    # at step 39 or later, so its 1.0 s hold runs to step 79 at the earliest and
    # covers steps 70-79.
    open_speeds, open_modes = [], []
    for step in range(70, 80):
        open_speeds.append(_scene_tick(node, published, step, None, 0.5))
        open_modes.append(node.opponent_mode)
    # oracle: the held cap binds on every one of those scans, and the mode names
    # the memory.
    assert open_modes == ['follow_memory'] * 10, open_modes
    assert all(v <= cap + 0.01 for v in open_speeds), (open_speeds, cap)


def _alongside_scan_car_gone(step):
    # The narrowing corridor of _alongside_scan, with the other car gone.
    walls = [((-10.0, s * 2.0), (0.3, s * 2.0)) for s in (-1, 1)]
    walls += [((0.3, s * 2.0), (0.6, 0.30 + s * 0.53)) for s in (-1, 1)]
    walls += [((0.6, 0.30 + s * 0.53), (20.0, 0.30 + s * 0.53)) for s in (-1, 1)]
    scan = _raycast(walls)
    ns = round((100.0 + step * DT) * 1e9)
    scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(ns, 10**9)
    return scan


def test_the_follow_memory_never_holds_the_alongside_abort(aware_node):
    node, published = aware_node
    for step in range(44):
        _alongside_tick(node, published, step)
    abort_speed = _alongside_tick(node, published, 44, narrow=True)
    # oracle: invariant -- the abort binds on the narrow tick and nothing else
    # does: no car is ahead of the bumper, so the follow law is uncapped.
    assert node.opponent_mode == 'follow' and 0.0 < abort_speed < OPP_SPEED
    assert node.opponents.follow_law_cap(False) == math.inf
    assert node.opponents.alongside_cap(False) < OPP_SPEED
    joy = Joy()
    joy.buttons = [0, 0, 0, 0, 1]
    # The car leaves the scan; its track coasts (and still aborts) for up to
    # opponent_max_misses scans, then is dropped. Stop at the first scan with
    # no tracked car at all.
    for step in range(45, 45 + 40):
        node.joy_callback(joy)
        odom = Odometry()
        odom.twist.twist.linear.x = EGO_SPEED
        node.odom_callback(odom)
        node.steering_basis = 0.0
        node.last_commanded_speed = abort_speed
        node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
        node.scan_callback(_alongside_scan_car_gone(step))
        if not node.opponents.followed_body():
            break
    assert not node.opponents.followed_body()
    # oracle: invariant -- on the first scan with no tracked car, the abort
    # ceiling is gone: the memory holds only the follow law (uncapped here),
    # so the command rises above the abort speed instead of being held at it.
    assert node.opponent_mode != 'follow_memory', node.opponent_mode
    assert published[-1].drive.speed > abort_speed, (published[-1].drive.speed, abort_speed)
