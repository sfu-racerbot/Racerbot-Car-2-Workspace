"""Multi-car bridge behavior; run with pytest in an isolated ROS domain."""

import json
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from rclpy.clock import ClockType
from rclpy.time import Time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_sim import sim_bridge as sim, tracks  # noqa: E402
from racerbot_sim.sim_joy_node import SimJoyNode  # noqa: E402


@pytest.mark.parametrize('topic', ['/drive', '/car2/drive', '/ackermann_cmd', '/car2/ackermann_cmd'])
def test_production_drive_publisher_is_refused_before_creation(topic):
    rclpy.init()
    node = rclpy.create_node('topic_guard_probe')
    try:
        # oracle: safety contract -- guard raises before ROS gets a publisher.
        before = list(node.publishers)
        with pytest.raises(pytest.fail.Exception, match='production drive topic forbidden'):
            node.create_publisher(AckermannDriveStamped, topic, 10)
        assert list(node.publishers) == before
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('scenario', ['release', 'silence'])
def test_joy_scenario_clock_starts_when_interlock_opens(scenario):
    rclpy.init()
    node = SimJoyNode()
    class Clock:
        seconds = 0.0
        def now(self):
            return Time(nanoseconds=int(self.seconds * 1e9), clock_type=ClockType.ROS_TIME)

    clock = Clock()
    node.get_clock = lambda: clock
    if node.start_time is not None:
        node.start_time = clock.now()

    class Gate:
        allowed = False
        def safe(self):
            return self.allowed

    class Capture:
        def __init__(self):
            self.messages = []
        def publish(self, message):
            self.messages.append(message)

    gate, capture = Gate(), Capture()
    node.interlock, node.publisher = gate, capture
    node.release_after_sec = 2.0 if scenario == 'release' else -1.0
    node.silence_after_sec = 2.0 if scenario == 'silence' else -1.0
    try:
        clock.seconds = 10.0
        node._publish()
        # oracle: the interlock blocks every message before discovery.
        assert capture.messages == []
        gate.allowed = True
        for t in (10.0, 11.999, 12.0):
            clock.seconds = t
            node._publish()
        # oracle: first permitted tick starts the two-second scenario.
        assert [m.buttons[4] for m in capture.messages[:2]] == [1, 1]
        if scenario == 'release':
            assert [m.buttons[4] for m in capture.messages] == [1, 1, 0]
        else:
            assert len(capture.messages) == 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_car_names_and_invalid_numbers():
    # oracle: specified ROS names, including frames and scoring topic.
    assert sim.car_names(1) == dict(namespace='', scan='/scan', odom='/odom',
        drive='/ackermann_cmd', joy='/joy', odom_frame='odom',
        base_frame='base_link', laser_frame='laser', truth='/sim/ground_truth_pose')
    assert sim.car_names(2) == dict(namespace='car2', scan='/car2/scan',
        odom='/car2/odom', drive='/car2/ackermann_cmd', joy='/car2/joy',
        odom_frame='car2/odom', base_frame='car2/base_link',
        laser_frame='car2/laser', truth='/sim/car2/ground_truth_pose')
    for value in (0, -1, 1.5, '2'):
        with pytest.raises(ValueError):
            sim.car_names(value)


def test_command_freshness_boundary():
    # oracle: timeout is inclusive, and an absent command is never fresh.
    assert not sim.command_is_fresh(None, 1.0, 0.2)
    assert sim.command_is_fresh(0.0, 0.2, 0.2)
    assert not sim.command_is_fresh(0.0, 0.201, 0.2)
    assert sim.command_is_fresh(1.0, 1.0, 0.2)


@pytest.fixture
def make_bridge(tmp_path):
    sim.add_sim_python_path()
    try:
        import f1tenth_gym  # noqa: F401
    except ModuleNotFoundError as exc:
        if exc.name != 'f1tenth_gym':
            raise
        pytest.skip('needs the gym checkout at .sim/ (docs/ros-simulator.md); runs wherever racerbot_sim runs')
    track = tracks.build('indoor_oval', str(tmp_path))
    centerline = tracks.load_centerline(str(tmp_path), 'indoor_oval')
    bridges = []

    def make(**kwargs):
        config = sim.SimConfig(track_path=str(track), centerline=centerline,
                               odom_speed_noise_std=0.0, **kwargs)
        bridge = sim.SimBridge(config)
        bridges.append(bridge)
        return bridge

    yield make
    for bridge in bridges:
        bridge.close()


@pytest.mark.parametrize('offsets', [(), (0.0, -5.0)])
def test_controlled_start_poses_follow_centerline_offsets(make_bridge, offsets):
    b = make_bridge(controlled_cars=2, controlled_start_offsets_m=offsets)
    xy = b.centerline
    lengths = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)
    arc = np.concatenate(([0.0], np.cumsum(lengths)))

    def point(s):
        s %= arc[-1]
        i = np.searchsorted(arc, s, side='right') - 1
        return xy[i] + (xy[(i + 1) % len(xy)] - xy[i]) * ((s - arc[i]) / lengths[i])

    for i, offset in enumerate(offsets or (0.0, -3.0)):
        p, ahead = point(offset), point(offset + 0.2)
        expected_yaw = math.atan2(ahead[1] - p[1], ahead[0] - p[0])
        state = b.agent_state(i)
        # oracle: independent arc-length interpolation on the generated loop.
        assert state.x == pytest.approx(p[0], abs=0.05)  # m
        assert state.y == pytest.approx(p[1], abs=0.05)  # m
        assert state.yaw == pytest.approx(expected_yaw, abs=0.05)  # rad


def test_invalid_controlled_offset_count_rejected():
    # oracle: one offset is required for each controlled car.
    with pytest.raises(ValueError, match='controlled_start_offsets_m'):
        sim.SimConfig(track_path='', centerline=np.zeros((4, 2)),
                      controlled_cars=2, controlled_start_offsets_m=(0.0,))


@pytest.mark.parametrize('moving', [0, 1])
def test_independent_motion_and_odometry(make_bridge, moving):
    b = make_bridge(controlled_cars=2)
    before = [b.agent_state(i) for i in range(2)]
    odom = [(b.odometry(i).x, b.odometry(i).y) for i in range(2)]
    # oracle: the starting offsets differ by 3 m along the straight.
    assert math.hypot(before[0].x-before[1].x, before[0].y-before[1].y) == pytest.approx(3.0, abs=0.05)  # m
    commands = [(0.0, 0.0), (0.0, 0.0)]
    commands[moving] = (0.0, 1.0)
    for _ in range(40):
        b.step_cars(commands)
    distances = [math.hypot(b.agent_state(i).x-before[i].x,
                            b.agent_state(i).y-before[i].y) for i in range(2)]
    odo_distances = [math.hypot(b.odometry(i).x-odom[i][0],
                                b.odometry(i).y-odom[i][1]) for i in range(2)]
    # oracle: zero speed stays put; a forward command moves its own car.
    assert distances[moving] > 0.3  # m
    assert distances[1-moving] < 0.001  # m
    assert odo_distances[moving] > 0.3  # m
    assert odo_distances[1-moving] < 0.001  # m


def test_car_scan_raycast_sees_other_car(make_bridge):
    b = make_bridge(controlled_cars=2)
    # oracle: bottom straight of indoor_oval runs along +x at y=-3.2 m.
    b.reset(poses=[[2.0, -3.2, 0.0], [0.0, -3.2, 0.0]])
    beam = sim.LIDAR_BEAMS // 2
    # oracle: ray meets the centred other box's rear face, measured from LiDAR.
    assert b.scan(1)[beam] == pytest.approx(2.0-sim.LIDAR_OFFSET_X-sim.CAR_LENGTH/2, abs=0.05)  # m, 5 sigma
    b.reset(poses=[[-2.0, -3.2, 0.0], [0.0, -3.2, 0.0]])
    assert b.scan(1)[beam] > 3.0  # m, wall beyond former obstacle


def test_gym_collision_response_control_and_disabled(make_bridge):
    from f1tenth_gym.envs.simulator import F110Simulator
    b = make_bridge(controlled_cars=2)
    yaw = 0.001
    poses = [[0.0, -3.2, yaw], [-0.6*math.cos(yaw), -3.2-0.6*math.sin(yaw), yaw]]
    # oracle: gym's own class default activates its collision response.
    b.reset(poses=poses)
    b.env.unwrapped.sim.ttc_threshold = F110Simulator.ttc_threshold
    flagged = snapped = False
    for _ in range(15):
        b.step_cars([(0.0, 0.0), (0.0, 0.1)])
        assert not b.car_contact()
        flagged |= b.agent_state(1).collision
        snapped |= abs(b.agent_state(1).yaw) < 1e-6
    assert flagged and snapped
    b.reset(poses=poses)
    disabled_flags = []
    for _ in range(15):
        b.step_cars([(0.0, 0.0), (0.0, 0.1)])
        # oracle: straight steering gives zero yaw rate; no collision response.
        assert not b.car_contact()
        for i in range(2):
            assert b.agent_state(i).yaw == pytest.approx(yaw, abs=1e-6)  # rad
            disabled_flags.append(b.agent_state(i).collision)
    assert not any(disabled_flags)


def test_single_car_step_compatibility_and_multi_car_rejection(make_bridge):
    plan = sim.OpponentPlan(start_offset_m=8.0, speed=0.6)
    a = make_bridge(controlled_cars=1, opponents=[plan])
    b = make_bridge(controlled_cars=1, opponents=[plan])
    follower = sim.CenterlineFollower(b.centerline, plan)
    obs = b._obs
    for _ in range(40):
        state = obs['agent_1']['std_state']
        steer, speed = follower.command(float(state[0]), float(state[1]), float(state[4]))
        obs, *_ = b.env.step(np.array([[0.1, 1.0], [steer, speed]], dtype=np.float32))
        a.step(0.1, 1.0)
        # oracle: direct Gym stepping bypasses both bridge step APIs, with
        # the scripted follower computed from agent 1's own state.
        for i in range(2):
            np.testing.assert_allclose(a._obs[f'agent_{i}']['std_state'],
                                       obs[f'agent_{i}']['std_state'], rtol=0, atol=1e-9)  # state units
    two = make_bridge(controlled_cars=2)
    with pytest.raises(ValueError, match='step_cars'):
        two.step(0.0, 1.0)


def _spin_until(node, predicate, seconds, tick=None):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if tick:
            tick()
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return False


@pytest.fixture
def ros_context():
    rclpy.init()
    node = rclpy.create_node('controlled_cars_test')
    processes = []
    def start(executable, *args):
        if executable == 'gym_bridge_node':
            args = (*args, '-p', 'drive_topic:=/test_only/ackermann_cmd',
                    '-p', 'car2_drive_topic:=/test_only/car2/ackermann_cmd')
        # Start the installed executable directly so teardown waits for the
        # actual node, rather than a ros2-run parent whose child can outlive it.
        executable_path = os.path.join(get_package_prefix('racerbot_sim'),
                                       'lib', 'racerbot_sim', executable)
        p = subprocess.Popen([executable_path,
                              '--ros-args', *args], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
        processes.append(p)
        return p
    yield node, start
    for process in processes:
        os.killpg(process.pid, signal.SIGTERM)
    for process in processes:
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=10)
    node.destroy_node()
    rclpy.shutdown()


def _shipped_interlock_sec(name):
    """The largest default of interlock parameter ``name`` declared by the two
    simulator nodes that publish (read from their source, no rclpy)."""
    import re
    root = os.path.join(os.path.dirname(__file__), '..', 'racerbot_sim')
    values = []
    for module in ('sim_joy_node.py', 'gym_bridge_node.py'):
        source = open(os.path.join(root, module), encoding='utf-8').read()
        found = re.search(rf"declare_parameter\('{name}', ([0-9.]+)\)", source)
        # A renamed parameter must fail here, not shrink the windows below.
        assert found, (module, name)
        values.append(float(found.group(1)))
    return max(values)


# The earliest the shipped interlock can open: discovery hold, then a full
# clean-graph confirmation window. The leak window below must outlast it, or
# the "no output beside hardware" check passes without checking anything.
INTERLOCK_OPENS_SEC = (_shipped_interlock_sec('interlock_discovery_hold_sec')
                       + _shipped_interlock_sec('interlock_clear_confirm_sec'))
LEAK_WINDOW_SEC = INTERLOCK_OPENS_SEC + 4.0      # 10 s on the shipped 3 s + 3 s
RESUME_WINDOW_SEC = _shipped_interlock_sec('interlock_clear_confirm_sec') + 12.0   # 15 s


def test_the_leak_window_outlasts_the_shipped_interlock():
    # oracle: invariant -- the window is derived from the nodes' own defaults and
    # leaves a margin past the earliest moment the interlock could open.
    assert INTERLOCK_OPENS_SEC > 0.0
    assert LEAK_WINDOW_SEC >= INTERLOCK_OPENS_SEC + 4.0


def test_interlock_suppresses_both_cars_and_namespaced_joy(ros_context):
    node, start = ros_context
    hardware = rclpy.create_node('urg_node')
    received = {topic: 0 for topic in ['/scan', '/car2/scan', '/odom', '/car2/odom', '/joy', '/car2/joy']}
    def callback(topic):
        def record(_):
            received[topic] += 1
        return record
    for topic in received:
        kind = Joy if topic.endswith('joy') else Odometry if topic.endswith('odom') else LaserScan
        node.create_subscription(kind, topic, callback(topic), 10)
    try:
        # oracle: all subscribers exist on the graph before any publisher starts.
        assert _spin_until(node, lambda: all(node.count_subscribers(topic) > 0
                                             for topic in received), 5.0)
        start('gym_bridge_node', '-p', 'controlled_cars:=2')
        start('sim_joy_node')
        start('sim_joy_node', '-r', '__ns:=/car2', '-p', 'joy_topic:=/car2/joy')
        # oracle: test the running publishers, not merely slow process startup.
        assert _spin_until(node, lambda: all(node.count_publishers(topic) > 0
                                             for topic in received), 20.0)
        # oracle: active hardware suppresses all synthetic sensor and joy output,
        # watched past the earliest moment the shipped interlock could open.
        assert not _spin_until(node, lambda: any(received.values()), LEAK_WINDOW_SEC), received
    finally:
        hardware.destroy_node()
    # oracle: after the hardware disappears both scans resume (after a full
    # clean-graph confirmation window).
    assert _spin_until(node, lambda: received['/scan'] > 0 and received['/car2/scan'] > 0,
                       RESUME_WINDOW_SEC)


def test_silent_car_stops_while_other_car_drives(ros_context):
    node, start = ros_context
    status = []
    node.create_subscription(String, '/sim/status', lambda m: status.append(json.loads(m.data)), 10)
    publishers = [node.create_publisher(AckermannDriveStamped, topic, 10)
                  for topic in ('/test_only/ackermann_cmd', '/test_only/car2/ackermann_cmd')]
    start('gym_bridge_node', '-p', 'controlled_cars:=2')
    assert _spin_until(node, lambda: bool(status), 20.0)
    command = AckermannDriveStamped()
    command.drive.speed = 1.0
    def both():
        for pub in publishers:
            pub.publish(command)
    # oracle: each input drives its own plant when refreshed.
    assert _spin_until(node, lambda: bool(status and len(status[-1].get('cars', [])) == 2 and status[-1]['cars'][1]['speed'] > 0.2), 5.0, both)
    assert not _spin_until(node, lambda: False, 1.0, both)
    def first():
        publishers[0].publish(command)
    # oracle: stale car 2 decelerates while the fresh car 1 keeps moving.
    assert _spin_until(node, lambda: len(status[-1].get('cars', [])) == 2 and not status[-1]['cars'][1]['commanded']['fresh'] and abs(status[-1]['cars'][1]['speed']) < 0.05 and status[-1]['cars'][0]['speed'] > 0.2, 3.0, first)
