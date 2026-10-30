"""ROS IMU wiring/interlock: pytest with rb in an isolated localhost domain.

Seen failing against pre-interlock publication, removal of the real IMU guard,
car 2 routed to root topics, ignored runtime faults, and ignored publish_imu.
"""

import json
import os
import re
import signal
import subprocess
import time

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_prefix
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from conftest import require_isolation, require_test_drive_topic
from test_controlled_cars import INTERLOCK_OPENS_SEC, LEAK_WINDOW_SEC, RESUME_WINDOW_SEC
from imu_bridge.convert import ORIENTATION_UNKNOWN, covariance_diag
from racerbot_sim.sim_imu import BNO_NOISE, VESC_NOISE

STATUS_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.fixture(autouse=True)
def isolated_ros_tests(monkeypatch):
    require_isolation()
    original = Node.create_publisher

    def guarded(self, msg_type, topic, *args, **kwargs):
        require_test_drive_topic(self.resolve_topic_name(topic))
        return original(self, msg_type, topic, *args, **kwargs)

    monkeypatch.setattr(Node, 'create_publisher', guarded)


def spin_until(node, predicate, seconds, tick=None):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if tick:
            tick()
        rclpy.spin_once(node, timeout_sec=0.01)
        if predicate():
            return True
    return False


@pytest.fixture
def ros_context(tmp_path):
    rclpy.init()
    node = rclpy.create_node('sim_imu_test')
    processes = []
    logs = []

    def start(*args):
        path = os.path.join(get_package_prefix('racerbot_sim'),
                            'lib', 'racerbot_sim', 'gym_bridge_node')
        log = (tmp_path / f'bridge-{len(processes)}.log').open('w')
        logs.append(log)
        process = subprocess.Popen([path, '--ros-args', *args,
            '-p', 'track_directory:=' + str(tmp_path),
            '-p', 'drive_topic:=/test_only/imu/ackermann_cmd',
            '-p', 'car2_drive_topic:=/test_only/imu/car2/ackermann_cmd'],
            stdout=log, stderr=log, start_new_session=True)
        processes.append(process)
        return process

    yield node, start
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
    for process in processes:
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=10)
    for log in logs:
        log.close()
    node.destroy_node()
    rclpy.shutdown()


def subscribe(node, topic, kind=Imu):
    messages = []
    node.create_subscription(kind, topic, messages.append, STATUS_QOS if kind is String else 10)
    return messages


def set_fault(node, value, parameter='imu_fault'):
    client = node.create_client(SetParameters, '/gym_bridge_node/set_parameters')
    try:
        assert client.wait_for_service(timeout_sec=2.0), 'parameter service absent'
        request = SetParameters.Request(parameters=[Parameter(name=parameter,
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value))])
        future = client.call_async(request)
        assert spin_until(node, future.done, 2.0), 'parameter service did not answer'
        return future.result().results[0]
    finally:
        node.destroy_client(client)


def test_interlock_beside_real_imu_stack(ros_context):
    node, start = ros_context
    hardware = rclpy.create_node('imu_bridge_node')
    topics = ['/imu/data', '/imu/bno/data', '/imu/vesc/data', '/imu/status',
              '/car2/imu/data', '/car2/imu/bno/data', '/car2/imu/vesc/data',
              '/car2/imu/status', '/scan']
    received = {topic: subscribe(node, topic, String if topic.endswith('status')
                else LaserScan if topic == '/scan' else Imu) for topic in topics}
    try:
        start('-p', 'controlled_cars:=2')
        # oracle: test live publishers, not startup delay; discovery/clear hold
        # defaults come from the production nodes via test_controlled_cars.
        assert spin_until(node, lambda: all(node.count_publishers(t) > 0 for t in topics), 20.0)
        assert LEAK_WINDOW_SEC > INTERLOCK_OPENS_SEC
        assert not spin_until(node, lambda: any(received.values()), LEAK_WINDOW_SEC), {
            topic: len(messages) for topic, messages in received.items()}
    finally:
        hardware.destroy_node()
    # oracle: removing real hardware allows both simulated primary streams.
    assert spin_until(node, lambda: received['/imu/data'] and received['/car2/imu/data'],
                      RESUME_WINDOW_SEC)


def test_car2_imu_reaches_only_car2(ros_context):
    node, start = ros_context
    first = subscribe(node, '/imu/data')
    second = subscribe(node, '/car2/imu/data')
    bno = subscribe(node, '/imu/bno/data')
    vesc = subscribe(node, '/imu/vesc/data')
    scans = subscribe(node, '/scan', LaserScan)
    status = subscribe(node, '/imu/status', String)
    publisher = node.create_publisher(AckermannDriveStamped, '/test_only/imu/car2/ackermann_cmd', 10)
    command = AckermannDriveStamped()
    command.drive.speed = 2.0
    command.drive.steering_angle = 0.15
    tick = lambda: publisher.publish(command)
    start('-p', 'controlled_cars:=2')
    # oracle: only the second plant receives a turning command. Wait for
    # actual turning before measuring, not merely the discovery hold.
    assert spin_until(node, lambda: second and abs(second[-1].angular_velocity.z) > 0.4,
                      INTERLOCK_OPENS_SEC + 20.0, tick)
    first.clear()
    second.clear()
    assert not spin_until(node, lambda: False, 1.0, tick)
    assert first and second
    assert node.count_publishers('/imu/data') == 1
    assert node.count_publishers('/car2/imu/data') == 1
    assert max(abs(m.angular_velocity.z) for m in first) < 0.05  # rad/s, stationary car
    assert min(abs(m.angular_velocity.z) for m in second) > 0.2  # rad/s, turning car
    assert {m.header.frame_id for m in first} == {'base_link'}
    assert {m.header.frame_id for m in second} == {'car2/base_link'}
    print(f'car1 max |gyro z|={max(abs(m.angular_velocity.z) for m in first):.9f}; '
          f'car2 min |gyro z|={min(abs(m.angular_velocity.z) for m in second):.9f} rad/s')
    # oracle: no sensor_msgs/Imu exists outside the specified per-car trees.
    imu_topics = [t for t, types in node.get_topic_names_and_types() if 'sensor_msgs/msg/Imu' in types]
    assert imu_topics
    assert all(re.fullmatch(r'(?:/car(?:[2-9]|[1-9][0-9]+))?/imu/(?:bno/data|vesc/data|data)', t)
               for t in imu_topics), imu_topics
    # oracle: imu_bridge format and primary=BNO, matched by shared stamp.
    assert bno and vesc and scans and status
    bno_by_stamp = {(m.header.stamp.sec, m.header.stamp.nanosec): m for m in bno}
    matches = [(m, bno_by_stamp[(m.header.stamp.sec, m.header.stamp.nanosec)])
               for m in first if (m.header.stamp.sec, m.header.stamp.nanosec) in bno_by_stamp]
    assert matches
    assert all(primary == sensor for primary, sensor in matches)
    scan_stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for m in scans}
    assert any((m.header.stamp.sec, m.header.stamp.nanosec) in scan_stamps for m in first)
    for messages, noise in [(bno, BNO_NOISE), (vesc, VESC_NOISE)]:
        assert list(messages[-1].orientation_covariance) == list(ORIENTATION_UNKNOWN)
        assert list(messages[-1].angular_velocity_covariance) == covariance_diag(noise.gyro_std)
        assert list(messages[-1].linear_acceleration_covariance) == covariance_diag(noise.accel_std)
    # oracle: unchanged status is published once; a late durable subscriber
    # still receives it, proving the depth-1 transient-local contract.
    assert len(status) == 1
    late = subscribe(node, '/imu/status', String)
    assert spin_until(node, lambda: late, 2.0, tick)
    assert json.loads(late[-1].data) == dict(state='ok', reason=None, imu='bno')


def test_fault_set_mid_run(ros_context):
    node, start = ros_context
    first = subscribe(node, '/imu/data')
    second = subscribe(node, '/car2/imu/data')
    status = subscribe(node, '/imu/status', String)
    start('-p', 'controlled_cars:=2', '-p', 'imu_fault:=none')
    # oracle: the initial healthy stream has the real consumer's ok state.
    assert spin_until(node, lambda: first and second and status
                      and json.loads(status[-1].data)['state'] == 'ok', INTERLOCK_OPENS_SEC + 20.0)
    assert set_fault(node, 'status_fault').successful
    assert spin_until(node, lambda: json.loads(status[-1].data)['state'] == 'fault', 1.0)
    assert json.loads(status[-1].data)['reason'] == 'simulated fault (imu_fault=status_fault)'
    # Restore ok before killing data: stale preserves the last published
    # verdict, not an invented fresh fault (controller's dead-node ruling).
    assert set_fault(node, 'none').successful
    assert spin_until(node, lambda: json.loads(status[-1].data)['state'] == 'ok', 1.0)
    assert set_fault(node, 'stale').successful
    spin_until(node, lambda: False, 0.2)  # drain samples already in flight
    first.clear()
    second.clear()
    status_count = len(status)
    # oracle: no primary messages for a full second, other car uninterrupted,
    # and no new status; consumers must detect the primary's data age.
    assert not spin_until(node, lambda: first, 1.0)
    assert len(second) >= 10  # ample margin below the 40 Hz configured stream
    assert len(status) == status_count
    assert json.loads(status[-1].data)['state'] == 'ok'
    rejected = set_fault(node, 'banana')
    assert not rejected.successful
    assert 'imu_fault' in rejected.reason
    assert not spin_until(node, lambda: first, 0.3)  # invalid set leaves stale active
    assert set_fault(node, 'none').successful
    assert spin_until(node, lambda: first, 1.0)
    assert json.loads(status[-1].data)['state'] == 'ok'


def test_publish_imu_false_creates_no_imu_publishers(ros_context):
    node, start = ros_context
    scan = subscribe(node, '/scan', LaserScan)
    start('-p', 'controlled_cars:=2', '-p', 'publish_imu:=false')
    # oracle: once the plant is publishing, disabled IMUs must have no graph
    # endpoints at all (not merely no samples during the discovery hold).
    assert spin_until(node, lambda: scan, INTERLOCK_OPENS_SEC + 20.0)
    for prefix in ('', '/car2'):
        for suffix in ('data', 'bno/data', 'vesc/data', 'status'):
            assert node.count_publishers(f'{prefix}/imu/{suffix}') == 0


def test_late_imu_status_withdrawn_when_hardware_appears_and_restored(ros_context):
    """I1 round 2: pre-fix durable simulated ok survived the hardware block.

    Run by pytest src/racerbot_sim/test/ with rb in an isolated ROS domain.
    Seen failing against c8b6932 and with destroy_publisher skipped (late ok
    leaked); skipping recreation failed the resume assertion with both topics
    empty. Both mutations were reverted and this test passed again.
    """
    node, start = ros_context
    topics = ('/imu/status', '/car2/imu/status')
    initial = {topic: subscribe(node, topic, String) for topic in topics}
    start('-p', 'controlled_cars:=2')
    # oracle: healthy default IMUs must have actually offered ok on both cars
    # before hardware arrives; startup silence cannot satisfy this regression.
    assert spin_until(node, lambda: all(messages and
        json.loads(messages[-1].data)['state'] == 'ok'
        for messages in initial.values()), INTERLOCK_OPENS_SEC + 20.0)

    hardware = rclpy.create_node('imu_bridge_node')
    late = None
    resumed = None
    try:
        spin_until(node, lambda: False, INTERLOCK_OPENS_SEC + 1.0)
        late = rclpy.create_node('sim_imu_blocked_late_subscriber')
        blocked = {topic: subscribe(late, topic, String) for topic in topics}
        # oracle: invariant -- nothing simulated may be offered beside hardware,
        # including a durable sample published before the interlock blocked.
        assert LEAK_WINDOW_SEC > INTERLOCK_OPENS_SEC
        assert not spin_until(late, lambda: any(blocked.values()), LEAK_WINDOW_SEC), {
            topic: [message.data for message in messages]
            for topic, messages in blocked.items()}

        hardware.destroy_node()
        hardware = None
        resumed = rclpy.create_node('sim_imu_resumed_late_subscriber')
        restored = {topic: subscribe(resumed, topic, String) for topic in topics}
        # oracle: healthy default status must be offered again on BOTH topics
        # after the clean-graph hold, even though its value has not changed.
        assert spin_until(resumed, lambda: all(messages and
            json.loads(messages[-1].data)['state'] == 'ok'
            for messages in restored.values()), RESUME_WINDOW_SEC), {
                topic: [message.data for message in messages]
                for topic, messages in restored.items()}
    finally:
        if hardware is not None:
            hardware.destroy_node()
        if late is not None:
            late.destroy_node()
        if resumed is not None:
            resumed.destroy_node()
