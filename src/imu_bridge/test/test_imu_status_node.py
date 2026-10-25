"""imu_bridge_node publishes /imu/status the way neural_racer reads it.

The verdict logic is tested unsourced in test_imu_health.py; this checks the
wiring: the primary IMU's raw VESC samples reach ImuHealth, and the verdict
arrives on /imu/status for a subscriber with neural_racer_node's own QoS
(transient local, reliable) and parser (json state == 'ok'). Needs ROS:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/imu_bridge/test/test_imu_status_node.py -v

Oracle: an identity mount and no calibration file (convert.load_calibration),
so a raw VESC reading of 1 g on z is gravity on +z in car axes.
"""
import json
import os
import sys
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from imu_bridge.imu_bridge_node import ImuBridgeNode  # noqa: E402

# neural_racer_node.py's subscription, verbatim.
NEURAL_RACER_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=QoSReliabilityPolicy.RELIABLE)
WINDOW_S = 0.2


@pytest.fixture
def bridge(tmp_path):
    car = tmp_path / 'config' / 'car.yaml'
    car.parent.mkdir()
    car.write_text('imus:\n  vesc:\n    rpy_deg: [0.0, 0.0, 0.0]\n')
    rclpy.init(args=['--ros-args',
                     '-p', f'car_yaml:={car}', '-p', f'racer_imu_root:={tmp_path}',
                     '-p', f'startup_bias_s:={WINDOW_S}', '-p', f'gravity_window_s:={WINDOW_S}'])
    node = ImuBridgeNode()
    listener = Node('imu_status_listener')
    received = []
    listener.create_subscription(String, '/imu/status', lambda m: received.append(m.data),
                                 NEURAL_RACER_QOS)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(listener)
    yield node, executor, received
    executor.shutdown()
    listener.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def _vesc(acc_g, gyro_dps):
    msg = Imu()
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc_g
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro_dps
    return msg


def _drive_samples(node, executor, seconds, acc_g=(0.0, 0.0, 1.0), gyro_dps=(0.0, 0.0, 0.05)):
    odom = Odometry()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        node._on_odom(odom)                 # wheel speed 0: still
        node._on_vesc(_vesc(acc_g, gyro_dps))
        executor.spin_once(timeout_sec=0.01)


def _last_state(executor, received, settle=0.3):
    end = time.monotonic() + settle
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
    assert received, 'nothing arrived on /imu/status'
    return json.loads(received[-1])


def test_still_upright_vesc_imu_reaches_ok_for_neural_racer(bridge):
    node, executor, received = bridge
    assert node.primary == 'vesc'
    _drive_samples(node, executor, 3 * WINDOW_S)
    status = _last_state(executor, received, settle=0.05)
    assert status['state'] == 'ok', status
    assert status['imu'] == 'vesc'


def test_before_any_sample_the_status_is_a_fault(bridge):
    _, executor, received = bridge
    assert _last_state(executor, received)['state'] == 'fault'


def test_all_zero_vesc_imu_never_reports_ok(bridge):
    node, executor, received = bridge
    _drive_samples(node, executor, 3 * WINDOW_S, acc_g=(0.0, 0.0, 0.0), gyro_dps=(0.0, 0.0, 0.0))
    states = {json.loads(r)['state'] for r in received}
    assert 'ok' not in states
    assert 'all zero' in json.loads(received[-1])['reason']


def test_upside_down_vesc_imu_faults(bridge):
    node, executor, received = bridge
    _drive_samples(node, executor, 3 * WINDOW_S, acc_g=(0.0, 0.0, -1.0))
    status = _last_state(executor, received, settle=0.05)
    assert status['state'] == 'fault' and 'gravity' in status['reason'], status


def test_ok_turns_to_fault_when_the_imu_goes_silent(bridge):
    node, executor, received = bridge
    _drive_samples(node, executor, 3 * WINDOW_S)
    assert _last_state(executor, received, settle=0.05)['state'] == 'ok'
    status = _last_state(executor, received, settle=0.3)    # no samples for 0.3 s > 0.1 s
    assert status['state'] == 'fault' and 'stale' in status['reason'], status
