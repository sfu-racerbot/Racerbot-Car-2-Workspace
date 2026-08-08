"""The interlock that keeps the simulator away from the real car.

`sim_joy_node` forges the LB deadman the entire workspace safety policy
rests on, and `gym_bridge_node` publishes an imaginary /scan and /odom.
Both are fine against a simulated car and unacceptable beside a real one
-- which is exactly where this is most convenient to run, since the car's
Jetson is where the workspace already builds.

    python3 -m pytest src/racerbot_sim/test/test_hardware_guard.py -v
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_sim import hardware_guard  # noqa: E402


class _Logger:
    def __init__(self):
        self.errors = []
        self.infos = []

    def error(self, message, throttle_duration_sec=None):
        self.errors.append(message)

    def info(self, message, throttle_duration_sec=None):
        self.infos.append(message)


class _Node:
    def __init__(self, names=()):
        self.names = list(names)
        self._logger = _Logger()

    def get_node_names_and_namespaces(self):
        return [(name, '/') for name in self.names]

    def get_logger(self):
        return self._logger


def test_no_hardware_means_no_conflict():
    node = _Node(['gym_bridge_node', 'sim_joy_node', 'ackermann_mux'])
    assert hardware_guard.conflicting_hardware_nodes(node) == []
    assert hardware_guard.HardwareInterlock(node, 'x').safe()


@pytest.mark.parametrize('name', hardware_guard.HARDWARE_NODE_NAMES)
def test_every_bringup_driver_blocks_the_simulator(name):
    node = _Node(['gym_bridge_node', name])
    assert hardware_guard.conflicting_hardware_nodes(node) == [name]
    interlock = hardware_guard.HardwareInterlock(node, 'simulated /scan')
    assert not interlock.safe()
    assert interlock.blocked
    assert name in interlock._node.get_logger().errors[0]


def test_the_real_joystick_counts_as_hardware():
    """joy_node is in bringup precisely so autonomy can check LB. A
    synthetic /joy alongside it would be two hands on one button."""
    assert 'joy' in hardware_guard.HARDWARE_NODE_NAMES


def test_hardware_appearing_mid_run_suppresses_output():
    """The check is repeated, not just made at startup: bringing the car up
    after the simulator must shut the simulator up too."""
    node = _Node(['gym_bridge_node'])
    interlock = hardware_guard.HardwareInterlock(node, 'simulated /scan')
    assert interlock.safe()
    node.names.append('vesc_driver_node')
    assert not interlock.safe()


def test_output_resumes_once_the_drivers_are_gone():
    node = _Node(['vesc_driver_node'])
    interlock = hardware_guard.HardwareInterlock(node, 'simulated /scan')
    assert not interlock.safe()
    node.names.clear()
    assert interlock.safe()
    assert not interlock.blocked
    assert any('resumed' in message for message in node.get_logger().infos)


def test_a_graph_query_failure_never_takes_the_node_down():
    class _Broken(_Node):
        def get_node_names_and_namespaces(self):
            raise RuntimeError('rmw is having a moment')

    assert hardware_guard.conflicting_hardware_nodes(_Broken()) == []
