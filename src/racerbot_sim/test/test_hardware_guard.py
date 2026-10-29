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


def _settled_interlock(node, purpose):
    # Existing tests exercise the post-discovery behavior, not the hold.
    times = iter([0.0] + [3.0] * 100)
    return hardware_guard.HardwareInterlock(
        node, purpose, discovery_hold_sec=3.0, clock=lambda: next(times))


class _Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def test_discovery_hold_blocks_empty_graph():
    clock = _Clock()
    guard = hardware_guard.HardwareInterlock(_Node(), 'output', 3.0, clock)
    clock.now = 2.999
    # oracle: output stays blocked strictly before the discovery hold expires.
    assert not guard.safe()
    assert guard.blocked


def test_discovery_hold_inclusive_boundary():
    clock = _Clock()
    guard = hardware_guard.HardwareInterlock(_Node(), 'output', 3.0, clock)
    clock.now = 3.0
    # oracle: a clean graph permits output at exactly the hold boundary.
    assert guard.safe()


def test_discovery_hold_then_hardware_still_blocks():
    clock = _Clock()
    guard = hardware_guard.HardwareInterlock(_Node(['urg_node']), 'output', 3.0, clock)
    clock.now = 3.0
    # oracle: a settled graph with a real driver must never permit output.
    assert not guard.safe()


def test_discovery_hold_then_unreadable_graph_blocks():
    clock = _Clock()
    guard = hardware_guard.HardwareInterlock(_Broken(), 'output', 3.0, clock)
    clock.now = 3.0
    # oracle: graph-query failure is not evidence of a clean graph.
    assert not guard.safe()


def test_no_hardware_means_no_conflict():
    node = _Node(['gym_bridge_node', 'sim_joy_node', 'ackermann_mux'])
    assert hardware_guard.conflicting_hardware_nodes(node) == []
    assert _settled_interlock(node, 'x').safe()


@pytest.mark.parametrize('name', hardware_guard.HARDWARE_NODE_NAMES)
def test_every_bringup_driver_blocks_the_simulator(name):
    node = _Node(['gym_bridge_node', name])
    assert hardware_guard.conflicting_hardware_nodes(node) == [name]
    interlock = _settled_interlock(node, 'simulated /scan')
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
    interlock = _settled_interlock(node, 'simulated /scan')
    assert interlock.safe()
    node.names.append('vesc_driver_node')
    assert not interlock.safe()


def test_output_resumes_once_the_drivers_are_gone():
    node = _Node(['vesc_driver_node'])
    interlock = _settled_interlock(node, 'simulated /scan')
    assert not interlock.safe()
    node.names.clear()
    assert interlock.safe()
    assert not interlock.blocked
    assert any('resumed' in message for message in node.get_logger().infos)


class _Broken(_Node):
    def get_node_names_and_namespaces(self):
        raise RuntimeError('rmw is having a moment')


def test_a_graph_query_failure_never_takes_the_node_down():
    """The query failing must not raise out of the guard -- and must not be
    reported as "no conflicts" either (that was the fail-open bug)."""
    assert hardware_guard.conflicting_hardware_nodes(_Broken()) is None


def test_an_unreadable_graph_blocks_the_simulator():
    """Fail closed: when the guard cannot tell whether real drivers are on
    the graph, forged LB and synthetic sensors must stay off."""
    node = _Broken()
    interlock = _settled_interlock(node, 'the synthetic LB deadman')
    assert not interlock.safe()
    assert interlock.blocked
    assert 'Cannot read the ROS graph' in node.get_logger().errors[0]


def test_a_failed_query_does_not_clear_a_hardware_block():
    """A latched block from real drivers must survive a later query failure;
    only a successful query showing a clear graph releases it."""
    node = _Node(['vesc_driver_node'])
    interlock = _settled_interlock(node, 'simulated /scan')
    assert not interlock.safe()

    def _fail():
        raise RuntimeError('rmw is having a moment')

    node.get_node_names_and_namespaces = _fail
    assert not interlock.safe()
    assert interlock.blocked
    assert node.get_logger().infos == []

    del node.get_node_names_and_namespaces  # back to the class method
    node.names.clear()
    assert interlock.safe()
    assert not interlock.blocked


# The clean-graph confirmation (interlock_clear_confirm_sec). Opening needs the
# graph clean on every check for the whole window, so a graph that has not yet
# discovered the car's drivers cannot open it on one lucky early check.

def _confirming(node, hold=3.0, confirm=3.0):
    clock = _Clock()
    return clock, hardware_guard.HardwareInterlock(
        node, 'output', hold, clock, clear_confirm_sec=confirm)


def test_clear_confirm_holds_a_clean_graph_until_the_window_has_passed():
    clock, guard = _confirming(_Node())
    # oracle: closed form -- the window starts at the first clean check (t = hold)
    # and output opens at hold + confirm inclusive, never one step before.
    for t in (3.0, 4.5, 5.999):
        clock.now = t
        assert not guard.safe(), t
        assert guard.blocked
    clock.now = 6.0
    assert guard.safe()
    assert not guard.blocked


@pytest.mark.parametrize('dirty', ['hardware', 'unreadable'])
def test_clear_confirm_restarts_after_any_dirty_check(dirty):
    node = _Node()
    clock, guard = _confirming(node)
    clock.now = 3.0
    assert not guard.safe()
    clock.now = 5.0
    if dirty == 'hardware':
        node.names.append('urg_node')
    else:
        node.get_node_names_and_namespaces = _Broken().get_node_names_and_namespaces
    # oracle: a late-discovered driver (or an unreadable graph) inside the window
    # blocks, and the window restarts from the next clean check.
    assert not guard.safe()
    node.names.clear()
    node.get_node_names_and_namespaces = lambda: []
    clock.now = 5.5
    assert not guard.safe()
    clock.now = 8.499
    assert not guard.safe()
    clock.now = 8.5
    assert guard.safe()


def test_clear_confirm_also_gates_resuming_after_the_drivers_leave():
    node = _Node()
    clock, guard = _confirming(node)
    clock.now = 6.0
    assert not guard.safe()   # window starts at 6.0
    clock.now = 9.0
    assert guard.safe()
    node.names.append('vesc_driver_node')
    clock.now = 10.0
    assert not guard.safe()
    node.names.clear()
    # oracle: closed form -- resuming needs the same full window, from 11.0.
    clock.now = 11.0
    assert not guard.safe()
    clock.now = 13.999
    assert not guard.safe()
    clock.now = 14.0
    assert guard.safe()


@pytest.mark.parametrize('confirm', [-0.01, float('nan'), float('inf')])
def test_clear_confirm_rejects_invalid_values(confirm):
    # oracle: spec -- the window is a finite, non-negative duration.
    with pytest.raises(ValueError, match='interlock_clear_confirm_sec must be finite and >= 0'):
        hardware_guard.HardwareInterlock(_Node(), 'output', 3.0, _Clock(), clear_confirm_sec=confirm)


def test_both_simulator_nodes_ship_the_confirmation_on():
    # oracle: invariant -- the forging node and the sensor node both declare a
    # positive window and pass it to their interlock (static check, no rclpy).
    import re
    root = os.path.join(os.path.dirname(__file__), '..', 'racerbot_sim')
    for name in ('sim_joy_node.py', 'gym_bridge_node.py'):
        source = open(os.path.join(root, name), encoding='utf-8').read()
        default = re.search(r"declare_parameter\('interlock_clear_confirm_sec', ([0-9.]+)\)", source)
        assert default and float(default.group(1)) > 0.0, name
        assert re.search(r"clear_confirm_sec=float\(", source), name
