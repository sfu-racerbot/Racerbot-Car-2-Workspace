"""Refuse to run the simulator while the real car's drivers are up.

Two nodes in this package are dangerous next to real hardware:

* `sim_joy_node` publishes a synthetic `/joy` with LB held. The entire
  workspace safety policy (docs/architecture.md) is that nothing moves
  unless a human is holding LB on the physical controller. A synthetic
  `/joy` is that human's hand, forged.
* `gym_bridge_node` publishes `/scan`, `/odom` and `/imu/*`. Alongside
  `urg_node`, `vesc_to_odom_node` or the real `imu_bridge_node`, these
  become two publishers per topic, and the
  driving nodes would be steering on a mixture of a real room and an
  imaginary one.

Neither is a problem on a laptop. Both are a problem on the car -- which
is exactly where this simulator is most convenient to run, since that is
where the workspace already builds. So the check is on the live ROS
graph, not on a flag someone has to remember: if the drivers are running,
these nodes hold their output and say why, and they keep checking, so
bringing the car up *after* the simulator shuts it up too.
"""

import math
import time

# Real bringup drivers and the hardware IMU stack: hardware on this graph.
HARDWARE_NODE_NAMES = (
    'vesc_driver_node',
    'ackermann_to_vesc_node',
    'vesc_to_odom_node',
    'urg_node',
    'joy',
    # The real IMU stack (vehicle_state_launch.py hardware:=true). The
    # simulator never starts it; two /imu/data publishers would mix a real
    # gyro with a simulated one.
    'imu_bridge_node',
)


def conflicting_hardware_nodes(node):
    """Names from HARDWARE_NODE_NAMES currently visible on the ROS graph.

    Returns None when the graph cannot be read at all. That is *not* the
    same answer as an empty list: "I could not look" must never be taken
    as "there is no car here" -- the interlock treats None as blocked.
    """
    try:
        live = {name for name, _namespace in node.get_node_names_and_namespaces()}
    except Exception:  # noqa: BLE001 - a graph query must never take a node down
        return None
    return sorted(live.intersection(HARDWARE_NODE_NAMES))


class HardwareInterlock:
    """Latched 'is it safe to publish' check with a throttled explanation."""

    def __init__(self, node, purpose: str, discovery_hold_sec=3.0, clock=time.monotonic,
                 clear_confirm_sec=0.0):
        if not math.isfinite(discovery_hold_sec) or discovery_hold_sec <= 0:
            raise ValueError('interlock_discovery_hold_sec must be finite and greater than zero')
        if not math.isfinite(clear_confirm_sec) or clear_confirm_sec < 0:
            raise ValueError('interlock_clear_confirm_sec must be finite and >= 0')
        self._node = node
        self._purpose = purpose
        self._blocked = True
        self._clock = clock
        self._started = clock()
        self._discovery_hold_sec = discovery_hold_sec
        # Opening needs the graph clean on every check for this long, not on
        # one check: discovery on a loaded machine can outlast the hold
        # (measured 2026-10-29: ~2.45 s typical, and a forged /joy leaked
        # beside a urg_node with a 3 s hold alone).
        self._clear_confirm_sec = clear_confirm_sec
        self._clean_since = None
        self._waiting_logged = False
        self._checked_graph = False

    @property
    def blocked(self) -> bool:
        return self._blocked

    def safe(self) -> bool:
        if self._clock() - self._started < self._discovery_hold_sec:
            if not self._waiting_logged:
                self._node.get_logger().info(
                    f'waiting {self._discovery_hold_sec}s for ROS discovery before {self._purpose}',
                    throttle_duration_sec=5.0)
                self._waiting_logged = True
            return False
        conflicts = conflicting_hardware_nodes(self._node)
        first_check = not self._checked_graph
        self._checked_graph = True
        if conflicts is None or conflicts:
            self._clean_since = None
        if conflicts is None:
            # Fail closed: an unreadable graph keeps (or starts) the block.
            # Only a successful query showing no drivers may clear it.
            if not self._blocked or first_check:
                self._blocked = True
                self._node.get_logger().error(
                    f'Cannot read the ROS graph, so cannot rule out real hardware. '
                    f'{self._purpose} is suppressed until a graph query succeeds.')
            else:
                self._node.get_logger().error(
                    'still suppressed: the ROS graph query is failing.',
                    throttle_duration_sec=5.0)
            return False
        if conflicts:
            if not self._blocked or first_check:
                self._blocked = True
                self._node.get_logger().error(
                    f'REAL HARDWARE DETECTED ({", ".join(conflicts)}). '
                    f'{self._purpose} is now suppressed. This package simulates the '
                    'car and must never run beside the car -- shut down '
                    'bringup_launch.py, or move the simulator to another machine.')
            else:
                self._node.get_logger().error(
                    f'still suppressed: {", ".join(conflicts)} is running.',
                    throttle_duration_sec=5.0)
            return False
        if self._blocked:
            now = self._clock()
            if self._clean_since is None:
                self._clean_since = now
            if now - self._clean_since < self._clear_confirm_sec:
                return False
            self._blocked = False
            self._node.get_logger().info(
                f'Hardware drivers are gone; {self._purpose} resumed.')
        return True
