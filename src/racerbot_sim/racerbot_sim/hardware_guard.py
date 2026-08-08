"""Refuse to run the simulator while the real car's drivers are up.

Two nodes in this package are dangerous next to real hardware:

* `sim_joy_node` publishes a synthetic `/joy` with LB held. The entire
  workspace safety policy (docs/architecture.md) is that nothing moves
  unless a human is holding LB on the physical controller. A synthetic
  `/joy` is that human's hand, forged.
* `gym_bridge_node` publishes `/scan` and `/odom`. Alongside `urg_node`
  and `vesc_to_odom_node` those become two publishers per topic, and the
  driving nodes would be steering on a mixture of a real room and an
  imaginary one.

Neither is a problem on a laptop. Both are a problem on the car -- which
is exactly where this simulator is most convenient to run, since that is
where the workspace already builds. So the check is on the live ROS
graph, not on a flag someone has to remember: if the drivers are running,
these nodes hold their output and say why, and they keep checking, so
bringing the car up *after* the simulator shuts it up too.
"""

# Node names started by f1tenth_stack/launch/bringup_launch.py that mean
# "there is real hardware on this graph".
HARDWARE_NODE_NAMES = (
    'vesc_driver_node',
    'ackermann_to_vesc_node',
    'vesc_to_odom_node',
    'urg_node',
    'joy',
)


def conflicting_hardware_nodes(node) -> list:
    """Names from HARDWARE_NODE_NAMES currently visible on the ROS graph."""
    try:
        live = {name for name, _namespace in node.get_node_names_and_namespaces()}
    except Exception:  # noqa: BLE001 - a graph query must never take a node down
        return []
    return sorted(live.intersection(HARDWARE_NODE_NAMES))


class HardwareInterlock:
    """Latched 'is it safe to publish' check with a throttled explanation."""

    def __init__(self, node, purpose: str):
        self._node = node
        self._purpose = purpose
        self._blocked = False

    @property
    def blocked(self) -> bool:
        return self._blocked

    def safe(self) -> bool:
        conflicts = conflicting_hardware_nodes(self._node)
        if conflicts:
            if not self._blocked:
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
            self._blocked = False
            self._node.get_logger().info(
                f'Hardware drivers are gone; {self._purpose} resumed.')
        return True
