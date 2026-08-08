"""A synthetic F710 that holds LB, so autonomy can be exercised headlessly.

This is the one piece of the simulator that stands in for a *person*.
Every node in this workspace that can move the car refuses to publish a
non-zero command unless button 4 (LB) is held on a live `/joy` stream
(docs/architecture.md). That policy exists so nothing moves without a
hand on the controller, and this node forges exactly that hand.

Which is fine against a simulated car and unacceptable next to a real
one, so it will not publish while the car's drivers are on the graph --
see hardware_guard.py. It is also never included by any launch file
outside this package.

`release_after_sec` / `hold_after_sec` exist to test the deadman path
itself: the car should come to a stop when LB goes away mid-run, and
that is worth proving rather than assuming.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from racerbot_sim.hardware_guard import HardwareInterlock

# Matches the physical F710 in XInput mode, which is what joy_node reports
# and what every deadman check in this workspace indexes into.
F710_AXES = 8
F710_BUTTONS = 11
DEADMAN_BUTTON = 4


class SimJoyNode(Node):

    def __init__(self):
        super().__init__('sim_joy_node')

        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deadman_button', DEADMAN_BUTTON)
        self.declare_parameter('hold_deadman', True)
        # Negative means "never": hold (or stay released) for the whole run.
        self.declare_parameter('release_after_sec', -1.0)
        self.declare_parameter('hold_after_sec', -1.0)
        # Stop publishing entirely, to test the joy_stale watchdogs.
        self.declare_parameter('silence_after_sec', -1.0)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.hold_deadman = bool(self.get_parameter('hold_deadman').value)
        self.release_after_sec = float(self.get_parameter('release_after_sec').value)
        self.hold_after_sec = float(self.get_parameter('hold_after_sec').value)
        self.silence_after_sec = float(self.get_parameter('silence_after_sec').value)

        self.interlock = HardwareInterlock(self, 'the synthetic LB deadman')
        self.start_time = self.get_clock().now()
        self.publisher = self.create_publisher(
            Joy, str(self.get_parameter('joy_topic').value), 10)
        self.create_timer(1.0 / self.rate_hz, self._publish)
        self.get_logger().warn(
            'sim_joy_node is publishing a SYNTHETIC deadman: button '
            f'{self.deadman_button} (LB) reported as '
            f"{'held' if self.hold_deadman else 'released'}. This defeats the "
            'workspace LB policy by design and is for simulation only.')

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def _deadman_state(self) -> bool:
        held = self.hold_deadman
        elapsed = self._elapsed()
        if self.release_after_sec >= 0.0 and elapsed >= self.release_after_sec:
            held = False
        if self.hold_after_sec >= 0.0 and elapsed >= self.hold_after_sec:
            held = True
        return held

    def _publish(self):
        if not self.interlock.safe():
            return
        if self.silence_after_sec >= 0.0 and self._elapsed() >= self.silence_after_sec:
            return

        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'sim_joy'
        message.axes = [0.0] * F710_AXES
        message.buttons = [0] * F710_BUTTONS
        if self._deadman_state():
            message.buttons[self.deadman_button] = 1
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SimJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
