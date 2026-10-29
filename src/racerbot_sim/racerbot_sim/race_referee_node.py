"""ROS node that scores a multi-car simulator race from ground truth."""

from __future__ import annotations

import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from racerbot_sim import tracks
from racerbot_sim.referee import Referee, format_table
from racerbot_sim.sim_bridge import SimConfig


class RaceRefereeNode(Node):
    def __init__(self):
        super().__init__('race_referee_node')
        self.declare_parameter('track', 'indoor_oval')
        self.declare_parameter('track_directory', '~/.ros/racerbot_sim/tracks')
        self.declare_parameter('laps', 3.0)
        self.declare_parameter('controlled_cars', 2)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('reverse', False)
        self.declare_parameter('timeout_s', 300.0)
        self.declare_parameter('control_parameters_json', '{}')
        value = lambda name: self.get_parameter(name).value
        self.track = str(value('track'))
        self.track_directory = Path(os.path.expanduser(str(value('track_directory'))))
        self.cars = int(value('controlled_cars'))
        if self.cars < 1:
            raise ValueError('controlled_cars must be positive')
        self.centerline = tracks.centerline_for(self.track, self.track_directory)
        self.output_dir = Path(os.path.expanduser(str(value('output_dir')) or
            f'~/racerbot-ws/data/racerbot_sim/races/{datetime.now(timezone.utc):%Y%m%d-%H%M%S}'))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.reverse = bool(value('reverse'))
        self.laps = float(value('laps'))
        # Scoring samples are simulator control steps, never wall-clock ticks.
        self.dt_s = SimConfig.control_dt
        self.timeout_s = float(value('timeout_s'))
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError('timeout_s must be positive and finite')
        self.control_parameters = json.loads(str(value('control_parameters_json')))
        self.referee = None
        self.done = False
        latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/sim/status', self._status, latched)
        self.result_pub = self.create_publisher(String, '/sim/race_result', latched)

    def _status(self, message):
        if self.done:
            return
        try:
            status = json.loads(message.data)
            cars = status['cars']
            if len(cars) != self.cars:
                return
            t_s = float(status['sim_time_s'])
            steps = status['steps']
            if type(steps) is not int or steps < 1:
                raise ValueError('steps must be a positive integer')
            if self.referee is not None and steps <= self.referee.steps:
                return
            # All fields belong to one simulator step. Separate pose topics
            # cannot be synchronized by arrival order across ROS subscriptions.
            xys = [(float(car['x']), float(car['y'])) for car in cars]
            if self.referee is None:
                self.referee = Referee(self.centerline, xys,
                                       self.laps, self.dt_s, self.reverse)
            wall = [car['wall_contact_now'] for car in cars]
            speeds = [float(car['speed']) for car in cars]
            self.referee.tick(t_s, xys, speeds, wall, status['car_contact_now'], steps=steps)
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().error(f'cannot score simulator status: {error}')
            return
        if self.referee.finished or self.referee.sim_time_s >= self.timeout_s:
            self._finish()

    def _finish(self):
        self.done = True
        result = self.referee.result()
        result.update(track=self.track, race_laps=self.laps, controlled_cars=self.cars,
                      timed_out=not self.referee.finished,
                      parameters=self.control_parameters)
        payload = json.dumps(result, indent=2, allow_nan=False) + '\n'
        (self.output_dir / 'race.json').write_text(payload, encoding='utf-8')
        message = String()
        message.data = payload
        self.result_pub.publish(message)
        self.get_logger().info('\n' + format_table(result))
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = RaceRefereeNode()
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
