"""
waypoint_recorder_node.py

Records the car's localized position to a .csv file while you drive a
lap, to be turned into a racing line afterward. This is Phase 3 of the
pipeline described in docs/racing-autonomy.md:

  1. Map the track once with slam_toolbox (racerbot_launch slam_launch.py).
  2. Localize against that saved map (particle_filter localize_launch.py).
  3. <- this node -> Drive one clean lap and record where the car was.
  4. Turn that recording into a paced racing line (generate_velocity_profile).
  5. Race it (pure_pursuit_node).

Usage (see docs/operations.md for the full step-by-step procedure):
    ros2 launch pure_pursuit waypoint_recorder_launch.py \
        output_file:=/home/racerbotcar-2/racerbot-ws/src/pure_pursuit/waypoints/my_track_raw.csv
Then drive one lap by hand (teleop) with localization already running,
and Ctrl+C this node once you're back at the start.
"""

import csv
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class WaypointRecorderNode(Node):
    """Samples localized (x, y) positions at a minimum spacing and appends
    them to a CSV file as they arrive."""

    def __init__(self):
        super().__init__('waypoint_recorder_node')

        self.declare_parameter('pose_topic', '/pf/viz/inferred_pose')
        self.declare_parameter('output_file', '')
        self.declare_parameter('min_spacing_m', 0.15)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.output_file = self.get_parameter('output_file').value
        self.min_spacing_m = float(self.get_parameter('min_spacing_m').value)

        if not self.output_file:
            raise RuntimeError(
                "waypoint_recorder_node: the 'output_file' parameter is not set. "
                "Point it at a writable .csv path -- see docs/operations.md."
            )

        # Opened once at startup and flushed after every single point
        # (not just on shutdown): if the Jetson crashes or the node gets
        # killed mid-lap, everything recorded up to that point is kept
        # instead of losing the whole lap.
        try:
            self._file = open(self.output_file, 'w', newline='')
        except OSError as exc:
            raise RuntimeError(
                f"waypoint_recorder_node: could not open output_file '{self.output_file}': {exc}"
            ) from exc
        self._writer = csv.writer(self._file)
        self._writer.writerow(['x', 'y'])
        self._file.flush()

        self.last_recorded_xy = None
        self.num_recorded = 0

        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)

        self.get_logger().info(
            f"Recording waypoints from '{self.pose_topic}' to '{self.output_file}' "
            f"(minimum spacing {self.min_spacing_m}m). Drive one clean lap, then Ctrl+C."
        )

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y

        if self.last_recorded_xy is not None:
            dist = math.hypot(x - self.last_recorded_xy[0], y - self.last_recorded_xy[1])
            if dist < self.min_spacing_m:
                return  # too close to the last recorded point -- skip it

        self._writer.writerow([f'{x:.4f}', f'{y:.4f}'])
        self._file.flush()
        self.last_recorded_xy = (x, y)
        self.num_recorded += 1
        if self.num_recorded % 20 == 0:
            self.get_logger().info(f"Recorded {self.num_recorded} waypoints so far.")

    def destroy_node(self):
        try:
            self._file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WaypointRecorderNode()
    except RuntimeError as exc:
        print(f"[waypoint_recorder_node] fatal: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1

    count = 0
    output_file = node.output_file
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        count = node.num_recorded
        node.destroy_node()
        rclpy.shutdown()
        print(f"[waypoint_recorder_node] done: recorded {count} waypoints to {output_file}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
