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
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('status_log_period_sec', 2.0)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.output_file = self.get_parameter('output_file').value
        self.min_spacing_m = float(self.get_parameter('min_spacing_m').value)
        self.pose_timeout_sec = max(
            0.05, float(self.get_parameter('pose_timeout_sec').value))
        self.status_log_period_sec = max(
            0.0, float(self.get_parameter('status_log_period_sec').value))

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
        self.total_recorded_distance = 0.0
        self.last_pose_xy = None
        self.last_pose_time = None
        self.last_sample_distance = None
        self.last_status_state = None
        self.last_status_log_time = None

        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)
        self.status_timer = self.create_timer(
            min(0.25, self.pose_timeout_sec / 2.0), self._status_callback)

        self.get_logger().info(
            f"Recording waypoints from '{self.pose_topic}' to '{self.output_file}' "
            f"(minimum spacing {self.min_spacing_m}m, pose timeout "
            f"{self.pose_timeout_sec:.2f}s). Drive one clean lap, then Ctrl+C; "
            f"status logs every {self.status_log_period_sec:.1f}s "
            "(plus immediate state changes)."
        )

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.last_pose_xy = (x, y)
        self.last_pose_time = self.get_clock().now()

        if self.last_recorded_xy is not None:
            dist = math.hypot(x - self.last_recorded_xy[0], y - self.last_recorded_xy[1])
            self.last_sample_distance = dist
            if dist < self.min_spacing_m:
                self._log_status('recording', self._recording_detail(0.0), level='info')
                return  # too close to the last recorded point -- skip it

        try:
            self._writer.writerow([f'{x:.4f}', f'{y:.4f}'])
            self._file.flush()
        except OSError as exc:
            self._log_status(
                'write_error',
                f"could not append waypoint {self.num_recorded + 1} to "
                f"'{self.output_file}': {exc}",
                level='error',
            )
            raise
        if self.last_recorded_xy is not None:
            self.total_recorded_distance += self.last_sample_distance
        self.last_recorded_xy = (x, y)
        self.num_recorded += 1
        self._log_status('recording', self._recording_detail(0.0), level='info')

    def _recording_detail(self, pose_age_sec: float) -> str:
        if self.last_pose_xy is None:
            return 'no localization pose has arrived yet'
        spacing_detail = (
            'first sample'
            if self.last_sample_distance is None
            else f'latest spacing={self.last_sample_distance:.3f}/{self.min_spacing_m:.3f}m')
        return (
            f'waypoints={self.num_recorded}, path distance={self.total_recorded_distance:.1f}m, '
            f'last pose=({self.last_pose_xy[0]:.2f},{self.last_pose_xy[1]:.2f}), '
            f"pose age={pose_age_sec:.2f}s, {spacing_detail}, output='{self.output_file}'")

    def _status_callback(self):
        if self.last_pose_time is None:
            self._log_status(
                'waiting_for_pose',
                f"no localization pose received on '{self.pose_topic}'; "
                'the output file still contains only its header',
            )
            return
        pose_age_sec = (
            self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if pose_age_sec >= self.pose_timeout_sec:
            self._log_status(
                'pose_stale',
                f"last localization pose is {pose_age_sec:.2f}s old "
                f"(limit {self.pose_timeout_sec:.2f}s); recording is not advancing; "
                f'waypoints={self.num_recorded}',
            )
            return
        self._log_status(
            'recording', self._recording_detail(pose_age_sec), level='info')

    def _log_status(self, state: str, detail: str, level: str = None):
        now = self.get_clock().now()
        state_changed = state != self.last_status_state
        period_elapsed = (
            self.status_log_period_sec > 0.0
            and (
                self.last_status_log_time is None
                or (now - self.last_status_log_time).nanoseconds / 1e9
                >= self.status_log_period_sec
            )
        )
        if not state_changed and not period_elapsed:
            return

        message = f"RECORDER [{state}] {detail}"
        if level == 'error':
            self.get_logger().error(message)
        elif level == 'info':
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        self.last_status_state = state
        self.last_status_log_time = now

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
