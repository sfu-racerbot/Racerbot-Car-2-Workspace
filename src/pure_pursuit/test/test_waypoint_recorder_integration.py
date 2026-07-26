"""Real-node coverage for waypoint recorder status diagnostics."""

from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.duration import Duration

from pure_pursuit.waypoint_recorder_node import WaypointRecorderNode


def _pose(x, y):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


def test_recorder_reports_pose_health_and_progress(tmp_path):
    output_path = tmp_path / 'recorded.csv'
    rclpy.init(args=['--ros-args',
                     '-p', f'output_file:={output_path}',
                     '-p', 'status_log_period_sec:=0.0'])
    node = WaypointRecorderNode()
    try:
        node._status_callback()
        assert node.last_status_state == 'waiting_for_pose'

        node.pose_callback(_pose(0.0, 0.0))
        node.pose_callback(_pose(0.05, 0.0))  # below min_spacing_m: skipped
        node.pose_callback(_pose(0.20, 0.0))
        assert node.last_status_state == 'recording'
        assert node.num_recorded == 2
        assert node.total_recorded_distance == pytest.approx(0.20)

        node.last_pose_time = node.last_pose_time - Duration(seconds=2.0)
        node._status_callback()
        assert node.last_status_state == 'pose_stale'
    finally:
        node.destroy_node()
        rclpy.shutdown()

    assert output_path.read_text().splitlines() == [
        'x,y',
        '0.0000,0.0000',
        '0.2000,0.0000',
    ]
