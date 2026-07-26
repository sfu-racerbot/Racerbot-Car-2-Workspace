"""Unit coverage for the automatic map-to-race transition helpers."""

import math
import os
import sys

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pure_pursuit.auto_map_race_node import angle_difference, LapRecorder  # noqa: E402
import pytest  # noqa: E402


def test_angle_difference_wraps_at_pi():
    assert angle_difference(-math.pi + 0.1, math.pi - 0.1) == pytest.approx(0.2)


def test_lap_recorder_requires_departure_distance_and_heading():
    recorder = LapRecorder(
        spacing=0.1,
        min_distance=3.0,
        departure_distance=0.75,
        closure_distance=0.25,
        closure_heading_rad=math.radians(15.0),
        min_duration_sec=2.0,
    )
    assert not recorder.update(0.0, 0.0, 0.0, 0.0)
    # A full 1m square, sampled in quarter-meter steps.
    samples = [
        (0.25, 0.0, 0.0), (0.5, 0.0, 0.0), (0.75, 0.0, 0.0), (1.0, 0.0, 0.0),
        (1.0, 0.25, math.pi / 2), (1.0, 0.5, math.pi / 2),
        (1.0, 0.75, math.pi / 2), (1.0, 1.0, math.pi / 2),
        (0.75, 1.0, math.pi), (0.5, 1.0, math.pi),
        (0.25, 1.0, math.pi), (0.0, 1.0, math.pi),
        (0.0, 0.75, -math.pi / 2), (0.0, 0.5, -math.pi / 2),
        (0.0, 0.25, -math.pi / 2),
    ]
    for i, (x, y, yaw) in enumerate(samples, start=1):
        assert not recorder.update(x, y, yaw, i * 0.2)

    # Near the start but facing across the start line is not completion.
    assert not recorder.update(0.05, 0.02, math.pi / 2, 3.5)
    # Same location and matching heading is a valid closure.
    assert recorder.update(0.05, 0.02, 0.0, 3.6)


def test_lap_recorder_does_not_close_before_departing():
    recorder = LapRecorder(0.1, 1.0, 0.75, 0.25, math.pi, 0.0)
    assert not recorder.update(0.0, 0.0, 0.0, 0.0)
    assert not recorder.update(0.1, 0.0, 0.0, 1.0)
    assert not recorder.update(0.0, 0.0, 0.0, 2.0)


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class _SupervisorStub:
    state = 'loading_profile'
    transition_stop_sec = 2.0

    def _now_sec(self):
        return 10.0

    def get_logger(self):
        return _Logger()


def test_profile_parameter_response_enables_racing_transition():
    class Result:
        successful = True
        reason = ''

    class Response:
        results = [Result()]

    class Future:
        def result(self):
            return Response()

    supervisor = _SupervisorStub()
    # Call the real callback against a lightweight state stub. This locks in
    # rclpy's SetParameters.Response.results shape without needing an executor.
    from pure_pursuit.auto_map_race_node import AutoMapRaceNode
    AutoMapRaceNode._profile_loaded_callback(supervisor, Future())
    assert supervisor.state == 'transition'
    assert supervisor.race_enable_time == pytest.approx(12.0)


def test_supervisor_reports_missing_then_forwards_fresh_mapping_command():
    rclpy.init(args=['--ros-args',
                     '-p', 'enable_deadman:=false',
                     '-p', 'decision_log_period_sec:=0.0'])
    from pure_pursuit.auto_map_race_node import AutoMapRaceNode
    node = AutoMapRaceNode()
    try:
        # This test targets command selection; a missing SLAM transform is
        # represented directly so no live TF graph is required.
        node._lookup_and_publish_pose = lambda: None
        published = []

        class CapturePublisher:
            def publish(self, msg):
                published.append(msg)

        node.drive_pub = CapturePublisher()

        node._control_step()
        assert node.last_decision_state == 'gap_follow_command_missing'
        assert published[-1].drive.speed == 0.0

        command = AckermannDriveStamped()
        command.drive.steering_angle = 0.12
        command.drive.speed = 0.8
        node.mapping_cmd = command
        node.mapping_cmd_time = (
            node._now_sec() - node.command_timeout_sec - 0.1)
        node._control_step()
        assert node.last_decision_state == 'gap_follow_command_stale'
        assert published[-1].drive.speed == 0.0

        node._mapping_drive_callback(command)
        node._control_step()
        assert node.last_decision_state == 'forwarding_mapping'
        assert published[-1].drive.steering_angle == pytest.approx(0.12)
        assert published[-1].drive.speed == pytest.approx(0.8)
    finally:
        node.destroy_node()
        rclpy.shutdown()
