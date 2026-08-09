"""Unit coverage for the automatic map-to-race transition helpers."""

import math
import os
import sys

from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
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
                     '-p', 'drive_topic:=/test_only/drive',
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


def _supervisor():
    """A real AutoMapRaceNode with the deadman off and /drive remapped away
    from the live topic (these nodes publish real drive commands)."""
    rclpy.init(args=['--ros-args',
                     '-p', 'enable_deadman:=false',
                     '-p', 'drive_topic:=/test_only/drive',
                     '-p', 'decision_log_period_sec:=0.0'])
    from pure_pursuit.auto_map_race_node import AutoMapRaceNode
    return AutoMapRaceNode()


def test_profile_handover_waits_for_the_blocking_slam_save():
    """Regression test for the 2026-07-27 collision.

    slam_toolbox's save_map/serialize_map block its executor, freezing the
    map->odom transform -- and so the /slam_pose this node republishes --
    while they run. Previously the save was fired concurrently with the
    handover, so pure pursuit began racing on a pose that was already
    stale. The handover must now wait for the save to settle, which
    happens while the car is deliberately stopped.
    """
    node = _supervisor()
    try:
        loaded = []
        node._try_load_profile = lambda: loaded.append(node._now_sec())
        node.state = 'loading_profile'
        node.profile_path = '/tmp/does-not-need-to-exist.csv'
        node.run_directory = '/tmp'
        node._lookup_and_publish_pose = lambda: None

        # A save is in flight: neither completion callback has fired yet.
        node.map_save_started = True
        node.map_save_deadline = node._now_sec() + 30.0
        node.map_saves_completed = 0
        node._control_step()
        assert loaded == [], 'the profile must not be handed over mid-save'

        # The occupancy map lands, but the pose graph is still serializing.
        node.map_saves_completed = 1
        node._control_step()
        assert loaded == [], 'one of two saves finishing is not enough'

        # Both done -- now the handover may proceed.
        node.map_saves_completed = 2
        node._control_step()
        assert len(loaded) == 1, 'the profile must load once both saves settle'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_handover_proceeds_after_the_save_times_out():
    """A wedged save must not strand the car forever: the racing line is
    already on disk, and pure_pursuit's pose_frozen watchdog is the
    backstop if SLAM really is stuck."""
    node = _supervisor()
    try:
        loaded = []
        node._try_load_profile = lambda: loaded.append(True)
        node.state = 'loading_profile'
        node.profile_path = '/tmp/does-not-need-to-exist.csv'
        node.run_directory = '/tmp'
        node._lookup_and_publish_pose = lambda: None
        node.map_save_started = True
        node.map_saves_completed = 0
        node.map_save_deadline = node._now_sec() - 1.0   # already overdue

        node._control_step()
        assert len(loaded) == 1
        assert node.map_save_timed_out is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


class _Failed:
    def result(self):
        raise RuntimeError('service call failed')


def test_a_failed_save_is_retried_before_the_gate_moves_on():
    """slam_toolbox's SaveMap runs nav2's map_saver inline, and map_saver
    gives up after ~2s of "Failed to spin map subscription" if no /map
    arrives in that window. /map is republished every map_update_interval
    (5s), so whether the save works is a race against when the request
    lands -- observed failing on one run and succeeding on the next with no
    other difference. A retry lands in a different part of that window.

    While a retry is pending the save is deliberately *not* settled: another
    request is about to block slam_toolbox's executor again, which is
    exactly what the handover gate exists to wait out.
    """
    node = _supervisor()
    try:
        node.map_save_attempts = 1          # as if one request went out
        node._map_save_callback(_Failed(), 'occupancy map')
        assert node.map_saves_completed == 0, 'not settled while a retry is due'
        assert node.map_save_retry_at is not None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_a_failed_save_settles_once_the_retries_are_used_up():
    """The gate cares whether slam_toolbox is still blocked, not whether
    the save succeeded -- once nothing more will be requested, a failed save
    is no longer holding the executor and racing must not wait on it."""
    node = _supervisor()
    try:
        node.map_save_attempts = node.map_save_retries + 1
        node._map_save_callback(_Failed(), 'occupancy map')
        assert node.map_saves_completed == 1
        assert node.map_save_retry_at is None
        assert not node.map_saved_ok
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_a_successful_save_is_not_retried():
    node = _supervisor()
    try:
        class _Ok:
            def result(self):
                class _Response:
                    result = 0
                return _Response()
        node.map_save_attempts = 1
        node._map_save_callback(_Ok(), 'occupancy map')
        assert node.map_saves_completed == 1
        assert node.map_save_retry_at is None
        assert node.map_saved_ok
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_a_failed_pose_graph_save_is_never_retried():
    """Only the occupancy map has the map_saver race. Retrying the pose
    graph would just block slam_toolbox again for no reason."""
    node = _supervisor()
    try:
        node._map_save_callback(_Failed(), 'pose graph')
        assert node.map_saves_completed == 1
        assert node.map_save_retry_at is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Localisation corrections, and what counts as a lap.
# ---------------------------------------------------------------------------

def _square_lap(size=4.0, step=0.25):
    """One anticlockwise lap of a square, as (x, y, yaw) samples.

    16m round, which is the point: it is the size of the loop this car is
    actually driven on, and smaller than the 20m `minimum_lap_distance`
    used to demand. Ends back at the start pointing the way it set off, so
    the closure heading gate is satisfied.
    """
    samples = []
    count = int(size / step)
    for i in range(1, count + 1):
        samples.append((i * step, 0.0, 0.0))
    for i in range(1, count + 1):
        samples.append((size, i * step, math.pi / 2))
    for i in range(1, count + 1):
        samples.append((size - i * step, size, math.pi))
    for i in range(1, count + 1):
        samples.append((0.0, size - i * step, -math.pi / 2))
    samples.append((0.0, 0.0, 0.0))
    return samples


def test_recorder_absorbs_a_slam_correction_instead_of_recording_it():
    """A pose that teleports is slam_toolbox re-optimising, not the car.

    Recorded verbatim those corrections become geometry, and on this car's
    real laps they were most of it -- a median 8.8-15.5 degrees of heading
    change between consecutive 0.15m samples, and a racing line demanding
    more steering than the rack has on a third of its waypoints.
    """
    recorder = LapRecorder(
        spacing=0.15, min_distance=1.0, departure_distance=1.0,
        closure_distance=0.3, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, max_pose_jump=0.12)

    # 0.025m per tick is 1.0m/s at the supervisor's 40Hz -- mapping speed.
    for index in range(40):
        recorder.update(index * 0.025, 0.0, 0.0, index * 0.025)
    before = list(recorder.points)
    assert recorder.reanchor_count == 0

    # The map shifts 0.8m sideways under the car between two ticks.
    recorder.update(39 * 0.025, 0.8, 0.0, 1.0)
    assert recorder.reanchor_count == 1
    # Every stored point moved with it, so the recorded *shape* is unchanged.
    after = np.asarray(recorder.points[:len(before)])
    assert np.allclose(np.diff(after, axis=0), np.diff(np.asarray(before), axis=0))
    # ...and no 0.8m step was recorded as a corner.
    steps = np.hypot(*np.diff(np.asarray(recorder.points), axis=0).T)
    assert steps.max() < 0.3


def test_recorder_reanchors_rotation_as_well_as_translation():
    """A correction can rotate the map, not just slide it."""
    recorder = LapRecorder(
        spacing=0.15, min_distance=1.0, departure_distance=1.0,
        closure_distance=0.3, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, max_pose_jump=0.12)
    for index in range(40):
        recorder.update(index * 0.025, 0.0, 0.0, index * 0.025)
    original = np.asarray(recorder.points)

    recorder.update(39 * 0.025, 0.5, math.pi / 2, 1.0)
    moved = np.asarray(recorder.points[:len(original)])
    # A rigid transform preserves every pairwise distance.
    def pairwise(points):
        return np.hypot(*(points[:, None, :] - points[None, :, :]).T)
    assert np.allclose(pairwise(original), pairwise(moved), atol=1e-9)


def test_recorder_without_jump_detection_records_the_correction():
    """max_pose_jump_m: 0.0 keeps the old behaviour, for comparison runs."""
    recorder = LapRecorder(
        spacing=0.15, min_distance=1.0, departure_distance=1.0,
        closure_distance=0.3, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, max_pose_jump=0.0)
    for index in range(40):
        recorder.update(index * 0.025, 0.0, 0.0, index * 0.025)
    recorder.update(39 * 0.025, 0.8, 0.0, 1.0)
    assert recorder.reanchor_count == 0
    steps = np.hypot(*np.diff(np.asarray(recorder.points), axis=0).T)
    assert steps.max() > 0.7


def test_a_lap_shorter_than_minimum_lap_distance_never_closes():
    """The defect behind every two-revolution recording this car made.

    `minimum_lap_distance` was 20.0m and the course is about 15m round, so
    the gate could not open until the car had been round twice -- and two
    overlapping passes are not a closed racing line.
    """
    recorder = LapRecorder(
        spacing=0.15, min_distance=20.0, departure_distance=1.0,
        closure_distance=0.4, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, min_turn_rad=math.radians(300.0))
    recorder.update(0.0, 0.0, 0.0, 0.0)
    closed = [recorder.update(x, y, yaw, 0.1 * i)
              for i, (x, y, yaw) in enumerate(_square_lap(), start=1)]
    assert not any(closed), 'a 16m lap cannot satisfy a 20m distance gate'


def test_turn_gate_closes_a_lap_the_distance_gate_would_miss():
    """Accumulated yaw does not need to be told how big the course is: one
    lap of a closed circuit is 360 degrees of turning whatever its size."""
    recorder = LapRecorder(
        spacing=0.15, min_distance=5.0, departure_distance=1.0,
        closure_distance=0.4, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, min_turn_rad=math.radians(300.0))
    recorder.update(0.0, 0.0, 0.0, 0.0)
    closed = [recorder.update(x, y, yaw, 0.1 * i)
              for i, (x, y, yaw) in enumerate(_square_lap(), start=1)]
    assert closed[-1], 'one full square is one lap'
    assert not any(closed[:-1]), 'and nothing before it is'
    assert abs(recorder.turn) >= math.radians(300.0)


def test_turn_gate_rejects_a_there_and_back_again_run():
    """Driving out and reversing back to the start passes every distance
    and proximity gate and is not a lap: it accumulates no net turning."""
    recorder = LapRecorder(
        spacing=0.15, min_distance=1.0, departure_distance=1.0,
        closure_distance=0.4, closure_heading_rad=math.radians(180.0),
        min_duration_sec=0.0, min_turn_rad=math.radians(300.0))
    recorder.update(0.0, 0.0, 0.0, 0.0)
    out = [(i * 0.25, 0.0, 0.0) for i in range(1, 17)]
    back = [(4.0 - i * 0.25, 0.0, math.pi) for i in range(1, 17)]
    closed = [recorder.update(x, y, yaw, 0.1 * i)
              for i, (x, y, yaw) in enumerate(out + back, start=1)]
    assert not any(closed)
