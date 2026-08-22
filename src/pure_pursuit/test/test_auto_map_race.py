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


# ============================================================================
# What the 2026-08-19 run exposed: one legitimate lap of the course this car
# actually maps is 126m and 136s, it absorbed 106 SLAM corrections along the
# way, and it measured 335deg of turning against a 300deg gate.
# ============================================================================

def test_odom_turn_survives_corrections_that_map_turn_loses():
    """The turn gate must not depend on how busy the pose graph was.

    Counting map yaw skips a correction tick, because across a re-anchor
    the yaw change is the map's and not the car's -- and the car's own
    turning during that tick goes with it. 106 corrections in one lap left
    only a 35deg margin against the 300deg gate. Odometry is never
    re-optimised, so it counts the real turning either way.
    """
    def run(with_odom):
        recorder = LapRecorder(
            spacing=0.15, min_distance=1.0, departure_distance=1.0,
            closure_distance=0.4, closure_heading_rad=math.radians(30.0),
            min_duration_sec=0.0, max_pose_jump=0.12)
        odom_yaw = 0.0
        recorder.update(0.0, 0.0, 0.0, 0.0,
                        odom_yaw if with_odom else None)
        x = y = 0.0
        for index in range(1, 121):
            # A steady 1deg per tick turn, with the map yanked sideways
            # every tenth tick.
            odom_yaw += math.radians(1.0)
            x += 0.025 * math.cos(odom_yaw)
            y += 0.025 * math.sin(odom_yaw)
            shifted_y = y + (0.8 if index % 10 == 0 else 0.0)
            recorder.update(x, shifted_y, odom_yaw, index * 0.025,
                            odom_yaw if with_odom else None)
        return recorder

    with_odom = run(True)
    without = run(False)
    assert with_odom.reanchor_count >= 10
    # 120 ticks at 1deg each is 120deg of real turning.
    assert math.degrees(with_odom.turn) == pytest.approx(120.0, abs=0.5)
    # Counting map yaw loses roughly one tick of turning per correction.
    assert math.degrees(without.turn) < math.degrees(with_odom.turn) - 8.0


def test_odom_turn_ignores_a_correction_that_only_rotates_the_map():
    """A pure map rotation is not the car turning, whatever the map says."""
    recorder = LapRecorder(
        spacing=0.15, min_distance=1.0, departure_distance=1.0,
        closure_distance=0.4, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, max_pose_jump=0.12)
    recorder.update(0.0, 0.0, 0.0, 0.0, 0.0)
    for index in range(1, 41):
        recorder.update(index * 0.025, 0.0, 0.0, index * 0.025, 0.0)
    # The pose graph re-optimises: same car, map rotated 90 degrees.
    recorder.update(0.0, 39 * 0.025, math.pi / 2, 1.05, 0.0)
    assert recorder.reanchor_count == 1
    assert recorder.turn == pytest.approx(0.0, abs=1e-9)


def test_missing_odom_falls_back_to_the_previous_behaviour():
    """Odometry not up yet is not a reason to stop counting turns."""
    recorder = LapRecorder(
        spacing=0.15, min_distance=5.0, departure_distance=1.0,
        closure_distance=0.4, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, min_turn_rad=math.radians(300.0))
    recorder.update(0.0, 0.0, 0.0, 0.0)
    closed = [recorder.update(x, y, yaw, 0.1 * i)
              for i, (x, y, yaw) in enumerate(_square_lap(), start=1)]
    assert closed[-1]


def test_a_lap_that_misses_a_fixed_gate_still_closes_once_widened():
    """The failure mode that costs a whole revolution per miss.

    A reactive controller does not repeat its line, so the car can lap a
    course indefinitely passing consistently just outside a fixed
    closure_distance. On a 126m course each of those misses is another 2.3
    minutes, which is what an operator reads as "it never switches".
    """
    def run(widen):
        recorder = LapRecorder(
            spacing=0.15, min_distance=5.0, departure_distance=1.0,
            closure_distance=0.4, closure_heading_rad=math.radians(30.0),
            min_duration_sec=0.0, min_turn_rad=math.radians(300.0),
            closure_widen_after_revolutions=1.25 if widen else 0.0,
            max_closure_distance=4.0 if widen else 0.0)
        recorder.update(0.0, 0.0, 0.0, 0.0)
        closed = []
        # Three laps of the square, every one of them passing 1.0m wide of
        # the start -- outside the 0.4m gate, inside the widened one.
        for lap in range(3):
            for x, y, yaw in _square_lap():
                closed.append(recorder.update(x, y + 1.0, yaw, 0.1 * len(closed)))
        return recorder, closed

    fixed, fixed_closed = run(False)
    assert not any(fixed_closed), 'a 1.0m miss never satisfies a 0.4m gate'
    assert fixed.revolutions > 2.5

    widened, widened_closed = run(True)
    assert any(widened_closed), 'the gate must open rather than lap forever'
    assert widened.closest_approach == pytest.approx(1.0, abs=0.3)


def test_widening_is_off_when_either_parameter_is_zero():
    recorder = LapRecorder(
        spacing=0.15, min_distance=5.0, departure_distance=1.0,
        closure_distance=0.4, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, min_turn_rad=math.radians(300.0),
        closure_widen_after_revolutions=0.0, max_closure_distance=4.0)
    recorder.update(0.0, 0.0, 0.0, 0.0)
    for _ in range(3):
        for x, y, yaw in _square_lap():
            recorder.update(x, y + 1.0, yaw, 0.1)
    assert recorder.effective_closure_distance == 0.4


def test_lap_points_trims_a_multi_revolution_recording_to_one_lap():
    """Two overlapping laps are not a racing line.

    A line fitted through both self-intersects, and the speed profile then
    brakes for corners the car will not be in.
    """
    recorder = LapRecorder(
        spacing=0.15, min_distance=5.0, departure_distance=1.0,
        closure_distance=0.4, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, min_turn_rad=math.radians(300.0),
        closure_widen_after_revolutions=1.25, max_closure_distance=4.0)
    recorder.update(0.0, 0.0, 0.0, 0.0)
    for lap in range(3):
        for x, y, yaw in _square_lap():
            recorder.update(x, y + 1.0, yaw, 0.1 * lap)

    trimmed = recorder.lap_points()
    assert len(trimmed) < len(recorder.points)
    # One lap of this square is 64 samples. Landing within a sample or two
    # of that is the whole point; landing a straight either side of it is
    # the float-tolerance bug this asserts against.
    assert 60 <= len(trimmed) <= 70
    # And it is the *end* of the recording that is kept.
    assert trimmed[-1] == recorder.points[-1]


def test_lap_points_returns_a_single_revolution_untouched():
    """The normal case must not be trimmed at all."""
    recorder = LapRecorder(
        spacing=0.15, min_distance=5.0, departure_distance=1.0,
        closure_distance=0.4, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, min_turn_rad=math.radians(300.0),
        closure_widen_after_revolutions=1.25, max_closure_distance=4.0)
    recorder.update(0.0, 0.0, 0.0, 0.0)
    for x, y, yaw in _square_lap():
        recorder.update(x, y, yaw, 0.1)
    assert recorder.lap_points() == recorder.points


def test_lap_points_trims_a_clockwise_recording_too():
    """Turning is signed, and half the courses go the other way."""
    recorder = LapRecorder(
        spacing=0.15, min_distance=5.0, departure_distance=1.0,
        closure_distance=0.4, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, min_turn_rad=math.radians(300.0),
        closure_widen_after_revolutions=1.25, max_closure_distance=4.0)
    recorder.update(0.0, 0.0, 0.0, 0.0)
    for lap in range(3):
        for x, y, yaw in _square_lap():
            recorder.update(x, -(y + 1.0), -yaw, 0.1 * lap)
    assert recorder.turn < 0.0
    trimmed = recorder.lap_points()
    assert 60 <= len(trimmed) <= 70
    assert trimmed[-1] == recorder.points[-1]


def test_closest_approach_only_counts_after_departing():
    """Sitting at the start is not a near miss."""
    recorder = LapRecorder(
        spacing=0.15, min_distance=5.0, departure_distance=1.0,
        closure_distance=0.4, closure_heading_rad=math.radians(30.0),
        min_duration_sec=0.0, min_turn_rad=math.radians(300.0))
    recorder.update(0.0, 0.0, 0.0, 0.0)
    for index in range(1, 10):
        recorder.update(0.01 * index, 0.0, 0.0, 0.01 * index)
    assert not math.isfinite(recorder.closest_approach)
    for x, y, yaw in _square_lap():
        recorder.update(x, y + 1.0, yaw, 1.0)
    assert recorder.closest_approach == pytest.approx(1.0, abs=0.3)


# ---------------------------------------------------------------------------
# Raceline optimization (Phase 4b, run inline in _write_profile)
# ---------------------------------------------------------------------------

def _circuit_map(scale=5.0, corridor_cells=44, lumpiness=0.30, resolution=0.05):
    """A closed circuit with corners tight enough for curvature to bind.

    Deliberately not an oval. On a gentle oval every point already reaches
    v_max, curvature never limits anything, and the only thing minimum
    curvature can change is to make the path longer -- so the optimizer
    correctly declines it and there is nothing to assert. A lumpy circuit
    (r varying with cos(3*theta)) has real corners, which is where a racing
    line is worth having.

    Returns the map and a lap hugging the inside of it, as gap_follow leaves
    one behind.
    """
    from pure_pursuit import occupancy_map
    size = int(2 * scale / resolution) + 80
    yy, xx = np.mgrid[0:size, 0:size]
    cx = cy = size / 2.0
    theta = np.arctan2(yy - cy, xx - cx)
    radius = np.hypot(xx - cx, yy - cy)
    wall = (scale / resolution) * (1.0 + lumpiness * np.cos(3 * theta))
    grid = np.full((size, size), 100, dtype=np.int8)
    grid[(radius > wall - corridor_cells / 2)
         & (radius < wall + corridor_cells / 2)] = 0
    occ = occupancy_map.OccupancyMap.from_grid_message(
        grid.flatten().tolist(), size, size, resolution, 0.0, 0.0,
        occupied_threshold=50, despeckle_max_cells=4)
    t = np.linspace(0.0, 2.0 * math.pi, 400, endpoint=False)
    r = scale * (1.0 + lumpiness * np.cos(3 * t)) - corridor_cells * resolution * 0.3
    lap = np.stack([cx * resolution + r * np.cos(t),
                    cy * resolution + r * np.sin(t)], 1)
    return occ, lap


class _Prepared:
    def __init__(self, xy):
        self.xy = xy


def test_optimizer_straightens_the_line_and_keeps_it_off_the_walls():
    node = _supervisor()
    try:
        occ, lap = _circuit_map()
        node.profile_grid = occ
        field = occ.clearance_field()

        def clearance_fn(xs, ys):
            return occ.clearance_at(xs, ys, field)

        line = node._optimize_line(_Prepared(lap), clearance_fn, 0.20)
        assert line is not None, 'a circuit with real corners must be optimizable'

        from pure_pursuit import racing_math
        before = np.abs(racing_math.estimate_path_curvature(lap, closed=True)).mean()
        after = np.abs(racing_math.estimate_path_curvature(line, closed=True)).mean()
        assert after < before, 'the whole point is less curvature'
        # And the acceptance rule is lap time, not curvature: a line is only
        # returned when it actually beats the one it would replace.
        assert node._estimated_lap_time(line) < node._estimated_lap_time(lap)
        # The line the car is asked to drive must clear the walls by at
        # least what was demanded, or the optimizer has quietly traded
        # safety for lap time.
        assert float(np.min(clearance_fn(line[:, 0], line[:, 1]))) >= 0.20
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_optimizer_is_skipped_when_turned_off_or_mapless():
    node = _supervisor()
    try:
        occ, lap = _circuit_map()
        node.profile_grid = occ
        node.optimize_raceline = False
        assert node._optimize_line(_Prepared(lap), None, 0.0) is None

        node.optimize_raceline = True
        node.profile_grid = None
        assert node._optimize_line(_Prepared(lap), None, 0.0) is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_optimized_line_too_close_to_a_wall_is_refused():
    """Failing the clearance check must fall back, not race the line.

    The optimizer holds itself off the walls using widths it measured
    itself. If that measurement is wrong the line goes through a wall with
    every internal number looking healthy, so the result is re-checked
    against the map and a failure returns None (= race the recording).
    """
    node = _supervisor()
    try:
        occ, lap = _circuit_map()
        node.profile_grid = occ
        # Demand more clearance than the 2.6m corridor can offer.
        line = node._optimize_line(
            _Prepared(lap),
            lambda xs, ys: np.full(len(np.atleast_1d(xs)), 0.05),
            0.20)
        assert line is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Handing localization to the particle filter
# ---------------------------------------------------------------------------

class _RunningProcess:
    """Stands in for the spawned localization launch: alive, pid-free."""

    pid = 12345
    returncode = None

    def poll(self):
        return None


def _pf_pose(x=1.0, y=2.0):
    from geometry_msgs.msg import PoseStamped
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


def test_particle_filter_is_only_trusted_after_it_settles():
    node = _supervisor()
    try:
        node.pf_process = _RunningProcess()          # pretend it was started
        node.pf_started_at = node._now_sec()
        assert not node.pf_active

        # A handful of poses is not enough to steer a car on.
        for _ in range(node.pf_settle_poses - 1):
            node._pf_pose_callback(_pf_pose())
        node._update_pf_handover(node._now_sec(), (0.0, 0.0, 0.0))
        assert not node.pf_active

        node._pf_pose_callback(_pf_pose())
        node._update_pf_handover(node._now_sec(), (0.0, 0.0, 0.0))
        assert node.pf_active, 'enough consecutive poses must promote it'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_a_silent_particle_filter_falls_back_to_slam_for_good():
    node = _supervisor()
    try:
        node.pf_process = _RunningProcess()
        node.pf_started_at = node._now_sec()
        for _ in range(node.pf_settle_poses):
            node._pf_pose_callback(_pf_pose())
        node._update_pf_handover(node._now_sec(), (0.0, 0.0, 0.0))
        assert node.pf_active

        # Nothing published for longer than the timeout.
        stale = node.pf_pose_time + node.pf_pose_timeout_sec + 0.1
        node._update_pf_handover(stale, (0.0, 0.0, 0.0))
        assert not node.pf_active
        assert node.pf_gave_up, 'one lapse at racing speed is not forgiven'

        # And it stays demoted even if poses come back.
        for _ in range(node.pf_settle_poses):
            node._pf_pose_callback(_pf_pose())
        node._update_pf_handover(node.pf_pose_time, (0.0, 0.0, 0.0))
        assert not node.pf_active
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_slow_particle_filter_is_given_up_on_at_the_timeout():
    node = _supervisor()
    try:
        node.pf_process = _RunningProcess()
        node.pf_started_at = node._now_sec()
        node._pf_pose_callback(_pf_pose())      # one lonely pose
        late = node._now_sec() + node.pf_startup_timeout_sec + 1.0
        node._update_pf_handover(late, (0.0, 0.0, 0.0))
        assert not node.pf_active
        assert node.pf_gave_up
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_the_published_pose_follows_whichever_source_is_trusted():
    """pure_pursuit reads one topic throughout; only the source changes."""
    node = _supervisor()
    try:
        published = []

        class Capture:
            def publish(self, msg):
                published.append(msg)

        node.pose_pub = Capture()
        node._pf_pose_callback(_pf_pose(x=7.5, y=-3.25))

        # Not yet trusted: the particle filter's pose must not be used, and
        # with no TF available this reports "no pose" as it always did.
        node.pf_active = False
        assert node._lookup_and_publish_pose() is None
        assert published == []

        node.pf_active = True
        pose = node._lookup_and_publish_pose()
        assert pose is not None
        assert pose[0] == pytest.approx(7.5)
        assert pose[1] == pytest.approx(-3.25)
        assert published[-1].pose.position.x == pytest.approx(7.5)
        assert published[-1].header.frame_id == node.map_frame
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_no_saved_map_means_no_handover_attempt():
    node = _supervisor()
    try:
        node.run_directory = '/tmp/definitely-not-a-run-directory-12345'
        node._start_particle_filter(node._now_sec())
        assert node.pf_process is None
        assert node.pf_gave_up, 'without a map it must commit to slam_toolbox'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_a_slower_optimized_line_is_refused():
    """The justification for optimizing is lap time, so lap time decides.

    Minimum curvature buys corner speed by using the track's full width,
    which makes the path longer. On a tight loop that trade loses -- this
    car's own 13.3m test course produced a 16.9m optimized line a second
    per lap slower. Racing it would be a regression dressed as an upgrade.
    """
    node = _supervisor()
    try:
        occ, lap = _circuit_map()
        node.profile_grid = occ
        field = occ.clearance_field()

        def clearance_fn(xs, ys):
            return occ.clearance_at(xs, ys, field)

        # Pretend every candidate is slower than what we already have.
        node._estimated_lap_time = lambda xy: (
            99.0 if len(xy) != len(lap) else 1.0)
        assert node._optimize_line(_Prepared(lap), clearance_fn, 0.20) is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_over_steering_is_tolerated_only_when_no_worse_than_the_recording():
    """`prepare` accepts a recording that exceeds the rack and warns about
    understeer. Refusing the optimized line outright for the same fault
    would discard it on exactly the tight courses where the recording is no
    better, quietly turning the whole step off."""
    from pure_pursuit import racing_math, recorded_path
    node = _supervisor()
    try:
        occ, lap = _circuit_map()
        node.profile_grid = occ
        field = occ.clearance_field()

        def clearance_fn(xs, ys):
            return occ.clearance_at(xs, ys, field)

        line = node._optimize_line(_Prepared(lap), clearance_fn, 0.20)
        assert line is not None
        limit = recorded_path.curvature_limit(
            node.profile_max_steering_angle, node.profile_wheelbase)
        got = float(np.abs(
            racing_math.estimate_path_curvature(line, closed=True)).max())
        recorded = float(np.abs(
            racing_math.estimate_path_curvature(lap, closed=True)).max())
        # Accepted, so it is either inside the rack limit or no worse than
        # the line it replaces -- never worse on both counts at once.
        assert got <= limit or got <= recorded
    finally:
        node.destroy_node()
        rclpy.shutdown()
