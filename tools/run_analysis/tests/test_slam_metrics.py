"""Tests for tools.run_analysis.slam_metrics. ROS-free, so this runs unsourced.

Runner: python3 -m pytest tools/run_analysis/tests/test_slam_metrics.py -v

Oracle markers (TEST_QUALITY_STANDARDS.md A3): "closed form" = recomputed in the test from
the geometry or the stated thresholds; "invariant" = a property every correct answer has;
"log" = a line copied verbatim from the recorded run
~/.ros/racerbot_runs/map_course-20261026-194912.log.
"""
import math
import os
import sys
from datetime import datetime

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # insert tools/
from run_analysis import slam_metrics  # noqa: E402

# oracle: log - lines 199 and 223 of map_course-20261026-194912.log, verbatim
CERES_A = "[async_slam_toolbox_node-4] W20261026 19:49:52.013816 166593 preprocessor.cc:67] Specified options.num_threads: 50 exceeds maximum available from the threading model Ceres was compiled with: 6.  Bounding to maximum number available."
CERES_B = "[async_slam_toolbox_node-4] W20261026 19:49:59.926345 166541 preprocessor.cc:67] Specified options.num_threads: 50 exceeds maximum available from the threading model Ceres was compiled with: 6.  Bounding to maximum number available."
# oracle: log - line 2 of the same file, a real line that is not a Ceres solve
LAUNCH_LINE = "[INFO] [launch]: Default logging verbosity is set to INFO"
# synthetic, not from the log: the same W-stamp format with no solve marker, so the regex must require num_threads
NOT_A_SOLVE = "[async_slam_toolbox_node-4] W20261026 19:50:00.000001 166593 preprocessor.cc:70] Some other warning."


def _car_at(x, y, yaw):
    """Odometry that holds the car still at (x, y, yaw) from t=0 to t=10 s."""
    return np.array([[0.0, x, y, yaw], [10.0, x, y, yaw]])


def test_pure_translation_jump_is_measured_at_the_car():
    # oracle: closed form - both corrections have yaw 0, so the car's displacement is the
    # correction's own (2.4, 0) jump, wherever the car is (here at (3, -1) with yaw 0.4).
    map_odom = np.array([[0.0, 0.0, 0.0, 0.0], [5.0, 2.4, 0.0, 0.0], [10.0, 2.4, 0.0, 0.0]])
    events = slam_metrics.correction_events(map_odom, _car_at(3.0, -1.0, 0.4))
    assert events.shape == (1, 3)
    assert events[0, 0] == 5.0  # the change is stamped with the time the new correction appears
    assert events[:, 1] == pytest.approx([2.4], abs=1e-9)  # m


def test_rotation_about_the_map_origin_moves_the_car_by_the_chord():
    # oracle: closed form - the correction turns by theta about the map origin (identity to
    # (0, 0, theta)) with the car on a circle of radius r about that origin. The car's
    # displacement is the chord 2 r sin(theta/2), and the rotation is theta itself.
    r, theta = 5.0, 0.1
    map_odom = np.array([[0.0, 0.0, 0.0, 0.0], [5.0, 0.0, 0.0, theta]])
    events = slam_metrics.correction_events(map_odom, _car_at(r, 0.0, 0.0))
    assert events.shape == (1, 3)
    assert events[0, 1] == pytest.approx(2 * r * math.sin(theta / 2), abs=1e-9)  # m, the chord
    assert events[0, 2] == pytest.approx(theta, abs=1e-12)  # rad, the correction's own turn


def test_no_change_no_events():
    # invariant - a correction that never changes produces no events, whatever the car does.
    map_odom = np.array([[0.0, 0.5, -0.2, 0.3], [1.0, 0.5, -0.2, 0.3], [2.0, 0.5, -0.2, 0.3]])
    odom = np.array([[0.0, 0.0, 0.0, 0.0], [2.0, 4.0, 1.0, 0.5]])  # the car moves
    assert slam_metrics.correction_events(map_odom, odom).shape == (0, 3)


def test_empty_and_single_sample_inputs():
    # invariant - no change between samples means no events; zero or one sample is not an error.
    assert slam_metrics.correction_events(np.zeros((0, 4)), np.zeros((0, 4))).shape == (0, 3)
    one = np.array([[0.0, 1.0, 2.0, 0.3]])
    assert slam_metrics.correction_events(one, one).shape == (0, 3)
    assert slam_metrics.correction_events(one, np.zeros((0, 4))).shape == (0, 3)


def test_nan_rows_are_skipped_not_propagated():
    odom = _car_at(5.0, 0.0, 0.0)
    nan_row = [1.0, np.nan, np.nan, np.nan]
    # invariant - a NaN row between two equal corrections is no change, and no NaN comes out.
    equal = np.array([[0.0, 0.0, 0.0, 0.0], nan_row, [2.0, 0.0, 0.0, 0.0]])
    assert slam_metrics.correction_events(equal, odom).shape == (0, 3)
    # A real jump across the NaN row is still measured once, at the row after it.
    # oracle: closed form - the pure-translation case, 2.4 m.
    jump = np.array([[0.0, 0.0, 0.0, 0.0], nan_row, [2.0, 2.4, 0.0, 0.0]])
    events = slam_metrics.correction_events(jump, odom)
    assert events.shape == (1, 3)
    assert events[0, 0] == 2.0
    assert events[0, 1] == pytest.approx(2.4, abs=1e-9)  # m
    assert np.isfinite(events).all()


def test_yaw_wraps_at_pi():
    # oracle: closed form - from yaw 3.13 to -3.13 the short way round is 2*pi - 6.26 rad
    # (0.0232 to three significant figures), not 6.26.
    map_odom = np.array([[0.0, 0.0, 0.0, 3.13], [1.0, 0.0, 0.0, -3.13]])
    events = slam_metrics.correction_events(map_odom, _car_at(5.0, 0.0, 0.0))
    assert events.shape == (1, 3)
    assert events[0, 2] == pytest.approx(2 * math.pi - 6.26, abs=1e-9)  # rad


def test_loop_closure_times_parse_the_ceres_lines():
    # oracle: log for the two Ceres lines. The times are closed form: datetime(...).timestamp()
    # reads the stamp as local time, which is the log's clock (the machine is on Pacific time).
    text = "\n".join([CERES_A, LAUNCH_LINE, NOT_A_SOLVE, CERES_B])
    times = slam_metrics.loop_closure_times(text)
    assert times.shape == (2,)
    assert times[0] == pytest.approx(datetime(2026, 10, 26, 19, 49, 52, 13816).timestamp(), abs=1e-6)  # s
    assert times[1] == pytest.approx(datetime(2026, 10, 26, 19, 49, 59, 926345).timestamp(), abs=1e-6)  # s
    assert slam_metrics.loop_closure_times("").shape == (0,)


def test_summarize_counts_only_jumps_near_closures():
    # oracle: closed form - the brief's synthetic case. Counts follow from the stated thresholds:
    # over 0.3 m, over 1.5 m, and within 1.5 s of a closure.
    events = np.array([[10.0, 2.4, 0.0], [20.0, 0.4, 0.0], [30.0, 0.05, 0.0]])
    s = slam_metrics.summarize(events, np.array([10.5, 30.2]))
    assert s["updates"] == 3
    assert s["loop_closures"] == 2
    assert s["jumps_over_0_3m"] == 2  # 2.4 and 0.4 m; 0.05 m is under 0.3 m
    assert s["jumps_over_false_jump_m"] == 1  # 2.4 m only
    assert s["jumps_at_loop_closures"] == 1  # 2.4 m is 0.5 s from 10.5 s; 0.4 m is 9.5 s from any closure
    assert s["max_move_m"] == pytest.approx(2.4, abs=1e-9)  # m


def test_summarize_thresholds_are_strict_and_the_window_inclusive():
    # oracle: closed form - "over" is strictly greater than the threshold, and "within
    # window_s" is |dt| <= window_s. A move of exactly 0.3 m is not over 0.3 m; a move of
    # exactly 1.5 m is over 0.3 m but not over 1.5 m; a closure exactly 1.5 s away is near.
    events = np.array([[10.0, 0.3, 0.0], [20.0, 1.5, 0.0], [30.0, 0.5, 0.0]])
    s = slam_metrics.summarize(events, np.array([31.5]))
    assert s["jumps_over_0_3m"] == 2
    assert s["jumps_over_false_jump_m"] == 0
    assert s["jumps_at_loop_closures"] == 1


def test_summarize_no_events_is_all_zero():
    # invariant - no events means no moves and no jumps; the closures are still counted.
    s = slam_metrics.summarize(np.zeros((0, 3)), np.array([5.0]))
    assert s == {"updates": 0, "loop_closures": 1, "max_move_m": 0.0, "max_rotate_deg": 0.0,
                 "jumps_over_0_3m": 0, "jumps_over_false_jump_m": 0, "jumps_at_loop_closures": 0}


def test_summarize_reports_the_largest_turn_in_unsigned_degrees():
    # oracle: closed form - the largest magnitude is 0.3 rad (the -0.3 sample), in degrees.
    events = np.array([[1.0, 0.0, 0.1], [2.0, 0.0, -0.3], [3.0, 0.0, 0.05]])
    s = slam_metrics.summarize(events, np.array([]))
    assert s["max_rotate_deg"] == pytest.approx(math.degrees(0.3), abs=1e-9)  # deg


def test_changes_without_odometry_raise_instead_of_reporting_no_events():
    # invariant - a correction that changed, with no odometry to place the car, is an error.
    # Zero events would read as "SLAM never corrected", a false pass.
    map_odom = np.array([[0.0, 0.0, 0.0, 0.0], [1.0, 2.4, 0.0, 0.0]])
    with pytest.raises(ValueError, match="no odometry rows"):
        slam_metrics.correction_events(map_odom, np.zeros((0, 4)))
    with pytest.raises(ValueError, match="no odometry rows"):
        slam_metrics.correction_events(map_odom, np.full((2, 4), np.nan))


def test_wrong_shapes_and_non_finite_events_are_refused():
    with pytest.raises(ValueError, match="4 columns"):
        slam_metrics.correction_events(np.zeros((3, 3)), np.zeros((0, 4)))
    with pytest.raises(ValueError, match="3 columns"):
        slam_metrics.summarize(np.zeros((2, 4)), np.array([]))
    with pytest.raises(ValueError, match="non-finite"):
        slam_metrics.summarize(np.array([[1.0, np.nan, 0.0]]), np.array([]))
