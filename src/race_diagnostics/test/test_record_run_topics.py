"""
What record_run.py bags by default.

Read out of the launch file with `ast` rather than imported, so this runs
without ROS like the rest of this package's tests.

The requirement (docs/run-diagnostics.md, "Why a rosbag"): parameter
changes -- from the web dashboard's live tuning, from Lichtblick through
foxglove_bridge, or from a terminal -- must land in each run's bag, and
they all travel on /parameter_events.

    python3 -m pytest src/race_diagnostics/test/test_record_run_topics.py -v
"""
import ast
import os

_LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'record_run.py')


def _default_topics():
    with open(_LAUNCH) as handle:
        tree = ast.parse(handle.read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'DEFAULT_TOPICS' for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError('record_run.py no longer defines DEFAULT_TOPICS')


def test_parameter_changes_are_bagged():
    assert '/parameter_events' in _default_topics()


def test_imu_dataset_topics_are_bagged():
    """A run must be usable as an IMU dataset (src/imu_bridge/README.md):
    raw + calibrated IMU, the wheel and command inputs, the shadow EKF, and
    the pose references it is compared against."""
    required = {
        '/sensors/core', '/sensors/imu/raw', '/sensors/servo_position_command',
        '/imu/data', '/imu/vesc/data', '/imu/bno/raw', '/imu/bno/data',
        '/imu_bridge/wheel_odom', '/odometry/filtered', '/pf/viz/inferred_pose',
        '/odom', '/drive', '/ackermann_cmd',
    }
    missing = required - set(_default_topics())
    assert not missing, f'record_run.py does not bag: {sorted(missing)}'


def test_vehicle_state_outputs_are_bagged_beside_what_they_are_judged_against():
    """A fused_speed:=true run is judged by /odom/fused against /odom, and
    /steering/measured against the commanded steering on the drive topics
    (src/racerbot_launch/launch/map_course_launch.py docstring)."""
    required = {'/odom/fused', '/steering/measured', '/odom',
                '/auto_map/drive', '/drive', '/sensors/servo_position_command'}
    missing = required - set(_default_topics())
    assert not missing, f'record_run.py does not bag: {sorted(missing)}'


def test_slam_replay_and_log_topics_are_bagged():
    """Every run must be replayable through slam_toolbox with either
    odometry and judged against what SLAM did live (map_course_launch.py
    gyro_odom docs): both odometries, the full TF stream, the scans, SLAM's
    pose and pose graph; and every node's log on the same clock."""
    required = {'/odom', '/odom/imu', '/tf', '/tf_static', '/scan', '/pose',
                '/slam_toolbox/graph_visualization', '/map_metadata', '/rosout',
                '/imu/bno/data', '/imu/vesc/data', '/imu/bno/raw', '/sensors/imu/raw'}
    missing = required - set(_default_topics())
    assert not missing, f'record_run.py does not bag: {sorted(missing)}'


def test_the_launch_argument_default_is_built_from_that_list():
    """The bag records LaunchConfiguration('topics'), whose default must be
    DEFAULT_TOPICS -- otherwise editing the list changes nothing."""
    with open(_LAUNCH) as handle:
        source = handle.read()
    assert "default_value=' '.join(DEFAULT_TOPICS)" in source


def test_topic_names_are_absolute_and_unique():
    topics = _default_topics()
    assert all(t.startswith('/') and ' ' not in t for t in topics)
    assert len(topics) == len(set(topics))
