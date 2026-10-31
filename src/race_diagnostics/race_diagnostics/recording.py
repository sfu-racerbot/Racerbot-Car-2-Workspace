"""
recording.py

What a recording bags and the command that bags it, shared by launch/record_run.py
and by auto_record_node.py, which starts the bag itself rather than through
record_run.py (docs/run-diagnostics.md#automatic-recording says why). No rclpy.
"""

# Everything needed to reconstruct a run offline. /scan dominates the size
# (~10x everything else combined) but is also the only way to answer "what
# did the car actually see" after the fact, which is precisely the question
# that could not be answered about the 2026-07-27 collision.
#
# /parameter_events is every parameter change on every node, whoever made
# it: the web dashboard's live tuning, Lichtblick through foxglove_bridge,
# or `ros2 param set` in a terminal. Without it a bag can show the car
# driving differently halfway through a run with no record of why. It is a
# few small messages per change, so it costs nothing next to /scan.
DEFAULT_TOPICS = [
    '/scan', '/odom', '/slam_pose', '/drive', '/ackermann_cmd',
    '/teleop', '/tf', '/tf_static', '/map', '/joy',
    '/auto_map/drive', '/auto_race/drive',
    # Every topic that starts bringup's automatic recording
    # (config/auto_record.yaml trigger_topics), so a run always holds the
    # commands that started it: neural_racer's shadow output and the
    # VESC commands downstream of the mux. /safety/brake is team A's
    # safety node, which their ftg brakes on.
    '/neural_racer/shadow_drive', '/commands/motor/speed',
    '/commands/servo/position', '/safety/brake',
    '/parameter_events',
    # The VESC's IMU (gyro + accelerometer). Nothing drives on it yet; bagged
    # so recorded runs can show what it would do for odometry and SLAM
    # (2026-10-23: odometry's steering-based turn rate was off by 0.44 rad/s
    # RMS on the 2026-10-19 run).
    '/sensors/imu/raw',
    # With imu_bridge (src/imu_bridge/README.md) each run is also an IMU
    # dataset: calibrated IMU next to the raw one above, so a calibration can
    # be redone offline; the VESC state and servo command that explain wheel
    # speed and yaw; the shadow EKF next to the particle filter's pose it is
    # judged against. All small next to /scan; unpublished topics record nothing.
    '/sensors/core', '/sensors/servo_position_command',
    '/imu/data', '/imu/vesc/data', '/imu/bno/raw', '/imu/bno/data',
    '/imu_bridge/wheel_odom', '/odometry/filtered', '/pf/viz/inferred_pose',
    # vehicle_state_node (imu_bridge): IMU-fused speed next to /odom's wheel
    # speed, and the steering angle the car really turned at next to the
    # command on /drive -- the two comparisons a fused_speed:=true run is
    # judged on (map_course_launch.py).
    '/odom/fused', '/steering/measured',
    # Gyro odometry (imu_bridge vehicle_state_node), recorded on every run
    # whether SLAM uses it or not (map_course gyro_odom:=), so each bag can be
    # replayed through slam_toolbox both ways. SLAM's own outputs are what a
    # replay is judged against: its pose with covariance, its pose graph
    # (loop closures show here), and the map's metadata.
    '/odom/imu', '/pose', '/slam_toolbox/graph_visualization', '/map_metadata',
    # Every node's log lines, timestamped on the same clock as the data --
    # "why did it stop" and "what was it doing" in one file -- and the
    # driving nodes' own account of what they were trying to do.
    '/rosout', '/diagnostics', '/drive_intent', '/initialpose',
]

# Runs are committed to git (data/README.md), and GitHub refuses any single
# file over 100 MB. The 2026-10-19 bag was 39 MB for one session, so a longer
# one would cross it. rosbag2 only checks the size after writing a buffered
# batch, so files overshoot: measured 2026-10-25, a 1 MB split at ~4 MB/s
# gave 1.6 MB files. Half the limit leaves 50 MB for that, more than ten
# seconds of the car's whole bag even at that rate. A split bag is still
# one bag -- `ros2 bag play` and rosbag2_py.SequentialReader open the
# directory, not bag_0.mcap (checked: 17 files read back as one, 258/258).
MAX_BAG_FILE_BYTES = 50 * 1000 * 1000


def bag_record_command(bag_dir, node_name, topics):
    """The `ros2 bag record` command line for one run's bag."""
    return ['ros2', 'bag', 'record', '--node-name', node_name,
            '-o', str(bag_dir),
            '--max-bag-size', str(MAX_BAG_FILE_BYTES)] + list(topics)
