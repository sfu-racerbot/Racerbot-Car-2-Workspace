"""
Which point on the car a localization pose describes.

particle_filter ray-casts every particle's scan from the particle's own pose
(particle_filter.py, sensor_model: ``queries = proposal_dist``), so each
particle -- and the weighted mean it publishes as /pf/viz/inferred_pose -- is
a pose of the **LiDAR**, 0.26 m ahead of the rear axle. Everything in
pure_pursuit (pure pursuit geometry, the racing line, the map ray caster that
adds laser_offset_x on top) assumes the rear-axle base_link pose that
docs/hardware-reference.md defines. Until 2026-09-27 the PF pose was used as
base_link unconverted, putting the car 0.26 m ahead of where it was.

Oracle throughout: the rigid 0.26 m offset along the car's heading, closed
form (racing_math.laser_pose_to_base_link is itself tested against the same
closed form in test_racing_math.py).

Needs ROS2 sourced:
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/pure_pursuit/test/test_pose_frame.py -v
"""
import math
import os
import re

import pytest
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped

from pure_pursuit import racing_math
from pure_pursuit.pure_pursuit_node import PurePursuitNode
from pure_pursuit.waypoint_recorder_node import WaypointRecorderNode

HERE = os.path.dirname(__file__)
LASER_OFFSET_X = 0.26  # m, docs/hardware-reference.md (measured 2026-08-24)


def _pose(x, y, yaw):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


@pytest.fixture(scope='module')
def profiled_csv(tmp_path_factory):
    xy = racing_math.load_xy_csv(
        os.path.join(HERE, '..', 'waypoints', 'example_stadium_raw.csv'))
    seg_len = racing_math.compute_segment_lengths(xy, closed=True)
    curvature = racing_math.estimate_path_curvature(xy, closed=True)
    speed = racing_math.compute_velocity_profile(
        seg_len, curvature, v_max=6.0, v_min=0.5, a_lat_max=8.0,
        a_accel_max=3.0, a_brake_max=8.0, closed=True)
    out_path = str(tmp_path_factory.mktemp('waypoints') / 'profiled.csv')
    racing_math.save_profiled_csv(out_path, xy, speed)
    return out_path


def _pure_pursuit(profiled_csv, *extra):
    # Deadman off only together with the /drive remap (CLAUDE.md): nothing in
    # this file ticks the control loop, but a stray tick must not reach the mux.
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'enable_deadman:=false',
                     '-p', 'drive_topic:=/test_only/drive', *extra])
    try:
        return PurePursuitNode()
    except Exception:
        rclpy.shutdown()
        raise


# --- pure_pursuit_node -------------------------------------------------------

def test_pure_pursuit_converts_a_lidar_pose_to_the_rear_axle(profiled_csv):
    """Default pose_frame is 'laser', matching the default pose_topic
    /pf/viz/inferred_pose. Facing +y, the rear axle is 0.26 m in -y."""
    node = _pure_pursuit(profiled_csv)
    try:
        node.pose_callback(_pose(5.0, 2.0, math.pi / 2))
        assert node.car_x == pytest.approx(5.0, abs=1e-9)                   # m
        assert node.car_y == pytest.approx(2.0 - LASER_OFFSET_X, abs=1e-9)  # m
        assert node.car_yaw == pytest.approx(math.pi / 2, abs=1e-9)         # rad
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pure_pursuit_converts_using_its_configured_offset(profiled_csv):
    # 0.20 m: a different offset still inside the car's footprint (the node
    # refuses a LiDAR origin outside the body at startup).
    node = _pure_pursuit(profiled_csv, '-p', 'laser_offset_x:=0.20')
    try:
        node.pose_callback(_pose(5.0, 2.0, 0.0))
        assert node.car_x == pytest.approx(4.8, abs=1e-9)  # m
        assert node.car_y == pytest.approx(2.0, abs=1e-9)  # m
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pure_pursuit_leaves_a_base_link_pose_alone(profiled_csv):
    """/slam_pose (auto_map_race) is already base_link: converting it again
    would put the car 0.26 m *behind* itself."""
    node = _pure_pursuit(profiled_csv, '-p', 'pose_frame:=base_link')
    try:
        node.pose_callback(_pose(5.0, 2.0, math.pi / 2))
        assert (node.car_x, node.car_y) == pytest.approx((5.0, 2.0), abs=1e-12)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pure_pursuit_refuses_an_unknown_pose_frame(profiled_csv):
    rclpy.init(args=['--ros-args',
                     '-p', f'waypoints_file:={profiled_csv}',
                     '-p', 'enable_deadman:=false',
                     '-p', 'drive_topic:=/test_only/drive',
                     '-p', 'pose_frame:=lidar'])
    try:
        with pytest.raises(RuntimeError, match="pose_frame"):
            PurePursuitNode()
    finally:
        rclpy.shutdown()


# --- waypoint_recorder_node --------------------------------------------------

def _recorder(output_path, *extra):
    rclpy.init(args=['--ros-args',
                     '-p', f'output_file:={output_path}',
                     '-p', 'status_log_period_sec:=0.0', *extra])
    try:
        return WaypointRecorderNode()
    except Exception:
        rclpy.shutdown()
        raise


def test_recorder_writes_the_rear_axle_position_of_a_lidar_pose(tmp_path):
    output_path = tmp_path / 'recorded.csv'
    node = _recorder(output_path)
    try:
        node.pose_callback(_pose(5.0, 2.0, math.pi / 2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    assert output_path.read_text().splitlines() == ['x,y', '5.0000,1.7400']


def test_recorder_leaves_a_base_link_pose_alone(tmp_path):
    output_path = tmp_path / 'recorded.csv'
    node = _recorder(output_path, '-p', 'pose_frame:=base_link')
    try:
        node.pose_callback(_pose(5.0, 2.0, math.pi / 2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    assert output_path.read_text().splitlines() == ['x,y', '5.0000,2.0000']


def test_recorder_refuses_an_unknown_pose_frame(tmp_path):
    rclpy.init(args=['--ros-args',
                     '-p', f'output_file:={tmp_path / "r.csv"}',
                     '-p', 'pose_frame:=lidar'])
    try:
        with pytest.raises(RuntimeError, match="pose_frame"):
            WaypointRecorderNode()
    finally:
        rclpy.shutdown()


# --- shipped configuration pairs each pose topic with its real frame --------
# The failure this guards against is silent: pointing a node at a base_link
# topic while it still believes 'laser' (or the reverse) moves the car 0.26 m.

def _params(name, node_name):
    with open(os.path.join(HERE, '..', 'config', name)) as f:
        return yaml.safe_load(f)[node_name]['ros__parameters']


@pytest.mark.parametrize('config, node_name', [
    ('pure_pursuit.yaml', 'pure_pursuit_node'),
    ('waypoint_recorder.yaml', 'waypoint_recorder_node'),
])
def test_shipped_particle_filter_topic_is_declared_as_a_lidar_pose(config, node_name):
    params = _params(config, node_name)
    assert params['pose_topic'] == '/pf/viz/inferred_pose'
    assert params['pose_frame'] == 'laser'
    assert params['laser_offset_x'] == LASER_OFFSET_X


def test_auto_map_race_launch_declares_slam_pose_as_base_link():
    """auto_map_race_launch.py points pure_pursuit at /slam_pose, which
    auto_map_race_node publishes as base_link (it converts the particle
    filter's pose before republishing it -- see test_auto_map_race.py)."""
    path = os.path.join(HERE, '..', '..', 'racerbot_launch', 'launch',
                        'auto_map_race_launch.py')
    with open(path) as f:
        source = f.read()
    block = re.search(r"executable='pure_pursuit_node'.*?\}\]", source, re.S)
    assert block, 'pure_pursuit_node block not found in auto_map_race_launch.py'
    assert "'pose_topic': '/slam_pose'" in block.group(0)
    assert "'pose_frame': 'base_link'" in block.group(0)
