source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
ros2 launch racerbot_launch dashboard_launch.py
ros2 launch racerbot_launch realsense_camera_launch.py
ros2 launch slam_toolbox online_async_launch.py params_fie:=/racerbot-ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
ros2 launch racerbot_launch slam_launch.py
colcon build --packages-select range_lib --cmake-args -DWITH_CUDO=ON
ros2 launch particle_filter localize_launch.py