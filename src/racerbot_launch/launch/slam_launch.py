"""slam_toolbox in async mapping mode, started as the lifecycle node it is.

IMPORTANT -- why this is not a plain `Node`: as of slam_toolbox 2.x (the
`ros-jazzy-slam-toolbox` apt package here is 2.8.5) `async_slam_toolbox_node`
is an `rclcpp_lifecycle::LifecycleNode`, and it does *not* configure itself.
Everything that matters -- reading the parameter file, loading the Ceres
solver, subscribing to `/scan`, publishing `/map`, publishing the map->odom
TF, and advertising `/slam_toolbox/save_map` -- happens in `on_configure`
and `on_activate`. Launched as a plain `launch_ros.actions.Node` it comes up
in the `unconfigured` state, prints only its "Node using stack size" banner,
and then sits there doing nothing forever: no map, no `map` frame, and every
downstream node (auto_map_race_node's lap recorder, pure_pursuit's
localization, the web dashboard's map view) waits on it indefinitely.

So this file mirrors upstream's `slam_toolbox/launch/online_async_launch.py`:
start it as a LifecycleNode, emit CONFIGURE, and emit ACTIVATE once it
reaches `inactive`. The one deliberate difference is `use_sim_time`, which
upstream defaults to `true` for Gazebo; on this car the clock is real.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    default_slam_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'f1tenth_online_async.yaml'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_config,
        description='Full path to the slam_toolbox parameter file to use.')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description=(
            'Configure and activate slam_toolbox automatically. Leave true unless an '
            'external lifecycle manager is driving the transitions -- a slam_toolbox '
            'left unconfigured silently publishes no map and no map frame.'))
    use_lifecycle_manager_arg = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable the bond connection to an external nav2-style lifecycle manager.')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='False on the physical car; only a simulator publishes /clock.')

    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_lifecycle_manager': use_lifecycle_manager,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    # Only autostart when no external lifecycle manager owns the node.
    autostart_here = IfCondition(
        AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=autostart_here,
    )
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] slam_toolbox configured; activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=autostart_here,
    )

    return LaunchDescription([
        slam_params_file_arg,
        autostart_arg,
        use_lifecycle_manager_arg,
        use_sim_time_arg,
        slam_node,
        configure_event,
        activate_event,
    ])
