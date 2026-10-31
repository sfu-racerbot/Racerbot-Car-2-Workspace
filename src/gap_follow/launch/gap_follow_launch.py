import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from gap_follow.launch_speed import speed_cap_overrides, yaml_parameters
from gap_follow.launch_imu import select_imu_hardware


FUSED_ODOM = '/odom/fused'   # vehicle_state.yaml's fused_odom_topic


def _flag(name, value):
    """A launch boolean, read the way racerbot_launch's map_course_launch.py reads its own."""
    text = str(value).strip().lower()
    if text in ('true', '1'):
        return True
    if text in ('false', '0'):
        return False
    raise RuntimeError(f"{name}:={value!r}: expected 'true' or 'false'")


def generate_launch_description(imu_probe=None):
    gap_follow_config = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'gap_follow.yaml'
    )

    opponent_arg = DeclareLaunchArgument(
        'opponent_awareness', default_value='true',
        description='on by default since 2026-10-28; false turns the whole layer off. '
                    'When on: track moving cars, follow them (the follow cap is held '
                    'for opponent_follow_memory_sec after one is lost), and pass only '
                    'with room on a straight (docs/gap-follow.md#several-cars-at-once). '
                    'Simulator-validated only.')
    max_speed_arg = DeclareLaunchArgument(
        'max_speed', default_value='',
        description='Top speed for this run (m/s), up to 5.0: e.g. 1.0 for '
                    'a first floor test, 5.0 for a fast straight. Empty: '
                    'gap_follow.yaml\'s max_speed (5.0). Refused above 5.0. Below '
                    'the YAML value, min_speed, corner_speed and '
                    'corner_speed_wide are lowered with it; above it, only '
                    'max_speed rises.')

    imu_arg = DeclareLaunchArgument(
        'imu', default_value='true',
        description='Allow the IMU stack; false disables default steering damping. '
                    'Enabled IMU features require imu:=true.')
    imu_all_arg = DeclareLaunchArgument(
        'imu_all', default_value='false',
        description='Enable fused_speed, imu_understeer, imu_traction and imu_impact_stop '
                    'unless explicitly set. Adaptive grip is excluded: its small floor '
                    'benefit and slower simulator result leave it for a separate trial.')
    imu_hardware_arg = DeclareLaunchArgument(
        'imu_hardware', default_value='auto',
        description='IMU source: auto uses simulated topics when gym_bridge_node is running, '
                    'otherwise real hardware. true/false explicitly select real/simulated inputs.')
    steering_damping_arg = DeclareLaunchArgument(
        # Car 2: a literal false, not car 1's imu default, until its BNO085 is
        # calibrated (docs/car1-vs-car2.md#imu). imu:=true only allows it.
        'steering_damping', default_value='false',
        description='true: steer against the car\'s own rotation, using the gyro yaw '
                    'rate in /imu/data. Starts imu_bridge\'s vehicle_state_launch.py '
                    '(the IMU stack). Car 2: false until its BNO085 is mounted and '
                    'calibrated (docs/car1-vs-car2.md#imu); car 1 ships true. '
                    'Simulator-validated only, not yet run on the floor '
                    '(docs/gap-follow.md#steering-damping-gyro).')
    adaptive_grip_arg = DeclareLaunchArgument(
        'adaptive_grip', default_value='false',
        description='true: raise or cut the lateral limit behind the corner speed '
                    'caps from the gyro\'s grip ratio, starting at max_lateral_accel. '
                    'Starts imu_bridge\'s vehicle_state_launch.py (the IMU stack), '
                    'once, even when steering_damping:=true as well. Ships off; '
                    'simulator-validated only, not yet run on the floor '
                    '(docs/gap-follow.md#adaptive-grip-imu).')

    launch_boost_arg = DeclareLaunchArgument(
        'launch_boost', default_value='true',
        description='true (the default since 2026-10-30): full speed off the start '
                    'line on the first LB press, once per launch -- no ramp, corner, '
                    'corridor or opponent caps until the end of the straight makes the '
                    'car brake, or 3 s / 5 m pass. Deadman, TTC, emergency clearance and '
                    'the braking and curvature ceilings still apply. false turns it off '
                    '(docs/gap-follow.md#launch-boost). Simulator-validated only.')

    fused_speed_arg = DeclareLaunchArgument(
        'fused_speed', default_value=LaunchConfiguration('imu_all', default='false'),
        description='true: read speed from /odom/fused (wheels plus BNO forward acceleration). '
                    'Starts the shared IMU stack once, including with either gyro feature on. '
                    'Ships off; floor trial needed (docs/gap-follow.md#fused-speed-imu).')

    understeer_arg = DeclareLaunchArgument(
        'imu_understeer', default_value=LaunchConfiguration('imu_all', default='false'),
        description='true: cap speed when measured steering shows sustained understeer. '
                    'Starts the shared IMU stack once. Ships off; floor trial needed '
                    '(docs/gap-follow.md#understeer-check-imu).')

    traction_arg = DeclareLaunchArgument(
        'imu_traction', default_value=LaunchConfiguration('imu_all', default='false'),
        description='true: hold acceleration briefly when wheel speed exceeds fused speed. '
                    'Starts the shared IMU stack once. Ships off; floor trial needed '
                    '(docs/gap-follow.md#traction-control-imu).')

    impact_arg = DeclareLaunchArgument(
        'imu_impact_stop', default_value=LaunchConfiguration('imu_all', default='false'),
        description='true: latch a zero-speed command on a trusted horizontal acceleration '
                    'spike. Release LB and press it again to resume. Ships off; threshold '
                    'is provisional until a recorded wheels-off bump. Shares the IMU stack '
                    '(docs/gap-follow.md#impact-stop-imu).')

    def node(context):
        imu = _flag('imu', LaunchConfiguration('imu').perform(context))
        all_imu = _flag('imu_all', LaunchConfiguration('imu_all').perform(context))
        enabled = LaunchConfiguration('opponent_awareness').perform(context).lower()
        if enabled not in ('true', 'false'):
            raise RuntimeError(
                f"opponent_awareness:={enabled!r}: use true or false")
        damping = _flag('steering_damping',
                        LaunchConfiguration('steering_damping').perform(context))
        grip = _flag('adaptive_grip',
                     LaunchConfiguration('adaptive_grip').perform(context))
        boost = _flag('launch_boost',
                      LaunchConfiguration('launch_boost').perform(context))
        fused = _flag('fused_speed',
                      LaunchConfiguration('fused_speed').perform(context))
        understeer = _flag('imu_understeer',
                          LaunchConfiguration('imu_understeer').perform(context))
        traction = _flag('imu_traction', LaunchConfiguration('imu_traction').perform(context))
        impact = _flag('imu_impact_stop', LaunchConfiguration('imu_impact_stop').perform(context))
        if not imu:
            features = (('steering_damping', damping), ('adaptive_grip', grip),
                        ('fused_speed', fused), ('imu_understeer', understeer),
                        ('imu_traction', traction), ('imu_impact_stop', impact))
            for name, active in features:
                if active:
                    source = ' (check imu_all:=true)' if all_imu else ''
                    raise RuntimeError(
                        f'{name}:=true{source} needs imu:=true; '
                        f'pass imu:=true or {name}:=false')
        speed = speed_cap_overrides(
            yaml_parameters(gap_follow_config),
            LaunchConfiguration('max_speed').perform(context))
        actions = [Node(
            package='gap_follow',
            executable='gap_follow_node',
            name='gap_follow_node',
            output='screen',
            parameters=[gap_follow_config,
                        {'enable_opponent_awareness': enabled == 'true',
                         'enable_steering_damping': damping,
                         'enable_adaptive_grip': grip,
                         'enable_imu_understeer': understeer,
                         'enable_imu_traction': traction,
                         'enable_imu_impact_stop': impact,
                         'enable_launch_boost': boost},
                        speed,
                        {'odom_topic': FUSED_ODOM} if fused else {}],
        )]
        if damping or grip or fused or understeer or traction or impact:
            # All IMU features share one estimator, but simulated IMUs
            # already come from gym_bridge_node. Start real readers only
            # when selected; any combination of features still includes once.
            hardware, reason = select_imu_hardware(
                LaunchConfiguration('imu_hardware').perform(context), probe=imu_probe)
            source = 'real IMU hardware' if hardware == 'true' else 'simulated IMU topics'
            actions.append(LogInfo(msg=f'IMU source: {source} ({reason}).'))
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('imu_bridge'), 'launch',
                             'vehicle_state_launch.py')),
                launch_arguments={'hardware': hardware}.items()))
        return actions

    return LaunchDescription([opponent_arg, max_speed_arg, imu_arg, imu_all_arg, steering_damping_arg,
                              adaptive_grip_arg, fused_speed_arg, understeer_arg,
                              traction_arg, impact_arg, imu_hardware_arg, launch_boost_arg,
                              OpaqueFunction(function=node)])
