"""The whole IMU side: BNO085 I2C bridge, imu_bridge, vehicle_state.

Written on car 1. On car 2 ``bno`` defaults to false: car 2 has no BNO085 yet
(docs/car1-vs-car2.md#imu). When it is mounted, decide how it is wired (car
1's Jetson-I2C bridge below, or a microcontroller on USB with
``bno_port``) before turning it on.

Support code: nothing here can move the car. Publishes /imu/* (imu_bridge),
/odom/fused and /steering/measured (vehicle_state). Run on top of bringup,
which provides /odom and the VESC's /sensors/imu/raw:

    ros2 launch imu_bridge vehicle_state_launch.py              # car 2: VESC IMU only
    ros2 launch imu_bridge vehicle_state_launch.py bno:=true    # BNO + VESC IMU (car 1)

publish_odom_tf:=true also publishes odom_imu -> odom (gyro odometry for
slam_toolbox). Only with slam_toolbox on odom_frame: odom_imu -- map_course
gyro_odom:=true sets both; the node refuses to keep publishing it if
anything else gives odom a parent.

Keep the car still for the first 3 s (imu_bridge's startup gyro bias).

Car 1's BNO085 is wired to the Jetson's own I2C bus, not to a
microcontroller, so imu-calibration's tools/bno_i2c_bridge.py turns it into
the serial stream imu_bridge reads, on a virtual port (bno_pty). Only one
process may own the sensor: if a bridge is already running (started by hand
for calibration, say) it is used as it is, not started twice.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BRIDGE_SCRIPT = 'bno_i2c_bridge.py'
# pgrep -f pattern (POSIX extended regex) for a *running* bridge: a Python
# interpreter executing the script. Bare 'bno_i2c_bridge.py' also matched an
# editor or `less` with the file open, and the launch then skipped the bridge.
BRIDGE_PATTERN = r'python[0-9.]* (-[^ ]+ )*[^ ]*bno_i2c_bridge\.py'


def bridge_running():
    """True when a bno_i2c_bridge.py process already owns the sensor."""
    return subprocess.run(['pgrep', '-f', BRIDGE_PATTERN],
                          capture_output=True).returncode == 0


def _actions(context, *args, **kwargs):
    hardware_value = LaunchConfiguration('hardware').perform(context).strip().lower()
    if hardware_value not in ('true', 'false'):
        raise RuntimeError('hardware must be true or false')
    hardware = hardware_value == 'true'
    share = get_package_share_directory('imu_bridge')
    root = os.path.expanduser(LaunchConfiguration('racer_imu_root').perform(context))
    bno = LaunchConfiguration('bno').perform(context).strip().lower() in ('true', '1')
    pty = LaunchConfiguration('bno_pty').perform(context)
    publish_tf = LaunchConfiguration('publish_odom_tf').perform(context).strip().lower() in (
        'true', '1')
    actions = []
    if not hardware:
        actions.append(LogInfo(msg='hardware:=false: starting only vehicle_state_node on simulated IMU topics.'))
    if hardware and bno:
        script = os.path.join(root, 'tools', BRIDGE_SCRIPT)
        if bridge_running():
            actions.append(LogInfo(msg=(
                f'{BRIDGE_SCRIPT} is already running: using it, not starting a '
                f'second one on the same I2C device. It must have been started with '
                f'--pty {pty} --no-dyncal.')))
        elif not os.path.isfile(script):
            actions.append(LogInfo(msg=(
                f'{script} not found: no BNO085. Clone imu-calibration to {root} '
                '(or pass racer_imu_root:=...). Steering uses the VESC gyro alone; '
                '/odom/fused is the wheel speed.')))
        else:
            # --no-dyncal: the BNO085's own accelerometer dynamic calibration
            # shifts offsets mid-run and broke the first calibration of this
            # sensor (imu-calibration calib/bno_calib_run1_dyncal_on.json).
            actions.append(ExecuteProcess(
                cmd=['python3', '-u', script, '--pty', pty, '--no-dyncal'],
                name='bno_i2c_bridge', output='screen'))
    if hardware:
        actions.append(Node(
            package='imu_bridge', executable='imu_bridge_node', name='imu_bridge_node',
            output='screen',
            parameters=[os.path.join(share, 'config', 'imu_bridge.yaml'),
                        {'racer_imu_root': root, 'bno_port': pty if bno else ''}],
        ))
    # Without the BNO there is no trusted forward accelerometer on car 1 (the
    # VESC's mount rotation is still car 2's): publish the wheel speed as is.
    overrides = {'publish_odom_tf': publish_tf}
    if hardware and not bno:
        overrides['accel_source'] = 'none'
    actions.append(Node(
        package='imu_bridge', executable='vehicle_state_node', name='vehicle_state_node',
        output='screen',
        parameters=[os.path.join(share, 'config', 'vehicle_state.yaml'), overrides],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware', default_value='true',
            description='true starts the real IMU stack; false uses simulated IMU topics.'),
        DeclareLaunchArgument(
            # Car 2: false, no BNO085 mounted yet (docs/car1-vs-car2.md#imu).
            'bno', default_value='false',
            description='Use the BNO085 (car 1: front axle, Jetson I2C). false = VESC '
                        'IMU only. Car 2 has no BNO085 yet.'),
        DeclareLaunchArgument(
            'racer_imu_root', default_value='~/imu-calibration',
            description='Where imu-calibration is cloned (car.yaml, calibration, '
                        'the I2C bridge).'),
        DeclareLaunchArgument(
            'bno_pty', default_value='/tmp/bno085',
            description="The I2C bridge's virtual serial port."),
        DeclareLaunchArgument(
            'publish_odom_tf', default_value='false',
            description='Publish odom_imu -> odom for slam_toolbox (gyro odometry). '
                        'Only with slam_toolbox on odom_frame: odom_imu.'),
        OpaqueFunction(function=_actions),
    ])
