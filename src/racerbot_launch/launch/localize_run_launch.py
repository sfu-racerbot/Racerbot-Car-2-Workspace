"""Particle-filter localization against a map saved by this run.

`particle_filter`'s own `localize_launch.py` cannot be used for this, for two
reasons that are both baked into it:

1. It points `map_server` at the map *packaged inside the particle_filter
   submodule* (`maps/levine.yaml`). The auto-map-race flow needs the map it
   just built, which lives in the run directory and does not exist until
   mid-run.
2. It hardcodes `use_sim_time: True` on `map_server` and the lifecycle
   manager. On the real car nothing publishes `/clock`, so a lifecycle node
   told to use simulated time is waiting on a clock that never ticks.

`particle_filter` is a git submodule (see docs/git-setup.md), so neither can
be fixed in place without the fix living only on this machine. This file is
the workspace's own copy of that launch, parameterised instead.

**The `/tf` remap is not optional.** `particle_filter` publishes a
`map -> laser` transform unconditionally -- it has no parameter to turn that
off. `slam_toolbox` is still running and still publishing `map -> odom`, and
`laser` already descends from `odom` via `base_link`. Two parents for one
frame is a broken TF tree, and the symptom is every consumer of TF
intermittently reading a pose from whichever parent won the race. So the
particle filter's transforms are sent to a dead topic and only its
*PoseStamped* output is consumed, by auto_map_race_node, which republishes it
on the topic pure pursuit already reads.

Usage (normally spawned by auto_map_race_node, not run by hand):

    ros2 launch racerbot_launch localize_run_launch.py \
        map_yaml:=/home/user/.ros/racerbot_auto/<run>/map.yaml
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Kept in step with race_launch.py's guard -- same library, same limits.
LIDAR_BEAMS = 1081
RANGE_LIBC_CHUNK_SIZE = 262144


def _check_gpu_ray_caster(params):
    """Refuse to start rather than crash or corrupt. See race_launch.py.

    Duplicated deliberately: this launch file is spawned mid-run, with the
    car stopped on track, and that is the worst possible moment to discover
    the GPU ray caster cannot work. Failing here means the run carries on
    with slam_toolbox instead of losing localization outright.
    """
    if params.get('range_method') != 'rmgpu':
        return
    try:
        import range_libc
    except ImportError as exc:
        raise RuntimeError(
            f'range_libc could not be imported ({exc}); build it with '
            'WITH_CUDA=ON -- see docs/gpu-acceleration.md') from exc
    if not range_libc.SHOULD_USE_CUDA:
        raise RuntimeError(
            "localize.yaml asks for range_method: 'rmgpu' but the installed "
            'range_libc was built WITHOUT CUDA. The particle filter would '
            'abort on its first scan. Rebuild it:\n'
            '    cd src/range_libc/pywrapper && '
            'WITH_CUDA=ON python3 setup.py install --user\n'
            'See docs/gpu-acceleration.md.')
    num_rays = math.ceil(LIDAR_BEAMS / int(params.get('angle_step', 1)))
    limit = RANGE_LIBC_CHUNK_SIZE // num_rays
    if int(params.get('max_particles', 0)) > limit:
        raise RuntimeError(
            f"max_particles={params.get('max_particles')} with "
            f'angle_step={params.get("angle_step")} ({num_rays} rays each) '
            f'exceeds the {limit}-particle limit above which range_libc\'s '
            'GPU ray caster returns silently wrong ranges. See '
            'docs/gpu-acceleration.md.')


def _nodes(context):
    map_yaml = LaunchConfiguration('map_yaml').perform(context)
    config = LaunchConfiguration('localize_config').perform(context)

    if not os.path.isfile(map_yaml):
        raise RuntimeError(
            f"map_yaml '{map_yaml}' does not exist. This launch localizes "
            'against an already-saved map; it does not build one.')

    params = yaml.safe_load(open(config, 'r'))['particle_filter']['ros__parameters']
    _check_gpu_ray_caster(params)

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    # Real car, real clock. See the module docstring.
                    {'use_sim_time': False}],
        # slam_toolbox already owns /map, and map_server publishing a second
        # one would give every /map subscriber two different maps at random.
        # The particle filter reaches this one through the map service, which
        # is remapped alongside it so the pairing cannot come apart.
        remappings=[('/map', '/pf/map'), ('/map_updates', '/pf/map_updates')],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pf_localization',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
    )
    particle_filter = Node(
        package='particle_filter',
        executable='particle_filter',
        name='particle_filter',
        parameters=[config],
        # Only /tf is remapped. The particle filter reaches the map through
        # the `/map_server/map` *service*, not the topic, and that service
        # name is left alone so the pairing with map_server above still
        # works -- remapping it here is what breaks this launch silently.
        # The filter is a TF broadcaster only, never a listener, so sending
        # its transforms nowhere costs it nothing.
        remappings=[('/tf', '/pf/tf'), ('/tf_static', '/pf/tf_static')],
    )
    return [lifecycle, map_server, particle_filter]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            description='map_server .yaml of the map to localize against. '
                        'Required -- normally the run directory\'s map.yaml.'),
        DeclareLaunchArgument(
            'localize_config',
            default_value=os.path.join(
                get_package_share_directory('particle_filter'),
                'config', 'localize.yaml'),
            description='particle_filter parameter file.'),
        OpaqueFunction(function=_nodes),
    ])
