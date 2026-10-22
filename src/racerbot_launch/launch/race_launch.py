import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pure_pursuit import run_commands

# The car's Hokuyo UST-10LX publishes 1081 beams (0.25 deg over 270 deg) --
# same constant racerbot_sim uses, see sim_bridge.py's LIDAR_BEAMS.
LIDAR_BEAMS = 1081

# range_libc's GPU ray caster allocates its device buffers to exactly
# CHUNK_SIZE floats at construction (kernels.cu's cudaMalloc calls), and
# CHUNK_SIZE is fixed at compile time in range_libc/pywrapper/setup.py.
RANGE_LIBC_CHUNK_SIZE = 262144


def _check_gpu_ray_caster(pf_params):
    """Refuse to launch if the particle filter's GPU settings would fail.

    Two ways `range_method: 'rmgpu'` goes wrong, both of them quiet:

    1. **range_libc built without CUDA.** `PyRayMarchingGPU.__cinit__` prints
       a warning and then returns *without constructing anything*, leaving a
       null pointer behind. The config is accepted, the node starts, and the
       first scan kills the process with a core dump -- taking the car's only
       source of map position with it. Rebuild with:

           cd src/range_libc/pywrapper && WITH_CUDA=ON python3 setup.py install --user

    2. **Too many particles.** `numpy_calc_range_angles` in RangeLib.h splits
       the work into chunks of `ceil(CHUNK_SIZE / num_rays)` particles -- and
       `ceil` makes that chunk *bigger* than the CHUNK_SIZE-sized device
       buffer whenever num_rays doesn't divide CHUNK_SIZE evenly. The copy
       fails, and rather than raising, the call leaves the output buffer
       untouched: measured on this car, one particle over the limit returns
       100% wrong ranges with nothing but a line on stdout to say so. Silent
       garbage into localization is worse than a crash, so it is checked here.

    Staying inside one chunk also sidesteps a second defect in the same loop
    (the input offset uses `num_in_chunk` where it means `particles_per_iter`,
    so the last chunk reads from the wrong place). Both are upstream bugs in
    f1tenth/range_libc, not something this workspace introduced.
    """
    if pf_params.get('range_method') != 'rmgpu':
        return

    try:
        import range_libc
    except ImportError as exc:  # pragma: no cover - depends on the machine
        raise RuntimeError(
            "particle_filter's localize.yaml asks for range_method: 'rmgpu', "
            f"but range_libc could not be imported ({exc}). Build it with:\n"
            '    cd src/range_libc/pywrapper && '
            'WITH_CUDA=ON python3 setup.py install --user'
        ) from exc

    if not range_libc.SHOULD_USE_CUDA:
        raise RuntimeError(
            "particle_filter's localize.yaml asks for range_method: 'rmgpu', "
            'but the installed range_libc was built WITHOUT CUDA '
            '(range_libc.SHOULD_USE_CUDA is False). The particle filter would '
            'start and then abort on its first scan. Rebuild it with:\n'
            '    cd src/range_libc/pywrapper && '
            'WITH_CUDA=ON python3 setup.py install --user\n'
            "...or set range_method to 'rm' or 'pcddt' to stay on the CPU."
        )

    angle_step = int(pf_params.get('angle_step', 1))
    max_particles = int(pf_params.get('max_particles', 0))
    num_rays = math.ceil(LIDAR_BEAMS / angle_step)
    particle_limit = RANGE_LIBC_CHUNK_SIZE // num_rays

    if max_particles > particle_limit:
        raise RuntimeError(
            f'particle_filter is configured for max_particles={max_particles} '
            f'with angle_step={angle_step} ({num_rays} rays per particle), which '
            f'is {max_particles * num_rays} ray queries per scan. '
            f"range_libc's GPU ray caster silently returns wrong ranges above "
            f'{RANGE_LIBC_CHUNK_SIZE} queries, so the limit here is '
            f'{particle_limit} particles. Either lower max_particles to '
            f'{particle_limit} or less, raise angle_step (fewer rays), or raise '
            "CHUNK_SIZE in range_libc/pywrapper/setup.py and rebuild."
        )


def generate_launch_description():
    """Race-day launch: localization (particle filter against a saved map)
    plus the pure-pursuit race controller, together.

    This is a control layer, same as teleop_launch.py or gap_follow_launch.py
    -- run it in its own terminal, on top of f1tenth_stack's bringup_launch.py
    (already up: VESC/LiDAR/mux), and don't also launch teleop_launch.py at
    the same time or its always-on /teleop will mask /drive at the mux
    (docs/architecture.md's safety model).

    Two ways to say which track:

    * ``run_dir:=<dir>`` -- a run directory written by a mapping run
      (``ros2 run racerbot_launch map_course``, or auto_map_race_launch.py).
      Uses that run's map.yaml and raceline_profiled.csv, and seeds the
      particle filter at the run's start_pose.yaml -- where the car stopped
      when mapping ended (the map origin for runs without one) -- so no RViz
      click is needed. Leave the car there, or put it back on that spot
      facing the same way, before launching. This is what map_course prints.

          ros2 launch racerbot_launch race_launch.py \
              run_dir:=$HOME/.ros/racerbot_auto/20261022-101500 steering_law:=stanley

    * Neither: the map named in particle_filter's localize.yaml and
      ``waypoints_file`` (default: pure_pursuit.yaml's), seeded by hand with
      RViz's "2D Pose Estimate" -- see docs/operations.md.

          ros2 launch racerbot_launch race_launch.py \
              waypoints_file:=/path/to/track_profiled.csv
    """
    # Read pure_pursuit's own config at generation time (same pattern
    # particle_filter's localize_launch.py and pure_pursuit_launch.py
    # both already use) so that *not* passing waypoints_file here falls
    # through to whatever pure_pursuit's own config/pure_pursuit.yaml
    # says, instead of silently overriding it with an empty string.
    pp_config_path = os.path.join(
        get_package_share_directory('pure_pursuit'), 'config', 'pure_pursuit.yaml')
    pp_config = yaml.safe_load(open(pp_config_path, 'r'))
    pp_params = pp_config['pure_pursuit_node']['ros__parameters']
    default_waypoints_file = pp_params.get('waypoints_file', '')
    default_steering_law = pp_params.get('steering_law', 'pure_pursuit')

    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints_file,
        description='Profiled (x,y,speed) waypoints CSV for this track. '
                    'Ignored when run_dir is given.'
    )
    run_dir_arg = DeclareLaunchArgument(
        'run_dir', default_value='',
        description='A mapping run\'s directory (map.yaml + raceline_profiled.csv). '
                    'Empty: use localize.yaml\'s map, waypoints_file and an RViz seed.')
    steering_law_arg = DeclareLaunchArgument(
        'steering_law', default_value=default_steering_law,
        description='pure_pursuit (default), stanley, lqr or mpc.')
    seed_arg = DeclareLaunchArgument(
        'seed_start', default_value='true',
        description='With run_dir: seed the particle filter at the mapping '
                    'end-of-mapping stop (start_pose.yaml). false: seed it yourself.')

    # Same trick for particle_filter's config, but to check it rather than to
    # read a default out of it: a bad GPU ray-caster setting fails here, at
    # launch time, with an explanation -- instead of on the first scan, as a
    # core dump or as silently wrong ranges. See _check_gpu_ray_caster.
    pf_config_path = os.path.join(
        get_package_share_directory('particle_filter'), 'config', 'localize.yaml')
    pf_config = yaml.safe_load(open(pf_config_path, 'r'))
    _check_gpu_ray_caster(pf_config['particle_filter']['ros__parameters'])

    def layers(context):
        run_dir = os.path.expanduser(LaunchConfiguration('run_dir').perform(context))
        steering_law = LaunchConfiguration('steering_law').perform(context)
        seeder = None
        if not run_dir:
            localize = [IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('particle_filter'),
                             'launch', 'localize_launch.py')))]
            waypoints = LaunchConfiguration('waypoints_file').perform(context)
        else:
            map_yaml = os.path.join(run_dir, 'map.yaml')
            waypoints = os.path.join(run_dir, 'raceline_profiled.csv')
            missing = [path for path in (map_yaml, waypoints) if not os.path.isfile(path)]
            if missing:
                raise RuntimeError(
                    f"run_dir '{run_dir}' is missing {', '.join(missing)}. Pass "
                    'the directory a mapping run printed (ls ~/.ros/racerbot_auto/).')
            localize = [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('racerbot_launch'),
                    'launch', 'localize_run_launch.py')),
                launch_arguments={'map_yaml': map_yaml, 'standalone': 'true'}.items())]
            if LaunchConfiguration('seed_start').perform(context).lower() in ('true', '1', 'yes'):
                # The start pose map_course recorded; runs from before it
                # existed (or auto_map_race_launch.py's) fall back to the map
                # origin, where odometry read zero when SLAM started.
                start = run_commands.read_start_pose(run_dir) or (0.0, 0.0, 0.0)
                seeder = Node(
                    package='pure_pursuit', executable='seed_start_pose',
                    name='seed_start_pose', output='screen',
                    parameters=[{'x': start[0], 'y': start[1], 'yaw': start[2],
                                 'laser_offset_x': float(pp_params.get('laser_offset_x', 0.26)),
                                 'laser_offset_y': float(pp_params.get('laser_offset_y', 0.0))}])
        if not run_dir:
            pure_pursuit = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('pure_pursuit'), 'launch',
                    'pure_pursuit_launch.py')),
                launch_arguments={'waypoints_file': waypoints,
                                  'steering_law': steering_law}.items(),
            )
        else:
            # Saved-map race: the map is a fixed file and no SLAM runs, so
            # opponent_detection_settle_sec -- which waits out slam_toolbox's
            # live map lagging the pose by up to 5 s -- has nothing to wait
            # for. Left on, the first 6 s treat every wall as an obstacle; in
            # simulation that boxed the car in (or onto a wall) right at the
            # start, which is where mapping leaves it: beside a wall, at the
            # racing line's seam.
            pure_pursuit = Node(
                package='pure_pursuit', executable='pure_pursuit_node',
                name='pure_pursuit_node', output='screen',
                parameters=[pp_config_path, {
                    'waypoints_file': waypoints, 'steering_law': steering_law,
                    'opponent_detection_settle_sec': 0.0}])
        if seeder is None:
            return localize + [pure_pursuit]

        # pure_pursuit starts only once the filter has taken the seed (the
        # seeder exits 0). Before that the filter publishes its own unseeded
        # guess; a controller driving on it steered into a wall in simulation.
        def after_seed(event, _context):
            if event.returncode == 0:
                return [pure_pursuit]
            return [LogInfo(msg='Localization never took the start pose; NOT starting '
                                'pure_pursuit. See the seed_start_pose error above.'),
                    EmitEvent(event=Shutdown(reason='start pose not confirmed'))]

        return localize + [seeder, RegisterEventHandler(
            OnProcessExit(target_action=seeder, on_exit=after_seed))]

    return LaunchDescription([waypoints_arg, run_dir_arg, steering_law_arg, seed_arg,
                              OpaqueFunction(function=layers)])
