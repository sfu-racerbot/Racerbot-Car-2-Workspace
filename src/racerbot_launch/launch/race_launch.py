import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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
    (docs/architecture.md's safety model). See docs/operations.md and
    docs/racing-autonomy.md for the full procedure, including giving the
    particle filter its "2D Pose Estimate" seed in RViz before the car
    will go anywhere.

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
    default_waypoints_file = pp_config['pure_pursuit_node']['ros__parameters'].get('waypoints_file', '')

    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints_file,
        description='Profiled (x,y,speed) waypoints CSV for this track.'
    )

    # Same trick for particle_filter's config, but to check it rather than to
    # read a default out of it: a bad GPU ray-caster setting fails here, at
    # launch time, with an explanation -- instead of on the first scan, as a
    # core dump or as silently wrong ranges. See _check_gpu_ray_caster.
    pf_config_path = os.path.join(
        get_package_share_directory('particle_filter'), 'config', 'localize.yaml')
    pf_config = yaml.safe_load(open(pf_config_path, 'r'))
    _check_gpu_ray_caster(pf_config['particle_filter']['ros__parameters'])

    localize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('particle_filter'), 'launch', 'localize_launch.py')
        )
    )

    pure_pursuit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pure_pursuit'), 'launch', 'pure_pursuit_launch.py')
        ),
        launch_arguments={'waypoints_file': LaunchConfiguration('waypoints_file')}.items(),
    )

    return LaunchDescription([waypoints_arg, localize_launch, pure_pursuit_launch])
