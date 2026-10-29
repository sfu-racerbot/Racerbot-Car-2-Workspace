"""map_course_launch.py's IMU wiring: imu:= and fused_speed:=.

Needs ROS sourced and the workspace built (it loads the real launch file and
evaluates its node-building step with launch's own LaunchContext):

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest src/racerbot_launch/test/test_map_course_imu.py -v

Also run by `colcon test --packages-select racerbot_launch`. Nothing is
started: no process, no ROS graph.

Oracles: vehicle_state.yaml's fused_odom_topic and gap_follow.yaml's
odom_topic (cited files), and the launch's own documented defaults
(fused_speed off: gap_follow keeps /odom unless asked).
"""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
import pytest
import yaml

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'map_course_launch.py'
SRC = Path(__file__).resolve().parents[2]


def _load():
    spec = importlib.util.spec_from_file_location('map_course_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ml = _load()


def _node_params(node, ctx):
    """A launch_ros Node's dict parameters, evaluated in ctx. launch_ros keeps
    them private (name-mangled): strings as YAML-encoded substitutions, other
    values as they are. A change there fails loudly rather than passing."""
    params = {}
    for entry in node._Node__parameters:
        if isinstance(entry, dict):
            for key, value in entry.items():
                name = ''.join(x.perform(ctx) for x in key)
                if isinstance(value, tuple) and all(hasattr(x, 'perform') for x in value):
                    value = yaml.safe_load(''.join(x.perform(ctx) for x in value))
                params[name] = value
    return params


def _context(ld, **overrides):
    ctx = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = ''.join(s.perform(ctx) for s in entity.default_value or [])
            ctx.launch_configurations[entity.name] = overrides.get(entity.name, default)
    return ctx


def _opaque(ld, function):
    """The launch's OpaqueFunction running `function` (None: the one that
    isn't _wiring, i.e. the node-building step)."""
    found = [e for e in ld.entities if isinstance(e, OpaqueFunction)
             and ((e._OpaqueFunction__function is function) if function is not None
                  else (e._OpaqueFunction__function is not ml._wiring))]
    assert len(found) == 1
    return found[0]


def _wired_context(**overrides):
    """A context after map_course's _wiring step has run, as launch runs it
    (before the SLAM and IMU includes)."""
    ld = ml.generate_launch_description()
    ctx = _context(ld, **overrides)
    for action in _opaque(ld, ml._wiring).execute(ctx):
        action.execute(ctx)
    return ld, ctx


def _gap_follow_params(**overrides):
    """The dict parameters the real launch hands gap_follow_node."""
    ld = ml.generate_launch_description()
    ctx = _context(ld, **overrides)
    nodes = [a for a in _opaque(ld, None).execute(ctx) if isinstance(a, Node)]
    gap = [n for n in nodes if n.node_package == 'gap_follow']
    assert len(gap) == 1
    return _node_params(gap[0], ctx)


def test_defaults_are_imu_off_on_car2_and_fused_speed_off():
    # oracle: spec -- docs/car1-vs-car2.md#imu: car 2 has no BNO085 and no
    # ~/imu-calibration yet, so the IMU stack is off by default here (car 1
    # ships imu:=true); driving on it only when asked, on either car.
    ld = ml.generate_launch_description()
    defaults = {e.name: ''.join(s.perform(LaunchContext()) for s in e.default_value)
                for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert defaults['imu'] == 'false'
    assert defaults['fused_speed'] == 'false'
    assert defaults['gyro_odom'] == 'false'
    # Every run is replayable through SLAM offline (docs/launchstart.md).
    assert defaults['record_bag'] == 'true'


def test_by_default_gap_follow_keeps_wheel_odometry():
    params = _gap_follow_params()
    assert 'odom_topic' not in params
    assert params['drive_topic'] == '/auto_map/drive'
    # oracle: spec -- gap_follow.yaml: the topic it falls back to is /odom.
    gap_yaml = yaml.safe_load(
        (Path(get_package_share_directory('gap_follow')) / 'config' / 'gap_follow.yaml')
        .read_text())
    assert gap_yaml['gap_follow_node']['ros__parameters']['odom_topic'] == '/odom'


def test_fused_speed_points_gap_follow_at_what_vehicle_state_publishes():
    params = _gap_follow_params(imu='true', fused_speed='true')
    # oracle: spec -- vehicle_state.yaml's fused_odom_topic, the topic
    # actually published; a typo on either side would leave gap_follow
    # listening to nothing.
    vs_yaml = yaml.safe_load(
        (SRC / 'imu_bridge' / 'config' / 'vehicle_state.yaml').read_text())
    fused = vs_yaml['vehicle_state_node']['ros__parameters']['fused_odom_topic']
    assert params['odom_topic'] == fused
    assert params['drive_topic'] == '/auto_map/drive'     # still the supervisor's input


def test_fused_speed_without_the_imu_stack_refuses_to_start():
    with pytest.raises(RuntimeError, match='needs imu:=true'):
        _gap_follow_params(imu='false', fused_speed='true')


@pytest.mark.parametrize('imu,fused,expected', [
    ('true', 'false', {}), ('false', 'false', {}), ('True', '0', {}),
    ('true', 'true', {'odom_topic': '/odom/fused'}),
    ('1', 'TRUE', {'odom_topic': '/odom/fused'}),
])
def test_odom_overrides(imu, fused, expected):
    assert ml.odom_overrides(imu, fused) == expected


@pytest.mark.parametrize('imu,fused', [('true', 'yes'), ('true', ''), ('maybe', 'true')])
def test_odom_overrides_refuses_values_launch_would_not_read_as_a_boolean(imu, fused):
    with pytest.raises(RuntimeError, match="expected 'true' or 'false'"):
        ml.odom_overrides(imu, fused)


@pytest.mark.parametrize('imu,runs', [('true', True), ('false', False)])
def test_imu_stack_is_included_only_with_imu_true(imu, runs):
    ld = ml.generate_launch_description()
    ctx = _context(ld, imu=imu)
    includes = []
    for e in ld.entities:
        if isinstance(e, IncludeLaunchDescription):
            e.launch_description_source.get_launch_description(ctx)   # resolves location
            if e.launch_description_source.location.endswith('/vehicle_state_launch.py'):
                includes.append(e)
    assert len(includes) == 1
    assert includes[0].condition.evaluate(ctx) is runs


def test_car2_default_run_does_not_start_the_imu_stack():
    # oracle: spec -- docs/car1-vs-car2.md#imu: with no override, car 2's
    # map_course must not include vehicle_state_launch.py (imu_bridge_node
    # raises without ~/imu-calibration, and there is no BNO085 to read).
    ld = ml.generate_launch_description()
    ctx = _context(ld)
    includes = []
    for e in ld.entities:
        if isinstance(e, IncludeLaunchDescription):
            e.launch_description_source.get_launch_description(ctx)
            if e.launch_description_source.location.endswith('/vehicle_state_launch.py'):
                includes.append(e)
    assert len(includes) == 1
    assert includes[0].condition.evaluate(ctx) is False


def test_car2_default_vehicle_state_launch_starts_no_bno_bridge():
    # oracle: spec -- docs/car1-vs-car2.md#imu: car 2 has no BNO085, so
    # vehicle_state_launch.py's default must neither start car 1's I2C bridge
    # nor hand imu_bridge_node a BNO port.
    spec = importlib.util.spec_from_file_location(
        'vehicle_state_launch', SRC / 'imu_bridge' / 'launch' / 'vehicle_state_launch.py')
    vl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vl)
    ld = vl.generate_launch_description()
    ctx = _context(ld)
    actions = [e for e in ld.entities if isinstance(e, OpaqueFunction)][0].execute(ctx)
    # Node subclasses ExecuteProcess: only a bare process is the I2C bridge.
    assert not [a for a in actions if isinstance(a, ExecuteProcess) and not isinstance(a, Node)]
    bridge = [a for a in actions if isinstance(a, Node) and a.node_executable == 'imu_bridge_node']
    assert len(bridge) == 1
    assert _node_params(bridge[0], ctx)['bno_port'] == ''


# ---------------------------------------------------------------- gyro_odom

@pytest.mark.parametrize('gyro_odom,frame,publish', [
    ('false', 'odom', 'false'), ('true', 'odom_imu', 'true'), ('1', 'odom_imu', 'true')])
def test_gyro_odom_moves_slam_and_the_transform_together(gyro_odom, frame, publish):
    _, ctx = _wired_context(imu='true', gyro_odom=gyro_odom)
    assert ctx.launch_configurations['slam_odom_frame'] == frame
    assert ctx.launch_configurations['publish_odom_tf'] == publish


def test_gyro_odom_without_the_imu_stack_refuses_to_start():
    with pytest.raises(RuntimeError, match='gyro_odom:=true needs imu:=true'):
        _wired_context(imu='false', gyro_odom='true')


def test_gyro_odom_frame_is_the_one_vehicle_state_publishes():
    # oracle: spec -- vehicle_state.yaml's imu_odom_frame: SLAM pointed at a
    # frame nobody publishes would have no odometry at all.
    vs_yaml = yaml.safe_load(
        (SRC / 'imu_bridge' / 'config' / 'vehicle_state.yaml').read_text())
    assert ml.IMU_ODOM_FRAME == vs_yaml['vehicle_state_node']['ros__parameters']['imu_odom_frame']
    assert ml.gyro_odom_settings('true', 'true') == (ml.IMU_ODOM_FRAME, 'true')


@pytest.mark.parametrize('frame', ['odom', 'odom_imu'])
def test_slam_toolbox_gets_the_wired_odom_frame(frame):
    # The real slam_launch.py, evaluated: its node's odom_frame parameter is
    # whatever _wiring set (and plain `odom` when nothing did).
    spec = importlib.util.spec_from_file_location(
        'slam_launch', LAUNCH.parent / 'slam_launch.py')
    sl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sl)
    ld = sl.generate_launch_description()
    ctx = _context(ld, slam_odom_frame=frame)
    slam = [e for e in ld.entities if isinstance(e, Node)]
    assert len(slam) == 1
    assert _node_params(slam[0], ctx)['odom_frame'] == frame
    defaults = {e.name: ''.join(x.perform(LaunchContext()) for x in e.default_value)
                for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert defaults['slam_odom_frame'] == 'odom'


def test_wiring_runs_before_the_includes_that_read_it():
    # Launch runs entities in order: the includes must come after _wiring,
    # or slam_launch.py would see no slam_odom_frame and fall back to odom.
    ld = ml.generate_launch_description()
    kinds = [type(e).__name__ for e in ld.entities if not isinstance(e, DeclareLaunchArgument)]
    first_include = kinds.index('IncludeLaunchDescription')
    wiring = [i for i, e in enumerate(
        e for e in ld.entities if not isinstance(e, DeclareLaunchArgument))
        if isinstance(e, OpaqueFunction) and e._OpaqueFunction__function is ml._wiring]
    assert wiring and wiring[0] < first_include


@pytest.mark.parametrize('publish,expected', [('true', True), ('false', False), ('1', True)])
def test_vehicle_state_launch_hands_publish_odom_tf_to_the_node(publish, expected):
    # The real vehicle_state_launch.py, evaluated with what _wiring set.
    spec = importlib.util.spec_from_file_location(
        'vehicle_state_launch', SRC / 'imu_bridge' / 'launch' / 'vehicle_state_launch.py')
    vl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vl)
    ld = vl.generate_launch_description()
    ctx = _context(ld, publish_odom_tf=publish)
    actions = [e for e in ld.entities if isinstance(e, OpaqueFunction)][0].execute(ctx)
    node = [a for a in actions if isinstance(a, Node) and a.node_executable == 'vehicle_state_node']
    assert len(node) == 1
    assert _node_params(node[0], ctx)['publish_odom_tf'] is expected


# ------------------------------------------------------------ SLAM profile

SLAM_GYRO = SRC / 'racerbot_launch' / 'config' / 'slam_gyro.yaml'


def test_gyro_odom_loads_the_gyro_slam_profile():
    # oracle: spec -- A1: the profile is loaded with gyro_odom:=true, from the
    # installed package, where slam_launch.py will read it.
    # imu:=true stated explicitly: gyro_odom needs it, and car 2's map_course
    # defaults to imu:=false (docs/car1-vs-car2.md#imu); car 1's defaults to true.
    _, ctx = _wired_context(imu='true', gyro_odom='true')
    profile = ctx.launch_configurations['slam_profile_file']
    assert profile.endswith('/slam_gyro.yaml')
    assert Path(profile).is_file()


def test_without_gyro_odom_no_profile_is_set():
    # oracle: spec -- A1: runs without the gyro keep slam_tracking.yaml's search.
    _, ctx = _wired_context(gyro_odom='false')
    assert 'slam_profile_file' not in ctx.launch_configurations


def test_slam_launch_layers_the_profile_last():
    # oracle: spec -- ROS 2 parameter files: later files win (slam_launch.py,
    # "Order matters"). So: tracking file, then profile, then the dict.
    spec = importlib.util.spec_from_file_location(
        'slam_launch', LAUNCH.parent / 'slam_launch.py')
    sl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sl)
    ld = sl.generate_launch_description()
    ctx = _context(ld, slam_profile_file='/profile/under/test.yaml')
    slam = [e for e in ld.entities if isinstance(e, Node)]
    assert len(slam) == 1
    entries = slam[0]._Node__parameters
    files = [''.join(s.perform(ctx) for s in e.param_file)
             for e in entries if isinstance(e, ParameterFile)]
    assert files == [ctx.launch_configurations['slam_params_file'],
                     ctx.launch_configurations['slam_tracking_file'],
                     '/profile/under/test.yaml']
    assert isinstance(entries[-1], dict)


def test_gyro_profile_window_cannot_reach_the_neighbouring_track():
    # oracle: measured docs/superpowers/specs/2026-10-26-slam-and-gap-follow-plan.md
    # (A1): on runs 20261026-194714 and -194913 the false loop-closure jumps were
    # at least 2.2 m. The correlation window is +/- loop_search_space_dimension / 2
    # (A1: the 8.0 m search is +/-4 m), so it must stay short of 2.2 m.
    profile = yaml.safe_load(SLAM_GYRO.read_text())['slam_toolbox']['ros__parameters']
    assert profile['loop_search_space_dimension'] / 2 < 2.2
