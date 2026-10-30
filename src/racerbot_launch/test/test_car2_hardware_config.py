"""
Car 2's hardware differences from car 1 survive a port from the car 1 repo.

The two cars share almost all code, and car 1's workspace is ported into
this one by hand (docs/car1-vs-car2.md). A port that copies car 1's files
over car 2's fails *silently*: urg_node starts and waits on a USB serial
port car 2 does not have, the doctor reports a webcam that was never
fitted, the steering hits car 1's limits, and nothing errors at build time.
This file makes those mistakes fail in pytest instead of on the car.

Mirror of car 1's test_car1_hardware_config.py.

Spec (the oracle for every assertion): docs/car1-vs-car2.md, which cites
docs/hardware-reference.md. Where two files must agree, the test reads both
and compares them, so it checks consistency rather than memorising a value.

No ROS needed:

    python3 -m pytest src/racerbot_launch/test/ -v
"""
import glob
import importlib.util
import os
import re

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_HERE))
_STACK = os.path.join(_SRC, 'f1tenth_system', 'f1tenth_stack')
_RB2 = os.path.join(_SRC, 'racerbot_launch', 'config', 'web_dashboard_rb2.yaml')
_DOCTOR_CAR = os.path.join(_SRC, 'racerbot_doctor', 'racerbot_doctor', 'car.py')
_HW_DOC = os.path.join(_SRC, '..', 'docs', 'hardware-reference.md')
_IMU_BRIDGE = os.path.join(_SRC, 'imu_bridge')


def _yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _read(*parts):
    with open(os.path.join(*parts)) as f:
        return f.read()


def _doctor_car():
    spec = importlib.util.spec_from_file_location('doctor_car', _DOCTOR_CAR)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _urg_params():
    return _yaml(os.path.join(_STACK, 'config', 'sensors.yaml'))['urg_node'][
        'ros__parameters']


def _doc_match(pattern):
    m = re.search(pattern, _read(_HW_DOC))
    assert m, f'hardware-reference.md no longer matches {pattern!r}'
    return m


def _doc_number(pattern):
    return float(_doc_match(pattern).group(1))


# ------------------------------------------------------------------ LiDAR

def test_urg_node_uses_the_ethernet_hokuyo_not_usb():
    # Car 2: Ethernet UST-10LX. Car 1: USB UST-30LX at /dev/sensors/hokuyo.
    # urg_node opens serial_port when ip_address is absent, so car 1's
    # sensors.yaml would leave car 2 with no /scan.
    ip, port = _doc_match(r"Sensor's IP:port \| `([0-9.]+):([0-9]+)`").groups()
    params = _urg_params()
    assert params.get('ip_address') == ip
    assert params.get('ip_port') == int(port)
    assert 'serial_port' not in params


def test_doctor_checks_the_ethernet_lidar_the_doc_describes():
    car = _doctor_car()
    ip, port = _doc_match(r"Sensor's IP:port \| `([0-9.]+):([0-9]+)`").groups()
    host = _doc_match(r'NetworkManager profile \| `hokuyo`, static IPv4 `([0-9./]+)`').group(1)
    assert (car.LIDAR_IP, car.LIDAR_PORT, car.LIDAR_HOST_IP) == (ip, int(port), host)
    assert car.LIDAR_IP == _urg_params()['ip_address']


def test_doctor_has_no_usb_lidar_settings():
    car = _doctor_car()
    for car1_only in ('LIDAR_USB', 'LIDAR_SYMLINK', 'LIDAR_KERNEL_DRIVER'):
        assert not hasattr(car, car1_only), car1_only


# ----------------------------------------------------------------- camera

def test_doctor_checks_the_realsense_the_dashboard_streams():
    # Car 2's camera is a RealSense D435i; the dashboard streams its ROS
    # image topic. Car 1's doctor looks for a UVC webcam at /dev/video0.
    car = _doctor_car()
    image_topic = _yaml(_RB2)['usb_cam_stream_node']['ros__parameters']['image_topic']
    assert car.CAMERA_TOPIC == image_topic
    assert car.EXE_REALSENSE == 'realsense2_camera_node'
    assert not hasattr(car, 'CAMERA_DEVICE')


def test_realsense_launch_and_driver_are_still_here():
    launch = _read(_SRC, 'racerbot_launch', 'launch', 'realsense_camera_launch.py')
    assert 'realsense2_camera' in launch
    assert os.path.isdir(os.path.join(_SRC, 'realsense-ros'))
    assert '<exec_depend>realsense2_camera</exec_depend>' in _read(
        _SRC, 'racerbot_launch', 'package.xml')


def test_no_car_1_webcam_launch():
    # Car 1's webcam_launch.py streams /dev/video0, which on car 2 is one of
    # the RealSense's own nodes (realsense2_camera_node holds it exclusively).
    assert not os.path.exists(
        os.path.join(_SRC, 'racerbot_launch', 'launch', 'webcam_launch.py'))


# -------------------------------------------------------- dashboard site id

def test_dashboard_launch_loads_car_2s_settings():
    # Car 2 is dashboard id rb2; a ported car 1 launch file would load rb1.
    configs = set(re.findall(r"'(web_dashboard_rb\d+\.yaml)'", _read(
        _SRC, 'racerbot_launch', 'launch', 'dashboard_launch.py')))
    assert configs == {'web_dashboard_rb2.yaml'}
    assert glob.glob(os.path.join(_SRC, 'racerbot_launch', 'config',
                                  'web_dashboard_rb*.yaml')) == [_RB2]


# --------------------------------------------------------------- Steering

def test_vesc_steering_calibration_is_car_2s_measured_one():
    # A port that takes car 1's vesc.yaml (its own circles, its own servo)
    # breaks this. Oracle: the vesc.yaml block quoted in hardware-reference.md.
    shared = _yaml(os.path.join(_STACK, 'config', 'vesc.yaml'))['/**'][
        'ros__parameters']
    for key in ('steering_angle_to_servo_gain', 'steering_angle_to_servo_offset',
                'servo_min', 'servo_max'):
        documented = _doc_number(r'(?m)^' + key + r':\s*(-?[0-9.]+)')
        assert shared[key] == documented, key


def test_full_stick_and_autonomy_limits_are_car_2s():
    # hardware-reference.md's steering bullets are car 2's numbers; car 1's
    # 0.38 full stick or 0.33 limit fail here.
    joy = _yaml(os.path.join(_STACK, 'config', 'joy_teleop.yaml'))['joy_teleop'][
        'ros__parameters']
    scale = joy['human_control']['axis_mappings']['drive-steering_angle']['scale']
    assert scale == _doc_number(r'Full stick \(`joy_teleop\.yaml` scale\) is \*\*([0-9.]+) rad')
    controller = _doc_number(r'`max_steering_angle` \*\*([0-9.]+) rad')
    for pkg, cfg, node in (('gap_follow', 'gap_follow.yaml', 'gap_follow_node'),
                           ('pure_pursuit', 'pure_pursuit.yaml', 'pure_pursuit_node')):
        params = _yaml(os.path.join(_SRC, pkg, 'config', cfg))[node]['ros__parameters']
        assert params['max_steering_angle'] == controller, cfg
    planner = _yaml(os.path.join(_SRC, 'pure_pursuit', 'config', 'auto_map_race.yaml'))[
        'auto_map_race_node']['ros__parameters']['profile_max_steering_angle']
    assert planner == _doc_number(r'`profile_max_steering_angle` is \*\*([0-9.]+) rad')


def test_odometry_gain_is_car_2s():
    # The sign is the fix (docs/git-setup.md), the magnitude car 2's own
    # calibration; car 1's is -4040.0. Oracle: git-setup.md's verify line.
    vesc = _yaml(os.path.join(_STACK, 'config', 'vesc.yaml'))
    gain = vesc['vesc_to_odom_node']['ros__parameters']['speed_to_erpm_gain']
    m = re.search(r'vesc_to_odom_node speed_to_erpm_gain\s+# must be NEGATIVE '
                  r'\(currently (-[0-9.]+)\)', _read(_SRC, '..', 'docs', 'git-setup.md'))
    assert m, 'git-setup.md no longer quotes the odometry gain'
    assert gain == float(m.group(1))
    assert gain < 0


# -------------------------------------------------------------------- IMU

def test_map_course_does_not_run_the_imu_stack_by_default():
    # Car 2 has no BNO085 yet and no ~/imu-calibration, without which
    # imu_bridge_node raises at startup. Car 1 ships imu:=true.
    launch = _read(_SRC, 'racerbot_launch', 'launch', 'map_course_launch.py')
    defaults = re.findall(r"'imu', default_value='(\w+)'", launch)
    assert defaults == ['false']


def test_vehicle_state_launch_does_not_use_a_bno_by_default():
    # Car 1's default starts its Jetson-I2C BNO bridge; car 2 has no BNO.
    launch = _read(_IMU_BRIDGE, 'launch', 'vehicle_state_launch.py')
    defaults = re.findall(r"'bno', default_value='(\w+)'", launch)
    assert defaults == ['false']


def test_gap_follow_steering_damping_is_off_by_default():
    # The gyro term needs the IMU stack, which needs the BNO085 and
    # ~/imu-calibration that car 2 does not have yet (docs/car1-vs-car2.md#imu).
    # Car 1 ships both true since its 2026-10-27 race defaults. Flip these two
    # with the IMU tests above, in the same commit.
    params = _yaml(os.path.join(_SRC, 'gap_follow', 'config', 'gap_follow.yaml'))
    assert params['gap_follow_node']['ros__parameters']['enable_steering_damping'] is False
    launch = _read(_SRC, 'gap_follow', 'launch', 'gap_follow_launch.py')
    defaults = re.findall(r"'steering_damping', default_value='(\w+)'", launch)
    assert defaults == ['false']


def test_gap_follow_imu_features_ship_off_on_car2():
    # Ported from car 1's Part B-2/3 (PR #8): the understeer, traction and
    # impact thresholds are car 1's measurements, and car 2's BNO085 is not
    # calibrated (docs/car1-vs-car2.md#imu-feature-thresholds). Every one of
    # them, and imu_all that would turn them all on, must default off.
    params = _yaml(os.path.join(_SRC, 'gap_follow', 'config', 'gap_follow.yaml'))
    p = params['gap_follow_node']['ros__parameters']
    for name in ('enable_imu_understeer', 'enable_imu_traction', 'enable_imu_impact_stop'):
        assert p[name] is False, name
    for launch_file in (os.path.join(_SRC, 'gap_follow', 'launch', 'gap_follow_launch.py'),
                        os.path.join(_SRC, 'racerbot_launch', 'launch', 'map_course_launch.py')):
        launch = _read(launch_file)
        assert re.findall(r"'imu_all', default_value='(\w+)'", launch) == ['false'], launch_file
        # Each feature flag either says false outright or follows imu_all.
        for flag in re.findall(r"'((?:fused_speed|imu_\w+))', default_value=", launch):
            if flag in ('imu_all', 'imu_hardware'):
                continue
            m = re.search(r"'%s', default_value=(LaunchConfiguration\('imu_all', "
                          r"default='false'\)|'false')" % flag, launch)
            assert m, f'{launch_file}: {flag} no longer defaults off'


def test_car_1_imu_values_stay_labelled_as_car_1s():
    # Car 1's numbers can stay as placeholders only while they say whose they
    # are; replace the label and the value together when car 2 records its own.
    yaml_text = _read(_SRC, 'gap_follow', 'config', 'gap_follow.yaml')
    if '33.7257297468959' in yaml_text or 'imu_traction_slip_threshold: 0.246' in yaml_text:
        assert "CAR 1'S VALUES" in yaml_text
    sim_imu = _read(_SRC, 'racerbot_sim', 'racerbot_sim', 'sim_imu.py')
    if '0.000711560678439245' in sim_imu:
        assert "CAR 1'S NOISE" in sim_imu


def test_grip_replay_finds_runs_in_car_2s_data_folder():
    # Car 1's grip_replay looks in ~/.ros/racerbot_runs; car 2 saves runs in
    # the workspace (docs/car1-vs-car2.md, "Where runs are saved").
    text = _read(_SRC, '..', 'tools', 'run_analysis', 'grip_replay.py')
    assert re.search(r"^RUNS_DIR = Path\.home\(\) / 'racerbot-ws' / 'data' / 'racerbot_runs'",
                     text, re.M)
    assert '.ros/racerbot_runs' not in text and "'.ros'" not in text
