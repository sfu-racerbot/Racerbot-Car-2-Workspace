"""
config/web_dashboard_rb2.yaml: car 2's settings for web_dashboard and
usb_cam_stream, which live in the src/web_dashboards submodule and ship
generic defaults.

ROS reads a parameters file without complaint when a key is misspelt or a
value is one the node will quietly drop -- the dashboard just starts
without that setting. So each check here compares the YAML with an
independent source it has to agree with:

  * key names    -> the declare_parameter() calls in the two nodes' source
  * laser_offset -> the base_link->laser static transform in every
                    f1tenth_stack *_bringup_launch.py (docs/hardware-reference.md)
  * site origin  -> PUBLIC_ORIGIN in the site's own wrangler.jsonc, and the
                    dashboard's real origins.parse_allowed_origins()
  * tuning files -> the files actually in this workspace
  * killable / map_roots -> the dashboard's real proccontrol / mapstore
                    filters, which drop protected entries at startup
                    (a stop panel missing a node, or a map list missing a
                    folder, would otherwise only show up trackside).

No ROS needed:

    python3 -m pytest src/racerbot_launch/test/ -v
"""
import glob
import os
import re
import sys

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_SRC = os.path.dirname(_PKG)
_CONFIG = os.path.join(_PKG, 'config', 'web_dashboard_rb2.yaml')
_WD = os.path.join(_SRC, 'web_dashboards', 'car', 'ros', 'web_dashboard')
_UCS = os.path.join(_SRC, 'web_dashboards', 'car', 'ros', 'usb_cam_stream')

sys.path.insert(0, _WD)
from web_dashboard import mapstore, origins, proccontrol  # noqa: E402


def _section(node):
    with open(_CONFIG) as handle:
        return yaml.safe_load(handle)[node]['ros__parameters']


def _declared(source_path):
    with open(source_path) as handle:
        names = re.findall(r"declare_parameter\(\s*'(\w+)'", handle.read())
    assert names, f'found no declare_parameter calls in {source_path}'
    return set(names)


class _Log:
    def __init__(self):
        self.lines = []

    def warn(self, text):
        self.lines.append(text)

    warning = error = info = warn


# --------------------------------------------------------------------------
# Every key is one the node declares
# --------------------------------------------------------------------------

@pytest.mark.parametrize('node, source', [
    ('web_dashboard_node', os.path.join(_WD, 'web_dashboard', 'dashboard_node.py')),
    ('usb_cam_stream_node', os.path.join(_UCS, 'usb_cam_stream', 'camera_stream_node.py')),
])
def test_every_key_is_a_parameter_the_node_declares(node, source):
    unknown = set(_section(node)) - _declared(source)
    assert unknown == set(), f'{node} does not declare {sorted(unknown)}; ROS would ignore them'


# --------------------------------------------------------------------------
# Values that must agree with something else
# --------------------------------------------------------------------------

def _bringup_laser_x():
    """{launch file: x} for each bringup's base_link->laser transform."""
    found = {}
    for path in glob.glob(os.path.join(
            _SRC, 'f1tenth_system', 'f1tenth_stack', 'launch', '*bringup_launch.py')):
        with open(path) as handle:
            match = re.search(r"arguments=\['([-\d.]+)',[^\]]*'base_link', 'laser'\]", handle.read())
        if match:
            found[os.path.basename(path)] = float(match.group(1))
    return found


def test_the_lidar_offset_matches_every_bringups_transform():
    bringups = _bringup_laser_x()
    assert len(bringups) >= 1, 'found no base_link->laser transform to compare with'
    assert set(bringups.values()) == {_section('web_dashboard_node')['laser_offset_x']}, bringups


def test_the_allowed_site_is_the_sites_own_public_origin():
    with open(os.path.join(_SRC, 'web_dashboards', 'wrangler.jsonc')) as handle:
        public = re.search(r'"PUBLIC_ORIGIN":\s*"([^"]+)"', handle.read()).group(1)
    allowed, rejected = origins.parse_allowed_origins(_section('web_dashboard_node')['allowed_origins'])
    assert rejected == []
    assert allowed == {public}


def test_every_tunable_node_has_a_config_file_that_exists_here():
    params = _section('web_dashboard_node')
    nodes, files = params['tuning_nodes'], params['tuning_config_files']
    assert len(nodes) == len(files) >= 1
    for relative in files:
        package, rest = relative.split('/', 1)
        assert os.path.isfile(os.path.join(_SRC, package, rest)), relative


def test_no_stoppable_process_is_silently_dropped_as_protected():
    log = _Log()
    wanted = _section('web_dashboard_node')['killable_nodes']
    assert proccontrol.sanitize_allowlist(wanted, log) == wanted
    assert log.lines == []


def test_every_tunable_node_can_also_be_stopped():
    params = _section('web_dashboard_node')
    assert set(params['tuning_nodes']) <= set(params['killable_nodes'])


def test_the_map_folders_are_the_two_run_folders_and_none_is_refused():
    log = _Log()
    roots = _section('web_dashboard_node')['map_roots']
    assert roots == ['~/.ros/racerbot_auto', '~/.ros/racerbot_sim/auto']
    assert len(mapstore.sanitize_roots(roots, log)) == 2
    assert log.lines == []


def test_the_camera_streams_the_realsense_colour_topic():
    camera = _section('usb_cam_stream_node')
    assert camera['image_topic'] == '/camera/camera/color/image_raw'
    assert camera['passthrough'] is False


# --------------------------------------------------------------------------
# The launch files that use it
# --------------------------------------------------------------------------

@pytest.mark.parametrize('launch', ['dashboard_launch.py', 'realsense_camera_launch.py'])
def test_the_car_2_launches_pass_this_yaml_as_car_config(launch):
    with open(os.path.join(_PKG, 'launch', launch)) as handle:
        text = handle.read()
    assert "'web_dashboard_rb2.yaml'" in text
    assert "'car_config'" in text


def test_the_forwarding_bridge_launch_includes_the_moved_one():
    """Kept only so the pre-move systemd unit survives a restart (its
    wrapper runs `ros2 launch racerbot_launch foxglove_bridge_launch.py`).
    It must start the bridge with web_dashboard's clientPublish-free
    config, not a copy of its own."""
    with open(os.path.join(_PKG, 'launch', 'foxglove_bridge_launch.py')) as handle:
        text = handle.read()
    assert "get_package_share_directory('web_dashboard')" in text
    assert "'foxglove_bridge_launch.py'" in text
    assert not os.path.exists(os.path.join(_PKG, 'config', 'foxglove_bridge.yaml'))
