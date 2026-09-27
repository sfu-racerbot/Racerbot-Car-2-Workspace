"""
config/foxglove_bridge.yaml: what a Lichtblick client can publish and see.

The client-publish check is the safety-relevant half (A8): a client able
to publish /drive, /teleop, /ackermann_cmd or /commands/* would drive the
car without any driving node's LB deadman (docs/architecture.md). On
foxglove_bridge 3.5.0 only removing the clientPublish capability prevents
that -- see test_client_publishing_is_switched_off_entirely. The
whitelist tests below still read the real YAML and apply its regexes the
way a bridge that enforced them would -- a topic is allowed when ANY entry
matches it -- so a widened regex fails here too.

The topic names are the real ones: the RealSense names come from
realsense2_camera 4.x as launched by realsense_camera_launch.py (namespace
'camera', name 'camera'), the rest from docs/architecture.md's topic table.

Python's `re` stands in for ECMAScript regex; the one construct used
beyond plain regex, a negative lookahead, means the same in both.

    python3 -m pytest src/racerbot_launch/test/ -v
"""
import os
import re

import pytest
import yaml

_CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'foxglove_bridge.yaml')


def _params():
    with open(_CONFIG) as handle:
        return yaml.safe_load(handle)['foxglove_bridge']['ros__parameters']


def _allowed(whitelist, topic):
    return any(re.search(pattern, topic) for pattern in whitelist)


# --------------------------------------------------------------------------
# Client publishing
# --------------------------------------------------------------------------

DRIVE_PATH = [
    '/drive', '/teleop', '/ackermann_cmd',
    '/commands/motor/speed', '/commands/motor/duty_cycle',
    '/commands/motor/current', '/commands/motor/brake',
    '/commands/motor/position', '/commands/servo/position',
    '/auto_map/drive', '/auto_race/drive',
    '/joy',                       # forging LB is forging the deadman itself
]


@pytest.mark.parametrize('topic', DRIVE_PATH + [
    '/initialpose/extra', '/x/initialpose', '/initialpose ', '/initialposes',
    '/goal_pose', '/map', '/scan', '/tf', '/parameter_events'])
def test_a_client_cannot_publish_anything_but_initialpose(topic):
    assert not _allowed(_params()['client_topic_whitelist'], topic)


def test_the_backup_whitelist_names_only_the_pose_estimate():
    assert _allowed(_params()['client_topic_whitelist'], '/initialpose')
    assert _params()['client_topic_whitelist'] == ['^/initialpose$']


def test_client_publishing_is_switched_off_entirely():
    """THE test that matters in this file.

    Oracle: a recorded measurement, 2026-09-27, foxglove_bridge 3.5.0 on
    this car (isolated ROS domain 79, port 18765, this YAML):
      * with clientPublish in `capabilities`, a client advertised /drive,
        published one message, and `ros2 topic echo /drive` received it --
        client_topic_whitelist is declared but NOT enforced;
      * without clientPublish, the advertisement was refused ("Server does
        not support clientPublish capability") and nothing arrived.
    So the capability, not the whitelist, is the only thing between a
    browser and a deadman-free /drive (docs/foxglove-bridge.md).
    """
    capabilities = [str(c) for c in _params()['capabilities']]
    assert 'clientPublish' not in capabilities
    assert not any(c.lower() == 'clientpublish' for c in capabilities)


# --------------------------------------------------------------------------
# Subscribing: no raw images
# --------------------------------------------------------------------------

RAW = [
    '/camera/camera/color/image_raw',
    '/camera/camera/depth/image_rect_raw',
    '/camera/camera/infra1/image_rect_raw',
    '/camera/camera/aligned_depth_to_color/image_raw',
    '/camera/camera/color/image_raw/theora',
    '/camera/camera/color/image_raw/zstd',
    '/image_raw',
    '/usb_cam/image',
]
KEPT = [
    '/camera/camera/color/image_raw/compressed',
    '/camera/camera/depth/image_rect_raw/compressedDepth',
    '/camera/camera/color/camera_info',
    '/camera/camera/imu',
    '/scan', '/map', '/odom', '/tf', '/tf_static', '/drive', '/ackermann_cmd',
    '/pf/viz/inferred_pose', '/slam_pose', '/drive_intent', '/racing_line',
    '/parameter_events', '/rosout', '/diagnostics',
    '/image_raw_stats',           # a name that merely starts like one
]


@pytest.mark.parametrize('topic', RAW)
def test_raw_images_are_not_offered(topic):
    assert not _allowed(_params()['topic_whitelist'], topic)


@pytest.mark.parametrize('topic', KEPT)
def test_everything_else_is_offered(topic):
    assert _allowed(_params()['topic_whitelist'], topic)


# --------------------------------------------------------------------------
# Where it listens, and what the prompt asked to be open
# --------------------------------------------------------------------------

def test_it_listens_on_loopback_8765_only():
    """Reached only through the tunnel (rb2-bridge-origin -> 127.0.0.1:8765)."""
    params = _params()
    assert params['address'] == '127.0.0.1'
    assert params['port'] == 8765


def test_services_and_parameters_are_open_and_foxglove_cloud_is_off():
    params = _params()
    for capability in ('parameters', 'parametersSubscribe', 'services'):
        assert capability in params['capabilities']
    assert params['service_whitelist'] == ['.*']
    assert params['param_whitelist'] == ['.*']
    assert params['remote_access'] is False


def test_every_regex_compiles():
    params = _params()
    for key in ('topic_whitelist', 'client_topic_whitelist', 'service_whitelist',
                'param_whitelist', 'asset_uri_allowlist'):
        for pattern in params[key]:
            re.compile(pattern)
