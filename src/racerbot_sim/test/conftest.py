"""Fail before ROS test setup can create a publisher or start a process."""
import os

import pytest


PRODUCTION_DRIVE_TOPICS = {'/drive', '/car2/drive', '/ackermann_cmd', '/car2/ackermann_cmd'}


def require_test_drive_topic(topic):
    if topic in PRODUCTION_DRIVE_TOPICS:
        pytest.fail(f'production drive topic forbidden in tests: {topic}')


def require_isolation():
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    if (not domain.isdecimal() or int(domain) == 0
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        pytest.fail('ROS tests require nonzero ROS_DOMAIN_ID and '
                    'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST before any publisher or process')


@pytest.fixture(autouse=True)
def isolated_ros_tests(request, monkeypatch):
    if request.module.__name__.split('.')[-1] not in ('test_controlled_cars', 'test_sim_race_launch'):
        return
    require_isolation()
    from rclpy.node import Node
    original = Node.create_publisher

    def guarded(self, msg_type, topic, *args, **kwargs):
        require_test_drive_topic(self.resolve_topic_name(topic))
        return original(self, msg_type, topic, *args, **kwargs)

    monkeypatch.setattr(Node, 'create_publisher', guarded)
