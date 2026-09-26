"""
What record_run.py bags by default.

Read out of the launch file with `ast` rather than imported, so this runs
without ROS like the rest of this package's tests.

The requirement (docs/run-diagnostics.md, "Why a rosbag"): parameter
changes -- from the web dashboard's live tuning, from Lichtblick through
foxglove_bridge, or from a terminal -- must land in each run's bag, and
they all travel on /parameter_events.

    python3 -m pytest src/race_diagnostics/test/test_record_run_topics.py -v
"""
import ast
import os

_LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'record_run.py')


def _default_topics():
    with open(_LAUNCH) as handle:
        tree = ast.parse(handle.read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'DEFAULT_TOPICS' for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError('record_run.py no longer defines DEFAULT_TOPICS')


def test_parameter_changes_are_bagged():
    assert '/parameter_events' in _default_topics()


def test_the_launch_argument_default_is_built_from_that_list():
    """The bag records LaunchConfiguration('topics'), whose default must be
    DEFAULT_TOPICS -- otherwise editing the list changes nothing."""
    with open(_LAUNCH) as handle:
        source = handle.read()
    assert "default_value=' '.join(DEFAULT_TOPICS)" in source


def test_topic_names_are_absolute_and_unique():
    topics = _default_topics()
    assert all(t.startswith('/') and ' ' not in t for t in topics)
    assert len(topics) == len(set(topics))
