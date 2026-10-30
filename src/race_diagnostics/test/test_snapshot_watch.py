"""ROS-free watcher rules; run with pytest src/race_diagnostics/test/."""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from race_diagnostics.run_metadata import NodeWatch  # noqa: E402


def test_new_node_is_not_returned_after_success():
    watch = NodeWatch()
    # oracle: first appearance uses the base name; continuous success is final.
    assert watch.update({'/a'}) == [('/a', 'a.yaml')]
    watch.done('/a')
    assert watch.update({'/a'}) == []


def test_late_node_is_returned_at_poll_50():
    watch = NodeWatch()
    watch.update({'/a'})
    watch.done('/a')
    # oracle: a continuously present successful node is not returned again.
    for _ in range(48):
        assert watch.update({'/a'}) == []
    # oracle: first appearance uses the base name, however late.
    assert watch.update({'/a', '/slam_toolbox'}) == [('/slam_toolbox', 'slam_toolbox.yaml')]
    watch.done('/slam_toolbox')
    assert watch.update({'/a', '/slam_toolbox'}) == []


def test_restart_increments_suffix():
    watch = NodeWatch()
    watch.update({'/a'})
    watch.done('/a')
    # oracle: absence after success followed by return is a numbered restart.
    assert watch.update(set()) == []
    assert watch.update({'/a'}) == [('/a', 'a.1.yaml')]
    watch.done('/a')
    watch.update(set())
    assert watch.update({'/a'}) == [('/a', 'a.2.yaml')]


def test_disappearance_before_success_is_not_restart():
    watch = NodeWatch()
    # oracle: absence before the first success keeps the base name.
    assert watch.update({'/a'}) == [('/a', 'a.yaml')]
    assert watch.update(set()) == []
    assert watch.update({'/a'}) == [('/a', 'a.yaml')]


def test_unanswered_node_is_retried_until_done():
    watch = NodeWatch()
    # oracle: unsuccessful snapshots are retried under the same filename.
    for _ in range(3):
        assert watch.update({'/a'}) == [('/a', 'a.yaml')]
    watch.done('/a')
    assert watch.update({'/a'}) == []


def test_same_node_name_in_two_namespaces():
    watch = NodeWatch()
    # oracle: namespace separators become __, results sorted by full name.
    assert watch.update({'/gap_follow_node', '/car2/gap_follow_node'}) == [
        ('/car2/gap_follow_node', 'car2__gap_follow_node.yaml'),
        ('/gap_follow_node', 'gap_follow_node.yaml')]


def test_collision_keeps_disambiguator_on_restarts():
    watch = NodeWatch()
    # oracle: sorted first name owns base; later collision gets ~2 permanently.
    assert watch.update({'/a__b', '/a/b'}) == [
        ('/a/b', 'a__b.yaml'), ('/a__b', 'a__b~2.yaml')]
    watch.done('/a/b')
    watch.done('/a__b')
    watch.update([])
    assert watch.update({'/a__b', '/a/b'}) == [
        ('/a/b', 'a__b.1.yaml'), ('/a__b', 'a__b~2.1.yaml')]


def test_ignore_duplicates_and_empty_sample():
    watch = NodeWatch(ignore={'/ignored'})
    # oracle: ignored names never appear; duplicates count once; empty yields [].
    assert watch.update([]) == []
    assert watch.update(['/ignored', '/a', '/a', '/ignored']) == [('/a', 'a.yaml')]
    watch.done('/a')
    assert watch.update(['/ignored', '/a']) == []


def test_random_sequence_never_shares_filename():
    rng = random.Random(1234)
    watch = NodeWatch()
    owners = {}
    names = ['/a/b', '/a__b', '/c', '/x/y', '/x__y', '/z']
    for _ in range(30):
        for name, filename in watch.update(n for n in names if rng.random() < 0.6):
            # oracle: invariant, no filename is ever assigned to two full names.
            assert owners.setdefault(filename, name) == name
            watch.done(name)
    # oracle: the fixed sequence includes both collisions, so the invariant is exercised.
    assert set(owners.values()) == set(names)
