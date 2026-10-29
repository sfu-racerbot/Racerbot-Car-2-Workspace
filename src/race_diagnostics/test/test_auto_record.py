"""
race_diagnostics.auto_record: when bringup's automatic recorder records.
No ROS needed:

    python3 -m pytest src/race_diagnostics/test/test_auto_record.py -v

Oracles: the rules in auto_record.py's docstring and
docs/run-diagnostics.md#automatic-recording (record only while an algorithm
runs; never beside another recorder; stop when the algorithm goes), applied
to the shipped config/auto_record.yaml and to the node names each driving
package publishes /drive under (gap_follow.yaml, pure_pursuit.yaml,
auto_map_race.yaml, neural_racer.yaml; team A's ftg_launch.py and
create_publisher("drive") calls in src/racerbot_a).
"""
import ast
import os
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from race_diagnostics import auto_record as ar  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG / 'config' / 'auto_record.yaml'
RECORD_RUN = PKG / 'launch' / 'record_run.py'


def shipped_params():
    return yaml.safe_load(CONFIG.read_text())['auto_record']['ros__parameters']


def shipped(own=ar.recorder_node_names(ar.AUTO_PREFIX)):
    return ar.settings_from_params(shipped_params(), own_nodes=own)


def plain(start=2.0, stop=3.0):
    return ar.Settings(
        trigger_topics=('/drive',), ignore_publishers=('ackermann_mux',),
        other_recorders=('race_diag_node', 'rosbag2_recorder*'),
        own_nodes=ar.recorder_node_names(ar.AUTO_PREFIX),
        start_delay_sec=start, stop_delay_sec=stop)


ALGO = {'/drive': ['/gap_follow_node']}
NOBODY = {}
BRINGUP = ['/joy', '/ackermann_mux', '/vesc_driver_node', '/urg_node', '/auto_record']


def run(machine, steps):
    """steps: [(t, publishers, nodes)] -> the Decision for each."""
    return [machine.update(t, pubs, nodes) for t, pubs, nodes in steps]


# ----------------------------------------------------------- starting

def test_nothing_publishing_never_starts():
    m = ar.AutoRecorder(plain())
    decisions = run(m, [(t, NOBODY, BRINGUP) for t in range(0, 600, 1)])
    assert all(d.action == '' for d in decisions)
    assert m.state == ar.IDLE


def test_starts_exactly_when_the_algorithm_has_been_up_start_delay():
    m = ar.AutoRecorder(plain(start=2.0))
    assert m.update(10.0, ALGO, BRINGUP).action == ''
    assert m.update(11.999, ALGO, BRINGUP).action == ''      # one step short
    d = m.update(12.0, ALGO, BRINGUP)                         # at the boundary
    assert d.action == 'start'
    assert d.algorithms == {'/drive': ['/gap_follow_node']}
    assert 'gap_follow_node on /drive' in d.reason
    # Once started it does not start again while the same run goes on.
    assert m.update(12.5, ALGO, BRINGUP).action == ''


def test_zero_start_delay_starts_on_the_first_sighting():
    m = ar.AutoRecorder(plain(start=0.0))
    assert m.update(5.0, ALGO, BRINGUP).action == 'start'


def test_an_algorithm_that_leaves_before_the_delay_is_never_recorded():
    m = ar.AutoRecorder(plain(start=2.0))
    m.update(0.0, ALGO, BRINGUP)
    m.update(1.5, NOBODY, BRINGUP)
    # It comes back: the delay counts from the return, not the first sighting.
    assert m.update(2.5, ALGO, BRINGUP).action == ''
    assert m.update(4.4, ALGO, BRINGUP).action == ''
    assert m.update(4.5, ALGO, BRINGUP).action == 'start'


# ------------------------------------------- another recorder (the deny path)

@pytest.mark.parametrize('recorder', [
    '/race_diag_node',              # record_run.py, map_course, auto_map_race
    '/rosbag2_recorder',            # a plain `ros2 bag record`
    '/rosbag2_recorder_12345',
    '/car1/race_diag_node',         # a namespace does not hide it
])
def test_never_records_while_another_recorder_is_on_the_graph(recorder):
    m = ar.AutoRecorder(plain(start=2.0))
    decisions = run(m, [(t * 0.5, ALGO, BRINGUP + [recorder]) for t in range(240)])
    assert all(d.action == '' for d in decisions), 'started beside another recorder'
    assert decisions[-1].recorders == [recorder]
    assert 'another recorder' in decisions[-1].reason


def test_a_recorder_that_appears_mid_run_stops_this_one():
    m = ar.AutoRecorder(plain(start=0.0))
    assert m.update(0.0, ALGO, BRINGUP).action == 'start'
    d = m.update(30.0, ALGO, BRINGUP + ['/race_diag_node'])
    assert d.action == 'stop'
    assert 'another recorder started: /race_diag_node' in d.reason
    # And it stays stopped while that recorder runs.
    assert all(m.update(30.0 + t, ALGO, BRINGUP + ['/race_diag_node']).action == ''
               for t in range(1, 60))


def test_recording_resumes_when_the_other_recorder_leaves_and_the_algorithm_still_runs():
    m = ar.AutoRecorder(plain(start=2.0))
    m.update(0.0, ALGO, BRINGUP + ['/rosbag2_recorder'])
    m.update(10.0, ALGO, BRINGUP + ['/rosbag2_recorder'])
    assert m.update(11.0, ALGO, BRINGUP).action == ''
    assert m.update(12.9, ALGO, BRINGUP).action == ''
    assert m.update(13.0, ALGO, BRINGUP).action == 'start'


def test_a_recorder_arriving_during_the_start_delay_resets_it():
    # map_course: the algorithm can reach the graph before its recorder does.
    m = ar.AutoRecorder(plain(start=3.0))
    m.update(0.0, ALGO, BRINGUP)
    assert m.update(1.0, ALGO, BRINGUP + ['/race_diag_node']).action == ''
    assert m.update(10.0, ALGO, BRINGUP + ['/race_diag_node']).action == ''
    assert m.state == ar.IDLE


def test_our_own_recording_nodes_are_not_another_recorder():
    m = ar.AutoRecorder(plain(start=0.0))
    assert m.update(0.0, ALGO, BRINGUP).action == 'start'
    ours = ['/' + n for n in ar.recorder_node_names(ar.AUTO_PREFIX)]
    decisions = run(m, [(t, ALGO, BRINGUP + ours) for t in range(1, 30)])
    assert all(d.action == '' for d in decisions)
    assert m.state == ar.RECORDING


def test_own_nodes_are_excluded_even_when_a_broad_pattern_matches_them():
    # Someone widens other_recorders to catch renamed recorders; the
    # recorder must still not take its own nodes for one and stop itself.
    s = ar.Settings(trigger_topics=('/drive',),
                    other_recorders=('*race_diag_node', '*rosbag2_recorder*'),
                    own_nodes=ar.recorder_node_names(ar.AUTO_PREFIX),
                    start_delay_sec=0.0)
    ours = ['/' + n for n in ar.recorder_node_names(ar.AUTO_PREFIX)]
    assert ar.other_recorders(ours, s) == []
    assert ar.other_recorders(ours + ['/race_diag_node'], s) == ['/race_diag_node']
    m = ar.AutoRecorder(s)
    assert m.update(0.0, ALGO, BRINGUP).action == 'start'
    assert m.update(1.0, ALGO, BRINGUP + ours).action == ''
    assert m.state == ar.RECORDING


# ----------------------------------------------------------- stopping

def test_stops_exactly_stop_delay_after_the_algorithm_goes():
    m = ar.AutoRecorder(plain(start=0.0, stop=3.0))
    m.update(0.0, ALGO, BRINGUP)
    m.update(20.0, ALGO, BRINGUP)                 # last seen at 20.0
    assert m.update(21.0, NOBODY, BRINGUP).action == ''
    assert m.update(22.999, NOBODY, BRINGUP).action == ''
    d = m.update(23.0, NOBODY, BRINGUP)
    assert d.action == 'stop'
    assert 'no algorithm publishing for 3.0 s' in d.reason
    assert m.state == ar.IDLE


def test_an_algorithm_back_within_the_stop_delay_keeps_the_same_recording():
    m = ar.AutoRecorder(plain(start=0.0, stop=3.0))
    m.update(0.0, ALGO, BRINGUP)
    m.update(9.5, ALGO, BRINGUP)                  # last seen at 9.5
    assert m.update(10.0, NOBODY, BRINGUP).action == ''
    assert m.update(12.0, NOBODY, BRINGUP).action == ''
    assert m.update(12.5, ALGO, BRINGUP).action == ''
    # The gone-clock restarts from the return.
    assert m.update(15.0, NOBODY, BRINGUP).action == ''
    assert m.update(15.5, NOBODY, BRINGUP).action == 'stop'


def test_a_new_algorithm_after_a_stop_is_a_new_recording():
    m = ar.AutoRecorder(plain(start=1.0, stop=1.5))
    starts = [d.action for d in run(m, [
        (0.0, ALGO, BRINGUP), (1.0, ALGO, BRINGUP),              # run 1
        (2.0, NOBODY, BRINGUP), (3.0, NOBODY, BRINGUP),          # gone 2.0 s: stopped
        (10.0, ALGO, BRINGUP), (11.0, ALGO, BRINGUP)])]          # run 2
    assert starts == ['', 'start', '', 'stop', '', 'start']


def test_a_recording_that_dies_is_not_restarted_until_the_algorithm_restarts():
    m = ar.AutoRecorder(plain(start=0.0))
    assert m.update(0.0, ALGO, BRINGUP).action == 'start'
    d = m.update(1.0, ALGO, BRINGUP, recording_alive=False)
    assert d.action == 'stop'
    assert 'exited on its own' in d.reason
    held = [m.update(1.0 + t, ALGO, BRINGUP).action for t in range(1, 120)]
    assert held == [''] * 119, 'restarted a recorder that keeps dying'
    m.update(200.0, NOBODY, BRINGUP)
    assert m.update(201.0, ALGO, BRINGUP).action == 'start'


# ------------------------------------------- which publishers count (shipped)

@pytest.mark.parametrize('topic,node', [
    ('/drive', '/gap_follow_node'),             # gap_follow.yaml
    ('/drive', '/pure_pursuit_node'),           # pure_pursuit.yaml
    ('/drive', '/auto_map_race_node'),          # auto_map_race.yaml
    ('/auto_map/drive', '/gap_follow_node'),    # map_course / auto_map_race
    ('/auto_race/drive', '/pure_pursuit_node'),
    ('/drive', '/neural_racer_node'),           # neural_racer.yaml, shadow:=false
    ('/neural_racer/shadow_drive', '/neural_racer_node'),  # its default shadow mode
    ('/drive', '/ftg_node'),                    # team A ftg_launch.py
    ('/drive', '/disparity_extender_node'),     # team A
    ('/drive', '/some_new_node'),               # anyone else's
    ('/ackermann_cmd', '/bypasses_the_mux'),    # a node skipping ackermann_mux
    ('/commands/motor/speed', '/straight_to_vesc'),
])
def test_every_algorithm_starts_a_recording_with_the_shipped_config(topic, node):
    m = ar.AutoRecorder(shipped())
    delay = shipped().start_delay_sec
    m.update(0.0, {topic: [node]}, BRINGUP)
    d = m.update(delay, {topic: [node]}, BRINGUP)
    assert d.action == 'start'
    assert d.algorithms == {topic: [node]}


@pytest.mark.parametrize('topic,node', [
    ('/teleop', '/joy_teleop'),                     # manual driving
    ('/ackermann_cmd', '/ackermann_mux'),           # bringup itself
    ('/commands/motor/speed', '/ackermann_to_vesc_node'),
    ('/commands/servo/position', '/ackermann_to_vesc_node'),
    ('/drive', '/rosbag2_player'),                  # a bag played back
    ('/drive_intent', '/gap_follow_node'),          # a diagnostic, not a command
])
def test_bringup_teleop_and_playback_never_start_a_recording(topic, node):
    m = ar.AutoRecorder(shipped())
    decisions = run(m, [(t, {topic: [node]}, BRINGUP) for t in range(0, 120)])
    assert all(d.action == '' for d in decisions)
    assert decisions[-1].algorithms == {}


def test_bringup_alone_with_its_command_publishers_is_idle():
    """What bringup really puts on the trigger topics, all at once."""
    pubs = {'/ackermann_cmd': ['/ackermann_mux'],
            '/commands/motor/speed': ['/ackermann_to_vesc_node'],
            '/commands/servo/position': ['/ackermann_to_vesc_node'],
            '/teleop': ['/joy_teleop']}
    m = ar.AutoRecorder(shipped())
    assert all(d.action == '' for d in run(m, [(t, pubs, BRINGUP) for t in range(60)]))


def test_an_algorithm_beside_bringup_is_reported_alone():
    pubs = {'/ackermann_cmd': ['/ackermann_mux'], '/drive': ['/ftg_node']}
    m = ar.AutoRecorder(shipped())
    m.update(0.0, pubs, BRINGUP)
    d = m.update(shipped().start_delay_sec, pubs, BRINGUP)
    assert d.action == 'start'
    assert d.algorithms == {'/drive': ['/ftg_node']}


def test_shipped_patterns_tell_a_manual_record_run_from_our_own():
    """record_run.py without a prefix is someone else's recorder; the same
    launch file started by auto_record (AUTO_PREFIX) is ours."""
    s = shipped()
    manual = ['/' + n for n in ar.recorder_node_names('')]
    ours = ['/' + n for n in ar.recorder_node_names(ar.AUTO_PREFIX)]
    assert ar.other_recorders(manual, s) == ['/race_diag_node', '/rosbag2_recorder']
    assert ar.other_recorders(ours, s) == []


def test_shipped_delays_are_positive_and_bounded():
    s = shipped()
    # A recorder brought by map_course must be able to win the start race,
    # and a stop must not linger long after the algorithm.
    assert 1.0 <= s.start_delay_sec <= 10.0
    assert 1.0 <= s.stop_delay_sec <= 10.0


def test_every_trigger_topic_is_bagged_by_record_run():
    """A recording must contain what started it."""
    tree = ast.parse(RECORD_RUN.read_text())
    topics = next(ast.literal_eval(node.value) for node in tree.body
                  if isinstance(node, ast.Assign)
                  and any(getattr(t, 'id', None) == 'DEFAULT_TOPICS' for t in node.targets))
    missing = set(shipped().trigger_topics) - set(topics)
    assert not missing, f'record_run.py does not bag trigger topics {sorted(missing)}'


def test_describe_names_every_algorithm():
    assert ar.describe({'/drive': ['/a', '/b'], '/auto_map/drive': ['/c']}) == \
        '/c on /auto_map/drive, /a+/b on /drive'
