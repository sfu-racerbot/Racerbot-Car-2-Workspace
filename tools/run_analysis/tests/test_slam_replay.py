"""Tests for tools.run_analysis.slam_replay. ROS-free, so this runs unsourced.

Runner (TEST_QUALITY_STANDARDS.md A9):
    python3 -m pytest tools/run_analysis/tests/test_slam_replay.py -v -p no:cacheprovider
slam_replay._run starts ROS processes and needs a live graph, so it is not run here. It is
verified by a smoke replay, recorded in the task report.

Oracle markers (A3): "closed form" = recomputed from the spec in the test; "spec" = the task
brief's values or a file named by path; "run" = a value read from the recorded run
~/.ros/racerbot_runs/20261026-194514. The fixtures copy that run's file shapes, not its folder.

The process table is a fake /proc under tmp_path (PROC_ROOT) and nothing is written under the
real ~/.ros: RUNS_ROOT is redirected too. Popen is replaced with a trap wherever a test must
prove that nothing was started.
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))  # insert tools/
from run_analysis import slam_replay  # noqa: E402
from run_analysis.slam_replay import ReplayFailed, ReplayRefused, ServiceError  # noqa: E402

RUN_NAME = '20261026-194514'
ALL_TOPICS = ['/scan', '/tf', '/tf_static', '/odom', '/odom/imu', '/map']
URG = b'/opt/ros/jazzy/lib/urg_node/urg_node_driver\x00--ros-args\x00-r\x00__node:=urg_node\x00'
VESC_PATH_ONLY = b'/home/x/install/vesc_driver/lib/vesc_driver/vesc_driver_node\x00'

# spec: the shape of ~/.ros/racerbot_runs/20261026-194514/metadata/params/slam_toolbox.yaml
# (a top key with a leading slash, ros__parameters below it), trimmed to the keys these tests read.
# The values are that run's: loop_search_maximum_distance 8.0, mode mapping, use_sim_time false.
SNAPSHOT = """\
/slam_toolbox:
  ros__parameters:
    loop_search_maximum_distance: 8.0
    mode: mapping
    odom_frame: odom_imu
    qos_overrides:
      /tf:
        publisher:
          depth: 100
    start_type_description_service: true
    use_sim_time: false
"""


def _bag_metadata(topics):
    """The layout of a rosbag2 metadata.yaml, as in <run>/bag/metadata.yaml of the recorded run."""
    return {'rosbag2_bagfile_information': {
        'version': 8,
        'storage_identifier': 'mcap',
        'topics_with_message_count': [
            {'topic_metadata': {'name': t, 'type': 'sensor_msgs/msg/LaserScan',
                                'serialization_format': 'cdr'},
             'message_count': 1}
            for t in topics],
    }}


def make_run(root: Path, topics=ALL_TOPICS, snapshot=SNAPSHOT) -> Path:
    """A complete fake run: a bag listing `topics`, and the recorded SLAM settings."""
    run = root / RUN_NAME
    (run / 'bag').mkdir(parents=True)
    (run / 'bag' / 'metadata.yaml').write_text(yaml.safe_dump(_bag_metadata(topics)), encoding='utf-8')
    params = run / 'metadata' / 'params'
    params.mkdir(parents=True)
    (params / 'slam_toolbox.yaml').write_text(snapshot, encoding='utf-8')
    return run


def _proc(root: Path, name: str, cmdline: bytes) -> None:
    """A fake /proc/<name>/cmdline."""
    (root / name).mkdir()
    (root / name / 'cmdline').write_bytes(cmdline)


# --- child_env ---------------------------------------------------------------------------------

def test_child_env_always_isolates():
    # spec: the task brief's values for the replay's own graph: domain 77, LOCALHOST discovery,
    # no static peers, whatever the caller had.
    base = {'ROS_DOMAIN_ID': '0', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET',
            'ROS_STATIC_PEERS': '10.0.0.5', 'PATH': '/x'}
    before = dict(base)
    env = slam_replay.child_env(base)
    assert slam_replay.REPLAY_DOMAIN_ID == 77  # spec: the task brief
    assert env['ROS_DOMAIN_ID'] == '77'
    assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert env['ROS_STATIC_PEERS'] == ''
    assert env['PATH'] == '/x'  # unrelated variables pass through
    assert base == before  # the caller's environment is not modified


def test_child_env_adds_the_isolation_to_an_empty_environment():
    # closed form - with no ROS variables at all, exactly the three are set.
    assert slam_replay.child_env({}) == {
        'ROS_DOMAIN_ID': '77', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST', 'ROS_STATIC_PEERS': ''}


# --- running_drivers (the interlock) -----------------------------------------------------------

@pytest.mark.parametrize('marker, cmdline', [
    ('vesc_driver_node', b'/home/x/install/vesc_driver/lib/vesc_driver/vesc_driver_node\x00'
                         b'--ros-args\x00-r\x00__node:=vesc_driver_node\x00'),
    ('urg_node', URG),
    ('joy_node', b'/opt/ros/jazzy/lib/joy/joy_node\x00'),
    ('__node:=joy', b'/opt/ros/jazzy/lib/joy/joy\x00--ros-args\x00-r\x00__node:=joy\x00'),
])
def test_running_drivers_finds_each_marker(tmp_path, marker, cmdline):
    # oracle: closed form - each cmdline names exactly one marker, so the answer is that marker.
    _proc(tmp_path, '4242', cmdline)
    assert slam_replay.running_drivers(tmp_path) == [marker]


def test_running_drivers_matches_executable_path_without_node_remap(tmp_path):
    # Review Focus 1: a driver started under another name (no __node:= remap) still trips the
    # interlock, because the match is on the whole command line.
    _proc(tmp_path, '4242', VESC_PATH_ONLY)
    assert slam_replay.running_drivers(tmp_path) == ['vesc_driver_node']


def test_running_drivers_skips_unreadable_and_vanished_pids(tmp_path):
    _proc(tmp_path, '7', URG)  # a live driver: it must still be found
    (tmp_path / '8').mkdir()  # no cmdline file: the process exited after it was listed
    _proc(tmp_path, '9', b'')  # empty cmdline: a kernel thread
    (tmp_path / 'sys').mkdir()  # not a pid directory, so it must not be read at all
    (tmp_path / 'sys' / 'cmdline').write_bytes(VESC_PATH_ONLY)
    assert slam_replay.running_drivers(tmp_path) == ['urg_node']


def test_running_drivers_reads_a_command_line_that_is_not_utf8(tmp_path):
    # A4 parser edge: an invalid byte in one argument must not hide a driver.
    _proc(tmp_path, '7', b'/x/\xff\xfe/urg_node\x00--ros-args\x00')
    assert slam_replay.running_drivers(tmp_path) == ['urg_node']


def test_running_drivers_ignores_processes_that_are_not_drivers(tmp_path):
    # invariant - a recorder that names /joy is not a driver, and neither is a shell or systemd.
    _proc(tmp_path, '10', b'/usr/bin/python3\x00-m\x00pytest\x00')
    _proc(tmp_path, '11', b'/usr/lib/systemd/systemd\x00--user\x00')
    _proc(tmp_path, '12', b'/opt/ros/jazzy/bin/ros2\x00bag\x00record\x00-o\x00/tmp/x/bag\x00/joy\x00/tf\x00')
    assert slam_replay.running_drivers(tmp_path) == []


def test_running_drivers_with_no_processes_is_empty(tmp_path):
    (tmp_path / 'proc').mkdir()
    assert slam_replay.running_drivers(tmp_path / 'proc') == []


# --- slam_parameters ---------------------------------------------------------------------------

SNAP = {'loop_search_maximum_distance': 8.0, 'odom_frame': 'odom', 'use_sim_time': False,
        'qos_overrides./tf.publisher.depth': 10}


def test_slam_parameters_layer_order():
    # closed form - the profile lays over the snapshot (3.0 beats 8.0), the forced keys lay over
    # both (odom_imu, true), and the qos_overrides key is dropped.
    params = slam_replay.slam_parameters(SNAP, 'odom_imu', {'loop_search_maximum_distance': 3.0})
    assert params == {'slam_toolbox': {'ros__parameters': {
        'loop_search_maximum_distance': 3.0, 'odom_frame': 'odom_imu', 'use_sim_time': True}}}
    assert params['slam_toolbox']['ros__parameters']['use_sim_time'] is True


def test_slam_parameters_forced_keys_beat_the_profile():
    # invariant - a profile cannot turn the replay's sim time off or move it to another frame.
    ros = slam_replay.slam_parameters({}, 'odom_imu', {'use_sim_time': False, 'odom_frame': 'odom'})
    ros = ros['slam_toolbox']['ros__parameters']
    assert ros['use_sim_time'] is True
    assert ros['odom_frame'] == 'odom_imu'


def test_slam_parameters_drops_the_live_graph_keys_from_both_layers():
    # closed form - of the snapshot and the profile, only `mode` survives, plus the forced keys.
    snap = {'start_type_description_service': True, 'mode': 'mapping',
            'qos_overrides': {'/tf': {'publisher': {'depth': 100}}}}
    prof = {'qos_overrides./tf.publisher.depth': 5, 'start_type_description_service': False}
    ros = slam_replay.slam_parameters(snap, 'odom', prof)['slam_toolbox']['ros__parameters']
    assert ros == {'mode': 'mapping', 'odom_frame': 'odom', 'use_sim_time': True}


def test_slam_parameters_without_a_profile_keeps_the_recorded_values():
    ros = slam_replay.slam_parameters(SNAP, 'odom', None)['slam_toolbox']['ros__parameters']
    assert ros['loop_search_maximum_distance'] == 8.0  # the recorded value, unchanged


def test_slam_parameters_does_not_modify_its_inputs():
    snap, prof = dict(SNAP), {'loop_search_maximum_distance': 3.0}
    slam_replay.slam_parameters(snap, 'odom_imu', prof)
    assert snap == SNAP
    assert prof == {'loop_search_maximum_distance': 3.0}


def test_params_file_leaves_out_only_nulls_and_empty_lists():
    # The smoke replay of 20261026-194514 aborted on map_start_pose: [] ("parameter_value_from failed
    # for parameter 'map_start_pose': No parameter value set", in its slam_toolbox.log). Values that
    # look empty or false are valid parameters and must stay: '', 0 and False.
    ros = {'map_start_pose': [], 'map_file_name': '', 'debug_logging': False, 'throttle_scans': 0,
           'start_pose': [0.0, 0.0, 0.0], 'enable_interactive_mode': None}
    kept = slam_replay.loadable({'slam_toolbox': {'ros__parameters': ros}})
    assert kept == {'slam_toolbox': {'ros__parameters': {
        'map_file_name': '', 'debug_logging': False, 'throttle_scans': 0, 'start_pose': [0.0, 0.0, 0.0]}}}


# --- bag_topics ----------------------------------------------------------------------------------

def test_bag_topics_reads_the_names_from_a_rosbag2_metadata_file(tmp_path):
    # spec: the layout of <run>/bag/metadata.yaml. The names are the ones the fixture wrote.
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(_bag_metadata(['/scan', '/odom/imu', '/tf_static'])))
    assert slam_replay.bag_topics(bag) == {'/scan', '/odom/imu', '/tf_static'}


def test_bag_topics_refuses_a_truncated_metadata_file(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information: {topics_with_message_count: [\n')
    with pytest.raises(ReplayRefused, match='unreadable'):
        slam_replay.bag_topics(bag)


def test_bag_topics_refuses_bytes_that_are_not_utf8(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_bytes(b'\xff\xfe\x00rosbag2')
    with pytest.raises(ReplayRefused, match='unreadable'):
        slam_replay.bag_topics(bag)


def test_bag_topics_refuses_a_file_with_no_topic_list(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information:\n  version: 8\n')
    with pytest.raises(ReplayRefused, match='topic list'):
        slam_replay.bag_topics(bag)


# --- plan_replay ----------------------------------------------------------------------------------

def test_plan_never_plays_recorded_tf(tmp_path):
    # spec: the task brief - /scan, /tf_static and the odometry topic; the recorded /tf is never played.
    run = make_run(tmp_path, topics=['/scan', '/tf', '/tf_static', '/odom/imu'])
    plan = slam_replay.plan_replay(run, 'odom_imu', None, tmp_path / 'out')
    assert plan.play_topics == ['/scan', '/tf_static', '/odom/imu']
    assert '/tf' not in plan.play_topics
    assert plan.odom_topic == '/odom/imu'


def test_plan_refuses_a_run_without_the_odometry_topic(tmp_path):
    # Review Focus 2: a gyro-off run has no /odom/imu. Refuse before anything is started, and
    # name the missing topic.
    run = make_run(tmp_path, topics=['/scan', '/odom', '/tf_static'])
    with pytest.raises(ReplayRefused, match='/odom/imu'):
        slam_replay.plan_replay(run, 'odom_imu', None, tmp_path / 'out')
    assert not (tmp_path / 'out').exists()  # nothing was created


@pytest.mark.parametrize('missing', ['/scan', '/tf_static'])
def test_plan_refuses_a_run_without_scans_or_static_transforms(tmp_path, missing):
    # The replay cannot localise without the scans, or without base_link -> laser in /tf_static.
    run = make_run(tmp_path, topics=[t for t in ALL_TOPICS if t != missing])
    with pytest.raises(ReplayRefused, match=missing):
        slam_replay.plan_replay(run, 'odom', None, tmp_path / 'out')


def test_plan_refuses_a_run_with_no_bag(tmp_path):
    run = make_run(tmp_path)
    shutil.rmtree(run / 'bag')
    with pytest.raises(ReplayRefused, match='has no bag/metadata.yaml'):
        slam_replay.plan_replay(run, 'odom', None, tmp_path / 'out')


def test_plan_refuses_a_run_without_its_recorded_slam_settings(tmp_path):
    run = make_run(tmp_path)
    (run / 'metadata' / 'params' / 'slam_toolbox.yaml').unlink()
    with pytest.raises(ReplayRefused, match='has no metadata/params/slam_toolbox.yaml'):
        slam_replay.plan_replay(run, 'odom', None, tmp_path / 'out')


def test_plan_names_the_output_folder_after_the_run_odometry_and_label(tmp_path):
    # spec: the task brief - <run name>-replay-<odom>-<label>, with the label defaulting to "default".
    run = make_run(tmp_path)
    out_root = tmp_path / 'runs'
    plan = slam_replay.plan_replay(run, 'odom_imu', None, out_root, label='smoke')
    assert plan.out_dir == out_root / f'{RUN_NAME}-replay-odom_imu-smoke'
    default = slam_replay.plan_replay(run, 'odom', None, out_root)
    assert default.out_dir == out_root / f'{RUN_NAME}-replay-odom-default'


def test_plan_refuses_an_output_folder_that_already_exists(tmp_path):
    run = make_run(tmp_path)
    out_root = tmp_path / 'runs'
    (out_root / f'{RUN_NAME}-replay-odom-smoke').mkdir(parents=True)
    with pytest.raises(ReplayRefused, match='already exists'):
        slam_replay.plan_replay(run, 'odom', None, out_root, label='smoke')


@pytest.mark.parametrize('label', ['', 'a/b', '../up', 'two words'])
def test_plan_refuses_a_label_that_is_not_a_plain_folder_name(tmp_path, label):
    run = make_run(tmp_path)
    with pytest.raises(ReplayRefused, match='label'):
        slam_replay.plan_replay(run, 'odom', None, tmp_path / 'out', label=label)


def test_plan_refuses_an_unknown_odometry_frame(tmp_path):
    run = make_run(tmp_path)
    with pytest.raises(ReplayRefused, match='unknown odometry frame'):
        slam_replay.plan_replay(run, 'odom_gps', None, tmp_path / 'out')


def test_plan_reads_the_recorded_settings_and_forces_sim_time(tmp_path):
    # oracle: run - loop_search_maximum_distance 8.0 is the value in 20261026-194514's snapshot.
    # The replay forces use_sim_time and the odometry frame, whatever was recorded.
    run = make_run(tmp_path)
    plan = slam_replay.plan_replay(run, 'odom_imu', None, tmp_path / 'out')
    ros = plan.params['slam_toolbox']['ros__parameters']
    assert ros['loop_search_maximum_distance'] == 8.0
    assert ros['mode'] == 'mapping'
    assert ros['use_sim_time'] is True
    assert ros['odom_frame'] == 'odom_imu'
    assert 'qos_overrides' not in ros
    assert 'start_type_description_service' not in ros


PROFILE_FLAT = 'loop_search_maximum_distance: 3.0\n'
PROFILE_WRAPPED = '/slam_toolbox:\n  ros__parameters:\n    loop_search_maximum_distance: 3.0\n'


@pytest.mark.parametrize('text', [PROFILE_FLAT, PROFILE_WRAPPED])
def test_profile_flat_and_recorded_shapes_give_the_same_overrides(tmp_path, text):
    # oracle: closed form - the 3.0 written in the profile is the value the plan must carry.
    run = make_run(tmp_path)
    profile = tmp_path / 'profile.yaml'
    profile.write_text(text, encoding='utf-8')
    plan = slam_replay.plan_replay(run, 'odom_imu', profile, tmp_path / 'out')
    assert plan.profile == {'loop_search_maximum_distance': 3.0}
    assert plan.params['slam_toolbox']['ros__parameters']['loop_search_maximum_distance'] == 3.0


@pytest.mark.parametrize('text, message', [
    ('- 1\n- 2\n', 'profile .* is not a parameter mapping'),  # a list
    ('just a string\n', 'profile .* is not a parameter mapping'),  # a scalar
    ('loop_search_maximum_distance: [3.0\n', 'profile .* is unreadable'),  # truncated YAML
])
def test_profile_that_is_not_a_parameter_mapping_is_refused(tmp_path, text, message):
    run = make_run(tmp_path)
    profile = tmp_path / 'profile.yaml'
    profile.write_text(text, encoding='utf-8')
    with pytest.raises(ReplayRefused, match=message):
        slam_replay.plan_replay(run, 'odom', profile, tmp_path / 'out')


# --- main: the refusals (the orchestration needs a live ROS graph and is smoke-tested) ----------

def _trap_popen(monkeypatch):
    """Any child process started is a failure: this stands in for the process boundary."""
    def trap(*args, **kwargs):
        raise AssertionError(f'a child process was started: {args[0]!r}')
    monkeypatch.setattr(subprocess, 'Popen', trap)


def test_main_refuses_while_drivers_run(tmp_path, monkeypatch, capsys):
    # The brief's test, on a fake /proc that holds a live urg_node. The run itself is valid, so
    # without the interlock main would go on to start processes, and the trap would fail the test.
    proc = tmp_path / 'proc'
    proc.mkdir()
    _proc(proc, '4242', URG)
    monkeypatch.setattr(slam_replay, 'PROC_ROOT', proc)
    monkeypatch.setattr(slam_replay, 'RUNS_ROOT', tmp_path / 'racerbot_runs')
    _trap_popen(monkeypatch)
    run = make_run(tmp_path / 'src')
    assert slam_replay.main([str(run), '--odom', 'odom_imu']) == 2
    assert 'urg_node' in capsys.readouterr().err
    assert not (tmp_path / 'racerbot_runs').exists()


def test_main_refuses_a_run_that_cannot_be_replayed(tmp_path, monkeypatch, capsys):
    # With no driver running, a run without /odom/imu is refused with code 2, and no folder is made.
    monkeypatch.setattr(slam_replay, 'PROC_ROOT', tmp_path / 'empty_proc')
    (tmp_path / 'empty_proc').mkdir()
    monkeypatch.setattr(slam_replay, 'RUNS_ROOT', tmp_path / 'racerbot_runs')
    _trap_popen(monkeypatch)
    run = make_run(tmp_path / 'src', topics=['/scan', '/tf_static', '/odom'])
    assert slam_replay.main([str(run), '--odom', 'odom_imu']) == 2
    assert '/odom/imu' in capsys.readouterr().err
    assert not (tmp_path / 'racerbot_runs').exists()


@pytest.mark.parametrize('rate', ['0', '-1', 'nan', 'fast'])
def test_main_rejects_a_rate_that_is_not_a_positive_number(rate):
    with pytest.raises(SystemExit) as exc:
        slam_replay.main(['unused-run-dir', '--rate', rate])
    assert exc.value.code == 2


# --- save_with_retries (fix round 1) -------------------------------------------------------------
# The fault: in one replay in four of a tuning batch, slam_toolbox's map saver answered
# /slam_toolbox/save_map with success and wrote no map.yaml. The replay now tries up to three
# times, two seconds apart, and asks the disk after every try. Oracles: the spec of that round
# (three tries, two seconds apart, map.yaml decides) and the counts the spec implies.

def _fake_saver(map_yaml: Path, outcomes):
    """A save() whose n-th call does outcomes[n - 1]. The outcomes are 'write', 'nothing' (answers
    success, writes no file), 'fail' (raises ServiceError), 'write-then-fail' and 'child-dies'."""
    calls = []

    def save():
        calls.append(len(calls) + 1)
        outcome = outcomes[len(calls) - 1]
        if outcome in ('write', 'write-then-fail'):
            map_yaml.write_text('image: map.pgm\n', encoding='utf-8')
        if outcome in ('fail', 'write-then-fail'):
            raise ServiceError('/slam_toolbox/save_map did not answer')
        if outcome == 'child-dies':
            raise ReplayFailed('slam_toolbox exited with -6; see slam_toolbox.log')

    return save, calls


def test_save_stops_at_the_first_attempt_that_writes_the_map(tmp_path):
    map_yaml = tmp_path / 'map.yaml'
    save, calls = _fake_saver(map_yaml, ['write'])
    sleeps, notes = [], []
    attempt = slam_replay.save_with_retries(save, lambda: map_yaml.is_file(),
                                            sleep=sleeps.append, note=notes.append)
    assert attempt == 1
    assert calls == [1]
    assert sleeps == []  # no pause before the first try, and none once it worked
    assert notes == []


def test_save_retries_after_a_failed_call_two_seconds_later(tmp_path):
    # oracle: spec - up to three tries, two seconds apart; the second try writes the map.
    map_yaml = tmp_path / 'map.yaml'
    save, calls = _fake_saver(map_yaml, ['fail', 'write'])
    sleeps, notes = [], []
    attempt = slam_replay.save_with_retries(save, lambda: map_yaml.is_file(),
                                            sleep=sleeps.append, note=notes.append)
    assert attempt == 2
    assert calls == [1, 2]
    assert sleeps == [2.0]  # oracle: spec, two seconds
    assert notes == ['save attempt 1 of 3 failed: /slam_toolbox/save_map did not answer']


def test_a_save_that_answers_but_writes_nothing_fails_every_try(tmp_path):
    # oracle: spec - the observed fault. Three tries in all, two pauses between them, none after the last.
    map_yaml = tmp_path / 'map.yaml'
    save, calls = _fake_saver(map_yaml, ['nothing', 'nothing', 'nothing'])
    sleeps, notes = [], []
    attempt = slam_replay.save_with_retries(save, lambda: map_yaml.is_file(),
                                            sleep=sleeps.append, note=notes.append)
    assert attempt is None
    assert calls == [1, 2, 3]
    assert sleeps == [2.0, 2.0]
    assert notes == [f'save attempt {n} of 3 failed: it answered, but map.yaml is missing'
                     for n in (1, 2, 3)]


def test_the_file_decides_not_the_call(tmp_path):
    # invariant - a save that writes map.yaml and then reports an error has still saved the map.
    map_yaml = tmp_path / 'map.yaml'
    save, calls = _fake_saver(map_yaml, ['write-then-fail'])
    sleeps, notes = [], []
    attempt = slam_replay.save_with_retries(save, lambda: map_yaml.is_file(),
                                            sleep=sleeps.append, note=notes.append)
    assert attempt == 1
    assert calls == [1]
    assert sleeps == []
    assert notes == []


def test_a_dead_child_is_not_retried(tmp_path):
    # A ReplayFailed means a child has died: a real failure, and retrying it would only wait out timeouts.
    map_yaml = tmp_path / 'map.yaml'
    save, calls = _fake_saver(map_yaml, ['child-dies'])
    sleeps = []
    with pytest.raises(ReplayFailed, match='exited with -6'):
        slam_replay.save_with_retries(save, lambda: map_yaml.is_file(), sleep=sleeps.append)
    assert calls == [1]
    assert sleeps == []


def test_save_needs_at_least_one_attempt():
    with pytest.raises(ValueError, match='at least 1'):
        slam_replay.save_with_retries(lambda: None, lambda: False, attempts=0)


def test_service_result_reads_the_code_the_answer_carries():
    # oracle: spec - slam_toolbox's srv/SaveMap.srv: RESULT_SUCCESS 0, RESULT_NO_MAP_RECEIEVD 1,
    # RESULT_UNDEFINED_FAILURE 255. The answer is the shape `ros2 service call` printed in the fixcheck replay.
    answer = ('waiting for service to become available...\n'
              "requester: making request: slam_toolbox.srv.SaveMap_Request(name=std_msgs.msg.String(data='/x/map'))\n"
              '\n'
              'response:\n'
              'slam_toolbox.srv.SaveMap_Response(result=255)\n')
    assert slam_replay.service_result(answer) == 255
    assert slam_replay.service_result(answer.replace('result=255', 'result=0')) == 0
    assert slam_replay.service_result(answer.replace('result=255', 'result=1')) == 1


def test_service_result_is_none_when_the_answer_has_no_response_line():
    # invariant - an answer with no response line carries no code; the caller then reads the exit code alone.
    assert slam_replay.service_result('waiting for service to become available...\n') is None
    assert slam_replay.service_result('') is None


# --- _service_call's reading of an answer. subprocess.run is the process boundary: the ros2 CLI.

def _ros2_answers(monkeypatch, stdout: str, returncode: int = 0):
    def fake_run(argv, **kwargs):
        return subprocess.CompletedProcess(argv, returncode, stdout=stdout, stderr='')
    monkeypatch.setattr(subprocess, 'run', fake_run)


SAVE_REQUEST = {'name': {'data': '/x/map'}}


def test_a_save_answer_with_result_255_is_a_service_error(monkeypatch):
    _ros2_answers(monkeypatch, 'response:\nslam_toolbox.srv.SaveMap_Response(result=255)\n')
    with pytest.raises(ServiceError, match='answered with result 255'):
        slam_replay._service_call(slam_replay.SAVE_MAP, SAVE_REQUEST, {}, lambda message: None)


def test_a_save_answer_with_result_0_is_logged_and_accepted(monkeypatch):
    _ros2_answers(monkeypatch, 'response:\nslam_toolbox.srv.SaveMap_Response(result=0)\n')
    lines = []
    slam_replay._service_call(slam_replay.SAVE_MAP, SAVE_REQUEST, {}, lines.append)
    assert lines == ['/slam_toolbox/save_map: exit 0; result 0']


def test_a_call_that_exits_nonzero_is_a_service_error(monkeypatch):
    _ros2_answers(monkeypatch, 'waiting for service to become available...\n', returncode=1)
    with pytest.raises(ServiceError, match='failed with exit 1'):
        slam_replay._service_call(slam_replay.SAVE_MAP, SAVE_REQUEST, {}, lambda message: None)


def test_a_call_that_never_answers_is_a_service_error(monkeypatch):
    def hang(argv, **kwargs):
        raise subprocess.TimeoutExpired(argv, 60.0)
    monkeypatch.setattr(subprocess, 'run', hang)
    with pytest.raises(ServiceError, match='did not answer'):
        slam_replay._service_call(slam_replay.SAVE_MAP, SAVE_REQUEST, {}, lambda message: None)
