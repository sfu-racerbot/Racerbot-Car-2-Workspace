"""Run with pytest tools/racerbot_sim/tests/; no ROS nodes are required."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_gap_follow_validation as validation


def test_arguments_accept_repeated_launch_options_and_known_fault():
    args = validation.parse_args(['--gap-follow-arg', 'fused_speed:=true',
                                  '--gap-follow-arg', 'imu_traction:=true',
                                  '--imu-fault', 'stale', '--fault-at', '40'])
    # oracle: command-line specification
    assert args.gap_follow_arg == ['fused_speed:=true', 'imu_traction:=true']
    assert args.imu_fault == 'stale' and args.fault_at == 40


def test_unknown_fault_is_rejected_before_run():
    # oracle: sim_imu.FAULTS does not contain this name.
    with pytest.raises(SystemExit) as exc:
        validation.parse_args(['--imu-fault', 'does_not_exist', '--fault-at', '40'])
    assert exc.value.code == 2


def test_fraction_uses_only_messages_after_settle():
    # A6: /home/bryan/racerbot-ws/.oc-runs/part-b23-slam-imu/task-I2b-answer-1.md
    # corrects the old denominator. IntentThrottle in
    # src/drive_intent/drive_intent/throttle.py sends reasons only on state
    # changes and periodically; counting all messages was therefore wrong.
    log = [(39.0, 'steering_damping_off: imu stale'),
           (40.5, 'steering_damping_off: imu stale'), (41.0, 'other reason')]
    # oracle: two eligible reason-bearing messages, one names the fallback.
    assert validation.fallback_fractions(log, 40.0, 'stale')['steering_damping'] == 0.5


def test_empty_and_fault_after_end_have_no_fraction():
    # oracle: no eligible observations cannot establish a fraction.
    assert validation.fallback_fractions([], 40, 'stale')['steering_damping'] is None
    assert validation.fallback_fractions([(1, '')], 40, 'stale')['steering_damping'] is None


def test_speed_summary_rejects_nonfinite_samples():
    # oracle: finite arithmetic means, with nonfinite samples excluded.
    result = validation.speed_summary([(1, 2.0, 3.0), (2, float('nan'), 5.0),
                                       (3, 4.0, float('nan'))], 2)
    assert result['before']['mean_speed'] == 2.0
    assert result['after']['mean_speed'] == 4.0
    assert result['after']['mean_commanded_speed'] == 5.0


def test_fault_evidence_includes_later_and_last_imu_sample():
    # oracle: a fault may take effect after the parameter request completes;
    # a stale stream must expose its final sample and zero must show later zeros.
    from types import SimpleNamespace
    rec = SimpleNamespace(intent=[], odom=[], sim=[], status=[], imu=[
        (139.9, 9.8, 0.2), (140.1, 9.8, 0.3),
        (141.1, 0.0, 0.0), (150.0, 0.0, 0.0)])
    evidence = validation.recorder_report(rec, 100.0, 40.0, 'zero')['imu_evidence']
    assert [41.1, 0.0, 0.0] in evidence
    assert [50.0, 0.0, 0.0] in evidence


def test_evidence_does_not_stop_at_first_three_post_fault_samples():
    # oracle: after a dense stream of pre-transition samples, the output must
    # still show the later zero reading and last available sample.
    from types import SimpleNamespace
    rec = SimpleNamespace(intent=[], odom=[], sim=[], status=[], imu=[
        (139.9, 9.8, 0.2), (140.1, 9.8, 0.2), (140.2, 9.8, 0.2),
        (140.3, 9.8, 0.2), (141.1, 0.0, 0.0), (150.0, 0.0, 0.0)])
    evidence = validation.recorder_report(rec, 100.0, 40.0, 'zero')['imu_evidence']
    assert [41.1, 0.0, 0.0] in evidence
    assert [50.0, 0.0, 0.0] in evidence


def test_fraction_ignores_reasonless_but_counts_other_reasons():
    # oracle: the revised brief counts non-empty reasons only after settling.
    log = [(40.1, 'early reason'), (40.5, 'steering_damping_off: imu stale'),
           (40.6, None), (40.7, ''), (40.8, 'ordinary driving')]
    assert validation.fallback_fractions(log, 40, 'stale')['steering_damping'] == 0.5


def test_reason_report_first_and_missing_examples():
    # oracle: first reason is measured from injection, not from settling;
    # missing examples include only post-settle, non-empty reasons.
    log = [(40.1, 'early reason'), (40.5, 'steering_damping_off: imu stale'),
           (40.6, None), (40.7, ''), (40.8, 'ordinary driving'),
           (41.0, 'another reason'), (41.2, 'third'), (41.4, 'fourth')]
    report = validation.reason_report(log, 40, 'stale')
    assert report['reason_messages_after_settle'] == 5
    assert report['first_reason_after_fault'] == {'time_s': 40.1, 'text': 'early reason'}
    assert report['missing_fallback']['steering_damping'] == {
        'count': 4, 'first_three': ['ordinary driving', 'another reason', 'third']}


def test_only_empty_reasons_cannot_establish_fraction():
    # oracle: no non-empty post-settle reasons means no observations.
    assert validation.fallback_fractions([(41, None), (42, '')], 40, 'stale')[
        'steering_damping'] is None


def test_injection_guard_requires_owned_unique_process_and_clean_graph():
    # oracle: review fix round 1 requires a unique descendant and no hardware.
    parents = {21: 15, 15: 10, 99: 1}
    assert validation.injection_guard([], [21], 10, parents) is None
    assert 'hardware' in validation.injection_guard(['joy'], [21], 10, parents)
    assert 'graph' in validation.injection_guard(None, [21], 10, parents)
    assert 'exactly one' in validation.injection_guard([], [], 10, parents)
    assert 'exactly one' in validation.injection_guard([], [21, 99], 10, parents)
    assert 'descendant' in validation.injection_guard([], [99], 10, parents)
    assert 'descendant' in validation.injection_guard([], [21], 10, {21: 15})
    assert 'descendant' in validation.injection_guard([], [21], 10, {21: 15, 15: 21})


def test_proc_snapshot_parses_parent_and_bridge_executable(tmp_path):
    # oracle: Linux proc stat field 4 is ppid; comm may contain spaces and ')'.
    for pid, ppid, command in [(12, 7, b'/usr/bin/python3\0/opt/gym_bridge_node\0'),
                               (13, 1, b'/usr/bin/python3\0/tmp/not_gym_bridge_node\0')]:
        proc = tmp_path / str(pid)
        proc.mkdir()
        (proc / 'stat').write_text(f'{pid} (name ) with spaces) S {ppid} 0 0\n')
        (proc / 'cmdline').write_bytes(command)
    bridges, parents = validation.process_snapshot(tmp_path)
    assert bridges == [12]
    assert parents == {12: 7, 13: 1}


@pytest.mark.parametrize('time_arg', ['0', '-1', '10', '11', 'nan', 'inf'])
def test_fault_time_must_be_inside_driving_window(time_arg):
    # oracle: review fix round 1 requires 0 < fault-at < duration.
    with pytest.raises(SystemExit) as exc:
        validation.parse_args(['--duration', '10', '--fault-at', time_arg, '--imu-fault', 'zero'])
    assert exc.value.code == 2


def test_missed_fault_is_failure_and_no_requested_split_is_used():
    # oracle: a requested but unacknowledged injection cannot pass or be a split.
    assert 'not acknowledged' in validation.injection_failure('zero', None, None)
    assert validation.injection_failure('zero', 4.2, None) is None
    assert validation.injection_failure(None, None, None) is None
    assert validation.injection_failure('zero', None, 'launch crashed') == 'launch crashed'
    assert validation.validation_split('zero', 4.0, None) is None
    assert validation.validation_split('zero', 4.0, 4.2) == 4.2
    assert validation.validation_split(None, 4.0, None) == 4.0


def test_parameter_write_failures_are_returned(monkeypatch):
    import subprocess
    # oracle: every parameter-client failure becomes a named injection failure.
    def timed_out(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], 15)
    monkeypatch.setattr(subprocess, 'run', timed_out)
    assert 'timed out' in validation.write_fault('stale', {'ROS_DOMAIN_ID': '64'})
    def unavailable(*args, **kwargs):
        raise OSError('client missing')
    monkeypatch.setattr(subprocess, 'run', unavailable)
    assert 'client missing' in validation.write_fault('stale', {'ROS_DOMAIN_ID': '64'})


def test_parameter_client_uses_given_domain_and_requires_success(monkeypatch):
    import subprocess
    from types import SimpleNamespace
    # oracle: the client uses the harness domain and must acknowledge the set.
    calls = []
    def run_client(command, **kwargs):
        calls.append((command, kwargs['env']))
        return SimpleNamespace(returncode=0, stdout='Set parameter successful', stderr='')
    monkeypatch.setattr(subprocess, 'run', run_client)
    assert validation.write_fault('zero', {'ROS_DOMAIN_ID': '64'}) is None
    assert calls == [(['ros2', 'param', 'set', '/gym_bridge_node', 'imu_fault', 'zero'],
                      {'ROS_DOMAIN_ID': '64'})]
    def rejected(*args, **kwargs):
        return SimpleNamespace(returncode=0, stdout='Set parameter failed', stderr='')
    monkeypatch.setattr(subprocess, 'run', rejected)
    assert 'failed' in validation.write_fault('zero', {'ROS_DOMAIN_ID': '64'})


def test_injection_target_refuses_hardware_and_unreadable_graph():
    # oracle: HardwareInterlock rejects any namespaced real hardware or unreadable graph.
    class Graph:
        def __init__(self, nodes):
            self.nodes = nodes
        def get_node_names_and_namespaces(self):
            if self.nodes is None:
                raise RuntimeError('graph unavailable')
            return self.nodes
    assert 'hardware' in validation.check_injection_target(
        Graph([('joy', '/car')]), 123)
    assert 'hardware' in validation.check_injection_target(
        Graph([('vesc_driver_node', '/'), ('urg_node', '/')]), 123)
    assert 'graph' in validation.check_injection_target(Graph(None), 123)


def test_owned_injection_refuses_before_parameter_write(tmp_path, monkeypatch):
    import subprocess
    from types import SimpleNamespace
    # oracle: the write must occur only for the unique owned bridge and clean graph.
    proc = tmp_path / '21'
    proc.mkdir()
    (proc / 'stat').write_text('21 (bridge) S 10 0 0\n')
    (proc / 'cmdline').write_bytes(b'/usr/bin/python3\0/opt/gym_bridge_node\0')
    class Graph:
        nodes = [('gym_bridge_node', '/')]
        def get_node_names_and_namespaces(self):
            return self.nodes
    graph = Graph()
    calls = []
    def client(cmd, **kwargs):
        calls.append((cmd, kwargs['env']['ROS_DOMAIN_ID']))
        return SimpleNamespace(returncode=0, stdout='Set parameter successful', stderr='')
    monkeypatch.setattr(subprocess, 'run', client)
    assert validation.inject_fault(graph, 10, 'zero', {'ROS_DOMAIN_ID': '64'}, tmp_path) is None
    assert len(calls) == 1 and calls[0][1] == '64'
    graph.nodes.append(('joy', '/car'))
    assert 'hardware' in validation.inject_fault(graph, 10, 'zero', {}, tmp_path)
    assert len(calls) == 1
    graph.nodes = [('gym_bridge_node', '/'), ('gym_bridge_node', '/other')]
    assert 'exactly one root' in validation.inject_fault(graph, 10, 'zero', {}, tmp_path)
    assert len(calls) == 1
    graph.nodes = [('gym_bridge_node', '/')]
    assert 'descendant' in validation.inject_fault(graph, 11, 'zero', {}, tmp_path)
    assert len(calls) == 1


def test_injection_environment_uses_localhost_and_harness_domain():
    # oracle: review fix round 2 requires a localhost client on the harness domain.
    harness = {'ROS_DOMAIN_ID': '64', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET'}
    client = validation.injection_environment(harness)
    assert client['ROS_DOMAIN_ID'] == '64'
    assert client['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert harness['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SUBNET'


@pytest.mark.parametrize('discovery', [None, 'SUBNET'])
def test_injection_refuses_nonlocal_harness_discovery(discovery):
    # oracle: review fix round 2 prohibits injection unless the harness is local.
    harness = {'ROS_DOMAIN_ID': '64'}
    if discovery is not None:
        harness['ROS_AUTOMATIC_DISCOVERY_RANGE'] = discovery
    client = {'ROS_DOMAIN_ID': '64', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    assert 'LOCALHOST' in validation.inject_with_environment(
        object(), 10, 'zero', harness, client)


def test_injection_refuses_domain_mismatch():
    # oracle: review fix round 2 binds the client to the harness's ROS domain.
    harness = {'ROS_DOMAIN_ID': '64', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    client = {'ROS_DOMAIN_ID': '65', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    assert 'domain' in validation.inject_with_environment(
        object(), 10, 'zero', harness, client).lower()


def test_injection_requires_local_client_even_if_harness_is_local():
    # oracle: both participants must use LOCALHOST, including a default domain.
    harness = {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    client = {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET'}
    assert 'LOCALHOST' in validation.injection_environment_refusal(harness, client)
    client['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    assert validation.injection_environment_refusal(harness, client) is None
    assert validation.injection_environment(harness)['ROS_DOMAIN_ID'] == '0'


def test_injection_refuses_harness_static_peers():
    # oracle: review fix round 3 refuses any configured harness static peers.
    harness = {'ROS_DOMAIN_ID': '64', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
               'ROS_STATIC_PEERS': '10.0.0.5'}
    client = {'ROS_DOMAIN_ID': '64', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    assert 'ROS_STATIC_PEERS' in validation.inject_with_environment(
        object(), 10, 'zero', harness, client)


@pytest.mark.parametrize('peers', [None, ''])
def test_empty_or_unset_harness_static_peers_are_allowed(peers):
    # oracle: review fix round 3 permits an empty or absent peer list.
    harness = {'ROS_DOMAIN_ID': '64', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    if peers is not None:
        harness['ROS_STATIC_PEERS'] = peers
    client = {'ROS_DOMAIN_ID': '64', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    assert validation.injection_environment_refusal(harness, client) is None


def test_injection_client_drops_harness_static_peers():
    # oracle: review fix round 3 requires the client to have no static peers.
    harness = {'ROS_DOMAIN_ID': '64', 'ROS_STATIC_PEERS': '10.0.0.5'}
    client = validation.injection_environment(harness)
    assert 'ROS_STATIC_PEERS' not in client
    assert harness['ROS_STATIC_PEERS'] == '10.0.0.5'


def test_harness_selects_the_simulated_imu_for_gap_follow():
    # oracle: spec (wrap-up M4): the simulator harness never leaves the IMU
    # source to the auto probe, which can race the simulator's start-up.
    assert validation.gap_follow_launch_args([]) == ['imu_hardware:=false']
    assert validation.gap_follow_launch_args(None) == ['imu_hardware:=false']
    assert validation.gap_follow_launch_args(['imu_all:=true']) == [
        'imu_all:=true', 'imu_hardware:=false']


def test_an_explicit_imu_hardware_argument_wins():
    # oracle: spec, the docstring: an explicit imu_hardware:= is kept as given.
    assert validation.gap_follow_launch_args(['imu_hardware:=true']) == ['imu_hardware:=true']
