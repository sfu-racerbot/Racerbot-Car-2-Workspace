"""Tests for proccontrol.py -- process discovery and the stop escalation.

Everything here runs against a fake /proc tree built in a tmpdir and a
fake signal sender that records calls instead of making them. No test in
this file signals a real process, and none of them need ROS, a browser, or
a car.

The tests that matter most are the refusals. A bug that makes the stop
button miss a stale pure_pursuit is annoying; a bug that lets it kill
ackermann_mux while the car is moving is the one that hurts someone.
"""

import json
import os
import signal
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from web_dashboard import proccontrol, protocol  # noqa: E402


# ---------------------------------------------------------------------------
# fake /proc
# ---------------------------------------------------------------------------

def make_proc(tmp_path, processes, states=None):
    """Build a /proc-shaped tree.

    `processes` maps pid -> argv list, or pid -> (argv, ppid).
    `states` optionally maps pid -> the single-letter process state
    ('Z' for a zombie); anything unlisted is 'S', a normal sleeping task.
    """
    states = states or {}
    root = tmp_path / 'proc'
    root.mkdir(exist_ok=True)
    for pid, spec in processes.items():
        argv, ppid = (spec, 1) if isinstance(spec, list) else spec
        entry = root / str(pid)
        entry.mkdir(exist_ok=True)
        (entry / 'cmdline').write_bytes(
            b'\0'.join(a.encode() for a in argv) + b'\0' if argv else b'')
        state = states.get(pid, 'S')
        (entry / 'status').write_text(
            f'Name:\ttest\nState:\t{state} (test)\nPPid:\t{ppid}\n')
    # Non-numeric entries the real /proc is full of.
    (root / 'meminfo').write_text('MemTotal: 1 kB\n')
    return str(root)


# The realistic cmdline for an installed ament_python node: a setuptools
# console-script run through the interpreter, named with no .py extension.
# Taken from a real `install/pure_pursuit/lib/pure_pursuit/pure_pursuit_node`
# -- an earlier version of classify() required a .py here and so found
# nothing at all on a real car.
PP_NODE = ['/usr/bin/python3',
           '/home/x/install/pure_pursuit/lib/pure_pursuit/pure_pursuit_node',
           '--ros-args', '--params-file', '/home/x/pp.yaml']
MUX_NODE = ['/opt/ros/jazzy/lib/ackermann_mux/ackermann_mux_node', '--ros-args']
VESC_NODE = ['/opt/ros/jazzy/lib/vesc_driver/vesc_driver_node', '--ros-args']
PP_LAUNCH = ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'launch',
             'pure_pursuit', 'pure_pursuit_launch.py']
BRINGUP_LAUNCH = ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'launch',
                  'f1tenth_stack', 'bringup_launch.py']


# ---------------------------------------------------------------------------
# classify
# ---------------------------------------------------------------------------

def test_classify_installed_node_executable():
    assert proccontrol.classify(PP_NODE) == ('node', 'pure_pursuit_node')


def test_classify_ros2_launch_names_the_launch_file():
    assert proccontrol.classify(PP_LAUNCH) == ('launch', 'pure_pursuit_launch.py')


def test_classify_ros2_run_names_the_executable():
    argv = ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'run',
            'gap_follow', 'gap_follow_node']
    assert proccontrol.classify(argv) == ('node', 'gap_follow_node')


def test_classify_prefers_an_explicit_node_remap():
    """A remapped node answers to the remapped name on the graph, so that
    is the name the allowlist has to be checked against."""
    argv = ['/home/x/install/gap_follow/lib/gap_follow/gap_follow_node',
            '--ros-args', '-r', '__node:=my_renamed_node']
    assert proccontrol.classify(argv) == ('node', 'my_renamed_node')


def test_classify_looks_past_a_python_interpreter():
    argv = ['/usr/bin/python3', '/home/x/scratch/wall_follow_node.py']
    assert proccontrol.classify(argv) == ('node', 'wall_follow_node.py')


def test_classify_finds_an_extensionless_console_script():
    """Regression. An installed ament_python node is a shebang script
    named `pure_pursuit_node`, with no .py -- requiring an extension here
    made every Python node in this workspace invisible to the scan."""
    argv = ['/usr/bin/python3',
            '/home/x/install/gap_follow/lib/gap_follow/gap_follow_node']
    assert proccontrol.classify(argv) == ('node', 'gap_follow_node')


def test_classify_skips_interpreter_flags():
    argv = ['/usr/bin/python3', '-u',
            '/home/x/install/pure_pursuit/lib/pure_pursuit/pure_pursuit_node']
    assert proccontrol.classify(argv) == ('node', 'pure_pursuit_node')


def test_classify_ignores_junk():
    assert proccontrol.classify([]) == (None, '')
    assert proccontrol.classify(['/usr/bin/python3']) == (None, '')


# ---------------------------------------------------------------------------
# the protected set -- the tests that actually matter
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('name', [
    'ackermann_mux', 'vesc_driver_node', 'joy_node', 'joy_teleop',
    'bringup_launch.py', 'teleop_launch.py', 'dashboard_node',
])
def test_actuation_path_is_protected_by_name(name):
    assert proccontrol.is_protected(name) is True


def test_protection_survives_a_node_suffix_mismatch():
    """`ackermann_mux` is the package; the executable is
    `ackermann_mux_node`. Both have to be refused."""
    assert proccontrol.is_protected('ackermann_mux_node', 'node', MUX_NODE)


def test_driving_nodes_are_not_protected():
    assert proccontrol.is_protected('pure_pursuit_node') is False
    assert proccontrol.is_protected('gap_follow_node') is False


def test_sanitize_allowlist_strips_protected_entries():
    kept = proccontrol.sanitize_allowlist(
        ['pure_pursuit_node', 'ackermann_mux', 'vesc_driver_node', 'ftg_node'])
    assert kept == ['pure_pursuit_node', 'ftg_node']


def test_sanitize_allowlist_warns_about_refusals():
    warnings = []

    class Logger:
        def warn(self, message):
            warnings.append(message)

    proccontrol.sanitize_allowlist(['joy_node', 'pure_pursuit_node'], Logger())
    assert len(warnings) == 1
    assert 'joy_node' in warnings[0]


def test_sanitize_allowlist_dedupes_and_drops_blanks():
    kept = proccontrol.sanitize_allowlist(
        ['ftg_node', '  ', 'ftg_node', ' safety_node '])
    assert kept == ['ftg_node', 'safety_node']


# ---------------------------------------------------------------------------
# scan
# ---------------------------------------------------------------------------

def test_scan_finds_a_driving_node_and_marks_it_stoppable(tmp_path):
    root = make_proc(tmp_path, {100: PP_NODE})
    targets = proccontrol.scan(root, self_pid=999, uid=os.getuid())
    assert len(targets) == 1
    assert targets[0].pid == 100
    assert targets[0].name == 'pure_pursuit_node'
    assert targets[0].protected is False


def test_scan_reports_the_actuation_path_but_refuses_it(tmp_path):
    root = make_proc(tmp_path, {100: PP_NODE, 101: MUX_NODE, 102: VESC_NODE})
    targets = proccontrol.scan(root, self_pid=999, uid=os.getuid())
    verdicts = {t.name: t.protected for t in targets}
    assert verdicts['pure_pursuit_node'] is False
    assert verdicts['ackermann_mux_node'] is True
    assert verdicts['vesc_driver_node'] is True


def test_scan_refuses_bringup_launch_even_though_it_is_a_launch(tmp_path):
    root = make_proc(tmp_path, {100: BRINGUP_LAUNCH, 101: PP_LAUNCH})
    targets = proccontrol.scan(root, self_pid=999, uid=os.getuid())
    verdicts = {t.name: t.protected for t in targets}
    assert verdicts['bringup_launch.py'] is True
    assert verdicts['pure_pursuit_launch.py'] is False


def test_scan_ignores_processes_that_are_not_ours(tmp_path):
    root = make_proc(tmp_path, {
        100: ['/usr/bin/firefox'],
        101: ['/usr/lib/systemd/systemd-journald'],
        102: PP_NODE,
    })
    targets = proccontrol.scan(root, self_pid=999, uid=os.getuid())
    assert [t.pid for t in targets] == [102]


def test_scan_will_not_offer_to_kill_the_dashboard_or_its_parents(tmp_path):
    """pid 100 is the dashboard's own ancestor chain. Even though the
    cmdline matches the allowlist, it must come back protected."""
    root = make_proc(tmp_path, {
        100: (PP_NODE, 1),
        200: (['/home/x/install/web_dashboard/lib/web_dashboard/x'], 100),
    })
    targets = proccontrol.scan(root, self_pid=200, uid=os.getuid())
    found = proccontrol.find(targets, 100)
    assert found.protected is True
    assert 'dashboard' in found.reason


def test_scan_refuses_processes_owned_by_another_user(tmp_path):
    root = make_proc(tmp_path, {100: PP_NODE})
    targets = proccontrol.scan(root, self_pid=999, uid=os.getuid() + 4242)
    assert targets[0].protected is True
    assert 'another user' in targets[0].reason


def test_scan_honours_a_custom_allowlist(tmp_path):
    root = make_proc(tmp_path, {100: PP_NODE})
    targets = proccontrol.scan(root, allowlist=('something_else',),
                               self_pid=999, uid=os.getuid())
    assert targets == []


def test_scan_survives_a_missing_proc_root():
    assert proccontrol.scan('/nonexistent-proc-root') == []


def test_scan_skips_kernel_threads_with_empty_cmdlines(tmp_path):
    root = make_proc(tmp_path, {100: [], 101: PP_NODE})
    targets = proccontrol.scan(root, self_pid=999, uid=os.getuid())
    assert [t.pid for t in targets] == [101]


# ---------------------------------------------------------------------------
# StopJob escalation
# ---------------------------------------------------------------------------

class FakeProcess:
    """A process that dies after `dies_after` signals (None = never)."""

    def __init__(self, dies_after=None):
        self.dies_after = dies_after
        self.signals = []
        self.dead = False

    def send(self, pid, signum):
        if self.dead:
            raise ProcessLookupError(pid)
        if signum == 0:
            return
        self.signals.append(signum)
        if self.dies_after is not None and len(self.signals) >= self.dies_after:
            self.dead = True


def test_stop_job_starts_with_sigint_like_ctrl_c():
    proc = FakeProcess(dies_after=1)
    job = proccontrol.StopJob(100, 'pure_pursuit_node', grace_sec=2.0, now=0.0)
    job.advance(0.0, proc.send)
    assert proc.signals == [signal.SIGINT]
    assert job.sent == ['SIGINT']


def test_stop_job_reports_success_once_the_process_is_gone():
    proc = FakeProcess(dies_after=1)
    job = proccontrol.StopJob(100, 'pure_pursuit_node', grace_sec=2.0, now=0.0)
    job.advance(0.0, proc.send)
    job.advance(2.5, proc.send)
    assert job.done is True
    assert job.ok is True
    assert job.detail == 'stopped'


def test_stop_job_escalates_when_ctrl_c_is_ignored():
    """The reported bug: SIGINT does nothing. Escalate rather than sulk."""
    proc = FakeProcess(dies_after=None)
    job = proccontrol.StopJob(100, 'stuck_node', grace_sec=2.0, now=0.0)
    job.advance(0.0, proc.send)
    job.advance(2.5, proc.send)
    job.advance(5.0, proc.send)
    assert proc.signals == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]


def test_stop_job_waits_out_the_grace_period_before_escalating():
    proc = FakeProcess(dies_after=None)
    job = proccontrol.StopJob(100, 'stuck_node', grace_sec=2.0, now=0.0)
    job.advance(0.0, proc.send)
    changed = job.advance(1.0, proc.send)      # still inside the grace period
    assert changed is False
    assert proc.signals == [signal.SIGINT]


def test_stop_job_gives_up_after_sigkill_with_an_actionable_message():
    proc = FakeProcess(dies_after=None)
    job = proccontrol.StopJob(100, 'stuck_node', grace_sec=1.0, now=0.0)
    for now in (0.0, 1.5, 3.0, 4.5):
        job.advance(now, proc.send)
    assert job.done is True
    assert job.ok is False
    assert 'reboot' in job.detail


def test_stop_job_on_an_already_dead_pid_is_a_success_not_an_error():
    proc = FakeProcess()
    proc.dead = True
    job = proccontrol.StopJob(100, 'gone_node', grace_sec=1.0, now=0.0)
    job.advance(0.0, proc.send)
    assert job.done is True
    assert job.ok is True
    assert job.detail == 'was already gone'


def test_stop_job_surfaces_a_permission_error():
    def send(pid, signum):
        if signum == 0:
            return
        raise PermissionError(pid)

    job = proccontrol.StopJob(100, 'someone_elses_node', grace_sec=1.0, now=0.0)
    job.advance(0.0, send)
    assert job.done is True
    assert job.ok is False
    assert 'not permitted' in job.detail


def test_stop_job_serialises_for_the_wire():
    proc = FakeProcess(dies_after=1)
    job = proccontrol.StopJob(100, 'pure_pursuit_node', grace_sec=1.0, now=0.0)
    job.advance(0.0, proc.send)
    payload = job.as_dict()
    assert payload['pid'] == 100
    assert payload['name'] == 'pure_pursuit_node'
    assert payload['sent'] == ['SIGINT']


def test_alive_reports_false_for_a_missing_pid():
    def send(pid, signum):
        raise ProcessLookupError(pid)

    assert proccontrol.alive(1234, send) is False


def test_alive_reports_true_when_signalling_is_forbidden():
    def send(pid, signum):
        raise PermissionError(pid)

    assert proccontrol.alive(1234, send) is True


# ---------------------------------------------------------------------------
# zombies
#
# The regression the end-to-end test caught and none of the unit tests
# above could: a killed process whose parent has not reaped it keeps its
# /proc entry and still answers signal 0. Treating that as "alive" made
# the dashboard report "survived SIGINT, SIGTERM, SIGKILL -- try a reboot"
# about a process it had just successfully killed.
#
# It is the common case, not an exotic one: what gets stopped here is
# usually the child of a wedged `ros2 launch`, and a wedged launch is
# exactly a parent that does not reap.
# ---------------------------------------------------------------------------

def test_a_zombie_is_not_alive(tmp_path):
    root = make_proc(tmp_path, {100: PP_NODE}, states={100: 'Z'})

    def send(pid, signum):
        return None       # a zombie answers signal 0 quite happily

    assert proccontrol.alive(100, send, root) is False


def test_a_running_process_is_alive(tmp_path):
    root = make_proc(tmp_path, {100: PP_NODE}, states={100: 'S'})
    assert proccontrol.alive(100, lambda pid, sig: None, root) is True


def test_stop_job_calls_a_zombie_stopped_rather_than_unkillable(tmp_path):
    """The exact end-to-end failure: SIGKILL lands, the process becomes a
    zombie because its parent is wedged, and the job must report success
    instead of telling the user to reboot the car."""
    root = make_proc(tmp_path, {100: PP_NODE}, states={100: 'S'})
    status = tmp_path / 'proc' / '100' / 'status'

    def send(pid, signum):
        # SIGKILL always lands -- but the wedged parent never reaps, so
        # the pid stays in /proc as a zombie rather than disappearing.
        if signum == signal.SIGKILL:
            status.write_text('Name:\ttest\nState:\tZ (zombie)\nPPid:\t1\n')

    job = proccontrol.StopJob(100, 'pure_pursuit_node', grace_sec=1.0,
                              now=0.0, proc_root=root)
    job.advance(0.0, send)        # SIGINT
    job.advance(1.5, send)        # SIGTERM
    job.advance(3.0, send)        # SIGKILL -> now a zombie
    assert 'Z' in status.read_text()

    job.advance(4.5, send)
    assert job.done is True
    assert job.ok is True, job.detail
    assert job.detail == 'stopped'


def test_scan_skips_zombies_entirely(tmp_path):
    """A zombie is not running anything, so it must not be listed as a
    driving process at all -- it has no cmdline, which is how it falls
    out of the scan."""
    root = tmp_path / 'proc'
    root.mkdir()
    entry = root / '100'
    entry.mkdir()
    (entry / 'cmdline').write_bytes(b'')        # zombies have no cmdline
    (entry / 'status').write_text('Name:\ttest\nState:\tZ (zombie)\nPPid:\t1\n')
    assert proccontrol.scan(str(root), self_pid=999, uid=os.getuid()) == []


# ---------------------------------------------------------------------------
# the wire messages
# ---------------------------------------------------------------------------

def test_process_state_message_carries_the_refusals_too(tmp_path):
    """The browser needs the protected rows, not just the stoppable ones:
    "the mux is up and you may not kill it from here" is the useful
    thing to show a person, and a silently shorter list is not."""
    root = make_proc(tmp_path, {100: PP_NODE, 101: MUX_NODE})
    targets = proccontrol.scan(root, self_pid=999, uid=os.getuid())
    message = protocol.process_state_message(targets, True)

    assert message['type'] == 'processes'
    assert message['enabled'] is True
    assert len(message['targets']) == 2
    protections = {t['name']: t['protected'] for t in message['targets']}
    assert protections['pure_pursuit_node'] is False
    assert protections['ackermann_mux_node'] is True


def test_process_state_message_is_json_serialisable(tmp_path):
    """It goes down a WebSocket as JSON -- a Target object that survived
    into the payload would raise at send time, on the rclpy thread."""
    root = make_proc(tmp_path, {100: PP_NODE})
    targets = proccontrol.scan(root, self_pid=999, uid=os.getuid())
    message = protocol.process_state_message(targets, True)

    # Round-trip and assert on what comes back. A bare json.dumps() call is
    # satisfied by None and by {} alike, so it proves only that nothing
    # raised -- not that the message survived encoding intact.
    restored = json.loads(json.dumps(message))
    assert restored['type'] == 'processes'
    assert restored['enabled'] is True
    assert [t['name'] for t in restored['targets']] == ['pure_pursuit_node']


def test_process_result_message_reports_the_escalation():
    message = protocol.process_result_message(
        100, 'pure_pursuit_node', True, 'stopped', ['SIGINT', 'SIGTERM'])
    assert message['type'] == 'process_result'
    assert message['ok'] is True
    assert message['sent'] == ['SIGINT', 'SIGTERM']
    json.dumps(message)


def test_a_disabled_dashboard_still_sends_an_explicit_empty_state():
    """So the browser hides the panel outright, rather than showing an
    empty list that reads as "nothing is running"."""
    message = protocol.process_state_message([], False)
    assert message['enabled'] is False
    assert message['targets'] == []


# ---------------------------------------------------------------------------
# Audit 2026-09-25: H13 (pid reuse mid-escalation), H14 (SLAM-reset gate
# failing open and ignoring protected controllers)
# ---------------------------------------------------------------------------

def _write_stat(root, pid, start_time, comm='pure_pursuit'):
    # Field 3 is the state, 4..21 filler, 22 the start time. comm carries a
    # ') ' inside it, as a real one may.
    filler = ' '.join(['0'] * 18)
    (root / str(pid) / 'stat').write_text(
        f'{pid} ({comm}) x) S {filler} {start_time} 0 0\n')


def test_read_start_time_counts_fields_from_the_last_paren(tmp_path):
    root = tmp_path / 'proc'
    make_proc(tmp_path, {100: PP_NODE})
    filler = ' '.join(str(n) for n in range(3, 22))   # fields 3..21
    (root / '100' / 'stat').write_text(f'100 (a) b) {filler} 987654 23 24\n')
    assert proccontrol.read_start_time(str(root), 100) == 987654


def test_read_start_time_of_a_missing_pid_is_none(tmp_path):
    make_proc(tmp_path, {})
    assert proccontrol.read_start_time(str(tmp_path / 'proc'), 100) is None


def test_stop_job_never_signals_a_recycled_pid(tmp_path):
    """The vetted process exits during the grace period and the kernel
    hands its pid to something else. The next escalation stage must not
    land on the newcomer: same number, different start time."""
    root = make_proc(tmp_path, {100: PP_NODE})
    _write_stat(tmp_path / 'proc', 100, start_time=5000)
    sent = []

    def send(pid, signum):
        if signum:
            sent.append(signum)

    job = proccontrol.StopJob(100, 'pure_pursuit_node', grace_sec=1.0,
                              now=0.0, proc_root=root)
    job.advance(0.0, send)                        # SIGINT to the real one
    assert sent == [signal.SIGINT]

    _write_stat(tmp_path / 'proc', 100, start_time=9999)   # recycled
    job.advance(1.5, send)
    job.advance(3.0, send)
    assert sent == [signal.SIGINT], 'SIGTERM/SIGKILL must not reach the newcomer'
    assert job.done is True
    assert job.ok is True
    assert job.detail == 'stopped'


def test_stop_job_keeps_escalating_on_the_same_process(tmp_path):
    """The permissive half: identical start time is the same process, and
    the pin must not stop a genuine escalation."""
    root = make_proc(tmp_path, {100: PP_NODE})
    _write_stat(tmp_path / 'proc', 100, start_time=5000)
    sent = []
    job = proccontrol.StopJob(100, 'stuck_node', grace_sec=1.0, now=0.0,
                              proc_root=root)
    for now in (0.0, 1.5, 3.0):
        job.advance(now, lambda pid, sig: sent.append(sig) if sig else None)
    assert sent == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]


def test_strict_scan_raises_when_proc_cannot_be_listed(tmp_path):
    missing = str(tmp_path / 'no_such_proc')
    assert proccontrol.scan(missing, self_pid=999, uid=os.getuid()) == []
    with pytest.raises(FileNotFoundError):
        proccontrol.scan(missing, self_pid=999, uid=os.getuid(), strict=True)


@pytest.mark.skipif(os.geteuid() == 0,
                    reason='root reads a mode-000 file anyway; the '
                           'permission case cannot be built as root')
def test_strict_scan_raises_on_an_unreadable_cmdline(tmp_path):
    root = make_proc(tmp_path, {100: PP_NODE})
    cmdline = tmp_path / 'proc' / '100' / 'cmdline'
    cmdline.chmod(0)
    try:
        assert proccontrol.scan(root, self_pid=999, uid=os.getuid()) == []
        with pytest.raises(PermissionError):
            proccontrol.scan(root, self_pid=999, uid=os.getuid(), strict=True)
    finally:
        cmdline.chmod(0o644)


def test_slam_reset_is_refused_when_processes_cannot_be_read():
    def broken():
        raise PermissionError('/proc is hidepid')

    reason = proccontrol.slam_reset_refusal(broken)
    assert 'could not check what is running' in reason


def test_slam_reset_is_refused_for_a_protected_controller(tmp_path):
    """pure_pursuit owned by another user is protected from *stopping*, but
    it is still steering from the pose a reset would freeze."""
    root = make_proc(tmp_path, {100: PP_NODE})
    targets = proccontrol.scan(root, allowlist=proccontrol.DRIVING_CONTROLLERS,
                               self_pid=999, uid=os.getuid() + 1)
    assert targets[0].protected  # sanity: other-user, so protected
    reason = proccontrol.slam_reset_refusal(lambda: targets)
    assert reason.startswith('refused')
    assert 'pure_pursuit_node (pid 100)' in reason


def test_slam_reset_is_not_blocked_by_the_always_running_mux(tmp_path):
    """scan() returns the actuation path whatever the allowlist; the mux is
    always up and must not make the reset permanently unavailable."""
    root = make_proc(tmp_path, {101: MUX_NODE})
    targets = proccontrol.scan(root, allowlist=proccontrol.DRIVING_CONTROLLERS,
                               self_pid=999, uid=os.getuid())
    assert len(targets) == 1 and targets[0].protected  # sanity: mux listed
    assert proccontrol.slam_reset_refusal(lambda: targets) == ''
