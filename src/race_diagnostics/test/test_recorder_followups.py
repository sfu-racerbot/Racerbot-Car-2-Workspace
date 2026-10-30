"""Follow-up recorder rules: snapshot back-off and tegrastats parent death.

Run with pytest src/race_diagnostics/test/.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from race_diagnostics.run_metadata import NodeWatch  # noqa: E402


def test_failing_node_is_offered_on_the_capped_back_off_polls():
    watch = NodeWatch()
    offered = []
    for poll in range(1, 101):
        pairs = watch.update({'/a'})
        if pairs:
            assert pairs == [('/a', 'a.yaml')]
            offered.append(poll)
            watch.failed('/a')
    # oracle: after the k-th consecutive failure of one appearance, skip
    # min(2**(k-1), 32) - 1 polls, so a node that fails on every offer is
    # returned on polls 1, 2, 4, 8, 16, 32, 64 and 96 (closed form from the
    # rule; the 32 cap makes every later gap 31 polls).
    assert offered == [1, 2, 4, 8, 16, 32, 64, 96]


def test_one_failure_then_success_is_never_retried():
    watch = NodeWatch()
    assert watch.update({'/a'}) == [('/a', 'a.yaml')]
    watch.failed('/a')
    # oracle: k=1 skips min(2**0, 32) - 1 = 0 polls, so the next poll retries.
    assert watch.update({'/a'}) == [('/a', 'a.yaml')]
    watch.done('/a')
    # oracle: a successful continuously present node is not returned again.
    for _ in range(40):
        assert watch.update({'/a'}) == []


def test_restart_resets_back_off_and_keeps_the_restart_name():
    watch = NodeWatch()
    assert watch.update({'/a'}) == [('/a', 'a.yaml')]
    watch.done('/a')
    # oracle: absence after success is a restart; the next appearance is .1.
    assert watch.update(set()) == []
    assert watch.update({'/a'}) == [('/a', 'a.1.yaml')]
    watch.failed('/a')
    # oracle: k=1 skips 0 polls, so the same appearance is offered again.
    assert watch.update({'/a'}) == [('/a', 'a.1.yaml')]
    watch.failed('/a')
    assert watch.update(set()) == []
    # oracle: absence then presence restarts the appearance, resetting the
    # failure count (a node that failed twice would otherwise skip the next
    # poll); the failed appearance never succeeded, so the name stays .1.
    assert watch.update({'/a'}) == [('/a', 'a.1.yaml')]


def test_backing_off_node_does_not_hold_back_a_second_node():
    watch = NodeWatch()
    assert watch.update({'/a'}) == [('/a', 'a.yaml')]
    watch.failed('/a')
    assert watch.update({'/a'}) == [('/a', 'a.yaml')]
    watch.failed('/a')
    # oracle: k=2 skips one poll, so /a is absent from this update.
    assert watch.update({'/a'}) == []
    assert watch.update({'/a'}) == [('/a', 'a.yaml')]
    watch.failed('/a')
    # oracle: the skip applies only to /a; a name first seen at poll 5 is
    # returned at poll 5 with its first-appearance name.
    assert watch.update({'/a', '/b'}) == [('/b', 'b.yaml')]


def _fake_tegrastats(tmp_path):
    script = tmp_path / 'fake_tegrastats'
    pidfile = tmp_path / 'tegra.pid'
    script.write_text('#!/bin/sh\n' + f'echo $$ > "{pidfile}"\n' +
                      'while :; do echo sample; sleep 0.1; done\n')
    script.chmod(0o755)
    return script, pidfile


def _process_state(pid):
    """proc(5) state letter, or None when /proc/<pid> no longer exists."""
    try:
        text = Path(f'/proc/{pid}/stat').read_text()
    except OSError:  # FileNotFoundError, or ProcessLookupError if it just exited
        return None
    return text.rsplit(')', 1)[1].split()[0]


def _wait_gone(pid, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = _process_state(pid)
        # oracle: /proc/<pid> disappears when reaped; a Z (zombie) has already
        # exited and only awaits its new parent's wait(), so it counts as gone.
        if state is None or state == 'Z':
            return True
        time.sleep(0.02)
    return False


def test_tegrastats_dies_when_its_parent_is_sigkilled(tmp_path):
    if not sys.platform.startswith('linux'):
        pytest.skip('PR_SET_PDEATHSIG is Linux-only')
    script, pidfile = _fake_tegrastats(tmp_path)
    package_root = str(Path(__file__).resolve().parents[1])
    parent = tmp_path / 'parent.py'
    parent.write_text(
        'import sys, time\n'
        'sys.path.insert(0, sys.argv[3])\n'
        'from race_diagnostics import system_log\n'
        'proc = system_log.start_tegrastats(sys.argv[1], binary=sys.argv[2])\n'
        'print(proc.pid, flush=True)\n'
        'time.sleep(60)\n')
    logger = tmp_path / 'tegrastats.log'
    proc = subprocess.Popen(
        [sys.executable, str(parent), str(logger), str(script), package_root],
        stdout=subprocess.PIPE, text=True)
    child_pid = None
    try:
        child_pid = int(proc.stdout.readline())
        deadline = time.monotonic() + 5
        while not pidfile.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        # oracle: the fake writes $$ (its own pid), which must be the child
        # Popen returned; it is running before the parent is killed.
        assert int(pidfile.read_text()) == child_pid
        assert _process_state(child_pid) not in (None, 'Z')
        proc.kill()
        proc.wait(timeout=5)
        # oracle: PR_SET_PDEATHSIG=SIGTERM delivers SIGTERM to the child when
        # its parent dies, so the orphaned tegrastats exits within 5 s.
        assert _wait_gone(child_pid)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate()
        if child_pid is not None:
            try:
                os.kill(child_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
