"""The validation harnesses' process hygiene (tools/racerbot_sim/harness_procs.py).

Lives in this package's suite so the command people actually run collects
it (the harnesses themselves have no test runner):

    python3 -m pytest src/racerbot_sim/test/test_harness_procs.py -v

Audit H3 (2026-09-25): the harnesses cleaned up with `pkill -9 -f
ackermann_mux/lib` and friends, which on the car's Jetson matches the real
stack. These tests use real processes -- the process table is the boundary
under test, so there is nothing to mock.
"""

import os
import subprocess
import sys
import time
import uuid

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'tools', 'racerbot_sim'))
import harness_procs  # noqa: E402


def _alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    # A reaped-by-init orphan disappears; an unreaped one is a zombie.
    try:
        with open(f'/proc/{pid}/status') as handle:
            return not any(line.startswith('State:') and 'Z' in line.split()[1]
                           for line in handle)
    except OSError:
        return False


def _wait_gone(pid, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _alive(pid):
            return True
        time.sleep(0.05)
    return False


def test_stop_group_kills_an_orphan_the_leader_left_behind(tmp_path):
    """The case a leader-only kill misses: the leader exits on SIGINT but a
    child that ignores SIGINT stays in the group."""
    pidfile = tmp_path / 'child.pid'
    # An ignored signal disposition survives exec, so the sleep itself
    # ignores SIGINT; only SIGKILL can remove it.
    script = (f"sh -c 'trap \"\" INT; echo $$ > {pidfile}; exec sleep 60' & "
              "wait")
    leader = subprocess.Popen(['sh', '-c', script], start_new_session=True)
    deadline = time.monotonic() + 5.0
    while not pidfile.exists() or not pidfile.read_text().strip():
        assert time.monotonic() < deadline, 'child never started'
        time.sleep(0.05)
    child = int(pidfile.read_text())
    assert _alive(child)  # sanity

    harness_procs.stop_group(leader, grace_sec=2.0, linger_sec=0.5)
    assert _wait_gone(child), 'a SIGINT-ignoring child in the group survived'
    assert leader.poll() is not None


def test_stop_group_leaves_processes_outside_the_group_alone():
    """The half that matters for the car: a process that is not ours --
    standing in for the real ackermann_mux -- must survive the cleanup."""
    bystander = subprocess.Popen(['sleep', '60'], start_new_session=True)
    ours = subprocess.Popen(['sleep', '60'], start_new_session=True)
    try:
        harness_procs.stop_group(ours, grace_sec=2.0, linger_sec=0.5)
        assert ours.poll() is not None
        assert bystander.poll() is None, 'cleanup killed a process it did not start'
    finally:
        bystander.kill()
        bystander.wait()


def test_refuse_if_strays_names_a_matching_process_and_kills_nothing():
    marker = f'harness-stray-{uuid.uuid4().hex}'
    # '; true' keeps sh from exec'ing sleep, so the marker stays in argv.
    stray = subprocess.Popen(['sh', '-c', f'sleep 60; true # {marker}'])
    try:
        time.sleep(0.1)
        with pytest.raises(SystemExit, match=str(stray.pid)):
            harness_procs.refuse_if_strays((marker,))
        assert stray.poll() is None, 'refusing must never kill'
    finally:
        stray.kill()
        stray.wait()


def test_refuse_if_strays_is_silent_when_nothing_matches():
    harness_procs.refuse_if_strays((f'no-such-process-{uuid.uuid4().hex}',))


def test_find_strays_never_reports_this_process():
    """pgrep -f on a pattern in our own argv must not make the harness
    refuse to run because of itself."""
    with open('/proc/self/cmdline', 'rb') as handle:
        own = [part.decode() for part in handle.read().split(b'\0') if part]
    # The interpreter path, anchored: guaranteed to match this process (so
    # the exclusion is actually exercised), and never starts with '-'.
    pattern = '^' + own[0]
    found = harness_procs.find_strays((pattern,))
    assert os.getpid() not in found
