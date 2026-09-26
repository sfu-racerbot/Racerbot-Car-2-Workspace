"""Process hygiene for the simulator validation harnesses.

The harnesses used to clean up with `pkill -f` / `pkill -9 -f` on patterns
like `ackermann_mux/lib` and `gap_follow/lib`, before and after every run.
Those patterns match the *installed real stack* just as well as the
simulated one -- and this workspace builds, and is most conveniently
validated, on the car's own Jetson. SIGKILLing the real ackermann_mux mid
run leaves the VESC on its last command (nothing downstream publishes a
zero). So the rules here are:

* Clean up only what this harness started: each launch runs in its own
  session (`start_new_session=True`), so its process group is exactly the
  set of processes it owns, including children orphaned when `ros2 launch`
  itself exits first.
* Never kill anything else. If matching processes are already running
  before a run starts, *refuse* and name them -- they may be a previous
  crashed harness, or they may be the car. A person decides which.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time


def find_strays(patterns, exclude_pids=()):
    """{pid: cmdline} for running processes whose cmdline matches a pattern.

    Read-only (`pgrep -a -f`). This process and its ancestry never count.
    """
    excluded = set(exclude_pids) | {os.getpid(), os.getppid()}
    found = {}
    for pattern in patterns:
        completed = subprocess.run(
            ['pgrep', '-a', '-f', pattern], capture_output=True, text=True)
        for line in completed.stdout.splitlines():
            pid_text, _, cmdline = line.partition(' ')
            try:
                pid = int(pid_text)
            except ValueError:
                continue
            if pid not in excluded:
                found[pid] = cmdline
    return found


def refuse_if_strays(patterns) -> None:
    """Exit with a clear message if anything matching `patterns` is running."""
    strays = find_strays(patterns)
    if not strays:
        return
    listed = '\n'.join(f'  {pid}  {cmd}' for pid, cmd in sorted(strays.items()))
    raise SystemExit(
        'Refusing to start: these processes are already running and would '
        'share the ROS graph with the simulator:\n'
        f'{listed}\n'
        'If they are left over from an earlier harness run, stop them '
        'yourself. If they are the real car\'s stack, shut it down first -- '
        'this harness never kills processes it did not start.')


def stop_group(process, grace_sec: float = 15.0, linger_sec: float = 3.0) -> None:
    """SIGINT the whole process group `process` leads, then SIGKILL it.

    Group membership, not the leader's exit, decides when this is done:
    `ros2 launch` can return while a child it failed to stop is still
    alive, and that child is still in the group. Whatever is left in the
    group `linger_sec` after the leader is gone gets SIGKILL.
    """
    if process is None:
        return
    pgid = process.pid          # start_new_session=True: leader pid == pgid
    try:
        os.killpg(pgid, signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        process.wait(timeout=grace_sec)
    except subprocess.TimeoutExpired:
        pass
    deadline = time.monotonic() + linger_sec
    while time.monotonic() < deadline:
        if not _group_alive(pgid):
            return
        time.sleep(0.2)
    try:
        os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass


def _group_alive(pgid) -> bool:
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True
