"""
proccontrol.py

The dashboard's half of "stop the driving algorithm from the browser":
finding the driving processes that are actually running on this machine,
deciding which of them a browser is allowed to stop, and escalating a stop
request from a polite Ctrl+C to an unignorable kill.

Like protocol.py and tuning.py, this file imports no ROS, no Tornado and
no network code, so it is directly unit-testable (test/test_proccontrol.py)
against a fake /proc tree, without a running robot or a browser -- and
without ever signalling a real process.

------------------------------------------------------------------------
Why this exists
------------------------------------------------------------------------
Ctrl+C in the terminal that owns a launch is supposed to bring the whole
launch down. In practice it routinely does not: a node wedged in a
callback, a `ros2 launch` that loses track of a child, or a terminal
closed out from under a launch all leave a node still spinning, still
subscribed, and -- crucially -- still publishing to /drive. The next run
then comes up beside the stale one and two controllers fight over the car.

This module lets the dashboard find those processes and end them.

------------------------------------------------------------------------
What this is NOT
------------------------------------------------------------------------
**This is not an emergency stop, and must never be documented as one.**

Killing a driving node does not command the car to stop. It removes the
thing that was commanding it. What happens next is:

  driving node dies -> /drive goes silent -> after `timeout: 0.2` the mux
  drops that input -> ackermann_mux publishes NOTHING (it only ever
  publishes from inside a subscription callback -- see
  ackermann_mux/include/ackermann_mux/topic_handle.hpp) -> vesc_driver has
  no watchdog of its own, so the VESC holds its last commanded value until
  the VESC *firmware's* own motor timeout releases it.

So the car coasts to a stop on a firmware timeout, over a distance, rather
than braking on command. The real stop is still the one it has always
been: **release LB**, which makes joy_teleop actively publish zeroes at
priority 100 and override /drive immediately.

That asymmetry is exactly why PROTECTED below is not negotiable.

------------------------------------------------------------------------
The protected set
------------------------------------------------------------------------
Killing a *driving algorithm* leaves a car with no algorithm, which is
safe. Killing anything in the *actuation path* leaves a car that is still
moving and can no longer be told to stop -- kill ackermann_mux mid-run and
releasing LB stops doing anything at all, because there is nothing left to
carry the zeroes to the VESC.

So the actuation path is refused unconditionally, before the allowlist is
even consulted. Putting one of these names in `killable_nodes` does not
enable it; it logs a warning and is ignored. This is a policy in the same
sense as the LB deadman (docs/architecture.md) -- not a tuning knob.
"""

import errno
import os
import signal

# Never signalled, whatever the config says. Three groups:
#
#   1. The actuation path -- what carries a command to the wheels, and
#      what carries an LB release to the wheels. See the module docstring.
#   2. The dashboard itself, and the launch that owns it. A stop button
#      that can kill the server serving the stop button is a trap.
#   3. The launches that own group 1.
#
# Matched against both the executable basename and the ROS node name.
PROTECTED = frozenset({
    # -- actuation path --------------------------------------------------
    'vesc_driver_node',
    'ackermann_to_vesc_node',
    'vesc_to_odom_node',
    'throttle_interpolator',
    'ackermann_mux',
    'joy_node',
    'joy_teleop',
    'joy_linux_node',
    # -- the dashboard itself --------------------------------------------
    'dashboard_node',
    'web_dashboard_node',
    'camera_stream_node',
    # -- launches that own the above -------------------------------------
    'bringup_launch.py',
    'teleop_launch.py',
    'web_dashboard_launch.py',
})

# What a browser may stop, out of the box: this workspace's own driving
# algorithms, teammates' driving nodes from racerbot_a / racerbot_b, and
# the simulator's two nodes (which forge LB and fake a /scan, so a stale
# one is its own hazard). Overridable via the `killable_nodes` parameter;
# PROTECTED still wins over anything added there.
#
# Deliberately absent, and worth knowing why: urg_node, particle_filter
# and slam_toolbox. None of them can move the car, but killing one mid-run
# silently degrades a *running* controller rather than stopping it --
# pure_pursuit with a frozen pose is more dangerous than pure_pursuit with
# no pose. Add them to `killable_nodes` if you want them; the default is
# "stop the thing that decides, not the things it reads".
DEFAULT_KILLABLE = (
    # this workspace
    'pure_pursuit_node',
    'gap_follow_node',
    'auto_map_race_node',
    # racerbot_a / racerbot_b
    'gap_finder_node',
    'ftg_node',
    'follow_the_gap_node',
    'reactive_node',
    'wall_follow_node',
    'safety_node',
    # racerbot_sim
    'gym_bridge_node',
    'sim_joy_node',
    # launches that own the above
    'pure_pursuit_launch.py',
    'gap_follow_launch.py',
    'auto_map_race_launch.py',
    'sim_launch.py',
)

# Stop escalation. SIGINT first because that is precisely what Ctrl+C
# sends: rclpy runs its shutdown handlers, the node deregisters from the
# graph, and a launch brings its own children down with it. Everything
# after that is for the case the user actually reported -- the process
# that does not answer Ctrl+C.
STAGES = (
    ('SIGINT', signal.SIGINT),
    ('SIGTERM', signal.SIGTERM),
    ('SIGKILL', signal.SIGKILL),
)


class Target:
    """One process the browser could be shown, and possibly stop."""

    __slots__ = ('pid', 'name', 'kind', 'cmdline', 'protected', 'reason')

    def __init__(self, pid, name, kind, cmdline, protected=False, reason=''):
        self.pid = int(pid)
        self.name = name
        self.kind = kind            # 'node' | 'launch'
        self.cmdline = cmdline
        self.protected = protected
        self.reason = reason

    def as_dict(self):
        return {
            'pid': self.pid,
            'name': self.name,
            'kind': self.kind,
            'cmdline': self.cmdline,
            'protected': self.protected,
            'reason': self.reason,
        }

    def __repr__(self):  # pragma: no cover - debugging aid
        return f'<Target {self.pid} {self.name} {self.kind}>'


def _basename(arg):
    """argv[0] -> the name we match on, with a .py kept intact."""
    return os.path.basename(arg.strip()) if arg else ''


def node_name_from_argv(argv):
    """Pull a `__node:=name` remap out of a cmdline.

    A node launched with a remapped name answers to that name on the ROS
    graph, and that is the name a person reads off `ros2 node list` and
    off this dashboard -- so it has to be what the allowlist is checked
    against, not just the executable it happened to be built from.
    """
    for arg in argv:
        if arg.startswith('__node:='):
            return arg[len('__node:='):]
    return ''


def classify(argv):
    """cmdline argv -> (kind, name), or (None, '') if it is not ours.

    'launch' is matched on the launch *file*, never on the package, so
    that `ros2 launch f1tenth_stack bringup_launch.py` is recognised and
    refused by name rather than falling through unclassified.
    """
    if not argv:
        return None, ''

    # `ros2 launch <pkg> <file.py> ...` / `ros2 run <pkg> <exe>` -- argv[0]
    # is the python interpreter or the ros2 script itself, so scan for the
    # verb rather than trusting a position.
    for index, arg in enumerate(argv):
        if _basename(arg) == 'ros2' and index + 1 < len(argv):
            verb = argv[index + 1]
            if verb == 'launch':
                for candidate in argv[index + 2:]:
                    if candidate.endswith('.py'):
                        return 'launch', _basename(candidate)
                return None, ''
            # The exec'd child is what we actually want, but the wrapper
            # is worth matching too so a stuck `ros2 run` is not invisible.
            if verb == 'run' and index + 3 < len(argv):
                return 'node', _basename(argv[index + 3])

    remapped = node_name_from_argv(argv)
    if remapped:
        return 'node', remapped

    name = _basename(argv[0])
    # `python3 <script>` names the script, not the interpreter.
    #
    # The script is deliberately NOT required to end in .py. An installed
    # ament_python node is a setuptools console-script -- a shebang file
    # named exactly `pure_pursuit_node`, with no extension -- so a .py
    # requirement here makes every Python node in this workspace, which is
    # nearly all of them, invisible to the scan. Verified against
    # install/pure_pursuit/lib/pure_pursuit/pure_pursuit_node, whose
    # cmdline really is `/usr/bin/python3 /…/pure_pursuit_node`.
    if name.startswith('python'):
        for arg in argv[1:]:
            if arg.startswith('-'):
                continue        # -m, -u, -c and friends
            return 'node', _basename(arg)
        return None, ''
    return ('node', name) if name else (None, '')


def is_protected(name, kind='node', argv=()):
    """True if this must never be signalled, whatever the config says.

    Checks the matched name first, then every argv element's basename --
    so a node started as `ros2 run ackermann_mux ackermann_mux_node` is
    refused on the package name even when the executable differs.
    """
    if name in PROTECTED:
        return True
    for arg in argv:
        base = _basename(arg)
        if base in PROTECTED:
            return True
        # `ackermann_mux_node` vs `ackermann_mux` -- both refused.
        for guard in PROTECTED:
            if base == guard + '_node' or base + '_node' == guard:
                return True
    return False


def sanitize_allowlist(names, logger=None):
    """Drop anything from a configured allowlist that PROTECTED covers.

    Returns the usable names. A rejected entry is a configuration mistake
    worth shouting about, not something to silently honour or silently
    drop -- so it is logged if a logger is supplied.
    """
    kept, refused = [], []
    for raw in names:
        name = str(raw).strip()
        if not name:
            continue
        if is_protected(name):
            refused.append(name)
            continue
        if name not in kept:
            kept.append(name)
    if refused and logger is not None:
        logger.warn(
            'process control: refusing to make '
            + ', '.join(sorted(refused))
            + ' stoppable -- these are in the actuation path and killing one '
              'leaves a moving car that LB can no longer stop. Ignoring them; '
              'see proccontrol.PROTECTED.')
    return kept


def _read_cmdline(proc_root, pid):
    try:
        with open(os.path.join(proc_root, str(pid), 'cmdline'), 'rb') as handle:
            raw = handle.read()
    except (OSError, ValueError):
        return []
    if not raw:
        # Kernel threads have an empty cmdline. They are never ours.
        return []
    return [part.decode('utf-8', 'replace')
            for part in raw.split(b'\0') if part]


def _read_uid(proc_root, pid):
    """Owner of a pid, or None if it cannot be determined."""
    try:
        return os.stat(os.path.join(proc_root, str(pid))).st_uid
    except OSError:
        return None


def _read_ppid(proc_root, pid):
    try:
        with open(os.path.join(proc_root, str(pid), 'status'), 'r') as handle:
            for line in handle:
                if line.startswith('PPid:'):
                    return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        return None
    return None


def _read_state(proc_root, pid):
    """The single-letter process state from /proc/<pid>/status, or None."""
    try:
        with open(os.path.join(proc_root, str(pid), 'status'), 'r') as handle:
            for line in handle:
                if line.startswith('State:'):
                    return line.split()[1]
    except (OSError, ValueError, IndexError):
        return None
    return None


def is_zombie(pid, proc_root='/proc'):
    return _read_state(proc_root, pid) == 'Z'


def ancestors(proc_root, pid, limit=64):
    """Every pid between `pid` and init, inclusive of `pid`.

    Used to make the dashboard unable to kill itself or whatever started
    it: a stop button that can take out its own server, or the terminal
    session holding the whole stack, is worse than no stop button.
    """
    seen, current = set(), pid
    while current and current > 1 and len(seen) < limit:
        seen.add(current)
        parent = _read_ppid(proc_root, current)
        if parent is None or parent in seen:
            break
        current = parent
    return seen


def scan(proc_root='/proc', allowlist=DEFAULT_KILLABLE, self_pid=None, uid=None):
    """Every driving process on this machine, with a verdict for each.

    Returns a list of Target. Entries the browser may stop have
    `protected=False`; entries it may not are still returned, with the
    reason, because "pure_pursuit is running and you may not kill it from
    here" is more useful to a person than an empty list.
    """
    self_pid = os.getpid() if self_pid is None else self_pid
    uid = os.getuid() if uid is None else uid
    allowed = set(allowlist)
    protected_pids = ancestors(proc_root, self_pid)

    targets = []
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return targets

    for entry in sorted(entries, key=lambda e: int(e) if e.isdigit() else 0):
        if not entry.isdigit():
            continue
        pid = int(entry)
        argv = _read_cmdline(proc_root, pid)
        if not argv:
            continue
        kind, name = classify(argv)
        if kind is None or not name:
            continue
        protected_here = is_protected(name, kind, argv)
        if name not in allowed and not protected_here:
            continue

        cmdline = ' '.join(argv)
        if protected_here:
            targets.append(Target(
                pid, name, kind, cmdline, True,
                'in the actuation path -- stopping it would leave a moving '
                'car that releasing LB can no longer stop'))
            continue
        if pid in protected_pids:
            targets.append(Target(
                pid, name, kind, cmdline, True,
                'this is the dashboard, or the process that started it'))
            continue
        owner = _read_uid(proc_root, pid)
        if owner is not None and uid is not None and owner != uid:
            targets.append(Target(
                pid, name, kind, cmdline, True,
                f'owned by another user (uid {owner})'))
            continue
        targets.append(Target(pid, name, kind, cmdline, False, ''))
    return targets


def find(targets, pid):
    for target in targets:
        if target.pid == pid:
            return target
    return None


def alive(pid, sender=None, proc_root='/proc'):
    """Is this pid still running? Signal 0 checks without delivering.

    The zombie check is not a nicety. A process that has exited but whose
    parent has not reaped it keeps its /proc entry and still answers
    signal 0 -- so a naive liveness check calls it alive forever, and this
    module would report "survived SIGINT, SIGTERM, SIGKILL, try a reboot"
    about a process it had in fact successfully killed.

    That is the *common* case here, not an exotic one. The thing being
    stopped is usually a child of a `ros2 launch` that is itself wedged --
    which is precisely why it needed stopping from a browser instead of
    with Ctrl+C -- and a wedged launch does not reap. Caught by the
    end-to-end test, never by the unit tests, because a fake signal sender
    has no zombies in it.
    """
    if is_zombie(pid, proc_root):
        return False
    sender = os.kill if sender is None else sender
    try:
        sender(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        # Exists, we simply may not touch it.
        return True
    except OSError as exc:
        return exc.errno != errno.ESRCH
    return True


class StopJob:
    """One browser stop request, walking SIGINT -> SIGTERM -> SIGKILL.

    Deliberately a state machine advanced by an external clock rather
    than something that sleeps: the dashboard drains it from a ROS timer
    on the executor thread, and a stop request must never be able to
    block the thread that is also serving telemetry.
    """

    def __init__(self, pid, name, grace_sec=2.0, now=0.0, proc_root='/proc'):
        self.pid = int(pid)
        self.name = name
        self.proc_root = proc_root
        self.grace_sec = float(grace_sec)
        self.stage = -1              # index into STAGES; -1 = nothing sent
        self.next_action_at = now    # monotonic deadline for the next step
        self.done = False
        self.ok = False
        self.detail = 'queued'
        self.sent = []

    def advance(self, now, sender=None):
        """Move the job along. Returns True if anything changed.

        Called repeatedly; does nothing until `now` passes the deadline
        set by the previous stage.
        """
        if self.done:
            return False
        if not alive(self.pid, sender, self.proc_root):
            self.done, self.ok = True, True
            self.detail = 'stopped' if self.sent else 'was already gone'
            return True
        if now < self.next_action_at:
            return False

        self.stage += 1
        if self.stage >= len(STAGES):
            self.done, self.ok = True, False
            self.detail = (f'pid {self.pid} survived '
                           + ', '.join(self.sent)
                           + ' -- it is probably stuck in the kernel '
                             '(uninterruptible I/O); a reboot is the next step')
            return True

        label, signum = STAGES[self.stage]
        send = os.kill if sender is None else sender
        try:
            send(self.pid, signum)
        except ProcessLookupError:
            self.done, self.ok = True, True
            self.detail = 'stopped'
            return True
        except PermissionError:
            self.done, self.ok = True, False
            self.detail = f'not permitted to signal pid {self.pid}'
            return True
        except OSError as exc:
            self.done, self.ok = True, False
            self.detail = f'could not signal pid {self.pid}: {exc}'
            return True

        self.sent.append(label)
        self.detail = f'sent {label}'
        self.next_action_at = now + self.grace_sec
        return True

    def as_dict(self):
        return {
            'pid': self.pid,
            'name': self.name,
            'done': self.done,
            'ok': self.ok,
            'detail': self.detail,
            'sent': list(self.sent),
        }
