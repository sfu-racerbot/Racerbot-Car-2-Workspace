"""
run_events.py

Framework-agnostic classification and rate-limiting for a recorded run --
no rclpy, no ROS, plain strings and numbers in and out, so all of it is
unit-testable without a robot (the same split as pure_pursuit's
racing_math.py and web_dashboard's protocol.py; see CLAUDE.md).

Two things live here:

  LogClassifier   Decides whether a line from the launch terminal is worth
                  reporting, and whether it is urgent enough to report
                  immediately or should be throttled. The launch terminal
                  emits several hundred lines a minute at 40Hz control
                  rates; roughly a dozen of them per run actually matter.

  RunTimeline     Accumulates the structured facts an analyst (human or
                  agent) needs afterwards: when each phase started, what
                  the worst pose lag was, which watchdogs fired and how
                  often, whether the run ended in a stop or a crash.

The categories below are the failure modes this car has actually shown,
each earning its place from a real incident -- see
docs/run-diagnostics.md and docs/troubleshooting.md.
"""

import re

# Lines that must never be throttled: a phase change or a failure that
# only ever prints once, and whose absence is itself the diagnosis.
CRITICAL_PATTERNS = [
    ('slam_lifecycle', re.compile(
        r'Configuring|Using solver plugin|Activating|LifecycleLaunch')),
    ('lap_closed', re.compile(r'Closed mapping lap')),
    ('profile_generated', re.compile(r'racing profile at')),
    ('profile_loaded', re.compile(r'Profile loaded successfully|pure_pursuit_node ready:')),
    ('handover', re.compile(r'Transition complete')),
    ('map_save', re.compile(
        r'Requested map and pose-graph save|Saved .* successfully|'
        r'Waiting for slam_toolbox to finish saving|did not finish within')),
    ('watchdog', re.compile(
        r'STOP \[(pose_frozen|pose_stale|body_contact|off_racing_line|'
        r'emergency_obstacle|waiting_for_pose)\]')),
    ('node_death', re.compile(r'process has died|Traceback|process exited')),
    ('error', re.compile(r'\[ERROR\]|Exception|rejected|[Cc]ould not|[Ff]ailed')),
]

# Worth seeing, but emitted continuously -- one every `throttle_sec`.
THROTTLED_PATTERNS = [
    ('lap_progress', re.compile(r'lap \d+/\d+: samples=')),
    ('tf_missing', re.compile(r'Waiting for map->base_link|waiting for a valid map->base_link')),
    ('scan_dropped', re.compile(r'Message Filter dropping message')),
    ('stopped', re.compile(r'STOP \[')),
]

LAP_PROGRESS_RE = re.compile(
    r'lap (\d+)/(\d+): samples=(\d+), distance=([\d.]+)/([\d.]+)m, '
    r'elapsed=([\d.]+)/([\d.]+)s, departed=(\w+), '
    r'start distance=([\d.]+)/([\d.]+)m, heading error=([\d.]+)/([\d.]+)deg')


class LogClassifier:
    """Categorize launch-terminal lines and rate-limit the noisy ones.

    `now_sec` is passed in rather than read from a clock so tests are
    deterministic and a recorded log can be re-classified offline at its
    original timestamps.
    """

    def __init__(self, throttle_sec: float = 20.0):
        if not (throttle_sec >= 0.0):
            raise ValueError('throttle_sec must be non-negative')
        self.throttle_sec = throttle_sec
        self._last_emit = {}

    def classify(self, line: str, now_sec: float):
        """Return (category, emit) for one line.

        category is None when the line is uninteresting. emit is False for
        a recognized-but-throttled line whose turn has not come round yet,
        so callers can still count it without printing it.
        """
        for category, pattern in CRITICAL_PATTERNS:
            if pattern.search(line):
                self._last_emit[category] = now_sec
                return category, True

        for category, pattern in THROTTLED_PATTERNS:
            if pattern.search(line):
                previous = self._last_emit.get(category)
                if previous is None or now_sec - previous >= self.throttle_sec:
                    self._last_emit[category] = now_sec
                    return category, True
                return category, False

        return None, False


def parse_lap_progress(line: str):
    """Pull the lap-closure gate numbers out of a supervisor line.

    Returns None if the line isn't one. Every gate is reported as
    (value, limit, satisfied) so an analyst can see *which* gate is the
    one holding a lap open -- on 2026-07-27 the car drove 114m without
    closing because the heading gate was failing by 0.2 degrees while
    every other gate had passed long before.
    """
    match = LAP_PROGRESS_RE.search(line)
    if match is None:
        return None
    g = match.groups()

    def gate(value, limit, at_most=True):
        value, limit = float(value), float(limit)
        return {'value': value, 'limit': limit,
                'ok': value <= limit if at_most else value >= limit}

    return {
        'lap': int(g[0]),
        'of': int(g[1]),
        'samples': int(g[2]),
        'distance': gate(g[3], g[4], at_most=False),
        'elapsed': gate(g[5], g[6], at_most=False),
        'departed': g[7] == 'yes',
        'start_distance': gate(g[8], g[9]),
        'heading_error': gate(g[10], g[11]),
    }


def blocking_gate(progress) -> str:
    """Which single gate is keeping this lap from closing, or '' if none.

    Reported in gate order so the answer is stable and actionable rather
    than whichever failing gate happened to be checked last.
    """
    if progress is None:
        return ''
    if not progress['departed']:
        return 'departed'
    for name in ('distance', 'elapsed', 'start_distance', 'heading_error'):
        if not progress[name]['ok']:
            return name
    return ''


class RunTimeline:
    """The structured story of one run, built up as events arrive."""

    def __init__(self):
        self.events = []
        self.counts = {}
        self.pose_lag_max = 0.0
        self.pose_lag_max_at = None
        self.worst_lap_gate = None
        self.phases = {}

    def add(self, timestamp: float, category: str, detail: str = ''):
        self.events.append({'t': timestamp, 'category': category, 'detail': detail})
        self.counts[category] = self.counts.get(category, 0) + 1
        # First occurrence of a phase-defining event is the phase start.
        if category in ('slam_lifecycle', 'lap_closed', 'profile_generated',
                        'handover') and category not in self.phases:
            self.phases[category] = timestamp

    def note_pose_lag(self, timestamp: float, lag_sec: float):
        if lag_sec > self.pose_lag_max:
            self.pose_lag_max = lag_sec
            self.pose_lag_max_at = timestamp

    def note_lap_progress(self, progress):
        gate = blocking_gate(progress)
        if gate:
            self.worst_lap_gate = gate

    def summary(self) -> dict:
        return {
            'event_counts': dict(sorted(self.counts.items())),
            'phases_reached': sorted(self.phases),
            'pose_lag_max_sec': round(self.pose_lag_max, 3),
            'pose_lag_max_at': self.pose_lag_max_at,
            'lap_gate_blocking': self.worst_lap_gate,
            'total_events': len(self.events),
        }
