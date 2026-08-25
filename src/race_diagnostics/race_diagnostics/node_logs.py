"""
node_logs.py

Locate the per-node stdout logs ROS 2 launch already writes on its own,
so a run can be diagnosed even when nobody remembered `| tee`.

Every node in this workspace already writes its full stdout to a loose
``~/.ros/log/python3_<PID>_<epoch_ms>.log`` file -- this is separate from,
and far more complete than, the mostly-empty aggregated ``launch.log`` ROS
also writes (see docs/run-diagnostics.md). Direct comparison against a
real day's runs found every one of that day's node decisions sitting in
these files already; the missing piece was never capture, only discovery
-- nothing correlated a run's time window back to which of the (currently
~19k, unpruned) loose files belong to it.

PID is not a safe key by itself. A single pytest process constructs and
destroys many rclpy Node objects in one run, and each one gets its own
log file sharing that one process's PID with just milliseconds between
their embedded timestamps -- confirmed directly on this machine. Two
genuinely unrelated real launches, weeks apart, can just as easily reuse
an OS PID. The embedded filename timestamp plus the node name read back
out of the file's own first line are what make a match trustworthy;
correlate by both together, never by PID alone.

Pure Python, no rclpy, same style as run_events.py -- importable and
unit-testable without a robot.
"""

import re
from pathlib import Path

DEFAULT_LOG_DIR = Path('~/.ros/log').expanduser()

FILENAME_RE = re.compile(r'^python3_(\d+)_(\d+)\.log$')
LOG_LINE_RE = re.compile(
    r'^\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\] \[(\d+\.\d+)\] \[([^\]]+)\]:')

# How many lines to read looking for the first recognizable one before
# giving up on a file. Real node logs open with one immediately; this is
# just a bound against a huge file that never has one.
_NODE_NAME_SEARCH_LINES = 20


def _node_name(path: Path):
    """The node name read from the first log-shaped line in `path`.

    None for a file with no such line -- common and not an error: a Node
    constructed and destroyed inside one test, or between two shutdown
    signals, can leave a file nothing was ever written to.
    """
    try:
        with path.open('r', errors='replace') as handle:
            for _ in range(_NODE_NAME_SEARCH_LINES):
                line = handle.readline()
                if not line:
                    break
                match = LOG_LINE_RE.match(line)
                if match is not None:
                    return match.group(2)
    except OSError:
        return None
    return None


def find_node_logs(run_start_epoch_sec: float, run_end_epoch_sec: float,
                    slack_sec: float = 10.0, log_dir: Path = DEFAULT_LOG_DIR):
    """Per-node log files whose filename timestamp falls inside one run.

    Returns ``{node_name: [Path, ...]}``, each list sorted oldest-first
    by its own filename timestamp. A file that logged nothing recognizable
    before EOF is grouped under ``'unknown'`` rather than dropped, so a
    caller can still see that *something* was writing there.

    `slack_sec` widens the window on both ends: a filename timestamp is
    when that file was opened, which lands at-or-just-after process start
    and strictly before the process's first useful line, and a run's own
    recorded start/end are never exactly one node's process start/exit.
    """
    if run_end_epoch_sec < run_start_epoch_sec:
        raise ValueError('run_end_epoch_sec must not precede run_start_epoch_sec')
    lo = run_start_epoch_sec - slack_sec
    hi = run_end_epoch_sec + slack_sec

    grouped = {}
    for path in Path(log_dir).glob('python3_*.log'):
        match = FILENAME_RE.match(path.name)
        if match is None:
            continue
        file_epoch_sec = int(match.group(2)) / 1000.0
        if not (lo <= file_epoch_sec <= hi):
            continue
        node_name = _node_name(path) or 'unknown'
        grouped.setdefault(node_name, []).append(path)

    for paths in grouped.values():
        paths.sort(key=lambda p: int(FILENAME_RE.match(p.name).group(2)))
    return grouped


def merged_lines(node_logs: dict) -> list:
    """Every line from every file in a `find_node_logs` result, time-ordered.

    Ordered by each line's own embedded ROS timestamp, not by file order
    or dict-iteration order -- required to correctly interleave multiple
    nodes' logs (and, for the pytest case described above, multiple
    short-lived files sharing one PID). A line with no parseable
    timestamp of its own (a blank line, a traceback continuation) is
    placed immediately after the last timestamped line read from the same
    file, on the reasonable assumption that any one file is written in
    order.
    """
    entries = []   # (sort_key, line), sort_key = (timestamp, insertion index)
    index = 0
    for paths in node_logs.values():
        for path in paths:
            last_stamp = 0.0
            try:
                text = path.read_text(errors='replace')
            except OSError:
                continue
            for line in text.splitlines():
                match = LOG_LINE_RE.match(line)
                if match is not None:
                    last_stamp = float(match.group(1))
                entries.append(((last_stamp, index), line))
                index += 1
    entries.sort(key=lambda entry: entry[0])
    return [line for _key, line in entries]
