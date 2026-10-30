"""Record per-process resource use and, when available, Jetson statistics."""
import argparse
import ctypes
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def node_name(argv) -> str | None:
    """Return the fully qualified name specified by ROS command-line remaps."""
    try:
        args = argv[argv.index('--ros-args') + 1:]
    except ValueError:
        return None
    name = None
    namespace = ''
    for index, arg in enumerate(args[:-1]):
        if arg not in ('-r', '--remap'):
            continue
        key, sep, value = args[index + 1].partition(':=')
        if not sep:
            continue
        key = key.split(':')[-1]
        if key == '__node':
            name = value
        elif key == '__ns':
            namespace = value
    if not name:
        return None
    return '/' + '/'.join(part.strip('/') for part in (namespace, name) if part.strip('/'))


def _stat_fields(text):
    end = text.rfind(')')
    if end < 0 or '(' not in text[:end]:
        raise ValueError('invalid proc stat comm')
    fields = text[end + 1:].split()
    if len(fields) < 20:
        raise ValueError('truncated proc stat')
    return fields


def parse_stat(text) -> tuple:
    """Return CPU ticks and thread count from proc(5) stat fields."""
    fields = _stat_fields(text)
    return int(fields[11]) + int(fields[12]), int(fields[17])


def parse_rss_kb(status_text) -> int | None:
    """Return VmRSS in kB, or None when the kernel omits it."""
    for line in status_text.splitlines():
        if line.startswith('VmRSS:'):
            return int(line.split()[1])
    return None


def cpu_percent(prev_ticks, ticks, dt_s, clk_tck) -> float:
    """Return CPU usage, with 100 percent equal to one core."""
    if dt_s <= 0:
        raise ValueError('elapsed time must be positive')
    return 100.0 * (ticks - prev_ticks) / clk_tck / dt_s


class ProcSampler:
    def __init__(self, proc_root='/proc', clk_tck=None, clock=time.monotonic,
                 wall=time.time):
        self.root = Path(proc_root)
        self.clk_tck = clk_tck if clk_tck is not None else os.sysconf('SC_CLK_TCK')
        self.clock = clock
        self.wall = wall
        self.cache = {}

    def sample(self) -> list:
        now = self.clock()
        timestamp = float(self.wall())
        records = []
        seen = set()
        for directory in sorted((p for p in self.root.iterdir() if p.name.isdigit()),
                                key=lambda p: int(p.name)):
            pid = int(directory.name)
            try:
                stat = (directory / 'stat').read_text()
                fields = _stat_fields(stat)
                start = int(fields[19])
                ticks, threads = parse_stat(stat)
                previous = self.cache.get(pid)
                if previous is None or previous[0] != start:
                    argv = (directory / 'cmdline').read_bytes().split(bytes([0]))
                    name = node_name([part.decode(errors='replace') for part in argv if part])
                    previous = None
                else:
                    name = previous[1]
                rss = parse_rss_kb((directory / 'status').read_text()) if name else None
            except (OSError, ValueError, IndexError):
                continue
            seen.add(pid)
            self.cache[pid] = (start, name, ticks, now)
            if name:
                cpu = None
                if previous is not None and now > previous[3]:
                    cpu = cpu_percent(previous[2], ticks, now - previous[3], self.clk_tck)
                records.append({'t': timestamp, 'pid': pid, 'node': name,
                                'cpu_pct': cpu, 'rss_kb': rss, 'threads': threads})
        self.cache = {pid: value for pid, value in self.cache.items() if pid in seen}
        return records


def _set_parent_death_signal() -> bool:
    """Ask Linux to SIGTERM the calling process when its parent dies."""
    if not sys.platform.startswith('linux'):
        return False
    libc = ctypes.CDLL(None, use_errno=True)
    return libc.prctl(1, signal.SIGTERM, 0, 0, 0) == 0  # PR_SET_PDEATHSIG


def _prepare_tegrastats_child(expected_parent):
    _set_parent_death_signal()
    if os.getppid() != expected_parent:
        os._exit(1)


def start_tegrastats(out_path, binary='tegrastats', interval_ms=1000, log=print):
    """Start tegrastats, warning once rather than failing the recording."""
    parent = os.getpid()
    preexec_fn = (lambda: _prepare_tegrastats_child(parent)) if sys.platform.startswith('linux') else None
    try:
        with open(out_path, 'a') as output:
            return subprocess.Popen([binary, '--interval', str(interval_ms)], stdout=output,
                                    preexec_fn=preexec_fn)
    except OSError as exc:
        log(f'Warning: could not start {binary}: {exc}')
        return None


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--proc-root', default='/proc')
    parser.add_argument('--tegrastats', default='tegrastats')
    parser.add_argument('--duration', type=float, default=math.inf)
    args = parser.parse_args(argv)
    if not math.isfinite(args.interval) or args.interval <= 0 or args.duration < 0 or math.isnan(args.duration):
        parser.error('interval must be positive and finite; duration must be nonnegative')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    stop = threading.Event()
    previous_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.signal(sig, lambda *_: stop.set())
    # ros2 run is a wrapper and can exit without forwarding launch's signal.
    # Linux delivers SIGTERM if that parent dies, including during setup.
    parent = os.getppid()
    if sys.platform.startswith('linux'):
        if not _set_parent_death_signal():
            print('Warning: cannot register parent-death signal', file=sys.stderr, flush=True)
        if os.getppid() != parent:
            stop.set()
    tegra = None
    try:
        tegra = start_tegrastats(output / 'tegrastats.log', binary=args.tegrastats,
                                 log=lambda message: print(message, file=sys.stderr, flush=True))
        sampler = ProcSampler(args.proc_root)
        start = time.monotonic()
        with (output / 'proc.jsonl').open('a') as handle:
            while not stop.is_set() and time.monotonic() - start < args.duration:
                try:
                    for record in sampler.sample():
                        handle.write(json.dumps(record) + '\n')
                    handle.flush()
                except Exception as exc:  # a bad tick must not stop the run
                    print(f'Warning: system sampler failed: {exc}', file=sys.stderr, flush=True)
                remaining = args.duration - (time.monotonic() - start)
                stop.wait(max(0.0, min(args.interval, remaining)))
    finally:
        if tegra is not None:
            if tegra.poll() is None:
                tegra.terminate()
            try:
                tegra.wait(timeout=3)
            except subprocess.TimeoutExpired:
                tegra.kill()
                tegra.wait()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    sys.exit(main())
