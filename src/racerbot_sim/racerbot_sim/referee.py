"""ROS-free race scoring from simulator ground truth."""

from __future__ import annotations

import math

import numpy as np


PERSIST_S = 0.5
STALL_SPEED_MPS = 0.05
STALL_MIN_S = 1.0


def _validate_traces(values, name):
    lengths = {len(value) for value in values}
    if len(lengths) > 1:
        raise ValueError(f'{name} traces differ in length: {sorted(lengths)}')
    for index, value in enumerate(values):
        if not all(math.isfinite(float(item)) for item in value):
            raise ValueError(f'car {index}: non-finite {name}')


def _solid_signs(gap, need):
    runs = []
    for value in gap:
        sign = 1 if value > 0 else (-1 if value < 0 else 0)
        if sign == 0:
            if runs:
                runs[-1][1] += 1
        elif runs and runs[-1][0] == sign:
            runs[-1][1] += 1
        else:
            runs.append([sign, 1])
    solid = []
    for sign, ticks in runs:
        if ticks >= need and (not solid or solid[-1] != sign):
            solid.append(sign)
    return solid


def count_overtakes(traces, dt_s, persist_s=PERSIST_S):
    _validate_traces(traces, 'progress')
    need = max(1, math.ceil(persist_s / dt_s - 1e-9))
    counts = [0] * len(traces)
    for first in range(len(traces)):
        for second in range(first + 1, len(traces)):
            gap = [b - a for a, b in zip(traces[first], traces[second])]
            signs = _solid_signs(gap, need)
            for after in signs[1:]:
                counts[second if after > 0 else first] += 1
    return counts


def count_stalls(speeds, dt_s):
    _validate_traces(speeds, 'speed')
    need = math.floor(STALL_MIN_S / dt_s + 1e-9) + 1
    counts = [0] * len(speeds)
    runs = [0] * len(speeds)
    for tick in zip(*speeds):
        moving = [abs(value) >= STALL_SPEED_MPS for value in tick]
        for index in range(len(speeds)):
            if not moving[index] and any(moving):
                runs[index] += 1
                if runs[index] == need:
                    counts[index] += 1
            else:
                runs[index] = 0
    return counts


def pair_gap_changes(traces, finish_times, dt_s, elapsed_s=None):
    _validate_traces(traces, 'progress')
    if len(finish_times) != len(traces):
        raise ValueError('finish times and progress traces differ in length')
    for index, value in enumerate(finish_times):
        if value is not None and not math.isfinite(float(value)):
            raise ValueError(f'car {index}: non-finite finish time')
    changes = []
    for first in range(len(traces)):
        for second in range(first + 1, len(traces)):
            n = len(traces[first])
            trailer = first if n and traces[first][0] < traces[second][0] else second
            leader = second if trailer == first else first
            start = end = change = None
            if n >= 2:
                elapsed = (n - 1) * dt_s if elapsed_s is None else elapsed_s
                speed = (traces[trailer][-1] - traces[trailer][0]) / elapsed
                if speed <= 0:
                    start = end = change = math.inf
                else:
                    start = (traces[leader][0] - traces[trailer][0]) / speed
                    if finish_times[trailer] is not None and finish_times[leader] is not None:
                        end = finish_times[trailer] - finish_times[leader]
                    else:
                        end = (traces[leader][-1] - traces[trailer][-1]) / speed
                    change = end - start
            changes.append(dict(pair=(first, second), leader=leader,
                                trailer=trailer, start_gap_s=start,
                                end_gap_s=end, gap_change_s=change))
    return changes


def time_gained(changes, cars):
    gained = [0.0] * cars
    for pair in changes:
        change = pair['gap_change_s']
        if change is not None:
            gained[pair['leader']] += change
            gained[pair['trailer']] -= change
    return gained


def json_safe(value):
    """Use JSON null for undefined or non-finite time metrics (not zero)."""
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def format_table(result):
    """Printable standings; '-' means unfinished or an undefined time metric."""
    def seconds(value):
        return '-' if value is None else f'{value:.3f}'

    lines = ['Place Car Laps Finish_s Overtakes Stalls Wall Car_contact Time_gained_s']
    for place, car in enumerate(result['finish_order'], 1):
        i = car - 1
        contact = result['contacts'][i]
        lines.append(f"{place} {car} {result['laps'][i]:.3f} "
                     f"{seconds(result['finish_time_s'][i])} {result['overtakes'][i]} "
                     f"{result['stalls'][i]} {contact['wall']} {contact['car']} "
                     f"{seconds(result['time_gained_s'][i])}")
    winner = result['winner']
    lines.append(f"winner: {'none' if winner is None else f'car {winner}'}; "
                 f"sim_time_s: {result['sim_time_s']:.3f}; gap_s: {seconds(result['gap_s'])}")
    return '\n'.join(lines)


class LapCounter:
    """Unwrapped centreline progress measured from the common start line."""

    def __init__(self, centerline_xy, start_xy, reverse=False):
        self.reference = np.asarray(centerline_xy, dtype=float)
        if self.reference.ndim != 2 or self.reference.shape[1] != 2 or len(self.reference) < 2:
            raise ValueError('centerline must contain at least two XY points')
        if not np.isfinite(self.reference).all():
            raise ValueError('centerline must be finite')
        self.segment = np.hypot(*np.diff(np.vstack((self.reference, self.reference[:1])), axis=0).T)
        self.arc = np.concatenate(([0.0], np.cumsum(self.segment)[:-1]))
        self.total = float(self.segment.sum())
        if self.total <= 0.0:
            raise ValueError('centerline length must be positive')
        self.sign = -1.0 if reverse else 1.0
        self.index = self._nearest(start_xy)
        self._position = self.sign * self._wrap(self.arc[self.index], self.total)

    def _nearest(self, xy):
        point = np.asarray(xy, dtype=float)
        if point.shape != (2,) or not np.isfinite(point).all():
            raise ValueError('XY position must contain two finite values')
        return int(np.argmin(np.hypot(self.reference[:, 0] - point[0],
                                        self.reference[:, 1] - point[1])))

    @staticmethod
    def _wrap(value, total):
        return (value + total / 2.0) % total - total / 2.0

    def update(self, xy):
        new = self._nearest(xy)
        delta = self.arc[new] - self.arc[self.index]
        delta = self._wrap(delta, self.total)
        self._position += self.sign * delta
        self.index = new
        return self.laps

    @property
    def position_m(self):
        return self._position

    @property
    def laps(self):
        return self._position / self.total


class Referee:
    """Score a race from ground truth and simulator-owned contact flags."""

    def __init__(self, centerline_xy, start_xys, laps_required, dt_s, reverse=False):
        if len(start_xys) == 0:
            raise ValueError('at least one car is required')
        if not math.isfinite(float(laps_required)) or laps_required <= 0.0:
            raise ValueError('laps_required must be positive and finite')
        if not math.isfinite(float(dt_s)) or dt_s <= 0.0:
            raise ValueError('dt_s must be positive and finite')
        self.dt_s = float(dt_s)
        self.laps_required = float(laps_required)
        self.counters = [LapCounter(centerline_xy, xy, reverse) for xy in start_xys]
        self.traces = [[] for _ in self.counters]
        self.speeds = [[] for _ in self.counters]
        self.times = [None] * len(self.counters)
        self.contacts = [{'wall': False, 'car': False} for _ in self.counters]
        self._sim_time = 0.0
        self._has_tick = False
        self._finished = False
        self.steps = 0
        self.missed_steps = 0
        self._sample_steps = []
        self._breaks = []

    def tick(self, t_s, xys, speeds, wall_contact, car_contact, *, steps=None):
        """Score one observed step; explicit step numbers expose missing samples.

        Legacy synthetic traces may omit steps to supply consecutive samples.
        Simulator step 1 is the first permitted sample. Older/duplicate steps
        are ignored. Missing steps are never filled with invented observations.
        """
        next_step = self.steps + 1 if steps is None else steps
        if type(next_step) is not int or next_step < 1:
            raise ValueError('steps must be a positive integer')
        if next_step <= self.steps:
            return
        if len(xys) != len(self.counters) or len(speeds) != len(self.counters):
            raise ValueError('positions and speeds must match the number of cars')
        if len(wall_contact) != len(self.counters):
            raise ValueError('wall contact must match the number of cars')
        for position in xys:
            point = np.asarray(position, dtype=float)
            if point.shape != (2,) or not np.isfinite(point).all():
                raise ValueError('positions must contain two finite XY values per car')
        if not all(isinstance(value, (bool, np.bool_)) for value in wall_contact):
            raise ValueError('wall contact must contain booleans')
        if not isinstance(car_contact, (bool, np.bool_)):
            raise ValueError('car_contact must be a boolean')
        if not math.isfinite(float(t_s)):
            raise ValueError('time must be finite')
        if self._has_tick and t_s <= self._sim_time:
            raise ValueError('time must increase with every tick')
        for index, speed in enumerate(speeds):
            try:
                finite = math.isfinite(float(speed))
            except (TypeError, ValueError):
                raise ValueError(f'car {index}: non-finite speed') from None
            if not finite:
                raise ValueError(f'car {index}: non-finite speed')
        # Validate all cars before changing any state: a rejected tick is atomic.
        missing = next_step - self.steps - 1
        if missing and self._sample_steps:
            self._breaks.append(len(self._sample_steps))
        self.missed_steps += missing
        self.steps = next_step
        self._sample_steps.append(next_step)
        contact_now = bool(car_contact or any(wall_contact))
        for index, speed in enumerate(speeds):
            self.counters[index].update(xys[index])
            self.traces[index].append(self.counters[index].position_m)
            self.speeds[index].append(abs(float(speed)))
            # run_multi records progress and contacts, then breaks BEFORE finishes.
            if (not contact_now and self.times[index] is None
                    and self.counters[index].laps >= self.laps_required):
                self.times[index] = float(t_s)
            self.contacts[index]['wall'] |= bool(wall_contact[index])
            self.contacts[index]['car'] |= bool(car_contact)
        self._sim_time = float(t_s)
        self._has_tick = True
        self._finished = bool(car_contact or any(wall_contact) or all(
            value is not None for value in self.times))

    @property
    def finished(self):
        return self._finished

    @property
    def sim_time_s(self):
        return self._sim_time

    @property
    def progress_m(self):
        """Snapshot of the per-car ground-truth progress traces, in metres."""
        return [list(trace) for trace in self.traces]

    def result(self):
        """Strict JSON standings. null means unfinished/undefined, never zero.

        finish_order/winner use 1-based car numbers; gap_changes retains
        eval_gap_multi's 0-based pair/leader/trailer indices.
        missed_steps counts absent simulator steps, including startup loss.
        Persistence counts restart after each gap; pair-gap speed uses the full
        elapsed step interval, so received samples are not falsely consecutive.
        """
        finished = sorted((i for i, value in enumerate(self.times) if value is not None),
                          key=lambda i: (self.times[i], i))
        unfinished = sorted((i for i, value in enumerate(self.times) if value is None),
                            key=lambda i: (-self.counters[i].laps, i))
        gap = (self.times[finished[1]] - self.times[finished[0]]
               if len(finished) >= 2 else math.inf)
        elapsed = ((self._sample_steps[-1] - self._sample_steps[0]) * self.dt_s
                   if len(self._sample_steps) >= 2 else None)
        changes = pair_gap_changes(self.traces, self.times, self.dt_s, elapsed)
        overtakes = [0] * len(self.counters)
        stalls = [0] * len(self.counters)
        boundaries = [0, *self._breaks, len(self._sample_steps)]
        for start, end in zip(boundaries, boundaries[1:]):
            passes = count_overtakes([trace[start:end] for trace in self.traces], self.dt_s)
            stops = count_stalls([speed[start:end] for speed in self.speeds], self.dt_s)
            overtakes = [a + b for a, b in zip(overtakes, passes)]
            stalls = [a + b for a, b in zip(stalls, stops)]
        contacts = [dict(value) for value in self.contacts]
        winner = next((i + 1 for i in finished if not any(contacts[i].values())), None)
        return json_safe({
            'finish_order': [i + 1 for i in finished + unfinished],
            'finish_time_s': list(self.times),
            'laps': [counter.laps for counter in self.counters],
            'gap_s': gap,
            'overtakes': overtakes,
            'stalls': stalls,
            'gap_changes': changes,
            'time_gained_s': time_gained(changes, len(self.counters)),
            'contacts': contacts,
            'winner': winner,
            'sim_time_s': self._sim_time,
            'steps': self.steps,
            'missed_steps': self.missed_steps,
        })
