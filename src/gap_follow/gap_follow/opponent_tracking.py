"""
opponent_tracking.py

Moving-obstacle awareness for gap_follow: finds other cars in the scan,
tracks them from scan to scan, and tells the gap search where they are about
to be. Importable and testable without rclpy, like gap_logic.py, and used
unchanged by both gap_follow_node.py and the simulator's mirror
(tools/f1tenth_sim/gap_mirror.py).

Off unless ``enable_opponent_awareness`` is true. With it off nothing here is
constructed and the node's output is exactly what it was before this file
existed (test_gap_mirror_parity / test_opponent_awareness_off_is_identical).

Why it exists. Follow-the-gap treats every LiDAR return as a fixed wall. A
car ahead is a "wall" that drives off, so gap_follow will happily pass it
with 0.15 m of room because the body's own swept path is clear -- and when
the other car turns into the next bend across that path, there is no room
left to stop. Measured in simulation on 2026-10-21 (docs/gap-follow.md,
"Several cars at once"). This layer adds the one thing that was missing: a
notion that some returns move, and where they are going.

The pipeline, once per scan:

  segment_objects    -> free-standing, car-sized clusters in the scan
  EgoPose.advance    -> dead-reckoned pose of this car (speed + steering)
  OpponentTracker    -> clusters associated across scans, alpha-beta filtered
                        into position and velocity in a fixed frame
  inflate_predicted  -> the gap search sees each *moving* car's footprint
                        swept along its predicted path, widened by a passing
                        margin, so it will only pass with real room
  follow_speed_cap   -> speed ceiling from the gap to a moving car ahead in
                        the path: (gap - min_gap) / time_gap

Nothing here touches the safety layers. TTC, the contact floor, the forward
reserve, swept-path clearance and the arc guard all keep reading the real
scan. The virtual footprints feed gap *selection* only, and the follow cap
can only lower speed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class OpponentParams:
    """Every tunable, read from the node's ``opponent_*`` parameters.

    Defaults are the shipped values in config/gap_follow.yaml; see the
    comments there for where each number comes from.
    """
    min_width: float = 0.15          # m, cluster chord, smallest car view
    max_width: float = 0.70          # m, a 0.30 x 0.58 car seen diagonally
    max_range: float = 6.0           # m, beyond this clusters are ignored
    segment_jump: float = 0.25       # m, range step that splits clusters
    min_beams: int = 3
    gate: float = 0.5                # m, association radius (plus v*dt)
    alpha: float = 0.5               # position gain
    beta: float = 0.15               # velocity gain
    min_hits: int = 8                # scans before a track can be "moving"
    max_misses: int = 6              # scans unseen before a track is dropped
    moving_speed: float = 0.6        # m/s, world-frame speed to count as moving
    stopped_release_sec: float = 1.0  # s, follow-only hold after moving releases
    follow_memory_sec: float = 1.0    # s, the follow cap held after its car is lost (0: off)
    radius: float = 0.30             # m, rear relevance cutoff only (not a footprint)
    length: float = 0.58             # m, padded opponent length
    width: float = 0.31              # m, tire width plus 5 mm padding per side
    passing_margin: float = 0.15     # m, extra room demanded beside it
    prediction_horizon: float = 0.8  # s, how far ahead its path is blocked
    prediction_step: float = 0.1     # s
    follow_min_gap: float = 0.5      # m, bumper-to-car gap at zero speed
    follow_time_gap: float = 1.0     # s
    abort_speed_margin: float = 0.5  # m/s, drop behind an alongside car on refusal
    follow_lateral: float = 0.45     # m, |y| within which a car is "in path"
    relevant_lateral: float = 1.0    # m, |y| beyond which a mover is ignored
    pass_max_steering: float = 0.10  # rad, no new pass while steering harder
    pass_max_heading_deg: float = 15.0  # no pass while the car ahead turns more
    max_track_speed: float = 8.0     # m/s, velocity estimate clamp
    # The rack the dead reckoning turns with: the command, delayed and
    # slewed the way the servo does it. Fitted to run 20261019-200703
    # (tools/f1tenth_sim/sim_fidelity/calibration.py: steering_delay_ticks
    # 3 at 40 Hz, steering_servo_gain, steering_rate_max).
    steer_delay: float = 0.075       # s
    steer_gain: float = 15.0         # 1/s
    steer_rate_max: float = 3.2      # rad/s

    @classmethod
    def from_mapping(cls, values) -> 'OpponentParams':
        """Build from a dict keyed ``opponent_<field>`` (the ROS names)."""
        kwargs = {}
        for name in cls.__dataclass_fields__:
            key = f'opponent_{name}'
            if key in values:
                kwargs[name] = values[key]
        return cls(**kwargs)

    def validate(self):
        """Raise ValueError naming the first unusable value."""
        positive = ('min_width', 'max_width', 'max_range', 'segment_jump',
                    'gate', 'moving_speed', 'radius', 'length', 'width', 'prediction_step',
                    'follow_time_gap', 'follow_lateral', 'relevant_lateral',
                    'pass_max_steering', 'pass_max_heading_deg', 'max_track_speed',
                    'steer_gain', 'steer_rate_max', 'stopped_release_sec')
        for name in positive:
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'opponent_{name} must be positive and finite, got {value!r}')
        for name in ('passing_margin', 'prediction_horizon', 'follow_min_gap',
                     'steer_delay', 'follow_memory_sec', 'abort_speed_margin'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f'opponent_{name} must be >= 0 and finite, got {value!r}')
        if self.min_width >= self.max_width:
            raise ValueError(
                f'opponent_min_width ({self.min_width}) must be below '
                f'opponent_max_width ({self.max_width})')
        for name in ('alpha', 'beta'):
            value = getattr(self, name)
            if not (0.0 < value <= 1.0):
                raise ValueError(f'opponent_{name} must be in (0, 1], got {value!r}')
        for name in ('min_beams', 'min_hits', 'max_misses'):
            value = getattr(self, name)
            if int(value) != value or value < 1:
                raise ValueError(f'opponent_{name} must be a positive integer, got {value!r}')


@dataclass
class Detection:
    """One free-standing, car-sized cluster, in base_link."""
    x: float          # centroid
    y: float
    width: float      # chord between the cluster's end points
    near_x: float     # x of the cluster point nearest the car (base_link)
    lo: int           # first and last beam index in the scan
    hi: int
    near_range: float  # minimum beam range, measured from the LiDAR


def segment_objects(ranges, valid, angles, laser_x, laser_y,
                    params: OpponentParams) -> list:
    """Free-standing, car-sized clusters in one scan.

    A cluster is a run of valid beams whose neighbouring ranges differ by
    less than ``segment_jump``. It is kept only if

    * it has at least ``min_beams`` beams and its nearest point is within
      ``max_range``;
    * its chord (end point to end point) is between ``min_width`` and
      ``max_width``: a wall is longer, a speck is shorter;
    * it is *free-standing*: the beam just outside each end is either
      invalid or lands more than ``segment_jump`` further away. That is
      what separates a car in front of a wall from a stub of the wall
      itself. A cluster touching either end of the scan is unknown, not
      free-standing, and is dropped.
    """
    r = np.asarray(ranges, dtype=float)
    ok = np.asarray(valid, dtype=bool) & np.isfinite(r)
    a = np.asarray(angles, dtype=float)
    n = r.size
    if n == 0 or a.size != n:
        return []
    out = []
    i = 0
    while i < n:
        if not ok[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and ok[j + 1] and abs(r[j + 1] - r[j]) < params.segment_jump:
            j += 1
        lo, hi = i, j
        i = j + 1
        if hi - lo + 1 < params.min_beams or lo == 0 or hi == n - 1:
            continue
        seg = r[lo:hi + 1]
        if float(np.min(seg)) > params.max_range:
            continue
        left_open = (not ok[lo - 1]) or r[lo - 1] > r[lo] + params.segment_jump
        right_open = (not ok[hi + 1]) or r[hi + 1] > r[hi] + params.segment_jump
        if not (left_open and right_open):
            continue
        xs = seg * np.cos(a[lo:hi + 1]) + laser_x
        ys = seg * np.sin(a[lo:hi + 1]) + laser_y
        width = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        if not (params.min_width <= width <= params.max_width):
            continue
        out.append(Detection(float(np.mean(xs)), float(np.mean(ys)), width,
                             float(np.min(xs)), lo, hi, float(np.min(seg))))
    return out


@dataclass
class EgoPose:
    """This car's pose in a fixed frame, dead-reckoned from speed and
    steering exactly as vesc_to_odom does (kinematic bicycle). Only ever
    used over a second or so, to tell a car that moves from a wall that
    only *appears* to move because this car does."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def advance(self, speed: float, steering: float, wheelbase: float, dt: float):
        yaw_rate = speed * math.tan(steering) / wheelbase
        mid = self.yaw + 0.5 * yaw_rate * dt
        self.x += speed * math.cos(mid) * dt
        self.y += speed * math.sin(mid) * dt
        self.yaw += yaw_rate * dt

    def to_world(self, bx: float, by: float):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * bx - s * by, self.y + s * bx + c * by

    def to_body(self, wx: float, wy: float):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = wx - self.x, wy - self.y
        return c * dx + s * dy, -s * dx + c * dy

    def vector_to_body(self, vx: float, vy: float):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * vx + s * vy, -s * vx + c * vy


class SteeringEstimate:
    """Where the rack probably is, given what was commanded.

    Turning the dead-reckoned pose with the *commanded* angle makes this car
    think it has turned before it has: measured in simulation, its heading
    estimate drifted up to ~25 deg/s while steering changed, enough to make
    a parked box 0.9 m away look like a car doing 1 m/s. The servo is
    modelled instead as a transport delay, then a proportional loop with a
    slew limit -- the same model, with the same fitted constants, the
    simulator drives the car with.
    """

    def __init__(self, delay: float, gain: float, rate_max: float):
        self.delay = delay
        self.gain = gain
        self.rate_max = rate_max
        self.reset()

    def reset(self, angle: float = 0.0):
        self.angle = angle
        self.clock = 0.0
        self.history = [(0.0, angle)]

    def update(self, command: float, dt: float) -> float:
        """Advance by ``dt`` with ``command`` the latest commanded angle;
        returns the estimated rack angle over that interval. ``command`` has
        been in force since the start of the interval (it is the last one
        published), so that is the time it is filed under."""
        self.history.append((self.clock, command))
        self.clock += dt
        # The newest command at least `delay` old; before any is, the angle
        # the estimate was reset to (history[0] at time 0).
        cutoff = self.clock - self.delay + 1e-12
        while len(self.history) > 1 and self.history[1][0] <= cutoff:
            self.history.pop(0)
        delayed = self.history[0][1]
        rate = max(-self.rate_max, min(self.rate_max, self.gain * (delayed - self.angle)))
        self.angle += rate * dt
        return self.angle


@dataclass
class Track:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    hits: int = 1
    misses: int = 0
    streak: int = 1        # consecutive scans detected, reset by a miss
    moving: bool = False
    near_dx: float = 0.0   # body-frame x of the nearest point minus centroid x
    track_id: int = 0
    confirmed_car: bool = False  # only latched by the existing moving rule
    stopped_sec: float = 0.0     # time since moving was last true
    near_range: float = math.nan  # last observed minimum LiDAR range

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)


@dataclass
class OpponentTracker:
    """Nearest-neighbour association plus an alpha-beta filter per track,
    in EgoPose's fixed frame.

    ``moving`` needs ``min_hits`` *consecutive* detections and a speed at or
    above ``moving_speed``. Consecutive, because a real car in view is a
    cluster on every scan, while the false movers measured in simulation
    were wall fragments that flicker in and out of segment_objects -- each
    reappearance a little off, which the filter reads as motion. Once set it
    holds until the speed falls below half the threshold (or the track is
    dropped), so a car that brakes behind a third one is not forgotten the
    instant it slows, nor when it is hidden for a scan or two.
    """
    params: OpponentParams
    tracks: list = field(default_factory=list)
    _next_id: int = 1

    def reset(self):
        self.tracks = []

    def update(self, detections_world, dt: float):
        """Detections: (world x, world y, body near_dx, LiDAR near_range).

        Three-element synthetic observations remain usable for tracking-only
        callers; only scan observations supply a range for inflation.
        """
        p = self.params
        for t in self.tracks:
            t.x += t.vx * dt
            t.y += t.vy * dt
        unused = list(range(len(detections_world)))
        # Greedy: closest pairs first, each detection used once.
        pairs = []
        for ti, t in enumerate(self.tracks):
            gate = p.gate + t.speed * dt
            for di in unused:
                dx = detections_world[di][0] - t.x
                dy = detections_world[di][1] - t.y
                d = math.hypot(dx, dy)
                if d <= gate:
                    pairs.append((d, ti, di))
        pairs.sort()
        used_t, used_d = set(), set()
        for _d, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            t = self.tracks[ti]
            zx, zy, near_dx, *near_range = detections_world[di]
            rx, ry = zx - t.x, zy - t.y
            t.x += p.alpha * rx
            t.y += p.alpha * ry
            t.vx += p.beta / dt * rx
            t.vy += p.beta / dt * ry
            sp = t.speed
            if sp > p.max_track_speed:
                t.vx *= p.max_track_speed / sp
                t.vy *= p.max_track_speed / sp
            t.near_dx = near_dx
            if near_range:
                t.near_range = near_range[0]
            t.hits += 1
            t.streak += 1
            t.misses = 0
        for ti, t in enumerate(self.tracks):
            if ti not in used_t:
                t.misses += 1
                t.streak = 0
        self.tracks = [t for t in self.tracks if t.misses <= p.max_misses]
        for di, det in enumerate(detections_world):
            if di not in used_d:
                self.tracks.append(Track(det[0], det[1], near_dx=det[2],
                                         near_range=det[3] if len(det) > 3 else math.nan,
                                         track_id=self._next_id))
                self._next_id += 1
        for t in self.tracks:
            if t.moving:
                t.moving = t.speed >= 0.5 * p.moving_speed
            else:
                t.moving = t.streak >= p.min_hits and t.speed >= p.moving_speed
            if t.moving:
                t.confirmed_car = True
                t.stopped_sec = 0.0
            elif t.confirmed_car:
                t.stopped_sec += dt


def _rectangle(opponent, laser_x, laser_y, params):
    """Centre and heading of a visible-face observation, in base_link."""
    ox, oy, vx, vy, near_range = opponent
    dx, dy = ox - laser_x, oy - laser_y
    distance = math.hypot(dx, dy)
    ux, uy = (dx / distance, dy / distance) if distance > 0.0 else (1.0, 0.0)
    speed = math.hypot(vx, vy)
    hx, hy = ((vx / speed, vy / speed) if speed >= 0.5 * params.moving_speed
              else (ux, uy))
    support = (params.length / 2 * abs(ux * hx + uy * hy)
               + params.width / 2 * abs(-ux * hy + uy * hx))
    return (laser_x + ux * (near_range + support),
            laser_y + uy * (near_range + support), hx, hy)


def inflate_predicted(window, beam_angles, opponents_body, laser_x, laser_y,
                      params: OpponentParams, floor: float = 0.05):
    """A copy of ``window`` in which each opponent's footprint, swept along
    its predicted path, reads as an obstacle.

    ``opponents_body`` is (x, y, vx, vy, near_range) in base_link, velocity
    relative to the ground (not to this car: the space the other car will
    occupy does not depend on how this one moves). Let P be the LiDAR,
    c the visible cluster centroid, u=(c-P)/|c-P|, h the ground-velocity
    heading, and h_perp its perpendicular. Recover the centre as
    P + u * (near_range + s), with support distance
    s = length/2 * |u.h| + width/2 * |u.h_perp|. The three exact support
    cases are a rear view (s=L/2), a side view (s=W/2), and a 45-degree
    corner view (s=(L/2+W/2)/sqrt(2)). Below half moving_speed use h=u.

    Half-extents are length/2+passing_margin and width/2+passing_margin.
    At every prediction_step through prediction_horizon, intersect rays
    with the oriented rectangle using closed-form slabs. Entry equals
    near_range-passing_margin for straight rear and side views. Never
    lengthen a beam; clamp new entries to floor and preserve NaN beams.
    """
    out = np.array(window, dtype=float, copy=True)
    if not opponents_body:
        return out
    ang = np.asarray(beam_angles, dtype=float)
    cos_a, sin_a = np.cos(ang), np.sin(ang)
    half_l = params.length / 2 + params.passing_margin
    half_w = params.width / 2 + params.passing_margin
    steps = int(math.floor(params.prediction_horizon / params.prediction_step + 1e-9))
    for opponent in opponents_body:
        _ox, _oy, vx, vy, near_range = opponent
        if not math.isfinite(near_range) or near_range < 0.0:
            continue
        centre_x, centre_y, hx, hy = _rectangle(opponent, laser_x, laser_y, params)
        directions = (cos_a * hx + sin_a * hy, -cos_a * hy + sin_a * hx)
        for k in range(steps + 1):
            t = k * params.prediction_step
            cx = centre_x + vx * t - laser_x
            cy = centre_y + vy * t - laser_y
            centres = (cx * hx + cy * hy, -cx * hy + cy * hx)
            entry = np.zeros_like(ang)
            exit_range = np.full_like(ang, math.inf)
            for direction, centre, extent in zip(directions, centres, (half_l, half_w)):
                parallel = np.abs(direction) < 1e-12
                divisor = np.where(parallel, 1.0, direction)
                a = (centre - extent) / divisor
                b = (centre + extent) / divisor
                slab_entry = np.where(parallel, -math.inf, np.minimum(a, b))
                slab_exit = np.where(parallel, math.inf, np.maximum(a, b))
                slab_exit = np.where(parallel & (abs(centre) > extent), -math.inf, slab_exit)
                entry = np.maximum(entry, slab_entry)
                exit_range = np.minimum(exit_range, slab_exit)
            hit = exit_range >= entry
            entry = np.maximum(entry, floor)
            out = np.where(hit, np.minimum(out, entry), out)
    return out


def pass_allowed(opponents_body, steering: float, params: OpponentParams) -> bool:
    """Whether this tick is one to pass on: a straight, not a bend.

    No if this car is steering harder than ``pass_max_steering``, or if any
    car ahead is heading more than ``pass_max_heading_deg`` away from this
    car's heading -- it is turning, so the road is. The failure this exists
    for, measured in simulation: a car passing on the inside of a bend, 0.6
    to 0.8 m to the side (outside the follow band), was squeezed into
    contact when the car being passed turned in. Passes already under way --
    the other car alongside -- are not judged here; see follow_speed_cap.

    ``opponents_body``: list of (x, y, vx, vy, near_x) in base_link.
    """
    if abs(steering) > params.pass_max_steering:
        return False
    limit = math.radians(params.pass_max_heading_deg)
    for opponent in opponents_body:
        vx, vy = opponent[2:4]
        if abs(math.atan2(vy, vx)) > limit:
            return False
    return True


def follow_speed_cap(opponents_body, front_x: float, params: OpponentParams,
                     lateral: float = None) -> float:
    """Speed ceiling for following the nearest moving car ahead in the path.

    ``(gap - follow_min_gap) / follow_time_gap``, where gap is from this
    car's front bumper (``front_x`` in base_link) to the other car's nearest
    point. Only cars with ``|y| <= lateral`` (default ``follow_lateral``:
    directly ahead; the caller widens it to ``relevant_lateral`` when passing
    is not allowed) and their nearest point ahead of the bumper count: one already alongside is being passed, and
    capping speed then would park this car beside it. ``inf`` when nothing
    qualifies. Deliberately does not use the other car's estimated speed,
    so a wrong velocity estimate cannot make it less cautious.

    ``opponents_body``: list of (near_x, y) in base_link.
    """
    lateral = params.follow_lateral if lateral is None else lateral
    cap = math.inf
    for near_x, y in opponents_body:
        if abs(y) > lateral or near_x <= front_x:
            continue
        gap = near_x - front_x
        cap = min(cap, max(0.0, (gap - params.follow_min_gap) / params.follow_time_gap))
    return cap


def avoidance_usable(avoid, plain, *, gap_width=math.inf, required_width=0.0) -> bool:
    """Whether the gap found with opponents inflated may replace the plain one.

    Both are ``(gap_start, gap_end, used_fallback)`` from
    gap_logic.find_gap_with_fallback on the same window. The inflated search
    may choose *which side* of the opening to take -- that is what passing
    with room is -- but never a different opening, and never a shallower
    one: measured in simulation on a 1.2 m loop, a car seen across a
    hairpin shortened the beams through the hairpin, and the inflated search
    sent the follower into a dead-end corner it could not creep out of.
    When this says no, the caller uses the plain gap and the follow cap.
    The physical gap beside the inflated opponent must also fit this car
    plus a passing margin on each side. Recheck every tick, including passes
    already under way: no passing latch overrides a newly narrow section.
    """
    a_start, a_end, a_fallback = avoid
    p_start, p_end, p_fallback = plain
    if a_start is None:
        return False
    if not gap_width >= required_width:
        return False
    if p_start is None:
        return True       # plain found nothing; avoid's gap is all there is
    if a_fallback and not p_fallback:
        return False
    return a_start <= p_end and p_start <= a_end


# A step between two calls that is longer than this, or not forward, starts a
# new clock: the same staleness rule OpponentAwareness.observe applies to a
# scan interval.
_MAX_STEP_SEC = 0.5

# Roundoff allowance on the hold's inclusive end, in seconds. The same allowance
# _held_body makes for its release tick, against a 0.025 s scan period.
_HOLD_TOLERANCE_SEC = 1e-12


class FollowCapMemory:
    """Holds the last finite follow cap for ``memory_sec`` after the follow cap
    disappears (the followed car's track was dropped or left the relevant band).
    Speed-only: it can only lower the commanded speed. 0 disables it.

    ``update(follow_cap, t_s)`` takes this tick's cap (inf for no cap; NaN is
    no cap too) and the time in seconds: the scan stamp in the node, the step
    count times CONTROL_DT in the simulator. It returns ``(cap, from_memory)``.
    A finite cap is stored with its time and returned as it is. An infinite cap
    within ``memory_sec`` (inclusive) of the stored time comes back as the stored
    cap, with from_memory True. Otherwise the answer is ``(inf, False)``. NaN
    clears the memory, and so does a time that is not finite, or a step since
    the last call that is not in (0, 0.5] s.
    """

    def __init__(self, memory_sec: float):
        self.memory_sec = memory_sec
        self.reset()

    def reset(self):
        self._cap = math.inf      # the last finite cap stored
        self._cap_t = None        # when it was stored; None: nothing is held
        self._last_t = None       # the time of the previous call

    def update(self, follow_cap: float, t_s: float) -> tuple:
        if self.memory_sec == 0.0:
            return follow_cap, False
        if not math.isfinite(t_s):
            self.reset()
            return (math.inf if math.isnan(follow_cap) else follow_cap), False
        if self._last_t is not None and not (0.0 < t_s - self._last_t <= _MAX_STEP_SEC):
            self._cap, self._cap_t = math.inf, None
        self._last_t = t_s
        if math.isnan(follow_cap):
            self._cap, self._cap_t = math.inf, None
            return math.inf, False
        if follow_cap != math.inf:
            # A finite cap is stored. -inf is not a cap at all and is passed
            # through untouched: the memory must never raise a speed.
            if math.isfinite(follow_cap):
                self._cap, self._cap_t = follow_cap, t_s
            return follow_cap, False
        if self._cap_t is not None and t_s - self._cap_t <= self.memory_sec + _HOLD_TOLERANCE_SEC:
            return self._cap, True
        return math.inf, False


class OpponentAwareness:
    """The whole layer for one car: what gap_follow_node and the mirror call.

    Per scan: ``observe(...)`` once, then ``augment(window, beam_angles)``
    for the gap search, ``may_pass(steering)`` and ``follow_cap(passing)``
    for the speed chain, then ``follow_memory.update(follow_cap, t_s)`` on the
    follow cap before the stops that come after it. ``reset`` clears the
    memory with the rest of the layer's state.
    """

    def __init__(self, params: OpponentParams, wheelbase: float,
                 laser_x: float, laser_y: float, car_length: float):
        params.validate()
        self.params = params
        self.wheelbase = wheelbase
        self.laser_x = laser_x
        self.laser_y = laser_y
        # Front bumper in base_link: body centred on the wheelbase midpoint
        # (gap_logic.vehicle_boundary_distances' model).
        self.front_x = wheelbase / 2.0 + car_length / 2.0
        self.centre_x = wheelbase / 2.0
        self.ego = EgoPose()
        self.tracker = OpponentTracker(params)
        self.steer = SteeringEstimate(params.steer_delay, params.steer_gain,
                                      params.steer_rate_max)
        self.detections = []
        self.follow_memory = FollowCapMemory(params.follow_memory_sec)

    def reset(self, steering: float = 0.0):
        self.ego = EgoPose()
        self.tracker.reset()
        self.steer.reset(steering)
        self.detections = []
        self.follow_memory.reset()

    def observe(self, ranges, valid, angles, speed: float, steering: float, dt: float):
        """Advance the ego pose over ``dt`` and fold this scan in.

        A ``dt`` that is not positive and finite, or longer than half a
        second, means the history no longer describes the present: it is
        discarded and tracking starts again from this scan.
        """
        if not (math.isfinite(dt) and 0.0 < dt <= 0.5) or not math.isfinite(speed):
            self.reset(steering if math.isfinite(steering) else 0.0)
            dt = None
        else:
            self.ego.advance(speed, self.steer.update(steering, dt), self.wheelbase, dt)
        self.detections = segment_objects(
            ranges, valid, angles, self.laser_x, self.laser_y, self.params)
        world = []
        for d in self.detections:
            wx, wy = self.ego.to_world(d.x, d.y)
            world.append((wx, wy, d.near_x - d.x, d.near_range))
        if dt is None:
            for wx, wy, near_dx, near_range in world:
                self.tracker.tracks.append(Track(wx, wy, near_dx=near_dx,
                                                 near_range=near_range,
                                                 track_id=self.tracker._next_id))
                self.tracker._next_id += 1
        else:
            self.tracker.update(world, dt)

    def moving_body(self):
        """Moving tracks: (x, y, vx, vy, near_x, LiDAR near_range).

        Relevant: not behind this car, within ``relevant_lateral`` to either
        side, and moving the same general way (forward ground velocity in
        this car's frame). The last two drop cars on the far side of a wall
        or a hairpin, seen through an opening -- in simulation, inflating
        one of those bent the follower's line into a dead-end corner. A car
        that matters is one this car could catch, follow or pass.
        """
        p = self.params
        out = []
        for t in self.tracker.tracks:
            if not t.moving:
                continue
            bx, by = self.ego.to_body(t.x, t.y)
            if (bx <= -p.radius or abs(by) > p.relevant_lateral
                    or math.hypot(bx, by) > p.max_range):
                continue
            bvx, bvy = self.ego.vector_to_body(t.vx, t.vy)
            if bvx < 0.0:
                continue
            out.append((bx, by, bvx, bvy, bx + t.near_dx, t.near_range))
        return out

    def augment(self, window, beam_angles):
        movers = self.moving_body()
        return inflate_predicted(
            window, beam_angles, [(*m[:4], m[5]) for m in movers],
            self.laser_x, self.laser_y, self.params)

    def followed_body(self):
        """Moving cars plus recently stopped confirmed cars, for speed only.

        Held tracks never feed inflation or passing. At low speed velocity
        noise cannot establish heading, so only the spatial relevance gates
        apply. A never-moving cluster cannot acquire this follow-only hold.
        """
        return self.moving_body() + self._held_body()

    def _held_body(self):
        """Relevant confirmed cars whose moving latch has recently released."""
        out = []
        p = self.params
        for t in self.tracker.tracks:
            # Accumulating 40 * 0.025 can exceed 1.0 by roundoff. Preserve
            # the inclusive time boundary without extending it by a scan.
            if (t.moving or not t.confirmed_car
                    or t.stopped_sec > p.stopped_release_sec + 1e-12):
                continue
            bx, by = self.ego.to_body(t.x, t.y)
            if (bx <= -p.radius or abs(by) > p.relevant_lateral
                    or math.hypot(bx, by) > p.max_range):
                continue
            bvx, bvy = self.ego.vector_to_body(t.vx, t.vy)
            out.append((bx, by, bvx, bvy, bx + t.near_dx, t.near_range))
        return out

    def may_pass(self, steering: float) -> bool:
        return pass_allowed(self.moving_body(), steering, self.params)

    def passing_gap_width(self, window, beam_angles, gap):
        """Physical room on the selected side, before ego-width inflation.

        Project sensed returns into base_link. Beside each opponent's
        predicted longitudinal footprint, the nearest return outside its
        inflated side is the wall bounding that side's gap. Check every
        prediction step, so a narrowing ahead vetoes passing immediately.
        The selected gap's middle bearing chooses the side; a gap pointing
        into the opponent has zero room. This measures room only for the
        pass decision; all driving safety checks retain the original scan.
        """
        start, end, _fallback = gap
        if start is None:
            return 0.0
        a = np.asarray(beam_angles, dtype=float)
        r = np.asarray(window, dtype=float)
        xs = self.laser_x + r * np.cos(a)
        ys = self.laser_y + r * np.sin(a)
        bearing = float(a[(start + end) // 2])
        room = math.inf
        p = self.params
        steps = int(math.floor(p.prediction_horizon / p.prediction_step + 1e-9))
        for m in self.moving_body():
            centre_x, centre_y, hx, hy = _rectangle(
                (*m[:4], m[5]), self.laser_x, self.laser_y, p)
            half_l = p.length / 2 + p.passing_margin
            half_w = p.width / 2 + p.passing_margin
            extent_x = abs(hx) * half_l + abs(hy) * half_w
            extent_y = abs(hy) * half_l + abs(hx) * half_w
            for k in range(steps + 1):
                cx = centre_x + m[2] * k * p.prediction_step
                cy = centre_y + m[3] * k * p.prediction_step
                target_y = self.laser_y + (cx - self.laser_x) * math.tan(bearing)
                side = 1.0 if target_y > cy else -1.0
                if abs(target_y - cy) <= extent_y:
                    return 0.0
                beside = (np.isfinite(r) & (xs >= cx - extent_x) & (xs <= cx + extent_x)
                          & (side * (ys - cy) > extent_y))
                if np.any(beside):
                    room = min(room, float(np.min(side * (ys[beside] - cy))) - extent_y)
        return room

    def follow_cap(self, passing: bool = True) -> float:
        """min(follow_law_cap(passing), alongside_cap(passing)), in m/s."""
        return min(self.follow_law_cap(passing), self.alongside_cap(passing))

    def follow_law_cap(self, passing: bool = True) -> float:
        """The gap-only follow law over cars ahead of our bumper, in m/s.

        Moving cars use the pass-dependent band; held cars always use the
        full relevance band, regardless of another car permitting a pass.
        This is the part FollowCapMemory may hold after the car is lost.
        """
        moving_cap = follow_speed_cap(
            [(m[4], m[1]) for m in self.moving_body()], self.front_x, self.params,
            lateral=None if passing else self.params.relevant_lateral)
        held_cap = follow_speed_cap(
            [(m[4], m[1]) for m in self._held_body()], self.front_x, self.params,
            lateral=self.params.relevant_lateral)
        return min(moving_cap, held_cap)

    def alongside_cap(self, passing: bool = True) -> float:
        """The alongside-abort ceiling, in m/s; inf when it does not apply.

        When passing is refused, each relevant moving or held car whose
        observed rear/nearest face is at or behind our front bumper, and
        whose centre is strictly ahead of ours, also imposes
        max(0, hypot(vx, vy) - abort_speed_margin), in m/s. This
        alongside-abort ceiling lets the other car pull ahead; once its face
        is ahead of the bumper the unchanged gap-only follow law takes over.
        Only the car that is behind yields: two cars running the same rule
        side by side would otherwise each cap below the other and ratchet
        both to zero (measured 2026-10-29, car1_course reversed, 0 of 2
        finished at a 2.5 m/s leader). With centres level neither yields.
        Relevance is exactly moving_body/_held_body's spatial band (including
        their rear cutoff), not an unbounded region behind us. With passing
        permitted, or no alongside car, this adds no ceiling. All ceilings
        combine by minimum and can only lower the commanded speed.

        It applies only on scans that see the alongside car, and is never
        held by FollowCapMemory: a wall segment briefly tracked as a mover
        beside the car would otherwise brake it for the whole memory window
        (measured 2026-10-29: two ticks of a false mover held for 1 s, and
        3 of 36 two-car runs on car 1's course ended wedged).
        """
        abort_cap = math.inf
        if not passing:
            for m in self.moving_body() + self._held_body():
                if m[4] <= self.front_x and m[0] > self.centre_x:
                    abort_cap = min(abort_cap, max(
                        0.0, math.hypot(m[2], m[3]) - self.params.abort_speed_margin))
        return abort_cap
