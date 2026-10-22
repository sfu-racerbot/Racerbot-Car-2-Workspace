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
    radius: float = 0.30             # m, opponent footprint around its centroid
    passing_margin: float = 0.15     # m, extra room demanded beside it
    prediction_horizon: float = 0.8  # s, how far ahead its path is blocked
    prediction_step: float = 0.1     # s
    follow_min_gap: float = 0.5      # m, bumper-to-car gap at zero speed
    follow_time_gap: float = 1.0     # s
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
                    'gate', 'moving_speed', 'radius', 'prediction_step',
                    'follow_time_gap', 'follow_lateral', 'relevant_lateral',
                    'pass_max_steering', 'pass_max_heading_deg', 'max_track_speed',
                    'steer_gain', 'steer_rate_max')
        for name in positive:
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'opponent_{name} must be positive and finite, got {value!r}')
        for name in ('passing_margin', 'prediction_horizon', 'follow_min_gap',
                     'steer_delay'):
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
                             float(np.min(xs)), lo, hi))
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
        """``detections_world``: list of (x, y, near_dx) in the fixed frame."""
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
            zx, zy, near_dx = detections_world[di]
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
                                         track_id=self._next_id))
                self._next_id += 1
        for t in self.tracks:
            if t.moving:
                t.moving = t.speed >= 0.5 * p.moving_speed
            else:
                t.moving = t.streak >= p.min_hits and t.speed >= p.moving_speed


def inflate_predicted(window, beam_angles, opponents_body, laser_x, laser_y,
                      params: OpponentParams, floor: float = 0.05):
    """A copy of ``window`` in which each opponent's footprint, swept along
    its predicted path, reads as an obstacle.

    ``opponents_body`` is a list of (x, y, vx, vy) in base_link, velocity
    relative to the ground (not to this car: the space the other car will
    occupy does not depend on how this one moves). The footprint is a disc
    of ``radius + passing_margin`` at every ``prediction_step`` from 0 to
    ``prediction_horizon``. Each beam is shortened to where it would enter
    the nearest disc, never lengthened, and never below ``floor``.
    """
    out = np.array(window, dtype=float, copy=True)
    if not opponents_body:
        return out
    ang = np.asarray(beam_angles, dtype=float)
    cos_a, sin_a = np.cos(ang), np.sin(ang)
    radius = params.radius + params.passing_margin
    steps = int(math.floor(params.prediction_horizon / params.prediction_step + 1e-9))
    for ox, oy, vx, vy in opponents_body:
        for k in range(steps + 1):
            t = k * params.prediction_step
            cx = ox + vx * t - laser_x
            cy = oy + vy * t - laser_y
            along = cx * cos_a + cy * sin_a
            perp_sq = cx * cx + cy * cy - along * along
            inside = cx * cx + cy * cy <= radius * radius
            hit = (perp_sq <= radius * radius) & ((along > 0.0) | inside)
            if not np.any(hit):
                continue
            entry = along - np.sqrt(np.maximum(radius * radius - perp_sq, 0.0))
            entry = np.where(inside, floor, np.maximum(entry, floor))
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
    for _x, _y, vx, vy, _near_x in opponents_body:
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


def avoidance_usable(avoid, plain) -> bool:
    """Whether the gap found with opponents inflated may replace the plain one.

    Both are ``(gap_start, gap_end, used_fallback)`` from
    gap_logic.find_gap_with_fallback on the same window. The inflated search
    may choose *which side* of the opening to take -- that is what passing
    with room is -- but never a different opening, and never a shallower
    one: measured in simulation on a 1.2 m loop, a car seen across a
    hairpin shortened the beams through the hairpin, and the inflated search
    sent the follower into a dead-end corner it could not creep out of.
    When this says no, the caller uses the plain gap and the follow cap.
    """
    a_start, a_end, a_fallback = avoid
    p_start, p_end, p_fallback = plain
    if a_start is None:
        return False
    if p_start is None:
        return True       # plain found nothing; avoid's gap is all there is
    if a_fallback and not p_fallback:
        return False
    return a_start <= p_end and p_start <= a_end


class OpponentAwareness:
    """The whole layer for one car: what gap_follow_node and the mirror call.

    Per scan: ``observe(...)`` once, then ``augment(window, beam_angles)``
    for the gap search, ``may_pass(steering)`` and ``follow_cap(passing)``
    for the speed chain.
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
        self.ego = EgoPose()
        self.tracker = OpponentTracker(params)
        self.steer = SteeringEstimate(params.steer_delay, params.steer_gain,
                                      params.steer_rate_max)
        self.detections = []

    def reset(self, steering: float = 0.0):
        self.ego = EgoPose()
        self.tracker.reset()
        self.steer.reset(steering)
        self.detections = []

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
            world.append((wx, wy, d.near_x - d.x))
        if dt is None:
            for wx, wy, near_dx in world:
                self.tracker.tracks.append(Track(wx, wy, near_dx=near_dx,
                                                 track_id=self.tracker._next_id))
                self.tracker._next_id += 1
        else:
            self.tracker.update(world, dt)

    def moving_body(self):
        """Relevant moving tracks as (x, y, vx, vy, near_x) in base_link.

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
            out.append((bx, by, bvx, bvy, bx + t.near_dx))
        return out

    def augment(self, window, beam_angles):
        movers = self.moving_body()
        return inflate_predicted(
            window, beam_angles, [m[:4] for m in movers],
            self.laser_x, self.laser_y, self.params)

    def may_pass(self, steering: float) -> bool:
        return pass_allowed(self.moving_body(), steering, self.params)

    def follow_cap(self, passing: bool = True) -> float:
        """Follow cap for cars directly ahead when passing is allowed, and for
        every relevant car ahead when it is not."""
        return follow_speed_cap(
            [(m[4], m[1]) for m in self.moving_body()], self.front_x, self.params,
            lateral=None if passing else self.params.relevant_lateral)
