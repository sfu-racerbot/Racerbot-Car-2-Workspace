"""Launch boost: full speed off the start line, once per race.

No ROS imports, so it is testable on its own (test/test_launch_boost.py), and
used unchanged by gap_follow_node.py and the simulator's mirror
(tools/f1tenth_sim/gap_mirror.py).

Why. Cars start side by side and the first corner can be as little as 2 m
away. Whoever reaches it first takes the line, and a car that is behind there
has to find room to pass. Recorded standing starts on car 1 (2026-10-26)
took 1.5-2.5 s to cover 2 m: the command climbed through the acceleration
ramp, and on the slow starts sat at the corner-fallback cap. The launch
removes those comfort limits for the first stretch of the race.

What changes while the launch is active (gap_follow_node._scan_step):
  - the speed command jumps to its ceiling instead of rising through
    max_acceleration's ramp,
  - the corner-fallback cap, the narrow-corridor cap and speed_hold are not
    applied,
  - opponent awareness is ignored: no avoidance footprints in the gap search,
    no follow cap, no alongside cap,
  - the car holds straight while its own straight path is clear for its
    stopping distance, so a car alongside does not steer it away. Anything in
    that path (a car cutting in, the end wall) hands steering back to the gap
    search, which still sees the other cars as obstacles.
What never changes: the LB deadman, emergency clearance, TTC, the braking
(clearance) ceiling and the curvature ceiling. A car that cuts in front on a
collision path trips TTC or the clearance ceiling, and either ends the launch.

Lifecycle: ARMED -> ACTIVE -> DONE, and never back. The node calls begin() on
every driving tick (LB held, a gap found, not creeping or escaping, and the
braking ceiling above the car's speed, so there is room to speed up); only the
first one does anything, so releasing and pressing LB again never launches a
second time. Stops before that first driving tick (a stale sensor or no gap
in the moment LB goes down; the simulator always has two) leave it armed:
the launch is spent by driving, not by the press. A new launch needs a new
node (a new gap_follow_launch.py), which is one race.

The launch ends at the first of (step()):
  - 'braking'   the clearance ceiling is at or below the car's speed: the end
                of the straight is close enough that the car must now slow,
  - 'turn'      the steering request is past end_steering (0 turns this
                check off, the shipped value: see below),
  - 'time'      max_sec since it began,
  - 'distance'  max_distance driven since it began,
or end() from the node: any stop (deadman released, TTC, emergency clearance,
no gap, stale sensors) or a TTC / emergency escape tick.

Why the turn check ships off. On a side-by-side grid, a neighbour whose nose
gets ahead fills part of the forward window, and gap_follow aims at the middle
of the free space beside it: about 0.23 rad of request in the 2026-10-30
parity scene (test_gap_mirror_parity.py), on a dead-straight track. A turn
check cannot tell that from a corner, and ending the launch there brings the
follow cap back exactly when the user asked for opponents to be ignored. A
corner does not need it: the curvature ceiling stays on through the launch,
and the wall at the end of the straight ends it through 'braking'.
"""

import math

ARMED = 'armed'
ACTIVE = 'active'
DONE = 'done'


class LaunchBoost:
    """One race start. See the module docstring for what it does."""

    def __init__(self, enabled: bool, max_sec: float, max_distance: float,
                 end_steering: float):
        for name, value in (('max_sec', max_sec), ('max_distance', max_distance)):
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'launch_boost_{name} must be positive and finite, got {value!r}')
        if not (math.isfinite(end_steering) and end_steering >= 0.0):
            raise ValueError(
                f'launch_boost_end_steering must be finite and 0 (off) or more, '
                f'got {end_steering!r}')
        self.max_sec = float(max_sec)
        self.max_distance = float(max_distance)
        self.end_steering = float(end_steering)
        self.state = ARMED if enabled else DONE
        self.end_reason = None if enabled else 'disabled'
        self.elapsed = 0.0
        self.distance = 0.0

    @property
    def active(self) -> bool:
        return self.state == ACTIVE

    @property
    def armed(self) -> bool:
        return self.state == ARMED

    def begin(self) -> bool:
        """This tick drives. Starts the launch the first time only; returns
        whether the launch is active."""
        if self.state == ARMED:
            self.state = ACTIVE
        return self.active

    def end(self, reason: str) -> None:
        """End an active launch for good. Does nothing while armed (a stop
        before the first press is not a start) or once done."""
        if self.state == ACTIVE:
            self.state = DONE
            self.end_reason = reason

    def step(self, dt: float, speed: float, steering: float,
             braking_cap: float) -> bool:
        """Advance one driving tick and check the end conditions.

        ``dt`` is the tick (s), ``speed`` the car's measured speed (m/s),
        ``steering`` the steering request (rad), ``braking_cap`` the clearance
        ceiling (m/s). Anything non-finite ends the launch: it only ever runs
        on numbers it can trust. Returns whether the launch is still active.
        """
        if self.state != ACTIVE:
            return False
        if not all(math.isfinite(v) for v in (dt, speed, steering, braking_cap)):
            self.end('invalid input')
            return False
        self.elapsed += max(0.0, dt)
        self.distance += max(0.0, dt) * max(0.0, speed)
        if braking_cap <= speed:
            self.end('braking')
        elif self.end_steering > 0.0 and abs(steering) > self.end_steering:
            self.end('turn')
        elif self.elapsed >= self.max_sec:
            self.end('time')
        elif self.distance >= self.max_distance:
            self.end('distance')
        return self.active
