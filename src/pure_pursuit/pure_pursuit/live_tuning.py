"""
live_tuning.py

Machinery for changing a driving node's tuning parameters *while it is
running* -- the node side of the web dashboard's live tuning panel (see
docs/web-dashboard.md#live-parameter-tuning).

The problem this solves. Both driving nodes in this workspace read every
parameter once in __init__ and cache it on an instance attribute
(`self.max_speed = float(self.get_parameter('max_speed').value)`), because
re-reading a parameter at 40Hz inside the control loop would be pointless
overhead. That caching means a plain `ros2 param set` -- or any dashboard
built on the standard `/<node>/set_parameters` service -- *succeeds* and
changes nothing: the parameter server dutifully stores the new value and
the control loop keeps using the number it cached at startup. A tuning UI
built on that would confidently display `max_speed: 2.0` while the car
kept driving at 4.0, which is exactly the kind of quiet disagreement
between the display and the machine that gets a car crashed.

So a parameter is only ever "live tunable" here if the node declares it
so, by listing a Tunable below whose `attr` names the cached attribute the
control loop actually reads. Everything else is refused at runtime rather
than silently accepted -- see `review()`.

Two more properties this module exists to guarantee:

  * **The node is the authority on what is safe, not the browser.** Every
    Tunable carries its own `minimum`/`maximum`, enforced here, in the
    node's own process, on every update. The dashboard reads those bounds
    (via the `live_tunable_spec` parameter) only so its sliders can't
    *offer* an out-of-range value in the first place. A hand-rolled
    `ros2 param set`, a stale browser tab, or a hostile client on the LAN
    all hit the same clamp, because it does not live in the browser.

  * **A rejected batch changes nothing at all.** `review()` validates the
    whole batch, including cross-parameter invariants like
    `min_speed <= max_speed`, before the caller applies any of it. Half of
    a speed change landing is worse than none of it landing.

This module deliberately imports no ROS: it is plain data validation, so
it is directly unit-testable (test/test_live_tuning.py) without a running
robot -- the same split `racing_math.py` and `gap_logic.py` already use.

Layout: everything above the "PER-NODE CATALOGUE" banner near the bottom
is generic machinery, byte-for-byte identical between
`pure_pursuit/live_tuning.py` and `gap_follow/live_tuning.py`. This
workspace's packages don't import across package boundaries (see
CLAUDE.md), so that half is duplicated rather than shared, and each
package's test suite asserts the two copies still match so they cannot
drift apart unnoticed. Below the banner is this package's own catalogue:
which parameters its node will accept live and within what bounds, which
is nobody else's business.
"""

from dataclasses import dataclass
import json
import math
from typing import Callable, Dict, Iterable, Optional, Sequence, Tuple

# Bumped if the wire shape of spec_json() ever changes incompatibly, so a
# dashboard talking to an older/newer node can say so instead of guessing.
SPEC_VERSION = 1


@dataclass(frozen=True)
class Tunable:
    """One parameter a node is willing to have changed while it drives.

    name        the ROS parameter name
    group       UI grouping label ("Speed", "Overtaking", ...)
    minimum     hard lower bound, enforced in this process
    maximum     hard upper bound, enforced in this process
    attr        the instance attribute the control loop actually reads;
                defaults to `name` when the two match (they usually do)
    transform   converts the parameter value into what `attr` stores, for
                the cases where they differ (degrees -> radians, say).
                Applied *after* the bounds check, which is always done in
                the parameter's own units.
    kind        'float' or 'bool'
    unit        display suffix ("m/s", "m", "deg"); UI only
    step        slider granularity hint; UI only
    safety      True marks a parameter that widens or narrows a collision
                margin / emergency stop. Purely a display flag -- the
                dashboard renders these groups with a warning treatment so
                nobody drags one thinking it is a lap-time knob.
    description one line explaining what the knob does, shown in the UI
    """

    name: str
    group: str
    minimum: float
    maximum: float
    attr: str = ''
    transform: Optional[Callable[[float], float]] = None
    kind: str = 'float'
    unit: str = ''
    step: float = 0.0
    safety: bool = False
    description: str = ''

    @property
    def target_attr(self) -> str:
        return self.attr or self.name

    @property
    def label(self) -> str:
        return self.name.replace('_', ' ')

    def store(self, value):
        """The value to actually assign to `target_attr`."""
        return self.transform(value) if self.transform is not None else value


def by_name(tunables: Iterable[Tunable]) -> Dict[str, Tunable]:
    return {tunable.name: tunable for tunable in tunables}


def spec_json(node_name: str, tunables: Sequence[Tunable]) -> str:
    """The self-describing catalogue a node publishes as its read-only
    `live_tunable_spec` parameter.

    One JSON string rather than a fistful of parallel string/double arrays
    (`live_tunable_names`, `live_tunable_mins`, ...) because the dashboard
    needs name+group+bounds+units+prose per entry, and keeping those in
    lockstep across five arrays is a bug waiting to happen. It also means
    one `get_parameters` call fetches the entire catalogue, and
    `ros2 param get /<node> live_tunable_spec` is a complete, readable
    answer to "what can I change while this thing is running".
    """
    return json.dumps({
        'version': SPEC_VERSION,
        'node': node_name,
        'params': [
            {
                'name': tunable.name,
                'group': tunable.group,
                'label': tunable.label,
                'kind': tunable.kind,
                'min': float(tunable.minimum),
                'max': float(tunable.maximum),
                'step': float(tunable.step),
                'unit': tunable.unit,
                'safety': bool(tunable.safety),
                'description': tunable.description,
            }
            for tunable in tunables
        ],
    }, separators=(',', ':'))


def _coerce(tunable: Tunable, value):
    """Parameter value -> the type this Tunable expects, or raise ValueError.

    Rejects bool-for-float outright (Python would happily make True into
    1.0) so a checkbox wired to the wrong parameter fails loudly.
    """
    if tunable.kind == 'bool':
        if not isinstance(value, bool):
            raise ValueError(f"'{tunable.name}' expects true/false")
        return value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"'{tunable.name}' expects a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"'{tunable.name}' must be finite")
    return number


def review(
    tunables: Dict[str, Tunable],
    requested: Dict[str, object],
    current: Dict[str, object],
    passthrough: Iterable[str] = (),
    invariants: Sequence[Callable[[Dict[str, object]], Optional[str]]] = (),
) -> Tuple[Dict[str, object], Optional[str]]:
    """Vet a whole batch of runtime parameter changes, all-or-nothing.

    tunables     name -> Tunable, the node's live-tunable whitelist
    requested    name -> new value, straight off the set_parameters call
    current      name -> current value for every tunable, so cross-parameter
                 invariants see the *resulting* configuration, not just the
                 handful of names in this particular batch
    passthrough  parameter names to accept and ignore (they are somebody
                 else's business -- `use_sim_time`, a node's own
                 already-handled special cases)
    invariants   callables given the merged {name: value} result; each
                 returns an error string, or None if satisfied

    Returns (accepted, error). On error, `accepted` is empty and the caller
    must apply nothing: a half-applied speed limit is its own hazard.

    Anything not in `tunables` or `passthrough` is an error rather than a
    silent success -- see this module's docstring for why a quietly ignored
    parameter change is the failure mode worth designing against.
    """
    accepted: Dict[str, object] = {}
    for name, value in requested.items():
        if name in passthrough:
            continue
        tunable = tunables.get(name)
        if tunable is None:
            return {}, (
                f"'{name}' cannot be changed while the node is running. "
                "The control loop caches its parameters at startup, so "
                "accepting this would change the reported value without "
                "changing how the car drives. Restart the node with a new "
                "config to change it."
            )
        try:
            number = _coerce(tunable, value)
        except ValueError as exc:
            return {}, str(exc)
        if tunable.kind != 'bool' and not (tunable.minimum <= number <= tunable.maximum):
            return {}, (
                f"'{name}' must be between {tunable.minimum:g} and "
                f"{tunable.maximum:g}{(' ' + tunable.unit) if tunable.unit else ''}, "
                f"got {number:g}"
            )
        accepted[name] = number

    if not accepted:
        return {}, None

    merged = dict(current)
    merged.update(accepted)
    for invariant in invariants:
        reason = invariant(merged)
        if reason:
            return {}, reason
    return accepted, None


# ============================================================================
# PER-NODE CATALOGUE -- pure_pursuit_node
#
# Everything above this banner is generic and duplicated verbatim in
# gap_follow/live_tuning.py. Everything below is specific to this node.
#
# Every `minimum`/`maximum` here is a *hard* bound enforced in this
# process on every update, not a slider hint. They are deliberately
# narrower than "whatever a float can hold": the point of a live tuning
# panel is to explore a tune between runs, not to reach a value that no
# amount of exploring should reach on a car that shares a room with
# people. Widening one is a code change and a code review, which is the
# correct amount of friction.
#
# Two parameters are conspicuously ABSENT and must stay that way:
#
#   enable_deadman  -- the mandatory workspace LB policy
#       (docs/architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car).
#       Relaxing it is a team decision, not a knob, and certainly not one
#       reachable from an unauthenticated browser on the LAN. review()
#       refuses it like any other non-whitelisted name.
#
#   enable_lidar_safety -- the hard-stop reactive net. Its *thresholds*
#       are tunable below, because a margin that is wrong for the track is
#       a real thing to discover mid-session. Switching the whole net off
#       is not a margin, it is a different vehicle, and a checkbox that
#       does it from a phone is exactly the affordance not to build.
# ============================================================================

PURE_PURSUIT_TUNABLES = (
    # --- Speed envelope: the knobs actually worth a slider trackside ---
    Tunable('max_speed', 'Speed', 0.3, 6.0, unit='m/s', step=0.1,
            description='Ceiling on commanded speed. The recorded speed '
                        'profile is clipped to this, so lowering it slows '
                        'the whole lap without re-recording a line.'),
    Tunable('min_speed', 'Speed', 0.0, 3.0, unit='m/s', step=0.1,
            description='Floor the profiled speed is held to, so the car '
                        'keeps creeping through the slowest corner instead '
                        'of stalling in it.'),
    Tunable('max_acceleration', 'Speed', 1.0, 8.0, unit='m/s^2', step=0.25,
            description='How fast a speed *command* may rise (not a demand '
                        'on the motor). Bounded on both sides: too low and '
                        'the car cannot rebuild speed behind a slower car, '
                        'too high and it arrives too hot to commit to a '
                        'pass. 5-6 is the validated band -- see '
                        'docs/simulator.md before leaving it.'),
    Tunable('max_braking_decel', 'Speed', 1.0, 12.0, unit='m/s^2', step=0.25,
            description='Shapes how fast a normal speed command may fall. '
                        'An emergency stop ignores this and is immediate.'),
    Tunable('max_lateral_accel', 'Speed', 0.5, 6.0, unit='m/s^2', step=0.1,
            description='Cornering aggression: the online cap holds speed '
                        'to sqrt(a_lat / curvature). The single best knob '
                        'for "it understeers wide" vs "it scrubs and slows".'),

    # --- Geometry of how the line is followed ---
    Tunable('min_lookahead', 'Line following', 0.2, 3.0, unit='m', step=0.05,
            description='Lookahead distance at a standstill. Smaller '
                        'corners harder and wobbles sooner; larger cuts '
                        'corners and settles.'),
    Tunable('max_lookahead', 'Line following', 0.5, 5.0, unit='m', step=0.05,
            description='Lookahead ceiling at speed. Must stay at or below '
                        'overtake_lookahead_distance.'),
    Tunable('lookahead_speed_gain', 'Line following', 0.0, 0.6,
            unit='m per m/s', step=0.01,
            description='How much the lookahead stretches with measured '
                        'speed: L = gain*v + min_lookahead, clipped to '
                        '[min, max].'),
    Tunable('max_steering_rate', 'Line following', 0.2, 4.0, unit='rad/s', step=0.1,
            description='Caps how fast the steering angle may slew between '
                        'commands. Lower is calmer and later; higher is '
                        'sharper and can chatter.'),

    # --- Reactive avoidance (steer around, rather than only stopping) ---
    Tunable('enable_obstacle_avoidance', 'Avoidance', 0.0, 1.0, kind='bool',
            description='Steer around something in the forward cone when '
                        'there is room, instead of only hard-stopping for '
                        'it. The hard stop still wins when it is too close.'),
    Tunable('avoidance_trigger_distance', 'Avoidance', 0.3, 4.0, unit='m', step=0.05,
            description='How far ahead a map-absent object starts a steer-'
                        'around. Applies only once a map is available to '
                        'subtract the walls.'),
    Tunable('avoidance_min_gap_distance', 'Avoidance', 0.3, 3.0, unit='m', step=0.05,
            description='How deep an opening must be before it counts as '
                        'somewhere to steer into.'),
    Tunable('avoidance_speed', 'Avoidance', 0.2, 3.0, unit='m/s', step=0.05,
            description='Speed held while actively steering around '
                        'something.'),

    # --- Overtaking ---
    Tunable('enable_opponent_overtake', 'Overtaking', 0.0, 1.0, kind='bool',
            description='Track a detected opponent and attempt a pass. '
                        'Turning this off leaves the car following the '
                        'racing line and braking for traffic.'),
    Tunable('overtake_trigger_gap', 'Overtaking', 0.5, 10.0, unit='m', step=0.1,
            description='Track distance to the opponent at which a pass '
                        'starts being considered.'),
    Tunable('overtake_closing_margin', 'Overtaking', 0.0, 2.0, unit='m/s', step=0.05,
            description='Minimum closing rate before a pass is worth '
                        'attempting. Raise it if the car keeps committing '
                        'to passes it cannot finish.'),
    Tunable('overtake_lateral_offset', 'Overtaking', 0.0, 1.0, unit='m', step=0.05,
            description='Sideways nudge applied to the steering target '
                        'while passing -- how wide the pass runs.'),
    Tunable('overtake_clear_margin', 'Overtaking', 0.0, 5.0, unit='m', step=0.1,
            description='Track distance to get past the opponent before '
                        'rejoining the racing line.'),
    Tunable('overtake_min_side_clearance', 'Overtaking', 0.0, 2.0, unit='m',
            step=0.05,
            description='Room the passing side must have before the car will '
                        'commit. Higher = choosier about passing places.'),

    # --- Safety margins. Real knobs, marked so nobody drags one thinking
    # it is a lap-time setting. The reactive net itself cannot be switched
    # off from here; see the banner above. ---
    Tunable('emergency_stop_distance', 'Safety margins', 0.15, 2.0, unit='m',
            step=0.05, safety=True,
            description='Range inside the forward cone that triggers an '
                        'immediate stop. Lowering it lets the car run '
                        'closer to traffic and leaves less room to be wrong.'),
    Tunable('emergency_stop_clearance', 'Safety margins', 0.0, 0.4, unit='m',
            step=0.01, safety=True,
            description='All-round contact floor measured from the car body '
                        'rather than the sensor -- this is the one that sees '
                        'a wall the car is alongside.'),
    Tunable('safety_fov_deg', 'Safety margins', 20.0, 180.0, unit='deg',
            step=5.0, safety=True,
            description='Width of the forward cone the hard-stop net '
                        'watches. Narrower ignores more of what is beside '
                        'the car.'),
    Tunable('max_cross_track_error', 'Safety margins', 0.3, 5.0, unit='m',
            step=0.1, safety=True,
            description='Stop if the car is further than this from every '
                        'waypoint -- the lost/kidnapped watchdog. Raising it '
                        'buys tolerance for a rough line and costs detection '
                        'of a genuinely lost car.'),
)


def _speed_order(merged):
    if merged['min_speed'] > merged['max_speed']:
        return (f"min_speed ({merged['min_speed']:g}) cannot exceed max_speed "
                f"({merged['max_speed']:g})")
    return None


def _lookahead_order(merged):
    if merged['min_lookahead'] > merged['max_lookahead']:
        return (f"min_lookahead ({merged['min_lookahead']:g}) cannot exceed "
                f"max_lookahead ({merged['max_lookahead']:g})")
    return None


def _overtake_preview(merged):
    # Mirrors the startup check: a pass is a lateral offset from the line,
    # and spread over too short a target it demands a curvature the online
    # lateral-acceleration cap answers by braking -- which stalls the pass
    # instead of completing it. overtake_lookahead_distance is not itself
    # tunable, so this only ever fires on a max_lookahead raise.
    if merged['max_lookahead'] > merged['overtake_lookahead_distance']:
        return (f"max_lookahead ({merged['max_lookahead']:g}) cannot exceed "
                f"overtake_lookahead_distance "
                f"({merged['overtake_lookahead_distance']:g}); a pass needs "
                f"the longer preview horizon")
    return None


PURE_PURSUIT_INVARIANTS = (_speed_order, _lookahead_order, _overtake_preview)

# Values review() needs in `current` to check the invariants above, beyond
# the tunables themselves.
PURE_PURSUIT_INVARIANT_CONTEXT = ('overtake_lookahead_distance',)
