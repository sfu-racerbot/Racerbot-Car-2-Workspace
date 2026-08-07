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
# PER-NODE CATALOGUE -- gap_follow_node
#
# Everything above this banner is generic and duplicated verbatim in
# pure_pursuit/live_tuning.py. Everything below is specific to this node.
#
# Every `minimum`/`maximum` here is a *hard* bound enforced in this
# process on every update, not a slider hint. Widening one is a code
# change and a code review, which is the correct amount of friction for a
# knob that is reachable from a phone while the car is moving.
#
# enable_deadman is conspicuously ABSENT and must stay that way: the
# mandatory workspace LB policy
# (docs/architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)
# is a team decision, not a knob. review() refuses it like any other
# non-whitelisted name.
#
# So are the structural settings -- forward_fov_deg, the footprint
# (car_width/car_length/wheelbase), and the laser offsets. Those describe
# what the car physically *is* and how its scan is framed; they are not
# things to discover between laps, and several of them invalidate the
# cached per-beam boundary geometry the control loop reuses.
# ============================================================================

GAP_FOLLOW_TUNABLES = (
    # --- Speed envelope ---
    Tunable('max_speed', 'Speed', 0.3, 4.0, unit='m/s', step=0.1,
            description='Ceiling on commanded speed on a clear straight.'),
    Tunable('min_speed', 'Speed', 0.0, 3.0, unit='m/s', step=0.1,
            description='Floor for ordinary driving, so the car keeps '
                        'moving through a tight section rather than '
                        'creeping to a halt.'),
    Tunable('corner_speed', 'Speed', 0.1, 2.0, unit='m/s', step=0.05,
            description='Speed held when the chosen gap is off to one side '
                        'rather than straight ahead.'),
    Tunable('max_acceleration', 'Speed', 0.5, 6.0, unit='m/s^2', step=0.25,
            description='How fast a speed command may rise. Lower is '
                        'smoother out of corners; higher recovers pace '
                        'sooner after a slowdown.'),
    Tunable('max_lateral_accel', 'Speed', 0.3, 4.0, unit='m/s^2', step=0.1,
            description='Cornering aggression: speed is held to '
                        'sqrt(a_lat / curvature) for the steering angle '
                        'being commanded.'),

    # --- Which gap gets chosen, and how hard the car points at it ---
    Tunable('min_gap_distance', 'Gap selection', 0.5, 5.0, unit='m', step=0.1,
            description='How deep an opening must be before it counts as a '
                        'gap worth steering into. Raise it in a cluttered '
                        'room so the car stops taking doorways.'),
    Tunable('fallback_min_gap_distance', 'Gap selection', 0.2, 3.0, unit='m',
            step=0.05,
            description='The shallower gap accepted at a blind or tight '
                        'corner when nothing meets min_gap_distance, so the '
                        'car crawls through instead of deadlocking.'),
    Tunable('disparity_threshold', 'Gap selection', 0.1, 1.5, unit='m', step=0.05,
            description='Range jump between adjacent beams that counts as an '
                        'obstacle edge to extend around.'),
    Tunable('steering_gain', 'Gap selection', 0.2, 2.5, step=0.05,
            description='Steering per unit of gap bearing; 1.0 means "point '
                        'the wheels straight at the gap". Too high weaves.'),
    Tunable('max_steering_rate', 'Gap selection', 0.2, 4.0, unit='rad/s', step=0.1,
            description='Caps how fast the steering angle may slew between '
                        'commands.'),

    # --- Safety margins. Real knobs, marked so nobody drags one thinking
    # it is a lap-time setting. ---
    Tunable('max_braking_decel', 'Safety margins', 0.5, 5.0, unit='m/s^2',
            step=0.25, safety=True,
            description='SAFETY-CRITICAL, and NOT the same knob as '
                        "pure_pursuit's identically named one. Here it is "
                        'the braking authority the car *assumes it has* when '
                        'deciding how fast it may drive for the clearance it '
                        'can see. Set above what the car can really achieve, '
                        'it drives faster than it can stop.'),
    Tunable('safety_margin', 'Safety margins', 0.02, 0.4, unit='m', step=0.01,
            safety=True,
            description='Extra padding added around the car footprint when '
                        'inflating obstacles.'),
    Tunable('emergency_stop_clearance', 'Safety margins', 0.0, 0.3, unit='m',
            step=0.01, safety=True,
            description='All-round contact floor measured from the padded '
                        'body. The last line before touching something.'),
    Tunable('forward_stop_clearance', 'Safety margins', 0.05, 1.0, unit='m',
            step=0.05, safety=True,
            description='Odom-independent reserve straight ahead. Must stay '
                        'at or above emergency_stop_clearance.'),
    Tunable('forward_stop_fov_deg', 'Safety margins', 10.0, 180.0, unit='deg',
            step=5.0, safety=True, attr='forward_stop_fov',
            transform=math.radians,
            description='Width of the cone that reserve watches. Narrow on '
                        'purpose: a wide cone treats a close side wall in a '
                        'corner as a frontal hazard.'),
    Tunable('escape_creep_speed', 'Safety margins', 0.05, 1.0, unit='m/s',
            step=0.05, safety=True,
            description='The one speed allowed inside the forward reserve, '
                        'so a car that has found an exit can inch toward it '
                        'instead of latching at zero. A crawl, not a drive '
                        'speed -- capped at max(min_speed, 0.5).'),
    Tunable('enable_ttc', 'Safety margins', 0.0, 1.0, kind='bool', safety=True,
            description='Time-to-collision automatic emergency braking. The '
                        'clearance-based stops stay active either way, but '
                        'turning this off removes the closing-speed brake.'),
    Tunable('ttc_threshold_sec', 'Safety margins', 0.1, 2.0, unit='s',
            step=0.05, safety=True,
            description='Time-to-impact at which that brake fires. Higher '
                        'brakes earlier and more often.'),
    Tunable('ttc_min_brake_speed', 'Safety margins', 0.0, 2.0, unit='m/s',
            step=0.05, safety=True,
            description='Speed below which the TTC brake is not armed. At a '
                        'crawl the clock measures almost no clearance, which '
                        'traps the car against a corner it has already eased '
                        'up to. The clearance stops and the forward-reserve '
                        'creep still run below this speed.'),
)


def _speed_order(merged):
    if merged['min_speed'] > merged['max_speed']:
        return (f"min_speed ({merged['min_speed']:g}) cannot exceed max_speed "
                f"({merged['max_speed']:g})")
    return None


def _forward_reserve_order(merged):
    # Mirrors the startup check: the forward reserve sits outside the
    # all-round contact floor, never inside it.
    if merged['forward_stop_clearance'] < merged['emergency_stop_clearance']:
        return (f"forward_stop_clearance ({merged['forward_stop_clearance']:g}) "
                f"cannot be smaller than emergency_stop_clearance "
                f"({merged['emergency_stop_clearance']:g})")
    return None


def _creep_stays_a_crawl(merged):
    # Mirrors the startup check. The creep is the only speed permitted
    # inside the forward reserve, so it must not quietly become the
    # ordinary driving speed by way of a slider.
    ceiling = max(merged['min_speed'], 0.5)
    if not 0.0 < merged['escape_creep_speed'] <= ceiling:
        return (f"escape_creep_speed ({merged['escape_creep_speed']:g}) must be "
                f"positive and no greater than max(min_speed, 0.5) = {ceiling:g} "
                f"-- it is a crawl, not a drive speed")
    return None


def _stop_cone_fits_scan_window(merged):
    # Mirrors the startup check, in degrees rather than radians because
    # that is the unit the parameter is in. forward_fov_deg is not itself
    # tunable, so this only fires on a forward_stop_fov_deg raise.
    if merged['forward_stop_fov_deg'] > merged['forward_fov_deg']:
        return (f"forward_stop_fov_deg ({merged['forward_stop_fov_deg']:g}) cannot "
                f"be wider than forward_fov_deg ({merged['forward_fov_deg']:g})")
    return None


GAP_FOLLOW_INVARIANTS = (
    _speed_order,
    _forward_reserve_order,
    _creep_stays_a_crawl,
    _stop_cone_fits_scan_window,
)

# Values review() needs in `current` to check the invariants above, beyond
# the tunables themselves.
GAP_FOLLOW_INVARIANT_CONTEXT = ('forward_fov_deg',)
