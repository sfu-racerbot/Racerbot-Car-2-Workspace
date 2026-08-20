"""Coupled speed overrides for launch files that cap gap_follow's speed.

Importable and unit-testable without rclpy, the same split gap_logic.py
uses (see docs/writing-your-own-node.md).

**No cap is the default.** The mapping launch files used to force
`max_speed: 1.0` on every run, and the 2026-08-19 run shows what that
cost: 154 of 191 logged driving ticks were commanded at exactly
1.00m/s, i.e. the car spent 81% of the mapping run pinned to that
override rather than to any sensed limit -- the curvature and clearance
caps never once bound below it. Passing no speed argument now leaves
gap_follow.yaml in charge, so the car maps at the speed it is actually
tuned for and the sensed caps do the limiting. A cap is still one
argument away (`mapping_max_speed:=1.5`) for a first look at a course
nobody trusts yet.

`max_speed` is not an independent knob. Two other parameters are defined
relative to it and the node checks the relation at startup
(`gap_follow_node._validate_adaptive_width_parameters`):

    corner_speed <= corner_speed_wide <= max_speed

so a launch file that lowers *only* max_speed -- which every mapping
launch here does, deliberately, to take a first cautious look at an
unfamiliar track -- pushes the packaged corner caps above the new top
speed and the node refuses to start. That is exactly what happened on
2026-08-19: auto_map_race_launch.py's `mapping_max_speed:=1.0` against
gap_follow.yaml's `corner_speed_wide: 1.4` killed gap_follow_node at
startup with exit code 1, so nothing ever published /auto_map/drive, the
supervisor never recorded a lap, and pure_pursuit_node sat in
`waiting_for_profile` forever with the car motionless.

Scaling rather than clamping is what the config itself asks for:
corner_speed's comment records that it is "raised with max_speed so
corners are quicker, not just straights". Clamping the caps to the new
max_speed would technically satisfy the relation but would make corners
and straights the same speed -- the opposite of what a cautious mapping
lap wants. Holding the tuned ratio keeps corners proportionally slower
than straights at whatever speed the course is being mapped at.
"""

import yaml

#: Parameters defined relative to `max_speed`, scaled with it.
COUPLED_TO_MAX_SPEED = ('corner_speed', 'corner_speed_wide')


def load_defaults(config_path, node_name='gap_follow_node'):
    """The `ros__parameters` block of a gap_follow parameter file."""
    with open(config_path) as handle:
        loaded = yaml.safe_load(handle) or {}
    return loaded.get(node_name, {}).get('ros__parameters', {})


def _requested(value):
    """A speed argument as a float, or None for "no override asked for".

    Launch arguments arrive as strings and default to empty, which is how
    a caller says "leave gap_follow.yaml alone". Whitespace and the words
    used for the same intent are accepted so a sweep script does not have
    to care which spelling it emits.
    """
    if value is None:
        return None
    text = str(value).strip()
    if text == '' or text.lower() in ('none', 'config', 'default'):
        return None
    return float(text)


def mapping_speed_overrides(config_path, max_speed, min_speed,
                            node_name='gap_follow_node'):
    """Parameter overrides that cap gap_follow's speeds self-consistently.

    Returns an **empty dict** when no `max_speed` is asked for: the car
    then maps at gap_follow.yaml's own tuned speeds, which is the default
    and the fast path. A `min_speed` given without a `max_speed` is still
    honoured on its own -- it changes no coupled parameter.

    With a `max_speed`, returns it and `min_speed` plus every parameter
    coupled to max_speed, scaled by the same factor, so the result always
    satisfies the relation the node validates at startup. A config file
    that omits a coupled parameter simply leaves it out of the result --
    the node's own declared default then applies, exactly as it does
    without any override at all.

    `min_speed` is clamped to `max_speed`: `min_speed > max_speed` fails a
    separate startup check, and a dead mapping controller is a worse
    answer to "you asked to map slower than your floor" than a floor
    quietly lowered to match.

    The scaled caps are in turn floored at `min_speed`. The corner cap is
    applied *after* the min_speed floor in the control path
    (`gap_follow_node.scan_callback`), so a cap scaled below the floor would
    quietly drive the car slower than the floor it was just told to keep
    -- and, once two caps land there, out of their own required ordering.
    """
    max_speed = _requested(max_speed)
    min_speed = _requested(min_speed)
    if max_speed is None:
        # Nothing is capped, so nothing is coupled and nothing needs
        # scaling. A bare min_speed still passes through.
        return {} if min_speed is None else {'min_speed': min_speed}

    defaults = load_defaults(config_path, node_name)
    if min_speed is None:
        # No floor asked for, so the config's own floor stands -- but it was
        # chosen against the config's top speed, and a floor above the new
        # cap fails a startup check of its own.
        min_speed = float(defaults.get('min_speed', max_speed))
    min_speed = min(min_speed, max_speed)
    overrides = {'max_speed': max_speed, 'min_speed': min_speed}

    default_max = float(defaults.get('max_speed', 0.0) or 0.0)
    if default_max <= 0.0:
        # Nothing to scale against; the node's declared defaults for the
        # coupled parameters are all that is left, and they are what
        # `max_speed`'s own declared default was tuned with.
        return overrides

    # COUPLED_TO_MAX_SPEED is in ascending order, so carrying the previous
    # result forward as the next one's floor is what keeps the ordering the
    # node checks intact no matter where the clamps land.
    factor = max_speed / default_max
    floor = min_speed
    for name in COUPLED_TO_MAX_SPEED:
        if name not in defaults:
            continue
        scaled = min(max(float(defaults[name]) * factor, floor), max_speed)
        overrides[name] = floor = round(scaled, 4)
    return overrides
