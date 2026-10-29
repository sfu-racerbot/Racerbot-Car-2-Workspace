"""The ``max_speed:=`` launch argument: a different top speed for one run.

No ROS imports, so it is testable on its own (test/test_launch_speed.py).

The argument may set the top speed anywhere up to MAX_SPEED_CEILING (5.0 m/s,
raised from "gap_follow.yaml's max_speed" on 2026-10-26 on request); asking
for more is refused rather than clamped, so a typo never quietly runs the car
at a speed nobody asked for. 5.0 m/s is 18250 erpm at vesc.yaml's 3650 gain,
inside its 23250 speed_max.

Lowering: the node refuses to start unless ``min_speed <= max_speed`` and
``corner_speed <= corner_speed_wide <= max_speed`` (gap_follow_node.py), so
the speeds above the new cap are lowered to it as well. Raising: only
max_speed changes. That already satisfies both checks, and the corner and
curvature caps still bind below it, so it is a faster straight, not a faster
car everywhere. Every other speed in the YAML is left alone: each is either a
ceiling below max_speed already or, like narrow_corridor_speed, turns itself
off at or above max_speed.
"""

import math

import yaml

# The most max_speed:= accepts (m/s). Not a tuned value: the requested limit.
MAX_SPEED_CEILING = 5.0

# Speeds the node requires to be no higher than max_speed.
CAPPED_WITH_MAX = ('min_speed', 'corner_speed', 'corner_speed_wide')


def yaml_parameters(config_path):
    """gap_follow_node's ros__parameters from a gap_follow.yaml."""
    with open(config_path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)['gap_follow_node']['ros__parameters']


def speed_cap_overrides(yaml_params, requested):
    """Parameter overrides for ``max_speed:=<requested>``.

    ``requested`` is the launch argument's text; empty means "use the YAML",
    and returns no overrides. Raises ValueError for anything that is not a
    positive, finite speed at or below MAX_SPEED_CEILING.
    """
    text = str(requested).strip()
    if text == '':
        return {}
    try:
        cap = float(text)
    except ValueError:
        raise ValueError(f'max_speed:={requested!r} is not a number') from None
    if not math.isfinite(cap) or cap <= 0.0:
        raise ValueError(f'max_speed:={requested!r} must be a positive, finite speed (m/s)')
    if cap > MAX_SPEED_CEILING:
        raise ValueError(
            f'max_speed:={cap} is above the {MAX_SPEED_CEILING} m/s ceiling '
            '(gap_follow/launch_speed.py MAX_SPEED_CEILING)')
    overrides = {'max_speed': cap}
    for key in CAPPED_WITH_MAX:
        if float(yaml_params[key]) > cap:
            overrides[key] = cap
    return overrides
