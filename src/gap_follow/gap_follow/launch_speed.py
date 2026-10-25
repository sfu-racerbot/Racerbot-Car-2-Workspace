"""The ``max_speed:=`` launch argument: a lower top speed for one run.

No ROS imports, so it is testable on its own (test/test_launch_speed.py).

The argument may only lower the speed. gap_follow.yaml's max_speed is the
ceiling; asking for more than that is refused rather than clamped, so a typo
never quietly runs the car at a speed nobody asked for. The node refuses to
start unless ``min_speed <= max_speed`` and
``corner_speed <= corner_speed_wide <= max_speed`` (gap_follow_node.py), so
the speeds above the new cap are lowered to it as well. Every other speed in
the YAML is left alone: each is either a ceiling below max_speed already or,
like narrow_corridor_speed, turns itself off at or above max_speed.
"""

import math

import yaml

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
    positive, finite speed at or below the YAML's max_speed.
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
    shipped = float(yaml_params['max_speed'])
    if cap > shipped:
        raise ValueError(
            f'max_speed:={cap} is above gap_follow.yaml\'s max_speed {shipped}: '
            'this argument only lowers the speed. Change the YAML to go faster.')
    overrides = {'max_speed': cap}
    for key in CAPPED_WITH_MAX:
        if float(yaml_params[key]) > cap:
            overrides[key] = cap
    return overrides
