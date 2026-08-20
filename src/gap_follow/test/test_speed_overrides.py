"""The coupled-speed override math used by every mapping launch file.

The bug these cover: a launch file lowering only `max_speed` leaves
`corner_speed`/`corner_speed_wide` above it, which
`gap_follow_node._validate_adaptive_width_parameters` rejects at startup
-- the node exits, nothing drives, and (under auto_map_race_launch.py)
pure_pursuit waits forever for a racing line that never gets recorded.
"""

import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from gap_follow.speed_overrides import (COUPLED_TO_MAX_SPEED,  # noqa: E402
                                        load_defaults,
                                        mapping_speed_overrides)

PACKAGED_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'gap_follow.yaml')


def write_config(tmp_path, params):
    path = tmp_path / 'gap_follow.yaml'
    path.write_text(yaml.safe_dump(
        {'gap_follow_node': {'ros__parameters': params}}))
    return str(path)


def assert_startup_relation_holds(config_path, overrides):
    """The exact relation the node checks before it agrees to run."""
    merged = dict(load_defaults(config_path))
    merged.update(overrides)
    assert 0.0 <= merged['min_speed'] <= merged['max_speed']
    assert merged['corner_speed'] <= merged['corner_speed_wide'] <= merged['max_speed']


def test_packaged_config_alone_is_self_consistent():
    # No overrides at all: the packaged defaults must already satisfy the
    # relation, or plain gap_follow_launch.py is broken too.
    assert_startup_relation_holds(PACKAGED_CONFIG, {})


@pytest.mark.parametrize('max_speed', [0.4, 1.0, 1.5, 2.5, 3.5])
def test_every_mapping_speed_yields_a_startable_set(max_speed):
    overrides = mapping_speed_overrides(PACKAGED_CONFIG, max_speed, 0.4)
    assert_startup_relation_holds(PACKAGED_CONFIG, overrides)


def test_the_exact_arguments_that_killed_the_node():
    # What auto_map_race_launch.py used to force on every run, which
    # before this helper produced corner_speed 1.1 / corner_speed_wide 1.4
    # against a max_speed of 1.0 and took gap_follow_node down on startup.
    overrides = mapping_speed_overrides(PACKAGED_CONFIG, 1.0, 0.4)
    assert overrides['max_speed'] == 1.0
    assert overrides['min_speed'] == 0.4
    assert_startup_relation_holds(PACKAGED_CONFIG, overrides)


def test_no_cap_is_the_default_and_overrides_nothing():
    """The launch files now pass empty strings unless a cap is asked for.

    The 2026-08-19 run is why: a forced 1.0m/s cap was the binding limit
    on 154 of 191 driving ticks, with no sensed cap ever below it. An
    empty result means gap_follow.yaml governs, which is also the only
    result that cannot violate the coupled-speed relation.
    """
    assert mapping_speed_overrides(PACKAGED_CONFIG, '', '') == {}
    assert mapping_speed_overrides(PACKAGED_CONFIG, None, None) == {}
    assert mapping_speed_overrides(PACKAGED_CONFIG, '  ', '  ') == {}


def test_a_floor_without_a_cap_passes_through_alone():
    # Nothing is coupled to min_speed, so it needs no scaling and must not
    # drag an unrequested max_speed cap in with it.
    assert mapping_speed_overrides(PACKAGED_CONFIG, '', 0.6) == {'min_speed': 0.6}


def test_a_cap_without_a_floor_uses_the_configured_floor(tmp_path):
    config = write_config(tmp_path, {
        'max_speed': 2.5, 'min_speed': 0.8,
        'corner_speed': 1.1, 'corner_speed_wide': 1.4,
    })
    overrides = mapping_speed_overrides(config, 2.0, '')
    assert overrides['min_speed'] == 0.8
    assert_startup_relation_holds(config, overrides)


def test_a_cap_below_the_configured_floor_still_starts(tmp_path):
    # The config floor was chosen against the config's top speed; a cap
    # under it would otherwise fail the min_speed <= max_speed check.
    config = write_config(tmp_path, {
        'max_speed': 2.5, 'min_speed': 0.8,
        'corner_speed': 1.1, 'corner_speed_wide': 1.4,
    })
    overrides = mapping_speed_overrides(config, 0.5, '')
    assert overrides['min_speed'] == 0.5
    assert_startup_relation_holds(config, overrides)


def test_coupled_caps_scale_with_max_speed_not_just_clamp(tmp_path):
    config = write_config(tmp_path, {
        'max_speed': 2.5, 'min_speed': 0.8,
        'corner_speed': 1.0, 'corner_speed_wide': 1.5,
    })
    overrides = mapping_speed_overrides(config, 1.25, 0.4)
    # Half the top speed means half the corner caps: the tuned ratio
    # survives, so corners stay slower than straights while mapping.
    assert overrides['corner_speed'] == pytest.approx(0.5)
    assert overrides['corner_speed_wide'] == pytest.approx(0.75)


def test_raising_max_speed_raises_the_caps_with_it(tmp_path):
    config = write_config(tmp_path, {
        'max_speed': 2.0, 'min_speed': 0.5,
        'corner_speed': 1.0, 'corner_speed_wide': 1.4,
    })
    overrides = mapping_speed_overrides(config, 4.0, 0.5)
    assert overrides['corner_speed'] == pytest.approx(2.0)
    assert overrides['corner_speed_wide'] == pytest.approx(2.8)


def test_caps_never_scale_past_the_new_max_speed(tmp_path):
    # A config whose wide cap already sits at max_speed must not scale to
    # something above it through float error or a >1 factor.
    config = write_config(tmp_path, {
        'max_speed': 2.0, 'min_speed': 0.5,
        'corner_speed': 2.0, 'corner_speed_wide': 2.0,
    })
    for target in (0.5, 2.0, 6.0):
        overrides = mapping_speed_overrides(config, target, 0.4)
        assert overrides['corner_speed_wide'] <= overrides['max_speed']
        assert_startup_relation_holds(config, overrides)


def test_min_speed_is_clamped_rather_than_left_above_max(tmp_path):
    config = write_config(tmp_path, {
        'max_speed': 2.5, 'min_speed': 0.8,
        'corner_speed': 1.1, 'corner_speed_wide': 1.4,
    })
    # Mapping a very tight course slower than the packaged floor must
    # still produce a node that starts.
    overrides = mapping_speed_overrides(config, 0.3, 0.4)
    assert overrides['min_speed'] == 0.3
    assert_startup_relation_holds(config, overrides)


def test_string_arguments_from_launch_configurations_are_accepted():
    # LaunchConfiguration.perform() hands back strings, always.
    overrides = mapping_speed_overrides(PACKAGED_CONFIG, '1.0', '0.4')
    assert overrides['max_speed'] == 1.0
    assert_startup_relation_holds(PACKAGED_CONFIG, overrides)


def test_absent_coupled_parameter_is_left_to_the_node_default(tmp_path):
    config = write_config(tmp_path, {'max_speed': 2.5, 'min_speed': 0.8})
    overrides = mapping_speed_overrides(config, 1.0, 0.4)
    assert set(overrides) == {'max_speed', 'min_speed'}


def test_config_without_a_max_speed_still_overrides_the_asked_for_speeds(tmp_path):
    config = write_config(tmp_path, {'corner_speed': 1.1})
    overrides = mapping_speed_overrides(config, 1.0, 0.4)
    assert overrides == {'max_speed': 1.0, 'min_speed': 0.4}


def test_every_coupled_name_exists_in_the_packaged_config():
    # A rename in gap_follow.yaml would otherwise silently stop scaling
    # the very parameter that made the node refuse to start.
    defaults = load_defaults(PACKAGED_CONFIG)
    for name in COUPLED_TO_MAX_SPEED:
        assert name in defaults, f'{name} no longer in gap_follow.yaml'


def test_scaled_caps_are_floored_at_min_speed(tmp_path):
    # The corner cap is applied after the min_speed floor in the control
    # path, so a cap scaled below that floor would silently undercut it.
    config = write_config(tmp_path, {
        'max_speed': 2.5, 'min_speed': 0.8,
        'corner_speed': 1.1, 'corner_speed_wide': 1.4,
    })
    overrides = mapping_speed_overrides(config, 0.6, 0.5)
    assert overrides['corner_speed'] >= overrides['min_speed']
    assert overrides['corner_speed_wide'] >= overrides['corner_speed']
    assert_startup_relation_holds(config, overrides)


def test_ordering_survives_both_caps_landing_on_the_floor(tmp_path):
    config = write_config(tmp_path, {
        'max_speed': 2.5, 'min_speed': 0.8,
        'corner_speed': 1.1, 'corner_speed_wide': 1.4,
    })
    overrides = mapping_speed_overrides(config, 0.5, 0.5)
    assert overrides['corner_speed'] == pytest.approx(0.5)
    assert overrides['corner_speed_wide'] == pytest.approx(0.5)
    assert_startup_relation_holds(config, overrides)
