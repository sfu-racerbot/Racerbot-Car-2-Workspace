"""gap_mirror.node_params(..., fill_legacy=True): an older gap_follow.yaml,
from before a parameter existed, must load with the value that reproduces
the behaviour from before it existed (gap_mirror.LEGACY_VALUES), so
tools/run_analysis can replay a run against the config it ran. Unsourced:

    python3 -m pytest tools/f1tenth_sim/tests/test_gap_mirror_legacy.py -v
"""
import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'src', 'gap_follow'))
import gap_mirror  # noqa: E402


def _old_yaml(tmp_path, keep=()):
    data = yaml.safe_load(gap_mirror.NODE_CONFIG.read_text())
    params = data['gap_follow_node']['ros__parameters']
    for name in gap_mirror.LEGACY_VALUES:
        if name not in keep:
            params.pop(name, None)
    path = tmp_path / 'old.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


def test_missing_parameters_take_their_pre_existence_values(tmp_path):
    p = gap_mirror.node_params(_old_yaml(tmp_path), fill_legacy=True)
    # Each value is the one that switches the feature off, per the YAML
    # comments in src/gap_follow/config/gap_follow.yaml.
    assert p['ttc_path_margin'] == p['safety_margin']      # the old corridor
    assert p['braking_latency_sec'] == 0.0
    assert p['steering_gain_speed_ref'] == 0.0
    assert p['speed_hold_sec'] == 0.0
    assert p['arc_guard_blocked_fallback'] == 'freest'
    assert p['narrow_corridor_speed'] >= p['max_speed']    # cap off


def test_a_parameter_the_file_has_is_never_overwritten(tmp_path):
    p = gap_mirror.node_params(_old_yaml(tmp_path, keep=('ttc_path_margin',)), fill_legacy=True)
    shipped = yaml.safe_load(gap_mirror.NODE_CONFIG.read_text())['gap_follow_node']['ros__parameters']
    assert p['ttc_path_margin'] == shipped['ttc_path_margin']
    assert p['ttc_path_margin'] != p['safety_margin'], 'precondition: distinguishable values'


def test_without_fill_an_old_file_is_not_silently_accepted(tmp_path):
    p = gap_mirror.node_params(_old_yaml(tmp_path))
    with pytest.raises(KeyError, match='ttc_path_margin'):
        p['ttc_path_margin']
