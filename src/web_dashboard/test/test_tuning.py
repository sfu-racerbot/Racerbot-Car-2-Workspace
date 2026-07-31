"""
Unit tests for web_dashboard.tuning -- spec parsing, request clamping, and
the comment-preserving YAML writer. No ROS, no Tornado, no network, no
browser. Run with:

    python3 -m pytest src/web_dashboard/test/test_tuning.py -v

The YAML tests lean on the real config files in this workspace rather than
only synthetic fixtures: the whole point of the line-surgery writer is
that it survives the *actual* files, which are dense with the comments
explaining why each number is what it is.
"""
import json
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from web_dashboard import tuning  # noqa: E402

WORKSPACE_SRC = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..'))

SAMPLE_YAML = """\
some_node:
  ros__parameters:
    # Why max_speed is what it is.
    max_speed: 4.0
    min_speed: 0.5     # trailing comment
    enable_thing: true

    # A trailing comment block.
other_node:
  ros__parameters:
    max_speed: 9.0
"""


def _spec(**overrides):
    param = {
        'name': 'max_speed', 'group': 'Speed', 'label': 'max speed',
        'kind': 'float', 'min': 0.5, 'max': 4.0, 'step': 0.1,
        'unit': 'm/s', 'safety': False, 'description': 'how fast',
    }
    param.update(overrides)
    return json.dumps({'version': 1, 'node': 'n', 'params': [param]})


# ---------------------------------------------------------------------------
# parse_spec
# ---------------------------------------------------------------------------

def test_parse_spec_reads_a_well_formed_catalogue():
    params, error = tuning.parse_spec(_spec())
    assert error is None
    assert len(params) == 1
    assert params[0]['name'] == 'max_speed'
    assert params[0]['min'] == 0.5 and params[0]['max'] == 4.0


def test_parse_spec_rejects_an_unsupported_version():
    text = json.dumps({'version': 99, 'params': []})
    params, error = tuning.parse_spec(text)
    assert params == []
    assert 'not supported' in error


@pytest.mark.parametrize('text', ['', 'not json at all', '[]', '{}'])
def test_parse_spec_survives_junk(text):
    """A node mid-restart, a different version, or the wrong node entirely
    must produce a message, never an exception."""
    params, error = tuning.parse_spec(text)
    assert params == []
    assert error


def test_parse_spec_drops_unusable_entries_but_keeps_the_rest():
    text = json.dumps({'version': 1, 'params': [
        {'name': 'good', 'group': 'g', 'kind': 'float', 'min': 0, 'max': 1},
        {'name': 'no_kind', 'group': 'g', 'min': 0, 'max': 1},
        {'name': 'bad_kind', 'group': 'g', 'kind': 'string', 'min': 0, 'max': 1},
        {'name': 'inverted', 'group': 'g', 'kind': 'float', 'min': 5, 'max': 1},
    ]})
    params, error = tuning.parse_spec(text)
    assert error is None
    assert [p['name'] for p in params] == ['good']


# ---------------------------------------------------------------------------
# coerce_request
# ---------------------------------------------------------------------------

def test_coerce_request_clamps_rather_than_refusing():
    param = tuning.parse_spec(_spec())[0][0]
    assert tuning.coerce_request(param, 99.0) == (4.0, None)
    assert tuning.coerce_request(param, -5.0) == (0.5, None)
    assert tuning.coerce_request(param, 2.0) == (2.0, None)


def test_coerce_request_refuses_wrong_types():
    param = tuning.parse_spec(_spec())[0][0]
    for bad in (True, 'fast', None, [1]):
        value, error = tuning.coerce_request(param, bad)
        assert value is None and error


def test_coerce_request_refuses_non_finite():
    param = tuning.parse_spec(_spec())[0][0]
    for bad in (float('inf'), float('nan')):
        value, error = tuning.coerce_request(param, bad)
        assert value is None and error


def test_coerce_request_bool_needs_a_real_bool():
    param = tuning.parse_spec(_spec(kind='bool'))[0][0]
    assert tuning.coerce_request(param, True) == (True, None)
    assert tuning.coerce_request(param, 1)[0] is None


# ---------------------------------------------------------------------------
# format_scalar
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('value,expected', [
    (4.0, '4.0'), (0.35, '0.35'), (0.05, '0.05'), (60.0, '60.0'),
    (0.0, '0.0'), (True, 'true'), (False, 'false'),
])
def test_format_scalar(value, expected):
    assert tuning.format_scalar(value) == expected


def test_format_scalar_keeps_floats_floating():
    """A whole-number double written as bare `4` reloads as an int and the
    node refuses to launch -- one run *after* the tune was saved."""
    assert yaml.safe_load(tuning.format_scalar(2.0)) == 2.0
    assert isinstance(yaml.safe_load(tuning.format_scalar(2.0)), float)


# ---------------------------------------------------------------------------
# update_yaml_values
# ---------------------------------------------------------------------------

def test_update_yaml_rewrites_only_the_requested_values():
    updated, changed, added = tuning.update_yaml_values(
        SAMPLE_YAML, 'some_node', {'max_speed': 2.5})
    assert changed == ['max_speed'] and added == []
    loaded = yaml.safe_load(updated)
    assert loaded['some_node']['ros__parameters']['max_speed'] == 2.5
    assert loaded['some_node']['ros__parameters']['min_speed'] == 0.5
    # The other node's identically named key is untouched.
    assert loaded['other_node']['ros__parameters']['max_speed'] == 9.0


def test_update_yaml_preserves_comments():
    updated, _, _ = tuning.update_yaml_values(
        SAMPLE_YAML, 'some_node', {'max_speed': 2.5, 'min_speed': 0.25})
    assert '# Why max_speed is what it is.' in updated
    assert 'min_speed: 0.25     # trailing comment' in updated


def test_update_yaml_appends_a_missing_key():
    updated, changed, added = tuning.update_yaml_values(
        SAMPLE_YAML, 'some_node', {'brand_new': 1.5})
    assert changed == [] and added == ['brand_new']
    assert yaml.safe_load(updated)['some_node']['ros__parameters']['brand_new'] == 1.5


def test_update_yaml_handles_bools():
    updated, _, _ = tuning.update_yaml_values(
        SAMPLE_YAML, 'some_node', {'enable_thing': False})
    assert yaml.safe_load(updated)['some_node']['ros__parameters']['enable_thing'] is False


def test_update_yaml_rejects_an_unknown_node():
    with pytest.raises(ValueError):
        tuning.update_yaml_values(SAMPLE_YAML, 'nope_node', {'max_speed': 1.0})


def test_update_yaml_rejects_a_file_without_ros_parameters():
    with pytest.raises(ValueError):
        tuning.update_yaml_values('some_node:\n  other: 1\n', 'some_node',
                                  {'max_speed': 1.0})


@pytest.mark.parametrize('package,node', [
    ('pure_pursuit', 'pure_pursuit_node'),
    ('gap_follow', 'gap_follow_node'),
])
def test_update_yaml_round_trips_the_real_configs(package, node):
    """The real files, which are mostly comments explaining the numbers.

    Asserts the two properties that make "save" safe to click: every
    comment survives, and no value other than the requested one moves.
    """
    path = os.path.join(WORKSPACE_SRC, package, 'config', f'{package}.yaml')
    original = open(path).read()
    updated, changed, added = tuning.update_yaml_values(
        original, node, {'max_speed': 1.25})
    assert changed == ['max_speed'] and added == []

    def comments(text):
        return [line.strip() for line in text.splitlines()
                if line.strip().startswith('#')]

    assert comments(updated) == comments(original)
    before = yaml.safe_load(original)[node]['ros__parameters']
    after = yaml.safe_load(updated)[node]['ros__parameters']
    assert set(before) == set(after)
    assert {k: v for k, v in after.items() if before[k] != v} == {'max_speed': 1.25}


# ---------------------------------------------------------------------------
# values_needing_save
# ---------------------------------------------------------------------------

def test_values_needing_save_only_reports_real_differences():
    pending = tuning.values_needing_save(
        SAMPLE_YAML, 'some_node',
        {'max_speed': 4.0, 'min_speed': 1.25, 'enable_thing': True})
    assert pending == {'min_speed': 1.25}


def test_values_needing_save_includes_keys_absent_from_the_file():
    pending = tuning.values_needing_save(
        SAMPLE_YAML, 'some_node', {'not_in_file': 0.5})
    assert pending == {'not_in_file': 0.5}


def test_values_needing_save_falls_back_to_everything_on_an_unreadable_file():
    """Better to write the whole tune than to silently save none of it."""
    pending = tuning.values_needing_save('', 'some_node', {'max_speed': 1.0})
    assert pending == {'max_speed': 1.0}


def test_values_needing_save_treats_bools_as_bools():
    pending = tuning.values_needing_save(
        SAMPLE_YAML, 'some_node', {'enable_thing': False})
    assert pending == {'enable_thing': False}
