"""
Unit tests for drive_intent.schema -- the /drive_intent wire format.

Run with:

    cd ~/racerbot-ws
    python3 -m pytest src/drive_intent/test/test_schema.py -v
"""
import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from drive_intent import schema  # noqa: E402


def sample_path(n=4, speed=2.0):
    return [(i * 0.5, 0.0, 0.0, speed) for i in range(n)]


# ============================================================================
# build + encode: what leaves the car
# ============================================================================

def test_a_built_payload_validates():
    payload = schema.build('gap_follow_node', 'gap_follow',
                           reason='clear corridor ahead',
                           path=sample_path(), commanded_path=sample_path(),
                           commanded_speed=2.0, horizon_s=1.5)
    assert schema.validate(payload) is None


def test_payload_carries_the_schema_version_so_consumers_can_refuse_it():
    payload = schema.build('gap_follow_node', 'gap_follow')
    assert payload['v'] == schema.SCHEMA_VERSION


def test_path_points_are_reduced_to_the_three_fields_the_browser_draws():
    payload = schema.build('n', 's', path=[(1.23456, -2.34567, 0.9, 3.14159)])
    assert payload['path'] == [{'x': 1.235, 'y': -2.346, 'v': 3.14}]


def test_a_three_tuple_path_treats_the_last_element_as_speed():
    payload = schema.build('n', 's', path=[(1.0, 2.0, 4.5)])
    assert payload['path'] == [{'x': 1.0, 'y': 2.0, 'v': 4.5}]


def test_reason_is_omitted_entirely_when_not_supplied():
    """The publisher attaches a reason only on state changes and on the
    slow period, so 'absent' has to be a legal, cheap state -- the browser
    keeps showing the last one it saw."""
    payload = schema.build('n', 'gap_follow')
    assert 'reason' not in payload
    assert schema.validate(payload) is None


def test_a_reason_thunk_is_resolved_at_build_time():
    payload = schema.build('n', 's', reason=lambda: 'computed lazily')
    assert payload['reason'] == 'computed lazily'


def test_an_enormous_reason_is_truncated_rather_than_broadcast():
    payload = schema.build('n', 's', reason='x' * 50000)
    assert len(payload['reason']) == schema.MAX_REASON_CHARS


def test_an_absurd_path_is_truncated_at_build_time():
    payload = schema.build('n', 's', path=sample_path(schema.MAX_PATH_POINTS + 50))
    assert len(payload['path']) == schema.MAX_PATH_POINTS


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
def test_a_non_finite_number_raises_instead_of_emitting_invalid_json(bad):
    """json.dumps would happily write a bare NaN, which JSON.parse rejects
    -- one numerical bug would break the whole socket's message stream,
    not just one arrow. It has to fail on the car instead."""
    with pytest.raises(ValueError):
        schema.build('n', 's', commanded_speed=bad)
    with pytest.raises(ValueError):
        schema.build('n', 's', path=[(0.0, bad, 1.0, 1.0)])


def test_encode_refuses_non_finite_values_even_if_one_slips_through():
    with pytest.raises(ValueError):
        schema.encode({'v': 1, 'commanded_speed': float('nan')})


def test_encode_decode_round_trips():
    payload = schema.build('pure_pursuit_node', 'overtake_left',
                           reason='passing on the left',
                           path=sample_path(), horizon_s=1.5,
                           targets=[schema.target('opponent', 2.0, 0.4)])
    assert schema.decode(schema.encode(payload)) == payload


def test_encoding_is_compact_enough_for_a_20hz_stream():
    payload = schema.build(
        'gap_follow_node', 'gap_follow', reason='x' * 200,
        path=sample_path(16), commanded_path=sample_path(16),
        factors=[schema.factor(f'cap {i}', i) for i in range(4)])
    assert len(schema.encode(payload)) < 2048


# ============================================================================
# factors: which constraint is actually in charge
# ============================================================================

def test_bind_min_marks_the_lowest_speed_ceiling():
    factors = schema.bind_min([
        schema.factor('curve cap', 2.4),
        schema.factor('clearance cap', 3.1),
        schema.factor('corner cap', 1.9),
    ])
    assert [f['binding'] for f in factors] == [False, False, True]
    assert schema.binding_factor({'factors': factors}) == 'corner cap'


def test_bind_min_marks_every_member_of_a_tie():
    """Naming one of two equal limits as 'the' reason would be a lie of
    precision -- both are holding the car back."""
    factors = schema.bind_min([
        schema.factor('a', 1.5), schema.factor('b', 1.5), schema.factor('c', 2.0)])
    assert [f['binding'] for f in factors] == [True, True, False]


def test_bind_min_does_not_mutate_its_input():
    original = [schema.factor('a', 1.0), schema.factor('b', 2.0)]
    schema.bind_min(original)
    assert all(f['binding'] is False for f in original)


def test_bind_min_of_nothing_is_nothing():
    assert schema.bind_min([]) == []


def test_binding_factor_is_none_when_no_constraint_is_active():
    assert schema.binding_factor({'factors': []}) is None
    assert schema.binding_factor({}) is None


# ============================================================================
# severity
# ============================================================================

def test_a_zero_speed_command_is_always_a_stop():
    assert schema.classify_severity('gap_follow', 0.0) == schema.SEVERITY_STOP
    assert schema.classify_severity('ttc_brake', 0.0) == schema.SEVERITY_STOP


def test_ordinary_driving_states_are_not_flagged():
    assert schema.classify_severity('gap_follow', 2.0) == schema.SEVERITY_DRIVE
    assert schema.classify_severity('pure_pursuit', 2.0) == schema.SEVERITY_DRIVE


def test_anything_out_of_the_ordinary_draws_the_eye():
    for state in ('corner_fallback', 'overtake_left', 'lidar_avoidance'):
        assert schema.classify_severity(state, 2.0) == schema.SEVERITY_CAUTION


def test_severity_is_derived_from_the_commanded_speed_not_the_desired_one():
    """A safety override zeroes the command while the plan still wants
    speed; the arrow must read as a stop."""
    payload = schema.build('n', 'gap_follow', desired_speed=3.0, commanded_speed=0.0)
    assert payload['severity'] == schema.SEVERITY_STOP


def test_an_explicit_severity_overrides_the_derived_one():
    payload = schema.build('n', 'gap_follow', commanded_speed=2.0,
                           severity=schema.SEVERITY_CAUTION)
    assert payload['severity'] == schema.SEVERITY_CAUTION


# ============================================================================
# memoize_reason
# ============================================================================

def test_an_expensive_reason_thunk_runs_at_most_once():
    calls = []

    def expensive():
        calls.append(1)
        return 'escape route: 12deg left'

    memo = schema.memoize_reason(expensive)
    assert schema.resolve_reason(memo) == 'escape route: 12deg left'
    assert schema.resolve_reason(memo) == 'escape route: 12deg left'
    assert len(calls) == 1


def test_memoize_passes_plain_strings_straight_through():
    assert schema.memoize_reason('already a string') == 'already a string'


def test_resolve_reason_of_none_is_none():
    assert schema.resolve_reason(None) is None


# ============================================================================
# validate: the consumer's guard against a misbehaving publisher
# ============================================================================

def test_a_future_schema_version_is_refused_rather_than_half_rendered():
    payload = schema.build('n', 's')
    payload['v'] = 99
    assert 'unsupported schema version' in schema.validate(payload)


def test_decode_rejects_garbage():
    with pytest.raises(ValueError):
        schema.decode('{not json')
    with pytest.raises(ValueError):
        schema.decode('[1, 2, 3]')       # valid JSON, wrong shape
    with pytest.raises(ValueError):
        schema.decode('"a bare string"')


def test_validate_rejects_a_non_object():
    assert schema.validate([1, 2]) is not None


@pytest.mark.parametrize('key', ['node', 'state', 'frame'])
def test_validate_requires_the_identifying_strings(key):
    payload = schema.build('n', 's')
    payload[key] = 17
    assert key in schema.validate(payload)


def test_validate_rejects_an_unknown_severity():
    payload = schema.build('n', 's')
    payload['severity'] = 'catastrophic'
    assert 'severity' in schema.validate(payload)


@pytest.mark.parametrize('key', ['stamp', 'horizon_s', 'desired_steering',
                                 'commanded_steering', 'desired_speed',
                                 'commanded_speed'])
def test_validate_requires_finite_numbers(key):
    payload = schema.build('n', 's')
    payload[key] = 'fast'
    assert key in schema.validate(payload)


def test_validate_rejects_a_nan_that_arrived_over_the_wire():
    """Python's json.loads accepts the non-standard bare NaN even though
    browsers don't, so a message from a C++ publisher using printf could
    carry one all the way here."""
    payload = json.loads(schema.encode(schema.build('n', 's')).replace(
        '"commanded_speed":0.0', '"commanded_speed":NaN'))
    assert 'commanded_speed' in schema.validate(payload)


def test_validate_rejects_a_malformed_path():
    payload = schema.build('n', 's')
    payload['path'] = [{'x': 1.0, 'y': 2.0}]     # no speed
    assert 'path' in schema.validate(payload)

    payload = schema.build('n', 's')
    payload['path'] = 'not a list'
    assert 'path' in schema.validate(payload)

    payload = schema.build('n', 's')
    payload['path'] = [{'x': 1.0, 'y': 2.0, 'v': True}]   # bool is not a number
    assert 'path' in schema.validate(payload)


def test_validate_rejects_an_oversized_path_from_the_wire():
    payload = schema.build('n', 's')
    payload['path'] = [{'x': 0.0, 'y': 0.0, 'v': 1.0}] * (schema.MAX_PATH_POINTS + 1)
    assert 'over the' in schema.validate(payload)


def test_validate_rejects_malformed_factors():
    payload = schema.build('n', 's')
    payload['factors'] = [{'name': 'cap', 'value': 'slow'}]
    assert 'factors' in schema.validate(payload)

    payload = schema.build('n', 's')
    payload['factors'] = [{'value': 1.0}]
    assert 'factors' in schema.validate(payload)


def test_validate_rejects_a_non_string_reason():
    payload = schema.build('n', 's')
    payload['reason'] = {'why': 'because'}
    assert 'reason' in schema.validate(payload)


# ============================================================================
# Optional geometry: targets and the gap wedge
# ============================================================================

def test_target_is_a_labelled_body_frame_point():
    assert schema.target('gap_target', 1.76123, -0.38456) == {
        'kind': 'gap_target', 'x': 1.761, 'y': -0.385}


def test_wedge_is_carried_through_and_validates():
    payload = schema.build('n', 's', wedge={
        'x': 0.33, 'y': 0.0, 'a0': -0.4, 'a1': 0.1, 'r': 3.0})
    assert payload['wedge']['r'] == pytest.approx(3.0)
    assert schema.validate(payload) is None


def test_no_wedge_means_no_key():
    assert 'wedge' not in schema.build('n', 's')


def test_frame_defaults_to_base_link_so_the_arrow_renders_without_a_pose():
    """Robot-centric mode has no map and no localization at all; publishing
    in the body frame is what lets the same arrow draw there and in
    map-relative mode."""
    assert schema.build('n', 's')['frame'] == schema.BODY_FRAME


def test_stamp_can_be_supplied_for_deterministic_tests():
    assert schema.build('n', 's', stamp=1234.5)['stamp'] == pytest.approx(1234.5)


def test_stamp_defaults_to_now():
    import time
    payload = schema.build('n', 's')
    assert abs(payload['stamp'] - time.time()) < 5.0
    assert math.isfinite(payload['stamp'])
