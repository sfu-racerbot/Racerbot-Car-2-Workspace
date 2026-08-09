"""
Unit tests for gap_follow.live_tuning -- the validation that stands
between a browser slider and a moving car. No ROS needed. Run with:

    python3 -m pytest src/gap_follow/test/test_gap_follow_live_tuning.py -v

Three kinds of test live here:

  * the generic review() machinery (accept, clamp, refuse, all-or-nothing);
  * guards on the *catalogue* -- that the parameters this node exposes are
    the ones intended, within bounds that contain the shipped config, and
    that the safety-critical ones nobody should be able to reach from a
    browser are still absent;
  * a structural guard that every Tunable's `attr` names an attribute the
    node genuinely assigns. That last one matters more than it looks: a
    typo there makes setattr() invent a brand-new attribute, the set
    reports success, the dashboard shows the new value, and the control
    loop keeps using the old one forever.
"""
import os
import re
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from gap_follow import live_tuning  # noqa: E402

PACKAGE_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
NODE_SOURCE = os.path.join(PACKAGE_DIR, 'gap_follow', 'gap_follow_node.py')
CONFIG = os.path.join(PACKAGE_DIR, 'config', 'gap_follow.yaml')

TUNABLES = live_tuning.by_name(live_tuning.GAP_FOLLOW_TUNABLES)


def config_values():
    with open(CONFIG) as handle:
        return yaml.safe_load(handle)['gap_follow_node']['ros__parameters']


def current_values():
    """The shipped config, plus the context values the invariants read."""
    config = config_values()
    names = tuple(TUNABLES) + live_tuning.GAP_FOLLOW_INVARIANT_CONTEXT
    return {name: config[name] for name in names}


def review(requested, current=None):
    return live_tuning.review(
        TUNABLES, requested, current or current_values(),
        passthrough=('use_sim_time',),
        invariants=live_tuning.GAP_FOLLOW_INVARIANTS)


# ---------------------------------------------------------------------------
# review(): the accept path
# ---------------------------------------------------------------------------

def test_a_value_inside_its_bounds_is_accepted():
    accepted, error = review({'max_speed': 2.0})
    assert error is None
    assert accepted == {'max_speed': 2.0}


def test_bounds_are_inclusive():
    # max_lateral_accel deliberately, not max_speed: max_speed's own floor
    # sits below the shipped min_speed, so pinning it there trips the
    # min<=max invariant rather than the bounds check under test.
    tunable = TUNABLES['max_lateral_accel']
    for value in (tunable.minimum, tunable.maximum):
        accepted, error = review({'max_lateral_accel': value})
        assert error is None, f'{value} should be allowed'
        assert accepted['max_lateral_accel'] == value


def test_integers_are_accepted_as_floats():
    """A slider landing exactly on 2 must not fail where 2.1 works."""
    accepted, error = review({'max_speed': 2})
    assert error is None and accepted['max_speed'] == 2.0


def test_passthrough_names_are_ignored_not_refused():
    accepted, error = review({'use_sim_time': True})
    assert error is None and accepted == {}


def test_a_bool_tunable_round_trips():
    accepted, error = review({'enable_ttc': False})
    assert error is None and accepted == {'enable_ttc': False}


# ---------------------------------------------------------------------------
# review(): the refuse path
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('value', [99.0, -1.0])
def test_out_of_range_is_refused(value):
    accepted, error = review({'max_speed': value})
    assert accepted == {}
    assert 'between' in error


def test_an_unknown_parameter_is_refused_rather_than_ignored():
    """The whole reason this module exists -- a silently accepted change
    the control loop never reads is worse than a loud refusal."""
    accepted, error = review({'car_width': 0.9})
    assert accepted == {}
    assert 'cannot be changed while the node is running' in error


def test_the_deadman_cannot_be_switched_off_at_runtime():
    accepted, error = review({'enable_deadman': False})
    assert accepted == {}
    assert error


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_non_finite_is_refused(value):
    accepted, error = review({'max_speed': value})
    assert accepted == {} and error


def test_a_bool_for_a_float_is_refused():
    accepted, error = review({'max_speed': True})
    assert accepted == {} and 'number' in error


def test_a_number_for_a_bool_is_refused():
    accepted, error = review({'enable_ttc': 1})
    assert accepted == {} and 'true/false' in error


def test_a_string_is_refused():
    accepted, error = review({'max_speed': 'fast'})
    assert accepted == {} and error


# ---------------------------------------------------------------------------
# review(): all-or-nothing, and the cross-parameter invariants
# ---------------------------------------------------------------------------

def test_one_bad_value_rejects_the_whole_batch():
    """Half a speed change landing is its own hazard."""
    accepted, error = review({'max_speed': 2.0, 'min_speed': 99.0})
    assert accepted == {}
    assert error


def test_min_speed_cannot_exceed_max_speed():
    accepted, error = review({'min_speed': 2.0}, dict(current_values(), max_speed=1.0))
    assert accepted == {}
    assert 'cannot exceed max_speed' in error


def test_a_batch_that_satisfies_the_invariant_together_is_accepted():
    """min_speed alone would be illegal; raising max_speed with it is not,
    and the batch must be judged on its combined result."""
    accepted, error = review({'min_speed': 2.5, 'max_speed': 3.0})
    assert error is None
    assert accepted == {'min_speed': 2.5, 'max_speed': 3.0}


def test_the_forward_reserve_cannot_sit_inside_the_contact_floor():
    accepted, error = review({'forward_stop_clearance': 0.05},
                             dict(current_values(), emergency_stop_clearance=0.2))
    assert accepted == {}
    assert 'emergency_stop_clearance' in error


def test_the_escape_creep_cannot_become_a_drive_speed():
    """The creep is the one speed permitted inside the forward reserve, so
    a slider must not be able to quietly promote it to normal pace."""
    accepted, error = review({'escape_creep_speed': 1.0},
                             dict(current_values(), min_speed=0.5))
    assert accepted == {}
    assert 'crawl, not a drive speed' in error


def test_the_stop_cone_cannot_be_wider_than_the_scan_window():
    accepted, error = review({'forward_stop_fov_deg': 170.0},
                             dict(current_values(), forward_fov_deg=90.0))
    assert accepted == {}
    assert 'forward_fov_deg' in error


# ---------------------------------------------------------------------------
# Catalogue guards
# ---------------------------------------------------------------------------

def test_the_shipped_config_is_inside_every_bound():
    """A default outside its own tunable range would mean the panel opens
    showing a value it cannot reproduce."""
    config = config_values()
    for name, tunable in TUNABLES.items():
        assert name in config, f'{name} is tunable but missing from the config'
        if tunable.kind == 'bool':
            continue
        assert tunable.minimum <= config[name] <= tunable.maximum, (
            f'{name}: config default {config[name]} is outside '
            f'[{tunable.minimum}, {tunable.maximum}]')


def test_the_shipped_config_satisfies_its_own_invariants():
    accepted, error = review({})
    assert error is None


@pytest.mark.parametrize('forbidden', [
    'enable_deadman', 'joy_topic', 'deadman_button', 'joy_timeout_sec',
    'wheelbase', 'car_width', 'car_length', 'max_steering_angle',
    'drive_topic', 'scan_topic', 'forward_fov_deg',
    'laser_offset_x', 'laser_offset_y',
])
def test_parameters_that_must_never_be_browser_tunable(forbidden):
    assert forbidden not in TUNABLES


def test_every_tunable_is_declared_by_the_node():
    source = open(NODE_SOURCE).read()
    declared = set(re.findall(r"declare_parameter\(\s*'([a-z0-9_]+)'", source))
    missing = sorted(set(TUNABLES) - declared)
    assert not missing, f'tunable but never declared: {missing}'


def test_every_tunable_attr_is_actually_assigned_by_the_node():
    """Catches a typo in `attr`, which would otherwise make setattr()
    create a fresh unused attribute and report success while the control
    loop kept reading the original."""
    source = open(NODE_SOURCE).read()
    assigned = set(re.findall(r'self\.([a-z0-9_]+)\s*=', source))
    missing = sorted(t.target_attr for t in TUNABLES.values()
                     if t.target_attr not in assigned)
    assert not missing, f'tunable attrs the node never assigns: {missing}'


def test_every_tunable_is_documented_and_bounded():
    for name, tunable in TUNABLES.items():
        assert tunable.description, f'{name} has no description for the UI'
        assert tunable.group, f'{name} has no group'
        assert tunable.minimum <= tunable.maximum, name


def test_safety_flagged_parameters_are_the_expected_ones():
    """A change here should be deliberate: the flag drives the warning
    treatment that stops a collision margin looking like a lap-time knob."""
    flagged = {name for name, t in TUNABLES.items() if t.safety}
    assert flagged == {
        'max_braking_decel', 'safety_margin', 'emergency_stop_clearance',
        'forward_stop_clearance', 'forward_stop_fov_deg',
        'escape_creep_speed', 'enable_ttc', 'ttc_threshold_sec',
        'ttc_min_brake_speed',
    }


# ---------------------------------------------------------------------------
# The spec the dashboard consumes
# ---------------------------------------------------------------------------

def test_spec_json_describes_every_tunable():
    import json
    spec = json.loads(live_tuning.spec_json(
        'gap_follow_node', live_tuning.GAP_FOLLOW_TUNABLES))
    assert spec['version'] == live_tuning.SPEC_VERSION
    assert spec['node'] == 'gap_follow_node'
    assert [p['name'] for p in spec['params']] == list(TUNABLES)
    for entry in spec['params']:
        assert entry['min'] <= entry['max']
        assert entry['kind'] in ('float', 'bool')


def test_spec_is_parseable_by_the_dashboard():
    """The producer and the consumer live in different packages and can
    only agree through this string."""
    sys.path.insert(0, os.path.join(PACKAGE_DIR, '..', 'web_dashboard'))
    from web_dashboard import tuning as dashboard_tuning

    params, error = dashboard_tuning.parse_spec(live_tuning.spec_json(
        'gap_follow_node', live_tuning.GAP_FOLLOW_TUNABLES))
    assert error is None
    assert len(params) == len(TUNABLES)


# ---------------------------------------------------------------------------
# Anti-drift guard for the duplicated machinery
# ---------------------------------------------------------------------------

BANNER = ('\n\n# ====================================================='
          '=======================\n# PER-NODE CATALOGUE')


def test_the_generic_half_matches_pure_pursuits_copy():
    """This workspace duplicates rather than imports across packages (see
    CLAUDE.md), so the shared half has to be pinned by a test or it will
    quietly diverge -- and it is validation logic for a moving car."""
    counterpart = os.path.join(
        PACKAGE_DIR, '..', 'pure_pursuit', 'pure_pursuit', 'live_tuning.py')
    mine = open(live_tuning.__file__).read().split(BANNER)[0]
    theirs = open(counterpart).read().split(BANNER)[0]
    assert mine == theirs, (
        'pure_pursuit/live_tuning.py and gap_follow/live_tuning.py have '
        'drifted above the PER-NODE CATALOGUE banner')
