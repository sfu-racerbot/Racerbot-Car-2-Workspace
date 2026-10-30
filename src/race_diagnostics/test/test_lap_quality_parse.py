"""Supervisor quality-message contract; runner: pytest race_diagnostics/test/."""
import pytest

from race_diagnostics.run_events import parse_lap_quality


CEILING = 'Lap 5 did not pass the quality check (revolutions=2.04 > 1.50 (closed over roughly two overlapping laps, not one)), but the 5-lap ceiling is reached -- accepting it anyway rather than mapping indefinitely.'
MULTI = 'Lap 4 did not pass the quality check (heading_error=19.2deg > 15.0deg (50% of the 30deg gate); revolutions=2.09 > 1.50 (...)) and the 5-lap ceiling has not been reached -- ...'
SINGLE = 'Lap 3 did not pass the quality check (heading_error=29.5deg > 15.0deg (50% of the 30deg gate)) and the 5-lap ceiling has not been reached -- recording one more lap instead of accepting it. Keep holding LB.'


def test_revolutions_failure_accepted_at_ceiling():
    # oracle: spec, the verbatim supervisor message says failed and accepted.
    assert parse_lap_quality(CEILING) == dict(
        lap=5, passed=False, metric='revolutions', value=2.04, limit=1.50,
        accepted_at_ceiling=True)


def test_every_metric_is_preserved_in_order():
    # oracle: spec, both comparisons in the verbatim message must survive.
    assert parse_lap_quality(MULTI) == dict(
        lap=4, passed=False, metric='heading_error', value=19.2, limit=15.0,
        accepted_at_ceiling=False, metrics=[
            dict(metric='heading_error', value=19.2, limit=15.0),
            dict(metric='revolutions', value=2.09, limit=1.50)])


def test_single_degree_metric_keeps_existing_contract():
    # oracle: spec, backwards compatibility with test_lap_closed_lines.py.
    assert parse_lap_quality(SINGLE) == dict(
        lap=3, passed=False, metric='heading_error', value=29.5, limit=15.0,
        accepted_at_ceiling=False)


@pytest.mark.parametrize('line', [
    CEILING[:80], SINGLE.split('))')[0], 'garbage', '', None, 17, [], {},
    CEILING.encode(), CEILING.replace('2.04', 'NaN'),
    MULTI.replace('2.09', 'garbage'),
])
def test_incomplete_or_invalid_quality_message_is_not_a_verdict(line):
    # oracle: invariant, an incomplete/malformed message cannot be a verdict.
    assert parse_lap_quality(line) is None


def test_unlisted_metric_without_degree_unit():
    # oracle: spec, metric names are not restricted to today's known gates.
    line = SINGLE.replace('heading_error=29.5deg > 15.0deg', 'new_gate=21 > 20')
    assert parse_lap_quality(line) == dict(
        lap=3, passed=False, metric='new_gate', value=21.0, limit=20.0,
        accepted_at_ceiling=False)


def _producer_failure_clauses():
    """Execute the real formatter without importing ROS or constructing a node."""
    import ast
    import math
    from pathlib import Path
    from types import SimpleNamespace

    source = (Path(__file__).resolve().parents[2]
              / 'pure_pursuit/pure_pursuit/auto_map_race_node.py').read_text()
    node_class, = [node for node in ast.parse(source).body
                   if isinstance(node, ast.ClassDef) and node.name == 'AutoMapRaceNode']
    method, = [node for node in node_class.body
               if isinstance(node, ast.FunctionDef) and node.name == '_lap_quality_ok']
    namespace = {'math': math}
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<quality producer>', 'exec'),
         namespace)
    subject = SimpleNamespace(
        recorder=SimpleNamespace(reanchor_count=21, closest_approach=1.8,
                                 closure_distance=1.5, heading_error=math.radians(19.2),
                                 closure_heading_rad=math.radians(30), revolutions=2.09),
        mapping_lap_max_reanchors=20, mapping_lap_require_unwidened_closure=True,
        mapping_lap_max_heading_margin_fraction=0.5, mapping_lap_max_revolutions=1.5)
    passed, clauses = namespace['_lap_quality_ok'](subject)
    # oracle: spec, these inputs violate every quality gate in the producer.
    assert passed is False
    formats = [call for call in ast.walk(method)
               if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
               and call.func.attr == 'append' and isinstance(call.func.value, ast.Name)
               and call.func.value.id == 'failed']
    # A newly added gate must be exercised even if these inputs do not yet
    # trigger it. Count every failure format in the source, not only output.
    assert len(clauses) == len(formats)
    return clauses


@pytest.mark.parametrize('accepted', [False, True])
@pytest.mark.parametrize('selection', range(1, 16))
def test_every_producer_failure_clause_singly_and_combined(selection, accepted):
    clauses = _producer_failure_clauses()
    expected = [dict(metric='reanchor_count', value=21.0, limit=20.0),
                dict(metric='closest_approach', value=1.8, limit=1.5),
                dict(metric='heading_error', value=19.2, limit=15.0),
                dict(metric='revolutions', value=2.09, limit=1.5)]
    # oracle: spec, the producer's complete failure vocabulary. A new clause
    # must extend this contract instead of becoming an unparsed false pass.
    assert len(clauses) == len(expected)
    chosen = [i for i in range(len(clauses)) if selection & (1 << i)]
    tail = (', but the 5-lap ceiling is reached -- accepting it anyway rather '
            'than mapping indefinitely.' if accepted else
            ' and the 5-lap ceiling has not been reached -- recording one more lap.')
    line = 'Lap 5 did not pass the quality check (' + '; '.join(
        clauses[i] for i in chosen) + ')' + tail
    parsed = parse_lap_quality(line)
    wanted = dict(lap=5, passed=False, **expected[chosen[0]], accepted_at_ceiling=accepted)
    if len(chosen) > 1:
        wanted['metrics'] = [expected[i] for i in chosen]
    assert parsed == wanted
