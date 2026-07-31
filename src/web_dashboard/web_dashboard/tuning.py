"""
tuning.py

The dashboard's half of live parameter tuning: reading the catalogue a
driving node advertises, vetting a browser's requested change against it,
and -- when asked -- writing a tune back into the package's YAML config so
it survives the next launch.

Like protocol.py, this file imports no ROS, no Tornado and no network
code, so it is directly unit-testable (test/test_tuning.py) without a
running robot, browser, or web server.

Trust boundary. Nothing in here is a safety mechanism, and it must not be
mistaken for one. The node that owns a parameter enforces its own bounds
in its own process on every update (pure_pursuit/live_tuning.py,
gap_follow/live_tuning.py); this module clamps too, but only so the UI
cannot *offer* a value that is going to bounce, and so an obviously junk
request never reaches the bus. If these two disagree, the node wins, and
the browser gets told why.
"""

import json
import re

# Spec versions this dashboard knows how to render. A node advertising
# anything else is shown as present-but-unreadable rather than silently
# half-rendered from fields that may have changed meaning.
SUPPORTED_SPEC_VERSIONS = (1,)

_REQUIRED_KEYS = ('name', 'group', 'kind', 'min', 'max')


def parse_spec(spec_text):
    """Parse a node's `live_tunable_spec` parameter.

    Returns (params, error): a list of normalized parameter dicts, or an
    empty list plus a human-readable reason. Defensive on purpose -- this
    string arrives from another process that may be a different version,
    mid-restart, or simply not the node we expected.
    """
    if not spec_text:
        return [], 'node advertises no live-tunable parameters'
    try:
        spec = json.loads(spec_text)
    except (TypeError, ValueError) as exc:
        return [], f'unreadable live_tunable_spec: {exc}'
    if not isinstance(spec, dict):
        return [], 'live_tunable_spec is not an object'
    version = spec.get('version')
    if version not in SUPPORTED_SPEC_VERSIONS:
        return [], (f'live_tunable_spec version {version!r} is not supported '
                    f'by this dashboard (expected one of '
                    f'{", ".join(str(v) for v in SUPPORTED_SPEC_VERSIONS)})')
    raw_params = spec.get('params')
    if not isinstance(raw_params, list):
        return [], 'live_tunable_spec has no params list'

    params = []
    for entry in raw_params:
        if not isinstance(entry, dict):
            continue
        if any(key not in entry for key in _REQUIRED_KEYS):
            continue
        kind = entry['kind']
        if kind not in ('float', 'bool'):
            continue
        try:
            minimum = float(entry['min'])
            maximum = float(entry['max'])
            step = float(entry.get('step', 0.0))
        except (TypeError, ValueError):
            continue
        if minimum > maximum:
            continue
        params.append({
            'name': str(entry['name']),
            'group': str(entry['group']),
            'label': str(entry.get('label') or entry['name']).replace('_', ' '),
            'kind': kind,
            'min': minimum,
            'max': maximum,
            'step': step,
            'unit': str(entry.get('unit', '')),
            'safety': bool(entry.get('safety', False)),
            'description': str(entry.get('description', '')),
        })
    if not params:
        return [], 'live_tunable_spec contained no usable parameters'
    return params, None


def coerce_request(param, value):
    """Vet one browser-requested value against a parsed spec entry.

    Returns (value, error). Floats are clamped into range rather than
    rejected: a slider pinned at its own maximum should apply the maximum,
    not fail. Wrong *types* are refused outright, because those mean the
    UI and the node disagree about what the knob is, which clamping would
    paper over.
    """
    if param['kind'] == 'bool':
        if not isinstance(value, bool):
            return None, f"'{param['name']}' expects true or false"
        return value, None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None, f"'{param['name']}' expects a number"
    number = float(value)
    if number != number or number in (float('inf'), float('-inf')):
        return None, f"'{param['name']}' must be a finite number"
    return min(max(number, param['min']), param['max']), None


def format_scalar(value):
    """A ROS-YAML-safe scalar.

    Floats always keep a decimal point: a `max_speed` written as bare `4`
    is an int as far as the parameter file loader is concerned, and a node
    that declared a double gets a type error on its next launch. That
    failure would land at the worst possible moment -- the run after the
    one where the tune was saved.
    """
    if isinstance(value, bool):
        return 'true' if value else 'false'
    text = f'{float(value):.6f}'.rstrip('0')
    return text + '0' if text.endswith('.') else text


def _indent_of(line):
    return len(line) - len(line.lstrip(' '))


def _is_structural(line):
    """A line that actually opens/closes YAML scope (not blank, not a comment)."""
    stripped = line.strip()
    return bool(stripped) and not stripped.startswith('#')


def update_yaml_values(text, node_name, values):
    """Rewrite `values` into a ROS parameter YAML, preserving everything else.

    Surgical line edits rather than a yaml.safe_load/safe_dump round trip,
    because these config files carry the reasoning behind their numbers --
    which value ranges were validated in the simulator, why a threshold is
    what it is, what breaks if you raise it. `yaml.dump` would return a
    correct file with every one of those comments deleted, quietly turning
    a "save my tune" click into the loss of the most valuable thing in the
    file. Key order, blank lines, and inline trailing comments all survive
    here; only the scalar after `key:` changes.

    Returns (new_text, changed, added):
      changed  keys that were found and rewritten
      added    keys that were absent and appended to the block
    Raises ValueError if the node's `ros__parameters` block isn't there.
    """
    lines = text.splitlines(keepends=True)

    node_pattern = re.compile(r'^(\s*)' + re.escape(node_name) + r'\s*:\s*(#.*)?$')
    node_index = next(
        (i for i, line in enumerate(lines) if node_pattern.match(line)), None)
    if node_index is None:
        raise ValueError(f"no '{node_name}:' block in this file")
    node_indent = _indent_of(lines[node_index])

    params_index = None
    for i in range(node_index + 1, len(lines)):
        line = lines[i]
        if not _is_structural(line):
            continue
        if _indent_of(line) <= node_indent:
            break  # left the node's block without finding it
        if re.match(r'^\s*ros__parameters\s*:\s*(#.*)?$', line):
            params_index = i
            break
    if params_index is None:
        raise ValueError(f"'{node_name}:' has no ros__parameters block")
    params_indent = _indent_of(lines[params_index])

    # Where the block ends, and what indent its keys sit at.
    block_end = len(lines)
    key_indent = None
    for i in range(params_index + 1, len(lines)):
        line = lines[i]
        if not _is_structural(line):
            continue
        indent = _indent_of(line)
        if indent <= params_indent:
            block_end = i
            break
        if key_indent is None:
            key_indent = indent
    if key_indent is None:
        key_indent = params_indent + 2

    # Trailing blank lines belong after anything we append, not before it.
    insert_at = block_end
    while insert_at > params_index + 1 and not lines[insert_at - 1].strip():
        insert_at -= 1

    changed, added = [], []
    for key, value in values.items():
        scalar = format_scalar(value)
        pattern = re.compile(
            r'^(?P<indent>\s*)(?P<key>' + re.escape(key) + r')'
            r'(?P<sep>\s*:\s*)(?P<value>.*?)(?P<comment>\s+#.*)?$')
        for i in range(params_index + 1, block_end):
            if not _is_structural(lines[i]):
                continue
            # Only keys at the block's own level -- never something nested
            # inside a mapping that happens to share a name.
            if _indent_of(lines[i]) != key_indent:
                continue
            match = pattern.match(lines[i].rstrip('\n'))
            if not match:
                continue
            newline = '\n' if lines[i].endswith('\n') else ''
            lines[i] = (match.group('indent') + match.group('key')
                        + match.group('sep') + scalar
                        + (match.group('comment') or '') + newline)
            changed.append(key)
            break
        else:
            lines.insert(insert_at, f"{' ' * key_indent}{key}: {scalar}\n")
            insert_at += 1
            block_end += 1
            added.append(key)

    return ''.join(lines), changed, added


def values_needing_save(file_text, node_name, live_values, tolerance=1e-9):
    """Which of the car's live values actually differ from the file's.

    Saving only the differences keeps `git diff` after a save honest: it
    shows the handful of numbers the session actually changed, instead of
    every tunable in the file reformatted by a round trip. Reviewing that
    diff is how a tune gets from "it felt better" to "it is committed", so
    it is worth keeping readable.
    """
    import yaml  # local: keeps this module importable without PyYAML

    try:
        document = yaml.safe_load(file_text) or {}
        on_disk = document[node_name]['ros__parameters']
    except (KeyError, TypeError, yaml.YAMLError):
        return dict(live_values)

    pending = {}
    for key, value in live_values.items():
        if key not in on_disk:
            pending[key] = value
            continue
        current = on_disk[key]
        if isinstance(value, bool) or isinstance(current, bool):
            if bool(current) != bool(value):
                pending[key] = value
        elif not isinstance(current, (int, float)):
            pending[key] = value
        elif abs(float(current) - float(value)) > tolerance:
            pending[key] = value
    return pending
