"""
schema.py

The `/drive_intent` wire format: what a driving node says it is trying to
do, and why.

One JSON object per control decision, carried as a std_msgs/String. JSON
over String rather than a custom .msg is a deliberate choice -- the
racerbot_a and racerbot_b codebases are separate repositories that also
build in their own workspaces, and a shared rosidl interface package
would make every one of them depend on this one and rebuild together. A
hand-written JSON object costs those teams ~120 lines of dependency-free
C++ (include/drive_intent/drive_intent.hpp) and lands in the dashboard's
existing JSON WebSocket protocol with no translation at all. The cost,
stated plainly, is that there is no compile-time type checking: the
schema lives here, `validate()` is the enforcement, and consumers must
treat every incoming message as untrusted.

See docs/drive-intent.md for the field-by-field specification and the
porting guide.

No ROS, no rclpy -- pure data shaping, so it unit-tests without a robot
(test/test_schema.py), exactly like web_dashboard/protocol.py.
"""

import json
import math
import time

# Bump only for a *breaking* change. Consumers reject anything they don't
# know: a dashboard that silently half-renders a schema it doesn't
# understand is worse than one that says "unsupported intent version".
SCHEMA_VERSION = 1

BODY_FRAME = 'base_link'

# How the dashboard colours the arrow. Not a safety signal -- the car's
# actual safety behaviour is decided by the driving node and is already
# in `state` -- just a three-level "how much should this draw the eye".
SEVERITY_DRIVE = 'drive'
SEVERITY_CAUTION = 'caution'
SEVERITY_STOP = 'stop'
SEVERITIES = (SEVERITY_DRIVE, SEVERITY_CAUTION, SEVERITY_STOP)

# States in which the car is doing the ordinary thing its controller
# exists to do. Everything else -- corner fallbacks, overtakes, reactive
# overrides, every stop -- is worth the operator's attention and gets
# drawn as caution or stop.
NOMINAL_STATES = frozenset({'gap_follow', 'pure_pursuit'})

# A buggy or hostile publisher must not be able to make a phone-sized
# browser chew through a million-point path at 20Hz.
MAX_PATH_POINTS = 512
MAX_FACTORS = 32
MAX_TARGETS = 32
MAX_REASON_CHARS = 2000


# ---------------------------------------------------------------------------
# Building
# ---------------------------------------------------------------------------

def _finite(name, value) -> float:
    value = float(value)
    if not math.isfinite(value):
        # Caught by the caller's try/except and turned into a skipped
        # message. This matters more than it looks: json.dumps happily
        # emits bare `NaN`, which is not valid JSON, and JSON.parse in the
        # browser throws on it -- so one NaN would break the socket's
        # message stream rather than just one arrow.
        raise ValueError(f'{name} must be finite, got {value!r}')
    return value


def _r(value, places: int) -> float:
    return round(_finite('value', value), places)


def factor(name: str, value: float, unit: str = 'm/s',
           binding: bool = False) -> dict:
    """One named constraint the controller evaluated this tick.

    The point of publishing these is to answer the question that
    otherwise costs an evening of log archaeology: *which* limit is
    actually holding the car back right now. Both driving nodes in this
    workspace already compute their speed ceilings as separate named
    quantities and then take a min() -- this just stops throwing away
    which one won.
    """
    return {
        'name': str(name),
        'value': _r(value, 3),
        'unit': str(unit),
        'binding': bool(binding),
    }


def bind_min(factors, tol: float = 1e-6):
    """Mark the smallest-valued factor(s) as binding.

    Correct for speed ceilings specifically, which is what every factor
    in this workspace currently is: the command is the min of them, so
    the smallest one is the one in charge. Ties are all marked -- if the
    curvature cap and the clearance cap agree to within a micrometre per
    second, saying only one of them is responsible would be a lie of
    precision.
    """
    factors = [dict(f) for f in factors]
    if not factors:
        return factors
    lowest = min(f['value'] for f in factors)
    for f in factors:
        f['binding'] = abs(f['value'] - lowest) <= tol
    return factors


def classify_severity(state: str, speed: float) -> str:
    """Three-level display severity from the controller's own state."""
    if speed <= 0.0:
        return SEVERITY_STOP
    return SEVERITY_DRIVE if state in NOMINAL_STATES else SEVERITY_CAUTION


def memoize_reason(detail):
    """Wrap a possibly-expensive reason thunk so it is computed at most once.

    gap_follow passes its stop reasons as callables precisely because
    some of them are expensive (`_escape_report` re-runs the entire gap
    pipeline). Both the logger and the intent publisher want that string,
    and they throttle independently, so without this the thunk would run
    twice on the ticks where both fire. Strings pass straight through.
    """
    if not callable(detail):
        return detail
    cache = []

    def resolve():
        if not cache:
            cache.append(detail())
        return cache[0]

    return resolve


def resolve_reason(detail):
    """Evaluate a reason that may be a plain string or a thunk."""
    if detail is None:
        return None
    return str(detail() if callable(detail) else detail)


def _encode_path(points):
    """Integration output -> the compact {x, y, v} the browser draws.

    Rounded hard on purpose: at 20Hz with two paths per message, full
    float repr roughly triples the message size for precision far below
    one screen pixel at any zoom the dashboard supports.
    """
    encoded = []
    for point in list(points)[:MAX_PATH_POINTS]:
        x, y = point[0], point[1]
        v = point[3] if len(point) >= 4 else point[2]
        encoded.append({'x': _r(x, 3), 'y': _r(y, 3), 'v': _r(v, 2)})
    return encoded


def build(node: str, state: str, *, reason=None, severity=None,
          path=(), commanded_path=(),
          desired_steering: float = 0.0, commanded_steering: float = 0.0,
          desired_speed: float = 0.0, commanded_speed: float = 0.0,
          horizon_s: float = 0.0, factors=(), targets=(), wedge=None,
          frame: str = BODY_FRAME, stamp=None) -> dict:
    """Assemble one intent payload. Raises ValueError on non-finite input.

    `path` is what the algorithm wants; `commanded_path` is what the
    command actually sent to the mux will produce after slew-rate and
    acceleration shaping. Publishing both is what makes the shaping
    visible instead of mysterious -- otherwise the first person to notice
    that the arrow and the car disagree files a bug against the arrow.
    """
    payload = {
        'v': SCHEMA_VERSION,
        'stamp': float(time.time() if stamp is None else stamp),
        'node': str(node),
        'frame': str(frame),
        'state': str(state),
        'severity': str(severity if severity is not None
                        else classify_severity(state, commanded_speed)),
        'horizon_s': _r(horizon_s, 3),
        'desired_steering': _r(desired_steering, 4),
        'commanded_steering': _r(commanded_steering, 4),
        'desired_speed': _r(desired_speed, 3),
        'commanded_speed': _r(commanded_speed, 3),
        'path': _encode_path(path),
        'commanded_path': _encode_path(commanded_path),
        'factors': [dict(f) for f in list(factors)[:MAX_FACTORS]],
        'targets': [dict(t) for t in list(targets)[:MAX_TARGETS]],
    }
    resolved = resolve_reason(reason)
    if resolved is not None:
        payload['reason'] = resolved[:MAX_REASON_CHARS]
    if wedge is not None:
        payload['wedge'] = {
            'x': _r(wedge['x'], 3),
            'y': _r(wedge['y'], 3),
            'a0': _r(wedge['a0'], 4),
            'a1': _r(wedge['a1'], 4),
            'r': _r(wedge['r'], 3),
        }
    return payload


def target(kind: str, x: float, y: float) -> dict:
    """A single labelled point of interest in the body frame -- the gap
    the reactive controller picked, the waypoint pure pursuit is steering
    at, the opponent it is going around."""
    return {'kind': str(kind), 'x': _r(x, 3), 'y': _r(y, 3)}


def encode(payload: dict) -> str:
    """Serialize for the wire. `allow_nan=False` is load-bearing: it turns
    a numerical bug into a caught exception on the car rather than
    invalid JSON that breaks the browser's socket."""
    return json.dumps(payload, separators=(',', ':'), allow_nan=False)


# ---------------------------------------------------------------------------
# Consuming
# ---------------------------------------------------------------------------

def decode(text: str) -> dict:
    """Parse a wire message. Raises ValueError on anything malformed."""
    try:
        payload = json.loads(text)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'not valid JSON: {exc}') from exc
    if not isinstance(payload, dict):
        raise ValueError(f'expected a JSON object, got {type(payload).__name__}')
    return payload


def _check_points(points, label):
    if not isinstance(points, list):
        return f'{label} must be a list'
    if len(points) > MAX_PATH_POINTS:
        return f'{label} has {len(points)} points, over the {MAX_PATH_POINTS} limit'
    for i, point in enumerate(points):
        if not isinstance(point, dict):
            return f'{label}[{i}] must be an object'
        for key in ('x', 'y', 'v'):
            value = point.get(key)
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                return f'{label}[{i}].{key} must be a number'
            if not math.isfinite(value):
                return f'{label}[{i}].{key} must be finite'
    return None


def validate(payload: dict):
    """Return None if this payload is safe to render, else why it isn't.

    The dashboard is the only consumer that matters here and it is
    read-only, so this is not a security boundary in the "can it move the
    car" sense -- nothing downstream of it can. It exists so that one
    misbehaving publisher on the bus degrades into a warning line instead
    of a wedged browser tab or a stream of exceptions in the node.
    """
    if not isinstance(payload, dict):
        return 'payload must be an object'
    if payload.get('v') != SCHEMA_VERSION:
        return f"unsupported schema version {payload.get('v')!r} (need {SCHEMA_VERSION})"
    for key in ('node', 'state', 'frame'):
        if not isinstance(payload.get(key), str):
            return f'{key} must be a string'
    if payload.get('severity') not in SEVERITIES:
        return f"severity must be one of {SEVERITIES}, got {payload.get('severity')!r}"
    for key in ('stamp', 'horizon_s', 'desired_steering', 'commanded_steering',
                'desired_speed', 'commanded_speed'):
        value = payload.get(key)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return f'{key} must be a number'
        if not math.isfinite(value):
            return f'{key} must be finite'
    for label in ('path', 'commanded_path'):
        problem = _check_points(payload.get(label, []), label)
        if problem is not None:
            return problem
    factors = payload.get('factors', [])
    if not isinstance(factors, list) or len(factors) > MAX_FACTORS:
        return 'factors must be a list within the size limit'
    for i, f in enumerate(factors):
        if not isinstance(f, dict):
            return f'factors[{i}] must be an object'
        if not isinstance(f.get('name'), str):
            return f'factors[{i}].name must be a string'
        value = f.get('value')
        if not isinstance(value, (int, float)) or isinstance(value, bool) \
                or not math.isfinite(value):
            return f'factors[{i}].value must be a finite number'
    targets = payload.get('targets', [])
    if not isinstance(targets, list) or len(targets) > MAX_TARGETS:
        return 'targets must be a list within the size limit'
    reason = payload.get('reason')
    if reason is not None and not isinstance(reason, str):
        return 'reason must be a string when present'
    return None


def binding_factor(payload: dict):
    """The name of the constraint currently limiting the car, or None."""
    for f in payload.get('factors', []):
        if f.get('binding'):
            return f.get('name')
    return None
