"""
Connection roles: who a WebSocket is, and what it may send and receive.

The remote site (sfu-racerbot/web-dashboards, served at
dashboard.sfuracerbot.ca) reaches this car through the Cloudflare Tunnel
and opens two kinds of connection, telling us which with a header:

  X-Racerbot-Role: relay    ONE per car, held by the site's Durable Object,
                            which fans the telemetry out to every viewer.
                            No person behind it, so it may not write.
  X-Racerbot-Role: control  One per signed-in person, for write actions.
                            Carries X-Racerbot-User: <email>.
  (no header)               A browser on the LAN or Tailscale, or the
                            frozen fallback page this node still serves.
                            Behaves exactly as it always has.

Why the headers can be believed at all: the Worker strips them from every
browser request and sets them itself, and the tunnel hostname is behind a
Cloudflare Access service-auth policy only the Worker passes. A direct LAN
client *can* forge them -- but forging `control` buys nothing a direct
connection did not already have, and forging `relay` only takes
capability away. X-Racerbot-User is therefore trustworthy for the audit
log only on connections that came through the tunnel.

An unrecognised role value is refused outright rather than guessed at: a
site and car that disagree about the contract should fail loudly at
connect time, not quietly get the wrong privileges.

No rclpy or Tornado import; unit-tested directly in test/test_roles.py.
"""

ROLE_RELAY = 'relay'
ROLE_CONTROL = 'control'
ROLE_DIRECT = None
KNOWN_ROLES = (ROLE_RELAY, ROLE_CONTROL)

UNKNOWN_USER = 'unknown (direct)'
RELAY_USER = 'none (relay)'
_MAX_USER_CHARS = 254  # the longest a valid email address can be

# --------------------------------------------------------------------------
# Browser -> car
# --------------------------------------------------------------------------

# (type, action) pairs that change nothing on the car or in this process:
# re-reading a list, or re-sending the map to the connection that asked.
# Everything NOT listed here is treated as a write -- including message
# types and actions nobody has invented yet, so a new write path added to
# dashboard_node.py is refused on the relay until someone decides
# otherwise, rather than allowed until someone remembers.
READ_ONLY_REQUESTS = frozenset({
    ('process_control', 'refresh'),
    ('map_control', 'refresh'),
    ('map_control', 'clear_view'),
    ('tuning_control', 'refresh'),
})

# Every write the node implements today, for documentation and tests. The
# classifier does not consult this -- it consults READ_ONLY_REQUESTS and
# defaults to "write".
KNOWN_WRITES = (
    ('tuning_control', 'arm'),
    ('tuning_control', 'set'),
    ('tuning_control', 'save'),
    ('process_control', 'stop'),
    ('map_control', 'delete'),
    ('map_control', 'reset_slam'),
    ('stopwatch_control', 'set_enabled'),
    ('stopwatch_control', 'reset'),
)

RELAY_REFUSAL = ('refused: this is the read-only relay connection -- '
                 'use /control for write actions')


def parse_role(header_value):
    """X-Racerbot-Role header -> (role, error).

    Absent -> (ROLE_DIRECT, None). 'relay'/'control' -> that role.
    Anything else, including an empty value or a different case, ->
    (None, reason) and the connection is refused.
    """
    if header_value is None:
        return ROLE_DIRECT, None
    value = str(header_value).strip()
    if value in KNOWN_ROLES:
        return value, None
    shown = value[:40] if value.isprintable() else repr(value[:40])
    return None, (f"unknown X-Racerbot-Role '{shown}' -- expected "
                  f"'relay' or 'control'")


def format_user(header_value, role=ROLE_DIRECT):
    """X-Racerbot-User header -> the name written to the log.

    Only printable characters survive, and the result is capped, so a
    header can never forge a second log line or bury the real one. The
    relay has no person behind it, and says so rather than claiming to be
    a direct connection.
    """
    fallback = RELAY_USER if role == ROLE_RELAY else UNKNOWN_USER
    if header_value is None:
        return fallback
    cleaned = ''.join(ch for ch in str(header_value) if ch.isprintable()).strip()
    if not cleaned:
        return fallback
    return cleaned[:_MAX_USER_CHARS]


def request_key(payload):
    """A browser message -> (type, action) as plain strings."""
    return (str(payload.get('type')), str(payload.get('action')))


def is_write(payload):
    """Does this browser message change anything? Unknown means yes."""
    return request_key(payload) not in READ_ONLY_REQUESTS


def inbound_refusal(role, payload):
    """None if `role` may send this message, else the refusal reason."""
    if role == ROLE_RELAY and is_write(payload):
        return RELAY_REFUSAL
    return None


def describe_request(payload):
    """One short, log-safe line naming what a write asked for."""
    kind, action = request_key(payload)
    detail = ''
    if kind == 'tuning_control' and action == 'set':
        detail = (f" {payload.get('node')!s:.60}.{payload.get('name')!s:.60}"
                  f"={payload.get('value')!r:.40}")
    elif kind == 'tuning_control' and action == 'arm':
        detail = f" armed={bool(payload.get('armed'))}"
    elif kind == 'process_control' and action == 'stop':
        detail = f" pid={payload.get('pid')!r:.20}"
    elif kind == 'map_control' and action == 'delete':
        detail = f" run={payload.get('id')!r:.80}"
    elif kind == 'stopwatch_control' and action == 'set_enabled':
        detail = f" enabled={bool(payload.get('enabled'))}"
    line = f'{kind:.40} {action:.40}{detail}'
    return ''.join(ch for ch in line if ch.isprintable())


# --------------------------------------------------------------------------
# Car -> browser
# --------------------------------------------------------------------------

# What a control connection always receives: the state its write panels
# are drawn from.
CONTROL_STATE_TYPES = frozenset({
    'hello', 'tuning', 'processes', 'saved_maps', 'stopwatch',
})

# Answers to a specific request. A control connection receives these only
# when the request was its own; relay and direct connections receive every
# one of them, exactly as every connection always has.
REPLY_TYPES = frozenset({
    'tuning_armed', 'tuning_result', 'tuning_saved',
    'process_result', 'map_delete_result', 'slam_reset_result',
    'map_cleared', 'write_refused',
})


def frames_for(role, header, is_origin):
    """Which JSON headers `role` should get for one outgoing message.

    `is_origin` is True when the message answers a request this very
    connection made. Returns a list of (header, send_binary) pairs -- an
    empty list means send nothing, including any binary payload, so a
    binary frame can never reach a connection whose header was dropped.

    Relay and direct: everything, untouched. Control: the state types, its
    own replies, and the stopwatch items pulled out of a batch as
    standalone messages (the stopwatch is the one piece of batched
    telemetry a write panel needs). Map keyframes and patches, scans,
    batches and everything unknown are dropped -- control is an allowlist.
    """
    if role != ROLE_CONTROL:
        return [(header, True)]
    kind = header.get('type')
    if kind in CONTROL_STATE_TYPES:
        return [(header, False)]
    if kind in REPLY_TYPES:
        return [(header, False)] if is_origin else []
    if kind == 'batch':
        return [(item, False) for item in header.get('items') or ()
                if isinstance(item, dict) and item.get('type') == 'stopwatch']
    return []
