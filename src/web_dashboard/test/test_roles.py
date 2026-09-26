"""
Connection roles (roles.py): what relay, control and direct connections
may send, and what each receives.

Oracle: the contract in docs/web-dashboard.md, "Remote access through
dashboard.sfuracerbot.ca" -- relay is read-only, control gets hello plus
the write panels' state and its own replies but no map/scan/batch, and a
connection with no role header is unchanged. Expected values are the
message types that contract names, not whatever the code emits today.

    python3 -m pytest src/web_dashboard/test/test_roles.py -v
"""
import pytest

from web_dashboard import roles


# Every browser->car write dashboard_node.py's on_message implements, as the
# browser actually sends it (web/dashboard.js). Kept here independently of
# roles.KNOWN_WRITES so a drift between the two shows up as a failure.
EVERY_WRITE = [
    {'type': 'tuning_control', 'action': 'arm', 'armed': True},
    {'type': 'tuning_control', 'action': 'set', 'node': 'pure_pursuit_node',
     'name': 'max_speed', 'value': 3.0},
    {'type': 'tuning_control', 'action': 'save'},
    {'type': 'process_control', 'action': 'stop', 'pid': 4242},
    {'type': 'map_control', 'action': 'delete', 'id': 'r1', 'confirm': 'r1',
     'digest': 'abc'},
    {'type': 'map_control', 'action': 'reset_slam'},
    {'type': 'stopwatch_control', 'action': 'set_enabled', 'enabled': True},
    {'type': 'stopwatch_control', 'action': 'reset'},
]
EVERY_READ = [
    {'type': 'process_control', 'action': 'refresh'},
    {'type': 'map_control', 'action': 'refresh'},
    {'type': 'map_control', 'action': 'clear_view'},
    {'type': 'tuning_control', 'action': 'refresh'},
]


def _ids(messages):
    return [f"{m['type']}/{m['action']}" for m in messages]


# --------------------------------------------------------------------------
# Parsing the role header
# --------------------------------------------------------------------------

def test_no_header_is_a_direct_connection():
    assert roles.parse_role(None) == (roles.ROLE_DIRECT, None)


@pytest.mark.parametrize('value, role', [
    ('relay', 'relay'), ('control', 'control'), (' control ', 'control')])
def test_the_two_contract_roles_are_recognised(value, role):
    assert roles.parse_role(value) == (role, None)


@pytest.mark.parametrize('value', [
    'admin', 'Relay', 'CONTROL', '', '   ', 'relay,control', 'control\x00',
    'x' * 500])
def test_an_unknown_role_is_refused_not_guessed(value):
    role, error = roles.parse_role(value)
    assert role is None
    assert error is not None and 'X-Racerbot-Role' in error
    assert len(error) < 200             # an attacker's header cannot flood the log
    assert '\x00' not in error


# --------------------------------------------------------------------------
# Who is writing
# --------------------------------------------------------------------------

def test_a_tunnel_user_is_logged_by_email():
    assert roles.format_user('alice@sfu.ca') == 'alice@sfu.ca'


@pytest.mark.parametrize('value', [None, '', '   ', '\n\r\t'])
def test_no_user_header_is_unknown_direct(value):
    assert roles.format_user(value) == 'unknown (direct)'


def test_the_relay_is_logged_as_having_no_user_not_as_direct():
    """The Durable Object has no person behind it; its log lines must not
    read as if they came from a LAN browser."""
    assert roles.format_user(None, roles.ROLE_RELAY) == 'none (relay)'
    assert roles.format_user(None, roles.ROLE_CONTROL) == 'unknown (direct)'


def test_a_user_header_cannot_forge_a_second_log_line():
    forged = 'mallory@x.ca\n[INFO] dashboard write: accepted (user alice@sfu.ca)'
    formatted = roles.format_user(forged)
    assert '\n' not in formatted and '\r' not in formatted
    assert formatted.startswith('mallory@x.ca')


def test_a_user_header_is_capped_at_an_email_length():
    """254 characters: the longest valid address (RFC 5321 path limit)."""
    assert len(roles.format_user('a' * 10_000)) == 254


# --------------------------------------------------------------------------
# Browser -> car: the relay may not write
# --------------------------------------------------------------------------

@pytest.mark.parametrize('payload', EVERY_WRITE, ids=_ids(EVERY_WRITE))
def test_a_relay_is_refused_every_write(payload):
    refusal = roles.inbound_refusal(roles.ROLE_RELAY, payload)
    assert refusal is not None
    assert 'use /control' in refusal         # what the troubleshooting table keys on


@pytest.mark.parametrize('payload', EVERY_WRITE, ids=_ids(EVERY_WRITE))
@pytest.mark.parametrize('role', [roles.ROLE_CONTROL, roles.ROLE_DIRECT])
def test_control_and_direct_may_send_every_write(role, payload):
    assert roles.inbound_refusal(role, payload) is None


def test_the_test_list_covers_every_write_the_module_knows():
    assert {(m['type'], m['action']) for m in EVERY_WRITE} == set(roles.KNOWN_WRITES)


@pytest.mark.parametrize('payload', EVERY_READ, ids=_ids(EVERY_READ))
def test_reads_are_allowed_on_the_relay(payload):
    """Re-listing and "send me the map again" change nothing -- and
    clear_view is how the Durable Object can resync its cached map."""
    assert not roles.is_write(payload)
    assert roles.inbound_refusal(roles.ROLE_RELAY, payload) is None


@pytest.mark.parametrize('payload', [
    {'type': 'tuning_control', 'action': 'delete_everything'},
    {'type': 'map_control', 'action': 'format_disk'},
    {'type': 'brand_new_control', 'action': 'go'},
    {'type': 'process_control'},                 # no action at all
    {},                                          # no type at all
    {'type': ['process_control'], 'action': 'refresh'},   # wrong JSON type
])
def test_anything_unrecognised_is_a_write_and_the_relay_refuses_it(payload):
    """Fail closed: a write path added later is refused on the relay
    until someone decides otherwise."""
    assert roles.is_write(payload)
    assert roles.inbound_refusal(roles.ROLE_RELAY, payload) is not None


def test_describe_request_names_the_write_without_newlines():
    line = roles.describe_request({
        'type': 'tuning_control', 'action': 'set', 'node': 'pure_pursuit_node',
        'name': 'max_speed\n[INFO] fake', 'value': 3.5})
    assert line.startswith('tuning_control set pure_pursuit_node.max_speed')
    assert '3.5' in line
    assert '\n' not in line


# --------------------------------------------------------------------------
# Car -> browser
# --------------------------------------------------------------------------

def _types(frames):
    return [header.get('type') for header, _ in frames]


TELEMETRY = [
    {'type': 'map', 'bytes': 10}, {'type': 'map_patch', 'bytes': 4},
    {'type': 'scan', 'bytes': 8}, {'type': 'pose'}, {'type': 'drive'},
    {'type': 'speed'}, {'type': 'intent'}, {'type': 'stats'},
    {'type': 'racing_line'},
]


@pytest.mark.parametrize('role', [roles.ROLE_RELAY, roles.ROLE_DIRECT])
@pytest.mark.parametrize('header', TELEMETRY + [
    {'type': 'batch', 'items': [{'type': 'pose'}]},
    {'type': 'process_result'}, {'type': 'something_new'}])
def test_relay_and_direct_get_every_message_untouched(role, header):
    """Including replies to someone else's request, and types this module
    has never heard of -- exactly what every connection got before."""
    for is_origin in (True, False):
        assert roles.frames_for(role, header, is_origin) == [(header, True)]


@pytest.mark.parametrize('header', TELEMETRY, ids=[h['type'] for h in TELEMETRY])
def test_control_never_gets_map_scan_or_telemetry(header):
    assert roles.frames_for(roles.ROLE_CONTROL, header, True) == []
    assert roles.frames_for(roles.ROLE_CONTROL, header, False) == []


@pytest.mark.parametrize('kind', ['hello', 'tuning', 'processes', 'saved_maps', 'stopwatch'])
def test_control_always_gets_the_write_panels_state(kind):
    header = {'type': kind}
    assert roles.frames_for(roles.ROLE_CONTROL, header, False) == [(header, False)]


@pytest.mark.parametrize('kind', [
    'tuning_armed', 'tuning_result', 'tuning_saved', 'process_result',
    'map_delete_result', 'slam_reset_result', 'write_refused'])
def test_control_gets_replies_to_its_own_requests_only(kind):
    header = {'type': kind}
    assert roles.frames_for(roles.ROLE_CONTROL, header, True) == [(header, False)]
    assert roles.frames_for(roles.ROLE_CONTROL, header, False) == []


def test_control_gets_the_stopwatch_out_of_a_batch_and_nothing_else():
    stopwatch = {'type': 'stopwatch', 'elapsed_s': 12.5}
    batch = {'type': 'batch', 'items': [
        {'type': 'pose', 'x': 1}, stopwatch, {'type': 'stats'},
        {'type': 'intent'}, 'not-a-dict']}
    assert roles.frames_for(roles.ROLE_CONTROL, batch, False) == [(stopwatch, False)]


@pytest.mark.parametrize('batch', [
    {'type': 'batch', 'items': [{'type': 'pose'}, {'type': 'speed'}]},
    {'type': 'batch', 'items': []},
    {'type': 'batch'},
    {'type': 'batch', 'items': None},
])
def test_a_batch_without_a_stopwatch_sends_control_nothing(batch):
    assert roles.frames_for(roles.ROLE_CONTROL, batch, False) == []


def test_control_never_gets_a_binary_frame():
    """Not even for a type it is allowed, should one ever grow a payload:
    a binary with no header of its own would be decoded as the wrong
    thing (see protocol.py on the one-slot desync)."""
    for kind in ('tuning', 'stopwatch', 'hello', 'process_result'):
        for _, with_binary in roles.frames_for(roles.ROLE_CONTROL, {'type': kind}, True):
            assert with_binary is False
