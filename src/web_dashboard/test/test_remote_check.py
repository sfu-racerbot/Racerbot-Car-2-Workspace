"""
remote_check.py's verdicts: how it reads each hop between the site and the car.

Oracles: the responses recorded from this car on 2026-09-27 (a zone-wide
bot challenge answering `cf-mitigated: challenge` + "Just a moment...", and
cloudflared's own journal lines), Cloudflare's documented error codes
(1016 no DNS/route, 1033 tunnel down, 530 origin unreachable), and the
tunnel contract in docs/web-dashboard.md "What runs where". The FAIL
verdicts are the half that matters (A8): a checker that says "ok" to a
broken hop sends people looking in the wrong place.

    python3 -m pytest src/web_dashboard/test/test_remote_check.py -v
"""
import pytest

from web_dashboard.remote_check import (
    FAIL, OK, WARN,
    classify_edge, classify_hello, classify_ingress, classify_listener,
    cloudflare_error_code, parse_ingress, recent_remote_events,
)

# Recorded 2026-09-27 from `curl https://rb2-dash-origin.sfuracerbot.ca/ws`.
CHALLENGE_HEADERS = {'cf-mitigated': 'challenge', 'server': 'cloudflare'}
CHALLENGE_BODY = '<!DOCTYPE html><html lang="en-US"><head><title>Just a moment...</title>'


# --------------------------------------------------------------------------
# The Cloudflare edge
# --------------------------------------------------------------------------

def test_a_bot_challenge_is_a_failure_even_though_it_is_a_403():
    """The trap: a challenge is also HTTP 403, which looks like Access
    doing its job. It is not -- the Worker cannot solve it."""
    status, message, fix = classify_edge(403, CHALLENGE_HEADERS, CHALLENGE_BODY)
    assert status == FAIL
    assert 'challenge' in message
    assert 'Bot Fight Mode' in fix


@pytest.mark.parametrize('headers, body', [
    (CHALLENGE_HEADERS, ''),                  # header alone
    ({'cf-mitigated': 'Challenge'}, ''),       # case
    ({}, CHALLENGE_BODY),                     # page alone
])
def test_either_sign_of_a_challenge_is_enough(headers, body):
    assert classify_edge(403, headers, body)[0] == FAIL


def test_access_refusing_an_unauthenticated_request_is_the_healthy_answer():
    assert classify_edge(403, {'server': 'cloudflare'}, 'Forbidden')[0] == OK
    assert classify_edge(302, {'location': 'https://sfu.cloudflareaccess.com/cdn-cgi/access/login'}, '')[0] == OK


@pytest.mark.parametrize('status', [200, 101, 204])
def test_an_answer_without_credentials_means_access_is_missing(status):
    status_, message, fix = classify_edge(status, {}, '<html>dashboard</html>')
    assert status_ == FAIL
    assert 'NOT protected' in message


@pytest.mark.parametrize('status, body, needle', [
    (530, 'error code: 1033', 'tunnel is not connected'),
    (530, '', 'tunnel is not connected'),
    (404, 'Error 1016', 'no DNS'),
    (502, 'Bad gateway', 'nothing answered on the car'),
    (504, '', 'nothing answered on the car'),
])
def test_cloudflare_errors_name_the_broken_hop(status, body, needle):
    status_, message, _ = classify_edge(status, {}, body)
    assert status_ == FAIL
    assert needle in message


def test_an_unknown_answer_is_a_warning_not_ok():
    assert classify_edge(418, {}, '')[0] == WARN
    assert classify_edge(None, {}, None)[0] == WARN


@pytest.mark.parametrize('body, code', [
    ('error code: 1033', 1033), ('Error 1016 Origin DNS error', 1016),
    ('ERROR CODE 1001', 1001), ('error code: 523', None), ('', None), (None, None)])
def test_cloudflare_error_code(body, code):
    assert cloudflare_error_code(body) == code


# --------------------------------------------------------------------------
# The tunnel route, from cloudflared's journal
# --------------------------------------------------------------------------

# Shape recorded from `journalctl -u cloudflared -o cat` on 2026-09-27.
JOURNAL = (
    '2026-09-27T19:38:30Z ERR Request failed error="dial tcp 127.0.0.1:8080: connect: connection refused"\n'
    '2026-09-27T19:40:11Z INF Updated to new configuration config="{\\"ingress\\":['
    '{\\"hostname\\":\\"ssh-rb2.sfuracerbot.ca\\",\\"service\\":\\"ssh://localhost:22\\"},'
    '{\\"hostname\\":\\"rb2-dash-origin.sfuracerbot.ca\\",\\"service\\":\\"http://127.0.0.1:8080\\"},'
    '{\\"hostname\\":\\"rb2-cam-origin.sfuracerbot.ca\\",\\"service\\":\\"http://localhost:9091\\"},'
    '{\\"service\\":\\"http_status:404\\"}],\\"warp-routing\\":{\\"enabled\\":false}}" version=13\n'
)


def test_the_last_configuration_line_is_parsed_including_the_catch_all():
    assert parse_ingress(JOURNAL) == [
        ('ssh-rb2.sfuracerbot.ca', 'ssh://localhost:22'),
        ('rb2-dash-origin.sfuracerbot.ca', 'http://127.0.0.1:8080'),
        ('rb2-cam-origin.sfuracerbot.ca', 'http://localhost:9091'),
        ('(catch-all)', 'http_status:404'),
    ]


def test_only_the_latest_configuration_counts():
    older = JOURNAL.replace('version=13', 'version=12').replace('127.0.0.1:8080', '127.0.0.1:1')
    assert ('rb2-dash-origin.sfuracerbot.ca', 'http://127.0.0.1:8080') in parse_ingress(older + JOURNAL)


@pytest.mark.parametrize('text', ['', 'no config lines here\n',
                                  'Updated to new configuration config="{not json" version=1'])
def test_an_unreadable_journal_is_none(text):
    assert parse_ingress(text) is None


def test_ingress_verdicts():
    ingress = parse_ingress(JOURNAL)
    assert classify_ingress(ingress, 'rb2-dash-origin', 8080)[0] == OK
    assert classify_ingress(ingress, 'rb2-bridge-origin', 8765)[0] == FAIL   # no route
    wrong_port = classify_ingress(ingress, 'rb2-cam-origin', 9090)
    assert wrong_port[0] == FAIL and '9091' in wrong_port[1]
    assert classify_ingress(None, 'rb2-dash-origin', 8080)[0] == WARN


def test_a_route_to_a_port_that_merely_starts_the_same_is_wrong():
    ingress = [('rb2-dash-origin.sfuracerbot.ca', 'http://127.0.0.1:80800')]
    assert classify_ingress(ingress, 'rb2-dash-origin', 8080)[0] == FAIL


# --------------------------------------------------------------------------
# Listeners and the local handshake
# --------------------------------------------------------------------------

@pytest.mark.parametrize('addresses, expected', [
    (['0.0.0.0:8080', '[::]:8080'], OK),
    (['127.0.0.1:8765'], OK),
    (['*:9090'], OK),
    (['192.168.0.15:8080'], FAIL),          # the tunnel dials 127.0.0.1
    (['0.0.0.0:18080'], FAIL),              # a different port that ends the same
    ([], FAIL),
])
def test_listener_verdicts(addresses, expected):
    port = int(addresses[0].rsplit(':', 1)[1]) if addresses and expected == OK else 8080
    assert classify_listener(port, addresses)[0] == expected


def test_hello_must_be_first():
    assert classify_hello([{'type': 'hello', 'protocol_version': 1}, {'type': 'scan'}])[0] == OK
    old = classify_hello([{'type': 'scan'}, {'type': 'hello'}])
    assert old[0] == FAIL and 'old dashboard_node' in old[1]
    assert classify_hello([])[0] == FAIL
    assert classify_hello(['<binary>'])[0] == FAIL


# --------------------------------------------------------------------------
# What reached the dashboard, from its log lines (server.py's format)
# --------------------------------------------------------------------------

LOG = [
    '[INFO] [1790450000.100000000] [web_dashboard_node]: relay connection #8 opened (user none (relay), from 127.0.0.1, 1 open)\n',
    '[INFO] [1790450001.000000000] [web_dashboard_node]: direct connection #9 opened (user unknown (direct), from 192.168.0.4, 2 open)\n',
    '[WARN] [1790450002.000000000] [web_dashboard_node]: refused a WebSocket from origin \'https://evil.example\' (not same-origin and not in allowed_origins)\n',
    '[INFO] [1790440000.000000000] [web_dashboard_node]: control connection #2 opened (user a@b, from 127.0.0.1, 1 open)\n',
    '[INFO] [1790450003.000000000] [other_node]: relay connection #1 opened\n',
]


def test_only_site_connections_and_refusals_since_the_cutoff_are_reported():
    events = recent_remote_events(LOG, since_epoch=1790449999.0)
    assert [text.split(' (')[0] for _, text in events] == [
        'relay connection #8 opened', 'refused a WebSocket from origin \'https://evil.example\'']


def test_duplicate_lines_from_two_log_files_count_once():
    assert len(recent_remote_events(LOG + LOG, since_epoch=1790449999.0)) == 2


def test_no_lines_means_no_events():
    assert recent_remote_events([], 0) == []
