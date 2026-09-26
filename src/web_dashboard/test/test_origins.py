"""
Which web pages may open the dashboard's WebSocket (origins.py).

The oracle for every case here is the contract in docs/web-dashboard.md,
"Remote access through dashboard.sfuracerbot.ca": same-origin always,
plus exact scheme+host+port matches from allowed_origins, no wildcards.
The refusals are the half that matters (A8): each one is a page that
must NOT be able to reach the dashboard's write paths.

    python3 -m pytest src/web_dashboard/test/test_origins.py -v
"""
import pytest

from web_dashboard.origins import (
    is_origin_allowed,
    normalize_origin,
    parse_allowed_origins,
)

SITE = 'https://dashboard.sfuracerbot.ca'
ALLOWED, _ = parse_allowed_origins([SITE])
# The Host header a request through the tunnel's rb2-dash-origin hostname
# carries: cloudflared passes the public hostname through unchanged.
TUNNEL_HOST = 'rb2-dash-origin.sfuracerbot.ca'


# --------------------------------------------------------------------------
# The listed remote site
# --------------------------------------------------------------------------

def test_the_listed_site_is_allowed_through_the_tunnel():
    assert is_origin_allowed(SITE, TUNNEL_HOST, ALLOWED)


def test_an_explicit_default_port_is_the_same_origin_as_none():
    """https://x and https://x:443 are the same origin; a browser sends
    the first, but a config written with the second must still match."""
    allowed, rejected = parse_allowed_origins(['https://dashboard.sfuracerbot.ca:443'])
    assert rejected == []
    assert is_origin_allowed(SITE, TUNNEL_HOST, allowed)


def test_host_case_does_not_matter():
    assert is_origin_allowed('https://Dashboard.SFURacerbot.ca', TUNNEL_HOST, ALLOWED)


@pytest.mark.parametrize('origin', [
    # Lookalike domains: suffix, prefix, and the listed name used as a
    # subdomain of somebody else's.
    'https://dashboard.sfuracerbot.ca.evil.example',
    'https://evildashboard.sfuracerbot.ca',
    'https://dashboard-sfuracerbot.ca',
    'https://xdashboard.sfuracerbot.ca',
    'https://sfuracerbot.ca',
    # Same host, wrong scheme.
    'http://dashboard.sfuracerbot.ca',
    'wss://dashboard.sfuracerbot.ca',
    # Same host, wrong port.
    'https://dashboard.sfuracerbot.ca:8443',
    'https://dashboard.sfuracerbot.ca:80',
    # A trailing slash, a path, credentials: never a real Origin header,
    # so never a match.
    'https://dashboard.sfuracerbot.ca/',
    'https://dashboard.sfuracerbot.ca/ws',
    'https://user@dashboard.sfuracerbot.ca',
    # Sandboxed frames and file:// pages send the literal string 'null'.
    'null',
    '',
])
def test_everything_that_is_not_exactly_the_listed_site_is_refused(origin):
    assert not is_origin_allowed(origin, TUNNEL_HOST, ALLOWED)


# --------------------------------------------------------------------------
# Same-origin: every LAN / Tailscale page the node serves itself
# --------------------------------------------------------------------------

@pytest.mark.parametrize('origin, host', [
    ('http://192.168.0.42:8080', '192.168.0.42:8080'),
    ('http://racerbotcar-2:8080', 'racerbotcar-2:8080'),
    ('http://[fd7a:115c:a1e0::4133:7a3b]:8080', '[fd7a:115c:a1e0::4133:7a3b]:8080'),
    ('http://localhost:8080', 'localhost:8080'),           # editor port-forward
    ('https://dashboard-rb2.sfuracerbot.ca', 'dashboard-rb2.sfuracerbot.ca'),
])
def test_a_page_served_by_this_node_is_allowed_with_nothing_configured(origin, host):
    """With an EMPTY allowed_origins -- the parameter's default -- every
    way people used the dashboard before the remote site still works."""
    assert is_origin_allowed(origin, host, frozenset())


def test_same_origin_includes_the_port():
    """The camera page on :9090 is a different origin from :8080."""
    assert not is_origin_allowed(
        'http://192.168.0.42:9090', '192.168.0.42:8080', frozenset())


def test_a_foreign_page_on_the_lan_is_refused():
    """The attack this exists for: any site open in a browser on the
    car's WiFi opening ws://<car>:8080/ws."""
    assert not is_origin_allowed(
        'https://example.com', '192.168.0.42:8080', ALLOWED)


def test_a_missing_host_header_is_not_same_origin():
    assert not is_origin_allowed('http://192.168.0.42:8080', None, frozenset())
    assert not is_origin_allowed('http://192.168.0.42:8080', '', frozenset())


# --------------------------------------------------------------------------
# No Origin header at all
# --------------------------------------------------------------------------

def test_a_missing_origin_header_is_allowed():
    """Browsers always send Origin on a WebSocket, so none means a
    non-browser client (tools/racerbot_sim/capture_dashboard.py, a
    Python script) -- which Tornado never runs check_origin for either."""
    assert is_origin_allowed(None, TUNNEL_HOST, frozenset())


# --------------------------------------------------------------------------
# Parsing the config
# --------------------------------------------------------------------------

@pytest.mark.parametrize('entry', [
    'https://dashboard.sfuracerbot.ca/',     # trailing slash
    'https://*.sfuracerbot.ca',              # wildcards are not supported
    'dashboard.sfuracerbot.ca',              # no scheme
    'ftp://dashboard.sfuracerbot.ca',
    'https://dashboard.sfuracerbot.ca:99999',
    'null',
    '',
    None,
    42,
])
def test_a_malformed_config_entry_is_rejected_and_reported(entry):
    allowed, rejected = parse_allowed_origins([entry])
    assert allowed == frozenset()
    assert rejected == [str(entry)]


def test_a_rejected_entry_does_not_drop_the_good_ones():
    allowed, rejected = parse_allowed_origins(
        [SITE + '/', SITE, 'http://127.0.0.1:5173'])
    assert allowed == frozenset({SITE, 'http://127.0.0.1:5173'})
    assert rejected == [SITE + '/']


@pytest.mark.parametrize('entries', [[], None])
def test_no_entries_means_nothing_extra_is_allowed(entries):
    assert parse_allowed_origins(entries) == (frozenset(), [])


@pytest.mark.parametrize('raw, expected', [
    ('HTTPS://Dashboard.sfuracerbot.ca', 'https://dashboard.sfuracerbot.ca'),
    ('https://dashboard.sfuracerbot.ca:443', 'https://dashboard.sfuracerbot.ca'),
    ('http://car.local:80', 'http://car.local'),
    ('http://car.local:8080', 'http://car.local:8080'),
    ('https://car.local:80', 'https://car.local:80'),       # 80 is not https's default
    ('  https://car.local  ', 'https://car.local'),
])
def test_normalization_follows_the_default_port_rule(raw, expected):
    """Oracle: RFC 6454 section 4 -- an origin is (scheme, host, port),
    with the scheme's default port implied when none is written."""
    assert normalize_origin(raw) == expected
