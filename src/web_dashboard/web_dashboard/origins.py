"""
Which web pages may open a WebSocket to this dashboard.

Every browser attaches an `Origin` header to a WebSocket handshake: the
scheme, host and port of the page that opened it. Until the remote site
existed, dashboard_node accepted every Origin at all. That let any web
page a person on the car's WiFi happened to have open talk to the car --
including the write paths (tuning, process stop, map delete). The name
for that is cross-site WebSocket hijacking.

The rule now is:

  * **Same origin** -- the page was served by this dashboard itself, so
    the Origin's host:port equals the request's own `Host` header. This is
    every LAN, Tailscale and forwarded-port use, and the old
    dashboard-rb2.sfuracerbot.ca tunnel (cloudflared passes the public
    hostname through as `Host`). Unchanged from what worked before.
  * **Listed** -- the Origin equals, exactly, one entry of
    `allowed_origins` (scheme + host + port, no wildcards, no paths).
    Shipped with `https://dashboard.sfuracerbot.ca`, the remote site whose
    Worker connects through the tunnel.
  * **No Origin header** -- allowed. Browsers always send one on a
    WebSocket, so a missing header means a non-browser client (the
    capture tools in tools/, a Python script). The attack this defends
    against needs a browser. Tornado itself never calls check_origin when
    the header is absent, and this function says the same thing so the
    decision is written down in one testable place.

Everything else is refused (Tornado answers 403 before the upgrade).

No rclpy or Tornado import, so it is unit-tested directly
(`test/test_origins.py`) the same way netbind.py is.
"""

from urllib.parse import urlsplit

_DEFAULT_PORTS = {'http': 80, 'https': 443}


def normalize_origin(value):
    """`'https://Dashboard.Example.ca:443'` -> `'https://dashboard.example.ca'`.

    Returns None for anything that is not a bare scheme://host[:port]:
    a path (even a lone trailing slash), a query, credentials, an unknown
    scheme, or the literal `null` browsers send from sandboxed frames.
    Being strict here is the point: an Origin header never has a path,
    so a config entry with one is a typo that must not match anything.
    """
    if not isinstance(value, str):
        return None
    text = value.strip()
    if not text or text == 'null':
        return None
    try:
        parts = urlsplit(text)
        port = parts.port
    except ValueError:
        return None
    scheme = parts.scheme.lower()
    if scheme not in _DEFAULT_PORTS:
        return None
    if parts.path or parts.query or parts.fragment:
        return None
    if parts.username is not None or parts.password is not None:
        return None
    host = (parts.hostname or '').lower()
    if not host or '*' in host:
        return None
    if ':' in host:  # IPv6 literal -- urlsplit drops the brackets
        host = f'[{host}]'
    if port is None or port == _DEFAULT_PORTS[scheme]:
        return f'{scheme}://{host}'
    return f'{scheme}://{host}:{port}'


def parse_allowed_origins(entries):
    """Config list -> (frozenset of normalized origins, rejected entries).

    The rejected ones are returned rather than dropped silently so the
    node can log each at startup -- `https://dashboard.sfuracerbot.ca/`
    with a trailing slash would otherwise just never match, and the only
    symptom would be a 403 on the far side of a tunnel.
    """
    allowed, rejected = set(), []
    for entry in entries or ():
        normalized = normalize_origin(entry)
        if normalized is None:
            rejected.append(str(entry))
        else:
            allowed.add(normalized)
    return frozenset(allowed), rejected


def _host_header_matches(origin, host_header):
    """Tornado's own same-origin rule: Origin's host:port == Host header.

    Compared on the netloc, as Tornado does, so the scheme is not part of
    the same-origin half -- a page served over the tunnel's https and
    connecting back to it is the same site.
    """
    if not host_header:
        return False
    try:
        netloc = urlsplit(origin.strip()).netloc.lower()
    except ValueError:
        return False
    return bool(netloc) and netloc == host_header.strip().lower()


def is_origin_allowed(origin, host_header, allowed):
    """The whole decision. `allowed` is parse_allowed_origins()'s set."""
    if origin is None:
        return True
    if _host_header_matches(origin, host_header):
        return True
    normalized = normalize_origin(origin)
    return normalized is not None and normalized in allowed
