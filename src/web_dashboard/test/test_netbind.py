"""
The dashboard has to be reachable over Tailscale, not just over the LAN.

These pin the one decision in netbind.py, and then actually bind a socket
the way dashboard_node.py does to prove the decision has the effect it
claims -- because the bug being guarded against here (IPv4-only listener)
is invisible to any test that only checks a return value.

    python3 -m pytest src/web_dashboard/test/test_netbind.py -v
"""
import socket

import pytest

from web_dashboard.netbind import describe, wants_all_interfaces


# --------------------------------------------------------------------------
# The decision itself
# --------------------------------------------------------------------------

@pytest.mark.parametrize('host', ['0.0.0.0', '::', '*', '', '  ', ' 0.0.0.0 '])
def test_wildcard_hosts_mean_every_interface(host):
    assert wants_all_interfaces(host)


@pytest.mark.parametrize('host', ['127.0.0.1', 'localhost', '192.168.0.15',
                                  '100.107.122.58', '::1'])
def test_a_real_address_is_left_alone(host):
    """Someone who names one address wants exactly that one -- deliberately
    restricting the dashboard to loopback is a legitimate thing to ask for
    and must not be silently widened to every interface."""
    assert not wants_all_interfaces(host)


def test_the_startup_line_never_prints_an_unopenable_url():
    """'Serving on http://0.0.0.0:8080/' is not an address anyone can
    paste into a browser, and it hides which families are listening."""
    line = describe('0.0.0.0', 8080)
    assert 'http://0.0.0.0' not in line
    assert 'IPv4 + IPv6' in line
    assert '8080' in line

    assert 'http://127.0.0.1:8080/' in describe('127.0.0.1', 8080)


# --------------------------------------------------------------------------
# ...and the effect it has, which is the part that actually broke
# --------------------------------------------------------------------------

def _families(sockets):
    return {s.family for s in sockets}


def test_a_wildcard_host_really_binds_both_ip_families():
    """The regression test for "unreachable over Tailscale".

    `bind_sockets(port, address='0.0.0.0')` returns one AF_INET socket, so
    a browser reaching the car over its IPv6 address -- which is what
    Tailscale MagicDNS hands out alongside the 100.x one -- got connection
    refused. Binding with no address returns both.
    """
    tornado_netutil = pytest.importorskip('tornado.netutil')

    old_way = tornado_netutil.bind_sockets(0, address='0.0.0.0')
    try:
        assert _families(old_way) == {socket.AF_INET}, (
            'assumption broken: 0.0.0.0 is expected to be IPv4-only')
    finally:
        for sock in old_way:
            sock.close()

    new_way = tornado_netutil.bind_sockets(0)
    try:
        families = _families(new_way)
        assert socket.AF_INET6 in families, (
            'no IPv6 listener: the dashboard stays unreachable at the '
            "car's Tailscale/MagicDNS IPv6 address")
        assert socket.AF_INET in families, (
            'no IPv4 listener: the dashboard stops working on the LAN')
    finally:
        for sock in new_way:
            sock.close()


def test_a_named_host_binds_only_that_host():
    tornado_netutil = pytest.importorskip('tornado.netutil')

    sockets = tornado_netutil.bind_sockets(0, address='127.0.0.1')
    try:
        assert _families(sockets) == {socket.AF_INET}
        assert all(s.getsockname()[0] == '127.0.0.1' for s in sockets)
    finally:
        for sock in sockets:
            sock.close()
