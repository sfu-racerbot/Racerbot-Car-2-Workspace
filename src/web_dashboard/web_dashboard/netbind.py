"""
Which socket(s) the dashboard's web server should actually listen on.

This is one small decision, pulled into its own module because it is the
difference between "reachable from the laptop on the bench" and "reachable
from anywhere", and because getting it wrong fails in a way nobody debugs
correctly: the page loads fine over the LAN and simply times out over
Tailscale, which reads as a VPN problem rather than as a bind problem.

The trap is that `0.0.0.0` does **not** mean "every interface". It means
every *IPv4* interface. A Tailscale node has both a 100.x IPv4 address and
an `fd7a:...` IPv6 one, and MagicDNS publishes both -- so a browser opening
`http://<machine>:8080/` may resolve the AAAA record, connect over IPv6,
and get connection-refused by a server that only ever bound IPv4.

So a wildcard host here is deliberately turned into "bind every address
family", which is what anyone writing `0.0.0.0` in the config meant.

No rclpy import: this is plain logic, so it is unit-tested directly
(`test/test_netbind.py`) with no ROS, no build and no running car.
"""

# Everything a person plausibly writes in the config to mean "listen
# everywhere". '::' is the IPv6 wildcard; '*' and '' are what other web
# servers accept for the same idea.
WILDCARD_HOSTS = frozenset({'0.0.0.0', '::', '*'})


def wants_all_interfaces(host):
    """True if `host` means "listen on every interface, both IP families".

    Callers bind with no address at all in that case, which is what yields
    both the IPv4 and the IPv6 wildcard socket -- rather than passing
    '0.0.0.0' straight through to the server and quietly getting IPv4 only.
    """
    return str(host).strip() in WILDCARD_HOSTS or not str(host).strip()


def describe(host, port):
    """The line the node logs at startup.

    It names the families that are actually listening, because "Serving on
    http://0.0.0.0:8080/" is not an address anyone can open, and it hides
    the exact distinction this module exists to get right.
    """
    if wants_all_interfaces(host):
        return (f'Serving on port {port}, every interface, IPv4 + IPv6 '
                f'-- open http://<car-ip-or-hostname>:{port}/')
    return f'Serving on http://{host}:{port}/ (this address only)'
