"""
"Why isn't dashboard.sfuracerbot.ca reaching this car?" -- answered from
the car, one hop at a time, each with a plain-language fix.

    ros2 run web_dashboard remote_check          # or: python3 -m web_dashboard.remote_check

Read-only and safe any time: it opens one relay WebSocket to the local
dashboard (read-only by definition), makes plain HTTPS GETs to the public
hostnames with NO credentials, and reads cloudflared's journal and ROS's
log files. It never publishes, never writes, never restarts anything.

Why it exists: on 2026-09-27 the site could not connect, and every piece on
the car looked healthy. The real faults were at Cloudflare -- a zone-wide
bot challenge in front of every hostname, and a tunnel route whose DNS
record was never created -- and none of that is visible from the car's own
logs. It is visible from the car's *network*, which is what this checks.

The site repo (sfu-racerbot/web-dashboards) has the other half: its
/<car>/check page probes the same hostnames from the Worker, with the real
service token. Run both; together they bracket the fault.

The classification functions have no network, ROS or Tornado imports, so
test/test_remote_check.py tests them directly.
"""

import asyncio
import glob
import json
import os
import re
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

SITE_ORIGIN = 'https://dashboard.sfuracerbot.ca'
DOMAIN = 'sfuracerbot.ca'
# The tunnel contract, docs/web-dashboard.md "What runs where".
ORIGINS = {
    'rb2-dash-origin': (8080, 'dashboard_node'),
    'rb2-bridge-origin': (8765, 'foxglove_bridge'),
    'rb2-cam-origin': (9090, 'usb_cam_stream (camera)'),
}

OK, WARN, FAIL = 'ok', 'WARN', 'FAIL'


# --------------------------------------------------------------------------
# Pure classification -- unit-tested
# --------------------------------------------------------------------------

def classify_listener(port, local_addresses):
    """`ss -tln` local addresses for one port -> (status, message, fix)."""
    mine = [a for a in local_addresses if a.rsplit(':', 1)[-1] == str(port)]
    if not mine:
        return (FAIL, f'nothing is listening on port {port}',
                'start it (see docs/web-dashboard.md "What runs where")')
    hosts = {a.rsplit(':', 1)[0] for a in mine}
    reachable = hosts & {'127.0.0.1', '0.0.0.0', '*', '[::]', '::'}
    if not reachable:
        return (FAIL, f'port {port} listens only on {sorted(hosts)}, which the '
                      f'tunnel (dialling 127.0.0.1) cannot reach',
                'bind it to 127.0.0.1 or 0.0.0.0')
    return (OK, f'port {port} listening on {", ".join(sorted(hosts))}', None)


def classify_edge(status, headers, body_start):
    """How Cloudflare's edge answered an UNAUTHENTICATED GET to one origin
    hostname -> (status, message, fix).

    `headers` is a dict with lower-case keys. What we WANT to see is Access
    refusing us (we sent no service token). Anything that answers before
    Access -- a bot challenge -- also stops the site's Worker, which cannot
    solve a JavaScript challenge.
    """
    body = body_start or ''
    location = headers.get('location', '')
    if headers.get('cf-mitigated', '').lower() == 'challenge' or 'Just a moment...' in body:
        return (FAIL, 'Cloudflare answers with a bot challenge ("Just a moment..."). '
                      'Browsers pass it; the site\'s Worker cannot, so it never reaches Access or the car',
                'Cloudflare dashboard > sfuracerbot.ca > Security: turn off Bot Fight Mode / '
                'lower Security Level, or add a WAF custom rule that SKIPS managed challenges '
                'for the rb2-*-origin hostnames')
    code = cloudflare_error_code(body)
    if code == 1016 or code == 1001:
        return (FAIL, f'Cloudflare has no DNS/route for this hostname (error {code})',
                'add the tunnel route (public hostname) for it')
    if code == 1033 or status == 530:
        return (FAIL, 'the tunnel is not connected to Cloudflare (530/1033)',
                'sudo systemctl status cloudflared')
    if 'cloudflareaccess.com' in location or status in (401, 403):
        return (OK, f'Cloudflare Access is in front of it (HTTP {status} without a token) -- expected', None)
    if status in (502, 503, 504):
        return (FAIL, f'tunnel up but nothing answered on the car (HTTP {status})',
                'check the service is running and the route\'s port')
    if status is not None and (200 <= status < 300 or status == 101):
        return (FAIL, f'answered HTTP {status} with NO credentials -- this hostname is NOT '
                      f'protected by Cloudflare Access',
                'add an Access application with a Service Auth policy for it')
    return (WARN, f'unexpected answer: HTTP {status}', None)


def cloudflare_error_code(body):
    match = re.search(r'error(?: code)?:?\s*(1\d{3})', body or '', re.IGNORECASE)
    return int(match.group(1)) if match else None


def parse_ingress(journal_text):
    """The LAST 'Updated to new configuration' line in cloudflared's journal
    -> list of (hostname, service). None if there is no such line."""
    lines = [l for l in journal_text.splitlines() if 'Updated to new configuration' in l]
    if not lines:
        return None
    raw = lines[-1].split('config="', 1)[1].replace('\\"', '"')
    try:
        config, _ = json.JSONDecoder().raw_decode(raw)
    except ValueError:
        return None
    return [(r.get('hostname', '(catch-all)'), r.get('service', ''))
            for r in config.get('ingress', [])]


def classify_ingress(ingress, host, port):
    if ingress is None:
        return (WARN, 'could not read the tunnel configuration from cloudflared\'s journal',
                'journalctl -u cloudflared | grep "new configuration"')
    services = [svc for h, svc in ingress if h == f'{host}.{DOMAIN}']
    if not services:
        return (FAIL, f'the tunnel has no route for {host}.{DOMAIN}',
                f'Zero Trust > Networks > Tunnels > Public Hostname: add it -> HTTP 127.0.0.1:{port}')
    if not re.search(rf'//(127\.0\.0\.1|localhost):{port}$', services[0]):
        return (FAIL, f'the route points at {services[0]}, not port {port}',
                f'edit it to HTTP 127.0.0.1:{port}')
    return (OK, f'tunnel routes it to {services[0]}', None)


def classify_hello(frames):
    """First frames from a local relay connection -> verdict."""
    if not frames:
        return (FAIL, 'connected but received nothing', None)
    first = frames[0]
    if not (isinstance(first, dict) and first.get('type') == 'hello'):
        return (FAIL, f'first message is {first!r:.60}, not hello -- an old dashboard_node is running',
                'rebuild web_dashboard and restart the dashboard')
    return (OK, f"hello first, protocol_version {first.get('protocol_version')}, "
                f"then {len(frames) - 1} more frame(s)", None)


REMOTE_LOG = re.compile(r'\[(\d+\.\d+)\] \[web_dashboard_node\]: '
                        r'((relay|control) connection #\d+ (opened|closed).*|refused a WebSocket.*)')


def recent_remote_events(log_lines, since_epoch):
    """ROS log lines -> [(epoch, text)] of site connections and refusals."""
    events = []
    for line in log_lines:
        match = REMOTE_LOG.search(line)
        if match and float(match.group(1)) >= since_epoch:
            events.append((float(match.group(1)), match.group(2)))
    return sorted(set(events))


# --------------------------------------------------------------------------
# The checks themselves (network, subprocess, files)
# --------------------------------------------------------------------------

def _local_addresses():
    out = subprocess.run(['ss', '-tln'], capture_output=True, text=True).stdout
    return [line.split()[3] for line in out.splitlines()[1:] if len(line.split()) > 3]


def _public_get(url):
    request = urllib.request.Request(url, headers={'User-Agent': 'racerbot-remote-check'})
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            return response.status, {k.lower(): v for k, v in response.headers.items()}, \
                response.read(600).decode('utf-8', 'replace')
    except urllib.error.HTTPError as err:
        return err.code, {k.lower(): v for k, v in err.headers.items()}, \
            err.read(600).decode('utf-8', 'replace')


async def _local_relay_frames(port):
    import tornado.httpclient
    import tornado.websocket
    request = tornado.httpclient.HTTPRequest(
        f'ws://127.0.0.1:{port}/ws?role=relay',
        headers={'Origin': SITE_ORIGIN, 'Host': f'rb2-dash-origin.{DOMAIN}',
                 'X-Racerbot-Role': 'relay'})
    ws = await tornado.websocket.websocket_connect(request)
    frames = []
    try:
        for _ in range(4):
            frame = await asyncio.wait_for(ws.read_message(), 3)
            if frame is None:
                break
            frames.append(json.loads(frame) if isinstance(frame, str) else '<binary>')
    except asyncio.TimeoutError:
        pass
    ws.close()
    return frames


def _print(label, verdict):
    status, message, fix = verdict
    print(f'  {status:4}  {label}: {message}')
    if fix and status != OK:
        print(f'        fix: {fix}')
    return status


def main(argv=None):
    statuses = []
    print('1. services on the car (the tunnel dials 127.0.0.1)')
    addresses = _local_addresses()
    for host, (port, name) in ORIGINS.items():
        statuses.append(_print(name, classify_listener(port, addresses)))

    print('2. local dashboard, as the site\'s relay connects')
    try:
        frames = asyncio.run(_local_relay_frames(8080))
        statuses.append(_print('relay handshake', classify_hello(frames)))
    except Exception as exc:  # noqa: BLE001 -- report, never crash the checker
        statuses.append(_print('relay handshake', (
            FAIL, f'{type(exc).__name__}: {exc}',
            '403 = allowed_origins; 400 = role header; refused = dashboard not running')))

    print('3. tunnel (cloudflared)')
    active = subprocess.run(['systemctl', 'is-active', 'cloudflared'],
                            capture_output=True, text=True).stdout.strip()
    statuses.append(_print('cloudflared service', (OK, 'active', None) if active == 'active'
                           else (FAIL, active or 'unknown', 'sudo systemctl status cloudflared')))
    journal = subprocess.run(['journalctl', '-u', 'cloudflared', '--since', '-7d',
                              '--no-pager', '-o', 'cat'],
                             capture_output=True, text=True).stdout
    ingress = parse_ingress(journal)
    for host, (port, _) in ORIGINS.items():
        statuses.append(_print(f'route {host}', classify_ingress(ingress, host, port)))

    print('4. public DNS and Cloudflare edge (no credentials sent)')
    for host in ['dashboard'] + list(ORIGINS):
        fqdn = f'{host}.{DOMAIN}'
        try:
            socket.getaddrinfo(fqdn, 443)
        except socket.gaierror:
            statuses.append(_print(fqdn, (
                FAIL, 'does not resolve (no DNS record)',
                f'DNS > add CNAME {host} -> <tunnel-id>.cfargotunnel.com, proxied '
                f'(re-saving the tunnel\'s public hostname usually creates it)')))
            continue
        try:
            verdict = classify_edge(*_public_get(f'https://{fqdn}/'))
        except Exception as exc:  # noqa: BLE001
            verdict = (FAIL, f'{type(exc).__name__}: {exc}', None)
        statuses.append(_print(fqdn, verdict))

    print('5. site connections seen by dashboard_node (last 24 h, from ~/.ros/log)')
    lines = []
    for path in glob.glob(os.path.expanduser('~/.ros/log/**/*.log'), recursive=True):
        if os.path.getmtime(path) > time.time() - 86400:
            with open(path, errors='replace') as handle:
                lines.extend(handle)
    events = recent_remote_events(lines, time.time() - 86400)
    if not events:
        print('  WARN  none -- nothing from the site has reached the dashboard. '
              'The fault is upstream of the car (sections 3-4, or the site\'s /rb2/check).')
    for stamp, text in events[-12:]:
        print(f'        {time.strftime("%H:%M:%S", time.localtime(stamp))}  {text[:150]}')

    failed = statuses.count(FAIL)
    print(f'\n{failed} problem(s) found.' if failed else '\nEverything this car can check is fine; '
          'open https://dashboard.sfuracerbot.ca/rb2/check for the Worker\'s side.')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
