"""scripts/wiimote_connect against a fake bluetoothctl.

bluetoothctl is the process boundary, so it is the one thing faked: a small
script that keeps device state in a JSON file and logs every call. Runs
unsourced: python3 -m pytest src/racerbot_wiimote/racerbot_wiimote_driver/test/
"""

import json
import os
from pathlib import Path
import stat
import subprocess
import time

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'wiimote_connect'
WIIMOTE = '00:1F:32:AA:BB:CC'
HEADSET = '11:22:33:44:55:66'

FAKE_BLUETOOTHCTL = r'''#!/usr/bin/env python3
import json, os, sys
state_path = os.environ['FAKE_BT_STATE']
state = json.load(open(state_path))
args = [a for a in sys.argv[1:]]
if args[:1] == ['--timeout']:
    args = args[2:]
open(os.environ['FAKE_BT_LOG'], 'a').write(' '.join(args) + '\n')
cmd, rest = args[0], args[1:]
devs = state['devices']
def save():
    json.dump(state, open(state_path, 'w'))
if cmd == 'show':
    print('Controller 00:00:00:00:00:01 (public)')
    print('\tPowered: ' + ('yes' if state['powered'] else 'no'))
elif cmd == 'power':
    state['powered'] = True; save()
elif cmd == 'devices':
    for mac, d in devs.items():
        if d.get('visible', True) and (not rest or d.get(rest[0].lower())):
            print(f"Device {mac} {d['name']}")
elif cmd == 'scan':
    for d in devs.values():
        d['visible'] = True
    save()
elif cmd == 'info':
    d = devs.get(rest[0])
    if not d:
        sys.exit(1)
    for key in ('paired', 'trusted', 'connected'):
        print(f"\t{key.capitalize()}: {'yes' if d.get(key) else 'no'}")
elif cmd in ('pair', 'trust', 'connect'):
    d = devs[rest[0]]
    if cmd == 'connect' and not d.get('awake', True):
        sys.exit(1)
    d[{'pair': 'paired', 'trust': 'trusted', 'connect': 'connected'}[cmd]] = True
    save()
'''


def _write(path, text):
    path.write_text(text)
    path.chmod(path.stat().st_mode | stat.S_IEXEC)


@pytest.fixture
def bt(tmp_path):
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    _write(bin_dir / 'bluetoothctl', FAKE_BLUETOOTHCTL)
    _write(bin_dir / 'systemctl', '#!/bin/sh\nexit 0\n')
    _write(bin_dir / 'modprobe', '#!/bin/sh\nexit 0\n')
    _write(bin_dir / 'xwiishow', '#!/bin/sh\nexit 0\n')
    _write(bin_dir / 'sudo', '#!/bin/sh\nexit 1\n')
    state = tmp_path / 'state.json'
    log = tmp_path / 'calls.log'
    log.write_text('')

    class Bluetooth:
        def set(self, devices, powered=True):
            state.write_text(json.dumps({'devices': devices, 'powered': powered}))

        def devices(self):
            return json.loads(state.read_text())['devices']

        def calls(self):
            return log.read_text().splitlines()

        def run(self, *args, timeout=20):
            env = {**os.environ, 'PATH': f'{bin_dir}:{os.environ["PATH"]}',
                   'FAKE_BT_STATE': str(state), 'FAKE_BT_LOG': str(log),
                   'WIIMOTE_SCAN_SECONDS': '1', 'WIIMOTE_WATCH_PERIOD': '0.2'}
            return subprocess.run(['bash', str(SCRIPT), *args], env=env, capture_output=True,
                                  text=True, timeout=timeout)

        def popen(self, *args):
            env = {**os.environ, 'PATH': f'{bin_dir}:{os.environ["PATH"]}',
                   'FAKE_BT_STATE': str(state), 'FAKE_BT_LOG': str(log),
                   'WIIMOTE_WATCH_PERIOD': '0.2'}
            return subprocess.Popen(['bash', str(SCRIPT), *args], env=env,
                                    stdout=subprocess.PIPE, text=True)
    return Bluetooth()


def wiimote(**fields):
    return {'name': 'Nintendo RVL-CNT-01', **fields}


def test_paired_but_disconnected_wiimote_is_reconnected(bt):
    bt.set({WIIMOTE: wiimote(paired=True, trusted=True),
            HEADSET: {'name': 'Headset', 'paired': True}})
    result = bt.run()
    assert 'Connected to Wiimote 00:1F:32:AA:BB:CC.' in result.stdout
    assert bt.devices()[WIIMOTE]['connected'] is True
    assert f'connect {HEADSET}' not in bt.calls(), 'never touch other devices'
    assert not any(c.startswith('scan') for c in bt.calls()), 'no scan needed'


def test_already_connected_wiimote_is_left_alone(bt):
    bt.set({WIIMOTE: wiimote(paired=True, trusted=True, connected=True)})
    result = bt.run()
    assert 'is connected' in result.stdout
    assert f'connect {WIIMOTE}' not in bt.calls()


def test_new_wiimote_is_scanned_paired_trusted_and_connected(bt):
    bt.set({WIIMOTE: wiimote(visible=False)})
    result = bt.run()
    assert 'press the small red SYNC button' in result.stdout
    calls = bt.calls()
    order = [calls.index(f'{c} {WIIMOTE}') for c in ('pair', 'trust', 'connect')]
    assert order == sorted(order), calls
    assert any(c.startswith('scan on') for c in calls)
    device = bt.devices()[WIIMOTE]
    assert (device.get('paired'), device.get('trusted'), device.get('connected')) == (
        True, True, True)


def test_no_wiimote_found_fails_with_instructions(bt):
    bt.set({})
    result = bt.run()
    assert result.returncode == 1
    assert 'No Wiimote found. Press SYNC again' in result.stdout


def test_sleeping_known_wiimote_says_press_a_button(bt):
    bt.set({WIIMOTE: wiimote(paired=True, trusted=True, awake=False)})
    result = bt.run()
    assert 'press any button on it' in result.stdout


def test_check_reports_a_powered_off_adapter_and_turns_it_on(bt):
    bt.set({WIIMOTE: wiimote(paired=True)}, powered=False)
    result = bt.run('--check')
    assert 'turning it on' in result.stdout
    assert 'power on' in bt.calls()
    assert 'Wiimote 00:1F:32:AA:BB:CC: paired=yes trusted=no connected=no' in result.stdout


def test_watch_reconnects_a_trusted_wiimote_once_it_wakes(bt):
    bt.set({WIIMOTE: wiimote(paired=True, trusted=True, awake=False),
            HEADSET: {'name': 'Headset', 'paired': True, 'trusted': True}})
    watcher = bt.popen('--watch')
    try:
        time.sleep(0.8)
        assert bt.devices()[WIIMOTE].get('connected') is not True
        devices = bt.devices()
        devices[WIIMOTE]['awake'] = True      # a button was pressed
        bt.set(devices)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not bt.devices()[WIIMOTE].get('connected'):
            time.sleep(0.1)
        assert bt.devices()[WIIMOTE]['connected'] is True
        assert f'connect {HEADSET}' not in bt.calls()
    finally:
        watcher.kill()
        watcher.wait()


def test_unknown_option_is_an_error(bt):
    bt.set({})
    result = bt.run('--frobnicate')
    assert result.returncode == 2
    assert 'Unknown option --frobnicate' in result.stdout


def test_help_lists_every_mode(bt):
    text = bt.run('--help').stdout
    for mode in ('--check', '--watch', '--install-service'):
        assert mode in text


def test_watch_never_grabs_an_untrusted_wiimote(bt):
    # Another team's Wiimote, seen in a scan but never trusted by this car.
    other = '00:1F:32:00:00:01'
    bt.set({WIIMOTE: wiimote(paired=True, trusted=True, awake=True),
            other: wiimote(awake=True)})
    watcher = bt.popen('--watch')
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not bt.devices()[WIIMOTE].get('connected'):
            time.sleep(0.1)
        time.sleep(0.5)
        assert bt.devices()[WIIMOTE]['connected'] is True
        assert f'connect {other}' not in bt.calls()
    finally:
        watcher.kill()
        watcher.wait()
