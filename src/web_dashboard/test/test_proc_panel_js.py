"""
Runs test/browser/proc_panel_test.js as part of the normal pytest run.

That suite checks the one decision the stop panel makes on its own:
whether a given process gets a stop button at all. Protected entries --
ackermann_mux, joy_teleop, the VESC chain, the dashboard itself -- must
never render one, and an entry the page cannot identify must fail closed
rather than open.

The server refuses those pids independently: it re-scans /proc and
re-vets every pid before signalling, so a hand-rolled WebSocket client
gets nowhere. This is the second lock, and it earns its place because a
button that is offered and then silently refused is worse than no button.
Someone presses it, watches nothing happen, presses it again -- instead of
releasing LB, which is the thing that actually stops the car.

Skipped, not failed, where node is unavailable:
`node src/web_dashboard/test/browser/proc_panel_test.js` is always the
direct way to run it.
"""
import os
import shutil
import subprocess

import pytest

_HERE = os.path.dirname(__file__)
_TEST_JS = os.path.join(_HERE, 'browser', 'proc_panel_test.js')
_NODE = shutil.which('node') or shutil.which('nodejs')


@pytest.mark.skipif(_NODE is None, reason='node is not installed')
def test_browser_proc_panel():
    result = subprocess.run(
        [_NODE, _TEST_JS],
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, (
        f'stop-panel tests failed (exit {result.returncode})\n'
        f'--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}'
    )
    assert 'checks passed' in result.stdout, result.stdout
