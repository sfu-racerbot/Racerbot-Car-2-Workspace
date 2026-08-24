"""
Runs test/browser/map_panel_test.js as part of the normal pytest run.

That suite checks the two decisions the saved-map panel makes on its own:
whether a run gets a delete control at all, and whether the typed
confirmation arms it. A run the server marked undeletable must never
render a button, an unidentifiable row must fail closed, and the
confirmation must match exactly -- no trimming, no case folding.

The server refuses all of this independently: it re-scans the map roots and
re-vets the name, the contents and the digest before removing anything, so
a hand-rolled WebSocket client gets nowhere. This is the second lock, and
it earns its place because a delete button that is offered and then
silently refused teaches people the panel is broken -- and the next thing
they reach for is `rm -rf`.

Skipped, not failed, where node is unavailable:
`node src/web_dashboard/test/browser/map_panel_test.js` is always the
direct way to run it.
"""
import os
import shutil
import subprocess

import pytest

_HERE = os.path.dirname(__file__)
_TEST_JS = os.path.join(_HERE, 'browser', 'map_panel_test.js')
_NODE = shutil.which('node') or shutil.which('nodejs')


@pytest.mark.skipif(_NODE is None, reason='node is not installed')
def test_browser_map_panel():
    result = subprocess.run(
        [_NODE, _TEST_JS],
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, (
        f'map-panel tests failed (exit {result.returncode})\n'
        f'--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}'
    )
    assert 'checks passed' in result.stdout, result.stdout
