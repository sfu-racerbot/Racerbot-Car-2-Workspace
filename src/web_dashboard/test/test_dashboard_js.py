"""
Runs the browser-side test suite (test/browser/map_decode_test.js) as part
of the normal pytest run, so `python3 -m pytest src/web_dashboard/test/`
covers both halves of the dashboard rather than just the Python one.

That JS test loads the real `web/dashboard.js` into a stubbed DOM and
checks the one thing about the map that cannot be eyeballed: that applying
a keyframe plus patches paints exactly the same pixels as a single
keyframe of the final grid. A patch dropped at the wrong offset, or
without the bottom-up-to-top-down row flip, still produces something
map-shaped -- it would just quietly put a wall in the wrong place.

Skipped, not failed, where node is unavailable: this is a convenience
wrapper, and `node src/web_dashboard/test/browser/map_decode_test.js` is
always the direct way to run it.
"""
import os
import shutil
import subprocess

import pytest

_HERE = os.path.dirname(__file__)
_TEST_JS = os.path.join(_HERE, 'browser', 'map_decode_test.js')
_NODE = shutil.which('node') or shutil.which('nodejs')


@pytest.mark.skipif(_NODE is None, reason='node is not installed')
def test_browser_map_decoding():
    result = subprocess.run(
        [_NODE, _TEST_JS],
        capture_output=True, text=True, timeout=120,
    )
    # The JS harness prints one line per check and exits non-zero on any
    # failure; surface its whole output so a failure here is readable
    # without re-running it by hand.
    assert result.returncode == 0, (
        f'browser tests failed (exit {result.returncode})\n'
        f'--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}'
    )
    assert 'checks passed' in result.stdout, result.stdout
