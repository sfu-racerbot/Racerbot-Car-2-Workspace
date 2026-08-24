"""
Runs test/browser/measure_test.js as part of the normal pytest run.

That suite checks the arithmetic behind the map measuring tool: segment
lengths and totals, the round-then-choose unit boundary in
formatDistance(), tap-versus-drag discrimination, and the label placement
invariants (never upside down, always perpendicular, always on the same
side of the line).

measure.js is pure -- no DOM, no canvas, no WebSocket -- so the node test
loads the real file with nothing stubbed and calls it for real. That is the
point of splitting it out of dashboard.js: the maths a wrong measurement
would come from is reachable without a browser.

Skipped, not failed, where node is unavailable:
`node src/web_dashboard/test/browser/measure_test.js` is always the direct
way to run it.
"""
import os
import shutil
import subprocess

import pytest

_HERE = os.path.dirname(__file__)
_TEST_JS = os.path.join(_HERE, 'browser', 'measure_test.js')
_NODE = shutil.which('node') or shutil.which('nodejs')


@pytest.mark.skipif(_NODE is None, reason='node is not installed')
def test_browser_measure_geometry():
    result = subprocess.run(
        [_NODE, _TEST_JS],
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, (
        f'measure.js geometry tests failed (exit {result.returncode})\n'
        f'--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}'
    )
    # A suite that silently collected nothing would otherwise pass here.
    assert 'checks passed' in result.stdout, result.stdout
