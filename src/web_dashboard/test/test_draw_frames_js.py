"""
Runs test/browser/draw_frames_test.js as part of the normal pytest run.

That suite checks which coordinate frame each canvas overlay is allowed to
use for a given set of arrived data. It exists because two real bugs lived
in exactly that decision, and neither was visible to any other test here:

  * with a map up but no pose, the car was still drawn in the body frame
    -- at the middle of the view, facing a fixed "up" -- on top of a
    world-frame map, which reads as a real position and a real heading;

  * with a pose but no map, the car went to world coordinates while its
    own LIDAR stayed in the body frame, so the car floated away from the
    scan it had produced.

Both are wrong pictures rather than crashes, which is the kind of bug this
dashboard produces and the kind that is hardest to notice.

Skipped, not failed, where node is unavailable:
`node src/web_dashboard/test/browser/draw_frames_test.js` is always the
direct way to run it.
"""
import os
import shutil
import subprocess

import pytest

_HERE = os.path.dirname(__file__)
_TEST_JS = os.path.join(_HERE, 'browser', 'draw_frames_test.js')
_NODE = shutil.which('node') or shutil.which('nodejs')


@pytest.mark.skipif(_NODE is None, reason='node is not installed')
def test_browser_draw_frames():
    result = subprocess.run(
        [_NODE, _TEST_JS],
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, (
        f'draw-frame tests failed (exit {result.returncode})\n'
        f'--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}'
    )
    assert 'checks passed' in result.stdout, result.stdout
