"""
Does the package actually install the files the page asks for?

This exists because of a real failure that nothing else here could catch.
`web/panels.js` was added to `index.html` but not to `setup.py`'s
`data_files`, which listed every web asset by hand. The result installs a
dashboard whose HTML requests a script that was never copied into
`share/`: the browser gets a 404, and every feature in that file silently
does not exist. The page still renders, the console has one line nobody
has open, and the only symptom is "the thing you told me about isn't
there".

Every other test in this directory reads the files in `web/` directly
from the source tree, so all of them passed the whole time.

`setup.py` is exec'd with a stubbed `setup()` that captures its keyword
arguments -- so this checks the real declaration, whatever form it takes,
rather than a copy of it that could drift.

    python3 -m pytest src/web_dashboard/test/test_packaging.py -v
"""
import os
import re

import pytest

_PKG = os.path.join(os.path.dirname(__file__), '..')
_WEB = os.path.join(_PKG, 'web')


def _captured_setup_kwargs():
    """Run setup.py with setuptools.setup() stubbed out, and return the
    kwargs it would have passed."""
    import setuptools

    captured = {}

    def fake_setup(**kwargs):
        captured.update(kwargs)

    real_setup = setuptools.setup
    cwd = os.getcwd()
    setuptools.setup = fake_setup
    try:
        # setup.py globs relative paths, so it has to run from the package
        # directory to see them -- exactly as colcon runs it.
        os.chdir(os.path.abspath(_PKG))
        with open('setup.py') as handle:
            source = handle.read()
        exec(compile(source, 'setup.py', 'exec'), {'__name__': '__main__'})
    finally:
        setuptools.setup = real_setup
        os.chdir(cwd)
    return captured


def _installed_web_files():
    """Basenames of everything setup.py installs into share/<pkg>/web."""
    for destination, sources in _captured_setup_kwargs()['data_files']:
        if destination.endswith('/web'):
            return {os.path.basename(path) for path in sources}
    raise AssertionError('setup.py installs nothing into share/<pkg>/web')


def test_every_web_asset_on_disk_is_installed():
    """The blanket rule: if it is in web/, it ships."""
    on_disk = {
        name for name in os.listdir(_WEB)
        if name.endswith(('.html', '.js', '.css'))
    }
    missing = sorted(on_disk - _installed_web_files())
    assert not missing, (
        f'these files exist in web/ but setup.py does not install them: '
        f'{missing}. The browser will 404 them and everything they do will '
        f'silently not happen.')


@pytest.mark.parametrize('page', ['index.html', 'camera.html'])
def test_every_script_and_stylesheet_a_page_requests_is_installed(page):
    """The tighter rule, and the one that actually broke: whatever the
    HTML asks the browser to fetch has to be a file that got installed."""
    with open(os.path.join(_WEB, page)) as handle:
        html = handle.read()

    referenced = set(re.findall(r'<script[^>]+src="([^"]+)"', html))
    referenced |= set(re.findall(r'<link[^>]+href="([^"]+)"', html))
    # Only local assets: an absolute URL is somebody else's to serve.
    referenced = {r for r in referenced if not re.match(r'^(https?:)?//', r)}

    installed = _installed_web_files()
    missing = sorted(r for r in referenced if os.path.basename(r) not in installed)
    assert not missing, (
        f'{page} asks the browser for {missing}, which setup.py does not '
        f'install into share/web_dashboard/web/')


def test_the_pages_themselves_are_installed():
    installed = _installed_web_files()
    for page in ('index.html', 'camera.html'):
        assert page in installed, f'{page} is not installed'
