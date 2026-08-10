"""
Consistency checks between the dashboard's HTML, CSS and JavaScript.

These exist because of how this UI actually breaks. `dashboard.js` looks up
about forty elements by id at load time and stores the results in consts;
if one id is renamed or dropped in the HTML, the lookup silently yields
null and the whole IIFE dies on the first property access -- so a single
typo in `index.html` produces a completely blank dashboard with one line in
a console nobody has open. That is not a failure mode worth discovering
trackside.

Nothing here needs a browser, ROS, or a running car:

    python3 -m pytest src/web_dashboard/test/test_web_assets.py -v
"""
import os
import re

import pytest

_WEB = os.path.join(os.path.dirname(__file__), '..', 'web')


def _read(name):
    with open(os.path.join(_WEB, name)) as handle:
        return handle.read()


def _ids_in(html):
    return re.findall(r'\bid="([^"]+)"', html)


def _ids_looked_up_by(js):
    return set(re.findall(r"getElementById\(\s*'([^']+)'\s*\)", js))


def _rule(css, selector):
    """The body of a TOP-LEVEL rule for `selector`.

    Anchored to the start of a line on purpose: the same selector also
    appears indented inside `@media` blocks, and matching one of those
    would test an override rather than the rule itself.
    """
    match = re.search(rf'^{re.escape(selector)}\s*\{{(.*?)\}}', css, re.M | re.S)
    assert match, f'no top-level rule for {selector}'
    return match.group(1)


# --------------------------------------------------------------------------
# Every element the JavaScript reaches for has to exist
# --------------------------------------------------------------------------

@pytest.mark.parametrize('script,page', [
    ('dashboard.js', 'index.html'),
    ('panels.js', 'index.html'),
    ('camera.js', 'camera.html'),
])
def test_every_element_the_script_looks_up_exists_in_its_page(script, page):
    html_ids = set(_ids_in(_read(page)))
    wanted = _ids_looked_up_by(_read(script))
    missing = sorted(wanted - html_ids)
    assert not missing, (
        f'{script} looks up ids that {page} does not define: {missing}. '
        f'Each one is a null at load time and a blank dashboard.')


@pytest.mark.parametrize('page', ['index.html', 'camera.html'])
def test_no_duplicate_ids(page):
    ids = _ids_in(_read(page))
    duplicates = sorted({i for i in ids if ids.count(i) > 1})
    assert not duplicates, f'{page} defines these ids more than once: {duplicates}'


def test_the_dashboard_still_loads_its_stylesheet_and_script():
    html = _read('index.html')
    assert 'href="style.css"' in html
    assert 'src="dashboard.js"' in html
    assert 'src="panels.js"' in html


def test_the_panel_manager_loads_after_the_dashboard():
    """panels.js MOVES section elements out of the sidebar. dashboard.js
    resolves ~forty elements by id at load time and keeps the references,
    which survive re-parenting -- but only if it ran first."""
    html = _read('index.html')
    assert html.index('src="dashboard.js"') < html.index('src="panels.js"')


# --------------------------------------------------------------------------
# Collapsible sections
# --------------------------------------------------------------------------

def test_every_section_is_a_details_element():
    """Collapsing is the browser's job here, not JavaScript's -- that is
    what gives keyboard and screen-reader behaviour for free."""
    html = _read('index.html')
    for match in re.finditer(r'<(\w+)([^>]*\bdata-section="[^"]+")', html):
        assert match.group(1) == 'details', (
            f'a section is a <{match.group(1)}>, not <details>: {match.group(2)}')


def test_every_section_has_a_digest_element_for_its_collapsed_headline():
    html = _read('index.html')
    sections = set(re.findall(r'data-section="([^"]+)"', html))
    ids = set(_ids_in(html))
    for name in sections:
        assert f'digest-{name}' in ids, (
            f"section '{name}' has no digest-{name} element, so collapsing it "
            f'would hide its value entirely')


def test_the_javascript_knows_about_exactly_the_sections_that_exist():
    html_sections = set(re.findall(r'data-section="([^"]+)"', _read('index.html')))
    js = _read('dashboard.js')
    block = js[js.index('const digestEls = {'):]
    block = block[:block.index('};')]
    js_sections = set(re.findall(r"(\w+):\s*document\.getElementById", block))
    assert js_sections == html_sections, (
        f'digestEls covers {sorted(js_sections)} but the page has '
        f'{sorted(html_sections)}')


def test_a_checkbox_never_sits_inside_a_summary():
    """Clicking a control inside <summary> also toggles the section, which
    is never what the person clicking it wanted."""
    html = _read('index.html')
    for summary in re.findall(r'<summary\b.*?</summary>', html, re.S):
        assert '<input' not in summary, f'interactive control inside a summary: {summary[:120]}'


# --------------------------------------------------------------------------
# The scroll bug this UI pass exists to fix
# --------------------------------------------------------------------------

def test_the_sidebar_can_receive_pointer_events():
    """The decision log had overflow-y:auto and was still impossible to
    scroll, because #overlay was pointer-events:none and the wheel went
    straight past it to the canvas, zooming the map instead."""
    overlay = _rule(_read('style.css'), '#overlay')
    assert 'pointer-events: none' not in overlay, (
        'the sidebar cannot receive wheel events, so nothing inside it can scroll')


def test_the_sidebar_is_bounded_and_scrolls():
    """Unbounded, its bottom (the decision log, the tuning button, reset
    view) simply ran off a short screen with no way to reach it, because
    html/body are overflow:hidden."""
    css = _read('style.css')
    assert 'max-height' in _rule(css, '#overlay'), '#overlay has no height bound'

    panels = _rule(css, '#panels')
    assert 'overflow-y: auto' in panels, '#panels does not scroll'
    # A flex child will not shrink below its content without this, so the
    # region would grow instead of scrolling.
    assert 'min-height: 0' in panels, '#panels will overflow rather than scroll'


def test_scrollable_regions_are_visibly_scrollable():
    css = _read('style.css')
    assert 'scrollbar-color' in css, 'no scrollbar styling: dark-theme scrollbars are invisible'
    assert '::-webkit-scrollbar' in css


def test_the_decision_log_scrolls():
    block = _rule(_read('style.css'), '.intent-log')
    assert 'overflow-y: auto' in block
    assert 'max-height' in block
    html = _read('index.html')
    # `scrolls` is what carries the visible scrollbar styling.
    assert re.search(r'id="intent-log"[^>]*class="[^"]*scrolls', html), (
        'the decision log is not marked as a scroll region')


def test_the_view_controls_are_pinned_outside_the_scroll_region():
    """The banner explaining why the picture looks odd, and the reset-view
    button, are the two things that must never be what scrolled away."""
    html = _read('index.html')
    # Anchor on the last </details>: every section lives inside #panels, so
    # anything after that is outside the scroll region.
    last_section_end = html.rindex('</details>')
    assert html.index('id="mode-banner"') > last_section_end, (
        'the mode banner is inside the scroll region and can scroll away')
    assert html.index('id="help"') > last_section_end, (
        'the view controls are inside the scroll region and can scroll away')


def test_the_page_is_well_formed():
    """A stray unclosed tag reflows the whole sidebar in ways that are
    tedious to spot by eye."""
    from html.parser import HTMLParser

    void = {'area', 'base', 'br', 'col', 'embed', 'hr', 'img', 'input',
            'link', 'meta', 'source', 'track', 'wbr'}

    class Checker(HTMLParser):
        def __init__(self):
            super().__init__()
            self.stack = []
            self.problems = []

        def handle_startendtag(self, tag, attrs):
            # `<meta ... />` and friends: already complete, nothing to nest.
            pass

        def handle_starttag(self, tag, attrs):
            if tag not in void:
                self.stack.append(tag)

        def handle_endtag(self, tag):
            if tag in void:
                return
            if not self.stack:
                self.problems.append(f'</{tag}> with nothing open')
            elif self.stack[-1] != tag:
                self.problems.append(f'</{tag}> closes <{self.stack[-1]}>')
                self.stack.pop()
            else:
                self.stack.pop()

    for page in ('index.html', 'camera.html'):
        checker = Checker()
        checker.feed(_read(page))
        assert not checker.problems, f'{page}: {checker.problems}'
        assert not checker.stack, f'{page} leaves these tags unclosed: {checker.stack}'


def test_the_stylesheet_has_balanced_braces():
    css = _read('style.css')
    assert css.count('{') == css.count('}'), 'unbalanced braces in style.css'
