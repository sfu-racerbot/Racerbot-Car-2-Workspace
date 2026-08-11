/*
 * Geometry checks for web/panels.js -- the dashboard's window manager.
 *
 * panels.js splits deliberately in two: pure box arithmetic (snapping,
 * clamping, resizing, the saved-layout round trip), and the DOM wiring
 * that uses it. Only the first half is checked here, and that is the
 * half worth checking -- a snap that picks the wrong edge or a resize
 * that inverts a box is a bug you would otherwise only find by dragging
 * panels around by hand and noticing something felt off.
 *
 * The file bails out before touching the DOM when `document` is absent,
 * so it loads under plain node with nothing stubbed.
 *
 * Run directly with `node panels_test.js`, or through pytest via
 * test_panels_js.py.
 */
'use strict';

const path = require('path');

const panels = require(path.join(__dirname, '..', '..', 'web', 'panels.js'));

let checks = 0;
let failures = 0;

function check(name, condition, detail) {
  checks++;
  if (condition) {
    console.log(`  ok    ${name}`);
  } else {
    failures++;
    console.log(`  FAIL  ${name}${detail ? `\n        ${detail}` : ''}`);
  }
}

function eq(name, actual, expected) {
  check(name, actual === expected, `expected ${expected}, got ${actual}`);
}

function section(title) {
  console.log(title);
}

// ---------------------------------------------------------------------------
section('snapAxis: adjacency and alignment');

// Flush after: my start lands on your end.
eq('snaps my start to your end', panels.snapAxis([104, 204], [[0, 100]]), -4);
// Flush before: my end lands on your start.
eq('snaps my end to your start', panels.snapAxis([-97, 3], [[0, 100]]), -3);
// Aligned: my start lands on your start.
eq('snaps my start to your start', panels.snapAxis([5, 105], [[0, 100]]), -5);
// Aligned: my end lands on your end. Deliberately a SHORT box, so its
// own start is nowhere near yours -- otherwise the start-to-start
// candidate is nearer and (correctly) wins instead.
eq('snaps my end to your end', panels.snapAxis([40, 94], [[0, 100]]), 6);

eq('leaves a box alone beyond the threshold', panels.snapAxis([140, 240], [[0, 100]]), 0);

check('picks the nearest of several candidates',
  panels.snapAxis([103, 203], [[0, 100], [0, 260]]) === -3,
  'a 3px adjacency should beat a 57px alignment');

eq('no candidates means no movement', panels.snapAxis([10, 50], []), 0);

// ---------------------------------------------------------------------------
section('snapBox: viewport edges count as snap targets');

{
  const box = { x: 6, y: 300, w: 200, h: 100 };
  const snapped = panels.snapBox(box, [], 1600, 900);
  eq('snaps to the left screen edge', snapped.x, 0);
  eq('leaves an axis with nothing near it alone', snapped.y, 300);
}

{
  const box = { x: 1391, y: 10, w: 200, h: 100 };
  const snapped = panels.snapBox(box, [], 1600, 900);
  eq('snaps to the right screen edge', snapped.x + snapped.w, 1600);
}

{
  // Sitting just past another panel's right edge -> flush against it.
  const other = { x: 12, y: 12, w: 320, h: 700 };
  const box = { x: 340, y: 20, w: 280, h: 200 };
  const snapped = panels.snapBox(box, [other], 1600, 900);
  eq('snaps flush against a neighbour', snapped.x, 332);
  eq('and aligns its top with the neighbour', snapped.y, 12);
}

check('snapping never changes a box\'s size', (() => {
  const box = { x: 6, y: 6, w: 234, h: 111 };
  const s = panels.snapBox(box, [{ x: 300, y: 300, w: 100, h: 100 }], 1600, 900);
  return s.w === box.w && s.h === box.h;
})());

// ---------------------------------------------------------------------------
section('resizeBox: the opposite side stays put');

{
  const box = { x: 100, y: 100, w: 200, h: 200 };

  const east = panels.resizeBox(box, 'e', 50, 0);
  check('dragging east widens without moving x', east.x === 100 && east.w === 250);

  const west = panels.resizeBox(box, 'w', -50, 0);
  check('dragging west moves x and keeps the right edge',
    west.x === 50 && west.w === 250 && west.x + west.w === box.x + box.w,
    `got x=${west.x} w=${west.w}`);

  const north = panels.resizeBox(box, 'n', 0, -30);
  check('dragging north keeps the bottom edge',
    north.y === 70 && north.y + north.h === box.y + box.h);

  const corner = panels.resizeBox(box, 'se', 40, 60);
  check('a corner resizes both axes', corner.w === 240 && corner.h === 260);
}

{
  // Dragging an edge straight through the opposite one must not invert.
  const box = { x: 100, y: 100, w: 200, h: 200 };
  const crushed = panels.resizeBox(box, 'w', 1000, 0);
  check('a box cannot be turned inside out',
    crushed.w === panels.MIN_W && crushed.w > 0,
    `width came out ${crushed.w}`);
  check('and its right edge is still where it was',
    crushed.x + crushed.w === box.x + box.w);
}

// ---------------------------------------------------------------------------
section('clampToViewport: a panel can always be grabbed again');

{
  const off = panels.clampToViewport({ x: 5000, y: 5000, w: 300, h: 200 }, 1600, 900);
  check('a panel dragged off-screen stays reachable',
    off.x <= 1600 - 48 && off.y <= 900,
    `got x=${off.x} y=${off.y}`);

  const negative = panels.clampToViewport({ x: -5000, y: -50, w: 300, h: 200 }, 1600, 900);
  check('and so does one dragged off the top-left',
    negative.x + 300 >= 48 && negative.y >= 0,
    `got x=${negative.x} y=${negative.y}`);

  const huge = panels.clampToViewport({ x: 0, y: 0, w: 9000, h: 9000 }, 1600, 900);
  check('a panel is never larger than the window',
    huge.w <= 1600 && huge.h <= 900);
}

// ---------------------------------------------------------------------------
section('layout state survives a save/load round trip');

{
  const state = {
    sections: {
      intent: { floating: true, box: { x: 10, y: 20, w: 300, h: 240 } },
      vehicle: { floating: false, box: null },
    },
    panels: { overlay: { x: 12, y: 12, w: 320, h: 700 } },
  };
  const round = panels.deserializeLayout(
    JSON.parse(JSON.stringify(panels.serializeLayout(state))));

  check('a floating section comes back floating, in the same place',
    round.sections.intent.floating === true
    && round.sections.intent.box.x === 10
    && round.sections.intent.box.w === 300);
  check('a docked section comes back docked',
    round.sections.vehicle.floating === false);
  check('fixed panel geometry survives too',
    round.panels.overlay.h === 700);
}

{
  // Whatever is in localStorage is untrusted: an older build, a newer
  // one, or something half-written. It must never throw on the way in.
  const junk = [null, undefined, 42, 'nonsense', [], {}, { sections: 5 },
    { sections: { a: null } }, { panels: { b: { x: 'no' } } },
    { sections: { a: { floating: true, box: { x: NaN, y: 0, w: 1, h: 1 } } } }];
  let threw = null;
  for (const value of junk) {
    try {
      panels.deserializeLayout(value);
    } catch (err) {
      threw = `${JSON.stringify(value)}: ${err.message}`;
    }
  }
  check('malformed saved layouts are ignored, not thrown on', threw === null, threw);

  const nan = panels.deserializeLayout(
    { sections: { a: { floating: true, box: { x: NaN, y: 0, w: 1, h: 1 } } } });
  check('a box with a NaN in it is dropped rather than restored',
    nan.sections.a.box === null);
}

// ---------------------------------------------------------------------------
section('the phone sheet: where a drag settles');

{
  // A 720px-tall sheet on a phone, closed to a 140px peek.
  const H = 720;
  const PEEK = 140;
  const offsets = panels.sheetOffsets(H, PEEK);

  eq('fully open is no offset at all', offsets.full, 0);
  eq('half sits at the fraction the stylesheet animates to',
    offsets.half, H * panels.SHEET_HALF_FRACTION);
  eq('closed leaves exactly the peek showing', offsets.peek, H - PEEK);

  // --- released slowly: nearest detent wins ---
  eq('a slow release near the top opens fully',
    panels.detentFor(20, H, PEEK, 0), 'full');
  eq('a slow release near the middle settles at half',
    panels.detentFor(offsets.half + 15, H, PEEK, 0), 'half');
  eq('a slow release near the bottom closes',
    panels.detentFor(offsets.peek - 20, H, PEEK, 0), 'peek');

  // --- flicked: direction beats distance ---
  // The gesture this protects: a short sharp flick up from the closed
  // sheet. It travels almost no distance, so "nearest detent" would snap
  // it straight back to peek and the flick would look like it failed.
  eq('a flick up from closed opens a detent',
    panels.detentFor(offsets.peek - 5, H, PEEK, -1.2), 'half');
  eq('a flick down from open closes a detent',
    panels.detentFor(5, H, PEEK, 1.2), 'half');
  eq('a flick up from half opens fully',
    panels.detentFor(offsets.half, H, PEEK, -1.2), 'full');
  eq('a flick down from half closes',
    panels.detentFor(offsets.half, H, PEEK, 1.2), 'peek');

  // A flick cannot walk off either end of the list.
  eq('a flick up from fully open stays fully open',
    panels.detentFor(0, H, PEEK, -3), 'full');
  eq('a flick down from closed stays closed',
    panels.detentFor(offsets.peek, H, PEEK, 3), 'peek');

  // Just under the flick threshold is still a drag, not a throw.
  eq('a firm but sub-threshold drag still lands on the nearest detent',
    panels.detentFor(offsets.peek - 5, H, PEEK, -(panels.SHEET_FLICK_PX_PER_MS - 0.01)),
    'peek');
}

{
  // A short phone in landscape: the peek is a big share of a small sheet,
  // so `peek` (h - peek) can end up ABOVE `half` (0.46h) on the axis. The
  // detents are sorted by offset rather than assumed to be in a fixed
  // order, so a flick still steps the way the finger went.
  const H = 300;
  const PEEK = 200;
  const offsets = panels.sheetOffsets(H, PEEK);
  check('the detents really have crossed over',
    offsets.peek < offsets.half,
    `peek=${offsets.peek} half=${offsets.half}`);

  eq('a flick down from fully open still moves toward closed',
    panels.detentFor(0, H, PEEK, 2), 'peek');
  eq('a flick up from the bottom detent still opens',
    panels.detentFor(offsets.half, H, PEEK, -2), 'peek');
}

{
  // Peeks taller than the sheet would put the closed detent at a negative
  // offset -- i.e. above the top of the screen.
  eq('a peek taller than the sheet cannot lift it off the top',
    panels.sheetOffsets(100, 400).peek, 0);
}

// ---------------------------------------------------------------------------
console.log('');
if (failures) {
  console.log(`${failures} of ${checks} checks FAILED`);
  process.exit(1);
}
console.log(`All ${checks} checks passed.`);
