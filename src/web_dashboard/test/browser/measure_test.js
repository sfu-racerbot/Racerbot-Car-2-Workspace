/*
 * Geometry checks for web/measure.js -- the map measuring tool's maths.
 *
 * measure.js is entirely pure (no DOM, no canvas, no WebSocket), so this
 * loads the real file under plain node with nothing stubbed and calls it
 * for real. Nothing here is mocked: mocking our own ROS-free module would
 * test the mock.
 *
 * Oracles used below:
 *   closed form  -- distances recomputed here from the coordinates
 *   invariant    -- properties that must hold for any input (the total is
 *                   the sum of the segments; reversing a chain cannot
 *                   change its length; a label offset is perpendicular)
 *   boundary     -- exactly at a threshold, and one step past it
 *
 * Run directly with `node measure_test.js`, or through pytest via
 * test_measure_js.py.
 */
'use strict';

const path = require('path');

const measure = require(path.join(__dirname, '..', '..', 'web', 'measure.js'));

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

function near(name, actual, expected, tolerance, unit) {
  check(name, Math.abs(actual - expected) <= tolerance,
    `expected ${expected} +/- ${tolerance} ${unit || ''}, got ${actual}`);
}

function section(title) {
  console.log(title);
}

const P = (x, y, frame) => ({ x, y, frame: frame || 'map' });

// ---------------------------------------------------------------------------
section('segments / total: closed form');

// Legs of a 3-4-5 right triangle, walked as a two-segment chain. The 5 is
// the straight line from the first point to the last, which is deliberately
// NOT what a chained measurement reports -- that distinction is the whole
// point of the tool, so it is asserted both ways just below.
const LEGS = [P(0, 0), P(3, 0), P(3, 4)];
eq('two segments from three points', measure.segments(LEGS).length, 2);
near('first leg is 3 m', measure.segments(LEGS)[0].length, 3, 1e-12, 'm');
near('second leg is 4 m', measure.segments(LEGS)[1].length, 4, 1e-12, 'm');
near('the chain totals 3 + 4', measure.total(LEGS), 7, 1e-12, 'm');
near('the straight line across the same two ends is 5, not 7',
  measure.total([P(0, 0), P(3, 4)]), 5, 1e-12, 'm');

// Three sides of a unit square: exactly 3, with no floating-point slack.
const SQUARE = [P(0, 0), P(1, 0), P(1, 1), P(0, 1)];
eq('three sides of a unit square total exactly 3', measure.total(SQUARE), 3);

near('a negative-going segment measures the same as a positive one',
  measure.total([P(0, 0), P(-2.5, 0)]), 2.5, 1e-12, 'm');
near('a diagonal uses both axes',
  measure.total([P(1, 1), P(2, 2)]), Math.SQRT2, 1e-12, 'm');

// ---------------------------------------------------------------------------
section('segments / total: empty, single and degenerate input');

eq('no points give no segments', measure.segments([]).length, 0);
eq('no points total zero', measure.total([]), 0);
eq('one point gives no segments', measure.segments([P(1, 2)]).length, 0);
eq('one point totals zero', measure.total([P(1, 2)]), 0);
eq('an undefined chain gives no segments', measure.segments(undefined).length, 0);
eq('two identical points give one zero-length segment',
  measure.segments([P(1, 1), P(1, 1)]).length, 1);
eq('two identical points total zero', measure.total([P(1, 1), P(1, 1)]), 0);

// ---------------------------------------------------------------------------
section('total: invariants');

check('the total is the sum of the segments',
  Math.abs(measure.total(SQUARE)
    - measure.segments(SQUARE).reduce((a, s) => a + s.length, 0)) < 1e-12);

check('reversing a chain cannot change its length',
  Math.abs(measure.total(SQUARE) - measure.total(SQUARE.slice().reverse())) < 1e-12,
  'an off-by-one in the segment loop shows up here and nowhere else');

check('adding a point never shortens the chain',
  measure.total(SQUARE.concat([P(5, 5)])) > measure.total(SQUARE));

check('the total is never NaN for finite input',
  Number.isFinite(measure.total([P(0, 0), P(1e6, -1e6)])));

// ---------------------------------------------------------------------------
section('formatDistance: the round-then-choose boundary');

// The naive form -- pick the unit from the raw value, then round inside it
// -- prints "100 cm" here. Rounding to centimetres first and comparing
// against 100 makes both branches agree at the boundary.
eq('0.999 m rounds up into metres', measure.formatDistance(0.999), '1.00 m');
eq('0.995 m rounds up into metres', measure.formatDistance(0.995), '1.00 m');
eq('0.994 m stays in centimetres', measure.formatDistance(0.994), '99 cm');
eq('exactly 1 m is metres', measure.formatDistance(1), '1.00 m');
eq('zero is centimetres, not a blank', measure.formatDistance(0), '0 cm');
eq('a sub-centimetre value rounds to zero', measure.formatDistance(0.004), '0 cm');
eq('a typical track width', measure.formatDistance(3.4159), '3.42 m');
eq('a large distance keeps two decimals', measure.formatDistance(123.456), '123.46 m');

section('formatDistance: values that must never reach the HUD');
eq('NaN is not a distance', measure.formatDistance(NaN), '--');
eq('Infinity is not a distance', measure.formatDistance(Infinity), '--');
eq('-Infinity is not a distance', measure.formatDistance(-Infinity), '--');
eq('a negative distance is refused', measure.formatDistance(-1), '--');
eq('a missing value is refused', measure.formatDistance(undefined), '--');
eq('a string is refused', measure.formatDistance('3'), '--');

// ---------------------------------------------------------------------------
section('isTap: boundaries');

const tap = (over) => Object.assign(
  { maxTravelPx: 0, durationMs: 100, pointerCount: 1, pointerType: 'mouse' }, over);

eq('a still, brief press is a tap', measure.isTap(tap({})), true);
eq('exactly at the mouse slop is still a tap',
  measure.isTap(tap({ maxTravelPx: measure.TAP_SLOP_MOUSE_PX })), true);
eq('one pixel past the mouse slop is a drag',
  measure.isTap(tap({ maxTravelPx: measure.TAP_SLOP_MOUSE_PX + 1 })), false);
eq('a finger gets the larger slop',
  measure.isTap(tap({ maxTravelPx: measure.TAP_SLOP_MOUSE_PX + 1,
                      pointerType: 'touch' })), true);
eq('one pixel past the touch slop is a drag',
  measure.isTap(tap({ maxTravelPx: measure.TAP_SLOP_TOUCH_PX + 1,
                      pointerType: 'touch' })), false);
eq('exactly at the time limit is still a tap',
  measure.isTap(tap({ durationMs: measure.TAP_MAX_MS })), true);
eq('one millisecond past the limit is a hold',
  measure.isTap(tap({ durationMs: measure.TAP_MAX_MS + 1 })), false);

section('isTap: the cases a naive implementation gets wrong');
eq('a drag out and back is NOT a tap, even ending where it started',
  measure.isTap(tap({ maxTravelPx: 400 })), false);
eq('one finger of a pinch is never a tap',
  measure.isTap(tap({ pointerCount: 2 })), false);
eq('a gesture with no pointers is not a tap',
  measure.isTap(tap({ pointerCount: 0 })), false);
eq('a non-finite travel is not a tap',
  measure.isTap(tap({ maxTravelPx: NaN })), false);
eq('a non-finite duration is not a tap',
  measure.isTap(tap({ durationMs: Infinity })), false);
eq('no gesture at all is not a tap', measure.isTap(null), false);

// ---------------------------------------------------------------------------
section('addPoint / tooClose / undo');

eq('a point is added', measure.addPoint([], P(1, 1), 0).length, 1);
eq('the chain is not mutated in place',
  (() => { const before = [P(0, 0)]; measure.addPoint(before, P(1, 1), 0); return before.length; })(), 1);
eq('a NaN coordinate is refused', measure.addPoint([], { x: NaN, y: 0 }, 0).length, 0);
eq('an Infinity coordinate is refused',
  measure.addPoint([], { x: 0, y: Infinity }, 0).length, 0);
eq('a missing point is refused', measure.addPoint([], null, 0).length, 0);
eq('the frame tag is carried onto the point',
  measure.addPoint([], P(1, 1, 'body'), 0)[0].frame, 'body');

eq('a second point on top of the first is refused',
  measure.addPoint([P(0, 0)], P(0, 0), 0.05).length, 1);
eq('exactly at the separation limit is still too close',
  measure.addPoint([P(0, 0)], P(0.05, 0), 0.05).length, 1);
eq('just past the separation limit is accepted',
  measure.addPoint([P(0, 0)], P(0.0501, 0), 0.05).length, 2);
// A zero-length segment is never wanted, so an exact repeat is refused
// even when no separation is asked for -- `tooClose` compares with <=, and
// a distance of 0 is not greater than 0.
eq('an exact repeat is refused even with no separation required',
  measure.addPoint([P(0, 0)], P(0, 0), 0).length, 1);
eq('a distinct point is accepted with no separation required',
  measure.addPoint([P(0, 0)], P(0.001, 0), 0).length, 2);

eq('undo removes the last point', measure.undo([P(0, 0), P(1, 1)]).length, 1);
eq('undo on a single point empties the chain', measure.undo([P(0, 0)]).length, 0);
eq('undo on an empty chain is a no-op, not a throw', measure.undo([]).length, 0);

// ---------------------------------------------------------------------------
section('frameFor');

eq('nothing measured yet', measure.frameFor([], true), 'idle');
eq('an undefined chain is idle', measure.frameFor(undefined, false), 'idle');
eq('a map chain with a pose is live', measure.frameFor([P(0, 0, 'map')], true), 'map');
eq('a body chain with no pose is live', measure.frameFor([P(0, 0, 'body')], false), 'body');
eq('a body chain once a pose arrives is stale',
  measure.frameFor([P(0, 0, 'body')], true), 'stale');
eq('a map chain that lost its pose is stale',
  measure.frameFor([P(0, 0, 'map')], false), 'stale');

// ---------------------------------------------------------------------------
section('labelPlacement');

// A horizontal segment 100px long, left to right.
const flat = measure.labelPlacement({ ax: 0, ay: 0, bx: 100, by: 0 });
near('the label sits at the midpoint (x)', flat.x, 50, 1e-12, 'px');
near('the label sits at the midpoint (y)', flat.y, 0, 1e-12, 'px');
near('a left-to-right segment needs no rotation', flat.angle, 0, 1e-12, 'rad');
check('the label is offset above the line (canvas +Y is down)', flat.offsetY < 0);

check('a segment shorter than the floor gets no label',
  measure.labelPlacement({ ax: 0, ay: 0, bx: measure.MIN_LABEL_PX - 1, by: 0 }) === null);
check('a segment exactly at the floor does get one',
  measure.labelPlacement({ ax: 0, ay: 0, bx: measure.MIN_LABEL_PX, by: 0 }) !== null);
check('a zero-length segment gets no label',
  measure.labelPlacement({ ax: 5, ay: 5, bx: 5, by: 5 }) === null);
check('a non-finite segment gets no label',
  measure.labelPlacement({ ax: 0, ay: 0, bx: NaN, by: 0 }) === null);
check('no segment at all gets no label', measure.labelPlacement(null) === null);

section('labelPlacement: invariants over every direction');

let uprightOk = true;
let perpendicularOk = true;
let offsetLengthOk = true;
let aboveOk = true;
for (let deg = 0; deg < 360; deg += 15) {
  const rad = (deg * Math.PI) / 180;
  const bx = 200 * Math.cos(rad);
  const by = 200 * Math.sin(rad);
  const plan = measure.labelPlacement({ ax: 0, ay: 0, bx, by });
  if (!plan) { uprightOk = false; break; }
  // Text is never upside down.
  if (Math.abs(plan.angle) > Math.PI / 2 + 1e-12) uprightOk = false;
  // The offset is perpendicular to the segment: their dot product is zero.
  const dot = plan.offsetX * bx + plan.offsetY * by;
  if (Math.abs(dot) > 1e-9) perpendicularOk = false;
  // And it is exactly one offset away, never scaled by the segment length.
  const len = Math.hypot(plan.offsetX, plan.offsetY);
  if (Math.abs(len - measure.LABEL_OFFSET_PX) > 1e-9) offsetLengthOk = false;
  // Always on the same side of the screen, so a chain's labels do not
  // flip from one side to the other segment by segment.
  if (plan.offsetY > 1e-12) aboveOk = false;
}
check('text is never upside down, at any segment angle', uprightOk);
check('the label offset is perpendicular to its segment', perpendicularOk,
  'a sign error here still passes a magnitude-only check');
check('the label offset is one fixed distance, not scaled by length', offsetLengthOk);
check('every label sits on the same side of its segment', aboveOk);

// ---------------------------------------------------------------------------
console.log('');
if (failures) {
  console.log(`${failures} of ${checks} checks FAILED`);
  process.exit(1);
}
console.log(`${checks} checks passed`);
