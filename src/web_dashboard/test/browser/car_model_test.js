/*
 * The car icon: is it the size and shape of the actual car, does the LIDAR
 * sit where the LIDAR sits, and does the scan get painted over it?
 *
 * Two things this exists to hold:
 *
 *   1. The overlay paint order. The car icon is drawn to scale, so at any
 *      useful zoom it covers the beams inside its own outline -- which are
 *      the ones reading a wall the car is about to touch. Painting the car
 *      after the scan hid exactly the points that matter.
 *
 *   2. The geometry. Every number in CAR_MODEL that the car was measured
 *      for is checked against a closed form or against the measurement it
 *      came from, never against a value recorded from the function itself:
 *
 *        wheelbase   0.36 m  measured 2026-08-24
 *        track       0.30 m  measured, outer edge of tire to outer edge
 *        LiDAR x     0.26 m  measured as 0.10 m behind the FRONT axle
 *
 *      and the Ackermann split is checked against the identity
 *      cot(outer) - cot(inner) = track / wheelbase, which holds for any
 *      correct pair of front-wheel angles and for no incorrect one.
 *
 * Run directly with `node car_model_test.js`, or through pytest via
 * test_car_model_js.py.
 */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const DASHBOARD_JS = path.join(__dirname, '..', '..', 'web', 'dashboard.js');

// dashboard.js is one big IIFE that wires up a whole page on load. It only
// has to reach the lines that publish the geometry, so give it a DOM stub
// that returns something for everything and throws for nothing.
function stubElement() {
  const el = {
    style: {}, dataset: {}, children: [], textContent: '', innerHTML: '',
    checked: false, value: '', disabled: false, width: 800, height: 600,
    classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
    addEventListener() {}, removeEventListener() {},
    appendChild(c) { this.children.push(c); return c; },
    setAttribute() {}, removeAttribute() {}, getAttribute() { return null; },
    querySelector() { return null; }, querySelectorAll() { return []; },
    getBoundingClientRect() { return { width: 100, height: 100, top: 0, left: 0, right: 100, bottom: 100 }; },
    focus() {}, blur() {},
    setPointerCapture() {}, releasePointerCapture() {}, hasPointerCapture() { return false; },
  };
  el.getContext = () => sharedContext(el);
  return el;
}

// One recording 2d context shared by every stub canvas, so the drawing path
// can be run for real and every argument that reaches it inspected.
const drawCalls = [];
let recordingContext = null;
function sharedContext(el) {
  if (recordingContext) return recordingContext;
  const base = {
    canvas: el,
    createImageData: (w, h) => ({ width: w, height: h, data: new Uint8ClampedArray(w * h * 4) }),
    putImageData() {}, measureText: () => ({ width: 0 }),
  };
  recordingContext = new Proxy(base, {
    get: (t, k) => (k in t ? t[k] : (...args) => { drawCalls.push({ op: k, args }); }),
    set: (t, k, v) => { drawCalls.push({ op: `set:${String(k)}`, args: [v] }); t[k] = v; return true; },
  });
  return recordingContext;
}

const sandbox = {
  document: {
    getElementById: () => stubElement(),
    createElement: () => stubElement(),
    querySelector: () => null, querySelectorAll: () => [],
    addEventListener() {}, activeElement: null, body: stubElement(),
  },
  WebSocket: class { constructor() { this.readyState = 1; } send() {} close() {} },
  location: { host: 'car:8080', hostname: 'car' },
  localStorage: { getItem: () => null, setItem() {}, removeItem() {} },
  performance: { now: () => Date.now() },
  requestAnimationFrame: () => 0,
  setTimeout, clearTimeout, setInterval: () => 0, clearInterval,
  addEventListener() {}, removeEventListener() {},
  innerWidth: 1280, innerHeight: 800, devicePixelRatio: 1,
  console,
};
sandbox.window = sandbox;
sandbox.WebSocket.OPEN = 1;

vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(DASHBOARD_JS, 'utf8'), sandbox, { filename: 'dashboard.js' });

const CAR = sandbox.window.__CAR_MODEL;
const geometry = sandbox.window.__carModelGeometry;
const ackermann = sandbox.window.__ackermannWheelAngles;
const order = sandbox.window.__overlayDrawOrder;

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

function near(a, b, tol, name, detail) {
  check(name, Number.isFinite(a) && Math.abs(a - b) <= tol,
    detail || `wanted ${b} +/- ${tol}, got ${a}`);
}

console.log('the geometry is reachable');
for (const [name, value] of [['__CAR_MODEL', CAR], ['__carModelGeometry', geometry],
  ['__ackermannWheelAngles', ackermann], ['__overlayDrawOrder', order]]) {
  check(`dashboard.js publishes ${name}`, value !== undefined && value !== null);
}
if (!CAR || !geometry || !ackermann || !order) {
  console.log('\ncannot continue without the published geometry');
  process.exit(1);
}

// --------------------------------------------------------------------------
console.log('\nthe measurements themselves');
// 1 mm: the tape these were taken with does not resolve finer, and a
// centimetre error here is a centimetre error on the map.
near(CAR.wheelbaseM, 0.36, 1e-9, 'wheelbase is the measured 0.36 m');
near(CAR.trackWidthM, 0.30, 1e-9, 'track is the measured 0.30 m over the tires');
// Stated as the relationship it was measured as, not as a second literal:
// "0.10 m behind the front axle" is the thing someone put a tape on.
near(CAR.wheelbaseM - CAR.lidarXM, 0.10, 1e-9,
  'the LiDAR is 0.10 m behind the FRONT axle');
near(CAR.lidarXM, 0.26, 1e-9,
  'which puts it 0.26 m ahead of the rear-axle base_link');

// --------------------------------------------------------------------------
console.log('\nthe footprint, in base_link coordinates');
const straight = geometry(0);
check('the rear axle is the origin -- base_link, where the pose is reported',
  straight.rearAxleX === 0);
near(straight.frontAxleX, CAR.wheelbaseM, 1e-9,
  'the front axle is one wheelbase ahead of it');
near(straight.halfTrack, 0.15, 1e-9, 'half the measured track is 0.15 m');
check('the outline reaches past both axles', straight.tailX < 0
  && straight.noseX > straight.frontAxleX);
check('there are four wheels', straight.wheels.length === 4);
// Each tire's OUTER edge must land exactly on the measured track, or the
// silhouette is not the width of the car.
for (const w of straight.wheels) {
  near(Math.abs(w.y) + w.width / 2, straight.halfTrack, 1e-9,
    `a tire's outer edge sits on the measured track (y=${w.y.toFixed(3)})`);
}
check('two wheels on the front axle, two on the rear',
  straight.wheels.filter((w) => w.x === CAR.wheelbaseM).length === 2
  && straight.wheels.filter((w) => w.x === 0).length === 2);
check('one wheel on each side of each axle',
  straight.wheels.filter((w) => w.y > 0).length === 2
  && straight.wheels.filter((w) => w.y < 0).length === 2);
check('going straight, no wheel is turned',
  straight.wheels.every((w) => w.angle === 0));
near(straight.steeringHalfWidth, straight.halfTrack, 1e-9,
  'and the car is exactly as wide as its track');

// --------------------------------------------------------------------------
console.log('\nAckermann: the identity, not a recorded number');
// cot(outer) - cot(inner) = track / wheelbase, for ANY steering angle.
const expectedCotDiff = CAR.trackWidthM / CAR.wheelbaseM;
for (const delta of [0.26, 0.15, 0.05, 0.01, -0.26, -0.15, -0.05, -0.01]) {
  const { left, right } = ackermann(delta);
  const inner = delta > 0 ? left : right;
  const outer = delta > 0 ? right : left;
  const cotDiff = (1 / Math.tan(Math.abs(outer))) - (1 / Math.tan(Math.abs(inner)));
  near(cotDiff, expectedCotDiff, 1e-9,
    `cot(outer) - cot(inner) = track/wheelbase at ${delta} rad`);
  check(`the inside wheel is turned further at ${delta} rad`,
    Math.abs(inner) > Math.abs(outer),
    `inner=${inner.toFixed(4)} outer=${outer.toFixed(4)}`);
  check(`both wheels turn the commanded way at ${delta} rad`,
    Math.sign(left) === Math.sign(delta) && Math.sign(right) === Math.sign(delta));
  // The commanded angle is the one that produces the intended turn radius,
  // so it must lie between the two wheels rather than outside them.
  check(`the commanded angle sits between the wheels at ${delta} rad`,
    Math.abs(outer) < Math.abs(delta) && Math.abs(delta) < Math.abs(inner));
}
{
  const zero = ackermann(0);
  check('straight ahead is straight on both wheels', zero.left === 0 && zero.right === 0);
  const left = ackermann(0.2);
  const right = ackermann(-0.2);
  near(left.left, -right.right, 1e-12, 'a left turn mirrors a right one (inside)');
  near(left.right, -right.left, 1e-12, 'a left turn mirrors a right one (outside)');
}

console.log('\nAckermann: inputs that are not steering angles');
for (const bad of [NaN, Infinity, -Infinity, undefined]) {
  const got = ackermann(bad);
  check(`${String(bad)} draws the wheels straight rather than NaN`,
    got.left === 0 && got.right === 0, JSON.stringify(got));
}
for (const extreme of [Math.PI / 2, -Math.PI / 2, Math.PI, -Math.PI, 100]) {
  const got = ackermann(extreme);
  check(`${extreme} still yields finite wheel angles`,
    Number.isFinite(got.left) && Number.isFinite(got.right), JSON.stringify(got));
  check(`${extreme} never pivots a wheel past a right angle`,
    Math.abs(got.left) <= Math.PI / 2 && Math.abs(got.right) <= Math.PI / 2);
}

// --------------------------------------------------------------------------
console.log('\nsteering clearance: the turned tire swings out of the footprint');
for (const delta of [0.26, -0.26, 0.15, -0.15]) {
  const g = geometry(delta);
  // Closed form: a rectangle of length L and width W centred at yc and
  // rotated by d reaches |y| = |yc| + (L/2)|sin d| + (W/2)|cos d|.
  const reach = (w) => Math.abs(w.y)
    + (w.length / 2) * Math.abs(Math.sin(w.angle))
    + (w.width / 2) * Math.abs(Math.cos(w.angle));
  const wantLeft = Math.max(...g.wheels.filter((w) => w.y > 0).map(reach));
  const wantRight = Math.max(...g.wheels.filter((w) => w.y < 0).map(reach));
  near(g.clearanceLeft, wantLeft, 1e-12, `left clearance at ${delta} rad`);
  near(g.clearanceRight, wantRight, 1e-12, `right clearance at ${delta} rad`);
  check(`a turned car needs more room than a parked one at ${delta} rad`,
    g.steeringHalfWidth > g.halfTrack,
    `${g.steeringHalfWidth} vs ${g.halfTrack}`);
  // The inside of the turn is the side that swings out furthest, and the
  // per-side split exists so the outside is not charged for it.
  const insideIsWider = delta > 0
    ? g.clearanceLeft > g.clearanceRight
    : g.clearanceRight > g.clearanceLeft;
  check(`the inside of the turn reaches furthest at ${delta} rad`, insideIsWider,
    `left=${g.clearanceLeft.toFixed(4)} right=${g.clearanceRight.toFixed(4)}`);
}
check('steering does not move the axles or the sensor', (() => {
  const g = geometry(0.26);
  return g.frontAxleX === straight.frontAxleX && g.rearAxleX === straight.rearAxleX
    && g.lidarX === straight.lidarX && g.noseX === straight.noseX;
})());

// --------------------------------------------------------------------------
console.log('\npaint order: the scan goes ON TOP of the car');
function orderFor(has) {
  return order({ scan: 'none', intent: 'none', car: 'none', ...has });
}
{
  const all = orderFor({ scan: 'map', intent: 'map', car: 'map' });
  check('the scan is painted after the car',
    all.indexOf('scan') > all.indexOf('car'), all.join(' -> '));
  check('the blind spot stays under the car',
    all.indexOf('blindspot') < all.indexOf('car'), all.join(' -> '));
  check('the blind spot stays under the scan',
    all.indexOf('blindspot') < all.indexOf('scan'), all.join(' -> '));
  check('intent stays under the car it belongs to',
    all.indexOf('intent') < all.indexOf('car'), all.join(' -> '));
}
// Every combination, so no frame state can sneak the car back on top.
for (const scan of ['none', 'map', 'body']) {
  for (const intent of ['none', 'map', 'body']) {
    for (const car of ['none', 'map', 'body']) {
      const got = orderFor({ scan, intent, car });
      const label = `scan=${scan} intent=${intent} car=${car}`;
      if (scan !== 'none' && car !== 'none') {
        check(`scan over car (${label})`,
          got.indexOf('scan') > got.indexOf('car'), got.join(' -> '));
      }
      check(`nothing is drawn for an overlay with no honest frame (${label})`,
        (scan !== 'none' || (!got.includes('scan') && !got.includes('blindspot')))
        && (intent !== 'none' || !got.includes('intent'))
        && (car !== 'none' || !got.includes('car')),
        got.join(' -> '));
      check(`every step is drawable (${label})`,
        got.every((s) => ['blindspot', 'intent', 'car', 'scan'].includes(s)),
        got.join(' -> '));
    }
  }
}

// --------------------------------------------------------------------------
console.log('\nthe drawing path actually runs, and never puts a NaN on the canvas');
const drawCarIcon = sandbox.window.__drawCarIcon;
check('dashboard.js publishes __drawCarIcon', typeof drawCarIcon === 'function');
if (typeof drawCarIcon === 'function') {
  // Every steering value a live /drive could carry, including ones it should
  // not. A NaN coordinate does not throw -- canvas quietly draws nothing --
  // so the car would simply disappear from the map with no error anywhere.
  for (const steering of [0, 0.26, -0.26, 0.05, NaN, Infinity, -Infinity, 10]) {
    drawCalls.length = 0;
    let threw = null;
    try {
      drawCarIcon(400, 300, -0.7, steering);
    } catch (err) {
      threw = err;
    }
    const label = `steering=${String(steering)}`;
    check(`drawing the car does not throw (${label})`, threw === null,
      threw && threw.stack);
    if (threw) continue;
    const numbers = drawCalls.flatMap((c) => c.args.filter((a) => typeof a === 'number'));
    check(`every number reaching the canvas is finite (${label})`,
      numbers.length > 0 && numbers.every((n) => Number.isFinite(n)),
      `${numbers.filter((n) => !Number.isFinite(n)).length} of ${numbers.length} were not`);
    // The parts that must exist: a body outline, four tires plus the
    // windshield band, and the ringed LIDAR puck.
    const ops = drawCalls.map((c) => c.op);
    check(`the LIDAR puck is drawn (${label})`, ops.includes('arc'));
    check(`the tires and windshield are drawn (${label})`,
      ops.filter((op) => op === 'fillRect').length >= 5,
      `${ops.filter((op) => op === 'fillRect').length} fillRect calls`);
    check(`the body outline is drawn (${label})`,
      ops.includes('quadraticCurveTo') && ops.includes('fill'));
    // Every wheel is drawn inside its own save/restore, so a rotation can
    // never leak into whatever is painted next.
    check(`the canvas state is left balanced (${label})`,
      ops.filter((op) => op === 'save').length === ops.filter((op) => op === 'restore').length,
      `${ops.filter((op) => op === 'save').length} save vs ${ops.filter((op) => op === 'restore').length} restore`);
  }
}

console.log('');
if (failures) {
  console.log(`${failures} of ${checks} checks FAILED`);
  process.exit(1);
}
console.log(`All ${checks} checks passed.`);
