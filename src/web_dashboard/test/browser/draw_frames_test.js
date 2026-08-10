/*
 * Which coordinate frame does each overlay get drawn in?
 *
 * This is the check that was missing when the car icon was drawn in the
 * car's own body frame while a world-frame map filled the screen. In that
 * state "the car is in the middle of the view, pointing up" is a fact
 * about the viewport, not about the car -- but on top of a map it reads
 * as a real position and a real heading. The scan already refused to draw
 * without a pose; the car did not, so it sat in the wrong place, faced a
 * fixed "up" whatever its true heading, and slid against the map whenever
 * the view was re-fitted.
 *
 * Nothing here renders anything. `drawFrames` is pure -- it takes four
 * booleans (do we have a map / pose / scan / intent) and returns the frame
 * each overlay may use -- which is exactly the decision that was wrong.
 *
 * Run directly with `node draw_frames_test.js`, or through pytest via
 * test_draw_frames_js.py.
 */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const DASHBOARD_JS = path.join(__dirname, '..', '..', 'web', 'dashboard.js');

// dashboard.js is one big IIFE that wires up a whole page on load. It only
// has to reach the line that publishes drawFrames, so give it a DOM stub
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
  el.getContext = () => new Proxy({
    canvas: el,
    createImageData: (w, h) => ({ width: w, height: h, data: new Uint8ClampedArray(w * h * 4) }),
    putImageData() {}, measureText: () => ({ width: 0 }), save() {}, restore() {},
  }, { get: (t, k) => (k in t ? t[k] : () => {}), set: (t, k, v) => { t[k] = v; return true; } });
  return el;
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

const drawFrames = sandbox.window.__drawFrames;

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

function frames(has) {
  return drawFrames({ map: false, pose: false, scan: false, intent: false, ...has });
}

function expect(name, has, want) {
  const got = frames(has);
  const same = Object.keys(want).every((k) => got[k] === want[k]);
  check(name, same, `wanted ${JSON.stringify(want)}, got ${JSON.stringify(got)}`);
}

console.log('drawFrames is reachable');
check('dashboard.js publishes drawFrames', typeof drawFrames === 'function');
if (typeof drawFrames !== 'function') {
  console.log('\ncannot continue without drawFrames');
  process.exit(1);
}

console.log('\nthe regression: a map is up, but no pose has arrived');
// Every one of these is the bug. The car must not be drawn at all: we do
// not know where it is, and body-frame "here" over a world map is a lie.
expect('the car is not drawn without a pose',
  { map: true, scan: true }, { car: 'none' });
expect('nor is the scan (this part was already right)',
  { map: true, scan: true }, { scan: 'none' });
expect('nor is the intent',
  { map: true, scan: true, intent: true }, { intent: 'none' });
expect('and a map with no scan draws nothing either',
  { map: true }, { car: 'none', scan: 'none', intent: 'none' });

console.log('\nfully localized: everything locks to the map');
expect('all three use world coordinates',
  { map: true, pose: true, scan: true, intent: true },
  { car: 'map', scan: 'map', intent: 'map' });
expect('the car is drawn from a pose even with no scan',
  { map: true, pose: true }, { car: 'map' });

console.log('\nno map at all: the body frame is honest');
expect('scan and car are robot-centric',
  { scan: true }, { car: 'body', scan: 'body' });
expect('intent is robot-centric too',
  { scan: true, intent: true }, { intent: 'body' });
expect('a pose without a map still uses the body frame',
  { pose: true, scan: true, intent: true },
  { scan: 'body', intent: 'body' });

console.log('\nnothing has arrived yet');
expect('draw nothing rather than something wrong',
  {}, { car: 'none', scan: 'none', intent: 'none' });

console.log('\nthe invariant behind all of the above');
// The car and the scan may never disagree about the frame while both are
// drawable. Disagreement is exactly what the bug looked like: a map-frame
// scan (or none) beside a body-frame car.
for (const map of [false, true]) {
  for (const pose of [false, true]) {
    for (const intent of [false, true]) {
      const got = frames({ map, pose, scan: true, intent });
      const agree = got.car === got.scan;
      check(`car and scan agree (map=${map} pose=${pose} intent=${intent})`,
        agree, `car=${got.car} scan=${got.scan}`);
    }
  }
}

console.log('');
if (failures) {
  console.log(`${failures} of ${checks} checks FAILED`);
  process.exit(1);
}
console.log(`All ${checks} checks passed.`);
