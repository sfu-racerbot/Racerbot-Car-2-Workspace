/*
 * Does the map panel ever offer to delete something it must not, or accept
 * a confirmation that does not match?
 *
 * The server is the real lock: it re-scans the map roots and re-vets the
 * name, the contents and the digest before it removes anything, so a
 * browser asking to delete a protected path is refused whatever its page
 * looks like. This checks the second lock -- that the page never offers
 * the control in the first place.
 *
 * Both matter, for the same reason the stop panel has both: a delete
 * button that is offered and then silently refused teaches people that
 * the panel is broken, and the next thing they reach for is `rm -rf`.
 *
 * `mapRowPlan` and `deleteConfirmState` are pure -- a run object in, a
 * verdict out -- so nothing here renders or deletes anything.
 *
 * Run directly with `node map_panel_test.js`, or through pytest via
 * test_map_panel_js.py.
 */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const MEASURE_JS = path.join(__dirname, '..', '..', 'web', 'measure.js');
const DASHBOARD_JS = path.join(__dirname, '..', '..', 'web', 'dashboard.js');

// Same stub as proc_panel_test.js: dashboard.js is one big IIFE that wires
// up a whole page on load, and it only has to reach the lines that publish
// the two pure functions.
function stubElement() {
  const el = {
    style: {}, dataset: {}, children: [], textContent: '', innerHTML: '',
    checked: false, value: '', disabled: false, hidden: false,
    width: 800, height: 600,
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
// measure.js first, exactly as index.html loads it: dashboard.js reads
// window.__measure while it initialises.
vm.runInContext(fs.readFileSync(MEASURE_JS, 'utf8'), sandbox, { filename: 'measure.js' });
vm.runInContext(fs.readFileSync(DASHBOARD_JS, 'utf8'), sandbox, { filename: 'dashboard.js' });

const mapRowPlan = sandbox.window.__mapRowPlan;
const deleteConfirmState = sandbox.window.__deleteConfirmState;

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

function section(title) { console.log(title); }

// A run as the server sends it.
const RUN = {
  id: '20260727-200103',
  path: '/home/x/.ros/racerbot_auto/20260727-200103',
  bytes: 29383, has_map: true, has_posegraph: true,
  contents: ['map', 'pose graph', 'racing line'],
  span_m: [6.35, 10.95], deletable: true, reason: '',
  digest: 'abc123', cited_by: '',
};
const withRun = (over) => Object.assign({}, RUN, over);

// ---------------------------------------------------------------------------
section('the panel is reachable');
check('mapRowPlan is published', typeof mapRowPlan === 'function');
check('deleteConfirmState is published', typeof deleteConfirmState === 'function');
check('measure.js loaded into the same context',
  typeof sandbox.window.__measure === 'object');

// ---------------------------------------------------------------------------
section('mapRowPlan: what a row offers');

eq('a complete run is deletable', mapRowPlan(RUN).deletable, true);
eq('the row is named by the run id', mapRowPlan(RUN).name, '20260727-200103');
check('the row lists what deleting takes with it',
  mapRowPlan(RUN).contents.includes('pose graph')
  && mapRowPlan(RUN).contents.includes('racing line'),
  'the whole point of the confirmation step is that this is on screen first');
check('the row carries the size', mapRowPlan(RUN).meta.includes('kB'));
// 6.35 and 10.95 are not exactly representable in binary floating point and
// both sit a hair BELOW the decimal they are written as, so toFixed(1)
// rounds them down: "6.3" and "10.9", not "6.4" and "11.0". Asserted as it
// actually is -- this is a display rounding of a display number, and
// pretending otherwise would just make the test wrong.
check('the row carries the map span in metres',
  mapRowPlan(RUN).meta.includes('6.3') && mapRowPlan(RUN).meta.includes('10.9'),
  `meta was: ${mapRowPlan(RUN).meta}`);

section('mapRowPlan: refusals from the server are honoured, not overridden');
eq('a run the server marked undeletable gets no control',
  mapRowPlan(withRun({ deletable: false, reason: 'holds files this dashboard does not recognise' })).deletable,
  false);
eq('and the server reason is shown, not replaced',
  mapRowPlan(withRun({ deletable: false, reason: 'holds foreign files' })).reason,
  'holds foreign files');
eq('a run marked undeletable with no reason still says something',
  mapRowPlan(withRun({ deletable: false, reason: '' })).reason, 'not deletable');
eq('deletable must be exactly true, not merely truthy',
  mapRowPlan(withRun({ deletable: 'yes' })).deletable, false);
eq('a missing deletable field fails closed',
  mapRowPlan({ id: 'x' }).deletable, false);

section('mapRowPlan: fails closed on anything it cannot identify');
eq('no run at all', mapRowPlan(undefined).deletable, false);
eq('an empty object', mapRowPlan({}).deletable, false);
eq('a run with no id', mapRowPlan(withRun({ id: '' })).deletable, false);
eq('a run whose id is not a string', mapRowPlan(withRun({ id: 42 })).deletable, false);
eq('and it says so', mapRowPlan({}).reason, 'unrecognised entry');

section('mapRowPlan: the things a person needs to see before confirming');
eq('a run with no map says so',
  mapRowPlan(withRun({ has_map: false, span_m: null })).meta.includes('no map'), true);
eq('a run used as a test oracle is flagged',
  mapRowPlan(withRun({ cited_by: 'test_map_despeckle.py' })).citedBy,
  'test_map_despeckle.py');
eq('an uncited run carries no citation', mapRowPlan(RUN).citedBy, '');
eq('a run with nothing recognised still describes itself',
  mapRowPlan(withRun({ contents: [] })).contents, 'nothing recognised');

// ---------------------------------------------------------------------------
section('deleteConfirmState: only an exact match arms the button');

eq('the exact name arms it',
  deleteConfirmState('20260727-200103', RUN).enabled, true);
eq('one character short does not',
  deleteConfirmState('20260727-20010', RUN).enabled, false);
eq('one character extra does not',
  deleteConfirmState('20260727-2001031', RUN).enabled, false);
eq('a trailing space does not -- the comparison is not trimmed',
  deleteConfirmState('20260727-200103 ', RUN).enabled, false);
eq('a leading space does not',
  deleteConfirmState(' 20260727-200103', RUN).enabled, false);
eq('an empty string does not', deleteConfirmState('', RUN).enabled, false);
eq('a different case does not',
  deleteConfirmState('RUNONE', withRun({ id: 'RunOne' })).enabled, false);
eq('the right case does', deleteConfirmState('RunOne', withRun({ id: 'RunOne' })).enabled, true);
eq('a truthy string is not a confirmation',
  deleteConfirmState('true', RUN).enabled, false);
eq('undefined input does not arm it',
  deleteConfirmState(undefined, RUN).enabled, false);

section('deleteConfirmState: the two locks compose');
eq('a perfect string cannot arm a run the server refused',
  deleteConfirmState('20260727-200103',
    withRun({ deletable: false, reason: 'in use' })).enabled, false);
eq('and the reason given is the server’s, not "type the name"',
  deleteConfirmState('20260727-200103',
    withRun({ deletable: false, reason: 'in use' })).reason, 'in use');
eq('a perfect string cannot arm an unidentifiable row',
  deleteConfirmState('anything', {}).enabled, false);

section('deleteConfirmState: the disabled state explains itself');
check('an unarmed button says what to type',
  deleteConfirmState('', RUN).reason.includes('20260727-200103'),
  'a disabled control with no explanation is one people press repeatedly');
eq('an armed button needs no explanation',
  deleteConfirmState('20260727-200103', RUN).reason, '');

// ---------------------------------------------------------------------------
console.log('');
if (failures) {
  console.log(`${failures} of ${checks} checks FAILED`);
  process.exit(1);
}
console.log(`${checks} checks passed`);
