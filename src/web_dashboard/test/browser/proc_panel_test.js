/*
 * Does the stop panel ever offer to stop something it must not?
 *
 * The server is the real lock here: it re-scans /proc and re-vets every
 * pid before it signals anything, so a browser that asks to kill the mux
 * is refused no matter what its page looks like. This checks the second
 * lock -- that the page never renders the control in the first place.
 *
 * Both matter. If only the server refused, the panel would show a stop
 * button that silently does nothing, and the person at trackside would
 * press it, watch nothing happen, and press it again instead of reaching
 * for the thing that actually stops the car (releasing LB).
 *
 * `procRowPlan` is pure -- target object in, "does this row get a button"
 * out -- so nothing here renders or signals anything.
 *
 * Run directly with `node proc_panel_test.js`, or through pytest via
 * test_proc_panel_js.py.
 */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const DASHBOARD_JS = path.join(__dirname, '..', '..', 'web', 'dashboard.js');

// Same stub as draw_frames_test.js: dashboard.js is one big IIFE that
// wires up a whole page on load, and it only has to reach the line that
// publishes procRowPlan.
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

const procRowPlan = sandbox.window.__procRowPlan;

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

console.log('procRowPlan is reachable');
check('dashboard.js publishes procRowPlan', typeof procRowPlan === 'function');
if (typeof procRowPlan !== 'function') {
  console.log('\ncannot continue without procRowPlan');
  process.exit(1);
}

console.log('\na stoppable driving node');
const driving = procRowPlan({
  pid: 4242, name: 'pure_pursuit_node', kind: 'node',
  cmdline: '/usr/bin/python3 /x/pure_pursuit_node', protected: false, reason: '',
});
check('gets a stop control', driving.stoppable === true);
check('is labelled with its node name', driving.name === 'pure_pursuit_node');
check('shows its pid', driving.meta === 'pid 4242');
check('carries no refusal reason', driving.reason === '');

console.log('\nthe actuation path');
const mux = procRowPlan({
  pid: 99, name: 'ackermann_mux', kind: 'node', cmdline: '/x/ackermann_mux',
  protected: true,
  reason: 'in the actuation path -- stopping it would leave a moving car '
        + 'that releasing LB can no longer stop',
});
check('never gets a stop control', mux.stoppable === false);
check('explains itself instead', mux.reason.includes('actuation path'));

console.log('\na protected row with no reason supplied');
const bare = procRowPlan({ pid: 7, name: 'joy_teleop', protected: true });
check('still gets no stop control', bare.stoppable === false);
check('falls back to a non-empty reason', bare.reason === 'protected');

console.log('\ndefensive: junk from the wire, which must fail closed');
// An entry this page cannot identify is not a licence to offer a kill
// button for it. `protected` being absent must not read as "allowed".
check('an undefined target is not stoppable',
  procRowPlan(undefined).stoppable === false);
check('an empty target is not stoppable',
  procRowPlan({}).stoppable === false);
check('a target with no pid is not stoppable',
  procRowPlan({ name: 'mystery_node' }).stoppable === false);
check('and it says why rather than showing a blank row',
  procRowPlan({}).reason === 'unrecognised entry');
check('nothing above throws', true);

console.log(`\n${checks - failures}/${checks} checks passed`);
process.exit(failures === 0 ? 0 : 1);
