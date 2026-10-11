// Run through pytest's test_wizard_ui.py, or directly with node.
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

const source = fs.readFileSync(path.join(__dirname, "../web/wizard.js"), "utf8");
const parameters = {
  speed_to_erpm_gain: -4447.983786, speed_to_erpm_offset: 0,
  steering_angle_to_servo_gain: -1.2135,
  steering_angle_to_servo_offset: 0.5304, wheelbase: 0.36,
};

function browser() {
  const elements = new Map();
  function element(id) {
    if (!elements.has(id)) elements.set(id, {
      innerHTML: "", dataset: {}, classList: {toggle() {}},
      addEventListener() {}, appendChild() {}, remove() {},
      setAttribute() {},
    });
    return elements.get(id);
  }
  const requests = [];
  const sandbox = {
    document: {
      getElementById: element, createElement: () => element("toast"),
      querySelectorAll: () => Object.entries(parameters).map(([key, value]) =>
        ({id: `param-${key}`, value: String(value)})),
    },
    location: {protocol: "http:", host: "localhost:8090"},
    WebSocket: class {close() {}},
    setTimeout() {}, clearTimeout() {}, setInterval() {},
    confirm: () => true,
    fetch: async (url, options) => {
      // Initial fetch waits for an explicit snapshot from the test.
      if (url === "/api/state") return new Promise(() => {});
      const body = JSON.parse(options.body);
      requests.push(body);
      return {ok: true, json: async () => ({ok: true, state: {
        session: {session_id: "new", mode: body.mode, stage: "preflight",
          current_parameters: body.current_parameters, trials: []},
        telemetry: {}, health: {},
      }})};
    },
  };
  sandbox.window = sandbox;
  vm.createContext(sandbox);
  vm.runInContext(source, sandbox);
  function snapshot(mode = "movement", extra = {}) {
    sandbox.updateSnapshot({session: {session_id: "original", mode,
      stage: "movement", current_parameters: {...parameters}, trials: [], ...extra},
      telemetry: {}, health: {}});
  }
  return {sandbox, element, requests, snapshot};
}

(async () => {
  // Regression: a restored wheel session must allow choosing every mode,
  // instead of silently creating another wheel session.
  for (const mode of ["steering", "movement", "movement_steering"]) {
    const page = browser();
    page.snapshot();
    page.sandbox.openCalibrationSetup();
    assert.equal(page.requests.length, 0, "opening setup must preserve the session");
    const setup = page.element("wizard-view").innerHTML;
    for (const label of ["Steering calibration", "Wheel calibration", "Both calibrations"]) {
      assert.ok(setup.includes(label), `setup must offer ${label}`);
    }
    page.sandbox.selectCalibrationMode(mode);
    page.snapshot(); // Live telemetry must not remove the selection screen.
    assert.ok(page.element("wizard-view").innerHTML.includes("Both calibrations"));
    await page.sandbox.createSession();
    assert.equal(page.requests.length, 1);
    assert.equal(page.requests[0].mode, mode);
    assert.equal(page.requests[0].replace_existing, true, "archive on replacement");
    assert.deepEqual(page.requests[0].current_parameters, parameters);
    assert.ok(page.element("wizard-view").innerHTML.includes("Preflight"));
    const nav = page.element("step-nav").innerHTML;
    assert.equal(nav.includes("Distance runs"), mode !== "steering");
    // Each steering test is its own sidebar step (README, "Guided workflow").
    assert.equal(nav.includes("Left circle"), mode !== "movement");
    assert.equal(nav.includes("Steering range"), mode !== "movement");
    const numbers = [...nav.matchAll(/class="step-number" aria-hidden="true">(\d+)/g)]
      .map(match => Number(match[1]));
    assert.deepEqual(numbers, Array.from({length: numbers.length}, (_, i) => i + 1));
    assert.ok(nav.includes('aria-current="step"'));
  }

  const canceled = browser();
  canceled.snapshot();
  canceled.sandbox.openCalibrationSetup();
  canceled.sandbox.selectCalibrationMode("steering");
  canceled.sandbox.confirm = () => false;
  await canceled.sandbox.createSession();
  assert.equal(canceled.requests.length, 0, "declining replacement must send no write");
  canceled.sandbox.cancelCalibrationSetup();
  assert.ok(canceled.element("wizard-view").innerHTML.includes("Distance runs"));
  assert.ok(!canceled.element("step-nav").innerHTML.includes("Left circle"));

  const recording = browser();
  recording.snapshot("steering", {active_capture: {kind: "steering_drift", started_at: "now"}});
  recording.sandbox.openCalibrationSetup();
  await recording.sandbox.createSession();
  assert.equal(recording.requests.length, 0, "recording must block session replacement");
  assert.equal(recording.element("new-session-button").disabled, true);
  assert.ok(recording.element("wizard-view").innerHTML.includes("Recording"));
  console.log("Wizard UI regression checks passed.");
})().catch(error => { console.error(error); process.exitCode = 1; });
