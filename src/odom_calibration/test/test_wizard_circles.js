// The multi-setting circle calibration in the page (2026-10-16 procedure).
// Run through pytest's test_wizard_circles.py, or directly with node.
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

const source = fs.readFileSync(path.join(__dirname, "../web/wizard.js"), "utf8");
// Car 2 after the 2026-10-16 calibration (docs/hardware-reference.md).
const parameters = {
  speed_to_erpm_gain: -4447.983786, speed_to_erpm_offset: 0,
  steering_angle_to_servo_gain: -0.78,
  steering_angle_to_servo_offset: 0.5904, wheelbase: 0.36,
};
const TRACK = 0.27;

function trial(kind, extra = {}) {
  return {id: `${kind}-${Math.random()}`, kind, accepted: true,
    summary: {servo: {median: 0.59}}, ...extra};
}

function stepLabels(nav) {
  return [...nav.matchAll(/class="step-label">([^<]*)</g)].map(match => match[1]);
}

// Real wizard.js against a stub DOM; fetch is the only boundary mocked.
function browser() {
  const elements = new Map();
  function element(id) {
    if (!elements.has(id)) elements.set(id, {
      innerHTML: "", value: "", dataset: {}, classList: {toggle() {}},
      addEventListener() {}, appendChild() {}, remove() {}, setAttribute() {},
    });
    return elements.get(id);
  }
  const requests = [];
  const confirms = [];
  const sandbox = {
    document: {
      getElementById: element, createElement: () => element("toast"),
      querySelector: () => null,
      querySelectorAll: () => Object.entries(parameters).map(([key, value]) =>
        ({id: `param-${key}`, value: String(value)})),
    },
    location: {protocol: "http:", host: "localhost:8090"},
    WebSocket: class {close() {}},
    setTimeout() {}, clearTimeout() {}, setInterval() {},
    confirm: question => { confirms.push(question); return true; },
    fetch: async (url, options) => {
      if (url === "/api/state") return new Promise(() => {});
      requests.push(JSON.parse(options.body));
      return {ok: true, json: async () => ({ok: true})};
    },
  };
  sandbox.window = sandbox;
  vm.createContext(sandbox);
  vm.runInContext(source, sandbox);
  function snapshot(session = {}, top = {}) {
    sandbox.updateSnapshot({session: {session_id: "s1", mode: "steering", stage: "steering",
      current_parameters: {...parameters},
      vehicle: {servo_min: 0.15, servo_max: 0.95, rear_track_m: TRACK}, trials: [], ...session},
      telemetry: {}, health: {}, apply: {allowed: true}, ...top});
  }
  return {sandbox, element, requests, confirms, snapshot};
}

(async () => {
  // Three steering tests now; no wheels-up range check.
  for (const [mode, labels] of Object.entries({
    steering: ["Check the car", "Drive straight", "Left circle", "Right circle", "Report"],
    movement_steering: ["Check the car", "Stand still", "Distance runs",
      "Drive straight", "Left circle", "Right circle", "Report"],
  })) {
    const page = browser();
    page.snapshot({mode, stage: mode === "steering" ? "steering" : "movement"});
    assert.deepEqual(stepLabels(page.element("step-nav").innerHTML), labels, `steps for ${mode}`);
  }
  // Drift and both full-lock circles unlock the report; the range check is not needed.
  const done = browser();
  done.snapshot({trials: [trial("steering_drift"), trial("steering_left"), trial("steering_right")]});
  assert.doesNotMatch(done.element("step-nav").innerHTML.match(/<button[^>]*data-step="report"[^>]*>/)[0], / disabled/);
  const missing = browser();
  missing.snapshot({trials: [trial("steering_drift"), trial("steering_left")]});
  assert.match(missing.element("step-nav").innerHTML.match(/<button[^>]*data-step="report"[^>]*>/)[0], / disabled/);

  // The circle form defaults to the inner tire only and sends just box A.
  const inner = browser();
  inner.snapshot({pending_capture: {id: "p1", kind: "steering_right", summary: {}}});
  const form = inner.element("wizard-view").innerHTML;
  assert.match(form, /id="circle-outer-field"[^>]*hidden/, "outer tire box starts hidden");
  assert.match(form, /id="circle-axle-field"[^>]*hidden/, "axle box starts hidden");
  assert.match(form, /rear track, 0\.27 m/);
  inner.element("circle-inner").value = "1.33";
  inner.element("circle-outer").value = "9.99";       // stale value in a hidden box
  inner.element("circle-fulllock").checked = true;
  inner.element("trial-confirmed").checked = true;
  await inner.sandbox.acceptPending();
  assert.equal(inner.requests.length, 1);
  const sent = inner.requests[0];
  assert.equal(sent.measurement_method, "inner_tire");
  assert.equal(sent.measured_inner_diameter_m, 1.33);
  assert.equal("measured_outer_diameter_m" in sent, false, "hidden outer box must not be sent");
  assert.equal(sent.full_lock, true);

  // Both rear tires is still available and sends A and B.
  const tires = browser();
  tires.snapshot({pending_capture: {id: "p2", kind: "steering_left", summary: {}}});
  tires.sandbox.setCircleMethod("rear_tires");
  assert.doesNotMatch(tires.element("wizard-view").innerHTML, /id="circle-outer-field"[^>]*hidden/);
  tires.element("circle-inner").value = "1.06";
  tires.element("circle-outer").value = "1.60";
  await tires.sandbox.acceptPending();
  assert.equal(tires.requests[0].measurement_method, "rear_tires");
  assert.equal(tires.requests[0].measured_outer_diameter_m, 1.6);
  tires.sandbox.setCircleMethod("not_a_method");      // ignored: the method stays
  await tires.sandbox.acceptPending();
  assert.equal(tires.requests.at(-1).measurement_method, "rear_tires");

  // Expected diameter for the settings the car runs: closed form, both sides.
  const scale = 25 * Math.PI / 180;
  for (const side of ["left", "right"]) {
    const page = browser();
    page.snapshot({}, {live_stick_scale: scale});
    await page.sandbox.goToStep(`steering_${side}`);
    const html = page.element("wizard-view").innerHTML;
    const servo = 0.5904 - 0.78 * (side === "left" ? scale : -scale);
    const expected = 2 * 0.36 / Math.tan(scale) - TRACK;
    assert.ok(html.includes(`servo <span class="mono">${servo.toFixed(3)}</span>`), `${side} servo shown`);
    assert.ok(html.includes(`about <strong>${expected.toFixed(2)} m</strong>`), `${side} expected diameter`);
    assert.equal(html.includes("cut by"), false, "25 deg is not clipped at servo_max 0.95");
  }
  // The 2026-10-10 settings: full right clipped by servo_max, diameter from the clipped angle.
  const clipped = browser();
  clipped.snapshot({current_parameters: {...parameters, steering_angle_to_servo_gain: -1.2135},
    vehicle: {servo_min: 0.15, servo_max: 0.85, rear_track_m: TRACK}}, {live_stick_scale: 0.34,
    clamp_check: {left_reach_rad: 0.3627, right_reach_rad: 0.2139,
      findings: ["Full stick asks for 19.5°, but servo_max cuts the right side at 12.3°."]}});
  await clipped.sandbox.goToStep("steering_right");
  const clippedHtml = clipped.element("wizard-view").innerHTML;
  const clippedAngle = (0.85 - 0.5904) / 1.2135;
  assert.ok(clippedHtml.includes("(cut by servo_max)"));
  assert.ok(clippedHtml.includes(`about <strong>${(2 * 0.36 / Math.tan(clippedAngle) - TRACK).toFixed(2)} m</strong>`));
  assert.ok(clippedHtml.includes("cuts the right side at 12.3°"), "clamp finding shown on the circle step");
  // No full-stick angle known: no prediction is made up.
  const unknown = browser();
  unknown.snapshot();
  await unknown.sandbox.goToStep("steering_left");
  assert.ok(unknown.element("wizard-view").innerHTML.includes("expected diameter appears once"));

  // Report: curve table, check, and the limit write sends exactly the ready names.
  const report = {
    overall_status: "review", safety_note: "", current_parameters: {...parameters, servo_max: 1.0, steering_stick_scale: 0.52},
    parameter_suggestions: {servo_max: 0.95, steering_stick_scale: scale,
      steering_angle_to_servo_gain: -0.79},
    not_ready_to_apply: {steering_angle_to_servo_gain: "confidence is low"},
    blocked_suggestions: {},
    clamp_check: {left_reach_rad: 0.56, right_reach_rad: 0.525, findings: []},
    steering: {
      status: "low", trial_results: [], servo_limits: [0.15, 1.0], warnings: [],
      sides: {left: {check: {commanded_deg: 29.8, measured_deg: 29.2, error_deg: -0.6, passed: true}},
              right: {check: {commanded_deg: 29.3, measured_deg: 26.1, error_deg: -3.2, passed: false}}},
      calibration_confirmed: false,
      curves: {left: {points: [], steps: []}, right: {
        points: [{servo: 0.85, angle_deg: 19.1, count: 1}, {servo: 0.95, angle_deg: 25.6, count: 1},
                 {servo: 1.0, angle_deg: 26.1, count: 1}],
        steps: [{added_deg: 6.5}, {added_deg: 0.5}],
        stop: {servo: 0.95, beyond_servo: 1.0, added_deg: 0.5}}},
    },
  };
  const page = browser();
  page.snapshot({stage: "report", report,
    trials: [trial("steering_drift"), trial("steering_left"), trial("steering_right")]});
  const html = page.element("wizard-view").innerHTML;
  assert.ok(html.includes("+0.5°"), "added angle per step shown");
  assert.equal((html.match(/past the stop/g) || []).length, 1, "only one circle is past the stop");
  const rows = html.split("<tr>").filter(row => row.includes('class="num">'));
  const stopRow = rows.find(row => row.includes("past the stop"));
  assert.ok(stopRow.includes(">1.000<"), "and it is the servo 1.00 circle");
  assert.ok(html.includes("-3.2°") && html.includes("off"), "failed check shown");
  assert.equal(html.includes("calibration is confirmed"), false);
  assert.ok(html.includes("Suggested full stick: 25.0°"));
  const button = html.match(/applyReport\('([^']*)'\)">Write servo limits/);
  assert.ok(button, "limit write button rendered");
  assert.deepEqual(button[1].split(","), ["servo_max", "steering_stick_scale"]);
  await page.sandbox.applyReport(button[1]);
  assert.deepEqual(page.requests.at(-1), {action: "apply_parameters", confirmed: true,
    source: "report", names: ["servo_max", "steering_stick_scale"]});
  assert.match(page.confirms.at(-1), /joy_teleop\.yaml/);
  // The gain that is not ready is never offered.
  assert.equal(/applyReport\('[^']*steering_angle_to_servo_gain/.test(html), false);

  // A pending stick-scale write asks for a teleop restart, not only bringup.
  for (const [waiting, words] of [[["steering_stick_scale"], "Restart teleop."],
                                  [["servo_max"], "Restart bringup."],
                                  [["servo_max", "steering_stick_scale"], "Restart bringup and teleop."]]) {
    const banner = browser();
    banner.snapshot({}, {pending_restart: waiting});
    await banner.sandbox.goToStep("steering_left");
    assert.ok(banner.element("wizard-view").innerHTML.includes(words), words);
  }

  console.log("Wizard circle checks passed.");
})().catch(error => { console.error(error); process.exitCode = 1; });
