// Overrides in the page: the servo-limit step on the circle pages, values
// typed by hand, editing a suggestion, keeping a flagged run, undo-all and
// the rear track. Run through pytest's test_wizard_overrides.py, or with node.
//
// Oracles: car 2 on 2026-10-16 (docs/hardware-reference.md: gain -0.78,
// offset 0.5904, clip 0.15-0.95, full stick 0.436 rad, right circles 0.85 ->
// 19.1 deg, 0.90 -> 23.3, 0.95 -> 25.6, 1.00 -> 26.1), and the servo map
// servo = gain * angle + offset.
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

const source = fs.readFileSync(path.join(__dirname, "../web/wizard.js"), "utf8");
const parameters = {
  speed_to_erpm_gain: -4447.983786, speed_to_erpm_offset: 0,
  steering_angle_to_servo_gain: -0.78,
  steering_angle_to_servo_offset: 0.5904, wheelbase: 0.36,
};

// Real wizard.js against a stub DOM; fetch and confirm are the boundaries.
function browser({confirmAnswer = true} = {}) {
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
  const toasts = [];
  const sandbox = {
    document: {
      getElementById: element,
      createElement: () => ({set textContent(text) { toasts.push(text); }, classList: {add() {}},
        remove() {}, setAttribute() {}}),
      querySelector: () => null,
      querySelectorAll: () => [],
    },
    location: {protocol: "http:", host: "localhost:8090"},
    WebSocket: class {close() {}},
    setTimeout() {}, clearTimeout() {}, setInterval() {},
    confirm: question => { confirms.push(question); return confirmAnswer; },
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
      vehicle: {servo_min: 0.15, servo_max: 0.95, rear_track_m: 0.27, steering_stick_scale: 0.436},
      trials: [], ...session},
      telemetry: {}, health: {}, apply: {allowed: true}, live_stick_scale: 0.436, ...top});
  }
  const view = () => element("wizard-view").innerHTML;
  return {sandbox, element, requests, confirms, toasts, snapshot, view};
}

// The whole <input ...> tag with this id, so attribute order and line breaks
// in the markup do not matter.
function inputTag(html, id) {
  const match = html.match(new RegExp(`<input id="${id}"[^>]*>`));
  assert.ok(match, `no input ${id}`);
  return match[0];
}
const isTicked = (html, id) => /\schecked[\s>]/.test(inputTag(html, id));

// The right-side curve as steering_curve.side_curve builds it.
function rightCurve(points, stop = null) {
  const steps = points.slice(1).map((point, i) => ({from_servo: points[i][0], to_servo: point[0],
    added_deg: point[1] - points[i][1]}));
  return {points: points.map(([servo, angle]) => ({servo, angle_deg: angle})), steps, stop};
}

async function openRight(page, curve, session = {}, top = {}) {
  page.snapshot(session, {steering_preview: {curves: {right: curve}, trials: {}}, ...top});
  await page.sandbox.goToStep("steering_right");
  return page.view();
}

(async () => {
  // --- the servo-limit step --------------------------------------------------
  // Right circles up to servo_max 0.95, still gaining: offer servo_max 1.000.
  const gaining = browser();
  let html = await openRight(gaining, rightCurve([[0.85, 19.1], [0.90, 23.3], [0.95, 25.6]]));
  assert.ok(html.includes("Find how far the right side steers"));
  assert.ok(html.includes("reached servo_max 0.950 at 25.6°"), "says where the last circle got");
  assert.ok(html.includes("Write servo_max 1.000 (now 0.950)"));
  assert.ok(html.includes("+4.2°") && html.includes("+2.3°"), "angle added per step");
  // Full right at 0.436 rad sends 0.5904 + 0.78 * 0.436 = 0.930 < 1.000:
  // full stick cannot reach the new limit, so the raise is offered.
  assert.ok(html.includes('id="limit-raise-stick"'));
  assert.ok(html.includes("Wheels off the ground"), "the wheels-up check comes before driving");
  gaining.element("limit-raise-stick").checked = true;
  await gaining.sandbox.stepLimit("right");
  assert.deepEqual(gaining.requests, [{action: "apply_parameters", confirmed: true,
    source: "limit_step", side: "right", step: 0.05, raise_stick: true}]);
  assert.match(gaining.confirms[0], /raise full stick/);
  assert.match(gaining.confirms[0], /wheels off the ground/);
  gaining.element("limit-raise-stick").checked = false;
  await gaining.sandbox.stepLimit("right");
  assert.equal(gaining.requests[1].raise_stick, false);

  // Full stick that already reaches the next limit is not offered a raise:
  // 0.60 rad sends 0.5904 + 0.468 = 1.058 > 1.000.
  const reaches = browser();
  html = await openRight(reaches, rightCurve([[0.95, 25.6]]), {}, {live_stick_scale: 0.60});
  assert.equal(html.includes('id="limit-raise-stick"'), false);
  assert.ok(html.includes("Restart bringup when the banner asks"));

  // The levelled-off side says so, marks the step past the stop, and names
  // the suggestion (2026-10-16: 0.95 -> 1.00 added 0.5 deg).
  const stopped = browser();
  html = await openRight(stopped, rightCurve(
    [[0.85, 19.1], [0.90, 23.3], [0.95, 25.6], [1.00, 26.1]],
    {servo: 0.95, beyond_servo: 1.00, added_deg: 0.5}), {
    vehicle: {servo_min: 0.15, servo_max: 1.0, rear_track_m: 0.27}});
  assert.ok(html.includes("levels off past servo 0.950"));
  assert.ok(html.includes("suggests servo_max 0.950"));
  assert.equal((html.match(/past the stop/g) || []).length, 1, "only the 1.00 circle");
  // At 1.0 there is no further step to take.
  assert.ok(html.includes("the end of the servo signal"));
  assert.equal(html.includes("stepLimit("), false);

  // The step size is editable and bounded.
  const sized = browser();
  await openRight(sized, rightCurve([[0.95, 25.6]]));
  sized.sandbox.setLimitStep("0.02");
  assert.ok(sized.view().includes("Write servo_max 0.970 (now 0.950)"));
  sized.sandbox.setLimitStep("0.5");
  assert.ok(sized.view().includes("Write servo_max 0.970"), "a refused size changes nothing");
  assert.ok(sized.toasts.some(text => /between 0.005 and 0.1/.test(text)));
  // 0.95 + 0.07 would pass 1.0: the button stops at the end of the signal.
  sized.sandbox.setLimitStep("0.07");
  assert.ok(sized.view().includes("Write servo_max 1.000"));

  // Left lowers servo_min with this car's negative gain; a positive gain
  // swaps the ends.
  const left = browser();
  left.snapshot({}, {steering_preview: {curves: {left: rightCurve([[0.18, 29.2]])}, trials: {}}});
  await left.sandbox.goToStep("steering_left");
  assert.ok(left.view().includes("Write servo_min 0.100 (now 0.150)"));
  const flipped = browser();
  flipped.snapshot({current_parameters: {...parameters, steering_angle_to_servo_gain: 0.78}},
    {steering_preview: {curves: {left: rightCurve([[0.9, 29.2]])}, trials: {}}});
  await flipped.sandbox.goToStep("steering_left");
  assert.ok(flipped.view().includes("Write servo_max 1.000 (now 0.950)"));

  // Before any circle: an explanation, no step button.
  const empty = browser();
  html = await openRight(empty, null);
  assert.ok(html.includes("Drive one full-lock right circle first"));
  // Writing switched off on the car: nothing to press.
  const off = browser();
  html = await openRight(off, rightCurve([[0.95, 25.6]]), {}, {apply: {allowed: false}});
  assert.equal(html.includes("stepLimit("), false);
  assert.equal(html.includes("writeValueEditor("), false);

  // --- values typed by hand ----------------------------------------------------
  const manual = browser();
  manual.snapshot({stage: "preflight"});
  html = manual.view();
  assert.ok(html.includes("Change a setting by hand"));
  for (const name of ["steering_angle_to_servo_gain", "servo_min", "servo_max", "steering_stick_scale"]) {
    assert.ok(html.includes(`id="manual-preflight-val-${name}"`), `box for ${name}`);
  }
  // Boxes start at what the car runs now; full stick is shown in degrees.
  assert.match(html, /id="manual-preflight-val-servo_max"[^>]*value="0.950"/);
  assert.match(html, /id="manual-preflight-val-steering_stick_scale"[^>]*value="25.0"/);
  const send = () => manual.sandbox.writeValueEditor("manual-preflight",
    "servo_max,steering_stick_scale,steering_angle_to_servo_gain");
  await send();
  assert.deepEqual(manual.requests, [], "nothing ticked, nothing sent");
  assert.ok(manual.toasts.some(text => /Tick at least one/.test(text)));
  manual.element("manual-preflight-use-servo_max").checked = true;
  manual.element("manual-preflight-val-servo_max").value = " ";
  await send();
  assert.deepEqual(manual.requests, [], "a ticked empty box is refused");
  manual.element("manual-preflight-val-servo_max").value = "0.97";
  manual.element("manual-preflight-use-steering_stick_scale").checked = true;
  manual.element("manual-preflight-val-steering_stick_scale").value = "27";
  await send();
  assert.equal(manual.requests.length, 1);
  const sent = manual.requests[0];
  assert.equal(sent.source, "manual");
  assert.deepEqual(Object.keys(sent.values).sort(), ["servo_max", "steering_stick_scale"]);
  assert.equal(sent.values.servo_max, 0.97);
  assert.ok(Math.abs(sent.values.steering_stick_scale - 27 * Math.PI / 180) < 1e-12, "degrees -> rad");
  assert.match(manual.confirms[0], /servo_max \(full-right end\): 0\.950 → 0\.97/);
  assert.match(manual.confirms[0], /Full stick steering: 25\.0 → 27/);
  const declined = browser({confirmAnswer: false});
  declined.snapshot({stage: "preflight"});
  declined.element("manual-preflight-use-servo_max").checked = true;
  declined.element("manual-preflight-val-servo_max").value = "0.97";
  await declined.sandbox.writeValueEditor("manual-preflight", "servo_max");
  assert.deepEqual(declined.requests, []);
  // Every test step and the report offer the same panel.
  const onStep = browser();
  onStep.snapshot();
  await onStep.sandbox.goToStep("steering_drift");
  assert.ok(onStep.view().includes('id="manual-step-val-servo_max"'));

  // --- editing a suggestion before writing ------------------------------------
  const report = {
    overall_status: "review", safety_note: "", current_parameters: parameters, movement: null,
    steering: {status: "good", trial_results: [], sides: {left: {}, right: {}}, warnings: [],
      curves: {}, checks: []},
    parameter_suggestions: {steering_angle_to_servo_gain: -0.79, servo_max: 0.95,
      steering_stick_scale: 0.436},
    blocked_suggestions: {}, not_ready_to_apply: {steering_angle_to_servo_gain: "confidence is low"},
  };
  const edit = browser();
  edit.snapshot({stage: "report", report, trials: [{id: "d", kind: "steering_drift", accepted: true},
    {id: "l", kind: "steering_left", accepted: true}, {id: "r", kind: "steering_right", accepted: true}]});
  html = edit.view();
  assert.ok(html.includes("Edit before writing"));
  assert.match(html, /id="report-steering-val-steering_angle_to_servo_gain"[^>]*value="-0.7900"/,
    "starts at the suggestion");
  assert.equal(isTicked(html, "report-steering-use-servo_max"), true);
  assert.equal(isTicked(html, "report-steering-use-steering_stick_scale"), true);
  assert.equal(isTicked(html, "report-steering-use-steering_angle_to_servo_gain"), false,
    "a not-ready value starts unticked");
  assert.equal(isTicked(html, "report-steering-use-servo_min"), false, "no suggestion, unticked");
  assert.ok(html.includes('id="manual-report-val-servo_max"'), "the hand panel is on the report too");

  // --- keeping a flagged run ----------------------------------------------------
  const pending = browser();
  pending.snapshot({pending_capture: {id: "p", kind: "steering_right",
    summary: {warnings: ["Only 4 servo samples were recorded"]}}});
  assert.ok(pending.view().includes('id="trial-override"'));
  pending.element("circle-inner").value = "1.33";
  pending.element("trial-confirmed").checked = true;
  pending.element("trial-override").checked = true;
  await pending.sandbox.acceptPending();
  assert.equal(pending.requests.length, 1);
  assert.equal(pending.requests[0].use_despite_warnings, true);
  const clean = browser();
  clean.snapshot({pending_capture: {id: "p", kind: "steering_right", summary: {warnings: []}}});
  assert.equal(clean.view().includes('id="trial-override"'), false);
  clean.element("circle-inner").value = "1.33";
  clean.element("trial-confirmed").checked = true;
  await clean.sandbox.acceptPending();
  assert.equal("use_despite_warnings" in clean.requests[0], false);

  const runs = browser();
  runs.snapshot({trials: [
    {id: "r1", kind: "steering_right", accepted: true, summary: {}},
    {id: "r2", kind: "steering_right", accepted: true, summary: {}, warnings_overridden: true},
    {id: "r3", kind: "steering_right", accepted: true, summary: {}},
  ]}, {steering_preview: {curves: {}, trials: {r1: {warnings: ["disagree by 12%"]},
    r2: {warnings: ["disagree by 12%"]}, r3: {warnings: []}}}});
  await runs.sandbox.goToStep("steering_right");
  html = runs.view();
  assert.ok(html.includes("setTrialOverride('r1', true)"));
  assert.ok(html.includes("setTrialOverride('r2', false)") && html.includes("kept anyway"));
  assert.equal(html.includes("setTrialOverride('r3'"), false, "a clean run has no toggle");
  await runs.sandbox.setTrialOverride("r1", true);
  assert.deepEqual(runs.requests, [{action: "set_trial_override", trial_id: "r1", value: true}]);

  // --- undo everything, rear track ---------------------------------------------
  const change = (id, restored = false) => ({apply_id: id, after: {servo_max: 1.0},
    before: {servo_max: 0.95}, backup: `/b/${id}`, restored});
  const one = browser();
  one.snapshot({applied_changes: [change("a")]});
  await one.sandbox.goToStep("steering_right");
  assert.ok(one.view().includes("restoreBackup()"));
  assert.equal(one.view().includes("restoreEverything()"), false, "one change: plain undo only");
  const two = browser();
  two.snapshot({applied_changes: [change("a"), change("b"), change("c", true)]});
  await two.sandbox.goToStep("steering_right");
  assert.ok(two.view().includes("restoreEverything()"));
  await two.sandbox.restoreEverything();
  assert.deepEqual(two.requests, [{action: "restore_backup", confirmed: true, everything: true}]);
  const undecided = browser({confirmAnswer: false});
  undecided.snapshot({applied_changes: [change("a"), change("b")]});
  await undecided.sandbox.restoreEverything();
  assert.deepEqual(undecided.requests, []);

  const track = browser();
  track.snapshot();
  await track.sandbox.goToStep("steering_left");
  assert.match(track.view(), /id="rear-track"[^>]*value="0.270"/);
  track.element("rear-track").value = "0.285";
  await track.sandbox.saveRearTrack();
  assert.deepEqual(track.requests, [{action: "update_vehicle", rear_track_m: 0.285}]);
  track.element("rear-track").value = "abc";
  await track.sandbox.saveRearTrack();
  assert.equal(track.requests.length, 1, "a non-number is not sent");

  console.log("Wizard override checks passed.");
})().catch(error => { console.error(error); process.exitCode = 1; });
