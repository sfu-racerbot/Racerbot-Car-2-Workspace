// Steering range check, blocked suggestions and vesc.yaml writes in the UI.
// Run through pytest's test_wizard_apply.py, or directly with node.
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

const source = fs.readFileSync(path.join(__dirname, "../web/wizard.js"), "utf8");
const parameters = {
  speed_to_erpm_gain: -4447.983786, speed_to_erpm_offset: 0,
  steering_angle_to_servo_gain: -1.2135,
  steering_angle_to_servo_offset: 0.5904, wheelbase: 0.36,
};

// Real wizard.js against a stub DOM; fetch and confirm are the boundaries.
function browser({confirmAnswer = true} = {}) {
  const elements = new Map();
  function element(id) {
    if (!elements.has(id)) elements.set(id, {
      innerHTML: "", dataset: {}, classList: {toggle() {}}, value: "",
      addEventListener() {}, appendChild() {}, remove() {}, setAttribute() {},
    });
    return elements.get(id);
  }
  const requests = [];
  const confirms = [];
  const sandbox = {
    document: {
      getElementById: element, createElement: () => element("toast"),
      querySelector: () => null, querySelectorAll: () => [],
    },
    location: {protocol: "http:", host: "localhost:8090"},
    WebSocket: class {close() {}},
    setTimeout() {}, clearTimeout() {}, setInterval() {},
    confirm: (question) => { confirms.push(question); return confirmAnswer; },
    fetch: async (url, options) => {
      if (url === "/api/state") return new Promise(() => {});
      requests.push(JSON.parse(options.body));
      return {ok: true, json: async () => ({ok: true})};
    },
  };
  sandbox.window = sandbox;
  vm.createContext(sandbox);
  vm.runInContext(source, sandbox);
  function snapshot(session, extra = {}) {
    sandbox.updateSnapshot({session: {session_id: "s1", mode: "steering", stage: "steering",
      current_parameters: {...parameters}, trials: [], ...session},
      telemetry: {}, health: {}, apply: {allowed: true, path: "/x/vesc.yaml"}, ...extra});
  }
  return {sandbox, element, requests, confirms, snapshot};
}

const done = kind => ({id: kind, kind, accepted: true, summary: {servo: {median: 0.59}}});

(async () => {
  // The range check's measuring screen sends what was typed, as numbers,
  // and leaves out boxes left empty.
  const sweep = browser();
  sweep.snapshot({pending_capture: {id: "p", kind: "steering_sweep", summary: {}}});
  const review = sweep.element("wizard-view").innerHTML;
  for (const id of ["sweep-ll-left", "sweep-ll-right", "sweep-rl-left", "sweep-rl-right"]) {
    assert.ok(review.includes(`id="${id}"`), `form must have ${id}`);
  }
  sweep.element("sweep-ll-left").value = "19.4";
  sweep.element("sweep-ll-right").value = "";
  sweep.element("sweep-rl-left").value = "12.1";
  sweep.element("sweep-rl-right").value = " 12.5 ";
  sweep.element("sweep-buzz-right").checked = true;
  sweep.element("trial-confirmed").checked = true;
  await sweep.sandbox.acceptPending();
  assert.equal(sweep.requests.length, 1);
  const sent = sweep.requests[0];
  assert.equal(sent.action, "accept_capture");
  assert.deepEqual(sent.left_lock, {left_wheel_deg: 19.4});
  assert.deepEqual(sent.right_lock, {left_wheel_deg: 12.1, right_wheel_deg: 12.5});
  assert.equal(sent.buzz_left, false);
  assert.equal(sent.buzz_right, true);

  // The diagnosis card names the cause and each layer.
  const chain = browser();
  chain.snapshot({trials: [done("steering_sweep")]}, {steering_chain: {
    verdict: "software", summary: "The difference comes from software settings.",
    layers: [{id: "clamp", label: "Servo clamp (servo_min / servo_max)", left_text: "0.1778",
      right_text: "0.8500", status: "limited", finding: "right is cut from 1.003 to 0.850 by servo_max"}],
    recommendations: ["re-centre the servo horn"]}});
  await chain.sandbox.goToStep("steering_sweep");
  const card = chain.element("wizard-view").innerHTML;
  assert.ok(card.includes("Cause: Software"));
  assert.ok(card.includes("right is cut from 1.003 to 0.850 by servo_max"));
  assert.ok(card.includes("re-centre the servo horn"));

  // A blocked suggestion shows its reason, is never written as a YAML value,
  // and is never offered for writing. A low-confidence one is shown but not
  // offered either.
  const report = {
    overall_status: "review", safety_note: "", current_parameters: parameters,
    movement: null,
    steering: {status: "invalid", trial_results: [], sides: {left: {}, right: {}}, warnings: []},
    parameter_suggestions: {steering_angle_to_servo_offset: 0.6},
    blocked_suggestions: {steering_angle_to_servo_gain: "the steering_angle_to_servo_gain fit is invalid"},
    not_ready_to_apply: {steering_angle_to_servo_offset: "confidence is low; add consistent runs first"},
  };
  const blocked = browser();
  blocked.snapshot({stage: "report", report, trials: [done("steering_sweep"),
    done("steering_drift"), done("steering_left"), done("steering_right")]});
  const page = blocked.element("wizard-view").innerHTML;
  assert.ok(page.includes("# steering_angle_to_servo_gain: blocked, the steering_angle_to_servo_gain fit is invalid"));
  assert.doesNotMatch(page, /\nsteering_angle_to_servo_gain: -?\d/);
  assert.doesNotMatch(page, /^steering_angle_to_servo_gain: -?\d/m);
  assert.ok(page.includes("Not ready to apply: confidence is low"));
  assert.equal(page.includes("applyReport("), false, "nothing here may be written");
  assert.ok(page.includes("Nothing here is ready to write to vesc.yaml yet."));

  // A ready suggestion is offered, and writing it sends exactly those names
  // after a confirmation.
  const ready = browser();
  ready.snapshot({stage: "report", report: {...report,
    steering: {...report.steering, status: "good"},
    parameter_suggestions: {steering_angle_to_servo_gain: -1.18, steering_angle_to_servo_offset: 0.6},
    blocked_suggestions: {}, not_ready_to_apply: {}},
    trials: [done("steering_sweep"), done("steering_drift"), done("steering_left"), done("steering_right")]});
  const offer = ready.element("wizard-view").innerHTML;
  const call = offer.match(/applyReport\('([^']*)'\)/);
  assert.ok(call, "a ready steering suggestion must be offered");
  await ready.sandbox.applyReport(call[1]);
  assert.deepEqual(ready.requests, [{action: "apply_parameters", confirmed: true, source: "report",
    names: ["steering_angle_to_servo_gain", "steering_angle_to_servo_offset"]}]);
  assert.match(ready.confirms[0], /backup is saved first/);

  // Declining the confirmation sends nothing.
  const declined = browser({confirmAnswer: false});
  declined.snapshot({});
  await declined.sandbox.applyReport("steering_angle_to_servo_offset");
  await declined.sandbox.applyCentreOffset();
  await declined.sandbox.restoreBackup();
  assert.deepEqual(declined.requests, []);

  // With writing switched off on the car, no write button appears at all.
  const off = browser();
  off.snapshot({stage: "report", report: {...report, parameter_suggestions: {steering_angle_to_servo_offset: 0.6},
    blocked_suggestions: {}, not_ready_to_apply: {}},
    trials: [done("steering_sweep"), done("steering_drift"), done("steering_left"), done("steering_right")]},
    {apply: {allowed: false}});
  assert.equal(off.element("wizard-view").innerHTML.includes("applyReport("), false);

  // The drive-straight step offers the centre offset, then reports done.
  const centre = browser();
  centre.snapshot({trials: [done("steering_sweep"), done("steering_drift")]}, {centre_offset: {
    suggested_offset: 0.5712, current_offset: 0.5904, drift_angle_rad: 0.0158,
    straight: false, blocked: null, run_count: 1}});
  await centre.sandbox.goToStep("steering_drift");
  const centrePage = centre.element("wizard-view").innerHTML;
  assert.ok(centrePage.includes("0.5712"));
  assert.ok(centrePage.includes("-0.0192"), "change from the current offset");
  assert.ok(centrePage.includes("applyCentreOffset()"));
  await centre.sandbox.applyCentreOffset();
  assert.deepEqual(centre.requests, [{action: "apply_parameters", confirmed: true, source: "centre"}]);
  const straight = browser();
  straight.snapshot({trials: [done("steering_sweep"), done("steering_drift")]}, {centre_offset: {
    suggested_offset: 0.5905, current_offset: 0.5904, drift_angle_rad: 0.0001,
    straight: true, blocked: null, run_count: 3}});
  await straight.sandbox.goToStep("steering_drift");
  assert.ok(straight.element("wizard-view").innerHTML.includes("The car drives straight"));
  assert.equal(straight.element("wizard-view").innerHTML.includes("applyCentreOffset()"), false);
  const refused = browser();
  refused.snapshot({trials: [done("steering_sweep"), done("steering_drift")]}, {centre_offset: {
    suggested_offset: 0.45, current_offset: 0.5904, drift_angle_rad: 0.1, straight: false,
    blocked: "it would move the offset by 0.140", run_count: 1}});
  await refused.sandbox.goToStep("steering_drift");
  assert.ok(refused.element("wizard-view").innerHTML.includes("No offset suggested: it would move the offset by 0.140"));
  assert.equal(refused.element("wizard-view").innerHTML.includes("applyCentreOffset()"), false);

  // After the car runs the new offset, the old run's suggestion is history:
  // the card asks for another run and offers no write.
  const stale = browser();
  stale.snapshot({trials: [done("steering_sweep"), done("steering_drift")]}, {centre_offset: {
    suggested_offset: null, current_offset: 0.5734, drift_angle_rad: null, straight: false,
    stale: true, blocked: "recorded with offset 0.5904", run_count: 1}});
  await stale.sandbox.goToStep("steering_drift");
  assert.ok(stale.element("wizard-view").innerHTML.includes("Record another drive-straight run"));
  assert.equal(stale.element("wizard-view").innerHTML.includes("applyCentreOffset()"), false);

  // After a write, every test page says bringup must restart until it has.
  const waiting = browser();
  waiting.snapshot({trials: [done("steering_sweep")], applied_changes: [{
    after: {steering_angle_to_servo_offset: 0.5712}, before: {steering_angle_to_servo_offset: 0.5904},
    backup: "/b/vesc.yaml.bak", restored: false, in_source_tree: true}]},
    {pending_restart: ["steering_angle_to_servo_offset"]});
  await waiting.sandbox.goToStep("steering_left");
  assert.ok(waiting.element("wizard-view").innerHTML.includes("Restart bringup."));
  const caughtUp = browser();
  caughtUp.snapshot({trials: [done("steering_sweep")]}, {pending_restart: []});
  await caughtUp.sandbox.goToStep("steering_left");
  assert.equal(caughtUp.element("wizard-view").innerHTML.includes("Restart bringup."), false);

  console.log("Wizard apply checks passed.");
})().catch(error => { console.error(error); process.exitCode = 1; });
