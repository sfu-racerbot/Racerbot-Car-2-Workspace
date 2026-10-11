// Every calibration test is its own sidebar step. Run through pytest's
// test_wizard_steps.py, or directly with node.
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

function trial(kind, extra = {}) {
  return {id: `${kind}-${Math.random()}`, kind, accepted: true,
    summary: {servo: {median: 0.53}}, ...extra};
}

function stepLabels(nav) {
  return [...nav.matchAll(/class="step-label">([^<]*)</g)].map(match => match[1]);
}

// The rendered sidebar button for one step, so a test can read its state.
function stepButton(nav, id) {
  const match = nav.match(new RegExp(`<button[^>]*data-step="${id}"[^>]*>`));
  assert.ok(match, `sidebar must contain step ${id}`);
  return match[0];
}

// Real wizard.js against a stub DOM; fetch is the only boundary mocked, and
// every request the page sends is recorded.
function browser() {
  const elements = new Map();
  function element(id) {
    if (!elements.has(id)) elements.set(id, {
      innerHTML: "", dataset: {}, classList: {toggle() {}},
      addEventListener() {}, appendChild() {}, remove() {}, setAttribute() {},
    });
    return elements.get(id);
  }
  const requests = [];
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
    confirm: () => true,
    fetch: async (url, options) => {
      if (url === "/api/state") return new Promise(() => {});
      requests.push(JSON.parse(options.body));
      return {ok: true, json: async () => ({ok: true})};
    },
  };
  sandbox.window = sandbox;
  vm.createContext(sandbox);
  vm.runInContext(source, sandbox);
  function snapshot(mode, extra = {}) {
    sandbox.updateSnapshot({session: {session_id: "s1", mode,
      stage: mode === "movement" ? "movement" : "steering",
      current_parameters: {...parameters}, trials: [], ...extra},
      telemetry: {}, health: {}});
  }
  return {sandbox, element, requests, snapshot};
}

(async () => {
  // Each test is its own sidebar step, in the order to do them, numbered 1..N.
  const expected = {
    steering: ["Check the car", "Steering range", "Drive straight", "Left circle",
      "Right circle", "Report"],
    movement: ["Check the car", "Stand still", "Distance runs", "Report"],
    movement_steering: ["Check the car", "Stand still", "Distance runs",
      "Steering range", "Drive straight", "Left circle", "Right circle", "Report"],
  };
  for (const [mode, labels] of Object.entries(expected)) {
    const page = browser();
    page.snapshot(mode);
    const nav = page.element("step-nav").innerHTML;
    assert.deepEqual(stepLabels(nav), labels, `sidebar steps for ${mode}`);
    assert.equal(nav.includes("segmented"), false, "no tabs inside a step");
  }

  // The report stays locked until the range check, drift AND both full-lock
  // circles exist. Each case removes one required test; the report must
  // refuse in every one.
  const full = [trial("steering_sweep"), trial("steering_drift"), trial("steering_left"),
    trial("steering_right")];
  for (const missing of ["steering_sweep", "steering_drift", "steering_left", "steering_right"]) {
    const page = browser();
    page.snapshot("steering", {trials: full.filter(t => t.kind !== missing)});
    const report = stepButton(page.element("step-nav").innerHTML, "report");
    assert.match(report, / disabled/, `report must be locked without ${missing}`);
    await page.sandbox.goToStep("report");
    await page.sandbox.generateReport();
    assert.deepEqual(page.requests, [], `no request may be sent without ${missing}`);
  }
  // A half-lock circle is an extra; it must not stand in for the full-lock test.
  const halfLock = browser();
  halfLock.snapshot("steering", {trials: [trial("steering_sweep"), trial("steering_drift"),
    trial("steering_left"), trial("steering_right", {full_lock: false})]});
  assert.match(stepButton(halfLock.element("step-nav").innerHTML, "report"), / disabled/);
  // With every test done, the report unlocks and generating sends the request.
  const complete = browser();
  complete.snapshot("steering", {trials: full});
  assert.doesNotMatch(stepButton(complete.element("step-nav").innerHTML, "report"), / disabled/);
  await complete.sandbox.goToStep("report");
  assert.deepEqual(complete.requests, [{action: "set_stage", stage: "report"}]);
  await complete.sandbox.generateReport();
  assert.equal(complete.requests.at(-1).action, "generate_report");
  // Wheels need the baseline and two distance runs; one run is one short.
  const oneRun = browser();
  oneRun.snapshot("movement", {trials: [trial("stationary"), trial("movement")]});
  assert.match(stepButton(oneRun.element("step-nav").innerHTML, "report"), / disabled/);
  const noBaseline = browser();
  noBaseline.snapshot("movement", {trials: [trial("movement"), trial("movement")]});
  assert.match(stepButton(noBaseline.element("step-nav").innerHTML, "report"), / disabled/);
  const twoRuns = browser();
  twoRuns.snapshot("movement", {trials: [trial("stationary"), trial("movement"), trial("movement")]});
  assert.doesNotMatch(stepButton(twoRuns.element("step-nav").innerHTML, "report"), / disabled/);

  // A restored steering session opens on its first unfinished test, and that
  // page's Record button records that test's kind and no other.
  const resumeCases = [
    [[], "steering_sweep", "Steering range"],
    [[trial("steering_sweep")], "steering_drift", "Drive straight"],
    [[trial("steering_sweep"), trial("steering_drift")], "steering_left", "Left circle"],
    [[trial("steering_sweep"), trial("steering_drift"), trial("steering_left")],
      "steering_right", "Right circle"],
  ];
  for (const [trials, kind, title] of resumeCases) {
    const page = browser();
    page.snapshot("steering", {trials});
    const view = page.element("wizard-view").innerHTML;
    assert.ok(view.includes(`<h1>${title}</h1>`), `resume must open ${title}`);
    assert.match(stepButton(page.element("step-nav").innerHTML, kind), /aria-current="step"/);
    const kinds = [...view.matchAll(/startCapture\('([a-z_]+)'\)/g)].map(m => m[1]);
    assert.deepEqual([...new Set(kinds)], [kind]);
    await page.sandbox.startCapture(kinds[0]);
    assert.deepEqual(page.requests, [{action: "start_capture", kind}]);
  }

  // Clicking a steering step switches the page with no server write: all
  // three share the server's single "steering" stage.
  const clicked = browser();
  clicked.snapshot("steering");
  await clicked.sandbox.goToStep("steering_right");
  assert.ok(clicked.element("wizard-view").innerHTML.includes("<h1>Right circle</h1>"));
  assert.deepEqual(clicked.requests, []);
  // A finished test offers the next one; an unfinished one does not.
  const finished = browser();
  finished.snapshot("steering", {trials: [trial("steering_drift")]});
  await finished.sandbox.goToStep("steering_drift");
  assert.ok(finished.element("wizard-view").innerHTML.includes("goToStep('steering_left')"));
  const unfinished = browser();
  unfinished.snapshot("steering");
  assert.equal(unfinished.element("wizard-view").innerHTML.includes("goToStep('steering_left')"), false);

  // The circle measuring screen defaults to the two-tire method and sends
  // exactly what was typed into boxes A and B.
  const circle = browser();
  circle.snapshot("steering", {pending_capture: {id: "p1", kind: "steering_left", summary: {}}});
  const review = circle.element("wizard-view").innerHTML;
  assert.match(review, /id="circle-axle-field"[^>]*hidden/, "axle method must start hidden");
  assert.doesNotMatch(review, /id="circle-tire-fields"[^>]*hidden/);
  circle.element("circle-inner").value = "2.41";
  circle.element("circle-outer").value = "2.97";
  circle.element("circle-fulllock").checked = true;
  circle.element("trial-confirmed").checked = true;
  await circle.sandbox.acceptPending();
  assert.equal(circle.requests.length, 1);
  assert.equal(circle.requests[0].measurement_method, "rear_tires");
  assert.equal(circle.requests[0].measured_inner_diameter_m, 2.41);
  assert.equal(circle.requests[0].measured_outer_diameter_m, 2.97);
  assert.equal(circle.requests[0].full_lock, true);
  // The "other way" link switches to the single axle-centre diameter.
  const axle = browser();
  axle.snapshot("steering", {pending_capture: {id: "p2", kind: "steering_right", summary: {}}});
  axle.sandbox.toggleCircleMethod();
  assert.equal(axle.element("circle-axle-field").hidden, false);
  assert.equal(axle.element("circle-tire-fields").hidden, true);
  axle.element("circle-diameter").value = "2.70";
  await axle.sandbox.acceptPending();
  assert.equal(axle.requests[0].measurement_method, "axle_centre");
  assert.equal(axle.requests[0].measured_diameter_m, 2.7);
  assert.equal("measured_inner_diameter_m" in axle.requests[0], false);

  console.log("Wizard step checks passed.");
})().catch(error => { console.error(error); process.exitCode = 1; });
