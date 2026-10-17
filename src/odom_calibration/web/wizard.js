"use strict";

const app = {
  snapshot: null,
  connected: false,
  socket: null,
  reconnectTimer: null,
  setupMode: "movement_steering",
  choosingMode: false,
  setupParameters: null,
  // The server has one "steering" stage; the page tracks which of its three
  // tests is open so each one can be its own sidebar step.
  steeringTest: "steering_drift",
  steeringSession: null,
  circleMethod: "inner_tire",
  halfLockNext: false,
  navHtml: null,
  // Servo-limit step size for "find how far this side steers".
  limitStep: 0.05,
};

const MODE_STAGES = {
  movement: ["preflight", "stationary", "movement", "report"],
  movement_steering: ["preflight", "stationary", "movement", "steering", "report"],
  steering: ["preflight", "steering", "report"],
};

const STEERING_STEPS = ["steering_drift", "steering_left", "steering_right"];

// Every step in the sidebar. A step's id equals the capture kind it records.
const STEPS = {
  preflight: {stage: "preflight", label: "Check the car"},
  stationary: {
    stage: "stationary", kind: "stationary", group: "Wheels", label: "Stand still", need: 1,
    finds: "The motor's reading when the car is perfectly still. The wizard subtracts it from every distance run.",
    phases: ["Hands off", "Record", "Wait 5 s", "Stop", "Accept"],
    steps: [
      "Put the car on the floor. Release LB and leave both sticks centred.",
      "Press <strong>Record</strong>.",
      "Wait 5 seconds. Do not touch the car or the remote.",
      "Press <strong>Stop</strong>, then <strong>Accept run</strong>. There is nothing to measure.",
    ],
    during: "Keep your hands off the car and the remote. Press Stop after 5 seconds.",
  },
  movement: {
    stage: "movement", kind: "movement", group: "Wheels", label: "Distance runs", need: 2, best: 3,
    finds: "How far the car really travels compared with how far it thinks it went. Fixes a car that counts 4.6 m for a 5 m lane.",
    phases: ["Mark start", "Record", "Drive", "Stop", "Measure"],
    steps: [
      "Lay a straight lane 5 to 10 m long and put a tape mark across the start.",
      "Park the car so the <strong>middle of the rear axle</strong> (the red dot in the picture, halfway between the rear wheels) is right over the start mark.",
      "Press <strong>Record</strong> before the car moves.",
      "Hold LB and drive slowly and smoothly straight down the lane.",
      "Stop near the end, release LB, then press <strong>Stop</strong>.",
      "Measure <strong>A</strong>: from the start mark to the middle of the rear axle.",
    ],
    tip: "Do three runs: two forward and one in reverse. For the reverse run, start at the far end and drive backwards. You pick Forward or Reverse when you enter the distance.",
    during: "Hold LB and drive smoothly down the lane. Stop, release LB, then press Stop.",
  },
  steering_drift: {
    stage: "steering", kind: "steering_drift", group: "Steering", label: "Drive straight", need: 1,
    finds: "The servo value where the car truly drives straight. Fixes a car that pulls to one side with the stick centred.",
    phases: ["Line up", "Record", "Drive", "Stop", "Measure"],
    steps: [
      "Stick a straight tape line at least 6 m long on the floor.",
      "Put the <strong>middle of the rear axle</strong> (red dot) on the start of the line, with the car pointing exactly along it. Take your time: 1° off adds about 9 cm of error over 5 m.",
      "Press <strong>Record</strong>.",
      "Hold LB and drive slowly about 5 m forward. <strong>Do not touch the steering stick.</strong>",
      "Stop, release LB, then press <strong>Stop</strong>.",
      "Measure <strong>A</strong>, how far along the line the rear axle got, and <strong>B</strong>, the sideways gap from the line to the rear axle. Note whether the car ended left or right of the line.",
    ],
    tip: "Ending a few centimetres to one side is normal. That drift is exactly what this test measures.",
    during: "Hold LB and drive about 5 m. Hands off the steering stick. Stop, release LB, then press Stop.",
  },
  steering_left: circleStep("left"),
  steering_right: circleStep("right"),
  report: {stage: "report", label: "Report"},
};

function circleStep(side) {
  return {
    stage: "steering", kind: `steering_${side}`, group: "Steering",
    label: `${side === "left" ? "Left" : "Right"} circle`, need: 1, side,
    finds: `How far the wheels really turn at full ${side} lock. This sets how sharp the car turns when a node asks for a ${side} turn.`,
    phases: ["Mark tire", "Record", "Half circle", "Stop", "Measure"],
    steps: [
      `Park in open space with about 4 m × 4 m clear, mostly to the car's ${side}.`,
      `Put a small tape mark on the floor <strong>right behind the middle of the ${side} rear tire</strong>, the one on the inside of the turn.`,
      "Press <strong>Record</strong>.",
      `Hold LB, push the steering stick <strong>fully ${side}</strong> and keep it there. Drive slowly (about walking pace) until the car has turned half a circle and points back the way it came.`,
      "Stop, release LB, then press <strong>Stop</strong>.",
      `Put a second mark behind the middle of the same tire. Measure <strong>A</strong>, straight across the circle between the two marks.`,
    ],
    tip: `To find how far the steering can go, drive more full-lock circles after raising servo_max (or lowering servo_min) a step at a time, checking wheels-up for binding first. The report finds where each step stops adding steering.`,
    during: `Hold LB with the stick fully ${side}. Stop once the car points back the way it came (about 0.50 turns below), release LB, then press Stop.`,
  };
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function finite(value) {
  return typeof value === "number" && Number.isFinite(value);
}

function fmt(value, digits = 2, fallback = "Not enough data") {
  return finite(value) ? value.toFixed(digits) : fallback;
}

function signed(value, digits = 3, fallback = "Not enough data") {
  return finite(value) ? `${value >= 0 ? "+" : ""}${value.toFixed(digits)}` : fallback;
}

function deg(value, digits = 1) {
  return finite(value) ? `${(value * 180 / Math.PI).toFixed(digits)}` : "Not enough data";
}

function toast(message, kind = "") {
  const region = document.getElementById("toast-region");
  const element = document.createElement("div");
  element.className = `toast ${kind}`;
  element.textContent = message;
  region.appendChild(element);
  setTimeout(() => element.remove(), 5000);
}

async function api(action, extra = {}) {
  const response = await fetch("/api/action", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({action, ...extra}),
  });
  const payload = await response.json();
  if (!response.ok || !payload.ok) {
    const error = new Error(payload.error || `Request failed (${response.status})`);
    error.details = payload.details || {};
    throw error;
  }
  if (payload.state) updateSnapshot(payload.state);
  return payload;
}

async function fetchState() {
  try {
    const response = await fetch("/api/state", {cache: "no-store"});
    if (!response.ok) throw new Error(`State request failed (${response.status})`);
    updateSnapshot(await response.json());
  } catch (error) {
    app.connected = false;
    renderConnection();
  }
}

function connectSocket() {
  if (app.socket) {
    try { app.socket.close(); } catch (_) {}
  }
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  const socket = new WebSocket(`${scheme}//${location.host}/ws`);
  app.socket = socket;
  socket.onopen = () => {
    app.connected = true;
    renderConnection();
  };
  socket.onmessage = (event) => {
    try {
      const message = JSON.parse(event.data);
      if (message.type === "snapshot") {
        const {type, ...snapshot} = message;
        updateSnapshot(snapshot);
      }
    } catch (_) {
      // A malformed telemetry frame should never break wizard controls.
    }
  };
  socket.onclose = () => {
    app.connected = false;
    renderConnection();
    clearTimeout(app.reconnectTimer);
    app.reconnectTimer = setTimeout(connectSocket, 1500);
  };
  socket.onerror = () => socket.close();
}

function updateSnapshot(snapshot) {
  const previousKey = renderKey(app.snapshot);
  app.snapshot = snapshot;
  app.connected = true;
  const session = snapshot.session;
  if (session && session.session_id !== app.steeringSession) {
    // A restored or new session opens on its first unfinished steering test.
    app.steeringSession = session.session_id;
    app.steeringTest = STEERING_STEPS.find(id => !stepDone(session, id)) || STEERING_STEPS[0];
  }
  renderConnection();
  renderNav();
  renderTelemetry();
  const nextKey = renderKey(snapshot);
  if (
    previousKey !== nextKey ||
    (snapshot.session?.stage === "preflight" && !app.choosingMode) ||
    !document.getElementById("wizard-view")?.dataset.rendered
  ) {
    renderWizardView();
  }
  const timer = document.getElementById("capture-timer");
  if (timer) timer.textContent = `${fmt(snapshot.capture_duration_sec, 1, "--")} s`;
  const dist = document.getElementById("capture-dist");
  if (dist) dist.textContent = `${fmt(snapshot.capture_progress?.odom_distance_m, 2, "--")} m`;
  const turns = document.getElementById("capture-turns");
  if (turns && finite(snapshot.capture_progress?.odom_yaw_rad)) {
    turns.textContent =
      `${(snapshot.capture_progress.odom_yaw_rad / (2 * Math.PI)).toFixed(2)} turns (odometry estimate)`;
  }
  updateDiagramLive();
}

function renderKey(snapshot) {
  if (!snapshot) return "loading";
  const session = snapshot.session;
  if (!session) {
    return `setup:${snapshot.live_parameter_status}:${JSON.stringify(snapshot.live_parameters)}`;
  }
  return [
    session.session_id,
    session.updated_at,
    session.stage,
    session.active_capture?.started_at || "",
    session.pending_capture?.id || "",
    (snapshot.pending_restart || []).join(","),
    snapshot.apply?.allowed ? "apply" : "",
  ].join(":");
}

function hasWheel(session) {
  return session.mode === "movement" || session.mode === "movement_steering";
}

function hasSteering(session) {
  return session.mode === "movement_steering" || session.mode === "steering";
}

function trialCounts(session) {
  const counts = {
    stationary: 0,
    movement: 0,
    steering_center: 0,
    steering_drift: 0,
    steering_left: 0,
    steering_right: 0,
  };
  for (const trial of session?.trials || []) {
    if (trial.accepted && Object.hasOwn(counts, trial.kind)) counts[trial.kind] += 1;
  }
  return counts;
}

function fullLockCounts(session) {
  const counts = {steering_drift: 0, steering_left: 0, steering_right: 0};
  for (const trial of session?.trials || []) {
    if (!trial.accepted || !(trial.kind in counts)) continue;
    if (trial.kind === "steering_drift" || trial.full_lock !== false) counts[trial.kind] += 1;
  }
  return counts;
}

function sessionSteps(session) {
  const ids = ["preflight"];
  if (hasWheel(session)) ids.push("stationary", "movement");
  if (hasSteering(session)) ids.push(...STEERING_STEPS);
  ids.push("report");
  return ids;
}

// Runs that count toward a step: half-lock circles are extras, not the test.
function stepRunCount(session, id) {
  if (id === "steering_left" || id === "steering_right") return fullLockCounts(session)[id];
  return trialCounts(session)[id] || 0;
}

function stepDone(session, id) {
  if (id === "preflight") return session.stage !== "preflight" || (session.trials || []).length > 0;
  if (id === "report") return Boolean(session.report);
  return stepRunCount(session, id) >= STEPS[id].need;
}

// Test steps still missing before the report may be generated.
function remainingSteps(session) {
  return sessionSteps(session).filter(id => STEPS[id].kind && !stepDone(session, id));
}

function reportLocked(session) {
  return remainingSteps(session).length > 0 && !session.report;
}

function currentStepId(session) {
  return session.stage === "steering" ? app.steeringTest : session.stage;
}

function groupPosition(session, id) {
  const group = STEPS[id].group;
  if (!group) return null;
  const members = sessionSteps(session).filter(item => STEPS[item].group === group);
  return {group, index: members.indexOf(id) + 1, total: members.length};
}

function nextStepId(session, id) {
  const ids = sessionSteps(session);
  const after = ids.slice(ids.indexOf(id) + 1);
  return after.find(item => !stepDone(session, item)) || "report";
}

function currentParams() {
  return app.snapshot?.session?.current_parameters || {};
}

function impliedSteeringAngle() {
  const params = currentParams();
  const telemetry = app.snapshot?.telemetry || {};
  const gain = params.steering_angle_to_servo_gain;
  const offset = params.steering_angle_to_servo_offset;
  if (!finite(telemetry.servo) || !finite(gain) || !finite(offset) || gain === 0) return null;
  return (telemetry.servo - offset) / gain;
}

function renderConnection() {
  const text = document.getElementById("connection-text");
  if (text) text.textContent = app.connected ? "Connected" : "Reconnecting…";
}

const CALIBRATION_MODES = [
  {value: "steering", label: "Steering calibration", short: "Steering", detail: "3 tests: drive straight, then one left and one right full-lock circle. Add more circles at other servo_max / servo_min values to find how far the steering really goes. Fixes a car that pulls to one side, turns unequally, or turns too sharply or too gently. About 15 minutes."},
  {value: "movement", label: "Wheel calibration", short: "Wheels", detail: "2 tests: stand still for 5 seconds, then 2 or 3 measured distance runs. Fixes a car that reports the wrong distance. About 15 minutes."},
  {value: "movement_steering", label: "Both calibrations", short: "Wheels + steering", detail: "All 5 tests in one session: the wheel tests first, then the steering tests. About 30 minutes."},
];

function stepStatus(session, id, active, locked) {
  if (locked) {
    const left = remainingSteps(session).length;
    return `Locked · ${left} test${left === 1 ? "" : "s"} left`;
  }
  const def = STEPS[id];
  const runs = def.kind ? stepRunCount(session, id) : 0;
  const runText = def.best ? `${runs} of ${def.best} runs` : `${runs} run${runs === 1 ? "" : "s"}`;
  if (stepDone(session, id)) return def.kind ? `Done · ${runText}` : "Done";
  if (active) return "You are here";
  if (def.best) return `Required · ${def.need} runs, ${def.best} best`;
  return def.kind ? "Required" : "";
}

function railButton(session, id, number, active, disabled, locked) {
  const def = STEPS[id];
  const done = stepDone(session, id);
  const position = groupPosition(session, id);
  return `<li><button class="step-button ${active ? "active" : ""} ${done ? "complete" : ""} ${locked ? "locked" : ""}" type="button"
      data-step="${id}" ${active ? 'aria-current="step"' : ""} ${disabled ? "disabled" : ""} onclick="goToStep('${id}')">
    <span class="step-number" aria-hidden="true">${done ? "✓" : locked ? "🔒" : number}</span>
    <span class="step-copy">${position ? `<span class="step-group">${escapeHtml(position.group)} ${position.index} of ${position.total}</span>` : ""}
      <span class="step-label">${escapeHtml(def.label)}</span>
      <span class="step-detail">${escapeHtml(stepStatus(session, id, active, locked))}</span></span>
  </button></li>`;
}

function renderNav() {
  const nav = document.getElementById("step-nav");
  const button = document.getElementById("new-session-button");
  const session = app.snapshot?.session;
  button.hidden = !session;
  button.disabled = Boolean(session?.active_capture) || app.choosingMode;
  button.textContent = "Change calibration";
  if (!nav) return;
  if (!session || app.choosingMode) {
    nav.innerHTML = `<div class="rail-heading">Calibration setup</div>
      <p class="rail-description">Choose steering, wheels, or both. Each test then appears here as its own step, in the order to do them.</p>`;
    app.navHtml = null;
    return;
  }
  const busy = Boolean(session.active_capture || session.pending_capture);
  const current = currentStepId(session);
  const mode = CALIBRATION_MODES.find(item => item.value === session.mode);
  const html = `<div class="rail-session"><span class="rail-caption">Selected calibration</span>
      <strong>${escapeHtml(mode?.short || session.mode)}</strong>
      <button class="rail-change" type="button" onclick="openCalibrationSetup()" ${session.active_capture ? "disabled" : ""}>Change calibration</button>
    </div><div class="rail-heading">Do every step, top to bottom</div>
    <ol class="step-list">${sessionSteps(session).map((id, index) => {
      const locked = id === "report" && reportLocked(session);
      return railButton(session, id, index + 1, current === id, busy || locked, locked);
    }).join("")}</ol>
    ${session.active_capture ? '<p class="rail-description">Stop recording before changing steps or calibration.</p>'
      : session.pending_capture ? '<p class="rail-description">Accept or discard this run before changing steps.</p>' : ""}`;
  // Telemetry arrives many times a second; only rebuild when the steps change,
  // so the phone's sideways step list keeps its scroll position.
  if (html === app.navHtml) return;
  app.navHtml = html;
  nav.innerHTML = html;
  const list = typeof nav.querySelector === "function" ? nav.querySelector(".step-list") : null;
  const active = list?.querySelector('[aria-current="step"]')?.parentElement;
  if (list && active) list.scrollLeft = Math.max(0, active.offsetLeft - list.offsetLeft - 16);
}

function renderTelemetry() {
  const snapshot = app.snapshot;
  if (!snapshot) return;
  const telemetry = snapshot.telemetry || {};
  const health = snapshot.health || {};
  const lbHeld = Boolean(telemetry.lb_held);
  const lbText = lbHeld ? "Held" : "Released";
  const speedText = finite(telemetry.odom_speed) ? `${signed(telemetry.odom_speed, 2)} m/s` : "--";
  const servoText = finite(telemetry.servo) ? telemetry.servo.toFixed(4) : "--";
  const angle = impliedSteeringAngle();
  const steerText = angle === null ? "--"
    : `${(Math.abs(angle) * 180 / Math.PI).toFixed(1)}° ${angle >= 0 ? "left" : "right"}`;
  const erpmText = finite(telemetry.raw_forward_erpm) ? signed(telemetry.raw_forward_erpm, 0) : "--";

  const set = (id, value) => { const el = document.getElementById(id); if (el) el.textContent = value; };
  const stripLb = document.getElementById("live-strip-lb");
  if (stripLb) {
    stripLb.textContent = lbHeld ? "LB held" : "LB released";
    stripLb.classList.toggle("held", lbHeld);
  }
  set("lb-state", lbText);
  set("live-speed", speedText);
  set("live-servo", servoText);
  set("live-steer", steerText);
  set("live-erpm", erpmText);
  set("live-strip-speed", finite(telemetry.odom_speed) ? `${signed(telemetry.odom_speed, 2)} m/s` : "--");
  set("live-strip-servo", finite(telemetry.servo) ? telemetry.servo.toFixed(4) : "--");

  const list = document.getElementById("topic-health");
  if (list) {
    list.innerHTML = Object.entries(health).map(([name, item]) => {
      const rate = finite(item.rate_hz) ? `${item.rate_hz.toFixed(1)} Hz` : "no rate";
      const age = finite(item.age_sec) ? `${item.age_sec.toFixed(2)} s old` : "no data";
      const required = name === "odom" ? "required" : "optional";
      return `<li><span class="topic-square ${escapeHtml(item.status)}"></span>
        <span class="topic-text"><span class="topic-name">${escapeHtml(item.label || name)}</span>
        <span class="topic-detail"><span class="mono">${escapeHtml(rate)}</span>, <span class="mono">${escapeHtml(age)}</span>, ${required}</span></span></li>`;
    }).join("");
  }
}

function eyebrow(text) {
  return `<p class="eyebrow">${escapeHtml(text)}</p>`;
}

function sectorRule() {
  return `<div class="sector-rule" aria-hidden="true"><span></span><span></span><span></span></div>`;
}

function pageHead(brow, title) {
  return `${eyebrow(brow)}<h1>${escapeHtml(title)}</h1>${sectorRule()}`;
}

function parameterInputs(parameters) {
  const fields = [
    ["speed_to_erpm_gain", "Odometry ERPM gain (signed)", "1"],
    ["speed_to_erpm_offset", "Odometry ERPM offset", "0.1"],
    ["steering_angle_to_servo_gain", "Steering to servo gain", "0.0001"],
    ["steering_angle_to_servo_offset", "Steering to servo offset", "0.0001"],
    ["wheelbase", "Wheelbase (m)", "0.001"],
  ];
  return fields.map(([name, label, step]) => `
    <label class="field">
      <span class="field-label">${label}</span>
      <input class="input parameter-input" type="number" step="${step}"
             id="param-${name}" value="${escapeHtml(parameters[name])}">
    </label>`).join("");
}

function currentParameterValues() {
  const values = {};
  for (const input of document.querySelectorAll(".parameter-input")) {
    values[input.id.replace("param-", "")] = Number(input.value);
  }
  return values;
}

function selectedMode() {
  return app.setupMode;
}

function renderSetup() {
  const live = app.setupParameters || app.snapshot?.live_parameters || {
    speed_to_erpm_gain: -4614,
    speed_to_erpm_offset: 0,
    steering_angle_to_servo_gain: -1.2135,
    steering_angle_to_servo_offset: 0.5904,
    wheelbase: 0.36,
  };
  const mode = selectedMode();
  return `<section>
    <h1>Calibration</h1>${sectorRule()}
    <p class="lead">Pick what to calibrate, check the current values, and start. You drive the car with the remote; this page only records.</p>
    ${app.snapshot?.session ? '<p class="tip">Starting a new calibration archives your current session. Cancel to keep working on it.</p>' : ""}
    <fieldset class="mode-picker"><legend>Calibration type</legend>
      ${CALIBRATION_MODES.map(item => `<label class="mode-option ${mode === item.value ? "selected" : ""}">
        <input type="radio" name="calibration-mode" value="${item.value}" ${mode === item.value ? "checked" : ""} onchange="selectCalibrationMode(this.value)">
        <span><strong>${item.label}</strong><span class="mode-detail">${item.detail}</span></span>
      </label>`).join("")}
    </fieldset>
    <p class="tip"><strong>You will need:</strong> a tape measure, a roll of masking tape, about 10 m of smooth open floor, and a spotter.</p>
    <h2>Current values</h2>
    <p class="muted small">Values were ${escapeHtml(app.snapshot?.live_parameter_status || "loaded from defaults")}.</p>
    <div class="form-grid">${parameterInputs(live)}</div>
    <div class="button-row">
      <button class="button" type="button" ${mode ? "" : "disabled"} onclick="createSession()">Start calibration</button>
      ${app.snapshot?.session ? '<button class="button button-ghost" type="button" onclick="cancelCalibrationSetup()">Cancel</button>' : ""}
      ${mode ? "" : `<span class="small muted">Pick at least one calibration.</span>`}
    </div>
  </section>`;
}

function topicRow(name, item, required) {
  const rate = finite(item?.rate_hz) ? `${item.rate_hz.toFixed(1)} Hz` : "no rate";
  const age = finite(item?.age_sec) ? `${item.age_sec.toFixed(2)} s old` : "no data";
  return `<li><span class="topic-square ${escapeHtml(item?.status || "missing")}"></span>
    <span class="topic-text"><span class="topic-name">${escapeHtml(item?.label || name)}</span>
    <span class="topic-detail"><span class="mono">${escapeHtml(rate)}</span>, <span class="mono">${escapeHtml(age)}</span>, ${required ? "required" : "optional"}</span></span></li>`;
}

function renderPreflight(session) {
  const health = app.snapshot.health || {};
  const odomReady = ["good", "warning"].includes(health?.odom?.status);
  const next = sessionSteps(session)[1];
  return `<section>
    ${pageHead("Preflight", "Check the car")}
    <p class="lead">Make sure the car is talking to the wizard before you start testing.</p>
    <ol class="instruction-list">
      <li>Put the car on level ground with clear space around it.</li>
      <li>Turn on the remote and start bringup. Leave the sticks centred.</li>
      <li>Wait for <strong>Odometry</strong> below to turn green. If it stays missing, hold LB for a moment with the sticks centred.</li>
    </ol>
    <ul class="topic-health">
      ${topicRow("odom", health.odom, true)}
      ${topicRow("vesc", health.vesc, false)}
      ${topicRow("servo", health.servo, false)}
      ${topicRow("servo_cmd", health.servo_cmd, false)}
      ${topicRow("drive", health.drive, false)}
      ${topicRow("joy", health.joy, false)}
    </ul>
    <div class="button-row">
      <button class="button" type="button" ${odomReady ? "" : "disabled"} onclick="goToStep('${next}')">Start step 2: ${escapeHtml(STEPS[next].label)}</button>
      ${odomReady ? "" : `<span class="small muted">Waiting for odometry</span>`}
    </div>
  </section>
  ${manualPanel("preflight")}`;
}

// Every page offers the hand-edit table, folded away until wanted.
function manualPanel(where) {
  if (!applyAllowed()) return "";
  return `<section><details class="details"><summary>Change a setting by hand</summary>
    <p>Tick a setting, type the new value and press write. Use it to try your own number or put back a known-good one. It goes straight into the car's files.</p>
    ${renderValueEditor(`manual-${where}`, Object.keys(EDITABLE))}</details></section>`;
}

function kindLabel(kind) {
  if (kind === "stationary") return "Stand still";
  if (kind === "movement") return "Distance run";
  if (kind === "steering_drift") return "Drive straight";
  if (kind === "steering_left") return "Left circle";
  if (kind === "steering_right") return "Right circle";
  if (kind === "steering_center") return "Centre";
  return String(kind).replaceAll("_", " ");
}

function trialMeasurement(trial) {
  if (trial.kind === "movement" && finite(trial.measured_distance_m)) {
    return `${escapeHtml(trial.direction || "")} ${trial.measured_distance_m.toFixed(3)} m`;
  }
  if ((trial.kind === "steering_left" || trial.kind === "steering_right")) {
    const lock = trial.full_lock === false ? ", half lock (extra)" : "";
    if (trial.measurement_method === "inner_tire" && finite(trial.measured_inner_diameter_m)) {
      return `inner tire ${trial.measured_inner_diameter_m.toFixed(2)} m${lock}`;
    }
    if (trial.measurement_method === "rear_tires"
        && finite(trial.measured_inner_diameter_m) && finite(trial.measured_outer_diameter_m)) {
      return `tires ${trial.measured_inner_diameter_m.toFixed(2)} and ${trial.measured_outer_diameter_m.toFixed(2)} m${lock}`;
    }
    if (finite(trial.measured_diameter_m)) return `axle ${trial.measured_diameter_m.toFixed(3)} m${lock}`;
  }
  if (trial.kind === "steering_drift" && finite(trial.measured_forward_m)) {
    const lateral = trial.measured_lateral_m;
    const side = !finite(lateral) || lateral === 0 ? "on the line"
      : lateral > 0 ? `${lateral.toFixed(2)} m left` : `${(-lateral).toFixed(2)} m right`;
    return `${trial.measured_forward_m.toFixed(2)} m along, ${side}`;
  }
  return "No tape measure";
}

// A run the fit flags (too few samples, a disagreeing repeat) can be kept
// anyway; a kept run says so and can be switched back.
function overrideToggle(trial) {
  const result = app.snapshot?.steering_preview?.trials?.[trial.id];
  const kept = Boolean(trial.warnings_overridden);
  if (!kept && !(result?.warnings?.length || trial.summary?.warnings?.length)) return "";
  return `${kept ? `<span class="layer-status ok">kept anyway</span>` : ""}<button class="button button-ghost button-small" type="button"
    onclick="setTrialOverride('${escapeHtml(trial.id)}', ${!kept})">${kept ? "Stop keeping" : "Use anyway"}</button>`;
}

function acceptedRunsTable(session, kinds) {
  const trials = (session.trials || []).filter(t => t.accepted && kinds.includes(t.kind));
  if (!trials.length) return `<p class="small muted">No accepted runs yet.</p>`;
  const table = `<div class="runs-table"><table class="data-table">
    <thead><tr><th>#</th><th>Test</th><th>Measurement</th><th>Servo</th><th>Odom</th><th></th></tr></thead>
    <tbody>${trials.map((trial, index) => `<tr>
      <td><span class="race-plate">${index + 1}</span></td>
      <td>${escapeHtml(kindLabel(trial.kind))}</td>
      <td>${trialMeasurement(trial)}</td>
      <td class="num">${fmt(trial.summary?.servo?.median, 4)}</td>
      <td class="num">${signed(trial.summary?.odom_distance_m, 3)} m</td>
      <td>${overrideToggle(trial)}<button class="button button-ghost button-small" type="button"
        onclick="deleteTrial('${escapeHtml(trial.id)}')">Remove</button></td>
    </tr>`).join("")}</tbody>
  </table></div>`;
  const cards = `<ul class="run-cards">${trials.map((trial, index) => `<li class="run-card">
      <span class="race-plate">${index + 1}</span>
      <strong>${escapeHtml(kindLabel(trial.kind))}</strong>
      <span class="run-measure">${trialMeasurement(trial)}</span>
      <span class="run-meta">Servo ${fmt(trial.summary?.servo?.median, 4, "--")}, odom ${signed(trial.summary?.odom_distance_m, 3, "--")} m</span>
      ${overrideToggle(trial)}<button class="button button-ghost button-small" type="button"
        onclick="deleteTrial('${escapeHtml(trial.id)}')">Remove</button>
    </li>`).join("")}</ul>`;
  return table + cards;
}

// "Set up → Record → Drive → Stop → Measure" with the current action lit.
function phaseStrip(phases, current) {
  return `<ol class="phase-strip" aria-label="What you will do">${phases.map((phase, index) => {
    const state = index < current ? "done" : index === current ? "current" : "";
    return `<li class="${state}" ${index === current ? 'aria-current="step"' : ""}>
      <span class="phase-dot" aria-hidden="true">${index < current ? "✓" : index + 1}</span>${escapeHtml(phase)}</li>`;
  }).join("")}</ol>`;
}

/* ---------- Diagrams ----------
 * Top-down drawings in SVG user units. The car is drawn pointing up with its
 * rear-axle centre (the red dot, the point every measurement uses) at 0,0.
 */

function carSvg(x, y, rotation, options = {}) {
  const steer = options.steer || 0;
  const tire = (cx, cy) => `<rect class="dg-tire" x="${cx - 3}" y="${cy - 7}" width="6" height="14" rx="1.5"/>`;
  return `<g class="${options.ghost ? "dg-ghost" : ""}" transform="translate(${x} ${y}) rotate(${rotation})">
    <rect class="dg-body" x="-11" y="-46" width="22" height="54" rx="5"/>
    <line class="dg-axle-line" x1="-14" y1="0" x2="14" y2="0"/>
    ${tire(-14, 0)}${tire(14, 0)}
    <g transform="rotate(${steer} -14 -36)">${tire(-14, -36)}</g>
    <g transform="rotate(${steer} 14 -36)">${tire(14, -36)}</g>
    <path class="dg-nose" d="M -6 -32 L 0 -40 L 6 -32"/>
    <circle class="dg-axle" cx="0" cy="0" r="3.2"/>
  </g>`;
}

function arrowHead(x, y, ux, uy) {
  const bx = x - ux * 8;
  const by = y - uy * 8;
  return `<polygon class="dg-arrow" points="${x.toFixed(1)},${y.toFixed(1)} ${(bx - uy * 4).toFixed(1)},${(by + ux * 4).toFixed(1)} ${(bx + uy * 4).toFixed(1)},${(by - ux * 4).toFixed(1)}"/>`;
}

function badge(x, y, letter) {
  return `<g class="dg-badge"><circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="10"/>
    <text x="${x.toFixed(1)}" y="${(y + 4).toFixed(1)}" text-anchor="middle">${letter}</text></g>`;
}

// A measured distance: line with arrowheads at both ends and a letter badge.
function dimension(x1, y1, x2, y2, letter, badgeAt = 0.5) {
  const length = Math.hypot(x2 - x1, y2 - y1) || 1;
  const ux = (x2 - x1) / length;
  const uy = (y2 - y1) / length;
  return `<line class="dg-dim" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}"/>
    ${arrowHead(x2, y2, ux, uy)}${arrowHead(x1, y1, -ux, -uy)}
    ${badge(x1 + (x2 - x1) * badgeAt, y1 + (y2 - y1) * badgeAt, letter)}`;
}

function extension(x1, y1, x2, y2) {
  return `<line class="dg-ext" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}"/>`;
}

function label(x, y, text, anchor = "start", strong = false) {
  return `<text class="${strong ? "dg-label-strong" : "dg-label"}" x="${x}" y="${y}" text-anchor="${anchor}">${escapeHtml(text)}</text>`;
}

function figure(svg, caption, viewBox, aria, extraClass = "") {
  return `<figure class="figure step-figure ${extraClass}"><svg class="diagram step-diagram" viewBox="${viewBox}" role="img" aria-label="${escapeHtml(aria)}">${svg}</svg>
    <figcaption>${caption}</figcaption></figure>`;
}

const AXLE_KEY = `<span class="key-dot" aria-hidden="true"></span> red dot = middle of the rear axle`;

function stationaryDiagram() {
  const svg = `${carSvg(90, 110, 0)}
    ${label(90, 140, "car on the floor", "middle")}
    <circle class="dg-watch" cx="235" cy="78" r="40"/>
    <rect class="dg-watch-button" x="229" y="30" width="12" height="7"/>
    <path class="dg-watch-fill" d="M 235 46 A 32 32 0 1 1 234.9 46"/>
    <text class="dg-big" x="235" y="86" text-anchor="middle">5 s</text>
    ${label(235, 140, "hands off car and remote", "middle")}`;
  return figure(svg, "Release LB, leave the sticks centred, and let the car sit completely still for 5 seconds.",
    "0 0 330 150", "Car standing still for five seconds", "step-figure-small");
}

function distanceDiagram(measure) {
  const svg = `<line class="dg-lane" x1="20" y1="45" x2="450" y2="45"/>
    <line class="dg-lane" x1="20" y1="115" x2="450" y2="115"/>
    <line class="dg-tape" x1="60" y1="55" x2="60" y2="105"/>
    ${label(60, 34, "start mark", "middle", true)}
    ${carSvg(60, 80, 90, {ghost: measure})}
    <line class="dg-path" x1="110" y1="80" x2="330" y2="80"/>${arrowHead(330, 80, 1, 0)}
    ${carSvg(380, 80, 90, {ghost: !measure})}
    ${label(380, 34, "stop anywhere near the end", "middle")}
    ${extension(60, 105, 60, 140)}${extension(380, 84, 380, 140)}
    ${dimension(60, 132, 380, 132, "A")}
    ${label(240, 158, measure ? "A = start mark to the red dot" : "A: 5 to 10 m, drive straight", "middle")}`;
  return figure(svg, `${measure ? "Measure <strong>A</strong> along the lane, from the start mark to where the red dot stopped." : "Start with the red dot on the start mark. Drive straight down the lane."} ${AXLE_KEY}.`,
    "0 0 470 170", "Distance run along a taped lane");
}

function driftDiagram(measure) {
  const endX = 380;
  const endY = 46;
  const svg = `<line class="dg-tape" x1="20" y1="80" x2="465" y2="80"/>
    <line class="dg-tape" x1="60" y1="62" x2="60" y2="98"/>
    ${label(160, 102, "tape line", "middle", true)}
    ${carSvg(60, 80, 90, {ghost: measure})}
    <path class="dg-path" d="M 60 80 Q ${(60 + endX) / 2} 80 ${endX} ${endY}"/>
    ${arrowHead(endX, endY, 0.99, -0.12)}
    ${label(220, 50, "about 5 m, steering stick untouched", "middle")}
    ${carSvg(endX, endY, 84, {ghost: !measure})}
    ${measure ? `${extension(60, 98, 60, 140)}${extension(endX, 80, endX, 140)}
      ${dimension(60, 132, endX, 132, "A")}
      ${extension(endX, endY, 448, endY)}${dimension(440, 80, 440, endY, "B")}
      ${label(472, 22, "here B is left of the line", "end")}` : ""}`;
  return figure(svg, measure
    ? `<strong>A</strong> = how far along the line the red dot got. <strong>B</strong> = sideways gap from the line to the red dot. ${AXLE_KEY}.`
    : `Start with the red dot on the line and the car pointing exactly along it. A small drift to one side is expected. ${AXLE_KEY}.`,
    measure ? "0 0 480 160" : "0 0 480 112", "Straight-line drift test");
}

function circleDiagram(side, measure, method = "inner_tire") {
  const s = side === "left" ? 1 : -1;
  const cx = 220;
  const cy = 168;
  const R = 118;
  const half = 14; // rear tire offset from the axle centre
  const startX = cx + s * R;
  const endX = cx - s * R;
  const sweep = s > 0 ? 0 : 1;
  const arc = r => `M ${cx + s * r} ${cy} A ${r} ${r} 0 0 ${sweep} ${cx - s * r} ${cy}`;
  const mark = (x, behind) => `<rect class="dg-mark" x="${x - 5}" y="${behind > 0 ? cy + 9 : cy - 14}" width="10" height="5"/>`;
  let marks = "";
  let dims = "";
  if (method === "inner_tire") {
    marks = mark(cx + s * (R - half), 1) + mark(cx - s * (R - half), -1);
    if (measure) {
      dims = `${extension(cx - (R - half), cy - 9, cx - (R - half), cy + 46)}${extension(cx + (R - half), cy + 14, cx + (R - half), cy + 46)}
        ${dimension(cx - (R - half), cy + 40, cx + (R - half), cy + 40, "A", 0.3)}`;
    }
  } else if (method === "rear_tires") {
    for (const r of [R - half, R + half]) {
      marks += mark(cx + s * r, 1) + mark(cx - s * r, -1);
    }
    if (measure) {
      dims = `${extension(cx - (R - half), cy - 9, cx - (R - half), cy + 46)}${extension(cx + (R - half), cy + 14, cx + (R - half), cy + 46)}
        ${extension(cx - (R + half), cy - 9, cx - (R + half), cy + 68)}${extension(cx + (R + half), cy + 14, cx + (R + half), cy + 68)}
        ${dimension(cx - (R - half), cy + 40, cx + (R - half), cy + 40, "A", 0.3)}
        ${dimension(cx - (R + half), cy + 62, cx + (R + half), cy + 62, "B", 0.7)}`;
    }
  } else {
    marks = `<path class="dg-xmark" d="M ${startX - 5} ${cy - 5} L ${startX + 5} ${cy + 5} M ${startX + 5} ${cy - 5} L ${startX - 5} ${cy + 5}"/>
      <path class="dg-xmark" d="M ${endX - 5} ${cy - 5} L ${endX + 5} ${cy + 5} M ${endX + 5} ${cy - 5} L ${endX - 5} ${cy + 5}"/>`;
    if (measure) {
      dims = `${extension(cx - R, cy, cx - R, cy + 58)}${extension(cx + R, cy, cx + R, cy + 58)}
        ${dimension(cx - R, cy + 52, cx + R, cy + 52, "C")}`;
    }
  }
  const svg = `<path class="dg-track" d="${arc(R - half)}"/><path class="dg-track" d="${arc(R + half)}"/>
    <path class="dg-path" d="${arc(R)}"/>${arrowHead(cx - s * 4, cy - R, -s, 0)}
    ${label(cx, cy - R - 26, `stick fully ${side}, drive slowly`, "middle")}
    <path class="dg-centre" d="M ${cx - 5} ${cy} L ${cx + 5} ${cy} M ${cx} ${cy - 5} L ${cx} ${cy + 5}"/>
    ${label(cx, cy - 10, "circle centre", "middle")}
    ${carSvg(startX, cy, 0, {ghost: measure, steer: -s * 20})}
    ${carSvg(endX, cy, 180, {ghost: !measure, steer: -s * 20})}
    ${marks}
    ${label(cx + s * (R - 26), cy - 28, "Start", s > 0 ? "end" : "start", true)}
    ${label(cx - s * (R - 26), cy - 28, "Stop: facing back", s > 0 ? "start" : "end", true)}
    ${dims}`;
  let caption;
  if (method === "inner_tire") {
    caption = measure
      ? `Measure straight across the circle between the two marks behind the inner (${side}) rear tire. That is <strong>A</strong>. Orange = your tape marks.`
      : `Mark behind the inner (${side}) rear tire, drive half a circle at full ${side} lock, then mark behind it again. Orange = your tape marks.`;
  } else if (method === "rear_tires") {
    caption = measure
      ? `Measure straight across the circle between matching marks. <strong>A</strong> = inner (${side}) tire marks. <strong>B</strong> = outer tire marks. Orange = your tape marks.`
      : `Mark behind both rear tires, drive half a circle at full ${side} lock, then mark behind them again. Orange = your tape marks.`;
  } else {
    caption = `Mark the floor under the red dot at the start and after the half circle. <strong>C</strong> = distance between the two marks. ${AXLE_KEY}.`;
  }
  return figure(svg, caption, "0 0 440 258", `${side} full-lock half circle`);
}

function setupDiagram(id) {
  if (id === "stationary") return stationaryDiagram();
  if (id === "movement") return distanceDiagram(false);
  if (id === "steering_drift") return driftDiagram(false);
  if (id === "steering_left") return circleDiagram("left", false);
  if (id === "steering_right") return circleDiagram("right", false);
  return "";
}

function rearTrack(session) {
  const track = session?.vehicle?.rear_track_m;
  return finite(track) && track > 0 ? track : 0.27;
}

function clientTrialPoints(session, wheelbase) {
  const points = [];
  const track = rearTrack(session);
  for (const trial of session?.trials || []) {
    if (!trial.accepted) continue;
    const servo = trial.summary?.servo?.median;
    if (!finite(servo) || !finite(wheelbase) || wheelbase <= 0) continue;
    let angle = null;
    if (trial.kind === "steering_center") {
      angle = 0;
    } else if (trial.kind === "steering_drift") {
      const s = trial.measured_forward_m;
      const d = trial.measured_lateral_m;
      if (!finite(s) || s <= 0 || !finite(d)) continue;
      angle = Math.atan(wheelbase * 2 * d / (s * s + d * d));
    } else if (trial.kind === "steering_left" || trial.kind === "steering_right") {
      let radius = null;
      if (trial.measurement_method === "inner_tire" && finite(trial.measured_inner_diameter_m)) {
        radius = (trial.measured_inner_diameter_m + track) / 2;
      } else if (trial.measurement_method === "rear_tires"
          && finite(trial.measured_inner_diameter_m) && finite(trial.measured_outer_diameter_m)) {
        radius = (trial.measured_inner_diameter_m + trial.measured_outer_diameter_m) / 4;
      } else if (finite(trial.measured_diameter_m)) {
        radius = trial.measured_diameter_m / 2;
      }
      if (!finite(radius) || radius <= 0) continue;
      const magnitude = Math.atan(wheelbase / radius);
      angle = trial.kind === "steering_left" ? magnitude : -magnitude;
    } else {
      continue;
    }
    points.push({x: angle, y: servo, kind: trial.kind});
  }
  return points;
}

function linearFit(points) {
  if (points.length < 2) return null;
  const meanX = points.reduce((sum, p) => sum + p.x, 0) / points.length;
  const meanY = points.reduce((sum, p) => sum + p.y, 0) / points.length;
  const denominator = points.reduce((sum, p) => sum + (p.x - meanX) ** 2, 0);
  if (denominator <= 1e-9) return null;
  const slope = points.reduce((sum, p) => sum + (p.x - meanX) * (p.y - meanY), 0) / denominator;
  const intercept = meanY - slope * meanX;
  const rmse = Math.sqrt(
    points.reduce((sum, p) => sum + (p.y - (slope * p.x + intercept)) ** 2, 0) / points.length);
  return {slope, intercept, rmse};
}

function servoLimits() {
  const vehicle = app.snapshot?.session?.vehicle || {};
  if (finite(vehicle.servo_min) && finite(vehicle.servo_max) && vehicle.servo_min < vehicle.servo_max) {
    return [vehicle.servo_min, vehicle.servo_max];
  }
  return null;
}

function roundDegStep(span) {
  return span / 4 <= 5 ? 5 : 10;
}

function renderFitChart(points, line, limits) {
  if (!points.length) return "";
  const W = 480, H = 270, L = 52, R = 14, T = 14, B = 44;
  const xs = points.map(p => p.x * 180 / Math.PI);
  const ys = points.map(p => p.y);
  const xStep = roundDegStep(Math.max(...xs) - Math.min(...xs));
  let x0 = Math.floor(Math.min(...xs) / xStep) * xStep;
  let x1 = Math.ceil(Math.max(...xs) / xStep) * xStep;
  if (x1 <= x0) { x0 -= xStep; x1 += xStep; }
  let y0, y1;
  if (limits) {
    y0 = Math.max(0, Math.min(limits[0], limits[1], ...ys) - 0.05);
    y1 = Math.min(1, Math.max(limits[0], limits[1], ...ys) + 0.05);
  } else {
    y0 = Math.min(...ys) - 0.1;
    y1 = Math.max(...ys) + 0.1;
  }
  if (!(y1 > y0)) { y0 -= 0.1; y1 += 0.1; }
  const X = v => L + (v - x0) / (x1 - x0) * (W - L - R);
  const Y = v => T + (1 - (v - y0) / (y1 - y0)) * (H - T - B);

  const pointShape = p => {
    const cx = X(p.x * 180 / Math.PI).toFixed(1);
    const cy = Y(p.y).toFixed(1);
    if (p.kind === "steering_left") {
      return `<circle cx="${cx}" cy="${cy}" r="5" style="fill:var(--blue)"/>`;
    }
    if (p.kind === "steering_right") {
      const px = X(p.x * 180 / Math.PI);
      const py = Y(p.y);
      return `<polygon points="${px.toFixed(1)},${(py - 6).toFixed(1)} ${(px - 5.5).toFixed(1)},${(py + 4).toFixed(1)} ${(px + 5.5).toFixed(1)},${(py + 4).toFixed(1)}" style="fill:var(--blue)"/>`;
    }
    if (p.kind === "steering_center") {
      return `<rect x="${(cx - 4.5).toFixed(1)}" y="${(cy - 4.5).toFixed(1)}" width="9" height="9" style="fill:var(--paper);stroke:var(--ink);stroke-width:1.5"/>`;
    }
    return `<rect x="${(cx - 4.5).toFixed(1)}" y="${(cy - 4.5).toFixed(1)}" width="9" height="9" style="fill:var(--ink)"/>`;
  };

  let inner = "";
  if (line) {
    inner += `<line x1="${X(x0).toFixed(1)}" y1="${Y(line.slope * x0 * Math.PI / 180 + line.intercept).toFixed(1)}"
      x2="${X(x1).toFixed(1)}" y2="${Y(line.slope * x1 * Math.PI / 180 + line.intercept).toFixed(1)}"
      style="stroke:var(--blue);stroke-width:2"/>`;
  }
  if (limits) {
    for (const limit of limits) {
      if (limit < y0 || limit > y1) continue;
      inner += `<line x1="${L}" y1="${Y(limit).toFixed(1)}" x2="${W - R}" y2="${Y(limit).toFixed(1)}"
        style="stroke:var(--bad);stroke-dasharray:5 4"/><text x="${W - R}" y="${(Y(limit) - 5).toFixed(1)}"
        text-anchor="end" font-size="11" style="fill:var(--muted)">servo limit</text>`;
    }
  }
  inner += points.map(pointShape).join("");
  const xTicks = [];
  for (let t = x0; t <= x1 + 1e-9; t += xStep) {
    xTicks.push(`<text x="${X(t).toFixed(1)}" y="${H - B + 18}" text-anchor="middle" font-size="11" class="tick" style="fill:var(--muted)">${t.toFixed(0)}</text>`);
  }
  const yTicks = [];
  for (let t = Math.ceil(y0 / 0.1) * 0.1; t <= y1 + 1e-9; t += 0.1) {
    const v = Math.round(t * 10) / 10;
    yTicks.push(`<text x="${L - 7}" y="${(Y(v) + 4).toFixed(1)}" text-anchor="end" font-size="11" class="tick" style="fill:var(--muted)">${v.toFixed(1)}</text>`);
  }
  const legends = [];
  if (points.some(p => p.kind === "steering_drift")) legends.push("Filled square: drive straight");
  if (points.some(p => p.kind === "steering_left")) legends.push("Circle: left circle");
  if (points.some(p => p.kind === "steering_right")) legends.push("Triangle: right circle");
  if (points.some(p => p.kind === "steering_center")) legends.push("Hollow square: centred (old sessions only)");
  const midY = T + (H - T - B) / 2;
  return `<figure class="figure"><svg class="diagram fit-chart" viewBox="0 0 ${W} ${H}" role="img"
      aria-label="Servo value against steering angle">
    <text x="${L}" y="${H - 8}" font-size="12" style="fill:var(--muted)">Steering angle (deg)</text>
    <text transform="rotate(-90 14 ${midY.toFixed(1)})" x="14" y="${midY.toFixed(1)}" text-anchor="middle" font-size="12" style="fill:var(--muted)">Servo</text>
    ${xTicks.join("")}${yTicks.join("")}${inner}</svg>
    <figcaption>Each point is one test. The line is the suggested setting.${legends.length ? ` ${legends.join(", ")}.` : ""}</figcaption></figure>`;
}

function sdPathD(angle, wheelbase) {
  const rearX = 160, rearY = 160;
  if (Math.abs(angle) < 0.002) return `M ${rearX} ${rearY} L ${rearX} 2`;
  const radius = Math.max(30, Math.min(600, Math.abs(wheelbase / Math.tan(angle)) * 120));
  const t = Math.min(2.0, 250 / radius);
  if (angle > 0) {
    const cx = rearX - radius;
    return `M ${rearX} ${rearY} A ${radius.toFixed(1)} ${radius.toFixed(1)} 0 0 0 ` +
      `${(cx + radius * Math.cos(t)).toFixed(1)} ${(rearY - radius * Math.sin(t)).toFixed(1)}`;
  }
  const cx = rearX + radius;
  return `M ${rearX} ${rearY} A ${radius.toFixed(1)} ${radius.toFixed(1)} 0 0 1 ` +
    `${(cx - radius * Math.cos(t)).toFixed(1)} ${(rearY - radius * Math.sin(t)).toFixed(1)}`;
}

function impliedReadout(raw) {
  return raw === null
    ? "Commanded angle not available (check the current settings)"
    : `Commanded: ${(Math.abs(raw) * 180 / Math.PI).toFixed(1)}° ${raw >= 0 ? "left" : "right"}. This is what the servo is told after the servo_min/servo_max clamp, not a measurement of the wheels.`;
}

function renderSteeringDiagram() {
  const params = currentParams();
  const wheelbase = finite(params.wheelbase) && params.wheelbase > 0 ? params.wheelbase : 0.36;
  const raw = impliedSteeringAngle();
  const angle = raw === null ? 0 : Math.max(-0.5, Math.min(0.5, raw));
  const rot = (-angle * 180 / Math.PI).toFixed(2);
  const pathD = sdPathD(angle, wheelbase);
  const capturing = app.snapshot?.session?.active_capture;
  const progress = app.snapshot?.capture_progress;
  let progressLine = "";
  if (capturing && (capturing.kind === "steering_left" || capturing.kind === "steering_right") && progress) {
    const turns = finite(progress.odom_yaw_rad)
      ? `${(progress.odom_yaw_rad / (2 * Math.PI)).toFixed(2)} turns (odometry estimate)` : "";
    const dist = finite(progress.odom_distance_m) ? `${progress.odom_distance_m.toFixed(2)} m` : "";
    progressLine = `<p id="capture-progress-line" class="small muted"><span id="capture-turns">${escapeHtml(turns)}</span>${turns && dist ? ", " : ""}<span id="capture-dist-live">${escapeHtml(dist)}</span></p>`;
  }
  return `<figure class="figure live-figure"><svg id="sd-svg" class="diagram" width="320" height="220" viewBox="0 0 320 220"
      role="img" aria-label="Live top-down steering diagram">
    <rect class="dg-body" x="118" y="28" width="84" height="150"/>
    <line class="dg-axle-line" x1="106" y1="160" x2="214" y2="160"/>
    <rect class="dg-tire" x="94" y="144" width="20" height="32"/>
    <rect class="dg-tire" x="206" y="144" width="20" height="32"/>
    <line class="dg-axle-line" x1="106" y1="58" x2="214" y2="58"/>
    <g id="sd-wheel-fl" transform="rotate(${rot} 106 58)">
      <rect class="dg-tire" x="96" y="42" width="20" height="32"/></g>
    <g id="sd-wheel-fr" transform="rotate(${rot} 214 58)">
      <rect class="dg-tire" x="204" y="42" width="20" height="32"/></g>
    <path id="sd-path" class="dg-path" d="${pathD}"/>
  </svg>
  <figcaption id="implied-readout">${escapeHtml(impliedReadout(raw))}</figcaption></figure>${progressLine}`;
}

function updateDiagramLive() {
  if (!document.getElementById("sd-svg")) return;
  const raw = impliedSteeringAngle();
  const readout = document.getElementById("implied-readout");
  if (readout) readout.textContent = impliedReadout(raw);
  const angle = raw === null ? 0 : Math.max(-0.5, Math.min(0.5, raw));
  const rot = (-angle * 180 / Math.PI).toFixed(2);
  const path = document.getElementById("sd-path");
  if (path) {
    const params = currentParams();
    const wheelbase = finite(params.wheelbase) && params.wheelbase > 0 ? params.wheelbase : 0.36;
    path.setAttribute("d", sdPathD(angle, wheelbase));
  }
  for (const id of ["sd-wheel-fl", "sd-wheel-fr"]) {
    const wheel = document.getElementById(id);
    if (wheel) {
      const cx = id === "sd-wheel-fl" ? 106 : 214;
      wheel.setAttribute("transform", `rotate(${rot} ${cx} 58)`);
    }
  }
}

// What the clamp does to the current settings; computed by the car from the
// settings alone (steering_chain.clamp_check), no test needed.
function renderClampCheck(clamp) {
  if (!clamp || !finite(clamp.left_reach_rad)) {
    return (clamp?.findings || []).length ? warningsBlock(clamp.findings) : "";
  }
  const reach = `<p>servo_min / servo_max let the software command up to ${deg(clamp.left_reach_rad)}° left and ${deg(clamp.right_reach_rad)}° right${
    finite(clamp.stick_scale_rad) ? `; full stick asks for ${deg(clamp.stick_scale_rad)}°` : ""}.</p>`;
  if (!clamp.findings?.length) return `${reach}<p class="ok-note">Nothing asks for more steering than the servo limits allow.</p>`;
  return `${reach}<ul class="warning-list">${clamp.findings.map(f => `<li>${escapeHtml(f)}</li>`).join("")}</ul>`;
}

function stickScale() {
  const live = app.snapshot?.live_stick_scale;
  if (finite(live)) return live;
  const stored = app.snapshot?.session?.vehicle?.steering_stick_scale;
  return finite(stored) ? stored : null;
}

// What a correct calibration predicts for this side's full-lock circle.
function expectedCircle(session, side) {
  const params = currentParams();
  const gain = params.steering_angle_to_servo_gain;
  const offset = params.steering_angle_to_servo_offset;
  const wheelbase = finite(params.wheelbase) && params.wheelbase > 0 ? params.wheelbase : 0.36;
  const scale = stickScale();
  const limits = servoLimits();
  if (!finite(gain) || gain === 0 || !finite(offset) || !finite(scale)) return null;
  const asked = side === "left" ? scale : -scale;
  let servo = offset + gain * asked;
  let clipped = null;
  if (limits) {
    if (servo > limits[1]) { servo = limits[1]; clipped = "servo_max"; }
    if (servo < limits[0]) { servo = limits[0]; clipped = "servo_min"; }
  }
  const commanded = (servo - offset) / gain;
  const inner = 2 * wheelbase / Math.tan(Math.abs(commanded)) - rearTrack(session);
  return {servo, clipped, commanded, inner: inner > 0 ? inner : null};
}

function renderExpectedCircle(session, side) {
  const expected = expectedCircle(session, side);
  if (!expected || !finite(expected.inner)) {
    return `<p class="small muted">The expected diameter appears once the car's gain, offset and full-stick angle are known.</p>`;
  }
  return `<p>With the settings the car runs now, full ${side} sends servo <span class="mono">${expected.servo.toFixed(3)}</span>${
    expected.clipped ? ` (cut by ${expected.clipped})` : ""} and commands ${deg(expected.commanded)}°.
    If the calibration is right, <strong>A</strong> comes out at about <strong>${expected.inner.toFixed(2)} m</strong>.</p>
    <p class="small muted">Clearly smaller means the wheels turn further than the gain thinks; clearly bigger, less. Within about ±5 cm is measuring error. After writing new values, restart bringup and teleop before checking.</p>`;
}

// Every value the wizard may write, as the page shows it. The full-stick
// angle is typed in degrees and stored in radians.
const EDITABLE = {
  steering_angle_to_servo_gain: {label: "Steering gain", file: "vesc.yaml", digits: 4},
  steering_angle_to_servo_offset: {label: "Straight-ahead servo (offset)", file: "vesc.yaml", digits: 4},
  servo_min: {label: "servo_min (full-left end)", file: "vesc.yaml", digits: 3},
  servo_max: {label: "servo_max (full-right end)", file: "vesc.yaml", digits: 3},
  steering_stick_scale: {label: "Full stick steering", file: "joy_teleop.yaml", degrees: true, digits: 1},
  speed_to_erpm_gain: {label: "Odometry speed gain", file: "vesc.yaml (odometry only)", digits: 2},
  speed_to_erpm_offset: {label: "Odometry speed offset", file: "vesc.yaml (odometry only)", digits: 2},
};
const STEERING_EDITABLE = ["steering_angle_to_servo_gain", "steering_angle_to_servo_offset",
  "servo_min", "servo_max", "steering_stick_scale"];

// What the car runs now, for every editable value.
function currentValues() {
  const session = app.snapshot?.session || {};
  const vehicle = session.vehicle || {};
  const values = {...(session.current_parameters || {})};
  for (const name of ["servo_min", "servo_max", "steering_stick_scale"]) {
    if (finite(vehicle[name])) values[name] = vehicle[name];
  }
  if (finite(app.snapshot?.live_stick_scale)) values.steering_stick_scale = app.snapshot.live_stick_scale;
  return values;
}

function shownValue(name, value) {
  const def = EDITABLE[name];
  if (!finite(value)) return "";
  return (def.degrees ? value * 180 / Math.PI : value).toFixed(def.degrees ? 1 : def.digits);
}

// A table of values to write: tick a row, change its number, press write.
// ``proposed`` prefills the boxes (a suggestion), else the current value.
function renderValueEditor(formId, names, proposed = {}, ticked = []) {
  const now = currentValues();
  const rows = names.map(name => {
    const def = EDITABLE[name];
    const start = finite(proposed[name]) ? proposed[name] : now[name];
    return `<tr><td><input id="${formId}-use-${name}" type="checkbox" ${ticked.includes(name) ? "checked" : ""}
        aria-label="Write ${escapeHtml(def.label)}"></td>
      <td>${escapeHtml(def.label)}${def.degrees ? " (°)" : ""}<span class="layer-finding">${escapeHtml(def.file)}</span></td>
      <td class="num">${escapeHtml(shownValue(name, now[name]) || "unknown")}</td>
      <td><input id="${formId}-val-${name}" class="input input-small" type="number" step="any"
        value="${escapeHtml(shownValue(name, start))}"></td></tr>`;
  }).join("");
  return `<div class="runs-table"><table class="data-table value-editor">
    <thead><tr><th>Write</th><th>Setting</th><th>Now</th><th>New value</th></tr></thead>
    <tbody>${rows}</tbody></table></div>
    <div class="button-row"><button class="button" type="button"
      onclick="writeValueEditor('${formId}', '${names.join(",")}')">Write the ticked values</button></div>
    <p class="small muted">The same safety checks as the suggestions apply (no wrong-sign gain, no offset outside the servo limits, servo values 0–1, full stick at most 40°). A backup is saved first; Undo puts it back.</p>`;
}

function sideCurve(side) {
  return app.snapshot?.steering_preview?.curves?.[side] || null;
}

// Which servo limit ends this side: servo = gain * angle + offset with left
// positive, so with the usual negative gain left runs to servo_min.
function sideLimitName(side) {
  const gain = currentValues().steering_angle_to_servo_gain;
  const towardsMax = (finite(gain) && gain > 0) === (side === "left");
  return towardsMax ? "servo_max" : "servo_min";
}

// "Find how far this side steers": the circles so far, and the next step.
function renderLimitCard(session, side) {
  const curve = sideCurve(side);
  const now = currentValues();
  const points = curve?.points || [];
  const limitName = sideLimitName(side);
  const limit = now[limitName];
  const rows = points.map((point, index) => {
    const step = index > 0 ? curve.steps[index - 1] : null;
    const pastStop = curve.stop && Math.abs(point.servo - curve.stop.beyond_servo) < 1e-9;
    return `<tr><td class="num">${point.servo.toFixed(3)}</td><td class="num">${point.angle_deg.toFixed(1)}°</td>
      <td class="num">${step ? `${step.added_deg >= 0 ? "+" : ""}${step.added_deg.toFixed(1)}°` : ""}</td>
      <td>${pastStop ? `<span class="layer-status limited">past the stop</span>` : ""}</td></tr>`;
  }).join("");
  const table = points.length ? `<div class="runs-table"><table class="data-table">
    <thead><tr><th>Servo</th><th>Wheel angle</th><th>Added</th><th></th></tr></thead>
    <tbody>${rows}</tbody></table></div>` : "";
  let verdict;
  if (!points.length) {
    verdict = `<p>Drive one full-lock ${side} circle first. Then this card shows how far it got and offers the next step.</p>`;
  } else if (curve.stop) {
    verdict = `<p class="done-banner"><span aria-hidden="true">✓</span> The ${side} side levels off past servo ${curve.stop.servo.toFixed(3)}: the last step added only ${curve.stop.added_deg.toFixed(1)}°. The report suggests ${limitName} ${curve.stop.servo.toFixed(3)}.</p>`;
  } else if (finite(limit) && Math.abs(points[points.length - 1].servo - limit) <= 0.005) {
    verdict = `<p>Your last ${side} circle reached ${limitName} ${limit.toFixed(3)} at ${points[points.length - 1].angle_deg.toFixed(1)}°. To see whether the wheels can turn further, try the next step below.</p>`;
  } else {
    verdict = `<p>The last ${side} circle did not reach ${limitName}. That is fine if you only want to check the calibration.</p>`;
  }
  if (!applyAllowed() || !finite(limit)) return `${table}${verdict}`;
  const towardsMax = limitName === "servo_max";
  const next = Math.round((towardsMax ? Math.min(1, limit + app.limitStep) : Math.max(0, limit - app.limitStep)) * 10000) / 10000;
  const atEnd = Math.abs(next - limit) < 1e-9;
  const gain = now.steering_angle_to_servo_gain;
  const offset = now.steering_angle_to_servo_offset;
  const scale = now.steering_stick_scale;
  const reaches = finite(gain) && finite(offset) && finite(scale)
    && Math.abs(gain) * scale >= Math.abs(next - offset);
  return `${table}${verdict}
    <h3>Try more ${side} lock</h3>
    ${atEnd ? `<p>${limitName} is already ${limit.toFixed(3)}, the end of the servo signal. There is no further step.</p>` : `
    <div class="field-row">
      <div class="field">
        <label class="field-label" for="limit-step">Step size (servo units)</label>
        <input id="limit-step" class="input input-small" type="number" min="0.005" max="0.1" step="0.005"
          value="${app.limitStep}" onchange="setLimitStep(this.value)">
      </div>
    </div>
    ${reaches ? "" : `<div class="check-row"><label><input id="limit-raise-stick" type="checkbox" checked>
      Full stick does not reach ${next.toFixed(3)} yet. Also raise full stick enough to reach it (temporary; the report suggests the final value).</label></div>`}
    <div class="button-row"><button class="button" type="button" onclick="stepLimit('${side}')">Write ${limitName} ${next.toFixed(3)} (now ${limit.toFixed(3)})</button></div>
    <ol class="instruction-list">
      <li>Restart bringup${reaches ? "" : " and teleop"} when the banner asks.</li>
      <li><strong>Wheels off the ground:</strong> hold LB and full ${side} for 3 seconds. Stop at any buzzing, heat or the linkage touching something, and press <strong>Undo the last change</strong>.</li>
      <li>If it was fine, put the car down and record another full-lock ${side} circle.</li>
    </ol>`}`;
}

// The car's measurements the circles depend on.
function renderCarCard(session) {
  const params = currentParams();
  const track = rearTrack(session);
  return `<dl class="def-list">
      <div><dt>Wheelbase</dt><dd>${finite(params.wheelbase) ? params.wheelbase.toFixed(3) : "unknown"} m <span class="small muted">(read from vesc.yaml)</span></dd></div>
    </dl>
    <div class="field-row">
      <div class="field">
        <label class="field-label" for="rear-track">Rear track: rear tire centre to centre (m)</label>
        <input id="rear-track" class="input input-small" type="number" min="0.1" max="0.6" step="0.005" value="${track.toFixed(3)}">
        <p class="field-hint">Used to turn an inner-tire circle into the rear-axle circle. Measure it once with "both rear tires": outer minus inner diameter is twice the track.</p>
      </div>
    </div>
    <div class="button-row"><button class="button button-ghost button-small" type="button" onclick="saveRearTrack()">Save rear track</button></div>`;
}

function applyAllowed() {
  return Boolean(app.snapshot?.apply?.allowed);
}

function restartBanner() {
  const waiting = app.snapshot?.pending_restart || [];
  if (!waiting.length) return "";
  const teleop = waiting.includes("steering_stick_scale");
  const bringup = waiting.some(name => name !== "steering_stick_scale");
  const what = [bringup ? "bringup" : "", teleop ? "teleop" : ""].filter(Boolean).join(" and ");
  return `<p class="warning-banner"><strong>Restart ${what}.</strong> The files now have new ${
    escapeHtml(waiting.join(", "))}, but the running car still uses the old value${waiting.length === 1 ? "" : "s"}. Stop ${what} (Ctrl+C) and start ${teleop && bringup ? "them" : "it"} again, then re-run the test.</p>`;
}

function appliedHistory(session) {
  const changes = (session.applied_changes || []);
  if (!changes.length) return "";
  const latest = [...changes].reverse().find(change => !change.restored);
  const items = changes.map(change => `<li><span class="mono">${escapeHtml(Object.entries(change.after || {})
      .map(([name, value]) => `${name} ${fmt(change.before?.[name], 6, "?")} → ${fmt(value, 6)}`).join(", "))}</span>
      <span class="small muted">${escapeHtml(change.at || "")}${change.restored ? ", undone" : ""}. Backup: ${escapeHtml(change.backup || "")}</span>${
      change.in_source_tree === false ? `<span class="small">This edited the installed copy, which the next colcon build overwrites.</span>` : ""}</li>`).join("");
  return `<h3>Changes written</h3><ul class="applied-list">${items}</ul>
    ${latest && applyAllowed() ? `<div class="button-row"><button class="button button-ghost button-small" type="button" onclick="restoreBackup()">Undo the last change</button>${
      changes.filter(change => !change.restored).length > 1 ? `<button class="button button-ghost button-small" type="button" onclick="restoreEverything()">Undo everything from this session</button>` : ""}</div>` : ""}`;
}

function renderCentreCard(session) {
  const centre = app.snapshot?.centre_offset;
  if (!centre) return "";
  const angle = centre.drift_angle_rad;
  const drift = finite(angle)
    ? `${Math.abs(angle * 180 / Math.PI).toFixed(2)}° ${angle >= 0 ? "left" : "right"}` : "not available";
  let body;
  if (centre.stale) {
    body = `<p class="tip">The car now uses offset ${fmt(centre.current_offset, 4)}. Record another drive-straight run to see whether it drives straight.</p>`;
  } else if (centre.blocked) {
    body = `<p class="warning-banner">No offset suggested: ${escapeHtml(centre.blocked)}.</p>`;
  } else if (centre.straight) {
    body = `<p class="done-banner"><span aria-hidden="true">✓</span> The car drives straight (within 0.14°). The centre offset is done.</p>`;
  } else {
    body = `<p>Suggested <span class="mono">steering_angle_to_servo_offset</span>: <strong class="mono">${fmt(centre.suggested_offset, 4)}</strong>
      (now ${fmt(centre.current_offset, 4)}, change ${signed(centre.suggested_offset - centre.current_offset, 4)}).</p>
      ${applyAllowed() ? `<div class="button-row"><button class="button" type="button" onclick="applyCentreOffset()">Write this offset to vesc.yaml</button></div>
      <p class="small muted">Then restart bringup and record another drive-straight run. Repeat until it says the car drives straight.</p>`
      : `<p class="small muted">Copy it into the shared block of vesc.yaml, restart bringup, and record another drive-straight run.</p>`}`;
  }
  return `<section><h2>Centre offset</h2>
    <p>Latest run (${centre.run_count} so far) curved ${escapeHtml(drift)}.</p>${body}
    ${appliedHistory(session)}</section>`;
}

function stepEyebrow(session, id) {
  const position = groupPosition(session, id);
  return position ? `${position.group} calibration · test ${position.index} of ${position.total}` : "";
}

// One page layout shared by every test step.
function renderTestStep(session, id) {
  const def = STEPS[id];
  const done = stepDone(session, id);
  const runs = stepRunCount(session, id);
  const next = nextStepId(session, id);
  const isCircle = id === "steering_left" || id === "steering_right";
  const nextLabel = next === "report"
    ? (reportLocked(session) ? "" : "Next: build the report")
    : `Next: ${STEPS[next].label}`;
  let status = "";
  if (done) {
    status = `<p class="done-banner"><span aria-hidden="true">✓</span> This test is done (${runs} run${runs === 1 ? "" : "s"} accepted).${
      def.best && runs < def.best ? ` One more run is recommended: ${runs} of ${def.best}.` : ""}</p>`;
  } else if (runs > 0) {
    status = `<p class="tip">${runs} of ${def.need} runs accepted. Record ${def.need - runs} more.</p>`;
  }
  const record = `<button class="button ${done ? "button-ghost" : ""}" type="button" onclick="startCapture('${def.kind}')">${done ? "Record another run" : "Record"}</button>`;
  const forward = done && nextLabel
    ? `<button class="button" type="button" onclick="goToStep('${next}')">${escapeHtml(nextLabel)}</button>` : "";
  const points = def.stage === "steering" ? clientTrialPoints(session, currentParams().wheelbase) : [];
  return `<section>
    ${pageHead(stepEyebrow(session, id), def.label)}
    <p class="finds"><strong>What this finds:</strong> ${escapeHtml(def.finds)}</p>
    ${phaseStrip(def.phases, done ? def.phases.length : 0)}
    ${restartBanner()}
    ${status}
    <div class="step-body">
      ${setupDiagram(id)}
      <ol class="instruction-list step-instructions">${def.steps.map(step => `<li>${step}</li>`).join("")}</ol>
    </div>
    ${def.tip ? `<p class="tip">${escapeHtml(def.tip)}</p>` : ""}
    <div class="button-row">${forward}${record}</div>
    ${isCircle ? `<p class="small muted extra-link">Optional extra: a circle with the stick only <em>half</em> way over shows whether the steering is linear. It does not count toward this test.
      <button class="link-button" type="button" onclick="recordHalfLock('${def.kind}')">Record a half-lock circle</button></p>` : ""}
  </section>
  ${isCircle ? `<section><h2>What to expect</h2>${renderExpectedCircle(session, def.side)}
    ${renderClampCheck(app.snapshot?.clamp_check)}</section>
    <section><h2>Find how far the ${def.side} side steers</h2>${renderLimitCard(session, def.side)}
    ${appliedHistory(session)}</section>
    <section><h2>Live steering</h2>${renderSteeringDiagram()}</section>` : ""}
  ${id === "steering_drift" ? renderCentreCard(session) : ""}
  <section>
    <h2>Runs for this test</h2>
    ${acceptedRunsTable(session, [def.kind])}
    ${points.length >= 2 ? `<h3>Steering fit so far</h3>${renderFitChart(points, linearFit(points), servoLimits())}` : ""}
  </section>
  ${isCircle ? `<section><details class="details"><summary>Car measurements</summary>${renderCarCard(session)}</details></section>` : ""}
  ${manualPanel("step")}`;
}

function renderRecording(session) {
  const active = session.active_capture;
  const def = STEPS[active.kind];
  const progress = app.snapshot.capture_progress;
  const isCircle = active.kind === "steering_left" || active.kind === "steering_right";
  const dist = finite(progress?.odom_distance_m) ? `${progress.odom_distance_m.toFixed(2)} m` : "--";
  const turns = isCircle && finite(progress?.odom_yaw_rad)
    ? `${(progress.odom_yaw_rad / (2 * Math.PI)).toFixed(2)} turns (odometry estimate)` : "";
  return `<section>
    ${pageHead(def ? stepEyebrow(session, active.kind) : "Recording", `Recording: ${kindLabel(active.kind)}`)}
    ${def ? phaseStrip(def.phases, 2) : ""}
    <div class="recording-timer"><span class="rec-dot" aria-hidden="true"></span>
      <span id="capture-timer">${fmt(app.snapshot.capture_duration_sec, 1, "--")} s</span></div>
    <p class="mono">Distance <span id="capture-dist">${escapeHtml(dist)}</span>${turns ? `, <span id="capture-turns">${escapeHtml(turns)}</span>` : `<span id="capture-turns"></span>`}</p>
    <p class="now-do">${escapeHtml(def?.during || "Stop the car and release LB before pressing Stop.")}</p>
    <div class="button-row">
      <button class="button button-danger" type="button" onclick="stopCapture()">Stop</button>
    </div>
  </section>
  ${isCircle ? `<section><h2>Live steering</h2>${renderSteeringDiagram()}</section>` : ""}`;
}

function summaryDefs(summary) {
  const rows = [
    ["Duration", finite(summary.duration_sec) ? `${summary.duration_sec.toFixed(1)} s` : "Not enough data"],
    ["Odom distance", finite(summary.odom_distance_m) ? `${signed(summary.odom_distance_m, 3)} m` : "Not enough data"],
    ["Raw ERPM integral", finite(summary.raw_erpm_integral) ? signed(summary.raw_erpm_integral, 1) : "Not enough data"],
    ["Odom yaw", finite(summary.odom_yaw_rad) ? `${signed(summary.odom_yaw_rad, 3)} rad` : "Not enough data"],
    ["Median servo", finite(summary.servo?.median) ? summary.servo.median.toFixed(4) : "Not enough data"],
  ];
  return `<dl class="def-list">${rows.map(([label, value]) =>
    `<div><dt>${escapeHtml(label)}</dt><dd>${escapeHtml(value)}</dd></div>`).join("")}</dl>`;
}

function warningsBlock(warnings) {
  if (!warnings?.length) return `<p class="ok-note">No problems found in the recording.</p>`;
  return `<ul class="warning-list">${warnings.map(w => `<li>${escapeHtml(w)}</li>`).join("")}</ul>`;
}

function letterLabel(letter, text) {
  return `<span class="field-letter" aria-hidden="true">${letter}</span>${text}`;
}

function driftForm() {
  return `${driftDiagram(true)}
    <div class="field">
      <label class="field-label" for="drift-forward">${letterLabel("A", "Distance along the line (m)")}</label>
      <input id="drift-forward" class="input" type="number" min="0.01" step="0.01" placeholder="e.g. 5.00">
    </div>
    <div class="field">
      <label class="field-label" for="drift-side-mag">${letterLabel("B", "Sideways gap from the line (m)")}</label>
      <input id="drift-side-mag" class="input" type="number" min="0" step="0.01" placeholder="e.g. 0.12">
      <p class="field-hint">Seen from behind the car, looking the way it drove.</p>
    </div>
    <div class="radio-row" role="radiogroup" aria-label="Side of the line">
      <label><input type="radio" name="drift-side" value="left"> Ended left of the line</label>
      <label><input type="radio" name="drift-side" value="right"> Ended right of the line</label>
    </div>
    <div class="check-row">
      <label><input id="drift-online" type="checkbox"> Ended exactly on the line (B is 0)</label>
    </div>`;
}

const CIRCLE_METHOD_LINKS = {
  inner_tire: "Measured only the inner tire (default)",
  rear_tires: "Measured both rear tires",
  axle_centre: "Measured under the rear-axle centre",
};

function circleForm(kind) {
  const side = kind === "steering_left" ? "left" : "right";
  const method = app.circleMethod;
  const shown = id => method === id ? "" : "hidden";
  const others = Object.keys(CIRCLE_METHOD_LINKS).filter(id => id !== method)
    .map(id => `<button class="link-button" type="button" onclick="setCircleMethod('${id}')">${escapeHtml(CIRCLE_METHOD_LINKS[id])}</button>`)
    .join(" · ");
  return `<div id="circle-fig-inner" ${shown("inner_tire")}>${circleDiagram(side, true, "inner_tire")}</div>
    <div id="circle-fig-tires" ${shown("rear_tires")}>${circleDiagram(side, true, "rear_tires")}</div>
    <div id="circle-fig-axle" ${shown("axle_centre")}>${circleDiagram(side, true, "axle_centre")}</div>
    <div id="circle-tire-fields" class="field-row" ${method === "axle_centre" ? "hidden" : ""}>
      <div class="field">
        <label class="field-label" for="circle-inner">${letterLabel("A", `Inner (${side}) tire, across the circle (m)`)}</label>
        <input id="circle-inner" class="input" type="number" min="0.1" step="0.01" placeholder="e.g. 1.33">
      </div>
      <div class="field" id="circle-outer-field" ${shown("rear_tires")}>
        <label class="field-label" for="circle-outer">${letterLabel("B", "Outer tire, across the circle (m)")}</label>
        <input id="circle-outer" class="input" type="number" min="0.1" step="0.01" placeholder="e.g. 1.87">
      </div>
    </div>
    <div id="circle-axle-field" class="field" ${shown("axle_centre")}>
      <label class="field-label" for="circle-diameter">${letterLabel("C", "Rear-axle centre, across the circle (m)")}</label>
      <input id="circle-diameter" class="input" type="number" min="0.1" step="0.01" placeholder="e.g. 1.60">
    </div>
    ${method === "inner_tire" ? `<p class="field-hint">The car's rear track, ${rearTrack(app.snapshot?.session).toFixed(2)} m, turns A into the rear-axle circle.</p>` : ""}
    <p class="small">Other ways to measure: ${others}</p>
    <div class="check-row">
      <label><input id="circle-fulllock" type="checkbox" ${app.halfLockNext ? "" : "checked"}> <span>The stick was held <strong>fully</strong> ${side} (full lock). Untick for a half-lock extra.</span></label>
    </div>`;
}

function movementForm() {
  return `${distanceDiagram(true)}
    <div class="field">
      <label class="field-label" for="trial-direction">Direction</label>
      <select id="trial-direction" class="select">
        <option value="forward">Forward</option>
        <option value="reverse">Reverse</option>
      </select>
    </div>
    <div class="field">
      <label class="field-label" for="trial-distance">${letterLabel("A", "Measured distance (m)")}</label>
      <input id="trial-distance" class="input" type="number" min="0.01" step="0.001" placeholder="e.g. 5.000">
      <p class="field-hint">Always a positive number, even for a reverse run.</p>
    </div>`;
}

function renderReview(session) {
  const pending = session.pending_capture;
  const summary = pending.summary || {};
  const def = STEPS[pending.kind];
  let form = "";
  if (pending.kind === "movement") form = movementForm();
  else if (pending.kind === "steering_drift") form = driftForm();
  else if (pending.kind === "steering_left" || pending.kind === "steering_right") form = circleForm(pending.kind);
  else form = `<p>Nothing to measure for this test. If the car stayed still, accept the run.</p>`;
  return `<section>
    ${pageHead(def ? stepEyebrow(session, pending.kind) : "Check this run", `Measure: ${kindLabel(pending.kind)}`)}
    ${def ? phaseStrip(def.phases, def.phases.length - 1) : ""}
    ${warningsBlock(summary.warnings)}
    ${form}
    <div class="field">
      <label class="field-label" for="trial-notes">Notes (optional)</label>
      <textarea id="trial-notes" class="textarea"></textarea>
    </div>
    ${summary.warnings?.length ? `<div class="check-row">
      <label><input id="trial-override" type="checkbox"> Use this run even though it has the warnings above.</label>
    </div>` : ""}
    <div class="check-row">
      <label><input id="trial-confirmed" type="checkbox"> These measurements are for this run.</label>
    </div>
    <div class="button-row">
      <button class="button" type="button" onclick="acceptPending()">Accept run</button>
      <button class="button button-ghost" type="button" onclick="discardPending()">Discard</button>
    </div>
    <details class="details"><summary>What the car recorded</summary>${summaryDefs(summary)}</details>
  </section>`;
}

function statusPlate(status) {
  const ready = status === "ready" || status === "good" || status === "high";
  return `<p class="status-plate ${ready ? "good" : ""}">${ready ? "Ready" : "Needs review"}</p>`;
}

function paramTable(current, suggestions, keys, blocked = {}, notReady = {}) {
  const rows = keys.filter(name => name in suggestions || name in blocked).map(name => {
    const now = current[name];
    if (name in blocked) {
      return `<tr><td class="mono">${escapeHtml(name)}</td>
        <td class="num">${fmt(now, 6)}</td>
        <td colspan="2"><span class="layer-status limited">blocked</span> ${escapeHtml(blocked[name])}</td></tr>`;
    }
    const suggested = suggestions[name];
    const change = finite(now) && finite(suggested) ? signed(suggested - now, 6) : "";
    const note = notReady[name] ? `<span class="layer-finding">Not ready to apply: ${escapeHtml(notReady[name])}</span>` : "";
    return `<tr><td class="mono">${escapeHtml(name)}</td>
      <td class="num">${fmt(now, 6)}</td>
      <td class="num">${fmt(suggested, 6)}${note}</td>
      <td class="num">${escapeHtml(change)}</td></tr>`;
  });
  if (!rows.length) return `<p class="small muted">No suggestions in this section.</p>`;
  return `<table class="data-table">
    <thead><tr><th>Parameter</th><th>Now</th><th>Suggested</th><th>Change</th></tr></thead>
    <tbody>${rows.join("")}</tbody></table>`;
}

function yamlBlock(suggestions, keys, blocked = {}) {
  const lines = keys.filter(name => name in suggestions || name in blocked).map(name => {
    if (name in blocked) return `# ${name}: blocked, ${blocked[name]}`;
    const value = suggestions[name];
    return finite(value) ? `${name}: ${value.toFixed(6)}` : `# ${name}: not enough data`;
  });
  return `<pre class="code-block">${escapeHtml(lines.join("\n") || "No suggestions yet.")}</pre>`;
}

function reportFitPoints(trialResults) {
  const points = [];
  for (const item of trialResults || []) {
    if (!item.usable) continue;
    if (finite(item.actual_steering_angle_rad) && finite(item.median_servo)) {
      points.push({x: item.actual_steering_angle_rad, y: item.median_servo, kind: item.kind});
    }
  }
  return points;
}

function applyButton(report, keys, label) {
  if (!applyAllowed()) return "";
  const ready = keys.filter(name => name in (report.parameter_suggestions || {})
    && !(name in (report.not_ready_to_apply || {})));
  if (!ready.length) return `<p class="small muted">Nothing here is ready to write to vesc.yaml yet.</p>`;
  return `<div class="button-row"><button class="button" type="button"
    onclick="applyReport('${ready.join(",")}')">${escapeHtml(label)}</button></div>`;
}

const LIMIT_KEYS = ["servo_min", "servo_max", "steering_stick_scale"];

function renderCurves(steering) {
  const rows = [];
  for (const side of ["left", "right"]) {
    const curve = steering.curves?.[side];
    if (!curve?.points?.length) continue;
    curve.points.forEach((point, index) => {
      const step = index > 0 ? curve.steps[index - 1] : null;
      const atStop = curve.stop && Math.abs(point.servo - curve.stop.beyond_servo) < 1e-9;
      rows.push(`<tr><td>${index === 0 ? escapeHtml(side) : ""}</td>
        <td class="num">${point.servo.toFixed(3)}</td>
        <td class="num">${point.angle_deg.toFixed(1)}°${point.count > 1 ? ` (${point.count} circles)` : ""}</td>
        <td class="num">${step ? `${step.added_deg >= 0 ? "+" : ""}${step.added_deg.toFixed(1)}°` : ""}</td>
        <td>${atStop ? `<span class="layer-status limited">past the stop</span>` : ""}</td></tr>`);
    });
  }
  if (!rows.length) return "";
  return `<div class="runs-table"><table class="data-table">
    <thead><tr><th>Side</th><th>Servo</th><th>Wheel angle (measured)</th><th>Added by this step</th><th></th></tr></thead>
    <tbody>${rows.join("")}</tbody></table></div>
    <p class="small muted">A step that adds less than ${"1.0"}° means more servo has stopped turning the wheels: the linkage is at or near its stop.</p>`;
}

function renderChecks(steering) {
  const rows = ["left", "right"].map(side => {
    const check = steering.sides?.[side]?.check;
    if (!check) return "";
    return `<tr><td>${escapeHtml(side)}</td><td class="num">${check.commanded_deg.toFixed(1)}°</td>
      <td class="num">${check.measured_deg.toFixed(1)}°</td>
      <td class="num">${check.error_deg >= 0 ? "+" : ""}${check.error_deg.toFixed(1)}°</td>
      <td>${check.passed ? "✓ matches" : "off"}</td></tr>`;
  }).join("");
  if (!rows) return "";
  const confirmed = steering.calibration_confirmed;
  return `<h3>Check: latest full-lock circle on each side</h3>
    <div class="runs-table"><table class="data-table">
    <thead><tr><th>Side</th><th>Commanded (gain at the time)</th><th>Measured</th><th>Difference</th><th></th></tr></thead>
    <tbody>${rows}</tbody></table></div>
    ${confirmed === true ? `<p class="done-banner"><span aria-hidden="true">✓</span> Both sides turn what the car commanded, within 1°. The calibration is confirmed.</p>`
      : confirmed === false ? `<p class="tip">At least one side does not turn what was commanded. Write the suggested values, restart bringup and teleop, and drive one more full-lock circle each way to check.</p>` : ""}`;
}

function renderReport(session) {
  const report = session.report;
  if (!report) {
    const remaining = remainingSteps(session);
    if (remaining.length) {
      return `<section>
        ${pageHead("Report", "Report")}
        <p class="lead">Finish ${remaining.length === 1 ? "this test" : `these ${remaining.length} tests`} first. The report needs every test to give a trustworthy suggestion.</p>
        <ul class="todo-list">${remaining.map(id => `<li><button class="link-button" type="button" onclick="goToStep('${id}')">${
          escapeHtml(`${STEPS[id].group}: ${STEPS[id].label}`)}</button></li>`).join("")}</ul>
        <div class="button-row">
          <button class="button" type="button" disabled>Generate report</button>
        </div>
      </section>`;
    }
    return `<section>
      ${pageHead("Report", "Report")}
      <p class="lead">Every test is done. Generate the report to turn the accepted runs into suggested values.</p>
      <div class="button-row">
        <button class="button" type="button" onclick="generateReport()">Generate report</button>
      </div>
    </section>`;
  }
  const suggestions = report.parameter_suggestions || {};
  const blocked = report.blocked_suggestions || {};
  const notReady = report.not_ready_to_apply || {};
  const current = report.current_parameters || {};
  let html = `<section>${pageHead("Report", "Report")}${statusPlate(report.overall_status)}
    <p class="lead">${escapeHtml(report.safety_note)}</p></section>`;

  if (report.movement) {
    const movement = report.movement;
    html += `<section><h2>Wheel calibration</h2>${sectorRule()}${statusPlate(movement.status)}
      <p>${movement.usable_trial_count} usable of ${movement.accepted_trial_count} accepted runs.</p>
      ${paramTable(current, suggestions, ["speed_to_erpm_gain", "speed_to_erpm_offset"], blocked, notReady)}
      ${warningsBlock(movement.warnings)}
      <p class="yaml-label">Put these in the vesc_to_odom_node section of vesc.yaml (odometry only)</p>
      ${yamlBlock(suggestions, ["speed_to_erpm_gain", "speed_to_erpm_offset"], blocked)}
      ${applyButton(report, ["speed_to_erpm_gain", "speed_to_erpm_offset"], "Write wheel values to vesc.yaml")}
    </section>`;
  }

  if (report.steering) {
    const steering = report.steering;
    const points = reportFitPoints(steering.trial_results);
    const line = finite(steering.suggested_steering_angle_to_servo_gain)
      && finite(steering.suggested_steering_angle_to_servo_offset)
      ? {slope: steering.suggested_steering_angle_to_servo_gain,
         intercept: steering.suggested_steering_angle_to_servo_offset} : null;
    const reach = steering.max_commandable_steering;
    const reachLine = reach && finite(reach.left_rad) && finite(reach.right_rad)
      ? `<p>Servo limits allow up to ${deg(reach.left_rad)} deg left and ${deg(-reach.right_rad)} deg right.</p>` : "";
    const trackLine = finite(steering.rear_track_m)
      ? `<p>Rear track ${steering.rear_track_m.toFixed(3)} m.</p>` : "";
    const residualLine = `<p>Fit residual ${fmt(steering.fit_rmse_servo, 4, "not available")} servo units.</p>`;
    const sideRow = (label, side) => `<tr><td>${escapeHtml(label)}</td>
      <td class="num">${fmt(side.full_lock_radius_m, 3)}</td>
      <td class="num">${side.full_lock_angle_rad === null || side.full_lock_angle_rad === undefined
        ? "Not enough data" : deg(Math.abs(side.full_lock_angle_rad))}</td>
      <td>${side.full_lock_at_servo_limit === true ? "Yes"
        : side.full_lock_at_servo_limit === false ? "No" : "Not enough data"}</td></tr>`;
    html += `<section><h2>Steering calibration</h2>${sectorRule()}${statusPlate(steering.status)}
      ${renderFitChart(points, line, steering.servo_limits)}
      <table class="data-table">
        <thead><tr><th>Side</th><th>Full-lock radius (m)</th><th>Full-lock angle (deg)</th><th>On servo limit</th></tr></thead>
        <tbody>${sideRow("Left", steering.sides.left)}${sideRow("Right", steering.sides.right)}</tbody>
      </table>
      ${reachLine}${trackLine}${residualLine}
      ${paramTable(current, suggestions,
        ["steering_angle_to_servo_gain", "steering_angle_to_servo_offset"], blocked, notReady)}
      ${warningsBlock(steering.warnings)}
      <p class="yaml-label">Put these in the shared steering values in vesc.yaml</p>
      ${yamlBlock(suggestions,
        ["steering_angle_to_servo_gain", "steering_angle_to_servo_offset"], blocked)}
      ${applyButton(report, ["steering_angle_to_servo_gain", "steering_angle_to_servo_offset"], "Write steering values to vesc.yaml")}
    </section>
    <section><h2>How far the steering goes</h2>${sectorRule()}
      ${renderCurves(steering)}
      ${renderChecks(steering)}
      ${paramTable(current, suggestions, LIMIT_KEYS, blocked, notReady)}
      ${finite(suggestions.steering_stick_scale) ? `<p>Suggested full stick: ${deg(suggestions.steering_stick_scale)}°, inside what both sides reach.</p>` : ""}
      ${applyButton(report, LIMIT_KEYS, "Write servo limits and full-stick angle")}
      ${applyAllowed() ? `<details class="details"><summary>Edit before writing</summary>
        <p>The boxes start at the suggestions. Tick what to write and change any number, including one marked not ready.</p>
        ${renderValueEditor("report-steering", STEERING_EDITABLE, suggestions,
          STEERING_EDITABLE.filter(name => finite(suggestions[name]) && !(name in notReady)))}</details>` : ""}
      <p class="small muted">servo_min / servo_max go into vesc.yaml (restart bringup); the full-stick angle into joy_teleop.yaml (restart teleop). Change autonomy's max_steering_angle by hand, below what both sides reach.</p>
    </section>
    <section><h2>Servo clamp, from the current settings</h2>${sectorRule()}
      ${renderClampCheck(report.clamp_check)}</section>`;
  }
  html += `<section>${restartBanner()}${appliedHistory(session)}</section>`;

  const wheel = report.movement ? `<button class="button button-ghost" type="button" onclick="goToStep('movement')">Add a wheel run</button>` : "";
  const steer = report.steering ? `<button class="button button-ghost" type="button" onclick="goToStep('steering_drift')">Add a steering test</button>` : "";
  html += `<section><div class="button-row">
      <a class="button" href="/api/report/md">Download Markdown</a>
      <a class="button button-ghost" href="/api/report/json">Download JSON</a>
      <button class="button button-ghost" type="button" onclick="generateReport()" ${remainingSteps(session).length ? "disabled" : ""}>Recalculate</button>
      ${wheel}${steer}
    </div></section>${manualPanel("report")}`;
  return html;
}

function render() {
  renderConnection();
  if (!app.snapshot) return;
  renderNav();
  renderTelemetry();
  renderWizardView();
}

function renderWizardView() {
  const session = app.snapshot.session;
  const view = document.getElementById("wizard-view");
  if (!view) return;
  view.dataset.rendered = "true";
  if (!session || app.choosingMode) {
    view.innerHTML = renderSetup();
    return;
  }
  if (session.active_capture) {
    view.innerHTML = renderRecording(session);
    return;
  }
  if (session.pending_capture) {
    view.innerHTML = renderReview(session);
    return;
  }
  const id = currentStepId(session);
  if (id === "report") view.innerHTML = renderReport(session);
  else if (STEPS[id]?.kind && sessionSteps(session).includes(id)) view.innerHTML = renderTestStep(session, id);
  else view.innerHTML = renderPreflight(session);
}

window.selectCalibrationMode = function selectCalibrationMode(mode) {
  if (!CALIBRATION_MODES.some(item => item.value === mode)) return;
  app.setupParameters = currentParameterValues();
  app.setupMode = mode;
  renderWizardView();
};

window.openCalibrationSetup = function openCalibrationSetup() {
  const session = app.snapshot?.session;
  if (session?.active_capture) {
    toast("Stop recording before changing calibration.", "error");
    return;
  }
  app.setupMode = session?.mode || app.setupMode;
  app.setupParameters = {...(session?.current_parameters || app.snapshot?.live_parameters)};
  app.choosingMode = true;
  renderNav();
  renderWizardView();
};

window.cancelCalibrationSetup = function cancelCalibrationSetup() {
  app.choosingMode = false;
  app.setupParameters = null;
  renderNav();
  renderWizardView();
};

window.goToStep = async function goToStep(id) {
  const session = app.snapshot?.session;
  const def = STEPS[id];
  if (!session || !def || !sessionSteps(session).includes(id)) return;
  if (session.active_capture || session.pending_capture) return;
  if (id === "report" && reportLocked(session)) {
    toast("Finish every test before building the report.", "error");
    return;
  }
  if (def.stage === "steering") app.steeringTest = id;
  if (typeof window.scrollTo === "function") window.scrollTo(0, 0);
  if (session.stage === def.stage) {
    renderNav();
    renderWizardView();
    return;
  }
  await window.setStage(def.stage);
};

window.setCircleMethod = function setCircleMethod(method) {
  if (!Object.hasOwn(CIRCLE_METHOD_LINKS, method)) return;
  app.circleMethod = method;
  renderWizardView();
};

window.createSession = async function createSession() {
  const mode = selectedMode();
  if (!mode) {
    toast("Pick at least one calibration.", "error");
    return;
  }
  try {
    const replaceExisting = Boolean(app.snapshot?.session);
    if (app.snapshot?.session?.active_capture) {
      toast("Stop recording before starting a new calibration.", "error");
      return;
    }
    if (replaceExisting && !confirm("Archive the current session and start the selected calibration?")) return;
    await api("new_session", {
      mode,
      current_parameters: currentParameterValues(),
      replace_existing: replaceExisting,
    });
    app.choosingMode = false;
    app.setupParameters = null;
    renderNav();
    renderWizardView();
    toast("Session started.");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.setStage = async function setStage(stage) {
  try {
    await api("set_stage", {stage});
  } catch (error) {
    toast(error.message, "error");
  }
};

window.startCapture = async function startCapture(kind) {
  if (STEERING_STEPS.includes(kind)) app.steeringTest = kind;
  try {
    await api("start_capture", {kind});
  } catch (error) {
    const health = error.details?.health;
    const suffix = health?.odom ? ` Odom is ${health.odom.status}.` : "";
    toast(`${error.message}${suffix}`, "error");
  }
};

window.recordHalfLock = function recordHalfLock(kind) {
  app.halfLockNext = true;
  return window.startCapture(kind);
};

window.stopCapture = async function stopCapture() {
  try {
    await api("stop_capture");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.acceptPending = async function acceptPending() {
  const pending = app.snapshot?.session?.pending_capture;
  if (!pending) {
    toast("There is no pending run to accept.", "error");
    return;
  }
  const confirmed = document.getElementById("trial-confirmed")?.checked === true;
  const extra = {
    confirmed,
    notes: document.getElementById("trial-notes")?.value || "",
  };
  if (pending.summary?.warnings?.length) {
    extra.use_despite_warnings = document.getElementById("trial-override")?.checked === true;
  }
  try {
    if (pending.kind === "movement") {
      extra.direction = document.getElementById("trial-direction").value;
      extra.measured_distance_m = Number(document.getElementById("trial-distance").value);
    } else if (pending.kind === "steering_drift") {
      const forward = Number(document.getElementById("drift-forward").value);
      const onLine = document.getElementById("drift-online")?.checked === true;
      extra.measured_forward_m = forward;
      if (onLine) {
        extra.measured_lateral_m = 0;
      } else {
        const rawOffset = document.getElementById("drift-side-mag")?.value ?? "";
        const offset = Number(rawOffset);
        if (rawOffset.trim() === "" || !Number.isFinite(offset)) {
          toast("Enter the sideways gap B, or tick Ended exactly on the line.", "error");
          return;
        }
        const magnitude = Math.abs(offset);
        if (magnitude === 0) {
          extra.measured_lateral_m = 0;
        } else {
          const side = document.querySelector('input[name="drift-side"]:checked')?.value;
          if (side !== "left" && side !== "right") {
            toast("Choose left or right of the line.", "error");
            return;
          }
          extra.measured_lateral_m = side === "right" ? -magnitude : magnitude;
        }
      }
    } else if (pending.kind === "steering_left" || pending.kind === "steering_right") {
      const method = app.circleMethod;
      extra.measurement_method = method;
      extra.full_lock = document.getElementById("circle-fulllock")?.checked === true;
      if (method === "inner_tire") {
        extra.measured_inner_diameter_m = Number(document.getElementById("circle-inner").value);
      } else if (method === "rear_tires") {
        extra.measured_inner_diameter_m = Number(document.getElementById("circle-inner").value);
        extra.measured_outer_diameter_m = Number(document.getElementById("circle-outer").value);
      } else {
        extra.measured_diameter_m = Number(document.getElementById("circle-diameter").value);
      }
    }
    if (STEERING_STEPS.includes(pending.kind)) app.steeringTest = pending.kind;
    await api("accept_capture", extra);
    app.halfLockNext = false;
    toast("Run accepted and saved on the car.");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.discardPending = async function discardPending() {
  if (!confirm("Discard this run? It cannot be recovered.")) return;
  try {
    await api("discard_capture");
    app.halfLockNext = false;
    toast("Run discarded.");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.generateReport = async function generateReport() {
  const session = app.snapshot?.session;
  if (session && remainingSteps(session).length) {
    toast("Finish every test before building the report.", "error");
    return;
  }
  try {
    await api("generate_report");
    toast("Report saved on the car.");
  } catch (error) {
    toast(error.message, "error");
  }
};

async function applyParameters(extra, question) {
  if (!confirm(question)) return;
  try {
    const result = await api("apply_parameters", {confirmed: true, ...extra});
    const applied = Array.isArray(result.applied) ? result.applied : [result.applied].filter(Boolean);
    toast(`Updated ${applied.map(a => String(a.path || "").split("/").pop()).join(" and ") || "the files"} (backup saved). Restart to use the new values.`);
  } catch (error) {
    toast(error.message, "error");
  }
}

window.applyReport = function applyReport(names) {
  const list = String(names).split(",").filter(Boolean);
  return applyParameters({source: "report", names: list},
    `Write ${list.join(", ")}? ${list.includes("steering_stick_scale") ? "The full-stick angle goes into joy_teleop.yaml; the rest into vesc.yaml. " : ""}A backup is saved first (one per file), and Undo puts them all back. The car only uses the new values after bringup${list.includes("steering_stick_scale") ? " and teleop restart" : " restarts"}.`);
};

window.applyCentreOffset = function applyCentreOffset() {
  return applyParameters({source: "centre"},
    "Write the suggested steering offset into vesc.yaml? A backup is saved first. Restart bringup afterwards, then record another drive-straight run.");
};

window.restoreBackup = async function restoreBackup() {
  if (!confirm("Put back the files from before the last change? Restart bringup (and teleop, if the full-stick angle changed) afterwards.")) return;
  try {
    await api("restore_backup", {confirmed: true});
    toast("Restored from the backup. Restart bringup (and teleop if needed).");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.setLimitStep = function setLimitStep(value) {
  const step = Number(value);
  if (!Number.isFinite(step) || step < 0.005 || step > 0.1) {
    toast("The step must be between 0.005 and 0.1.", "error");
    renderWizardView();
    return;
  }
  app.limitStep = step;
  renderWizardView();
};

window.stepLimit = function stepLimit(side) {
  const limitName = sideLimitName(side);
  const raise = document.getElementById("limit-raise-stick")?.checked === true;
  return applyParameters({source: "limit_step", side, step: app.limitStep, raise_stick: raise},
    `Write the next ${limitName} step for the ${side} side${raise ? ", and raise full stick to reach it" : ""}? A backup is saved first. ` +
    `Then restart bringup${raise ? " and teleop" : ""}, and check with the wheels off the ground before driving: hold LB and full ${side} for 3 seconds. Stop at any buzzing, heat or contact, and press Undo.`);
};

window.writeValueEditor = function writeValueEditor(formId, names) {
  const values = {};
  const lines = [];
  for (const name of String(names).split(",").filter(Boolean)) {
    if (document.getElementById(`${formId}-use-${name}`)?.checked !== true) continue;
    const raw = String(document.getElementById(`${formId}-val-${name}`)?.value ?? "").trim();
    const typed = Number(raw);
    if (raw === "" || !Number.isFinite(typed)) {
      toast(`Type a number for ${EDITABLE[name].label}, or untick it.`, "error");
      return undefined;
    }
    values[name] = EDITABLE[name].degrees ? typed * Math.PI / 180 : typed;
    lines.push(`${EDITABLE[name].label}: ${shownValue(name, currentValues()[name]) || "?"} → ${raw}`);
  }
  if (!lines.length) {
    toast("Tick at least one value to write.", "error");
    return undefined;
  }
  return applyParameters({source: "manual", values},
    `Write these values? A backup is saved first, and Undo puts it back.\n\n${lines.join("\n")}\n\nRestart bringup (and teleop, for full stick) for the car to use them.`);
};

window.saveRearTrack = async function saveRearTrack() {
  const value = Number(document.getElementById("rear-track")?.value);
  if (!Number.isFinite(value)) {
    toast("Type the rear track in metres.", "error");
    return;
  }
  try {
    await api("update_vehicle", {rear_track_m: value});
    toast("Rear track saved for this session.");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.setTrialOverride = async function setTrialOverride(trialId, value) {
  try {
    await api("set_trial_override", {trial_id: trialId, value: value === true});
    toast(value ? "This run is now used despite its warnings." : "This run's warnings apply again.");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.restoreEverything = async function restoreEverything() {
  if (!confirm("Put every file back the way it was before this session's first change? Restart bringup and teleop afterwards.")) return;
  try {
    await api("restore_backup", {confirmed: true, everything: true});
    toast("Everything from this session undone. Restart bringup and teleop.");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.deleteTrial = async function deleteTrial(trialId) {
  if (!confirm("Remove this accepted run from the calibration? The archived event log will record the removal.")) return;
  try {
    await api("delete_trial", {trial_id: trialId});
    toast("Run removed.");
  } catch (error) {
    toast(error.message, "error");
  }
};

document.getElementById("new-session-button").addEventListener("click", openCalibrationSetup);

connectSocket();
fetchState();
setInterval(() => {
  if (!app.connected) fetchState();
}, 2500);
