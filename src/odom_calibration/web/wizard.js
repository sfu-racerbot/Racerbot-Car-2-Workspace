"use strict";

const app = {
  snapshot: null,
  connected: false,
  socket: null,
  reconnectTimer: null,
  wheelSelected: true,
  steeringSelected: true,
  steeringTest: "steering_drift",
};

const MODE_STAGES = {
  movement: ["preflight", "stationary", "movement", "report"],
  movement_steering: ["preflight", "stationary", "movement", "steering", "report"],
  steering: ["preflight", "steering", "report"],
};

const STEERING_TESTS = [
  ["steering_drift", "Straight-line drift"],
  ["steering_left", "Left circle"],
  ["steering_right", "Right circle"],
];

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
  renderConnection();
  renderNav();
  renderTelemetry();
  const nextKey = renderKey(snapshot);
  if (
    previousKey !== nextKey ||
    snapshot.session?.stage === "preflight" ||
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

function railButton(stage, label, plate, active, done) {
  return `<button class="step-button ${active ? "active" : ""}" type="button" onclick="setStage('${stage}')">
    <span class="race-plate ${done ? "done" : ""}">${done ? `${escapeHtml(plate)} done` : escapeHtml(plate)}</span>
    <span>${escapeHtml(label)}${done ? ' <span class="done-mark">Done</span>' : ""}</span>
  </button>`;
}

function railGroup(heading, inner) {
  return `<div class="rail-group"><h2 class="rail-group-heading">${escapeHtml(heading)}</h2>
    <div class="rail-rule" aria-hidden="true"><span></span><span></span><span></span></div>
    ${inner}</div>`;
}

function renderNav() {
  const nav = document.getElementById("step-nav");
  const button = document.getElementById("new-session-button");
  const session = app.snapshot?.session;
  button.hidden = !session;
  if (!nav) return;
  if (!session) {
    nav.innerHTML = `<button class="step-button active" type="button">
      <span class="race-plate">Setup</span><span>Setup</span>
    </button>`;
    return;
  }
  const counts = trialCounts(session);
  const full = fullLockCounts(session);
  const stage = session.stage;
  const preflightDone = stage !== "preflight";
  const steeringDone = full.steering_left >= 1 && full.steering_right >= 1;
  const reportDone = Boolean(session.report);
  let html = railButton("preflight", "Preflight", "Preflight", stage === "preflight", preflightDone);
  if (hasWheel(session)) {
    html += railGroup("Wheel calibration",
      railButton("stationary", "Stationary baseline", "1", stage === "stationary", counts.stationary >= 1) +
      railButton("movement", "Distance runs", "2", stage === "movement", counts.movement >= 2));
  }
  if (hasSteering(session)) {
    html += railGroup("Steering calibration",
      railButton("steering", "Steering tests", "1", stage === "steering", steeringDone));
  }
  html += railButton("report", "Report", "Report", stage === "report", reportDone);
  nav.innerHTML = html;
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
  const steerText = angle === null ? "--" : `${deg(Math.abs(angle))} deg ${angle >= 0 ? "left" : "right"}`;
  const erpmText = finite(telemetry.raw_forward_erpm) ? signed(telemetry.raw_forward_erpm, 0) : "--";

  const set = (id, value) => { const el = document.getElementById(id); if (el) el.textContent = value; };
  set("lb-state", lbText);
  set("live-speed", speedText);
  set("live-servo", servoText);
  set("live-steer", steerText);
  set("live-erpm", erpmText);
  set("live-strip-lb", lbHeld ? "LB held" : "LB released");
  set("live-strip-speed", finite(telemetry.odom_speed) ? `${signed(telemetry.odom_speed, 2)} m/s` : "--");
  set("live-strip-servo", finite(telemetry.servo) ? telemetry.servo.toFixed(4) : "--");

  const list = document.getElementById("topic-health");
  if (list) {
    list.innerHTML = Object.entries(health).map(([name, item]) => {
      const rate = finite(item.rate_hz) ? `${item.rate_hz.toFixed(1)} Hz` : "no rate";
      const age = finite(item.age_sec) ? `${item.age_sec.toFixed(2)} s old` : "no data";
      const required = name === "odom" ? "required" : "optional";
      return `<li><span class="topic-square ${escapeHtml(item.status)}"></span>
        <span class="topic-name">${escapeHtml(item.label || name)}</span>
        <span class="topic-detail">${escapeHtml(rate)}, ${escapeHtml(age)}, ${required}</span></li>`;
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
  if (app.wheelSelected && app.steeringSelected) return "movement_steering";
  if (app.wheelSelected) return "movement";
  if (app.steeringSelected) return "steering";
  return null;
}

function renderSetup() {
  const live = app.snapshot?.live_parameters || {
    speed_to_erpm_gain: -4614,
    speed_to_erpm_offset: 0,
    steering_angle_to_servo_gain: -1.2135,
    steering_angle_to_servo_offset: 0.5304,
    wheelbase: 0.36,
  };
  const mode = selectedMode();
  return `<section>
    ${pageHead("Setup", "Calibration")}
    <p class="lead">Pick what to calibrate, check the current values, and start. You drive the car with the remote; this page only records.</p>
    <div class="choice-list">
      <button class="slot" type="button" aria-pressed="${app.wheelSelected}" onclick="toggleSetup('wheel')">
        <strong>Wheel calibration</strong>
        <span>Stationary baseline and tape-measured distance runs. Corrects the distance the car thinks it travelled. About 15 minutes.</span>
      </button>
      <button class="slot" type="button" aria-pressed="${app.steeringSelected}" onclick="toggleSetup('steering')">
        <strong>Steering calibration</strong>
        <span>A straight-line drift run and left and right circles. Corrects where centre is and how far the wheels turn. About 15 minutes.</span>
      </button>
    </div>
    <h2>Current values</h2>
    <p class="muted small">Values were ${escapeHtml(app.snapshot?.live_parameter_status || "loaded from defaults")}.</p>
    <div class="form-grid">${parameterInputs(live)}</div>
    <div class="button-row">
      <button class="button" type="button" ${mode ? "" : "disabled"} onclick="createSession()">Start calibration</button>
      ${mode ? "" : `<span class="small muted">Pick at least one calibration.</span>`}
    </div>
  </section>`;
}

function topicRow(name, item, required) {
  const rate = finite(item?.rate_hz) ? `${item.rate_hz.toFixed(1)} Hz` : "no rate";
  const age = finite(item?.age_sec) ? `${item.age_sec.toFixed(2)} s old` : "no data";
  return `<li><span class="topic-square ${escapeHtml(item?.status || "missing")}"></span>
    <span class="topic-name">${escapeHtml(item?.label || name)}</span>
    <span class="topic-detail">${escapeHtml(rate)}, ${escapeHtml(age)}, ${required ? "required" : "optional"}</span></li>`;
}

function renderPreflight() {
  const health = app.snapshot.health || {};
  const odomReady = ["good", "warning"].includes(health?.odom?.status);
  const next = hasWheel(app.snapshot.session) ? "stationary" : "steering";
  return `<section>
    ${pageHead("Preflight", "Preflight")}
    <p class="lead">Start the normal bringup and keep the car still with the remote on.</p>
    <ul class="topic-health">
      ${topicRow("odom", health.odom, true)}
      ${topicRow("vesc", health.vesc, false)}
      ${topicRow("servo", health.servo, false)}
      ${topicRow("drive", health.drive, false)}
      ${topicRow("joy", health.joy, false)}
    </ul>
    <ol class="instruction-list">
      <li>Put the car on level ground with clear space around it.</li>
      <li>Turn on the remote and start bringup. Leave the sticks centred.</li>
      <li>If odometry shows missing, hold LB for a moment with the sticks centred.</li>
    </ol>
    <div class="button-row">
      <button class="button" type="button" ${odomReady ? "" : "disabled"} onclick="setStage('${next}')">Continue</button>
      ${odomReady ? "" : `<span class="small muted">Waiting for odometry</span>`}
    </div>
  </section>`;
}

function kindLabel(kind) {
  if (kind === "stationary") return "Stationary";
  if (kind === "movement") return "Distance run";
  if (kind === "steering_drift") return "Drift";
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
    if (trial.measurement_method === "rear_tires"
        && finite(trial.measured_inner_diameter_m) && finite(trial.measured_outer_diameter_m)) {
      return `tires ${trial.measured_inner_diameter_m.toFixed(2)} and ${trial.measured_outer_diameter_m.toFixed(2)} m`;
    }
    if (finite(trial.measured_diameter_m)) return `axle ${trial.measured_diameter_m.toFixed(3)} m`;
  }
  if (trial.kind === "steering_drift" && finite(trial.measured_forward_m)) {
    return `${trial.measured_forward_m.toFixed(2)} m forward`;
  }
  return "No tape measure";
}

function acceptedRunsTable(session, kinds) {
  const trials = (session.trials || []).filter(t => t.accepted && kinds.includes(t.kind));
  if (!trials.length) return `<p class="small muted">No accepted runs yet.</p>`;
  return `<table class="data-table">
    <thead><tr><th>#</th><th>Test</th><th>Measurement</th><th>Servo</th><th>Odom</th><th></th></tr></thead>
    <tbody>${trials.map((trial, index) => `<tr>
      <td><span class="race-plate">${index + 1}</span></td>
      <td>${escapeHtml(kindLabel(trial.kind))}</td>
      <td>${trialMeasurement(trial)}</td>
      <td class="num">${fmt(trial.summary?.servo?.median, 4)}</td>
      <td class="num">${signed(trial.summary?.odom_distance_m, 3)} m</td>
      <td><button class="button button-ghost button-small" type="button"
        onclick="deleteTrial('${escapeHtml(trial.id)}')">Remove</button></td>
    </tr>`).join("")}</tbody>
  </table>`;
}

function plate(label, done, extra = "") {
  return `<span class="race-plate ${done ? "done" : ""}">${escapeHtml(label)}${done ? " done" : ` ${extra}`}</span>`;
}

function renderStationary(session) {
  const counts = trialCounts(session);
  return `<section>
    ${pageHead("Wheel calibration, step 1 of 2", "Stationary baseline")}
    <p class="lead">Measures the raw motor reading when nothing moves.</p>
    <ol class="instruction-list">
      <li>Release LB and leave the sticks centred.</li>
      <li>Press Record, do not touch the car for 5 seconds, then press Stop.</li>
    </ol>
    <div class="button-row">
      <button class="button" type="button" onclick="startCapture('stationary')">Record</button>
      ${counts.stationary ? `<button class="button button-ghost" type="button" onclick="setStage('movement')">Continue</button>` : ""}
    </div>
  </section>
  <section>
    <h2>Accepted runs</h2>
    ${acceptedRunsTable(session, ["stationary"])}
  </section>`;
}

function renderMovement(session) {
  const counts = trialCounts(session);
  const forward = hasSteering(session)
    ? `<button class="button button-ghost" type="button" onclick="setStage('steering')">Go to steering calibration</button>`
    : `<button class="button button-ghost" type="button" onclick="setStage('report')">Build report</button>`;
  return `<section>
    ${pageHead("Wheel calibration, step 2 of 2", "Distance runs")}
    <p class="lead">Drive a measured straight line; the wizard compares it with what the car counted.</p>
    <ol class="instruction-list">
      <li>Tape out a straight 5 to 10 m lane.</li>
      <li>Mark the rear-axle centre at the start.</li>
      <li>Press Record before moving. Hold LB and drive smoothly to the end mark.</li>
      <li>Stop, release LB, press Stop, then measure.</li>
    </ol>
    <p>${plate("Run 1", counts.movement >= 1)} ${plate("Run 2", counts.movement >= 2)} ${plate("Run 3", counts.movement >= 3, "recommended")}</p>
    <p class="small muted">Two forward and one reverse is best.</p>
    <div class="button-row">
      <button class="button" type="button" onclick="startCapture('movement')">Record</button>
      ${counts.movement >= 1 ? forward : ""}
    </div>
  </section>
  <section>
    <h2>Accepted runs</h2>
    ${acceptedRunsTable(session, ["movement"])}
  </section>`;
}

function steeringInstructions(kind) {
  if (kind === "steering_drift") {
    return `<ol class="instruction-list">
      <li>Tape a straight line at least 6 m long.</li>
      <li>Put the rear-axle centre on the line with the car pointing along it.</li>
      <li>Press Record. Hold LB and drive slowly about 5 m forward without touching the steering stick.</li>
      <li>Stop, release LB, press Stop.</li>
      <li>Measure how far along the line the rear-axle centre is, and how far it ended up to the left or right of the line.</li>
    </ol>`;
  }
  const side = kind === "steering_left" ? "left" : "right";
  return `<ol class="instruction-list">
    <li>Mark the ground under the rear-axle centre.</li>
    <li>Press Record. Hold LB, hold the steering stick fully ${side}, and drive one slow full circle back to your mark.</li>
    <li>Stop, release LB, press Stop.</li>
    <li>Measure the circle. Either measure the diameter traced by the rear-axle centre, or measure the inner and outer rear-tire circles; the wizard takes the midpoint.</li>
  </ol>
  <p class="tip">A second circle with the stick half over shows whether the steering is linear.</p>`;
}

function clientTrialPoints(session, wheelbase) {
  const points = [];
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
      if (trial.measurement_method === "rear_tires"
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

function linearTicks(min, max, count) {
  if (!(min < max)) return [min];
  const ticks = [];
  for (let i = 0; i < count; i++) ticks.push(min + (max - min) * i / (count - 1));
  return ticks;
}

function renderFitChart(points, line, limits) {
  if (!points.length) return "";
  const W = 380, H = 250, L = 48, R = 12, T = 12, B = 36;
  const xs = points.map(p => p.x * 180 / Math.PI);
  const ys = points.map(p => p.y);
  let x0 = Math.min(...xs), x1 = Math.max(...xs);
  let y0 = Math.min(...ys), y1 = Math.max(...ys);
  if (limits) { y0 = Math.min(y0, ...limits); y1 = Math.max(y1, ...limits); }
  if (line) {
    const ends = [x0 * Math.PI / 180, x1 * Math.PI / 180]
      .map(a => line.slope * a + line.intercept);
    y0 = Math.min(y0, ...ends); y1 = Math.max(y1, ...ends);
  }
  const xPad = Math.max((x1 - x0) * 0.15, 1);
  const yPad = Math.max((y1 - y0) * 0.15, 0.005);
  x0 -= xPad; x1 += xPad; y0 -= yPad; y1 += yPad;
  const X = v => L + (v - x0) / (x1 - x0) * (W - L - R);
  const Y = v => T + (1 - (v - y0) / (y1 - y0)) * (H - T - B);

  const pointShape = p => {
    const cx = X(p.x * 180 / Math.PI).toFixed(1);
    const cy = Y(p.y).toFixed(1);
    if (p.kind === "steering_left") {
      return `<circle cx="${cx}" cy="${cy}" r="4" style="fill:var(--blue)"/>`;
    }
    if (p.kind === "steering_right") {
      const px = X(p.x * 180 / Math.PI);
      const py = Y(p.y);
      return `<polygon points="${px.toFixed(1)},${(py - 5).toFixed(1)} ${(px - 4.5).toFixed(1)},${(py + 3.5).toFixed(1)} ${(px + 4.5).toFixed(1)},${(py + 3.5).toFixed(1)}" style="fill:var(--blue)"/>`;
    }
    if (p.kind === "steering_center") {
      return `<rect x="${(cx - 4).toFixed(1)}" y="${(cy - 4).toFixed(1)}" width="8" height="8" style="fill:var(--paper);stroke:var(--navy)"/>`;
    }
    return `<rect x="${(cx - 4).toFixed(1)}" y="${(cy - 4).toFixed(1)}" width="8" height="8" style="fill:var(--navy)"/>`;
  };

  let inner = `<line x1="${L}" y1="${Y(0) > T && Y(0) < H - B ? Y(0) : H - B}" x2="${W - R}" y2="${Y(0) > T && Y(0) < H - B ? Y(0) : H - B}" style="stroke:var(--line-2)"/>`;
  if (line) {
    inner += `<line x1="${X(x0).toFixed(1)}" y1="${Y(line.slope * x0 * Math.PI / 180 + line.intercept).toFixed(1)}"
      x2="${X(x1).toFixed(1)}" y2="${Y(line.slope * x1 * Math.PI / 180 + line.intercept).toFixed(1)}"
      style="stroke:var(--blue);stroke-width:2"/>`;
  }
  if (limits) {
    for (const limit of limits) {
      if (limit < y0 || limit > y1) continue;
      inner += `<line x1="${L}" y1="${Y(limit).toFixed(1)}" x2="${W - R}" y2="${Y(limit).toFixed(1)}"
        style="stroke:var(--bad);stroke-dasharray:5 4"/><text x="${W - R}" y="${(Y(limit) - 4).toFixed(1)}"
        text-anchor="end" font-size="10" style="fill:var(--muted)">servo limit</text>`;
    }
  }
  inner += points.map(pointShape).join("");
  const xTicks = linearTicks(x0, x1, 4).map(t =>
    `<text x="${X(t).toFixed(1)}" y="${H - B + 16}" text-anchor="middle" font-size="10" class="tick" style="fill:var(--muted)">${t.toFixed(1)}</text>`).join("");
  const yTicks = linearTicks(y0, y1, 4).map(t =>
    `<text x="${L - 6}" y="${(Y(t) + 3).toFixed(1)}" text-anchor="end" font-size="10" class="tick" style="fill:var(--muted)">${t.toFixed(3)}</text>`).join("");
  return `<figure class="figure"><svg class="diagram" viewBox="0 0 ${W} ${H}" role="img"
      aria-label="Servo value against steering angle">
    <text x="${L}" y="${H - 6}" font-size="11" style="fill:var(--muted)">Steering angle (deg)</text>
    <text x="10" y="${T + 6}" font-size="11" style="fill:var(--muted)">Servo</text>
    ${xTicks}${yTicks}${inner}</svg>
    <figcaption>Each point is one test. The line is the suggested setting.</figcaption></figure>`;
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
  const readout = raw === null
    ? "Implied steering not available (check the current settings)"
    : `Implied steering ${raw >= 0 ? "+" : ""}${(raw * 180 / Math.PI).toFixed(1)} deg ${raw >= 0 ? "left" : "right"} (from current settings)`;
  return `<figure class="figure"><svg id="sd-svg" class="diagram" width="320" height="220" viewBox="0 0 320 220"
      role="img" aria-label="Top-down steering diagram">
    <rect x="118" y="28" width="84" height="150" style="fill:var(--paper);stroke:var(--navy);stroke-width:2"/>
    <line x1="106" y1="160" x2="214" y2="160" style="stroke:var(--navy);stroke-width:2"/>
    <rect x="94" y="144" width="20" height="32" style="fill:var(--paper);stroke:var(--navy);stroke-width:2"/>
    <rect x="206" y="144" width="20" height="32" style="fill:var(--paper);stroke:var(--navy);stroke-width:2"/>
    <line x1="106" y1="58" x2="214" y2="58" style="stroke:var(--navy);stroke-width:2"/>
    <g id="sd-wheel-fl" transform="rotate(${rot} 106 58)">
      <rect x="96" y="42" width="20" height="32" style="fill:var(--paper);stroke:var(--navy);stroke-width:2"/></g>
    <g id="sd-wheel-fr" transform="rotate(${rot} 214 58)">
      <rect x="204" y="42" width="20" height="32" style="fill:var(--paper);stroke:var(--navy);stroke-width:2"/></g>
    <path id="sd-path" d="${pathD}" style="stroke:var(--blue);stroke-dasharray:6 5;stroke-width:2;fill:none"/>
  </svg>
  <figcaption id="implied-readout">${escapeHtml(readout)}</figcaption></figure>${progressLine}`;
}

function updateDiagramLive() {
  if (!document.getElementById("sd-svg")) return;
  const raw = impliedSteeringAngle();
  const readout = document.getElementById("implied-readout");
  if (readout) {
    readout.textContent = raw === null
      ? "Implied steering not available (check the current settings)"
      : `Implied steering ${raw >= 0 ? "+" : ""}${(raw * 180 / Math.PI).toFixed(1)} deg ${raw >= 0 ? "left" : "right"} (from current settings)`;
  }
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

function renderSteering(session) {
  const counts = trialCounts(session);
  const full = fullLockCounts(session);
  const test = STEERING_TESTS.some(([kind]) => kind === app.steeringTest)
    ? app.steeringTest : "steering_drift";
  const testName = STEERING_TESTS.find(([kind]) => kind === test)[1];
  const wheelbase = currentParams().wheelbase;
  const points = clientTrialPoints(session, wheelbase);
  const fit = linearFit(points);
  const ready = full.steering_left >= 1 && full.steering_right >= 1;
  return `<section>
    ${pageHead("Steering calibration", "Steering")}
    <p class="lead">Three kinds of test: a straight run finds true centre, and circles to each side find how far the wheels really turn.</p>
    <div class="segmented" role="group" aria-label="Steering test">
      ${STEERING_TESTS.map(([kind, label]) =>
        `<button type="button" aria-pressed="${test === kind}" onclick="selectSteeringTest('${kind}')">${escapeHtml(label)}</button>`).join("")}
    </div>
    ${steeringInstructions(test)}
    <p>${plate("Drift", full.steering_drift >= 1, "needed")}
      ${plate("Left full lock", full.steering_left >= 1, "needed")}
      ${plate("Right full lock", full.steering_right >= 1, "needed")}</p>
    ${renderSteeringDiagram()}
    ${points.length >= 2 ? renderFitChart(points, fit, servoLimits()) : ""}
    <div class="button-row">
      <button class="button" type="button" onclick="startCapture('${test}')">Record ${escapeHtml(testName.toLowerCase())}</button>
      ${ready ? `<button class="button button-ghost" type="button" onclick="setStage('report')">Build report</button>` : ""}
    </div>
  </section>
  <section>
    <h2>Accepted runs</h2>
    ${acceptedRunsTable(session, ["steering_drift", "steering_left", "steering_right", "steering_center"])}
  </section>`;
}

function renderRecording(session) {
  const active = session.active_capture;
  const progress = app.snapshot.capture_progress;
  const isCircle = active.kind === "steering_left" || active.kind === "steering_right";
  const dist = finite(progress?.odom_distance_m) ? `${progress.odom_distance_m.toFixed(2)} m` : "--";
  const turns = isCircle && finite(progress?.odom_yaw_rad)
    ? `${(progress.odom_yaw_rad / (2 * Math.PI)).toFixed(2)} turns (odometry estimate)` : "";
  return `<section>
    ${pageHead(session.stage === "steering" ? "Steering calibration" : "Wheel calibration", `Recording ${kindLabel(active.kind).toLowerCase()}`)}
    <div class="recording-timer"><span class="rec-dot" aria-hidden="true"></span>
      <span id="capture-timer">${fmt(app.snapshot.capture_duration_sec, 1, "--")} s</span></div>
    <p class="mono">Distance <span id="capture-dist">${escapeHtml(dist)}</span>${turns ? `, <span id="capture-turns">${escapeHtml(turns)}</span>` : `<span id="capture-turns"></span>`}</p>
    <p>Stop the car and release LB before pressing Stop.</p>
    <div class="button-row">
      <button class="button button-danger" type="button" onclick="stopCapture()">Stop</button>
    </div>
  </section>`;
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

function driftForm() {
  return `<div class="field">
      <label class="field-label" for="drift-forward">Forward distance along the line (m)</label>
      <input id="drift-forward" class="input" type="number" min="0.01" step="0.01">
    </div>
    <div class="field">
      <label class="field-label" for="drift-side-mag">Sideways offset at the end (m)</label>
      <input id="drift-side-mag" class="input" type="number" min="0" step="0.01" value="0">
    </div>
    <div class="radio-row" role="radiogroup" aria-label="Side of the line">
      <label><input type="radio" name="drift-side" value="left" checked> Left of the line</label>
      <label><input type="radio" name="drift-side" value="right"> Right of the line</label>
    </div>
    <div class="check-row">
      <label><input id="drift-online" type="checkbox"> Ended exactly on the line</label>
    </div>`;
}

function circleForm() {
  return `<div class="radio-row" role="radiogroup" aria-label="Circle measurement method">
      <label><input type="radio" name="circle-method" value="axle_centre" checked
        onchange="toggleCircleMethod()"> Rear-axle centre</label>
      <label><input type="radio" name="circle-method" value="rear_tires"
        onchange="toggleCircleMethod()"> Inner and outer rear tires</label>
    </div>
    <div id="circle-axle-field" class="field">
      <label class="field-label" for="circle-diameter">Diameter (m)</label>
      <input id="circle-diameter" class="input" type="number" min="0.1" step="0.01">
    </div>
    <div id="circle-tire-fields" hidden>
      <div class="field">
        <label class="field-label" for="circle-inner">Inner tire circle diameter (m)</label>
        <input id="circle-inner" class="input" type="number" min="0.1" step="0.01">
      </div>
      <div class="field">
        <label class="field-label" for="circle-outer">Outer tire circle diameter (m)</label>
        <input id="circle-outer" class="input" type="number" min="0.1" step="0.01">
      </div>
    </div>
    <div class="check-row">
      <label><input id="circle-fulllock" type="checkbox" checked> Stick was held fully over (full lock)</label>
    </div>`;
}

function movementForm() {
  return `<div class="field">
      <label class="field-label" for="trial-direction">Direction</label>
      <select id="trial-direction" class="select">
        <option value="forward">Forward</option>
        <option value="reverse">Reverse</option>
      </select>
    </div>
    <div class="field">
      <label class="field-label" for="trial-distance">Measured distance (m)</label>
      <input id="trial-distance" class="input" type="number" min="0.01" step="0.001" value="5">
    </div>`;
}

function renderReview(session) {
  const pending = session.pending_capture;
  const summary = pending.summary || {};
  let form = "";
  if (pending.kind === "movement") form = movementForm();
  else if (pending.kind === "steering_drift") form = driftForm();
  else if (pending.kind === "steering_left" || pending.kind === "steering_right") form = circleForm();
  return `<section>
    ${pageHead("Check this run", "Check this run")}
    ${summaryDefs(summary)}
    ${warningsBlock(summary.warnings)}
    ${form}
    <div class="field">
      <label class="field-label" for="trial-notes">Notes</label>
      <textarea id="trial-notes" class="textarea"></textarea>
    </div>
    <div class="check-row">
      <label><input id="trial-confirmed" type="checkbox"> These measurements are for this run.</label>
    </div>
    <div class="button-row">
      <button class="button" type="button" onclick="acceptPending()">Accept run</button>
      <button class="button button-ghost" type="button" onclick="discardPending()">Discard</button>
    </div>
  </section>`;
}

function statusPlate(status) {
  const ready = status === "ready" || status === "good" || status === "high";
  return `<p class="status-plate ${ready ? "good" : ""}">${ready ? "Ready" : "Needs review"}</p>`;
}

function paramTable(current, suggestions, keys) {
  const rows = keys.filter(name => name in suggestions).map(name => {
    const now = current[name];
    const suggested = suggestions[name];
    const change = finite(now) && finite(suggested) ? signed(suggested - now, 6) : "Not enough data";
    return `<tr><td class="mono">${escapeHtml(name)}</td>
      <td class="num">${fmt(now, 6)}</td>
      <td class="num">${fmt(suggested, 6)}</td>
      <td class="num">${escapeHtml(change)}</td></tr>`;
  });
  if (!rows.length) return `<p class="small muted">No suggestions in this section.</p>`;
  return `<table class="data-table">
    <thead><tr><th>Parameter</th><th>Now</th><th>Suggested</th><th>Change</th></tr></thead>
    <tbody>${rows.join("")}</tbody></table>`;
}

function yamlBlock(suggestions, keys) {
  const lines = keys.filter(name => name in suggestions).map(name => {
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

function renderReport(session) {
  const report = session.report;
  if (!report) {
    const back = hasSteering(session) ? "steering" : "movement";
    return `<section>
      ${pageHead("Report", "Report")}
      <p class="lead">Generate the report to turn the accepted runs into suggestions.</p>
      <div class="button-row">
        <button class="button" type="button" onclick="generateReport()">Generate report</button>
        <button class="button button-ghost" type="button" onclick="setStage('${back}')">Back to tests</button>
      </div>
    </section>`;
  }
  const suggestions = report.parameter_suggestions || {};
  const current = report.current_parameters || {};
  let html = `<section>${pageHead("Report", "Report")}${statusPlate(report.overall_status)}
    <p class="lead">${escapeHtml(report.safety_note)}</p></section>`;

  if (report.movement) {
    const movement = report.movement;
    html += `<section><h2>Wheel calibration</h2>${sectorRule()}${statusPlate(movement.status)}
      <p>${movement.usable_trial_count} usable of ${movement.accepted_trial_count} accepted runs.</p>
      ${paramTable(current, suggestions, ["speed_to_erpm_gain", "speed_to_erpm_offset"])}
      ${warningsBlock(movement.warnings)}
      <p class="yaml-label">Put these in the vesc_to_odom_node section of vesc.yaml (odometry only)</p>
      ${yamlBlock(suggestions, ["speed_to_erpm_gain", "speed_to_erpm_offset"])}
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
        ["steering_angle_to_servo_gain", "steering_angle_to_servo_offset"])}
      ${warningsBlock(steering.warnings)}
      <p class="yaml-label">Put these in the shared steering values in vesc.yaml</p>
      ${yamlBlock(suggestions,
        ["steering_angle_to_servo_gain", "steering_angle_to_servo_offset"])}
    </section>`;
  }

  const wheel = report.movement ? `<button class="button button-ghost" type="button" onclick="setStage('movement')">Add a wheel run</button>` : "";
  const steer = report.steering ? `<button class="button button-ghost" type="button" onclick="setStage('steering')">Add a steering test</button>` : "";
  html += `<section><div class="button-row">
      <a class="button" href="/api/report/md">Download Markdown</a>
      <a class="button button-ghost" href="/api/report/json">Download JSON</a>
      <button class="button button-ghost" type="button" onclick="generateReport()">Recalculate</button>
      ${wheel}${steer}
    </div></section>`;
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
  if (!session) {
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
  if (session.stage === "preflight") view.innerHTML = renderPreflight();
  else if (session.stage === "stationary") view.innerHTML = hasWheel(session) ? renderStationary(session) : renderPreflight();
  else if (session.stage === "movement") view.innerHTML = hasWheel(session) ? renderMovement(session) : renderPreflight();
  else if (session.stage === "steering") view.innerHTML = hasSteering(session) ? renderSteering(session) : renderPreflight();
  else if (session.stage === "report") view.innerHTML = renderReport(session);
  else view.innerHTML = renderPreflight();
}

window.toggleSetup = function toggleSetup(which) {
  if (which === "wheel") app.wheelSelected = !app.wheelSelected;
  if (which === "steering") app.steeringSelected = !app.steeringSelected;
  renderWizardView();
};

window.selectSteeringTest = function selectSteeringTest(kind) {
  app.steeringTest = kind;
  renderWizardView();
};

window.toggleCircleMethod = function toggleCircleMethod() {
  const method = document.querySelector('input[name="circle-method"]:checked')?.value;
  const axle = document.getElementById("circle-axle-field");
  const tires = document.getElementById("circle-tire-fields");
  if (axle) axle.hidden = method !== "axle_centre";
  if (tires) tires.hidden = method !== "rear_tires";
};

window.createSession = async function createSession() {
  const mode = selectedMode();
  if (!mode) {
    toast("Pick at least one calibration.", "error");
    return;
  }
  try {
    await api("new_session", {
      mode,
      current_parameters: currentParameterValues(),
    });
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
  try {
    await api("start_capture", {kind});
  } catch (error) {
    const health = error.details?.health;
    const suffix = health?.odom ? ` Odom is ${health.odom.status}.` : "";
    toast(`${error.message}${suffix}`, "error");
  }
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
  try {
    if (pending.kind === "movement") {
      extra.direction = document.getElementById("trial-direction").value;
      extra.measured_distance_m = Number(document.getElementById("trial-distance").value);
    } else if (pending.kind === "steering_drift") {
      const forward = Number(document.getElementById("drift-forward").value);
      const onLine = document.getElementById("drift-online")?.checked === true;
      const magnitude = Math.abs(Number(document.getElementById("drift-side-mag").value));
      const side = document.querySelector('input[name="drift-side"]:checked')?.value;
      extra.measured_forward_m = forward;
      if (onLine) extra.measured_lateral_m = 0;
      else extra.measured_lateral_m = side === "right" ? -magnitude : magnitude;
    } else if (pending.kind === "steering_left" || pending.kind === "steering_right") {
      const method = document.querySelector('input[name="circle-method"]:checked')?.value || "axle_centre";
      extra.measurement_method = method;
      extra.full_lock = document.getElementById("circle-fulllock")?.checked === true;
      if (method === "rear_tires") {
        extra.measured_inner_diameter_m = Number(document.getElementById("circle-inner").value);
        extra.measured_outer_diameter_m = Number(document.getElementById("circle-outer").value);
      } else {
        extra.measured_diameter_m = Number(document.getElementById("circle-diameter").value);
      }
    }
    await api("accept_capture", extra);
    toast("Run accepted and saved on the car.");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.discardPending = async function discardPending() {
  if (!confirm("Discard this run? It cannot be recovered.")) return;
  try {
    await api("discard_capture");
    toast("Run discarded.");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.generateReport = async function generateReport() {
  try {
    await api("generate_report");
    toast("Report saved on the car.");
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

document.getElementById("new-session-button").addEventListener("click", async () => {
  if (!confirm("Start a new session? The current session will be archived on the car before replacement.")) return;
  try {
    const mode = app.snapshot?.session?.mode || "movement";
    const params = app.snapshot?.session?.current_parameters || app.snapshot.live_parameters;
    await api("new_session", {
      mode,
      current_parameters: params,
      replace_existing: true,
    });
    toast("Session started.");
  } catch (error) {
    toast(error.message, "error");
  }
});

connectSocket();
fetchState();
setInterval(() => {
  if (!app.connected) fetchState();
}, 2500);
