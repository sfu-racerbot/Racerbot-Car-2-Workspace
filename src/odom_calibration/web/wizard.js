"use strict";

const app = {
  snapshot: null,
  connected: false,
  socket: null,
  reconnectTimer: null,
  selectedMode: "movement",
};

const steps = [
  ["preflight", "Preflight"],
  ["stationary", "Stationary"],
  ["movement", "Movement"],
  ["steering", "Steering"],
  ["report", "Report"],
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

function fmt(value, digits = 2, fallback = "—") {
  return finite(value) ? value.toFixed(digits) : fallback;
}

function signed(value, digits = 3, fallback = "—") {
  return finite(value) ? `${value >= 0 ? "+" : ""}${value.toFixed(digits)}` : fallback;
}

function healthClass(status) {
  if (status === "good") return "good";
  if (status === "warning") return "warning";
  return "bad";
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
  if (timer) timer.textContent = `${fmt(snapshot.capture_duration_sec, 1)} s`;
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

function renderConnection() {
  const pill = document.getElementById("connection-pill");
  if (!pill) return;
  pill.textContent = app.connected ? "Car connected" : "Reconnecting…";
  pill.className = `pill ${app.connected ? "pill-good" : "pill-warn"}`;
}

function trialCounts(session) {
  const counts = {
    stationary: 0,
    movement: 0,
    steering_center: 0,
    steering_left: 0,
    steering_right: 0,
  };
  for (const trial of session?.trials || []) {
    if (trial.accepted && Object.hasOwn(counts, trial.kind)) counts[trial.kind] += 1;
  }
  return counts;
}

function stepComplete(stage, session, counts) {
  if (stage === "preflight") return ["stationary", "movement", "steering", "report"].includes(session.stage);
  if (stage === "stationary") return counts.stationary >= 1;
  if (stage === "movement") return counts.movement >= 2;
  if (stage === "steering") {
    return session.mode === "movement_steering" &&
      counts.steering_left >= 1 && counts.steering_right >= 1;
  }
  return Boolean(session.report);
}

function renderNav() {
  const nav = document.getElementById("step-nav");
  const button = document.getElementById("new-session-button");
  const session = app.snapshot?.session;
  button.hidden = !session;
  if (!session) {
    nav.innerHTML = `<button class="step-button active" type="button">
      <span class="step-number">1</span><span class="step-label">Setup</span>
    </button>`;
    return;
  }
  const counts = trialCounts(session);
  let visibleIndex = 0;
  nav.innerHTML = steps
    .filter(([stage]) => stage !== "steering" || session.mode === "movement_steering")
    .map(([stage, label]) => {
      visibleIndex += 1;
      const active = session.stage === stage;
      const complete = stepComplete(stage, session, counts);
      return `<button class="step-button ${active ? "active" : ""} ${complete ? "complete" : ""}"
        type="button" onclick="setStage('${stage}')">
        <span class="step-number">${complete ? "✓" : visibleIndex}</span>
        <span class="step-label">${label}</span>
      </button>`;
    }).join("");
}

function renderTelemetry() {
  const snapshot = app.snapshot;
  if (!snapshot) return;
  const telemetry = snapshot.telemetry || {};
  const health = snapshot.health || {};
  document.getElementById("telemetry-age").textContent =
    app.connected ? "live" : "offline";
  const lbClass = telemetry.lb_held ? "lb-on" : "lb-off";
  document.getElementById("telemetry-values").innerHTML = `
    <div class="metric">
      <div class="metric-label">Odom speed</div>
      <div class="metric-value">${signed(telemetry.odom_speed, 2)}<span class="metric-unit">m/s</span></div>
    </div>
    <div class="metric">
      <div class="metric-label">Raw ERPM</div>
      <div class="metric-value">${signed(telemetry.raw_forward_erpm, 0)}</div>
    </div>
    <div class="metric">
      <div class="metric-label">Command</div>
      <div class="metric-value">${signed(telemetry.command_speed, 2)}<span class="metric-unit">m/s</span></div>
    </div>
    <div class="metric">
      <div class="metric-label">Servo</div>
      <div class="metric-value">${fmt(telemetry.servo, 3)}</div>
    </div>
    <div class="metric metric-wide ${lbClass}">
      <div class="metric-label">Remote deadman</div>
      <div class="metric-value">${telemetry.lb_held ? "LB HELD" : "LB released"}</div>
    </div>`;
  document.getElementById("topic-health").innerHTML = Object.entries(health)
    .map(([name, item]) => {
      const rate = finite(item.rate_hz) ? `${item.rate_hz.toFixed(1)} Hz` : "—";
      const age = finite(item.age_sec) ? `${item.age_sec.toFixed(2)}s old` : "no data";
      return `<div class="health-row" title="${escapeHtml(name)}">
        <span class="health-light ${escapeHtml(item.status)}"></span>
        <span class="health-name">${escapeHtml(item.label)}</span>
        <span class="health-detail">${rate}<br>${age}</span>
      </div>`;
    }).join("");
}

function parameterInputs(parameters) {
  const fields = [
    ["speed_to_erpm_gain", "Speed → ERPM gain", "1"],
    ["speed_to_erpm_offset", "Speed → ERPM offset", "0.1"],
    ["steering_angle_to_servo_gain", "Steering → servo gain", "0.0001"],
    ["steering_angle_to_servo_offset", "Steering → servo offset", "0.0001"],
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

function renderSetup() {
  const live = app.snapshot?.live_parameters || {
    speed_to_erpm_gain: 4614,
    speed_to_erpm_offset: 0,
    steering_angle_to_servo_gain: -1.2135,
    steering_angle_to_servo_offset: 0.5304,
    wheelbase: 0.324,
  };
  return `<section class="hero-card">
    <div class="eyebrow">Tape measure + human driver</div>
    <h1>Measure the real car. Let the data explain the rest.</h1>
    <p class="lead">This wizard records the car while you drive manually, asks you to confirm what physically happened, and turns signed VESC and odometry readings into reviewable parameter suggestions.</p>
    <div class="choice-grid">
      <label class="choice ${app.selectedMode === "movement" ? "selected" : ""}" onclick="selectMode('movement')">
        <input type="radio" name="mode" value="movement">
        <strong>Movement only</strong>
        <span>Stationary offset, forward/reverse sign, known-distance speed scale, and odometry health.</span>
      </label>
      <label class="choice ${app.selectedMode === "movement_steering" ? "selected" : ""}" onclick="selectMode('movement_steering')">
        <input type="radio" name="mode" value="movement_steering">
        <strong>Movement + steering</strong>
        <span>Add visually centred wheels and tape-measured left/right turning circles.</span>
      </label>
    </div>
  </section>
  <section class="card">
    <h2>Current parameters</h2>
    <p>Values were ${escapeHtml(app.snapshot?.live_parameter_status || "loaded from defaults")}. Verify them before starting; the report compares against this baseline.</p>
    <div class="form-grid">${parameterInputs(live)}</div>
    <div class="button-row">
      <button class="button" type="button" onclick="createSession()">Start guided calibration</button>
    </div>
  </section>`;
}

function preflightCard(name, item, required) {
  const klass = healthClass(item?.status);
  const titleStatus = item?.status || "missing";
  const details = item?.message_count
    ? `${fmt(item.rate_hz, 1)} Hz · ${fmt(item.age_sec, 2)}s old · ${item.invalid_count} invalid`
    : "No messages received yet";
  return `<div class="check-card ${klass}">
    <div class="check-title">
      <span>${escapeHtml(item?.label || name)}</span>
      <span class="report-status ${escapeHtml(titleStatus)}">${escapeHtml(titleStatus)}</span>
    </div>
    <div class="check-detail">${details}${required ? " · required" : " · recommended"}</div>
  </div>`;
}

function renderPreflight(session) {
  const health = app.snapshot.health;
  const odomReady = ["good", "warning"].includes(health?.odom?.status);
  return `<section class="hero-card">
    <div class="eyebrow">Step 1 · Preflight</div>
    <h1>Make sure every signal has a pulse.</h1>
    <p class="lead">Start the normal F1TENTH bringup. Keep the car still, remote throttle neutral, and steering centred. If odometry is missing, briefly hold LB with neutral controls so a servo command reaches <code>vesc_to_odom</code>.</p>
    <div class="check-grid">
      ${preflightCard("odom", health.odom, true)}
      ${preflightCard("vesc", health.vesc, false)}
      ${preflightCard("servo", health.servo, false)}
      ${preflightCard("drive", health.drive, false)}
      ${preflightCard("joy", health.joy, false)}
    </div>
    <ul class="instruction-list">
      <li>Put the car on level ground with clear space around it. Remove anything touching a wheel.</li>
      <li>Turn on the remote and start normal vehicle bringup. Do not lift or push the car during a recording.</li>
      <li>Confirm odom speed and raw ERPM are finite and close to zero while stationary.</li>
    </ul>
    <div class="button-row">
      <button class="button" type="button" ${odomReady ? "" : "disabled"} onclick="setStage('stationary')">Signals checked — continue</button>
      ${odomReady ? "" : `<span class="small muted">Waiting for fresh /odom before movement tests.</span>`}
    </div>
  </section>`;
}

function captureActiveCard(active) {
  return `<section class="card recording-card">
    <div class="recording-line">
      <span class="recording-dot"></span>
      <div>
        <div class="eyebrow">Recording ${escapeHtml(active.kind.replaceAll("_", " "))}</div>
        <div id="capture-timer" class="timer">${fmt(app.snapshot.capture_duration_sec, 1)} s</div>
      </div>
    </div>
    <p>Drive only as instructed. Stop the car and release LB before ending a movement or circle capture.</p>
    <div class="button-row">
      <button class="button button-danger" type="button" onclick="stopCapture()">Stop recording</button>
    </div>
  </section>`;
}

function summaryCells(summary) {
  return `<div class="summary-grid">
    <div class="summary-cell"><span>Duration</span><strong>${fmt(summary.duration_sec, 1)} s</strong></div>
    <div class="summary-cell"><span>Odom distance</span><strong>${signed(summary.odom_distance_m, 3)} m</strong></div>
    <div class="summary-cell"><span>Raw ERPM integral</span><strong>${signed(summary.raw_erpm_integral, 1)}</strong></div>
    <div class="summary-cell"><span>Pose displacement</span><strong>${fmt(summary.pose_displacement_m, 3)} m</strong></div>
    <div class="summary-cell"><span>Odom yaw</span><strong>${signed(summary.odom_yaw_rad, 3)} rad</strong></div>
    <div class="summary-cell"><span>Median servo</span><strong>${fmt(summary.servo?.median, 4)}</strong></div>
  </div>`;
}

function pendingCard(pending, defaultDistance = 5) {
  const summary = pending.summary;
  const warnings = summary.warnings || [];
  const isMovement = pending.kind === "movement";
  const isCircle = ["steering_left", "steering_right"].includes(pending.kind);
  return `<section class="card record-card">
    <div class="eyebrow">Review before accepting</div>
    <h2>Does this match what happened?</h2>
    ${summaryCells(summary)}
    ${warnings.length ? `<ul class="warning-list">${warnings.map(w => `<li>${escapeHtml(w)}</li>`).join("")}</ul>` :
      `<ul class="success-list"><li>No capture-quality warnings were detected.</li></ul>`}
    <div class="form-grid">
      ${isMovement ? `
        <label class="field">
          <span class="field-label">What direction did the car travel?</span>
          <select id="trial-direction" class="select">
            <option value="forward">Forward</option>
            <option value="reverse">Reverse</option>
          </select>
        </label>
        <label class="field">
          <span class="field-label">Tape-measured distance (m)</span>
          <input id="trial-distance" class="input" type="number" min="0.01" step="0.001" value="${defaultDistance}">
        </label>` : ""}
      ${isCircle ? `
        <label class="field">
          <span class="field-label">Rear-axle path diameter (m)</span>
          <input id="trial-diameter" class="input" type="number" min="0.10" step="0.001" placeholder="e.g. 1.65">
        </label>` : ""}
      <label class="field field-full">
        <span class="field-label">Notes (optional)</span>
        <textarea id="trial-notes" class="textarea" placeholder="Wheel slip, steering variation, marker overshoot…"></textarea>
      </label>
    </div>
    <label class="confirm-line">
      <input id="trial-confirmed" type="checkbox">
      <span>I confirm the direction and tape measurement above describe this exact capture. I reviewed the warnings.</span>
    </label>
    <div class="button-row">
      <button class="button" type="button" onclick="acceptPending()">Accept trial</button>
      <button class="button button-ghost" type="button" onclick="discardPending()">Discard and repeat</button>
    </div>
  </section>`;
}

function acceptedTrialsTable(session, kinds) {
  const trials = (session.trials || []).filter(t => t.accepted && kinds.includes(t.kind));
  if (!trials.length) return `<p class="small muted">No accepted trials yet.</p>`;
  return `<table class="trial-table">
    <thead><tr><th>#</th><th>Type</th><th>Tape measure</th><th>Odom</th><th>Samples</th><th></th></tr></thead>
    <tbody>${trials.map((trial, index) => {
      const tape = trial.kind === "movement"
        ? `${escapeHtml(trial.direction)} ${fmt(trial.measured_distance_m, 3)} m`
        : trial.measured_diameter_m
          ? `${fmt(trial.measured_diameter_m, 3)} m diameter`
          : "centred";
      return `<tr>
        <td>${index + 1}</td>
        <td class="trial-kind">${escapeHtml(trial.kind.replaceAll("_", " "))}</td>
        <td>${tape}</td>
        <td>${signed(trial.summary?.odom_distance_m, 3)} m</td>
        <td>${trial.summary?.odom_integration?.sample_count || 0}</td>
        <td><button class="button button-ghost button-small" type="button"
          onclick="deleteTrial('${escapeHtml(trial.id)}')">Remove</button></td>
      </tr>`;
    }).join("")}</tbody>
  </table>`;
}

function renderStationary(session) {
  if (session.active_capture) return captureActiveCard(session.active_capture);
  if (session.pending_capture) return pendingCard(session.pending_capture);
  const counts = trialCounts(session);
  return `<section class="hero-card">
    <div class="eyebrow">Step 2 · Stationary baseline</div>
    <h1>Record what “not moving” looks like.</h1>
    <p class="lead">This estimates the ERPM offset and catches noise or sign problems before distance trials.</p>
    <ul class="instruction-list">
      <li>Place the car on level ground. All four wheels must be completely still.</li>
      <li>Release LB, leave throttle neutral, and visually centre the steering.</li>
      <li>Start recording, do not touch the car for at least 5 seconds, then stop.</li>
    </ul>
    <div class="progress-strip">
      <div class="progress-item ${counts.stationary ? "done" : ""}">Stationary sample ${counts.stationary ? "✓" : "needed"}</div>
    </div>
    <div class="button-row">
      <button class="button" type="button" onclick="startCapture('stationary')">Start stationary recording</button>
      ${counts.stationary ? `<button class="button button-secondary" type="button" onclick="setStage('movement')">Continue to movement</button>` : ""}
    </div>
  </section>
  <section class="card">
    <h2>Accepted stationary captures</h2>
    ${acceptedTrialsTable(session, ["stationary"])}
  </section>`;
}

function renderMovement(session) {
  if (session.active_capture) return captureActiveCard(session.active_capture);
  if (session.pending_capture) return pendingCard(session.pending_capture, 5);
  const counts = trialCounts(session);
  const nextStage = session.mode === "movement_steering" ? "steering" : "report";
  const nextLabel = session.mode === "movement_steering" ? "Continue to steering" : "Build report";
  return `<section class="hero-card">
    <div class="eyebrow">Step 3 · Known distance</div>
    <h1>Drive a line you can prove with a tape measure.</h1>
    <p class="lead">Three trials are recommended: two forward and one reverse. The wizard keeps every sign—negative readings are diagnosed, never silently flipped.</p>
    <ul class="instruction-list">
      <li>Mark a straight 5–10 m lane. Measure from the centre of the rear axle at the start to the same point at the finish.</li>
      <li>Align straight and click record before moving. Hold LB and drive smoothly; avoid wheelspin and sharp acceleration.</li>
      <li>Stop exactly at the mark, release LB, then stop recording. Confirm direction and enter the tape distance.</li>
    </ul>
    <div class="progress-strip">
      <div class="progress-item ${counts.movement >= 1 ? "done" : ""}">Trial 1 ${counts.movement >= 1 ? "✓" : ""}</div>
      <div class="progress-item ${counts.movement >= 2 ? "done" : ""}">Trial 2 ${counts.movement >= 2 ? "✓" : ""}</div>
      <div class="progress-item ${counts.movement >= 3 ? "done" : ""}">Trial 3 ${counts.movement >= 3 ? "✓" : "recommended"}</div>
    </div>
    <div class="button-row">
      <button class="button" type="button" onclick="startCapture('movement')">Start movement recording</button>
      ${counts.movement >= 1 ? `<button class="button button-secondary" type="button" onclick="setStage('${nextStage}')">${nextLabel}</button>` : ""}
      ${counts.movement === 1 ? `<span class="small muted">One trial produces a low-confidence suggestion.</span>` : ""}
    </div>
  </section>
  <section class="card">
    <h2>Accepted movement trials</h2>
    ${acceptedTrialsTable(session, ["movement"])}
  </section>`;
}

function renderSteering(session) {
  if (session.active_capture) return captureActiveCard(session.active_capture);
  if (session.pending_capture) return pendingCard(session.pending_capture);
  const counts = trialCounts(session);
  const ready = counts.steering_left >= 1 && counts.steering_right >= 1;
  return `<section class="hero-card">
    <div class="eyebrow">Step 4 · Steering geometry</div>
    <h1>Measure the circle at the rear axle.</h1>
    <p class="lead">The wizard fits servo value against real steering angle. Wheelbase stays fixed at 0.324 m; tape measurements calibrate gain and centre, not vehicle geometry.</p>
    <ul class="instruction-list">
      <li><strong>Centre:</strong> visually align both front wheels straight, keep the car still, and record 3–5 seconds.</li>
      <li><strong>Circle:</strong> mark the rear-axle centre, hold steering steady, and drive one slow complete circle back to the starting heading.</li>
      <li>Measure the diameter traced by the centre of the rear axle—not the outer body or tire edge. Repeat once left and once right.</li>
    </ul>
    <div class="progress-strip">
      <div class="progress-item ${counts.steering_center ? "done" : ""}">Centre ${counts.steering_center ? "✓" : "recommended"}</div>
      <div class="progress-item ${counts.steering_left ? "done" : ""}">Left circle ${counts.steering_left ? "✓" : "needed"}</div>
      <div class="progress-item ${counts.steering_right ? "done" : ""}">Right circle ${counts.steering_right ? "✓" : "needed"}</div>
    </div>
    <div class="button-row">
      <select id="steering-kind" class="select" style="width:auto">
        <option value="steering_center">Centred wheels</option>
        <option value="steering_left">Left circle</option>
        <option value="steering_right">Right circle</option>
      </select>
      <button class="button" type="button" onclick="startSelectedSteering()">Start recording</button>
      ${ready ? `<button class="button button-secondary" type="button" onclick="setStage('report')">Build report</button>` : ""}
    </div>
  </section>
  <section class="card">
    <h2>Accepted steering trials</h2>
    ${acceptedTrialsTable(session, ["steering_center", "steering_left", "steering_right"])}
  </section>`;
}

function suggestionRow(name, current, suggested) {
  return `<div class="suggestion">
    <span class="suggestion-name">${escapeHtml(name)}</span>
    <span class="suggestion-value">${fmt(current, 6)}</span>
    <span class="suggestion-arrow">→</span>
    <span class="suggestion-value">${finite(suggested) ? fmt(suggested, 6) : "insufficient data"}</span>
  </div>`;
}

function warningList(warnings) {
  if (!warnings?.length) return `<ul class="success-list"><li>No report-level warnings.</li></ul>`;
  return `<ul class="warning-list">${warnings.map(w => `<li>${escapeHtml(w)}</li>`).join("")}</ul>`;
}

function renderReport(session) {
  const report = session.report;
  if (!report) {
    return `<section class="hero-card">
      <div class="eyebrow">Final step · Report</div>
      <h1>Turn measurements into suggestions.</h1>
      <p class="lead">The report keeps confidence, outliers, negative-sign failures, and all warnings beside the proposed YAML values. Nothing is applied automatically.</p>
      <div class="button-row">
        <button class="button" type="button" onclick="generateReport()">Generate calibration report</button>
        <button class="button button-ghost" type="button" onclick="setStage('movement')">Add another trial</button>
      </div>
    </section>`;
  }
  const current = report.current_parameters || {};
  const suggestions = report.parameter_suggestions || {};
  return `<section class="hero-card">
    <div class="eyebrow">Calibration report</div>
    <div class="report-status ${escapeHtml(report.overall_status)}">${escapeHtml(report.overall_status)}</div>
    <h1 style="margin-top:14px">Review before changing the car.</h1>
    <p class="lead">${escapeHtml(report.safety_note)}</p>
    <div class="suggestions">
      ${suggestionRow("speed_to_erpm_gain", current.speed_to_erpm_gain, suggestions.speed_to_erpm_gain)}
      ${suggestionRow("speed_to_erpm_offset", current.speed_to_erpm_offset, suggestions.speed_to_erpm_offset)}
      ${report.steering ? suggestionRow("steering_angle_to_servo_gain", current.steering_angle_to_servo_gain, suggestions.steering_angle_to_servo_gain) : ""}
      ${report.steering ? suggestionRow("steering_angle_to_servo_offset", current.steering_angle_to_servo_offset, suggestions.steering_angle_to_servo_offset) : ""}
    </div>
    <div class="button-row">
      <a class="button" href="/api/report/md">Download Markdown</a>
      <a class="button button-secondary" href="/api/report/json">Download JSON</a>
      <button class="button button-ghost" type="button" onclick="generateReport()">Recalculate</button>
    </div>
  </section>
  <section class="card">
    <h2>Movement <span class="report-status ${escapeHtml(report.movement.status)}">${escapeHtml(report.movement.status)}</span></h2>
    <p>${report.movement.usable_trial_count} usable of ${report.movement.accepted_trial_count} accepted movement trials.</p>
    ${warningList(report.movement.warnings)}
  </section>
  ${report.steering ? `<section class="card">
    <h2>Steering <span class="report-status ${escapeHtml(report.steering.status)}">${escapeHtml(report.steering.status)}</span></h2>
    <p>${report.steering.usable_point_count} usable geometry points · fit residual ${fmt(report.steering.fit_rmse_servo, 4)} servo units.</p>
    ${warningList(report.steering.warnings)}
  </section>` : ""}
  <section class="card">
    <h2>Suggested YAML</h2>
    <pre class="code-block">${Object.entries(suggestions).map(([name, value]) =>
      finite(value) ? `${escapeHtml(name)}: ${value.toFixed(6)}` : `# ${escapeHtml(name)}: insufficient data`
    ).join("\n")}</pre>
  </section>`;
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
  view.dataset.rendered = "true";
  if (!session) {
    view.innerHTML = renderSetup();
    return;
  }
  if (session.stage === "preflight") view.innerHTML = renderPreflight(session);
  else if (session.stage === "stationary") view.innerHTML = renderStationary(session);
  else if (session.stage === "movement") view.innerHTML = renderMovement(session);
  else if (session.stage === "steering") view.innerHTML = renderSteering(session);
  else if (session.stage === "report") view.innerHTML = renderReport(session);
  else view.innerHTML = renderPreflight(session);
}

window.selectMode = function selectMode(mode) {
  app.selectedMode = mode;
  render();
};

window.createSession = async function createSession() {
  try {
    await api("new_session", {
      mode: app.selectedMode,
      current_parameters: currentParameterValues(),
    });
    toast("Calibration session created.");
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

window.startSelectedSteering = function startSelectedSteering() {
  const kind = document.getElementById("steering-kind")?.value;
  if (kind) window.startCapture(kind);
};

window.stopCapture = async function stopCapture() {
  try {
    await api("stop_capture");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.acceptPending = async function acceptPending() {
  const confirmed = document.getElementById("trial-confirmed")?.checked === true;
  const extra = {
    confirmed,
    notes: document.getElementById("trial-notes")?.value || "",
  };
  const direction = document.getElementById("trial-direction");
  const distance = document.getElementById("trial-distance");
  const diameter = document.getElementById("trial-diameter");
  if (direction) extra.direction = direction.value;
  if (distance) extra.measured_distance_m = Number(distance.value);
  if (diameter) extra.measured_diameter_m = Number(diameter.value);
  try {
    await api("accept_capture", extra);
    toast("Trial accepted and saved on the car.");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.discardPending = async function discardPending() {
  if (!confirm("Discard this capture? It cannot be recovered.")) return;
  try {
    await api("discard_capture");
    toast("Capture discarded.");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.generateReport = async function generateReport() {
  try {
    await api("generate_report");
    toast("Report generated and archived on the car.");
  } catch (error) {
    toast(error.message, "error");
  }
};

window.deleteTrial = async function deleteTrial(trialId) {
  if (!confirm("Remove this accepted trial from the calibration? The archived event log will record the removal.")) return;
  try {
    await api("delete_trial", {trial_id: trialId});
    toast("Trial removed.");
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
    toast("New session started.");
  } catch (error) {
    toast(error.message, "error");
  }
});

connectSocket();
fetchState();
setInterval(() => {
  if (!app.connected) fetchState();
}, 2500);
