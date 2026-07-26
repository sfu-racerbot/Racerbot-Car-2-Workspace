(() => {
  'use strict';

  const stage = document.getElementById('camera-stage');
  const cameraFeed = document.getElementById('recording-camera-feed');
  const cameraStatus = document.getElementById('camera-status');
  const currentTime = document.getElementById('current-time');
  const currentDate = document.getElementById('current-date');
  const speedEl = document.getElementById('recording-speed');
  const steeringEl = document.getElementById('recording-steering');
  const stopwatchEl = document.getElementById('recording-stopwatch');
  const timerStateEl = document.getElementById('recording-timer-state');
  const lbEl = document.getElementById('recording-lb');
  const commandEl = document.getElementById('recording-command');
  const cpuEl = document.getElementById('recording-cpu');
  const wifiEl = document.getElementById('recording-wifi');
  const telemetryStatus = document.getElementById('telemetry-status');

  const state = { speed: null, drive: null, stats: null, stopwatch: null };
  let ws = null;
  let cameraConnected = false;
  const CAMERA_PORT = 9090;

  function connectTelemetry() {
    const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
    ws = new WebSocket(`${scheme}://${location.host}/ws`);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => {
      telemetryStatus.textContent = 'TELEMETRY LIVE';
      telemetryStatus.className = 'status-pill status-good';
    };
    ws.onclose = () => {
      telemetryStatus.textContent = 'TELEMETRY RETRYING';
      telemetryStatus.className = 'status-pill status-bad';
      setTimeout(connectTelemetry, 1000);
    };
    ws.onerror = () => ws.close();
    ws.onmessage = (event) => {
      if (typeof event.data !== 'string') return;
      const msg = JSON.parse(event.data);
      const receivedAt = performance.now();
      if (msg.type === 'speed') {
        state.speed = { value: msg.speed, receivedAt };
      } else if (msg.type === 'drive') {
        state.drive = { speed: msg.speed, steeringAngle: msg.steering_angle, receivedAt };
      } else if (msg.type === 'stats') {
        state.stats = { cpuPercent: msg.cpu_percent, wifiDbm: msg.wifi_dbm, receivedAt };
      } else if (msg.type === 'stopwatch') {
        state.stopwatch = {
          elapsedS: msg.elapsed_s,
          enabled: msg.enabled,
          running: msg.running,
          lbHeld: msg.lb_held,
          joyFresh: msg.joy_fresh,
          buttonAvailable: msg.button_available,
          receivedAt,
        };
      }
      renderTelemetry();
    };
  }

  function isFresh(entry, limitMs) {
    return entry && performance.now() - entry.receivedAt <= limitMs;
  }

  function stopwatchElapsed() {
    if (!state.stopwatch) return 0;
    const extra = state.stopwatch.running
      ? Math.min((performance.now() - state.stopwatch.receivedAt) / 1000, 0.5)
      : 0;
    return state.stopwatch.elapsedS + extra;
  }

  function formatStopwatch(totalSeconds) {
    const centiseconds = Math.floor(Math.max(0, totalSeconds) * 100);
    const hours = Math.floor(centiseconds / 360000);
    const minutes = Math.floor((centiseconds % 360000) / 6000);
    const seconds = Math.floor((centiseconds % 6000) / 100);
    const fraction = centiseconds % 100;
    const body = `${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}.${String(fraction).padStart(2, '0')}`;
    return hours > 0 ? `${hours}:${body}` : body;
  }

  function setLbStatus(text, tier) {
    lbEl.textContent = text;
    lbEl.className = `status-pill status-${tier}`;
  }

  function renderTelemetry() {
    speedEl.textContent = isFresh(state.speed, 1000) ? state.speed.value.toFixed(2) : '--';
    steeringEl.textContent = isFresh(state.drive, 1000)
      ? (state.drive.steeringAngle * 180 / Math.PI).toFixed(1)
      : '--';
    commandEl.textContent = isFresh(state.drive, 1000)
      ? `${state.drive.speed.toFixed(2)} m/s`
      : '-- m/s';
    cpuEl.textContent = isFresh(state.stats, 3000) ? `${state.stats.cpuPercent.toFixed(0)}%` : '--%';
    wifiEl.textContent = isFresh(state.stats, 3000) && state.stats.wifiDbm != null
      ? `${state.stats.wifiDbm.toFixed(0)} dBm`
      : '-- dBm';

    const timer = state.stopwatch;
    stopwatchEl.textContent = formatStopwatch(stopwatchElapsed());
    stopwatchEl.classList.toggle('running', !!(timer && timer.running));
    if (!timer || !isFresh(timer, 1000) || !timer.joyFresh) {
      setLbStatus('LB: NO JOYSTICK', 'bad');
      timerStateEl.textContent = timer && timer.enabled ? 'PAUSED · JOYSTICK STALE' : 'TIMER OFF';
    } else if (!timer.buttonAvailable) {
      setLbStatus('LB: INPUT MISSING', 'bad');
      timerStateEl.textContent = 'PAUSED · LB UNAVAILABLE';
    } else if (timer.lbHeld) {
      setLbStatus('LB: HELD', 'good');
      timerStateEl.textContent = timer.enabled ? 'RUNNING' : 'TIMER OFF';
    } else {
      setLbStatus('LB: RELEASED', timer.enabled ? 'warn' : 'bad');
      timerStateEl.textContent = timer.enabled ? 'PAUSED · HOLD LB TO RUN' : 'TIMER OFF';
    }
  }

  function updateClock() {
    const now = new Date();
    currentTime.textContent = now.toLocaleTimeString([], {
      hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false,
    });
    currentDate.textContent = now.toLocaleDateString([], {
      weekday: 'short', year: 'numeric', month: 'short', day: '2-digit',
    });
  }

  function tryCameraConnect() {
    cameraFeed.src = `http://${location.hostname}:${CAMERA_PORT}/stream?_=${Date.now()}`;
  }

  cameraFeed.addEventListener('load', () => {
    cameraConnected = true;
    stage.classList.add('has-feed');
    cameraStatus.textContent = 'CAMERA LIVE';
  });
  cameraFeed.addEventListener('error', () => {
    cameraConnected = false;
    stage.classList.remove('has-feed');
    cameraStatus.textContent = 'CAMERA RETRYING';
  });

  tryCameraConnect();
  connectTelemetry();
  updateClock();
  renderTelemetry();
  setInterval(() => {
    updateClock();
    renderTelemetry();
  }, 50);
  setInterval(() => { if (!cameraConnected) tryCameraConnect(); }, 3000);
})();
