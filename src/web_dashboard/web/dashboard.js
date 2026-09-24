// dashboard.js
//
// Browser side of the live dashboard: connects to this same server's
// WebSocket endpoint, receives map/scan/pose updates (see
// web_dashboard/protocol.py for the exact wire format), and draws all of
// it on an HTML5 canvas -- the map as a background image, the LIDAR scan
// as points, and the car as an arrow, all in one consistent world (map)
// frame so their relative positions are directly comparable.
//
// No build step, no framework, no external dependencies -- plain ES2017
// in one file, deliberately, so it's easy to read start to finish.

(() => {
  'use strict';

  // ---------------------------------------------------------------------
  // DOM handles
  // ---------------------------------------------------------------------
  const canvas = document.getElementById('view');
  const ctx = canvas.getContext('2d');
  const connDot = document.getElementById('conn-dot');
  const connText = document.getElementById('conn-text');
  // The phone layout's always-on status strip. It is display:none on every
  // other layout, so on a laptop these four writes go to elements nobody
  // can see -- which is the point: there is no "phone mode" branch to get
  // wrong, and the strip can never show a different number from the panel
  // it summarises because it is filled from the same values at the same
  // instant (see updateDigests).
  const stripDot = document.getElementById('strip-dot');
  const stripState = document.getElementById('strip-state');
  const stripSpeed = document.getElementById('strip-speed');
  const stripFeeds = document.getElementById('strip-feeds');
  const infoMap = document.getElementById('info-map');
  const infoScan = document.getElementById('info-scan');
  const infoPose = document.getElementById('info-pose');
  const infoDrive = document.getElementById('info-drive');
  const vehicleSpeed = document.getElementById('vehicle-speed');
  const vehicleSteering = document.getElementById('vehicle-steering');
  const vehicleLb = document.getElementById('vehicle-lb');
  const stopwatchDisplay = document.getElementById('stopwatch-display');
  const stopwatchState = document.getElementById('stopwatch-state');
  const stopwatchToggle = document.getElementById('stopwatch-toggle');
  const stopwatchReset = document.getElementById('stopwatch-reset');
  const infoCpu = document.getElementById('info-cpu');
  const infoMem = document.getElementById('info-mem');
  const infoTemp = document.getElementById('info-temp');
  // The ring around each of those three. They are a second encoding of the
  // number, never a replacement for it -- see setGauge below.
  const arcCpu = document.getElementById('arc-cpu');
  const arcMem = document.getElementById('arc-mem');
  const arcTemp = document.getElementById('arc-temp');
  const infoWifiText = document.getElementById('info-wifi-text');
  const infoUptime = document.getElementById('info-uptime');
  const wifiBarEls = document.querySelectorAll('#wifi-bars .wifi-bar');
  const dots = {
    map: document.getElementById('dot-map'),
    scan: document.getElementById('dot-scan'),
    pose: document.getElementById('dot-pose'),
    drive: document.getElementById('dot-drive'),
    stats: document.getElementById('dot-stats'),
    measure: document.getElementById('dot-measure'),
    maps: document.getElementById('dot-maps'),
  };
  const intentSection = document.getElementById('intent-section');
  const intentDot = document.getElementById('dot-intent');
  const intentState = document.getElementById('intent-state');
  const intentNode = document.getElementById('intent-node');
  const intentReason = document.getElementById('intent-reason');
  const intentSpeeds = document.getElementById('intent-speeds');
  const intentSteering = document.getElementById('intent-steering');
  const intentFactors = document.getElementById('intent-factors');
  const intentLog = document.getElementById('intent-log');
  const intentToggle = document.getElementById('intent-toggle');
  const racelineStatus = document.getElementById('raceline-status');
  const racelineToggle = document.getElementById('raceline-toggle');
  const commandedToggle = document.getElementById('commanded-toggle');

  const modeBanner = document.getElementById('mode-banner');
  const resetViewBtn = document.getElementById('reset-view');

  // Measuring
  const measureEnable = document.getElementById('measure-enable');
  const measureList = document.getElementById('measure-list');
  const measureStatus = document.getElementById('measure-status');
  const measurePanel = document.getElementById('measure-panel');
  const measureTotal = document.getElementById('measure-total');
  const measureHint = document.getElementById('measure-hint');
  const measureUndoBtn = document.getElementById('measure-undo');
  const measureClearBtn = document.getElementById('measure-clear');
  const measureNote = document.getElementById('measure-note');

  // Saved maps
  const mapsSection = document.getElementById('maps-section');
  const mapList = document.getElementById('map-list');
  const mapStatus = document.getElementById('map-status');
  const mapResetBlock = document.getElementById('map-reset-block');
  const mapDeleteBlock = document.getElementById('map-delete-block');
  const mapClearViewBtn = document.getElementById('map-clear-view');
  const mapResetSlamBtn = document.getElementById('map-reset-slam');

  if (racelineToggle) {
    racelineToggle.addEventListener('change', () => {
      state.showRacingLine = racelineToggle.checked;
    });
  }

  if (intentToggle) {
    intentToggle.addEventListener('change', () => {
      state.showIntent = intentToggle.checked;
      scheduleRender();
    });
  }

  if (commandedToggle) {
    commandedToggle.addEventListener('change', () => {
      state.showCommanded = commandedToggle.checked;
      scheduleRender();
    });
  }

  const overlay = document.getElementById('overlay');

  const tuningSection = document.getElementById('tuning-section');
  const tuningSummary = document.getElementById('tuning-summary');
  const tuningDot = document.getElementById('dot-tuning');
  const tuningOpen = document.getElementById('tuning-open');
  const tuningPanel = document.getElementById('tuning-panel');
  const tuningClose = document.getElementById('tuning-close');
  const tuningArm = document.getElementById('tuning-arm');
  const tuningArmNote = document.getElementById('tuning-arm-note');
  const tuningBody = document.getElementById('tuning-body');
  const tuningSave = document.getElementById('tuning-save');
  const tuningSaveStatus = document.getElementById('tuning-save-status');

  const minimapPanel = document.getElementById('minimap-panel');
  const minimapCanvas = document.getElementById('minimap');
  const minimapCtx = minimapCanvas.getContext('2d');
  const cameraPanel = document.getElementById('camera-panel');
  const cameraFeed = document.getElementById('camera-feed');
  const cameraResize = document.getElementById('camera-resize');
  const scaleRuler = document.getElementById('scale-ruler');
  const scaleLabel = document.getElementById('scale-label');

  // ---------------------------------------------------------------------
  // State. Each of map/scan/pose carries `receivedAt`, stamped with this
  // browser's own clock (performance.now()) on arrival, so staleness can
  // be judged locally without needing the server and browser clocks to
  // be in sync.
  // ---------------------------------------------------------------------
  const state = {
    map: null,   // { width, height, resolution, originX, originY, canvas: <offscreen canvas>, receivedAt }
    scan: null,  // { angleMin, angleIncrement, rangeMin, rangeMax, laserOffsetX, laserOffsetY, ranges: Float32Array, receivedAt }
    pose: null,  // { x, y, yaw, receivedAt }
    drive: null, // { speed, steeringAngle, receivedAt } -- selected /ackermann_cmd
    speed: null, // { speed, receivedAt } -- measured /odom longitudinal speed
    stopwatch: null, // { elapsedS, enabled, running, lbHeld, joyFresh, buttonAvailable, receivedAt }
    stats: null, // { cpuPercent, memPercent, cpuTempC, uptimeS, wifiDbm, receivedAt }
    tuning: null, // { enabled, allowSave, nodes: [...] } -- see protocol.tuning_state_message
    tuningArmed: false, // server-confirmed, never assumed from the checkbox
    // { enabled, targets: [...] } -- see protocol.process_state_message.
    // null until the first snapshot arrives, which is why the panel starts
    // out saying "looking for..." rather than "none running".
    processes: null,
    savedMaps: null,
    // What the driving node says it is *trying* to do -- see
    // docs/drive-intent.md. `intent` is the raw schema payload; `reason`
    // is held separately because the car only re-sends the (sometimes
    // expensive) explanation on state changes and on a slow period, so
    // most messages deliberately arrive without one.
    intent: null,
    // The racing line pure_pursuit is following, latched from
    // /racing_line. Null until a controller actually loads a profile,
    // which is exactly what makes its presence meaningful.
    racingLine: null,
    showRacingLine: true,
    intentReason: '',
    intentReceivedAt: 0,
    intentLog: [],   // [{ state, severity, at, heldMs }] newest first
    showIntent: true,
    // The commanded-path ghost and the lookahead reticle. Off unless
    // asked for -- see drawIntent. Seeded from the checkbox rather than a
    // literal false, because a browser restoring a ticked box across a
    // reload would otherwise leave the box ticked and the paths hidden.
    showCommanded: !!(commandedToggle && commandedToggle.checked),
    // Binary frames whose length did not match the header that claimed
    // them -- see handleBinary. Surfaced rather than swallowed, because
    // the visible symptom is a map that has turned to garbage and there
    // is otherwise nothing to tell you why.
    desyncCount: 0,
    desyncAt: 0,
    desyncDetail: '',
  };

  // How the arrow is drawn. Half-width is proportional to the *planned*
  // speed at each sample, which is what makes "wider = faster" readable at
  // a glance; the clamps keep a crawl visible and stop a 6m/s straight
  // from covering the track.
  const INTENT_HALF_WIDTH_PER_MPS = 0.07; // meters of half-width per m/s
  const INTENT_MIN_HALF_WIDTH = 0.04;     // meters
  const INTENT_MAX_HALF_WIDTH = 0.35;     // meters
  // Older than this and the arrow is a claim about a moment that has
  // passed; it fades rather than disappearing, so "the publisher died"
  // and "the car is stopped" stay visually distinct.
  const INTENT_STALE_MS = 1000;
  const INTENT_LOG_LIMIT = 20;

  // ---------------------------------------------------------------------
  // The car itself, in meters.
  //
  // `base_link` -- the origin of every pose, of /drive_intent, and of the
  // body-frame view -- is the REAR AXLE. Every x below is therefore
  // "meters ahead of the rear axle", and the icon is anchored at x = 0,
  // not at the middle of the car. (It used to be anchored at neither: the
  // old icon was a pile of multiples of a `size` that tracked the zoom
  // level, so the drawn car was never any particular size and its origin
  // sat 42% of the way down a body of no defined length.)
  //
  // Measured on this car with a tape, 2026-08-24:
  //
  //   wheelbaseM   0.36  rear axle centre to front axle centre
  //   trackWidthM  0.30  outer edge of tire to outer edge of the other
  //                      tire -- the widest part of the car
  //   lidarXM      0.26  = wheelbase - 0.10. The Hokuyo sits 0.10 m behind
  //                      the FRONT AXLE. The old 0.33 m figure came from
  //                      assuming 0.10 m behind the physical NOSE, which
  //                      is a different reference point and 0.07 m out.
  //
  // overhangM is the one number nobody has measured on this car: it is
  // half of (Traxxas 74276-4 published length 0.535 m - published
  // wheelbase 0.324 m). It only sets how far the drawn outline extends
  // past the axles. No clearance, TTC or stopping distance anywhere in
  // this workspace is computed from anything in this object -- those come
  // from gap_follow.yaml / pure_pursuit.yaml, which carry a deliberately
  // inflated envelope. This is a picture, and it says so.
  //
  // Tire size is likewise approximate (a 1/10-scale rally tire), and only
  // ever affects how chunky the four wheels look and how far a steered
  // front tire swings out of the footprint.
  // ---------------------------------------------------------------------
  const CAR_MODEL = {
    wheelbaseM: 0.36,      // measured
    trackWidthM: 0.30,     // measured, over the tires
    lidarXM: 0.26,         // measured: 0.10 m behind the front axle
    overhangM: 0.1055,     // derived, drawing only -- see above
    tireDiameterM: 0.10,   // approximate, drawing only
    tireWidthM: 0.04,      // approximate, drawing only
    lidarRadiusM: 0.025,   // approximate: a UST-10LX is ~50 mm across
  };

  // Below this the icon stops being to scale and is drawn at a fixed pixel
  // length instead, because a 0.57 m car at the zoom that fits a whole
  // 30 m map on screen is under two pixels long and simply vanishes.
  const CAR_MIN_LENGTH_PX = 24;

  // Steering beyond a right angle is not a steering angle, and tan() blows
  // up at exactly pi/2. Drawing-only clamp; the commanded number itself is
  // reported unclamped in the vehicle panel.
  const CAR_MAX_DRAWN_STEER = Math.PI / 2 - 1e-3;

  // ---------------------------------------------------------------------
  // Ackermann steering geometry, from the two measured numbers.
  //
  // On a real steering rack the inside wheel of a turn traces a tighter
  // circle than the outside one, so it is turned further. Drawing both
  // front wheels at the same angle would understate exactly the thing
  // worth showing: how far the inside tire swings outside the car's
  // straight-ahead footprint, which is the clearance a corner actually
  // has to give it.
  //
  //   R           = wheelbase / tan(delta)   turn radius at the rear axle,
  //                                          positive = centre to the left
  //   delta_inner = atan(wheelbase / (|R| - track/2))
  //   delta_outer = atan(wheelbase / (|R| + track/2))
  //
  // which satisfies the Ackermann condition cot(outer) - cot(inner) =
  // track / wheelbase exactly -- that identity is what the test checks
  // against, rather than any number recorded from this function.
  // ---------------------------------------------------------------------
  function ackermannWheelAngles(steering,
                                wheelbase = CAR_MODEL.wheelbaseM,
                                track = CAR_MODEL.trackWidthM) {
    if (!Number.isFinite(steering) || steering === 0) return { left: 0, right: 0 };
    const delta = Math.max(-CAR_MAX_DRAWN_STEER, Math.min(CAR_MAX_DRAWN_STEER, steering));
    const halfTrack = track / 2;
    const radius = Math.abs(wheelbase / Math.tan(delta));
    // A turn radius inside the car's own half-track has no Ackermann
    // solution -- the inside wheel would have to pivot past a right angle.
    // Unreachable on this car (the rack limit is 0.26 rad, a 1.35 m
    // radius), but a bad /drive command must not produce a NaN or a wheel
    // drawn pointing backwards.
    if (!(radius > halfTrack)) return { left: delta, right: delta };
    const sign = delta > 0 ? 1 : -1;
    const inner = sign * Math.atan(wheelbase / (radius - halfTrack));
    const outer = sign * Math.atan(wheelbase / (radius + halfTrack));
    // Turning left (delta > 0) puts the turn centre to the left, so the
    // LEFT wheel is the inside one.
    return delta > 0 ? { left: inner, right: outer } : { left: outer, right: inner };
  }

  // The car's footprint at a given steering angle, in body coordinates
  // (x forward from the rear axle, y to the LEFT, both meters). Pure --
  // no canvas, no state -- so it can be checked against the closed forms
  // above rather than against a screenshot.
  function carModelGeometry(steering = 0) {
    const {
      wheelbaseM, trackWidthM, lidarXM, overhangM, tireDiameterM, tireWidthM,
    } = CAR_MODEL;
    const halfTrack = trackWidthM / 2;
    const angles = ackermannWheelAngles(steering);
    // Wheel centres sit half a tire inboard of the measured track, so that
    // each tire's OUTER edge lands exactly on +/- trackWidth/2.
    const wheelY = halfTrack - tireWidthM / 2;
    const wheel = (x, y, angle) => ({
      x, y, angle, length: tireDiameterM, width: tireWidthM,
    });
    const wheels = [
      wheel(wheelbaseM, wheelY, angles.left),
      wheel(wheelbaseM, -wheelY, angles.right),
      wheel(0, wheelY, 0),
      wheel(0, -wheelY, 0),
    ];
    // How far the widest part of the car reaches from the centreline once
    // the front wheels are turned. A rectangle of length L and width W
    // rotated by d reaches |y| = |yc| + (L/2)|sin d| + (W/2)|cos d|.
    // Kept per side, because a left turn swings the LEFT tire out further
    // than the right one and reporting the worse of the two on both sides
    // would claim clearance the car does not actually need.
    const reach = (w) => Math.abs(w.y)
      + (w.length / 2) * Math.abs(Math.sin(w.angle))
      + (w.width / 2) * Math.abs(Math.cos(w.angle));
    const worstOn = (side) => wheels
      .filter((w) => Math.sign(w.y) === side)
      .reduce((worst, w) => Math.max(worst, reach(w)), 0);
    const clearanceLeft = worstOn(1);
    const clearanceRight = worstOn(-1);
    return {
      rearAxleX: 0,
      frontAxleX: wheelbaseM,
      lidarX: lidarXM,
      tailX: -overhangM,
      noseX: wheelbaseM + overhangM,
      halfTrack,
      wheels,
      clearanceLeft,
      clearanceRight,
      steeringHalfWidth: Math.max(clearanceLeft, clearanceRight),
    };
  }

  if (typeof window !== 'undefined') {
    window.__CAR_MODEL = CAR_MODEL;
    window.__ackermannWheelAngles = ackermannWheelAngles;
    window.__carModelGeometry = carModelGeometry;
    // drawCarIcon itself, so a test can run the drawing path and check that
    // no NaN reaches the canvas. A NaN coordinate does not throw -- canvas
    // silently draws nothing -- so "the car vanished" is a failure mode
    // that only an assertion on the arguments can catch.
    window.__drawCarIcon = (...args) => drawCarIcon(...args);
  }

  // ---------------------------------------------------------------------
  // The HUD palette, in one place.
  //
  // These are the canvas half of the theme and they are deliberately the
  // same values as the CSS custom properties at the top of style.css --
  // --void, --accent, --go, --warn, --bad, --ink. Canvas cannot read CSS
  // variables, so the two lists have to be kept in step by hand; if you
  // change a colour there, change it here.
  //
  // The rule the palette encodes: cyan is "the system" (the car itself,
  // UI marks drawn over the map), and green/amber/red are reserved for
  // what the car has *decided*. That is why the car icon is no longer red
  // -- a permanently red car competes with "stop" meaning stop.
  // ---------------------------------------------------------------------
  const HUD = {
    void: '#04070c',      // the field everything is drawn on
    accent: '#3ddcff',    // cyan: the car, view marks, scale
    go: '#2bf58a',
    warn: '#ffb636',
    bad: '#ff4560',
    ink: '#dcecf7',
  };

  const INTENT_COLORS = {
    drive:   { fill: 'rgba(43, 245, 138, 0.26)', edge: 'rgba(43, 245, 138, 0.92)', ink: HUD.go },
    caution: { fill: 'rgba(255, 182, 54, 0.28)', edge: 'rgba(255, 182, 54, 0.94)', ink: HUD.warn },
    stop:    { fill: 'rgba(255, 69, 96, 0.28)',  edge: 'rgba(255, 69, 96, 0.94)',  ink: HUD.bad },
  };

  function intentColors(severity) {
    return INTENT_COLORS[severity] || INTENT_COLORS.caution;
  }

  // Pending "what does the next binary frame mean" -- set when a JSON
  // header arrives, consumed the moment the binary payload right after
  // it does (the server always sends them as an immediate pair).
  //
  // One slot, so a header that never gets its binary points this at the
  // wrong thing and every later payload is decoded as the wrong type.
  // The header's `bytes` field is what makes that detectable; see
  // handleBinary and protocol.py.
  let pendingBinaryType = null;
  let pendingHeader = null;
  // Warn at most this often, so a broken link cannot flood the console.
  const DESYNC_WARN_MS = 5000;
  const DESYNC_BANNER_MS = 10000;
  let lastDesyncWarnAt = 0;

  // ---------------------------------------------------------------------
  // View transform: world meters (map frame: +X right, +Y up, exactly as
  // ROS/REP-103 use) <-> canvas pixels (+X right, +Y DOWN, standard for
  // <canvas>). `scale` is canvas pixels per meter; (centerX, centerY) is
  // the world point currently drawn at the canvas's own center.
  // ---------------------------------------------------------------------
  const view = {
    scale: 100,
    centerX: 0,
    centerY: 0,
    // Robot-centric mode has no map/world frame to pan around in, so it
    // gets its own pan offset in body-frame meters (bodyPanX = forward,
    // bodyPanY = left) rather than reusing centerX/centerY, which only
    // mean something once a map or pose exists.
    bodyPanX: 0,
    bodyPanY: 0,
    userAdjusted: false, // once the user pans/zooms, stop auto-fitting on new data
    fittedToMap: false,  // a map has been framed at least once -- see maybeAutoFit
  };

  function worldToCanvas(wx, wy) {
    return [
      canvas.width / 2 + (wx - view.centerX) * view.scale,
      canvas.height / 2 - (wy - view.centerY) * view.scale, // minus: world +Y is up, canvas +Y is down
    ];
  }

  // Robot-centric fallback transform, used only when no pose has arrived
  // yet: the car is drawn at the canvas center (offset by bodyPan once the
  // user drags), always facing "up", using body-frame coordinates straight
  // off the LIDAR (x forward, y left) -- no map, no pose, no localization
  // needed at all, just /scan.
  function bodyToCanvas(bx, by) {
    return [
      canvas.width / 2 - (by - view.bodyPanY) * view.scale,
      canvas.height / 2 - (bx - view.bodyPanX) * view.scale,
    ];
  }

  // The inverses, for turning a click back into a position. Written once
  // here rather than inline at each call site: zoomAt used to carry the
  // only copy of this arithmetic, and the world-frame/body-frame split is
  // exactly the kind of thing that drifts apart when it exists twice.
  //
  // `clientX/clientY` are CSS pixels; the canvas backing store is scaled by
  // devicePixelRatio, which is why every one of these multiplies by it. The
  // canvas fills the viewport at (0,0), so no getBoundingClientRect() is
  // needed -- if that ever stops being true, it is needed in all three.
  // ---------------------------------------------------------------------
  // Measuring
  //
  // A chain of points on the map, with each leg's length and a running
  // total. Two points is the point-to-point case; more keeps going.
  //
  // The arithmetic and every decision live in web/measure.js, which is pure
  // and unit-tested under node. What is here is the parts that need a
  // canvas: turning a tap into a position, and drawing.
  // ---------------------------------------------------------------------
  const Measure = (typeof window !== 'undefined' && window.__measure) || null;

  const measure = {
    active: false,
    pts: [],
    note: '',       // why the chain was cleared, when it was cleared for us
  };

  // Two points closer together than this on screen are the same point --
  // a double-click delivers two pointerup events a pixel apart, and a
  // zero-length leg is noise in the list.
  const MEASURE_MIN_SEPARATION_PX = 4;

  function canvasToWorld(clientX, clientY) {
    const dpr = window.devicePixelRatio || 1;
    const px = clientX * dpr;
    const py = clientY * dpr;
    return [
      view.centerX + (px - canvas.width / 2) / view.scale,
      view.centerY - (py - canvas.height / 2) / view.scale,
    ];
  }

  function canvasToBody(clientX, clientY) {
    const dpr = window.devicePixelRatio || 1;
    const px = clientX * dpr;
    const py = clientY * dpr;
    // Inverse of bodyToCanvas: body +X is forward (canvas up), +Y is left
    // (canvas left), so both axes are negated relative to the screen.
    return [
      view.bodyPanX + (canvas.height / 2 - py) / view.scale,
      view.bodyPanY + (canvas.width / 2 - px) / view.scale,
    ];
  }

  /**
   * A click, in whichever frame the map is currently being drawn in, tagged
   * with which one that was.
   *
   * The tag is the whole point. A body-frame position means "relative to
   * where the car is right now" and stops meaning anything the moment the
   * car moves; a map-frame one is a place on the track. They are not
   * convertible without a pose, and quietly treating one as the other would
   * put a wrong distance on screen with nothing marking it as wrong.
   */
  function canvasToActive(clientX, clientY) {
    if (state.pose) {
      const [x, y] = canvasToWorld(clientX, clientY);
      return { frame: 'map', x, y };
    }
    const [x, y] = canvasToBody(clientX, clientY);
    return { frame: 'body', x, y };
  }

  // ---------------------------------------------------------------------
  // WebSocket connection, with automatic reconnect -- a dropped WiFi link
  // shouldn't require reloading the page.
  // ---------------------------------------------------------------------
  let ws = null;

  function connect() {
    // wss under https (e.g. a Cloudflare tunnel) — browsers block ws:// there.
    const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
    ws = new WebSocket(`${scheme}://${location.host}/ws`);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => setConnected(true);
    ws.onclose = () => {
      setConnected(false);
      // A dropped link disarms tuning. The server has already forgotten
      // this connection's arm state, so anything else here would be the
      // UI claiming an authority it no longer has.
      setTuningArmed(false);
      setTimeout(connect, 1000); // keep trying -- cheap, and self-heals a dropped link
    };
    ws.onerror = () => ws.close();
    ws.onmessage = onMessage;
  }

  function setConnected(connected) {
    connDot.className = 'dot ' + (connected ? 'dot-green' : 'dot-red');
    connText.textContent = connected ? 'connected' : 'disconnected -- retrying...';
    stripDot.className = 'dot ' + (connected ? 'dot-green' : 'dot-red');
    if (!connected) {
      // Said outright rather than left showing the last state the car was
      // in, which on a strip with no other context reads as current.
      stripState.textContent = 'link lost';
      stripState.className = 'strip-state intent-stop';
      stripSpeed.textContent = '--';
      stripFeeds.textContent = '--';
    }
  }

  function onMessage(event) {
    if (typeof event.data === 'string') {
      handleHeader(JSON.parse(event.data));
    } else {
      handleBinary(event.data);
    }
  }

  function handleHeader(header) {
    if (header.type === 'batch') {
      // One frame carrying everything that happened in the last tick --
      // see web_dashboard/batching.py. Each item is exactly the message it
      // would have been on its own, so every handler below is reused as-is
      // rather than duplicated for the batched case.
      const items = header.items || [];
      for (let i = 0; i < items.length; i++) handleHeader(items[i]);
      return;
    }
    if (header.type === 'map' || header.type === 'map_patch' || header.type === 'scan') {
      pendingBinaryType = header.type;
      pendingHeader = header;
    } else if (header.type === 'pose') {
      state.pose = { x: header.x, y: header.y, yaw: header.yaw, receivedAt: performance.now() };
      maybeAutoFit();
      scheduleRender();
    } else if (header.type === 'drive') {
      state.drive = { speed: header.speed, steeringAngle: header.steering_angle, receivedAt: performance.now() };
      scheduleRender();
    } else if (header.type === 'speed') {
      state.speed = { speed: header.speed, receivedAt: performance.now() };
      scheduleRender();
    } else if (header.type === 'stopwatch') {
      state.stopwatch = {
        elapsedS: header.elapsed_s,
        enabled: header.enabled,
        running: header.running,
        lbHeld: header.lb_held,
        joyFresh: header.joy_fresh,
        buttonAvailable: header.button_available,
        receivedAt: performance.now(),
      };
      scheduleRender();
    } else if (header.type === 'intent') {
      applyIntent(header.intent);
      scheduleRender();
    } else if (header.type === 'racing_line') {
      state.racingLine = header.line || null;
      renderRacingLineStatus();
    } else if (header.type === 'tuning') {
      state.tuning = { enabled: header.enabled, allowSave: header.allow_save, nodes: header.nodes || [] };
      renderTuning();
    } else if (header.type === 'tuning_armed') {
      setTuningArmed(header.armed);
    } else if (header.type === 'tuning_result') {
      applyTuningResult(header);
    } else if (header.type === 'tuning_saved') {
      showTuningSaveResult(header);
    } else if (header.type === 'processes') {
      state.processes = { enabled: header.enabled, targets: header.targets || [] };
      renderProcesses();
    } else if (header.type === 'process_result') {
      applyProcessResult(header);
    } else if (header.type === 'saved_maps') {
      state.savedMaps = header;
      renderSavedMaps();
    } else if (header.type === 'map_delete_result') {
      applyMapDeleteResult(header);
    } else if (header.type === 'slam_reset_result') {
      applySlamResetResult(header);
    } else if (header.type === 'map_cleared') {
      applyMapCleared(header);
    } else if (header.type === 'stats') {
      state.stats = {
        cpuPercent: header.cpu_percent,
        memPercent: header.mem_percent,
        cpuTempC: header.cpu_temp_c,
        uptimeS: header.uptime_s,
        wifiDbm: header.wifi_dbm,
        receivedAt: performance.now(),
      };
      scheduleRender();
    }
  }

  function noteDesync(detail) {
    state.desyncCount += 1;
    state.desyncAt = performance.now();
    state.desyncDetail = detail;
    if (state.desyncAt - lastDesyncWarnAt >= DESYNC_WARN_MS) {
      lastDesyncWarnAt = state.desyncAt;
      console.warn(`dashboard: dropped a binary frame -- ${detail} `
        + `(${state.desyncCount} so far)`);
    }
    scheduleRender();
  }

  function handleBinary(buffer) {
    // Consume the pending slot *first*, whatever happens next. Leaving a
    // stale header in it is precisely the failure being guarded against:
    // the next payload would then be decoded as the previous type, and a
    // 1081-beam scan read as occupancy cells is 4324 bytes against an
    // 80000-cell header -- every read past the end is undefined, every
    // colour computes to NaN, and the map paints as garbage instead of
    // failing.
    const header = pendingHeader;
    const type = pendingBinaryType;
    pendingBinaryType = null;
    pendingHeader = null;

    if (!header) {
      noteDesync('binary frame with no header before it');
      return;
    }
    // `bytes` is what the server says must follow. Older servers do not
    // send it; fall back to the type's own arithmetic rather than
    // trusting the pairing blindly.
    const expected = typeof header.bytes === 'number'
      ? header.bytes
      : (type === 'map' ? header.width * header.height : 4 * header.count);
    if (buffer.byteLength !== expected) {
      noteDesync(`${type} payload is ${buffer.byteLength} bytes, header says ${expected}`);
      return;
    }

    if (type === 'map' || type === 'map_patch') {
      queueMapFrame(type, header, buffer);
      return; // the chain re-renders once the frame has actually landed
    }
    if (type === 'scan') {
      applyScan(header, buffer);
    }
    maybeAutoFit();
    scheduleRender();
  }

  // ---------------------------------------------------------------------
  // Map frames arrive as a keyframe (the whole grid) followed by patches
  // (just the rectangle that changed) -- see web_dashboard/mapstream.py.
  // Decoding one is asynchronous, because inflating is, and patches MUST be
  // applied in the order the car sent them: applying two out of order would
  // leave the map quietly wrong rather than visibly broken. So every map
  // frame goes through one promise chain, which serialises them no matter
  // how the inflates interleave.
  // ---------------------------------------------------------------------
  let mapChain = Promise.resolve();

  function queueMapFrame(type, header, buffer) {
    mapChain = mapChain
      .then(() => applyMapFrame(type, header, buffer))
      .then(() => { maybeAutoFit(); scheduleRender(); })
      .catch((err) => {
        noteDesync(`could not decode a ${type} frame -- ${err && err.message ? err.message : err}`);
      });
  }

  // Whether this browser can inflate at all. DecompressionStream is in
  // every current browser (Chrome 80+, Firefox 113+, Safari 16.4+), but a
  // silently blank map on an older one would be a miserable thing to
  // debug, so it is detected and reported rather than assumed.
  const CAN_INFLATE = typeof DecompressionStream === 'function';
  let warnedAboutInflate = false;

  async function decodePayload(header, buffer) {
    const expected = header.raw_bytes;
    if (header.encoding !== 'deflate') {
      const raw = new Uint8Array(buffer);
      if (typeof expected === 'number' && raw.length !== expected) {
        throw new Error(`payload is ${raw.length} bytes, header says ${expected}`);
      }
      return raw;
    }
    if (!CAN_INFLATE) {
      if (!warnedAboutInflate) {
        warnedAboutInflate = true;
        console.error(
          'dashboard: this browser has no DecompressionStream, so the '
          + 'compressed map cannot be drawn. Set map_compression: false in '
          + 'web_dashboard.yaml, or use a newer browser.');
      }
      throw new Error('this browser cannot inflate the map (set map_compression: false)');
    }
    const stream = new Blob([buffer]).stream()
      .pipeThrough(new DecompressionStream('deflate'));
    const raw = new Uint8Array(await new Response(stream).arrayBuffer());
    if (typeof expected === 'number' && raw.length !== expected) {
      throw new Error(`inflated to ${raw.length} bytes, header says ${expected}`);
    }
    return raw;
  }

  async function applyMapFrame(type, header, buffer) {
    const raw = await decodePayload(header, buffer);
    // Int8 view over the same bytes: occupancy values are signed (-1 is
    // 'unknown'), and the palette below is indexed by the raw byte anyway.
    if (type === 'map') {
      applyMap(header, raw);
    } else {
      applyMapPatch(header, raw);
    }
  }

  // ---------------------------------------------------------------------
  // Map palette. Deliberately NOT the ROS/RViz convention of white free
  // space on a mid-gray unknown: on this dashboard's dark theme that put a
  // glaring white slab in the middle of the canvas, washed out the
  // proximity-colored scan drawn on top of it, and read as a foreign image
  // pasted onto the UI rather than part of it. Inverted instead, in the
  // same palette style.css uses everywhere else:
  //
  //   unknown  -- a near-transparent hint of the panel border color, so
  //               unmapped area recedes into the page background instead
  //               of dominating it (drawMap outlines the map's extent, so
  //               nothing is lost by letting it fade out)
  //   free     -- a dark slate "track surface", clearly a mapped region
  //               but dim enough to sit behind the scan and car
  //   occupied -- the bright end: walls are the actual information in an
  //               occupancy grid, and desaturated blue-gray keeps them
  //               from competing with the saturated red/yellow/green of
  //               the LIDAR points or the red car icon.
  //
  // Intermediate probabilities (1..99) interpolate between free and
  // occupied, so a half-confident wall still reads as a half-bright one.
  // ---------------------------------------------------------------------
  const MAP_FREE_RGB = [14, 30, 44];        // #0e1e2c -- cyan-shifted "track surface"
  const MAP_OCCUPIED_RGB = [138, 190, 216]; // #8abed8 -- walls, the actual information
  const MAP_UNKNOWN_RGBA = [30, 54, 72, 46]; // faint cyan haze at ~18% alpha
  const MAP_EDGE_COLOR = 'rgba(61, 220, 255, 0.30)'; // same hairline as every panel border

  // ---------------------------------------------------------------------
  // Turning a raw occupancy grid into something drawable, once per map
  // update (not once per frame): render it into an off-screen canvas at
  // its native resolution (1 pixel per cell), so the visible canvas can
  // just scale/position that image with a single fast drawImage() call
  // every frame instead of redrawing every cell every frame.
  // ---------------------------------------------------------------------
  // A 256-entry palette indexed by the RAW BYTE of each cell, so colouring
  // is one lookup and one 32-bit store per cell instead of a branch and
  // four byte stores. That matters: a 2048x2048 keyframe is 4.2 million
  // cells, and this runs on whatever laptop or phone is watching.
  //
  // Indexing by the raw byte is what removes the branch. Occupancy values
  // are signed: 0..100 are probabilities and -1 is "unknown", which as a
  // byte is 255. So bytes 0..100 take the gradient, 101..127 clamp to
  // occupied (out of spec, but that is what the old code did with them),
  // and 128..255 -- every negative value -- are unknown.
  const LITTLE_ENDIAN = (() => {
    const probe = new ArrayBuffer(4);
    new Uint32Array(probe)[0] = 0x01020304;
    return new Uint8Array(probe)[0] === 0x04;
  })();

  const MAP_PALETTE = (() => {
    const lut = new Uint32Array(256);
    const pack = (r, g, b, a) => (LITTLE_ENDIAN
      ? ((a * 16777216) + (b * 65536) + (g * 256) + r)
      : ((r * 16777216) + (g * 65536) + (b * 256) + a)) >>> 0;
    const unknown = pack(MAP_UNKNOWN_RGBA[0], MAP_UNKNOWN_RGBA[1],
                         MAP_UNKNOWN_RGBA[2], MAP_UNKNOWN_RGBA[3]);
    for (let byte = 0; byte < 256; byte++) {
      if (byte > 127) { lut[byte] = unknown; continue; } // negative int8
      const t = Math.min(byte, 100) / 100;               // 0 free -> 1 occupied
      lut[byte] = pack(
        Math.round(MAP_FREE_RGB[0] + (MAP_OCCUPIED_RGB[0] - MAP_FREE_RGB[0]) * t),
        Math.round(MAP_FREE_RGB[1] + (MAP_OCCUPIED_RGB[1] - MAP_FREE_RGB[1]) * t),
        Math.round(MAP_FREE_RGB[2] + (MAP_OCCUPIED_RGB[2] - MAP_FREE_RGB[2]) * t),
        255);
    }
    return lut;
  })();

  // OccupancyGrid.data is row-major with row 0 at the *bottom* of the map
  // (smallest world Y); a plain <canvas> image has row 0 at the *top*.
  // Flipping rows here, once, means everywhere else in this file can treat
  // "top of the map image" as "largest world Y" without re-deriving it.
  // Patches arrive in the same grid coordinates, so they flip the same way
  // -- see applyMapPatch.
  function paintCells(target, cells, width, height) {
    const words = new Uint32Array(target.data.buffer);
    for (let row = 0; row < height; row++) {
      const src = (height - 1 - row) * width;
      const dst = row * width;
      for (let col = 0; col < width; col++) {
        words[dst + col] = MAP_PALETTE[cells[src + col]];
      }
    }
  }

  function applyMap(header, cells) {
    const { width, height, resolution, origin_x: originX, origin_y: originY } = header;
    // Belt and braces with handleBinary's length check: this indexes
    // width*height entries and a read past the end would paint garbage
    // rather than throwing.
    if (!(width > 0 && height > 0) || cells.length < width * height) {
      noteDesync(`map payload holds ${cells.length} cells, header says ${width * height}`);
      return;
    }
    const off = document.createElement('canvas');
    off.width = width;
    off.height = height;
    const octx = off.getContext('2d');
    const img = octx.createImageData(width, height);
    paintCells(img, cells, width, height);
    octx.putImageData(img, 0, 0);

    state.map = {
      width, height, resolution, originX, originY,
      canvas: off, ctx: octx,
      seq: typeof header.seq === 'number' ? header.seq : null,
      receivedAt: performance.now(),
    };
  }

  // Just the rectangle that changed, blitted into the image we already
  // hold. This is what lets the car send ~200 bytes instead of 4MB while it
  // is mapping, and it is the clearest case of the browser rather than the
  // car doing the work of keeping a map picture current.
  function applyMapPatch(header, cells) {
    const map = state.map;
    if (!map) return; // no keyframe yet; the next one brings everything
    const { x, y, w, h, seq } = header;

    // A patch describes a change from one exact grid to the next. Applied
    // to anything else it would leave the map quietly, plausibly wrong --
    // so on any gap we stop applying patches and wait for the keyframe the
    // car sends every map_keyframe_sec.
    if (map.seq === null || seq !== map.seq + 1) {
      noteDesync(`map patch ${seq} does not follow frame ${map.seq}; waiting for a keyframe`);
      return;
    }
    if (!(w > 0 && h > 0) || x < 0 || y < 0
        || x + w > map.width || y + h > map.height) {
      noteDesync(`map patch (${x},${y},${w},${h}) does not fit a ${map.width}x${map.height} map`);
      return;
    }
    if (cells.length < w * h) {
      noteDesync(`map patch holds ${cells.length} cells, header says ${w * h}`);
      return;
    }

    const img = map.ctx.createImageData(w, h);
    paintCells(img, cells, w, h);
    // Grid rows [y, y+h) are image rows [height-y-h, height-y): paintCells
    // flips the patch within itself, and this puts that flipped block at
    // the mirrored offset.
    map.ctx.putImageData(img, x, map.height - y - h);
    map.seq = seq;
    map.receivedAt = performance.now();
  }

  function applyScan(header, buffer) {
    // 'u16mm' is millimetres in a uint16: half the bytes of float32, for a
    // difference far below one screen pixel and below the LIDAR's own
    // accuracy. 0 means "nothing came back", which the drawing code already
    // discards because it is under every real scanner's range_min.
    let ranges;
    if (header.encoding === 'u16mm') {
      const millimetres = new Uint16Array(buffer);
      ranges = new Float32Array(millimetres.length);
      for (let i = 0; i < millimetres.length; i++) ranges[i] = millimetres[i] / 1000;
    } else {
      ranges = new Float32Array(buffer);
    }
    state.scan = {
      angleMin: header.angle_min,
      angleIncrement: header.angle_increment,
      rangeMin: header.range_min,
      rangeMax: header.range_max,
      laserOffsetX: header.laser_offset_x,
      laserOffsetY: header.laser_offset_y,
      ranges,
      receivedAt: performance.now(),
    };
  }

  // ---------------------------------------------------------------------
  // Auto-fit the view the first time a map (or, lacking a map, a pose)
  // arrives -- but only until the user manually pans/zooms, so this never
  // fights their input.
  // ---------------------------------------------------------------------
  // Fraction of the shorter canvas dimension kept as a border when fitting,
  // and how much of it a growing map may eat before the view is re-fitted.
  // Re-fitting on every map instead was measurably the worst thing on this
  // screen during a mapping run: slam_toolbox resizes and re-origins its
  // grid constantly as the map grows -- 27 map messages in 130 seconds,
  // shrinking as often as growing -- and re-deriving centre and zoom from
  // each one moved the whole picture by up to 3.6m and rescaled it by up to
  // 35%, sixteen times, while the map itself was perfectly good. A viewer
  // reasonably reads that as the map being glitchy. It is the camera.
  const FIT_MARGIN = 1.15;

  function mapWorldExtent() {
    const { width, height, resolution, originX, originY } = state.map;
    return {
      minX: originX,
      maxX: originX + width * resolution,
      minY: originY,
      maxY: originY + height * resolution,
    };
  }

  function extentIsVisible(extent) {
    const halfWidth = canvas.width / (2 * view.scale);
    const halfHeight = canvas.height / (2 * view.scale);
    return (extent.minX >= view.centerX - halfWidth
      && extent.maxX <= view.centerX + halfWidth
      && extent.minY >= view.centerY - halfHeight
      && extent.maxY <= view.centerY + halfHeight);
  }

  function maybeAutoFit() {
    if (view.userAdjusted) return;
    if (state.map) {
      const extent = mapWorldExtent();
      // Grow to fit, never twitch. Once the map is framed, a grid that
      // resizes by a few cells changes nothing the viewer needs to see.
      if (view.fittedToMap && extentIsVisible(extent)) return;
      view.centerX = (extent.minX + extent.maxX) / 2;
      view.centerY = (extent.minY + extent.maxY) / 2;
      const spanMeters = Math.max(extent.maxX - extent.minX,
                                  extent.maxY - extent.minY);
      view.scale = Math.min(canvas.width, canvas.height) / (spanMeters * FIT_MARGIN);
      view.fittedToMap = true;
    } else if (state.pose) {
      view.centerX = state.pose.x;
      view.centerY = state.pose.y;
    }
  }

  // ---------------------------------------------------------------------
  // Rendering
  //
  // Everything that changes state asks for a repaint through
  // scheduleRender() rather than painting immediately. Updates arrive far
  // faster than a screen can show them -- pose alone used to force a full
  // canvas repaint 40 times a second, and a batch frame carries several
  // updates that would each have triggered their own -- so they are
  // coalesced into at most one repaint per animation frame.
  //
  // The other half of the win is free: requestAnimationFrame does not fire
  // in a hidden tab, so a dashboard left open on a second monitor or a
  // phone in someone's pocket stops drawing entirely while still tracking
  // everything the car sends.
  // ---------------------------------------------------------------------
  let renderPending = false;

  function scheduleRender() {
    if (renderPending) return;
    renderPending = true;
    requestAnimationFrame(() => {
      renderPending = false;
      render();
    });
  }

  // ---------------------------------------------------------------------
  // Which coordinate frame each overlay may be drawn in, given what has
  // actually arrived. One function, because the three used to decide it
  // separately and drifted apart:
  //
  //   'map'  -- world coordinates, locked to the map
  //   'body' -- the car's own frame, car fixed at the view origin
  //   'none' -- we do not know enough to draw it honestly
  //
  // The rule that ties them together: a body-frame picture is only honest
  // when there is NO map. With a map on screen and no pose, "the car is at
  // the middle of the view, pointing up" is not a fact about the car, it
  // is a fact about the viewport -- and drawn over a world-frame map it
  // reads as a real position. The scan already refused to draw in that
  // state; the car did not, and so sat at the wrong place, facing a fixed
  // "up" regardless of its real heading, sliding against the map whenever
  // the view was re-fitted. That is the bug this function exists to make
  // impossible to reintroduce in one overlay but not the others.
  // ---------------------------------------------------------------------
  function drawFrames(has) {
    const mapRelative = !!has.pose;
    const bodyIsHonest = !has.map;
    const overlay = (present) => {
      if (!present) return 'none';
      if (mapRelative && has.map) return 'map';
      return bodyIsHonest ? 'body' : 'none';
    };
    return {
      scan: overlay(has.scan),
      intent: overlay(has.intent),
      // The car follows the same rule, and specifically must land in the
      // SAME frame as the scan whenever both are drawn. It used to go to
      // world coordinates on a pose alone -- so with a pose but no map
      // yet, the car was projected through worldToCanvas while its own
      // LIDAR was projected through bodyToCanvas, and the car floated
      // away from the scan it had produced.
      car: (mapRelative && has.map) ? 'map'
        : (bodyIsHonest && has.scan) ? 'body'
        : 'none',
    };
  }
  if (typeof window !== 'undefined') window.__drawFrames = drawFrames;

  // ---------------------------------------------------------------------
  // Paint order for the three live overlays, as data rather than as the
  // order four `if` statements happen to be written in.
  //
  // The rule this exists to hold: the SCAN IS PAINTED AFTER THE CAR. The
  // car icon is a real 0.36 x 0.30 m footprint drawn to scale, and the
  // LIDAR sits inside that outline (0.26 m ahead of the rear axle), so at
  // any zoom close enough to be useful the icon covers the beams that
  // matter most -- the ones reading a wall the car is about to touch. An
  // obstacle hidden underneath the picture of the car is precisely the
  // obstacle you need to see. The car being drawn over its own scan was
  // the bug; the sizes of the icon make it worse, not better, which is
  // why the order is asserted in test/browser/car_model_test.js.
  //
  // The blind-spot wedge stays first: it is a translucent red fill over
  // everything the LIDAR cannot see, and painting it late would tint the
  // car and the scan points instead of sitting behind them. Intent stays
  // under the car so the arrow reads as belonging to the car.
  // ---------------------------------------------------------------------
  function overlayDrawOrder(frames) {
    const steps = [];
    if (frames.scan !== 'none') steps.push('blindspot');
    if (frames.intent !== 'none') steps.push('intent');
    if (frames.car !== 'none') steps.push('car');
    if (frames.scan !== 'none') steps.push('scan');
    return steps;
  }
  if (typeof window !== 'undefined') window.__overlayDrawOrder = overlayDrawOrder;

  function render() {
    resizeCanvasIfNeeded();
    ctx.fillStyle = HUD.void;
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    const mapRelative = !!state.pose; // do we know exactly where the car is in the map frame?

    // Behind the map on purpose: mapped ground is opaque and covers it,
    // so the grid shows exactly where the car has no map yet.
    drawBackdropGrid(mapRelative ? worldToCanvas : bodyToCanvas);

    if (state.map) {
      drawMap();
      modeBanner.textContent = mapRelative ? '' : 'map loaded -- waiting for a localization pose (RViz "2D Pose Estimate"?)';
    } else {
      modeBanner.textContent = 'no map yet -- showing raw LIDAR relative to the car';
    }

    // Last, so it wins: a dropped frame is the thing most likely to be
    // making the picture wrong, and "the map looks glitchy" with nothing
    // on screen to explain it is exactly the situation this avoids.
    if (state.desyncCount > 0
        && performance.now() - state.desyncAt < DESYNC_BANNER_MS) {
      modeBanner.textContent =
        `link problem: dropped ${state.desyncCount} corrupted frame(s) `
        + `-- ${state.desyncDetail}. The picture below may be stale.`;
    }

    // One decision for all three overlays -- see drawFrames above. A
    // 'none' here means "a map is up but no pose has arrived": the banner
    // already says so, and drawing any of them anyway would be a guess
    // dressed up as data.
    const frames = drawFrames({
      map: !!state.map, pose: !!state.pose,
      scan: !!state.scan, intent: !!state.intent,
    });

    // Over the map, under everything live. The line is fixed geometry the
    // car is trying to follow; the scan, intent and car are what is
    // happening now, and they should never be hidden behind it.
    if (state.map && state.showRacingLine && state.racingLine) {
      drawRacingLine();
    }

    for (const step of overlayDrawOrder(frames)) {
      if (step === 'blindspot') {
        if (frames.scan === 'map') drawBlindSpotMapRelative();
        else drawBlindSpotRobotCentric();
      } else if (step === 'intent') {
        // Intent under the car icon, so the car always reads as the thing
        // the arrow belongs to.
        drawIntent(frames.intent === 'map');
      } else if (step === 'car') {
        if (frames.car === 'map') drawCarMapRelative();
        else drawCarRobotCentric();
      } else if (step === 'scan') {
        if (frames.scan === 'map') drawScanMapRelative();
        else drawScanRobotCentric();
      }
    }

    // Last, over everything: this is the tool the person is actively
    // driving, and a label hidden behind a scan point is a label that
    // cannot be read. Cyan throughout -- the doc's colour rule reserves
    // green, amber and red for what the CAR has decided, and a measurement
    // is the system talking, not a verdict about the drive.
    drawMeasure(mapRelative);

    updateScaleBar();
    drawMinimap();

    updateStatusText();
  }

  function drawMeasure(mapRelative) {
    if (!Measure) return;
    // A chain taken before localization converged is relative to where the
    // car was standing; once a pose exists it would be drawn somewhere it
    // never was. Drop it and say so rather than showing a confident wrong
    // line. (state.pose is set once and never cleared, so in practice this
    // only fires body -> map, when localization first gets a fix.)
    const frame = Measure.frameFor(measure.pts, !!state.pose);
    if (frame === 'stale') {
      measure.pts = [];
      measure.note = state.pose
        ? 'a localization pose arrived, so the robot-centric measurement was '
          + 'cleared -- those points were relative to the car, not the map'
        : 'the localization pose was lost, so the measurement was cleared';
      renderMeasurePanel();
      return;
    }
    if (!measure.pts.length) return;

    const project = mapRelative ? worldToCanvas : bodyToCanvas;
    const screen = measure.pts.map((p) => project(p.x, p.y));

    ctx.save();
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';

    // Dark halo under the line, same trick as the racing line: it has to
    // read over both the pale mapped ground and the dark unmapped void.
    if (screen.length > 1) {
      ctx.beginPath();
      ctx.moveTo(screen[0][0], screen[0][1]);
      for (let i = 1; i < screen.length; i++) ctx.lineTo(screen[i][0], screen[i][1]);
      ctx.strokeStyle = 'rgba(0,0,0,0.6)';
      ctx.lineWidth = 6;
      ctx.stroke();
      ctx.strokeStyle = HUD.accent;
      ctx.lineWidth = 2;
      ctx.setLineDash([]);
      ctx.stroke();
    }

    // Vertices. The first gets a wider ring so the chain's direction is
    // readable without labels.
    screen.forEach(([cx, cy], index) => {
      ctx.beginPath();
      ctx.arc(cx, cy, index === 0 ? 6 : 4, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(0,0,0,0.6)';
      ctx.fill();
      ctx.strokeStyle = HUD.accent;
      ctx.lineWidth = 2;
      ctx.stroke();
    });

    // One label per leg, in screen space at a fixed size -- never scaled
    // with the view, or a zoomed-out chain would be unreadable and a
    // zoomed-in one absurd.
    const dpr = window.devicePixelRatio || 1;
    const segments = Measure.segments(measure.pts);
    for (let i = 0; i < segments.length; i++) {
      const a = screen[i];
      const b = screen[i + 1];
      const plan = Measure.labelPlacement(
        { ax: a[0], ay: a[1], bx: b[0], by: b[1] },
        Measure.MIN_LABEL_PX * dpr);
      // Too short to label. Its length still counts toward the total,
      // which is always on screen in the measure panel.
      if (!plan) continue;
      drawMeasureLabel(plan, Measure.formatDistance(segments[i].length), dpr);
    }
    ctx.restore();
  }

  function drawMeasureLabel(plan, text, dpr) {
    ctx.save();
    ctx.translate(plan.x + plan.offsetX * dpr, plan.y + plan.offsetY * dpr);
    ctx.rotate(plan.angle);
    ctx.font = `${11 * dpr}px ui-monospace, Menlo, Consolas, monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const width = ctx.measureText(text).width;
    const padX = 4 * dpr;
    const padY = 3 * dpr;
    const height = 12 * dpr;
    ctx.fillStyle = 'rgba(0,0,0,0.72)';
    ctx.fillRect(-width / 2 - padX, -height / 2 - padY,
                 width + padX * 2, height + padY * 2);
    ctx.fillStyle = HUD.accent;
    ctx.fillText(text, 0, 0);
    ctx.restore();
  }

  // ---------------------------------------------------------------------
  // Minimap (top-right inset): always shows the *whole* map, independent
  // of the main canvas's own pan/zoom, plus a rectangle for what the main
  // view currently shows and a small car marker -- so zooming into a
  // corner of the track on the main canvas doesn't lose the big picture.
  // ---------------------------------------------------------------------
  function resizeMinimapIfNeeded() {
    const dpr = window.devicePixelRatio || 1;
    const targetW = Math.round(minimapCanvas.clientWidth * dpr);
    const targetH = Math.round(minimapCanvas.clientHeight * dpr);
    if (targetW > 0 && (minimapCanvas.width !== targetW || minimapCanvas.height !== targetH)) {
      minimapCanvas.width = targetW;
      minimapCanvas.height = targetH;
    }
  }

  function drawMinimap() {
    resizeMinimapIfNeeded();
    minimapCtx.fillStyle = HUD.void;
    minimapCtx.fillRect(0, 0, minimapCanvas.width, minimapCanvas.height);

    if (!state.map) {
      minimapPanel.classList.remove('has-map');
      return;
    }
    minimapPanel.classList.add('has-map');

    const { canvas: mapCanvas, width, height, resolution, originX, originY } = state.map;
    const spanMeters = Math.max(width, height) * resolution;
    const scale = Math.min(minimapCanvas.width, minimapCanvas.height) / (spanMeters * 1.15);
    const centerX = originX + (width * resolution) / 2;
    const centerY = originY + (height * resolution) / 2;
    const toMinimap = (wx, wy) => [
      minimapCanvas.width / 2 + (wx - centerX) * scale,
      minimapCanvas.height / 2 - (wy - centerY) * scale,
    ];

    const [x0, y0] = toMinimap(originX, originY + height * resolution);
    const [x1, y1] = toMinimap(originX + width * resolution, originY);
    minimapCtx.imageSmoothingEnabled = false;
    minimapCtx.drawImage(mapCanvas, x0, y0, x1 - x0, y1 - y0);
    minimapCtx.strokeStyle = MAP_EDGE_COLOR; // same map-extent hairline as the main canvas
    minimapCtx.lineWidth = 1;
    minimapCtx.strokeRect(x0 + 0.5, y0 + 0.5, x1 - x0 - 1, y1 - y0 - 1);

    if (!state.pose) return; // no map-frame pose yet -- nothing meaningful to overlay

    // Outline of what the main canvas currently shows, so the minimap
    // reads as "you are here", not just a static overview.
    const halfW = (canvas.width / 2) / view.scale;
    const halfH = (canvas.height / 2) / view.scale;
    const corners = [
      [view.centerX - halfW, view.centerY - halfH],
      [view.centerX + halfW, view.centerY - halfH],
      [view.centerX + halfW, view.centerY + halfH],
      [view.centerX - halfW, view.centerY + halfH],
    ].map(([wx, wy]) => toMinimap(wx, wy));
    // The HUD accent cyan rather than plain white: it reads as a UI
    // element on top of the map instead of another shade of map.
    minimapCtx.strokeStyle = 'rgba(61, 220, 255, 0.80)';
    minimapCtx.lineWidth = 1;
    minimapCtx.beginPath();
    minimapCtx.moveTo(corners[0][0], corners[0][1]);
    for (let i = 1; i < corners.length; i++) minimapCtx.lineTo(corners[i][0], corners[i][1]);
    minimapCtx.closePath();
    minimapCtx.stroke();

    // Small car marker -- doesn't need the full car icon at this scale.
    const [cx, cy] = toMinimap(state.pose.x, state.pose.y);
    minimapCtx.save();
    minimapCtx.translate(cx, cy);
    minimapCtx.rotate(-state.pose.yaw);
    minimapCtx.beginPath();
    minimapCtx.moveTo(5, 0);
    minimapCtx.lineTo(-3, 3);
    minimapCtx.lineTo(-3, -3);
    minimapCtx.closePath();
    minimapCtx.fillStyle = HUD.accent;
    minimapCtx.fill();
    minimapCtx.restore();
  }

  // A classic map "ruler": picks a round length (in meters) that renders
  // at least 60 CSS pixels at the current zoom. It is an HTML element in
  // the right rail rather than canvas paint, so no floating panel can
  // cover it.
  const SCALE_BAR_STEPS_M = [0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200];

  // A world-aligned reference grid, drawn behind everything else.
  //
  // It earns its place rather than being texture: the spacing is one of
  // the same round meter steps the scale bar reports, so a distance on the
  // map can be counted off in squares -- and it gives pan/zoom something
  // fixed to move against. Without it, dragging across an unmapped area
  // shows no motion at all, because there is nothing there to move.
  const GRID_MIN_PX = 80;   // never denser than this on screen
  const GRID_COLOR = 'rgba(61, 220, 255, 0.055)';

  function drawBackdropGrid(project) {
    const dpr = window.devicePixelRatio || 1;
    if (!(view.scale > 0) || !isFinite(view.scale)) return;

    let spacing = SCALE_BAR_STEPS_M[0];
    for (const step of SCALE_BAR_STEPS_M) {
      spacing = step;
      if (step * view.scale >= GRID_MIN_PX * dpr) break;
    }
    const pitch = spacing * view.scale;
    // Below a few pixels the grid is moire rather than information, and
    // the loops below would run once per pixel column. Bail instead.
    if (!isFinite(pitch) || pitch < 4) return;

    const [originX, originY] = project(0, 0);
    if (!isFinite(originX) || !isFinite(originY)) return;

    ctx.save();
    ctx.strokeStyle = GRID_COLOR;
    ctx.lineWidth = 1;
    ctx.beginPath();
    // Anchor the lines to the projected world origin, so the grid is
    // genuinely fixed to the world and slides under the car rather than
    // being painted onto the screen.
    for (let x = originX - Math.floor(originX / pitch) * pitch; x < canvas.width; x += pitch) {
      ctx.moveTo(Math.round(x) + 0.5, 0);
      ctx.lineTo(Math.round(x) + 0.5, canvas.height);
    }
    for (let y = originY - Math.floor(originY / pitch) * pitch; y < canvas.height; y += pitch) {
      ctx.moveTo(0, Math.round(y) + 0.5);
      ctx.lineTo(canvas.width, Math.round(y) + 0.5);
    }
    ctx.stroke();
    ctx.restore();
  }

  function updateScaleBar() {
    const dpr = window.devicePixelRatio || 1;
    const cssPixelsPerMeter = view.scale / dpr;
    let meters = SCALE_BAR_STEPS_M[0];
    for (const step of SCALE_BAR_STEPS_M) {
      meters = step;
      if (step * cssPixelsPerMeter >= 60) break;
    }
    scaleRuler.style.width = `${meters * cssPixelsPerMeter}px`;
    scaleLabel.textContent = meters >= 1 ? `${meters} m` : `${Math.round(meters * 100)} cm`;
  }

  function resizeCanvasIfNeeded() {
    const dpr = window.devicePixelRatio || 1;
    const targetW = Math.round(window.innerWidth * dpr);
    const targetH = Math.round(window.innerHeight * dpr);
    if (canvas.width !== targetW || canvas.height !== targetH) {
      canvas.width = targetW;
      canvas.height = targetH;
      canvas.style.width = window.innerWidth + 'px';
      canvas.style.height = window.innerHeight + 'px';
    }
  }

  function drawMap() {
    const { canvas: mapCanvas, width, height, resolution, originX, originY } = state.map;
    // Top-left of the map IMAGE (row 0, after the flip done in applyMap)
    // is the map's largest-Y, smallest-X corner in world coordinates.
    const [x0, y0] = worldToCanvas(originX, originY + height * resolution);
    const [x1, y1] = worldToCanvas(originX + width * resolution, originY);
    ctx.imageSmoothingEnabled = false; // crisp cell boundaries, not a blurry interpolation
    ctx.drawImage(mapCanvas, x0, y0, x1 - x0, y1 - y0);
    // Hairline around the grid's extent, matching the panel borders in
    // style.css: with "unknown" deliberately faded almost into the page
    // background, this is what still says "the map covers exactly here"
    // when zoomed out -- and it frames the map as part of the UI.
    ctx.strokeStyle = MAP_EDGE_COLOR;
    ctx.lineWidth = 1;
    ctx.strokeRect(x0 + 0.5, y0 + 0.5, x1 - x0 - 1, y1 - y0 - 1);
  }

  // Proximity scale. Anything at or inside 10cm is fully red; anything at
  // or beyond 2m is fully green, with orange and yellow in between. The
  // band is deliberately tight around the distances that matter to a car
  // this size -- a 5m ceiling spent most of the ramp on ranges where
  // nothing is at stake, and left everything inside a metre looking
  // much the same shade. Keep .lidar-gradient in style.css in step.
  const LIDAR_NEAR_M = 0.1;
  const LIDAR_FAR_M = 2.0;
  const LIDAR_COLORS = Array.from({ length: 17 }, (_, i) => {
    const hue = Math.round((i / 16) * 120); // red (near) -> yellow -> green (far)
    // Brighter and more saturated than a chart palette would be: these
    // are single pixels on a near-black field, and they have to read as
    // emitted light rather than as ink. Matches .lidar-gradient in
    // style.css, which is the legend for exactly this scale.
    return `hsl(${hue} 95% 58%)`;
  });

  function lidarColor(range) {
    const normalized = Math.min(1, Math.max(0,
      (range - LIDAR_NEAR_M) / (LIDAR_FAR_M - LIDAR_NEAR_M)));
    return LIDAR_COLORS[Math.round(normalized * (LIDAR_COLORS.length - 1))];
  }

  // Green where the profile wants full speed, amber where it plans to be
  // slow. Two hues only, because a racing line's whole story is "where do
  // I have to slow down" -- a rainbow would say less.
  function racingLineColor(v, vMin, vMax) {
    const span = Math.max(1e-6, vMax - vMin);
    const t = Math.max(0, Math.min(1, (v - vMin) / span));
    // amber (255,182,54) -> green (43,245,138)
    const r = Math.round(255 + (43 - 255) * t);
    const g = Math.round(182 + (245 - 182) * t);
    const b = Math.round(54 + (138 - 54) * t);
    return `rgb(${r},${g},${b})`;
  }

  function drawRacingLine() {
    const line = state.racingLine;
    const pts = line.points || [];
    if (pts.length < 2) return;

    const vMin = Number.isFinite(line.speed_min) ? line.speed_min : 0;
    const vMax = Number.isFinite(line.speed_max) ? line.speed_max : vMin + 1;
    const screen = pts.map((p) => worldToCanvas(p[0], p[1]));
    if (line.closed) screen.push(screen[0]);

    ctx.save();
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';

    // A dark halo underneath, so the line stays readable over both the
    // pale mapped ground and the dark unmapped void.
    ctx.beginPath();
    ctx.moveTo(screen[0][0], screen[0][1]);
    for (let i = 1; i < screen.length; i++) ctx.lineTo(screen[i][0], screen[i][1]);
    ctx.strokeStyle = 'rgba(0,0,0,0.55)';
    ctx.lineWidth = 6;
    ctx.stroke();

    // Then one coloured segment per waypoint pair.
    ctx.lineWidth = 2.5;
    for (let i = 1; i < screen.length; i++) {
      const v = pts[Math.min(i, pts.length - 1)][2];
      ctx.beginPath();
      ctx.moveTo(screen[i - 1][0], screen[i - 1][1]);
      ctx.lineTo(screen[i][0], screen[i][1]);
      ctx.strokeStyle = racingLineColor(v, vMin, vMax);
      ctx.stroke();
    }
    ctx.restore();
  }

  function renderRacingLineStatus() {
    if (!racelineStatus) return;
    const line = state.racingLine;
    if (!line || !(line.points || []).length) {
      racelineStatus.textContent =
        'no racing line loaded -- pure pursuit is not racing yet';
      return;
    }
    const decimated = line.decimation > 1
      ? `, drawn every ${line.decimation} points` : '';
    // Deliberately says "loaded", not "racing". The line appears when a
    // profile is accepted, which under auto_map_race is a couple of
    // seconds before command authority actually moves (transition_stop_sec).
    // The DRIVING badge in the tuning panel is what says it has.
    racelineStatus.textContent =
      `${line.node} loaded ${line.points.length} waypoints, ${line.length_m}m, `
      + `${line.speed_min}-${line.speed_max}m/s${decimated}`;
  }

  function drawScanMapRelative() {
    const { pose } = state;
    const { angleMin, angleIncrement, rangeMin, rangeMax, laserOffsetX, laserOffsetY, ranges } = state.scan;
    const cosYaw = Math.cos(pose.yaw);
    const sinYaw = Math.sin(pose.yaw);
    // The LIDAR's own world position: the car's pose, plus its mounting
    // offset rotated by the car's current heading.
    const laserWorldX = pose.x + laserOffsetX * cosYaw - laserOffsetY * sinYaw;
    const laserWorldY = pose.y + laserOffsetX * sinYaw + laserOffsetY * cosYaw;

    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (!Number.isFinite(r) || r < rangeMin || r > rangeMax) continue;
      ctx.fillStyle = lidarColor(r);
      const angle = pose.yaw + angleMin + i * angleIncrement;
      const wx = laserWorldX + r * Math.cos(angle);
      const wy = laserWorldY + r * Math.sin(angle);
      const [cx, cy] = worldToCanvas(wx, wy);
      ctx.fillRect(cx - 1, cy - 1, 2, 2);
    }
  }

  // Ranges arrive in the `laser` frame, not `base_link`, so the mount
  // offset has to be added here exactly as drawScanMapRelative adds it.
  // Without it the beams radiate from the rear axle instead of from the
  // LIDAR 0.26 m ahead of it, and the whole scan sits a quarter of a metre
  // behind where it belongs relative to the car icon -- invisible while
  // the icon was a vague blob, obvious now that the icon draws the sensor
  // in its real place.
  function drawScanRobotCentric() {
    const {
      angleMin, angleIncrement, rangeMin, rangeMax,
      laserOffsetX, laserOffsetY, ranges,
    } = state.scan;
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (!Number.isFinite(r) || r < rangeMin || r > rangeMax) continue;
      ctx.fillStyle = lidarColor(r);
      const angle = angleMin + i * angleIncrement;
      const bx = laserOffsetX + r * Math.cos(angle);
      const by = laserOffsetY + r * Math.sin(angle);
      const [cx, cy] = bodyToCanvas(bx, by);
      ctx.fillRect(cx - 1, cy - 1, 2, 2);
    }
  }

  // ---------------------------------------------------------------------
  // Blind spot: the arc the LIDAR physically never scans at all (e.g. the
  // Hokuyo's ~270 deg field of view leaves a real gap behind its mount),
  // as opposed to a beam that scanned but found nothing within range --
  // computed from the scan's own angle_min/angle_increment/count, so it's
  // exactly right regardless of what's currently in front of the car,
  // rather than guessed from which beams happen to read "no return" this
  // frame (open space would look identical to a blind spot that way).
  // ---------------------------------------------------------------------
  function drawWedge(ox, oy, a0, a1, angleToPoint) {
    const steps = Math.max(2, Math.round(Math.abs(a1 - a0) / 0.05)); // ~3 deg per segment
    ctx.beginPath();
    ctx.moveTo(ox, oy);
    for (let s = 0; s <= steps; s++) {
      const a = a0 + (a1 - a0) * (s / steps);
      const [x, y] = angleToPoint(a);
      ctx.lineTo(x, y);
    }
    ctx.closePath();
    ctx.fill();
  }

  function blindSpotSpan() {
    const { angleMin, angleIncrement, ranges } = state.scan;
    const angleMax = angleMin + (ranges.length - 1) * angleIncrement;
    const gap = 2 * Math.PI - (angleMax - angleMin);
    return gap > 0.01 ? { from: angleMax, to: angleMin + 2 * Math.PI } : null;
  }

  function drawBlindSpotRobotCentric() {
    const span = blindSpotSpan();
    if (!span) return;
    // Apexed at the LIDAR, not at base_link -- same reason as
    // drawScanRobotCentric above: the wedge is what the sensor cannot see.
    const { rangeMax, laserOffsetX, laserOffsetY } = state.scan;
    const toPoint = (a, r) => bodyToCanvas(
      laserOffsetX + r * Math.cos(a), laserOffsetY + r * Math.sin(a));
    const [ox, oy] = toPoint(0, 0);
    ctx.fillStyle = 'rgba(255, 69, 96, 0.16)';
    drawWedge(ox, oy, span.from, span.to, (a) => toPoint(a, rangeMax));
    drawBlindSpotLabel(ox, oy, (span.from + span.to) / 2, rangeMax, toPoint);
  }

  function drawBlindSpotMapRelative() {
    const span = blindSpotSpan();
    if (!span) return;
    const { pose } = state;
    const { rangeMax, laserOffsetX, laserOffsetY } = state.scan;
    const cosYaw = Math.cos(pose.yaw);
    const sinYaw = Math.sin(pose.yaw);
    const laserWorldX = pose.x + laserOffsetX * cosYaw - laserOffsetY * sinYaw;
    const laserWorldY = pose.y + laserOffsetX * sinYaw + laserOffsetY * cosYaw;
    const toPoint = (a, r) => worldToCanvas(laserWorldX + r * Math.cos(pose.yaw + a), laserWorldY + r * Math.sin(pose.yaw + a));
    const [ox, oy] = toPoint(0, 0);
    ctx.fillStyle = 'rgba(255, 69, 96, 0.16)';
    drawWedge(ox, oy, span.from, span.to, (a) => toPoint(a, rangeMax));
    drawBlindSpotLabel(ox, oy, (span.from + span.to) / 2, rangeMax, toPoint);
  }

  function drawBlindSpotLabel(ox, oy, midAngle, rangeMax, angleToPoint) {
    const [lx, ly] = angleToPoint(midAngle, rangeMax * 0.55);
    ctx.save();
    ctx.fillStyle = 'rgba(220, 236, 247, 0.62)';
    ctx.font = '11px ui-monospace, SFMono-Regular, Menlo, monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('blind spot', lx, ly);
    ctx.restore();
  }

  // ---------------------------------------------------------------------
  // Drive intent: what the algorithm is *trying* to do (docs/drive-intent.md)
  //
  // Deliberately not derived from measured speed or heading -- those are
  // already on screen, and the whole point of this overlay is to show the
  // plan *before* the car acts it out, so a wrong plan can be caught while
  // it is still only a plan.
  //
  // The car publishes in base_link, which is what lets the same arrow draw
  // in robot-centric mode (no map, no pose, just /scan) and in map-relative
  // mode without two code paths or a TF lookup in the browser.
  // ---------------------------------------------------------------------
  function applyIntent(payload) {
    if (!payload || typeof payload !== 'object') return;
    const previous = state.intent;
    if (!previous || previous.state !== payload.state) {
      const now = Date.now();
      const head = state.intentLog[0];
      if (head) head.heldMs = now - head.at;
      state.intentLog.unshift({ state: payload.state, severity: payload.severity, at: now, heldMs: null });
      state.intentLog.length = Math.min(state.intentLog.length, INTENT_LOG_LIMIT);
      // A transition always carries its reason (the car guarantees that),
      // so clearing here can never leave a transition unexplained -- and it
      // stops the previous state's explanation from lingering under a new
      // state label, which would be actively misleading.
      state.intentReason = '';
    }
    if (typeof payload.reason === 'string') state.intentReason = payload.reason;
    state.intent = payload;
    state.intentReceivedAt = performance.now();
  }

  function intentAgeMs() {
    return performance.now() - state.intentReceivedAt;
  }

  // Body frame (+X forward, +Y left) -> canvas, in whichever mode is live.
  function bodyFrameProjector(mapRelative) {
    if (mapRelative && state.pose) {
      const cos = Math.cos(state.pose.yaw);
      const sin = Math.sin(state.pose.yaw);
      const px = state.pose.x;
      const py = state.pose.y;
      return (bx, by) => worldToCanvas(px + bx * cos - by * sin, py + bx * sin + by * cos);
    }
    return (bx, by) => bodyToCanvas(bx, by);
  }

  function intentHalfWidth(v) {
    return Math.max(INTENT_MIN_HALF_WIDTH,
      Math.min(INTENT_MAX_HALF_WIDTH, INTENT_HALF_WIDTH_PER_MPS * Math.abs(v)));
  }

  // Unit tangent at each sample: central difference where possible, so the
  // ribbon's edges stay parallel to the path through a curve instead of
  // kinking at every sample.
  function pathTangents(pts) {
    const out = [];
    for (let i = 0; i < pts.length; i++) {
      const a = pts[Math.max(0, i - 1)];
      const b = pts[Math.min(pts.length - 1, i + 1)];
      let dx = b.x - a.x;
      let dy = b.y - a.y;
      const len = Math.hypot(dx, dy);
      if (len < 1e-9) {
        const prev = out[i - 1];
        out.push(prev || { x: 1, y: 0 });
      } else {
        out.push({ x: dx / len, y: dy / len });
      }
    }
    return out;
  }

  function polylineLength(pts) {
    let total = 0;
    for (let i = 1; i < pts.length; i++) {
      total += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    }
    return total;
  }

  // Cut `back` meters off the end of a polyline, returning the shortened
  // path plus the point where it was cut. The arrow head occupies exactly
  // that trimmed length, so the whole arrow -- head included -- is as long
  // as the distance the plan actually covers, rather than overshooting it
  // by the size of the head.
  function trimTail(pts, back) {
    if (pts.length < 2) return { body: pts.slice(), tip: pts[pts.length - 1] };
    let remaining = back;
    const body = pts.slice();
    while (body.length >= 2) {
      const last = body[body.length - 1];
      const prev = body[body.length - 2];
      const seg = Math.hypot(last.x - prev.x, last.y - prev.y);
      if (seg >= remaining) {
        const t = seg < 1e-9 ? 0 : (seg - remaining) / seg;
        body[body.length - 1] = {
          x: prev.x + (last.x - prev.x) * t,
          y: prev.y + (last.y - prev.y) * t,
          v: last.v,
        };
        return { body, tip: pts[pts.length - 1] };
      }
      remaining -= seg;
      body.pop();
    }
    return { body, tip: pts[pts.length - 1] };
  }

  function drawIntent(mapRelative) {
    const intent = state.intent;
    if (!intent || !state.showIntent) return;
    const project = bodyFrameProjector(mapRelative);
    const colors = intentColors(intent.severity);
    const stale = intentAgeMs() > INTENT_STALE_MS;

    ctx.save();
    if (stale) ctx.globalAlpha = 0.35;

    drawIntentWedge(intent, project, colors);
    // Off by default. The arrow already says where the car is going, and
    // a dashed second path plus a crosshair sitting in front of the car
    // read as more vehicles rather than as annotations on this one.
    //
    // They are still worth having when you want them: the ghost is the
    // *commanded* path against the arrow's *desired* one, so the gap
    // between them is the slew-rate and acceleration shaping -- the
    // answer to "the arrow says 4 m/s and the car is doing 2".
    if (state.showCommanded) {
      // The ghost goes underneath, so where it separates from the ribbon
      // the gap itself is the thing you see.
      drawIntentGhost(intent, project);
    }
    drawIntentRibbon(intent, project, colors);
    if (state.showCommanded) {
      drawIntentTargets(intent, project, colors);
    }

    ctx.restore();
  }

  function drawIntentRibbon(intent, project, colors) {
    const pts = anchoredToCar(intent.path || []);
    if (polylineLength(pts) < 0.05) {
      drawIntentHold(intent, project, colors);
      return;
    }

    const tipHalfWidth = intentHalfWidth(pts[pts.length - 1].v);
    const headLength = Math.max(0.18, 2.0 * tipHalfWidth);
    const { body, tip } = trimTail(pts, headLength);
    if (body.length < 2) {
      drawIntentHold(intent, project, colors);
      return;
    }

    const tangents = pathTangents(body);
    const left = [];
    const right = [];
    for (let i = 0; i < body.length; i++) {
      const half = intentHalfWidth(body[i].v);
      // Normal is the tangent rotated +90deg in the body frame.
      const nx = -tangents[i].y;
      const ny = tangents[i].x;
      left.push(project(body[i].x + nx * half, body[i].y + ny * half));
      right.push(project(body[i].x - nx * half, body[i].y - ny * half));
    }

    ctx.beginPath();
    ctx.moveTo(left[0][0], left[0][1]);
    for (let i = 1; i < left.length; i++) ctx.lineTo(left[i][0], left[i][1]);
    for (let i = right.length - 1; i >= 0; i--) ctx.lineTo(right[i][0], right[i][1]);
    ctx.closePath();
    ctx.fillStyle = colors.fill;
    ctx.fill();
    ctx.strokeStyle = colors.edge;
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Head, based at the trim point and tipped at the true path end.
    const base = body[body.length - 1];
    const dirX = tip.x - base.x;
    const dirY = tip.y - base.y;
    const dirLen = Math.hypot(dirX, dirY) || 1;
    const ux = dirX / dirLen;
    const uy = dirY / dirLen;
    const headHalf = Math.max(1.9 * tipHalfWidth, 0.09);
    const p1 = project(base.x - uy * headHalf, base.y + ux * headHalf);
    const p2 = project(base.x + uy * headHalf, base.y - ux * headHalf);
    const p3 = project(tip.x, tip.y);
    ctx.beginPath();
    ctx.moveTo(p1[0], p1[1]);
    ctx.lineTo(p3[0], p3[1]);
    ctx.lineTo(p2[0], p2[1]);
    ctx.closePath();
    ctx.fillStyle = colors.edge;
    ctx.fill();
  }

  // A stop is not "no intent" -- gap_follow deliberately *holds the rack*
  // where it is while stopped, because centring it would throw away the
  // steering the car needs to get out of trouble. Drawing that held angle
  // is the difference between "stopped" and "stopped, aimed left".
  function drawIntentHold(intent, project, colors) {
    const steer = intent.desired_steering || 0;
    const stub = 0.55;
    const a = project(0, 0);
    const b = project(stub * Math.cos(steer), stub * Math.sin(steer));
    ctx.save();
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = colors.edge;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(b[0], b[1]);
    ctx.stroke();
    ctx.restore();

    ctx.beginPath();
    ctx.arc(a[0], a[1], 7, 0, Math.PI * 2);
    ctx.strokeStyle = colors.edge;
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  // Both intent polylines are in the car's own body frame, so the car is
  // exactly (0, 0) in them. A publisher whose first point is some way out
  // in front left the line hanging in space with a gap between it and the
  // car -- which reads as "that floating thing is a second vehicle"
  // rather than "this is where THIS car is going". Anchoring the line at
  // the origin makes the claim unambiguous: it always grows out of the
  // car that is making it.
  const PATH_ANCHOR_GAP_M = 0.05;

  function anchoredToCar(pts) {
    if (!pts.length) return pts;
    const first = pts[0];
    if (Math.hypot(first.x, first.y) <= PATH_ANCHOR_GAP_M) return pts;
    // Spread the first point rather than building a bare {x, y}: the
    // ribbon reads a per-point speed (`v`) off these to decide its own
    // width, and an anchor without one would widen to NaN.
    return [{ ...first, x: 0, y: 0 }, ...pts];
  }

  function drawIntentGhost(intent, project) {
    const pts = anchoredToCar(intent.commanded_path || []);
    if (polylineLength(pts) < 0.05) return;
    ctx.save();
    ctx.setLineDash([5, 5]);
    ctx.strokeStyle = 'rgba(220, 236, 247, 0.55)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    pts.forEach((p, i) => {
      const [cx, cy] = project(p.x, p.y);
      if (i === 0) ctx.moveTo(cx, cy); else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
    ctx.restore();
  }

  // A reticle -- a hollow ring with a cross through it -- rather than a
  // solid disc. A filled dot sitting out in front of the car, at roughly
  // the car's own on-screen size, read as a second vehicle; the first
  // question anyone asked of this display was "which one is the car?".
  // A sight is not a thing that drives, so it cannot be confused for one.
  function drawIntentTargets(intent, project, colors) {
    (intent.targets || []).forEach((t) => {
      const [cx, cy] = project(t.x, t.y);
      const r = 6;
      ctx.save();
      ctx.strokeStyle = colors.edge;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.stroke();
      // Cross arms, drawn past the ring on both axes.
      ctx.beginPath();
      ctx.moveTo(cx - r - 3, cy);
      ctx.lineTo(cx - 2, cy);
      ctx.moveTo(cx + 2, cy);
      ctx.lineTo(cx + r + 3, cy);
      ctx.moveTo(cx, cy - r - 3);
      ctx.lineTo(cx, cy - 2);
      ctx.moveTo(cx, cy + 2);
      ctx.lineTo(cx, cy + r + 3);
      ctx.stroke();
      ctx.restore();

      ctx.font = '11px ui-monospace, SFMono-Regular, Menlo, monospace';
      ctx.fillStyle = 'rgba(220, 236, 247, 0.78)';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(t.kind || '').replace(/_/g, ' '), cx + r + 7, cy);
    });
  }

  // The angular span the reactive controller picked out of the scan. Drawn
  // from the LIDAR's own origin, which is where those bearings were
  // measured -- not from base_link, 0.33m behind it.
  function drawIntentWedge(intent, project) {
    const w = intent.wedge;
    if (!w) return;
    const steps = 24;
    ctx.save();
    ctx.beginPath();
    const [ox, oy] = project(w.x, w.y);
    ctx.moveTo(ox, oy);
    for (let i = 0; i <= steps; i++) {
      const a = w.a0 + (w.a1 - w.a0) * (i / steps);
      const [px, py] = project(w.x + w.r * Math.cos(a), w.y + w.r * Math.sin(a));
      ctx.lineTo(px, py);
    }
    ctx.closePath();
    ctx.fillStyle = 'rgba(61, 220, 255, 0.10)';
    ctx.fill();
    ctx.restore();
  }

  // ---------------------------------------------------------------------
  // Decision panel: why the car is doing what it is doing
  // ---------------------------------------------------------------------
  function updateIntentPanel() {
    const intent = state.intent;
    if (!intent) {
      intentState.textContent = 'no intent yet';
      intentState.className = 'intent-chip';
      intentNode.textContent = '';
      intentReason.textContent =
        'no driving node is publishing /drive_intent -- start gap_follow or pure_pursuit';
      intentSpeeds.textContent = '--';
      intentSteering.textContent = '--';
      intentFactors.innerHTML = '';
      setDot(intentDot, null);
      return;
    }

    const stale = intentAgeMs() > INTENT_STALE_MS;
    intentState.textContent = intent.state.replace(/_/g, ' ');
    intentState.className = `intent-chip intent-${intent.severity}`;
    intentNode.textContent = stale ? `${intent.node} (stale)` : intent.node;
    intentReason.textContent = state.intentReason || 'waiting for the next explanation...';

    const dv = intent.desired_speed;
    const cv = intent.commanded_speed;
    // Showing both, always, rather than only when they differ: the gap
    // between "asked for" and "sent" is the shaping, and someone hunting a
    // sluggish car needs to see it is zero as much as they need to see it
    // is large.
    intentSpeeds.textContent = `${cv.toFixed(2)} / want ${dv.toFixed(2)} m/s`;
    const ds = (intent.desired_steering * 180 / Math.PI).toFixed(1);
    const cs = (intent.commanded_steering * 180 / Math.PI).toFixed(1);
    intentSteering.textContent = `${cs} / want ${ds} deg`;

    intentFactors.innerHTML = '';
    (intent.factors || []).forEach((f) => {
      const row = document.createElement('div');
      row.className = 'row intent-factor' + (f.binding ? ' intent-binding' : '');
      const label = document.createElement('span');
      label.className = 'row-label';
      label.textContent = f.name;
      const value = document.createElement('span');
      value.className = 'row-value metric-value';
      value.textContent = `${Number(f.value).toFixed(2)} ${f.unit || ''}`.trim();
      row.appendChild(label);
      row.appendChild(value);
      intentFactors.appendChild(row);
    });

    intentLog.innerHTML = '';
    state.intentLog.forEach((entry) => {
      const line = document.createElement('div');
      line.className = `intent-log-line intent-${entry.severity}`;
      const held = entry.heldMs == null ? 'now' : `${(entry.heldMs / 1000).toFixed(1)}s`;
      const at = new Date(entry.at);
      const clock = `${String(at.getHours()).padStart(2, '0')}:` +
        `${String(at.getMinutes()).padStart(2, '0')}:` +
        `${String(at.getSeconds()).padStart(2, '0')}`;
      line.textContent = `${clock}  ${entry.state.replace(/_/g, ' ')}  (${held})`;
      intentLog.appendChild(line);
    });

    setDot(intentDot, { receivedAt: state.intentReceivedAt }, INTENT_STALE_MS);
  }

  function drawCarMapRelative() {
    const [cx, cy] = worldToCanvas(state.pose.x, state.pose.y);
    // Canvas angle = -yaw: world yaw is measured counterclockwise, but
    // canvas rotation is clockwise once Y has been flipped -- negating
    // here keeps the icon pointing the same visual direction the car is
    // actually facing.
    drawCarIcon(cx, cy, -state.pose.yaw, drawnSteeringAngle());
  }

  function drawCarRobotCentric() {
    // base_link -- the rear axle -- at the canvas centre until the user
    // pans. The icon's origin is the rear axle too, so this is the same
    // point in both frames rather than "somewhere in the middle of a car".
    const [cx, cy] = bodyToCanvas(0, 0);
    // drawCarIcon's un-rotated "front" points along local +X (canvas
    // right, see the comment on drawCarIcon) -- but bodyToCanvas renders
    // forward (bx) as canvas "up", not "right". -PI/2 rotates the icon to
    // actually point up, matching where the scan/blind-spot are drawn;
    // passing 0 here previously left the icon facing sideways while the
    // blind-spot wedge (correctly) rendered behind it.
    drawCarIcon(cx, cy, -Math.PI / 2, drawnSteeringAngle());
  }

  // The steering angle the front wheels are drawn at: the last commanded
  // one, and only while it is fresh. A stale /drive is a command nobody is
  // sending any more, and leaving the wheels cocked over from it would
  // show a turn the car is not being asked to make.
  function drawnSteeringAngle() {
    if (!state.drive || isStale(state.drive)) return 0;
    const angle = state.drive.steeringAngle;
    return Number.isFinite(angle) ? angle : 0;
  }

  // ---------------------------------------------------------------------
  // The car, to scale: a 0.36 m wheelbase, 0.30 m over the tires, LIDAR
  // 0.26 m ahead of the rear axle (see CAR_MODEL). Front along local +X
  // before rotation, so angle 0 = facing canvas right.
  //
  // Local canvas axes, worked out once here because getting it wrong is
  // silent: both callers translate to base_link and then rotate, and in
  // both the resulting frame has local +X = body forward and local +Y =
  // body RIGHT (canvas Y grows downward, body Y grows to the left). So
  // every body-frame y is negated on the way in, and a body-frame
  // counterclockwise steer becomes a clockwise canvas rotation.
  // ---------------------------------------------------------------------
  function drawCarIcon(cx, cy, angle, steering = 0) {
    const car = carModelGeometry(steering);
    const lengthM = car.noseX - car.tailX;
    // Pixels per meter. Exactly the view scale, so the icon really is the
    // size of the car against the map -- except when zoomed so far out
    // that the car would be a couple of pixels, where a floor takes over
    // and the icon is knowingly bigger than life rather than invisible.
    const m = Math.max(view.scale, CAR_MIN_LENGTH_PX / lengthM);
    const X = (x) => x * m;
    const Y = (y) => -y * m;

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(angle);

    // Body outline. Its width is the MEASURED 0.30 m over the tires --
    // the true widest part of the car -- so the silhouette on the map is
    // the footprint, not a narrower styling guess with the tires hanging
    // out of it.
    const r = Math.min(X(0.05), X(car.halfTrack) * 0.6);
    const tail = X(car.tailX);
    const nose = X(car.noseX);
    const half = X(car.halfTrack);
    ctx.beginPath();
    ctx.moveTo(tail + r, -half);
    ctx.lineTo(nose - r, -half);
    ctx.quadraticCurveTo(nose, -half, nose, -half + r);
    ctx.lineTo(nose, half - r);
    ctx.quadraticCurveTo(nose, half, nose - r, half);
    ctx.lineTo(tail + r, half);
    ctx.quadraticCurveTo(tail, half, tail, half - r);
    ctx.lineTo(tail, -half + r);
    ctx.quadraticCurveTo(tail, -half, tail + r, -half);
    ctx.closePath();
    // Cyan, not red: on this palette red means "the car has decided to
    // stop", and a permanently red car icon competes with that. Cyan is
    // the accent that means "this mark is the system talking".
    ctx.fillStyle = HUD.accent;
    ctx.shadowColor = 'rgba(61, 220, 255, 0.85)';
    ctx.shadowBlur = Math.max(3, m * 0.08);
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.strokeStyle = '#eafaff';
    ctx.lineWidth = Math.max(1, m * 0.006);
    ctx.stroke();

    // Windshield-ish band near the front -- the one cue that survives at
    // the zoom where the whole car is 24 px long and reads "which end is
    // the front" without having to find the LIDAR puck.
    ctx.fillStyle = 'rgba(4, 10, 18, 0.65)';
    ctx.fillRect(X(car.noseX - 0.10), -half * 0.7, Math.max(1, X(0.045)), half * 1.4);

    // Wheels last of the body parts, and dark: they are the widest thing
    // on the car, they sit exactly ON the outline rather than inside it,
    // and a steered front tire is supposed to be seen poking out past it.
    ctx.fillStyle = '#0a1119';
    for (const w of car.wheels) {
      ctx.save();
      ctx.translate(X(w.x), Y(w.y));
      ctx.rotate(-w.angle); // body-frame CCW -> canvas CW, see above
      ctx.fillRect(-X(w.length) / 2, -X(w.width) / 2,
                   Math.max(1, X(w.length)), Math.max(1, X(w.width)));
      ctx.restore();
    }

    // Where the beams actually come from. Worth its own mark: the LIDAR is
    // 0.26 m ahead of the rear axle the pose is reported at, so the scan
    // radiates from a point 72% of the way up the car, not from its
    // middle and not from the dot the pose puts on the map.
    const lidarR = Math.max(1.5, X(CAR_MODEL.lidarRadiusM));
    ctx.beginPath();
    ctx.arc(X(car.lidarX), 0, lidarR, 0, 2 * Math.PI);
    ctx.fillStyle = '#04121b';
    ctx.fill();
    ctx.strokeStyle = 'rgba(234, 250, 255, 0.9)';
    ctx.lineWidth = Math.max(1, m * 0.004);
    ctx.stroke();

    // Steering clearance: how far the turned front tire reaches out past
    // the parked footprint, per side, straight off the measured wheelbase
    // and track (carModelGeometry). Only drawn while it is actually
    // wider than the car -- so at rest there is nothing extra on screen,
    // and mid-corner there is a mark showing the room the front end needs
    // that the body outline alone does not ask for.
    const overhangs = [
      [1, car.clearanceLeft],
      [-1, car.clearanceRight],
    ].filter(([, reach]) => reach > car.halfTrack + 0.002);
    if (overhangs.length) {
      ctx.save();
      ctx.strokeStyle = 'rgba(61, 220, 255, 0.55)';
      ctx.lineWidth = Math.max(1, m * 0.004);
      ctx.setLineDash([Math.max(2, X(0.03)), Math.max(2, X(0.02))]);
      for (const [side, reach] of overhangs) {
        ctx.beginPath();
        ctx.moveTo(X(car.frontAxleX - 0.12), Y(side * reach));
        ctx.lineTo(X(car.frontAxleX + 0.12), Y(side * reach));
        ctx.stroke();
      }
      ctx.restore();
    }

    ctx.restore();
  }

  // ---------------------------------------------------------------------
  // Status text + staleness. Age is recomputed on a fixed timer (below),
  // not just whenever a message happens to arrive, specifically so a
  // *frozen* feed is visibly reported as stale instead of silently
  // leaving the last good value on screen forever.
  // ---------------------------------------------------------------------
  const STALE_AFTER_MS = 1000;
  const STALE_COLOR = HUD.bad;

  function ageText(entry) {
    if (!entry) return 'never';
    const ageMs = performance.now() - entry.receivedAt;
    return ageMs < 1000 ? `${Math.round(ageMs)}ms ago` : `${(ageMs / 1000).toFixed(1)}s ago`;
  }

  function isStale(entry, thresholdMs = STALE_AFTER_MS) {
    return !entry || (performance.now() - entry.receivedAt) > thresholdMs;
  }

  function formatUptime(totalSeconds) {
    const s = Math.max(0, Math.round(totalSeconds));
    const h = Math.floor(s / 3600);
    const m = Math.floor((s % 3600) / 60);
    return `${h}h${String(m).padStart(2, '0')}m`;
  }

  function stopwatchElapsed() {
    if (!state.stopwatch) return 0;
    const interpolation = state.stopwatch.running
      ? Math.min((performance.now() - state.stopwatch.receivedAt) / 1000, 0.5)
      : 0;
    return state.stopwatch.elapsedS + interpolation;
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

  function updateVehicleAndStopwatch() {
    vehicleSpeed.textContent = state.speed && !isStale(state.speed)
      ? `${state.speed.speed.toFixed(2)} m/s`
      : '-- m/s';
    vehicleSteering.textContent = state.drive && !isStale(state.drive)
      ? `${(state.drive.steeringAngle * 180 / Math.PI).toFixed(1)} deg`
      : '-- deg';

    const timer = state.stopwatch;
    stopwatchDisplay.textContent = formatStopwatch(stopwatchElapsed());
    stopwatchDisplay.classList.toggle('running', !!(timer && timer.running));
    stopwatchToggle.textContent = timer && timer.enabled ? 'disable' : 'enable';
    stopwatchToggle.classList.toggle('enabled', !!(timer && timer.enabled));

    vehicleLb.className = 'row-value';
    stopwatchState.className = '';
    if (!timer || isStale(timer) || !timer.joyFresh) {
      vehicleLb.textContent = 'joystick stale / offline';
      stopwatchState.textContent = timer && timer.enabled
        ? 'paused · waiting for live joystick'
        : 'off · enable, then hold LB to run';
      stopwatchState.classList.add('error');
    } else if (!timer.buttonAvailable) {
      vehicleLb.textContent = 'LB button unavailable';
      stopwatchState.textContent = 'paused · LB input unavailable';
      stopwatchState.classList.add('error');
    } else if (timer.lbHeld) {
      vehicleLb.textContent = 'HELD · deadman active';
      stopwatchState.textContent = timer.enabled ? 'running · release LB to pause' : 'ready · stopwatch disabled';
      stopwatchState.classList.add('ready');
    } else {
      vehicleLb.textContent = 'released';
      stopwatchState.textContent = timer.enabled ? 'paused · hold LB to run' : 'off · enable, then hold LB to run';
      if (timer.enabled) stopwatchState.classList.add('warning');
    }
  }

  // Each feed's status dot: gray (nothing ever received), green (fresh),
  // or red (stale) -- a glance at four dots is faster to read than four
  // separate "updated Xs ago" strings.
  function setDot(el, entry, thresholdMs = STALE_AFTER_MS) {
    el.className = 'dot ' + (!entry ? 'dot-gray' : isStale(entry, thresholdMs) ? 'dot-red' : 'dot-green');
  }

  // Signal-bar thresholds follow the same rough dBm bands phones/laptops
  // use for their own WiFi icons (-50 excellent ... -80 unusable).
  function wifiBarCount(dbm) {
    if (dbm >= -55) return 4;
    if (dbm >= -65) return 3;
    if (dbm >= -75) return 2;
    if (dbm >= -85) return 1;
    return 0;
  }

  function updateWifiBars(dbm) {
    const count = dbm == null ? 0 : wifiBarCount(dbm);
    const tierClass = count >= 3 ? '' : count === 2 ? 'weak' : 'bad';
    wifiBarEls.forEach((bar, i) => {
      bar.className = 'wifi-bar' + (i < count ? ` active ${tierClass}` : '');
    });
  }

  // ---------------------------------------------------------------------
  // The three system dials.
  //
  // The ring is a second encoding of the number printed inside it, never a
  // replacement for it: the text element is still written exactly as it
  // always was, and everything here is additive. If any of this were to
  // throw, the numbers would still be right.
  //
  // Nothing in here touches geometry. `pathLength="100"` in the markup
  // normalises the arc's dash units to percent, so the only thing that
  // moves is stroke-dashoffset -- a paint-only property. A stats packet
  // cannot reflow the sidebar (rule 1 at the top of style.css).
  // ---------------------------------------------------------------------

  // Temperature has no natural 0-100% range, so its ring maps the band
  // that actually matters on this board: roughly ambient at the bottom,
  // thermal throttling at the top.
  const TEMP_MIN_C = 20;
  const TEMP_MAX_C = 100;
  const tempFraction = (c) => (c - TEMP_MIN_C) / (TEMP_MAX_C - TEMP_MIN_C);

  /**
   * Point one ring at `fraction` of full scale, or empty it when
   * `fraction` is null (no reading -- an unreadable thermal zone, or no
   * stats at all). `warn`/`bad` are the fractions at which the ring stops
   * being cyan; colour on this page means state, so they are the same
   * thresholds a person would use to decide the car needs attention.
   */
  function setGauge(arc, fraction, warn, bad) {
    if (!arc) return;
    if (fraction == null || !Number.isFinite(fraction)) {
      arc.style.strokeDashoffset = '100';
      arc.classList.remove('gauge-warn', 'gauge-bad');
      return;
    }
    const clamped = Math.max(0, Math.min(1, fraction));
    arc.style.strokeDashoffset = String(100 - clamped * 100);
    arc.classList.toggle('gauge-warn', clamped >= warn && clamped < bad);
    arc.classList.toggle('gauge-bad', clamped >= bad);
  }

  // Two values per row now, so each one gets the short form and keeps the
  // full detail in its tooltip. The freshness dot beside it already says
  // what "12ms ago" said, which is what made room for a second column.
  function setMetric(element, text, detail) {
    element.textContent = text;
    const holder = element.parentElement || element;
    holder.title = detail || '';
  }

  function updateStatusText() {
    setMetric(infoMap,
      state.map ? `${state.map.width}x${state.map.height}` : '--',
      state.map
        ? `${state.map.width}x${state.map.height} cells @ `
          + `${state.map.resolution.toFixed(3)}m/cell, updated ${ageText(state.map)}`
        : 'no map yet');
    setMetric(infoScan,
      state.scan ? `${state.scan.ranges.length} pts` : '--',
      state.scan ? `${state.scan.ranges.length} beams, updated ${ageText(state.scan)}`
                 : 'no scan yet');
    setMetric(infoPose,
      state.pose ? `${state.pose.x.toFixed(1)}, ${state.pose.y.toFixed(1)}` : '--',
      state.pose
        ? `${state.pose.x.toFixed(2)}, ${state.pose.y.toFixed(2)}m @ `
          + `${(state.pose.yaw * 180 / Math.PI).toFixed(0)}deg, updated ${ageText(state.pose)}`
        : 'no pose yet');
    setMetric(infoDrive,
      state.drive ? `${state.drive.speed.toFixed(1)} m/s` : '--',
      state.drive
        ? `${state.drive.speed.toFixed(2)}m/s @ `
          + `${(state.drive.steeringAngle * 180 / Math.PI).toFixed(1)}deg, `
          + `updated ${ageText(state.drive)}`
        : 'no command yet');
    updateVehicleAndStopwatch();

    if (state.stats) {
      infoCpu.textContent = `${state.stats.cpuPercent.toFixed(0)}%`;
      infoMem.textContent = `${state.stats.memPercent.toFixed(0)}%`;
      infoTemp.textContent = state.stats.cpuTempC != null ? `${state.stats.cpuTempC.toFixed(0)}C` : 'n/a';
      infoWifiText.textContent = state.stats.wifiDbm != null ? `${state.stats.wifiDbm.toFixed(0)}dBm` : 'n/a';
      infoUptime.textContent = formatUptime(state.stats.uptimeS);
      updateWifiBars(state.stats.wifiDbm);
      setGauge(arcCpu, state.stats.cpuPercent / 100, 0.75, 0.90);
      setGauge(arcMem, state.stats.memPercent / 100, 0.80, 0.92);
      setGauge(arcTemp,
        state.stats.cpuTempC == null ? null : tempFraction(state.stats.cpuTempC),
        tempFraction(70), tempFraction(85));
    } else {
      infoCpu.textContent = infoMem.textContent = infoTemp.textContent = infoUptime.textContent = '--';
      infoWifiText.textContent = '--';
      updateWifiBars(null);
      setGauge(arcCpu, null);
      setGauge(arcMem, null);
      setGauge(arcTemp, null);
    }

    updateIntentPanel();

    setDot(dots.map, state.map);
    setDot(dots.scan, state.scan);
    setDot(dots.pose, state.pose);
    setDot(dots.drive, state.drive);
    // Stats only tick once per stats_interval_sec (default 1Hz) -- the
    // shared 1s STALE_AFTER_MS would flicker red between every tick, so
    // this row gets a longer threshold (a few sample periods of slack).
    setDot(dots.stats, state.stats, 3000);

    updateDigests();
  }

  // Re-render periodically even with no new messages, purely so the
  // "updated Xs ago" readout and stale-data coloring stay live.
  setInterval(() => { if (ws && ws.readyState === WebSocket.OPEN) scheduleRender(); }, 250);

  // ---------------------------------------------------------------------
  // Pan / zoom
  //
  // Two primitives, and every input goes through one of them: pan by a
  // screen delta, and zoom about a screen point. A mouse drag, a wheel, a
  // trackpad pinch and two fingers on a phone all end up here, which is
  // what keeps the one genuinely fiddly part -- the world-frame versus
  // body-frame distinction -- written down exactly once.
  // ---------------------------------------------------------------------

  /** Move the view by a delta in CSS pixels. */
  function panBy(dxCss, dyCss) {
    const dpr = window.devicePixelRatio || 1;
    const dx = dxCss * dpr;
    const dy = dyCss * dpr;
    // Update both the world-frame pan (read by worldToCanvas, once a map
    // or pose exists) and the body-frame pan (read by bodyToCanvas, before
    // then) -- render() only uses whichever is actually active, but a drag
    // can happen in either mode so both need to track it.
    view.centerX -= dx / view.scale;
    view.centerY += dy / view.scale; // canvas +Y is down, world +Y is up
    view.bodyPanY += dx / view.scale;
    view.bodyPanX += dy / view.scale;
    view.userAdjusted = true;
  }

  /**
   * Scale the view by `factor`, keeping whatever is under the screen point
   * (clientX, clientY) exactly where it is -- "zoom to the pointer", not
   * "zoom to the middle of the canvas". For a pinch, that point is the
   * midpoint between the two fingers.
   */
  function zoomAt(clientX, clientY, factor) {
    const dpr = window.devicePixelRatio || 1;
    const px = clientX * dpr;
    const py = clientY * dpr;

    if (state.pose) {
      // World point currently under the pointer, before changing scale.
      const [worldXBefore, worldYBefore] = canvasToWorld(clientX, clientY);
      view.scale = Math.min(Math.max(view.scale * factor, 2), 4000);
      view.centerX = worldXBefore - (px - canvas.width / 2) / view.scale;
      view.centerY = worldYBefore + (py - canvas.height / 2) / view.scale;
    } else {
      // Same idea in body-frame coordinates (see bodyToCanvas) -- keeping
      // this mode-aware, rather than always updating centerX/centerY,
      // avoids leaving stale world-frame values that would otherwise make
      // the view jump the instant a pose first arrives and mapRelative
      // mode switches on.
      const [bxBefore, byBefore] = canvasToBody(clientX, clientY);
      view.scale = Math.min(Math.max(view.scale * factor, 2), 4000);
      view.bodyPanY = byBefore + (px - canvas.width / 2) / view.scale;
      view.bodyPanX = bxBefore + (py - canvas.height / 2) / view.scale;
    }
    view.userAdjusted = true;
  }

  // ---------------------------------------------------------------------
  // Input, as pointer events rather than mouse events.
  //
  // This is not a tidy-up. The dashboard was mouse-and-wheel only, so on a
  // phone -- the layout with the biggest map and the least room for
  // anything else -- the map could not be panned or zoomed AT ALL. There
  // was no gesture that did anything, and no error to notice.
  //
  // One handler covers every device, because tracking the set of live
  // pointers collapses the two gestures into the same arithmetic:
  //
  //   one pointer  -- the "midpoint" is that pointer, its spread is
  //                   meaningless, so the move is a pure pan;
  //   two pointers -- the midpoint moving is a pan, the spread changing
  //                   is a zoom about that midpoint, and doing both is a
  //                   pinch.
  //
  // Scale first about the OLD midpoint, then translate by how far the
  // midpoint moved: that order is what keeps the world point under each
  // finger under that finger for the whole gesture.
  // ---------------------------------------------------------------------
  const pointers = new Map();   // pointerId -> {x, y} in CSS pixels

  function pointerMidpoint() {
    let x = 0;
    let y = 0;
    for (const p of pointers.values()) { x += p.x; y += p.y; }
    return { x: x / pointers.size, y: y / pointers.size };
  }

  /** Distance between the two live pointers; 0 for any other count. */
  function pointerSpread() {
    if (pointers.size !== 2) return 0;
    const [a, b] = Array.from(pointers.values());
    return Math.hypot(a.x - b.x, a.y - b.y);
  }

  // The one gesture that might turn out to be a tap. Tracked separately
  // from `pointers` because it has to outlive the pointer being lifted --
  // that is the moment the decision is made -- and because it remembers
  // the FURTHEST the pointer got, not where it ended. A drag out and back
  // would otherwise drop a point in the middle of a pan.
  let tapCandidate = null;

  canvas.addEventListener('pointerdown', (e) => {
    // A third finger would drag the midpoint sideways for no reason.
    if (pointers.size >= 2) return;
    e.preventDefault();     // no text selection, no native image drag
    canvas.setPointerCapture(e.pointerId);
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (pointers.size === 1) {
      tapCandidate = {
        id: e.pointerId, x: e.clientX, y: e.clientY,
        t0: performance.now(), maxTravel: 0,
        pointerType: e.pointerType, count: 1,
      };
    } else if (tapCandidate) {
      // A second finger landed. Whatever this gesture turns into, it is
      // not a tap -- and one finger lifting early must not make the other
      // one into a click.
      tapCandidate.count = pointers.size;
    }
  });

  canvas.addEventListener('pointermove', (e) => {
    if (!pointers.has(e.pointerId)) return;

    const before = pointerMidpoint();
    const spreadBefore = pointerSpread();
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    const after = pointerMidpoint();
    const spreadAfter = pointerSpread();

    if (tapCandidate && e.pointerId === tapCandidate.id) {
      tapCandidate.maxTravel = Math.max(
        tapCandidate.maxTravel,
        Math.hypot(e.clientX - tapCandidate.x, e.clientY - tapCandidate.y));
    }

    if (spreadBefore > 0 && spreadAfter > 0) {
      zoomAt(before.x, before.y, spreadAfter / spreadBefore);
    }
    // While measuring, hold the view still until the pointer has clearly
    // committed to a drag. Without this every point placed nudges the map
    // a pixel or two, which reads as the tool being imprecise.
    if (!measuringStill()) {
      panBy(after.x - before.x, after.y - before.y);
    }
    scheduleRender();
  });

  /** True while a single pointer is down, measuring, and still within the
   *  tap slop -- i.e. it may yet turn out to be a tap rather than a pan. */
  function measuringStill() {
    if (!measure.active || !Measure || !tapCandidate) return false;
    if (pointers.size !== 1) return false;
    return tapCandidate.maxTravel <= Measure.slopFor(tapCandidate.pointerType);
  }

  function endPointer(e) {
    if (!pointers.has(e.pointerId)) return;
    const candidate = (tapCandidate && tapCandidate.id === e.pointerId)
      ? tapCandidate : null;
    pointers.delete(e.pointerId);
    if (candidate) tapCandidate = null;
    // pointercancel means the browser took the gesture away (a system
    // gesture, a scroll it decided to own). That is not a tap.
    if (candidate && e.type === 'pointerup' && measure.active && Measure
        && Measure.isTap({
          maxTravelPx: candidate.maxTravel,
          durationMs: performance.now() - candidate.t0,
          pointerCount: candidate.count,
          pointerType: candidate.pointerType,
        })) {
      addMeasurePoint(e.clientX, e.clientY);
    }
    if (canvas.hasPointerCapture(e.pointerId)) canvas.releasePointerCapture(e.pointerId);
    // Lifting one finger of a pinch needs no fix-up: the next move reads
    // its "before" midpoint from the pointers still down, which is the
    // remaining finger, so the view does not jump to it.
  }

  canvas.addEventListener('pointerup', endPointer);
  canvas.addEventListener('pointercancel', endPointer);

  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    zoomAt(e.clientX, e.clientY, Math.exp(-e.deltaY * 0.001));
    scheduleRender();
  }, { passive: false });

  function resetView() {
    view.userAdjusted = false;
    view.bodyPanX = 0;
    view.bodyPanY = 0;
    maybeAutoFit();
    scheduleRender();
  }

  resetViewBtn.addEventListener('click', resetView);
  // Double-click, and on a touchscreen double-tap, does the same thing.
  // On a phone the reset button is inside a sheet that has to be dragged
  // up before it can be pressed, which is a lot of interaction to undo an
  // accidental pinch.
  canvas.addEventListener('dblclick', (e) => {
    // Two quick taps is how a point-to-point measurement gets made, and
    // the browser synthesises a dblclick from a double-tap on a touchscreen
    // too. Resetting the view out from under that would be maddening.
    // The reset button and the M key both stay available.
    if (measure.active) { e.preventDefault(); return; }
    resetView();
  });

  function sendStopwatchControl(action, enabled) {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify({ type: 'stopwatch_control', action, enabled }));
  }

  stopwatchToggle.addEventListener('click', () => {
    sendStopwatchControl('set_enabled', !(state.stopwatch && state.stopwatch.enabled));
  });
  stopwatchReset.addEventListener('click', () => sendStopwatchControl('reset'));

  window.addEventListener('resize', render);

  // ---------------------------------------------------------------------
  // Live parameter tuning.
  //
  // This is the one part of the dashboard that writes to the car, so two
  // rules shape all of it:
  //
  //   * The server is the source of truth, always. The arm switch shows
  //     what the server confirmed (`tuning_armed`), not what was clicked;
  //     a control that gets refused snaps back to the value the node
  //     actually holds. A UI that optimistically displays what you asked
  //     for is exactly how you end up believing the car is set to 2 m/s
  //     while it drives 4.
  //   * Never rebuild a control somebody is holding. The panel's DOM is
  //     built once per parameter and only *updated* afterwards, and an
  //     update skips any control that currently has focus or is mid-drag
  //     -- otherwise a 2-second refresh landing mid-slide would yank the
  //     slider out from under a thumb.
  // ---------------------------------------------------------------------
  const tuningControls = new Map(); // "node/name" -> control record
  let tuningStructure = '';         // signature of the currently built DOM

  function tuningKey(node, name) { return `${node}/${name}`; }

  function decimalsFor(step) {
    if (!step || step <= 0) return 3;
    if (step >= 1) return 0;
    return Math.min(4, Math.ceil(-Math.log10(step)));
  }

  function formatTuningValue(param, value) {
    if (value === null || value === undefined) return '--';
    if (param.kind === 'bool') return value ? 'on' : 'off';
    return Number(value).toFixed(decimalsFor(param.step));
  }

  function sendTuningControl(payload) {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify(Object.assign({ type: 'tuning_control' }, payload)));
  }

  // ---------------------------------------------------------------------
  // Stopping driving processes.
  //
  // Two-step by design: the first press arms *that one button* and starts
  // a 4-second countdown, the second actually sends. Not because stopping
  // is dangerous -- a mistaken stop is the safe direction, which is
  // exactly why this needs no server-side arm the way tuning does -- but
  // because "which pid did I just kill?" is a horrible question to have
  // to answer at trackside, and a fat-fingered scroll on a phone should
  // not silently end a run that was going fine.
  // ---------------------------------------------------------------------

  const PROC_CONFIRM_MS = 4000;
  const procPending = new Map();   // pid -> timeout id for the armed state

  function sendProcessControl(payload) {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify(Object.assign({ type: 'process_control' }, payload)));
  }


  // ---------------------------------------------------------------------
  // Measuring: the sidebar panel and the on-map readout
  // ---------------------------------------------------------------------

  function addMeasurePoint(clientX, clientY) {
    if (!Measure) return;
    const point = canvasToActive(clientX, clientY);
    // Points from two different frames can never share a chain -- see
    // canvasToActive. Starting a new one is the honest response.
    if (measure.pts.length && measure.pts[0].frame !== point.frame) {
      measure.pts = [];
    }
    const before = measure.pts.length;
    measure.pts = Measure.addPoint(
      measure.pts, point, MEASURE_MIN_SEPARATION_PX / view.scale);
    if (measure.pts.length !== before) measure.note = '';
    renderMeasurePanel();
    scheduleRender();
  }

  function setMeasureActive(active) {
    measure.active = !!active;
    if (measureEnable) measureEnable.checked = measure.active;
    document.body.classList.toggle('measuring', measure.active);
    if (measurePanel) measurePanel.hidden = !measure.active;
    canvas.style.cursor = measure.active ? 'crosshair' : '';
    renderMeasurePanel();
    scheduleRender();
  }

  function clearMeasure() {
    measure.pts = [];
    measure.note = '';
    renderMeasurePanel();
    scheduleRender();
  }

  function undoMeasure() {
    if (!Measure) return;
    measure.pts = Measure.undo(measure.pts);
    measure.note = '';
    renderMeasurePanel();
    scheduleRender();
  }

  function renderMeasurePanel() {
    if (!Measure || !measureList) return;
    const segments = Measure.segments(measure.pts);
    const total = Measure.total(measure.pts);
    const totalText = measure.pts.length > 1 ? Measure.formatDistance(total) : '--';

    if (measureTotal) {
      measureTotal.textContent = measure.pts.length > 1
        ? Measure.formatDistance(total) : '0 cm';
    }
    if (measureHint) {
      measureHint.textContent = measure.pts.length === 0
        ? 'tap the map'
        : (measure.pts.length === 1 ? 'tap again' : `${segments.length} leg(s)`);
    }
    if (dots.measure) {
      dots.measure.className = 'dot ' + (measure.active ? 'dot-green' : 'dot-gray');
    }
    setDigest('measure', measure.pts.length > 1 ? totalText
      : (measure.active ? 'measuring' : '--'));

    measureList.innerHTML = '';
    if (!segments.length) {
      const empty = document.createElement('div');
      empty.className = 'measure-empty';
      empty.textContent = measure.active
        ? 'tap two points on the map' : 'measuring is off';
      measureList.appendChild(empty);
    } else {
      segments.forEach((seg, index) => {
        const row = document.createElement('div');
        row.className = 'measure-row';
        const name = document.createElement('span');
        name.className = 'measure-leg';
        name.textContent = `leg ${index + 1}`;
        const value = document.createElement('span');
        value.className = 'measure-value';
        value.textContent = Measure.formatDistance(seg.length);
        row.appendChild(name);
        row.appendChild(value);
        measureList.appendChild(row);
      });
      const row = document.createElement('div');
      row.className = 'measure-row measure-row-total';
      const name = document.createElement('span');
      name.className = 'measure-leg';
      name.textContent = 'total';
      const value = document.createElement('span');
      value.className = 'measure-value';
      value.textContent = totalText;
      row.appendChild(name);
      row.appendChild(value);
      measureList.appendChild(row);
    }

    if (measureStatus) {
      // A robot-centric measurement is a statement about the picture, not
      // about the track, and it moves with the car. Saying so beats
      // letting someone write the number down.
      const frameNote = (measure.pts.length && measure.pts[0].frame === 'body')
        ? 'robot-centric -- these points are relative to the car and move with it. '
        : '';
      measureStatus.textContent = frameNote + measure.note;
    }
  }

  // ---------------------------------------------------------------------
  // Saved maps: list, delete, reset SLAM, clear view
  // ---------------------------------------------------------------------

  function sendMapControl(payload) {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify(Object.assign({ type: 'map_control' }, payload)));
  }

  function formatBytes(bytes) {
    const n = Number(bytes);
    if (!Number.isFinite(n) || n < 0) return '--';
    if (n >= 1024 * 1024 * 1024) return `${(n / (1024 ** 3)).toFixed(1)} GB`;
    if (n >= 1024 * 1024) return `${(n / (1024 ** 2)).toFixed(1)} MB`;
    if (n >= 1024) return `${(n / 1024).toFixed(0)} kB`;
    return `${n} B`;
  }

  /**
   * The one decision this panel makes on its own: does this run get a
   * delete control at all, and what does the row say?
   *
   * Pulled out as a pure function so browser/map_panel_test.js can assert
   * the invariant directly. The server refuses these independently -- it
   * re-scans and re-vets before removing anything -- so this is the second
   * of two locks, not the only one. It earns its place because a control
   * that is offered and then silently refused is worse than no control.
   *
   * Fails closed: a row this page cannot identify gets no delete button.
   */
  function mapRowPlan(run) {
    const known = !!(run && typeof run.id === 'string' && run.id);
    if (!known) {
      return {
        deletable: false, name: '(unrecognised entry)', meta: '',
        reason: 'unrecognised entry', contents: '', citedBy: '',
      };
    }
    const parts = [formatBytes(run.bytes)];
    if (Array.isArray(run.span_m) && run.span_m.length === 2) {
      parts.push(`${run.span_m[0].toFixed(1)} x ${run.span_m[1].toFixed(1)} m`);
    } else if (!run.has_map) {
      parts.push('no map');
    }
    return {
      deletable: run.deletable === true,
      name: run.id,
      meta: parts.join(' \u00b7 '),
      reason: run.deletable === true ? '' : (run.reason || 'not deletable'),
      contents: (run.contents || []).join(', ') || 'nothing recognised',
      citedBy: run.cited_by || '',
    };
  }

  /**
   * Is the delete button live yet?
   *
   * Exact string equality, deliberately: no trim, no case folding. What the
   * server checks is precisely what the person was asked to type, and the
   * two must not disagree about what counts as a match.
   *
   * And never enabled for a run the server already said is not deletable,
   * however perfect the typing -- the two conditions compose rather than
   * either one alone being enough.
   */
  function deleteConfirmState(typed, run) {
    const plan = mapRowPlan(run);
    if (!plan.deletable) return { enabled: false, reason: plan.reason };
    if (typed !== plan.name) {
      return { enabled: false, reason: `type ${plan.name} to confirm` };
    }
    return { enabled: true, reason: '' };
  }

  if (typeof window !== 'undefined') {
    window.__mapRowPlan = mapRowPlan;
    window.__deleteConfirmState = deleteConfirmState;
  }

  function renderSavedMaps() {
    const snapshot = state.savedMaps;
    if (!mapsSection || !mapList) return;
    if (!snapshot) return;

    mapsSection.style.display = snapshot.enabled ? '' : 'none';
    if (!snapshot.enabled) return;

    if (mapResetBlock) {
      mapResetBlock.style.display = snapshot.can_reset_slam ? '' : 'none';
    }
    if (mapDeleteBlock) {
      mapDeleteBlock.style.display = snapshot.can_delete ? '' : 'none';
    }

    const runs = snapshot.runs || [];
    if (dots.maps) {
      dots.maps.className = 'dot ' + (runs.length ? 'dot-green' : 'dot-gray');
    }
    const totalBytes = runs.reduce((sum, r) => sum + (Number(r.bytes) || 0), 0);
    setDigest('maps', runs.length
      ? `${runs.length} run(s), ${formatBytes(totalBytes)}` : '--');

    if (!snapshot.can_delete) return;

    mapList.innerHTML = '';
    if (!runs.length) {
      const empty = document.createElement('div');
      empty.className = 'map-empty';
      // "No saved runs" and "nowhere configured to look" call for
      // completely different fixes, so they must not look the same.
      empty.textContent = (snapshot.roots || []).length
        ? `no saved runs under ${(snapshot.roots || []).join(', ')}`
        : 'no map directories are configured (see map_roots)';
      mapList.appendChild(empty);
      return;
    }
    for (const run of runs) mapList.appendChild(buildMapRow(run));
  }

  function buildMapRow(run) {
    const plan = mapRowPlan(run);
    const row = document.createElement('div');
    row.className = 'map-row' + (plan.deletable ? '' : ' map-row-blocked');

    const name = document.createElement('div');
    name.className = 'map-name';
    name.textContent = plan.name;
    row.appendChild(name);

    const meta = document.createElement('div');
    meta.className = 'map-meta';
    meta.textContent = plan.meta;
    meta.title = (run && run.path) || '';
    row.appendChild(meta);

    const contents = document.createElement('div');
    contents.className = 'map-contents';
    contents.textContent = `deletes: ${plan.contents}`;
    row.appendChild(contents);

    if (plan.citedBy) {
      // A run some test reads as its oracle. Deleting it turns a real test
      // into a permanent skip, which is worth knowing before, not after.
      const cited = document.createElement('div');
      cited.className = 'map-cited';
      cited.textContent = `used as a test oracle by ${plan.citedBy}`;
      row.appendChild(cited);
    }

    if (!plan.deletable) {
      const why = document.createElement('div');
      why.className = 'map-reason';
      why.textContent = plan.reason;
      row.appendChild(why);
      return row;
    }

    const confirm = document.createElement('div');
    confirm.className = 'map-confirm';
    const field = document.createElement('input');
    field.type = 'text';
    field.className = 'map-confirm-input';
    field.setAttribute('autocomplete', 'off');
    field.setAttribute('spellcheck', 'false');
    field.placeholder = `type ${plan.name}`;
    field.setAttribute('aria-label', `type ${plan.name} to confirm deleting it`);
    const button = document.createElement('button');
    button.className = 'map-delete';
    button.textContent = 'delete';
    button.disabled = true;

    const sync = () => {
      const verdict = deleteConfirmState(field.value, run);
      button.disabled = !verdict.enabled;
      button.title = verdict.reason;
    };
    field.addEventListener('input', sync);
    button.addEventListener('click', () => {
      if (!deleteConfirmState(field.value, run).enabled) return;
      setMapStatus(`deleting ${plan.name}...`);
      sendMapControl({
        action: 'delete', id: run.id, confirm: field.value,
        // The digest the server published with this listing. If the run
        // changed since -- a map finished writing, say -- the delete is
        // refused rather than removing something never shown here.
        digest: run.digest,
      });
      button.disabled = true;
    });
    sync();

    confirm.appendChild(field);
    confirm.appendChild(button);
    row.appendChild(confirm);
    return row;
  }

  function setMapStatus(text, kind) {
    if (!mapStatus) return;
    mapStatus.textContent = text;
    mapStatus.className = 'map-status' + (kind ? ` map-status-${kind}` : '');
  }

  function applyMapDeleteResult(message) {
    if (message.ok) {
      setMapStatus(
        `${message.id}: deleted, ${formatBytes(message.freed_bytes)} freed`,
        'good');
    } else {
      setMapStatus(`${message.id || 'delete'}: ${message.detail}`, 'bad');
    }
  }

  function applySlamResetResult(message) {
    if (!message.done) {
      setMapStatus(message.detail, '');
      return;
    }
    setMapStatus(message.detail, message.ok ? 'good' : 'bad');
  }

  function applyMapCleared(message) {
    // Drop our copy. applyMapPatch already returns early on a null map
    // ("no keyframe yet; the next one brings everything"), so patches
    // arriving before the next keyframe are ignored rather than painted
    // onto nothing.
    state.map = null;
    if (message.has_map) {
      setMapStatus('view cleared -- the car is still publishing this map, '
                   + 'so it came straight back', '');
    } else {
      setMapStatus('view cleared -- nothing is publishing /map right now', '');
    }
    scheduleRender();
  }
  function setProcStatus(text, ok) {
    const element = document.getElementById('proc-status');
    if (!element) return;
    element.textContent = text || '';
    element.classList.toggle('proc-status-bad', ok === false);
    element.classList.toggle('proc-status-good', ok === true);
  }

  function applyProcessResult(header) {
    const label = header.name || `pid ${header.pid}`;
    const escalation = (header.sent || []).length > 1
      ? ` (needed ${header.sent.join(' -> ')})`
      : '';
    if (!header.done) {
      setProcStatus(`${label}: ${header.detail}`);
      return;
    }
    setProcStatus(
      header.ok ? `${label}: ${header.detail}${escalation}`
                : `${label}: ${header.detail}`,
      header.ok);
    // A stop that needed more than a SIGINT is a bug in that node worth
    // seeing in the console too, not just in a line that scrolls away.
    if (header.ok && (header.sent || []).length > 1) {
      console.warn(`[racerbot] ${label} ignored SIGINT; escalated via `
                   + `${header.sent.join(' -> ')}`);
    }
  }

  function renderProcesses() {
    const listEl = document.getElementById('proc-list');
    const sectionEl = document.getElementById('processes-section');
    if (!listEl) return;
    const info = state.processes;

    if (!info || !info.enabled) {
      if (sectionEl) sectionEl.hidden = true;
      setDigest('processes', '--');
      return;
    }
    if (sectionEl) sectionEl.hidden = false;

    const targets = info.targets || [];
    const stoppable = targets.filter((t) => !t.protected);
    setDigest('processes', targets.length
      ? `${stoppable.length}/${targets.length} stoppable`
      : 'none running');
    // Cyan-family green means "something is running", grey means nothing
    // is. Deliberately never red: a driving node being up is not a fault,
    // and red on this page is reserved for what the car has decided.
    const dot = document.getElementById('dot-processes');
    if (dot) dot.className = 'dot ' + (targets.length ? 'dot-green' : 'dot-gray');

    if (!targets.length) {
      listEl.innerHTML = '';
      const empty = document.createElement('div');
      empty.className = 'proc-empty';
      empty.textContent = 'no driving processes running';
      listEl.appendChild(empty);
      return;
    }

    listEl.innerHTML = '';
    for (const target of targets) {
      listEl.appendChild(buildProcRow(target));
    }
  }

  // The one decision in this panel worth testing away from the DOM: does
  // this row get a stop control at all? Pulled out as a pure function so
  // browser/proc_panel_test.js can assert the invariant directly --
  // a protected target must never produce a button, however the server
  // labelled it. The server refuses those pids anyway (it re-scans and
  // re-vets before signalling), so this is the second of two locks, not
  // the only one.
  function procRowPlan(target) {
    // Fails closed. A target with no pid, or no target at all, is
    // something this page does not understand, and the answer to "should
    // I offer to kill a process I cannot identify" is no.
    const known = !!(target && Number.isFinite(Number(target.pid)));
    const protectedRow = !known || !!target.protected;
    return {
      stoppable: !protectedRow,
      name: (target && target.name) || '(unknown)',
      meta: known ? `pid ${target.pid}` : 'no pid',
      reason: protectedRow
        ? ((target && target.reason) || (known ? 'protected' : 'unrecognised entry'))
        : '',
    };
  }

  function buildProcRow(target) {
    const plan = procRowPlan(target);
    const row = document.createElement('div');
    row.className = 'proc-row' + (plan.stoppable ? '' : ' proc-row-protected');

    const name = document.createElement('div');
    name.className = 'proc-name';
    name.textContent = plan.name;
    const kind = document.createElement('span');
    kind.className = 'proc-kind';
    kind.textContent = target.kind === 'launch' ? 'launch' : 'node';
    name.appendChild(kind);

    const meta = document.createElement('div');
    meta.className = 'proc-meta';
    meta.textContent = plan.meta;
    // The full cmdline only as a tooltip: it is how you tell two copies of
    // the same node apart (different worktrees, different params), but it
    // is far too long to sit in a phone-width sidebar.
    meta.title = target.cmdline || '';

    row.appendChild(name);
    row.appendChild(meta);

    if (!plan.stoppable) {
      const why = document.createElement('div');
      why.className = 'proc-reason';
      why.textContent = plan.reason;
      row.appendChild(why);
      return row;
    }

    const button = document.createElement('button');
    button.className = 'proc-stop';
    button.textContent = 'stop';
    button.addEventListener('click', () => onStopClicked(target, button));
    row.appendChild(button);
    return row;
  }

  if (typeof window !== 'undefined') window.__procRowPlan = procRowPlan;

  function onStopClicked(target, button) {
    if (procPending.has(target.pid)) {
      clearTimeout(procPending.get(target.pid));
      procPending.delete(target.pid);
      button.classList.remove('proc-stop-armed');
      button.textContent = 'stop';
      setProcStatus(`stopping ${target.name}...`);
      sendProcessControl({ action: 'stop', pid: target.pid });
      return;
    }
    button.classList.add('proc-stop-armed');
    button.textContent = 'confirm?';
    setProcStatus(`press again to stop ${target.name} (pid ${target.pid})`);
    procPending.set(target.pid, setTimeout(() => {
      procPending.delete(target.pid);
      button.classList.remove('proc-stop-armed');
      button.textContent = 'stop';
      setProcStatus('');
    }, PROC_CONFIRM_MS));
  }

  function sendTuningSet(node, name, value) {
    sendTuningControl({ action: 'set', node, name, value });
  }

  // A signature of the panel's *shape* only -- which nodes are up and
  // which parameters they offer. Values deliberately excluded, so the
  // 2-second value refresh never triggers a rebuild.
  function structureSignature(nodes) {
    return JSON.stringify(nodes.map((n) => [
      // `driving` is in the signature deliberately: a handover from
      // gap_follow to pure_pursuit changes no parameter, so without this
      // the panel would keep the old node badged as the one driving.
      n.node, n.online, n.error || '', n.savable, !!n.driving,
      (n.params || []).map((p) => p.name),
    ]));
  }

  function renderTuning() {
    const tuning = state.tuning;
    if (!tuning) return;

    tuningSection.style.display = tuning.enabled ? '' : 'none';
    if (!tuning.enabled) {
      tuningPanel.classList.remove('open');
      return;
    }

    const nodes = tuning.nodes || [];
    const onlineNodes = nodes.filter((n) => n.online);
    tuningDot.className = 'dot ' + (onlineNodes.length ? 'dot-green' : 'dot-gray');
    const drivingNode = onlineNodes.find((n) => n.driving);
    tuningSummary.textContent = onlineNodes.length
      ? (drivingNode
        ? `${drivingNode.node.replace(/_node$/, '')} driving`
        : `${onlineNodes.map((n) => n.node.replace(/_node$/, '')).join(', ')} tunable`)
      : 'no driving node running';

    const signature = structureSignature(nodes);
    if (signature !== tuningStructure) {
      tuningStructure = signature;
      buildTuningPanel(nodes);
    }
    updateTuningValues(nodes);
    updateTuningEnabled();
  }

  function buildTuningPanel(nodes) {
    tuningControls.clear();
    tuningBody.textContent = '';

    const usable = nodes.filter((n) => n.online);
    if (!usable.length) {
      const empty = document.createElement('div');
      empty.className = 'tuning-empty';
      empty.textContent =
        'No driving node is running. Start gap_follow or pure_pursuit and '
        + 'its knobs will appear here.';
      tuningBody.appendChild(empty);
      return;
    }

    // With an auto_map_race supervisor up, both controllers are online and
    // tunable at once but only one of them is actually driving. Sort the
    // driving one to the top and badge it, so the panel cannot be mistaken
    // for "these knobs all reach the car" -- they do not, and on
    // 2026-08-19 a whole run was tuned through the idle one.
    const anyDriving = usable.some((n) => n.driving);
    const ordered = anyDriving
      ? usable.slice().sort((a, b) => (b.driving ? 1 : 0) - (a.driving ? 1 : 0))
      : usable;

    ordered.forEach((node) => {
      const title = document.createElement('div');
      title.className = 'tuning-node-title'
        + (anyDriving ? (node.driving ? ' driving' : ' idle') : '');
      title.textContent = node.node;
      if (anyDriving) {
        const badge = document.createElement('span');
        badge.className = 'tuning-node-badge'
          + (node.driving ? ' driving' : ' idle');
        badge.textContent = node.driving ? 'driving' : 'not driving';
        badge.title = node.driving
          ? 'The supervisor is forwarding this node\'s commands to the car. '
            + 'These knobs take effect now.'
          : 'This node is running and tunable, but the supervisor is not '
            + 'forwarding its commands. Changing these knobs will not change '
            + 'how the car is driving right now.';
        title.appendChild(badge);
      }
      tuningBody.appendChild(title);

      if (node.error) {
        const error = document.createElement('div');
        error.className = 'tuning-node-error';
        error.textContent = node.error;
        tuningBody.appendChild(error);
        return;
      }

      let currentGroup = null;
      (node.params || []).forEach((param) => {
        if (param.group !== currentGroup) {
          currentGroup = param.group;
          const groupTitle = document.createElement('div');
          groupTitle.className = 'tuning-group-title' + (param.safety ? ' safety' : '');
          groupTitle.textContent = param.group;
          tuningBody.appendChild(groupTitle);
        }
        tuningBody.appendChild(buildTuningParam(node.node, param));
      });
    });
  }

  function buildTuningParam(nodeName, param) {
    const row = document.createElement('div');
    row.className = 'tuning-param' + (param.safety ? ' safety' : '');

    const head = document.createElement('div');
    head.className = 'tuning-param-head';
    const label = document.createElement('span');
    label.className = 'tuning-param-name';
    label.textContent = param.label;
    head.appendChild(label);

    const valueWrap = document.createElement('span');
    valueWrap.className = 'tuning-param-value';

    const revert = document.createElement('button');
    revert.className = 'tuning-revert';
    revert.title = 'Revert to the value this node started with';
    revert.textContent = '↺';
    valueWrap.appendChild(revert);

    const record = { param, node: nodeName, row, revert, sliding: false };

    if (param.kind === 'bool') {
      const toggle = document.createElement('input');
      toggle.type = 'checkbox';
      toggle.addEventListener('change', () => {
        sendTuningSet(nodeName, param.name, toggle.checked);
      });
      valueWrap.appendChild(toggle);
      record.toggle = toggle;
      head.appendChild(valueWrap);
      row.appendChild(head);
    } else {
      const number = document.createElement('input');
      number.type = 'number';
      number.min = param.min;
      number.max = param.max;
      number.step = param.step || 'any';
      valueWrap.appendChild(number);

      const unit = document.createElement('span');
      unit.className = 'tuning-unit';
      unit.textContent = param.unit || '';
      valueWrap.appendChild(unit);
      head.appendChild(valueWrap);
      row.appendChild(head);

      const slider = document.createElement('input');
      slider.type = 'range';
      slider.min = param.min;
      slider.max = param.max;
      slider.step = param.step || (param.max - param.min) / 200;
      row.appendChild(slider);

      // Drag: mirror into the number box continuously so the readout
      // tracks your thumb, but only send on release. Sending every
      // intermediate value would put a whole sweep of speeds on the bus
      // for one adjustment.
      slider.addEventListener('pointerdown', () => { record.sliding = true; });
      slider.addEventListener('input', () => {
        number.value = Number(slider.value).toFixed(decimalsFor(param.step));
      });
      slider.addEventListener('change', () => {
        record.sliding = false;
        sendTuningSet(nodeName, param.name, Number(slider.value));
      });
      slider.addEventListener('pointerup', () => { record.sliding = false; });

      number.addEventListener('change', () => {
        const clamped = Math.min(Math.max(Number(number.value), param.min), param.max);
        if (!Number.isFinite(clamped)) return;
        number.value = clamped.toFixed(decimalsFor(param.step));
        slider.value = clamped;
        sendTuningSet(nodeName, param.name, clamped);
      });

      record.number = number;
      record.slider = slider;
    }

    revert.addEventListener('click', () => {
      if (record.baseline === null || record.baseline === undefined) return;
      sendTuningSet(nodeName, param.name, record.baseline);
    });

    if (param.description) {
      const desc = document.createElement('div');
      desc.className = 'tuning-desc';
      desc.textContent = param.description;
      row.appendChild(desc);
    }

    const note = document.createElement('div');
    note.className = 'tuning-note';
    row.appendChild(note);
    record.note = note;

    tuningControls.set(tuningKey(nodeName, param.name), record);
    return row;
  }

  function updateTuningValues(nodes) {
    nodes.forEach((node) => {
      (node.params || []).forEach((param) => {
        const record = tuningControls.get(tuningKey(node.node, param.name));
        if (!record) return;
        record.baseline = param.baseline;
        setTuningControlValue(record, param.value);
      });
    });
  }

  function setTuningControlValue(record, value) {
    if (value === null || value === undefined) return;
    const { param } = record;
    if (record.toggle) {
      if (document.activeElement !== record.toggle) record.toggle.checked = !!value;
    } else {
      // Skip anything the user is actively holding -- see the section
      // comment. Their input wins until they let go.
      if (!record.sliding && document.activeElement !== record.slider) {
        record.slider.value = value;
      }
      if (!record.sliding && document.activeElement !== record.number) {
        record.number.value = Number(value).toFixed(decimalsFor(param.step));
      }
    }
    const baseline = record.baseline;
    const dirty = baseline !== null && baseline !== undefined
      && (param.kind === 'bool'
        ? !!baseline !== !!value
        : Math.abs(Number(baseline) - Number(value)) > 1e-9);
    record.row.classList.toggle('dirty', dirty);
    record.revert.title = `Revert to ${formatTuningValue(param, baseline)}`;
  }

  function applyTuningResult(message) {
    const record = tuningControls.get(tuningKey(message.node, message.name));
    if (!record) return;
    // Snap to whatever the node reports it now holds -- on a rejection
    // that is the *unchanged* value, so the control stops showing a
    // number the car never accepted.
    if (message.value !== null && message.value !== undefined) {
      record.sliding = false;
      setTuningControlValue(record, message.value);
    }
    record.row.classList.remove('applied', 'rejected');
    if (message.ok) {
      // Restart the flash animation rather than relying on class toggling
      // alone, which a browser will collapse into no animation at all.
      void record.row.offsetWidth;
      record.row.classList.add('applied');
      record.note.textContent = '';
    } else {
      record.row.classList.add('rejected');
      record.note.textContent = message.reason || 'rejected';
    }
  }

  function setTuningArmed(armed) {
    state.tuningArmed = !!armed;
    tuningArm.checked = state.tuningArmed;
    tuningPanel.classList.toggle('armed', state.tuningArmed);
    tuningArmNote.textContent = state.tuningArmed
      ? 'ARMED · changes reach the car as soon as you release a control'
      : 'disarmed · controls are read-only until you arm';
    updateTuningEnabled();
  }

  function updateTuningEnabled() {
    const armed = state.tuningArmed;
    tuningControls.forEach((record) => {
      if (record.toggle) record.toggle.disabled = !armed;
      if (record.slider) record.slider.disabled = !armed;
      if (record.number) record.number.disabled = !armed;
      record.revert.disabled = !armed;
    });
    const savable = !!(state.tuning && state.tuning.allowSave
      && (state.tuning.nodes || []).some((n) => n.online && n.savable));
    tuningSave.disabled = !armed || !savable;
  }

  function showTuningSaveResult(message) {
    tuningSaveStatus.className = message.ok ? 'ok' : 'error';
    tuningSaveStatus.textContent = message.detail
      + (message.files && message.files.length ? ` [${message.files.join(', ')}]` : '');
  }

  tuningOpen.addEventListener('click', () => {
    tuningPanel.classList.add('open');
    tuningPanel.setAttribute('aria-hidden', 'false');
  });
  tuningClose.addEventListener('click', () => {
    tuningPanel.classList.remove('open');
    tuningPanel.setAttribute('aria-hidden', 'true');
  });
  tuningArm.addEventListener('change', () => {
    // Ask, don't assume: the checkbox is repainted from the server's
    // answer (setTuningArmed), so if the server refuses, it stays off.
    sendTuningControl({ action: 'arm', armed: tuningArm.checked });
  });
  tuningSave.addEventListener('click', () => {
    const files = (state.tuning ? state.tuning.nodes : [])
      .filter((n) => n.online && n.savable)
      .map((n) => n.config_path)
      .join('\n');
    if (!window.confirm(
      `Write the current tune into:\n\n${files}\n\n`
      + 'These are the workspace\'s tracked config files -- review the '
      + 'change with `git diff` afterwards.')) return;
    tuningSaveStatus.className = '';
    tuningSaveStatus.textContent = 'saving...';
    sendTuningControl({ action: 'save' });
  });

  // ---------------------------------------------------------------------
  // Camera feed (bottom-right inset): usb_cam_stream is a separate node
  // on its own port (9090, see docs/usb-camera-livestream.md), not part
  // of this WebSocket protocol at all -- an MJPEG stream is just an <img>
  // whose connection never closes, so it's simplest to point one at it
  // directly rather than routing frames through dashboard_node.
  // ---------------------------------------------------------------------
  const CAMERA_PORT = 9090;
  let cameraConnected = false;

  function tryCameraConnect() {
    // Cache-bust: without this, a browser that already failed to load
    // this exact URL once may just replay the cached failure instead of
    // actually retrying the connection.
    // The preview tier -- small and cheap. This inset is at most 220 CSS
    // pixels wide, and it used to be fed a 1280x720 stream: roughly 34x
    // more picture than it could ever show, at 12-18 Mbit/s, competing
    // with this dashboard's own telemetry for the same WiFi link. The
    // recording view (camera.js) asks for the full tier instead.
    cameraFeed.src =
      `http://${location.hostname}:${CAMERA_PORT}/stream?tier=preview&_=${Date.now()}`;
  }

  cameraFeed.addEventListener('load', () => {
    cameraConnected = true;
    cameraPanel.classList.add('has-feed');
    // First frame is also the first time the stream's real shape is known,
    // which is what the panel gets sized to from here on.
    if (cameraFeed.naturalWidth > 0 && cameraFeed.naturalHeight > 0) {
      cameraAspect = cameraFeed.naturalWidth / cameraFeed.naturalHeight;
      applyCameraSize();
    }
  });
  cameraFeed.addEventListener('error', () => {
    cameraConnected = false;
    cameraPanel.classList.remove('has-feed');
  });
  tryCameraConnect();
  setInterval(() => { if (!cameraConnected) tryCameraConnect(); }, 3000);

  // ---------------------------------------------------------------------
  // Resizing the camera inset. The panel is pinned to the bottom-right, so
  // its top-left corner is the only one that can move -- that's where the
  // grip is. A drag *scales* the panel along the stream's own aspect ratio
  // rather than reshaping it freely, so the inset is always exactly the
  // shape of the frame: the whole image stays visible, never cropped and
  // never letterboxed, at whatever size the driver wants it.
  //
  // The size is remembered in localStorage, since "make the camera bigger"
  // is a per-person preference, not a per-session one. Double-clicking the
  // grip forgets it and returns to the CSS default.
  // ---------------------------------------------------------------------
  const CAMERA_MIN_WIDTH = 120;
  const CAMERA_WIDTH_KEY = 'racerbot.dashboard.cameraWidth';
  const PANEL_GAP = 12; // matches the 12px inset every fixed panel uses

  let cameraAspect = 4 / 3; // until the first frame reports its real shape
  let cameraWidth = null;   // null = untouched, follow the CSS default size

  // Grow only into empty space: never across the sidebar, never up into
  // the minimap, never off the top of the window.
  function cameraMaxSize() {
    const style = getComputedStyle(cameraPanel);
    const right = parseFloat(style.right) || PANEL_GAP;
    const bottom = parseFloat(style.bottom) || 47;
    // "Do not grow across the sidebar" means nothing in the phone layout,
    // where the sidebar is a full-width sheet along the bottom: its right
    // edge is the right edge of the screen, so this clamp would compute a
    // negative width and pin the camera to its 120px minimum forever.
    const overlayRight = document.body.classList.contains('layout-phone')
      ? PANEL_GAP
      : overlay.getBoundingClientRect().right;
    const minimapBottom = minimapPanel.getBoundingClientRect().bottom;
    return {
      width: Math.max(CAMERA_MIN_WIDTH, window.innerWidth - right - overlayRight - PANEL_GAP),
      height: Math.max(CAMERA_MIN_WIDTH / cameraAspect,
                       window.innerHeight - bottom - minimapBottom - PANEL_GAP),
    };
  }

  // Lay the panel out at the stream's aspect ratio. Until the grip is
  // actually dragged the width stays on its responsive CSS clamp and only
  // the height is derived from it, so an untouched dashboard still scales
  // with the window the way it always did; once dragged, both are pinned.
  function applyCameraSize() {
    const max = cameraMaxSize();
    if (cameraWidth === null) {
      cameraPanel.style.width = '';
      const cssWidth = cameraPanel.getBoundingClientRect().width;
      const width = Math.max(CAMERA_MIN_WIDTH, Math.min(cssWidth, max.height * cameraAspect));
      // Only override the CSS width in the corner case where the derived
      // height wouldn't fit -- otherwise leave the clamp alone.
      if (width < cssWidth - 0.5) cameraPanel.style.width = `${Math.round(width)}px`;
      cameraPanel.style.height = `${Math.round(width / cameraAspect)}px`;
      return;
    }
    cameraWidth = Math.max(CAMERA_MIN_WIDTH,
                           Math.min(cameraWidth, max.width, max.height * cameraAspect));
    cameraPanel.style.width = `${Math.round(cameraWidth)}px`;
    cameraPanel.style.height = `${Math.round(cameraWidth / cameraAspect)}px`;
  }

  function setCameraWidth(width) {
    cameraWidth = width;
    applyCameraSize();
  }

  function resetCameraSize() {
    cameraWidth = null;
    cameraPanel.style.width = '';
    cameraPanel.style.height = '';
    applyCameraSize();
    try {
      localStorage.removeItem(CAMERA_WIDTH_KEY);
    } catch (err) { /* private mode / storage disabled -- size just won't persist */ }
  }

  let resizeStart = null;

  cameraResize.addEventListener('pointerdown', (e) => {
    e.preventDefault(); // no text selection, no native link drag
    const rect = cameraPanel.getBoundingClientRect();
    resizeStart = { x: e.clientX, y: e.clientY, width: rect.width, height: rect.height };
    cameraResize.setPointerCapture(e.pointerId);
    cameraPanel.classList.add('resizing');
  });

  cameraResize.addEventListener('pointermove', (e) => {
    if (!resizeStart) return;
    // Dragging up/left grows the panel, since the opposite corner is fixed.
    const wantWidth = resizeStart.width + (resizeStart.x - e.clientX);
    const wantHeight = resizeStart.height + (resizeStart.y - e.clientY);
    // The pointer can wander off the fixed-aspect diagonal, so project onto
    // it (least-squares) instead of picking one axis -- that way a mostly
    // sideways drag and a mostly vertical one both feel like they're
    // dragging the corner, in either direction.
    const scale = (wantWidth * cameraAspect + wantHeight) / (cameraAspect * cameraAspect + 1);
    setCameraWidth(scale * cameraAspect);
  });

  function endCameraResize(e) {
    if (!resizeStart) return;
    resizeStart = null;
    cameraPanel.classList.remove('resizing');
    if (cameraResize.hasPointerCapture(e.pointerId)) cameraResize.releasePointerCapture(e.pointerId);
    try {
      if (cameraWidth !== null) localStorage.setItem(CAMERA_WIDTH_KEY, String(cameraWidth));
    } catch (err) { /* see resetCameraSize */ }
  }

  cameraResize.addEventListener('pointerup', endCameraResize);
  cameraResize.addEventListener('pointercancel', endCameraResize);
  cameraResize.addEventListener('dblclick', resetCameraSize);

  // A window that shrank can leave the panel overlapping the sidebar or
  // minimap, so re-clamp (this rides along with the canvas's own resize
  // handler above, which only re-renders the map).
  window.addEventListener('resize', applyCameraSize);

  try {
    const saved = parseFloat(localStorage.getItem(CAMERA_WIDTH_KEY));
    if (Number.isFinite(saved)) setCameraWidth(saved);
  } catch (err) { /* see resetCameraSize */ }

  // ---------------------------------------------------------------------
  // Collapsible sections.
  //
  // The sidebar carries six sections and they do not all fit a laptop, let
  // alone a phone. Rather than shrink everything until it is unreadable,
  // sections collapse -- and a collapsed one still shows its headline value
  // in its own header, so folding "vehicle" away does not cost you the
  // speed. Which sections you keep open is a personal preference rather
  // than a per-session one, so it is remembered in localStorage, the same
  // way the camera inset remembers its size.
  // ---------------------------------------------------------------------
  const SECTION_STATE_KEY = 'racerbot.dashboard.sections';
  const sectionEls = Array.from(document.querySelectorAll('.section[data-section]'));
  const digestEls = {
    feeds: document.getElementById('digest-feeds'),
    intent: document.getElementById('digest-intent'),
    vehicle: document.getElementById('digest-vehicle'),
    stopwatch: document.getElementById('digest-stopwatch'),
    system: document.getElementById('digest-system'),
    tuning: document.getElementById('digest-tuning'),
    processes: document.getElementById('digest-processes'),
    measure: document.getElementById('digest-measure'),
    maps: document.getElementById('digest-maps'),
  };

  function restoreSectionState() {
    let saved = null;
    try {
      saved = JSON.parse(localStorage.getItem(SECTION_STATE_KEY) || 'null');
    } catch (err) { /* private mode / storage disabled -- keep the defaults */ }
    if (!saved || typeof saved !== 'object') return;
    sectionEls.forEach((section) => {
      const name = section.dataset.section;
      if (typeof saved[name] === 'boolean') section.open = saved[name];
    });
  }

  function saveSectionState() {
    const state = {};
    sectionEls.forEach((section) => { state[section.dataset.section] = section.open; });
    try {
      localStorage.setItem(SECTION_STATE_KEY, JSON.stringify(state));
    } catch (err) { /* see restoreSectionState */ }
  }

  sectionEls.forEach((section) => {
    section.addEventListener('toggle', () => {
      saveSectionState();
      // An opened section may need its contents laid out against a canvas
      // that has not been repainted since.
      scheduleRender();
    });
  });
  restoreSectionState();

  function setDigest(name, text) {
    const element = digestEls[name];
    if (element) element.textContent = text;
  }

  // The one number each collapsed section is actually about.
  function updateDigests() {
    const live = [state.map, state.scan, state.pose, state.drive]
      .filter((entry) => entry && !isStale(entry)).length;
    setDigest('feeds', `${live}/4 live`);

    const intentStale = intentAgeMs() > INTENT_STALE_MS;
    const intentText = state.intent
      ? String(state.intent.state || '').replace(/_/g, ' ') : '--';
    setDigest('intent', state.intent
      ? intentText + (intentStale ? ' (stale)' : '')
      : '--');

    const speedText = state.speed && !isStale(state.speed)
      ? `${state.speed.speed.toFixed(1)} m/s` : '--';
    setDigest('vehicle', speedText);

    // The phone strip, from the values just computed. Only the live
    // connection sets it back to a real state after a drop, so a stale
    // intent is shown uncoloured rather than as a confident green DRIVE
    // that stopped being true a second ago.
    if (ws && ws.readyState === WebSocket.OPEN) {
      stripState.textContent = state.intent ? intentText : 'no intent';
      stripState.className = 'strip-state'
        + (state.intent && !intentStale ? ` intent-${state.intent.severity}` : '');
      stripSpeed.textContent = speedText;
      stripFeeds.textContent = `${live}/4`;
    }

    setDigest('stopwatch', formatStopwatch(stopwatchElapsed()));

    setDigest('system', state.stats
      ? `${state.stats.cpuPercent.toFixed(0)}%`
        + (state.stats.cpuTempC != null ? ` ${state.stats.cpuTempC.toFixed(0)}C` : '')
      : '--');

    const tuning = state.tuning;
    const onlineNodes = tuning ? (tuning.nodes || []).filter((n) => n.online) : [];
    setDigest('tuning', onlineNodes.length ? `${onlineNodes.length} node(s)` : '--');
  }

  // ---------------------------------------------------------------------
  // Measuring and map controls: wiring
  // ---------------------------------------------------------------------
  if (measureEnable) {
    measureEnable.addEventListener('change', () => setMeasureActive(measureEnable.checked));
  }
  if (measureUndoBtn) measureUndoBtn.addEventListener('click', undoMeasure);
  if (measureClearBtn) measureClearBtn.addEventListener('click', clearMeasure);
  if (mapClearViewBtn) {
    mapClearViewBtn.addEventListener('click', () => {
      setMapStatus('clearing...');
      sendMapControl({ action: 'clear_view' });
    });
  }
  if (mapResetSlamBtn) {
    mapResetSlamBtn.addEventListener('click', () => {
      setMapStatus('asking slam_toolbox to reset...');
      sendMapControl({ action: 'reset_slam' });
    });
  }

  // Keyboard, and only when the person is not typing. The delete
  // confirmation is a text field, and "M" is a letter that appears in a
  // run name -- toggling the measuring tool while someone types it would
  // be its own small disaster.
  function typingInAField() {
    const el = document.activeElement;
    if (!el) return false;
    const tag = (el.tagName || '').toLowerCase();
    return tag === 'input' || tag === 'textarea' || el.isContentEditable;
  }

  window.addEventListener('keydown', (e) => {
    if (e.metaKey || e.ctrlKey || e.altKey) return;
    if (typingInAField()) return;
    const key = e.key;
    if (key === 'm' || key === 'M') {
      e.preventDefault();
      setMeasureActive(!measure.active);
      return;
    }
    if (!measure.active) return;
    if (key === 'Escape') {
      e.preventDefault();
      // Escape clears a measurement in progress; a second Escape, with
      // nothing left to clear, puts the tool away.
      if (measure.pts.length) clearMeasure();
      else setMeasureActive(false);
    } else if (key === 'Backspace' || key === 'Delete' || key === 'u') {
      e.preventDefault();
      undoMeasure();
    }
  });

  renderMeasurePanel();

  // ---------------------------------------------------------------------
  // Go
  // ---------------------------------------------------------------------
  resizeCanvasIfNeeded();
  connect();
  render();
})();
