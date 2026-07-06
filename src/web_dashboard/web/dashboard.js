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
  const infoMap = document.getElementById('info-map');
  const infoScan = document.getElementById('info-scan');
  const infoPose = document.getElementById('info-pose');
  const infoDrive = document.getElementById('info-drive');
  const infoCpu = document.getElementById('info-cpu');
  const infoMem = document.getElementById('info-mem');
  const infoTemp = document.getElementById('info-temp');
  const infoWifiText = document.getElementById('info-wifi-text');
  const infoUptime = document.getElementById('info-uptime');
  const wifiBarEls = document.querySelectorAll('#wifi-bars .wifi-bar');
  const dots = {
    map: document.getElementById('dot-map'),
    scan: document.getElementById('dot-scan'),
    pose: document.getElementById('dot-pose'),
    drive: document.getElementById('dot-drive'),
    stats: document.getElementById('dot-stats'),
  };
  const modeBanner = document.getElementById('mode-banner');
  const resetViewBtn = document.getElementById('reset-view');

  const minimapPanel = document.getElementById('minimap-panel');
  const minimapCanvas = document.getElementById('minimap');
  const minimapCtx = minimapCanvas.getContext('2d');
  const cameraPanel = document.getElementById('camera-panel');
  const cameraFeed = document.getElementById('camera-feed');

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
    drive: null, // { speed, steeringAngle, receivedAt } -- whatever /drive currently carries
    stats: null, // { cpuPercent, memPercent, cpuTempC, uptimeS, wifiDbm, receivedAt }
  };

  // Pending "what does the next binary frame mean" -- set when a JSON
  // header arrives, consumed the moment the binary payload right after
  // it does (the server always sends them as an immediate pair).
  let pendingBinaryType = null;
  let pendingHeader = null;

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

  // ---------------------------------------------------------------------
  // WebSocket connection, with automatic reconnect -- a dropped WiFi link
  // shouldn't require reloading the page.
  // ---------------------------------------------------------------------
  let ws = null;

  function connect() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => setConnected(true);
    ws.onclose = () => {
      setConnected(false);
      setTimeout(connect, 1000); // keep trying -- cheap, and self-heals a dropped link
    };
    ws.onerror = () => ws.close();
    ws.onmessage = onMessage;
  }

  function setConnected(connected) {
    connDot.className = 'dot ' + (connected ? 'dot-green' : 'dot-red');
    connText.textContent = connected ? 'connected' : 'disconnected -- retrying...';
  }

  function onMessage(event) {
    if (typeof event.data === 'string') {
      handleHeader(JSON.parse(event.data));
    } else {
      handleBinary(event.data);
    }
  }

  function handleHeader(header) {
    if (header.type === 'map' || header.type === 'scan') {
      pendingBinaryType = header.type;
      pendingHeader = header;
    } else if (header.type === 'pose') {
      state.pose = { x: header.x, y: header.y, yaw: header.yaw, receivedAt: performance.now() };
      maybeAutoFit();
      render();
    } else if (header.type === 'drive') {
      state.drive = { speed: header.speed, steeringAngle: header.steering_angle, receivedAt: performance.now() };
      render();
    } else if (header.type === 'stats') {
      state.stats = {
        cpuPercent: header.cpu_percent,
        memPercent: header.mem_percent,
        cpuTempC: header.cpu_temp_c,
        uptimeS: header.uptime_s,
        wifiDbm: header.wifi_dbm,
        receivedAt: performance.now(),
      };
      render();
    }
  }

  function handleBinary(buffer) {
    if (pendingBinaryType === 'map') {
      applyMap(pendingHeader, new Int8Array(buffer));
    } else if (pendingBinaryType === 'scan') {
      applyScan(pendingHeader, new Float32Array(buffer));
    }
    pendingBinaryType = null;
    pendingHeader = null;
    maybeAutoFit();
    render();
  }

  // ---------------------------------------------------------------------
  // Turning a raw occupancy grid into something drawable, once per map
  // update (not once per frame): render it into an off-screen canvas at
  // its native resolution (1 pixel per cell), so the visible canvas can
  // just scale/position that image with a single fast drawImage() call
  // every frame instead of redrawing every cell every frame.
  // ---------------------------------------------------------------------
  function applyMap(header, cells) {
    const { width, height, resolution, origin_x: originX, origin_y: originY } = header;
    const off = document.createElement('canvas');
    off.width = width;
    off.height = height;
    const octx = off.getContext('2d');
    const img = octx.createImageData(width, height);

    // OccupancyGrid.data is row-major with row 0 at the *bottom* of the
    // map (smallest world Y); a plain <canvas> image has row 0 at the
    // *top*. Flipping rows here, once, means everywhere else in this
    // file can treat "top of the map image" as "largest world Y" without
    // re-deriving that each time it's needed.
    for (let row = 0; row < height; row++) {
      const srcRow = height - 1 - row;
      for (let col = 0; col < width; col++) {
        const value = cells[srcRow * width + col];
        const p = (row * width + col) * 4;
        let gray;
        if (value < 0) {
          gray = 128; // unknown
        } else {
          // 0 (free) -> white (255), 100 (occupied) -> near-black (20)
          gray = 255 - Math.round((Math.min(value, 100) / 100) * 235);
        }
        img.data[p] = gray;
        img.data[p + 1] = gray;
        img.data[p + 2] = value < 0 ? 150 : gray; // faint blue tint on "unknown" so it visually reads differently from "free"
        img.data[p + 3] = 255;
      }
    }
    octx.putImageData(img, 0, 0);

    state.map = { width, height, resolution, originX, originY, canvas: off, receivedAt: performance.now() };
  }

  function applyScan(header, ranges) {
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
  function maybeAutoFit() {
    if (view.userAdjusted) return;
    if (state.map) {
      const { width, height, resolution, originX, originY } = state.map;
      view.centerX = originX + (width * resolution) / 2;
      view.centerY = originY + (height * resolution) / 2;
      const spanMeters = Math.max(width, height) * resolution;
      view.scale = Math.min(canvas.width, canvas.height) / (spanMeters * 1.15);
    } else if (state.pose) {
      view.centerX = state.pose.x;
      view.centerY = state.pose.y;
    }
  }

  // ---------------------------------------------------------------------
  // Rendering
  // ---------------------------------------------------------------------
  function render() {
    resizeCanvasIfNeeded();
    ctx.fillStyle = '#0b0f14';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    const mapRelative = !!state.pose; // do we know exactly where the car is in the map frame?

    if (state.map) {
      drawMap();
      modeBanner.textContent = mapRelative ? '' : 'map loaded -- waiting for a localization pose (RViz "2D Pose Estimate"?)';
    } else {
      modeBanner.textContent = 'no map yet -- showing raw LIDAR relative to the car';
    }

    if (state.scan) {
      if (mapRelative && state.map) {
        drawBlindSpotMapRelative();
        drawScanMapRelative();
      } else if (!state.map) {
        drawBlindSpotRobotCentric();
        drawScanRobotCentric();
      }
      // Map loaded but no pose yet: deliberately not drawing the scan --
      // plotting it without a pose would just be a guess dressed up as
      // data, and the banner above already explains why.
    }

    if (mapRelative) {
      drawCarMapRelative();
    } else if (state.scan) {
      drawCarRobotCentric();
    }

    drawScaleBar();
    drawMinimap();

    updateStatusText();
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
    minimapCtx.fillStyle = '#0b0f14';
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
    minimapCtx.strokeStyle = 'rgba(255, 255, 255, 0.55)';
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
    minimapCtx.fillStyle = '#f85149';
    minimapCtx.fill();
    minimapCtx.restore();
  }

  // A classic map "ruler": picks a round length (in meters) that renders
  // between 80-160px at the current zoom, so it stays readable whether
  // you're zoomed into a corner or fitted to the whole map.
  const SCALE_BAR_STEPS_M = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200];

  function drawScaleBar() {
    let meters = SCALE_BAR_STEPS_M[0];
    for (const step of SCALE_BAR_STEPS_M) {
      meters = step;
      if (step * view.scale >= 80) break;
    }
    const px = meters * view.scale;
    const dpr = window.devicePixelRatio || 1;
    const x0 = 20 * dpr;
    const y0 = canvas.height - 26 * dpr;

    ctx.save();
    ctx.strokeStyle = '#e6edf3';
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x0 + px, y0);
    ctx.moveTo(x0, y0 - 5 * dpr);
    ctx.lineTo(x0, y0 + 5 * dpr);
    ctx.moveTo(x0 + px, y0 - 5 * dpr);
    ctx.lineTo(x0 + px, y0 + 5 * dpr);
    ctx.stroke();

    ctx.fillStyle = '#e6edf3';
    ctx.font = `${12 * dpr}px sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    const label = meters >= 1 ? `${meters} m` : `${Math.round(meters * 100)} cm`;
    ctx.fillText(label, x0 + px / 2, y0 - 8 * dpr);
    ctx.restore();
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

    ctx.fillStyle = '#58a6ff';
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (!Number.isFinite(r) || r < rangeMin || r > rangeMax) continue;
      const angle = pose.yaw + angleMin + i * angleIncrement;
      const wx = laserWorldX + r * Math.cos(angle);
      const wy = laserWorldY + r * Math.sin(angle);
      const [cx, cy] = worldToCanvas(wx, wy);
      ctx.fillRect(cx - 1, cy - 1, 2, 2);
    }
  }

  function drawScanRobotCentric() {
    const { angleMin, angleIncrement, rangeMin, rangeMax, ranges } = state.scan;
    ctx.fillStyle = '#58a6ff';
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (!Number.isFinite(r) || r < rangeMin || r > rangeMax) continue;
      const angle = angleMin + i * angleIncrement;
      const bx = r * Math.cos(angle);
      const by = r * Math.sin(angle);
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
    const { rangeMax } = state.scan;
    const [ox, oy] = bodyToCanvas(0, 0);
    ctx.fillStyle = 'rgba(248, 81, 73, 0.18)';
    drawWedge(ox, oy, span.from, span.to, (a) => bodyToCanvas(rangeMax * Math.cos(a), rangeMax * Math.sin(a)));
    drawBlindSpotLabel(ox, oy, (span.from + span.to) / 2, rangeMax, (a, r) => bodyToCanvas(r * Math.cos(a), r * Math.sin(a)));
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
    ctx.fillStyle = 'rgba(248, 81, 73, 0.18)';
    drawWedge(ox, oy, span.from, span.to, (a) => toPoint(a, rangeMax));
    drawBlindSpotLabel(ox, oy, (span.from + span.to) / 2, rangeMax, toPoint);
  }

  function drawBlindSpotLabel(ox, oy, midAngle, rangeMax, angleToPoint) {
    const [lx, ly] = angleToPoint(midAngle, rangeMax * 0.55);
    ctx.save();
    ctx.fillStyle = 'rgba(230, 237, 243, 0.6)';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('blind spot', lx, ly);
    ctx.restore();
  }

  function drawCarMapRelative() {
    const [cx, cy] = worldToCanvas(state.pose.x, state.pose.y);
    // Canvas angle = -yaw: world yaw is measured counterclockwise, but
    // canvas rotation is clockwise once Y has been flipped -- negating
    // here keeps the icon pointing the same visual direction the car is
    // actually facing.
    drawCarIcon(cx, cy, -state.pose.yaw);
  }

  function drawCarRobotCentric() {
    const [cx, cy] = bodyToCanvas(0, 0); // at canvas center until the user pans
    // drawCarIcon's un-rotated "front" points along local +X (canvas
    // right, see the comment on drawCarIcon) -- but bodyToCanvas renders
    // forward (bx) as canvas "up", not "right". -PI/2 rotates the icon to
    // actually point up, matching where the scan/blind-spot are drawn;
    // passing 0 here previously left the icon facing sideways while the
    // blind-spot wedge (correctly) rendered behind it.
    drawCarIcon(cx, cy, -Math.PI / 2);
  }

  // A top-down car silhouette, front along local +X before rotation (angle
  // 0 = facing canvas right) -- a rounded body plus a lighter front stripe
  // and four wheels so heading is obvious at a glance, unlike a bare
  // rectangle which looks the same front-to-back.
  function drawCarIcon(cx, cy, angle) {
    const size = Math.max(10, view.scale * 0.22);
    const bodyLen = size * 1.8;
    const halfWidth = size * 0.6;
    const rearX = -bodyLen * 0.42;
    const frontX = bodyLen * 0.58;
    const r = size * 0.28;

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(angle);

    // Wheels first, so the body draws on top of their inner edges.
    const wheelLen = size * 0.5;
    const wheelThick = size * 0.22;
    ctx.fillStyle = '#1c1f24';
    for (const ax of [frontX - bodyLen * 0.22, rearX + bodyLen * 0.22]) {
      for (const side of [-1, 1]) {
        const wy = side * (halfWidth + wheelThick * 0.35);
        ctx.fillRect(ax - wheelLen / 2, wy - wheelThick / 2, wheelLen, wheelThick);
      }
    }

    // Body: rounded rectangle, nose toward +X.
    ctx.beginPath();
    ctx.moveTo(rearX + r, -halfWidth);
    ctx.lineTo(frontX - r, -halfWidth);
    ctx.quadraticCurveTo(frontX, -halfWidth, frontX, -halfWidth + r);
    ctx.lineTo(frontX, halfWidth - r);
    ctx.quadraticCurveTo(frontX, halfWidth, frontX - r, halfWidth);
    ctx.lineTo(rearX + r, halfWidth);
    ctx.quadraticCurveTo(rearX, halfWidth, rearX, halfWidth - r);
    ctx.lineTo(rearX, -halfWidth + r);
    ctx.quadraticCurveTo(rearX, -halfWidth, rearX + r, -halfWidth);
    ctx.closePath();
    ctx.fillStyle = '#f85149';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = Math.max(1, size * 0.06);
    ctx.stroke();

    // Windshield-ish stripe near the front -- the one visual cue that
    // makes "which end is the front" unambiguous at a glance.
    ctx.fillStyle = 'rgba(255, 255, 255, 0.55)';
    ctx.fillRect(frontX - bodyLen * 0.34, -halfWidth * 0.7, bodyLen * 0.14, halfWidth * 1.4);

    ctx.restore();
  }

  // ---------------------------------------------------------------------
  // Status text + staleness. Age is recomputed on a fixed timer (below),
  // not just whenever a message happens to arrive, specifically so a
  // *frozen* feed is visibly reported as stale instead of silently
  // leaving the last good value on screen forever.
  // ---------------------------------------------------------------------
  const STALE_AFTER_MS = 1000;
  const STALE_COLOR = '#f85149';

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

  function updateStatusText() {
    infoMap.textContent = state.map
      ? `${state.map.width}x${state.map.height} @ ${state.map.resolution.toFixed(3)}m/cell, ${ageText(state.map)}`
      : 'no map yet';
    infoScan.textContent = state.scan ? `${state.scan.ranges.length} pts, ${ageText(state.scan)}` : 'no scan yet';
    infoPose.textContent = state.pose
      ? `${state.pose.x.toFixed(2)}, ${state.pose.y.toFixed(2)}m @ ${(state.pose.yaw * 180 / Math.PI).toFixed(0)}deg, ${ageText(state.pose)}`
      : 'no pose yet';
    infoDrive.textContent = state.drive
      ? `${state.drive.speed.toFixed(2)}m/s @ ${(state.drive.steeringAngle * 180 / Math.PI).toFixed(1)}deg, ${ageText(state.drive)}`
      : 'no drive yet';

    if (state.stats) {
      infoCpu.textContent = `${state.stats.cpuPercent.toFixed(0)}%`;
      infoMem.textContent = `${state.stats.memPercent.toFixed(0)}%`;
      infoTemp.textContent = state.stats.cpuTempC != null ? `${state.stats.cpuTempC.toFixed(0)}C` : 'n/a';
      infoWifiText.textContent = state.stats.wifiDbm != null ? `${state.stats.wifiDbm.toFixed(0)}dBm` : 'n/a';
      infoUptime.textContent = formatUptime(state.stats.uptimeS);
      updateWifiBars(state.stats.wifiDbm);
    } else {
      infoCpu.textContent = infoMem.textContent = infoTemp.textContent = infoUptime.textContent = '--';
      infoWifiText.textContent = '--';
      updateWifiBars(null);
    }

    setDot(dots.map, state.map);
    setDot(dots.scan, state.scan);
    setDot(dots.pose, state.pose);
    setDot(dots.drive, state.drive);
    // Stats only tick once per stats_interval_sec (default 1Hz) -- the
    // shared 1s STALE_AFTER_MS would flicker red between every tick, so
    // this row gets a longer threshold (a few sample periods of slack).
    setDot(dots.stats, state.stats, 3000);
  }

  // Re-render periodically even with no new messages, purely so the
  // "updated Xs ago" readout and stale-data coloring stay live.
  setInterval(() => { if (ws && ws.readyState === WebSocket.OPEN) render(); }, 250);

  // ---------------------------------------------------------------------
  // Pan / zoom
  // ---------------------------------------------------------------------
  let dragging = false;
  let lastX = 0;
  let lastY = 0;

  canvas.addEventListener('mousedown', (e) => {
    dragging = true;
    lastX = e.clientX;
    lastY = e.clientY;
  });
  window.addEventListener('mouseup', () => { dragging = false; });
  window.addEventListener('mousemove', (e) => {
    if (!dragging) return;
    const dpr = window.devicePixelRatio || 1;
    const dx = (e.clientX - lastX) * dpr;
    const dy = (e.clientY - lastY) * dpr;
    lastX = e.clientX;
    lastY = e.clientY;
    // Update both the world-frame pan (read by worldToCanvas, once a map
    // or pose exists) and the body-frame pan (read by bodyToCanvas, before
    // then) -- render() only uses whichever is actually active, but a drag
    // can happen in either mode so both need to track it.
    view.centerX -= dx / view.scale;
    view.centerY += dy / view.scale; // canvas +Y is down, world +Y is up
    view.bodyPanY += dx / view.scale;
    view.bodyPanX += dy / view.scale;
    view.userAdjusted = true;
    render();
  });

  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const dpr = window.devicePixelRatio || 1;
    const mouseCanvasX = e.clientX * dpr;
    const mouseCanvasY = e.clientY * dpr;
    const zoomFactor = Math.exp(-e.deltaY * 0.001);

    if (state.pose) {
      // World point currently under the cursor, before changing scale.
      const worldXBefore = view.centerX + (mouseCanvasX - canvas.width / 2) / view.scale;
      const worldYBefore = view.centerY - (mouseCanvasY - canvas.height / 2) / view.scale;
      view.scale = Math.min(Math.max(view.scale * zoomFactor, 2), 4000);
      // Re-center so the same world point stays under the cursor -- "zoom
      // to cursor" rather than "zoom to canvas center".
      view.centerX = worldXBefore - (mouseCanvasX - canvas.width / 2) / view.scale;
      view.centerY = worldYBefore + (mouseCanvasY - canvas.height / 2) / view.scale;
    } else {
      // Same idea in body-frame coordinates (see bodyToCanvas) -- keeping
      // this mode-aware, rather than always updating centerX/centerY,
      // avoids leaving stale world-frame values that would otherwise make
      // the view jump the instant a pose first arrives and mapRelative
      // mode switches on.
      const byBefore = view.bodyPanY - (mouseCanvasX - canvas.width / 2) / view.scale;
      const bxBefore = view.bodyPanX - (mouseCanvasY - canvas.height / 2) / view.scale;
      view.scale = Math.min(Math.max(view.scale * zoomFactor, 2), 4000);
      view.bodyPanY = byBefore + (mouseCanvasX - canvas.width / 2) / view.scale;
      view.bodyPanX = bxBefore + (mouseCanvasY - canvas.height / 2) / view.scale;
    }
    view.userAdjusted = true;
    render();
  }, { passive: false });

  resetViewBtn.addEventListener('click', () => {
    view.userAdjusted = false;
    view.bodyPanX = 0;
    view.bodyPanY = 0;
    maybeAutoFit();
    render();
  });

  window.addEventListener('resize', render);

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
    cameraFeed.src = `http://${location.hostname}:${CAMERA_PORT}/stream?_=${Date.now()}`;
  }

  cameraFeed.addEventListener('load', () => {
    cameraConnected = true;
    cameraPanel.classList.add('has-feed');
  });
  cameraFeed.addEventListener('error', () => {
    cameraConnected = false;
    cameraPanel.classList.remove('has-feed');
  });
  tryCameraConnect();
  setInterval(() => { if (!cameraConnected) tryCameraConnect(); }, 3000);

  // ---------------------------------------------------------------------
  // Go
  // ---------------------------------------------------------------------
  resizeCanvasIfNeeded();
  connect();
  render();
})();
