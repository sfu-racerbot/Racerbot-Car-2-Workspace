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
  const modeBanner = document.getElementById('mode-banner');
  const resetViewBtn = document.getElementById('reset-view');

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
    userAdjusted: false, // once the user pans/zooms, stop auto-fitting on new data
  };

  function worldToCanvas(wx, wy) {
    return [
      canvas.width / 2 + (wx - view.centerX) * view.scale,
      canvas.height / 2 - (wy - view.centerY) * view.scale, // minus: world +Y is up, canvas +Y is down
    ];
  }

  // Robot-centric fallback transform, used only when no pose has arrived
  // yet: the car is drawn fixed at the canvas center, always facing "up",
  // using body-frame coordinates straight off the LIDAR (x forward, y
  // left) -- no map, no pose, no localization needed at all, just /scan.
  function bodyToCanvas(bx, by) {
    return [
      canvas.width / 2 - by * view.scale,
      canvas.height / 2 - bx * view.scale,
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
        drawScanMapRelative();
      } else if (!state.map) {
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

    updateStatusText();
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

  function drawCarMapRelative() {
    const [cx, cy] = worldToCanvas(state.pose.x, state.pose.y);
    // Canvas angle = -yaw: world yaw is measured counterclockwise, but
    // canvas rotation is clockwise once Y has been flipped -- negating
    // here keeps the arrow pointing the same visual direction the car
    // is actually facing.
    drawCarTriangle(cx, cy, -state.pose.yaw);
  }

  function drawCarRobotCentric() {
    drawCarTriangle(canvas.width / 2, canvas.height / 2, 0); // fixed at center, always pointing "up" (forward)
  }

  function drawCarTriangle(cx, cy, angle) {
    const size = Math.max(10, view.scale * 0.22);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(angle);
    ctx.beginPath();
    ctx.moveTo(size, 0);
    ctx.lineTo(-size * 0.7, size * 0.6);
    ctx.lineTo(-size * 0.7, -size * 0.6);
    ctx.closePath();
    ctx.fillStyle = '#f85149';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.stroke();
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

  function isStale(entry) {
    return !entry || (performance.now() - entry.receivedAt) > STALE_AFTER_MS;
  }

  function updateStatusText() {
    infoMap.textContent = state.map
      ? `${state.map.width}x${state.map.height} cells @ ${state.map.resolution.toFixed(3)}m/cell, updated ${ageText(state.map)}`
      : 'no map yet';
    infoScan.textContent = state.scan ? `${state.scan.ranges.length} points, updated ${ageText(state.scan)}` : 'no scan yet';
    infoPose.textContent = state.pose
      ? `x=${state.pose.x.toFixed(2)}m y=${state.pose.y.toFixed(2)}m yaw=${(state.pose.yaw * 180 / Math.PI).toFixed(0)}deg, `
        + `updated ${ageText(state.pose)}`
      : 'no pose yet';

    infoMap.style.color = state.map && isStale(state.map) ? STALE_COLOR : '';
    infoScan.style.color = state.scan && isStale(state.scan) ? STALE_COLOR : '';
    infoPose.style.color = state.pose && isStale(state.pose) ? STALE_COLOR : '';
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
    view.centerX -= dx / view.scale;
    view.centerY += dy / view.scale; // canvas +Y is down, world +Y is up
    view.userAdjusted = true;
    render();
  });

  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const dpr = window.devicePixelRatio || 1;
    const mouseCanvasX = e.clientX * dpr;
    const mouseCanvasY = e.clientY * dpr;
    // World point currently under the cursor, before changing scale.
    const worldXBefore = view.centerX + (mouseCanvasX - canvas.width / 2) / view.scale;
    const worldYBefore = view.centerY - (mouseCanvasY - canvas.height / 2) / view.scale;

    const zoomFactor = Math.exp(-e.deltaY * 0.001);
    view.scale = Math.min(Math.max(view.scale * zoomFactor, 2), 4000);
    view.userAdjusted = true;

    // Re-center so the same world point stays under the cursor -- "zoom
    // to cursor" rather than "zoom to canvas center".
    view.centerX = worldXBefore - (mouseCanvasX - canvas.width / 2) / view.scale;
    view.centerY = worldYBefore + (mouseCanvasY - canvas.height / 2) / view.scale;
    render();
  }, { passive: false });

  resetViewBtn.addEventListener('click', () => {
    view.userAdjusted = false;
    maybeAutoFit();
    render();
  });

  window.addEventListener('resize', render);

  // ---------------------------------------------------------------------
  // Go
  // ---------------------------------------------------------------------
  resizeCanvasIfNeeded();
  connect();
  render();
})();
