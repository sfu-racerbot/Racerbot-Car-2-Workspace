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

  const modeBanner = document.getElementById('mode-banner');
  const resetViewBtn = document.getElementById('reset-view');

  if (intentToggle) {
    intentToggle.addEventListener('change', () => {
      state.showIntent = intentToggle.checked;
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
    // What the driving node says it is *trying* to do -- see
    // docs/drive-intent.md. `intent` is the raw schema payload; `reason`
    // is held separately because the car only re-sends the (sometimes
    // expensive) explanation on state changes and on a slow period, so
    // most messages deliberately arrive without one.
    intent: null,
    intentReason: '',
    intentReceivedAt: 0,
    intentLog: [],   // [{ state, severity, at, heldMs }] newest first
    showIntent: true,
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

  const INTENT_COLORS = {
    drive:   { fill: 'rgba(63, 185, 80, 0.28)',  edge: 'rgba(63, 185, 80, 0.90)',  ink: '#3fb950' },
    caution: { fill: 'rgba(210, 153, 34, 0.30)', edge: 'rgba(210, 153, 34, 0.92)', ink: '#d29922' },
    stop:    { fill: 'rgba(248, 81, 73, 0.28)',  edge: 'rgba(248, 81, 73, 0.92)',  ink: '#f85149' },
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
    } else if (header.type === 'tuning') {
      state.tuning = { enabled: header.enabled, allowSave: header.allow_save, nodes: header.nodes || [] };
      renderTuning();
    } else if (header.type === 'tuning_armed') {
      setTuningArmed(header.armed);
    } else if (header.type === 'tuning_result') {
      applyTuningResult(header);
    } else if (header.type === 'tuning_saved') {
      showTuningSaveResult(header);
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
  const MAP_FREE_RGB = [30, 44, 61];        // #1e2c3d
  const MAP_OCCUPIED_RGB = [150, 172, 196]; // #96acc4
  const MAP_UNKNOWN_RGBA = [38, 49, 64, 50]; // #263140 (panel border) at ~20% alpha
  const MAP_EDGE_COLOR = '#263140';         // same hairline as every panel border

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

    // Last, so it wins: a dropped frame is the thing most likely to be
    // making the picture wrong, and "the map looks glitchy" with nothing
    // on screen to explain it is exactly the situation this avoids.
    if (state.desyncCount > 0
        && performance.now() - state.desyncAt < DESYNC_BANNER_MS) {
      modeBanner.textContent =
        `link problem: dropped ${state.desyncCount} corrupted frame(s) `
        + `-- ${state.desyncDetail}. The picture below may be stale.`;
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

    // Intent under the car icon, so the car always reads as the thing the
    // arrow belongs to. Needs a body frame it can place: either a pose, or
    // the robot-centric fallback that exists whenever there is no map.
    if (state.intent && (mapRelative || !state.map)) {
      drawIntent(mapRelative && !!state.map);
    }

    if (mapRelative) {
      drawCarMapRelative();
    } else if (state.scan) {
      drawCarRobotCentric();
    }

    updateScaleBar();
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
    // The dashboard's accent blue (#58a6ff, as used for links/hover in
    // style.css) rather than plain white: it reads as a UI element on top
    // of the map instead of another shade of map.
    minimapCtx.strokeStyle = 'rgba(88, 166, 255, 0.75)';
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
  // at least 60 CSS pixels at the current zoom. It is an HTML element in
  // the right rail rather than canvas paint, so no floating panel can
  // cover it.
  const SCALE_BAR_STEPS_M = [0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200];

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

  const LIDAR_NEAR_M = 0.3;
  const LIDAR_FAR_M = 5.0;
  const LIDAR_COLORS = Array.from({ length: 17 }, (_, i) => {
    const hue = Math.round((i / 16) * 120); // red (near) -> yellow -> green (far)
    return `hsl(${hue} 85% 50%)`;
  });

  function lidarColor(range) {
    const normalized = Math.min(1, Math.max(0,
      (range - LIDAR_NEAR_M) / (LIDAR_FAR_M - LIDAR_NEAR_M)));
    return LIDAR_COLORS[Math.round(normalized * (LIDAR_COLORS.length - 1))];
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

  function drawScanRobotCentric() {
    const { angleMin, angleIncrement, rangeMin, rangeMax, ranges } = state.scan;
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (!Number.isFinite(r) || r < rangeMin || r > rangeMax) continue;
      ctx.fillStyle = lidarColor(r);
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
    // The ghost goes underneath: where it separates from the ribbon, the
    // gap is the slew-rate and acceleration shaping between what the
    // algorithm asked for and what actually went on the wire.
    drawIntentGhost(intent, project);
    drawIntentRibbon(intent, project, colors);
    drawIntentTargets(intent, project, colors);

    ctx.restore();
  }

  function drawIntentRibbon(intent, project, colors) {
    const pts = intent.path || [];
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

  function drawIntentGhost(intent, project) {
    const pts = intent.commanded_path || [];
    if (polylineLength(pts) < 0.05) return;
    ctx.save();
    ctx.setLineDash([5, 5]);
    ctx.strokeStyle = 'rgba(230, 237, 243, 0.55)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    pts.forEach((p, i) => {
      const [cx, cy] = project(p.x, p.y);
      if (i === 0) ctx.moveTo(cx, cy); else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
    ctx.restore();
  }

  function drawIntentTargets(intent, project, colors) {
    (intent.targets || []).forEach((t) => {
      const [cx, cy] = project(t.x, t.y);
      ctx.beginPath();
      ctx.arc(cx, cy, 5, 0, Math.PI * 2);
      ctx.fillStyle = colors.edge;
      ctx.fill();
      ctx.font = '11px sans-serif';
      ctx.fillStyle = 'rgba(230, 237, 243, 0.75)';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(t.kind || '').replace(/_/g, ' '), cx + 9, cy);
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
    ctx.fillStyle = 'rgba(88, 166, 255, 0.10)';
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
    } else {
      infoCpu.textContent = infoMem.textContent = infoTemp.textContent = infoUptime.textContent = '--';
      infoWifiText.textContent = '--';
      updateWifiBars(null);
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
    scheduleRender();
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
    scheduleRender();
  }, { passive: false });

  resetViewBtn.addEventListener('click', () => {
    view.userAdjusted = false;
    view.bodyPanX = 0;
    view.bodyPanY = 0;
    maybeAutoFit();
    scheduleRender();
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

  function sendTuningSet(node, name, value) {
    sendTuningControl({ action: 'set', node, name, value });
  }

  // A signature of the panel's *shape* only -- which nodes are up and
  // which parameters they offer. Values deliberately excluded, so the
  // 2-second value refresh never triggers a rebuild.
  function structureSignature(nodes) {
    return JSON.stringify(nodes.map((n) => [
      n.node, n.online, n.error || '', n.savable,
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
    tuningSummary.textContent = onlineNodes.length
      ? `${onlineNodes.map((n) => n.node.replace(/_node$/, '')).join(', ')} tunable`
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

    usable.forEach((node) => {
      const title = document.createElement('div');
      title.className = 'tuning-node-title';
      title.textContent = node.node;
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
    const overlayRight = overlay.getBoundingClientRect().right;
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

    setDigest('intent', state.intent
      ? String(state.intent.state || '').replace(/_/g, ' ')
        + (intentAgeMs() > INTENT_STALE_MS ? ' (stale)' : '')
      : '--');

    setDigest('vehicle', state.speed && !isStale(state.speed)
      ? `${state.speed.speed.toFixed(1)} m/s` : '--');

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
  // Go
  // ---------------------------------------------------------------------
  resizeCanvasIfNeeded();
  connect();
  render();
})();
