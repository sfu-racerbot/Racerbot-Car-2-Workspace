/*
 * Runs the real web/dashboard.js against a stubbed DOM and feeds it the
 * exact frames web_dashboard/protocol.py + mapstream.py produce.
 *
 * The map is the one place in the browser where a bug does not look like a
 * bug. A patch applied at the wrong offset, or without the bottom-up ->
 * top-down row flip, still paints something map-shaped; you would only
 * notice by staring at a wall that had moved. So the check here is an
 * invariant rather than a screenshot:
 *
 *     keyframe(A) + patches(A -> B)  must be pixel-identical to  keyframe(B)
 *
 * That compares the incremental path against the full-redraw path with no
 * duplicated palette constants and no golden image to keep up to date -- if
 * the two ever disagree, one of them is wrong.
 *
 * Run directly with `node map_decode_test.js`, or through pytest via
 * test_dashboard_js.py.
 */
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');
const zlib = require('zlib');

const DASHBOARD_JS = path.join(__dirname, '..', '..', 'web', 'dashboard.js');

// ---------------------------------------------------------------------------
// A DOM stub: just enough for dashboard.js's IIFE to run start to finish.
// Canvases are real enough to hold pixels, because those are what we assert
// on; everything else is a no-op that must merely not throw.
// ---------------------------------------------------------------------------

function makeCanvas() {
  const canvas = {
    _pixels: null,
    _w: 0,
    _h: 0,
    style: {},
    classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
    addEventListener() {},
    getBoundingClientRect() { return { width: 100, height: 100, right: 100, bottom: 100 }; },
  };
  Object.defineProperty(canvas, 'width', {
    get() { return canvas._w; },
    set(v) { canvas._w = v; canvas._pixels = new Uint8ClampedArray(v * canvas._h * 4); },
  });
  Object.defineProperty(canvas, 'height', {
    get() { return canvas._h; },
    set(v) { canvas._h = v; canvas._pixels = new Uint8ClampedArray(canvas._w * v * 4); },
  });
  canvas.getContext = () => makeContext(canvas);
  return canvas;
}

function makeContext(canvas) {
  const real = {
    canvas,
    createImageData(w, h) {
      return { width: w, height: h, data: new Uint8ClampedArray(w * h * 4) };
    },
    putImageData(img, dx, dy) {
      // Composite into the canvas's own pixel store, which is what the
      // assertions read back.
      const target = canvas._pixels;
      if (!target) return;
      for (let row = 0; row < img.height; row++) {
        const y = dy + row;
        if (y < 0 || y >= canvas._h) continue;
        for (let col = 0; col < img.width; col++) {
          const x = dx + col;
          if (x < 0 || x >= canvas._w) continue;
          const src = (row * img.width + col) * 4;
          const dst = (y * canvas._w + x) * 4;
          target[dst] = img.data[src];
          target[dst + 1] = img.data[src + 1];
          target[dst + 2] = img.data[src + 2];
          target[dst + 3] = img.data[src + 3];
        }
      }
    },
    measureText() { return { width: 0 }; },
    save() {}, restore() {},
  };
  // Anything else a 2d context is asked for is a no-op; assignments (fillStyle
  // and friends) just stick.
  return new Proxy(real, {
    get(target, key) {
      if (key in target) return target[key];
      return () => {};
    },
    set(target, key, value) { target[key] = value; return true; },
  });
}

function makeElement() {
  const element = {
    style: {},
    dataset: {},
    classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
    children: [],
    textContent: '',
    innerHTML: '',
    checked: false,
    value: '',
    disabled: false,
    addEventListener() {},
    removeEventListener() {},
    appendChild(child) { this.children.push(child); return child; },
    setAttribute() {}, removeAttribute() {}, getAttribute() { return null; },
    querySelectorAll() { return []; },
    querySelector() { return null; },
    getBoundingClientRect() {
      return { width: 100, height: 100, top: 0, left: 0, right: 100, bottom: 100 };
    },
    focus() {}, blur() {},
    setPointerCapture() {}, releasePointerCapture() {},
    hasPointerCapture() { return false; },
  };
  return element;
}

function buildSandbox(sockets, canvases, warnings) {
  const mainCanvas = makeCanvas();
  mainCanvas.width = 800;
  mainCanvas.height = 600;

  const document = {
    getElementById(id) {
      return id === 'view' || id === 'minimap' ? mainCanvas : makeElement();
    },
    querySelectorAll() { return []; },
    querySelector() { return null; },
    createElement(tag) {
      if (tag === 'canvas') {
        const canvas = makeCanvas();
        canvases.push(canvas);
        return canvas;
      }
      return makeElement();
    },
    addEventListener() {},
    activeElement: null,
    body: makeElement(),
  };

  class FakeWebSocket {
    constructor(url) {
      this.url = url;
      this.readyState = 1;
      this.binaryType = '';
      sockets.push(this);
      setTimeout(() => { if (this.onopen) this.onopen(); }, 0);
    }
    send() {}
    close() {}
  }
  FakeWebSocket.OPEN = 1;

  const sandbox = {
    document,
    WebSocket: FakeWebSocket,
    location: { host: 'car:8080', hostname: 'car' },
    localStorage: { getItem() { return null; }, setItem() {}, removeItem() {} },
    performance: { now: () => Date.now() },
    requestAnimationFrame: (fn) => setTimeout(fn, 0),
    setTimeout,
    clearTimeout,
    setInterval: () => 0,
    clearInterval,
    addEventListener() {},
    removeEventListener() {},
    innerWidth: 1280,
    innerHeight: 800,
    devicePixelRatio: 1,
    getComputedStyle() { return { right: '12px', bottom: '47px' }; },
    confirm() { return false; },
    // dashboard.js reports every dropped/undecodable frame through
    // console.warn (noteDesync) or console.error, so capturing these is how
    // a test asserts "this frame was accepted cleanly" rather than just
    // "nothing threw".
    console: {
      log: () => {},
      warn: (...args) => { warnings.push(args.join(' ')); },
      error: (...args) => { warnings.push(args.join(' ')); },
    },
    Date,
    Math,
    JSON,
    // Present in node 18+, and what dashboard.js uses to inflate the map.
    DecompressionStream,
    Blob,
    Response,
    Uint8Array,
    Uint16Array,
    Uint32Array,
    Int8Array,
    Float32Array,
    ArrayBuffer,
    Promise,
    Number,
    String,
    Object,
    Array,
    Error,
  };
  sandbox.window = sandbox;
  sandbox.globalThis = sandbox;
  return { sandbox, mainCanvas };
}

// ---------------------------------------------------------------------------
// One browser: dashboard.js loaded, with a way to push frames at it.
// ---------------------------------------------------------------------------

class Browser {
  constructor() {
    this.sockets = [];
    this.canvases = [];
    this.warnings = [];
    const { sandbox } = buildSandbox(this.sockets, this.canvases, this.warnings);
    this.sandbox = sandbox;
    vm.createContext(sandbox);
    vm.runInContext(fs.readFileSync(DASHBOARD_JS, 'utf8'), sandbox,
                    { filename: 'dashboard.js' });
    if (!this.sockets.length) throw new Error('dashboard.js never opened a WebSocket');
    this.ws = this.sockets[0];
  }

  header(obj) {
    this.ws.onmessage({ data: JSON.stringify(obj) });
  }

  binary(bytes) {
    const copy = new Uint8Array(bytes);
    this.ws.onmessage({ data: copy.buffer });
  }

  frame(header, payload) {
    this.header(header);
    this.binary(payload);
  }

  settle() {
    // Map frames are decoded through an async promise chain; give it room.
    return new Promise((resolve) => setTimeout(resolve, 60));
  }

  // The offscreen canvas dashboard.js built for the map.
  get mapPixels() {
    const mapCanvas = this.canvases[this.canvases.length - 1];
    return mapCanvas ? mapCanvas._pixels : null;
  }
}

// ---------------------------------------------------------------------------
// Frames, built the way the server builds them.
// ---------------------------------------------------------------------------

const WIDTH = 37;   // deliberately not round, and not a multiple of anything
const HEIGHT = 23;

function keyframe(cells, seq, { compress = true } = {}) {
  const raw = Buffer.from(cells);
  const payload = compress ? zlib.deflateSync(raw) : raw;
  return [{
    type: 'map', seq,
    width: WIDTH, height: HEIGHT, resolution: 0.05,
    origin_x: -1.0, origin_y: -2.0, origin_yaw: 0.0,
    encoding: compress ? 'deflate' : 'raw',
    bytes: payload.length, raw_bytes: raw.length,
    stamp: 0,
  }, payload];
}

function patch(cells, x, y, w, h, seq) {
  const rows = [];
  for (let row = 0; row < h; row++) {
    const start = (y + row) * WIDTH + x;
    rows.push(Buffer.from(cells.slice(start, start + w)));
  }
  const raw = Buffer.concat(rows);
  const payload = zlib.deflateSync(raw);
  return [{
    type: 'map_patch', seq, x, y, w, h,
    encoding: 'deflate', bytes: payload.length, raw_bytes: raw.length,
    stamp: 0,
  }, payload];
}

function blankGrid() {
  return new Uint8Array(WIDTH * HEIGHT).fill(0xFF); // -1, unknown
}

function paintRect(cells, x, y, w, h, value) {
  const out = Uint8Array.from(cells);
  for (let row = y; row < y + h; row++) {
    for (let col = x; col < x + w; col++) out[row * WIDTH + col] = value;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Checks
// ---------------------------------------------------------------------------

const results = [];
function check(label, ok, detail = '') {
  results.push({ label, ok, detail });
  console.log(`  ${ok ? 'ok  ' : 'FAIL'}  ${label}${ok || !detail ? '' : '  -- ' + detail}`);
}

function samePixels(a, b) {
  if (!a || !b) return false;
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
  return true;
}

function firstPixelDifference(a, b) {
  for (let i = 0; i < a.length; i += 4) {
    if (a[i] !== b[i] || a[i + 1] !== b[i + 1] || a[i + 2] !== b[i + 2]
        || a[i + 3] !== b[i + 3]) {
      const px = i / 4;
      return `pixel (${px % WIDTH}, ${Math.floor(px / WIDTH)})`;
    }
  }
  return 'none';
}

async function main() {
  console.log('map: patches must reconstruct exactly what a keyframe would draw');

  // The reference: one keyframe of the final grid.
  let grid = blankGrid();
  grid = paintRect(grid, 5, 3, 6, 4, 100);
  grid = paintRect(grid, 20, 14, 4, 3, 0);
  grid = paintRect(grid, WIDTH - 2, HEIGHT - 2, 2, 2, 100);  // touches both far edges
  grid = paintRect(grid, 0, 0, 2, 2, 50);                    // and both near edges

  const reference = new Browser();
  reference.frame(...keyframe(grid, 1));
  await reference.settle();
  const expected = reference.mapPixels;
  check('a keyframe paints the map at all',
        expected && expected.some((v) => v !== 0));

  // The incremental path: blank keyframe, then the same regions as patches.
  const incremental = new Browser();
  let building = blankGrid();
  incremental.frame(...keyframe(building, 1));
  await incremental.settle();

  const steps = [
    [5, 3, 6, 4, 100],
    [20, 14, 4, 3, 0],
    [WIDTH - 2, HEIGHT - 2, 2, 2, 100],
    [0, 0, 2, 2, 50],
  ];
  let seq = 1;
  for (const [x, y, w, h, value] of steps) {
    building = paintRect(building, x, y, w, h, value);
    seq += 1;
    incremental.frame(...patch(building, x, y, w, h, seq));
    await incremental.settle();
  }

  const got = incremental.mapPixels;
  check('keyframe + patches is pixel-identical to a full keyframe',
        samePixels(expected, got),
        expected && got ? `first difference at ${firstPixelDifference(expected, got)}`
                        : 'no pixels captured');

  // Row flip: a patch in the bottom-left of the GRID must land in the
  // bottom-left of the IMAGE, not the top-left. An unflipped patch passes
  // every length check and still puts the wall in the wrong place.
  console.log('map: the bottom-up to top-down row flip');
  const flipRef = new Browser();
  const bottomLeft = paintRect(blankGrid(), 0, 0, 3, 3, 100);
  flipRef.frame(...keyframe(bottomLeft, 1));
  await flipRef.settle();

  const flipInc = new Browser();
  flipInc.frame(...keyframe(blankGrid(), 1));
  await flipInc.settle();
  flipInc.frame(...patch(bottomLeft, 0, 0, 3, 3, 2));
  await flipInc.settle();
  check('a patch at grid row 0 lands where a keyframe puts grid row 0',
        samePixels(flipRef.mapPixels, flipInc.mapPixels),
        'a missing row flip would put it at the top of the image');

  // Out-of-order patches must be refused, not applied.
  console.log('map: a patch out of sequence is refused rather than applied');
  const gap = new Browser();
  gap.frame(...keyframe(blankGrid(), 1));
  await gap.settle();
  const before = Uint8ClampedArray.from(gap.mapPixels);
  const skipped = paintRect(blankGrid(), 8, 8, 4, 4, 100);
  gap.frame(...patch(skipped, 8, 8, 4, 4, 7));   // seq 7 after seq 1
  await gap.settle();
  check('a patch that does not follow the last frame changes nothing',
        samePixels(before, gap.mapPixels),
        'it was applied anyway, which would leave the map quietly wrong');

  // ... and a later keyframe recovers.
  gap.frame(...keyframe(skipped, 8));
  await gap.settle();
  const recovered = new Browser();
  recovered.frame(...keyframe(skipped, 1));
  await recovered.settle();
  check('a keyframe afterwards restores the correct map',
        samePixels(recovered.mapPixels, gap.mapPixels));

  // Uncompressed frames must work too -- that is the map_compression:false
  // escape hatch, and the fallback for a browser with no DecompressionStream.
  console.log('map: uncompressed frames');
  const rawBrowser = new Browser();
  rawBrowser.frame(...keyframe(grid, 1, { compress: false }));
  await rawBrowser.settle();
  check('a raw keyframe paints the same picture as a deflated one',
        samePixels(expected, rawBrowser.mapPixels));

  // Scans: u16mm must decode to the same metres float32 carried.
  console.log('scan: uint16 millimetres');
  const scanBrowser = new Browser();
  const metres = [0.5, 1.2345, 9.999, 0];
  const mm = new Uint16Array(metres.map((m) => Math.round(m * 1000)));
  scanBrowser.header({
    type: 'scan', encoding: 'u16mm', bytes: mm.byteLength,
    angle_min: -1, angle_increment: 0.01, range_min: 0.1, range_max: 10,
    count: metres.length, laser_offset_x: 0.33, laser_offset_y: 0, stamp: 0,
  });
  scanBrowser.binary(new Uint8Array(mm.buffer));
  await scanBrowser.settle();
  check('a u16mm scan is accepted without a desync',
        scanBrowser.warnings.length === 0, scanBrowser.warnings.join('; '));

  // A float32 scan must still decode -- scan_encoding is a parameter, and
  // 'f32' is what an older or reconfigured car sends.
  const f32Browser = new Browser();
  const floats = new Float32Array(metres);
  f32Browser.header({
    type: 'scan', encoding: 'f32', bytes: floats.byteLength,
    angle_min: -1, angle_increment: 0.01, range_min: 0.1, range_max: 10,
    count: metres.length, laser_offset_x: 0.33, laser_offset_y: 0, stamp: 0,
  });
  f32Browser.binary(new Uint8Array(floats.buffer));
  await f32Browser.settle();
  check('a float32 scan still decodes',
        f32Browser.warnings.length === 0, f32Browser.warnings.join('; '));

  // A payload whose length contradicts its header is exactly the failure
  // the `bytes` field exists to catch.
  const badBrowser = new Browser();
  badBrowser.header({
    type: 'scan', encoding: 'u16mm', bytes: 999,
    angle_min: -1, angle_increment: 0.01, range_min: 0.1, range_max: 10,
    count: metres.length, laser_offset_x: 0.33, laser_offset_y: 0, stamp: 0,
  });
  badBrowser.binary(new Uint8Array(mm.buffer));
  await badBrowser.settle();
  check('a payload that contradicts its header is reported, not drawn',
        badBrowser.warnings.some((w) => w.includes('dropped a binary frame')),
        badBrowser.warnings.join('; '));

  // Batched telemetry must be unwrapped and applied.
  console.log('telemetry: batch frames');
  const batchBrowser = new Browser();
  batchBrowser.header({
    type: 'batch',
    items: [
      { type: 'pose', x: 1.5, y: -2.5, yaw: 0.25, stamp: 0 },
      { type: 'speed', speed: 2.75, stamp: 0 },
      { type: 'drive', speed: 2.8, steering_angle: 0.1, stamp: 0 },
      { type: 'stats', cpu_percent: 30, mem_percent: 50, cpu_temp_c: 51,
        uptime_s: 100, wifi_dbm: -55, stamp: 0 },
      { type: 'intent', intent: { state: 'racing', severity: 'drive',
        node: 'pure_pursuit_node', desired_speed: 3, commanded_speed: 2.8,
        desired_steering: 0.1, commanded_steering: 0.09, path: [], factors: [] },
        stamp: 0 },
    ],
  });
  await batchBrowser.settle();
  check('a batch frame is unwrapped and applied cleanly',
        batchBrowser.warnings.length === 0, batchBrowser.warnings.join('; '));

  // A batch must not disturb the header/binary pairing that map and scan
  // frames rely on: a batch arriving between a header and its payload is
  // normal traffic, not a desync.
  const interleaved = new Browser();
  interleaved.frame(...keyframe(grid, 1));
  await interleaved.settle();
  interleaved.header({ type: 'batch', items: [
    { type: 'pose', x: 0, y: 0, yaw: 0, stamp: 0 }] });
  interleaved.frame(...patch(paintRect(grid, 9, 9, 2, 2, 100), 9, 9, 2, 2, 2));
  await interleaved.settle();
  check('a batch between map frames does not break the pairing',
        interleaved.warnings.length === 0, interleaved.warnings.join('; '));

  console.log('');
  const failures = results.filter((r) => !r.ok);
  if (failures.length) {
    console.log(`FAILED: ${failures.length} of ${results.length} checks`);
    process.exit(1);
  }
  console.log(`All ${results.length} checks passed.`);
}

main().catch((err) => {
  console.error('test harness error:', err);
  process.exit(2);
});
