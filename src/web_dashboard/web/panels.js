/*
 * panels.js -- the dashboard's window manager.
 *
 * Three things live here, and nothing else:
 *
 *   1. DETACHING. Any section of the info panel can be pulled out into its
 *      own floating panel, and pushed back in. The section's own <details>
 *      element is MOVED, not copied or re-rendered -- same DOM node, same
 *      ids, same children. That is the whole trick that keeps this file
 *      independent of dashboard.js: dashboard.js goes on writing to
 *      `#vehicle-speed` by id and neither file has to know the element is
 *      now living somewhere else on the page.
 *
 *   2. GEOMETRY. Dragging, magnetic snapping, and 8-way resizing, for
 *      floating panels and for the fixed ones (info panel, minimap).
 *
 *   3. LAYOUT STATE. Where everything is, persisted per browser, plus the
 *      rule that a window too narrow to float panels in re-docks them all
 *      without losing the layout you set up on a big screen.
 *
 *   4. THE PHONE SHEET. Below the phone breakpoint the info panel stops
 *      being a sidebar and becomes a sheet parked off the bottom of the
 *      screen, with three detents you drag it between. That is a layout
 *      state too, and it is the same element -- so it lives here rather
 *      than in a fifth file that would also want to move #overlay.
 *
 * Deliberately NOT in here: anything that reads telemetry or touches the
 * WebSocket. This file only ever moves boxes around.
 *
 * The pure geometry -- snapping, clamping, the layout round-trip -- is
 * exported on `window.__panels` so test/browser/panels_test.js can check
 * it under node with no browser. Everything else needs a real DOM.
 */
'use strict';

(function () {
  const STORE_KEY = 'racerbot.dashboard.layout.v1';

  // Distance in CSS pixels within which a dragged edge jumps flush to
  // another edge. Tuned by feel: below ~8 you have to aim, above ~16 the
  // panel fights you when you genuinely want it slightly offset.
  const SNAP_PX = 12;

  // Below this the screen cannot usefully hold floating panels, so they
  // all go home. Chosen to be wider than the info panel (320) plus a
  // reasonable panel (280) plus the right rail (220) plus margins.
  const FLOAT_MIN_WIDTH = 900;

  const MIN_W = 180;
  const MIN_H = 90;

  // The phone breakpoint. MUST match the `max-width: 640px` media query in
  // style.css: that query decides whether the sheet styling applies at
  // all, and this decides whether the gestures that drive it are live. If
  // they ever disagree there is a window width with a sheet nobody can
  // open, or a grip that drags a sidebar.
  const PHONE_MAX_WIDTH = 640;

  // Where the middle detent sits, as a fraction of the sheet's own height.
  // MUST match `body.sheet-half #overlay { transform: translateY(46%) }`.
  const SHEET_HALF_FRACTION = 0.46;

  // Above this a drag counts as a flick and moves a detent in the
  // direction it was thrown, however far it actually travelled. In CSS
  // pixels per millisecond -- about a fast thumb flick, well clear of the
  // speed a careful drag reaches.
  const SHEET_FLICK_PX_PER_MS = 0.5;

  // Past this a gesture is a drag, below it a tap. 3px absorbs the wobble
  // of a thumb pressing a handle without moving it.
  const SHEET_TAP_SLOP_PX = 3;

  // ---------------------------------------------------------------------
  // Pure geometry. No DOM, no state -- these are the parts worth testing.
  // ---------------------------------------------------------------------

  /** Clamp a box to the viewport, keeping at least `keep` px reachable. */
  function clampToViewport(box, vw, vh, keep = 48) {
    const w = Math.max(MIN_W, Math.min(box.w, vw));
    const h = Math.max(MIN_H, Math.min(box.h, vh));
    return {
      x: Math.max(keep - w, Math.min(box.x, vw - keep)),
      y: Math.max(0, Math.min(box.y, vh - Math.min(keep, h))),
      w,
      h,
    };
  }

  /**
   * Magnetic snap for one axis.
   *
   * `moving` is [start, end] of the dragged box on this axis; each entry
   * in `others` is [start, end] of another box. A dragged edge snaps to
   * an opposing edge (flush adjacency: my left to your right) or to the
   * matching edge (alignment: my left to your left). Returns the offset
   * to add to `moving`, or 0.
   *
   * Both kinds matter and they are different intentions: adjacency is
   * "put these side by side", alignment is "line these up".
   */
  function snapAxis(moving, others, threshold = SNAP_PX) {
    const [lo, hi] = moving;
    let best = 0;
    let bestDist = threshold + 1;

    const consider = (delta) => {
      const dist = Math.abs(delta);
      if (dist <= threshold && dist < bestDist) {
        bestDist = dist;
        best = delta;
      }
    };

    for (const [olo, ohi] of others) {
      consider(ohi - lo);   // my start   -> your end    (flush after you)
      consider(olo - hi);   // my end     -> your start  (flush before you)
      consider(olo - lo);   // my start   -> your start  (aligned)
      consider(ohi - hi);   // my end     -> your end    (aligned)
    }
    return bestDist <= threshold ? best : 0;
  }

  /** Snap a whole box against other boxes and the viewport edges. */
  function snapBox(box, others, vw, vh, threshold = SNAP_PX) {
    const xs = others.map((o) => [o.x, o.x + o.w]);
    const ys = others.map((o) => [o.y, o.y + o.h]);
    xs.push([0, vw]);
    ys.push([0, vh]);
    return {
      ...box,
      x: box.x + snapAxis([box.x, box.x + box.w], xs, threshold),
      y: box.y + snapAxis([box.y, box.y + box.h], ys, threshold),
    };
  }

  /**
   * Apply an 8-way resize. `edge` is any combination of n/s/e/w.
   * The opposite side stays put, and the box never inverts.
   */
  function resizeBox(box, edge, dx, dy) {
    let { x, y, w, h } = box;
    if (edge.includes('e')) w = Math.max(MIN_W, box.w + dx);
    if (edge.includes('s')) h = Math.max(MIN_H, box.h + dy);
    if (edge.includes('w')) {
      w = Math.max(MIN_W, box.w - dx);
      x = box.x + (box.w - w);
    }
    if (edge.includes('n')) {
      h = Math.max(MIN_H, box.h - dy);
      y = box.y + (box.h - h);
    }
    return { x, y, w, h };
  }

  /**
   * Where each sheet detent sits, as a downward translation in pixels
   * from fully open. `height` is the sheet's own height and `peek` is how
   * much of it stays on screen when closed.
   */
  function sheetOffsets(height, peek) {
    return {
      full: 0,
      half: height * SHEET_HALF_FRACTION,
      peek: Math.max(0, height - peek),
    };
  }

  /**
   * Which detent a drag ending at `offset` should settle into.
   *
   * Nearest wins, except that a flick always moves at least one detent in
   * the direction it was thrown -- that is what makes a short fast gesture
   * feel like it did something, instead of snapping back to where it
   * started because the finger never travelled far enough.
   *
   * `velocity` is CSS pixels per millisecond, positive downwards (closing).
   * The detents are sorted by offset rather than assumed to be in a fixed
   * order: on a short screen `peek` and `half` can cross over, and a
   * hard-coded order would then step the sheet the wrong way.
   */
  function detentFor(offset, height, peek, velocity = 0) {
    const sorted = Object.entries(sheetOffsets(height, peek))
      .sort((a, b) => a[1] - b[1]);

    let nearest = 0;
    for (let i = 1; i < sorted.length; i++) {
      if (Math.abs(sorted[i][1] - offset) < Math.abs(sorted[nearest][1] - offset)) {
        nearest = i;
      }
    }

    if (Math.abs(velocity) > SHEET_FLICK_PX_PER_MS) {
      const step = velocity > 0 ? 1 : -1;
      nearest = Math.max(0, Math.min(sorted.length - 1, nearest + step));
    }
    return sorted[nearest][0];
  }

  /** Layout state -> a plain object safe to JSON round-trip. */
  function serializeLayout(state) {
    const out = { sections: {}, panels: {} };
    for (const [name, entry] of Object.entries(state.sections)) {
      out.sections[name] = {
        floating: !!entry.floating,
        box: entry.box ? { ...entry.box } : null,
      };
    }
    for (const [name, box] of Object.entries(state.panels)) {
      out.panels[name] = box ? { ...box } : null;
    }
    return out;
  }

  /** The inverse, tolerant of anything an older/newer build wrote. */
  function deserializeLayout(raw) {
    const state = { sections: {}, panels: {} };
    if (!raw || typeof raw !== 'object') return state;
    const sections = raw.sections && typeof raw.sections === 'object' ? raw.sections : {};
    for (const [name, entry] of Object.entries(sections)) {
      if (!entry || typeof entry !== 'object') continue;
      const box = entry.box;
      const ok = box && ['x', 'y', 'w', 'h'].every((k) => typeof box[k] === 'number' && isFinite(box[k]));
      state.sections[name] = { floating: !!entry.floating, box: ok ? { ...box } : null };
    }
    const panels = raw.panels && typeof raw.panels === 'object' ? raw.panels : {};
    for (const [name, box] of Object.entries(panels)) {
      const ok = box && ['x', 'y', 'w', 'h'].every((k) => typeof box[k] === 'number' && isFinite(box[k]));
      if (ok) state.panels[name] = { ...box };
    }
    return state;
  }

  // Exported for the node test. Assigned unconditionally so the test can
  // load this file with a DOM stub and still reach the maths.
  const api = {
    SNAP_PX, FLOAT_MIN_WIDTH, MIN_W, MIN_H,
    PHONE_MAX_WIDTH, SHEET_HALF_FRACTION, SHEET_FLICK_PX_PER_MS,
    clampToViewport, snapAxis, snapBox, resizeBox,
    sheetOffsets, detentFor,
    serializeLayout, deserializeLayout,
  };
  if (typeof window !== 'undefined') window.__panels = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;

  // Everything below needs a real document. The node test stops here.
  if (typeof document === 'undefined' || !document.getElementById) return;

  // ---------------------------------------------------------------------
  // DOM wiring
  // ---------------------------------------------------------------------
  const overlay = document.getElementById('overlay');
  const panelsHost = document.getElementById('panels');
  const tuningPanel = document.getElementById('tuning-panel');
  if (!overlay || !panelsHost) return;

  const EDGES = ['n', 's', 'e', 'w', 'ne', 'nw', 'se', 'sw'];

  // Floating panels all sit on layer 11 in the stylesheet; touching one
  // lifts it within that layer, so an overlapping pair behaves the way
  // every other window does rather than by document order.
  let floatZ = 11;

  function raise(el) {
    floatZ += 1;
    el.style.zIndex = String(floatZ);
  }

  const state = { sections: {}, panels: {} };
  const floatEls = {};      // section name -> its floating wrapper
  const sectionEls = {};    // section name -> the <details> element
  const dockOrder = [];     // the order sections appear in when docked

  for (const el of panelsHost.querySelectorAll('[data-section]')) {
    const name = el.getAttribute('data-section');
    sectionEls[name] = el;
    dockOrder.push(name);
    state.sections[name] = { floating: false, box: null };
  }

  const titleOf = (name) => {
    const el = sectionEls[name];
    const label = el && el.querySelector('.section-name');
    return label ? label.textContent : name;
  };

  // --- persistence ---------------------------------------------------
  function save() {
    try {
      window.localStorage.setItem(STORE_KEY, JSON.stringify(serializeLayout(state)));
    } catch (err) {
      /* private browsing, quota, disabled storage: layout is a nicety */
    }
  }

  function load() {
    try {
      const raw = window.localStorage.getItem(STORE_KEY);
      if (!raw) return null;
      return deserializeLayout(JSON.parse(raw));
    } catch (err) {
      return null;
    }
  }

  // --- geometry helpers ----------------------------------------------
  const viewport = () => ({ vw: window.innerWidth, vh: window.innerHeight });

  function boxOf(el) {
    const r = el.getBoundingClientRect();
    return { x: r.left, y: r.top, w: r.width, h: r.height };
  }

  function applyBox(el, box) {
    el.style.left = `${Math.round(box.x)}px`;
    el.style.top = `${Math.round(box.y)}px`;
    el.style.width = `${Math.round(box.w)}px`;
    el.style.height = `${Math.round(box.h)}px`;
    el.style.right = 'auto';
    el.style.bottom = 'auto';
    el.style.maxHeight = 'none';
  }

  /** Every managed box except `exceptEl`, for snapping against. */
  function peerBoxes(exceptEl) {
    const out = [];
    for (const el of managedElements()) {
      if (el === exceptEl || !el.isConnected) continue;
      const box = boxOf(el);
      // Zero-sized means display:none. Emphatically NOT an offsetParent
      // check: offsetParent is null for every position:fixed element, and
      // every panel here is fixed -- so that test silently excluded all of
      // them, which quietly turned off panel-to-panel snapping and left
      // new panels landing on top of existing ones.
      if (box.w === 0 && box.h === 0) continue;
      out.push(box);
    }
    return out;
  }

  function managedElements() {
    const els = [overlay];
    const minimap = document.getElementById('minimap-panel');
    const camera = document.getElementById('camera-panel');
    if (minimap) els.push(minimap);
    if (camera) els.push(camera);
    for (const el of Object.values(floatEls)) els.push(el);
    return els;
  }

  // The tuning drawer docks against the info panel's right edge; keep the
  // custom property style.css reads in step with where that edge is now.
  function syncTuningOffset() {
    if (!tuningPanel) return;
    const r = overlay.getBoundingClientRect();
    document.documentElement.style.setProperty(
      '--tuning-left', `${Math.round(r.right + 12)}px`);
  }

  // --- snap guides ----------------------------------------------------
  const guides = { x: null, y: null };

  function showGuide(axis, pos) {
    if (!guides[axis]) {
      const g = document.createElement('div');
      g.className = `snap-guide snap-guide-${axis}`;
      document.body.appendChild(g);
      guides[axis] = g;
    }
    const g = guides[axis];
    g.style.display = 'block';
    if (axis === 'x') g.style.left = `${Math.round(pos)}px`;
    else g.style.top = `${Math.round(pos)}px`;
  }

  function hideGuides() {
    for (const axis of ['x', 'y']) {
      if (guides[axis]) guides[axis].style.display = 'none';
    }
  }

  // --- drag + resize ---------------------------------------------------
  function beginDrag(el, event, onCommit) {
    const start = boxOf(el);
    const originX = event.clientX;
    const originY = event.clientY;
    el.classList.add('panel-dragging');

    const move = (e) => {
      const { vw, vh } = viewport();
      const raw = { ...start, x: start.x + (e.clientX - originX), y: start.y + (e.clientY - originY) };
      const snapped = snapBox(raw, peerBoxes(el), vw, vh);
      // Guides only where the snap actually moved us, so they mark a
      // decision rather than decorating every drag.
      if (snapped.x !== raw.x) showGuide('x', snapped.x <= raw.x ? snapped.x : snapped.x + snapped.w);
      else if (guides.x) guides.x.style.display = 'none';
      if (snapped.y !== raw.y) showGuide('y', snapped.y <= raw.y ? snapped.y : snapped.y + snapped.h);
      else if (guides.y) guides.y.style.display = 'none';
      applyBox(el, clampToViewport(snapped, vw, vh));
      syncTuningOffset();
    };

    const up = () => {
      window.removeEventListener('pointermove', move);
      window.removeEventListener('pointerup', up);
      el.classList.remove('panel-dragging');
      hideGuides();
      onCommit(boxOf(el));
    };

    window.addEventListener('pointermove', move);
    window.addEventListener('pointerup', up);
  }

  function beginResize(el, edge, event, onCommit) {
    const start = boxOf(el);
    const originX = event.clientX;
    const originY = event.clientY;
    el.classList.add('panel-resizing');

    const move = (e) => {
      const { vw, vh } = viewport();
      const next = resizeBox(start, edge, e.clientX - originX, e.clientY - originY);
      applyBox(el, clampToViewport(next, vw, vh));
      syncTuningOffset();
    };

    const up = () => {
      window.removeEventListener('pointermove', move);
      window.removeEventListener('pointerup', up);
      el.classList.remove('panel-resizing');
      onCommit(boxOf(el));
    };

    window.addEventListener('pointermove', move);
    window.addEventListener('pointerup', up);
  }

  /** Give an element the eight resize grips. */
  function addResizeHandles(el, onCommit) {
    for (const edge of EDGES) {
      const handle = document.createElement('div');
      handle.className = `panel-resize panel-resize-${edge}`;
      handle.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        e.stopPropagation();
        beginResize(el, edge, e, onCommit);
      });
      el.appendChild(handle);
    }
  }

  // --- detaching -------------------------------------------------------
  function overlapArea(a, b) {
    const w = Math.min(a.x + a.w, b.x + b.w) - Math.max(a.x, b.x);
    const h = Math.min(a.y + a.h, b.y + b.h) - Math.max(a.y, b.y);
    return w > 0 && h > 0 ? w * h : 0;
  }

  /**
   * Where to put a newly detached panel.
   *
   * A fixed cascade put every panel in the same strip just right of the
   * info panel -- which is also exactly where the tuning drawer opens, so
   * two detached sections and an open drawer ended up in a stack nobody
   * could read. Instead: walk a coarse grid across the free middle of the
   * screen and take the first slot that overlaps nothing, falling back to
   * whichever slot overlaps least.
   */
  function placementFor(w, h, selfEl) {
    const { vw, vh } = viewport();
    // Excluding the panel being placed: it is already in floatEls by this
    // point, and counting its own current box as an obstacle would make
    // every candidate look occupied.
    const taken = peerBoxes(selfEl);
    const left = Math.round(overlay.getBoundingClientRect().right + 12);
    const stepX = 40;
    const stepY = 36;

    let best = { x: left, y: 12 };
    let bestScore = Infinity;

    for (let y = 12; y + h <= vh - 12; y += stepY) {
      for (let x = left; x + w <= vw - 12; x += stepX) {
        const candidate = { x, y, w, h };
        let score = 0;
        for (const other of taken) score += overlapArea(candidate, other);
        if (score === 0) return candidate;
        if (score < bestScore) {
          bestScore = score;
          best = { x, y };
        }
      }
    }
    return { x: best.x, y: best.y, w, h };
  }

  function detach(name, box) {
    const section = sectionEls[name];
    if (!section || floatEls[name]) return;

    const wrap = document.createElement('div');
    wrap.className = 'float-panel hud-panel';

    const bar = document.createElement('div');
    bar.className = 'float-bar';

    const title = document.createElement('span');
    title.className = 'float-title';
    title.textContent = titleOf(name);

    const dock = document.createElement('button');
    dock.className = 'float-dock';
    dock.type = 'button';
    dock.title = 'Return this section to the info panel';
    dock.setAttribute('aria-label', `Return ${titleOf(name)} to the info panel`);
    dock.textContent = '⤓';
    dock.addEventListener('pointerdown', (e) => e.stopPropagation());
    dock.addEventListener('click', () => { redock(name); save(); });

    bar.appendChild(title);
    bar.appendChild(dock);

    const body = document.createElement('div');
    body.className = 'float-body scrolls';
    body.appendChild(section);
    // A detached section is always open -- you detached it to watch it,
    // and a collapsed floating panel is just a title bar taking up room.
    section.open = true;

    wrap.appendChild(bar);
    wrap.appendChild(body);
    document.body.appendChild(wrap);

    wrap.addEventListener('pointerdown', () => raise(wrap));
    bar.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      e.preventDefault();
      beginDrag(wrap, e, (b) => { state.sections[name].box = b; save(); });
    });
    addResizeHandles(wrap, (b) => { state.sections[name].box = b; save(); });

    floatEls[name] = wrap;
    state.sections[name].floating = true;

    const { vw, vh } = viewport();
    let target = box || state.sections[name].box;
    if (!target) {
      // No remembered geometry: size the panel to the section that is now
      // inside it, so a four-line readout does not get a 240px box with
      // 150px of nothing under it, then find somewhere free to put it.
      wrap.style.width = '300px';
      const natural = Math.ceil(bar.getBoundingClientRect().height
        + section.getBoundingClientRect().height) + 18;
      target = placementFor(300, Math.max(MIN_H, Math.min(natural, vh - 24)), wrap);
    }
    applyBox(wrap, clampToViewport(target, vw, vh));
    state.sections[name].box = boxOf(wrap);
    updateDetachButtons();
  }

  function redock(name) {
    const wrap = floatEls[name];
    const section = sectionEls[name];
    if (!wrap || !section) return;

    // Back into its original position among whatever is currently docked.
    const wanted = dockOrder.indexOf(name);
    let before = null;
    for (const other of dockOrder) {
      if (dockOrder.indexOf(other) <= wanted) continue;
      if (!state.sections[other].floating && sectionEls[other].parentNode === panelsHost) {
        before = sectionEls[other];
        break;
      }
    }
    panelsHost.insertBefore(section, before);

    wrap.remove();
    delete floatEls[name];
    state.sections[name].floating = false;
    updateDetachButtons();
    syncTuningOffset();
  }

  /** The little "pop out" button that lives in every section header. */
  function addDetachButtons() {
    for (const [name, section] of Object.entries(sectionEls)) {
      const head = section.querySelector('.section-head');
      if (!head || head.querySelector('.section-detach')) continue;
      const button = document.createElement('button');
      button.className = 'section-detach';
      button.type = 'button';
      button.title = 'Pop this section out into its own panel';
      button.setAttribute('aria-label', `Detach ${titleOf(name)} into its own panel`);
      button.textContent = '↗';
      // A <summary> toggles on click of anything inside it, so this has to
      // stop the event dead or detaching would also collapse the section.
      button.addEventListener('click', (e) => {
        e.preventDefault();
        e.stopPropagation();
        detach(name);
        save();
      });
      head.appendChild(button);
    }
  }

  function updateDetachButtons() {
    for (const [name, section] of Object.entries(sectionEls)) {
      const button = section.querySelector('.section-detach');
      if (button) button.style.display = state.sections[name].floating ? 'none' : '';
    }
  }

  // --- narrow screens ---------------------------------------------------
  // Panels go home below FLOAT_MIN_WIDTH, but `state` keeps saying they
  // are floating so the big-screen layout comes back untouched when the
  // window widens again.
  let narrow = false;

  function applyWidthMode() {
    const shouldBeNarrow = window.innerWidth < FLOAT_MIN_WIDTH;
    if (shouldBeNarrow === narrow) return;
    narrow = shouldBeNarrow;
    document.body.classList.toggle('panels-narrow', narrow);

    if (narrow) {
      for (const name of Object.keys(floatEls)) {
        const remembered = state.sections[name].box;
        redock(name);
        state.sections[name].floating = true;   // remember the intent
        state.sections[name].box = remembered;
      }
      overlay.style.cssText = '';               // back to the CSS defaults
    } else {
      for (const name of dockOrder) {
        if (state.sections[name].floating && !floatEls[name]) {
          detach(name, state.sections[name].box);
        }
      }
      if (state.panels.overlay) applyBox(overlay, state.panels.overlay);
    }
    syncTuningOffset();
  }

  // --- the phone sheet ---------------------------------------------------
  // Below PHONE_MAX_WIDTH the info panel is a bottom sheet with three
  // detents. style.css owns where each detent is (as a transform on a
  // body class); this owns which one is current, and the gesture that
  // changes it. The only time an inline transform is written here is
  // while a finger is actually down -- the rest of the time the class
  // decides, so a resize or an orientation change cannot leave the sheet
  // stranded at a pixel offset that no longer means anything.
  const DETENTS = ['peek', 'half', 'full'];
  const phoneQuery = window.matchMedia(`(max-width: ${PHONE_MAX_WIDTH}px)`);
  let phone = false;
  let detent = 'peek';
  let swallowClick = false;

  /**
   * The peek height, read from CSS so the two cannot drift apart -- the
   * media queries change it per screen size, and this has to follow.
   * The fallback is only reached if --sheet-peek is missing entirely, and
   * matches the value declared in :root so the two agree even then.
   */
  function peekPx() {
    const raw = getComputedStyle(overlay).getPropertyValue('--sheet-peek');
    const value = parseFloat(raw);
    return Number.isFinite(value) && value > 0 ? value : 140;
  }

  function setDetent(name) {
    detent = DETENTS.includes(name) ? name : 'peek';
    for (const other of DETENTS) {
      document.body.classList.toggle(`sheet-${other}`, other === detent);
    }
    const grip = document.getElementById('sheet-grip');
    if (grip) grip.setAttribute('aria-expanded', detent === 'peek' ? 'false' : 'true');
  }

  function bindSheetGrip() {
    const grip = document.getElementById('sheet-grip');
    if (!grip) return;
    let drag = null;

    const maxOffset = (height) => Math.max(0, height - peekPx());

    grip.addEventListener('pointerdown', (e) => {
      if (!phone || e.button !== 0) return;
      e.preventDefault();
      grip.setPointerCapture(e.pointerId);
      const height = overlay.getBoundingClientRect().height;
      drag = {
        startY: e.clientY,
        lastY: e.clientY,
        lastT: e.timeStamp,
        velocity: 0,
        height,
        from: sheetOffsets(height, peekPx())[detent],
        moved: false,
      };
      document.body.classList.add('sheet-dragging');
    });

    grip.addEventListener('pointermove', (e) => {
      if (!drag) return;
      const dt = e.timeStamp - drag.lastT;
      // Instantaneous rather than averaged: what decides a flick is how
      // fast the finger was moving when it left, not how fast the whole
      // gesture was.
      if (dt > 0) drag.velocity = (e.clientY - drag.lastY) / dt;
      drag.lastY = e.clientY;
      drag.lastT = e.timeStamp;

      const dy = e.clientY - drag.startY;
      if (Math.abs(dy) > SHEET_TAP_SLOP_PX) drag.moved = true;
      const offset = Math.max(0, Math.min(maxOffset(drag.height), drag.from + dy));
      overlay.style.transform = `translateY(${Math.round(offset)}px)`;
    });

    const release = (e) => {
      if (!drag) return;
      const finished = drag;
      drag = null;
      document.body.classList.remove('sheet-dragging');
      if (grip.hasPointerCapture(e.pointerId)) grip.releasePointerCapture(e.pointerId);
      // Hand the position back to the stylesheet before choosing the
      // detent, so the class transition animates from where the finger
      // left it rather than fighting a stale inline transform.
      overlay.style.transform = '';

      if (!finished.moved) {
        // A tap. Handled here rather than in the click listener because a
        // touch that began with preventDefault may never produce a click
        // -- and the click listener below is what keyboard activation
        // needs, so it stays, guarded.
        swallowClick = true;
        setDetent(DETENTS[(DETENTS.indexOf(detent) + 1) % DETENTS.length]);
        return;
      }

      const offset = Math.max(0, Math.min(
        maxOffset(finished.height), finished.from + (e.clientY - finished.startY)));
      setDetent(detentFor(offset, finished.height, peekPx(), finished.velocity));
    };

    grip.addEventListener('pointerup', release);
    grip.addEventListener('pointercancel', release);

    // Keyboard: the grip is a real <button>, so Enter and Space arrive
    // here as a click with no pointer sequence in front of them.
    grip.addEventListener('click', () => {
      if (swallowClick) {
        swallowClick = false;
        return;
      }
      if (!phone) return;
      setDetent(DETENTS[(DETENTS.indexOf(detent) + 1) % DETENTS.length]);
    });
  }

  function applyPhoneMode() {
    const shouldBePhone = phoneQuery.matches;
    if (shouldBePhone === phone) return;
    phone = shouldBePhone;
    document.body.classList.toggle('layout-phone', phone);

    if (phone) {
      // Any width/height/left/top this panel picked up from a drag on a
      // wider screen would override the sheet's own geometry, since those
      // land as inline styles.
      overlay.style.cssText = '';
      setDetent('peek');
    } else {
      overlay.style.transform = '';
      for (const name of DETENTS) document.body.classList.remove(`sheet-${name}`);
      document.body.classList.remove('sheet-dragging');
    }
    syncTuningOffset();
  }

  // --- the fixed panels -------------------------------------------------
  function makeFixedPanelAdjustable(el, key, opts = {}) {
    if (!el) return;
    addResizeHandles(el, (b) => { state.panels[key] = b; save(); syncTuningOffset(); });
    if (opts.draggable) {
      el.addEventListener('pointerdown', (e) => {
        // Only a drag started on the panel's own chrome, never on a
        // control, a link, or inside the scrolling region.
        if (e.button !== 0) return;
        // In the phone layout the info panel is a sheet pinned to the
        // bottom of the screen. Dragging it around as a floating panel
        // would leave it somewhere it has no styling for, and there is no
        // second window to move it out of the way of.
        if (phone) return;
        if (e.target.closest('button, input, a, select, textarea, .scrolls, #panels')) return;
        if (e.target.classList.contains('panel-resize')) return;
        beginDrag(el, e, (b) => { state.panels[key] = b; save(); syncTuningOffset(); });
      });
    }
  }

  // --- reset ------------------------------------------------------------
  function resetLayout() {
    for (const name of Object.keys(floatEls)) redock(name);
    for (const name of Object.keys(state.sections)) {
      state.sections[name] = { floating: false, box: null };
    }
    state.panels = {};
    overlay.style.cssText = '';
    const minimap = document.getElementById('minimap-panel');
    if (minimap) minimap.style.cssText = '';
    try {
      window.localStorage.removeItem(STORE_KEY);
    } catch (err) { /* nothing to clean up */ }
    applyDefaults();
    syncTuningOffset();
  }

  /**
   * The out-of-the-box arrangement: the LB stopwatch lives in the right
   * rail, between the minimap and the camera, rather than in the info
   * panel. It is the one readout people watch while standing away from
   * the laptop, so it gets its own space by default.
   */
  function applyDefaults() {
    const minimap = document.getElementById('minimap-panel');
    const camera = document.getElementById('camera-panel');
    if (!minimap || !camera || !sectionEls.stopwatch) return;
    if (window.innerWidth < FLOAT_MIN_WIDTH) return;

    const mapBox = boxOf(minimap);
    const camBox = boxOf(camera);
    const top = mapBox.y + mapBox.h + 10;
    // Sized to the clock and its two buttons, not stretched to fill the
    // whole gap -- a rail panel that is two-thirds empty looks broken.
    // Still bounded by the gap, so it cannot grow into the camera.
    const gap = camBox.y - top - 10;
    detach('stopwatch', {
      x: mapBox.x,
      y: top,
      w: mapBox.w,
      h: Math.max(MIN_H, Math.min(156, gap)),
    });
  }

  // --- boot -------------------------------------------------------------
  addDetachButtons();
  bindSheetGrip();
  makeFixedPanelAdjustable(overlay, 'overlay', { draggable: true });
  makeFixedPanelAdjustable(document.getElementById('minimap-panel'), 'minimap', { draggable: true });

  const saved = load();
  if (saved) {
    for (const [name, entry] of Object.entries(saved.sections)) {
      if (state.sections[name]) state.sections[name] = entry;
    }
    state.panels = saved.panels;
  }

  if (window.innerWidth >= FLOAT_MIN_WIDTH) {
    if (saved) {
      for (const name of dockOrder) {
        if (state.sections[name].floating) detach(name, state.sections[name].box);
      }
      if (state.panels.overlay) applyBox(overlay, state.panels.overlay);
      if (state.panels.minimap) applyBox(document.getElementById('minimap-panel'), state.panels.minimap);
    } else {
      applyDefaults();
    }
  }
  narrow = window.innerWidth < FLOAT_MIN_WIDTH;
  document.body.classList.toggle('panels-narrow', narrow);
  applyPhoneMode();

  updateDetachButtons();
  syncTuningOffset();

  const resetButton = document.getElementById('reset-layout');
  if (resetButton) resetButton.addEventListener('click', resetLayout);

  window.addEventListener('resize', () => {
    applyWidthMode();
    applyPhoneMode();
    syncTuningOffset();
  });
})();
