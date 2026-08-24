/*
 * measure.js -- the geometry behind the dashboard's map measuring tool.
 *
 * Click points on the map, get the distance along the chain. Two clicks is
 * the point-to-point case; more keeps adding segments and a running total.
 *
 * Everything in this file is PURE: no DOM, no canvas, no WebSocket, no
 * globals. dashboard.js owns the pointer events, the drawing and the
 * sidebar; this file owns the arithmetic and the decisions. That split is
 * what lets test/browser/measure_test.js `require()` it under plain node
 * and check the maths for real, the same way panels.js splits its box
 * geometry from its DOM wiring.
 *
 * ---------------------------------------------------------------------
 * The two frames, and why a measurement can never span them
 * ---------------------------------------------------------------------
 * The dashboard draws in one of two coordinate frames (see dashboard.js's
 * worldToCanvas / bodyToCanvas):
 *
 *   'map'   world metres, once a localization pose exists. A point stays
 *           on the wall you put it on.
 *   'body'  metres relative to the car itself, before any pose has
 *           arrived. The car is at the origin, facing "up".
 *
 * A body-frame point means "0.8 m to the left of where the car is right
 * now". It is a statement about the picture, not about the world, and it
 * moves as the car moves. There is no conversion between the frames --
 * that is the whole reason dashboard.js tracks bodyPanX/bodyPanY separately
 * from centerX/centerY.
 *
 * So every point carries the frame it was taken in, and a chain is only
 * ever drawn while that frame is still the active one. When a pose arrives
 * mid-measurement the body-frame chain is cleared and said so, rather than
 * reinterpreted as world coordinates -- which would silently draw a
 * distance wrong by the car's entire map offset. A confidently wrong number
 * on a HUD is worse than no number.
 */
(function () {
  'use strict';

  // --- Tap discrimination ------------------------------------------------
  //
  // The canvas is a pan surface first. A tap has to be told from a drag
  // that happened to end near where it started, and from one finger of a
  // pinch, without making either gesture feel sticky.

  /** A mouse is steady; 8 CSS px is a slip, not an intent to drag. */
  const TAP_SLOP_MOUSE_PX = 8;
  /** A finger is not steady. Below this and every tap would pan instead. */
  const TAP_SLOP_TOUCH_PX = 14;
  /** Longer than this and the person was holding, not tapping. */
  const TAP_MAX_MS = 500;

  // --- Drawing thresholds ------------------------------------------------

  /** A segment shorter than this on screen has no room for its own label.
   *  Its length still counts toward the total, which is always shown. */
  const MIN_LABEL_PX = 28;
  /** How far off the line a label sits, in canvas pixels. */
  const LABEL_OFFSET_PX = 12;

  function slopFor(pointerType) {
    return pointerType === 'mouse' ? TAP_SLOP_MOUSE_PX : TAP_SLOP_TOUCH_PX;
  }

  /**
   * Was this gesture a tap?
   *
   * `maxTravelPx` is the FURTHEST the pointer got from where it went down,
   * not the distance between the two endpoints. A drag out and back would
   * otherwise read as a tap and drop a point in the middle of a pan.
   *
   * A gesture that ever had two pointers down is never a tap, whatever the
   * numbers say -- that is a pinch, and one finger lifting early must not
   * turn the other into a click.
   */
  function isTap(gesture) {
    if (!gesture) return false;
    const { maxTravelPx, durationMs, pointerCount, pointerType } = gesture;
    if (!Number.isFinite(maxTravelPx) || !Number.isFinite(durationMs)) return false;
    if (pointerCount !== 1) return false;
    if (durationMs > TAP_MAX_MS) return false;
    return maxTravelPx <= slopFor(pointerType);
  }

  // --- The chain ---------------------------------------------------------

  function isPoint(p) {
    return !!p && Number.isFinite(p.x) && Number.isFinite(p.y);
  }

  /**
   * Add a point, or return the chain unchanged.
   *
   * Refuses a non-finite coordinate (a transform run before the canvas has
   * been sized produces them) and a point on top of the previous one --
   * a double-click fires two pointerup events at the same place, and a
   * zero-length segment is noise in the list and a divide-by-zero in the
   * label placement.
   */
  function addPoint(points, point, minSeparationM) {
    if (!isPoint(point)) return points;
    const gap = Number.isFinite(minSeparationM) ? minSeparationM : 0;
    const last = points[points.length - 1];
    if (last && tooClose(last, point, gap)) return points;
    return points.concat([{ frame: point.frame, x: point.x, y: point.y }]);
  }

  function tooClose(a, b, minSeparationM) {
    if (!isPoint(a) || !isPoint(b)) return false;
    return Math.hypot(b.x - a.x, b.y - a.y) <= minSeparationM;
  }

  function undo(points) {
    return points.length ? points.slice(0, -1) : points;
  }

  /** One entry per gap between consecutive points. n points give n-1. */
  function segments(points) {
    const out = [];
    if (!points) return out;
    for (let i = 1; i < points.length; i++) {
      const a = points[i - 1];
      const b = points[i];
      if (!isPoint(a) || !isPoint(b)) continue;
      out.push({
        ax: a.x, ay: a.y, bx: b.x, by: b.y,
        length: Math.hypot(b.x - a.x, b.y - a.y),
      });
    }
    return out;
  }

  /** Total length along the chain, in metres. 0 for fewer than two points. */
  function total(points) {
    let sum = 0;
    for (const seg of segments(points)) sum += seg.length;
    return sum;
  }

  /**
   * Metres as a string, rounding BEFORE choosing the unit.
   *
   * The obvious form -- `m >= 1 ? metres : centimetres` -- prints "100 cm"
   * for 0.999, because it picks the unit from the unrounded value and then
   * rounds inside it. Rounding to centimetres first and comparing against
   * 100 makes the two branches agree at the boundary.
   *
   * Deliberately NOT shared with dashboard.js's updateScaleBar(), whose
   * values are round steps out of SCALE_BAR_STEPS_M by construction and so
   * can never land on this boundary. Sharing it would tie a display detail
   * of one to the correctness of the other.
   */
  function formatDistance(m) {
    if (!Number.isFinite(m) || m < 0) return '--';
    const cm = Math.round(m * 100);
    return cm >= 100 ? `${(cm / 100).toFixed(2)} m` : `${cm} cm`;
  }

  /**
   * What state the tool is in, given the chain and whether a pose exists.
   *
   *   'idle'   nothing measured yet
   *   'map'    world-frame chain, and we still have a pose
   *   'body'   robot-centric chain, and still no pose
   *   'stale'  the frame changed underneath the chain -- dashboard.js
   *            clears it and says why
   *
   * `state.pose` is set once in dashboard.js and never cleared, so in
   * practice the only flip is body -> map, when localization first
   * converges. Handled generally anyway: the cost is one comparison and
   * the alternative is a wrong number on screen.
   */
  function frameFor(points, hasPose) {
    if (!points || !points.length) return 'idle';
    const active = hasPose ? 'map' : 'body';
    const chain = points[0].frame;
    if (chain !== active) return 'stale';
    return active;
  }

  // --- Label placement ---------------------------------------------------

  /**
   * Where and how to draw one segment's label, in canvas pixels.
   *
   * Returns null for a segment too short to label -- its length still
   * counts toward the total.
   *
   * Two things matter here and both are easy to get subtly wrong:
   *
   *   * The text is rotated to lie along the segment, but a segment running
   *     right-to-left would put it upside down. The angle is folded into
   *     [-pi/2, pi/2] so text is always readable, without moving the label.
   *   * The label sits off the line rather than on it, along the segment's
   *     perpendicular, and always on the upper side of the screen so a
   *     chain of segments does not have labels flipping from side to side.
   */
  function labelPlacement(screenSegment, minPx) {
    if (!screenSegment) return null;
    const { ax, ay, bx, by } = screenSegment;
    if (![ax, ay, bx, by].every(Number.isFinite)) return null;

    const dx = bx - ax;
    const dy = by - ay;
    const length = Math.hypot(dx, dy);
    const floor = Number.isFinite(minPx) ? minPx : MIN_LABEL_PX;
    if (!(length > 0) || length < floor) return null;

    let angle = Math.atan2(dy, dx);
    if (angle > Math.PI / 2) angle -= Math.PI;
    else if (angle < -Math.PI / 2) angle += Math.PI;

    // Unit perpendicular, chosen to point up the screen (canvas +Y is
    // down, so "up" is the one with a negative Y component).
    let nx = -dy / length;
    let ny = dx / length;
    if (ny > 0) { nx = -nx; ny = -ny; }

    return {
      x: (ax + bx) / 2,
      y: (ay + by) / 2,
      angle,
      offsetX: nx * LABEL_OFFSET_PX,
      offsetY: ny * LABEL_OFFSET_PX,
    };
  }

  // Exported unconditionally so the node test can load this file with no
  // DOM at all -- there is nothing in here that needs one.
  const api = {
    TAP_SLOP_MOUSE_PX, TAP_SLOP_TOUCH_PX, TAP_MAX_MS,
    MIN_LABEL_PX, LABEL_OFFSET_PX,
    slopFor, isTap,
    addPoint, tooClose, undo, segments, total,
    formatDistance, frameFor, labelPlacement,
  };
  if (typeof window !== 'undefined') window.__measure = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})();
