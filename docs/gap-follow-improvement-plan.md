# gap_follow improvement — findings & plan

Baseline measured with `tools/f1tenth_sim/eval_gap.py` (seed 12345, fidelity
profile `car`), which drives the real `gap_logic` pipeline with the shipped
`config/gap_follow.yaml` values:

| track | result | laps | sim_time | mean_speed | stops | contact | min_fwd |
|---|---|---|---|---|---|---|---|
| indoor_oval (1.8m) | PASS | 1.0 | 15.8s | 1.91 | 2 | 0 | 1.50 |
| indoor_tight (1.4m) | **FAIL** | 0.0 | 6.2s | 1.49 | 9 | **1** | 1.19 |
| indoor_wide (2.6m) | PASS | 1.0 | 19.9s | 2.21 | 2 | 0 | 1.94 |
| asb_10000 (1.5m) | PASS | 1.0 | 24.6s | 2.05 | 2 | 0 | 1.34 |

## Symptoms (match the real-car report)

1. **Too slow** — mean speed 1.9-2.2 m/s against a 3.5 m/s cap, on tracks the
   curvature ceiling should let it take much faster.
2. **Turns too tightly / hugs the inside wall.** On indoor_tight the left wall
   distance collapses 0.72 -> 0.23 m through the corner: the aim point runs
   along the inside wall, so the car arrives at the apex nearly touching it,
   and then the TTC brake (0.30s) and the forward-reserve creep fight the turn.
3. **Gets stuck near walls / deadlocks.** indoor_tight ends in wall contact
   after repeated ttc_brake -> creep -> ttc_brake cycles.

## Root causes (from reading + tracing)

- **Aim point runs along the inside of a corner.** `aim_within_gap` picks the
  gap midpoint between real edges, or the deepest beam at a FOV-clipped edge.
  Neither has any notion of setting up a corner *wide*: the midpoint of the
  visible opening on a right-hand corner is biased toward the inside (left)
  wall, and "deepest beam" points down the wall, not around the corner. There
  is no apex/exit geometry anywhere in the node.
- **No corner anticipation.** `enable_cornering_anticipation` exists but is
  shipped off (measured to over-brake in open rooms). The first time the node
  learns about a corner is when the preferred 2.0 m gap disappears and the
  0.8 m fallback engages — already at the corner entry, already too close.
- **Speed is set by a stack of caps computed from where the car is *pointed*,
  not from where it will be.** `curvature_speed_limit` uses the current
  commanded curvature; a car still pointing straight down a straight reads
  ~zero curvature and full speed right up to the moment the fallback gap
  flips the steering to full lock. That is the indoor_tight trace: 2.26 m/s
  at t=1.5s, ttc_brake at t=2.53s.
- **Corner setup is a single gain.** `steering_gain=1.0` points the wheels at
  the gap bearing; there is no term that pulls the car toward the *outside*
  of an approaching corner (the racing-line entry), which is the actual fix
  for both "too tight" and "hits the inside wall".

## Candidate changes (each measurable with eval_gap)

A. **Corner-aware aim: bias the aim point toward the outside of a detected
   turn.** When the chosen gap is a corner (fallback tier, or a strong bearing
   lean), aim not at the geometric midpoint but at a point offset toward the
   outside wall — the standard "slow in, fast out" entry. Bounded, faded, and
   never overriding the safety bubble / disparity inflation.

B. **Lookahead curvature speed cap.** Estimate the curvature the car is about
   to need (from the gap's near/far bearing disagreement, normalised by the
   gap's own width so open rooms don't false-trigger) and feed it through the
   existing `curvature_speed_limit` so the car slows *before* turn-in, not at
   the apex. This is the fix for the shipped-off cornering anticipation: same
   mechanism, properly normalised.

C. **Stop-go recovery.** The TTC/clearance -> creep -> TTC cycle is what
   "gets stuck" looks like. The escape creep already exists; the gap is that
   a directional trip that finds a gap still caps to creep speed even when
   the gap is clearly open.

D. **Tune pass with the harness** once A-C land: corner_speed, safety_margin,
   the centering gains, against all four indoor tracks + the three gym
   circuits, and re-run the real node tests.

## Constraint reminders (from CLAUDE.md / TEST_QUALITY_STANDARDS)

- LB deadman stays enabled; no change to the deadman path.
- Every new test must be seen to fail against a stub before being trusted.
- Never weaken a test to make a suite pass.
- eval_gap.py is a mirror of gap_follow_node._scan_step — keep them in step.
