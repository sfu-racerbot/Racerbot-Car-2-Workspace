# TEST_QUALITY_STANDARDS.md

What separates a **real test** from a **hollow test** in this workspace, in terms you can check
mechanically rather than argue about.

Companion to `AGENTS.md` → *Testing & Safety* (which sets the house rules: pytest, `test_*.py`, tests
in `src/<package>/test/`, no numeric coverage threshold, never disable `enable_deadman` unilaterally).
This file answers the question that one leaves open: **what makes a test worth having.**

## Why this file exists

This repo drives a physical car. A failing test is *information* — it is the cheapest place in the
whole system to find out that the steering math is wrong. A test that **cannot fail** is worse than
no test: it costs review time, it occupies the name of the coverage it isn't providing, and it turns
"the suite is green" from evidence into noise.

This is not hypothetical here. Two documented cases from this repo:

- **A whole simulator's safety verdict was true by construction.** `tools/f1tenth_sim/sim_fidelity/plant.py`
  records that gym's collision check never fired in this harness's geometry — the LiDAR was placed
  0.33 m forward of `base_link` while the 0.58 m collision box is centred there spanning only ±0.29 m,
  so the sensor sat outside its own collision body, `side_distances` came back all zeros, and the test
  degenerated to "is any beam below 0.005 m" — which `ScanSimulator2D.scan` makes impossible by
  clipping every range to `range_min` = 0.05 m. Measured: **a car driven straight into the Spielberg
  barrier travelled 35.5 m through it, reaching 0.058 m from a wall, and was never flagged.**
  (The car was tape-measured on 2026-08-24 and the LiDAR is 0.26 m forward, inside the box, so that
  first fault no longer holds as written — and nobody has re-measured whether the flag fires now.
  The lesson is unchanged and so is the rule: **take the verdict from geometry you compute yourself,
  never from gym's flag.**)
- **A control assertion that passed on noise.** `src/pure_pursuit/test/test_opponent_integration.py:241`
  documents an overtake test that asserted on the *sign* of a steering angle. Back-to-back
  `control_loop()` calls are microseconds apart, `max_steering_rate` allowed ~0.0001 rad of change
  per call, and the command stayed pinned near zero "where its sign was noise." It passed for the
  wrong reason until a few microseconds of extra work per tick flipped it. It was, as the docstring
  says, "an assertion about the test track."

**The standard: every test must be able to fail, for a specific real reason, and you must be able to
say what that reason is.**

## Scope and current state

Applies to the first-party packages — `drive_intent`, `gap_follow`, `odom_calibration`,
`pure_pursuit`, `race_diagnostics`, `racerbot_sim`, `usb_cam_stream`, `web_dashboard` — and to
`tools/f1tenth_sim/`. It does **not** govern vendored or submodule trees (`f1tenth_system`,
`realsense-ros`, `particle_filter`, `range_libc`), whose tests are upstream's.

As of 2026-08-22 those eight packages hold roughly **800 test functions**, and the suite is in good
shape: three mock usages in total, no swallowed exceptions, no `pytest.raises(Exception)`, and every
`skip` carrying a reason. **This document mostly codifies conventions the suite already follows** —
the examples cited throughout are real tests in this repo, not hypotheticals. Read it as the bar to
keep, not a remediation plan.

**How to use it:** §1 when writing or reviewing a test, §3 as the verdict you write in a review
comment, §5 as a pre-merge sweep you can paste into a terminal.

---

## 0. The stub swap — the one check that subsumes most of the rest

Before anything else, ask the only question that matters: *if the code were wrong, would this test
notice?* Don't reason about it. Measure it.

**Procedure (about a minute per unit):**

1. Pick the function or class the test claims to cover.
2. Replace its body with one of:
   - `pass` (so it returns `None`),
   - `return <the constant the happy-path test expects>`,
   - `raise NotImplementedError`.
3. Run *only* the tests for that unit.
4. **If they still pass, the test is hollow.** It is detecting nothing. Rewrite it or delete it.
5. Revert the stub. Never commit it, and never leave it in place on the car.

Case 2 is the important one and the one people skip. A test that survives `return 0.42` has
memorised one answer; it has not checked a computation.

### The mutation set (numeric, geometric, and control code)

The stub swap catches gross hollowness. For anything doing math — `racing_math`, pure-pursuit
lookahead, gap-follow scan processing, the servo mapping, `odom_calibration` — apply each mutation
below and require **at least one test to fail for each**:

| # | Mutation | What it proves is tested |
|---|---|---|
| M1 | Flip a comparison (`<` → `<=`, `>` → `<`) | Boundary behavior, not just the interior |
| M2 | Negate a sign (`-x` → `x`) | Direction and handedness (steer left vs. right) |
| M3 | Change a constant by 10% | The value matters, not just its presence |
| M4 | Swap two adjacent same-typed args (`(x, y)` → `(y, x)`) | Argument order — a classic pose/point bug |
| M5 | Return an input unchanged instead of the computed result | That a computation happens at all |
| M6 | Delete a clamp or bounds check | That limits are enforced, not assumed |

M2 is the one the overtake bug above would have failed: a sign assertion on a value pinned near zero
survives negation. If a mutant lives, that is a real gap — and it tells you more than a coverage
percentage would. **Line coverage is not evidence here**, which is why `AGENTS.md` sets no threshold:
a test can execute every line of `racing_math.py` and assert nothing about any of them.

---

## 1. Anti-pattern checklist

Flag a test matching any of the following. Each entry gives the mechanical detection and the fix.

### A1 — Asserts on a mock's return value instead of real behavior

The test configures a fake to return `X`, then asserts something equals `X`. It is testing the
mocking library.

**Detect:**
```bash
grep -rn "Mock\|MagicMock\|patch(" src/{drive_intent,gap_follow,odom_calibration,pure_pursuit,race_diagnostics,racerbot_sim,usb_cam_stream,web_dashboard}/test --include='*.py'
```
For each hit, check whether the configured `return_value` appears in **both** the arrange block and
the assertion — that is the signature. Also flag `assert_called_once()` / `assert_called_with(...)`
standing as a test's *only* assertion: "the function was called" is not "the function was correct."

**Fix:** assert on what the code under test *did with* the mock — the arguments it passed, the state
it reached, the message it published, the branch it took.

**Where mocking is legitimate here:** only at a process boundary a test genuinely cannot cross —
serial port, socket, wall clock, filesystem. **Never mock code we own that is already ROS-free** —
`racing_math`, `protocol`, `netbind`, `drive_intent`. Those modules were deliberately split out so
they can be called for real; mocking one is choosing not to test it. The workspace currently has
only three mock sites total (`test_map_despeckle.py`, `test_mapstream.py`) — keep it that way.

### A2 — Would still pass against a stub, `pass`, or a hardcoded return

The core failure. Detect with §0. Common shapes:

- Asserts only on the *type* or *shape* of the result (`isinstance(out, float)`, `len(out) == 2`).
- Asserts only that the call didn't raise.
- Asserts on something the test itself constructed and passed straight through.
- Probes only inputs where the correct answer coincides with the degenerate one (the overtake bug:
  asserting a sign on a value the rate limiter had pinned to ~0).

`src/web_dashboard/test/test_netbind.py` states the principle in its own module docstring: it binds
a real socket the way `dashboard_node.py` does, "because the bug being guarded against here
(IPv4-only listener) is invisible to any test that only checks a return value."

**Fix:** choose an input where a *wrong* implementation gives a *different* answer, and assert that
answer. If you cannot construct one, you do not yet know what the function guarantees — establishing
that is the actual work.

### A3 — Hardcodes today's output with no independent notion of "correct"

Someone ran the code, pasted the number in, and the test now enforces "behaves as it did the day it
was written." That is a **change detector**, not a correctness test: it locks in bugs permanently and
fails on legitimate improvements.

**The oracle rule: every expected value must trace to one of four sources, and the source must be
stated in the test.**

| Oracle | What it means | Real example in this repo |
|---|---|---|
| **Closed form** | Recomputed independently in the test from the math, not copied from output | `test_racing_math.py` — "plain synthetic geometry (circles, straight lines, a stadium shape) with known-by-construction answers" |
| **Invariant** | A property any correct implementation must satisfy | `test_steering_arc_curvature_sign_matches_left_right`; `test_adaptive_lookahead_clips_to_bounds`; the cross-parameter invariants in `test_*_live_tuning.py` |
| **Spec** | A datasheet, message definition, or doc, cited by path | the `0.5304` servo neutral from `docs/hardware-reference.md`; LiDAR `range_min` 0.05 m |
| **Recorded reality** | A committed measurement, with the run identified | `docs/asb-10000-sim-results.json`, `docs/auto-map-sim-results.json`, `docs/f1tenth-sim-results.json`, `docs/sim-fidelity-audit.md` |

The existing suite already does this in prose — "known-by-construction", "Guards the invariant the
preview exists for", "it measured 335deg of turning against a 300deg gate". **That prose form
counts.** Do not retrofit a tag onto 800 existing tests.

**For new or changed tests**, make the oracle greppable with a marker comment on or above the
assertion: `# oracle: closed form`, `# oracle: invariant`, `# oracle: spec docs/hardware-reference.md`,
`# oracle: measured docs/asb-10000-sim-results.json`.

**Change detectors are allowed** — they catch accidental drift — but only when (a) labelled
`# change-detector (not an oracle)` and (b) the same unit also has at least one real-oracle test. An
unexplained golden number is an A3 finding.

### A4 — Happy path only, no edge cases or error states

The car does not live on the happy path. Empty scans, a path that ends, a controller released
mid-corner, a stale topic — these are *normal operating conditions* for this vehicle.

**Detect:** read the file's test names and inputs. If nothing names or exercises a boundary, a
degenerate input, or a failure condition, it fails this check.

**Required edge classes** — a unit is not covered until its row is:

| Kind of code | Must be tested with |
|---|---|
| Numeric / geometry | zero, negative, exactly at the boundary, one step past it, `NaN`/`inf`, empty array, single element |
| Parsers (`web_dashboard/protocol.py`) | empty input, truncated message, wrong type, unknown field, oversized payload, malformed UTF-8 |
| Path / control (`racing_math`, pure pursuit) | no path points, path entirely behind the car, car exactly on the path, lookahead past the end of the path, wrap-around at lap close |
| Scan consumers (`gap_follow`) | all-`inf` ranges, all-zero ranges, `NaN` beams, unexpected beam count, stale timestamp |
| Anything that can publish `/drive` | deadman released, `/joy` never received, `/joy` stale, odometry stale (see §4) |

**Fix:** assert the *specified* behavior at each edge — which for driving code is nearly always
"command zero / publish nothing," never "raises whatever it happens to raise."

### A5 — Trivially-true assertion, or a try/except that swallows failures

**Detect** (regexes tuned to avoid the false positives the naive versions produce — `assert 1\b`
matches the legitimate `assert 1.0 / RACK_LIMIT == ...`):
```bash
T=src/{drive_intent,gap_follow,odom_calibration,pure_pursuit,race_diagnostics,racerbot_sim,usb_cam_stream,web_dashboard}/test
grep -rnE "assert +(True|False|1|0) *(,|$|#)"  $T --include='*.py'
grep -rnE "assert .+ is not None *$"           $T --include='*.py'
grep -rnE "except[^:]*: *(pass|continue) *$"   $T --include='*.py'
grep -rn  "pytest.raises(Exception)"           $T --include='*.py'
```
Also flag any test function containing **zero** `assert` statements.

**Rules:**

- `assert x is not None` is not a behavioral assertion unless `None` is a real specified outcome for
  some other input. Assert the value. (The suite has ~21 of these; most are reasonable
  pre-conditions guarding a *following* assertion — that is fine. One standing alone is not.)
- **No bare `except`, no `except Exception: pass` in a test.** A test that catches exceptions to "be
  robust" has inverted its purpose: the exception *is* the result you were hired to report.
- Expecting a raise means `pytest.raises(SpecificError, match="...")`. `pytest.raises(Exception)`
  passes on a typo in the test itself.
- `pytest.approx` needs an explicit tolerance with units:
  `== pytest.approx(expected, abs=1e-3)  # 1 mm`. A bare `approx` hides how much error you accepted.
- Assertions must be unconditional. An assertion inside an `if`, or a `try` around the call under
  test, silently skips the check.

### A6 — Loosened, skipped, or deleted because it was failing

**A failing test is a finding, not an obstacle.** The default response is to fix the code.

Changing the test instead is permitted **only** when the specification genuinely changed, and the
commit body must record all four of:

1. what the old test asserted,
2. what the new specified behavior is,
3. **where that spec is written down** — a `docs/` section, an upstream doc, a datasheet,
4. why the old behavior was *wrong*, not merely inconvenient.

"Flaky", "no longer relevant", "was failing after the refactor", and "the new value is what the code
does now" are **not** reasons. The last is A3 wearing a disguise.

All of these count as loosening and need the same justification: widening a tolerance (`abs=1e-3` →
`abs=1e-1` is not a tweak — it is the difference between a 1 mm and a 10 cm claim about the car);
weakening an equality to a bound, or a bound to a type check; dropping the edge case that failed;
adding `skip`/`xfail`; deleting the test.

The *right* way to do this is already in the tree: when the overtake sign assertion was found to be
hollow, it was replaced with a stronger comparison between the two sides
(`right.drive.steering_angle < left.drive.steering_angle`) and a docstring explaining why the old
form was wrong. Restated, not relaxed.

**Detect:**
```bash
grep -rnE "@pytest\.mark\.(skip|xfail)" $T --include='*.py' | grep -v "reason="   # decorator, no reason
grep -rnE "pytest\.(skip|xfail)\( *\)"  $T --include='*.py'                       # genuinely bare skip
git diff main... -- 'src/*/test/*' 'tools/**' | grep -E '^[-+].*(assert|approx|abs=|rel=|skip|xfail)'
```
Note `pytest.skip("...")` takes its reason **positionally**, so don't filter it on `reason=` — that
flags the legitimate `test_map_despeckle.py:245`
(`pytest.skip('the recorded reference map is not on this machine')`).
Every `skip`/`xfail` needs a `reason=` naming an issue or doc section **and** a condition for its
removal; use `xfail(strict=True)` so it fails loudly once the bug is fixed. `pytest.importorskip`
for a genuinely optional dependency (as in `test_opponent_integration.py` for `range_libc`, or
`test_netbind.py` for `tornado`) is fine and needs no reason string.

**Review stop:** a test deleted or loosened in the same commit as a change to the code it covered.
That is the exact shape of "make the suite green," and it needs an explicit explanation before merge.

---

### Repo-specific anti-patterns

Additions beyond the six general ones, drawn from failure modes this workspace has actually hit.

### A7 — The verdict comes from the thing under test

Asserting on a simulator's own self-report is asserting that the simulator agrees with itself. This
produced 35.5 m of driving through a barrier under a `"collision": false` banner (see *Why this file
exists*).

**Rule:** the pass/fail signal must be computed **independently** of the system producing the
behavior. `tools/f1tenth_sim/sim_fidelity/plant.py` supplies `_wall_contact` and `_chassis_contact`
against the real chassis geometry; `racerbot_sim/sim_bridge.py:403` ORs gym's flag with its own
`body_contact()` sampling rather than trusting it alone. **Never reinstate a "no collision" claim
resting on gym's flag by itself.**

### A8 — Testing only the permissive side of a safety check

A deadman test proving "LB held → the car drives" has tested the half that fails *safely*. The
refusal is the half that matters. `src/gap_follow/test/test_gap_follow_node.py:101` is the model —
it asserts both the command and the reason:

```python
def test_refuses_to_drive_when_the_deadman_is_released(node):
    published = _capture(node)
    _ready(node)
    _hold_deadman(node, held=False)
    node.scan_callback(_scan())
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'deadman_released'
```

See §4 for the required set.

### A9 — The test isn't run by the command people actually run

A test that exists but never executes is worse than a deleted one, because someone will count it.

**`CLAUDE.md` describes `python3 -m pytest src/<pkg>/test/ -v` as "standalone (non-ROS) unit tests,
no sourcing/build required". That is no longer true for every package.** Nine first-party test files
now import `rclpy` — in `gap_follow`, `pure_pursuit`, and `usb_cam_stream`. Verified behavior:

| Command | Result |
|---|---|
| `python3 -m pytest src/pure_pursuit/test/` **unsourced** | 185 of 274 collected, 5 collection errors, run **aborts** |
| same, after `source /opt/ros/jazzy/setup.bash` | 274 collected, all run |

Unsourced it fails loudly, which is fine. The trap is reaching for
`--continue-on-collection-errors`, or running only the files that import cleanly: that shows green
while **89 pure_pursuit tests never ran**, including every deadman, watchdog, and opponent test.

**Rule:** source ROS before claiming a package is green, and run both runners:
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m pytest src/<pkg>/test/ -v
colcon test --packages-select <pkg> && colcon test-result --verbose
```
Compare the collected count against `grep -c '^def test' src/<pkg>/test/*.py`. A new test must state
which runner executes it. Never paper over a collection error with
`--continue-on-collection-errors`.

---

## 2. Positive criteria — what a real test verifies

A test earns its place by doing at least one of these, concretely.

### P1 — Real output for real input

Calls the actual function with a meaningful input constructed in the test, and asserts a value that
comes from an oracle (§A3), not from a previous run.

> **Model:** `src/pure_pursuit/test/test_racing_math.py` — synthetic geometry with
> known-by-construction answers, no ROS, no hardware, no simulator. `test_quaternion_to_yaw_90_degrees`
> derives the quaternion from `sin(45°)/cos(45°)` in the test itself rather than pasting a float.

### P2 — An observable side effect

Asserts on something the system *did*, at the boundary a real consumer would see: a message
published with specific field values, a file written, a state transition. For a ROS node, drive a
callback and inspect the captured `AckermannDriveStamped` — `.drive.speed`, `.drive.steering_angle` —
not internal attributes a refactor could rename without changing behavior.

**Publishing nothing is an observable side effect too**, and here it is often the important one:
assert that no command went out, or that a zero command did.

### P3 — An integration point across components

Two real components with no mock between them. `CLAUDE.md` notes wiring bugs are invisible to the
math-level simulator and are most of what has actually broken — so these carry unusually high value.

> **Models in the tree:** `test_netbind.py` binds a real socket rather than checking a return value.
> `test_mapstream.py:58` builds "a model of the browser's side of this protocol, used to prove that"
> the two halves agree. `drive_intent` should be round-tripped between the Python schema and the C++
> single-header port.

### P4 — A regression tied to a specific past bug

The strongest kind of test here, because the bug is proof the test can fail.

Requirements: name the bug (in the test name or docstring), cite the commit or `docs/` writeup, state
the symptom in a line, and — **say so explicitly** — confirm it fails against the pre-fix code. A
regression test never checked against the old code is an assumption.

> **Model:** `test_localization_watchdogs.py:248` — the car "refused to move at all. Restricting to a
> 180deg forward window fixes..." The symptom, the cause, and the fix, all in the test.

### Non-negotiable properties

Deterministic (seed every RNG; inject the clock — **never `time.sleep` to sequence events**),
independent of test order and other tests' state, free of network and real hardware, and fast enough
that people run it. A flaky test gets fixed or deleted, never re-run until green.

---

## 3. Pass/fail rubric — apply test-by-test

Three gates. **All three must be YES for a PASS.**

| Gate | Question | How you check it |
|---|---|---|
| **G1 — KILL** | Does it fail when the code is wrong? | Stub swap (§0), plus ≥1 mutant from M1–M6 for numeric code |
| **G2 — ORACLE** | Is the expected value justified independently of the implementation? | Closed form, invariant, cited spec, or recorded measurement (§A3) — prose counts on existing tests, a marker on new ones |
| **G3 — BOUNDARY** | Is the failure/edge behavior of this same unit covered, by this test or a sibling in the file? | The relevant row of the §A4 table |

**Verdicts:**

| Verdict | Condition | Action |
|---|---|---|
| **PASS** | G1 ✓, G2 ✓, G3 ✓ | Merge. |
| **WEAK** | G1 ✓, one of G2/G3 ✗ | Fix before merge. It detects *change* but doesn't establish *correctness*, or the unit's edges are unguarded. |
| **HOLLOW** | G1 ✗ | Do not merge. Delete or rewrite — it cannot fail, so it is not a test. |

**Hard fails regardless of the gates** (driving code, per the workspace safety policy):

- A node able to publish `/drive`, `/ackermann_cmd`, or `/commands/motor|servo/*` whose tests never
  exercise the **deadman deny path** (§A8, §4).
- `enable_deadman:=false` in a node test **without** a `drive_topic` remap away from `/drive` (§4).
- A safety or collision claim resting on a signal that cannot fire (§A7).
- New tests added to a package without confirming they are collected by a runner (§A9).
- A test loosened, skipped, or deleted alongside a change to the code it covered, without the §A6
  justification.

**Review-comment form** — one line per test:

```
test_overtake_steers_toward_the_gap — G1:✗ (sign assertion on a rate-limited ~0 value) → HOLLOW
test_lookahead_wraps_at_lap_close   — G1:✓ G2:✗ (golden 0.1834, no oracle) G3:✓ → WEAK
```

---

## 4. Standing requirements for specific subsystems

### The LB deadman

Every node that can move the car enforces the deadman independently. Its tests must cover all four
states, asserting the **command**, not an internal flag:

| State | Required assertion |
|---|---|
| LB held | non-zero command published |
| LB released | zero command / no command |
| `/joy` never received | zero command / no command |
| `/joy` stale (past timeout) | zero command / no command |

`gap_follow` covers these directly (`test_refuses_to_drive_when_the_deadman_is_released`,
`test_stops_when_the_joy_stream_goes_stale`, `test_recovery_never_depends_on_cycling_the_deadman`).
Both `gap_follow` and `pure_pursuit` additionally pin
`test_the_deadman_cannot_be_switched_off_at_runtime` — the live-tuning path rejects
`{'enable_deadman': False}`. Keep both layers.

**On `enable_deadman:=false` in tests — this is permitted, and the blanket ban you might expect is
wrong.** `pure_pursuit`'s rclpy tests disable it at 11 sites so the control math can be driven
without forging a joy stream. What makes that safe is the pairing, documented in
`test_localization_watchdogs.py:49`:

> `drive_topic` is remapped away from the real `/drive`: these nodes run with the deadman disabled,
> so an un-remapped test would publish live commands straight into `ackermann_mux` if the driver
> stack were up.

**The rule is therefore the pairing, and it is mechanically checkable:** `enable_deadman:=false` in a
node test MUST appear with `drive_topic:=/test_only/drive` in the same node construction. Audited
2026-08-22: **11 of 11 sites paired**, across `test_opponent_integration.py` (5),
`test_localization_watchdogs.py` (2), `test_auto_map_race.py` (2), `test_pure_pursuit_intent.py` (2).
Never in shipped config; never in a test whose subject *is* the deadman.

### `/drive_intent` diagnostics

Per the safety contract in `docs/drive-intent.md`, tests must assert that when the diagnostic path
raises: (a) the drive command for that tick is still published and unchanged, (b) the diagnostic is
disabled rather than the node dying, and (c) the drive command was published **before** the intent.
Assert the ordering, not just the presence.

### Simulators

Correctness signals come from independent geometry (§A7), never gym's own flag. Note which simulator
a test belongs to: `tools/f1tenth_sim/` tests controller *math* with no ROS; `racerbot_sim` tests the
*wiring* through real topics. A wiring bug is invisible to the first by construction — don't claim
the first covers it.

### Where a test belongs

| What you're testing | Where it goes | What runs it |
|---|---|---|
| Pure logic, no `rclpy` | `src/<pkg>/test/` | `python3 -m pytest src/<pkg>/test/ -v` |
| Node behavior, topics, params | `src/<pkg>/test/`, imports `rclpy` | needs ROS sourced; also `colcon test --packages-select <pkg>` |
| Whole-stack wiring, launch files | `racerbot_sim` | `colcon test` / a sim run |

Keep non-trivial math and parsing importable without `rclpy` — that is what makes it testable at all.

---

## 5. Pre-merge sweep (copy-paste)

```bash
cd ~/racerbot-ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
T="src/drive_intent/test src/gap_follow/test src/odom_calibration/test src/pure_pursuit/test
   src/race_diagnostics/test src/racerbot_sim/test src/usb_cam_stream/test src/web_dashboard/test"

# A1 - every mock hit must assert on behavior, not on the configured return
grep -rn "Mock\|MagicMock\|patch(" $T --include='*.py'

# A3 - numeric literals in assertions with no nearby oracle rationale (new tests)
grep -rnE "assert.*[0-9]+\.[0-9]+" $T --include='*.py' | grep -v "oracle:"

# A5 - trivially-true assertions and swallowed failures
grep -rnE "assert +(True|False|1|0) *(,|$|#)" $T --include='*.py'
grep -rnE "except[^:]*: *(pass|continue) *$"  $T --include='*.py'
grep -rn  "pytest.raises(Exception)"          $T --include='*.py'

# A6 - skips without a reason; assertions/tolerances changed on this branch
grep -rnE "@pytest\.mark\.(skip|xfail)" $T --include='*.py' | grep -v "reason="
grep -rnE "pytest\.(skip|xfail)\( *\)"  $T --include='*.py'
git diff main... -- 'src/*/test/*' 'tools/**' | grep -E '^[-+].*(assert|approx|abs=|rel=|skip|xfail)'

# A8/§4 - deadman deny paths present, and every deadman-off site remapped
grep -rn "deadman" $T --include='*.py'
grep -rln "enable_deadman:=false" $T --include='*.py' | while read f; do
  echo "$f: off=$(grep -c 'enable_deadman:=false' $f) remap=$(grep -c 'drive_topic:=' $f)"
done                                    # the two counts must match, per file
grep -rn "enable_deadman" src --include='*.yaml'        # shipped config: must be true

# A7 - no safety claim resting on gym's own flag alone
grep -rn "collision" src/racerbot_sim tools/f1tenth_sim --include='*.py'

# A9 - everything collects; compare against the raw count of test functions
for p in drive_intent gap_follow odom_calibration pure_pursuit race_diagnostics \
         racerbot_sim usb_cam_stream web_dashboard; do
  echo "== $p: $(grep -rhc '^def test' src/$p/test/*.py | paste -sd+ | bc) defined"
  python3 -m pytest src/$p/test/ --collect-only -q 2>&1 | tail -1
done
```

Then, for anything the diff touched in a math or control path: run the §0 stub swap and at least one
mutant from M1–M6. **Revert every mutation before committing.**

## 6. Reviewer's short checklist

1. For each new or changed test, write the §3 one-liner. Anything HOLLOW blocks the merge.
2. Was any existing test loosened, skipped, or deleted? If so, is the §A6 four-part justification in
   the commit body, pointing at a written spec?
3. Does the diff touch a node that can publish `/drive`? Are all four §4 deadman states covered, and
   is every `enable_deadman:=false` paired with a `drive_topic` remap?
4. Did the author say which runner executes the new tests — and did they source ROS and check the
   collected count (§A9)?
5. For any new golden number: is the oracle stated, and does it name a real source?
