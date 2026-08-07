# Drive intent: showing what the algorithm is trying to do, and why

> **Who this is for:** anyone adding intent publishing to a driving node, or trying to read the dashboard's decision panel.
> **Read first:** [architecture.md](architecture.md) — this is a diagnostic published *from* driving code, so the safety model matters.
> **You'll be able to:** publish `/drive_intent` from your own node without putting the control path at risk.

The [web dashboard](web-dashboard.md) draws a curved arrow ahead of the
car showing where the driving algorithm **intends** to go, and a panel
explaining **why** it is making the decision it is making right now.

This is deliberately not derived from measured speed or heading. Those
are already on screen, and they only tell you what the car *did*. The
arrow shows the plan the controller is acting on, so a wrong plan can be
caught while it is still only a plan — before the car has driven it into
a wall.

```
    wider  = the plan wants more speed here
    longer = it will cover more ground in the horizon
    curved = re-evaluated along the path, not a frozen tangent

              ______
        ____/‾‾      \____
    [car]                  ▶      solid ribbon : what the algorithm wants
        ‾‾‾‾\______/‾‾‾‾          dashed line  : what the command will do
                                  (the gap between them is command shaping)
```

- **Workflow and what you'll see:** [web-dashboard.md](web-dashboard.md#drive-intent-the-arrow-and-the-decision-panel)
- **This doc:** the `/drive_intent` schema, the safety contract for
  publishers, and the porting guide for `racerbot_a` / `racerbot_b`.

## Contents

- [Why this exists](#why-this-exists)
- [The topic](#the-topic)
- [Safety contract for publishers](#safety-contract-for-publishers-read-this-first)
- [Schema v1](#schema-v1)
- [How the arrow is drawn](#how-the-arrow-is-drawn)
- [Parameters](#parameters)
- [Porting guide: racerbot_a and racerbot_b](#porting-guide-racerbot_a-and-racerbot_b)
- [Testing](#testing)
- [Limitations](#limitations)

## Why this exists

Both driving nodes in this workspace already knew everything shown here.
`gap_follow_node` and `pure_pursuit_node` each funnel every control tick
through a `_log_decision(state, detail, steering, speed)` call, and both
compute their speed ceilings as separate named quantities before taking a
`min()` of them. All of that went to the terminal and nowhere else, which
meant two recurring problems:

1. **Diagnosis was retrospective.** You watched the car do something
   wrong, then went looking for the log line that explained it. The plan
   was only visible after it had been executed.
2. **`min()` throws away the answer.** Four speed ceilings compete every
   tick and exactly one wins. The log printed all four numbers; working
   out which one was actually holding the car back was left to the reader,
   at 40Hz, in a scrolling terminal.

`/drive_intent` publishes both — the predicted trajectory and the named
constraint currently in charge — as structured data a browser can draw.

## The topic

| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `/drive_intent` | `std_msgs/String` (JSON, see [Schema v1](#schema-v1)) | `gap_follow_node`, `pure_pursuit_node`, and any node you add | `web_dashboard` |

**Why JSON in a `String` and not a custom `.msg`.** `racerbot_a` and
`racerbot_b` are separate GitHub repositories
(`sfu-racerbot/racerbot_a`, `sfu-racerbot/racerbot_b`) pulled in as
submodules, and they also build in their teams' own workspaces. A shared
`rosidl` interface package would make all three repos take a build
dependency on this one and rebuild together — a real coupling cost for a
diagnostics feature. A hand-written JSON object costs a C++ node
[one header](../src/drive_intent/include/drive_intent/drive_intent.hpp)
with no dependency beyond `std_msgs`, and it lands in the dashboard's
existing JSON WebSocket protocol with no translation step at all.

The cost, stated plainly: **no compile-time type checking.** The schema
lives in [`schema.py`](../src/drive_intent/drive_intent/schema.py),
`schema.validate()` is the enforcement, and every consumer must treat
incoming messages as untrusted. The dashboard does exactly that — a
malformed message is dropped with a throttled warning and never reaches
the browser.

## Safety contract for publishers (read this first)

`/drive_intent` is diagnostics bolted onto nodes that steer a physical
car. Three rules make that safe, and they are not optional:

**1. Publish intent only *after* the drive command for the tick has
already gone out.** Nothing in the intent path may sit in front of a
command — least of all a stop. In both Python nodes the call is the last
statement of the control path, below `_publish_drive()`.

**2. Wrap the whole thing in one `try`/`except`.** An exception raised by
a diagnostic drawing must never propagate into `scan_callback` or
`control_loop` and take down the node holding the car's steering. After a
run of failures, [`FailureLatch`](../src/drive_intent/drive_intent/throttle.py)
switches intent off, logs once, and lets the node carry on driving. Both
behaviours are tested by deliberately breaking intent generation and
asserting the car still drives — see
[`test_gap_follow_intent.py`](../src/gap_follow/test/test_gap_follow_intent.py).

**3. Read only what the control path already computed; write nothing it
reads back.** Intent is a pure function of the tick. If a future change
makes the control path depend on something intent produced, that is no
longer diagnostics and the LB-deadman rules in
[architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)
apply to it.

Two supporting rules follow from those:

- **Throttle it.** `intent_rate_hz` defaults to 20Hz against a 40Hz
  control rate. No browser needs more, and the Jetson is also driving.
- **Never emit a non-finite number.** `json.dumps` will happily write a
  bare `NaN`, which is not valid JSON; `JSON.parse` rejects it and the
  browser loses *the whole message stream*, not one arrow. Both the
  Python (`allow_nan=False` plus explicit checks) and C++ (`std::isfinite`
  guards) encoders raise instead.

Adding intent to an existing driving node changes no drive-path
arithmetic, but it does change a file that can move the car. Walk the
usual ladder in
[writing-your-own-node.md](writing-your-own-node.md#testing-before-its-on-wheels):
static topic check → wheels off the ground → floor, low speed, open space.

## Schema v1

One JSON object per published decision.

```json
{
  "v": 1,
  "stamp": 1786062657.489,
  "node": "gap_follow_node",
  "frame": "base_link",
  "state": "corner_fallback",
  "severity": "caution",
  "reason": "selected fallback depth gap -21.4deg to +3.2deg; target=1.8m ...",
  "horizon_s": 1.5,
  "desired_steering": 0.2130,
  "commanded_steering": 0.1400,
  "desired_speed": 2.400,
  "commanded_speed": 1.900,
  "path":           [{"x": 0.0, "y": 0.0, "v": 2.4}, ...],
  "commanded_path": [{"x": 0.0, "y": 0.0, "v": 1.9}, ...],
  "factors": [
    {"name": "curve cap",     "value": 2.410, "unit": "m/s", "binding": false},
    {"name": "clearance cap", "value": 3.100, "unit": "m/s", "binding": false},
    {"name": "corner cap",    "value": 1.900, "unit": "m/s", "binding": true},
    {"name": "accel ceiling", "value": 2.050, "unit": "m/s", "binding": false}
  ],
  "targets": [{"kind": "gap_target", "x": 1.761, "y": -0.385}],
  "wedge": {"x": 0.33, "y": 0.0, "a0": -0.3700, "a1": 0.0600, "r": 2.0}
}
```

| Field | Required | Meaning |
|---|---|---|
| `v` | yes | Schema version. Consumers **reject** anything they don't know rather than half-rendering it. |
| `stamp` | yes | Publisher's clock, seconds. The dashboard adds its own receive stamp separately — two clocks, two fields, so a laptop whose clock disagrees with the Jetson's can still tell a stale arrow from a fresh one. |
| `node` | yes | Which node is speaking. Shown in the panel; lets several publishers coexist. |
| `frame` | yes | Always `base_link` in v1. Body frame is what lets the same arrow draw in robot-centric mode (no map, no pose, just `/scan`) and in map-relative mode with no second code path and no TF lookup in the browser. |
| `state` | yes | The controller's own decision state — see [the state list](#decision-states). |
| `severity` | yes | `drive` \| `caution` \| `stop`. Display only; colours the arrow and the chip. |
| `reason` | **no** | The human sentence. Deliberately absent on most messages — see [below](#why-reason-is-usually-absent). |
| `horizon_s` | yes | Seconds the prediction covers. |
| `desired_*` | yes | What the algorithm asked for, *before* slew-rate and acceleration shaping. |
| `commanded_*` | yes | What actually went on the wire this tick. |
| `path` | yes | The intended trajectory: `{x, y, v}` in metres/`m/s`, body frame, first point at the origin. |
| `commanded_path` | yes | Same shape: what the current command will produce. Drawn as the dashed ghost. |
| `factors` | yes | Every constraint that competed. `binding` marks the one(s) in charge. |
| `targets` | yes | Labelled points of interest — `gap_target`, `steering_target`, … |
| `wedge` | no | An angular span to shade: origin `(x, y)`, bearings `a0`→`a1`, radius `r`. `gap_follow` uses it to show the gap it selected. |

Limits enforced by `validate()`: `path`/`commanded_path` ≤ 512 points,
`factors` ≤ 32, `targets` ≤ 32, `reason` ≤ 2000 chars. A buggy publisher
must not be able to make a phone chew through a million-point path at
20Hz.

### `factors` and `binding` — the part worth the most

`factors` is the list of speed ceilings the controller evaluated this
tick. They are combined with `min()`, which is exactly what makes
"smallest one is binding" the correct reading, and exactly what a single
final number throws away.

`bind_min()` marks the smallest — **all** of them on a tie, because
naming one of two equal limits as "the" reason would be a lie of
precision. Only true ceilings belong in the list: `gap_follow`'s
`min_speed` floor is folded into the reported `curve cap` value rather
than listed separately, precisely so it cannot be mistaken for a cap and
wrongly marked binding.

### Why `reason` is usually absent

Reason strings can be expensive. `gap_follow`'s TTC stop reason calls
`_escape_report()`, which re-runs the entire gap pipeline to tell you
whether there is a way out. Paying for that 20 times a second would be
absurd.

So the reason is attached on **state transitions** (the transition *is*
the diagnostic event — it is the moment someone is asking "why did it
just do that?") and then repeated every `decision_log_period_sec`,
mirroring what the terminal log already does. The browser holds the last
reason it saw for the current state, and clears it on a transition so a
stale explanation can never sit under a new state label.

A message with no `reason` key is valid and normal. Treating it as
malformed would drop nearly every message.

### Decision states

Published as-is from each controller, so this list is whatever the code
does — check the source if in doubt.

| Node | States |
|---|---|
| `gap_follow` | `gap_follow`, `corner_fallback`, `emergency_clearance`, `ttc_brake`, `no_safe_gap`, `odometry_stale`, `scan_empty`, `scan_invalid`, `scan_window_empty`, `waiting_for_scan`, `scan_stale`, `waiting_for_joy`, `deadman_released` |
| `pure_pursuit` | `pure_pursuit`, `overtake_left`, `overtake_right`, `lidar_avoidance`, `avoidance_boxed_in`, `avoidance_scan_empty`, `body_contact`, `emergency_obstacle`, `off_racing_line`, `pose_frozen`, `pose_stale`, `waiting_for_pose`, `waiting_for_profile`, `lidar_scan_stale`, `lidar_scan_missing`, `deadman_released`, `control_exception` |

`severity` is derived from the state and the **commanded** speed:
zero speed is always `stop`; `gap_follow` and `pure_pursuit` are `drive`;
everything else is `caution`. Deriving it from the commanded rather than
the desired speed matters — a safety override zeroes the command while
the plan still wants speed, and that must read as a stop.

## How the arrow is drawn

The prediction is a kinematic bicycle model rolled forward over
`intent_horizon_sec`, using the **exact** constant-curvature update rather
than an Euler step. Over a 1.5s horizon at full lock the two disagree by
several centimetres, which is precisely the scale at which someone is
squinting at the arrow to decide whether the car will clip a cone.

**Length** is distance covered over the horizon, so a stopped car draws
nothing and a fast one draws far. **Width** is per-sample half-width
proportional to that sample's planned speed, so the ribbon tapers into a
corner and flares out of it. Both are clamped (`intent_max_length`, and
the width constants in `dashboard.js`) so the arrow stays readable rather
than swallowing the map.

**Curvature** is where the two controllers differ, and it is the whole
reason the arrow is worth drawing:

- **`gap_follow` publishes one constant-curvature arc.** That is the
  honest picture: follow-the-gap produces *a direction to head*, not a
  path to converge onto (see the long comment in `scan_callback`). It also
  publishes the gap it chose as a `wedge` and a `gap_target`.
- **`pure_pursuit` re-runs the pure pursuit law at every integration
  step** against the racing line, advancing along the line by the arc
  length each step covers. So the arrow bends through the corner ahead
  instead of leaving on a tangent — and when the plan is wrong, it shows
  it being wrong before the car gets there. It falls back to a single arc
  whenever following the line is *not* the plan (a committed overtake, a
  reactive override), because drawing a line the controller is currently
  ignoring would show intent it does not have.

The dashed **ghost** is a plain one-tick projection of the command
actually on the wire. Where it separates from the solid ribbon, the gap
*is* the slew-rate and acceleration shaping. Drawing it is what stops the
first person who notices the arrow and the car disagreeing from filing a
bug against the arrow.

A **stop** is not "no intent": `gap_follow` deliberately holds the
steering rack where it is through a stop rather than centring it, because
centring throws away the steering the car needs to get out of trouble.
The dashboard draws that held angle as a short dashed stub with a ring at
the car, so "stopped" and "stopped, aimed left" stay distinguishable.

## Parameters

Identical in `gap_follow.yaml` and `pure_pursuit.yaml`:

| Parameter | Default | Notes |
|---|---|---|
| `publish_intent` | `true` | `false` creates no publisher at all — not even an idle one. |
| `intent_topic` | `/drive_intent` | |
| `intent_rate_hz` | `20.0` | Independent of the control rate. `0.0` means every tick (the way to switch intent *off* is `publish_intent`, never a rate of zero). |
| `intent_horizon_sec` | `1.5` | Long enough to show the plan through a corner; short enough that it stays a claim about *now* rather than a lap projection. |
| `intent_samples` | `16` | |
| `intent_max_length` | `8.0` | Metres. Truncates the drawn path however fast the plan is. |

Dashboard side, in `web_dashboard.yaml`:

| Parameter | Default | Notes |
|---|---|---|
| `intent_topic` | `/drive_intent` | |
| `intent_warn_period_sec` | `5.0` | How often malformed messages are reported. A publisher getting the schema wrong is usually getting it wrong at 20Hz. |

## Porting guide: racerbot_a and racerbot_b

**Each repo now carries its own step-by-step guide, written against its
actual code**, plus a vendored copy of the header:

| Repo | Guide | Vendored header |
|---|---|---|
| `racerbot_a` | `docs/DRIVE_INTENT.md` | `src/reactive/include/drive_intent/` |
| `racerbot_b` | `DRIVE_INTENT.md` | `src/gap_follow_node/include/drive_intent/`, `src/pure_pursuit/include/drive_intent/` |

The section below is the general version; the per-repo guides are more
specific and should be preferred when working in those repos.

Both are separate repositories, so nothing here requires a change to this
workspace. Copy
[`drive_intent.hpp`](../src/drive_intent/include/drive_intent/drive_intent.hpp)
into your package's `include/` (or add
`${drive_intent_DIR}/../include` to your include path — the header is
installed to `install/drive_intent/include/drive_intent/`), add
`std_msgs` to `package.xml` and `CMakeLists.txt`, and follow the worked
example in the header's own comment block.

The header is a line-for-line translation of the Python modules and
carries the same `IntentThrottle` and `FailureLatch`. It is compiled
under `-Wall -Wextra -Wpedantic -Werror` and its output is checked
against the Python validator (see [Testing](#testing)).

### The five-step recipe

1. Create the publisher: `create_publisher<std_msgs::msg::String>("/drive_intent", 10)`.
2. Keep a `drive_intent::IntentThrottle throttle_{20.0, 1.0};` and a
   `drive_intent::FailureLatch latch_;` as members.
3. At the **end** of your control callback, after `drive_pub_->publish(...)`,
   build a `drive_intent::Intent` from locals you already have.
4. Put every speed ceiling you computed into `factors` and pass them
   through `drive_intent::bind_min(...)`.
5. Wrap steps 3–4 in `try`/`catch (const std::exception &)` and feed the
   latch. **Never** let it throw into the callback.

### racerbot_a

- **`src/reactive/src/gap_follow_node.cpp`** — subscribes to
  `reactive::msg::Gap`, so the gap bounds for a `wedge` and a
  `gap_target` are already in hand. Its two modes map cleanly:
  - `drive_best_point()` is a heading choice → `constant_arc(...)`, same
    as this workspace's `gap_follow`.
  - `least_squares_pathfinding()` **fits a polynomial to the gap points**
    — that is already a path. Sample the fitted polynomial straight into
    `intent.path` and set each point's `v` to the speed you are
    commanding. This gives racerbot_a the single best intent arrow of the
    three codebases for almost no new maths, because unlike a constant
    arc it shows the actual curve the fit produced.
- **`src/ftg_node/src/ftg_node.cpp`** — nothing to do yet:
  `FollowTheGapNode::lidar_callback` is currently an empty stub, so there
  is no intent to report. Follow the `drive_best_point()` pattern once it
  is implemented.
- **`src/reactive/src/safety_node.cpp`** — if it can zero the command,
  publish a `stop` intent from it so the dashboard explains the stop.

### racerbot_b

- **`src/gap_follow_node/src/gap_follow_node.cpp`** — the command is a
  clamped `steering` plus a three-branch speed ladder
  (`1.2` / `1.3` m/s depending on `|steering|`). Those branches are your
  `factors`: emit all three with `binding` on the one that fired. Use
  `constant_arc(steering, speed, wheelbase, 1.5, 16, 8.0)` for the path.
  The `lidar_callback` early-return that publishes `speed = 0.0` should
  publish a `stop` intent with a `state` naming *why*.
- **`src/pure_pursuit/src/pure_pursuit_node.cpp`** — already computes
  `closest_index` and `target_index` and already publishes
  `visualization_msgs::Marker` for both, so the data is there. Two
  options, in increasing order of usefulness:
  - quick: `constant_arc(steering_angle, speed, wheelbase_, 1.5, 16, 8.0)`.
  - better: walk `waypoints_` forward from `closest_index`, transform each
    into the body frame with the same rotation `pose_callback` already
    does, and use those as `intent.path` — the racing-line equivalent of
    what `_intent_path()` does in this workspace's `pure_pursuit_node.py`.
- **`src/wall_follow_node/src/wall_follow_node.cpp`** — a constant arc
  plus a `factors` entry for whichever error term is dominating.

### Checklist before you merge it

- [ ] `intent_pub_->publish(...)` is strictly below `drive_pub_->publish(...)`.
- [ ] The whole block is inside `try`/`catch`, and the catch never rethrows.
- [ ] A `FailureLatch` disables intent after repeated failures.
- [ ] Publishing is throttled (~20Hz), not once per control tick.
- [ ] `severity` comes from `classify_severity(state, commanded_speed)`.
- [ ] Nothing in the control path reads anything the intent code wrote.
- [ ] `ros2 topic echo /drive_intent` shows valid JSON with your node's name.

## Testing

```bash
# Pure logic -- no ROS, no build, no robot:
python3 -m pytest src/drive_intent/test/ -v

# Node integration, including the safety rules -- needs the workspace built:
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m pytest src/gap_follow/test/test_gap_follow_intent.py -v
python3 -m pytest src/pure_pursuit/test/test_pure_pursuit_intent.py -v
python3 -m pytest src/web_dashboard/test/test_intent_protocol.py -v
```

The tests worth knowing about, because they are the ones encoding the
claims this feature makes:

- `test_a_broken_intent_builder_does_not_stop_the_car_driving` and
  `test_sustained_intent_failure_switches_intent_off_not_the_node` —
  break intent generation on purpose, assert the car keeps driving.
- `test_the_drive_command_is_published_before_the_intent` — rule 1,
  checked on both the driving path and the stop path.
- `test_the_intent_arrow_shows_the_plan_not_the_acceleration_ramp` —
  the distinction the whole feature rests on. On a clear straight the car
  *wants* max speed from the first scan and only the ramp holds it back,
  so the intent arrow is full length immediately while the ghost grows.
- `test_the_arrow_follows_the_racing_line_round_a_corner` — stops
  `pure_pursuit`'s prediction quietly regressing to a straight tangent.
- `test_the_binding_factor_matches_the_speed_actually_commanded` — the
  panel's central claim, that the highlighted limit is the limit in charge.

To check the C++ header against the Python schema (what the dashboard
actually enforces), compile
`src/drive_intent/include/drive_intent/drive_intent.hpp` into a small
program that calls `encode()` and pipe its output through
`drive_intent.schema.validate`.

## Limitations

- **The arrow is a prediction under the assumption the current command
  holds**, not a promise. It cannot know about a wheel slipping, an
  obstacle that appears next tick, or a parameter you are about to change.
  The ghost line makes the largest source of divergence — command shaping
  — legible rather than mysterious, but the arrow is still a model.
- **`gap_follow`'s arc is a one-tick projection.** It holds the current
  desired steering for the whole horizon, whereas the real controller
  re-decides every scan. It shows where the car is *aimed*, not where
  follow-the-gap will end up after five more decisions.
- **Bandwidth.** Roughly 1KB per message; at 20Hz that is ~20KB/s per
  connected browser. Fine over the LAN, but it is throttled independently
  of the scan feed for a reason — don't raise both.
- **`pure_pursuit`'s prediction assumes the profile speed**, not the
  acceleration-limited speed, when walking the line forward. It answers
  "what shape is the plan", not "when exactly will the car be there".
- **No history scrubbing yet.** The panel keeps the last 20 state
  transitions with timestamps and how long each state held, but you
  cannot click one and see the arrow as it was at that moment. Recording
  `/drive_intent` into the [run diagnostics](run-diagnostics.md) bag and
  replaying it is the natural next step.
