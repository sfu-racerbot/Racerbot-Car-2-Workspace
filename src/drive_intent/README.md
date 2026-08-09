# `drive_intent`

> **Who this is for:** someone about to read or change this package's code.
> **Read first:** [docs/drive-intent.md](../../docs/drive-intent.md) for what `/drive_intent` is for and the safety contract for publishing it.
> **What's in it:** the shared schema, the trajectory prediction, and the C++ port.

Shared schema and trajectory prediction for `/drive_intent`: the topic on
which a driving node says **what it is trying to do, and why**, so the
[web dashboard](../web_dashboard/README.md) can draw an intent arrow and a
decision panel.

**Workflow, schema reference, safety contract, and the porting guide for
the `racerbot_a` / `racerbot_b` codebases:
[docs/drive-intent.md](../../docs/drive-intent.md).** This README covers
only the code layout.

This package deliberately **does not depend on `rclpy`**. It is pure data
shaping and geometry, which buys three things: it unit-tests without a
robot, it can be imported by any node in the workspace without dragging in
a driving package, and it translates line-for-line into the single-header
C++ port teammates' codebases use. The ROS plumbing — publishers,
parameters, callbacks — lives in the nodes that use it.

## Files

```
drive_intent/
├── predict.py    forward-integrate the intended trajectory
├── schema.py     build / encode / decode / validate the wire format
├── throttle.py   publish rate limiting and failure containment
include/drive_intent/
└── drive_intent.hpp   single-header C++ port of all three of the above
test/
├── test_predict.py    test_schema.py    test_throttle.py
```

### `predict.py`

Rolls a kinematic bicycle model forward from the controller's *desired*
steering and speed. `arc_step()` is the exact constant-curvature update
rather than an Euler step — over a 1.5s horizon at full lock the two
disagree by several centimetres, which is exactly the scale at which
someone is squinting at the arrow to decide whether the car will clip a
cone.

`integrate()` takes `steering_of` / `speed_of` callbacks, which is what
lets one function serve both driving styles here: `gap_follow` passes
constants (it chooses a heading, so its intent really is one arc), while
`pure_pursuit` re-asks the pure pursuit law against the racing line at
every step, so its arrow bends through the corner ahead.

`constant_arc()`, `to_body()`, `polar_to_body()`, and `path_length()` are
the conveniences built on top.

### `schema.py`

`build()` assembles a payload; `encode()`/`decode()` cross the wire;
`validate()` is what every consumer runs on anything it receives.

Two details carry more weight than their size suggests:

- **`bind_min()`** marks the smallest speed ceiling as `binding`. The
  controllers combine their ceilings with `min()`, so the smallest is the
  one in charge — and that single fact is what a final commanded-speed
  number throws away. Ties are all marked rather than picking one.
- **Non-finite values raise.** `json.dumps` writes a bare `NaN` quite
  happily; that is not valid JSON, `JSON.parse` rejects it, and the
  browser loses the whole message stream rather than one arrow. Failing on
  the car is the cheaper failure.

### `throttle.py`

`IntentThrottle` rate-limits publishing, and separately decides when to
attach the `reason` string — on state transitions and on a slow period,
because some reasons are expensive to compute (`gap_follow`'s TTC stop
reason re-runs the entire gap pipeline).

`FailureLatch` disables intent generation after repeated failures. The
alternative — letting an exception escape into a control callback — would
take down a node holding a moving car's steering, to protect a drawing.

### `include/drive_intent/drive_intent.hpp`

A line-for-line C++17 translation of the three modules above, with no
dependency beyond the standard library and `std_msgs`. Installed to
`install/drive_intent/include/drive_intent/` so a C++ node can include it
directly. When you change a module here, change the header too — the
Python side is where the tests are.

## Tests

No ROS, no build, no robot:

```bash
python3 -m pytest src/drive_intent/test/ -v
```

The node-level tests that exercise this package through a real driving
node — including the ones that deliberately break intent generation and
assert the car still drives — live with those nodes:
`src/gap_follow/test/test_gap_follow_intent.py` and
`src/pure_pursuit/test/test_pure_pursuit_intent.py`.
