# Follow-the-gap: swept clearance, arc guard and corner setup

> **Who this is for:** anyone about to run `gap_follow` (the reactive, no-map autonomy) after 2026-10-19, or tune it.
> **Read first:** [src/gap_follow/README.md](../src/gap_follow/README.md) — what follow-the-gap is and how the basic pipeline picks a gap.
> **You'll be able to:** say what the three newer features do, turn any of them off, check them in the simulator, and bring them up on the car in the safe order.
> **Time:** 20 minutes to read; the simulator checks take a few minutes each.

In October 2026 `gap_follow` got three additions that change how it drives through corridors. In simulation the car went from hitting walls in 24 of 60 indoor runs to 0 of 60, and lapped about 20% faster. **None of it has been driven on the real car yet.**

## Contents

1. [Highlights](#highlights)
2. [What changed, in plain words](#what-changed-in-plain-words)
3. [Safety](#safety)
4. [Validation](#validation) — the order to bring it up on the car
5. [Turning a feature off](#turning-a-feature-off)
6. [Checking it in the simulator](#checking-it-in-the-simulator)
7. [Swept-path clearance](#swept-path-clearance)
8. [Arc guard](#arc-guard)
9. [Corner setup](#corner-setup)
10. [Measured results](#measured-results)
11. [Ideas that were tried and dropped](#ideas-that-were-tried-and-dropped)
12. [Keeping the mirror honest](#keeping-the-mirror-honest)

## Highlights

- **Every simulated indoor lap completes with no wall contact.** That is 5 layouts × 2 directions × 6 seeds. Before the change, `indoor_tight` and `asb_sharp` touched a wall on every run.
- **Faster where the old ceiling was wrong.** The `asb_10000` hallway lap dropped from 24.6 s to 18.9 s. The old speed limit read the hallway's *side* walls as if they were in front of the car.
- **More room, not less.** The closest the body came to anything went up on every layout. On `asb_10000` it went from 0.07 m to 0.17 m.
- **Each feature has its own off switch** in `config/gap_follow.yaml`, and the simulator measures what each one contributes.
- **Bounded authority.** Arc guard and corner setup can each move the steering by at most half of `max_steering_angle`. The node refuses to start if they are configured for more.
- **Cheap.** About 1.4 ms per LiDAR scan ([glossary](glossary.md#scan)) on average on the Jetson, and about 2.2 ms when the arc guard searches.
- **Tested against the real node.** A test drives the real ROS node (a running program on the ROS graph — [glossary](glossary.md#node)) and the simulator's copy through the same scans, and requires identical commands to 1e-6.

**Limits:** these numbers come from the simulator only. `max_lateral_accel` was raised from 1.0 to 2.0 m/s² for this work, and that value is simulator-only until it has been run on the floor.

## What changed, in plain words

Follow-the-gap looks at the LiDAR scan (one sweep of distance readings around the front of the car), finds the widest open gap, and points the wheels at it. It has no map. That is the whole idea, and it still is.

Three things went wrong with that in narrow indoor corridors, and each new feature fixes one:

| Problem | Feature | What it does |
|---|---|---|
| The car crawled down empty hallways. The speed limit used the closest thing in a ±30° cone ahead, and in a 1.5 m hallway that cone hits the side walls. | [Swept-path clearance](#swept-path-clearance) | Limits speed by how far the car's **body** can actually drive along the curve it is on before reaching something. |
| Turning toward a gap sometimes swept the rear of the car into an inside corner. | [Arc guard](#arc-guard) | Before steering, checks that the curve the body would follow is clear. If it isn't, it picks the nearest steering angle whose curve is. |
| The car started turning the moment a bend came into view. It reached every corner on the inside line, the tightest it could pick. | [Corner setup](#corner-setup) | On the approach to a turn, moves the car toward the **outside** wall so it can turn in late, the way a racing line does. |

A *curve* or *arc* here means the path the car follows if it holds its current steering. A car with fixed steering drives in a circle, and the arc is the piece of that circle ahead of it.

## Safety

**Hold LB on the F710 the whole time the car moves. Let go and it stops.** These changes do not touch the deadman, the time-to-collision brake, or the emergency stop.

If you test this with `enable_deadman: false`, a wrong steering decision has nothing between it and a wall except you catching the car. Never do that; it is workspace policy ([architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)).

The rule exists because a driving node with a bug does not know it has a bug. LB is the one stop that doesn't depend on the code being right.

**The new code has only run in the simulator. Bring it up wheels-off first, then at low speed** — the exact order is in [Validation](#validation).

The simulator's car is close to this one, but not identical. In particular, its grip and the 2.0 m/s² cornering limit have not been checked on this floor.

## Validation

**Do these in order and do not skip ahead.** This is the workspace's standard order for any changed driving node ([writing-your-own-node.md](writing-your-own-node.md#testing-before-its-on-wheels)).

1. **Simulator.** Run the [checks below](#checking-it-in-the-simulator) and confirm every indoor run passes.
2. **Wheels off the ground.** Put the car on a stand, start the full stack, hold LB, and move your hand or a box around in front of the LiDAR.
   - **Working when:** the wheels turn away from your hand.
   - The node's decision log (one line a second in the `gap_follow_launch.py` terminal) mentions `swept clearance`, `corner setup` or `arc guard` when they act.
3. **Floor, low speed, open space.** Lower the speed first: set `max_speed` to about 1.0 m/s and `max_lateral_accel` back to 1.0. Both can be changed live from the dashboard's tuning panel.
   - Drive a straight corridor, then a single corner, with someone ready to let go of LB.
4. **Raise speed in steps.** Raise `max_lateral_accel` toward 2.0 only after the low-speed runs look right.
   - **If the car slides or runs wide in corners:** the floor has less grip than the simulator. Keep `max_lateral_accel` at the highest value that stayed planted.

If anything looks wrong, turn the suspect feature off (next section) and compare. That is what the separate switches are for.

## Turning a feature off

Each feature has a switch in `src/gap_follow/config/gap_follow.yaml`:

| Switch | Default | Off means |
|---|---|---|
| `enable_swept_clearance` | `true` | Speed is limited by the closest reading in the ±30° forward cone again, as before. |
| `enable_arc_guard` | `true` | The car steers exactly where the gap logic says. |
| `enable_corner_setup` | `true` | No move to the outside before a turn. Corridor centering works as before. |

These switches are **not** live-tunable: edit the YAML and restart `gap_follow_launch.py`. The workspace is built with `--symlink-install`, so no rebuild is needed.

`max_lateral_accel` (default 2.0, was 1.0) is live-tunable from the dashboard.

## Checking it in the simulator

These use the no-ROS simulator ([simulator.md](simulator.md)) and run on the Jetson or a laptop. They never touch the car.

**Any terminal**, from `~/racerbot-ws`:

```bash
# every indoor layout, 6 seeds each, both directions
python3 tools/f1tenth_sim/sweep_gap.py --track indoor --seeds 6
python3 tools/f1tenth_sim/sweep_gap.py --track indoor --seeds 6 --reverse
# the same with one feature off
python3 tools/f1tenth_sim/sweep_gap.py --track indoor --seeds 6 --set enable_arc_guard=false
```

**Working when:** each line reads `pass 6/6 ... contact 0`, and the command exits 0. It takes about 30 seconds on the Jetson.

**If it doesn't:** a `contact` count above 0 means the body touched a wall (the simulator checks this itself; gym's own collision flag is not trusted). Look at that run with the trace tool:

```bash
python3 tools/f1tenth_sim/trace_gap.py --track indoor_tight --plot /tmp/trace.png
```

That writes one row per tick (every speed cap, the chosen gap, side distances, what each feature did) and a plot of the lap.

`--set name=value` overrides any `gap_follow.yaml` parameter for a run. Every other value comes from the YAML itself, so the simulator runs what the car would run.

## Swept-path clearance

**The old speed limit:** the car may drive at most as fast as it can stop in the distance to the closest LiDAR reading within ±30° of straight ahead.

In a 1.5 m hallway, that cone reaches the side walls about 1.25 m away. So on an empty straight the car was capped at about 2.4 m/s by walls it was never going to hit.

**The new limit** uses how far the car's body can drive along its current arc before any part of it touches something. On a straight, that is the distance to the end of the hallway.

It takes the shorter of two arcs: the one the car is steering toward, and the one the steering rack can actually reach this tick. So turning the wheels does not instantly unlock a speed the car cannot use yet.

It only changes the *speed limit*. Creeping, the escape logic and the TTC brake still use the forward cone, exactly as before.

<details>
<summary><b>How the swept distance is computed</b> — click to expand. Skip it unless you're changing <code>gap_logic.swept_path_clearance</code>.</summary>

The car's body is a rectangle `car_width` wide, extending `car_length` from the rear axle (`base_link`). Driving at steering δ, the rear axle follows a circle of radius R = L / tan δ (L = wheelbase).

The body then sweeps a ring around that circle's centre:

- inner edge: R − w/2 (the inside of the rear axle line),
- outer edge: hypot(front, R + w/2) (the outer front corner),

where w is the width plus `swept_clearance_margin` on each side.

Every LiDAR point inside that ring is an obstacle on the path. For each one, the free distance is how far the body's front can travel along the arc before reaching it. The clearance is the minimum over all such points, and `inf` beyond `swept_clearance_max_length` (3.0 m).

A straight path (δ = 0) is the limiting case: a band `w` wide straight ahead.

The function is `swept_path_clearance` in `src/gap_follow/gap_follow/gap_logic.py`. Its tests (`test_gap_logic_corners.py`) check it against hand-built geometry, including points just inside and just outside each edge.

</details>

## Arc guard

The gap logic decides where to point the wheels from the gap's direction alone. It does not check whether the *body* fits along the curve that steering produces.

Usually it does fit. Hugging a wall just before a square corner, it may not: turn toward the opening too early and the rear of the car clips the inner corner.

The arc guard checks the requested arc first. It looks `arc_guard_length` (1.0 m) ahead, or the stopping distance at the current speed if that is longer.

- **Arc clear:** nothing happens. On open track the guard never acts.
- **Arc blocked:** it tries steering angles across the full range (`arc_guard_candidates`, 29 of them) and takes the clear one *nearest* to what was asked.
- **Nothing clear within `arc_guard_max_correction`:** it takes the freest arc within that bound, so it can never swing the wheels further than 0.1745 rad (10°) from the request.

When it acts, the decision log says `arc guard <requested>-><used>rad (requested arc blocked)`. It is off while the car is creeping out of a tight spot.

## Corner setup

Racing drivers take a corner **outside → late apex → outside**: wide before the turn, clipping the inside late, wide again on the exit. That gives the biggest radius, so the most speed and the most room.

Follow-the-gap does the opposite on its own. As a bend comes into view, the gap's middle leans into the turn, so the car drifts to the inside long before the corner. In simulation that put the inner corner about 8 cm from the body on `indoor_tight`, and the TTC brake fired on the way into every corner.

Corner setup adds a small steering bias on the approach that moves the car toward the **outside** wall:

1. **Spot the turn.** On each side, it checks whether LiDAR beams reach well past where that side's wall would be if it carried on straight. If they do, the wall ends there: a turn, or an opening. The signal is the left fraction minus the right.
2. **Check this is a corridor approach.** Both side walls within 1.5–2.5 m, and an end wall ahead between 4.0 m and 1.1 m away. In an open room or on a plain straight it does nothing.
3. **Aim for a point toward the outside wall:** `corner_setup_fraction` (0.6) of the room between the corridor's middle and the outside wall (less half the car and a 10 cm margin). It steers toward that point with the same simple rule corridor centering uses, at most 0.10 rad.
4. **Let go at the turn-in point.** The bias fades out as the end wall gets close (1.6 m → 1.1 m), so the car is free to turn.

Corridor centering fades out while setup is active, so the two never fight. A doorway reads like a small turn while it passes. The other checks and the 0.10 rad bound keep that to a brief nudge.

The decision log shows `corner setup <bias>rad at weight <w>` while it acts.

<details>
<summary><b>Every corner-setup parameter</b> — click to expand. Skip it unless you're tuning the approach.</summary>

All in `config/gap_follow.yaml` under `# --- Corner setup`, with comments there:

| Parameter | Default | Meaning |
|---|---|---|
| `corner_setup_fraction` | 0.6 | how far toward the outside wall to aim, as a fraction of the available room |
| `corner_setup_wall_margin` | 0.10 m | room kept from the outside wall |
| `corner_setup_gain` | 0.6 rad/m | steering per metre of lateral error |
| `corner_setup_max_steering` | 0.10 rad | authority bound (startup refuses more than half of `max_steering_angle`) |
| `corner_setup_zero_signal` / `_full_signal` | 0.03 / 0.12 | turn-signal strength where setup starts / is fully on |
| `corner_setup_sector_lo_deg` / `_hi_deg` | 10 / 80 | the forward-diagonal sector each side is checked in |
| `corner_setup_excess_threshold` | 0.30 m | how far past the projected wall a beam must reach to count |
| `corner_setup_far_zero` / `_far_full` | 4.0 / 2.5 m | fade in as the end wall gets this close |
| `corner_setup_near_full` / `_near_zero` | 1.6 / 1.1 m | fade out by the turn-in point |
| `corner_setup_full_side_distance` / `_zero_side_distance` | 1.5 / 2.5 m | both side walls must be this close (a corridor) |

The node checks `near_zero < near_full <= far_full < far_zero` at startup and refuses to run otherwise.

</details>

## Measured results

Recorded 2026-10-19 on the Jetson, simulator fidelity profile `car`, seeds from 12345. Full numbers are in [gap-follow-sim-results.json](gap-follow-sim-results.json). "Old" is the same code with the three features off and `max_lateral_accel: 1.0`.

| Layout (forward) | Old: passed | Old: lap | New: passed | New: lap | Closest approach, old → new |
|---|---|---|---|---|---|
| `indoor_oval` | 6/6 | 15.9 s | 6/6 | 12.9 s | 0.20 → 0.33 m |
| `indoor_tight` | 0/6 (wall contact) | — | 6/6 | 11.8 s | 0.05 → 0.11 m |
| `indoor_wide` | 6/6 | 20.0 s | 6/6 | 16.3 s | 0.77 → 0.87 m |
| `asb_10000` | 6/6 | 24.6 s | 6/6 | 18.9 s | 0.07 → 0.17 m |
| `asb_sharp` | 0/6 (wall contact) | — | 6/6 | 19.3 s | 0.08 → 0.37 m |

Reverse direction: the same pass counts, laps within 0.1 s. On the three large gym circuits every run passed both before and after; laps went from 125 / 176 / 126 s to 107 / 144 / 110 s (Spielberg / Silverstone / BrandsHatch).

`asb_10000` is the real Applied Science Building 10000-level hallway. `asb_sharp` is the same building with its corners squared off, added for this work because square corners were where the old line failed.

<details>
<summary><b>What each feature contributes on its own</b> — click to expand. Skip it unless you're deciding whether to turn one off.</summary>

Each row turns **one** feature off (or `max_lateral_accel` back to 1.0) and keeps the rest. Forward direction, 6 seeds per layout, recorded 2026-10-19. Every one of these runs still passed with no contact. The old failures needed all three features missing at once.

| Configuration | `indoor_tight` lap / closest / stop ticks | `asb_10000` lap / closest | `asb_sharp` lap / closest |
|---|---|---|---|
| All on (shipped) | 11.8 s / 0.11 m / 19 | 18.9 s / 0.17 m | 19.3 s / 0.37 m |
| `enable_swept_clearance: false` | 13.2 s / 0.12 m / 34 | 22.3 s / 0.16 m | 22.6 s / 0.14 m |
| `enable_arc_guard: false` | 12.4 s / 0.12 m / 35 | 19.4 s / 0.15 m | 19.5 s / 0.18 m |
| `enable_corner_setup: false` | 12.1 s / 0.08 m / 80 | 18.8 s / 0.11 m | 19.4 s / 0.18 m |
| `max_lateral_accel: 1.0` | 15.1 s / 0.09 m / 12 | 23.0 s / 0.13 m | 23.0 s / 0.11 m |

How to read it:

- **Swept clearance is the speed.** Without it the hallway laps lose about 3.5 s.
- **Corner setup is the room.** Without it the closest approach on `indoor_tight` drops from 0.11 m to 0.08 m, and the safety stops fire four times as often.
- **The arc guard matters at square corners.** It halves the stops on `indoor_tight` and doubles the room on `asb_sharp`.
- **`max_lateral_accel` 2.0** is worth about 20% of lap time, and is the one value whose real-floor grip is unchecked.

A "stop tick" is one control tick (25 ms) commanded to zero by a safety layer. Most layouts show 12: that is the standing start.

</details>

## Ideas that were tried and dropped

Two simpler turn detectors for corner setup were built, measured and removed. They are written down so nobody rebuilds them:

- **The direction of the deepest beam.** About 2 m before a square corner, the deepest return in view is the far outer corner of the end wall, so this pointed the *wrong way*.
- **The direction the gap logic is aiming.** It wanders a few degrees either side of straight on a plain hallway. At a square inner corner it leans *away* from the turn, because the safety bubble around the corner clips the gap's inside edge.

The side-opening signal is measured against each side's own wall, so a car sitting off-centre on a straight reads exactly zero.

An earlier experiment, also reverted, is recorded in [gap-follow-improvement-plan.md](gap-follow-improvement-plan.md).

## Keeping the mirror honest

The simulator does not run the ROS node. It runs a copy of the node's per-scan pipeline, `GapController` in `tools/f1tenth_sim/gap_mirror.py`, because the no-ROS simulator has no ROS graph to put a node on.

A copy can drift. Before this work it had: `run_validation.py`'s old `gap_command` still had five stale constants and was missing three whole features, so its numbers described a controller the car didn't run.

Three things now stop that, all in `src/gap_follow/test/test_gap_mirror_parity.py`:

1. **One source of numbers.** The copy reads every parameter from `config/gap_follow.yaml`. A test fails if the YAML gains a parameter the copy neither uses nor lists as deliberately not modelled.
2. **It refuses what it doesn't model.** With `enable_cornering_anticipation: true` it raises instead of silently ignoring it.
3. **Same commands.** A test drives the real `GapFollowNode` and the copy through the same sequence of scans: straights, corners both ways, wall hugging, a blocked arc, a creep. It requires the same steering and speed every tick, to 1e-6.

**If you change `gap_follow_node._scan_step`, change `gap_mirror.GapController` the same way**, then run the test.

**Terminal 1** — with ROS sourced, on an isolated test domain (never the car's):

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest src/gap_follow/test/test_gap_mirror_parity.py -v
```

**Working when:** `3 passed`. A failure names the tick and whether steering or speed differed.

## See also

- [src/gap_follow/README.md](../src/gap_follow/README.md) — the whole follow-the-gap pipeline and every parameter.
- [simulator.md](simulator.md) — the no-ROS simulator these numbers come from.
- [gap-follow-improvement-plan.md](gap-follow-improvement-plan.md) — the original diagnosis that led here.
- [steering-laws.md](steering-laws.md) — the other October 2026 change, to the map-based `pure_pursuit` controller.
