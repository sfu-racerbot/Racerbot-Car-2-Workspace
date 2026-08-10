# Racing autonomy: SLAM, localization, and a pure-pursuit race controller

> **Who this is for:** anyone running, tuning, or trying to understand the map-based race stack. It's the biggest doc here, and it opens with a plain-language summary — read that even if you skip the rest.
> **Read first:** [operations.md](operations.md#racing-with-the-pure-pursuit-stack) for how to actually run it, and [architecture.md](architecture.md) for the safety model.
> **You'll be able to:** explain every stage from SLAM to steering command, and tune the stack without guessing.
> **Time:** an hour to read properly. Every derivation is inside a collapsible block you can skip on a first pass.

This is the algorithm reference for `pure_pursuit`: a race stack that drives from a *saved map of the track*.

It's built on top of two things this car already has — mapping, and [localization](glossary.md#localization) (the car working out where on that map it currently is).

This doc is about *why* it's built this way and *how each algorithm works*. The code is written to be read alongside it — `src/pure_pursuit/pure_pursuit/racing_math.py` above all, which holds every formula below with no ROS in the way.

---

## Highlights

- **Maps a brand-new track and races it from one command, with nobody steering.** `auto_map_race_launch.py` drives the car round reactively, detects the closed lap, records and paces a racing line, then hands control to the race controller. No map saving, no offline tooling, no process restarts, no RViz pose seed.
- **Turns a hand-driven lap into a real speed plan.** An offline pass measures how sharply the track turns at every point and works backwards from each corner, so braking starts *before* the corner rather than at it. Nothing about this needs a solver or a dynamics model.
- **Optionally re-derives the fastest line, not just its speed.** The [minimum-curvature optimizer](#phase-4b-optional-optimize-the-line-itself-not-just-its-speed) finds the geometrically fastest path within the track's real width. Against a bare centerline at Brands Hatch: **94.85 s → 92.30 s** — about half the gain of TU Munich's reference implementation, at a clearance margin the reference wasn't holding.
- **Overtakes, from the [LiDAR](glossary.md#lidar) alone.** No second sensor, no neural network, no radio link to the other car. It subtracts the known map from the live [scan](glossary.md#scan) to find what shouldn't be there, tracks how fast that thing is moving *along the track*, and steers around it when it's catching up.
- **Seven independent safety layers, any one of which can stop the car.** They run every control tick, they're ordered, and a stop is published unshaped — no rate limiter can turn a stop into a slow-down. Even an unhandled exception publishes a stop before it propagates.
- **Refuses to race a line the car can't physically steer.** The supervisor checks the generated racing line against the steering rack's real limit and against the map's own walls, and says no with numbers rather than degrading on track. That check exists because every line this car generated for months was unfollowable for a third of its length.
- **The math is a plain library, not a [ROS node](glossary.md#node).** All of it lives in `racing_math.py` with no ROS imports, so `python3 -m pytest src/pure_pursuit/test/ -v` runs the whole thing on a laptop — no robot, no build, no `rclpy`.

**Honest limits:** the velocity profile is a single-number friction model, not vehicle dynamics. Opponent handling reasons about one car at a time, with no identity tracking across occlusions. There's no defensive driving.

Two known latch-ups are documented below rather than hidden — one fixed, one still open. And a course narrower than about 2.6 m will map and profile cleanly, then still run wide at the first corner, for reasons that are [measured and explained](#how-fast-a-mapped-course-can-actually-be-raced-and-why) rather than mysterious.

### Why it exists

[`gap_follow`](../src/gap_follow/README.md), this car's other driving node, is **reactive**: it looks at the current LiDAR scan, steers at the biggest gap, and repeats — with no memory of the track and no map at all.

That makes it robust and simple. It's also fundamentally short-sighted. It cannot see around a corner, cannot plan a smooth line through an S-curve, and has no notion of "this is a known 90° hairpin, start braking *now*."

On a track you get to drive and map in advance — which is almost every real race — that short-sightedness costs real lap time.

`pure_pursuit` is **map-based**. It knows the whole track in advance as a racing line with a precomputed speed at every point, and it knows exactly where the car is on that line. That lets it brake early, carry more speed through a corner it knows is coming, and drive the same line lap after lap.

The trade-off is honest: it depends on a good map and working localization.

Which is exactly why a LiDAR-based reactive safety net — the same idea `gap_follow` is built on — is still layered underneath it. That net covers everything the map cannot know about: an opponent's car, a spun-out car, debris.

---

## The 60-second version

Skip the equations for a moment. Here's the whole system in plain language, the way you'd explain it to someone who has never touched ROS:

- **Mapping.** Drive the track once — by hand, or let the car do it *itself*, reactively, with nobody touching the steering — so the car ends up with a picture of where the walls are. This is [SLAM](glossary.md#slam): the car draws the map and works out where it is on it, at the same time, from nothing but its own laser scans.
- **Localization.** The car constantly compares what its [LiDAR](glossary.md#lidar) (its spinning laser rangefinder) sees right now against that picture, to work out "where am I, exactly." Like finding your spot on a paper map by matching the shape of the room around you to the shape drawn on the page.
- **Recording a line, then pacing it.** Drive one good lap. A small program then looks at how sharply the track turns everywhere, and works out a sensible speed for every point on it. Slow for tight corners, fast on straights, braking that starts *before* the corner.
- **Driving it.** Imagine picking a spot a little way ahead on the track — the **lookahead point** — and steering toward it, over and over. Go faster or slower depending on how fast you're supposed to be right there. That is the entire control algorithm. It's called **Pure Pursuit**, and that's genuinely all it is.
- **Handling other cars.** Notice something car-shaped in the way. Work out whether you're catching up to it. If you are, aim slightly toward whichever side has more room until you're past — then go straight back to the normal line.
- **Safety, always on top.** No matter what any of the above wants to do, if something gets too close for comfort the car stops or steers around it first and asks questions later. And none of it moves an inch unless a human is holding the LB button.

Everything past this point explains *why* each piece works the way it does, with the actual math. That's useful once you want to tune it, extend it, or just understand it properly — but it isn't required to get the gist.

Two words worth pinning down before you go further, because the rest of the doc leans on them:

- A **racing line** is the specific path around the track you want the car to follow — stored here as a plain list of `(x, y)` points, and after pacing, `(x, y, speed)`. It is not the centre of the track; it's wherever you decided is fast.
- A **velocity profile** is that same list of points with a target speed attached to each one. Generating it is [Phase 4](#phase-4-generate-the-velocity-profile).

---

## The fast path: map and race from one launch

The five phases further down are still the best way to *understand* the stack, and they remain available individually for reusing a saved course. But for a course the car has never seen, the supported route is one command:

**Terminal 2, from `~/racerbot-ws`** (with `bringup_launch.py` already running in terminal 1, and **not** `teleop_launch.py`):

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_launch auto_map_race_launch.py
```

**Working when:** the log reports a closed lap, then a profiled path, then a handover to pure pursuit. Hold LB the entire time — the car does not move without it.

`auto_map_race_node` is a safety-gated command selector and a state machine. In order, it:

1. Forwards cautious `gap_follow` commands while `slam_toolbox` builds the map.
2. Detects a closed lap from the `map -> base_link` transform.
3. Records a second lap after loop closure (the default), then writes and paces that path.
4. Loads the result into an already-running `pure_pursuit_node`.
5. Switches command authority after a stop interval.

It also republishes the SLAM transform as `/slam_pose`, so SLAM keeps localizing the car during the race itself. That removes manual map saving, offline profiling, process restarts, particle-filter configuration, and the RViz pose seed from a first visit to a course.

**The supervisor, `gap_follow` and `pure_pursuit` each keep their own LB deadman gate**, independently. Only the supervisor publishes to the real `/drive`.

The generated map, pose graph, raw path and profiled path are all saved under `~/.ros/racerbot_auto/<timestamp>/`. See [operations.md](operations.md#automatic-map--raceline--race-recommended-for-a-new-course) for usage and overrides.

### What a recorded lap actually looks like

**A live SLAM pose is not a trajectory, and for a long time this pipeline treated it as one.**

`slam_toolbox` re-optimises its pose graph continuously. Every correction moves `map->odom`, which moves the car's map-frame pose *without the car having moved*. Recorded verbatim, those corrections become geometry — phantom corners in a line the car never drove.

Measured on the three laps this car has actually recorded (`~/.ros/racerbot_auto`, 2026-07-27), against a steering rack that reaches 14.9 deg:

| | 195630 | 200103 | 202458 |
|---|---:|---:|---:|
| revolutions in the "lap" | 1.98 | 1.96 | 1.98 |
| median heading change per 0.15m sample | 10.1 deg | 8.8 deg | 15.5 deg |
| 95th-percentile steering demanded | 27.2 deg | 29.7 deg | 30.5 deg |
| peak steering demanded | 35.7 deg | 43.3 deg | 46.7 deg |
| waypoints past the rack limit | 33.5% | 34.3% | 32.5% |
| start/finish seam heading mismatch | 34.8 deg | 38.6 deg | 110.1 deg |

Read the bottom two rows. **Every racing line this car ever generated was physically unfollowable for a third of its length.**

And the seam was a corner up to 110 degrees wide — the very first thing pure pursuit drives, because closure is *detected* at the seam.

`smooth_path(half_window=3)` was the only cleanup in the pipeline, and it is nowhere near enough for that.

**After the fixes below**, on those same three recordings, peak steering demand drops from 2.7–4.0× the rack down to 1.2–1.8×. On a clean simulated lap the pipeline now produces a line needing 12.7 deg of the available 14.9, with **no** waypoint over the limit and a 3.8 deg seam.

<details>
<summary><b>The five causes, and what each fix actually does</b> — the full forensic breakdown. Worth reading before you change anything in <code>recorded_path.py</code> or the closure gate; skippable otherwise.</summary>

**1. Every lap was two laps.**

`minimum_lap_distance` was `20.0` — longer than the roughly 15 m loop this car is driven on. So the closure gate could not open until the car had been round twice.

Two overlapping passes are not a closed line. It doubles back on itself, so `find_nearest_index` can jump between passes, and the three-point curvature estimate becomes meaningless.

Closure is now gated on **accumulated yaw** (`minimum_lap_turn_deg`, 300). By the turning-tangent theorem, one lap of a closed circuit is 360 degrees of turning *whatever its size* — so unlike a distance in metres, it does not have to be told how big the course is. `minimum_lap_distance` drops to `5.0` as a sanity floor.

**2. Localisation corrections were recorded as driving.**

A map-frame pose that moves more than `max_pose_jump_m` (0.12 m, i.e. 4.8 m/s at 40 Hz) between control ticks is SLAM correcting itself, not the car moving.

`LapRecorder` now applies that correction to the *already recorded* points instead — which is what a correction means. The whole map moved, including the part already driven.

The recorded shape stays exactly as driven, and the start pose the closure test measures against stays attached to the same piece of track.

**3. Nothing cleaned the line up.**

`pure_pursuit/recorded_path.py` now trims to one revolution, resamples to uniform spacing, and low-passes the closed loop *in space*.

That discards wiggles shorter than the car's own turning circle, on the grounds that anything tighter is not something the car actually drove.

That's a Fourier low-pass, deliberately, not repeated averaging. Repeated averaging of a closed curve is curve-shortening flow, and it collapses the loop to a point. It was tried first, and reduced a 30 m lap to a 0.0 m dot while reporting zero curvature — because a point is very feasible.

**4. Nothing checked the result.**

The supervisor now refuses to hand pure pursuit a line needing more than `profile_reject_ratio` times the rack limit, or needing more than the rack has over more than `profile_reject_fraction` of the lap. It says so with numbers.

A line the rack cannot follow does not degrade gracefully. It saturates the steering, runs wide, and latches on the emergency stop.

**5. Nothing checked the line against the map.**

This one only showed up once the other four were fixed and the car was actually racing.

Filtering a recorded lap rounds its corners *inward*, toward the chord and therefore toward the wall.

`racing_math.smooth_path`'s own docstring says so, and calls it "a conservative direction for the velocity profile to err in". Which it is — and which is the exact opposite of conservative for the *geometry*, when the corner is already near the car's turning circle.

A measured simulator run finished with the line **0.05 m from a wall**. The car's half-width alone is 0.155 m, so the body was inside it — with peak curvature, seam error and deviation all reporting healthy.

The supervisor now subscribes to `/map` and builds a clearance field from SLAM's own grid.

"Stays `profile_wall_clearance` from a wall" then becomes a constraint on which filter cutoff gets chosen. That distance is 0.20 m — the car's own half-width, plus a little.

That constraint outranks curvature, because a line through a wall is not a slower racing line.

**Why that requirement is capped by what the driven line itself achieved** — and why that is not a softening:

Mapping with other cars on the track paints them into the grid. A moving opponent leaves a smear of occupied cells across a line the ego demonstrably drove.

The two-car scenario produced exactly that: a recorded line measuring 0.00 m of clearance on a lap the car had just completed twice without touching anything. Refusing there would be refusing to believe the lap that happened.

The absolute requirement still catches the real failure, which is the *cleanup* moving the line closer to a wall than driving ever took it. The log says when the cap was applied, and why.

</details>

### Racing a course nobody has surveyed

Two further defaults were wrong for this mode specifically, and both were found by racing the result rather than by reading it.

**`profile_max_brake` was `8.0`.** In `compute_velocity_profile` that number is a claim about how hard the car can *brake* — the backward pass uses it to decide how late the car may still be at speed approaching a corner.

It is **not** a command slew rate, which is what the identically named parameter in `pure_pursuit.yaml` is. Two different things, same name.

`gap_follow.yaml` is only willing to assume `3.0` for this same car. At `8.0` the profile carried speed into the first corner of the first simulated race and put the car in the wall. It is now `3.0`.

**`profile_max_speed` was `4.0` with `profile_max_lateral_accel: 2.5`.** That is the surveyed-track, hand-checked-raceline profile from `pure_pursuit.yaml` — applied to a course discovered thirty seconds earlier at 1 m/s.

Now `2.0` and `1.2`. Both are still overridable, and `auto_map_race_launch.py` takes a `supervisor_config:=` argument so a particular course can have its own file without editing the packaged one.

### How fast a mapped course can actually be raced, and why

This turns out not to be a grip question. It's a **width** question, and the two numbers that decide it were both measured:

- The racing line comes from `gap_follow`, which drives about **0.25–0.35 m from the wall** at a corner, because `car_width/2 + safety_margin` puts it there. Take the car's own 0.155 m half-width out and roughly **0.1–0.2 m** is left over.
- Pure pursuit's cross-track error through a corner near this car's turning circle measured **0.39–0.57 m at 2.5–3.0 m/s**.

Those two do not both fit in a 1.8 m corridor, and no amount of filtering adds room the driven line never had.

So the car maps such a course fine, and generates a clean line for it: peak curvature well inside the rack, zero waypoints over the limit, a 0.6° seam.

And then it runs wide at the first corner and touches the wall.

**That is a true statement about the course, not a remaining defect.** A ~2.6 m corridor absorbs the error. 1.8 m does not.

So the defaults are set for the narrow case, and a course wide enough to carry more speed gets its own `supervisor_config:=` file. Sanity-check the trade with `tools/racerbot_sim/run_auto_map_validation.py --track indoor_oval` before assuming a tight course will race.

### One more: the hard stop had no way out

The racing controller's `emergency_obstacle` tier stopped the car and left it stopped.

That sounds fine until you notice that at zero speed *nothing about the scene changes*. Whatever put something inside the 0.40 m safety cone keeps it there. The stop therefore ends the run wherever it fires — which in the first simulated race was four seconds after the handover, 0.38 m from a wall, for the remaining eighty.

`gap_follow` reached the same conclusion about its own forward reserve on 2026-08-06, and answered it with `escape_creep_speed`.

`pure_pursuit` now has the same answer, kept deliberately narrower. A 0.25 m/s crawl, and only when all three of these hold:

- it's toward an opening genuinely deeper than `emergency_escape_min_gap`;
- the whole body still has `emergency_escape_clearance` of room;
- and it is **never** ahead of the contact or stale-scan stops.

Setting `emergency_escape_speed: 0.0` restores the old behaviour.

### Still open: `off_racing_line` is a third latch

Observed once during simulator validation. Not fixed here, because unlike the other two its trigger is a *localisation* claim rather than a geometric one.

`max_cross_track_error` (1.0 m) stops the car when it is further than that from every waypoint — "lost or kidnapped" — and stopping is the right answer to that.

But the stop is permanent by construction. At zero speed the pose does not change, so the cross-track error does not change, so the stop never clears. In the observed run the car had been tracking the line to 0.09 m at 1.98 m/s, went past 1.0 m about five seconds later, and stayed stopped for the remaining seventy-five seconds of the racing window.

Here's the part worth investigating before a race. The same log line shows pure pursuit reporting `opponent tracked: gap=3.00m` in a **solo** run.

So map-subtraction opponent detection was producing false positives on that fresh SLAM map, and a committed overtake offsets the target 0.35 m sideways. A false overtake pushing the car off its own line is a plausible route to a genuine 1 m cross-track error — and *that* would be the thing to fix, rather than the watchdog.

All of this is exercised end to end by `tools/racerbot_sim/run_auto_map_validation.py` — see [ros-simulator.md](ros-simulator.md).

---

## The five-phase pipeline

```mermaid
flowchart LR
    subgraph offline["Done once per track, before racing"]
        A["Phase 1<br/>Map the track<br/>(slam_toolbox)"] --> B["Phase 2<br/>Localize against<br/>the saved map<br/>(particle_filter)"]
        B --> C["Phase 3<br/>Record a lap<br/>(waypoint_recorder_node)"]
        C --> D["Phase 4<br/>Generate velocity profile<br/>(generate_velocity_profile)"]
    end
    subgraph race["Every race run"]
        B2["Phase 2<br/>Localize<br/>(particle_filter)"] --> E["Phase 5<br/>Race it<br/>(pure_pursuit_node)"]
        F["/scan<br/>(LIDAR)"] -.reactive safety net.-> E
    end
    D -->|"track_profiled.csv"| E
```

Phases 1–4 happen once, before you race, whenever the track is new or has changed.

Phase 5 is what actually drives the car. It's the only one running during the race itself, and it depends on the outputs of all four phases before it: a saved map, a working localization launch, and a profiled `.csv`.

---

## Phase 1: Map the track (SLAM)

**In plain terms:** the car needs a picture of the track before it can race on it, the same way you'd want to walk a new track once before driving it flat-out.

This phase is entirely about building that picture. Nothing here decides how fast or how well the car eventually races.

**SLAM** stands for Simultaneous Localization And Mapping — the car works out where it is *and* draws the map, at the same time, from nothing but its own laser scans and wheel odometry. The output is an [occupancy grid](glossary.md#occupancy-grid): a big array of cells, each one marked free, occupied, or unknown.

Two ways to actually do the lap. Either one produces the same kind of map:

- **By hand** — see [operations.md](operations.md#building-a-map). You drive the car around the track once while `slam_toolbox` builds the grid from `/scan` and `/odom`, then save it with `map_saver_cli`.
- **Autonomously, with nobody steering** — see [operations.md](operations.md#building-a-map-autonomously-no-steering-required). `gap_follow` already drives the car with no map at all, reactively steering into open space.
  - Run it *at the same time* as `slam_toolbox` and the car maps the track by driving itself around it. `slam_toolbox` records the map exactly as it would if a human were driving.
  - Zero new code. The same two existing pieces, run together.

Either way, `pure_pursuit` doesn't touch this step at all. It consumes the same saved map that `particle_filter` already uses.

**"I'm not driving it" still means someone is supervising it.**

This workspace's [mandatory LB-deadman policy](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car) applies to `gap_follow` here exactly as it does everywhere else. A **deadman** is a control that only works while you actively hold it — let go and the thing stops. The car will not move an inch unless a human is holding LB on the physical controller, autonomous driving or not.

"Not driving" means not touching the steering and throttle sticks. It does not mean nobody needs to be there. Think of it as a safety supervisor role, not a driving one.

**Why SLAM at all, if the racing line is what we actually drive?**

Because the racing line alone has no way to know *where the car currently is*. The map is what makes localization (Phase 2) possible, and localization is what lets Phase 5 know "the car is here on the racing line" every control tick.

Without a map there is nothing to localize against, and the racing line is just a shape with no connection to reality.

## Phase 2: Localize against the map (Monte Carlo Localization)

Also unchanged — this is `particle_filter`, already in this workspace, using `range_libc` for fast GPU-accelerated ray casting.

See [operations.md](operations.md#localizing-against-a-saved-map) for the exact launch procedure, **including seeding it with RViz's "2D Pose Estimate"**. Pure pursuit will not drive correctly without that seed, and may drive confidently in the wrong direction.

Phase 5 depends entirely on trusting this output, so here's the concept in one paragraph.

Monte Carlo Localization tracks a cloud of thousands of weighted **particles** — candidate guesses at where the car might be.

Every timestep, each particle is nudged forward by the motion model: `/odom`, the car's own wheel-and-motor estimate of how far it moved.

Each particle is then re-weighted by how well a *simulated* LiDAR scan taken from that particle's position matches the **actual** `/scan` against the known map.

Particles that don't match reality die off. Particles near the true position multiply. The weighted average of the surviving cloud is published as `/pf/viz/inferred_pose` — the single best-guess position and heading that `pure_pursuit_node` subscribes to.

## Phase 3: Record a racing line

New: `waypoint_recorder_node`.

It runs with localization already up (Phase 2) and the car under manual [teleop](glossary.md#teleop) control — teleop being ordinary human driving with the joystick.

It subscribes to `/pf/viz/inferred_pose` and appends the car's `(x, y)` position to a `.csv` file.

A point goes in every time the car has moved at least `min_spacing_m` (default `0.15 m`) since the last one. That filters out the dense cluster of near-duplicate points you'd otherwise collect while stopped or crawling.

The file is opened once and **flushed to disk after every single point**, not just on shutdown. If the Jetson crashes mid-lap you keep everything recorded up to that moment instead of losing the whole lap.

Stop recording with `Ctrl+C` once you're back near your start point. A closed-loop racing line doesn't need to close *exactly*, since Phase 4's smoothing already treats the path as wrapping around.

**Where you drive matters.** This recorded line *is* the racing line — Phase 4 only paces it, it never reshapes it.

So drive close to the line you actually want: hug the inside of corners where appropriate, wide smooth arcs rather than jerky manual corrections. That's what the car repeats, lap after lap.

Unless you run [Phase 4b](#phase-4b-optional-optimize-the-line-itself-not-just-its-speed), which does reshape it.

## Phase 4: Generate the velocity profile

New: the `generate_velocity_profile` command-line tool. Not a ROS node — it's an offline file-processing step, run once per recorded lap, never while the car is moving.

This is the first of the two genuinely interesting algorithms in this stack. It turns a bare `(x, y)` path into an `(x, y, speed)` racing line, by working out how fast the car can safely go at every single point.

It happens in three steps, and the third is the one that matters most for lap time.

<details>
<summary><b>Step 1 — curvature from three points</b> — the circumradius formula, and why it needs no calculus. Read if you're changing how curvature is estimated.</summary>

**Curvature** is just "how sharply is the track bending here" as a single number. Big number, tight corner.

For every waypoint, look at it and its two immediate neighbours — call them $A$, $B$, $C$. There is exactly one circle passing through all three.

A tight corner produces a small circle (small radius, high curvature). A gentle bend produces a large circle. A straight line produces an effectively infinite circle, so zero curvature.

Using the identity that a triangle's area relates to its circumradius $R$ by $\text{area} = \frac{abc}{4R}$, where $a,b,c$ are its side lengths:

$$R = \frac{|AB| \cdot |BC| \cdot |CA|}{4 \cdot \text{area}(A,B,C)} \qquad \kappa = \frac{1}{R} = \frac{4 \cdot \text{area}(A,B,C)}{|AB| \cdot |BC| \cdot |CA|}$$

This needs no calculus and no curve-fitting — just three neighbouring recorded points. Which is exactly why it works directly on a raw, slightly noisy, hand-driven recording.

See `racing_math.estimate_path_curvature()`.

</details>

<details>
<summary><b>Step 2 — cornering speed from a simplified friction circle</b> — where the speed limit at each point comes from, and what the model deliberately ignores.</summary>

A car driving a circular arc of curvature $\kappa$ at speed $v$ experiences lateral acceleration $a_{lat} = v^2 \kappa$. That's plain uniform circular motion — the same thing that pushes you sideways in a car going round a roundabout.

Cap that at the car's actual grip limit $a_{lat,max}$ and solve for $v$:

$$v_{corner} = \min\left(v_{max}, \sqrt{\dfrac{a_{lat,max}}{\kappa}}\right)$$

Tighter corners (bigger $\kappa$) get a lower speed limit automatically, with no per-corner tuning.

This is a *simplified* friction circle. Real tires trade off lateral against longitudinal grip on a combined ellipse, and real chassis have weight transfer, suspension behaviour, and so on.

This model ignores all of that and uses one number, `a_lat_max`, as a conservative stand-in for "how hard can this car actually corner." Tune it empirically — see below.

</details>

<details>
<summary><b>Step 3 — forward/backward smoothing</b> — this is the step that creates real braking zones, and the reason the sweeps run five times. Worth reading even if you skip the other two.</summary>

A raw per-point cornering-speed limit on its own would ask the car to *teleport* from race speed on a straight down to walking pace at a corner's apex, one waypoint before it. Physically impossible.

Two more passes fix that, each capping how much the speed may change between adjacent waypoints a distance $ds$ apart:

- **Forward (acceleration) pass**, left to right: $v_i \leftarrow \min\left(v_i,\ \sqrt{v_{i-1}^2 + 2\, a_{accel,max}\, ds}\right)$
- **Backward (braking) pass**, right to left: $v_i \leftarrow \min\left(v_i,\ \sqrt{v_{i+1}^2 + 2\, a_{brake,max}\, ds}\right)$

**The backward pass is the important one for lap time.** It propagates a corner's low speed limit *backward* along the straight leading into it.

That's what makes the profile tell the car to start braking early enough to actually make the corner — instead of "discovering" the corner's speed limit only once it's already there.

A closed-loop track has no clean starting point to seed these sweeps from. Index 0's "previous" waypoint is the *last* waypoint, whose value isn't finalized on the first sweep. So both passes are repeated `smoothing_passes` times (default 5), to let that start/finish seam converge.

Both passes only ever *lower* a speed, never raise one. So extra passes past convergence are harmless no-ops — which is why the same code path handles open paths too, with no special case.

</details>

**This is not a time-optimal racing line.** A truly time-optimal line solves for the path geometry *and* the speed profile together.

That usually means a nonlinear or QP optimizer over the minimum-curvature path within track bounds — for example TU Munich's open-source [global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization).

Phase 4 doesn't reshape the path at all. It only paces whatever line you drove in Phase 3.

That's a deliberate trade-off: no heavy extra dependencies, no QP solver, a result you can sanity-check by eye, and a lap time that's still competitive if you record a good line by hand. [Phase 4b](#phase-4b-optional-optimize-the-line-itself-not-just-its-speed) is where the reshaping lives if you want it.

### Choosing `a_lat_max` / `a_accel_max` / `a_brake_max` / `v_max`

Exactly like every other speed parameter on this car: **start conservative, raise gradually, re-test wheels-off-ground after every change.**

1. Start with the simulator-validated defaults (`a_lat_max=2.5`, `a_accel_max=3.0`, `a_brake_max=8.0`, `v_max=4.0` — all SI units, m/s and m/s²). These are starting points for physical testing, not measured tire limits.
2. Race a lap. If the car slides or understeers off the racing line in a corner, `a_lat_max` is set higher than the car's actual grip — lower it and regenerate the profile.
3. If the car brakes too late and runs wide entering a corner, `a_brake_max` is set higher than the car can achieve — lower it and regenerate.
4. Only once cornering is solid, raise `v_max` to actually use more of the straights.

**Note the distinction between these four and the controller's own limits**, because it catches people out.

`a_lat_max` / `a_accel_max` / `a_brake_max` / `v_max` shape the *recorded speed profile*. Changing them means regenerating that profile — a rerun of [Phase 4](#phase-4-generate-the-velocity-profile).

`pure_pursuit_node`'s `max_speed`, `max_lateral_accel` and friends are online ceilings applied on top of whatever profile is loaded. Those you can change on a *running* node from the dashboard's [live tuning panel](web-dashboard.md#live-parameter-tuning), which is the fast way to answer "is it the profile or the controller?" between runs.

Lowering `max_speed` there clips the whole profile without re-recording anything.

## Phase 4b (optional): optimize the line itself, not just its speed

Phase 4 answers "how fast can the car drive *this* line". It never asks whether the line is any good.

That's a real ceiling. The recorded lap is wherever you happened to drive, and the racing line is a property of the *track*.

`optimize_raceline` is the tool that closes the gap — same output format, same `waypoints_file` parameter, and no change whatsoever to `pure_pursuit_node` or any of its safety layers.

**Terminal 1, from `~/racerbot-ws`.** The normal path on this car, a saved SLAM map plus a recorded lap:

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 run pure_pursuit optimize_raceline \
    --map maps/my_track.yaml \
    --recorded-lap src/pure_pursuit/waypoints/my_track_raw.csv \
    --output src/pure_pursuit/waypoints/my_track_optimized.csv
```

**Working when:** it writes the output file and reports the feasibility checks passing. If it refuses to write, that's the tool working — see [the safety checks](#the-safety-checks-and-the-one-dial-that-matters) below.

Or from a ready-made centerline in the standard TUM/F1TENTH format (`x_m, y_m, w_tr_right_m, w_tr_left_m`), which the Gym tracks ship.

**Terminal 1, after the same two `source` lines as above:**

```bash
ros2 run pure_pursuit optimize_raceline \
    --centerline Spielberg_centerline.csv --output spielberg.csv
```

**Working when:** same signal — the output file appears and the checks pass.

<details>
<summary><b>The algorithm: iterative minimum curvature</b> — the formulation, the Frenet approximation, and the two mistakes that cost real debugging time. Read before changing <code>raceline_optimizer.py</code>.</summary>

This is the method from Heilmeier et al., *Minimum curvature trajectory planning and control for an autonomous race car* (Vehicle System Dynamics, 2019, [DOI 10.1080/00423114.2019.1631455](https://doi.org/10.1080/00423114.2019.1631455)).

It's the same method behind TUM's [`global_racetrajectory_optimization`](https://github.com/TUMFTM/global_racetrajectory_optimization).

**Why minimum curvature and not minimum lap time.** Genuine time-optimality needs a nonlinear optimizer over path *and* speed together: expensive, and not guaranteed to converge.

Minimum curvature is the standard convex stand-in. It works because cornering speed is $v = \sqrt{a_{lat,max} / \kappa}$ — so minimising curvature raises the speed ceiling everywhere at once.

Heilmeier et al. measure it within a few tenths of a second per lap of the true optimum. It gives up ground only where the limit is engine power rather than grip, which is not this car's problem.

**The formulation.** Write every candidate line as a lateral offset $\alpha(s)$ from the centerline along its normals — one number per waypoint. Staying on the track is then just a box constraint, $-w_{right} \le \alpha \le +w_{left}$.

For that *parallel offset curve*, the Frenet relations give the curvature to first order as

$$\kappa_P \approx \kappa + \alpha'' + \alpha\,\kappa^2$$

Three terms with three plain meanings: the curvature already there; the bending caused by *changing* the offset; and the fact that a fixed offset toward the inside of a corner tightens it.

Minimising $\sum \kappa_P^2$ is then a linear least-squares in $\alpha$ with box constraints — exactly what `scipy.optimize.lsq_linear` solves. So there is no `quadprog` or `cvxpy` to install on the Jetson.

Re-linearising about the answer and re-solving a few times is the "iterative" part, and it's what removes the linearisation error.

**Two details are load-bearing, and both were found the hard way:**

*Each pass must re-parameterise before re-linearising.* Linearising about an already-offset copy of the reference is wrong, because that curve's parameterisation is stretched by $(1 - \alpha\kappa)$.

That factor collapses toward zero wherever the offset approaches the local radius of curvature — i.e. at an apex. The objective then went *up* on every pass after the first, and the line came back with a 6 cm-radius kink in it.

*The $+\alpha\kappa^2$ term must not be dropped.* Linearising the general curvature quotient with its denominator frozen is algebraically tempting, and gets $-2\alpha\kappa^2$ — inverted, not merely inaccurate.

On a circular test track, where the answer is obviously the outer wall since that's the largest circle that fits, it converged confidently on the *inner* wall.

A closed-form case with a known answer is what caught it, which is why one is kept in the tests.

</details>

<details>
<summary><b>Getting a centerline out of a SLAM map</b> — why a recorded lap is the right seed, and the two ways the refinement loop goes unstable. Read if the optimizer's centerline looks wrong.</summary>

The optimizer needs a centerline with a drivable width at every point. A recorded lap is neither of those things.

But it is an excellent *seed*. It's guaranteed to be inside the track, to go round exactly once, and to run in the racing direction — none of which a skeletonisation of the occupancy grid gives you for free.

So `refine_centerline` measures the walls either side of the seed, steps it to the middle of what it measured, and repeats.

Two things make that loop unstable on a real map, and both are handled rather than hoped away.

A pit entry, an unmapped doorway or a hole in a one-cell-thick wall lets a ray escape. That reads as an enormous amount of room on that side and throws the point out of the track — on Spielberg, 3 escaped rays became 13 in four passes.

Blocked cells are therefore dilated by one cell, which closes the diagonal corners a ray can slip through at the cost of one cell of measured width, in the conservative direction. Points whose rays still escape get no vote, and the correction is smoothed and capped.

Where a side genuinely has no wall, the reported width falls back to the map's clearance field rather than the ray cap.

</details>

### The safety checks, and the one dial that matters

`--safety-margin` (default 0.15 m, on top of half the car's padded width) is the fast-versus-safe dial.

**The optimizer will use every centimetre it is given** — that is what it is for. So this is the parameter that stops it apexing on the paint. Raise it, don't lower it.

The finished line is then checked *independently* of anything the optimizer believed:

- **Steering feasibility.** $\kappa_{max}$ against $\tan(\delta_{max})/L$ — 0.821/m, a 1.22 m radius, on this car. A line the rack cannot physically steer is worthless.
- **Wall clearance**, sampled from the map's distance transform along the whole line.

If either check fails, the tool **refuses to write the file**. `--allow-infeasible` downgrades that to a warning for inspection, and says so loudly.

### What it actually buys, measured

In the F1TENTH Gym harness (`--scenario pure --raceline optimized`), against the same track's bare centerline and against the reference TUM raceline that ships with the track:

| Track | Centerline | This optimizer | TUM reference |
|---|---:|---:|---:|
| Brands Hatch | 94.85 s | **92.30 s (−2.7%)** | 90.20 s (−4.9%) |

Brands Hatch is the only one of the three tracks where that comparison is readable, and the reason is worth understanding because **it applies to the real car too**.

`pure_solo` deliberately exercises the *no-map fallback* reactive-avoidance trigger, which fires on anything within 0.7 m. A centerline never comes that close to a wall, so it never triggers.

Any racing line — mine or TUM's — apexes closer than 0.7 m and gets capped to `avoidance_speed` (1.0 m/s) for 20–30% of the lap on Spielberg and Silverstone, which swamps the lap time. Brands Hatch is wide enough that nobody triggers it.

**The operational lesson: a racing line apexes inside the reactive avoidance trigger distance by design.**

On the car that's fine, because the default `opponent_detection_mode: map` subtracts mapped walls before the traffic layer ever sees them. But it means map subtraction has to actually be working before an optimized line is worth anything.

If localization is off or the map is stale, the car will crawl round its own racing line. Check the decision log for avoidance engagements on the first laps.

So: roughly **half the available gain** against the mature reference implementation, at a 0.15 m clearance margin the reference was not holding. That's the honest summary.

The gain against a *hand-recorded* lap — which is what you actually have for your own track — is larger than the centerline figure above, because a recorded lap is a good deal worse than a centerline.

---

## Phase 5: Race it — the Pure Pursuit controller

`pure_pursuit_node` is the only node that runs *during* the race.

Every control tick — default 40 Hz, matching the LiDAR's scan rate — it does exactly two jobs, steer and set speed, followed by a set of independent safety checks that can override either one.

### Why a fixed-rate timer, not the pose callback directly

The subscription callbacks (`pose_callback`, `scan_callback`) only ever *cache* the latest message and its arrival time. The actual driving logic in `control_loop()` runs on a `create_timer()` at a fixed rate instead.

Here's why that matters. If localization died outright and the control loop were driven directly by `pose_callback`, the loop would simply stop being invoked.

The last command published would then stay "live" on `/drive` forever, with nothing left running to notice and stop it.

A timer-driven loop keeps checking "is my data still fresh?" on its own schedule, regardless of whether new sensor data is still arriving. That's what makes a dead sensor feed something the watchdogs below can actually catch.

### Steering: adaptive lookahead + Pure Pursuit geometry

The **lookahead** is the spot on the racing line the car is currently steering toward — a point some distance ahead of it, not the nearest point. Choosing that distance well is most of what makes Pure Pursuit work.

Six steps per tick:

1. **Find the nearest waypoint.** Compute the distance from the car's current `(x, y)` to every waypoint, and take the minimum. (Once running, it searches only a small window near last tick's answer — see *"Why a windowed nearest-point search"* below.) This doubles as the **cross-track error**: how far the car currently is from the racing line.

2. **Pick a lookahead distance that scales with speed:**

   $$L_d = \text{clip}(k \cdot v + L_{min},\ L_{min},\ L_{max})$$

3. **Walk forward from the nearest waypoint** along the recorded path, accumulating segment distances, until $L_d$ has been covered. That waypoint is the steering target.

4. **Transform the target into the car's body frame** — the car's own coordinate system, x forward and y left.

5. **Compute the curvature** of the one circle that gets the car from where it is to that target.

6. **Convert curvature to a steering angle**, clip it to `max_steering_angle`, and rate-limit it against the previous command.

Steps 2 and 5 are the two that carry real reasoning, and both are unpacked below.

<details>
<summary><b>Why the lookahead scales with speed, and why it uses measured speed</b> — the parameter values, and the failure mode of each fixed alternative.</summary>

A *fixed* lookahead is a bad compromise in both directions. Short enough to corner tightly at parking-lot speed, and the car oscillates and overshoots at race speed. Long enough to be smooth at race speed, and it cuts corners at low speed.

Scaling the lookahead with current speed fixes both at once.

Simulator-validated defaults are $L_{min}=0.6\,m$, $L_{max}=1.5\,m$, $k=0.15$ — at the 4.0 m/s speed cap that's a 1.2 m lookahead. The old 2.0 m-at-4 m/s setting cut corners and collided in the dynamics model; see [simulator.md](simulator.md).

**The $v$ here is the car's measured speed** from `/odom` (`odom_topic`) — its own estimate of how fast it's actually going — **not** the profiled target.

The two differ exactly when it matters. While braking into a corner, or while recovering from a safety stop, the profile still says "4 m/s" long before the car is going that fast. Sizing the lookahead off the target would keep aiming far ahead while the car is actually crawling.

If `/odom` is missing or staler than `odom_timeout_sec` (default `0.5 s`), it falls back to the profiled speed at the nearest waypoint — the pre-existing behaviour. The decision log names which one it used, so this is visible rather than silent.

This is a *sizing* input only. It is deliberately **not** a new watchdog, and stale odometry never stops the car on its own.

</details>

<details>
<summary><b>The Pure Pursuit geometry</b> — the body-frame transform, the curvature formula, and the bicycle model. The actual control law, in three equations.</summary>

**Walking the polyline vs. the textbook circle.** Textbook Pure Pursuit intersects the path with a circle of radius $L_d$ centered on the car.

Walking the polyline and snapping to the next recorded point is a simpler approximation, accurate up to the spacing between recorded waypoints. Keep that spacing small — Phase 3's default is 0.15 m — and the difference is negligible.

**Transforming the target into the body frame.** The map frame and the car's body frame (x forward, y left — [REP-103](https://ros.org/reps/rep-0103.html)) differ by the car's current heading $\psi$, its yaw, extracted from the pose's quaternion. Rotating a world-frame offset $(dx, dy)$ into body coordinates:

$$x_{body} = \cos\psi \cdot dx + \sin\psi \cdot dy \qquad y_{body} = -\sin\psi \cdot dx + \cos\psi \cdot dy$$

**Pure Pursuit's curvature formula.** Picture the one circle that passes through the origin (the car's rear axle) *and* through the target at $(x_{body}, y_{body})$, tangent to the car's current heading.

That circle is centered somewhere on the body-frame y-axis, at $(0, R)$. Solving for where it also passes through the target gives:

$$\kappa = \frac{2\, y_{body}}{x_{body}^2 + y_{body}^2}$$

A target to the left ($y_{body}>0$) gives positive curvature; a target to the right gives negative. That matches `AckermannDriveStamped`'s "positive `steering_angle` = left" convention directly, with no sign-flipping needed anywhere.

**Bicycle-model steering angle.** Collapsing the car's front and rear wheel pairs to a single front and single rear wheel — the standard car-like robot approximation — a vehicle with wheelbase $L$ needs a front steer angle of:

$$\delta = \arctan(L \cdot \kappa)$$

That's finally clipped to `max_steering_angle` (default `0.26 rad`, ≈15°, derived from this car's real servo limits below) and rate-limited against the previous command by `max_steering_rate`.

</details>

### Speed: the profile, then two online ceilings

The **base** speed is simply the profiled speed at the car's *current* nearest waypoint — not the steering target's — clipped to `[min_speed, max_speed]` as a hard ceiling independent of whatever the `.csv` says.

Using the car's current position rather than the lookahead target means the speed command reflects "how fast should I be going *right here, right now*". The braking zones baked into the profile by Phase 4 already account for what's coming up.

That profile, however, only knows about the *recorded* line. It has nothing to say about a turn the car is taking that isn't on that line — a correction after a localization jump, a reactive swerve, or the offset arc of an overtake.

Those are precisely the moments the car is asking for its sharpest steering while the profile is still handing it a straight-line speed.

So the final command also passes an **online curvature ceiling**, computed from the steering angle actually being commanded this tick:

$$\kappa_{cmd} = \frac{\tan\delta}{L} \qquad v \le \sqrt{\frac{a_{lat,max}}{|\kappa_{cmd}|}}$$

with `max_lateral_accel` (default `2.5 m/s²`).

It's evaluated on the larger of the requested and the rate-limited curvature, so the slowdown lands as the turn is *asked for*, not after the rack has caught up.

On the recorded line this ceiling is inactive by construction — Phase 4 already profiled that curvature, and did so more permissively, since `a_lat_max` is typically higher. It only ever binds when the car is doing something the offline profile never saw.

A reactive override — `avoidance_speed`, or a hard stop — is treated as a **ceiling too**, not merely a target. It can lower the command instantly.

### Online command shaping

The last stage before publishing bounds how fast a command may *change*, so one noisy tick cannot become a step input at the servo or the motor:

| Limit | Default | Applies to |
|---|---|---|
| `max_steering_rate` | `1.0 rad/s` | steering, both directions |
| `max_acceleration` | `6.0 m/s²` | speed, **rising only** |
| `max_braking_decel` | `8.0 m/s²` | speed, falling, for normal commands |
| `command_slew_max_dt` | `0.10 s` | cap on the `dt` any single slew step integrates |

`command_slew_max_dt` exists so that a stalled control loop followed by a resumed one cannot cash in a large accumulated interval as one big jump.

**The acceleration ramp starts from the car's measured speed, not from the last command.** This matters more than it sounds.

A ceiling — avoidance, curvature, a hard stop — drops the *command* instantly. But the car keeps rolling; it cannot shed 3 m/s in one 25 ms tick.

Ramping back up from that dropped command would hold the throttle far below the car's real speed for the whole climb, actively braking a car that never actually slowed.

So the ramp basis is `max(last command, measured /odom speed)`, clamped to `max_speed` so a bad reading cannot inflate it. It only ever *raises* the basis, every ceiling below still applies, and stale odometry falls back to the old command-based behaviour.

Two properties of this stage are load-bearing and should not be quietly changed:

- **Emergency stops bypass it entirely.** A zero-speed command is published immediately and unshaped, whether it came from the deadman check, the pose timeout, the cross-track watchdog, a missing or stale scan, or the LiDAR hard-stop net.
  - Rate-limiting a stop into a nonzero command would be a safety regression. So the shaping path is only ever reached with a positive desired speed.
- **`max_acceleration` is bounded on both sides, and `6.0` is near the top of the usable band.** It caps how fast a *command* may rise; it is not a demand on the motor.
  - Too low (`3.0`) and the car cannot re-accelerate out of a safety stop behind a slower car — it stalls on track.
  - Too high (`7.0+`) and it arrives behind that slower car too fast for the overtake to commit, hard-stops, and enters a stop-go cycle it never leaves.
  - Re-run the traffic scenario after touching it; see [simulator.md](simulator.md#the-adaptive-speed-work-two-values-the-traffic-scenario-pinned-down).

<details>
<summary><b>Why a windowed nearest-point search</b> — the self-intersecting-track failure it prevents, and why speed is not the reason.</summary>

Some tracks come close to themselves: a hairpin, a figure-eight, a pit lane splitting off the main straight.

There, the *globally* nearest waypoint by raw distance can be on a completely different part of the track from the one the car is on.

Restricting the search to a small window around *last tick's* answer keeps the tracker locked onto the correct branch, instead of teleporting its target across the track. That window is `nearest_search_window`, default 40.

It's also simply faster — O(window) instead of O(N) every tick. But at typical racing-line sizes, a few hundred to a couple of thousand points, that speed difference doesn't actually matter on the Jetson. Correctness at self-intersections is the real reason this exists.

</details>

### The safety layers

Seven independent checks, each capable of unilaterally forcing a stop or a steering override, regardless of what the steering and speed logic above computed. Ordered from "must never be violated" down to "nice to have":

| Check | Triggers when | Why |
|---|---|---|
| **LB deadman button** (checked first, ahead of everything else) | LB not held on a live `/joy` stream within `joy_timeout_sec` (default 0.5s) | **Mandatory workspace policy** — see [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car). Stays on (`enable_deadman: true`) until the team explicitly decides the car's behavior is trustworthy enough to relax it — don't set it `false` otherwise |
| Localization watchdog | No pose received yet, or `pose_topic` has gone quiet for more than `pose_timeout_sec` (default 0.5s) | Never drive on a stale or absent position estimate |
| Cross-track error | Nearest waypoint is farther than `max_cross_track_error` (default 1.0m) | Car is lost, kidnapped, or localization has diverged — the steering geometry would be aiming at a point unrelated to reality |
| Opponent detection + overtake steering | Another car detected and being closed on within `overtake_trigger_gap` (default 3.0m of *track* distance) | Not a safety check at all — a racing one. See [Racing against opponents](#racing-against-opponents-detection-tracking-and-overtaking) below. Always subordinate to the two checks after it |
| Reactive avoidance (steer around) | An unmapped return in the 60° cone is under 1.5m; before a map is ready, raw range is under the 0.7m fallback | Map subtraction prevents ordinary walls from continuously triggering the traffic layer. A committed pass bypasses the generic 1m/s cap, while the emergency tier remains active |
| Emergency hard stop (always wins) | Minimum range in a narrower `safety_fov_deg` cone (default 60°) is under `emergency_stop_distance` (default 0.4m), or `/scan` itself is stale/missing | Last resort, unconditional — a safety net that's gone blind is treated identically to "obstacle detected" |
| Unhandled exception | Anything in the control step raises | `control_loop()` wraps the whole step in try/except; on *any* exception it publishes a stop command *before* re-raising, so an unexpected bug can't leave the last (possibly full-speed) command sitting on `/drive` forever |

**Every stop in this table is published immediately and unshaped.** The acceleration and steering rate limits described in [Online command shaping](#online-command-shaping) sit on the *normal* command path only. A zero-speed command never passes through them, because rate-limiting a stop into a nonzero command would defeat the whole table.

Because the deadman check runs first, holding LB is a precondition for the car moving at all. Releasing it stops the car immediately, regardless of what every other watchdog says.

Concretely, that means **`joy_node` must be up** while racing. It lives in `bringup_launch.py` — the shared foundation every control layer needs — not in `teleop_launch.py`, which is the manual-driving control layer you simply don't launch during a race. See [operations.md](operations.md#racing-with-the-pure-pursuit-stack).

All of this sits *underneath* the same arbitration the rest of this repo uses. `pure_pursuit_node` publishes to `/drive` exactly like `gap_follow` does, and `ackermann_mux` plus the joystick still have final say — see [architecture.md](architecture.md#the-safety-model-read-this-before-writing-autonomy-code).

None of the above replaces wheels-off-ground testing, or a human ready to cut power. See [operations.md](operations.md#racing-with-the-pure-pursuit-stack).

### Where `0.26 rad` comes from

<details>
<summary><b>The derivation</b> — from this car's real servo calibration, and asymmetric. Re-derive it if the servo ever changes.</summary>

This car's actual servo calibration lives in `src/f1tenth_system/f1tenth_stack/config/vesc.yaml`:

```
servo_position = -1.2135 * steering_angle + 0.5304,   servo clamped to [0.15, 0.85]
```

Solving both ends for `steering_angle`: `servo=0.15` gives `+0.313 rad` (≈18°), and `servo=0.85` gives `-0.263 rad` (≈−15°).

The rack is *asymmetric* — it can turn further left than right.

`max_steering_angle` uses the smaller magnitude (`0.26`), so that a command in *either* direction is one the servo can physically achieve, with a small margin.

If this car's `vesc.yaml` gain, offset or servo limits ever change — a different servo, a re-calibration — re-derive this number rather than leaving it stale.

</details>

---

## Racing against opponents: detection, tracking, and overtaking

**In plain terms:** a real racer doesn't just drive their own line and hope. They notice the car ahead, work out whether they're closing the gap or falling behind, and if they're closing it, they look for room to get past instead of following forever.

This section is that same thinking, done with LiDAR and arithmetic instead of eyes and instinct. Three questions, asked every control tick:

1. **"Is that actually a car?"** — look at the live scan for something shaped and sized like an opponent, sitting out in the open track rather than being part of a wall.
2. **"Am I catching them?"** — track how far along the track they are, the same way the car's own progress is already tracked, and compare how fast each is gaining ground.
3. **"Where's the room?"** — if catching them, find whichever side has more space and steer the racing-line target over there until safely past, then merge straight back onto the recorded line.

None of this needs a second sensor, a neural network, or any communication with the other car. It's built entirely from the same `/scan` and racing line every other part of this stack already uses.

### 1. "Is that a car?" — map subtraction, then geometric filtering

The default `map` detector ray-casts what every LiDAR beam *should* hit in the static occupancy map, and compares that to what it actually got back.

A measured return at least `map_subtraction_margin` (default 0.4 m) **shorter** than that prediction is dynamic: something exists there that the map does not explain.

That single test directly removes walls and corners before any clustering happens — including an opponent backed against a wall, which is the case naive clustering handles worst. Every `map_beam_step`-th beam is cast, to control CPU cost.

The remaining dynamic returns still pass the width and range checks below. Map subtraction distinguishes static from dynamic; it does not distinguish race car from debris.

Until `/map` arrives, or when `opponent_detection_mode: heuristic` is selected, the controller falls back to the raw geometric detector described next rather than racing blind.

<details>
<summary><b>The three geometric filtering steps</b> — clustering, chord width, and the car-shaped test. This is the no-map fallback, and the second stage of the map path.</summary>

**Step 1 — group the scan into objects.** Walk the scan and split it into clusters.

A cluster is a run of consecutive readings that are all "something's there" — clearly less than the sensor's max range — and that don't jump by more than a small threshold from one beam to the next.

A big jump between neighbours means a *different* object, even if both readings are close. A car sitting in front of a wall shows up as one cluster for the car, a jump, then a separate cluster for the wall behind it (`cluster_scan_ranges`).

**Step 2 — measure each cluster.** For a cluster spanning `start_idx` to `end_idx`, convert its first and last point to Cartesian coordinates and take the straight-line distance between them — its **chord width**:

$$\text{width} = \sqrt{(x_{end}-x_{start})^2 + (y_{end}-y_{start})^2}$$

That's a far better size estimate than angular width alone, which exaggerates anything close and shrinks anything far away (`cluster_geometry`).

**Step 3 — keep only the ones shaped like a car.** A real opponent, seen from the side or the back, is roughly car-width.

Reject anything narrower than `opponent_min_width` (default 0.15 m — noise, or a thin post) or wider than `opponent_max_width` (default 0.7 m — almost certainly a wall segment).

Then confirm there's clearly *more open space* immediately on both sides of the cluster than the cluster's own distance (`opponent_open_side_margin`). A car sitting in the middle of the track has open track on both sides of it; a bump in a curving wall usually doesn't (`detect_opponent_cluster`).

Among everything that passes every check, the **closest** one wins — the one most immediately relevant to a decision right now.

</details>

The geometric fallback is a heuristic, not certainty. Map mode removes the most common wall false positives, but it is only as accurate as the map and pose alignment underneath it — see [Limitations](#limitations-and-how-to-go-further).

### 2. "Am I catching them?" — tracking progress along the track, not raw position

**In plain terms:** instead of asking "where is the other car in x/y space" — and then having to guess where the track goes from there to predict anything — this asks "how far around the *track* are they."

That's the exact same question already asked about the car itself every tick. Comparing two of those numbers directly answers "am I ahead or behind, and by how much track distance."

Every waypoint on the racing line already has a **cumulative arc length**: the track distance from the start line to that point.

It's computed once at startup by `compute_cumulative_arc_length`, as a running total of `seg_len`.

Find the opponent's *own* nearest waypoint — using the exact same `find_nearest_index` the car uses on itself — and read its arc length.

That gives "how far around the track the opponent currently is", directly comparable to the car's own position, on the same scale.

<details>
<summary><b>Predicting where they'll be</b> — the smoothed progress rate, why this is a Frenet-frame prediction, and how "ahead" wraps around the finish line.</summary>

Predicting where an opponent will be is then just tracking how that arc-length number changes over time.

`OpponentTracker` keeps an exponentially-smoothed estimate of the opponent's **progress rate** — their speed *along the track*, in m/s — from tick to tick:

$$\text{rate} \leftarrow \alpha \cdot \frac{\Delta(\text{arc length})}{\Delta t} + (1-\alpha)\cdot\text{rate}$$

This is a deliberately simple stand-in for what's sometimes called a **Frenet-frame** prediction in more formal autonomous-driving research.

That means reasoning about another vehicle's position and speed *relative to a reference path*, rather than in raw x/y.

Predicting "opponent's arc length one second from now" is then just `arc_length + rate * 1.0`.

That prediction automatically follows the track's own curvature, because it's expressed in track distance. The alternative would be a straight line the opponent has to be assumed to be driving along.

**"Ahead" wraps around the finish line.** On a closed loop, an opponent being "0.3 laps ahead" and "0.7 laps behind" describe the same physical gap looked at from two directions.

`track_progress_gap` always reports the *ahead* distance, wrapping past the start/finish line where needed — so "how close am I to catching them" is always one consistent, positive number.

</details>

### 3. Deciding, and executing, an overtake

An overtake starts when **both** of these are true:

- the opponent is within `overtake_trigger_gap` metres of *track distance* ahead. Not straight-line distance — a hairpin apex might be 1 m away in a straight line but 8 m away along the actual track.
- the car's current profiled speed exceeds the opponent's tracked progress rate by at least `overtake_closing_margin` — i.e. it's actually gaining ground, not just nearby.

Once triggered, **which side to pass on** is decided once, from the same scan that found the opponent. Average the range readings in a small window just past each end of the opponent's cluster, and pick whichever side is more open (`pick_pass_side`).

That reuses exactly the reasoning `gap_follow`'s avoidance logic already uses: finding open space in a scan. It's just applied to "which side of this one object" instead of "which gap in this whole scene."

**Executing the pass doesn't touch the recorded racing line at all.** It nudges the *steering target* sideways instead.

`lateral_offset_point` takes a waypoint ahead and estimates the track's local direction of travel, from it to the next waypoint.

It then offsets that waypoint perpendicular to that direction by `overtake_lateral_offset` metres, toward the chosen side.

Steering is then computed from *that* shifted point using the exact same Pure Pursuit geometry as always. The overtake is really just "aim slightly to one side for a while", not a separate control system.

<details>
<summary><b>Why the offset uses a 4 m preview instead of the normal target</b> — the coupling with the curvature ceiling that made the car brake mid-pass. Read before touching <code>overtake_lookahead_distance</code>.</summary>

The waypoint it offsets is deliberately **not** the normal Pure Pursuit target. It's a longer preview: `overtake_lookahead_distance` (default `4.0 m`) of arc length ahead of the car.

This matters more than it looks.

The normal target is at most `max_lookahead` (`1.5 m`) away. Offsetting a point that close by 0.35 m sideways demands a large curvature — roughly a 0.45 rad heading change, well past the `0.26 rad` steering clamp.

With the [online curvature ceiling](#speed-the-profile-then-two-online-ceilings) in the loop, the controller answers that demand the only way it can: by braking. The car then slows down in the middle of the pass, which is exactly backwards.

In the simulator's traffic scenario this was not a subtle degradation — the ego stalled behind the opponent and covered 0.1 laps in 240 s.

Spread over a 4 m preview, the same 0.35 m offset is a gentle arc the car can hold at speed, and the same scenario completes a clean lap with the pass done.

Because of that coupling, the node **refuses to start** if `overtake_lookahead_distance` is less than `max_lookahead`.

</details>

**Ending the overtake** happens once the car's own arc length is at least `overtake_clear_margin` metres past the opponent's *last known* position.

That comparison uses `racing_math.track_lead_distance`, a signed lead that takes the shorter way round the loop.

That's deliberately not re-checked against a fresh detection every tick. Alongside or just past an opponent, it commonly falls completely out of the forward LiDAR cone, and that must not read as "lost it, panic" rather than "passed it, done."

If the tracked opponent goes stale (`opponent_lost_timeout_sec`, default 1 s, with no update at all) and no overtake is in progress, tracking is simply cleared — nothing to react to.

> **Two known defects in this logic, found in simulation on 2026-08-05. Read both before relying on the overtake.**
>
> **The completion test used to compare `track_progress_gap` against `total_length - overtake_clear_margin`.** Because that gap wraps into `[0, total_length)`, the comparison means "*at most* clear_margin past", not "at least".
>
> It went true the instant the car's nose edged in front, with the two 0.535 m cars still fully overlapped — and the car cut back onto the racing line and sideswiped the opponent 0.45 s later. Fixed by comparing `racing_math.track_lead_distance(...) >= overtake_clear_margin`. **Do not reintroduce the wrapped gap here.**
>
> **`pick_pass_side` returned whichever side was *more* open but never asked whether that side had *enough* room** — and a committed pass disables the avoidance tier (`allow_avoidance=not self.overtake_active`).
>
> The car would commit, steer 0.35 m into a wall, and only react at the 0.4 m emergency stop. Measured forward clearance at contact: 0.19–0.34 m.
>
> Now guarded by `overtake_min_side_clearance` at commit time, plus a speed cap during the pass computed from the *mapped* track edge (`_static_closest_in_cone`). **Do not compute that cap from the raw scan** — the nearest thing ahead during a pass is the car being passed, so the ego would throttle below the opponent and the pass becomes impossible.
>
> Floor-test the overtake before relying on it.

**This always sits underneath the existing reactive safety net, never instead of it.**

If an overtake manoeuvre — or anything else — brings the car within `emergency_stop_distance` of *anything*, the hard-stop tier still wins, unconditionally, regardless of what the overtake logic wanted.

Racing strategy never gets to override safety. See [The safety layers](#the-safety-layers) above.

### Why this design, and not something fancier

A full solution to "race well against an opponent" is a genuinely hard, active research problem — game-theoretic planning, learned opponent models, joint trajectory optimization.

None of that is what's built here, deliberately:

- **No opponent communication or shared telemetry.** This works from `/scan` alone, the same sensor everything else in this stack already depends on. No assumption that the other car is friendly, instrumented, or running compatible software.
- **Single-opponent, not a field.** "The closest qualifying cluster wins" means this reasons about one opponent at a time. A real multi-car pack would need per-object identity tracking — recognizing cluster #3 this tick as the same car as cluster #3 last tick, even after a brief occlusion. A legitimate next step, not attempted here.
- **Map subtraction, not semantic classification.** It reliably excludes mapped walls when localization is aligned, but an unmapped car-sized object can still qualify. The geometric fallback is less selective still.
- **No blocking or defensive manoeuvres.** If an opponent is closing in from *behind*, this stack does nothing different — it just keeps driving its own optimized line.
  - That's a deliberate, safety-conscious choice. Defensive blocking in real racing carries real contact risk, and "drive your own best line consistently" is a legitimate strategy that needs no reasoning about another car's intentions.

---

## Why Pure Pursuit (and not gap_follow alone, and not full MPC)

Three broad options exist for the control layer once you have a racing line:

1. Stay purely reactive — `gap_follow`'s approach, but that throws away the racing line entirely.
2. Pure Pursuit, which is what's implemented here.
3. A full Model Predictive Controller, optimizing steering *and* speed together over a rolling time horizon.

Pure Pursuit was chosen deliberately:

- **Robust and simple to reason about.** The entire control law is two closed-form formulas — curvature, then steering angle. No solver, no iteration, no convergence to worry about, no risk of a control loop silently taking too long and missing a deadline on a resource-limited Jetson Orin Nano.
- **Provably bounded per-tick cost.** A nearest-point search, a short forward walk, and two `atan`s. Comfortably real-time at 40 Hz.
- **Well-understood failure modes.** "Lookahead too short → oscillation, too long → corner-cutting" is a one-line tuning heuristic, not a cost function to re-derive.
- **A genuinely strong track record.** This is the same core algorithm used across a large fraction of competitive F1TENTH/roboracer teams' race stacks — precisely because it's fast enough to trust under race-day pressure.

A full MPC can, in principle, out-perform this by planning several moves ahead and reasoning explicitly about the car's dynamic limits.

But it needs an accurate dynamics model, a QP or NLP solver running fast enough for 40 Hz control on limited hardware, and a lot more that can silently go wrong under time pressure. That's a legitimate next step, not a reason to ship something harder to trust for this iteration.

---

## Parameter reference

All of these live in `src/pure_pursuit/config/pure_pursuit.yaml` — see that file for inline comments too.

<details>
<summary><b>The full parameter table</b> — 51 rows covering every parameter, with defaults and meanings. Open it when you're tuning; the tuning guide below is the faster route to a specific symptom.</summary>

| Parameter | Default | Meaning |
|---|---|---|
| `waypoints_file` | *(required normally)* | Profiled `(x,y,speed)` `.csv`; auto-map mode loads it at runtime |
| `wait_for_waypoints` | `false` | Start stopped awaiting a runtime profile; only the automatic launch sets this |
| `closed_loop` | `true` | Whether the racing line wraps around (a normal lap track) |
| `pose_topic` | `/pf/viz/inferred_pose` | Localization input |
| `scan_topic` | `/scan` | LIDAR input for the reactive safety net |
| `odom_topic` | `/odom` | Measured speed, used only to size the adaptive lookahead |
| `drive_topic` | `/drive` | Output, arbitrated by `ackermann_mux` like every other autonomy node |
| `control_rate_hz` | `40.0` | Control loop frequency |
| `wheelbase` | `0.324` | Traxxas 74276-4 specification in meters; must match `vesc.yaml` |
| `min_lookahead` / `max_lookahead` / `lookahead_speed_gain` | `0.6` / `1.5` / `0.15` | Adaptive lookahead formula, see above |
| `nearest_search_window` | `40` | +/- waypoints searched around last tick's nearest point (`0` = search all) |
| `max_speed` / `min_speed` | `4.0` / `0.5` | Hard safety ceiling/floor, independent of the `.csv` |
| `max_lateral_accel` | `2.5` | m/s²; online cornering ceiling on the *commanded* curvature, see [Speed](#speed-the-profile-then-two-online-ceilings) |
| `max_acceleration` | `6.0` | m/s²; cap on how fast a speed command may *rise*. Deliberately loose — too tight and the car cannot recover from a safety stop |
| `max_braking_decel` | `8.0` | m/s²; cap on how fast a *normal* command may fall. Emergency stops are never rate-limited |
| `max_steering_rate` | `1.0` | rad/s; steering slew limit between commands |
| `command_slew_max_dt` | `0.10` | s; longest interval one slew step may integrate, so a stalled loop can't cash in a jump |
| `max_steering_angle` | `0.26` | rad; derived from this car's real servo limits, see above |
| `odom_timeout_sec` | `0.5` | Lookahead falls back to the profiled speed past this. Not a stop watchdog |
| `pose_timeout_sec` | `0.5` | Localization watchdog |
| `max_cross_track_error` | `1.0` | Lost/kidnapped watchdog, meters |
| `enable_lidar_safety` | `true` | Master switch for the entire reactive net below (avoidance + opponent overtaking both require this too) |
| `safety_fov_deg` | `60.0` | Width of the narrow forward cone checked for the hard emergency stop |
| `emergency_stop_distance` | `0.4` | Meters; hard stop, always wins |
| `scan_timeout_sec` | `0.5` | LIDAR staleness watchdog |
| `enable_obstacle_avoidance` | `true` | Steer around something close instead of just stopping, when there's room |
| `avoidance_fov_deg` | `60.0` | Forward cone used for avoidance steering and opponent gating |
| `avoidance_trigger_distance` | `1.5` | Meters; map-filtered dynamic-object trigger |
| `avoidance_fallback_trigger_distance` | `0.7` | Meters; shorter raw-scan trigger before map subtraction is available |
| `avoidance_min_gap_distance` | `1.0` | Meters; minimum depth for a gap to be considered driveable during avoidance |
| `avoidance_speed` | `1.0` | m/s; capped speed while avoidance steering is active |
| `enable_opponent_overtake` | `true` | See [Racing against opponents](#racing-against-opponents-detection-tracking-and-overtaking). Requires `enable_lidar_safety` too |
| `opponent_min_width` / `opponent_max_width` | `0.15` / `0.7` | Meters; car-shaped cluster width bounds |
| `opponent_cluster_gap` | `0.3` | Meters; range jump that splits one cluster into two |
| `opponent_engagement_range` | `5.0` | Meters; ignore detections farther than this |
| `opponent_open_side_margin` | `0.5` | Meters; how much more open the surroundings must be to count as "isolated" |
| `opponent_velocity_smoothing` | `0.3` | 0-1; exponential smoothing on the tracked progress-rate estimate |
| `opponent_lost_timeout_sec` | `1.0` | Forget the tracked opponent if not re-detected within this long |
| `overtake_trigger_gap` | `3.0` | Meters of *track distance*; start considering a pass this close |
| `overtake_closing_margin` | `0.3` | m/s; must be closing at least this fast to attempt a pass |
| `overtake_clear_margin` | `1.0` | Meters of track distance past the opponent before resuming the racing line |
| `overtake_min_side_clearance` | `0.70` | Meters of room the passing side must have before the car will commit to a pass. `overtake_lateral_offset` + half the car width + margin |
| `overtake_lateral_offset` | `0.35` | Meters; sideways nudge to the steering target while passing |
| `overtake_lookahead_distance` | `4.0` | Meters of arc ahead the offset above is applied to, instead of the normal target. Must be >= `max_lookahead` — the node refuses to start otherwise |
| `opponent_detection_mode` | `map` | Map subtraction by default; `heuristic` is the no-map fallback |
| `map_topic` / `map_beam_step` / `map_subtraction_margin` | `/map` / `4` / `0.4` | Occupancy map, ray-cast downsampling, and residual margin |
| `laser_offset_x` / `laser_offset_y` | `0.33` / `0.0` | Estimated LIDAR mounting offset from `base_link`, used to place detections in the map frame |
| `enable_deadman` | `true` | **Mandatory workspace policy** — LB deadman button, checked first. Leave `true`; see [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car) |
| `joy_topic` | `/joy` | Deadman button input |
| `deadman_button` | `4` | Button index (LB on the F710 in XInput mode) |
| `joy_timeout_sec` | `0.5` | Deadman button staleness watchdog |

</details>

`generate_velocity_profile`'s physical-limit flags (`--v-max`, `--a-lat-max`, `--a-accel-max`, `--a-brake-max`, `--smoothing-passes`) are documented via `--help` and in [Phase 4](#phase-4-generate-the-velocity-profile) above.

## Tuning guide: symptom → likely cause → fix

| Symptom | Likely cause | Fix |
|---|---|---|
| Car oscillates side to side on straights | Lookahead too short | Raise `min_lookahead` and/or `lookahead_speed_gain` |
| Car cuts across the inside of corners | Lookahead too long | Lower `lookahead_speed_gain` and/or `max_lookahead` |
| Car slides/understeers off the line mid-corner | `a_lat_max` set higher than actual grip | Lower `--a-lat-max`, regenerate the profile |
| Car runs wide exiting into a corner (braked too late/too gently) | `a_brake_max` set higher than the car can achieve | Lower `--a-brake-max`, regenerate the profile |
| Car never reaches a satisfying top speed on straights | `v_max`/`max_speed` capped low, or straights too short to reach it (see the lookahead note above) | Raise gradually, re-test wheels-off-ground each time |
| Car stops unexpectedly mid-lap | Cross-track watchdog tripped — localization drifted, bad "2D Pose Estimate" seed, or genuinely off the recorded line | Check RViz's localized pose against reality; re-seed; only loosen `max_cross_track_error` once you've confirmed localization itself is healthy |
| Node refuses to launch | `waypoints_file` unset, missing, or still a raw (no `speed` column) recording | Point it at a *profiled* `.csv`; run `generate_velocity_profile` first |
| Car stops the instant it starts, even in open space | `enable_lidar_safety` is on but no `/scan` has arrived yet (fails safe on purpose) | Confirm `urg_node`/the LIDAR driver is actually running and publishing `/scan` |
| Car swerves at a wall/curve like it's an opponent | A curving wall segment briefly measured as car-width | Narrow `opponent_min_width`/`opponent_max_width`, or raise `opponent_open_side_margin` so only genuinely isolated objects qualify |
| Car never attempts to overtake a slower car ahead | Not closing fast enough, or opponent not detected at all | Check `ros2 topic echo /scan` for a plausible cluster; lower `overtake_closing_margin`; confirm the opponent isn't outside `opponent_engagement_range` |
| Car overtakes then swerves back too early/late | `overtake_clear_margin` mismatched to this car's actual length/handling | Raise it if the pass looks unfinished when it ends, lower it if the car lingers off-line too long after passing |
| Car slows down in the middle of an overtake instead of completing it | The offset passing line is sharper than `max_lateral_accel` allows, so the online curvature ceiling brakes for it | Raise `overtake_lookahead_distance` so the same offset is spread over a gentler arc — prefer this over raising `max_lateral_accel` past real grip |
| Car stops behind traffic and never gets going again | `max_acceleration` too tight to rebuild speed between safety stops | Raise it. It caps how fast a *command* rises, not the motor; `3.0` was measurably too tight in simulation |
| Lap times worse than before the adaptive-speed work | The `max_acceleration` ramp climbing back to speed after every avoidance event (~0.5 s each at `6.0`) | Expected, and free when avoidance isn't firing. Raise `max_acceleration` — solo lap time improves monotonically with it and nothing else measurably degrades. See [simulator.md](simulator.md#current-validated-result) |
| Speed sags on a straight after a localization correction | Online curvature ceiling reacting to the correction's steering | Expected and usually correct. If localization itself is jittery, fix that first; only then loosen `max_steering_rate`/`max_lateral_accel` |
| Steering feels laggy responding to an obstacle | `max_steering_rate` too low | Raise it. Hard stops are unaffected — they bypass rate limiting entirely |

---

## How this wins races

On a track you get to map and drive in advance — true of nearly every real race — the single biggest lap-time lever isn't reaction speed.

It's *carrying more speed through corners you already know are coming*, and *starting to brake at exactly the right moment, every single lap, identically*.

A purely reactive controller re-derives "what should I do right now" from scratch every cycle, with no memory of the track. Which means it can't plan a smooth line through a corner it can't yet see, and it can't consistently reproduce a good line lap after lap.

This stack's whole design is aimed at removing that ceiling: know the track, know exactly where you are on it, and drive the fastest line your tires can actually hold.

And keep a reactive safety net running underneath, for the one thing a map genuinely can't know about — whatever wasn't there when you built it.

## Limitations and how to go further

Being direct about what this *doesn't* do, as a map for where to take it next:

- **The racing line is only as good as the lap you recorded** — *unless* you run [Phase 4b](#phase-4b-optional-optimize-the-line-itself-not-just-its-speed). Phase 4 alone paces your line and never reshapes it; `optimize_raceline` re-derives the geometrically fastest line within the track's actual width.
  - What's still missing is the step beyond minimum curvature. That would be a genuine minimum-*time* optimizer over path and speed jointly, plus a velocity profile that knows which specific corner it's entering rather than only how sharp it is.
- **The velocity profile is a simplified friction-circle model**, not a full vehicle dynamics simulation. No combined lateral/longitudinal tire ellipse, no weight transfer, no slip-angle model.
- **Pure Pursuit doesn't reason about the future beyond one lookahead point.** A full MPC could plan the next N steps jointly against an actual dynamics model — a legitimate, harder next project once this baseline is solid and trusted.
- **Localization is dead-reckoning fused with LiDAR only.** No IMU or wheel-encoder sensor fusion beyond what `particle_filter` and `vesc_to_odom` already do. Better odometry directly means a tighter, more trustworthy `max_cross_track_error`.
- **Opponent detection is map subtraction plus geometry, not semantic recognition.** Mapped walls are excluded, but an unmapped car-sized object can still qualify, and map/pose misalignment can create residuals. The fallback has only cluster geometry.
  - It also reasons about one opponent at a time (closest qualifying cluster wins), with zero identity-tracking across brief occlusions. See [Racing against opponents](#racing-against-opponents-detection-tracking-and-overtaking) for what a sturdier version would need: camera plus learned detection, multi-object tracking, or both.
- **No defensive or blocking driving.** If an opponent is closing from behind, this stack doesn't react any differently. A deliberate, safety-conscious choice, not an oversight — see the same section above.

## File map

```
src/pure_pursuit/
├── pure_pursuit/
│   ├── racing_math.py              # all the math above, framework-agnostic, unit-tested
│   ├── raceline_optimizer.py       # Phase 4b: iterative minimum-curvature line optimization
│   ├── occupancy_map.py            # offline SLAM-map reader: ray casts, clearance field
│   ├── optimize_raceline.py        # Phase 4b CLI: map + recorded lap -> optimized profiled line
│   ├── pure_pursuit_node.py        # Phase 5 — the race controller
│   ├── waypoint_recorder_node.py   # Phase 3 — records a driven lap
│   └── generate_velocity_profile.py # Phase 4 — CLI tool, paces a recorded lap
├── config/
│   ├── pure_pursuit.yaml
│   └── waypoint_recorder.yaml
├── launch/
│   ├── pure_pursuit_launch.py
│   └── waypoint_recorder_launch.py
├── waypoints/
│   └── example_stadium_raw.csv     # synthetic example track — see docs/operations.md
└── test/
    └── test_racing_math.py         # run: python3 -m pytest src/pure_pursuit/test/ -v

src/racerbot_launch/launch/race_launch.py   # localization + pure_pursuit together, race day
```

> **This doc covers the algorithms and the reasoning.** For a code-adjacent, file-by-file reference with every formula and parameter in one place, see [src/pure_pursuit/README.md](../src/pure_pursuit/README.md).
