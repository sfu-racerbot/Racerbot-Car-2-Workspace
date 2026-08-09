# Racing autonomy: SLAM, localization, and a pure-pursuit race controller

> **Who this is for:** anyone running, tuning, or trying to understand the map-based race stack. It's the biggest doc here, and it opens with a plain-language summary — read that even if you skip the rest.
> **Read first:** [operations.md](operations.md#racing-with-the-pure-pursuit-stack) for how to actually run it, and [architecture.md](architecture.md) for the safety model.
> **You'll be able to:** explain every stage from SLAM to steering command, and tune the stack without guessing.
> **Time:** an hour to read properly. Sections marked as deep dives can be skipped on a first pass.

This is the algorithm reference for the `pure_pursuit` package: a map-based
race stack built on top of this car's existing SLAM (`slam_toolbox`) and
localization (`particle_filter`) packages. Read
[architecture.md](architecture.md) first if you haven't — this doc assumes
you already know the node graph and the safety model (joystick always wins
arbitration unless you deliberately stop it — see
[architecture.md](architecture.md#the-safety-model-read-this-before-writing-autonomy-code)).

For exact commands, see [operations.md](operations.md#racing-with-the-pure-pursuit-stack).
This doc is about *why* it's built this way and *how the algorithm works*,
line of reasoning by line of reasoning — the code itself
(`src/pure_pursuit/pure_pursuit/racing_math.py` above all) is written to be
read alongside this. For a more code-adjacent, file-by-file reference with
every formula and parameter in one place, see
[src/pure_pursuit/README.md](../src/pure_pursuit/README.md).

## Quick intuition (read this first if any of the below looks intimidating)

Skip the equations for a second — here's the whole system in plain
language, the way you'd explain it to someone who's never touched ROS:

- **Mapping:** drive the track once — by hand, or let the car do it
  *itself*, reactively, with nobody touching the steering (see Phase 1)
  — so the car ends up with a picture of where the walls are.
- **Localization:** the car constantly compares what its LIDAR sees right
  now against that picture to work out "where am I, exactly." Like
  finding your spot on a paper map by matching the shape of the room
  around you to the shape drawn on the page.
- **Recording a line, then pacing it:** drive one good lap, then a small
  program looks at how sharply the track turns everywhere and works out
  a sensible speed for every point on it — slow for tight corners, fast
  on straights, with braking that starts *before* the corner instead of
  right at it.
- **Driving it (Pure Pursuit):** imagine always picking a spot a little
  way ahead on the track and steering toward it, over and over, faster
  or slower depending on how fast you're supposed to be going right
  there. That's the entire control algorithm — see "Pure Pursuit, in
  plain terms" below.
- **Handling other cars:** notice something car-shaped in the way, work
  out whether you're catching up to it, and if so, aim slightly toward
  whichever side has more room until you're past it — then go straight
  back to the normal line.
- **Safety, always on top:** no matter what any of the above wants to do,
  if something gets too close for comfort, the car stops or steers
  around it first and asks questions later.

Everything past this point explains *why* each piece works the way it
does, with the actual math — useful once you want to tune it, extend it,
or just understand it properly, but not required to get the gist.

## Why this exists alongside `gap_follow`

`gap_follow` is *reactive*: it looks at the current LIDAR scan and steers at
the biggest gap, every cycle, with no memory of the track and no map. That
makes it robust and simple, but it is fundamentally short-sighted — it
cannot see around a corner, cannot plan a smooth line through an S-curve,
and has no notion of "this is a known 90° hairpin, start braking now." On a
track you get to drive/map in advance (i.e. almost every real race), that
short-sightedness costs real lap time.

`pure_pursuit` is *map-based*: it knows the whole track in advance as a
racing line with a precomputed speed at every point, and it knows exactly
where the car is on that line via localization. That lets it brake early,
carry more speed through corners it knows are coming, and drive the same
optimized line lap after lap. The trade-off is that it depends on a good
map and working localization — which is exactly why the LIDAR-based
reactive safety net (borrowed from the same idea as `gap_follow`) is still
layered underneath it, for anything the map doesn't know about (an
opponent's car, a spun-out car, debris).

## Automatic path: map and race from one launch

The five phases below remain available individually and are still the best way
to understand or reuse a saved course. For a new course, the supported automatic
composition is:

```bash
ros2 launch racerbot_launch auto_map_race_launch.py
```

`auto_map_race_node` is a safety-gated command selector and state machine. It
forwards cautious `gap_follow` commands while `slam_toolbox` maps, detects a
closed lap from the `map -> base_link` transform, records a second lap by
default after loop closure, writes and profiles that path, loads it into an
already-running `pure_pursuit_node`, then switches command authority after a
stop interval. It also republishes the SLAM transform as `/slam_pose`, so SLAM
continues to localize during racing. This removes manual map saving, offline
profiling, process restarts, particle-filter configuration, and the RViz pose
seed from a first visit to a course.

The supervisor, gap follow, and pure pursuit each retain their LB deadman gate;
only the supervisor publishes to the real `/drive`. The generated map, pose
graph, raw path, and profiled path are saved under
`~/.ros/racerbot_auto/<timestamp>/`. See [operations](operations.md#automatic-map--raceline--race-recommended-for-a-new-course) for usage and overrides.

### What a recorded lap actually looks like

**A live SLAM pose is not a trajectory, and for a long time this pipeline
treated it as one.** `slam_toolbox` re-optimises its pose graph
continuously; every correction moves `map->odom`, which moves the car's
map-frame pose without the car having moved. Recorded verbatim, those
corrections become *geometry*.

Measured on the three laps this car has actually recorded
(`~/.ros/racerbot_auto`, 2026-07-27), against a steering rack that reaches
14.9 deg:

| | 195630 | 200103 | 202458 |
|---|---:|---:|---:|
| revolutions in the "lap" | 1.98 | 1.96 | 1.98 |
| median heading change per 0.15m sample | 10.1 deg | 8.8 deg | 15.5 deg |
| 95th-percentile steering demanded | 27.2 deg | 29.7 deg | 30.5 deg |
| peak steering demanded | 35.7 deg | 43.3 deg | 46.7 deg |
| waypoints past the rack limit | 33.5% | 34.3% | 32.5% |
| start/finish seam heading mismatch | 34.8 deg | 38.6 deg | 110.1 deg |

Every racing line this car ever generated was physically unfollowable for
a third of its length, and the seam -- the first thing pure pursuit drives,
because closure is *detected* at the seam -- was a corner up to 110 degrees
wide. `smooth_path(half_window=3)` was the only cleanup, and it is nowhere
near enough for that.

Four separate causes, all now fixed:

1. **Every lap was two laps.** `minimum_lap_distance` was `20.0`, longer
   than the roughly 15m loop this car is driven on, so the closure gate
   could not open until the car had been round twice. Two overlapping
   passes are not a closed line: it doubles back on itself, so
   `find_nearest_index` can jump between passes and the three-point
   curvature estimate is meaningless. Closure is now gated on accumulated
   yaw (`minimum_lap_turn_deg`, 300) -- by the turning-tangent theorem one
   lap of a closed circuit is 360 degrees of turning *whatever its size*,
   so unlike a distance in metres it does not have to be told how big the
   course is. `minimum_lap_distance` drops to `5.0` as a sanity floor.
2. **Localisation corrections were recorded as driving.** A map-frame pose
   that moves more than `max_pose_jump_m` (0.12m, i.e. 4.8 m/s at 40Hz)
   between control ticks is SLAM correcting itself. `LapRecorder` now
   applies that correction to the *already recorded* points instead --
   which is what a correction means: the whole map moved, including the
   part already driven. The recorded shape stays exactly as driven, and
   the start pose the closure test measures against stays attached to the
   same piece of track.
3. **Nothing cleaned the line up.** `pure_pursuit/recorded_path.py` now
   trims to one revolution, resamples to uniform spacing, and low-passes
   the closed loop *in space*, discarding wiggles shorter than the car's
   own turning circle -- anything tighter is not something the car drove.
   (A Fourier low-pass, not repeated averaging: repeated averaging of a
   closed curve is curve-shortening flow and collapses the loop to a
   point. It was tried first, and reduced a 30m lap to a 0.0m dot while
   reporting zero curvature, because a point is very feasible.)
4. **Nothing checked the result.** The supervisor now refuses to hand
   pure pursuit a line needing more than `profile_reject_ratio` times the
   rack limit, or needing more than the rack has over more than
   `profile_reject_fraction` of the lap, and says so with numbers. A line
   the rack cannot follow does not degrade gracefully -- it saturates the
   steering, runs wide, and latches on the emergency stop.
5. **Nothing checked the line against the map.** This one only showed up
   once the other four were fixed and the car was actually racing.
   Filtering a recorded lap rounds its corners *inward*, toward the chord
   and therefore toward the wall -- `racing_math.smooth_path`'s own
   docstring says so, and calls it "a conservative direction for the
   velocity profile to err in", which it is, and the exact opposite of
   conservative for the *geometry* when the corner is already near the
   car's turning circle. A measured simulator run finished with the
   finished line **0.05m from a wall** -- the car's half-width alone is
   0.155m, so the body was inside it -- with peak curvature, seam error
   and deviation all reporting healthy. The supervisor now subscribes to
   `/map`, builds a clearance field from SLAM's own grid, and makes
   "stays `profile_wall_clearance` (0.20m -- the car's own half-width plus
   a little) from a wall" a constraint on which filter cutoff is chosen,
   outranking curvature, because a line through a wall is not a slower
   racing line.

   The requirement is capped by **what the driven line itself achieved**,
   and that is not a softening: mapping with other cars on the track paints
   them into the grid, so a moving opponent leaves a smear of occupied
   cells across a line the ego demonstrably drove. The two-car scenario
   produced exactly that -- a recorded line measuring 0.00m of clearance on
   a lap the car had just completed twice without touching anything.
   Refusing there would be refusing to believe the lap that happened. The
   absolute requirement still catches the real failure, which is the
   *cleanup* moving the line closer to a wall than driving ever took it,
   and the log says when the cap was applied and why.

On the same three recordings, the cleanup takes peak demand from 2.7-4.0x
the rack down to 1.2-1.8x. On a clean simulated lap it produces a line
needing 12.7 deg of the available 14.9, with **no** waypoint over the
limit and a 3.8 deg seam.

### Racing a course nobody has surveyed

Two further defaults were wrong for this mode specifically, and both were
found by racing the result rather than by reading it:

- **`profile_max_brake` was `8.0`.** In `compute_velocity_profile` that is
  a claim about how hard the car can *brake* -- the backward pass uses it
  to decide how late the car may still be at speed approaching a corner --
  not a command slew rate, which is what the identically named parameter
  in `pure_pursuit.yaml` is. `gap_follow.yaml` is only willing to assume
  `3.0` for this same car. At `8.0` the profile carried speed into the
  first corner of the first simulated race and put the car in the wall;
  it is now `3.0`.
- **`profile_max_speed` was `4.0` with `profile_max_lateral_accel: 2.5`.**
  That is the surveyed-track, hand-checked-raceline profile from
  `pure_pursuit.yaml`, applied to a course discovered thirty seconds
  earlier at 1 m/s. Now `2.0` and `1.2`. Both are still overridable, and
  `auto_map_race_launch.py` takes a `supervisor_config:=` argument so a
  particular course can have its own file without editing the packaged one.

### How fast a mapped course can actually be raced, and why

Not a grip question — a **width** question, and the two numbers that decide
it were both measured:

- the racing line comes from `gap_follow`, which drives about **0.25–0.35m
  from the wall** at a corner, because `car_width/2 + safety_margin` puts
  it there. Take the car's own 0.155m half-width out and roughly
  **0.1–0.2m** is left over;
- pure pursuit's cross-track error through a corner near this car's
  turning circle measured **0.39–0.57m at 2.5–3.0 m/s**.

Those do not both fit in a 1.8m corridor, and no amount of filtering adds
room the driven line never had. The car maps such a course fine, generates
a clean line for it (peak curvature well inside the rack, zero waypoints
over the limit, a 0.6° seam) — and then runs wide at the first corner and
touches the wall. **That is a true statement about the course, not a
remaining defect.** A ~2.6m corridor absorbs the error; 1.8m does not.

So: the defaults are set for the narrow case, and a course wide enough to
carry more speed gets its own `supervisor_config:=` file. Sanity-check the
trade with `tools/racerbot_sim/run_auto_map_validation.py --track
indoor_oval` before assuming a tight course will race.

### One more: the hard stop had no way out

The racing controller's `emergency_obstacle` tier stopped the car and left
it stopped. At zero speed nothing about the scene changes, so whatever put
something inside the 0.40m safety cone keeps it there -- the stop ends the
run wherever it fires, which in the first simulated race was four seconds
after the handover, 0.38m from a wall, for the remaining eighty.

`gap_follow` reached the same conclusion about its own forward reserve on
2026-08-06 and answered it with `escape_creep_speed`. `pure_pursuit` now
has the same answer, kept deliberately narrower: a 0.25m/s crawl, only
toward an opening genuinely deeper than `emergency_escape_min_gap`, only
while the whole body still has `emergency_escape_clearance` of room, and
never ahead of the contact or stale-scan stops. `emergency_escape_speed:
0.0` restores the old behaviour.

### Still open: `off_racing_line` is a third latch

Observed once during simulator validation, and not fixed here because
unlike the other two its trigger is a *localisation* claim rather than a
geometric one.

`max_cross_track_error` (1.0m) stops the car when it is further than that
from every waypoint -- "lost or kidnapped", and stopping is the right
answer to that. But the stop is permanent by construction: at zero speed
the pose does not change, so the cross-track error does not change, so the
stop never clears. In the observed run the car had been tracking the line
to 0.09m at 1.98m/s, went past 1.0m about five seconds later, and stayed
stopped for the remaining seventy-five seconds of the racing window.

Worth knowing, and worth investigating before a race: the same log line
shows pure pursuit reporting `opponent tracked: gap=3.00m` in a **solo**
run, so map-subtraction opponent detection was producing false positives on
that fresh SLAM map, and a committed overtake offsets the target 0.35m
sideways. A false overtake pushing the car off its own line is a plausible
route to a genuine 1m cross-track error, and would be the thing to fix
rather than the watchdog.

All of this is exercised end to end by
`tools/racerbot_sim/run_auto_map_validation.py` -- see
[ros-simulator.md](ros-simulator.md).

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

Phases 1–4 happen once, before you race, whenever the track is new or has
changed. Phase 5 is what actually drives the car; it's the only one running
during the race itself, and it depends on the outputs of all four phases
before it (a saved map, a working localization launch, and a profiled
`.csv`).

---

## Phase 1: Map the track (SLAM)

**In plain terms:** the car needs a picture of the track before it can
race on it, the same way you'd want to walk a new track once before
driving it flat-out. This phase is entirely about building that picture
— nothing here decides how fast or how well the car eventually races.

Two ways to actually do this lap, either one produces the same kind of
map:

- **By hand** — see [operations.md](operations.md#building-a-map). You
  drive the car around the track once by hand while `slam_toolbox`
  builds an occupancy grid map from `/scan` and `/odom`, then save it
  with `map_saver_cli`.
- **Autonomously, with nobody steering** — see
  [operations.md](operations.md#building-a-map-autonomously-no-steering-required).
  `gap_follow` already drives the car with no map at all, reactively
  steering into open space; running it *at the same time* as
  `slam_toolbox` means the car maps the track by driving itself around
  it, with `slam_toolbox` recording the map exactly as it would if a
  human were driving. **A human still has to hold LB the entire time** —
  see the note below — but nobody touches the steering stick. This needs
  zero new code: it's the same two existing pieces, just run together.

Either way, `pure_pursuit` doesn't touch this step at all; it consumes
the same saved map that `particle_filter` already uses.

**"I'm not driving it" still means someone is supervising it.** This
workspace's [mandatory LB-deadman
policy](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)
applies to `gap_follow` here exactly as it does everywhere else: the car
will not move an inch unless a human is actively holding LB on the
physical controller, autonomous driving or not. "Not driving" means not
touching the steering/throttle sticks — it does not mean nobody needs to
be there. Think of it as a safety supervisor role, not a driving one.

**Why SLAM at all, if the racing line is what we actually drive?** Because
the racing line alone has no way to know *where the car currently is*. The
map is what makes localization (Phase 2) possible, and localization is what
lets Phase 5 know "the car is here on the racing line" every control tick.
Without a map, there is nothing to localize against, and the racing line is
just a shape with no connection to reality.

## Phase 2: Localize against the map (Monte Carlo Localization)

Also unchanged — this is `particle_filter`, already in this workspace,
using `range_libc` for fast GPU-accelerated ray casting. See
[operations.md](operations.md#localizing-against-a-saved-map) for the exact
launch procedure (including seeding it with RViz's "2D Pose Estimate" —
**pure_pursuit will not drive correctly, and may drive confidently in the
wrong direction, without this seed step**).

Quick conceptual summary, since Phase 5 depends entirely on trusting this
output: Monte Carlo Localization tracks a cloud of thousands of weighted
"particles" (candidate poses), each nudged forward by the motion model
(`/odom`) every timestep and re-weighted by how well a simulated LIDAR scan
from that particle's pose matches the *actual* `/scan` against the known
map. Particles that don't match reality die off; particles near the true
pose multiply. The weighted average of the surviving cloud is published as
`/pf/viz/inferred_pose` — the single best-guess pose `pure_pursuit_node`
subscribes to.

## Phase 3: Record a racing line

New: `waypoint_recorder_node`. With localization already running (Phase 2)
and the car under manual teleop control, this node subscribes to
`/pf/viz/inferred_pose` and appends the car's `(x, y)` position to a `.csv`
file every time the car has moved at least `min_spacing_m` (default
`0.15m`) since the last recorded point — filtering out the dense cluster of
near-duplicate points you'd otherwise get while stopped or moving slowly.

The file is opened once and **flushed to disk after every single point**,
not just on shutdown — if the Jetson crashes mid-lap, you keep everything
recorded up to that point instead of losing the whole lap. Stop recording
(`Ctrl+C`) once you're back near your start point; a "closed loop" racing
line doesn't need to close *exactly*, since Phase 4's smoothing already
treats the path as wrapping around.

Where you drive matters: this recorded line is *the* racing line — Phase 4
only paces it, it never reshapes it. Driving close to the actual racing
line you want (hugging the inside of corners where appropriate, wide
smooth arcs rather than jerky manual corrections) directly becomes what
the car repeats, lap after lap.

## Phase 4: Generate the velocity profile

New: the `generate_velocity_profile` command-line tool (not a ROS node —
it's an offline file-processing step, run once per recorded lap, not while
the car is moving). This is the first "very good algorithm" half of this
stack: turning a bare `(x, y)` path into a `(x, y, speed)` racing line by
figuring out how fast the car can safely go at every single point.

### Step 1 — curvature from three points

For every waypoint, look at it and its immediate neighbors (call them
$A$, $B$, $C$). There is exactly one circle passing through all three; a
tight corner produces a small circle (small radius, high curvature), a
gentle bend produces a large circle, and a straight line produces an
(effectively) infinite circle (zero curvature). Using the identity that a
triangle's area relates to its circumradius $R$ by $\text{area} = \frac{abc}{4R}$
(where $a,b,c$ are its side lengths):

$$R = \frac{|AB| \cdot |BC| \cdot |CA|}{4 \cdot \text{area}(A,B,C)} \qquad \kappa = \frac{1}{R} = \frac{4 \cdot \text{area}(A,B,C)}{|AB| \cdot |BC| \cdot |CA|}$$

This needs no calculus and no curve-fitting — just three neighboring
recorded points — which is exactly why it works directly on a raw,
slightly-noisy hand-driven recording. See
`racing_math.estimate_path_curvature()`.

### Step 2 — cornering speed from a simplified friction circle

A car driving a circular arc of curvature $\kappa$ at speed $v$ experiences
lateral acceleration $a_{lat} = v^2 \kappa$ (plain uniform circular motion).
Capping that at the car's actual grip limit $a_{lat,max}$ and solving for
$v$:

$$v_{corner} = \min\left(v_{max}, \sqrt{\dfrac{a_{lat,max}}{\kappa}}\right)$$

Tighter corners (`bigger kappa`) get a lower speed limit automatically.
This is a *simplified* friction circle — real tires trade off lateral vs.
longitudinal grip on a combined ellipse, and real chassis have weight
transfer, suspension behavior, etc. This model ignores all of that and
just uses one number, `a_lat_max`, as a conservative stand-in for "how hard
can this car actually corner." Tune it empirically (below).

### Step 3 — forward/backward smoothing (this is what creates real braking zones)

A raw per-point cornering-speed limit alone would ask the car to
*teleport* from race speed on a straight to walking pace at a corner's
apex, one waypoint before it — physically impossible. Two more passes fix
this, each capping how much the speed is allowed to change between
adjacent waypoints a distance $ds$ apart:

- **Forward (acceleration) pass**, left to right: $v_i \leftarrow \min\left(v_i,\ \sqrt{v_{i-1}^2 + 2\, a_{accel,max}\, ds}\right)$
- **Backward (braking) pass**, right to left: $v_i \leftarrow \min\left(v_i,\ \sqrt{v_{i+1}^2 + 2\, a_{brake,max}\, ds}\right)$

The backward pass is the important one for lap time: it's what propagates
a corner's low speed limit *backward* along the straight leading into it,
so the profile tells the car to start braking early enough to actually
make the corner, instead of "discovering" the corner's speed limit only
once it's already there. A closed-loop track has no single starting point
to seed these sweeps from cleanly (index 0's "previous" waypoint is the
*last* waypoint, whose value isn't finalized on the first sweep) — so both
passes are repeated `smoothing_passes` times (default 5) to let that
start/finish seam converge. Both passes only ever *lower* a speed, never
raise one, so extra passes past convergence are harmless no-ops — this is
why the same code path is used for open paths too, without a special case.

**This is not a time-optimal racing line.** A truly time-optimal line
solves for the path geometry *and* the speed profile together — usually
with a nonlinear/QP optimizer over the minimum-curvature path within track
bounds (e.g. TU Munich's open-source
[global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)).
This tool doesn't reshape the path at all — it only paces whatever line you
drove in Phase 3. That's a deliberate trade-off: no extra heavy
dependencies (no QP solver), a result you can sanity-check by eye, and a
lap time that's still very competitive if you record a good line by hand.
See [Limitations and how to go further](#limitations-and-how-to-go-further).

### Choosing `a_lat_max` / `a_accel_max` / `a_brake_max` / `v_max`

Exactly like every other speed parameter on this car: **start
conservative, raise gradually, re-test wheels-off-ground after every
change.**

1. Start with the simulator-validated defaults (`a_lat_max=2.5`, `a_accel_max=3.0`,
   `a_brake_max=8.0`, `v_max=4.0` — all in SI units, m/s and m/s²). These are
   still starting points for physical testing, not measured tire limits.
2. Race a lap. If the car slides/understeers off the racing line in a
   corner, `a_lat_max` is set higher than the car's actual grip — lower it
   and regenerate the profile.
3. If the car brakes too late and runs wide exiting into a corner,
   `a_brake_max` is set higher than the car can actually achieve — lower
   it and regenerate.
4. Only once cornering is solid, raise `v_max` to actually use more of the
   straights.

Note the distinction between these four and the controller's own limits.
`a_lat_max`/`a_accel_max`/`a_brake_max`/`v_max` shape the *recorded speed
profile*, so changing them means regenerating that profile
([Phase 4](#phase-4-generate-the-velocity-profile)). `pure_pursuit_node`'s
`max_speed`, `max_lateral_accel`, and friends are online ceilings applied
on top of whatever profile is loaded — those you can change on a running
node from the dashboard's
[live tuning panel](web-dashboard.md#live-parameter-tuning), which is the
fast way to answer "is it the profile or the controller?" between runs.
Lowering `max_speed` there clips the whole profile without re-recording
anything.

## Phase 4b (optional): optimize the line itself, not just its speed

Phase 4 answers "how fast can the car drive *this* line". It never asks
whether the line is any good. That is a real ceiling: the recorded lap is
wherever you happened to drive, and the racing line is a property of the
*track*. `optimize_raceline` is the tool that closes the gap — same output
format, same `waypoints_file` parameter, no change whatsoever to
`pure_pursuit_node` or any of its safety layers.

```bash
# The normal path on this car: a saved SLAM map plus a recorded lap.
ros2 run pure_pursuit optimize_raceline \
    --map maps/my_track.yaml \
    --recorded-lap src/pure_pursuit/waypoints/my_track_raw.csv \
    --output src/pure_pursuit/waypoints/my_track_optimized.csv

# Or from a ready-made centerline in the standard TUM/F1TENTH format
# (x_m, y_m, w_tr_right_m, w_tr_left_m), which the Gym tracks ship.
ros2 run pure_pursuit optimize_raceline \
    --centerline Spielberg_centerline.csv --output spielberg.csv
```

### The algorithm: iterative minimum curvature

This is the method from Heilmeier et al., *Minimum curvature trajectory
planning and control for an autonomous race car* (Vehicle System Dynamics,
2019, [DOI 10.1080/00423114.2019.1631455](https://doi.org/10.1080/00423114.2019.1631455)),
the same one behind TUM's
[`global_racetrajectory_optimization`](https://github.com/TUMFTM/global_racetrajectory_optimization).

**Why minimum curvature and not minimum lap time.** Genuine time-optimality
needs a nonlinear optimizer over path *and* speed together — expensive, and
not guaranteed to converge. Minimum curvature is the standard convex
stand-in, and it works because cornering speed is
$v = \sqrt{a_{lat,max} / \kappa}$: minimising curvature raises the speed
ceiling everywhere at once. Heilmeier et al. measure it within a few tenths
of a second per lap of the true optimum. It gives up ground only where the
limit is engine power rather than grip, which is not this car's problem.

**The formulation.** Write every candidate line as a lateral offset
$\alpha(s)$ from the centerline along its normals — one number per waypoint.
Staying on the track is then just a box constraint,
$-w_{right} \le \alpha \le +w_{left}$. For that *parallel offset curve*, the
Frenet relations give the curvature to first order as

$$\kappa_P \approx \kappa + \alpha'' + \alpha\,\kappa^2$$

Three terms with three plain meanings: the curvature already there, the
bending caused by *changing* the offset, and the fact that a fixed offset
toward the inside of a corner tightens it. Minimising $\sum \kappa_P^2$ is
then a linear least-squares in $\alpha$ with box constraints — exactly what
`scipy.optimize.lsq_linear` solves, so there is no `quadprog`/`cvxpy` to
install on the Jetson. Re-linearising about the answer and re-solving a few
times is the "iterative" part, and it is what removes the linearisation
error.

Two details are load-bearing, and both were found the hard way:

- **Each pass must re-parameterise before re-linearising.** Linearising
  about an already-offset copy of the reference is wrong, because that
  curve's parameterisation is stretched by $(1 - \alpha\kappa)$, which
  collapses toward zero wherever the offset approaches the local radius of
  curvature — an apex. The objective then went *up* on every pass after the
  first and the line came back with a 6cm-radius kink in it.
- **The $+\alpha\kappa^2$ term must not be dropped.** Linearising the
  general curvature quotient with its denominator frozen is algebraically
  tempting and gets $-2\alpha\kappa^2$ — inverted, not merely inaccurate. On
  a circular test track, where the answer is obviously the outer wall since
  that is the largest circle that fits, it converged confidently on the
  *inner* wall. A closed-form case with a known answer is what caught it,
  which is why one is kept in the tests.

### Getting a centerline out of a SLAM map

The optimizer needs a centerline with a drivable width at every point; a
recorded lap is neither. But it is an excellent *seed* — guaranteed to be
inside the track, to go round exactly once, and to run in the racing
direction, none of which a skeletonisation of the occupancy grid gives you
for free. So `refine_centerline` measures the walls either side of the seed,
steps it to the middle of what it measured, and repeats.

Two things make that loop unstable on a real map, and both are handled
rather than hoped away. A pit entry, an unmapped doorway or a hole in a
one-cell-thick wall lets a ray escape, which reads as an enormous amount of
room on that side and throws the point out of the track — on Spielberg,
3 escaped rays became 13 in four passes. Blocked cells are therefore dilated
by one cell (closing the diagonal corners a ray can slip through, at the
cost of one cell of measured width, in the conservative direction), points
whose rays still escape get no vote, and the correction is smoothed and
capped. Where a side genuinely has no wall, the reported width falls back to
the map's clearance field rather than the ray cap.

### The safety checks, and the one dial that matters

`--safety-margin` (default 0.15m, on top of half the car's padded width) is
the fast-versus-safe dial. **The optimizer will use every centimetre it is
given** — that is what it is for — so this is the parameter that stops it
apexing on the paint. Raise it, don't lower it.

Then the finished line is checked *independently* of anything the optimizer
believed:

- **Steering feasibility.** $\kappa_{max}$ against $\tan(\delta_{max})/L$ —
  0.821/m, a 1.22m radius, on this car. A line the rack cannot physically
  steer is worthless.
- **Wall clearance**, sampled from the map's distance transform along the
  whole line.

If either fails the tool **refuses to write the file**. `--allow-infeasible`
downgrades that to a warning for inspection and says so loudly.

### What it actually buys, measured

In the F1TENTH Gym harness (`--scenario pure --raceline optimized`), against
the same track's bare centerline and against the reference TUM raceline that
ships with the track:

| Track | Centerline | This optimizer | TUM reference |
|---|---:|---:|---:|
| Brands Hatch | 94.85 s | **92.30 s (−2.7%)** | 90.20 s (−4.9%) |

Brands Hatch is the only one of the three tracks where that comparison is
readable, and the reason is worth understanding because **it applies to the
real car too**. `pure_solo` deliberately exercises the *no-map fallback*
reactive-avoidance trigger, which fires on anything within 0.7m. A
centerline never comes that close to a wall, so it never triggers; any
racing line — mine or TUM's — apexes closer than 0.7m and gets capped to
`avoidance_speed` (1.0 m/s) for 20–30% of the lap on Spielberg and
Silverstone, which swamps the lap time. Brands Hatch is wide enough that
nobody triggers it.

The operational lesson: **a racing line apexes inside the reactive
avoidance trigger distance by design.** On the car that is fine, because
the default `opponent_detection_mode: map` subtracts mapped walls before the
traffic layer sees them — but it means map subtraction has to actually be
working before an optimized line is worth anything. If localization is off
or the map is stale, the car will crawl round its own racing line. Check the
decision log for avoidance engagements on the first laps.

So: roughly **half the available gain** against the mature reference
implementation, at a 0.15m clearance margin the reference was not holding.
That is the honest summary. The gain against a *hand-recorded* lap — which
is what you actually have for your own track — is larger than the
centerline figure above, because a recorded lap is a good deal worse than a
centerline.

## Phase 5: Race it — the Pure Pursuit controller

`pure_pursuit_node` is the only node that runs *during* the race. Every
control tick (default 40Hz, matching the LIDAR's scan rate), it does
exactly two jobs — steer, and set speed — followed by a set of
independent safety checks that can override either one.

### Why a fixed-rate timer, not the pose callback directly

The subscription callbacks (`pose_callback`, `scan_callback`) only ever
*cache* the latest message and its arrival time; the actual driving logic
in `control_loop()` runs on a `create_timer()` at a fixed rate instead.
If localization died outright and the control loop were driven directly by
`pose_callback`, the loop would simply stop being invoked — and the last
command published would stay "live" on `/drive` forever, with nothing left
to notice and stop it. A timer-driven loop keeps checking "is my data
still fresh?" on its own schedule regardless of whether new sensor data is
still arriving, so a dead sensor feed is something the watchdogs below can
actually catch.

### Steering: adaptive lookahead + Pure Pursuit geometry

1. **Find the nearest waypoint.** Compute the distance from the car's
   current `(x, y)` to every waypoint (or, once running, only to a small
   window of waypoints near last tick's answer — see
   *"Why a windowed nearest-point search"* below) and take the minimum.
   This also doubles as the **cross-track error** — how far the car
   currently is from the racing line.

2. **Pick a lookahead distance that scales with speed:**

   $$L_d = \text{clip}(k \cdot v + L_{min},\ L_{min},\ L_{max})$$

   A *fixed* lookahead is a bad compromise — short enough to corner
   tightly at parking-lot speed and the car oscillates/overshoots at race
   speed; long enough to be smooth at race speed and it cuts corners at
   low speed. Scaling lookahead with the current speed fixes both at once.
   Simulator-validated defaults are $L_{min}=0.6m$, $L_{max}=1.5m$,
   $k=0.15$ — at the 4.0 m/s speed cap that is a 1.2m lookahead. The old
   2.0m-at-4m/s setting cut corners and collided in the dynamics model;
   see [simulator.md](simulator.md).

   The $v$ here is the car's **measured** speed from `/odom`
   (`odom_topic`), not the profiled target. The two differ exactly when it
   matters: while braking into a corner, or while recovering from a safety
   stop, the profile still says "4 m/s" long before the car is going that
   fast, and sizing the lookahead off the target would keep aiming far
   ahead while the car is actually crawling. If `/odom` is missing or
   staler than `odom_timeout_sec` (default `0.5s`), it falls back to the
   profiled speed at the nearest waypoint — the pre-existing behavior. The
   decision log names which one it used, so this is visible rather than
   silent. This is a *sizing* input only: it is deliberately **not** a new
   watchdog, and stale odometry never stops the car on its own.

3. **Walk forward from the nearest waypoint** along the recorded path,
   accumulating segment distances, until $L_d$ has been covered — that
   waypoint is the steering target. (Textbook Pure Pursuit intersects the
   path with a circle of radius $L_d$ centered on the car; walking the
   polyline and snapping to the next recorded point is a simpler
   approximation, accurate up to the spacing between recorded waypoints —
   keep that spacing small, per Phase 3's default of 0.15m, and the
   difference is negligible.)

4. **Transform the target into the car's body frame.** The map/world frame
   and the car's body frame (x forward, y left — REP-103) differ by the
   car's current heading $\psi$ (yaw, extracted from the pose's
   quaternion). Rotating a world-frame offset $(dx, dy)$ into body-frame
   coordinates:

   $$x_{body} = \cos\psi \cdot dx + \sin\psi \cdot dy \qquad y_{body} = -\sin\psi \cdot dx + \cos\psi \cdot dy$$

5. **Pure Pursuit's curvature formula.** Picture the one circle that
   passes through the origin (the car's rear axle) *and* through
   $(x_{body}, y_{body})$ (the target), tangent to the car's current
   heading (the body-frame x-axis) — i.e. centered somewhere on the
   body-frame y-axis at $(0, R)$. Solving for where that circle also
   passes through the target point gives:

   $$\kappa = \frac{2\, y_{body}}{x_{body}^2 + y_{body}^2}$$

   Target to the left ($y_{body}>0$) gives positive curvature; target to
   the right gives negative curvature — matching
   `AckermannDriveStamped`'s "positive `steering_angle` = left" convention
   directly, with no sign-flipping needed anywhere.

6. **Bicycle-model steering angle.** Collapsing the car's front/rear wheel
   pairs to a single front and single rear wheel (the standard car-like
   robot approximation), a vehicle with wheelbase $L$ needs a front steer
   angle:

   $$\delta = \arctan(L \cdot \kappa)$$

   Finally clipped to `max_steering_angle` (default `0.26 rad`, ≈15°) —
   see *"Where 0.26 rad comes from"* below — and then rate-limited against
   the previous command by `max_steering_rate` (see *"Online command
   shaping"*).

### Speed: the profile, then two online ceilings

The **base** speed is simply the profiled speed at the car's *current*
nearest waypoint (not the steering target's), clipped to
`[min_speed, max_speed]` as a hard safety ceiling independent of whatever
the `.csv` says. Using the car's current position (rather than the
lookahead target) means the speed command reflects "how fast should I be
going *right here, right now*" — the braking zones baked into the profile
by Phase 4 already account for what's coming up.

That profile, however, only knows about the *recorded* line. It has
nothing to say about a turn the car is taking that isn't on that line —
a correction after a localization jump, a reactive swerve around
something, or the offset arc of an overtake. Those are precisely the
moments the car is asking for its sharpest steering while the profile is
still handing it a straight-line speed. So the final command also passes
an **online curvature ceiling**, from the steering angle actually being
commanded this tick:

$$\kappa_{cmd} = \frac{\tan\delta}{L} \qquad v \le \sqrt{\frac{a_{lat,max}}{|\kappa_{cmd}|}}$$

with `max_lateral_accel` (default `2.5 m/s²`). It is evaluated on the
larger of the requested and the rate-limited curvature, so the slowdown
lands as the turn is *asked for*, not after the rack has caught up. On the
recorded line this ceiling is inactive by construction — Phase 4 already
profiled that curvature, and more permissively (`a_lat_max` is typically
higher). It only ever binds when the car is doing something the offline
profile never saw.

A reactive override (`avoidance_speed`, or a hard stop) is treated as a
**ceiling too**, not merely a target: it can lower the command instantly.

### Online command shaping

The last stage before publishing bounds how fast a command may *change*,
so one noisy tick cannot become a step input at the servo or the motor:

| Limit | Default | Applies to |
|---|---|---|
| `max_steering_rate` | `1.0 rad/s` | steering, both directions |
| `max_acceleration` | `6.0 m/s²` | speed, **rising only** |
| `max_braking_decel` | `8.0 m/s²` | speed, falling, for normal commands |
| `command_slew_max_dt` | `0.10 s` | cap on the `dt` any single slew step integrates |

`command_slew_max_dt` exists so that a stalled control loop followed by a
resumed one cannot cash in a large accumulated interval as one big jump.

**The acceleration ramp starts from the car's measured speed, not from the
last command.** This matters more than it sounds. A ceiling — avoidance,
curvature, a hard stop — drops the *command* instantly, but the car keeps
rolling: it cannot shed 3 m/s in one 25 ms tick. Ramping back up from that
dropped command would hold the throttle far below the car's real speed for
the whole climb, actively braking a car that never actually slowed. So the
ramp basis is `max(last command, measured /odom speed)`, clamped to
`max_speed` so a bad reading cannot inflate it. It only ever *raises* the
basis, every ceiling below still applies, and stale odometry falls back to
the old command-based behavior.

Two properties of this stage are load-bearing and should not be quietly
changed:

- **Emergency stops bypass it entirely.** A zero-speed command from the
  deadman check, pose timeout, cross-track watchdog, missing/stale scan,
  or the LiDAR hard-stop net is published immediately and unshaped. Rate
  limiting a stop into a nonzero command would be a safety regression, so
  the shaping path is only ever reached with a positive desired speed.
- **`max_acceleration` is bounded on both sides, and `6.0` is near the top
  of the usable band.** It caps how fast a *command* may rise; it is not a
  demand on the motor. Too low (`3.0`) and the car cannot re-accelerate out
  of a safety stop behind a slower car — it stalls on track. Too high
  (`7.0+`) and it arrives behind that slower car too fast for the overtake
  to commit, hard-stops, and enters a stop-go cycle it never leaves.
  Re-run the traffic scenario after touching it; see
  [simulator.md](simulator.md#the-adaptive-speed-work-two-values-the-traffic-scenario-pinned-down).

### Why a windowed nearest-point search

On a track that comes close to itself — a hairpin, a figure-eight, a pit
lane splitting off the main straight — the *globally* nearest waypoint by
raw distance is sometimes on a completely different part of the track than
the one the car is actually on. Restricting the nearest-point search to a
small window of waypoints (`nearest_search_window`, default 40) around
*last tick's* answer keeps the tracker locked onto the correct branch
instead of "teleporting" its target across the track. It's also simply
faster — O(window) instead of O(N) every tick — though at typical racing
line sizes (a few hundred to a couple thousand points) that speed
difference doesn't actually matter on the Jetson; correctness at
self-intersections is the real reason this exists.

### The safety layers

Seven independent checks, each capable of unilaterally forcing a stop (or a
steering override), regardless of what the steering/speed logic above
computed. Ordered from "must never be violated" down to "nice to have":

| Check | Triggers when | Why |
|---|---|---|
| **LB deadman button** (checked first, ahead of everything else) | LB not held on a live `/joy` stream within `joy_timeout_sec` (default 0.5s) | **Mandatory workspace policy** — see [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car). Stays on (`enable_deadman: true`) until the team explicitly decides the car's behavior is trustworthy enough to relax it — don't set it `false` otherwise |
| Localization watchdog | No pose received yet, or `pose_topic` has gone quiet for more than `pose_timeout_sec` (default 0.5s) | Never drive on a stale or absent position estimate |
| Cross-track error | Nearest waypoint is farther than `max_cross_track_error` (default 1.0m) | Car is lost, kidnapped, or localization has diverged — the steering geometry would be aiming at a point unrelated to reality |
| Opponent detection + overtake steering | Another car detected and being closed on within `overtake_trigger_gap` (default 3.0m of *track* distance) | Not a safety check at all — a racing one. See [Racing against opponents](#racing-against-opponents-detection-tracking-and-overtaking) below. Always subordinate to the two checks after it |
| Reactive avoidance (steer around) | An unmapped return in the 60° cone is under 1.5m; before a map is ready, raw range is under the 0.7m fallback | Map subtraction prevents ordinary walls from continuously triggering the traffic layer. A committed pass bypasses the generic 1m/s cap, while the emergency tier remains active |
| Emergency hard stop (always wins) | Minimum range in a narrower `safety_fov_deg` cone (default 60°) is under `emergency_stop_distance` (default 0.4m), or `/scan` itself is stale/missing | Last resort, unconditional — a safety net that's gone blind is treated identically to "obstacle detected" |
| Unhandled exception | Anything in the control step raises | `control_loop()` wraps the whole step in try/except; on *any* exception it publishes a stop command *before* re-raising, so an unexpected bug can't leave the last (possibly full-speed) command sitting on `/drive` forever |

**Every stop in this table is published immediately and unshaped.** The
acceleration/steering rate limits described in
[Online command shaping](#online-command-shaping) sit on the *normal*
command path only; a zero-speed command never passes through them, because
rate-limiting a stop into a nonzero command would defeat the whole table.

Because the deadman check runs first, holding LB is a precondition for the
car moving at all — releasing it stops the car immediately regardless of
what every other watchdog says. Concretely, this means **`joy_node` must be
up** while racing — it lives in `bringup_launch.py` (the shared foundation
every control layer needs), not `teleop_launch.py` (the manual-driving
control layer, which you simply don't launch during a race) — see
[operations.md](operations.md#racing-with-the-pure-pursuit-stack).

All of this sits *underneath* the same arbitration the rest of this repo
uses — `pure_pursuit_node` publishes to `/drive` exactly like `gap_follow`
does, and `ackermann_mux` + the joystick still have final say (see
[architecture.md](architecture.md#the-safety-model-read-this-before-writing-autonomy-code)).
None of the above replaces wheels-off-ground testing or a human ready to
cut power — see [operations.md](operations.md#racing-with-the-pure-pursuit-stack).

### Where `0.26 rad` comes from

This car's actual servo calibration
(`src/f1tenth_system/f1tenth_stack/config/vesc.yaml`):

```
servo_position = -1.2135 * steering_angle + 0.5304,   servo clamped to [0.15, 0.85]
```

Solving both ends for `steering_angle`: `servo=0.15` → `+0.313 rad`
(≈18°); `servo=0.85` → `-0.263 rad` (≈-15°). The rack is *asymmetric* —
it can turn further left than right. `max_steering_angle` uses the
smaller magnitude (`0.26`) so that a command in *either* direction is one
the servo can physically achieve, with a small margin. If this car's
`vesc.yaml` gain/offset/servo limits ever change (a different servo,
re-calibration), re-derive this number rather than leaving it stale.

---

## Racing against opponents: detection, tracking, and overtaking

**In plain terms:** a real racer doesn't just drive their own line and
hope -- they notice the car ahead, work out whether they're closing the
gap or falling behind, and if they're closing it, they look for room to
get past instead of following forever. This section is that same
thinking, done with LIDAR and arithmetic instead of eyes and instinct.
Three questions, asked every control tick:

1. **"Is that actually a car?"** -- look at the live scan for something
   shaped and sized like an opponent, sitting out in the open track (not
   a wall).
2. **"Am I catching them?"** -- track how far along the track they are,
   the same way the ego car's own progress is already tracked, and
   compare how fast each is gaining ground.
3. **"Where's the room?"** -- if catching them, find whichever side has
   more space and steer the racing-line target over there until safely
   past, then merge straight back onto the recorded line.

None of this needs a second sensor, a neural network, or any
communication with the other car -- it's built entirely from the same
`/scan` and racing line every other part of this stack already uses.

### 1. "Is that a car?" -- map subtraction, then geometric filtering

The default `map` detector ray-casts what every LiDAR beam should hit in the
static occupancy map. A measured return at least `map_subtraction_margin`
(default 0.4m) shorter than that prediction is dynamic: something exists there
that the map does not explain. This directly removes walls and corners before
clustering, including an opponent backed against a wall. Every
`map_beam_step`-th beam is cast to control CPU cost.

The remaining dynamic returns still pass the width/range checks below; map
subtraction distinguishes static from dynamic, not race car from debris. Until
`/map` arrives, or when `opponent_detection_mode: heuristic` is selected, the
controller falls back to the raw geometric detector described next rather than
racing blind.

**Step 1 -- group the scan into objects.** Walk the scan and split it into
clusters: runs of consecutive readings that are all "something's there"
(clearly less than the sensor's max range) and don't jump by more than a
small threshold from one beam to the next. A big jump between neighbors
means a *different* object, even if both readings are close -- e.g. a car
sitting in front of a wall shows up as one cluster for the car, a jump,
then a separate cluster for the wall behind it (`cluster_scan_ranges`).

**Step 2 -- measure each cluster.** For a cluster spanning `start_idx` to
`end_idx`, convert its first and last point to Cartesian coordinates and
take the straight-line distance between them -- its **chord width**. This
is a far better size estimate than angular width alone, which
exaggerates anything close and shrinks anything far away
(`cluster_geometry`):

$$\text{width} = \sqrt{(x_{end}-x_{start})^2 + (y_{end}-y_{start})^2}$$

**Step 3 -- keep only the ones shaped like a car.** A real opponent, seen
from the side or the back, is roughly car-width: reject anything
narrower (`opponent_min_width`, default 0.15m -- noise, a thin post) or
wider (`opponent_max_width`, default 0.7m -- almost certainly a wall
segment, which produces much longer or far more irregular runs). Also
confirm there's clearly *more open space* immediately on both sides of
the cluster than the cluster's own distance (`opponent_open_side_margin`)
-- a car sitting in the middle of the track has open track on both sides
of it; a bump in a curving wall usually doesn't
(`detect_opponent_cluster`). Among everything that passes every check,
the *closest* one wins -- the one most immediately relevant to a decision
right now.

The geometric fallback is a heuristic, not certainty. Map mode removes the
most common wall false positives but is only as accurate as the map/pose
alignment; see [Limitations](#limitations-and-how-to-go-further).

### 2. "Am I catching them?" -- tracking progress along the track, not raw position

**In plain terms:** instead of asking "where is the other car in x/y
space" (and then having to guess where the track goes from there to
predict anything), this asks "how far around the *track* are they" -- the
exact same question already asked about the ego car every tick.
Comparing two of those numbers directly answers "am I ahead or behind,
and by how much track distance."

Every waypoint on the racing line already has a **cumulative arc
length** -- the track distance from the start to that point
(`compute_cumulative_arc_length`, computed once at startup, a running
total of `seg_len`). Finding the opponent's *own* nearest waypoint (the
exact same `find_nearest_index` the ego car uses on itself) and reading
its arc length gives "how far around the track the opponent currently
is" -- directly comparable to the ego car's own position, on the same
scale.

**Predicting where they'll be** is then just tracking how that number
changes over time. `OpponentTracker` keeps an exponentially-smoothed
estimate of the opponent's **progress rate** (their speed *along the
track*, in m/s) from tick to tick:

$$\text{rate} \leftarrow \alpha \cdot \frac{\Delta(\text{arc length})}{\Delta t} + (1-\alpha)\cdot\text{rate}$$

This is a deliberately simple stand-in for what's sometimes called a
*Frenet-frame* prediction in more formal autonomous-driving research:
reasoning about another vehicle's position and speed **relative to a
reference path**, rather than in raw x/y. Predicting "opponent's arc
length one second from now" is then just `arc_length + rate * 1.0` -- a
prediction that automatically follows the track's own curvature, because
it's expressed in track distance rather than a straight line the
opponent would otherwise have to be assumed to be driving off of.

**"Ahead" wraps around the finish line.** On a closed loop, the opponent
being "0.3 laps ahead" and "0.7 laps behind" describe the same physical
gap looked at from two directions; `track_progress_gap` always reports
the *ahead* distance, wrapping past the start/finish line where needed,
so "how close am I to catching them" is always one consistent, positive
number.

### 3. Deciding, and executing, an overtake

An overtake starts when **both** are true:
- the opponent is within `overtake_trigger_gap` meters of *track
  distance* ahead (not straight-line distance -- a hairpin apex might be
  1m away in a straight line but 8m away along the actual track), and
- the ego car's current profiled speed exceeds the opponent's tracked
  progress rate by at least `overtake_closing_margin` -- i.e. actually
  gaining ground, not just nearby.

Once triggered, **which side to pass on** is decided once, from the same
scan that found the opponent in the first place: average the range
readings in a small window just past each end of the opponent's cluster,
and pick whichever side is more open (`pick_pass_side`). This reuses
exactly the reasoning `gap_follow`'s own avoidance logic already uses --
finding open space in a scan -- just applied to "which side of this one
object" instead of "which gap in this whole scene."

**Executing the pass** doesn't touch the recorded racing line at all -- it
nudges the *steering target* sideways instead. `lateral_offset_point`
takes a waypoint ahead, estimates the track's local direction of travel
from it to the next waypoint, and offsets it perpendicular to that
direction by `overtake_lateral_offset` meters, toward the chosen side.
Steering is then computed from *that* shifted point using the exact same
Pure Pursuit geometry as always -- the overtake is really just "aim
slightly to one side for a while," not a separate control system.

The waypoint it offsets is deliberately **not** the normal Pure Pursuit
target. It is a longer preview, `overtake_lookahead_distance` (default
`4.0m`) of arc length ahead of the car, and this matters more than it
looks. The normal target is at most `max_lookahead` (`1.5m`) away, and
offsetting a point that close by 0.35m sideways demands a large curvature
-- roughly a 0.45 rad heading change, well past the `0.26 rad` steering
clamp. With the online curvature ceiling described above now in the loop,
the controller answers that demand the only way it can: by braking. The
car then slows down in the middle of the pass, which is exactly backwards.
In the simulator's traffic scenario this was not a subtle degradation --
the ego stalled behind the opponent and covered 0.1 laps in 240s. Spread
over a 4m preview, the same 0.35m offset is a gentle arc the car can hold
at speed, and the same scenario completes a clean lap with the pass done.
Because of that coupling, the node **refuses to start** if
`overtake_lookahead_distance` is less than `max_lookahead`.

**Ending the overtake** happens once the ego car's own arc length is
`overtake_clear_margin` meters past the opponent's *last known* position
-- deliberately not re-checked against a fresh detection every tick, since
alongside or just past an opponent it commonly falls completely out of
the forward LIDAR cone, and that must not look like "lost it, panic"
rather than "passed it, done." If the tracked opponent goes stale
(`opponent_lost_timeout_sec`, default 1s, with no update at all) with no
overtake in progress, tracking is simply cleared -- nothing to react to.

**This always sits underneath the existing reactive safety net, never
instead of it.** If an overtake maneuver (or anything else) brings the
car within `emergency_stop_distance` of *anything*, the hard-stop tier
described above still wins, unconditionally, regardless of what the
overtake logic wanted to do. Racing strategy never gets to override
safety -- see [The safety layers](#the-safety-layers) above.

### Why this design, and not something fancier

A full solution to "race well against an opponent" is a genuinely hard,
active research problem -- game-theoretic planning, learned opponent
models, joint trajectory optimization. None of that is what's built here,
deliberately:

- **No opponent communication or shared telemetry.** This works from
  `/scan` alone, the same sensor everything else in this stack already
  depends on -- no assumption the other car is friendly, instrumented, or
  running compatible software.
- **Single-opponent, not a field.** "The closest qualifying cluster wins"
  means this reasons about one opponent at a time. A real multi-car pack
  would need per-object identity tracking (recognizing cluster #3 this
  tick as the same car as cluster #3 last tick, even after a brief
  occlusion) -- a legitimate next step, not attempted here.
- **Map subtraction, not semantic classification.** It reliably excludes
  mapped walls when localization is aligned, but an unmapped car-sized object
  can still qualify. The geometric fallback is less selective.
- **No blocking/defensive maneuvers.** If an opponent is closing in from
  *behind*, this stack does nothing different -- it just keeps driving
  its own optimized line. That's a deliberate, safety-conscious choice:
  defensive blocking in real racing carries real contact risk, and
  "drive your own best line consistently" is itself a legitimate,
  effective strategy without needing to reason about another car's
  intentions at all.

---

## Why Pure Pursuit (and not gap_follow alone, and not full MPC)

Three broad options exist for the control layer once you have a racing
line: stay purely reactive (`gap_follow`'s approach, but that throws away
the racing line entirely), Pure Pursuit (what's implemented here), or a
full Model Predictive Controller that optimizes steering *and* speed
together over a rolling time horizon.

Pure Pursuit was chosen deliberately:

- **Robust and simple to reason about.** The entire control law is two
  closed-form formulas (curvature, then steering angle) — no solver, no
  iteration, no convergence to worry about, no risk of a control loop
  silently taking too long and missing a deadline on a resource-limited
  Jetson Orin Nano.
- **Provably bounded per-tick cost.** A nearest-point search plus a short
  forward walk plus two `atan`s — comfortably real-time at 40Hz.
- **Well-understood failure modes.** "Lookahead too short → oscillation,
  too long → corner-cutting" is a one-line tuning heuristic, not a cost
  function to re-derive.
- **A genuinely strong track record** — this is the same core algorithm
  used across a large fraction of real competitive F1TENTH/roboracer
  teams' race stacks, precisely because it's fast enough to actually trust
  under race-day time pressure.

A full MPC can, in principle, out-perform this by planning several moves
ahead and reasoning explicitly about the car's dynamic limits — but it
needs an accurate dynamics model, a QP/NLP solver running fast enough for
40Hz control on limited hardware, and a lot more that can silently go
wrong under time pressure. That's a legitimate next step (see below), not
a reason to ship something harder to trust for this iteration.

## Parameter reference

All of these live in `src/pure_pursuit/config/pure_pursuit.yaml` (see that
file for inline comments too):

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
| `overtake_lateral_offset` | `0.35` | Meters; sideways nudge to the steering target while passing |
| `overtake_lookahead_distance` | `4.0` | Meters of arc ahead the offset above is applied to, instead of the normal target. Must be >= `max_lookahead` — the node refuses to start otherwise |
| `opponent_detection_mode` | `map` | Map subtraction by default; `heuristic` is the no-map fallback |
| `map_topic` / `map_beam_step` / `map_subtraction_margin` | `/map` / `4` / `0.4` | Occupancy map, ray-cast downsampling, and residual margin |
| `laser_offset_x` / `laser_offset_y` | `0.33` / `0.0` | Estimated LIDAR mounting offset from `base_link`, used to place detections in the map frame |
| `enable_deadman` | `true` | **Mandatory workspace policy** — LB deadman button, checked first. Leave `true`; see [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car) |
| `joy_topic` | `/joy` | Deadman button input |
| `deadman_button` | `4` | Button index (LB on the F710 in XInput mode) |
| `joy_timeout_sec` | `0.5` | Deadman button staleness watchdog |

`generate_velocity_profile`'s physical-limit flags (`--v-max`,
`--a-lat-max`, `--a-accel-max`, `--a-brake-max`, `--smoothing-passes`) are
documented via `--help` and in Phase 4 above.

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

## How this wins races

On a track you get to map and drive in advance — true of nearly every
real race — the single biggest lap-time lever isn't reaction speed, it's
*carrying more speed through corners you already know are coming* and
*starting to brake at exactly the right moment, every single lap,
identically*. A purely reactive controller re-derives "what should I do
right now" from scratch every cycle with no memory of the track, which
means it can't plan a smooth line through a corner it can't yet see, and
it can't consistently reproduce a good line lap after lap. This stack's
whole design is aimed at removing that ceiling: know the track, know
exactly where you are on it, and drive the fastest line your tires can
actually hold — while keeping a reactive safety net running underneath
for the one thing a map genuinely can't know about: whatever wasn't there
when you built it.

## Limitations and how to go further

Being direct about what this *doesn't* do, as a map for where to take it
next:

- **The racing line is only as good as the lap you recorded** -- *unless*
  you run [Phase 4b](#phase-4b-optional-optimize-the-line-itself-not-just-its-speed).
  Phase 4 alone paces your line and never reshapes it; `optimize_raceline`
  re-derives the geometrically fastest line within the track's actual width.
  What is still missing there is the last step beyond minimum curvature: a
  genuine minimum-*time* optimizer over path and speed jointly, and a
  velocity profile that knows about the specific corner it is entering
  rather than only its curvature.
- **The velocity profile is a simplified friction-circle model**, not a
  full vehicle dynamics simulation — no combined lateral/longitudinal tire
  ellipse, no weight transfer, no slip-angle model.
- **Pure Pursuit doesn't reason about the future beyond one lookahead
  point.** A full MPC could plan the next N steps jointly against an
  actual dynamics model — a legitimate, harder next project once this
  baseline is solid and trusted.
- **Localization is dead-reckoning-fused-with-LIDAR only** — no IMU/wheel
  encoder sensor fusion beyond what `particle_filter`/`vesc_to_odom`
  already do. Better odometry directly means a tighter, more trustworthy
  `max_cross_track_error`.
- **Opponent detection is map subtraction plus geometry, not semantic
  recognition.** Mapped walls are excluded, but an unmapped car-sized object
  can still qualify, and map/pose misalignment can create residuals. The
  fallback has only cluster geometry. It also reasons about one opponent at a time
  (closest qualifying cluster wins) and does zero identity-tracking
  across brief occlusions — see ["Racing against
  opponents"](#racing-against-opponents-detection-tracking-and-overtaking)
  for the full reasoning and what a sturdier version would need (camera +
  learned detection, multi-object tracking, or both).
- **No defensive/blocking driving.** If an opponent is closing from
  behind, this stack doesn't react any differently — a deliberate,
  safety-conscious choice, not an oversight; see the same section above.

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
