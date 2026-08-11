# Validating your code in the simulator

> **Who this is for:** anyone who has written or changed driving code and wants to know whether it works — without putting the real car on the floor.
> **Read first:** [ros-simulator.md](ros-simulator.md) — what `racerbot_sim` is and why it refuses to run beside the real car. [concepts.md](concepts.md) if you've never used ROS2.
> **You'll be able to:** run your code against simulated physics, and watch what the car sees and decides on the web dashboard from your laptop or phone.
> **Time:** about 20 minutes the first time, then 5 minutes per test.

The simulator replaces three pieces of hardware with software: the [LiDAR](glossary.md#lidar) (the spinning laser that measures distances), the motor controller, and the gamepad.

Everything above that is the code that runs on the real car, unchanged — your [node](glossary.md#node) (one running program in the system), [SLAM](glossary.md#slam) (map-building), the racing controller, and the dashboard.

So you can break things, drive into walls, and try a bad idea at 3 m/s without anyone having to catch the car.

## Highlights

- **No car, no floor space, no spotter.** The whole driving stack runs on the Jetson with nothing plugged in.
- **The same launch files you run on the car.** `sim_auto_map_race_launch.py` *includes* the real `auto_map_race_launch.py` rather than copying it, so what you test is what you run.
- **The dashboard works unchanged.** No config edits, no sim-specific flag — the dashboard subscribes to topics, and the simulator publishes the same topic names the hardware does.
- **It refuses to run beside the real car.** Two nodes here forge a held deadman and a fake LiDAR; both go silent if the real drivers appear on the ROS graph. See [Safety](#safety-two-things-this-simulator-forges).
- **A full mapping-plus-racing run takes under four minutes.** Measured on this Jetson: SLAM up at 7 s, mapping lap done at 46 s, racing from 90 s.
- **One command gives a pass/fail verdict.** `run_auto_map_validation.py` exits non-zero if a scenario fails, so it can gate a change.
- **It runs in real time, so results vary slightly between runs.** The ROS graph is real and scheduling varies. A result that hinges on a thousandth of a radian will not reproduce exactly.
- **It does not model tyre grip, VESC dynamics, or WiFi.** A pass here is not a promise about the floor — see [sim-fidelity-audit.md](sim-fidelity-audit.md).

### Why it exists

The other simulator ([simulator.md](simulator.md)) calls the controller math directly, with no ROS at all. That is the right shape for tuning a control law, and blind to everything else.

The way the automatic race stack actually broke was in the wiring *around* the math:

- SLAM, and [TF](glossary.md#tf--transform--frame) — the bookkeeping that relates one coordinate frame to another.
- A [racing line](glossary.md#racing-line) — the path around the track the car intends to drive — recorded from a live pose.
- A parameter handover between two nodes.
- A safety layer with no way out of its own stop.

None of that exists in a harness with no ROS in it.

This is the half that catches those. It is also the only way to see your node's decisions on the dashboard without a car.

## Before you start

- [ ] The workspace builds and you can source it (`colcon build --symlink-install`, see [concepts.md](concepts.md)).
- [ ] The F1TENTH Gym physics engine is installed. **One time only:** `tools/f1tenth_sim/setup.sh`.
- [ ] **No real drivers are running.** If `bringup_launch.py` is up anywhere on your network, the simulator will refuse to [publish](glossary.md#publish--subscribe) (send messages out). Check with `ros2 node list`.
- [ ] You know the Jetson's address, for viewing the dashboard from your laptop: run `hostname -I` on the car.

<details>
<summary><b>How the simulator finds the physics engine</b> — click to expand. Skip it unless <code>setup.sh</code> failed or you're moving the workspace.</summary>

`tools/f1tenth_sim/setup.sh` clones a pinned revision of the F1TENTH Gym into `.sim/f1tenth_gym` at the workspace root, and installs its Python dependencies into `.sim/python`. Both are gitignored — they are build artifacts, not source.

You do **not** need to export `PYTHONPATH`. `racerbot_sim/sim_bridge.py` walks up from its own location looking for a `.sim/f1tenth_gym` directory and adds it to `sys.path` itself, and sets `NUMBA_CACHE_DIR` while it's there.

That means the simulator works from any terminal and any working directory, once the workspace is sourced.

This is the opposite of how the standalone `tools/f1tenth_sim/run_validation.py` harness works. That one does need the `PYTHONPATH` that `setup.sh` exports.

If a launch fails with `ModuleNotFoundError: No module named 'f1tenth_gym'`, `.sim/` is missing or incomplete. Re-run `tools/f1tenth_sim/setup.sh`.

</details>

## Safety: two things this simulator forges

**Never run `racerbot_sim` while the real car's drivers are up.**

Two of its nodes are dangerous next to real hardware:

- **`sim_joy_node` publishes a synthetic `/joy` with LB reported as held.** The entire workspace safety policy is that nothing moves unless a human is holding LB on the physical gamepad (the [deadman](glossary.md#deadman) button). This node is that human's hand, forged.
- **`gym_bridge_node` publishes an invented `/scan` and `/odom`.** Alongside the real `urg_node` and `vesc_to_odom_node` that is two publishers per topic, and the driving nodes would be steering on a blend of a real room and an imaginary one.

If both ran at once, a driving node could command a real motor based on an imaginary room, with the safety button already reported as pressed.

**You do not have to remember this.** Both nodes check the live ROS graph rather than trusting a flag, because a flag is something you can forget. If any of `vesc_driver_node`, `ackermann_to_vesc_node`, `vesc_to_odom_node`, `urg_node` or `joy` is present, they hold their output and log:

```
REAL HARDWARE DETECTED (urg_node). ... is now suppressed. This package
simulates the car and must never run beside the car -- shut down
bringup_launch.py, or move the simulator to another machine.
```

The check repeats, so bringing the car up *after* the simulator silences the simulator too. **Do not defeat this check.**

> **This is the one place the LB rule is legitimately bypassed, and only because nothing physical is attached.** The rule itself has not changed: see [the workspace policy](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car). A node you validated in the simulator still needs a real LB check before it goes near the car.

## Which workflow do you want?

| You want to | Go to |
|---|---|
| Check the simulator works at all, and see the dashboard fill in | [Steps 1–3](#step-1--start-the-simulated-car) below |
| Test a driving node **you wrote** | [Validating your own node](#validating-your-own-driving-node) |
| Test a tuning or config change to `gap_follow` / `pure_pursuit` | [Steps 1–3](#step-1--start-the-simulated-car), then [live tuning](#tuning-while-it-drives) |
| Test the whole mapping → racing composition | [The full auto-map-race stack](#the-full-auto-map-race-stack) |
| Get a pass/fail verdict, or run over SSH with no browser | [Automated and headless validation](#automated-and-headless-validation) |

---

## Step 1 — start the simulated car

This is the simulator's version of `bringup_launch.py`: the hardware layer and the referee that picks which command wins, and **nothing that drives on its own**.

**Terminal 1** — the simulated hardware. Leave it running.

```bash
source /opt/ros/jazzy/setup.bash
source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_sim sim_bringup_launch.py
```

**Working when:** after a few seconds of engine startup you see a line like

```
gym_bridge_node ready: track='indoor_oval' (596 centerline points),
1 car(s), 1081 beams, 40Hz. Listening on '/ackermann_cmd'.
```

You will also see a `WARN` from `sim_joy_node` announcing the synthetic deadman. That warning is correct and expected — it is the node telling you it is forging the safety button.

**If it doesn't:** `ModuleNotFoundError: No module named 'f1tenth_gym'` means the physics engine isn't installed — run `tools/f1tenth_sim/setup.sh`. A `REAL HARDWARE DETECTED` error means the car's drivers are running; shut them down.

Nothing moves yet. That is correct — there is no control layer, exactly as on the real car.

<details>
<summary><b>Choosing a different track, or adding other cars</b> — click to expand. The defaults are fine for a first run.</summary>

| Argument | Default | What it does |
|---|---|---|
| `track` | `indoor_oval` | `indoor_oval` (30 m lap, 1.8 m corridor), `indoor_tight` (27 m, 1.4 m), `indoor_wide` (42 m, 2.6 m) |
| `opponents` | *(none)* | `;`-separated `offset_m,speed_mps,lateral_m`. Speed `0` parks that car on the line as a static obstacle |
| `hold_deadman` | `true` | `false` proves your node stays stopped with LB released |
| `release_after_sec` | `-1` | Release the synthetic LB this far into the run |
| `odom_speed_scale` | `1.0` | [Odometry](glossary.md#odometry--odom) (the car's own estimate of how far it has travelled) scale error, as a miscalibrated `speed_to_erpm_gain` would give |
| `seed` | `12345` | LiDAR and odometry noise |

**Terminal 1**, in place of the plain command above:

```bash
ros2 launch racerbot_sim sim_bringup_launch.py \
    track:=indoor_tight opponents:="6,0.7,0.0;14,0.6,0.0"
```

**Working when:** the same `gym_bridge_node ready` line appears, reporting the track you asked for and `3 car(s)` instead of one.

The tracks are deliberately room-sized rather than the official 300 m F1TENTH circuits, so a full mapping-and-racing run is four minutes instead of twenty. Their corner radii are all outside this car's 1.22 m minimum turning circle.

**`hold_deadman:=false` is a test worth running.** It is the simulator's version of letting go of LB: your node should command zero and stay there.

</details>

## Step 2 — put a driving node on top

Exactly one control layer, in its own terminal, exactly as on the car. `gap_follow` needs no map, so it's the quickest thing to put on top.

**Terminal 2** — the control layer.

```bash
source /opt/ros/jazzy/setup.bash
source ~/racerbot-ws/install/setup.bash
ros2 launch gap_follow gap_follow_launch.py
```

**Working when:** the node logs a `DRIVE` decision about once a second, and Terminal 1's numbers start changing:

```
DRIVE [gap_follow] selected preferred depth 2.00m gap -26.8deg to +26.7deg;
target=4.91m at -0.0deg, ... command: steering=-0.001rad, speed=2.50m/s
```

**If it doesn't:** a single `STOP [waiting_for_joy]` at startup is normal — it clears as soon as the synthetic `/joy` arrives. If it *keeps* saying `waiting_for_joy`, `sim_joy_node` is suppressed; check Terminal 1 for `REAL HARDWARE DETECTED`.

## Step 3 — connect the dashboard

**There is nothing to configure.** The dashboard subscribes to topics and publishes to none, and the simulator publishes the same topic names the real hardware does — `/scan`, `/odom`, `/ackermann_cmd`, `/joy`, `/drive_intent`. It cannot tell the difference, and neither of them needs to know about the other.

**Terminal 3** — the dashboard. Leave it running.

```bash
source /opt/ros/jazzy/setup.bash
source ~/racerbot-ws/install/setup.bash
ros2 launch web_dashboard web_dashboard_launch.py
```

**Working when:** you see the server announce itself:

```
web_dashboard_node ready: map=/map scan=/scan pose=/pf/viz/inferred_pose|/slam_pose
drive=/ackermann_cmd odom=/odom joy=/joy (LB index 4).
live tuning enabled for pure_pursuit_node, gap_follow_node.
Serving on port 8080, every interface, IPv4 + IPv6
```

**Then open `http://<jetson-ip>:8080/`** in a browser on your laptop or phone. Get `<jetson-ip>` with `hostname -I` on the car, or use its Tailscale name — see [the dashboard's networking section](web-dashboard.md#finding-the-cars-address-and-viewing-through-a-forwarded-port).

**If it doesn't:** `Address already in use` means a dashboard is already running — you only ever need one, and it can stay up across simulator restarts. If the page loads but every panel is empty, Terminal 1 isn't publishing.

> **Shortcut for the full race stack:** `sim_auto_map_race_launch.py` takes `dashboard:=true` and starts the dashboard for you, so you don't need Terminal 3 at all. See [the full stack](#the-full-auto-map-race-stack).

### What you're looking at

With Steps 1–3 running, the dashboard is showing a **simulated** car with no map. So it opens in **robot-centric mode**: the car sits fixed at the centre of the screen facing up, with LiDAR points drawn around it.

| On the page | What it means for a simulated car |
|---|---|
| **LiDAR points** | What the car *sees* — ray-cast against the generated track walls. 1081 beams over 270°, matching the real Hokuyo. Red at 10 cm through to green at 2 m. |
| **The car arrow** | Where the car is. Cyan; it is the system talking, never an opinion. |
| **Speed and steering readouts** | The command that won [arbitration](glossary.md#mux--multiplexer) — the referee picking between competing commands — on `/ackermann_cmd`. The same topic the VESC would have obeyed. |
| **The intent arrow and decision panel** | What your node is *trying* to do and why, from `/drive_intent`. This is the "why did it do that" panel. |
| **The stopwatch** | Runs off the synthetic LB hold, so it starts immediately rather than when a human presses a button. |
| **The camera panel** | Reads `camera offline`. There is no simulated camera — this panel only fills in with a real RealSense or webcam. |
| **The map** | Empty until something publishes `/map`. `gap_follow` doesn't; SLAM does. |

That last row is worth stating plainly, because it looks like a bug and isn't:

**With `sim_bringup` + `gap_follow` alone, `/map`, `/slam_pose` and `/pf/viz/inferred_pose` have zero publishers.** Verified with `ros2 topic info /map`. The dashboard is subscribed and waiting; nothing is producing them. Run SLAM or the [full stack](#the-full-auto-map-race-stack) and the map appears.

### Tuning while it drives

The dashboard's live tuning panel works against the simulator exactly as it does against the car — it calls the standard `set_parameters` service on `pure_pursuit_node` and `gap_follow_node`. The startup log above confirms which nodes it found.

This is the fastest way to test a tuning change: move a slider, watch the sim car's behaviour change on the same screen, and only then save it into the config file. Full detail in [web-dashboard.md](web-dashboard.md#live-parameter-tuning).

---

## Validating your own driving node

Your node goes in Terminal 2, in place of `gap_follow`. Nothing else changes.

```bash
ros2 launch racerbot_sim sim_bringup_launch.py     # terminal 1
ros2 launch <your_package> <your_node>_launch.py   # terminal 2
ros2 launch web_dashboard web_dashboard_launch.py  # terminal 3
```

That works because your node talks to the same topics the real car uses. If it subscribes to `/scan` and publishes to `/drive`, it is already simulator-compatible — see [writing-your-own-node.md](writing-your-own-node.md).

**What to check, in order:**

1. **It publishes at all.** `ros2 topic hz /drive` in a fourth terminal. Nothing there means the node isn't running or is stopped for a reason it should be logging.
2. **It respects the deadman.** Restart Terminal 1 with `hold_deadman:=false`. Your node must command zero speed and stay there. **If it drives anyway, the node is unsafe and must not go on the car.**
3. **It survives a lap.** Watch the dashboard. Wall contact shows up as the car stopping with LiDAR points crowding one side.
4. **It recovers from its own safety stop.** The most valuable thing this simulator catches — see the worked example below.
5. **It says why.** If your node publishes `/drive_intent` ([drive-intent.md](drive-intent.md)), the decision panel explains each choice. Without it, you are reading logs.

<details>
<summary><b>Worked example: a latched safety stop, seen on the dashboard</b> — click to expand. This is the failure class this simulator exists to catch.</summary>

During a run recorded while writing this doc — on the default `indoor_oval` — the racing phase covered 58 m and then stopped dead for the rest of the run. The dashboard showed the car nose-first into the inner wall, stationary, with the LiDAR points bunched on one side.

The decision panel said `emergency_clearance`, and the node's log said:

```
STOP [body_contact] minimum clearance from the car body is -0.046m,
at or below the 0.050m contact threshold ... command: speed=0.00m/s
```

The clearance is *negative* — the car body is overlapping a wall. The safety layer correctly refuses to drive, and the car has no way out of its own stop, so it sits there forever.

**In this case the code was fine and the track was too narrow** — see [Why that `track:=indoor_wide`](#why-that-trackindoor_wide). The same scenario on `indoor_wide` raced 113.6 m in 66.9 s with zero wall contact.

That is the point of the example. Three things make the shape worth recognising:

- **Total distance travelled looked fine** (58 m), because it is dominated by the mapping laps. Only distance covered *after the handover* exposes it. That is exactly why `run_auto_map_validation.py` gates on `--min-raced-distance` separately.
- **On the dashboard it was obvious in one glance**, and in the logs it was fifty identical warning lines a minute. This is the case for having the browser open.
- **A latched stop looks the same whether the cause is your code or the course.** Check the track before you go debugging: reproduce on `indoor_wide` first, and only suspect the node if it still stops there.

</details>

---

## The full auto-map-race stack

The whole automatic composition, driven by the simulator, with the dashboard included.

In order: SLAM maps the track, a racing line is recorded and cleaned, the map is saved, control hands over to [pure pursuit](glossary.md#pure-pursuit) (the map-based racing controller), and it races.

**Terminal 1** — everything, including the dashboard. This is the only terminal you need.

```bash
source /opt/ros/jazzy/setup.bash
source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_sim sim_auto_map_race_launch.py track:=indoor_wide dashboard:=true
```

**Working when:** open `http://<jetson-ip>:8080/` and watch the map draw itself as the car drives its mapping laps. In the terminal, these four milestones appear in order over about 90 seconds:

```
Recorded lap cleaned up: 267 points over 40.0m ... peak curvature 0.744/m of
the 0.821/m the rack can reach (needs 13.6deg steering, limit 14.9deg) ...
Saved occupancy map successfully.
Saved pose graph successfully.
Transition complete: pure pursuit now has drive control.
```

**If it doesn't:** if the run never reaches "Transition complete", the mapping lap failed — usually a smeared map or a corner tighter than the car's 1.22 m turning circle. See [racing-autonomy.md](racing-autonomy.md#what-a-recorded-lap-actually-looks-like).

### Why that `track:=indoor_wide`

**The launch default is `indoor_oval`, and on that track the racing phase is expected to touch a wall.** That is a true statement about the course, not a bug in the car's code. It is why the command above overrides the default.

The reason is width. The racing line is recorded from `gap_follow`, which drives 0.25–0.35 m from a corner's wall.

Pure pursuit then follows that line with a cross-track error of 0.39–0.57 m through a corner near this car's turning circle, at 2.5–3.0 m/s. Those two do not both fit inside `indoor_oval`'s 1.8 m corridor.

`indoor_wide` has a 2.6 m corridor, which is why all three automated scenarios use it (see [Automated validation](#a-passfail-verdict)).

So pick the track by the question you're asking:

| Track | Corridor | The question it asks |
|---|---|---|
| `indoor_wide` | 2.6 m | Can the car map, plan, hand over, and **race** cleanly? Use this to watch it work. |
| `indoor_oval` | 1.8 m | The launch default. Maps and plans fine; expect wall contact once racing. |
| `indoor_tight` | 1.4 m | Does the pipeline still map, clean up, and **refuse** correctly when the course is too tight? |

If you launch on the default and watch the car wedge itself nose-first into a corner, nothing has gone wrong — you asked the narrow question.

This [launch file](glossary.md#launch-file) — the recipe that starts a set of nodes together — deliberately *includes* `racerbot_launch`'s real `auto_map_race_launch.py` with `include_bringup:=false` rather than reproducing it. A copy would drift, and then this would be validating a launch file nobody runs.

**This is the fastest way to check a change to any part of the automatic path**, and the only thing that exercises the launch *wiring* rather than the control math.

### Watching the phases on the dashboard

This is where the dashboard earns its place, because each phase looks different:

| Phase | What the dashboard shows |
|---|---|
| **Mapping** | The map builds up live in the background as the car drives. The [scan](glossary.md#scan) — one sweep of LiDAR readings — stays robot-centric. |
| **Line cleanup and save** | The car stops. The map stops changing. |
| **Handover** | The commanded speed and steering readouts start coming from pure pursuit instead. |
| **Racing** | Map-relative mode: the map is the background and the car is drawn at its localized position, so you can see it relative to the walls. |

"The SLAM map looked really glitchy" is how a mapping run usually gets judged, and it's hard to act on without the picture. [web-dashboard.md](web-dashboard.md#the-map-looks-glitchy) breaks that down into the three things it can actually be.

### Memory on the Jetson

The full stack plus the dashboard is the heaviest thing you can run here. Measured during a run for this doc: **6.1 GB of the Jetson's 7.5 GB in use, about 1.4 GB free.**

It fits, but it is not roomy. Don't run a `colcon build` at the same time, and if you're also building, add `--parallel-workers 1`.

---

## Automated and headless validation

### A pass/fail verdict

```bash
tools/racerbot_sim/run_auto_map_validation.py --scenario all
```

**Working when:** it runs three scenarios end to end and exits `0`. It exits non-zero if any fails, so it can gate a change.

| Scenario | What's on the track | What it proves |
|---|---|---|
| `solo` | nothing | mapping, cleanup, save, handover, and a clean racing stint |
| `obstacle` | one parked car on the racing line | the racing controller gets *past* a static obstacle rather than latching stopped in front of it |
| `traffic` | two slower cars | mapping with moving traffic, and racing among it without contact |

**All three run on `indoor_wide`, overriding the launch default**, for the width reasons above. Pass `--track indoor_oval` or `--track indoor_tight` to deliberately ask the narrower question instead.

Each scenario is judged on every phase being reached, no node crashing, no wall or car contact, and — separately — **distance covered after the handover**. Total distance is dominated by the mapping laps and stays high even when the racing phase never moves.

Useful flags: `--scenario solo` for one scenario, `--keep-logs` to keep the launch log for a passing run, `--output <file>` for the JSON report.

A passing `solo` run on this Jetson, for calibration:

| Phase | Reached at |
|---|---|
| SLAM up | 7.0 s |
| Mapping lap 1 | 46.1 s |
| Line cleaned, profile written | 85.1 s |
| Map saved, profile loaded | 88.1 s |
| Racing | 90.1 s |

It then watched 66.9 s of racing, over which the car covered **113.6 m with zero wall contact and zero car contact**. The checked-in reference result is [auto-map-sim-results.json](auto-map-sim-results.json).

> **Expect some run-to-run variation.** The simulator runs in real time against a real ROS graph, so scheduling varies and runs are not bit-identical. A scenario that fails once is worth re-running before you go hunting; a scenario that fails repeatedly is a real result.

### A picture, with no browser

If you're on SSH with no way to open a browser, `capture_dashboard.py` connects to the same WebSocket a browser does, decodes the same messages, and draws them into a PNG.

```bash
tools/racerbot_sim/capture_dashboard.py --output /tmp/dashboard.png
```

**Working when:** it writes the PNG and prints a JSON line per frame, including the decision your node was making:

```json
{"frame": 0, "map": "237x174 @ 0.050m/px", "scan_beams": 1081,
 "pose": [3.444, -2.577, 0.669], "drive": [0.0, -0.26],
 "intent_state": "emergency_clearance"}
```

**If it doesn't:** `no map was received -- is slam_toolbox publishing /map?` and exit code 1 means there is no map to draw. **This tool requires `/map`**, so it works with the full stack but not with `sim_bringup` + `gap_follow` alone. For a map-less run, use the browser.

With `--seconds` and `--interval` it becomes the dashboard's test instrument rather than a screenshot tool.

It watches a whole run, writes a numbered frame per interval, checks every binary frame against the length its header declared, and exits non-zero if any failed.

```bash
tools/racerbot_sim/capture_dashboard.py --seconds 240 --interval 60 \
    --output /tmp/run.png --report /tmp/run.json
```

It only listens and never sends a control message, so it is safe against the real car too.

---

## Ground truth: what the dashboard can't tell you

The dashboard shows what the car **believes**. That is the point — it is the same view you get from the real car, and hiding the car's own errors would defeat it.

`/odom` in the simulator is **dead-reckoned, not ground truth**. It is integrated from wheel speed and the commanded servo angle exactly as `vesc_to_odom` does on the car, so it drifts in the same shape. Publishing perfect odometry there would hide every mapping problem this simulator exists to find.

What is actually true is published separately, and **nothing that drives the car may read it**:

| Topic | What's on it |
|---|---|
| `/sim/ground_truth_pose` | The car's true pose, in its own unconnected `sim_world` frame |
| `/sim/opponent_poses` | True poses of the other cars |
| `/sim/status` | JSON: track, sim time, true position and speed, distance travelled, and wall/car contact counters |

`/sim/status` is the quickest way to check a run objectively:

```bash
ros2 topic echo /sim/status --once
```

```json
{"track": "indoor_oval", "sim_time_s": 102.9, "distance_travelled_m": 57.9,
 "wall_contact_now": false, "wall_contact_steps": 1, "ever_contacted": true, ...}
```

Comparing that against what the dashboard shows is how you tell "the car is lost" from "the car is where it thinks it is, and the plan is wrong".

<details>
<summary><b>Why collision comes from here and not from the physics engine</b> — click to expand. Skip unless you're changing the validation criteria.</summary>

In the pinned Gym revision, gym's own wall-collision check **never fires** with this workspace's vehicle parameters. `side_distances` computes to all zeros, and the test reduces to "did a beam return less than 5 mm", which it cannot, because `range_min` is 0.05 m.

Measured directly: a car driven straight into a wall at 1 m/s reports `collision=False` and keeps going, right off the edge of the map.

So `racerbot_sim` samples the padded car body against the occupancy grid itself and treats off-map as contact. That is what `/sim/status` reports and what the validation criteria use. **Do not reinstate a "no collision" claim that rests on gym's own flag.**

</details>

## What it does and does not model

**Real:** F1TENTH Gym single-track dynamics with RK4 at 5 ms, multi-car ray-cast LiDAR, 1081 beams over 270° matching the Hokuyo UST-10LX, the 0.33 m LiDAR offset, the padded 0.58 × 0.31 m body, 40 Hz control, and the real ROS graph above the drivers.

**Not modelled:** tyre grip against a real floor, VESC and servo dynamics, LiDAR reflectivity and multi-echo, WiFi, and CPU contention on the Jetson.

**A pass here is not permission to skip the on-car test ladder.** Static topic check → wheels off the ground with LB held → floor, low speed, open space. See [writing-your-own-node.md](writing-your-own-node.md#testing-before-its-on-wheels). [sim-fidelity-audit.md](sim-fidelity-audit.md) covers how far the shared vehicle model is from this car.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ModuleNotFoundError: No module named 'f1tenth_gym'` | The physics engine isn't installed | Run `tools/f1tenth_sim/setup.sh` once |
| `REAL HARDWARE DETECTED (...)` and nothing publishes | The car's drivers are on the ROS graph | Shut down `bringup_launch.py`, or run the simulator on another machine. Don't defeat the check |
| Your node logs `waiting_for_joy` forever | `sim_joy_node` is suppressed, or Terminal 1 died | Check Terminal 1 for the hardware-detected error |
| Dashboard page loads but everything is empty | Nothing is publishing | Confirm Terminal 1 is up: `ros2 topic hz /scan` |
| Dashboard shows scan but never a map | Nothing publishes `/map` | Expected without SLAM. Use the [full stack](#the-full-auto-map-race-stack) |
| Camera panel says `camera offline` | There is no simulated camera | Expected. Not a fault |
| `Address already in use` on port 8080 | A dashboard is already running | Use the existing one; one is enough |
| Nodes keep running after you kill the launch | `pkill` on `ros2 launch` orphans its child nodes | Prefer `Ctrl+C` in the terminal. To clean up: `pkill -f gym_bridge_node` and friends, then confirm with `pgrep` |
| The car races, then stops against a wall and never restarts | Usually the track, not your code — `indoor_oval` is too narrow to race | Re-run with `track:=indoor_wide`. If it still stops, read the `STOP [...]` reason. See [the worked example](#validating-your-own-driving-node) |
| Results differ between identical runs | Real-time execution on a real ROS graph | Expected. Re-run before investigating |

General problems that aren't specific to the simulator are in [troubleshooting.md](troubleshooting.md).

## See also

- [ros-simulator.md](ros-simulator.md) — what `racerbot_sim` is, the interlock, and the measured validation results.
- [simulator.md](simulator.md) — the other simulator, which tests controller math with no ROS at all.
- [web-dashboard.md](web-dashboard.md) — everything the dashboard shows, and live parameter tuning.
- [sim-fidelity-audit.md](sim-fidelity-audit.md) — how far the simulated car is from this physical car. Read before trusting a result.
- [writing-your-own-node.md](writing-your-own-node.md) — the contract your driving node has to meet, and the on-car test ladder.
- [drive-intent.md](drive-intent.md) — publishing what your algorithm is trying to do, so the decision panel can show it.
