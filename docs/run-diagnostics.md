# Recording and diagnosing a run

How to capture everything needed to work out what a run did — and how to
hand that to a person or an AI agent so they can act on it.

This exists because of a debugging session on **2026-07-27** in which four
separate faults stacked on top of each other, each one hiding the next.
Every recommendation below is here because its absence cost real time or a
real collision that night. The postmortem is at the end.

---

## TL;DR

Three terminals. Start them in this order.

```bash
# --- Terminal 1: the recorder. Start FIRST, so nothing is missed. ---
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch race_diagnostics record_run.py
# It prints the run directory it created, and the exact tee command to use.

# --- Terminal 2: the driving stack. Note the `| tee`. ---
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_launch auto_map_race_launch.py \
  2>&1 | tee ~/.ros/racerbot_runs/<the-dir-it-printed>/launch.log

# --- Terminal 3 (optional): live high-signal view of terminal 2 ---
ros2 run race_diagnostics filter_log ~/.ros/racerbot_runs/<dir>/launch.log
```

Afterwards:

```bash
ros2 run race_diagnostics summarize_run ~/.ros/racerbot_runs/<dir>
```

**The `| tee` is not optional.** ROS's own `~/.ros/log/<run>/launch.log`
captures only a fraction of what appears on screen — during the 2026-07-27
session it held 48 lines of a run that printed several thousand. Without
`tee`, the node output that explains the run is simply gone.

---

## What gets captured, and why each piece earns its place

One timestamped directory per run under `~/.ros/racerbot_runs/`:

| Artifact | Produced by | Answers |
|---|---|---|
| `launch.log` | your `tee` | What every node decided, tick by tick |
| `events.jsonl` | `race_diag_node` | Machine-readable pipeline state, **pose lag**, watchdog fires |
| `probe.log` | `race_diag_node` stdout | Same, human-readable |
| `bag/` | `ros2 bag record` | Everything, replayable offline |
| `map.pgm` / `map.yaml` | `slam_toolbox` save | The map as SLAM saw it |
| `raceline_*.csv` | `auto_map_race_node` | The line that was generated and driven. `raceline_raw.csv` is the unmodified recording and is written even when the cleanup *refuses* the run, so a refusal can be plotted against the map |

### Why the probe exists at all

Each node's own log says what *it* is doing. None of them can say why the
node upstream is silent. When SLAM was dead on 2026-07-27, every
downstream node logged a perfectly reasonable "waiting…" — which reads
identically to "healthy but idle". The probe watches the whole chain at
once:

```
/scan → slam_toolbox → /map + map→base_link TF → /slam_pose
      → auto_map_race_node → /drive
```

### Why pose lag is the single most valuable number

Localization staleness is **invisible in a topic's message rate**.
`auto_map_race_node` republishes SLAM's transform at a fixed 40 Hz
regardless of its age, so a frozen transform arrives exactly as punctually
as a live one. Only the header stamp reveals it.

This is not theoretical: it is what drove the car into a wall. Pure
pursuit spent a full second steering from a position the car had already
left, while its staleness watchdog — which measured *message arrival* —
saw nothing wrong.

The same number explains the dashboard's scan-not-lining-up-with-the-map
complaint. Overlay error ≈ **speed × pose lag**. Measured that night:
0.01 s parked, 0.31 s under load, **3.38 s peak** — 3 m of error at
1 m/s.

Rules of thumb:

| Worst pose lag | Meaning |
|---|---|
| < 0.15 s | healthy |
| 0.15–0.5 s | marginal; visible dashboard misalignment |
| > 0.5 s | `pure_pursuit` will hard-stop through the stall (`pose_timeout_sec`) |

### Why a rosbag

Because "what did the car actually see at the moment it hit the wall" is
not answerable from log text. The bag makes a run replayable:

```bash
ros2 bag play ~/.ros/racerbot_runs/<dir>/bag --clock
```

`/scan` is ~90% of the size. Drop it for a 10× smaller bag if you only
need pose/TF/command timing:

```bash
ros2 launch race_diagnostics record_run.py \
  topics:="/odom /slam_pose /drive /ackermann_cmd /tf /tf_static /map /joy"
```

---

## Reading the results

```bash
ros2 run race_diagnostics summarize_run ~/.ros/racerbot_runs/<dir>
ros2 run race_diagnostics summarize_run ~/.ros/racerbot_runs/<dir> --json   # for an agent
```

It reports, in the order that matters:

1. **How far the run got** — SLAM active → lap closed → profile generated
   → profile loaded → pure pursuit driving. The first `[ NOT ]` is where
   to look.
2. **Localization health** — worst pose lag, frozen-pose samples, TF losses.
3. **Watchdog stops by reason** — which safety net fired, how often.
4. **Which lap-closure gate is holding a lap open** — see below.
5. **Errors and node deaths**, de-duplicated.
6. **Missing artifacts** — itself a finding. No `map.pgm` means the
   occupancy save failed, which happened when slam_toolbox's executor was
   too busy to serve its own map subscription.

### The lap-closure gates

The supervisor declares a lap only when *all* of these pass at once, and
the log line reports every one as `value/limit`:

| Gate | Config key | Failure looks like |
|---|---|---|
| departed the start | `departure_distance` | never leaves the start area |
| distance travelled | `minimum_lap_distance` | loop shorter than the minimum can never close |
| elapsed time | `minimum_lap_duration_sec` | rarely the blocker |
| back near the start | `closure_distance` | passes the start but too wide |
| heading matches start | `closure_heading_deg` | comes back the "wrong way round" |

`summarize_run` names the single blocking gate. On 2026-07-27 the car
drove **114 m without closing a lap** because heading error was 30.2°
against a 30.0° limit while every other gate had passed long before —
invisible unless you read that one field.

---

## Safety notes for anyone (or anything) changing this code

Read [architecture.md](architecture.md) and
[writing-your-own-node.md](writing-your-own-node.md) first. The
non-negotiables:

- **Every node that can move the car requires the LB deadman held.** Never
  set `enable_deadman: false` — that is a policy change, not a tuning knob.
- **`race_diagnostics` only subscribes.** It publishes nothing and is
  therefore not driving code. Keep it that way; if it ever needs to
  publish, it becomes driving code and inherits the whole safety policy.
- **Test order for driving changes, never skipped:** static topic check →
  wheels off the ground → floor, low speed, open space.
- Integration tests construct real nodes with the deadman disabled. They
  remap `drive_topic` to `/test_only/drive` for that reason. **Never
  remove that remap** — without it, `pytest` sends live commands to the
  VESC if the driver stack happens to be up.

---

## Prompt structure for an AI agent

For a fresh Claude Code session in this workspace. Fill the brackets. The
ordering is deliberate: constraints before task, evidence before
hypothesis, and an explicit statement of what is *not* yet known.

```markdown
## Context
ROS2 Jazzy F1TENTH car (Jetson Orin Nano, 8GB). This is a PHYSICAL robot
that can hurt itself or people. Read CLAUDE.md and docs/architecture.md
before proposing any change to code that publishes to /drive.

## What I ran
[e.g. ros2 launch racerbot_launch auto_map_race_launch.py, 2 mapping laps
then automatic handover to pure pursuit]

## What I expected
[e.g. car maps 2 laps, generates a raceline, then races it faster than the
1.0 m/s mapping cap]

## What actually happened
[e.g. handover completed, car accelerated, then hit a wall on the left]

## Recorded evidence
Run directory: ~/.ros/racerbot_runs/<dir>
- launch.log       full terminal output of every node
- events.jsonl     probe stream: pipeline state + pose lag
- bag/             replayable with `ros2 bag play <dir>/bag --clock`
- summarize_run output pasted below:

<paste `ros2 run race_diagnostics summarize_run <dir>` here>

## Your task
1. Read the artifacts above BEFORE forming a hypothesis. Quote the
   specific log lines or events.jsonl records that support your
   conclusion — do not infer from code alone.
2. Distinguish clearly between what the evidence PROVES and what it merely
   suggests. Say which is which.
3. Identify root cause. If several faults stacked, order them by which one
   masked the others.
4. Propose fixes. For each: what breaks if it's wrong, and how it would be
   verified.
5. Do NOT change safety watchdog behaviour, the deadman gate, or speed
   limits without flagging it explicitly and waiting for my confirmation.
6. Add a regression test for every behavioural fix. Tests that need no
   rclpy go in the pure-logic modules (racing_math.py, run_events.py,
   protocol.py); node tests must remap drive_topic to /test_only/drive.
7. Run the full suite and report the real result:
   `python3 -m pytest src/<pkg>/test/ -q`

## Constraints
- --symlink-install: edits to .py/launch/config take effect on next launch
  with no rebuild. Rebuild only after touching package.xml/setup.py/C++.
- Full rebuild on this Jetson needs --parallel-workers 1 (8GB RAM).
- Do not commit unless I ask.
```

### What makes this work

- **Evidence before hypothesis.** Asking for quoted log lines is what
  separates "the pose was frozen — here are two identical poses 1.0 s
  apart while odometry read 1.25 m/s" from a plausible-sounding guess.
- **Proved vs. suggested.** On 2026-07-27 the pose freeze was *proved* by
  the logs; the blocking map-save being its *cause* was only inferred. Both
  were worth acting on, but conflating them would have been wrong.
- **Ordering stacked faults.** Four faults were present at once. Reporting
  them flat would have been useless — what mattered was that the SLAM
  lifecycle bug made the other three unreachable.
- **Explicit safety veto.** An agent asked to "fix the car hitting things"
  will otherwise cheerfully relax a threshold that exists for a reason.

### Also worth telling the agent

- Which fix attempts already failed, and why. Nothing wastes an agent's
  time faster than re-deriving a dead end.
- Whether the car is physically undamaged. A bent steering linkage
  presents exactly like a control bug and no log will ever show it.
- That a monitor's *silence* is ambiguous: it can mean healthy, or that
  the monitor's filter never matched. Ask it to confirm the pipeline is
  alive rather than infer it from an absence of complaints.

---

## Postmortem: 2026-07-27

Four faults, each hiding the next. In the order they had to be peeled:

**1. `slam_toolbox` never started.** As of 2.x it is a lifecycle node that
does not configure itself; `slam_launch.py` started it as a plain `Node`,
so it sat in `unconfigured` forever — no `/map`, no `map` frame, no
`/slam_toolbox/save_map`. The tell was a *missing* log line: the
constructor's "Node using stack size" appeared, `on_configure`'s "Using
solver plugin" never did. Everything downstream waited forever and
reported it as normal idling.

**2. `AttributeError` at the handover.** `AsyncParameterClient` exposes
`services_are_ready()`, not `service_is_ready()`. The call sat in the code
path that only runs after the final mapping lap closes — unreachable while
fault 1 kept any lap from ever closing. It killed the supervisor at the
exact moment mapping succeeded.

**3. The collision.** Three things at once: the map/pose-graph save
blocked slam_toolbox's executor and froze the pose; pure pursuit's
staleness watchdog measured message *arrival*, so a faithfully
republished stale transform looked perfectly fresh; and its safety net was
a 60° forward-cone minimum, structurally blind to the wall the car was
already touching *beside* it. It logged `LIDAR clear` while in contact.

**4. A fix that made things worse.** The footprint check added for fault 3
was applied across the Hokuyo's full 270° sweep. The rearmost beams see
the car's own chassis, which is inside the footprint by construction, so
clearance read a permanent −0.110 m and the car could not move at all.
Negative clearance is physically impossible for a real obstacle — that is
what gave it away. `gap_follow` had always windowed to 180° before the
same computation; the maths was copied, the windowing was not.

### What the tooling in this document would have changed

- Faults 1 and 4 were both diagnosed from a **number that could not be
  real** (a missing log line; a negative distance). The probe surfaces
  both classes directly.
- Fault 3 needed pose lag, which nothing was measuring at the time. It had
  to be reconstructed by hand from two log lines that happened to be one
  second apart.
- No rosbag existed, so "what did the LiDAR see at the moment of impact"
  was permanently unanswerable.
- The 114 m of laps that never closed was one unread field in a line that
  scrolled past once a second.
