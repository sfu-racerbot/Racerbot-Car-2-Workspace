# Recording and diagnosing a run

> **Who this is for:** anyone trying to work out why a run went wrong, after the fact.
> **Read first:** [operations.md](operations.md) — record the run first, diagnose it second.
> **You'll be able to:** find the run bringup recorded for you (or record one by hand), then analyze pipeline health, pose lag and watchdog stops.

How to capture everything needed to work out what a run did — and how to
hand that to a person or an AI agent so they can act on it.

This exists because of a debugging session on **2026-07-27** in which four
separate faults stacked on top of each other, each one hiding the next.
Every recommendation below is here because its absence cost real time or a
real collision that night. The postmortem is at the end.

---

## Automatic recording

**Since 2026-10-26, every run of an algorithm is recorded for you.** Start
bringup the usual way, start any algorithm on top, and a run directory
appears under `~/racerbot-ws/data/racerbot_runs/` while it runs. You do not need the
recorder terminal from the [TL;DR](#tldr) below any more, unless you want
one for a run that is not an algorithm (manual driving, say).

**Highlights**

- **On by default, in every bringup** (`bringup_launch.py` and its two
  alternates). Off with one flag: `record:=false`.
- **Records only while an algorithm runs.** Bringup alone, the dashboard,
  and manual driving with the gamepad record nothing.
- **Any algorithm, anyone's.** It watches for any [node](glossary.md#node)
  publishing `/drive` (or `/auto_map/drive`, `/auto_race/drive`,
  `neural_racer`'s shadow output, or the motor command topics). Our
  `gap_follow`, `pure_pursuit`, `auto_map_race` and `neural_racer` all count,
  and so do team A's `ftg`, `disparity_extender` and `reactive`.
- **Never two recorders at once.** If `record_run.py`, `map_course`,
  `auto_map_race_launch.py` or any `ros2 bag record` is already recording,
  it stays out of the way. If one starts mid-run, it stops its own recording
  and keeps what it has.
- **Same layout as a hand-started recording**, so `analyze_run.py` and
  `summarize_run` work on it unchanged. It adds which algorithm ran, team
  A's git commit when it was their code, and every ROS process's command line.
- **Cannot move the car.** It creates no publisher and no subscription: it
  only reads the list of nodes and topics, the way `ros2 node list` does.

**Why it exists:** on 2026-10-26 the races against team A were run with
plain `gap_follow_launch.py`, and nothing recorded them. All that was left
was the node's one-line-a-second log, which cannot show what the car saw or
how smoothly it steered.

### Using it

**Terminal 1** — bringup, exactly as before:

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** among bringup's startup lines you see
`auto_record ready: records a run while any node publishes /drive, ...`.

**Terminal 2** — any algorithm, for example:

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch gap_follow gap_follow_launch.py
```

**Working when:** about 3 seconds later, Terminal 1 prints
`recording /gap_follow_node on /drive -> /home/.../racerbot-ws/data/racerbot_runs/<time>`.

When you stop the algorithm with `Ctrl+C`, about 3 seconds later Terminal 1
prints `saved run ... (<seconds> s, bag <size> MB)` and the command to
analyse it:

```bash
python3 ~/racerbot-ws/tools/run_analysis/analyze_run.py ~/racerbot-ws/data/racerbot_runs/<time>
```

**If it says** `another recorder is on the graph (...): not recording it`:
that is correct. Something else (a `record_run.py` you started, or the one
`map_course` brings) is already recording this run.

**To turn it off** for a session: `ros2 launch f1tenth_stack bringup_launch.py record:=false`.

**Disk use:** about 0.8 MB per second of driving (3 GB an hour), almost all
of it [LiDAR](glossary.md#lidar) scans. Nothing deletes old runs. It refuses to start a recording
with less than 2 GB free, and says so in Terminal 1.

### What a run directory holds, beyond the usual

Everything in [the table below](#what-gets-captured-and-why-each-piece-earns-its-place),
plus three files only automatic runs have:

| File | What it says |
|---|---|
| `auto_record.yaml` | Which node started the recording and on which topic, when it started and stopped, why it stopped (`no algorithm publishing for 3.0 s`, `another recorder started: ...`, `bringup shut down`), and whether the bag closed cleanly (`recorder_exit_code: 0`) |
| `metadata/processes.txt` | The command line of every ROS process running when it started: which [launch file](glossary.md#launch-file), which executable, which params files. This is how you tell team A's code from ours later |
| `recorder.log` | `record_run.py`'s own output, for when a recording went wrong |

`metadata/git/` also holds `racerbot_a.txt` and `racerbot_a.diff`: team A's
commit and uncommitted edits. That holds whether the run was theirs or ours.

<details>
<summary><b>How it decides, and every setting</b> (click to expand). Skip unless you're changing it.</summary>

Bringup starts `auto_record` (`race_diagnostics/auto_record_node.py`). Twice
a second it lists every node on the graph, and the nodes that
[publish](glossary.md#publish--subscribe) on each trigger topic. It hands
that list to a small state machine, `race_diagnostics/auto_record.py`. That
file has no ROS in it and is tested in `test/test_auto_record.py`.

1. A publisher on a trigger topic is an **algorithm**, unless it is on the
   ignore list. `ackermann_mux` and `ackermann_to_vesc_node` are there
   because they publish the motor command topics all the time, as part of
   bringup. `rosbag2_player` is there because it is a bag being played back.
2. An algorithm that has been up for `start_delay_sec` (3 s) starts a
   recording, unless another recorder is on the graph. The delay gives a
   launch that brings its own recorder (`map_course`) time to get it up:
   measured on the car, `map_course`'s recorder appears about 1 s after its
   algorithm.
3. **Another recorder** is any node named `race_diag_node` or
   `rosbag2_recorder*`. The recording `auto_record` starts is
   `record_run.py` run with `name_prefix:=auto_`, so its own nodes are
   `auto_race_diag_node` and `auto_rosbag2_recorder` and are never mistaken
   for someone else's.
4. With no algorithm for `stop_delay_sec` (3 s), it stops the recording:
   `SIGINT` to `record_run.py`, which closes the bag the same way `Ctrl+C`
   does.
5. If a recording dies on its own, it does not start another until the
   algorithm has gone and come back, so a broken recorder cannot restart in
   a loop.

Stopping is designed not to lose the bag:

- On bringup's `Ctrl+C`, `auto_record` closes the recording first (up to
  `stop_timeout_sec`, 15 s) and only then exits.
- If `auto_record` is killed outright, the recording still gets `SIGINT` from
  the kernel (`setpriv --pdeathsig`) and closes its bag.
- The recording runs in its own process session, so killing it as a last
  resort cannot touch bringup.

All settings are in `src/race_diagnostics/config/auto_record.yaml`, each
commented there. They are `trigger_topics`, `ignore_publishers`,
`other_recorders`, `start_delay_sec`, `stop_delay_sec`, `poll_period_sec`,
`record_bag`, `topics` (empty means the same topic list as `record_run.py`),
`min_free_gb` and `stop_timeout_sec`.

Recordings go to `~/racerbot-ws/data/racerbot_runs`. Pass
`record_output_directory:=<dir>` to bringup to send them elsewhere.

**Tests.** Three files cover it:

- `test_auto_record.py`: the decisions, no ROS.
- `test_bringup_auto_record.py`: every bringup includes the recorder, on by
  default and off with `record:=false`, and a missing `race_diagnostics`
  cannot stop bringup.
- `test_auto_record_node.py`: the real node on an isolated ROS domain. It
  records, closes the bag, defers to another recorder and survives being
  killed. It takes about a minute.

The last one needs ROS sourced. **Any terminal:**

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest src/race_diagnostics/test/ -v
```

**Working when:** every test passes and none is skipped. Unsourced, the
four node tests and two of the bringup tests skip, and say why.

</details>

## TL;DR

For a run that [automatic recording](#automatic-recording) does not cover, or to record by hand: three terminals. Start them in this order.

```bash
# --- Terminal 1: the recorder. Start FIRST, so nothing is missed. ---
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch race_diagnostics record_run.py
# It prints the run directory it created, and the exact tee command to use.

# --- Terminal 2: the driving stack. Note the `| tee` (optional, see below). ---
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_launch auto_map_race_launch.py \
  2>&1 | tee ~/racerbot-ws/data/racerbot_runs/<the-dir-it-printed>/launch.log

# --- Terminal 3 (optional): live high-signal view of terminal 2 ---
ros2 run race_diagnostics filter_log ~/racerbot-ws/data/racerbot_runs/<dir>/launch.log
```

> **`auto_map_race_launch.py` already starts this for you.** It runs the probe by default (`diagnostics:=true`), so a one-command map-and-race run leaves a machine-readable record without a second terminal. The rosbag stays off there unless you pass `record_bag:=true`. Everything below still applies to any other workflow.

Afterwards:

```bash
ros2 run race_diagnostics summarize_run ~/racerbot-ws/data/racerbot_runs/<dir>
```

**`| tee` is no longer strictly required, but is still the simplest option.**
`summarize_run` can reconstruct an equivalent merged log on its own: ROS 2
launch already writes every node's complete stdout to a loose
`~/.ros/log/python3_<PID>_<epoch_ms>.log` file per node process, whether or
not anyone tees anything. When `launch.log` is missing, `summarize_run`
locates the per-node files that fall inside the run's own time window and
merges them back into one chronological stream automatically — see
`race_diagnostics/node_logs.py`.

This is a **different** file from ROS's own aggregated
`~/.ros/log/<run>/launch.log`, which is still close to useless and this
reconstruction does not rely on: during the 2026-07-27 session that
aggregated file held 48 lines of a run that printed several thousand.
`| tee` remains the most direct way to get one complete, already-ordered
log without depending on after-the-fact file-hunting, so keep using it
when you remember to — the fallback exists for when you didn't.

*(Housekeeping, not an action item: `~/.ros/log/` currently holds around
19,000 of these loose per-process log files on this machine, going back
months, and nothing prunes them.)*

---

## What gets captured, and why each piece earns its place

One timestamped directory per run under `~/racerbot-ws/data/racerbot_runs/`. That folder is inside the git repo on purpose, so runs can be committed and pushed. After a session, run `tools/save_runs.sh` to do both ([data/README.md](../data/README.md)).

The bag is split into 50 MB files (`bag/bag_0.mcap`, `bag_1.mcap`, ...) because GitHub refuses any file over 100 MB. You never open those files one at a time: `ros2 bag play <run>/bag` and the analysis tools read the whole `bag/` folder as one recording.

| Artifact | Produced by | Answers |
|---|---|---|
| `launch.log` | your `tee` | What every node decided, tick by tick |
| `events.jsonl` | `race_diag_node` | Machine-readable pipeline state, **pose lag**, watchdog fires |
| `probe.log` | `race_diag_node` stdout | Same, human-readable |
| `bag/` | `ros2 bag record` | Everything, replayable offline. Includes both odometries (`/odom`, `/odom/imu`), both IMUs, SLAM's `/pose` and pose graph, and every node's log on `/rosout` |
| `metadata/git/` | `record_run.py` at start | Which code ran: HEAD, branch, `git status` and the full uncommitted diff of `racerbot-ws` and `~/imu-calibration`. Untracked files are listed, not copied |
| `metadata/files/` | `record_run.py` at start | Copies of imu-calibration's `car.yaml` and calibration JSONs: the IMU mounts and calibration, which no ROS parameter holds |
| `metadata/params/` | `snapshot_params`, 15 s in | Every node's parameters, one YAML per node, in `ros2 param dump` format (usable as `--params-file`). Later changes are in the bag's `/parameter_events` |
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
ros2 bag play ~/racerbot-ws/data/racerbot_runs/<dir>/bag --clock
```

`/scan` is ~90% of the size. Drop it for a 10× smaller bag if you only
need pose/TF/command timing:

```bash
ros2 launch race_diagnostics record_run.py \
  topics:="/odom /slam_pose /drive /ackermann_cmd /tf /tf_static /map /joy /parameter_events"
```

**Parameter changes are in the bag too.** `/parameter_events` is recorded by default. Every node announces each change to one of its settings on that topic, so the bag holds every change made during the run: from the [web dashboard's tuning panel](../src/web_dashboards/car/docs/web-dashboard.md#live-parameter-tuning), from Lichtblick through [foxglove_bridge](foxglove-bridge.md), or from `ros2 param set` in a terminal. Without it, a bag can show the car driving differently halfway through a run with no record of why.

To read them back:

```bash
ros2 bag play ~/racerbot-ws/data/racerbot_runs/<dir>/bag --clock &
ros2 topic echo /parameter_events
```

Keep `/parameter_events` in the list if you pass your own `topics:=`, as above.

---

## Reading the results

```bash
ros2 run race_diagnostics summarize_run ~/racerbot-ws/data/racerbot_runs/<dir>
ros2 run race_diagnostics summarize_run ~/racerbot-ws/data/racerbot_runs/<dir> --json   # for an agent
```

It reports, in the order that matters:

1. **How far the run got** — SLAM active → lap closed → profile generated
   → profile loaded → pure pursuit driving. The first `[ NOT ]` is where
   to look.
2. **Localization health** — worst pose lag, frozen-pose samples, TF losses.
3. **Watchdog stops by reason** — which safety net fired, how often. Every
   hard-stop state either driving node can log is named here now, not just
   the six that were originally listed — see `run_events.py`'s
   `CRITICAL_PATTERNS`.
4. **Unclassified stops** — how many `STOP [...]` lines matched no known
   watchdog state. 0 is the expected value; a nonzero count means
   `CRITICAL_PATTERNS` has drifted out of sync with what the driving nodes
   actually log (exactly what happened to the lap-progress regex below,
   before 2026-08-25) and is itself a finding worth fixing. (This
   deliberately excludes `auto_map_race_node`'s own routine bookkeeping
   stops — `loading_profile`, `transition_hold`, and the rest of
   `supervisor_stop` — which fire on every ordinary run and would
   otherwise make this count permanently, uninformatively nonzero.)
5. **Which lap-closure gate is holding a lap open** — see below.
6. **Errors and node deaths**, de-duplicated.
7. **Missing artifacts** — itself a finding. No `map.pgm` means the
   occupancy save failed, which happened when slam_toolbox's executor was
   too busy to serve its own map subscription. `launch.log` itself is no
   longer fatal if missing — see the `| tee` note above.

Escape/recovery maneuvers (`emergency_escape`, and — once landed —
`ttc_escape`/`body_contact_escape`/`off_racing_line_recovery`) get their
own `escape_maneuver` count too: these publish a nonzero crawl speed and
log with the `DRIVE` prefix rather than `STOP`, so before 2026-08-25 they
were invisible to this whole classification scheme, not merely
miscategorized.

### The lap-closure gates

The supervisor declares a lap only when *all* of these pass at once, and
the log line reports every one as `value/limit`:

| Gate | Config key | Failure looks like |
|---|---|---|
| departed the start | `departure_distance` | never leaves the start area |
| distance travelled | `minimum_lap_distance` | loop shorter than the minimum can never close |
| turning done | `minimum_lap_turn_deg` | repeated reactive stops keep resetting progress before a full 300° accumulates — the actual blocker on every run in the 2026-08-25 session, and silently unreported by `summarize_run` before that day's regex fix |
| elapsed time | `minimum_lap_duration_sec` | rarely the blocker |
| back near the start | `closure_distance` | passes the start but too wide |
| heading matches start | `closure_heading_deg` | comes back the "wrong way round" |

> **With `close_at_closest_approach: true`, the last two gates are checked at one point.** `auto_map_race.yaml` sets it.
>
> A lap does not close on the first tick inside the closure circle. That circle's radius is `closure_distance` until the gate widens after 1.25 laps (`closure_widen_after_revolutions`).
>
> It closes at the nearest point the car came to its start, once the car is 0.05 m farther away (`closest_approach_margin_m`) or has left the circle.
>
> The heading check is made at that nearest point. If the heading is off there, that point is dropped and the lap goes on. [operations.md](operations.md) shows the log line for a lap that closes this way.

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
  `auto_record` goes further: it does not even subscribe, it only reads
  the graph. That is why it may run inside bringup, which must start
  nothing that can move the car.
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
Run directory: ~/racerbot-ws/data/racerbot_runs/<dir>
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
