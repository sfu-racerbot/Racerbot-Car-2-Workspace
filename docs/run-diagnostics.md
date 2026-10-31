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

**Working when:** within about 2 seconds, Terminal 1 prints
`recording /gap_follow_node on /drive -> /home/.../racerbot-ws/data/racerbot_runs/<time>`.

**Then wait for the second line before you press LB:**
`recording live: /drive /joy /odom /scan are in the bag, <n> s after the algorithm appeared`.
From that line on, what the car does is in the recording. Expect it about 3 to 7 seconds
after the algorithm appears (measured 2026-10-31).

The recorder never holds the car back: LB works before that line appears. The
beginning of the run is just not recorded. If that happens the run says so
(see `activity` below), and `summarize_run` prints `STARTED LATE`.

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
with less than 2 GB free, and says so in Terminal 1 and in `auto_record.log`.

**To see what it decided,** read `data/racerbot_runs/auto_record.log`. It keeps one
line for every start, stop, refusal and late start, across sessions. This is the
place to look when a run you expected is not there. The same lines are in
`~/.ros/log/python3_<pid>_*.log`, which is hard to find and is not saved with the runs.

### What a run directory holds, beyond the usual

Everything in [the table below](#what-gets-captured-and-why-each-piece-earns-its-place),
plus three files only automatic runs have:

| File | What it says |
|---|---|
| `auto_record.yaml` | Which node started the recording and on which topic, when it started and stopped, and why it stopped (`no algorithm publishing for 3.0 s`, `another recorder started: ...`, `bringup shut down`). See the fields below for how to tell whether it went well |
| `metadata/processes.txt` | The command line of every ROS process running when it started: which [launch file](glossary.md#launch-file), which executable, which params files. This is how you tell team A's code from ours later |
| `recorder.log` | `record_run.py`'s own output, for when a recording went wrong |

`metadata/git/` also holds `racerbot_a.txt` and `racerbot_a.diff`: team A's
commit and uncommitted edits. That holds whether the run was theirs or ours.

**How to tell whether a recording went well.** These are the `auto_record.yaml`
fields to check. `summarize_run` prints a `Recording problems` section if any of them is bad.

| Field | Good value | What a bad value means |
|---|---|---|
| `recorder_failures` | `[]` | These recording processes died. Check this one first: `recorder_exit_code` stays `0` even when a process under it dies |
| `bag_exit_code` | `0` | The bag did not close normally |
| `recorder_killed` | `false` | It did not stop within 15 s and was killed. The bag may need `ros2 bag reindex` |
| `bag_live_sec` | Under about 7 | Seconds from the algorithm appearing to the bag listening, as seen on the graph. Missing means it never got that far |
| `activity.started_late` | `false` | The car was already moving at the bag's first `/odom`, so **the start of the run is not in the bag** |
| `activity.moved` | `true` | The car never moved. `null` means the bag has no `/odom`, so it is unknown |
| `superseded` | absent | Another recorder replaced this one within 10 s, so this run is only a stub |

`activity` also has `first_odom_speed_mps`, `max_speed_mps` and `lb_held_fraction`.
It appears a few seconds after the run stops, because `run_activity` reads the closed bag at low priority.

`tools/save_runs.sh` lists the runs where the car never moved and the stubs before
committing. `tools/save_runs.sh --skip-idle` leaves those uncommitted. Nothing deletes them.

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
2. An algorithm that has been up for `start_delay_sec` (1.5 s) starts a
   recording, unless another recorder is on the graph. The delay gives a
   launch that brings its own recorder (`map_course`) time to get it up:
   measured on the car, `map_course`'s recorder appears about 1 s after its
   algorithm. If one still arrives later, the recording stops and is labelled
   `superseded: true`.
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

**Why the bag is started separately.** Starting a recording starts `ros2 bag record`
first and on its own, and `record_run.py` with `record_bag:=false` only once the bag
is live (or after 5 s). Measured on the Jetson on 2026-10-31:

| How the bag is started | Start of recording to topics subscribed |
|---|---|
| Through `record_run.py`, as it was | 5.4 to 5.9 s |
| Beside `record_run.py`, both at once | Slower than alone: the two compete for the CPU |
| Alone | 1.4 to 2.6 s in that timing run; up to about 4 s in the test suite |

On 2026-10-30, with the 3 s delay and the first row, the bag began about 9 to 10 s after
the launch command, and two runs (`20261030-183134`, `-184149`) were already moving at
2.3 and 3.5 m/s at their first `/odom`.

**How much of a run's start the bag misses now.** This is from the algorithm's first
message to the bag's first message, measured on the Jetson on 2026-10-31 with two small test topics:

| Recorder | Missed | Median | Runs |
|---|---|---|---|
| Before this change | 5.45 to 7.0 s | about 6.6 s | 4 |
| Now | 2.95 to 6.9 s | about 4 s | 8 |

The typical run improved by about 2.5 s, **but the worst case barely moved**. The cost is the
1.5 s delay, about 1 s of `ros2 bag` startup, and 0.3 to 2.7 s for ROS discovery to
find the topics. So **wait for the `recording live` line if the start matters**.

Only an always-running recorder could remove that discovery wait. It was left out on purpose: it
would keep a process subscribed to every topic all the time, including during manual driving.

Stopping is designed not to lose the bag:

- On bringup's `Ctrl+C`, `auto_record` closes the recording first (up to
  `stop_timeout_sec`, 15 s) and only then exits.
- The bag gets its stop signal at once. `record_run.py` gets its signal once
  `race_diag_node` has written `events.jsonl` (or after 8 s), because a `ros2 launch`
  signalled while still starting its processes can hang until it is killed.
- `ros2 launch` can also hang at exit with all its children gone. `record_run.py`
  therefore exits on its own 18 s after shutdown begins. A normal stop that hits this
  still reports `recorder_killed: true` after 15 s, though the bag is fine (`bag_exit_code: 0`).
- If `auto_record` is killed outright, the recording still gets `SIGINT` from
  the kernel (`setpriv --pdeathsig`) and closes its bag.
- The recording runs in its own process session, so killing it as a last
  resort cannot touch bringup.

All settings are in `src/race_diagnostics/config/auto_record.yaml`, each
commented there. They are `trigger_topics`, `ignore_publishers`,
`other_recorders`, `start_delay_sec`, `stop_delay_sec`, `poll_period_sec`,
`record_bag`, `live_topics`, `topics` (empty means the same topic list as `record_run.py`),
`min_free_gb` and `stop_timeout_sec`.

Recordings go to `~/racerbot-ws/data/racerbot_runs`. Pass
`record_output_directory:=<dir>` to bringup to send them elsewhere.

**Tests.** These files cover it:

- `test_auto_record.py`: the decisions, no ROS.
- `test_run_activity.py`: what `activity` says about a run, no ROS.
- `test_save_runs.py`: `tools/save_runs.sh` against a throwaway git repository, no ROS.
- `test_bringup_auto_record.py`: every bringup includes the recorder, on by
  default and off with `record:=false`, and a missing `race_diagnostics`
  cannot stop bringup.
- `test_auto_record_node.py`: the real node on an isolated ROS domain. It
  records, closes the bag, defers to another recorder and survives being
  killed. It also judges how much of a run's start the bag misses, from the bag's
  own first message. It takes about two minutes.

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
| `system/proc.jsonl` | `system_log`, every second | CPU use, memory and thread count for each named ROS node process. CPU use of 100% means one full core. Was a node starved of CPU? |
| `system/tegrastats.log` | The Jetson's `tegrastats`, every second, when available on the car | CPU use per core, GPU use, memory and temperatures |
| `bag/` | `ros2 bag record` | Everything, replayable offline. Includes both odometries (`/odom`, `/odom/imu`), both IMUs, SLAM's `/pose` and pose graph, and every node's log on `/rosout` |
| `metadata/git/` | `record_run.py` at start | Which code ran: HEAD, branch, `git status` and the full uncommitted diff of `racerbot-ws` and `~/imu-calibration`. Untracked files are listed, not copied |
| `metadata/files/` | `record_run.py` at start | Copies of imu-calibration's `car.yaml` and calibration JSONs: the IMU mounts and calibration, which no ROS parameter holds |
| `metadata/packages.txt` | `record_run.py` at start | The installed version of every `ros-jazzy-*` apt package, with `slam_toolbox` listed first. Git state cannot say which `slam_toolbox` ran, because it is apt-installed, not code in this repo — so this file is the answer to "did a package upgrade change SLAM" |
| `metadata/submodules.txt` | `record_run.py` at start | For each submodule in `.gitmodules` — team A's `src/racerbot_a` and B's `src/racerbot_b` included — the commit that was checked out, the commit the workspace itself records for it, and whether it is dirty. A `MOVED: <path>` line at the end names any submodule whose checked-out commit differs from what the workspace expects, which usually means someone advanced it without committing the change in the main repo |
| `metadata/params/` | `snapshot_params`, from 3 s in and every 2 s for the whole run | Each node's parameters in `ros2 param dump` YAML format (usable as `--params-file`). A node that starts later gets a file when it appears. A restarted node gets `<name>.<n>.yaml`. Namespaced nodes use `ns__name.yaml`; names that collide get `~2`, `~3`, and so on before any restart number. Later parameter changes are in the bag's `/parameter_events` |
| `map.pgm` / `map.yaml` | `slam_toolbox` save | The map as SLAM saw it |
| `raceline_*.csv` | `auto_map_race_node` | The line that was generated and driven. `raceline_raw.csv` is the unmodified recording and is written even when the cleanup *refuses* the run, so a refusal can be plotted against the map |

If `tegrastats` is missing, the recorder logs a warning and continues recording.

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
| elapsed time | `minimum_lap_duration_sec` (4.0 s), then also `minimum_lap_duration_fraction` (0.5) of the previous lap | The first lap needs 4 s. Later laps need the larger of 4 s and half the previous lap's time, so the gate follows the course length. The former fixed 15 s blocked short, fast laps; the 300° turning gate still applies. |
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

## SLAM report

After the car has stopped and the recording has finished, use this report to
check how [SLAM](glossary.md#slam) built the map. It reads the recorded bag and
does its analysis afterwards, so do not run it while the car is driving or
mapping.

In a terminal, from the workspace root, replace `RUN_DIR` with the path to the
recorded run. The first read needs ROS sourced; later reads use cached data in
the run's `analysis/` directory.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/run_analysis/slam_report.py RUN_DIR
```

The command prints seven sections and writes `slam_report.txt`, `summary.json`
and available PNG plots under `RUN_DIR/analysis/slam_report/`. To count loop
closures, add `--log LOG_PATH` with the path to the launch terminal log. Without
that log, the loop-closure count is unknown. The command uses `launch.log` in
the run directory when it exists. Add `--refresh` to reread the bag, or
`--no-plots` to skip PNGs.

To compare two runs, replace `RUN_A`, `RUN_B` and `OUTPUT_DIR` with paths in
this terminal command:

```bash
python3 tools/run_analysis/slam_report.py --compare RUN_A RUN_B --out OUTPUT_DIR
```

The comparison prints the numbers side by side and writes overlay plots to the
output directory. It does not write into either run.

### Reading the seven sections

1. **Setup** compares SLAM's configured odometry frame with the frame in the
   recorded transforms. A mismatch is flagged. It also gives wheel
   [odometry](glossary.md#odometry--odom) distance and mean and maximum speed
   while moving faster than 0.3 m/s.
2. **Heading drift** shows how far two heading estimates drift apart after
   starting at the same difference. Smaller differences are generally better,
   but SLAM uses the odometry it was given, so agreement alone does not prove
   either estimate is correct.
3. **Corrections and loop closures** counts changes SLAM made to its map
   transform, the largest movement and turn, and jumps over 0.3 m and 1.5 m.
   A Ceres solve in the launch log is used as a proxy for a loop closure.
4. **Laps** lists each closed lap's distance, closest approach to its start,
   corrections absorbed and quality verdict. Laps before the last requested
   lap are not judged. A judged lap is marked failed when the supervisor logs
   a failure; it may be accepted anyway at the maximum lap count, which the
   report also says. A judged lap with no failure is marked passed only when a
   later message confirms that its racing profile was written. Without that
   evidence, its verdict is unknown. So each lap gets exactly one of "not
   judged", "passed", "failed", "failed (accepted at the ceiling)" or
   "unknown", always from what the supervisor logged.
5. **Map consistency** is the percentage of finite LiDAR endpoints that land
   on or within one grid cell of an occupied wall in the final map. A higher
   percentage means more of what the LiDAR saw lines up with those walls. It
   is shown for the whole run and for each lap window. Each lap's scans are
   placed using the poses SLAM published at that time, then compared with the
   final map. Later loop closures can move the map, so early laps may score
   lower even when the final map is not wrong. In the 19:49 run, laps 1 and 2
   scored 19–27%, while lap 4 scored 55%. Compare corresponding laps between
   runs rather than treating an early-to-late difference as a map fault.
6. **Scan drops** counts dropped scan messages recorded on `/rosout`. The
   10-second bins also show slam_toolbox CPU use when a system log exists;
   100% means one full CPU core. A missing system log leaves CPU unknown.
7. **Wheel vs scan-matched distance** compares the wheel path with SLAM's
   map-frame path over the same time span. A ratio near 1 means their lengths
   agree, though it does not establish that either path is accurate.

For example, run `20261026-194913` used `odom_imu` in both the parameter
snapshot and the transforms. The final heading difference between `/odom/imu`
and SLAM was −16.44°, compared with 102.61° for `/odom` and SLAM. The report
found 32 scan drops, but that older recording has no system log, so CPU use
cannot be checked. Its map consistency was 34.1% overall, and three corrections
moved the estimated car position by more than 1.5 m. Those low agreement and
large corrections are reasons to investigate, rather than a healthy-map
verdict. Laps 3 and 4 failed their heading quality check at 29.5° and 29.2°
against a 15° limit; the run did not save a map.

### Limitations

The final map and SLAM poses are estimates, so consistency is not independent
proof of map accuracy. Missing inputs are labelled in the report. If SLAM did
not map the run, the report says so and still shows the other available data.

---

## Replaying SLAM with the car parked

`tools/run_analysis/slam_replay.py` runs SLAM again on a recorded bag and
writes its results into a new run folder. Use it to compare settings or check
whether SLAM can reproduce the recorded poses without another drive.

**Highlights**

- Starts SLAM with the recording's parameter snapshot, plus optional profile overrides.
- Preserves the recorded odometry TF tree in forced mode and recomputes map corrections.
- Forces the recorded scan-admission sequence for stamp-by-stamp pose comparison.
- Saves a replay bag, logs, map files when available, and a `replay.json` summary.

**Stop bringup before replaying.** The command refuses while a
[VESC](glossary.md#vesc) motor-controller, LiDAR or joystick driver process is
alive. Simulator joystick processes also match this
check. Every replay child uses ROS domain 77 with localhost discovery, keeping
the replay's messages on a separate ROS graph from the car.

### Run an exact replay

In one terminal at the workspace root, replace `RUN_DIR` with the recorded run
folder. This example uses `odom_imu`, the wheel-and-gyro odometry used by run
`20261026-194913`. Use `--odom odom` for a recording mapped on wheel odometry.

```bash
RB_WS="$(git rev-parse --show-toplevel)" rb python3 tools/run_analysis/slam_replay.py \
  RUN_DIR --exact --odom odom_imu --label exact
```

The machine's `rb` wrapper loads the ROS environment. The replay prints its
output folder on success. Open that folder's `replay.json` to check `status`
and `pose_sequence_error`; a timeout prints the failed scan stamp and keeps the
logs. See the measured limitation below before trying the 19:49 recording.

Output defaults to
`~/.ros/racerbot_runs/RUN_NAME-replay-odom_imu-exact/`. Existing output folders
are refused, so use a different `--label` for another attempt. Add
`--out-root /tmp/slam-replays` to put results in a scratch directory instead.
The source run is read without being modified.

Resolved output paths may enter an existing run only under its `analysis/` directory; a `bag/` or `metadata/` directory marks a run.

### Forced admission: the default exact mode

`--exact` selects `--exact-mode forced` by default. It starts `sync_slam_toolbox_node` and
uses the recorded `/pose` stamps to identify the scans the car processed.
These are the **planned scans**. It feeds them in recorded pose order,
waiting for each matching `/pose` before advancing: strict lockstep.

The feed sequence is:

1. The first planned scan, followed by its pose acknowledgement.
2. The three recorded scans immediately before the second planned scan.
   These are **sacrificial scans**: they consume startup callback counts 2–4
   and must produce no poses. If one produces a pose, replay fails with its stamp.
3. The remaining planned scans, each followed by its pose acknowledgement.

The startup rejection is hard-coded in
[`slam_toolbox_common.cpp:795`](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp#L795).
Forced mode also changes four admission gates, while retaining the recorded
scan-matching, map and solver settings. For the 19:49 recording:

| Admission parameter | Recorded value | Forced value |
|---|---:|---:|
| `throttle_scans` | 2 | 1 |
| `minimum_time_interval` | 0.2 s | 0.0 s |
| `minimum_travel_distance` | 0.25 m | 0.0 m |
| `minimum_travel_heading` | 0.25 rad | 0.0 rad |

This forces the already-known admission sequence rather than trying to
reproduce timing-dependent rejections. On the car, every planned scan passed
the travel and time filters measured from the same previous processed pose.

With identical inputs those scans pass again; opening the gates removes the
callback-count and timing dependency without changing their scan-matching
configuration.

`replay.json` records `exact.feed_mode: "forced"` and, under
`exact.admission_overrides`, each original snapshot value and its override.
The original value comes from the recording even if a profile changed it.

A [TF transform](glossary.md#tf--transform--frame) relates a child coordinate
frame to its parent.

Forced mode publishes the recorded `/tf` messages, keeping their order and
transform payloads. It removes every transform whose parent frame is `map`,
so the car's map corrections cannot replace the replay's new corrections.

If a message mixes map and non-map transforms, only the map entries are
removed. `/tf_static` is published with its recorded transforms as before.

The needed chain is derived from the configured `odom_frame` and `base_frame`,
using the recorded dynamic and static tree. For the 19:49 run this is
`odom_imu → odom → base_link → laser`. The final edge is a fixed, static mount; both earlier
edges need dynamic interpolation brackets.

Each needed dynamic pair gets a sample at or before the scan and the first
sample at or after it. Before a planned or sacrificial scan, the feeder
publishes a recorded-message prefix that covers all those brackets. It keeps
message order rather than replaying the pairs as separate streams.

This lets [TF](glossary.md#tf--transform--frame) perform the same interpolation
and composition as on the car. Forced mode starts no `odom_tf_relay.py` and
does not publish the odometry topic. `exact.tf_source` is `"recorded"`, and
`exact.needed_tf_pairs` lists the derived dynamic pairs.

`/clock` advances to the scan stamp, never to a future bracket's stamp.
`exact.lookahead_samples` counts newly sent TF transforms beyond that stamp
in forced mode, or newly sent odometry messages in all-scans mode. Already
sent brackets are reused.

Both exact modes log a planned pose missing after its 10-second pose timeout
and continue. **More than ten consecutive missing planned poses ends replay
as failed**, naming the stamps.

A later pose can resolve an earlier timeout
in the final accounting; `exact.timed_out_stamps` retains its history.

Forced mode refuses before starting if a pose's matching scan was not recorded
or scan stamps are duplicated. It also refuses if the tree has no needed chain
or a pair lacks a planned or sacrificial scan's bracket. Bracket errors
name the pair and scan stamp.

It also refuses when there are fewer than three recorded scans before the
second planned scan, naming that second stamp.

A missing processed scan means the recording cannot reproduce that run;
even the first pose has no exception. Both exact modes refuse `--rate`
and mode-related profile overrides.

### All-scans admission experiment

Use `all-scans` to investigate SLAM's timing-dependent admission behaviour.
It sends **every recorded scan in bag order**, keeps the recorded admission
parameters, and waits for poses only at planned checkpoints. Extra poses are
recorded without stopping. `exact.admission_overrides` is empty in this mode.

All-scans mode keeps the odometry relay, as does rate mode. This preserves the
round-1 experiment, including its composed-odometry interpolation, instead of
changing both admission and the TF source at once. Its `exact.tf_source` is
`"odom_relay"`.

In the same terminal, replace `RUN_DIR` as above and choose a fresh label:

```bash
RB_WS="$(git rev-parse --show-toplevel)" rb python3 tools/run_analysis/slam_replay.py \
  RUN_DIR --exact --exact-mode all-scans --odom odom_imu --label all-scans
```

Check that the summary says `exact.feed_mode: "all-scans"`. A failed summary
with missing checkpoints can be the experimental result; see the measured
19:49 outcome below. This mode does not guarantee the car's admission sequence,
because recorded header stamps do not reproduce transform arrival timing.

Unplanned scans outside odometry coverage are sent with the available odometry,
without complete brackets. SLAM's TF message filter decides their fate.
`exact.unbracketed_scans` counts these sends; forced mode sends no such scans.

For each covered all-scans input, the feeder sends odometry through the first
message at or after that scan, letting the relay build its transform.

The 19:49 bag has 98 scans before odometry starts and one after it ends, so
a complete all-scans feed would send 99 unbracketed scans. Planned scans still
require brackets in either mode.

Without `--exact`, the command uses asynchronous SLAM and plays all scans at
`--rate` (default `1.0`). A YAML file passed with `--profile` overrides the
recorded settings. Rate replay is useful for tuning, but an overloaded SLAM
node can drop different scans, so its pose sequence is not a tick-for-tick
reproduction.

### Keeping scan receipt observable

The feeder checks that each bracketed scan reached SLAM's scan callback before
sending another. This receipt check applies even when SLAM rejects the scan;
it does not wait for an unplanned pose. It prevents a burst of replay input
from overrunning SLAM's subscription or its TF message filter.

In slam_toolbox 2.8.5, the synchronous processing queue is an unbounded FIFO:
[`slam_toolbox_sync.hpp:54`](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/include/slam_toolbox/slam_toolbox_sync.hpp#L54)
declares it, and
[`slam_toolbox_sync.cpp:47–53`](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_sync.cpp#L47-L53)
removes entries in order. Exceeding ten entries emits a warning, not a drop.
The earlier
[TF message filter](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp#L472-L475)
is separate and bounded; this recording sets `scan_queue_size: 1`.

The scan publisher uses reliable `KEEP_ALL` history, so it has no fixed-length
outgoing queue that discards older scans.

For a covered scan, SLAM's
`map → odom` transform header acknowledges receipt of that scan's callback.
The check requires `restamp_tf: false` and a positive
`transform_publish_period`, as recorded in the 19:49 run.

In all-scans mode, unbracketed boundary scans cannot have this acknowledgement;
the feeder yields between sends so
SLAM can handle them through its TF filter.

### Measured result on the 19:49 run

Run `20261026-194913` contains **3,826 scans and 315 poses**. Every pose stamp
matches a distinct recorded scan. The SLAM report's **314 updates** count
changes between successive `map → odom_imu` corrections; the initial
correction is identity. That count does not mean a pose lacks its scan.

#### Forced admission with recorded TF: round 3

**The 1 mm / 0.1° bar was met.** The `recorded-tf3` replay matched all **315
poses**, with none missing or extra. Maximum errors were
**1.171361 × 10⁻¹³ m** and **3.053332 × 10⁻¹³°**; `first_over_bar` is null.
It sent 318 scans: the 315 planned inputs and three sacrifices, which produced
no poses. All 318 had callback receipts. The map and pose graph were saved.

The dynamic pairs derived from the recording are `odom_imu → odom` and
`odom → base_link`; the final `base_link → laser` edge is static.

The replay
bag retained **9,475 non-map TF messages**, verified to have identical payloads
and order to the required source prefix. The source contains 9,487 such
messages in total; the remaining tail is beyond the last required bracket.

The filter removed **4,606 map-parented transforms** from the candidate stream.
There are **no mixed messages** in this source bag. Synthetic tests also check
that a mixed message retains its non-map transforms rather than being dropped
as a whole.

#### Comparison with round 2

Round 2 (`forced1`) used the odometry relay and matched all 315 stamps, but
failed the numerical bar: maximum errors were **2.246775 m** and **3.625845°**
at pose 59. Its first exceedance was pose **20**, stamp
**1793069383585897818 ns**, at **0.00998585 m** and **0.199962°**.

The comparison uses one-based pose indices and rounded values.

| Pose | Round 2 position (m) | Round 3 position (m) | Round 2 yaw (°) | Round 3 yaw (°) |
|---|---:|---:|---:|---:|
| 2 | 0.00004318 | 9.02e-15 | < 0.000001 | 2.54e-14 |
| 10 | 0.00012115 | 2.89e-14 | < 0.000001 | 5.09e-14 |
| 19 | 0.00000583 | 2.94e-14 | < 0.000001 | 0 |
| 20 | 0.00998585 | 2.86e-14 | 0.199962 | 2.54e-14 |
| 50 | 0.01999860 | 2.39e-14 | 0.399925 | 1.02e-13 |
| 59 | 2.24677468 | 8.14e-15 | 3.625845 | 2.54e-14 |
| 100 | 0.12057477 | 1.94e-14 | 0.586101 | 5.09e-14 |
| 315 | 0.72786463 | 8.10e-14 | 0.065573 | 1.53e-13 |

Round 2 interpolated the already-composed `/odom/imu` pose. The car's recorded
TF tree interpolates `odom_imu → odom` and `odom → base_link` separately, then
composes them. At pose 20, those odometry guesses differed by **0.00003035 m**,
with no measured yaw difference. Round 3 preserves the recorded tree instead.

Round 2's first yaw discrepancy was one fine-search angular step, **0.00349 rad**
(0.199962°). Karto passes this configured value as the angular resolution in
[`Mapper.cpp:624–628`](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/lib/karto_sdk/src/Mapper.cpp#L624-L628).

The recorded-TF result removes that discrepancy and the later large errors.
This supports the interpolation difference as the cause of the earlier
scan-match divergence.

Ceres threading can also affect numerical reproducibility: reductions are not
bit-identical across thread counts. In 2.8.5,
[`ceres_solver.cpp:166`](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/solvers/ceres_solver.cpp#L166)
hard-codes `num_threads = 50`; the replay log reports Ceres capping that to
**8**.

The plugin exposes no thread-count parameter, so a controlled one-thread
replay was not available. The car's effective thread count is unknown. No
thread-count change was needed to meet the bar with the recorded TF tree.

#### All-scans timing experiment

The all-scan replay with slam_toolbox **2.8.5** sent **973 scans**, then stopped
after **11 consecutive planned-pose timeouts**. Its bag contains **12 extra
poses and no matching pose stamps**.

The comparison reports `matched: 0`, `missing_in_replay: 315`,
`extra_in_replay: 12`, `max_position_m: 0.0`, `max_yaw_deg: 0.0`, and
`first_over_bar: null`. The 315 missing stamps include planned scans not
reached after the abort; eleven were actually attempted and timed out.

**The 1 mm / 0.1° reproduction bar was not met.** With no matched stamps,
the zero numeric errors and null `first_over_bar` provide no accuracy
measurement. The first missing planned stamp is **1793069360801010451 ns**.

Replay first diverges by admitting scan **1793069359721058241 ns** (zero-based
bag scan index 98), which produced no pose on the car. The car's first pose
instead used scan index 141, at **1793069360801010451 ns**. The car's `/rosout`
records TF-filter queue-full drops, including scan time **1793069360.650 s**,
before that first pose.

Transform arrival timing differs from header timing: the first recorded
`odom_imu → odom` transform has stamp **1793069359712544260 ns**, but the bag
received it **488.759 ms later**.

Replay provides recorded odometry brackets
before each covered scan, making transforms available earlier. The recorded
`throttle_scans: 2` then makes differences in callback admission affect later
scan choices.

The evidence points to input-admission differences, not solver
numerical error; no corresponding poses exist to compare numerically.

All **875 bracketed scans** sent had callback receipts, and **98 leading
unbracketed scans** were also sent. A full feed would send **99 unbracketed
scans**, including the final scan after odometry ends, but the consecutive
timeout guard stopped this attempt before that final scan.

Its `exact.lookahead_samples` count is **867**. The original output label was
`all-scans1`. A repeat with the explicit `--exact-mode all-scans` flag, labelled
`all-scans2`, reproduced these counts and retained all four recorded admission
values with `exact.admission_overrides: {}`.

The round-3 `all-scans3` smoke replay also started the relay and reached the
same eleven-missing guard, confirming that this experiment kept its earlier
TF source and admission behaviour.

### When map saving fails

Both modes try SLAM's map-saving service up to three times. If no `map.yaml`
appears, replay closes and flushes its bag, then writes `map.pgm` and
`map.yaml` from the bag's last `/map`. This converts the recorded occupancy
grid, whose cells mark free, occupied or unknown space, into map files.

`replay.json` distinguishes `map_saved: true` for a service save,
`map_saved: "from_bag"` for this fallback, and `map_saved: false` if neither
produced a map. A forced service failure on the 19:49 recording at rate `2.0`
produced `map_saved: "from_bag"`, a 272,967-byte `map.pgm` and a 166-byte
`map.yaml`.

If a replay fails before reaching its save step, the summary can
still report `map_saved: false` even when an earlier map exists in the bag.

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
