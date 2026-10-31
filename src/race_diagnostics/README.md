# race_diagnostics

> **Who this is for:** someone reading or changing this package's code.
> **Read first:** [docs/run-diagnostics.md](../../docs/run-diagnostics.md) for the recording and analysis workflow.
> **What's in it:** the recorder and analyzer internals. Read-only — this package publishes to no topic and cannot move the car.

Read-only run recorder and post-run analyzer for this car's driving stack.

**This package only ever subscribes.** It publishes to no topic and cannot
influence how the car drives, which puts it in the support/tooling
category rather than driving code (see
[docs/adding-your-own-code.md](../../docs/adding-your-own-code.md)) — the
same category as `web_dashboard`. It is safe to leave running alongside
anything, including a race. If it ever gains a publisher it becomes
driving code and inherits the LB deadman policy in full.

`auto_record` goes one step further: it neither publishes nor subscribes.
It only reads the ROS graph (which nodes exist, who publishes what), which
is why every bringup can start it.

Full workflow, rationale, and the AI-agent prompt template:
**[docs/run-diagnostics.md](../../docs/run-diagnostics.md)**

## Use

**Usually nothing:** every `f1tenth_stack` bringup starts `auto_record`, which
records each run of an algorithm into `~/racerbot-ws/data/racerbot_runs/<time>/` and
prints the folder in the bringup terminal. Off with `record:=false`. See
[automatic recording](../../docs/run-diagnostics.md#automatic-recording).

By hand, for anything else (bringup's recorder then stands aside):

```bash
# Terminal 1 -- start FIRST so nothing is missed
ros2 launch race_diagnostics record_run.py

# Terminal 2 -- the driving stack, with tee (see the path it printed)
ros2 launch racerbot_launch auto_map_race_launch.py 2>&1 | tee <dir>/launch.log

# afterwards
ros2 run race_diagnostics summarize_run <dir>
```

## What's in here

| File | Role |
|---|---|
| `race_diagnostics/race_diag_node.py` | The ROS probe. Watches every link of the pipeline; records `events.jsonl`. |
| `race_diagnostics/run_events.py` | Pure logic: log classification, lap-gate parsing, run timeline. No rclpy — unit-testable with a bare `pytest`. |
| `race_diagnostics/filter_log.py` | Follows a tee'd launch log, prints only the lines that matter. |
| `race_diagnostics/summarize_run.py` | Turns a run directory into one page of findings (`--json` for agents). |
| `launch/record_run.py` | Probe + rosbag into one timestamped directory. `run_directory:=` and `name_prefix:=` are for `auto_record`. |
| `race_diagnostics/auto_record.py` | Pure logic: when the automatic recorder starts and stops. No rclpy. |
| `race_diagnostics/auto_record_node.py` | The automatic recorder bringup starts: reads the graph, runs `ros2 bag record` and `record_run.py` while an algorithm runs. |
| `race_diagnostics/recording.py` | What a recording bags and the command that bags it, shared by `record_run.py` and `auto_record`. No rclpy. |
| `race_diagnostics/run_activity.py` | After a recording closes: adds what the car did to `auto_record.yaml`, including whether the bag missed the start. |
| `launch/auto_record_launch.py` | Starts `auto_record`; included by every `f1tenth_stack` bringup. |
| `config/race_diagnostics.yaml` | Topics, reporting cadence, pose-lag alert threshold. |
| `config/auto_record.yaml` | Which topics mean "an algorithm is running", which nodes are another recorder, the start/stop delays. |

## Tests

`run_events.py` and `auto_record.py` are deliberately free of ROS dependencies, so:

```bash
python3 -m pytest src/race_diagnostics/test/ -v     # no sourcing, no build
```

**Working when:** everything passes, with three skips: the file that runs
the real `auto_record` node (one skip for all four of its tests) and two
tests that load the real bringup launch file. They need ROS. To run those too, source first, on an isolated ROS domain
(never the car's):

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest src/race_diagnostics/test/ -v     # about a minute; no skips
```

The test fixtures are real log lines from the 2026-07-27 session, so the
classifier is verified against output the car actually produced rather
than output we imagined it would.

## The one number to look at

**Worst pose lag.** Localization staleness is invisible in a topic's
message rate — `auto_map_race_node` republishes SLAM's transform at a
fixed 40 Hz whatever its age, so a frozen transform arrives just as
punctually as a live one. Only the header stamp reveals it, and a stale
pose is what put the car into a wall. Under 0.15 s is healthy; over 0.5 s
and `pure_pursuit` will hard-stop through the stalls.
