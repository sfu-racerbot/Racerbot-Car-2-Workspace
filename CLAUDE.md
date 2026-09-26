# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS2 Jazzy workspace for a team's roboracer/F1TENTH car (Jetson Orin Nano Super, JetPack 7.2, Ubuntu 24.04). This is a physical robot that can hurt itself, people, or property if driving code is wrong — treat any change to a node that can publish `/drive` with corresponding care (see "Safety model" below).

**`docs/` is the primary source of truth and is unusually thorough — read the relevant doc before making non-trivial changes rather than inferring behavior from code alone.** Start with `docs/architecture.md` (node/topic graph, safety model) before writing or touching any driving code. Full index in [README.md](README.md).

## Build, test, run

```bash
source /opt/ros/jazzy/setup.bash        # every new terminal, always first
cd ~/racerbot-ws
colcon build --symlink-install          # full build
colcon build --symlink-install --packages-select <pkg>   # fast, single-package build while iterating
source install/setup.bash               # every new terminal, always second
```
- On the Jetson's 8GB RAM, add `--parallel-workers 1` to a full rebuild to avoid OOM.
- `--symlink-install` means edited `.py`/launch/config files take effect on next launch with no rebuild — only rebuild after touching `package.xml`, `setup.py`, C++ sources, or adding/removing files.
- `rm -rf build install log && colcon build` is safe if the build state is ever broken (these three dirs are gitignored, pure build artifacts).

Run the tests — **source ROS first.** `gap_follow`, `pure_pursuit`, and `usb_cam_stream` each contain test files that import `rclpy`; unsourced they fail to collect, hiding **177 of the 900 tests**, including every deadman, watchdog, and node-behavior test:
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m pytest src/<pkg>/test/ -v                              # one package
colcon test --packages-select <pkg> && colcon test-result --verbose
```
Verified 2026-08-22: unsourced, `src/pure_pursuit/test/` collects 185 of 274 and aborts on 5 collection errors, `gap_follow` hides 70, and `usb_cam_stream` collects nothing at all. **Never reach for `--continue-on-collection-errors`** — it turns that loud abort into a green run over a fifth of the suite. Compare the collected count against `grep -c '^def test' src/<pkg>/test/*.py`.

`drive_intent`, `odom_calibration`, `race_diagnostics`, `racerbot_sim`, and `web_dashboard` are pure Python and do run unsourced — that is the pattern to follow for any new package with non-trivial math or parsing: keep the logic importable without `rclpy` (`pure_pursuit/racing_math.py`, `web_dashboard/protocol.py`, all of `drive_intent`).

Drive the car (manual):
```bash
ros2 launch f1tenth_stack bringup_launch.py     # terminal 1: hardware + arbitration, never moves the car alone
ros2 launch f1tenth_stack teleop_launch.py       # terminal 2: control layer, hold LB, sticks = drive
```
Full command reference for every workflow (mapping, localization, autonomy, racing, shutdown) is in `docs/operations.md`.

## Architecture

Everything communicates over ROS2 topics only — no shared memory, no direct function calls between packages. The topic graph *is* the system; full diagram and topic table in `docs/architecture.md`.

**Two-tier launch pattern, always:**
1. `bringup_launch.py` (`f1tenth_stack`) — the shared foundation, started once: `joy_node`, VESC chain, LiDAR (`urg_node`), `ackermann_mux`. Deliberately starts nothing that can move the car by itself.
2. Exactly one **control layer** on top, in a separate terminal: `teleop_launch.py` (manual), `gap_follow_launch.py` (reactive autonomy), `pure_pursuit_launch.py` (map-based race controller), or a new node. Each publishes `AckermannDriveStamped` to either `/teleop` or `/drive`; `ackermann_mux` arbitrates.

**Arbitration:** `/teleop` (priority 100) always beats `/drive` (priority 10) while `/teleop` hasn't timed out. Verified behavior: `joy_teleop`'s default profile publishes continuously even when LB isn't held, so **`/teleop` never times out while `teleop_launch.py` is running** — meaning autonomy's `/drive` commands never reach the VESC if `teleop_launch.py` is also up. Running autonomy means simply not launching `teleop_launch.py`, not starting-then-stopping it.

**Packages** (`src/`):
| Package | Kind | Role |
|---|---|---|
| `f1tenth_system` (+ `ackermann_mux`, `teleop_tools`, `vesc`) | vendored, plain tracked files (not a submodule — see `docs/git-setup.md`) | VESC driver, `urg_node`, joystick teleop, command muxing; owns all launch files + YAML config |
| `transport_drivers` | git submodule (`humble`) | serial transport dep for `vesc` |
| `particle_filter` (+ `range_libc`) | git submodules (`humble-devel`) | Monte Carlo localization against a saved map |
| `gap_follow` | local | reactive autonomy, follow-the-gap on `/scan` → `/drive`, no map — the reference template for new driving nodes |
| `pure_pursuit` | local | map-based race controller: recorded+paced racing line, pure pursuit control, reactive safety net, opponent overtaking — see `docs/racing-autonomy.md` |
| `drive_intent` | local | shared schema + trajectory prediction for `/drive_intent` (what a driving algorithm is trying to do, and why); pure Python, no `rclpy`, plus a single-header C++ port for `racerbot_a`/`racerbot_b` — see `docs/drive-intent.md` |
| `racerbot_sim` | local | F1TENTH Gym exposed as the car's own topics (`/scan`, `/odom`, TF), replacing only the hardware layer so the real stack — SLAM, `auto_map_race`, the dashboard — runs unchanged above it. Has a hard interlock against running beside real drivers — see `docs/ros-simulator.md` |
| `web_dashboard` | local | browser dashboard over WebSocket. Publishes to **no ROS topic**, so it is not subject to the safety policy below. Four bounded write paths: live tuning (`set_parameters`), stopping a driving process (OS signals), resetting live SLAM (`slam_toolbox/reset`, refused while a driving node runs), and deleting a saved run directory on disk. All on by default; see `docs/web-dashboard.md`'s security note |
| `race_diagnostics` | local | read-only run recorder + post-run analyzer (pipeline health, pose lag, watchdog stops, rosbag) — subscribes only; see `docs/run-diagnostics.md` |
| `usb_cam_stream` | local | MJPEG webcam stream over plain HTTP |
| `racerbot_launch` | local | launch glue not owned by any single driver repo (SLAM, race-day localization+pure_pursuit combos) |

`slam_toolbox` is apt-installed (`ros-jazzy-slam-toolbox`), not vendored.

## Safety model — read before writing or running any driving code

**Workspace policy, currently in force, do not relax unilaterally:** every node that can move the car requires the driver to be actively holding **LB** on the physical F710 controller (XInput mode), on top of `ackermann_mux` arbitration. This is enforced in code, independently, by every autonomy node — `gap_follow_node` and `pure_pursuit_node` each subscribe to `/joy` directly and refuse to publish non-zero drive commands without a live LB hold (`enable_deadman: true` default in both configs). **Never set `enable_deadman: false`** — that's a unilateral policy change, not a tuning knob. Full reasoning: `docs/architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car`.

**Any new node that publishes to `/drive`, `/ackermann_cmd`, or `/commands/motor|servo/*` must implement the same LB deadman check** — copy-paste pattern from `gap_follow_node.py`'s `joy_callback`/`_deadman_engaged`, documented in full in `docs/writing-your-own-node.md`. Decide which category new code falls into (driving vs. support/tooling) using `docs/adding-your-own-code.md` — when in doubt, treat it as driving code.

**Diagnostics published from a driving node** (currently only `/drive_intent`) must never be able to cost the car anything: publish strictly *after* the drive command for the tick, wrap the whole thing in one try/except that disables the diagnostic rather than the node, and read only what the control path already computed. Full contract and tests: `docs/drive-intent.md#safety-contract-for-publishers-read-this-first`.

Test order for any new driving node, never skip ahead: static topic check (no driver stack running) → wheels off the ground (full stack + LB held) → floor, low speed, open space. See `docs/writing-your-own-node.md#testing-before-its-on-wheels`.

## Test Quality — binding on all test work in this repo

**Source of truth: [TEST_QUALITY_STANDARDS.md](TEST_QUALITY_STANDARDS.md)** — the reasoning, worked examples from this tree, and copy-paste sweeps live there. This section is the short version, meant to be re-read every session. **It governs all test-writing in this repo from now on** — new packages, edits to existing suites, and work done in future sessions by anyone, human or agent. It is not scoped to the audit that produced it.

The rule behind all of it: **a test that cannot fail is worse than no test.** It costs review time, occupies the name of coverage it isn't providing, and turns "the suite is green" from evidence into noise. On a car that can hurt someone, that is a safety problem, not a tidiness one.

### Hard rule 1 — prove every new test can fail, before calling it done

**Before marking any new or changed test complete, break the code it covers and watch the test fail.**

1. Stub the unit under test — `pass`, `return <the exact constant the test expects>`, or `raise NotImplementedError`.
2. Run only that test.
3. It **must** fail. If it still passes, the test is hollow — it is detecting nothing. Rewrite it.
4. Revert the break, re-run, confirm green. Never commit a stub.

`return <the expected constant>` is the case people skip and the one that catches a test that has memorised an answer instead of checking a computation. For math, geometry, and control code, also require at least one of these to be caught: flipped comparison (`<`→`<=`), negated sign, a constant changed by 10%, two adjacent same-typed args swapped, an input returned unchanged, a clamp deleted. Line coverage is not evidence — a test can execute every line of `racing_math.py` and assert nothing about any of it.

A test never seen to fail is an assumption, not a test.

### Hard rule 2 — never weaken a test to make a suite pass

**A failing test is a finding, not an obstacle. The default response is to fix the code.**

Do not widen a tolerance, weaken an equality to a bound or a bound to a type check, drop the edge case that failed, add `skip`/`xfail`, or delete the test — and never do any of these in the same change as an edit to the code it covers.

If a test genuinely looks wrong after a spec change, **stop and flag it to me with your reasoning instead of editing it silently.** Say what the old test asserted, what the new behavior is, **where that behavior is written down** (a `docs/` section, datasheet, or message definition), and why the old behavior was *wrong* rather than merely inconvenient. "Flaky", "no longer relevant", "failing after the refactor", and "the new value is what the code does now" are not reasons.

### Anti-pattern checklist

| # | Reject a test that… |
|---|---|
| A1 | asserts on a mock's own `return_value`, or whose only assertion is `assert_called_*`. Mock only a real process boundary (socket, serial, clock, filesystem) — **never** our own ROS-free modules (`racing_math`, `protocol`, `netbind`, `drive_intent`); they were split out precisely so they can be called for real. |
| A2 | would still pass against a stub — asserts only a type or shape (`isinstance`, `len(out) == 2`), only that nothing raised, or only on a value the test itself passed straight through. |
| A3 | hardcodes today's output with no independent oracle. Every expected value must trace to a **closed form** recomputed in the test, an **invariant**, a **cited spec** (give the path), or a **recorded measurement** (name the run) — and the test must say which. An unexplained golden number is a change detector: label it `# change-detector (not an oracle)` and give the same unit a real-oracle test too. |
| A4 | covers only the happy path. Required edges: zero, negative, exactly at the boundary and one step past it, `NaN`/`inf`, empty and single-element input; parsers also need truncated, wrong-type, unknown-field, oversized, malformed-UTF-8; scan consumers also need all-`inf`, all-zero, `NaN` beams, wrong beam count, stale stamp. For driving code the specified edge behavior is nearly always "command zero / publish nothing", never "raises whatever it happens to raise". |
| A5 | asserts something trivially true, swallows exceptions (`except: pass`), uses `pytest.raises(Exception)` rather than a specific error with `match=`, uses a bare `pytest.approx` with no explicit tolerance and unit (`abs=1e-3  # 1 mm`), or hides its only assertion behind an `if`/loop filter that may match zero times and pass vacuously. |
| A6 | was loosened, skipped, or deleted because it was failing — see Hard rule 2. Every `skip`/`xfail` needs a reason naming an issue or doc section and a condition for removal; prefer `xfail(strict=True)`. |
| A7 | takes its verdict from the thing under test. Above all: gym's own collision flag never fires with this workspace's geometry — a car drove 35.5 m through a barrier unflagged. The pass/fail signal must be computed independently of the system producing the behavior. |
| A8 | tests only the permissive side of a safety check. "LB held → the car drives" is the half that fails safely; the refusal is the half that matters. |
| A9 | isn't run by the command people actually run. State which runner executes a new test, and check it is actually collected — never paper over a collection error with `--continue-on-collection-errors`. |

### Standing requirement — the LB deadman

Every node that can publish `/drive`, `/ackermann_cmd`, or `/commands/motor|servo/*` implements the deadman gate independently, so **each node needs its own four deny-path tests — another node's coverage does not transfer.** Assert the published **command**, not an internal flag:

| State | Required assertion |
|---|---|
| LB held | non-zero command published |
| LB released | zero command / no command |
| `/joy` never received | zero command / no command |
| `/joy` stale past timeout | zero command / no command |

`enable_deadman:=false` is permitted in a node test **only** when the same node construction also passes `drive_topic:=/test_only/drive` — without the remap, a test publishes live commands straight into `ackermann_mux` if the driver stack is up. Never in shipped config, and never in a test whose subject *is* the deadman.

## Package anatomy (local `ament_python` packages)

Every local package (`gap_follow`, `pure_pursuit`, `web_dashboard`, `racerbot_launch`) follows the same shape — see `docs/concepts.md#anatomy-of-a-package` for the full breakdown:
```
src/<pkg>/
├── package.xml            # deps: rclpy always; sensor_msgs/ackermann_msgs/nav_msgs as needed
├── setup.py / setup.cfg   # ament_python build, registers console_script entry points
├── resource/<pkg>         # empty marker file, required by ament_python
├── <pkg>/                 # importable module; ROS-dependent node + any framework-agnostic logic split out
├── launch/<pkg>_launch.py
├── config/<pkg>.yaml      # tune behavior here, not in code — every knob is a declared ROS parameter
└── test/                  # only for logic with no rclpy dependency (see racing_math.py, protocol.py)
```
New packages go under `src/`, one per feature — don't add files into an existing package unless genuinely extending its purpose.

## Known non-obvious facts worth knowing before you touch things

- `f1tenth_system` is vendored, not a submodule, specifically to carry four committed local fixes: `joy_teleop.yaml`'s `human_control` steering axis (`axis: 3`, not upstream's `axis: 2` — this F710's right stick, not its left trigger); splitting `joy_teleop` out of `bringup_launch.py` into its own `teleop_launch.py`; `vesc_to_odom_node`'s **negative** `speed_to_erpm_gain` (`-4447.983786` after a September 2026 recalibration; the sign is the fix, the magnitude is calibration); and this car's measured geometry (`wheelbase: 0.36` in `vesc.yaml`, and `0.26 0.0 0.11` on the `base_link`->`laser` static transform in all three `*_bringup_launch.py`). All four get silently clobbered by a naive upstream sync, and the last two fail silently — see `docs/git-setup.md` before touching this package.
- **The car's geometry was tape-measured on 2026-08-24 and is not the Traxxas spec.** Wheelbase **0.36 m** (not 0.324), **0.30 m** outer tire to outer tire (not the 0.281 body figure), LiDAR **0.10 m behind the front axle** = **0.26 m** ahead of the rear-axle `base_link` (not 0.33). `car_width: 0.31` is that 0.30 plus 5 mm a side of deliberate padding (cut from 14.5 mm/side on 2026-08-25, after false `emergency_clearance` latches in real run logs tracked the old padding almost exactly — see `docs/hardware-reference.md`); `car_length: 0.58` is unchanged and still pads an **unmeasured** overall length. Consequences: minimum turning radius is **1.35 m**, not 1.22 m, and racing lines recorded under the old LiDAR offset are ~0.07 m out. Every file each number has to stay in step with is listed in `docs/hardware-reference.md#physical-dimensions-used-in-config` — change a row together or two parts of the stack model different cars.
- Servo position `0.5304` is neutral/center, not a bug (`servo_position = -1.2135 * steering_angle + 0.5304`, see `docs/hardware-reference.md`).
- None of the upstream f1tenth/roboracer repos have a `jazzy` branch yet; everything here is `humble`/`humble-devel` source built against Jazzy.
- Two simulators, deliberately: `tools/f1tenth_sim/run_validation.py` calls the controller *math* with no ROS at all (`docs/simulator.md`), while `racerbot_sim` puts the same F1TENTH Gym physics behind the real ROS topics so whole launch files can be validated (`docs/ros-simulator.md`). Wiring bugs are invisible to the first and are most of what has actually broken. The official ROS bridge `f1tenth_gym_ros` is still not installed.
- **`racerbot_sim` must never run beside the real drivers**: `sim_joy_node` forges the LB deadman and `gym_bridge_node` publishes a second `/scan` and `/odom`. Both refuse while `vesc_driver_node`/`urg_node`/`joy` are on the graph, re-checked continuously — but do not defeat that check.
- **Maps live in three unrelated places and there is no canonical store.** `~/.ros/racerbot_auto/<ts>/` (what `auto_map_race_node` writes), `~/.ros/racerbot_sim/auto/<ts>/` (the simulator's), and the git-tracked upstream demo maps in `src/particle_filter/maps/`. The manual `map_saver_cli` workflow writes into whatever CWD you ran it from. The dashboard's delete panel only ever touches the first two, and refuses anything inside a git working tree — **never make the third deletable**.
- In the pinned F1TENTH Gym revision, gym's own collision check **is not trusted by anything here**. Measured under the pre-2026-08-24 geometry it never fired at all (`side_distances` computed to all zeros with the LiDAR outside the ±0.29 m box, and `range_min` 0.05 exceeds the 0.005 `ttc_threshold`) — a car drove the whole circuit through walls unflagged. The corrected 0.26 m LiDAR offset puts the sensor back inside that box, so the all-zeros half of that no longer holds, and **nobody has re-measured whether the flag fires now**. Neither simulator depends on the answer: `tools/f1tenth_sim/sim_fidelity/` checks the chassis against the map distance transform and does car-to-car overlap itself (`tools/f1tenth_sim/README.md`), and `racerbot_sim` samples the body against the occupancy grid. Do not reinstate a "no collision" claim that rests on gym's own flag.
