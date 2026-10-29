# Neural racer: design and implementation plan

> **Who this is for:** the team deciding whether to build a learned racing policy for car 1, and whoever implements it, human or agent. Implementers must also read `CLAUDE.md`, `TEST_QUALITY_STANDARDS.md`, [architecture.md](../../architecture.md) and [racing-autonomy.md](../../racing-autonomy.md) before starting any phase.
> **Read first:** [architecture.md](../../architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car) — the LB [deadman](../../glossary.md#deadman) rule. Nothing in this plan relaxes it.
> **You'll be able to:** say what network to build, what goes in and comes out, how to train it on FIR so it wins races against other cars, and then build it phase by phase, file by file, with the test that ends each phase.
> **Time:** 20 minutes for [Part A's first section](#a1-the-plan-in-one-screen). Part B is a work reference, read one phase at a time.

**Status:** proposed, 2026-10-23.

- Revision 2 made the IMU a prerequisite (Phase 0) and an input, and pre-wired the camera as an option.
- Revision 3 records the team's decisions: 1–4 cars with a grid start, a default FIR allocation, a second car for low-speed tests, the IMU at the front bumper, and the camera after the first network. It also adds the [questions and answers](#common-questions) section, a second track generator, handover episodes, and fading of beaten baselines.

- **Revision 4, 2026-10-23: implemented** on branch `neural-racer` (worktree `~/racerbot-ws-neural`). What exists, what was found while building it, and what is still to do: [Implementation status](#implementation-status).

---

## Implementation status

**Built and tested, on branch `neural-racer`:**

| Plan part | Where | Evidence |
|---|---|---|
| Phase 0 code: the `/imu/status` verdict on `/imu/data` | `src/imu_bridge/` (`imu_health.py`) | **Changed 2026-10-24:** `racerbot_imu` (conditioner, two-IMU selector, axis calibration, P0.5 fits) was removed, because it published a second `/imu/data` and `/imu/vesc/data` beside `imu_bridge`'s. `imu_bridge` now owns every `/imu/*` topic and publishes `/imu/status` for the primary IMU: startup bias, stationary gravity check, all-zero, non-finite and stale checks. Not ported: the two-IMU cross-check and the `calibrate_axes`/`imu_fit` tools (in git history). See the Phase 0 note below |
| Phase 2: observation spec, NumPy policy, safety wrapper, model store, `approve_model`, the node | `src/neural_racer/` | 141 tests, including the four deadman deny paths on the published command. Whole tick 3.90 ms median / 4.03 ms p99 on one Jetson core. Static launch check: an unapproved model is refused, and there is no `/drive` publisher |
| Phase 3: GPU simulator, parity | `tools/neural_racer/neural_racer_train/gpu_sim/` | dynamics match gym's functions to 1e-9; a 10 s trajectory matches `FidelityPlant` within 2 cm / 0.5°; observation and wrapper match the car's code to 1e-6 |
| Phase 4: two generators, checks, mutation, PLR, held-out set | `tracks.py`, `plr.py`, `heldout.py`, `heldout.yaml` | every generated track independently re-checked; held-out digests and seeds refused by the training pool |
| Phases 5, 6, 8: teacher, DAgger student, league role, benchmark | `teacher.py`, `distill.py`, `league_learner.py`, `benchmark.py` | each ran end to end on the Jetson's CPU; trainer contract (exit 85 on `SIGUSR1`, exit 0 when done, resume) tested on a real process |
| Export to the car | `export.py` | PyTorch and the car's NumPy agree to 1e-5; the unmodified `approve_model` accepts the export |
| Phase 9: referee, eval worker, selection | `referee.py`, `eval_worker.py`, `selection.py` | races the real `gap_follow` mirror and `pure_pursuit` on held-out tracks; Wilson sequential test, max-entropy Nash averaging and the veto tested against closed forms |
| Hard rule 1 | `tools/neural_racer/check_mutations.py` | applies one deliberate break per key test and requires it to fail |

**Found while building, and already changed in this plan:**

1. **The camera "starts identical" promise was false as first specified.** The camera features went through the shared LayerNorm, so they shifted its statistics even with zero weights, and the two camera flags also fed the ego layer. Now the LayerNorm covers only the non-camera features, and the ego layer's camera-flag weights start at zero too. Turning a camera on is now *exactly* identical until fine-tuning (A4, and `test_zero_initialised_camera_starts_identical`).
2. **The 0.6 m grid stagger overlapped cars on bends.** The 0.535 m bodies touched at the start. It is now 1.0 m (A8), and the referee refuses a race whose cars touch at the start.
3. **The parameter count is 646,274,** not "about 629k". The benchmark's 629k was measured before the 23-wide ego vector and the camera slot.
4. **The referee's pivot point differs from this simulator's.** Gym treats its state point as `base_link` (LiDAR 0.26 m and body 0.18 m ahead of it), but the single-track model it integrates describes the centre of gravity, 0.18 m ahead of the rear axle. So in the referee, the rear axle follows centre-of-gravity physics.
   - The LiDAR-to-body geometry is the same in both simulators, so scans agree; only the pivot differs.
   - The GPU simulator keeps the physically consistent convention: body at the centre of gravity, LiDAR 0.08 m ahead of it.
   - **Not changed, because it is existing code.** Decide whether to correct it in `tools/f1tenth_sim`.

5. **Tail latency doubles under CPU contention.** The whole tick measures 4.03 ms p99 on a quiet core, but **13.9 ms p99 with every core busy** (measured 2026-10-23, six busy loops on the Jetson).
   - The car runs SLAM and the particle filter beside the node, so Phase 6's gate, measured with the full stack up, is the one that decides.
   - Pinning `neural_racer_node` to its own core (`taskset`) is the first thing to try if it fails.
6. **Four tests could not fail, and were rewritten.** The mutation checker found:
   - the batched-safety parity test braked every sample to zero, so the cornering cap was never compared
   - the archive floor test was satisfied by PFSP alone
   - the Wilson test took its oracle from the code under test
   - and one miss was the checker's own bug: Python's whole-second bytecode cache ran the old code

**Simplified in v1, on purpose:**

- **Auxiliary-head targets** are extrapolated from the simulator's true state: constant turn for the ego path, constant velocity for opponents. They are not yet read from later rollout frames.
- **The race-context block** carries the lap fraction only. Place and gaps are sent as "place 1, no gaps" by both the node and the simulator, until the map-subtraction detector is wired into the node.
- **The map-mode reference path** in simulation is the centreline plus a smooth sideways offset and SLAM-like noise. It is not the path the car actually drove on lap 1.
- **Car 1's IMU config and launch** live in `racerbot_imu` (`config/imu_rb1.yaml`, `launch/imu_launch.py`), not `racerbot_launch`, so no existing package was edited.

**Still to do. Each needs existing code changed, or the car:**

- **Car work:** Phase 0's on-car steps (P0.1–P0.5), Phase 7 shadow runs, Phase 10 and 11.
- **Existing code, deliberately not touched on this branch:**
  - `sim_fidelity/calibration.py`: car 1's speed gain and steering lock. They live as `CAR1_OVERRIDES` in `gpu_sim/params.py` for now.
  - `sim_fidelity/sensing.py`: an `ImuSensor`. The referee's `NeuralDriver` derives the IMU from ground truth instead.
  - `racerbot_sim`: publishing raw IMU topics.
  - `racerbot_doctor`: the IMU ladder.
  - `auto_map_race`: the `race_controller` switch and `/auto_map_race/race_path`.
  - The docs: `hardware-reference.md`, `car1-vs-car2.md`, and the `CLAUDE.md` package table.
- **FIR:** the first `benchmark` run, which replaces the A9 throughput estimate with a measurement.

## How to use this document

**Part A is the design**: what to build and why. **Part B is the implementation**: one phase at a time, each with the files to create or change, the exact interfaces, the tests, and a **gate**: the measurable result that ends the phase.

- **Phases run in order.** Phase 0 (the IMU) comes first. Phases 1–4 need no car and no trained policy, so a second person can start them while Phase 0 is under way. **Nothing trains a policy meant for the car until Phase 0's gate has passed**, because the student policy's IMU model uses Phase 0's measurements.
- **A missed gate is a finding.** Report it with numbers. Don't loosen the gate, and don't move on.
- **Each phase lists its own stop points.** These apply to every phase:
  - **Stop and report** if a test seems to need loosening (`CLAUDE.md` Hard rule 2).
  - **Stop and report** if any change would need `enable_deadman: false` outside a node test that also remaps to `/test_only/drive`.
  - **Stop and report** if the Jetson would need a system-wide `pip install`.
  - **Stop and report** if a referee verdict and a training-simulator verdict disagree. That is a simulator bug, not noise.
- **Every new test must be seen to fail** against a broken version of the code it covers before it counts (`CLAUDE.md` Hard rule 1). Each phase's test table names the break to use.

---

## Contents

**Start here**

- [Implementation status](#implementation-status)
- [Common questions](#common-questions)

**Part A: design**

- [A1. The plan in one screen](#a1-the-plan-in-one-screen)
- [A2. What "winning" means, and how we'll know](#a2-what-winning-means-and-how-well-know)
- [A3. What the car can sense and do](#a3-what-the-car-can-sense-and-do)
- [A4. The network](#a4-the-network)
- [A5. The camera: an option that is already wired in](#a5-the-camera-an-option-that-is-already-wired-in)
- [A6. Safety: what wraps the network](#a6-safety-what-wraps-the-network)
- [A7. The training simulator](#a7-the-training-simulator)
- [A8. The training regime](#a8-the-training-regime)
- [A9. FIR and GPU efficiency](#a9-fir-and-gpu-efficiency)

**Part B: implementation**

- [B0. Conventions every phase uses](#b0-conventions-every-phase-uses)
- [Phase 0: IMU ready](#phase-0-imu-ready)
- [Phase 1: baselines and the held-out set](#phase-1-baselines-and-the-held-out-set)
- [Phase 2: the ROS-free core](#phase-2-the-ros-free-core)
- [Phase 3: GPU simulator and parity](#phase-3-gpu-simulator-and-parity)
- [Phase 4: tracks and the curriculum](#phase-4-tracks-and-the-curriculum)
- [Phase 5: privileged teacher](#phase-5-privileged-teacher)
- [Phase 6: student, export, and the node](#phase-6-student-export-and-the-node)
- [Phase 7: shadow mode on car 1](#phase-7-shadow-mode-on-car-1)
- [Phase 8: the league](#phase-8-the-league)
- [Phase 9: selection and the referee](#phase-9-selection-and-the-referee)
- [Phase 10: solo on the floor](#phase-10-solo-on-the-floor)
- [Phase 11: auto_map_race and two cars](#phase-11-auto_map_race-and-two-cars)
- [Phase C: adding the camera (optional)](#phase-c-adding-the-camera-optional)
- [Order of work](#order-of-work)
- [Risks, and plan B](#risks-and-plan-b)
- [Decisions for you](#decisions-for-you)
- [Sources](#sources)

---

## Common questions

Short answers, each linking to the section with the detail.

**Do the cars start not knowing the track?**
Yes. Lap 1 is driven in **reactive mode**: LiDAR, IMU and speed only, with no map and no memory of any track ([A4](#a4-the-network)). While it drives, SLAM builds the map. From lap 2 the same network switches to **map mode** and gets a preview of the track ahead.

In training, the GRU's memory is wiped at the start of every episode, so every episode is a first sight of that track as far as the policy is concerned. Generalization is then tested on held-out tracks no stage ever trains on ([A8](#choosing-the-release-never-the-latest-checkpoint)).

**Is the mapping our existing SLAM, with the path generated on top of it?**
Yes. Nothing new is built for mapping. The existing pipeline does it ([racing-autonomy.md](../../racing-autonomy.md#the-fast-path-map-and-race-from-one-launch)):

1. `slam_toolbox` maps while lap 1 is driven.
2. `auto_map_race_node` detects the closed lap and cleans the recorded path.
3. It freezes the map and keeps publishing the car's pose on `/slam_pose`.

The one difference is in what the network takes from that pipeline:

- **It takes:** the cleaned lap-1 path, used only as a ruler along the track, plus the left and right free width measured from the frozen map.
- **It doesn't take:** the minimum-curvature line or the velocity profile. It picks its own line and speed.

Phase 11 wires this up ([Phase 11](#phase-11-auto_map_race-and-two-cars)).

**Are the cars allowed to drive at their max speed?**
Not on day one. They work up to it in measured steps ([A6](#how-fast-the-car-is-allowed-to-go)).

- **The firmware cap** is 6.37 m/s (car 1's `vesc.yaml`).
- **In training**, the speed cap is randomized up to that 6.37 m/s, so the policy learns to race at any cap.
- **On the car**, the cap starts at 1.0 m/s and rises 0.5 m/s at a time after clean runs.
- **Two safety caps stay on top of the network:** a braking cap from what the LiDAR sees, and a cornering cap at `gap_follow`'s conservative 2.0 m/s². Raising the cornering cap is a separate, measured decision after Phase 10.

**How do we generate random tracks?**
A track comes from one of three sources, chosen per level ([Phase 4](#phase-4-tracks-and-the-curriculum)):

| Source | Share | What it produces |
|---|---|---|
| Smooth loops: random control points on a wobbly circle, joined by a closed spline | 40% | flowing tracks of varying width |
| Filleted polygons: a random polygon with dents, each corner rounded with a random radius | 40% | real-track features: long straights, hairpins, chicanes |
| Imported maps: scaled F1TENTH circuits and our own SLAM maps (training split only) | 20% | real layouts |

Every track is checked before use. It is rejected if any corner is tighter than 1.3 m radius (the car can't turn tighter than 1.05 m), if its edges cross, or if any gap is under 0.8 m wide. Then the curriculum replays the tracks the policy is learning most from, and mutates them into new ones.

**Are our current algorithms included as the baseline?**
Yes, in three places:

- **The yardstick (Phase 1):** `gap_follow`, and `pure_pursuit` on the optimized line, with and without traffic. These numbers define "better".
- **The benchmark panel:** the real code of `gap_follow` (with opponent awareness) and `pure_pursuit` (with overtaking, using whichever steering law is shipped). Every release has to beat both.
- **Training opponents:** GPU versions of the same algorithms are in the scripted pool. They are approximations for speed; the real code is only used in the referee.

**Once we beat the baselines easily, do they fade so they stop costing compute?**
Yes, in both training and evaluation, but never to zero:

- **In training:** opponents we beat get picked less automatically. The scripted pool's share also shrinks from 15% to a 3% floor as our win rate against it rises. The floor stays because forgetting how to beat a simple opponent is a known league failure, and 3% of matches is cheap insurance ([A8](#stage-3-the-league-phase-8)).
- **In evaluation:** matchups stop early once the result is statistically clear (a sequential test). A baseline we beat 95% of the time is settled in about 20 races instead of 200. A baseline beaten that decisively for 10 snapshots in a row is checked only every 10th snapshot after that ([Phase 9](#phase-9-selection-and-the-referee)).
- **At release:** the full panel is still always played. It runs on CPU nodes, so it costs no GPU time.

**How does a network trained on FIR get onto the Jetson?**
As a small file, not as PyTorch ([B0](#getting-a-model-from-fir-to-the-car)):

1. `export.py` on FIR writes the weights as a NumPy file (about 2.5 MB), plus a model card and a "golden" file of test inputs with the expected outputs.
2. You copy that folder to the car.
3. `approve_model` on the car checks its hash, its referee reports and its sensor list.
4. On the car, `PolicyNumpy` runs the network in plain NumPy and must reproduce the golden outputs to within 1e-5 before it is allowed to load.

Whether the *behaviour* transfers is a separate question. That is what the randomization, the simulator parity tests, the CPU referee and shadow mode are for ([A7](#a7-the-training-simulator), [Phase 7](#phase-7-shadow-mode-on-car-1)).

---

# Part A: design

## A1. The plan in one screen

- **One small network, two modes.** The network is a **1D convolution over the [LiDAR](../../glossary.md#lidar) [scan](../../glossary.md#scan), feeding a small recurrent core (a GRU, a layer that carries a memory from one tick to the next)**. It outputs steering and speed directly, 40 times a second.
  - In **reactive mode**, on a track it has never seen, it gets LiDAR, its own speed, and the IMU (the gyro and accelerometer).
  - In **map mode**, from lap 2 onward, it also gets a preview of the track ahead, built from the lap-1 map. It picks its own racing line; nothing tells it to follow one.
  - It is the same weights in both modes.
- **The IMU is an input, and it has to be fixed first.** The car has two, the VESC's and one at the front bumper: one feeds the network and the other cross-checks it. On car 2 the IMU read all zeros; on car 1 it has never been checked. The driver also publishes it in the wrong units. Phase 0 fixes that before anything trains for the car.
  - **Why it matters:** without a gyro, nothing measures how fast the car is turning or sliding, which is exactly what driving at the limit needs.
- **The camera is pre-wired but off.** Version 1 trains and races without it.
  - The network already has an input slot for a camera, and the observation format, model files and node already handle it.
  - Adding it later means building one camera node, then fine-tuning an existing policy. No change to the architecture, no retraining from scratch ([A5](#a5-the-camera-an-option-that-is-already-wired-in)).
- **About 630k parameters, measured at 3.4 ms per tick** on one of the Jetson's CPU cores, in plain NumPy. That is 14% of the 25 ms tick. No PyTorch, ONNX Runtime or GPU is needed on the car, which matters because [gpu-acceleration.md](../../gpu-acceleration.md) keeps the GPU out of the control loop and none of those libraries are installed.
- **Not a transformer.** A scan is a 1D signal whose structure is local, so a convolution fits it. Self-attention would cost CPU time and buy nothing at this size.
- **Opponents are learned from the scan.** Other cars show up in the LiDAR. A stack of three scans plus the GRU's memory gives the network their speed and habits. An auxiliary output trained to forecast each opponent's position forces it to actually find them.
- **Trained in a GPU simulator that we write.** It runs in PyTorch, with about 16k races at once on one H100. It must match the existing calibrated CPU simulator before any result from it counts.
- **Training regime: a league plus a track curriculum, not PSRO.**
  - **Opponents:** an AlphaStar-style league with prioritized fictitious self-play, exploiter agents, and a pool of scripted classical drivers. Every snapshot is kept, and a sampling floor means no agent's share ever reaches zero.
  - **Tracks:** prioritized level replay with mutation, over procedurally generated tracks. This is what makes the policy generalize.
  - **Choosing the release:** cross-play on **held-out tracks** against a fixed benchmark panel, ranked by Nash averaging, with a worst-case veto. Nash ideas only choose the release; they never drop anyone from training.
- **Deterministic safety wraps the network.** The LB deadman, input and output checks, and the existing LiDAR brake all run around it, and the network can't switch any of them off.
- **On the car, step by step.** It first runs in **shadow mode**: it computes commands, but only `gap_follow` drives. Then wheels off the ground, then the floor at a hard 1.0 m/s cap.

---

## A2. What "winning" means, and how we'll know

The goal is to **win races against other teams' cars**: finish ahead, without causing contact, on a track the car may have seen only once.

The training reward is not the success criterion. These are, measured on **held-out tracks that no training stage ever sees**:

| Metric | How it's measured | Gate to ship |
|---|---|---|
| Win rate | 2-, 3- and 4-car races against the benchmark panel (solo runs score lap time, below) | over 50% against every panel member, and over 65% against the classical stack |
| At-fault contact | the referee's own collision check, with fault assigned by geometry ([A8](#reward)) | **0** at-fault car contacts in 200 held-out races, and no wall contact |
| Solo lap time, map mode | vs the minimum-curvature line driven by `pure_pursuit` | faster on at least 80% of held-out tracks |
| Solo lap time, lap 1 (reactive) | vs `gap_follow` | at least 10% faster, with zero contacts |
| Safety-wrapper interventions | share of ticks where the wrapper overrode the network | under 1%. A policy that leans on the wrapper is lucky, not good |
| Generalization gap | win rate on training tracks minus held-out tracks | under 10 points; more means it memorised tracks |

The **benchmark panel** is fixed and versioned. Winning against the latest league members is not evidence; winning against a fixed panel is. Its members:

1. `gap_follow` with opponent awareness on
2. `pure_pursuit` with overtaking, on the optimized line
3. a scripted defensive blocker (built in Phase 9)
4. the previous shipped release, once there is one

> **TODO(verify) before Phase 8:** get the current F1TENTH/RoboRacer head-to-head rulebook (start procedure, contact and passing rules, number of cars). It sets the race format the simulator plays and how fault is assigned.

---

## A3. What the car can sense and do

These are car 1's real constraints. Every design choice below follows from them.

| | Value | Source |
|---|---|---|
| Control rate | 40 Hz, one tick per scan, so a 25 ms budget | [gpu-acceleration.md](../../gpu-acceleration.md) |
| LiDAR | Hokuyo UST-30LX over USB, about 40 Hz, 270°. Expected 1081 beams at 0.25°. **TODO(verify on car 1)** in Phase 1; `calibration.py` still cites car 2's UST-10LX | [hardware-reference.md](../../hardware-reference.md#lidar--hokuyo-ust-30lx) |
| LiDAR mount | 0.26 m ahead of the rear axle (`base_link`), 0.11 m up | assumed from car 2, not measured on car 1 |
| Wheelbase | 0.36 m | assumed from car 2 |
| Steering | autonomy capped at **±0.33 rad** | `gap_follow.yaml`, `pure_pursuit.yaml` |
| Speed | `pure_pursuit` ceiling 4.0 m/s today | `pure_pursuit.yaml` |
| Speed measurement | `/odom` ([odometry](../../glossary.md#odometry--odom)) from the [VESC](../../glossary.md#vesc) motor controller's eRPM, about 5% error | `calibration.py` |
| **IMUs** | **two** (team, 2026-10-23): the VESC's onboard IMU, read by `vesc_driver` at 50 Hz onto `/sensors/imu/raw`, and a **separate IMU at the front bumper**, roughly 0.45 m ahead of `base_link`. Positions measured in P0.4. **Neither is usable yet:** see below | `vesc_driver.cpp`, hardware-reference.md |
| Top speed | firmware cap 23250 eRPM = **6.37 m/s** at car 1's command gain of 3650 | `vesc.yaml` |
| Camera | UVC webcam on `/dev/video0`, 1280×720 at 30 fps. Opened directly by `usb_cam_stream`, which publishes no ROS image topic | `camera_stream_node.py` in the `web_dashboards` submodule |
| Inference compute | 6× Cortex-A78AE. The GPU is shared and stays out of the control loop | gpu-acceleration.md |
| ML runtime installed | TensorRT only. No PyTorch, ONNX Runtime or CuPy | gpu-acceleration.md |

**Two things stand between the IMU and this plan.** Phase 0 fixes both.

1. **It may be switched off.** On 2026-08-19 every field read exactly zero, with no gravity on any axis. That note lives in a doc ported from car 2, and car 1's IMU has never been checked. All zeros means either no IMU chip on the board, or the IMU disabled in the VESC firmware (VESC Tool → App Settings → IMU).
2. **It is published in the wrong units.** `vesc_driver.cpp` copies the VESC's raw values straight into a `sensor_msgs/Imu`. The packet parser's own comments give gyro in **deg/s** and acceleration in **g**. The ROS message standard (REP-145) says rad/s and m/s².

   Any node reading `/sensors/imu/raw` as SI would be off by about 57× on rotation and 9.8× on acceleration.

   The message is also stamped when it *arrives*, not when it was measured, so its delay is unknown.

**What the front-bumper mount means.** The bumper is about 0.27 m ahead of the car's centre of gravity, which sits about 0.18 m ahead of the rear axle. An accelerometer that far forward reads more than the car's own acceleration:

- **In a turn**, it reads an extra `−ω² · r` along the car. At 2 rad/s that is about 1.1 m/s², as large as moderate braking.
- **When the turn rate changes**, it reads an extra `ω̇ · r` sideways.
- **The gyro is unaffected**: yaw rate is the same everywhere on a rigid car.

This matters most if P0.5 picks the front unit as the primary. Either way, it is handled by **simulating the primary IMU exactly where it sits** ([A7](#what-it-simulates)), so the network learns from the same signal it gets on the car. Nothing needs correcting on either side, but it does mean the mount position must be measured, not guessed (P0.4).

The bumper is also the first part of the car to hit a wall. Two consequences:

- The mount must be rigid. A flexible mount adds vibration the simulator doesn't model.
- Readings during an impact saturate. The conditioner flags saturated samples, and the node treats them as missing (P0.3).

**Two IMUs: one feeds the network, the other checks it.**

- **The primary** feeds the network. It is chosen in P0.5 by measurement: lower delay first, then lower noise while driving. The simulator models the primary, at its mount point.
- **The secondary is a live cross-check.** Yaw rate is the same at every point of a rigid car, so the two gyros must agree, wherever each is mounted. If they disagree for more than 0.5 s, the IMU is reported faulted and the node commands zero. With only two sensors there's no majority to say which one is wrong, so stopping is the only safe answer.
- **No automatic failover in v1.** The two accelerometers sit at different points, so they read different signals in a turn, and the network was trained on the primary's. Swapping in the other mid-race would feed it a signal it never saw. A later version could train with both mount points and a mount-position input, and fail over safely.

**How the front IMU is connected decides whether the cross-check is possible.** On 2026-10-23 it didn't appear on the Jetson's USB, and no I²C sensor was registered with the kernel. So it is either:

- **wired to the VESC as its external IMU.** VESC firmware can read an external IMU on its own I²C pins, but it reports **one** IMU at a time, whichever VESC Tool selects. Then there's no live cross-check: choose the primary by measuring each in turn, and run single-IMU.
- **or wired to the Jetson** (its 40-pin header, or USB once powered), with no driver installed yet. Then both stream at once and the cross-check runs.

> **TODO(verify):** which of these, and the front IMU's model. P0.1 settles it.

---

## A4. The network

### Inputs: observation spec, version 1

Every input is something the car actually has, and every scale is a fixed constant, never running statistics. Fixed scales keep the car's NumPy copy exact and can't drift. All constants live in one module, `src/neural_racer/neural_racer/observation.py`, which both the car and the training code import ([B0](#one-observation-spec-two-implementations)).

**Scan block**, `scan`, float32, shape `(4, 270)`:

| Channel | Content |
|---|---|
| 0, 1, 2 | scans at ticks t, t−1, t−2, each as `log(clip(r, 0.05, 10) / 0.05) / log(200)`, which maps 5 cm to 10 m onto [0, 1] |
| 3 | validity of scan t: 1 where a bin had at least one valid return, else 0 |

- **Pooling.** Beams 0–1079 are grouped 4 to a bin, and each bin takes the **minimum** valid range in it. The minimum keeps the nearest return, so a thin chair leg or the corner of a car can't vanish into an average. Beam 1080 is dropped.
- **Invalid beams.** A beam is invalid if it is `NaN`, `inf`, at or below `range_min`, or at or above `range_max`. A bin with no valid beam reads 10 m with validity 0. "No echo" and "far away" are different things, and the network must be able to tell them apart.
- **Beam count.** Any count other than 1081 means a zero command on the car, and an assertion failure in training.

**Ego vector**, `ego`, float32, shape `(23,)`:

| Index | Content | Scale |
|---|---|---|
| 0–2 | measured speed at t, t−1, t−2 (`/odom`) | ÷ 5 m/s |
| 3–5 | steering command sent at t−1, t−2, t−3 | ÷ 0.33 rad |
| 6–8 | speed command sent at t−1, t−2, t−3 | ÷ 5 m/s |
| 9–11 | yaw rate ω_z at t, t−1, t−2 (conditioned IMU, base_link frame) | ÷ 3 rad/s |
| 12–14 | longitudinal acceleration a_x at t, t−1, t−2 | ÷ 9.81 m/s² |
| 15–17 | lateral acceleration a_y at t, t−1, t−2 | ÷ 9.81 m/s² |
| 18 | IMU present (1) or missing (0). When missing, 9–17 are 0 | — |
| 19 | mode: 0 = reactive, 1 = map | — |
| 20 | speed cap `v_cap` currently in force | ÷ 5 m/s |
| 21 | camera present (1) or missing (0). Always 0 in v1 | — |
| 22 | camera embedding age, clipped to [0, 1] | ÷ 0.2 s |

The IMU value for tick t is the newest conditioned sample at the scan's arrival time. The IMU runs at 50 Hz, so that sample is at most 20 ms old plus its measured delay.

**Track preview**, `preview`, float32, shape `(4, 32)`, map mode only, all zeros otherwise. Thirty-two points every 0.5 m along the next 16 m of the reference path, in the car's frame:

| Row | Content | Scale |
|---|---|---|
| 0, 1 | point x, y | ÷ 16 m |
| 2, 3 | free width to the left and right of that point, from the lap-1 map | ÷ 2 m |

**Race context**, `race`, float32, shape `(6,)`, map mode only, zeros otherwise: lap fraction; current place ÷ 4; time gap to the car ahead and behind along the track (÷ 10 s, clipped to 1); car-ahead present; car-behind present. It comes from the existing map-subtraction opponent detector.

**Camera embedding**, `camera`, float32, shape `(64,)`: zeros in v1. See [A5](#a5-the-camera-an-option-that-is-already-wired-in).

### Layers, version 1

```
scan (4,270) ─ Conv1d 4→32 k7 s2 p3 ─ SiLU ─ Conv1d 32→64 k5 s2 p2 ─ SiLU
             ─ Conv1d 64→64 k5 s2 p2 ─ SiLU ─ Conv1d 64→64 k3 s2 p1 ─ SiLU     lengths 270→135→68→34→17
             ─ flatten (1088) ─ Linear 1088→256 ─ SiLU ──────────────────────────────┐  z_scan  256
ego (23)     ─ Linear 23→64 ─ SiLU ─ Linear 64→64 ─ SiLU ──────────────────────────────┤  z_ego    64
preview      ─ Conv1d 4→32 k5 s2 p2 ─ SiLU ─ Conv1d 32→32 k3 s2 p1 ─ SiLU             │
  (4,32)     ─ flatten (256) ─ Linear 256→64 ─ SiLU ─ × mode ────────────────────────────┤  z_prev   64
race (6)     ─ Linear 6→32 ─ SiLU ─ × mode ──────────────────────────────────────────────┤  z_race   32
camera (64)  ─ × camera_present ─────────────────────────────────────────────────────────┤  z_cam    64
                                                                                        ▼
      LayerNorm(z_scan, z_ego, z_prev, z_race: 416) ─ concat z_cam (480) ─ Linear 480→256 ─ SiLU ─ GRUCell 256→128 ─ h
                                                                                        │
   h ─ Linear 128→128 ─ SiLU ─ Linear 128→2 ─┬─ steering = 0.33 · tanh(a₀)              actions
                                             └─ speed    = v_cap · sigmoid(a₁)
   h ─ Linear 128→12                            ego path: 6 points (x, y), every 0.25 s to 1.5 s     auxiliary
   h ─ Linear 128→52                            4 opponent slots × (present logit + 6 points)       auxiliary
```

- **The `× mode` gates make reactive mode exact.** Multiplying the preview and race embeddings by the mode flag *after* their layers makes them exactly zero in reactive mode, whatever the layers' biases are.
- **The `× camera_present` gate is what makes the camera an easy add.** In v1 the camera input is always zero, so the camera columns of the fusion layer (`Linear 480→256`) receive zero gradient and stay at their initial value.
  - Initialize those 64 columns to exactly zero, **and** the ego layer's weights for the `camera_present` and `camera_age` inputs.
  - Keep the camera **out of the LayerNorm**: inside it, a camera embedding would shift the normalization statistics even with zero weights.
  - Then enabling a camera later starts from *exactly* the same behaviour, and fine-tuning only adds to it (found while implementing; see [Implementation status](#implementation-status)).
- **Width is a setting.** The scan encoder's widths are a config value, `scan_width: full | half` (half is 16-32-32-32 channels and `Linear 544→128`). That is the fallback if the full-width latency gate fails ([measurement below](#size-and-measured-speed-on-the-jetson)).
- **The critic is training-only.** It is a separate network that sees privileged simulator state: true poses and speeds of every car, the track ahead, friction and delays. It is an MLP (privileged input → 512 → 512 → 1) with no GRU, since its input is already nearly complete.
  - This is an *asymmetric actor-critic*: the critic can judge what the actor can't see, which speeds learning, and the actor only ever gets what the car can sense. The critic never ships.

### Outputs

- **Steering angle and target speed**, exactly what `AckermannDriveStamped` carries. They are bounded *inside* the network by `tanh` and `sigmoid`, so no value can leave ±0.33 rad or [0, `v_cap`].
  - `v_cap` is a runtime parameter, not a trained constant. The car starts at 1.0 m/s.
  - Training randomizes `v_cap` and the network sees it (ego index 20), so it learns to race well at whatever cap it's given.
- **Ego path forecast**: where the network predicts the car will be over the next 1.5 s, in the car's frame. It is trained against what the car actually did next, so it is a forecast, not a promise. On the car it fills [`/drive_intent`](../../drive-intent.md) and gives the safety wrapper something to check against the map.
- **Opponent forecast**: up to 4 opponents, sorted by distance at time t, each with a presence logit and 6 future points. Trained against simulator ground truth. It forces the convolution to learn what a car looks like in a scan. On the car it is a diagnostic only, never a safety input.

### Size and measured speed on the Jetson

**Size:** **646,274 parameters** (2.6 MB as float32), counted layer by layer in `test_parameter_count_full_width`, and about 2.3M multiply-adds per tick. The 628,930 in the latency table below was measured before the 23-wide ego vector and the camera slot.

**Measured 2026-10-23 on car 1's Jetson**, with the v1 architecture above (fusion width 416, before the camera slot, which adds about 16k multiply-adds). Plain NumPy 1.26.4, random weights, 4,000 ticks pinned to one core (`taskset -c 5`), `OPENBLAS_NUM_THREADS=1`:

| Variant | Median | p99 | Max | Share of a 25 ms tick |
|---|---|---|---|---|
| Full width, first implementation | 3.81 ms | 3.91 ms | 4.09 ms | 16% |
| Full width, im2col with precomputed views | **3.40 ms** | **3.50 ms** | 3.63 ms | 14% |
| Half-width scan encoder (about 354k parameters) | 2.17 ms | 2.23 ms | 2.33 ms | 9% |

- The scan encoder is 71% of the full-width time.
- Most of the remaining cost is NumPy's fixed overhead per call on small arrays, not arithmetic. That is why halving the channels saves only a third.
- The optimized version matches the first implementation to `atol=1e-5`.
- Phase 6 repeats this measurement on the real `policy_numpy.py` with the whole race stack running. That number, not this one, is the gate.

### Why this shape, and not a transformer, a camera-first design, or a raceline output

<details>
<summary><b>The alternatives considered, and why each lost</b> — read this before proposing a change to the architecture.</summary>

**Feed-forward on one scan (an MLP or CNN, no memory).**
This is the simplest option, but one scan has no speed in it. The car can't tell whether the opponent ahead is pulling away or braking. Frame stacking covers the shortest horizon (75 ms). The GRU covers seconds: "this car brakes early into this corner," "I'm sliding."

The IMU helps a lot here, since it measures yaw rate and lateral acceleration directly. It doesn't remove the need for memory of opponents. The UZH league work ships both a recurrent and an `_mlp` variant of the same policy; Phase 8 runs the same ablation, and the GRU has to earn its place.

**Transformer over the scan.**
Attention over 270 bins buys nothing a convolution doesn't already capture, since a scan's structure is local and the same at every angle. It costs O(n²) on a CPU with a 25 ms budget. At this size, transformers also lose their main advantage, which is scaling.

Revisit only if a later version adds a *set* input, such as a list of tracked opponents. A single attention-pooling layer, with the car's own state as the query, would fit that, because it doesn't care about the order or number of opponents.

**Camera-first, or camera from v1.**
The webcam runs at 30 fps against a 40 Hz tick. A realistic camera image is expensive to render in a batched simulator and transfers poorly to the real car. And [gpu-acceleration.md](../../gpu-acceleration.md) already rules the camera out as a safety sensor. LiDAR has to be authoritative anyway, so v1 is LiDAR plus IMU, and the camera comes as an add-on through the pre-wired slot ([A5](#a5-the-camera-an-option-that-is-already-wired-in)).

**Network outputs a racing line, a classical controller follows it.**
This is safer to inspect, and it reuses `pure_pursuit`'s safety layers. But it splits one decision in two: overtaking at the limit couples steering and speed in a way a lookahead-point follower blunts. It is also unavailable on lap 1, which has no map. It is kept as **plan B** ([Risks](#risks-and-plan-b)).

**Raw motor and servo commands.**
No. Steering and speed in physical units keep `vesc.yaml`'s calibration outside the network. A recalibrated servo must not need a retrained policy.

**A separate network that "finds the best path".**
The minimum-curvature optimizer (`raceline_optimizer.py`) already solves the geometric problem well, and a network imitating an optimizer brings no guarantee. What an optimizer can't do is find the line that's fastest *for this car's real, laggy, sliding dynamics* and around *this opponent*. A policy trained for lap time and wins finds that implicitly. The optimizer line stays as the baseline it must beat.

</details>

---

## A5. The camera: an option that is already wired in

**Version 1 never uses the camera.** But four pieces are built in v1 so that adding it later is one new node and a fine-tune, not a redesign:

| Pre-wired in v1 | What it buys later |
|---|---|
| A 64-wide camera slot in the fusion layer, gated by `camera_present`, with its weights initialized to zero | adding the camera starts from exactly the v1 policy's behaviour; no architecture change, no retraining from scratch |
| `camera_present` and `camera_age` in the ego vector, always 0 in v1 | the policy is already used to running without a camera, so a camera dropout mid-race is a case it knows |
| A `modalities` list in every model card, and a node that refuses a model needing a modality that isn't configured | a camera model can't be launched on a car without the camera node by mistake |
| A camera hook in the GPU simulator (`gpu_sim/camera.py`) that returns zeros in v1 | the Phase C renderer drops in behind a fixed interface |

### How it will work, once added

**The camera never feeds pixels to the control loop.** A separate `camera_encoder_node` does all camera work, asynchronously, at the camera's own rate:

1. **Grab** a frame from a V4L2 publisher on `/camera/image_raw`. `usb_cam_stream` is restarted with `image_topic:=/camera/image_raw`, so the dashboard keeps its stream from the same frames. Only one process can hold `/dev/video0`.
2. **Segment** the image into floor, track boundary (tape or wall base), car, and other, with a small segmentation network on the Jetson's GPU through TensorRT. This is the "tensor cores for the camera" work [gpu-acceleration.md](../../gpu-acceleration.md) describes, and it stays off the control path.
3. **Project** the segmentation onto the floor plane (a fixed homography from the camera's measured mount pose) as a bird's-eye grid: 3 channels × 32 × 64 cells at 5 cm, covering 3.2 m ahead by 1.6 m either side.
4. **Encode** that grid with the policy's own camera encoder. These are layers exported from the same checkpoint: `Conv2d 3→16 k5 s2`, `16→32 k3 s2`, `32→32 k3 s2`, flatten, then `Linear →64`.
5. **Publish** the 64-value embedding with its timestamp and the model hash, on `/neural_racer/camera_embedding`.

`neural_racer_node` uses the newest embedding. If the embedding is older than 0.2 s, comes from a different model hash, or never arrived, the node sets `camera_present = 0` and drives on LiDAR and IMU, the same way it was trained to.

**Why a bird's-eye grid and not raw pixels:** the simulator can render a bird's-eye grid exactly and cheaply from the same track geometry it already has.

That leaves only one sim-to-real gap, the segmentation network's quality, and that can be trained on real images auto-labelled by the LiDAR. Raw pixels would need photorealistic rendering, which is a research project of its own.

**What a camera could add that LiDAR can't:** the LiDAR sees one horizontal plane 0.11 m above the floor. The camera sees track tape and floor markings, obstacles below or above that plane, and an opponent's colour and shape, which helps tell cars apart. Whether that wins races is measured by the Phase C ablation, not assumed.

---

## A6. Safety: what wraps the network

**The network is never the last word on the car.** It runs inside a deterministic wrapper in `neural_racer_node`. Every tick, in this order, each step can only make the command safer:

1. **LB deadman.** This is the same pattern as `gap_follow_node.py`'s `joy_callback` and `_deadman_status`, with `deadman_button: 4`, `joy_timeout_sec: 0.5` and `enable_deadman: true` in shipped config. LB released, `/joy` never received, or `/joy` stale means a zero command and a GRU reset, whatever the network says.
   - **Why:** the network has no idea a person is standing in front of the car. The person holding LB does.
2. **Input checks.** Each of these means a zero command, a GRU reset, and a logged reason:
   - a scan older than 75 ms, or a beam count other than 1081
   - more than 30% invalid beams
   - `/odom` older than 0.1 s
   - with `require_imu: true` (the default), a conditioned IMU sample older than 0.1 s, or `/imu/status` not `ok`. That covers a failed startup check, a saturated or missing sample, and the two IMUs disagreeing
   - in map mode, a pose older than 0.2 s
3. **Output checks.** A `NaN` or `inf` from the network means a zero command and a logged fault.
4. **Clip** steering to ±`max_steering_angle` (0.33) and speed to [0, `max_speed`]. This is redundant with the network's own bounds, on purpose.
5. **LiDAR brake.** These are `gap_follow`'s ROS-free functions, imported from `gap_logic`, never copied, with `gap_follow.yaml`'s values:
   - `boundary = vehicle_boundary_distances(angles, car_width=0.31, car_length=0.58, wheelbase=0.36, laser_offset_x=0.26)`
   - `clearance = minimum_footprint_clearance(clean, valid, boundary)`
   - `speed ≤ braking_speed_limit(clearance, reserve_distance=0.20, max_braking_decel=3.0, max_speed, latency=0.25)`
6. **Curvature cap.** `speed ≤ sqrt(max_lateral_accel / |κ|)` with `κ = tan(δ) / wheelbase` and `max_lateral_accel: 2.0`, as in `gap_follow`.
   - **Dated note:** `gap_follow` moved to 2.5 (decided 2026-10-26, committed 2026-10-27 as `9fa9642`). neural_racer deliberately keeps 2.0 (training cap unchanged).
   - This is a **hard cap on lap time**, so the network can't use more lateral grip than `gap_follow` assumes. That's correct for v1. Raising it is a measured decision for after Phase 10, never a training-time tweak.
7. **Steering rate.** `|Δδ| ≤ max_steering_rate · dt`, with `max_steering_rate: 2.0` rad/s.
8. **Map mode only:** if the ego-path forecast crosses an occupied cell of the frozen lap-1 map, the speed cap for that tick falls to `braking_speed_limit` on the distance to that crossing, and the event is logged.
9. **Publish**, then publish `/drive_intent` after the drive command, inside one try/except that disables the diagnostic, not the node ([drive-intent.md](../../drive-intent.md#safety-contract-for-publishers-read-this-first)).

### How fast the car is allowed to go

Four limits stack. The lowest one wins every tick:

| Limit | Value | Who sets it, and when it changes |
|---|---|---|
| Firmware | 6.37 m/s (23250 eRPM in `vesc.yaml`) | fixed by the VESC configuration |
| `max_speed` (the network's `v_cap`) | **1.0 m/s at first**, then +0.5 m/s per clean run, to at most 6.37 | Phase 10's ladder. Training randomizes `v_cap` from 1.0 to 6.37 so the policy is good at every step of the ladder |
| Braking cap (step 5) | the fastest speed that can stop within the clearance the LiDAR sees | physics, every tick; never raised |
| Cornering cap (step 6) | `sqrt(2.0 / |κ|)`: 1.6 m/s in the tightest legal corner | `max_lateral_accel: 2.0`, `gap_follow`'s value when this spec was written. **Dated note:** `gap_follow` moved to 2.5 (decided 2026-10-26); neural_racer deliberately keeps 2.0 (training cap unchanged). Raising it is a measured decision after Phase 10, never a training-time change |

Training uses the same cornering cap as the car, so the policy never learns to rely on grip the car isn't allowed to use.

The simulator's friction is randomized from 0.4 to 1.1, and 0.55 is the measured value. At that value the tyres can give about 5.4 m/s². So the 2.0 m/s² cap leaves room to raise later, once real runs show the car holding the line at the current cap.

**Training runs the same wrapper.** The GPU simulator has a batched copy of steps 4–7, tested for parity against the NumPy version. Every tick it overrides the network costs a penalty, so the policy learns not to need it. The under-1% metric in [A2](#a2-what-winning-means-and-how-well-know) is the check.

> **A note on opponents behind you.** The brake will stop the car hard if something appears in front of it, even with a rival right behind. Under most rules that rear-end collision isn't our fault, but it still costs the race and maybe a car. That trade is accepted on purpose: safety beats position.
>
> The fix is a policy that rarely triggers the brake, never a weaker brake.

---

## A7. The training simulator

### Why we need a new one

Reinforcement learning for racing is limited by **simulation speed, not the network**. Most of the GPU's work is simulating cars and casting LiDAR rays. The existing F1TENTH Gym (`tools/f1tenth_sim/`) runs one car per process on the CPU through numba, a few thousand steps per second per core. A league run needs billions of steps.

So training uses a **batched simulator written in PyTorch**. Every tensor has a leading dimension of about 16,384 environments, and nothing leaves the GPU during a rollout.

**The existing simulators are not replaced. They become the referee:**

- `tools/f1tenth_sim/` with `sim_fidelity/` is calibrated to the car, with a provenance tag on every parameter, and has its own geometric collision check. It judges every candidate (Phase 9).
- `racerbot_sim` runs the whole ROS node, so wiring bugs show up (Phases 6 and 11).

### What it simulates

- **Vehicle:** the same single-track model, parameters and friction circle as `sim_fidelity` (`calibration.py`, `grip.py`), integrated with RK4 at 5 ms, 5 substeps per 25 ms tick, matching gym.
- **Actuation:** the steering servo (`steering_servo_gain` 15/s, `steering_rate_max`), the fitted speed loop (`speed_loop_*`, run 20261019-200703), and per-environment command delays held in a GPU ring buffer (steering 3 ticks, speed 2 ticks by default).
- **LiDAR:** 1081 beams by sphere tracing on a signed-distance field of the track (5 cm grid, read with `grid_sample`, 24 iterations). Other cars are added by exact ray-versus-rectangle intersection. Then noise, dropout and the one-tick delay, then the same pooling as [A4](#inputs-observation-spec-version-1).
- **IMU:** yaw rate and specific force **at the IMU's measured mount point**, the front bumper, including the `−ω²r` and `ω̇r` terms.
  - It is computed from the vehicle state at every 5 ms substep and sampled at 50 Hz. Then Phase 0's measured delay, white noise, a slowly wandering gyro bias, accelerometer bias and scale error. Modelling the sensor where it really sits means neither side needs a lever-arm correction.
- **Cars:** 1 to 4 per race, so a solo car is a time trial. Opponent footprint, speed and behaviour are randomized, because other teams' cars are not ours.
- **Collision:** chassis versus the distance field, and chassis versus chassis by the separating-axis test, as in `plant.py`. Gym's own collision flag is not used anywhere (see `CLAUDE.md`).
- **Camera hook:** `camera.py` returns zeros in v1. Phase C adds a bird's-eye renderer behind the same interface.
- **Track pool:** about 4,096 tracks resident on the GPU as fp16 distance fields. At 800×800 cells × 2 bytes each that is about 5 GB of the H100's 80 GB.

### Parity: the GPU simulator has to earn trust

**No training result counts until the GPU simulator matches `sim_fidelity` on the same inputs.** The exact tests are in [Phase 3](#phase-3-gpu-simulator-and-parity).

### Randomization, keyed to how well we know each number

`calibration.py` tags every parameter MEASURED, DERIVED, SPEC, ESTIMATED or STOCK. That tag sets each parameter's per-episode randomization range:

| Provenance | Range | Examples |
|---|---|---|
| MEASURED | ±5% | speed-loop gains, odom gain, IMU noise and delay after Phase 0 |
| DERIVED / SPEC | ±10% | LiDAR noise, beam spacing |
| ESTIMATED | ±30%, or a discrete set | command delay 1–3 ticks, pose lag, LiDAR dropout |
| STOCK (never measured) | wide | tyre stiffness, centre-of-gravity height, friction 0.4–1.1 |

Also randomized:

- steering gain error ±10% (car 1's gain was 30% off until 2026-10-21)
- LiDAR mount ±2 cm
- floor friction varying along the track
- IMU mount rotation ±2° (the residual after Phase 0's axis calibration)
- episodes with the IMU missing (5%) or frozen for a few ticks (1% of ticks)

---

## A8. The training regime

### Why not PSRO

PSRO grows a population of policies. Each round it solves the meta-game (who beats whom) for a Nash mixture, then trains a new best response to that mixture. Three things go wrong for this project:

1. **It drops good agents.** A Nash mixture is usually supported on very few policies. A policy that's excellent against 90% of opponents but loses to one gets probability zero, and stops being trained against. That's the "accidentally discards good candidates" failure.
2. **It hides track weaknesses.** The payoff matrix averages over tracks. An agent that wins on fast tracks and crashes on tight ones looks average, not broken.
3. **Its guarantees don't apply, and it's costly.** Its convergence guarantee is for two-player zero-sum games, and a 4-car race where both cars can crash is neither. Every round also needs a full N×N cross-play matrix, which spends compute evaluating, not learning.

Nash ideas are still useful for **ranking** at selection time, not for choosing who gets trained against.

### What we do instead

Three published methods, each fixing one of the problems above:

| Problem | Method | What it does here |
|---|---|---|
| Beat many styles, never forget one | **League with prioritized fictitious self-play (PFSP)** (AlphaStar) | opponents drawn from *all* past snapshots, weighted toward those we lose to, with a floor so no one reaches zero |
| Find our own blind spots | **Exploiter agents** (AlphaStar) | agents trained only to beat the main agents, reset when they succeed |
| Generalize to every track | **Prioritized level replay with mutation** (robust PLR, ACCEL) | replays the tracks the agent learns most from, mutates them into new ones, and never sees the held-out set |

### Stage 1: privileged solo teacher (Phase 5)

A single car, no opponents. The actor sees privileged state, so it learns raw car control at the limit fast. That is the hard part, and learning it from LiDAR and tactics at the same time is slow and unstable.

- **Teacher input:** 64 points of the track ahead (x, y, left width, right width in the car's frame, every 0.25 m), true speed, slip angle, yaw rate, actual steering angle, friction, and delays as a one-hot.
- **Teacher network:** MLP 512 → 512 → 256 → 2, using the same action squashing as the student.
- **Tracks:** from the PLR curriculum (Phase 4).

### Stage 2: distil into the deployable student (Phase 6)

The student is the network in [A4](#a4-the-network), with real inputs only. It learns from the teacher by **DAgger**: the student drives, and the teacher labels what it would have done in every state the student reaches. This is supervised learning on huge batches, the most GPU-efficient stage of the project.

- Opponents in this stage are scripted only. Episodes use the three types in the [league section](#stage-3-the-league-phase-8): reactive, map and handover.
- The IMU noise model uses **Phase 0's measured values**. This is why Phase 0 gates this stage.

### Stage 3: the league (Phase 8)

Every league member is a deployable-input student with a privileged critic, so opponents in training sense the world as real cars do. Each role is its own learner process on its own GPU:

| Role | Count | Trains against | Reset |
|---|---|---|---|
| Main agent `main0` | 1 | 35% current self, 40% PFSP over the whole archive, 15% scripted pool, 10% uniform over the whole archive | never |
| Main agent `main1_mlp` | 1 | same mix; MLP core instead of the GRU, for the ablation | never |
| Main exploiter | 1 | the current main agents only | to the Stage 2 student, when its win rate passes 70% or after 2×10⁹ steps |
| League exploiter | 1 | PFSP over the whole archive, using `p(1 − p)` | to the Stage 2 student, with 25% chance after each snapshot |

- **PFSP weighting:** an archive member is sampled in proportion to `(1 − p)²`, where `p` is our estimated win rate against it. Win rates are `(wins + 1) / (games + 2)` from training matches, scored pairwise in multi-car races.
- **The floor guarantees nothing is discarded.** 10% of matches are uniform over the entire archive, so every saved agent keeps at least `0.10 / N` of the matches, however well we already do against it.
- **Beaten baselines fade, but never vanish.** The scripted pool's share starts at 15%, and shrinks as our win rate against it rises: `share = max(0.03, 0.15 · (1 − p_scripted) / 0.5)`, capped at 0.15, where `p_scripted` is the win rate over the last 10,000 scripted races.
  - At a 50% win rate it is the full 15%; at 90%, 3%.
  - The freed share goes to PFSP over the archive, where the hard opponents are.
  - The 3% floor stays: a league that forgets how to beat a simple, steady driver is a known failure, and it's the cheapest guard against polite collusion.
  - Within the pool, each scripted driver is weighted by PFSP too, so the blocker we still lose to gets more races than the parked car we always pass.
- **Archive:** a snapshot every 200 updates from every role, written to `$SCRATCH`, **never deleted**.
- **Scripted pool**, with randomized parameters, so the league can't collude into a polite equilibrium where nobody overtakes:
  - pure pursuit on random lateral offsets of the optimized line, at 50–110% of our pace
  - a `gap_follow` clone
  - a defensive blocker that holds the inside before corners
  - an erratic driver
  - a parked car
- **Levels for PLR:** a level is a track, plus a start grid, a car count and a physics sample.
  - Score: positive value loss, the robust-PLR measure of what's left to learn.
  - 70% of environments replay high-scoring levels and are trained on; 30% are fresh levels, scored but not trained on (robust PLR's rule).
  - ACCEL mutations perturb replayed tracks' control points and widths.
- **Race format (team decision, 2026-10-23):** grid start, 1.0 m stagger along the lap (0.6 m, the first choice, left the 0.535 m bodies overlapping on bends), with the grid order randomized. Car count is drawn uniformly from 1 to 4, so a quarter of races are solo time trials. Three laps or 90 s, whichever comes first.
- **Three episode types**, so the policy practises the real sequence and not just its parts:

  | Type | Share | What happens |
  |---|---|---|
  | Reactive | 30% | the whole race in reactive mode: no map, ever |
  | Map | 30% | map mode from the start, with a reference path from a simulated lap 1 |
  | **Handover** | 40% | lap 1 reactive; at the line, switch to map mode, using the path this car actually drove on lap 1 as the reference, exactly as `auto_map_race` does |

  - **The simulated lap-1 map is deliberately imperfect.** The reference path gets SLAM-like noise: a slowly drifting offset of up to 5 cm, and a heading error of up to 1°. The widths are measured on a copy of the walls with 10% of cells randomly dropped, and a seam where the lap closes. The policy has to cope with the map the car will really have.
- **Many opponents, one batched pass.** Each rollout uses 16 opponent policies. Their weights are stacked (`torch.func.stack_module_state`) and run in one `vmap` call, so 16 different opponents cost about one batched forward pass, not 16. They are resampled every update.

### Hyperparameters, initial values

| | Stage 1 teacher (PPO) | Stage 2 student (DAgger) | Stage 3 league (PPO) |
|---|---|---|---|
| Environments | 16,384 (set by `benchmark`) | 16,384 | 16,384 |
| Rollout length | 32 ticks (0.8 s) | 64-tick sequences | 32 ticks |
| Discount γ / GAE λ | 0.995 / 0.95 | — | 0.995 / 0.95 |
| Epochs × minibatches | 4 × 4 | 1 pass per 8 rollouts | 4 × 4 |
| Clip / value coefficient | 0.2 / 0.5 | — | 0.2 / 0.5 |
| Learning rate | 3×10⁻⁴, linear decay | 1×10⁻³, cosine | 1×10⁻⁴, constant |
| Initial log-std | −1.0, state-independent | — | −1.5 |
| Grad-norm clip | 1.0 | 1.0 | 1.0 |
| GRU | — | BPTT over the 64-tick sequence | BPTT over the rollout, hidden state stored at its start |
| Teacher-drives probability β | — | 0.5ⁱ at iteration i | — |
| Aux loss weights | — | ego path 0.5, opponents 0.5 | same, kept on |
| Budget | 2×10⁹ steps × 3 seeds | 3×10⁸ steps | 2×10¹⁰ steps total |
| Precision | bf16 network, fp32 physics | same | same |

These are starting values. The reward weights and the league learning rate are swept in Phase 8, and judged on the held-out selection metrics, never on reward.

### Reward

Per car, per tick, in initial values:

```
r =   0.1 · Δs / (v_ref · dt)          progress along the centreline; v_ref = 4 m/s
    + 2.0 · (places gained)            and −2.0 per place lost (0.3 m hysteresis)
    − 20  · at-fault contact           ends this car's episode
    −  2  · not-at-fault contact       still bad, still usually avoidable
    − 20  · wall contact               ends this car's episode
    − 0.5 · wrapper intervened         the policy must not lean on the safety wrapper
    − 0.01 · (Δδ / 0.33)²              steering smoothness
    + 10 · (N − place) / (N − 1)       at race end, N cars
```

**Fault:** in a car-to-car contact, car A is at fault when its closing speed toward B, `max(0, v_A · n_AB)` with `n_AB` the unit vector from A to B, is larger than B's toward A. A rear-ender is therefore at fault, and so is a car that turns into another. Replace this with the rulebook's definition when it's verified ([A2](#a2-what-winning-means-and-how-well-know)).

### Choosing the release: never "the latest checkpoint"

Training produces hundreds of snapshots. The one that ships is chosen by an evaluation nothing in training can see (Phase 9):

1. **Candidates:** the last 5 snapshots of each main agent, plus the top 10 of the archive by training win rate.
2. **Held-out tracks:** a fixed set that no stage ever trains on, PLR included. It holds the real course maps (`room_2026_07_27`, `levine_blocked`, `levine_obs`, `circular_obs`) and 20 procedural tracks from a frozen seed.
3. **Cross-play** among the candidates and against the panel, in 2-, 3- and 4-car races, on the referee (CPU, `sim_fidelity`).
4. **Rank by Nash averaging** (Balduzzi et al., 2018): score each candidate against the maximum-entropy Nash mixture of everyone. Fifty near-identical weak snapshots can't inflate a score this way. This is game theory used for ranking, not training.
5. **Worst-case veto:** a candidate under 50% against *any* panel member, or with any at-fault contact, is out, whatever its rank.

**Not wasting races on settled matchups.** Every matchup is played as a **sequential test**: races are added in batches of 10, and the matchup stops as soon as the 95% confidence interval on the win rate (Wilson) sits wholly above or below the threshold it's being tested against. Two consequences:

- A decisive matchup is settled in about 20 races. Only close ones need the full 200.
- **During training**, a panel member a role has beaten above 95% for 10 consecutive snapshots is re-checked only every 10th snapshot after that. A failed re-check puts it straight back to every snapshot.

At release, every panel member is always played, sequentially. These runs are CPU-only, so they cost no GPU time. The zero-contact rule counts contacts over **all** of a candidate's held-out races, panel and league alike, and needs at least 200 races in total.

---

## A9. FIR and GPU efficiency

**Fir's GPU nodes:** 4× NVIDIA H100 SXM5 80 GB, 48 cores and 1 TB RAM each, so one GPU's share is 12 cores and about 250 GB (Alliance wiki, retrieved 2026-10-23).

**The team has a default (`def-`) allocation** (2026-10-23). That means no reserved share of the cluster: jobs start when fair-share priority and free nodes allow. So plan for waiting, not for four GPUs at once:

- **Expect roles to run one or two at a time.** The league submit script chains each role independently, so roles simply progress at different rates when the queue is busy. Nothing breaks.
- **If 12-hour jobs wait long, try shorter segments** (`--time 3:00:00`). Short jobs fit into gaps between larger ones, and the checkpoint chain makes a short segment cost only its startup, a couple of minutes.
- **Cut league breadth before depth.** If throughput stays low, drop `main1_mlp` first (the ablation can wait), then merge the two exploiters into one.
- **Keep a record of what the league actually needed.** Steps, GPU-hours and queue waits support a resource allocation (RAC) application for the next cycle, if one turns out to be needed.

> **TODO(verify):** the wiki still listed access details as "to be determined" when this was written. Check the current maximum wall time, the GPU request syntax and the Python module before the first submission. The scripts assume `--gpus-per-node=h100:1` and 12-hour segments.

GPU efficiency is lost in three places, and each has a fix built in:

| Where time is lost | Fix |
|---|---|
| **The GPU waits on Python**: small kernels, a CPU round-trip per step, `.item()` in the loop | simulation, LiDAR, reward, buffer and advantages are all GPU tensors; no host sync inside a rollout; metrics copied out every N updates; sim step and policy forward compiled with `torch.compile` and replayed as CUDA graphs |
| **Batches too small for an H100** | 16k environments by default, set per GPU by the `benchmark` stage, which doubles the count until throughput gains under 5% or memory reaches 70% |
| **The GPU idles during evaluation** | evaluation never runs on the GPU. The referee is CPU-only, so it runs on the job's spare cores while the GPU trains |

**The same layout fixes CPU efficiency**, the other number the Alliance reports. A GPU-resident trainer keeps about one core busy, so 11 idle cores would read about 8%. Of the 12 cores, the job uses:

- 1–2 for the trainer
- **3 track-generator workers** feeding fresh tracks into the GPU pool (Phase 4)
- **7 referee workers** scoring archive snapshots on held-out tracks

**Other choices:**

- **One learner per GPU, not data-parallel.** The network is too small for gradient synchronisation to pay. Four GPUs are four league roles, each its own 1-GPU job, and smaller requests start sooner.
- **Resumable 12-hour segments.** Slurm sends `SIGUSR1` 10 minutes before the limit. The trainer checkpoints and exits with code 85, and the next chained segment resumes.
- **A crash stops the chain.** A crashed trainer writes `FAILED`, so later segments don't repeat the crash on your allocation.

**Proof, every job:** `nvidia-smi` is logged every 30 s, and the job prints its mean GPU utilisation at exit, flagging anything under 80%. Afterwards, `seff <jobid>` gives CPU and memory efficiency. The scripts and their trainer contract are in [`tools/neural_racer/slurm/`](../../../tools/neural_racer/slurm/README.md).

**Budget, as estimates to replace with Phase 3's measurement:**

| Stage | Steps | At an assumed 500k steps/s per H100 | GPUs |
|---|---|---|---|
| Stage 1 teacher | 2×10⁹ per seed | about 1.1 h | 3 seeds in parallel |
| Stage 2 student | 3×10⁸ | under 1 h | 1 |
| Stage 3 league | 2×10¹⁰ in total | about 11 h per role | 4, over several days of wall time |
| Selection | CPU only | — | CPU nodes |

If the measured throughput is 10× lower, the league budget is the first thing to cut.

---

# Part B: implementation

## B0. Conventions every phase uses

### Frames, signs and units

- **Frames:** REP-103. x forward, y left, z up. Yaw is positive counter-clockwise seen from above. Steering is positive left.
- **Pose:** `base_link` is the rear axle. The particle filter's pose is the LiDAR's and is converted on arrival (see `docs/localization.md`). Never convert twice.
- **Units:** SI everywhere in new code. The *only* non-SI values anywhere are the raw VESC IMU message, converted once, in `imu_conditioner_node` (Phase 0).
- **Tick:** one LiDAR scan, nominally 25 ms. The node runs on scan arrival, as `gap_follow` does.

### Where the code goes

```
src/racerbot_imu/                       # Phase 0: ament_python, IMU conditioning (support code, not driving)
src/neural_racer/                       # Phases 2, 6: ament_python, the on-car policy (driving code)
├── neural_racer/
│   ├── observation.py                  # ROS-free: the observation spec v1, NumPy reference builder
│   ├── policy_numpy.py                 # ROS-free: forward pass, NumPy only
│   ├── safety_wrapper.py               # ROS-free: the A6 steps 2-8; imports gap_logic
│   ├── model_store.py                  # ROS-free: load, verify hash, check manifest
│   ├── preview.py                      # ROS-free: reference path + frozen map -> preview block
│   ├── neural_racer_node.py            # rclpy: deadman, topics, shadow mode, intent
│   └── approve_model.py                # CLI: add a model to the manifest after checking its reports
├── config/neural_racer.yaml
├── launch/neural_racer_launch.py
└── test/
tools/neural_racer/                     # Phases 3-9: training, runs on FIR or a CUDA workstation, never colcon-built
├── slurm/                              # written; see its README
├── requirements-fir.txt
├── heldout.yaml                        # Phase 1: the held-out track list and hashes
├── neural_racer_train/
│   ├── gpu_sim/  dynamics.py lidar.py imu.py actuation.py contact.py camera.py
│   │             observation_torch.py safety_torch.py race.py sim.py
│   ├── tracks.py  track_pool.py  plr.py
│   ├── model.py  critic.py  teacher_model.py  ppo.py
│   ├── teacher.py  distill.py  league_learner.py  benchmark.py      # Slurm stages
│   ├── archive.py  matchmaking.py  opponents.py  scripted.py
│   ├── eval_worker.py  select.py  export.py
└── tests/
```

`neural_racer` is a **driving package**: it publishes `/drive`. Everything in `CLAUDE.md`'s safety model applies to it. `racerbot_imu` and everything under `tools/` publish nothing that moves the car.

### One observation spec, two implementations

- **The constants live only in `observation.py`:** pooling, clip ranges, scales, index layout, `OBS_SPEC_VERSION = 1`.
- **The car** uses its NumPy `build_observation`, one tick at a time.
- **Training** uses `gpu_sim/observation_torch.py`, a batched PyTorch version that imports every constant from `observation.py`.
- **A parity test** (Phase 3) holds the two together on random and recorded inputs. Duplicated preprocessing is a classic silent sim-to-real bug, and this is the guard against it.
- The same pattern holds the safety wrapper (`safety_wrapper.py` and `safety_torch.py`) and the network (`policy_numpy.py` and `model.py`) together.

### Test runners

| Tests | Command | Where they run |
|---|---|---|
| `src/neural_racer/test/` and `src/racerbot_imu/test/`, ROS-free files | `python3 -m pytest src/<pkg>/test/ -v` | the Jetson, unsourced |
| node tests in the same folders | sourced, with `export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` | the Jetson. Never on the car's graph |
| `tools/neural_racer/tests/` | `python3 -m pytest tools/neural_racer/tests/ -v` | anywhere with PyTorch. GPU-only tests are marked `@pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA parity: run fir_train.sbatch tests on FIR")` and run there |
| `tools/f1tenth_sim/tests/` additions | as today | the Jetson or FIR |

Every new test file states its runner in its docstring (anti-pattern A9). After writing a test file, compare the collected count against `grep -c '^def test'` for that file.

### Model artifact

A model is a directory, `~/.ros/neural_racer/models/<hash>/`, holding:

| File | Content |
|---|---|
| `weights.npz` | float32 arrays named exactly as `policy_numpy.py` expects |
| `model_card.yaml` | `obs_spec_version`, `scan_width`, `modalities` (for example `[scan, ego, imu, preview, race]`), parameter count, training commit, run name, snapshot id |
| `golden.npz` | 1,000 recorded inputs (with GRU state) and the PyTorch model's outputs for them, written by `export.py` |
| `selection_report.json`, `referee_report.json` | Phase 9's results |

`<hash>` is the first 16 hex digits of the SHA-256 of `weights.npz`.

### Getting a model from FIR to the car

**On a Fir login node**, from the workspace. `export.py` needs PyTorch, so it runs on FIR, where the training venv is:

```bash
python -m neural_racer_train.export "$SCRATCH/neural_racer/runs/<run>/<role>/checkpoints/<id>" \
    "$SCRATCH/neural_racer/exports/"
```

**Working when:** it prints the new model's hash, and `$SCRATCH/neural_racer/exports/<hash>/` holds `weights.npz`, `model_card.yaml` and `golden.npz`. Phase 9 adds the two reports to the same folder.

**On your laptop**, copy it across. Fir's login needs multi-factor authentication, so this is done by a person, never by a script on the car:

```bash
rsync -av <you>@<fir login host>:scratch/neural_racer/exports/<hash> .   # ~/scratch links to $SCRATCH on Alliance clusters; TODO(verify) on Fir
rsync -av <hash> racerbotcar-1@<car>:~/.ros/neural_racer/incoming/
```

**On the car:**

```bash
ros2 run neural_racer approve_model ~/.ros/neural_racer/incoming/<hash>
```

**Working when:** it reports the golden check passed and the hash was added to the manifest. It refuses, and says why, if any of these fail:

- the hash doesn't match the weights
- the observation spec version doesn't match the car's code
- a report is missing or failing
- `PolicyNumpy` doesn't reproduce `golden.npz` to `atol=1e-5`

The weights are float32. Training uses bf16 only inside autocast, and keeps float32 master weights. So the exported network is exactly the one that was evaluated, and the golden check proves the car runs the same numbers.

`manifest.yaml` in the models folder lists approved hashes. Only `approve_model` writes it, and it checks that both reports pass first. **The node refuses any model that is not in the manifest**, the same idea as `race_launch.py` refusing a broken GPU config. Models are never committed to git.

---

## Phase 0: IMU ready

> **Changed 2026-10-24.** This phase was written around `racerbot_imu`, which has since been removed. `imu_bridge` now owns `/imu/data` and publishes `/imu/status` (see `src/imu_bridge/README.md`, "The `/imu/status` verdict"). Its mounts and calibration come from the imu-calibration repo, not `imu_rb1.yaml`. Read the `racerbot_imu` names below as history. Two parts are not ported yet: the two-IMU yaw-rate cross-check, and the P0.4/P0.5 fitting tools. Measure the noise and delay (P0.5) on `imu_bridge`'s `/imu/data`, since that is what the policy sees.

**Goal:** both IMUs working and conditioned, one chosen as primary by measurement and published on `/imu/data` (SI units, `base_link` axes, gyro bias removed), the other cross-checking it, their noise and delay measured, and both simulators producing the same signals.

**Why first:** the student policy's IMU model (Stage 2) is fitted to these measurements, and a policy trained on the wrong IMU is worse than one trained on none. It also helps the rest of the car: today `/odom`'s heading is integrated from the *commanded* steering, and a working gyro is the biggest available improvement to that ([hardware-reference.md](../../hardware-reference.md)).

### P0.1 Find out how each IMU is connected, and what it reads

**First, follow the front IMU's cable.** Does it go to the VESC (its COMM or I²C header), or to the Jetson (USB, or the 40-pin header)? Note the IMU's model printed on the board. Then:

| Front IMU goes to | Then |
|---|---|
| the VESC | open VESC Tool (P0.2) and note which IMU type is selected, internal or an external model. The VESC reports one at a time, so run P0.1 to P0.5 once per IMU, switching the type between runs, and the selector (P0.3) in single-IMU mode |
| the Jetson | it needs a driver: an apt ROS 2 driver for that model if one exists for Jazzy, otherwise a small one in `racerbot_imu`. It must publish a raw `sensor_msgs/Imu` on `/sensors/imu_front/raw`, in whatever units the chip gives, with the scales set in P0.3. A USB unit also gets a udev rule and a stable `/dev/sensors/imu_front` link, like the VESC and LiDAR |

Then read the VESC's IMU:

**Terminal 1** — the VESC driver on its own. Nothing publishes commands to it, so the car can't move.

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 run vesc_driver vesc_driver_node --ros-args \
  --params-file install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml
```

**Working when:** it logs `Connected to VESC with firmware version 7.0` and keeps running.

**Terminal 2** — read one sample with the car sitting still and level.

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /sensors/imu/raw --once
ros2 topic hz /sensors/imu/raw
```

What you see decides what's next:

| You see | Meaning | Next |
|---|---|---|
| Every field exactly `0.0` | IMU disabled in firmware, or no IMU chip | P0.2 |
| `linear_acceleration` magnitude about **1.0** on one axis | IMU working, in g as the driver's comments say | P0.3 |
| magnitude about **9.8** | IMU working and already in m/s². The conversion factor in P0.3 becomes 1 | P0.3, and record this in hardware-reference.md |
| Anything else, or a rate far from 50 Hz | unknown | stop and report the numbers |

Repeat the same check on `/sensors/imu_front/raw` if the front IMU has its own driver.

### P0.2 Enable the IMU in VESC Tool

1. Stop every ROS process holding the VESC. Only one program can hold its serial port.
2. Connect **VESC Tool** over USB.
3. Go to **App Settings → IMU**. Note the selected IMU type. Set it to the IMU you're testing (internal, or the front unit's model if it's wired to the VESC), and the sample rate to a non-zero value. **TODO(verify):** the exact option names for firmware 7.0.
4. Write the app configuration, and power-cycle.
5. Repeat P0.1.

If the internal IMU stays at zero with the type set to internal, **the board has no IMU of its own**. Then the front IMU is the only one: it becomes the primary, and the selector runs single-IMU. Everything from P0.3 onward still applies, because the conditioner is source-agnostic (`input_gyro_scale`, `input_accel_scale`, `input_topic`).

### P0.3 The `racerbot_imu` package

New `ament_python` package, `src/racerbot_imu/`. It publishes no command topic, so it's support code, not driving code.

| File | Purpose |
|---|---|
| `racerbot_imu/imu_math.py` | ROS-free: unit conversion, rotation, bias estimation, sample validation |
| `racerbot_imu/imu_conditioner_node.py` | one instance per IMU: subscribes to that IMU's raw topic, publishes `/imu/<name>/data` and `/imu/<name>/status` |
| `racerbot_imu/imu_select_node.py` | republishes the primary on `/imu/data`, cross-checks it against the secondary, and publishes the combined `/imu/status` |
| `racerbot_imu/imu_crosscheck.py` | ROS-free: time alignment and the gyro agreement test |
| `racerbot_imu/calibrate_axes.py` | console script for P0.4 |
| `config/imu_conditioner.yaml` | the generic defaults |
| `test/test_imu_math.py`, `test/test_imu_conditioner_node.py` | tests |

Car 1's own values go in `src/racerbot_launch/config/imu_rb1.yaml`, beside `web_dashboard_rb1.yaml`: one block per IMU (`vesc`, `front`) and one for the selector. They are started by a new `racerbot_launch/launch/imu_launch.py`, which runs two conditioners and the selector. `bringup_launch.py` is vendored and stays untouched.

```python
# racerbot_imu/imu_math.py -- no rclpy
G0 = 9.80665  # m/s^2, standard gravity

@dataclass(frozen=True)
class ImuSample:
    t: float                      # s, receive stamp
    gyro: np.ndarray              # (3,) rad/s, base_link axes after conditioning
    accel: np.ndarray             # (3,) m/s^2, specific force, base_link axes

class ImuSampleError(ValueError): ...

def to_si(gyro_raw, accel_raw, gyro_scale: float, accel_scale: float) -> tuple[np.ndarray, np.ndarray]:
    """Multiply raw axes by the input scales (VESC: pi/180 and G0)."""

def rotate_to_base(vec, r_base_imu: np.ndarray) -> np.ndarray:
    """r_base_imu is a proper rotation (orthonormal, det +1); raises ImuSampleError otherwise."""

def validate(gyro_raw, accel_raw) -> None:
    """Raise ImuSampleError(match='all zero') when all six raw fields are exactly 0.0,
    and ImuSampleError(match='non-finite') for NaN/inf."""

def gravity_check(accel_si_samples: np.ndarray, tol: float = 0.5) -> float:
    """Mean |a| over a stationary window; ImuSampleError(match='gravity') unless within G0 +/- tol."""

class GyroBiasEstimator:
    """Mean gyro over a stationary window. Stationary means |odom speed| < 0.01 m/s for the
    whole window and gyro std < 0.02 rad/s per axis. Window 2.0 s. Re-estimates every
    time the car is stationary for a full window; never updates while moving."""
    def update(self, t: float, gyro_si: np.ndarray, odom_speed: float) -> None: ...
    @property
    def bias(self) -> np.ndarray | None: ...   # None until the first full window
```

**`imu_conditioner_node`** (one per IMU; topics below are for the instance named `<name>`)

- **Subscribes:** `input_topic` (`/sensors/imu/raw` for the VESC, `/sensors/imu_front/raw` for the front unit) and `/odom` (for "stationary").
- **Publishes:** `/imu/<name>/data`, a `sensor_msgs/Imu` in SI, `frame_id: base_link`, gyro bias removed. Orientation covariance is set to −1 (not provided), per REP-145. It publishes nothing until the first bias window and the gravity check have both passed.
- **Publishes:** `/imu/<name>/status`, a latched `std_msgs/String` holding JSON: `{state: waiting|ok|fault, reason, bias, rate_hz}`.
- **Parameters:** `input_topic`, `input_gyro_scale` (0.0174533), `input_accel_scale` (9.80665), `r_base_imu` (9 floats, row-major, default identity), `stationary_window_sec` (2.0), `gravity_tolerance` (0.5), `stale_timeout_sec` (0.1).
- **Saturation.** A sample with any gyro axis beyond `gyro_saturation` (default 34.9 rad/s, 2000 °/s) or any accelerometer axis beyond `accel_saturation` (default 156.9 m/s², 16 g) is flagged as saturated in `/imu/<name>/status` and **not published**. The node then sees a short gap, as it would for a dropout. With the IMU on the front bumper, a wall strike is exactly when this happens. **TODO(verify):** the real ranges from the IMU chip's datasheet once P0.1 identifies it.
- **On a fault** (all zeros, gravity check failed, input stale), it stops publishing `/imu/<name>/data` and sets state `fault`. Stopping, rather than publishing zeros, is deliberate: a consumer sees a stale stream and refuses it, instead of reading "not turning".

| Test | Asserts | Oracle | Break it with |
|---|---|---|---|
| `test_to_si_vesc_units` | 90 deg/s → π/2 rad/s; 1 g → 9.80665 m/s² | closed form | swap the two scales |
| `test_rotate_known_mount` | an IMU mounted rotated 90° about z: raw +x reads as base +y | closed form, rotation matrix | transpose the rotation |
| `test_rotation_must_be_proper` | a reflection (det −1) raises `match='proper'` | spec above | drop the determinant check |
| `test_all_zero_rejected` | six zeros raise `match='all zero'`; one non-zero field passes | car 2's 2026-08-19 measurement | return early |
| `test_gravity_check_bounds` | 9.81 passes; 9.81 ± 0.51 fails; exactly ± 0.50 passes; `NaN` raises | spec | `<` → `<=` |
| `test_bias_needs_full_stationary_window` | a 1.9 s window gives `None`; 2.0 s gives the mean; one moving tick in the window resets it | spec | drop the moving check |
| `test_bias_never_updates_while_moving` | a constant turn at 1 rad/s while moving leaves the bias unchanged | invariant | remove the speed gate |
| `test_node_publishes_nothing_until_ready` (node) | no `/imu/<name>/data` before the bias and gravity checks; `/imu/<name>/status` says `waiting` | spec | publish immediately |
| `test_node_stops_on_all_zero` (node) | after an all-zero raw message, `/imu/<name>/data` stops and status is `fault` | spec | publish zeros instead |
| `test_saturated_sample_dropped` | a sample at exactly the limit is published; one step past it is not, and is counted in status | spec | `>` → `>=` |

**`imu_select_node`**

- **Subscribes:** `/imu/<primary>/data` and `/status`, and the same for the secondary if `secondary` is set.
- **Publishes:** `/imu/data`, the primary's samples **unchanged**, and `/imu/status` (`ok` or `fault`, with the reason and the latest disagreement).
- **Parameters:**
  - `primary`: `vesc` or `front`, chosen in P0.5
  - `secondary`: the other, or empty for single-IMU mode
  - `secondary_required`: `true`. If the secondary is configured but stale or faulted, that's a fault
  - `delay_primary_sec`, `delay_secondary_sec`: from P0.5, used to time-align the two
  - `gyro_tolerance`: `max(0.05 rad/s, 5σ of the disagreement measured in P0.5)`, plus 5% of `|ω|` to allow for scale error
  - `disagreement_sec`: `0.5`
- **Agreement test:** compare the mean `gyro.z` of each IMU over the same 0.2 s window, shifted by their delays. A fault is the difference staying above tolerance for `disagreement_sec`. Only yaw rate is compared live; the accelerometers are checked against each other offline in P0.5, because comparing them live needs the yaw acceleration, which is too noisy to differentiate from a 50 Hz gyro.

```python
# racerbot_imu/imu_crosscheck.py -- no rclpy
def aligned_window_mean(samples: Sequence[ImuSample], t_end: float, window: float, delay: float) -> float | None:
    """Mean gyro.z over [t_end - delay - window, t_end - delay]; None if fewer than 5 samples."""

class GyroAgreement:
    def __init__(self, base_tol: float, rel_tol: float, hold_sec: float): ...
    def update(self, t: float, w_primary: float | None, w_secondary: float | None) -> tuple[bool, float]:
        """(agree, disagreement). A missing value counts as disagreeing."""
```

| Test | Asserts | Oracle | Break it with |
|---|---|---|---|
| `test_agreement_within_tolerance` | 1.00 vs 1.04 rad/s with tolerance 0.05 + 5% agrees | closed form | drop the relative term (then 1.00 vs 1.09 wrongly faults) |
| `test_disagreement_needs_hold_time` | a 0.4 s disagreement stays `ok`; 0.5 s faults | spec | fault immediately |
| `test_delay_alignment` | two identical traces, one delayed 30 ms, agree once the delays are given, and disagree in a fast turn when they aren't | closed form | ignore the delays |
| `test_primary_passed_through_unchanged` (node) | every `/imu/data` message equals the primary's input, field for field | invariant | average the two |
| `test_secondary_stale_faults_when_required` (node) | secondary silent 0.2 s with `secondary_required` → `fault` | spec | treat silence as agreement |
| `test_single_imu_mode` (node) | with `secondary` empty, status follows the primary alone | spec | require a secondary |

### P0.4 Calibrate both IMUs' axes and mount positions on car 1

Do this once per IMU. **First, measure where it is.** With a tape measure, from the rear axle's centre (`base_link`) to the centre of the IMU chip, or the VESC's case for its onboard IMU:

- forward distance: about 0.45 m expected for the front unit; the VESC's depends on where it sits
- left distance, positive if left of centre
- height above the axle

Record them as that IMU's `offset_x/y/z` in `imu_rb1.yaml` and in `calibration.py` (`imu_front_offset_*`, `imu_vesc_offset_*`), as MEASURED with the date.

In a steady low-speed turn the rear axle has no forward acceleration, so the bumper's forward reading is `−ω² · imu_offset_x`. A 2 cm error in a 0.45 m offset is about 4% of that reading.

So measure to the centimetre, and **check the mount is rigid**: press on the bumper and watch the IMU housing. Anything that flexes adds vibration the simulator doesn't have.

**Then calibrate the axes.** `ros2 run racerbot_imu calibrate_axes --imu front` (then `--imu vesc`) records two things and writes that IMU's `r_base_imu` into `imu_rb1.yaml`. If both stream at once, one recording calibrates both:

1. **10 s with the car still and level.** Gravity gives the IMU's up axis.
2. **A gentle hand-push straight forward** on the floor, with the drivers up and **no control layer running**. The acceleration direction, with gravity removed, gives forward.

Gram–Schmidt on those two vectors gives the rotation. Then it **checks** the result: spin the car counter-clockwise by hand, seen from above, and conditioned `gyro.z` must read positive. Anything else fails loudly.

| Test | Oracle | Break it with |
|---|---|---|
| `test_axes_from_synthetic_mount` | 20 random proper rotations: synthetic gravity and push data recover each to 0.5°, with noise added | swap the order of Gram–Schmidt |
| `test_axes_rejects_parallel_push` | a "push" parallel to gravity raises `match='parallel'` | drop the check |

### P0.5 Measure noise, bias and delay

**Record** with `race_diagnostics`, in three parts, driven with teleop:

1. 60 s still.
2. 60 s straight at a steady 1.5 m/s. Noise while driving is what the policy actually sees, and at the bumper it's mostly vibration, which the still recording can't show.
3. 5 minutes of the circle and figure-eight runs from [odom-calibration.md](../../odom-calibration.md).

Record every IMU that's streaming. If the VESC reports only one at a time, record the whole set once per IMU.

**Analyse** with a new `tools/run_analysis/imu_fit.py`, which outputs, for each IMU:

- **Gyro and accelerometer white-noise standard deviation**, from the still part, and again from the steady straight. The simulator uses the **driving** values, and its tests use the still ones.
- **The lever-arm check:** in the steady circles, the measured `a_x` against `−ω²·imu_offset_x` (plus the speed change from scan matching). A slope far from 1 means the mount position or the axes are wrong.
- **Bias stability** across stationary windows.
- **IMU delay:** the lag that maximises the cross-correlation between conditioned `gyro.z` and the LiDAR scan-matched yaw rate from `scan_motion.py`.
- **Gyro scale error:** the slope of `gyro.z` against the scan-matched yaw rate.

And, when both stream at once:

- **The gyro disagreement** between them, after alignment. Its standard deviation sets the selector's `gyro_tolerance`.
- **The rigid-body accelerometer check:** in steady circles, `a_x,front − a_x,vesc` must equal `−ω² · (x_front − x_vesc)`. A slope far from 1 means one of the two mount positions or axis calibrations is wrong.

**Choosing the primary**, recorded with the numbers in hardware-reference.md:

1. Lower delay wins, if the two differ by more than 10 ms. The network acts on the IMU, so delay costs most.
2. Otherwise, lower gyro and accelerometer noise **while driving** wins.
3. On a tie, the VESC's IMU, if it's mounted more rigidly.

Write each value into `sim_fidelity/calibration.py` as MEASURED, with the run name, and set `imu_primary`.

| Test (`tools/run_analysis/tests/`) | Oracle | Break it with |
|---|---|---|
| `test_delay_recovered_from_synthetic` | a yaw-rate trace delayed by 3 ticks plus noise: recovers 3 | swap the correlation's arguments (gives −3) |
| `test_scale_recovered` | a trace scaled by 1.07 recovers 1.07 to ±0.01 | return 1.0 |
| `test_lever_arm_slope` | a synthetic steady circle with the IMU 0.45 m ahead recovers a slope of 1.0 ± 0.02; with the offset entered as 0.30 m, about 1.5 | drop the ω² term |
| `test_two_imu_rigid_body` | synthetic IMUs at 0.45 m and 0.15 m in a steady circle: the difference fits slope 1.0 ± 0.02; swapping their offsets gives −1 | swap the subtraction |
| `test_primary_choice_rule` | spec: 25 ms vs 10 ms delay picks the 10 ms one whatever the noise; equal delays pick the lower driving noise | compare still noise instead |

### P0.6 Self-check: `racerbot_doctor` learns the IMU

Add an IMU ladder to `racerbot_doctor`, one per configured IMU, plus an agreement layer (thresholds in `car.py`, check in `checks.py`, live reading in `live.py`). It stays read-only: it never opens a device, an I²C bus included, and never publishes.

| Layer | Pass | Fail message |
|---|---|---|
| rate | `/sensors/imu/raw` at 45–55 Hz | "VESC driver not polling the IMU" |
| non-zero | any non-zero field in 2 s | "IMU disabled in VESC firmware (VESC Tool → App Settings → IMU) or not fitted" |
| gravity | still car: magnitude 1.0 ± 0.05 raw (g) | "IMU units or mount unexpected: read N" |
| conditioned | `/imu/<name>/status` is `ok` | its `reason` |
| agreement (both IMUs) | `/imu/status` is `ok`; disagreement under tolerance | "IMUs disagree by N rad/s: a loose mount, or wrong axes on one of them" |
| front IMU device (if on the Jetson) | `/dev/sensors/imu_front` exists, for a USB unit | "front IMU not enumerated: check its cable and power" |

Tests follow the doctor's existing pattern: pure functions over recorded message values, plus its static test that nothing opens a device.

### P0.7 Both simulators get the IMU

- **`tools/f1tenth_sim/sim_fidelity/sensing.py`:** a new `ImuSensor(noise, bias, delay_ticks, rng)` computing yaw rate and specific force at `imu_offset_x` from gym's single-track state.
  - Yaw rate comes from the state.
  - Acceleration is the finite difference of body-frame velocity over the 5 ms substep, plus `ω × v`.
  - New `calibration.py` fields, per IMU (`imu_front_*`, `imu_vesc_*`): `rate_hz` (50, SPEC), `offset_x/y` (MEASURED in P0.4), `gyro_noise`, `gyro_bias_walk`, `accel_noise`, `delay_ticks` (MEASURED in P0.5). Plus `imu_primary`.
  - The GPU training simulator models the **primary only**, because only the primary reaches the network.
- **`racerbot_sim`'s `gym_bridge_node`:** publishes **both** raw IMU topics, each in its own source's raw units (deg/s and g for the VESC's), so the real conditioners and selector run unchanged on top, cross-check included. This is the same reason the simulator publishes `/scan` and `/odom` rather than anything pre-digested.
  - It stays inside `hardware_guard`'s refusal to run beside real drivers.

| Test | Oracle | Break it with |
|---|---|---|
| `test_imu_constant_turn` | constant speed v on a circle of radius R: `ω_z = v/R`, `a_y = v²/R` at the CoG, ±2% | flip the sign of `a_y` |
| `test_imu_straight_accel` | straight-line constant acceleration a: `a_x = a`, `ω_z = 0` | drop the `ω × v` term (the turn test catches it) |
| `test_imu_offset_lever_arm` | an IMU 0.1 m ahead of the CoG in a steady turn reads `a_x = −ω²·0.1` | ignore `imu_offset_x` |
| `test_sim_bridge_raw_units` (racerbot_sim) | the simulated raw message, conditioned with the real scales, matches the simulated SI truth | publish SI in the raw topic |

### P0.8 Docs

- **[hardware-reference.md](../../hardware-reference.md):** car 1's IMU status, the raw units, the mount rotation and the measured noise and delay.
- **[car1-vs-car2.md](../../car1-vs-car2.md):** a row if car 1's IMU differs from car 2's (enabled, external, or a different mount). If it's car-specific config, add a check to `test_car1_hardware_config.py`.
- **[localization.md](../../localization.md):** a note that a working gyro now exists for `/odom` heading. Changing `vesc_to_odom` to use it is a separate piece of work, not part of this plan.

### Gate G0

All of these on car 1:

- Both IMUs (or, if the VESC reports one at a time, each in turn) pass the checks below.
- `/imu/data` at 50 ± 3 Hz, and `/imu/status` `ok`.
- With both streaming: **no false cross-check fault** over the whole P0.5 recording. Then, later, none over Phase 7's shadow laps either.
- The primary is chosen by the P0.5 rule, and the numbers are recorded in hardware-reference.md.
- At rest, conditioned `|a|` is 9.81 ± 0.3 m/s², with z up.
- A hand-spin counter-clockwise reads positive `gyro.z`.
- Residual gyro bias under 0.01 rad/s.
- Noise, delay and scale error from P0.5 recorded in `calibration.py`, with the run name.
- The doctor's IMU section passes.
- Every test above passes, and has been seen to fail against its break.

**Stop points:** P0.1's "anything else" row. The two gyros disagreeing by more than 10% in steady turns (one has a wrong scale or axis). An IMU delay over 60 ms (it would make the IMU a poor control input; report it). A gyro scale error over 10% (suspect units or mount).

---

## Phase 1: baselines and the held-out set

**Goal:** fixed numbers for "better", and a held-out set that training can never see.

1. **Give the CPU simulator car 1's speed gain.** `calibration.py` still has car 2's `SPEED_TO_ERPM_GAIN = 4614`, which makes the simulated car 1's top speed 5.04 m/s instead of 6.37.
   - Set it to car 1's 3650 from `vesc.yaml`. `racerbot_sim/sim_bridge.py` sets no top speed at all in its `F1TENTH_VEHICLE_PARAMETERS.with_updates(...)` overrides, so it runs at gym's stock value; add car 1's there too.
   - This is the gap [car1-vs-car2.md](../../car1-vs-car2.md) already lists ("Simulator vehicle model: not car 1's").
   - Update that row, and re-run `tools/f1tenth_sim/tests/` before recording any baseline.
2. **Confirm the LiDAR.** Run `ros2 topic echo /scan --once --field angle_increment` and `--field ranges` (count them) on car 1. Expected: 0.004363 rad, 1081 ranges. If they differ, **stop and report**: the observation spec's pooling assumes 1081.
3. **Fix the LiDAR's provenance.** If car 1 matches, correct `calibration.py`'s `lidar_*` text from UST-10LX to UST-30LX. Also check `lidar_range_max` against the UST-30LX datasheet.
4. **Write `tools/neural_racer/heldout.yaml`:**
   - the four real course maps
   - 20 procedural tracks by seed (generated in Phase 4, so their hashes are filled in then)
   - the SHA-256 of each rasterized occupancy grid
5. **Baselines on the held-out set in `sim_fidelity`:** `gap_follow` (with `GapController` from `gap_mirror.py`) solo, `pure_pursuit` on the optimized line solo, and `pure_pursuit` with overtaking against scripted traffic (`run_pure_traffic`'s setup). Record lap time, contacts and minimum clearance per track. Use `run_validation.py`'s existing scenarios where they exist; add the held-out tracks to its track list.
6. **Save** the results as `docs/neural-racer-baselines.json`, with the commit and run date.

**Gate G1:** the baselines file exists, covers every held-out track, and its numbers are reproducible: re-running gives the same results with the same seeds.

---

## Phase 2: the ROS-free core

**Goal:** everything the car needs, apart from trained weights, built and tested on the Jetson. Nothing here needs a GPU or PyTorch, so it can start right away.

### Files

`src/neural_racer/` as laid out in [B0](#where-the-code-goes), with `package.xml` dependencies on `rclpy`, `sensor_msgs`, `nav_msgs`, `ackermann_msgs`, `std_msgs` and `gap_follow` (for `gap_logic`).

### Interfaces

```python
# observation.py -- no rclpy
OBS_SPEC_VERSION = 1
N_BEAMS = 1081; POOL = 4; N_BINS = 270
R_MIN, R_MAX = 0.05, 10.0
V_SCALE, STEER_SCALE, YAW_SCALE, ACC_SCALE = 5.0, 0.33, 3.0, 9.81
PREVIEW_POINTS, PREVIEW_SPACING = 32, 0.5
EGO = {"speed": slice(0, 3), "steer_cmd": slice(3, 6), "speed_cmd": slice(6, 9),
       "gyro_z": slice(9, 12), "acc_x": slice(12, 15), "acc_y": slice(15, 18),
       "imu_present": 18, "mode": 19, "v_cap": 20, "camera_present": 21, "camera_age": 22}

class ObservationError(ValueError): ...

def pool_scan(ranges: np.ndarray, range_min: float, range_max: float) -> tuple[np.ndarray, np.ndarray]:
    """1081 raw ranges -> (270 encoded log-ranges, 270 validity). ObservationError(match='beam count')."""

@dataclass
class History:                      # what the node keeps between ticks
    scans: deque                    # last 3 encoded scans
    speeds: deque; steer_cmds: deque; speed_cmds: deque
    gyro_z: deque; acc_x: deque; acc_y: deque
    def push(...): ...
    def reset(self) -> None: ...

def build_observation(history: History, *, imu_present: bool, mode: int, v_cap: float,
                      preview: np.ndarray | None, race: np.ndarray | None,
                      camera: np.ndarray | None, camera_age: float) -> dict[str, np.ndarray]:
    """Returns {'scan': (4,270), 'ego': (23,), 'preview': (4,32), 'race': (6,), 'camera': (64,)},
    all float32. Missing blocks are zeros, and their flags say so."""

# policy_numpy.py -- NumPy only
class PolicyNumpy:
    def __init__(self, weights: dict[str, np.ndarray], card: dict): ...    # validates shapes against the card
    def reset(self) -> None: ...                                            # GRU state to zeros
    def step(self, obs: dict[str, np.ndarray]) -> PolicyOutput: ...

@dataclass(frozen=True)
class PolicyOutput:
    steering: float; speed: float                 # already bounded
    ego_path: np.ndarray                          # (6, 2) car frame, m
    opponents: np.ndarray                         # (4, 7): presence prob + 6 x (x, y) ... flattened per slot

# safety_wrapper.py -- no rclpy
@dataclass(frozen=True)
class WrapperConfig:
    max_steering_angle: float = 0.33; max_speed: float = 1.0
    car_width: float = 0.31; car_length: float = 0.58; wheelbase: float = 0.36; laser_offset_x: float = 0.26
    reserve_distance: float = 0.20; max_braking_decel: float = 3.0; braking_latency_sec: float = 0.25
    max_lateral_accel: float = 2.0; max_steering_rate: float = 2.0  # dated note: gap_follow moved to 2.5 (decided 2026-10-26); this stays 2.0 (training cap unchanged)
    max_invalid_fraction: float = 0.30

@dataclass(frozen=True)
class WrapperResult:
    steering: float; speed: float
    stopped: bool; reason: str | None             # e.g. 'deadman', 'stale_scan', 'nan_output'
    binding: str | None                           # which cap set the speed: 'network', 'braking', 'curvature', ...
    intervened: bool                              # anything other than the network set the command

def apply(cfg: WrapperConfig, net: PolicyOutput, scan_ranges: np.ndarray, angles: np.ndarray,
          valid: np.ndarray, prev_steering: float, dt: float,
          frozen_map=None, pose=None) -> WrapperResult: ...
```

The deadman and input-freshness checks (A6 steps 1–2) live in the node, which owns the clocks. `apply` covers steps 3–8.

### Tests (Jetson, unsourced)

| Test | Asserts | Oracle | Break it with |
|---|---|---|---|
| `test_pool_keeps_minimum` | a bin with ranges [3, 0.4, 5, 5] encodes 0.4 | spec (min-pool) | mean instead of min |
| `test_pool_invalid_beams` | `NaN`, `inf`, below `range_min`, at `range_max` → invalid; an all-invalid bin reads 10 m with validity 0 | spec | treat `inf` as valid |
| `test_pool_beam_count` | 1080 and 1082 raise `match='beam count'` | spec | drop the check |
| `test_encode_endpoints` | 0.05 m → 0.0, 10 m → 1.0, 20 m → 1.0, 0.0 m → 0.0 | closed form | swap `R_MIN`/`R_MAX` |
| `test_ego_layout` | each history value lands at its `EGO` index, with its scale | spec table | swap two slices |
| `test_missing_blocks_are_zero` | reactive mode: preview and race are exactly 0, mode is 0 | spec | leave stale preview in |
| `test_imu_missing_zeroes_imu_slots` | `imu_present=False` → indices 9–17 are 0 and 18 is 0 | spec | keep the last IMU values |
| `test_policy_golden` | `PolicyNumpy` reproduces a model's `golden.npz` to `atol=1e-5` (dimensionless) | PyTorch, an independent implementation | transpose one weight |
| `test_policy_bounds` | 10,000 random and adversarial inputs (huge, `NaN`-free) never give steering beyond ±0.33 or speed outside [0, v_cap] | invariant | remove the `tanh` |
| `test_policy_reactive_ignores_preview` | in reactive mode, changing the preview changes no output | invariant (`× mode`) | gate before the bias |
| `test_policy_camera_slot_inert` | a v1 model's output doesn't change when `camera` changes while `camera_present = 0` | invariant | gate after the fusion |
| `test_wrapper_nan_output` | `NaN` steering or speed → speed 0, `reason='nan_output'` | spec | skip the check |
| `test_wrapper_braking_cap` | an obstacle at clearance c → speed equals `braking_speed_limit(c, ...)` | `gap_logic`'s closed form | drop the cap |
| `test_wrapper_curvature_cap` | δ = 0.33 → speed ≤ `sqrt(2.0 · 0.36 / tan 0.33)` | closed form | use δ instead of tan δ |
| `test_wrapper_steering_rate` | a jump from 0 to 0.33 in one 25 ms tick is limited to 0.05 rad | closed form | drop the slew |
| `test_wrapper_never_raises_speed` | over random inputs, output speed ≤ network speed | invariant | `max` for `min` |
| `test_wrapper_edge_scans` | all-`inf`, all-zero, all-`NaN`, wrong length: each gives a defined result (zero command or a capped speed), never an unhandled exception | A4 edges | — |
| `test_model_store_rejects_unapproved` | a model directory missing from the manifest raises `match='not approved'`; a modified `weights.npz` raises `match='hash'` | spec | skip the hash |
| `test_model_store_modalities` | a model needing `camera` with `camera_enabled: false` raises `match='modality'` | spec | ignore the card |
| `test_latency_budget` | p99 of `build_observation` + `PolicyNumpy.step` + `apply` over 2,000 ticks with random weights ≤ 5 ms | measured: 3.50 ms p99 for the forward pass alone (A4) | — (this is a performance gate, not a behaviour test) |

**Gate G2:** every test passes on the Jetson unsourced, and each was seen to fail against its break. `test_policy_golden` runs against a golden file produced by a randomly initialised PyTorch model exported by Phase 3's `export.py`. Until then it is `xfail(strict=True, reason="needs export.py, Phase 3")`.

---

## Phase 3: GPU simulator and parity

**Goal:** a batched simulator that matches `sim_fidelity`, and a measured throughput.

### Files and interfaces

```python
# gpu_sim/sim.py
@dataclass(frozen=True)
class SimConfig:
    num_envs: int = 16384; max_cars: int = 4
    dt: float = 0.025; substeps: int = 5
    beams: int = 1081; fov: float = math.radians(270); lidar_range_max: float = 25.0
    sphere_trace_iters: int = 24; sphere_trace_eps: float = 0.01
    imu: bool = True; camera: bool = False
    device: str = "cuda"

class BatchedRaceSim:
    def __init__(self, cfg: SimConfig, pool: TrackPool, calib_batch: CalibBatch, gen: torch.Generator): ...
    def reset(self, env_ids: torch.Tensor, levels: LevelBatch) -> None: ...
    def step(self, actions: torch.Tensor) -> StepOut: ...      # actions: (E, C, 2) steering, speed

@dataclass
class StepOut:
    obs: dict[str, torch.Tensor]      # per car, the A4 blocks: scan (E,C,4,270), ego (E,C,23), ...
    priv: torch.Tensor                # (E, C, P) critic / teacher input
    reward: torch.Tensor              # (E, C)
    done: torch.Tensor                # (E, C) bool;  truncated: (E, C) bool
    info: dict[str, torch.Tensor]     # contact, fault, wall, place, lap, progress, intervened
```

- **`dynamics.py`:** `st_derivative(state, steer_actual, accel, params)` and `rk4(state, ..., h)`. State per car: x, y, yaw, v, β, ω, δ.
- **`lidar.py`:** `cast(sdf, track_idx, origins, yaws, angles, boxes)` → ranges `(E, C, 1081)`.
- **`imu.py`:** `sample(state, deriv, params, noise_state)` → `(gyro_z, a_x, a_y)`, sampled at 50 Hz with a sample-and-hold to the 40 Hz tick.
- **`actuation.py`:** servo, delay ring buffers, speed loop.
- **`contact.py`:** `wall(sdf, ...)` and `cars(poses, body)` → contact plus fault, as in [A8](#reward).
- **`observation_torch.py`** and **`safety_torch.py`**: the batched versions of Phase 2's modules, importing their constants.
- **`camera.py`:** `render(...)` returns zeros of shape `(E, C, 64)` while `camera=False`.

The simulator must also run on `device="cpu"`, at small `num_envs`, so the parity tests run anywhere.

### Parity tests (`tools/neural_racer/tests/`)

| Test | Oracle | Tolerance | Break it with |
|---|---|---|---|
| `test_trajectory_matches_fidelity` | `sim_fidelity`'s `FidelityPlant` (via `make_plant`) stepping the same recorded command sequence from the same start, 10 s, randomized physics | 2 cm position, 0.5° heading | flip steering sign; drop the friction clamp |
| `test_circle_radius_low_speed` | `L / tan δ` at 0.5 m/s | 1% | wrong wheelbase |
| `test_scan_matches_gym` | gym's `ScanSimulator2D` on the same map and pose | one grid cell (5 cm) on 99% of beams | swap x and y |
| `test_ray_box_exact` | closed-form ray-versus-rectangle on 1,000 random cases | 1 mm | — |
| `test_car_contact_matches_plant` | `plant.py`'s separating-axis test on 10,000 random pose pairs | exact agreement | use the padded envelope for both cars |
| `test_fault_rear_ender` | A drives into B's rear: A at fault, B not | spec | invert the comparison |
| `test_imu_matches_fidelity_imu` | Phase 0's `ImuSensor` with noise off, same run | 1e-4 rad/s and 1e-3 m/s² | drop `imu_offset_x` |
| `test_observation_torch_matches_numpy` | Phase 2's `build_observation` on random and recorded inputs | `atol=1e-6` | change one scale |
| `test_safety_torch_matches_numpy` | Phase 2's `apply` on random inputs | `atol=1e-6` | drop one cap |
| `test_export_golden_roundtrip` | `model.py` → `export.py` → `PolicyNumpy` | `atol=1e-5` | transpose in the exporter |
| `test_no_host_sync_in_step` | `torch.cuda.set_sync_debug_mode("error")` around 100 `step` calls raises nothing (CUDA only) | spec ([A9](#a9-fir-and-gpu-efficiency)) | add an `.item()` |

### The `benchmark` stage

`neural_racer_train/benchmark.py` follows the trainer contract in [the scripts' README](../../../tools/neural_racer/slurm/README.md#the-trainer-contract). It runs the simulator plus a random policy for 60 s at each of 4k, 8k, 16k, 32k and 64k environments. For each it writes `benchmark.json` with steps/s, GPU memory and mean utilisation, then picks the smallest count within 5% of the best throughput.

```bash
sbatch --account=def-<pi> --time=00:30:00 tools/neural_racer/slurm/fir_train.sbatch benchmark bench-v1
```

**Gate G3:** every parity test passes, and was seen to fail against its break. The FIR benchmark has run, the [A9 budget table](#a9-fir-and-gpu-efficiency) has been updated with the measured steps/s, and the chosen count's GPU utilisation is at least 80%.

**Stop points:** any parity test that fails and seems to need a wider tolerance. A measured throughput under 50k steps/s (the plan's budget no longer holds; report before cutting scope).

---

## Phase 4: tracks and the curriculum

### Files and interfaces

```python
# tracks.py -- NumPy/SciPy, runs in CPU worker processes
@dataclass(frozen=True)
class TrackSpec:
    seed: int; control_points: np.ndarray        # (K, 2)
    widths: np.ndarray                           # (K,) m, interpolated periodically
    obstacles: np.ndarray                        # (M, 4) x, y, w, h boxes

def generate(seed: int, rng_cfg: TrackGenConfig) -> TrackSpec: ...
def mutate(spec: TrackSpec, rng: np.random.Generator, cfg: MutationConfig) -> TrackSpec: ...
def rasterize(spec: TrackSpec, resolution: float = 0.05) -> RasterTrack: ...   # occupancy, SDF, centreline table
def check(spec: TrackSpec) -> list[str]: ...      # empty if valid; reasons otherwise

# track_pool.py -- GPU side
class TrackPool:                                 # fixed slots, LRU eviction of low-score levels
    def __init__(self, slots: int, max_cells: tuple[int, int], device): ...
    def insert(self, raster: RasterTrack) -> int: ...
    sdf: torch.Tensor                            # (T, H, W) fp16
    centreline: torch.Tensor                     # (T, N, 5): x, y, s, w_left, w_right

# plr.py
class LevelBuffer:
    def __init__(self, capacity: int = 4000, replay_prob: float = 0.7,
                 staleness: float = 0.3, temperature: float = 0.3): ...
    def sample(self, n: int) -> tuple[list[Level], torch.Tensor]: ...   # levels, is_replay mask
    def update_scores(self, levels, scores) -> None: ...
```

**Generators (`TrackGenConfig` defaults).** Each level draws its source by these shares:

| Source | Share | How it's built |
|---|---|---|
| `smooth_loop` | 40% | K from 8 to 16 control points at evenly spaced angles (jittered ±30%) and random radii, joined by a periodic cubic spline |
| `filleted_polygon` | 40% | a random convex polygon of 5–10 vertices. 1–3 vertices are pushed inward to make dents, which become hairpins and chicanes. Each corner is rounded with a circular arc of random radius (1.3–6 m), leaving straight edges between them as straights |
| `imported` | 20% | the training split of imported maps (below) |

For both generated sources:

- Lap length 15–120 m (the polygon is scaled to fit).
- Width 0.9–3.0 m, varying smoothly along the lap.
- 0–4 static obstacles, each leaving at least 0.8 m of free width.

Why two generators: smooth loops alone are blob-shaped, with few long straights and no real hairpins. Those are the places races are won: braking points and passing zones.

**`check` rejects** a track with any of these:

- centreline turning radius under 1.3 m (the car's minimum at 0.33 rad is 1.05 m)
- a self-intersection of either boundary
- free width under 0.8 m anywhere

**Mutations:** move 1–3 control points by N(0, 0.5 m), change one width by ±0.2 m, add or remove one obstacle. A mutation that fails `check` is redrawn.

**Imported tracks** are added to the training pool through `track_import.py`: the scaled F1TENTH racetracks, and any new SLAM maps from the car. Every imported or generated track's hash is checked against `heldout.yaml` first.

**Workers:** `--cpu-workers 3` generator processes, started by the trainer, put rasterized tracks on a queue that the trainer inserts into the GPU pool between updates.

| Test | Oracle | Break it with |
|---|---|---|
| `test_generated_tracks_valid` | 500 seeds per generator: every track passes `check`, using an independent curvature computation in the test | drop the radius check |
| `test_fillet_radius_closed_form` | closed form: a square with 2 m fillets has curvature 1/2 m⁻¹ ± 1% on each arc and 0 on the straights | fillet with the wrong tangent length |
| `test_polygon_has_straights` | invariant of the generator's purpose: over 200 seeds, the median longest straight is at least 3 m | round every corner with the 6 m maximum (blob tracks) |
| `test_source_shares` | spec: over 10,000 levels, 40/40/20 ± 2% | swap two shares |
| `test_check_catches_tight_corner` | a hand-built track with a 1.0 m radius is rejected with `match='radius'` | `<` → `<=` at exactly 1.3 |
| `test_check_catches_self_intersection` | a figure-eight is rejected | — |
| `test_sdf_matches_brute_force` | brute-force distance on a 100×100 crop | one cell |
| `test_heldout_never_in_pool` | inserting a held-out hash raises `match='held-out'` | drop the check |
| `test_plr_replay_ratio` | over 10,000 samples, 70% ± 2% are replays | change `replay_prob` |
| `test_plr_prefers_high_score` | with scores [1, 0, 0, …], the first level is sampled most | invert the rank |
| `test_plr_fresh_levels_not_trained` | the `is_replay` mask is false for fresh levels | — |

**Gate G4:** the tests pass, the held-out procedural tracks are generated with their hashes recorded in `heldout.yaml`, and the generator sustains 30 tracks/s on 3 cores.

---

## Phase 5: privileged teacher

**Files:** `teacher_model.py`, `critic.py`, `ppo.py`, `race.py` (the reward), `teacher.py` (the Slurm stage).

**Run:**

```bash
for seed in 0 1 2; do
  sbatch --account=def-<pi> --export=ALL,NR_ROLE=seed$seed \
      tools/neural_racer/slurm/fir_train.sbatch teacher teacher-v1 --seed $seed
done
```

| Test | Oracle | Break it with |
|---|---|---|
| `test_reward_progress_closed_form` | a car driven along a straight at 4 m/s for one tick earns 0.1 | change `v_ref` |
| `test_reward_place_hysteresis` | two cars swapping places within 0.3 m earn nothing; a clean pass earns +2 | drop the hysteresis |
| `test_reward_fault_terminates` | an at-fault contact gives −20 and ends that car's episode only | end both |
| `test_gae_matches_reference` | a hand-computed 5-step GAE example | swap γ and λ |
| `test_ppo_learns_bandit` | a 1-step continuous bandit with its optimum at 0.7: PPO's mean converges to 0.7 ± 0.05 | negate the advantage |
| `test_checkpoint_resume_identical` | train 10 updates; or train 5, checkpoint, resume, train 5: identical weights | drop the optimizer state from the checkpoint |
| `test_sigusr1_exits_85` | sending `SIGUSR1` mid-update writes a checkpoint and exits 85 | exit 0 |

**Gate G5:** in the **referee** (`sim_fidelity`, CPU), the best teacher's solo laps on the held-out tracks are within 5% of the minimum-curvature line's time, with zero contacts.

The teacher can't run on the real car, since it needs privileged state. The referee needs a small privileged-state adapter for this check, used only in Phase 5.

---

## Phase 6: student, export, and the node

### Training

**Files:** `model.py`, which implements [A4](#a4-the-network) exactly, with module names that match `export.py`'s array names, and `distill.py`. Initialize the camera columns of the fusion layer to zero.

**Run** after G0, so the IMU model uses measured values:

```bash
sbatch --account=def-<pi> tools/neural_racer/slurm/fir_train.sbatch distill distill-v1 \
    --teacher "$SCRATCH/neural_racer/runs/teacher-v1/seed<best>/checkpoints/latest"
```

| Test | Oracle | Break it with |
|---|---|---|
| `test_dagger_beta_schedule` | iteration i uses β = 0.5ⁱ | constant β |
| `test_student_never_sees_priv` | `model.py`'s forward takes only the A4 blocks; passing `priv` is a `TypeError` | add a priv input |
| `test_camera_columns_zero_init` | the 64 camera columns of the fusion weights are exactly 0 at initialization | random init |
| `test_aux_targets_from_truth` | the ego-path target equals the car's actual positions over the next 6 × 0.25 s | off-by-one in time |

### Export and approval

- `export.py <checkpoint> <out_dir>` writes `weights.npz`, `model_card.yaml` and `golden.npz` ([B0](#model-artifact)).
- `approve_model` copies the directory into `~/.ros/neural_racer/models/` and adds the hash to `manifest.yaml` only if both reports say pass. Before Phase 9 exists, only `--shadow-only` approval is possible, and the node refuses to drive `/drive` with a shadow-only model.

### The node, `neural_racer_node.py`

**Parameters** (`config/neural_racer.yaml`):

| Parameter | Default | Meaning |
|---|---|---|
| `enable_deadman` | `true` | never `false` in shipped config |
| `deadman_button`, `joy_timeout_sec` | `4`, `0.5` | as `gap_follow` |
| `scan_topic`, `odom_topic`, `imu_topic`, `joy_topic` | `/scan`, `/odom`, `/imu/data`, `/joy` | inputs |
| `drive_topic` | `/drive` | output when not in shadow mode |
| `shadow` | `true` | **shadow is the default.** Driving needs `shadow:=false` explicitly |
| `shadow_topic` | `/neural_racer/shadow_drive` | the only output in shadow mode |
| `intent_topic` | `/drive_intent` | diagnostics |
| `model_hash` | *(required)* | which approved model to load |
| `max_speed` | `1.0` | the wrapper's cap, and `v_cap` for the network |
| `mode` | `reactive` | `reactive`, `map`, or `auto` (map once a reference path arrives) |
| `require_imu` | `true` | stale or faulted IMU → zero command |
| `camera_enabled`, `camera_topic` | `false`, `/neural_racer/camera_embedding` | Phase C |
| `pose_topic`, `path_topic`, `frozen_map_topic` | `/slam_pose`, `/auto_map_race/race_path`, `/auto_map_race/frozen_map` | map mode |
| timeouts | scan 0.075, odom 0.1, IMU 0.1, pose 0.2 s | [A6](#a6-safety-what-wraps-the-network) step 2 |
| wrapper values | as `WrapperConfig` | [A6](#a6-safety-what-wraps-the-network) steps 4–7 |

**Start-up refusals.** The node logs the reason and publishes only zero commands until it is fixed. It refuses:

- a model that isn't approved, or needs a modality that isn't configured
- `shadow: true` with `shadow_topic` set to `/drive`, `/teleop`, `/ackermann_cmd` or anything under `/commands/`
- `shadow: false` with a shadow-only model
- `/imu/status` not `ok` while `require_imu` is set

**Each tick**, on scan arrival:

1. Deadman → input checks → `build_observation` → `PolicyNumpy.step` → `safety_wrapper.apply`.
2. Publish the `AckermannDriveStamped`.
3. Push the command into the history.
4. Publish intent: `drive_intent.build(node='neural_racer', ...)` with `path=ego_path`, `desired_*` from the network and `commanded_*` from the wrapper. It is wrapped in its own try/except, per [drive-intent.md](../../drive-intent.md).

Any stop resets the GRU and the history.

### Node tests (sourced, isolated domain)

Every test builds the node with `drive_topic:=/test_only/drive` (and `shadow_topic:=/test_only/shadow`), with the remap placed **after** `--params-file`.

| Test | Asserts, on the **published command** | Break it with |
|---|---|---|
| `test_lb_held_drives` | LB held, fresh inputs, `shadow:=false` → a non-zero command | — (the permissive half) |
| `test_lb_released_zero` | LB released → speed 0 | return `True` from the deadman check |
| `test_joy_never_received_zero` | no `/joy` ever → speed 0 | default `deadman_held = True` |
| `test_joy_stale_zero` | last `/joy` 0.6 s old → speed 0 | skip the age check |
| `test_imu_stale_zero` | IMU 0.15 s old with `require_imu` → speed 0 | drop the IMU check |
| `test_shadow_never_publishes_drive` | `shadow:=true` → nothing on `drive_topic`, commands on `shadow_topic` | publish to both |
| `test_shadow_topic_guard` | `shadow_topic:=/drive` → the node publishes only zeros and logs the refusal | drop the guard |
| `test_unapproved_model_refused` | a model hash missing from the manifest → only zero commands | skip `model_store` |
| `test_intent_failure_keeps_driving` | an intent payload that raises → the drive command is still published, and intent is disabled | publish intent first |
| `test_gru_reset_on_stop` | after a deadman release and re-press, the network state equals a fresh `reset()` | keep the state |

A static check, like `gap_follow`'s `test_no_test_here_builds_a_node_on_the_real_drive`, fails if any test in the package builds a node without the `/test_only/` remap.

### On the ROS simulator

Run `racerbot_sim` (with its simulated IMU from P0.7), then `imu_launch.py`, then `neural_racer_launch.py shadow:=false max_speed:=1.0`, on the `indoor_tight` track with `opponents:="6,0.7,0.0"`. Then check:

- it completes laps
- `/drive_intent` paths look right on the dashboard
- the latency test from Phase 2 passes **with the full simulated stack running**

**Gate G6:**

- In the referee, the student's lap 1 is at least 10% faster than `gap_follow` on held-out tracks, with zero contacts.
- Every node test, including the four deadman deny paths, passes and was seen to fail against its break.
- Whole-tick p99 latency is ≤ 5 ms on the Jetson with SLAM and the simulator running. If it fails, switch to `scan_width: half` and retrain; don't relax the budget.
- A 10-lap `racerbot_sim` run completes with no wrapper stop other than braking.

---

## Phase 7: shadow mode on car 1

**Goal:** find the sim-to-real gap at zero risk. The network sees real scans and real IMU data while `gap_follow` drives.

**Run these in order.** Hold LB the whole time: `gap_follow` is the one driving, and it needs LB.

**Terminal 1** — the recorder. Start it first, so nothing is missed:

```bash
ros2 launch race_diagnostics record_run.py
```

**Working when:** it reports the run directory it is writing to ([run-diagnostics.md](../../run-diagnostics.md)).

**Terminal 2** — bringup:

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** the VESC, LiDAR and joystick nodes start and keep running. `ros2 run racerbot_doctor doctor` in another terminal passes.

**Terminal 3** — the IMU conditioner:

```bash
ros2 launch racerbot_launch imu_launch.py
```

**Working when:** `ros2 topic echo /imu/status --once` shows `ok`.

**Terminal 4** — `gap_follow`, driving:

```bash
ros2 launch gap_follow gap_follow_launch.py
```

**Working when:** holding LB, the car drives the course reactively. Release LB at any time to stop it.

**Terminal 5** — the network, in shadow mode:

```bash
ros2 launch neural_racer neural_racer_launch.py model_hash:=<hash> shadow:=true
```

**Working when:** `ros2 topic hz /neural_racer/shadow_drive` reads about 40 Hz, and `ros2 topic echo /drive --once` shows only `gap_follow`'s commands.

**Analyse** with a new `tools/run_analysis/shadow_report.py` over the bag:

- shadow versus `gap_follow` commands, per tick
- the ego-path forecast error against scan-matched motion (`scan_motion.py`)
- the network's opponent forecasts during a run with a second, slow, teleop-driven car, if one is available

**Gate G7:** at least 10 laps recorded. The ego-forecast error at 1.0 s is within 1.5× the error the same model shows in the referee on the room map. Large, structured disagreements, such as one corner always steered differently, are explained before Phase 10.

---

## Phase 8: the league

**Files:** `league_learner.py`, `archive.py`, `matchmaking.py`, `opponents.py`, `scripted.py`.

```python
# archive.py
class Archive:
    def __init__(self, root: Path): ...
    def add(self, role: str, update: int, state_dict: dict, meta: dict) -> str: ...  # atomic: tmp + os.replace
    def entries(self) -> list[ArchiveEntry]: ...      # tolerant of a half-written last line in index.jsonl
    def load(self, entry_id: str) -> dict: ...

# matchmaking.py
def pfsp_weights(win_rates: np.ndarray, kind: Literal["hard", "var"]) -> np.ndarray: ...
def opponent_mix(role: str, archive: Archive, win_rates: dict, rng) -> list[OpponentRef]:
    """Applies the A8 role table: self share, PFSP share, scripted share, uniform floor."""

# opponents.py
class StackedOpponents:          # 16 policies, one vmap call
    def __init__(self, entries: list[dict], model_ctor, device): ...
    def act(self, obs_blocks: dict, hidden: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]: ...
```

**Run:**

```bash
tools/neural_racer/slurm/fir_league_submit.sh --account def-<pi> --run league-v1 \
    --init "$SCRATCH/neural_racer/runs/distill-v1/solo/checkpoints/latest" --segments 6
```

| Test | Oracle | Break it with |
|---|---|---|
| `test_pfsp_hard_closed_form` | win rates [0.9, 0.5, 0.1] → weights ∝ [0.01, 0.25, 0.81] | use p instead of 1 − p |
| `test_floor_keeps_everyone` | an archive of 500 where we beat everyone at 0.99: over 100,000 draws, each member is sampled at least `0.10/500` of the time, ±20% | drop the floor |
| `test_role_mix_shares` | `main0`'s draws match 35/40/15/10 ± 2% | swap two shares |
| `test_archive_atomic` | a writer killed mid-write leaves no partial entry visible | write in place |
| `test_archive_never_deletes` | no code path in `archive.py` deletes; a static check greps for `unlink`/`rmtree` | — |
| `test_stacked_matches_individual` | 16 stacked opponents' actions equal each policy run alone | `atol=1e-5`; break: wrong stacking axis |
| `test_win_rate_estimate` | 3 wins in 4 games → (3 + 1)/(4 + 2) | drop the prior |
| `test_exploiter_reset_rule` | win rate 0.71 triggers a reset; 0.70 does not | `>` → `>=` |
| `test_scripted_share_fades` | closed form `max(0.03, min(0.15, 0.15·(1 − p)/0.5))`: p = 0.5 → 0.15, p = 0.9 → 0.03, p = 0.99 → 0.03, p = 0.2 → 0.15 | drop the floor |
| `test_freed_share_goes_to_pfsp` | at p = 0.9 the role's mix sums to 1, with PFSP at 40% + 12% | give the freed share to self-play |

**Phase 8 also runs** the reward-weight sweep (3 variants of the contact and place weights) and the GRU-versus-MLP ablation, which is `main1_mlp` against `main0`, judged in Phase 9.

**Gate G8:** the league has run its budget; the archive holds snapshots from all four roles; the referee workers' logs show held-out win rates rising against the panel over the run.

**Stop point:** overtakes per race falling steadily while at-fault contact stays at zero. That is the polite-collusion failure; report it before changing the reward.

---

## Phase 9: selection and the referee

**Files:**

- `eval_worker.py`: CPU, polls the archive, exports each new snapshot to NumPy, plays held-out referee races against the panel, and writes `archive/eval/<id>.json`.
- `scripted.py`: the panel's defensive blocker.
- `select.py`: the cross-play matrix, Nash averaging and the veto, writing `selection_report.json`.
- A new `tools/neural_racer/slurm/fir_select.sbatch`: a CPU-only job array, one task per candidate pair.

**Referee races** use `make_plant(track, num_agents, seed)` from `run_validation.py`. The panel's `gap_follow` uses `GapController` from `gap_mirror.py`, and its `pure_pursuit` uses `PathFollower`. Verdicts come from `plant.py`'s collision geometry only.

**Nash averaging:** solve the linear program for the game value v* of the antisymmetric payoff matrix `A = winrate − 0.5`. Then find the maximum-entropy p subject to `Aᵀp ≥ v* − 1e-6` with `scipy.optimize.minimize` (SLSQP). Rank candidates by `A·p`.

| Test | Oracle | Break it with |
|---|---|---|
| `test_nash_rock_paper_scissors` | uniform 1/3 each | — |
| `test_nash_redundant_copies` | adding 10 copies of a weak agent doesn't change the ranking of the others | rank by mean win rate instead |
| `test_veto_any_panel_loss` | a top-ranked candidate at 49% against one panel member is vetoed | `<` → `<=` at exactly 50% |
| `test_veto_any_contact` | one at-fault contact vetoes | count only wall contact |
| `test_referee_uses_plant_contacts` | the verdict changes when `plant.py` reports a contact, and is unaffected by gym's own flag | read gym's flag |
| `test_sequential_stops_when_clear` | Wilson interval, closed form: a matchup won 20 of 20 stops after 20 races against a 0.5 threshold; one at 11 of 20 keeps going | stop on the point estimate |
| `test_sequential_caps_at_200` | spec: a matchup at exactly 50% stops at 200 races and reports "inconclusive", which the veto treats as a loss | treat inconclusive as a win |
| `test_panel_retirement_and_return` | spec: 10 snapshots above 95% → checked every 10th; one failed re-check → every snapshot again | never return |
| `test_release_plays_full_panel` | spec: the release gate plays every panel member, whatever their retirement state | reuse the training-time schedule |

**Gate G9:** a candidate passes every row of the [A2 success table](#a2-what-winning-means-and-how-well-know) in the referee. `approve_model` accepts it.

---

## Phase 10: solo on the floor

The ladder from [writing-your-own-node.md](../../writing-your-own-node.md#testing-before-its-on-wheels). Every run is recorded with `race_diagnostics` and analysed before the next.

1. **Static topic check.** No driver stack running: the node starts, refuses to drive (no `/joy`, no IMU), and publishes nothing but zeros.
2. **Wheels off the ground.** Full stack, LB held, `shadow:=false max_speed:=1.0`. Steering responds sensibly to a hand moved in front of the LiDAR; releasing LB stops the wheels.
3. **Floor, 1.0 m/s, solo, open space.**
4. **Raise `max_speed`** by 0.5 m/s at a time, to at most `pure_pursuit`'s 4.0. Each step needs a clean run: no wrapper stop other than braking, and wrapper interventions under 1% of ticks.

**Gate G10:** clean solo runs at 2.5 m/s in both modes (reactive, then map mode after a manual mapping lap), with lap times recorded against `gap_follow` and `pure_pursuit` on the same course.

---

## Phase 11: auto_map_race and two cars

**`auto_map_race` integration:**

- **New parameter `race_controller`** in `auto_map_race.yaml`: `pure_pursuit` (the default, today's behaviour) or `neural_racer`.
  - With `neural_racer`, the supervisor publishes the cleaned lap path as a latched `nav_msgs/Path` on `/auto_map_race/race_path`. Its existing frozen map and `/slam_pose` already exist.
- **New launch `racerbot_launch/launch/auto_map_race_neural_launch.py`.** It starts `neural_racer` twice, one per role:
  - **`neural_racer_mapping`**: `mode:=reactive`, publishing on `/auto_map/drive`. This replaces `gap_follow` for the mapping lap.
  - **`neural_racer_racing`**: `mode:=map`, publishing on `/auto_race/drive`.
  - The supervisor keeps its own deadman and is still the only node on `/drive`.
- `auto_map_race`'s tests gain cases for `race_controller: neural_racer`. The existing tests must still pass unchanged.

**Two cars:** the team has a second car for low-speed tests (2026-10-23). First in `racerbot_sim` with `opponents:=`, then the two real cars at walking pace, a person holding LB on each car's own controller. Only after G10.

- Start with the second car driven by teleop at a steady 0.8 m/s, then by `gap_follow`, before trying it as a moving neural-racer opponent.
- Each car runs its own bringup and its own deadman. They share no ROS graph: give them different `ROS_DOMAIN_ID`s, so one car's `/drive` can never reach the other.
- The second car can also join Phase 7: a slow teleop-driven car in front of car 1 is the first real test of the opponent forecast.

**Gate G11:**

- A complete `auto_map_race` run in `racerbot_sim` with the neural racer in both roles.
- Then on car 1.
- Then 10 two-car runs at 2.0 m/s with zero at-fault contact and the wrapper under 1%.

---

## Phase C: adding the camera (optional)

**After the first network is trained and released (G9)**, the team's choice (2026-10-23). It doesn't block anything else. These steps build out [A5](#a5-the-camera-an-option-that-is-already-wired-in):

1. **Camera publisher.** Add a V4L2 node (for example `ros-jazzy-v4l2-camera`, apt) publishing `/camera/image_raw` at 640×360, 30 fps. Point `webcam_launch.py`'s `usb_cam_stream` at it with `image_topic:=/camera/image_raw`. **TODO(verify):** that `webcam_launch.py` passes `image_topic` through.
2. **Mount calibration.** Measure the camera's pose on `base_link`, and compute the floor homography with a checkerboard flat on the floor. Record both in `racerbot_launch/config/camera_rb1.yaml`.
3. **Segmentation.** Train a small segmentation network (4 classes) on car-1 images, auto-labelled by projecting LiDAR returns (walls and cars) and the free floor between them into the image. Export it to TensorRT on the Jetson.
4. **`camera_encoder_node`**, in `neural_racer`: grab, segment, project to the 3×32×64 bird's-eye grid, encode with the checkpoint's exported camera encoder, then publish the embedding with its stamp and model hash.
   - It needs a small new message package, `neural_racer_msgs` (`Embedding.msg`: header, `string model_hash`, `float32[] data`).
   - The node publishes no command topic.
5. **Simulator:** `gpu_sim/camera.py` renders the same bird's-eye grid exactly, from the track SDF, car boxes and boundary tape. Add noise that matches the segmentation's measured confusion matrix on held-out real images.
6. **Fine-tune** a released model with `camera: true` in the league. Use camera dropout at 30% of episodes, so the policy never depends on it.
7. **Ablation:** the camera model against its own camera-off parent, in the Phase 9 referee.

**Gate GC:** the camera model beats its parent by a margin that Phase 9's procedure calls significant, and passes every A2 row. **With the camera unplugged mid-run, it still passes the A2 rows.** If it doesn't beat its parent, the camera stays off.

---

## Order of work

```
Phase 0  IMU ready ─────────────────────────────┐
Phase 1  baselines, held-out set ──┐            │
Phase 2  ROS-free core ────────────┤            │
Phase 3  GPU sim + parity ─────────┤            │
Phase 4  tracks + PLR ─────────────┴─► Phase 5  teacher
                                                 │
                                  (G0 required) ─┴─► Phase 6 student + node ─► Phase 7 shadow
                                                                                   │
                                                     Phase 8 league ◄──────────────┘
                                                        │
                                                     Phase 9 selection ─► Phase 10 floor ─► Phase 11 auto_map_race, two cars
                                                     Phase C camera: after G9 (team decision)
```

- **Phase 0 is first.** Phases 1–4 can run alongside it with a second person, because none of them trains a policy for the car.
- **Phase 5's teacher** may use ESTIMATED IMU values, since the teacher never ships.
- **Phase 6 cannot start its training run until G0 passes.**

---

## Risks, and plan B

| Risk | Sign | Response |
|---|---|---|
| **Car 1 has no IMU chip** | P0.1–P0.2 | [decision D5](#decisions-for-you): fit an external IMU. Phase 0 is source-agnostic, so the rest of the plan is unchanged |
| **IMU too delayed** to help control | P0.5 measures over 60 ms | keep the IMU as an input (the simulator models the delay), but expect less from it; report |
| **Sim-to-real gap**, the single biggest risk. Car 1's steering gain was 30% off until 2026-10-21 | shadow commands disagree, or the forecast drifts on real data | fit more parameters from bags; widen randomization for ESTIMATED and STOCK parameters; add real-scan replay to distillation |
| **Reward hacking** | lap time improves in training but the referee shows contact | the referee's check is independent by design; any disagreement is a simulator bug report |
| **League collusion** (everyone polite) or **mutual aggression** | overtakes per race fall; at-fault contact rises | raise the scripted pool's share; strengthen the fault asymmetry |
| **Overfitting to the generator's track style** | generalization gap over 10 points; worse on the real maps | more imported real maps and scaled circuits in training, with the held-out set kept apart |
| **Opponents unlike ours** in shape, size, reflectivity or behaviour | contact or hesitation near unfamiliar cars in two-car tests | widen opponent footprint and dropout randomization; add recorded real behaviours to the scripted pool |
| **Latency over budget** on the car | Phase 6's gate | `scan_width: half` (measured 2.23 ms p99 for the forward pass) |
| **Direct control won't transfer** at Phase 10 | wobble, oscillation, or a high wrapper rate despite a good referee result | **plan B:** same network and training, but the action becomes "lateral offset from the optimized line + speed scale", tracked by `pure_pursuit` with all seven of its safety layers. Lap 1 then stays with `gap_follow` |
| **FIR throughput far below estimate** | Phase 3's benchmark | 2 cars instead of 4, fewer beams in the teacher, a smaller league |

---

## Decisions for you

1. ~~**D1, race format.**~~ **Decided 2026-10-23:** 1 to 4 cars, drawn at random, grid start ([A8](#stage-3-the-league-phase-8)). The contact and passing rules still come from the rulebook TODO in [A2](#a2-what-winning-means-and-how-well-know).
2. ~~**D2, FIR allocation.**~~ **Decided:** a default `def-` allocation. See [A9](#a9-fir-and-gpu-efficiency) for what that changes.
3. ~~**D3, a second car.**~~ **Decided:** a second car is available for low-speed real tests (Phases 7 and 11).
4. **D4, ownership.** Who owns the GPU simulator (Phase 3)? It is the biggest single piece of work, and everything rests on its parity tests.
5. **D5, IMUs.** **Decided 2026-10-23:** two IMUs, the VESC's onboard one and a separate unit at the front bumper. **Still open:** is the front unit wired to the VESC (one IMU readable at a time, so no live cross-check) or to the Jetson (both live)? And what model is it? P0.1 settles both.
6. ~~**D6, the camera.**~~ **Decided:** after the first network is trained (Phase C follows G9).

---

## Sources

- Fir node specifications: [Alliance wiki — Fir](https://docs.alliancecan.ca/wiki/Fir/en)
- League self-play beating a champion human pilot in multi-drone races. The code has fixed opponent pools and `lp` / `lp_mlp` variants. Geles, Bauersfeld, Wulfmeier, Scaramuzza, *Superhuman Safe and Agile Racing through Multi-Agent Reinforcement Learning*, 2026 — [arXiv 2605.22748](https://arxiv.org/abs/2605.22748), [project page](https://rpg.ifi.uzh.ch/marl/), [code](https://github.com/uzh-rpg/marl-racing)
- Self-play with massively parallel simulation and zero-shot sim-to-real on 1/10-scale cars: Werner, Seyde et al., *Dynamic Multi-Team Racing*, CoRL 2023 — [PMLR](https://proceedings.mlr.press/v229/werner23a.html)
- PSRO: Lanctot et al., *A Unified Game-Theoretic Approach to Multiagent RL*, 2017 — [arXiv 1711.00832](https://arxiv.org/abs/1711.00832)
- IMU message conventions: REP-145 (units and frames of `sensor_msgs/Imu`)
- Also relied on, cited from memory, so check before quoting:
  - League training and PFSP: Vinyals et al., *Grandmaster level in StarCraft II*, Nature 2019
  - Racing etiquette and fault-based penalties: Wurman et al., GT Sophy, Nature 2022
  - Robust PLR: Jiang et al., *Replay-Guided Adversarial Environment Design*, NeurIPS 2021
  - ACCEL: Parker-Holder et al., ICML 2022
  - Nash averaging: Balduzzi et al., *Re-evaluating Evaluation*, NeurIPS 2018
  - DAgger: Ross et al., AISTATS 2011
  - Asymmetric actor-critic: Pinto et al., 2017
  - Privileged teacher and student: Chen et al., *Learning by Cheating*, CoRL 2019
- Measured in this workspace: the policy forward-pass latency on car 1's Jetson (2026-10-23, [A4](#size-and-measured-speed-on-the-jetson)); the VESC IMU's raw units, from `src/f1tenth_system/vesc/vesc_driver/src/vesc_packet.cpp` and `vesc_driver.cpp`.
