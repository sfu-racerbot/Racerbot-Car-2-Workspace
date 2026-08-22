# Using the Jetson's GPU

> **Who this is for:** anyone wondering whether this car's code should be using CUDA, the tensor cores, or the rest of the Jetson's hardware — and anyone who touches the particle filter.
> **Read first:** [glossary.md](glossary.md) for *particle filter*, *localization*, and *scan*. [localization.md](localization.md) explains what the particle filter is for.
> **What's in it:** a measured audit of every part of this workspace against the GPU, plus the record of one bug it found and fixed.
> **Time:** 15 minutes for the answer; the rest is reference.

This car has a GPU that, until August 2026, had never run a single instruction. This doc is the audit that asked whether that was a waste — measured on the car, not estimated.

The short version: **one job in this workspace ([glossary](glossary.md#workspace) — the `~/racerbot-ws` folder and everything built from it) belongs on the GPU, and it was already broken.** Everything else is better off where it is.

## Contents

- [What changed on the car](#what-changed-on-the-car)
- [Safety](#safety)
- [The short answer](#the-short-answer)
- [What's actually in the Jetson](#whats-actually-in-the-jetson)
- [The rule that decides everything else](#the-rule-that-decides-everything-else)
- [The one job worth moving: localization](#the-one-job-worth-moving-localization)
- [Why the driving code stays on the CPU](#why-the-driving-code-stays-on-the-cpu)
- [What the tensor cores could do: the camera](#what-the-tensor-cores-could-do-the-camera)
- [The small stuff](#the-small-stuff)
- [What to do next, in order](#what-to-do-next-in-order)
- [Troubleshooting](#troubleshooting)

---

## What changed on the car

Two things changed on 2026-08-22. Both are live now.

**1. `range_libc` was rebuilt with CUDA support.**

`range_libc` is the library that answers "if the car were standing *here*, what would the LiDAR see?" — a job called **ray casting**. The [particle filter](glossary.md#particle-filter) asks it that question hundreds of thousands of times per second.

It was installed without GPU support. `particle_filter`'s config asks for the GPU version anyway, which meant the localization [node](glossary.md#node) would **abort with a core dump on its first scan**. That is the bug this audit found.

The rebuild fixes the crash and makes the same work **4.8× faster**.

**2. `race_launch.py` now refuses to start on a broken GPU config.**

`racerbot_launch/race_launch.py` checks `particle_filter`'s settings before launching anything. If the GPU ray caster would crash or return wrong answers, you get an explanation in the terminal instead of a car that misbehaves on track.

<details>
<summary><b>The exact commands used</b> — click to expand. You only need these if you are rebuilding on another car or after a fresh clone.</summary>

**Terminal 1** — from `~/racerbot-ws`. This installs into your user site-packages, not the system.

```bash
source /opt/ros/jazzy/setup.bash
cd src/range_libc/pywrapper
rm -rf build RangeLibc.cpp
WITH_CUDA=ON python3 setup.py install --user
```

**Working when:** the output contains `Compiling with CUDA support` and a series of `ptxas info : Compiling entry function ... for 'sm_87'` lines. `sm_87` is the Orin's GPU architecture — if you see a different number, the build is targeting the wrong hardware.

**If it doesn't:** `nvcc` not found means CUDA isn't on `PATH`; check `/usr/local/cuda/bin/nvcc` exists. The build takes about two minutes.

**Then verify, in the same terminal:**

```bash
cd ~/racerbot-ws
python3 -c "import range_libc; print(range_libc.SHOULD_USE_CUDA)"
```

**Working when:** it prints `True`. If it prints `False`, the build silently fell back to CPU-only and the crash is still there.

`range_libc` is a git submodule with no `package.xml`, so `colcon build` does not touch it. It is installed once, by hand, and survives rebuilds of everything else. See [git-setup.md](git-setup.md) for what that means when updating submodules.

</details>

## Safety

**Re-test localization on the bench before you race it.** The library underneath it was replaced.

The particle filter feeds `pure_pursuit`, which publishes to `/drive`, which moves the car. A localization estimate that is wrong — not missing, *wrong* — steers a car at speed toward where it thinks the racing line is.

Follow the standard order from [writing-your-own-node.md](writing-your-own-node.md#testing-before-its-on-wheels): static topic check with no drivers running, then wheels off the ground with LB held, then floor at low speed in open space. Nothing about this change lets you skip a step.

**Never raise `max_particles` past the limit `race_launch.py` prints.**

Above that limit the GPU ray caster does not fail loudly. It returns **wrong distances**, silently, with one line on stdout. Measured on this car: one particle over the limit made 100% of the returned ranges wrong. Localization would keep running and keep publishing a position that is nonsense.

The limit exists because of a bug in `range_libc` itself, described in [the deep dive below](#deep-dive-the-particle-count-cliff). With the current settings the limit is **4297 particles**, and the config ships 4000.

The [LB deadman](glossary.md#deadman) rule is unchanged and still applies to everything here. Nothing in this doc touches it.

## The short answer

**Yes, for one thing.** The particle filter's ray casting is exactly the shape of work a GPU is built for — hundreds of thousands of identical, independent calculations. It is now 4.8× faster and no longer crashes.

**No, for the driving code.** `gap_follow` uses 2.8% of its time budget and `pure_pursuit` uses 4.3%. Just handing work to the GPU and getting it back costs more than most of those calculations take. Moving them would make the car react *later*.

**Not yet, for the tensor cores.** Nothing in this workspace runs a neural network, so all 32 tensor cores sit at zero. There is no switch to flip — using them means building camera perception that does not exist today. That is a project, and [realsense-lidar-perception-research-report.md](realsense-lidar-perception-research-report.md) already designed it.

## What's actually in the Jetson

Everything in this table was read off the machine on 2026-08-22, not from a spec sheet. Three rows are things people commonly assume are present and are not.

| | What's there | Why it matters |
|---|---|---|
| GPU | Ampere, 8 SMs, compute capability 8.7, 1020 MHz | 1024 CUDA cores, 32 tensor cores |
| CPU | 6× Cortex-A78AE, up to 1.73 GHz | Idles down to 729 MHz between bursts |
| Memory | 7.4 GB, **shared** between CPU and GPU | There is no separate video memory. Anything the GPU holds comes out of the same pool the ROS nodes use, and 3.8 GB is already spoken for. |
| Power mode | `MAXN_SUPER`, already set | Board draws 4.9 W of a 25 W budget at idle |
| GPU load today | `GR3D_FREQ 0%` | Sampled with `tegrastats`. It has never been used. |
| CUDA / cuDNN | 13.2 / 9.20 | Installed and working |
| TensorRT / VPI | 10.16 / 4.1.3 | Installed and working — the right tools for camera work on a Jetson |
| PyTorch, ONNX Runtime, CuPy | **Not installed** | Needed for any learned perception. Ordinary PyPI packages will not work — Jetson builds only. |
| OpenCV CUDA support | **Absent** | The apt build is CPU-only. Every `cv2.cuda` module reports "Unavailable". Do not design around `cv2.cuda`. |
| DLA (deep-learning accelerator) | **Absent** | Bigger Orins have two, letting a network run without touching the GPU. Not here — everything shares one GPU. |
| PVA (vision accelerator) | **Absent** | VPI reports "Failed to open PVA device node". Its CUDA and VIC backends work; PVA-based designs do not. |
| NVENC (video encoder) | **Absent** | No hardware H.264/H.265 anywhere on this board. The camera stream's MJPEG design was the right call, not a compromise. GPU JPEG (nvJPEG) *is* available. |
| Disk | 180 GB free of 233 GB | Room for models and TensorRT engines |

One more, worth knowing before anyone plans a perception project: **Isaac ROS 4.6 is an exact match for this machine.** It targets ROS 2 Jazzy and requires JetPack 7.2 (L4T R39 revision 2.0), which is precisely what this car runs. It ships as apt packages, not Docker-only.

## The rule that decides everything else

A GPU is not simply "faster". It is a machine that does ten thousand identical things at once, and it charges a **fixed fee** every time you hand it a job.

The fee is real work: package up the data, ship it across, wake the GPU, wait, ship the answer back. Measured on this board:

| The fixed cost of using the GPU at all | Measured |
|---|---|
| Start a job and immediately start another, never waiting | 6.2 µs |
| Start a job and wait for the answer | 35.7 µs |
| Send one LiDAR [scan](glossary.md#scan) across and get it back (1080 numbers) | 61 µs |
| Send 240,000 numbers across and get them back | 234 µs |

That gives one working rule for this car:

> **If a job takes the CPU less than about half a millisecond, the GPU will make it slower.** You spend more time on the paperwork than on the work.

Only jobs that are both *big* and *repetitive* pay off. Held against this workspace, almost everything sorts itself out immediately.

| Part of the system | Verdict | Why |
|---|---|---|
| Particle filter ray casting | **Move it** | 244,000 identical ray casts per scan. 4.8× measured. |
| `gap_follow` scan processing | Leave it | 0.70 ms of a 25 ms budget |
| `pure_pursuit` control tick | Leave it | 1.07 ms of a 25 ms budget, and it can move the car |
| Map-subtraction opponent detection | Leave it | A few hundred rays ≈ 40 µs. Already uses the simple CPU method on purpose. |
| Racing line optimiser, [velocity profile](glossary.md#velocity-profile) | Leave it | Runs offline, and the slow part is a step-by-step pass where each answer depends on the previous one — the one shape a GPU cannot help with |
| Dashboard map streaming | Leave it | Read-only, low rate, already compressed cheaply |
| Camera JPEG encoding | Only if you go high-resolution | 0.80 ms per frame at today's size |
| Camera perception (learned) | The real opportunity | Does not exist yet |
| Simulators | Only with a new planner | Off-car, so slowness costs your time, not lap time |

## The one job worth moving: localization

The particle filter works out where the car is by holding **4,000 guesses at once**. For each guess it asks "if I were standing here, what would the LiDAR see?" — 61 rays traced across the saved map — then keeps the guesses whose prediction matches the real scan.

That is 244,000 ray casts, every scan, forty times a second. It is the single largest piece of arithmetic this car does, and every one of those casts is independent of the others.

### What was broken

`src/particle_filter/config/localize.yaml` asks for the GPU ray caster:

```yaml
range_method: 'rmgpu'
```

That setting is correct — the upstream commit it came from is literally titled *"rmgpu works on course car"*. The problem was the **build**, not the config.

`range_libc` on this machine had been compiled without CUDA. The GPU class still exists as a name, so the setting was accepted. Its constructor printed a warning, then returned **without building anything**, leaving an unset pointer behind. The first time the filter asked for ranges, the process died:

```
CANNOT USE RayMarchingGPU - must compile RangeLib with USE_CUDA=1
terminate called after throwing an instance of 'std::__cxx11::basic_string...'
Aborted (core dumped)
```

`racerbot_launch/race_launch.py` — the documented race-day launch — includes `particle_filter`'s `localize_launch.py`, which loads that config.

So localization would have died on its first scan, taking the car's only source of map position with it.

It stayed hidden because the auto-map-then-race flow keeps `slam_toolbox` running through the racing phase instead. [localization.md](localization.md#deep-dive-what-would-improve-this-next) lists switching to the particle filter as a *future* improvement — so this was a landmine waiting for whoever made that switch, not an active fire.

### What the rebuild bought

Same job, same map, the same code path the particle filter actually uses:

| Particles | CPU (ray marching) | CPU (CDDT) | GPU | Speed-up |
|---|---|---|---|---|
| 1,000 | 7.32 ms | 4.17 ms | 2.00 ms | 3.7× |
| 2,000 | 14.34 ms | 8.30 ms | 3.08 ms | 4.7× |
| **4,000** — the shipped setting | **28.86 ms** | 16.56 ms | **5.96 ms** | **4.8×** |

Scans arrive every 25 ms. The CPU number for the shipped setting is **28.86 ms**, which does not fit — the filter would fall behind and start dropping scans. On the GPU the same work takes a quarter of the budget and leaves a CPU core free.

Stripped of the surrounding bookkeeping, the raw ray casting is faster still: **240,000 rays in 3.19 ms on the GPU against 34.05 ms on the CPU, a 10.7× difference.** It returns bit-for-bit identical distances to the CPU version — verified, maximum difference exactly zero. This is not an approximation, it is the same answer computed in parallel.

### Deep dive: the particle-count cliff

> **Skip this unless** you are about to change `max_particles` or `angle_step`, or you are reading `race_launch.py`'s guard and wondering where its numbers came from.

The obvious next thought is "great, let's use 16,000 particles and localize better". **That does not work, and it fails silently.**

<details>
<summary><b>The measurement and the upstream defect</b> — click to expand.</summary>

Beyond a hard limit, the GPU ray caster stops returning correct answers without stopping:

| Particles | Ray queries | Result |
|---|---|---|
| 4,000 | 244,000 | Correct |
| 4,369 | 262,140 | Correct |
| 4,370 | 262,200 | **100% of returned ranges wrong** |
| 8,000 | 480,000 | 54.6% of returned ranges wrong |

*(These figures used 60 rays per particle; the car's Hokuyo gives 61 at `angle_step: 18`, which is why `race_launch.py` computes 4297 rather than 4369.)*

The cause is two defects in the same loop, `numpy_calc_range_angles` in `src/range_libc/includes/RangeLib.h`:

```cpp
int particles_per_iter = std::ceil((float)CHUNK_SIZE / (float)num_angles);
int iters = std::ceil((float)num_particles / (float)particles_per_iter);
for (int i = 0; i < iters; ++i) {
    int num_in_chunk = particles_per_iter;
    if (i == iters - 1) num_in_chunk = num_particles - i*particles_per_iter;
    rmc->numpy_calc_range_angles(&ins[i*num_in_chunk*3], angles,
        &outs[i*num_in_chunk*num_angles], num_in_chunk, num_angles);
}
```

**Defect 1 — the chunk is too big.** The GPU's buffers are allocated to exactly `CHUNK_SIZE` floats — 262,144, fixed at compile time in `range_libc/pywrapper/setup.py`.

But `ceil` rounds the chunk *up*. With 60 rays, `ceil(262144/60)` is 4370 particles, which needs 262,200 slots — more than the buffer holds.

The copy then overruns, CUDA rejects it with `Error: invalid argument` on stdout, and the output array is left holding whatever was in it before. It should be `floor`.

**Defect 2 — the offset is computed from the wrong variable.** `&ins[i*num_in_chunk*3]` uses `num_in_chunk`, which shrinks on the final iteration, where it means `particles_per_iter`. So the last chunk reads from the wrong place. This is why 8,000 particles gives partially rather than wholly wrong output.

Confirmed by measurement. With 64 rays per particle — a number that divides 262,144 exactly — defect 1 never fires, and 4,096 particles comes back correct.

But 4,097 still corrupts 64 values. That is defect 2 on its own.

Both are bugs in upstream `f1tenth/range_libc`, not something this workspace introduced. `range_libc` is a git submodule, so patching it here would leave the fix on this machine only — see [git-setup.md](git-setup.md#updating-an-actual-submodule). Staying inside a single chunk avoids both defects, which is what the guard enforces.

**If you genuinely need more particles**, there are three honest options:

- Raise `CHUNK_SIZE` in `range_libc/pywrapper/setup.py` and rebuild. This costs GPU memory, and it moves the cliff rather than removing it.
- Raise `angle_step`, so each particle uses fewer rays.
- Fix the loop upstream and push it.

</details>

## Why the driving code stays on the CPU

This is the answer people expect to be different, so here are the numbers. Every function in the per-scan and per-tick path, run on a realistic 1080-beam scan and a 1,500-point racing line:

| Called every scan or tick | CPU cost | Share of the 25 ms budget |
|---|---|---|
| `gap_logic.find_best_gap` | 333 µs | 1.3% |
| `gap_logic.time_to_collision` | 105 µs | 0.4% |
| `gap_logic.vehicle_boundary_distances` | 91 µs | 0.4% |
| …and five more | 166 µs | 0.7% |
| **`gap_follow`, whole scan** | **695 µs** | **2.8%** |
| `racing_math.detect_opponent_cluster` | 715 µs | 2.9% |
| `racing_math.detect_dynamic_cluster` | 201 µs | 0.8% |
| `racing_math.find_nearest_index` | 79 µs | 0.3% |
| …and four more | 79 µs | 0.3% |
| **`pure_pursuit`, whole tick** | **1,074 µs** | **4.3%** |

There is nothing to win. The *entire* scan pipeline costs under a millisecond, and the largest single piece — 715 µs — is barely above the point where the GPU's fixed fee eats the gain. Several of these functions cost less than it would take to send their own input across.

Two reasons beyond speed to leave this alone.

**It would make the car react later, not sooner.** Every trip to the GPU adds a wait for a second processor to finish. In a loop whose whole job is to notice a wall and steer away, added delay is the thing you are trying to avoid.

**This is code that can move the car.** [architecture.md](architecture.md) treats anything publishing to `/drive` as needing special care.

Moving it to the GPU adds a CUDA context, a second memory space, and a whole new class of failure — out of GPU memory, driver hiccup, lost context. Putting that inside the deadman-guarded path buys a rounding error and costs a lot of trust.

If the control loop ever *does* need more compute, the answer is not to move today's maths across. It is to change the algorithm to one that genuinely wants ten thousand parallel evaluations — a sampling planner such as MPPI, which rolls out thousands of candidate trajectories per tick. That is a real GPU workload and a real project.

## What the tensor cores could do: the camera

Two things get lumped together and should not be.

**CUDA cores** do ordinary arithmetic in parallel. That is what the localization fix above uses.

**Tensor cores** do one specific thing very fast: the matrix multiplications inside neural networks. This board has 32 of them and is rated at 67 TOPS.

Nothing in this workspace runs a neural network, so all 32 sit permanently idle. **There is no way to switch them on for existing code** — ray casting, gap finding and pure pursuit are not matrix multiplication and never will be. Using them means adding a capability the car does not have.

The natural candidate is already plugged in. The RealSense D435i publishes colour and depth, and [realsense-lidar-perception-research-report.md](realsense-lidar-perception-research-report.md) has already worked out the architecture. What this audit adds is the platform detail:

**Ready:** TensorRT 10.16 and VPI 4.1.3 are installed and working. 180 GB of disk. The GPU is completely idle. Isaac ROS 4.6 matches this machine's ROS distro and JetPack revision exactly.

**Missing, and what it costs:**

- **No PyTorch, ONNX Runtime or CuPy.** These need Jetson-specific builds; ordinary PyPI wheels will not install on aarch64 with CUDA 13.
- **No CUDA in OpenCV.** Use VPI, TensorRT, or plain CUDA. Do not build a design on `cv2.cuda` and find this out later.
- **No DLA.** Everything shares the one GPU.
- **Memory is the real ceiling.** 8 GB shared with every ROS node, 3.8 GB already used. Budget this before choosing a model, not after.
- **The camera is slower than the control loop.** 15 fps is a frame every 66.7 ms against a 25 ms control tick. At 4 m/s the car moves 27 cm between frames before any processing. Camera information must be timestamped and treated as advisory — it can never be a synchronous safety sensor, and the LiDAR stays authoritative.

There is also an open upstream issue about Isaac ROS visual [SLAM](glossary.md#slam) failing to start on an Orin Nano with JetPack 7. Check that before committing to that package in particular; the inference and detection packages are separate.

## The small stuff

Things that were checked and are not worth doing, plus one free win with no GPU involved.

**Camera JPEG encoding — only if you go high-resolution.** Measured on this board: 0.80 ms per frame at the 424×240 preview size (2.4% of one core at 30 fps), 7.57 ms at 720p, and 17.07 ms at 1080p (51% of one core).

At the size actually streamed today this is not worth touching. If a 1080p tier ever appears, nvJPEG is installed and is the answer.

Switching the stream to H.264 is *not* an option — this board has no hardware video encoder at all.

**Simulators — only with a new planner.** Both run off the car, so slowness costs your time rather than lap time. A GPU could simulate many cars or many candidate trajectories at once, but that only pays off with a planner that needs thousands of rollouts.

**Lock the CPU clocks — free, and nothing to do with the GPU.** `tegrastats` shows the cores idling at 729 MHz and stepping up to 1.7 GHz under load. That ramp-up is not instant, and it lands as jitter on a control loop that wants to be metronomic.

**Terminal 1** — run this before a session where timing matters.

```bash
sudo jetson_clocks
```

**Working when:** `sudo jetson_clocks --show` reports the CPU and GPU at their maximum frequencies. Nothing else visibly changes.

**If it doesn't:** the setting does not survive a reboot; re-run it each session. It costs power and heat — the board draws 4.9 W of its 25 W budget at idle, so there is room, but check temperatures if you run long sessions. Measure the benefit with [run-diagnostics.md](run-diagnostics.md) before and after rather than assuming it helped.

## What to do next, in order

1. **Re-validate localization on the bench.** The library underneath it was replaced. Follow the test order in [Safety](#safety) above — this is the only item here that is not optional.
2. **Decide the particle count deliberately.** The config ships 4000 and the ceiling is 4297. You now have a real reason to pick a number rather than inherit one.
3. **Consider handing the racing phase to the particle filter.** [localization.md](localization.md#deep-dive-what-would-improve-this-next) has wanted this for a while, and the cost argument against it just got 4.8× weaker.
4. **Only then, look at the camera.** Install the Jetson builds of ONNX Runtime or the Isaac ROS packages, and start with something purely advisory — a node that publishes what it thinks it sees and touches no drive command at all. Prove it out as a read-only observer first.

And one thing not to do, at any point: **do not put the GPU inside the 40 Hz control path.** The measurements say there is nothing to gain, and the safety model says there is a lot to lose.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `race_launch.py` exits with "built WITHOUT CUDA" | `range_libc` was reinstalled without the `WITH_CUDA=ON` flag, or this is a fresh clone | Rebuild — see [What changed on the car](#what-changed-on-the-car) |
| `race_launch.py` exits complaining about `max_particles` | `localize.yaml` was edited past the GPU's safe limit | Lower `max_particles` to the number the message prints, or raise `angle_step` |
| Localization node dies instantly with `Aborted (core dumped)` | The old CPU-only `range_libc` is still installed and `rmgpu` is set | Rebuild with CUDA, then confirm `range_libc.SHOULD_USE_CUDA` prints `True` |
| `Error: invalid argument` repeating on stdout | The particle count is over the limit and ranges are being silently corrupted | Stop the car. Lower `max_particles`. Do not race until the message is gone. |
| `import range_libc` fails with `No module named 'nav_msgs'` | ROS isn't sourced in that terminal | `source /opt/ros/jazzy/setup.bash` first |
| `python3 setup.py install --user` can't find `nvcc` | CUDA isn't on `PATH` | Check `/usr/local/cuda/bin/nvcc` exists; the build script looks for `/usr/local/cuda` |

## See also

- [localization.md](localization.md) — what the particle filter is for, and why the car's position estimate used to arrive late
- [racing-autonomy.md](racing-autonomy.md) — the race stack this sits underneath
- [realsense-lidar-perception-research-report.md](realsense-lidar-perception-research-report.md) — the camera perception design this audit points at
- [git-setup.md](git-setup.md) — why `range_libc` is a submodule and what that means for patching it
- [hardware-reference.md](hardware-reference.md) — the rest of this car's hardware specifics
