# Documentation index

> **Who this is for:** everyone. Start here if you don't know which doc you need.
> **Read first:** nothing.
> **What's in it:** a reading order for newcomers, then every doc grouped by what you're trying to do.

This workspace has a lot of documentation. That's deliberate — most of it exists because somebody lost an afternoon to something and wrote it down. But it does mean "just read the docs" is unhelpful advice, so this page tells you which ones, in what order.

---

## Brand new? Read these in order.

Roughly a day's reading and doing, spread over your first week. Don't skip ahead to autonomy — the ladder exists because the car is a physical machine that can hurt someone.

1. **[glossary.md](glossary.md)** — the vocabulary. Skim it now; come back whenever a word bites. Everything else assumes these terms.

2. **[concepts.md](concepts.md)** — what ROS2, a node, a topic, a launch file, and `colcon build` actually are, and why you have to `source` things in every terminal. Skip only if you've used ROS2 before.

3. **[operations.md](operations.md#manual-driving-teleop)** — drive the car by hand. Do this before anything autonomous. It's also the fastest way to make the concepts above concrete.

4. **[architecture.md](architecture.md)** — what talks to what, and the safety model. **Required reading before you write any code that can move the car.**

5. **[adding-your-own-code.md](adding-your-own-code.md)** — where new code goes, and what it must have. Start here when you're ready to build something.

6. **[writing-your-own-node.md](writing-your-own-node.md)** — the full contract for driving code specifically, worked through `gap_follow` as a template.

After that, follow whichever branch below matches what you're doing.

---

## Learn how it works

| Doc | What's in it, and who it's for |
|---|---|
| [glossary.md](glossary.md) | Short definitions of every term the docs use. For anyone who hits an unfamiliar word. |
| [concepts.md](concepts.md) | ROS2, colcon, sourcing, and what each top-level folder is for. For anyone new to ROS2. |
| [architecture.md](architecture.md) | The node/topic graph, the two-layer launch pattern, and the safety model. For anyone about to write or run driving code. |

## Do a thing

| Doc | What's in it, and who it's for |
|---|---|
| [operations.md](operations.md) | Step-by-step procedures: driving, mapping, localizing, running autonomy, racing, shutting down. For anyone actually using the car. |
| [run-diagnostics.md](run-diagnostics.md) | Recording a run so it can be diagnosed afterwards, plus the AI-agent prompt template. For anyone debugging a run that went wrong. |
| [odom-calibration.md](odom-calibration.md) | A browser wizard for tape-measure calibration of VESC speed odometry. For anyone whose distances read wrong. |
| [git-setup.md](git-setup.md) | Which `src/` packages are real submodules vs. vendored code, and how to update them safely. Read before pulling upstream changes into `f1tenth_system`. |

## Write code

| Doc | What's in it, and who it's for |
|---|---|
| [adding-your-own-code.md](adding-your-own-code.md) | Where new packages go and what they're required to have. **Start here for any new code.** |
| [writing-your-own-node.md](writing-your-own-node.md) | The full contract for code that can move the car, including the mandatory deadman pattern. |
| [drive-intent.md](drive-intent.md) | The `/drive_intent` schema — publishing what your algorithm is trying to do and why, without risking the control path. |

## Go deep on a subsystem

| Doc | What's in it, and who it's for |
|---|---|
| [racing-autonomy.md](racing-autonomy.md) | The map-based race stack end to end: SLAM, localization, racing line, velocity profile, pure pursuit, overtaking. The biggest doc here; it opens with a plain-language summary. |
| [web-dashboard.md](web-dashboard.md) | The browser dashboard: what it shows, how it works, and live parameter tuning. Safe to run alongside anything. |
| [simulator.md](simulator.md) | The no-ROS simulator that tests controller *math* directly. |
| [ros-simulator.md](ros-simulator.md) | The same physics behind the real ROS topics, so whole launch files can be validated without the car. Includes the interlock that stops it running beside real hardware. |
| [sim-fidelity-audit.md](sim-fidelity-audit.md) | How closely the simulator matches this physical car, and where it doesn't. Read before trusting a simulator result. |
| [../tools/f1tenth_sim/README.md](../tools/f1tenth_sim/README.md) | Our fidelity layer over stock F1TENTH Gym — this car's real parameters, a friction circle, the real servo, and collision detection that actually fires. What it changed, and what it then found. |

## Look something up

| Doc | What's in it, and who it's for |
|---|---|
| [hardware-reference.md](hardware-reference.md) | VESC, LiDAR, joystick: exact addresses, ports, config values, and gotchas for this specific car. |
| [troubleshooting.md](troubleshooting.md) | Real problems hit during bring-up and how they were diagnosed. Check here when something doesn't work. |

## Cameras and sensors

| Doc | What's in it, and who it's for |
|---|---|
| [realsense-camera.md](realsense-camera.md) | Intel RealSense D435i color/depth over ROS2: install notes, measured performance, and a known IMU limitation on this hardware. |
| [usb-camera-livestream.md](usb-camera-livestream.md) | Live MJPEG video from a USB webcam, viewable in any browser. |
| [realsense-lidar-perception-research-report.md](realsense-lidar-perception-research-report.md) | Design/research report on combining camera and LiDAR perception. Background reading — it changes no driving behavior. |

---

## Documentation that lives elsewhere

- **`src/<package>/README.md`** — code-level docs for each package: module layout, the algorithm as implemented, parameters, and how to test it. Read the `docs/` topic doc for the workflow, the package README for the code.
- **[CLAUDE.md](../CLAUDE.md)** — instructions for AI coding agents working in this repo. Written for an agent, not a person, but a useful summary of the invariants.
- **[CHANGELOG.md](../CHANGELOG.md)** — what changed in the team's own packages, and whether it still needs on-car validation.

## Writing or improving docs

There's a house standard for documentation in this workspace, aimed at readers new to robotics: `.claude/skills/novice-docs/`. It covers the writing standard, a rewrite playbook, and a checker script:

```bash
python3 .claude/skills/novice-docs/scripts/check_docs.py
```

**Working when:** it prints a list of findings grouped by category. Those are a to-do list, not a score — some will be wrong for a given file.
