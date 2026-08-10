# Adding your own code

> **Who this is for:** anyone about to add anything new to this workspace — a driving node, a tool, a dashboard.
> **Read first:** [concepts.md](concepts.md), so the words *package*, *node* and *launch file* mean something.
> **You'll be able to:** work out which category your code is in, where it goes, and what it must contain.
> **Time:** about 15 minutes.

Start here before you write anything new — an autonomy [node](glossary.md#node), a dashboard, a logging tool, whatever. (A node is one running program that does one job.)

This page answers three questions: **where does it go, what must it contain, and how do you run it.**

For the deep reference on any one topic it points you somewhere else rather than repeating it. It assumes you know what a [topic](glossary.md#topic) is — the named channel nodes send messages over.

---

## The one question that decides everything

Before anything else, work out which of two categories your code is in. It changes what's required of you, how you test it, and how careful you have to be.

```
                  Will your node publish to
              /drive, /ackermann_cmd, or any of
          /commands/motor/* or /commands/servo/* ?
                            │
              ┌─────────────┴─────────────┐
             YES                          NO
              │                            │
              ▼                            ▼
       DRIVING CODE                SUPPORT / TOOLING
   It can move the car.          It cannot move the car.
              │                            │
   Full contract in              Step 3 skeleton, and
   writing-your-own-node.md      that's the requirement.
   LB deadman is MANDATORY.
              │
   Not sure? ────────────────────────► treat it as DRIVING CODE
```

The topic list above is exact — check it against the topic table in [architecture.md](architecture.md#topic-reference) if you're unsure whether something you're publishing counts.

### Yes: this is driving code

It can move the car, so it must follow the full contract in [writing-your-own-node.md](writing-your-own-node.md), including the **mandatory LB deadman-button check** ([the policy](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)).

Two worked examples exist today: `gap_follow` ([README](../src/gap_follow/README.md)) and `pure_pursuit` ([README](../src/pure_pursuit/README.md)). Both show the required deadman pattern in place.

### No: this is support or tooling code

It only subscribes, or publishes things that can't move the car — visualizations, logs, diagnostics, recorded data. No deadman check, no interface contract. It needs to be a well-formed package (Step 3 below) and to not do anything surprising to topics it doesn't own.

`web_dashboard` is the reference example. It publishes to no topic at all, so none of the driving-code precautions apply to running it. See [web-dashboard.md](web-dashboard.md).

<details>
<summary><b>The one genuinely ambiguous case</b> — the dashboard's live tuning panel, and where the category line actually falls. Worth reading before you build anything that writes back to a driving node.</summary>

`web_dashboard` has one narrow write path: its [live tuning panel](web-dashboard.md#live-parameter-tuning) calls the driving nodes' `set_parameters` service.

That still isn't driving code. It cannot command motion — it can only adjust a node that is *already* driving, within bounds that node enforces on itself, in its own process.

But it's the line worth noticing, and the principle generalizes:

> **Publishing something that *can* move the car puts you in the driving category, no matter how the code is packaged.**

The test isn't "is this a driving algorithm?" It's "can a message I send cause the wheels to turn?" A logging tool that happens to republish `/drive` for convenience is driving code. A path planner that only publishes a visualization is not.

</details>

### If you're genuinely unsure

**Err toward treating it as driving code.**

The deadman check costs you a few lines of boilerplate. Skipping it on something that turns out to touch `/drive` is a real safety gap — a node that can move the car with no way for a human to stop it by letting go. The asymmetry is not close.

---

## Step 2: where it goes

**Every piece of new code is its own package under `src/`.**

```
src/your_package_name/
```

Don't add files into an existing package unless you're genuinely extending that package's own purpose. One package per feature or tool:

- keeps `--packages-select` fast, so your rebuilds stay in seconds rather than minutes
- keeps `package.xml` dependencies honest, since each package declares only what it actually uses
- means someone can delete or disable your thing without touching anything else

Name it for what it does, in `snake_case`, matching the folder and Python-module name. That's ROS2 convention, and `ament_python` requires it.

---

## Step 3: what every package must have

This applies to **both** categories. It's the minimum universal skeleton, and every local package here (`gap_follow`, `pure_pursuit`, `web_dashboard`) follows it:

```
src/your_package_name/
├── package.xml                    # manifest: name, version, dependencies
├── setup.py                       # registers console_script entry points + installed data files
├── setup.cfg                      # boilerplate, points the script installer at the right dir
├── resource/your_package_name     # empty marker file — required by ament_python
├── your_package_name/             # the actual importable Python module
│   ├── __init__.py
│   └── your_node.py                # or protocol.py + your_node.py if you split logic from ROS plumbing
├── launch/
│   └── your_node_launch.py        # starts the node with its config as parameters
└── config/
    └── your_node.yaml              # parameter values — tune behavior here, not in the code
```

See [concepts.md](concepts.md#anatomy-of-a-package) for what each file is for and why `ament_python` needs them. Four things matter more than the rest:

**Declare every dependency in `package.xml`.** Everything you actually import: `rclpy` always, plus `sensor_msgs`, `ackermann_msgs`, `nav_msgs` or `geometry_msgs` as needed, and any system Python library such as `python3-tornado`. `rosdep` and `colcon` read this file to know what to install — an undeclared dependency works on your machine and fails on a fresh clone.

**Use parameters, not hardcoded constants.** Topic names and every tuning knob should be `declare_parameter(...)` calls with sane defaults, loaded from `config/your_node.yaml` at launch. This is what lets someone retarget or tune your node without editing code. Copy the pattern from any existing node's `__init__`.

**Write a [launch file](glossary.md#launch-file), even for a single node.** A launch file is the script that starts your node with its parameters loaded. It's what `ros2 launch your_package_name your_node_launch.py` runs, and it's what resolves your config YAML's installed path via `get_package_share_directory`. Without it, your config lives at a path nobody can predict.

**Pull testable logic out of the ROS plumbing.** If your node has non-trivial logic that doesn't need `rclpy` — math, protocol conversion, parsing — put it in its own plain-Python file with no ROS imports.

> This is the highest-leverage habit in the whole workspace. `pure_pursuit/racing_math.py` and `web_dashboard/protocol.py` both do it, and it means that logic can be unit-tested with no robot, no simulator, and no ROS install at all:
>
> ```bash
> python3 -m pytest src/your_package_name/test/ -v
> ```
>
> On a project where testing normally means finding floor space and a charged battery, this is the difference between testing your math and hoping about it.

---

## Step 4A: driving code — the additional mandatory requirements

Covered in full in [writing-your-own-node.md](writing-your-own-node.md). **Don't skip it.** In short, on top of everything in Step 3:

**Subscribe to the sensors you need, publish to `/drive`.** Take `/scan`, `/odom` or whatever else you need, and publish `ackermann_msgs/AckermannDriveStamped` to `/drive`. That's the entire arbitration contract — `ackermann_mux` takes it from there, and you never touch anything downstream of it directly.

**Implement the LB deadman check.** Subscribe to `/joy`, and refuse to publish a non-zero drive command unless LB is held on a live `/joy` stream. This is copy-paste from `gap_follow_node.py`'s `joy_callback` and `_deadman_engaged` — the exact snippet is in [writing-your-own-node.md](writing-your-own-node.md#the-interface-contract).

**Test in this order, and don't skip ahead:**

1. Static topic check, with no driver stack running
2. Wheels off the ground, with the stack up and LB held
3. Floor, low speed, open space

Full procedure and rationale: [writing-your-own-node.md](writing-your-own-node.md#testing-before-its-on-wheels). The order exists because each step catches a class of bug that the next step would let hurt something.

---

## Step 4B: support/tooling code — what's actually different

Nothing extra is *mandatory* beyond Step 3. Four things are worth keeping in mind:

**Don't touch topics you don't own.** A dashboard, logger, or analysis tool should subscribe only. If you find yourself wanting to publish something, stop and reconsider whether Step 4A actually applies to you.

**Be honest in your `package.xml` description** about whether the node publishes anything at all — see `web_dashboard`'s description field. The next person reading it needs to tell at a glance whether your tool carries any driving risk.

**Network-facing tools need a stated trade-off.** Anything binding a port, like `web_dashboard`'s web server, should default to listening in a way that's safe for a LAN-only debugging tool.

Document that choice explicitly, and never expose it past a trusted network. Follow the reasoning in [web-dashboard.md](web-dashboard.md#security-note).

**Say in your docs that it's safe to start and stop freely.** Support code is independent of the driver stack and of the safety procedures in [operations.md](operations.md), specifically because it can't move the car. Write that down, so nobody wastes an afternoon treating your log viewer with driving-code caution it doesn't need.

---

## Step 5: building and running it

Same for both categories.

**Terminal 1, from `~/racerbot-ws`:**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/racerbot-ws
colcon build --symlink-install --packages-select your_package_name
source install/setup.bash
ros2 launch your_package_name your_node_launch.py
```

**Working when:** the build ends with `Summary: 1 package finished`, and the launch prints your node's startup logs and then stays running. If `ros2 launch` says the package isn't found, you almost certainly skipped `source install/setup.bash` after the build.

A few notes on those commands:

- `--symlink-install` means editing your `.py` files takes effect on the next launch with no rebuild. Only rebuild after changing `package.xml`, `setup.py`, or adding/removing files — see [concepts.md](concepts.md#what-colcon-build-actually-does).
- `--packages-select your_package_name` keeps the build fast while iterating. Drop it to rebuild everything.
- Standalone tests (per Step 3) run with no sourcing and no build at all: `python3 -m pytest src/your_package_name/test/ -v`.

**If it's driving code**, launching it is only part of the procedure. Follow [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node): start the bringup, launch your node as the control layer on top in a second terminal, **don't also launch `teleop_launch.py`**, hold LB, and put the wheels off the ground first.

**If it's support code**, just launch it. `ros2 launch web_dashboard web_dashboard_launch.py` is the entire procedure, on top of anything else already running.

---

## Quick reference: existing packages as examples

| Package | Category | Docs |
|---|---|---|
| `gap_follow` | Driving (reactive) | [src/gap_follow/README.md](../src/gap_follow/README.md) |
| `pure_pursuit` | Driving (map-based) | [src/pure_pursuit/README.md](../src/pure_pursuit/README.md), [racing-autonomy.md](racing-autonomy.md) |
| `web_dashboard` | Support (visualization + live parameter tuning; no `/drive` publisher) | [web-dashboard.md](web-dashboard.md) |

When in doubt, read the one closest to what you're building. Copying an existing package's shape is faster and safer than assembling one from this page.
