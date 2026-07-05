# Adding your own code

Start here if you're about to add anything new to this workspace — an autonomy node, a dashboard, a logging tool, whatever. It answers three questions: where does it go, what does it need to have, and how do you actually run it. For the deeper reference on any one of these topics, this doc points you to the right place rather than repeating it.

## Step 1: decide what kind of code this is

Everything in this workspace falls into one of two categories, and which one you're writing determines what's actually required of it.

**Does your node publish to `/drive`, `/ackermann_cmd`, or any of the `/commands/motor/*` / `/commands/servo/*` topics** (see the topic table in [architecture.md](architecture.md#topic-reference))?

- **Yes → this is driving code.** It can move the car, so it must follow the full contract in [writing-your-own-node.md](writing-your-own-node.md), including the **mandatory LB deadman-button check** (see [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)). `gap_follow` and `pure_pursuit` are the two examples today — see their package-local READMEs ([src/gap_follow/README.md](../src/gap_follow/README.md), [src/pure_pursuit/README.md](../src/pure_pursuit/README.md)) for worked examples of the required deadman pattern.
- **No → this is support/tooling code.** It only subscribes, or publishes things that can't move the car (visualizations, logs, diagnostics, recorded data). No deadman check, no interface contract — just needs to be a well-formed package (below) and shouldn't do anything surprising to topics it doesn't own. `web_dashboard` (a read-only live browser dashboard — see [web-dashboard.md](web-dashboard.md)) is the reference example: it "only ever subscribes, it never publishes anything," and its own docs are explicit that none of the driving-code precautions apply to it.

If you're not sure which category something falls into, err toward treating it as driving code until you've confirmed otherwise — the deadman check costs you a few lines of boilerplate; skipping it on something that turns out to touch `/drive` is a real safety gap (see the incident this policy came out of in [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)).

## Step 2: where it goes

Every piece of new code is its own **package** under `src/` — don't add files into an existing package unless you're genuinely extending that package's own purpose. One package per feature/tool keeps `--packages-select` fast, keeps `package.xml` dependencies honest, and means someone can delete or disable your thing without touching anything else.

```
src/your_package_name/
```

Name it for what it does, snake_case, matching the folder/Python-module name (ROS2 convention, and `ament_python` requires it — see below).

## Step 3: what every package must have (regardless of category)

This is the minimum, universal skeleton — every local package in this workspace (`gap_follow`, `pure_pursuit`, `web_dashboard`) follows it:

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

See [concepts.md](concepts.md#anatomy-of-a-package-using-gap_follow-as-the-example) for what each of these files is actually for and why `ament_python` needs them. Concretely:

- **`package.xml`** — declare every ROS/Python dependency you actually import (`rclpy` always; `sensor_msgs`, `ackermann_msgs`, `nav_msgs`, `geometry_msgs` as needed; a system Python library like `python3-tornado` if you use one — `rosdep`/`colcon` read this to know what to install).
- **Parameters, not hardcoded constants.** Topic names and every tuning knob should be `declare_parameter(...)` calls with sane defaults, loaded from your `config/your_node.yaml` at launch — this is what lets someone retarget or tune your node without editing code. See any existing node's `__init__` for the pattern.
- **A launch file**, even for a single node — this is what `ros2 launch your_package_name your_node_launch.py` actually runs, and it's what resolves your config YAML's installed path via `get_package_share_directory`.
- **Tests where the logic can be tested without ROS.** If your node has any non-trivial logic that doesn't need `rclpy` to run (math, protocol/wire-format conversion, parsing), pull it into its own plain-Python file with no ROS imports, the same way `pure_pursuit/racing_math.py` and `web_dashboard/protocol.py` do — it makes that logic unit-testable in isolation (see each package's `test/` folder) without a robot, simulator, or ROS install.

## Step 4A: driving code — the additional mandatory requirements

Covered in full in [writing-your-own-node.md](writing-your-own-node.md) — don't skip it. In short, on top of everything in Step 3, your node must:
- Subscribe to whatever sensor topics it needs (`/scan`, `/odom`, etc.) and publish `ackermann_msgs/AckermannDriveStamped` to `/drive` — that's the entire arbitration contract; `ackermann_mux` takes it from there and you never touch anything downstream of it directly.
- Implement the **LB deadman check** — subscribe to `/joy`, refuse to publish a non-zero drive command unless LB is held on a live `/joy` stream. This is copy-paste from `gap_follow_node.py`'s `joy_callback`/`_deadman_engaged` — see [writing-your-own-node.md](writing-your-own-node.md#the-interface-contract) for the exact snippet.
- Be tested in the specific order in [writing-your-own-node.md](writing-your-own-node.md#testing-before-its-on-wheels): static topic check with no driver stack running, then wheels-off-ground with the stack up and LB held, then floor at low speed — never skip straight to the floor.

## Step 4B: support/tooling code — what's actually different

Nothing extra is *mandatory* beyond Step 3, but keep these in mind:
- **Don't touch topics you don't own.** A dashboard, logger, or analysis tool should subscribe only — if you ever find yourself wanting to publish something, stop and reconsider whether Step 4A actually applies to you.
- **Be honest in your `package.xml` description** about whether the node publishes anything at all (see `web_dashboard`'s description field) — the next person reading it needs to be able to tell at a glance whether your tool carries any driving risk.
- **Network-facing tools** (anything binding a port, like `web_dashboard`'s web server) should default to listening in a way that's safe for a LAN-only debugging tool, document that trade-off explicitly, and never be exposed past a trusted network — see [web-dashboard.md](web-dashboard.md#security-note) for the reasoning to follow.
- Support code can be started/stopped at any time, independent of the driver stack or safety procedures in [operations.md](operations.md) — that's specifically because it can't move the car. Say so explicitly in your own package's docs so nobody wastes time treating it with driving-code caution it doesn't need.

## Step 5: building and running it

Same for both categories:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/racerbot-ws
colcon build --symlink-install --packages-select your_package_name
source install/setup.bash
ros2 launch your_package_name your_node_launch.py
```

- `--symlink-install` means editing your `.py` files takes effect on the next launch with no rebuild — only rebuild after changing `package.xml`, `setup.py`, or adding/removing files (see [concepts.md](concepts.md#what-colcon-build-actually-does)).
- `--packages-select your_package_name` keeps the build fast while iterating; drop it to rebuild everything.
- If your node has a standalone test file with no ROS dependency (per Step 3), run it directly without even sourcing ROS: `python3 -m pytest src/your_package_name/test/ -v`.
- **Driving code only:** running it for real also means following the procedure in [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node) — start the bringup, launch your node as the control layer on top in a second terminal (don't also launch `teleop_launch.py`), hold LB, wheels off the ground first.
- **Support code:** just launch it — `ros2 launch web_dashboard web_dashboard_launch.py` is the entire procedure, on top of anything else already running.

## Quick reference: existing packages as examples

| Package | Category | Docs |
|---|---|---|
| `gap_follow` | Driving (reactive) | [src/gap_follow/README.md](../src/gap_follow/README.md) |
| `pure_pursuit` | Driving (map-based) | [src/pure_pursuit/README.md](../src/pure_pursuit/README.md), [racing-autonomy.md](racing-autonomy.md) |
| `web_dashboard` | Support (read-only visualization) | [web-dashboard.md](web-dashboard.md) |
