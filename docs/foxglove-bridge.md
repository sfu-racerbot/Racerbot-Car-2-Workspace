# foxglove_bridge: the car's ROS graph in Lichtblick

> **Who this is for:** anyone who wants to look at the car's topics, parameters and services from a browser through dashboard.sfuracerbot.ca, or who is about to change what the bridge allows.
> **Read first:** [concepts.md](concepts.md) for what a [node](glossary.md#node), [topic](glossary.md#topic) and parameter are, and [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car) for the LB deadman rule.
> **You'll be able to:** start the bridge, say exactly what a Lichtblick user can and cannot do to the car, and explain why publishing is limited to one topic.
> **Time:** about 15 minutes.

**foxglove_bridge** is a ready-made ROS2 program that lets a web app see the car's ROS graph over one WebSocket. The web app here is **Lichtblick**, an open-source robotics viewer (a fork of Foxglove Studio). Through the bridge it can plot any topic, draw the map and scan in 3D, read and change parameters, and call services, all from a browser.

It is installed from apt, not written by us. This page covers how this car runs it and what it lets people do.

---

## Highlights

- **The whole graph, no ROS on the viewing machine.** Every topic, every node's parameters and every service, in a browser tab, through dashboard.sfuracerbot.ca.
- **Loopback only.** It listens on `127.0.0.1:8765`. It cannot be reached on the LAN or Tailscale at all. The only way in from off the car is the Cloudflare Tunnel, which sits behind Cloudflare Access.
- **One topic can be published, and it is not a driving topic.** `/initialpose`, the "2D Pose Estimate" that seeds localization. `/drive`, `/teleop`, `/ackermann_cmd`, `/joy` and `/commands/*` are refused. A test holds that line.
- **No raw camera frames.** Raw images would cost about 7 MB/s per viewer through the tunnel, so only the compressed versions are offered.
- **Every change it makes is recorded.** Parameter changes land on `/parameter_events`, which [race_diagnostics](run-diagnostics.md) now bags by default.

**Honest limits:** it is much less fenced-in than the [web dashboard](web-dashboard.md). A Lichtblick user can change any parameter on any node and call any service. See [What it exposes](#what-it-exposes). The config was checked against foxglove_bridge 3.5.0 only.

### Why it exists

The web dashboard shows what the team decided to show. Lichtblick shows everything, which is what you want when the question is "what is on `/diagnostics` right now" or "what did `particle_filter`'s parameters end up as". Before this, answering those needed a laptop with ROS installed on the car's network.

---

## Before you start

- [ ] foxglove_bridge is installed: `ls /opt/ros/jazzy/share/foxglove_bridge` lists files. If it doesn't, install it (needs sudo, ask whoever administers the car):

  ```bash
  sudo apt install ros-jazzy-foxglove-bridge
  ```

- [ ] For camera images in Lichtblick: the compressed image transports are installed (`ls /opt/ros/jazzy/share/compressed_image_transport`). Without them the camera publishes only raw frames, which the bridge deliberately hides. The [MJPEG camera stream](usb-camera-livestream.md) is unaffected either way.

  ```bash
  sudo apt install ros-jazzy-image-transport-plugins
  ```

- [ ] The workspace is built, so `racerbot_launch` has the launch file: `colcon build --symlink-install --packages-select racerbot_launch`.

---

## Start it

It is support tooling, like the web dashboard. It starts nothing that moves the car, so there is no bringup order and no wheels-off precaution for *starting* it.

**Terminal 1, from `~/racerbot-ws`** — the bridge. Leave it running.

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_launch foxglove_bridge_launch.py
```

**Working when:** the log says `Server listening on port 8765`, and a second terminal shows it listening on loopback only:

```bash
ss -tlnp | grep 8765
```

That should print one line containing `127.0.0.1:8765`. `0.0.0.0:8765` or `*:8765` means it is using some other parameter file — stop it and check.

**If it doesn't:** `package 'foxglove_bridge' not found` means it isn't installed (see [Before you start](#before-you-start)).

Then open dashboard.sfuracerbot.ca, sign in, and pick Lichtblick. It connects through `rb2-bridge-origin.sfuracerbot.ca`.

### Running it at boot

`tools/systemd/foxglove-bridge.service` runs the same launch at boot, as the `racerbotcar-2` user, through `tools/systemd/foxglove-bridge.sh`. That wrapper sources `/opt/ros/jazzy` and `~/racerbot-ws/install` first, because systemd starts services with an almost empty environment.

It is **not installed or enabled** by anything in the repo. To install it (needs sudo):

```bash
sudo cp ~/racerbot-ws/tools/systemd/foxglove-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now foxglove-bridge.service
```

**Working when:** `systemctl status foxglove-bridge` says `active (running)`, and `journalctl -u foxglove-bridge -n 20` shows `Server listening on port 8765`.

To stop it for good: `sudo systemctl disable --now foxglove-bridge.service`.

> **The ROS domain must match the stack's.** The car runs with `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` both unset (domain 0, Fast DDS). The wrapper leaves them unset too. A bridge on a different domain starts fine and shows an empty topic list — nothing tells you why. If the team ever sets either variable for the stack, put the same line in `/etc/default/foxglove-bridge`, which the service reads.

---

## What it exposes

Everything is set in `src/racerbot_launch/config/foxglove_bridge.yaml`, which explains each line.

| A Lichtblick user can… | Scope |
|---|---|
| Subscribe to topics | All of them, **except** raw camera and raw depth images (see below) |
| Publish to topics | **`/initialpose` only** |
| Read parameters | Every node |
| **Change** parameters | **Every node** |
| Call services | **Every service** |
| See the connection graph | Which node publishes and subscribes what |

Read the bold rows twice. They are much broader than the web dashboard's [tuning panel](web-dashboard.md#live-parameter-tuning), which only reaches the two driving nodes, only the parameters those nodes advertise as tunable, within bounds each node enforces, behind a per-tab arm.

Through the bridge, someone can change a parameter on `ackermann_mux`, the VESC driver, `slam_toolbox`, or anything else, and call `/slam_toolbox/reset` while a controller is driving — which the dashboard refuses. What still protects the car:

- The driving nodes refuse `enable_deadman: false` at runtime, from any client, so the LB deadman cannot be switched off this way.
- Nobody can *publish* a drive command (next section).
- Only people Cloudflare Access lets through the site can reach it at all.

So treat access to Lichtblick as "may reconfigure the running car", and keep the Access list to people you'd trust at the laptop.

### Why client publishing is limited to `/initialpose`

**The bridge must never be allowed to publish `/drive`, `/teleop`, `/ackermann_cmd`, `/joy` or `/commands/motor|servo/*`.**

A message published straight onto `/drive` goes to `ackermann_mux`, which forwards it to the motors. It never passes through a driving node. And the driving nodes are where the LB deadman lives: each one checks that someone is holding LB before it publishes. A raw `/drive` from a browser skips all of that.

The result would be a car driven from a web page with no dead-man switch — exactly the state the [workspace policy](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car) exists to make impossible. Publishing `/joy` is the same hazard by a different route: it would forge the LB button itself.

`/initialpose` is safe to allow. It tells [particle_filter](glossary.md#localization) roughly where the car is, the same thing RViz's "2D Pose Estimate" does. It moves nothing.

`src/racerbot_launch/test/test_foxglove_bridge_config.py` fails if the rule is widened to any of those topics, or loosened in a way that would let one through (dropping the `^`…`$` anchors, say).

### Why no raw images

One raw 424×240 color frame is about 300 kB, and the camera sends 15 a second; depth is similar. That is about 7 MB/s through the tunnel for a single viewer, before the map and scan. The compressed versions (`…/image_raw/compressed`, `…/compressedDepth`) are a small fraction of that and stay available.

The rule is one regular expression in `topic_whitelist`. It hides any topic whose last part is `image_raw`, `image_rect_raw`, `image`, and a few similar names, plus the `theora` and `zstd` transports.

<details>
<summary><b>How the parameter names were checked</b> — skip unless you're upgrading the bridge.</summary>

The parameter names in the YAML were read from the installed version's own files, not from web docs for other versions: `foxglove_bridge_launch.xml` and the declared strings in `libfoxglove_bridge_component.so` from `ros-jazzy-foxglove-bridge` 3.5.0 (2026-09-26). That version has 31 parameters, including `remote_access`, which is Foxglove's own cloud relay and is set `false` here.

After an upgrade, compare with:

```bash
ros2 param list /foxglove_bridge
```

A renamed parameter in the YAML is silently ignored, and the bridge falls back to its default. For `client_topic_whitelist` that default is `.*` — every topic, `/drive` included.

</details>

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Lichtblick connects but shows no topics | Bridge on a different `ROS_DOMAIN_ID` from the stack | Make both the same; see [Running it at boot](#running-it-at-boot) |
| Lichtblick can't connect at all | Bridge not running, or the tunnel's `rb2-bridge-origin` hostname points somewhere else | `ss -tlnp \| grep 8765`; check the tunnel's public hostname entry |
| No camera in Lichtblick | Only raw image topics exist, and they are hidden on purpose | Install `ros-jazzy-image-transport-plugins` (see [Before you start](#before-you-start)) |
| "Publishing not allowed" on a topic | Working as intended — only `/initialpose` is allowed | See [why](#why-client-publishing-is-limited-to-initialpose) |

## See also

- [web-dashboard.md](web-dashboard.md#remote-access-through-dashboardsfuracerbotca) — the rest of the remote site, and how the tunnel hostnames fit together
- [run-diagnostics.md](run-diagnostics.md#why-a-rosbag) — where parameter changes made through the bridge get recorded
