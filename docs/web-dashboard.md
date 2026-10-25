# Web dashboard on car 2

> **Who this is for:** anyone who wants to watch or tune car 2 from a browser, or change how its dashboard, camera stream or Lichtblick bridge are set up.
> **Read first:** nothing. The full manual — every panel, tuning, measuring, clearing maps, stopping processes, the security note — is **[web-dashboards/car/docs/web-dashboard.md](../src/web_dashboards/car/docs/web-dashboard.md)**.
> **You'll be able to:** check the dashboard's boot service and start the camera on car 2, open them at https://dashboard.sfuracerbot.ca, and know which file holds car 2's settings.
> **Time:** 5 minutes.

The dashboard moved out of this workspace on 2026-09-27. Its code, its docs and the site now live in one repo, **[sfu-racerbot/web-dashboards](https://github.com/sfu-racerbot/web-dashboards)**, which this workspace includes as a git submodule at `src/web_dashboards`. The package names did not change: `web_dashboard` and `usb_cam_stream` build and run exactly as before.

This page is only what is specific to car 2.

## Highlights

- **One address, behind a login.** Open https://dashboard.sfuracerbot.ca, sign in with your team email, pick **Car 2**. The car serves no pages itself any more.
- **Car 2's settings are one file here.** [`src/racerbot_launch/config/web_dashboard_rb2.yaml`](../src/racerbot_launch/config/web_dashboard_rb2.yaml) holds everything that makes the generic packages car 2's: the site, the LiDAR offset, which nodes are tunable and stoppable, where saved runs live, and the RealSense stream. A test checks each value against the file it has to agree with.
- **Always on.** The dashboard's server (`racerbot-dashboard.service`) and the Lichtblick bridge (`foxglove-bridge.service`) start at boot and restart themselves. Car 2 shows up on the site whenever it is powered and online. Only the camera is started by hand, below.

## Start it

**No driving layer is needed, and none of this can move the car**: the dashboard and the camera publish to no topic ([why](../src/web_dashboards/car/docs/web-dashboard.md#why-its-safe-to-start-at-any-time)).

**The dashboard's server is already running.** Since 2026-10-24 it starts at boot as `racerbot-dashboard.service` ([below](#the-dashboards-boot-service)). Check it rather than starting it — **any terminal on the car:**

```bash
systemctl status racerbot-dashboard
```

**Working when:** it says `active (running)`, and the site shows Car 2 as `CAR ONLINE`. **Don't also run `ros2 launch racerbot_launch dashboard_launch.py`** on car 2: the second copy joins the ROS graph, then dies with `Address already in use` on port 8080.

**Terminal 1, from `~/racerbot-ws`** — the RealSense and its MJPEG stream, for the camera inset:

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_launch realsense_camera_launch.py
```

**Working when:** the camera inset on the site fills in within a few seconds.

**Everything but the driving stack, in one command** (LiDAR, RealSense, camera stream — for bench testing): `ros2 launch racerbot_launch dashboard_test_launch.py`. It leaves the dashboard to the boot service; add `dashboard:=true` only on a machine without it.

**With the simulator:** `ros2 launch racerbot_sim sim_auto_map_race_launch.py track:=indoor_wide` — the boot service's dashboard shows it, so leave `dashboard:=` at its default `false` on car 2 ([sim-validation.md](sim-validation.md)).

`http://<car-ip>:8080/` now answers 404 — correct, not a fault. The old page that used to be served there (`serve_static`) was removed; use the site.

## If the site can't reach the car

**Terminal 1, on the car:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 run web_dashboard remote_check --site https://dashboard.sfuracerbot.ca --car rb2
```

**Working when:** it ends with `Everything this car can check is fine`. Then open https://dashboard.sfuracerbot.ca/rb2/check for the site's side. Together they name the hop that fails.

Car 2's tunnel is the `rb2` tunnel in the team's Cloudflare account; its three routes are `rb2-dash-origin.sfuracerbot.ca` → `:8080`, `rb2-bridge-origin.sfuracerbot.ca` → `:8765` and `rb2-cam-origin.sfuracerbot.ca` → `:9090`. Setting one up from scratch: [web-dashboards/car/README.md](../src/web_dashboards/car/README.md).

## Changing car 2's settings

Edit [`web_dashboard_rb2.yaml`](../src/racerbot_launch/config/web_dashboard_rb2.yaml), not the package's own YAML in the submodule: that one holds the generic defaults every team shares. Only the keys you change go in the rb2 file; it is loaded on top.

Then check it before trusting it — **Terminal 1, from `~/racerbot-ws`:**

```bash
python3 -m pytest src/racerbot_launch/test/ -v
```

**Working when:** every test passes. They catch a misspelt key (which ROS would otherwise silently ignore), a LiDAR offset that disagrees with the bringup launches' transform, a site origin that disagrees with the site's own, and a stoppable process or map folder the dashboard would silently drop.

## Live tuning and the contract with the driving nodes

The tuning panel reaches `pure_pursuit_node` and `gap_follow_node` because each advertises a `live_tunable_spec`, and the rb2 YAML names them in `tuning_nodes`.

That agreement — the spec format, who enforces bounds, how "save" rewrites `config/*.yaml` — now spans two repos. It is written down in [web-dashboards/car/docs/web-dashboard.md, "The live_tunable_spec contract"](../src/web_dashboards/car/docs/web-dashboard.md#the-live_tunable_spec-contract).

This workspace's half is tested in `src/gap_follow/test/test_gap_follow_live_tuning.py` and `src/pure_pursuit/test/test_pure_pursuit_live_tuning.py`: each node's real spec must parse with the dashboard's `tuning.parse_spec`, and each node's live config must round-trip through the dashboard's YAML writer. **Change a spec or a config's shape and run those.**

## The dashboard's boot service

`racerbot-dashboard.service` runs `ros2 launch racerbot_launch dashboard_launch.py` at boot, as the `racerbotcar-2` user. It joins the same ROS graph as everything else: domain 0, Fast DDS, exactly like the bridge below. It restarts 5 s after any exit, crash or not. Its template and installer are car 2's own, in [`src/racerbot_launch/systemd/`](../src/racerbot_launch/systemd/). The submodule deliberately ships none, so other teams' cars only run a dashboard when they ask for one.

**Why it's safe to leave running:** it publishes to no ROS topic, so it can never move the car. Its four write paths (live tuning, stopping a driving process, SLAM reset, deleting a saved run) are only reachable through the site, which is behind the team's Cloudflare Access login.

**Day-to-day — any terminal on the car:**

| To | Run |
|---|---|
| see its log | `journalctl -u racerbot-dashboard -f` |
| pick up an edited `web_dashboard_rb2.yaml` | `sudo systemctl restart racerbot-dashboard` |
| stop it for now (it comes back at boot) | `sudo systemctl stop racerbot-dashboard` |
| stop it starting at boot | `sudo systemctl disable --now racerbot-dashboard` |

A rebuild (`colcon build`) doesn't need a reinstall: the service runs the workspace's `install/` space. Restart it after the rebuild so it loads the new code.

**To install or reinstall it** — after moving the workspace, say — **Terminal 1, from `~/racerbot-ws`** (needs sudo):

```bash
sudo src/racerbot_launch/systemd/install.sh racerbotcar-2 /home/racerbotcar-2/racerbot-ws
```

**Working when:** it prints `active (running)`, and `journalctl -u racerbot-dashboard -n 20` shows `Serving the WebSocket (/ws) on port 8080`. **If it doesn't:** `install/racerbot_launch not found` means the workspace isn't built; build it first. A unit that keeps restarting usually means the same — `journalctl` shows the Python error.

`src/racerbot_launch/test/test_dashboard_service.py` checks the unit runs `dashboard_launch.py` (not the bare launch, which allows no site), joins the same graph as the bridge's unit, restarts on any exit, and that `install.sh` fills in every placeholder.

## The Lichtblick bridge's boot service

`foxglove-bridge.service` starts foxglove_bridge at boot. Its unit and wrapper now live in the submodule (`src/web_dashboards/car/systemd/`), and the bridge's launch file in `web_dashboard`. **Browsers can publish to no topic through it** — see [foxglove-bridge.md](foxglove-bridge.md).

It was reinstalled from the submodule on 2026-09-27. To reinstall it again — after moving the workspace, say — **Terminal 1, on the car** (needs sudo):

```bash
sudo ~/racerbot-ws/src/web_dashboards/car/systemd/install.sh racerbotcar-2 /home/racerbotcar-2/racerbot-ws
```

**Working when:** it prints `active (running)`, `systemctl cat foxglove-bridge` shows `ExecStart=/home/racerbotcar-2/racerbot-ws/src/web_dashboards/car/systemd/foxglove-bridge.sh`, and `journalctl -u foxglove-bridge -n 20` shows `Server listening on port 8765`.

(Before the move the unit ran `tools/systemd/foxglove-bridge.sh` → `ros2 launch racerbot_launch foxglove_bridge_launch.py`. Both were removed once the unit was reinstalled; if `systemctl cat foxglove-bridge` still names `tools/systemd`, run the command above.)

## Tests

`web_dashboard` and `usb_cam_stream` are still built and tested by this workspace's colcon, from the submodule. **Run ROS tests on an isolated domain** — node tests start real driving nodes that join whatever graph they can see. They publish their drive commands on `/test_only/drive`, never the real `/drive`, but still put latched topics such as `/racing_line` on the graph, and the running dashboard shows them:

**Terminal 1, from `~/racerbot-ws`:**

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  colcon test --packages-select web_dashboard usb_cam_stream racerbot_launch
colcon test-result --verbose
```

**Working when:** 0 errors, 0 failures. Which runner runs which of the packages' tests: [web-dashboards/car/README.md, "Running the tests"](../src/web_dashboards/car/README.md#running-the-tests).

## Updating the submodule

The submodule is pinned to a commit. To take a newer web-dashboards:

**Terminal 1, from `~/racerbot-ws`:**

```bash
git -C src/web_dashboards fetch origin && git -C src/web_dashboards checkout origin/main
colcon build --symlink-install --packages-select web_dashboard usb_cam_stream
```

**Working when:** the build finishes, the tests above pass, and the site still shows Car 2 online. Then commit the new pointer (`git add src/web_dashboards`). A site whose protocol version moved past the car's shows a red banner — rebuilding and restarting the dashboard fixes it.
