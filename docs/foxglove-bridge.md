# foxglove_bridge on car 2 (moved)

> **Who this is for:** anyone who followed a link here looking for how car 2 runs foxglove_bridge for Lichtblick.
> **Read first:** nothing.
> **You'll be able to:** find the bridge's real doc, and the car-2 steps that are left here.

The bridge's config, launch file, safety test and boot service moved to **[sfu-racerbot/web-dashboards](https://github.com/sfu-racerbot/web-dashboards)** on 2026-09-27, included here as the `src/web_dashboards` submodule. The full doc — what a Lichtblick user can and cannot do, and why nothing can be published — is **[web-dashboards/car/docs/foxglove-bridge.md](../src/web_dashboards/car/docs/foxglove-bridge.md)**.

**Safety, unchanged: browsers can publish to no topic through the bridge.**

Measured 2026-09-27 on foxglove_bridge 3.5.0: with `clientPublish` on, `client_topic_whitelist` is **not enforced**, and a client's `/drive` reached `ackermann_mux`, skipping every driving node's LB deadman.

So `clientPublish` is off, and `test_client_publishing_is_switched_off_entirely` (now in `src/web_dashboards/car/ros/web_dashboard/test/test_foxglove_bridge_config.py`) fails if it is ever added back.

| What | Where now |
|---|---|
| Config | `src/web_dashboards/car/ros/web_dashboard/config/foxglove_bridge.yaml` |
| Launch | `ros2 launch web_dashboard foxglove_bridge_launch.py` |
| Boot service | `src/web_dashboards/car/systemd/` (`foxglove-bridge.service`, `foxglove-bridge.sh`, `install.sh`) |

**Car 2 specifics** — the tunnel route (`rb2-bridge-origin.sfuracerbot.ca` → `127.0.0.1:8765`) and reinstalling the boot service from the submodule — are in [web-dashboard.md](web-dashboard.md#the-lichtblick-bridges-boot-service).
