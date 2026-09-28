# `web_dashboard` (moved)

> **Who this is for:** anyone who followed a link to this package's old location.
> **Read first:** nothing.
> **What's in it:** where the package went, and where car 2's settings for it are.

This folder is no longer a package. `web_dashboard` moved, with its git history, to **[sfu-racerbot/web-dashboards](https://github.com/sfu-racerbot/web-dashboards)** on 2026-09-27. This workspace includes that repo as the `src/web_dashboards` submodule, so the package now builds from **[`src/web_dashboards/car/ros/web_dashboard`](../web_dashboards/car/ros/web_dashboard/README.md)** — same name, same `ros2 run` / `ros2 launch` commands.

- Code walkthrough: [`src/web_dashboards/car/ros/web_dashboard/README.md`](../web_dashboards/car/ros/web_dashboard/README.md)
- Using the dashboard on car 2, and car 2's settings file: [docs/web-dashboard.md](../../docs/web-dashboard.md)
- Car 2's settings: [`src/racerbot_launch/config/web_dashboard_rb2.yaml`](../racerbot_launch/config/web_dashboard_rb2.yaml)

The old browser page (`web/`) was not moved: the site's `apps/simple` replaced it, and the car serves no pages any more.
