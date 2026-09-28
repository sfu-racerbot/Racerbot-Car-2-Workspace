# `usb_cam_stream` (moved)

> **Who this is for:** anyone who followed a link to this package's old location.
> **Read first:** nothing.
> **What's in it:** where the package went, and where car 2's settings for it are.

This folder is no longer a package. `usb_cam_stream` moved, with its git history, to **[sfu-racerbot/web-dashboards](https://github.com/sfu-racerbot/web-dashboards)** on 2026-09-27. This workspace includes that repo as the `src/web_dashboards` submodule, so the package now builds from **[`src/web_dashboards/car/ros/usb_cam_stream`](../web_dashboards/car/ros/usb_cam_stream/README.md)** — same name, same launch commands.

- Car 2's camera (the RealSense, via `image_topic`): [docs/usb-camera-livestream.md](../../docs/usb-camera-livestream.md)
- Car 2's settings: the `usb_cam_stream_node` section of [`src/racerbot_launch/config/web_dashboard_rb2.yaml`](../racerbot_launch/config/web_dashboard_rb2.yaml)
