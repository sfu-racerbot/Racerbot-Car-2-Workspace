# Camera livestream on car 2 (moved)

> **Who this is for:** anyone who followed a link here looking for `usb_cam_stream`, the camera's MJPEG stream.
> **Read first:** nothing.
> **You'll be able to:** find the stream's real doc, and start car 2's camera.

`usb_cam_stream` moved to **[sfu-racerbot/web-dashboards](https://github.com/sfu-racerbot/web-dashboards)** on 2026-09-27, included here as the `src/web_dashboards` submodule. The package name did not change. The full doc — cameras, the two stream tiers, `image_topic` mode, every parameter, the security note — is **[web-dashboards/car/docs/usb-camera-livestream.md](../src/web_dashboards/car/docs/usb-camera-livestream.md)**.

**Car 2** streams its RealSense D435i's colour topic, not a webcam. Its settings (`image_topic`, `passthrough: false`) are in the `usb_cam_stream_node` section of [`src/racerbot_launch/config/web_dashboard_rb2.yaml`](../src/racerbot_launch/config/web_dashboard_rb2.yaml); the package's own YAML holds generic webcam defaults. (`realsense_stream_launch.py` and `realsense_stream.yaml` are gone: that was car-2 config.)

**Terminal 1, from `~/racerbot-ws`** — the RealSense driver and the stream together:

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_launch realsense_camera_launch.py
```

**Working when:** the camera inset on https://dashboard.sfuracerbot.ca fills in within a few seconds, or `http://<car-ip>:9090/` shows the picture on the car's own network.

It publishes nothing and cannot move the car. More car-2 detail: [web-dashboard.md](web-dashboard.md) and [realsense-camera.md](realsense-camera.md).
