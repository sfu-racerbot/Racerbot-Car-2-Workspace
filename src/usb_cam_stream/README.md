# `usb_cam_stream`

> **Who this is for:** someone reading or changing this package's code.
> **Read first:** [docs/usb-camera-livestream.md](../../docs/usb-camera-livestream.md) for setup and the security note.
> **What's in it:** how the MJPEG server works and what it exposes.

Serves a camera as a live MJPEG video stream over plain HTTP — open
`http://<car-ip>:9090/` in any browser to watch, no ROS install or plugins
needed on the viewing device. Two frame sources, one node: a USB webcam
captured directly over V4L2 (the default), or any `sensor_msgs/Image` ROS
topic (`image_topic` mode — how the RealSense D435i's color feed is
streamed, since its V4L2 device is held by `realsense2_camera_node`). Full
write-up (camera recommendations, wire format, security, parameter
reference):
[docs/usb-camera-livestream.md](../../docs/usb-camera-livestream.md).

## Files

| File | What it is |
|---|---|
| [`usb_cam_stream/camera_stream_node.py`](usb_cam_stream/camera_stream_node.py) | The node: frame source (OpenCV/V4L2 capture thread, or an image-topic subscription) + a Tornado web server serving the MJPEG stream. Small enough to keep in one file, same as `gap_follow`. |
| [`config/usb_cam_stream.yaml`](config/usb_cam_stream.yaml) | UVC-webcam variant: device path, resolution, FPS, JPEG quality, host/port. Change behavior here, not in the code. |
| [`config/realsense_stream.yaml`](config/realsense_stream.yaml) | RealSense variant: `image_topic` mode on the D435i's color topic, port 9090 — the port [web_dashboard's camera panel](../../docs/web-dashboard.md) auto-connects to. |
| [`launch/usb_cam_stream_launch.py`](launch/usb_cam_stream_launch.py) | Starts the node with the UVC-webcam YAML. |
| [`launch/realsense_stream_launch.py`](launch/realsense_stream_launch.py) | Standalone RealSense image-topic bridge. Normally included by `racerbot_launch realsense_camera_launch.py`; use it alone only when the driver is already running. |
| [`web/index.html`](web/index.html) | The entire browser side — a single `<img src="/stream">` tag, no JS needed. |
| `resource/usb_cam_stream` | Empty marker file required by `ament_python` — not code. |

## Interface

- **Publishes:** nothing, in either mode.
- **Subscribes:** nothing in the default V4L2 mode (talks to the camera
  directly, no ROS topics at all); only the configured `image_topic`
  (`sensor_msgs/Image`) in topic mode. Either way it never touches
  `/drive` or anything another node reads, so it's safe to run alongside
  any other node in this workspace, at any time.

## Running it

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch usb_cam_stream usb_cam_stream_launch.py       # UVC webcam
# or, for RealSense driver + stream together:
ros2 launch racerbot_launch realsense_camera_launch.py
```
then open `http://<car-ip>:9090/` (port `9090`, not `8080` — that's
already used by `web_dashboard` on this car), or just open the
[web dashboard](../../docs/web-dashboard.md) — its camera panel picks the
stream up automatically.

The launch terminal reports the exact camera state: waiting for a device or
image topic, negotiated V4L2 mode, first-frame recovery, healthy frame count/
age/JPEG size, stale input, lost camera, conversion failure, and JPEG encoding
failure. State changes print immediately; healthy or unchanged faults repeat
every `status_log_period_sec` (default `5.0` s). `frame_timeout_sec` (default
`2.0` s) sets the stale-frame threshold; use a log period of `0.0` for changes
only.
