# `usb_cam_stream`

Captures a USB webcam and serves it as a live MJPEG video stream over
plain HTTP — open `http://<car-ip>:9090/` in any browser to watch, no ROS
install or plugins needed on the viewing device. Full write-up (camera
recommendations, wire format, security, parameter reference):
[docs/usb-camera-livestream.md](../../docs/usb-camera-livestream.md).

## Files

| File | What it is |
|---|---|
| [`usb_cam_stream/camera_stream_node.py`](usb_cam_stream/camera_stream_node.py) | The node: OpenCV/V4L2 capture thread + a Tornado web server serving the MJPEG stream. Small enough to keep in one file, same as `gap_follow`. |
| [`config/usb_cam_stream.yaml`](config/usb_cam_stream.yaml) | Device path, resolution, FPS, JPEG quality, host/port. Change behavior here, not in the code. |
| [`launch/usb_cam_stream_launch.py`](launch/usb_cam_stream_launch.py) | Starts the node with the YAML above as its parameters. |
| [`web/index.html`](web/index.html) | The entire browser side — a single `<img src="/stream">` tag, no JS needed. |
| `resource/usb_cam_stream` | Empty marker file required by `ament_python` — not code. |

## Interface

- **Subscribes / publishes:** none — this node talks directly to the USB
  camera via OpenCV/V4L2, it doesn't go through any ROS topic. It never
  touches `/drive` or anything else, so it's safe to run alongside any
  other node in this workspace, at any time.

## Running it

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch usb_cam_stream usb_cam_stream_launch.py
```
then open `http://<car-ip>:9090/` (port `9090`, not `8080` — that's
already used by `web_dashboard` on this car).
