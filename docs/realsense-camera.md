# RealSense D435i: color + depth over ROS2

Publishes the Intel RealSense D435i's color and depth streams as ROS2
topics, low-res/low-fps so it doesn't compete with the LiDAR/SLAM/
localization/pure_pursuit stack already running on this Jetson. This is a
**pure sensor publisher** — it never touches `/drive`, `/ackermann_cmd`, or
any `/commands/motor|servo/*` topic, so it's support/tooling code per
[adding-your-own-code.md](adding-your-own-code.md): no LB-deadman check, safe
to launch alongside anything else (bringup, teleop, `gap_follow`,
`pure_pursuit`, SLAM) at any time.

```bash
ros2 launch racerbot_launch realsense_camera_launch.py
```

## What's wired up, and what isn't

| Stream | Status |
|---|---|
| Color (`/camera/camera/color/image_raw`) | Working — verified at 424×240@15fps |
| Depth (`/camera/camera/depth/image_rect_raw`) | Working — verified at 424×240@15fps |
| Browser livestream / dashboard camera panel | Working — see [Seeing the feed in a browser](#seeing-the-feed-in-a-browser--the-dashboards-camera-panel) |
| IMU (accel/gyro) | **Not working** — see [Known limitation: no IMU](#known-limitation-no-imu-on-this-backend) below |
| Pointcloud | Off by default — not needed yet, costs CPU. Turn on via `pointcloud.enable` if a future use case needs it |

## How it's installed

Unlike this workspace's other vendored/submoduled drivers (`f1tenth_system`,
`particle_filter`, `range_libc`), `librealsense2` did **not** need a
from-source build here. `ros-jazzy-librealsense2` is a prebuilt `arm64`
package on the ROS apt repo (distinct from Intel's own apt repo, which has no
`arm64` build) — `rosdep install` pulls it in automatically as a dependency
of the `realsense-ros` submodule below. This was confirmed live on this car:
no kernel patching, no `librealsense` source clone needed.

`realsense-ros` itself is a real git submodule (`src/realsense-ros`, branch
`ros2-master` — see [git-setup.md](git-setup.md)), the same pattern as
`particle_filter`/`range_libc`/`transport_drivers`. Unlike those three, its
`ros2-master` branch natively supports Jazzy already, so there's no
"waiting on an upstream `jazzy` branch" concern here.

`src/racerbot_launch/launch/realsense_camera_launch.py` wraps
`realsense2_camera`'s own shipped `rs_launch.py` (same pattern
`race_launch.py` uses for `particle_filter`/`pure_pursuit`), passing this
car's tuning as launch arguments — color+depth at `424x240x15`, pointcloud
off — plus a placeholder `base_link`→`camera_link` static transform.

## Seeing the feed in a browser — the dashboard's camera panel

[web_dashboard](web-dashboard.md)'s bottom-right camera inset auto-connects
to an MJPEG stream on port `9090`. To fill it with the RealSense's color
feed, on top of `realsense_camera_launch.py` above, run:

```bash
ros2 launch usb_cam_stream realsense_stream_launch.py
```

then the dashboard itself. **Full step-by-step (all three launch commands,
in which order, and how to find `<car-ip>`) is in
[web-dashboard.md#with-the-camera-panel-filled-in-too](web-dashboard.md#with-the-camera-panel-filled-in-too)
— including a real gotcha if you're viewing through an editor's
port-forwarding (VS Code, SSH `-L`) instead of the car's actual address,
where the camera panel specifically breaks even though everything is
running correctly**
([web-dashboard.md#finding-the-cars-address-and-viewing-through-a-forwarded-port](web-dashboard.md#finding-the-cars-address-and-viewing-through-a-forwarded-port)).

Under the hood, `realsense_stream_launch.py` runs `usb_cam_stream`'s node
in its `image_topic` mode — subscribing to
`/camera/camera/color/image_raw` and re-serving it as MJPEG — rather than
opening the camera's V4L2 device directly, because `realsense2_camera_node`
holds that device exclusively (a second `cv2.VideoCapture` on it just
fails). Full detail:
[usb-camera-livestream.md](usb-camera-livestream.md#image_topic-mode-streaming-a-ros-image-topic-instead).
The stream is also directly viewable at `http://<car-ip>:9090/` without
the dashboard.

## Known limitation: no IMU on this backend

`librealsense2` logs `No HID info provided, IMU is disabled` on startup, and
no `/camera/camera/accel`/`gyro`/`imu` topic is published, despite
`enable_gyro`/`enable_accel` being set in the launch file.

This was investigated on this car and is **not a udev/permissions problem**
— the D435i's own HID node (`idVendor 8086`) already has correct `plugdev`
group permissions out of the box. The actual cause is that
`ros-jazzy-librealsense2` is built against the **RSUSB (libuvc) backend**
(the only option for `arm64` without a from-source build + kernel metadata
patches), and that backend has documented, known IMU/motion-module gaps on
Jetson — this isn't specific to a misconfiguration here.

**Follow-up, not done**: Intel's "V4L native" backend (kernel-patched,
Intel's documented recommendation for full capability) would need building
`librealsense2` from source with `./scripts/patch-realsense-ubuntu-L4T.sh`
against this car's kernel (`6.8.12-1021-tegra`, L4T R39.2) — Intel's patch
script is only verified up through JetPack 6.0/7.0-beta as of this writing,
so this isn't guaranteed to work and wasn't attempted. If IMU data becomes
genuinely needed (e.g. for visual-inertial work), that's the next thing to
try, on a non-race-critical afternoon.

## Verified performance impact

Launched standalone on this car: `free -h` showed ~1.9GB/7.5GB RAM in use,
`tegrastats` showed single-digit percent CPU per core and no thermal
pressure, at the 424×240@15fps config above. Plenty of headroom left for the
rest of the stack.

## Follow-up: camera mount offset

`realsense_camera_launch.py`'s `static_transform_publisher` currently
publishes `base_link`→`camera_link` at `0,0,0` — a placeholder, not yet
measured, same treatment as `laser_offset_x`/`laser_offset_y` in
[web_dashboard.yaml](../src/web_dashboard/config/web_dashboard.yaml). Update
the `--x`/`--y`/`--z` arguments in that launch file once the camera's
physical mount position relative to `base_link` is measured — see
[hardware-reference.md](hardware-reference.md).

## Parameter reference

All set as launch arguments in
`src/racerbot_launch/launch/realsense_camera_launch.py` (not a separate
YAML — `rs_launch.py` accepts these directly, and this workspace's
`race_launch.py` already uses the same launch-argument pattern rather than a
config file for a wrapped driver package):

| Argument | Value | Meaning |
|---|---|---|
| `enable_color` / `enable_depth` | `true` | Stream enable flags |
| `rgb_camera.color_profile` / `depth_module.depth_profile` | `424,240,15` | Resolution/fps — kept low for the Jetson's sake |
| `enable_gyro` / `enable_accel` | `true` | Requested, but non-functional — see limitation above |
| `unite_imu_method` | `2` (linear_interpolation) | Would combine accel+gyro into one `/camera/camera/imu` topic, if IMU worked |
| `pointcloud.enable` | `false` | Off by default — not needed yet |
| `base_frame_id` | `camera_link` | Root frame of the camera's own TF tree, matched by the static transform above |

## File map

```
src/realsense-ros/                                  # git submodule, ros2-master
src/racerbot_launch/
├── launch/realsense_camera_launch.py                # wraps realsense2_camera's rs_launch.py + static TF
├── package.xml                                       # exec_depend on realsense2_camera
src/usb_cam_stream/
├── launch/realsense_stream_launch.py                # browser MJPEG stream of the color feed (dashboard camera panel)
└── config/realsense_stream.yaml
```
