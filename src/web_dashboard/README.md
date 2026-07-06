# `web_dashboard`

Live browser dashboard: streams the SLAM/localization map, the raw LIDAR
scan, and the car's pose to any web browser on the network over a
WebSocket, rendered on an HTML5 canvas. This file documents the code in
detail; for the workflow (what you'll see at each stage, quick start,
security note) see [docs/web-dashboard.md](../../docs/web-dashboard.md).

**Not an autonomy node** — it only ever subscribes, never publishes, so
none of [architecture.md](../../docs/architecture.md)'s safety model or
the [mandatory LB-deadman policy](../../docs/architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)
apply to it; both are scoped to nodes that can move the car (see
[writing-your-own-node.md](../../docs/writing-your-own-node.md#the-interface-contract)).

## Files

| File | What it is |
|---|---|
| [`web_dashboard/protocol.py`](web_dashboard/protocol.py) | Wire-format conversion — turns ROS messages into JSON headers + binary payloads. No `rclpy`/Tornado/network imports, so it's unit-testable in isolation (see [`test/test_protocol.py`](test/test_protocol.py)). |
| [`web_dashboard/dashboard_node.py`](web_dashboard/dashboard_node.py) | The ROS2 node: subscribes to map/scan/pose, runs a [Tornado](https://www.tornadoweb.org/) web + WebSocket server, bridges rclpy's executor thread to Tornado's IOLoop thread. |
| [`web/index.html`](web/index.html), [`web/dashboard.js`](web/dashboard.js), [`web/style.css`](web/style.css) | The browser side — plain HTML/JS/CSS, no build step, no framework. |
| [`config/web_dashboard.yaml`](config/web_dashboard.yaml) | Every parameter, loaded at launch. |
| [`launch/web_dashboard_launch.py`](launch/web_dashboard_launch.py) | Starts the node with the YAML above. |

## Interface

- **Subscribes:** `<map_topic>` (`nav_msgs/OccupancyGrid`, default `/map`, transient-local QoS), `<scan_topic>` (`sensor_msgs/LaserScan`, default `/scan`, sensor QoS), `<pose_topic>` (`geometry_msgs/PoseStamped`, default `/pf/viz/inferred_pose`), `<drive_topic>` (`ackermann_msgs/AckermannDriveStamped`, default `/drive`, read-only — for display only)
- Also samples CPU%/mem%/CPU temp/WiFi signal/uptime on a timer (`psutil` + `/sys/class/thermal` + `/proc/net/wireless`), not from a topic.
- **Publishes:** nothing, to `/drive` or anywhere else. No `/joy` subscription, no deadman check — there is nothing this node could do to move the car even by accident; reading `/drive` only feeds the browser's `drive` readout.

## Two concurrency models, one process

rclpy's executor (which calls `map_callback`/`scan_callback`/`pose_callback`)
and Tornado's IOLoop (which runs the web server and every WebSocket
connection) don't share a thread by default. `main()` spins rclpy on a
background thread and lets Tornado's IOLoop own the main thread:

```python
ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
ros_thread.start()

app = node.make_app()
app.listen(node.port, address=node.host)
node._loop = tornado.ioloop.IOLoop.current()
node._loop.start()
```

Tornado documents `IOLoop.add_callback()` as safe to call from any thread,
specifically to hand work back onto the IOLoop's own thread — so every
subscription callback uses it instead of ever touching a WebSocket
directly:

```python
def _broadcast(self, header, binary_payload=None):
    if self._loop is None:
        return
    self._loop.add_callback(functools.partial(self._send_to_all, header, binary_payload))
```

`_send_to_all` (which actually calls `client.write_message(...)`) only
ever runs on the IOLoop thread as a result — the one place Tornado
guarantees it's safe to do so. This is a reusable pattern any time you
need to bridge `rclpy` to an `asyncio`-based library.

One naming gotcha hit while building this: `rclpy.node.Node` already
defines a **read-only** `clients` property (service clients created via
`create_client`) — assigning `self.clients = set()` in a subclass raises
`AttributeError: property 'clients' has no setter`. This node's
WebSocket-client set is named `ws_clients` to avoid the collision. (Other
reserved `Node` properties worth knowing about: `context`,
`default_callback_group`, `executor`, `guards`, `handle`, `publishers`,
`services`, `subscriptions`, `timers`, `waitables`.)

## The wire protocol (`protocol.py`)

Sending a large occupancy grid as a JSON array of numbers would be huge
and slow to parse. Instead, every update is **one JSON text message**
(metadata), immediately followed by **one binary message** (the raw
payload), laid out to match a JavaScript `TypedArray` byte-for-byte:

| Update | JSON header fields | Binary payload |
|---|---|---|
| `map` | `width`, `height`, `resolution`, `origin_x`, `origin_y`, `origin_yaw` | `Int8Array` — one signed byte per cell, matching `OccupancyGrid.data` exactly (`-1` unknown, `0` free, `100` occupied) |
| `scan` | `angle_min`, `angle_increment`, `range_min`, `range_max`, `count`, `laser_offset_x`, `laser_offset_y` | `Float32Array` — one little-endian float per beam, matching `LaserScan.ranges` |
| `pose` | `x`, `y`, `yaw` | *(none — small enough to just be JSON)* |
| `drive` | `speed`, `steering_angle` | *(none)* |
| `stats` | `cpu_percent`, `mem_percent`, `cpu_temp_c` (nullable), `uptime_s`, `wifi_dbm` (nullable) | *(none)* |

```python
def map_cells(msg) -> bytes:
    data = list(msg.data)
    return struct.pack(f'<{len(data)}b', *data)
```

`struct.pack`'s signed-char format (`b`) is what makes a cell value of
`-1` round-trip correctly as the single byte `0xFF` — plain `bytes(data)`
can't do this (it only accepts values `0`-`255`). The browser then reads
it with zero parsing beyond `new Int8Array(arrayBuffer)`.

`dashboard_node.py` throttles `scan` broadcasts to `scan_broadcast_rate_hz`
(default `10Hz`) regardless of how fast `/scan` itself publishes (~40Hz) —
no browser needs to redraw that often, and it keeps WiFi/CPU load down.
`map`, `pose`, and `drive` updates are broadcast immediately, with no
throttling (maps update rarely; poses and drive commands are tiny).
`stats` isn't event-driven at all — it's sampled on its own timer
(`stats_interval_sec`, default 1Hz) since there's no ROS topic to hang it
off of. Both `_read_cpu_temp_c()` and `_read_wifi_signal_dbm()` read
straight from the kernel (`/sys/class/thermal/thermal_zone*`,
`/proc/net/wireless` — the same source `iwconfig`/`nmcli` use) rather than
adding a dependency for something this simple, and both return `None`
instead of raising if the expected file/interface isn't there (e.g.
developing on a laptop with no `cpu-thermal` zone, or docked over
Ethernet only) — system stats should degrade gracefully, not crash the
node.

### QoS notes

`/map` is subscribed with **transient-local** durability, matching what
`nav2_map_server` and `slam_toolbox` both publish with — a *volatile*
(default) subscription would silently miss any map published before this
node started. `/scan` uses `qos_profile_sensor_data` (best-effort): a
best-effort subscriber can match either a best-effort *or* reliable
publisher, which is the broadly-compatible choice when you don't control
the publisher's exact QoS.

## The browser side (`web/dashboard.js`)

One plain file, no build step, no framework. Renders in one of two modes,
chosen automatically based on what data has arrived:

- **Map-relative** (a pose has been received): the map is drawn as a
  background image in true world coordinates, the car is drawn at its
  actual localized position/heading, and LIDAR points are transformed
  through the car's pose (plus the LIDAR's mounting offset from
  `base_link`) into the same world frame — so everything is directly,
  correctly comparable.
- **Robot-centric** (no pose yet, e.g. no `particle_filter` running): the
  car is drawn at the canvas center (offset by `view.bodyPanX/bodyPanY`
  once the user drags) always facing "up", and LIDAR points are drawn
  straight from the scan's own body-frame angles. No map, no pose, no
  localization needed — this is "what the car is seeing" in the most
  literal sense, and it's what you get from just `/scan` alone.

The two modes use different coordinate transforms (`bodyToCanvas` vs
`worldToCanvas`), each with its own pan/zoom state (`bodyPanX/bodyPanY` vs
`centerX/centerY`) — a drag before localization has no meaningful
world-frame equivalent, so the two are tracked independently rather than
sharing one, which also means the view doesn't jump the instant a pose
first arrives mid-drag.

If a map has arrived but no pose has (localization not yet seeded with
RViz's "2D Pose Estimate"), the scan is deliberately **not drawn at all**
rather than guessed — plotting LIDAR points without knowing the car's
position would just be a guess dressed up as data. A banner explains why.

The car is rendered as a small top-down silhouette (`drawCarIcon`) —
rounded body, four wheels, a lighter "windshield" stripe near the front —
rather than a bare arrow, so which end is the front is obvious at a
glance; the un-rotated icon points along local +X (canvas right), which
is why `drawCarRobotCentric` passes `-Math.PI/2` (bodyToCanvas renders
forward as canvas "up", not "right"). A translucent wedge
(`drawBlindSpotRobotCentric`/`drawBlindSpotMapRelative`) marks the arc the
LIDAR never physically scans, computed from the scan's own
`angle_min`/`angle_increment`/count rather than from which beams happen to
read "no return" in a given frame — the latter would be indistinguishable
from open space with nothing in range.

The occupancy grid is rendered into an off-screen canvas once per map
update (not once per frame) and scaled onto the visible canvas with one
`drawImage()` call — redrawing every cell every frame would be needlessly
slow for a large grid. `OccupancyGrid.data` has row 0 at the map's
*bottom* (smallest world Y); a `<canvas>` image has row 0 at the *top* —
`applyMap()` flips rows once, at update time, so every other place in the
file can treat "top of the image" as "largest world Y" without
re-deriving that.

Every one of map/scan/pose/drive/stats carries its own `receivedAt` (this
browser's own clock via `performance.now()`, not the server's), and a
250ms timer recomputes "updated Xs ago" and turns the relevant readout red
past a staleness threshold even if nothing new ever arrives again — so a
frozen feed is visibly reported as stale instead of silently leaving the
last good frame on screen forever. Most rows use `STALE_AFTER_MS`
(1000ms); `stats` uses a longer 3000ms threshold instead, since it only
ticks once per `stats_interval_sec` (default 1Hz) and the tighter default
would flicker red between every sample. `setDot()` turns the same
freshness signal into the sidebar's per-row status dot instead of text
color (gray = `null` entry, i.e. never received at all; green/red = fresh
vs. stale, same threshold logic).

### Layout: sidebar + two corner insets

The left sidebar (`#overlay`) has no fixed or max height in CSS — it's a
`position: fixed` box sized by its content, split into a `feeds` section
(map/scan/pose/drive, each with a status dot) and a `system` section
(cpu/mem/temp/wifi/uptime, one row each). WiFi gets a small 4-bar icon
(`updateWifiBars()`) alongside the raw dBm reading, using the same
dBm-band thresholds phones use for their own signal icons.

Two more elements sit outside `#overlay`, each independently positioned
via CSS (`#minimap-panel` top-right, `#camera-panel` bottom-right):

- **`drawMinimap()`** renders `#minimap`, a *second* canvas with its own
  auto-fit transform, entirely independent of the main canvas's
  `view.centerX/centerY/scale`. It always shows the whole map, plus a
  white outline of whatever rectangle the main canvas currently frames
  (so panning/zooming the main view doesn't lose the big picture) and a
  small car marker. Shows a placeholder (`.has-map` CSS class toggle)
  until a map exists.
- **The camera inset** isn't part of this WebSocket protocol at all —
  `usb_cam_stream` is a separate node on its own port (`9090`), and an
  MJPEG stream is just a never-closing HTTP response, so the browser
  points `#camera-feed`'s `src` directly at
  `http://<host>:9090/stream`. `tryCameraConnect()` retries every 3s
  (with a cache-busting query param) as long as the `<img>`'s `error`
  event has fired more recently than its `load` event, so a camera node
  started after the dashboard page loads still gets picked up without a
  page reload.

## Parameters (`config/web_dashboard.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `map_topic` / `scan_topic` / `pose_topic` / `drive_topic` | `/map` / `/scan` / `/pf/viz/inferred_pose` / `/drive` | Input topics |
| `host` | `0.0.0.0` | Listen on every interface — see the security note in [docs/web-dashboard.md](../../docs/web-dashboard.md#security-note) |
| `port` | `8080` | Web server port |
| `scan_broadcast_rate_hz` | `10.0` | Throttle for `/scan` broadcasts (input itself runs ~40Hz) |
| `stats_interval_sec` | `1.0` | How often CPU%/mem%/temp/WiFi/uptime are sampled and broadcast |
| `laser_offset_x` / `laser_offset_y` | `0.27` / `0.0` | LIDAR mounting offset from `base_link` (matches [hardware-reference.md](../../docs/hardware-reference.md)) |

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Page loads but says "disconnected — retrying..." forever | `dashboard_node` isn't running, or a firewall is blocking the port; check the node's own terminal output |
| "no map yet" never clears | Nothing has published `/map` yet (no SLAM/localization running), or a durability/QoS mismatch — check `ros2 topic info /map` |
| Map shows but scan/car never appear | No pose yet — seed localization with RViz's "2D Pose Estimate" (see [operations.md](../../docs/operations.md)) |
| A feed's status dot is red | That feed has gone stale (>1s since the last update, >3s for `stats`) — check the corresponding ROS topic with `ros2 topic hz`, or the node's own terminal output for `stats`/`drive` |
| `stats` never shows real numbers | The running `dashboard_node` process predates a rebuild — Python files aren't hot-reloaded, so restart `ros2 launch web_dashboard web_dashboard_launch.py` after any `colcon build` that touches this package |
| `temp`/`wifi` show `n/a` | No readable `cpu-thermal` thermal zone / no wireless interface on this machine (e.g. developing on a laptop docked to Ethernet) — expected, not a bug |
| Camera inset shows "camera offline" | `usb_cam_stream` isn't running, or is on a different port than the hardcoded `CAMERA_PORT` (`9090`) in `dashboard.js` |
