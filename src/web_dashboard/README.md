# `web_dashboard`

Live browser dashboard: streams the SLAM/localization map, proximity-colored
LIDAR, pose, measured speed, selected steering command, LB state, and a shared
stopwatch to any web browser over a WebSocket. This file documents the code in
detail; for the workflow (what you'll see at each stage, quick start,
security note) see [docs/web-dashboard.md](../../docs/web-dashboard.md).

**Not an autonomy node** — it publishes to no ROS topic, so none of
[architecture.md](../../docs/architecture.md)'s safety model or the
[mandatory LB-deadman policy](../../docs/architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)
apply to it; both are scoped to nodes that can move the car (see
[writing-your-own-node.md](../../docs/writing-your-own-node.md#the-interface-contract)).

It does have exactly one write path: [live parameter tuning](#live-parameter-tuning-tuningpy)
calls the driving nodes' `set_parameters` service. That changes how a car
that is *already* being driven behaves; it cannot command motion, start
the car, or relax the deadman. See
[docs/web-dashboard.md](../../docs/web-dashboard.md#live-parameter-tuning)
for the user-facing account and `enable_tuning: false` to remove it.

## Files

| File | What it is |
|---|---|
| [`web_dashboard/protocol.py`](web_dashboard/protocol.py) | Wire-format conversion — turns ROS messages into JSON headers + binary payloads. No `rclpy`/Tornado/network imports, so it's unit-testable in isolation (see [`test/test_protocol.py`](test/test_protocol.py)). |
| [`web_dashboard/stopwatch.py`](web_dashboard/stopwatch.py) | ROS-free deadman-gated stopwatch state machine, including joystick timeout handling. |
| [`web_dashboard/tuning.py`](web_dashboard/tuning.py) | Live-tuning support: parsing a node's advertised catalogue, clamping a browser request, and the comment-preserving YAML writer. No ROS/Tornado imports either (see [`test/test_tuning.py`](test/test_tuning.py)). |
| [`web_dashboard/dashboard_node.py`](web_dashboard/dashboard_node.py) | The ROS2 node: subscribes to map/scan/pose/command/odom/joy, runs a [Tornado](https://www.tornadoweb.org/) web + WebSocket server, and bridges its two threads. |
| [`web/index.html`](web/index.html), [`web/dashboard.js`](web/dashboard.js), [`web/style.css`](web/style.css) | The main browser dashboard — plain HTML/JS/CSS, no build step. |
| [`web/camera.html`](web/camera.html), [`web/camera.js`](web/camera.js), [`web/camera.css`](web/camera.css) | Full-window camera recording view with clock and telemetry overlays. |
| [`config/web_dashboard.yaml`](config/web_dashboard.yaml) | Every parameter, loaded at launch. |
| [`launch/web_dashboard_launch.py`](launch/web_dashboard_launch.py) | Starts the node with the YAML above. |

## Interface

- **Subscribes:** map (`/map`), scan (`/scan`), pose (`/pf/viz/inferred_pose` *and* `/slam_pose`), selected command (`/ackermann_cmd`), measured odometry (`/odom`), and joystick state (`/joy`). Every subscription is display/timer input only.
- Also samples CPU%/mem%/CPU temp/WiFi signal/uptime on a timer (`psutil` + `/sys/class/thermal` + `/proc/net/wireless`).
- **Publishes:** nothing, to any topic. Browser input can enable/reset the dashboard-local stopwatch (which never leaves this process) and — once armed — change live-tunable parameters on the nodes in `tuning_nodes`.
- **Calls (services):** `/<node>/get_parameters` and `/<node>/set_parameters` for each node in `tuning_nodes`, and nothing else.

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
| `drive` | selected-command `speed`, `steering_angle` | *(none)* |
| `speed` | measured odometry `speed` | *(none)* |
| `stopwatch` | `elapsed_s`, enabled/running flags, LB/freshness flags | *(none)* |
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
`map`, `pose`, `drive`, and `speed` updates are broadcast immediately.
LB/stopwatch state emits at `stopwatch_update_rate_hz` so every open tab stays
synchronized and sees the same reset/elapsed value without redrawing the map for
every joystick message.
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

Its palette (`MAP_FREE_RGB`/`MAP_OCCUPIED_RGB`/`MAP_UNKNOWN_RGBA`) is
deliberately *not* the ROS/RViz convention of white free space on mid-gray
unknown — on this dark UI that reads as a glaring white slab with a gray
border, and washes out the proximity-colored scan drawn on top of it. The
polarity is inverted instead, in `style.css`'s own colors: unknown is a
near-transparent hint of the panel border color (so unmapped area recedes
into the page background), free space is a dark slate "track surface", and
occupied cells are the bright end — desaturated blue-gray, so walls read
clearly without competing with the saturated LIDAR points or the red car.
Intermediate probabilities interpolate between free and occupied. Because
unknown fades out, `drawMap()` strokes a one-pixel `#263140` hairline —
the same border every panel uses — around the grid's extent, so the mapped
area still has a visible boundary when zoomed out.

Every one of map/scan/pose/drive/speed/stopwatch/stats carries its own `receivedAt` (this
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
`position: fixed` box sized by its content, split into `feeds`, `vehicle`
(measured speed, selected steering, LB), `LB stopwatch` (enable/reset), and
`system` sections. WiFi gets a small 4-bar icon
(`updateWifiBars()`) alongside the raw dBm reading, using the same
dBm-band thresholds phones use for their own signal icons.

Two more elements sit outside `#overlay`, each independently positioned
via CSS (`#minimap-panel` top-right, `#camera-panel` bottom-right):

- **`drawMinimap()`** renders `#minimap`, a *second* canvas with its own
  auto-fit transform, entirely independent of the main canvas's
  `view.centerX/centerY/scale`. It always shows the whole map (same
  palette and extent hairline as the main canvas), plus an accent-blue
  outline of whatever rectangle the main canvas currently frames
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
  page reload. Clicking the inset opens a full-window recording tab with
  clock, speed, steering, LB, stopwatch, CPU, and WiFi overlays.
- **`#camera-resize`** is the inset's drag-to-resize grip. The panel is
  pinned to the bottom-right, so its top-left corner is the only one that
  can move — which is also where `.panel-label` sits, so the two
  cross-fade on hover rather than sharing the corner. The grip is a
  *sibling* of `#camera-link`, not a child: a drag that ended inside the
  anchor would otherwise open the recording tab on mouse-up. A drag scales
  the panel along the stream's own aspect ratio (`cameraAspect`, read from
  the first frame's `naturalWidth/naturalHeight`) instead of reshaping it
  freely, so the frame is never cropped or letterboxed; the pointer's
  offset from that fixed-aspect diagonal is projected onto it
  (least-squares), so a mostly-sideways drag and a mostly-vertical one
  both track the corner. `cameraMaxSize()` keeps it out of the sidebar and
  the minimap, `applyCameraSize()` leaves the width on its responsive CSS
  clamp until the grip is actually dragged, the chosen width is persisted
  in `localStorage`, and double-clicking the grip clears it.

The recording view (`camera.html`) letterboxes the feed while windowed and
switches to `object-fit: cover` in fullscreen (`.is-fullscreen` on
`#camera-stage`), so it fills the screen with no black bars and no
stretching — entered from the `#fullscreen-toggle` button, the `F` key, or
a double-click on the video, all of which go through `toggleFullscreen()`
on `document.documentElement` so the telemetry overlay comes along. The
button and cursor hide after `CONTROLS_IDLE_MS` of no input
(`.controls-idle`) so they stay out of recordings.

## Parameters (`config/web_dashboard.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `map_topic` / `scan_topic` | `/map` / `/scan` | Map and LIDAR inputs |
| `pose_topics` | `[/pf/viz/inferred_pose, /slam_pose]` | Every map-frame pose source, subscribed at once so one dashboard works across the localization *and* SLAM stacks; last message wins |
| `drive_topic` / `odom_topic` | `/ackermann_cmd` / `/odom` | Selected steering command / measured speed |
| `joy_topic` / `deadman_button` / `joy_timeout_sec` | `/joy` / `4` / `0.5` | Read-only LB input and freshness timeout for the stopwatch |
| `stopwatch_update_rate_hz` | `10.0` | Shared stopwatch broadcast rate |
| `host` | `0.0.0.0` | Listen on every interface — see the security note in [docs/web-dashboard.md](../../docs/web-dashboard.md#security-note) |
| `port` | `8080` | Web server port |
| `scan_broadcast_rate_hz` | `10.0` | Throttle for `/scan` broadcasts (input itself runs ~40Hz) |
| `stats_interval_sec` | `1.0` | How often CPU%/mem%/temp/WiFi/uptime are sampled and broadcast |
| `laser_offset_x` / `laser_offset_y` | `0.33` / `0.0` | Estimated LIDAR mounting offset from `base_link` (matches [hardware-reference.md](../../docs/hardware-reference.md)) |
| `enable_tuning` | `true` | Whether live tuning exists at all; `false` never creates the service clients |
| `tuning_nodes` | `[pure_pursuit_node, gap_follow_node]` | The only nodes ever probed or written to |
| `tuning_config_files` | see YAML | Parallel to `tuning_nodes`: `<package>/<path>` that "save" writes back to |
| `tuning_allow_save` | `true` | `false` allows live tuning but forbids writing to disk |
| `tuning_refresh_sec` / `tuning_request_rate_hz` / `tuning_service_timeout_sec` | `2.0` / `20.0` / `3.0` | Value refresh period, how fast a released slider reaches the car, and when to give up on a service call |

## Live parameter tuning (`tuning.py`)

Four pieces, split across three packages that only agree through ROS
interfaces:

1. **Each driving node advertises a catalogue.** `pure_pursuit` and
   `gap_follow` each declare a read-only `live_tunable_spec` string
   parameter holding JSON: every parameter they will accept live, with a
   hard min/max, a group, a unit, a safety flag, and prose for the UI.
   Built by their own `live_tuning.py`. One `get_parameters` call fetches
   the whole thing, and `ros2 param get /gap_follow_node live_tunable_spec`
   is a readable answer to "what can I change while this is running".

2. **Each driving node enforces its own bounds.** Their
   `add_on_set_parameters_callback` validates every update against that
   same catalogue plus cross-parameter invariants (`min_speed <=
   max_speed`, and so on), applies accepted values to the attributes the
   control loop actually reads, and **refuses** anything it doesn't know
   how to apply. That last part is the crux: these nodes cache parameters
   at startup, so a change they can't apply would otherwise succeed
   silently and leave the reported value disagreeing with how the car
   drives. A rejected batch changes nothing at all.

3. **This node brokers.** `dashboard_node` creates `get_parameters` /
   `set_parameters` clients for each node in `tuning_nodes` only, tracks
   which are alive via `get_node_names_and_namespaces()`, and re-reads
   values on a timer. `tuning.py` parses the catalogue defensively (it
   comes from another process, possibly a different version) and clamps
   incoming requests — not as a safety mechanism, but so the UI can't
   offer a value that will bounce.

4. **The browser renders it.** `dashboard.js` builds the panel from the
   catalogue rather than any hardcoded list, so it can't fall out of sync
   with the nodes.

### Threading

The rule from ["Two concurrency models, one process"](#two-concurrency-models-one-process)
is applied strictly here. All tuning state is owned by the rclpy spin
thread; the IOLoop thread never touches it. A browser request goes onto a
`queue.Queue` and a 20Hz timer on the rclpy thread drains it, coalescing
repeated writes to the same parameter so a dragged slider produces one
service call rather than forty. Service calls use `call_async`, whose done
callbacks already run on the spin thread. The single value the IOLoop
reads directly is `_last_tuning_json`, an immutable string replaced
wholesale, so a newly-connected tab gets either the old snapshot or the
new one, never a half-built dict.

### Arming

Arm state lives on the `WebSocketHandler` instance, not the node: it is a
statement about the person holding *this* device and should not outlive
their tab. It starts `False` on every connection and is enforced
server-side, so a stale tab or a hand-rolled client is refused too.

### Saving (`update_yaml_values`)

Writes are surgical line edits, not a `yaml.safe_load`/`safe_dump` round
trip. These config files are mostly comments explaining why each number is
what it is — which ranges were validated in the simulator, what breaks if
you raise one — and a dump would return a correct file with every one of
them deleted, turning "save my tune" into the silent loss of the most
valuable content in the file. Only values that actually differ from what's
on disk are written, so the resulting `git diff` is reviewable. The write
is atomic (temp file + `os.replace`): a half-written parameter YAML is a
node that won't launch, discovered one run later.

Paths resolve through `os.path.realpath()` on the package share directory,
which follows the `--symlink-install` chain back to the git-tracked file
in `src/`. Without `--symlink-install` this would land in `install/` and be
overwritten by the next build — the panel reports the exact path it wrote,
so that case is visible rather than silent.

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
