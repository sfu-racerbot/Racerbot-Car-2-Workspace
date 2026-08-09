"""
camera_stream_node.py

Captures frames from a USB webcam (any UVC-compliant device -- see
docs/usb-camera-livestream.md for camera recommendations) and serves them
as a live MJPEG video stream over plain HTTP: open http://<car-ip>:9090/
in any browser and watch, no plugins, no WebRTC signaling, no ROS install
needed on the viewing device.

Alternate source mode: set the `image_topic` parameter to a
sensor_msgs/Image topic (e.g. the RealSense's
/camera/camera/color/image_raw) and the node subscribes to that instead of
opening a V4L2 device. This exists because a camera whose device is
already held open by its own ROS driver node (realsense2_camera_node holds
the D435i's /dev/videoN exclusively) can't be captured a second time via
V4L2 -- but its frames are right there on a topic. Everything downstream
(the MJPEG endpoints, the web_dashboard camera panel that points at them)
is identical in both modes. See docs/realsense-camera.md.

## Two tiers, because there are two viewers with opposite needs

The stream used to be one size for everybody: 1280x720 at JPEG quality 80,
which is roughly 100-150kB a frame. The dashboard's camera inset is at
most 220 CSS pixels wide. We were encoding and transmitting about 34 times
more picture than that panel could ever show, at 12-18 Mbit/s, over the
same WiFi link the dashboard's own telemetry needs -- which is most of why
both felt laggy.

So there are two endpoints now:

  /stream                 preview: small and cheap, for the dashboard inset
  /stream?tier=full       full: native resolution and high quality, for the
                          recording view (camera.html) and anyone watching
                          the raw feed

Each tier is encoded once and shared by every viewer of that tier, and a
tier with no viewers is not encoded at all. Nobody watching costs nothing.

## Not re-compressing what the camera already compressed

`_open_capture` asks the camera for MJPEG, which any UVC webcam produces
in hardware. OpenCV then silently *decodes* that to BGR inside `.read()`,
and this node used to *re-encode* it to JPEG -- a second lossy generation
on top of the camera's own, which is what made the picture look mushy.
With `CAP_PROP_CONVERT_RGB` turned off, the camera's own JPEG can be
passed through untouched: sharper, and essentially free. Backend support
for that varies, so it is probed once at open and falls back to
decode-and-encode, logging which mode is actually in use.

## Threads

Four things run concurrently here, and which thread does what matters:
  - rclpy's executor, for parameters and the optional image subscription.
    Subscription callbacks must return immediately, so they only hand the
    frame over -- they never encode.
  - The capture thread, which owns cv2.VideoCapture exclusively. OpenCV's
    blocking .read() must never run on the IOLoop.
  - The encode thread, which turns the newest source frame into JPEGs for
    whichever tiers have viewers. It always works on the *newest* frame and
    drops anything that piled up behind it, so a slow encode adds no
    latency, only dropped frames.
  - Tornado's IOLoop, which serves HTTP and owns every response.

Tier state is written by the encode thread and read by the IOLoop. Those
are plain attribute assignments of immutable values, so under the GIL a
reader gets either the old frame or the new one -- and one frame late is
meaningless for live video. Waking the IOLoop is done with
`add_callback()`, which is the documented thread-safe way in.
"""

import asyncio
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import tornado.ioloop
import tornado.iostream
import tornado.web
from ament_index_python.packages import get_package_share_directory

MJPEG_BOUNDARY = b'racerbotframe'

PREVIEW = 'preview'
FULL = 'full'


class _Tier:
    """One encoded size of the stream, shared by everyone watching it."""

    def __init__(self, name, max_width, quality):
        self.name = name
        self.max_width = int(max_width)   # 0 = leave at the source's size
        self.quality = int(quality)
        self.jpeg = None
        self.seq = 0
        # Number of connected viewers. Only the IOLoop mutates it; the
        # encode thread reads it to decide whether this tier is worth
        # encoding at all.
        self.viewers = 0
        # Futures belonging to handlers parked waiting for the next frame.
        # Only ever touched on the IOLoop thread.
        self._waiters = []

    def store(self, jpeg):
        self.jpeg = jpeg
        self.seq += 1

    # -- IOLoop-thread only ------------------------------------------------

    def wait_for_frame(self):
        # get_running_loop, not get_event_loop: this is only ever called
        # from inside a running handler coroutine, and the deprecated form
        # would quietly create a second loop if that ever stopped being true.
        future = asyncio.get_running_loop().create_future()
        self._waiters.append(future)
        return future

    def wake_waiters(self):
        for future in self._waiters:
            if not future.done():
                future.set_result(None)
        self._waiters.clear()


class MJPEGStreamHandler(tornado.web.RequestHandler):
    """One instance per connected browser tab / <img> element. The HTTP
    response here never actually ends -- it just keeps streaming new
    multipart JPEG chunks down the same connection for as long as the
    client stays connected, which is exactly what an <img src="/stream">
    tag expects (no JS required on the browser side)."""

    def initialize(self, node: 'CameraStreamNode'):
        self.node = node

    async def get(self):
        tier = self.node.tier_for_request(
            self.get_argument('tier', None), self.get_argument('full', None))
        self.set_header(
            'Content-Type', f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY.decode()}")
        self.set_header('Cache-Control', 'no-cache, private')
        self.set_header('Pragma', 'no-cache')

        tier.viewers += 1
        self.node.note_viewers_changed()
        last_seq = -1
        min_period = 1.0 / max(self.node.stream_fps, 0.1)
        try:
            while True:
                jpeg, seq = tier.jpeg, tier.seq
                if jpeg is None or seq == last_seq:
                    # Nothing new: park until the encoder says otherwise,
                    # rather than waking on a timer to find out. That poll
                    # used to add up to a whole frame period of latency to
                    # every single frame.
                    await tier.wait_for_frame()
                    continue
                last_seq = seq
                self.write(
                    b'--' + MJPEG_BOUNDARY + b'\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n'
                    + jpeg + b'\r\n'
                )
                # Waiting for the flush is what applies backpressure. Any
                # frames encoded while it is in flight are simply skipped:
                # the next pass reads whatever is newest at that moment, so
                # a slow link loses frames instead of accumulating delay.
                await self.flush()
                if min_period > 0:
                    await asyncio.sleep(min_period)
        except (tornado.iostream.StreamClosedError, ConnectionResetError):
            pass  # browser tab closed / <img> removed -- not an error
        except asyncio.CancelledError:
            raise
        finally:
            tier.viewers = max(0, tier.viewers - 1)
            self.node.note_viewers_changed()


class CameraStreamNode(Node):
    """Owns the frame source (camera capture thread, or an image-topic
    subscription in image_topic mode) and builds the Tornado app. It never
    publishes anything -- read-only in both modes -- so it's safe to run
    alongside anything else in this workspace, at any time, including
    during a race."""

    def __init__(self):
        super().__init__('usb_cam_stream_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('image_topic', '')   # non-empty switches source: ROS topic instead of V4L2
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('capture_fps', 30)
        self.declare_parameter('stream_fps', 30.0)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 9090)
        self.declare_parameter('frame_timeout_sec', 2.0)
        self.declare_parameter('status_log_period_sec', 5.0)

        # --- Tiers (see the module docstring) ---
        # The dashboard inset is at most 220 CSS pixels wide, so 480 is
        # already generous enough for a high-density display.
        self.declare_parameter('preview_width', 480)
        self.declare_parameter('preview_quality', 65)
        # The recording view gets the real thing. Quality can be high here
        # precisely because the small tier carries the routine traffic.
        self.declare_parameter('full_width', 0)      # 0 = the camera's own size
        self.declare_parameter('full_quality', 90)
        # Pass the camera's own MJPEG through instead of decoding and
        # re-encoding it. Set false if a camera or OpenCV build misbehaves.
        self.declare_parameter('passthrough', True)
        # Legacy: a single quality knob, used as the default for both tiers
        # if someone's config still sets it.
        self.declare_parameter('jpeg_quality', 0)

        self.device = self.get_parameter('device').value
        self.image_topic = self.get_parameter('image_topic').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.capture_fps = int(self.get_parameter('capture_fps').value)
        self.stream_fps = float(self.get_parameter('stream_fps').value)
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.frame_timeout_sec = max(
            0.1, float(self.get_parameter('frame_timeout_sec').value))
        self.status_log_period_sec = max(
            0.0, float(self.get_parameter('status_log_period_sec').value))
        self.passthrough_wanted = bool(self.get_parameter('passthrough').value)

        legacy_quality = int(self.get_parameter('jpeg_quality').value)
        preview_quality = int(self.get_parameter('preview_quality').value)
        full_quality = int(self.get_parameter('full_quality').value)
        if legacy_quality:
            self.get_logger().warn(
                'jpeg_quality is deprecated; use preview_quality/full_quality. '
                f'Applying {legacy_quality} to both tiers.')
            preview_quality = full_quality = legacy_quality

        self.tiers = {
            PREVIEW: _Tier(PREVIEW, self.get_parameter('preview_width').value,
                           preview_quality),
            FULL: _Tier(FULL, self.get_parameter('full_width').value, full_quality),
        }

        # Kept for diagnostics and for anything reading the node's overall
        # liveness; `latest_jpeg` mirrors the full tier.
        self.latest_jpeg = None
        self.latest_seq = 0
        self.latest_frame_time = None
        self.last_stream_state = None
        self.last_stream_log_time = None

        self._stop_event = threading.Event()
        self._cap = None
        self._passthrough_active = False
        self._loop = None

        # The newest source frame waiting to be encoded, and the doorbell
        # that tells the encode thread about it. Holding only the newest is
        # deliberate: if encoding falls behind, dropping the backlog is what
        # keeps latency flat instead of letting it grow without bound.
        self._pending_lock = threading.Lock()
        self._pending_frame = None      # (bgr_frame_or_None, jpeg_or_None)
        self._frame_ready = threading.Event()

        self.status_timer = self.create_timer(
            min(0.5, self.frame_timeout_sec / 2.0), self._status_callback)

        if self.image_topic:
            # Topic source: frames arrive via subscription callbacks on the
            # rclpy executor thread; no capture thread, and the device/
            # width/height/capture_fps parameters are unused (the publisher
            # of the topic owns those).
            self._cv_bridge = CvBridge()
            # Sensor-data QoS, depth 1. The default depth of 10 meant up to
            # ten already-stale frames queued ahead of the current one --
            # a third of a second of pure latency on a 30fps camera, for
            # frames nobody would ever see.
            self.create_subscription(
                Image, self.image_topic, self._image_callback, qos_profile_sensor_data)
            self.get_logger().info(
                f"usb_cam_stream_node ready: source topic={self.image_topic}. "
                f"Once the web server starts, open "
                f"http://<this car's IP>:{self.port}/ in a browser. "
                f"Frame timeout={self.frame_timeout_sec:.1f}s; status logs every "
                f"{self.status_log_period_sec:.1f}s."
            )
        else:
            self.get_logger().info(
                f"usb_cam_stream_node ready: device={self.device} {self.width}x{self.height}"
                f"@{self.capture_fps}fps. Once the web server starts, open "
                f"http://<this car's IP>:{self.port}/ in a browser. "
                f"Frame timeout={self.frame_timeout_sec:.1f}s; status logs every "
                f"{self.status_log_period_sec:.1f}s."
            )

    # ------------------------------------------------------------------------
    # Tier selection and viewer bookkeeping
    # ------------------------------------------------------------------------

    def tier_for_request(self, tier_argument, full_argument):
        """Which stream a request wants. Preview unless it asks otherwise --
        the dashboard inset is the common case and the one that must be
        cheap."""
        if tier_argument in self.tiers:
            return self.tiers[tier_argument]
        if full_argument not in (None, '', '0', 'false'):
            return self.tiers[FULL]
        return self.tiers[PREVIEW]

    def watched_tiers(self):
        return [tier for tier in self.tiers.values() if tier.viewers]

    def note_viewers_changed(self):
        """A viewer arrived or left. Wake the encoder so a newly watched
        tier gets a frame immediately instead of after the next capture."""
        self._frame_ready.set()

    # ------------------------------------------------------------------------
    # Frame sources -- neither of these ever encodes.
    # ------------------------------------------------------------------------

    def _image_callback(self, msg: Image):
        """Runs on the rclpy executor thread, so it must return at once."""
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self._log_stream_status(
                'topic_conversion_failed',
                f"could not convert Image from '{self.image_topic}' to bgr8: "
                f'{type(exc).__name__}: {exc}',
                level='error',
            )
            return
        self._submit(frame, None)

    def _submit(self, frame, jpeg):
        """Hand the newest source frame to the encode thread."""
        with self._pending_lock:
            self._pending_frame = (frame, jpeg)
        self._frame_ready.set()

    # ------------------------------------------------------------------------
    # Capture thread -- the only thread that ever touches cv2.VideoCapture.
    # ------------------------------------------------------------------------

    def _open_capture(self) -> bool:
        # Accept either a plain device index ("0") or a full V4L2 path
        # (the default, "/dev/video0") -- a path is more robust across
        # reboots/hotplugs than an index if more than one video device is
        # ever present (e.g. a UVC camera enumerating alongside some other
        # capture device), see docs/usb-camera-livestream.md.
        device = int(self.device) if str(self.device).isdigit() else self.device
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False

        # Ask the camera to send MJPEG over USB rather than raw YUYV --
        # most UVC webcams (Logitech C920/C922 included) have an onboard
        # hardware encoder for this, which cuts USB bandwidth dramatically
        # at 720p/1080p.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.capture_fps)
        # Ask for the shallowest queue the driver will give us. A deep one
        # hands out frames that were captured several frames ago, which is
        # latency we can never recover.
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except cv2.error:
            pass

        self._cap = cap
        self._passthrough_active = self._probe_passthrough(cap)
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"CAMERA [opened] device='{self.device}', negotiated "
            f'{actual_width}x{actual_height}@{actual_fps:.1f}fps, '
            f'{"passing the camera JPEG through untouched" if self._passthrough_active else "decoding and re-encoding frames"}'
            f'; waiting for first frame')
        return True

    def _probe_passthrough(self, cap) -> bool:
        """Can we serve the camera's own JPEG without touching it?

        Worth a probe rather than an assumption: it removes both a decode
        and an encode per frame *and* the second lossy generation that made
        the picture look soft -- but whether a given OpenCV/V4L2 build
        honours CAP_PROP_CONVERT_RGB is genuinely not knowable in advance.
        """
        if not self.passthrough_wanted:
            return False
        try:
            if not cap.set(cv2.CAP_PROP_CONVERT_RGB, 0):
                return False
            for _ in range(5):   # give the stream a moment to settle
                ok, buf = cap.read()
                if ok and _looks_like_jpeg(buf):
                    return True
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        except cv2.error as exc:
            self.get_logger().warn(
                f'CAMERA [passthrough_unavailable] {exc}; falling back to re-encoding')
            try:
                cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            except cv2.error:
                pass
        return False

    def capture_loop(self):
        """Runs on its own dedicated thread (started in main()). Keeps
        trying to (re)open the camera if it isn't plugged in yet or gets
        unplugged mid-stream, rather than crashing the node -- a USB
        webcam is far more likely to get bumped loose on a moving car than
        the LiDAR/VESC's more permanent connectors."""
        retry_period_sec = 3.0

        while not self._stop_event.is_set():
            if self._cap is None and not self._open_capture():
                self._log_stream_status(
                    'waiting_for_camera',
                    f"Could not open camera '{self.device}' -- retrying in "
                    f"{retry_period_sec:.0f}s. Check it's plugged in and that the "
                    f"device path is correct (`v4l2-ctl --list-devices`).",
                    level='error',
                )
                self._stop_event.wait(retry_period_sec)
                continue

            ok, frame = self._cap.read()
            if not ok:
                self._log_stream_status(
                    'camera_lost',
                    f"Lost camera '{self.device}' during frame capture -- reopening",
                )
                self._cap.release()
                self._cap = None
                continue

            if self._passthrough_active and _looks_like_jpeg(frame):
                # The camera's own JPEG, exactly as it produced it.
                self._submit(None, frame.tobytes())
            else:
                self._submit(frame, None)

        if self._cap is not None:
            self._cap.release()

    # ------------------------------------------------------------------------
    # Encode thread -- the only place JPEGs are made.
    # ------------------------------------------------------------------------

    def encode_loop(self):
        while not self._stop_event.is_set():
            if not self._frame_ready.wait(timeout=0.2):
                continue
            self._frame_ready.clear()
            self.encode_pending()

    def encode_pending(self):
        """Encode the newest waiting frame for every watched tier.

        Split out from the loop so it can be driven a frame at a time from
        a test without threads or a camera.
        """
        with self._pending_lock:
            pending = self._pending_frame
            self._pending_frame = None
        if pending is None:
            return False
        frame, source_jpeg = pending

        watched = self.watched_tiers()
        if not watched:
            # Nobody is looking. Encoding for an audience of zero is exactly
            # the waste the dashboard used to make; skip it, but keep the
            # frame counter honest so the health checks still work.
            self._note_frame()
            return True

        # Decoded lazily and at most once, however many tiers want it.
        decoded = frame
        for tier in watched:
            if source_jpeg is not None and tier.max_width == 0:
                # Native size wanted and the camera already handed us a
                # JPEG: pass it through untouched. No decode, no re-encode,
                # and no second generation of compression loss.
                jpeg = source_jpeg
            else:
                if decoded is None:
                    decoded = cv2.imdecode(
                        np.frombuffer(source_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if decoded is None:
                        self._log_stream_status(
                            'jpeg_decode_failed',
                            'could not decode the frame the camera sent',
                            level='error')
                        break
                jpeg = self._encode(decoded, tier)
            if jpeg is not None:
                tier.store(jpeg)
                if tier.name == FULL:
                    self.latest_jpeg = jpeg
        self._note_frame()
        self._wake_streams(watched)
        return True

    def _encode(self, frame, tier: _Tier):
        scaled = frame
        if tier.max_width and frame.shape[1] > tier.max_width:
            height = max(1, round(frame.shape[0] * tier.max_width / frame.shape[1]))
            # INTER_AREA is the right filter for shrinking: it averages the
            # pixels being merged instead of point-sampling them, so the
            # small tier stays legible rather than aliased.
            scaled = cv2.resize(frame, (tier.max_width, height),
                                interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(
            '.jpg', scaled, [int(cv2.IMWRITE_JPEG_QUALITY), tier.quality])
        if not ok:
            self._log_stream_status(
                'jpeg_encode_failed',
                f"OpenCV could not JPEG-encode a frame for the {tier.name} stream",
                level='error')
            return None
        return buf.tobytes()

    def _wake_streams(self, tiers):
        loop = self._loop
        if loop is None:
            return
        for tier in tiers:
            loop.add_callback(tier.wake_waiters)

    # ------------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------------

    def _note_frame(self):
        recovering = self.last_stream_state != 'streaming'
        self.latest_seq += 1
        self.latest_frame_time = time.monotonic()
        if recovering:
            self._log_stream_status(
                'streaming', self._streaming_detail(0.0), level='info')

    def _store_frame(self, jpeg: bytes):
        """Record one already-encoded frame directly.

        Retained because it is the simplest way to drive this node's status
        machinery without a camera (see test/test_camera_stream_node.py).
        """
        self.latest_jpeg = jpeg
        self.tiers[FULL].store(jpeg)
        self._note_frame()

    def _streaming_detail(self, frame_age_sec: float) -> str:
        source = (
            f"topic='{self.image_topic}'" if self.image_topic
            else f"camera='{self.device}'")
        watching = ', '.join(
            f'{tier.name}x{tier.viewers}' for tier in self.watched_tiers()) or 'nobody'
        return (
            f'{source}, frames={self.latest_seq}, latest frame age='
            f'{frame_age_sec:.2f}s, JPEG bytes={len(self.latest_jpeg or b"")}, '
            f'watching: {watching}')

    def _status_callback(self):
        now = time.monotonic()
        if self.latest_frame_time is None:
            if self.image_topic:
                state = 'waiting_for_image_topic'
                detail = f"no Image received on '{self.image_topic}'"
            elif self._cap is None:
                state = 'waiting_for_camera'
                detail = f"camera '{self.device}' is not open"
            else:
                state = 'waiting_for_first_frame'
                detail = f"camera '{self.device}' is open but has not produced a frame"
            self._log_stream_status(state, detail)
            return

        frame_age_sec = now - self.latest_frame_time
        if frame_age_sec >= self.frame_timeout_sec:
            if self.image_topic:
                state = 'image_topic_stale'
                source = f"Image topic '{self.image_topic}'"
            else:
                state = 'camera_frames_stale'
                source = f"camera '{self.device}'"
            self._log_stream_status(
                state,
                f'{source} has produced no frame for {frame_age_sec:.2f}s '
                f'(limit {self.frame_timeout_sec:.2f}s)',
            )
            return

        self._log_stream_status(
            'streaming', self._streaming_detail(frame_age_sec), level='info')

    def _log_stream_status(self, state: str, detail: str, level: str = None):
        now = time.monotonic()
        state_changed = state != self.last_stream_state
        period_elapsed = (
            self.status_log_period_sec > 0.0
            and (
                self.last_stream_log_time is None
                or now - self.last_stream_log_time >= self.status_log_period_sec
            )
        )
        if not state_changed and not period_elapsed:
            return

        message = f'CAMERA [{state}] {detail}'
        if level == 'error':
            self.get_logger().error(message)
        elif level == 'info':
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        self.last_stream_state = state
        self.last_stream_log_time = now

    def stop(self):
        self._stop_event.set()
        self._frame_ready.set()   # let the encode thread notice and exit

    # ------------------------------------------------------------------------
    # Web server
    # ------------------------------------------------------------------------

    def make_app(self) -> tornado.web.Application:
        static_dir = os.path.join(get_package_share_directory('usb_cam_stream'), 'web')
        return tornado.web.Application([
            (r'/stream', MJPEGStreamHandler, {'node': self}),
            # Catch-all *after* /stream -- Tornado matches routes in order.
            (r'/(.*)', tornado.web.StaticFileHandler, {'path': static_dir, 'default_filename': 'index.html'}),
        ])


def _looks_like_jpeg(buffer) -> bool:
    """Is this the camera's raw JPEG rather than a decoded BGR image?

    A decoded frame is a 3-dimensional array; a JPEG comes back as a flat
    byte run starting with the SOI marker FF D8.
    """
    if buffer is None:
        return False
    try:
        if getattr(buffer, 'ndim', 0) != 1 or buffer.size < 4:
            return False
        return int(buffer[0]) == 0xFF and int(buffer[1]) == 0xD8
    except (TypeError, ValueError, IndexError):
        return False


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    encode_thread = threading.Thread(target=node.encode_loop, daemon=True)
    encode_thread.start()

    if not node.image_topic:
        capture_thread = threading.Thread(target=node.capture_loop, daemon=True)
        capture_thread.start()

    app = node.make_app()
    app.listen(node.port, address=node.host)
    node._loop = tornado.ioloop.IOLoop.current()
    node.get_logger().info(f"Serving on http://{node.host}:{node.port}/ (Ctrl+C to stop)")

    try:
        node._loop.start()
    except KeyboardInterrupt:
        pass
    finally:
        node._loop.stop()
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
