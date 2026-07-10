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
(the MJPEG endpoint, the web_dashboard camera panel that points at it) is
identical in both modes. See docs/realsense-camera.md.

Three concurrency models share this one process (an extra one compared to
web_dashboard_node -- see that file's docstring for the base pattern this
extends):
  - rclpy's own executor, spun on a background thread purely so
    `ros2 param`/lifecycle introspection still works on this node -- it
    has no subscriptions or publishers of its own.
  - A dedicated capture thread that owns cv2.VideoCapture exclusively.
    OpenCV's blocking .read() call must never run on Tornado's IOLoop
    thread (it would stall every HTTP client, including the WebSocket-less
    MJPEG stream below, for the duration of each camera read).
  - Tornado's IOLoop, which owns the main thread and serves HTTP,
    including the long-lived multipart MJPEG response each connected
    browser tab keeps open.

The capture thread and IOLoop handlers only ever communicate through one
`(sequence number, JPEG bytes)` pair, written by the capture thread and
read by every stream handler. Both are plain attribute assignments, so
under the GIL they're atomic; a handler occasionally reading one frame
late is a non-issue for a live video feed, so -- same reasoning as
web_dashboard_node's `_last_pose` etc. -- no lock is used here.
"""

import asyncio
import os
import threading

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import tornado.ioloop
import tornado.iostream
import tornado.web
from ament_index_python.packages import get_package_share_directory

MJPEG_BOUNDARY = b'racerbotframe'


class MJPEGStreamHandler(tornado.web.RequestHandler):
    """One instance per connected browser tab / <img> element. The HTTP
    response here never actually ends -- it just keeps streaming new
    multipart JPEG chunks down the same connection for as long as the
    client stays connected, which is exactly what an <img src="/stream">
    tag expects (no JS required on the browser side)."""

    def initialize(self, node: 'CameraStreamNode'):
        self.node = node

    async def get(self):
        self.set_header(
            'Content-Type', f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY.decode()}")
        self.set_header('Cache-Control', 'no-cache, private')
        self.set_header('Pragma', 'no-cache')

        last_seq = -1
        try:
            while True:
                seq, jpeg = self.node.latest_seq, self.node.latest_jpeg
                if jpeg is not None and seq != last_seq:
                    last_seq = seq
                    self.write(
                        b'--' + MJPEG_BOUNDARY + b'\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n'
                        + jpeg + b'\r\n'
                    )
                    await self.flush()
                await asyncio.sleep(1.0 / self.node.stream_fps)
        except (tornado.iostream.StreamClosedError, ConnectionResetError):
            pass  # browser tab closed / <img> removed -- not an error


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
        self.declare_parameter('stream_fps', 15.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 9090)

        self.device = self.get_parameter('device').value
        self.image_topic = self.get_parameter('image_topic').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.capture_fps = int(self.get_parameter('capture_fps').value)
        self.stream_fps = float(self.get_parameter('stream_fps').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)

        # Written only by the frame source (capture thread, or the rclpy
        # executor thread in image_topic mode -- exactly one of the two
        # exists per instance), read only by MJPEGStreamHandler (IOLoop
        # thread) -- see module docstring.
        self.latest_jpeg = None
        self.latest_seq = 0

        self._stop_event = threading.Event()
        self._cap = None
        self._encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]

        if self.image_topic:
            # Topic source: frames arrive via subscription callbacks on the
            # rclpy executor thread; no capture thread, and the device/
            # width/height/capture_fps parameters are unused (the publisher
            # of the topic owns those).
            self._cv_bridge = CvBridge()
            self.create_subscription(Image, self.image_topic, self._image_callback, 10)
            self.get_logger().info(
                f"usb_cam_stream_node ready: source topic={self.image_topic}. "
                f"Once the web server starts, open "
                f"http://<this car's IP>:{self.port}/ in a browser."
            )
        else:
            self.get_logger().info(
                f"usb_cam_stream_node ready: device={self.device} {self.width}x{self.height}"
                f"@{self.capture_fps}fps. Once the web server starts, open "
                f"http://<this car's IP>:{self.port}/ in a browser."
            )

    # ------------------------------------------------------------------------
    # Topic source -- runs on the rclpy executor thread (image_topic mode only).
    # ------------------------------------------------------------------------

    def _image_callback(self, msg: Image):
        frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, buf = cv2.imencode('.jpg', frame, self._encode_params)
        if ok:
            self.latest_jpeg = buf.tobytes()
            self.latest_seq += 1

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
        # at 720p/1080p. Frames are still re-encoded to JPEG below at our
        # own configured quality regardless, so this only affects
        # camera->Jetson bandwidth, not the stream actually served.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.capture_fps)

        self._cap = cap
        return True

    def capture_loop(self):
        """Runs on its own dedicated thread (started in main()). Keeps
        trying to (re)open the camera if it isn't plugged in yet or gets
        unplugged mid-stream, rather than crashing the node -- a USB
        webcam is far more likely to get bumped loose on a moving car than
        the LiDAR/VESC's more permanent connectors."""
        retry_period_sec = 3.0

        while not self._stop_event.is_set():
            if self._cap is None and not self._open_capture():
                self.get_logger().error(
                    f"Could not open camera '{self.device}' -- retrying in "
                    f"{retry_period_sec:.0f}s. Check it's plugged in and that the "
                    f"device path is correct (`v4l2-ctl --list-devices`)."
                )
                self._stop_event.wait(retry_period_sec)
                continue

            ok, frame = self._cap.read()
            if not ok:
                self.get_logger().warning(f"Lost camera '{self.device}' -- reopening.")
                self._cap.release()
                self._cap = None
                continue

            ok, buf = cv2.imencode('.jpg', frame, self._encode_params)
            if ok:
                self.latest_jpeg = buf.tobytes()
                self.latest_seq += 1

        if self._cap is not None:
            self._cap.release()

    def stop(self):
        self._stop_event.set()

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


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    if not node.image_topic:
        capture_thread = threading.Thread(target=node.capture_loop, daemon=True)
        capture_thread.start()

    app = node.make_app()
    app.listen(node.port, address=node.host)
    node.get_logger().info(f"Serving on http://{node.host}:{node.port}/ (Ctrl+C to stop)")

    loop = tornado.ioloop.IOLoop.current()
    try:
        loop.start()
    except KeyboardInterrupt:
        pass
    finally:
        loop.stop()
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
