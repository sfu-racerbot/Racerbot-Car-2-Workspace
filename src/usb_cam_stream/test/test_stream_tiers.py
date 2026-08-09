"""
The camera stream's two tiers, and the transcoding it now avoids.

The dashboard's camera inset is at most 220 CSS pixels wide and was being
fed a 1280x720 stream -- roughly 34x more picture than it could show, at
12-18 Mbit/s, over the same WiFi link the dashboard's telemetry needs.
These tests pin down the three things that fixed it: a small tier for the
inset, no encoding at all for a tier nobody is watching, and passing the
camera's own JPEG through instead of decoding and re-encoding it (which
was a second lossy generation, and what made the picture look soft).

Needs rclpy but no camera and no network -- frames are handed straight to
the encoder.

    python3 -m pytest src/usb_cam_stream/test/test_stream_tiers.py -v
"""
import cv2
import numpy as np
import pytest
import rclpy

from usb_cam_stream.camera_stream_node import (
    FULL, PREVIEW, CameraStreamNode, _looks_like_jpeg,
)


@pytest.fixture
def node():
    rclpy.init(args=[
        '--ros-args',
        '-p', 'image_topic:=/test/image',   # no V4L2 device is opened
        '-p', 'status_log_period_sec:=0.0',
        '-p', 'preview_width:=480',
        '-p', 'preview_quality:=65',
        '-p', 'full_quality:=90',
    ])
    node = CameraStreamNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _frame(width=1280, height=720):
    """A picture with real detail in it, so JPEG sizes mean something --
    a flat colour would compress to nearly nothing at any quality."""
    rng = np.random.default_rng(1234)
    noise = rng.integers(0, 255, size=(height, width, 3), dtype=np.uint8)
    gradient = np.linspace(0, 255, width, dtype=np.uint8)
    base = np.repeat(gradient[None, :, None], height, axis=0).repeat(3, axis=2)
    return ((base.astype(np.uint16) + noise) // 2).astype(np.uint8)


# --------------------------------------------------------------------------
# Which stream a request gets
# --------------------------------------------------------------------------

def test_a_plain_request_gets_the_cheap_preview(node):
    # The dashboard inset is the common case, and the one that has to be
    # cheap; anything that has not asked for more should not get more.
    assert node.tier_for_request(None, None).name == PREVIEW


def test_the_recording_view_can_ask_for_the_full_stream(node):
    assert node.tier_for_request('full', None).name == FULL


def test_the_legacy_full_flag_still_works(node):
    assert node.tier_for_request(None, '1').name == FULL
    assert node.tier_for_request(None, '0').name == PREVIEW
    assert node.tier_for_request(None, 'false').name == PREVIEW


def test_an_unknown_tier_falls_back_to_preview_rather_than_failing(node):
    assert node.tier_for_request('enormous', None).name == PREVIEW


# --------------------------------------------------------------------------
# Nobody watching costs nothing
# --------------------------------------------------------------------------

def test_no_viewers_means_no_encoding_at_all(node):
    node._submit(_frame(), None)
    node.encode_pending()
    assert node.tiers[PREVIEW].jpeg is None
    assert node.tiers[FULL].jpeg is None
    # The frame still counts, so the health/stale checks keep working.
    assert node.latest_seq == 1


def test_only_the_watched_tier_is_encoded(node):
    node.tiers[PREVIEW].viewers = 1
    node._submit(_frame(), None)
    node.encode_pending()
    assert node.tiers[PREVIEW].jpeg is not None
    assert node.tiers[FULL].jpeg is None, 'encoded a tier nobody asked for'


def test_both_tiers_are_encoded_when_both_are_watched(node):
    node.tiers[PREVIEW].viewers = 1
    node.tiers[FULL].viewers = 2
    node._submit(_frame(), None)
    node.encode_pending()
    assert node.tiers[PREVIEW].jpeg is not None
    assert node.tiers[FULL].jpeg is not None


# --------------------------------------------------------------------------
# The preview really is small
# --------------------------------------------------------------------------

def test_the_preview_is_downscaled_to_its_configured_width(node):
    node.tiers[PREVIEW].viewers = 1
    node._submit(_frame(1280, 720), None)
    node.encode_pending()
    decoded = cv2.imdecode(
        np.frombuffer(node.tiers[PREVIEW].jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape[1] == 480
    # Aspect ratio preserved: a stretched frame would misrepresent how far
    # away things are, which matters on a view used to judge driving.
    assert decoded.shape[0] == pytest.approx(480 * 720 / 1280, abs=1)


def test_the_full_tier_keeps_the_cameras_own_size(node):
    node.tiers[FULL].viewers = 1
    node._submit(_frame(1280, 720), None)
    node.encode_pending()
    decoded = cv2.imdecode(
        np.frombuffer(node.tiers[FULL].jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert (decoded.shape[1], decoded.shape[0]) == (1280, 720)


def test_the_preview_is_dramatically_smaller_on_the_wire(node):
    """The whole point, asserted rather than assumed."""
    node.tiers[PREVIEW].viewers = 1
    node.tiers[FULL].viewers = 1
    node._submit(_frame(1280, 720), None)
    node.encode_pending()
    preview = len(node.tiers[PREVIEW].jpeg)
    full = len(node.tiers[FULL].jpeg)
    assert preview * 5 < full, f'preview {preview}B vs full {full}B is not a real saving'


def test_a_frame_smaller_than_the_preview_width_is_not_upscaled(node):
    node.tiers[PREVIEW].viewers = 1
    node._submit(_frame(320, 240), None)
    node.encode_pending()
    decoded = cv2.imdecode(
        np.frombuffer(node.tiers[PREVIEW].jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert (decoded.shape[1], decoded.shape[0]) == (320, 240)


# --------------------------------------------------------------------------
# Passing the camera's own JPEG through
# --------------------------------------------------------------------------

def test_a_camera_jpeg_reaches_the_full_tier_untouched(node):
    """No decode, no re-encode, and so no second generation of loss."""
    ok, buf = cv2.imencode('.jpg', _frame(640, 480))
    assert ok
    source = buf.tobytes()

    node.tiers[FULL].viewers = 1
    node._submit(None, source)
    node.encode_pending()
    assert node.tiers[FULL].jpeg == source, 'the camera JPEG was re-encoded'


def test_a_camera_jpeg_is_decoded_only_when_the_preview_needs_scaling(node):
    ok, buf = cv2.imencode('.jpg', _frame(1280, 720))
    assert ok
    source = buf.tobytes()

    node.tiers[PREVIEW].viewers = 1
    node.tiers[FULL].viewers = 1
    node._submit(None, source)
    node.encode_pending()

    # Full is the original bytes; preview had to be decoded and shrunk.
    assert node.tiers[FULL].jpeg == source
    preview = cv2.imdecode(
        np.frombuffer(node.tiers[PREVIEW].jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert preview.shape[1] == 480


def test_recognising_a_camera_jpeg_from_a_decoded_frame():
    ok, buf = cv2.imencode('.jpg', _frame(64, 48))
    assert ok
    # A raw MJPEG buffer: one-dimensional, starting with the SOI marker.
    assert _looks_like_jpeg(buf.reshape(-1))
    # A decoded BGR frame is three-dimensional and must never be mistaken
    # for one -- serving it as a JPEG would send raw pixels to an <img>.
    assert not _looks_like_jpeg(_frame(64, 48))
    assert not _looks_like_jpeg(None)
    assert not _looks_like_jpeg(np.zeros(2, dtype=np.uint8))


# --------------------------------------------------------------------------
# Latency: only the newest frame survives
# --------------------------------------------------------------------------

def test_only_the_newest_frame_waiting_is_encoded(node):
    """If encoding falls behind, dropping the backlog is what keeps latency
    flat instead of letting it grow without bound."""
    node.tiers[FULL].viewers = 1
    node._submit(_frame(320, 240), None)
    node._submit(_frame(640, 480), None)   # arrives before the encoder runs
    node.encode_pending()
    decoded = cv2.imdecode(
        np.frombuffer(node.tiers[FULL].jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert (decoded.shape[1], decoded.shape[0]) == (640, 480)
    # And the stale one is gone rather than queued behind it.
    assert node.encode_pending() is False


def test_each_stored_frame_advances_the_tier_sequence(node):
    """The sequence is how a parked HTTP handler knows there is something
    new to send."""
    node.tiers[FULL].viewers = 1
    for expected in (1, 2, 3):
        node._submit(_frame(160, 120), None)
        node.encode_pending()
        assert node.tiers[FULL].seq == expected


def test_the_topic_subscription_never_encodes_on_the_ros_thread(node):
    """A subscription callback that encodes a 720p JPEG stalls every other
    ROS callback behind it. It must only hand the frame over."""
    from sensor_msgs.msg import Image

    node.tiers[FULL].viewers = 1
    message = Image()
    message.height, message.width = 240, 320
    message.encoding = 'bgr8'
    message.step = 320 * 3
    message.data = (_frame(320, 240)).tobytes()

    node._image_callback(message)
    # Nothing encoded yet -- the callback returned without doing the work.
    assert node.tiers[FULL].jpeg is None
    node.encode_pending()
    assert node.tiers[FULL].jpeg is not None
