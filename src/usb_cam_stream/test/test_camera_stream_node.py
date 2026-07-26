"""Real-node coverage for camera-stream status diagnostics."""

import time

import rclpy

from usb_cam_stream.camera_stream_node import CameraStreamNode


def test_topic_stream_reports_missing_live_and_stale_frames():
    rclpy.init(args=[
        '--ros-args',
        '-p', 'image_topic:=/test/image',
        '-p', 'frame_timeout_sec:=0.5',
        '-p', 'status_log_period_sec:=0.0',
    ])
    node = CameraStreamNode()
    try:
        node._status_callback()
        assert node.last_stream_state == 'waiting_for_image_topic'

        node._store_frame(b'jpeg')
        assert node.last_stream_state == 'streaming'
        assert node.latest_seq == 1

        node.latest_frame_time = time.monotonic() - 1.0
        node._status_callback()
        assert node.last_stream_state == 'image_topic_stale'
    finally:
        node.destroy_node()
        rclpy.shutdown()
