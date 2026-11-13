#!/usr/bin/env python3
"""Expose a ROS image topic as an HTTP MJPEG stream for a web GCS.

Browsers cannot consume RTSP directly.  This node intentionally serves MJPEG
over HTTP, which can be displayed by a web page with an <img> element:
http://JETSON_IP:8080/stream.mjpg
"""

import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from msgs.msg import MissionStatus


class _MjpegHandler(BaseHTTPRequestHandler):
    server_version = "DagozillaMjpeg/1.0"

    def _headers(self, content_type):
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")

    def do_GET(self):  # noqa: N802 - required by BaseHTTPRequestHandler
        streamer = self.server.streamer
        if self.path in ("/", "/health"):
            self.send_response(HTTPStatus.OK)
            self._headers("text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(
                f"ok\ntopic={streamer.image_topic}\nframes={streamer.frame_count}\n".encode()
            )
            return

        if self.path == "/snapshot.jpg":
            frame = streamer.get_frame()
            if frame is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No frame received")
                return
            self.send_response(HTTPStatus.OK)
            self._headers("image/jpeg")
            self.end_headers()
            self.wfile.write(frame)
            return

        if self.path != "/stream.mjpg":
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        self.send_response(HTTPStatus.OK)
        self._headers("multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while rclpy.ok():
                frame = streamer.wait_for_frame(streamer.frame_count)
                if frame is None:
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, fmt, *args):
        # Keep HTTP requests out of the ROS console unless there is an error.
        return


class _MjpegServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, streamer):
        super().__init__(address, _MjpegHandler)
        self.streamer = streamer


class CameraRtspStreamerNode(Node):
    def __init__(self):
        super().__init__("camera_rtsp_streamer_node")
        self.image_topic = self.declare_parameter(
            "image_topic", "/asv/guidance_debug").value
        host = self.declare_parameter("host", "0.0.0.0").value
        port = int(self.declare_parameter("port", 8080).value)
        self.jpeg_quality = int(self.declare_parameter("jpeg_quality", 80).value)
        self._lock = threading.Condition()
        self._frame = None
        self.frame_count = 0
        self._stream_active = False
        self._bridge = CvBridge()
        self._server = _MjpegServer((host, port), self)

        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(MissionStatus, "/asv/mission_status", self.mission_callback, 10)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever, daemon=True)
        self._server_thread.start()
        self.get_logger().info(
            "Web video stream aktif: http://%s:%d/stream.mjpg (topic %s)" %
            (host, port, self.image_topic))

    def mission_callback(self, msg):
        active = msg.state not in (MissionStatus.IDLE, MissionStatus.COMPLETED, MissionStatus.ABORTED, MissionStatus.TIMEOUT, MissionStatus.FINISH)
        with self._lock:
            self._stream_active = active
            if not active:
                self._frame = None
            self._lock.notify_all()

    def image_callback(self, msg):
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                return
            with self._lock:
                self._frame = encoded.tobytes()
                self.frame_count += 1
                self._lock.notify_all()
        except Exception as exc:  # keep the ROS node alive on malformed frames
            self.get_logger().warning("Gagal encode frame: %s" % exc)

    def get_frame(self):
        with self._lock:
            if not self._stream_active:
                return None
            return self._frame

    def wait_for_frame(self, previous_count):
        with self._lock:
            self._lock.wait_for(
                lambda: (self._stream_active and self.frame_count > previous_count) or not rclpy.ok(), 1.0)
            return self._frame

    def close(self):
        self._server.shutdown()
        self._server.server_close()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRtspStreamerNode()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
