#!/usr/bin/env python3
"""ROS 2 -> GCS media bridge.

The bridge consumes ROS image topics (it never opens /dev/video*), publishes
the surface topic to MediaMTX over RTSP, and uploads still captures to the
existing FastAPI image endpoint. Mission hooks are deliberately small:
SURFACE_IMAGING and UNDERWATER_IMAGING on a std_msgs/String status topic, plus
the two std_srvs/Trigger services for dry tests.
"""

import datetime as dt
import logging
import threading
import urllib.error
import urllib.request
import uuid
from concurrent.futures import ThreadPoolExecutor

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except ImportError as exc:  # pragma: no cover - exercised on Jetson only
    Gst = None
    _GST_IMPORT_ERROR = exc


LOG = logging.getLogger("gcs_media_bridge")


class GcsMediaBridge(Node):
    def __init__(self):
        super().__init__("gcs_media_bridge")
        self.declare_parameter("gcs_api_url", "http://<GCS_IP>:8000")
        self.declare_parameter("jetson_api_key", "test_key")
        self.declare_parameter("surface_stream_ros_topic", "/asv/camera_debug")
        self.declare_parameter("surface_stream_rtsp_url", "rtsp://<GCS_IP>:8554/surface")
        self.declare_parameter("surface_image_ros_topic", "/camera/image_raw")
        self.declare_parameter("underwater_image_ros_topic", "/underwater_camera/image_raw")
        self.declare_parameter("mission_status_topic", "/asv/mission_status")
        self.declare_parameter("run_id", 0)
        self.declare_parameter("grid_position", "")
        self.declare_parameter("stream_fps", 15.0)

        self.bridge = CvBridge()
        self.executor = ThreadPoolExecutor(max_workers=2)
        self.latest = {"SURFACE": None, "UNDERWATER": None}
        self.latest_lock = threading.Lock()
        self.previous_status = ""
        self.stream_pipeline = None
        self.stream_caps_set = False

        surface_stream = self.param("surface_stream_ros_topic")
        surface_image = self.param("surface_image_ros_topic")
        underwater_image = self.param("underwater_image_ros_topic")
        status_topic = self.param("mission_status_topic")

        self.create_subscription(Image, surface_image, self.surface_image_callback, 10)
        self.create_subscription(Image, underwater_image, self.underwater_image_callback, 10)
        self.create_subscription(Image, surface_stream, self.stream_callback, 10)
        self.create_subscription(String, status_topic, self.status_callback, 10)
        self.create_service(Trigger, "mark_surface_image", self.surface_trigger)
        self.create_service(Trigger, "mark_underwater_image", self.underwater_trigger)

        if Gst is None:
            self.get_logger().error("GStreamer Python bindings unavailable: %s", _GST_IMPORT_ERROR)
        else:
            Gst.init(None)
            self.start_stream_pipeline()

    def param(self, name):
        return self.get_parameter(name).value

    def surface_image_callback(self, message):
        self.cache_frame("SURFACE", message)

    def underwater_image_callback(self, message):
        self.cache_frame("UNDERWATER", message)

    def cache_frame(self, image_type, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            with self.latest_lock:
                self.latest[image_type] = frame.copy()
        except Exception as exc:
            self.get_logger().warning("Cannot decode %s ROS image: %s", image_type, exc)

    def start_stream_pipeline(self):
        fps = float(self.param("stream_fps"))
        pipeline = (
            "appsrc name=source is-live=true block=false format=time "
            "max-buffers=1 leaky-type=downstream "
            f"caps=video/x-raw,format=BGR,framerate={int(fps)}/1 "
            "! videoconvert ! video/x-raw,format=I420 "
            "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2500 key-int-max=30 "
            "! rtph264pay pt=96 config-interval=1 "
            f"! rtspclientsink location={self.param('surface_stream_rtsp_url')} protocols=tcp"
        )
        try:
            self.stream_pipeline = Gst.parse_launch(pipeline)
            self.stream_source = self.stream_pipeline.get_by_name("source")
            self.stream_pipeline.set_state(Gst.State.PLAYING)
            self.get_logger().info("Surface RTSP publisher: %s", self.param("surface_stream_rtsp_url"))
        except Exception as exc:
            self.get_logger().error("Unable to start GStreamer RTSP pipeline: %s", exc)
            self.stream_pipeline = None

    def stream_callback(self, message):
        if self.stream_pipeline is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            height, width = frame.shape[:2]
            if not self.stream_caps_set:
                fps = int(float(self.param("stream_fps")))
                self.stream_source.set_property(
                    "caps", Gst.Caps.from_string(
                        f"video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1"
                    )
                )
                self.stream_caps_set = True
            buffer = Gst.Buffer.new_wrapped(frame.tobytes())
            result = self.stream_source.emit("push-buffer", buffer)
            if result != Gst.FlowReturn.OK:
                self.get_logger().debug("RTSP push returned %s", result)
        except Exception as exc:
            self.get_logger().warning("Cannot publish stream frame: %s", exc)

    def status_callback(self, message):
        status = message.data.strip().upper()
        if status == self.previous_status:
            return
        self.previous_status = status
        if status == "SURFACE_IMAGING":
            self.capture("SURFACE")
        elif status == "UNDERWATER_IMAGING":
            self.capture("UNDERWATER")

    def surface_trigger(self, request, response):
        del request
        return self.trigger_response(response, "SURFACE")

    def underwater_trigger(self, request, response):
        del request
        return self.trigger_response(response, "UNDERWATER")

    def trigger_response(self, response, image_type):
        with self.latest_lock:
            available = self.latest[image_type] is not None
        if not available:
            response.success = False
            response.message = f"No frame cached for {image_type}"
            return response
        self.capture(image_type)
        response.success = True
        response.message = f"{image_type} capture queued"
        return response

    def capture(self, image_type):
        with self.latest_lock:
            frame = None if self.latest[image_type] is None else self.latest[image_type].copy()
        if frame is None:
            self.get_logger().warning("No cached frame for %s", image_type)
            return
        self.executor.submit(self.upload_frame, image_type, frame)

    def upload_frame(self, image_type, frame):
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            self.get_logger().error("JPEG encoding failed for %s", image_type)
            return
        fields = {
            "image_type": image_type,
            "captured_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        }
        run_id = int(self.param("run_id"))
        if run_id > 0:
            fields["run_id"] = str(run_id)
        grid_position = str(self.param("grid_position"))
        if grid_position:
            fields["grid_position"] = grid_position
        boundary = "----gcs-" + uuid.uuid4().hex
        body = bytearray()
        for name, value in fields.items():
            body.extend(f"--{boundary}\r\nContent-Disposition: form-data; name=\"{name}\"\r\n\r\n{value}\r\n".encode())
        body.extend(f"--{boundary}\r\nContent-Disposition: form-data; name=\"file\"; filename=\"{image_type.lower()}.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n".encode())
        body.extend(encoded.tobytes())
        body.extend(f"\r\n--{boundary}--\r\n".encode())
        url = self.param("gcs_api_url").rstrip("/") + "/api/v1/images/"
        request = urllib.request.Request(url, data=bytes(body), method="POST", headers={
            "Content-Type": f"multipart/form-data; boundary={boundary}",
            "X-API-Key": str(self.param("jetson_api_key")),
        })
        try:
            with urllib.request.urlopen(request, timeout=15) as result:
                self.get_logger().info("Uploaded %s image: HTTP %s", image_type, result.status)
        except (urllib.error.URLError, urllib.error.HTTPError) as exc:
            self.get_logger().error("Upload %s failed: %s", image_type, exc)

    def destroy_node(self):
        if self.stream_pipeline is not None:
            self.stream_pipeline.set_state(Gst.State.NULL)
        self.executor.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main():
    logging.basicConfig(level=logging.INFO)
    rclpy.init()
    node = GcsMediaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
