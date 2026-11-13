#!/usr/bin/env python3
"""YOLO buoy detector adapted to Dagozilla's existing obstacle interface."""

from pathlib import Path
import uuid
import os

import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from msgs.msg import ObstacleArray, TrackedObstacleArray

try:
    # Never modify the deployment environment from inside inference/tracking.
    os.environ.setdefault("YOLO_AUTOINSTALL", "false")
    from ultralytics import YOLO
except ImportError:
    YOLO = None


CLASS_TO_OBSTACLE = {
    "red buoy": ObstacleArray.RED_BUOY,
    "green buoy": ObstacleArray.GREEN_BUOY,
}


class BallDetectorNode(Node):
    """Detect navigation buoys with YOLO and publish ``msgs/ObstacleArray``."""

    def __init__(self):
        super().__init__("ball_detector_node")
        default_weights = Path(get_package_share_directory("perception")) / "weights" / "best.pt"
        self.weights_path = Path(self.declare_parameter(
            "weights_path", str(default_weights)).value)
        self.confidence_threshold = float(self.declare_parameter(
            "confidence_threshold", 0.25).value)
        self.iou_threshold = float(self.declare_parameter("iou_threshold", 0.45).value)
        self.device = str(self.declare_parameter("device", "cpu").value)
        self.use_object_tracking = bool(self.declare_parameter("use_object_tracking", True).value)
        self.tracker_type = str(self.declare_parameter("tracker_type", "bytetrack.yaml").value)
        if self.tracker_type not in ("bytetrack.yaml", "botsort.yaml"):
            raise ValueError("tracker_type must be bytetrack.yaml or botsort.yaml")
        self.tracker_session = str(uuid.uuid4())

        if YOLO is None:
            raise RuntimeError("ultralytics is not installed; install the Dagozilla Docker image dependencies")
        if not self.weights_path.is_file():
            raise RuntimeError(f"YOLO weights not found: {self.weights_path}")

        self.get_logger().info(f"Loading YOLO buoy model: {self.weights_path}")
        self.model = YOLO(str(self.weights_path))
        self.class_names = self.model.names
        self.get_logger().info(f"YOLO model classes: {self.class_names}")

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, "camera/image_raw", self.image_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.obstacle_pub = self.create_publisher(ObstacleArray, "obstacles", 10)
        self.tracked_pub = self.create_publisher(TrackedObstacleArray, "/asv/tracked_obstacles", 1)
        self.debug_image_pub = self.create_publisher(Image, "/asv/camera_debug", 10)
        self.get_logger().info(
            "YOLO buoy detector ready: Red Buoy and Green Buoy publish to obstacles; "
            "Green Box is ignored.")

    def class_name(self, class_index):
        if isinstance(self.class_names, dict):
            return str(self.class_names.get(class_index, class_index))
        if 0 <= class_index < len(self.class_names):
            return str(self.class_names[class_index])
        return str(class_index)

    def image_callback(self, image_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            infer = self.model.track if self.use_object_tracking else self.model.predict
            tracking = {"persist": True, "tracker": self.tracker_type} if self.use_object_tracking else {}
            results = infer(
                frame,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                device=self.device,
                verbose=False,
                **tracking,
            )
        except Exception as error:
            self.get_logger().error(f"YOLO inference failed: {error}", throttle_duration_sec=5.0)
            return

        obstacles = ObstacleArray()
        obstacles.header = image_msg.header
        debug_frame = frame.copy()
        track_ids = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_index = int(box.cls.item())
                label = self.class_name(class_index)
                obstacle_class = CLASS_TO_OBSTACLE.get(label.casefold())
                if obstacle_class is None:
                    continue

                x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
                confidence = float(box.conf.item())
                width = max(0.0, x2 - x1)
                height = max(0.0, y2 - y1)

                point = Point()
                point.x = (x1 + x2) / 2.0
                point.y = (y1 + y2) / 2.0
                obstacles.positions.append(point)
                obstacles.radii.append((width + height) / 4.0)
                obstacles.classes.append(obstacle_class)
                obstacles.confidences.append(confidence)
                track_id = int(box.id.item()) if box.id is not None else -1
                track_ids.append(track_id)

                color = (0, 0, 255) if obstacle_class == ObstacleArray.RED_BUOY else (0, 255, 0)
                cv2.rectangle(debug_frame, (round(x1), round(y1)), (round(x2), round(y2)), color, 2)
                cv2.putText(
                    debug_frame, f"{label} {confidence:.2f} ID:{track_id}", (round(x1), max(18, round(y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

        self.obstacle_pub.publish(obstacles)
        tracked = TrackedObstacleArray()
        tracked.obstacles = obstacles
        tracked.tracker_session = self.tracker_session
        tracked.track_ids = track_ids
        tracked.image_width = frame.shape[1]
        tracked.image_height = frame.shape[0]
        self.tracked_pub.publish(tracked)
        debug_message = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
        debug_message.header = image_msg.header
        self.debug_image_pub.publish(debug_message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BallDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
