#!/usr/bin/env python3
"""Publish synthetic red/green gate images for a deterministic Docker smoke test."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8


class SyntheticBuoyCamera(Node):
    def __init__(self):
        super().__init__('synthetic_buoy_camera')
        self.declare_parameter('total_gates', 10)
        self.declare_parameter('approach_frames', 8)
        self.declare_parameter('publish_rate', 5.0)

        self.total_gates = max(1, self.get_parameter('total_gates').value)
        self.approach_frames = max(2, self.get_parameter('approach_frames').value)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.gate_count_subscription = self.create_subscription(
            UInt8, '/asv/buoy_gate_count', self.gate_count_callback, 10)
        self.command_subscription = self.create_subscription(
            Twist, '/asv/cmd_vel', self.command_callback, 10)
        self.gate_index = 0
        self.phase = 0
        self.passing_started = False
        self.finished_logged = False
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)
        self.get_logger().info(
            f'Synthetic camera aktif: {self.total_gates} gate, {rate:.1f} FPS')

    def gate_count_callback(self, msg):
        if msg.data > self.gate_index:
            self.gate_index = msg.data
            self.phase = 0
            self.passing_started = False

    def command_callback(self, msg):
        if (self.phase >= self.approach_frames - 1 and
                abs(msg.linear.x - 0.30) < 0.02):
            self.passing_started = True

    def publish_frame(self):
        frame = np.full((240, 320, 3), 25, dtype=np.uint8)

        if self.gate_index < self.total_gates and not self.passing_started:
            capped_phase = min(self.phase, self.approach_frames - 1)
            progress = capped_phase / float(self.approach_frames - 1)
            radius = int(20 + progress * 42)
            center_shift = int(20 * np.sin((self.gate_index + self.phase / 5.0) * 0.7))
            red_center = (80 + center_shift, 140)
            green_center = (240 + center_shift, 140)
            cv2.circle(frame, red_center, radius, (0, 0, 255), -1)
            cv2.circle(frame, green_center, radius, (0, 255, 0), -1)
            cv2.putText(
                frame, f'SYNTHETIC GATE {self.gate_index + 1}/{self.total_gates}',
                (35, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        elif self.gate_index >= self.total_gates and not self.finished_logged:
            self.get_logger().info('Semua frame gate sintetis selesai; menerbitkan frame kosong.')
            self.finished_logged = True

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'synthetic_camera_frame'
        self.publisher.publish(msg)
        self.phase += 1


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticBuoyCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass


if __name__ == '__main__':
    main()
