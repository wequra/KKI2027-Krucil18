#!/usr/bin/env python3
"""Conservative state-gated underwater, surface, or docking vision controller."""

import os
import time
from datetime import datetime, timezone

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from msgs.msg import MissionStatus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

MODES = {
    'underwater': (MissionStatus.UNDERWATER_IMAGING, '/asv/mark_underwater_image'),
    'surface': (MissionStatus.SURFACE_IMAGING, '/asv/mark_surface_image'),
    'docking': (MissionStatus.DOCKING, '/asv/complete_docking'),
}


class Controller(Node):
    """Vision-based behavioral controller for imaging and docking tasks."""

    def __init__(self):
        super().__init__('mission_controller_node')
        self.mode = str(self.declare_parameter('controller_mode', 'underwater').value)
        if self.mode not in MODES:
            raise ValueError('controller_mode must be underwater, surface, or docking')

        self.required_state, service = MODES[self.mode]
        default_topic = '/underwater_camera/image_raw' if self.mode == 'underwater' else '/camera/image_raw'
        self.image_topic = str(self.declare_parameter('image_topic', default_topic).value)
        self.output_topic = '/asv/cmd_vel/' + self.mode

        self.search_timeout = float(
            self.declare_parameter('search_timeout_sec', 10.0 if self.mode == 'underwater' else 20.0).value
        )
        self.search_linear = float(self.declare_parameter('search_linear_speed', 0.05).value)
        self.search_angular = float(self.declare_parameter('search_angular_speed', 0.12).value)
        self.center_tol = float(self.declare_parameter('center_tolerance_ratio', 0.10).value)
        self.stabilize = float(self.declare_parameter('stabilize_sec', 0.75).value)
        self.capture_width = float(self.declare_parameter('target_width_ratio_capture', 0.25).value)
        self.max_speed = float(self.declare_parameter('max_mission_forward_speed', 0.8).value)
        self.near_ratio = float(self.declare_parameter('dock_near_speed_ratio', 0.40).value)
        self.final_ratio = float(self.declare_parameter('dock_final_speed_ratio', 0.20).value)
        self.near_size = float(self.declare_parameter('dock_near_size_threshold', 0.18).value)
        self.contact_size = float(self.declare_parameter('dock_contact_size_threshold', 0.38).value)
        self.min_area = float(self.declare_parameter('min_target_area', 700.0).value)
        self.consistent_frames = int(self.declare_parameter('consistent_detection_frames', 3).value)
        self.output_dir = os.path.expanduser(
            str(self.declare_parameter('output_directory', '~/dagozilla_logs/images/' + self.mode).value)
        )

        self.bridge = CvBridge()
        self.state = MissionStatus.IDLE
        self.frame = None
        self.started = None
        self.stabilizing = None
        self.hits = 0
        self.done = False
        self.target = None
        self.near_seen = False

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.client = self.create_client(Trigger, service)
        self.create_subscription(MissionStatus, '/asv/mission_status', self.on_state, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.create_timer(0.1, self.tick)

    def on_state(self, msg):
        if msg.state != self.state:
            self.state = msg.state
            self.started = time.monotonic() if self.active() else None
            self.near_seen = False
            self.stabilizing = None
            self.hits = 0
            self.done = False
            self.pub.publish(Twist())

    def active(self):
        return self.state == self.required_state

    def on_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
        except Exception as e:
            self.get_logger().warning(f'Invalid image: {e}')
            return
        self.target = self.detect(self.frame)
        self.hits = (self.hits + 1) if self.target else 0

    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lo, hi = (
            ((95, 80, 45), (135, 255, 255))
            if self.mode in ('underwater', 'docking')
            else ((35, 80, 55), (85, 255, 255))
        )
        mask = cv2.inRange(hsv, lo, hi)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        best = None

        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if area < self.min_area or w < 1 or h < 1:
                continue
            fill = area / (w * h)
            aspect = w / h
            if self.mode != 'docking' and (fill < 0.65 or not (0.45 <= aspect <= 3.0)):
                continue
            if best is None or area > best[0]:
                best = (area, x, y, w, h)
        return best

    def command(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def finish(self, fallback=False):
        self.command()
        if self.mode != 'docking':
            if self.frame is None:
                self.get_logger().error(f'{self.mode} capture failed: no valid frame')
                return
            os.makedirs(self.output_dir, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
            path = os.path.join(self.output_dir, f'{self.mode}_{stamp}.jpg')
            if not cv2.imwrite(path, self.frame):
                self.get_logger().error(f'Capture write failed: {path}')
                return
            meta = path + '.json'
            with open(meta, 'w') as f:
                fallback_str = str(fallback).lower()
                basename = os.path.basename(path)
                f.write(
                    f'{{"capture_type":"{self.mode}","timestamp":"{stamp}",'
                    f'"success":true,"fallback":{fallback_str},"filename":"{basename}"}}\n'
                )
            self.get_logger().info(f"Saved {self.mode}{' fallback' if fallback else ''}: {path}")

        self.done = True
        if self.client.service_is_ready():
            self.client.call_async(Trigger.Request())
        else:
            self.get_logger().error('Transition service unavailable; holding STOP')

    def tick(self):
        if not self.active() or self.done:
            self.command()
            return

        now = time.monotonic()
        elapsed = now - (self.started or now)

        if self.mode == 'underwater':
            if self.hits >= self.consistent_frames and self.target:
                _, x, _, w, _ = self.target
                center_offset = x + w / 2 - self.frame.shape[1] / 2
                err = center_offset / (self.frame.shape[1] / 2)
                if abs(err) > self.center_tol:
                    self.command(0.03, -0.25 * err)
                    return
                if self.stabilizing is None:
                    self.stabilizing = now
                    self.command()
                    return
                if now - self.stabilizing >= self.stabilize:
                    self.finish(False)
                else:
                    self.command()
            elif elapsed >= self.search_timeout:
                if self.stabilizing is None:
                    self.stabilizing = now
                    self.command()
                elif now - self.stabilizing >= self.stabilize:
                    self.finish(True)
            else:
                oscillation = self.search_angular if int(elapsed / 2) % 2 == 0 else -self.search_angular
                self.command(self.search_linear, oscillation)

        elif self.mode == 'surface':
            self.surface_tick(now, elapsed)
        else:
            self.dock_tick()

    def surface_tick(self, now, elapsed):
        if not self.target:
            if self.near_seen:
                self.command(0.0, 0.0)
            else:
                self.command(self.search_linear, self.search_angular)
            if elapsed >= self.search_timeout:
                if self.stabilizing is None:
                    self.stabilizing = now
                elif now - self.stabilizing >= self.stabilize:
                    self.finish(True)
            return

        _, x, _, w, _ = self.target
        ratio = w / self.frame.shape[1]
        center_offset = x + w / 2 - self.frame.shape[1] / 2
        err = center_offset / (self.frame.shape[1] / 2)

        if ratio >= min(self.capture_width * 0.75, 0.18):
            self.near_seen = True

        if ratio >= self.capture_width and abs(err) <= self.center_tol:
            if self.stabilizing is None:
                self.stabilizing = now
            self.command()
            if now - self.stabilizing >= self.stabilize:
                self.finish(False)
        else:
            self.stabilizing = None
            forward_speed = 0.12 if abs(err) <= 0.35 and ratio < self.capture_width else 0.0
            self.command(forward_speed, -0.35 * err)

    def dock_tick(self):
        if not self.target:
            self.command()
            return

        _, x, _, w, _ = self.target
        ratio = w / self.frame.shape[1]
        center_offset = x + w / 2 - self.frame.shape[1] / 2
        err = center_offset / (self.frame.shape[1] / 2)

        if ratio >= self.contact_size:
            self.finish()
            return

        speed = self.max_speed * 0.65
        if ratio >= self.near_size:
            speed = self.max_speed * self.near_ratio
        if ratio >= self.contact_size * 0.75:
            speed = self.max_speed * self.final_ratio

        forward_speed = speed if abs(err) < 0.4 else 0.0
        self.command(forward_speed, -0.40 * err)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.command()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
