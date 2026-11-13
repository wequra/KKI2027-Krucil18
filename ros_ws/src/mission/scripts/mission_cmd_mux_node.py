#!/usr/bin/env python3
"""State-gated command mux; sole publisher to the hardware command topic."""
import time
import rclpy
from geometry_msgs.msg import Twist
from msgs.msg import MissionStatus
from rclpy.node import Node

SOURCES = {
    MissionStatus.NAVIGATING_BUOYS: ('/asv/cmd_vel/buoy', 'buoy'),
    MissionStatus.UNDERWATER_IMAGING: ('/asv/cmd_vel/underwater', 'underwater'),
    MissionStatus.SURFACE_IMAGING: ('/asv/cmd_vel/surface', 'surface'),
    MissionStatus.DOCKING: ('/asv/cmd_vel/docking', 'docking'),
}

class MissionCmdMux(Node):
    def __init__(self):
        super().__init__('mission_cmd_mux_node')
        self.timeout = float(self.declare_parameter('command_timeout_sec', 0.5).value)
        self.state = MissionStatus.IDLE
        self.state_time = 0.0
        self.state_timeout = float(self.declare_parameter("mission_state_timeout_sec", 2.5).value)
        self.commands = {}
        self.pub = self.create_publisher(Twist, '/asv/cmd_vel', 10)
        self.create_subscription(MissionStatus, '/asv/mission_status', self.on_state, 10)
        for state, (topic, _) in SOURCES.items():
            self.create_subscription(Twist, topic, lambda msg, s=state: self.on_cmd(s, msg), 10)
        self.create_timer(0.1, self.tick)

    def on_state(self, msg):
        self.state_time = time.monotonic()
        if msg.state != self.state:
            self.state = msg.state
            self.pub.publish(Twist())  # neutral at every ownership handoff

    def on_cmd(self, state, msg):
        self.commands[state] = (msg, time.monotonic())

    def tick(self):
        if time.monotonic() - self.state_time > self.state_timeout:
            self.pub.publish(Twist())
            return
        item = self.commands.get(self.state)
        self.pub.publish(item[0] if item and time.monotonic() - item[1] <= self.timeout else Twist())

def main(args=None):
    rclpy.init(args=args)
    node = MissionCmdMux()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
