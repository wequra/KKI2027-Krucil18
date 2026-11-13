#!/usr/bin/env python3
"""Terminal keyboard teleop for manual Dagozilla ASV testing.

This node uses terminal key-repeat as a held-key signal. When the repeat stops
for ``key_timeout`` seconds, the command ramps back to zero. Explicit stop
keys always bypass that ramp and publish zero immediately. It deliberately
does not arbitrate with any other /asv/cmd_vel publisher.
"""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MOVEMENT_KEYS = {
    "w": ("W", "linear", 1.0),
    "s": ("S", "linear", -1.0),
    "a": ("A", "angular", 1.0),
    "d": ("D", "angular", -1.0),
}
STOP_KEYS = {"x", " "}


class KeyboardTeleopNode(Node):
    """Publish /asv/cmd_vel from one held terminal movement key at a time."""

    def __init__(self):
        super().__init__("keyboard_teleop_node")

        self.max_linear_speed = abs(float(self.declare_parameter(
            "max_linear_speed", 1.0).value))
        self.max_angular_speed = abs(float(self.declare_parameter(
            "max_angular_speed", 1.0).value))
        self.linear_increment = abs(float(self.declare_parameter(
            "linear_increment", 0.05).value))
        self.angular_increment = abs(float(self.declare_parameter(
            "angular_increment", 0.05).value))
        self.publish_rate = float(self.declare_parameter(
            "publish_rate", 20.0).value)
        self.key_timeout = float(self.declare_parameter(
            "key_timeout", 0.12).value)

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if self.key_timeout <= 0.0:
            raise ValueError("key_timeout must be greater than zero")

        self.publisher = self.create_publisher(Twist, "/asv/cmd_vel", 10)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.active_key = None
        self.last_key_time = 0.0
        self._terminal_fd = None
        self._terminal_settings = None
        self._terminal_restored = False
        self._stopping = False

        self._configure_terminal()
        self._print_help()
        self._publish_zero()
        self.timer = self.create_timer(1.0 / self.publish_rate, self._tick)

    def _configure_terminal(self):
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard_teleop_node requires an interactive terminal")
        self._terminal_fd = sys.stdin.fileno()
        self._terminal_settings = termios.tcgetattr(self._terminal_fd)
        tty.setcbreak(self._terminal_fd)

    def restore_terminal(self):
        if self._terminal_restored or self._terminal_fd is None:
            return
        termios.tcsetattr(self._terminal_fd, termios.TCSADRAIN, self._terminal_settings)
        self._terminal_restored = True
        sys.stdout.write("\n")
        sys.stdout.flush()

    @staticmethod
    def _zero_twist():
        return Twist()

    def _publish(self):
        message = Twist()
        message.linear.x = self.linear_x
        message.angular.z = self.angular_z
        self.publisher.publish(message)

    def _publish_zero(self):
        self.publisher.publish(self._zero_twist())

    def _publish_zero_burst(self):
        # Multiple transmissions make a stop more robust if a downstream
        # subscriber joins late or one packet is lost.
        for _ in range(3):
            self._publish_zero()
            time.sleep(min(0.05, 1.0 / self.publish_rate))

    def _release_movement_key(self):
        self.active_key = None

    def _emergency_stop(self):
        self._release_movement_key()
        self.linear_x = 0.0
        self.angular_z = 0.0

    def _read_keys(self):
        while select.select([sys.stdin], [], [], 0.0)[0]:
            raw = os.read(self._terminal_fd, 1)
            if not raw:
                return
            key = raw.decode(errors="ignore").lower()
            if key == "q":
                self._stop_and_shutdown()
                return
            if key in STOP_KEYS:
                self._emergency_stop()
                self._publish_zero_burst()
                continue
            if key in MOVEMENT_KEYS:
                # Terminal input has no reliable key-up events. Use only the
                # latest repeated movement key; optional multi-key control is
                # intentionally not guessed from incomplete repeat events.
                if key != self.active_key:
                    self.linear_x = 0.0
                    self.angular_z = 0.0
                self.active_key = key
                self.last_key_time = time.monotonic()

    def _apply_ramp(self):
        if self.active_key is None:
            return
        _, axis, direction = MOVEMENT_KEYS[self.active_key]
        if axis == "linear":
            self.linear_x = max(
                -self.max_linear_speed,
                min(self.max_linear_speed, self.linear_x + direction * self.linear_increment),
            )
            self.angular_z = 0.0
        else:
            self.angular_z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, self.angular_z + direction * self.angular_increment),
            )
            self.linear_x = 0.0

    @staticmethod
    def _ramp_toward_zero(value, increment):
        if abs(value) <= increment:
            return 0.0
        return value - increment if value > 0.0 else value + increment

    def _apply_release_ramp(self):
        self.linear_x = self._ramp_toward_zero(self.linear_x, self.linear_increment)
        self.angular_z = self._ramp_toward_zero(self.angular_z, self.angular_increment)

    def _write_status(self):
        if self.active_key:
            key_label = MOVEMENT_KEYS[self.active_key][0]
        elif self.linear_x != 0.0 or self.angular_z != 0.0:
            key_label = "RELEASING"
        else:
            key_label = "RELEASED"
        sys.stdout.write(
            f"\r\033[KKEY: {key_label} | linear.x: {self.linear_x:+.2f} | "
            f"angular.z: {self.angular_z:+.2f}"
        )
        sys.stdout.flush()

    def _tick(self):
        try:
            self._read_keys()
            if not rclpy.ok():
                return
            if self.active_key is not None and (
                time.monotonic() - self.last_key_time > self.key_timeout
            ):
                self._release_movement_key()
            if self.active_key is None:
                self._apply_release_ramp()
            else:
                self._apply_ramp()
            self._publish()
            self._write_status()
        except Exception as error:  # Ensure terminal/motors are safe on I/O failures.
            self.get_logger().error(f"Keyboard teleop error: {error}")
            self._stop_and_shutdown()

    def _stop_and_shutdown(self):
        if self._stopping:
            return
        self._stopping = True
        self._emergency_stop()
        self._publish_zero_burst()
        self.restore_terminal()
        if rclpy.ok():
            rclpy.shutdown()

    def _print_help(self):
        print(
            "\n========================================\n"
            "       DAGOZILLA ASV KEYBOARD TELEOP\n"
            "========================================\n\n"
            "HOLD KEY TO MOVE\n\n"
            "W : Forward\nS : Reverse\nA : Left\nD : Right\n\n"
            "X / SPACE : STOP\nQ         : Quit\n\n"
            "Linear  : -1.0 ... +1.0\nAngular : -1.0 ... +1.0\n\n"
            "Release movement key = RAMP TO STOP\n\n"
            "WARNING: Do not run an autonomous /asv/cmd_vel publisher at the same time.\n"
            "         This node does not arbitrate or disable other publishers.\n\n"
            "========================================"
        )

    def shutdown_safely(self):
        self._emergency_stop()
        # Q may already have shut down rclpy after sending its stop burst.
        # Ctrl+C and ordinary exceptions still send one here while valid.
        if rclpy.ok():
            self._publish_zero_burst()
        self.restore_terminal()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"keyboard_teleop_node failed safely: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.shutdown_safely()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
