import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import Point
from msgs.msg import ControllerDiagnostic, ObstacleArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import UInt8


def wait(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class Probe(Node):
    def __init__(self):
        super().__init__('buoy_target_locking_probe')
        self.diagnostics = []
        self.gates = []
        self.obstacles = self.create_publisher(ObstacleArray, '/asv/obstacles', 10)
        self.create_subscription(
            ControllerDiagnostic, '/asv/control_diagnostic/buoy',
            self.diagnostics.append, 50)
        self.create_subscription(UInt8, '/asv/buoy_gate_count',
                                 lambda message: self.gates.append(message.data), 50)

    def publish(self, detections):
        message = ObstacleArray()
        for x, y, radius, classification in detections:
            message.positions.append(Point(x=float(x), y=float(y)))
            message.radii.append(float(radius))
            message.classes.append(classification)
            message.confidences.append(0.9)
        self.obstacles.publish(message)

    def empty(self):
        self.publish([])


RED = ObstacleArray.RED_BUOY
GREEN = ObstacleArray.GREEN_BUOY


def gate(red_x, green_x, radius_red, radius_green, y=240.0):
    return [
        (red_x, y, radius_red, RED),
        (green_x, y, radius_green, GREEN),
    ]


def two_gates(first, second):
    return first + second


def wait_diagnostic(probe, state, detected=None, timeout=3.0):
    def matches():
        if not probe.diagnostics:
            return False
        diagnostic = probe.diagnostics[-1]
        return diagnostic.controller_state == state and (
            detected is None or diagnostic.target_detected == detected)
    assert wait(matches, timeout)
    return probe.diagnostics[-1]


@pytest.fixture
def rig():
    command = [
        'ros2', 'run', 'guidance_control', 'buoy_centering_node', '--ros-args',
        '-p', 'show_gui:=false',
        '-p', 'use_object_tracking:=false',
        '-p', 'candidate_confirm_frames:=1',
        '-p', 'approaching_lost_frames_tolerance:=3',
        '-p', 'lost_frames_for_pass:=2',
        '-p', 'passed_gate_reacquire_block_frames:=3',
    ]
    process = subprocess.Popen(command, start_new_session=True, stdout=subprocess.DEVNULL)
    rclpy.init()
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert wait(lambda: probe.count_publishers('/asv/buoy_gate_count') == 1, 8)
    yield probe
    executor.shutdown()
    probe.destroy_node()
    rclpy.shutdown()
    os.killpg(process.pid, signal.SIGTERM)
    process.wait(timeout=5)


def test_single_normal_gate_transitions_to_passed(rig):
    probe = rig
    probe.publish(gate(180, 340, 35, 35))
    wait_diagnostic(probe, 'APPROACHING', True)
    probe.publish(gate(180, 340, 60, 60))
    wait_diagnostic(probe, 'PASSING')
    probe.empty()
    probe.empty()
    assert wait(lambda: max(probe.gates or [0]) == 1)


def test_locked_target_survives_loss_and_advances_sequential_gates(rig):
    probe = rig

    # 1-3: lock a normal gate, tolerate a short dropout, then release after
    # permanent loss. The original steering sign is asserted as a regression.
    probe.publish(gate(80, 240, 35, 35))
    initial = wait_diagnostic(probe, 'APPROACHING', True)
    assert initial.angular_z > 0.5
    probe.empty()
    wait_diagnostic(probe, 'APPROACHING', False)
    probe.empty()
    wait_diagnostic(probe, 'APPROACHING', False)
    probe.publish(gate(90, 250, 38, 38))
    resumed = wait_diagnostic(probe, 'APPROACHING', True)
    assert resumed.angular_z > 0.5
    for _ in range(3):
        probe.empty()
        time.sleep(0.06)
    wait_diagnostic(probe, 'SEARCHING')

    # 4-5: two simultaneous gates. The cross pair R1+G2 has the largest
    # radius score, but mutual-nearest selection must lock R1+G1 instead.
    gate_one = gate(80, 220, 45, 35, y=300)
    gate_two = gate(400, 540, 34, 50, y=200)
    probe.publish(two_gates(gate_one, gate_two))
    selected = wait_diagnostic(probe, 'APPROACHING', True)
    assert selected.angular_z > 0.5

    # Make Gate 2 globally larger on the next frame. The locked Gate 1 must
    # still determine the command, proving no switch or cross-pair occurs.
    gate_one = gate(90, 230, 45, 35)
    gate_two = gate(400, 540, 60, 60)
    probe.publish(two_gates(gate_one, gate_two))
    locked = wait_diagnostic(probe, 'APPROACHING', True)
    assert locked.angular_z > 0.5

    # 1: the locked Gate 1 reaches the existing passing threshold.
    gate_one_near = gate(90, 230, 60, 60)
    probe.publish(two_gates(gate_one_near, gate_two))
    wait_diagnostic(probe, 'PASSING')

    # 6: Gate 2 is similarly sized but spatially different. It cannot replace
    # the passing target, so disappearance of Gate 1 completes exactly one gate.
    for _ in range(2):
        probe.publish(gate_two)
        time.sleep(0.08)
    assert wait(lambda: max(probe.gates or [0]) == 1)

    # 7: Gate 1 briefly reappears by itself. The controller stays SEARCHING,
    # does not reacquire it, and does not recount it.
    for _ in range(2):
        probe.publish(gate_one_near)
        time.sleep(0.06)
    wait_diagnostic(probe, 'SEARCHING')
    assert max(probe.gates or [0]) == 1

    # Gate 2 is then free to acquire as the next target.
    probe.publish(gate(400, 540, 35, 35))
    second_selected = wait_diagnostic(probe, 'APPROACHING', True)
    assert second_selected.angular_z < -0.5
    for _ in range(2):
        probe.publish(gate_one_near)
        time.sleep(0.06)
    assert max(probe.gates or [0]) == 1

    # 8: reacquire Gate 2 after Gate 1 is gone, pass it, and count once more.
    probe.publish(gate(400, 540, 35, 35))
    wait_diagnostic(probe, 'APPROACHING', True)
    probe.publish(gate(400, 540, 60, 60))
    wait_diagnostic(probe, 'PASSING')
    probe.empty()
    probe.empty()
    assert wait(lambda: max(probe.gates or [0]) == 2)
