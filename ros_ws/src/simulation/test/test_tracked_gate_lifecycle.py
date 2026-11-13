"""Target ROS 2 Jazzy tests. No camera, YOLO, MAVROS or motor nodes launched."""
import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import Point, Twist
from msgs.msg import TrackedObstacleArray
from std_msgs.msg import Float64, String
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor


class Probe(Node):
    def __init__(self):
        super().__init__('tracked_gate_probe')
        self.debug = ''
        self.commands = []
        self.pub = self.create_publisher(TrackedObstacleArray, '/asv/tracked_obstacles', 1)
        self.hdg = self.create_publisher(Float64, '/mavros/global_position/compass_hdg', 1)
        self.create_subscription(String, '/asv/gate_debug', self.on_debug, 10)
        self.create_subscription(Twist, '/test/buoy_cmd', self.commands.append, 10)

    def on_debug(self, message):
        self.debug = message.data

    def frame(self, objects=(), heading=350., session='test-session'):
        self.hdg.publish(Float64(data=float(heading)))
        message = TrackedObstacleArray()
        message.obstacles.header.stamp = self.get_clock().now().to_msg()
        message.tracker_session = session
        message.image_width, message.image_height = 640, 480
        for identity, color, x, y, radius in objects:
            message.track_ids.append(identity)
            message.obstacles.positions.append(Point(x=float(x), y=float(y)))
            message.obstacles.radii.append(float(radius))
            message.obstacles.classes.append(color)
            message.obstacles.confidences.append(.9)
        self.pub.publish(message)
        time.sleep(.08)

    def feed(self, objects=(), heading=350., frames=4, session='test-session'):
        for _ in range(frames):
            self.frame(objects, heading, session)

    def expect(self, text):
        deadline = time.monotonic() + 2
        while text not in self.debug and time.monotonic() < deadline:
            time.sleep(.02)
        assert text in self.debug, self.debug


def gate(identity=1, x=180, radius=30, y=260):
    return [(identity, 1, x, y, radius), (identity+1, 2, x+140, y, radius)]


@pytest.fixture(params=['A', 'B'])
def rig(request):
    process = subprocess.Popen([
        'ros2', 'run', 'guidance_control', 'buoy_centering_node', '--ros-args',
        '-p', 'show_gui:=false', '-p', 'candidate_confirm_frames:=3',
        '-p', 'tracked_target_lost_frames:=4', '-p', 'lost_frames_for_pass:=2',
        '-p', 'gate_3_clear_time_s:=0.6', '-p', 'gate_7_clear_time_s:=0.6',
        '-p', 'turn_stable_samples:=3', '-p', 'heading_timeout_s:=0.3',
        '-p', f'course_layout:={request.param}',
        '-r', '/asv/cmd_vel:=/test/buoy_cmd'], start_new_session=True)
    rclpy.init()
    probe = Probe()
    probe.layout = request.param
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 10
        while probe.pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(.05)
        assert probe.pub.get_subscription_count() == 1
        yield probe
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        probe.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def test_lock_cross_pair_occlusion_loss(rig):
    p = rig
    first = gate(1, 100)
    second = gate(11, 420, 29)
    p.frame(first + second)
    p.expect('state=SEARCHING')  # one frame cannot lock
    p.feed(first + second)
    p.expect('locked=1/2')
    p.expect('candidates=1/2,11/12,')
    p.feed(first + gate(11, 420, 60))
    p.expect('locked=1/2')
    p.feed(first[:1] + second, frames=2)
    p.expect('state=APPROACHING')
    p.expect('locked=1/2')
    p.feed(first + second, frames=1)
    p.expect('lost=0/0')
    # Same-session ID churn is reassociated only when gate geometry still matches.
    reassociated = [(99, *first[0][1:]), first[1]]
    p.frame(reassociated + second)
    p.expect('locked=99/2')
    p.expect('reassociated=1->99')
    # An implausible new red ID cannot steal the surviving green constituent.
    wrong_red = [(100, 1, 520, 100, 10), first[1]]
    p.feed(wrong_red + second, frames=2)
    p.expect('locked=99/2')
    p.expect('partial_recovery=1')
    p.expect('gate_count=0')



def test_rational_target_switch_hysteresis(rig):
    p = rig
    far = gate(1, 180, 20, 185)
    near = gate(11, 230, 45, 315)
    p.feed(far)
    p.expect('locked=1/2')
    p.feed(far + near, frames=2)
    p.expect('locked=1/2')
    p.expect('switch_confirm=2')
    p.frame(far + near)
    p.expect('locked=11/12')
    p.expect('switch_reason=switched_rational_near_gate')


def test_loss_guards_and_partial_recovery(rig):
    p = rig
    current = gate(21, 180, 30, 260)
    p.feed(current)
    p.feed(current[:1], frames=2)
    p.expect('state=APPROACHING')
    p.expect('partial_recovery=1')
    p.expect('recovery_missing=GREEN')
    p.expect('gate_count=0')
    p.feed([], frames=4)
    p.expect('state=SEARCHING')
    p.expect('gate_count=0')

    near = gate(31, 180, 60, 300)
    p.feed(near)
    p.frame(near)
    p.expect('state=PASSING')
    p.feed(near[:1], frames=4)
    p.expect('state=PASSING')
    p.expect('gate_count=0')
    p.feed([], frames=2)
    p.expect('gate_count=1')

def test_retirement_and_tracker_reset(rig):
    p = rig
    p.feed(gate())
    p.feed(gate(radius=60), frames=1)
    p.expect('state=PASSING')
    p.feed([], frames=2)
    p.expect('gate_count=1')
    p.feed(gate(radius=60), frames=8)
    p.expect('state=SEARCHING')
    # New IDs/session at the retired image footprint remain suppressed.
    p.feed(gate(91, radius=60), session='restarted')
    p.expect('state=SEARCHING')
    p.feed(gate(11, 440, 30))
    p.expect('locked=11/12')
    # A camera outage must stop and must not be counted as a passed gate.
    time.sleep(1.2)
    assert p.commands[-1].linear.x == p.commands[-1].angular.z == 0
    p.feed([])
    p.expect('gate_count=1')


def test_first_gate_asymmetric(rig):
    p = rig
    first = [(1, 1, 210, 310, 50), (2, 2, 350, 310, 25)]
    p.feed(first + gate(11, 430, 36, 220))
    p.expect('locked=1/2')


def test_gate_3_7_maneuver_and_normal_gates(rig):
    p = rig
    heading = 350.
    for number in range(1, 11):
        # Radius 25 differs from retired radius 60 beyond suppression ratio.
        current = gate(number*10, 180, 25)
        p.feed(current, heading)
        p.expect(f'locked={number*10}/{number*10+1}')
        p.frame(gate(number*10, 180, 60), heading)
        p.expect('state=PASSING')
        p.feed([], heading, frames=2)
        p.expect(f'gate_count={number}')
        if number == 10:
            p.expect('state=FINISHED')
            p.feed(gate(900, 440), heading)
            p.expect('state=FINISHED')
            assert p.commands[-1].linear.x == p.commands[-1].angular.z == 0
            continue
        if number not in (3, 7):
            p.expect('state=SEARCHING')
            continue
        p.expect('state=CLEARING_GATE')
        p.feed(gate(900, 440), heading, frames=2)
        p.expect('acquisition=OFF')
        p.expect('locked=-1/-1')
        # Hold heading until clearing's timer enters TURNING.
        deadline = time.monotonic() + 3
        while 'state=TURNING ' not in p.debug and time.monotonic() < deadline:
            p.frame(gate(900, 440), heading)
        p.expect('state=TURNING')
        p.frame(gate(900, 440), heading)
        assert p.commands[-1].angular.z * (1 if p.layout == 'A' else -1) > 0
        # Stale heading: stop and keep acquisition disabled.
        time.sleep(.45)
        assert p.commands[-1].linear.x == p.commands[-1].angular.z == 0
        p.expect('acquisition=OFF')
        heading = (heading + (-90 if p.layout == 'A' else 90)) % 360
        p.frame(gate(900, 440), heading)
        p.expect('state=TURN_STABILIZING')
        p.feed([], heading, frames=3)
        p.expect('state=SEARCHING')
        # Count 3/7 remains unchanged: must not retrigger.
        p.feed([], heading, frames=5)
        p.expect('state=SEARCHING')
