import math, os, signal, subprocess, threading, time
import pytest, rclpy
from geometry_msgs.msg import Point, Twist
from mavros_msgs.msg import OverrideRCIn
from msgs.msg import MissionStatus, ObstacleArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import UInt8, Float64
from std_srvs.srv import Trigger

def wait(p,t=3):
 e=time.monotonic()+t
 while time.monotonic()<e:
  if p(): return True
  time.sleep(.02)
 return False

class Probe(Node):
 def __init__(self):
  super().__init__('stress_probe');self.cmd=[];self.gates=[];self.states=[];self.rc=[]
  self.obs=self.create_publisher(ObstacleArray,'/asv/obstacles',10);self.vel=self.create_publisher(Twist,'/asv/cmd_vel',10)
  self.heading=self.create_publisher(Float64,'/mavros/global_position/compass_hdg',10)
  self.create_subscription(Twist,'/asv/cmd_vel/buoy',self.cmd.append,50);self.create_subscription(UInt8,'/asv/buoy_gate_count',lambda m:self.gates.append(m.data),50)
  self.create_subscription(MissionStatus,'/asv/mission_status',self.states.append,50);self.create_subscription(OverrideRCIn,'/mavros/rc/override',self.rc.append,50)
 def pair(self,r=25,g=25,rx=220.,gx=420.):
  m=ObstacleArray();m.positions=[Point(x=rx,y=240.),Point(x=gx,y=240.)];m.radii=[r,g];m.classes=[m.RED_BUOY,m.GREEN_BUOY];m.confidences=[.9,.9];self.obs.publish(m)
 def empty(self):self.obs.publish(ObstacleArray())
 def two_gates(self):
  m=ObstacleArray();m.positions=[Point(x=100.,y=240.),Point(x=220.,y=240.),Point(x=440.,y=240.),Point(x=540.,y=240.)]
  m.radii=[55.,55.,20.,20.];m.classes=[m.RED_BUOY,m.GREEN_BUOY,m.RED_BUOY,m.GREEN_BUOY];m.confidences=[.9,.9,.9,.9];self.obs.publish(m)
 def call(self,n):
  c=self.create_client(Trigger,n);assert c.wait_for_service(timeout_sec=3);f=c.call_async(Trigger.Request());assert wait(f.done);return f.result()

@pytest.fixture(params=["A", "B"])
def rig(request):
 layout=request.param
 cs=[["ros2","run","mission","mission_manager_node"],["ros2","run","guidance_control","buoy_centering_node","--ros-args","-p","show_gui:=false","-p","lost_frames_for_pass:=3","-p",f"course_layout:={layout}","-r","/asv/cmd_vel:=/asv/cmd_vel/buoy"],["ros2","run","guidance_control","cmd_vel_to_rc_override_node","--ros-args","-p","deadman_timeout:=0.25","-p","publish_rate:=20.0"]]
 # Legacy untracked regression stream; tracked lifecycle has its own black-box suite.
 cs[1] += ['-p','use_object_tracking:=false','-p','candidate_confirm_frames:=1',
           '-p','passed_gate_suppression_time_s:=0.3',
           '-p','gate_3_clear_time_s:=0.2','-p','gate_7_clear_time_s:=0.2']
 ps=[subprocess.Popen(c,start_new_session=True,stdout=subprocess.DEVNULL) for c in cs];rclpy.init();n=Probe();n.layout=layout;ex=SingleThreadedExecutor();ex.add_node(n);threading.Thread(target=ex.spin,daemon=True).start();assert wait(lambda:n.count_publishers("/asv/buoy_gate_count")==1,8);yield n;ex.shutdown();n.destroy_node();rclpy.shutdown()
 for x in ps:
  os.killpg(x.pid,signal.SIGTERM)

def test_black_box_matrix(rig):
 p=rig;p.empty();assert wait(lambda:p.cmd);assert p.cmd[-1].linear.x==pytest.approx(.3);assert p.cmd[-1].angular.z==pytest.approx(0.)
 one=ObstacleArray(positions=[Point(x=200.,y=200.)],radii=[30.],classes=[ObstacleArray.RED_BUOY],confidences=[.9]);p.obs.publish(one);time.sleep(.1);assert p.cmd[-1].linear.x==pytest.approx(.3);assert p.cmd[-1].angular.z==pytest.approx(0.)
 for _ in range(4):p.pair(rx=0.,gx=100.)
 for _ in range(3):p.empty()
 for _ in range(5):p.pair(rx=540.,gx=640.)
 time.sleep(.1);assert all(math.isfinite(x.linear.x) and math.isfinite(x.angular.z) and abs(x.angular.z)<=1 and abs(x.linear.x)<=.8 for x in p.cmd)
 before=max(p.gates or [0])
 for _ in range(4):p.pair(70.,15.)
 for _ in range(3):p.empty()
 time.sleep(.1);assert max(p.gates or [0])==before;assert p.call("/asv/start_mission").success
 # The near gate is selected from two simultaneous gates. Once it is passed,
 # the smaller distant gate remains visible and must not block the count.
 for _ in range(2):p.two_gates();time.sleep(.08)
 for _ in range(3):p.pair(20.,20.,440.,540.);time.sleep(.08)
 assert wait(lambda:max(p.gates or [0])==1)
 for expected in range(2,11):
  # ROS delivery and controller transitions are asynchronous. Keep feeding a
  # gate until its acknowledgement arrives instead of relying on fixed sleeps.
  deadline=time.monotonic()+4
  while max(p.gates or [0])<expected and time.monotonic()<deadline:
   p.pair();time.sleep(.08);p.pair(55.,55.);time.sleep(.08)
   for _ in range(3):p.empty();time.sleep(.08)
  assert max(p.gates or [0])==expected
  if expected < 10:
   if expected in (3,7):
    for _ in range(8):p.heading.publish(Float64(data=350.));p.empty();time.sleep(.08)
    assert wait(lambda:p.cmd and p.cmd[-1].angular.z==pytest.approx(.4 if p.layout=='A' else -.4))
    for _ in range(8):p.heading.publish(Float64(data=260. if p.layout=='A' else 80.));p.empty();time.sleep(.08)
   p.empty()
   assert wait(lambda:p.cmd and p.cmd[-1].linear.x==pytest.approx(.15) and p.cmd[-1].angular.z==pytest.approx(0.))
 assert wait(lambda:p.states and p.states[-1].state==MissionStatus.UNDERWATER_IMAGING,3)
 assert p.call("/asv/mark_underwater_image").success;assert not p.call("/asv/mark_underwater_image").success;assert p.call("/asv/mark_surface_image").success;assert p.call("/asv/complete_docking").success
 assert wait(lambda:p.states[-1].state==MissionStatus.COMPLETED,2);assert p.states[-1].buoy_pairs_passed==10
 for linear,angular in [(0,0),(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
  m=Twist();m.linear.x=float(linear);m.angular.z=float(angular);p.vel.publish(m);time.sleep(.08);o=p.rc[-1].channels;assert o[0]==1500-300*angular and o[2]==1500+300*linear
 bad=Twist();bad.linear.x=float("nan");p.vel.publish(bad);assert wait(lambda:p.rc[-1].channels[0]==1500 and p.rc[-1].channels[2]==1500,1)
 m=Twist();m.linear.x=1.;p.vel.publish(m);assert wait(lambda:p.rc[-1].channels[2]==1800);assert wait(lambda:p.rc[-1].channels[2]==1500,1)
