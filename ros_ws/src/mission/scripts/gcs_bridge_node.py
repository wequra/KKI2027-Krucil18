#!/usr/bin/env python3
"""Read-only, failure-isolated HTTP interface for laptop GCS clients."""
import glob, json, math, os, threading, time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from msgs.msg import MissionStatus
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, UInt8

STATE_NAMES={v:k for k,v in vars(MissionStatus).items() if k.isupper() and isinstance(v,int)}
class Handler(BaseHTTPRequestHandler):
 def send(self,status,ctype,data):
  self.send_response(status); self.send_header('Content-Type',ctype); self.send_header('Access-Control-Allow-Origin','*'); self.send_header('Cache-Control','no-store'); self.end_headers(); self.wfile.write(data)
 def do_GET(self):
  node=self.server.node; path=urlparse(self.path).path
  if path=='/health': self.send(200,'application/json',json.dumps(node.health()).encode()); return
  if path in ('/api/telemetry','/api/mission'): self.send(200,'application/json',json.dumps(node.payload(mission_only=path.endswith('mission'))).encode()); return
  for kind in ('underwater','surface'):
   if path in ('/api/images/%s/latest'%kind,'/api/images/%s/latest/metadata'%kind):
    image=node.latest_image(kind)
    if not image: self.send(404,'application/json',b'{"error":"no capture available"}'); return
    if path.endswith('metadata'):
     meta=node.image_meta(image,kind); self.send(200,'application/json',json.dumps(meta).encode())
    else:
     try:
      with open(image,'rb') as f: data=f.read()
      self.send(200,'image/jpeg',data)
     except OSError as e: self.send(500,'application/json',json.dumps({'error':str(e)}).encode())
    return
  self.send(404,'application/json',b'{"error":"not found"}')
 def log_message(self,*_): pass
class Server(ThreadingHTTPServer):
 daemon_threads=True
 def __init__(self,addr,node): super().__init__(addr,Handler); self.node=node
class Bridge(Node):
 def __init__(self):
  super().__init__('gcs_bridge_node'); host=str(self.declare_parameter('bind_host','0.0.0.0').value); port=int(self.declare_parameter('port',8090).value); self.stale=float(self.declare_parameter('telemetry_stale_sec',2.0).value); self.telemetry_rate=float(self.declare_parameter('gcs_telemetry_rate_hz',5.0).value)
  self.root=os.path.expanduser(str(self.declare_parameter('image_root','~/dagozilla_logs/images').value)); self.data={'latitude':None,'longitude':None,'speed_mps':None,'heading_deg':None,'armed':False,'mode':'','connected':False,'mission_state':'IDLE','mission_state_id':MissionStatus.IDLE,'gate_count':0}; self.seen={}
  self.create_subscription(NavSatFix,'/mavros/global_position/global',self.gps,10); self.create_subscription(TwistStamped,'/mavros/local_position/velocity_local',self.velocity,10); self.create_subscription(Float64,'/mavros/global_position/compass_hdg',self.heading,10); self.create_subscription(State,'/mavros/state',self.state_cb,10); self.create_subscription(MissionStatus,'/asv/mission_status',self.mission,10); self.create_subscription(UInt8,'/asv/buoy_gate_count',self.gates,10)
  self.server=Server((host,port),self); self.thread=threading.Thread(target=self.server.serve_forever,daemon=True); self.thread.start(); self.get_logger().info('GCS HTTP API: http://%s:%d' %(host,port))
 def mark(self,key): self.seen[key]=time.monotonic()
 def gps(self,m): self.data.update(latitude=m.latitude,longitude=m.longitude); self.mark('gps')
 def velocity(self,m): self.data['speed_mps']=math.hypot(m.twist.linear.x,m.twist.linear.y); self.mark('velocity')
 def heading(self,m): self.data['heading_deg']=m.data; self.mark('heading')
 def state_cb(self,m): self.data.update(armed=m.armed,mode=m.mode,connected=m.connected); self.mark('mavros_state')
 def mission(self,m): self.data.update(mission_state=STATE_NAMES.get(m.state,'UNKNOWN'),mission_state_id=m.state,gate_count=m.buoy_pairs_passed); self.mark('mission')
 def gates(self,m): self.data['gate_count']=m.data; self.mark('gate_count')
 def payload(self,mission_only=False):
  now=time.time(); stale={k:(time.monotonic()-self.seen.get(k,0)>self.stale) for k in ('gps','velocity','heading','mavros_state','mission')}; result=dict(self.data); result.update(timestamp=now,stale=stale,telemetry_stale=any(stale[k] for k in ('gps','velocity','heading','mavros_state')))
  return {k:result[k] for k in ('mission_state','mission_state_id','gate_count','timestamp','stale')} if mission_only else result
 def health(self): return {'ok':True,'mission_independent':True,'timestamp':time.time()}
 def latest_image(self,kind):
  files=glob.glob(os.path.join(self.root,kind,'%s_*.jpg'%kind)); return max(files,key=os.path.getmtime) if files else None
 def image_meta(self,path,kind):
  meta={'capture_type':kind,'filename':os.path.basename(path),'timestamp':os.path.getmtime(path),'success':True,'fallback':False,'download_url':'/api/images/%s/latest'%kind}
  try:
   with open(path+'.json') as f: meta.update(json.load(f))
  except (OSError,ValueError): pass
  return meta
 def close(self): self.server.shutdown(); self.server.server_close()
def main(args=None):
 rclpy.init(args=args); n=Bridge()
 try:rclpy.spin(n)
 finally:n.close(); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__':main()
