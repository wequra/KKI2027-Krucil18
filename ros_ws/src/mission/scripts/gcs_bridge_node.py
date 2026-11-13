#!/usr/bin/env python3
"""Read-only, failure-isolated HTTP interface for laptop GCS clients."""

import glob
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import threading
import time
from urllib.parse import urlparse

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from msgs.msg import MissionStatus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, UInt8

STATE_NAMES = {
    v: k for k, v in vars(MissionStatus).items() if k.isupper() and isinstance(v, int)
}


class Handler(BaseHTTPRequestHandler):
    """HTTP request handler providing read-only REST endpoints for GCS."""

    def send(self, status: int, content_type: str, data: bytes):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        node = self.server.node
        path = urlparse(self.path).path

        if path == '/health':
            self.send(HTTPStatus.OK, 'application/json', json.dumps(node.health()).encode())
            return

        if path in ('/api/telemetry', '/api/mission'):
            is_mission_only = path.endswith('mission')
            payload_data = node.payload(mission_only=is_mission_only)
            self.send(HTTPStatus.OK, 'application/json', json.dumps(payload_data).encode())
            return

        for kind in ('underwater', 'surface'):
            if path in (f'/api/images/{kind}/latest', f'/api/images/{kind}/latest/metadata'):
                image = node.latest_image(kind)
                if not image:
                    self.send(HTTPStatus.NOT_FOUND, 'application/json', b'{"error":"no capture available"}')
                    return

                if path.endswith('metadata'):
                    meta = node.image_meta(image, kind)
                    self.send(HTTPStatus.OK, 'application/json', json.dumps(meta).encode())
                else:
                    try:
                        with open(image, 'rb') as f:
                            data = f.read()
                        self.send(HTTPStatus.OK, 'image/jpeg', data)
                    except OSError as e:
                        self.send(HTTPStatus.INTERNAL_SERVER_ERROR, 'application/json', json.dumps({'error': str(e)}).encode())
                return

        self.send(HTTPStatus.NOT_FOUND, 'application/json', b'{"error":"not found"}')

    def log_message(self, format, *args):  # noqa: A002
        pass


class Server(ThreadingHTTPServer):
    """Threading HTTP server carrying a reference to the ROS bridge node."""

    daemon_threads = True

    def __init__(self, addr, node):
        super().__init__(addr, Handler)
        self.node = node


class Bridge(Node):
    """Bridge node translating ROS 2 topics to lightweight JSON telemetry."""

    def __init__(self):
        super().__init__('gcs_bridge_node')
        host = str(self.declare_parameter('bind_host', '0.0.0.0').value)
        port = int(self.declare_parameter('port', 8090).value)
        self.stale = float(self.declare_parameter('telemetry_stale_sec', 2.0).value)
        self.telemetry_rate = float(self.declare_parameter('gcs_telemetry_rate_hz', 5.0).value)
        self.root = os.path.expanduser(
            str(self.declare_parameter('image_root', '~/dagozilla_logs/images').value)
        )

        self.data = {
            'latitude': None,
            'longitude': None,
            'speed_mps': None,
            'heading_deg': None,
            'armed': False,
            'mode': '',
            'connected': False,
            'mission_state': 'IDLE',
            'mission_state_id': MissionStatus.IDLE,
            'gate_count': 0,
        }
        self.seen = {}

        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps, 10)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.velocity, 10)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.heading, 10)
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(MissionStatus, '/asv/mission_status', self.mission, 10)
        self.create_subscription(UInt8, '/asv/buoy_gate_count', self.gates, 10)

        self.server = Server((host, port), self)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(f'GCS HTTP API: http://{host}:{port}')

    def mark(self, key: str):
        self.seen[key] = time.monotonic()

    def gps(self, m: NavSatFix):
        self.data.update(latitude=m.latitude, longitude=m.longitude)
        self.mark('gps')

    def velocity(self, m: TwistStamped):
        self.data['speed_mps'] = math.hypot(m.twist.linear.x, m.twist.linear.y)
        self.mark('velocity')

    def heading(self, m: Float64):
        self.data['heading_deg'] = m.data
        self.mark('heading')

    def state_cb(self, m: State):
        self.data.update(armed=m.armed, mode=m.mode, connected=m.connected)
        self.mark('mavros_state')

    def mission(self, m: MissionStatus):
        self.data.update(
            mission_state=STATE_NAMES.get(m.state, 'UNKNOWN'),
            mission_state_id=m.state,
            gate_count=m.buoy_pairs_passed,
        )
        self.mark('mission')

    def gates(self, m: UInt8):
        self.data['gate_count'] = m.data
        self.mark('gate_count')

    def payload(self, mission_only: bool = False):
        now = time.time()
        stale = {
            k: (time.monotonic() - self.seen.get(k, 0) > self.stale)
            for k in ('gps', 'velocity', 'heading', 'mavros_state', 'mission')
        }
        result = dict(self.data)
        result.update(
            timestamp=now,
            stale=stale,
            telemetry_stale=any(stale[k] for k in ('gps', 'velocity', 'heading', 'mavros_state')),
        )
        if mission_only:
            return {
                k: result[k]
                for k in ('mission_state', 'mission_state_id', 'gate_count', 'timestamp', 'stale')
            }
        return result

    def health(self):
        return {'ok': True, 'mission_independent': True, 'timestamp': time.time()}

    def latest_image(self, kind: str):
        files = glob.glob(os.path.join(self.root, kind, f'{kind}_*.jpg'))
        return max(files, key=os.path.getmtime) if files else None

    def image_meta(self, path: str, kind: str):
        meta = {
            'capture_type': kind,
            'filename': os.path.basename(path),
            'timestamp': os.path.getmtime(path),
            'success': True,
            'fallback': False,
            'download_url': f'/api/images/{kind}/latest',
        }
        sidecar_path = path + '.json'
        if os.path.isfile(sidecar_path):
            try:
                with open(sidecar_path, 'r') as f:
                    meta.update(json.load(f))
            except (OSError, ValueError):
                pass
        return meta

    def close(self):
        self.server.shutdown()
        self.server.server_close()


def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
