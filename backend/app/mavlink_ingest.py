import os
import time
import logging
import requests
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO, format="[MAVLink-Ingest] %(levelname)s: %(message)s")

MAVLINK_ENDPOINT = os.getenv("MAVLINK_ENDPOINT", "udp:127.0.0.1:14550")
BACKEND_URL = os.getenv("BACKEND_INGEST_URL", "http://127.0.0.1:8000/api/v1/ingest/telemetry")
API_KEY = os.getenv("API_KEY", "changeme-dev-key")
PUBLISH_HZ = float(os.getenv("TELEMETRY_HZ", "5.0"))

MPS_TO_KNOTS = 1.943844
MPS_TO_KMH = 3.6

class MavlinkTelemetryBridge:
    def __init__(self, endpoint: str, backend_url: str, api_key: str) -> None:
        self.endpoint = endpoint
        self.backend_url = backend_url
        self.headers = {"X-API-Key": api_key, "Content-Type": "application/json"}
        self.state = {
            "lane": "A",
            "position": {"lat": -6.89148, "lon": 107.61065},
            "heading": 0.0,
            "sog_kmh": 0.0,
            "sog_knots": 0.0,
            "cog": 0.0,
            "battery": {"voltage": 16.8, "percentage": 100},
            "mission_state": "standby",
        }

    def build_payload(self) -> dict:
        return {
            "timestamp": int(time.time() * 1000),
            **self.state,
        }

    def run(self) -> None:
        logging.info("Connecting to MAVLink stream at %s...", self.endpoint)
        conn = mavutil.mavlink_connection(self.endpoint)
        logging.info("Waiting for MAVLink heartbeat...")
        conn.wait_heartbeat()
        logging.info("Heartbeat received from system %u component %u", conn.target_system, conn.target_component)

        period = 1.0 / PUBLISH_HZ
        last_post = 0.0

        while True:
            msg = conn.recv_match(
                type=["GLOBAL_POSITION_INT", "VFR_HUD", "SYS_STATUS"],
                blocking=True,
                timeout=0.5,
            )
            if msg is not None:
                mtype = msg.get_type()
                if mtype == "GLOBAL_POSITION_INT":
                    self.state["position"]["lat"] = msg.lat / 1e7
                    self.state["position"]["lon"] = msg.lon / 1e7
                    if msg.hdg != 65535:
                        self.state["heading"] = round((msg.hdg / 100.0) % 360.0, 2)
                    vx_mps = msg.vx / 100.0
                    vy_mps = msg.vy / 100.0
                    speed_mps = (vx_mps ** 2 + vy_mps ** 2) ** 0.5
                    self.state["sog_knots"] = round(speed_mps * MPS_TO_KNOTS, 3)
                    self.state["sog_kmh"] = round(speed_mps * MPS_TO_KMH, 3)

                elif mtype == "VFR_HUD":
                    self.state["heading"] = float(msg.heading % 360)
                    self.state["cog"] = float(msg.heading % 360)
                    self.state["sog_knots"] = round(msg.groundspeed * MPS_TO_KNOTS, 3)
                    self.state["sog_kmh"] = round(msg.groundspeed * MPS_TO_KMH, 3)

                elif mtype == "SYS_STATUS":
                    if msg.voltage_battery != 65535:
                        self.state["battery"]["voltage"] = round(msg.voltage_battery / 1000.0, 2)
                    if msg.battery_remaining != -1:
                        self.state["battery"]["percentage"] = int(max(0, min(100, msg.battery_remaining)))

            now = time.monotonic()
            if now - last_post >= period:
                payload = self.build_payload()
                try:
                    resp = requests.post(self.backend_url, json=payload, headers=self.headers, timeout=1.5)
                    if resp.status_code != 200:
                        logging.warning("Ingest returned %d: %s", resp.status_code, resp.text)
                except requests.RequestException as exc:
                    logging.warning("Backend unreachable (will retry): %s", exc)
                last_post = now

if __name__ == "__main__":
    bridge = MavlinkTelemetryBridge(MAVLINK_ENDPOINT, BACKEND_URL, API_KEY)
    bridge.run()
