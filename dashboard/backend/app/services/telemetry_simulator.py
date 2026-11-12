import asyncio
import random
import time
from app.services.influxdb_service import influx_service
from app.core.websocket import manager
from datetime import datetime, timezone

class TelemetrySimulator:
    def __init__(self):
        self.is_running = False
        self.task = None

    async def start(self):
        if self.is_running:
            return
        self.is_running = True
        self.task = asyncio.create_task(self._simulate_loop())
        print("[Simulator] Telemetry simulator started.")

    async def stop(self):
        if not self.is_running:
            return
        self.is_running = False
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
        print("[Simulator] Telemetry simulator stopped.")

    async def _simulate_loop(self):
        """Looping background task untuk generate data telemetri 10Hz (0.1 detik)."""
        # Posisi awal (misal: Waduk PDAM Bengkalis)
        lat = 1.4883
        lon = 102.1465
        heading = 90.0
        battery_rem = 100
        
        loop_counter = 0

        try:
            while self.is_running:
                # Modifikasi data sedikit demi sedikit seolah-olah kapal bergerak
                lat += random.uniform(-0.00001, 0.00001)
                lon += random.uniform(-0.00001, 0.00001)
                heading = (heading + random.uniform(-2.0, 2.0)) % 360
                battery_rem = max(0, battery_rem - 0.01) # Batere perlahan habis

                data = {
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": 10.0,
                    "speed": random.uniform(1.0, 2.5),
                    "heading": heading,
                    "battery_voltage": 14.8 + random.uniform(-0.2, 0.2),
                    "battery_remaining": int(battery_rem),
                    "roll": random.uniform(-5.0, 5.0),
                    "pitch": random.uniform(-2.0, 2.0),
                    "yaw": heading
                }

                # 1. Tulis ke InfluxDB (tetap 10Hz)
                await influx_service.write_telemetry(data)

                # 2. Broadcast ke WebSocket (Throttled ke 5Hz)
                # Loop jalan tiap 0.1 detik (10Hz). Jika kita broadcast setiap kelipatan 2, 
                # jadinya 5 kali per detik (5Hz).
                if loop_counter % 2 == 0:
                    # Tambahkan timestamp simulasi untuk UI
                    ws_data = data.copy()
                    ws_data["timestamp"] = datetime.now(timezone.utc).isoformat()
                    await manager.broadcast("telemetry_update", ws_data)

                loop_counter += 1

                # Simulasi 10Hz (10 data point per detik)
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass

simulator = TelemetrySimulator()
