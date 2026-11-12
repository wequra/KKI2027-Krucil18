import asyncio
import logging
from pymavlink import mavutil
from datetime import datetime, timezone
from app.core.config import settings
from app.services.influxdb_service import influx_service
from app.core.websocket import manager

logger = logging.getLogger(__name__)

class MAVLinkService:
    def __init__(self):
        self.is_running = False
        self.task = None
        self.connection = None
        
        # Simpan state terakhir untuk digabung jadi satu payload lengkap
        self.latest_telemetry = {
            "latitude": 0.0,
            "longitude": 0.0,
            "altitude": 0.0,
            "speed": 0.0,
            "heading": 0.0,
            "battery_voltage": 0.0,
            "battery_remaining": 0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0
        }

    async def start(self):
        if self.is_running:
            return
        
        try:
            logger.info(f"[MAVLink] Mencoba konek ke {settings.MAVLINK_CONNECTION_STRING} (Baud: {settings.MAVLINK_BAUDRATE})...")
            # Inisialisasi koneksi MAVLink
            self.connection = mavutil.mavlink_connection(
                settings.MAVLINK_CONNECTION_STRING, 
                baud=settings.MAVLINK_BAUDRATE,
                autoreconnect=True
            )
            
            # Tunggu heartbeat pertama (non-blocking lewat asyncio)
            logger.info("[MAVLink] Menunggu heartbeat dari Pixhawk...")
            
            self.is_running = True
            self.task = asyncio.create_task(self._listen_loop())
            logger.info("[MAVLink] MAVLink service berhasil dijalankan.")
        except Exception as e:
            logger.error(f"[MAVLink] Gagal menyalakan service: {e}")

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
        
        if self.connection:
            self.connection.close()
        logger.info("[MAVLink] MAVLink service dimatikan.")

    async def _listen_loop(self):
        loop_counter = 0
        try:
            while self.is_running:
                logger.info("listen loop alive")
                # Cek pesan non-blocking
                msg = self.connection.recv_match(blocking=False)

                if msg is None:
                        logger.info("NO MAVLINK")
                else:
                    logger.info(f"RECV: {msg.get_type()}")
                if msg is not None:
                    msg_type = msg.get_type()
                    updated = False
                    
                    if msg_type == 'GLOBAL_POSITION_INT':
                        # Lintang/Bujur MAVLink dikali 1e7
                        self.latest_telemetry["latitude"] = msg.lat / 1e7
                        self.latest_telemetry["longitude"] = msg.lon / 1e7
                        self.latest_telemetry["altitude"] = msg.alt / 1000.0  # mm to m
                        self.latest_telemetry["heading"] = msg.hdg / 100.0 if msg.hdg != 65535 else 0.0 # cdeg to deg
                        # Kecepatan gabungan VX, VY (m/s) dikali 100
                        import math
                        self.latest_telemetry["speed"] = math.sqrt((msg.vx/100.0)**2 + (msg.vy/100.0)**2)
                        updated = True

                    elif msg_type == 'ATTITUDE':
                        import math
                        # Roll, Pitch, Yaw dalam radian, kita ubah ke derajat
                        self.latest_telemetry["roll"] = math.degrees(msg.roll)
                        self.latest_telemetry["pitch"] = math.degrees(msg.pitch)
                        self.latest_telemetry["yaw"] = math.degrees(msg.yaw)
                        updated = True

                    elif msg_type == 'SYS_STATUS':
                        self.latest_telemetry["battery_voltage"] = msg.voltage_battery / 1000.0  # mV to V
                        self.latest_telemetry["battery_remaining"] = msg.battery_remaining # persentase
                        updated = True
                    
                    # Jika ada update paket krusial, tulis ke InfluxDB
                    # Pixhawk biasanya ngirim 10Hz, jadi ini akan cukup sering.
                    if updated:
                        data_copy = self.latest_telemetry.copy()
                        await influx_service.write_telemetry(data_copy)

                        # Throttle Broadcast WebSocket (mirip seperti di simulator)
                        # Kita broadcast tiap ~5 kali iterasi yang valid
                        if loop_counter % 5 == 0:
                            ws_data = data_copy.copy()
                            ws_data["timestamp"] = datetime.now(timezone.utc).isoformat()
                            await manager.broadcast("telemetry_update", ws_data)
                            
                            # Enqueue ke Sync Agent agar dikirim ke Cloud
                            from app.services.sync_agent import enqueue_data
                            enqueue_data(topic="telemetry", payload=ws_data)
                        
                        loop_counter += 1

                # Beri nafas ke asyncio event loop (sangat penting agar server FastAPI tidak freeze)
                await asyncio.sleep(0.01)
                
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"[MAVLink] Error di listen loop: {e}")

mavlink_service = MAVLinkService()
