from influxdb_client.client.influxdb_client_async import InfluxDBClientAsync
from influxdb_client import Point
from app.core.config import settings
from datetime import datetime
from typing import List, Dict, Any

class InfluxDBService:
    def __init__(self):
        self.url = settings.INFLUXDB_URL
        self.token = settings.INFLUXDB_TOKEN
        self.org = settings.INFLUXDB_ORG
        self.bucket = settings.INFLUXDB_BUCKET
        self.client = None
        self.write_api = None
        self.query_api = None

    async def connect(self):
        """Inisialisasi koneksi client InfluxDB Async."""
        if not self.client:
            self.client = InfluxDBClientAsync(
                url=self.url,
                token=self.token,
                org=self.org
            )
            self.write_api = self.client.write_api()
            self.query_api = self.client.query_api()

    async def close(self):
        """Tutup koneksi InfluxDB."""
        if self.client:
            await self.client.close()
            self.client = None

    async def write_telemetry(self, data: Dict[str, Any]):
        """Tulis satu data point telemetri ke InfluxDB."""
        if not self.write_api:
            await self.connect()

        # Buat data Point
        point = Point("asv_telemetry") \
            .tag("vessel", "Qing") \
            .field("latitude", float(data.get("latitude", 0.0))) \
            .field("longitude", float(data.get("longitude", 0.0))) \
            .field("altitude", float(data.get("altitude", 0.0))) \
            .field("speed", float(data.get("speed", 0.0))) \
            .field("heading", float(data.get("heading", 0.0))) \
            .field("battery_voltage", float(data.get("battery_voltage", 0.0))) \
            .field("battery_remaining", int(data.get("battery_remaining", 0))) \
            .field("roll", float(data.get("roll", 0.0))) \
            .field("pitch", float(data.get("pitch", 0.0))) \
            .field("yaw", float(data.get("yaw", 0.0)))

        # Tulis secara asynchronous
        try:
            await self.write_api.write(bucket=self.bucket, record=point)
        except Exception as e:
            print(f"[InfluxDB] Gagal menulis data: {e}")

    async def query_latest_telemetry(self) -> Dict[str, Any]:
        """Ambil data telemetri terakhir (paling baru)."""
        if not self.query_api:
            await self.connect()

        # Query Flux untuk ambil data terakhir dalam 5 menit terakhir
        query = f'''
        from(bucket: "{self.bucket}")
          |> range(start: -5m)
          |> filter(fn: (r) => r["_measurement"] == "asv_telemetry")
          |> last()
        '''
        
        try:
            tables = await self.query_api.query(query)
            result = {}
            for table in tables:
                for record in table.records:
                    result[record.get_field()] = record.get_value()
                    # Ambil timestamp dari field manapun yang pertama
                    if "timestamp" not in result:
                        result["timestamp"] = record.get_time().isoformat()
            return result
        except Exception as e:
            print(f"[InfluxDB] Gagal query data: {e}")
            return {}

# Singleton instance
influx_service = InfluxDBService()
