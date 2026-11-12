from fastapi import APIRouter, status
from app.services.influxdb_service import influx_service
from app.services.telemetry_simulator import simulator
from typing import Dict, Any

router = APIRouter(prefix="/api/v1/telemetry", tags=["Telemetry"])

@router.get("/latest", response_model=Dict[str, Any])
async def get_latest_telemetry():
    """Ambil data telemetri terakhir dari InfluxDB (untuk dashboard)."""
    data = await influx_service.query_latest_telemetry()
    return data

@router.post("/simulator/start", status_code=status.HTTP_200_OK)
async def start_simulator():
    """Mulai simulator telemetri (generate data 10Hz)."""
    await simulator.start()
    return {"message": "Simulator started"}

@router.post("/simulator/stop", status_code=status.HTTP_200_OK)
async def stop_simulator():
    """Hentikan simulator telemetri."""
    await simulator.stop()
    return {"message": "Simulator stopped"}
