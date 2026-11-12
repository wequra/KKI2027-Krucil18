import asyncio
import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routers import events, runs, images, dashboard, telemetry, ws
from app.core.database import engine, Base
from app.services.influxdb_service import influx_service
from app.services.telemetry_simulator import simulator
from app.services.mavlink_service import mavlink_service
from app.services.sync_agent import sync_worker
from app.core.config import settings

# Konfigurasi logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Import semua models agar SQLAlchemy tahu tabel apa saja yang harus dibuat
from app.models import event, run, image  # noqa: F401

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Setup awal ketika aplikasi berjalan
    logger.info("Memulai aplikasi Dagozilla ASV Backend...")
    
    # Startup: Buat semua tabel yang didefinisikan di Base.metadata
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    
    # Inisialisasi InfluxDB
    await influx_service.connect()
    
    # Mulai service Telemetri otomatis
    if settings.USE_SIMULATOR:
        await simulator.start()
    else:
        await mavlink_service.start()
    
    # Jalankan background worker untuk Cloud Sync Agent
    sync_task = asyncio.create_task(sync_worker())
    
    yield
    
    # Shutdown: Cleanup
    if settings.USE_SIMULATOR:
        await simulator.stop()
    else:
        await mavlink_service.stop()
        
    sync_task.cancel()
    await influx_service.close()
    logger.info("Mematikan aplikasi Dagozilla ASV Backend...")

app = FastAPI(
    title="Dagozilla ASV Backend",
    description="Local-first backend for KKI 2026 ASV Mission Monitoring",
    version="1.0.0",
    lifespan=lifespan
)

# The dashboard is normally served by Vite on port 5173 while the API is on
# port 8000. Image metadata and files are read by the existing panels.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["*"],
)

# Include routers
app.include_router(events.router)
app.include_router(runs.router)
app.include_router(images.router)
app.include_router(dashboard.router)
app.include_router(telemetry.router)
app.include_router(ws.router)

@app.get("/health")
async def health_check():
    return {"status": "OK", "vessel": "Qing"}
