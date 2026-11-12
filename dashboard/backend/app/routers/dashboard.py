# app/routers/dashboard.py
from fastapi import APIRouter, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, desc, func
from typing import Optional
from pydantic import BaseModel
from datetime import datetime
from app.core.database import get_db
from app.models.run import MissionRun
from app.models.event import MissionEvent
from app.models.image import MissionImage

router = APIRouter(prefix="/api/v1/dashboard", tags=["Dashboard"])


class DashboardSummary(BaseModel):
    """Ringkasan status misi untuk dashboard."""
    # Run aktif
    active_run_id: Optional[int] = None
    active_run_name: Optional[str] = None
    active_run_status: Optional[str] = None

    # Statistik keseluruhan
    total_runs: int = 0
    total_events: int = 0
    total_images: int = 0

    # Event terakhir
    last_event_type: Optional[str] = None
    last_event_time: Optional[datetime] = None
    last_event_grid: Optional[str] = None

    # Statistik run aktif
    active_run_events: int = 0
    active_run_images: int = 0


@router.get(
    "/summary",
    response_model=DashboardSummary
)
async def get_dashboard_summary(
    db: AsyncSession = Depends(get_db)
):
    """
    Satu endpoint ringkasan untuk dashboard.
    Mengembalikan: run aktif, jumlah events, jumlah images, event terakhir.
    Terbuka tanpa API Key — untuk dashboard.
    """
    summary = DashboardSummary()

    # Total runs
    result = await db.execute(select(func.count(MissionRun.id)))
    summary.total_runs = result.scalar() or 0

    # Total events
    result = await db.execute(select(func.count(MissionEvent.id)))
    summary.total_events = result.scalar() or 0

    # Total images
    result = await db.execute(select(func.count(MissionImage.id)))
    summary.total_images = result.scalar() or 0

    # Cari run yang sedang aktif (RUNNING atau PENDING terakhir)
    result = await db.execute(
        select(MissionRun)
        .where(MissionRun.status.in_(["RUNNING", "PENDING"]))
        .order_by(desc(MissionRun.created_at))
        .limit(1)
    )
    active_run = result.scalar_one_or_none()

    if active_run:
        summary.active_run_id = active_run.id
        summary.active_run_name = active_run.run_name
        summary.active_run_status = active_run.status

        # Hitung events & images di run aktif
        result = await db.execute(
            select(func.count(MissionEvent.id))
            .where(MissionEvent.run_id == active_run.id)
        )
        summary.active_run_events = result.scalar() or 0

        result = await db.execute(
            select(func.count(MissionImage.id))
            .where(MissionImage.run_id == active_run.id)
        )
        summary.active_run_images = result.scalar() or 0

    # Event terakhir
    result = await db.execute(
        select(MissionEvent)
        .order_by(desc(MissionEvent.timestamp))
        .limit(1)
    )
    last_event = result.scalar_one_or_none()

    if last_event:
        summary.last_event_type = last_event.event_type
        summary.last_event_time = last_event.timestamp
        summary.last_event_grid = last_event.grid_position

    return summary
