# app/routers/runs.py
from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, desc, func
from typing import Optional
from app.core.database import get_db
from app.models.run import MissionRun
from app.models.event import MissionEvent
from app.models.image import MissionImage
from app.schemas.run import RunCreate, RunUpdate, RunResponse, RunStatus
from app.core.security import verify_jetson_api_key

router = APIRouter(prefix="/api/v1/runs", tags=["Mission Runs"])


@router.post(
    "/",
    response_model=RunResponse,
    status_code=status.HTTP_201_CREATED,
    dependencies=[Depends(verify_jetson_api_key)]  # Proteksi: hanya operator/Jetson
)
async def create_run(
    run: RunCreate,
    db: AsyncSession = Depends(get_db)
):
    """
    Buat run baru. Dipanggil saat kapal akan memulai sesi misi baru.
    """
    db_run = MissionRun(
        run_name=run.run_name,
        status=run.status.value,
        started_at=run.started_at,
        notes=run.notes
    )

    db.add(db_run)
    await db.commit()
    await db.refresh(db_run)

    return _run_to_response(db_run, 0, 0)


@router.get(
    "/",
    response_model=list[RunResponse]
)
async def list_runs(
    status_filter: Optional[RunStatus] = Query(
        None, alias="status", description="Filter by run status"
    ),
    limit: int = Query(20, ge=1, le=100),
    offset: int = Query(0, ge=0),
    db: AsyncSession = Depends(get_db)
):
    """
    Ambil daftar semua runs. Bisa difilter berdasarkan status.
    Terbuka tanpa API Key — untuk dashboard.
    """
    query = select(MissionRun).order_by(desc(MissionRun.created_at))

    if status_filter is not None:
        query = query.where(MissionRun.status == status_filter.value)

    query = query.limit(limit).offset(offset)

    result = await db.execute(query)
    runs = result.scalars().all()

    # Hitung event_count dan image_count untuk setiap run
    response_list = []
    for run in runs:
        event_count = await _count_events(db, run.id)
        image_count = await _count_images(db, run.id)
        response_list.append(_run_to_response(run, event_count, image_count))

    return response_list


@router.get(
    "/{run_id}",
    response_model=RunResponse
)
async def get_run(
    run_id: int,
    db: AsyncSession = Depends(get_db)
):
    """
    Ambil detail satu run berdasarkan ID, termasuk jumlah events dan images.
    Terbuka tanpa API Key — untuk dashboard.
    """
    run = await _get_run_or_404(db, run_id)
    event_count = await _count_events(db, run_id)
    image_count = await _count_images(db, run_id)

    return _run_to_response(run, event_count, image_count)


@router.patch(
    "/{run_id}",
    response_model=RunResponse,
    dependencies=[Depends(verify_jetson_api_key)]  # Proteksi: hanya operator/Jetson
)
async def update_run(
    run_id: int,
    run_update: RunUpdate,
    db: AsyncSession = Depends(get_db)
):
    """
    Update run (status, notes, score, dll). 
    Contoh: ubah status dari RUNNING ke FINISHED saat misi selesai.
    """
    run = await _get_run_or_404(db, run_id)

    # Update hanya field yang dikirim (bukan None)
    update_data = run_update.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        if field == "status" and value is not None:
            value = value.value  # Convert enum ke string
        setattr(run, field, value)

    await db.commit()
    await db.refresh(run)

    event_count = await _count_events(db, run_id)
    image_count = await _count_images(db, run_id)

    return _run_to_response(run, event_count, image_count)


# ─── Helper Functions ─────────────────────────────────────────────

async def _get_run_or_404(db: AsyncSession, run_id: int) -> MissionRun:
    """Helper: ambil run atau raise 404."""
    result = await db.execute(
        select(MissionRun).where(MissionRun.id == run_id)
    )
    run = result.scalar_one_or_none()

    if run is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Run dengan ID {run_id} tidak ditemukan."
        )
    return run


async def _count_events(db: AsyncSession, run_id: int) -> int:
    """Helper: hitung jumlah events di run tertentu."""
    result = await db.execute(
        select(func.count(MissionEvent.id)).where(MissionEvent.run_id == run_id)
    )
    return result.scalar() or 0


async def _count_images(db: AsyncSession, run_id: int) -> int:
    """Helper: hitung jumlah images di run tertentu."""
    result = await db.execute(
        select(func.count(MissionImage.id)).where(MissionImage.run_id == run_id)
    )
    return result.scalar() or 0


def _run_to_response(run: MissionRun, event_count: int, image_count: int) -> RunResponse:
    """Helper: konversi ORM object ke RunResponse dengan counts."""
    return RunResponse(
        id=run.id,
        run_name=run.run_name,
        status=run.status,
        started_at=run.started_at,
        finished_at=run.finished_at,
        total_score=run.total_score,
        notes=run.notes,
        created_at=run.created_at,
        event_count=event_count,
        image_count=image_count
    )
