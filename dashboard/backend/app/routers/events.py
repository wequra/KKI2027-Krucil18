from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, desc
from typing import Optional
from app.core.database import get_db
from app.models.event import MissionEvent
from app.schemas.event import EventCreate, EventResponse, EventType
from app.core.security import verify_jetson_api_key
from app.core.websocket import manager
from app.services.scoring_engine import calculate_event_score, update_run_score
from app.services.sync_agent import enqueue_data

router = APIRouter(prefix="/api/v1/events", tags=["Mission Events"])

@router.post(
    "/", 
    response_model=EventResponse, 
    status_code=status.HTTP_201_CREATED,
    dependencies=[Depends(verify_jetson_api_key)]  # Proteksi: hanya Jetson yang boleh POST
)
async def ingest_mission_event(
    event: EventCreate, 
    db: AsyncSession = Depends(get_db)
):
    """
    Endpoint untuk menerima mission events dari Jetson Orin Nano.
    Dilindungi API Key — hanya Jetson yang bisa kirim data.
    """
    db_event = MissionEvent(
        run_id=event.run_id,
        event_type=event.event_type.value,
        timestamp=event.timestamp,
        grid_position=event.grid_position,
        event_metadata=event.metadata
    )
    
    db.add(db_event)
    await db.commit()
    await db.refresh(db_event)
    
    # Hitung Skoring
    if db_event.run_id:
        score_delta = calculate_event_score(db_event.event_type)
        await update_run_score(db, db_event.run_id, score_delta)
    
    # Konversi SQLAlchemy model ke format dict untuk JSON broadcast
    event_dict = {
        "id": db_event.id,
        "run_id": db_event.run_id,
        "event_type": db_event.event_type,
        "timestamp": db_event.timestamp.isoformat(),
        "grid_position": db_event.grid_position,
        "metadata": db_event.event_metadata
    }
    
    # Broadcast event baru ke semua client WebSocket
    await manager.broadcast("new_event", event_dict)
    
    # Enqueue ke Sync Agent agar dikirim ke Cloud
    enqueue_data(topic="events", payload=event_dict)
    
    return db_event


@router.get(
    "/", 
    response_model=list[EventResponse]
)
async def list_events(
    run_id: Optional[int] = Query(None, description="Filter by run ID"),
    event_type: Optional[EventType] = Query(None, description="Filter by event type"),
    limit: int = Query(50, ge=1, le=200, description="Jumlah event per halaman"),
    offset: int = Query(0, ge=0, description="Offset untuk pagination"),
    db: AsyncSession = Depends(get_db)
):
    """
    Ambil daftar mission events. Bisa difilter berdasarkan run_id dan event_type.
    Terbuka tanpa API Key — untuk dashboard.
    """
    query = select(MissionEvent).order_by(desc(MissionEvent.timestamp))

    if run_id is not None:
        query = query.where(MissionEvent.run_id == run_id)
    if event_type is not None:
        query = query.where(MissionEvent.event_type == event_type.value)

    query = query.limit(limit).offset(offset)
    
    result = await db.execute(query)
    events = result.scalars().all()
    
    return events


@router.get(
    "/{event_id}", 
    response_model=EventResponse
)
async def get_event(
    event_id: int, 
    db: AsyncSession = Depends(get_db)
):
    """
    Ambil detail satu event berdasarkan ID.
    Terbuka tanpa API Key — untuk dashboard.
    """
    result = await db.execute(
        select(MissionEvent).where(MissionEvent.id == event_id)
    )
    event = result.scalar_one_or_none()
    
    if event is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Event dengan ID {event_id} tidak ditemukan."
        )
    
    return event