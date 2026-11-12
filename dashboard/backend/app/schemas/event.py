from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime
from typing import Optional, Dict, Any
from enum import Enum

class EventType(str, Enum):
    SYSTEM_INIT = "SYSTEM_INIT"
    START = "START"
    BUOY_PAIR = "BUOY_PAIR"
    PENALTY = "PENALTY"
    SURFACE_IMAGING = "SURFACE_IMAGING"
    UNDERWATER_IMAGING = "UNDERWATER_IMAGING"
    DOCKING = "DOCKING"
    APPROACHING = "APPROACHING"
    FINISH = "FINISH"
    ABORT = "ABORT"

class EventCreate(BaseModel):
    """Schema untuk request body saat Jetson mengirim event baru."""
    run_id: Optional[int] = Field(None, description="ID run yang sedang aktif (opsional)")
    event_type: EventType
    timestamp: datetime = Field(description="Waktu kejadian di Jetson (UTC)")
    grid_position: Optional[str] = Field(
        None, 
        max_length=5, 
        pattern=r"^[A-E][1-5]$", 
        description="Posisi arena grid KKI (Kolom A-E, Baris 1-5). Contoh: 'B3'"
    )
    metadata: Optional[Dict[str, Any]] = Field(
        default_factory=dict, 
        description="Data tambahan. Ex: {'reason': 'touch_buoy'} atau {'image_path': '/img/01.jpg'}"
    )

class EventResponse(BaseModel):
    """Schema untuk response body setelah event berhasil disimpan ke DB."""
    id: int
    run_id: Optional[int] = None
    event_type: EventType
    timestamp: datetime
    grid_position: Optional[str] = None
    # validation_alias="event_metadata" → baca atribut 'event_metadata' dari ORM object
    # Field name 'metadata' → tampilkan sebagai 'metadata' di JSON response
    metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        validation_alias="event_metadata"
    )
    created_at: datetime

    model_config = ConfigDict(from_attributes=True, populate_by_name=True)