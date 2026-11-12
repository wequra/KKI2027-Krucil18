# app/schemas/run.py
from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime
from typing import Optional
from enum import Enum


class RunStatus(str, Enum):
    PENDING = "PENDING"
    RUNNING = "RUNNING"
    FINISHED = "FINISHED"
    ABORTED = "ABORTED"


class RunCreate(BaseModel):
    """Schema untuk membuat run baru."""
    run_name: str = Field(
        ..., max_length=100,
        description="Nama run. Contoh: 'Run Latihan 1', 'Run Final'"
    )
    status: RunStatus = Field(
        default=RunStatus.PENDING,
        description="Status awal run"
    )
    started_at: Optional[datetime] = Field(
        None, description="Waktu mulai (opsional, bisa diisi nanti)"
    )
    notes: Optional[str] = Field(
        None, description="Catatan operator"
    )


class RunUpdate(BaseModel):
    """Schema untuk update run (semua field opsional)."""
    run_name: Optional[str] = Field(None, max_length=100)
    status: Optional[RunStatus] = None
    started_at: Optional[datetime] = None
    finished_at: Optional[datetime] = None
    total_score: Optional[float] = None
    notes: Optional[str] = None


class RunResponse(BaseModel):
    """Schema untuk response run."""
    id: int
    run_name: str
    status: RunStatus
    started_at: Optional[datetime] = None
    finished_at: Optional[datetime] = None
    total_score: Optional[float] = None
    notes: Optional[str] = None
    created_at: datetime
    event_count: Optional[int] = Field(
        None, description="Jumlah event di run ini"
    )
    image_count: Optional[int] = Field(
        None, description="Jumlah image di run ini"
    )

    model_config = ConfigDict(from_attributes=True)
