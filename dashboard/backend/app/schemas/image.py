# app/schemas/image.py
from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime
from typing import Optional, Dict, Any
from enum import Enum


class ImageType(str, Enum):
    SURFACE = "SURFACE"
    UNDERWATER = "UNDERWATER"


class ImageResponse(BaseModel):
    """Schema untuk response image metadata."""
    id: int
    run_id: Optional[int] = None
    image_type: ImageType
    file_path: str
    file_size: Optional[int] = None
    grid_position: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        validation_alias="image_metadata"
    )
    captured_at: datetime
    created_at: datetime

    model_config = ConfigDict(from_attributes=True, populate_by_name=True)
