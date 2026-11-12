# app/models/image.py
from sqlalchemy import Column, Integer, String, ForeignKey
from sqlalchemy.dialects.postgresql import JSONB, TIMESTAMP
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from app.core.database import Base


class MissionImage(Base):
    """
    Menyimpan metadata foto yang diambil oleh Jetson.
    File foto disimpan di filesystem (uploads/images/),
    sedangkan metadata-nya disimpan di tabel ini.
    """
    __tablename__ = "mission_images"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    run_id = Column(Integer, ForeignKey("mission_runs.id"), nullable=True, index=True)
    image_type = Column(
        String(20), nullable=False, index=True
    )  # SURFACE atau UNDERWATER
    file_path = Column(String(500), nullable=False)  # Path ke file di filesystem
    file_size = Column(Integer, nullable=True)  # Ukuran file dalam bytes
    grid_position = Column(String(5), nullable=True)  # Posisi arena (A1-E5)

    # JSONB untuk info tambahan (geo_tag, confidence, color_band, dll)
    image_metadata = Column("metadata", JSONB, nullable=True, default=dict)

    captured_at = Column(TIMESTAMP(timezone=True), nullable=False)  # Waktu foto diambil
    created_at = Column(TIMESTAMP(timezone=True), server_default=func.now())

    # Relationship: image milik satu run
    run = relationship("MissionRun", back_populates="images")
