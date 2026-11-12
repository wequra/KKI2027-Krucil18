# app/models/event.py
from sqlalchemy import Column, Integer, String, ForeignKey
from sqlalchemy.dialects.postgresql import JSONB, TIMESTAMP
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from app.core.database import Base

class MissionEvent(Base):
    __tablename__ = "mission_events"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    run_id = Column(Integer, ForeignKey("mission_runs.id"), nullable=True, index=True)
    event_type = Column(String(50), nullable=False, index=True)
    timestamp = Column(TIMESTAMP(timezone=True), nullable=False, index=True)
    grid_position = Column(String(5), nullable=True)
    
    # JSONB sangat optimal untuk PostgreSQL dalam menyimpan metadata fleksibel
    event_metadata = Column("metadata", JSONB, nullable=True, default=dict) 
    
    created_at = Column(TIMESTAMP(timezone=True), server_default=func.now())

    # Relationship: event milik satu run
    run = relationship("MissionRun", back_populates="events")