import os
from sqlalchemy import Column, Integer, String, JSON, create_engine
from sqlalchemy.orm import declarative_base, sessionmaker

DATABASE_URL = os.getenv("DATABASE_URL", "sqlite:///./kki_telemetry.db")

connect_args = {"check_same_thread": False} if DATABASE_URL.startswith("sqlite") else {}
engine = create_engine(DATABASE_URL, connect_args=connect_args)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

class TelemetryRecord(Base):
    __tablename__ = "telemetry_logs"
    id = Column(Integer, primary_key=True, index=True)
    timestamp = Column(String, index=True, nullable=False)
    lane = Column(String, nullable=True)
    mission_state = Column(String, nullable=True)
    data = Column(JSON, nullable=False)

class EventRecord(Base):
    __tablename__ = "mission_events"
    id = Column(Integer, primary_key=True, index=True)
    timestamp = Column(String, index=True, nullable=False)
    event_type = Column(String, index=True, nullable=True)
    severity = Column(String, nullable=True)
    data = Column(JSON, nullable=False)

class ImageMetaRecord(Base):
    __tablename__ = "image_metadata"
    id = Column(Integer, primary_key=True, index=True)
    timestamp = Column(String, index=True, nullable=False)
    box_type = Column(String, index=True, nullable=True)
    data = Column(JSON, nullable=False)

def init_db() -> None:
    Base.metadata.create_all(bind=engine)

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
