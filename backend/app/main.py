import os
from contextlib import asynccontextmanager
from typing import Any, Dict, List
from fastapi import Body, FastAPI, Depends, HTTPException, Security, WebSocket, WebSocketDisconnect, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security.api_key import APIKeyHeader
from sqlalchemy.orm import Session

from .db import init_db, get_db, TelemetryRecord, EventRecord, ImageMetaRecord
from .schemas import validate_contract

API_KEY = os.getenv("API_KEY", "changeme-dev-key")
api_key_header = APIKeyHeader(name="X-API-Key", auto_error=False)

async def verify_api_key(api_key: str = Security(api_key_header)) -> str:
    if api_key != API_KEY:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or missing X-API-Key header",
        )
    return api_key

class ConnectionManager:
    def __init__(self) -> None:
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, envelope: dict) -> None:
        stale: List[WebSocket] = []
        for connection in self.active_connections:
            try:
                await connection.send_json(envelope)
            except Exception:
                stale.append(connection)
        for conn in stale:
            self.disconnect(conn)

manager = ConnectionManager()

@asynccontextmanager
async def lifespan(app: FastAPI):
    init_db()
    yield

app = FastAPI(
    title="KKI 2027 Krucil 18 - ASV Telemetry & Mission Backend",
    version="0.2.0-M2",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/health")
def health_check():
    return {"status": "ok", "milestone": "M2", "ws_clients": len(manager.active_connections)}

@app.post("/api/v1/ingest/telemetry", dependencies=[Depends(verify_api_key)])
async def ingest_telemetry(payload: Dict[str, Any] = Body(...), db: Session = Depends(get_db)):
    validated = validate_contract("telemetry", payload)
    record = TelemetryRecord(
        timestamp=str(validated.get("timestamp", "")),
        lane=str(validated.get("lane", "")),
        mission_state=str(validated.get("mission_state", "")),
        data=validated,
    )
    db.add(record)
    db.commit()
    await manager.broadcast({"channel": "telemetry", "data": validated})
    return {"status": "ingested", "channel": "telemetry"}

@app.post("/api/v1/ingest/event", dependencies=[Depends(verify_api_key)])
async def ingest_event(payload: Dict[str, Any] = Body(...), db: Session = Depends(get_db)):
    validated = validate_contract("event", payload)
    record = EventRecord(
        timestamp=str(validated.get("timestamp", "")),
        event_type=str(validated.get("type", validated.get("event_type", ""))),
        severity=str(validated.get("severity", "")),
        data=validated,
    )
    db.add(record)
    db.commit()
    await manager.broadcast({"channel": "event", "data": validated})
    return {"status": "ingested", "channel": "event"}

@app.post("/api/v1/ingest/image-meta", dependencies=[Depends(verify_api_key)])
async def ingest_image_meta(payload: Dict[str, Any] = Body(...), db: Session = Depends(get_db)):
    validated = validate_contract("image-meta", payload)
    record = ImageMetaRecord(
        timestamp=str(validated.get("timestamp", "")),
        box_type=str(validated.get("box_type", "")),
        data=validated,
    )
    db.add(record)
    db.commit()
    await manager.broadcast({"channel": "image_meta", "data": validated})
    return {"status": "ingested", "channel": "image_meta"}

@app.websocket("/ws/stream")
async def websocket_stream(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)
