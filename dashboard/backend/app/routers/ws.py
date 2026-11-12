# app/routers/ws.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from app.core.websocket import manager

router = APIRouter(prefix="/ws", tags=["WebSocket"])

@router.websocket("/dashboard")
async def websocket_endpoint(websocket: WebSocket):
    """
    Endpoint WebSocket untuk Front-End Dashboard.
    Client cukup connect ke ws://<host>/ws/dashboard dan mendengarkan JSON message.
    """
    await manager.connect(websocket)
    try:
        while True:
            # Tetap listen barangkali client kirim sesuatu (misal: ping)
            # Untuk sekarang backend hanya mem-broadcast, tidak merespon pesan spesifik.
            data = await websocket.receive_text()
            # Bisa ditambahkan logic jika butuh merespon ping/command
    except WebSocketDisconnect:
        manager.disconnect(websocket)
