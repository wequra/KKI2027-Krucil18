# app/core/websocket.py
from fastapi import WebSocket
from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)

class ConnectionManager:
    def __init__(self):
        # Menyimpan daftar websocket yang aktif terhubung
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"Client terhubung. Total client: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            logger.info(f"Client terputus. Total client: {len(self.active_connections)}")

    async def broadcast(self, message_type: str, payload: Dict[str, Any]):
        """
        Kirim pesan ke semua client yang terhubung.
        Format payload: {"type": "event|telemetry", "data": {...}}
        """
        if not self.active_connections:
            return  # Tidak ada client yang connect, skip broadcast

        message = {
            "type": message_type,
            "data": payload
        }
        
        # Kirim ke semua connection
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception as e:
                logger.error(f"Error broadcasting to client: {e}")
                # Auto disconnect jika gagal kirim (client mungkin sudah mati/terputus diam-diam)
                self.disconnect(connection)

# Singleton instance
manager = ConnectionManager()
