import sqlite3
import json
import logging
import asyncio
import httpx
from datetime import datetime
from pathlib import Path
import os
from app.core.config import settings

logger = logging.getLogger("sync_agent")

DB_PATH = Path("sync_outbox.db")

def init_outbox_db():
    """Inisialisasi SQLite dengan mode WAL untuk antrean data."""
    conn = sqlite3.connect(DB_PATH)
    conn.execute("PRAGMA journal_mode=WAL;")
    conn.execute('''
        CREATE TABLE IF NOT EXISTS outbox (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            topic TEXT NOT NULL,
            payload JSON NOT NULL,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            status TEXT DEFAULT 'PENDING'
        )
    ''')
    conn.commit()
    conn.close()

def enqueue_data(topic: str, payload: dict):
    """Memasukkan data ke dalam outbox queue."""
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.execute(
            "INSERT INTO outbox (topic, payload) VALUES (?, ?)",
            (topic, json.dumps(payload, default=str))
        )
        conn.commit()
        conn.close()
    except Exception as e:
        logger.error(f"Failed to enqueue data: {e}")

async def sync_worker():
    """Background task yang terus-menerus mencoba mengirim data ke Cloud."""
    init_outbox_db()
    
    # URL Cloud VPS dari environment (default ke dummy jika belum diset)
    cloud_url = os.getenv("CLOUD_VPS_URL", "http://localhost:8000/api/v1/dummy_sync")
    
    logger.info(f"Sync Agent started. Target Cloud: {cloud_url}")
    
    async with httpx.AsyncClient(timeout=5.0) as client:
        while True:
            try:
                conn = sqlite3.connect(DB_PATH)
                cursor = conn.cursor()
                cursor.execute("SELECT id, topic, payload FROM outbox WHERE status = 'PENDING' ORDER BY id ASC LIMIT 10")
                rows = cursor.fetchall()
                
                if not rows:
                    conn.close()
                    await asyncio.sleep(2.0)
                    continue

                for row_id, topic, payload_str in rows:
                    payload = json.loads(payload_str)
                    
                    try:
                        # Kirim ke cloud
                        # Misal: POST {cloud_url}/events atau /telemetry tergantung topic
                        target_endpoint = f"{cloud_url}/{topic}"
                        
                        response = await client.post(
                            target_endpoint, 
                            json=payload,
                            headers={"X-API-Key": settings.JETSON_API_KEY}
                        )
                        
                        if response.status_code in (200, 201):
                            # Jika sukses, hapus dari queue
                            conn.execute("DELETE FROM outbox WHERE id = ?", (row_id,))
                            logger.debug(f"Synced {topic} id={row_id} to Cloud")
                        else:
                            # Jika gagal karena server cloud error (bukan koneksi), biarkan pending
                            logger.warning(f"Cloud returned {response.status_code} for {topic}")
                            break # Hentikan batch ini, coba lagi nanti

                    except httpx.RequestError as exc:
                        # Jika internet mati, hentikan pengiriman dan tunggu
                        logger.warning(f"Internet down or Cloud unreachable: {exc}")
                        break

                conn.commit()
                conn.close()
                await asyncio.sleep(1.0)
                
            except Exception as e:
                logger.error(f"Sync Worker error: {e}")
                await asyncio.sleep(5.0)
