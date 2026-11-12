# Dagozilla ASV Backend API Documentation

Selamat datang di modul Backend untuk Sistem Kapal Otonom Dagozilla (KKCTBN/KKI 2026). Modul ini menyediakan layanan API, Time-Series Database (InfluxDB), Relational Database (PostgreSQL), dan saluran WebSocket untuk transmisi data *real-time* ke Frontend/Dashboard.

---

## 🚀 Cara Menjalankan (Docker)

Pastikan **Docker Desktop** sudah menyala.
Jalankan perintah ini di dalam folder `backend/`:

```bash
docker compose up --build -d
```

Akan ada 3 layanan yang menyala:
1. **API Server (FastAPI)**: `http://localhost:8000`
2. **InfluxDB (Database Telemetri)**: `http://localhost:8086`
3. **PostgreSQL (Database Data Misi)**: `localhost:5432`

---

## 📡 Integrasi untuk Tim ROS 2 (Autonomy)

Agar UI Dashboard bisa menyala dan menampilkan log kapal, tim ROS 2 (`mission` / `vision`) **Wajib** menembak API Backend saat kapal mendeteksi sesuatu atau berganti mode.

**URL Endpoint:** `POST http://localhost:8000/api/v1/events/`
**Header Wajib:** `X-API-Key: dagozilla_super_secret_key_2026`

**Contoh Python Script (Gunakan library `requests` di node ROS 2):**

```python
import requests
from datetime import datetime, timezone

# Contoh 1: Kapal mendeteksi bola merah di grid C3
payload = {
    "event_type": "BUOY_PAIR",
    "timestamp": datetime.now(timezone.utc).isoformat(),
    "grid_position": "C3",
    "metadata": {"color": "red_buoy_detected"}
}

headers = {"X-API-Key": "dagozilla_super_secret_key_2026"}
response = requests.post("http://localhost:8000/api/v1/events/", json=payload, headers=headers)
```

**Daftar `event_type` yang valid:**
- `SYSTEM_INIT`
- `START`
- `BUOY_PAIR` (Saat mendeteksi gerbang / pelampung)
- `PENALTY` (Jika menabrak)
- `SURFACE_IMAGING` (Mengambil foto atas air)
- `UNDERWATER_IMAGING` (Mengambil foto bawah air)
- `DOCKING`
- `APPROACHING`
- `FINISH`
- `ABORT` (Berhenti darurat)

---

## 🖥️ Integrasi untuk Tim Frontend (Vercel)

Semua endpoint dilindungi oleh API Key, kecuali disebutkan khusus. Masukkan `X-API-Key: dagozilla_super_secret_key_2026` di setiap HTTP Request Headers.

### 1. REST API (Ambil Data Ringkasan)
Kalian bisa melihat dokumentasi lengkap, testing interaktif, dan format JSON dari setiap endpoint di Swagger UI:
👉 **Buka di Browser:** `http://localhost:8000/docs`

Beberapa Endpoint Kunci:
- `GET /api/v1/dashboard/summary`: Mengambil ringkasan poin, status baterai, dan log event terakhir.
- `GET /api/v1/events/`: Mengambil sejarah event (Pagination).
- `GET /api/v1/telemetry/latest`: Mendapatkan koordinat (Lat/Lon) kapal terakhir.

### 2. Live WebSocket (Grafik Peta Bergerak)
Untuk membuat kapal bergerak halus di peta tanpa perlu me-refresh halaman (polling HTTP), hubungkan Frontend Vercel kalian ke WebSocket ini:

**Koneksi:** `ws://localhost:8000/ws/dashboard`

Begitu terkoneksi, kalian akan menerima rentetan JSON otomatis sebanyak 5 kali per detik (5Hz). 
**Contoh Data yang Diterima:**
```json
{
  "type": "telemetry_update",
  "data": {
    "latitude": -6.8915,
    "longitude": 107.6107,
    "heading": 45.5,
    "speed": 1.2,
    "battery_voltage": 14.8,
    "battery_remaining": 95,
    "roll": -1.2,
    "pitch": 0.5,
    "yaw": 45.5,
    "timestamp": "2026-07-20T12:00:00Z"
  }
}
```
*Tugas Frontend: Parse JSON ini dan update koordinat Marker ikon kapal di React Leaflet / Mapbox secara *real-time*!*
