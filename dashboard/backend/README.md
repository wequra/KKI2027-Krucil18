# Dagozilla GCS - Backend API

Modul ini adalah pusat komunikasi dan pengolahan data untuk sistem Ground Control Station (GCS) Dagozilla. Dibangun menggunakan arsitektur modern (FastAPI + Docker) agar mampu menangani *high-frequency telemetry* dan sinkronisasi awan (*Cloud Sync*).

## Fitur Utama
1. **API RESTful (FastAPI):** Endpoint untuk menerima *mission events* dan gambar/log dari Jetson Nano.
2. **Time-Series Telemetry:** Menyimpan data pergerakan kapal (10Hz) menggunakan InfluxDB.
3. **Relational Database:** Menyimpan *mission log*, metrik penilaian (*Scoring Engine*), dan metadata gambar menggunakan PostgreSQL.
4. **WebSocket Broadcast:** Memancarkan data secara *real-time* ke *dashboard* *Frontend* (5Hz).
5. **MediaMTX WebRTC Gateway:** Menerima RTSP dari kapal dan mengubahnya menjadi WebRTC *low-latency* ke browser.
6. **Cloud Sync Agent:** Pekerja *background* yang menyalin data lokal ke VPS Cloud. Dilengkapi dengan SQLite WAL *outbox queue* (tahan terhadap internet terputus).
7. **Image Processor:** Mengompres dan memvalidasi ukuran foto bawah air/permukaan (*OpenCV* + *Pillow*).

---

## ⚠️ Limitasi & Pekerjaan Rumah (TODO)
- **Scoring Engine:** *Rulebook* skor masih menggunakan *default* (`+10 BUOY_PAIR`, `-5 PENALTY`). Nilai dan jenis misi ini **harus disesuaikan ulang** jika buku panduan KKI 2026 sudah rilis secara resmi.
- **Hardware Integration:** Script MAVLink (di `app/services/mavlink_service.py`) saat ini masih mendengarkan sinyal UDP dari Simulator. Jika ingin dicolok ke radio telemetri asli (Pixhawk), `MAVLINK_CONNECTION_STRING` di file `.env` harus diganti (misalnya menjadi `/dev/ttyUSB0` dengan *baudrate* 57600).
- **Jetson API Key:** Untuk percobaan, kunci otorisasi Jetson (`X-API-Key`) belum dikunci secara ketat. Di perlombaan sungguhan, pastikan membuat *secret string* yang kuat.

## Cara Menjalankan (Docker)
Kodingan backend sudah di- *bundle* rapi dalam wadah Docker. Kamu tidak perlu pusing memikirkan *dependencies* di laptopmu.

```bash
# 1. Buat file .env dari template
cp .env.example .env

# 2. Bangun dan nyalakan semua container
docker compose up --build -d

# 3. Cek log jika terjadi error
docker compose logs -f
```

## MediaMTX dan image API

MediaMTX memakai konfigurasi `mediamtx.yml`: RTSP `8554` dan WebRTC `8889`.
Jetson mengirim RTSP ke `rtsp://<GCS_IP>:8554/surface`; browser dashboard
membuka `http://localhost:8889/surface`. Image Jetson dikirim ke
`POST /api/v1/images/` dengan header `X-API-Key` dan ditampilkan oleh panel
existing berdasarkan gambar terbaru untuk masing-masing `image_type`.
