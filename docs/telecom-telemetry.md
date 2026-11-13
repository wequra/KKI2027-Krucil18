# Sistem Telekomunikasi, Jaringan, dan Telemetri ASV

## 1. Arsitektur Komunikasi Dual-Link
Sistem komunikasi wahana *Autonomous Surface Vessel* (ASV) memisahkan kanal kendali kritis dan kanal data *high-bandwidth* melalui arsitektur *dual-link*:

1. **Radio Telemetri 433 MHz (MAVLink Control & Heartbeat):**
   * Menggunakan modul SiK Telemetry Radio 433 MHz (100 mW, antena *omnidirectional* 5 dBi) antara *Flight Controller* (ArduPilot Rover) dan laptop *Ground Control Station* (GCS).
   * Frekuensi 433 MHz ditetapkan secara konsisten di seluruh dokumen proposal untuk menghindari inkonsistensi spesifikasi frekuensi dan meminimalkan *multipath fading* di permukaan air.
   * Menyalurkan paket `GLOBAL_POSITION_INT`, `VFR_HUD`, `SYS_STATUS`, dan *heartbeat* pada frekuensi 5 Hz.
2. **Wi-Fi 5 GHz / 4G LTE Fallback (Visual Stream, Event & Image Ingest):**
   * Menghubungkan *Onboard Computer* (NVIDIA Jetson) ke *Backend Server* (FastAPI) pada GCS maupun *Cloud Mirror*.
   * Menyalurkan *stream* video langsung berbasis **WebRTC (MediaMTX)** serta mengirim *event* misi dan metadata citra (*Rumpon*, *Sonar*, dan *TPI Docking*) melalui REST API terautentikasi (`X-API-Key`).

## 2. Alur Data (*Data-Flow Reasoning*)
1. **Wahana (*Boat Zone*):** Sensor GNSS/IMU dibaca oleh *Flight Controller* dan diteruskan ke ROS 2 melalui *node* **MAVROS**. *Node* YOLOv8 dan *Mission FSM* mendeteksi objek serta membangkitkan *event* dan tangkapan citra ber-*geotag*.
2. **Ingesti & Validasi (*Backend Zone*):** Modul `mavlink_ingest.py` (`pymavlink`) mengonversi kecepatan wahana ke satuan **km/jam** dan **knot**, lalu mengirimkan data ke `/api/v1/ingest/telemetry`. Server **FastAPI** memvalidasi seluruh *payload* terhadap skema *Data Contract* (`telemetry.schema.json`, `event.schema.json`, `image-meta.schema.json`) dan menyimpannya ke basis data.
3. **Distribusi *Real-Time* (*Dashboard Zone*):** Setiap data yang lolos validasi langsung disiarkan (*broadcast*) melalui **WebSocket (`/ws/stream`)** ke *Web Monitoring Dashboard* React.

## 3. *Store-and-Forward* dan *Failsafe Hold*
* **Outbox Queue (*Store-and-Forward*):** Jika koneksi Wi-Fi terputus sementara, *event* misi dan citra disimpan di antrean lokal pada Jetson dan dikirim ulang secara berurutan saat jaringan pulih.
* **Failsafe *Hold Position*:** Sesuai regulasi KKI, kehilangan sinyal *heartbeat* melebihi batas waktu memicu mode **`HOLD` (*Stop and Hold in Place*)**, bukan *Return-to-Launch* (RTL), agar wahana berhenti dengan aman tanpa menabrak lintasan pelampung (*buoy*).
