# ASV Software Stack — Tim Dagozilla — KKI 2026

Repository software untuk kapal Autonomous Surface Vessel (ASV) kategori
Prototype KKI 2026. Stack: **ROS2 Jazzy** + **Pixhawk (ArduRover)** via **MAVROS**.

## Arsitektur Singkat

```
Pixhawk (ArduRover firmware)
        │  MAVLink
        ▼
   MAVROS (driver resmi PX4/ArduPilot <-> ROS2)
        │
        ▼
 mavros_bridge   ← baca state/GPS Pixhawk, service ganti mode
        │
        ▼
 mission (state machine misi)
        │  publish active_waypoint
        ▼
 guidance_control (waypoint follower / PID / pure pursuit / buoy centering)
        │  publish cmd_vel
        ▼
   MAVROS setpoint  →  Pixhawk (mode GUIDED)

 perception (deteksi buoy/obstacle OpenCV, HSV tuner, camera stream) ──► obstacles ──► guidance/mission
 navigation (opsional: EKF lokal jika GPS Pixhawk kurang presisi)
```

## Pengujian Docker

Panduan terminal-per-terminal:

- laptop dengan Docker: [docs/DOCKER_TESTING.md](docs/DOCKER_TESTING.md);
- Jetson native tanpa Docker: [docs/JETSON_NATIVE.md](docs/JETSON_NATIVE.md).

## Struktur Package

| Package | Bahasa | Fungsi |
|---|---|---|
| `msgs` | interfaces (msg/srv) | Definisi message & service kustom (Waypoint, MissionStatus, ObstacleArray, dst) |
| `mavros_bridge` | C++ | Jembatan node ROS2 ↔ Pixhawk lewat MAVROS |
| `guidance_control` | C++ | Guidance & control: waypoint follower, buoy centering dengan aturan safety stop & live visualizer stream |
| `navigation` | C++ (belum ada node) | State estimation tambahan (opsional, mis. EKF lokal) |
| `vision` | C++ | Stub deteksi obstacle generik (lidar/sensor non-vision) |
| `perception` | C++ | Driver kamera OpenCV, vision detector buoy/obstacle KKI 2026, & HSV calibration tool |
| `mission` | C++ | State machine misi ASV KKI 2026: buoy, imaging, docking, timeout, penalti |
| `bringup` | launch/config | Launch file utama + file konfigurasi terpusat |
| `simulation` | launch/config | Konfigurasi simulasi Gazebo + ArduPilot SITL untuk testing tanpa kapal fisik |

---

See [docs/FULL_MISSION_AND_GCS.md](docs/FULL_MISSION_AND_GCS.md) for the autonomous full-mission launch, safety architecture, and laptop-facing HTTP schemas.

## Cara Pakai & Tools Utama

### 1. Build Workspace
```bash
cd dagozilla-asv
colcon build --symlink-install
source install/setup.bash
```

### 2. HSV Threshold Calibration Tool (`perception`)
Digunakan untuk kalibrasi batas warna HSV buoy (Merah, Hijau, Blue Dock) secara interaktif dengan GUI OpenCV:
```bash
ros2 launch perception hsv_tuner.launch.py camera_device:=/dev/video0
```
- **Fitur & Hotkey GUI**:
  - **`S` / `P`**: Print/dump nilai HSV saat ini langsung formatted dalam format YAML ke terminal (tinggal copy-paste ke `vision_params.yaml`).
  - **`R`**: Load preset HSV Buoy Merah (`[0-10, 120-255, 70-255]`).
  - **`G`**: Load preset HSV Buoy Hijau (`[35-85, 80-255, 60-255]`).
  - **`B`**: Load preset HSV Blue Dock (`[95-130, 80-255, 50-255]`).

### 3. ASV Vision Guidance & Buoy Centering Live Stream (`guidance_control`)
Menjalankan pipeline navigasi berbasis penglihatan komputer (kamera + vision detector + buoy centering + MAVROS bridge):
```bash
ros2 launch guidance_control asv_vision_guidance.launch.py camera_device:=/dev/video0
```
- **Fitur Live Display Window & Feedback Visual**:
  - Visualisasi deteksi lingkaran & label objek (`RED BUOY`, `GREEN BUOY`, `BLUE DOCK`).
  - Garis vektor penghubung buoy merah & hijau beserta crosshair titik tengah target (midpoint target).
  - Indikator bar kemudi di bagian bawah tampilan kamera.
  - **Banner Status OSD**:
    - **HIJAU**: `[ STATUS: CENTERING ACTIVE ]` saat kedua buoy (merah & hijau) terdeteksi.
    - **MERAH**: `[ STATUS: SAFETY STOP - BUOY MISSING ]` saat kurang dari 2 buoy terdeteksi (aturan keselamatan ketat: motor otomatis mati).
  - Stream gambar terannotasi juga dipublikasikan ke topik ROS `/asv/guidance_debug` (`sensor_msgs/Image`) untuk remote viewer / GCS.
  - HTTP MJPEG streamer aktif secara default di `http://192.168.88.253:8080/stream.mjpg` untuk web GCS. Browser tidak dapat memutar RTSP murni secara langsung.

Contoh elemen pada `dagozilla-gcs`:
```html
<img src="http://192.168.88.253:8080/stream.mjpg" alt="ASV camera stream">
```

### 4. System Bringup Utama (Kapal Asli)
```bash
ros2 launch bringup bringup.launch.py fcu_url:=/dev/ttyACM0:115200
```
Mulai state machine misi:
```bash
ros2 service call /asv/start_mission std_srvs/srv/Trigger
```

Services pendukung sesuai panduan KKI 2026:
```bash
ros2 service call /asv/complete_buoy_pair std_srvs/srv/Trigger
ros2 service call /asv/mark_surface_image std_srvs/srv/Trigger
ros2 service call /asv/mark_underwater_image std_srvs/srv/Trigger
ros2 service call /asv/complete_docking std_srvs/srv/Trigger
ros2 service call /asv/record_penalty std_srvs/srv/Trigger
```

### 5. Full Mission dengan Pilihan Lintasan

Full mission memakai kamera utama dan kamera underwater, lalu mengaktifkan
urutan buoy, imaging, dan docking. Pilih lintasan sebelum node dimulai:

```bash
source /opt/ros/jazzy/setup.bash
source /dagozilla_ws/install/setup.bash

# Lintasan A: maneuver SEARCHING setelah pair ke-3 dan ke-7: maju-kiri
ros2 launch bringup asv_full_mission.launch.py \
  use_mavros:=true \
  fcu_url:=/dev/ttyACM0:115200 \
  main_camera_device:=/dev/video0 \
  underwater_camera_device:=/dev/video1 \
  control_backend:=rc_override \
  course_layout:=A
```

Untuk lintasan B, ganti argumen terakhir menjadi `course_layout:=B`; maneuver
setelah pair ke-3 dan ke-7 menjadi maju-kanan. Pada kedua lintasan, SEARCHING
sebelum pair pertama memakai `linear.x=0.3`, sedangkan SEARCHING lain tetap
menggunakan parameter normal `search_linear_speed`.

Mulai state machine setelah seluruh node dan FCU siap:

```bash
ros2 service call /asv/start_mission std_srvs/srv/Trigger "{}"
```

---

## Alur Kerja Tim (Rekomendasi)

1. **Branch per fitur**: `feat/guidance-pid`, `feat/obstacle-yolo`, dst. Jangan langsung push ke `main`.
2. **Testing di simulasi dulu** (`simulation`, ArduPilot SITL) sebelum diuji di kapal fisik/kolam.
3. **Satu orang pegang `mavros_bridge` & `mission`** (interface kritis), lainnya bisa paralel di `guidance_control` / `perception`.
4. **Isi `docs/`** dengan wiring diagram, daftar topic/service, dan catatan kalibrasi kompas/GPS — penting banget buat laporan teknis & saat ditanya juri.

---

## Rulebook Reference

Sesuaikan task (navigation channel, obstacle avoidance, docking, dst) dan
scoring dengan **Buku Panduan KKI 2026** kategori ASV dari BELMAWA/Puspresnas.
Simpan salinan PDF-nya di `docs/rulebook/`.
