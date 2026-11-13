# Tutorial pengujian ASV di Docker

Dokumen ini menjalankan pipeline berikut tanpa mengubah instalasi ROS host:

```text
/dev/video0 atau synthetic_buoy_camera
  -> /camera/image_raw
  -> vision_detector_node
  -> /asv/obstacles
  -> buoy_centering_node
  -> /asv/cmd_vel
  -> mavros_bridge_node
  -> /mavros/setpoint_velocity/cmd_vel_unstamped
```

Container menggunakan ROS 2 Jazzy dan `ROS_DOMAIN_ID=42`. Karena domain berbeda,
node di container tidak bercampur dengan node ROS host yang memakai domain default.
Semua instalasi `apt`, ROS Jazzy, MAVROS, OpenCV, dan hasil `colcon build` berada di
image Docker, bukan di ROS host.

## 1. Persiapan host

Jalankan dari root repository:

```bash
cd /dagozilla_ws
docker --version
docker compose version
```

Untuk pengujian webcam, pastikan device tersedia dan tidak sedang dipakai aplikasi
lain:

```bash
ls -l /dev/video0
```

Jika kamera berada di `/dev/video1`, ubah mapping pada
`docker/compose.camera.yaml` atau buat mapping override yang sesuai.

## 2. Build image

Terminal 1:

```bash
cd /dagozilla_ws
docker compose build
docker compose up -d
docker compose ps
```

Build pertama mengunduh ROS 2 Jazzy, MAVROS, OpenCV, dependency rosdep, dan dataset
GeographicLib. Build berikutnya memakai cache selama Dockerfile/source tidak berubah.

Periksa lingkungan container:

```bash
docker compose exec asv bash
printenv ROS_DISTRO
printenv ROS_DOMAIN_ID
ros2 pkg list | grep -E '^(perception|guidance_control|mavros_bridge|simulation)$'
exit
```

Hasil yang diharapkan: `jazzy`, domain `42`, dan keempat package ditemukan.

## 3. Smoke test penuh tanpa webcam dan Pixhawk

Mode ini membuat gambar buoy merah-hijau secara sintetis. Gambar tetap melewati
vision detector asli; simulator tidak langsung menyuntikkan `/asv/obstacles`.

Terminal 1 — jalankan pipeline:

```bash
cd /dagozilla_ws
docker compose exec asv bash
ros2 launch simulation synthetic_buoy_pipeline.launch.py
```

Terminal 2 — pantau hitungan gate:

```bash
cd /dagozilla_ws
docker compose exec asv bash
ros2 topic echo /asv/buoy_gate_count std_msgs/msg/UInt8
```

Terminal 3 — pantau perintah guidance:

```bash
cd /dagozilla_ws
docker compose exec asv bash
ros2 topic echo /asv/cmd_vel geometry_msgs/msg/Twist
```

Terminal 4 — pastikan bridge meneruskan perintah:

```bash
cd /dagozilla_ws
docker compose exec asv bash
ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist
```

Indikator berhasil:

- log berubah `SEARCHING -> APPROACHING -> PASSING`;
- `/asv/buoy_gate_count` naik sampai `data: 10`;
- log menampilkan `FINISHED - STOP`;
- setelah selesai, `linear.x` dan `angular.z` bernilai nol;
- topic keluaran bridge sama dengan `/asv/cmd_vel`.

Urutan sintetis biasanya selesai sekitar 30–50 detik. Tekan `Ctrl+C` di Terminal 1 setelah
verifikasi. Hentikan `ros2 topic echo` di terminal lain dengan `Ctrl+C`.

Untuk tes singkat tiga gate:

```bash
ros2 launch simulation synthetic_buoy_pipeline.launch.py total_gates:=3
```

## 4. Uji webcam laptop tanpa MAVROS

Hentikan container dasar, lalu buat ulang dengan akses hanya ke kamera:

```bash
cd /dagozilla_ws
docker compose down
docker compose -f compose.yaml -f docker/compose.camera.yaml up -d
```

Terminal 1 — jalankan pipeline webcam dalam mode headless:

```bash
cd /dagozilla_ws
docker compose -f compose.yaml -f docker/compose.camera.yaml exec asv bash
ros2 launch guidance_control asv_vision_guidance.launch.py \
  camera_device:=/dev/video0 \
  use_camera:=true \
  use_mavros:=false \
  show_gui:=false
```

`use_mavros:=false` berarti tidak ada koneksi FCU/Pixhawk. Bridge tetap berjalan
agar forwarding topic dapat diuji, tetapi tidak mempunyai perangkat tujuan.

Terminal 2 — cek kamera:

```bash
cd /dagozilla_ws
docker compose -f compose.yaml -f docker/compose.camera.yaml exec asv bash
ros2 topic hz /camera/image_raw
```

Terminal 3 — cek deteksi:

```bash
cd /dagozilla_ws
docker compose -f compose.yaml -f docker/compose.camera.yaml exec asv bash
ros2 topic echo /asv/obstacles msgs/msg/ObstacleArray
```

Terminal 4 — cek command dan penghitung:

```bash
cd /dagozilla_ws
docker compose -f compose.yaml -f docker/compose.camera.yaml exec asv bash
ros2 topic echo /asv/cmd_vel geometry_msgs/msg/Twist
```

Gunakan dua objek merah dan hijau di depan kamera. Pastikan pencahayaan cukup.
Jika tidak terdeteksi, kalibrasi HSV sebelum mengubah logika guidance.

## 5. Menampilkan GUI OpenCV dari container

Bagian ini opsional. Mode headless adalah default dan paling stabil.

Pada host Linux dengan X11/XWayland:

```bash
xhost +local:root
```

Kemudian jalankan pipeline dengan:

```bash
ros2 launch guidance_control asv_vision_guidance.launch.py \
  camera_device:=/dev/video0 \
  use_mavros:=false \
  show_gui:=true
```

Setelah selesai, cabut kembali izin X11:

```bash
xhost -local:root
```

Jika `$DISPLAY` kosong, tetap gunakan `show_gui:=false` dan periksa topic ROS.

## 6. HSV tuner

Terminal 1 menjalankan kamera dan detector seperti pada bagian 4. Hentikan launch
tersebut, lalu izinkan X11 dan jalankan:

```bash
xhost +local:root
docker compose -f compose.yaml -f docker/compose.camera.yaml exec asv bash
ros2 launch perception hsv_tuner.launch.py camera_device:=/dev/video0
```

Hotkey: `R` preset merah, `G` preset hijau, dan `S` mencetak YAML. Setelah selesai:

```bash
xhost -local:root
```

Jangan menjalankan dua camera node sekaligus karena `/dev/video0` umumnya hanya
dapat dibuka satu proses pada satu waktu.

## 7. Build ulang setelah source berubah

Image tidak me-mount source host. Ini disengaja agar test reproducible dan instalasi
hasil build tidak tertimpa bind mount. Setelah source berubah:

```bash
cd /dagozilla_ws
docker compose down
docker compose build
docker compose up -d
```

Untuk memaksa build bersih:

```bash
docker compose build --no-cache
```

Docker hanya digunakan pada laptop. Untuk Jetson native tanpa Docker, ikuti
[JETSON_NATIVE.md](JETSON_NATIVE.md).

## 8. Menghentikan dan membersihkan

Menghentikan container tanpa menghapus image:

```bash
cd /dagozilla_ws
docker compose down
```

Perintah ini tidak menghapus ROS host atau source repository. Bila ingin menghapus
image proyek saja:

```bash
docker image rm dagozilla-asv:jazzy
```

## 9. Troubleshooting cepat

### Kamera gagal dibuka

```bash
ls -l /dev/video0
docker compose -f compose.yaml -f docker/compose.camera.yaml exec asv ls -l /dev/video0
```

Pastikan container dibuat memakai override camera, bukan hanya `docker compose up`.

### Topic tidak terlihat antar-terminal

```bash
printenv ROS_DOMAIN_ID
ros2 daemon stop
ros2 daemon start
```

Semua terminal container harus menggunakan domain yang sama, default `42`.

### MAVROS tidak muncul

Pastikan launch memakai `use_mavros:=true`, device serial ada di container, baud rate
sesuai parameter Pixhawk, dan port tidak sedang dibuka QGroundControl secara langsung.

### GUI gagal dibuka

Gunakan `show_gui:=false`. GUI bukan syarat pipeline kontrol; hasil tetap tersedia di
`/asv/guidance_debug`, `/asv/obstacles`, dan `/asv/cmd_vel`.
