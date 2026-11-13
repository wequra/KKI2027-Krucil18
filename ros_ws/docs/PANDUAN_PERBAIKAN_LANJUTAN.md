# Panduan Perbaikan Lanjutan — `ros_ws`

Dokumen ini mencatat hasil pembersihan awal dan daftar
perbaikan yang **sengaja belum dikerjakan** karena butuh 
pengujian di hardware.

> **Status:** semua perubahan pembersihan awal masih *uncommitted*. Belum ada
> `colcon build` maupun `colcon test` yang dijalankan setelah perubahan.

---

## 0. Wajib dulu: verifikasi pembersihan awal

Sebelum commit, jalankan di Docker atau Jetson:

```bash
cd ~/dagozilla_ws
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

# bobot YOLO harus ter-install dari lokasi barunya
test -s "$(ros2 pkg prefix --share perception)/weights/best.pt" && echo OK

colcon test --packages-select guidance_control simulation --event-handlers console_direct+
colcon test-result --verbose
```

Lalu uji singkat tiga node Python yang diformat ulang (logikanya tidak diubah,
tetapi belum pernah dijalankan setelah diformat):

```bash
ros2 launch bringup asv_full_mission.launch.py use_mavros:=false
# di terminal lain:
curl http://localhost:8090/health
curl http://localhost:8090/api/telemetry
ros2 topic echo /asv/cmd_vel --once
```

### Ringkasan perubahan yang perlu diverifikasi

| File | Perubahan |
|---|---|
| `ekstrak.py` (root) | Dihapus → diganti `src/perception/tools/extract_video_frames.py` (pakai argumen CLI `--input/--output/--skip`) |
| `install_geographiclib_datasets.sh` (root) | Dihapus; sudah diunduh otomatis oleh `docker/Dockerfile` dan `scripts/setup_env.sh` |
| `src/ball_detector_pkg/weights/best.pt` | Dipindah ke `src/perception/weights/best.pt` |
| `src/perception/CMakeLists.txt` | `install(FILES ../ball_detector_pkg/...)` → `weights/best.pt` |
| `src/simulation/CMakeLists.txt` | Test pytest dibungkus `if(BUILD_TESTING)` |
| `src/bringup/launch/asv_full_mission.launch.py` | `/home/rie/...` → `os.path.expanduser('~/...')`; default lain **tidak** diubah |
| `src/mission/scripts/mission_controller_node.py` | Format ulang PEP 8 (tanpa perubahan logika) |
| `src/mission/scripts/gcs_bridge_node.py` | Format ulang PEP 8 (tanpa perubahan logika) |
| `src/mission/scripts/mission_cmd_mux_node.py` | Format ulang blok `main()` |
| `scripts/sync_to_jetson.sh`, `README.md`, `docs/gate_tracking_audit.md` | Contoh path diseragamkan ke `~/dagozilla_ws` |
| `.gitignore` | Tambah `.pytest_cache/`, `*.swp`, `*.swo`, `*~` |

Jika ada yang rusak, kembalikan per file:

```bash
git restore <path>                      # file yang diubah
git restore src/ball_detector_pkg       # kembalikan bobot lama
```

---

## 1. Item tertunda — butuh keputusan tim

### 1.1 Kredensial default `jetson_api_key = test_key`

- **Lokasi:** `src/bringup/launch/asv_full_mission.launch.py` (argumen `jetson_api_key`)
- **Masalah:** kredensial tersimpan di repo sebagai nilai default.
- **Risiko perubahan:** jika GCS masih memakai `test_key`, media bridge berhenti tersambung.
- **Langkah:**
  1. Ganti key di sisi GCS (`dagozilla-gcs`) dengan key baru.
  2. Ubah default menjadi `''`, lalu berikan key lewat environment variable:
     ```python
     DeclareLaunchArgument('jetson_api_key',
         default_value=os.environ.get('DAGOZILLA_JETSON_API_KEY', ''))
     ```
  3. Simpan key di `~/.bashrc` Jetson atau file `.env` (sudah di-ignore oleh `.gitignore`).
  4. Pertimbangkan *rotate* key lama karena sudah tercatat di riwayat git.

### 1.2 IP hardcoded `192.168.88.253` di `README.md`

- **Lokasi:** `README.md` baris ~88 dan ~92.
- **Langkah:** ganti dengan placeholder `JETSON_IP` agar konsisten dengan
  `docs/FULL_MISSION_AND_GCS.md`. Jika IP ini memang IP tetap Jetson tim, catat
  di bagian tersendiri (misal tabel "Konfigurasi Jaringan Tim").

### 1.3 Email maintainer `tim@example.com`

- **Lokasi:** `package.xml` di `bringup`, `guidance_control`, `mavros_bridge`,
  `mission`, `msgs`, `navigation`, `perception`, `simulation`, `vision`.
- **Langkah:** ganti dengan nama + email PIC tiap package. Tidak memengaruhi build.

### 1.4 Package `vision` (stub) dan `navigation` (kosong)

- **Kondisi:**
  - `vision`: hanya `obstacle_detector_node` yang mem-publish `ObstacleArray`
    kosong ke `/asv/obstacles`. Tidak dipakai oleh launch file mana pun.
  - `navigation`: tidak punya node, hanya `CMakeLists.txt` dan folder kosong.
  - `bringup/package.xml` memiliki `<exec_depend>vision</exec_depend>`.
- **Opsi A — hapus:**
  1. Hapus `<exec_depend>vision</exec_depend>` dari `src/bringup/package.xml`.
  2. Hapus `src/vision/` dan/atau `src/navigation/`.
  3. Perbarui tabel package di `README.md`.
  4. `colcon build` ulang dari bersih.
- **Opsi B — pertahankan sebagai placeholder:** tambahkan `README.md` singkat
  di masing-masing package yang menjelaskan rencana isinya (misalnya EKF lokal
  untuk `navigation`, lidar untuk `vision`).

> [!WARNING]
> Jangan hapus `.gitkeep` di `src/mavros_bridge/config/` dan
> `src/mavros_bridge/launch/`. Folder itu hanya berisi `.gitkeep`, dan
> `install(DIRECTORY launch config ...)` di CMakeLists akan gagal jika folder
> tersebut hilang. Hal yang sama berlaku untuk folder `config/` dan `launch/`
> di `vision`, `navigation`, `mission/launch`, dan `simulation/config`.

### 1.5 `src/cv_bridge` yang di-vendor

- **Kondisi:** source `cv_bridge` lengkap masuk lewat commit
  `5c6cb70 "Update from Jetson testing"`, padahal `ros-jazzy-cv-bridge` juga
  di-install via apt di Dockerfile dan `setup_env.sh`.
- **Dugaan:** dipakai agar `cv_bridge` dikompilasi terhadap OpenCV yang sama
  dengan OpenCV di Jetson (lihat pengecekan `OpenCV_CONFIG_PATH` di CMakeLists
  `perception` dan `guidance_control`).
- **Langkah:**
  1. Tanyakan ke penulis commit tersebut alasannya.
  2. Di Jetson, coba build **tanpa** `src/cv_bridge` (pindahkan sementara ke
     luar `src/`). Jika build dan runtime kamera/YOLO normal, hapus vendor.
  3. Jika memang dibutuhkan, tulis alasannya di `docs/JETSON_NATIVE.md` dan
     catat versi upstream yang di-vendor.

### 1.6 `TODO_#1.md` di root

- **Langkah:** pindahkan ke `docs/` (misal `docs/prd/TODO_01_delaunay_ilos.md`)
  dan beri tautan dari `README.md`. Nama file dengan karakter `#` juga
  merepotkan di shell.

---

## 2. Perapian lain (risiko rendah)

| Item | Langkah |
|---|---|
| `compose.yaml` di root, `compose.camera.yaml` di `docker/` | Satukan lokasinya, lalu perbarui `docs/DOCKER_TESTING.md` |
| Executable bit file Python | Script yang di-install lewat `install(PROGRAMS ...)` masih `100644` di git. Jalankan `git update-index --chmod=+x <file>` untuk script node dan tools agar bisa dieksekusi langsung |
| Linter belum dipakai | Aktifkan `ament_lint_auto` (sudah ada di `test_depend`) dengan `ament_lint_auto_find_test_dependencies()` di CMakeLists, atau jalankan `ament_flake8` / `ament_cpplint` secara manual |
| Belum ada CI | Tambahkan `.github/workflows/ci.yml`: `colcon build` + `colcon test` pada image `ros:jazzy` |
| Deskripsi `package.xml` | Beberapa sudah tidak akurat (misal `simulation` menyebut Gazebo + SITL, padahal isinya kamera sintetis + pytest) |

---

## 3. Peningkatan teknis jangka menengah

Tidak termasuk pembersihan; dicatat sebagai roadmap. Detail lengkapnya ada di
`TODO_#1.md`.

1. **Pecah `buoy_centering_node.cpp` (~1300 baris)** menjadi modul pairing,
   lifecycle gate, steering controller, dan visualisasi. Pertahankan
   `test_gate_logic.cpp` sebagai jaring pengaman sebelum refactor.
2. **Inferensi YOLO di GPU.** `ball_detector_node.py` saat ini memakai
   `device: "cpu"`. Coba `device: 0` (CUDA) terlebih dahulu, baru TensorRT.
3. **Estimasi posisi metrik.** `ObstacleArray` masih berisi koordinat piksel.
   Butuh kalibrasi kamera (`camera_info`) dan proyeksi ke bidang air dengan IMU.
4. **Composable nodes** untuk pipeline kamera → detektor → guidance agar image
   tidak disalin antar-proses.
5. **Kompensasi deadband ESC** di `cmd_vel_to_rc_override_node.cpp`.

---

## Checklist singkat

- [ ] Bagian 0: build + test lulus, lalu commit pembersihan awal
- [ ] 1.1 Kredensial `jetson_api_key`
- [ ] 1.2 IP di README
- [ ] 1.3 Email maintainer
- [ ] 1.4 Keputusan package `vision` / `navigation`
- [ ] 1.5 Uji build tanpa `src/cv_bridge` di Jetson
- [ ] 1.6 Pindahkan `TODO_#1.md`
- [ ] Bagian 2: perapian risiko rendah
