# Audit dan implementasi gate tracking — 7 September 2026

Status: implementasi tersedia untuk validasi target. Pipeline hardware sebelumnya
sudah bekerja; audit ini tidak mengulang diagnosis motor atau membatalkan hasil
steering/target-locking sebelumnya. Laptop menyediakan ROS Lyrical, bukan Jazzy.
Tidak ada instalasi Ultralytics, PyTorch, perubahan distro, atau system Python.

## Root cause dan perubahan

| Masalah | Existing | Perubahan |
|---|---|---|
| Lock berpindah/anggota tertukar | Association warna menggunakan posisi hingga 0.25 lebar gambar dan rasio radius 3, tanpa identitas fisik | Kedua ID + session harus sama, ditambah batas gerakan/radius; ID baru tidak ditransfer ke lock |
| Cross-pair | Semua red×green, mutual-nearest horizontal; ties tidak membatasi pemakaian satu buoy | Assignment interval global: maksimum jumlah opening yang tidak overlap, kemudian minimum geometric cost |
| Gate pertama kalah | Skor utama `min(red_radius, green_radius)` | Skor posisi depan/midpoint, posisi vertikal, mean radius, confidence, dan continuity; konfirmasi beberapa frame |
| Reacquire gate passed | Satu target diblokir selama tiga callback | Daftar retired selama 30 s: setiap constituent ID disuppress, ditambah midpoint/radius history untuk ID baru |
| Gate 3/7 langsung belok | SEARCHING memberi 0.6 dan yaw +0.4 A / -0.4 B selama count tetap 3/7 | Trigger satu kali dari event passed → CLEARING → TURNING → TURN_STABILIZING |
| Belok tanpa completion | Tidak ada target heading maupun stabilization | Compass heading relatif, wrap [-180,180), consecutive samples, timeout stop |
| Kamera terhenti | Command dan loss counter bergantung callback | Watchdog detection berhenti dan melepas lock tanpa menambah count; timer heading terpisah |

Audit mencakup node/header/config buoy, detector, launch vision/guidance/bringup/full
mission, mission manager/controller/mux, message, serta test locking/full mission.
Mission manager menerima `/asv/buoy_gate_count`; full mission tetap meremap output
buoy ke `/asv/cmd_vel/buoy`, dan mux tetap pemilik final `/asv/cmd_vel`.

## State sebelum dan sesudah

Sebelum: SEARCHING → APPROACHING → PASSING → SEARCHING atau FINISHED.
Lock langsung pada satu frame. PASSING dimulai jika kedua radius >= threshold;
selesai setelah associated gate hilang/mengecil selama `lost_frames_for_pass`.
`markGatePassed()` satu-satunya increment count. Special turn adalah cabang command
SEARCHING pada count 3/7, sehingga candidate baru dapat mengambil alih belokan.

Sesudah: detection → candidate assignment → konfirmasi → locked APPROACHING →
PASSING → retirement/count. Event GATE_PASSED diimplementasikan dalam
`markGatePassed()`, bukan state berumur satu callback. Gate normal menuju SEARCHING;
3/7 menuju CLEARING_GATE → TURNING → TURN_STABILIZING → SEARCHING. Gate terakhir
tetap FINISHED. Trigger hanya dari event passed, bukan pemeriksaan count di SEARCHING.

`linear.x > 0` tetap forward; `angular.z > 0` tetap left.

| State | linear.x | angular.z | Exit condition | Gate acquisition |
|---|---|---|---|---|
| SEARCHING | 0.3 sebelum gate pertama; sesudahnya search_linear_speed 0.15 | 0 | Candidate sama terkonfirmasi 3 frame | ON |
| APPROACHING | Existing max speed × speed factor | Existing steering PID dan clamp | Kedua buoy dekat → PASSING; loss 6 frame → SEARCHING | OFF, hanya update locked gate |
| PASSING | passing_linear_speed 0.30 | 0 | Near gate hilang/mengecil 8 frame → retired/count | OFF |
| CLEARING_GATE | clear_forward_speed 0.30 | -Kp × wrapped compass error, limit 0.4 | Waktu clearing dengan heading fresh mencapai 3 s | OFF |
| TURNING | turn_forward_speed 0.20 | -sign(compass error) × 0.4 | Heading dalam tolerance 5° | OFF |
| TURN_STABILIZING | 0 | Heading hold terbatas 0.4 | 5 callback heading berturut-turut dalam tolerance; keluar tolerance → TURNING | OFF |
| FINISHED | 0 | 0 | Latched | OFF |

Override kondisi safety: sebelum detection pertama atau detection stale >1 s →
0/0; callback yang menyatakan loss permanen → 0/0, frame berikut dapat menjalankan
SEARCHING normal. Saat maneuver, heading invalid/stale → 0/0 dan clear timer
dipause; turn timeout 30 s → fault latched 0/0, restart guidance setelah investigasi.
Stop bukan dianggap successful pass/turn. Watchdog camera tidak membatalkan maneuver
yang menggunakan heading fresh; acquisition baru menunggu frame berikutnya.

## Tracking, pairing, lock, retirement

ByteTrack dipilih karena tidak memerlukan model ReID/neural network tambahan.
Satu instance YOLO existing `best.pt`, satu `model.track(frame, persist=True,
tracker='bytetrack.yaml')` per callback; hasil yang sama dipakai untuk obstacles,
tracked message, dan overlay. `box.id` diambil jika tersedia, selain itu -1.
Auto-install Ultralytics dinonaktifkan; dependency yang tidak tersedia harus
diperiksa di target. Tidak ada fallback inference kedua.

Image subscriber depth 1 best-effort membatasi backlog. Tracked messages depth 1;
guidance menolak duplicate/out-of-order stamps dan data lebih tua dari 1 s.
Reset/replay stream memerlukan restart detector untuk session/tracker baru.
Satu detector hanya untuk satu aliran kamera kontinu.

`TrackedObstacleArray` berisi ObstacleArray utuh, session UUID per startup,
track_ids sejajar dengan array, image_width dan image_height. Data/ID atomik:
tidak ada pencocokan dua topic berdasarkan callback timing. `/asv/obstacles`
tetap dipublish untuk consumer existing, definisinya tidak berubah.

Candidate valid memerlukan class/confidence/radius/vertical/horizontal constraints
existing, ID valid pada tracked mode, dan optional color order. Assignment global
mengutamakan jumlah opening non-overlap, lalu cost horizontal width/image width +
vertical offset/image height + abs(log(radius ratio)), dengan masing-masing weight.
Span yang berbagi endpoint/overlap/nested tidak dapat terpilih bersama.
R1 G1 R2 G2 menghasilkan R1-G1 dan R2-G2 ketika keduanya valid, termasuk ties.

Selection score = center_weight × (1-|midpoint-center|/center) +
vertical_weight × mean_y/image_height + size_weight × mean_radius/image_width +
confidence_weight × mean_confidence + continuity_weight bila sama dengan pending
candidate. Kandidat pemenang harus konsisten `candidate_confirm_frames` frame.
Scoring tidak dijalankan untuk mengganti lock. Saat debug, scoring state lain
hanya memperbarui visual/debug, tanpa acquisition.

Saat APPROACHING/PASSING, association mensyaratkan session dan dua ID sama serta
batas perubahan posisi/radius. Partial occlusion mempertahankan lock dan command
buoy sebelumnya selama tolerance. Tidak ada fallback otomatis penggantian ID:
setelah loss permanen, lock dilepas secara eksplisit, kemudian candidate baru
harus dikonfirmasi lagi. Session restart mengakhiri lock lama tanpa menambah count.
Legacy `use_object_tracking=false` tersedia secara eksplisit untuk sumber HSV/test;
mode ini tidak menawarkan jaminan identitas tracked.

Retirement dilakukan sebelum maneuver. Dalam suppression window, pemakaian salah
satu constituent ID saja ditolak, termasuk cross-pair dengan buoy baru. Untuk
session/ID baru, midpoint dekat dan rasio total radius yang mirip diblokir juga.
Entry kedaluwarsa menurut waktu; ID tidak diblacklist selamanya. Tidak diperlukan
mempertahankan track sepanjang belokan.

Batas observabilitas: kamera tunggal/bbox tidak membuktikan pasangan fisik atau
jarak meter. Opening yang overlap dalam proyeksi diperlakukan konservatif. Gate
berbeda yang menempati footprint dan ukuran hampir sama dapat tertunda oleh
retirement; gate lama dengan ID baru dan gerakan jauh dari footprint dapat lolos.
Threshold memerlukan replay lapangan dan tuning. PASSING masih memakai bukti
near→disappearance existing, bukan pembuktian posisi kapal; occlusion lama setelah
near masih merupakan keterbatasan yang harus diuji di air.

## Heading convention

Input existing `/mavros/global_position/compass_hdg`, std_msgs/Float64,
SensorDataQoS, derajat compass. MAVROS meneruskan `GLOBAL_POSITION_INT.hdg / 100`;
compass 0 utara, 90 timur, bertambah clockwise. Maka Track A (yaw positif/kiri)
target = start−90°, Track B = start+90°. Heading hold command = −Kp×error.
Contoh B 350°→80°. Clearing capture heading awal fresh; turn start dicapture
ulang setelah clearing selesai. GPS displacement tidak digunakan.

Referensi API yang diperiksa:

- [Ultralytics tracking API](https://docs.ultralytics.com/modes/track/)
- [MAVROS ROS2 global position source](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/global_position.cpp)
- [MAVLink GLOBAL_POSITION_INT](https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT)

Docker existing tidak mem-pin versi Ultralytics. Versi target belum diinspeksi;
API runtime harus diverifikasi di Jetson, tanpa upgrade otomatis.

## Parameter baru

Semua nilai berikut initial dry-test, bukan nilai final lapangan. Normal speed,
PID buoy, dan threshold PASSING existing dipertahankan.

| Parameter | Default | Fungsi |
|---|---|---|
| use_object_tracking | true | Detector memakai track; guidance hanya subscribe tracked topic |
| tracker_type | bytetrack.yaml | Detector; botsort.yaml juga diterima secara eksplisit |
| candidate_confirm_frames | 3 | Konfirmasi sebelum lock |
| tracked_target_lost_frames | 6 | Default mewarisi approaching_lost_frames_tolerance jika tidak dioverride |
| expected_gate_color_order | any | any/red_left/green_left |
| score_center_weight | 1.0 | Posisi di depan kamera |
| score_vertical_weight | 1.0 | Mean y/depth heuristic |
| score_size_weight | 2.0 | Mean radius heuristic |
| score_confidence_weight | 0.2 | Confidence |
| score_continuity_weight | 0.2 | Pending candidate continuity |
| pairing_width_weight | 1.0 | Assignment cost width |
| pairing_vertical_weight | 1.0 | Assignment cost alignment |
| pairing_radius_weight | 1.0 | Assignment cost radius ratio |
| passed_gate_suppression_time_s | 30.0 | Retired identity/history lifetime |
| retired_position_ratio | 0.12 | Midpoint distance / image width |
| retired_radius_ratio | 1.8 | Rasio total radius terhadap retired gate |
| gate_3_clear_time_s | 3.0 | Clearance gate 3; perlu tuning lintasan |
| gate_7_clear_time_s | 3.0 | Clearance gate 7; perlu tuning lintasan |
| clear_forward_speed | 0.30 | Clearance forward command |
| clear_heading_kp | 0.02 | Heading hold command per degree |
| turn_angle_deg | 90.0 | Relatif terhadap turn start |
| turn_forward_speed | 0.20 | Forward ketika turning |
| turn_angular_command | 0.40 | Besar turn dan heading-hold limit |
| turn_heading_tolerance_deg | 5.0 | Arrival tolerance |
| turn_stable_samples | 5 | Consecutive heading callbacks |
| heading_timeout_s | 1.0 | Heading freshness |
| detection_timeout_s | 1.0 | Input watchdog |
| max_detection_age_s | 1.0 | Timestamp age/future bound |
| turn_timeout_s | 30.0 | Latched stop; bukan successful termination |
| debug_pairing | false | Extra candidate computation dan rejection log DEBUG |

`passed_gate_reacquire_block_frames=3` tetap kompatibel sebagai tambahan legacy
block, tetapi bukan protection utama. Association position/radius parameters
existing tetap digunakan; `approaching_lost_frames_tolerance` menjadi default
untuk tracked_target_lost_frames. Detector parameters YAML di perception/config;
launch vision/guidance memuatnya, launch bringup/full mission memakai default node
yang identik. Jika mematikan tracking, atur parameter kedua node secara konsisten.

## Topic/debug

- Baru `/asv/tracked_obstacles`: msgs/TrackedObstacleArray.
- Baru `/asv/gate_debug`: std_msgs/String; state, count, acquisition, detections,
  session/IDs, candidate IDs, locked IDs, score/confirm, losses, retired IDs,
  clear elapsed, start/target/current heading, error, stable count, fault.
- Existing `/asv/camera_debug`: bbox/class/confidence/track ID.
- Existing `/asv/guidance_debug`: candidate lines, locked line/midpoint, state/count.
- Existing `/asv/control_diagnostic/buoy`: actual last published command/state.

DEBUG rejection log memuat confidence, ratio, dx/dy, order, IDs; aktifkan
`debug_pairing:=true --log-level buoy_centering_node:=debug`. Tidak ada console
per-frame pada default. Debug topic tidak menjadi dependency control.

## Test dan status yang benar-benar diperoleh

| Test | Status laptop |
|---|---|
| Pure C++ heading wrap, compass target A/B, heading-hold sign | PASS |
| Pure C++ global cross-pair, shared endpoint, empty input | PASS |
| Pure C++ asymmetric first-gate score | PASS |
| Pure C++ special trigger predicate hanya 3/7 | PASS |
| Python compileall modified scripts/launch/tests, tanpa import dependencies | PASS |
| git diff --check | PASS |
| 1 Persistent gate lock, larger competing gate | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |
| 2 Cross-pair melalui ROS | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |
| 3 Temporary occlusion | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |
| 4 Permanent loss + exact-position impostor ID | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |
| 5 Passed gate + session/ID reset history | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |
| 6 Asymmetric first gate melalui ROS | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |
| 7 Gate 3 clear/turn/stabilize/acquisition, A dan B | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |
| 8 Gate 7 clear/turn/stabilize/acquisition, A dan B | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |
| 9 Wrap 350→80 / 350→260 melalui heading ROS | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |
| 10 Normal gates tidak maneuver, 3/7 tidak retrigger | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |
| 11 Output zero pada stale heading/camera/permanent-loss callback dan FINISHED latched | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |
| 12 Actual YOLO ByteTrack persistence + one missed frame | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment dan BUOY_TRACK_VIDEO |
| Legacy target-locking/full-mission stress regression | NOT EXECUTED — requires target Jetson ROS 2 Jazzy environment |

Test source: guidance_control/test/test_gate_logic.cpp,
simulation/test/test_tracked_gate_lifecycle.py (parameterized A/B), dan
simulation/test/test_tracker_persistence.py. Existing test_buoy_target_locking.py
dan test_full_mission_stress.py disesuaikan untuk legacy untracked input dan
heading maneuver. Synthetic HSV launch eksplisit untracked; tidak menjalankan
YOLO bersamaan. ROS suites serial karena memakai topic yang sama.

Tidak menjalankan colcon build/test/test-result. Compile C++ helper berhasil
dengan g++ C++17 -Wall -Wextra -Werror; compile/link node C++ ROS belum diverifikasi.
Environment mismatch bukan hasil FAIL implementasi. Tidak ada test yang diklaim
PASS berdasarkan inspeksi atau sekadar tersedianya test source.

## SAFE SOFTWARE TEST — Jetson ROS 2 Jazzy

Jalankan dari workspace Jetson. Setiap terminal memakai environment/domain yang
sama. Suite tracked tidak menjalankan MAVROS, bridge, RC node atau motor; output
diremap ke `/test/buoy_cmd`. Legacy stress menjalankan RC node hanya pada domain
software ini tanpa MAVROS/FCU; jangan jalankan pada domain hardware.

```bash
cd ~/dagozilla_ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=86
export YOLO_AUTOINSTALL=false
python3 -c 'import ultralytics; print(ultralytics.__version__); from ultralytics import YOLO; print(callable(YOLO.track))'
test -s src/perception/weights/best.pt
colcon build --symlink-install --packages-up-to perception guidance_control mission simulation bringup
source install/setup.bash
test -s "$(ros2 pkg prefix --share perception)/weights/best.pt"
colcon test --packages-select guidance_control simulation --event-handlers console_direct+
colcon test-result --verbose
```

Actual tracker fixture test terpisah; file harus minimal 20 frame dengan buoy
kontinu terlihat. Tidak menyediakan fixture berarti SKIP/NOT EXECUTED, bukan PASS.

```bash
export BUOY_TRACK_VIDEO=/absolute/path/to/continuous_buoys.mp4
python3 -m pytest -v src/simulation/test/test_tracker_persistence.py
python3 -m pytest -v src/simulation/test/test_tracked_gate_lifecycle.py
# Fokus CLEARING / heading turning tanpa kamera atau motor:
python3 -m pytest -v src/simulation/test/test_tracked_gate_lifecycle.py -k maneuver
```

Camera/detector saja, kemudian guidance saja pada terminal lain:

```bash
ros2 launch perception vision.launch.py use_camera:=true camera_device:=/dev/video0
ros2 run guidance_control buoy_centering_node --ros-args \
  --params-file "$(ros2 pkg prefix --share guidance_control)/config/guidance_params.yaml" \
  -p show_gui:=false -r /asv/cmd_vel:=/test/buoy_cmd
ros2 topic echo /asv/tracked_obstacles
ros2 topic echo /asv/gate_debug
ros2 topic echo /asv/control_diagnostic/buoy
ros2 topic echo /test/buoy_cmd
ros2 topic info /asv/obstacles --verbose
ros2 run rqt_image_view rqt_image_view /asv/camera_debug
ros2 run rqt_image_view rqt_image_view /asv/guidance_debug
```

Periksa satu publisher buoy `/asv/obstacles`, bukan HSV dan YOLO bersamaan.
Untuk manual heading input di domain software, tanpa MAVROS:

```bash
ros2 topic pub -r 10 /mavros/global_position/compass_hdg std_msgs/msg/Float64 '{data: 350.0}'
```

Command heading saja tidak memicu gate count; suite maneuver di atas mengirim
gate 1–10 dan memeriksa CLEARING, acquisition OFF, heading stale stop, target
compass, stabilization, serta tidak retrigger. Jangan memalsukan gate count dari
mission service untuk menguji internal buoy count; itu bukan input controller.

Replay raw camera bag: jalankan detector dengan use_camera=false dan
use_sim_time=true, guidance juga use_sim_time=true/output remap. Jangan hidupkan
live camera/publisher heading lain. Pilih hanya topic yang diperlukan sehingga
command motor dari bag tidak direplay:

```bash
ros2 run perception ball_detector_node.py --ros-args -p use_sim_time:=true \
  -r camera/image_raw:=/camera/image_raw -r obstacles:=/asv/obstacles
ros2 run guidance_control buoy_centering_node --ros-args \
  -p use_sim_time:=true -p show_gui:=false -r /asv/cmd_vel:=/test/buoy_cmd
ros2 bag play /absolute/path/to/bag --clock \
  --topics /camera/image_raw /mavros/global_position/compass_hdg
```

Untuk bag berisi tracked data, hentikan detector, replay
`/asv/tracked_obstacles` dan heading saja. Restart guidance sebelum replay ulang
dari awal; stamp/session replay tidak boleh digabung dengan live stream.

Full mission launch parsing dan optional software smoke:

```bash
ros2 launch bringup asv_full_mission.launch.py --show-args
# Memerlukan kedua kamera/konfigurasi kamera yang sesuai. Tidak menjalankan backend actuator:
ros2 launch bringup asv_full_mission.launch.py use_mavros:=false \
  control_backend:=software enable_gcs_media_bridge:=false
```

`software` tidak cocok dengan kondisi `velocity` maupun `rc_override` existing,
sehingga kedua backend tidak diluncurkan. Gunakan domain 86 yang terisolasi seperti
di atas; tidak ada perubahan behavior mission mux untuk mode ini.

## HARDWARE / MOTOR TEST

Tidak diperlukan untuk validasi pairing/locking/manuver software. Kapal tidak
tersedia; dynamic water test NOT EXECUTED. Setelah kapal tersedia, tuning
clearance 3 s/speed, turn magnitude/tolerance, dan suppression membutuhkan lintasan
nyata. Tidak ada jarak meter yang dijanjikan dari timer dan command speed.
Deployment aktuator tetap memakai prosedur existing yang sudah bekerja.

## File disentuh oleh tugas ini

- guidance_control: src/buoy_centering_node.cpp, include/guidance_control/buoy_centering_node.hpp,
  include/guidance_control/gate_logic.hpp (baru), config/guidance_params.yaml,
  launch/asv_vision_guidance.launch.py, CMakeLists.txt, test/test_gate_logic.cpp (baru).
- perception: scripts/ball_detector_node.py, config/tracking_params.yaml (baru), launch/vision.launch.py.
- msgs: msg/TrackedObstacleArray.msg (baru), CMakeLists.txt.
- simulation: CMakeLists.txt, launch/synthetic_buoy_pipeline.launch.py,
  test/test_buoy_target_locking.py, test/test_full_mission_stress.py,
  test/test_tracked_gate_lifecycle.py (baru), test/test_tracker_persistence.py (baru).
- docs/gate_tracking_audit.md (laporan ini).

Tidak mengedit motor, RC override, channel/PWM, MAVROS connection/configuration,
steering sign/PID implementation, Pixhawk/mixer/output, atau mission mux.
Worktree sudah memiliki perubahan pengguna pada sebagian file tersebut sebelum
tugas dimulai; perubahan existing dipertahankan, bukan hasil tugas ini.
