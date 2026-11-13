# Tutorial Jetson native tanpa Docker

Jetson menjalankan ROS 2 Jazzy dan semua node langsung pada host. Docker hanya
digunakan untuk pengujian di laptop.

```text
/dev/video0 -> perception -> guidance -> /asv/cmd_vel
             -> mavros_bridge -> MAVROS -> /dev/ttyACM0 -> Pixhawk
```

## 1. Syarat sistem

Periksa OS dan arsitektur:

```bash
cat /etc/os-release
dpkg --print-architecture
```

Lanjutkan hanya jika OS adalah Ubuntu 24.04 (Noble). Arsitektur Jetson normalnya
`arm64`. Script setup sengaja berhenti pada versi Ubuntu lain agar repository ROS
yang tidak cocok tidak dipasang.

Pastikan waktu sistem dan koneksi internet benar sebelum instalasi:

```bash
timedatectl status
```

## 2. Instal ROS 2 Jazzy dan dependency

Terminal 1:

```bash
cd ~/dagozilla_ws
bash scripts/setup_env.sh
```

Script memasang ROS 2 Jazzy, MAVROS, OpenCV C++/Python, cv_bridge, colcon, rosdep,
tool kamera, serta dataset GeographicLib langsung pada Jetson.

Inisialisasi rosdep bila belum pernah dilakukan:

```bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
```

Pasang dependency repository:

```bash
cd ~/dagozilla_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 3. Izin kamera dan serial

```bash
sudo usermod -aG video,dialout "$USER"
```

Logout/login atau reboot, kemudian periksa:

```bash
groups
ls -l /dev/video0
ls -l /dev/ttyACM0
```

Jika Pixhawk muncul sebagai `/dev/ttyUSB0`, gunakan path tersebut pada `fcu_url`.
Jangan membuka port serial yang sama langsung dari QGroundControl dan MAVROS secara
bersamaan.

## 4. Build workspace

```bash
cd ~/dagozilla_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash
```

Periksa package:

```bash
ros2 pkg list | grep -E '^(perception|guidance_control|mavros_bridge|simulation)$'
```

Opsional, source otomatis untuk terminal baru:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/dagozilla_ws/install/setup.bash' >> ~/.bashrc
```

Setelah source berubah, ulangi `colcon build` dan `source install/setup.bash`.

## 5. Smoke test tanpa hardware

Terminal 1:

```bash
cd ~/dagozilla_ws
source install/setup.bash
ros2 launch simulation synthetic_buoy_pipeline.launch.py
```

Terminal 2:

```bash
source ~/dagozilla_ws/install/setup.bash
ros2 topic echo /asv/buoy_gate_count std_msgs/msg/UInt8
```

Tes lulus jika hitungan mencapai 10 dan Terminal 1 menampilkan:

```text
PASSING -> FINISHED
Gate 10/10 passed - FINISHED - STOP
```

Tekan `Ctrl+C` setelah tes.

## 6. Uji kamera tanpa Pixhawk

Terminal 1:

```bash
source ~/dagozilla_ws/install/setup.bash
ros2 launch guidance_control asv_vision_guidance.launch.py \
  camera_device:=/dev/video0 \
  use_camera:=true \
  use_mavros:=false \
  show_gui:=false
```

Terminal 2, periksa frame dan deteksi:

```bash
source ~/dagozilla_ws/install/setup.bash
ros2 topic hz /camera/image_raw
```

```bash
ros2 topic echo /asv/obstacles msgs/msg/ObstacleArray
```

Terminal 3, periksa command:

```bash
source ~/dagozilla_ws/install/setup.bash
ros2 topic echo /asv/cmd_vel geometry_msgs/msg/Twist
```

Tunjukkan objek merah dan hijau. Pastikan class `1` dan `2` muncul, arah
`angular.z` sesuai posisi titik tengah, dan gate count tidak bertambah palsu.

## 7. Hubungkan Pixhawk tanpa arm

Pengujian pertama dilakukan dengan propeller dilepas.

Terminal 1:

```bash
source ~/dagozilla_ws/install/setup.bash
ros2 launch guidance_control asv_vision_guidance.launch.py \
  camera_device:=/dev/video0 \
  use_camera:=true \
  use_mavros:=true \
  fcu_url:=/dev/ttyACM0:115200 \
  show_gui:=false
```

Terminal 2:

```bash
source ~/dagozilla_ws/install/setup.bash
ros2 topic echo /mavros/state
```

Harus terlihat `connected: true`. Pastikan bridge meneruskan command:

```bash
ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped \
  geometry_msgs/msg/Twist
```

## 8. Verifikasi frame velocity

Controller kamera menghasilkan perintah relatif terhadap badan kapal. Cari node
plugin velocity:

```bash
ros2 node list | grep setpoint_velocity
```

Kemudian gunakan nama node yang ditemukan untuk memeriksa `mav_frame`, contoh:

```bash
ros2 param get /mavros/setpoint_velocity mav_frame
```

Jangan melakukan arm sebelum makna `linear.x` dan tanda `angular.z` diverifikasi
dengan ArduRover. Untuk controller kamera, gerak yang dibutuhkan adalah body-relative;
default `LOCAL_NED` tidak boleh diasumsikan sama dengan maju relatif kapal.

## 9. Uji mode dan motor tanpa propeller

Pastikan RC takeover, emergency stop, dan failsafe sudah diuji. Ubah ke GUIDED:

```bash
ros2 service call /asv/set_mode msgs/srv/SetMode "{mode: 'GUIDED'}"
```

Periksa `/mavros/state`, lalu arm hanya untuk uji darat yang diamankan:

```bash
ros2 service call /mavros/cmd/arming \
  mavros_msgs/srv/CommandBool "{value: true}"
```

Disarm:

```bash
ros2 service call /mavros/cmd/arming \
  mavros_msgs/srv/CommandBool "{value: false}"
```

## 10. Urutan uji di air

1. Gunakan tether, area kosong, dan operator RC.
2. Nyalakan Jetson dan Pixhawk, tetapi jangan arm.
3. Jalankan launch lengkap.
4. Pastikan kamera, obstacle, command, dan MAVROS normal.
5. Posisikan kapal menghadap gate pertama.
6. Pastikan frame velocity dan arah kemudi sudah benar.
7. Aktifkan GUIDED lalu arm.
8. Pantau `/mavros/state`, `/asv/cmd_vel`, dan `/asv/buoy_gate_count`.
9. Disarm segera jika deteksi hilang, arah salah, koneksi putus, atau hitungan palsu.

Peringatan: controller belum mempunyai tombol START/PAUSE. Saat node aktif, state
`SEARCHING` menerbitkan maju `0.15 m/s`. Kapal dapat mulai bergerak segera setelah
Pixhawk berada pada kondisi ARMED + GUIDED. Jangan arm sebelum semua operator siap.

## 11. Menghentikan

Disarm terlebih dahulu, tekan `Ctrl+C` pada launch, lalu pastikan:

```bash
ros2 topic echo /mavros/state --once
```

Nilai `armed` harus `false` sebelum mematikan Jetson atau Pixhawk.
