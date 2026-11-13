# DOKUMEN KEBUTUHAN PRODUK (PRD) — TODO #1
## Peningkatan Sistem Navigasi Otonom: Segmentasi Batas Delaunay & Perencanaan Trajektori Metrik 3D Berbasis Visi (Dagozilla ASV)

* **Status Dokumen:** *Production Technical Specification (Enhanced Delta Baseline)*
* **Prinsip Eliminasi:** Komponen yang **sudah ada dan berfungsi** pada repositori eksisting (manajer misi, jembatan MAVROS dasar, *wrapper* penangkap citra dasar, dan *cmd_vel mux*) **tidak dicantumkan ulang**. Dokumen ini secara eksklusif mendefinisikan **peningkatan teknis, modul baru, dan formulasi matematis** yang wajib dicapai.
* **Referensi Arsitektur Visual:** Segmentasi batas (garis jaring putih) dan perencanaan trajektori garis tengah (garis hijau) mengimplementasikan paradigma visual berikut:

```
               [Kiri: Pelampung Merah]             [Kanan: Pelampung Hijau]
                      R_k  ●─────────────────────────●  G_k
                           │ ╲                     ╱ │
      Garis Batas          │   ╲  Jaring Delaunay╱   │  Garis Batas
      Kiri (Putih)         │     ╲   (Putih)   ╱     │  Kanan (Putih)
                           │       ╲         ╱       │
                           │         ╲  │  ╱         │
                           │          ● P_mid        │
                           │            │            │
                           │            │            │
                     R_k-1 ●────────────┼────────────● G_k-1
                                        │  Garis Trajektori Hijau
                                        ▼  (Centerline Spline C^2)
                                      [ASV]
```

---

## 1. Fondasi Teoretis & Sintesis Literatur Ilmiah

Peningkatan ini mengintegrasikan 4 pilar teoretis utama dari literatur sistem otonom:
1. **Perencanaan Jalur Graf Delaunay Otonom (*Alvarez et al., 2022; Kabzan et al., 2020*)**: Mengadopsi arsitektur pemenang kompetisi *Formula Student Driverless* (AMZ Racing ETH Zurich & TU Munich) yang memetakan kerucut/pelampung ke dalam graf *Delaunay Triangulation*, memfilter sisi non-koridor (*topological edge pruning*), dan mengekstraksi koridor gerbang konveks.
2. **Dynamic Inverse Perspective Mapping (IPM) & Virtual Gimbal Stabilization**: Menghilangkan distorsi perspektif akibat gelombang air melalui *dynamic homography* yang dikompensasi data orientasi IMU berkecepatan tinggi ($\ge 100\text{ Hz}$).
3. **Peredaman Pantulan Air Berbasis Hukum Brewster & Proyeksi Horizon Analitik**: Memanfaatkan prinsip polarisasi cahaya permukaan air serta pemotongan batas horizon inersial untuk mencapai nol deteksi semu (*ghost detection*).
4. **Panduan Hidrodinamika Integral Line-of-Sight (ILOS) (*Fossen, Pettersen, & Galeazzi, 2014*)**: Mengatasi hanyutan samping (*sideslip drift angle* $\beta$) kapal pada kecepatan tinggi ($Fn > 1.0$) dengan kompensasi integral adaptif anti-windup.

---

## 2. Ruang Lingkup Peningkatan (*Scope of Enhancements / Delta*)

Peningkatan ini menggantikan paradigma **Image-Based Visual Servoing (IBVS) 2D pada ruang piksel** yang ada saat ini dengan **Metric 3D Delaunay Midline Planning**:

| Area Modul | Status Codebase Saat Ini | Target Peningkatan Spesifik (*The Delta*) |
| :--- | :--- | :--- |
| **Inferensi Model** | PyTorch CPU murni (`device: "cpu"`), latensi $80 - 160\text{ ms}$, hanya 2 kelas. | **TensorRT FP16 C++ Engine**, latensi $\le 8\text{ ms}$, deteksi 4 kelas (`buoy_red`, `buoy_green`, `buoy_yellow`, `buoy_white`). |
| **Ruang Kerja Persepsi** | Piksel 2D $[u, v]$ tanpa estimasi jarak metrik. | **Dynamic IPM 3D**, proyeksi sinar metrik ke bidang air $Z=0$ dengan koreksi dinamis *pitch-roll* IMU real-time. |
| **Filter & Tracking** | ByteTrack di ruang piksel (mudah *lost track* saat ombak). | **Filter Horizon Dinamis + Refleksi Air** dan **3D EKF Tracking (Constant Velocity)** pada koordinat fisik kapal `base_link`. |
| **Segmentasi Lintasan** | Penjodohan gerbang piksel terdekat (*heuristic matching*). | **Delaunay Triangulation Graph (Garis Putih)** dengan filter anti-diagonal dan repulsi rintangan kuning. |
| **Pembangkit Trajektori** | Kontrol diskrit titik tengah piksel (*centering error*). | **Parametric Natural Cubic Spline $C^2$ (Garis Hijau)** berkesinambungan kurvatur ($\kappa \le \kappa_{max}$) dengan *Gate Normal Alignment*. |
| **Algoritma Kontrol** | P-Controller lateral 2D pada rate 10-20 Hz. | **Integral Line-of-Sight (ILOS) Guidance pada 50 Hz**, meniadakan *steady-state drift* arus air pada $3.5\text{ m/s}$. |
| **Manuver Tikungan (Gate 3 $\to$ 4)** | Pemotongan diagonal (*corner cutting*), skipping gerbang, dan timer *open-loop*. | **Protokol Khusus Tikungan Tajam**: *Gate Normal Spline Alignment*, *Topological Anti-Skip*, *3D Spatial Memory*, dan *Proactive Lead-in Turn*. |
| **Verifikasi Offline Tanpa Wahana** | Observasi pasif rosbag tanpa kepastian respons kemudi. | **Sistem Visualisasi AR Reprojection di Rosbag**: Proyeksi kurva trajektori hijau ($C^2$ Spline) langsung di atas bidang air + bujur sangkar target dinamis (*destination bracket*) pada gerbang aktif untuk validasi visual playback rosbag. |
| **Arsitektur Eksekusi** | Multi-proses ROS 2 terpisah (overhead DDS $\approx 15\text{ ms}$). | **ROS 2 Composable Nodes (`rclcpp_components`)** dengan *Zero-Copy Pointer IPC* (total latensi $\le 20\text{ ms}$). |

---

## 3. Kebutuhan Fungsional Peningkatan (Enhanced Functional Requirements)

### ENH-FR-1: Akselerasi Inferensi TensorRT C++ & Ekstraksi Titik Kontak Air

* **ENH-FR-1.1 (Ekspansi 4-Kelas KKI):** Mengganti model 2-kelas eksisting dengan model *lightweight deep learning* (YOLOv8n/YOLOv11n) 4-kelas:
  1. `buoy_red` (Batas Kiri / Port Hand Marker).
  2. `buoy_green` (Batas Kanan / Starboard Hand Marker).
  3. `buoy_yellow` (Rintangan Tengah Lintasan / Obstacle).
  4. `buoy_white` (Target Bola Penalti).
* **ENH-FR-1.2 (Pipeline Inferensi TensorRT C++ Native):**
  - Mengeliminasi *overhead* eksekusi Python CPU (`device: "cpu"`). Inferensi wajib menggunakan C++ TensorRT 8.6+ Runtime API langsung di memori GPU Jetson (FP16 Engine).
  - Pemuatan bobot engine menggunakan *Unified Memory / Zero-Copy CUDA Host Pointer* untuk menghilangkan salinan memori CPU $\leftrightarrow$ GPU.
* **ENH-FR-1.3 (Sub-Pixel Water-Contact Base Extraction):**
  Detektor mengekstrak koordinat dasar pelampung yang bersentuhan langsung dengan air:
  $$u_{base} = \frac{u_{min} + u_{max}}{2}, \quad v_{base} = v_{max}$$
* **Acceptance Criteria:**
  - mAP@0.5 $\ge 88\%$ pada resolusi $640 \times 640$ piksel.
  - Latensi inferensi *forward pass* C++ $\le 8\text{ ms}$ pada Jetson Orin Nano dan $\le 5\text{ ms}$ pada Orin NX.

---

### ENH-FR-2: Dynamic Inverse Perspective Mapping (IPM) & Stabilisasi Virtual

Modul ini memproyeksikan piksel $(u_{base}, v_{base})$ ke koordinat fisik metrik wahana `base_link` $(X, Y, Z=0)$ dengan kompensasi dinamika sikap kapal secara *real-time*.

```
        Kamera Miring (Pitch nominal θ0)
             ╲
              ╲  Sinar Proyeksi r_c
               ╲
  ──────────────╳ Piksel Dasar (u_base, v_base)
                 ╲
  Bidang Air      ╲─────────────● Pelampung (Z_base = 0)
 (Z_base = 0)                   Jarak X = x_offset + λ d_wx
```

#### ENH-FR-2.1: Sinkronisasi Waktu & Ekstrapolasi Sikap IMU
Untuk mencegah desinkronisasi antara paparan kamera dan pembacaan IMU, orientasi quaternion diinterpolasi ke titik tengah waktu eksposur kamera ($t_{mid} = t_{trigger} + \frac{1}{2} t_{exposure}$) menggunakan kecepatan sudut $\boldsymbol{\omega}_b$:
$$R_{b/n}(t_{mid}) \approx R_{b/n}(t_{imu}) \cdot \exp\left(\boldsymbol{\omega}_b^\wedge \cdot (t_{mid} - t_{imu})\right)$$
di mana toleransi jitter sinkronisasi wajib $\Delta t_{jitter} \le 1.0\text{ ms}$.

#### ENH-FR-2.2: Formulasi Dynamic Homography & Virtual Gimbal
1. Sinar terproyeksi pada frame optik kamera (REP-104: $X$ kanan, $Y$ bawah, $Z$ maju):
   $$r_c = K^{-1} \begin{bmatrix} u_{base} \\ v_{base} \\ 1 \end{bmatrix}$$
2. Matriks rotasi dari kamera optik ke frame wahana `base_link` (REP-103: $X$ maju, $Y$ kiri, $Z$ atas) dengan sudut tunduk nominal $\theta_0$:
   $$R_c^b = \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix} \begin{bmatrix} \cos\theta_0 & 0 & \sin\theta_0 \\ 0 & 1 & 0 \\ -\sin\theta_0 & 0 & \cos\theta_0 \end{bmatrix}$$
3. **Koreksi Sikap Inersial Terhadap Bidang Air Datar ($Z_{base} = 0$):**
   $$R_{cam}^{inertial}(\phi, \theta) = R_{IMU}(\phi, \theta) \cdot R_c^b$$
   Vektor arah sinar inersial:
   $$d_w = R_{cam}^{inertial} \cdot r_c = \begin{bmatrix} d_{wx} \\ d_{wy} \\ d_{wz} \end{bmatrix}$$
4. **Perpotongan Garis Sinar Terhadap Permukaan Air:**
   Dengan elevasi kamera terhadap air tenang $h_{cam} > 0$ dan estimasi gelombang *heave* $z_{heave}(t)$:
   $$h_{eff}(t) = h_{cam} + z_{heave}(t)$$
   Parameter skala jarak sinar diperoleh dari perpotongan bidang $Z = 0$:
   $$\lambda = \frac{-h_{eff}(t)}{d_{wz}} \quad (\text{valid jika } d_{wz} < -10^{-3})$$
5. Koordinat posisi fisik pelampung dalam frame `base_link`:
   $$\begin{bmatrix} X_{base} \\ Y_{base} \\ Z_{base} \end{bmatrix} = \begin{bmatrix} x_{offset} + \lambda d_{wx} \\ y_{offset} + \lambda d_{wy} \\ 0 \end{bmatrix}$$

* **Acceptance Criteria:**
  - Error estimasi jarak metrik $\le 0.30\text{ m}$ pada rentang $1.0 - 5.0\text{ m}$, dan $\le 0.80\text{ m}$ pada rentang $5.0 - 10.0\text{ m}$.
  - Posisi marker metrik tidak bergeser $> 0.20\text{ m}$ saat kapal mengalami dinamika ombak buatan $\pm 10^\circ$.

---

### ENH-FR-3: Filter Outlier Pantulan Air & 3D EKF Tracking

#### ENH-FR-3.1: Proyeksi Garis Horizon Inersial Analitik (*Analytical Horizon Cutoff*)
Batas pemisah antara air dan langit dihitung langsung dari orientasi inersial sensor:
Vektor normal bidang horizon pada frame kamera adalah:
$$\mathbf{l}_\infty = K^{-\top} (R_{cam}^{inertial})^\top \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} = \begin{bmatrix} a_h \\ b_h \\ c_h \end{bmatrix}$$
Persamaan garis horizon pada ruang piksel:
$$v_{horizon}(u) = -\frac{a_h u + c_h}{b_h}$$
Setiap deteksi dengan $v_{base} < v_{horizon}(u)$ langsung dibuang dari pipa pemrosesan karena berada di wilayah udara/langit.

#### ENH-FR-3.2: Penekanan Refleksi Permukaan Air Berdasarkan Teori Brewster & Simetri
1. **Aturan Geometris Simetri Refleksi:** Deteksi sekunder yang berada di bawah pelampung terkonfirmasi dibuang jika memenuhi kriteria pantulan cermin air:
   $$\left| \frac{u_{base, 1} - u_{base, 2}}{w_1} \right| < 0.20 \quad \text{dan} \quad v_{min, 2} \ge v_{max, 1} - \epsilon$$
2. **Filter Polarisasi Hardware:** Lensa kamera wajib dipasangi filter *Circular Polarizer* (CPL) yang disetel pada sudut Brewster air ($\theta_B \approx 53.1^\circ$) untuk menekan transmisi cahaya terpolarisasi-s (*s-polarized glint*) hingga $\ge 85\%$.

#### ENH-FR-3.3: 3D Extended Kalman Filter (Constant Velocity Model pada `base_link`)
1. State vector untuk setiap pelampung aktif ke-$i$:
   $$\mathbf{x}_i = \begin{bmatrix} X & Y & \dot{X} & \dot{Y} \end{bmatrix}^\top$$
2. Model transisi keadaan dengan kompensasi pergerakan kapal (*ego-motion compensation*):
   $$\mathbf{x}_{k|k-1} = F \mathbf{x}_{k-1} + \begin{bmatrix} -u \Delta t + \frac{1}{2} r Y \Delta t \\ -r X \Delta t \\ 0 \\ 0 \end{bmatrix}$$
   di mana $u$ adalah kecepatan linier maju dan $r = \dot{\psi}$ adalah kecepatan yaw dari odometri kapal.
3. Asosiasi data spasial menggunakan metrik jarak *Mahalanobis Distance*:
   $$d_M^2 = (\mathbf{z} - H\mathbf{x})^\top S^{-1} (\mathbf{z} - H\mathbf{x}) \le 9.21 \quad (\chi^2\text{ threshold, } p=0.01)$$
4. Objek dipromosikan berstatus `CONFIRMED` dan dikirim ke planner jika terasosiasi minimal **3 frame berturut-turut** ($3/3$).

#### ENH-FR-3.4: Memori Spasial 3D EKF Saat Oklusi Bidang Pandang (*FOV Occlusion Persistence*)
Khusus pada manuver tikungan tajam (seperti belokan Gate 3 ke Gate 4):
1. Ketika pelampung Gate 4 terdeteksi di tepi kanan atas frame, koordinat metrik 3D-nya langsung terkunci ke dalam EKF lokal.
2. Ketika haluan kapal mulai berbelok dan Gate 4 sempat menghilang sesaat dari frame kamera (oklusi sudut kamera), EKF **mempertahankan posisi Gate 4 dalam memori lokal** melalui tahap prediksi pergerakan kapal (*dead reckoning step* via gyroskop IMU $r$ dan kecepatan $u$).
3. Pelampung Gate 4 **dilarang dihapus** dari memori selama durasi oklusi $t_{lost} \le 1.5\text{ detik}$, dan hanya di-retire jika secara fisik koordinat relatifnya telah berada di belakang moncong kapal ($X_{rel} < -0.3\text{ m}$).

* **Acceptance Criteria:**
  - *False Positive Rate* (FPR) akibat buih air dan pantulan matahari $\le 0.5\%$.
  - Nol *ghost-buoy* yang bertahan $\ge 3$ frame yang lolos ke modul planner.
  - Posisi target gerbang yang hilang dari pandangan kamera saat berbelok tetap presisi dalam memori EKF ($\text{drift} \le 0.25\text{ m}$ selama 1.5 detik).

---

### ENH-FR-4: Segmentasi Batas Delaunay & Penghindaran Rintangan (Sesuai Visual Gambar)

Modul ini merekonstruksi jaring topologi lintasan (garis jaring putih pada gambar referensi) dan menentukan titik tengah koridor yang aman.

```
       R_k+1 ●───────────────────────────────────● G_k+1
             │  ╲                             ╱  │
             │    ╲       DIAGONAL          ╱    │
   Batas     │      ╲   (WAJIB DIBUANG)   ╱      │  Batas
   Kiri      │        ╲                 ╱        │  Kanan
  (Putih)    │          ╲             ╱          │ (Putih)
             │            ╲         ╱            │
             │              ╲  │  ╱              │
             │                 ● P_mid (Repulsed)│
             │               ╱   ╲               │
             │             ╱       ╲ [Kuning]    │
             │           ╱       ▲   ● Obstacle  │
             │         ╱         │               │
         R_k ●───────────────────┼───────────────● G_k
                                 │ Garis Hijau
                                [ASV]
```

#### ENH-FR-4.1: Konstruksi Graf Delaunay 2D (Garis Putih)
1. Kumpulkan seluruh koordinat pelampung terkonfirmasi di depan kapal ($X > 0.5\text{ m}, \sqrt{X^2+Y^2} \le 15\text{ m}$).
2. Bangun jaring segitiga 2D (*Delaunay Triangulation Graph*) $\mathcal{DT}(\mathcal{P})$ menggunakan algoritma *sweep-hull* C++ (`delaunator-cpp`).

#### ENH-FR-4.2: Klasifikasi Sisi & Eliminasi Perangkap Diagonal (*Anti-Diagonal Filter*)
Setiap sisi $e_{ij} = (P_i, P_j)$ diklasifikasikan dengan aturan ketat:
1. **Boundary Edges (Garis Tepi Putih):** Menghubungkan warna sejenis:
   - Merah-Merah $\to$ Batas terluar sisi kiri koridor (*Port boundary*).
   - Hijau-Hijau $\to$ Batas terluar sisi kanan koridor (*Starboard boundary*).
2. **Cross-Track Edges (Garis Melintang Gerbang):** Menghubungkan pelampung berbeda warna (Merah-Hijau):
   - **Filter Lebar Regulasi KKI:** $1.5\text{ m} \le \|P_i - P_j\| \le 6.0\text{ m}$.
   - **Filter Orientasi Anti-Diagonal (*The Anti-Diagonal Angle Filter*):** Untuk membuang garis diagonal antar-gerbang ($R_k - G_{k+1}$ yang berjarak tipikal $\approx 4.24\text{ m}$ pada gambar referensi), sudut vektor garis gerbang $\psi_{edge}$ harus tegak lurus terhadap arah rambat lintasan lokal $\psi_{corridor}$:
     $$\left| |\psi_{edge} - \psi_{corridor}| - 90^\circ \right| \le 25^\circ$$
   - **Invarian Bipartite Monogami:** Satu pelampung merah hanya boleh dipasangkan dengan tepat satu pelampung hijau terdekat yang lolos uji sudut ortogonalitas.

#### ENH-FR-4.3: Invarian Planar Graf Delaunay Mencegah Skipping Gerbang
Sesuai teorema geometri komputasi, segitiga graf Delaunay bersifat planar dan **sisinya dilarang saling berpotongan (*non-crossing planar edges*)**:
1. Karena sepasang pelampung Gate 4 ($R_4, G_4$) terletak di antara Gate 3 ($R_3, G_3$) dan Gate 5 ($R_5, G_5$), jaring Delaunay secara topologis **wajib memotong Gate 4 terlebih dahulu** sebelum dapat menghubungkan sisi ke Gate 5.
2. Planner menerapkan aturan sekuensial ketat: Gerbang ke-$(k+1)$ dilarang dijadikan target utama sebelum gerbang ke-$k$ dinyatakan *passed* secara fisik ($X_{rel, k} < 0$). Ini secara mutlak mengeliminasi anomali kapal melompati Gate 4 menuju Gate 5.

#### ENH-FR-4.4: Medan Tolakan Rintangan Kuning (`buoy_yellow`)
1. Titik tengah gerbang nominal dihitung dari sisi melintang valid:
   $$P_{mid} = \frac{P_{red} + P_{green}}{2}$$
2. Jika terdeteksi pelampung rintangan kuning $P_{yellow}$ dalam radius bahaya lateral $r \le 1.2\text{ m}$:
   $$P_{mid}^{repulsed} = P_{mid} + \text{sgn}\left((P_{mid} - P_{yellow}) \cdot \hat{n}_{gate}\right) \cdot d_{safe} \cdot \hat{n}_{gate}$$
   dengan jarak toleransi aman $d_{safe} = 1.0\text{ meter}$.
3. **Fallback Mode $N=2$:** Jika hanya terdeteksi sepasang pelampung (1 Merah, 1 Hijau), proses graf Delaunay dilewati dan titik tengah langsung dihitung.

* **Acceptance Criteria:**
  - Waktu komputasi graf Delaunay dan penyaringan gerbang $\le 3\text{ ms}$ untuk hingga 30 titik pelampung.
  - Nol garis diagonal palsu yang diteruskan ke generator kurva trajektori.
  - Nol kasus pelompatan gerbang (*gate skipping*) pada koridor belokan.

---

### ENH-FR-5: Pembangkit Trajektori Hijau (*Green Centerline Spline*)

Modul ini menghasilkan kurva trajektori mulus berkesinambungan kurvatur (garis hijau pada gambar referensi).

* **ENH-FR-5.1: Parametric Natural Cubic Spline ($C^2$ Continuity)**
  1. Himpunan titik tengah $\{P_0, P_{mid, 1}, P_{mid, 2}, \dots, P_{mid, M}\}$ (dimulai dari posisi haluan kapal $P_0 = [0, 0]^\top$) diurutkan secara sekuensial menaik berdasarkan koordinat $X$.
  2. Pembangkitan kurva parametrik terhadap panjang busur kumulatif $s$:
     $$X(s) = S_x(s), \quad Y(s) = S_y(s), \quad s \in [0, S_{total}]$$
  3. Menjamin **kontinuitas geometris $C^2$** (posisi, heading tangensial $\psi(s)$, dan kurvatur $\kappa(s)$ kontinu di setiap simpul):
     $$\lim_{s \to s_k^-} \kappa(s) = \lim_{s \to s_k^+} \kappa(s)$$
     *(Menghilangkan goncangan step response pada servo kemudi).*

#### ENH-FR-5.2: Pembatasan Vektor Normal Gerbang (*Gate Normal Vector Alignment*) & Pola Bujur Sangkar (*Orthogonal L-Turn*)
Untuk mengatasi **gerakan memotong diagonal (*diagonal corner-cutting*)** yang dialami sistem lama saat beralih dari Gate 3 $\to$ Gate 4:

```
PERBANDINGAN TRAJEKTORI:
[Sistem Lama: Diagonal Naif - GAGAL]     [Sistem Baru: Pola Bujur Sangkar Dinamis - SUKSES]
              Gate 4                                      Gate 4
              ●     ●                                     ●     ●
               \                                         ┌───────  (3. Masuk Tegak Lurus)
                \  <-- Memotong serong                  │
                 \     & meleset/menyerempet!          ╭╯          (2. Sudut Bujur Sangkar R=2.2m)
      ●     ●     \                              ●     ●│
      Gate 3       ASV                           Gate 3 │          (1. Lurus Keluar Gerbang)
                                                       [ASV]
```

1. **Akar Masalah Diagonal:** Algoritma lama hanya menghubungkan titik koordinat kapal ke titik tengah Gate 4 dengan garis lurus Euclidean miring tanpa pembatas sudut masuk (*tangent constraint*).
2. **Formulasi Vektor Melintang Gerbang:** Untuk setiap gerbang $k$, hitung vektor celah:
   $$\vec{v}_{gate, k} = P_{green, k} - P_{red, k}$$
3. **Vektor Normal Satuan Gerbang ($\hat{n}_{gate, k}$):** Vektor arah masuk yang tegak lurus sempurna terhadap celah 2 meter:
   $$\hat{n}_{gate, k} = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix} \frac{\vec{v}_{gate, k}}{\|\vec{v}_{gate, k}\|}$$
4. **Syarat Batas Tangensial Spline:** Arah tangensial kurva spline pada setiap titik tengah gerbang $P_{mid, k}$ diwajibkan sejajar dengan vektor normal gerbang:
   $$\left. \frac{d \mathbf{r}(s)}{ds} \right|_{s = s_k} = \hat{n}_{gate, k}$$
5. **Dampak Kinematis & Rekayasa Lintasan Bujur Sangkar:**
   - Secara matematis, kondisi batas normal ini melarang garis miring/diagonal.
   - Garis hijau dipaksa membentuk pola **bujur sangkar melengkung (*filleted orthogonal L-turn*)**:
     - *Segmen 1:* Maju lurus keluar Gate 3 sejauh margin aman ($d \approx 1.0 - 1.5\text{ m}$).
     - *Segmen 2:* Membentuk sudut bujur sangkar melengkung dengan radius belok fisik terukur ($R \approx 2.2\text{ m}$).
     - *Segmen 3:* Menembus Gate 4 tepat di sumbu tengah tegak lurus celah gerbang ($\Delta \psi_{entry} \le 5^\circ$).

* **ENH-FR-5.3: Pembatasan Kurvatur Maksimum Fisik Kapal**
  Kurvatur analitik $\kappa(s)$ dibatasi oleh kemampuan radius belok minimum kapal ($R_{min} \approx 1.8\text{ m}$):
  $$\kappa(s) = \frac{|\dot{X}\ddot{Y} - \dot{Y}\ddot{X}|}{(\dot{X}^2 + \dot{Y}^2)^{3/2}} \le \kappa_{max} = \frac{1}{R_{min}} \approx 0.55\text{ m}^{-1}$$
  Jika $\kappa(s) > \kappa_{max}$, terapkan relaksasi optimasi *gradient descent smoothing* pada titik kontrol lokal hingga kurvatur berada dalam batas aman.
* **ENH-FR-5.4: Format Publikasi ROS 2**
  Menerbitkan kurva trajektori ke topik `/planning/centerline_path` bertipe `nav_msgs/msg/Path` dengan resolusi sampling antar-titik $\Delta s = 0.10\text{ m}$.
* **Acceptance Criteria:**
  - Kurva trajektori terverifikasi kontinu $C^2$ tanpa diskontinuitas sudut.
  - Sudut masuk kapal pada setiap gerbang tidak melenceng $> 15^\circ$ dari vektor normal gerbang.
  - Waktu komputasi pembangkitan spline $\le 2\text{ ms}$.

---

### ENH-FR-6: Pengendali Hidrodinamika Integral Line-of-Sight (ILOS)

Menggantikan pengendali lateral 2D sederhana untuk mengatasi dinamika *planing hull* pada kecepatan tinggi $3.5\text{ m/s}$ ($Fn \approx 1.02$).

```
                      Haluan Kapal (Heading ψ)
                             ↗
                            ↗
                           ●───────► Arah Gerak Sebenarnya (Course χ = ψ + β)
                          ASV         (Tereduksi oleh Integral ILOS)
```

#### ENH-FR-6.1: Ekstraksi Titik Pandang Adaptif Kecepatan
Titik target (*look-ahead point*) $P_{los}(s)$ diekstrak dari kurva hijau pada jarak pandang dinamis $\Delta(U)$:
$$\Delta(U) = \Delta_{min} + k_\Delta \cdot U \quad (\Delta_{min} = 2.0\text{ m}, \, k_\Delta = 0.7\text{ s} \implies \Delta \approx 4.45\text{ m pada } 3.5\text{ m/s})$$

#### ENH-FR-6.2: Hukum Kendali ILOS Fossen (Anti-Windup Formulation)
Untuk mengeliminasi kesalahan hanyutan lateral tunak (*steady-state cross-track drift* $e_y^{ss} = \Delta \tan\beta$) akibat arus air:
$$\psi_d = \psi_p + \arctan\left(\frac{-(e_y + \sigma y_{int})}{\Delta(U)}\right)$$
dengan dinamika pembaruan integral anti-windup:
$$\dot{y}_{int} = \frac{\Delta(U) \cdot e_y}{(\Delta(U) + \sigma y_{int})^2 + e_y^2}$$
di mana $e_y$ adalah deviasi jarak lateral wahana terhadap kurva spline hijau, $\psi_p$ adalah arah tangensial jalur, dan $\sigma > 0$ adalah gain integral adaptif ($0.1 - 0.3\text{ s}^{-1}$).

#### ENH-FR-6.3: Pengendali Kemudi Rudder & Limitasi Slew Rate
1. Perintah sudut kemudi servo rudder ($\delta_r$) dihitung melalui pengendali PD dengan *feedforward* kurvatur:
   $$\delta_r = K_p (\psi_d - \psi) + K_d (\dot{\psi}_d - \dot{\psi}) + \arctan(L_{wheelbase} \cdot \kappa)$$
2. Pembatasan laju gerak kemudi (*Slew Rate Limiting*):
   $$|\dot{\delta}_r| \le 60^\circ/\text{detik}$$
   guna mencegah kavitasi dan pelepasan aliran fluida (*hydrodynamic stall*) pada daun kemudi.
3. Frekuensi pengiriman perintah aktuator ke MAVROS wajib ditingkatkan dari 10 Hz ke **50 Hz** secara sinkron.

#### ENH-FR-6.4: Invarian Pelacakan Jalur Tunggal (*Strict Single-Reference Path Following Invariant*)
1. **Otoritas Tunggal Navigasi Fisik:**
   Pergerakan fisik wahana (sudut servo rudder dan persentase throttle) secara eksklusif dan 100% **HANYA digerakkan untuk melacak garis trajektori hijau (`/planning/centerline_path`)**.
2. **Ketiadaan *Ad-Hoc Steering*:**
   Tidak ada logika navigasi yang mengizinkan kapal membidik pelampung individual secara lepas/bebas. Kemudi hanya merespons *cross-track error* ($e_y \to 0$) dan *heading error* ($e_\psi \to 0$) relatif terhadap garis trajektori hijau.
3. **Kepatuhan Terhadap Dinamika Pola Siku/Bujur Sangkar:**
   Karena kurva trajektori hijau telah dikonstruksi membentuk pola bujur sangkar melengkung ($R = 2.2\text{ m}$) pada tikungan antargate, maka pergerakan fisik kapal di air dipastikan **100% mengikuti pola bujur sangkar tersebut**, meniadakan fenomena hanyut diagonal atau pemotongan sudut.

---

### ENH-FR-7: Protokol Khusus Penyelesaian Manuver Tikungan Tajam Antargate (Gate 3 $\to$ Gate 4 & Gate 7 $\to$ Gate 8)

Protokol ini secara komprehensif menyelesaikan *bottleneck* ketika kapal telah menuntaskan gerbang lurus (Gate 3 atau Gate 7) dan **gerbang berikutnya (Gate 4 atau Gate 8) berada di belokan $90^\circ$ serta BELUM MASUK sama sekali ke dalam bidang pandang (*Field of View - FOV*) kamera** akibat ruang kosong antargate:

```
                      [Gate 4 Terlihat!]
                R4 ●──────────────────────● G4
                           ▲
                           │  (Fase 3: Hand-off Visi Seketika)
                           │  Spline C^2 Mengunci Gate 4 Tegak Lurus
                          ╭╯
                         ╭╯
                       ╭╯  (Fase 2: Busur Virtual Berbasis IMU)
                      ╭╯   Kapal berbelok kurvatur R = 2.2 m
                     ╭╯    Moncong kapal aktif menoleh ke arah Gate 4
                    ╭╯
              R3 ●──┼──● G3   (Fase 1: Keluar Gate 3)
                    │         Kamera belum melihat apa-apa (Air Kosong)
                  [ASV]
```

#### ENH-FR-7.1: Pemicu Keluar Deterministik (*Deterministic Exit Trigger*)
* Mengeliminasi timer buta waktu eksisting (`clearing_elapsed_ >= 3.0 detik`).
* Transisi ke mode belok dipicu oleh kondisi geometris metrik absolut:
  $$\text{Gate } k \text{ dinyatakan PASSED jika: } X_{rel, base} < 0.0\text{ meter dan } \|P_{mid, k}\| < 1.5\text{ meter}$$
* Tepat saat `gate_count == 3` (atau `gate_count == 7`) berstatus *PASSED*, sistem langsung mengalihkan status kendali dari `VISUAL_CORRIDOR` ke **`BLIND_CORNERING_ARC`**.

#### ENH-FR-7.2: Pembangkit Busur Lingkaran Buta Virtual (*Virtual Blind Cornering Arc Generator*)
Selama kamera hanya melihat air kosong, perencana jalur secara otonom membangkitkan trajektori busur lingkaran virtual di frame lokal `base_link` tanpa memerlukan titik deteksi pelampung:
1. **Arah Putar (*Turn Direction*):** Diketahui secara apriori dari regulasi layout sirkuit KKI 2026:
   - Sisi Kiri (Course A): Belok Kiri ($\text{sign} = +1$).
   - Sisi Kanan (Course B): Belok Kanan ($\text{sign} = -1$).
2. **Geometri Busur Lingkaran Virtual:**
   Jalur virtual diparametrisasikan terhadap sudut haluan relatif $\theta \in [0, \pi/2]$:
   $$X_{virtual}(\theta) = R_{arc} \cdot \sin(\theta), \quad Y_{virtual}(\theta) = \text{sign} \cdot R_{arc} \cdot (1 - \cos(\theta))$$
   dengan radius belok aman:
   $$R_{arc} = 2.2\text{ meter} \quad (1.8\text{ m} \le R_{arc} \le 2.5\text{ m})$$
   menjamin kapal tidak meluncur melebar (*drift out*) mendekati batas dinding arena $30\text{ m}$.
3. **Pelacakan Haluan Presisi via Gyroscope IMU:**
   Kontroler ILOS melacak kurva busur ini menggunakan kecepatan sudut geleng (*yaw rate*) $r = \dot{\psi}$ dari **Gyroskop IMU Pixhawk pada rate 100 Hz**:
   $$\psi_{target}(t) = \psi_{start} + \text{sign} \cdot \int_0^t \left(\frac{U_{turn}}{R_{arc}}\right) dt$$
   menjamin kapal berbelok stabil dengan radius kurvatur konstan di atas air, bukan berputar di tempat (*spinning*) atau meluncur liar.

#### ENH-FR-7.3: Interupsi Preemption Visi Seketika (*Visual Re-acquisition Hand-off*)
Sambil kapal melaju menyusuri busur putar, kamera aktif memindai perairan di depan haluan kapal yang terus berotasi:
1. **Titik Temu Visual:** Pada sudut putar haluan $\theta \approx 25^\circ - 35^\circ$, pasangan pelampung Gate 4 dijamin memasuki bidang pandang (FOV) kamera.
2. **Kondisi Interupsi (*Preemption Trigger*):**
   $$\text{Jika: } N_{buoy\_detected} \ge 2 \text{ dan } \text{Confidence} \ge 0.60 \text{ terkonfirmasi } \ge 2 \text{ frame berturut-turut}$$
   maka:
   - **Mode Busur Virtual langsung DIBATALKAN seketika (*immediate preemption*)**.
   - Status sistem beralih mulus ke **`VISUAL_APPROACHING`**.
3. **Penyelarasan Masuk Tegak Lurus (*Gate Normal Hand-off*):**
   Modul Spline langsung menyambungkan posisi haluan kapal saat interupsi terjadi ke titik tengah Gate 4 dengan mematuhi vektor normal gerbang ($\hat{n}_4$). Kapal otomatis meluruskan haluan dan menembus celah 2 meter Gate 4 tepat di garis sumbu tengah.

#### ENH-FR-7.4: Profil Kecepatan Adaptif Tikungan Tajam (*Curvature Speed Profiling*)
Untuk mencegah terjadinya gaya sentrifugal berlebih dan *hydrodynamic sideslip* saat melintasi tikungan kosong antargate:
Kecepatan linier diturunkan secara mulus dari kecepatan jelajah ($3.0\text{ m/s}$) ke kecepatan tikungan aman ($1.8\text{ m/s}$):
$$U_{turn} = \max\left(1.8\text{ m/s}, \, U_{cruise} - k_{slow} \cdot |\kappa|\right) \quad (k_{slow} = 2.2\text{ m}^2/\text{s})$$
Memberikan margin waktu respons hidrodinamik yang cukup bagi servo kemudi rudder ($|\dot{\delta}_r| \le 60^\circ/\text{s}$) untuk mempertahankan trajektori.

#### ENH-FR-7.5: Watchdog Failsafe Belokan Buta (*Blind Cornering Watchdog*)
Jika terjadi anomali ekstrem di mana Gate 4 hanyut/hilang sehingga kapal telah menyelesaikan putaran busur $\theta \ge 95^\circ$ namun kamera tetap belum mendeteksi pelampung:
1. Kapal menahan haluan lurus pada sudut $90^\circ$ tersebut.
2. Melaju perlahan pada kecepatan aman $1.0\text{ m/s}$ mencari objek selama maksimal $2.5\text{ detik}$.
3. Jika $> 2.5\text{ detik}$ tetap tidak ada pelampung terdeteksi, kapal mengaktifkan **`FAILSAFE_IDLE`** (throttle 0, rudder netral) untuk mencegah tabrakan dengan tepi pembatas danau.

#### ENH-FR-7.6: Arsitektur Transisi State Machine C++
```cpp
// Contoh implementasi logika transisi pada BuoyCenteringNode:
void BuoyCenteringNode::processCorneringState(double dt) {
    if (state_ == ControllerState::CLEARING_GATE) {
        if (is_gate_behind_boat(current_gate_id_)) { // X_rel < 0
            arc_accumulated_angle_rad_ = 0.0;
            turn_start_heading_ = heading_deg_;
            transitionTo(ControllerState::BLIND_CORNERING_ARC);
        }
    }
    else if (state_ == ControllerState::BLIND_CORNERING_ARC) {
        // Cek apakah mata kamera sudah menemukan gerbang berikutnya
        if (tracker_->has_confirmed_next_gate()) {
            RCLCPP_INFO(get_logger(), "Gate %d terakuisisi visual! Interupsi busur virtual.", current_gate_id_ + 1);
            transitionTo(ControllerState::APPROACHING_GATE); // Visi Delaunay + Spline C^2 aktif
            return;
        }

        // Eksekusi trajektori busur virtual berbasis IMU
        arc_accumulated_angle_rad_ += (turn_linear_speed_ / arc_radius_) * dt;
        publishVirtualArcTrajectory(arc_radius_, arc_direction_);
        
        // Watchdog pengaman putaran penuh
        if (arc_accumulated_angle_rad_ >= (M_PI / 2.0 + 0.1)) {
            transitionTo(ControllerState::SEARCHING_FORWARD);
        }
    }
}
```

* **Acceptance Criteria:**
  - Kapal sukses mengeksekusi manuver tikungan $90^\circ$ antargate tanpa melenceng ke luar radius aman $R \le 2.5\text{ m}$.
  - Transisi dari *blind virtual arc* ke *visual spline* berlangsung dalam waktu $\le 35\text{ ms}$ begitu gerbang baru terdeteksi.
  - Sudut masuk kapal saat melintasi Gate 4 (dan Gate 8) tidak melenceng $> 15^\circ$ dari garis normal celah gerbang 2 meter.
  - Nol kasus serempetan pelampung pada belokan Gate 3 $\to$ 4 dan Gate 7 $\to$ 8.

---

### ENH-FR-8: Sistem Verifikasi Visi Berbasis Augmented Reality (AR) Reprojection pada Playback Rosbag

Modul ini dirancang khusus untuk memecahkan *bottleneck* pengujian saat wahana fisik telah di-*shipping* ke venue lomba, memanfaatkan rekaman rosbag video perairan nyata menjadi sarana verifikasi diagnostik aktif (*Active Visual HUD Debugger*). 

Sesuai dengan visualisasi standar *Formula Student Driverless*, sistem **tidak memerlukan panah tebal terpisah yang membuat frame padat/cluttered**, melainkan langsung memproyeksikan **kurva trajektori hijau ($C^2$ Spline / Virtual Arc)** yang telah dihitung planner ke atas permukaan air secara fotorealistik, berujung tepat pada **bujur sangkar target dinamis (*destination bracket*)** di gerbang aktif.

```
┌─────────────────────────────────────────────────────────────┐
│  Kamera Asli Rosbag + AR Reprojection Overlay               │
│                                                             │
│                 [ TARGET GATE 4 ]                           │
│                        ┌───┐                                │
│                        │ ● │ (Pelampung Target Terkunci)    │
│                        └───┘                                │
│                     ╭──────╯                                │
│                   ╭─╯   <-- Proyeksi Garis Trajektori Hijau │
│                 ╭─╯         (C^2 Spline / Arc di Bidang Air)│
│                │                                            │
│              [ASV]                                          │
│ Speed: 3.5 m/s | Steer: +24.2° | ψ_err: 1.8°                │
└─────────────────────────────────────────────────────────────┘
```

#### ENH-FR-8.1: Formulasi Matematika Proyeksi Balik 3D ke 2D (*Forward Perspective Projection*)
Setiap titik 3D pada kurva trajektori metrik $P_{path, k} = [X_k, Y_k, 0]^\top$ (baik dari *Spline C^2* maupun *Virtual Blind Arc*) diproyeksikan kembali ke ruang piksel kamera 2D $(u_k, v_k)$:
1. Transformasi koordinat dari `base_link` ke frame optik kamera dengan kompensasi sudut IMU (roll $\phi$, pitch $\theta$) yang terekam pada frame rosbag tersebut:
   $$P_{cam} = (R_c^b)^{-1} \cdot R_{IMU}^{-1}(\phi, \theta) \cdot \begin{bmatrix} X_k - x_{cam\_offset} \\ Y_k - y_{cam\_offset} \\ -h_{cam} \end{bmatrix}$$
2. Titik piksel 2D pada citra diperoleh melalui perkalian matriks intrinsik kamera $K$:
   $$\begin{bmatrix} u_k \\ v_k \\ 1 \end{bmatrix} \sim K \cdot \begin{bmatrix} P_{cam, x} / P_{cam, z} \\ P_{cam, y} / P_{cam, z} \\ 1 \end{bmatrix}$$
   *(Diimplementasikan secara efisien via OpenCV `cv2.projectPoints()` atau `cv::projectPoints()`).*

#### ENH-FR-8.2: Reproyeksi Kurva Trajektori Hijau di Permukaan Air (*Ground-Plane Trajectory Reprojection*)
1. Menghubungkan titik-titik piksel terproyeksi $(u_k, v_k)$ menjadi garis kurva trajektori mulus berwarna hijau terang (*solid green*, BGR: `(0, 255, 0)`):
   - Ketebalan garis disesuaikan secara perspektif: lebih tebal di dekat moncong kapal ($w \approx 6 - 8\text{ px}$) dan menipis proporsional seiring bertambahnya jarak menuju titik target ($w \approx 2 - 3\text{ px}$).
   - Garis ini secara instan merefleksikan arah manuver kapal:
     - **Lurus:** Garis hijau membentang lurus membelah celah gerbang-gerbang berikutnya.
     - **Belok Tajam (Gate 3 $\to$ 4):** Garis hijau melengkung mulus ke arah kanan, memperlihatkan *lead-in cornering arc* yang membawa kapal mengitari tikungan tanpa memotong diagonal.
2. Status Manuver Khusus:
   - Warna garis hijau bertransisi menjadi **Kuning / Cyan Terang** saat wahana berada dalam fase *blind cornering arc* untuk membedakan mode trajektori visual vs inersial.

#### ENH-FR-8.3: Bujur Sangkar Target Dinamis (*Dynamic Destination Bounding Box / Bracket*)
1. Untuk gerbang aktif ke-$k$ yang menjadi target kontroler (koordinat $P_{mid, k}$):
   - Proyeksikan koordinat $P_{mid, k}$ ke titik piksel layar $(u_{t}, v_{t})$.
2. Gambarkan bingkai sudut bujur sangkar dinamis (*corner brackets*) di sekeliling target:
   - Ukuran bujur sangkar berskala adaptif terhadap jarak: $S_{box} = \frac{f_x \cdot W_{box\_metric}}{Z_{cam}}$.
   - Label teks HUD di atas bujur sangkar: `[ TARGET GATE k ]` disertai jarak metrik $d = \sqrt{X^2+Y^2}$.
3. **Verifikasi Bebas Salah (*No Gate Skipping*):** Memberikan konfirmasi visual instan bahwa tracker mengunci gerbang target yang benar secara sekuensial (Gate 1 $\to$ 2 $\to$ 3 $\to$ 4 $\to$ 5), membuktikan bahwa masalah lompat gerbang pada tikungan Gate 3 $\to$ 4 telah tereliminasi.

#### ENH-FR-8.4: Node ROS 2 `ar_guidance_visualizer`
* **Input Topik:**
  - `/camera/image_raw` (`sensor_msgs/msg/Image`) dari playback rosbag.
  - `/planning/centerline_path` (`nav_msgs/msg/Path`).
  - `/planning/target_pose` (`geometry_msgs/msg/PoseStamped`).
  - `/mavros/imu/data` (`sensor_msgs/msg/Imu`).
* **Output Topik:**
  - `/camera/debug_ar_overlay` (`sensor_msgs/msg/Image`) untuk dipantau secara real-time di RViz2 atau diekspor ke format video `.mp4`.
* **Implementasi Ringan & Bersih (Python / OpenCV):**
```python
import cv2, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class ArGuidanceVisualizer(Node):
    def __init__(self):
        super().__init__('ar_guidance_visualizer')
        self.bridge = CvBridge()
        self.pub_overlay = self.create_publisher(Image, '/camera/debug_ar_overlay', 10)
        # Subscribe /camera/image_raw, /planning/centerline_path, /planning/target_pose, /mavros/imu/data
        
    def render_overlay(self, cv_image, path_points_3d, target_point_3d, is_blind_arc=False):
        # 1. Gambar Kurva Trajektori Hijau di Permukaan Air
        if len(path_points_3d) >= 2:
            pts_2d, _ = cv2.projectPoints(path_points_3d, self.rvec, self.tvec, self.K, self.dist)
            pts_2d = np.int32(pts_2d).reshape(-1, 2)
            color = (0, 255, 255) if is_blind_arc else (0, 255, 0) # Kuning saat blind arc, Hijau saat normal
            for i in range(len(pts_2d) - 1):
                w = max(2, int(8 * (1.0 - i / len(pts_2d))))
                cv2.line(cv_image, tuple(pts_2d[i]), tuple(pts_2d[i+1]), color, w, cv2.LINE_AA)
            
        # 2. Gambar Bujur Sangkar Target Dinamis (Destination Bracket)
        if target_point_3d is not None:
            t_2d, _ = cv2.projectPoints(np.array([target_point_3d]), self.rvec, self.tvec, self.K, self.dist)
            u, v = int(t_2d[0][0][0]), int(t_2d[0][0][1])
            s = max(15, min(50, int(600.0 / max(target_point_3d[0], 1.0)))) # Skala dinamis thd jarak X
            # Gambar Corner Brackets
            c_len = s // 3
            c_color = (0, 255, 255) # Cyan/Yellow
            cv2.line(cv_image, (u - s, v - s), (u - s + c_len, v - s), c_color, 2)
            cv2.line(cv_image, (u - s, v - s), (u - s, v - s + c_len), c_color, 2)
            cv2.line(cv_image, (u + s, v - s), (u + s - c_len, v - s), c_color, 2)
            cv2.line(cv_image, (u + s, v - s), (u + s, v - s + c_len), c_color, 2)
            cv2.line(cv_image, (u - s, v + s), (u - s + c_len, v + s), c_color, 2)
            cv2.line(cv_image, (u - s, v + s), (u - s, v + s - c_len), c_color, 2)
            cv2.line(cv_image, (u + s, v + s), (u + s - c_len, v + s), c_color, 2)
            cv2.line(cv_image, (u + s, v + s), (u + s, v + s - c_len), c_color, 2)
            cv2.putText(cv_image, "TARGET GATE", (u - s, v - s - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, c_color, 1)
        return cv_image
```

* **Acceptance Criteria:**
  - Rendering AR overlay berjalan real-time ($\ge 30\text{ FPS}$) sinkron dengan pemutaran rosbag tanpa penurunan performa.
  - Kurva trajektori hijau dan bujur sangkar target secara visual mengonfirmasi bahwa wahana membidik celah gerbang secara presisi dan urut sepanjang rekaman video rosbag.

---

## 4. Kebutuhan Non-Fungsional Peningkatan (Targeted NFRs)

| Metrik | Target Spesifikasi | Metode Pengujian & Validasi |
| :--- | :--- | :--- |
| **End-to-End Latency** | Waktu dari shutter kamera hingga penerbitan paket PWM aktuator $\le 20\text{ ms}$ ($\ge 50\text{ FPS}$). | Profiling timestamp hardware C++ `std::chrono::steady_clock` dari eksposur V4L2 hingga publikasi `/mavros/rc/override`. |
| **Throughput Arsitektur** | Zero-Copy Intra-Process Communication. | Mengompilasi seluruh modul Persepsi, IPM, Delaunay, dan Spline ke dalam container tunggal `rclcpp_components`. |
| **Ketahanan Dinamika Gelombang** | Estimasi koordinat metrik 3D tetap stabil pada guncangan: Roll $\pm 15^\circ$, Pitch $\pm 10^\circ$, Heave $\pm 0.25\text{ m}$. | Pengujian simulator SITL dengan gangguan ombak buatan dan uji fisik di kolam bergelombang. |
| **Kepatuhan Standar ROS** | Kepatuhan penuh pohon TF2 terhadap **REP-103** (`base_link`), **REP-104** (`camera_optical_frame`), dan **REP-105** (`map` $\to$ `odom` $\to$ `base_link`). | Verifikasi grafik pohon koordinat menggunakan `ros2 run tf2_tools view_frames`. |
| **Fail-safe Reaktif** | Kapal mempertahankan haluan terakhir selama $1.0\text{ s}$ lalu memotong throttle ke nol (*idle*) jika kehilangan visual $> 0.5\text{ s}$. | Pengujian pemutusan sinyal video secara mendadak saat kapal melaju pada kecepatan penuh. |

---

## 5. Antarmuka Topik ROS 2 yang Diperbarui

```
                  ┌─────────────────────────────────────────┐
                  │          /camera/image_raw              │
                  │       (sensor_msgs/msg/Image)           │
                  └────────────────────┬────────────────────┘
                                       │
                                       ▼
                  ┌─────────────────────────────────────────┐
                  │       /perception/detections_2d         │
                  │        (msgs/msg/ObstacleArray)         │
                  │   [u_min, v_min, u_max, v_max, class]   │
                  └────────────────────┬────────────────────┘
                                       │
                                       ▼
                  ┌─────────────────────────────────────────┐
                  │       /perception/metric_buoys_3d       │
                  │        (msgs/msg/ObstacleArray)         │
                  │           [X, Y, Z=0 in base_link]      │
                  └────────────────────┬────────────────────┘
                                       │
                                       ▼
                  ┌─────────────────────────────────────────┐
                  │       /planning/centerline_path         │
                  │          (nav_msgs/msg/Path)            │
                  │     [C^2 Trajektori Mulus - Hijau]      │
                  └────────────────────┬────────────────────┘
                                       │
                                       ▼
                  ┌─────────────────────────────────────────┐
                  │          /mavros/rc/override            │
                  │      (mavros_msgs/msg/OverrideRCIn)     │
                  │        [Rate: 50 Hz, Ch1 & Ch3]         │
                  └─────────────────────────────────────────┘
```

1. **`/perception/detections_2d` (`msgs/msg/ObstacleArray` - Diperbarui):**
   - Menambahkan array koordinat piksel $[u_{min}, v_{min}, u_{max}, v_{max}]$.
   - Menambahkan definisi enum konstanta kelas: `RED=1`, `GREEN=2`, `YELLOW_OBSTACLE=6`, `WHITE_TARGET=7`.
2. **`/perception/metric_buoys_3d` (`msgs/msg/ObstacleArray` - Topik Baru):**
   - Berisi posisi metrik fisik $(X, Y)$ dalam frame koordinat `base_link`.
   - Mengelompokkan pelampung yang telah divalidasi oleh 3D EKF Tracking ($\ge 3$ frame) dan persistensi oklusi tikungan.
3. **`/planning/delaunay_mesh` (`visualization_msgs/msg/MarkerArray` - Topik Visualisasi Baru):**
   - Menampilkan garis putih jaring Delaunay dan garis batas koridor di RViz2 persis seperti visual gambar referensi.
4. **`/planning/centerline_path` (`nav_msgs/msg/Path` - Topik Baru):**
   - Trajektori garis tengah kontinu $C^2$ dengan *Gate Normal Alignment* (garis hijau pada gambar referensi).
5. **`/mavros/rc/override` (`mavros_msgs/msg/OverrideRCIn` - Peningkatan Rate):**
   - Frekuensi penerbitan dinaikkan dari 10 Hz menjadi **50 Hz** sinkron.
6. **`/camera/debug_ar_overlay` (`sensor_msgs/msg/Image` - Topik Visualisasi AR Baru):**
   - Citra video asli beranotasi kurva trajektori hijau ($C^2$ Spline) di atas permukaan air dan bujur sangkar target dinamis (*destination bracket*) untuk validasi visual playback rosbag.

---

## 6. Referensi Literatur Ilmiah yang Digunakan (*Academic Citations*)

1. **Alvarez, A., Denner, N., Feng, Z., et al. (2022).** *The Software Stack That Won the Formula Student Driverless Competition.* arXiv preprint.  
   Tautan: [https://arxiv.org/abs/2210.10933](https://arxiv.org/abs/2210.10933) (PDF: [https://arxiv.org/pdf/2210.10933v1](https://arxiv.org/pdf/2210.10933v1))  
   *Rujukan untuk: Pemetaan koridor batas lintasan berbasis Triangulasi Delaunay dan penghalusan kurva kubik parametrik.*

2. **Lens, S., & Boigelot, B. (2016).** *From Constrained Delaunay Triangulations to Roadmap Graphs with Arbitrary Clearance.* arXiv preprint.  
   Tautan: [https://arxiv.org/abs/1606.02055](https://arxiv.org/abs/1606.02055) (PDF: [https://arxiv.org/pdf/1606.02055v1](https://arxiv.org/pdf/1606.02055v1))  
   *Rujukan untuk: Pemangkasan edge pada graf triangulasi terkonstrain untuk jalur aman bebas rintangan.*

3. **Fossen, T. I., Pettersen, K. Y., & Galeazzi, R. (2014).** *Line-of-Sight Path Following for Dubins Paths with Sideslip Angle Compensation of Marine Craft.* IEEE Transactions on Control Systems Technology.  
   *Rujukan untuk: Formulasi matematis Integral Line-of-Sight (ILOS) dengan aturan diferensial anti-windup guna mengeliminasi penyimpangan lateral tunak akibat arus.*

4. **Kabzan, J., Valls, M. I., Reijgwart, V., et al. (2020).** *AMZ Driverless: The Full Autonomous Racing System.* Journal of Field Robotics, 37(7), 1267–1294.  
   *Rujukan untuk: Pipeline deteksi objek berbasis kamera-IMU, penjejakan temporal konfirmasi 3-frame, dan pengoptimalan trajektori kurvatur minimum.*

5. **Lekkas, A. M., & Fossen, T. I. (2014).** *Integral LOS Path Following for Curved Paths Based on a Monomolecular Growth Law.* IEEE Conference on Control Applications (CCA).  
   *Rujukan untuk: Penentuan jarak pandang adaptif dinamis ($\Delta(U)$) terhadap kecepatan linier wahana.*
