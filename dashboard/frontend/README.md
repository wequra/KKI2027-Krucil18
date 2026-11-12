# Dagozilla GCS - Frontend Dashboard

Aplikasi web modern berbasis **React 18** dan **Vite** ini dirancang secara khusus untuk menjadi "wajah" (antarmuka) dari sistem kendali Dagozilla ASV. 

Aplikasi ini menyajikan seluruh informasi kapal secara langsung (tanpa perlu *refresh* halaman) berkat koneksi **WebSocket** 2 arah ke layanan Backend, serta mampu menampilkan *live video feed* latensi-rendah menggunakan **WebRTC**.

## Fitur Unggulan
- **Estetika Premium (Dark Mode):** Desain UI yang padat informasi namun elegan, mengambil filosofi warna kapal Dagozilla (Biru Dongker, Kuning Emas, Merah, Putih).
- **Custom CSS Grid:** Menggunakan CSS *Grid Layout* dan struktur komponen tanpa framework tambahan, menjamin *dashboard* tidak berat dan ringan dimuat.
- **WebSocket Telemetry:** Papan indikator dan Peta Navigasi 5x5 merespons pergerakan kapal (Heading, SOG, Koordinat) secara instan.
- **Glassmorphism & Micro-animations:** Tampilan layar yang merespons status secara interaktif.
- **Collapsible UI (Accordion):** Panel kontrol dan kamera di sisi kanan dapat dilipat (buka-tutup) untuk meminimalisir distraksi visual dan menghemat ruang layar bagi operator.

---

## ⚠️ Limitasi & Pekerjaan Rumah (TODO)
- **Status Dummy Data:** Beberapa fitur seperti koneksi IP Mission Control, tombol `Start`, dan `Abort` saat ini baru berupa tampilan antarmuka (*UI Mockup*). Logikanya harus disambungkan dengan pemanggilan API (menggunakan *fetch* atau *axios* ke `POST /events/` di Backend) agar tombol tersebut benar-benar mengirim perintah.
- **Map Origin Calibration:** Kanvas Peta 5x5 saat ini memetakan kapal secara semu. Di lapangan sungguhan, titik pojok kiri bawah peta harus dikalibrasi (dikunci ke Latitude/Longitude awal arena lomba) agar pergerakan kapal di kanvas 100% akurat dengan pergerakan fisiknya di danau/waduk.
- **Integrasi WebRTC MediaMTX:** Komponen *Surface* dan *Underwater Imaging* masih menampilkan teks status `STANDBY`. Belum ada skrip untuk membaca *stream* WebRTC dari MediaMTX. Ini dapat diselesaikan setelah kamera fisik (hardware) terpasang.

## Cara Menjalankan (Development)
Pastikan kamu sudah menginstal Node.js versi 18 ke atas.

### Live stream dari ROS camera_raw

Node ROS meneruskan `/camera/image_raw` ke RTSP MediaMTX pada path `surface`,
misalnya `rtsp://<IP-GCS>:8554/surface`. Frontend mengambil versi WebRTC dari
MediaMTX melalui `http://localhost:8889/surface`, yang sudah dikonfigurasi di
file `.env` lokal. Jika browser frontend berjalan di komputer lain, ubah nilai
`VITE_SURFACE_CAMERA_WEBRTC_URL` ke alamat host MediaMTX yang dapat dijangkau
browser tersebut.

```bash
# 1. Instal seluruh dependensi
npm install

# 2. Nyalakan Development Server (Vite)
npm run dev
```

Aplikasi akan berjalan secara instan di `http://localhost:5173`. Pastikan juga *Backend* kamu sudah menyala agar lampu status di pojok kanan atas berubah menjadi "MONITORING ACTIVE".
