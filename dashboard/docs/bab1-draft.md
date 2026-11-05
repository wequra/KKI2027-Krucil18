# Bab I - Pendahuluan (draft v0.1)

> Draft for team review. Fill in the exact theme wording from the guidebook and last year's proposal format.

## 1.1 Latar Belakang
Indonesia adalah negara maritim dengan sektor perikanan tangkap yang besar, namun nelayan kecil masih menghadapi
tantangan keselamatan, efisiensi rute, dan pemantauan kondisi perairan. Kendaraan permukaan otonom (ASV) dapat
membantu menyusuri rute penangkapan, mendeteksi rintangan terapung (misalnya sampah laut atau rumpon), dan
mengirim data kondisi perairan ke darat secara real time.

## 1.2 Rumusan Masalah
1. Bagaimana ASV menyusuri rute pasangan pelampung secara otonom dan aman?
2. Bagaimana sistem mendeteksi objek target dan mengambil citra permukaan dan bawah air?
3. Bagaimana data posisi, kecepatan, dan misi dipantau secara real time di dasbor?

## 1.3 Tujuan
Merancang perangkat lunak ASV (autonomi, visi komputer, telemetri, dasbor) yang memenuhi aturan lomba KKI 2027.

## 1.4 Batasan
Prototipe diuji pada simulasi (ArduPilot SITL) dan data video; integrasi perangkat keras mengikuti rancangan divisi elektrik dan mekanik.

## 1.5 Relevansi dengan Misi ASV
Rute pelampung -> rute penangkapan; kotak hijau -> deteksi rumpon/sampah; kotak biru -> pencitraan sonar/terumbu;
docking -> sandar di TPI (tempat pelelangan ikan).
