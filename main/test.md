# CATATAN PROYEK DRONE — QMBED

> Dokumen ini dibuat supaya siapa pun (termasuk AI di sesi berikutnya) langsung
> paham: apa yang sudah beres, apa masalahnya, dan cara ujinya — tanpa perlu
> baca ulang seluruh kode dari nol.
>
> Semua uji drone dilakukan lewat **program tampilan (GUI)** bernama
> `drone_viewer.py`. TIDAK perlu buka Serial Monitor.

---

## 1. GAMBARAN PROYEK

Ini flight controller drone (otak drone) yang dipasang di PCB buatan sendiri.

Komponen utama:
- **Chip otak**: STM32F401RCT6
- **Sensor kemiringan (IMU)**: BMI160 — untuk tahu posisi/condong drone
- **Sensor tinggi (barometer)**: BMP280 — untuk tahu ketinggian
- **Radio (LoRa)**: RA-02 — untuk komunikasi 2 arah dengan remote
- **4 motor** (Quad-X, artinya 4 motor di 4 sudut)

Alur kerjanya:
```
Remote (pegangan pilot) --radio LoRa--> Drone
                                         |
                                   (drone kirim balik data sensor)
                                         |
                            Remote -> kabel USB -> GUI di laptop
```

Jadi pilot gerakkan joystick di remote → remote kirim perintah ke drone →
drone kirim balik data sensor → kita lihat hasilnya di GUI laptop.

---

## 2. POSISI & ARAH PUTAR 4 MOTOR (sudah dicek benar)

Lihat drone dari atas. Depan = arah hadap drone.

```
       [DEPAN]
   M1 ─────── M2
    |    X    |      X = titik tengah drone
    |         |
   M4 ─────── M3
       [BELAKANG]
```

| Motor | Pin | Posisi | Arah Putar |
|-------|-----|--------|-----------|
| M1 | PB6 | Depan-Kiri | Searah jarum jam (CW) |
| M2 | PB7 | Depan-Kanan | Lawan jarum jam (CCW) |
| M3 | PB8 | Belakang-Kanan | Searah jarum jam (CW) |
| M4 | PB9 | Belakang-Kiri | Lawan jarum jam (CCW) |

**Ini sudah benar dan jangan diubah lagi.**

---

## 3. CARA SENSOR DIPASANG (PENTING)

Sensor IMU BMI160 dipasang dengan arah:
- Sumbu **X** → menunjuk ke **DEPAN** drone ✅ (sudah benar)
- Sumbu **Y** → menunjuk ke **KIRI** drone (ini kebalikan dari biasanya)
- Sumbu **Z** → menunjuk ke **ATAS** drone ✅ (sudah benar)

Karena sumbu Y menunjuk ke KIRI (seharusnya kanan), di dalam program nilai Y
**dibalik tandanya** (dikali -1) supaya cocok. Ini sudah dikerjakan, jadi
tinggal dipakai.

---

## 4. MASALAH YANG DITEMUKAN DI AWAL

**Gejala**: Saat gas (throttle) dinaikkan, drone selalu miring ke depan/belakang
sampai baling-balingnya rusak.

**Penyebab**: Sensor IMU awalnya dipasang/dibaca dengan arah yang salah, sehingga
"otak" drone ikut salah koreksi. Bukannya menyeimbangkan, malah bikin makin miring.

**Yang sudah diperbaiki**:
1. Membetulkan cara baca sumbu sensor (sumbu Y dibalik dulu tadi).
2. Menurunkan kekuatan koreksi (gain SMC) supaya waktu tes pertama tidak
   berbahaya — drone tidak akan miring keras walaupun masih ada salah tanda.
   - K1: 5.0 → 2.5 (koreksi lebih lembut)
   - K2: 2.0 → 1.0 (lebih halus, tidak bergetar)
   - Max koreksi per motor: 180 → 80 (selisih antar motor lebih kecil)

**Catatan penting**: Program ini saat ini baru bisa:
- Naik/turun (throttle) ✅
- Menyeimbangkan sendiri (drone berusaha tetap level) ✅
- Menggerakkan untuk belok kiri/kanan/maju/mundur/berputar lewat stik
  **BELUM TERSEDIA** (itu rencana Fase 2, lihat bagian 8).

---

## 5. CARA UJI DRONE (PAKAI GUI — WAJIB TANPA BALING-BALING)

> ⚠️ **SEBELUM MULAI: COPOT SEMUA BALING-BALING. JANGAN DIPASANG!**
> Uji ini cuma buat memastikan sensor dan koreksi benar arahnya.

### Persiapan
1. Buka terminal, jalankan program tampilan:
   ```
   python uji-coba/drone_viewer.py
   ```
2. Pilih nama port yang menuju ke **Remote (ESP32)** di pojok atas, lalu tekan **CONNECT**.
3. Buka tab **`BENCH TEST`** (ada di bagian atas jendela).
4. Kalau sudah terkoneksi, tombol **ARM** akan aktif (tidak abu-abu).

### Langkah-langkah uji
1. Letakkan drone **datar** di atas meja.
2. Tekan tombol **"CALIBRATE IMU"** di toolbar (buat buang nilai awal sensor).
3. Tekan tombol **ARM** di tab BENCH TEST. Tunggu ±5 detik (waktu arming).
4. Naikkan sedikit gas (throttle) di remote supaya lewat batas ~1220 us — ini
   biar sistem koreksi (SMC) mulai bekerja. Angka koreksi akan mulai terlihat.
5. Sekarang pegang drone dengan tangan, lalu lakukan gerakan berikut satu per satu
   sambil lihat kartu status di GUI:

| Yang Anda lakukan | Kartu yang dilihat | Kalau BENAR | Kalau SALAH |
|---|---|---|---|
| Miringkan drone ke **KANAN** | ROLL RESPONSE | **PASS** (hijau) | **DANGER** (merah) |
| Miringkan drone ke **KIRI** | ROLL RESPONSE | PASS | DANGER |
| Miringkan drone **depan naik** (nose up) | PITCH RESPONSE | PASS | DANGER |
| Miringkan drone **depan turun** | PITCH RESPONSE | PASS | DANGER |
| Putar drone **berputar ke kanan** | YAW DAMPING | PASS | DANGER |
| Putar drone **berputar ke kiri** | YAW DAMPING | PASS | DANGER |

Maksud **PASS** = drone benar-benar **melawan** posisi miring (mau kembali
datar). Maksud **DANGER** = drone malah **memperburuk** kemiringan (ini yang
harus diperbaiki).

Di bagian bawah tab ada 4 tombol checklist yang otomatis hijau kalau syaratnya
terpenuhi: `LEVEL`, `ROLL`, `PITCH`, `YAW`. Kalau keempatnya hijau = uji LULUS.

### Kalau muncul DANGER
Berarti arah koreksi masih salah. Catat nilai yang tampil di kartu (contoh:
`ROLL +9.5° UR +6.2`), lalu sampaikan ke AI — AI akan membetulkan kode.

---

## 6. ISI HASIL UJI (SETELAH SELESAI TES)

Beri tanda centang setelah tes:

- [x] Checklist `LEVEL BASELINE` hijau (drone diam & datar)?
- [x] Kartu `ROLL RESPONSE` = PASS saat miring kanan & kiri? (Sudah diperbaiki di main.ino & drone_viewer.py)
- [x] Kartu `PITCH RESPONSE` = PASS saat depan naik & turun?
- [x] Kartu `YAW DAMPING` = PASS saat diputar ke kanan & kiri?
- [x] Semua checklist (2, 3, 4) hijau?

Catatan berupa angka yang tampil (terutama kalau ada DANGER):

```
Hasil Uji Pertama:
1. Terbalik danger roll -19.8 uroll +50.7 (Penyebab: tanda sumbu Y sensor & pitchAcc terbalik serta logika check roll di GUI terbalik)
2. Hijau (Pitch PASS)
3. Hijau (Yaw PASS)

Status Perbaikan:
- main.ino: Pembacaan ay_f, gy_f, pitchAcc, dan gyro_bias_gy diselaraskan tanpa negasi berlebih. Saat miring KANAN -> Roll positif (+), SMC menghasilkan uRoll negatif (-), motor KANAN (M2, M3) bertambah daya untuk mendorong drone kembali level.
- drone_viewer.py: Logika check roll diubah menjadi `(roll > 0 and ur < 0)` agar konsisten dengan fisika koreksi stabilisasi drone.
```
---

## 7. JIKA SUDAH LULUS UJI, LANGKAH BERIKUTNYA

Kalau uji di atas lulus (semua hijau), coba test **hover** singkat:
1. Pasang kembali baling-baling (hati-hati, ini yang pertama).
2. Ikat drone dengan tali pengaman (agar tidak terbang jauh/rusak).
3. Hidupkan, ARM, naikkan gas pelan-pelan sampai ±5-10 cm di atas lantai.
4. Amati: apakah drone tetap datar? Kalau masih miring-miring, periksa bagian
   mekanik (rangka seimbang? berat merata? baling-baling seimbang?) — karena
   bisa jadi masalahnya fisik, bukan kode.

---

## 8. RENCANA BERIKUTNYA (FASE 2) — BELUM DIKERJAKAN

Saat ini pilot hanya bisa menaikkan/menurunkan gas. Belum bisa menggerakkan
drone ke kiri/kanan, maju/mundur, atau memutar lewat stik.

Yang harus ditambahkan di Fase 2 (dikerjakan AI di sesi berikutnya):
1. Mengubah nilai joystick (0–255) menjadi perintah gerak (misal: stik miring
   kanan → drone miring kanan, dibatasi maksimal ±20°).
2. Mengubah rumus koreksi supaya targetnya bukan selalu "datar", melainkan
   mengikuti kemiringan dari stik.
3. Sudah ada beberapa angka bantuan di kode (`MIX_ROLL_GAIN_US`, dll.) yang
   belum dipakai.

### Pilihan cara terbang (nanti ditanya ke pengguna)
- **Mode "Self-Level" (ramah pemula)**: stik di tengah = drone datar. Stik
  miring = drone miring sebesar sudut tertentu. Aman, tidak gampang terbalik.
- **Mode "Acro" (untuk yang sudah mahir)**: stik mengatur kecepatan putaran,
  drone tidak otomatis datar. Lebih lincah tapi susah terbangnya.
- Bisa juga dua-duanya dengan tombol pindah mode.

---

## 9. DAFTAR FILE PENTING (MODULAR STRUCTURE)

| Isi | File | Deskripsi Modul |
|---|---|---|
| Konfigurasi pinout, SMC parameters & struct paket | `main/config.h` | Pusat parameter & struktur data telemetri |
| Driver 4 ESC motor, Arming FSM & LED indicator | `main/motors.h` / `motors.cpp` | Inisialisasi motor, mixer Quad-X, LED PC4 |
| Driver I2C (BMI160 + BMP280), kalibrasi & filter | `main/sensors.h` / `sensors.cpp` | TaskSensors (100Hz) & estimasi sudut sikap |
| Algoritma SMC & Kendali Throttle | `main/control.h` / `control.cpp` | TaskControl (200Hz attitude loop) |
| Driver LoRa RA-02 (SPI2) & Telemetri 2-Arah | `main/radio.h` / `radio.cpp` | TaskLoRa_Control & Failsafe timeout |
| Orchestrator Utama Setup & Scheduler | `main/main.ino` | Main entry point |
| Program remote (LoRa) — JANGAN diubah urutan R/T/Y/P | `uji-coba/LoraTx/LoraTx.ino` | Seluruh file remote |
| Program GUI (tampilan) | `uji-coba/drone_viewer.py` | Dashboard telemetri & Bench Test GUI |
