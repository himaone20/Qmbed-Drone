# Dokumentasi Drone Viewer — QMBED

**Penulis:** Himawan  
**Program Studi:** Teknologi Rekayasa Instrumentasi dan Kontrol  
**Source code:** `drone_viewer.py`

## 1. Gambaran Umum

QMBED adalah aplikasi desktop Python berbasis PySide6 untuk memantau telemetry dan membantu pengujian drone Quad-X. Aplikasi menampilkan orientasi drone, altitude, baterai, input remote, grafik respons PID, dan estimasi output motor.

GUI juga menyediakan pengiriman parameter PID, ARM/DISARM, kalibrasi stick, perekaman CSV, dan pengaturan pengujian Fault Tolerant Control (FTC).

Cara penggunaan singkat:

1. Jalankan `python drone_viewer.py`.
2. Pilih COM Port perangkat, lalu klik **CONNECT**.
3. Pilih halaman melalui menu navigasi: Attitude, Control, Bench Test, PID Tuning, atau Fault Tolerant.
4. Pantau telemetry, ubah parameter bila diperlukan, atau mulai perekaman pada halaman PID Tuning.
5. Hentikan perekaman dan klik **DISCONNECT** setelah selesai.

## 2. Fitur GUI

| Fitur | Komponen GUI | Fungsi dan data/input |
| --- | --- | --- |
| COM Port dan Refresh | Combo box dan tombol refresh | Memilih port dari daftar perangkat serial serta memindai ulang port. |
| Connect / Disconnect | Tombol koneksi | Membuka atau menutup serial pada baud rate tetap **115200**. |
| Status koneksi dan pesan | Label header | Menampilkan port aktif, koneksi, aktivitas, dan pesan error. Status CONNECTED menunjukkan port terbuka, bukan jaminan telemetry drone masih masuk. |
| Navigasi | Menu dan `QStackedWidget` | Memilih satu dari lima halaman utama. |
| CAL IMU | Tombol header | Mereset filter lokal dan tampilan attitude/altitude. Tidak mengirim perintah kalibrasi ke flight controller. Data berikutnya memperbarui tampilan kembali. |
| CAL STICKS / RESET CAL | Tombol dan banner | Mengirim `CAL SAMPLE`, menunggu titik tengah stick dari ESP32, lalu menampilkan hasil. RESET CAL menjalankan kalibrasi ulang. |
| ARM / DISARM | Tombol header dan Bench Test | Mengirim perintah ARM atau DISARM. Status juga dapat diperbarui oleh field `ARM` pada `[TX]`. |
| Horizon dan drone 3D | `HorizonDroneWidget` | Menampilkan roll, pitch, compass/yaw, dan penanda target roll. Model drone memakai chase-view: yaw tidak memutar model. |
| Kartu attitude | `MetricCard` | Menampilkan roll, pitch, yaw, altitude, serta command remote. Roll/pitch juga menampilkan error. |
| Altitude dan vertical speed | `AltitudeTapeWidget` | Menampilkan altitude dan estimasi kecepatan vertikal dari perubahan altitude terhadap waktu. |
| Info sensor | `SecondaryInfoBar` | Menampilkan tegangan baterai 3S, tekanan, status kalibrasi gyro, dan estimasi frekuensi kedatangan IMU. |
| Output motor | `MotorRpmCard` | Menampilkan estimasi PWM, RPM, dan persentase output M1–M4 dari throttle, koreksi kontrol, dan baterai. |
| Monitor remote | `JoystickWidget`, `RCReadoutCard` | Menampilkan kanal R/T/Y/P pada skala 0–255 dan target fisik. Mapping CUSTOM: kiri = roll/throttle, kanan = yaw/pitch. Widget merupakan monitor, bukan joystick kontrol dengan mouse. |
| Bench Test | Diagram Quad-X dan `BenchCheckCard` | Menampilkan koreksi motor dan evaluasi otomatis IDLE/PASS/DANGER berdasarkan tanda roll/pitch/gyro Z terhadap koreksi kontrol. Halaman ini memuat petunjuk pengujian tanpa propeller. |
| Checklist bench | Indikator non-interaktif | Menampilkan kondisi baseline level, roll response, pitch response, dan yaw damping saat ini. |
| Parameter PID | Form spinbox | Mengatur angle PID, rate PID, yaw PID, batas ESC, hover throttle, max angle, dan max delta PWM. |
| Apply / Default PID | Tombol | Apply menyimpan JSON dan mengirim parameter jika terhubung. Default mengembalikan nilai form; pengiriman memerlukan Apply. |
| Grafik PID | `PidPlotWidget` | Tiga subplot: target/aktual roll, target/aktual pitch, serta uRoll/uPitch/uYaw. Jendela waktu lima detik dengan buffer maksimal 250 sampel. |
| Start / Stop Recording | Tombol dan label status | Membuat CSV, merekam saat paket `[SMC]` diterima, lalu menutup file. Menampilkan durasi, jumlah baris, dan nama file. |
| Buka Folder | Tombol recorder | Membuka direktori `records` di file explorer. |
| Panduan tuning | Teks halaman PID | Menjelaskan fungsi gain PID dan batas ESC. |
| FTC aktif/nonaktif | Tombol toggle | Mengubah mode lokal dan mengirim `FTC 1` atau `FTC 0` bila terhubung. |
| Fault motor M1–M4 | Toggle, slider, spinbox, preset | Mengatur aktivasi fault dan persentase kehilangan efektivitas motor, kemudian mengirim `FAULT`. |
| Skenario cepat FTC | Tombol | Memilih M1/M2/M3/M4 gagal 100% atau M1 kehilangan 50%. |
| Normalkan motor | Tombol FTC | Menonaktifkan seluruh fault dan mengirim keadaan baru. |
| Visualisasi dan diagnostik FTC | Diagram dan kartu status | Menampilkan hasil perhitungan lokal PWM nominal/setelah fault, kompensasi, dan selisih output untuk roll/pitch. |
| Window FTC terpisah | `FtcStandaloneWindow` | Membuka diagram dan kontrol fault dalam jendela tambahan yang memakai state aplikasi yang sama. |

**Batas interpretasi tampilan:** PWM/RPM motor dan keluaran FTC dihitung di GUI, bukan pembacaan langsung empat motor dari telemetry. Label keseimbangan FTC bukan pengukuran torsi fisik atau bukti drone stabil. Tidak terdapat pemilih baud rate, tombol Clear Plot, atau Save Data terpisah; CSV disimpan melalui fitur recording.

Layout ringkas:

```text
Header: QMBED | COM Port | Refresh | Connect | Kalibrasi | ARM | Status | Menu
                              |
                    Halaman yang dipilih
                              |
  Attitude       : Horizon + altitude tape; kartu attitude; info sensor; motor
  Control        : Dua joystick; kanal RC; banner kalibrasi
  Bench Test     : Diagram motor; tiga pemeriksaan respons; checklist
  PID Tuning     : Form parameter; recorder CSV; tiga grafik; panduan
  Fault Tolerant : Diagram/diagnostik; kartu motor 2 x 2; skenario cepat
```

## 3. Alur Komunikasi Data

Berdasarkan komentar dan pesan dalam aplikasi, jalur yang dituju adalah:

```text
Sensor → Flight Controller → LoRa → Remote ESP32
                                         |
                                   Serial / COM Port
                                         |
                              Python → Parser → GUI

Tombol GUI → Command serial → Remote ESP32 → Drone
```

Implementasi langsung di Python adalah koneksi serial ke perangkat. Detail transport firmware/LoRa dan model flight controller tidak diverifikasi oleh file ini: **TBD**.

| Parameter | Implementasi |
| --- | --- |
| Port | Pilihan pengguna melalui `_port_cb`; nomor COM aktual **TBD**. |
| Baud rate | `115200`, tetap di `_toggle_serial()`. |
| Timeout baca | `0.02` detik. |
| Format pesan | Teks ASCII bertag, diakhiri `\n`. |
| Polling serial | `_serial_timer`, interval 16 ms, nominal 62,5 pemanggilan/detik. |
| Animasi GUI | `_anim_timer`, interval 16 ms. |
| Frekuensi pengiriman firmware | **TBD**; tidak sama dengan frekuensi timer GUI. |
| Data rate yang ditampilkan | Estimasi dari selisih waktu kedatangan paket `[IMU]`, dengan smoothing. |
| Thread | Satu GUI thread; tidak ada worker serial terpisah. |

Perintah keluar, masing-masing diakhiri newline:

```text
ARM
DISARM
CAL SAMPLE
FTC 1
FTC 0
FAULT <aktif1> <loss1> <aktif2> <loss2> <aktif3> <loss3> <aktif4> <loss4>
PID <angleKp> <angleKi> <angleKd> <rateKp> <rateKi> <rateKd>
    <yawKp> <yawKi> <yawKd> <maxAngle> <maxYawRate> <maxDeltaPwm>
    <escMinPwm> <escArmSpinPwm> <escMaxPwm> <hoverThrottlePwm>
```

Command PID sebenarnya dikirim dalam **satu baris**. Nilai `maxYawRate` pada command di-hardcode `150.0`. Pada FAULT, aktivasi bernilai 0/1 dan loss bernilai 0–100%; loss 100% berarti kehilangan penuh bila fault aktif.

## 4. Format Data Telemetry

Contoh berikut menggambarkan sintaks yang diterima parser, bukan rekaman pengukuran:

```text
[IMU] AX:0.02 AY:-0.10 AZ:9.78 GX:0.15 GY:-0.20 GZ:1.35
[BMP] P:1012.80 A:1.42
[TX] R:5.2deg T:1220us Y:0.0dps P:-3.5deg ARM:1 RAW_T:140
[SMC] ROLL:1.20 PITCH:-0.75 YAW:32.40 UR:-15.20 UP:8.10 UY:-2.30
[BAT] V:11.35
[FLAGS] G:1 B:1 P:1 V:1 BS:0 FS:0
[CAL] OK CR=2048 CT=2048 CY=2048 CP=2048
[PID] OK
[FAULT] OK
[FTC] ON
```

| Tag/Field | Arti | Satuan atau nilai |
| --- | --- | --- |
| IMU: AX, AY, AZ | Percepatan tiga sumbu | Satuan tidak dinyatakan eksplisit: **TBD**. |
| IMU: GX, GY, GZ | Kecepatan sudut tiga sumbu | derajat/detik sesuai penggunaan kode. |
| BMP: P, A | Tekanan dan altitude | hPa; meter. Referensi nol altitude dari firmware: **TBD**. |
| TX: R, P | Target roll dan pitch | derajat pada format fisik; 0–255 pada format raw. |
| TX: T | Throttle | mikrodetik pada format fisik; 0–255 pada format raw. |
| TX: Y | Target yaw rate | derajat/detik pada format fisik; 0–255 pada format raw. |
| TX: ARM, RAW_T | Status arm dan raw throttle | Opsional; 0/1 dan 0–255. |
| SMC: ROLL, PITCH, YAW | Attitude aktual dari drone | derajat; YAW opsional. |
| SMC: UR, UP, UY | Koreksi kontrol roll, pitch, yaw | Ditampilkan/digunakan GUI sebagai delta PWM dalam mikrodetik. |
| BAT: V | Tegangan baterai | volt. |
| FLAGS: G, B, P, V | Gyro terkalibrasi, IMU OK, barometer OK, baterai OK | 0/1; disimpan pada `_flags`. |
| FLAGS: BS | Tahap baterai | 0=OK, 1=WARN, 2=LIMIT, 3=CRITICAL. |
| FLAGS: FS | Tahap failsafe | 0=OK, 2=DESCENT, 3=LAND sesuai komentar kode. |
| CAL: CR, CT, CY, CP | Titik tengah kanal stick dari ESP32 | Integer ADC/raw; rentang aktual **TBD**. Separator yang benar adalah `=`. |
| PID / FAULT / FTC | Respons perangkat | PID menerima `[PID] OK`, FAULT menerima `[FAULT] OK...`, FTC menerima teks apa pun setelah `[FTC]`. |

Format raw `[TX]` juga didukung, misalnya `[TX] R:128 T:128 Y:128 P:128 ARM:0`. Parser membedakan raw dan unit fisik melalui suffix, nilai throttle, atau adanya desimal pada roll/pitch.

Jika `[SMC]` tidak memuat YAW, GUI memakai `_orient.yaw` sebagai fallback. `[CAL] OK` hanya diproses ketika aplikasi sedang menunggu hasil kalibrasi.

## 5. Pemrosesan Data

```text
Byte serial → Buffer _buf → Pisah newline → Decode ASCII → Regex parser
     → Konversi angka → Simpan state → Update widget / plot / CSV
```

1. `_read_serial()` membaca byte yang tersedia dengan `read(in_waiting or 1)`.
2. Byte ditambahkan ke `_buf`. Baris yang belum lengkap tetap disimpan sampai newline datang.
3. Setiap baris di-decode menggunakan ASCII, karakter tak valid diabaikan, lalu diproses oleh `_parse_line()`.
4. Parser memakai regex per tag, mengonversi field menjadi `float` atau `int`, dan memperbarui state. Parser berhenti setelah tag pertama yang cocok; format sebaiknya satu pesan per baris.
5. `_push_att_metrics()` dan `_push_rc_values()` memasukkan data ke widget. `update()` meminta Qt menggambar ulang widget melalui `paintEvent()`.
6. Paket `[SMC]` menambahkan sampel grafik dan, ketika recording aktif, satu baris CSV. Sampel CSV menggabungkan SMC baru dengan nilai terakhir sensor/remote lain, bukan paket serentak bertimestamp dari drone.

State penting:

| Variable | Isi |
| --- | --- |
| `_serial`, `_buf` | Koneksi serial dan buffer byte yang belum selesai diproses. |
| `_orient` | Filter attitude lokal dari IMU. |
| `_bench_smc` | `(roll, pitch, yaw, uRoll, uPitch, uYaw)` dari drone. |
| `_bench_imu` | `(gx, gy, gz)` untuk bench dan CSV. |
| `_alt_smooth`, `_press`, `_vbat` | Altitude tersaring, tekanan, tegangan baterai. |
| `_js_raw`, `_js_target` | Kanal RC 0–255 dan target fisik `(roll, throttle, yaw_rate, pitch)`. |
| `_armed`, `_flags` | Status arm dan flag kesehatan perangkat. |
| `_pid_inputs` | Dictionary widget input parameter PID/ESC. |
| `_ftc_faults`, `_ftc_enabled` | Konfigurasi fault empat motor dan mode FTC lokal. |
| `_is_recording`, `_record_writer` | Status recording dan penulis CSV. |

Pengolahan utama:

- **Attitude:** display utama memakai `_bench_smc`, bukan roll/pitch hasil complementary filter lokal. `[SMC]` berisi attitude aktual dan output koreksi, bukan setpoint.
- **Filter lokal:** `OrientationFilter.update()` menggabungkan accelerometer dan integrasi gyro, memakai interval aktual, reset setelah gap panjang, dan deadband gyro Z 0,12 derajat/detik.
- **Altitude:** sampel pertama dipakai langsung; berikutnya memakai `0.80 * nilai_lama + 0.20 * nilai_baru`. Vertical speed dihitung dari perubahan altitude dan dihaluskan lagi.
- **Remote:** raw dibatasi 0–255 lalu dinormalisasi untuk posisi joystick. Beberapa konversi memakai konstanta 25 derajat, 150 derajat/detik, dan PWM 1000/1200/1300.
- **Grafik/CSV:** target roll/pitch dihitung ulang dari `_js_raw` dan `max_angle`. Dalam implementasi saat ini, rumus target pitch grafik tidak membalik tanda seperti konversi target pitch `[TX]`; target grafik dapat berbeda tanda dari command pada kartu attitude. Error grafik/CSV adalah target dikurangi aktual, sedangkan kartu metrik memakai aktual dikurangi target.
- **Motor:** mixer Quad-X menghitung M1=`T+UR+UP-UY`, M2=`T-UR+UP+UY`, M3=`T-UR-UP-UY`, M4=`T+UR-UP+UY`. Estimasi RPM memakai `1400 * Vbat * 0.75` sebagai acuan maksimum dan skala PWM.
- **FTC:** `_update_ftc()` menghitung simulasi lokal pengurangan output dan kompensasi. Saat satu motor fault, motor diagonal ikut dikurangi dan dua motor lain diberi tambahan terbatas. Nilai delta roll/pitch berasal dari selisih PWM, bukan satuan torsi N·m.

### Penyimpanan CSV

Nama file: `records/pid_record_YYYYMMDD_HHMMSS.csv`.

Kolom yang disimpan:

```text
timestamp, elapsed_sec,
roll_actual_deg, roll_target_deg, roll_error_deg,
pitch_actual_deg, pitch_target_deg, pitch_error_deg,
yaw_actual_deg, yaw_target_dps,
u_roll, u_pitch, u_yaw,
gyro_x_dps, gyro_y_dps, gyro_z_dps,
altitude_m, vbat_v, armed,
angle_kp, angle_ki, angle_kd,
rate_kp, rate_ki, rate_kd,
yaw_kp, yaw_ki, yaw_kd
```

Timestamp berasal dari PC. Gain PID yang direkam berasal dari nilai form GUI. File di-flush setiap 10 baris dan saat Stop Recording. Timer recorder 200 ms hanya memperbarui status tampilan; frekuensi penulisan mengikuti paket `[SMC]`. Menutup aplikasi menghentikan recording; Disconnect sendiri tidak menghentikan recorder.

## 6. Struktur Program

Seluruh logika aplikasi berada dalam satu file Python:

```text
drone_viewer.py
├── Import, regex telemetry, konstanta warna/font/path
├── qfont(), add_shadow(): helper tampilan
├── OrientationFilter, rot_matrix(), mv(): filter dan matematika
├── Widget monitoring attitude, altitude, remote, grafik, motor
├── Widget Bench Test dan FTC
├── MainWindow: layout, state, serial, parser, command, logging
└── main(): menjalankan event loop Qt

pid_config.json  ← konfigurasi lokal yang dimuat/disimpan aplikasi
records/         ← keluaran CSV; dibuat bila diperlukan
```

| Class | Tanggung jawab |
| --- | --- |
| `OrientationFilter` | Mengolah IMU menjadi estimasi orientasi lokal. |
| `HorizonDroneWidget`, `AltitudeTapeWidget` | Menggambar attitude, model drone, altitude, dan vertical speed. |
| `MetricCard`, `SecondaryInfoBar` | Kartu nilai, target, error, dan informasi sensor. |
| `JoystickWidget`, `RCReadoutCard`, `StatusBanner` | Visualisasi remote dan pesan kalibrasi. |
| `PidPlotWidget` | Buffer sampel dan tiga grafik custom berbasis `QPainter`. |
| `MotorRpmCard` | Estimasi output PWM/RPM empat motor. |
| `BenchMotorMixWidget`, `BenchCheckCard` | Diagram mixer dan evaluasi respons kontrol. |
| `FtcMotorMixWidget`, `MotorFaultControlCard` | Diagram hasil perhitungan FTC dan input fault per motor. |
| `FtcStandaloneWindow` | Jendela tambahan FTC. |
| `MainWindow` | Menghubungkan widget, data, timer, serial, konfigurasi, dan recorder. |

## 7. Function Penting

Method berikut berada pada `MainWindow`, kecuali yang diberi nama class lain.

| Function / Method | Fungsi |
| --- | --- |
| `main()` | Membuat `QApplication`, menampilkan window, menjalankan `app.exec()`. |
| `_build_header()`, `_build_attitude_tab()`, `_build_rc_tab()` | Membuat header dan halaman monitoring. |
| `_build_bench_tab()`, `_build_pid_tab()`, `_build_ftc_tab()` | Membuat halaman pengujian dan tuning. |
| `_show_nav_menu()`, `_switch_page()` | Mengelola perpindahan halaman. |
| `_refresh_ports()` | Memindai COM Port. |
| `_toggle_serial()` | Connect/disconnect; mencoba mengirim DISARM saat disconnect jika armed. |
| `_read_serial()` | Membaca byte, buffering, decode, dan meneruskan baris ke parser. |
| `_parse_line()` | Memproses format telemetry yang dikenali. |
| `_push_att_metrics()`, `_push_rc_values()` | Memperbarui data widget dari state aplikasi. |
| `_tick()` | Animasi dan pembaruan bench/FTC setiap interval timer. |
| `_calibrate_imu()` | Reset filter dan tampilan lokal. |
| `_start_stick_calibration()`, `_on_cal_ok()`, `_on_calib_timeout()` | Mengirim kalibrasi, menerima hasil, atau menangani timeout 2 detik. |
| `_reset_stick_calibration()` | Memicu pengambilan ulang titik tengah stick. |
| `_toggle_arm()` | Memperbarui state lokal dan mengirim ARM/DISARM. |
| `_update_gui_throttle()` | Menghitung preview throttle tersaring dari stick dan batas ESC. |
| `_update_bench()` | Memeriksa arah koreksi dan memperbarui checklist. |
| `_apply_pid_parameters()` | Menyimpan parameter dan mengirim command PID saat serial terbuka. |
| `_save_pid_config()`, `_load_pid_config()`, `_restore_pid_defaults()` | Menyimpan, memuat, dan mereset konfigurasi form. |
| `_toggle_recording()`, `_start_recording()`, `_stop_recording()` | Mengelola sesi CSV. |
| `_record_csv_sample()`, `_update_record_ui()` | Menulis sampel dan memperbarui informasi recording. |
| `_open_records_folder()` | Membuka folder CSV. |
| `_on_ftc_fault_changed()`, `_send_ftc_command()` | Menyimpan perubahan fault dan mengirim command. |
| `_on_ftc_mode_toggled()`, `_reset_all_ftc_faults()`, `_apply_ftc_scenario()` | Memilih mode FTC, kondisi normal, atau skenario fault. |
| `_update_ftc()`, `_open_ftc_standalone_window()` | Menghitung visualisasi FTC dan membuka jendela tambahan. |
| `closeEvent()` | Menghentikan recorder, menyimpan konfigurasi, menutup serial/window tambahan. Tidak mengirim DISARM seperti jalur tombol Disconnect. |
| `OrientationFilter.update()` | Complementary filter dan integrasi gyro lokal. |
| `PidPlotWidget.add_sample()` | Menambahkan sampel bertimestamp lokal ke buffer grafik. |
| `paintEvent()` pada widget | Menggambar komponen visual memakai `QPainter`. |

## 8. Hubungan Antarbagian Program

```text
main() → QApplication → MainWindow
                           |
            +--------------+----------------+
            |                               |
     QTimer serial 16 ms              QTimer animasi 16 ms
            |                               |
      _read_serial()                     _tick()
            |                               |
      _parse_line()                  Animasi + bench + FTC
            |
       State aplikasi
            |
   +--------+------------+-------------------+
   |                     |                   |
Widget attitude/RC   Grafik pada [SMC]   CSV jika recording

Input pengguna → signal Qt → handler → state lokal / command serial
```

Serial, parser, penulisan CSV, dan GUI memakai thread yang sama. Timeout 20 ms membatasi waktu tunggu baca, tetapi pembacaan tetap dapat memblokir sementara, khususnya saat `in_waiting` kosong. Karena itu interval 16 ms bukan jaminan laju aktual. Operasi yang lama pada thread ini akan menunda respons tombol dan repaint; source code saat ini tidak memakai `QThread`.

## 9. Library yang Digunakan

| Library | Penggunaan |
| --- | --- |
| `PySide6.QtWidgets` | Window, layout, tombol, menu, input angka, slider, dan halaman GUI. |
| `PySide6.QtGui` | `QPainter`, warna, font, ikon, gradient, dan membuka folder lewat `QDesktopServices`. |
| `PySide6.QtCore` | Timer, signal, konstanta Qt, geometry, dan URL lokal. |
| `serial`, `serial.tools.list_ports` | Membaca/menulis serial dan memindai perangkat; berasal dari pyserial. |
| `re` | Regex parser telemetry. |
| `math` | Filter, trigonometri, rotasi, dan proyeksi visual. |
| `time`, `datetime` | Interval, timestamp, elapsed time, dan penamaan CSV. |
| `csv` | Menulis rekaman telemetry. |
| `json`, `os` | Konfigurasi PID, path file, dan pembuatan folder records. |
| `collections.deque` | Buffer grafik dengan kapasitas terbatas. |
| `sys` | Argumen aplikasi dan exit event loop. |
| `traceback` | Menampilkan traceback bila startup/event loop mengalami exception yang tertangkap di `main()`. |

Grafik memakai `QPainter`; tidak menggunakan matplotlib atau pyqtgraph. Jika pyserial tidak tersedia, GUI dapat dibuat tetapi koneksi serial tidak tersedia.

## 10. Kesimpulan

QMBED mengelola telemetry dengan alur sederhana: **serial → parser → state aplikasi → widget, grafik, dan CSV**. Attitude utama berasal dari `[SMC]`, input remote dari `[TX]`, altitude dari `[BMP]`, dan tegangan dari `[BAT]`. GUI menyediakan kontrol konfigurasi serta pengujian, sedangkan estimasi motor dan visualisasi FTC merupakan hasil perhitungan lokal yang perlu dibedakan dari telemetry langsung flight controller.
