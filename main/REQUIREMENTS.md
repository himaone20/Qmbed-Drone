# Requirement dan Spesifikasi Teknis Flight Controller QMBED

| Atribut | Nilai |
|---|---|
| Ruang lingkup source | Folder `main/` |
| Target firmware | Quadcopter Quad-X berbasis STM32F401RCT6 |
| Platform | Arduino IDE, STM32duino, STM32FreeRTOS |
| Revisi dokumen | 1.0 |
| Dasar dokumen | Inspeksi source code `main` pada 21 September 2026 |

## 1. Tujuan, Ruang Lingkup, dan Status

Dokumen ini mendefinisikan requirement, rancangan teknis, antarmuka, batasan, dan rencana verifikasi firmware flight controller pada folder `main`. Isi dokumen diturunkan dari implementasi aktual, bukan dari target fitur yang belum diwujudkan.

Status yang digunakan:

| Status | Makna |
|---|---|
| Diimplementasikan | Ada dalam source dan dipanggil oleh alur firmware aktif. |
| Sebagian | Ada dalam source, namun belum lengkap atau belum memiliki pemicu dari antarmuka utama. |
| Dikonfigurasi saja | Konstanta atau API tersedia tetapi tidak dipakai oleh alur aktif. |
| Batasan/Risiko | Perilaku aktual yang dapat memengaruhi keselamatan, stabilitas, atau kompatibilitas. |
| Belum diverifikasi | Membutuhkan pengujian fisik atau source di luar folder `main`. |

### 1.1 Istilah

| Istilah | Arti |
|---|---|
| IMU | Inertial Measurement Unit, yaitu BMI160. |
| ESC | Electronic Speed Controller untuk motor brushless. |
| PWM | Lebar pulsa kendali ESC dalam mikrodetik. |
| PID | Proportional, Integral, Derivative controller. |
| FTC | Fault Tolerant Control. |
| Uplink | Paket perintah dari remote menuju drone. |
| Downlink | Paket telemetri dari drone menuju remote/GUI. |
| `dt` | Selang waktu aktual antar-eksekusi yang dihitung dengan `micros()`. |

### 1.2 Konvensi Satuan dan Tanda

| Besaran | Satuan | Konvensi |
|---|---|---|
| Roll, pitch, yaw | derajat | Roll positif: miring kanan. Pitch positif: nose-up. Yaw positif: putar kanan/CW. |
| Angular rate | deg/s | `gx`, `gy`, `gz` sudah ditransformasi ke body frame. |
| Akselerasi | m/s2 | Sumbu sensor: X depan, Y kiri, Z atas. |
| Tekanan | hPa | Telemetri diskalakan x10. |
| Ketinggian | meter | Relatif terhadap baseline saat boot. |
| Tegangan | volt | Telemetri diskalakan x100. |
| PWM | mikrodetik | Nilai keluaran per ESC. |

## 2. Ringkasan Sistem

### 2.1 Tujuan Sistem

Firmware harus menerima perintah pilot melalui LoRa, membaca sensor penerbangan, menghitung kendali stabilisasi, menulis PWM empat ESC, dan mengirim telemetri kembali melalui LoRa.

```text
Remote/GUI -- Uplink LoRa --> STM32 flight controller --> TIM4 PWM --> 4 ESC/motor
                                  |          ^
                                  v          |
                         BMI160, BMP180, ADC |
                                  |
                                  +-- Downlink LoRa --> Remote/GUI
```

### 2.2 Batas Ruang Lingkup

Dokumen ini mencakup `main.ino`, `config.h`, `sensors.*`, `control.*`, `motors.*`, dan `radio.*`. Source remote, GUI, skematik PCB, jenis ESC, kapasitas baterai, konfigurasi propeller, dan versi library tidak dapat dipastikan hanya dari folder ini.

### 2.3 Struktur Modul

| Modul | Tanggung jawab utama |
|---|---|
| `main.ino` | Startup perangkat dan pembuatan task FreeRTOS. |
| `config.h` | Pinout, parameter kontrol, parameter sensor, task, dan format paket. |
| `sensors.cpp` | BMI160, BMP180, ADC baterai, kalibrasi gyro, estimasi sikap, telemetri sensor. |
| `control.cpp` | Cascade PID roll/pitch, yaw rate/heading lock, parameter PID runtime. |
| `motors.cpp` | TIM4 PWM, throttle vertikal, Quad-X mixer, desaturasi, FTC. |
| `radio.cpp` | LoRa RA-02, penerimaan command/configuration, pengiriman telemetri. |

## 3. Kebutuhan Perangkat Keras dan Perangkat Lunak

### 3.1 Platform dan Dependensi

| Item | Requirement | Status |
|---|---|---|
| Mikrokontroler | STM32F401RCT6 atau board Generic F401RCTx yang kompatibel. | Diimplementasikan |
| Framework | Arduino STM32duino dan STM32FreeRTOS. | Diimplementasikan |
| Radio | RA-02 berbasis SX1278 dengan library `LoRa`. | Diimplementasikan |
| IMU | BMI160 pada I2C alamat `0x68`. | Diimplementasikan |
| Barometer | BMP180 pada I2C alamat `0x77`. | Diimplementasikan |
| Baterai | Pembagi tegangan untuk LiPo 3S menuju ADC. | Diimplementasikan |
| Motor | Empat ESC PWM dan konfigurasi Quad-X. | Diimplementasikan |

Catatan: komentar lama yang menyebut BMP280 tidak sesuai dengan driver aktif. Implementasi melakukan inisialisasi, pembacaan kalibrasi, dan state machine khusus BMP180.

### 3.2 Pinout

| Fungsi | Pin STM32 | Antarmuka | Detail |
|---|---|---|---|
| Serial TX | PA9 | USART1 | Debug, 115200 baud. |
| Serial RX | PA10 | USART1 | Didefinisikan, tidak dipakai dalam alur utama. |
| LoRa SCK | PB13 | SPI2 | RA-02. |
| LoRa MISO | PB14 | SPI2 | RA-02. |
| LoRa MOSI | PB15 | SPI2 | RA-02. |
| LoRa NSS | PB12 | GPIO/SPI | Chip select RA-02. |
| LoRa reset | PB1 | GPIO | Reset RA-02. |
| LoRa DIO0 | PB0 | GPIO | Interrupt/status RA-02. |
| I2C SCL | PB10 | I2C | Clock 400 kHz. |
| I2C SDA | PB3 | I2C | Data. |
| Tegangan baterai | PA0 | ADC 12-bit | Pembagi R1 100 kohm dan R2 20 kohm. |
| Motor M1 | PB6 | TIM4 CH1 | Depan-kiri, CW. |
| Motor M2 | PB7 | TIM4 CH2 | Depan-kanan, CCW. |
| Motor M3 | PB8 | TIM4 CH3 | Belakang-kanan, CW. |
| Motor M4 | PB9 | TIM4 CH4 | Belakang-kiri, CCW. |
| LED | PC4 | GPIO | Aktif LOW; hidup ketika armed dan IMU valid. |

### 3.3 Konfigurasi Fisik Quad-X

```text
                 DEPAN
             M1        M2
           FL, CW     FR, CCW
                 X
             M4        M3
           BL, CCW    BR, CW
                BELAKANG
```

IMU dipasang dengan X menuju depan, Y menuju kiri, dan Z menuju atas. Firmware membalik tanda gyro Y dan gyro Z untuk menghasilkan konvensi pitch dan yaw pada Bagian 1.2.

## 4. Arsitektur Eksekusi dan Timing

### 4.1 Urutan Startup

1. UART debug dikonfigurasi pada PA9/PA10 dengan 115200 baud.
2. Motor dan LED diinisialisasi; semua ESC disetel ke PWM minimum.
3. Mutex PID dan state cascade PID diinisialisasi.
4. ADC baterai, I2C, BMI160, kalibrasi gyro, BMP180, dan baseline altitude diinisialisasi.
5. LoRa SPI2 diinisialisasi. Jika gagal, firmware berhenti di loop tanpa akhir.
6. `sensorMutex` dibuat.
7. Task motor, sensor legacy, dan LoRa dibuat, lalu scheduler FreeRTOS dimulai.

### 4.2 Task dan Sinkronisasi

| Task | Prioritas | Stack | Periode nominal | Fungsi aktual |
|---|---:|---:|---:|---|
| `TaskMotors` | 4 | 256 words | 5 ms | IMU, attitude, vertical control, PID, mixer, PWM, polling telemetry. |
| `TaskSensors` | 3 | 256 words | 100 ms aktual | Placeholder; hanya `vTaskDelay(100 ms)`. |
| `TaskLoRa_Control` | 2 | 384 words | 5 ms | Poll paket LoRa, command/config, downlink telemetry. |

`sensorMutex` melindungi `gSensorData` saat task motor menyalin IMU atau memperbarui telemetry, serta saat task LoRa mengambil snapshot. Mutex PID melindungi perubahan `PidParams` yang diterima melalui LoRa.

### 4.3 Frekuensi dan Periode Sampling

Semua angka berikut adalah target nominal dari konfigurasi/kode, bukan hasil profiling perangkat keras.

| ID | Aktivitas | Lokasi | Frekuensi nominal | Periode nominal | Status |
|---|---|---|---:|---:|---|
| REQ-TIME-001 | Sampling accelerometer dan gyro BMI160 | `sensors_step_imu()` | 200 Hz | 5 ms | Diimplementasikan |
| REQ-TIME-002 | Estimasi roll, pitch, yaw | `sensors_step_imu()` | 200 Hz | 5 ms | Diimplementasikan |
| REQ-TIME-003 | Cascade PID | `computeCascadePid()` | 200 Hz | 5 ms | Diimplementasikan |
| REQ-TIME-004 | Kendali kolektif vertikal | `updateVerticalCollective()` | 200 Hz | 5 ms | Diimplementasikan |
| REQ-TIME-005 | Pembaruan perintah PWM motor | `writeMotorMix()` | 200 Hz | 5 ms | Diimplementasikan |
| REQ-TIME-006 | Carrier PWM ESC TIM4 | `HardwareTimer` | 250 Hz | 4 ms | Diimplementasikan |
| REQ-TIME-007 | Polling LoRa | `TaskLoRa_Control()` | 200 Hz | 5 ms | Diimplementasikan |
| REQ-TIME-008 | Sampling ADC baterai | `sensors_poll_telemetry()` | 10 Hz | 100 ms | Diimplementasikan |
| REQ-TIME-009 | Sampling tekanan/altitude BMP180 | `bmp180_poll()` | Maksimum 20 Hz | Minimum 50 ms | Diimplementasikan |
| REQ-TIME-010 | Sampling temperatur BMP180 | `bmp180_poll()` | 0.1 Hz | 10 s | Diimplementasikan |
| REQ-TIME-011 | Sampel gyro ketika kalibrasi boot | `sensors_init()` | Sekitar 333 Hz | 3 ms antar sampel | Diimplementasikan |
| REQ-TIME-012 | Task sensor legacy | `TaskSensors()` | 10 Hz | 100 ms | Placeholder |

`dt` kontrol dan attitude dihitung dari `micros()`. Nilai di luar 0.5 ms sampai 50 ms digantikan dengan 5 ms. Frekuensi aktual dapat berubah akibat penjadwalan, waktu I2C/SPI, dan `Serial.print()`.

### 4.4 Siklus Kendali Nominal 5 ms

1. `TaskMotors` membaca BMI160 dan menghitung attitude.
2. Jika tidak armed atau IMU gagal, motor dihentikan, PID dan vertical-control di-reset.
3. Jika armed dan IMU valid, throttle kolektif dihitung, cascade PID dihitung, lalu mixer menulis PWM motor.
4. Telemetri barometer dan baterai dipoll tanpa menunggu konversi sensor.
5. Snapshot IMU terbaru disalin ke `gSensorData` jika mutex tersedia.

## 5. Requirement Fungsional

### REQ-BOOT-001 - Startup Aman

**Status:** Diimplementasikan. **Sumber:** `main.ino::setup()`, `motors.cpp::motors_init()`.

Firmware harus mengatur seluruh motor ke PWM minimum sebelum scheduler berjalan. Kegagalan LoRa saat inisialisasi harus menghentikan startup. Kegagalan inisialisasi sensor hanya dicetak sebagai peringatan; task motor kemudian menahan motor di kondisi aman jika BMI160 tidak valid.

**Kriteria penerimaan:** sebelum command armed yang valid dan pembacaan IMU berhasil, semua `gMotorPWM` bernilai PWM minimum.

### REQ-SENS-001 - BMI160 dan Kalibrasi Gyro

**Status:** Diimplementasikan. **Sumber:** `sensors.cpp::bmi160_init()`, `sensors_init()`.

BMI160 harus diverifikasi dengan chip ID `0xD1`, dikonfigurasi pada accelerometer 200 Hz +/-2 g dan gyro 200 Hz +/-2000 deg/s. Saat boot, firmware harus mengambil 250 sampel gyro per percobaan, maksimal tiga percobaan. Kalibrasi valid bila jumlah sampel valid minimal 50 dan total varians tiga sumbu tidak melebihi 5.0 dps2.

Jika kalibrasi gagal, bias diatur nol dan `gyroCalibValid` bernilai false; IMU tetap dapat dipakai.

### REQ-SENS-002 - Estimasi Sikap

**Status:** Diimplementasikan. **Sumber:** `sensors.cpp::sensors_step_imu()`.

Firmware harus mengonversi data accelerometer ke m/s2 dan gyro ke deg/s, mengurangi bias gyro, menerapkan filter gyro/accelerometer, lalu menghasilkan roll dan pitch dengan complementary filter. Konstanta complementary filter aktual menggunakan tau 0.50 s yang dihitung dinamis dari `dt`. Yaw dihitung dari integrasi gyro Z dan dibungkus ke rentang -180 sampai +180 derajat.

### REQ-SENS-003 - BMP180, Altitude, dan Tegangan Baterai

**Status:** Diimplementasikan. **Sumber:** `sensors.cpp::bmp180_poll()`, `sensors_poll_telemetry()`.

BMP180 harus menggunakan pembacaan non-blocking dengan state machine. Pressure valid berada pada 300--1200 hPa. Altitude relatif adalah hasil altitude terfilter dikurangi baseline median hingga 15 sampel saat boot. ADC baterai harus memakai referensi 3.3 V, resolusi 12-bit, dan rasio pembagi 6.0.

Tegangan harus difilter dan diklasifikasikan menjadi `BATT_OK`, `BATT_WARN` di bawah 10.8 V, `BATT_LIMIT` di bawah 10.2 V, atau `BATT_CRITICAL` di bawah 9.9 V, dengan hysteresis 500 ms.

### REQ-RAD-001 - Command dan Telemetri LoRa

**Status:** Diimplementasikan. **Sumber:** `radio.cpp::TaskLoRa_Control()`.

Firmware harus menerima `UplinkPacket` dengan magic `0xA5`, memperbarui roll, pitch, yaw rate, throttle, dan state armed. Setiap uplink valid harus dibalas dengan satu `DownlinkPacket` setelah snapshot telemetry tersedia.

LoRa menggunakan 433 MHz, spreading factor 7, bandwidth 125 kHz, dan coding rate 4/5. Paket dengan ukuran atau magic tidak sesuai harus diabaikan dan radio harus kembali ke mode receive.

### REQ-RAD-002 - Konfigurasi Runtime

**Status:** Diimplementasikan. **Sumber:** `radio.cpp::TaskLoRa_Control()`.

Firmware harus menerima `ConfigPacket` magic `0xC3` untuk mengganti parameter PID, batas ESC, dan hover throttle pada RAM. Batas validasi ESC adalah min 900--1400 us, arm-spin 1000--1600 us, dan max 1100--2200 us. Konfigurasi tidak disimpan ke flash, sehingga hilang saat restart.

### REQ-ARM-001 - Arming, Disarming, dan LED

**Status:** Diimplementasikan. **Sumber:** `motors.cpp::TaskMotors()`.

Motor hanya boleh menjalankan loop kontrol ketika `gArmedCmd` true dan pembacaan BMI160 berhasil. Selain kondisi tersebut, seluruh motor harus di-set ke `gEscMinPwm`, LED harus mati, dan state PID serta vertical control harus di-reset. Ketika armed dan IMU valid, LED aktif LOW harus menyala.

### REQ-CTRL-001 - Cascade PID Roll dan Pitch

**Status:** Diimplementasikan. **Sumber:** `control.cpp::computeCascadePid()`.

Firmware harus menggunakan outer loop sudut untuk menghasilkan desired angular rate dan inner loop rate untuk menghasilkan delta PWM. Outer loop memakai proportional, integral trapezoidal opsional, dan derivative dari rate gyro. Inner loop memakai derivative-on-measurement dengan low-pass filter.

PID harus reset dan output roll/pitch/yaw harus nol ketika throttle kurang dari 1150 us. Integrator hanya aktif ketika throttle minimal PWM arm-spin.

### REQ-CTRL-002 - Yaw Rate dan Heading Lock

**Status:** Diimplementasikan. **Sumber:** `control.cpp::computeCascadePid()`.

Jika nilai absolut target yaw rate melebihi 3 deg/s, firmware harus memakai rate mode dan melepas heading lock. Jika tidak, heading saat ini harus dikunci dan error yaw circular harus diubah menjadi desired rate. Koreksi yaw berikutnya dihitung oleh inner rate PID.

### REQ-CTRL-003 - Throttle dan Damping Vertikal

**Status:** Diimplementasikan. **Sumber:** `motors.cpp::updateVerticalCollective()`.

Input throttle 0--255 harus diperlakukan sebagai stik self-centering dengan titik tengah 128. Firmware harus menerapkan deadband, expo, slew target vertical velocity, proyeksi akselerasi terhadap world-up, low-pass, velocity leak, damping terbatas, hover adaptation, dan collective slew-rate.

Hanya perintah throttle 0--3 yang memotong motor langsung ke PWM minimum. Perintah turun parsial tetap diperlakukan sebagai descent. Kendali ini adalah damping jangka pendek berbasis IMU, bukan altitude hold absolut.

### REQ-MOTOR-001 - Mixer Quad-X dan Desaturasi

**Status:** Diimplementasikan. **Sumber:** `motors.cpp::writeMotorMix()`.

Firmware harus menghitung PWM motor dari base throttle dan koreksi PID:

```text
M1 = base + uRoll + uPitch - uYaw
M2 = base - uRoll + uPitch + uYaw
M3 = base - uRoll - uPitch - uYaw
M4 = base + uRoll - uPitch + uYaw
```

Jika output melebihi PWM maksimum, semua motor harus dikurangi dengan nilai overflow. Jika output kurang dari arm-spin ketika base throttle sudah arm-spin atau lebih, semua motor harus dinaikkan dengan nilai underflow. Output akhir harus dibatasi pada `gEscMinPwm` hingga `gEscMaxPwm`.

### REQ-MOTOR-002 - FTC dan Injeksi Gangguan Motor

**Status:** Sebagian. **Sumber:** `motors.cpp::setMotorFault()`, `writeMotorMix()`.

Firmware menyediakan API untuk menandai degradasi 0--100% pada satu atau lebih motor. Jika FTC aktif dan tepat satu motor mengalami fault lebih dari 5%, yaw dinolkan dan koreksi roll/pitch direalokasi. Setelah mixer, output motor fault dikurangi sesuai faktor kehilangan efektivitas.

Tidak ditemukan antarmuka LoRa atau pemicu otomatis yang memanggil API ini dari alur utama.

## 6. Desain Algoritma dan Parameter Utama

### 6.1 Filter dan Kendali

| Mekanisme | Nilai/Perilaku aktual |
|---|---|
| Filter gyro | `0.15 * sebelumnya + 0.85 * pembacaan`. |
| Filter accelerometer | `0.40 * sebelumnya + 0.60 * pembacaan`. |
| Complementary filter | Tau 0.50 s, alpha = tau/(tau + dt). |
| Deadband gyro | 0.10 deg/s pada tiap sumbu. |
| Integral PID maksimum | 80.0. |
| Desired angular rate maksimum roll/pitch | 250 deg/s. |
| Delta PWM maksimum default | 300 us per sumbu. |
| Filter D-term PID | Tau 0.008 s. |
| Heading-lock yaw outer-loop | Kp 2.50, Ki 0.02, Kd 0.00. |

### 6.2 Parameter Default PID dan ESC

| Parameter | Default |
|---|---:|
| Angle Kp, Ki, Kd | 5.00, 0.05, 0.12 |
| Rate Kp, Ki, Kd | 1.60, 0.30, 0.045 |
| Yaw rate Kp, Ki, Kd | 2.00, 0.15, 0.00 |
| Max angle | 25 deg |
| Max yaw rate | 150 deg/s |
| ESC minimum | 1000 us |
| ESC arm-spin | 1200 us |
| ESC maksimum | 1300 us |
| Hover throttle awal | 1260 us |

### 6.3 Parameter Kendali Vertikal

| Parameter | Nilai |
|---|---:|
| Throttle center/deadband | 128 / 8 unit stick |
| Target vertical speed maksimum | 0.60 m/s |
| Expo stick | 0.45 |
| Slew target vertical speed | 0.80 m/s2 |
| LPF percepatan vertikal | 0.12 |
| Batas percepatan vertikal | 4.0 m/s2 |
| Batas estimated vertical speed | 2.0 m/s |
| Damping acceleration | 5.0 us per m/s2 |
| Collective slew rate | 700 us/s |

## 7. Antarmuka dan Protokol

### 7.1 UART dan I2C

UART debug menggunakan USART1 115200 baud. Pesan meliputi hasil inisialisasi, kalibrasi, status radio, command yang diterima, motor PWM, PID output, dan tegangan baterai. I2C berjalan pada 400 kHz untuk BMI160 dan BMP180.

### 7.2 Paket LoRa Biner

Struktur memakai `#pragma pack(push, 1)`. Firmware dan remote/GUI harus sepakat pada urutan field, ukuran tipe C/C++, skala, signedness, dan endianness platform. Implementasi tidak menambahkan checksum, nomor urut, atau autentikasi.

#### UplinkPacket

| Offset | Field | Tipe | Skala/rentang |
|---:|---|---|---|
| 0 | `magic` | `uint8_t` | Harus `0xA5`. |
| 1 | `targetRoll` | `int16_t` | deg x100, -25.00 hingga +25.00. |
| 3 | `targetPitch` | `int16_t` | deg x100, -25.00 hingga +25.00. |
| 5 | `targetYaw` | `int16_t` | deg/s x100, -150.00 hingga +150.00. |
| 7 | `targetThrottle` | `uint16_t` | Command stick 0--255. |
| 9 | `armed` | `uint8_t` | 0 disarm, 1 arm. |

Ukuran aktual: **10 byte**.

#### DownlinkPacket

| Offset | Field | Tipe | Skala |
|---:|---|---|---|
| 0 | `magic` | `uint8_t` | `0x5A`. |
| 1--6 | `ax`, `ay`, `az` | 3 x `int16_t` | m/s2 x100. |
| 7--12 | `gx`, `gy`, `gz` | 3 x `int16_t` | deg/s x100. |
| 13 | `press` | `uint16_t` | hPa x10. |
| 15 | `alt` | `int16_t` | meter x100. |
| 17--22 | `roll`, `pitch`, `yaw` | 3 x `int16_t` | deg x100. |
| 23--28 | `uRoll`, `uPitch`, `uYaw` | 3 x `int16_t` | PWM delta us x100. |
| 29 | `vbat` | `uint16_t` | volt x100. |
| 31 | `flags` | `uint8_t` | Lihat tabel flags. |

Ukuran aktual: **32 byte**. Komentar source yang menyebut 30 byte tidak benar.

#### ConfigPacket

| Field | Tipe | Keterangan |
|---|---|---|
| `magic` | `uint8_t` | Harus `0xC3`. |
| `angleKp..angleKd` | 3 x `float` | PID angle roll/pitch. |
| `rateKp..rateKd` | 3 x `float` | PID rate roll/pitch. |
| `yawKp..yawKd` | 3 x `float` | PID rate yaw. |
| `maxAngle`, `maxYawRate`, `maxDeltaPwm` | 3 x `float` | Batas kontrol. |
| `escMinPwm`, `escArmSpinPwm`, `escMaxPwm` | 3 x `float` | Batas ESC. |
| `hoverThrottlePwm` | `float` | Target hover awal. |

Ukuran aktual: **65 byte** (1 byte magic ditambah 16 `float` x 4 byte).

### 7.3 Flags Downlink

| Bit | Nama | Arti |
|---:|---|---|
| 0 | `gyroCalibValid` | Kalibrasi gyro boot valid. |
| 1 | `bmiOK` | BMI160 terdeteksi/valid. |
| 2 | `bmpOK` | BMP180 terdeteksi/valid. |
| 3 | `vbatOK` | Tegangan terbaca di atas 3.0 V. |
| 4--5 | `battStage` | 0 OK, 1 WARN, 2 LIMIT, 3 CRITICAL. |
| 6--7 | `failsafeStage` | Dikomentari dalam source, tetapi selalu nol karena tidak pernah diisi. |

## 8. Kebutuhan Nonfungsional

| ID | Requirement | Status |
|---|---|---|
| NFR-TIME-001 | Loop IMU, kontrol, mixer, dan motor ditargetkan tiap 5 ms. | Diimplementasikan sebagai target nominal |
| NFR-DATA-001 | Snapshot telemetri bersama harus memakai `sensorMutex` bila mutex dapat diperoleh. | Diimplementasikan |
| NFR-DATA-002 | Perubahan PID runtime harus dilindungi `pidMutex` bila tersedia. | Diimplementasikan |
| NFR-DIAG-001 | Firmware harus memberi diagnosis startup dan command via serial. | Diimplementasikan |
| NFR-PERSIST-001 | Konfigurasi runtime tidak dipersistenkan setelah reset/power-cycle. | Batasan |
| NFR-SEC-001 | Paket radio tidak memiliki checksum, sequence number, atau autentikasi. | Batasan/Risiko |

## 9. Keselamatan, Batasan, dan Risiko

### 9.1 Prosedur Operasional Minimum

1. Lepaskan propeller untuk bench test awal.
2. Pastikan frame diam selama kalibrasi gyro boot; variance tinggi menyebabkan kalibrasi tidak valid.
3. Pastikan LoRa dan sensor terinisialisasi sebelum uji arming.
4. Uji respons roll, pitch, dan yaw dengan motor tanpa propeller sebelum penerbangan.
5. Untuk uji hover pertama, gunakan tali pengaman/tether dan area aman.

### 9.2 Batasan dan Risiko yang Ditemukan

| ID | Temuan | Dampak |
|---|---|---|
| LIM-001 | Timeout LoRa >3 s hanya mengubah `linkOK` dan mencetak log. | Tidak ada auto-disarm/failsafe motor saat link putus. |
| LIM-002 | Status baterai hanya ditransmisikan. | Tidak ada pembatasan throttle atau emergency landing otomatis. |
| LIM-003 | Yaw hanya menggunakan integrasi gyro. | Heading lock dapat drift tanpa magnetometer. |
| LIM-004 | Vertical control memakai accelerometer dengan velocity leak. | Bukan altitude hold absolut dan sensitif terhadap drift/akselerasi. |
| LIM-005 | Watchdog BMI160 dan parameter ZUPT tersedia di `config.h`. | Belum dipakai pada alur sensor aktif. |
| LIM-006 | `CF_ALPHA`, `BARO_EMA_ALPHA`, dan `ALT_SLEW_MAX_MPS` didefinisikan. | Tidak dipakai langsung oleh algoritma aktif. |
| LIM-007 | `TaskSensors` dibuat dengan prioritas 3. | Tidak melakukan pembacaan sensor; IMU sebenarnya ada di `TaskMotors`. |
| LIM-008 | `failsafeStage` dikomentari dalam format flags. | Tidak ada implementasi pembentukan stage tersebut. |
| LIM-009 | FTC hanya diekspos sebagai API internal. | Tidak ada trigger radio/diagnostik otomatis pada firmware ini. |
| LIM-010 | Debug serial dilakukan pada jalur LoRa. | Dapat menambah jitter periode aktual saat traffic tinggi. |

### 9.3 Informasi Belum Diverifikasi

| Informasi | Alasan |
|---|---|
| Jenis/firmware ESC dan kebutuhan kalibrasinya | Tidak ada pada source `main`. |
| Kapasitas, C-rating, dan konfigurasi fisik baterai | Hanya asumsi LiPo 3S yang tercantum dalam konfigurasi. |
| Versi library/board package | Tidak ada berkas proyek Arduino atau lockfile pada folder `main`. |
| Kompatibilitas byte order remote/GUI | Bergantung pada implementasi di luar folder `main`. |
| Respons stabilitas dan latency aktual | Memerlukan pengukuran bench/flight. |

## 10. Rencana Verifikasi

| Test ID | Requirement | Metode | Hasil lulus |
|---|---|---|---|
| TEST-BOOT-001 | REQ-BOOT-001 | Nyalakan board tanpa uplink arm. | Semua motor berada di PWM minimum. |
| TEST-SENS-001 | REQ-SENS-001 | Diamkan drone saat boot dan baca serial/downlink flags. | BMI valid dan `gyroCalibValid` bernilai 1. |
| TEST-SENS-002 | REQ-SENS-002 | Miringkan frame kanan/kiri serta nose-up/down. | Tanda roll/pitch sesuai konvensi dan kembali mendekati nol saat datar. |
| TEST-BARO-001 | REQ-SENS-003 | Amati telemetry barometer dan baterai. | Pressure/altitude/tegangan masuk rentang masuk akal dan status terbarui. |
| TEST-RAD-001 | REQ-RAD-001 | Kirim uplink valid, invalid magic, dan ukuran invalid. | Uplink valid dibalas downlink; paket invalid diabaikan. |
| TEST-RAD-002 | REQ-RAD-002 | Kirim config valid dan reset board. | Parameter diterapkan saat runtime serta kembali default setelah reset. |
| TEST-ARM-001 | REQ-ARM-001 | Ubah armed dan simulasi IMU gagal. | Disarm/IMU gagal menghentikan motor dan mematikan LED. |
| TEST-CTRL-001 | REQ-CTRL-001 | Bench test tanpa propeller sambil memiringkan frame. | Perbedaan PWM melawan arah kemiringan. |
| TEST-CTRL-002 | REQ-CTRL-002 | Putar yaw kanan/kiri dengan yaw stick netral dan aktif. | Rate mode aktif saat stick bergerak; damping/heading lock saat netral. |
| TEST-MOTOR-001 | REQ-MOTOR-001 | Catat empat PWM untuk command R/P/Y terpisah. | Pola PWM mengikuti persamaan Quad-X dan batas ESC. |
| TEST-FTC-001 | REQ-MOTOR-002 | Panggil API fault lewat harness khusus. | Yaw direlaksasi dan output motor fault berkurang sesuai persen loss. |

Buktilah setiap pengujian dengan versi firmware, konfigurasi parameter, log serial/downlink, kondisi fisik, hasil, dan keputusan lulus/gagal. Semua test motor awal wajib tanpa propeller.

## 11. Keterlacakan Source

| Area | File/Fungsi utama |
|---|---|
| Startup dan scheduler | `main.ino::setup()` |
| Konfigurasi dan paket | `config.h` |
| BMI160, BMP180, ADC, telemetry sensor | `sensors.cpp` |
| PID | `control.cpp::computeCascadePid()` |
| PWM, collective, mixer, FTC | `motors.cpp::TaskMotors()`, `writeMotorMix()` |
| LoRa command, config, telemetry | `radio.cpp::TaskLoRa_Control()` |

## 12. Riwayat Dokumen

## Lampiran A. Inventaris Konfigurasi `config.h`

Tabel ini mencakup seluruh konstanta konfigurasi. Kolom penggunaan menyatakan pemakaian pada jalur firmware aktif.

| Kelompok | Konstanta | Nilai default | Penggunaan |
|---|---|---|---|
| Serial | `SERIAL_TX_PIN`, `SERIAL_RX_PIN`, `SERIAL_BAUD` | PA9, PA10, 115200 | Aktif di `setup()`. |
| LoRa | `LORA_SCK_PIN`, `LORA_MISO_PIN`, `LORA_MOSI_PIN` | PB13, PB14, PB15 | Aktif saat membuat SPI2. |
| LoRa | `LORA_NSS_PIN`, `LORA_RST_PIN`, `LORA_DIO0_PIN` | PB12, PB1, PB0 | Aktif di `radio_init()`. |
| LoRa | `LORA_FREQUENCY`, `LORA_SPREADING_FACTOR` | 433E6, 7 | Aktif di `radio_init()`. |
| LoRa | `LORA_SIGNAL_BANDWIDTH`, `LORA_CODING_RATE` | 125E3, 5 | Aktif di `radio_init()`, coding rate 4/5. |
| I2C | `I2C_SCL_PIN`, `I2C_SDA_PIN`, `I2C_CLOCK_SPEED` | PB10, PB3, 400000 | Aktif di `sensors_init()`. |
| I2C | `BMI160_ADDR`, `BMP180_ADDR`, `BMP280_ADDR` | 0x68, 0x77, 0x77 | BMI160 dan BMP180 aktif; `BMP280_ADDR` hanya alias. |
| Baterai | `VBAT_PIN`, `VBAT_ADC_REF`, `VBAT_ADC_RES` | PA0, 3.3, 4095 | Aktif pada pembacaan ADC. |
| Baterai | `VBAT_R1`, `VBAT_R2`, `VBAT_DIV_RATIO`, `VBAT_CAL_FACTOR` | 100000, 20000, 6.0, 1.0 | Aktif pada perhitungan tegangan. |
| Motor | `MOTOR1_PIN` hingga `MOTOR4_PIN` | PB6, PB7, PB8, PB9 | Aktif pada konfigurasi TIM4 PWM. |
| Motor | `ESC_MIN_US`, `ESC_ARM_SPIN_US`, `ESC_MAX_US` | 1000, 1200, 1300 us | Default aktif; dapat berubah sementara melalui config LoRa. |
| PID | `PID_ANGLE_KP_DEFAULT`, `PID_ANGLE_KI_DEFAULT`, `PID_ANGLE_KD_DEFAULT` | 5.00, 0.05, 0.12 | Default aktif pada cascade PID. |
| PID | `PID_RATE_KP_DEFAULT`, `PID_RATE_KI_DEFAULT`, `PID_RATE_KD_DEFAULT` | 1.60, 0.30, 0.045 | Default aktif pada cascade PID. |
| PID | `PID_MAX_ANGLE_DEG`, `PID_MAX_RATE_DPS` | 25 deg, 250 deg/s | `MAX_RATE` aktif; `MAX_ANGLE` sebagai default runtime config. |
| PID | `PID_MAX_DELTA_PWM`, `PID_INTEGRAL_MAX`, `PID_TAU_FILTER` | 300 us, 80, 0.008 s | Aktif pada cascade PID. |
| PID | `MIX_ACTIVE_MIN_US` | 1150 us | Aktif sebagai ambang reset PID. |
| Vertikal | `VERTICAL_THROTTLE_CENTER`, `VERTICAL_THROTTLE_DEADBAND` | 128, 8 | Aktif. |
| Vertikal | `VERTICAL_MAX_TARGET_SPEED_MPS`, `VERTICAL_STICK_EXPO`, `VERTICAL_TARGET_SLEW_MPS2` | 0.60, 0.45, 0.80 | Aktif. |
| Vertikal | `VERTICAL_ACCEL_LPF_ALPHA`, `VERTICAL_MAX_ACCEL_MPS2` | 0.12, 4.0 | Aktif. |
| Vertikal | `VERTICAL_VELOCITY_LPF_ALPHA`, `VERTICAL_VELOCITY_LEAK_PER_S`, `VERTICAL_MAX_ESTIMATED_SPEED_MPS` | 0.10, 0.80, 2.0 | Aktif. |
| Vertikal | `VERTICAL_ACCEL_DAMP_US_PER_MPS2`, `VERTICAL_KP_FULL_ERROR_FRACTION`, `VERTICAL_OUTPUT_LIMIT_FRACTION` | 5.0, 0.18, 0.22 | Aktif. |
| Hover | `HOVER_THROTTLE_INITIAL_US`, `HOVER_THROTTLE_MARGIN_US` | 1260, 0 us | Aktif. |
| Hover | `HOVER_ADAPT_RATE_US_PER_S`, `HOVER_ADAPT_MAX_VZ_MPS`, `HOVER_ADAPT_MAX_ACCEL_MPS2`, `HOVER_ADAPT_MAX_ATTITUDE_DEG` | 0.25, 0.10, 0.35, 10 | Aktif. |
| Vertikal | `COLLECTIVE_SLEW_US_PER_S` | 700 us/s | Aktif. |
| Debug | `VERTICAL_DEBUG`, `VERTICAL_DEBUG_PERIOD_MS` | 0, 200 ms | Kode debug ada, nonaktif secara default. |
| LED | `LED_PIN` | PC4 | Aktif pada `TaskMotors`. |
| FreeRTOS | `TASK_SENSOR_PERIOD_MS`, `TASK_SENSOR_PRIORITY`, `TASK_SENSOR_STACK_SIZE` | 5 ms, 3, 256 words | Periode tidak dipakai oleh task legacy; priority/stack aktif saat create task. |
| FreeRTOS | `TASK_LORA_PERIOD_MS`, `TASK_LORA_PRIORITY`, `TASK_LORA_STACK_SIZE` | 5 ms, 2, 384 words | Aktif. |
| FreeRTOS | `TASK_MOTOR_PERIOD_MS`, `TASK_MOTOR_PRIORITY`, `TASK_MOTOR_STACK_SIZE` | 5 ms, 4, 256 words | Aktif. |
| Barometer | `BARO_EMA_ALPHA`, `ALT_SLEW_MAX_MPS` | 0.20, 2.5 m/s | Dikonfigurasi saja, tidak dipakai langsung. |
| Barometer | `VZ_ACCEL_WEIGHT`, `VZ_BARO_WEIGHT`, `ALT_VZ_DEADBAND` | 0.75, 0.25, 0.06 m/s | Aktif saat memperbarui telemetry `vz`. |
| Attitude | `CF_ALPHA` | 0.98 | Dikonfigurasi saja; alpha aktual dihitung dari tau 0.50 s. |
| Sensor | `SENSOR_FAIL_THRESHOLD`, `SENSOR_RECOVER_COUNT` | 5, 5 | Dikonfigurasi saja; tidak ada watchdog/recovery aktif. |
| Kalibrasi | `GYRO_CALIB_MAX_RETRY`, `GYRO_CALIB_VARIANCE_MAX`, `GYRO_CALIB_RETRY_DELAY_MS` | 3, 5.0 dps2, 500 ms | Aktif saat boot. |
| ZUPT | `ZUPT_ACCEL_TOLERANCE`, `ZUPT_GYRO_MAGNITUDE_MAX`, `ZUPT_STABLE_SAMPLES`, `ZUPT_BIAS_LEARN_RATE` | 0.3, 1.5, 300, 0.05 | Dikonfigurasi saja; ZUPT tidak diimplementasikan. |
| Baterai | `VBAT_WARN_V`, `VBAT_LIMIT_V`, `VBAT_CRITICAL_V`, `VBAT_HYSTERESIS_MS` | 10.8, 10.2, 9.9 V, 500 ms | Aktif untuk status telemetry. |
| Protokol | `UPLINK_MAGIC`, `DOWNLINK_MAGIC`, `CONFIG_MAGIC` | 0xA5, 0x5A, 0xC3 | Aktif. |

`PidParams`, `BattStage`, `UplinkPacket`, `DownlinkPacket`, `ConfigPacket`, dan `SensorData` adalah kontrak data yang juga didefinisikan dalam `config.h`. Detail paket dan flags terdapat pada Bagian 7; `SensorData` memuat akselerasi, gyro, pressure, altitude, vertical speed, attitude, yaw rate, tegangan, status sensor, status kalibrasi, serta tahap baterai.

## Lampiran B. Requirement ke Implementasi

| Requirement | Implementasi utama | Parameter/kontrak terkait |
|---|---|---|
| REQ-BOOT-001 | `setup()`, `motors_init()`, `TaskMotors()` | `ESC_MIN_US`, `LED_PIN` |
| REQ-SENS-001 | `bmi160_init()`, `sensors_init()` | alamat BMI160, parameter kalibrasi gyro |
| REQ-SENS-002 | `sensors_step_imu()` | filter sensor, `dt` |
| REQ-SENS-003 | `bmp180_poll()`, `sensors_poll_telemetry()` | parameter ADC, barometer, baterai |
| REQ-RAD-001 | `radio_init()`, `TaskLoRa_Control()` | parameter LoRa, `UplinkPacket`, `DownlinkPacket` |
| REQ-RAD-002 | `TaskLoRa_Control()`, `setPidParams()` | `ConfigPacket`, `PidParams` |
| REQ-ARM-001 | `TaskMotors()` | `gArmedCmd`, `gEscMinPwm` |
| REQ-CTRL-001 | `computeCascadePid()` | parameter PID dan batas output |
| REQ-CTRL-002 | `computeCascadePid()` | `maxYawRate`, heading lock |
| REQ-CTRL-003 | `updateVerticalCollective()` | parameter vertical/hover |
| REQ-MOTOR-001 | `writeMotorMix()` | pin motor, batas ESC |
| REQ-MOTOR-002 | `setMotorFault()`, `writeMotorMix()` | `gMotorFaultActive`, `gMotorFaultPercent`, `gEnableFtc` |

## Lampiran C. Riwayat Dokumen

| Versi | Tanggal | Perubahan |
|---|---|---|
| 1.0 | 2026-09-21 | Dokumentasi awal berdasarkan seluruh source dalam folder `main`, termasuk timing/sampling, desain, protokol, requirement, test plan, dan batasan. |
