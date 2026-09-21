# Requirements dan Perancangan Firmware LoraTx

**Versi dokumen:** 1.0  
**Tanggal:** 21 September 2026  
**Sumber utama:** [`LoraTx.ino`](LoraTx.ino)  
**Target:** ESP32-WROOM-32, remote/master LoRa RA-02  
**Metode penyusunan:** inspeksi statis source; belum merupakan hasil pengujian hardware, kompilasi, atau pengukuran timing.

## 1. Tujuan, ruang lingkup, dan status requirement

Dokumen ini menjelaskan kebutuhan, rancangan, kontrak antarmuka, timing, dan kriteria verifikasi firmware LoraTx. Remote membaca dua joystick, menerima command laptop melalui USB Serial, mengirim target kontrol melalui LoRa, menerima telemetri, dan menampilkan status pada OLED.

Pembahasan flight controller dan aplikasi GUI dibatasi pada kontrak komunikasi yang harus mereka penuhi. Algoritma kontrol penerbangan, pembacaan sensor drone, GUI internal, dan mixer motor berada di luar ruang lingkup.

Status yang digunakan:

| Status | Arti |
| --- | --- |
| **Aktual** | Perilaku terlihat dalam source aktif; belum berarti lulus uji hardware. |
| **Integrasi** | Syarat untuk hardware, host serial, atau peer LoRa agar kompatibel. |
| **Usulan** | Requirement pengembangan yang belum diterapkan atau belum dipenuhi sepenuhnya. |
| **TBD** | Belum dapat ditetapkan dari source; membutuhkan keputusan desain atau pengukuran. |

Kata **harus** pada requirement Aktual menyatakan baseline yang hendak diverifikasi. Kata **harus** pada Usulan menyatakan target revisi, bukan klaim kemampuan firmware saat ini.

## 2. Arsitektur dan pembagian tanggung jawab

```text
Joystick kiri: roll/throttle ── ADC ──┐
Joystick kanan: yaw/pitch ───── ADC ──┤
                                    v
Laptop/GCS <── USB Serial ──> ESP32 LoraTx <── SPI ──> RA-02
                                    |                  ⇅ LoRa half-duplex
                                    I2C             Peer drone
                                    v
                              OLED SH1106G
```

| Bagian | Tanggung jawab LoraTx |
| --- | --- |
| Input analog | Sampling, kalibrasi pusat, inversi, EMA, deadzone, normalisasi. |
| Command host | Parsing ARM, kalibrasi, override, dan konfigurasi PID. |
| Target kontrol | Membentuk target roll/pitch, yaw-rate, throttle, dan bit ARM. |
| Radio | Mengirim paket kontrol/config dan menunggu balasan telemetri. |
| Telemetri | Decode fixed-point dan meneruskan data menjadi baris teks serial. |
| Tampilan | Joystick, target kontrol, ARM lokal, dan kualitas balasan radio. |

Firmware berupa satu sketch dengan `setup()` dan `loop()` berurutan. Tidak terdapat scheduler sampling tersendiri, task FreeRTOS buatan sketch, atau ISR sampling joystick. Pemanggilan driver dapat memiliki mekanisme internal sendiri.

### 2.1 Alur startup

1. Mengisi `PINS[]` dan `INVERT[]` dalam urutan R/T/Y/P.
2. Mengaktifkan Serial 115200, menunggu 300 ms, mengatur ADC 12-bit dan attenuasi `ADC_11db`.
3. Mengaktifkan I2C 400 kHz dan mencoba OLED alamat `0x3C`.
4. Jika OLED tersedia: animasi radar, drone, tulisan QMBED, kalibrasi dengan progress bar, dan ready flash.
5. Jika OLED gagal: pesan serial, hitung mundur tiga detik, lalu kalibrasi tanpa tampilan.
6. Mencoba `LoRa.begin(433E6)` dan mengatur parameter radio jika berhasil.
7. Menampilkan hasil inisialisasi radio bila OLED tersedia dan mengosongkan riwayat link.

`[CAL] READY` menandakan kalibrasi joystick selesai, bukan radio sudah siap. Tampilan `LORA OK` hanya menyatakan radio lokal berhasil diinisialisasi, bukan peer drone telah terhubung.

### 2.2 Alur satu siklus

```text
Periksa loraOK
  ├─ gagal → pesan error + delay 1 detik → kembali
  └─ berhasil
       → catat cycleStart
       → proses command Serial yang tersedia
       → baca joystick ATAU gunakan GUI override
       → konversi target dan kirim UplinkPacket
       → tulis [TX] ke Serial
       → RX: tunggu DownlinkPacket sampai valid atau timeout
       → jika valid: tulis telemetri ke Serial
       → perbarui riwayat reply/RSSI
       → gambar dashboard OLED
       → delay sisa periode jika masih tersedia
```

### 2.3 Model state konseptual

Model berikut menjelaskan alur; tidak ada enum state machine lengkap seperti ini dalam source.

| State/dimensi | Pemicu masuk | Keluar/perilaku |
| --- | --- | --- |
| BOOT / CALIBRATING | Reset ESP32 | Kalibrasi selesai → inisialisasi LoRa. |
| RADIO ERROR | `LoRa.begin()` gagal | Mengulang pesan; memerlukan reset untuk mencoba lagi. |
| RUNNING | Radio lokal siap | Mengulang TX → WAIT REPLY → DISPLAY. |
| DISARMED / ARMED | Awal `false`; command ARM/DISARM/STOP | Mengubah `armedState` yang dimasukkan ke uplink berikutnya. |
| HARDWARE / GUI OVERRIDE | Awal hardware; command STICK | STICKSTOP/STICKRESET mengembalikan pembacaan hardware. |
| RECALIBRATING | CAL SAMPLE atau CAL RESET | Sampling blocking lalu melanjutkan siklus. |

ARM dan override merupakan dua state independen. DISARM tidak mematikan override. STICKSTOP/STICKRESET tidak melakukan DISARM. Kehilangan balasan tidak otomatis mengubah kedua state tersebut.

## 3. Kebutuhan hardware dan lingkungan pengembangan

### 3.1 Perangkat dan pinout aktif

| Perangkat/sinyal | Pin/parameter | Status |
| --- | --- | --- |
| MCU | ESP32-WROOM-32 dengan lingkungan Arduino ESP32 | Integrasi |
| RA-02 SX1278 NSS / RESET / DIO0 | GPIO5 / GPIO14 / GPIO26 | Aktual |
| SPI SCK / MISO / MOSI | GPIO18 / GPIO19 / GPIO23 pada default SPI ESP32 yang dimaksud source | Integrasi |
| OLED | SH1106G, 128 × 64, alamat `0x3C`, reset `-1` | Aktual |
| I2C SDA / SCL | GPIO21 / GPIO22, 400 kHz | Aktual |
| Joystick kiri X: roll | GPIO33, `PIN_LEFT_X` | Aktual |
| Joystick kiri Y: throttle | GPIO32, `PIN_LEFT_Y` | Aktual |
| Joystick kanan X: yaw | GPIO35, `PIN_RIGHT_X` | Aktual |
| Joystick kanan Y: pitch | GPIO34, `PIN_RIGHT_Y` | Aktual |
| Sambungan host | USB Serial, 115200 baud | Integrasi |

**Konstanta aktif menjadi acuan wiring.** Komentar pembuka sketch menukar pasangan GPIO32/33 dan GPIO34/35 dibandingkan implementasi. Mapping kiri roll/throttle dan kanan yaw/pitch adalah mapping khusus, bukan susunan Mode 2 konvensional yang tertulis pada header.

SPI tidak diinisialisasi dengan daftar pin eksplisit oleh sketch. Board/core terpilih harus memiliki default SPI sesuai tabel, atau konfigurasi source harus disesuaikan.

### 3.2 Kebutuhan listrik dan RF

| ID | Requirement | Status |
| --- | --- | --- |
| HW-01 | Input ADC dan level sinyal harus sesuai batas listrik ESP32; gunakan referensi ground bersama. Attenuasi ADC bukan izin memberikan tegangan di luar batas pin. | Integrasi |
| HW-02 | RA-02 harus mendapat suplai 3,3 V stabil sesuai datasheet modul, dengan kapasitas arus dan decoupling yang memadai saat TX. Anggaran arus aktual: TBD. | Integrasi |
| HW-03 | Gunakan antena yang sesuai 433 MHz dan koneksi antena yang benar sebelum pengujian transmisi. | Integrasi |
| HW-04 | OLED harus kompatibel dengan SH1106G dan alamat I2C yang ditetapkan. | Integrasi |
| HW-05 | Penggunaan kanal, daya, gain antena, EIRP, dan duty cycle harus disesuaikan ketentuan lokasi pengoperasian; kepatuhan tidak dapat ditentukan dari source saja. | Integrasi |

### 3.3 Dependensi dan build

| Komponen | Kebutuhan |
| --- | --- |
| Toolchain | Arduino IDE atau Arduino CLI dengan board package Arduino ESP32. |
| Board | Target ESP32 yang cocok dengan WROOM-32 dan pin SPI di atas; FQBN/versi core harus dicatat saat build. |
| Bawaan core | `SPI.h`, `Wire.h`, API ADC, Serial, `String`, `millis()`, `delay()`. |
| Library radio | Library `LoRa.h` kompatibel SX1278 dengan API yang dipanggil sketch. |
| Library OLED | Adafruit GFX dan Adafruit SH110X beserta dependensi yang diminta library manager. |
| C/C++ | `string.h`, `stdio.h`, tipe integer ukuran tetap, dukungan `#pragma pack`. |

Versi core/library tidak dikunci dalam folder LoraTx. Reproduksibilitas build memerlukan pencatatan versi, board, opsi build, dan hasil kompilasi. Python bukan dependensi untuk membangun atau menjalankan firmware remote; host apa pun dapat digunakan jika memenuhi protokol serial.

Prosedur build: buka `LoraTx.ino`, pasang core/library, pilih board dan port, lakukan Verify, Upload, lalu periksa log boot pada 115200 baud. Jangan mengganti sketch aktif dengan `.bak` sebagai langkah build.

## 4. Baseline kebutuhan fungsional

| ID | Requirement | Status |
| --- | --- | --- |
| FUN-01 | Saat boot, remote harus memulai `armedState=false` dan `guiOverride=false`. | Aktual |
| FUN-02 | Remote harus mengkalibrasi pusat empat kanal sebelum operasi radio normal. | Aktual |
| FUN-03 | Dalam mode hardware, remote harus membaca empat kanal sekali per siklus dan menerapkan inversi → EMA → normalisasi berpusat. | Aktual |
| FUN-04 | Remote harus membentuk target roll/pitch dalam derajat, yaw dalam derajat/detik, throttle command 0–255, dan status ARM. | Aktual |
| FUN-05 | Remote harus mengirim satu uplink kontrol pada setiap siklus normal meskipun input tetap. | Aktual |
| FUN-06 | Setelah TX, remote harus menunggu balasan pada jendela RX dan meneruskan telemetri valid ke Serial. | Aktual |
| FUN-07 | Remote harus memproses command host sebagaimana kontrak pada Bagian 8. | Aktual |
| FUN-08 | Override harus menggunakan throttle host dan roll/yaw/pitch netral 128. | Aktual |
| FUN-09 | Remote harus dapat mengirim konfigurasi PID dalam ConfigPacket. Respons lokal tidak menyatakan peer telah menerapkannya. | Aktual |
| FUN-10 | Remote harus menghitung kualitas reply dengan riwayat 20 siklus dan memperbarui OLED bila tersedia. | Aktual |
| FUN-11 | Kegagalan OLED harus tetap memungkinkan kalibrasi dan percobaan inisialisasi radio. | Aktual |
| FUN-12 | Kegagalan inisialisasi radio harus menghasilkan pesan diagnostik berulang. | Aktual |

## 5. Sampling joystick, kalibrasi, dan konversi target

### 5.1 Urutan kanal dan algoritma

Urutan array adalah `0=R`, `1=T`, `2=Y`, `3=P`. ADC 12-bit memberi nilai nominal 0–4095. Keempat sumbu diinversi karena semua `INVERT_*` bernilai `true`.

```text
center[ch] = rata-rata integer sampel kalibrasi kanal
v          = 2 × center[ch] − raw[ch]       (jika inversi aktif)
filt[ch]   = filt[ch] + 0,35 × (v − filt[ch])
delta      = int(filt[ch]) − center[ch]
dead       = int(0,07 × 4096) = 286 count ADC
half       = max(center[ch], 4095 − center[ch])

jika abs(delta) <= dead: cmd = 128
selain itu: cmd = int(128 + 128 × clamp(delta / half, −1, +1))
```

Deadzone sebesar 286 count **pada setiap sisi pusat**, bukan 7% dari setengah rentang. Di luar deadzone, rentang tidak diskalakan ulang dari nol sehingga terdapat transisi nilai saat keluar deadzone. Kalibrasi hanya menentukan pusat, bukan minimum/maksimum travel atau linearitas joystick.

**Batas implementasi:** fungsi `centerAxisToCmd()` dapat menghasilkan **256** pada batas positif karena rumus `128 + 128 × 1`. Rentang yang dimaksud adalah 0–255, tetapi hasil fungsi belum di-clamp. Throttle uplink di-clamp ke 255; target sudut juga dibatasi; nilai dashboard dicast ke `uint8_t` sehingga 256 dapat berubah menjadi 0. `RAW_T` serial dapat tetap memperlihatkan 256. Pengujian endpoint diperlukan.

### 5.2 Kalibrasi

| Mode | Fungsi | Sampel per kanal | Jeda eksplisit | Penyimpanan |
| --- | --- | --- | --- | --- |
| Startup dengan OLED | `calibrateWithLoadingBar()` | 400 | 2 ms per set empat kanal, ditambah refresh OLED; delay 150 ms setelah selesai | RAM |
| Startup tanpa OLED | `calibrateCenters()` | 400 | 2 ms per set, setelah countdown 3 detik | RAM |
| Permintaan host | `calibrateFromGui()` | 300 | 3 ms per set | RAM |

Jumlah konversi ADC adalah 1.600 saat startup dan 1.200 saat kalibrasi host. Empat kanal dibaca berurutan, bukan serentak. Setiap kalibrasi mereset filter ke pusat yang baru. CAL RESET melakukan sampling ulang, bukan mengembalikan nilai pabrik. Tidak ada penyimpanan NVS/EEPROM atau pemeriksaan variansi untuk memastikan joystick diam.

### 5.3 Target fisik dan tanda

Untuk R/P/Y, `abs(cmd−128) <= 6` menghasilkan target nol. Di luar itu:

| Target | Rumus sebelum pembatasan | Default batas |
| --- | --- | --- |
| Roll | `(R−128)/127 × remoteMaxAngleDeg` | −25 sampai +25° |
| Pitch | `−(P−128)/127 × remoteMaxAngleDeg` | −25 sampai +25° |
| Yaw-rate | `(Y−128)/127 × remoteMaxYawRateDps` | −150 sampai +150°/s |
| Throttle | `clamp(T, 0, 255)` | 0–255, pusat 128 |

Pitch command tinggi berarti target negatif; komentar kode mengartikan ini sebagai nose-down/maju. Yaw positif diartikan CW/kanan oleh komentar. Kesesuaian arah fisik joystick dan frame koordinat receiver harus dibuktikan pada integrasi.

Throttle **bukan PWM motor langsung**. Nilai 128 adalah command netral yang dimaksud untuk pengendalian vertikal oleh peer; LoraTx tidak mengukur atau menjamin altitude hold. Nama lokal `targetThrottlePwm` dan label `us` pada OLED tidak mengubah satuan paket: nilainya tetap command 0–255.

## 6. Timing, frekuensi sampling, dan latensi

### 6.1 Definisi

- **Ts**: interval waktu antara dua sampling/kejadian sejenis.
- **fs**: frekuensi sampling, `fs = 1/Ts` untuk interval tetap.
- **Waktu konversi ADC**: durasi satu `analogRead()`; berbeda dari Ts loop dan belum diukur.
- **Jitter**: variasi interval aktual terhadap periode acuan.
- **Timeout RX**: lama jendela menunggu balasan, bukan batas latensi end-to-end.
- **Airtime**: waktu paket berada di udara; berbeda dari waktu seluruh siklus.

### 6.2 Tabel timing

| Aktivitas | Waktu/periode yang ditetapkan | Frekuensi nominal | Interpretasi |
| --- | --- | --- | --- |
| Loop mode normal | 145 ms | 6,897 Hz | Target minimum durasi siklus, bukan jaminan interval sampling presisi. |
| ADC hardware saat normal | Sekali per loop per kanal | Sekitar 6,897 sampel/detik/kanal jika loop mencapai target | Offset sampling berubah akibat proses serial sebelum sampling. |
| Loop override | 45 ms | 22,222 Hz | Target scheduler; waktu TX+RX dapat melampauinya. |
| ADC hardware saat override | Tidak dipanggil pada jalur operasi reguler | Tidak berlaku | Command CAL tetap dapat melakukan sampling. |
| RX normal | 100 ms maksimum yang diminta | Bukan fs | Dimulai setelah TX uplink dan output `[TX]`. |
| RX override | 35 ms maksimum yang diminta | Bukan fs | Decode/output sesudah paket terdeteksi dapat menambah durasi. |
| Kalibrasi startup | Delay 2 ms per set | Acuan delay-only 500 Hz | Aktual lebih lambat karena empat ADC dan, pada jalur OLED, refresh layar. |
| Durasi sampling startup | Akumulasi delay 800 ms | 400 sampel/kanal | Belum termasuk ADC, OLED, animasi, atau delay akhir. |
| Kalibrasi host | Delay 3 ms per set | Acuan delay-only 333,333 Hz | Bukan timer sampling tetap. |
| Durasi sampling host | Akumulasi delay 900 ms | 300 sampel/kanal | Aktual bertambah oleh ADC, komputasi, dan respons serial. |
| OLED dashboard | Sekali per loop jika OLED tersedia | Mengikuti loop aktual | Tidak ada refresh timer independen. |
| Riwayat link | 20 siklus | Resolusi loss 5 poin persentase | Sekitar 2,9 s normal / 0,9 s override hanya jika target periode tercapai. |
| Label WASPADA / BAHAYA | Toggle berdasar 400 / 200 ms | Siklus kedip penuh 800 / 400 ms | Terlihat hanya saat layar diperbarui. |
| Error radio | Delay 1.000 ms per iterasi error | Sekitar 1 Hz | Ditambah biaya serial/OLED. |

Kalibrasi OLED memperbarui progress pada `i % 16 == 0` dan sampel terakhir, sehingga interval antarset tidak seragam. Tidak ada satu frekuensi sampling startup presisi yang dapat diklaim hanya dari `delay(2)`.

### 6.3 Periode aktual dan sumber blocking

Secara pendekatan:

```text
Tkerja = Tserial_input + Tinput + TTX_uplink + Tserial_TX
       + Twait_RX + Tdecode_serial + TOLED + Tkomputasi
Tsiklus ≈ max(Ttarget, Tkerja)
```

`LoRa.endPacket()` dipanggil tanpa mode asynchronous. Pada library Arduino LoRa yang umum, pemanggilan ini menunggu TX selesai. Versi library harus dikonfirmasi. Ketika Tkerja melebihi target, kode tidak menambah delay dan tidak melakukan catch-up.

Sumber variasi utama: airtime, turnaround peer, timeout balasan, refresh OLED, buffering Serial, command PID tambahan, dan kalibrasi blocking. `readStringUntil('\n')` dapat menunggu timeout Stream jika baris belum lengkap; sketch tidak mengatur `Serial.setTimeout()`, sehingga nilai default core harus diverifikasi. Arus command terus-menerus dapat memperpanjang `while (Serial.available())` sebelum uplink berikutnya.

EMA menggunakan alpha per sampel, bukan berbasis elapsed time. Untuk Ts tetap 145 ms, konstanta waktu ekuivalen `−Ts/ln(1−0,35)` sekitar 337 ms; ini perkiraan filter diskret, bukan hasil pengukuran respons remote. Jitter atau blocking mengubah respons terhadap waktu.

### 6.4 Estimasi airtime bersyarat

Parameter yang eksplisit: SF7, BW125 kHz, CR4/5. Preamble, sync word, header mode, serta CRC tidak diatur eksplisit oleh sketch dan harus diperiksa pada library/peer.

Contoh estimasi berikut **mengasumsikan** preamble 8 simbol, explicit header, CRC payload nonaktif, dan low-data-rate optimization nonaktif:

```text
Tsym = 2^7 / 125000 = 1,024 ms
Npayload = 8 + ceil((8×PL − 4×7 + 28) / (4×7)) × 5
ToA = (8 + 4,25 + Npayload) × Tsym
```

| Paket | Payload | ToA estimasi |
| --- | --- | --- |
| Uplink | 10 byte | 36,096 ms |
| Downlink | 32 byte | 71,936 ms |
| Config | 65 byte | 118,016 ms |

Satu transaksi uplink+downlink pada asumsi tersebut memerlukan sekitar **108,032 ms airtime**, sebelum turnaround, serial, dan OLED. Karena itu **22,222 Hz bukan throughput transaksi yang terbukti**. Jendela RX override 35 ms juga lebih pendek dari airtime downlink contoh 71,936 ms; balasan langsung untuk uplink terkini dapat tidak selesai di dalam jendela itu. Tanpa nomor urut, balasan terlambat juga tidak dapat dihubungkan pasti ke uplink tertentu.

Nilai di atas harus dihitung ulang bila parameter PHY aktual berbeda. Timeout normal 100 ms pun bukan jaminan transaksi berhasil jika pemrosesan peer terlambat.

### 6.5 Requirement timing

| ID | Requirement | Status |
| --- | --- | --- |
| TIM-01 | Scheduler harus menargetkan 145 ms normal dan 45 ms override dengan hanya menunda sisa waktu. | Aktual |
| TIM-02 | Jendela tunggu RX harus menggunakan 100 ms normal atau 35 ms override. | Aktual |
| TIM-03 | Sampling harus dilaporkan per kanal dan dipisahkan dari laju radio, refresh OLED, serta baud rate. | Integrasi |
| TIM-04 | Sebelum menetapkan target performa, ukur interval ADC, TX, balasan valid, jitter, waktu boot, dan latensi command. | Usulan |
| TIM-05 | Periode dan timeout override harus disesuaikan airtime serta turnaround peer yang terukur. | Usulan |
| TIM-06 | Tetapkan batas latensi/jitter dan umur command yang dapat diterima; nilai numeriknya masih TBD. | Usulan |

## 7. Kontrak komunikasi LoRa

### 7.1 PHY dan ABI

| Parameter | Nilai |
| --- | --- |
| Frekuensi | 433 MHz |
| Spreading factor | 7 |
| Bandwidth | 125 kHz |
| Coding rate | 4/5 melalui `setCodingRate4(5)` |
| Daya TX lokal | 17 dBm yang diminta melalui API; bukan pengukuran EIRP |
| Topologi | Remote mengawali uplink, lalu beralih RX untuk balasan |
| Serialisasi | Salinan memori struct dengan `#pragma pack(push, 1)` |
| Integer | 8/16-bit sesuai tipe field; skala fixed-point |
| Float config | Memerlukan float 32-bit kompatibel IEEE-754 |
| Endianness | Tidak dikonversi oleh sketch; peer harus sesuai layout little-endian target ESP32 |

Peer harus cocok dalam PHY, header/CRC/sync word, magic, ukuran/urutan field, signedness, satuan, dan packing. Payload tidak memiliki versi protokol, sequence number, timestamp, checksum aplikasi, autentikasi, atau enkripsi yang diterapkan oleh sketch. Telemetri adalah balasan operasional, bukan ACK teridentifikasi untuk ARM atau ConfigPacket.

### 7.2 UplinkPacket — 10 byte, magic 0xA5

Offset di bawah mulai dari nol.

| Offset | Field | Tipe | Interpretasi |
| --- | --- | --- | --- |
| 0 | magic | uint8 | `0xA5` |
| 1–2 | targetRoll | int16 | derajat ×100 |
| 3–4 | targetPitch | int16 | derajat ×100 |
| 5–6 | targetYaw | int16 | derajat/detik ×100 |
| 7–8 | targetThrottle | uint16 | command 0–255; pusat 128 |
| 9 | armed | uint8 | 0 atau 1 |

Konversi float ke int16 memakai cast setelah perkalian 100, sehingga pecahan dibuang, bukan dibulatkan eksplisit. Paket netral disarmed yang diharapkan pada little-endian: `A5 00 00 00 00 00 00 80 00 00`.

### 7.3 DownlinkPacket — 32 byte, magic 0x5A

| Offset | Field | Tipe | Decode/satuan |
| --- | --- | --- | --- |
| 0 | magic | uint8 | `0x5A` |
| 1–6 | ax, ay, az | 3 × int16 | /100, m/s² |
| 7–12 | gx, gy, gz | 3 × int16 | /100, °/s |
| 13–14 | press | uint16 | /10, hPa |
| 15–16 | alt | int16 | /100, meter; referensi altitude ditentukan peer |
| 17–22 | roll, pitch, yaw | 3 × int16 | /100, derajat |
| 23–28 | uRoll, uPitch, uYaw | 3 × int16 | /100, koreksi PWM menurut kontrak, dalam µs |
| 29–30 | vbat | uint16 | /100, volt |
| 31 | flags | uint8 | bitfield di bawah |

Angka 30 byte pada komentar source merupakan ketidaksesuaian dokumentasi. **Kontrak integrasi aktual harus mengikuti `sizeof(DownlinkPacket)`, yaitu 32 byte pada layout packed ini:** magic 1 + accel 6 + gyro 6 + tekanan 2 + altitude 2 + attitude 6 + koreksi 6 + baterai 2 + flags 1. Ini juga menjelaskan penggunaan buffer RX 32 byte.

| Bit | Nama | Makna |
| --- | --- | --- |
| 0 | G | gyroCalibValid |
| 1 | B | bmiOK |
| 2 | P | bmpOK |
| 3 | V | vbatOK |
| 4–5 | BS | 0=OK, 1=WARN, 2=LIMIT, 3=CRITICAL menurut komentar |
| 6–7 | FS | 0=OK, 2=DESCENT, 3=LAND menurut komentar; nilai 1 tidak dijelaskan |

LoraTx hanya mendecode flag; tidak menjalankan tindakan failsafe berdasarkan flag tersebut. Rentang representasi int16 /100 adalah −327,68 sampai +327,67. Uint16 /100 sampai 655,35; tekanan /10 sampai 6553,5 hPa. Peer bertanggung jawab terhadap saturasi/wrap sebelum packing.

**Penerimaan aktual:** `packetSize >= sizeof(DownlinkPacket)` dan magic `0x5A`. Kode tidak mensyaratkan ukuran persis sama. Maksimal 32 byte dibaca ke buffer, kemudian disalin ke struct. Paket lebih pendek dibuang; magic salah tidak dianggap reply. Tidak ada validasi rentang fisik sensor atau identitas pengirim.

### 7.4 ConfigPacket — 65 byte, magic 0xC3

Byte 0 berisi magic. Enam belas field float masing-masing 4 byte mengikuti urutan:

| Indeks float | Field | Satuan/arti |
| --- | --- | --- |
| 1–3 | angleKp, angleKi, angleKd | Gain PID angle menurut implementasi peer |
| 4–6 | rateKp, rateKi, rateKd | Gain PID rate menurut implementasi peer |
| 7–9 | yawKp, yawKi, yawKd | Gain PID yaw menurut implementasi peer |
| 10 | maxAngle | derajat |
| 11 | maxYawRate | °/s |
| 12 | maxDeltaPwm | µs koreksi |
| 13 | escMinPwm | µs |
| 14 | escArmSpinPwm | µs |
| 15 | escMaxPwm | µs |
| 16 | hoverThrottlePwm | µs |

Offset float ke-i adalah `1 + 4×(i−1)`. Paket dikirim langsung saat command PID diproses. Tidak ada retry, readback, atau ACK konfigurasi khusus. `[PID] OK...` berarti jalur kirim lokal selesai, bukan konfigurasi telah diterapkan peer.

## 8. Antarmuka USB Serial

### 8.1 Framing dan command

Host menggunakan 115200 baud, konfigurasi serial default core (umumnya 8N1), dan mengakhiri satu command dengan newline `\n`. `trim()` membuang whitespace pinggir termasuk CR dari CRLF. Gunakan ejaan huruf besar di bawah; hanya cabang tertentu memakai pencocokan case-insensitive.

| Command | Perilaku aktual dan respons |
| --- | --- |
| `ARM` | `armedState=true`; case-insensitive; tidak ada ACK khusus, status terlihat pada `[TX]` berikutnya. |
| `DISARM` atau `STOP` | `armedState=false`; case-insensitive. |
| `CAL SAMPLE` | Kalibrasi 300 sampel/kanal dan respons `[CAL] OK CR=... CT=... CY=... CP=...`. |
| `CAL RESET` | Sama seperti CAL SAMPLE. Prefix `CAL ` case-sensitive, payload SAMPLE/RESET case-insensitive. |
| `STICK <t>` | `toInt()`, terima 0–255, aktifkan override; `[STICK] Override ON T=...`. |
| `STICKSTOP` | Kembali ke joystick; pencocokan `startsWith`, sehingga suffix tambahan juga dapat diterima. |
| `STICKRESET` | Set `guiThrottle=0`, matikan override; uplink selanjutnya membaca joystick, bukan dipaksa throttle nol. |
| `PID <angka...>` | Parsing maksimal 16 float; minimal 12 hasil konversi diterima; kirim ConfigPacket. |
| `FAULT <a1> <p1> ... <a4> <p4>` | Memerlukan delapan integer yang berhasil diparse; mencetak persentase lokal. Tidak dikirim melalui LoRa. |
| `FTC <mode>` | `toInt()`, nol → STANDBY, nonnol → ACTIVE; hanya pesan lokal `[FTC] MODE=...`. |
| Command lain | Diabaikan tanpa respons error umum. |

`STICK abc` dapat ditafsirkan nol oleh `String.toInt()` dan mengaktifkan override; rentang dicek tetapi sintaks numerik tidak diperiksa ketat. FAULT tidak membatasi aktivasi ke 0/1 atau persentase ke 0–100. Tidak terdapat expiry override ketika host berhenti mengirim.

### 8.2 PID: format dan validasi

Format yang direkomendasikan untuk host baru adalah tepat 16 angka dalam **satu baris**:

```text
PID <angleKp> <angleKi> <angleKd> <rateKp> <rateKi> <rateKd> <yawKp> <yawKi> <yawKd> <maxAngle> <maxYawRate> <maxDeltaPwm> <escMinPwm> <escArmSpinPwm> <escMaxPwm> <hoverThrottlePwm>
```

Perilaku parser yang perlu dipertahankan/diperbaiki secara sadar:

- 12 angka: empat field ESC/hover menggunakan default state remote saat itu.
- 13 angka: field escMinPwm dalam paket berubah, tetapi state ESC lokal tidak diperbarui oleh cabang `count >= 15` atau `count == 14`.
- 14 angka: terdapat cabang legacy; angka ke-14 dipakai untuk memperbarui maksimum lokal jika memenuhi rentang, lalu field arm-spin paket diganti idle lokal. Field escMaxPwm paket yang sebelumnya diisi default tidak ikut ditulis ulang ke maksimum lokal baru. Host sebaiknya tidak mengandalkan format ambigu ini.
- 15/16 angka: cabang ESC lengkap; angka ke-16 opsional untuk hover.
- Angka tambahan/trailing text setelah 16 hasil parse tidak diperiksa sebagai kesalahan.

| State lokal yang diubah | Syarat penerimaan lokal |
| --- | --- |
| remoteMaxAngleDeg | 5–45° |
| remoteMaxYawRateDps | 30–300°/s |
| throttleMinPwm, format ≥15 | 900–1400 µs |
| throttleIdlePwm, format ≥15 | 1000–1600 µs |
| throttleMaxPwm, format ≥15 | 1100–2200 µs dan ≥ idle lokal |
| hoverThrottlePwm, format 16 | Di antara idle dan maksimum lokal |

Pemeriksaan di atas hanya menentukan perubahan state lokal. **Nilai cfg yang gagal syarat lokal tetap dapat dikirim ke peer.** Gain, maxDeltaPwm, finite/NaN, serta hubungan semua batas belum divalidasi lengkap; pembaruan tidak atomik. Parameter ini tidak disimpan permanen di ESP32.

### 8.3 Output serial dan satuan

Contoh sintaks, bukan hasil pengukuran:

```text
[TX] R:0.0deg T:128cmd Y:0.0dps P:0.0deg ARM:0 RAW_T:128
[IMU] AX:0.00 AY:0.00 AZ:9.81 GX:0.00 GY:0.00 GZ:0.00
[BMP] P:1013.20 A:0.00
[BAT] V:11.10
[SMC] ROLL:0.00 PITCH:0.00 YAW:0.00 UR:0.00 UP:0.00 UY:0.00
[FLAGS] G:1 B:1 P:1 V:1 BS:0 FS:0
[CAL] OK CR=2048 CT=2048 CY=2048 CP=2048
[PID] OK CONFIG + HOVER THROTTLE SENT TO DRONE
[FAULT] OK M1=0% M2=0% M3=0% M4=0%
[FTC] MODE=ACTIVE
```

`[TX]` dibuat setiap siklus yang mencapai TX, termasuk ketika drone tidak membalas. Kelima baris telemetri dibuat hanya setelah downlink valid, dalam urutan IMU, BMP, BAT, SMC, FLAGS. Tag `[SMC]` adalah nama historis format output; bukan bukti algoritma SMC dijalankan di remote.

Host harus membedakan `T:128cmd` dari format lama dengan `us`, menerima log diagnostik non-tag, dan tidak menafsirkan `ARM:1` sebagai status motor yang dikonfirmasi drone. Tidak ada timestamp akuisisi sensor di payload; waktu penerimaan host tidak identik dengan waktu sampling di drone.

## 9. Konfigurasi terpusat dan persistensi

| Konstanta/state | Default | Dampak |
| --- | --- | --- |
| remoteMaxAngleDeg | 25,0 | Skala roll/pitch; dapat diubah lewat PID. |
| remoteMaxYawRateDps | 150,0 | Skala yaw; dapat diubah lewat PID. |
| throttleMinPwm / throttleIdlePwm / throttleMaxPwm | 1000 / 1200 / 1300 | Default field konfigurasi peer; tidak memetakan throttle uplink ke µs. |
| hoverThrottlePwm | 1260,0 | Default field hover peer. |
| JOYSTICK_FILTER_ALPHA | 0,35 | Smoothing per sampel. |
| JOYSTICK_DEADZONE | 0,07 | Deadzone ADC terhadap full-scale 4096. |
| STICK_DEADBAND | 6 | Deadband target R/P/Y dalam count command. |
| CALIBRATION_SAMPLES / GUI_CAL_SAMPLES | 400 / 300 | Jumlah set sampel kalibrasi. |
| CYCLE_PERIOD_MS / REPLY_TIMEOUT_MS | 145 / 100 ms | Scheduling normal. |
| CYCLE_PERIOD_OVERRIDE_MS / REPLY_TIMEOUT_OVERRIDE_MS | 45 / 35 ms | Scheduling override. |
| SIGNAL_WINDOW | 20 | Panjang riwayat reply. |
| INVERT_LEFT_X/Y, INVERT_RIGHT_X/Y | true | Inversi semua sumbu. |
| THROTTLE_EXPO | 0,40 | Dideklarasikan tetapi tidak digunakan dalam jalur throttle aktif. |
| SWAP_LEFT_X_Y / SWAP_RIGHT_X_Y | false | Dideklarasikan tetapi tidak dipakai untuk menukar kanal dalam jalur aktif. |

`throttleToCmd()` tersedia tetapi tidak dipakai oleh `processJoystick()`. Seluruh kanal aktif memakai `centerAxisToCmd()`. Perubahan konstanta yang tidak dipakai tidak memberi efek runtime yang mungkin diharapkan dari namanya.

State kalibrasi, PID lokal, ARM, dan override berada di RAM. Reset mengembalikan default dan menjalankan kalibrasi lagi.

## 10. Kualitas link dan OLED

### 10.1 Metrik

- Reply valid: simpan RSSI, set history sukses, reset `consecutiveMiss`.
- Tanpa reply valid: history gagal, RSSI slot −120, tambah `consecutiveMiss`.
- `lossPct = misses × 100 / 20`.
- `avgRssi` adalah rata-rata integer dari slot sukses saja; bila tidak ada sukses gunakan −120.
- SNR terakhir disimpan dalam `lastSnr`, tetapi tidak dipakai untuk klasifikasi atau tampilan.

Metrik loss adalah **persentase siklus tanpa balasan valid**, bukan pengukuran seluruh paket RF yang hilang. Tidak diketahui apakah uplink gagal, peer lambat, atau downlink gagal. Riwayat awal diisi gagal, sehingga indikator startup bias buruk sampai jendela terisi; ini bukan hasil 20 transaksi yang benar-benar pernah dicoba.

### 10.2 Klasifikasi label (urutan evaluasi)

| Label | Kondisi |
| --- | --- |
| BAHAYA | consecutiveMiss ≥5 **atau** loss ≥35% **atau** avgRssi <−110 |
| WASPADA | Jika bukan BAHAYA: loss ≥10% **atau** avgRssi <−98 |
| AMAN | Selain kondisi di atas |

Saat consecutiveMiss ≥5, teks menjadi `NO LINK`. Indikator batang memakai aturan berbeda:

| Batang | Kondisi berurutan |
| --- | --- |
| 0 | miss beruntun ≥5 atau loss ≥60% atau RSSI ≤−120 |
| 4 | RSSI ≥−82 dan loss <5% |
| 3 | RSSI ≥−95 dan loss <15% |
| 2 | RSSI ≥−105 dan loss <30% |
| 1 | Sisanya |

Dashboard menampilkan gimbal kiri R/T, kanan Y/P, target, status ARM lokal, RSSI, dan indikator kualitas. **Angka loss tidak dicetak langsung**, walaupun dipakai untuk label/batang. RSSI pada layar berlabel `dB`; satuan RSSI yang dimaksud adalah dBm. Throttle masih berlabel `us` walaupun nilai aktual adalah command. Kedua label merupakan temuan dokumentasi/UI.

## 11. Error, batasan, dan requirement pengembangan

### 11.1 Perilaku error aktual

| Kondisi | Respons aktual | Recovery |
| --- | --- | --- |
| OLED gagal | Log serial, jalur kalibrasi tanpa OLED | Operasi radio tetap dicoba. |
| LoRa gagal startup | Pesan OLED/serial berulang, return sebelum parser command | Perbaiki penyebab lalu reset. |
| Downlink salah/timeout | Tidak meneruskan telemetri baru, tambah miss jika tidak ada reply valid sampai akhir jendela | Coba uplink lagi pada siklus berikutnya. |
| Host terputus | Tidak ada deteksi host/expiry eksplisit | State ARM/override terakhir dapat tetap berlaku. |
| CAL saat ARM | Tetap diterima, sampling blocking | Tidak ada interlock otomatis. |
| PID tidak cukup angka | `[PID] ERR FORMAT INVALID (need 12-16 float values)` | Host mengirim ulang format benar. |

### 11.2 Daftar usulan yang dapat diverifikasi

| ID | Requirement usulan | Kriteria selesai |
| --- | --- | --- |
| IMP-01 | Batasi hasil normalisasi semua kanal ke 0–255. | Endpoint tidak menghasilkan 256 atau wrap dashboard. |
| IMP-02 | Selaraskan timeout/periode override dengan airtime dan turnaround aktual. | Reply yang berkorelasi ke command diterima pada laju target yang disepakati. |
| IMP-03 | Validasi sintaks numerik, finite, rentang, dan hubungan PID/ESC secara atomik sebelum kirim. | Input rusak ditolak; state lokal/paket tidak berubah sebagian. |
| IMP-04 | Tambahkan ACK/readback config dan identifikasi transaksi. | Host dapat membedakan sent, applied, rejected, timeout. |
| IMP-05 | Gunakan parsing serial berbatas panjang dan nonblocking. | Command parsial/burst tidak menghentikan layanan radio melampaui batas yang disepakati. |
| IMP-06 | Tolak/tunda kalibrasi saat ARMED dan tetapkan syarat ARM. | Transisi terlarang menghasilkan respons eksplisit tanpa menghentikan layanan kontrol. |
| IMP-07 | Tentukan expiry command/override dan kontrak kehilangan link bersama peer. | Host/link terputus menghasilkan state yang disepakati; penghentian motor tidak hanya bergantung pada paket DISARM yang mungkin gagal terkirim. |
| IMP-08 | Tambahkan identitas/version/sequence/freshness serta mekanisme autentikasi jika menjadi kebutuhan deployment. | Paket salah versi, duplikat, kedaluwarsa, atau tidak sah ditangani sesuai kontrak yang diuji. |
| IMP-09 | Perbaiki komentar pin, ukuran downlink, Mode 2, dan label satuan OLED. | Dokumentasi, tampilan, dan paket menyatakan nilai yang sama. |
| IMP-10 | Tegaskan FAULT/FTC hanya lokal atau implementasikan protokol peer yang nyata. | Respons tidak memberi kesan penerapan di drone tanpa konfirmasi. |
| IMP-11 | Tambahkan recovery radio dan diagnostik yang terukur bila dibutuhkan. | Retry memiliki batas/backoff dan tidak membuat status siap palsu. |
| IMP-12 | Pisahkan startup history yang belum terisi dari reply gagal aktual. | Loss awal dihitung hanya dari transaksi yang sudah dicoba. |

Indikator BAHAYA/NO LINK saat ini hanya tampilan, bukan mekanisme disarm. Tidak ada verifikasi peer ARM, autentikasi command, atau perlindungan replay di sketch. Requirement keselamatan di atas merupakan target desain bersama sistem, bukan klaim tindakan motor yang dilakukan LoraTx.

## 12. Prosedur operasional dan kriteria penerimaan

### 12.1 Prosedur penggunaan untuk verifikasi

1. Cocokkan wiring dengan konstanta aktif, suplai, antena, dan board yang dipilih.
2. Untuk pengujian ARM/target/PID, gunakan peer simulator atau bench tanpa propeller.
3. Nyalakan remote dengan semua sumbu netral dan diam selama kalibrasi.
4. Periksa log kalibrasi dan radio; bedakan radio lokal siap dari balasan peer valid.
5. Gerakkan tiap sumbu satu per satu, cocokkan arah/nilai serial dan OLED.
6. Gunakan command lengkap ber-newline; untuk PID gunakan 16 angka dan catat konfigurasi.
7. Setelah override, gunakan STICKSTOP untuk kembali hardware dan verifikasi nilai; lakukan DISARM secara terpisah bila diperlukan.
8. Saat mengakhiri pengujian, kirim DISARM dan pastikan state peer melalui sarana pengujian yang tersedia sebelum mematikan perangkat.

### 12.2 Matriks pengujian

**Status semua pengujian berikut: direncanakan, belum dijalankan dalam penyusunan dokumen.** Uji baseline dan uji target perbaikan harus dilaporkan terpisah; sebuah known issue tidak menjadi lulus hanya karena sudah terdokumentasi.

| ID | Skenario/metode | Hasil yang harus diperiksa | Acuan |
| --- | --- | --- | --- |
| TEST-01 | Build pada board/core/library tercatat | Kompilasi berhasil; catat ukuran binary dan versi dependency. | Bagian 3 |
| TEST-02 | Boot dengan OLED; boot tanpa OLED | Dua jalur kalibrasi, ARM awal 0, radio tetap dicoba jika OLED gagal. | FUN-01/02/11 |
| TEST-03 | Lepas/tahan joystick netral | Pusat dihitung, filter direset, command netral 128; ulang sesudah reset. | FUN-02/03 |
| TEST-04 | Sapuan empat sumbu dan endpoint | GPIO benar, inversi/tanda target benar, batas target; reproduksi risiko 256 lalu verifikasi perbaikan. | FUN-04, IMP-01 |
| TEST-05 | Tangkap paket uplink di peer simulator | 10 byte, magic, packing, unit, ARM; paket netral sesuai hex Bagian 7.2. | FUN-05 |
| TEST-06 | Kirim downlink sintetis valid | 32 byte packed, semua skala dan flag benar; lima baris telemetri sesuai urutan. | FUN-06 |
| TEST-07 | Paket kurang dari struct, magic salah, paket lebih panjang | Pendek/salah magic tidak menjadi reply; catat penerimaan paket lebih panjang pada baseline. | Bagian 7.3 |
| TEST-08 | ARM/DISARM/STOP, STICK 0/128/255, STICKSTOP/RESET | State sesuai tabel; DISARM dan override independen; STICKRESET kembali hardware. | FUN-07/08 |
| TEST-09 | PID 12/13/14/15/16 angka, kurang angka, NaN, trailing text | Catat paket/state aktual serta kelemahan parsing; target revisi harus menolak invalid tanpa perubahan sebagian. | FUN-09, IMP-03 |
| TEST-10 | FAULT/FTC dengan capture radio | Respons lokal ada, tidak muncul paket khusus FAULT/FTC. | Bagian 8 |
| TEST-11 | Hilangkan reply; injeksikan riwayat/RSSI terkontrol | Loss langkah 5%, threshold tepat termasuk 35%, 5 miss → NO LINK; ARM tidak berubah pada baseline. | FUN-10 |
| TEST-12 | Radio gagal startup | Pesan error berulang, parser normal tidak berjalan, retry hanya setelah reset. | FUN-12 |
| TEST-13 | Ukur normal/override: reply sukses, tidak ada peer, peer terlambat | Distribusi periode dan reply valid; bandingkan dengan airtime, bukan hanya 1/konstanta periode. | TIM-01/02/04/05 |
| TEST-14 | Serial tanpa newline, burst command, dan CAL | Ukur celah TX/ADC; verifikasi biaya blocking dan target nonblocking setelah revisi. | IMP-05/06 |
| TEST-15 | Putus host saat override, putus RF saat ARM | Catat persistensi state baseline; uji expiry/failsafe setelah kontrak revisi ditetapkan. | IMP-07 |

### 12.3 Rancangan pengukuran timing

Gunakan instrumentasi timestamp sementara atau logic analyzer pada GPIO penanda di awal sampling, awal/akhir TX, dan penerimaan reply. Dokumentasikan overhead instrumentasi; log Serial berlebihan dapat mengubah timing yang sedang diukur.

Untuk setiap mode/kondisi, rekam sedikitnya 1.000 siklus sebagai rencana awal pengukuran, lalu laporkan:

- interval sampling per kanal dan periode loop: minimum, rata-rata, maksimum, persentil ke-95;
- frekuensi efektif `jumlah interval / total waktu`, bukan sekadar invers rata-rata frekuensi instan;
- waktu TX, turnaround peer, penerimaan paket lengkap, dan rasio reply valid;
- latensi ARM/DISARM dari newline diterima sampai uplink terkait; tanpa ACK belum dapat mengukur waktu penerapan peer dari remote saja;
- durasi startup, kalibrasi OLED/non-OLED, dan jeda terpanjang tanpa uplink;
- versi firmware/core/library, parameter PHY lengkap, kondisi suplai, dan konfigurasi host.

Ambang kelulusan performa numerik selain konstanta source harus ditetapkan setelah kebutuhan aplikasi dan hasil pengukuran tersedia. Jangan mengklaim 22 Hz, hard real-time, jarak jangkauan, atau reliabilitas penerbangan hanya dari definisi konstanta.

## 13. Traceability ke source

Nomor baris mengacu snapshot saat penyusunan dan dapat bergeser setelah revisi. Nama fungsi/simbol merupakan acuan utama. Pengiriman dan penerimaan berada langsung di `loop()`; tidak ada fungsi `sendUplink()` atau `receiveTelemetry()` terpisah.

| Requirement/topik | Fungsi/simbol nyata | Rentang baris acuan |
| --- | --- | --- |
| Hardware, protokol | define pin, tiga struct packet | 50–108, 120–139 |
| Konfigurasi dan timing | remoteMaxAngleDeg, CYCLE_PERIOD_MS, SIGNAL_WINDOW | 110–180 |
| FUN-02, kalibrasi | calibrateCenters, calibrateFromGui, calibrateWithLoadingBar | 187–217, 330–384 |
| FUN-03/04, input | readRaw, applyInvert, centerAxisToCmd, processJoystick | 183–253 |
| FUN-10, klasifikasi | getSignalStrength, classifySignal | 466–491 |
| FUN-10, dashboard | renderDashboard | 517–613 |
| FUN-01/11, boot | setup, inisialisasi state global | 148–160, 618–708 |
| FUN-12, radio error | loop: cabang !loraOK | 715–732 |
| FUN-07/08/09, Serial | loop: parser command | 736–853 |
| FUN-04/05, uplink | loop: pemilihan input, mapping, LoRa.write | 855–920 |
| FUN-06, downlink | loop: parsePacket, decode, Serial.print | 922–992 |
| FUN-10, history | replyHist, rssiHist, classifySignal | 994–1025 |
| TIM-01/02 | replyTimeout, elapsed, cyclePeriod | 925–928, 1028–1033 |

## 14. Known issues dan keputusan terbuka

Temuan utama yang harus dibawa ke revisi berikutnya:

1. Ukuran **downlink aktual 32 byte**, bukan 30 pada komentar source maupun 27 pada header lama; uplink aktif 10 byte, bukan 6 pada header.
2. Pin joystick header berbeda dari konstanta aktif; label Mode 2 tidak sesuai mapping aktual.
3. Endpoint normalisasi dapat 256 dan wrap saat ditampilkan sebagai uint8.
4. Label throttle OLED `us` tidak sesuai command; RSSI berlabel `dB`, bukan dBm.
5. Periode override 45 ms dan RX 35 ms perlu dievaluasi terhadap airtime transaksi; laju 22 Hz belum terbukti.
6. State ARM/override tidak memiliki expiry; CAL blocking diterima ketika ARM; kualitas link tidak memicu aksi otomatis.
7. Konfigurasi PID hanya memiliki validasi lokal parsial, jalur legacy ambigu, dan tidak ada ACK/readback peer.
8. FAULT/FTC adalah respons lokal, bukan forwarding kendali fault melalui radio.
9. Riwayat loss startup berisi kegagalan sintetis; paket panjang diterima berdasarkan ambang minimum, bukan ukuran persis.
10. File `LoraTx.ino.bak` adalah artefak historis, bukan sumber requirement aktif. Jangan mengasumsikan kompatibilitas paketnya tanpa pemeriksaan ulang.

Keputusan TBD: versi core/library dan FQBN yang dipakai, parameter PHY default aktual, anggaran arus hardware, tingkat akurasi/jitter yang diperlukan, umur maksimum command, tindakan saat host/link hilang, syarat ARM, strategi persistensi, serta ambang penerimaan pengujian hardware.

## 15. Riwayat dokumen

| Versi | Tanggal | Perubahan |
| --- | --- | --- |
| 1.0 | 2026-09-21 | Requirements khusus LoraTx berbasis source aktif, kontrak paket/serial, sampling dan airtime, traceability, known issues, serta rencana verifikasi. |
