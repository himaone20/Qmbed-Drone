# Program Drone — Flight Controller STM32F401RCT6

Program flight controller drone berbasis **STM32F401RCT6** dengan **FreeRTOS** (Arduino IDE + STM32duino).

## Status Pengembangan

| Modul | Status |
|---|---|
| BMI160 IMU (gyro + accel + complementary filter) | Selesai |
| BMP280 Barometer | Skeleton / TODO |
| Kontrol SMC | Belum dimulai |
| Mixer & output ESC | Belum dimulai |
| Komunikasi LoRa (drone <-> remote) | Belum dimulai |

## Struktur Folder

```
main/
├── main.ino      # Setup FreeRTOS, task pembacaan sensor, scheduler
├── config.h      # Pin I2C, alamat sensor, parameter task
├── BMI160.h/.cpp # Class driver IMU BMI160 (baca gyro/accel + fusi sudut)
└── BMP280.h/.cpp # Class skeleton barometer BMP280 (belum diimplementasi)
```

`Refrensi/` berisi kode referensi proyek sebelumnya (tidak di-track oleh git).

## Hardware

- **MCU:** STM32F401RCT6
- **IMU:** BMI160 — alamat I2C `0x68`
- **Barometer:** BMP280 — library & alamat menyusul
- **I2C:** SDA = `PB7`, SCL = `PB6`, clock 400 kHz

## Toolchain

- Arduino IDE dengan board package **STM32duino** (Generic STM32F4 series)
- Library **STM32FreeRTOS** (Library Manager)

## Arsitektur Task (saat ini)

- `TaskReadSensors`: periodik 4 ms (250 Hz) via `vTaskDelayUntil`, membaca BMI160 + BMP280 lalu mencetak roll/pitch/altitude ke Serial1 (115200 baud, PA10/PA9) untuk verifikasi.
