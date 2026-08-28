/* ==========================================================================
 * CONFIG.H — Konfigurasi Pinout, Parameter Sensor, ESC, LoRa & FreeRTOS
 * Target Board: STM32F401RCT6 (Generic F401RCTx)
 * ==========================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/* ── Serial Debug (USART1) ───────────────────────────────────────────── */
#define SERIAL_TX_PIN         PA9
#define SERIAL_RX_PIN         PA10
#define SERIAL_BAUD           115200

/* ── LoRa RA-02 (SX1278 di SPI2) ─────────────────────────────────────── */
#define LORA_SCK_PIN          PB13
#define LORA_MISO_PIN         PB14
#define LORA_MOSI_PIN         PB15
#define LORA_NSS_PIN          PB12
#define LORA_RST_PIN          PB1
#define LORA_DIO0_PIN         PB0
#define LORA_FREQUENCY        433E6
#define LORA_SPREADING_FACTOR 7
#define LORA_SIGNAL_BANDWIDTH 125E3
#define LORA_CODING_RATE      5

/* ── I2C Bus (Sensor BMI160 + BMP280) ────────────────────────────────── */
#define I2C_SCL_PIN           PB10
#define I2C_SDA_PIN           PB3
#define I2C_CLOCK_SPEED       400000
#define BMI160_ADDR           0x68
#define BMP280_ADDR           0x76

/* ── ESC 4 Motor (Quad-X) ────────────────────────────────────────────── */
/* Arah putaran nyata (dikonfirmasi user): M1 FL CW, M2 FR CCW,
 * M3 BR CW, M4 BL CCW. Sesuai tabel mixer di writeSmcMotorMix(). */
#define MOTOR1_PIN            PB6    // Motor 1: CW  (Depan-Kiri)
#define MOTOR2_PIN            PB7    // Motor 2: CCW (Depan-Kanan)
#define MOTOR3_PIN            PB8    // Motor 3: CW  (Belakang-Kanan)
#define MOTOR4_PIN            PB9    // Motor 4: CCW (Belakang-Kiri)

#define ESC_MIN_US            1000   // PWM Stop / Idle (0%)
#define ESC_MAX_US            2000   // PWM Full (100%)
#define ESC_ARM_SPIN_US       1200   // 20% throttle saat ARMED (1000 + 1000*0.20)
#define ARMING_DURATION_MS    5000   // Waktu tunggu arming (5 detik)
#define LINK_TIMEOUT_MS       1000   // Failsafe timeout jika LoRa hilang > 1s

/* ── Manual Quad-X Mixer (tanpa PID / self-level) ────────────────────── */
#define MIX_ROLL_GAIN_US      150.0f // Koreksi maksimum roll pada defleksi stick penuh
#define MIX_PITCH_GAIN_US     150.0f // Koreksi maksimum pitch pada defleksi stick penuh
#define MIX_YAW_GAIN_US       100.0f // Koreksi maksimum yaw pada defleksi stick penuh
#define MIX_ACTIVE_MIN_US     1220   // R/P/Y aktif hanya setelah throttle sedikit di atas idle
#define THROTTLE_DEADBAND      4      // Toleransi ADC di sekitar titik tengah stick throttle
#define THROTTLE_RATE_US_PER_S 150.0f // Laju perubahan throttle pada defleksi stick penuh

/* ── LED Indikator ───────────────────────────────────────────────────── */
#define LED_PIN               PC4    // Aktif LOW (Arming blink 1Hz, Armed ON, Disarm OFF)

/* ── FreeRTOS Task Configuration ─────────────────────────────────────── */
#define TASK_SENSOR_PERIOD_MS    10     // 100 Hz periodik
#define TASK_SENSOR_PRIORITY     3      // Prioritas tinggi (Realtime Sensor Read)
#define TASK_SENSOR_STACK_SIZE   256    // Stack size (words)

#define TASK_LORA_PERIOD_MS      5      // Polling periodik ~5 ms
#define TASK_LORA_PRIORITY       2      // Prioritas menengah (Radio & Control)
#define TASK_LORA_STACK_SIZE     384    // Stack size (words)

#define TASK_CONTROL_PERIOD_MS   5      // 200 Hz attitude-control loop
#define TASK_CONTROL_PRIORITY    4      // Prioritas tertinggi: menulis PWM motor
#define TASK_CONTROL_STACK_SIZE  384    // Stack size (words)

/* ── Sliding Mode Controller (initial values, tune via GUI) ───────────── */
#define SMC_K1_DEFAULT           5.0f
#define SMC_K2_DEFAULT           2.0f
#define SMC_EPS_DEFAULT          8.0f
#define SMC_IX_DEFAULT           0.0030f // Estimasi awal quad 450-class (kg m^2)
#define SMC_IY_DEFAULT           0.0030f // Estimasi awal quad 450-class (kg m^2)
#define SMC_IZ_DEFAULT           0.0050f // Estimasi awal quad 450-class (kg m^2)
#define SMC_ARM_LENGTH_DEFAULT   0.225f  // Jarak pusat ke motor, meter
#define SMC_FORCE_TO_PWM_DEFAULT 30.0f
#define SMC_DELTA_MAX_DEFAULT    180.0f // Batas koreksi PWM per motor, us
#define SMC_CF_ALPHA             0.98f  // Complementary filter roll/pitch
#define SMC_BENCH_DEBUG          1      // Cetak data SMC untuk props-off bench test

/* ── Protokol Paket Biner LoRa ───────────────────────────────────────── */
#define UPLINK_MAGIC          0xA5
#define DOWNLINK_MAGIC        0x5A
#define CONFIG_MAGIC          0xC3

#pragma pack(push, 1)
struct UplinkPacket {
  uint8_t magic;    // harus == UPLINK_MAGIC (0xA5)
  uint8_t r, t, y, p;
  uint8_t armed;    // 0 = DISARM, 1 = ARM
};

struct DownlinkPacket {
  uint8_t  magic;   // harus == DOWNLINK_MAGIC (0x5A)
  int16_t  ax, ay, az;   // m/s^2  x100
  int16_t  gx, gy, gz;   // deg/s  x100
  uint16_t press;        // hPa    x10
  int16_t  alt;          // meter  x100
  int16_t  roll, pitch;  // derajat x100, complementary filter onboard
  int16_t  uRoll, uPitch, uYaw; // koreksi SMC PWM x100
};

struct ConfigPacket {
  uint8_t magic;         // harus == CONFIG_MAGIC (0xC3)
  float k1;
  float k2;
  float eps;
  float forceToPwm;
  float deltaMaxPwm;
};
#pragma pack(pop)

/* ── Shared Telemetry Struct (Protected by Mutex) ─────────────────────── */
struct SensorData {
  float ax, ay, az;      // m/s^2
  float gx, gy, gz;      // deg/s
  float press;           // hPa
  float alt;             // meter relatif
  float roll;            // derajat, complementary filter onboard
  float pitch;           // derajat, complementary filter onboard
  float yawRate;         // deg/s, untuk yaw-rate damping
  bool  bmiOK;
  bool  bmpOK;
};

#endif // CONFIG_H
