/* ==========================================================================
 * CONFIG.H — Konfigurasi Pinout, Parameter Sensor, LoRa & FreeRTOS
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

/* ── I2C Bus (Sensor BMI160 + BMP280/BMP180) ─────────────────────────── */
#define I2C_SCL_PIN           PB10
#define I2C_SDA_PIN           PB3
#define I2C_CLOCK_SPEED       400000
#define BMI160_ADDR           0x68
#define BMP180_ADDR           0x77
#define BMP280_ADDR           0x77  // Alias

/* ── ADC Voltage Sensor (Baterai 3S LiPo) ─────────────────────────────── */
#define VBAT_PIN              PA0
#define VBAT_ADC_REF          3.3f
#define VBAT_ADC_RES          4095.0f  // 12-bit ADC
#define VBAT_R1               100000.0f
#define VBAT_R2               20000.0f
#define VBAT_DIV_RATIO        ((VBAT_R1 + VBAT_R2) / VBAT_R2) // 6.0f
#define VBAT_CAL_FACTOR       1.0f

/* ── ESC 4 Motor (Quad-X) ────────────────────────────────────────────── */
/* Konfigurasi motor:
 * M1: PB6 (Depan-Kiri, CW)
 * M2: PB7 (Depan-Kanan, CCW)
 * M3: PB8 (Belakang-Kanan, CW)
 * M4: PB9 (Belakang-Kiri, CCW)
 */
#define MOTOR1_PIN            PB6    // M1: Depan-Kiri (CW)
#define MOTOR2_PIN            PB7    // M2: Depan-Kanan (CCW)
#define MOTOR3_PIN            PB8    // M3: Belakang-Kanan (CW)
#define MOTOR4_PIN            PB9    // M4: Belakang-Kiri (CCW)

#define ESC_MIN_US            1000   // PWM Stop / Disarmed (0%)
#define ESC_ARM_SPIN_US       1200   // PWM Idle spin saat Armed (20%)
#define ESC_MAX_US            1300   // PWM Maksimum (100%)

/* ── Cascade PID Parameters (Sesuai default GUI drone_viewer.py) ─────── */
#define PID_ANGLE_KP_DEFAULT     5.00f  // Outer loop angle P (galat deg -> target rate deg/s)
#define PID_ANGLE_KI_DEFAULT     0.05f  // Outer loop angle I
#define PID_ANGLE_KD_DEFAULT     0.12f  // Outer loop angle D

#define PID_RATE_KP_DEFAULT      1.60f  // Inner loop rate P (galat deg/s -> delta PWM us)
#define PID_RATE_KI_DEFAULT      0.30f  // Inner loop rate I
#define PID_RATE_KD_DEFAULT      0.045f // Inner loop rate D (Peredam instan anti-terbalik)

#define PID_MAX_ANGLE_DEG        25.0f  // Sudut roll/pitch maksimum saat full stick (derajat)
#define PID_MAX_RATE_DPS         250.0f // Batas kecepatan angular rate (deg/s)
#define PID_MAX_DELTA_PWM        300.0f // Batas koreksi maksimum per sumbu (us)
#define PID_INTEGRAL_MAX         80.0f  // Batas anti-windup akumulasi integral (us)
#define PID_TAU_FILTER           0.008f // Filter time constant derivatif (8ms respons cepat)

#define MIX_ACTIVE_MIN_US        1150   // Ambang batas throttle aktif PID (> 1150us)

/* ── Vertical Velocity Control (BMI160 only, no barometer feedback) ──── */
// targetThrottle pada paket LoRa adalah command stick 0..255 dari remote.
#define VERTICAL_THROTTLE_CENTER             128.0f
#define VERTICAL_THROTTLE_DEADBAND            8.0f
#define VERTICAL_MAX_TARGET_SPEED_MPS         0.60f
#define VERTICAL_STICK_EXPO                   0.45f
#define VERTICAL_TARGET_SLEW_MPS2             0.80f
#define VERTICAL_ACCEL_LPF_ALPHA              0.12f
#define VERTICAL_MAX_ACCEL_MPS2                4.0f
#define VERTICAL_VELOCITY_LPF_ALPHA           0.10f
#define VERTICAL_VELOCITY_LEAK_PER_S          0.80f
#define VERTICAL_MAX_ESTIMATED_SPEED_MPS      2.00f
#define VERTICAL_ACCEL_DAMP_US_PER_MPS2       5.0f
// IMU velocity is only short-term damping. It must not be able to cancel
// the pilot collective command because accelerometer integration drifts.
#define VERTICAL_KP_FULL_ERROR_FRACTION        0.18f
#define VERTICAL_OUTPUT_LIMIT_FRACTION         0.22f

// Nilai awal hover tidak mengikuti Max ESC GUI: Max ESC hanya mengatur headroom
// stick, bukan thrust hover. Sesuaikan setelah uji tethered bila diperlukan.
#define HOVER_THROTTLE_INITIAL_US             1260.0f
#define HOVER_THROTTLE_MARGIN_US              0.0f
#define HOVER_ADAPT_RATE_US_PER_S             0.25f
#define HOVER_ADAPT_MAX_VZ_MPS                0.10f
#define HOVER_ADAPT_MAX_ACCEL_MPS2            0.35f
#define HOVER_ADAPT_MAX_ATTITUDE_DEG          10.0f
#define COLLECTIVE_SLEW_US_PER_S              700.0f

#define VERTICAL_DEBUG                         0
#define VERTICAL_DEBUG_PERIOD_MS               200

struct PidParams {
  float angleKp, angleKi, angleKd;
  float rateKp, rateKi, rateKd;
  float yawKp, yawKi, yawKd;
  float maxAngle;
  float maxYawRate;
  float maxDeltaPwm;
  float escMinPwm;
  float escArmSpinPwm;
  float escMaxPwm;
};

/* ── LED Indikator ───────────────────────────────────────────────────── */
#define LED_PIN               PC4    // Aktif LOW

/* ── FreeRTOS Task Configuration (200 Hz Ultra-Fast Loop) ────────────── */
#define TASK_SENSOR_PERIOD_MS    5      // 200 Hz periodik (5ms - respons instan)
#define TASK_SENSOR_PRIORITY     3      // Prioritas tinggi (Realtime Sensor Read)
#define TASK_SENSOR_STACK_SIZE   256    // Stack size (words)

#define TASK_LORA_PERIOD_MS      5      // Polling periodik ~5 ms
#define TASK_LORA_PRIORITY       2      // Prioritas menengah (Radio Telemetri)
#define TASK_LORA_STACK_SIZE     384    // Stack size (words)

#define TASK_MOTOR_PERIOD_MS     5      // 200 Hz periodik (5ms - output PWM cepat)
#define TASK_MOTOR_PRIORITY      4      // Prioritas tertinggi (Motor PWM Output)
#define TASK_MOTOR_STACK_SIZE    256    // Stack size (words)

/* ── Baro Altitude Conditioning (Median -> EMA -> Slew-Rate Limiter) ──── */
#define BARO_EMA_ALPHA           0.20f   // Alpha EMA setelah median (0=lambat/halus, 1=cepat/kasar)
#define ALT_SLEW_MAX_MPS         2.5f    // Batas laju perubahan altitude (m/s) anti lompatan step
#define VZ_ACCEL_WEIGHT          0.75f   // Bobot integrasi accel pada fusi vz
#define VZ_BARO_WEIGHT           0.25f   // Bobot turunan baro pada fusi vz
#define ALT_VZ_DEADBAND          0.06f   // Deadband kecepatan vertikal (m/s)

/* ── Complementary Filter Attitude (Roll / Pitch) ────────────────────── */
#define CF_ALPHA                 0.98f  // Complementary filter (fusi accel + gyro)

/* ── Sensor Watchdog (BMI160 liveness) ───────────────────────────────── */
#define SENSOR_FAIL_THRESHOLD    5      // 5 x 10ms = 50ms tanpa data valid -> fail
#define SENSOR_RECOVER_COUNT     5      // 5 sukses berturut -> recovered

/* ── Gyro Calibration Robustness (variance check + retry) ─────────────── */
#define GYRO_CALIB_MAX_RETRY       3
#define GYRO_CALIB_VARIANCE_MAX    5.0f   // dps^2, threshold "drone diam"
#define GYRO_CALIB_RETRY_DELAY_MS  500

/* ── Zero-Motion Update (ZUPT) Runtime Gyro Bias Refresh ─────────────── */
#define ZUPT_ACCEL_TOLERANCE       0.3f   // m/s^2 dari 9.81
#define ZUPT_GYRO_MAGNITUDE_MAX    1.5f   // dps, sum |gx|+|gy|+|gz|
#define ZUPT_STABLE_SAMPLES        300    // 300 x 10ms = 3s stable
#define ZUPT_BIAS_LEARN_RATE       0.05f  // EMA alpha

/* ── Battery Low-Voltage Protection (3S LiPo, nominal 11.1V) ─────────── */
#define VBAT_WARN_V              10.8f  // 3.60 V/cell - WARN
#define VBAT_LIMIT_V             10.2f  // 3.40 V/cell - LIMIT
#define VBAT_CRITICAL_V           9.9f  // 3.30 V/cell - CRITICAL
#define VBAT_HYSTERESIS_MS        500   // Hysteresis stabilisasi stage

enum BattStage {
  BATT_OK       = 0,
  BATT_WARN     = 1,
  BATT_LIMIT    = 2,
  BATT_CRITICAL = 3
};

/* ── Protokol Paket Biner LoRa ───────────────────────────────────────── */
#define UPLINK_MAGIC          0xA5
#define DOWNLINK_MAGIC        0x5A
#define CONFIG_MAGIC          0xC3

#pragma pack(push, 1)
struct UplinkPacket {
  uint8_t  magic;          // == UPLINK_MAGIC (0xA5)
  int16_t  targetRoll;     // Derajat x100 (-2500 s.d. +2500 -> -25.00° s.d. +25.00°)
  int16_t  targetPitch;    // Derajat x100 (-2500 s.d. +2500 -> -25.00° s.d. +25.00°)
  int16_t  targetYaw;      // Deg/s x100 (-15000 s.d. +15000 -> -150.00°/s s.d. +150.00°/s)
  uint16_t targetThrottle; // Command stick throttle (0..255, 128 = netral)
  uint8_t  armed;          // 0 = DISARM, 1 = ARM
}; // Total: 10 byte (Sangat cepat & rendah latensi di LoRa)

struct DownlinkPacket {
  uint8_t  magic;   // == DOWNLINK_MAGIC (0x5A)
  int16_t  ax, ay, az;   // m/s^2  x100
  int16_t  gx, gy, gz;   // deg/s  x100
  uint16_t press;        // hPa    x10
  int16_t  alt;          // meter  x100
  int16_t  roll, pitch, yaw; // derajat x100, fusi onboard (200Hz)
  int16_t  uRoll, uPitch, uYaw; // koreksi kontrol PWM x100 (PID output)
  uint16_t vbat;         // Volts x100 (contoh: 1110 = 11.10V)
  uint8_t  flags;        // bit0=gyroCalibValid, bit1=bmiOK, bit2=bmpOK,
                         // bit3=vbatOK, bit4-5=battStage, bit6-7=failsafeStage
};  // total 30 bytes

struct ConfigPacket {
  uint8_t magic;         // harus == CONFIG_MAGIC (0xC3)
  float angleKp, angleKi, angleKd;
  float rateKp, rateKi, rateKd;
  float yawKp, yawKi, yawKd;
  float maxAngle;
  float maxYawRate;
  float maxDeltaPwm;
  float escMinPwm;
  float escArmSpinPwm;
  float escMaxPwm;
  float hoverThrottlePwm;
};
#pragma pack(pop)

/* ── Shared Telemetry Struct (Protected by Mutex) ─────────────────────── */
struct SensorData {
  float ax, ay, az;      // m/s^2
  float gx, gy, gz;      // deg/s
  float press;           // hPa
  float alt;             // meter relatif
  float vz;              // m/s kecepatan vertikal (climb/descent rate)
  float roll;            // derajat, complementary filter onboard
  float pitch;           // derajat, complementary filter onboard
  float yaw;             // derajat, integrasi gyro 200Hz onboard
  float yawRate;         // deg/s, untuk yaw-rate damping
  float vbat;            // Volt (baterai real-time)
  bool  bmiOK;
  bool  bmpOK;
  bool  vbatOK;
  bool  gyroCalibValid;  // false bila kalibrasi boot gagal (drone bergetar)
  uint8_t battStage;     // BATT_OK / WARN / LIMIT / CRITICAL
};

#endif // CONFIG_H
