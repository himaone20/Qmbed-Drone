/* ==========================================================================
 * SIMPLE SENSOR READ - Orientasi (Roll/Pitch/Yaw) + Altitude Relatif
 * Target Board : STM32F401RCT6
 * --------------------------------------------------------------------------
 * IMU  : BMI160 (accel + gyro) -> complementary filter -> ROLL, PITCH, YAW
 *        Driver: register-level via Wire.h.
 * Baro : BMP280 -> tekanan (hPa) + altitude relatif (m) terhadap baseline
 *                  yang diambil saat power-on.
 *        Driver: Adafruit_BMP280 library (install via Library Manager:
 *                "Adafruit BMP280 Library" + "Adafruit Unified Sensor").
 *
 * Interface:
 *   I2C   : PB10 (SCL) + PB3 (SDA) @ 400 kHz
 *   Serial: PA9 (TX) + PA10 (RX)   @ 115200 baud
 *
 * Alamat I2C:
 *   BMI160 : 0x68
 *   BMP280 : 0x76
 *
 * Timing:
 *   IMU  : sampling + integrasi orientasi @ 50 Hz (tiap 20 ms)
 *   Send : kirim serial @ 20 Hz (tiap 50 ms)
 *
 * Format Output:
 *   SENS ROLL:<f> PITCH:<f> YAW:<f> P:<f> ALT:<f>
 *     - ROLL/PITCH/YAW : derajat
 *     - P              : hPa
 *     - ALT            : meter (relatif ke tekanan saat startup)
 *
 * Perintah Serial (dari GUI):
 *   'r' atau 'R' : reset baseline altitude    -> ACK BASELINE=<hPa>
 *   'y' atau 'Y' : reset yaw = 0              -> ACK YAW=0.00
 *
 * Catatan:
 *   Yaw dihitung murni dari integrasi gyro Z, sehingga akan mengalami DRIFT
 *   perlahan (tidak ada magnetometer di BMI160). Gunakan tombol reset yaw
 *   di GUI untuk re-zero secara manual.
 * ==========================================================================
 */

#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Adafruit_BMP280.h>

/* ── Konfigurasi Pin & Bus ───────────────────────────────────────────── */
#define SERIAL_TX_PIN   PA9
#define SERIAL_RX_PIN   PA10
#define SERIAL_BAUD     115200

#define I2C_SCL_PIN     PB10
#define I2C_SDA_PIN     PB3
#define I2C_CLOCK_SPEED 400000

#define BMI160_ADDR     0x68
#define BMP280_ADDR     0x76

/* ── Timing ──────────────────────────────────────────────────────────── */
#define IMU_PERIOD_MS     20      // 50 Hz internal (baca + fusi)
#define SEND_PERIOD_MS    50      // 20 Hz kirim serial
#define COMP_ALPHA        0.98f   // complementary filter (gyro-heavy)
#define RAD_TO_DEG_F      57.2957795f

/* ── Register BMI160 ─────────────────────────────────────────────────── */
#define BMI160_REG_CHIP_ID     0x00
#define BMI160_REG_GYRO_X_L    0x0C
#define BMI160_REG_ACCEL_X_L   0x12
#define BMI160_REG_ACC_CONF    0x40
#define BMI160_REG_ACC_RANGE   0x41
#define BMI160_REG_GYR_CONF    0x42
#define BMI160_REG_GYR_RANGE   0x43
#define BMI160_REG_CMD         0x7E

#define BMI160_CMD_SOFT_RESET  0xB6
#define BMI160_CMD_ACC_NORMAL  0x11
#define BMI160_CMD_GYR_NORMAL  0x15

/* ── Instance BMP280 (Adafruit library, mode I2C) ────────────────────── */
Adafruit_BMP280 bmp;

bool  bmiOK = false;
bool  bmpOK = false;
float press_baseline_hpa = 1013.25f;

/* ── State Orientasi ─────────────────────────────────────────────────── */
float    roll_deg  = 0.0f;
float    pitch_deg = 0.0f;
float    yaw_deg   = 0.0f;
uint32_t last_us   = 0;

/* ── State cache untuk kirim serial ──────────────────────────────────── */
float last_press_hpa = 0.0f;
float last_alt_m     = 0.0f;

/* ── Scheduler non-blocking ──────────────────────────────────────────── */
uint32_t next_imu_ms  = 0;
uint32_t next_send_ms = 0;

/* ========================= FUNGSI I2C LOW-LEVEL ========================= */
void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t i2c_read_reg(uint8_t addr, uint8_t reg)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(addr, (uint8_t)1) == 1) {
    return Wire.read();
  }
  return 0;
}

bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
  memset(buf, 0, len);
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t count = Wire.requestFrom(addr, len);
  if (count < len) return false;
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
  return true;
}

/* ========================= DRIVER BMI160 ============================ */
bool bmi160_init(void)
{
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
  delay(100);

  uint8_t id = i2c_read_reg(BMI160_ADDR, BMI160_REG_CHIP_ID);
  if (id != 0xD1) {
    Serial.print("# BMI160 Chip ID salah (0x");
    Serial.print(id, HEX);
    Serial.println(")");
    return false;
  }

  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
  delay(50);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
  delay(50);

  // +/- 2G  -> 16384 LSB/g   ;   +/- 2000 dps -> 16.4 LSB/dps
  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_CONF,  0x28);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_RANGE, 0x03);
  delay(10);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_GYR_CONF,  0x28);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_GYR_RANGE, 0x00);
  delay(50);

  return true;
}

bool bmi160_read(float *ax, float *ay, float *az,
                 float *gx, float *gy, float *gz)
{
  uint8_t buf[12];
  if (!i2c_read_regs(BMI160_ADDR, BMI160_REG_GYRO_X_L, buf, 12)) {
    return false;
  }

  int16_t rgx = (int16_t)(buf[1]  << 8 | buf[0]);
  int16_t rgy = (int16_t)(buf[3]  << 8 | buf[2]);
  int16_t rgz = (int16_t)(buf[5]  << 8 | buf[4]);
  int16_t rax = (int16_t)(buf[7]  << 8 | buf[6]);
  int16_t ray = (int16_t)(buf[9]  << 8 | buf[8]);
  int16_t raz = (int16_t)(buf[11] << 8 | buf[10]);

  *ax = (rax / 16384.0f) * 9.80665f;
  *ay = (ray / 16384.0f) * 9.80665f;
  *az = (raz / 16384.0f) * 9.80665f;

  *gx = rgx / 16.4f;
  *gy = rgy / 16.4f;
  *gz = rgz / 16.4f;

  return true;
}

/* ========================= DRIVER BMP280 (Adafruit library) =========== */
bool bmp280_init(void)
{
  // NOTE: Wire.begin() sudah dipanggil di setup() sebelum fungsi ini.
  //       Library Adafruit akan pakai instance Wire default.
  if (!bmp.begin(BMP280_ADDR)) {
    Serial.println("# BMP280 begin() gagal");
    return false;
  }

  // Preset drone standard: mode NORMAL, oversampling P=x16, filter=x16.
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,     // temperature
    Adafruit_BMP280::SAMPLING_X16,    // pressure
    Adafruit_BMP280::FILTER_X16,      // IIR filter
    Adafruit_BMP280::STANDBY_MS_500   // standby time
  );

  return true;
}

bool bmp280_read_pressure(float *press_hpa)
{
  float p = bmp.readPressure() / 100.0f;   // Pascal -> hPa
  if (p < 300.0f || p > 1200.0f) return false;
  *press_hpa = p;
  return true;
}

float compute_alt_rel(float press_hpa)
{
  if (press_baseline_hpa <= 0.0f) return 0.0f;
  return 44330.0f * (1.0f - powf(press_hpa / press_baseline_hpa, 0.190284f));
}

void calibrate_baseline(void)
{
  float sum = 0.0f;
  int   n   = 0;
  for (int i = 0; i < 10; i++) {
    float p;
    if (bmp280_read_pressure(&p)) {
      sum += p;
      n++;
    }
    delay(50);
  }
  if (n > 0) {
    press_baseline_hpa = sum / (float)n;
  }
}

/* ========================= FUSI ORIENTASI =============================== */
void update_orientation(float ax, float ay, float az,
                        float gx, float gy, float gz)
{
  uint32_t now = micros();
  float dt = (last_us == 0) ? 0.0f : (now - last_us) * 1e-6f;
  last_us = now;
  if (dt <= 0.0f || dt > 0.2f) return;   // proteksi glitch/first sample

  // Sudut dari accel (derajat)
  float roll_acc  = atan2f(ay, az) * RAD_TO_DEG_F;
  float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;

  // Complementary filter (gx/gy/gz sudah dps -> hasilnya derajat)
  roll_deg  = COMP_ALPHA * (roll_deg  + gx * dt) + (1.0f - COMP_ALPHA) * roll_acc;
  pitch_deg = COMP_ALPHA * (pitch_deg + gy * dt) + (1.0f - COMP_ALPHA) * pitch_acc;
  yaw_deg  += gz * dt;

  // Normalisasi yaw ke [-180, 180]
  if (yaw_deg >  180.0f) yaw_deg -= 360.0f;
  if (yaw_deg < -180.0f) yaw_deg += 360.0f;
}

/* ========================= SETUP ========================================= */
void setup()
{
  Serial.setTx(SERIAL_TX_PIN);
  Serial.setRx(SERIAL_RX_PIN);
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  Serial.println("# SimpleSensorRead booting...");

  Serial.print("# BMI160 init: ");
  bmiOK = bmi160_init();
  Serial.println(bmiOK ? "OK" : "FAIL");

  Serial.print("# BMP280 init: ");
  bmpOK = bmp280_init();
  Serial.println(bmpOK ? "OK" : "FAIL");

  if (bmpOK) {
    Serial.println("# Kalibrasi baseline tekanan...");
    calibrate_baseline();
    Serial.print("# baseline=");
    Serial.print(press_baseline_hpa, 2);
    Serial.println(" hPa");
  }

  // Inisialisasi orientasi awal dari accel (kalau IMU tersedia)
  if (bmiOK) {
    float ax, ay, az, gx, gy, gz;
    if (bmi160_read(&ax, &ay, &az, &gx, &gy, &gz)) {
      roll_deg  = atan2f(ay, az) * RAD_TO_DEG_F;
      pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;
      yaw_deg   = 0.0f;
    }
  }
  last_us = micros();

  next_imu_ms  = millis();
  next_send_ms = millis();

  Serial.println("# ready. Format: SENS ROLL PITCH YAW P ALT");
}

/* ========================= LOOP =========================================== */
void loop()
{
  // ── Handle perintah dari GUI ─────────────────────────────────────────
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'r' || c == 'R') {
      if (bmpOK) {
        calibrate_baseline();
        Serial.print("ACK BASELINE=");
        Serial.println(press_baseline_hpa, 2);
      } else {
        Serial.println("ACK BASELINE=NA");
      }
      // reset timing setelah delay panjang di calibrate_baseline()
      last_us      = micros();
      next_imu_ms  = millis();
      next_send_ms = millis();
    } else if (c == 'y' || c == 'Y') {
      yaw_deg = 0.0f;
      Serial.println("ACK YAW=0.00");
    }
  }

  uint32_t now = millis();

  // ── 50 Hz : baca sensor + update orientasi + cache baro ──────────────
  if ((int32_t)(now - next_imu_ms) >= 0) {
    next_imu_ms += IMU_PERIOD_MS;

    if (bmiOK) {
      float ax, ay, az, gx, gy, gz;
      if (bmi160_read(&ax, &ay, &az, &gx, &gy, &gz)) {
        update_orientation(ax, ay, az, gx, gy, gz);
      }
    }

    if (bmpOK) {
      float p;
      if (bmp280_read_pressure(&p)) {
        last_press_hpa = p;
        last_alt_m     = compute_alt_rel(p);
      }
    }
  }

  // ── 20 Hz : kirim data ke serial ────────────────────────────────────
  if ((int32_t)(now - next_send_ms) >= 0) {
    next_send_ms += SEND_PERIOD_MS;

    Serial.print("SENS ROLL:"); Serial.print(roll_deg, 2);
    Serial.print(" PITCH:");    Serial.print(pitch_deg, 2);
    Serial.print(" YAW:");      Serial.print(yaw_deg, 2);
    Serial.print(" P:");        Serial.print(last_press_hpa, 2);
    Serial.print(" ALT:");      Serial.println(last_alt_m, 2);
  }
}
