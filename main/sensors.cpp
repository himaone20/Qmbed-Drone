/* ==========================================================================
 * SENSORS.CPP — Driver I2C BMI160 + BMP180 (Non-Blocking), ADC VBat & Attitude Estimation
 * ==========================================================================
 */

#include "sensors.h"

/* ── Shared Sensor State & Mutex ─────────────────────────────────────── */
SensorData gSensorData;
SemaphoreHandle_t sensorMutex = NULL;

/* ── Gyro Offsets & Baro Baseline ─────────────────────────────────────── */
static float gyro_bias_gx = 0.0f;
static float gyro_bias_gy = 0.0f;
static float gyro_bias_gz = 0.0f;

static float altitude_offset = 0.0f;
static float alt_filtered = 0.0f;

/* ── Attitude State ───────────────────────────────────────────────────── */
static float roll_f = 0.0f;
static float pitch_f = 0.0f;
static float yaw_f = 0.0f;
static float gx_f = 0.0f;
static float gy_f = 0.0f;
static float gz_f = 0.0f;
static float ax_f = 0.0f;
static float ay_f = 0.0f;
static float az_f = 9.80665f;
static bool attitudeInitialized = false;
static unsigned long lastAttitudeUs = 0;

/* ── Baro Altitude Conditioning Pipeline ───────────────────────────────── */
struct MedianFilter5 {
  float buf[5];
  uint8_t count = 0;
  uint8_t idx = 0;

  float update(float sample) {
    buf[idx] = sample;
    idx = (idx + 1) % 5;
    if (count < 5) count++;

    float sorted[5];
    memcpy(sorted, buf, sizeof(float) * count);
    for (uint8_t i = 1; i < count; i++) {
      float key = sorted[i];
      int8_t j = i - 1;
      while (j >= 0 && sorted[j] > key) {
        sorted[j + 1] = sorted[j];
        j--;
      }
      sorted[j + 1] = key;
    }
    return sorted[count / 2];
  }

  void reset(float value) {
    for (uint8_t i = 0; i < 5; i++) buf[i] = value;
    count = 5;
    idx = 0;
  }
};

struct AltitudeConditioner {
  MedianFilter5 median;
  float ema = 0.0f;
  float slewed = 0.0f;
  bool initialized = false;

  float update(float raw_alt, float dt) {
    float med = median.update(raw_alt);
    if (!initialized) {
      ema = med;
      slewed = med;
      initialized = true;
      return slewed;
    }
    ema = (1.0f - BARO_EMA_ALPHA) * ema + BARO_EMA_ALPHA * med;

    float maxStep = ALT_SLEW_MAX_MPS * dt;
    float delta = constrain(ema - slewed, -maxStep, maxStep);
    slewed += delta;
    return slewed;
  }

  void reset(float value) {
    median.reset(value);
    ema = value;
    slewed = value;
    initialized = true;
  }
};

static AltitudeConditioner altitudeCond;

/* ========================= FUNGSI ADC VBAT (NON-BLOCKING) ================ */

float readVBat()
{
  uint32_t raw = analogRead(VBAT_PIN);
  float vadc = (raw / VBAT_ADC_RES) * VBAT_ADC_REF;
  return vadc * VBAT_DIV_RATIO * VBAT_CAL_FACTOR;
}

/* ========================= FUNGSI I2C LOW-LEVEL ========================= */

static void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

static uint8_t i2c_read_reg(uint8_t addr, uint8_t reg)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }
  if (Wire.requestFrom(addr, (uint8_t)1) == 1) {
    return Wire.read();
  }
  return 0;
}

static bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
  memset(buf, 0, len);
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  uint8_t count = Wire.requestFrom(addr, len);
  if (count < len) {
    return false;
  }
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
  return true;
}

/* ========================= BMI160 REGISTERS & DRIVER ==================== */
#define BMI160_REG_CHIP_ID    0x00
#define BMI160_REG_GYRO_X_L   0x0C
#define BMI160_REG_ACCEL_X_L  0x12
#define BMI160_REG_ACC_CONF   0x40
#define BMI160_REG_ACC_RANGE  0x41
#define BMI160_REG_GYR_CONF   0x42
#define BMI160_REG_GYR_RANGE  0x43
#define BMI160_REG_CMD        0x7E

#define BMI160_CMD_SOFT_RESET 0xB6
#define BMI160_CMD_ACC_NORMAL 0x11
#define BMI160_CMD_GYR_NORMAL 0x15

static bool bmi160_init(void)
{
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
  delay(100);

  uint8_t id = i2c_read_reg(BMI160_ADDR, BMI160_REG_CHIP_ID);
  if (id != 0xD1) {
    Serial.print("BMI160: wrong chip ID 0x");
    Serial.println(id, HEX);
    return false;
  }

  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
  delay(50);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
  delay(50);

  // Accel: 200Hz ODR (0x28), +/- 2G range (0x03)
  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_CONF, 0x28);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_RANGE, 0x03);
  delay(10);

  // Gyro: 200Hz ODR (0x28), +/- 2000 dps range (0x00)
  i2c_write_reg(BMI160_ADDR, BMI160_REG_GYR_CONF, 0x28);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_GYR_RANGE, 0x00);
  delay(50);

  return true;
}

static bool bmi160_read(int16_t *ax, int16_t *ay, int16_t *az,
                        int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t buf[12];
  if (!i2c_read_regs(BMI160_ADDR, BMI160_REG_GYRO_X_L, buf, 12)) {
    return false;
  }

  *gx = (int16_t)(buf[1] << 8 | buf[0]);
  *gy = (int16_t)(buf[3] << 8 | buf[2]);
  *gz = (int16_t)(buf[5] << 8 | buf[4]);

  *ax = (int16_t)(buf[7] << 8 | buf[6]);
  *ay = (int16_t)(buf[9] << 8 | buf[8]);
  *az = (int16_t)(buf[11] << 8 | buf[10]);

  return true;
}

/* ========================= BMP180 REGISTERS & DRIVER ==================== */
#define BMP180_REG_CAL_AC1    0xAA
#define BMP180_REG_CONTROL    0xF4
#define BMP180_REG_RESULT     0xF6

#define BMP180_CMD_READ_TEMP  0x2E
#define BMP180_CMD_READ_PRESS 0x34

struct bmp180_calib {
  int16_t  ac1;
  int16_t  ac2;
  int16_t  ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t  b1;
  int16_t  b2;
  int16_t  mb;
  int16_t  mc;
  int16_t  md;
};

static bmp180_calib cal180;
static const uint8_t bmp180_oss = 1; // Standard resolution (oss = 1, delay 7.5ms)

static bool bmp180_init(void)
{
  uint8_t buf[22];
  if (!i2c_read_regs(BMP180_ADDR, BMP180_REG_CAL_AC1, buf, 22)) {
    return false;
  }

  cal180.ac1 = (int16_t)(buf[0] << 8 | buf[1]);
  cal180.ac2 = (int16_t)(buf[2] << 8 | buf[3]);
  cal180.ac3 = (int16_t)(buf[4] << 8 | buf[5]);
  cal180.ac4 = (uint16_t)(buf[6] << 8 | buf[7]);
  cal180.ac5 = (uint16_t)(buf[8] << 8 | buf[9]);
  cal180.ac6 = (uint16_t)(buf[10] << 8 | buf[11]);
  cal180.b1  = (int16_t)(buf[12] << 8 | buf[13]);
  cal180.b2  = (int16_t)(buf[14] << 8 | buf[15]);
  cal180.mb  = (int16_t)(buf[16] << 8 | buf[17]);
  cal180.mc  = (int16_t)(buf[18] << 8 | buf[19]);
  cal180.md  = (int16_t)(buf[20] << 8 | buf[21]);

  if (cal180.ac1 == 0 || cal180.ac1 == -1 || cal180.ac5 == 0) {
    return false;
  }

  return true;
}

/* Non-blocking BMP180 State Machine: 100% bebas blocking delay */
static bool bmp180_poll(float &press_hpa, float &alt_m)
{
  static uint8_t phase = 0; // 0: Idle, 1: Reading temp, 2: Reading press
  static unsigned long last_start_ms = 0;
  static unsigned long due_ms = 0;
  static unsigned long last_temp_ms = 0;
  static int32_t cached_b5 = 0;
  static bool temp_valid = false;

  unsigned long now = millis();

  if (phase == 0) {
    if (now - last_start_ms < 50) return false; // 20 Hz baro poll
    last_start_ms = now;

    if (!temp_valid || (now - last_temp_ms >= 2000)) {
      i2c_write_reg(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP);
      due_ms = now + 6;
      phase = 1;
    } else {
      uint8_t press_cmd = BMP180_CMD_READ_PRESS + (bmp180_oss << 6);
      i2c_write_reg(BMP180_ADDR, BMP180_REG_CONTROL, press_cmd);
      due_ms = now + 9;
      phase = 2;
    }
    return false;
  }

  if ((long)(now - due_ms) < 0) {
    return false; // Sedang konversi hardware di sensor, jangan block CPU!
  }

  if (phase == 1) {
    uint8_t t_buf[2];
    if (i2c_read_regs(BMP180_ADDR, BMP180_REG_RESULT, t_buf, 2)) {
      int32_t ut = (int32_t)(t_buf[0] << 8 | t_buf[1]);
      int32_t x1 = ((ut - (int32_t)cal180.ac6) * (int32_t)cal180.ac5) >> 15;
      int32_t denom = x1 + cal180.md;
      if (denom != 0) {
        int32_t x2 = ((int32_t)cal180.mc << 11) / denom;
        cached_b5 = x1 + x2;
        temp_valid = true;
        last_temp_ms = now;
      }
    }
    uint8_t press_cmd = BMP180_CMD_READ_PRESS + (bmp180_oss << 6);
    i2c_write_reg(BMP180_ADDR, BMP180_REG_CONTROL, press_cmd);
    due_ms = now + 9;
    phase = 2;
    return false;
  }

  if (phase == 2) {
    phase = 0;
    uint8_t p_buf[3];
    if (!i2c_read_regs(BMP180_ADDR, BMP180_REG_RESULT, p_buf, 3)) {
      return false;
    }
    int32_t up = (((int32_t)p_buf[0] << 16) | ((int32_t)p_buf[1] << 8) | (int32_t)p_buf[2]) >> (8 - bmp180_oss);

    int32_t b5 = cached_b5;
    int32_t b6 = b5 - 4000;
    int32_t x1 = (cal180.b2 * ((b6 * b6) >> 12)) >> 11;
    int32_t x2 = (cal180.ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = ((((int32_t)cal180.ac1 * 4 + x3) << bmp180_oss) + 2) >> 2;

    x1 = (cal180.ac3 * b6) >> 13;
    x2 = (cal180.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = ((uint32_t)cal180.ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)up - b3) * (50000 >> bmp180_oss);

    int32_t p;
    if (b7 < 0x80000000) {
      p = (b7 * 2) / b4;
    } else {
      p = (b7 / b4) * 2;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    float press = p / 100.0f;
    if (press > 300.0f && press < 1200.0f) {
      press_hpa = press;
      alt_m = 44330.0f * (1.0f - powf(press / 1013.25f, 0.190284f));
      return true;
    }
  }
  return false;
}

/* Synchronous Baro Read for Setup Baseline Only */
static bool bmp180_read_sync(float *press_hpa, float *alt_m)
{
  i2c_write_reg(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP);
  delay(6);
  uint8_t t_buf[2];
  int32_t b5 = 0;
  if (i2c_read_regs(BMP180_ADDR, BMP180_REG_RESULT, t_buf, 2)) {
    int32_t ut = (int32_t)(t_buf[0] << 8 | t_buf[1]);
    int32_t x1 = ((ut - (int32_t)cal180.ac6) * (int32_t)cal180.ac5) >> 15;
    int32_t denom = x1 + cal180.md;
    if (denom != 0) {
      int32_t x2 = ((int32_t)cal180.mc << 11) / denom;
      b5 = x1 + x2;
    }
  }

  uint8_t press_cmd = BMP180_CMD_READ_PRESS + (bmp180_oss << 6);
  i2c_write_reg(BMP180_ADDR, BMP180_REG_CONTROL, press_cmd);
  delay(9);
  uint8_t p_buf[3];
  if (!i2c_read_regs(BMP180_ADDR, BMP180_REG_RESULT, p_buf, 3)) {
    return false;
  }
  int32_t up = (((int32_t)p_buf[0] << 16) | ((int32_t)p_buf[1] << 8) | (int32_t)p_buf[2]) >> (8 - bmp180_oss);

  int32_t b6 = b5 - 4000;
  int32_t x1 = (cal180.b2 * ((b6 * b6) >> 12)) >> 11;
  int32_t x2 = (cal180.ac2 * b6) >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = ((((int32_t)cal180.ac1 * 4 + x3) << bmp180_oss) + 2) >> 2;

  x1 = (cal180.ac3 * b6) >> 13;
  x2 = (cal180.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = ((uint32_t)cal180.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  uint32_t b7 = ((uint32_t)up - b3) * (50000 >> bmp180_oss);

  int32_t p = (b7 < 0x80000000) ? ((b7 * 2) / b4) : ((b7 / b4) * 2);
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p = p + ((x1 + x2 + 3791) >> 4);

  *press_hpa = p / 100.0f;
  if (*press_hpa > 300.0f && *press_hpa < 1200.0f) {
    *alt_m = 44330.0f * (1.0f - powf(*press_hpa / 1013.25f, 0.190284f));
    return true;
  }
  return false;
}

/* ========================= SENSORS INIT & CALIBRATION ==================== */

bool sensors_init()
{
  memset(&gSensorData, 0, sizeof(gSensorData));
  gSensorData.gyroCalibValid = false;

  analogReadResolution(12);
  pinMode(VBAT_PIN, INPUT_ANALOG);

  float init_vbat = readVBat();
  gSensorData.vbat = init_vbat;
  gSensorData.vbatOK = (init_vbat > 3.0f);
  gSensorData.battStage = BATT_OK;
  Serial.print("Sensor Tegangan Baterai (PA0) init... ");
  Serial.print(init_vbat, 2);
  Serial.println(" V");

  Wire.setSCL(I2C_SCL_PIN);
  Wire.setSDA(I2C_SDA_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  Serial.print("Sensor IMU BMI160 (0x68) init... ");
  gSensorData.bmiOK = bmi160_init();
  Serial.println(gSensorData.bmiOK ? "OK" : "FAILED");

  if (gSensorData.bmiOK) {
    const int SAMPLES = 250;
    bool calibOK = false;
    float mean_x = 0, mean_y = 0, mean_z = 0;

    for (int attempt = 1; attempt <= GYRO_CALIB_MAX_RETRY && !calibOK; attempt++) {
      Serial.print("Kalibrasi Gyro attempt "); Serial.print(attempt);
      Serial.print("/"); Serial.print(GYRO_CALIB_MAX_RETRY);
      Serial.print(" (drone HARUS diam)... ");

      double sum_gx = 0, sum_gy = 0, sum_gz = 0;
      float samples_gx[SAMPLES], samples_gy[SAMPLES], samples_gz[SAMPLES];
      int valid_samples = 0;

      for (int i = 0; i < SAMPLES; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        if (bmi160_read(&ax, &ay, &az, &gx, &gy, &gz)) {
          float gx_f_raw = (float)gx / 16.4f;
          float gy_f_raw = (float)gy / 16.4f;
          float gz_f_raw = (float)gz / 16.4f;
          samples_gx[valid_samples] = gx_f_raw;
          samples_gy[valid_samples] = gy_f_raw;
          samples_gz[valid_samples] = gz_f_raw;
          sum_gx += gx_f_raw;
          sum_gy += gy_f_raw;
          sum_gz += gz_f_raw;
          valid_samples++;
        }
        delay(3);
      }

      if (valid_samples < 50) {
        Serial.println("FAIL (sampel valid < 50, sensor error)");
        delay(GYRO_CALIB_RETRY_DELAY_MS);
        continue;
      }

      mean_x = sum_gx / valid_samples;
      mean_y = sum_gy / valid_samples;
      mean_z = sum_gz / valid_samples;

      double var_x = 0, var_y = 0, var_z = 0;
      for (int i = 0; i < valid_samples; i++) {
        float dx = samples_gx[i] - mean_x;
        float dy = samples_gy[i] - mean_y;
        float dz = samples_gz[i] - mean_z;
        var_x += dx * dx;
        var_y += dy * dy;
        var_z += dz * dz;
      }
      var_x /= valid_samples;
      var_y /= valid_samples;
      var_z /= valid_samples;
      float total_var = (float)(var_x + var_y + var_z);

      if (total_var <= GYRO_CALIB_VARIANCE_MAX) {
        gyro_bias_gx = mean_x;
        gyro_bias_gy = mean_y;
        gyro_bias_gz = mean_z;
        gSensorData.gyroCalibValid = true;
        calibOK = true;
        Serial.print("OK (variance="); Serial.print(total_var, 2);
        Serial.println(")");
      } else {
        Serial.print("VARIANCE TOO HIGH ("); Serial.print(total_var, 2);
        Serial.print(" > "); Serial.print(GYRO_CALIB_VARIANCE_MAX, 1);
        Serial.println("), retry...");
        delay(GYRO_CALIB_RETRY_DELAY_MS);
      }
    }

    if (!calibOK) {
      gyro_bias_gx = 0.0f;
      gyro_bias_gy = 0.0f;
      gyro_bias_gz = 0.0f;
      gSensorData.gyroCalibValid = false;
      Serial.println("!! [CAL] GYRO FAIL: drone bergetar saat init, bias di-reset ke 0 !!");
    }

    Serial.print("[CAL] Bias Gyro: X="); Serial.print(gyro_bias_gx, 3);
    Serial.print(" Y="); Serial.print(gyro_bias_gy, 3);
    Serial.print(" Z="); Serial.println(gyro_bias_gz, 3);
  }

  Serial.print("Sensor Barometer BMP180 (0x77) init... ");
  gSensorData.bmpOK = bmp180_init();
  if (gSensorData.bmpOK) {
    Serial.println("OK");
    Serial.print("Kalibrasi baseline Barometer (Median, tahan spike)... ");
    const int BASELINE_SAMPLES = 15;
    float samples[BASELINE_SAMPLES];
    float p, a;
    int valid_baro = 0;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
      if (bmp180_read_sync(&p, &a)) {
        samples[valid_baro] = a;
        valid_baro++;
      }
    }
    if (valid_baro > 3) {
      for (int i = 1; i < valid_baro; i++) {
        float key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
          samples[j + 1] = samples[j];
          j--;
        }
        samples[j + 1] = key;
      }
      float firstReading = samples[valid_baro / 2];
      altitude_offset = firstReading;
      altitudeCond.reset(firstReading);
    } else {
      altitude_offset = 0.0f;
      altitudeCond.reset(0.0f);
    }
    alt_filtered = 0.0f;
    Serial.println("OK");
    Serial.print("  Altitude baseline: ");
    Serial.print(altitude_offset, 2);
    Serial.println(" m");
  } else {
    Serial.println("FAILED");
  }

  lastAttitudeUs = micros();
  return (gSensorData.bmiOK || gSensorData.bmpOK);
}

/* ========================= SYNCHRONOUS 200Hz IMU READ & ATTITUDE ========= */

bool sensors_step_imu(SensorData &outData)
{
  if (!gSensorData.bmiOK) {
    return false;
  }

  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  if (!bmi160_read(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw)) {
    return false;
  }

  // Exact dt measurement using micros()
  unsigned long nowUs = micros();
  float dt = (lastAttitudeUs > 0) ? (float)(nowUs - lastAttitudeUs) * 1e-6f : 0.005f;
  lastAttitudeUs = nowUs;
  if (dt < 0.0005f || dt > 0.050f) dt = 0.005f;

  // 1. Convert BMI160 raw to metric / dps units:
  // Accel: +/- 2G -> 16384 LSB/g
  float raw_ax = ((float)ax_raw / 16384.0f) * 9.80665f; // Sensor X (Depan)
  float raw_ay = ((float)ay_raw / 16384.0f) * 9.80665f; // Sensor Y (Kiri)
  float raw_az = ((float)az_raw / 16384.0f) * 9.80665f; // Sensor Z (Atas)

  // Gyro: +/- 2000 dps -> 16.4 LSB/dps
  float raw_gx = ((float)gx_raw / 16.4f) - gyro_bias_gx;
  float raw_gy = ((float)gy_raw / 16.4f) - gyro_bias_gy;
  float raw_gz = ((float)gz_raw / 16.4f) - gyro_bias_gz;

  // 2. Transform to Body Frame Coordinates:
  // - Roll (X-axis): Miring KANAN -> Sensor Y naik (+Y) -> raw_ay > 0, roll > 0, gx_body > 0
  // - Pitch (Y-axis): Hidung NAIK (Nose-Up) -> Sensor X naik (+X) -> reaksi pegas memanjang ke depan -> raw_ax > 0!
  //   Maka pitchAcc = atan2f(ax_f, ...) agar Nose-Up bernilai POSITIF (+pitch > 0)!
  //   Saat Nose-Up, rotasi mengelilingi sumbu Kiri (+Y) adalah negatif, maka gy_body = -raw_gy (> 0 saat nose up)!
  // - Yaw (Z-axis): Putar KANAN (CW / Heading naik) -> sensor Z negatif -> maka gz_body = -raw_gz (> 0 saat putar kanan)!
  float gx_body = raw_gx;
  float gy_body = -raw_gy;
  float gz_body = -raw_gz;

  // Deadband filter: bersihkan noise sensor mikro saat diam (< 0.10 dps)
  if (fabsf(gx_body) < 0.10f) gx_body = 0.0f;
  if (fabsf(gy_body) < 0.10f) gy_body = 0.0f;
  if (fabsf(gz_body) < 0.10f) gz_body = 0.0f;

  // Gyro Fast-Response Filter (respons cepat < 1ms untuk Inner Rate PID)
  gx_f = 0.15f * gx_f + 0.85f * gx_body;
  gy_f = 0.15f * gy_f + 0.85f * gy_body;
  gz_f = 0.15f * gz_f + 0.85f * gz_body;

  // Accel Filter
  ax_f = 0.40f * ax_f + 0.60f * raw_ax;
  ay_f = 0.40f * ay_f + 0.60f * raw_ay;
  az_f = 0.40f * az_f + 0.60f * raw_az;

  // Accel Angles (Roll & Pitch)
  float rollAcc  = atan2f(ay_f, az_f) * (180.0f / PI);
  float pitchAcc = atan2f(ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * (180.0f / PI);

  // 3. Fusi Attitude Complementary Filter
  // Dynamic alpha based on dt (tau ~ 0.50 s) -> alpha = tau / (tau + dt)
  float tau_cf = 0.50f;
  float alpha_cf = tau_cf / (tau_cf + dt);

  if (!attitudeInitialized) {
    roll_f = rollAcc;
    pitch_f = pitchAcc;
    yaw_f = 0.0f;
    attitudeInitialized = true;
  } else {
    roll_f  = alpha_cf * (roll_f  + gx_f * dt) + (1.0f - alpha_cf) * rollAcc;
    pitch_f = alpha_cf * (pitch_f + gy_f * dt) + (1.0f - alpha_cf) * pitchAcc;

    // 4. Integrasi Yaw Onboard 200 Hz presisi tinggi (bebas aliasing transmisi radio lambat)
    if (fabsf(gz_f) >= 0.10f) {
      yaw_f += gz_f * dt;
    }
    if (yaw_f > 180.0f) {
      yaw_f -= 360.0f;
    } else if (yaw_f < -180.0f) {
      yaw_f += 360.0f;
    }
  }

  // Populate output
  outData.ax = ax_f;
  outData.ay = ay_f;
  outData.az = az_f;
  outData.gx = gx_f;       // deg/s (body roll rate)
  outData.gy = gy_f;       // deg/s (body pitch rate)
  outData.gz = gz_f;       // deg/s (body yaw rate)
  outData.roll = roll_f;   // degrees (+ when bank right)
  outData.pitch = pitch_f; // degrees (+ when nose up)
  outData.yaw = yaw_f;     // degrees (+ when turning right/CW)
  outData.yawRate = gz_f;
  outData.bmiOK = true;
  outData.gyroCalibValid = gSensorData.gyroCalibValid;

  return true;
}

/* ========================= NON-BLOCKING TELEMETRY POLLING ================ */

void sensors_poll_telemetry()
{
  static unsigned long last_vbat_ms = 0;
  static unsigned long last_baro_ms = 0;
  static float prev_alt = 0.0f;
  static float vz_filtered = 0.0f;
  static float vbat_f = 11.1f;
  static uint8_t batt_committed_stage = BATT_OK;
  static uint8_t batt_candidate_stage = BATT_OK;
  static unsigned long batt_candidate_start_ms = 0;

  unsigned long now = millis();

  // 1. Poll VBat ADC setiap 100ms (Non-blocking, 0 delay)
  if (now - last_vbat_ms >= 100) {
    last_vbat_ms = now;
    float raw_vbat = readVBat();
    if (raw_vbat > 0.5f) {
      vbat_f = 0.85f * vbat_f + 0.15f * raw_vbat;
    }

    uint8_t new_stage;
    if      (vbat_f < VBAT_CRITICAL_V) new_stage = BATT_CRITICAL;
    else if (vbat_f < VBAT_LIMIT_V)    new_stage = BATT_LIMIT;
    else if (vbat_f < VBAT_WARN_V)     new_stage = BATT_WARN;
    else                                new_stage = BATT_OK;

    if (new_stage != batt_candidate_stage) {
      batt_candidate_stage = new_stage;
      batt_candidate_start_ms = now;
    } else if (new_stage != batt_committed_stage &&
               (now - batt_candidate_start_ms) >= VBAT_HYSTERESIS_MS) {
      batt_committed_stage = new_stage;
    }
  }

  // 2. Poll Barometer BMP180 (Non-blocking state machine)
  float raw_p, raw_a;
  if (gSensorData.bmpOK && bmp180_poll(raw_p, raw_a)) {
    float dt_baro = (last_baro_ms > 0) ? (now - last_baro_ms) / 1000.0f : 0.05f;
    last_baro_ms = now;
    if (dt_baro < 0.005f || dt_baro > 0.5f) dt_baro = 0.05f;

    float cond_alt = altitudeCond.update(raw_a, dt_baro);
    float raw_rel_alt = cond_alt - altitude_offset;
    if (raw_rel_alt < 0.0f) raw_rel_alt = 0.0f;

    float baro_vz = (raw_rel_alt - prev_alt) / dt_baro;
    prev_alt = raw_rel_alt;
    if (fabsf(baro_vz) < ALT_VZ_DEADBAND) baro_vz = 0.0f;

    vz_filtered = VZ_ACCEL_WEIGHT * vz_filtered + VZ_BARO_WEIGHT * baro_vz;
    alt_filtered = raw_rel_alt;

    if (sensorMutex && xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
      gSensorData.press = raw_p;
      gSensorData.alt = alt_filtered;
      gSensorData.vz = vz_filtered;
      xSemaphoreGive(sensorMutex);
    }
  }

  // 3. Update VBat di gSensorData
  if (sensorMutex && xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
    gSensorData.vbat = vbat_f;
    gSensorData.vbatOK = (vbat_f > 3.0f);
    gSensorData.battStage = batt_committed_stage;
    xSemaphoreGive(sensorMutex);
  }
}

/* ========================= LEGACY TaskSensors (Optional) ================= */

void TaskSensors(void *pvParameters)
{
  (void) pvParameters;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
