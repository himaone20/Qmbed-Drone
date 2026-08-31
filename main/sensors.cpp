/* ==========================================================================
 * SENSORS.CPP — Driver I2C BMI160 + BMP180 (Kalman 1D), ADC VBat & TaskSensors
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

/* ── Kalman Filter 1D (Ported dari BMP180.ino) ───────────────────────── */
struct KalmanFilter1D {
  float q; // process noise covariance
  float r; // measurement noise covariance
  float x; // estimasi nilai
  float p; // estimasi error covariance
  float k; // kalman gain

  void init(float process_noise, float measurement_noise, float initial_value) {
    q = process_noise;
    r = measurement_noise;
    x = initial_value;
    p = 1.0f;
  }

  float update(float measurement) {
    p = p + q;
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1.0f - k) * p;
    return x;
  }
};

static KalmanFilter1D altitudeKF;

/* ========================= FUNGSI ADC VBAT ============================== */

float readVBat()
{
  const int N = 8;
  uint32_t sum = 0;
  for (int i = 0; i < N; i++) {
    sum += analogRead(VBAT_PIN);
    delayMicroseconds(200);
  }
  float raw = sum / (float)N;
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

  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_CONF, 0x28);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_RANGE, 0x03);
  delay(10);

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

static bool bmp180_read(float *press_hpa, float *alt_m)
{
  bool rtos_running = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);

  // 1. Baca Uncompensated Temperature (UT)
  i2c_write_reg(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP);
  if (rtos_running) {
    vTaskDelay(pdMS_TO_TICKS(5));
  } else {
    delay(5);
  }

  uint8_t t_buf[2];
  if (!i2c_read_regs(BMP180_ADDR, BMP180_REG_RESULT, t_buf, 2)) {
    return false;
  }
  int32_t ut = (int32_t)(t_buf[0] << 8 | t_buf[1]);

  // 2. Baca Uncompensated Pressure (UP)
  uint8_t press_cmd = BMP180_CMD_READ_PRESS + (bmp180_oss << 6);
  i2c_write_reg(BMP180_ADDR, BMP180_REG_CONTROL, press_cmd);
  if (rtos_running) {
    vTaskDelay(pdMS_TO_TICKS(8));
  } else {
    delay(8);
  }

  uint8_t p_buf[3];
  if (!i2c_read_regs(BMP180_ADDR, BMP180_REG_RESULT, p_buf, 3)) {
    return false;
  }
  int32_t up = (((int32_t)p_buf[0] << 16) | ((int32_t)p_buf[1] << 8) | (int32_t)p_buf[2]) >> (8 - bmp180_oss);

  // 3. Kalkulasi Kompensasi Bosch BMP180
  int32_t x1 = ((ut - (int32_t)cal180.ac6) * (int32_t)cal180.ac5) >> 15;
  int32_t x2 = ((int32_t)cal180.mc << 11) / (x1 + cal180.md);
  int32_t b5 = x1 + x2;

  int32_t b6 = b5 - 4000;
  x1 = (cal180.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (cal180.ac2 * b6) >> 11;
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
  // 1. Setup 12-bit ADC Voltage Sensor PA0
  analogReadResolution(12);
  pinMode(VBAT_PIN, INPUT_ANALOG);

  float init_vbat = readVBat();
  gSensorData.vbat = init_vbat;
  gSensorData.vbatOK = (init_vbat > 3.0f);
  Serial.print("Sensor Tegangan Baterai (PA0) init... ");
  Serial.print(init_vbat, 2);
  Serial.println(" V");

  // 2. Setup I2C Bus & IMU/Baro
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  Serial.print("BMI160 init... ");
  gSensorData.bmiOK = bmi160_init();
  Serial.println(gSensorData.bmiOK ? "OK" : "FAILED");

  if (gSensorData.bmiOK) {
    Serial.print("Kalibrasi zero-bias Gyro (jangan gerakkan drone)... ");
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int valid_samples = 0;
    const int SAMPLES = 250;
    for (int i = 0; i < SAMPLES; i++) {
      int16_t ax, ay, az, gx, gy, gz;
      if (bmi160_read(&ax, &ay, &az, &gx, &gy, &gz)) {
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        valid_samples++;
      }
      delay(3);
    }
    if (valid_samples > 50) {
      gyro_bias_gx = (float)sum_gx / valid_samples / 16.4f;
      gyro_bias_gy = (float)sum_gy / valid_samples / 16.4f;
      gyro_bias_gz = (float)sum_gz / valid_samples / 16.4f;
    }
    Serial.println("OK");
    Serial.print("  Bias Gyro: X="); Serial.print(gyro_bias_gx, 3);
    Serial.print(" Y="); Serial.print(gyro_bias_gy, 3);
    Serial.print(" Z="); Serial.print(gyro_bias_gz, 3);
    Serial.println(" dps");
  }

  Serial.print("BMP180 init... ");
  gSensorData.bmpOK = bmp180_init();
  if (gSensorData.bmpOK) {
    Serial.println("OK");
    Serial.print("Kalibrasi baseline Barometer + Kalman Filter 1D... ");
    float sum_a = 0.0f, p, a;
    int valid_baro = 0;
    for (int i = 0; i < 15; i++) {
      if (bmp180_read(&p, &a)) {
        sum_a += a;
        valid_baro++;
      }
    }
    if (valid_baro > 3) {
      float firstReading = sum_a / valid_baro;
      altitude_offset = firstReading;
      altitudeKF.init(0.01f, 0.50f, firstReading);
    } else {
      altitude_offset = 0.0f;
      altitudeKF.init(0.01f, 0.50f, 0.0f);
    }
    alt_filtered = 0.0f;
    Serial.println("OK");
    Serial.print("  Altitude baseline: ");
    Serial.print(altitude_offset, 2);
    Serial.println(" m");
  } else {
    Serial.println("FAILED");
  }

  return (gSensorData.bmiOK || gSensorData.bmpOK);
}

/* ========================= FREERTOS TASK 1: SENSORS ===================== */

void TaskSensors(void *pvParameters)
{
  (void) pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TASK_SENSOR_PERIOD_MS);

  float ax_f = 0, ay_f = 0, az_f = 9.81f;
  float gx_f = 0, gy_f = 0, gz_f = 0;
  float press_f = 1013.25f;
  float roll_f = 0, pitch_f = 0;
  float vbat_f = 11.1f;
  bool attitudeInitialized = false;

  for (;;)
  {
    /* Read ADC Voltage Sensor PA0 */
    float raw_vbat = readVBat();
    if (raw_vbat > 0.5f) {
      vbat_f = 0.90f * vbat_f + 0.10f * raw_vbat;
    }

    if (gSensorData.bmiOK) {
      int16_t ax, ay, az, gx, gy, gz;
      if (bmi160_read(&ax, &ay, &az, &gx, &gy, &gz)) {
        // BMI160: +/- 2G range -> 16384 LSB/g
        // Remap ke body frame standar (miring kanan -> roll > 0, depan naik -> pitch > 0):
        float raw_ax = ( ax / 16384.0f) * 9.80665f;  // body depan   <- sensor X (depan)
        float raw_ay = ( ay / 16384.0f) * 9.80665f;  // body roll    <- sensor Y (kiri, terangkat saat miring kanan)
        float raw_az = ( az / 16384.0f) * 9.80665f;  // body atas    <- sensor Z

        // BMI160: +/- 2000 dps range -> 16.4 LSB/dps
        float raw_gx = ( gx / 16.4f) - gyro_bias_gx;  // body roll rate  <- sensor X
        float raw_gy = ( gy / 16.4f) - gyro_bias_gy;  // body pitch rate <- sensor Y
        float raw_gz = ( gz / 16.4f) - gyro_bias_gz;  // yaw rate tidak diubah

        // Deadband filter: hilangkan noise mikro saat diam (< 0.12 dps)
        if (fabsf(raw_gx) < 0.12f) raw_gx = 0.0f;
        if (fabsf(raw_gy) < 0.12f) raw_gy = 0.0f;
        if (fabsf(raw_gz) < 0.12f) raw_gz = 0.0f;

        // IIR Low-Pass Filter (Alpha = 0.70) untuk meredam noise
        ax_f = 0.70f * ax_f + 0.30f * raw_ax;
        ay_f = 0.70f * ay_f + 0.30f * raw_ay;
        az_f = 0.70f * az_f + 0.30f * raw_az;

        gx_f = 0.70f * gx_f + 0.30f * raw_gx;
        gy_f = 0.70f * gy_f + 0.30f * raw_gy;
        gz_f = 0.70f * gz_f + 0.30f * raw_gz;

        float rollAcc = atan2f(ay_f, az_f) * 180.0f / PI;
        float pitchAcc = atan2f(ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * 180.0f / PI;
        if (!attitudeInitialized) {
          // Start from gravity angle so SMC never corrects a false 0-degree attitude.
          roll_f = rollAcc;
          pitch_f = pitchAcc;
          attitudeInitialized = true;
        } else {
          roll_f = SMC_CF_ALPHA * (roll_f + gx_f * (TASK_SENSOR_PERIOD_MS / 1000.0f)) +
            (1.0f - SMC_CF_ALPHA) * rollAcc;
          pitch_f = SMC_CF_ALPHA * (pitch_f + gy_f * (TASK_SENSOR_PERIOD_MS / 1000.0f)) +
            (1.0f - SMC_CF_ALPHA) * pitchAcc;
        }
      }
    }

    /* Read BMP180 Barometer (setiap 100ms / 10Hz) */
    static uint8_t baro_div = 0;
    if (gSensorData.bmpOK && (++baro_div >= 10)) {
      baro_div = 0;
      float raw_p, raw_a;
      if (bmp180_read(&raw_p, &raw_a)) {
        // Smooth altitude dengan Kalman Filter 1D (porting dari BMP180.ino)
        float kf_alt = altitudeKF.update(raw_a);
        float raw_rel_alt = kf_alt - altitude_offset;
        if (raw_rel_alt < 0.0f) {
          raw_rel_alt = 0.0f; // Nilai negatif dianggap 0 meter
        }
        alt_filtered = raw_rel_alt;
        press_f = 0.85f * press_f + 0.15f * raw_p;
      }
    }

    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      gSensorData.ax = ax_f;
      gSensorData.ay = ay_f;
      gSensorData.az = az_f;
      gSensorData.gx = gx_f;
      gSensorData.gy = gy_f;
      gSensorData.gz = gz_f;
      gSensorData.press = press_f;
      gSensorData.alt = alt_filtered;
      gSensorData.roll = roll_f;
      gSensorData.pitch = pitch_f;
      gSensorData.yawRate = gz_f;
      gSensorData.vbat = vbat_f;
      gSensorData.vbatOK = (vbat_f > 3.0f);
      xSemaphoreGive(sensorMutex);
    }

    vTaskDelayUntil(&lastWakeTime, period);
  }
}
