/* ==========================================================================
 * MAIN.INO — Flight Controller Drone (FreeRTOS Base)
 * Target Board: STM32F401RCT6 (Arduino IDE + STM32duino + STM32FreeRTOS)
 *
 * Menggunakan basic program & driver langsung dari LoraRx.ino yang teruji:
 *  - FreeRTOS multitasking (TaskSensors 100Hz + TaskLoRa_Control)
 *  - Sensor IMU BMI160 + Barometer BMP280 (I2C: PB10/PB3)
 *  - Komunikasi 2 Arah LoRa RA-02 (SPI2: PB12-15, PB0, PB1 @ 433MHz)
 *  - Kontrol 4 ESC Motor: PB6 (M1), PB7 (M2), PB8 (M3), PB9 (M4)
 *  - Indikator LED PC4 (Arming blink 1Hz, Armed solid ON, Disarm OFF)
 *
 * ORIENTASI IMU BMI160: dipasang dengan X_sensor = DEPAN drone,
 * Y_sensor = KIRI, Z_sensor = ATAS (right-handed). Di TaskSensors,
 * sumbu di-remap ke body frame standar (Y_body = kanan -> negasi Y):
 *   ax_body =  ax_sensor  (depan)   |   gx_body =  gx_sensor  (roll rate)
 *   ay_body = -ay_sensor  (kanan)   |   gy_body = -gy_sensor  (pitch rate)
 *   az_body =  az_sensor  (atas)    |   gz_body =  gz_sensor  (yaw rate, tdk diubah)
 * ==========================================================================
 */

#include <STM32FreeRTOS.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>
#include <string.h>

#include "config.h"

/* ── Objek Hardware SPI2 & ESC ────────────────────────────────────────── */
SPIClass SPI_2(LORA_MOSI_PIN, LORA_MISO_PIN, LORA_SCK_PIN);   // PB15, PB14, PB13

Servo motors[4];
const int MOTOR_PINS[4] = { MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN };

/* ── State Machine ESC & Link ────────────────────────────────────────── */
enum EscState {
  ESC_DISARMED,
  ESC_ARMING,
  ESC_ARMED
};

EscState escState = ESC_DISARMED;
unsigned long armingStartMs = 0;

unsigned long lastLinkMs = 0;
bool linkOK = false;
float altitude_offset = 0.0f;
float alt_filtered = 0.0f;

/* Perintah joystick valid terakhir dari remote. Nilai netral aman saat boot. */
uint8_t lastRollCmd = 128;
uint8_t lastThrottleCmd = 0;
uint8_t lastYawCmd = 128;
uint8_t lastPitchCmd = 128;
float throttleSmoothed = ESC_ARM_SPIN_US;
unsigned long throttleRampLastMs = 0;
float lastURoll = 0.0f;
float lastUPitch = 0.0f;
float lastUYaw = 0.0f;

struct SmcParams {
  float k1;
  float k2;
  float eps;
  float forceToPwm;
  float deltaMaxPwm;
};

SmcParams gSmcParams = {
  SMC_K1_DEFAULT,
  SMC_K2_DEFAULT,
  SMC_EPS_DEFAULT,
  SMC_FORCE_TO_PWM_DEFAULT,
  SMC_DELTA_MAX_DEFAULT
};
SemaphoreHandle_t smcMutex = NULL;

/* ── Gyro Zero-Bias Calibration Offsets ───────────────────────────────── */
float gyro_bias_gx = 0.0f;
float gyro_bias_gy = 0.0f;
float gyro_bias_gz = 0.0f;

/* ── FreeRTOS Shared State & Mutex ────────────────────────────────────── */
SensorData gSensorData;
SemaphoreHandle_t sensorMutex = NULL;

/* ========================= FUNGSI KONTROL ESC =========================== */

void setAllMotorsPWM(int us)
{
  if (us < ESC_MIN_US) us = ESC_MIN_US;
  if (us > ESC_MAX_US) us = ESC_MAX_US;
  for (int i = 0; i < 4; i++) {
    motors[i].writeMicroseconds(us);
  }
}

void updateThrottleCommand(uint8_t t)
{
  int throttleDelta = (int)t - 128;
  if (abs(throttleDelta) <= THROTTLE_DEADBAND) {
    throttleDelta = 0;
  }

  unsigned long now = millis();
  float elapsedSeconds = (now - throttleRampLastMs) / 1000.0f;
  throttleRampLastMs = now;

  // Stick tengah tidak mengubah PWM; nilai throttle terakhir tetap dipakai.
  float stickNorm = throttleDelta >= 0 ?
    (float)throttleDelta / 127.0f : (float)throttleDelta / 128.0f;
  throttleSmoothed += stickNorm * THROTTLE_RATE_US_PER_S * elapsedSeconds;
  throttleSmoothed = constrain(throttleSmoothed,
                               (float)ESC_MIN_US, (float)ESC_MAX_US);
}

void writeSmcMotorMix(float basePwm, float uRoll, float uPitch, float uYaw)
{
  // Quad-X, arah putaran nyata: M1 FL CW, M2 FR CCW, M3 BR CW, M4 BL CCW.
  // uRoll + = naikkan kiri (dikoreksi dari tanda sensor), uPitch + = naikkan depan,
  // uYaw + = meredam rotasi (CW motor naik, CCW motor turun).
  int motorPwm[4] = {
    (int)(basePwm + uRoll + uPitch + uYaw),  // M1 FL CW
    (int)(basePwm - uRoll + uPitch - uYaw),  // M2 FR CCW
    (int)(basePwm - uRoll - uPitch + uYaw),  // M3 BR CW
    (int)(basePwm + uRoll - uPitch - uYaw)   // M4 BL CCW
  };

  for (int i = 0; i < 4; i++) {
    motorPwm[i] = constrain(motorPwm[i], ESC_MIN_US, ESC_MAX_US);
    motors[i].writeMicroseconds(motorPwm[i]);
  }
}

void computeSmc(const SensorData &sensor, const SmcParams &params,
                float *uRoll, float *uPitch, float *uYaw)
{
  // Hover mode: target roll/pitch is level. Yaw uses rate damping only.
  float rollError = -sensor.roll;
  float pitchError = -sensor.pitch;
  float rollSurface = -sensor.gx + params.k1 * rollError;
  float pitchSurface = -sensor.gy + params.k1 * pitchError;

  float rollTorque = SMC_IX_DEFAULT *
    (params.k1 * (params.k1 * rollError - sensor.gx) +
     params.k2 * tanhf(rollSurface / params.eps));
  float pitchTorque = SMC_IY_DEFAULT *
    (params.k1 * (params.k1 * pitchError - sensor.gy) +
     params.k2 * tanhf(pitchSurface / params.eps));
  float yawTorque = SMC_IZ_DEFAULT *
    (-params.k1 * sensor.gz - params.k2 * tanhf(sensor.gz / params.eps));

  *uRoll = constrain((rollTorque / SMC_ARM_LENGTH_DEFAULT) * params.forceToPwm,
                     -params.deltaMaxPwm, params.deltaMaxPwm);
  *uPitch = constrain((pitchTorque / SMC_ARM_LENGTH_DEFAULT) * params.forceToPwm,
                      -params.deltaMaxPwm, params.deltaMaxPwm);
  *uYaw = constrain((yawTorque / SMC_ARM_LENGTH_DEFAULT) * params.forceToPwm,
                    -params.deltaMaxPwm, params.deltaMaxPwm);
}

void TaskControl(void *pvParameters)
{
  (void)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS);
  unsigned long lastDebugMs = 0;

  for (;;) {
    if (escState == ESC_ARMED) {
      SensorData sensor = gSensorData;
      SmcParams params = gSmcParams;
      if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        sensor = gSensorData;
        xSemaphoreGive(sensorMutex);
      }
      if (xSemaphoreTake(smcMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        params = gSmcParams;
        xSemaphoreGive(smcMutex);
      }

      updateThrottleCommand(lastThrottleCmd);
      float uRoll = 0.0f, uPitch = 0.0f, uYaw = 0.0f;
      if (sensor.bmiOK && throttleSmoothed >= MIX_ACTIVE_MIN_US) {
        computeSmc(sensor, params, &uRoll, &uPitch, &uYaw);
      }
      lastURoll = uRoll;
      lastUPitch = uPitch;
      lastUYaw = uYaw;
      writeSmcMotorMix(throttleSmoothed, uRoll, uPitch, uYaw);

#if SMC_BENCH_DEBUG
      if (millis() - lastDebugMs >= 100) {
        lastDebugMs = millis();
        Serial.print("[SMC] R:"); Serial.print(sensor.roll, 2);
        Serial.print(" P:"); Serial.print(sensor.pitch, 2);
        Serial.print(" T_CMD:"); Serial.print(lastThrottleCmd);
        Serial.print(" T_PWM:"); Serial.print(throttleSmoothed, 1);
        Serial.print(" U_R:"); Serial.print(uRoll, 1);
        Serial.print(" U_P:"); Serial.print(uPitch, 1);
        Serial.print(" U_Y:"); Serial.println(uYaw, 1);
      }
#endif
    }
    vTaskDelayUntil(&lastWakeTime, period);
  }
}

void updateEscFSM(bool armCommand)
{
  if (!armCommand) {
    if (escState != ESC_DISARMED) {
      escState = ESC_DISARMED;
      throttleSmoothed = ESC_ARM_SPIN_US;
      throttleRampLastMs = millis();
      setAllMotorsPWM(ESC_MIN_US);
      Serial.println("[ESC] DISARMED -> 4 Motor STOP (1000 us)");
    }
    return;
  }

  // armCommand == true
  if (escState == ESC_DISARMED) {
    escState = ESC_ARMING;
    armingStartMs = millis();
    throttleSmoothed = ESC_ARM_SPIN_US;
    throttleRampLastMs = millis();
    setAllMotorsPWM(ESC_MIN_US);
    Serial.println("[ESC] Mulai Arming 5 detik (1000 us)...");
  }
  else if (escState == ESC_ARMING) {
    if (millis() - armingStartMs >= ARMING_DURATION_MS) {
      escState = ESC_ARMED;
      throttleSmoothed = ESC_ARM_SPIN_US;
      throttleRampLastMs = millis();
      Serial.println("[ESC] ARMED! Motor mengikuti throttle dan Quad-X mixer.");
    } else {
      setAllMotorsPWM(ESC_MIN_US);
    }
  }
}

void updateLedIndicator()
{
  if (escState == ESC_DISARMED) {
    digitalWrite(LED_PIN, HIGH);   // MATI saat DISARM (aktif LOW)
  }
  else if (escState == ESC_ARMING) {
    // Blink tiap detik selama 5 detik arming (500ms ON, 500ms OFF)
    unsigned long elapsed = millis() - armingStartMs;
    bool blinkOn = ((elapsed / 500) % 2) == 0;
    digitalWrite(LED_PIN, blinkOn ? LOW : HIGH);
  }
  else if (escState == ESC_ARMED) {
    digitalWrite(LED_PIN, LOW);    // KONTINYU NYALA saat ARMED & motor berputar
  }
}

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
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }
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

bool bmi160_init(void)
{
  // 1. Soft reset
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
  delay(100);

  // 2. Cek Chip ID
  uint8_t id = i2c_read_reg(BMI160_ADDR, BMI160_REG_CHIP_ID);
  if (id != 0xD1) {
    Serial.print("BMI160: wrong chip ID 0x");
    Serial.println(id, HEX);
    return false;
  }

  // 3. Set PMU mode: Accel Normal (0x11), Gyro Normal (0x15)
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
  delay(50);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
  delay(50);

  // 4. Set ODR dan Range setelah normal mode aktif
  // ACC_CONF: 100Hz, normal filter (0x28)
  // ACC_RANGE: +/- 2G -> 16384 LSB/g (0x03)
  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_CONF, 0x28);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_RANGE, 0x03);
  delay(10);

  // GYR_CONF: 100Hz, normal filter (0x28)
  // GYR_RANGE: +/- 2000 dps -> 16.4 LSB/dps (0x00)
  i2c_write_reg(BMI160_ADDR, BMI160_REG_GYR_CONF, 0x28);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_GYR_RANGE, 0x00);
  delay(50);

  return true;
}

bool bmi160_read(int16_t *ax, int16_t *ay, int16_t *az,
                 int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t buf[12];

  // Baca 12 byte sekaligus (0x0C Gyro sampai 0x17 Accel) dalam satu burst I2C
  if (!i2c_read_regs(BMI160_ADDR, BMI160_REG_GYRO_X_L, buf, 12)) {
    return false;
  }

  *gx = (int16_t)(buf[1]  << 8 | buf[0]);
  *gy = (int16_t)(buf[3]  << 8 | buf[2]);
  *gz = (int16_t)(buf[5]  << 8 | buf[4]);

  *ax = (int16_t)(buf[7]  << 8 | buf[6]);
  *ay = (int16_t)(buf[9]  << 8 | buf[8]);
  *az = (int16_t)(buf[11] << 8 | buf[10]);

  return true;
}

/* ========================= BMP280 REGISTERS & DRIVER ==================== */
#define BMP280_REG_CHIP_ID     0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CALIB_00    0x88

#define BMP280_CHIP_ID_VALUE   0x58
#define BMP280_RESET_VALUE     0xB6

struct bmp280_calib {
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;
};

bmp280_calib calib;
int32_t t_fine;

bool bmp280_init(void)
{
  i2c_write_reg(BMP280_ADDR, BMP280_REG_RESET, BMP280_RESET_VALUE);
  delay(100);

  uint8_t id = i2c_read_reg(BMP280_ADDR, BMP280_REG_CHIP_ID);
  if (id != BMP280_CHIP_ID_VALUE) {
    Serial.print("BMP280: wrong chip ID 0x");
    Serial.println(id, HEX);
    return false;
  }

  uint8_t buf[26];
  if (!i2c_read_regs(BMP280_ADDR, BMP280_REG_CALIB_00, buf, 26)) {
    return false;
  }

  calib.dig_T1 = (uint16_t)(buf[1]  << 8 | buf[0]);
  calib.dig_T2 = (int16_t)(buf[3]  << 8 | buf[2]);
  calib.dig_T3 = (int16_t)(buf[5]  << 8 | buf[4]);
  calib.dig_P1 = (uint16_t)(buf[7]  << 8 | buf[6]);
  calib.dig_P2 = (int16_t)(buf[9]  << 8 | buf[8]);
  calib.dig_P3 = (int16_t)(buf[11] << 8 | buf[10]);
  calib.dig_P4 = (int16_t)(buf[13] << 8 | buf[12]);
  calib.dig_P5 = (int16_t)(buf[15] << 8 | buf[14]);
  calib.dig_P6 = (int16_t)(buf[17] << 8 | buf[16]);
  calib.dig_P7 = (int16_t)(buf[19] << 8 | buf[18]);
  calib.dig_P8 = (int16_t)(buf[21] << 8 | buf[20]);
  calib.dig_P9 = (int16_t)(buf[23] << 8 | buf[22]);

  i2c_write_reg(BMP280_ADDR, BMP280_REG_CONFIG, 0x90);
  i2c_write_reg(BMP280_ADDR, BMP280_REG_CTRL_MEAS, 0x57);
  delay(50);

  return true;
}

void bmp280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
          ((int32_t)calib.dig_T3)) >> 14;
  t_fine = var1 + var2;
}

uint32_t bmp280_compensate_P(int32_t adc_P)
{
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) +
         ((var1 * (int64_t)calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
  if (var1 == 0) return 0;
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
  return (uint32_t)(p >> 8);
}

bool bmp280_read(float *press_hpa, float *alt_m)
{
  uint8_t buf[6];
  if (!i2c_read_regs(BMP280_ADDR, BMP280_REG_PRESS_MSB, buf, 6)) {
    return false;
  }

  int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
  int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);

  if (adc_P == 0 || adc_T == 0) {
    return false;
  }

  bmp280_compensate_T(adc_T);
  uint32_t P = bmp280_compensate_P(adc_P);

  // Kompensasi Bosch mengembalikan tekanan dalam Pascal (Pa) -> / 100.0f = hPa
  *press_hpa = P / 100.0f;
  if (*press_hpa > 300.0f && *press_hpa < 1200.0f) {
    *alt_m = 44330.0f * (1.0f - powf(*press_hpa / 1013.25f, 0.190284f));
    return true;
  }
  return false;
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
  bool attitudeInitialized = false;

  for (;;)
  {
    if (gSensorData.bmiOK) {
      int16_t ax, ay, az, gx, gy, gz;
      if (bmi160_read(&ax, &ay, &az, &gx, &gy, &gz)) {
        // BMI160: +/- 2G range -> 16384 LSB/g
        // IMU dipasang X_sensor = DEPAN, Y_sensor = KIRI, Z_sensor = ATAS.
        // Remap ke body frame standar (Y_body = kanan -> negasi aksis Y):
        float raw_ax = ( ax / 16384.0f) * 9.80665f;  // body depan   <- sensor X (depan)
        float raw_ay = (-ay / 16384.0f) * 9.80665f;  // body kanan   <- sensor Y (kiri, dinegasi)
        float raw_az = ( az / 16384.0f) * 9.80665f;  // body atas    <- sensor Z

        // BMI160: +/- 2000 dps range -> 16.4 LSB/dps
        float raw_gx = ( gx / 16.4f) - gyro_bias_gx;  // body roll rate  <- sensor X
        float raw_gy = (-gy / 16.4f) - gyro_bias_gy;  // body pitch rate <- sensor Y (kiri, dinegasi)
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
        float pitchAcc = atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * 180.0f / PI;
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

    if (gSensorData.bmpOK) {
      float raw_p, raw_a;
      if (bmp280_read(&raw_p, &raw_a)) {
        float raw_rel_alt = raw_a - altitude_offset;
        // IIR Low-Pass Filter untuk altitude (Alpha = 0.85)
        alt_filtered = 0.85f * alt_filtered + 0.15f * raw_rel_alt;
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
      xSemaphoreGive(sensorMutex);
    }

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

/* ========================= FREERTOS TASK 2: LORA & CONTROL ============== */
void TaskLoRa_Control(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    int packetSize = LoRa.parsePacket();

    if (packetSize == sizeof(UplinkPacket))
    {
      uint8_t buf[sizeof(UplinkPacket)];
      for (uint8_t i = 0; i < sizeof(UplinkPacket) && LoRa.available(); i++) {
        buf[i] = (uint8_t)LoRa.read();
      }

      UplinkPacket up;
      memcpy(&up, buf, sizeof(UplinkPacket));

      if (up.magic == UPLINK_MAGIC)
      {
        lastLinkMs = millis();
        if (!linkOK) {
          linkOK = true;
          Serial.println(">>> Link dengan remote aktif <<<");
        }

        /* Update State Motor ESC (ARM / DISARM) */
        lastRollCmd = up.r;
        lastThrottleCmd = up.t;
        lastYawCmd = up.y;
        lastPitchCmd = up.p;
        bool armCmd = (up.armed == 1);
        updateEscFSM(armCmd);

        /* Ambil snapshot data sensor terbaru (thread-safe) */
        SensorData snap;
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          snap = gSensorData;
          xSemaphoreGive(sensorMutex);
        }

        /* Balas ke remote: DownlinkPacket biner (27 byte, termasuk data SMC) */
        DownlinkPacket down;
        down.magic = DOWNLINK_MAGIC;
        down.ax = (int16_t)(snap.ax * 100.0f);
        down.ay = (int16_t)(snap.ay * 100.0f);
        down.az = (int16_t)(snap.az * 100.0f);
        down.gx = (int16_t)(snap.gx * 100.0f);
        down.gy = (int16_t)(snap.gy * 100.0f);
        down.gz = (int16_t)(snap.gz * 100.0f);
        down.press = (uint16_t)(snap.press * 10.0f);
        down.alt = (int16_t)(snap.alt * 100.0f);
        down.roll = (int16_t)(snap.roll * 100.0f);
        down.pitch = (int16_t)(snap.pitch * 100.0f);
        down.uRoll = (int16_t)(lastURoll * 100.0f);
        down.uPitch = (int16_t)(lastUPitch * 100.0f);
        down.uYaw = (int16_t)(lastUYaw * 100.0f);

        LoRa.beginPacket();
        LoRa.write((uint8_t *)&down, sizeof(DownlinkPacket));
        LoRa.endPacket();

        LoRa.receive();   // kembali siaga menunggu paket joystick berikutnya

        /* Debug lokal via USB-TTL (USART1) */
        Serial.print("[RX] R:"); Serial.print(up.r);
        Serial.print(" T:"); Serial.print(up.t);
        Serial.print(" Y:"); Serial.print(up.y);
        Serial.print(" P:"); Serial.print(up.p);
        Serial.print(" ARM:"); Serial.print(up.armed);
        if (escState == ESC_ARMED) {
          Serial.print(" | M1:"); Serial.print(motors[0].readMicroseconds());
          Serial.print(" M2:"); Serial.print(motors[1].readMicroseconds());
          Serial.print(" M3:"); Serial.print(motors[2].readMicroseconds());
          Serial.print(" M4:"); Serial.print(motors[3].readMicroseconds());
        }
        Serial.print(" | AX:"); Serial.print(snap.ax, 2);
        Serial.print(" AY:"); Serial.print(snap.ay, 2);
        Serial.print(" AZ:"); Serial.print(snap.az, 2);
        Serial.print(" | P:"); Serial.print(snap.press, 2);
        Serial.print(" A:"); Serial.println(snap.alt, 2);
      }
      else
      {
        LoRa.receive();
        Serial.println("[RX] Magic byte uplink tidak cocok, paket diabaikan.");
      }
    }
    else if (packetSize == sizeof(ConfigPacket))
    {
      uint8_t buf[sizeof(ConfigPacket)];
      for (uint8_t i = 0; i < sizeof(ConfigPacket) && LoRa.available(); i++) {
        buf[i] = (uint8_t)LoRa.read();
      }

      ConfigPacket config;
      memcpy(&config, buf, sizeof(ConfigPacket));
      if (config.magic == CONFIG_MAGIC && config.k1 > 0.0f && config.k2 > 0.0f &&
          config.eps > 0.1f && config.forceToPwm > 0.0f && config.deltaMaxPwm > 0.0f) {
        if (xSemaphoreTake(smcMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          gSmcParams.k1 = config.k1;
          gSmcParams.k2 = config.k2;
          gSmcParams.eps = config.eps;
          gSmcParams.forceToPwm = config.forceToPwm;
          gSmcParams.deltaMaxPwm = config.deltaMaxPwm;
          xSemaphoreGive(smcMutex);
        }
        Serial.print("[SMC] Params k1="); Serial.print(config.k1, 2);
        Serial.print(" k2="); Serial.print(config.k2, 2);
        Serial.print(" eps="); Serial.println(config.eps, 2);
      }
      LoRa.receive();
    }
    else if (packetSize > 0)
    {
      while (LoRa.available()) { LoRa.read(); }
      LoRa.receive();
    }

    /* Update proses arming 5 detik saat timer berjalan */
    if (escState == ESC_ARMING) {
      updateEscFSM(true);
    }

    /* Update LED PC4 (Indikator Arming / Armed) */
    updateLedIndicator();

    /* Link timeout & Failsafe motor */
    if (linkOK && (millis() - lastLinkMs > LINK_TIMEOUT_MS))
    {
      linkOK = false;
      updateEscFSM(false);   // FAILSAFE: matikan motor seketika
      Serial.println("! Link terputus: sinyal remote hilang > 1 detik. Motor STOP.");
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_LORA_PERIOD_MS));
  }
}

/* ========================= SETUP ========================================= */
void setup()
{
  Serial.setTx(SERIAL_TX_PIN);
  Serial.setRx(SERIAL_RX_PIN);
  Serial.begin(SERIAL_BAUD);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);   // PC4 mati saat startup / disarmed (aktif LOW)

  Serial.println();
  Serial.println("=== QMBED DRONE FLIGHT CONTROLLER (FreeRTOS) ===");
  Serial.println("Board: STM32F401RCT6 | LoRa RA-02 + 4 ESC (PB6-PB9)");
  Serial.println("-------------------------------------------------");

  /* Inisialisasi 4 Motor ESC (1000 us) */
  for (int i = 0; i < 4; i++) {
    motors[i].attach(MOTOR_PINS[i]);
    motors[i].writeMicroseconds(ESC_MIN_US);
  }

  /* Inisialisasi I2C Sensor (PB10/PB3) */
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  Serial.print("BMI160 init... ");
  gSensorData.bmiOK = bmi160_init();
  Serial.println(gSensorData.bmiOK ? "OK" : "FAILED");

  /* Kalibrasi Zero-Bias Gyro (kondisi drone diletakkan diam) */
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
      // BMI160 +/- 2000 dps -> 16.4 LSB/dps
      // Bias zero offset: body_gy (pitch rate) memakai -gy (Y=kiri dinegasi),
      // sehingga bias untuk aksis Y harus dinegasikan agar raw_gy = 0 saat diam.
      gyro_bias_gx = (float)sum_gx / valid_samples / 16.4f;
      gyro_bias_gy = -(float)sum_gy / valid_samples / 16.4f;
      gyro_bias_gz = (float)sum_gz / valid_samples / 16.4f;
    }
    Serial.println("OK");
    Serial.print("  Bias Gyro: X="); Serial.print(gyro_bias_gx, 3);
    Serial.print(" Y="); Serial.print(gyro_bias_gy, 3);
    Serial.print(" Z="); Serial.print(gyro_bias_gz, 3);
    Serial.println(" dps");
  }

  Serial.print("BMP280 init... ");
  gSensorData.bmpOK = bmp280_init();
  if (gSensorData.bmpOK) {
    Serial.println("OK");
    Serial.print("Kalibrasi baseline Barometer... ");
    float sum_a = 0.0f, p, a;
    int valid_baro = 0;
    for (int i = 0; i < 30; i++) {
      if (bmp280_read(&p, &a)) {
        sum_a += a;
        valid_baro++;
      }
      delay(15);
    }
    if (valid_baro > 5) {
      altitude_offset = sum_a / valid_baro;
    }
    alt_filtered = 0.0f;
    Serial.println("OK");
    Serial.print("  Altitude baseline: ");
    Serial.print(altitude_offset, 2);
    Serial.println(" m");
  } else {
    Serial.println("FAILED");
  }

  /* Inisialisasi LoRa RA-02 (SPI2) */
  LoRa.setPins(LORA_NSS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  LoRa.setSPI(SPI_2);

  if (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println("GAGAL! Modul LoRa RA-02 tidak terdeteksi.");
    Serial.println("Cek wiring SPI2 & pastikan VCC = 3.3V.");
    while (1) { delay(1000); }
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.receive();

  Serial.println("LoRa siap. Memulai FreeRTOS scheduler...");
  Serial.println("-------------------------------------------------");

  /* ── Inisialisasi Mutex & FreeRTOS Tasks ────────────────────────────── */
  sensorMutex = xSemaphoreCreateMutex();
  smcMutex = xSemaphoreCreateMutex();

  xTaskCreate(
    TaskSensors,
    "Sensors",
    TASK_SENSOR_STACK_SIZE,
    NULL,
    TASK_SENSOR_PRIORITY,
    NULL
  );

  xTaskCreate(
    TaskLoRa_Control,
    "LoRa_Ctrl",
    TASK_LORA_STACK_SIZE,
    NULL,
    TASK_LORA_PRIORITY,
    NULL
  );

  xTaskCreate(
    TaskControl,
    "Control",
    TASK_CONTROL_STACK_SIZE,
    NULL,
    TASK_CONTROL_PRIORITY,
    NULL
  );

  /* Start Scheduler */
  vTaskStartScheduler();
}

/* ========================= LOOP =========================================== */
void loop()
{
  // Kosong - scheduler FreeRTOS mengambil alih eksekusi
}
