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

void updateEscFSM(bool armCommand)
{
  if (!armCommand) {
    if (escState != ESC_DISARMED) {
      escState = ESC_DISARMED;
      setAllMotorsPWM(ESC_MIN_US);
      Serial.println("[ESC] DISARMED -> 4 Motor STOP (1000 us)");
    }
    return;
  }

  // armCommand == true
  if (escState == ESC_DISARMED) {
    escState = ESC_ARMING;
    armingStartMs = millis();
    setAllMotorsPWM(ESC_MIN_US);
    Serial.println("[ESC] Mulai Arming 5 detik (1000 us)...");
  }
  else if (escState == ESC_ARMING) {
    if (millis() - armingStartMs >= ARMING_DURATION_MS) {
      escState = ESC_ARMED;
      setAllMotorsPWM(ESC_ARM_SPIN_US);
      Serial.println("[ESC] ARMED! 4 Motor berputar 20% (1200 us)");
    } else {
      setAllMotorsPWM(ESC_MIN_US);
    }
  }
  else if (escState == ESC_ARMED) {
    setAllMotorsPWM(ESC_ARM_SPIN_US);
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

  for (;;)
  {
    if (gSensorData.bmiOK) {
      int16_t ax, ay, az, gx, gy, gz;
      if (bmi160_read(&ax, &ay, &az, &gx, &gy, &gz)) {
        // BMI160: +/- 2G range -> 16384 LSB/g
        float raw_ax = (ax / 16384.0f) * 9.80665f;
        float raw_ay = (ay / 16384.0f) * 9.80665f;
        float raw_az = (az / 16384.0f) * 9.80665f;

        // BMI160: +/- 2000 dps range -> 16.4 LSB/dps
        float raw_gx = (gx / 16.4f) - gyro_bias_gx;
        float raw_gy = (gy / 16.4f) - gyro_bias_gy;
        float raw_gz = (gz / 16.4f) - gyro_bias_gz;

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
        bool armCmd = (up.armed == 1);
        updateEscFSM(armCmd);

        /* Ambil snapshot data sensor terbaru (thread-safe) */
        SensorData snap;
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          snap = gSensorData;
          xSemaphoreGive(sensorMutex);
        }

        /* Balas ke remote: DownlinkPacket biner (17 byte) */
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

  /* Start Scheduler */
  vTaskStartScheduler();
}

/* ========================= LOOP =========================================== */
void loop()
{
  // Kosong - scheduler FreeRTOS mengambil alih eksekusi
}
