/* ==========================================================================
 * PROGRAM BACA SENSOR RAW (BMI160 IMU + BMP280 BAROMETER)
 * Target Board : STM32F401RCT6
 * --------------------------------------------------------------------------
 * Murni membaca data mentah (RAW DATA) langsung dari register sensor
 * TANPA FILTER APAPUN (tanpa bias offset, tanpa deadband, tanpa IIR smoothing,
 * dan tanpa complementary filter).
 *
 * KONFIGURASI PIN & BUS:
 *   I2C   : PB10 (SCL) + PB3 (SDA)  -> Shared Bus 400 kHz
 *   Serial: PA9 (TX) + PA10 (RX)     -> Serial Monitor @ 115200 baud
 *
 * ALAMAT I2C SENSOR:
 *   BMI160 : 0x68
 *   BMP280 : 0x76
 * ==========================================================================
 */

#include <Wire.h>
#include <math.h>
#include <string.h>

/* ── Konfigurasi Pin ─────────────────────────────────────────────────── */
#define SERIAL_TX_PIN   PA9
#define SERIAL_RX_PIN   PA10
#define SERIAL_BAUD     115200

#define I2C_SCL_PIN     PB10
#define I2C_SDA_PIN     PB3
#define I2C_CLOCK_SPEED 400000

#define BMI160_ADDR     0x68
#define BMP280_ADDR     0x76

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

/* ── Register BMP280 ─────────────────────────────────────────────────── */
#define BMP280_REG_CHIP_ID     0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CALIB_00    0x88

#define BMP280_CHIP_ID_VALUE   0x58
#define BMP280_RESET_VALUE     0xB6

/* ── Struct Kalibrasi BMP280 ─────────────────────────────────────────── */
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

bool bmiOK = false;
bool bmpOK = false;

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

/* ========================= DRIVER BMI160 RAW ============================ */

bool bmi160_init(void)
{
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
  delay(100);

  uint8_t id = i2c_read_reg(BMI160_ADDR, BMI160_REG_CHIP_ID);
  if (id != 0xD1) {
    Serial.print("BMI160: Chip ID salah (0x");
    Serial.print(id, HEX);
    Serial.println(")");
    return false;
  }

  // Set PMU mode: normal
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
  delay(50);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
  delay(50);

  // Set konfigurasi: +/- 2G (16384 LSB/g) & +/- 2000 dps (16.4 LSB/dps)
  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_CONF, 0x28);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_RANGE, 0x03);
  delay(10);

  i2c_write_reg(BMI160_ADDR, BMI160_REG_GYR_CONF, 0x28);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_GYR_RANGE, 0x00);
  delay(50);

  return true;
}

// Baca nilai register mentah (ADC counts int16_t)
bool bmi160_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t buf[12];
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

/* ========================= DRIVER BMP280 RAW ============================ */

bool bmp280_init(void)
{
  i2c_write_reg(BMP280_ADDR, BMP280_REG_RESET, BMP280_RESET_VALUE);
  delay(100);

  uint8_t id = i2c_read_reg(BMP280_ADDR, BMP280_REG_CHIP_ID);
  if (id != BMP280_CHIP_ID_VALUE) {
    Serial.print("BMP280: Chip ID salah (0x");
    Serial.print(id, HEX);
    Serial.println(")");
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

// Baca tekanan mentah (hPa) & altitude mentah (m ASL) tanpa filter
bool bmp280_read_raw(int32_t *raw_p_adc, int32_t *raw_t_adc, float *press_hpa, float *alt_m)
{
  uint8_t buf[6];
  if (!i2c_read_regs(BMP280_ADDR, BMP280_REG_PRESS_MSB, buf, 6)) {
    return false;
  }

  int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
  int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);

  *raw_p_adc = adc_P;
  *raw_t_adc = adc_T;

  if (adc_P == 0 || adc_T == 0) return false;

  bmp280_compensate_T(adc_T);
  uint32_t P = bmp280_compensate_P(adc_P);

  *press_hpa = P / 100.0f;  // Pascal -> hPa
  if (*press_hpa > 300.0f && *press_hpa < 1200.0f) {
    // Rumus standar hypsometrik internasional (sea level 1013.25 hPa)
    *alt_m = 44330.0f * (1.0f - powf(*press_hpa / 1013.25f, 0.190284f));
    return true;
  }
  return false;
}

/* ========================= SETUP ========================================= */
void setup()
{
  Serial.setTx(SERIAL_TX_PIN);
  Serial.setRx(SERIAL_RX_PIN);
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("==========================================================");
  Serial.println("   SENSOR RAW READ — BMI160 & BMP280 (NO FILTER)");
  Serial.println("   Target: STM32F401RCT6 | I2C: SCL=PB10, SDA=PB3");
  Serial.println("==========================================================");

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  Serial.print("Inisialisasi BMI160... ");
  bmiOK = bmi160_init();
  Serial.println(bmiOK ? "OK (+/-2G, +/-2000dps)" : "FAILED!");

  Serial.print("Inisialisasi BMP280... ");
  bmpOK = bmp280_init();
  Serial.println(bmpOK ? "OK (Normal Mode)" : "FAILED!");

  Serial.println("----------------------------------------------------------");
  Serial.println("Memulai pembacaan RAW data (20 Hz)...");
  Serial.println("Kolom: AX, AY, AZ (m/s2) | GX, GY, GZ (dps) | PRESS (hPa) | ALT (m)");
  Serial.println("----------------------------------------------------------");
  delay(1000);
}

/* ========================= LOOP =========================================== */
void loop()
{
  int16_t raw_ax = 0, raw_ay = 0, raw_az = 0;
  int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0;
  int32_t adc_p = 0, adc_t = 0;
  float press_hpa = 0.0f, alt_m = 0.0f;

  float ax_g = 0, ay_g = 0, az_g = 0;
  float gx_dps = 0, gy_dps = 0, gz_dps = 0;

  if (bmiOK) {
    if (bmi160_read_raw(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz)) {
      // Konversi langsung dari register ADC counts murni tanpa filter:
      // +/- 2G -> 16384 LSB/g
      ax_g = (raw_ax / 16384.0f) * 9.80665f;
      ay_g = (raw_ay / 16384.0f) * 9.80665f;
      az_g = (raw_az / 16384.0f) * 9.80665f;

      // +/- 2000 dps -> 16.4 LSB/dps
      gx_dps = raw_gx / 16.4f;
      gy_dps = raw_gy / 16.4f;
      gz_dps = raw_gz / 16.4f;
    }
  }

  if (bmpOK) {
    bmp280_read_raw(&adc_p, &adc_t, &press_hpa, &alt_m);
  }

  /* ── 1. Tampilan Terstruktur untuk Serial Monitor ─────────────────────── */
  Serial.print("[IMU RAW] Acc(m/s2): ");
  Serial.print(ax_g, 2); Serial.print(", ");
  Serial.print(ay_g, 2); Serial.print(", ");
  Serial.print(az_g, 2);
  Serial.print(" | Gyr(dps): ");
  Serial.print(gx_dps, 2); Serial.print(", ");
  Serial.print(gy_dps, 2); Serial.print(", ");
  Serial.print(gz_dps, 2);
  Serial.print(" || [BMP RAW] Press: ");
  Serial.print(press_hpa, 2); Serial.print(" hPa | Alt: ");
  Serial.print(alt_m, 2); Serial.println(" m");

  /* ── 2. Format CSV Mentah (bisa di-plot via Serial Plotter) ─────────── */
  /*
  Serial.print(ax_g, 2); Serial.print(",");
  Serial.print(ay_g, 2); Serial.print(",");
  Serial.print(az_g, 2); Serial.print(",");
  Serial.print(gx_dps, 2); Serial.print(",");
  Serial.print(gy_dps, 2); Serial.print(",");
  Serial.print(gz_dps, 2); Serial.print(",");
  Serial.print(press_hpa, 2); Serial.print(",");
  Serial.println(alt_m, 2);
  */

  delay(50);   // Update @ 20 Hz
}
