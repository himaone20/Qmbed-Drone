/* ==========================================================================
 * PROGRAM SENSOR BMI160 (IMU) + BMP280 (Pressure) - STM32F401RCT6
 * Driver I2C langsung via Wire.h (tanpa library eksternal)
 *
 * Interface:
 *   I2C   : PB6 (SCL) + PB7 (SDA)  -> BMI160 & BMP280 (shared bus)
 *   Serial: PA9 (TX) + PA10 (RX)    -> debug output @ 115200 baud
 *
 * Sensor addresses (I2C default):
 *   BMI160 : 0x68 (SDO/GND) atau 0x69 (SDO/VCC)
 *   BMP280 : 0x76 (SDO/GND) atau 0x77 (SDO/VCC)
 *
 * Output serial @ 100 Hz:
 *   [IMU] AX:xxx.xx AY:xxx.xx AZ:xxx.xx GX:xxx.xx GY:xxx.xx GZ:xxx.xx
 *   [BMP] T:xx.xx P:xxxxx.xx A:xxx.xx
 *   CSV: ax,ay,az,gx,gy,gz,temp,press,alt
 * ==========================================================================
 */

#include <Wire.h>

/* ========================= KONFIGURASI ============================ */
#define SERIAL_BAUD   115200
#define SENSOR_HZ     100
#define SENSOR_PERIOD (1000 / SENSOR_HZ)

/* I2C pins */
#define I2C_SCL  PB10
#define I2C_SDA  PB3

/* Alamat I2C */
#define BMI160_ADDR  0x68
#define BMP280_ADDR  0x76

/* ========================= BMI160 REGISTERS ======================= */
#define BMI160_REG_CHIP_ID       0x00
#define BMI160_REG_ERR_REG       0x02
#define BMI160_REG_PMU_STATUS    0x03
#define BMI160_REG_GYRO_X_L      0x0C
#define BMI160_REG_GYRO_X_H      0x0D
#define BMI160_REG_GYRO_Y_L      0x0E
#define BMI160_REG_GYRO_Y_H      0x0F
#define BMI160_REG_GYRO_Z_L      0x10
#define BMI160_REG_GYRO_Z_H      0x11
#define BMI160_REG_ACCEL_X_L     0x12
#define BMI160_REG_ACCEL_X_H     0x13
#define BMI160_REG_ACCEL_Y_L     0x14
#define BMI160_REG_ACCEL_Y_H     0x15
#define BMI160_REG_ACCEL_Z_L     0x16
#define BMI160_REG_ACCEL_Z_H     0x17
#define BMI160_REG_STATUS        0x1B
#define BMI160_REG_ACC_CONF      0x40
#define BMI160_REG_ACC_RANGE     0x41
#define BMI160_REG_GYR_CONF      0x42
#define BMI160_REG_GYR_RANGE     0x43
#define BMI160_REG_CMD           0x7E

#define BMI160_CMD_SOFT_RESET    0xB6
#define BMI160_CMD_ACC_NORMAL    0x11
#define BMI160_CMD_GYR_NORMAL    0x15

/* ========================= BMP280 REGISTERS ======================= */
#define BMP280_REG_CHIP_ID       0xD0
#define BMP280_REG_RESET         0xE0
#define BMP280_REG_STATUS        0xF3
#define BMP280_REG_CTRL_MEAS     0xF4
#define BMP280_REG_CONFIG        0xF5
#define BMP280_REG_PRESS_MSB     0xF7
#define BMP280_REG_TEMP_MSB      0xFA
#define BMP280_REG_CALIB_00      0x88

#define BMP280_CHIP_ID_VALUE     0x58
#define BMP280_RESET_VALUE       0xB6

/* ========================= FUNGSI I2C LOW-LEVEL =================== */

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
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

void i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

/* ========================= BMI160 DRIVER ========================== */

bool bmi160_init(void)
{
  /* Soft reset */
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
  delay(100);

  /* Cek chip ID */
  uint8_t id = i2c_read_reg(BMI160_ADDR, BMI160_REG_CHIP_ID);
  if (id != 0xD1) {
    Serial.print("BMI160: wrong chip ID 0x");
    Serial.println(id, HEX);
    return false;
  }

  /* Set accel range: +/- 4G (0x05) */
  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_RANGE, 0x05);

  /* Set accel ODR 100Hz, normal mode */
  i2c_write_reg(BMI160_ADDR, BMI160_REG_ACC_CONF, 0x2C);

  /* Set gyro range: +/- 250 dps (0x03) */
  i2c_write_reg(BMI160_ADDR, BMI160_REG_GYR_RANGE, 0x03);

  /* Set gyro ODR 100Hz, normal mode */
  i2c_write_reg(BMI160_ADDR, BMI160_REG_GYR_CONF, 0x28);

  /* Enable accel + gyro normal mode */
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
  delay(10);
  i2c_write_reg(BMI160_ADDR, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
  delay(100);

  return true;
}

void bmi160_read(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t buf[12];

  /* Gyro: reg 0x0C - 0x11 (6 bytes) */
  i2c_read_regs(BMI160_ADDR, BMI160_REG_GYRO_X_L, buf, 6);
  *gx = (int16_t)(buf[1] << 8 | buf[0]);
  *gy = (int16_t)(buf[3] << 8 | buf[2]);
  *gz = (int16_t)(buf[5] << 8 | buf[4]);

  /* Accel: reg 0x12 - 0x17 (6 bytes) */
  i2c_read_regs(BMI160_ADDR, BMI160_REG_ACCEL_X_L, buf + 6, 6);
  *ax = (int16_t)(buf[7] << 8 | buf[6]);
  *ay = (int16_t)(buf[9] << 8 | buf[8]);
  *az = (int16_t)(buf[11] << 8 | buf[10]);
}

/* ========================= BMP280 DRIVER ========================== */

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
  /* Soft reset */
  i2c_write_reg(BMP280_ADDR, BMP280_REG_RESET, BMP280_RESET_VALUE);
  delay(100);

  /* Cek chip ID */
  uint8_t id = i2c_read_reg(BMP280_ADDR, BMP280_REG_CHIP_ID);
  if (id != BMP280_CHIP_ID_VALUE) {
    Serial.print("BMP280: wrong chip ID 0x");
    Serial.println(id, HEX);
    return false;
  }

  /* Baca kalibrasi data */
  uint8_t buf[26];
  i2c_read_regs(BMP280_ADDR, BMP280_REG_CALIB_00, buf, 26);

  calib.dig_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
  calib.dig_T2 = (int16_t)(buf[3] << 8 | buf[2]);
  calib.dig_T3 = (int16_t)(buf[5] << 8 | buf[4]);
  calib.dig_P1 = (uint16_t)(buf[7] << 8 | buf[6]);
  calib.dig_P2 = (int16_t)(buf[9] << 8 | buf[8]);
  calib.dig_P3 = (int16_t)(buf[11] << 8 | buf[10]);
  calib.dig_P4 = (int16_t)(buf[13] << 8 | buf[12]);
  calib.dig_P5 = (int16_t)(buf[15] << 8 | buf[14]);
  calib.dig_P6 = (int16_t)(buf[17] << 8 | buf[16]);
  calib.dig_P7 = (int16_t)(buf[19] << 8 | buf[18]);
  calib.dig_P8 = (int16_t)(buf[21] << 8 | buf[20]);
  calib.dig_P9 = (int16_t)(buf[23] << 8 | buf[22]);

  /* ctrl_meas: osrs_t=x2, osrs_p=x16, mode=normal (0x57)
     config:    t_sb=50ms, filter=x16, spi3w_en=0 (0x90) */
  i2c_write_reg(BMP280_ADDR, BMP280_REG_CONFIG,  0x90);
  i2c_write_reg(BMP280_ADDR, BMP280_REG_CTRL_MEAS, 0x57);

  return true;
}

int32_t bmp280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
          ((int32_t)calib.dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
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

void bmp280_read(float *temp_c, float *press_hpa, float *alt_m)
{
  uint8_t buf[6];
  i2c_read_regs(BMP280_ADDR, BMP280_REG_PRESS_MSB, buf, 6);

  int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
  int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);

  int32_t T = bmp280_compensate_T(adc_T);
  uint32_t P = bmp280_compensate_P(adc_P);

  *temp_c    = T / 100.0f;
  *press_hpa = P / 256.0f / 100.0f;

  /* Altitude dari rumus barometrik */
  *alt_m = 44330.0f * (1.0f - powf(*press_hpa / 1013.25f, 0.190284f));
}

/* ========================= STATE ================================== */
bool bmiOK = false;
bool bmpOK = false;
float altitude_offset = 0.0f;

/* ========================= SETUP ================================== */
void setup()
{
  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("=== BMI160 + BMP280 - STM32F401RCT6 ===");
  Serial.println("I2C: PB6(SCL) + PB7(SDA)");
  Serial.print("Serial: PA9(TX) + PA10(RX) @ ");
  Serial.print(SERIAL_BAUD);
  Serial.println(" baud");
  Serial.println("--------------------------------------------------");

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);

  /* Scan I2C */
  Serial.println("I2C scan...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("--------------------------------------------------");

  /* BMI160 */
  Serial.print("BMI160 init... ");
  bmiOK = bmi160_init();
  if (bmiOK) {
    Serial.println("OK!");
    Serial.println("  Accel: +/- 4G, 100Hz");
    Serial.println("  Gyro:  +/- 250 dps, 100Hz");
  } else {
    Serial.println("FAILED! Cek wiring SDA->PB7, SCL->PB6");
  }

  /* BMP280 */
  Serial.print("BMP280 init... ");
  bmpOK = bmp280_init();
  if (bmpOK) {
    Serial.println("OK!");
    Serial.println("  Temp x2, Press x16, Filter x16, Normal mode");
    /* Kalibrasi altitude */
    float t, p, a;
    bmp280_read(&t, &p, &a);
    altitude_offset = a;
    Serial.print("  Altitude offset: ");
    Serial.print(altitude_offset);
    Serial.println(" m");
  } else {
    Serial.println("FAILED! Cek wiring SDA->PB7, SCL->PB6");
  }

  Serial.println("--------------------------------------------------");
  Serial.println("Siap mengirim data...");
  Serial.println("--------------------------------------------------");
}

/* ========================= LOOP =================================== */
void loop()
{
  static unsigned long last_ms = 0;
  unsigned long now = millis();

  if (now - last_ms < SENSOR_PERIOD) return;
  last_ms = now;

  if (bmiOK) {
    int16_t ax, ay, az, gx, gy, gz;
    bmi160_read(&ax, &ay, &az, &gx, &gy, &gz);

    /* Convert raw to physical:
       Accel: +/-4G range, 16-bit -> sensitivity ~8192 LSB/G
       Gyro:  +/-250dps range, 16-bit -> sensitivity ~131 LSB/dps */
    float ax_g = ax / 8192.0f * 9.80665f;  /* m/s^2 */
    float ay_g = ay / 8192.0f * 9.80665f;
    float az_g = az / 8192.0f * 9.80665f;
    float gx_dps = gx / 131.0f;            /* dps */
    float gy_dps = gy / 131.0f;
    float gz_dps = gz / 131.0f;

    Serial.print("[IMU] AX:");
    Serial.print(ax_g, 2);
    Serial.print(" AY:");
    Serial.print(ay_g, 2);
    Serial.print(" AZ:");
    Serial.print(az_g, 2);
    Serial.print(" GX:");
    Serial.print(gx_dps, 2);
    Serial.print(" GY:");
    Serial.print(gy_dps, 2);
    Serial.print(" GZ:");
    Serial.println(gz_dps, 2);

    Serial.print("CSV,");
    Serial.print(ax_g, 2);  Serial.print(',');
    Serial.print(ay_g, 2);  Serial.print(',');
    Serial.print(az_g, 2);  Serial.print(',');
    Serial.print(gx_dps, 2);  Serial.print(',');
    Serial.print(gy_dps, 2);  Serial.print(',');
    Serial.println(gz_dps, 2);
  }

  if (bmpOK) {
    float temp, press, alt;
    bmp280_read(&temp, &press, &alt);
    alt -= altitude_offset;

    Serial.print("[BMP] T:");
    Serial.print(temp, 2);
    Serial.print(" P:");
    Serial.print(press, 2);
    Serial.print(" A:");
    Serial.println(alt, 2);

    Serial.print("CSV,");
    Serial.print(temp, 2);  Serial.print(',');
    Serial.print(press, 2); Serial.print(',');
    Serial.println(alt, 2);
  }
}
