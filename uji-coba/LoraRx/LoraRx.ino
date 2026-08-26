/* ==========================================================================
 * PROGRAM LORA RA-02 (SX1278) - DRONE SIDE (SLAVE) - STM32F401RCT6
 * Sesuai pinout skematik: SPI2 + RST/DIO0 di PORT B
 * Serial biasa di-set manual ke PA9 (TX) / PA10 (RX), 115200 baud (debug lokal)
 *
 * TOPOLOGI KOMUNIKASI 2 ARAH (half-duplex, skema PING-PONG):
 *   1) Remote (ESP32, master) mengirim data joystick lalu langsung berpindah
 *      ke mode terima dan menunggu balasan telemetri.
 *   2) Drone (STM32, slave - program ini) selalu diam di mode RX. Begitu
 *      paket joystick diterima, drone langsung membaca sensor BMI160+BMP280
 *      dan MEMBALAS satu paket telemetri biner ringkas ke remote, lalu
 *      kembali ke mode RX menunggu paket berikutnya.
 *   Skema ini memastikan hanya SATU sisi yang memancar pada satu waktu
 *   (tidak ada tabrakan half-duplex) dan bandwidth dipakai bergantian
 *   secara efisien untuk kedua arah.
 *
 * FORMAT PAKET LORA (BINER, fixed-size -> airtime presisi & minimal):
 *   Alasan pakai biner (bukan teks/ASCII CSV): payload ASCII seperti
 *   "0.12,-0.34,...,12.34" bisa 40-60 byte lebih dan airtime-nya (~90-110 ms
 *   pada SF7/BW125k) jauh melebihi timeout balasan yang wajar, sehingga
 *   balasan drone SERING TERLEWAT oleh remote (inilah penyebab GUI selalu
 *   membaca 0 pada versi ASCII sebelumnya). Payload biner fixed-size
 *   menjamin airtime konsisten & dapat dihitung presisi.
 *
 *   Uplink   (Remote -> Drone) : struct UplinkPacket   (6 byte)
 *     uint8_t magic = 0xA5
 *     uint8_t r, t, y, p        (0..255, center 128 utk R/Y/P; T:0=bawah,255=atas)
 *     uint8_t armed             (0 = DISARM, 1 = ARM)
 *
 *   Downlink (Drone -> Remote) : struct DownlinkPacket  (17 byte)
 *     uint8_t magic = 0x5A
 *     int16_t ax, ay, az        (m/s^2  x100, contoh 981 = 9.81 m/s^2)
 *     int16_t gx, gy, gz        (deg/s  x100)
 *     uint16_t press            (hPa    x10 , contoh 10132 = 1013.2 hPa)
 *     int16_t  alt              (meter  x100)
 *     Semua nilai 16-bit dikirim little-endian. TIDAK ada suhu (tidak
 *     dipakai sama sekali pada proyek ini).
 *
 * KONTROL ESC 4 MOTOR:
 *   - Motor 1: PB6 (Depan-Kiri)
 *   - Motor 2: PB7 (Depan-Kanan)
 *   - Motor 3: PB8 (Belakang-Kanan)
 *   - Motor 4: PB9 (Belakang-Kiri)
 *   - Perintah ARM dari GUI -> Remote -> LoRa -> Drone:
 *     1) Kirim 1000 us selama 5 detik (Arming sequence, non-blocking)
 *     2) Setelah 5 detik: ARMED -> 4 motor berputar 20% (1200 us)
 *     3) Jika DISARM / Failsafe (sinyal hilang > 1 detik) -> Motor STOP (1000 us)
 * ==========================================================================
 * PERSIAPAN:
 *  1. Tools > Board > Generic STM32F4 series -> Board part number: Generic F401RCTx
 *  2. Install library "LoRa" (Sandeep Mistry) via Library Manager
 *
 * KONFIGURASI PIN (sesuai skematik):
 *  Serial (USART1, di-remap manual, untuk debug lokal via USB-TTL)
 *    PA9  -> TX0
 *    PA10 -> RX0
 *    Baudrate: 115200
 *
 *  SPI2 (ke modul LoRa RA-02)
 *    PB13 -> RA02_SCK
 *    PB14 -> RA02_MISO
 *    PB15 -> RA02_MOSI
 *    PB12 -> RA02_NSS (CS)
 *    PB1  -> RA02_RST
 *    PB0  -> RA02_DIO0
 *
 *  I2C (ke BMI160 + BMP280, shared bus)
 *    PB10 -> SCL
 *    PB3  -> SDA
 *
 *  PC4 -> LED indikator status ARM/DISARM (aktif LOW)
 * ==========================================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>
#include <string.h>

/* ---------------- Definisi pin LoRa (sesuai skematik) ---------------- */
#define LORA_NSS  PB12
#define LORA_RST  PB1
#define LORA_DIO0 PB0

/* ---------------- Definisi pin ESC 4 Motor ---------------------------- */
#define MOTOR1_PIN PB6   // Motor 1: CCW (Depan-Kiri)
#define MOTOR2_PIN PB7   // Motor 2: CW  (Depan-Kanan)
#define MOTOR3_PIN PB8   // Motor 3: CCW (Belakang-Kanan)
#define MOTOR4_PIN PB9   // Motor 4: CW  (Belakang-Kiri)

/* ---------------- Definisi pin LED status ----------------------------- */
#define LED_PIN PC4      // LED indikator ARM/DISARM di PC4 (aktif LOW)

/* ---------------- Objek SPI2 ---------------- */
SPIClass SPI_2(PB15, PB14, PB13);   // MOSI, MISO, SCK

/* ---------------- Konfigurasi Frekuensi & radio ------------------------ */
/* Harus SAMA dengan sisi remote (ESP32): 433 MHz */
#define LORA_FREQUENCY 433E6

/* ---------------- Konfigurasi failsafe & ESC --------------------------- */
#define LINK_TIMEOUT_MS     1000   // batas hilang sinyal joystick dari remote
#define ESC_MIN_US          1000   // PWM stop / idle (0%)
#define ESC_MAX_US          2000   // PWM full throttle (100%)
#define ESC_ARM_SPIN_US     1200   // 20% throttle saat ARMED (1000 + 1000*0.20)
#define ARMING_DURATION_MS  5000   // waktu tunggu arming ESC (5 detik)

/* ---------------- Objek & State Motor ESC ----------------------------- */
Servo motors[4];
const int MOTOR_PINS[4] = { MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN };

enum EscState {
  ESC_DISARMED,
  ESC_ARMING,
  ESC_ARMED
};

EscState escState = ESC_DISARMED;
unsigned long armingStartMs = 0;

/* ---------------- I2C pins (sensor) ------------------------------------- */
#define I2C_SCL PB10
#define I2C_SDA PB3

/* Alamat I2C sensor */
#define BMI160_ADDR 0x68
#define BMP280_ADDR 0x76

/* ========================= PROTOKOL PAKET BINER ========================= */
#define UPLINK_MAGIC   0xA5
#define DOWNLINK_MAGIC 0x5A

#pragma pack(push, 1)
struct UplinkPacket {
  uint8_t magic;   // harus == UPLINK_MAGIC
  uint8_t r, t, y, p;
  uint8_t armed;   // 0 = DISARM, 1 = ARM
};

struct DownlinkPacket {
  uint8_t  magic;   // harus == DOWNLINK_MAGIC
  int16_t  ax, ay, az;   // m/s^2  x100
  int16_t  gx, gy, gz;   // deg/s  x100
  uint16_t press;        // hPa    x10
  int16_t  alt;          // meter  x100
};
#pragma pack(pop)

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

/* ========================= BMI160 REGISTERS ============================ */
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

/* ========================= BMP280 REGISTERS ============================= */
#define BMP280_REG_CHIP_ID     0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CALIB_00    0x88

#define BMP280_CHIP_ID_VALUE   0x58
#define BMP280_RESET_VALUE     0xB6

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

/* ========================= BMI160 DRIVER ================================ */

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

/* ========================= BMP280 DRIVER ================================= */

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
int32_t t_fine;   // dipakai internal untuk kompensasi tekanan (suhu tidak dipakai/dikirim)

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
  i2c_read_regs(BMP280_ADDR, BMP280_REG_CALIB_00, buf, 26);

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

  /* ctrl_meas: osrs_t=x2, osrs_p=x16, mode=normal (0x57)
     config: t_sb=50ms, filter=x16, spi3w_en=0 (0x90) */
  i2c_write_reg(BMP280_ADDR, BMP280_REG_CONFIG, 0x90);
  i2c_write_reg(BMP280_ADDR, BMP280_REG_CTRL_MEAS, 0x57);

  return true;
}

/* Kompensasi suhu WAJIB dihitung (menghasilkan t_fine) karena dipakai oleh
 * kompensasi tekanan, walau nilai suhunya sendiri tidak dipakai/dikirim. */
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

/* Baca tekanan (hPa) & altitude (m). Suhu TIDAK dikembalikan/dipakai. */
void bmp280_read(float *press_hpa, float *alt_m)
{
  uint8_t buf[6];
  i2c_read_regs(BMP280_ADDR, BMP280_REG_PRESS_MSB, buf, 6);

  int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
  int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);

  bmp280_compensate_T(adc_T);   // wajib, mengisi t_fine
  uint32_t P = bmp280_compensate_P(adc_P);

  // Kompensasi Bosch mengembalikan tekanan dalam Pascal (Pa) -> / 100.0f = hPa
  *press_hpa = P / 100.0f;
  *alt_m = 44330.0f * (1.0f - powf(*press_hpa / 1013.25f, 0.190284f));
}

/* ========================= STATE ========================================= */
bool bmiOK = false;
bool bmpOK = false;
float altitude_offset = 0.0f;
float alt_filtered = 0.0f;

unsigned long lastLinkMs = 0;
bool linkOK = false;

/* ── Gyro Zero-Bias Calibration Offsets ───────────────────────────────── */
float gyro_bias_gx = 0.0f;
float gyro_bias_gy = 0.0f;
float gyro_bias_gz = 0.0f;

/* ========================= SETUP ========================================= */
void setup()
{
  /* Set pin Serial biasa ke PA9 (TX) / PA10 (RX) SEBELUM Serial.begin() */
  Serial.setTx(PA9);
  Serial.setRx(PA10);
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);   // PC4 mati saat startup / disarmed (aktif LOW)

  Serial.println();
  Serial.println("=== LoRa RA-02 STM32F401RCT6 - DRONE (SLAVE) ===");
  Serial.println("Mode: terima joystick & ARM, balas telemetri IMU+altitude.");
  Serial.println("4 Motor ESC: M1(PB6), M2(PB7), M3(PB8), M4(PB9)");
  Serial.println("-------------------------------------------");

  /* ---------------- Inisialisasi 4 Motor ESC (1000 us) ---------------- */
  for (int i = 0; i < 4; i++) {
    motors[i].attach(MOTOR_PINS[i]);
    motors[i].writeMicroseconds(ESC_MIN_US);
  }

  /* ---------------- Inisialisasi sensor ---------------- */
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);

  Serial.print("BMI160 init... ");
  bmiOK = bmi160_init();
  Serial.println(bmiOK ? "OK" : "FAILED");

  /* Kalibrasi Zero-Bias Gyro (kondisi drone diletakkan diam) */
  if (bmiOK) {
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
  bmpOK = bmp280_init();
  if (bmpOK) {
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

  /* ---------------- Inisialisasi LoRa ---------------- */
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  LoRa.setSPI(SPI_2);

  if (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println("GAGAL! Modul LoRa RA-02 tidak terdeteksi.");
    Serial.println("Cek wiring & pastikan VCC = 3.3V.");
    while (1) { delay(1000); }
  }

  /* Parameter radio harus SAMA dengan sisi remote */
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  LoRa.receive();   // langsung siaga di mode terima (slave)

  Serial.println("LoRa siap. Menunggu paket joystick dari remote...");
  Serial.println("-------------------------------------------");
}

/* ========================= LOOP =========================================== */
void loop()
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

      /* ---------------- Update State Motor ESC ---------------- */
      bool armCmd = (up.armed == 1);
      updateEscFSM(armCmd);

      /* ---------------- Baca sensor & susun balasan ---------------- */
      float ax_g = 0, ay_g = 0, az_g = 0, gx_dps = 0, gy_dps = 0, gz_dps = 0;
      float press = 0.0f, alt = 0.0f;

      if (bmiOK) {
        int16_t ax, ay, az, gx, gy, gz;
        if (bmi160_read(&ax, &ay, &az, &gx, &gy, &gz)) {
          // BMI160: +/- 2G -> 16384 LSB/g
          ax_g = (ax / 16384.0f) * 9.80665f;
          ay_g = (ay / 16384.0f) * 9.80665f;
          az_g = (az / 16384.0f) * 9.80665f;

          // BMI160: +/- 2000 dps -> 16.4 LSB/dps
          gx_dps = (gx / 16.4f) - gyro_bias_gx;
          gy_dps = (gy / 16.4f) - gyro_bias_gy;
          gz_dps = (gz / 16.4f) - gyro_bias_gz;

          // Deadband filter: hilangkan noise mikro saat diam (< 0.12 dps)
          if (fabsf(gx_dps) < 0.12f) gx_dps = 0.0f;
          if (fabsf(gy_dps) < 0.12f) gy_dps = 0.0f;
          if (fabsf(gz_dps) < 0.12f) gz_dps = 0.0f;
        }
      }

      if (bmpOK) {
        float raw_p, raw_a;
        if (bmp280_read(&raw_p, &raw_a)) {
          press = raw_p;
          float raw_rel_alt = raw_a - altitude_offset;
          // Digital IIR Low-Pass Filter: y[k] = 0.85 * y[k-1] + 0.15 * x[k]
          alt_filtered = 0.85f * alt_filtered + 0.15f * raw_rel_alt;
          alt = alt_filtered;
        }
      }

      /* Balas ke remote: paket biner ringkas, TANPA suhu */
      DownlinkPacket down;
      down.magic = DOWNLINK_MAGIC;
      down.ax = (int16_t)(ax_g * 100.0f);
      down.ay = (int16_t)(ay_g * 100.0f);
      down.az = (int16_t)(az_g * 100.0f);
      down.gx = (int16_t)(gx_dps * 100.0f);
      down.gy = (int16_t)(gy_dps * 100.0f);
      down.gz = (int16_t)(gz_dps * 100.0f);
      down.press = (uint16_t)(press * 10.0f);
      down.alt = (int16_t)(alt * 100.0f);

      LoRa.beginPacket();
      LoRa.write((uint8_t *)&down, sizeof(DownlinkPacket));
      LoRa.endPacket();

      LoRa.receive();   // kembali siaga menunggu paket joystick berikutnya

      /* Debug lokal via USB-TTL (opsional, tidak dipakai GUI karena drone
         tidak terhubung langsung ke laptop pada topologi final) */
      Serial.print("[RX] R:"); Serial.print(up.r);
      Serial.print(" T:"); Serial.print(up.t);
      Serial.print(" Y:"); Serial.print(up.y);
      Serial.print(" P:"); Serial.print(up.p);
      Serial.print(" | AX:"); Serial.print(ax_g, 2);
      Serial.print(" AY:"); Serial.print(ay_g, 2);
      Serial.print(" AZ:"); Serial.print(az_g, 2);
      Serial.print(" | P:"); Serial.print(press, 2);
      Serial.print(" A:"); Serial.println(alt, 2);
    }
    else
    {
      LoRa.receive();
      Serial.println("[RX] Magic byte uplink tidak cocok, paket diabaikan.");
    }
  }
  else if (packetSize > 0)
  {
    /* Ukuran paket tidak sesuai protokol biner: buang & abaikan */
    while (LoRa.available()) { LoRa.read(); }
    LoRa.receive();
  }

  /* ---------------- Update proses arming saat timer berjalan ---------------- */
  if (escState == ESC_ARMING) {
    updateEscFSM(true);
  }

  /* ---------------- Update LED PC4 (Indikator Arming / Armed) ---------------- */
  updateLedIndicator();

  /* ---------------- Link timeout & Failsafe motor ---------------- */
  if (linkOK && (millis() - lastLinkMs > LINK_TIMEOUT_MS))
  {
    linkOK = false;
    updateEscFSM(false);   // FAILSAFE: matikan motor seketika
    Serial.println("! Link terputus: sinyal remote hilang > 1 detik. Motor STOP.");
  }
}
