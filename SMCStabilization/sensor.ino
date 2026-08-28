/*
  Bagian untuk Pembacaan Sensor dan Fusi Data
  Ditulis oleh: Aji dan Jans
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2025
*/
void updateRollPitch() {
  uint8_t data[12];
  readRegisters(0x0C, data, 12);

  int16_t gyro_x = (int16_t)(data[1] << 8 | data[0]);
  int16_t gyro_y = (int16_t)(data[3] << 8 | data[2]);
  int16_t gyro_z = (int16_t)(data[5] << 8 | data[4]);
  int16_t acc_x  = (int16_t)(data[7] << 8 | data[6]);
  int16_t acc_y  = (int16_t)(data[9] << 8 | data[8]);
  int16_t acc_z  = (int16_t)(data[11] << 8 | data[10]);

  gx = gyro_x / 16.4;
  gy = gyro_y / 16.4;
  gz = gyro_z / 16.4;

  float ax = acc_x / 16384.0;
  float ay = acc_y / 16384.0;
  float az = acc_z / 16384.0;

  float roll_acc = atan2(ay, az) * 180.0 / PI;
  float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll_raw = roll_acc;
  pitch_raw = pitch_acc;
  static unsigned long last_time = millis();
  float dt = (millis() - last_time) / 1000.0;
  last_time = millis();

  roll_cf = alpha_cf * (roll_cf + gx * dt) + (1 - alpha_cf) * roll_acc;
  pitch_cf = alpha_cf * (pitch_cf + gy * dt) + (1 - alpha_cf) * pitch_acc;

  roll = roll_cf;
  pitch = pitch_cf;
}

void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readRegisters(uint8_t startReg, uint8_t *buffer, uint8_t len) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, len);
  for (int i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
}

// === Inisialisasi IMU ===
void initBMI160() {
  writeRegister(0x7E, 0x15); delay(50);
  writeRegister(0x7E, 0x11); delay(50);
  writeRegister(0x7C, 0x00);
  writeRegister(0x7D, 0x0F);
}

