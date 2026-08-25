/*
  Bagian untuk Pembacaan Sensor IMU BMI160 (Gyro + Accel) dan Fusi Data
  Diadaptasi dari: Refrensi/SMCStabilization/sensor.ino
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2025
*/

#include "BMI160.h"

BMI160::BMI160(uint8_t i2cAddress, float cfAlpha)
  : _addr(i2cAddress), _cfAlpha(cfAlpha),
    _gx(0), _gy(0), _gz(0),
    _rollRaw(0), _pitchRaw(0),
    _rollCf(0), _pitchCf(0),
    _lastTimeUs(0) {
}

void BMI160::begin() {
  writeRegister(0x7E, 0x15); delay(50); // soft-reset
  writeRegister(0x7E, 0x11); delay(50); // accel: normal mode
  writeRegister(0x7C, 0x00);            // power management
  writeRegister(0x7D, 0x0F);            // gyro: normal mode

  _lastTimeUs = micros();
}

void BMI160::update() {
  uint8_t data[12];
  readRegisters(0x0C, data, 12);

  int16_t gyro_x = (int16_t)(data[1] << 8 | data[0]);
  int16_t gyro_y = (int16_t)(data[3] << 8 | data[2]);
  int16_t gyro_z = (int16_t)(data[5] << 8 | data[4]);
  int16_t acc_x  = (int16_t)(data[7] << 8 | data[6]);
  int16_t acc_y  = (int16_t)(data[9] << 8 | data[8]);
  int16_t acc_z  = (int16_t)(data[11] << 8 | data[10]);

  _gx = gyro_x / 16.4f;
  _gy = gyro_y / 16.4f;
  _gz = gyro_z / 16.4f;

  float ax = acc_x / 16384.0f;
  float ay = acc_y / 16384.0f;
  float az = acc_z / 16384.0f;

  float roll_acc = atan2(ay, az) * 180.0f / PI;
  float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
  _rollRaw = roll_acc;
  _pitchRaw = pitch_acc;

  unsigned long now = micros();
  float dt = (now - _lastTimeUs) / 1000000.0f;
  _lastTimeUs = now;

  _rollCf = _cfAlpha * (_rollCf + _gx * dt) + (1 - _cfAlpha) * roll_acc;
  _pitchCf = _cfAlpha * (_pitchCf + _gy * dt) + (1 - _cfAlpha) * pitch_acc;
}

float BMI160::getRoll() const { return _rollCf; }
float BMI160::getPitch() const { return _pitchCf; }
float BMI160::getRollRaw() const { return _rollRaw; }
float BMI160::getPitchRaw() const { return _pitchRaw; }
float BMI160::getGyroX() const { return _gx; }
float BMI160::getGyroY() const { return _gy; }
float BMI160::getGyroZ() const { return _gz; }

void BMI160::writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void BMI160::readRegisters(uint8_t startReg, uint8_t *buffer, uint8_t len) {
  Wire.beginTransmission(_addr);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(_addr, len);
  for (int i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
}
