/*
  Bagian untuk Pembacaan Sensor IMU BMI160 (Gyro + Accel) dan Fusi Data
  Diadaptasi dari: Refrensi/SMCStabilization/sensor.ino
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2025
*/

#ifndef BMI160_H
#define BMI160_H

#include <Arduino.h>
#include <Wire.h>

class BMI160 {
  public:
    BMI160(uint8_t i2cAddress = 0x68, float cfAlpha = 0.98f);

    // Inisialisasi sensor (soft-reset + set mode normal accel & gyro)
    void begin();

    // Baca data sensor terbaru + update estimasi roll/pitch (complementary filter)
    void update();

    // Getter hasil fusi (derajat)
    float getRoll() const;
    float getPitch() const;

    // Getter raw angle dari accelerometer saja (derajat), untuk keperluan debug
    float getRollRaw() const;
    float getPitchRaw() const;

    // Getter angular rate gyro (derajat/detik)
    float getGyroX() const;
    float getGyroY() const;
    float getGyroZ() const;

  private:
    uint8_t _addr;
    float _cfAlpha;

    float _gx, _gy, _gz;          // gyro (dps)
    float _rollRaw, _pitchRaw;    // sudut dari accelerometer saja (deg)
    float _rollCf, _pitchCf;      // hasil complementary filter (deg)
    unsigned long _lastTimeUs;

    void writeRegister(uint8_t reg, uint8_t val);
    void readRegisters(uint8_t startReg, uint8_t *buffer, uint8_t len);
};

#endif // BMI160_H
