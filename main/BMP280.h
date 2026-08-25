/*
  Bagian untuk Pembacaan Sensor Barometer BMP280 (Skeleton/Placeholder)
  Library & alamat I2C aktual belum ditentukan - TODO diisi menyusul.
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2025
*/

#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>
#include <Wire.h>

class BMP280 {
  public:
    BMP280(uint8_t i2cAddress = 0x76);

    // TODO: implementasi inisialisasi sensor setelah library BMP280 ditentukan
    void begin();

    // TODO: implementasi pembacaan tekanan/suhu/altitude
    void update();

    // Getter (placeholder, mengembalikan 0.0f sampai implementasi selesai)
    float getAltitude() const;
    float getTemperature() const;
    float getPressure() const;

  private:
    uint8_t _addr;

    float _altitude;
    float _temperature;
    float _pressure;
};

#endif // BMP280_H
