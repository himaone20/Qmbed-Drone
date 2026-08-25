/*
  Bagian untuk Pembacaan Sensor Barometer BMP280 (Skeleton/Placeholder)
  Library & alamat I2C aktual belum ditentukan - TODO diisi menyusul.
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2025
*/

#include "BMP280.h"

BMP280::BMP280(uint8_t i2cAddress)
  : _addr(i2cAddress), _altitude(0.0f), _temperature(0.0f), _pressure(0.0f) {
}

void BMP280::begin() {
  // TODO: implementasi setelah library BMP280 ditentukan
  // Contoh rencana: Adafruit_BMP280 / BMx280MI, cek sensor.begin(_addr)
}

void BMP280::update() {
  // TODO: implementasi pembacaan tekanan, suhu, dan estimasi altitude
  // _pressure    = ...;
  // _temperature = ...;
  // _altitude    = ...;
}

float BMP280::getAltitude() const { return _altitude; }
float BMP280::getTemperature() const { return _temperature; }
float BMP280::getPressure() const { return _pressure; }
