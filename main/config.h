/*
  Bagian untuk Konfigurasi Umum (Pin, Alamat I2C, Parameter Task)
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2025
*/

#ifndef CONFIG_H
#define CONFIG_H

// === Konfigurasi Pin I2C (dipakai bersama oleh BMI160 & BMP280) ===
#define I2C_SDA PB7
#define I2C_SCL PB6
#define I2C_CLOCK 400000

// === Alamat I2C Sensor ===
#define BMI160_ADDR 0x68
// #define BMP280_ADDR 0x76   // TODO: tentukan alamat BMP280 saat referensi library sudah ada

// === Parameter Complementary Filter (BMI160) ===
#define CF_ALPHA 0.98f

// === Parameter Task FreeRTOS: Task Baca Sensor ===
#define TASK_SENSOR_PERIOD_MS   4     // 250 Hz, samakan dengan referensi (dt >= 0.004 s)
#define TASK_SENSOR_PRIORITY    2
#define TASK_SENSOR_STACK_SIZE  256   // dalam words, sesuaikan lagi saat implementasi penuh

#endif // CONFIG_H
