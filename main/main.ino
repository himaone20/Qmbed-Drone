/*
  Bagian untuk Main Program (FreeRTOS)
  Target Board: STM32F401RCT6 (Arduino IDE + STM32duino + STM32FreeRTOS)
  Tahap saat ini: Struktur task pembacaan sensor BMI160 (aktif) & BMP280 (skeleton)
  Program Studi Teknologi Rekayasa Instrumentasi dan Kontrol
  Departemen Teknik Elektro dan Informatika
  Sekolah Vokasi
  Universitas Gadjah Mada
  2025
*/

#include <STM32FreeRTOS.h>
#include <Wire.h>

#include "config.h"
#include "BMI160.h"
#include "BMP280.h"

HardwareSerial Serial1(PA10, PA9);

BMI160 imu(BMI160_ADDR, CF_ALPHA);
BMP280 baro;

// === Task: Baca Sensor (BMI160 + BMP280) ===
void TaskReadSensors(void *pvParameters) {
  (void) pvParameters;

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TASK_SENSOR_PERIOD_MS);

  for (;;) {
    imu.update();
    baro.update();

    Serial1.print("Roll: ");
    Serial1.print(imu.getRoll(), 2);
    Serial1.print("\tPitch: ");
    Serial1.print(imu.getPitch(), 2);
    Serial1.print("\tAlt: ");
    Serial1.println(baro.getAltitude(), 2);

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

void setup() {
  delay(2000);
  Serial1.begin(115200);

  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.setClock(I2C_CLOCK);
  Wire.begin();
  delay(100);

  imu.begin();
  baro.begin();

  xTaskCreate(
    TaskReadSensors,
    "ReadSensors",
    TASK_SENSOR_STACK_SIZE,
    NULL,
    TASK_SENSOR_PRIORITY,
    NULL
  );

  vTaskStartScheduler();
}

void loop() {
  // Kosong - scheduler FreeRTOS mengambil alih eksekusi setelah vTaskStartScheduler()
}
