/* ==========================================================================
 * MAIN.INO — Flight Controller Drone (FreeRTOS Modular Base)
 * Target Board: STM32F401RCT6 (Arduino IDE + STM32duino + STM32FreeRTOS)
 *
 * Struktur Modul Terpisah Rapi:
 *  - config.h  : Pinout, Parameter SMC, Telemetry Structs
 *  - motors.*  : 4 ESC PWM, FSM Arming (5s), Quad-X Mixer & LED Indikator PC4
 *  - sensors.* : Driver I2C (BMI160 + BMP280), Zero-bias Calibration & TaskSensors
 *  - control.* : Sliding Mode Controller (SMC), Throttle Ramping & TaskControl
 *  - radio.*   : Modul LoRa RA-02 (SPI2), Telemetri 2-Arah & TaskLoRa_Control
 * ==========================================================================
 */

#include <STM32FreeRTOS.h>
#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "control.h"
#include "radio.h"

/* ========================= SETUP ========================================= */
void setup()
{
  Serial.setTx(SERIAL_TX_PIN);
  Serial.setRx(SERIAL_RX_PIN);
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("=== QMBED DRONE FLIGHT CONTROLLER (FreeRTOS) ===");
  Serial.println("Board: STM32F401RCT6 | LoRa RA-02 + 4 ESC (PB6-PB9)");
  Serial.println("-------------------------------------------------");

  /* 1. Inisialisasi Hardware Motor ESC & LED */
  motors_init();

  /* 2. Inisialisasi Module Kendali Throttle & SMC */
  control_init();

  /* 3. Inisialisasi I2C & Kalibrasi Sensor */
  sensors_init();

  /* 4. Inisialisasi Modul Radio LoRa RA-02 (SPI2) */
  if (!radio_init())
  {
    Serial.println("GAGAL! Modul LoRa RA-02 tidak terdeteksi.");
    Serial.println("Cek wiring SPI2 & pastikan VCC = 3.3V.");
    while (1) { delay(1000); }
  }

  Serial.println("LoRa siap. Memulai FreeRTOS scheduler.");
  Serial.println("-------------------------------------------------");

  /* 5. Inisialisasi Mutex & FreeRTOS Tasks */
  sensorMutex = xSemaphoreCreateMutex();
  smcMutex = xSemaphoreCreateMutex();

  xTaskCreate(
    TaskSensors,
    "Sensors",
    TASK_SENSOR_STACK_SIZE,
    NULL,
    TASK_SENSOR_PRIORITY,
    NULL
  );

  xTaskCreate(
    TaskLoRa_Control,
    "LoRa_Ctrl",
    TASK_LORA_STACK_SIZE,
    NULL,
    TASK_LORA_PRIORITY,
    NULL
  );

  xTaskCreate(
    TaskControl,
    "Control",
    TASK_CONTROL_STACK_SIZE,
    NULL,
    TASK_CONTROL_PRIORITY,
    NULL
  );

  /* 6. Start Scheduler */
  vTaskStartScheduler();
}

/* ========================= LOOP =========================================== */
void loop()
{
  // Kosong - scheduler FreeRTOS mengambil alih eksekusi
}
