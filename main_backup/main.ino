/* ==========================================================================
 * MAIN.INO — Flight Controller Drone (FreeRTOS Base) - Sensor, Radio, Motor & Cascade PID
 * Target Board: STM32F401RCT6 (Arduino IDE + STM32duino + STM32FreeRTOS)
 *
 * Modul Aktif:
 * - config.h   : Konfigurasi Pinout, Parameter Sensor, PID & Telemetri
 * - sensors.*  : Driver I2C (BMI160 + BMP180), Kalibrasi Boot & Fusi Attitude Roll/Pitch (100Hz)
 * - control.*  : Cascade PID Controller (Roll & Pitch Loop) & Window Tuning GUI Handler
 * - radio.*    : Driver LoRa RA-02 (SPI2) & Telemetri 2-Arah ke Remote / GUI drone_viewer
 * - motors.*   : Driver 4 ESC PWM PB6..PB9 (Quad-X Mixer) & TaskMotors (100Hz)
 * ==========================================================================
 */

#include <STM32FreeRTOS.h>
#include "config.h"
#include "sensors.h"
#include "control.h"
#include "radio.h"
#include "motors.h"

/* ========================= SETUP ========================================= */
void setup()
{
  Serial.setTx(SERIAL_TX_PIN);
  Serial.setRx(SERIAL_RX_PIN);
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("=================================================");
  Serial.println("=== QMBED DRONE FLIGHT CONTROLLER (QUAD-X)    ===");
  Serial.println("=== Target: STM32F401RCT6 | PID + Sensor + LoRa===");
  Serial.println("=================================================");

  /* 1. Inisialisasi Hardware Motor ESC (PB6..PB9) & LED PC4 */
  Serial.println("\n[INIT] Menginisialisasi 4 ESC Motor (PB6..PB9) & LED PC4...");
  motors_init();
  Serial.println("[INIT] 4 ESC Motor Siap (PWM 1000us Stop).");

  /* 2. Inisialisasi Cascade PID Controller (Roll & Pitch) */
  Serial.println("\n[INIT] Menginisialisasi Modul Cascade PID...");
  control_init();
  Serial.println("[INIT] PID Controller Siap.");

  /* 3. Inisialisasi I2C & Kalibrasi Sensor BMI160 / BMP280 */
  Serial.println("\n[INIT] Menginisialisasi Sensor I2C & Kalibrasi Gyro...");
  if (!sensors_init()) {
    Serial.println("[INIT] Peringatan: Inisialisasi sensor mengalami kendala.");
  } else {
    Serial.println("[INIT] Sensor I2C Berhasil Diinisialisasi.");
  }

  /* 4. Inisialisasi Modul Radio LoRa RA-02 (SPI2) */
  Serial.println("\n[INIT] Menginisialisasi Modul LoRa RA-02 (SPI2)...");
  if (!radio_init())
  {
    Serial.println("[INIT] GAGAL! Modul LoRa RA-02 tidak terdeteksi.");
    Serial.println("Cek wiring SPI2 & pastikan VCC = 3.3V.");
    while (1) { delay(1000); }
  }
  Serial.println("[INIT] LoRa RA-02 Siap (433MHz SF7 BW125kHz).");

  Serial.println("\n[INIT] Memulai FreeRTOS Scheduler (TaskMotors, TaskSensors, TaskLoRa_Control)...");
  Serial.println("-------------------------------------------------");

  /* 5. Inisialisasi Mutex & FreeRTOS Tasks */
  sensorMutex = xSemaphoreCreateMutex();

  xTaskCreate(
    TaskMotors,
    "Motors",
    TASK_MOTOR_STACK_SIZE,
    NULL,
    TASK_MOTOR_PRIORITY,
    NULL
  );

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

  /* 6. Start Scheduler */
  vTaskStartScheduler();
}

/* ========================= LOOP =========================================== */
void loop()
{
  // Kosong - FreeRTOS scheduler mengambil alih eksekusi sistem
}
