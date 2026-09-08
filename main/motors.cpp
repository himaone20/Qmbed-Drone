/* ==========================================================================
 * MOTORS.CPP — Management 4 ESC Motor (Fast 250Hz Hardware PWM TIM4) & TaskMotors
 * Target: STM32F401RCT6 (PB6..PB9 di TIM4 CH1..CH4)
 *
 * Konfigurasi Quad-X & Arah Putar:
 * - M1 (PB6): Depan-Kiri   (Front-Left, CW)   -> TIM4 CH1
 * - M2 (PB7): Depan-Kanan  (Front-Right, CCW)  -> TIM4 CH2
 * - M3 (PB8): Belakang-Kanan (Rear-Right, CW)  -> TIM4 CH3
 * - M4 (PB9): Belakang-Kiri  (Rear-Left, CCW)  -> TIM4 CH4
 * ==========================================================================
 */

#include "motors.h"
#include "radio.h"
#include "control.h"
#include "sensors.h"

static HardwareTimer *timerMotors = NULL;
static uint32_t chan1, chan2, chan3, chan4;

int gMotorPWM[4] = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };
int gEscMinPwm = ESC_MIN_US;         // 1000us (Stop / Disarmed)
int gEscArmSpinPwm = ESC_ARM_SPIN_US; // 1200us (Idle spin saat Armed)
int gEscMaxPwm = ESC_MAX_US;         // 1300us (PWM Maksimum)

void setEscPwmLimits(int minPwm, int armSpinPwm, int maxPwm)
{
  if (minPwm >= 900 && minPwm <= 1400) {
    gEscMinPwm = minPwm;
  }
  if (armSpinPwm >= gEscMinPwm && armSpinPwm <= 1600) {
    gEscArmSpinPwm = armSpinPwm;
  }
  if (maxPwm >= 1100 && maxPwm <= 2200 && maxPwm >= gEscArmSpinPwm) {
    gEscMaxPwm = maxPwm;
  }
}

void motors_init()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED mati saat awal (Aktif LOW)

  // Inisialisasi HardwareTimer TIM4 pada 250 Hz (periode 4000us)
  // Kecepatan respon 5x lebih cepat daripada Servo 50Hz, bebas lag
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(MOTOR1_PIN), PinMap_PWM);
  timerMotors = new HardwareTimer(Instance);

  chan1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(MOTOR1_PIN), PinMap_PWM));
  chan2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(MOTOR2_PIN), PinMap_PWM));
  chan3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(MOTOR3_PIN), PinMap_PWM));
  chan4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(MOTOR4_PIN), PinMap_PWM));

  timerMotors->setMode(chan1, TIMER_OUTPUT_COMPARE_PWM1, MOTOR1_PIN);
  timerMotors->setMode(chan2, TIMER_OUTPUT_COMPARE_PWM1, MOTOR2_PIN);
  timerMotors->setMode(chan3, TIMER_OUTPUT_COMPARE_PWM1, MOTOR3_PIN);
  timerMotors->setMode(chan4, TIMER_OUTPUT_COMPARE_PWM1, MOTOR4_PIN);

  timerMotors->setOverflow(250, HERTZ_FORMAT); // 250 Hz Hardware PWM
  timerMotors->setCaptureCompare(chan1, gEscMinPwm, MICROSEC_COMPARE_FORMAT);
  timerMotors->setCaptureCompare(chan2, gEscMinPwm, MICROSEC_COMPARE_FORMAT);
  timerMotors->setCaptureCompare(chan3, gEscMinPwm, MICROSEC_COMPARE_FORMAT);
  timerMotors->setCaptureCompare(chan4, gEscMinPwm, MICROSEC_COMPARE_FORMAT);
  timerMotors->resume();

  setAllMotorsPWM(gEscMinPwm);
}

void setAllMotorsPWM(int us)
{
  us = constrain(us, gEscMinPwm, gEscMaxPwm);
  if (timerMotors) {
    timerMotors->setCaptureCompare(chan1, us, MICROSEC_COMPARE_FORMAT);
    timerMotors->setCaptureCompare(chan2, us, MICROSEC_COMPARE_FORMAT);
    timerMotors->setCaptureCompare(chan3, us, MICROSEC_COMPARE_FORMAT);
    timerMotors->setCaptureCompare(chan4, us, MICROSEC_COMPARE_FORMAT);
  }

  for (int i = 0; i < 4; i++) {
    gMotorPWM[i] = us;
  }
}

void writeMotorMix(float basePwm, float uRoll, float uPitch, float uYaw)
{
  /*
   * Persamaan Motor Mixing Quad-X:
   * M1 (FL CW):  basePwm + uRoll + uPitch - uYaw
   * M2 (FR CCW): basePwm - uRoll + uPitch + uYaw
   * M3 (BR CW):  basePwm - uRoll - uPitch - uYaw
   * M4 (BL CCW): basePwm + uRoll - uPitch + uYaw
   */
  float m1 = basePwm + uRoll + uPitch - uYaw;
  float m2 = basePwm - uRoll + uPitch + uYaw;
  float m3 = basePwm - uRoll - uPitch - uYaw;
  float m4 = basePwm + uRoll - uPitch + uYaw;

  // Attitude-Priority Desaturation:
  // Jaga diferensial torsi Roll & Pitch tetap 100% utuh saat mencapai limit PWM
  float maxM = max(max(m1, m2), max(m3, m4));
  float minM = min(min(m1, m2), min(m3, m4));

  if (maxM > (float)gEscMaxPwm) {
    float over = maxM - (float)gEscMaxPwm;
    m1 -= over;
    m2 -= over;
    m3 -= over;
    m4 -= over;
  }

  if (minM < (float)gEscArmSpinPwm && basePwm >= (float)gEscArmSpinPwm) {
    float under = (float)gEscArmSpinPwm - minM;
    m1 += under;
    m2 += under;
    m3 += under;
    m4 += under;
  }

  int out1 = constrain((int)round(m1), gEscMinPwm, gEscMaxPwm);
  int out2 = constrain((int)round(m2), gEscMinPwm, gEscMaxPwm);
  int out3 = constrain((int)round(m3), gEscMinPwm, gEscMaxPwm);
  int out4 = constrain((int)round(m4), gEscMinPwm, gEscMaxPwm);

  if (timerMotors) {
    timerMotors->setCaptureCompare(chan1, out1, MICROSEC_COMPARE_FORMAT);
    timerMotors->setCaptureCompare(chan2, out2, MICROSEC_COMPARE_FORMAT);
    timerMotors->setCaptureCompare(chan3, out3, MICROSEC_COMPARE_FORMAT);
    timerMotors->setCaptureCompare(chan4, out4, MICROSEC_COMPARE_FORMAT);
  }

  gMotorPWM[0] = out1;
  gMotorPWM[1] = out2;
  gMotorPWM[2] = out3;
  gMotorPWM[3] = out4;
}

int getMotorPWM(int index)
{
  if (index >= 0 && index < 4) {
    return gMotorPWM[index];
  }
  return gEscMinPwm;
}

void TaskMotors(void *pvParameters)
{
  (void) pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TASK_MOTOR_PERIOD_MS);

  for (;;)
  {
    // 1. Eksekusi Pembacaan IMU & Fusi Sikap Sinkron 200 Hz (< 1ms Zero-Lag Pipeline)
    SensorData snap;
    bool imuOk = sensors_step_imu(snap);

    if (!gArmedCmd || !imuOk)
    {
      // ── Status DISARM (Aman / Cut-off) ──
      digitalWrite(LED_PIN, HIGH); // LED PC4 Mati
      setAllMotorsPWM(gEscMinPwm);  // Stop Total
      resetPidState();
    }
    else
    {
      // ── Status ARMED (Motor Live & Fast Cascade PID 200 Hz) ──
      digitalWrite(LED_PIN, LOW); // LED PC4 Hidup

      float targetThrottle = (float)gTargetThrottlePwm;
      targetThrottle = constrain(targetThrottle, (float)gEscMinPwm, (float)gEscMaxPwm);

      // Hitung koreksi Cascade PID untuk Roll & Pitch
      float uRoll = 0.0f;
      float uPitch = 0.0f;
      computeCascadePid(snap, gTargetRollDeg, gTargetPitchDeg, targetThrottle, uRoll, uPitch);

      // Terapkan hasil pencampuran Base Throttle + Koreksi PID ke 4 Motor ESC via Fast Hardware PWM
      writeMotorMix(targetThrottle, uRoll, uPitch, 0.0f);
    }

    // 2. Poll Telemetri Non-Blocking (Baro BMP180 & ADC VBat)
    sensors_poll_telemetry();

    // 3. Salin data snapshot sensor ke shared gSensorData untuk radio LoRa
    if (sensorMutex && xSemaphoreTake(sensorMutex, 0) == pdTRUE) {
      gSensorData.ax = snap.ax;
      gSensorData.ay = snap.ay;
      gSensorData.az = snap.az;
      gSensorData.gx = snap.gx;
      gSensorData.gy = snap.gy;
      gSensorData.gz = snap.gz;
      gSensorData.roll = snap.roll;
      gSensorData.pitch = snap.pitch;
      gSensorData.yawRate = snap.yawRate;
      gSensorData.bmiOK = snap.bmiOK;
      xSemaphoreGive(sensorMutex);
    }

    vTaskDelayUntil(&lastWakeTime, period);
  }
}
