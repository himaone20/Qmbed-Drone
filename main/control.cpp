/* ==========================================================================
 * CONTROL.CPP — Implementation Kendali SMC, Throttle Ramping & TaskControl
 * ==========================================================================
 */

#include "control.h"

SmcParams gSmcParams = {
  SMC_K1_DEFAULT,
  SMC_K2_DEFAULT,
  SMC_EPS_DEFAULT,
  SMC_FORCE_TO_PWM_DEFAULT,
  SMC_DELTA_MAX_DEFAULT
};
SemaphoreHandle_t smcMutex = NULL;

uint8_t lastRollCmd = 128;
uint8_t lastThrottleCmd = 0;
uint8_t lastYawCmd = 128;
uint8_t lastPitchCmd = 128;

float throttleSmoothed = ESC_ARM_SPIN_US;
static unsigned long throttleRampLastMs = 0;

float lastURoll = 0.0f;
float lastUPitch = 0.0f;
float lastUYaw = 0.0f;

void control_init()
{
  throttleRampLastMs = millis();
}

void updateThrottleCommand(uint8_t t)
{
  int throttleDelta = (int)t - 128;
  if (abs(throttleDelta) <= THROTTLE_DEADBAND) {
    throttleDelta = 0;
  }

  unsigned long now = millis();
  float elapsedSeconds = (now - throttleRampLastMs) / 1000.0f;
  throttleRampLastMs = now;

  // Stick tengah tidak mengubah PWM; nilai throttle terakhir tetap dipakai.
  float stickNorm = throttleDelta >= 0 ?
    (float)throttleDelta / 127.0f : (float)throttleDelta / 128.0f;
  throttleSmoothed += stickNorm * THROTTLE_RATE_US_PER_S * elapsedSeconds;
  throttleSmoothed = constrain(throttleSmoothed,
                               (float)ESC_MIN_US, (float)ESC_MAX_US);
}

void computeSmc(const SensorData &sensor, const SmcParams &params,
                float *uRoll, float *uPitch, float *uYaw)
{
  // Hover mode: target roll/pitch is level. Yaw uses rate damping only.
  float rollError = -sensor.roll;
  float pitchError = -sensor.pitch;
  float rollSurface = -sensor.gx + params.k1 * rollError;
  float pitchSurface = -sensor.gy + params.k1 * pitchError;

  float rollTorque = SMC_IX_DEFAULT *
    (params.k1 * (params.k1 * rollError - sensor.gx) +
     params.k2 * tanhf(rollSurface / params.eps));
  float pitchTorque = SMC_IY_DEFAULT *
    (params.k1 * (params.k1 * pitchError - sensor.gy) +
     params.k2 * tanhf(pitchSurface / params.eps));
  float yawTorque = SMC_IZ_DEFAULT *
    (-params.k1 * sensor.gz - params.k2 * tanhf(sensor.gz / params.eps));

  *uRoll = constrain((rollTorque / SMC_ARM_LENGTH_DEFAULT) * params.forceToPwm,
                     -params.deltaMaxPwm, params.deltaMaxPwm);
  *uPitch = constrain((pitchTorque / SMC_ARM_LENGTH_DEFAULT) * params.forceToPwm,
                      -params.deltaMaxPwm, params.deltaMaxPwm);
  *uYaw = constrain((yawTorque / SMC_ARM_LENGTH_DEFAULT) * params.forceToPwm,
                    -params.deltaMaxPwm, params.deltaMaxPwm);
}

void TaskControl(void *pvParameters)
{
  (void)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS);
  unsigned long lastDebugMs = 0;

  for (;;) {
    if (escState == ESC_ARMED) {
      SensorData sensor = gSensorData;
      SmcParams params = gSmcParams;
      if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        sensor = gSensorData;
        xSemaphoreGive(sensorMutex);
      }
      if (xSemaphoreTake(smcMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        params = gSmcParams;
        xSemaphoreGive(smcMutex);
      }

      updateThrottleCommand(lastThrottleCmd);
      float uRoll = 0.0f, uPitch = 0.0f, uYaw = 0.0f;
      if (sensor.bmiOK && throttleSmoothed >= MIX_ACTIVE_MIN_US) {
        computeSmc(sensor, params, &uRoll, &uPitch, &uYaw);
      }
      lastURoll = uRoll;
      lastUPitch = uPitch;
      lastUYaw = uYaw;
      writeSmcMotorMix(throttleSmoothed, uRoll, uPitch, uYaw);

#if SMC_BENCH_DEBUG
      if (millis() - lastDebugMs >= 100) {
        lastDebugMs = millis();
        Serial.print("[SMC] R:"); Serial.print(sensor.roll, 2);
        Serial.print(" P:"); Serial.print(sensor.pitch, 2);
        Serial.print(" T_CMD:"); Serial.print(lastThrottleCmd);
        Serial.print(" T_PWM:"); Serial.print(throttleSmoothed, 1);
        Serial.print(" U_R:"); Serial.print(uRoll, 1);
        Serial.print(" U_P:"); Serial.print(uPitch, 1);
        Serial.print(" U_Y:"); Serial.println(uYaw, 1);
      }
#endif
    }
    vTaskDelayUntil(&lastWakeTime, period);
  }
}
