/* ==========================================================================
 * CONTROL.CPP — Cascade PID Flight Controller (Roll & Pitch Only)
 * Target: Quadcopter Quad-X (STM32F401RCT6)
 * ==========================================================================
 */

#include "control.h"

float lastURoll  = 0.0f;
float lastUPitch = 0.0f;

static SemaphoreHandle_t pidMutex = NULL;

static PidParams gPidParams = {
  PID_ANGLE_KP_DEFAULT, PID_ANGLE_KI_DEFAULT, PID_ANGLE_KD_DEFAULT, // Outer Angle (Kp=5.0, Ki=0.05, Kd=0.0)
  PID_RATE_KP_DEFAULT,  PID_RATE_KI_DEFAULT,  PID_RATE_KD_DEFAULT,  // Inner Rate (Kp=1.6, Ki=0.3, Kd=0.045)
  2.00f, 0.15f, 0.00f,                                              // Yaw (not used in mixing)
  PID_MAX_ANGLE_DEG,                                                // maxAngle = 25°
  150.0f,                                                           // maxYawRate
  PID_MAX_DELTA_PWM,                                                // maxDeltaPwm = 300us
  (float)ESC_MIN_US,                                                // escMinPwm = 1000us
  (float)ESC_ARM_SPIN_US,                                           // escArmSpinPwm = 1200us
  (float)ESC_MAX_US                                                 // escMaxPwm = 1300us
};

struct PidChannelState {
  float prevMeasurement;
  float prevError;
  float iTerm;
  float dTerm;
};

static PidChannelState stateAngleRoll;
static PidChannelState stateAnglePitch;
static PidChannelState stateRateRoll;
static PidChannelState stateRatePitch;

static unsigned long lastComputeUs = 0;

void control_init()
{
  pidMutex = xSemaphoreCreateMutex();
  resetPidState();
}

void setPidParams(const PidParams &newParams)
{
  if (pidMutex && xSemaphoreTake(pidMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    gPidParams = newParams;
    xSemaphoreGive(pidMutex);
  } else {
    gPidParams = newParams;
  }
}

void getPidParams(PidParams *outParams)
{
  if (!outParams) return;
  if (pidMutex && xSemaphoreTake(pidMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    *outParams = gPidParams;
    xSemaphoreGive(pidMutex);
  } else {
    *outParams = gPidParams;
  }
}

void resetPidState()
{
  memset(&stateAngleRoll, 0, sizeof(PidChannelState));
  memset(&stateAnglePitch, 0, sizeof(PidChannelState));
  memset(&stateRateRoll, 0, sizeof(PidChannelState));
  memset(&stateRatePitch, 0, sizeof(PidChannelState));
  lastURoll  = 0.0f;
  lastUPitch = 0.0f;
}

/* Outer Loop: Angle Error (deg) -> Desired Angular Rate (deg/s) */
static float computeAngleP(float angleError, float currentRate,
                           float Kp, float Ki, float Kd,
                           PidChannelState &state, float dt,
                           float maxI, float maxOutput, bool allowIntegrate)
{
  float P = Kp * angleError;

  if (allowIntegrate && Ki > 0.0f) {
    state.iTerm += 0.5f * Ki * dt * (angleError + state.prevError);
    state.iTerm = constrain(state.iTerm, -maxI, maxI);
  } else if (!allowIntegrate) {
    state.iTerm = 0.0f;
  }
  state.prevError = angleError;

  // D-term pada outer loop: turunan sudut adalah rate gyro langsung (-Kd * rate)
  float D = -Kd * currentRate;

  float output = P + state.iTerm + D;
  return constrain(output, -maxOutput, maxOutput);
}

/* Inner Loop: Rate Error (deg/s) -> Motor Delta PWM (us) dengan Derivative-on-Measurement */
static float computeRatePID(float rateError, float gyroRate,
                            float Kp, float Ki, float Kd,
                            PidChannelState &state, float dt, float tau,
                            float maxI, float maxOutput, bool allowIntegrate)
{
  float P = Kp * rateError;

  if (allowIntegrate && Ki > 0.0f) {
    state.iTerm += 0.5f * Ki * dt * (rateError + state.prevError);
    state.iTerm = constrain(state.iTerm, -maxI, maxI);
  } else if (!allowIntegrate) {
    state.iTerm = 0.0f;
  }
  state.prevError = rateError;

  // Derivative-on-Measurement: dRate/dt dihitung murni dari gyro measurement (tanpa kick setpoint)
  float deltaMeasurement = (dt > 0.0001f) ? (gyroRate - state.prevMeasurement) / dt : 0.0f;
  state.prevMeasurement = gyroRate;

  // 1st-order Low-pass filter untuk D-term (tau ~ 8ms)
  float dRaw = -Kd * deltaMeasurement;
  float alpha_d = (tau > 0.0001f) ? (dt / (tau + dt)) : 1.0f;
  state.dTerm = state.dTerm + alpha_d * (dRaw - state.dTerm);

  float output = P + state.iTerm + state.dTerm;
  return constrain(output, -maxOutput, maxOutput);
}

void computeCascadePid(const SensorData &snap,
                       float targetRollDeg, float targetPitchDeg,
                       float throttlePwm,
                       float &outURoll, float &outUPitch)
{
  unsigned long nowUs = micros();
  float dt = (lastComputeUs > 0) ? (float)(nowUs - lastComputeUs) * 1e-6f : 0.005f;
  lastComputeUs = nowUs;
  if (dt < 0.0005f || dt > 0.050f) dt = 0.005f;

  // Bila throttle di bawah ambang aktif (1000 - 1150us / cut-off), matikan PID dan reset integrator
  if (throttlePwm < (float)MIX_ACTIVE_MIN_US) {
    resetPidState();
    outURoll  = 0.0f;
    outUPitch = 0.0f;
    lastURoll = 0.0f;
    lastUPitch = 0.0f;
    return;
  }

  PidParams params;
  getPidParams(&params);

  // Integrator hanya aktif saat throttle di atas idle spin (anti-windup di tanah)
  bool allowIntegrate = (throttlePwm >= params.escArmSpinPwm);

  // ── [1] Outer Loop (Angle Controller): Galat Sudut -> Target Laju Putar (°/s) ──
  float galatAngleRoll  = targetRollDeg  - snap.roll;
  float galatAnglePitch = targetPitchDeg - snap.pitch;

  float desiredRateRoll = computeAngleP(galatAngleRoll, snap.gx,
                                        params.angleKp, params.angleKi, params.angleKd,
                                        stateAngleRoll, dt,
                                        PID_INTEGRAL_MAX, PID_MAX_RATE_DPS, allowIntegrate);

  float desiredRatePitch = computeAngleP(galatAnglePitch, snap.gy,
                                         params.angleKp, params.angleKi, params.angleKd,
                                         stateAnglePitch, dt,
                                         PID_INTEGRAL_MAX, PID_MAX_RATE_DPS, allowIntegrate);

  // ── [2] Inner Loop (Rate Controller): Galat Rate -> Koreksi PWM Delta (us) ──
  float galatRateRoll  = desiredRateRoll  - snap.gx;
  float galatRatePitch = desiredRatePitch - snap.gy;

  outURoll = computeRatePID(galatRateRoll, snap.gx,
                            params.rateKp, params.rateKi, params.rateKd,
                            stateRateRoll, dt, PID_TAU_FILTER,
                            PID_INTEGRAL_MAX, params.maxDeltaPwm, allowIntegrate);

  outUPitch = computeRatePID(galatRatePitch, snap.gy,
                             params.rateKp, params.rateKi, params.rateKd,
                             stateRatePitch, dt, PID_TAU_FILTER,
                             PID_INTEGRAL_MAX, params.maxDeltaPwm, allowIntegrate);

  lastURoll  = outURoll;
  lastUPitch = outUPitch;
}
