/* ==========================================================================
 * CONTROL.H — Cascade PID Controller (Roll, Pitch & Yaw Rate Damping)
 * Target: Quadcopter Quad-X (STM32F401RCT6)
 * ==========================================================================
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include "config.h"
#include "sensors.h"

extern float lastURoll;
extern float lastUPitch;
extern float lastUYaw;

void control_init();
void setPidParams(const PidParams &newParams);
void getPidParams(PidParams *outParams);
void resetPidState();

void computeCascadePid(const SensorData &snap,
                       float targetRollDeg, float targetPitchDeg, float targetYawRateDps,
                       float throttlePwm,
                       float &outURoll, float &outUPitch, float &outUYaw);

#endif // CONTROL_H
