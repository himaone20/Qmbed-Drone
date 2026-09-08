/* ==========================================================================
 * CONTROL.H — Cascade PID Controller (Roll & Pitch)
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

void control_init();
void setPidParams(const PidParams &newParams);
void getPidParams(PidParams *outParams);
void resetPidState();

void computeCascadePid(const SensorData &snap,
                       float targetRollDeg, float targetPitchDeg,
                       float throttlePwm,
                       float &outURoll, float &outUPitch);

#endif // CONTROL_H
