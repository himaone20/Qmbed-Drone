/* ==========================================================================
 * CONTROL.H — Kendali Sliding Mode Controller (SMC), Throttle & TaskControl
 * ==========================================================================
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"

extern SmcParams gSmcParams;
extern SemaphoreHandle_t smcMutex;

extern uint8_t lastRollCmd;
extern uint8_t lastThrottleCmd;
extern uint8_t lastYawCmd;
extern uint8_t lastPitchCmd;

extern float throttleSmoothed;
extern float lastURoll;
extern float lastUPitch;
extern float lastUYaw;

void control_init();
void updateThrottleCommand(uint8_t t);
void computeSmc(const SensorData &sensor, const SmcParams &params,
                float *uRoll, float *uPitch, float *uYaw);
void TaskControl(void *pvParameters);

#endif // CONTROL_H
