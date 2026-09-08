/* ==========================================================================
 * MOTORS.H — Management 4 ESC Motor (Fast 250Hz Hardware PWM TIM4) & TaskMotors
 * Target: STM32F401RCT6 (PB6..PB9 di TIM4 CH1..CH4)
 *
 * Konfigurasi Quad-X:
 * - M1: PB6 (Depan-Kiri / Front-Left, CW)
 * - M2: PB7 (Depan-Kanan / Front-Right, CCW)
 * - M3: PB8 (Belakang-Kanan / Rear-Right, CW)
 * - M4: PB9 (Belakang-Kiri / Rear-Left, CCW)
 * ==========================================================================
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "config.h"

extern int gMotorPWM[4]; // M1, M2, M3, M4 PWM microseconds
extern int gEscMinPwm;
extern int gEscArmSpinPwm;
extern int gEscMaxPwm;

void motors_init();
void setEscPwmLimits(int minPwm, int armSpinPwm, int maxPwm);
void writeMotorMix(float basePwm, float uRoll = 0.0f, float uPitch = 0.0f, float uYaw = 0.0f);
void setAllMotorsPWM(int us);
int  getMotorPWM(int index);
void TaskMotors(void *pvParameters);

#endif // MOTORS_H
