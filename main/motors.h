/* ==========================================================================
 * MOTORS.H — Management 4 ESC Motor (Quad-X), FSM Arming & Indikator LED
 * ==========================================================================
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <Servo.h>
#include "config.h"

extern EscState escState;
extern unsigned long armingStartMs;

void motors_init();
void setAllMotorsPWM(int us);
void updateEscFSM(bool armCommand);
void updateLedIndicator();
void writeSmcMotorMix(float basePwm, float uRoll, float uPitch, float uYaw);
int getMotorPWM(int index);

#endif // MOTORS_H
