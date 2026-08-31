/* ==========================================================================
 * MOTORS.CPP — Implementation 4 ESC Motor (Quad-X), FSM Arming & LED
 * ==========================================================================
 */

#include "motors.h"

Servo motors[4];
const int MOTOR_PINS[4] = { MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN };

EscState escState = ESC_DISARMED;
unsigned long armingStartMs = 0;

void motors_init()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // PC4 mati saat startup / disarmed (aktif LOW)

  for (int i = 0; i < 4; i++) {
    motors[i].attach(MOTOR_PINS[i]);
    motors[i].writeMicroseconds(ESC_MIN_US);
  }
}

void setAllMotorsPWM(int us)
{
  if (us < ESC_MIN_US) us = ESC_MIN_US;
  if (us > ESC_MAX_US) us = ESC_MAX_US;
  for (int i = 0; i < 4; i++) {
    motors[i].writeMicroseconds(us);
  }
}

int getMotorPWM(int index)
{
  if (index >= 0 && index < 4) {
    return motors[index].readMicroseconds();
  }
  return ESC_MIN_US;
}

void writeSmcMotorMix(float basePwm, float uRoll, float uPitch, float uYaw)
{
  // Quad-X, arah putaran nyata: M1 FL CW, M2 FR CCW, M3 BR CW, M4 BL CCW.
  // uRoll - = naikkan kanan M2,M3 (koreksi miring kanan), uPitch - = naikkan belakang M3,M4.
  int motorPwm[4] = {
    (int)(basePwm + uRoll + uPitch + uYaw),  // M1 FL CW
    (int)(basePwm - uRoll + uPitch - uYaw),  // M2 FR CCW
    (int)(basePwm - uRoll - uPitch + uYaw),  // M3 BR CW
    (int)(basePwm + uRoll - uPitch - uYaw)   // M4 BL CCW
  };

  for (int i = 0; i < 4; i++) {
    motorPwm[i] = constrain(motorPwm[i], ESC_MIN_US, ESC_MAX_US);
    motors[i].writeMicroseconds(motorPwm[i]);
  }
}

void updateEscFSM(bool armCommand)
{
  if (!armCommand) {
    if (escState != ESC_DISARMED) {
      escState = ESC_DISARMED;
      setAllMotorsPWM(ESC_MIN_US);
      Serial.println("[ESC] DISARMED -> 4 Motor STOP (1000 us)");
    }
    return;
  }

  // armCommand == true
  if (escState == ESC_DISARMED) {
    escState = ESC_ARMING;
    armingStartMs = millis();
    setAllMotorsPWM(ESC_MIN_US);
    Serial.println("[ESC] Mulai Arming 5 detik (1000 us).");
  }
  else if (escState == ESC_ARMING) {
    if (millis() - armingStartMs >= ARMING_DURATION_MS) {
      escState = ESC_ARMED;
      Serial.println("[ESC] ARMED! Motor mengikuti throttle dan Quad-X mixer.");
    } else {
      setAllMotorsPWM(ESC_MIN_US);
    }
  }
}

void updateLedIndicator()
{
  if (escState == ESC_DISARMED) {
    digitalWrite(LED_PIN, HIGH); // MATI saat DISARM (aktif LOW)
  }
  else if (escState == ESC_ARMING) {
    // Blink tiap detik selama 5 detik arming (500ms ON, 500ms OFF)
    unsigned long elapsed = millis() - armingStartMs;
    bool blinkOn = ((elapsed / 500) % 2) == 0;
    digitalWrite(LED_PIN, blinkOn ? LOW : HIGH);
  }
  else if (escState == ESC_ARMED) {
    digitalWrite(LED_PIN, LOW); // KONTINYU NYALA saat ARMED & motor berputar
  }
}
