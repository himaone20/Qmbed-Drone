/* ==========================================================================
 * RADIO.H — Modul Komunikasi 2-Arah LoRa RA-02 (SPI2) & TaskLoRa_Control
 * ==========================================================================
 */

#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <STM32FreeRTOS.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "control.h"

bool radio_init();
void TaskLoRa_Control(void *pvParameters);

#endif // RADIO_H
