/* ==========================================================================
 * SENSORS.H — Management I2C Sensor BMI160, BMP180 + Kalman, ADC VBat & TaskSensors
 * ==========================================================================
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include <string.h>
#include "config.h"

extern SensorData gSensorData;
extern SemaphoreHandle_t sensorMutex;

bool sensors_init();
float readVBat();
void TaskSensors(void *pvParameters);

#endif // SENSORS_H
