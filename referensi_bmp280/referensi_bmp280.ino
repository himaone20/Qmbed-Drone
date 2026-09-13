#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <math.h>

Adafruit_BMP280 bmp;

#define FILTER_SAMPLES 20
#define TARE_SAMPLES 50

float tarePressure = 0.0;

float altitudeBuffer[FILTER_SAMPLES];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 tidak ditemukan!");
    while (1);
  }

  // BMP280 configuration
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_125
  );

  // =========================================
  // TARE
  // =========================================

  delay(1000);

  float pressureSum = 0.0;

  for (int i = 0; i < TARE_SAMPLES; i++) {
    pressureSum += bmp.readPressure();
    delay(50);
  }

  tarePressure = pressureSum / TARE_SAMPLES;

  // =========================================
  // INITIALIZE FILTER
  // =========================================

  for (int i = 0; i < FILTER_SAMPLES; i++) {
    altitudeBuffer[i] = 0.0;
  }

  // =========================================
  // START
  // =========================================

  Serial.println("START");
  delay(500);
}

void loop() {

  // =========================================
  // READ PRESSURE
  // =========================================

  float pressure = bmp.readPressure();

  // =========================================
  // CALCULATE RELATIVE ALTITUDE
  // =========================================

  float rawAltitude =
    44330.0 *
    (1.0 - pow(pressure / tarePressure, 0.1902949));

  // Tidak boleh negatif
  if (rawAltitude < 0.0) {
    rawAltitude = 0.0;
  }

  // =========================================
  // MOVING AVERAGE 20 SAMPLE
  // =========================================

  altitudeBuffer[bufferIndex] = rawAltitude;

  bufferIndex++;

  if (bufferIndex >= FILTER_SAMPLES) {
    bufferIndex = 0;
  }

  float sum = 0.0;

  for (int i = 0; i < FILTER_SAMPLES; i++) {
    sum += altitudeBuffer[i];
  }

  float filteredAltitude = sum / FILTER_SAMPLES;

  // Tidak boleh negatif
  if (filteredAltitude < 0.0) {
    filteredAltitude = 0.0;
  }

  // =========================================
  // SERIAL DATA
  // =========================================
  //
  // Format:
  // Raw:0.123 Filtered:0.100
  //
  // Python akan membaca format ini.
  // =========================================

  Serial.print("Raw:");
  Serial.print(rawAltitude, 1);

  Serial.print(" Filtered:");
  Serial.println(filteredAltitude, 1);

  delay(100);
}