#include <Adafruit_BMP085.h>

/***************************************************
  Modifikasi dari contoh Adafruit BMP085.
  Ditambahkan:
   - Kalman filter 1D sederhana untuk menghaluskan pembacaan altitude
   - Output ke Serial Monitor DAN Serial Plotter (format label:nilai)
   - Fitur "zero"/tare: kirim karakter 'z' (atau '0') lewat Serial Monitor
     untuk menjadikan ketinggian saat ini sebagai referensi 0 meter

  Wiring sama seperti contoh asli:
  VCC -> 3.3V (JANGAN 5V!)
  GND -> GND
  SCL -> A5 (Uno/Duemilanove/dll)
  SDA -> A4
****************************************************/

Adafruit_BMP085 bmp;

// ---------- Konfigurasi ----------
const float SEA_LEVEL_PRESSURE = 101500.0; // Pa, sesuaikan dengan kondisi cuaca
const unsigned long SAMPLE_INTERVAL_MS = 100;

// ---------- Kalman Filter 1D ----------
// Referensi umum: x = estimasi, p = error covariance
// q = process noise, r = measurement noise
struct KalmanFilter1D {
  float q; // process noise covariance
  float r; // measurement noise covariance
  float x; // estimasi nilai
  float p; // estimasi error covariance
  float k; // kalman gain

  void init(float process_noise, float measurement_noise, float initial_value) {
    q = process_noise;
    r = measurement_noise;
    x = initial_value;
    p = 1.0;
  }

  float update(float measurement) {
    // Prediksi
    p = p + q;

    // Update
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1 - k) * p;

    return x;
  }
};

KalmanFilter1D altitudeKF;

// ---------- Variabel Zero/Tare ----------
float altitudeOffset = 0.0;

unsigned long lastSample = 0;

void setup() {
  Serial.begin(9600);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }

  // Inisialisasi Kalman filter dengan pembacaan pertama sebagai nilai awal
  float firstReading = bmp.readAltitude(SEA_LEVEL_PRESSURE);
  // q (process noise) kecil karena altitude berubah perlahan
  // r (measurement noise) lebih besar karena sensor BMP085 cukup berisik
  altitudeKF.init(0.01, 0.5, firstReading);

  Serial.println("BMP085 + Kalman Filter siap.");
  Serial.println("Kirim 'z' atau '0' lewat Serial Monitor untuk set ketinggian saat ini = 0 meter.");
  delay(500);
}

void loop() {
  // Cek perintah zero/tare dari Serial Monitor
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'z' || cmd == 'Z' || cmd == '0') {
      // Set offset = estimasi Kalman saat ini, sehingga bacaan berikutnya jadi 0
      altitudeOffset = altitudeKF.x;
      Serial.println("=== Altitude di-ZERO-kan ===");
    }
  }

  unsigned long now = millis();
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;

    float rawAltitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    float filteredAltitude = altitudeKF.update(rawAltitude);
    float zeroedAltitude = filteredAltitude - altitudeOffset;

    // Format "label:nilai" dipisah spasi -> bisa dibaca di Serial Monitor
    // dan otomatis diplot sebagai 3 garis terpisah di Serial Plotter
    Serial.print("RawAltitude:");
    Serial.print(rawAltitude);
    Serial.print(" FilteredAltitude:");
    Serial.print(filteredAltitude);
    Serial.print(" ZeroedAltitude:");
    Serial.println(zeroedAltitude);
  }
}