/* ==========================================================================
 * PROGRAM LORA RA-02 (SX1278) - RECEIVER + 4 ESC/MOTOR - STM32F401RCT6
 * Sesuai pinout skematik: SPI2 + RST/DIO0 di PORT B
 * Serial biasa di-set manual ke PA9 (TX) / PA10 (RX), 115200 baud
 *
 * Menerima data joystick dari transmitter ESP32 dalam format teks CSV:
 *   "R,T,Y,P"   contoh: "128,200,128,90"
 *   R=ROLL, T=THROTTLE, Y=YAW, P=PITCH (masing-masing 0..255,
 *   center 128 untuk ROLL/PITCH/YAW; throttle bawah=0, atas=255)
 *
 * KONFIGURASI MOTOR (Quad X):
 *   Motor 1 : CCW | DEPAN-KIRI      -> PB6
 *   Motor 2 : CW  | DEPAN-KANAN     -> PB7
 *   Motor 3 : CCW | BELAKANG-KANAN  -> PB8
 *   Motor 4 : CW  | BELAKANG-KIRI   -> PB9
 *
 * KONVERSI CHANNEL (persis seperti DroneSimulator / _on_channels):
 *   roll_pct  = (R - 128) / 127 * 100      -> -100..+100
 *   yaw_pct   = (Y - 128) / 127 * 100      -> -100..+100
 *   pitch_pct = (P - 128) / 127 * 100      -> -100..+100 (positif = maju)
 *   THROTTLE: TENGAH = 0%  -> T <= 128 : 0%, di atas tengah : 0..100%
 *
 * MIXING QUAD-X:
 *   M1 (FL CCW) = T + Roll - Pitch + Yaw
 *   M2 (FR CW)  = T - Roll - Pitch - Yaw
 *   M3 (RR CCW) = T - Roll + Pitch + Yaw
 *   M4 (RL CW)  = T + Roll + Pitch - Yaw
 *   (roll kanan -> motor kiri naik; pitch maju -> motor belakang naik;
 *    yaw kanan -> motor CCW naik). Hasil di-clamp 0..100% lalu
 *    dikonversi ke pulsa 1000..2000 us.
 *
 * Respons motor diberi smoothing eksponensial (sama seperti drone_model.py)
 * agar gerakan halus, bukan patah-patah.
 *
 * KEAMANAN:
 *   - Saat boot semua ESC di-set ke throttle minimum (1000 us)
 *   - Arming: menunggu 5 detik sebelum siap
 *   - FAILSAFE: jika sinyal LoRa hilang > 1 detik, semua motor
 *     langsung (tanpa smoothing) kembali ke 1000 us + LED kedip cepat
 *   - Emergency STOP: ketik "STOP" di serial monitor -> motor minimum.
 *     Normal kembali hanya setelah throttle kembali ke posisi tengah/bawah.
 * ==========================================================================
 * PERSIAPAN:
 * 1. Tools > Board > Generic STM32F4 series -> Board part number: Generic F401RCTx
 * 2. Install library "LoRa" (Sandeep Mistry) via Library Manager
 * 3. Library Servo sudah bawaan core STM32duino
 *
 * KONFIGURASI PIN (sesuai skematik):
 *   Serial (USART1, di-remap manual)
 *     PA9  -> TX0
 *     PA10 -> RX0
 *     Baudrate: 115200
 *
 *   SPI2 (ke modul LoRa RA-02)
 *     PB13 -> RA02_SCK
 *     PB14 -> RA02_MISO
 *     PB15 -> RA02_MOSI
 *     PB12 -> RA02_NSS (CS)
 *     PB1  -> RA02_RST
 *     PB0  -> RA02_DIO0
 *
 *   ESC (signal, semua channel TIM4)
 *     PB6 -> ESC Motor 1
 *     PB7 -> ESC Motor 2
 *     PB8 -> ESC Motor 3
 *     PB9 -> ESC Motor 4
 *     PC13 -> LED status (bawaan board, aktif LOW)
 * ==========================================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>
#include <math.h>

/* ---------------- Definisi pin LoRa (sesuai skematik) ---------------- */
#define LORA_NSS   PB12
#define LORA_RST   PB1
#define LORA_DIO0  PB0

/* ---------------- Definisi pin ESC & LED ----------------------------- */
#define MOTOR1_PIN  PB6    // CCW, DEPAN-KIRI
#define MOTOR2_PIN  PB7    // CW , DEPAN-KANAN
#define MOTOR3_PIN  PB8    // CCW, BELAKANG-KANAN
#define MOTOR4_PIN  PB9    // CW , BELAKANG-KIRI
#define LED_PIN     PC13   // LED board, aktif LOW

/* ---------------- Objek SPI2 ---------------- */
SPIClass SPI_2(PB15, PB14, PB13);   // MOSI, MISO, SCK

/* ---------------- Konfigurasi Frekuensi ---------------- */
/* Harus SAMA dengan transmitter (ESP32): 433 MHz */
#define LORA_FREQUENCY  433E6

/* ---------------- Konfigurasi ESC & Mixing --------------------------- */
#define ESC_MIN_US          1000    // throttle minimum (0%)
#define ESC_MAX_US          2000    // throttle maksimum (100%)
#define ARM_DELAY_MS        5000    // waktu tunggu arming ESC
#define FAILSAFE_TIMEOUT_MS 1000    // batas hilang sinyal LoRa
#define THROTTLE_CENTER     128     // stick tengah = 0%
#define SMOOTH_RATE         3.0f    // konstanta smoothing (sama dgn simulator)
#define LOOP_DT             0.05f   // perkiraan periode paket (20 Hz)

Servo motors[4];
const int MOTOR_PINS[4] = { MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN };

/* ---------------- State ------------------------------------------------ */
unsigned long lastPacketMs = 0;
bool failsafeActive  = true;    // mulai dalam kondisi aman (belum ada sinyal)
bool emergencyStop   = false;   // terkunci setelah perintah STOP dari serial
unsigned long ledBlinkMs = 0;
bool ledState = false;

float mPct[4] = { 0, 0, 0, 0 }; // keluaran mixing yang sudah di-smooth (%)

/* ---------------- Fungsi bantu ESC ------------------------------------- */
void forceStop()
{
  for (int i = 0; i < 4; i++) {
    mPct[i] = 0.0f;
    motors[i].writeMicroseconds(ESC_MIN_US);
  }
}

/*
 * Mixing Quad-X + smoothing.
 * Semua input dalam persen (-100..+100, kecuali thr 0..100).
 * Arah sesuai konvensi DroneSimulator:
 *   Roll  positif = miring ke kanan  -> motor kiri naik
 *   Pitch positif = maju             -> motor belakang naik
 *   Yaw   positif = putar ke kanan   -> motor CCW naik
 */
void applyMix(float rollPct, float yawPct, float pitchPct, float thrPct)
{
  float raw[4];
  raw[0] = thrPct + rollPct - pitchPct + yawPct;   // M1 FL CCW
  raw[1] = thrPct - rollPct - pitchPct - yawPct;   // M2 FR CW
  raw[2] = thrPct - rollPct + pitchPct + yawPct;   // M3 RR CCW
  raw[3] = thrPct + rollPct + pitchPct - yawPct;   // M4 RL CW

  /* Smoothing eksponensial ala drone_model.py: k = 1 - e^(-rate*dt) */
  float k = 1.0f - expf(-SMOOTH_RATE * LOOP_DT);

  for (int i = 0; i < 4; i++) {
    if (raw[i] < 0.0f)   raw[i] = 0.0f;
    if (raw[i] > 100.0f) raw[i] = 100.0f;

    mPct[i] += (raw[i] - mPct[i]) * k;

    int pulse = ESC_MIN_US + (int)(mPct[i] * 10.0f);
    if (pulse < ESC_MIN_US) pulse = ESC_MIN_US;
    if (pulse > ESC_MAX_US) pulse = ESC_MAX_US;
    motors[i].writeMicroseconds(pulse);
  }
}

void setup()
{
  /* Set pin Serial biasa ke PA9 (TX) / PA10 (RX) SEBELUM Serial.begin() */
  Serial.setTx(PA9);
  Serial.setRx(PA10);
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);   // LED nyala saat program jalan (aktif LOW)

  Serial.println();
  Serial.println("=== LoRa RA-02 STM32F401RCT6 - RECEIVER + 4 ESC ===");
  Serial.println("Motor1(PB6,CCW,DepanKiri) Motor2(PB7,CW,DepanKanan)");
  Serial.println("Motor3(PB8,CCW,BelakangKanan) Motor4(PB9,CW,BelakangKiri)");
  Serial.println("Throttle: TENGAH=0%, atas=100% | Mixing Quad-X aktif");
  Serial.println("-------------------------------------------");

  /* SAFETY: semua ESC ke throttle minimum SEBELUM apapun */
  for (int i = 0; i < 4; i++) {
    motors[i].attach(MOTOR_PINS[i]);
    motors[i].writeMicroseconds(ESC_MIN_US);
  }

  /* Set pin NSS, RESET, DIO0 untuk modul LoRa */
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  /* Arahkan library LoRa supaya pakai SPI2, bukan SPI1 default */
  LoRa.setSPI(SPI_2);

  if (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println("GAGAL! Modul LoRa RA-02 tidak terdeteksi.");
    Serial.println("Cek wiring & pastikan VCC = 3.3V.");
    while (1)
    {
      delay(1000);
    }
  }

  /* Parameter radio harus SAMA dengan sisi transmitter */
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  Serial.println("LoRa RA-02 siap menerima data joystick...");

  /* Tunggu ESC melakukan arming (sama seperti program ESC test) */
  Serial.println("ARMING ESC...");
  forceStop();
  for (int i = ARM_DELAY_MS / 1000; i > 0; i--)
  {
    Serial.print("Arming dalam ");
    Serial.print(i);
    Serial.println(" detik...");
    delay(1000);
  }
  forceStop();

  Serial.println("=== ESC READY (throttle = 0%) ===");
  Serial.println("Emergency STOP: ketik STOP di serial monitor");
  Serial.println("-------------------------------------------");
}

void loop()
{
  /* ---------- Perintah darurat dari serial monitor ---------- */
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("STOP"))
    {
      emergencyStop = true;
      forceStop();
      Serial.println("!!! EMERGENCY STOP !!!");
      Serial.println("Motor dikunci di 1000 us.");
      Serial.println("Kembalikan throttle ke TENGAH/BAWAH untuk membuka kunci.");
    }
  }

  /* ---------- Terima paket LoRa ---------- */
  int packetSize = LoRa.parsePacket();

  if (packetSize)
  {
    String received = "";
    while (LoRa.available())
    {
      received += (char)LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    /* Parsing format CSV "R,T,Y,P" */
    int rVal, tVal, yVal, pVal;
    if (sscanf(received.c_str(), "%d,%d,%d,%d", &rVal, &tVal, &yVal, &pVal) == 4)
    {
      lastPacketMs = millis();

      if (failsafeActive)
      {
        failsafeActive = false;
        digitalWrite(LED_PIN, LOW);
        Serial.println(">>> Sinyal kembali normal <<<");
      }

      /* Konversi channel persis seperti DroneSimulator._on_channels */
      float rollPct  = (rVal - 128) / 127.0f * 100.0f;
      float yawPct   = (yVal - 128) / 127.0f * 100.0f;
      float pitchPct = (pVal - 128) / 127.0f * 100.0f;
      /* Throttle: TENGAH = 0% (setengah bawah stick tidak berfungsi) */
      float thrPct   = (tVal <= THROTTLE_CENTER)
                       ? 0.0f
                       : (tVal - THROTTLE_CENTER) / 127.0f * 100.0f;

      /* Buka kunci emergency stop hanya jika sudah di tengah/bawah */
      if (emergencyStop && tVal <= THROTTLE_CENTER + 5)
      {
        emergencyStop = false;
        Serial.println(">>> Emergency stop dilepas (throttle di tengah/bawah). <<<");
      }

      if (emergencyStop || failsafeActive)
      {
        /* Kondisi tidak aman: paksa motor ke minimum tanpa smoothing */
        forceStop();
      }
      else
      {
        applyMix(rollPct, yawPct, pitchPct, thrPct);
      }

      /* Status untuk serial monitor */
      Serial.print("[RX] R:");
      Serial.print(rVal);
      Serial.print(" T:");
      Serial.print(tVal);
      Serial.print(" Y:");
      Serial.print(yVal);
      Serial.print(" P:");
      Serial.print(pVal);
      Serial.print(" | M:");
      for (int i = 0; i < 4; i++)
      {
        Serial.print(ESC_MIN_US + (int)(mPct[i] * 10.0f));
        Serial.print(i < 3 ? "/" : " us");
      }
      if (emergencyStop)  Serial.print(" [ESTOP]");
      if (failsafeActive) Serial.print(" [FAILSAFE]");
      Serial.print(" | RSSI: ");
      Serial.print(rssi);
      Serial.print(" dBm | SNR: ");
      Serial.println(snr);
    }
    else
    {
      /* Format tidak dikenal: tampilkan apa adanya */
      Serial.print("[RX] Data mentah: ");
      Serial.print(received);
      Serial.print(" | RSSI: ");
      Serial.print(rssi);
      Serial.print(" dBm | SNR: ");
      Serial.println(snr);
    }
  }

  /* ---------- FAILSAFE: sinyal hilang lebih lama dari batas ---------- */
  if (!failsafeActive && (millis() - lastPacketMs > FAILSAFE_TIMEOUT_MS))
  {
    failsafeActive = true;
    forceStop();
    Serial.println("!!! FAILSAFE: sinyal LoRa hilang > 1 detik. Motor = 0%. !!!");
  }

  /* ---------- Indikator LED ---------- */
  if (failsafeActive)
  {
    /* Kedip cepat saat tidak ada sinyal */
    if (millis() - ledBlinkMs > 100)
    {
      ledBlinkMs = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
  }
  else
  {
    digitalWrite(LED_PIN, LOW);   // nyala terus saat sinyal normal
  }
}
