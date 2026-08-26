/* ==========================================================================
 * PROGRAM LORA RA-02 (SX1278) - TRANSMITTER + JOYSTICK - ESP32-WROOM-32
 * Mengirim data joystick (MODE 2) sebagai teks CSV tiap 20 Hz ke RX STM32:
 *   "R,T,Y,P"   contoh: "128,200,128,90"
 *   R=ROLL, T=THROTTLE, Y=YAW, P=PITCH (masing-masing 0..255,
 *   center 128 untuk ROLL/PITCH/YAW; throttle bawah=0, atas=255)
 *
 * Mapping joystick (sama dengan program JoystickTest):
 *   J1 kiri : VRX -> GPIO32 -> ROLL,     VRY -> GPIO33 -> THROTTLE
 *   J2 kanan: VRX -> GPIO34 -> YAW,      VRY -> GPIO35 -> PITCH
 *
 * Alur pemrosesan:
 *   RAW ADC (0-4095) -> kalibrasi center -> invert axis -> deadzone
 *   -> filter EMA -> normalisasi 0-255
 * ==========================================================================
 * PERSIAPAN:
 * 1. Tools > Board > pilih "ESP32 Dev Module" (atau board ESP32 lainnya)
 * 2. Install library "LoRa" (Sandeep Mistry) via Library Manager
 *
 * KONFIGURASI PIN (sesuai skematik, sama dengan default VSPI ESP32):
 *   GPIO5  -> RA02_NSS (CS)
 *   GPIO14 -> RA02_RST
 *   GPIO18 -> RA02_SCK
 *   GPIO19 -> RA02_MISO
 *   GPIO23 -> RA02_MOSI
 *   GPIO26 -> RA02_DIO0
 *
 *   Serial debug -> USB bawaan ESP32 (IO1/IO3), tidak perlu di-set manual
 *
 * PARAMETER RADIO harus SAMA dengan sisi receiver (STM32F401):
 *   433 MHz, SF7, BW 125 kHz, CR 4/5, TxPower 17 dBm
 * ==========================================================================
 */

#include <SPI.h>
#include <LoRa.h>

/* ---------------- Definisi pin LoRa (sesuai skematik) ---------------- */
/* SCK=18, MISO=19, MOSI=23 adalah default VSPI ESP32, sehingga library
 * LoRa cukup memakai SPI default tanpa perlu SPI.begin() custom. */
#define LORA_NSS   5
#define LORA_RST   14
#define LORA_DIO0  26

/* RA-02 umumnya versi 433 MHz. Ganti ke 868E6 / 915E6 jika modulmu beda. */
#define LORA_FREQUENCY  433E6

/* ------------------------- konfigurasi joystick --------------------------- */
const int PIN_LEFT_X  = 32;   // ROLL
const int PIN_LEFT_Y  = 33;   // THROTTLE
const int PIN_RIGHT_X = 34;   // YAW
const int PIN_RIGHT_Y = 35;   // PITCH

const int   CALIBRATION_SAMPLES   = 400;   // sampel saat startup (beberapa ratus)
const float JOYSTICK_DEADZONE     = 0.07f; // 7% dari rentang penuh (5-10% disarankan)
const float JOYSTICK_FILTER_ALPHA = 0.35f; // 0..1, makin kecil makin halus
const float SEND_PERIOD_MS        = 50.0f; // 20 Hz (kirim + debug serial)

// Balikkan arah sumbu melalui software jika orientasi modul terbalik.
const bool INVERT_LEFT_X  = true;    // ROLL (kiri = ROLL kiri)
const bool INVERT_LEFT_Y  = true;    // THROTTLE (atas = naik)
const bool INVERT_RIGHT_X = true;    // YAW (kanan = ke kanan)
const bool INVERT_RIGHT_Y = true;    // PITCH (atas = maju)

// Jika sumbu X dan Y joystick tertukar (VRX/VRY terbalik) sehingga
// ROLL dan THROTTLE kebalik, set SWAP_LEFT_X_Y = true.
// SWAP_RIGHT_X_Y untuk joystick kanan (YAW/PITCH).
const bool SWAP_LEFT_X_Y  = true;
const bool SWAP_RIGHT_X_Y = true;

// -------------------------------- state -----------------------------------
const int N_CH = 4;
int  PINS[N_CH];
bool INVERT[N_CH];

int   center[N_CH];
float filt[N_CH];

bool loraOK = false;   // status deteksi modul LoRa saat init

// ------------------------------ fungsi ADC --------------------------------
int readRaw(int ch)
{
  return analogRead(PINS[ch]);
}

void calibrateCenters()
{
  long sum[N_CH] = { 0, 0, 0, 0 };
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    for (int ch = 0; ch < N_CH; ch++) sum[ch] += readRaw(ch);
    delay(2);
  }
  for (int ch = 0; ch < N_CH; ch++) {
    center[ch] = (int)(sum[ch] / CALIBRATION_SAMPLES);
    filt[ch]   = (float)center[ch];
  }
}

int applyInvert(int ch, int raw)
{
  if (INVERT[ch]) return 2 * center[ch] - raw;
  return raw;
}

// ROLL/PITCH/YAW: 128 = tengah, 0 = min, 255 = max
int centerAxisToCmd(int ch, int value)
{
  long delta = value - center[ch];
  long dead  = (long)(JOYSTICK_DEADZONE * 4096.0f);
  if (abs(delta) <= dead) return 128;

  long half = center[ch] > (4095 - center[ch]) ? center[ch] : (4095 - center[ch]);
  float n = (float)delta / (float)half;
  if (n > 1.0f)  n = 1.0f;
  if (n < -1.0f) n = -1.0f;
  return (int)(128 + 128.0f * n);
}

// THROTTLE: 0 = bawah, 255 = atas
int throttleToCmd(int value)
{
  if (value < 0)    value = 0;
  if (value > 4095) value = 4095;
  return (int)(value * 255.0f / 4095.0f);
}

// ----------------------------- proses joystick ----------------------------
// Baca semua channel: invert -> filter EMA -> normalisasi.
// Hasil disimpan di cmd[] : ROLL, THROTTLE, YAW, PITCH (masing-masing 0..255)
void processJoystick(int cmd[N_CH])
{
  for (int ch = 0; ch < N_CH; ch++) {
    int v = applyInvert(ch, readRaw(ch));
    filt[ch] += JOYSTICK_FILTER_ALPHA * (v - filt[ch]);
    if (ch == 1) {                        // LEFT_Y = THROTTLE
      cmd[ch] = throttleToCmd((int)filt[ch]);
    } else {
      cmd[ch] = centerAxisToCmd(ch, (int)filt[ch]);
    }
  }
}

// --------------------------------- setup ----------------------------------
void setup()
{
  PINS[0] = SWAP_LEFT_X_Y  ? PIN_LEFT_Y  : PIN_LEFT_X;   // ROLL
  PINS[1] = SWAP_LEFT_X_Y  ? PIN_LEFT_X  : PIN_LEFT_Y;   // THROTTLE
  PINS[2] = SWAP_RIGHT_X_Y ? PIN_RIGHT_Y : PIN_RIGHT_X;  // YAW
  PINS[3] = SWAP_RIGHT_X_Y ? PIN_RIGHT_X : PIN_RIGHT_Y;  // PITCH
  INVERT[0] = INVERT_LEFT_X;
  INVERT[1] = INVERT_LEFT_Y;
  INVERT[2] = INVERT_RIGHT_X;
  INVERT[3] = INVERT_RIGHT_Y;

  Serial.begin(115200);
  delay(500);
  analogReadResolution(12);               // ADC 12-bit: 0..4095
  for (int ch = 0; ch < N_CH; ch++) {
    analogSetPinAttenuation(PINS[ch], ADC_11db);  // rentang ~0..3.3V
  }

  Serial.println();
  Serial.println("=== LORA RA-02 TRANSMITTER + JOYSTICK - ESP32-WROOM-32 ===");
  Serial.println("Pin: NSS=5, RST=14, SCK=18, MISO=19, MOSI=23, DIO0=26");
  Serial.println("-------------------------------------------------------");

  /* Kalibrasi titik tengah joystick */
  Serial.println("LETAKKAN KEDUA STICK DI POSISI TENGAH.");
  for (int i = 3; i > 0; i--) {
    Serial.print("Kalibrasi mulai dalam ");
    Serial.print(i);
    Serial.println(" detik...");
    delay(1000);
  }
  calibrateCenters();
  Serial.println("=== KALIBRASI JOYSTICK SELESAI ===");

  /* Set pin NSS, RESET, DIO0 untuk modul LoRa (SPI default VSPI) */
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  Serial.print("Menginisialisasi modul LoRa RA-02... ");

  if (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println("GAGAL!");
    Serial.println();
    Serial.println(">> Kemungkinan penyebab:");
    Serial.println("   1. Wiring salah / longgar (cek SCK, MISO, MOSI, NSS, RST)");
    Serial.println("   2. Modul RA-02 rusak");
    Serial.println("   3. VCC modul TIDAK boleh 5V, harus 3.3V");
    Serial.println("   4. Frekuensi modul beda (coba ganti LORA_FREQUENCY)");
    loraOK = false;
  }
  else
  {
    Serial.println("BERHASIL!");
    Serial.println(">> Modul LoRa RA-02 terdeteksi dan siap dipakai.");
    loraOK = true;

    /* Parameter radio harus SAMA dengan sisi receiver */
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setTxPower(17);

    Serial.println("LoRa siap mengirim data joystick (CSV: R,T,Y,P @ 20Hz)...");
  }

  Serial.println("-------------------------------------------------------");
}

// ---------------------------------- loop ----------------------------------
void loop()
{
  if (!loraOK)
  {
    /* Kalau init gagal, cukup ingatkan tiap 1 detik */
    Serial.println("Modul LoRa belum terdeteksi. Cek wiring lalu reset ESP32.");
    delay(1000);
    return;
  }

  int cmd[N_CH];
  processJoystick(cmd);

  /* Kirim data sebagai teks CSV: "R,T,Y,P" */
  LoRa.beginPacket();
  LoRa.print(cmd[0]);
  LoRa.print(',');
  LoRa.print(cmd[1]);
  LoRa.print(',');
  LoRa.print(cmd[2]);
  LoRa.print(',');
  LoRa.print(cmd[3]);
  LoRa.endPacket();

  /* Tampilkan juga di serial monitor untuk pengecekan lokal */
  Serial.print("[TX] R:");
  Serial.print(cmd[0]);
  Serial.print(" T:");
  Serial.print(cmd[1]);
  Serial.print(" Y:");
  Serial.print(cmd[2]);
  Serial.print(" P:");
  Serial.print(cmd[3]);
  Serial.println();

  delay(SEND_PERIOD_MS);
}
