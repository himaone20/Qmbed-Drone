/* ==========================================================================
 * PROGRAM LORA RA-02 (SX1278) - REMOTE SIDE (MASTER) - ESP32-WROOM-32
 * Hub tunggal antara laptop (GUI GCS) <-> drone via LoRa RA-02.
 *
 * TOPOLOGI KOMUNIKASI 2 ARAH (half-duplex, skema PING-PONG):
 *   1) Remote (program ini, MASTER) membaca joystick lalu mengirim paket
 *      biner ke drone, kemudian langsung berpindah ke mode terima dan
 *      menunggu balasan telemetri dengan jendela waktu terbatas (timeout).
 *   2) Drone (STM32, SLAVE) menerima joystick, membaca sensor BMI160+BMP280,
 *      lalu membalas satu paket telemetri biner ringkas.
 *   3) Remote menerima balasan (atau timeout), lalu meneruskan SEMUA data
 *      (joystick + telemetri) ke laptop melalui SATU port USB serial dalam
 *      format teks yang sudah dikenali GUI drone_viewer.py:
 *        [TX] R:.. T:.. Y:.. P:..
 *        [IMU] AX:.. AY:.. AZ:.. GX:.. GY:.. GZ:..
 *        [BMP] P:.. A:..
 *   Skema ini menjamin hanya SATU sisi radio yang memancar pada satu waktu
 *   (tidak ada tabrakan half-duplex) dan bandwidth LoRa dipakai bergantian
 *   secara efisien untuk kedua arah.
 *
 * FORMAT PAKET LORA (BINER, fixed-size -> airtime presisi & minimal):
 *   Payload ASCII CSV ("0.12,-0.34,...,12.34") pada versi sebelumnya bisa
 *   40-60 byte -> airtime ~90-110 ms (SF7/BW125k), jauh melebihi jendela
 *   timeout balasan yang wajar sehingga balasan drone SERING TERLEWAT
 *   (itulah sebab GUI selalu membaca 0 pada versi ASCII). Payload biner
 *   fixed-size membuat airtime presisi & dapat dihitung, sehingga timeout
 *   bisa diset tepat tanpa membuang bandwidth.
 *
 *   Uplink   (Remote -> Drone) : struct UplinkPacket   (6 byte)
 *     uint8_t magic = 0xA5
 *     uint8_t r, t, y, p
 *     uint8_t armed             (0 = DISARM, 1 = ARM)
 *
 *   Downlink (Drone -> Remote) : struct DownlinkPacket  (27 byte)
 *     uint8_t magic = 0x5A
 *     int16_t ax, ay, az        (m/s^2  x100)
 *     int16_t gx, gy, gz        (deg/s  x100)
 *     uint16_t press            (hPa    x10)
 *     int16_t  alt              (meter  x100)
 *     int16_t roll, pitch        (derajat x100, filter onboard)
 *     int16_t uRoll, uPitch, uYaw (koreksi SMC PWM x100)
 *     TIDAK ada suhu (tidak dipakai sama sekali pada proyek ini).
 *
 * Mapping joystick (sama dengan program JoystickTest):
 *   J1 kiri : VRX -> GPIO32 -> ROLL,  VRY -> GPIO33 -> THROTTLE
 *   J2 kanan: VRX -> GPIO34 -> YAW,   VRY -> GPIO35 -> PITCH
 *
 * Alur pemrosesan joystick:
 *   RAW ADC (0-4095) -> kalibrasi center -> invert axis -> deadzone
 *   -> filter EMA -> normalisasi 0-255
 *
 * CATATAN: Data suhu (temperature) TIDAK dipakai sama sekali pada proyek
 * ini (tidak dikirim lewat LoRa, tidak dicetak ke USB, tidak ditampilkan
 * di GUI).
 * ==========================================================================
 * PERSIAPAN:
 *  1. Tools > Board > pilih "ESP32 Dev Module" (atau board ESP32 lainnya)
 *  2. Install library "LoRa" (Sandeep Mistry) via Library Manager
 *
 * KONFIGURASI PIN (sesuai skematik, sama dengan default VSPI ESP32):
 *  GPIO5  -> RA02_NSS (CS)
 *  GPIO14 -> RA02_RST
 *  GPIO18 -> RA02_SCK
 *  GPIO19 -> RA02_MISO
 *  GPIO23 -> RA02_MOSI
 *  GPIO26 -> RA02_DIO0
 *
 *  Serial debug/data -> USB bawaan ESP32 (IO1/IO3), dipakai GUI drone_viewer.py
 *
 *  PARAMETER RADIO harus SAMA dengan sisi drone (STM32F401):
 *  433 MHz, SF7, BW 125 kHz, CR 4/5, TxPower 17 dBm
 * ==========================================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <string.h>
#include <stdio.h>

/* ---------------- Definisi pin LoRa (sesuai skematik) ---------------- */
/* SCK=18, MISO=19, MOSI=23 adalah default VSPI ESP32, sehingga library
 * LoRa cukup memakai SPI default tanpa perlu SPI.begin() custom. */
#define LORA_NSS  5
#define LORA_RST  14
#define LORA_DIO0 26

/* RA-02 umumnya versi 433 MHz. Ganti ke 868E6 / 915E6 jika modulmu beda. */
#define LORA_FREQUENCY 433E6

/* ========================= PROTOKOL PAKET BINER ========================= */
/* HARUS identik dengan definisi di LoraRx.ino (sisi drone). */
#define UPLINK_MAGIC   0xA5
#define DOWNLINK_MAGIC 0x5A
#define CONFIG_MAGIC   0xC3

#pragma pack(push, 1)
struct UplinkPacket {
  uint8_t magic;   // == UPLINK_MAGIC
  uint8_t r, t, y, p;
  uint8_t armed;   // 0 = DISARM, 1 = ARM
};

struct DownlinkPacket {
  uint8_t  magic;   // == DOWNLINK_MAGIC
  int16_t  ax, ay, az;   // m/s^2  x100
  int16_t  gx, gy, gz;   // deg/s  x100
  uint16_t press;        // hPa    x10
  int16_t  alt;          // meter  x100
  int16_t  roll, pitch;  // derajat x100, complementary filter onboard
  int16_t  uRoll, uPitch, uYaw; // koreksi SMC PWM x100
};

struct ConfigPacket {
  uint8_t magic;
  float k1;
  float k2;
  float eps;
  float forceToPwm;
  float deltaMaxPwm;
};
#pragma pack(pop)

/* ------------------------- konfigurasi joystick --------------------------- */
const int PIN_LEFT_X  = 32;   // ROLL
const int PIN_LEFT_Y  = 33;   // THROTTLE
const int PIN_RIGHT_X = 34;   // YAW
const int PIN_RIGHT_Y = 35;   // PITCH

const int   CALIBRATION_SAMPLES   = 400;   // sampel saat startup (beberapa ratus)
const float JOYSTICK_DEADZONE     = 0.07f; // 7% dari rentang penuh (5-10% disarankan)
const float JOYSTICK_FILTER_ALPHA = 0.35f; // 0..1, makin kecil makin halus

/* ------------------------- konfigurasi siklus ping-pong ------------------- */
/* Nilai ini dihitung dari AIRTIME AKTUAL paket biner pada radio SF7/BW125k/
 * CR4/5 (preamble 8, CRC on, explicit header):
 *   uplink (5 byte)   airtime ~31 ms
 *   downlink (27 byte) airtime ~62 ms
 * REPLY_TIMEOUT_MS diberi margin ekstra utk waktu proses drone (baca sensor
 * I2C, overhead SPI LoRa) + jitter. CYCLE_PERIOD_MS >= uplink + timeout.
 * PENTING: jika nilai timeout < airtime downlink, balasan drone akan SELALU
 * terlewat (baca 0 di GUI) walau modul & sensor bekerja normal. */
const unsigned long CYCLE_PERIOD_MS  = 145;  // total siklus (~6.9 Hz), termasuk telemetri SMC
const unsigned long REPLY_TIMEOUT_MS = 100;  // margin untuk paket downlink 27 byte + proses STM32

// Balikkan arah sumbu melalui software jika orientasi modul terbalik.
const bool INVERT_LEFT_X  = true;  // ROLL     (kiri = ROLL kiri)
const bool INVERT_LEFT_Y  = true;  // THROTTLE (atas = naik)
const bool INVERT_RIGHT_X = true;  // YAW      (kanan = ke kanan)
const bool INVERT_RIGHT_Y = true;  // PITCH    (atas = maju)

// Jika sumbu X dan Y joystick tertukar (VRX/VRY terbalik) sehingga
// ROLL dan THROTTLE kebalik, set SWAP_LEFT_X_Y = true.
// SWAP_RIGHT_X_Y untuk joystick kanan (YAW/PITCH).
// Wiring mengikuti deklarasi PIN_*: stick kiri Y=throttle dan stick kanan Y=pitch.
const bool SWAP_LEFT_X_Y  = false;
const bool SWAP_RIGHT_X_Y = false;

// -------------------------------- state -----------------------------------
const int N_CH = 4;
int  PINS[N_CH];
bool INVERT[N_CH];

int   center[N_CH];
float filt[N_CH];

bool loraOK = false;   // status deteksi modul LoRa saat init
bool armedState = false; // status ARM/DISARM dari GUI
bool smcConfigPending = false;
ConfigPacket pendingSmcConfig;

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
    filt[ch] = (float)center[ch];
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
  long dead = (long)(JOYSTICK_DEADZONE * 4096.0f);
  if (abs(delta) <= dead) return 128;

  long half = center[ch] > (4095 - center[ch]) ? center[ch] : (4095 - center[ch]);
  float n = (float)delta / (float)half;
  if (n > 1.0f) n = 1.0f;
  if (n < -1.0f) n = -1.0f;
  return (int)(128 + 128.0f * n);
}

// THROTTLE: 0 = bawah, 255 = atas
int throttleToCmd(int value)
{
  if (value < 0) value = 0;
  if (value > 4095) value = 4095;
  return (int)(value * 255.0f / 4095.0f);
}

// ----------------------------- proses joystick ----------------------------
// Baca semua channel: invert -> filter EMA -> normalisasi.
// Hasil disimpan di cmd[]: ROLL, THROTTLE, YAW, PITCH (masing-masing 0..255)
void processJoystick(int cmd[N_CH])
{
  for (int ch = 0; ch < N_CH; ch++) {
    int v = applyInvert(ch, readRaw(ch));
    filt[ch] += JOYSTICK_FILTER_ALPHA * (v - filt[ch]);
    if (ch == 1) {   // LEFT_Y = THROTTLE
      cmd[ch] = throttleToCmd((int)filt[ch]);
    } else {
      cmd[ch] = centerAxisToCmd(ch, (int)filt[ch]);
    }
  }
}

// --------------------------------- setup ----------------------------------
void setup()
{
  PINS[0] = SWAP_LEFT_X_Y  ? PIN_LEFT_Y  : PIN_LEFT_X;    // ROLL
  PINS[1] = SWAP_LEFT_X_Y  ? PIN_LEFT_X  : PIN_LEFT_Y;    // THROTTLE
  PINS[2] = SWAP_RIGHT_X_Y ? PIN_RIGHT_Y : PIN_RIGHT_X;   // YAW
  PINS[3] = SWAP_RIGHT_X_Y ? PIN_RIGHT_X : PIN_RIGHT_Y;   // PITCH
  INVERT[0] = INVERT_LEFT_X;
  INVERT[1] = INVERT_LEFT_Y;
  INVERT[2] = INVERT_RIGHT_X;
  INVERT[3] = INVERT_RIGHT_Y;

  Serial.begin(115200);
  delay(500);
  analogReadResolution(12);   // ADC 12-bit: 0..4095
  for (int ch = 0; ch < N_CH; ch++) {
    analogSetPinAttenuation(PINS[ch], ADC_11db);   // rentang ~0..3.3V
  }

  Serial.println();
  Serial.println("=== LORA RA-02 REMOTE (MASTER) + JOYSTICK - ESP32-WROOM-32 ===");
  Serial.println("Pin: NSS=5, RST=14, SCK=18, MISO=19, MOSI=23, DIO0=26");
  Serial.println("---------------------------------------------------------------");

  /* Kalibrasi titik tengah joystick */
  Serial.println("LETAKKAN KEDUA STICK DI POSISI TENGAH.");
  for (int i = 3; i > 0; i--) {
    Serial.print("Kalibrasi mulai dalam ");
    Serial.print(i);
    Serial.println(" detik.");
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

    /* Parameter radio harus SAMA dengan sisi drone */
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setTxPower(17);

    Serial.println("LoRa siap: mode PING-PONG (kirim joystick, tunggu telemetri).");
  }

  Serial.println("---------------------------------------------------------------");
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

  unsigned long cycleStart = millis();

  /* ---------- [0] Baca perintah ARM / DISARM dari GUI Serial USB ---------- */
  while (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("ARM")) {
      armedState = true;
    } else if (input.equalsIgnoreCase("DISARM") || input.equalsIgnoreCase("STOP")) {
      armedState = false;
    } else if (input.startsWith("SMC ")) {
      float k1, k2, eps, forceToPwm, deltaMaxPwm;
      int parsed = sscanf(input.c_str(), "SMC %f %f %f %f %f",
                          &k1, &k2, &eps, &forceToPwm, &deltaMaxPwm);
      if (parsed == 5 && k1 > 0.0f && k2 > 0.0f && eps > 0.1f &&
          forceToPwm > 0.0f && deltaMaxPwm > 0.0f) {
        pendingSmcConfig.magic = CONFIG_MAGIC;
        pendingSmcConfig.k1 = k1;
        pendingSmcConfig.k2 = k2;
        pendingSmcConfig.eps = eps;
        pendingSmcConfig.forceToPwm = forceToPwm;
        pendingSmcConfig.deltaMaxPwm = deltaMaxPwm;
        smcConfigPending = true;
        Serial.println("[SMC] Parameter diterima, dikirim ke drone.");
      } else {
        Serial.println("[SMC] Format: SMC k1 k2 eps forceToPwm deltaMaxPwm");
      }
    }
  }

  if (smcConfigPending) {
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&pendingSmcConfig, sizeof(ConfigPacket));
    LoRa.endPacket();
    smcConfigPending = false;
    LoRa.receive();
    delay(20);
  }

  /* ---------- [1] Baca joystick & kirim ke drone (uplink, biner) ---------- */
  int cmd[N_CH];
  processJoystick(cmd);

  UplinkPacket up;
  up.magic = UPLINK_MAGIC;
  up.r = (uint8_t)cmd[1];
  up.t = (uint8_t)cmd[2];
  up.y = (uint8_t)cmd[3];
  up.p = (uint8_t)cmd[0];
  up.armed = armedState ? 1 : 0;

  LoRa.beginPacket();
  LoRa.write((uint8_t *)&up, sizeof(UplinkPacket));
  LoRa.endPacket();

  /* Teruskan data joystick ke GUI via USB (format dikenali drone_viewer.py) */
  Serial.print("[TX] R:"); Serial.print(cmd[1]);
  Serial.print(" T:");     Serial.print(cmd[2]);
  Serial.print(" Y:");     Serial.print(cmd[3]);
  Serial.print(" P:");     Serial.println(cmd[0]);

  /* ---------- [2] Beralih ke RX, tunggu balasan telemetri drone ---------- */
  LoRa.receive();

  bool gotReply = false;
  unsigned long waitStart = millis();
  while (!gotReply && (millis() - waitStart < REPLY_TIMEOUT_MS))
  {
    int packetSize = LoRa.parsePacket();
    if (packetSize == sizeof(DownlinkPacket))
    {
      uint8_t buf[sizeof(DownlinkPacket)];
      for (uint8_t i = 0; i < sizeof(DownlinkPacket) && LoRa.available(); i++) {
        buf[i] = (uint8_t)LoRa.read();
      }

      DownlinkPacket down;
      memcpy(&down, buf, sizeof(DownlinkPacket));

      if (down.magic == DOWNLINK_MAGIC)
      {
        gotReply = true;

        float ax = down.ax / 100.0f, ay = down.ay / 100.0f, az = down.az / 100.0f;
        float gx = down.gx / 100.0f, gy = down.gy / 100.0f, gz = down.gz / 100.0f;
        float press = down.press / 10.0f;
        float alt = down.alt / 100.0f;

        /* Teruskan telemetri ke GUI via USB (format [IMU]/[BMP], tanpa suhu) */
        Serial.print("[IMU] AX:"); Serial.print(ax, 2);
        Serial.print(" AY:");      Serial.print(ay, 2);
        Serial.print(" AZ:");      Serial.print(az, 2);
        Serial.print(" GX:");      Serial.print(gx, 2);
        Serial.print(" GY:");      Serial.print(gy, 2);
        Serial.print(" GZ:");      Serial.println(gz, 2);

        Serial.print("[BMP] P:"); Serial.print(press, 2);
        Serial.print(" A:");      Serial.println(alt, 2);

        Serial.print("[SMC] ROLL:"); Serial.print(down.roll / 100.0f, 2);
        Serial.print(" PITCH:"); Serial.print(down.pitch / 100.0f, 2);
        Serial.print(" UR:"); Serial.print(down.uRoll / 100.0f, 2);
        Serial.print(" UP:"); Serial.print(down.uPitch / 100.0f, 2);
        Serial.print(" UY:"); Serial.println(down.uYaw / 100.0f, 2);
      }
    }
    else if (packetSize > 0)
    {
      /* Ukuran paket tidak sesuai protokol biner (mis. noise/tabrakan):
         buang isinya dan terus menunggu sampai timeout habis. */
      while (LoRa.available()) { LoRa.read(); }
    }
  }

  if (!gotReply)
  {
    /* Tidak ada balasan valid dalam jendela waktu: link ke drone kemungkinan
       putus/lemah. GUI cukup tidak menerima update [IMU]/[BMP] pada siklus
       ini; siklus berikutnya akan mencoba lagi. */
  }

  /* ---------- [3] Jaga total periode siklus tetap ~8 Hz ---------- */
  unsigned long elapsed = millis() - cycleStart;
  if (elapsed < CYCLE_PERIOD_MS) {
    delay(CYCLE_PERIOD_MS - elapsed);
  }
}
