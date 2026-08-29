/* ==========================================================================
 * PROGRAM LORA RA-02 (SX1278) - REMOTE SIDE (MASTER) - ESP32-WROOM-32
 * Hub tunggal antara laptop (GUI GCS) <-> drone via LoRa RA-02 + Layar OLED
 *
 * FITUR REMOTE:
 *   1) Animasi Loading Screen Startup OLED (Radar -> Drone Fly-In -> Zoom QMBED
 *      -> Calibrate Progress Bar -> Ready Flash).
 *   2) Dual Joystick Display Mode 2 pada OLED (Gimbal Left: YAW/THROTTLE,
 *      Gimbal Right: ROLL/PITCH) lengkap dengan crosshair, reference ring,
 *      knob position, dan nilai numerik R/T/Y/P.
 *   3) Sinkronisasi Kalibrasi Joystick dengan GUI drone_viewer.py:
 *      Menerima perintah "CAL SAMPLE" dan "CAL RESET" via Serial. Sampling
 *      ADC raw dilakukan SEPENUHNYA di ESP32 (bebas deadzone/EMA), si GUI cukup
 *      memicu via "CAL SAMPLE" dan membaca balasan "[CAL] OK CR= CT= CY= CP=".
 *      Nilai joystick di OLED dan di GUI dijamin SAMA PERSIS (netral = 128).
 *   4) Status Kualitas Sinyal LoRa Real-time:
 *      Menghitung sliding-window RSSI dan Packet Loss % dengan klasifikasi:
 *      - AMAN    : RSSI kuat, Loss < 10%
 *      - WASPADA : RSSI menengah (-98 s.d. -110 dBm) atau Loss 10-35%
 *      - BAHAYA  : Sinyal kritis (Loss > 35%, RSSI < -110 dBm, atau link putus)
 *   5) Topologi Half-Duplex Ping-Pong (Uplink joystick 6 byte -> Downlink telemetri 27 byte).
 * ==========================================================================
 * KONFIGURASI PIN:
 *   OLED SH1106G (I2C):
 *     GPIO21 -> SDA
 *     GPIO22 -> SCL
 *   LoRa RA-02 SX1278 (SPI):
 *     GPIO5  -> NSS (CS)
 *     GPIO14 -> RST
 *     GPIO18 -> SCK
 *     GPIO19 -> MISO
 *     GPIO23 -> MOSI
 *     GPIO26 -> DIO0
 *   Joystick Analog (ADC):
 *     GPIO32 -> J1 VRX (ROLL)
 *     GPIO33 -> J1 VRY (THROTTLE)
 *     GPIO34 -> J2 VRX (YAW)
 *     GPIO35 -> J2 VRY (PITCH)
 * ==========================================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <string.h>
#include <stdio.h>

/* ======================== OLED (SH1106G 128x64) ======================== */
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
#define SDA_PIN        21
#define SCL_PIN        22

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledOK = false;

/* ======================== LoRa RA-02 Pin ======================== */
#define LORA_NSS       5
#define LORA_RST       14
#define LORA_DIO0      26
#define LORA_FREQUENCY 433E6

/* ========================= PROTOKOL PAKET BINER ========================= */
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
  float k1, k2, eps, forceToPwm, deltaMaxPwm;
};
#pragma pack(pop)

/* ------------------------- konfigurasi joystick --------------------------- */
const int PIN_LEFT_X  = 34;   // ROLL
const int PIN_LEFT_Y  = 35;   // THROTTLE
const int PIN_RIGHT_X = 32;   // YAW
const int PIN_RIGHT_Y = 33;   // PITCH

const int   CALIBRATION_SAMPLES   = 400;   // sampel saat startup
const float JOYSTICK_DEADZONE     = 0.07f; // 7% rentang
const float JOYSTICK_FILTER_ALPHA = 0.35f; // EMA filter

/* ------------------------- konfigurasi siklus ping-pong ------------------- */
const unsigned long CYCLE_PERIOD_MS  = 145;  // ~6.9 Hz
const unsigned long REPLY_TIMEOUT_MS = 100;  // jendela terima telemetri

const bool INVERT_LEFT_X  = true;
const bool INVERT_LEFT_Y  = true;
const bool INVERT_RIGHT_X = true;
const bool INVERT_RIGHT_Y = true;
const bool SWAP_LEFT_X_Y  = false;
const bool SWAP_RIGHT_X_Y = false;

/* -------------------------------- state ----------------------------------- */
const int N_CH = 4;
int  PINS[N_CH];
bool INVERT[N_CH];
int  center[N_CH];
float filt[N_CH];

bool loraOK = false;
bool armedState = false;
bool smcConfigPending = false;
ConfigPacket pendingSmcConfig;

/* ----------------------- Kalibrasi stick (dipicu GUI) --------------------- */
// Kalibrasi dilakukan SEPENUHNYA di ESP32 pada ADC raw (0-4095), sehingga
// hasilnya bebas deadzone/EMA dan selalu tepat 128 di titik netral.
const int GUI_CAL_SAMPLES = 300;   // sampel ADC saat "CAL SAMPLE"

/* ----------------------- Kualitas Sinyal LoRa ----------------------------- */
#define SIGNAL_WINDOW 20
bool   replyHist[SIGNAL_WINDOW];
int    rssiHist[SIGNAL_WINDOW];
int    signalIdx       = 0;
int    lastRssi        = -120;
float  lastSnr         = 0.0f;
int    consecutiveMiss = 0;

enum SignalStatus { SIG_AMAN, SIG_WASPADA, SIG_BAHAYA };

// ------------------------------ fungsi ADC --------------------------------
int readRaw(int ch) {
  return analogRead(PINS[ch]);
}

void calibrateCenters() {
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

// Kalibrasi ulang saat GUI mengirim "CAL SAMPLE". Ambil GUI_CAL_SAMPLES sampel
// ADC raw per channel, set center[] + reset filter EMA agar tidak ada transient.
// User dianjurkan menahan kedua stick tepat di tengah selama sampling.
void calibrateFromGui() {
  long sum[N_CH] = { 0, 0, 0, 0 };
  for (int i = 0; i < GUI_CAL_SAMPLES; i++) {
    for (int ch = 0; ch < N_CH; ch++) sum[ch] += readRaw(ch);
    delay(3);
  }
  for (int ch = 0; ch < N_CH; ch++) {
    center[ch] = (int)(sum[ch] / GUI_CAL_SAMPLES);
    filt[ch] = (float)center[ch];
  }
  // Label sesuai mapping akhir: R=cmd[1], T=cmd[2], Y=cmd[3], P=cmd[0]
  Serial.print("[CAL] OK CR=");  Serial.print(center[1]);
  Serial.print(" CT=");          Serial.print(center[2]);
  Serial.print(" CY=");          Serial.print(center[3]);
  Serial.print(" CP=");          Serial.println(center[0]);
}

int applyInvert(int ch, int raw) {
  if (INVERT[ch]) return 2 * center[ch] - raw;
  return raw;
}

// ROLL/PITCH/YAW: 128 = tengah, 0 = min, 255 = max
int centerAxisToCmd(int ch, int value) {
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
int throttleToCmd(int value) {
  if (value < 0) value = 0;
  if (value > 4095) value = 4095;
  return (int)(value * 255.0f / 4095.0f);
}

// Baca semua channel: invert -> filter EMA -> normalisasi.
// Hasil disimpan di cmd[]: ROLL, THROTTLE, YAW, PITCH (masing-masing 0..255)
void processJoystick(int cmd[N_CH]) {
  for (int ch = 0; ch < N_CH; ch++) {
    int v = applyInvert(ch, readRaw(ch));
    filt[ch] += JOYSTICK_FILTER_ALPHA * (v - filt[ch]);
    if (ch == 1) { // LEFT_Y = THROTTLE
      cmd[ch] = throttleToCmd((int)filt[ch]);
    } else {
      cmd[ch] = centerAxisToCmd(ch, (int)filt[ch]);
    }
  }
}

/* ==========================================================================
 * ANIMASI LOADING SCREEN OLED
 * ========================================================================== */

void drawDrone(int cx, int cy, bool spinFrame) {
  display.fillRoundRect(cx - 6, cy - 3, 12, 6, 2, SH110X_WHITE);
  display.drawLine(cx - 6, cy - 3, cx - 14, cy - 11, SH110X_WHITE);
  display.drawLine(cx + 6, cy - 3, cx + 14, cy - 11, SH110X_WHITE);
  display.drawLine(cx - 6, cy + 3, cx - 14, cy + 11, SH110X_WHITE);
  display.drawLine(cx + 6, cy + 3, cx + 14, cy + 11, SH110X_WHITE);

  int propR = 4;
  int pos[4][2] = {
    {cx - 14, cy - 11}, {cx + 14, cy - 11},
    {cx - 14, cy + 11}, {cx + 14, cy + 11}
  };
  for (int i = 0; i < 4; i++) {
    int px = pos[i][0];
    int py = pos[i][1];
    if (spinFrame) {
      display.drawLine(px - propR, py, px + propR, py, SH110X_WHITE);
      display.drawLine(px, py - propR, px, py + propR, SH110X_WHITE);
    } else {
      display.drawLine(px - propR, py - propR, px + propR, py + propR, SH110X_WHITE);
      display.drawLine(px - propR, py + propR, px + propR, py - propR, SH110X_WHITE);
    }
  }
}

void radarPing(int cx, int cy, int maxR) {
  for (int r = 2; r <= maxR; r += 4) {
    display.clearDisplay();
    display.drawCircle(cx, cy, r, SH110X_WHITE);
    if (r > 8) display.drawCircle(cx, cy, r - 8, SH110X_WHITE);
    display.display();
    delay(20);
  }
}

void droneFlyIn() {
  bool spin = false;
  for (int x = -20; x <= 64; x += 4) {
    display.clearDisplay();
    drawDrone(x, 32, spin);
    display.display();
    spin = !spin;
    delay(25);
  }
  for (int i = 0; i < 4; i++) {
    display.clearDisplay();
    drawDrone(64, 32, spin);
    display.display();
    spin = !spin;
    delay(45);
  }
}

void zoomText() {
  const char* txt = "QMBED";
  for (int size = 1; size <= 3; size++) {
    display.clearDisplay();
    display.setTextSize(size);
    display.setTextColor(SH110X_WHITE);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
    int x = (SCREEN_WIDTH - w) / 2;
    int y = (SCREEN_HEIGHT - h) / 2;
    display.setCursor(x, y);
    display.println(txt);
    display.display();
    delay(150);
  }
  delay(250);
}

void calibrateWithLoadingBar() {
  int barX = 14, barY = 48, barW = 100, barH = 8;

  display.clearDisplay();

  // 1. Header Brand "QMBED" (Size 2, Center X)
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(34, 6);
  display.print("QMBED");

  // 2. Sub-header "CALIBRATING..." (Size 1, Center X)
  display.setTextSize(1);
  display.setCursor(22, 26);
  display.print("CALIBRATING...");

  // 3. Outline Loading Bar
  display.drawRect(barX, barY, barW, barH, SH110X_WHITE);
  display.display();

  long sum[N_CH] = { 0, 0, 0, 0 };
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    for (int ch = 0; ch < N_CH; ch++) sum[ch] += readRaw(ch);
    delay(2);

    if (i % 16 == 0 || i == CALIBRATION_SAMPLES - 1) {
      int pct = ((long)(i + 1) * 100) / CALIBRATION_SAMPLES;
      int fillW = ((long)pct * (barW - 2)) / 100;

      // Isi progress bar
      display.fillRect(barX + 1, barY + 1, barW - 2, barH - 2, SH110X_BLACK);
      display.fillRect(barX + 1, barY + 1, fillW, barH - 2, SH110X_WHITE);

      // Posisi X persentase dinamis agar selalu di tengah layar
      int pctX = 55; // 2 digit (10% - 99%)
      if (pct < 10) pctX = 58;        // 1 digit (0% - 9%)
      else if (pct >= 100) pctX = 52; // 3 digit (100%)

      // Bersihkan hanya area teks persen
      display.fillRect(44, 36, 40, 10, SH110X_BLACK);
      display.setTextSize(1);
      display.setCursor(pctX, 37);
      display.print(pct);
      display.print("%");

      display.display();
    }
  }

  for (int ch = 0; ch < N_CH; ch++) {
    center[ch] = (int)(sum[ch] / CALIBRATION_SAMPLES);
    filt[ch] = (float)center[ch];
  }
  delay(150);
}

void readyFlash() {
  for (int i = 0; i < 2; i++) {
    display.invertDisplay(true);
    delay(70);
    display.invertDisplay(false);
    delay(70);
  }
}

void drawHudCorners() {
  // Top-Left
  display.drawFastHLine(2, 2, 8, SH110X_WHITE);
  display.drawFastVLine(2, 2, 8, SH110X_WHITE);
  // Top-Right
  display.drawFastHLine(118, 2, 8, SH110X_WHITE);
  display.drawFastVLine(125, 2, 8, SH110X_WHITE);
  // Bottom-Left
  display.drawFastHLine(2, 61, 8, SH110X_WHITE);
  display.drawFastVLine(2, 54, 8, SH110X_WHITE);
  // Bottom-Right
  display.drawFastHLine(118, 61, 8, SH110X_WHITE);
  display.drawFastVLine(125, 54, 8, SH110X_WHITE);
}

void showLoraReadyScreen(bool ok) {
  display.clearDisplay();
  drawHudCorners();

  // Header identitas radio
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(13, 5);
  display.print("LORA RA-02 SX1278");
  display.drawFastHLine(14, 15, 100, SH110X_WHITE);
  display.fillCircle(11, 15, 1, SH110X_WHITE);
  display.fillCircle(116, 15, 1, SH110X_WHITE);

  if (ok) {
    // Ikon Antena & Gelombang Radio Vektor
    display.drawFastVLine(17, 25, 12, SH110X_WHITE);
    display.fillCircle(17, 24, 2, SH110X_WHITE);
    display.drawCircle(17, 24, 6, SH110X_WHITE);
    display.drawCircle(17, 24, 10, SH110X_WHITE);

    // Badge Utama Invert [ LINK OK ]
    display.fillRoundRect(32, 20, 88, 19, 3, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setTextSize(2);
    display.setCursor(35, 23);
    display.print("LORA OK");

    // Divider & Footer Info Frekuensi
    display.setTextColor(SH110X_WHITE);
    display.drawFastHLine(14, 45, 100, SH110X_WHITE);
    display.setTextSize(1);
    display.setCursor(4, 50);
    display.print("433MHz - SF7 - READY");
  } else {
    // Tampilan Warning jika LoRa gagal terdeteksi
    display.drawRoundRect(18, 20, 92, 19, 3, SH110X_WHITE);
    display.setTextSize(2);
    display.setCursor(20, 23);
    display.print("NO RADIO");

    display.drawFastHLine(14, 45, 100, SH110X_WHITE);
    display.setTextSize(1);
    display.setCursor(13, 50);
    display.print("CHECK SPI WIRING!");
  }

  display.display();
  delay(850);
  display.clearDisplay();
  display.display();
}

/* ==========================================================================
 * OLED DASHBOARD: DUAL JOYSTICK + KUALITAS SINYAL LORA
 * ========================================================================== */

int getSignalStrength(int rssi, int lossPct, int consecMiss) {
  if (consecMiss >= 5 || lossPct >= 60 || rssi <= -120) return 0;
  if (rssi >= -82 && lossPct < 5)   return 4;
  if (rssi >= -95 && lossPct < 15)  return 3;
  if (rssi >= -105 && lossPct < 30) return 2;
  return 1;
}

void drawSignalBars(int x, int y, int strength) {
  const int heights[] = { 2, 4, 6, 8 };
  for (int i = 0; i < 4; i++) {
    int bx = x + i * 3;
    int by = y + (8 - heights[i]);
    if (i < strength) {
      display.fillRect(bx, by, 2, heights[i], SH110X_WHITE);
    } else {
      display.drawRect(bx, by, 2, heights[i], SH110X_WHITE);
    }
  }
}

SignalStatus classifySignal(int avgRssi, int lossPct, int consecMiss) {
  if (consecMiss >= 5 || lossPct >= 35 || avgRssi < -110) return SIG_BAHAYA;
  if (lossPct >= 10 || avgRssi < -98) return SIG_WASPADA;
  return SIG_AMAN;
}

void drawJoystickPad(int cx, int cy, int r, uint8_t xVal, uint8_t yVal) {
  // Lingkaran luar gimbal
  display.drawCircle(cx, cy, r, SH110X_WHITE);

  // Crosshair
  display.drawFastHLine(cx - r, cy, 2 * r + 1, SH110X_WHITE);
  display.drawFastVLine(cx, cy - r, 2 * r + 1, SH110X_WHITE);

  // Inner reference ring
  display.drawCircle(cx, cy, r / 2, SH110X_WHITE);

  // Posisi knob (xVal: 0..255, yVal: 0..255)
  // Sumbu Y dibalik agar nilai 255 (naik/maju) berada di sisi atas pad
  int maxOffset = r - 2;
  int kx = cx + (((int)xVal - 128) * maxOffset) / 128;
  int ky = cy - (((int)yVal - 128) * maxOffset) / 128;

  // Garis konektor tengah -> knob
  display.drawLine(cx, cy, kx, ky, SH110X_WHITE);

  // Knob
  display.fillCircle(kx, ky, 2, SH110X_WHITE);
}

void renderDashboard(uint8_t r, uint8_t t, uint8_t y, uint8_t p,
                     int rssi, int lossPct, SignalStatus status, int consecMiss) {
  display.clearDisplay();

  // ---------------- [1] STATUS BAR (y: 0 - 10) ----------------
  int sigBars = getSignalStrength(rssi, lossPct, consecMiss);
  drawSignalBars(1, 1, sigBars);

  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // Label Kualitas Sinyal (blinking saat WASPADA / BAHAYA)
  bool showLabel = true;
  if (status == SIG_WASPADA) showLabel = ((millis() / 400) % 2 == 0);
  if (status == SIG_BAHAYA)  showLabel = ((millis() / 200) % 2 == 0);

  if (showLabel) {
    display.setCursor(15, 1);
    if (consecMiss >= 5) {
      display.print("NO LINK");
    } else {
      switch (status) {
        case SIG_AMAN:    display.print("AMAN");    break;
        case SIG_WASPADA: display.print("WASPADA"); break;
        case SIG_BAHAYA:  display.print("BAHAYA");  break;
      }
    }
  }

  // RSSI / Loss
  if (consecMiss < 5 && rssi > -120) {
    display.setCursor(62, 1);
    display.print(rssi);
    display.print("dB");
  }

  // Badge Status ARM / DISARM
  if (armedState) {
    display.fillRoundRect(104, 0, 23, 9, 2, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(106, 1);
    display.print("ARM");
    display.setTextColor(SH110X_WHITE);
  } else {
    display.drawRoundRect(104, 0, 23, 9, 2, SH110X_WHITE);
    display.setCursor(106, 1);
    display.print("DIS");
  }

  // Garis pemisah status bar
  display.drawFastHLine(0, 10, 128, SH110X_WHITE);

  // ---------------- [2] JOYSTICK PADS (y: 11 - 52) ----------------
  // Mode 2 Standard:
  // Stick Kiri  : X = YAW (y), Y = THROTTLE (t)
  // Stick Kanan : X = ROLL (r), Y = PITCH (p)
  drawJoystickPad(32, 32, 18, y, t);
  drawJoystickPad(96, 32, 18, r, p);

  // Garis pemisah data row
  display.drawFastHLine(0, 53, 128, SH110X_WHITE);

  // ---------------- [3] VALUE READOUT ROW (y: 55 - 63) ----------------
  display.setCursor(1, 55);
  display.print("R:"); display.print(r);
  display.setCursor(33, 55);
  display.print("T:"); display.print(t);
  display.setCursor(65, 55);
  display.print("Y:"); display.print(y);
  display.setCursor(97, 55);
  display.print("P:"); display.print(p);

  display.display();
}

/* ==========================================================================
 * SETUP
 * ========================================================================== */
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
  delay(300);
  analogReadResolution(12);
  for (int ch = 0; ch < N_CH; ch++) {
    analogSetPinAttenuation(PINS[ch], ADC_11db);
  }

  // Inisialisasi I2C & OLED SH1106G
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!display.begin(SCREEN_ADDRESS, true)) {
    Serial.println(F("OLED gagal diinisialisasi"));
    oledOK = false;
  } else {
    oledOK = true;
    display.clearDisplay();
    display.display();
    delay(100);
  }

  Serial.println();
  Serial.println("=== LORA RA-02 REMOTE (MASTER) + JOYSTICK + OLED ===");
  Serial.println("Pin: NSS=5, RST=14, SCK=18, MISO=19, MOSI=23, DIO0=26");
  Serial.println("OLED: SDA=21, SCL=22 (SH1106G 128x64)");
  Serial.println("---------------------------------------------------------------");

  // Jalankan Animasi Boot Loading Screen & Kalibrasi Hardware Stick
  if (oledOK) {
    radarPing(64, 32, 36);
    droneFlyIn();
    zoomText();
    calibrateWithLoadingBar();
    readyFlash();
  } else {
    Serial.println("LETAKKAN KEDUA STICK DI POSISI TENGAH.");
    for (int i = 3; i > 0; i--) {
      Serial.print("Kalibrasi mulai dalam ");
      Serial.print(i);
      Serial.println(" detik.");
      delay(1000);
    }
    calibrateCenters();
  }
  Serial.println("=== KALIBRASI HARDWARE SELESAI ===");
  Serial.println("[CAL] READY");

  // Inisialisasi modul LoRa
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  Serial.print("Menginisialisasi modul LoRa RA-02... ");

  if (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println("GAGAL!");
    loraOK = false;
  }
  else
  {
    Serial.println("BERHASIL!");
    loraOK = true;

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setTxPower(17);

    Serial.println("LoRa siap: mode PING-PONG.");
  }

  // Tampilkan status LoRa bergaya Tactical HUD di OLED
  if (oledOK) {
    showLoraReadyScreen(loraOK);
  }

  // Reset riwayat sinyal
  memset(replyHist, 0, sizeof(replyHist));
  for (int i = 0; i < SIGNAL_WINDOW; i++) rssiHist[i] = -120;

  Serial.println("---------------------------------------------------------------");
}

/* ==========================================================================
 * LOOP
 * ========================================================================== */
void loop()
{
  if (!loraOK)
  {
    if (oledOK) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SH110X_WHITE);
      display.setCursor(10, 15);
      display.print("LORA NOT FOUND");
      display.setCursor(10, 30);
      display.print("Check wiring RA-02");
      display.setCursor(10, 45);
      display.print("Then reset ESP32");
      display.display();
    }
    Serial.println("Modul LoRa belum terdeteksi. Cek wiring lalu reset ESP32.");
    delay(1000);
    return;
  }

  unsigned long cycleStart = millis();

  /* ---------- [0] Baca Perintah dari GUI (Serial USB) ---------- */
  while (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("ARM")) {
      armedState = true;
    } else if (input.equalsIgnoreCase("DISARM") || input.equalsIgnoreCase("STOP")) {
      armedState = false;
    } else if (input.startsWith("CAL ")) {
      String payload = input.substring(4);
      payload.trim();
      if (payload.equalsIgnoreCase("SAMPLE")) {
        // Pause LoRa sementara; sample ADC raw lalu update center[].
        calibrateFromGui();
      } else if (payload.equalsIgnoreCase("RESET")) {
        // Kembalikan ke kalibrasi default dengan satu sampling cepat.
        calibrateFromGui();
      } else {
        Serial.println("[CAL] Format: CAL SAMPLE or CAL RESET");
      }
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

  /* ---------- [1] Baca Joystick (kalibrasi terpusat di ESP32) ---------- */
  // Proses kalibrasi (center[], deadzone, EMA filter) sudah menghasilkan nilai
  // 0-255 yang JAMIN = 128 di titik netral. Tidak perlu offset tambahan dari GUI.
  int cmd[N_CH];
  processJoystick(cmd);

  // Mapping internal (sama persis dengan referensi JoystickTest yang sudah
  // terbukti benar): cmd[1]->R, cmd[2]->T, cmd[3]->Y, cmd[0]->P
  int dispR = cmd[1];
  int dispT = cmd[2];
  int dispY = cmd[3];
  int dispP = cmd[0];

  // Siapkan paket biner uplink
  UplinkPacket up;
  up.magic = UPLINK_MAGIC;
  up.r = (uint8_t)dispR;
  up.t = (uint8_t)dispT;
  up.y = (uint8_t)dispY;
  up.p = (uint8_t)dispP;
  up.armed = armedState ? 1 : 0;

  LoRa.beginPacket();
  LoRa.write((uint8_t *)&up, sizeof(UplinkPacket));
  LoRa.endPacket();

  /* Teruskan data joystick ke GUI via USB */
  Serial.print("[TX] R:"); Serial.print(dispR);
  Serial.print(" T:");     Serial.print(dispT);
  Serial.print(" Y:");     Serial.print(dispY);
  Serial.print(" P:");     Serial.println(dispP);

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
        lastRssi = LoRa.packetRssi();
        lastSnr  = LoRa.packetSnr();

        float ax = down.ax / 100.0f, ay = down.ay / 100.0f, az = down.az / 100.0f;
        float gx = down.gx / 100.0f, gy = down.gy / 100.0f, gz = down.gz / 100.0f;
        float press = down.press / 10.0f;
        float alt = down.alt / 100.0f;

        Serial.print("[IMU] AX:"); Serial.print(ax, 2);
        Serial.print(" AY:");      Serial.print(ay, 2);
        Serial.print(" AZ:");      Serial.print(az, 2);
        Serial.print(" GX:");      Serial.print(gx, 2);
        Serial.print(" GY:");      Serial.print(gy, 2);
        Serial.print(" GZ:");      Serial.println(gz, 2);

        Serial.print("[BMP] P:"); Serial.print(press, 2);
        Serial.print(" A:");      Serial.println(alt, 2);

        Serial.print("[SMC] ROLL:");  Serial.print(down.roll / 100.0f, 2);
        Serial.print(" PITCH:");     Serial.print(down.pitch / 100.0f, 2);
        Serial.print(" UR:");        Serial.print(down.uRoll / 100.0f, 2);
        Serial.print(" UP:");        Serial.print(down.uPitch / 100.0f, 2);
        Serial.print(" UY:");        Serial.println(down.uYaw / 100.0f, 2);
      }
    }
    else if (packetSize > 0)
    {
      while (LoRa.available()) { LoRa.read(); }
    }
  }

  /* ---------- [3] Update Metrik Kualitas Sinyal LoRa ---------- */
  if (gotReply) {
    replyHist[signalIdx] = true;
    rssiHist[signalIdx]  = lastRssi;
    consecutiveMiss = 0;
  } else {
    replyHist[signalIdx] = false;
    rssiHist[signalIdx]  = -120;
    consecutiveMiss++;
  }
  signalIdx = (signalIdx + 1) % SIGNAL_WINDOW;

  int misses = 0;
  int rssiSum = 0;
  int rssiCount = 0;
  for (int i = 0; i < SIGNAL_WINDOW; i++) {
    if (!replyHist[i]) {
      misses++;
    } else {
      rssiSum += rssiHist[i];
      rssiCount++;
    }
  }
  int lossPct = (misses * 100) / SIGNAL_WINDOW;
  int avgRssi = (rssiCount > 0) ? (rssiSum / rssiCount) : -120;
  SignalStatus sigStatus = classifySignal(avgRssi, lossPct, consecutiveMiss);

  /* ---------- [4] Render Layar OLED ---------- */
  if (oledOK) {
    renderDashboard((uint8_t)dispR, (uint8_t)dispT, (uint8_t)dispY, (uint8_t)dispP,
                    avgRssi, lossPct, sigStatus, consecutiveMiss);
  }

  /* ---------- [5] Jaga total periode siklus tetap ~7 Hz ---------- */
  unsigned long elapsed = millis() - cycleStart;
  if (elapsed < CYCLE_PERIOD_MS) {
    delay(CYCLE_PERIOD_MS - elapsed);
  }
}
