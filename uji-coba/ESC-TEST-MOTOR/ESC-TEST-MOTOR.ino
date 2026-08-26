/* ==========================================================================
 * ESC-TEST-MOTOR.ino — Tes ESC / Motor via Serial Monitor
 * --------------------------------------------------------------------------
 * Board  : STM32F401RCT6
 * ESC pin: PB6  (unidirectional, 1000-2000 us)
 * LED    : PC4 (active LOW)
 * Serial : PA9 (TX) / PA10 (RX) — 115200 baud
 *
 * PERINTAH SERIAL:
 *   arm       — Arm ESC (kirim 1000 us selama 5 detik)
 *   stop / 0  — Matikan motor (1000 us)
 *   1  -  9   — Throttle 10 % - 90 %  (1100 - 1900 us)
 *   max       — Full throttle (2000 us)
 *   raw <v>   — Set PWM manual (1000 - 2000)
 *   status    — Tampilkan status ESC saat ini
 *   help      — Tampilkan menu bantuan
 *
 * !!! JANGAN PASANG PROPELLER !!!
 * ==========================================================================
 */

#include <Servo.h>  

/* ── Pin Definitions ─────────────────────────────────────────────────── */
#define ESC_PIN     PB9
#define LED_PIN     PC4
#define SERIAL_TX   PA9
#define SERIAL_RX   PA10

/* ── ESC Timing ──────────────────────────────────────────────────────── */
#define ESC_MIN_US          1000
#define ESC_MAX_US          2000
#define ESC_CENTER_US       1500
#define ARM_TIME_MS         5000
#define IDLE_TIMEOUT_MS     3000

/* ── Global State ────────────────────────────────────────────────────── */
Servo esc;
bool   armed       = false;
int    currentUs   = ESC_MIN_US;
unsigned long lastInputMs = 0;

/* ── Helper: kirim PWM + update status ──────────────────────────────── */
void setPWM(int us)
{
  if (us < ESC_MIN_US) us = ESC_MIN_US;
  if (us > ESC_MAX_US) us = ESC_MAX_US;

  currentUs = us;
  esc.writeMicroseconds(us);
  lastInputMs = millis();

  int throttlePct = map(us, ESC_MIN_US, ESC_MAX_US, 0, 100);

  Serial.print("[PWM] ");
  Serial.print(us);
  Serial.print(" us | Throttle: ");
  Serial.print(throttlePct);
  Serial.print("% | ");

  if (us == ESC_MIN_US)
    Serial.println("STOPPED");
  else
    Serial.println("RUNNING");
}

/* ── Helper: tampilkan status ──────────────────────────────────────── */
void showStatus()
{
  int throttlePct = map(currentUs, ESC_MIN_US, ESC_MAX_US, 0, 100);
  unsigned long uptime = millis() / 1000;

  Serial.println();
  Serial.println("=== ESC STATUS ===");
  Serial.print("  Armed    : ");
  Serial.println(armed ? "YES" : "NO");
  Serial.print("  PWM      : ");
  Serial.print(currentUs);
  Serial.println(" us");
  Serial.print("  Throttle : ");
  Serial.print(throttlePct);
  Serial.println("%");
  Serial.print("  State    : ");
  Serial.println(currentUs == ESC_MIN_US ? "STOPPED" : "RUNNING");
  Serial.print("  Uptime   : ");
  Serial.print(uptime);
  Serial.println(" s");
  Serial.println("==================");
  Serial.println();
}

/* ── Helper: tampilkan menu ───────────────────────────────────────── */
void showHelp()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("  ESC-TEST-MOTOR  —  Serial Command");
  Serial.println("========================================");
  Serial.println("  arm       Arm ESC (5 detik)");
  Serial.println("  stop / 0  Matikan motor (1000 us)");
  Serial.println("  1  -  9   Throttle 10% - 90%");
  Serial.println("  max       Full throttle (2000 us)");
  Serial.println("  raw <v>   Set PWM manual 1000-2000");
  Serial.println("  status    Tampilkan status ESC");
  Serial.println("  help      Menu bantuan ini");
  Serial.println("========================================");
  Serial.println();
}

/* ── Setup ─────────────────────────────────────────────────────────── */
void setup()
{
  Serial.setTx(SERIAL_TX);
  Serial.setRx(SERIAL_RX);
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);   // LED ON = standby

  esc.attach(ESC_PIN);
  esc.writeMicroseconds(ESC_MIN_US);

  Serial.println();
  Serial.println("==============================");
  Serial.println("  ESC-TEST-MOTOR — READY");
  Serial.println("  Ketik 'help' untuk bantuan");
  Serial.println("==============================");
  Serial.println();

  showHelp();
}

/* ── Loop ──────────────────────────────────────────────────────────── */
void loop()
{
  /* ── Baca input serial ── */
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    /* --- arm --- */
    if (input == "arm")
    {
      if (armed)
      {
        Serial.println("[INFO] ESC sudah armed.");
        return;
      }

      Serial.println("[ARM] Mulai arming...");
      esc.writeMicroseconds(ESC_MIN_US);
      digitalWrite(LED_PIN, LOW);   // LED OFF = armed

      for (int i = 5; i > 0; i--)
      {
        Serial.print("[ARM] ");
        Serial.print(i);
        Serial.println(" detik...");
        delay(1000);
      }

      armed = true;
      currentUs = ESC_MIN_US;
      lastInputMs = millis();
      Serial.println("[ARM] ARMED! Ketik angka 1-9, 'max', atau 'stop'.");
      Serial.println();
      return;
    }

    /* --- help --- */
    if (input == "help")
    {
      showHelp();
      return;
    }

    /* --- status --- */
    if (input == "status")
    {
      showStatus();
      return;
    }

    /* --- stop / 0 --- */
    if (input == "stop" || input == "0")
    {
      if (!armed)
      {
        Serial.println("[WARN] ESC belum armed! Ketik 'arm' dulu.");
        return;
      }
      setPWM(ESC_MIN_US);
      return;
    }

    /* --- max --- */
    if (input == "max")
    {
      if (!armed)
      {
        Serial.println("[WARN] ESC belum armed! Ketik 'arm' dulu.");
        return;
      }
      Serial.println("[WARN] FULL THROTTLE — pastikan motor aman!");
      setPWM(ESC_MAX_US);
      return;
    }

    /* --- raw <value> --- */
    if (input.startsWith("raw "))
    {
      if (!armed)
      {
        Serial.println("[WARN] ESC belum armed! Ketik 'arm' dulu.");
        return;
      }
      int val = input.substring(4).toInt();
      if (val < ESC_MIN_US || val > ESC_MAX_US)
      {
        Serial.println("[ERR] Nilai harus 1000 - 2000.");
        return;
      }
      setPWM(val);
      return;
    }

    /* --- angka 1-9 → throttle 10%-90% --- */
    if (input.length() == 1 && input[0] >= '1' && input[0] <= '9')
    {
      if (!armed)
      {
        Serial.println("[WARN] ESC belum armed! Ketik 'arm' dulu.");
        return;
      }
      int digit = input[0] - '0';
      int us = map(digit, 1, 9, 1100, 1900);
      setPWM(us);
      return;
    }

    /* --- input tidak dikenali --- */
    Serial.print("[ERR] Perintah tidak dikenali: ");
    Serial.println(input);
    Serial.println("Ketik 'help' untuk daftar perintah.");
    Serial.println();
  }

  /* ── Safety: auto-off setelah idle ── */
  if (armed && currentUs > ESC_MIN_US)
  {
    if (millis() - lastInputMs > IDLE_TIMEOUT_MS)
    {
      Serial.println("[SAFE] Idle timeout — motor dimatikan.");
      setPWM(ESC_MIN_US);
    }
  }
}
