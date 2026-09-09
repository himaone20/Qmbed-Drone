/* ==========================================================================
 * RADIO.CPP — Implementation LoRa RA-02 (SPI2), Telemetry & TaskLoRa_Control
 * ==========================================================================
 */

#include "radio.h"
#include "motors.h"
#include "control.h"

SPIClass SPI_2(LORA_MOSI_PIN, LORA_MISO_PIN, LORA_SCK_PIN); // PB15, PB14, PB13

// Global buffer perintah terakhir dari remote
float    gTargetRollDeg    = 0.0f;
float    gTargetPitchDeg   = 0.0f;
float    gTargetYawRateDps = 0.0f;
uint16_t gTargetThrottlePwm = 128;
bool     gArmedCmd         = false;

static unsigned long lastLinkMs = 0;
static bool linkOK = false;

bool radio_init()
{
  LoRa.setPins(LORA_NSS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  LoRa.setSPI(SPI_2);

  if (!LoRa.begin(LORA_FREQUENCY))
  {
    return false;
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.receive();

  return true;
}

void TaskLoRa_Control(void *pvParameters)
{
  (void) pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TASK_LORA_PERIOD_MS);

  for (;;)
  {
    int packetSize = LoRa.parsePacket();

    if (packetSize == sizeof(UplinkPacket))
    {
      uint8_t buf[sizeof(UplinkPacket)];
      for (uint8_t i = 0; i < sizeof(UplinkPacket) && LoRa.available(); i++) {
        buf[i] = (uint8_t)LoRa.read();
      }

      UplinkPacket up;
      memcpy(&up, buf, sizeof(UplinkPacket));

      if (up.magic == UPLINK_MAGIC)
      {
        lastLinkMs = millis();
        if (!linkOK) {
          linkOK = true;
          Serial.println(">>> Link LoRa Remote Aktif <<<");
        }

        // Simpan data perintah stik; throttle adalah command self-centering 0..255.
        gTargetRollDeg     = (float)up.targetRoll / 100.0f;
        gTargetPitchDeg    = (float)up.targetPitch / 100.0f;
        gTargetYawRateDps  = (float)up.targetYaw / 100.0f;
        gTargetThrottlePwm = up.targetThrottle;
        gArmedCmd          = (up.armed == 1);

        /* Ambil snapshot data sensor terbaru (thread-safe, inisialisasi aman anti-glitch) */
        SensorData snap = {};
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          snap = gSensorData;
          xSemaphoreGive(sensorMutex);
        }

        /* Balas ke remote & GUI drone_viewer: DownlinkPacket biner (30 byte) */
        DownlinkPacket down;
        memset(&down, 0, sizeof(DownlinkPacket));
        down.magic = DOWNLINK_MAGIC;
        down.ax = (int16_t)(snap.ax * 100.0f);
        down.ay = (int16_t)(snap.ay * 100.0f);
        down.az = (int16_t)(snap.az * 100.0f);
        down.gx = (int16_t)(snap.gx * 100.0f);
        down.gy = (int16_t)(snap.gy * 100.0f);
        down.gz = (int16_t)(snap.gz * 100.0f);
        down.press = (uint16_t)(snap.press * 10.0f);
        down.alt = (int16_t)(snap.alt * 100.0f);
        down.roll = (int16_t)(snap.roll * 100.0f);
        down.pitch = (int16_t)(snap.pitch * 100.0f);
        down.yaw = (int16_t)(snap.yaw * 100.0f);
        down.uRoll = (int16_t)(lastURoll * 100.0f);   // Koreksi PID Roll aktual
        down.uPitch = (int16_t)(lastUPitch * 100.0f); // Koreksi PID Pitch aktual
        down.uYaw = (int16_t)(lastUYaw * 100.0f);     // Koreksi PID Yaw aktual
        down.vbat = (uint16_t)(snap.vbat * 100.0f);

        // Encode status flags: bit0=gyroCalibValid, bit1=bmiOK, bit2=bmpOK,
        //   bit3=vbatOK, bit4-5=battStage (0..3), bit6-7=failsafeStage (0..3)
        uint8_t flags = 0;
        if (snap.gyroCalibValid) flags |= 0x01;
        if (snap.bmiOK)          flags |= 0x02;
        if (snap.bmpOK)          flags |= 0x04;
        if (snap.vbatOK)         flags |= 0x08;
        flags |= (snap.battStage & 0x03) << 4;
        down.flags = flags;

        LoRa.beginPacket();
        LoRa.write((uint8_t *)&down, sizeof(DownlinkPacket));
        LoRa.endPacket();

        LoRa.receive(); // Kembali siaga menunggu paket berikutnya

        /* Debug via Serial USART1 */
        Serial.print("[RX CMD] R:"); Serial.print(gTargetRollDeg, 1);
        Serial.print("° P:");        Serial.print(gTargetPitchDeg, 1);
        Serial.print("° Y:");        Serial.print(gTargetYawRateDps, 1);
        Serial.print("°/s T:");      Serial.print(gTargetThrottlePwm);
        Serial.print("cmd ARM:");    Serial.print(gArmedCmd ? "1" : "0");
        if (gArmedCmd) {
          Serial.print(" | M1:");    Serial.print(getMotorPWM(0));
          Serial.print(" M2:");      Serial.print(getMotorPWM(1));
          Serial.print(" M3:");      Serial.print(getMotorPWM(2));
          Serial.print(" M4:");      Serial.print(getMotorPWM(3));
          Serial.print(" | uR:");    Serial.print(lastURoll, 1);
          Serial.print(" uP:");      Serial.print(lastUPitch, 1);
          Serial.print(" uY:");      Serial.print(lastUYaw, 1);
        }
        Serial.print(" | Roll:");    Serial.print(snap.roll, 1);
        Serial.print("° Pitch:");    Serial.print(snap.pitch, 1);
        Serial.print("° VBat:");     Serial.print(snap.vbat, 2);
        Serial.println("V");
      }
      else
      {
        LoRa.receive();
        Serial.println("[RX] Magic byte uplink tidak cocok, paket diabaikan.");
      }
    }
    else if (packetSize == sizeof(ConfigPacket))
    {
      uint8_t cfgBuf[sizeof(ConfigPacket)];
      for (uint8_t i = 0; i < sizeof(ConfigPacket) && LoRa.available(); i++) {
        cfgBuf[i] = (uint8_t)LoRa.read();
      }
      ConfigPacket cfg;
      memcpy(&cfg, cfgBuf, sizeof(ConfigPacket));
      if (cfg.magic == CONFIG_MAGIC) {
        PidParams newParams;
        newParams.angleKp        = cfg.angleKp;
        newParams.angleKi        = cfg.angleKi;
        newParams.angleKd        = cfg.angleKd;
        newParams.rateKp         = cfg.rateKp;
        newParams.rateKi         = cfg.rateKi;
        newParams.rateKd         = cfg.rateKd;
        newParams.yawKp          = cfg.yawKp;
        newParams.yawKi          = cfg.yawKi;
        newParams.yawKd          = cfg.yawKd;
        newParams.maxAngle       = cfg.maxAngle;
        newParams.maxYawRate     = cfg.maxYawRate;
        newParams.maxDeltaPwm    = cfg.maxDeltaPwm;
        newParams.escMinPwm      = (cfg.escMinPwm >= 900.0f && cfg.escMinPwm <= 1400.0f) ? cfg.escMinPwm : (float)ESC_MIN_US;
        newParams.escArmSpinPwm  = (cfg.escArmSpinPwm >= 1000.0f && cfg.escArmSpinPwm <= 1600.0f) ? cfg.escArmSpinPwm : (float)ESC_ARM_SPIN_US;
        newParams.escMaxPwm      = (cfg.escMaxPwm >= 1100.0f && cfg.escMaxPwm <= 2200.0f) ? cfg.escMaxPwm : (float)ESC_MAX_US;

        setPidParams(newParams);
        setEscPwmLimits((int)newParams.escMinPwm, (int)newParams.escArmSpinPwm, (int)newParams.escMaxPwm);
        setHoverThrottlePwm(cfg.hoverThrottlePwm);
        Serial.print("[PID] Parameter PID, Hover Throttle & Limit ESC PWM Baru Diterapkan. Hover:");
        Serial.println(gHoverThrottlePwm, 1);
      }
      LoRa.receive();
    }
    else if (packetSize > 0)
    {
      while (LoRa.available()) { LoRa.read(); }
      LoRa.receive();
    }

    /* Link timeout check */
    if (linkOK && (millis() - lastLinkMs > 3000))
    {
      linkOK = false;
      Serial.println("[RADIO] Link LoRa terputus (> 3s tidak ada paket).");
    }

    vTaskDelayUntil(&lastWakeTime, period);
  }
}
