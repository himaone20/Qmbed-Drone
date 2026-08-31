/* ==========================================================================
 * RADIO.CPP — Implementation LoRa RA-02 (SPI2), Telemetry & TaskLoRa_Control
 * ==========================================================================
 */

#include "radio.h"

SPIClass SPI_2(LORA_MOSI_PIN, LORA_MISO_PIN, LORA_SCK_PIN); // PB15, PB14, PB13

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
          Serial.println(">>> Link dengan remote aktif <<<");
        }

        /* Update State Motor ESC (ARM / DISARM) */
        lastRollCmd = up.r;
        lastThrottleCmd = up.t;
        lastYawCmd = up.y;
        lastPitchCmd = up.p;
        bool armCmd = (up.armed == 1);
        updateEscFSM(armCmd);

        /* Ambil snapshot data sensor terbaru (thread-safe) */
        SensorData snap;
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          snap = gSensorData;
          xSemaphoreGive(sensorMutex);
        }

        /* Balas ke remote: DownlinkPacket biner (27 byte, termasuk data SMC) */
        DownlinkPacket down;
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
        down.uRoll = (int16_t)(lastURoll * 100.0f);
        down.uPitch = (int16_t)(lastUPitch * 100.0f);
        down.uYaw = (int16_t)(lastUYaw * 100.0f);
        down.vbat = (uint16_t)(snap.vbat * 100.0f);

        LoRa.beginPacket();
        LoRa.write((uint8_t *)&down, sizeof(DownlinkPacket));
        LoRa.endPacket();

        LoRa.receive(); // kembali siaga menunggu paket joystick berikutnya

        /* Debug lokal via USB-TTL (USART1) */
        Serial.print("[RX] R:"); Serial.print(up.r);
        Serial.print(" T:"); Serial.print(up.t);
        Serial.print(" Y:"); Serial.print(up.y);
        Serial.print(" P:"); Serial.print(up.p);
        Serial.print(" ARM:"); Serial.print(up.armed);
        if (escState == ESC_ARMED) {
          Serial.print(" | M1:"); Serial.print(getMotorPWM(0));
          Serial.print(" M2:"); Serial.print(getMotorPWM(1));
          Serial.print(" M3:"); Serial.print(getMotorPWM(2));
          Serial.print(" M4:"); Serial.print(getMotorPWM(3));
        }
        Serial.print(" | AX:"); Serial.print(snap.ax, 2);
        Serial.print(" AY:"); Serial.print(snap.ay, 2);
        Serial.print(" AZ:"); Serial.print(snap.az, 2);
        Serial.print(" | P:"); Serial.print(snap.press, 2);
        Serial.print(" A:"); Serial.print(snap.alt, 2);
        Serial.print(" | V:"); Serial.print(snap.vbat, 2); Serial.println("V");
      }
      else
      {
        LoRa.receive();
        Serial.println("[RX] Magic byte uplink tidak cocok, paket diabaikan.");
      }
    }
    else if (packetSize == sizeof(ConfigPacket))
    {
      uint8_t buf[sizeof(ConfigPacket)];
      for (uint8_t i = 0; i < sizeof(ConfigPacket) && LoRa.available(); i++) {
        buf[i] = (uint8_t)LoRa.read();
      }

      ConfigPacket config;
      memcpy(&config, buf, sizeof(ConfigPacket));
      if (config.magic == CONFIG_MAGIC && config.k1 > 0.0f && config.k2 > 0.0f &&
          config.eps > 0.1f && config.forceToPwm > 0.0f && config.deltaMaxPwm > 0.0f) {
        if (xSemaphoreTake(smcMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          gSmcParams.k1 = config.k1;
          gSmcParams.k2 = config.k2;
          gSmcParams.eps = config.eps;
          gSmcParams.forceToPwm = config.forceToPwm;
          gSmcParams.deltaMaxPwm = config.deltaMaxPwm;
          xSemaphoreGive(smcMutex);
        }
        Serial.print("[SMC] Params k1="); Serial.print(config.k1, 2);
        Serial.print(" k2="); Serial.print(config.k2, 2);
        Serial.print(" eps="); Serial.println(config.eps, 2);
      }
      LoRa.receive();
    }
    else if (packetSize > 0)
    {
      while (LoRa.available()) { LoRa.read(); }
      LoRa.receive();
    }

    /* Update proses arming 5 detik saat timer berjalan */
    if (escState == ESC_ARMING) {
      updateEscFSM(true);
    }

    /* Update LED PC4 (Indikator Arming / Armed) */
    updateLedIndicator();

    /* Link timeout & Failsafe motor */
    if (linkOK && (millis() - lastLinkMs > LINK_TIMEOUT_MS))
    {
      linkOK = false;
      updateEscFSM(false); // FAILSAFE: matikan motor seketika
      Serial.println("! Link terputus: sinyal remote hilang > 1 detik. Motor STOP.");
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_LORA_PERIOD_MS));
  }
}
