/* ==========================================================================
 * PROGRAM LORA RA-02 (SX1278) - RECEIVER - STM32F401RCT6 - ARDUINO IDE
 * Sesuai pinout skematik: SPI2 + RST/DIO0 di PORT B
 * Serial biasa di-set manual ke PA9 (TX) / PA10 (RX)
 * ==========================================================================
 * PERSIAPAN:
 * 1. Tools > Board > Generic STM32F4 series -> Board part number: Generic F401RCTx
 * 2. Install library "LoRa" (Sandeep Mistry) via Library Manager
 *
 * KONFIGURASI PIN (sesuai skematik):
 *   Serial (USART1, di-remap manual)
 *     PA9  -> TX0
 *     PA10 -> RX0
 *     Baudrate: 9600
 *
 *   SPI2 (ke modul LoRa RA-02)
 *     PB13 -> RA02_SCK
 *     PB14 -> RA02_MISO
 *     PB15 -> RA02_MOSI
 *     PB12 -> RA02_NSS (CS)
 *     PB1  -> RA02_RST
 *     PB0  -> RA02_DIO0
 * ==========================================================================
 */

#include <SPI.h>
#include <LoRa.h>

/* ---------------- Definisi pin LoRa (sesuai skematik) ---------------- */
#define LORA_NSS   PB12
#define LORA_RST   PB1
#define LORA_DIO0  PB0

/* ---------------- Objek SPI2 ---------------- */
SPIClass SPI_2(PB15, PB14, PB13);   // MOSI, MISO, SCK

/* ---------------- Konfigurasi Frekuensi ---------------- */
/* RA-02 umumnya versi 433 MHz. Ganti ke 868E6 / 915E6 jika modulmu beda. */
#define LORA_FREQUENCY  433E6

void setup()
{
  /* Set pin Serial biasa ke PA9 (TX) / PA10 (RX) SEBELUM Serial.begin() */
  Serial.setTx(PA9);
  Serial.setRx(PA10);
  Serial.begin(9600);
  delay(500);
  Serial.println();
  Serial.println("=== LoRa RA-02 STM32F401RCT6 - RECEIVER (SPI2) ===");

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

  Serial.println("LoRa RA-02 siap menerima data...");
  Serial.println("-------------------------------------------");
}

void loop()
{
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

    Serial.print("RX <- ");
    Serial.print(received);
    Serial.print("  | Ukuran: ");
    Serial.print(packetSize);
    Serial.print(" byte | RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm | SNR: ");
    Serial.println(snr);
  }
}