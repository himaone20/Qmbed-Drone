#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================= DRONE ICON =================
void drawDrone(int cx, int cy, bool spinFrame) {
  // Body drone
  display.fillRoundRect(cx - 6, cy - 3, 12, 6, 2, SH110X_WHITE);

  // Lengan drone (4 arah diagonal)
  display.drawLine(cx - 6, cy - 3, cx - 14, cy - 11, SH110X_WHITE);
  display.drawLine(cx + 6, cy - 3, cx + 14, cy - 11, SH110X_WHITE);
  display.drawLine(cx - 6, cy + 3, cx - 14, cy + 11, SH110X_WHITE);
  display.drawLine(cx + 6, cy + 3, cx + 14, cy + 11, SH110X_WHITE);

  // Baling-baling (efek spin blur: gantian bentuk + dan x biar kelihatan muter)
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

// ================= ANIMASI RADAR PING =================
void radarPing(int cx, int cy, int maxR) {
  for (int r = 2; r <= maxR; r += 4) {
    display.clearDisplay();
    display.drawCircle(cx, cy, r, SH110X_WHITE);
    if (r > 8) display.drawCircle(cx, cy, r - 8, SH110X_WHITE); // ring kedua
    display.display();
    delay(30);
  }
}

// ================= ANIMASI DRONE TERBANG MASUK =================
void droneFlyIn() {
  bool spin = false;
  for (int x = -20; x <= 64; x += 4) {
    display.clearDisplay();
    drawDrone(x, 32, spin);
    display.display();
    spin = !spin;
    delay(35);
  }
  // Hover di tengah sambil baling-baling tetap berputar
  for (int i = 0; i < 8; i++) {
    display.clearDisplay();
    drawDrone(64, 32, spin);
    display.display();
    spin = !spin;
    delay(60);
  }
}

// ================= ANIMASI ZOOM LOGO "QMBED" =================
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
    delay(220);
  }
  delay(400);
}

// ================= LOADING BAR =================
void loadingBar() {
  int barX = 14, barY = 48, barW = 100, barH = 8;

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(28, 12);
  display.println("QMBED");
  display.drawRect(barX, barY, barW, barH, SH110X_WHITE);
  display.display();

  for (int p = 0; p <= 100; p += 4) {
    int fillW = map(p, 0, 100, 0, barW - 2);
    display.fillRect(barX + 1, barY + 1, barW - 2, barH - 2, SH110X_BLACK); // reset isi bar
    display.fillRect(barX + 1, barY + 1, fillW, barH - 2, SH110X_WHITE);    // isi progress

    // Update teks persen
    display.fillRect(48, 30, 32, 10, SH110X_BLACK);
    display.setTextSize(1);
    display.setCursor(48, 30);
    display.print(p);
    display.print("%");

    display.display();
    delay(35);
  }
  delay(300);
}

// ================= FLASH "READY" =================
void readyFlash() {
  for (int i = 0; i < 3; i++) {
    display.invertDisplay(true);
    delay(80);
    display.invertDisplay(false);
    delay(80);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  if (!display.begin(SCREEN_ADDRESS, true)) {
    Serial.println(F("OLED gagal diinisialisasi"));
    while (true) delay(1000);
  }

  display.clearDisplay();
  display.display();
  delay(200);

  // ---- SEQUENCE ANIMASI START SCREEN ----
  radarPing(64, 32, 40);   // efek radar ping
  radarPing(64, 32, 40);   // ping kedua
  droneFlyIn();             // drone terbang masuk + baling-baling spin
  zoomText();                // logo "QMBED" zoom in
  loadingBar();               // loading bar + persen
  readyFlash();                 // flash siap

  // ---- MENU UTAMA: HELLO WORLD ----
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 25);
  display.println("Hello World");
  display.display();

  Serial.println("Boot animation selesai, masuk Hello World");
}

void loop() {
  // Kosong - halaman utama statis
}