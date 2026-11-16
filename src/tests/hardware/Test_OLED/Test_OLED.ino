/*
  TEST OLED 0.96" / 1.3" I2C  -  Line Robot (ESP32-S3)
  -------------------------------------------------
  NO LIBRARY NEEDED: the code drives the OLED directly over Wire, works with both SSD1306 and SH1106 chips.
  Wiring: OLED VCC -> 3V3 | GND -> GND | SCL -> GPIO9 | SDA -> GPIO8
           (note: many OLEDs print the pin order GND-VCC-SCL-SDA, some VCC-GND-SCL-SDA -> read the OLED silkscreen carefully)
  Serial Monitor 115200.

  Code se:
    1. Check whether SDA/SCL have pull-up resistors (must read 1 when idle)
    2. Scan I2C on pins 8/9, if nothing is found try SWAPPED 9/8 (in case SDA-SCL are plugged reversed)
    3. Try initializing as SSD1306, run the test pattern; then try SH1106
  Watch the screen: ALL WHITE -> checkerboard -> horizontal stripes -> border.
  Type 1 = rerun the SSD1306 test | 2 = SH1106 test | s = rescan I2C
*/
#include <Arduino.h>
#include <Wire.h>

int SDA_PIN = 8, SCL_PIN = 9;
uint8_t addr = 0;

bool cmd(uint8_t c) { Wire.beginTransmission(addr); Wire.write(0x00); Wire.write(c); return Wire.endTransmission() == 0; }

bool initOled(bool sh1106) {
  const uint8_t ssd[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x02,
                         0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF};
  const uint8_t sh[]  = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0xAD, 0x8B, 0xA1, 0xC8,
                         0xDA, 0x12, 0x81, 0xCF, 0xD9, 0x22, 0xDB, 0x35, 0x32, 0xA4, 0xA6, 0xAF};
  const uint8_t* s = sh1106 ? sh : ssd; int n = sh1106 ? sizeof(sh) : sizeof(ssd);
  bool ok = true;
  for (int i = 0; i < n; i++) ok &= cmd(s[i]);
  return ok;
}

// draw one page (page = 8 pixel rows) with a byte generator function
void fill(bool sh1106, uint8_t (*pat)(int page, int col)) {
  int off = sh1106 ? 2 : 0;
  for (int p = 0; p < 8; p++) {
    cmd(0xB0 + p); cmd(0x00 + (off & 0x0F)); cmd(0x10 + (off >> 4));
    for (int c = 0; c < 128; c += 16) {
      Wire.beginTransmission(addr); Wire.write(0x40);
      for (int k = 0; k < 16; k++) Wire.write(pat(p, c + k));
      Wire.endTransmission();
    }
  }
}
uint8_t pWhite(int, int) { return 0xFF; }
uint8_t pBlack(int, int) { return 0x00; }
uint8_t pChess(int p, int c) { return ((c / 8 + p) & 1) ? 0xFF : 0x00; }
uint8_t pStripe(int, int) { return 0x55; }
uint8_t pFrame(int p, int c) {
  uint8_t b = 0;
  if (c == 0 || c == 127) b = 0xFF;
  if (p == 0) b |= 0x01;
  if (p == 7) b |= 0x80;
  return b;
}

void runTest(bool sh1106) {
  if (!addr) { Serial.println("OLED not found on I2C -> cannot test"); return; }
  Serial.printf("\n--- Trying type %s at 0x%02X ---\n", sh1106 ? "SH1106 (usually 1.3\")" : "SSD1306 (usually 0.96\")", addr);
  if (!initOled(sh1106)) { Serial.println("Init command FAILED (no I2C answer)"); return; }
  Serial.println("1/4 ALL WHITE"); fill(sh1106, pWhite); delay(1500);
  Serial.println("2/4 CHECKER");    fill(sh1106, pChess); delay(1500);
  Serial.println("3/4 H-STRIPES"); fill(sh1106, pStripe); delay(1500);
  Serial.println("4/4 BORDER (all 4 edges must show)"); fill(sh1106, pBlack); fill(sh1106, pFrame);
  Serial.println("=> Clear image, correct position: this chip type is right.");
  Serial.println("   Image shifted / 1-2 noisy columns at the edge: it is the other chip type -> type the other number to try.");
}

bool scan(int sda, int scl) {
  Wire.end(); delay(10);
  Wire.begin(sda, scl); Wire.setClock(100000);
  Serial.printf("I2C scan SDA=%d SCL=%d:", sda, scl);
  int n = 0; addr = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a); n++;
      if (a == 0x3C || a == 0x3D) addr = a;
    }
  }
  Serial.println(n ? "" : " NOTHING FOUND");
  if (n && !addr) Serial.println("  Devices found but no 0x3C/0x3D (0x68 = MPU, 0x29 = VL53) -> OLED not detected");
  return addr != 0;
}

void checkPullup(int pin, const char* name) {
  pinMode(pin, INPUT); delay(5);
  int v = digitalRead(pin);
  Serial.printf("Pin %s (GPIO%d) at rest = %d %s\n", name, pin, v, v ? "OK (pull-up resistor present)" : "<- = 0: SHORTED to GND or OLED not powered!");
}

void fullScan() {
  Serial.println("\n=== TEST OLED ===");
  checkPullup(8, "SDA"); checkPullup(9, "SCL");
  if (scan(8, 9)) { SDA_PIN = 8; SCL_PIN = 9; }
  else if (scan(9, 8)) { SDA_PIN = 9; SCL_PIN = 8; Serial.println("!! OLED answers with SWAPPED pins -> SDA and SCL are wired reversed. Swap the wires."); }
  else {
    Serial.println("\n!! OLED NOT FOUND. Check in this order:");
    Serial.println("   1. Measure VCC-GND right at the OLED pins with power on: must be ~3.3 V");
    Serial.println("   2. Pin order printed on the OLED: GND-VCC or VCC-GND? reversed power = OLED dead (may be damaged)");
    Serial.println("   3. SDA -> GPIO8, SCL -> GPIO9; are the dupont wires firm, are the OLED header pins soldered");
    Serial.println("   4. Unplug MPU/VL53, leave only the OLED on the I2C bus and run again (type s)");
    return;
  }
  runTest(false);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  fullScan();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '1') runTest(false);
  else if (c == '2') runTest(true);
  else if (c == 's' || c == 'S') fullScan();
}
