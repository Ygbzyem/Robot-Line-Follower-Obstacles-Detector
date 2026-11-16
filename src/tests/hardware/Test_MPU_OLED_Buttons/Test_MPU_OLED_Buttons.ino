/*
  TEST MPU6050 + OLED 0.96" + 3 BUTTONS  -  Line Robot (MKE-K01 ESP32-S3)
  -----------------------------------------------------------------
  Wiring (NO driver/motor needed, powering the ESP32 from USB is enough):
    MPU6050 (GY-521): VCC -> 3V3   GND -> GND   SCL -> GPIO9   SDA -> GPIO8
                      AD0, INT, XDA, XCL left unconnected  (address 0x68)
    OLED SSD1306    : VCC -> 3V3   GND -> GND   SCL -> GPIO9   SDA -> GPIO8   (address 0x3C)
    START button : one pin -> GPIO17, the other -> GND
    SET button   : one pin -> GPIO18, the other -> GND
    MODE button  : one pin -> GPIO0 , the other -> GND  (or just use the BOOT button on the board)
               !! DO NOT hold MODE at power-up / reset (it enters flash download mode)
    All 3 buttons use the chip's INPUT_PULLUP -> NO resistor needed.

  Libraries (Library Manager): "Adafruit SSD1306" (installs "Adafruit GFX" with it).
  MPU6050 read directly over Wire, NO library needed.
  Board "ESP32S3 Dev Module", USB CDC On Boot = Enabled, Serial Monitor 115200.

  USAGE:
    MODE  : change page  1 Accel/Gyro  ->  2 Yaw angle  ->  3 Button test
    START : page 2 = reset yaw to 0
    SET   : recalibrate the gyro (keep the MPU STILL 2 s)
  At boot: scans I2C, must find 0x3C (OLED) and 0x68 (MPU). Then self-calibrates, keep still.
*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define PIN_SDA   8
#define PIN_SCL   9
#define BTN_START 17
#define BTN_SET   18
#define BTN_MODE  0

#define MPU_ADDR  0x68
#define OLED_ADDR 0x3C

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false, mpuOk = false;

// ---------------- MPU6050 ----------------
const float ACC_LSB  = 8192.0f;   // +-4 g
const float GYRO_LSB = 16.4f;     // +-2000 deg/s
float ax, ay, az, gx, gy, gz, tempC;   // g, do/s, do C
float bgx = 0, bgy = 0, bgz = 0;       // bias gyro
float yaw = 0;                          // deg, integrated gyro Z

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuInit() {
  if (!mpuWrite(0x6B, 0x80)) return false;   // reset
  delay(100);
  mpuWrite(0x6B, 0x01);   // wake up, clock = gyro X PLL
  mpuWrite(0x1A, 0x03);   // low-pass filter ~44 Hz
  mpuWrite(0x19, 0x00);   // sample rate 1 kHz
  mpuWrite(0x1B, 0x18);   // gyro +-2000 deg/s
  mpuWrite(0x1C, 0x08);   // accel +-4 g
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x75);   // WHO_AM_I
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  uint8_t id = Wire.available() ? Wire.read() : 0;
  Serial.printf("MPU WHO_AM_I = 0x%02X (0x68 = genuine MPU6050, 0x70/0x71/0x72 = clones that still work)\n", id);
  return id != 0;
}

bool mpuRead() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) { int16_t h = Wire.read(); r[i] = (h << 8) | Wire.read(); }
  ax = r[0] / ACC_LSB; ay = r[1] / ACC_LSB; az = r[2] / ACC_LSB;
  tempC = r[3] / 340.0f + 36.53f;
  gx = r[4] / GYRO_LSB - bgx; gy = r[5] / GYRO_LSB - bgy; gz = r[6] / GYRO_LSB - bgz;
  return true;
}

void showMsg(const char* l1, const char* l2 = "") {
  if (!oledOk) return;
  oled.clearDisplay(); oled.setTextSize(1); oled.setCursor(0, 0);
  oled.println(l1); oled.println(l2); oled.display();
}

void calibrateGyro() {
  Serial.println("Gyro calibration: KEEP MPU STILL 2 s...");
  showMsg("Gyro calibration", "KEEP STILL 2 s...");
  bgx = bgy = bgz = 0;
  double sx = 0, sy = 0, sz = 0; int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    if (mpuRead()) { sx += gx; sy += gy; sz += gz; n++; }
    delay(2);
  }
  if (n) { bgx = sx / n; bgy = sy / n; bgz = sz / n; }
  yaw = 0;
  Serial.printf("Gyro bias: X %.2f  Y %.2f  Z %.2f deg/s (%d samples). |bias| < 5 is normal\n", bgx, bgy, bgz, n);
}

// ---------------- BUTTONS ----------------
struct Btn { uint8_t pin; const char* name; bool down; uint32_t tChange; uint32_t count; bool pressed; };
Btn btns[3] = { {BTN_START, "START", false, 0, 0, false},
                {BTN_SET,   "SET",   false, 0, 0, false},
                {BTN_MODE,  "MODE",  false, 0, 0, false} };

void readButtons() {
  for (auto &b : btns) {
    b.pressed = false;
    bool raw = digitalRead(b.pin) == LOW;          // pressed = tied to GND = LOW
    if (raw != b.down && millis() - b.tChange > 30) {   // 30 ms debounce
      b.down = raw; b.tChange = millis();
      if (raw) { b.pressed = true; b.count++; Serial.printf("Button %s PRESSED (press %lu)\n", b.name, (unsigned long)b.count); }
    }
  }
}

// ---------------- I2C SCAN ----------------
void i2cScan() {
  Serial.println("I2C scan (SDA 8, SCL 9):");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found 0x%02X %s\n", a, a == 0x3C || a == 0x3D ? "(OLED)" : a == 0x68 || a == 0x69 ? "(MPU6050)" : "");
      n++;
    }
  }
  if (!n) Serial.println("  NOTHING found -> check VCC/GND, are SDA-SCL swapped");
}

// ---------------- SCREEN ----------------
int page = 0;

void drawPage() {
  if (!oledOk) return;
  oled.clearDisplay(); oled.setTextSize(1); oled.setCursor(0, 0);
  if (page == 0) {
    oled.println("1/3 ACCEL (g) GYRO");
    oled.printf("aX %6.2f gX %7.1f\n", ax, gx);
    oled.printf("aY %6.2f gY %7.1f\n", ay, gy);
    oled.printf("aZ %6.2f gZ %7.1f\n", az, gz);
    oled.printf("|a| %5.2f  T %4.1fC\n", sqrtf(ax * ax + ay * ay + az * az), tempC);
    oled.println(mpuOk ? "Lying flat: aZ~1, g~0" : "MPU ERROR!");
  } else if (page == 1) {
    oled.println("2/3 YAW ANGLE");
    oled.setTextSize(2); oled.setCursor(0, 14);
    oled.printf("%7.1f", yaw);
    oled.setTextSize(1); oled.setCursor(0, 34);
    float roll = atan2f(ay, az) * 57.2958f;
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
    oled.printf("roll %5.1f pit %5.1f\n", roll, pitch);
    oled.println("START=zero  SET=calib");
    // heading needle
    int cx = 112, cy = 20, r = 13;
    float a = yaw * 0.0174533f;
    oled.drawCircle(cx, cy, r, SSD1306_WHITE);
    oled.drawLine(cx, cy, cx + (int)(r * sinf(a)), cy - (int)(r * cosf(a)), SSD1306_WHITE);
  } else {
    oled.println("3/3 BUTTON TEST");
    for (int i = 0; i < 3; i++) {
      int y = 14 + i * 16;
      if (btns[i].down) oled.fillRect(0, y - 2, 128, 12, SSD1306_WHITE);
      oled.setTextColor(btns[i].down ? SSD1306_BLACK : SSD1306_WHITE);
      oled.setCursor(2, y);
      oled.printf("%-5s GPIO%-2d %s %3lu", btns[i].name, btns[i].pin, btns[i].down ? "PRESS" : "rel ", (unsigned long)btns[i].count);
      oled.setTextColor(SSD1306_WHITE);
    }
  }
  oled.display();
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  for (auto &b : btns) pinMode(b.pin, INPUT_PULLUP);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Serial.println("\n=== TEST MPU6050 + OLED + 3 BUTTONS ===");
  i2cScan();

  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  Serial.println(oledOk ? "OLED OK" : "OLED ERROR (try OLED_ADDR = 0x3D)");
  if (oledOk) { oled.setTextColor(SSD1306_WHITE); showMsg("Line Robot test", "MPU + OLED + buttons"); delay(800); }

  mpuOk = mpuInit();
  Serial.println(mpuOk ? "MPU OK" : "MPU ERROR: check wiring / address 0x68");
  if (mpuOk) calibrateGyro();
  else showMsg("MPU ERROR!", "check wiring");
}

void loop() {
  static uint32_t tLast = micros(), tDraw = 0, tPrint = 0;

  // read the MPU + integrate yaw every 2 ms (500 Hz)
  if (mpuOk && micros() - tLast >= 2000) {
    uint32_t now = micros(); float dt = (now - tLast) * 1e-6f; tLast = now;
    if (mpuRead()) {
      float z = fabsf(gz) < 0.3f ? 0 : gz;   // drop small noise when stationary
      yaw += z * dt;
      if (yaw > 180) yaw -= 360; else if (yaw < -180) yaw += 360;
    }
  }

  readButtons();
  if (btns[2].pressed) { page = (page + 1) % 3; Serial.printf("-> page %d\n", page + 1); }
  if (page != 2) {   // button test page: START/SET only counted, no action
    if (btns[0].pressed) { yaw = 0; Serial.println("-> yaw = 0"); }
    if (btns[1].pressed && mpuOk) calibrateGyro();
  }

  if (millis() - tDraw >= 50) { tDraw = millis(); drawPage(); }
  if (millis() - tPrint >= 200) {
    tPrint = millis();
    Serial.printf("a %6.2f %6.2f %6.2f g | gyro %7.1f %7.1f %7.1f deg/s | yaw %7.1f | btn %d%d%d\n",
                  ax, ay, az, gx, gy, gz, yaw, btns[0].down, btns[1].down, btns[2].down);
  }
}
