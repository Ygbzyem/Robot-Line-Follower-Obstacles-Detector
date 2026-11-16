/*
  ==========================================================================================
   FULL ROBOT TEST  -  Line Robot (MKE-K01 ESP32-S3 + MKE-B01 shield)          v1.0
  ==========================================================================================
   Checks EACH MODULE and the WHOLE ROBOT: power, buttons, buzzer, MPU, line sensor, VL53, motors,
   encoders, straight/turn, line following, obstacle avoidance. Results on the OLED + Serial 115200.
   Measured parameters (CPR, motor/encoder direction, line calibration, battery ratio, Kp/Kd...)
   SAVED automatically to flash (NVS) -> kept after power-off, the main firmware can read it.

   ---------------------------- NETLIST SUMMARY (v5 - perfboard) ----------------------------
   Shield placed vertically, USB toward the robot front. Each pin group: GND (outer) - 5V - SIG (inner). The MIDDLE ROW = 5 V!
   LEFT HEADER (19..TX)   : encoders + driver
     21 C1 left | 47 C2 left | 48 C1 right | 1 C2 right
     38 PWMA (+10k to GND) | 39 AIN2 | 40 AIN1 | 41 BIN1 | 42 BIN2 | 2 PWMB (+10k to GND)
     STBY: soldered to VCC (3V3) on the driver perfboard. Unused: 19 20 (USB) 45 0 35 36 37 RX TX
   RIGHT HEADER (14..4)   : line sensor, I2C, buttons, buzzer, battery sense
     Line D1..D8 -> 14 13 12 11 10 7 6 5 | SCL 9, SDA 8 (shared I2C rail MPU + OLED + VL53)
     START 18 | SET 17 | MODE 16 | Buzzer 15 (3-pin, 5V OK) | VBAT 4 (divider 100k/33k)
   3V3 SUPPLY (shield 3V3 rail): encoders, line sensor, MPU, OLED, VL53, TB6612 VCC+STBY
     -> NEVER take 5V from the G-5V-S headers for these modules.
   2S battery -> BMS -> 5A fuse -> switch -> shield DC jack + TB6612 VM (+470uF +100nF)
   Full per-pin netlist: file Robot_Netlist_v4.pdf
   --------------------------------------------------------------------------------------

   USAGE:  MODE = choose test  |  SET = change speed  |  START = run
             While running: press ANY BUTTON = STOP (tests 2, 4, 7: see the instructions on the OLED)
   RECOMMENDED ORDER FOR THE FIRST RUN: 0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 7 -> 6 -> 8 -> 9 -> 10 -> 11

    0 SELF TEST        robot still          : I2C, MPU, VL53, battery, buttons, line sensor
    1 BATTERY / POWER  any               : battery voltage directly (compare with a meter)
    2 BUTTONS + BUZZER any               : press all 3 buttons, the buzzer beeps on each press
    3 MPU6050          hold still, then turn : yaw, turn rate
    4 LINE SENSOR      on the track     : 8 black-level bars; SET = calibrate
    5 VL53L0X          any               : distance ahead
    6 MOTOR + ENCODER  LIFT THE WHEELS   : rpm, mm/s, encoder sign, wheel mismatch
    7 PULSES / TURN    LIFT THE WHEELS   : turn by hand 10 turns -> saves CPR + encoder sign
    8 CHECK DIRECTION  robot on the floor : uses the gyro to fix motor/encoder direction
    9 STRAIGHT + TURN  robot on the floor : 50 cm, turn 180, drive back, turn 180
   10 LINE FOLLOW      on the track     : follow the line to the finish / line loss
   11 LINE + AVOID     on the track     : follow the line, avoid obstacles then catch the line again

   SERIAL COMMANDS (Newline):  info | save | reset
     vcal 8.30   (enter the battery voltage measured with a meter -> adjusts the ratio)
     cpr 1400    | kp 10 | kd 0.25 | obs 120 | avs 150 | avf 600 | avdir 1 (1 left, -1 right)
     wheel 43    | lineflip 1 (if the line position is reversed: line on the right gives a negative value)

   Libraries: Adafruit SSD1306 + Adafruit GFX, VL53L0X (Pololu).
   Board "ESP32S3 Dev Module", USB CDC On Boot = Enabled.
*/
#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <VL53L0X.h>

// ===================================== PINS =====================================
#define PIN_MAP 1   // 1 = OLD PINS (breadboard) | 2 = NEW PINS v5 (perfboard)
#if PIN_MAP == 1
#define PIN_PWMA 38
#define PIN_AIN1 39
#define PIN_AIN2 40
#define PIN_PWMB 41
#define PIN_BIN1 42
#define PIN_BIN2 16
#define ENC_LA 12
#define ENC_LB 13
#define ENC_RA 14
#define ENC_RB 15
#define PIN_SDA 8
#define PIN_SCL 9
#define PIN_VBAT 11
#define PIN_BUZ 47
#define BTN_START 17
#define BTN_SET   18
#define BTN_MODE  0
const uint8_t LINE_PIN[8] = {1, 2, 3, 4, 5, 6, 7, 10};
#else
#define PIN_PWMA 38
#define PIN_AIN2 39
#define PIN_AIN1 40
#define PIN_BIN1 41
#define PIN_BIN2 42
#define PIN_PWMB 2
#define ENC_LA 21
#define ENC_LB 47
#define ENC_RA 48
#define ENC_RB 1
#define PIN_SDA 8
#define PIN_SCL 9
#define PIN_VBAT 4
#define PIN_BUZ 15
#define BTN_START 18
#define BTN_SET   17
#define BTN_MODE  16
const uint8_t LINE_PIN[8] = {14, 13, 12, 11, 10, 7, 6, 5};   // 11-14 are ADC2
#endif
#define MPU_ADDR 0x68
#define OLED_ADDR 0x3C

// ============================ PARAMETERS (STORED IN NVS) ============================
struct Cfg {
  uint32_t magic;
  float cpr;            // pulses per output shaft turn
  float wheelD;         // mm
  uint8_t motorRev[2], encRev[2];
  uint8_t dirChecked;   // test 8 already run
  uint16_t lmin[8], lmax[8];
  uint8_t lineBlackHigh, lineCal, lineFlip;
  float vbatCal;
  float kp, kd;         // line following
  float obsMM, avSide, avFwd; int8_t avDir;
};
const uint32_t MAGIC = 0xF3A2E005;
Cfg cfg;
void cfgDefault() {
  cfg = Cfg();
  cfg.magic = MAGIC; cfg.cpr = 1400; cfg.wheelD = 43;   // 300 rpm motor: ~1400 -> measure with test 7
  cfg.motorRev[0] = 0; cfg.motorRev[1] = 1; cfg.encRev[0] = 0; cfg.encRev[1] = 1;
  for (int i = 0; i < 8; i++) { cfg.lmin[i] = 0; cfg.lmax[i] = 4095; }
  cfg.lineBlackHigh = 1; cfg.lineCal = 0; cfg.vbatCal = 1.0f;
  cfg.kp = 10; cfg.kd = 0.25f; cfg.obsMM = 120; cfg.avSide = 150; cfg.avFwd = 600; cfg.avDir = 1;
}
Preferences prefs;
void cfgLoad() {
  prefs.begin("fmaze", false);   // NVS name kept from the first firmware so data already stored on the robot survives updates
  Cfg t;
  if (prefs.getBytes("cfg", &t, sizeof(t)) == sizeof(t) && t.magic == MAGIC) cfg = t; else cfgDefault();
}
void cfgSave() { prefs.putBytes("cfg", &cfg, sizeof(cfg)); Serial.println("-> Parameters saved to NVS"); }

// ============================== BASIC HARDWARE ==============================
const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = 1023;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_OUT(pin, ch, d) ledcWrite(pin, d)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_OUT(pin, ch, d) ledcWrite(ch, d)
#endif

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
VL53L0X vl;
bool oledOk = false, mpuOk = false, vlOk = false;

// ---- encoder ----
volatile int32_t encL = 0, encR = 0;
volatile uint8_t prevL = 0, prevR = 0;
volatile bool encRevL = false, encRevR = true;
const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};
void IRAM_ATTR isrL() {
  uint8_t s = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB);
  int8_t d = QDEC[(prevL << 2) | s]; prevL = s; encL += encRevL ? -d : d;
}
void IRAM_ATTR isrR() {
  uint8_t s = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB);
  int8_t d = QDEC[(prevR << 2) | s]; prevR = s; encR += encRevR ? -d : d;
}
void applyEncRev() { encRevL = cfg.encRev[0]; encRevR = cfg.encRev[1]; }
float mmPerCount() { return PI * cfg.wheelD / cfg.cpr; }
float distMM() { return (encL + encR) * 0.5f * mmPerCount(); }

// ---- motor ----
void motorOne(int side, float pct) {          // positive pct = robot forward
  uint8_t pwm = side ? PIN_PWMB : PIN_PWMA, in1 = side ? PIN_BIN1 : PIN_AIN1, in2 = side ? PIN_BIN2 : PIN_AIN2;
  if (cfg.motorRev[side]) pct = -pct;
  pct = constrain(pct, -100.0f, 100.0f);
  if (pct > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (pct < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else              { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  PWM_OUT(pwm, side, (int)(fabsf(pct) * PWM_MAX / 100.0f));
}
void drive(float l, float r) { motorOne(0, l); motorOne(1, r); }
void coast() { drive(0, 0); }
void brake() {
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH); PWM_OUT(PIN_PWMA, 0, PWM_MAX);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH); PWM_OUT(PIN_PWMB, 1, PWM_MAX);
}

// ---- MPU6050 ----
float gz = 0, bgz = 0, yaw = 0;
void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
uint8_t mpuWho() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(MPU_ADDR, 1); return Wire.available() ? Wire.read() : 0;
}
bool mpuInit() {
  if (!mpuWho()) return false;
  mpuWrite(0x6B, 0x80); delay(100); mpuWrite(0x6B, 0x01); mpuWrite(0x1A, 0x03);
  mpuWrite(0x19, 0x00); mpuWrite(0x1B, 0x18); mpuWrite(0x1C, 0x08);
  return true;
}
bool readGz(float &o) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 2) != 2) return false;
  int16_t h = Wire.read(); int16_t raw = (h << 8) | Wire.read(); o = raw / 16.4f; return true;
}

// ---- VL53L0X (non-blocking read) ----
uint16_t vlMM = 0; bool vlFresh = false; uint32_t vlT = 0;
void vlPoll() {
  if (!vlOk || millis() - vlT < 10) return;
  vlT = millis();
  if ((vl.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return;
  vlMM = vl.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  vl.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  vlFresh = true;
}
bool obstacleAhead() { return vlOk && vlMM > 20 && vlMM < cfg.obsMM; }

// ---- battery ----
float vbat = 0;
float readVbat() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_VBAT);
  return mv / 8.0f / 1000.0f * (133.0f / 33.0f) * cfg.vbatCal;
}

// ---- buzzer ----
uint32_t buzOff = 0;
void beep(uint16_t ms) { digitalWrite(PIN_BUZ, HIGH); buzOff = millis() + ms; }
void beepBlock(int n, uint16_t on, uint16_t off) { for (int i = 0; i < n; i++) { digitalWrite(PIN_BUZ, HIGH); delay(on); digitalWrite(PIN_BUZ, LOW); delay(off); } }

// ---- line sensor ----
uint16_t lraw[8]; float lval[8];   // lval: 0 = white, 1 = black
int lineBlack = 0; float linePos = 0; bool lineSeen = false;
void readLine() {
  float sw = 0, s = 0; lineBlack = 0;
  for (int i = 0; i < 8; i++) {
    lraw[i] = analogRead(LINE_PIN[cfg.lineFlip ? 7 - i : i]);
    float span = max(1, (int)cfg.lmax[i] - (int)cfg.lmin[i]);
    float v = constrain(((int)lraw[i] - (int)cfg.lmin[i]) / span, 0.0f, 1.0f);
    if (!cfg.lineBlackHigh) v = 1 - v;
    lval[i] = v;
    if (v > 0.5f) lineBlack++;
    if (v > 0.2f) { sw += v; s += v * (i - 3.5f); }
  }
  lineSeen = sw > 0.4f;
  if (lineSeen) linePos = s / sw;   // -3.5 (left) .. +3.5 (right)
}

// ---- buttons ----
struct Btn { uint8_t pin; const char* name; bool down; uint32_t t; bool pressed; uint16_t cnt; };
Btn btns[3] = { {BTN_START, "START", false, 0, false, 0}, {BTN_SET, "SET", false, 0, false, 0}, {BTN_MODE, "MODE", false, 0, false, 0} };
void readButtons() {
  for (auto &b : btns) {
    b.pressed = false;
    bool raw = digitalRead(b.pin) == LOW;
    if (raw != b.down && millis() - b.t > 30) { b.down = raw; b.t = millis(); if (raw) { b.pressed = true; b.cnt++; } }
  }
}
bool anyPressed() { return btns[0].pressed || btns[1].pressed || btns[2].pressed; }

// ============================== COMMON UPDATE ==============================
bool aborted = false;
float rpmL = 0, rpmR = 0;
void update() {
  static uint32_t tG = micros(), tS = 0, tV = 0;
  static int32_t lastL = 0, lastR = 0;
  uint32_t now = micros();
  if (mpuOk && now - tG >= 2000) {
    float dt = (now - tG) * 1e-6f; tG = now; float g;
    if (readGz(g)) { gz = g - bgz; yaw += gz * dt; }
  }
  if (millis() - tS >= 100) {
    uint32_t dt = millis() - tS; tS = millis(); int32_t l = encL, r = encR;
    rpmL = (l - lastL) / cfg.cpr * 60000.0f / dt; rpmR = (r - lastR) / cfg.cpr * 60000.0f / dt;
    lastL = l; lastR = r;
  }
  if (millis() - tV >= 200) { tV = millis(); vbat = readVbat(); }
  vlPoll();
  if (buzOff && (int32_t)(millis() - buzOff) >= 0) { digitalWrite(PIN_BUZ, LOW); buzOff = 0; }
  readButtons();
  if (anyPressed()) aborted = true;
}
void waitMs(uint32_t ms) { uint32_t t0 = millis(); while (millis() - t0 < ms && !aborted) update(); }
void calibrateGyro() {
  coast(); double s = 0; int n = 0; float g; uint32_t t0 = millis();
  while (millis() - t0 < 1500) { if (readGz(g)) { s += g; n++; } delay(2); }
  bgz = n ? s / n : 0; yaw = 0;
}

// ============================== MOTION ==============================
const float KP_TURN = 1.2f, KD_TURN = 0.08f, TURN_MIN = 16;
const float KP_HEAD = 2.0f, KD_HEAD = 0.05f;
float turnTo(float target, float maxPct) {
  uint32_t t0 = millis(), tOk = 0;
  while (!aborted && millis() - t0 < 4000) {
    update();
    float e = target - yaw, u = constrain(KP_TURN * e - KD_TURN * gz, -maxPct, maxPct);
    if (fabsf(e) > 1.0f && fabsf(u) < TURN_MIN) u = u >= 0 ? TURN_MIN : -TURN_MIN;
    if (fabsf(e) < 1.5f && fabsf(gz) < 15) { coast(); if (!tOk) tOk = millis(); if (millis() - tOk > 150) break; }
    else { tOk = 0; drive(-u, u); }
  }
  brake(); waitMs(120); coast();
  return yaw - target;
}
// drive straight holding heading; stop when the distance is reached or (stopOnLine && line seen)
float straight(float mm, float heading, float pct, bool stopOnLine = false) {
  float d0 = distMM(), dir = mm >= 0 ? 1 : -1, goal = fabsf(mm); uint32_t t0 = millis();
  while (!aborted) {
    update();
    float done = fabsf(distMM() - d0);
    if (done >= goal) break;
    if (stopOnLine) { readLine(); if (lineBlack >= 1 && done > 30) break; }
    float base = min(pct, min(12 + (millis() - t0) * 0.08f, 12 + (goal - done) * 0.6f));
    float corr = constrain(KP_HEAD * (heading - yaw) - KD_HEAD * gz, -base, base);
    drive(dir * base - corr, dir * base + corr);
    if (millis() - t0 > 15000) break;
  }
  brake(); waitMs(100); coast();
  return fabsf(distMM() - d0);
}

// ============================== SCREEN ==============================
const char* TESTS[] = {"0 SELF TEST", "1 BATTERY / POWER", "2 BUTTONS + BUZZER", "3 MPU6050", "4 LINE SENSOR",
                       "5 VL53L0X", "6 MOTOR+ENCODER", "7 PULSES/TURN", "8 CHECK DIRECTION",
                       "9 STRAIGHT + TURN", "10 LINE FOLLOW", "11 LINE+AVOID"};
const char* WHERE[] = {"Robot still", "Any", "Any", "Hold still then turn", "Place on the track",
                       "Any", "LIFT THE WHEELS!", "LIFT THE WHEELS!", "Robot on the floor",
                       "Free floor 60x30", "On the line", "On the line"};
const int NT = 12;
const int SPEEDS[] = {20, 30, 40, 50, 60, 80, 100};
int testSel = 0, spdIdx = 1;
char res[6][22];
void setRes(int i, const char* fmt, ...) {
  va_list ap; va_start(ap, fmt); vsnprintf(res[i], sizeof(res[i]), fmt, ap); va_end(ap);
  Serial.println(res[i]);
}
void clearRes() { for (auto &r : res) r[0] = 0; }
void oledText(const char* l1, const char* l2 = "", const char* l3 = "", const char* l4 = "") {
  if (!oledOk) return;
  oled.clearDisplay(); oled.setCursor(0, 0);
  oled.println(l1); oled.println(l2); oled.println(l3); oled.println(l4); oled.display();
}
void drawMenu() {
  if (!oledOk) return;
  oled.clearDisplay(); oled.setCursor(0, 0);
  oled.println(TESTS[testSel]);
  oled.printf("%-16.16s%3d%%\n", WHERE[testSel], SPEEDS[spdIdx]);
  for (auto &r : res) oled.println(r);
  oled.display();
}

// ================================ TESTS ================================
// ---------- 0. self test ----------
void t0_post() {
  int ok = 0, n = 0;
  // I2C
  bool hasOled = false, hasMpu = false, hasVl = false;
  for (uint8_t a = 1; a < 127; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) {
      if (a == 0x3C || a == 0x3D) hasOled = true; else if (a == 0x68) hasMpu = true; else if (a == 0x29) hasVl = true;
      Serial.printf("  I2C found 0x%02X\n", a); } }
  n++; if (hasOled && hasMpu && hasVl) { ok++; setRes(0, "I2C OLED MPU VL53 OK"); }
  else setRes(0, "I2C MISSING:%s%s%s", hasOled ? "" : " OLED", hasMpu ? "" : " MPU", hasVl ? "" : " VL53");
  // MPU
  n++; uint8_t who = mpuWho();
  if (mpuOk && fabsf(bgz) < 5) { ok++; setRes(1, "MPU 0x%02X bias %.1f OK", who, bgz); }
  else setRes(1, "MPU ERR who=0x%02X", who);
  // VL53
  n++; uint32_t t0 = millis(); vlFresh = false; while (millis() - t0 < 300) update();
  if (vlOk && vlFresh) { ok++; if (vlMM > 8000) setRes(2, "VL53 OK (no object)"); else setRes(2, "VL53 OK %u mm", vlMM); }
  else setRes(2, "VL53 ERR / no read");
  // battery
  n++; float v = readVbat();
  if (v > 9.0f) setRes(3, "BATT %.2fV BAD DIVIDER!", v);
  else if (v < 3.0f) setRes(3, "BATT %.2fV no battery?", v);
  else if (v < 6.6f) setRes(3, "BATT %.2fV LOW!", v);
  else { ok++; setRes(3, "PIN %.2f V OK", v); }
  // buttons
  n++; bool stuck = false; for (auto &b : btns) if (digitalRead(b.pin) == LOW) stuck = true;
  if (!stuck) { ok++; setRes(4, "No stuck button OK"); } else setRes(4, "BTN STUCK (LOW)");
  // line
  n++; readLine(); int bad = 0;
  for (int i = 0; i < 8; i++) { Serial.printf("  line D%d = %u\n", i + 1, lraw[i]); if (lraw[i] < 20 || lraw[i] > 4075) bad++; }
  if (bad >= 6) setRes(5, "LINE: %d bad ch", bad);
  else { ok++; setRes(5, cfg.lineCal ? "LINE OK (calibrated)" : "LINE OK, NOT CAL"); }
  Serial.printf("=> SELF TEST: %d/%d PASSED\n", ok, n);
  if (ok == n) beepBlock(2, 60, 60); else beepBlock(1, 600, 0);
}
// ---------- 1. battery ----------
void t1_battery() {
  float vmin = 99, vmax = 0;
  while (!aborted) {
    update();
    static uint32_t tp = 0;
    if (millis() - tp > 300) {
      tp = millis(); vmin = min(vmin, vbat); vmax = max(vmax, vbat);
      uint32_t mv = analogReadMilliVolts(PIN_VBAT);
      Serial.printf("VBAT %.2f V | GPIO11 %u mV | min %.2f max %.2f\n", vbat, mv, vmin, vmax);
      if (oledOk) { oled.clearDisplay(); oled.setCursor(0, 0); oled.println("1 BATTERY / POWER");
        oled.setTextSize(2); oled.setCursor(0, 14); oled.printf("%5.2f V", vbat); oled.setTextSize(1);
        oled.setCursor(0, 36); oled.printf("GPIO11 %4u mV\n", mv);
        oled.println("Clock drift? Serial:"); oled.println("vcal <meter reading>"); oled.display(); }
    }
  }
  setRes(0, "Battery %.2f V", vbat); setRes(1, "Min %.2f  Max %.2f", vmin, vmax);
  setRes(2, vbat >= 7.0f ? "Battery level good" : "Battery low, charge it");
}
// ---------- 2. buttons + buzzer ----------
void t2_buttons() {
  for (auto &b : btns) b.cnt = 0;
  uint32_t t0 = millis(), tAll = 0;
  while (millis() - t0 < 20000) {
    update(); aborted = false;
    for (auto &b : btns) if (b.pressed) { beep(60); Serial.printf("Button %s (press %u)\n", b.name, b.cnt); }
    bool all = btns[0].cnt && btns[1].cnt && btns[2].cnt;
    if (all && !tAll) tAll = millis();
    if (all && millis() - tAll > 1500) break;
    static uint32_t tp = 0;
    if (oledOk && millis() - tp > 80) { tp = millis(); oled.clearDisplay(); oled.setCursor(0, 0);
      oled.println("2 BTN+BUZ: press 3"); oled.printf("left %2lu s\n", (unsigned long)(20 - (millis() - t0) / 1000));
      for (auto &b : btns) oled.printf("%-5s GPIO%-2d %s %u\n", b.name, b.pin, b.down ? "PRESS" : "    ", b.cnt);
      oled.display(); }
  }
  for (int i = 0; i < 3; i++) setRes(i, "%-5s %s (%u times)", btns[i].name, btns[i].cnt ? "OK" : "NOT PRESSED", btns[i].cnt);
  setRes(3, "Buzzer beeping?");
}
// ---------- 3. MPU ----------
void t3_mpu() {
  if (!mpuOk) { setRes(0, "MPU ERROR"); return; }
  calibrateGyro(); float drift0 = yaw; uint32_t t0 = millis();
  while (!aborted) {
    update();
    static uint32_t tp = 0;
    if (millis() - tp > 150) { tp = millis(); Serial.printf("yaw %7.1f | gz %6.1f deg/s\n", yaw, gz);
      if (oledOk) { oled.clearDisplay(); oled.setCursor(0, 0); oled.println("3 MPU6050  (btn=stop)");
        oled.setTextSize(2); oled.setCursor(0, 14); oled.printf("%7.1f", yaw); oled.setTextSize(1);
        oled.setCursor(0, 36); oled.printf("gz %6.1f deg/s\n", gz); oled.printf("bias %.2f\n", bgz);
        oled.println("Turn 90 -> reads ~90"); oled.display(); } }
  }
  setRes(0, "Bias %.2f deg/s", bgz); setRes(1, "Final yaw %.1f deg", yaw);
  setRes(2, "Time %lu s", (unsigned long)((millis() - t0) / 1000));
  (void)drift0;
}
// ---------- 4. line sensor ----------
void lineCalibrate() {
  uint16_t mn[8], mx[8]; double sum[8] = {0}; uint32_t cnt = 0;
  for (int i = 0; i < 8; i++) { mn[i] = 4095; mx[i] = 0; }
  bool spin = cfg.dirChecked && mpuOk;
  oledText("LINE CALIBRATION", spin ? "Robot sweeps over line" : "SWEEP the sensor back and forth", spin ? "" : "over the line for 5 s", "");
  Serial.println(spin ? "Calibration: robot turns +-35 deg by itself" : "Calibration: sweep the sensor across the line by hand for 5 s");
  calibrateGyro();
  uint32_t t0 = millis(); int phase = 0; float targets[4] = {35, -35, 35, 0};
  while (millis() - t0 < 5000) {
    update(); aborted = false;
    for (int i = 0; i < 8; i++) { uint16_t r = analogRead(LINE_PIN[cfg.lineFlip ? 7 - i : i]); mn[i] = min(mn[i], r); mx[i] = max(mx[i], r); sum[i] += r; }
    cnt++;
    if (spin && phase < 4) {
      float e = targets[phase] - yaw;
      if (fabsf(e) < 3) phase++;
      else { float u = constrain(1.0f * e, -30.0f, 30.0f); if (fabsf(u) < TURN_MIN) u = u > 0 ? TURN_MIN : -TURN_MIN; drive(-u, u); }
    } else coast();
    delay(2);
  }
  coast();
  int good = 0, highVotes = 0;
  for (int i = 0; i < 8; i++) {
    float mean = sum[i] / max<uint32_t>(cnt, 1);
    if (mx[i] - mn[i] > 300) good++;
    // most of the time the sensor is over the white floor -> the mean is close to the white level
    if (mean < (mn[i] + mx[i]) / 2.0f) highVotes++;   // white = low -> black = high
    cfg.lmin[i] = mn[i]; cfg.lmax[i] = mx[i];
    Serial.printf("  D%d min %4u max %4u avg %6.0f\n", i + 1, mn[i], mx[i], mean);
  }
  cfg.lineBlackHigh = highVotes >= 4; cfg.lineCal = good >= 7;
  setRes(0, "Calib %d/8 ch good", good);
  setRes(1, "Black = %s", cfg.lineBlackHigh ? "HIGH value" : "LOW value");
  if (cfg.lineCal) { cfgSave(); setRes(2, "Saved"); beepBlock(2, 60, 60); }
  else { setRes(2, "FAIL: sweep again"); beepBlock(1, 500, 0); }
}
void t4_line() {
  Serial.println("Test 4: SET = calibrate | START/MODE = exit");
  while (true) {
    update(); aborted = false;
    if (btns[1].pressed) { lineCalibrate(); continue; }
    if (btns[0].pressed || btns[2].pressed) break;
    static uint32_t tp = 0;
    if (millis() - tp > 60) {
      tp = millis(); readLine();
      if (oledOk) {
        oled.clearDisplay(); oled.setCursor(0, 0);
        oled.printf("4 LINE %s SET=calib\n", cfg.lineCal ? "OK" : "--");
        for (int i = 0; i < 8; i++) { int h = (int)(lval[i] * 36); oled.fillRect(4 + i * 15, 48 - h, 11, h, SSD1306_WHITE); oled.drawRect(4 + i * 15, 12, 11, 36, SSD1306_WHITE); }
        oled.setCursor(0, 54);
        if (lineSeen) oled.printf("pos %+5.2f black %d", linePos, lineBlack); else oled.print("LINE NOT FOUND");
        oled.display();
      }
      static uint32_t ts = 0;
      if (millis() - ts > 300) { ts = millis(); Serial.print("raw:");
        for (int i = 0; i < 8; i++) Serial.printf(" %4u", lraw[i]);
        Serial.printf(" | pos %+5.2f black %d\n", linePos, lineBlack); }
    }
  }
  if (!res[0][0]) { setRes(0, cfg.lineCal ? "Line calibrated" : "NOT calibrated"); }
}
// ---------- 5. VL53 ----------
void t5_vl53() {
  if (!vlOk) { setRes(0, "VL53 ERR - check wire"); return; }
  uint16_t mn = 9999;
  while (!aborted) {
    update();
    static uint32_t tp = 0;
    if (millis() - tp > 100) { tp = millis(); bool far = vlMM > 8000; if (!far) mn = min(mn, vlMM);
      Serial.printf("VL53 %u mm %s\n", vlMM, obstacleAhead() ? "<- VAT CAN" : "");
      if (obstacleAhead()) beep(30);
      if (oledOk) { oled.clearDisplay(); oled.setCursor(0, 0); oled.println("5 VL53L0X (btn=stop)");
        oled.setTextSize(2); oled.setCursor(0, 16); if (far) oled.print(" ---- "); else oled.printf("%4u mm", vlMM);
        oled.setTextSize(1); oled.setCursor(0, 40); oled.printf("Obstacle threshold %.0f\n", cfg.obsMM);
        oled.println(obstacleAhead() ? ">> OBSTACLE <<" : ""); oled.display(); } }
  }
  setRes(0, "VL53 OK"); setRes(1, "Nearest %u mm", mn); setRes(2, "Thresh %.0f mm", cfg.obsMM);
}
// ---------- 6. motor + encoder (wheels lifted) ----------
void t6_motor(int pct) {
  int32_t l0 = encL, r0 = encR; uint32_t t0 = millis();
  float sumL = 0, sumR = 0; int n = 0; float vMin = 99;
  while (!aborted && millis() - t0 < 6000) {
    update();
    float b = min((float)pct, (millis() - t0) * 0.1f); drive(b, b);
    if (millis() - t0 > 2500) { static uint32_t tm = 0; if (millis() - tm >= 100) { tm = millis(); sumL += rpmL; sumR += rpmR; n++; vMin = min(vMin, vbat); } }
    static uint32_t tp = 0;
    if (millis() - tp > 300) { tp = millis();
      Serial.printf("rpm L %6.1f R %6.1f | pulses %ld %ld | batt %.2f\n", rpmL, rpmR, (long)(encL - l0), (long)(encR - r0), vbat);
      if (oledOk) { oled.clearDisplay(); oled.setCursor(0, 0); oled.printf("6 MOTOR %d%%\n\n", pct);
        oled.printf("rpm L %6.1f\nrpm R %6.1f\nbatt %.2f V\n", rpmL, rpmR, vbat); oled.display(); } }
  }
  coast();
  float aL = n ? sumL / n : 0, aR = n ? sumR / n : 0, k = PI * cfg.wheelD / 60.0f;
  setRes(0, "L %5.0f rpm %4.0fmm/s", aL, aL * k);
  setRes(1, "R %5.0f rpm %4.0fmm/s", aR, aR * k);
  setRes(2, "Diff %.1f %%", 100.0f * (aL - aR) / max(1.0f, fabsf(aL)));
  setRes(3, "Encoder %s / %s", aL > 0 ? "+OK" : "NEG!", aR > 0 ? "+OK" : "NEG!");
  setRes(4, "Batt sag %.2f V", vMin);
  if (aL <= 0 || aR <= 0) setRes(5, "-> run test 7 then 8");
}
// ---------- 7. pulses / turn ----------
void t7_cpr() {
  coast(); encL = 0; encR = 0;
  Serial.println("Turn EACH wheel by hand exactly 10 turns TOWARD THE ROBOT FRONT, then press START");
  while (true) {
    update(); aborted = false;
    if (btns[0].pressed) break;
    if (btns[2].pressed) { setRes(0, "Cancel"); return; }
    static uint32_t tp = 0;
    if (millis() - tp > 150) { tp = millis();
      if (oledOk) { oled.clearDisplay(); oled.setCursor(0, 0); oled.println("7 PULSES/TURN");
        oled.println("Turn by hand 10 turns"); oled.println("FORWARD on each wheel");
        oled.printf("\nL %7ld\nR %7ld\n\nSTART=done MODE=cancel", (long)encL, (long)encR); oled.display(); } }
  }
  long L = encL, R = encR;
  setRes(0, "L %ld = %.0f/turn", L, labs(L) / 10.0f);
  setRes(1, "R %ld = %.0f/turn", R, labs(R) / 10.0f);
  if (labs(L) < 100 || labs(R) < 100) { setRes(2, "Encoder no count!"); return; }
  if (L < 0) cfg.encRev[0] ^= 1;
  if (R < 0) cfg.encRev[1] ^= 1;
  applyEncRev();
  cfg.cpr = (labs(L) + labs(R)) / 20.0f;
  setRes(2, "CPR = %.0f", cfg.cpr);
  setRes(3, "ENC_REV {%d,%d}", cfg.encRev[0], cfg.encRev[1]);
  setRes(4, "L/R mismatch %.1f%%", 100.0f * (labs(L) - labs(R)) / max(1L, labs(L)));
  cfgSave();
}
// ---------- 8. direction check (on the floor) ----------
void t8_direction() {
  const char* nm[2] = {"LEFT", "RIGHT"}; int okCount = 0;
  calibrateGyro();
  for (int side = 0; side < 2 && !aborted; side++) {
    float y0 = yaw; int32_t e0 = side ? encR : encL; uint32_t t0 = millis();
    while (millis() - t0 < 700 && !aborted) { update(); if (side) drive(0, 30); else drive(30, 0); }
    brake(); waitMs(400); coast();
    float dy = yaw - y0; int32_t de = (side ? encR : encL) - e0;
    Serial.printf("Wheel %s: yaw %+.1f, encoder %+ld\n", nm[side], dy, (long)de);
    if (fabsf(dy) < 3) { setRes(side, "%s: robot NOT turning", nm[side]); continue; }
    bool motorOk = side ? dy > 0 : dy < 0;
    if (!motorOk) cfg.motorRev[side] ^= 1;
    if (labs(de) < 20) { setRes(side, "%s mot%s ENC NO CNT", nm[side], motorOk ? "OK" : "FIX"); continue; }
    bool encOk = (de > 0) == motorOk;
    if (!encOk) cfg.encRev[side] ^= 1;
    setRes(side, "%s mot%s enc%s", nm[side], motorOk ? "OK" : "FIX", encOk ? "OK" : "FIX");
    okCount++;
    waitMs(300);
  }
  applyEncRev();
  setRes(2, "MOTOR_REV {%d,%d}", cfg.motorRev[0], cfg.motorRev[1]);
  setRes(3, "ENC_REV   {%d,%d}", cfg.encRev[0], cfg.encRev[1]);
  if (okCount == 2 && !aborted) { cfg.dirChecked = 1; cfgSave(); setRes(4, "Saved. Run again to"); setRes(5, "confirm: both OK"); }
}
// ---------- 9. straight + turn ----------
void t9_straightTurn(int pct) {
  calibrateGyro(); float h = 0, mx = max(25, min(pct, 60)); uint32_t t0 = millis();
  float d1 = straight(500, h, pct);       float e1 = turnTo(h + 180, mx);
  float d2 = straight(500, h + 180, pct); float e2 = turnTo(h + 360, mx);
  setRes(0, "Drive: %.0f + %.0f mm", d1, d2);
  setRes(1, "Turn 180: err %+.1f", e1);
  setRes(2, "Turn 360: err %+.1f", e2);
  setRes(3, "Time %.1f s", (millis() - t0) / 1000.0f);
  setRes(4, "Back near the start?");
}
// ---------- 10/11. line following (+ obstacle avoidance) ----------
const uint32_t DASH_MS = 400;   // max line loss (dashed segment)
const float FIN_MM = 40;        // wide black >= 6 channels continuously for 40 mm = finish
bool avoidObstacle(int pct) {
  float h = yaw, s = cfg.avDir, mx = max(25, min(pct, 60));
  Serial.printf("OBSTACLE AVOID at %u mm\n", vlMM);
  brake(); waitMs(150);
  turnTo(h + 90 * s, mx);  straight(cfg.avSide, h + 90 * s, pct);
  turnTo(h, mx);           straight(cfg.avFwd, h, pct);
  turnTo(h - 90 * s, mx);
  straight(cfg.avSide + 120, h - 90 * s, min(pct, 30), true);   // toward the line, stop when the line is seen
  readLine(); bool found = lineBlack >= 1;
  if (found) straight(55, h - 90 * s, 20);   // bring the wheel axle close to the line
  turnTo(h, mx);                           // turn back to the old heading, follow the line again
  return found;
}
void t10_follow(int pct, bool withObstacle) {
  calibrateGyro();
  if (!cfg.lineCal) { setRes(0, "LINE NOT CALIBRATED!"); setRes(1, "Run test 4 first"); return; }
  uint32_t t0 = millis(), tLost = 0, tLast = micros();
  float lastErr = 0, maxErr = 0, finStart = -1, d0 = distMM(), lostHeading = 0;
  int obst = 0; const char* why = "STOP BUTTON";
  while (!aborted) {
    update(); readLine();
    uint32_t now = micros(); float dt = max(1e-4f, (now - tLast) * 1e-6f); tLast = now;
    float d = distMM() - d0;
    // finish: wide black continuously
    if (lineBlack >= 6 && millis() - t0 > 1000) { if (finStart < 0) finStart = d; if (d - finStart >= FIN_MM) { why = "TO FINISH"; break; } }
    else finStart = -1;
    // vat can
    if (withObstacle && obstacleAhead() && millis() - t0 > 500) {
      obst++; bool f = avoidObstacle(pct); if (aborted) break;
      if (!f) Serial.println("Avoid done but no line yet, keep trying");
      tLast = micros(); lastErr = 0; continue;
    }
    if (lineSeen) {
      tLost = 0;
      float err = linePos; maxErr = max(maxErr, fabsf(err));
      float u = cfg.kp * err + cfg.kd * (err - lastErr) / dt; lastErr = err;
      float base = pct * (1.0f - 0.45f * min(1.0f, fabsf(err) / 3.0f));
      drive(base + u, base - u);
      lostHeading = yaw;
    } else {
      if (!tLost) tLost = millis();
      if (millis() - tLost > DASH_MS) { why = "LINE LOST"; break; }
      float corr = KP_HEAD * (lostHeading - yaw) - KD_HEAD * gz;   // hold heading through the dashed segment
      drive(pct * 0.7f - corr, pct * 0.7f + corr);
    }
  }
  brake(); waitMs(200); coast();
  float t = (millis() - t0) / 1000.0f, d = distMM() - d0;
  setRes(0, "Stop: %s", why);
  setRes(1, "%.1f s  %.0f mm", t, d);
  setRes(2, "AVG %.0f mm/s", d / max(0.1f, t));
  setRes(3, "Max error %.2f", maxErr);
  setRes(4, "Kp %.1f Kd %.2f", cfg.kp, cfg.kd);
  if (withObstacle) setRes(5, "Avoided %d obstacles", obst);
}

// ================================ SERIAL COMMANDS ================================
void printInfo() {
  Serial.printf("CPR %.0f | wheel %.0f mm | MOTOR_REV {%d,%d} ENC_REV {%d,%d} | dirChecked %d\n",
    cfg.cpr, cfg.wheelD, cfg.motorRev[0], cfg.motorRev[1], cfg.encRev[0], cfg.encRev[1], cfg.dirChecked);
  Serial.printf("Line calib %d, black=%s, flip %d | vbatCal %.3f | kp %.2f kd %.3f\n", cfg.lineCal, cfg.lineBlackHigh ? "high" : "low", cfg.lineFlip, cfg.vbatCal, cfg.kp, cfg.kd);
  Serial.printf("obs %.0f mm | avSide %.0f | avFwd %.0f | avDir %d\n", cfg.obsMM, cfg.avSide, cfg.avFwd, cfg.avDir);
}
void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n'); s.trim();
  char cmd[12] = {0}; float v = 0;
  int n = sscanf(s.c_str(), "%11s %f", cmd, &v);
  if (n < 1) return;
  String c(cmd);
  if (c == "info") printInfo();
  else if (c == "save") cfgSave();
  else if (c == "reset") { cfgDefault(); cfgSave(); applyEncRev(); Serial.println("-> Reset to defaults"); }
  else if (n < 2) Serial.println("Missing value. E.g.: kp 10");
  else if (c == "vcal") { float now = readVbat(); if (now > 1) { cfg.vbatCal *= v / now; Serial.printf("vbatCal = %.3f\n", cfg.vbatCal); cfgSave(); } }
  else if (c == "cpr") { cfg.cpr = v; cfgSave(); }
  else if (c == "kp") { cfg.kp = v; cfgSave(); }
  else if (c == "kd") { cfg.kd = v; cfgSave(); }
  else if (c == "obs") { cfg.obsMM = v; cfgSave(); }
  else if (c == "avs") { cfg.avSide = v; cfgSave(); }
  else if (c == "avf") { cfg.avFwd = v; cfgSave(); }
  else if (c == "avdir") { cfg.avDir = v >= 0 ? 1 : -1; cfgSave(); }
  else if (c == "wheel") { cfg.wheelD = v; cfgSave(); }
  else if (c == "lineflip") { cfg.lineFlip = v != 0; cfgSave(); }
  else Serial.println("Unknown command. Type: info");
}

// ================================ SETUP / LOOP ================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZ, OUTPUT); digitalWrite(PIN_BUZ, LOW);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  PWM_ATTACH(PIN_PWMA, 0); PWM_ATTACH(PIN_PWMB, 1);
  coast();
  delay(800);
  cfgLoad(); applyEncRev();
  analogReadResolution(12);
  for (auto &b : btns) pinMode(b.pin, INPUT_PULLUP);
  pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP); pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
  prevL = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB);
  prevR = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), isrL, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_LB), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), isrR, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_RB), isrR, CHANGE);

  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) { oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE); }
  oledText("Line Robot FULL TEST", "Keep the robot still...", "", "");
  mpuOk = mpuInit();
  vl.setTimeout(50);
  vlOk = vl.init();
  if (vlOk) { vl.setMeasurementTimingBudget(20000); vl.startContinuous(); }
  if (mpuOk) calibrateGyro();
  Serial.printf("\n=== LINE ROBOT FULL TEST === OLED %d MPU %d VL53 %d\n", oledOk, mpuOk, vlOk);
  printInfo();
  clearRes();
  t0_post();
}

void loop() {
  update(); handleSerial();
  aborted = false;
  if (btns[2].pressed) { testSel = (testSel + 1) % NT; clearRes(); beep(15); }
  if (btns[1].pressed) { spdIdx = (spdIdx + 1) % 7; beep(15); }
  if (btns[0].pressed) {
    int pct = SPEEDS[spdIdx];
    clearRes();
    oledText(TESTS[testSel], "Get ready... 1 s", "Press a button = STOP", WHERE[testSel]);
    beep(80);
    uint32_t t0 = millis(); while (millis() - t0 < 1000) update();
    aborted = false;
    Serial.printf("\n=== %s (%d%%) ===\n", TESTS[testSel], pct);
    switch (testSel) {
      case 0: t0_post(); break;
      case 1: t1_battery(); break;
      case 2: t2_buttons(); break;
      case 3: t3_mpu(); break;
      case 4: t4_line(); break;
      case 5: t5_vl53(); break;
      case 6: t6_motor(pct); break;
      case 7: t7_cpr(); break;
      case 8: t8_direction(); break;
      case 9: t9_straightTurn(pct); break;
      case 10: t10_follow(pct, false); break;
      case 11: t10_follow(pct, true); break;
    }
    coast();
    beep(aborted && testSel >= 8 ? 400 : 60);
    uint32_t t1 = millis(); while (millis() - t1 < 300) update();
    aborted = false;
  }
  static uint32_t tD = 0;
  if (millis() - tD > 150) { tD = millis(); drawMenu(); }
}
