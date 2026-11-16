/*
  INTEGRATION TEST: MOTOR + ENCODER + MPU6050 + OLED + 3 BUTTONS  -  Line Robot (MKE-K01 ESP32-S3)
  -------------------------------------------------------------------------------------
  No line sensor needed yet. Uses the current motors (100 rpm). Switching to 500 rpm tomorrow: edit COUNTS_PER_REV.

  WIRING (same as the previous tests):
    TB6612 : VM <- 2S battery through a fuse (+470uF), VCC + STBY <- 3V3, GND <- common GND
             PWMA 38, AIN1 39, AIN2 40 -> LEFT motor (AO1/AO2)
             PWMB 41, BIN1 42, BIN2 16 -> RIGHT motor (BO1/BO2)
    Encoder: VCC 3V3, GND.  LEFT A 12, B 13   RIGHT A 14, B 15
    MPU6050 + OLED: SDA 8, SCL 9, VCC 3V3.  MPU LYING FLAT on the robot, near the wheel axle.
    Buttons: START 17, SET 18, MODE 0 (BOOT) - each button one pin to GPIO, one pin to GND.

  OLED MENU:
    MODE  : choose the test        SET : change speed (20/30/40/60/80/100 %)
    START : run the selected test (waits 1 s so you can move your hand away)
    While running: press ANY BUTTON for an EMERGENCY STOP.

  TESTS:
    0 CHECK DIRECTION  (robot ON THE FLOOR) : runs each wheel, checks with gyro + encoders
                                           motor/encoder direction, AUTO-FIXED for this run and printed
                                           values to edit in the code.
    1 MOTOR+ENCODER   (LIFT THE WHEELS)  : both wheels forward, shows rpm and mm/s per wheel.
    2 STRAIGHT 50 cm  (robot on the floor) : holds heading with the gyro, measures distance with the encoders.
    3 TURN 90 DEG     (robot on the floor) : left 90 -> back to 0 -> right 90 -> back to 0, reports the error.
    4 TRY OBSTACLE AVOID (robot on the floor) : turn left, sideways, straight, turn right, back to the old heading.

  Libraries: Adafruit SSD1306 (+ Adafruit GFX). Serial Monitor 115200.
*/
#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================== EDIT HERE ==================
float COUNTS_PER_REV = 4240.0f;   // 100 rpm motor = 4240 (measured). 500 rpm motor ~848 -> measure again with Test_N20
const float WHEEL_D_MM = 43.0f;   // wheel diameter
bool MOTOR_REV[2] = {false, true};   // [left, right] true = reverse motor direction   (test 0 reports the right value)
bool ENC_REV[2]   = {false, true};   // [left, right] true = reverse encoder sign

const float KP_TURN = 1.2f,  KD_TURN = 0.08f;   // pivot: % / deg, % / (deg/s)
const float TURN_MIN = 14.0f;                    // minimum % so the robot still turns (raise if the robot stops before reaching the angle)
const float KP_HEAD = 2.0f,  KD_HEAD = 0.05f;    // hold heading while driving straight
const float AV_SIDE_MM = 150, AV_FWD_MM = 400;   // test 4: sideways 15 cm, straight 40 cm
// =================================================

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
#define BTN_START 17
#define BTN_SET   18
#define BTN_MODE  0
#define MPU_ADDR  0x68

const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = 1023;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_OUT(pin, ch, d) ledcWrite(pin, d)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_OUT(pin, ch, d) ledcWrite(ch, d)
#endif

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false, mpuOk = false;

// ---------------- ENCODER ----------------
volatile int32_t encL = 0, encR = 0;
volatile uint8_t prevL = 0, prevR = 0;
const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};
void IRAM_ATTR isrL() {
  uint8_t s = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB);
  int8_t d = QDEC[(prevL << 2) | s]; prevL = s;
  encL += ENC_REV[0] ? -d : d;
}
void IRAM_ATTR isrR() {
  uint8_t s = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB);
  int8_t d = QDEC[(prevR << 2) | s]; prevR = s;
  encR += ENC_REV[1] ? -d : d;
}
float mmPerCount() { return PI * WHEEL_D_MM / COUNTS_PER_REV; }
float distMM() { return (encL + encR) * 0.5f * mmPerCount(); }

// ---------------- MOTOR ----------------
void motorOne(int side, float pct) {      // pct -100..100, positive = robot forward
  uint8_t pwm = side ? PIN_PWMB : PIN_PWMA, in1 = side ? PIN_BIN1 : PIN_AIN1, in2 = side ? PIN_BIN2 : PIN_AIN2;
  if (MOTOR_REV[side]) pct = -pct;
  pct = constrain(pct, -100.0f, 100.0f);
  int duty = (int)(fabsf(pct) * PWM_MAX / 100.0f);
  if (pct > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (pct < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else              { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  PWM_OUT(pwm, side, duty);
}
void drive(float l, float r) { motorOne(0, l); motorOne(1, r); }
void coast() { drive(0, 0); }
void brake() {   // short brake: IN1 = IN2 = HIGH, PWM = max
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH); PWM_OUT(PIN_PWMA, 0, PWM_MAX);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH); PWM_OUT(PIN_PWMB, 1, PWM_MAX);
}

// ---------------- MPU6050 ----------------
float gz = 0, bgz = 0;   // do/s
float yaw = 0;           // deg, NOT wrapped to +-180 (easier to compute turns)
void mpuWrite(uint8_t reg, uint8_t val) { Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission(); }
bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x80); delay(100);
  mpuWrite(0x6B, 0x01); mpuWrite(0x1A, 0x03); mpuWrite(0x19, 0x00);
  mpuWrite(0x1B, 0x18); mpuWrite(0x1C, 0x08);
  return true;
}
bool readGz(float &out) {   // read only gyro Z (2 bytes) for speed
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 2) != 2) return false;
  int16_t h = Wire.read(); int16_t raw = (h << 8) | Wire.read();
  out = raw / 16.4f;
  return true;
}
void calibrateGyro() {
  coast();
  if (oledOk) { oled.clearDisplay(); oled.setCursor(0, 0); oled.println("Gyro calibration"); oled.println("KEEP ROBOT STILL 2 s"); oled.display(); }
  double s = 0; int n = 0; uint32_t t0 = millis(); float g;
  while (millis() - t0 < 2000) { if (readGz(g)) { s += g; n++; } delay(2); }
  bgz = n ? s / n : 0; yaw = 0;
  Serial.printf("Gyro bias Z = %.2f deg/s (%d samples)\n", bgz, n);
}

// ---------------- BUTTONS ----------------
struct Btn { uint8_t pin; bool down; uint32_t t; bool pressed; };
Btn btns[3] = { {BTN_START, false, 0, false}, {BTN_SET, false, 0, false}, {BTN_MODE, false, 0, false} };
void readButtons() {
  for (auto &b : btns) {
    b.pressed = false;
    bool raw = digitalRead(b.pin) == LOW;
    if (raw != b.down && millis() - b.t > 30) { b.down = raw; b.t = millis(); if (raw) b.pressed = true; }
  }
}
bool anyPressed() { return btns[0].pressed || btns[1].pressed || btns[2].pressed; }

// ---------------- COMMON UPDATE (called continuously) ----------------
bool aborted = false;
float rpmL = 0, rpmR = 0;
void update() {
  static uint32_t tG = micros(), tS = 0;
  static int32_t lastL = 0, lastR = 0;
  uint32_t now = micros();
  if (mpuOk && now - tG >= 2000) {          // 500 Hz
    float dt = (now - tG) * 1e-6f; tG = now;
    float g;
    if (readGz(g)) { gz = g - bgz; yaw += gz * dt; }
  }
  if (millis() - tS >= 100) {               // wheel speed every 100 ms
    uint32_t dt = millis() - tS; tS = millis();
    int32_t l = encL, r = encR;
    rpmL = (l - lastL) / COUNTS_PER_REV * 60000.0f / dt;
    rpmR = (r - lastR) / COUNTS_PER_REV * 60000.0f / dt;
    lastL = l; lastR = r;
  }
  readButtons();
  if (anyPressed()) aborted = true;
}
// wait ms milliseconds while still updating the gyro
void waitMs(uint32_t ms) { uint32_t t0 = millis(); while (millis() - t0 < ms && !aborted) update(); }

// ---------------- BASIC ACTIONS ----------------
// pivot to the absolute angle target (deg). Positive = left (counter-clockwise)
float turnTo(float target, float maxPct) {
  uint32_t t0 = millis(), tOk = 0;
  while (!aborted && millis() - t0 < 4000) {
    update();
    float e = target - yaw;
    float u = KP_TURN * e - KD_TURN * gz;
    u = constrain(u, -maxPct, maxPct);
    if (fabsf(e) > 1.0f && fabsf(u) < TURN_MIN) u = u >= 0 ? TURN_MIN : -TURN_MIN;
    if (fabsf(e) < 1.5f && fabsf(gz) < 15) {
      coast();
      if (!tOk) tOk = millis();
      if (millis() - tOk > 150) break;      // standing still within the angle for 150 ms -> done
    } else {
      tOk = 0;
      drive(-u, u);
    }
  }
  brake(); waitMs(150); coast();
  return yaw - target;   // error
}
// drive straight mm (negative = backward) holding heading. Returns the actual distance measured by the encoders
float straight(float mm, float heading, float pct) {
  float d0 = distMM(), dir = mm >= 0 ? 1 : -1, goal = fabsf(mm);
  uint32_t t0 = millis();
  while (!aborted) {
    update();
    float done = fabsf(distMM() - d0);
    if (done >= goal) break;
    float ramp = 12 + (millis() - t0) * 0.08f;                 // gradual acceleration
    float slow = 12 + (goal - done) * 0.6f;                    // slow down when close
    float base = min(pct, min(ramp, slow));
    float corr = KP_HEAD * (heading - yaw) - KD_HEAD * gz;     // drifted right -> positive corr -> right wheel faster
    corr = constrain(corr, -base, base);
    drive(dir * base - corr, dir * base + corr);
    if (millis() - t0 > 15000) break;
  }
  brake(); waitMs(150); coast();
  return fabsf(distMM() - d0);
}

// ---------------- SCREEN ----------------
const char* TESTS[] = {"0 CHECK DIRECTION", "1 MOTOR+ENCODER", "2 STRAIGHT 50cm", "3 TURN 90 DEG", "4 TRY OBSTACLE AVOID"};
const char* WHERE[] = {"Put the robot on the floor", "LIFT THE WHEELS", "Put the robot on the floor", "Put the robot on the floor", "Needs ~60x40cm free"};
const int NT = 5;
const int SPEEDS[] = {20, 30, 40, 60, 80, 100};
int testSel = 0, spdIdx = 1;
char res[4][24] = {"", "", "", ""};

void setRes(int i, const char* fmt, ...) {
  va_list ap; va_start(ap, fmt); vsnprintf(res[i], sizeof(res[i]), fmt, ap); va_end(ap);
  Serial.println(res[i]);
}
void clearRes() { for (auto &r : res) r[0] = 0; }

void drawMenu() {
  if (!oledOk) return;
  oled.clearDisplay(); oled.setCursor(0, 0);
  oled.println(TESTS[testSel]);
  oled.printf("Speed %d%%  yaw%6.1f\n", SPEEDS[spdIdx], yaw);
  oled.println(WHERE[testSel]);
  oled.drawLine(0, 25, 127, 25, SSD1306_WHITE);
  oled.setCursor(0, 28);
  for (auto &r : res) oled.println(r);
  oled.display();
}
void showRunning() {
  if (!oledOk) return;
  oled.clearDisplay(); oled.setCursor(0, 0);
  oled.println(TESTS[testSel]); oled.println("RUNNING...");
  oled.println("Any button = STOP"); oled.display();
}

// ---------------- TESTS ----------------
void test0_direction() {
  Serial.println("\n=== TEST 0: DIRECTION CHECK (robot on the floor) ===");
  const char* nm[2] = {"LEFT", "RIGHT"};
  for (int side = 0; side < 2 && !aborted; side++) {
    float y0 = yaw; int32_t e0 = side ? encR : encL;
    uint32_t t0 = millis();
    while (millis() - t0 < 700 && !aborted) { update(); if (side) drive(0, 30); else drive(30, 0); }
    brake(); waitMs(400); coast();
    float dy = yaw - y0; int32_t de = (side ? encR : encL) - e0;
    // left wheel forward -> robot turns right (yaw decreases). Right wheel forward -> yaw increases
    bool turned = fabsf(dy) > 3;
    bool motorOk = side ? dy > 3 : dy < -3;
    bool encOk = (de > 0) == motorOk;
    Serial.printf("Wheel %s: yaw %+.1f deg, encoder %+ld pulses\n", nm[side], dy, (long)de);
    if (!turned) { setRes(side, "%s: robot NOT turning!", nm[side]); continue; }
    if (!motorOk) MOTOR_REV[side] = !MOTOR_REV[side];
    if (abs(de) < 20) { setRes(side, "%s mot%s ENC NO CNT", nm[side], motorOk ? "OK" : "FIX"); }
    else {
      if (!encOk) ENC_REV[side] = !ENC_REV[side];
      setRes(side, "%s mot%s enc%s", nm[side], motorOk ? "OK" : "FIX", encOk ? "OK" : "FIX");
    }
    waitMs(300);
  }
  setRes(2, "MOTOR_REV={%d,%d}", MOTOR_REV[0], MOTOR_REV[1]);
  setRes(3, "ENC_REV  ={%d,%d}", ENC_REV[0], ENC_REV[1]);
  Serial.println("-> Auto-fixed for this run. Copy the 2 lines above into the EDIT HERE section to keep permanently.");
}

void test1_motorEncoder(int pct) {
  Serial.printf("\n=== TEST 1: MOTOR + ENCODER %d%% (wheels lifted) - press a button to stop ===\n", pct);
  int32_t l0 = encL, r0 = encR;
  uint32_t tP = 0, t0 = millis();
  float base = 0;
  while (!aborted) {
    update();
    base = min((float)pct, (millis() - t0) * 0.1f);
    drive(base, base);
    if (millis() - tP > 300) {
      tP = millis();
      float k = PI * WHEEL_D_MM / 60.0f;   // rpm -> mm/s
      Serial.printf("rpm L %6.1f R %6.1f | %4.0f / %4.0f mm/s | pulses %ld %ld\n", rpmL, rpmR, rpmL * k, rpmR * k,
                    (long)(encL - l0), (long)(encR - r0));
      if (oledOk) {
        oled.clearDisplay(); oled.setCursor(0, 0);
        oled.printf("TEST 1  %d%%\n\n", pct);
        oled.printf("rpm  L%6.1f R%6.1f\n", rpmL, rpmR);
        oled.printf("mm/s L%6.0f R%6.0f\n", rpmL * k, rpmR * k);
        oled.printf("diff %5.1f %%\n", 100.0f * (rpmL - rpmR) / max(1.0f, fabsf(rpmL)));
        oled.println("\nA button = STOP"); oled.display();
      }
    }
  }
  coast();
  float k = PI * WHEEL_D_MM / 60.0f;
  setRes(0, "L %.0f rpm %.0f mm/s", rpmL, rpmL * k);
  setRes(1, "R %.0f rpm %.0f mm/s", rpmR, rpmR * k);
  setRes(2, "Both must be POSITIVE");
}

void test2_straight(int pct) {
  Serial.printf("\n=== TEST 2: STRAIGHT 500 mm %d%% ===\n", pct);
  float h = yaw; uint32_t t0 = millis();
  int32_t l0 = encL, r0 = encR;
  float d = straight(500, h, pct);
  float t = (millis() - t0) / 1000.0f;
  setRes(0, "Distance %.0f mm %.2f s", d, t);
  setRes(1, "Heading error %+.1f deg", yaw - h);
  setRes(2, "Pulse L%ld R%ld", (long)(encL - l0), (long)(encR - r0));
  setRes(3, "Ruler: ~50 cm?");
}

void test3_turn(int pct) {
  Serial.printf("\n=== TEST 3: TURN 90 DEG (max %d%%) ===\n", pct);
  float h = yaw, mx = max(25, min(pct, 60));
  float e1 = turnTo(h + 90, mx); waitMs(400);
  float e2 = turnTo(h, mx);      waitMs(400);
  float e3 = turnTo(h - 90, mx); waitMs(400);
  float e4 = turnTo(h, mx);
  setRes(0, "Left 90: err %+.1f", e1);
  setRes(1, "Back 0 : err %+.1f", e2);
  setRes(2, "Right 90: err %+.1f", e3);
  setRes(3, "Back 0 : err %+.1f", e4);
}

void test4_avoid(int pct) {
  Serial.printf("\n=== TEST 4: TRY OBSTACLE AVOID %d%% ===\n", pct);
  float h = yaw, mx = max(25, min(pct, 60));
  uint32_t t0 = millis();
  turnTo(h + 90, mx);  straight(AV_SIDE_MM, h + 90, pct);   // turn left, drive sideways
  turnTo(h, mx);       straight(AV_FWD_MM, h, pct);         // drive parallel past the obstacle
  turnTo(h - 90, mx);  straight(AV_SIDE_MM, h - 90, pct);   // turn right, back toward the line
  float e = turnTo(h, mx);                                  // turn back to the initial heading
  setRes(0, "Done %.1f s", (millis() - t0) / 1000.0f);
  setRes(1, "Heading error %+.1f deg", e);
  setRes(2, "Robot must be clear");
  setRes(3, "nose ~40cm, on line");
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  PWM_ATTACH(PIN_PWMA, 0); PWM_ATTACH(PIN_PWMB, 1);
  coast();
  pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
  prevL = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB);
  prevR = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LB), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), isrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RB), isrR, CHANGE);
  for (auto &b : btns) pinMode(b.pin, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOk) { oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE); }
  mpuOk = mpuInit();
  Serial.printf("\n=== INTEGRATION TEST === OLED %s, MPU %s, CPR %.0f\n", oledOk ? "OK" : "ERROR", mpuOk ? "OK" : "ERROR", COUNTS_PER_REV);
  if (mpuOk) calibrateGyro();
  else { setRes(0, "MPU ERR! test 0,2,3,4"); setRes(1, "won't run correctly"); }
}

void loop() {
  update();
  if (aborted) aborted = false;   // in the menu: a button is not an emergency stop
  if (btns[2].pressed) { testSel = (testSel + 1) % NT; clearRes(); }
  if (btns[1].pressed) { spdIdx = (spdIdx + 1) % 6; }
  if (btns[0].pressed) {
    int pct = SPEEDS[spdIdx];
    clearRes(); showRunning();
    uint32_t t0 = millis(); while (millis() - t0 < 1000) update();   // 1 s to move your hand away
    aborted = false;
    switch (testSel) {
      case 0: test0_direction(); break;
      case 1: test1_motorEncoder(pct); break;
      case 2: test2_straight(pct); break;
      case 3: test3_turn(pct); break;
      case 4: test4_avoid(pct); break;
    }
    coast();
    if (aborted) { setRes(3, "** STOP BY BUTTON **"); }
    uint32_t t1 = millis(); while (millis() - t1 < 300) update();     // skip the stop press
    aborted = false;
  }
  static uint32_t tD = 0;
  if (millis() - tD > 150) { tD = millis(); drawMenu(); }
}
