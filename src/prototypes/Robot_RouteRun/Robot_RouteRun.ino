/*
  ROBOT - ROUTE RUN WITHOUT LINE, NO OLED NEEDED (NEW pin map v5)      v1.2
  ---------------------------------------------------------------------------
  The robot runs a pre-programmed route by itself, using the ENCODERS for distance and the MPU6050 GYRO to
  hold the heading / turn angles. No MPU -> uses the encoders for turning (less accurate).

  PINS:
    TB6612 : PWMA 38, AIN2 39, AIN1 40 (LEFT) | BIN1 41, BIN2 42, PWMB 2 (RIGHT) | STBY = VCC
    Encoder: LEFT A21 B47 | RIGHT A48 B1  (VCC 3V3)
    MPU6050: SDA 8, SCL 9, VCC 3V3, LYING FLAT
    Buttons: START 18 = run | SET 17 = change speed | MODE 16 = change route
    Buzzer : 15

  ROUTES (MODE to change, number of beeps = route number):
    1 STRAIGHT 1 m
    2 OUT - BACK   : 1 m -> turn 180 -> 1 m -> turn 180  (robot returns to the start)
    3 SQUARE       : 4 x (50 cm + turn left 90)
    4 OBSTACLE AVOID : turn left -> 15 cm -> turn right -> 40 cm -> turn right -> 15 cm -> turn left -> 30 cm
    5 STRAIGHT + VL53 : drive straight up to 2 m, ON AN OBSTACLE (VL53 < 15 cm) AVOID IT then continue
                     (put 1-2 paper boxes ~10x6 cm on the path to test)
  Free floor at least ~1.2 x 1.2 m. Start at 30-40 % speed.

  FIRST RUN: the robot nudges forward 0.3 s to fix the encoder SIGN by itself. If one wheel turns backward (robot spins)
  -> flip REV_L / REV_R below.
  Serial 115200: 1..5 or g (run) | btn (button test) | cpr 1400 | wheel 43 | track 100 | obs 150
  Libraries: VL53L0X (Pololu). NO OLED needed, NO line sensor bar needed.
*/
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ================= EDIT HERE =================
bool REV_L = true;              // LEFT motor reversed (confirmed)
bool REV_R = false;
float COUNTS_PER_REV = 1400.0f; // measure with Test_N20 (command c) then edit
float WHEEL_D_MM = 43.0f;       // wheel diameter
float TRACK_MM = 100.0f;        // wheel track (left wheel center -> right wheel center) - MEASURE WITH A RULER
float OBS_MM = 150;             // VL53: closer than this = obstacle
const float AV_SIDE = 150, AV_FWD = 400;   // avoid: sideways 15 cm, parallel 40 cm
const float KP_HEAD = 2.0f, KD_HEAD = 0.05f, KP_TURN = 1.2f, KD_TURN = 0.08f, TURN_MIN = 16;
// ===============================================

#define PIN_MAP 2   // 2 = NEW PINS v5 (perfboard) | 1 = OLD PINS (breadboard, as in Test_FullSpeed10s / Test_DriveStraight_Simple)
#if PIN_MAP == 2
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
#define BTN_START 18
#define BTN_SET 17
#define BTN_MODE 16
#define PIN_BUZ 15
#else
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
#define BTN_START 17
#define BTN_SET 18
#define BTN_MODE 0
#define PIN_BUZ 47
#endif
#define PIN_SDA 8
#define PIN_SCL 9
#define MPU_ADDR 0x68

const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = 1023;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_OUT(pin, ch, d) ledcWrite(pin, d)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_OUT(pin, ch, d) ledcWrite(ch, d)
#endif

// ---------- encoder ----------
volatile int32_t rawL = 0, rawR = 0;
volatile uint8_t prevL = 0, prevR = 0;
const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};
void IRAM_ATTR isrL() { uint8_t s = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB); rawL += QDEC[(prevL << 2) | s]; prevL = s; }
void IRAM_ATTR isrR() { uint8_t s = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB); rawR += QDEC[(prevR << 2) | s]; prevR = s; }
bool encRevL = false, encRevR = false, encChecked = false;
float mmL() { return (encRevL ? -rawL : rawL) * PI * WHEEL_D_MM / COUNTS_PER_REV; }
float mmR() { return (encRevR ? -rawR : rawR) * PI * WHEEL_D_MM / COUNTS_PER_REV; }

// ---------- motor ----------
void motorOne(int side, float pct) {
  uint8_t pwm = side ? PIN_PWMB : PIN_PWMA, in1 = side ? PIN_BIN1 : PIN_AIN1, in2 = side ? PIN_BIN2 : PIN_AIN2;
  if (side ? REV_R : REV_L) pct = -pct;
  pct = constrain(pct, -100.0f, 100.0f);
  digitalWrite(in1, pct > 0); digitalWrite(in2, pct < 0);
  PWM_OUT(pwm, side, (int)(fabsf(pct) * PWM_MAX / 100.0f));
}
void drive(float l, float r) { motorOne(0, l); motorOne(1, r); }
void coast() { drive(0, 0); }
void brake() {
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH); PWM_OUT(PIN_PWMA, 0, PWM_MAX);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH); PWM_OUT(PIN_PWMB, 1, PWM_MAX);
}

// ---------- MPU ----------
bool mpuOk = false; float gz = 0, bgz = 0, yaw = 0;
void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR); if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x80); delay(100); mpuWrite(0x6B, 0x01); mpuWrite(0x1A, 0x03); mpuWrite(0x19, 0x00); mpuWrite(0x1B, 0x18);
  return true;
}
bool readGz(float &o) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 2) != 2) return false;
  int16_t h = Wire.read(); int16_t r = (h << 8) | Wire.read(); o = r / 16.4f; return true;
}
void calibrateGyro() {
  coast(); double s = 0; int n = 0; float g; uint32_t t0 = millis();
  while (millis() - t0 < 1500) { if (readGz(g)) { s += g; n++; } delay(2); }
  bgz = n ? s / n : 0; yaw = 0;
}

// ---------- VL53L0X (non-blocking read) ----------
VL53L0X vl; bool vlOk = false; uint16_t vlMM = 8190; uint32_t vlT = 0;
void vlPoll() {
  if (!vlOk || millis() - vlT < 10) return;
  vlT = millis();
  if ((vl.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return;
  vlMM = vl.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  vl.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
}
bool obstacle() { return vlOk && vlMM > 20 && vlMM < OBS_MM; }

// ---------- buttons + buzzer ----------
void beep(int n, int on = 70) { for (int i = 0; i < n; i++) { digitalWrite(PIN_BUZ, HIGH); delay(on); digitalWrite(PIN_BUZ, LOW); delay(90); } }
const uint8_t BTN[3] = {BTN_START, BTN_SET, BTN_MODE};
bool bDown[3] = {false, false, false}; uint32_t bT[3] = {0, 0, 0};
int pressed() {
  for (int i = 0; i < 3; i++) {
    bool raw = digitalRead(BTN[i]) == LOW;
    if (raw != bDown[i] && millis() - bT[i] > 30) { bDown[i] = raw; bT[i] = millis(); if (raw) return i; }
  }
  return -1;
}

// ---------- common update ----------
bool aborted = false;
float yawEnc() { return (mmR() - mmL()) / TRACK_MM * 57.2958f; }   // angle from the encoders (deg)
float heading() { return mpuOk ? yaw : yawEnc(); }
void update() {
  static uint32_t t = micros();
  uint32_t now = micros();
  if (mpuOk && now - t >= 2000) { float dt = (now - t) * 1e-6f; t = now; float g; if (readGz(g)) { gz = g - bgz; yaw += gz * dt; } }
  vlPoll();
  if (pressed() >= 0) aborted = true;
}
void waitMs(uint32_t ms) { uint32_t t0 = millis(); while (millis() - t0 < ms && !aborted) update(); }

void checkEncoder() {
  Serial.println("Checking encoder signs: nudge forward 0.3 s");
  int32_t l0 = rawL, r0 = rawR; uint32_t t0 = millis();
  while (millis() - t0 < 300) { update(); drive(30, 30); }
  brake(); waitMs(200); coast();
  int32_t dl = rawL - l0, dr = rawR - r0;
  encRevL = dl < 0; encRevR = dr < 0;
  Serial.printf("  pulses L %+ld R %+ld -> encRev {%d,%d}\n", (long)dl, (long)dr, encRevL, encRevR);
  if (labs(dl) < 20) Serial.println("  !! LEFT encoder NOT COUNTING (21/47, VCC 3V3)");
  if (labs(dr) < 20) Serial.println("  !! RIGHT encoder NOT COUNTING (48/1, VCC 3V3)");
  encChecked = true;
}

// ---------- actions ----------
float straight(float mm, float pct) {
  float h = heading(), l0 = mmL(), r0 = mmR(); uint32_t t0 = millis();
  while (!aborted) {
    update();
    float dL = mmL() - l0, dR = mmR() - r0, done = (dL + dR) * 0.5f;
    if (done >= mm || millis() - t0 > 15000) break;
    float base = min(pct, min(15 + (millis() - t0) * 0.1f, 15 + (mm - done) * 0.35f));
    float corr = mpuOk ? KP_HEAD * (h - yaw) - KD_HEAD * gz : 0.8f * (dL - dR);
    corr = constrain(corr, -base * 0.6f, base * 0.6f);
    drive(base - corr, base + corr);
  }
  brake(); waitMs(120); coast();
  return (mmL() - l0 + mmR() - r0) * 0.5f;
}
// pivot another 'deg' degrees (positive = left / counter-clockwise)
float turn(float deg, float maxPct) {
  float target = heading() + deg; uint32_t t0 = millis(), tOk = 0;
  maxPct = constrain(maxPct, 25.0f, 60.0f);
  while (!aborted && millis() - t0 < 4000) {
    update();
    float e = target - heading();
    float rate = mpuOk ? gz : 0;
    float u = constrain(KP_TURN * e - KD_TURN * rate, -maxPct, maxPct);
    if (fabsf(e) > 1.0f && fabsf(u) < TURN_MIN) u = u >= 0 ? TURN_MIN : -TURN_MIN;
    if (fabsf(e) < 1.5f) { coast(); if (!tOk) tOk = millis(); if (millis() - tOk > 150) break; }
    else { tOk = 0; drive(-u, u); }
  }
  brake(); waitMs(120); coast();
  return heading() - target;
}

// ---------- routes ----------
const char* NAMES[] = {"", "STRAIGHT 1 m", "OUT - BACK", "SQUARE 50 cm", "OBSTACLE AVOID", "STRAIGHT + VL53 AVOID"};
void runRoute(int r, int pct) {
  if (!encChecked) { checkEncoder(); waitMs(500); }
  if (mpuOk) calibrateGyro();
  Serial.printf("\n=== ROUTE %d: %s  (%d%%, %s) ===\n", r, NAMES[r], pct, mpuOk ? "gyro" : "encoder only");
  float h0 = heading(); uint32_t t0 = millis(); float total = 0;
  switch (r) {
    case 1: total += straight(1000, pct); break;
    case 2: total += straight(1000, pct); turn(180, pct); total += straight(1000, pct); turn(180, pct); break;
    case 3: for (int i = 0; i < 4 && !aborted; i++) { total += straight(500, pct); turn(90, pct); } break;
    case 4: turn(90, pct); total += straight(150, pct); turn(-90, pct); total += straight(400, pct);
            turn(-90, pct); total += straight(150, pct); turn(90, pct); total += straight(300, pct); break;
    case 5: {
      if (!vlOk) { Serial.println("!! No VL53 -> route 5 cannot run"); break; }
      float h = heading(), done = 0; int n = 0;
      while (!aborted && done < 2000) {
        // drive straight piece by piece, checking for obstacles continuously
        float l0 = mmL(), r0 = mmR(); uint32_t t0 = millis(); bool hit = false;
        while (!aborted && done + ((mmL() - l0) + (mmR() - r0)) * 0.5f < 2000) {
          update();
          if (obstacle()) { hit = true; break; }
          float base = min((float)pct, 15 + (millis() - t0) * 0.1f);
          float corr = mpuOk ? KP_HEAD * (h - yaw) - KD_HEAD * gz : 0.8f * ((mmL() - l0) - (mmR() - r0));
          corr = constrain(corr, -base * 0.6f, base * 0.6f);
          drive(base - corr, base + corr);
        }
        done += ((mmL() - l0) + (mmR() - r0)) * 0.5f;
        brake(); waitMs(150); coast();
        if (!hit) break;
        n++; Serial.printf("Obstacle #%d at %u mm (after %.0f mm) -> AVOID\n", n, vlMM, done);
        turn(h + 90 - heading(), pct);  straight(AV_SIDE, pct);
        turn(h - heading(), pct);       straight(AV_FWD, pct);
        turn(h - 90 - heading(), pct);  straight(AV_SIDE, pct);
        turn(h - heading(), pct);
        done += AV_FWD;
      }
      total += done;
      Serial.printf("Avoided %d obstacles\n", n);
      break;
    }
  }
  coast();
  float err = heading() - h0;
  if (r == 2 || r == 3) err -= 360;
  if (r == 5) err = heading() - h0;   // 2 x 180 / 4 x 90 -> turned 360
  Serial.println(aborted ? "=== STOPPED BY BUTTON ===" : "=== DONE ===");
  Serial.printf("Time %.1f s | total distance %.0f mm | final heading error %+.1f deg\n", (millis() - t0) / 1000.0f, total, err);
  Serial.println("Pass: heading error < 5 deg, robot returns to the expected position (error < 5 cm).");
  if (!mpuOk) Serial.println("No MPU: turn angle from encoders -> adjust TRACK_MM if turns are short/long.");
  beep(aborted ? 1 : 2, aborted ? 400 : 70);
  aborted = false;
}

int route = 1, spdIdx = 1;
const int SPEEDS[] = {30, 40, 50, 60, 80};
void printSel() { Serial.printf("-> Route %d (%s) | speed %d%% | START = run\n", route, NAMES[route], SPEEDS[spdIdx]); }

void i2cScan() {
  Serial.print("\nI2C scan:");
  int n = 0, err = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a); uint8_t e = Wire.endTransmission();
    if (e == 0) { Serial.printf(" 0x%02X", a); n++; } else if (e == 5) err++;
  }
  Serial.println(n ? "" : " NOTHING FOUND");
  if (err > 50) Serial.println("!! Many error 5 (timeout): SDA/SCL pulled to GND or loose contact -> unplug OLED, check I2C solder joints");
}

const char* BNAME[3] = {"START", "SET", "MODE"};
void btnShow(uint32_t ms) {
  Serial.printf("Button states (1 = released, 0 = pressed) for %lu s - press each button to try:\n", (unsigned long)(ms / 1000));
  uint32_t t0 = millis(); int last = -1;
  while (millis() - t0 < ms) {
    int st = 0; for (int i = 0; i < 3; i++) st |= digitalRead(BTN[i]) << i;
    if (st != last) { last = st;
      Serial.printf("  START(GPIO%d)=%d  SET(GPIO%d)=%d  MODE(GPIO%d)=%d\n", BTN_START, st & 1, BTN_SET, (st >> 1) & 1, BTN_MODE, (st >> 2) & 1); }
    delay(10);
  }
}
void startRun(int r) {
  route = constrain(r, 1, 5);
  Serial.printf(">> RUN route %d in 1 s\n", route);
  beep(1, 150); delay(1000); aborted = false;
  for (int i = 0; i < 3; i++) { bDown[i] = digitalRead(BTN[i]) == LOW; bT[i] = millis(); }   // skip a button being held
  runRoute(route, SPEEDS[spdIdx]);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZ, OUTPUT); digitalWrite(PIN_BUZ, LOW);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  PWM_ATTACH(PIN_PWMA, 0); PWM_ATTACH(PIN_PWMB, 1);
  coast();
  for (int i = 0; i < 3; i++) pinMode(BTN[i], INPUT_PULLUP);
  pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP); pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
  prevL = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB);
  prevR = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), isrL, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_LB), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), isrR, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_RB), isrR, CHANGE);
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(100000);   // 100 kHz: more stable with long wires on the perfboard
  delay(800);
  i2cScan();
  for (int i = 0; i < 3 && !mpuOk; i++) { mpuOk = mpuInit(); if (!mpuOk) delay(200); }
  vl.setTimeout(100);
  for (int i = 0; i < 3 && !vlOk; i++) { vlOk = vl.init(); if (!vlOk) delay(200); }
  if (vlOk) { vl.setMeasurementTimingBudget(20000); vl.startContinuous(); }
  Serial.printf("\n=== ROBOT ROUTE RUN (NO LINE) === MPU %s | VL53 %s | CPR %.0f | wheel %.0f | track %.0f mm\n",
                mpuOk ? "OK" : "NONE", vlOk ? "OK" : "NONE", COUNTS_PER_REV, WHEEL_D_MM, TRACK_MM);
  if (mpuOk) calibrateGyro();
  Serial.println("MODE = route | SET = speed | START = run");
  for (int i = 0; i < 3; i++) if (digitalRead(BTN[i]) == LOW)
    Serial.printf("!! Button %s (GPIO%d) reads 0 when NOT pressed -> shorted to GND / wrong pin\n", BNAME[i], BTN[i]);
  Serial.println("Serial: g = run | 1..5 = run that route | btn = button test | scan = I2C scan");
  printSel(); beep(1);
}

void loop() {
  int b = pressed();
  if (b >= 0) Serial.printf("[button %s]\n", BNAME[b]);
  if (b == 2) { route = route % 5 + 1; printSel(); beep(route, 50); }
  if (b == 1) { spdIdx = (spdIdx + 1) % 5; printSel(); beep(1, 30); }
  if (b == 0) startRun(route);
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    char c[8] = {0}; float v = 0; int n = sscanf(s.c_str(), "%7s %f", c, &v); String k(c);
    if (k == "cpr" && n == 2) COUNTS_PER_REV = v;
    else if (k == "wheel" && n == 2) WHEEL_D_MM = v;
    else if (k == "track" && n == 2) TRACK_MM = v;
    else if (k == "obs" && n == 2) OBS_MM = v;
    else if (k == "scan") i2cScan();
    else if (k == "go" && n == 2) startRun((int)v);
    else if (k == "g") startRun(route);
    else if (k.length() == 1 && k[0] >= '1' && k[0] <= '5') startRun(k[0] - '0');
    else if (k == "btn") btnShow(10000);
    Serial.printf("CPR %.0f | wheel %.1f | track %.0f | MPU %s\n", COUNTS_PER_REV, WHEEL_D_MM, TRACK_MM, mpuOk ? "OK" : "NO");
  }
}
