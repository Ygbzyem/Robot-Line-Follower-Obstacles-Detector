/*
  ============================================================================
  LINE ROBOT  -  Firmware v0.3-MKE  (winding track, NO encoders)
  Board : MakerEDU MKE-K01 ESP32-S3 Dev Kit (DevKitC-1 compatible)
          mounted on the MKE-B01 ESP32-S3 DK IO Shield
  IDE   : Arduino IDE 2.x + "esp32 by Espressif Systems" core 3.x
          Tools > Board: "ESP32S3 Dev Module", USB CDC On Boot: Enabled
          (the MKE-K01 has no PSRAM: PSRAM = Disabled, Flash Size per board)
  Libraries: VL53L0X (Pololu), Adafruit SSD1306 + Adafruit GFX Library
  ----------------------------------------------------------------------------
  TRACK (updated 10/2026): one path from START -> FINISH, no closed loops,
  only curves (no 90-degree corners), with these sections:
    - DASHED LINE / CROSSBAR : short line gap or a crossbar across the line
    - HONEYCOMB GRID             : many Y-junctions -> pick the branch that keeps the overall heading
    - FINISH = BLACK BOX        : solid black continuously > FinMs -> stop
    - 2 OBSTACLES 5-10 cm apart, not on curves -> go around BOTH at once
  No encoders: avoid distance = time x speed (AvoidSpd,
  measured with SPEED mode), turn angle = MPU6050 gyro. Battery voltage compensation keeps the speed steady.
  ----------------------------------------------------------------------------
  Modes (MODE/SET buttons, see the OLED):
    RUN   : run at Base speed        FAST : run at Fast speed
    CALIB : calibrate the line sensor bar (put the robot on the line, it sweeps back and forth)
    SPEED : drive straight 2 s at AvoidPWM -> measure the distance (mm), divide by 2, enter as AvoidSpd
  Buttons: MODE short = next page | SET short = increase / change mode | SET hold = decrease
       START = run (keep the robot still ~1 s for gyro calibration); press again to stop.
  EVERY PARAMETER IS SAVED IN NVS -> tune at the track without re-flashing the code.
  ============================================================================
*/
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// WOKWI SIMULATION: uncomment the line below when running on wokwi.com (DO NOT enable when flashing the real robot)
//   - gyro simulated from PWM, 3 VL53L0X replaced by 3 potentiometers on GPIO12/13/14, buzzer uses tone()
// #define SIM_WOKWI

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_OUT(pin, ch, d) ledcWrite(pin, d)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_OUT(pin, ch, d) ledcWrite(ch, d)
#endif

// ============================== PINS ========================================
static const uint8_t LINE_PINS[8] = {1, 2, 3, 4, 5, 6, 7, 10};  // S1 (left) -> S8 (right), ADC1
#define PIN_VBAT      11     // voltage divider 100k/33k
#define PIN_BTN_START 17
#define PIN_BTN_MODE   0     // BOOT button
#define PIN_BTN_SET   18
#define PIN_SDA        8
#define PIN_SCL        9
#define PIN_XSHUT_R   21
#define PIN_XSHUT_L   48
#define PIN_BUZ       47
#define PIN_PWMA      38     // left motor
#define PIN_AIN1      39
#define PIN_AIN2      40
#define PIN_PWMB      41     // right motor
#define PIN_BIN1      42
#define PIN_BIN2      16
// GPIO12-15 (old encoder pins) are left unused

// ============================ CONSTANTS (MEASURE AGAIN!) =============================
const bool  LINE_BLACK_HIGH   = true;    // does the black line give a HIGH voltage? if wrong set false
const float SENSOR_TO_AXLE_MM = 120.0f;  // line sensor bar -> wheel axle (anchor-shaped chassis, MEASURE AGAIN)
const float GYRO_LSB_PER_DPS  = 16.4f;   // MPU6050 +-2000 dps
const float VNOM              = 7.8f;    // main-menu PWMs are set for this battery level
const float VBAT_WARN         = 7.2f;
const float VBAT_CUTOFF       = 6.6f;    // no BMS: < 3.3 V/cell for 2 s -> lock the robot
const float VBAT_MIN_START    = 6.8f;
const bool  BUZ_ON_LEVEL      = HIGH;
const int   PWM_FREQ = 20000;
const int   PWM_BITS = 10;
const int   PWM_MAX  = (1 << PWM_BITS) - 1;   // 1023
const uint32_t LOOP_US = 2000;                // 500 Hz
const float DT       = LOOP_US * 1e-6f;
const int   BLACK_TH = 600;                   // "black" threshold after normalizing 0..1000
const int   WIDE_N   = 6;                     // >= 6 black channels = crossbar / honeycomb apex / finish box
const float YAWREF_TAU = 0.8f;                // s: overall heading (slow filter) used to choose honeycomb branches

// ======================= TUNABLE PARAMETERS (NVS) ===========================
struct Params {
  float kp, kd;      // PD line following
  float base, fast;  // base PWM for RUN / FAST (at 7.8 V)
  float slow;        // 0..0.9: slow down when the line error is at full scale (entering a curve)
  float turnPwm;     // pivot PWM
  float obstMm;      // obstacle threshold at Base speed (mm)
  float avoidSide;   // 0 = go around left, 1 = go around right, 2 = choose the inner side
  float avoidPwm;    // PWM while avoiding obstacles
  float avoidSpd;    // actual mm/s at AvoidPWM (measured with SPEED mode)
  float margin;      // extra mm of clearance when going around an obstacle
  float gapMm;       // largest gap between 2 obstacles (rule: 5-10 cm)
  float dashMs;      // max ms of blind driving holding heading on a dashed line
  float finMs;       // ms of solid black continuously -> FINISH
  float forkMs;      // ms to keep the branch choice at a honeycomb T-junction
};
Params P = {0.10f, 1.0f, 500, 750, 0.45f, 380, 200, 2, 450, 400, 70, 100, 250, 120, 150};

struct MenuItem { const char* name; float* v; float step; float lo; float hi; };
MenuItem MENU[] = {
  {"Kp",        &P.kp,        0.01f, 0.0f, 2.0f},
  {"Kd",        &P.kd,        0.10f, 0.0f, 20.0f},
  {"Base",      &P.base,      25,    100,  1023},
  {"Fast",      &P.fast,      25,    100,  1023},
  {"Slow",      &P.slow,      0.05f, 0.0f, 0.9f},
  {"TurnPWM",   &P.turnPwm,   20,    100,  1023},
  {"ObstMm",    &P.obstMm,    10,    60,   500},
  {"Side0L1R2A",&P.avoidSide, 1,     0,    2},
  {"AvoidPWM",  &P.avoidPwm,  25,    150,  1023},
  {"AvoidSpd",  &P.avoidSpd,  10,    50,   3000},
  {"Margin",    &P.margin,    10,    0,    200},
  {"GapMm",     &P.gapMm,     10,    20,   250},
  {"DashMs",    &P.dashMs,    10,    0,    1000},
  {"FinMs",     &P.finMs,     10,    20,   1000},
  {"ForkMs",    &P.forkMs,    10,    0,    600},
};
const int N_MENU = sizeof(MENU) / sizeof(MENU[0]);

enum Mode { M_RUN, M_FAST, M_CALIB, M_SPEED, M_COUNT };
const char* MODE_NAME[] = {"RUN", "FAST", "CALIB", "SPEED"};
int mode = M_RUN, page = 0;

// ============================== OBJECTS ===================================
Preferences prefs;
VL53L0X tofF, tofL, tofR;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false, tofFOk = false, tofLOk = false, tofROk = false;

// ============================== STATE ==================================
uint16_t calMin[8], calMax[8];
int   lv[8];
bool  blk[8];
int   nBlk = 0;
bool  onLine = false;
// consecutive black clusters on the sensor bar (1 cluster = 1 line branch)
int   nGrp = 0;
float grpPos[4];                  // cluster center position: -3500 (left) .. +3500 (right)
float lineErr = 0, lastErr = 0;

float yaw = 0, yawRef = 0, gyroBias = 0;    // deg, positive = turning left (CCW)
int   distF = 8190, distL = 8190, distR = 8190;
float vbatF = VNOM;
uint32_t tNext = 0, tickCount = 0;
bool  abortRun = false, lowBattLock = false;
uint32_t lowSince = 0;
int   forkSide = 0;               // 0 = not chosen, -1 = left branch, +1 = right branch
uint32_t forkUntil = 0;
uint32_t nForks = 0, nDash = 0, nWide = 0, nObst = 0;   // statistics printed to Serial

#ifdef SIM_WOKWI
const uint8_t SIM_POT_F = 12, SIM_POT_L = 13, SIM_POT_R = 14;
const float   SIM_TRACK_MM = 140.0f, SIM_V_FULL = 1400.0f;   // mm/s at max PWM
float simCmdL = 0, simCmdR = 0;
int simTof(uint8_t pin) { return 20 + (long)analogRead(pin) * 980 / 4095; }
#endif

float wrap180(float a) { while (a > 180) a -= 360; while (a <= -180) a += 360; return a; }

// ================================= MOTOR ====================================
void motorOne(uint8_t pwmPin, uint8_t in1, uint8_t in2, int v) {
  v = constrain(v, -PWM_MAX, PWM_MAX);
  if (v >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else        { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); v = -v; }
  PWM_OUT(pwmPin, (pwmPin == PIN_PWMA ? 0 : 1), v);
}
// voltage compensation: battery drops 8.4 -> 7.0 V, PWM rises automatically -> speed nearly constant
void drive(float l, float r) {
  float k = constrain(VNOM / vbatF, 0.85f, 1.25f);
  motorOne(PIN_PWMA, PIN_AIN1, PIN_AIN2, (int)(l * k));
  motorOne(PIN_PWMB, PIN_BIN1, PIN_BIN2, (int)(r * k));
#ifdef SIM_WOKWI
  simCmdL = constrain(l * k, -PWM_MAX, PWM_MAX); simCmdR = constrain(r * k, -PWM_MAX, PWM_MAX);
#endif
}
void coast() {
  PWM_OUT(PIN_PWMA, 0, 0); PWM_OUT(PIN_PWMB, 1, 0);
#ifdef SIM_WOKWI
  simCmdL = simCmdR = 0;
#endif
}
void shortBrake() {   // TB6612: IN1 = IN2 = HIGH -> short-circuit brake
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH);
  PWM_OUT(PIN_PWMA, 0, PWM_MAX); PWM_OUT(PIN_PWMB, 1, PWM_MAX);
#ifdef SIM_WOKWI
  simCmdL = simCmdR = 0;
#endif
}

// =============================== BUZZER / PIN ===============================
#ifdef SIM_WOKWI
void beep(int ms) { tone(PIN_BUZ, 2000, ms); delay(ms); }
#else
void beep(int ms) { digitalWrite(PIN_BUZ, BUZ_ON_LEVEL); delay(ms); digitalWrite(PIN_BUZ, !BUZ_ON_LEVEL); }
#endif
float vbat() { return analogReadMilliVolts(PIN_VBAT) * (133.0f / 33.0f) / 1000.0f; }

// ================================ MPU6050 ===================================
const uint8_t MPU = 0x68;
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
void mpuInit() {
  mpuWrite(0x6B, 0x00);   // leave sleep
  mpuWrite(0x1A, 0x03);   // DLPF ~44 Hz
  mpuWrite(0x1B, 0x18);   // +-2000 dps
}
int16_t gyroZraw() {
  Wire.beginTransmission(MPU); Wire.write(0x47); Wire.endTransmission(false);
  Wire.requestFrom((int)MPU, 2);
  if (Wire.available() < 2) return (int16_t)gyroBias;
  int16_t hi = Wire.read(), lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}
void gyroCalibrate() {           // the robot must be still ~1 s
  long sum = 0;
  for (int i = 0; i < 400; i++) { sum += gyroZraw(); delayMicroseconds(2500); }
  gyroBias = sum / 400.0f;
  yaw = yawRef = 0;
}

// =============================== LINE FOLLOWING =============================
// Read 8 channels, split into black CLUSTERS. 1 cluster = normal line; 2 or more = T-junction
// (honeycomb grid); no cluster = line lost (dashed line / sharp bend); nBlk >= WIDE_N = crossbar/finish box.
void readLine() {
  nBlk = 0; nGrp = 0; onLine = false;
  for (int i = 0; i < 8; i++) {
    int raw  = analogRead(LINE_PINS[i]);
    int span = max(1, (int)calMax[i] - (int)calMin[i]);
    int v = constrain((long)(raw - (int)calMin[i]) * 1000 / span, 0L, 1000L);
    if (!LINE_BLACK_HIGH) v = 1000 - v;
    lv[i] = v;
    blk[i] = v > BLACK_TH;
    if (blk[i]) { nBlk++; onLine = true; }
  }
  int i = 0;
  while (i < 8 && nGrp < 4) {
    if (!blk[i]) { i++; continue; }
    // widen the cluster to the 2 neighboring channels (the outermost channels also add to the weighting)
    int a = i; while (i < 8 && blk[i]) i++;
    int b = i - 1;
    int lo = max(0, a - 1), hi = min(7, b + 1);
    long sw = 0, s = 0;
    for (int k = lo; k <= hi; k++) {
      if (k < a || k > b) { if (lv[k] < 150) continue; }
      sw += (long)lv[k] * k * 1000; s += lv[k];
    }
    grpPos[nGrp++] = (float)sw / max(1L, s) - 3500.0f;
  }
}

// choose the line position to follow (handles honeycomb T-junctions); returns false if the line is lost
bool pickLine(float &err) {
  if (nGrp == 0) return false;
  if (nGrp == 1) { err = grpPos[0]; forkSide = 0; return true; }
  // >= 2 branches: pick the branch that brings the heading back to the OVERALL HEADING (yawRef).
  // Robot drifted left of the overall heading -> pick the right branch, and vice versa.
  if (forkSide == 0 || millis() > forkUntil) {
    float dev = wrap180(yaw - yawRef);
    int s;
    if (dev > 4)       s = +1;
    else if (dev < -4) s = -1;
    else s = (lastErr >= 0) ? +1 : -1;   // nearly straight: pick the branch close to the old line
    if (s != forkSide) nForks++;
    forkSide = s;
    forkUntil = millis() + (uint32_t)P.forkMs;
  }
  err = (forkSide > 0) ? grpPos[nGrp - 1] : grpPos[0];
  return true;
}

void readTof(VL53L0X &t, bool ok, int &dist) {   // non-blocking read
  if (!ok) return;
  if (t.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t d = t.readRangeContinuousMillimeters();
    dist = (d < 20 || d > 2000) ? 8190 : d;
  }
}

// ======================= TICK: one 2 ms control cycle =====================
bool freezeRef = false;
void tick() {
  while ((int32_t)(micros() - tNext) < 0) { }
  tNext += LOOP_US;
  if ((int32_t)(micros() - tNext) > (int32_t)(5 * LOOP_US)) tNext = micros();
  tickCount++;

#ifdef SIM_WOKWI
  float vL = simCmdL * SIM_V_FULL / PWM_MAX, vR = simCmdR * SIM_V_FULL / PWM_MAX;
  float gz = (vR - vL) / SIM_TRACK_MM * 57.2958f;
  if (tickCount % 50 == 0) Serial.printf("err:%.0f grp:%d L:%.0f R:%.0f yaw:%.0f ref:%.0f F:%d\n",
                                         lineErr, nGrp, simCmdL, simCmdR, yaw, yawRef, distF);
#else
  float gz = (gyroZraw() - gyroBias) / GYRO_LSB_PER_DPS;
#endif
  yaw += gz * DT;
  if (!freezeRef) yawRef += (yaw - yawRef) * (DT / YAWREF_TAU);   // overall heading (slow filter)

  readLine();
#ifdef SIM_WOKWI
  if (tickCount % 10 == 0) { distF = simTof(SIM_POT_F); distL = simTof(SIM_POT_L); distR = simTof(SIM_POT_R); }
#else
  readTof(tofF, tofFOk, distF);
  readTof(tofL, tofLOk, distL);
  readTof(tofR, tofROk, distR);
#endif

  if (tickCount % 50 == 0) {               // 10 times/s: filter the battery voltage
    float v = vbat();
    if (v > 5.0f) vbatF = 0.9f * vbatF + 0.1f * v;
    if (v > 3.0f && vbatF < VBAT_CUTOFF) {
      if (!lowSince) lowSince = millis();
      else if (millis() - lowSince > 2000) { lowBattLock = true; abortRun = true; }
    } else lowSince = 0;
  }
  if (digitalRead(PIN_BTN_START) == LOW) abortRun = true;
}

void brakeTicks(int n) {
  for (int i = 0; i < n; i++) { tick(); shortBrake(); }
  coast();
}

// ============================ BASIC CONTROL ==============================
void followStep(float base, float err) {
  float d = err - lastErr;
  lastErr = err;
  lineErr = err;
  float corr = P.kp * err + P.kd * d;
  float b = base * (1.0f - P.slow * fabsf(err) / 3500.0f);   // slows down by itself entering bends
  drive(b + corr, b - corr);
}

// hold heading h with the gyro
void holdHeading(float pwm, float h) {
  float c = 12.0f * wrap180(yaw - h);
  drive(pwm + c, pwm - c);
}

// pivot by an angle (deg), deg > 0 = left
void turnDeg(float deg, float pwm) {
  float y0 = yaw; int s = (deg > 0) ? 1 : -1;
  while (fabsf(yaw - y0) < fabsf(deg) - 4.0f && !abortRun) { tick(); drive(-s * pwm, s * pwm); }
  brakeTicks(15);
}

// pivot until the line is caught again; dir = +1 left, -1 right
void turnToLine(int dir, float blindDeg) {
  float y0 = yaw;
  while (!abortRun) {
    tick();
    drive(-dir * P.turnPwm, dir * P.turnPwm);
    float turned = fabsf(yaw - y0);
    if (turned > blindDeg && (blk[3] || blk[4])) break;
    if (turned > 300) break;
  }
  brakeTicks(10);
  lastErr = 0;
}

// NO ENCODER: distance = time x AvoidSpd (at AvoidPWM, with voltage compensation)
uint32_t msFor(float mm) { return (uint32_t)(mm * 1000.0f / max(50.0f, P.avoidSpd)); }

typedef bool (*StopFn)();
// drive straight holding heading; returns true if stopped by stopIf()
bool straight(float mm, StopFn stopIf) {
  float h = yaw;
  uint32_t t0 = millis(), dur = msFor(mm);
  while (millis() - t0 < dur && !abortRun) {
    tick();
    if (stopIf && stopIf()) return true;
    holdHeading(P.avoidPwm, h);
  }
  return false;
}

// ============================== OBSTACLE AVOIDANCE ==================================
// 2 obstacles 5-10 cm apart < robot length -> do NOT return to the line between them.
// Rectangle around BOTH: only counted as passed when the robot side is clear continuously
// longer than the gap between 2 obstacles (GapMm). avoidDir = +1: go around on the LEFT (obstacle on the robot's right).
int  avoidDir = 1;
int  sideDist()  { return (avoidDir > 0) ? distR : distL; }
bool sideSees()  { return sideDist() < 250; }
bool sideClear() { return sideDist() > 320; }
bool lineFound() { return blk[3] || blk[4] || nBlk >= 2; }

void avoidObstacle() {
  nObst++;
  freezeRef = true;
  if      (P.avoidSide < 0.5f) avoidDir = 1;
  else if (P.avoidSide < 1.5f) avoidDir = -1;
  else                         avoidDir = (distL >= distR) ? 1 : -1;
  int s = avoidDir;
  float pw = P.avoidPwm;
  // emergency brake: short reverse pulse then short-circuit brake
  for (int i = 0; i < 15; i++) { tick(); drive(-PWM_MAX * 0.6f, -PWM_MAX * 0.6f); }
  brakeTicks(40);

  // 1. turn 90 outward, drive sideways until the side sensor no longer sees the obstacle + margin
  turnDeg(90 * s, P.turnPwm);
  straight(350, sideClear);
  straight(80 + P.margin, nullptr);
  turnDeg(-90 * s, P.turnPwm);

  // 2. drive along the 2 obstacles. A short gap (5-10 cm) is skipped, only a gap longer than GapMm ends it.
  straight(250, sideSees);
  float h = yaw;
  uint32_t t0 = millis(), clearSince = 0;
  uint32_t needClear = msFor(P.gapMm + 40), maxT = msFor(900);
  while (!abortRun && millis() - t0 < maxT) {
    tick();
    if (distF < 120) {                       // the 2nd obstacle sticks out further: shift further sideways
      brakeTicks(20);
      turnDeg(90 * s, P.turnPwm); straight(100, nullptr); turnDeg(-90 * s, P.turnPwm);
      h = yaw; clearSince = 0; continue;
    }
    if (sideSees() || !sideClear()) clearSince = 0;
    else if (!clearSince) clearSince = millis();
    else if (millis() - clearSince > needClear) break;
    holdHeading(pw, h);
  }
  straight(P.margin, nullptr);

  // 3. turn into the line, drive until the line is seen, bring the wheel axle to the line then turn to the old heading
  turnDeg(-90 * s, P.turnPwm);
  if (straight(500, lineFound)) {
    straight(SENSOR_TO_AXLE_MM - 20, nullptr);
    turnToLine(s, 45);
  }
  distF = 8190;
  yawRef = yaw;
  freezeRef = false;
}

// ============================== NVS =========================================
void saveParams() { prefs.putBytes("P3", &P, sizeof(P)); }
void saveCal()    { prefs.putBytes("cmin", calMin, sizeof(calMin)); prefs.putBytes("cmax", calMax, sizeof(calMax)); }
void loadAll() {
  if (prefs.getBytesLength("P3") == sizeof(P)) prefs.getBytes("P3", &P, sizeof(P));
  for (int i = 0; i < 8; i++) { calMin[i] = 0; calMax[i] = 4095; }
  if (prefs.getBytesLength("cmin") == sizeof(calMin)) {
    prefs.getBytes("cmin", calMin, sizeof(calMin));
    prefs.getBytes("cmax", calMax, sizeof(calMax));
  }
}

// ============================== CALIBRATION ==================================
void calibrateLine() {
  for (int i = 0; i < 8; i++) { calMin[i] = 4095; calMax[i] = 0; }
  const int phase[3] = {+1, -1, +1};
  const int dur[3]   = {500, 1000, 500};
  for (int p = 0; p < 3; p++) {
    uint32_t t0 = millis();
    while (millis() - t0 < (uint32_t)dur[p]) {
      drive(-phase[p] * P.turnPwm * 0.7f, phase[p] * P.turnPwm * 0.7f);
      for (int i = 0; i < 8; i++) {
        uint16_t r = analogRead(LINE_PINS[i]);
        if (r < calMin[i]) calMin[i] = r;
        if (r > calMax[i]) calMax[i] = r;
      }
      delayMicroseconds(LOOP_US);
    }
  }
  coast();
  bool ok = true;
  for (int i = 0; i < 8; i++) if (calMax[i] - calMin[i] < 300) ok = false;
  saveCal();
  if (ok) beep(150); else { beep(80); delay(80); beep(80); delay(80); beep(80); }
  Serial.print("CAL min/max: ");
  for (int i = 0; i < 8; i++) Serial.printf("%u/%u ", calMin[i], calMax[i]);
  Serial.println(ok ? " OK" : " ERROR: some channels differ < 300");
}

// SPEED mode: drive straight 2 s at AvoidPWM -> measure the distance with a ruler, divide by 2 = AvoidSpd
void speedTest() {
  abortRun = false; vbatF = max(5.0f, vbat()); tNext = micros();
  uint32_t t0 = millis();
  while (millis() - t0 < 2000 && !abortRun) { tick(); holdHeading(P.avoidPwm, 0); }
  brakeTicks(40);
  beep(200);
  Serial.println("SPEED: measure distance (mm), divide by 2 -> enter as AvoidSpd");
  while (digitalRead(PIN_BTN_START) == LOW) delay(10);
}

// ================================ RUN ======================================
void runMode(int m) {
  float base = (m == M_FAST) ? P.fast : P.base;
  float obstTh = P.obstMm * max(1.0f, base / P.base);    // faster -> sees the obstacle earlier
  int obstHits = 0;
  uint32_t lostT = 0, wideT = 0;
  float lostErr = 0, lostHdg = 0, wideHdg = 0;
  bool finished = false;

  abortRun = false; lastErr = 0; forkSide = 0;
  nForks = nDash = nWide = nObst = 0;
  vbatF = max(5.0f, vbat());
  tNext = micros();
  uint32_t t0 = millis();

  while (true) {
    tick();
    if (lowBattLock) break;
    if (abortRun && millis() - t0 > 800) break;
    abortRun = false;
    uint32_t now = millis();

    // --- obstacle (2 consecutive reads against noise) ---
    obstHits = (distF < obstTh) ? obstHits + 1 : 0;
    if (obstHits >= 2) { avoidObstacle(); obstHits = 0; lostT = wideT = 0; continue; }

    // --- wide black area: crossbar / honeycomb apex (short) or FINISH BOX (long) ---
    if (nBlk >= WIDE_N) {
      if (!wideT) { wideT = now; wideHdg = yaw; nWide++; }
      if (now - t0 > 1000 && now - wideT > (uint32_t)P.finMs) { finished = true; break; }  // skip the start box
      holdHeading(base * 0.8f, wideHdg);       // drive straight over the crossbar without being pulled off
      lostT = 0;
      continue;
    }
    wideT = 0;

    // --- normal / honeycomb T-junction ---
    float err;
    if (pickLine(err)) { lostT = 0; followStep(base, err); continue; }

    // --- line lost ---
    if (!lostT) { lostT = now; lostErr = lastErr; lostHdg = yaw; }
    if (fabsf(lostErr) < 1800 && now - lostT < (uint32_t)P.dashMs) {
      if (now - lostT < 4) nDash++;
      holdHeading(base * 0.8f, lostHdg);       // DASHED LINE: drive straight holding heading
    } else {
      // SHARP BEND / past a dashed gap with no line: turn toward the side the line was lost
      followStep(base * 0.6f, (lostErr >= 0) ? 3500.0f : -3500.0f);
      if (now - lostT > 2500) break;           // line lost for too long: stop so the robot does not run off the track
    }
  }

  brakeTicks(40);
  coast();
  if (finished) { beep(100); delay(80); beep(300); }
  else          { beep(500); }
  Serial.printf("Finished %s: %s, %lu ms | T-junction:%lu dashed:%lu wide-bar:%lu obstacle:%lu\n",
                MODE_NAME[m], finished ? "FINISH" : "STOP", (unsigned long)(millis() - t0),
                (unsigned long)nForks, (unsigned long)nDash, (unsigned long)nWide, (unsigned long)nObst);
  while (digitalRead(PIN_BTN_START) == LOW) delay(10);
}

// ================================ BUTTONS ========================================
struct Btn { uint8_t pin; bool last; uint32_t tDown; };
Btn bStart = {PIN_BTN_START, true, 0}, bMode = {PIN_BTN_MODE, true, 0}, bSet = {PIN_BTN_SET, true, 0};
int readBtn(Btn &b) {               // 0 = none, 1 = short press, 2 = long press
  bool s = digitalRead(b.pin);
  int ev = 0;
  if (b.last && !s) b.tDown = millis();
  if (!b.last && s) {
    uint32_t dt = millis() - b.tDown;
    if (dt > 600) ev = 2; else if (dt > 30) ev = 1;
  }
  b.last = s;
  return ev;
}

// ================================ OLED =======================================
void drawMenu() {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  if (lowBattLock) oled.printf("PIN CAN %.2fV-SAC!", vbat());
  else             oled.printf("ROBOT v0.3 %.2fV", vbat());
  oled.setCursor(0, 10); oled.printf("%cMode: %s", page == 0 ? '>' : ' ', MODE_NAME[mode]);
  if (page > 0) { oled.setCursor(0, 20); oled.printf(">%s = %.2f", MENU[page - 1].name, *MENU[page - 1].v); }
  readLine();
  oled.setCursor(0, 30); oled.printf("Clu:%d Blk:%d", nGrp, nBlk);
  oled.setCursor(0, 40); oled.printf("F%4d L%4d R%4d", distF, distL, distR);
  for (int i = 0; i < 8; i++) {
    int h = lv[i] * 12 / 1000;
    oled.fillRect(i * 16 + 2, 63 - h, 12, h, SSD1306_WHITE);
  }
  oled.display();
}

// ================================ SETUP ======================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_MODE,  INPUT_PULLUP);
  pinMode(PIN_BTN_SET,   INPUT_PULLUP);
  pinMode(PIN_BUZ, OUTPUT); digitalWrite(PIN_BUZ, !BUZ_ON_LEVEL);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  PWM_ATTACH(PIN_PWMA, 0);
  PWM_ATTACH(PIN_PWMB, 1);
  coast();
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  // 3 VL53L0X sharing address 0x29: hold left + right in reset, change addresses one by one
  pinMode(PIN_XSHUT_R, OUTPUT); digitalWrite(PIN_XSHUT_R, LOW);
  pinMode(PIN_XSHUT_L, OUTPUT); digitalWrite(PIN_XSHUT_L, LOW);
  delay(10);
  tofF.setTimeout(50);
  if (tofF.init()) { tofF.setAddress(0x30); tofFOk = true; }
  pinMode(PIN_XSHUT_R, INPUT); delay(10);
  tofR.setTimeout(50);
  if (tofR.init()) { tofR.setAddress(0x31); tofROk = true; }
  pinMode(PIN_XSHUT_L, INPUT); delay(10);
  tofL.setTimeout(50);
  if (tofL.init()) tofLOk = true;
  if (tofFOk) { tofF.setMeasurementTimingBudget(20000); tofF.startContinuous(0); }
  if (tofLOk) { tofL.setMeasurementTimingBudget(20000); tofL.startContinuous(0); }
  if (tofROk) { tofR.setMeasurementTimingBudget(20000); tofR.startContinuous(0); }

  mpuInit();
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  prefs.begin("fmaze", false);   // NVS name kept from the first firmware so data already stored on the robot survives updates
  loadAll();
  Serial.printf("TOF F:%d L:%d R:%d OLED:%d  Vbat=%.2f\n", tofFOk, tofLOk, tofROk, oledOk, vbat());
  beep(60);
}

// ================================ LOOP =======================================
void loop() {
  static uint32_t tDraw = 0;
#ifdef SIM_WOKWI
  distF = simTof(SIM_POT_F); distL = simTof(SIM_POT_L); distR = simTof(SIM_POT_R);
#else
  readTof(tofF, tofFOk, distF);
  readTof(tofL, tofLOk, distL);
  readTof(tofR, tofROk, distR);
#endif
  int eM = readBtn(bMode), eS = readBtn(bSet), eG = readBtn(bStart);
  if (eM == 1) page = (page + 1) % (N_MENU + 1);
  if (eS) {
    if (page == 0) { if (eS == 1) mode = (mode + 1) % M_COUNT; }
    else {
      MenuItem &it = MENU[page - 1];
      *it.v = constrain(*it.v + (eS == 1 ? it.step : -it.step), it.lo, it.hi);
      saveParams();
    }
  }
  if (eG == 1 && (lowBattLock || (vbat() > 3.0f && vbat() < VBAT_MIN_START))) {
    lowBattLock = true; eG = 0; beep(600);
  }
  if (eG == 1) {
    beep(40); delay(150);
    gyroCalibrate();
    beep(40);
    if      (mode == M_CALIB) calibrateLine();
    else if (mode == M_SPEED) speedTest();
    else                      runMode(mode);
  }
  float v = vbat();
  if (v > 3.0f && v < VBAT_CUTOFF) {
    if (!lowSince) lowSince = millis();
    else if (millis() - lowSince > 2000) lowBattLock = true;
  } else if (!lowBattLock) lowSince = 0;
  if (lowBattLock) { coast(); if (millis() % 1000 < 20) beep(150); }
  else if (v < VBAT_WARN && v > 3.0f && millis() % 5000 < 20) beep(30);

  if (millis() - tDraw > 100) { tDraw = millis(); drawMenu(); }
}
