/*
  ============================================================================
  LINE ROBOT  -  Firmware skeleton (v0.2)
  Board : ESP32-S3 DevKitC-1 (N16R8)
  IDE   : Arduino IDE 2.x + "esp32 by Espressif Systems" core 3.x
          Tools > Board: "ESP32S3 Dev Module"
                  USB CDC On Boot: Enabled (Serial runs over the native USB port)
                  Flash Size: 16MB, PSRAM: OPI PSRAM
  Libraries (Library Manager):
          - VL53L0X  (by Pololu)
          - Adafruit SSD1306 + Adafruit GFX Library
  ----------------------------------------------------------------------------
  v0.2: 3 x VL53L0X (front/left/right), obstacle avoidance choosing the side, maze map by
        odometry (handles dead ends + CLOSED LOOPS), Dijkstra finds the shortest path,
        battery voltage compensation (batteries must not be swapped between runs).
  ----------------------------------------------------------------------------
  Modes (selected with the MODE/SET buttons, see the OLED):
    FOLLOW  : line following + obstacle avoidance, straight through at crossroads
    EXPLORE : 3-minute trial run - builds the map, finds the shortest path, saves to NVS
    SAFE    : replay the saved path at Base speed
    FAST    : replay the saved path at the Fast speed
    CALIB   : calibrate the line sensor bar (robot sweeps back and forth over the line)
  Buttons:
    MODE short : next page (page 0 = mode select, later pages = parameters)
    SET  short : increase value / change mode      SET hold (>0.6s): decrease value
    START     : run the selected mode (countdown ~1 s, keep the robot still for gyro calibration)
                While running, press START again to stop.
  EVERY PARAMETER IS SAVED IN NVS -> tune at the track without re-flashing the code.
  RULE: no corner cutting -> the robot always follows the line, leaves it only to avoid obstacles.
  ============================================================================
*/
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================== PINS ========================================
static const uint8_t LINE_PINS[8] = {1, 2, 3, 4, 5, 6, 7, 8};   // S1 (left) -> S8 (right), ADC1
#define PIN_VBAT       9
#define PIN_BTN_START 10
#define PIN_BTN_MODE   0     // BOOT button on the board
#define PIN_BTN_SET   11
#define PIN_SDA       17
#define PIN_SCL       18
#define PIN_XSHUT_R   21     // right VL53L0X XSHUT
#define PIN_XSHUT_L   48     // left VL53L0X XSHUT (RGB LED pin on the board, idle level -> LED stays off)
#define PIN_BUZ       47
#define PIN_PWMA      38     // left motor
#define PIN_AIN1      39
#define PIN_AIN2      40
#define PIN_PWMB      41     // right motor
#define PIN_BIN1      42
#define PIN_BIN2      16
#define PIN_ENC_LA    12
#define PIN_ENC_LB    13
#define PIN_ENC_RA    14
#define PIN_ENC_RB    15

// ============================ CONSTANTS (MEASURE AGAIN!) =============================
const bool  LINE_BLACK_HIGH   = true;    // line sensor bar: does the black line give a HIGH voltage? if wrong set false
const float WHEEL_D_MM        = 34.0f;
const float TICKS_PER_REV     = 420.0f;  // pulses per WHEEL turn (measure: turn by hand 10 turns, divide by 10)
const float MM_PER_TICK       = PI * WHEEL_D_MM / TICKS_PER_REV;
const float SENSOR_TO_AXLE_MM = 120.0f;  // anchor-shaped chassis: line sensor bar -> wheel axle (MEASURE AGAIN)
const float GYRO_LSB_PER_DPS  = 16.4f;   // MPU6050 at +-2000 dps range
const float VNOM              = 7.8f;    // menu PWMs are set at this battery level
const float D2R               = 0.01745329f;
const int   PWM_FREQ = 20000;
const int   PWM_BITS = 10;
const int   PWM_MAX  = (1 << PWM_BITS) - 1;   // 1023
const uint32_t LOOP_US = 2000;                // 500 Hz control loop
const int   BLACK_TH = 600;                   // "sees black" threshold after normalizing 0..1000

// ======================= TUNABLE PARAMETERS (NVS) ===========================
struct Params {
  float kp;        // he so P
  float kd;        // he so D
  float base;      // base PWM in SAFE / FOLLOW mode (at 7.8 V)
  float fast;      // base PWM in FAST mode
  float turnPwm;   // PWM while pivoting in place
  float obstMm;    // obstacle detection threshold (mm) at Base speed
  float margin;    // extra mm of overshoot when going around an obstacle
  float avoidSide; // 0 = go around left, 1 = go around right, 2 = choose the more inner side
  float rule;      // EXPLORE: 0 = prefer left branch, 1 = prefer right branch
  float expTime;   // max seconds for the EXPLORE run (rule: 3 minutes)
  float expAll;    // 0 = stop at the finish, 1 = keep measuring the rest until time is up
  float nodeTol;   // mm: 2 intersections closer than nodeTol are treated as one (loop detection)
  float turnCost;  // mm: penalty per intersection when computing the shortest path
};
Params P = {0.10f, 1.0f, 450, 700, 380, 150, 60, 0, 0, 160, 0, 120, 120};

struct MenuItem { const char* name; float* v; float step; float lo; float hi; };
MenuItem MENU[] = {
  {"Kp",       &P.kp,        0.01f, 0.0f, 2.0f},
  {"Kd",       &P.kd,        0.10f, 0.0f, 20.0f},
  {"Base",     &P.base,      25,    100,  1023},
  {"Fast",     &P.fast,      25,    100,  1023},
  {"TurnPWM",  &P.turnPwm,   20,    100,  1023},
  {"ObstMm",   &P.obstMm,    10,    60,   400},
  {"Margin",   &P.margin,    10,    0,    200},
  {"Side0L1R2A",&P.avoidSide,1,     0,    2},
  {"Rule0L1R", &P.rule,      1,     0,    1},
  {"ExpTime s",&P.expTime,   10,    30,   175},
  {"ExpAll",   &P.expAll,    1,     0,    1},
  {"NodeTol",  &P.nodeTol,   10,    40,   300},
  {"TurnCost", &P.turnCost,  20,    0,    500},
};
const int N_MENU = sizeof(MENU) / sizeof(MENU[0]);

enum Mode { M_FOLLOW, M_EXPLORE, M_SAFE, M_FAST, M_CALIB, M_COUNT };
const char* MODE_NAME[] = {"FOLLOW", "EXPLORE", "SAFE", "FAST", "CALIB"};
int mode = M_FOLLOW;
int page = 0;

// ============================== OBJECTS ===================================
Preferences prefs;
VL53L0X tofF, tofL, tofR;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false, tofFOk = false, tofLOk = false, tofROk = false;

// ============================== STATE ==================================
uint16_t calMin[8], calMax[8];
int   lv[8];          // normalized value 0..1000 (1000 = black)
bool  blk[8];         // true = the sensor sees black
int   nBlk = 0;
bool  onLine = false;
float lineErr = 0, lastErr = 0;   // line position error: -3500 (left) .. +3500 (right)

volatile int32_t encL = 0, encR = 0;
float yaw = 0, gyroBias = 0;      // deg, positive = turning left (CCW)
float odoX = 0, odoY = 0, odoPrevMm = 0;
int   distF = 8190, distL = 8190, distR = 8190; // mm
float vbatF = VNOM;
uint32_t tNext = 0, tickCount = 0;
bool  abortRun = false;

char  path[256];
int   pathLen = 0;

// ================================ ENCODER ===================================
void IRAM_ATTR isrEncL() {
  bool a = digitalRead(PIN_ENC_LA), b = digitalRead(PIN_ENC_LB);
  encL += (a == b) ? 1 : -1;        // counting backward means the sign is flipped
}
void IRAM_ATTR isrEncR() {
  bool a = digitalRead(PIN_ENC_RA), b = digitalRead(PIN_ENC_RB);
  encR += (a == b) ? -1 : 1;        // right motor mounted reversed so flip the sign
}
float distMm() { return (encL + encR) * 0.5f * MM_PER_TICK; }

// ================================= MOTOR ====================================
void motorOne(uint8_t pwmPin, uint8_t in1, uint8_t in2, int v) {
  v = constrain(v, -PWM_MAX, PWM_MAX);
  if (v >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else        { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); v = -v; }
  ledcWrite(pwmPin, v);
}
// voltage compensation: battery drops from 8.4 V to 7.4 V, PWM rises automatically -> speed stays nearly the same
void drive(float l, float r) {
  float k = constrain(VNOM / vbatF, 0.85f, 1.25f);
  motorOne(PIN_PWMA, PIN_AIN1, PIN_AIN2, (int)(l * k));
  motorOne(PIN_PWMB, PIN_BIN1, PIN_BIN2, (int)(r * k));
}
void coast() { ledcWrite(PIN_PWMA, 0); ledcWrite(PIN_PWMB, 0); }
void shortBrake() {   // TB6612: IN1 = IN2 = HIGH -> short-circuit brake
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH);
  ledcWrite(PIN_PWMA, PWM_MAX); ledcWrite(PIN_PWMB, PWM_MAX);
}

// =============================== BUZZER / PIN ===============================
void beep(int ms) { digitalWrite(PIN_BUZ, HIGH); delay(ms); digitalWrite(PIN_BUZ, LOW); }
float vbat() { return analogReadMilliVolts(PIN_VBAT) * (133.0f / 33.0f) / 1000.0f; }

// ================================ MPU6050 ===================================
const uint8_t MPU = 0x68;
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
void mpuInit() {
  mpuWrite(0x6B, 0x00);   // leave sleep
  mpuWrite(0x1A, 0x03);   // DLPF ~44 Hz
  mpuWrite(0x1B, 0x18);   // gyro +-2000 dps
}
int16_t gyroZraw() {
  Wire.beginTransmission(MPU); Wire.write(0x47); Wire.endTransmission(false);
  Wire.requestFrom((int)MPU, 2);
  if (Wire.available() < 2) return (int16_t)gyroBias;
  int16_t hi = Wire.read(), lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}
void gyroCalibrate() {           // the robot must be still
  long sum = 0;
  for (int i = 0; i < 400; i++) { sum += gyroZraw(); delayMicroseconds(2500); }
  gyroBias = sum / 400.0f;
  yaw = 0;
}

// =============================== LINE FOLLOWING =============================
void readLine() {
  long sw = 0, s = 0;
  nBlk = 0; onLine = false;
  for (int i = 0; i < 8; i++) {
    int raw  = analogRead(LINE_PINS[i]);
    int span = max(1, (int)calMax[i] - (int)calMin[i]);
    int v = (long)(raw - (int)calMin[i]) * 1000 / span;
    v = constrain(v, 0, 1000);
    if (!LINE_BLACK_HIGH) v = 1000 - v;
    lv[i] = v;
    blk[i] = v > BLACK_TH;
    if (blk[i]) { nBlk++; onLine = true; }
    if (v > 150) { sw += (long)v * i * 1000; s += v; }
  }
  if (onLine && s > 0) lineErr = (float)sw / s - 3500.0f;
  else                 lineErr = (lastErr > 0) ? 3500.0f : -3500.0f;  // line lost: steer back toward the old side
}

// non-blocking VL53L0X read: only read when a new result exists
void readTof(VL53L0X &t, bool ok, int &dist) {
  if (!ok) return;
  if (t.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t d = t.readRangeContinuousMillimeters();
    dist = (d < 20 || d > 2000) ? 8190 : d;
  }
}

// ======================= TICK: one 2 ms control cycle =====================
void tick() {
  while ((int32_t)(micros() - tNext) < 0) { }
  tNext += LOOP_US;
  if ((int32_t)(micros() - tNext) > (int32_t)(5 * LOOP_US)) tNext = micros();  // latency too large -> reset
  tickCount++;

  float gz = (gyroZraw() - gyroBias) / GYRO_LSB_PER_DPS;
  yaw += gz * (LOOP_US * 1e-6f);

  // odometry: position (mm) in the start coordinate frame
  float dm = distMm(), dd = dm - odoPrevMm;
  odoPrevMm = dm;
  odoX += dd * cosf(yaw * D2R);
  odoY += dd * sinf(yaw * D2R);

  readLine();
  readTof(tofF, tofFOk, distF);
  readTof(tofL, tofLOk, distL);
  readTof(tofR, tofROk, distR);

  if (tickCount % 50 == 0) {               // 10 times/s: filter the battery voltage
    float v = vbat();
    if (v > 5.0f) vbatF = 0.9f * vbatF + 0.1f * v;
  }
  if (digitalRead(PIN_BTN_START) == LOW) abortRun = true;
}

void brakeTicks(int n) {
  for (int i = 0; i < n; i++) { tick(); shortBrake(); }
  coast();
}

// ============================ BASIC CONTROL ==============================
void followStep(float base) {
  float d = lineErr - lastErr;
  lastErr = lineErr;
  float corr = P.kp * lineErr + P.kd * d;
  float b = base * (1.0f - 0.45f * fabsf(lineErr) / 3500.0f);   // slows down by itself entering bends
  drive(b + corr, b - corr);
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
    if (turned > 300) break;                 // safe
  }
  brakeTicks(10);
  lastErr = 0;
}

typedef bool (*StopFn)();
// drive straight holding heading with the gyro; returns true if stopped by stopIf()
bool straight(float mm, float pwm, StopFn stopIf) {
  float d0 = distMm(), h = yaw;
  while (fabsf(distMm() - d0) < mm && !abortRun) {
    tick();
    if (stopIf && stopIf()) return true;
    float c = 12.0f * (yaw - h);
    drive(pwm + c, pwm - c);
  }
  return false;
}

// ============================== OBSTACLE AVOIDANCE ==================================
// Rectangle around the box. avoidDir = +1: go around on the LEFT (box on the robot's right,
// right VL53L0X follows the box edge); -1: go around on the RIGHT (left VL53L0X follows the box edge).
int  avoidDir = 1;
int  sideDist()  { return (avoidDir > 0) ? distR : distL; }
bool sideSees()  { return sideDist() < 250; }
bool sideClear() { return sideDist() > 320; }
bool lineFound() { return blk[3] || blk[4] || nBlk >= 2; }

void avoidObstacle() {
  if      (P.avoidSide < 0.5f) avoidDir = 1;
  else if (P.avoidSide < 1.5f) avoidDir = -1;
  else                         avoidDir = (distL >= distR) ? 1 : -1;   // go around on whichever side is more inside
  int s = avoidDir;
  float pw = P.base * 0.7f;
  brakeTicks(40);
  turnDeg(90 * s, P.turnPwm);                         // 1. turn 90 toward the go-around side
  straight(350, pw, sideClear);                       // 2. drive sideways until the box is gone
  straight(80 + P.margin, pw, nullptr);               //    + half the robot body + margin
  turnDeg(-90 * s, P.turnPwm);                        // 3. turn back, drive along the box edge
  straight(250, pw, sideSees);                        //    until the box edge is seen
  straight(250, pw, sideClear);                       //    to the end of the box edge
  straight(60 + P.margin, pw, nullptr);
  turnDeg(-90 * s, P.turnPwm);                        // 4. turn toward the line
  if (straight(500, pw, lineFound)) {
    straight(SENSOR_TO_AXLE_MM - 20, pw * 0.8f, nullptr);  // bring the wheel axle onto the line
    turnToLine(s, 45);                                // 5. turn to catch the line, continue on the old heading
  }
  distF = 8190;
}

// ============================ INTERSECTION ======================================
bool isNode() {
  bool side = (blk[0] && blk[1]) || (blk[6] && blk[7]);
  return side && (blk[3] || blk[4]) && nBlk >= 4;
}

// drive forward another SENSOR_TO_AXLE_MM so the wheel axle is at the middle of the intersection, scan the branches
void scanNode(bool &L, bool &S, bool &R, bool &fin) {
  L = R = false;
  float d0 = distMm(), h = yaw, pw = P.base * 0.6f;
  float allBlackMm = 0, dPrev = d0;
  while (distMm() - d0 < SENSOR_TO_AXLE_MM && !abortRun) {
    tick();
    L |= (blk[0] && blk[1]);
    R |= (blk[6] && blk[7]);
    float dn = distMm();
    if (nBlk >= 7) allBlackMm += dn - dPrev;
    dPrev = dn;
    float c = 12.0f * (yaw - h);
    drive(pw + c, pw - c);
  }
  S   = blk[2] || blk[3] || blk[4] || blk[5];
  fin = (nBlk >= 7) && (allBlackMm > SENSOR_TO_AXLE_MM * 0.8f);  // solid black finish box
}

char decide(bool L, bool S, bool R) {
  if (P.rule < 0.5f) { if (L) return 'L'; if (S) return 'S'; if (R) return 'R'; }
  else               { if (R) return 'R'; if (S) return 'S'; if (L) return 'L'; }
  return 'B';
}

void execTurn(char c) {
  switch (c) {
    case 'L': turnToLine(+1, 50);  break;
    case 'R': turnToLine(-1, 50);  break;
    case 'B': turnToLine(+1, 140); break;
    default: break;   // 'S'
  }
}

float wrap180(float a) { while (a > 180) a -= 360; while (a <= -180) a += 360; return a; }
char relChar(float rel) {           // exit angle relative to the current heading -> turn command
  rel = wrap180(rel);
  if (fabsf(rel) > 135) return 'B';
  if (rel > 45)  return 'L';
  if (rel < -45) return 'R';
  return 'S';
}

// ===================== MAZE MAP (dead ends + closed loops) =====================
// Every intersection = 1 node with coordinates (odometry). Every branch = 1 "exit" with an absolute heading.
// Reached a known intersection (within nodeTol) => loop detected, snap odometry to the node coordinates.
// Explore: if the current node still has an untried exit, take it; otherwise Dijkstra to the nearest node with an untried exit.
const int MAX_NODES = 64;
struct Exit { int16_t hdg; int8_t to; int8_t toExit; float len; };   // to = -1: not driven yet
struct Node { float x, y; uint8_t nEx; bool dead; Exit ex[4]; };
Node G[MAX_NODES];
int nNodes = 0, finishNode = -1;
int curNode = -1, curExit = -1;    // edge being driven: leaving curNode through exit curExit
float edgeStartMm = 0;

int findExit(int n, float hdg, float tol) {
  int best = -1; float bd = 1e9;
  for (int i = 0; i < G[n].nEx; i++) {
    float d = fabsf(wrap180(G[n].ex[i].hdg - hdg));
    if (d < bd) { bd = d; best = i; }
  }
  return (bd <= tol) ? best : -1;
}
int addExit(int n, float hdg) {
  int k = findExit(n, hdg, 35);
  if (k >= 0) return k;
  if (G[n].nEx >= 4) return findExit(n, hdg, 180);
  Exit &e = G[n].ex[G[n].nEx];
  e.hdg = (int16_t)wrap180(hdg); e.to = -1; e.toExit = -1; e.len = 0;
  return G[n].nEx++;
}
int newNode(float x, float y) {
  if (nNodes >= MAX_NODES) return -1;
  Node &n = G[nNodes];
  n.x = x; n.y = y; n.nEx = 0; n.dead = false;
  return nNodes++;
}
int matchNode(float x, float y, float edgeLen) {
  int best = -1; float bd = P.nodeTol;
  for (int i = 0; i < nNodes; i++) {
    if (i == curNode && edgeLen < 2 * P.nodeTol) continue;   // just left this node
    float d = hypotf(G[i].x - x, G[i].y - y);
    if (d < bd) { bd = d; best = i; }
  }
  return best;
}

// called when the wheel axle is at the middle of an intersection (or at a dead end / inside the finish box)
int arriveNode(bool L, bool S, bool R, bool endPoint) {
  float hIn = wrap180(yaw);
  float len = distMm() - edgeStartMm;
  int n = matchNode(odoX, odoY, len);
  if (n < 0) {
    n = newNode(odoX, odoY);
    if (n < 0) return -1;                         // map memory full
    G[n].dead = endPoint;
    if (!endPoint) {
      if (L) addExit(n, hIn + 90);
      if (S) addExit(n, hIn);
      if (R) addExit(n, hIn - 90);
    }
  } else {
    odoX = G[n].x; odoY = G[n].y;               // closed loop: clear the accumulated error
  }
  int back = addExit(n, hIn + 180);             // exit just entered
  if (curNode >= 0 && curExit >= 0) {
    Exit &a = G[curNode].ex[curExit]; a.to = n;       a.toExit = back;    a.len = len;
    Exit &b = G[n].ex[back];          b.to = curNode; b.toExit = curExit; b.len = len;
  }
  return n;
}

float dj[MAX_NODES]; int pv[MAX_NODES]; int pvEx[MAX_NODES];   // pvEx: exit at node pv leading to this node
void dijkstra(int src) {
  bool done[MAX_NODES];
  for (int i = 0; i < nNodes; i++) { dj[i] = 1e9; pv[i] = -1; pvEx[i] = -1; done[i] = false; }
  dj[src] = 0;
  for (int it = 0; it < nNodes; it++) {
    int u = -1;
    for (int i = 0; i < nNodes; i++) if (!done[i] && (u < 0 || dj[i] < dj[u])) u = i;
    if (u < 0 || dj[u] >= 1e9) break;
    done[u] = true;
    for (int k = 0; k < G[u].nEx; k++) {
      const Exit &e = G[u].ex[k];
      if (e.to < 0) continue;
      float w = e.len + P.turnCost;
      if (dj[u] + w < dj[e.to]) { dj[e.to] = dj[u] + w; pv[e.to] = u; pvEx[e.to] = k; }
    }
  }
}
bool hasOpenExit(int n) {
  if (G[n].dead || n == finishNode) return false;
  for (int k = 0; k < G[n].nEx; k++) if (G[n].ex[k].to == -1) return true;
  return false;
}
// choose the exit to take while exploring; -1 = whole map explored
int pickExit(int n, float hIn) {
  int best = -1, bestRank = 99;
  if (hasOpenExit(n)) {
    for (int k = 0; k < G[n].nEx; k++) {
      if (G[n].ex[k].to != -1) continue;
      float rel = wrap180(G[n].ex[k].hdg - hIn);
      int rank = (fabsf(rel) <= 45) ? 1 : ((rel > 0) == (P.rule < 0.5f) ? 0 : 2);
      if (fabsf(rel) > 135) rank = 3;
      if (rank < bestRank) { bestRank = rank; best = k; }
    }
    return best;
  }
  dijkstra(n);
  int t = -1;
  for (int i = 0; i < nNodes; i++) if (hasOpenExit(i) && dj[i] < 1e9 && (t < 0 || dj[i] < dj[t])) t = i;
  if (t < 0) return -1;
  while (pv[t] != n) t = pv[t];
  return pvEx[t];
}

// shortest path start(0) -> finish, converted into a command sequence at each intersection
void buildPath() {
  pathLen = 0;
  if (finishNode < 0) return;
  dijkstra(0);
  if (dj[finishNode] >= 1e9) return;
  int seq[MAX_NODES], m = 0;
  for (int v = finishNode; v >= 0 && m < MAX_NODES; v = pv[v]) seq[m++] = v;   // reversed
  for (int i = m - 2; i >= 1; i--) {             // drop start (seq[m-1]) and finish (seq[0])
    int j = seq[i], prev = seq[i + 1], next = seq[i - 1];
    int back  = G[prev].ex[pvEx[j]].toExit;      // exit entering node j
    int leave = pvEx[next];                      // exit leaving node j
    float hIn = G[j].ex[back].hdg + 180;
    if (pathLen < 255) path[pathLen++] = relChar(G[j].ex[leave].hdg - hIn);
  }
  path[pathLen] = 0;
}

void printMap() {
  Serial.printf("MAP: %d nodes, finish = %d\n", nNodes, finishNode);
  for (int i = 0; i < nNodes; i++) {
    Serial.printf(" N%d (%.0f,%.0f)%s:", i, G[i].x, G[i].y, G[i].dead ? " cut" : "");
    for (int k = 0; k < G[i].nEx; k++) Serial.printf(" [%d deg -> %d, %.0fmm]", G[i].ex[k].hdg, G[i].ex[k].to, G[i].ex[k].len);
    Serial.println("");
  }
}

// ============================== NVS =========================================
void saveParams() { prefs.putBytes("P", &P, sizeof(P)); }
void saveCal()    { prefs.putBytes("cmin", calMin, sizeof(calMin)); prefs.putBytes("cmax", calMax, sizeof(calMax)); }
void savePath()   { path[pathLen] = 0; prefs.putString("path", path); }
void loadAll() {
  if (prefs.getBytesLength("P") == sizeof(P)) prefs.getBytes("P", &P, sizeof(P));
  for (int i = 0; i < 8; i++) { calMin[i] = 0; calMax[i] = 4095; }
  if (prefs.getBytesLength("cmin") == sizeof(calMin)) {
    prefs.getBytes("cmin", calMin, sizeof(calMin));
    prefs.getBytes("cmax", calMax, sizeof(calMax));
  }
  String s = prefs.getString("path", "");
  pathLen = min((int)s.length(), 255);
  memcpy(path, s.c_str(), pathLen);
  path[pathLen] = 0;
}

// ============================== CALIBRATION ==================================
void calibrateLine() {
  for (int i = 0; i < 8; i++) { calMin[i] = 4095; calMax[i] = 0; }
  const int phase[3] = {+1, -1, +1};
  const int dur[3]   = {500, 1000, 500};          // ms
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
  for (int i = 0; i < 8; i++) { Serial.printf("%u/%u ", calMin[i], calMax[i]); }
  Serial.println(ok ? " OK" : " ERROR: some channels differ < 300");
}

// ================================ RUN ======================================
void runMode(int m) {
  float base = (m == M_EXPLORE) ? P.base * 0.7f : (m == M_FAST ? P.fast : P.base);
  float obstTh = P.obstMm * max(1.0f, base / P.base);    // faster -> detects earlier
  int idx = 0, obstHits = 0;
  float lostStart = -1;

  abortRun = false;
  encL = encR = 0; lastErr = 0;
  odoX = odoY = odoPrevMm = 0;
  vbatF = max(5.0f, vbat());
  if (m == M_EXPLORE) {                         // start point = node 0, exit 0 = straight ahead
    nNodes = 0; finishNode = -1; pathLen = 0;
    curNode = newNode(0, 0); curExit = addExit(curNode, 0); edgeStartMm = 0;
  }
  tNext = micros();
  uint32_t t0 = millis();
  bool finished = false;

  while (true) {
    tick();
    if (abortRun && millis() - t0 > 800) break;      // press START to stop
    abortRun = false;
    if (m == M_EXPLORE && millis() - t0 > (uint32_t)(P.expTime * 1000)) break;   // measurement time is up

    // --- obstacle (needs 2 consecutive reads against noise) ---
    obstHits = (distF < obstTh) ? obstHits + 1 : 0;
    if (obstHits >= 2) { avoidObstacle(); obstHits = 0; continue; }

    // --- intersection ---
    bool node = isNode(), deadEnd = false;
    if (!node && m == M_EXPLORE) {                   // dead end: line lost > 40 mm
      if (onLine) lostStart = -1;
      else if (lostStart < 0) lostStart = distMm();
      else if (distMm() - lostStart > 40) deadEnd = true;
    }
    if (node || deadEnd) {
      bool L = false, S = false, R = false, fin = false;
      if (node) scanNode(L, S, R, fin);
      lostStart = -1;

      if (m == M_EXPLORE) {
        int n = arriveNode(L, S, R, deadEnd || fin);
        if (n < 0) break;                               // map memory full
        if (fin) {
          finishNode = n;
          if (P.expAll < 0.5f) { finished = true; break; }
        }
        int k = pickExit(n, wrap180(yaw));
        if (k < 0) { finished = (finishNode >= 0); break; }   // whole map explored
        execTurn(relChar(G[n].ex[k].hdg - yaw));
        curNode = n; curExit = k; edgeStartMm = distMm();
        continue;
      }

      if (fin) { finished = true; break; }
      char c;
      if (m == M_FOLLOW) c = S ? 'S' : (L ? 'L' : (R ? 'R' : 'B'));
      else               c = (idx < pathLen) ? path[idx++] : decide(L, S, R);
      execTurn(c);
      continue;
    }

    followStep(base);
  }

  brakeTicks(40);
  coast();
  if (m == M_EXPLORE) {
    printMap();
    buildPath();
    savePath();
    finished = (finishNode >= 0) && (dj[finishNode] < 1e9);   // a path to the finish exists
  }
  if (finished) { beep(100); delay(80); beep(300); }
  else          { beep(500); }
  Serial.printf("Finished %s, %lu ms, path=%s\n", MODE_NAME[m], (unsigned long)(millis() - t0), path);
  while (digitalRead(PIN_BTN_START) == LOW) delay(10);   // wait for button release
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
  oled.setCursor(0, 0);  oled.printf("ROBOT    %.2fV", vbat());
  oled.setCursor(0, 10); oled.printf("%cMode: %s", page == 0 ? '>' : ' ', MODE_NAME[mode]);
  if (page > 0) { oled.setCursor(0, 20); oled.printf(">%s = %.2f", MENU[page - 1].name, *MENU[page - 1].v); }
  oled.setCursor(0, 30); oled.printf("Path(%d): %.*s", pathLen, min(pathLen, 9), path);
  oled.setCursor(0, 40); oled.printf("F%4d L%4d R%4d", distF, distL, distR);
  readLine();
  for (int i = 0; i < 8; i++) {             // column showing the 8 line channels
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
  pinMode(PIN_BUZ, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_BITS);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_BITS);
  coast();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(PIN_ENC_LA, INPUT_PULLUP); pinMode(PIN_ENC_LB, INPUT_PULLUP);
  pinMode(PIN_ENC_RA, INPUT_PULLUP); pinMode(PIN_ENC_RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LA), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RA), isrEncR, CHANGE);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  // --- 3 VL53L0X sharing address 0x29: hold left + right in reset, change addresses one by one ---
  pinMode(PIN_XSHUT_R, OUTPUT); digitalWrite(PIN_XSHUT_R, LOW);
  pinMode(PIN_XSHUT_L, OUTPUT); digitalWrite(PIN_XSHUT_L, LOW);
  delay(10);
  tofF.setTimeout(50);
  if (tofF.init()) { tofF.setAddress(0x30); tofFOk = true; }   // front -> 0x30
  pinMode(PIN_XSHUT_R, INPUT); delay(10);                      // release the right sensor from reset
  tofR.setTimeout(50);
  if (tofR.init()) { tofR.setAddress(0x31); tofROk = true; }   // right -> 0x31
  pinMode(PIN_XSHUT_L, INPUT); delay(10);                      // release the left sensor from reset
  tofL.setTimeout(50);
  if (tofL.init()) tofLOk = true;                              // left keeps 0x29
  if (tofFOk) { tofF.setMeasurementTimingBudget(20000); tofF.startContinuous(0); }   // 20 ms per measurement
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
  readTof(tofF, tofFOk, distF);                 // update the distance for display
  readTof(tofL, tofLOk, distL);
  readTof(tofR, tofROk, distR);

  int eM = readBtn(bMode), eS = readBtn(bSet), eG = readBtn(bStart);
  if (eM == 1) page = (page + 1) % (N_MENU + 1);
  if (eS) {
    if (page == 0) { if (eS == 1) mode = (mode + 1) % M_COUNT; }
    else {
      MenuItem &it = MENU[page - 1];
      float v = *it.v + (eS == 1 ? it.step : -it.step);
      *it.v = constrain(v, it.lo, it.hi);
      saveParams();
    }
  }
  if (eG == 1) {
    beep(40); delay(150);
    gyroCalibrate();                         // ~1 s, robot still
    beep(40);
    if (mode == M_CALIB) calibrateLine();
    else runMode(mode);
  }
  float v = vbat();
  if (v < 7.0f && v > 3.0f && millis() % 5000 < 20) beep(30);   // low battery

  if (millis() - tDraw > 100) { tDraw = millis(); drawMenu(); }
}
