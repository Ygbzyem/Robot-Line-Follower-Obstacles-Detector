/*
  DRIVE STRAIGHT TEST  -  Line Robot (ESP32-S3 + IO shield, pin map v5 on the perfboard)
  ------------------------------------------------------------------------
  The robot drives straight for a set distance: MPU6050 GYRO holds the heading + ENCODERS measure the distance.
  If the MPU is missing / faulty -> automatically keeps straight with the ENCODERS (wheel synchronization).

  PINS: selected with PIN_MAP below. 1 = OLD pins (breadboard): TB6612 38/39/40 + 41/42/16,
        encoders 12/13 + 14/15, buttons 17/18/0, buzzer 47.  2 = NEW pins v5 (perfboard) as follows:
    TB6612 : PWMA 38, AIN2 39, AIN1 40 (LEFT motor) | BIN1 41, BIN2 42, PWMB 2 (RIGHT motor)
             STBY bridged to VCC (3V3). VM <- battery, GND <- battery (-)
    Encoder: LEFT A21 B47 | RIGHT A48 B1   (encoder VCC = 3V3 from the 3V3 rail, NOT 5V)
    MPU6050: SDA 8, SCL 9, VCC 3V3, lying flat on the robot
    Buttons: START 18 = run | SET 17 = change distance | MODE 16 = change speed
    Buzzer : 15

  USAGE: put the robot on the floor, nose toward free space. Power on -> keep still 2 s (gyro calibration).
    Press START -> buzzer beeps, the robot runs 1 s later. Press any button while running = STOP.
    Serial 115200 (Newline):  go <mm> <speed %>   e.g.: go 1000 50
                              cpr 1400 | kp 2.0 | kd 0.05 | wheel 43 | info

  FIRST TIME: if the robot SPINS instead of driving straight -> the code detects the reversed motor and fixes it.
           if the robot DRIVES BACKWARD -> set both MOTOR_REV to true (or swap the wires of both motors).
*/
#include <Arduino.h>
#include <Wire.h>

// ============== EDIT HERE ==============
float COUNTS_PER_REV = 1400.0f;     // 300 rpm motor (estimate) - measure with Test_N20 command c
float WHEEL_D_MM = 43.0f;
bool MOTOR_REV[2] = {false, false}; // [left, right] reverse motor direction
bool ENC_REV[2]   = {false, false}; // [left, right] reverse encoder sign (the code fixes it after the first run)
float KP_HEAD = 2.0f, KD_HEAD = 0.05f;   // heading hold with the gyro (% / deg, % / (deg/s))
float KP_ENC  = 0.8f;                     // keep straight with the encoders when there is no gyro (% / mm difference)
const int DISTS[] = {500, 1000, 2000};    // mm
const int SPEEDS[] = {30, 50, 70, 100};   // %
// =========================================

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
#define BTN_START 17
#define BTN_SET 18
#define BTN_MODE 0
#define PIN_BUZ 47
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
#define BTN_START 18
#define BTN_SET 17
#define BTN_MODE 16
#define PIN_BUZ 15
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
int32_t encL() { return ENC_REV[0] ? -rawL : rawL; }
int32_t encR() { return ENC_REV[1] ? -rawR : rawR; }
float mmPerCount() { return PI * WHEEL_D_MM / COUNTS_PER_REV; }

// ---------- motor ----------
void motorOne(int side, float pct) {   // pct > 0 = robot forward
  uint8_t pwm = side ? PIN_PWMB : PIN_PWMA, in1 = side ? PIN_BIN1 : PIN_AIN1, in2 = side ? PIN_BIN2 : PIN_AIN2;
  if (MOTOR_REV[side]) pct = -pct;
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

// ---------- MPU6050 (only gyro Z needed) ----------
bool mpuOk = false;
float gz = 0, bgz = 0, yaw = 0;
bool readGz(float &o) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 2) != 2) return false;
  int16_t h = Wire.read(); int16_t raw = (h << 8) | Wire.read(); o = raw / 16.4f; return true;
}
void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR); if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x80); delay(100); mpuWrite(0x6B, 0x01); mpuWrite(0x1A, 0x03); mpuWrite(0x19, 0x00); mpuWrite(0x1B, 0x18);
  return true;
}
void calibrateGyro() {
  coast(); double s = 0; int n = 0; float g; uint32_t t0 = millis();
  while (millis() - t0 < 1500) { if (readGz(g)) { s += g; n++; } delay(2); }
  bgz = n ? s / n : 0; yaw = 0;
  Serial.printf("Gyro bias %.2f deg/s (%d samples)\n", bgz, n);
}
void updateGyro() {
  static uint32_t t = micros();
  uint32_t now = micros();
  if (!mpuOk || now - t < 2000) return;
  float dt = (now - t) * 1e-6f; t = now; float g;
  if (readGz(g)) { gz = g - bgz; yaw += gz * dt; }
}

// ---------- buttons + buzzer ----------
bool btnDown[3] = {false, false, false}; uint32_t btnT[3] = {0, 0, 0};
const uint8_t BTN[3] = {BTN_START, BTN_SET, BTN_MODE};
int pressed() {   // returns 0/1/2 on a press, -1 otherwise
  for (int i = 0; i < 3; i++) {
    bool raw = digitalRead(BTN[i]) == LOW;
    if (raw != btnDown[i] && millis() - btnT[i] > 30) { btnDown[i] = raw; btnT[i] = millis(); if (raw) return i; }
  }
  return -1;
}
void beep(int n, int on = 70) { for (int i = 0; i < n; i++) { digitalWrite(PIN_BUZ, HIGH); delay(on); digitalWrite(PIN_BUZ, LOW); delay(70); } }

// ---------- first-run direction check ----------
bool dirChecked = false;
void checkDirection() {
  if (!mpuOk) { Serial.println("No MPU -> skipping direction self-check"); dirChecked = true; return; }
  Serial.println("Direction check: both wheels forward (30%) for 0.4 s...");
  float y0 = yaw; int32_t l0 = rawL, r0 = rawR; uint32_t t0 = millis();
  while (millis() - t0 < 400) { updateGyro(); drive(30, 30); }
  brake(); uint32_t t1 = millis(); while (millis() - t1 < 300) updateGyro(); coast();
  float dy = yaw - y0; int32_t dl = rawL - l0, dr = rawR - r0;
  Serial.printf("  yaw %+.1f do | encoder tho L %+ld R %+ld\n", dy, (long)dl, (long)dr);
  // 1 wheel reversed -> robot spins in place: yaw + (turning left) = LEFT wheel is reversing
  if (dy > 15)       { MOTOR_REV[0] = !MOTOR_REV[0]; dl = -dl; Serial.println("  -> LEFT motor reversed: FIXED (MOTOR_REV[0])"); }
  else if (dy < -15) { MOTOR_REV[1] = !MOTOR_REV[1]; dr = -dr; Serial.println("  -> RIGHT motor reversed: FIXED (MOTOR_REV[1])"); }
  // encoder: must count positive when the wheel moves forward
  if (labs(dl) > 20) { ENC_REV[0] = dl < 0; } else Serial.println("  !! LEFT encoder NOT COUNTING - check wires A21/B47, VCC 3V3");
  if (labs(dr) > 20) { ENC_REV[1] = dr < 0; } else Serial.println("  !! RIGHT encoder NOT COUNTING - check wires A48/B1, VCC 3V3");
  Serial.printf("  => MOTOR_REV {%d,%d}  ENC_REV {%d,%d}  (copy into the code to keep permanently)\n",
                MOTOR_REV[0], MOTOR_REV[1], ENC_REV[0], ENC_REV[1]);
  Serial.println("  If the robot just DROVE BACKWARD: set MOTOR_REV = {true, true} and upload again.");
  dirChecked = true;
  delay(500);
}

// ---------- DRIVE STRAIGHT ----------
void goStraight(float mm, float pct) {
  if (!dirChecked) checkDirection();
  if (mpuOk) calibrateGyro();
  Serial.printf("\n=== STRAIGHT %.0f mm, %.0f%% (%s) ===\n", mm, pct, mpuOk ? "gyro + encoder" : "encoder only");
  int32_t l0 = encL(), r0 = encR();
  float heading = yaw, maxErr = 0;
  uint32_t t0 = millis(), tP = 0;
  bool stopped = false;
  while (true) {
    updateGyro();
    if (pressed() >= 0) { stopped = true; break; }
    float dL = (encL() - l0) * mmPerCount(), dR = (encR() - r0) * mmPerCount();
    float done = (dL + dR) * 0.5f;
    if (done >= mm) break;
    if (millis() - t0 > 20000) { Serial.println("Over 20 s - stop (are the encoders counting?)"); break; }
    // accelerate at the start, slow down at the end
    float base = min(pct, min(15 + (millis() - t0) * 0.1f, 15 + (mm - done) * 0.35f));
    float corr;
    if (mpuOk) { float e = heading - yaw; maxErr = max(maxErr, fabsf(e)); corr = KP_HEAD * e - KD_HEAD * gz; }   // drifted right (negative yaw) -> positive corr -> right wheel faster
    else       { corr = KP_ENC * (dL - dR); }
    corr = constrain(corr, -base * 0.6f, base * 0.6f);
    drive(base - corr, base + corr);
    if (millis() - tP >= 200) {
      tP = millis();
      Serial.printf("%5.0f mm | L %5.0f R %5.0f mm | yaw %+5.1f | base %3.0f%% corr %+5.1f\n", done, dL, dR, yaw - heading, base, corr);
    }
  }
  brake(); uint32_t t1 = millis(); while (millis() - t1 < 300) updateGyro(); coast();
  float dL = (encL() - l0) * mmPerCount(), dR = (encR() - r0) * mmPerCount(), t = (t1 - t0) / 1000.0f;
  Serial.println(stopped ? "=== STOP BY BUTTON ===" : "=== RESULT ===");
  Serial.printf("Distance: L %.0f mm, R %.0f mm, AVG %.0f mm (target %.0f)\n", dL, dR, (dL + dR) / 2, mm);
  Serial.printf("Time %.2f s, avg speed %.0f mm/s\n", t, (dL + dR) / 2 / max(0.1f, t));
  if (mpuOk) Serial.printf("Final heading error %+.1f deg, max %.1f deg\n", yaw - heading, maxErr);
  Serial.println("Pass: heading error < 3 deg, robot stops ~ in line. Measure with a ruler: error < 3% (large error -> adjust wheel/cpr).");
  beep(stopped ? 1 : 2, stopped ? 400 : 70);
}

// ---------- SETUP / LOOP ----------
int distIdx = 1, spdIdx = 1;
void printSel() { Serial.printf("-> Distance %d mm | speed %d%%  (START = run)\n", DISTS[distIdx], SPEEDS[spdIdx]); }
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
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000);
  delay(1000);
  mpuOk = mpuInit();
  Serial.printf("\n=== DRIVE STRAIGHT TEST === MPU: %s | CPR %.0f | wheel %.0f mm\n", mpuOk ? "OK" : "NONE (using encoder)", COUNTS_PER_REV, WHEEL_D_MM);
  if (mpuOk) calibrateGyro();
  Serial.println("SET = change distance | MODE = change speed | START = run | Serial: go 1000 50");
  printSel();
  beep(1);
}
void loop() {
  int b = pressed();
  if (b == 1) { distIdx = (distIdx + 1) % 3; printSel(); beep(1, 30); }
  if (b == 2) { spdIdx = (spdIdx + 1) % 4; printSel(); beep(1, 30); }
  if (b == 0) { beep(1, 150); delay(1000); goStraight(DISTS[distIdx], SPEEDS[spdIdx]); }
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    char cmd[8] = {0}; float a = 0, c = 0;
    int n = sscanf(s.c_str(), "%7s %f %f", cmd, &a, &c);
    String k(cmd);
    if (k == "go" && n >= 2) { beep(1, 150); delay(1000); goStraight(a, n >= 3 ? c : SPEEDS[spdIdx]); }
    else if (k == "cpr" && n >= 2) { COUNTS_PER_REV = a; Serial.printf("CPR = %.0f\n", a); }
    else if (k == "kp" && n >= 2) { KP_HEAD = a; Serial.printf("KP_HEAD = %.2f\n", a); }
    else if (k == "kd" && n >= 2) { KD_HEAD = a; Serial.printf("KD_HEAD = %.3f\n", a); }
    else if (k == "wheel" && n >= 2) { WHEEL_D_MM = a; Serial.printf("Wheel = %.1f mm\n", a); }
    else if (k == "info") { Serial.printf("CPR %.0f wheel %.1f KP %.2f KD %.3f MOTOR_REV {%d,%d} ENC_REV {%d,%d}\n",
                                           COUNTS_PER_REV, WHEEL_D_MM, KP_HEAD, KD_HEAD, MOTOR_REV[0], MOTOR_REV[1], ENC_REV[0], ENC_REV[1]); }
    else Serial.println("Commands: go <mm> <%> | cpr | kp | kd | wheel | info");
  }
}
