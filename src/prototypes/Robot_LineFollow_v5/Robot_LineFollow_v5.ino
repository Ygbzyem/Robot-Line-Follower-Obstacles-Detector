/*
  ROBOT - LINE FOLLOW (NEW pin map v5 on the perfboard)                 v1.0
  ---------------------------------------------------------------------------
  PINS:
    TB6612 : PWMA 38, AIN2 39, AIN1 40 -> LEFT motor | BIN1 41, BIN2 42, PWMB 2 -> RIGHT motor
             STBY bridged to VCC (3V3). PWMA/PWMB have 10k to GND.
    QTR-8RA: D1..D8 -> GPIO 14, 13, 12, 11, 10, 7, 6, 5  (D1 = LEFTMOST channel)
             VCC -> 3V3 rail (NOT 5V) | GND -> G | LEDON left open
    Buttons: START 18 = run | SET 17 = calibrate | MODE 16 = change speed
    Buzzer : 15   (encoders 21/47, 48/1 not needed for this test)

  USAGE:
    1. CALIBRATION (required the first time, saved to flash - kept after power-off):
       Place the robot so the line sensor bar is ON THE LINE, press SET.
       The robot sweeps left-right over the line ~5 s. 2 beeps = pass, 1 long beep = fail (do it again).
    2. Put the robot on the line, press START -> the robot runs 1 s later. Press any button = STOP.
    3. MODE = change speed 30/40/50/60/70/80 %. Start at 30-40 %, raise only after tuning.

  SERIAL 115200 (Newline):
    kp 14 | kd 0.35 | sp 40 | cal | go | raw (view 8 channels) | flip (reverse channel order) | info
    -> every command that changes a value is SAVED AUTOMATICALLY.

  PD TUNING (do it at low speed first):
    Robot keeps wobbling         -> lower kp or raise kd
    Robot slides out on bends   -> raise kp, or lower the speed
    Robot jerks, vibrates        -> lower kd
*/
#include <Arduino.h>
#include <Preferences.h>

// ================= EDIT HERE =================
bool REV_L = true;    // LEFT motor reversed (confirmed in the straight-drive test)
bool REV_R = false;
const uint32_t DASH_MS = 350;    // max line loss in a dashed segment (driving straight)
const uint32_t SEARCH_MS = 1500; // max line search turn before stopping
const float SEARCH_PCT = 35;     // line search turn speed
// ===============================================

#define PIN_PWMA 38
#define PIN_AIN2 39
#define PIN_AIN1 40
#define PIN_BIN1 41
#define PIN_BIN2 42
#define PIN_PWMB 2
#define BTN_START 18
#define BTN_SET 17
#define BTN_MODE 16
#define PIN_BUZ 15
const uint8_t LPIN[8] = {14, 13, 12, 11, 10, 7, 6, 5};

const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = 1023;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_OUT(pin, ch, d) ledcWrite(pin, d)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_OUT(pin, ch, d) ledcWrite(ch, d)
#endif

// ---------- parameters stored in NVS ----------
struct Cfg {
  uint32_t magic;
  uint16_t lmin[8], lmax[8];
  uint8_t blackHigh, calOk, flip;
  float kp, kd; int base;
};
const uint32_t MAGIC = 0xD011E501;
Cfg cfg;
Preferences prefs;
void cfgDefault() {
  cfg = Cfg(); cfg.magic = MAGIC;
  for (int i = 0; i < 8; i++) { cfg.lmin[i] = 0; cfg.lmax[i] = 4095; }
  cfg.blackHigh = 1; cfg.calOk = 0; cfg.flip = 0; cfg.kp = 14; cfg.kd = 0.35f; cfg.base = 40;
}
void cfgLoad() { prefs.begin("doline", false); Cfg t; if (prefs.getBytes("c", &t, sizeof(t)) == sizeof(t) && t.magic == MAGIC) cfg = t; else cfgDefault(); }
void cfgSave() { prefs.putBytes("c", &cfg, sizeof(cfg)); }

// ---------- motor ----------
void motorOne(int side, float pct) {   // pct > 0 = robot forward, -100..100
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

// ---------- buzzer + buttons ----------
void beep(int n, int on = 70) { for (int i = 0; i < n; i++) { digitalWrite(PIN_BUZ, HIGH); delay(on); digitalWrite(PIN_BUZ, LOW); delay(70); } }
const uint8_t BTN[3] = {BTN_START, BTN_SET, BTN_MODE};
bool bDown[3] = {false, false, false}; uint32_t bT[3] = {0, 0, 0};
int pressed() {
  for (int i = 0; i < 3; i++) {
    bool raw = digitalRead(BTN[i]) == LOW;
    if (raw != bDown[i] && millis() - bT[i] > 30) { bDown[i] = raw; bT[i] = millis(); if (raw) return i; }
  }
  return -1;
}

// ---------- line sensor ----------
uint16_t raw[8]; float val[8]; int nBlack = 0; float pos = 0; bool seen = false;
void readLine() {
  float sw = 0, s = 0; nBlack = 0;
  for (int i = 0; i < 8; i++) {
    raw[i] = analogRead(LPIN[cfg.flip ? 7 - i : i]);
    float span = max(1, (int)cfg.lmax[i] - (int)cfg.lmin[i]);
    float v = constrain(((int)raw[i] - (int)cfg.lmin[i]) / span, 0.0f, 1.0f);
    if (!cfg.blackHigh) v = 1 - v;   // v: 0 = white, 1 = black
    val[i] = v;
    if (v > 0.5f) nBlack++;
    if (v > 0.2f) { sw += v; s += v * (i - 3.5f); }
  }
  seen = sw > 0.4f;
  if (seen) pos = s / sw;   // -3.5 (line on the left) .. +3.5 (line on the right)
}

// ---------- calibration ----------
void calibrate() {
  Serial.println("\n=== CALIBRATION: robot sweeps over the line for 5 s (sensor bar must be on the line) ===");
  beep(1, 150); delay(800);
  uint16_t mn[8], mx[8]; double sum[8] = {0}; uint32_t n = 0;
  for (int i = 0; i < 8; i++) { mn[i] = 4095; mx[i] = 0; }
  // turn: left 0.35 s - right 0.7 s - left 0.7 s - right 0.7 s - left 0.35 s (ends near the start)
  const float seq[5][2] = {{-1, 350}, {1, 700}, {-1, 700}, {1, 700}, {-1, 350}};
  for (auto &st : seq) {
    uint32_t t0 = millis();
    while (millis() - t0 < (uint32_t)st[1]) {
      drive(-st[0] * 28, st[0] * 28);   // st = -1: turn left
      for (int i = 0; i < 8; i++) { uint16_t r = analogRead(LPIN[cfg.flip ? 7 - i : i]); mn[i] = min(mn[i], r); mx[i] = max(mx[i], r); sum[i] += r; }
      n++; delay(2);
    }
  }
  brake(); delay(150); coast();
  int good = 0, highVotes = 0;
  for (int i = 0; i < 8; i++) {
    float mean = sum[i] / max<uint32_t>(n, 1);
    if (mx[i] - mn[i] > 300) good++;
    if (mean < (mn[i] + mx[i]) / 2.0f) highVotes++;   // most of the time on the white floor -> white low -> black high
    cfg.lmin[i] = mn[i]; cfg.lmax[i] = mx[i];
    Serial.printf("  D%d: min %4u  max %4u  diff %4u %s\n", i + 1, mn[i], mx[i], mx[i] - mn[i], mx[i] - mn[i] > 300 ? "" : "<-- WEAK");
  }
  cfg.blackHigh = highVotes >= 4;
  cfg.calOk = good >= 7;
  Serial.printf("=> %d/8 channels good, black = %s value -> %s\n", good, cfg.blackHigh ? "HIGH" : "LOW", cfg.calOk ? "PASS, saved" : "FAIL");
  if (!cfg.calOk) Serial.println("   Check: sensor bar ~3-8 mm above the floor, VCC 3V3, does the robot sweep across the line.");
  cfgSave();
  if (cfg.calOk) beep(2); else beep(1, 600);
}

// ---------- line following ----------
void runLine() {
  if (!cfg.calOk) { Serial.println("!! NOT CALIBRATED - press SET first"); beep(1, 600); return; }
  Serial.printf("\n=== LINE FOLLOW  speed %d%%  kp %.1f  kd %.2f ===\n", cfg.base, cfg.kp, cfg.kd);
  beep(1, 120); delay(1000);
  uint32_t t0 = millis(), tLost = 0, tFin = 0, tP = 0, tPrev = micros();
  float lastErr = 0, lastPos = 0, maxErr = 0; uint32_t loops = 0;
  const char* why = "STOP BUTTON";
  while (true) {
    if (pressed() >= 0) break;
    readLine(); loops++;
    uint32_t now = micros(); float dt = max(1e-4f, (now - tPrev) * 1e-6f); tPrev = now;
    // FINISH: wide black box (>= 7 black channels) continuously for 120 ms
    if (nBlack >= 7 && millis() - t0 > 800) { if (!tFin) tFin = millis(); if (millis() - tFin > 120) { why = "TO FINISH"; break; } }
    else tFin = 0;
    if (seen) {
      tLost = 0;
      float e = pos, d = (e - lastErr) / dt; lastErr = e; lastPos = e;
      maxErr = max(maxErr, fabsf(e));
      float u = cfg.kp * e + cfg.kd * d;
      float base = cfg.base * (1.0f - 0.45f * min(1.0f, fabsf(e) / 3.0f));   // slow down on large error (entering a bend)
      drive(base + u, base - u);    // line on the right -> left wheel faster -> turn right
    } else {
      if (!tLost) tLost = millis();
      uint32_t el = millis() - tLost;
      if (fabsf(lastPos) < 2.0f && el < DASH_MS) drive(cfg.base * 0.7f, cfg.base * 0.7f);   // dashed segment: drive straight
      else if (el < SEARCH_MS) { float s = lastPos > 0 ? 1 : -1; drive(s * SEARCH_PCT, -s * SEARCH_PCT); }   // turn toward the side the line was just lost
      else { why = "LINE LOST"; break; }
    }
    if (millis() - tP >= 100) {
      tP = millis();
      Serial.printf("pos %+5.2f black %d %s\n", pos, nBlack, seen ? "" : "(line lost)");
    }
  }
  brake(); delay(200); coast();
  float t = (millis() - t0) / 1000.0f;
  Serial.printf("=== STOP: %s | %.1f s | max error %.2f | %lu loops/s ===\n", why, t, maxErr, (unsigned long)(loops / max(0.1f, t)));
  beep(strcmp(why, "TO FINISH") == 0 ? 3 : 1, strcmp(why, "TO FINISH") == 0 ? 70 : 400);
}

// ---------- serial ----------
const int SPEEDS[] = {30, 40, 50, 60, 70, 80};
void info() {
  Serial.printf("kp %.2f kd %.3f speed %d%% | calib %s, black %s, flip %d | REV_L %d REV_R %d\n",
    cfg.kp, cfg.kd, cfg.base, cfg.calOk ? "OK" : "NO", cfg.blackHigh ? "high" : "low", cfg.flip, REV_L, REV_R);
}
void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n'); s.trim();
  char c[8] = {0}; float v = 0; int n = sscanf(s.c_str(), "%7s %f", c, &v);
  if (n < 1) return;
  String k(c);
  if (k == "kp" && n == 2) { cfg.kp = v; cfgSave(); info(); }
  else if (k == "kd" && n == 2) { cfg.kd = v; cfgSave(); info(); }
  else if (k == "sp" && n == 2) { cfg.base = constrain((int)v, 10, 100); cfgSave(); info(); }
  else if (k == "cal") calibrate();
  else if (k == "go") runLine();
  else if (k == "flip") { cfg.flip ^= 1; cfg.calOk = 0; cfgSave(); Serial.println("Channel order reversed -> RECALIBRATE"); }
  else if (k == "info") info();
  else if (k == "raw") {
    Serial.println("View 8 channels for 5 s (move the line back and forth under the sensor):");
    uint32_t t0 = millis();
    while (millis() - t0 < 5000) { readLine();
      for (int i = 0; i < 8; i++) Serial.printf("%4u ", raw[i]);
      Serial.printf("| pos %+5.2f black %d\n", pos, nBlack); delay(150); }
  }
  else Serial.println("Commands: kp | kd | sp | cal | go | raw | flip | info");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZ, OUTPUT); digitalWrite(PIN_BUZ, LOW);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  PWM_ATTACH(PIN_PWMA, 0); PWM_ATTACH(PIN_PWMB, 1);
  coast();
  for (int i = 0; i < 3; i++) pinMode(BTN[i], INPUT_PULLUP);
  analogReadResolution(12);
  delay(800);
  cfgLoad();
  Serial.println("\n=== ROBOT LINE FOLLOW v5 ===  SET = calibrate | START = run | MODE = speed");
  info();
  beep(cfg.calOk ? 1 : 2, 60);
}

void loop() {
  int b = pressed();
  if (b == 0) runLine();
  if (b == 1) calibrate();
  if (b == 2) {
    int i = 0; while (i < 6 && SPEEDS[i] != cfg.base) i++;
    cfg.base = SPEEDS[(i + 1) % 6]; cfgSave();
    Serial.printf("-> speed %d%%\n", cfg.base);
    beep(cfg.base / 10 - 2, 40);   // 30% = 1 beep, 40% = 2 beeps ...
  }
  handleSerial();
}
