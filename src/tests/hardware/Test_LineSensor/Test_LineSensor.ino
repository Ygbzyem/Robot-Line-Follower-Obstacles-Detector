/*
  LINE SENSOR TEST (8 channels) - Line Robot ESP32-S3   -- SENSOR TEST (only command v runs the motors), no I2C / WiFi
  ---------------------------------------------------------------------------------------------
  Pins kept AS BEFORE:  D1..D8 -> GPIO 14, 13, 12, 11, 10, 7, 6, 5   (D1 = LEFT end of the sensor bar)
  Serial Monitor 115200, select "Newline".

  Each printed line (5 times/s):
     raw D1..D8 (0..4095) | sensor bar: # = black  . = white | line position (-3.5 .. +3.5)
  Commands (type then Enter):
     c   = CALIBRATE: type c, drag the sensor bar back and forth over the LINE and WHITE FLOOR ~5 s -> computes each channel's threshold
     r   = view RAW values (uncalibrated) continuously
     n   = 4 s NOISE TEST (on WHITE floor, keep still): swing of each channel
     d   = SENSOR TYPE: analog or digital (only 0 / 4095)
     i   = invert black/white (if black reads a LOW value)
     f   = reverse the channel order (if D1 is on the RIGHT)
     t   = READ SPEED: how long one analogRead takes, how long reading all 8 channels takes, how many reads/s
     *** NO COMPUTER NEEDED: unplug USB, place the robot, press START -> 3 beeps -> robot measures itself, then 3 beeps (error: 1 long beep).
         Plug USB back in, open the Serial Monitor -> the result prints at boot (or type res). ***
     res = print the previous latency RESULT (stored in flash)
     v   = SENSOR LATENCY BY DRIVING OVER THE LINE (no CTRL pin needed). The robot SELF-CALIBRATES before measuring. THE ROBOT DRIVES by itself forward/backward over a crosswise line
           at 4 speeds. If the sensor lags by tau seconds, the faster the robot the further forward the "line seen" point shifts:
           latency = position offset / speed difference. Calibrate (c) first.
           Robot placement: sensor bar ~8-10 cm from a CROSSWISE LINE (perpendicular to the robot heading), >= 30 cm clear ahead.
     h   = print help
*/
#include <Arduino.h>

const uint8_t LPIN[8] = {14, 13, 12, 11, 10, 7, 6, 5};
// motor + encoder (only used by command v) - pins as in the contest code
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
const bool REV_L = true, REV_R = false;
const float CPR = 815, WHEEL = 48;      // pulses/turn and wheel diameter (same as the contest code)
struct Pass { float v, xIn, xOut; int n; };   // 1 pass over the line: speed, position of black entry / white exit (avg of 8 channels)
uint16_t lo[8], hi[8];          // calibration: white / black
bool calOk = false, blackHigh = true, flip = false, rawMode = false;
uint16_t raw[8]; float val[8];

uint16_t readPin(int i) {        // mean of 4 reads
  uint8_t p = LPIN[flip ? 7 - i : i]; uint32_t s = 0;
  for (int k = 0; k < 4; k++) s += analogRead(p);
  return s / 4;
}
void readAll() {
  for (int i = 0; i < 8; i++) {
    raw[i] = readPin(i);
    if (calOk) {
      float span = max(1, (int)hi[i] - (int)lo[i]);
      float v = constrain(((int)raw[i] - (int)lo[i]) / span, 0.0f, 1.0f);
      val[i] = blackHigh ? v : 1 - v;
    } else {                     // not calibrated: use the midpoint 0..4095 as a temporary threshold
      float v = raw[i] / 4095.0f; val[i] = blackHigh ? v : 1 - v;
    }
  }
}
float linePos(bool &seen) {
  float s = 0, w = 0; seen = false;
  for (int i = 0; i < 8; i++) if (val[i] > 0.35f) { float k = (val[i] - 0.35f) / 0.65f; s += k * (i - 3.5f); w += k; if (val[i] > 0.5f) seen = true; }
  return w > 0 ? s / w : 0;
}

void help() {
  Serial.println("\n=== LINE SENSOR TEST (8 channels, pins 14 13 12 11 10 7 6 5) ===");
  Serial.println("Commands: c = calibrate | r = raw values | n = noise test | d = analog/digital | t = read speed | v = latency (or START BUTTON) | res = latency result | i = invert black/white | f = reverse order | h = help");
  Serial.printf("State: %s | black = %s | order %s\n", calOk ? "CALIBRATED" : "NOT calibrated (type c)", blackHigh ? "HIGH VALUE" : "LOW VALUE", flip ? "REVERSED" : "normal");
}

void calibrate() {
  Serial.println("\n== 5 s CALIBRATION: drag the sensor bar back and forth over the LINE and WHITE FLOOR ==");
  delay(500);
  uint16_t mn[8], mx[8]; double sum[8] = {0}; uint32_t n = 0;
  for (int i = 0; i < 8; i++) { mn[i] = 4095; mx[i] = 0; }
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    for (int i = 0; i < 8; i++) { uint16_t r = readPin(i); mn[i] = min(mn[i], r); mx[i] = max(mx[i], r); sum[i] += r; }
    n++; delay(2);
  }
  int good = 0, highVotes = 0;
  Serial.println("  ch | white?  black? | diff | verdict");
  for (int i = 0; i < 8; i++) {
    int ch = mx[i] - mn[i];
    float mean = sum[i] / max<uint32_t>(n, 1);
    if (mean < (mn[i] + mx[i]) / 2.0f) highVotes++;     // most of the time on the white floor -> the mean is close to the WHITE value
    lo[i] = mn[i]; hi[i] = mx[i];
    const char* g = ch < 150 ? "<-- ERROR: almost no change (wire / channel / IR LED?)" : ch < 600 ? "<-- WEAK (set height 3 mm, IR LED)" : ch < 1500 ? "acceptable" : "GOOD";
    if (ch >= 600) good++;
    Serial.printf("  D%d  | %4u - %4u  | %4d  | %s\n", i + 1, mn[i], mx[i], ch, g);
  }
  blackHigh = highVotes >= 4;
  calOk = true;
  Serial.printf("=> %d/8 channels good | black = %s\n", good, blackHigh ? "HIGH VALUE" : "LOW VALUE");
  Serial.println("   Good: diff > 1500. If all 8 channels < 600 -> check power, LEDON/CTRL pin, sensor bar height.");
}

void noiseTest() {
  Serial.println("\n== 4 s NOISE TEST: place the sensor bar on WHITE FLOOR, keep STILL ==");
  delay(300);
  uint16_t mn[8], mx[8]; double sum[8] = {0}; uint32_t n = 0;
  for (int i = 0; i < 8; i++) { mn[i] = 4095; mx[i] = 0; }
  uint32_t t0 = millis();
  while (millis() - t0 < 4000) {
    for (int i = 0; i < 8; i++) { uint16_t r = analogRead(LPIN[flip ? 7 - i : i]); mn[i] = min(mn[i], r); mx[i] = max(mx[i], r); sum[i] += r; }
    n++;
  }
  Serial.println("  ch | mean | min - max | swing");
  for (int i = 0; i < 8; i++)
    Serial.printf("  D%d  |   %4.0f     | %4u - %4u | %4u %s\n", i + 1, sum[i] / n, mn[i], mx[i], mx[i] - mn[i], mx[i] - mn[i] > 200 ? "<-- NOISY (loose wire / power)" : "");
  Serial.printf("  (%lu reads per channel)\n", (unsigned long)n);
}

void typeTest() {
  Serial.println("\n== 4 s SENSOR TYPE: drag the sensor bar over the LINE and WHITE FLOOR ==");
  delay(300);
  uint32_t mid[8] = {0}, n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 4000) {
    for (int i = 0; i < 8; i++) { uint16_t r = analogRead(LPIN[flip ? 7 - i : i]); if (r > 300 && r < 3800) mid[i]++; }
    n++; delay(1);
  }
  int nAnalog = 0;
  for (int i = 0; i < 8; i++) {
    float pm = 100.0f * mid[i] / n; bool an = pm > 10;
    if (an) nAnalog++;
    Serial.printf("  D%d: %5.1f %% mid-range values -> %s\n", i + 1, pm, an ? "ANALOG" : "maybe DIGITAL (only 0 / 4095)");
  }
  Serial.println(nAnalog >= 6 ? "=> ANALOG sensor: works with the contest code right away (remember to calibrate)."
                              : "=> Mostly 0/4095 only: DIGITAL sensor (e.g. module with a trimmer). Report back to adjust the contest code.");
}

// ---------- READ SPEED (ESP32 side) ----------
void speedTest() {
  Serial.println("\n== READ SPEED ==");
  uint32_t t0 = micros(); for (int k = 0; k < 1000; k++) analogRead(LPIN[3]); float one = (micros() - t0) / 1000.0f;
  t0 = micros(); for (int k = 0; k < 200; k++) readAll(); float all = (micros() - t0) / 200.0f;
  Serial.printf("  1 analogRead          : %.1f us\n", one);
  Serial.printf("  read all 8 channels (x4 samples) : %.0f us  -> max %.0f reads/s (same as contest code)\n", all, 1e6f / all);
  Serial.printf("  at 1 m/s the robot travels %.1f mm per read -> %s\n", all / 1000.0f, all < 2000 ? "negligible" : "A BIT SLOW, reduce the sample count");
}

// ---------- MOTOR / ENCODER ----------
volatile int32_t encL = 0, encR = 0; volatile uint8_t pvL = 0, pvR = 0;
const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};
void IRAM_ATTR isrL() { uint8_t s = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB); encL += QDEC[(pvL << 2) | s]; pvL = s; }
void IRAM_ATTR isrR() { uint8_t s = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB); encR += QDEC[(pvR << 2) | s]; pvR = s; }
int encDir = 1, encSum = 0; bool motorInit = false;
float posMM() { return encDir * (encSum ? encL + encR : encL - encR) * 0.5f * PI * WHEEL / CPR; }   // default left +, right - (as in the log: enc +1 -1)
void motorOne(int side, float pct) {
  uint8_t pwm = side ? PIN_PWMB : PIN_PWMA, in1 = side ? PIN_BIN1 : PIN_AIN1, in2 = side ? PIN_BIN2 : PIN_AIN2;
  if (side ? REV_R : REV_L) pct = -pct;
  pct = constrain(pct, -100.0f, 100.0f);
  digitalWrite(in1, pct > 0); digitalWrite(in2, pct < 0);
  ledcWrite(pwm, (int)(fabsf(pct) * 1023 / 100.0f));
}
void drive(float l, float r) { motorOne(0, l); motorOne(1, r); }
void brakeAll() { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH); ledcWrite(PIN_PWMA, 1023);
                  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH); ledcWrite(PIN_PWMB, 1023); }
void initMotor() {
  if (motorInit) return; motorInit = true;
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  ledcAttach(PIN_PWMA, 20000, 10); ledcAttach(PIN_PWMB, 20000, 10);
  pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP); pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), isrL, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_LB), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), isrR, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_RB), isrR, CHANGE);
}
float blackOf(int i, uint16_t r) {   // 0 = white, 1 = black (by calibration)
  float span = max(1, (int)hi[i] - (int)lo[i]);
  float v = constrain(((int)r - (int)lo[i]) / span, 0.0f, 1.0f);
  return blackHigh ? v : 1 - v;
}
// drive 'mm' (positive = forward) at speed pct, record where each channel goes ONTO black / OFF to white (interpolated at the 50% level)
Pass runPass(float mm, float pct) {
  Pass P = {0, 0, 0, 0};
  float xIn[8], xOut[8], vIn[8], vOut[8]; bool gIn[8] = {0}, gOut[8] = {0};
  float prevB[8], prevX = posMM();
  for (int i = 0; i < 8; i++) prevB[i] = blackOf(i, analogRead(LPIN[flip ? 7 - i : i]));
  const int NH = 64; float hx[NH]; uint32_t ht[NH]; int hn = 0;   // position history to compute speed
  float x0 = posMM(); uint32_t t0 = millis(); const int dir = mm > 0 ? 1 : -1;
  drive(dir * pct, dir * pct);
  while (millis() - t0 < 4000) {
    float x = posMM(); uint32_t tu = micros();
    hx[hn % NH] = x; ht[hn % NH] = tu; hn++;
    if ((x - x0) * dir >= fabsf(mm)) break;
    if (millis() - t0 > 400 && fabsf(x - x0) < 5) { brakeAll(); Serial.println("!! Encoders not counting / robot not moving"); P.n = -1; return P; }
    // speed = distance over the last ~20 ms
    int k = max(0, hn - 1 - 40); float v = 0;
    if (hn > 2) { uint32_t dt = ht[(hn - 1) % NH] - ht[k % NH]; if (dt > 0) v = (hx[(hn - 1) % NH] - hx[k % NH]) * 1e6f / dt; }
    // first 3 points are accelerating -> skip
    for (int i = 0; i < 8; i++) {
      float b = blackOf(i, analogRead(LPIN[flip ? 7 - i : i]));
      float xi = x;
      if (prevB[i] < 0.5f && b >= 0.5f && !gIn[i]) { float f = (0.5f - prevB[i]) / max(1e-3f, b - prevB[i]); xIn[i] = prevX + f * (xi - prevX); vIn[i] = v; gIn[i] = true; }
      if (prevB[i] >= 0.5f && b < 0.5f && gIn[i] && !gOut[i]) { float f = (prevB[i] - 0.5f) / max(1e-3f, prevB[i] - b); xOut[i] = prevX + f * (xi - prevX); vOut[i] = v; gOut[i] = true; }
      prevB[i] = b;
    }
    prevX = x;
  }
  brakeAll(); delay(250); drive(0, 0); delay(150);
  float si = 0, so = 0, sv = 0; int n = 0;
  for (int i = 0; i < 8; i++) if (gIn[i] && gOut[i]) { si += xIn[i]; so += xOut[i]; sv += (vIn[i] + vOut[i]) / 2; n++; }
  if (n) { P.xIn = si / n; P.xOut = so / n; P.v = sv / n; }
  P.n = n;
  return P;
}
// linear regression y = a + b*x -> returns b (the slope)
float slope(const float* x, const float* y, int n) {
  float mx = 0, my = 0; for (int i = 0; i < n; i++) { mx += x[i]; my += y[i]; } mx /= n; my /= n;
  float sxx = 0, sxy = 0; for (int i = 0; i < n; i++) { sxx += (x[i] - mx) * (x[i] - mx); sxy += (x[i] - mx) * (y[i] - my); }
  return sxx > 1 ? sxy / sxx : 0;
}
// ---------- latency result: print to Serial + SAVE to flash (works without USB, plug in again and type "res" to view) ----------
#include <Preferences.h>
String resLog; bool okRun = false; Preferences pref;
void outl(const char* t) { Serial.println(t); resLog += t; resLog += "\n"; }
void outf(const char* fmt, ...) { char b[200]; va_list a; va_start(a, fmt); vsnprintf(b, sizeof b, fmt, a); va_end(a); Serial.print(b); resLog += b; }
void saveResult() { pref.begin("tredo", false); pref.putString("kq", resLog.length() > 3800 ? resLog.substring(0, 3800) : resLog); pref.end(); }   // NVS name kept from the first firmware so data already stored on the robot survives updates
void printResult() { pref.begin("tredo", true); String t = pref.getString("kq", ""); pref.end();
  Serial.println(t.length() ? "\n===== PREVIOUS LATENCY RESULT =====" : "(no latency result yet)"); Serial.print(t); }
// ---------- buttons + buzzer (same as contest code) ----------
#define BTN_START 18
#define BTN_SET 17
#define PIN_BUZ 15
void beep(int n, int on = 80) { for (int i = 0; i < n; i++) { digitalWrite(PIN_BUZ, HIGH); delay(on); digitalWrite(PIN_BUZ, LOW); delay(90); } }
bool pressed(int pin) { if (digitalRead(pin)) return false; delay(30); if (digitalRead(pin)) return false; while (!digitalRead(pin)) delay(5); return true; }
void runDelayFromButton() {
  // 3 s countdown: 3 short beeps -> robot runs
  for (int i = 0; i < 3; i++) { beep(1, 60); delay(850); }
  okRun = false; delayTest();
  saveResult();
  if (okRun) beep(3, 70); else beep(1, 700);     // 3 beeps = done OK, 1 long beep = error (plug in USB, type res to see why)
}

void delayTest() {
  initMotor(); resLog = "";
  outl("\n== SENSOR LATENCY BY DRIVING OVER THE LINE ==");
  outl("Sensor bar ~8-10 cm from a CROSSWISE LINE, >= 30 cm clear ahead.");
  // encoder direction check: short forward run
  int32_t l0 = encL, r0 = encR; drive(25, 25); delay(250); brakeAll(); delay(200); drive(0, 0);
  int32_t dl = encL - l0, dr = encR - r0;
  encSum = ((dl > 0) == (dr > 0)) ? 1 : 0;                       // 2 encoders same sign going forward -> add, opposite sign -> subtract
  float d = (encSum ? dl + dr : dl - dr) * 0.5f;
  if (fabsf(d) * PI * WHEEL / CPR < 3) { outf("!! Encoders not counting (L %ld R %ld) -> check encoder wiring\n", (long)dl, (long)dr); return; }
  encDir = d > 0 ? 1 : -1;
  // SELF-CALIBRATION: drive slowly over the line and back, record min/max per channel (no need to type c)
  {
    uint16_t mn[8], mx[8]; double sm[8] = {0}; uint32_t n = 0; for (int i = 0; i < 8; i++) { mn[i] = 4095; mx[i] = 0; }
    // WHITE LEVEL = reading while the robot is STILL at the start (sensor bar on WHITE floor, 8-10 cm from the line).
    // (old version guessed black/white polarity from the mean -> one noise spike to 4095 at motor start made it guess REVERSED)
    float w0[8];
    for (int i = 0; i < 8; i++) { uint32_t a = 0; for (int k = 0; k < 32; k++) a += analogRead(LPIN[flip ? 7 - i : i]); w0[i] = a / 32.0f; }
    float h0 = posMM();
    for (int dirp = 1; dirp >= -1; dirp -= 2) {
      drive(dirp * 18, dirp * 18); uint32_t t0 = millis();
      while (millis() - t0 < 4000 && (dirp > 0 ? posMM() - h0 < 220 : posMM() - h0 > 0)) {
        static uint16_t pv[8]; static bool hp = false;   // min/max must hold for >= 2 consecutive reads (drops noise spikes)
        for (int i = 0; i < 8; i++) { uint16_t r = analogRead(LPIN[flip ? 7 - i : i]); if (hp) { mn[i] = min(mn[i], max(r, pv[i])); mx[i] = max(mx[i], min(r, pv[i])); } pv[i] = r; sm[i] += r; }
        hp = true;
        n++;
      }
      brakeAll(); delay(250); drive(0, 0); delay(150);
    }
    int votes = 0, weak = 0;
    for (int i = 0; i < 8; i++) if (mx[i] - w0[i] > w0[i] - mn[i]) votes++;    // white closer to MIN -> black = HIGH value
    blackHigh = votes >= 4; calOk = true;
    for (int i = 0; i < 8; i++) {
      if (blackHigh) { lo[i] = (uint16_t)w0[i]; hi[i] = mx[i]; } else { lo[i] = mn[i]; hi[i] = (uint16_t)w0[i]; }   // the white level is taken WHILE STILL (no noise spikes)
      if (hi[i] < lo[i] + 300) weak++;
    }
    outf("  White level at rest:"); for (int i = 0; i < 8; i++) outf(" %.0f", w0[i]); outf("\n");
    outf("  Black level (peak when crossing the line):"); for (int i = 0; i < 8; i++) outf(" %u", blackHigh ? mx[i] : mn[i]); outf("\n");
    outf("  Self-calibration: black = %s, %d/8 channels diff < 300%s\n", blackHigh ? "HIGH VALUE" : "LOW VALUE", weak, weak > 2 ? " <- ROBOT NOT CROSSING THE LINE? reposition the robot" : "");
    if (weak > 4) { outl("!! Line not seen at low speed -> place the sensor bar 8-10 cm from a CROSSWISE LINE and retry"); return; }
  }
  const float SP[4] = {20, 30, 40, 55}; const int NS = 4;
  float vF[NS], inF[NS], outF[NS], vB[NS], inB[NS], outB[NS]; int nF = 0, nB = 0;
  float pdv[NS], pdc[NS]; int np = 0;   // FORWARD/BACKWARD pair at the same speed: line-center difference vs speed difference
  float home = posMM();
  outl("  speed | dir | v (mm/s) | onto black (mm) | onto white (mm) | channels");
  for (int k = 0; k < NS; k++) {
    float back = posMM() - home; if (fabsf(back) > 10) { Pass r = runPass(-back, 25); (void)r; }   // back to the start point
    Pass f = runPass(220, SP[k]);
    if (f.n < 0) return;
    outf("   %3.0f%%  | TIEN  | %7.0f  | %10.1f   | %10.1f    | %d\n", SP[k], f.v, f.xIn - home, f.xOut - home, f.n);
    if (f.n >= 4) { vF[nF] = f.v; inF[nF] = f.xIn - home; outF[nF] = f.xOut - home; nF++; }
    Pass b = runPass(-220, SP[k]);
    if (b.n < 0) return;
    outf("   %3.0f%%  | LUI   | %7.0f  | %10.1f   | %10.1f    | %d\n", SP[k], b.v, b.xIn - home, b.xOut - home, b.n);
    if (b.n >= 4) { vB[nB] = b.v; inB[nB] = b.xIn - home; outB[nB] = b.xOut - home; nB++; }
    if (f.n >= 4 && b.n >= 4 && f.v > 30 && b.v < -30) { pdv[np] = f.v - b.v; pdc[np] = (f.xIn + f.xOut) / 2 - (b.xIn + b.xOut) / 2; np++; }
  }
  // ACCURATE METHOD: at the same speed, FORWARD sees the line center shifted forward by v*lag, BACKWARD shifted back by v*lag -> diff = (vT - vL) * lag.
  // Measure at several speeds -> removes the fixed offset (gear backlash, position offset).
  if (np >= 2) {
    float t = slope(pdv, pdc, np) * 1000;
    outl("  Forward/backward pair at the same speed (line center FWD - BACK):");
    for (int k = 0; k < np; k++) outf("    speed diff %4.0f mm/s -> center diff %5.1f mm -> %.1f ms\n", pdv[k], pdc[k], pdc[k] / pdv[k] * 1000);
    outf("=> SENSOR LATENCY ~ %.1f ms  (contest code: type  lag %.0f)\n", t, max(0.0f, t));
    okRun = true;
  } else if (np == 1) {
    float t = pdc[0] / pdv[0] * 1000;
    outf("=> SENSOR LATENCY ~ %.1f ms (only 1 speed pair, measure again)  (contest code: type  lag %.0f)\n", t, max(0.0f, t));
    okRun = true;
  }
  if (nF < 3 || nB < 3) { if (!okRun) outl("!! Not enough line detections (need >= 3 speeds). Check: line across, 8-10 cm away, calibrated."); return; }
  // seen position = real position + v * lag  ->  slope versus speed = latency (seconds)
  outl("  (Old method - each direction separately, affected while the robot accelerates:)");
  float tInF = slope(vF, inF, nF), tOutF = slope(vF, outF, nF), tInB = slope(vB, inB, nB), tOutB = slope(vB, outB, nB);
  float tIn = (tInF + tInB) / 2 * 1000, tOut = (tOutF + tOutB) / 2 * 1000;
  float wSlow = outF[0] - inF[0];
  outf("=> Latency onto black line  : %.1f ms (fwd %.1f / back %.1f)\n", tIn, tInF * 1000, tInB * 1000);
  outf("=> Latency onto white floor : %.1f ms (fwd %.1f / back %.1f)\n", tOut, tOutF * 1000, tOutB * 1000);
  outf("   Line width measured at low speed: %.1f mm (real line ~20 mm)\n", wSlow);
  float w = max(tIn, tOut);
  outf("   At 1 m/s the robot 'sees' %.0f mm late. ", w);
  outl(w < 3 ? "VERY FAST (like a QTR phototransistor bar)."
               : w < 10 ? "Acceptable, lower the corner entry speed (cspd) a little."
               : "SLOW (typical of PHOTORESISTOR sensors). Lower the run speed and raise mpre / lower cspd.");
  outl("   (Error ~ +-2 ms. Run 2-3 times to be sure.)");
  okRun = true;
}

void setup() {
  pinMode(BTN_START, INPUT_PULLUP); pinMode(BTN_SET, INPUT_PULLUP); pinMode(PIN_BUZ, OUTPUT); digitalWrite(PIN_BUZ, LOW);
  Serial.begin(115200);
  analogReadResolution(12);
  for (int i = 0; i < 8; i++) { pinMode(LPIN[i], INPUT); lo[i] = 0; hi[i] = 4095; }
  delay(1500);
  help();
  printResult();
  beep(1, 40);
}

void loop() {
  if (pressed(BTN_START)) runDelayFromButton();     // UNPLUG USB, place the robot, press START -> measures latency (self-calibrating)
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s == "c") calibrate();
    else if (s == "r") { rawMode = !rawMode; Serial.printf("Raw: %s\n", rawMode ? "ON" : "OFF"); }
    else if (s == "n") noiseTest();
    else if (s == "d") typeTest();
    else if (s == "t") speedTest();
    else if (s == "v") runDelayFromButton();
    else if (s == "res") printResult();
    else if (s == "i") { blackHigh = !blackHigh; Serial.printf("Black = %s\n", blackHigh ? "HIGH VALUE" : "LOW VALUE"); }
    else if (s == "f") { flip = !flip; calOk = false; Serial.println("Channel order reversed -> recalibrate (c)"); }
    else if (s.length()) help();
  }
  static uint32_t tP = 0;
  if (millis() - tP < 200) return;
  tP = millis();
  readAll();
  bool seen; float p = linePos(seen);
  char bar[9]; int nb = 0;
  for (int i = 0; i < 8; i++) { bar[i] = val[i] > 0.5f ? '#' : '.'; if (val[i] > 0.5f) nb++; }
  bar[8] = 0;
  Serial.printf("%s  ", bar);
  if (rawMode || !calOk) for (int i = 0; i < 8; i++) Serial.printf("%4u ", raw[i]);
  else for (int i = 0; i < 8; i++) Serial.printf("%3d ", (int)(val[i] * 100));
  if (seen) Serial.printf("| line %+5.2f  black %d%s\n", p, nb, nb >= 5 ? " JUNCTION/BLACK BOX" : "");
  else Serial.println("| -- line lost --");
}
