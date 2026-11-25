/*
  WHEEL SPEED + ODOMETER TEST, NO USB CABLE NEEDED  -  Line Robot (MKE-K01 ESP32-S3, pin map v5)
  -------------------------------------------------------------------------------------------
  The robot drives by itself on the floor; results show on the OLED and are kept in flash.
  Buttons: MODE = choose test | START = run it (starts 1.5 s after the beeps) | any button while driving = STOP.
  Tests (MODE cycles through them):
    ODO 1m : drives straight at low speed until the encoders say 1000 mm, then brakes.
             Measure the REAL distance with a tape (same point on the robot, start -> stop).
             Real distance / encoder distance = correction for the wheel diameter (or a counting problem).
    PWM 40 / 60 / 80 / 100 : accelerates at that PWM until the speed is steady, measures for 300 ms, brakes.
             Needs about 2 m of clear floor ahead (the test stops itself after 2 m).
  Every run drives FORWARD in a straight line (the faster wheel is slowed a little to keep it straight).
  Later, with USB: Serial Monitor 115200, type p -> prints every saved result (copy it to the chat). c = clear results.
*/
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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
#define BTN_START 18
#define BTN_SET 17
#define BTN_MODE 16
#define PIN_BUZ 15
#define PIN_VBAT 4

const bool REV_L = true, REV_R = false;   // same as the contest firmware (left motor is mounted reversed)
const float CPR = 576.3f;                 // encoder counts per WHEEL turn
const float WHEEL_MM = 48.0f;             // wheel diameter
const float VBAT_RATIO = 4.33f;           // battery divider, same as the contest firmware
const float MAX_MM = 2000;                // safety: never drive further than this
const float KS = 0.08f;                   // straight keeping: % PWM per count of left/right difference

const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = 1023;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_OUT(pin, ch, d) ledcWrite(pin, d)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_OUT(pin, ch, d) ledcWrite(ch, d)
#endif

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;
Preferences nvs;

volatile int32_t encL = 0, encR = 0;
volatile uint8_t prevL = 0, prevR = 0;
const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};
void IRAM_ATTR isrL() { uint8_t s = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB); encL += QDEC[(prevL << 2) | s]; prevL = s; }
void IRAM_ATTR isrR() { uint8_t s = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB); encR += QDEC[(prevR << 2) | s]; prevR = s; }
int32_t cntL() { return abs(encL); }      // driving forward only -> the sign does not matter
int32_t cntR() { return abs(encR); }
float mmOf(float counts) { return counts * PI * WHEEL_MM / CPR; }

const int NT = 5;
const char *const NAME[NT] = {"ODO 1m", "PWM 40", "PWM 60", "PWM 80", "PWM 100"};
const int PCT[NT] = {30, 40, 60, 80, 100};
int sel = 0;

void motorOne(int side, float pct) {
  uint8_t pwm = side ? PIN_PWMB : PIN_PWMA, in1 = side ? PIN_BIN1 : PIN_AIN1, in2 = side ? PIN_BIN2 : PIN_AIN2;
  if (side ? REV_R : REV_L) pct = -pct;
  pct = constrain(pct, -100.0f, 100.0f);
  digitalWrite(in1, pct > 0); digitalWrite(in2, pct < 0);
  PWM_OUT(pwm, side, (int)(fabsf(pct) * PWM_MAX / 100.0f));
}
void brake() {
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH); PWM_OUT(PIN_PWMA, 0, PWM_MAX);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH); PWM_OUT(PIN_PWMB, 1, PWM_MAX);
}
void coast() { motorOne(0, 0); motorOne(1, 0); }
bool anyBtn() { return digitalRead(BTN_START) == LOW || digitalRead(BTN_SET) == LOW || digitalRead(BTN_MODE) == LOW; }
void beep(int n, int on = 70) { for (int i = 0; i < n; i++) { digitalWrite(PIN_BUZ, HIGH); delay(on); digitalWrite(PIN_BUZ, LOW); delay(70); } }
float vbat() { uint32_t t = 0; for (int i = 0; i < 8; i++) t += analogRead(PIN_VBAT); return t / 8.0f * 3.3f / 4095.0f * VBAT_RATIO; }

// drive forward at pct, slowing the wheel that is ahead so the robot stays straight
void driveStraight(float pct) {
  float e = KS * (cntL() - cntR());
  motorOne(0, pct - max(0.0f, e)); motorOne(1, pct - max(0.0f, -e));
}

void show(const char *l1, const char *l2, const char *l3, const char *l4) {
  if (!oledOk) return;
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  const char *L[4] = {l1, l2, l3, l4};
  for (int i = 0; i < 4; i++) { oled.setCursor(0, i * 16); oled.print(L[i]); }
  oled.display();
}
void showMenu() {
  char b[24], v[24]; snprintf(b, sizeof b, "> %s", NAME[sel]); snprintf(v, sizeof v, "batt %.2f V", vbat());
  show("MODE=next START=run", b, v, sel ? "needs 2 m clear" : "tape ready");
}

// print / show one saved result
void printResult(int i, bool toOled) {
  char k[4] = {'r', (char)('0' + i), 0};
  float r[6] = {0};
  if (nvs.getBytes(k, r, sizeof r) != sizeof r) { if (!toOled) Serial.printf("%-8s: not run yet\n", NAME[i]); return; }
  char a[24], b[24], c[24];
  if (i == 0) {   // r = countsL, countsR, mm by encoders, batt
    Serial.printf("%-8s: counts L %.0f R %.0f | encoder distance %.0f mm | batt %.2f V  -> tell Claude the TAPE distance\n", NAME[i], r[0], r[1], r[2], r[3]);
    snprintf(a, sizeof a, "L %.0f R %.0f", r[0], r[1]); snprintf(b, sizeof b, "enc %.0f mm", r[2]); snprintf(c, sizeof c, "batt %.2f V", r[3]);
    if (toOled) show("ODO DONE", a, b, "measure tape mm");
  } else {        // r = mm/s L, mm/s R, avg mm/s, batt, settle ms, avg PWM %
    Serial.printf("%-8s: L %4.0f R %4.0f avg %4.0f mm/s (%4.0f rpm) | real PWM %.0f %% | steady after %.0f ms | batt %.2f V\n",
                  NAME[i], r[0], r[1], r[2], r[2] * 60 / (PI * WHEEL_MM), r[5], r[4], r[3]);
    snprintf(a, sizeof a, "%.0f mm/s", r[2]); snprintf(b, sizeof b, "L %.0f R %.0f", r[0], r[1]); snprintf(c, sizeof c, "%.0fms %.2fV", r[4], r[3]);
    if (toOled) show(NAME[i], a, b, c);
  }
}

// returns false if stopped by a button or the distance limit
bool waitDriving(uint32_t ms, float pct) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (anyBtn() || mmOf(0.5f * (cntL() + cntR())) > MAX_MM) return false;
    driveStraight(pct); delay(2);
  }
  return true;
}

bool stopAndAbort(const char *why) {
  brake(); delay(300); coast();
  Serial.printf("STOPPED: %s\n", why); show("STOPPED", why, "", "press MODE"); beep(3, 40);
  while (anyBtn()) delay(10);
  return false;
}

// both tests return false when stopped early (nothing is saved then)
bool runOdo() {
  const float target = 1000 * CPR / (PI * WHEEL_MM);   // counts for 1000 mm
  float vb = vbat();
  for (float p = 10; p < PCT[0]; p += 1) if (!waitDriving(10, p)) return stopAndAbort("button");
  while (0.5f * (cntL() + cntR()) < target) {
    if (anyBtn()) return stopAndAbort("button");
    float left = mmOf(target - 0.5f * (cntL() + cntR()));
    driveStraight(left < 200 ? max(16.0f, PCT[0] * left / 200) : PCT[0]);   // slow down over the last 200 mm
    delay(2);
  }
  brake(); delay(300); coast(); delay(500);                                 // keep counting the roll-out
  float r[6] = {(float)cntL(), (float)cntR(), mmOf(0.5f * (cntL() + cntR())), vb, 0, 0};
  nvs.putBytes("r0", r, sizeof r);
  return true;
}

bool runSpeed(int i) {
  const float pct = PCT[i];
  uint32_t t0 = millis(); float vPrev = 0;
  for (float p = 0; p < pct; p += 2) if (!waitDriving(4, p)) return stopAndAbort("button");   // ~0.2 s ramp, avoids a supply dip
  while (true) {                                    // steady = speed changes < 2 % between two 100 ms windows (max 1.2 s from start)
    int32_t a = cntL() + cntR();
    if (!waitDriving(100, pct)) return stopAndAbort(anyBtn() ? "button" : "2 m limit");
    float v = mmOf(0.5f * (cntL() + cntR() - a)) / 0.1f;
    if ((fabsf(v - vPrev) < 0.02f * v && v > 50) || millis() - t0 > 1200) break;
    vPrev = v;
  }
  float settle = millis() - t0;                      // from start, ramp included
  int32_t l0 = cntL(), r0 = cntR(); float vs = 0, ps = 0; int n = 0;
  uint32_t t1 = millis();
  while (millis() - t1 < 300) {
    if (anyBtn() || mmOf(0.5f * (cntL() + cntR())) > MAX_MM) return stopAndAbort(anyBtn() ? "button" : "2 m limit");
    driveStraight(pct); delay(2);
    if (++n % 10 == 0) vs += vbat();
    float e = KS * (cntL() - cntR()); ps += pct - 0.5f * fabsf(e);
  }
  float dt = (millis() - t1) / 1000.0f;
  float vl = mmOf(cntL() - l0) / dt, vr = mmOf(cntR() - r0) / dt;
  brake(); delay(300); coast();
  float r[6] = {vl, vr, 0.5f * (vl + vr), vs / max(1, n / 10), settle, min(pct, ps / n)};
  char k[4] = {'r', (char)('0' + i), 0};
  nvs.putBytes(k, r, sizeof r);
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  PWM_ATTACH(PIN_PWMA, 0); PWM_ATTACH(PIN_PWMB, 1); coast();
  pinMode(BTN_START, INPUT_PULLUP); pinMode(BTN_SET, INPUT_PULLUP); pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_BUZ, OUTPUT); digitalWrite(PIN_BUZ, LOW);
  pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP); pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), isrL, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_LB), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), isrR, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_RB), isrR, CHANGE);
  analogReadResolution(12);
  Wire.begin(PIN_SDA, PIN_SCL);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  nvs.begin("wtest", false);
  Serial.printf("\n=== WHEEL SPEED TEST === cpr %.1f wheel %.0f mm | p = print results, c = clear\n", CPR, WHEEL_MM);
  showMenu(); beep(1);
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'p') { Serial.println("--- saved results ---"); for (int i = 0; i < NT; i++) printResult(i, false); }
    else if (ch == 'c') { nvs.clear(); Serial.println("results cleared"); }
  }
  if (digitalRead(BTN_MODE) == LOW) { sel = (sel + 1) % NT; showMenu(); beep(1, 30); while (digitalRead(BTN_MODE) == LOW) delay(10); delay(30); }
  else if (digitalRead(BTN_START) == LOW) {
    while (digitalRead(BTN_START) == LOW) delay(10);
    show(NAME[sel], "hands off...", "", ""); beep(2); delay(1300);
    encL = encR = 0;
    if (sel == 0 ? runOdo() : runSpeed(sel)) { printResult(sel, true); beep(1, 200); }
    while (anyBtn()) delay(10);
  }
}
