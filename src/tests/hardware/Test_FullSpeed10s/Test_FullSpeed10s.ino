/*
  TEST 2 MOTORS AT MAX (100%) FOR 10 SECONDS  -  Line Robot (MKE-K01 ESP32-S3)
  --------------------------------------------------------------------------
  LIFT BOTH WHEELS OFF THE TABLE before connecting the battery!
  Wiring same as Test_N20:
    TB6612: VM <- battery (+) (+470uF), GND <- battery (-), VCC + STBY <- 3V3
            PWMA 38, AIN2 39, AIN1 40 -> LEFT motor | BIN1 41, BIN2 42, PWMB 2 -> RIGHT motor (v5)
    Encoder: VCC 3V3 (3V3 rail, NOT 5V), GND | LEFT A21 B47 | RIGHT A48 B1
    ESP32 on USB -> ALSO CONNECT ESP32 GND to the driver GND.
  Serial Monitor 115200, "Newline".
  Run: 3 s countdown -> accelerate 0.5 s -> 100% for 10 s -> stop.
  Run again: type r (or press the BOOT button). Emergency stop: type s or press BOOT.
*/
#include <Arduino.h>

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
#endif
#define BTN_BOOT 0

float COUNTS_PER_REV = 1400.0f;   // 300 rpm motor (estimate) - edit to the value measured with Test_N20 command c
const float WHEEL_D_MM = 43.0f;
const uint32_t RUN_MS = 10000;

const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = 1023;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_OUT(pin, ch, d) ledcWrite(pin, d)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_OUT(pin, ch, d) ledcWrite(ch, d)
#endif

volatile int32_t encL = 0, encR = 0;
volatile uint8_t prevL = 0, prevR = 0;
const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};
void IRAM_ATTR isrL() { uint8_t s = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB); encL += QDEC[(prevL << 2) | s]; prevL = s; }
void IRAM_ATTR isrR() { uint8_t s = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB); encR -= QDEC[(prevR << 2) | s]; prevR = s; }

void motor(uint8_t pwm, uint8_t in1, uint8_t in2, int duty, uint8_t ch) {
  digitalWrite(in1, duty > 0); digitalWrite(in2, LOW);   // forward only
  PWM_OUT(pwm, ch, constrain(duty, 0, PWM_MAX));
}
void both(int duty) { motor(PIN_PWMA, PIN_AIN1, PIN_AIN2, duty, 0); motor(PIN_PWMB, PIN_BIN1, PIN_BIN2, duty, 1); }
void stopAll() { both(0); }

bool stopRequested() {
  if (digitalRead(BTN_BOOT) == LOW) return true;
  if (Serial.available()) { char c = Serial.read(); if (c == 's' || c == 'S') return true; }
  return false;
}

void runTest() {
  Serial.println("\n=== MAX SPEED 10 SECONDS - LIFT THE WHEELS! ===");
  for (int i = 3; i > 0; i--) { Serial.printf("Starting in %d...\n", i); delay(1000); }
  while (Serial.available()) Serial.read();
  delay(300);   // ignore a BOOT button still being held

  encL = encR = 0;
  uint32_t t0 = millis(), tP = t0;
  int32_t lastL = 0, lastR = 0, l10 = 0, r10 = 0;   // pulses when the 100% segment started
  float maxL = 0, maxR = 0, minL = 1e9, minR = 1e9;
  bool fullStarted = false, aborted = false;

  while (millis() - t0 < 500 + RUN_MS) {
    uint32_t el = millis() - t0;
    both(el < 500 ? (int)(PWM_MAX * el / 500) : PWM_MAX);   // accelerate for 0.5 s then 100%
    if (!fullStarted && el >= 500) { fullStarted = true; l10 = encL; r10 = encR; }
    if (stopRequested()) { aborted = true; break; }
    if (millis() - tP >= 500) {
      uint32_t dt = millis() - tP; tP = millis();
      int32_t l = encL, r = encR;
      float rl = (l - lastL) / COUNTS_PER_REV * 60000.0f / dt, rr = (r - lastR) / COUNTS_PER_REV * 60000.0f / dt;
      lastL = l; lastR = r;
      if (fullStarted && el > 1500) { maxL = max(maxL, rl); maxR = max(maxR, rr); minL = min(minL, rl); minR = min(minR, rr); }
      Serial.printf("t %5.1f s | PWM %3d%% | rpm L %6.1f R %6.1f | pulses L %7ld R %7ld\n",
                    el / 1000.0f, el < 500 ? (int)(100 * el / 500) : 100, rl, rr, (long)l, (long)r);
    }
  }
  uint32_t tFull = millis() - t0 > 500 ? millis() - t0 - 500 : 1;
  stopAll();

  float revL = (encL - l10) / COUNTS_PER_REV, revR = (encR - r10) / COUNTS_PER_REV;
  float avgL = revL * 60000.0f / tFull, avgR = revR * 60000.0f / tFull;
  float k = PI * WHEEL_D_MM / 60.0f;   // rpm -> mm/s
  Serial.println(aborted ? "\n=== EMERGENCY STOP ===" : "\n=== RESULT 10 SECONDS 100% ===");
  Serial.printf("LEFT : AVG %6.1f rpm (%4.0f mm/s) | min %6.1f max %6.1f | %.1f turns\n", avgL, avgL * k, minL, maxL, revL);
  Serial.printf("RIGHT: AVG %6.1f rpm (%4.0f mm/s) | min %6.1f max %6.1f | %.1f turns\n", avgR, avgR * k, minR, maxR, revR);
  Serial.printf("Wheel mismatch: %.1f %%\n", 100.0f * (avgL - avgR) / max(1.0f, fabsf(avgL)));
  Serial.println("Pass: both POSITIVE, ~380-470 rpm (full battery), diff < 10%, steady rpm (min~max).");
  Serial.println("After the test: touch the motors and TB6612 - warm is normal, too HOT to touch means a problem.");
  Serial.println("Type r (or press BOOT) to run again.");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  PWM_ATTACH(PIN_PWMA, 0); PWM_ATTACH(PIN_PWMB, 1);
  stopAll();
  pinMode(BTN_BOOT, INPUT_PULLUP);
  pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP); pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
  prevL = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB);
  prevR = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), isrL, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_LB), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), isrR, CHANGE); attachInterrupt(digitalPinToInterrupt(ENC_RB), isrR, CHANGE);
  delay(1500);
  runTest();
}

void loop() {
  bool again = digitalRead(BTN_BOOT) == LOW;
  if (Serial.available()) { char c = Serial.read(); if (c == 'r' || c == 'R') again = true; }
  if (again) { delay(300); runTest(); }
}
