/*
  TEST MOTOR JGA-N20 ENCODER 300 rpm  -  Line Robot (MKE-K01 ESP32-S3)
  -------------------------------------------------------------------
  Wiring (same as the real robot, no perfboard needed yet):
    TB6612: VM <- 2S battery (through a fuse), VCC + STBY <- 3V3, GND <- common GND
            PWMA 38, AIN2 39, AIN1 40  -> LEFT motor (AO1/AO2)  (v5)
            BIN1 41, BIN2 42, PWMB 2   -> RIGHT motor (BO1/BO2)
    Encoder: VCC <- 3V3 (NOT 5V), GND <- GND
            LEFT: A -> GPIO21, B -> GPIO47     RIGHT: A -> GPIO48, B -> GPIO1
  Arduino IDE: board "ESP32S3 Dev Module", USB CDC On Boot = Enabled, Serial Monitor 115200,
               select "Newline" at the bottom of the Serial Monitor.

  COMMANDS (type into the Serial Monitor then Enter):
    l / r / a   : select LEFT / RIGHT / BOTH motors
    0..100      : set speed % (e.g. 50). Negative = backward (e.g. -50)
    s           : stop (motor coasting)
    z           : reset the pulse count to 0
    c           : MANUAL COUNT mode: motors off, turn the wheel exactly 10 turns then type c again
    t           : automatic test: find the minimum PWM, measure rpm at 30/60/100 %
  Every 250 ms prints: PWM | left, right pulses | left, right output shaft rpm
  LIFT THE WHEELS OFF THE TABLE while running.
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

const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = 1023;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_OUT(pin, ch, d) ledcWrite(pin, d)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_OUT(pin, ch, d) ledcWrite(ch, d)
#endif

// Pulses per OUTPUT SHAFT turn (counting all 4 edges of A and B). Estimate 7 pulses x gear ratio x 4.
// Measure the real value with command 'c' then edit this number.
float COUNTS_PER_REV = 1400.0f;   // 300 rpm: estimate ~1400 -> MEASURE AGAIN with command c

volatile int32_t encL = 0, encR = 0;
volatile uint8_t prevL = 0, prevR = 0;
// quadrature decode table: [old state*4 + new state] -> +1 / -1 / 0
const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};

void IRAM_ATTR isrL() {
  uint8_t s = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB);
  encL += QDEC[(prevL << 2) | s]; prevL = s;
}
void IRAM_ATTR isrR() {
  uint8_t s = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB);
  encR -= QDEC[(prevR << 2) | s]; prevR = s;   // right motor mounted reversed -> flip the sign
}

int sel = 2;          // 0 = left, 1 = right, 2 = both
int cmdL = 0, cmdR = 0;   // % target (-100..100)
int outL = 0, outR = 0;   // PWM being output (ramped)

void motorOne(uint8_t pwm, uint8_t in1, uint8_t in2, int duty, uint8_t ch) {
  if (duty > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (duty < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); duty = -duty; }
  else               { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }   // coast: the wheels can be turned by hand
  PWM_OUT(pwm, ch, min(duty, PWM_MAX));
}
void applyOut() {
  motorOne(PIN_PWMA, PIN_AIN1, PIN_AIN2, outL, 0);
  motorOne(PIN_PWMB, PIN_BIN1, PIN_BIN2, outR, 1);
}
// ramp PWM over 30 ms so the battery voltage does not sag
void rampStep() {
  int tL = cmdL * PWM_MAX / 100, tR = cmdR * PWM_MAX / 100;
  const int STEP = PWM_MAX / 15;
  outL += constrain(tL - outL, -STEP, STEP);
  outR += constrain(tR - outR, -STEP, STEP);
  applyOut();
}
void setCmd(int pct) {
  pct = constrain(pct, -100, 100);
  if (sel == 0 || sel == 2) cmdL = pct;
  if (sel == 1 || sel == 2) cmdR = pct;
}
void stopAll() { cmdL = cmdR = outL = outR = 0; applyOut(); }

// output shaft rpm over dt ms
float rpm(int32_t dTicks, uint32_t dtMs) { return dTicks / COUNTS_PER_REV * 60000.0f / max<uint32_t>(dtMs, 1); }

void waitMs(uint32_t ms) { uint32_t t0 = millis(); while (millis() - t0 < ms) { rampStep(); delay(2); } }

// measure steady rpm at pct (both motors)
void measureAt(int pct, float &rl, float &rr) {
  cmdL = cmdR = pct; waitMs(800);                 // let it settle
  int32_t l0 = encL, r0 = encR; uint32_t t0 = millis();
  waitMs(1000);
  rl = rpm(encL - l0, millis() - t0); rr = rpm(encR - r0, millis() - t0);
}

void autoTest() {
  Serial.println("\n=== AUTO TEST (lift the wheels!) starts in 2 s ===");
  stopAll(); delay(2000);
  // 1. Minimum PWM for the wheels to start turning
  for (int side = 0; side < 2; side++) {
    int found = -1;
    for (int pct = 2; pct <= 60; pct += 2) {
      if (side == 0) { cmdL = pct; cmdR = 0; } else { cmdL = 0; cmdR = pct; }
      int32_t e0 = side == 0 ? encL : encR;
      waitMs(400);
      int32_t d = (side == 0 ? encL : encR) - e0;
      if (abs(d) > COUNTS_PER_REV / 20) { found = pct; break; }   // > 1/20 turn in 0.4 s
    }
    Serial.printf("Minimum PWM motor %s: %d %%\n", side == 0 ? "LEFT" : "RIGHT", found);
    stopAll(); delay(500);
  }
  // 2. rpm at 30/60/100 %, forward and backward
  int lv[3] = {30, 60, 100};
  for (int dir = 1; dir >= -1; dir -= 2)
    for (int i = 0; i < 3; i++) {
      float rl, rr; measureAt(dir * lv[i], rl, rr);
      Serial.printf("%4d %%  rpm left %6.1f  right %6.1f  diff %5.1f %%\n", dir * lv[i], rl, rr,
                    100.0f * (rl - rr) / max(1.0f, fabsf(rl)));
    }
  stopAll();
  Serial.println("Done. Pass when: both forward = POSITIVE rpm, backward = NEGATIVE; 2 motors differ < 10 %;");
  Serial.println("100 % at ~7.5-8 V battery: about 380-470 rpm (6 V 300 rpm motor).");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  PWM_ATTACH(PIN_PWMA, 0); PWM_ATTACH(PIN_PWMB, 1);
  stopAll();
  pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
  prevL = (digitalRead(ENC_LA) << 1) | digitalRead(ENC_LB);
  prevR = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LB), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), isrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RB), isrR, CHANGE);
  Serial.println("\n=== N20 ENCODER TEST === commands: l r a | -100..100 | s | z | c | t");
}

bool counting = false;
void loop() {
  static uint32_t tPrint = 0, tRamp = 0;
  static int32_t lastL = 0, lastR = 0;

  if (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s == "l") { sel = 0; Serial.println("-> LEFT motor"); }
    else if (s == "r") { sel = 1; Serial.println("-> RIGHT motor"); }
    else if (s == "a") { sel = 2; Serial.println("-> BOTH motors"); }
    else if (s == "s") { stopAll(); Serial.println("-> STOP"); }
    else if (s == "z") { encL = encR = 0; Serial.println("-> pulses cleared"); }
    else if (s == "t") autoTest();
    else if (s == "c") {
      if (!counting) {
        stopAll(); encL = encR = 0; counting = true;
        Serial.println("MANUAL COUNT: turn the wheel (left and/or right) exactly 10 turns FORWARD, then type c");
      } else {
        counting = false;
        Serial.printf("Left: %ld pulses / 10 turns = %.1f pulses/turn\n", (long)encL, encL / 10.0f);
        Serial.printf("Right: %ld pulses / 10 turns = %.1f pulses/turn\n", (long)encR, encR / 10.0f);
        Serial.println("POSITIVE sign = correct direction. Pulses/turn is used to fix COUNTS_PER_REV.");
        float c = max(fabsf(encL), fabsf(encR)) / 10.0f;
        if (c > 50) { COUNTS_PER_REV = c; Serial.printf("Using COUNTS_PER_REV = %.1f for now\n", c); }
      }
    }
    else if (s.length() && (isdigit(s[0]) || s[0] == '-')) { setCmd(s.toInt()); Serial.printf("-> %d %%\n", (int)s.toInt()); }
  }

  if (millis() - tRamp >= 2) { tRamp = millis(); rampStep(); }

  if (millis() - tPrint >= 250) {
    uint32_t dt = millis() - tPrint; tPrint = millis();
    int32_t l = encL, r = encR;
    if (!counting)
      Serial.printf("PWM L%4d R%4d | pulses L%8ld R%8ld | rpm L%6.1f R%6.1f\n",
                    outL, outR, (long)l, (long)r, rpm(l - lastL, dt), rpm(r - lastR, dt));
    else
      Serial.printf("manual count: L%ld  R%ld\n", (long)l, (long)r);
    lastL = l; lastR = r;
  }
}
