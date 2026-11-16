/*
  TB6612FNG BOARD TEST (no encoder needed) - Line Robot, MKE-K01 ESP32-S3
  ------------------------------------------------------------------
  Wiring:
    VM   <- battery (+) through a fuse      GND  <- battery (-) AND ESP32 GND (common GND REQUIRED)
    VCC  <- 3V3 ESP32                STBY <- 3V3 ESP32
    PWMA <- GPIO38  AIN1 <- GPIO39  AIN2 <- GPIO40   -> AO1/AO2: motor (or leave open to measure voltage)
    PWMB <- GPIO41  BIN1 <- GPIO42  BIN2 <- GPIO16   -> BO1/BO2: motor (or leave open)
  ESP32 powered from the USB cable. Serial Monitor 115200.

  The code repeats 6 steps, 4 SECONDS PER STEP (enough time to measure with a meter).
  Each step prints: which pins must be 3.3 V, which 0 V, and the AO1-AO2 / BO1-BO2 voltage you should measure.
*/
#include <Arduino.h>

#define PWMA 38
#define AIN1 39
#define AIN2 40
#define PWMB 41
#define BIN1 42
#define BIN2 16

struct Step { const char* name; int pa, a1, a2, pb, b1, b2; const char* expect; };
const Step STEPS[] = {
  {"1. ALL OFF",            0,0,0, 0,0,0, "AO1-AO2 = 0 V, BO1-BO2 = 0 V, motors stopped"},
  {"2. CHANNEL A FORWARD",    1,1,0, 0,0,0, "AO1-AO2 = +VM (~7-8 V), motor A turns; BO = 0 V"},
  {"3. CHANNEL A BACKWARD",    1,0,1, 0,0,0, "AO1-AO2 = -VM (reversed), motor A turns backward"},
  {"4. CHANNEL B FORWARD",    0,0,0, 1,1,0, "BO1-BO2 = +VM, motor B turns; AO = 0 V"},
  {"5. CHANNEL B BACKWARD",    0,0,0, 1,0,1, "BO1-BO2 = -VM, motor B turns backward"},
  {"6. BOTH FORWARD",    1,1,0, 1,1,0, "both channels = +VM, both motors turn"},
};

void apply(const Step& s) {
  digitalWrite(PWMA, s.pa); digitalWrite(AIN1, s.a1); digitalWrite(AIN2, s.a2);
  digitalWrite(PWMB, s.pb); digitalWrite(BIN1, s.b1); digitalWrite(BIN2, s.b2);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  int pins[] = {PWMA, AIN1, AIN2, PWMB, BIN1, BIN2};
  for (int p : pins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  Serial.println("\n=== TB6612 TEST: 4 s per step, meter BLACK probe on GND ===");
  Serial.println("Measure first: VM ~7-8 V, VCC = 3.3 V, STBY = 3.3 V.");
}

void loop() {
  for (const Step& s : STEPS) {
    apply(s);
    Serial.printf("\n%s\n  PWMA=%s AIN1=%s AIN2=%s | PWMB=%s BIN1=%s BIN2=%s\n  Expect: %s\n",
      s.name, s.pa ? "3V3" : "0V", s.a1 ? "3V3" : "0V", s.a2 ? "3V3" : "0V",
      s.pb ? "3V3" : "0V", s.b1 ? "3V3" : "0V", s.b2 ? "3V3" : "0V", s.expect);
    delay(4000);
  }
}
