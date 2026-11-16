/*
  SIMPLE DRIVE STRAIGHT TEST - 2 MOTORS + ENCODERS ONLY (OLD PINS, no MPU/buttons/OLED needed)
  ---------------------------------------------------------------------------------
  Wiring (same as when Test_FullSpeed10s worked):
    TB6612: VM <- battery (+), GND <- battery (-), VCC + STBY <- 3V3
            PWMA 38, AIN1 39, AIN2 40 -> LEFT motor (AO1/AO2)
            PWMB 41, BIN1 42, BIN2 16 -> RIGHT motor (BO1/BO2)
    Encoder: VCC 3V3, GND | LEFT A12 B13 | RIGHT A14 B15
    ESP32 on USB -> CONNECT ESP32 GND to the driver GND.
  Serial Monitor 115200.

  Run: power on -> 3 s countdown -> robot drives straight 2 s at 100% (~1.8 m) -> stops, prints the result.
  Serial commands:  g = run again | s = stop | number 20..100 = change speed (e.g. 50) | t3 = time 3 s
  BOOT button on the board: press = run again / stop.

  Keeps straight with the ENCODERS: the wheel that turns more is slowed, the other one sped up.
  If one wheel TURNS BACKWARD (robot spins): set REV_L or REV_R below to true and upload again.
*/
#include <Arduino.h>

// ================= EDIT HERE =================
bool REV_L = true;         // true = reverse the LEFT motor (CONFIRMED: left wheel reversed)
bool REV_R = false;        // true = reverse the RIGHT motor
int  SPEED = 100;          // % speed (100% ~ 0.9 m/s -> needs ~2 m of free floor)
float RUN_S = 2.0f;        // run time in seconds (100% x 2 s ~ 1.8 m)
float KP = 0.6f;           // straight-hold gain (% per 10 pulses of difference)
// ===============================================

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
#define BTN_BOOT 0

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
void IRAM_ATTR isrR() { uint8_t s = (digitalRead(ENC_RA) << 1) | digitalRead(ENC_RB); encR += QDEC[(prevR << 2) | s]; prevR = s; }

// pct: 0..100, forward only
void motorL(float pct) {
  int d = constrain((int)(pct * PWM_MAX / 100), 0, PWM_MAX);
  digitalWrite(PIN_AIN1, REV_L ? LOW : HIGH); digitalWrite(PIN_AIN2, REV_L ? HIGH : LOW);
  if (d == 0) { digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); }
  PWM_OUT(PIN_PWMA, 0, d);
}
void motorR(float pct) {
  int d = constrain((int)(pct * PWM_MAX / 100), 0, PWM_MAX);
  digitalWrite(PIN_BIN1, REV_R ? LOW : HIGH); digitalWrite(PIN_BIN2, REV_R ? HIGH : LOW);
  if (d == 0) { digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW); }
  PWM_OUT(PIN_PWMB, 1, d);
}
void stopAll() { motorL(0); motorR(0); }
void brakeAll() {
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH); PWM_OUT(PIN_PWMA, 0, PWM_MAX);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH); PWM_OUT(PIN_PWMB, 1, PWM_MAX);
}

bool stopReq() {
  if (digitalRead(BTN_BOOT) == LOW) return true;
  if (Serial.available()) { char c = Serial.peek(); if (c == 's' || c == 'S') { Serial.read(); return true; } }
  return false;
}

void runStraight() {
  Serial.printf("\n=== STRAIGHT %d%% for %.1f s ===\n", SPEED, RUN_S);
  for (int i = 3; i > 0; i--) { Serial.printf("%d...\n", i); delay(1000); }
  while (digitalRead(BTN_BOOT) == LOW) delay(10);   // wait for the BOOT button release
  int32_t l0 = encL, r0 = encR;
  uint32_t t0 = millis(), tP = 0;
  bool aborted = false;
  float maxDiff = 0;
  while (millis() - t0 < (uint32_t)(RUN_S * 1000)) {
    if (stopReq()) { aborted = true; break; }
    // distance per wheel (absolute value -> independent of the encoder sign)
    long dL = labs((long)(encL - l0)), dR = labs((long)(encR - r0));
    float diff = (float)(dL - dR);            // positive: left wheel traveled more
    maxDiff = max(maxDiff, fabsf(diff));
    float base = min((float)SPEED, 15.0f + (millis() - t0) * 0.15f);   // gradual acceleration ~0.6 s up to 100%
    float corr = constrain(KP * diff / 10.0f, -base * 0.5f, base * 0.5f);
    float pl = base - corr, pr = base + corr;
    float over = max(pl, pr) - 100.0f;          // at 100%: cannot raise the slow wheel -> lower the fast wheel
    if (over > 0) { pl -= over; pr -= over; }
    motorL(pl);
    motorR(pr);
    if (millis() - tP >= 200) {
      tP = millis();
      Serial.printf("t %4.1f | pulses L %7ld R %7ld | diff %+6.0f | PWM L %5.1f R %5.1f\n",
                    (millis() - t0) / 1000.0f, dL, dR, diff, pl, pr);
    }
    delay(2);
  }
  brakeAll(); delay(200); stopAll();
  long dL = labs((long)(encL - l0)), dR = labs((long)(encR - r0));
  Serial.println(aborted ? "=== STOPPED ===" : "=== RESULT ===");
  Serial.printf("Pulses: L %ld  R %ld  | final diff %ld (%.1f %%), max diff %.0f\n",
                dL, dR, dL - dR, 100.0f * (dL - dR) / max(1L, dL), maxDiff);
  if (dL < 50) Serial.println("!! LEFT encoder not counting: check wires 12/13, VCC 3V3");
  if (dR < 50) Serial.println("!! RIGHT encoder not counting: check wires 14/15, VCC 3V3");
  Serial.println("Robot spins in place? -> 1 motor reversed: flip REV_L or REV_R.  Robot drives backward? -> flip BOTH.");
  Serial.println("Commands: g = run again | number 20..100 = speed | t<seconds> = time | s = stop");
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
  Serial.println("\n=== SIMPLE DRIVE STRAIGHT TEST (old pins) ===");
  runStraight();
}

void loop() {
  if (digitalRead(BTN_BOOT) == LOW) { delay(300); runStraight(); }
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s == "g" || s == "G") runStraight();
    else if (s.length() > 1 && (s[0] == 't' || s[0] == 'T')) { RUN_S = constrain(s.substring(1).toFloat(), 0.5f, 20.0f); Serial.printf("-> run %.1f s\n", RUN_S); }
    else if (s.length() && isdigit(s[0])) { SPEED = constrain(s.toInt(), 15, 100); Serial.printf("-> speed %d%%\n", SPEED); }
  }
}
