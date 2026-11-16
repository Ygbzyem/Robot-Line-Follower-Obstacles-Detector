/*
  RIGHT MOTOR TEST (JGA-N20 encoder) - Line Robot, MKE-K01 ESP32-S3
  ---------------------------------------------------------------
  Wiring:
    TB6612: VM  <- 2S battery (through a fuse) + 470uF capacitor right at the VM pin
            VCC, STBY <- 3V3      GND <- common GND (battery -, shield GND)
            PWMB <- GPIO41   BIN1 <- GPIO42   BIN2 <- GPIO16
            BO1, BO2 -> 2 motor wires (M1, M2)
    Right motor encoder: VCC <- 3V3 (NOT 5V), GND <- GND
                        A (C1) -> GPIO14     B (C2) -> GPIO15
  Arduino IDE: "ESP32S3 Dev Module", USB CDC On Boot = Enabled, Serial Monitor 115200 + "Newline".

  Run: LIFT THE WHEELS OFF THE TABLE. Open the Serial Monitor, the code runs one test loop by itself:
        forward 30% -> forward 60% -> forward 100% -> stop -> backward 60% -> stop, 2 s per step.
  Then type commands:
     number -100..100 : run at that % (negative = backward)     s : stop
     z            : clear the pulse count                  c : manual count (turn the wheel 10 turns then type c again)
     t            : rerun the automatic test loop
*/
#include <Arduino.h>

#define PIN_PWMB 41
#define PIN_BIN1 42
#define PIN_BIN2 16
#define ENC_A    14
#define ENC_B    15

const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = 1023;
float COUNTS_PER_REV = 848.0f;    // 500 rpm: estimate -> MEASURE AGAIN with command c

volatile int32_t enc = 0;
volatile uint8_t prevState = 0;
const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};

void IRAM_ATTR isrEnc() {
  uint8_t s = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  enc += QDEC[(prevState << 2) | s];
  prevState = s;
}

int target = 0;   // target PWM (-1023..1023)
int out = 0;      // PWM being output (ramped to avoid voltage sag)

void writeMotor(int duty) {
  if (duty > 0)      { digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW); }
  else if (duty < 0) { digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, HIGH); duty = -duty; }
  else               { digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, LOW); }   // coast
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_PWMB, min(duty, PWM_MAX));
#else
  ledcWrite(1, min(duty, PWM_MAX));
#endif
}

void setPercent(int pct) { target = constrain(pct, -100, 100) * PWM_MAX / 100; }

// called continuously: raise/lower PWM in small steps, print data every 250 ms
void service() {
  static uint32_t tRamp = 0, tPrint = 0;
  static int32_t last = 0;
  if (millis() - tRamp >= 2) {
    tRamp = millis();
    out += constrain(target - out, -PWM_MAX / 15, PWM_MAX / 15);
    writeMotor(out);
  }
  if (millis() - tPrint >= 250) {
    uint32_t dt = millis() - tPrint; tPrint = millis();
    int32_t e = enc;
    float rpm = (e - last) / COUNTS_PER_REV * 60000.0f / dt;
    last = e;
    Serial.printf("PWM %5d (%4d%%) | pulses %8ld | rpm %7.1f\n", out, out * 100 / PWM_MAX, (long)e, rpm);
  }
}

void runFor(int pct, uint32_t ms) {
  Serial.printf("--- %d %% for %lu ms ---\n", pct, (unsigned long)ms);
  setPercent(pct);
  uint32_t t0 = millis();
  while (millis() - t0 < ms) service();
}

void autoTest() {
  Serial.println("\n=== AUTO TEST LOOP (lift the wheels) ===");
  int32_t e0 = enc;
  runFor(30, 2000);
  runFor(60, 2000);
  runFor(100, 2000);
  runFor(0, 1000);
  int32_t fwd = enc - e0;
  runFor(-60, 2000);
  runFor(0, 1000);
  Serial.println("=== RESULT ===");
  Serial.printf("Driving forward: pulses increase %ld -> %s\n", (long)fwd, fwd > 0 ? "CORRECT direction" : "WRONG: swap the 2 encoder wires A/B");
  Serial.println("Watch the wheel: drives forward but wheel turns backward -> swap the 2 motor wires at BO1/BO2.");
  Serial.println("100% at ~7.5-8 V battery: 650-750 rpm is normal.");
  Serial.println("Commands: -100..100 | s | z | c | t");
}

bool counting = false;

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(1, PWM_FREQ, PWM_BITS); ledcAttachPin(PIN_PWMB, 1);
#endif
  writeMotor(0);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  prevState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrEnc, CHANGE);
  Serial.println("\n=== RIGHT MOTOR TEST ===");
  autoTest();
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s == "s") { setPercent(0); Serial.println("-> stop"); }
    else if (s == "z") { enc = 0; Serial.println("-> pulses cleared"); }
    else if (s == "t") autoTest();
    else if (s == "c") {
      if (!counting) {
        target = out = 0; writeMotor(0); counting = true; enc = 0;
        Serial.println("MANUAL COUNT: turn the wheel exactly 10 turns FORWARD, then type c");
      } else {
        counting = false;
        Serial.printf("%ld pulses / 10 turns = %.1f pulses/turn (positive = correct direction)\n", (long)enc, enc / 10.0f);
        if (abs(enc) > 500) { COUNTS_PER_REV = fabsf(enc / 10.0f); Serial.printf("Using COUNTS_PER_REV = %.1f for now\n", COUNTS_PER_REV); }
      }
    }
    else if (s.length() && (isdigit(s[0]) || s[0] == '-')) {
      int pct = s.toInt(); setPercent(pct); Serial.printf("-> %d %%\n", pct);
    }
  }
  if (counting) {
    static uint32_t t = 0;
    if (millis() - t > 250) { t = millis(); Serial.printf("manual count: %ld\n", (long)enc); }
  } else {
    service();
  }
}
