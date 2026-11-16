/*
  BUTTON TEST - Line Robot (ESP32-S3)  - tests the 3 buttons only, no motor/I2C needed
  -----------------------------------------------------------------------
  Wiring: one button pin -> GPIO, the other pin (OPPOSITE side / diagonal) -> GND.
       DO NOT connect a button to the V (5V) pin in the middle of the shield's G-V-S header.
  Serial Monitor 115200.

  Every press / release prints:  button name, press count, hold time, bounce count.
  The buzzer beeps once per press (if a buzzer is on pin 15).
  Verdict:
    - One press with "bounce" > 5, or one press giving 2-3 presses -> poor contact / bad solder joint
    - Pressing prints nothing -> broken wire, wrong pin, or the button tied to 5V instead of GND
    - Startup reports "= 0 when not pressed" -> shorted to GND or a solder bridge keeps the button closed
  Serial commands:  r = reset counters | p = print current state | shake = 10 s wire shake test
*/
#include <Arduino.h>

#define PIN_MAP 2   // 2 = new pins v5 | 1 = old pins (breadboard)
#if PIN_MAP == 2
const uint8_t PIN[3] = {18, 17, 16};   // START, SET, MODE
#define PIN_BUZ 15
#else
const uint8_t PIN[3] = {17, 18, 0};
#define PIN_BUZ 47
#endif
const char* NAME[3] = {"START", "SET  ", "MODE "};
const uint32_t DEBOUNCE_MS = 25;

bool stable[3], lastRaw[3];
uint32_t tChange[3], tPress[3];
uint32_t nPress[3], nBounce[3], bounceNow[3];

void printState() {
  for (int i = 0; i < 3; i++)
    Serial.printf("  %s GPIO%-2d = %d (%s) | pressed %lu times\n", NAME[i], PIN[i], digitalRead(PIN[i]),
                  digitalRead(PIN[i]) ? "released" : "PRESSED", (unsigned long)nPress[i]);
}

void beepShort() { digitalWrite(PIN_BUZ, HIGH); delay(15); digitalWrite(PIN_BUZ, LOW); }

// Wire shake: count every level change (unfiltered) for 10 s -> hold the button still, just shake/press the wires and solder joints lightly
void shakeTest() {
  Serial.println("\n== 10 s WIRE SHAKE TEST: DO NOT press buttons, just shake the wires / press on the solder joints ==");
  uint32_t cnt[3] = {0, 0, 0}; bool prev[3];
  for (int i = 0; i < 3; i++) prev[i] = digitalRead(PIN[i]);
  uint32_t t0 = millis();
  while (millis() - t0 < 10000) {
    for (int i = 0; i < 3; i++) { bool v = digitalRead(PIN[i]); if (v != prev[i]) { prev[i] = v; cnt[i]++; } }
  }
  for (int i = 0; i < 3; i++)
    Serial.printf("  %s: %lu level changes %s\n", NAME[i], (unsigned long)cnt[i], cnt[i] ? "<- BAD CONTACT when shaken: resolder this button's wire/joint" : "OK");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZ, OUTPUT); digitalWrite(PIN_BUZ, LOW);
  for (int i = 0; i < 3; i++) pinMode(PIN[i], INPUT_PULLUP);
  delay(1500);
  Serial.println("\n=== BUTTON TEST ===");
  for (int i = 0; i < 3; i++) {
    stable[i] = lastRaw[i] = digitalRead(PIN[i]);
    if (!stable[i]) Serial.printf("!! %s (GPIO%d) = 0 when NOT pressed -> shorted to GND / solder bridge, always closed\n", NAME[i], PIN[i]);
  }
  printState();
  Serial.println("Press each button in turn, 5 times each. Commands: r | p | shake");
}

void loop() {
  uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    bool raw = digitalRead(PIN[i]);
    if (raw != lastRaw[i]) { lastRaw[i] = raw; tChange[i] = now; bounceNow[i]++; }
    if (raw != stable[i] && now - tChange[i] >= DEBOUNCE_MS) {
      stable[i] = raw;
      uint32_t b = bounceNow[i] > 1 ? bounceNow[i] - 1 : 0;
      nBounce[i] += b; bounceNow[i] = 0;
      if (!raw) {
        nPress[i]++; tPress[i] = now; beepShort();
        Serial.printf("[PRESS] %s #%lu | bounce %lu%s\n", NAME[i], (unsigned long)nPress[i], (unsigned long)b,
                      b > 5 ? "  <- heavy bounce: poor contact" : "");
      } else {
        uint32_t held = now - tPress[i];
        Serial.printf("[REL ] %s held %lu ms | bounce %lu%s\n", NAME[i], (unsigned long)held, (unsigned long)b,
                      held < 40 ? "  <- too short: may be noise/bad contact, not a real press" : "");
      }
    }
  }
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s == "r") { for (int i = 0; i < 3; i++) nPress[i] = nBounce[i] = 0; Serial.println("Counters reset"); }
    else if (s == "p") printState();
    else if (s == "shake") shakeTest();
  }
}
