/*
  POWER TEST - Line Robot (MKE-K01 board + MKE-B01 shield)
  - Prints battery voltage every 200 ms (divider 100k/33k on GPIO11)
  - Reports the previous reset reason: "BROWNOUT" means the supply sagged while the motors ran
  - NO BMS: stops by itself when the battery < 6.6 V for 2 s
  - Press START (GPIO17): both motors forward 0.4 s -> reverse 0.4 s, repeated 60 times (~50 s)
    LIFT THE WHEELS OFF THE TABLE before pressing.
  Arduino IDE: board "ESP32S3 Dev Module". Serial 115200.
*/
#include <Arduino.h>
#include "esp_system.h"

#define PIN_VBAT      11
#define PIN_BTN_START 17
#define PIN_PWMA 38
#define PIN_AIN1 39
#define PIN_AIN2 40
#define PIN_PWMB 41
#define PIN_BIN1 42
#define PIN_BIN2 16

const float VBAT_CUTOFF = 6.6f;   // no BMS: below 3.3 V/cell for 2 s continuously -> stop the motors
const int PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = 1023;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_BITS)
  #define PWM_OUT(pin, ch, d) ledcWrite(pin, d)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); } while (0)
  #define PWM_OUT(pin, ch, d) ledcWrite(ch, d)
#endif

float vbat() { return analogReadMilliVolts(PIN_VBAT) * (133.0f / 33.0f) / 1000.0f; }

void motors(int dir, int duty) {          // dir = +1 forward, -1 backward, 0 stop
  digitalWrite(PIN_AIN1, dir > 0); digitalWrite(PIN_AIN2, dir < 0);
  digitalWrite(PIN_BIN1, dir > 0); digitalWrite(PIN_BIN2, dir < 0);
  PWM_OUT(PIN_PWMA, 0, dir ? duty : 0);
  PWM_OUT(PIN_PWMB, 1, dir ? duty : 0);
}

const char* resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "POWERON (normal power-up)";
    case ESP_RST_BROWNOUT: return "BROWNOUT (power sagged - check battery/wires/capacitor)";
    case ESP_RST_SW:       return "SOFTWARE";
    case ESP_RST_PANIC:    return "PANIC (code error)";
    case ESP_RST_USB:      return "USB";
    default:               return "OTHER";
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n=== POWER TEST ===\nLast reset reason: %s\n", resetName(esp_reset_reason()));
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  PWM_ATTACH(PIN_PWMA, 0); PWM_ATTACH(PIN_PWMB, 1);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  motors(0, 0);
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t > 200) {
    t = millis();
    Serial.printf("VBAT = %.2f V\n", vbat());
  }
  if (digitalRead(PIN_BTN_START) == LOW) {
    Serial.println("Starting load test: forward/back 60 times ...");
    float vmin = 99; uint32_t lowSince = 0; bool stop = false;
    for (int i = 0; i < 60; i++) {
      for (int d = 1; d >= -1; d -= 2) {
        motors(d, PWM_MAX);
        uint32_t t0 = millis();
        while (millis() - t0 < 400) {
          float v = vbat(); if (v < vmin) vmin = v;
          if (v < VBAT_CUTOFF) { if (!lowSince) lowSince = millis(); else if (millis() - lowSince > 2000) stop = true; }
          else lowSince = 0;
        }
        if (stop) break;
      }
      if (stop) { Serial.println("BATTERY FLAT (< 6.6 V for 2 s) -> load test stopped. Recharge each cell."); break; }
    }
    motors(0, 0);
    Serial.printf("Done. Lowest VBAT measured = %.2f V (needs > 7.0 V)\n", vmin);
    while (digitalRead(PIN_BTN_START) == LOW) delay(10);
  }
}
