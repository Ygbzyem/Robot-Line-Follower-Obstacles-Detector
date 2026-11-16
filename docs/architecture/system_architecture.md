# System Architecture

## Block diagram

```
                 2S 18650 ── fuse ── switch ──┬── TB6612 VM (+470 µF) ──► 2 × N20 motors
                                              ├── MKE-B01 shield DC jack ──► ESP32-S3 (MKE-K01)
                                              └── 100k/33k divider ──► GPIO 4 (VBAT)

 ESP32-S3 3V3 rail ──► line sensor bar, encoders, MPU6050, VL53L0X, OLED, TB6612 VCC + STBY

                 ┌───────────────── ESP32-S3 ─────────────────┐
 Line sensor x8 ─┤ ADC1/ADC2   GPIO 14 13 12 11 10 7 6 5      │
 Encoders L/R ───┤ GPIO IRQ    21 47 / 48 1                   ├── PWM 20 kHz ──► TB6612 ──► motors
 MPU6050 ────────┤ I2C 400 kHz SDA 8 SCL 9                    │
 VL53L0X ────────┤ (shared bus)                               ├── GPIO 15 ──► buzzer
 OLED SSD1306 ───┤                                            │
 Buttons x3 ─────┤ GPIO 18 17 16                              ├── Wi-Fi AP "LINE-ROBOT" (idle only)
                 └────────────────────────────────────────────┘
```

Wi-Fi is turned off while running because it blocks ADC2, which reads four of the line channels.

## Firmware structure

The contest firmware is one sketch (`src/competition/Robot_Competition/Robot_Competition.ino`) running in a single Arduino `loop()` with encoder interrupts; it creates no extra FreeRTOS tasks.

```
setup()                     pins, PWM 20 kHz / 10-bit, I2C 400 kHz, NVS config, MPU6050, VL53L0X, OLED, encoder ISRs
loop()
├── uiButton() / uiLoop()   OLED pages: READY, CORNER SPEED, STRAIGHT SPEED, SENSOR, CALIBRATE, OBSTACLE AVOID, RESULT, INFO, WIFI LOG
├── handleSerial()          text commands, saved to NVS
└── runCourse()             START pressed → one run until finish / abort
    ├── update()            every 3 ms: gyro heading (encoder fallback), odometry x/y, wheel speed, spin guard
    │   ├── vlPoll()        VL53L0X distance
    │   └── traceTick()     50 ms trace for the web log
    ├── readLine()          8-channel line position, junction / finish detection
    ├── driveV()            per-wheel speed PID → TB6612
    ├── avoidObstacle()     pivot out, arc around, grabLine() / sweepFind() to re-acquire
    └── map*()              DEMO run records the track, FAST run replays it
isrL() / isrR()             quadrature decoding of both encoders (IRAM)
```

## Data flow

```
 Line sensor ──► readLine() ──position / junction──┐
 VL53L0X ──────► vlPoll() ───distance─────────────┤
 MPU6050 ──────► update() ───heading──────────────┤──► runCourse() ──speeds──► driveV() ──PWM──► TB6612
 Encoders ─────► update() ───odometry, speed──────┘                              ▲
                                                   wheel speed ──────────────────┘
 Buttons / Serial / Web ──► config (NVS) ──► runCourse() ──► OLED, event log, 50 ms trace ──► Serial / Wi-Fi page
```

## Stored data (NVS)

| Namespace | Content |
|---|---|
| `thidau` | Tuned parameters and calibration |
| `bando` | Track map recorded by the DEMO run |

The namespace names come from the first firmware and are kept so the data stored on the robot survives updates.
