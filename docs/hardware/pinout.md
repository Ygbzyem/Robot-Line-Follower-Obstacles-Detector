# Pinout

Pin map v5, used by the contest firmware and the newer tests.

| Signal | GPIO | Peripheral | Note |
|---|---|---|---|
| PWMA / AIN2 / AIN1 | 38 / 39 / 40 | TB6612 channel A | Left motor |
| BIN1 / BIN2 / PWMB | 41 / 42 / 2 | TB6612 channel B | Right motor; PWMA/PWMB have 10k to GND |
| STBY | 3V3 | TB6612 | Tied high |
| ENC_LA / ENC_LB | 21 / 47 | GPIO interrupt | Left encoder, VCC 3V3 |
| ENC_RA / ENC_RB | 48 / 1 | GPIO interrupt | Right encoder, VCC 3V3 |
| D1 … D8 | 14, 13, 12, 11, 10, 7, 6, 5 | ADC | Line sensor, D1 = left; 11–14 are on ADC2 |
| SDA / SCL | 8 / 9 | I2C 400 kHz | MPU6050 0x68, VL53L0X 0x29, OLED 0x3C |
| START / SET / MODE | 18 / 17 / 16 | GPIO input (pull-up) | Buttons to GND |
| BUZ | 15 | GPIO output | Buzzer |
| VBAT | 4 | ADC | Divider 100k/33k, ratio 4.33 |

Do not take power for the modules from the 5 V pin in the middle of the shield's G-V-S headers; all modules run from 3V3.

The prototypes in `src/prototypes/` and some tests use an older pin map; it is listed in the header comment of each sketch.

Wiring diagram: [`hardware/schematics/`](../../hardware/schematics/).
