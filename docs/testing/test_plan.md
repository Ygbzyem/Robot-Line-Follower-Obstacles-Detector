# Test Plan

Bring up the robot in this order; each test checks one concern before the next layer depends on it. Flash a test with:

```sh
PLATFORMIO_SRC_DIR=src/tests/hardware/Test_Power pio run -t upload && pio device monitor
```

| Order | Sketch | Folder | Checks | Pass criterion |
|---:|---|---|---|---|
| 1 | Test_Power | `src/tests/hardware/` | Battery voltage, brownout | No BROWNOUT reset, lowest VBAT > 7.0 V |
| 2 | Test_Buttons | `src/tests/hardware/` | 3 buttons, contact bounce | Each press counted once |
| 3 | Test_OLED | `src/tests/hardware/` | OLED controller and wiring | Test pattern shows all 4 borders |
| 4 | Test_MPU_OLED_Buttons | `src/tests/hardware/` | MPU6050 + OLED + buttons | Lying flat: aZ ≈ 1 g, gyro ≈ 0 |
| 5 | Test_TB6612 | `src/tests/hardware/` | Driver outputs | Measured AO/BO voltages match each step |
| 6 | Test_RightMotor, Test_N20 | `src/tests/hardware/` | Motor direction, encoder count | Positive count forward, CPR measured |
| 7 | Test_FullSpeed10s | `src/tests/hardware/` | Both motors at 100 % | Both rpm positive, mismatch < 10 % |
| 8 | Test_LineSensor | `src/tests/hardware/` | Readout, noise, latency | All 8 channels differ > 1500 black/white |
| 9 | Test_DriveStraight_Simple, Test_DriveStraight | `src/tests/hardware/` | Straight driving | Heading error < 3°, distance error < 3 % |
| 10 | Test_Integration | `src/tests/integration/` | Motors, encoders, IMU, OLED, buttons together | All test steps pass |
| 11 | Test_FullRobot | `src/tests/integration/` | Every module and the whole robot | Self test reports all PASS |

After the tests, flash the contest firmware and run the first-run steps in the main README.
