# Source Code

**English** | [Tiếng Việt](vn.md)

All firmware of the robot. Every sketch sits in a folder with the same name, so it can be opened in the Arduino IDE or built with PlatformIO.

```
src/
├── competition/    code that runs on the robot
├── archive/        earlier contest versions
├── prototypes/     development firmware written before the contest code
└── tests/          bring-up sketches, never used in a run
    ├── hardware/
    └── integration/
```

## Which sketch to flash

| Goal | Sketch |
|---|---|
| Race on the track | `competition/Robot_Competition` |
| Check one part after wiring or repair | the matching sketch in `tests/hardware/` |
| Check the whole robot before a run | `tests/integration/Test_FullRobot` |
| Compare with an older contest behavior | a folder in `archive/` |

PlatformIO builds `competition/Robot_Competition` by default. Build any other sketch by overriding the source folder from the repository root:

```sh
PLATFORMIO_SRC_DIR=src/tests/hardware/Test_LineSensor pio run -t upload
```

## competition/

| Folder | Version | Description |
|---|---|---|
| `Robot_Competition` | C3.13 | Contest firmware currently flashed on the robot (defaults for the new motors, gearbox ~20.7:1): line following, junction clusters, obstacle avoidance, DEMO/FAST map modes, OLED menu, Serial and Wi-Fi tuning. |

Only this folder is used in a race. New contest versions replace the file here; the previous file moves to `archive/`.

## archive/

Earlier contest versions, kept to compare behavior or roll back. They are not flashed any more.

| Folder | Version | Main change |
|---|---|---|
| `Robot_Competition_v31` | v3.1 | Run modes NORMAL, DEMO (records the track map) and FAST (replays the map) |
| `Robot_Competition_A` | v3.5.1 | Per-wheel speed PID, parameter export, flowing obstacle avoidance |
| `Robot_Competition_B` | v3.9.1 | Parallel version B with fast single 90-degree corners |
| `Robot_Competition_C` | v3.8 | Version C with wrong-way guard, base of C3.x |
| `Robot_Competition_C311b` | C3.11b | Best version for 500 rpm motors: hexagon-cluster heading lock, straight start out of the black box |
| `Robot_Competition_C39` | C3.9 | Circle-cluster exit, avoidance toward the bend, line search in the right direction, anti-stall pivot |

## prototypes/

Firmware written while the robot and the track rules were still changing. Each one introduced pieces reused in the contest code.

| Folder | Version | Description | Status |
|---|---|---|---|
| `Robot_Skeleton_v02` | v0.2 | First skeleton for ESP32-S3 DevKitC-1: 3 × VL53L0X, modes FOLLOW / EXPLORE / FAST / SAFE / CALIB, maze map from odometry | Does not compile (see main README) |
| `Robot_MKE_v03` | v0.3 | Port to the MKE-K01 board for the winding track, no encoders | Does not compile (see main README) |
| `Robot_LineFollow_v5` | v1.0 | First PD line follower on pin map v5, with sensor calibration and Serial tuning | Builds |
| `Robot_RouteRun` | v1.2 | Drives pre-programmed routes without a line (straight, out and back, square, obstacle avoid) using encoders and the gyro | Builds |

## tests/

Bring-up sketches. Each one checks a single concern and prints the result on Serial (115200) and, where an OLED is used, on the screen. Run them in the order of [docs/testing/test_plan.md](../docs/testing/test_plan.md).

### tests/hardware/

| Folder | Checks |
|---|---|
| `Test_Power` | Battery voltage, previous reset reason (brownout), load test with motors switching direction |
| `Test_Buttons` | START / SET / MODE buttons, press count, contact bounce, wire shake test |
| `Test_OLED` | OLED wiring and controller type (SSD1306 or SH1106), I2C scan, test patterns; no library needed |
| `Test_MPU_OLED_Buttons` | MPU6050 acceleration, gyro and yaw shown on the OLED, buttons |
| `Test_TB6612` | TB6612FNG outputs, one input state every 4 s to measure with a multimeter |
| `Test_RightMotor` | Right N20 motor and its encoder, rpm at set PWM, manual pulse count |
| `Test_N20` | Both N20 encoder motors: minimum PWM, rpm at 30 / 60 / 100 %, pulses per turn |
| `Test_FullSpeed10s` | Both motors at 100 % for 10 s, rpm of each wheel and mismatch |
| `Test_WheelSpeed` | Robot drives itself on the floor (buttons + OLED, no USB): 1 m odometer check, loaded speed at 40 / 60 / 80 / 100 % PWM |
| `Test_LineSensor` | 8-channel line sensor: raw and calibrated values, noise, analog/digital type, read speed, sensor latency |
| `Test_DriveStraight_Simple` | Straight driving with encoders only |
| `Test_DriveStraight` | Straight driving for a set distance with gyro heading hold and encoder distance |

### tests/integration/

| Folder | Checks |
|---|---|
| `Test_Integration` | Motors, encoders, MPU6050, OLED and buttons together: direction check, motor test, 500 mm straight, 90-degree turn, obstacle avoid |
| `Test_FullRobot` | Every module and the whole robot from an OLED menu: power, buttons, MPU, line sensor, VL53L0X, motors, encoders, straight and turn, line following, obstacle avoidance |

## Pin maps

The contest firmware uses pin map v5 ([docs/hardware/pinout.md](../docs/hardware/pinout.md)). Sketches written before the perfboard was built use the older breadboard pins; the pins of every sketch are listed in its header comment.
