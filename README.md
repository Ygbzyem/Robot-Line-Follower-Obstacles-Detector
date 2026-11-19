# Line Follower Robot

**English** | [Tiếng Việt](vn.md)

[![build](https://github.com/Ygbzyem/Robot-Line-Follower-Obstacles-Detector/actions/workflows/build.yml/badge.svg)](https://github.com/Ygbzyem/Robot-Line-Follower-Obstacles-Detector/actions/workflows/build.yml)
![MCU](https://img.shields.io/badge/MCU-ESP32--S3-blue)
![Framework](https://img.shields.io/badge/Arduino--ESP32-3.0.7-teal)
![License](https://img.shields.io/badge/license-MIT-green)

High-speed line-following robot with obstacle avoidance on ESP32-S3, built for the F-Chip line-maze contest.

![Robot](media/photos/robot.jpg)

## Features

- 8-channel analog line sensing with per-channel calibration, sensor-lag compensation and stuck-channel rejection
- Junction and cluster handling (square and hexagon grids), 90° corners, zigzags, dashed lines and arcs
- Per-wheel speed control from quadrature encoders, gyro heading from an MPU6050
- Obstacle avoidance around a brick detected by a front VL53L0X, then line re-acquisition
- DEMO run that records the track map and FAST run that replays it
- Tuning through an OLED menu, Serial commands or a Wi-Fi page; all values stored in NVS

## Repository Structure

```
line-follower-robot/
├── README.md                              # English
├── vn.md                                  # Vietnamese
├── LICENSE
├── CONTRIBUTING.md
├── CHANGELOG.md
├── .gitignore
├── .editorconfig
├── platformio.ini                         # builds src/competition by default
│
├── .github/
│   ├── ISSUE_TEMPLATE/
│   │   ├── bug_report.md
│   │   ├── feature_request.md
│   │   └── hardware_issue.md
│   ├── workflows/
│   │   └── build.yml                      # compiles every sketch on each push
│   └── PULL_REQUEST_TEMPLATE.md
│
├── src/
│   ├── README.md                          # what each folder contains
│   ├── competition/                       # RUNNING on the robot
│   │   └── Robot_Competition/             # contest firmware C3.11b
│   │
│   ├── archive/                           # earlier contest versions, not flashed any more
│   │   ├── Robot_Competition_v31/         # v3.1   DEMO / FAST map modes
│   │   ├── Robot_Competition_C39/         # C3.9   circle-cluster exit, anti-stall pivot
│   │   ├── Robot_Competition_A/           # v3.5.1 wheel speed PID, flowing avoidance
│   │   ├── Robot_Competition_B/           # v3.9.1 parallel version B
│   │   └── Robot_Competition_C/           # v3.8   version C, wrong-way guard
│   │
│   ├── prototypes/                        # development firmware before the contest code
│   │   ├── Robot_Skeleton_v02/
│   │   ├── Robot_MKE_v03/
│   │   ├── Robot_LineFollow_v5/
│   │   └── Robot_RouteRun/
│   │
│   └── tests/                             # bring-up sketches, not used in a run
│       ├── hardware/                      # one peripheral or behavior per sketch
│       │   ├── Test_Power/             Test_Buttons/       Test_OLED/
│       │   ├── Test_MPU_OLED_Buttons/  Test_TB6612/        Test_RightMotor/
│       │   ├── Test_N20/               Test_FullSpeed10s/  Test_LineSensor/
│       │   └── Test_DriveStraight_Simple/  Test_DriveStraight/
│       └── integration/                   # several modules or the whole robot
│           ├── Test_Integration/
│           └── Test_FullRobot/
│
├── docs/
│   ├── architecture/
│   │   └── system_architecture.md         # block diagram, firmware structure, data flow
│   ├── hardware/
│   │   ├── components.md                  # parts, prices, shops
│   │   └── pinout.md
│   └── testing/
│       └── test_plan.md                   # bring-up order and pass criteria
│
├── hardware/
│   └── schematics/
│
└── media/
    ├── photos/
    └── videos/
```

Each sketch sits in a folder with the same name (`Robot_Competition/Robot_Competition.ino`), so it opens directly in the Arduino IDE. See [src/README.md](src/README.md) for what every folder and sketch does.

## Hardware

ESP32-S3 (MakerEDU MKE-K01 + MKE-B01 shield), 2 × N20 encoder motors on a TB6612FNG, 8-channel analog line sensor, VL53L0X, MPU6050, SSD1306 OLED, 3 buttons, buzzer, 2S 18650 battery.

- Parts, prices and shops: [docs/hardware/components.md](docs/hardware/components.md)
- Pin mapping: [docs/hardware/pinout.md](docs/hardware/pinout.md)
- Wiring diagram: [hardware/schematics/](hardware/schematics/)
- System overview: [docs/architecture/system_architecture.md](docs/architecture/system_architecture.md)

## Getting Started

### Build and flash

Install VS Code with the **PlatformIO IDE** extension (or the PlatformIO CLI), connect the robot over USB, then:

```sh
pio run -t upload          # build and flash src/competition/Robot_Competition
pio device monitor         # Serial 115200
```

With the Arduino IDE (esp32 core 3.0.7, board **ESP32S3 Dev Module**), open `src/competition/Robot_Competition/Robot_Competition.ino` and install the four libraries from the Library Manager.

Libraries (Adafruit_VL53L0X, Adafruit GFX, Adafruit SSD1306, Pololu VL53L0X) are installed automatically from `platformio.ini`.

### First run

1. Power on with the wheels lifted. The Serial banner prints the battery voltage and `MPU OK`, `VL53 OK`.
2. `cal` calibrates the line sensor, `raw` checks that all 8 channels change between black and white.
3. `wcal` measures the wheel speed at 100 % for the speed control.
4. Start with `sp 30` and `vmax 30`, then raise the speed step by step.

## Testing

Bring-up sketches are in `src/tests/`. Follow the order and pass criteria in [docs/testing/test_plan.md](docs/testing/test_plan.md). Flash a test with:

```sh
PLATFORMIO_SRC_DIR=src/tests/hardware/Test_LineSensor pio run -t upload
```

On PowerShell: `$env:PLATFORMIO_SRC_DIR="src/tests/hardware/Test_LineSensor"; pio run -t upload`, then `Remove-Item Env:PLATFORMIO_SRC_DIR`.

## Tuning

- **OLED + buttons**: MODE = next page, SET = page action, START = run, any button = stop.
- **Serial 115200 (Newline)**: `kp 14`, `kd 0.35`, `sp 40`, `vmax 60`, `obs 130`; several at once: `kp 12;kd 0.4;sp 70`. `export` prints every parameter in one line that can be pasted back.
- **Wi-Fi page**: when idle the robot opens the access point `LINE-ROBOT`; browse to `192.168.4.1` for results, event log, parameter editing and `log.txt`.

Command names of earlier builds are still accepted, so exported parameter lines from those builds load unchanged.

## Known Issues

`Robot_Skeleton_v02` and `Robot_MKE_v03` do not compile because `typedef StopFn` and `struct Btn` are declared after the first function. They are kept unchanged in `src/prototypes/` as history and are skipped by the CI build.

## Version History

The robot runs `src/competition/Robot_Competition` (C3.11b). Earlier contest versions are kept in `src/archive/`; see [CHANGELOG.md](CHANGELOG.md).

| Version | Folder | Summary |
|---|---|---|
| C3.11b | `src/competition/Robot_Competition` | Latest contest firmware: hexagon-cluster heading lock, straight start out of the black box |
| C3.9 | `src/archive/Robot_Competition_C39` | Circle-cluster exit, avoidance toward the bend, anti-stall pivot |
| v3.9.1 | `src/archive/Robot_Competition_B` | Parallel version B |
| v3.8 | `src/archive/Robot_Competition_C` | Version C with wrong-way guard |
| v3.5.1 | `src/archive/Robot_Competition_A` | Wheel speed PID, flowing avoidance |
| v3.1 | `src/archive/Robot_Competition_v31` | DEMO / FAST map modes |

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## Author

**Phạm Minh Tùng** — Semiconductor IC Design, FPT University. [Teammates]

## License

[MIT](LICENSE) © 2026 Phạm Minh Tùng
