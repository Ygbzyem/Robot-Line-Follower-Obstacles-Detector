# Robot Dò Line

[English](README.md) | **Tiếng Việt**

[![build](https://github.com/<user>/line-follower-robot/actions/workflows/build.yml/badge.svg)](https://github.com/<user>/line-follower-robot/actions/workflows/build.yml)
![MCU](https://img.shields.io/badge/MCU-ESP32--S3-blue)
![Framework](https://img.shields.io/badge/Arduino--ESP32-3.0.7-teal)
![License](https://img.shields.io/badge/license-MIT-green)

Robot dò line tốc độ cao có né vật cản trên ESP32-S3, làm cho cuộc thi robot dò line mê cung của F-Chip.

![Robot](media/photos/robot.jpg)

## Tính năng

- Cảm biến line analog 8 mắt, hiệu chuẩn riêng từng mắt, bù trễ cảm biến và tự bỏ qua mắt bị kẹt
- Xử lý cụm ngã rẽ (lưới ô vuông và lục giác), góc 90°, zigzag, line đứt và cung tròn
- Điều khiển tốc độ từng bánh bằng encoder, giữ hướng bằng gyro MPU6050
- Né viên gạch phát hiện bằng VL53L0X phía trước rồi bắt lại line
- Lượt DEMO ghi bản đồ sa bàn, lượt FAST chạy lại theo bản đồ
- Chỉnh thông số qua menu OLED, lệnh Serial hoặc trang Wi-Fi; mọi giá trị lưu trong NVS

## Cấu trúc repo

```
line-follower-robot/
├── README.md                              # tiếng Anh
├── vn.md                                  # tiếng Việt
├── LICENSE
├── CONTRIBUTING.md
├── CHANGELOG.md
├── .gitignore
├── .editorconfig
├── platformio.ini                         # mặc định build src/competition
│
├── .github/
│   ├── ISSUE_TEMPLATE/
│   │   ├── bug_report.md
│   │   ├── feature_request.md
│   │   └── hardware_issue.md
│   ├── workflows/
│   │   └── build.yml                      # tự biên dịch mọi sketch mỗi lần push
│   └── PULL_REQUEST_TEMPLATE.md
│
├── src/
│   ├── README.md                          # giải thích từng thư mục
│   ├── competition/                       # code ĐANG CHẠY trên xe
│   │   └── Robot_Competition/             # firmware thi đấu C3.9
│   │
│   ├── archive/                           # các bản thi đấu cũ, không nạp nữa
│   │   ├── Robot_Competition_v31/         # v3.1   chế độ DEMO / FAST theo bản đồ
│   │   ├── Robot_Competition_A/           # v3.5.1 PID tốc độ bánh, né lướt
│   │   ├── Robot_Competition_B/           # v3.9.1 bản B chạy song song
│   │   └── Robot_Competition_C/           # v3.8   bản C, chống chạy ngược
│   │
│   ├── prototypes/                        # firmware phát triển trước code thi đấu
│   │   ├── Robot_Skeleton_v02/
│   │   ├── Robot_MKE_v03/
│   │   ├── Robot_LineFollow_v5/
│   │   └── Robot_RouteRun/
│   │
│   └── tests/                             # sketch kiểm tra phần cứng, không dùng khi chạy
│       ├── hardware/                      # mỗi sketch kiểm tra một linh kiện hoặc một hành vi
│       │   ├── Test_Power/             Test_Buttons/       Test_OLED/
│       │   ├── Test_MPU_OLED_Buttons/  Test_TB6612/        Test_RightMotor/
│       │   ├── Test_N20/               Test_FullSpeed10s/  Test_LineSensor/
│       │   └── Test_DriveStraight_Simple/  Test_DriveStraight/
│       └── integration/                   # nhiều module cùng lúc hoặc cả xe
│           ├── Test_Integration/
│           └── Test_FullRobot/
│
├── docs/
│   ├── architecture/
│   │   └── system_architecture.md         # sơ đồ khối, cấu trúc firmware, luồng dữ liệu
│   ├── hardware/
│   │   ├── components.md                  # linh kiện, giá, nơi mua
│   │   └── pinout.md
│   └── testing/
│       └── test_plan.md                   # thứ tự kiểm tra và tiêu chí đạt
│
├── hardware/
│   └── schematics/
│
└── media/
    ├── photos/
    └── videos/
```

Mỗi sketch nằm trong thư mục cùng tên (`Robot_Competition/Robot_Competition.ino`) nên mở thẳng được bằng Arduino IDE. Xem [src/README.md](src/README.md) để biết từng thư mục và từng sketch dùng làm gì (tài liệu tiếng Anh).

## Phần cứng

ESP32-S3 (MakerEDU MKE-K01 + shield MKE-B01), 2 motor N20 có encoder qua driver TB6612FNG, cảm biến line analog 8 mắt, VL53L0X, MPU6050, OLED SSD1306, 3 nút nhấn, còi, pin 2S 18650.

- Linh kiện, giá, nơi mua: [docs/hardware/components.md](docs/hardware/components.md)
- Sơ đồ chân: [docs/hardware/pinout.md](docs/hardware/pinout.md)
- Sơ đồ đấu dây: [hardware/schematics/](hardware/schematics/)
- Tổng quan hệ thống: [docs/architecture/system_architecture.md](docs/architecture/system_architecture.md)

Các tài liệu trong `docs/` viết bằng tiếng Anh.

## Bắt đầu

### Biên dịch và nạp

Cài VS Code với extension **PlatformIO IDE** (hoặc PlatformIO CLI), cắm xe qua USB rồi chạy:

```sh
pio run -t upload          # biên dịch và nạp src/competition/Robot_Competition
pio device monitor         # Serial 115200
```

Nếu dùng Arduino IDE (core esp32 3.0.7, board **ESP32S3 Dev Module**), mở `src/competition/Robot_Competition/Robot_Competition.ino` và cài bốn thư viện bằng Library Manager.

Các thư viện (Adafruit_VL53L0X, Adafruit GFX, Adafruit SSD1306, Pololu VL53L0X) được PlatformIO tự cài theo `platformio.ini`.

### Lần chạy đầu

1. Bật nguồn khi bánh đang nhấc lên. Serial in ra điện áp pin và `MPU OK`, `VL53 OK`.
2. Lệnh `cal` hiệu chuẩn cảm biến line, lệnh `raw` kiểm tra cả 8 mắt đều thay đổi giữa đen và trắng.
3. Lệnh `wcal` đo tốc độ bánh ở 100 % cho bộ điều khiển tốc độ.
4. Chạy thử với `sp 30` và `vmax 30`, sau đó tăng dần tốc độ.

## Kiểm tra phần cứng

Các sketch kiểm tra nằm trong `src/tests/`. Làm theo thứ tự và tiêu chí đạt trong [docs/testing/test_plan.md](docs/testing/test_plan.md). Nạp một sketch test bằng:

```sh
PLATFORMIO_SRC_DIR=src/tests/hardware/Test_LineSensor pio run -t upload
```

Trên PowerShell: `$env:PLATFORMIO_SRC_DIR="src/tests/hardware/Test_LineSensor"; pio run -t upload`, sau đó `Remove-Item Env:PLATFORMIO_SRC_DIR`.

## Chỉnh thông số

- **OLED + nút**: MODE = sang trang, SET = thao tác của trang, START = chạy, nút bất kỳ = dừng.
- **Serial 115200 (Newline)**: `kp 14`, `kd 0.35`, `sp 40`, `vmax 60`, `obs 130`; nhiều lệnh một lúc: `kp 12;kd 0.4;sp 70`. Lệnh `export` in toàn bộ thông số thành một dòng, dán lại dòng đó là nạp lại được.
- **Trang Wi-Fi**: khi đứng yên xe phát Wi-Fi `LINE-ROBOT`; vào `192.168.4.1` để xem kết quả, nhật ký sự kiện, sửa thông số và tải `log.txt`.

Firmware vẫn nhận tên lệnh của các bản trước, nên dòng thông số xuất từ bản cũ vẫn nạp được.

## Lỗi đã biết

`Robot_Skeleton_v02` và `Robot_MKE_v03` không biên dịch được vì `typedef StopFn` và `struct Btn` được khai báo sau hàm đầu tiên. Hai bản này được giữ nguyên trong `src/prototypes/` làm lịch sử và được bỏ qua khi CI biên dịch.

## Lịch sử phiên bản

Xe đang chạy `src/competition/Robot_Competition` (C3.9). Các bản thi đấu cũ nằm trong `src/archive/`; xem chi tiết ở [CHANGELOG.md](CHANGELOG.md).

| Phiên bản | Thư mục | Tóm tắt |
|---|---|---|
| C3.9 | `src/competition/Robot_Competition` | Firmware thi đấu mới nhất |
| v3.9.1 | `src/archive/Robot_Competition_B` | Bản B chạy song song |
| v3.8 | `src/archive/Robot_Competition_C` | Bản C có chống chạy ngược |
| v3.5.1 | `src/archive/Robot_Competition_A` | PID tốc độ bánh, né lướt |
| v3.1 | `src/archive/Robot_Competition_v31` | Chế độ DEMO / FAST theo bản đồ |

## Đóng góp

Xem [CONTRIBUTING.md](CONTRIBUTING.md).

## Tác giả

**Phạm Minh Tùng** — Thiết kế vi mạch bán dẫn, Đại học FPT. [Thành viên nhóm]

## Giấy phép

[MIT](LICENSE) © 2026 Phạm Minh Tùng
