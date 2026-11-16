# Mã nguồn

[English](README.md) | **Tiếng Việt**

Toàn bộ firmware của robot. Mỗi sketch nằm trong thư mục cùng tên, nên mở được bằng Arduino IDE hoặc biên dịch bằng PlatformIO.

```
src/
├── competition/    code đang chạy trên xe
├── archive/        các bản thi đấu cũ
├── prototypes/     firmware phát triển trước code thi đấu
└── tests/          sketch kiểm tra phần cứng, không dùng khi chạy
    ├── hardware/
    └── integration/
```

## Nạp sketch nào

| Mục đích | Sketch |
|---|---|
| Chạy thi trên sa bàn | `competition/Robot_Competition` |
| Kiểm tra một linh kiện sau khi đấu dây hoặc sửa | sketch tương ứng trong `tests/hardware/` |
| Kiểm tra cả xe trước khi chạy | `tests/integration/Test_FullRobot` |
| So sánh với cách chạy của bản thi đấu cũ | một thư mục trong `archive/` |

PlatformIO mặc định biên dịch `competition/Robot_Competition`. Muốn biên dịch sketch khác thì đổi thư mục nguồn, chạy ở thư mục gốc repo:

```powershell
$env:PLATFORMIO_SRC_DIR="src/tests/hardware/Test_LineSensor"; pio run -t upload
Remove-Item Env:PLATFORMIO_SRC_DIR
```

## competition/

| Thư mục | Phiên bản | Mô tả |
|---|---|---|
| `Robot_Competition` | C3.9 | Firmware thi đấu đang nạp trên xe: dò line, cụm ngã rẽ, né vật cản, chế độ DEMO/FAST theo bản đồ, menu OLED, chỉnh thông số qua Serial và Wi-Fi. |

Chỉ thư mục này được dùng khi thi. Khi có bản thi đấu mới, file mới thay vào đây, còn file cũ chuyển sang `archive/`.

## archive/

Các bản thi đấu trước, giữ lại để so sánh hoặc quay về khi cần. Không nạp nữa.

| Thư mục | Phiên bản | Thay đổi chính |
|---|---|---|
| `Robot_Competition_v31` | v3.1 | Chế độ chạy THƯỜNG, DEMO (ghi bản đồ sa bàn) và NHANH (chạy theo bản đồ) |
| `Robot_Competition_A` | v3.5.1 | PID tốc độ từng bánh, xuất thông số, né vật cản kiểu lướt |
| `Robot_Competition_B` | v3.9.1 | Bản B chạy song song, góc 90 độ đơn xoay nhanh |
| `Robot_Competition_C` | v3.8 | Bản C có chống chạy ngược, là nền của C3.x |

## prototypes/

Firmware viết khi robot và luật sa bàn còn đang thay đổi. Mỗi bản đóng góp một phần được dùng lại trong code thi đấu.

| Thư mục | Phiên bản | Mô tả | Trạng thái |
|---|---|---|---|
| `Robot_Skeleton_v02` | v0.2 | Khung đầu tiên cho ESP32-S3 DevKitC-1: 3 × VL53L0X, các chế độ FOLLOW / EXPLORE / FAST / SAFE / CALIB, bản đồ mê cung từ odometry | Không biên dịch được (xem README chính) |
| `Robot_MKE_v03` | v0.3 | Chuyển sang board MKE-K01 cho sa bàn uốn lượn, không dùng encoder | Không biên dịch được (xem README chính) |
| `Robot_LineFollow_v5` | v1.0 | Bộ dò line PD đầu tiên trên sơ đồ chân v5, có hiệu chuẩn cảm biến và chỉnh qua Serial | Biên dịch được |
| `Robot_RouteRun` | v1.2 | Chạy lộ trình lập sẵn không cần line (đi thẳng, đi - về, hình vuông, né vật) bằng encoder và gyro | Biên dịch được |

## tests/

Các sketch kiểm tra phần cứng. Mỗi sketch kiểm tra một việc và in kết quả ra Serial (115200), có OLED thì hiện cả trên màn hình. Chạy theo thứ tự trong [docs/testing/test_plan.md](../docs/testing/test_plan.md) (tiếng Anh).

### tests/hardware/

| Thư mục | Kiểm tra |
|---|---|
| `Test_Power` | Điện áp pin, lý do khởi động lại lần trước (sụt nguồn), thử tải motor đảo chiều liên tục |
| `Test_Buttons` | Nút START / SET / MODE, số lần nhấn, rung tiếp điểm, thử lắc dây |
| `Test_OLED` | Đấu dây OLED và loại chip (SSD1306 hoặc SH1106), quét I2C, hình kiểm tra; không cần thư viện |
| `Test_MPU_OLED_Buttons` | Gia tốc, gyro và góc yaw của MPU6050 hiện trên OLED, cùng các nút |
| `Test_TB6612` | Ngõ ra TB6612FNG, mỗi 4 s một trạng thái để đo bằng đồng hồ |
| `Test_RightMotor` | Motor N20 bên phải và encoder, rpm theo PWM, đếm xung bằng tay |
| `Test_N20` | Cả hai motor N20: PWM tối thiểu, rpm ở 30 / 60 / 100 %, số xung mỗi vòng |
| `Test_FullSpeed10s` | Hai motor chạy 100 % trong 10 s, rpm từng bánh và độ lệch |
| `Test_LineSensor` | Cảm biến line 8 mắt: giá trị thô và đã hiệu chuẩn, nhiễu, loại analog/digital, tốc độ đọc, độ trễ cảm biến |
| `Test_DriveStraight_Simple` | Đi thẳng chỉ bằng encoder |
| `Test_DriveStraight` | Đi thẳng một quãng định trước, giữ hướng bằng gyro, đo quãng bằng encoder |

### tests/integration/

| Thư mục | Kiểm tra |
|---|---|
| `Test_Integration` | Motor, encoder, MPU6050, OLED và nút cùng lúc: kiểm tra chiều, thử motor, đi thẳng 500 mm, quay 90 độ, né vật cản |
| `Test_FullRobot` | Từng module và cả xe qua menu OLED: nguồn, nút, MPU, cảm biến line, VL53L0X, motor, encoder, đi thẳng và quay, dò line, né vật cản |

## Sơ đồ chân

Firmware thi đấu dùng sơ đồ chân v5 ([docs/hardware/pinout.md](../docs/hardware/pinout.md)). Các sketch viết trước khi hàn mạch phíp dùng sơ đồ chân breadboard cũ; chân của từng sketch được ghi trong phần chú thích đầu file.
