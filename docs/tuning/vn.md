# Lệnh chỉnh thông số

[English](README.md) | **Tiếng Việt**

Mọi thông số của firmware thi đấu (`src/competition/Robot_Competition`) đều chỉnh được mà không cần nạp lại code. Trang này liệt kê từng lệnh, giá trị trong firmware hiện tại (C3.14, motor mới) và khi nào nên chỉnh.

## Cách gửi lệnh

- **Serial Monitor**, 115200 baud, kết thúc dòng *Newline*:
  - một giá trị: `kp 12`
  - nhiều giá trị: `kp 12;kd 0.4;sp 70`
  - `export` in toàn bộ thông số thành một dòng; dán lại dòng đó (hoặc `load <dòng>`) để nạp lại.
- **Trang Wi-Fi**: kết nối `LINE-ROBOT`, mở `192.168.4.1`, sửa các ô trong mục *Parameters* rồi bấm *Save parameters*.
- **OLED + nút**: `sp`, `vmax`, bên né và Wi-Fi đổi được ở các trang menu bằng nút SET.

Giá trị được lưu trên xe (NVS), tắt nguồn hay nạp lại code vẫn còn. Dòng `USER_CFG` đầu sketch chứa giá trị mặc định; lệnh `reset` nạp lại các giá trị đó.

Mỗi lần chỉ đổi một giá trị, chạy thử rồi so sánh. Tốc độ tính bằng % tốc độ tối đa, khoảng cách bằng mm, góc bằng độ.

## Thứ tự chỉnh

1. `cal`: hiệu chuẩn cảm biến line.
2. `wcal`: đo tốc độ bánh ở công suất tối đa (`vfull`).
3. `lag`: đặt độ trễ cảm biến đo bằng `Test_LineSensor`.
4. `kp`, `kd` ở tốc độ thấp (`sp 30`, `vmax 30`).
5. Tăng dần `sp` và `vmax`; chỉnh `cspd`, `brake`, `kpb`, `piv`, `alat` cho cua và góc.
6. Ngã rẽ và cụm: `jmode`, `vclust`, `pclust`.
7. Né vật cản: `obs`, `aout`, `out`, `par`, `aback`.
8. Ô đích: `fin`, `fgo`.

## Tốc độ

| Lệnh | Giá trị | Chức năng | Khi nào chỉnh |
|---|---:|---|---|
| `sp` | 80 | Tốc độ cơ bản khi vào cua và qua ngã rẽ | Xe lắc hoặc văng khỏi line khi vào cua → giảm |
| `vmax` | 100 | Tốc độ khi line đã thẳng và ổn định 0,25 s; phải ≥ `sp` | Xe trượt hoặc lắc trên đoạn thẳng → giảm |
| `cspd` | 38 | Tốc độ xe giảm xuống ngay khi line chạm mép thanh cảm biến (sắp tới góc 90° hoặc cua gắt) | Lố góc → giảm; vào góc quá chậm → tăng |
| `alat` | 3800 | Giới hạn tốc độ khi vào cua: cua càng gắt càng chậm. `0` = tắt; cần có `vfull` | Văng ra ở cua → giảm; cua nhẹ mà chạy quá chậm → tăng |

## Dò line

| Lệnh | Giá trị | Chức năng | Khi nào chỉnh |
|---|---:|---|---|
| `kp` | 10.2 | Độ mạnh bẻ lái: line càng lệch tâm, bẻ lái càng mạnh | Văng ra ở cua → tăng; lắc liên tục → giảm |
| `kd` | 0.39 | Dập dao động khi bẻ lái | Chạy zigzag → tăng mỗi lần 0.05; giật, rung → giảm |
| `brake` | 0.78 | Giảm tốc khi line lệch xa tâm (0.45–0.95) | Văng ra khỏi cua → tăng; vào cua quá chậm → giảm |
| `kpb` | 1.45 | Nhân thêm `kp` khi line lệch từ 1,8 mắt trở lên (1.0 = tắt) | Mất line ở cua gắt → tăng; lố ra khi ra cua → giảm |
| `piv` | 1.75 | Khi line lệch tới mức này (tính bằng mắt) và không phải ngã rẽ, xe xoay tại chỗ với bánh trong quay ngược. `0` = tắt | Mất line ở mép → giảm; giật ở cua nhẹ → tăng |
| `curve` | 1 | Mất line ở cua tròn hoặc chữ S: `1` = phanh và quay lại ngay, `0` = xử lý như góc 90° (cách cũ) | Giữ `1`, trừ khi xe quay sai hướng sau khi mất line |
| `lag` | 8 | Bù độ trễ của thanh cảm biến quang trở (ms). `0` = tắt | Dùng giá trị đo bằng `Test_LineSensor`; bẻ lái bị nhiễu → giảm |
| `tlead` | 20 | Dừng quay sớm một chút để không quay lố (ms). Log hiện `WRONG TURN ANGLE` khi lệch quá 5° | Quay lố → tăng (40–60); quay thiếu → giảm |

## Điều khiển tốc độ bánh

| Lệnh | Giá trị | Chức năng | Khi nào chỉnh |
|---|---:|---|---|
| `wpid` | 1 | `1` = mỗi bánh tự giữ tốc độ đặt nhờ encoder; `0` = PWM trực tiếp | Giữ `1`; chỉ dùng `0` khi thử không có encoder |
| `vfull` | 1640 | Tốc độ bánh ở 100 % công suất (mm/s). Lệnh `wcal` tự đặt; `0` sẽ tắt điều khiển tốc độ bánh | Chạy lại `wcal` sau khi đổi motor, bánh hoặc hộp số |
| `wkp` | 1.5 | Bộ điều khiển tốc độ bánh: phản ứng với sai lệch tốc độ | Bánh vọt lên vọt xuống hoặc dao động → giảm |
| `wki` | 6 | Bộ điều khiển tốc độ bánh: khử sai lệch tốc độ kéo dài | Bánh chậm lại khi có tải → tăng; dao động chậm → giảm |

## Ngã rẽ và cụm

| Lệnh | Giá trị | Chức năng | Khi nào chỉnh |
|---|---:|---|---|
| `jmode` | 1 | Trong cụm ô vuông hoặc lục giác: `1` = chọn nhánh đưa xe về hướng thẳng ban đầu (đi xuyên qua cụm), `0` = luôn theo bên `pref` | `0` có thể làm xe chạy vòng một ô lục giác; giữ `1` |
| `pref` | 0 | Bên chọn ở ngã ba chữ T: `0` = trái, `1` = phải | Đặt theo bên sa bàn yêu cầu |
| `jnmode` | 1 | Trong vòng 60 cm quanh ngã rẽ hoặc cụm, dùng kiểu dò cẩn thận để rẽ đúng nhánh ở các vòng tròn tiếp xúc và ô vuông | Giữ `1` |
| `cluster` | 1 | Cụm ô vuông và ngã rẽ liên tiếp: `1` = giữ hướng và rẽ ở đỉnh, `0` = xử lý thường | Giữ `1` |
| `vclust` | 25 | Tốc độ trong cụm ô vuông; càng chậm càng ít lố | Lố trong lưới ô → giảm |
| `pclust` | 2 | Bên chọn ở ngã ba vuông góc trong cụm ô vuông: `0` = trái, `1` = phải, `2` = theo `pref` | Đặt khi lưới ô cần một bên cố định |
| `uturn` | 1 | Nhớ đoạn đường đã đi để xe không quay ngược lại đường cũ. `0` = tắt | Giữ `1` |
| `mask` | 0 | Tắt mắt cảm biến bị hỏng: D1 = 1, D2 = 2, D3 = 4, D4 = 8, D5 = 16, D6 = 32, D7 = 64, D8 = 128; cộng các số để tắt nhiều mắt | Một mắt đọc sai → đặt số của mắt đó (D8 hỏng → `mask 128`) |
| `auto` | 1 | `1` = bỏ qua mắt báo đen sai hơn 25 % thời gian trong lượt chạy; `0` = chỉ dùng `mask` | Giữ `1` |

## Né vật cản

Xe né theo bốn đoạn: ① quay ra góc `aout`, ② đi chéo `out`, ③ đi song song với line `par`, ④ quay về line với góc `aback`.

| Lệnh | Giá trị | Chức năng | Khi nào chỉnh |
|---|---:|---|---|
| `obs` | 280 | Khoảng cách (mm) mà VL53L0X bắt đầu kích hoạt né | Mũi xe đụng gạch khi quay ra → tăng; né quá sớm → giảm |
| `side` | 0 | Bên né: `0` = trái, `1` = phải | Chọn bên còn chỗ trống |
| `avmode` | 2 | `0` = xe tự đo viên gạch và tính đường né, `1` = khoảng cách cố định, dừng sau mỗi đoạn, `2` = khoảng cách cố định, không dừng giữa các đoạn | Thi đấu dùng `2` |
| `aout` | 58 | ① Góc quay ra (30–90°) | Đi sát gạch quá → tăng; ra quá rộng → giảm |
| `out` | 76 | ② Quãng đi chéo sau khi quay ra (mm) | Ra quá xa → giảm; quẹt cạnh gạch → tăng |
| `par` | 500 | ③ Quãng đi song song để vượt qua gạch (mm) | Đi quá xa → giảm; đuôi xe quẹt gạch khi quay về → tăng |
| `aback` | 65 | ④ Góc quay về line (20–90°) | Cắt qua line quá xa → giảm; về line quá chậm → tăng |
| `retm` | 1 | Cách bắt lại line: `1` = phanh, canh giữa line rồi quay (ổn định); `0` = đi theo cung tròn vào line (nhanh hơn, cần `sens` chính xác) | Giữ `1` |
| `avscan` | 0 | `1` = tìm line cả trong đoạn ② và ③; `0` = chỉ khi quay về | Giữ `0` trên sa bàn hẹp để xe không bắt nhầm line khác |
| `vout` | 49 | Tốc độ đoạn ② | Trượt quá quãng đặt → giảm |
| `vpar` | 49 | Tốc độ đoạn ③ | Không bắt được line ở cua → giảm |
| `vret` | 42 | Tốc độ khi quay về (gần line tự giảm thêm) | Cắt qua line rộng → giảm |
| `vturn` | 49 | Tốc độ quay ở ① và ④ | Quay lố góc → giảm |
| `brev` | 35 | Lực đảo chiều motor để phanh ở cuối mỗi đoạn (%). `0` = chỉ phanh ngắn mạch | Vẫn trượt → tăng (40–60); giật lùi → giảm |
| `bms` | 50 | Thời gian phanh tối đa (ms) | Thường 40–80 |
| `dec` | 50 | Quãng giảm tốc trước mỗi điểm dừng (mm) | Vẫn trượt quá → tăng (60–80); né quá chậm → giảm |
| `lat` | 165 | Chỉ dùng với `avmode 0`: độ lệch ngang tối thiểu (mm) | Chỉ khi dùng `avmode 0` |
| `pass` | 80 | Chỉ dùng với `avmode 0`: quãng đi qua khỏi đuôi gạch trước khi quay về (mm) | Chỉ khi dùng `avmode 0` |
| `nobs` | 0 | Số vật cản tối đa, sau đó VL53L0X tắt để không nhầm viền sa bàn là gạch. `0` = không giới hạn | Đặt `2` nếu sau hai viên gạch xe né nhầm viền sa bàn |

## Chạy theo bản đồ (DEMO / FAST)

| Lệnh | Giá trị | Chức năng | Khi nào chỉnh |
|---|---:|---|---|
| `mode` | 0 | `0` = thường, `1` = DEMO (chạy một lượt và ghi bản đồ sa bàn), `2` = FAST (chạy theo bản đồ). Tắt nguồn sẽ về `0`, bản đồ vẫn giữ | Cũng chọn được ở trang READY |
| `vfast` | 100 | Chế độ FAST: tốc độ khi bản đồ báo 25 cm phía trước là đường thẳng | Lắc trên đoạn thẳng → giảm |
| `mpre` | 250 | Chế độ FAST: bắt đầu giảm tốc trước góc, cua hoặc vật cản bao xa (mm) | Lố góc → tăng (300–400); giảm tốc quá sớm → giảm |

## Ô đích

| Lệnh | Giá trị | Chức năng | Khi nào chỉnh |
|---|---:|---|---|
| `fin` | 500 | Quãng đường (mm) xe phải đi trước khi một ô đen được tính là đích, để bỏ qua ô xuất phát | Xe dừng giữa sa bàn → tăng |
| `fgo` | 180 | Sau khi thanh cảm biến chạm ô đích, đi thẳng thêm quãng này (mm) rồi dừng, để xe nằm giữa ô | Dừng chưa tới giữa → tăng; lố → giảm |

## Kích thước xe

Đo bằng thước khi thay đổi phần cơ khí.

| Lệnh | Giá trị | Chức năng | Khi nào chỉnh |
|---|---:|---|---|
| `sens` | 115 | Khoảng cách từ hàng mắt cảm biến tới trục bánh (mm), dùng ở góc và khi bắt lại line | Sau khi dời thanh cảm biến |
| `track` | 128 | Khoảng cách giữa tâm hai bánh (mm), dùng khi quay | Sau khi đổi bánh hoặc pát motor |
| `wheel` | 48 | Đường kính bánh tính cả lốp (mm) | Sau khi đổi bánh |
| `cpr` | 576.3 | Số xung encoder mỗi vòng bánh. Kiểm tra: đẩy xe 60 cm, trang Sensor phải hiện D ≈ 600 | Sau khi đổi motor hoặc hộp số; đo bằng `cprcnt` |

## Hệ thống

| Lệnh | Giá trị | Chức năng | Khi nào chỉnh |
|---|---:|---|---|
| `roled` | 0 | Cập nhật OLED khi đang chạy: `0` = tắt, giữ vòng điều khiển đều | Giữ `0` khi thi |
| `wifi` | 1 | Phát Wi-Fi khi xe đứng yên | Đặt `0` trước ngày thi: mật khẩu đang công khai |
| `vk` | 1 | Hệ số hiệu chỉnh số đo điện áp pin | Lệnh `vcal` tự đặt; hiếm khi gõ tay |
| `vcal <vôn>` | – | Gõ điện áp pin đo bằng đồng hồ (ví dụ `vcal 8.12`); xe tự tính `vk` | Khi điện áp trên OLED khác số đo của đồng hồ |

## Lệnh thao tác

| Lệnh | Chức năng |
|---|---|
| `cal` | Hiệu chuẩn cảm biến line: đặt thanh cảm biến lên line, xe tự quay trái phải qua line khoảng 5 s |
| `wcal` | Đo `vfull`: xe chạy 50 % trong 1,6 s, cần 1,5 m trống phía trước |
| `cprcnt` | Đo `cpr`: gõ lệnh, quay tay một bánh đúng 10 vòng, gõ lệnh lần nữa |
| `raw` | In 8 mắt, vị trí line, khoảng cách VL53L0X và hướng xe trong 5 s |
| `noise` | Kiểm tra nhiễu cảm biến khi motor tắt và chạy (xe trên nền trắng, nhấc bánh) |
| `btn` | Hiện trạng thái nút trong 10 s |
| `scan` | Quét bus I2C |
| `info` | In thông số hiện tại |
| `export` | In toàn bộ thông số thành một dòng |
| `load <dòng>` | Nạp dòng do `export` in ra |
| `reset` | Khôi phục mặc định từ `USER_CFG`; sau đó chạy `cal` |
| `flip` | Đảo thứ tự mắt cảm biến; sau đó chạy `cal` |
| `map` / `mapclr` | In / xoá bản đồ sa bàn đã ghi |
| `go` | Bắt đầu chạy (như nút START) |
| `log` | In nhật ký sự kiện của lượt chạy trước |

## Giá trị đổi cho motor mới

Từ C3.13 trở đi dùng cho motor nhanh hơn (hộp số khoảng 20,7:1). Giá trị cho motor 500 rpm vẫn giữ trong `src/archive/Robot_Competition_C311b`.

| Lệnh | C3.11b (500 rpm) | C3.14 (motor mới) |
|---|---:|---:|
| `kp` | 14.5 | 10.2 |
| `kd` | 0.55 | 0.39 |
| `sp` | 76 | 80 |
| `cspd` | 54 | 38 |
| `cpr` | 816.9 | 576.3 |
| `vfull` | 1157.63 | 1640 (ước tính, chạy `wcal`) |
| `vout` / `vpar` | 70 | 49 |
| `vret` | 60 | 42 |
| `vturn` | 70 | 49 |
| `vclust` | 40 | 25 |

## Tên lệnh cũ

Các bản trước dùng tên lệnh tiếng Việt. Firmware vẫn nhận các tên này, nên dòng `export` cũ vẫn nạp được.

| Cũ | Mới | Cũ | Mới |
|---|---|---|---|
| `xuat` | `export` | `cum` | `cluster` |
| `ham` | `brake` | `vcum` | `vclust` |
| `tre` | `lag` | `pcum` | `pclust` |
| `dem` | `cprcnt` | `qd` | `uturn` |
| `cong` | `curve` | `nedo` | `avscan` |
| `nhieu` | `noise` | `ngare` | `jnmode` |
| `mapxoa` | `mapclr` | `nap` | `load` |
