# Components

Parts used on the robot, with the price the team paid and where to buy them.

| # | Part | Qty | Role | Unit price (VND) | Shop | Link |
|---|---|---:|---|---:|---|---|
| 1 | MakerEDU MKE-K01 ESP32-S3 Dev Kit (DevKitC-1 compatible) | 1 | Main controller: reads every sensor, runs line following and obstacle avoidance, drives the motors, serves the Wi-Fi tuning page | 235k–295k VND | Hshop | [hshop.vn](https://hshop.vn/mach-phat-trien-mke-k01-esp32-s3-dev-kit) |
| 2 | MakerEDU MKE-B01 ESP32-S3 DK IO Shield | 1 | Holds the ESP32-S3, brings every GPIO out to pin headers and steps the battery voltage down to power the board | 90k VND | Hshop | [hshop.vn](https://hshop.vn/mach-de-ket-noi-mke-b01-esp32-s3-dk-io-shield) |
| 3 | JGA N20 gear motor with encoder, 500 rpm | 2 | Drive wheels; the encoders measure wheel speed and distance | 280k VND | Điện Tử Tuyết Nga | [dientutuyetnga.com](https://dientutuyetnga.com/products/jga-n20-dong-co-giam-toc-encoder-500rpm) |
| 4 | TB6612FNG dual motor driver | 1 | Dual H-bridge: turns the PWM and direction signals into motor current from the battery | 45k VND | Hshop | [hshop.vn](https://hshop.vn/mach-dieu-khien-dong-co-dc-tb6612fng-dc-motor-driver) |
| 5 | 8-channel analog line sensor bar (photoresistor) | 1 | Reads the line position under the front of the robot (channels D1…D8) | 120k VND | Xe Dò Line (Shopee) | [shopee.vn/xedoline](https://shopee.vn/xedoline) |
| 6 | VL53L0X ToF distance sensor | 1 | Measures the distance to the obstacle ahead | 115k VND | Hshop | [hshop.vn](https://hshop.vn/cam-bien-khoang-cach-tof-laser-radar-vl53l0x) |
| 7 | MPU6050 (GY-521) | 1 | Gyroscope: holds the heading on straights and measures turn angles | 85k VND | Hshop | [hshop.vn](https://hshop.vn/cam-bien-6-dof-bac-tu-do-gy-521-mpu6050) |
| 8 | OLED 0.96" SSD1306 I2C | 1 | Shows the menu, sensor status, battery voltage and run results | 65k VND | Hshop | [hshop.vn](https://hshop.vn/lcd-oled-trang-0-96-inch-i2c) |
| 9 | Push button 6 × 6 mm | 3 | START / SET / MODE: start a run, run the page action, change page | 1.4k VND | Thế Giới IC | [thegioiic.com](https://www.thegioiic.com/nut-nhan-6x6mm-cao-6mm-4-chan-xuyen-lo) |
| 10 | MKE-M03 buzzer module | 1 | Beeps at start, at track events and on warnings | 25k VND | Hshop | [hshop.vn](https://hshop.vn/mach-coi-bao-mke-m03-buzzer-module) |
| 11 | 2S balance and protection board, 20 A | 1 | Protects the pack from overcharge, over-discharge and short circuit, and balances the two cells while charging | 34k VND | Điện Tử Tuyết Nga | [dientutuyetnga.com](https://dientutuyetnga.com/products/mach-sac-can-bang-bao-ve-pin-2s-20a-1) |
| 12 | BAK 18650 cell, 2500 mAh, 12C 30 A | 2 | Two cells in series make the 2S pack (7.4 V nominal, 8.4 V full) | 65k VND | Điện Tử Tuyết Nga | [dientutuyetnga.com](https://dientutuyetnga.com/products/bak-pin-18650-cong-suat-cao-2500mah-12c-30a) |
| 13 | Capacitors 470 µF + 100 nF | 1 set | At TB6612 VM: 470 µF absorbs motor current spikes, 100 nF filters high-frequency noise | 5.5k VND | Điện Tử Tuyết Nga | [dientutuyetnga.com](https://dientutuyetnga.com/) |
| 14 | Quick wire connector, 2-wire, 0.34–0.54 mm² (22–20 AWG) | 1 | Joins the battery leads to layer 2 without soldering, so the pack can be unplugged to charge (no fuse) | 8k VND | Điện Tử Tuyết Nga | [dientutuyetnga.com](https://dientutuyetnga.com/) |
| 15 | Resistors 100 kΩ + 33 kΩ + 2 × 10 kΩ | 1 set | 100 k + 33 k: battery voltage divider to GPIO4; 10 k: pull-downs on PWMA / PWMB so the motors stay off while the ESP32-S3 boots | 6k VND | Điện Tử Tuyết Nga | [dientutuyetnga.com](https://dientutuyetnga.com/) |
| 16 | 3D chassis base, made to order | 1 | Layer 1: holds the motors, the battery pack and the line sensor arm | 120k VND | Xe Dò Line (Shopee) | [shopee.vn/xedoline](https://shopee.vn/xedoline) |
| 17 | Silicone wheels, pair | 1 | High-grip wheels for the N20 motors | 120k VND | Xe Dò Line (Shopee) | [shopee.vn](https://shopee.vn/product/97386457/27856237220) |
| 18 | Perfboard and wires | 1 | Layer 2 board and its wiring | 41k VND | IC Đây Rồi | [icdayroi.com](https://icdayroi.com/) |
| 19 | Brass standoffs, screws and nuts | 1 bag | Hold layer 2 above layer 1 | ~100k VND | Ốc Vít Min | [ocvitmin.com](https://ocvitmin.com/) |
| 20 | Power switch (SPST) | 1 | Turns the robot on and off (SW1 on layer 1) | 2k VND | Điện Tử Tuyết Nga | [dientutuyetnga.com](https://dientutuyetnga.com/products/kcd1-cong-tac-on-off-3a-2-chan-1) |
| 21 | 2-pin battery connector (J1, J2) | 2 | Connect the battery pack and the layer 1 power wires | 16k VND | Điện Tử Bách Việt | [dientubachviet.com](https://dientubachviet.com/dau-noi-lo-xo-2-chau-chuyen-dung) |
| 22 | Pin headers and jumper wires | 1 set | Plug the modules into the IO shield | 60k VND | Hshop | [hshop.vn](https://hshop.vn/) |
| 23 | Hex Coupling, 3mm Wheel Coupling | 2 | Connect the motor to the wheel | 52k VND | Shopee | [shopee.vn](https://shopee.vn/product/1485101835/43379572221) |
| | **Total** | | | **≈ 2,080.7k–2,140.7k VND** | | |

Prices are what the team paid in 2026 and may change. The total multiplies each unit price by its quantity; the range comes from the price of the ESP32-S3 board.

The cells are charged off the robot with a 2S (8.4 V) charger, which is not in this list.

The battery pack is the author's own design; see [Mechanical Build](../../README.md#mechanical-build) for its benefits and limitations.
