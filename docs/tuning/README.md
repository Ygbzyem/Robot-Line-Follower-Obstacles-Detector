# Tuning Commands

**English** | [Tiếng Việt](vn.md)

Every setting of the contest firmware (`src/competition/Robot_Competition`) can be changed without re-flashing. This page lists each command, its value in the current firmware (C3.14, new motors) and when to change it.

## How to send a command

- **Serial Monitor**, 115200 baud, line ending *Newline*:
  - one value: `kp 12`
  - several values: `kp 12;kd 0.4;sp 70`
  - `export` prints every parameter in one line; paste that line back (or `load <line>`) to restore it.
- **Wi-Fi page**: join `LINE-ROBOT`, open `192.168.4.1`, edit the fields under *Parameters* and press *Save parameters*.
- **OLED + buttons**: `sp`, `vmax`, avoid side and Wi-Fi can be changed from the menu pages with SET.

Values are saved on the robot (NVS) and survive power-off and re-flashing. The `USER_CFG` line at the top of the sketch holds the defaults; `reset` loads them again.

Change one value at a time, run, then compare. Speeds are in % of full speed, distances in mm, angles in degrees.

## Tuning order

1. `cal`: calibrate the line sensor.
2. `wcal`: measure the wheel speed at full power (`vfull`).
3. `lag`: set the sensor latency measured with `Test_LineSensor`.
4. `kp`, `kd` at low speed (`sp 30`, `vmax 30`).
5. Raise `sp` and `vmax` step by step; adjust `cspd`, `brake`, `kpb`, `piv`, `alat` for bends and corners.
6. Junctions and clusters: `jmode`, `vclust`, `pclust`.
7. Obstacle avoidance: `obs`, `aout`, `out`, `par`, `aback`.
8. Finish box: `fin`, `fgo`.

## Speed

| Command | Value | What it does | When to change |
|---|---:|---|---|
| `sp` | 80 | Base speed on curves and through junctions | Robot wobbles or leaves the line on curves → lower |
| `vmax` | 100 | Speed once the line has been straight and steady for 0.25 s; must be ≥ `sp` | Robot slides or wobbles on straights → lower |
| `cspd` | 38 | Speed the robot drops to as soon as the line reaches the edge of the sensor bar (90° corner or sharp bend ahead) | Overshoots corners → lower; too slow into corners → raise |
| `alat` | 3800 | Limits the speed entering a bend: the sharper the bend, the slower. `0` = off; needs `vfull` | Slides out in bends → lower; too slow in gentle bends → raise |

## Line following

| Command | Value | What it does | When to change |
|---|---:|---|---|
| `kp` | 10.2 | Steering gain: correction proportional to how far the line is from the center | Slides out on curves → raise; keeps wobbling → lower |
| `kd` | 0.39 | Damps steering wobble | Zigzags → raise by 0.05; jerks or vibrates → lower |
| `brake` | 0.78 | Cuts speed when the line is far from the center (0.45–0.95) | Slides out of curves → raise; too slow in curves → lower |
| `kpb` | 1.45 | Multiplies `kp` when the line is 1.8 channels or more from the center (1.0 = off) | Loses the line in sharp bends → raise; swings out of bends → lower |
| `piv` | 1.75 | When the line reaches this far out (in channels) and it is not a junction, the robot pivots with the inner wheel reversing. `0` = off | Loses the line at the edge → lower; jerky on gentle curves → raise |
| `curve` | 1 | Line lost in a curve or S-bend: `1` = brake and turn back at once, `0` = handle it like a 90° corner (older behavior) | Keep `1` unless the robot turns the wrong way after losing the line |
| `lag` | 8 | Compensates the delay of the photoresistor sensor bar (ms). `0` = off | Use the value measured by `Test_LineSensor`; steering turns noisy → lower |
| `tlead` | 20 | Stops a turn slightly early so it does not overshoot (ms). The log shows `WRONG TURN ANGLE` when a turn misses by more than 5° | Overshoots turns → raise (40–60); stops short → lower |

## Wheel speed control

| Command | Value | What it does | When to change |
|---|---:|---|---|
| `wpid` | 1 | `1` = each wheel keeps its target speed using the encoder; `0` = direct PWM | Keep `1`; use `0` only to test without encoders |
| `vfull` | 1640 | Wheel speed at 100 % power (mm/s). Set by `wcal`; `0` turns wheel speed control off | Run `wcal` again after changing motors, wheels or gearbox |
| `wkp` | 1.5 | Wheel speed controller: reaction to speed error | Wheels surge or oscillate → lower |
| `wki` | 6 | Wheel speed controller: removes steady speed error | Wheels slow down under load → raise; slow oscillation → lower |

## Junctions and clusters

| Command | Value | What it does | When to change |
|---|---:|---|---|
| `jmode` | 1 | Inside square or hexagon clusters: `1` = take the branch that brings the robot back to its straight heading (passes through the cluster), `0` = always the `pref` side | `0` may make the robot circle one hexagon; keep `1` |
| `pref` | 0 | Side taken at a T-junction: `0` = left, `1` = right | Set to the side the track needs |
| `jnmode` | 1 | Within 60 cm of a junction or cluster, use the careful following mode that takes the right branch at touching circles and squares | Keep `1` |
| `cluster` | 1 | Square clusters and back-to-back junctions: `1` = hold heading and turn at the apex, `0` = plain handling | Keep `1` |
| `vclust` | 25 | Speed inside a square cluster; slower means less overshoot | Overshoots inside the grid → lower |
| `pclust` | 2 | Side taken at a perpendicular T inside a square cluster: `0` = left, `1` = right, `2` = same as `pref` | Set when the grid needs a fixed side |
| `uturn` | 1 | Remembers the path already driven so the robot does not turn back the way it came. `0` = off | Keep `1` |
| `mask` | 0 | Turns off faulty sensor channels: D1 = 1, D2 = 2, D3 = 4, D4 = 8, D5 = 16, D6 = 32, D7 = 64, D8 = 128; add values for several | A channel reads wrong → set its value (D8 bad → `mask 128`) |
| `auto` | 1 | `1` = ignore a channel that reads false black more than 25 % of the time during a run; `0` = only `mask` is used | Keep `1` |

## Obstacle avoidance

The robot avoids in four segments: ① turn out by `aout`, ② drive `out` diagonally, ③ drive `par` parallel to the line, ④ return at `aback`.

| Command | Value | What it does | When to change |
|---|---:|---|---|
| `obs` | 280 | Distance (mm) at which the VL53L0X triggers avoidance | Nose hits the brick while turning out → raise; avoids too early → lower |
| `side` | 0 | Avoid side: `0` = left, `1` = right | Pick the side with free space |
| `avmode` | 2 | `0` = robot measures the brick and plans the path, `1` = fixed distances with a stop after each segment, `2` = fixed distances without stopping between segments | Contest uses `2` |
| `aout` | 58 | ① Turn-out angle (30–90°) | Too close to the brick → raise; swings out too wide → lower |
| `out` | 76 | ② Diagonal distance after turning out (mm) | Goes out too far → lower; scrapes the brick side → raise |
| `par` | 500 | ③ Parallel distance to pass the brick (mm) | Goes too far → lower; tail scrapes the brick on the way back → raise |
| `aback` | 65 | ④ Return angle to the line (20–90°) | Crosses the line too far → lower; return too slow → raise |
| `retm` | 1 | Reaching the line again: `1` = brake, center on the line, then turn (reliable); `0` = follow an arc onto the line (faster, needs an accurate `sens`) | Keep `1` |
| `avscan` | 0 | `1` = also look for the line during ② and ③; `0` = only while returning | Keep `0` on narrow tracks so the robot does not catch another line |
| `vout` | 49 | Speed in ② | Slides past the set distance → lower |
| `vpar` | 49 | Speed in ③ | Misses the line on a curve → lower |
| `vret` | 42 | Speed while returning (drops further near the line) | Crosses the line widely → lower |
| `vturn` | 49 | Turn speed in ① and ④ | Overshoots the angle → lower |
| `brev` | 35 | Reverse power used to brake at the end of a segment (%). `0` = short-brake only | Still slides → raise (40–60); jerks backward → lower |
| `bms` | 50 | Maximum braking time (ms) | Typical 40–80 |
| `dec` | 50 | Slow-down stretch before each stop point (mm) | Still slides past → raise (60–80); avoidance too slow → lower |
| `lat` | 165 | `avmode 0` only: minimum sideways offset (mm) | Only with `avmode 0` |
| `pass` | 80 | `avmode 0` only: distance driven past the back of the brick before turning back (mm) | Only with `avmode 0` |
| `nobs` | 0 | Number of obstacles before the VL53L0X is turned off, so the track border is not taken for a brick. `0` = unlimited | Set `2` if the border triggers false avoidance after both bricks |

## Map run (DEMO / FAST)

| Command | Value | What it does | When to change |
|---|---:|---|---|
| `mode` | 0 | `0` = normal, `1` = DEMO (drive once and record the track map), `2` = FAST (replay the map). Returns to `0` at power-off; the map is kept | Also selectable on the READY page |
| `vfast` | 100 | FAST mode: speed when the map says the next 25 cm is straight | Wobbles on straights → lower |
| `mpre` | 250 | FAST mode: how far before a corner, curve or obstacle to start slowing down (mm) | Overshoots corners → raise (300–400); slows too early → lower |

## Finish box

| Command | Value | What it does | When to change |
|---|---:|---|---|
| `fin` | 500 | Distance (mm) the robot must have driven before a black box counts as the finish, so the start box is skipped | Stops in the middle of the track → raise |
| `fgo` | 180 | After the sensor bar reaches the finish box, drive this far straight (mm) and stop, so the robot ends in the middle of the box | Stops short of the middle → raise; overshoots → lower |

## Robot dimensions

Measure these with a ruler when the mechanics change.

| Command | Value | What it does | When to change |
|---|---:|---|---|
| `sens` | 115 | Distance from the sensor row to the wheel axle (mm), used at corners and when catching the line again | After moving the sensor bar |
| `track` | 128 | Distance between the centers of the two wheels (mm), used for turns | After changing wheels or motor mounts |
| `wheel` | 48 | Wheel diameter including the tire (mm) | After changing wheels |
| `cpr` | 576.3 | Encoder pulses per wheel turn. Check: push the robot 60 cm, the Sensor page should show D ≈ 600 | After changing motors or gearbox; measure with `cprcnt` |

## System

| Command | Value | What it does | When to change |
|---|---:|---|---|
| `roled` | 0 | OLED updates while running: `0` = off, keeps the control loop regular | Keep `0` in the contest |
| `wifi` | 1 | Wi-Fi access point while idle | Set `0` before the contest: the password is public |
| `vk` | 1 | Correction factor for the battery reading | Set by `vcal`; rarely typed by hand |
| `vcal <volts>` | – | Type the battery voltage measured with a multimeter (for example `vcal 8.12`); the robot works out `vk` | When the OLED battery voltage differs from the meter |

## Action commands

| Command | What it does |
|---|---|
| `cal` | Calibrate the line sensor: with the sensor bar on the line, the robot turns left and right over it for about 5 s |
| `wcal` | Measure `vfull`: the robot drives at 50 % for 1.6 s, needs 1.5 m clear ahead |
| `cprcnt` | Measure `cpr`: type it, turn a wheel exactly 10 turns by hand, type it again |
| `raw` | Print the 8 channels, line position, VL53L0X distance and heading for 5 s |
| `noise` | Sensor noise test with motors off and on (robot on white floor, wheels lifted) |
| `btn` | Show button states for 10 s |
| `scan` | Scan the I2C bus |
| `info` | Print the current settings |
| `export` | Print every parameter in one line |
| `load <line>` | Load a line printed by `export` |
| `reset` | Restore the defaults from `USER_CFG`; run `cal` afterwards |
| `flip` | Reverse the channel order; run `cal` afterwards |
| `map` / `mapclr` | Print / clear the recorded track map |
| `go` | Start a run (same as START) |
| `log` | Print the event log of the last run |

## Values changed for the new motors

C3.13 and later target the faster motors (gearbox about 20.7:1). The 500 rpm values are kept in `src/archive/Robot_Competition_C311b`.

| Command | C3.11b (500 rpm) | C3.14 (new motors) |
|---|---:|---:|
| `kp` | 14.5 | 10.2 |
| `kd` | 0.55 | 0.39 |
| `sp` | 76 | 80 |
| `cspd` | 54 | 38 |
| `cpr` | 816.9 | 576.3 |
| `vfull` | 1157.63 | 1640 (estimate, run `wcal`) |
| `vout` / `vpar` | 70 | 49 |
| `vret` | 60 | 42 |
| `vturn` | 70 | 49 |
| `vclust` | 40 | 25 |

## Old command names

Earlier builds used Vietnamese names. They are still accepted, so old `export` lines load unchanged.

| Old | New | Old | New |
|---|---|---|---|
| `xuat` | `export` | `cum` | `cluster` |
| `ham` | `brake` | `vcum` | `vclust` |
| `tre` | `lag` | `pcum` | `pclust` |
| `dem` | `cprcnt` | `qd` | `uturn` |
| `cong` | `curve` | `nedo` | `avscan` |
| `nhieu` | `noise` | `ngare` | `jnmode` |
| `mapxoa` | `mapclr` | `nap` | `load` |
