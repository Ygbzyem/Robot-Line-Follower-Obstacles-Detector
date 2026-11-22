# Changelog

All notable changes to the contest firmware. Format based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [C3.13] - 2026-11-21

For the new motors (gearbox about 20.7:1, both wheels 576.3 counts per turn, about 1.4 times faster). Same code as C3.11b; only the defaults change.

### Changed
- `USER_CFG` for the new motors: `cpr 576.3`, `vfull 1640` (estimate, run `wcal`), speed percentages × 0.71 to keep the same real speed in corners, clusters and avoidance (`cspd 38`, `vclust 25`, `vout 49`, `vpar 49`, `vret 42`, `vturn 49`), `kp 10.2`, `kd 0.39`, `sp 80`.
- After flashing run `reset` (loads `USER_CFG`), `cal`, `wcal`, `lag`.

## [C3.11b] - 2026-11-16

Best version for the 500 rpm motors (tag `best-500rpm`). Kept in `src/archive/Robot_Competition_C311b`.

### Added
- Hexagon (honeycomb) cluster: a Y-junction switches to hexagon mode, which locks the main heading to the heading at the cluster entry and picks the branch that brings the robot back to it (log `H`).
- Start inside the black box (from C3.10): drive straight on the gyro heading until the sensor bar leaves the box plus `START_GO` (60 mm), then follow the line (log `P`).

### Changed
- Hexagon entry right after a bend uses the robot heading at the first Y-junction.
- Circle-exit rule is disabled inside hexagon clusters, re-enabled after more than 10 cm of continuous curving.
- Tuned defaults: `track 128`, `aout 58`, `out 76`, `par 500`.

## [C3.9] - 2026-11-10

Kept in `src/archive/Robot_Competition_C39`.

### Added
- Circle-cluster exit: while curving, junctions take the branch on the outer side of the arc.
- Anti-stall when pivoting: PWM ramps up to +30 % if both wheels stay still.

### Changed
- Obstacle on a bend: avoid toward the bend and catch the line as soon as it appears on that side.
- Line search after avoidance sweeps toward where the line continues.
- Spin guard only counts rotation when turning nearly in place.

## [v3.9.1-B] - 2026-11-07

Branch `version-b`.

### Added
- Fast single 90-degree corners (`fastcnr 1`).

### Fixed
- Saving parameters on the web page changed `avmode 2` to 1.

## [v3.8-C] - 2026-11-04

### Added
- Version C: line-following algorithm with wrong-way guard.

### Changed
- Obstacle avoidance and line re-catch returned to the v3.8 logic.

## [v3.5.1-A]

### Added
- Per-wheel speed PID (`wcal`, `wpid`), sensor-lag compensation (`lag`).
- Parameter export (`export`) and `USER_CFG` defaults.
- Flowing obstacle avoidance (`avmode 2`).

## [v3.1]

### Added
- Run modes NORMAL, DEMO (records the track map) and FAST (replays the map).
