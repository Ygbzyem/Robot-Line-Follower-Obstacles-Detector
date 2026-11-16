# Changelog

All notable changes to the contest firmware. Format based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [C3.9] - 2026-11-10

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
