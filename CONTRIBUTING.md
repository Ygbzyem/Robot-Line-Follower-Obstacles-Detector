# Contributing

## Workflow

1. Create a branch from `main`: `feat/<topic>`, `fix/<topic>` or `docs/<topic>`.
2. Keep one focused change per commit and use [Conventional Commits](https://www.conventionalcommits.org/):
   `feat(control): ...`, `fix(avoid): ...`, `test(hw): ...`, `docs: ...`, `build: ...`, `chore: ...`.
3. Build with PlatformIO before pushing (`pio run`); the CI build must pass.
4. Test on the robot and write the result in the pull request.
5. Add an entry to `CHANGELOG.md`.

## Code style

- English only: code, comments, Serial / OLED / web text, file names.
- 2-space indentation, UTF-8, LF line endings (see `.editorconfig`).
- New parameters must be stored in NVS and appear in the `export` line.
- Keep the tuned values in `USER_CFG` when changing defaults.

## Releases

The firmware flashed on the robot is `src/competition/Robot_Competition`. When a new contest version replaces it, move the previous file to `src/archive/` and tag the commit (e.g. `C3.9`) after it has been tested on the track.
