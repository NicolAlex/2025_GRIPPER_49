<!-- .github/copilot-instructions.md - project-specific guidance for AI assistants -->

This repository is a PlatformIO-based firmware project for an ESP32 board (Arduino framework).
Keep instructions concise and actionable: reference files, env names, and exact commands.

- Project layout
  - `platformio.ini` — single environment `[env:esp32doit-devkit-v1]` (board: esp32doit-devkit-v1, framework: arduino). Use this env name when running PIO commands.
  - `src/` — C++/Arduino source. Primary entry: `src/main.cpp` (currently minimal; include `Arduino.h` and implement `setup()`/`loop()` as usual).
  - `include/` — project headers. Use `#include "my_header.h"` for headers placed here.
  - `lib/` — private libraries compiled by PlatformIO. Each library should live in its own subfolder and expose headers from `src`.
  - `test/` — PlatformIO unit tests. Use the PlatformIO Test Runner for CI or local testing.

- Build / test / upload (explicit examples)
  - Build the default env: `pio run -e esp32doit-devkit-v1`
  - Upload to device (uses env from `platformio.ini`): `pio run -t upload -e esp32doit-devkit-v1`
  - Run unit tests (PlatformIO test runner): `pio test -e esp32doit-devkit-v1`
  - Serial monitor: `pio device monitor -e esp32doit-devkit-v1`

- Dependencies and important config
  - `platformio.ini` contains `lib_deps = microbahner/MobaTools@^2.7.0`. When adding libs prefer `lib/` for private code and `lib_deps` for upstream libs.
  - Do not modify the env name unless you update example commands in this file and any CI.

- Source patterns and conventions to follow
  - Prefer small headers in `include/` and implementation in `src/` to keep PlatformIO's LDF simple.
  - Use Arduino-style `setup()` and `loop()` in `src/main.cpp` for sketch-like firmware. If switching to a non-Arduino entry, update `platformio.ini` accordingly.
  - Put reusable components in `lib/<ComponentName>/src` and expose a single public header (e.g., `LibName.h`).

- Debugging and build artifacts
  - Build artifacts and virtual envs are under `.pio/`. Don't edit compiled outputs.
  - For interactive debugging or advanced workflows, use PlatformIO's debug features (IDE integration or `pio run --target debug`) — only if the board supports the debug probe configured in `platformio.ini`.

- When modifying files an AI should check
  - `platformio.ini` — for env names, framework, and library deps
  - `src/main.cpp` — entry point and Arduino lifecycle
  - `lib/` and `include/` — where to add reusable code and headers

Examples to reference in edits
- To add a header in `include/`: `#include "my_header.h"` in `src/main.cpp`.
- To add a private library: create `lib/MyLib/src/MyLib.h` and `lib/MyLib/src/MyLib.cpp`, then `#include <MyLib.h>` from `src/`.

If anything in this short guide is unclear or you'd like more details (CI, specific debug setup, or intended hardware behavior), tell me which area to expand and I'll update this file.
