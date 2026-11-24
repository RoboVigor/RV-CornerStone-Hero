# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

STM32F427II firmware (RoboVigor "CornerStone" base, Hero/infantry robot) on FreeRTOS + ST Standard Peripheral Library. Comments, commit messages, and docs are mostly Chinese; files are UTF-8.

Deeper onboarding material already exists — read it rather than re-deriving:
- `system-overview-for-agent.md` — code-grounded architecture, task table, protocol IDs, known traps (snapshot from 2026-04; see "Stale docs" below)
- `docs/README.md` + `docs/session-*.md` — ramp-up sessions (partially stale)
- `user/infantry/README.md` — operator key bindings

## Build / flash / test

- Two builds share one file list, `mdk/infantry.uvprojx` (CMake parses the C sources and include paths from it):
  - **GCC/CMake (macOS/Linux):** `cmake --preset default` (-O0, `build/`) or `cmake --preset release` (-O2, `build-release/`), then `cmake --build build` (or `build-release`). Flash with `cmake --build build -t flash` (OpenOCD + CMSIS-DAP; flashes real hardware, so only when asked). Needs `arm-none-eabi-gcc`, Ninja, OpenOCD; details in `cmake/README.md`.
  - **Keil MDK (Windows):** `Target 1`, ARM Compiler 5.06 (ARMCC, `__CC_ARM`). `UV4 -b mdk\infantry.uvprojx -t "Target 1" -o build.log`; flash with `UV4 -f ...` (J-Link, see `mdk/JLinkSettings.ini`). Also via the VS Code "Keil Assistant" extension (`mdk/.vscode/`). It cannot run on macOS.
- A GCC pass says nothing about Keil: state which one was actually built. Code must compile with both. GCC-only pieces (startup, linker script, syscalls) live in `cmake/`; the FreeRTOS port is swapped from RVDS to GCC automatically.
- Project-wide defines live in the uvprojx `<Define>` and are repeated in `CMakeLists.txt` (keep both in sync): `STM32F427_437xx, USE_STDPERIPH_DRIVER, ROBOT_LOOP_ONE, ARM_MATH_CM4, USE_USB_OTG_FS`.
- **New `.c` files must be added to a group in `mdk/infantry.uvprojx`** (groups: core, library, driver, user_common, user_loop_infantry, device); new include dirs go in its `<IncludePath>`. Dropping a file in a directory does nothing. CMake picks up the change by itself (it re-configures when the uvprojx changes).
- No unit tests. Verification is on hardware; runtime telemetry goes out via VOFA (`Vofa_Send(..., &Node_Debug, 0x1024)` on USART6, fields `debug1..N` set in `tasks.c`).
- Formatting: `.clang-format` (LLVM base, 4-space indent, right-aligned `*`, aligned consecutive assignments/declarations, no include sorting).

## Architecture (big picture)

Data flows in one direction: **ISR → Bridge → shared globals → control tasks → `Motor_*.input` → `Task_Can_Send` → CAN ESCs.**

- `user/infantry/main.c` — inits motors/PIDs/protocol nodes, BSP peripherals, binds bus endpoints to handles (`Bridge_Bind`), creates FreeRTOS tasks, starts scheduler. `#define __HANDLE_GLOBALS` here makes `handle.h` *define* (not extern) every global.
- `user/infantry/handle.h` — the single home of all shared runtime state (motors, PIDs, `ProtocolData`, nodes, mode flags, `InitEventGroup`). Globals are shared lock-free between ISRs and tasks.
- `user/infantry/tasks.c` — all robot behavior. `Task_Control` turns remote/keyboard input into mode flags; `Task_Gimbal`, `Task_Chassis`, `Task_Fire_Stir`, `Task_Fire_Frict` read flags + feedback and write `Motor_*.input`; `Task_Can_Send` is the only actuation egress (`Bridge_Send_Motor(&BridgeData, SafetyMode)`, which sends zeros when `SafetyMode`); `Task_Host` handles supercap command (CAN 0x4ff) and host autoaim data; `Task_Imu` runs the gyroscope/EKF and sets `INIT_EVENT_IMU`. Control tasks block on `InitEventGroup` until IMU is ready. Check `main.c` for which tasks are actually created — several are defined but commented out.
- `user/infantry/interrupt.c` — IRQ handlers: USART1 = DBUS remote; USART6/UART7/UART8 and CAN1/CAN2 RX → `Bridge_Receive_*`; EXTI9_5 = IMU data-ready semaphore; TIM2 = high-frequency tick.
- `driver/src/Driver_Bridge.c` — maps physical endpoints (CAN ID per bus, USART number) to motor handles or protocol `Node_Type`s. To route a new device, the `main.c` bind + ISR + bridge lookup must all agree.
- `driver/src/Driver_Protocol.c` + `user/infantry/protocol.h` — framed packet protocol (header/len/CRC8/CRC16, CAN frames partially skip CRC8). `protocol.h` declares packed payload structs and `PROTOCOL_INFO_LIST`; offsets into `ProtocolData_Type` are computed at init. Command ID ranges: judge 0x0000–0x03FF, host 0x0400–0x04FF, board 0x0500–0x05FF.
- Other drivers: `Driver_Motor/PID/Chassis/Gyroscope/Filter/Remote/Vofa/BSP`. `library/` holds FreeRTOS, EKF (`library/ekf`), IMU drivers (bmi088/mpu6500/ist8310), USB OTG CDC, DM motor driver, `vegmath`.

### Configuration

- Compile-time params: `user/infantry/config.h` overrides `user/common/default_config.h` (chassis geometry, power-limit polynomial, reduction ratios, IMU mounting rotation `angle_x/y/z`, pitch limits).
- Runtime robot identity: GPIO strap pins read by `BSP_Stone_Id_Init` → `Robot_Id`; macros `ROBOT_MIAO/WANG/SHARK` (`Robot_Id == 1/2/3`) gate per-robot behavior (e.g. magazine servo presets).

## Rules / traps

- **`protocol.h` is generated** ("由 Node_Bridge 生成, 请勿直接修改", from judge/host/user `.yml` not in this repo). Hand edits shift offsets and silently break decoding.
- This is competition-era code: expect hard-coded overrides, commented-out alternatives, and tuning shortcuts in `tasks.c` (e.g. fixed friction target speeds, forced flags). Treat `tasks.c` as current behavior, not intent; flag suspicious overrides instead of "fixing" them unasked.
- `library/usb/.../usbd_cdc_if.c` is not pure vendor code — it feeds bytes into `Protocol_Unpack(&Node_Host, ...)`.
- **No latches that need an MCU reset.** The robot cannot be reset on the field during a match, so protection/cutoff logic must recover by itself once the condition clears, or be clearable by the operator (remote / keyboard / serial command). The pitch `pitchTripped` latch (angle window + 70 °C) was removed for this reason. Decided 2026-11-24: no software over-temperature protection on pitch (the motor has its own) and no IMU-stale cutoff for now; don't re-propose them unprompted.
- Before changing an actuator: find which task writes its `.input`, confirm it leaves via `Task_Can_Send`, and trace the mode flags it reads back to `Task_Control`.

### Stale docs (as of 2026-11)

`system-overview-for-agent.md` / `docs/` predate recent commits. Known differences in current code:
- Host (vision/autoaim) link is **USB CDC** (`USBD_Init` in `main.c`, unpack in `usbd_cdc_if.c`); the UART7 `Node_Host` bind is commented out.
- `Task_Can_Send` priority is 6, not 4.
- CAN bindings changed (e.g. chassis on CAN1 0x202–0x205, yaw CAN1 0x206, pitch CAN2 0x207, friction FL/FR/FT on CAN2 0x202–0x204) — always read `main.c`.

## Workflow

Gitflow: work on `develop` (or feature branches off it); `main` is the release branch.
