# STM32-DuraQuad

STM32-based quadcopter firmware bring-up focused on **embedded control**, **sensor integration**, and **real-time debugging** on an STM32F401 platform.

This repository captures the lower-level firmware side of Aaron's multirotor work: setting up timers/PWM, UART debug output, I2C IMU communication, and an iterative flash-debug-test workflow on STM32 hardware.

## What this repo demonstrates
- Embedded C development on **STM32F4**
- **CubeMX / HAL**-generated project structure with custom user code
- **BNO055 IMU** integration over I2C
- **PWM output** setup for motor-control experimentation
- On-target debugging with **OpenOCD** + **gdb-multiarch**
- Serial telemetry / printf-style instrumentation over UART

## Repository layout
- `Core/` — application code and STM32 user logic
- `Drivers/` — STM32 HAL and CMSIS dependencies
- `Makefile` — command-line build flow
- `startup_stm32f401xe.s` — startup code for the MCU
- `STM32F401RETx_FLASH.ld` — linker script
- `.vscode/` — local debug/build configuration for iterative firmware development

## Current firmware snapshot
The checked-in firmware currently includes:
- UART-backed `_write()` support for debug prints
- BNO055 initialization and NDOF mode setup
- an I2C scan on boot for device bring-up validation
- periodic Euler-angle telemetry (`heading`, `roll`, `pitch`)
- PWM startup on multiple timer channels for quad hardware experiments
- LED heartbeat / timing diagnostics

## Development workflow
### Clone
```bash
git clone --recurse-submodules https://github.com/aarwitz/STM32-DuraQuad.git
cd STM32-DuraQuad
```

### Build
```bash
make
```

### Flash + debug
Typical debug flow used in this repo:

```bash
gdb-multiarch build/MyFirstCubeMXProject.elf
(gdb) target extended-remote :3333
(gdb) monitor reset halt
(gdb) load
(gdb) monitor reset init
(gdb) continue
```

In another terminal, start OpenOCD:

```bash
openocd \
  -f /usr/share/openocd/scripts/interface/stlink-dap.cfg \
  -f /usr/share/openocd/scripts/target/stm32f4x.cfg
```

For serial monitoring:

```bash
minicom -D /dev/ttyACM0 -b 115200
```

## Notes
- This is an experimental firmware repo rather than a polished flight stack.
- Some project paths and debug settings reflect Aaron's local STM32/CubeMX environment.
- Build artifacts should ideally be excluded from future cleanup passes.

## Related projects
- [`aarwitz/Quadcopter-PID-Controller`](https://github.com/aarwitz/Quadcopter-PID-Controller) — Arduino-based flight-control experiments
- [`aarwitz/Gazebo-Digital-Twin-Plugin`](https://github.com/aarwitz/Gazebo-Digital-Twin-Plugin) — simulation-side digital twin for the quad platform
