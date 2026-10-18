# Driver TCC

Firmware, PC software and hardware design of a field-oriented-control (FOC) driver for 3-phase BLDC/PMSM motors, built as a graduation thesis (TCC) project.

- **Controller:** STSPIN32G4 (STM32G431 Cortex-M4F at 170 MHz with an integrated 3-phase gate driver), six 100 V MOSFETs, three 2 mΩ low-side shunts.
- **Control:** current loop at 25 kHz; speed and position loops at 2.5 kHz. Commutation from an MT6835 21-bit magnetic encoder, from the encoder with a back-EMF flux observer blended in at speed, or sensorless (high-frequency injection at standstill, the observer at speed).
- **Links:** USB-C (virtual COM port: telemetry, configuration, calibration) and CANopen (CiA 301 node, CiA 402 drive profile: PP, PV, HM, CSP, CSV, CST).
- **Protections:** overcurrent, over/undervoltage, regen clamp, overspeed, encoder loss, gate-driver faults. Stops coast instead of short-braking a spinning motor.
- **PC software:** a Python/Qt GUI with live plots, a parameter editor, a calibration wizard and a CANopen master, plus a CANopen test runner.

## Documentation

| Document | For |
|---|---|
| [docs/user-manual-driver.md](docs/user-manual-driver.md) | Using the driver: connections, flashing, commissioning a motor, run modes, protections, CANopen, parameter reference, troubleshooting |
| [docs/user-manual-software.md](docs/user-manual-software.md) | Using the PC software: plotter (Scope, Config, CANopen tabs), test runner, helper scripts |
| [CLAUDE.md](CLAUDE.md) | Developer overview: build system, CubeMX rules, architecture, execution model, conventions |
| [docs/design-notes.md](docs/design-notes.md) | Design decisions, bench measurements and open items: config protocol, CANopen drive and PC tool, HFI, sensorless tuning, flash budget |

## Repository layout

| Path | Content |
|---|---|
| `Core/` | Firmware: CubeMX-generated peripheral setup plus the hand-written control code (`motorCtrl.c`, `calib.c`, `hfi.c`, `mt6835.c`, `proto.c`, `objdict.c`, `params.c`, …) |
| `CANopen/` | CANopenNode stack, the FDCAN port, the object dictionary (exported from `driverTCC.xdd`) and the CiA 402 layer |
| `Drivers/`, `Middlewares/`, `USB_Device/` | ST HAL, CMSIS and USB device stack (generated) |
| `Driver TCC.ioc` | STM32CubeMX project: the source of truth for the peripheral setup |
| `driverTCC.xdd` | CANopenEditor project of the object dictionary |
| `software/` | PC software: `plotter.py` (GUI), `cotest.py` (CANopen tests), `driverTCC.eds` (EDS for CANopen masters) |
| `hardware/` | KiCad project of the board: schematic (controller, connections, power sheets) and PCB, the schematic as PDF (`Driver TCC schematic.pdf`) |
| `hardware/datasheets/` | Datasheets of the main parts: STSPIN32G4 (controller + gate driver), STM32G431 (its MCU), DI100N10PQ (bridge MOSFETs), MT6835 (encoder), TJA1051 (CAN transceiver), USBLC6-2 (USB ESD protection), ANTC1608-103F3450FB (board NTC). The STM32G4 reference manual (RM0440, register descriptions) is not included: get it from [st.com](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf). |
| `docs/` | Manuals and design notes |

## Quick start

### Firmware
Requirements: arm-none-eabi-gcc, make, OpenOCD and an ST-Link (paths in `.stm32env`). The project is set up for the **STM32 for VSCode** extension.

```
make -f STM32Make.make -j                                   # debug build -> build/debug/Driver_TCC.elf
make -f STM32Make.make -j flash                             # flash over ST-Link
make -f STM32Make.make -j DEBUG=0 OPTIMIZATION=-Os flash    # release build
```

A new board needs its option bytes set once before it boots from flash with CAN connected; see the driver manual, section 4.1. After the first flash, updates can also go over USB DFU (Config tab, **Reboot to DFU**, then STM32CubeProgrammer).

### PC software
```
pip install -r software/requirements.txt
python software/plotter.py
```
Connect to the port marked **[driver USB]**, then follow the calibration steps in the Config tab (driver manual, section 5).

### Hardware
Open `hardware/Driver TCC.kicad_pro` in KiCad (made with KiCad 10).

## License

MIT, see [LICENSE](LICENSE).
