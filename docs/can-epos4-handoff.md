# CANopen / EPOS4-compatible interface handoff

**Goal:** make the drive answer on CAN like a **maxon EPOS4 50/5**, so a master written for an EPOS4 can drive it with little or no change. The intended masters:
- a ROS controller (`ros2_canopen` + `ros2_control`)
- for now, a PC with a USB-CAN adapter

The interface is standard CANopen:
- **CiA 301:** NMT, SDO, PDO, SYNC, EMCY, heartbeat
- **CiA 305:** LSS, node-ID and bit rate set over the bus
- **CiA 402:** drive profile, i.e. the state machine, controlword/statusword and modes
- **plus EPOS4's maxon-specific objects**

**Status, 2026-10-12:** postponed. Nothing implemented. The FDCAN1 peripheral and pins exist in the `.ioc`, but the bit timing is a placeholder. Do `docs/config-gui-handoff.md` first: its object table is meant to become the backing store of the CANopen OD.

## Scope: how far to emulate
| Level | Content | Decision |
|---|---|---|
| A | CiA 301 + 402 using EPOS4's object layout, units, default PDO mapping, modes and error codes | **Do** |
| B | maxon-specific configuration objects (motor data, controller gains, sensor config…) at EPOS4 indices and units, as views onto `g_params` | **Do**, the subset that exists in this drive |
| C | Pass as a real EPOS4 for EPOS Studio / the EPOS Command Library (firmware version objects, all maxon objects, exact behavior everywhere) | **Don't**: large and fragile |

**Identity (0x1018):** report **this project's own** vendor ID/product code by default. maxon's vendor ID makes tools believe the device is a genuine EPOS4, which crosses from compatible to impersonating. If a specific master insists on the EPOS4 identity for bench tests, make that a build option and clear it with the thesis advisor.

## Reference material (needed before starting; put it in `docs/epos4/`)
Every EPOS4 index, subindex, unit and default below must be checked against these. The few given in this document are from memory.
- **EPOS4 XDD/EDS device description:** maxon's EPOS4 product page → Downloads. It also ships with the EPOS Studio / "EPOS4 Software and Documentation" setup; search the install folder for `*.xdd`.
- **EPOS4 Firmware Specification (PDF):** objects, units, bit meanings, error codes.
- **EPOS4 Communication Guide (PDF):** CAN details, and the USB/RS232 protocol (see "USB" below).
- **EPOS4 Application Notes:** CSP/CSV/CST timing, homing methods.
- Use files that match **one firmware version**; the dictionary changes slightly between releases.

## CiA 402 in brief
The mode is selected in **0x6060** (modes of operation); the active one shows in 0x6061. Supported modes are listed in 0x6502.

| Mode | 0x6060 | Master sends | Drive does | Maps onto this firmware |
|---|---|---|---|---|
| PPM, Profile Position | 1 | 0x607A target + 0x6081/0x6083/0x6084 profile vel/acc/dec, then the "new setpoint" bit | Plans a trapezoid, reports "target reached" | Position loop + **new trajectory generator** |
| PVM, Profile Velocity | 3 | 0x60FF target + acc/dec | Ramps and holds | Speed loop with `acc_lim`: almost exactly |
| HMM, Homing | 6 | 0x6098 method + speeds, start bit | Finds the reference, sets zero | Method 37 (current position) = `g_home_req`. Others (index, hard stop) later |
| CSP, Cyclic Sync Position | 8 | 0x607A every SYNC period via RPDO | Tracks it, interpolating between SYNCs | Position loop via `g_pos_ref_new` handshake |
| CSV, Cyclic Sync Velocity | 9 | 0x60FF every SYNC | Tracks it | Speed loop (`g_vel_ref`) |
| CST, Cyclic Sync Torque | 10 | 0x6071 every SYNC | Tracks it | Current loop (`g_iq_ref` = torque / Kt) |

**Order to implement:** CSV → CSP → CST (what ROS uses), then PVM → HMM → PPM.

**State machine (FSA)** on controlword 0x6040 / statusword 0x6041:

| FSA state | Firmware |
|---|---|
| Not ready to switch on / Switch on disabled | `CTRL_OFF`, bridge may be off |
| Ready to switch on / Switched on | `CTRL_OFF`, bridge switching (zero volts), prerequisites (`PF_*`) checked here: refuse "switch on" without `e`/`t` |
| **Operation enabled** | `CTRL_RUN`, `g_loop` from 0x6060 |
| Quick stop active | Decelerate with quick-stop decel (0x6085), as `run_stop()` does, then disable |
| Fault reaction active → Fault | Today's `run_watchdog()` reasons + bridge fault → EMCY, 0x603F error code; fault reset (bit 7) clears it |

**Keep the CLAUDE.md rule:** leaving `CTRL_RUN` while spinning short-brakes. Every transition out of Operation enabled except "disable voltage" goes through a controlled stop (`run_stop()`-like). For "disable voltage", the bridge outputs must go *off* (MOE off, coast), not to a zero vector.

## Units: EPOS4 view ↔ native firmware (verify every row in the Firmware Specification)
| Quantity | EPOS4 (CAN side) | Firmware (native) | Conversion |
|---|---|---|---|
| Position | int32 increments | `pos_q32_t` Q32.32 turns, 2²¹ counts/turn | inc = q >> 11. int32 wraps at ±1024 turns, as on any EPOS4 with a high-resolution encoder; define the wrap behavior |
| Velocity | int32, default unit rpm (check 0x60A9) | float turn/s | × 60 |
| Torque | int16, ‰ of motor rated torque (0x6076) | float A (iq) | via Kt = 1.5 · pp · λ (from kv, `cal_kv()`) |
| Current | mA in maxon objects | float A | × 1000 |
| R / L | maxon motor/electrical objects (mΩ / µH or similar: check) | Ω / H | scale |

## Firmware architecture (proposal)
- **Stack:** **CANopenNode v4** (Apache-2.0) with the STM32 driver from **CanOpenSTM32**, which supports the G4 FDCAN. It covers 301 and LSS. The **402 layer is written here** (FSA, mode mapping, profile generator, homing).
- **Object dictionary:** import the EPOS4 XDD into **CANopenEditor**, delete what this drive doesn't support, add the native 0x4000-range objects from the config-GUI work, and generate `OD.c`/`OD.h`.
  - Use **OD extensions** (read/write callbacks) for the EPOS4 views that need unit conversion, and for objects with side effects (`pi_apply()`, refusing changes while running).
  - Export **our own EDS** from the trimmed OD. That file is what `python-can`/`canopen` and `ros2_canopen` load.
- **Execution:**

  | CANopenNode part | Where it runs | Notes |
  |---|---|---|
  | `CO_process()` (NMT, SDO, heartbeat, LSS, EMCY) | Main loop | Next to `cmd_poll()` |
  | `CO_process_SYNC/RPDO/TPDO` | A **1 kHz interrupt below the ADC priority**, e.g. from TIM4, which is initialized but unused | The control IRQs stay at priority 0 |
  | FDCAN RX/TX interrupts | **Below** priority 0 | Must not delay `currentLoop()` |

  **Breaks a current invariant:** CLAUDE.md says all IRQs are priority 0 and can't preempt each other. Setpoints from PDOs must go to the ISR with the existing rules: write the reference first, then switch the mode; 64-bit position through the `g_pos_ref_new`/`g_pos_ref_req` handshake. Update CLAUDE.md when this lands.
- **Gotchas found in today's code:**
  - **Blocking calibrations** (`HAL_Delay` loops in `calib.c`) stop `CO_process()`, so the heartbeat stops and the master declares the node lost. Call `CO_process()` from inside `chk()`/the delay loops, or turn the calibrations into state machines.
  - **`params_save()` stalls the CPU ~20 ms** (single-bank flash erase). The FDCAN RX FIFOs hold 3 frames each, so frames are lost, and the heartbeat jitters. Allow "store parameters" (0x1010) only outside Operation enabled; that already holds because the save disables the bridge.
  - **Master-loss safety:** consume the master's heartbeat (0x1016) and/or check SYNC timeouts in CSP/CSV/CST. On loss, do the fault reaction (controlled stop), never keep running on a stale setpoint.
- **Cyclic modes:** a SYNC period (e.g. 1 ms) is 2.5 outer-loop cycles (2.5 kHz). Interpolate the setpoint linearly between SYNCs and derive velocity feed-forward from consecutive CSP targets, or the position loop will see steps.
- **Storage:**
  - **Pages 60–61** (reserved now) hold the communication settings: LSS node-ID and bit rate, heartbeat, PDO mapping. Use CANopenNode's `CO_storage` with a small flash backend.
  - **Pages 62–63** keep the motor params (`params.c`, A/B + CRC). 0x1010 "store" can call `params_save()` for the motor part.
  - Pages 60–61 aren't in the program region (`FLASH` LENGTH 120K) and must stay that way.
- **EMCY / error codes:** map the `run_watchdog()`/`chk()` reasons to CiA 402 / EPOS4 error codes (overcurrent, undervoltage, overspeed, position sensor errors from the MT6835 CRC/stall counters, gate-driver fault). Take the code list from the Firmware Specification.

## Hardware prerequisites
- **PB8 is BOOT0 and also FDCAN1_RX.** A CAN transceiver idles RX high, so at reset the chip may boot the ST bootloader. Set the option bytes once per chip; they survive `make erase`. Read them first:
  ```powershell
  $cli = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
  & $cli -c port=SWD -ob displ
  & $cli -c port=SWD -ob nSWBOOT0=0 nBOOT0=1   # boot from the nBOOT0 bit (1 = main flash), PB8 free
  ```
  - **Don't set nBOOT0 = 0:** with nSWBOOT0 = 0 that boots system memory/SRAM every time (RM0440 boot table).
  - **OpenOCD alternative:** `stm32l4x option_write 0 0x20 0x08000000 0x0C000000` then `stm32l4x option_load 0`. In `FLASH_OPTR`, nSWBOOT0 is bit 26 and nBOOT0 bit 27.
  - **Side effect:** the BOOT0-pin way into the ST bootloader is gone. USB DFU updates later need a jump to system memory from the firmware.
- **FDCAN bit timing:** fix it in the `.ioc`. Today it's Seg1 = 1, Seg2 = 1, an invalid ~3.5 Mbit/s.
  - **Settings:** classic CAN, **1 Mbit/s** (EPOS4 default): FDCAN clock 170 MHz (PCLK1), nominal prescaler **10**, Seg1 **13**, Seg2 **3**, SJW **3** → 17 tq, sample point 82 %.
  - CANopenNode/LSS reprogram the timing at runtime for other bit rates.
  - **Filters:** set the standard ID filters for the node's COB-IDs, or accept all and filter in software at first.
- **Bus:**
  - **Wiring:** CANH, CANL **and GND**. The test adapter is isolated, so its CAN ground must be tied to the board's ground.
  - **Termination:** 120 Ω at both ends, ≈ 60 Ω measured between CANH and CANL with the power off.
  - Check whether the board or the adapter has termination built in.

## Test setup
- **Adapter:** WeAct **USB2CANISO V2** (an STM32G431-based, isolated adapter), default firmware **SLCAN**.
- **Windows:** `pip install python-can canopen pyserial`.
  ```python
  import canopen
  net = canopen.Network()
  net.connect(interface="slcan", channel="COM7", bitrate=1000000)
  node = net.add_node(1, "drive.eds")          # our exported EDS
  print(hex(node.sdo.upload(0x1000, 0)))        # device type
  ```
  `canopen.profiles.p402.BaseNode402` drives the FSA and modes, which makes it a good automated test harness. It could also back a CAN tab in the plotter later.
- **Linux / ROS:**
  ```bash
  sudo slcand -o -c -s8 /dev/ttyACM0 can0   # -s8 = 1 Mbit/s
  sudo ip link set can0 up
  ```
  `ros2_canopen` (Lely) uses our EDS plus a `bus.yml`; its CiA 402 driver plus `ros2_control` typically runs CSP/CSV. If WeAct offers candleLight/gs_usb firmware for the adapter, it shows up as native `can0` with better timing for 1 kHz cyclic modes; not needed for PC tests.
- **SLCAN limits:** coarse timestamps and ASCII overhead. Fine for SDO/NMT/PDO with one drive at ~1 kHz.

## Budget
- **Flash:** the debug build has ~49 KB free and release `-Os` ~57 KB (`docs/flash-optimization-handoff.md`). The estimate for this work, with USB CDC staying, is about 25–35 KB:
  - CANopenNode 301 + LSS: ~15–25 KB
  - FDCAN HAL: 2–3 KB
  - 402 layer: ~4–8 KB
  - the OD
- **RAM:** 18.7 KB of 32 KB used; CANopen adds ~2–4 KB.
- **Measure** with an empty application before committing to the full OD.

## Steps
1. **Physical layer:**
   - option bytes (nSWBOOT0), 1 Mbit/s timing in the `.ioc`, start FDCAN, send a test frame (e.g. a 0x701 boot-up `00`) once per second
   - see it with python-can; receive one frame and echo it
   - **Verify:** no error frames, a clean bus with the adapter; the board still boots with the adapter connected and powered first.
2. **CANopenNode bare:** NMT, heartbeat, SDO, our identity, an empty OD. Measure flash/RAM, read 0x1000/0x1018 from Python.
3. **OD from the EPOS4 XDD**, trimmed. Native 0x4000 objects shared with the config GUI; EPOS4 views via OD extensions. Export the EDS.
4. **402 FSA + CSV,** with master-loss (heartbeat consumer) handling. Test with `BaseNode402`: enable → velocity → disable, quick stop while spinning, a fault while spinning, master unplugged while spinning.
5. **CSP** (interpolation + feed-forward) and **CST**. Default PDO mapping per EPOS4. SYNC at 1 ms.
6. **PVM, HMM (method 37 first), PPM** (trapezoid generator).
7. **LSS + storage** in pages 60–61, plus EMCY codes.
8. **ROS:** `ros2_canopen` on Linux with the exported EDS.

## USB (related, optional)
EPOS4's USB/RS232 interface reads and writes the same OD objects with a framed protocol (documented in the Communication Guide; I believe it uses opcodes for read/write object and a CRC — check). If the config protocol from `docs/config-gui-handoff.md` is ever switched to that framing, the GUI could talk to a real EPOS4 as well. Not needed: the object model is what matters, and it is already shared.
