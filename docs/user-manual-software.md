# Driver TCC: user manual (PC software)

This manual covers the PC tools in `software/`: the plotter GUI (a USB page with the Scope and Config tabs, and a CANopen page), the CANopen test runner, and the helper scripts. How the driver itself behaves (calibration, modes, protections, CANopen profile, parameters) is in [user-manual-driver.md](user-manual-driver.md).

| Tool | What it is for |
|---|---|
| `plotter.py` | The GUI. Live telemetry, parameters, calibration, running the motor over USB, and a CANopen master over a CAN adapter. |
| `cotest.py` | The CANopen test sequences from the command line (the same tests as the GUI's Tests tab). |
| `drivectl.py` | Headless command line: the Config tab's USB commands and CANopen master commands, one-shot, scripted or as a shell. |
| `check_sync.py` | Checks the PC tables against the firmware sources (for developers). |
| `flash_report.py` | Flash usage per module of a build (for developers). |

---

## 1. Installation

Needs Python 3 on Windows, Linux or macOS.

```
pip install -r software/requirements.txt
```

This installs numpy, pyqtgraph (≥ 0.13), pyserial, PyQt6, python-can (≥ 4.6) and canopen (≥ 2.4). PyQt5 or PySide6 also work instead of PyQt6.

For the CANopen tab you also need a CAN adapter supported by python-can. The bench setup uses a WeAct USB2CANISO with SLCAN firmware; gs_usb (candleLight), PCAN, Kvaser, SocketCAN, IXXAT and Vector are also selectable.

---

## 2. Starting and connecting

```
python software/plotter.py            pick the port in the window
python software/plotter.py COM7       connect to COM7 on start
pythonw software/plotter.py           Windows, without a console window
```

1. Plug in the driver's USB. It appears as a virtual COM port; the port list marks it **[driver USB]**, and it is picked automatically when the last-used port is gone.
2. Press **Connect**. The baud rate only matters for a UART build of the firmware; USB ignores it.
3. On connect the GUI pings the device and compares the protocol version and the object-table hash with its own. A mismatch (GUI and firmware from different versions) is shown in orange above the parameter grid; parameters that differ are disabled rather than written wrongly. Then all parameters are read.

The window remembers its size, the port, the page and tab, and the plot selection between sessions.

**Numbers** are always entered with a `.` decimal point, whatever the operating system's language settings.

---

## 3. Window layout

The window has two pages, one per link: **USB** (the config protocol and telemetry: Scope and Config tabs) and **CANopen** (section 6, with its own left panel).

On the USB page the left sidebar is shared by the Scope and Config tabs:

| Box | Content |
|---|---|
| **Connection** | Port, baud, Connect/Disconnect. |
| **Run** | Motor commands over USB (below). |
| **Status / faults** | State (off / calibration / run, loop, coasting, running calibration), bridge enabled or disabled with nFAULT / break / READY warnings, latched trip, gate-driver status bits, bus voltage, temperatures (board, MCU die, and motor when `mtmax` is set; "—" for an open or shorted sensor; derating with its current cap, over temperature or sensor fault), flash state (saved or unsaved changes, save count). |
| **Statistics** | Control-loop time and its max (red at ≥ 19 µs), encoder reads and CRC / late / stall counters and sensor status, the Hall state with its bad-state and HFI re-seed counters, current offsets in use and their source, link timeouts and device RX errors, CAN node state, frame counters and the CiA 402 state. **Clear max / counters** resets them. **Apply CAN settings** makes a new node ID / bit rate take effect. |
| **Display**, **Plots** | Only with the Scope tab (section 4). |

Status is polled 4 times per second while connected.

The status bar at the bottom shows frames per second, total frames, dropped frames (sequence gaps), resyncs, link timeouts and ● REC while recording.

### 3.1 Run panel
| Control | Sends |
|---|---|
| `id` / `iq` + **Current** | Current mode with these d/q currents (A). |
| rpm + **Speed** | Speed mode. The label next to it shows the speed limit `vlim`. |
| turns + **Position** | Position mode, turns from home. Accepts many digits; it is sent exactly. |
| **Home** | Present position becomes 0. |
| **STOP** | Always enabled. Jumps ahead of any queued request, stops the motor, and also ends a running calibration. |

Pressing Enter in a field sends that field's command. The result ("ok" or the reason for a refusal) goes to the Config tab's log. A stop by the driver's watchdog (overcurrent, encoder lost, …) or an overvoltage resume is shown as a banner under the Run panel.

---

## 4. Scope tab

Live plots of the telemetry the driver sends at 2.5 kHz (one frame every 10 control cycles).

### 4.1 Plots
Tick the plots to show in the **Plots** box:

| Plot | Signals |
|---|---|
| Angle (deg) | Electrical angle. |
| Bus voltage (V) | |
| HFI / Hall error (deg elec) | HFI − encoder (NaN with HFI off); with `angsrc` 3 Hall − encoder (NaN without an encoder), with 4 HFI − Hall. |
| Id / Iq (A) *(default)* | Measured d/q currents. |
| Speed (rpm) *(default)* | Speed and its reference. |
| Position (turns) *(default)* | Position and its reference. |
| Observer − encoder (deg elec) | Observer angle error. |
| Observer flux / blend | \|flux\|/λ and the blend weight (0–1). |
| Speed sources (rpm) | Speed used by the loop, observer speed, encoder speed (Hall speed with `angsrc` 3 and 4). |
| HFI speed (rpm) | HFI and encoder (or Hall) speed. |
| HFI signal (A) | HF current step (size the injection voltage with it). |

All plots share the time axis. Drag to pan, wheel to zoom, double-click (or **Auto-scale**) to return to the full window with automatic vertical scale.

### 4.2 Display box
| Setting | Meaning |
|---|---|
| Window | Time span shown (s). |
| Loop rate | Control-loop rate (25 000 Hz); sets the time axis. |
| Cycles/frame | Control cycles per telemetry frame (10). |
| Refresh | Redraw rate. Data is never lost at a low rate; it only looks less smooth. |
| **Pause** | Freezes the plots (recording continues). |
| **Record…** | Writes every raw frame to a `.bin` file until **Stop rec**. |

### 4.3 Loading a recording
```python
import numpy as np
dt = np.dtype([("sync", "u1", 2), ("seq", "<u2"), ("v", "<f4", 16), ("sum", "<u4")])
frames = np.fromfile("run1.bin", dtype=dt)
v = frames["v"]            # v[:, 5] = id, v[:, 6] = iq, v[:, 7] = rpm, v[:, 9] = position ...
t = np.arange(len(v)) / 2500.0
```
The 16 values per frame are: 0 angle, 1 bus voltage, 2 HFI − encoder (Hall − encoder with `angsrc` 3, HFI − Hall with 4), 3 HFI rpm, 4 HFI step, 5 id, 6 iq, 7 rpm, 8 rpm reference, 9 position, 10 position reference, 11 observer − encoder, 12 \|flux\|/λ, 13 blend weight, 14 observer rpm, 15 encoder rpm (Hall rpm with `angsrc` 3 and 4).

---

## 5. Config tab

Three areas: the parameter grid (left), the calibration panel and the log (right).

### 5.1 Parameter grid
Parameters are grouped (Motor, Current loop, Speed / position, Limits, Current sensing, Observer, HF injection, CAN). Columns: name, value, unit, the value on the device, and the short key. Hover a row for its help text, object address, allowed range, and whether it can only be changed while stopped.

Editing works in two steps:
1. **Double-click** a value and type the new one. The GUI checks it against the range. Objects with named settings (angle source, encoder direction, observer blend, HFI mode, CAN bit rate) show the name and open a drop-down instead; the number is in the tooltip and is what the JSON export holds. A pending edit is shown bold, with a yellow background and `*` after the name.
2. **Write changed** sends all pending edits to the device's RAM and reads them back. A refused write turns red; the reason is in the log.

| Button | Action |
|---|---|
| **Read all** | Reads every parameter from the device. |
| **Write changed** | Sends pending edits (RAM only). |
| **Revert** | Drops pending edits. |
| **Save to flash** | Stores what the device holds (not pending edits) in flash, plus the CANopen settings. Refused while the motor runs. The orange line "device RAM differs from flash" means there is something unsaved. |
| **Export JSON…** | Reads everything and saves it with a timestamp. Use it as a backup of a calibrated motor. |
| **Import JSON…** | Loads a file as pending edits (nothing is sent yet). Then **Write changed** and **Save to flash**. This is how a calibration is restored after a firmware update that reset the parameters. A value whose unit has changed since the export (e.g. `olo`/`ohi`, V before `PARAM_VERSION` 9, rpm now) is skipped and listed in the log: set it by hand. |
| **Reboot** | Resets the driver (unsaved RAM edits are lost). The port closes and the plotter reopens it when the driver is back (it retries for 10 s). |
| **Reboot to DFU** | Resets into the ST USB bootloader for flashing with STM32CubeProgrammer. |

Reboot and DFU are refused while the motor runs.

### 5.2 Calibration panel
One row per calibration step, with its state (**done** / **missing** / **running…**) and a **Run** button. What each step does, and the order, are in the driver manual, section 5.

| Key | Step |
|---|---|
| c | Current-sense offsets (rotor still) |
| e | Encoder: direction, pole pairs, offset (rotor turns) |
| H | Hall table (rotor turns; set `pp` first without an encoder) |
| r | Resistance |
| l | Inductance d |
| q | Inductance q (optional, hold the shaft) |
| t | Current PI autotune |
| k | kv (motor spins freely) |
| z | Reset encoder zero (EEPROM) |
| h | HFI saliency map |
| p | HFI lock + polarity test |
| u | Encoder UVW outputs as Hall sensors (bench; pole-pairs field) |

- Steps that move the motor or need the shaft held ask for confirmation first.
- **u** has a pole-pairs field: "default" (0) uses `pp`.
- **l**, **q** and **h** have a current field. "default" (0) measures at the standard current and stores the result; an explicit current measures that point only and stores nothing (for an L-vs-current curve or a saliency check at one bias).
- During a step, a countdown shows in large red text when the driver is about to apply torque or spin ("HOLD THE SHAFT: q test starts in …", "motor spins in …").
- **Abort** (or STOP) ends a running step. The motor is switched off at the end of every step, successful or not.
- When a step ends, the parameters are read again, so the grid shows the new values. Remember to **Save to flash**.

### 5.3 Log
Every command, its result, and every event from the driver, with measurement results (R and its two points, L and time constant, the L curve, encoder spread, autotune gains, kv / flux / Kt, HFI fits, current offsets). Errors are red, warnings orange. **Clear** empties it.

---

## 6. CANopen page

A CANopen master for testing the drive over its CAN port. It uses its own CAN adapter and can run at the same time as the USB link (keep USB connected: its STOP is the independent stop and E-STOP sends it too, and the USB page's Statistics box shows the drive side).

**Safety:** the tab never enables the drive by itself. The mode combo only sends when you pick a mode. Closing the window sends disable voltage first, then stops the PC heartbeat.

### 6.1 CAN bus box
| Control | Meaning |
|---|---|
| Adapter | python-can interface (slcan, gs_usb, pcan, …) and channel (COM port for SLCAN). |
| Bit rate, node | Must match the drive (default 1000 kbit/s, node 1). |
| **Connect** | Opens the adapter. EMCYs from the first 0.5 s are tagged "at connect": the drive was error-passive while nothing acknowledged its frames, which is harmless. |
| Drive / Bus | The drive's heartbeat and NMT state; frames per second and bus load. |
| NMT | Start, Pre-op, Stop, Reset (node), Reset comm. |
| PC heartbeat | The PC sends a heartbeat at this node ID and period. **pause** simulates a lost master. |
| **Watch PC heartbeat** | Writes 0x1016:1 so the drive faults (0x8130) if the PC heartbeat stops for the given time. |
| SYNC producer | Sends SYNC at the given period. TPDO2/3 (position, speed, torque, current, bus) are sent on SYNC, so this feeds the plots. 5–10 ms is realistic over SLCAN on Windows; the measured period is shown next to it. |

### 6.2 CiA 402 drive box
- The state in large text (green when operation enabled, red on fault), the statusword in hex, and its bits decoded for the present mode.
- The fault code (603F) with its text, and the reason for a refused switch-on (0x4042:2).
- Controlword buttons: **Shutdown 06**, **Switch on 07**, **Enable 0F**, **Disable op 07**, **Quick stop 02**, **Disable voltage 00**, **Fault reset**, and **Halt** (bit 8, held until released).
- The mode combo (0x6060) and the mode display (6061). **Enable in mode** writes the mode and walks 06 → 07 → 0F, checking the state after each step.
- **E-STOP (Esc)**: disable voltage over RPDO1 and SDO, and STOP over USB if connected. The Esc key works anywhere in the tab.

### 6.3 Mode box
Changes with the selected mode:

| Mode | Controls |
|---|---|
| PP | Target (turns), absolute or relative, change immediately (bit 5), profile velocity / acceleration / deceleration. **Write profile**, then **Go (bit 4)**. The handshake state is shown. |
| PV | Target rpm, acceleration, deceleration. **Set** (or Enter), **Zero**. |
| HM | Method 37 or 35 (home here), home offset (the position at home becomes −offset). **Start homing**. |
| CSP / CSV / CST | The set-point streamer (below). |

**Set-point streamer** (CSP in turns, CSV in rpm, CST in ‰ of rated current):
- Generator: sine, step (single or square), ramp, chirp, trapezoid, or a CSV file of `t,value` rows.
- Period: RPDO + SYNC period (CSP also sets 0x60C2 to it). Duration (0 = until Stop).
- Guards: max |speed| and max |position excursion|; past either, the drive is disabled.
- "NMT start and enable in the mode first" does the bring-up for you.
- A histogram shows the PC's actual SYNC period, so jitter can be blamed on the right side.

### 6.4 Sub-tabs
| Tab | Content |
|---|---|
| **Plots** | Position (actual 6064, demand 6062, target sent), velocity (606C, 606B, sent), torque (6077, 6074, 6078, sent), and optionally following error (60F4), DC link (6079) and statusword bits. Data comes from TPDO2/3, so switch on the SYNC producer and NMT start. Pause, Record… (every TPDO sample and streamed set-point to `.npz` or `.csv`), Auto-scale. |
| **Object dictionary** | Every object from the EDS with its value. Filter by index or name, **Read selected**, **Read all**, double-click a value to write it. Bold = a writable value that differs from the EDS default. **Store 0x1010** / **Restore 0x1011**. |
| **PDO mapping** | View and edit each PDO's mapping (e.g. `6064:0 606C:0`, empty = disabled), transmission type and event timer. Editing needs pre-operational. **Store comm 0x1010:2** keeps it over a reset. Presets map extra signals on the spare TPDO4 for the plots; **Revert TPDO4** disables it again. |
| **Trace** | Raw frames, decoded (NMT, SYNC, EMCY, PDOs, SDO, heartbeat). ID filter (e.g. `0x181, 0x580-0x5FF, !0x80`), option to hide SYNC / TPDO2/3 / PC heartbeat, Pause, Clear, **Save…** to `.csv`, `.asc`, `.blf` or `.log`. |
| **EMCY** | Time, code, text, info and error register for each emergency message. |
| **Tests** | The test sequences (section 7): tick them, **Run checked** or **Run all**, **Stop**. "allow motion (shaft free)" must be ticked for the tests that spin the motor. Each check shows pass/fail; the result file path is shown at the end. |

---

## 7. CANopen test runner (`cotest.py`)

The same tests as the Tests tab, without the GUI:

```
python software/cotest.py --can slcan:COM8 --usb COM7            all tests
python software/cotest.py --can slcan:COM8 csv pp                 only these
python software/cotest.py --list                                  list the tests
python software/cotest.py --can slcan:COM8 --no-motion            skip the ones that spin the motor
```

| Option | Default | Meaning |
|---|---|---|
| `--can interface:channel` | `slcan:COM8` | e.g. `pcan:PCAN_USBBUS1`, `socketcan:can0` |
| `--bitrate` | 1000000 | |
| `--node` | 1 | |
| `--usb PORT` | — | The driver's config link. With it, tests also check the firmware side (loop time, encoder late counter, USB refusals) and send STOP over USB at the end of each test. Some tests need it. |
| `--usb-baud` | 2000000 | UART builds only. |
| `--no-motion` | off | Skip motion tests. |

Tests: `identity`, `sdo_errors`, `node_id_bitrate` (USB), `store_restore`, `fsa_walk`, `refusal` (USB), `timing`, and the motion tests `csv`, `stops_at_speed`, `local_stop` (USB), `master_loss`, `cst`, `csp`, `following_error`, `pv`, `hm`, `pp`.

Every test ends with disable voltage, the mode restored, NMT pre-operational, and any changed object restored. Results (JSON + CSV per run) go to `software/cotest_runs/`.

Close the GUI's CAN connection before running `cotest.py`: an SLCAN adapter can only be opened by one program.

---

## 8. Headless command line (`drivectl.py`)

The USB commands of the Config tab and the basic CANopen master commands, without the GUI. For scripts, a remote bench, or a quick check from a terminal.

```
python software/drivectl.py --usb COM7 status                                 one command
python software/drivectl.py --usb COM7 "set vlim 3000; vel 300; sleep 5; stop" several, separated by ';'
python software/drivectl.py --usb auto --can slcan:COM8 -f bench.txt          a script, one command per line
python software/drivectl.py --usb COM7                                        interactive shell
python software/drivectl.py help                                              the command list
```

| Option | Default | Meaning |
|---|---|---|
| `--usb PORT` | — | The config link. `auto` tries each port with the driver's USB ids (1209:0001) and keeps the one that answers PING; the `--can` port is skipped. |
| `--usb-baud` | 2000000 | UART builds only. |
| `--can interface:channel` | — | e.g. `slcan:COM8`, `pcan:PCAN_USBBUS1`, `socketcan:can0`. The PC heartbeat (ID 0x7F) runs while the tool is connected. |
| `--bitrate`, `--node` | 1000000, 1 | |
| `-f FILE` | — | Run a script. `#` starts a comment; each command is echoed with `>`. |
| `-k` | off | Scripts: go on after a failed command (otherwise the script stops there). |

USB commands:

| Command | What it does |
|---|---|
| `ping` | Protocol version, object table hash, PARAM_VERSION, build id; warns if the table differs from `objects.py`. |
| `get KEY…` | Reads objects by key (`vlim`) or address (`0x4021:4`). `all`, `params`, `status`, `diag`, `info` read a whole group. |
| `set KEY VALUE …` or `set KEY=VALUE …` | Writes to RAM, in the Config tab's display units (e.g. `hsal` in %), choices by name or number; the limits are checked first. Prints the value read back. |
| `status` | The status objects. |
| `save` | Parameters and CANopen record to flash (motor off). |
| `cal WHICH [ARG] [--timeout S]` | A calibration, with the Config tab's letters (`c e H r l q t k z u h p`) or names (`offsets encoder hall r ld lq tune kv zero uvw hfimap hfipol`). Prints the events and waits for the end; Ctrl-C sends ABORT. There are no safety prompts: read section 5.2 before running `e`, `H`, `q`, `k`, `z`, `u`, `h`, `p`. |
| `cur ID IQ`, `vel RPM`, `pos TURNS`, `stop`, `home`, `abort` | As in the run panel. |
| `reboot [dfu]` | Reset (the port comes back after 1–2 s and the tool reopens it), or into the ST bootloader. |
| `export FILE.json`, `import FILE.json` | The Config tab's JSON format. `import` writes the values to RAM; `save` afterwards stores them. Values whose unit has changed since the export are skipped and listed. |
| `telem [SECONDS] [FILE.csv]` | Telemetry: to a CSV (time from the frame counter, then `seq` and the 16 channels), or 10 lines/s of bus voltage, currents, speed and position. 0 or no time: until Ctrl-C. |
| `watch KEY… [-p PERIOD]` | Prints the objects every PERIOD s (0.5) until Ctrl-C. |
| `events [SECONDS]`, `sleep SECONDS` | Wait, printing events (and EMCYs with `--can`). |

CANopen commands (`help co`), with raw OD values (increments, increments/s), except where a unit is named:

| Command | What it does |
|---|---|
| `co read OBJ…`, `co write OBJ VALUE` | SDO. OBJ is `0x6041`, `0x608F:1`, a native key (`vlim`) or an EDS name (`Statusword`). |
| `co nmt [op\|preop\|stop\|reset\|comm]` | NMT command; without one, the state from the heartbeat. |
| `co state` | Statusword and FSA state, mode, position (turns), speed (rpm), fault and last refusal. |
| `co enable [pp\|pv\|hm\|csp\|csv\|cst]`, `co disable`, `co shutdown`, `co qstop`, `co fault-reset`, `co halt on\|off` | The CiA 402 walk and controlword commands. `co disable` waits until CAN has let go of the motor. |
| `co pv RPM` | Profile velocity (enables PV first if needed). |
| `co pp TURNS [rel] [--timeout S]` | Profile position move; waits for target reached. |
| `co home [METHOD] [OFFSET_TURNS]` | Homing (method 37: here). |
| `co store [all\|comm\|app]`, `co emcy` | 0x1010 store; the EMCY log. |

The cyclic modes (CSP, CSV, CST) need set-points every SYNC: use the CANopen tab's stream or `co_link.Streamer` from a script.

Safety: **Ctrl-C** during `sleep`, `cal` or a waiting CANopen command sends STOP over USB and disable voltage over CAN; during `telem`, `watch` and `events` it only ends that command. When the tool exits, the motor keeps doing what it was last told (there is no USB watchdog): end scripts with `stop` or `co disable`. The exit code is 0 when every command succeeded, 1 otherwise, 2 when a link could not be opened.

Like `cotest.py`, it cannot share a port with the GUI.

---

## 9. Helper scripts (developers)

- `python software/check_sync.py`: compares the GUI's object table, message/status/event codes and telemetry layout with the firmware sources and the EDS. Run it after changing either side; it also notes when `driverTCC.xdd` is newer than the exported OD files.
- `python software/flash_report.py`: flash use per object file of a build.

---

## 10. Troubleshooting

| Symptom | What to check |
|---|---|
| No **[driver USB]** port | USB cable (data, not charge-only); the driver may be in the ST bootloader (option bytes, driver manual section 4.1) or in DFU after **Reboot to DFU** (power cycle). |
| "no reply to PING" | Wrong port (e.g. the CAN adapter or the ST-Link UART), or a firmware without the binary protocol. |
| Orange "object table differs" | GUI and firmware come from different versions. Update one of them; mismatched rows are disabled. |
| Dropped frames counting up | The PC is not reading fast enough: lower the plot refresh rate or close other heavy programs. A few drops when commands are sent are normal. |
| "connection lost" | The cable was pulled, or the driver reset (Reboot, or a CANopen reset node, e.g. the `identity` test). The plotter retries the same port for 10 s (the button reads **Cancel** meanwhile); if it does not come back, reconnect by hand. |
| A typed value becomes 10× larger | It should not: inputs use `.` decimals. Use `.` not `,`. |
| CANopen tab: replies late or SYNC uneven | Use a 5–10 ms period over SLCAN on Windows; a gs_usb adapter on Linux times better. About one SDO reply in a few hundred is lost by SLCAN on the PC side; the tool retries. |
| CANopen tab: no plots | Switch on the SYNC producer and NMT start (TPDO2/3 are sent on SYNC in operational). |
| Enable refused, warning bit set | The reason is shown under the state (e.g. speed gains missing, USB has the motor). |
| `cotest.py` / `drivectl.py` cannot open a port ("Access is denied") | The GUI (or another script) still has it open. |
