# CANopen PC interface handoff

**Goal:** a PC tool for testing the drive over CAN in depth, beyond what one-off scripts give. It should cover:
- **Interactive control:** drive the CiA 402 state machine and every mode by hand, and see what the drive reports.
- **Plots:** live plots of the cyclic data (TPDOs), including streamed CSP/CSV/CST set-points.
- **The bus:** inspect it (frames, NMT, heartbeats, EMCYs, errors) and the object dictionary (SDO browser, PDO mapping, store/restore).
- **Repeatable test sequences:** with pass/fail, runnable from the GUI and headless from the command line.

The drive side is done and documented in `docs/canopen-handoff.md`: units, PDO map, statusword meanings, fault codes, known limits. Read it first. This document only covers the PC side.

**Status, 2026-10-17:** all seven steps implemented; the decisions below were confirmed (third tab).
- **Done and run on the bench:** `co_link.py`, `cotest.py` with the 17 seed tests (all passed in one run with `--usb COM6`), and the CANopen tab. The tab was driven by script on the bench (connect, NMT, SYNC, CSV stream, PV/halt, PP, HM, E-STOP, OD read-all, TPDO4 preset and revert, record to .npz, trace to .asc, two tests from the Tests list). Clicking through it by hand has not been done yet.
- **Open:** after that run the motor started to stick-slip going negative in PV (0 rpm at the rated current, then breaking loose at −450 rpm). It looks mechanical or a commutation offset, not CAN. Check the bench, then rerun the motion tests.

What the bench showed, beyond the plan:
- **python-can 4.6 SLCAN on Windows** reads the port with one `read(1)` per byte, and flushes after every frame (pyserial's Win32 `flush()` sleeps 50 ms while the driver still holds TX bytes). In the GUI, replies came 50–300 ms late and SYNC had 50–100 ms gaps. `co_link._fast_slcan()` replaces both on the bus instance: replies 2–4 ms, SYNC 5.00 ms median.
- **Lost frames are requests:** the trace showed a controlword write that never reached the drive (read-back = old value). `DriveNode.write(retry=False)` reads back and resends only when the old value is still there.
- **GIL:** the GUI thread starved the SYNC thread (20 ms late every cycle). The busy-wait yields (`sleep(0)`), the tab sets `sys.setswitchinterval(0.0005)`, and the hidden Scope plot no longer redraws.
- **Stale 0x607A in CSP (fixed in firmware):** the drive used to step toward a 607A left over from the last session at CSP start (300 rpm jumps). Now only a 607A written after the mode starts moves it. `goto_enabled()` still writes 607A = 6064 first, as a master should.
- **CSP clicks (fixed in firmware):** the drive takes a new 607A on its 1 kHz tick, so targets arrived 4-6 ms apart for a 5 ms stream. Late ones stopped the reference and dropped the feed-forward (~1 ms dips of the full speed reference, 0.4 A current spikes). The ISR now moves at the master's rate whenever the frame arrives, rides through a late one for up to one period, and corrects half its offset per period; an unchanged target is a hold that ends exactly on it. Measured on the CSP sine: current spikes 26 -> 0, speed-reference dips 74 -> 0. The same change fixed 500 Hz feed-forward dips in PP.
- **Heartbeat loss sends the drive to pre-operational** (`NMT_CONTROL` has no `ERR_TO_STOPPED`), so TPDOs stop; the tests time the fault from the EMCY (0.23 s with 0x1016 = 300 ms).
- **PP "target reached"** can come on while 6064 passes through the 0.35° window during the slow `pkp` settling. The test checks demand 6062 on target and 6064 inside 6067 at the moment bit 10 sets, and notes the settled error (≤ 0.12°).

## Decisions (proposed; confirm before starting)
| Topic | Proposal | Why / alternative |
|---|---|---|
| Where | A **third tab, "CANopen"**, in `software/plotter.py`, next to Scope and Config | One window: the USB link (telemetry, STOP, diagnostics) and CAN run side by side, which is how every test so far was done. Alternative: a separate `canopen_gui.py` app sharing the core module. |
| Core / GUI split | A Qt-free core `software/co_link.py`; the Qt tab is `software/canopen_tab.py`; tests in `software/cotests/` | The same core drives the GUI, the test runner and ad-hoc scripts. |
| Stack | `python-can` (bus) + `canopen` (SDO, NMT, PDO, EDS parsing) | Already used and proven against the drive. Don't use `canopen.BaseNode402`'s state machine: it hides the controlword timing and the refusals we want to see. Drive 0x6040 directly. |
| OD source | `software/driverTCC.eds` via `canopen.import_od()` | The EDS is exported from the same XDD as the firmware. Never keep a second hand-written table of CAN objects. |
| Adapter | WeAct USB2CANISO, `interface="slcan"`, COM port, 1 Mbit/s | Make interface/channel/bitrate configurable (python-can also supports `gs_usb`, `pcan`, `kvaser`, `socketcan`). |

## Bench facts the design must handle
- **SLCAN timing on Windows:** python-can goes through a USB serial port. Measured: SDO replies come back in 2–3 ms, and **about one in a few hundred replies is lost on the PC side**; the drive's counters show nothing lost.
  - **Retries:** every SDO goes through a retry wrapper (1 s timeout, 2 retries, count the retries).
  - **Writes are not always idempotent:** don't blindly retry a 0x6040 edge or 0x4040:5.
- **Software timing:** PC-side cyclic sending (SYNC, RPDO streams) is timed in software. A 5 ms period with a busy-wait was clean; 1 ms is not realistic over SLCAN on Windows.
  - Default cycle 5–10 ms. Set 0x60C2 (CSP interpolation period) to the same value.
  - Show the measured jitter of the PC's own transmissions (histogram), so a bad trace can be blamed on the right side.
  - For 1 kHz work: a `gs_usb`/candleLight adapter on Linux, or the ROS setup.
- **EMCYs at connect:** while no adapter is open, nothing on the bench acknowledges the drive's frames, so it goes error passive. On connect it reports EMCY 0x8120 (info 0x06/0x07), 0x8110 (info 0x14) and their resets. Show them, but tag them as "at connect" (first ~0.5 s) so nobody chases them.
- **The drive needs NMT operational for PDOs.** SDO works in pre-operational.
- **Heartbeat consumer:** the drive only notices a lost master through its heartbeat consumer (0x1016). The tool should **produce a heartbeat** (configurable node ID, default 0x7F, 100 ms) and offer to set 0x1016:1 to watch it (e.g. 300 ms). Then closing the tool or pulling the adapter faults the drive (0x8130, master-loss reaction 0x6007 = 1). SYNC and RPDO timeouts do **not** stop the drive.
- **Ownership:** while CAN has the motor, USB RUN/CAL/HOME/SAVE/REBOOT are refused with `ST_CAN_ACTIVE`, but **USB STOP always works** (the CAN side faults with 0xFF01). The existing Run panel's STOP stays the hardware-independent stop.

## Core module `software/co_link.py` (no Qt imports)
- **`CoBus`:** connect/disconnect (interface, channel, bitrate), a `canopen.Network`, and a raw frame tap (`network.notifier` listener) feeding a bounded ring of `(t, id, data, dir)`.
  - **Counters:** frames/s and bus load (estimate from frame lengths), error frames if the interface reports them.
  - **PC heartbeat producer:** on/off, ID and period, plus a "stop heartbeat" switch to simulate master loss.
- **`DriveNode`:** wraps `network.add_node(id, eds)`.
  - **SDO:** `read(idx, sub)` / `write(idx, sub, value)` with retry, typed from the EDS, abort codes turned into text (a table of CiA 301 abort codes, e.g. 0x06090030 "value range exceeded", 0x08000022 "not in this state").
  - **Native objects:** `read_native(key)` using the names in `objects.py` for 0x4000-range objects.
  - **NMT:** start / stop / pre-op / reset node / reset comm.
  - **Heartbeat watch:** the drive's heartbeat with a timeout flag, and its NMT state.
  - **EMCY log:** decoded to `(t, code, register, info bits, text)`. Texts from the fault table in `docs/canopen-handoff.md` plus the CANopenNode info codes (`CO_EM_*` in `CANopen/CANopenNode/301/CO_Emergency.h`).
  - **PDOs:**
    - `pdo_read()` uses canopen's `node.tpdo.read()` / `rpdo.read()` to get the live mapping.
    - TPDO callbacks push decoded samples into a ring `(t, {name: value})`, for the plots.
    - RPDO send helpers: `send_rpdo(n, values)`, and `sync()`.
  - **Store/restore:** 0x1010:1–3 (`"save"` signature) and 0x1011:1–3 (`"load"` signature). Refused while the motor runs; show the abort.
- **`Drive402`:** a thin CiA 402 layer on `DriveNode`. No hidden state machine:
  - **Controlword commands:** `shutdown()`, `switch_on()`, `enable()`, `disable_op()`, `quick_stop()`, `disable_voltage()`, `fault_reset()` (0→1 edge on bit 7), `halt(on)`.
  - **`decode_sw(sw)`:** the FSA state, plus bits 4/5/7/9/10/11/12/13 named **per mode** (PP: set-point ack; HM: attained/error; PV: speed = 0; CSx: follows).
  - **`goto_enabled(mode, timeout)`:** a helper that walks 0x06 → 0x07 → 0x0F and checks the statusword after each step. On a refusal it reports 0x4042:2 (`st_t` reason, text from `protocol.status_text`).
  - **PP:** `pp_move(target, relative, immediate)` does the bit 4/12 handshake properly (wait for ack, drop bit 4, wait for ack to clear) and `wait_reached(timeout)`.
  - **HM:** `home(method=37, offset)`.
  - **Units:** helpers between inc ↔ turn/deg, inc/s ↔ rpm, inc/s² ↔ rpm/s, ‰ ↔ A (needs 6075). The GUI shows engineering units and sends increments.
- **`Streamer` (thread):** for CSP/CSV/CST, sends set-points + SYNC every period.
  - **Timing:** `time.perf_counter()` busy-wait for the last ~2 ms; record the actual send times.
  - **Generators:** step, ramp, sine, chirp, trapezoid; a CSV file of `(t, value)`.
  - **Logging:** every sent set-point, for plotting against the TPDO response.
- **Thread rule:** canopen/python-can callbacks run on the notifier thread. The core only appends to locked ring buffers and sets flags. The GUI polls them on a `QTimer`, like the Scope tab does with the serial receiver. No Qt calls from those threads.

## GUI: "CANopen" tab (`software/canopen_tab.py`)
Left to right / top to bottom; it reuses the plotter's sidebar for the USB side.
1. **Bus box:** interface, channel (port list), bitrate, Connect.
   - **Status:** node ID, NMT buttons, the drive's heartbeat/NMT state, frames/s.
   - **PC heartbeat:** on/off, plus "Watch PC heartbeat" (writes 0x1016:1).
   - **Settings:** remembered in `QSettings` like the plotter's.
2. **Drive state panel:**
   - **Statusword:** big FSA state label; statusword hex plus decoded bits (mode-aware); 603F code with text; 0x4042 refusal text.
   - **Controlword:** one button per command, the halt toggle, the mode combo (0x6060) and the mode display (0x6061).
   - **E-STOP:** sends disable voltage over SDO, and STOP over USB if connected. Bound to a key.
3. **Mode panel** (changes with the mode):
   - **PP:** target (turn or deg), abs/rel, immediate, profile velocity/acc/dec, Go, and the handshake state shown.
   - **PV:** target rpm, acc/dec, set.
   - **HM:** method, offset, Start.
   - **CSP / CSV / CST:** the Streamer: generator type and parameters, period, Start/Stop. Guards: max |speed| and max |position excursion|; past them, disable voltage.
4. **Plots** (pyqtgraph, rolling window):
   - **Signals:** position (demand 6062 / actual 6064 / streamed target), velocity (606B / 606C), torque (6074 / 6077), following error (60F4), DC link (6079), statusword bits as digital traces.
   - **Data source:** TPDO2/TPDO3. Offer "map more" to remap TPDO3/4 for a test, with a revert.
   - **Record:** the same Record button pattern as Scope, into a CSV/NPZ with the sent set-points.
5. **OD browser:**
   - **Tree:** from the EDS (index, sub, name, type, access, value, unit/description from the EDS `Description`). Read / read all / write.
   - **Changed values:** marked against the EDS default.
   - **Store / restore** buttons (0x1010/0x1011).
   - **Mapping:** a PDO mapping view/editor through canopen's PDO module, with save.
6. **Trace:** raw frames with ID filter, decoded columns (NMT, SYNC, EMCY, TPDOx, RPDOx, SDO tx/rx, heartbeat by COB-ID), pause, and save to CSV (or `.asc`/`.blf` via python-can's loggers).
7. **EMCY log:** time, code, text, info, register; "at connect" tag; clear.
8. **Tests:** a list of the test sequences (below). Run one or all, live log, pass/fail per check, and the result file path.

Keep the plotter's conventions: C locale for numeric inputs (`.` decimals), no text formatting in firmware (everything is decoded on the PC), and `check_sync.py` still passing.

## Test sequences (`software/cotests/`)
Plain Python files with one function per test, using the core API and a small `check(name, cond, detail)` helper.
- **Runner:** `python software/cotest.py --can slcan:COM8 [--usb COM6] [names…]` runs headless; the GUI calls the same runner on a thread.
- **Results:** a JSON + CSV per run in `software/cotest_runs/` (git-ignored).
- **USB link:** optional. When it's given, tests also check the firmware side (diagnostics 0x4002 loop-time max and encoder `late`, USB refusals) and STOP over USB at the end of every test.
- **Every test:** ends with disable voltage + 6060 back to its start value + NMT pre-operational, in a `finally`, and restores any OD value it changed.

Seed cases, with what the firmware did on the bench (2026-10-17, 345 kv test motor, 1 Mbit/s):

| Test | Steps | Expected (measured) |
|---|---|---|
| boot / identity | reset node, wait for boot-up 0x700+ID | boot-up `00`; 0x1018 vendor 0, product 1, revision = `PROTO_VERSION<<16 \| PARAM_VERSION`; heartbeat at 0x1017 |
| SDO errors | write RO, out of range, wrong size; 6060 = 2; 6098 = 1 | correct abort codes (0x06010002, 0x06090030/31/32, 0x06070010…) |
| node ID / bit rate | via USB 0x4040:1/2 + apply (5) | node answers at the new ID / rate; 0x4040:3/4 show it; back again |
| store / restore | change a 402 value, 0x1010:3 "save", reset node; then 0x1011:3 "load", reset | survives the reset; then back to the EDS default. Refused while running |
| FSA walk | 0x06, 0x07, 0x0F, 0x07, 0x06, 0x00 | 0x0231 → 0x0233 → 0x0237 → back to 0x0250 (bit 4 voltage enabled with the bus up, bit 9 remote) |
| refusal | enable with the speed gains zeroed (USB), or with USB running the motor | stays switched on, warning bit 7, 0x4042:2 = the reason |
| CSV | ±60 rpm | follows; statusword bit 12 |
| disable op / quick stop at speed | 0x07 / 0x02 at 60 rpm | ramp at 6084 / 6085, coast, ends in SO / SOD |
| local STOP | USB STOP while CAN runs | fault 0xFF01, EMCY; fault reset works after the coast |
| master loss | PC heartbeat watched by 0x1016 (300 ms), then stop it | fault 0x8130 within ~0.3 s |
| CST | ±20 ‰ pulses, guard at 4 turn/s | current follows; speed guard |
| CSP | sine 0.5 turn, 0.5 Hz, 5 ms stream + SYNC | tracking within 3–5° at 94 rpm (one period of lag); loop max ≤ 18.3 µs, encoder late 0 |
| following error | 6065 = 200 inc, 6066 = 5 ms, ¼-turn CSP step | fault 0x8611, EMCY; reset OK |
| PV | 0 → +60 → −60 rpm at 240 rpm/s, halt, release | ramps ~0.5 s per 120 rpm; target reached with 606D = 10 rpm (flickers with the speed ripple); halt → bit 10 at standstill |
| HM | 6098 = 37, 607C = ¼ turn, bit 4 | 6064 = −¼ turn; statusword 0x1637 |
| PP | +1 turn abs, −0.5 rel, change immediately, buffered pair, halt mid-move | ends within 6067; buffered pair moves exactly 1.0 turn; halt holds, release continues |
| timing | SDO latency (n = 500), TPDO period with SYNC at 5 ms | latency 2–3 ms; reports lost replies |

## Implementation order
1. **`co_link.py` + `cotest.py`:**
   - Port the seed tests (FSA walk, CSV, SDO errors first).
   - Headless only.
   - It replaces the scattered scripts and pins down the API.
2. **Tab skeleton:** bus box, drive state panel with controlword buttons, E-STOP, EMCY log.
3. **Plots from TPDOs**, then the Streamer (CSP/CSV/CST) with the jitter readout.
4. **Mode panels** for PP/PV/HM.
5. **OD browser + store/restore**, then PDO mapping.
6. **Trace view and recording.**
7. **Tests list in the GUI.**

## Files to add / touch
- **New:** `software/co_link.py`, `software/canopen_tab.py`, `software/cotest.py`, `software/cotests/*.py`.
- **`software/requirements.txt`:** add `python-can>=4.6` and `canopen>=2.4`.
- **`software/plotter.py`:** add the tab. Its STOP and the Run panel stay as they are.
- **`.gitignore`:** `software/cotest_runs/`.
- **`CLAUDE.md`:** one line under Build/Plotter, and the keep-in-sync note: the tab reads everything from the EDS, so nothing new to sync.

## Risks / things to verify early
- **canopen 2.4 API details:** PDO `read()`/`save()`, TPDO callbacks, and the SDO block/segmented paths for strings (0x1008). Try them against the drive in step 1.
- **Shared adapter:** python-can's SLCAN can't share the COM port, so the CAN side has one owner. Close the tool before running scripts, or route scripts through the tool's runner.
- **GUI load:** at 5 ms SYNC with three TPDOs, ~600 frames/s reach the PC. Decode in the notifier thread into numpy rings, and redraw at the plotter's refresh rate, not per frame.
- **Safety:**
  - **Startup:** the tool must never enable the drive by itself on connect, and the 6060 combo must not send on load.
  - **Closing:** disable voltage first, then stop the heartbeat, so the drive isn't left enabled without a watched master.
