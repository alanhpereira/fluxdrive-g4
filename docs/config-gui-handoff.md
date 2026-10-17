# Binary config protocol + plotter Config tab: handoff

**Goal:** replace the text command console with a binary, table-driven protocol, and add a matching **Config** tab to `software/plotter.py`. Benefits:
- Commands and replies become checksummed frames like telemetry, so the two can share the link without `begin()`/`finish()` pausing telemetry.
- Floats travel as raw IEEE 754 values, so no text formatting or parsing in either direction.
- The GUI owns presentation: names, units, limits, help, and a calibration wizard.
- Removing the text console saves roughly **4–5 KB net** of flash (measured below).

**Status, 2026-10-12:** phases 1–3 are implemented: the text console is gone and the plotter's Config tab is the only interface. Hardware testing so far: run modes from the GUI, which found the RUN_CUR reply / EVENT type collision (EVENT is now `0x7F`, `PROTO_VERSION` 2). Still to verify on hardware: the phase 1 and 2 **Verify** steps below (save/power-cycle, every calibration against known values, Abort/STOP partway through, loop time). Phase 4 (USB) is next. Tested without hardware:
- the real `proto.c` + `objdict.c`, compiled for the PC against stub HAL headers and driven by `protocol.py`: framing, CRC, READ/WRITE/LIST/PING, every error path, and busy mode with READ, BUSY, ABORT, STOP and `x`;
- the plotter's Config tab against that build through a fake serial port: read, edit, write and read back, save, calibration done and aborted, run, and JSON export/import;
- `python software/check_sync.py`.

### What was built, and where it differs from the proposal below
- **Files:**
  - firmware: `Core/Src/proto.c` + `Core/Inc/proto.h` (RX ring, parser, TX, enums), `Core/Src/objdict.c` + `Core/Inc/objdict.h` (table), changes in `calib.c`;
  - PC: `software/protocol.py`, `software/objects.py`, `software/config_tab.py` (the Config tab, kept out of `plotter.py`), `software/check_sync.py`.
- **RX:** the 256-byte IT ring, as recommended. `proto_init()` also **enables the USART1 8-byte RX FIFO** at runtime: it is off in the `.ioc`, and without it, bytes arriving every 5 µs while `currentLoop()` runs (same IRQ priority) would overrun. A frame whose bytes stop for 50 ms is dropped. A good frame clears any partial text line, and text drops control and non-ASCII bytes.
- **Output:** commands report through `EV(code, floats…)` with the request's token (`s_tok` in `calib.c`) and never pause telemetry. `cal_*` and `run_*` return `st_t`; only `protocol.py` has the English text.
- **CAL:** the immediate reply is the ACK (or the reason it was refused). The result comes in an **`EV_DONE` event** (status, which) once the motor is off again. If that event is lost, the GUI notices `busy` dropping back to 0. CAL is refused while running (`ST_RUNNING`).
- **STOP during a calibration aborts it**, like ABORT and `x`. SAVE is refused unless the motor is off. A RUN_* that fails stops the motor, as the console does. RUN_* reject NaN/Inf.
- **Objects beyond the proposal:**
  - `0x4010:11` calibration `flags`, writable, so a JSON import restores a calibrated state;
  - `0x4001:9` unsaved changes (`params_dirty()` compares RAM with the active flash slot), `0x4001:10` save count;
  - `0x4002:9` protocol RX errors.
  - Diagnostic counters reset by writing 0 (min = max = 0).
  - Every writable object has min/max on the device.
- **Build id:** an FNV-1a hash of the flash image, computed at start-up (<1 ms), so any rebuild changes it.
- **Phase 3:** `say`/`fmt_v`/`parse_*`/help/`kTab`/`print_*`/`set_param`, the `begin()`/`finish()` telemetry pause (`g_telem_en`) and the plotter's command line/quick buttons are gone. Bytes outside frames are ignored. `f`'s break-flag and READY-pin output moved to `0x4001:11`/`:12`. `EV()` passes `const float[]` instead of `double` varargs, which removed every `__aeabi_f2d` call.
- **Flash (release `-Os`):** 62 116 B before → 66 456 B with both interfaces → **56 268 B** after phase 3 (−5.8 KB net, better than the −4 to −5 KB estimate). No tiny text console was kept (open decision below: answered "no").

**Shaped for what comes later:**
- **Transport:** **USB CDC** (phase 4), with the same frames as on USART1, which remains a build-time fallback.
- **Addressing:** everything goes through **index:subindex objects**, like a CANopen object dictionary. That keeps it compatible with `docs/can-epos4-handoff.md`: the same table can later back the CANopen OD.

## Why: what the text console costs today
Measured from `build/release/calib.o` (`-Os`, after the flash work in `docs/flash-optimization-handoff.md`):

| Part | Flash | With the binary protocol |
|---|---|---|
| Calibration/run logic (`cal_*`, `run_*`, `run_watchdog`, `chk`) | ~6 KB | Stays: it must run next to the ISR |
| Strings (help, messages, `print_params`/`print_fault` formats) | ~3.8 KB | Mostly gone (messages become event codes) |
| `say`/`fmt_v`, `parse_*`, formatting code in `print_params`/`print_fault` | ~2.5 KB | Gone |
| `kTab` with key strings | 0.4 KB | Becomes the object table (no strings) |
| New: framing, CRC, object table, dispatch | — | +1–1.5 KB |

Flash is not the main reason: the debug build has ~49 KB free. The main reasons are robustness and the GUI.

## Constraints from the current firmware
- **UART load:** telemetry is 72 B every 10 cycles = 2.5 kHz × 72 B = 180 kB/s on a 2 Mbaud link that carries 200 kB/s, so **~90 % used**. Telemetry is sent from the ISR only when `huart1.gState == READY` (`motorCtrl.c`, line ~491). Replies will take a telemetry slot now and then; a skipped frame shows up as a `seq` gap, which the plotter already counts as dropped. That is acceptable.
- **TX ownership:**
  - Today `say()` uses blocking `HAL_UART_Transmit` and pauses telemetry with `g_telem_en`.
  - For binary replies: main checks `gState == READY` and starts `HAL_UART_Transmit_DMA` with IRQs disabled (`__disable_irq()` around check + start). The ISR does its existing check. All IRQs are priority 0, so the two can't collide. No telemetry pause is needed.
- **RX:** `HAL_UART_Receive_IT` byte by byte into a **32-byte** ring (`calib.c`, `RXQ`). That is why the plotter's `Sender` writes one byte per ms. Binary frames need the whole frame accepted at full speed:
  - **Recommended:** raise `RXQ` to 256. Each byte IRQ is ~0.5 µs at priority 0; check that `f` still shows the loop max under ~19 µs.
  - **Alternative:** USART1 RX DMA in circular mode with idle-line detection. This needs a `.ioc` change (add a USART1_RX DMA request); follow the CubeMX rules in CLAUDE.md.
- **Calibrations block in main** (`HAL_Delay` loops) and check for abort in `chk()`, which pops raw bytes looking for `'x'`.
  - `chk()` must instead run the frame parser in a "busy" mode: ABORT works, READ is allowed (harmless), and everything else gets a BUSY reply.
  - That means the parser must be re-entrant from inside `chk()`, i.e. a plain function `proto_poll(int busy)`, not a second main loop.
- **No newlib text I/O** (CLAUDE.md gotcha): don't reintroduce `printf`/`strtof`.
- **The params layout is unaffected** (`PARAM_VERSION` 4). The object table points into `g_params`.

## Frame format
Telemetry frames stay exactly as they are (`A5 5A`, 72 bytes, rotate-add sum), so old recordings and `frame_ok()` keep working. Messages get their own sync:

```
A5 5C | type u8 | tok u8 | len u8 | payload[len] | crc u16
crc = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload, little-endian
len <= 64
```
- **`tok`:** chosen by the host and echoed in the reply, so replies are matched to requests. `0` is reserved for unsolicited device events.
- **Endianness:** all fields little-endian; floats are IEEE 754 binary32.
- **Receiver:** the plotter's `Receiver` already scans for `A5`. Extend it to recognize `5A` (fixed 72 bytes) and `5C` (length from the header). In phase 1, bytes outside frames are still text.
- **Example on the wire:** READ of 0x4010:7 with token 0x12 is `A5 5C 10 12 03 10 40 07 <crc lo> <crc hi>`.

### Message types (proposal)
| Type | Dir | Payload | Reply payload |
|---|---|---|---|
| `0x01` PING | → | — | proto ver u16, table hash u32, `PARAM_VERSION` u16, build id u32 |
| `0x10` READ | → | index u16, sub u8 | status u32, value (type-sized) |
| `0x11` WRITE | → | index u16, sub u8, value | status u32 |
| `0x12` LIST | → | first entry u16 | n u8, then n × (index u16, sub u8, type u8, flags u8) |
| `0x20` SAVE | → | — | status u32 (disables the bridge, saves, re-inits, as `w` does) |
| `0x30` CAL | → | which u8 (enc, R, Ld, Lq, autotune, kv, encoder-zero reset), arg f32 (test current for Ld/Lq; 0 = default/store) | status u32 at the end (an immediate ACK, then progress events) |
| `0x31` ABORT | → | — | status u32 |
| `0x40` RUN_CUR | → | id f32, iq f32 | status u32 |
| `0x41` RUN_VEL | → | rpm f32 | status u32 |
| `0x42` RUN_POS | → | turns as Q32.32 i64 (`pos_q32_t`) | status u32 |
| `0x43` STOP | → | — | status u32 (ramped stop, as `o`; keep the no-short-brake behavior) |
| `0x44` HOME | → | — | status u32 |
| `0x80 \| t` | ← | reply to type `t` (requests stay below `0x7F`) | — |
| `0x7F` EVENT (no reply bit: `0xC0` would collide with the RUN_CUR reply) | ← | code u16, n u8, n × f32 | — (tok = token of the CAL/RUN that caused it, or 0) |

- **Status codes:**
  - For object access, use the **CANopen SDO abort codes**, e.g. `0x06020000` object does not exist, `0x06090011` subindex does not exist, `0x06090030` value out of range, `0x06010002` read-only, `0x08000022` not allowed in the current device state. CANopen later reuses the same checks.
  - For commands, use one enum replacing today's error strings: aborted by user, bridge fault, bus voltage too low, overcurrent, prerequisites missing ('e'/'r'/'l'/'t'), voltage limit hit, current change too small, recording timeout, L/R too fast/slow, rotor did not follow, pole pairs not an integer, back-EMF wrong sign, too few samples, BUSY, …
  - Keep the enum in one header, and mirror it in `software/protocol.py` with the English text.
- **Events** replace the progress and result lines `say()` prints today:
  - "aligning", "sweeping", "hold the shaft, starting in 5 s" (the GUI shows a dialog with a countdown)
  - results: R with the two points and v_off; L, tau, I0; the secant-L curve, one event per point; encoder dir/pp/offset/spread; autotune gains; kv/λ/Kt/offset/samples/peak rpm
  - watchdog "STOPPED: overcurrent" plus id/iq

## The object table (single source on the device)
New files `Core/Src/objdict.c/h` (hand-written, outside CubeMX):
```c
typedef struct {
	uint16_t index;
	uint8_t sub;
	uint8_t type;  /* T_U8, T_I8, T_U16, T_U32, T_I32, T_F32, T_I64 */
	uint8_t flags; /* RO, STORED (in g_params), RUNTIME, needs pi_apply(), not while running */
	void* p;
	/* optional: float min/max, or a check callback */
} obj_t;
```
- **One entry per parameter.** WRITE applies the side effects `set_param()` has today (`pi_apply()` after gain changes), and refuses changes that are unsafe while `g_mode == CTRL_RUN` (e.g. `pole_pairs`, `enc_dir`, `elec_offset`). Today's `kTab` and `print_params()` are replaced by this table.
- **Table hash:** compute it over (index, sub, type, flags) at build time or at startup. The GUI compares it with its own copy and warns if the firmware and the GUI disagree.
- **New source files:** the STM32 for VSCode extension adds files under `Core/Src` to `STM32Make.make` when it regenerates it. When building from the command line only, add them to `C_SOURCES` by hand.

### Proposed indices: own "native" objects in 0x4000–0x4FFF, SI units as stored today
The CAN work will add EPOS4-compatible objects (maxon's 0x2000/0x3000 ranges, integer units such as mA, mΩ, rpm, increments) as **unit-converting views** onto the same variables. The GUI keeps using the native ones. Keep 0x4000+ clear of anything EPOS4 defines. Check against the XDD when it is in the repo (`docs/epos4/`).

| Index | Group | Subindices (1…) |
|---|---|---|
| 0x4000 | Device info (RO) | proto version, table hash, `PARAM_VERSION`, build id |
| 0x4001 | Status (RO, poll ~5 Hz) | `g_mode`, `g_loop`, `g_params.flags`, MOE, nFAULT, gate-driver status byte, `g_vin`, busy (which cal) |
| 0x4002 | Diagnostics | `g_loop_cyc`, `g_loop_cyc_max` (write 0 clears), encoder samples / CRC errors / late / stalls / status (`hMT6835`) |
| 0x4010 | Motor | kv, pp, dir, elec_offset, rd, rq, ld, lq, rated, imax |
| 0x4011 | Sensing | cscale, vscale (flag "not used by the control path yet", see CLAUDE.md) |
| 0x4020 | Current loop | kpd, kid, kpq, kiq, bw |
| 0x4021 | Speed/position | vkp, vki, pkp, vlim, acc |
| 0x4022 | Limits | vmin, vmax, tmax |
| 0x4030 | Observer | oblend, ogain, olo, ohi, oftol, oerr |

## PC side (`software/`)
- **`protocol.py`:** framing, CRC, the message-type and status-code enums, and a `Link` class.
  - **Requests:** the `Link` sends requests, matches replies by `tok` with a timeout (~200 ms, longer for SAVE because of the ~20 ms flash stall), and hands events to a callback.
  - **Telemetry and text:** the `Receiver` thread feeds it; telemetry keeps going to the ring buffer, and text (phase 1) to the console.
  - **Sending:** with the larger RX ring the `Sender` can write whole frames, so drop the 1-byte-per-ms pacing for binary messages and keep it only for text lines.
- **`objects.py`:** the PC copy of the table, one row per object:
  - **identity:** index, sub, type, the old `s` key name (so people who know `s vkp` find it)
  - **display:** label, unit, display scale (e.g. H shown as µH, turn/s as rpm), min/max, group, help text (move the comments from `params.h` here)
  - **behavior:** flags for "stored", "RO", "dangerous while running"

  On connect: PING, compare the table hash, and LIST to cross-check index/type.
- **UI:** wrap the current central widget in a `QTabWidget`: **Scope** (today's plots + console, unchanged) and **Config**. In the Config tab:
  - **Parameters:** a grouped tree/table with columns name, value (editable), unit, device value, and changed/unsaved markers. Buttons: Read all, Write changed, **Save to flash** (warn: the bridge is disabled briefly), and **Export/Import JSON**. Export/Import is the backup/restore across `PARAM_VERSION` bumps, which today wipe everything (no migration).
  - **Calibration wizard:** steps e → r → l (→ q) → t → k, each showing done/missing from the `PF_*` flags in 0x4001. A Run button per step with its argument (test current for Ld/Lq) and a live event log. The result fields are filled from events. Safety prompts ("unload the shaft", "hold the shaft") come up as modal dialogs. **Abort** is always visible.
  - **Run panel:** mode (current/speed/position), setpoint inputs, Home, and a big **STOP** that is always enabled. Show status from 0x4001 polling. Show the speed limit `vlim` next to the speed and position inputs.
  - **Diagnostics:** 0x4001/0x4002 polled — mode, MOE, gate-driver flags decoded (VCC_UVLO, THSD, VDS_P, RESET), encoder counters, loop time with max/clear. This replaces `f`.
- **Keep in sync (add to CLAUDE.md):** `objdict.c` ↔ `software/objects.py`; status/event enums ↔ `software/protocol.py`; the frame format ↔ `Receiver`.

## Phases
1. **Objects over binary, text still alive.**
   - Firmware: framing, `proto_poll()`, the object table, READ/WRITE/LIST/PING/SAVE, RX ring 256.
   - Plotter: `protocol.py`, `objects.py`, Config tab with the parameter grid and JSON export/import.
   - Text commands keep working for everything else.
   - **Verify:** read all, write a gain, save, power-cycle, read back. Telemetry `seq` drops stay small while the grid is open. `f` loop max is unchanged.
2. **Commands and events.**
   - CAL/RUN/STOP/HOME/ABORT plus events; `chk()` calls `proto_poll(busy=1)`.
   - Cal routines report through one function, e.g. `report(code, n, floats…)`, which in this phase both emits an event and `say()`s the text.
   - Plotter: wizard, Run panel, Diagnostics.
   - **Verify:** every calibration from the GUI matches the text-console result on the same motor; Abort works mid-calibration; the watchdog STOP event shows.
3. **Remove the text console.**
   - Delete `say`/`fmt_v`/`parse_*`/help/`kTab`/`print_*`, the `begin()`/`finish()` telemetry pause, and `CommandLine`/quick buttons in the plotter (or keep a raw hex console for debugging).
   - Measure flash (`python software/flash_report.py`); expect −4 to −5 KB net. Update CLAUDE.md's serial-protocol section.
4. **USB CDC transport** (implemented 2026-10-17, not yet run on hardware; `Core/Src/link.c`).
   - Same frames over CDC: `CDC_Receive_FS` feeds the same parser through `proto_rx_bytes()`. Telemetry and replies share a 2 KB TX ring, drained only from the USB IRQ (pended by the producers, chained from TX complete). Data flows while the host holds DTR. USART1 stays as the `PROTO_LINK_USB 0` fallback.
   - CRS trims HSI48 to the USB SOF (`usbd_conf.c`); the plotter picks the 0483:5740 port by itself.
   - **Verify:** enumeration, telemetry without `seq` gaps, every Config tab action, loop time max unchanged with USB traffic, port close/reopen and cable replug while running, and the UART build.
   - **Drop `USB_LP_IRQn` priority below the ADC** (it is 0 in the .ioc today, like everything else), so USB traffic can't delay `currentLoop()`. This breaks the "all IRQs at priority 0" assumption in CLAUDE.md, so make the data shared between the USB callback and main safe.
   - Telemetry can stay on USART1 or move to USB: USB FS bulk does ~1 MB/s, enough for all channels at full rate.

## Open decisions
- RX: a 256-byte IT ring (no `.ioc` change) or circular DMA. Start with the ring.
- Whether to keep a tiny text console (`?`, `p`) after phase 3 for bench use without the GUI. It costs ~1 KB.
- Whether the device stores min/max per object (≈8 B × ~40 entries) or only the GUI checks them. Device-side checks protect against other hosts (and the future CAN master): recommended for anything that can hurt the hardware (currents, limits, `vlim`).
