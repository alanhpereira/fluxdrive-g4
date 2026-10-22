# Software structure

This document shows how the firmware and the PC tools fit together, as a set of flowcharts. It covers the paths code runs on (boot, main loop, interrupts), what each module calls and where data crosses from one context to another. The design reasons and bench numbers are in `design-notes.md`; usage is in the two user manuals.

The diagrams are Mermaid. GitHub and VS Code (with a Mermaid preview extension) draw them.

Contents:
1. [System overview](#1-system-overview)
2. [Firmware modules and layers](#2-firmware-modules-and-layers)
3. [Boot sequence](#3-boot-sequence)
4. [Main loop](#4-main-loop)
5. [One PWM period (40 µs)](#5-one-pwm-period-40-s)
6. [currentLoop(), the FOC ISR body](#6-currentloop-the-foc-isr-body)
7. [Angle and speed estimation](#7-angle-and-speed-estimation)
8. [USB config protocol path](#8-usb-config-protocol-path)
9. [Telemetry path](#9-telemetry-path)
10. [CANopen path](#10-canopen-path)
11. [Stops, trips and faults](#11-stops-trips-and-faults)
12. [Parameters in flash](#12-parameters-in-flash)
13. [PC software](#13-pc-software)
14. [Interrupt priorities](#14-interrupt-priorities)

---

## 1. System overview

```mermaid
flowchart LR
    subgraph PC["PC"]
        PL["plotter.py<br/>(PyQt GUI)"]
        CT["cotest.py<br/>(CLI test runner)"]
        DC["drivectl.py<br/>(headless commands)"]
        CS["check_sync.py<br/>flash_report.py"]
    end

    subgraph DRV["Driver board (STSPIN32G4)"]
        subgraph FW["Firmware (STM32G431)"]
            LINK["link / proto<br/>binary frames"]
            CO["CANopen node<br/>CiA 301 / 402"]
            CTRL["Control core<br/>FOC, calibrations"]
        end
        GD["Gate driver<br/>(I2C3 0x47)"]
        BR["3-phase bridge<br/>TIM1 PWM"]
        SH["3 shunts<br/>OPAMP x16 → ADC1/2"]
        ENC["MT6835 encoder<br/>SPI1 + DMA"]
    end

    MOT(("Motor"))

    PL -- "USB CDC (COM7):<br/>config + telemetry" --> LINK
    PL -- "SLCAN adapter (COM8)" --> CO
    CT -- "SLCAN (+ USB)" --> CO
    CT -.-> LINK
    DC -- "USB" --> LINK
    DC -- "SLCAN" --> CO
    LINK <--> CTRL
    CO <--> CTRL
    CTRL --> BR
    CTRL <--> GD
    GD --> BR
    BR --> MOT
    MOT --> SH --> CTRL
    MOT --> ENC --> CTRL
```

Two ways in:
- **USB CDC**: the binary config protocol (READ/WRITE/LIST of the object table, commands, calibrations) and the 72-byte telemetry frames. USART1 is a build-time fallback (`PROTO_LINK_USB 0`).
- **CANopen on FDCAN1**: normal operation through CiA 402 (PP, PV, HM, CSP, CSV, CST). The same native objects 0x4000–0x40FF are reachable over SDO.

---

## 2. Firmware modules and layers

```mermaid
flowchart TB
    subgraph APP["Application (hand-written, Core/Src + CANopen/app)"]
        direction TB
        CALIB["calib.c<br/>commands, calibrations, run modes,<br/>watchdog, drive_* API, shared globals"]
        CO402["co402.c<br/>CiA 402 FSA + modes"]
        COAPP["co_app.c<br/>node, TIM7 tick, 0x4000 bridge"]
        COST["co_store.c<br/>CANopen record (pages 60-61)"]
        OBJ["objdict.c<br/>object table"]
        PAR["params.c<br/>g_params (pages 62-63)"]
        PROTO["proto.c<br/>RX ring, frame parser/sender"]
        LNK["link.c<br/>USB / UART byte pipe"]
        BOOT["boot.c<br/>reboot / DFU"]
    end

    subgraph CORE["Control core (ISR path, -O2, CCM SRAM)"]
        direction TB
        MC["motorCtrl.c<br/>currentLoop(), observer, PLL,<br/>outer loops, trips, telemetry"]
        HFI["hfi.c<br/>high-frequency injection"]
        CSN["curSnsCtrl.c<br/>ADC/OPAMP/DAC, cs_adc_irq()"]
        MT["mt6835.c<br/>encoder, burst DMA read"]
        BRG["bridgeCtrl.c<br/>PWM, duty, faults"]
        PI["pi_ctrl.h"]
        CRD["cordicMgr.h"]
    end

    subgraph LIB["Generated / third party"]
        HAL["CubeMX init + HAL<br/>(adc, tim, spi, ...)"]
        USB["USB_Device<br/>usbd_cdc_if.c hooks"]
        CON["CANopenNode stack<br/>+ port/CO_driver_stm32g4.c"]
        OD["od/OD.c, OD.h<br/>(exported from driverTCC.xdd)"]
    end

    CALIB --> MC
    CALIB --> HFI
    CALIB --> PAR
    CALIB --> PROTO
    CALIB --> BRG
    CALIB --> MT
    CO402 --> CALIB
    COAPP --> CO402
    COAPP --> OBJ
    COAPP --> COST
    COAPP --> CON
    CON --> OD
    PROTO --> OBJ
    PROTO --> CALIB
    PROTO --> LNK
    LNK --> USB
    OBJ --> PAR
    CSN --> MC
    MC --> HFI
    MC --> MT
    MC --> BRG
    MC --> PI
    MC --> CRD
    MC --> LNK
    BRG --> HAL
    CSN --> HAL
    MT --> HAL
```

Arrows mean "calls into or uses". `calib.c` owns the globals the ISR reads (`g_mode`, `g_loop`, references, PI states); the ISR never writes `g_mode`.

---

## 3. Boot sequence

```mermaid
flowchart TD
    RST(["Reset"]) --> BC{"boot_check():<br/>DFU magic in TAMP BKP0?"}
    BC -- yes --> SYS(["Clear magic, jump to<br/>ST bootloader 0x1FFF0000"])
    BC -- no --> CCM["Copy .ccmram section<br/>flash → CCM SRAM"]
    CCM --> HAL["HAL_Init()<br/>SystemClock_Config() 170 MHz<br/>flash prefetch on"]
    HAL --> MX["MX_*_Init()<br/>GPIO, DMA, ADC1/2, FDCAN1, I2C3,<br/>OPAMP1-3, TIM1, DAC1, SPI1, TIM4,<br/>USART1, CORDIC, TIM2, USB"]
    MX --> CI["cmd_init()"]
    CI --> PLD{"params_load():<br/>valid A/B slot,<br/>PARAM_VERSION matches?"}
    PLD -- yes --> PIA
    PLD -- no --> DEF["k_defaults<br/>(not calibrated)"] --> PIA
    PIA["pi_apply()<br/>ki → ki_ts"] --> IM

    subgraph IM["initMotor()"]
        direction TB
        I1["cordic_init(), DWT cycle counter"] --> I2["initCS(): OPAMP, ADC injected,<br/>DAC SC reference, ADC IER"]
        I2 --> I3["initBridge(): gate driver over I2C,<br/>TIM1 PWM, CC4 sample point"]
        I3 --> I4["MT6835_StartBurst()<br/>CS low, TIM1 update IRQ on"]
        I4 --> I5["cs_boot_offsets()<br/>64 ms current zero, fallback if spread"]
    end

    IM --> COI["co_app_init()<br/>co_store load, CANopenNode init,<br/>FDCAN1 IRQ, TIM7 1 kHz"]
    COI --> LOOP(["Main loop"])
```

From the end of `initBridge()`, the 25 kHz control interrupt runs. In `CTRL_OFF` it applies zero volts.

---

## 4. Main loop

Everything slow or blocking runs here: commands, calibrations, flash writes, CiA 402 state changes, communication resets.

```mermaid
flowchart TD
    W(["while (1)"]) --> CP

    subgraph CP["cmd_poll()"]
        direction TB
        TP["temp_poll()<br/>every 10 ms: read the last regular conversions,<br/>arm the next (g_temp_start),<br/>g_i_temp / g_i_vel, over-temperature latches"] --> RW
        RW["run_watchdog()<br/>trip_watchdog(), coast end,<br/>bridge/vbus/overspeed/angle-lost/<br/>temperature checks<br/>→ EV_STOPPED"] --> ER["enc_recover()<br/>resend burst cmd every 100 ms<br/>while motor off"]
        ER --> PP["proto_poll(0)<br/>parse frames →<br/>objdict / cmd_message()"]
        PP --> CAP["co_app_poll()<br/>NMT communication / node resets"]
        CAP --> C4P["co402_poll()<br/>CiA 402 FSA transitions"]
        C4P --> SV{"0x1010 store<br/>request pending<br/>and motor off?"}
        SV -- yes --> SR["save_records()"]
        SV -- no --> E0["done"]
        SR --> E0
    end

    CP --> CF["checkAndClearBridgeFault()<br/>read gate driver over I2C,<br/>re-arm MOE when safe"]
    CF --> GP["read DRV_RDY, nFAULT pins"]
    GP --> W
```

### Blocking commands

A calibration or a speed/position run with a ramp blocks inside `cmd_message()`. It keeps the protocol alive through `chk()`:

```mermaid
flowchart TD
    CM["cmd_message(CAL ...)"] --> ACK["proto_send ACK"]
    ACK --> STEP["set refs / g_mode / g_hfi_cmd<br/>for the next step"]
    STEP --> DLY["HAL_Delay(...)"]
    DLY --> CHK["chk(): proto_poll(1)"]
    CHK --> Q{"frame received?"}
    Q -- READ / PING / LIST --> SERVE["serve it"] --> MORE
    Q -- "ABORT / STOP" --> AB["abort: motor off<br/>→ EV_DONE (ST_ABORTED)"]
    Q -- other --> BUSY["reply ST_BUSY"] --> MORE
    Q -- none --> MORE{"more steps?"}
    MORE -- yes --> STEP
    MORE -- no --> RES["EV_* results (floats),<br/>store in g_params (RAM)"] --> DONE["EV_DONE"]
```

The CANopen tick (TIM7) keeps running while main blocks, so heartbeat and SDO keep working.

---

## 5. One PWM period (40 µs)

TIM1 counts up and down (center-aligned, ARR 3399). The order of events in one period:

```mermaid
flowchart LR
    B["TIM1 counter bottom<br/>t = 0"] --> UP["TIM1_UP_TIM16_IRQHandler<br/>(mt6835.c)<br/>start SPI1 DMA read of<br/>MT6835_GROUPS groups"]
    UP --> TS["temp_isr_start():<br/>ADSTART on ADC1 + ADC2 regular<br/>if g_temp_start (ends ≤ 6.1 µs)"]
    UP --> DMA["DMA clocks 6 groups<br/>≈ 18.1 µs (no CPU)"]
    DMA --> CC4["TIM1 CC4 ≈ 19.6 µs<br/>triggers injected ADC1 + ADC2"]
    CC4 --> ADC["ADC1_2_IRQHandler<br/>→ cs_adc_irq()<br/>at ADC2 JEOS"]
    ADC --> CL["currentLoop()<br/>≈ 2-2.5 µs after CC4"]
    CL --> CCR["new CCR1-3 values<br/>(SVPWM)"]
    CCR --> NB["must finish before<br/>next bottom"]
    NB --> B
```

`cs_adc_irq()` reads both ADCs (three phase currents and the bus voltage), subtracts the offsets, scales to amperes and volts, and calls `currentLoop()`.

---

## 6. currentLoop(), the FOC ISR body

```mermaid
flowchart TD
    S(["currentLoop()"]) --> ENC["MT6835_Sample()<br/>last complete group, CRC,<br/>angle + turns, sample age"]
    ENC --> FR{"fresh angle?"}
    FR -- yes --> LZ["g_enc_lost_n = 0"]
    FR -- no --> LI["g_enc_lost_n++"]
    LZ --> MU
    LI --> MU
    MU["motion_update()<br/>encoder PLL: speed;<br/>position from the encoder,<br/>or (ang_src 3/4) last cycle's Hall position"] --> CLK["clarke(ia, ib, ic) → iα, iβ"]
    CLK --> OBS["obs_update()<br/>flux observer (every mode)"]
    OBS --> HE["hfi_estimate()<br/>HFI angle + speed"]
    HE --> HU["hall_update() (flash)<br/>Hall state, interpolated angle + speed"]
    HU --> SRC{"ang_src"}
    SRC -- "0/1 encoder" --> BASE_E["base = encoder angle,<br/>advanced by speed × age"]
    SRC -- "2 sensorless" --> BASE_H["base = HFI angle"]
    SRC -- "3 Hall" --> BASE_L["base = Hall angle"]
    SRC -- "4 Hall + HFI" --> BASE_Y["guard: HFI outside the Hall sector<br/>for 2 ms → hfi_reseed()<br/>base = HFI → Hall by speed<br/>(hyb_lo..hyb_hi)"]
    BASE_L --> HP2["hall_pos_step()<br/>position += Δbase / pp"]
    BASE_Y --> HP2
    BASE_E --> OPLL
    BASE_H --> OPLL
    HP2 --> OPLL
    OPLL["obs_pll()<br/>observer speed"] --> WGT["obs_weight()<br/>blend weight w (0..1)<br/>lock / drop checks"]
    WGT --> VEL["s_vel = blend(base speed, observer speed, w)"]
    VEL --> MODE{"g_mode"}
    MODE -- CTRL_CAL --> TF["θ = g_theta_cmd (forced)"]
    MODE -- "RUN / OFF" --> TB["θm = base + w·(obs − base)<br/>θo = θm + output advance"]
    TF --> SC
    TB --> SC
    SC["sincos_q31(θm), sincos_q31(θo)<br/>(CORDIC)"] --> PARK["park() → id, iq"]
    PARK --> TRIP["trip_check(id, iq)<br/>overvoltage, overcurrent"]
    TRIP --> HS["hfi_step()<br/>injection + demodulation,<br/>hfi_filter(id, iq)"]
    HS --> RC["regen clamp factor rk<br/>(vbus near vbus_max)"]
    RC --> OL["outer_loops()<br/>every OUTER_DIV = 10 cycles:<br/>position P → speed PI → iq_ref<br/>(limit g_i_vel: rated, temperature cap)"]
    OL --> M2{"g_mode"}
    M2 -- CTRL_RUN --> PIC["pi_current_step()<br/>id, iq → vd, vq<br/>(voltage circle, d first)"]
    M2 -- CTRL_CAL --> VC["vd, vq = g_vd_cmd, g_vq_cmd"]
    M2 -- CTRL_OFF --> V0["vd = vq = 0, PI reset"]
    PIC --> IP
    VC --> IP
    V0 --> IP
    IP["ipark(θo) → vα, vβ<br/>+ HFI injection, limit |v| ≤ 1"] --> SVP["SVPWM() → CCR1-3"]
    SVP --> REC["step recorder<br/>(g_rec_state)"]
    REC --> TLM{"10th cycle and<br/>no outer-loop run?"}
    TLM -- yes --> ST["send_telemetry()<br/>fill Frame, link_defer()"]
    TLM -- no --> TM
    ST --> TM["loop time → g_loop_cyc_max"]
    TM --> E(["return"])
```

Notes:
- The outer loops and the telemetry fill never run in the same cycle, which keeps the worst-case loop time down.
- `trip_check()` can force a coast (id = iq = 0) or switch MOE off; see [section 11](#11-stops-trips-and-faults).
- Everything in this chart runs from CCM SRAM except `send_telemetry()` and `hall_update()` (CCM is full).

---

## 7. Angle and speed estimation

```mermaid
flowchart LR
    subgraph SRCS["Sources"]
        E["Encoder<br/>MT6835, 21 bit"]
        H["HFI<br/>(saliency, low speed)"]
        L["Hall sensors<br/>(J13, 6 states per electrical turn)"]
        O["Flux observer<br/>(back-EMF, speed)"]
    end

    E --> EP["encoder PLL<br/>motion_update()"]
    H --> HP["HFI PLL"]
    O --> OP["observer PLL<br/>obs_pll()"]
    L --> LI["hall_update()<br/>edge + speed × time, clamped to the sector"]

    EP --> SEL{"ang_src"}
    HP --> SEL
    SEL -- "0: encoder" --> BASE["base angle + speed"]
    SEL -- "1: encoder + obs" --> BASE
    SEL -- "2: sensorless" --> BASE
    LI --> SEL
    SEL -- "3: Hall" --> BASE
    SEL -- "4: Hall + HFI" --> BASE

    BASE --> BL["blend with weight w<br/>w = 0 below obs_lo rpm,<br/>1 at obs_hi rpm,<br/>only after lock"]
    OP --> BL
    BL --> TH["commutation angle θm, θo"]
    BL --> SV["s_vel → speed loop,<br/>g_vel, output advance"]
    EP --> POS["position: encoder (0-2),<br/>accumulated base angle (3, 4)<br/>→ position loop, g_pos_cnt"]
    BASE --> POS
```

- `ang_src` 0: encoder only (w is computed but forced to 0).
- `ang_src` 1: blends the observer in when `obs_blend` = 1.
- `ang_src` 2: HFI is the base, the observer always blends in above `obs_lo`; the encoder is only logged and position mode is refused.
- `ang_src` 3: the interpolated Hall angle is the base; the observer blends in with `obs_blend` = 1.
- `ang_src` 4: HFI, started on the Hall angle and kept in the Hall sector, crossfades to the Hall angle between `hyb_lo` and `hyb_hi` (the injection pauses above); the observer blends in with `obs_blend` = 1. In 3 and 4 the position is the base angle accumulated.

---

## 8. USB config protocol path

```mermaid
flowchart TD
    PC["PC: protocol.Link<br/>A5 5C | type | tok | len | payload | crc16"] --> USBH["USB CDC OUT endpoint"]
    USBH --> RX["CDC_Receive_FS()<br/>(usbd_cdc_if.c, USB IRQ prio 1)"]
    RX --> RB["proto_rx_bytes()<br/>256-byte RX ring"]
    RB --> PPO["proto_poll() in main:<br/>feed() byte-by-byte,<br/>check CRC, drop bytes outside frames"]
    PPO --> T{"type"}
    T -- "PING" --> PING["hash + build id"]
    T -- "READ / WRITE / LIST" --> OD["objdict.c<br/>k_tab: index:sub → variable,<br/>type, flags, min/max"]
    T -- "SAVE, CAL, RUN_CUR/VEL/POS,<br/>STOP, HOME, ABORT, REBOOT" --> CMD["cmd_message()<br/>(calib.c)"]
    OD --> REP
    PING --> REP
    CMD --> CHKS{"checks:<br/>g_can_owner? trip?<br/>PF_* flags? motor off?"}
    CHKS -- refused --> REP
    CHKS -- ok --> ACT["run mode / calibration /<br/>params_save / boot_reboot"]
    ACT --> REP["proto_send(type | 0x80, tok, st_t ...)"]
    ACT --> EV["proto_event(tok, EV_*, floats)<br/>(calibration progress, EV_DONE)"]
    REP --> TX["link_tx() → 2 KB TX ring<br/>→ pend USB_LP_IRQn"]
    EV --> TX
    TX --> PUMP["link_usb_pump()<br/>(USB IRQ only)"] --> PCR["PC"]
```

A WRITE takes effect in RAM. SAVE writes flash (both records). With a calibration blocking, the same parser runs with `busy = 1` (see [section 4](#4-main-loop)).

---

## 9. Telemetry path

```mermaid
flowchart LR
    CL["currentLoop()<br/>every 10th cycle (2.5 kHz)"] --> FILL["send_telemetry()<br/>16 floats into Frame"]
    FILL --> DEF["link_defer()<br/>pend link IRQ"]
    DEF --> LIRQ["link IRQ (prio 1)<br/>tlm_finish(): seq, frame_sum()"]
    LIRQ --> Q{"TX ring has room?<br/>DTR held?"}
    Q -- no --> DROP["g_tlm_drop++<br/>(0x4002:10)"]
    Q -- yes --> RING["TX ring → USB IN"]
    RING --> PCR["plotter Receiver:<br/>Splitter → telem_ok()<br/>→ numpy ring buffer → plots"]
```

The frame is `A5 5A`, `seq u16`, 16 floats, `sum u32` (72 bytes). The meaning of each `v[i]` is in `PLOTS` in `software/plotter.py`; `check_sync.py` checks it.

---

## 10. CANopen path

```mermaid
flowchart TD
    BUS["CAN bus 1 Mbit/s"] --> FD["FDCAN1_IT0_IRQHandler<br/>(port/CO_driver_stm32g4.c, prio 2)<br/>RX into CANopenNode buffers"]
    FD --> TICK

    subgraph TICK["TIM7_IRQHandler, 1 kHz (co_app.c, prio 2)"]
        direction TB
        P1["CO_process()<br/>NMT, heartbeat, SDO, EMCY"] --> R{"reset requested?"}
        R -- yes --> RQ["note reset, stop tick<br/>(main does it)"]
        R -- no --> P2["CO_process_SYNC()"]
        P2 --> P3["CO_process_RPDO()"]
        P3 --> P4["co402_tick()<br/>controlword / targets → refs<br/>(only while main armed it),<br/>actual values → TPDO data"]
        P4 --> P5["CO_process_TPDO()"]
    end

    P1 -- "SDO 0x4000-0x40FF" --> BR["co_app.c bridge<br/>→ objdict.c"]
    P1 -- "SDO 0x1010 / 0x1011" --> STQ["co_store request<br/>(deferred to main)"]
    P4 --> REFS["g_vel_ref, g_iq_ref,<br/>g_pos_ref_new + g_pos_ref_req"]
    REFS --> ISR["currentLoop()"]

    subgraph MAIN["main loop"]
        CAP["co_app_poll()<br/>communication / node reset"]
        C4["co402_poll()<br/>FSA: Switch on disabled → Ready<br/>→ Switched on → Operation enabled,<br/>mode changes, faults → 0x603F + EMCY"]
    end

    RQ --> CAP
    C4 --> DAPI["drive_check / drive_enter /<br/>drive_coast / drive_trip_reset<br/>(calib.c), g_can_owner"]
    DAPI --> ISR
```

While CANopen owns the drive (`g_can_owner`), USB RUN/CAL/HOME/SAVE/REBOOT are refused with `ST_CAN_ACTIVE`. USB STOP still works; the CANopen side then faults with 0xFF01.

---

## 11. Stops, trips and faults

```mermaid
flowchart TD
    subgraph ISRSIDE["ISR: trip_check() every cycle"]
        OV{"g_vin > vbus_max<br/>for 1 ms?"}
        OC{"|id| or |iq| > max_current<br/>for 3 cycles?"}
        OV -- yes --> TR
        OC -- yes --> TR
        TR{"g_mode == CTRL_RUN?"}
        TR -- yes --> COAST["coast: id = iq = 0,<br/>outer loops held"]
        TR -- no --> MOE["bridge_trip_off():<br/>MOE = 0, g_trip_moe"]
        COAST --> RISE{"current still rising?<br/>> 1.5·imax, or > imax after 5 ms"}
        RISE -- yes --> MOE
        COAST --> RES{"overvoltage and bus below<br/>vbus_max − vbus_resume for 10 ms?"}
        RES -- yes --> RESUME["restore refs, EV_RESUMED"]
    end

    subgraph MAINSIDE["main: run_watchdog()"]
        TW["trip_watchdog():<br/>|speed| < 30 rpm → CTRL_OFF,<br/>EV_STOPPED (ST_VBUS_HIGH / ST_OVERCURRENT)"]
        CHKW{"bridge fault, vbus low,<br/>overspeed, HFI lost,<br/>encoder lost, over temperature?"}
        CHKW -- "angle lost" --> OFF1["MOE = 0 (coasts)"]
        CHKW -- other --> CO["coast_or_off()"]
        CO --> SPD{"|speed| > COAST_END_TURNS<br/>(30 rpm)?"}
        SPD -- yes --> CST["stay CTRL_RUN at id = iq = 0<br/>(g_coasting), CTRL_OFF when slow"]
        SPD -- no --> OFF2["CTRL_OFF (zero vector)"]
        CHKW --> EVS["EV_STOPPED, g_stop_reason"]
    end

    COAST --> TW
    MOE --> TW

    subgraph REARM["main: checkAndClearBridgeFault()"]
        RA{"MOE off and<br/>not g_trip_moe?"}
        RA -- yes --> SL{"speed < 30 rpm<br/>(or SL_REARM_MS sensorless)?"}
        SL -- yes --> ARM["read/clear gate driver faults,<br/>MOE = 1"]
        SL -- no --> SPIN["BRIDGE_ERR_SPIN, wait"]
    end
```

The rule behind most of this chart: never apply the zero vector to a motor spinning above 30 rpm. That would short-brake it at about λ/L, far above `imax`.

---

## 12. Parameters in flash

```mermaid
flowchart LR
    subgraph FLASH["Flash, 128 KB"]
        CODE["code 0-120K"]
        P6061["pages 60-61<br/>CANopen record<br/>(co_store.c)"]
        P6263["pages 62-63<br/>motor params A/B<br/>(params.c)"]
    end

    RAM["g_params in RAM"]
    P6263 -- "boot: params_load()<br/>newest valid slot,<br/>CRC32 + PARAM_VERSION" --> RAM
    RAM -- "WRITE (USB) / SDO 0x40xx" --> RAM
    RAM -- "SAVE or 0x1010:3,<br/>motor off, bridge disabled<br/>→ params_save() (other slot)" --> P6263
    COREC["CANopen OD<br/>PERSIST_COMM / PERSIST_APP,<br/>node ID, bit rate"] -- "SAVE or 0x1010:2" --> P6061
    P6061 -- "boot (layout signature checked)" --> COREC
```

---

## 13. PC software

```mermaid
flowchart TD
    subgraph GUI["plotter.py (Qt main thread)"]
        MAINW["Main window<br/>plots (pyqtgraph), sidebar"]
        CFG["config_tab.py<br/>ConfigTab, RunPanel"]
        COT["canopen_tab.py<br/>CanopenTab"]
    end

    subgraph USBTH["USB side threads"]
        RCV["Receiver thread<br/>reads COM port"]
        SPL["protocol.Splitter<br/>telemetry vs A5 5C frames"]
        LNK["protocol.Link thread<br/>one request in flight,<br/>replies by token, events queue"]
        SND["Sender thread<br/>only writer of the port"]
    end

    subgraph CANTH["CAN side (co_link.py, no Qt)"]
        CB["CoBus<br/>python-can SLCAN (patched)"]
        HB["Heartbeat / SyncProducer threads"]
        DN["DriveNode / Drive402<br/>SDO, PDO, FSA"]
        STR["Streamer<br/>gen_step / ramp / sine /<br/>chirp / trapezoid / file"]
        WK["Worker thread"]
    end

    OBJ["objects.py<br/>labels, units, limits"]
    EDS["driverTCC.eds"]
    TESTS["cotest.py + cotests/<br/>basic.py, motion.py"]
    DCTL["drivectl.py<br/>(UsbSerial: own reader,<br/>Link and drain threads)"]

    RCV --> SPL
    SPL -- telemetry --> MAINW
    SPL -- messages --> LNK
    LNK -- "drain() results/events" --> CFG
    CFG --> LNK
    LNK --> SND
    CFG --> OBJ
    COT --> DN
    COT --> TESTS
    DN --> CB
    HB --> CB
    STR --> DN
    WK --> DN
    DN --> EDS
    TESTS --> DN
    TESTS -. "--usb" .-> LNK
    DCTL --> DN
    DCTL -. "--usb" .-> LNK
```

The same test runner works from the command line (`cotest.py`) and from the CANopen tab. `drivectl.py` sends the Config tab's USB commands and basic CANopen master commands without Qt, through the same `Link` (in `co_link.UsbSerial`, whose reader thread also hands telemetry frames to it) and `Drive402`. `check_sync.py` compares the PC tables (`objects.py`, `protocol.py`, `PLOTS`, the EDS) with the firmware sources.

---

## 14. Interrupt priorities

```mermaid
flowchart TB
    P0["Priority 0 (cannot preempt each other)<br/>ADC1_2 → cs_adc_irq → currentLoop<br/>TIM1_UP (encoder DMA start, temperature ADSTART)<br/>TIM1_BRK (nFAULT break)<br/>(EXTI0-2: disabled, the Halls are polled)"]
    P1["Priority 1<br/>USB_LP (or USART1): link RX/TX,<br/>deferred telemetry"]
    P2["Priority 2 (CO_IRQ_PRIO)<br/>FDCAN1_IT0, TIM7 1 kHz CANopen tick"]
    P15["Priority 15<br/>SysTick (HAL_GetTick)"]
    TH["Thread: main loop<br/>cmd_poll, calibrations, flash"]

    P0 -->|preempts| P1 -->|preempts| P2 -->|preempts| P15 -->|preempts| TH
```

Shared data between levels:
- main → ISR: references first, then mode/loop. 64-bit position through the `g_pos_ref_req` handshake.
- ISR → main: `g_pos_cnt` (atomic int32), filtered values (`g_vq_f`, `g_iq_f`), counters.
- Link IRQ ↔ main: single-producer/single-consumer rings; the TX ring head is written only with priority ≥ 1 masked (BASEPRI).
- CANopen locks mask priority ≥ 2 only, never the control ISR.
