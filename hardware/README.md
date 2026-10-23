# Hardware

KiCad project of the fluxdrive-g4 board (made with KiCad 10): open `fluxdrive-g4.kicad_pro`. The root schematic holds three sheets (`01 - Controller`, `02 - Connections`, `03 - Power`); `fluxdrive-g4 schematic.pdf` is the same schematic as a PDF.

The hardware design files are licensed under the CERN Open Hardware Licence Version 2 - Permissive (CERN-OHL-P-2.0), see [LICENSE](LICENSE).

## Datasheets of the main parts

The datasheets are not in the repository (they are the manufacturers' documents). Download them from the links below; `hardware/datasheets/` is git-ignored, so local copies can go there.

| Part | Manufacturer | Role | Datasheet |
|---|---|---|---|
| STSPIN32G4 | STMicroelectronics | Controller: STM32G431 + 3-phase gate driver | [st.com](https://www.st.com/resource/en/datasheet/stspin32g4.pdf) |
| STM32G431VB | STMicroelectronics | The MCU inside the STSPIN32G4 | [st.com](https://www.st.com/resource/en/datasheet/stm32g431vb.pdf) |
| DI100N10PQ | Diotec | Bridge MOSFETs (100 V) | [diotec.com](https://diotec.com/request/datasheet/di100n10pq.pdf) |
| MT6835 | MagnTek (Novosense) | 21-bit magnetic encoder | [novosns.com](https://www.novosns.com/en/) (search for MT6835) |
| TJA1051 | NXP | CAN transceiver | [nxp.com](https://www.nxp.com/docs/en/data-sheet/TJA1051.pdf) |
| USBLC6-2 | STMicroelectronics | USB ESD protection | [st.com](https://www.st.com/resource/en/datasheet/usblc6-2.pdf) |
| ANTC1608-103F3450FB | APV Technologies | Board NTC (10 kΩ, B 3450 K, 0603) | [apvind.com](https://www.apvind.com/) (ANTC series) |

The STM32G4 reference manual (RM0440, register descriptions): [st.com](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).
