# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

bladeRF is a Nuand software-defined radio platform. The repo contains four distinct build domains, each with its own toolchain:

| Domain | Language | Toolchain | Output |
|--------|----------|-----------|--------|
| FX3 firmware | C (ARM9) | arm-none-eabi-gcc + Cypress FX3 SDK | `.img` bootable image |
| Host library & CLI | C/C++ | Host GCC/MSVC | `libbladeRF.so`, `bladeRF-cli` |
| FPGA HDL | VHDL | Quartus Prime (Lite OK) | `.rbf` bitstream |
| FPGA/firmware common | C headers | — | Shared constants |

## Build Commands

### FX3 Firmware (cross-compiled ARM)

Requires Cypress FX3 SDK v1.3.3+ (default: `C:\Program Files (x86)\Cypress\EZ-USB FX3 SDK\1.3` on Windows, `/opt/cypress/fx3_sdk` on Linux).

```bash
cd fx3_firmware
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/fx3-toolchain.cmake
make
# Output: build/output/bladeRF_fw_v<VERSION>.img
```

From the repo root (delegates to ExternalProject):
```bash
cmake -DENABLE_FX3_BUILD=ON -DENABLE_HOST_BUILD=OFF ..
make
```

### Host Library & CLI

```bash
cd host
mkdir build && cd build
cmake ..
make -j$(nproc)
# Output: build/output/<build_type>/
```

Requires: `libusb >= 1.0.16` (Linux), `>= 1.0.19` (Windows).

### FPGA (Quartus)

```bash
cd hdl/quartus
./build_bladerf.sh -b bladeRF-micro -s A4 -r hosted
# Output: hostedxA4-<timestamp>/ directory with .rbf bitstream
# Must run from within NIOS II command shell environment
```

## Architecture

### FX3 Firmware (`fx3_firmware/src/`)

The firmware runs ThreadX RTOS on Cypress FX3 (ARM926EJ-S). It exposes multiple USB interfaces and bridges them to the FPGA via the GPIF II parallel bus.

**USB Interface layout (current EEM branch):**
- Interface 0 — Vendor-specific RF + UART (4 alt settings 0–3)
- Interface 1 — CDC EEM (Ethernet Emulation Model, single alt setting)

**Application abstraction (`struct NuandApplication` in `bladeRF.h`):**  
Function-pointer struct used for all "modes". Implementations:
- `NuandRFLink` (`rf.c`) — RF sample path + UART bridge (alt 2)
- `NuandFpgaConfig` (`fpga.c`) — FPGA bitstream loading (alt 3)
- `NuandEEMLink` (`eem.c`) — CDC EEM DMA path (started/stopped within RFLink lifecycle)

**GPIF socket assignment:**
| PIB Socket | Direction | Usage |
|-----------|-----------|-------|
| 0 (RX0) | FPGA→FX3 | RF RX samples |
| 1 (RX1) | FPGA→FX3 | EEM data (FPGA→host) |
| 2 (TX2) | FX3→FPGA | EEM data (host→FPGA) |
| 3 (TX3) | FX3→FPGA | RF TX samples |

The GPIF state machine activates all 4 threads (0–3) simultaneously in RF_LINK mode. EEM DMA channels are created/destroyed inside `NuandRFLinkStart()`/`NuandRFLinkStop()` in `rf.c`, ensuring correct GPIF lifecycle ordering.

**DMA channel types:** `CY_U3P_DMA_TYPE_AUTO` channels — data flows between USB endpoints and PIB sockets with zero CPU involvement.

**USB Descriptor files:**
- `cyfxbladeRFusbdscr.c` — All descriptor byte arrays. Contains `_DUAL` variants (with EEM Interface 1) used when the EEM interface is active.
- `bladeRF.h` — Master header: GPIO defines, mode constants (`MODE_RF_CONFIG=2`, etc.), `struct NuandApplication`
- `bladeRF.c` — USB event handlers, vendor request dispatch, `bladeRFInit()`

**EEM endpoint constants** (defined in `bladeRF.h`):
- `BLADE_RF_EEM_EP_PRODUCER` = `0x03` (EP3 OUT, USB→FX3→FPGA)
- `BLADE_RF_EEM_EP_CONSUMER` = `0x83` (EP3 IN, FPGA→FX3→USB)
- Corresponding PIB sockets: `CY_U3P_UIB_SOCKET_PROD_3` / `CY_U3P_UIB_SOCKET_CONS_3`

### FPGA HDL (`hdl/fpga/`)

Single top-level entity `bladerf.vhd`. Multiple Quartus "Revisions" select different architectures (e.g., `hosted`, `atsc_tx`). Targets Cyclone IV (bladeRF 1.0) or Cyclone V (bladeRF Micro / bladeRF 2.0).

EEM transmit framing logic lives in `hdl/fpga/platforms/bladerf-micro/vhdl/eem/eem_tx_framer.vhd`:
- Captures byte-serial Ethernet frames into a 1536-byte M10K buffer
- Prepends a 2-byte CDC-EEM header (bit15=0 data, bits13:0=length)
- Flushes captured frame in 32-bit words into the RX1 FIFO (FPGA→host path)

### Shared Headers (`firmware_common/`, `fpga_common/`)

Shared between FX3 firmware and host code (and sometimes HDL). Contains USB vendor request codes, SPI flash layout, version definitions, logger IDs.

`firmware_common/logger_id.h` — assigns a numeric ID to each firmware source file for compact on-device logging.

## Code Conventions

From `doc/development/style_and_conventions.md`:
- 4-space indentation, no tabs, Unix line endings
- Function naming: `[module]_[submodule]_[verb]_[noun]`
- Use explicit comparisons (`ptr != NULL`, not `!ptr`)
- Commit messages: `[module]: Brief summary` (< 80 chars/line), one objective per commit

## Licensing

Component licenses differ — check before adding dependencies:
- FX3 firmware & FPGA HDL: MIT
- `libbladeRF`: LGPLv2.1
- `bladeRF-cli`: GPLv2
- Linux kernel driver: GPLv2

## Active Development Branch

`eem2` — Adding CDC EEM USB network interface (Interface 1) alongside the existing vendor RF interface (Interface 0). New files: `fx3_firmware/src/eem.c`, `fx3_firmware/src/eem.h`. Modified: `bladeRF.c`, `rf.c`, `cyfxbladeRFusbdscr.c`, `bladeRF.h`, `CMakeLists.txt`, `firmware_common/logger_id.h`.
