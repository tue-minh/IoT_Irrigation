# Firmware Modification & Implementation Log

**Project:** IoT Irrigation Firmware (STM32F401CCU6)  
**Date:** 2026-11-11  
**Module:** Quectel EC600N 4G LTE AT Command Driver (USART1 2-Way DMA)

---

## 1. Summary of Changes

| Target | File | Action | Purpose / Description |
|---|---|---|---|
| **Linker Script** | [`STM32F401XX_FLASH.ld`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/STM32F401XX_FLASH.ld) | **Fixed** | Removed unsupported `(READONLY)` keyword from `.ARM.extab`, `.ARM`, `.preinit_array`, `.init_array`, and `.fini_array` sections to resolve GCC 8.2 linker failure. |
| **Build System** | [`CMakeLists.txt`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/CMakeLists.txt) | **Updated** | Added `Drivers/EC600N/Src/ec600n.c` and `Drivers/EC600N/Src/ec600n_at.c` to `target_sources`, and `Drivers/EC600N/Inc` to `target_include_directories`. |
| **Driver Config** | [`Drivers/EC600N/Inc/ec600n_config.h`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Drivers/EC600N/Inc/ec600n_config.h) | **Created** | Configured buffer sizes (DMA RX/TX 512B, Ring buffer 2048B, line buffer 512B, response buffer 1024B), timeout limits, and USB CDC / printf debug switches. |
| **Low-Level Header** | [`Drivers/EC600N/Inc/ec600n_at.h`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Drivers/EC600N/Inc/ec600n_at.h) | **Created** | Declared low-level AT command engine, 2-way DMA buffers, software FIFO ring buffer, URC table, error codes, and HAL callback interfaces. |
| **Low-Level Source** | [`Drivers/EC600N/Src/ec600n_at.c`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Drivers/EC600N/Src/ec600n_at.c) | **Created** | Implemented non-blocking DMA TX, DMA RX with IDLE line detection (`HAL_UARTEx_ReceiveToIdle_DMA`), ring buffer, line parser, prompt `>` detector, URC router, UART error/overrun auto-recovery, and USB CDC debug logging. |
| **High-Level Header** | [`Drivers/EC600N/Inc/ec600n.h`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Drivers/EC600N/Inc/ec600n.h) | **Created** | Declared high-level user API: SIM status, 4G network registration, signal quality (dBm/bars), PDP context / APN, RTC time, ping test, TCP sockets, hardware MQTT client, HTTP(S) client, and SMS text mode. |
| **High-Level Source** | [`Drivers/EC600N/Src/ec600n.c`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Drivers/EC600N/Src/ec600n.c) | **Created** | Implemented all Quectel EC600N AT command routines and built-in URC dispatchers (`+QIURC: "recv"`, `+QIURC: "closed"`, `+QMTRECV:`, `+CMTI:`, `+CPIN:`, `+CEREG:`). |
| **Documentation** | [`Drivers/EC600N/README.md`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Drivers/EC600N/README.md) | **Created** | Quick-start guide, hardware wiring pinout, AT command mapping table, and usage examples. |
| **Application** | [`Core/Src/main.c`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Core/Src/main.c) | **Updated** | Integrated EC600N initialization, self-test, SIM check, 4G network connection, RTC clock reading, ping test, and non-blocking event loop servicing in `while(1)`. |

---

## 2. Detailed Breakdown of Changes

### 2.1 Linker Script Incompatibility Fix
- **File:** [`STM32F401XX_FLASH.ld`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/STM32F401XX_FLASH.ld#L106-L150)
- **Problem:** When building with ARM GCC 8.2.1, the linker returned error:
  `non constant or forward reference address expression for section .ARM.extab`
- **Cause:** The `(READONLY)` section attribute was introduced in GCC 11+. GCC 8 fails on this keyword.
- **Resolution:** Removed `(READONLY)` from lines 106, 113, 122, 131, and 141 as instructed by ST's own comment inside the linker script.

### 2.2 2-Way DMA Implementation on USART1
- **Files:** [`Drivers/EC600N/Src/ec600n_at.c`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Drivers/EC600N/Src/ec600n_at.c), [`Drivers/EC600N/Inc/ec600n_at.h`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Drivers/EC600N/Inc/ec600n_at.h)
- **Reception (RX DMA + IDLE):**
  - Uses `HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx_buf, sizeof(dma_rx_buf))`.
  - When the EC600N finishes transmitting a response or URC, the UART IDLE line event fires.
  - In `HAL_UARTEx_RxEventCallback()`, incoming bytes are copied into a 2048-byte circular ring buffer and DMA reception is immediately re-armed.
  - Supports both DMA Normal and DMA Circular modes without code modifications.
- **Transmission (TX DMA):**
  - Uses `HAL_UART_Transmit_DMA(&huart1, dma_tx_buf, len)`.
  - Transfers are hardware-offloaded to DMA2 Stream 7 with completion monitored via `HAL_UART_TxCpltCallback()`.
- **Fault Tolerance:**
  - Implemented `HAL_UART_ErrorCallback()` to clear `ORE` (overrun), `NE` (noise), and `FE` (framing) error flags and automatically restart reception, preventing the UART from hanging.
- **URC Separation:**
  - Asynchronous URC lines (e.g., `+QIURC:`, `+QMTRECV:`, `+CMTI:`) are dispatched directly to registered callbacks and filtered out of AT command response buffers.
- **Data Prompt Handling:**
  - Handles raw prompt `> ` without newlines (required for `AT+QISEND`, `AT+QMTPUBEX`, and `AT+CMGS`).

### 2.3 High-Level Quectel EC600N Features
- **Files:** [`Drivers/EC600N/Src/ec600n.c`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Drivers/EC600N/Src/ec600n.c), [`Drivers/EC600N/Inc/ec600n.h`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Drivers/EC600N/Inc/ec600n.h)
- **Supported Capabilities:**
  1. **System & SIM:** `AT`, `ATE0`, `ATI`, `AT+GSN` (IMEI), `AT+CIMI` (IMSI), `AT+QCCID` (ICCID), `AT+CPIN?` (SIM Ready/PIN/PUK).
  2. **Radio & Network:** `AT+CSQ` (RSSI, BER, dBm, 0–5 bars), `AT+CEREG?` (LTE registration), `AT+COPS?` (Operator name e.g., Viettel).
  3. **Data Connection:** `AT+QICSGP=1` (APN configuration), `AT+QIACT=1` (PDP activation), `AT+QIACT?` (IP query), `AT+QIDEACT=1`.
  4. **Ping & Clock:** `AT+QPING` (Ping 8.8.8.8), `AT+CCLK?` (RTC clock), `AT+QNTP` (Network time sync).
  5. **TCP Sockets:** `AT+QIOPEN`, `AT+QISEND`, `AT+QIRD`, `AT+QICLOSE`, with `+QIURC: "recv"` data notifications.
  6. **Hardware MQTT:** `AT+QMTCFG`, `AT+QMTOPEN`, `AT+QMTCONN`, `AT+QMTPUBEX`, `AT+QMTSUB`, `AT+QMTDISC`.
  7. **HTTP(S) Client:** `AT+QHTTPCFG`, `AT+QHTTPURL`, `AT+QHTTPGET`, `AT+QHTTPPOST`, `AT+QHTTPREAD`.
  8. **SMS (Text Mode):** `AT+CMGF=1`, `AT+CMGS` (Send SMS), `AT+CMGR` (Read SMS), `AT+CMGD` (Delete), `+CMTI:` incoming notification.
  9. **Power Management:** `AT+QPOWD=1` (Graceful power-down), `AT+CFUN=1,1` (Soft reboot).

### 2.4 Application Implementation in `main.c`
- **File:** [`Core/Src/main.c`](file:///C:/Users/nguye/OneDrive/Desktop/IoT_Irrigation/firmware/Core/Src/main.c#L102-L170)
- Demonstrates a complete sequence:
  1. Initializes the EC600N driver on `huart1`.
  2. Registers MQTT and SMS callbacks.
  3. Tests communication (`AT`) and disables echo (`ATE0`).
  4. Reads module metadata and verifies SIM card status.
  5. Waits for 4G LTE registration and prints signal strength (dBm & bars).
  6. Connects 4G data (PDP context with APN `v-internet`) and retrieves local IP.
  7. Synchronizes and prints network time.
  8. Pings Google DNS (`8.8.8.8`) to verify end-to-end internet connectivity.
  9. Runs `ec600n_process(&ec600n)` inside `while(1)` to service DMA reception and URCs non-blockingly.

---

## 3. Build & Resource Verification

- **Compiler:** `arm-none-eabi-gcc 8.2.1`
- **Build System:** `CMake 3.22` + `Ninja`
- **Build Status:** Clean compilation (`0 errors, 0 warnings`)
- **Memory Consumption:**
  - **Flash:** `54,556 bytes / 262,144 bytes (20.81%)`
  - **RAM:** `11,624 bytes / 65,536 bytes (17.74%)`
