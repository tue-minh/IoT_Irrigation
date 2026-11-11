# Quectel EC600N 4G LTE Library for STM32

Production-grade C driver for **Quectel EC600N** (Cat 1 LTE) module communicating over **USART1 with 2-way DMA** (STM32 HAL).

---

## 1. Architecture & 2-Way DMA Design

- **UART Reception (RX with DMA & IDLE)**:
  - Uses `HAL_UARTEx_ReceiveToIdle_DMA()` with USART1 IDLE line detection.
  - Automatically captures variable-length AT command responses and Unsolicited Result Codes (URCs) without CPU polling.
  - Transfers bytes from DMA buffer into a **2048-byte software ring buffer (circular FIFO)**.
  - Recovers automatically from UART Overrun Errors (ORE), Noise Errors (NE), and Framing Errors (FE) via `HAL_UART_ErrorCallback()`.
  - Supports both DMA Normal and Circular modes.
- **UART Transmission (TX with DMA)**:
  - Uses `HAL_UART_Transmit_DMA()`.
  - Transmission is hardware-accelerated with completion tracking (`HAL_UART_TxCpltCallback()`) and configurable timeout.
- **AT Command Engine**:
  - Full request-response synchronization with timeout.
  - Prompt handling for data entry commands (`> ` for SMS, MQTT publish, TCP send).
  - Unsolicited Result Code (URC) dispatcher filters out unsolicited events (`+QIURC:`, `+QMTRECV:`, `+CMTI:`, `+CEREG:`) so they never corrupt command responses.
- **Debug Monitor**:
  - Outputs all AT logs and debug status to **USB CDC (Virtual COM Port)** or `printf` in real time.

---

## 2. File Structure

```
Drivers/EC600N/
├── Inc/
│   ├── ec600n.h        # High-level API (SIM, Network, MQTT, HTTP, TCP, SMS, RTC)
│   ├── ec600n_at.h     # AT engine, 2-way DMA transport, ring buffer, URC table
│   └── ec600n_config.h # Buffer sizes, timeouts, debug settings
└── Src/
    ├── ec600n.c        # High-level function implementations
    └── ec600n_at.c     # Low-level DMA ISR callbacks, parser, and logging
```

---

## 3. Supported Features & AT Commands

| Category | Functions | Underlying AT Commands |
|---|---|---|
| **System** | `ec600n_init`, `ec600n_check_communication`, `ec600n_echo`, `ec600n_get_device_info`, `ec600n_restart`, `ec600n_power_down` | `AT`, `ATE0/1`, `ATI`, `AT+GSN`, `AT+CIMI`, `AT+CFUN=1,1`, `AT+QPOWD=1` |
| **SIM Card** | `ec600n_check_sim`, `ec600n_get_iccid` | `AT+CPIN?`, `AT+QCCID` |
| **Network & Signal** | `ec600n_get_signal`, `ec600n_get_network_status`, `ec600n_wait_network_registered`, `ec600n_get_operator` | `AT+CSQ`, `AT+CEREG?`, `AT+COPS?` |
| **4G Data (PDP)** | `ec600n_activate_pdp`, `ec600n_deactivate_pdp`, `ec600n_get_ip`, `ec600n_ping` | `AT+QICSGP=1`, `AT+QIACT=1`, `AT+QIACT?`, `AT+QIDEACT=1`, `AT+QPING` |
| **Date & Time** | `ec600n_get_time`, `ec600n_ntp_sync` | `AT+CCLK?`, `AT+QNTP` |
| **TCP Sockets** | `ec600n_tcp_connect`, `ec600n_tcp_send`, `ec600n_tcp_read`, `ec600n_tcp_close`, `ec600n_set_tcp_recv_cb` | `AT+QIOPEN`, `AT+QISEND`, `AT+QIRD`, `AT+QICLOSE`, URC `+QIURC: "recv"` |
| **MQTT Client** | `ec600n_mqtt_connect`, `ec600n_mqtt_publish`, `ec600n_mqtt_subscribe`, `ec600n_mqtt_disconnect`, `ec600n_set_mqtt_recv_cb` | `AT+QMTCFG`, `AT+QMTOPEN`, `AT+QMTCONN`, `AT+QMTPUBEX`, `AT+QMTSUB`, `AT+QMTDISC` |
| **HTTP Client** | `ec600n_http_get`, `ec600n_http_post` | `AT+QHTTPCFG`, `AT+QHTTPURL`, `AT+QHTTPGET`, `AT+QHTTPPOST`, `AT+QHTTPREAD` |
| **SMS** | `ec600n_sms_send`, `ec600n_sms_read`, `ec600n_sms_delete`, `ec600n_set_sms_recv_cb` | `AT+CMGF=1`, `AT+CMGS`, `AT+CMGR`, `AT+CMGD`, URC `+CMTI:` |

---

## 4. Hardware Pin Connections (STM32F401 <-> EC600N)

- **USART1_TX (PA9)**  -->  **EC600N RXD**
- **USART1_RX (PA10)** <--  **EC600N TXD**
- **GND**              <--> **GND**
- **VCC (3.8V - 4.2V, 2A peak)** --> Power supply for EC600N
- *(Optional)* PWRKEY / RESET GPIOs can be registered with `ec600n_set_hw_pins()` if wired.
