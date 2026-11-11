/**
  ******************************************************************************
  * @file    ec600n_at.c
  * @brief   Implementation of AT command engine and 2-way DMA transport.
  ******************************************************************************
  */

#include "ec600n_at.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#if (EC600N_DEBUG_USE_USB_CDC == 1)
#include "usbd_cdc_if.h"
#include "usb_device.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif

/* Global AT handle pointer */
ec600n_at_t *g_ec600n_at_handle = NULL;

/* -------------------------------------------------------------------------- */
/* Logging Function                                                           */
/* -------------------------------------------------------------------------- */
void ec600n_log(const char *fmt, ...)
{
#if (EC600N_DEBUG_ENABLE == 1)
    char log_buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(log_buf, sizeof(log_buf), fmt, args);
    va_end(args);

    if (len <= 0) return;

#if (EC600N_DEBUG_USE_USB_CDC == 1)
    /* Transmit over USB CDC if device is configured */
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
    {
        uint32_t start = HAL_GetTick();
        while (CDC_Transmit_FS((uint8_t *)log_buf, (uint16_t)len) == USBD_BUSY)
        {
            if (HAL_GetTick() - start > 10) break;
        }
    }
#else
    printf("%s", log_buf);
#endif
#endif
}

/* -------------------------------------------------------------------------- */
/* Ring Buffer Helpers                                                        */
/* -------------------------------------------------------------------------- */
static inline bool ring_push(ec600n_ring_buffer_t *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1) % EC600N_RING_BUFFER_SIZE;
    if (next == rb->tail) {
        return false; /* Buffer overflow */
    }
    rb->buffer[rb->head] = byte;
    rb->head = next;
    return true;
}

static inline bool ring_pop(ec600n_ring_buffer_t *rb, uint8_t *byte)
{
    if (rb->head == rb->tail) {
        return false; /* Empty */
    }
    *byte = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % EC600N_RING_BUFFER_SIZE;
    return true;
}

static inline uint16_t ring_count(const ec600n_ring_buffer_t *rb)
{
    if (rb->head >= rb->tail) {
        return (rb->head - rb->tail);
    }
    return (EC600N_RING_BUFFER_SIZE - rb->tail + rb->head);
}

/* -------------------------------------------------------------------------- */
/* Engine Initialization                                                      */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_at_init(ec600n_at_t *at, UART_HandleTypeDef *huart)
{
    if (at == NULL || huart == NULL) {
        return EC600N_ERR_PARAM;
    }

    memset(at, 0, sizeof(ec600n_at_t));
    at->huart = huart;
    at->tx_complete = true;
    g_ec600n_at_handle = at;

    /* Start Reception with DMA and IDLE detection */
    HAL_StatusTypeDef hal_st = HAL_UARTEx_ReceiveToIdle_DMA(huart, at->dma_rx_buf, EC600N_DMA_RX_BUFFER_SIZE);
    if (hal_st != HAL_OK) {
        ec600n_log("[EC600N] DMA RX start failed: %d\r\n", (int)hal_st);
        return EC600N_ERR_NOT_READY;
    }

    ec600n_log("[EC600N] AT Engine initialized on USART1 with 2-way DMA\r\n");
    return EC600N_OK;
}

/* -------------------------------------------------------------------------- */
/* Low-Level Raw Transmission with DMA                                        */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_at_send_raw(ec600n_at_t *at,
                                const uint8_t *data,
                                uint16_t len,
                                uint32_t timeout_ms)
{
    if (at == NULL || at->huart == NULL || data == NULL || len == 0) {
        return EC600N_ERR_PARAM;
    }

    if (len > EC600N_DMA_TX_BUFFER_SIZE) {
        return EC600N_ERR_PARAM;
    }

    /* Wait for any ongoing DMA TX to complete */
    uint32_t start = HAL_GetTick();
    while (!at->tx_complete) {
        if (HAL_GetTick() - start >= timeout_ms) {
            return EC600N_ERR_TIMEOUT;
        }
    }

    memcpy(at->dma_tx_buf, data, len);
    at->tx_complete = false;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(at->huart, at->dma_tx_buf, len);
    if (status != HAL_OK) {
        at->tx_complete = true;
        return EC600N_ERR_BUSY;
    }

    /* Wait for transmission to finish */
    start = HAL_GetTick();
    while (!at->tx_complete) {
        if (HAL_GetTick() - start >= timeout_ms) {
            return EC600N_ERR_TIMEOUT;
        }
    }

    return EC600N_OK;
}

/* -------------------------------------------------------------------------- */
/* Synchronous Command Execution                                              */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_at_send_cmd(ec600n_at_t *at,
                                const char *cmd,
                                const char *expected_resp,
                                char *resp_out,
                                uint16_t resp_max_len,
                                uint32_t timeout_ms)
{
    if (at == NULL || cmd == NULL) {
        return EC600N_ERR_PARAM;
    }

    /* Configure response waiting context */
    at->expected_resp = expected_resp;
    at->cmd_resp_buf = resp_out;
    at->cmd_resp_max_len = resp_max_len;
    at->cmd_resp_len = 0;
    at->cmd_status = EC600N_ERR_TIMEOUT;
    at->waiting_response = true;

    if (resp_out && resp_max_len > 0) {
        resp_out[0] = '\0';
    }

#if (EC600N_DEBUG_DUMP_AT == 1)
    ec600n_log("[EC600N TX] %s\r\n", cmd);
#endif

    /* Build command string with \r\n */
    char full_cmd[EC600N_DMA_TX_BUFFER_SIZE];
    int len = snprintf(full_cmd, sizeof(full_cmd), "%s\r\n", cmd);
    if (len <= 0 || (uint16_t)len >= sizeof(full_cmd)) {
        at->waiting_response = false;
        return EC600N_ERR_PARAM;
    }

    /* Send via DMA */
    ec600n_err_t ret = ec600n_at_send_raw(at, (const uint8_t *)full_cmd, (uint16_t)len, timeout_ms);
    if (ret != EC600N_OK) {
        at->waiting_response = false;
        return ret;
    }

    /* Wait for response while driving the AT process loop */
    uint32_t start = HAL_GetTick();
    while (at->waiting_response) {
        ec600n_at_process(at);

        if (HAL_GetTick() - start >= timeout_ms) {
            at->waiting_response = false;
            ec600n_log("[EC600N] Command '%s' timeout (%lu ms)\r\n", cmd, timeout_ms);
            return EC600N_ERR_TIMEOUT;
        }
    }

    return at->cmd_status;
}

/* -------------------------------------------------------------------------- */
/* URC Management                                                             */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_at_register_urc(ec600n_at_t *at,
                                    const char *prefix,
                                    ec600n_urc_cb_t callback,
                                    void *user_data)
{
    if (at == NULL || prefix == NULL || callback == NULL) {
        return EC600N_ERR_PARAM;
    }

    if (at->urc_count >= EC600N_MAX_URC_HANDLERS) {
        return EC600N_ERR_BUFFER_FULL;
    }

    at->urc_table[at->urc_count].prefix = prefix;
    at->urc_table[at->urc_count].callback = callback;
    at->urc_table[at->urc_count].user_data = user_data;
    at->urc_count++;

    return EC600N_OK;
}

ec600n_err_t ec600n_at_unregister_urc(ec600n_at_t *at, const char *prefix)
{
    if (at == NULL || prefix == NULL) {
        return EC600N_ERR_PARAM;
    }

    for (uint8_t i = 0; i < at->urc_count; i++) {
        if (strcmp(at->urc_table[i].prefix, prefix) == 0) {
            for (uint8_t j = i; j < at->urc_count - 1; j++) {
                at->urc_table[j] = at->urc_table[j + 1];
            }
            at->urc_count--;
            return EC600N_OK;
        }
    }
    return EC600N_ERR_PARAM;
}

/* -------------------------------------------------------------------------- */
/* Line and URC Processing Routine                                            */
/* -------------------------------------------------------------------------- */
static void handle_line(ec600n_at_t *at, const char *line)
{
    if (line == NULL || line[0] == '\0') {
        return;
    }

#if (EC600N_DEBUG_DUMP_AT == 1)
    ec600n_log("[EC600N RX] %s\r\n", line);
#endif

    /* 1. Check against registered URC handlers */
    bool was_urc = false;
    for (uint8_t i = 0; i < at->urc_count; i++) {
        if (strncmp(line, at->urc_table[i].prefix, strlen(at->urc_table[i].prefix)) == 0) {
            was_urc = true;
            if (at->urc_table[i].callback) {
                at->urc_table[i].callback(line, at->urc_table[i].user_data);
            }
            break;
        }
    }

    /* If it was a recognized URC, don't treat it as AT command response */
    if (was_urc) {
        return;
    }

    /* 2. Process command response if in waiting state */
    if (at->waiting_response) {
        /* Append line to output buffer if provided */
        if (at->cmd_resp_buf && at->cmd_resp_max_len > 0) {
            uint16_t cur_len = at->cmd_resp_len;
            int appended = snprintf(at->cmd_resp_buf + cur_len,
                                    at->cmd_resp_max_len - cur_len,
                                    "%s\r\n", line);
            if (appended > 0) {
                at->cmd_resp_len += (uint16_t)appended;
            }
        }

        /* Check for expected response */
        if (at->expected_resp != NULL && strstr(line, at->expected_resp) != NULL) {
            at->cmd_status = EC600N_OK;
            at->waiting_response = false;
            return;
        }

        /* Check for error responses */
        if (strcmp(line, "ERROR") == 0 ||
            strncmp(line, "+CME ERROR:", 11) == 0 ||
            strncmp(line, "+CMS ERROR:", 11) == 0) {
            at->cmd_status = EC600N_ERR_AT_ERROR;
            at->waiting_response = false;
            return;
        }
    }
}

void ec600n_at_process(ec600n_at_t *at)
{
    if (at == NULL) return;

    uint8_t b;
    while (ring_pop(&at->ring_buf, &b)) {
        /* Check special case: Prompt '>' for data entry (no newline) */
        if (b == '>' && at->waiting_response && at->expected_resp && strcmp(at->expected_resp, ">") == 0) {
            at->cmd_status = EC600N_OK;
            at->waiting_response = false;
            at->line_len = 0;
            continue;
        }

        if (b == '\r') {
            continue; /* Ignore carriage return */
        }

        if (b == '\n') {
            if (at->line_len > 0) {
                at->line_buf[at->line_len] = '\0';
                handle_line(at, at->line_buf);
                at->line_len = 0;
            }
        } else {
            if (at->line_len < EC600N_LINE_BUFFER_SIZE - 1) {
                at->line_buf[at->line_len++] = (char)b;
            } else {
                /* Buffer overflow protection: flush line */
                at->line_buf[at->line_len] = '\0';
                handle_line(at, at->line_buf);
                at->line_len = 0;
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Interrupt & HAL Callbacks                                                  */
/* -------------------------------------------------------------------------- */
void ec600n_at_rx_event_callback(ec600n_at_t *at, uint16_t size)
{
    if (at == NULL || at->huart == NULL) return;

    if (at->huart->hdmarx != NULL && at->huart->hdmarx->Init.Mode == DMA_CIRCULAR) {
        /* Circular mode: calculate delta from last position */
        if (size >= at->dma_rx_last_pos) {
            for (uint16_t i = at->dma_rx_last_pos; i < size; i++) {
                ring_push(&at->ring_buf, at->dma_rx_buf[i]);
            }
        } else {
            for (uint16_t i = at->dma_rx_last_pos; i < EC600N_DMA_RX_BUFFER_SIZE; i++) {
                ring_push(&at->ring_buf, at->dma_rx_buf[i]);
            }
            for (uint16_t i = 0; i < size; i++) {
                ring_push(&at->ring_buf, at->dma_rx_buf[i]);
            }
        }
        at->dma_rx_last_pos = size % EC600N_DMA_RX_BUFFER_SIZE;
    } else {
        /* Normal mode: push size bytes and re-arm DMA immediately */
        for (uint16_t i = 0; i < size; i++) {
            ring_push(&at->ring_buf, at->dma_rx_buf[i]);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(at->huart, at->dma_rx_buf, EC600N_DMA_RX_BUFFER_SIZE);
    }
}

void ec600n_at_tx_cplt_callback(ec600n_at_t *at)
{
    if (at != NULL) {
        at->tx_complete = true;
    }
}

void ec600n_at_error_callback(ec600n_at_t *at)
{
    if (at == NULL || at->huart == NULL) return;

    /* Clear UART error flags */
    __HAL_UART_CLEAR_OREFLAG(at->huart);
    __HAL_UART_CLEAR_NEFLAG(at->huart);
    __HAL_UART_CLEAR_FEFLAG(at->huart);
    __HAL_UART_CLEAR_PEFLAG(at->huart);

    /* Re-arm reception */
    HAL_UARTEx_ReceiveToIdle_DMA(at->huart, at->dma_rx_buf, EC600N_DMA_RX_BUFFER_SIZE);
}

/* -------------------------------------------------------------------------- */
/* HAL Weak Symbols Implementation                                            */
/* -------------------------------------------------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (g_ec600n_at_handle && g_ec600n_at_handle->huart == huart) {
        ec600n_at_rx_event_callback(g_ec600n_at_handle, Size);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (g_ec600n_at_handle && g_ec600n_at_handle->huart == huart) {
        ec600n_at_tx_cplt_callback(g_ec600n_at_handle);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (g_ec600n_at_handle && g_ec600n_at_handle->huart == huart) {
        ec600n_at_error_callback(g_ec600n_at_handle);
    }
}
