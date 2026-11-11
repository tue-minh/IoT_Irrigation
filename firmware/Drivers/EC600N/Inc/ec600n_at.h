/**
  ******************************************************************************
  * @file    ec600n_at.h
  * @brief   AT command engine and 2-way DMA transport for Quectel EC600N.
  ******************************************************************************
  */

#ifndef __EC600N_AT_H__
#define __EC600N_AT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ec600n_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* Result / Error codes */
typedef enum {
    EC600N_OK                 =  0,
    EC600N_ERR_TIMEOUT        = -1,
    EC600N_ERR_AT_ERROR       = -2,
    EC600N_ERR_PARAM          = -3,
    EC600N_ERR_BUSY           = -4,
    EC600N_ERR_BUFFER_FULL    = -5,
    EC600N_ERR_NOT_READY      = -6,
    EC600N_ERR_SIM            = -7,
    EC600N_ERR_NET            = -8,
    EC600N_ERR_SOCKET         = -9,
    EC600N_ERR_MQTT           = -10,
    EC600N_ERR_HTTP           = -11
} ec600n_err_t;

/* URC Callback signature */
typedef void (*ec600n_urc_cb_t)(const char *line, void *user_data);

/* Single URC registration entry */
typedef struct {
    const char          *prefix;
    ec600n_urc_cb_t      callback;
    void                *user_data;
} ec600n_urc_entry_t;

/* Ring buffer structure */
typedef struct {
    uint8_t             buffer[EC600N_RING_BUFFER_SIZE];
    volatile uint16_t   head;
    volatile uint16_t   tail;
} ec600n_ring_buffer_t;

/* AT Command Engine Handle */
typedef struct {
    UART_HandleTypeDef *huart;

    /* Hardware DMA buffers */
    uint8_t             dma_rx_buf[EC600N_DMA_RX_BUFFER_SIZE];
    uint8_t             dma_tx_buf[EC600N_DMA_TX_BUFFER_SIZE];
    uint16_t            dma_rx_last_pos;

    /* Software RX Ring Buffer */
    ec600n_ring_buffer_t ring_buf;

    /* DMA TX State */
    volatile bool       tx_complete;

    /* Synchronous Command State */
    volatile bool       waiting_response;
    const char         *expected_resp;
    char               *cmd_resp_buf;
    uint16_t            cmd_resp_max_len;
    uint16_t            cmd_resp_len;
    volatile ec600n_err_t cmd_status;

    /* Line parser assembly buffer */
    char                line_buf[EC600N_LINE_BUFFER_SIZE];
    uint16_t            line_len;

    /* URC Handlers Table */
    ec600n_urc_entry_t  urc_table[EC600N_MAX_URC_HANDLERS];
    uint8_t             urc_count;
} ec600n_at_t;

/* Global instance pointer for HAL interrupt routing */
extern ec600n_at_t *g_ec600n_at_handle;

/* Initialization and Lifecycle */
ec600n_err_t ec600n_at_init(ec600n_at_t *at, UART_HandleTypeDef *huart);
void         ec600n_at_process(ec600n_at_t *at);

/* AT Command Communication */
ec600n_err_t ec600n_at_send_cmd(ec600n_at_t *at,
                                const char *cmd,
                                const char *expected_resp,
                                char *resp_out,
                                uint16_t resp_max_len,
                                uint32_t timeout_ms);

ec600n_err_t ec600n_at_send_raw(ec600n_at_t *at,
                                const uint8_t *data,
                                uint16_t len,
                                uint32_t timeout_ms);

/* URC Handling */
ec600n_err_t ec600n_at_register_urc(ec600n_at_t *at,
                                    const char *prefix,
                                    ec600n_urc_cb_t callback,
                                    void *user_data);

ec600n_err_t ec600n_at_unregister_urc(ec600n_at_t *at, const char *prefix);

/* HAL Callbacks (can be invoked directly or automatically via weak HAL symbols) */
void ec600n_at_rx_event_callback(ec600n_at_t *at, uint16_t size);
void ec600n_at_tx_cplt_callback(ec600n_at_t *at);
void ec600n_at_error_callback(ec600n_at_t *at);

/* Debug / Logging Helper */
void ec600n_log(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __EC600N_AT_H__ */
