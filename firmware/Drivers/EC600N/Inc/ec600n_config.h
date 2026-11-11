/**
  ******************************************************************************
  * @file    ec600n_config.h
  * @brief   Configuration parameters for Quectel EC600N 4G LTE Driver.
  ******************************************************************************
  */

#ifndef __EC600N_CONFIG_H__
#define __EC600N_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Debug output options */
#define EC600N_DEBUG_ENABLE             1       /* 1 = Enable debug logging, 0 = Disable */
#define EC600N_DEBUG_USE_USB_CDC        1       /* 1 = Print via USB CDC (Virtual COM), 0 = Use printf */
#define EC600N_DEBUG_DUMP_AT            1       /* 1 = Dump all raw AT commands and responses */

/* Buffer sizes */
#define EC600N_DMA_RX_BUFFER_SIZE       512     /* UART DMA RX buffer size */
#define EC600N_DMA_TX_BUFFER_SIZE       512     /* UART DMA TX buffer size */
#define EC600N_RING_BUFFER_SIZE         2048    /* Software circular FIFO buffer for incoming UART bytes */
#define EC600N_CMD_RESP_BUFFER_SIZE     1024    /* Maximum buffer size for command responses */
#define EC600N_LINE_BUFFER_SIZE         512     /* Maximum length for a single AT response line */

/* Handler limits */
#define EC600N_MAX_URC_HANDLERS         12      /* Maximum number of registered URC handlers */

/* Default Timeouts (in milliseconds) */
#define EC600N_DEFAULT_TIMEOUT_MS       2000    /* Default timeout for simple AT commands */
#define EC600N_EXTENDED_TIMEOUT_MS      10000   /* Timeout for network / SIM queries */
#define EC600N_NETWORK_ACT_TIMEOUT_MS   30000   /* Timeout for PDP context activation (QIACT) */
#define EC600N_MQTT_TIMEOUT_MS          15000   /* Timeout for MQTT operations */
#define EC600N_HTTP_TIMEOUT_MS          20000   /* Timeout for HTTP operations */
#define EC600N_SMS_TIMEOUT_MS           15000   /* Timeout for SMS operations */

#ifdef __cplusplus
}
#endif

#endif /* __EC600N_CONFIG_H__ */
