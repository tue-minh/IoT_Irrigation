/**
  ******************************************************************************
  * @file    ec600n.h
  * @brief   Main API header for Quectel EC600N 4G LTE Module Driver.
  ******************************************************************************
  */

#ifndef __EC600N_H__
#define __EC600N_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ec600n_at.h"
#include <stdbool.h>
#include <stdint.h>

/* SIM Status */
typedef enum {
    EC600N_SIM_UNKNOWN = 0,
    EC600N_SIM_READY,
    EC600N_SIM_PIN_REQUIRED,
    EC600N_SIM_PUK_REQUIRED,
    EC600N_SIM_NOT_INSERTED,
    EC600N_SIM_ERROR
} ec600n_sim_status_t;

/* Network Registration Status */
typedef enum {
    EC600N_NET_NOT_REGISTERED = 0,
    EC600N_NET_REGISTERED_HOME,
    EC600N_NET_SEARCHING,
    EC600N_NET_DENIED,
    EC600N_NET_UNKNOWN_STAT,
    EC600N_NET_REGISTERED_ROAMING
} ec600n_net_stat_t;

/* Signal Quality Info */
typedef struct {
    uint8_t rssi;          /* 0..31, 99 = unknown */
    uint8_t ber;           /* 0..7, 99 = unknown */
    int16_t rssi_dbm;      /* Calculated in dBm (-113 to -51 dBm) */
    uint8_t bars;          /* 0 to 5 signal strength bars */
} ec600n_signal_t;

/* Device hardware and firmware information */
typedef struct {
    char model[32];
    char fw_rev[32];
    char imei[24];
    char imsi[24];
    char iccid[24];
} ec600n_info_t;

/* Network information */
typedef struct {
    ec600n_net_stat_t status;
    char              operator_name[32];
    char              ip_address[20];
    ec600n_signal_t   signal;
} ec600n_net_info_t;

/* Real-Time Clock / Date-Time */
typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    int8_t   tz_quarters;  /* Timezone in quarter of an hour (+28 = +7h for Vietnam/ICT) */
} ec600n_time_t;

/* MQTT Configuration */
typedef struct {
    const char *broker;
    uint16_t    port;
    const char *client_id;
    const char *username;
    const char *password;
    uint8_t     keepalive_sec;
    uint8_t     clean_session;
} ec600n_mqtt_cfg_t;

/* SMS message structure */
typedef struct {
    uint8_t index;
    char    sender[24];
    char    timestamp[24];
    char    text[160];
} ec600n_sms_t;

/* Callback Types */
typedef void (*ec600n_tcp_recv_cb_t)(uint8_t connect_id, const uint8_t *data, uint16_t len);
typedef void (*ec600n_tcp_closed_cb_t)(uint8_t connect_id);
typedef void (*ec600n_mqtt_recv_cb_t)(const char *topic, const uint8_t *payload, uint16_t len);
typedef void (*ec600n_sms_recv_cb_t)(uint8_t index);

/* Optional GPIO hardware control pins */
typedef struct {
    GPIO_TypeDef *pwrkey_port;
    uint16_t      pwrkey_pin;
    GPIO_TypeDef *reset_port;
    uint16_t      reset_pin;
} ec600n_hw_pins_t;

/* Main EC600N Driver Instance */
typedef struct {
    ec600n_at_t          at;
    ec600n_info_t        info;
    ec600n_sim_status_t  sim_status;
    ec600n_net_info_t    net_info;
    ec600n_hw_pins_t     pins;

    /* Callback Hooks */
    ec600n_tcp_recv_cb_t   tcp_recv_cb;
    ec600n_tcp_closed_cb_t tcp_closed_cb;
    ec600n_mqtt_recv_cb_t  mqtt_recv_cb;
    ec600n_sms_recv_cb_t   sms_recv_cb;
} ec600n_device_t;

/* Global Device Reference */
extern ec600n_device_t g_ec600n_dev;

/* Initialization & Lifecycle */
ec600n_err_t ec600n_init(ec600n_device_t *dev, UART_HandleTypeDef *huart);
void         ec600n_set_hw_pins(ec600n_device_t *dev,
                                GPIO_TypeDef *pwrkey_port, uint16_t pwrkey_pin,
                                GPIO_TypeDef *reset_port,  uint16_t reset_pin);
void         ec600n_process(ec600n_device_t *dev);

/* Basic Module Control */
ec600n_err_t ec600n_check_communication(ec600n_device_t *dev);
ec600n_err_t ec600n_echo(ec600n_device_t *dev, bool enable);
ec600n_err_t ec600n_get_device_info(ec600n_device_t *dev);
ec600n_err_t ec600n_power_down(ec600n_device_t *dev);
ec600n_err_t ec600n_restart(ec600n_device_t *dev);

/* SIM Card Management */
ec600n_err_t ec600n_check_sim(ec600n_device_t *dev);
ec600n_err_t ec600n_get_iccid(ec600n_device_t *dev, char *iccid, uint8_t max_len);

/* Signal Quality & Network Status */
ec600n_err_t ec600n_get_signal(ec600n_device_t *dev, ec600n_signal_t *signal);
ec600n_err_t ec600n_get_network_status(ec600n_device_t *dev, ec600n_net_stat_t *stat);
ec600n_err_t ec600n_wait_network_registered(ec600n_device_t *dev, uint32_t timeout_ms);
ec600n_err_t ec600n_get_operator(ec600n_device_t *dev, char *operator_name, uint8_t max_len);

/* PDP Context & 4G Internet */
ec600n_err_t ec600n_activate_pdp(ec600n_device_t *dev, const char *apn, const char *user, const char *pwd);
ec600n_err_t ec600n_deactivate_pdp(ec600n_device_t *dev);
ec600n_err_t ec600n_get_ip(ec600n_device_t *dev, char *ip_buf, uint8_t max_len);
ec600n_err_t ec600n_ping(ec600n_device_t *dev, const char *host, uint32_t *rtt_ms);

/* Date, Time & NTP */
ec600n_err_t ec600n_get_time(ec600n_device_t *dev, ec600n_time_t *time);
ec600n_err_t ec600n_ntp_sync(ec600n_device_t *dev, const char *ntp_server);

/* TCP Socket API */
ec600n_err_t ec600n_tcp_connect(ec600n_device_t *dev, uint8_t connect_id, const char *host, uint16_t port);
ec600n_err_t ec600n_tcp_send(ec600n_device_t *dev, uint8_t connect_id, const uint8_t *data, uint16_t len);
ec600n_err_t ec600n_tcp_read(ec600n_device_t *dev, uint8_t connect_id, uint8_t *buf, uint16_t max_len, uint16_t *read_len);
ec600n_err_t ec600n_tcp_close(ec600n_device_t *dev, uint8_t connect_id);
void         ec600n_set_tcp_recv_cb(ec600n_device_t *dev, ec600n_tcp_recv_cb_t cb);
void         ec600n_set_tcp_closed_cb(ec600n_device_t *dev, ec600n_tcp_closed_cb_t cb);

/* MQTT Client API (Quectel Hardware MQTT) */
ec600n_err_t ec600n_mqtt_connect(ec600n_device_t *dev, const ec600n_mqtt_cfg_t *cfg);
ec600n_err_t ec600n_mqtt_disconnect(ec600n_device_t *dev);
ec600n_err_t ec600n_mqtt_publish(ec600n_device_t *dev, const char *topic, const char *payload, uint8_t qos, uint8_t retain);
ec600n_err_t ec600n_mqtt_subscribe(ec600n_device_t *dev, const char *topic, uint8_t qos);
void         ec600n_set_mqtt_recv_cb(ec600n_device_t *dev, ec600n_mqtt_recv_cb_t cb);

/* HTTP Client API */
ec600n_err_t ec600n_http_get(ec600n_device_t *dev, const char *url, char *response, uint16_t max_len, uint32_t timeout_ms);
ec600n_err_t ec600n_http_post(ec600n_device_t *dev, const char *url, const char *content_type, const char *body, char *response, uint16_t max_len, uint32_t timeout_ms);

/* SMS Text API */
ec600n_err_t ec600n_sms_send(ec600n_device_t *dev, const char *phone_number, const char *message);
ec600n_err_t ec600n_sms_read(ec600n_device_t *dev, uint8_t index, ec600n_sms_t *sms);
ec600n_err_t ec600n_sms_delete(ec600n_device_t *dev, uint8_t index);
void         ec600n_set_sms_recv_cb(ec600n_device_t *dev, ec600n_sms_recv_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* __EC600N_H__ */
