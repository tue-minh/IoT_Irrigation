/**
  ******************************************************************************
  * @file    ec600n.c
  * @brief   Implementation of high-level API for Quectel EC600N 4G LTE Module.
  ******************************************************************************
  */

#include "ec600n.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Global Device Instance */
ec600n_device_t g_ec600n_dev;

/* -------------------------------------------------------------------------- */
/* Static URC Dispatchers                                                     */
/* -------------------------------------------------------------------------- */
static void urc_tcp_recv(const char *line, void *user_data)
{
    ec600n_device_t *dev = (ec600n_device_t *)user_data;
    if (dev == NULL || dev->tcp_recv_cb == NULL) return;

    /* Example line: +QIURC: "recv",<connectID>,<len> */
    uint8_t id = 0;
    uint16_t len = 0;
    if (sscanf(line, "+QIURC: \"recv\",%hhu,%hu", &id, &len) == 2) {
        dev->tcp_recv_cb(id, NULL, len);
    }
}

static void urc_tcp_closed(const char *line, void *user_data)
{
    ec600n_device_t *dev = (ec600n_device_t *)user_data;
    if (dev == NULL || dev->tcp_closed_cb == NULL) return;

    /* Example line: +QIURC: "closed",<connectID> */
    uint8_t id = 0;
    if (sscanf(line, "+QIURC: \"closed\",%hhu", &id) == 1) {
        dev->tcp_closed_cb(id);
    }
}

static void urc_mqtt_recv(const char *line, void *user_data)
{
    ec600n_device_t *dev = (ec600n_device_t *)user_data;
    if (dev == NULL || dev->mqtt_recv_cb == NULL) return;

    /* Example: +QMTRECV: <client_idx>,<msgid>,"<topic>",<payload_len>,"<payload>" */
    char topic[64] = {0};

    const char *first_quote = strchr(line, '\"');
    if (first_quote) {
        const char *second_quote = strchr(first_quote + 1, '\"');
        if (second_quote) {
            size_t topic_len = second_quote - (first_quote + 1);
            if (topic_len < sizeof(topic)) {
                strncpy(topic, first_quote + 1, topic_len);
                topic[topic_len] = '\0';

                const char *third_quote = strchr(second_quote + 1, '\"');
                if (third_quote) {
                    const char *fourth_quote = strrchr(third_quote + 1, '\"');
                    if (fourth_quote) {
                        size_t p_len = fourth_quote - (third_quote + 1);
                        dev->mqtt_recv_cb(topic, (const uint8_t *)(third_quote + 1), (uint16_t)p_len);
                        return;
                    }
                }
            }
        }
    }
}

static void urc_sms_incoming(const char *line, void *user_data)
{
    ec600n_device_t *dev = (ec600n_device_t *)user_data;
    if (dev == NULL || dev->sms_recv_cb == NULL) return;

    /* Example: +CMTI: "SM",<index> */
    uint8_t index = 0;
    const char *comma = strchr(line, ',');
    if (comma) {
        index = (uint8_t)atoi(comma + 1);
        dev->sms_recv_cb(index);
    }
}

static void urc_sim_state(const char *line, void *user_data)
{
    ec600n_device_t *dev = (ec600n_device_t *)user_data;
    if (dev == NULL) return;

    if (strstr(line, "READY") != NULL) {
        dev->sim_status = EC600N_SIM_READY;
        ec600n_log("[EC600N] SIM Status: READY\r\n");
    }
}

static void urc_cereg_state(const char *line, void *user_data)
{
    ec600n_device_t *dev = (ec600n_device_t *)user_data;
    if (dev == NULL) return;

    /* Example: +CEREG: <stat> or +CEREG: <n>,<stat> */
    int stat = -1;
    const char *comma = strchr(line, ',');
    if (comma) {
        stat = atoi(comma + 1);
    } else {
        const char *colon = strchr(line, ':');
        if (colon) stat = atoi(colon + 1);
    }

    if (stat == 1) {
        dev->net_info.status = EC600N_NET_REGISTERED_HOME;
        ec600n_log("[EC600N] Network: Registered (Home)\r\n");
    } else if (stat == 5) {
        dev->net_info.status = EC600N_NET_REGISTERED_ROAMING;
        ec600n_log("[EC600N] Network: Registered (Roaming)\r\n");
    }
}

/* -------------------------------------------------------------------------- */
/* Initialization and Hardware Pins                                           */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_init(ec600n_device_t *dev, UART_HandleTypeDef *huart)
{
    if (dev == NULL || huart == NULL) {
        return EC600N_ERR_PARAM;
    }

    memset(dev, 0, sizeof(ec600n_device_t));

    ec600n_err_t ret = ec600n_at_init(&dev->at, huart);
    if (ret != EC600N_OK) {
        return ret;
    }

    /* Register built-in URC handlers */
    ec600n_at_register_urc(&dev->at, "+QIURC: \"recv\"", urc_tcp_recv, dev);
    ec600n_at_register_urc(&dev->at, "+QIURC: \"closed\"", urc_tcp_closed, dev);
    ec600n_at_register_urc(&dev->at, "+QMTRECV:", urc_mqtt_recv, dev);
    ec600n_at_register_urc(&dev->at, "+CMTI:", urc_sms_incoming, dev);
    ec600n_at_register_urc(&dev->at, "+CPIN:", urc_sim_state, dev);
    ec600n_at_register_urc(&dev->at, "+CEREG:", urc_cereg_state, dev);

    return EC600N_OK;
}

void ec600n_set_hw_pins(ec600n_device_t *dev,
                        GPIO_TypeDef *pwrkey_port, uint16_t pwrkey_pin,
                        GPIO_TypeDef *reset_port,  uint16_t reset_pin)
{
    if (dev == NULL) return;
    dev->pins.pwrkey_port = pwrkey_port;
    dev->pins.pwrkey_pin  = pwrkey_pin;
    dev->pins.reset_port  = reset_port;
    dev->pins.reset_pin   = reset_pin;
}

void ec600n_process(ec600n_device_t *dev)
{
    if (dev == NULL) return;
    ec600n_at_process(&dev->at);
}

/* -------------------------------------------------------------------------- */
/* Basic Control                                                              */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_check_communication(ec600n_device_t *dev)
{
    if (dev == NULL) return EC600N_ERR_PARAM;

    /* Try AT up to 5 times (module may be auto-bauding or waking up) */
    for (int i = 0; i < 5; i++) {
        if (ec600n_at_send_cmd(&dev->at, "AT", "OK", NULL, 0, 500) == EC600N_OK) {
            return EC600N_OK;
        }
        HAL_Delay(200);
    }
    return EC600N_ERR_TIMEOUT;
}

ec600n_err_t ec600n_echo(ec600n_device_t *dev, bool enable)
{
    if (dev == NULL) return EC600N_ERR_PARAM;
    return ec600n_at_send_cmd(&dev->at, enable ? "ATE1" : "ATE0", "OK", NULL, 0, 1000);
}

ec600n_err_t ec600n_get_device_info(ec600n_device_t *dev)
{
    if (dev == NULL) return EC600N_ERR_PARAM;

    char resp[128];

    /* 1. Model & Revision */
    if (ec600n_at_send_cmd(&dev->at, "ATI", "OK", resp, sizeof(resp), 1000) == EC600N_OK) {
        char *line = strtok(resp, "\r\n");
        while (line) {
            if (strstr(line, "Quectel") || strstr(line, "EC600")) {
                strncpy(dev->info.model, line, sizeof(dev->info.model) - 1);
            } else if (strncmp(line, "Revision:", 9) == 0) {
                strncpy(dev->info.fw_rev, line + 10, sizeof(dev->info.fw_rev) - 1);
            }
            line = strtok(NULL, "\r\n");
        }
    }

    /* 2. IMEI */
    if (ec600n_at_send_cmd(&dev->at, "AT+GSN", "OK", resp, sizeof(resp), 1000) == EC600N_OK) {
        char *p = strtok(resp, "\r\n");
        while (p) {
            if (strlen(p) >= 14 && strlen(p) <= 17 && strspn(p, "0123456789") == strlen(p)) {
                strncpy(dev->info.imei, p, sizeof(dev->info.imei) - 1);
                break;
            }
            p = strtok(NULL, "\r\n");
        }
    }

    /* 3. IMSI */
    if (ec600n_at_send_cmd(&dev->at, "AT+CIMI", "OK", resp, sizeof(resp), 1000) == EC600N_OK) {
        char *p = strtok(resp, "\r\n");
        while (p) {
            if (strlen(p) >= 14 && strlen(p) <= 16 && strspn(p, "0123456789") == strlen(p)) {
                strncpy(dev->info.imsi, p, sizeof(dev->info.imsi) - 1);
                break;
            }
            p = strtok(NULL, "\r\n");
        }
    }

    /* 4. ICCID */
    ec600n_get_iccid(dev, dev->info.iccid, sizeof(dev->info.iccid));

    ec600n_log("[EC600N] Model: %s | FW: %s | IMEI: %s\r\n",
               dev->info.model, dev->info.fw_rev, dev->info.imei);

    return EC600N_OK;
}

ec600n_err_t ec600n_power_down(ec600n_device_t *dev)
{
    if (dev == NULL) return EC600N_ERR_PARAM;
    return ec600n_at_send_cmd(&dev->at, "AT+QPOWD=1", "OK", NULL, 0, 5000);
}

ec600n_err_t ec600n_restart(ec600n_device_t *dev)
{
    if (dev == NULL) return EC600N_ERR_PARAM;
    return ec600n_at_send_cmd(&dev->at, "AT+CFUN=1,1", "OK", NULL, 0, 5000);
}

/* -------------------------------------------------------------------------- */
/* SIM Card Management                                                        */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_check_sim(ec600n_device_t *dev)
{
    if (dev == NULL) return EC600N_ERR_PARAM;

    char resp[64];
    ec600n_err_t ret = ec600n_at_send_cmd(&dev->at, "AT+CPIN?", "OK", resp, sizeof(resp), 2000);
    if (ret != EC600N_OK) {
        dev->sim_status = EC600N_SIM_ERROR;
        return ret;
    }

    if (strstr(resp, "+CPIN: READY")) {
        dev->sim_status = EC600N_SIM_READY;
    } else if (strstr(resp, "SIM PIN")) {
        dev->sim_status = EC600N_SIM_PIN_REQUIRED;
    } else if (strstr(resp, "SIM PUK")) {
        dev->sim_status = EC600N_SIM_PUK_REQUIRED;
    } else {
        dev->sim_status = EC600N_SIM_NOT_INSERTED;
    }

    return (dev->sim_status == EC600N_SIM_READY) ? EC600N_OK : EC600N_ERR_SIM;
}

ec600n_err_t ec600n_get_iccid(ec600n_device_t *dev, char *iccid, uint8_t max_len)
{
    if (dev == NULL || iccid == NULL || max_len == 0) return EC600N_ERR_PARAM;

    char resp[64];
    if (ec600n_at_send_cmd(&dev->at, "AT+QCCID", "OK", resp, sizeof(resp), 1000) == EC600N_OK) {
        char *p = strstr(resp, "+QCCID: ");
        if (p) {
            p += 8;
            char *end = strpbrk(p, "\r\n");
            if (end) *end = '\0';
            strncpy(iccid, p, max_len - 1);
            iccid[max_len - 1] = '\0';
            return EC600N_OK;
        }
    }
    return EC600N_ERR_SIM;
}

/* -------------------------------------------------------------------------- */
/* Signal Quality & Network Status                                            */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_get_signal(ec600n_device_t *dev, ec600n_signal_t *signal)
{
    if (dev == NULL) return EC600N_ERR_PARAM;

    char resp[64];
    ec600n_err_t ret = ec600n_at_send_cmd(&dev->at, "AT+CSQ", "OK", resp, sizeof(resp), 1000);
    if (ret != EC600N_OK) return ret;

    char *p = strstr(resp, "+CSQ: ");
    if (!p) return EC600N_ERR_AT_ERROR;

    uint8_t rssi = 99, ber = 99;
    if (sscanf(p + 6, "%hhu,%hhu", &rssi, &ber) >= 1) {
        dev->net_info.signal.rssi = rssi;
        dev->net_info.signal.ber  = ber;

        if (rssi == 99) {
            dev->net_info.signal.rssi_dbm = 0;
            dev->net_info.signal.bars = 0;
        } else {
            dev->net_info.signal.rssi_dbm = -113 + (rssi * 2);
            if (rssi >= 20)      dev->net_info.signal.bars = 5;
            else if (rssi >= 15) dev->net_info.signal.bars = 4;
            else if (rssi >= 10) dev->net_info.signal.bars = 3;
            else if (rssi >= 5)  dev->net_info.signal.bars = 2;
            else if (rssi > 0)   dev->net_info.signal.bars = 1;
            else                 dev->net_info.signal.bars = 0;
        }

        if (signal) {
            *signal = dev->net_info.signal;
        }
        return EC600N_OK;
    }
    return EC600N_ERR_AT_ERROR;
}

ec600n_err_t ec600n_get_network_status(ec600n_device_t *dev, ec600n_net_stat_t *stat)
{
    if (dev == NULL) return EC600N_ERR_PARAM;

    char resp[64];
    /* Query LTE EPS Registration */
    if (ec600n_at_send_cmd(&dev->at, "AT+CEREG?", "OK", resp, sizeof(resp), 1000) == EC600N_OK) {
        char *p = strstr(resp, "+CEREG: ");
        if (p) {
            int n = 0, s = 0;
            if (sscanf(p + 8, "%d,%d", &n, &s) == 2 || sscanf(p + 8, "%d", &s) == 1) {
                switch (s) {
                    case 1: dev->net_info.status = EC600N_NET_REGISTERED_HOME; break;
                    case 2: dev->net_info.status = EC600N_NET_SEARCHING; break;
                    case 3: dev->net_info.status = EC600N_NET_DENIED; break;
                    case 5: dev->net_info.status = EC600N_NET_REGISTERED_ROAMING; break;
                    default: dev->net_info.status = EC600N_NET_NOT_REGISTERED; break;
                }
                if (stat) *stat = dev->net_info.status;
                return EC600N_OK;
            }
        }
    }
    return EC600N_ERR_NET;
}

ec600n_err_t ec600n_wait_network_registered(ec600n_device_t *dev, uint32_t timeout_ms)
{
    if (dev == NULL) return EC600N_ERR_PARAM;

    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout_ms) {
        ec600n_net_stat_t stat = EC600N_NET_NOT_REGISTERED;
        if (ec600n_get_network_status(dev, &stat) == EC600N_OK) {
            if (stat == EC600N_NET_REGISTERED_HOME || stat == EC600N_NET_REGISTERED_ROAMING) {
                ec600n_get_signal(dev, NULL);
                ec600n_get_operator(dev, dev->net_info.operator_name, sizeof(dev->net_info.operator_name));
                return EC600N_OK;
            }
        }
        HAL_Delay(1000);
    }
    return EC600N_ERR_TIMEOUT;
}

ec600n_err_t ec600n_get_operator(ec600n_device_t *dev, char *operator_name, uint8_t max_len)
{
    if (dev == NULL || operator_name == NULL || max_len == 0) return EC600N_ERR_PARAM;

    char resp[64];
    if (ec600n_at_send_cmd(&dev->at, "AT+COPS?", "OK", resp, sizeof(resp), 3000) == EC600N_OK) {
        char *first_quote = strchr(resp, '\"');
        if (first_quote) {
            char *second_quote = strchr(first_quote + 1, '\"');
            if (second_quote) {
                size_t len = second_quote - (first_quote + 1);
                if (len >= max_len) len = max_len - 1;
                strncpy(operator_name, first_quote + 1, len);
                operator_name[len] = '\0';
                return EC600N_OK;
            }
        }
    }
    return EC600N_ERR_NET;
}

/* -------------------------------------------------------------------------- */
/* PDP Context & 4G Internet                                                  */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_activate_pdp(ec600n_device_t *dev, const char *apn, const char *user, const char *pwd)
{
    if (dev == NULL) return EC600N_ERR_PARAM;

    char cmd[128];

    /* Deactivate context 1 if previously active */
    ec600n_at_send_cmd(&dev->at, "AT+QIDEACT=1", "OK", NULL, 0, 5000);

    /* Configure PDP context 1 with APN */
    snprintf(cmd, sizeof(cmd), "AT+QICSGP=1,1,\"%s\",\"%s\",\"%s\",1",
             (apn && strlen(apn) > 0) ? apn : "v-internet",
             user ? user : "",
             pwd ? pwd : "");

    if (ec600n_at_send_cmd(&dev->at, cmd, "OK", NULL, 0, 3000) != EC600N_OK) {
        return EC600N_ERR_NET;
    }

    /* Activate context 1 */
    ec600n_err_t ret = ec600n_at_send_cmd(&dev->at, "AT+QIACT=1", "OK", NULL, 0, EC600N_NETWORK_ACT_TIMEOUT_MS);
    if (ret != EC600N_OK) {
        return ret;
    }

    /* Read assigned IP address */
    ec600n_get_ip(dev, dev->net_info.ip_address, sizeof(dev->net_info.ip_address));
    ec600n_log("[EC600N] PDP Active! Local IP: %s\r\n", dev->net_info.ip_address);

    return EC600N_OK;
}

ec600n_err_t ec600n_deactivate_pdp(ec600n_device_t *dev)
{
    if (dev == NULL) return EC600N_ERR_PARAM;
    return ec600n_at_send_cmd(&dev->at, "AT+QIDEACT=1", "OK", NULL, 0, 10000);
}

ec600n_err_t ec600n_get_ip(ec600n_device_t *dev, char *ip_buf, uint8_t max_len)
{
    if (dev == NULL || ip_buf == NULL || max_len == 0) return EC600N_ERR_PARAM;

    char resp[128];
    if (ec600n_at_send_cmd(&dev->at, "AT+QIACT?", "OK", resp, sizeof(resp), 3000) == EC600N_OK) {
        /* Format: +QIACT: 1,1,1,"100.75.22.45" */
        char *first_quote = strchr(resp, '\"');
        if (first_quote) {
            char *second_quote = strchr(first_quote + 1, '\"');
            if (second_quote) {
                size_t len = second_quote - (first_quote + 1);
                if (len >= max_len) len = max_len - 1;
                strncpy(ip_buf, first_quote + 1, len);
                ip_buf[len] = '\0';
                return EC600N_OK;
            }
        }
    }
    return EC600N_ERR_NET;
}

ec600n_err_t ec600n_ping(ec600n_device_t *dev, const char *host, uint32_t *rtt_ms)
{
    if (dev == NULL || host == NULL) return EC600N_ERR_PARAM;

    char cmd[64];
    char resp[128];
    snprintf(cmd, sizeof(cmd), "AT+QPING=1,\"%s\",4,1", host);

    if (ec600n_at_send_cmd(&dev->at, cmd, "OK", resp, sizeof(resp), 10000) == EC600N_OK) {
        /* Look for +QPING: 0,"x.x.x.x",32,<rtt>,... */
        char *p = strstr(resp, "+QPING: ");
        if (p) {
            int err = -1, bytes = 0, rtt = 0;
            char ip[32] = {0};
            if (sscanf(p + 8, "%d,\"%[^\"]\",%d,%d", &err, ip, &bytes, &rtt) >= 4 && err == 0) {
                if (rtt_ms) *rtt_ms = (uint32_t)rtt;
                return EC600N_OK;
            }
        }
    }
    return EC600N_ERR_NET;
}

/* -------------------------------------------------------------------------- */
/* Date, Time & NTP                                                           */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_get_time(ec600n_device_t *dev, ec600n_time_t *time)
{
    if (dev == NULL || time == NULL) return EC600N_ERR_PARAM;

    char resp[64];
    if (ec600n_at_send_cmd(&dev->at, "AT+CCLK?", "OK", resp, sizeof(resp), 1000) == EC600N_OK) {
        /* Format: +CCLK: "yy/MM/dd,hh:mm:ss±zz" */
        char *p = strchr(resp, '\"');
        if (p) {
            int y, m, d, hh, mm, ss, tz;
            char sign;
            if (sscanf(p + 1, "%d/%d/%d,%d:%d:%d%c%d", &y, &m, &d, &hh, &mm, &ss, &sign, &tz) >= 7) {
                time->year   = 2000 + y;
                time->month  = (uint8_t)m;
                time->day    = (uint8_t)d;
                time->hour   = (uint8_t)hh;
                time->minute = (uint8_t)mm;
                time->second = (uint8_t)ss;
                time->tz_quarters = (sign == '-') ? -tz : tz;
                return EC600N_OK;
            }
        }
    }
    return EC600N_ERR_AT_ERROR;
}

ec600n_err_t ec600n_ntp_sync(ec600n_device_t *dev, const char *ntp_server)
{
    if (dev == NULL) return EC600N_ERR_PARAM;
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+QNTP=1,\"%s\",123", ntp_server ? ntp_server : "pool.ntp.org");
    return ec600n_at_send_cmd(&dev->at, cmd, "OK", NULL, 0, 15000);
}

/* -------------------------------------------------------------------------- */
/* TCP Socket API                                                             */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_tcp_connect(ec600n_device_t *dev, uint8_t connect_id, const char *host, uint16_t port)
{
    if (dev == NULL || host == NULL) return EC600N_ERR_PARAM;

    char cmd[96];
    char resp[64];
    snprintf(cmd, sizeof(cmd), "AT+QIOPEN=1,%hhu,\"TCP\",\"%s\",%hu,0,0", connect_id, host, port);

    ec600n_err_t ret = ec600n_at_send_cmd(&dev->at, cmd, "OK", resp, sizeof(resp), 15000);
    return ret;
}

ec600n_err_t ec600n_tcp_send(ec600n_device_t *dev, uint8_t connect_id, const uint8_t *data, uint16_t len)
{
    if (dev == NULL || data == NULL || len == 0) return EC600N_ERR_PARAM;

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+QISEND=%hhu,%hu", connect_id, len);

    /* Wait for prompt '>' */
    if (ec600n_at_send_cmd(&dev->at, cmd, ">", NULL, 0, 5000) != EC600N_OK) {
        return EC600N_ERR_SOCKET;
    }

    /* Send raw data */
    return ec600n_at_send_raw(&dev->at, data, len, 5000);
}

ec600n_err_t ec600n_tcp_read(ec600n_device_t *dev, uint8_t connect_id, uint8_t *buf, uint16_t max_len, uint16_t *read_len)
{
    if (dev == NULL || buf == NULL || max_len == 0) return EC600N_ERR_PARAM;

    char cmd[32];
    char resp[EC600N_CMD_RESP_BUFFER_SIZE];
    snprintf(cmd, sizeof(cmd), "AT+QIRD=%hhu,%hu", connect_id, max_len);

    if (ec600n_at_send_cmd(&dev->at, cmd, "OK", resp, sizeof(resp), 5000) == EC600N_OK) {
        char *p = strstr(resp, "+QIRD: ");
        if (p) {
            int actual = 0;
            if (sscanf(p + 7, "%d", &actual) == 1 && actual > 0) {
                char *payload_start = strstr(p, "\r\n");
                if (payload_start) {
                    payload_start += 2;
                    uint16_t cpy = (uint16_t)actual < max_len ? (uint16_t)actual : max_len;
                    memcpy(buf, payload_start, cpy);
                    if (read_len) *read_len = cpy;
                    return EC600N_OK;
                }
            }
        }
    }
    return EC600N_ERR_SOCKET;
}

ec600n_err_t ec600n_tcp_close(ec600n_device_t *dev, uint8_t connect_id)
{
    if (dev == NULL) return EC600N_ERR_PARAM;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+QICLOSE=%hhu,10", connect_id);
    return ec600n_at_send_cmd(&dev->at, cmd, "OK", NULL, 0, 5000);
}

void ec600n_set_tcp_recv_cb(ec600n_device_t *dev, ec600n_tcp_recv_cb_t cb)
{
    if (dev) dev->tcp_recv_cb = cb;
}

void ec600n_set_tcp_closed_cb(ec600n_device_t *dev, ec600n_tcp_closed_cb_t cb)
{
    if (dev) dev->tcp_closed_cb = cb;
}

/* -------------------------------------------------------------------------- */
/* MQTT Client API (Quectel Hardware Stack)                                   */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_mqtt_connect(ec600n_device_t *dev, const ec600n_mqtt_cfg_t *cfg)
{
    if (dev == NULL || cfg == NULL || cfg->broker == NULL) return EC600N_ERR_PARAM;

    char cmd[128];
    char resp[64];

    /* Configure MQTT version 3.1.1 */
    ec600n_at_send_cmd(&dev->at, "AT+QMTCFG=\"version\",0,4", "OK", NULL, 0, 2000);

    /* Open network connection to broker */
    snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=0,\"%s\",%hu", cfg->broker, cfg->port);
    if (ec600n_at_send_cmd(&dev->at, cmd, "OK", resp, sizeof(resp), EC600N_MQTT_TIMEOUT_MS) != EC600N_OK) {
        return EC600N_ERR_MQTT;
    }

    /* Wait for URC network open indication: +QMTOPEN: 0,0 */
    uint32_t start = HAL_GetTick();
    bool open_ok = false;
    while (HAL_GetTick() - start < EC600N_MQTT_TIMEOUT_MS) {
        ec600n_process(dev);
        if (strstr(dev->at.line_buf, "+QMTOPEN: 0,0") || strstr(resp, "+QMTOPEN: 0,0")) {
            open_ok = true;
            break;
        }
        HAL_Delay(50);
    }

    if (!open_ok) {
        return EC600N_ERR_MQTT;
    }

    /* Connect client */
    if (cfg->username && strlen(cfg->username) > 0) {
        snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
                 cfg->client_id ? cfg->client_id : "STM32_Irrigation",
                 cfg->username,
                 cfg->password ? cfg->password : "");
    } else {
        snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\"",
                 cfg->client_id ? cfg->client_id : "STM32_Irrigation");
    }

    return ec600n_at_send_cmd(&dev->at, cmd, "OK", NULL, 0, EC600N_MQTT_TIMEOUT_MS);
}

ec600n_err_t ec600n_mqtt_disconnect(ec600n_device_t *dev)
{
    if (dev == NULL) return EC600N_ERR_PARAM;
    ec600n_at_send_cmd(&dev->at, "AT+QMTDISC=0", "OK", NULL, 0, 5000);
    return ec600n_at_send_cmd(&dev->at, "AT+QMTCLOSE=0", "OK", NULL, 0, 5000);
}

ec600n_err_t ec600n_mqtt_publish(ec600n_device_t *dev, const char *topic, const char *payload, uint8_t qos, uint8_t retain)
{
    if (dev == NULL || topic == NULL || payload == NULL) return EC600N_ERR_PARAM;

    char cmd[128];
    uint16_t len = (uint16_t)strlen(payload);
    snprintf(cmd, sizeof(cmd), "AT+QMTPUBEX=0,0,%hhu,%hhu,\"%s\",%hu", qos, retain, topic, len);

    /* Wait for prompt '>' */
    if (ec600n_at_send_cmd(&dev->at, cmd, ">", NULL, 0, 5000) != EC600N_OK) {
        return EC600N_ERR_MQTT;
    }

    /* Send payload */
    return ec600n_at_send_raw(&dev->at, (const uint8_t *)payload, len, 5000);
}

ec600n_err_t ec600n_mqtt_subscribe(ec600n_device_t *dev, const char *topic, uint8_t qos)
{
    if (dev == NULL || topic == NULL) return EC600N_ERR_PARAM;
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+QMTSUB=0,1,\"%s\",%hhu", topic, qos);
    return ec600n_at_send_cmd(&dev->at, cmd, "OK", NULL, 0, 5000);
}

void ec600n_set_mqtt_recv_cb(ec600n_device_t *dev, ec600n_mqtt_recv_cb_t cb)
{
    if (dev) dev->mqtt_recv_cb = cb;
}

/* -------------------------------------------------------------------------- */
/* HTTP Client API                                                            */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_http_get(ec600n_device_t *dev, const char *url, char *response, uint16_t max_len, uint32_t timeout_ms)
{
    if (dev == NULL || url == NULL) return EC600N_ERR_PARAM;

    char cmd[64];
    ec600n_at_send_cmd(&dev->at, "AT+QHTTPCFG=\"contextid\",1", "OK", NULL, 0, 2000);
    ec600n_at_send_cmd(&dev->at, "AT+QHTTPCFG=\"responseheader\",0", "OK", NULL, 0, 2000);

    /* Set URL */
    uint16_t url_len = (uint16_t)strlen(url);
    snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%hu,60", url_len);
    if (ec600n_at_send_cmd(&dev->at, cmd, "CONNECT", NULL, 0, 5000) != EC600N_OK) {
        return EC600N_ERR_HTTP;
    }

    /* Send URL string */
    if (ec600n_at_send_raw(&dev->at, (const uint8_t *)url, url_len, 5000) != EC600N_OK) {
        return EC600N_ERR_HTTP;
    }

    /* Execute GET */
    if (ec600n_at_send_cmd(&dev->at, "AT+QHTTPGET=60", "OK", NULL, 0, timeout_ms) != EC600N_OK) {
        return EC600N_ERR_HTTP;
    }

    /* Read content */
    if (response && max_len > 0) {
        return ec600n_at_send_cmd(&dev->at, "AT+QHTTPREAD=60", "OK", response, max_len, timeout_ms);
    }
    return EC600N_OK;
}

ec600n_err_t ec600n_http_post(ec600n_device_t *dev, const char *url, const char *content_type, const char *body, char *response, uint16_t max_len, uint32_t timeout_ms)
{
    if (dev == NULL || url == NULL || body == NULL) return EC600N_ERR_PARAM;

    char cmd[64];
    ec600n_at_send_cmd(&dev->at, "AT+QHTTPCFG=\"contextid\",1", "OK", NULL, 0, 2000);

    /* Set URL */
    uint16_t url_len = (uint16_t)strlen(url);
    snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%hu,60", url_len);
    if (ec600n_at_send_cmd(&dev->at, cmd, "CONNECT", NULL, 0, 5000) != EC600N_OK) {
        return EC600N_ERR_HTTP;
    }
    if (ec600n_at_send_raw(&dev->at, (const uint8_t *)url, url_len, 5000) != EC600N_OK) {
        return EC600N_ERR_HTTP;
    }

    /* POST request */
    uint16_t body_len = (uint16_t)strlen(body);
    snprintf(cmd, sizeof(cmd), "AT+QHTTPPOST=%hu,60,60", body_len);
    if (ec600n_at_send_cmd(&dev->at, cmd, "CONNECT", NULL, 0, 5000) != EC600N_OK) {
        return EC600N_ERR_HTTP;
    }

    if (ec600n_at_send_raw(&dev->at, (const uint8_t *)body, body_len, 5000) != EC600N_OK) {
        return EC600N_ERR_HTTP;
    }

    /* Read response */
    if (response && max_len > 0) {
        return ec600n_at_send_cmd(&dev->at, "AT+QHTTPREAD=60", "OK", response, max_len, timeout_ms);
    }
    return EC600N_OK;
}

/* -------------------------------------------------------------------------- */
/* SMS Text API                                                               */
/* -------------------------------------------------------------------------- */
ec600n_err_t ec600n_sms_send(ec600n_device_t *dev, const char *phone_number, const char *message)
{
    if (dev == NULL || phone_number == NULL || message == NULL) return EC600N_ERR_PARAM;

    char cmd[64];
    /* Configure Text Mode */
    ec600n_at_send_cmd(&dev->at, "AT+CMGF=1", "OK", NULL, 0, 2000);
    ec600n_at_send_cmd(&dev->at, "AT+CSCS=\"GSM\"", "OK", NULL, 0, 2000);

    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", phone_number);

    /* Wait for prompt '>' */
    if (ec600n_at_send_cmd(&dev->at, cmd, ">", NULL, 0, 5000) != EC600N_OK) {
        return EC600N_ERR_AT_ERROR;
    }

    /* Send message content followed by Ctrl+Z (0x1A) */
    char full_msg[200];
    int len = snprintf(full_msg, sizeof(full_msg), "%s\x1A", message);
    if (len <= 0) return EC600N_ERR_PARAM;

    return ec600n_at_send_cmd(&dev->at, full_msg, "+CMGS:", NULL, 0, EC600N_SMS_TIMEOUT_MS);
}

ec600n_err_t ec600n_sms_read(ec600n_device_t *dev, uint8_t index, ec600n_sms_t *sms)
{
    if (dev == NULL || sms == NULL) return EC600N_ERR_PARAM;

    char cmd[32];
    char resp[256];
    snprintf(cmd, sizeof(cmd), "AT+CMGR=%hhu", index);

    if (ec600n_at_send_cmd(&dev->at, cmd, "OK", resp, sizeof(resp), 5000) == EC600N_OK) {
        /* Format: +CMGR: "REC UNREAD","+849xxxxxx",,"26/11/11,13:30:00+28"\r\nMessage Text\r\nOK */
        char *p = strstr(resp, "+CMGR: ");
        if (p) {
            sms->index = index;
            char *line1 = strtok(p, "\r\n");
            (void)line1;
            char *body = strtok(NULL, "\r\n");
            if (body) {
                strncpy(sms->text, body, sizeof(sms->text) - 1);
                sms->text[sizeof(sms->text) - 1] = '\0';
                return EC600N_OK;
            }
        }
    }
    return EC600N_ERR_AT_ERROR;
}

ec600n_err_t ec600n_sms_delete(ec600n_device_t *dev, uint8_t index)
{
    if (dev == NULL) return EC600N_ERR_PARAM;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CMGD=%hhu", index);
    return ec600n_at_send_cmd(&dev->at, cmd, "OK", NULL, 0, 5000);
}

void ec600n_set_sms_recv_cb(ec600n_device_t *dev, ec600n_sms_recv_cb_t cb)
{
    if (dev) dev->sms_recv_cb = cb;
}
