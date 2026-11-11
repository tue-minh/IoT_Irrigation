/**
  ******************************************************************************
  * @file    irrigation_ctrl.c
  * @brief   Implementation of Irrigation Actuator Controller and REST API Parser.
  ******************************************************************************
  */

#include "irrigation_ctrl.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

/* Current hardware state cache */
static irrigation_state_t s_current_state = {
    .pump   = false,
    .valve1 = false,
    .valve2 = false,
    .valve3 = false
};

/* -------------------------------------------------------------------------- */
/* Safety Helper Functions                                                    */
/* -------------------------------------------------------------------------- */
uint8_t irrigation_get_open_valves_count(bool v1, bool v2, bool v3)
{
    uint8_t count = 0;
    if (v1) count++;
    if (v2) count++;
    if (v3) count++;
    return count;
}

bool irrigation_is_pump_allowed(bool v1, bool v2, bool v3)
{
    /* Motor pump is ONLY allowed if 1, 2, or 3 valves are open */
    uint8_t count = irrigation_get_open_valves_count(v1, v2, v3);
    return (count >= 1 && count <= 3);
}

/* -------------------------------------------------------------------------- */
/* Hardware Actuator Control                                                  */
/* -------------------------------------------------------------------------- */
void irrigation_init(void)
{
    /* Initialize all outputs to LOW (all closed/off) */
    HAL_GPIO_WritePin(motor_GPIO_Port, motor_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(valve1_GPIO_Port, valve1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(valve2_GPIO_Port, valve2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(valve3_GPIO_Port, valve3_Pin, GPIO_PIN_RESET);

    s_current_state.pump   = false;
    s_current_state.valve1 = false;
    s_current_state.valve2 = false;
    s_current_state.valve3 = false;

    ec600n_log("[IRRIGATION] Actuators initialized: Pump(PB4)=OFF, Valves(PB5,PB8,PB9)=OFF\r\n");
}

void irrigation_emergency_stop(void)
{
    /* Force pump off FIRST */
    HAL_GPIO_WritePin(motor_GPIO_Port, motor_Pin, GPIO_PIN_RESET);
    s_current_state.pump = false;

    /* Close all valves */
    HAL_GPIO_WritePin(valve1_GPIO_Port, valve1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(valve2_GPIO_Port, valve2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(valve3_GPIO_Port, valve3_Pin, GPIO_PIN_RESET);
    s_current_state.valve1 = false;
    s_current_state.valve2 = false;
    s_current_state.valve3 = false;

    ec600n_log("[IRRIGATION] EMERGENCY STOP ACTIVATED!\r\n");
}

irrigation_status_t irrigation_apply_state(const irrigation_state_t *target, irrigation_state_t *actual)
{
    if (target == NULL) return IRRIGATION_ERR_PARAM;

    bool target_pump   = target->pump;
    bool target_valve1 = target->valve1;
    bool target_valve2 = target->valve2;
    bool target_valve3 = target->valve3;
    bool interlock_tripped = false;

    /* SAFETY INTERLOCK ENFORCEMENT:
     * Check if pump is requested with ZERO valves open.
     */
    if (target_pump && !irrigation_is_pump_allowed(target_valve1, target_valve2, target_valve3)) {
        target_pump = false;
        interlock_tripped = true;
        ec600n_log("[IRRIGATION SAFETY] Denied pump start: All valves are closed!\r\n");
    }

    /* Hydraulic Switching Sequence:
     * 1. If opening valves, open valves FIRST before starting pump
     */
    HAL_GPIO_WritePin(valve1_GPIO_Port, valve1_Pin, target_valve1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(valve2_GPIO_Port, valve2_Pin, target_valve2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(valve3_GPIO_Port, valve3_Pin, target_valve3 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 2. Update pump state */
    HAL_GPIO_WritePin(motor_GPIO_Port, motor_Pin, target_pump ? GPIO_PIN_SET : GPIO_PIN_RESET);

    s_current_state.pump   = target_pump;
    s_current_state.valve1 = target_valve1;
    s_current_state.valve2 = target_valve2;
    s_current_state.valve3 = target_valve3;

    if (actual) {
        *actual = s_current_state;
    }

    ec600n_log("[IRRIGATION] State Applied -> Pump(PB4):%d, V1(PB5):%d, V2(PB8):%d, V3(PB9):%d (OpenValves:%d)\r\n",
               s_current_state.pump, s_current_state.valve1, s_current_state.valve2, s_current_state.valve3,
               irrigation_get_open_valves_count(s_current_state.valve1, s_current_state.valve2, s_current_state.valve3));

    return interlock_tripped ? IRRIGATION_WARN_INTERLOCK_TRIPPED : IRRIGATION_OK;
}

irrigation_status_t irrigation_set_state(bool pump, bool v1, bool v2, bool v3)
{
    irrigation_state_t target = {
        .pump   = pump,
        .valve1 = v1,
        .valve2 = v2,
        .valve3 = v3
    };
    return irrigation_apply_state(&target, NULL);
}

void irrigation_get_state(irrigation_state_t *state)
{
    if (state) {
        *state = s_current_state;
    }
}

/* -------------------------------------------------------------------------- */
/* Lightweight JSON / REST API Parser Helpers                                 */
/* -------------------------------------------------------------------------- */
static bool parse_boolean_value(const char *str, bool *value_out)
{
    while (*str && (isspace((unsigned char)*str) || *str == ':' || *str == '=')) {
        str++;
    }

    if (*str == '1' || *str == 't' || *str == 'T' || *str == 'y' || *str == 'Y') {
        *value_out = true;
        return true;
    } else if (*str == '0' || *str == 'f' || *str == 'F' || *str == 'n' || *str == 'N') {
        *value_out = false;
        return true;
    }
    return false;
}

static const char* find_key(const char *src, const char *key)
{
    const char *pos = strstr(src, key);
    while (pos) {
        /* Check if key is preceded by start, quote, or delimiter */
        if (pos == src || *(pos - 1) == '\"' || *(pos - 1) == '?' || *(pos - 1) == '&' || isspace((unsigned char)*(pos - 1))) {
            const char *end_key = pos + strlen(key);
            if (*end_key == '\"') end_key++;
            while (*end_key && isspace((unsigned char)*end_key)) end_key++;
            if (*end_key == ':' || *end_key == '=') {
                return end_key + 1;
            }
        }
        pos = strstr(pos + 1, key);
    }
    return NULL;
}

/* -------------------------------------------------------------------------- */
/* REST API Payload Processing                                                */
/* -------------------------------------------------------------------------- */
irrigation_status_t irrigation_process_rest_payload(const char *payload,
                                                    char *json_resp_out,
                                                    uint16_t max_resp_len)
{
    if (payload == NULL) return IRRIGATION_ERR_PARAM;

    ec600n_log("[IRRIGATION REST] Received payload: %s\r\n", payload);

    /* Start with current state for partial updates */
    irrigation_state_t target = s_current_state;
    bool found_any_key = false;

    /* 1. Parse Pump / Motor */
    const char *p_pump = find_key(payload, "pump");
    if (!p_pump) p_pump = find_key(payload, "motor");
    if (p_pump) {
        bool val;
        if (parse_boolean_value(p_pump, &val)) {
            target.pump = val;
            found_any_key = true;
        }
    }

    /* 2. Parse Valve 1 */
    const char *p_v1 = find_key(payload, "valve1");
    if (!p_v1) p_v1 = find_key(payload, "v1");
    if (p_v1) {
        bool val;
        if (parse_boolean_value(p_v1, &val)) {
            target.valve1 = val;
            found_any_key = true;
        }
    }

    /* 3. Parse Valve 2 */
    const char *p_v2 = find_key(payload, "valve2");
    if (!p_v2) p_v2 = find_key(payload, "v2");
    if (p_v2) {
        bool val;
        if (parse_boolean_value(p_v2, &val)) {
            target.valve2 = val;
            found_any_key = true;
        }
    }

    /* 4. Parse Valve 3 */
    const char *p_v3 = find_key(payload, "valve3");
    if (!p_v3) p_v3 = find_key(payload, "v3");
    if (p_v3) {
        bool val;
        if (parse_boolean_value(p_v3, &val)) {
            target.valve3 = val;
            found_any_key = true;
        }
    }

    /* 5. Parse Array format: "valves": [1, 0, 1] */
    const char *p_valves = find_key(payload, "valves");
    if (p_valves) {
        const char *bracket = strchr(p_valves, '[');
        if (bracket) {
            int v1_int = 0, v2_int = 0, v3_int = 0;
            if (sscanf(bracket + 1, "%d,%d,%d", &v1_int, &v2_int, &v3_int) == 3) {
                target.valve1 = (v1_int != 0);
                target.valve2 = (v2_int != 0);
                target.valve3 = (v3_int != 0);
                found_any_key = true;
            }
        }
    }

    if (!found_any_key) {
        ec600n_log("[IRRIGATION REST] No recognizable control keys found in payload\r\n");
        if (json_resp_out && max_resp_len > 0) {
            snprintf(json_resp_out, max_resp_len,
                     "{\"status\":\"error\",\"message\":\"Invalid JSON/REST payload\"}");
        }
        return IRRIGATION_ERR_PARSE;
    }

    /* Apply target state through Safety Interlock */
    irrigation_state_t actual;
    irrigation_status_t status = irrigation_apply_state(&target, &actual);

    /* Construct REST JSON response */
    if (json_resp_out && max_resp_len > 0) {
        uint8_t open_count = irrigation_get_open_valves_count(actual.valve1, actual.valve2, actual.valve3);

        if (status == IRRIGATION_WARN_INTERLOCK_TRIPPED) {
            snprintf(json_resp_out, max_resp_len,
                     "{\"status\":\"warning\",\"error\":\"Safety Interlock: Pump denied because 0 valves are open\","
                     "\"pump\":%d,\"valve1\":%d,\"valve2\":%d,\"valve3\":%d,\"open_valves\":%d}",
                     actual.pump ? 1 : 0, actual.valve1 ? 1 : 0, actual.valve2 ? 1 : 0, actual.valve3 ? 1 : 0,
                     open_count);
        } else {
            snprintf(json_resp_out, max_resp_len,
                     "{\"status\":\"ok\",\"pump\":%d,\"valve1\":%d,\"valve2\":%d,\"valve3\":%d,\"open_valves\":%d}",
                     actual.pump ? 1 : 0, actual.valve1 ? 1 : 0, actual.valve2 ? 1 : 0, actual.valve3 ? 1 : 0,
                     open_count);
        }
    }

    return status;
}

/* -------------------------------------------------------------------------- */
/* Cloud / Backend REST API Integration                                       */
/* -------------------------------------------------------------------------- */
irrigation_status_t irrigation_poll_backend_api(ec600n_device_t *dev,
                                                const char *backend_url,
                                                char *resp_buf,
                                                uint16_t resp_max_len)
{
    if (dev == NULL || backend_url == NULL) return IRRIGATION_ERR_PARAM;

    char http_resp[512] = {0};
    char *buf = resp_buf ? resp_buf : http_resp;
    uint16_t max_len = resp_buf ? resp_max_len : sizeof(http_resp);

    ec600n_log("[IRRIGATION] Polling backend REST API: %s\r\n", backend_url);

    ec600n_err_t err = ec600n_http_get(dev, backend_url, buf, max_len, 10000);
    if (err != EC600N_OK) {
        ec600n_log("[IRRIGATION] HTTP GET failed (%d)\r\n", (int)err);
        return IRRIGATION_ERR_HTTP;
    }

    /* Process response payload */
    return irrigation_process_rest_payload(buf, NULL, 0);
}

irrigation_status_t irrigation_report_status_to_backend(ec600n_device_t *dev,
                                                        const char *backend_url,
                                                        const char *device_id)
{
    if (dev == NULL || backend_url == NULL) return IRRIGATION_ERR_PARAM;

    char json_body[256];
    char http_resp[128];

    uint8_t open_count = irrigation_get_open_valves_count(s_current_state.valve1,
                                                          s_current_state.valve2,
                                                          s_current_state.valve3);

    snprintf(json_body, sizeof(json_body),
             "{\"device_id\":\"%s\",\"pump\":%d,\"valve1\":%d,\"valve2\":%d,\"valve3\":%d,\"open_valves\":%d}",
             device_id ? device_id : "STM32_Irrigation_01",
             s_current_state.pump ? 1 : 0,
             s_current_state.valve1 ? 1 : 0,
             s_current_state.valve2 ? 1 : 0,
             s_current_state.valve3 ? 1 : 0,
             open_count);

    ec600n_log("[IRRIGATION] Reporting status to backend: %s\r\n", json_body);

    ec600n_err_t err = ec600n_http_post(dev, backend_url, "application/json", json_body,
                                        http_resp, sizeof(http_resp), 10000);
    if (err != EC600N_OK) {
        ec600n_log("[IRRIGATION] HTTP POST status failed (%d)\r\n", (int)err);
        return IRRIGATION_ERR_HTTP;
    }

    return IRRIGATION_OK;
}
