/**
  ******************************************************************************
  * @file    irrigation_ctrl.h
  * @brief   Irrigation Actuator Controller and REST API Handler for STM32.
  *          Controls:
  *            - PB4: Motor Water Pump
  *            - PB5: Valve 1
  *            - PB8: Valve 2
  *            - PB9: Valve 3
  *          Includes Safety Interlock: Pump is only allowed to run when at
  *          least 1 valve (1, 2, or 3) is open.
  ******************************************************************************
  */

#ifndef __IRRIGATION_CTRL_H__
#define __IRRIGATION_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ec600n.h"
#include <stdbool.h>
#include <stdint.h>

/* Actuator state structure */
typedef struct {
    bool pump;      /* PB4: Motor water pump */
    bool valve1;    /* PB5: Valve 1 */
    bool valve2;    /* PB8: Valve 2 */
    bool valve3;    /* PB9: Valve 3 */
} irrigation_state_t;

/* Status and error codes */
typedef enum {
    IRRIGATION_OK                     =  0,
    IRRIGATION_WARN_INTERLOCK_TRIPPED =  1, /* Pump denied/stopped: 0 valves are open */
    IRRIGATION_ERR_PARAM              = -1,
    IRRIGATION_ERR_PARSE              = -2,
    IRRIGATION_ERR_HTTP               = -3
} irrigation_status_t;

/* Controller Lifecycle */
void irrigation_init(void);

/* Hardware Actuator Control */
irrigation_status_t irrigation_set_state(bool pump, bool v1, bool v2, bool v3);
irrigation_status_t irrigation_apply_state(const irrigation_state_t *target, irrigation_state_t *actual);
void                irrigation_get_state(irrigation_state_t *state);
void                irrigation_emergency_stop(void);

/* Safety Interlock Checkers */
bool    irrigation_is_pump_allowed(bool v1, bool v2, bool v3);
uint8_t irrigation_get_open_valves_count(bool v1, bool v2, bool v3);

/* REST API / JSON Parser & Processor */
irrigation_status_t irrigation_process_rest_payload(const char *payload,
                                                    char *json_resp_out,
                                                    uint16_t max_resp_len);

/* Cloud / Backend REST API Integration via EC600N 4G */
irrigation_status_t irrigation_poll_backend_api(ec600n_device_t *dev,
                                                const char *backend_url,
                                                char *resp_buf,
                                                uint16_t resp_max_len);

irrigation_status_t irrigation_report_status_to_backend(ec600n_device_t *dev,
                                                        const char *backend_url,
                                                        const char *device_id);

#ifdef __cplusplus
}
#endif

#endif /* __IRRIGATION_CTRL_H__ */
